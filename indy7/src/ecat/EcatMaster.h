/*****************************************************************************
*	Name: EcatMaster.h
*	Description: The EtherCAT master of the robot on the IgH master (ecrt.h,
*	1.6), with two domains: the inputs and the outputs of every slave.
*
*	One exchange per period of the EtherCAT task (paper, Sec. III-B), in
*	this order:
*	  Receive()       the frame of the last period, both domains; the
*	                  inputs are read only if the working counter of the
*	                  input domain is complete. TRUE if both are complete.
*	  Send(release)   the outputs (only if the working counter of the output
*	                  domain was complete), the application time (the
*	                  planned release of the period), with SYNC_MODE=DC the
*	                  clock synchronization, one send of both domains.
*	  UpdateStates()  after the send: the AL states of the master and the
*	                  slaves, for the checks of the next period.
*	DeInit() makes one last exchange and releases the master, right after
*	the last period: the slaves then go to PREOP while the master itself
*	keeps the bus running (a slave left in OP without process data trips
*	its sync-manager watchdog).
*
*	Synchronization of the drives (SYNC_MODE of the cfg):
*	  FREE_RUN  the default, and the setting of the experiments of the
*	            paper: no distributed clocks, nothing is written to the
*	            drives; they run on their own cycle (0x1C32:01 = 0).
*	  SM        SM-synchronous: 0x1C32:01 = 1 is written to every joint
*	            drive at the configuration of the bus.
*	  DC        distributed clocks: SYNC0 every cycle, DC_SYNC0_SHIFT after
*	            the release (AssignActivate 0x0300) on every joint drive,
*	            joint drive 0 the reference clock, which follows the host
*	            clock (ecrt_master_sync_reference_clock_to) and the other
*	            slaves follow.
*	For SM and DC, CheckSyncSupport() reads 0x1C32:04 of every joint drive
*	before the bus starts (the drive must support the mode), and
*	CheckSyncInOp() reads 0x1C32:01 once the bus is OP (the drive must run
*	in it); the application ends if either fails.
*****************************************************************************/
#ifndef G2T_ECAT_MASTER_H
#define G2T_ECAT_MASTER_H

#include <vector>

#include "ecrt.h"
#include "../base/Common.h"
#include "../base/Config.h"

/* a slave of the bus */
class CEcatSlave
{
public:
	CEcatSlave() : m_uVendor(0), m_uProduct(0), m_usPosition(0), m_pSc(NULL), m_pIn(NULL), m_pOut(NULL)
	{
		memset(&m_stState, 0, sizeof(m_stState));
	}
	virtual ~CEcatSlave() {}

	/* before the activation: the slave configuration, its PDOs, and the
	 * registration of its entries in the two domains */
	virtual BOOL	Configure		(ec_master_t* apMaster, ec_domain_t* apIn, ec_domain_t* apOut) = 0;
	/* a joint drive (the checks and the settings of SYNC_MODE apply to it) */
	virtual BOOL	IsDrive			(	) const { return FALSE; }
	/* every period: the inputs into the slave object, its outputs into the
	 * process image */
	virtual void	ReadFromSlave	(	) = 0;
	virtual void	WriteToSlave	(	) = 0;
	/* before the last exchange: the commands of the shutdown */
	virtual void	OnRelease		(	) {}

	void	SetDomainData	(uint8_t* apIn, uint8_t* apOut) { m_pIn = apIn; m_pOut = apOut; }
	void	UpdateConfigState(	) { if (m_pSc != NULL) ecrt_slave_config_state(m_pSc, &m_stState); }
	BOOL	IsOp			(	) const { return (m_stState.al_state == 0x08) ? TRUE : FALSE; }
	BOOL	IsOnline		(	) const { return m_stState.online ? TRUE : FALSE; }
	unsigned int AlState	(	) const { return m_stState.al_state; }

	void	SetPosition		(uint16_t ausPos) { m_usPosition = ausPos; }
	uint16_t GetPosition	(	) const { return m_usPosition; }
	UINT32	GetVendorId		(	) const { return m_uVendor; }
	UINT32	GetProductCode	(	) const { return m_uProduct; }
	const TSTRING& GetSlaveName(	) const { return m_strSlaveName; }
	ec_slave_config_t* GetConfig(	) const { return m_pSc; }

protected:
	/* the slave configuration (alias 0, the position on the bus), its PDOs */
	BOOL	ConfigureSlave	(ec_master_t* apMaster, const ec_sync_info_t* apSyncs);
	/* one entry; its byte offset in the domain, -1 on failure */
	int		RegisterEntry	(ec_domain_t* apDomain, uint16_t ausIndex, uint8_t aucSub);

	TSTRING					m_strSlaveName;
	UINT32					m_uVendor, m_uProduct;
	uint16_t				m_usPosition;
	ec_slave_config_t*		m_pSc;
	ec_slave_config_state_t	m_stState;
	uint8_t*				m_pIn;		// process image of the input domain
	uint8_t*				m_pOut;		// ... of the output domain
};

class CEcatMaster
{
public:
	CEcatMaster();
	~CEcatMaster();

	/* the slaves in the order of the bus (position 0, 1, ...), before Init() */
	void	AddSlave		(CEcatSlave* apSlave);
	/* request the master, check the bus against the slaves (number, vendor,
	 * product), with SM or DC check that the drives support it, configure
	 * the slaves and the synchronization, activate */
	BOOL	Init			(UINT32 auCycleNs, int anSyncMode, INT32 anSync0Shift);
	void	DeInit			(	);

	/* the exchange of one period (see above) */
	BOOL	Receive			(	);
	void	Send			(UINT64 aulReleaseNs);
	void	UpdateStates	(	);

	BOOL	IsMasterOp		(	) const { return (m_stMasterState.al_states == 0x08) ? TRUE : FALSE; }
	BOOL	IsAllSlavesOp	(	) const;
	unsigned int GetSlaveCnt(	) const { return (unsigned int)m_vSlaves.size(); }
	int		GetSyncMode		(	) const { return m_nSyncMode; }
	BOOL	IsDc			(	) const { return (m_nSyncMode == eSyncDc) ? TRUE : FALSE; }
	/* SYNC_MODE=DC: upper estimate of the largest difference between the
	 * clocks of the slaves (0x092C) of the last receive [ns]; 0xFFFFFFFF if
	 * none */
	UINT32	GetSyncDiff		(	) const { return m_uSyncDiff; }

	/* SYNC_MODE=SM or DC, not real-time: once the bus is OP, 0x1C32:01 of
	 * every joint drive must be 1 (SM) or 2 (DC); FALSE with the message if
	 * not. TRUE for FREE_RUN. */
	BOOL	CheckSyncInOp	(	);

private:
	BOOL	CheckBus		(	);
	BOOL	CheckSyncSupport(	);
	BOOL	SdoUpload16		(uint16_t ausPos, uint16_t ausIndex, uint8_t aucSub, uint16_t* apusValue,
	                         uint32_t* apuAbort);
	void	ReadSlaves		(	);
	void	WriteSlaves		(UINT64 aulAppTime);

	std::vector<CEcatSlave*>	m_vSlaves;
	ec_master_t*		m_pMaster;
	ec_domain_t*		m_pDomainIn;
	ec_domain_t*		m_pDomainOut;
	uint8_t*			m_pPdIn;
	uint8_t*			m_pPdOut;
	ec_master_state_t	m_stMasterState;
	ec_domain_state_t	m_stDomainIn, m_stDomainOut;
	BOOL				m_bInit;
	UINT32				m_uCycleNs;
	int					m_nSyncMode;
	INT32				m_nSync0Shift;
	UINT32				m_uSyncDiff;
};

#endif // G2T_ECAT_MASTER_H
