/*****************************************************************************
*	Name: Indy7Joint.h
*	Description: A joint drive of the Indy7 (CiA 402, cyclic synchronous
*	torque) with the PDOs of Table 5 and the conversions of eq. (2):
*	    q_i = s_i 2 pi (c_i - c_0,i) / (C_i N_i)
*	    u_i = trunc(s_i kappa_i tau_i / (N_i K_t,i))
*	(s_i = +1 with MOVE_CCW=1, -1 with 0; C_i ENCODER_RESOLUTION; N_i
*	GEAR_RATIO; c_0,i HOME_POSITION_OFFSET; kappa_i CURRENT_RATIO; K_t,i
*	TORQUE_CONSTANT; TRANSMISSION_RATIO divides like N_i, 1 on the Indy7).
*
*	The state machine of the drive (CiA 402): while ServoOn() is requested
*	and the drive is not in Operation enabled, every period writes the next
*	command of the enable sequence (Switch on disabled: Shutdown; Ready to
*	switch on: Switch on; Switched on: the target position = the actual
*	one, target velocity and torque 0, Enable operation; Fault: Fault reset;
*	Quick stop active: Disable voltage). Without the request every period
*	writes Shutdown. The target torque follows MoveTorque() only while the
*	drive reports a torque mode (0x6061) and the joint is ready (its
*	statusword has been read in a cyclic mode, Running or Idle).
*	EtherCAT task only.
*****************************************************************************/
#ifndef G2T_INDY7_JOINT_H
#define G2T_INDY7_JOINT_H

#include "EcatMaster.h"
#include "Joint.h"

class CIndy7Joint : public CEcatSlave, public CJoint
{
public:
	/* the CiA 402 objects */
	enum {
		CW_SHUTDOWN = 0x0006, CW_SWITCH_ON = 0x0007, CW_ENABLE_OPERATION = 0x000F, CW_DISABLE_VOLTAGE = 0x0000,
		CW_QUICK_STOP = 0x0002, CW_FAULT_RESET = 0x0080, CW_HALT = 0x0100,
		SW_NOT_READY = 0x0000, SW_SWITCH_ON_DISABLED = 0x0040, SW_READY_TO_SWITCH_ON = 0x0021,
		SW_SWITCHED_ON = 0x0023, SW_OPERATION_ENABLED = 0x0027, SW_QUICK_STOP_ACTIVE = 0x0007,
		SW_FAULT_REACTION_ACTIVE = 0x000F, SW_FAULT = 0x0008, SW_UNKNOWN = 0xFFFF,
		MODE_PROFILE_TORQUE = 4, MODE_CYCLIC_POSITION = 8, MODE_CYCLIC_VELOCITY = 9, MODE_CYCLIC_TORQUE = 10
	};

	CIndy7Joint(const ST_CONFIG_ECAT_SLAVE& astCfg);

	/* CEcatSlave */
	virtual BOOL	Configure		(ec_master_t* apMaster, ec_domain_t* apIn, ec_domain_t* apOut);
	virtual BOOL	IsDrive			(	) const { return TRUE; }
	virtual void	ReadFromSlave	(	);
	virtual void	WriteToSlave	(	);
	virtual void	OnRelease		(	);

	/* CJoint */
	virtual double	GetCurrentPos	(	) const { return m_dPos; }
	virtual double	GetCurrentVel	(	) const { return m_dVel; }
	virtual double	GetCurrentTor	(	) const { return m_dTor; }
	virtual BOOL	IsServoOn		(	) const { return m_bServoOn; }
	virtual UINT16	GetStatusWord	(	) const { return m_usRawStatus; }
	virtual BOOL	ServoOn			(	);
	virtual BOOL	ServoOff		(	);
	virtual BOOL	StopAxis		(	);
	virtual BOOL	MoveTorque		(double adTor);
	virtual INT32	GetActualTorRaw	(	) const { return (INT32)m_sActualTor; }
	virtual INT32	GetTargetTorRaw	(	) const { return (INT32)m_sTargetTor; }
	virtual INT32	GetCurrentRawPos(	) const { return m_nActualPos; }

	/* the decoded state of the statusword (SW_*) */
	static UINT16	DecodeStatus	(UINT16 ausRaw);

private:
	/* the state of the drive as the enable sequence and the analysis of the
	 * statusword set it, and the readiness of the joint that follows from it */
	enum eServo { eServoOff = 0, eServoIdle, eServoRunning, eServoFault, eServoDisconnected };
	enum eReady { eReadyInit = 0, eReadyYes, eReadyFault, eReadyDisconnected };
	void	SetServoStatus	(eServo aeStatus);
	BOOL	IsMovable		(	) const { return (m_eReady == eReadyYes) ? TRUE : FALSE; }
	void	UpdateSi		(	);

	/* configuration */
	int		m_nDir;
	double	m_dRes, m_dGear, m_dTrans, m_dKt, m_dKappa;
	INT32	m_nHome;

	/* process data: offsets in the domains */
	int		m_nOffCtrl, m_nOffTpos, m_nOffTvel, m_nOffTtor, m_nOffMode;
	int		m_nOffStatus, m_nOffPos, m_nOffVel, m_nOffTor, m_nOffModeDisp;
	/* outputs */
	UINT16	m_usControl;
	INT32	m_nTargetPos, m_nTargetVel;
	INT16	m_sTargetTor;
	INT8	m_cMode;
	/* inputs */
	UINT16	m_usRawStatus, m_usStatus;
	INT32	m_nActualPos, m_nActualVel;
	INT16	m_sActualTor;
	INT8	m_cModeDisplay;

	BOOL	m_bServoReq;		// ServoOn() .. ServoOff()
	BOOL	m_bServoOn;			// Operation enabled in the last read
	BOOL	m_bFaultTold;
	eServo	m_eServo;
	eReady	m_eReady;
	double	m_dPos, m_dVel, m_dTor;

	ec_pdo_entry_info_t	m_astEntries[10];
	ec_pdo_info_t		m_astPdos[2];
	ec_sync_info_t		m_astSyncs[5];
};

#endif // G2T_INDY7_JOINT_H
