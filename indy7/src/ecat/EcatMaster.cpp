/*****************************************************************************
*	Name: EcatMaster.cpp
*	Description: The EtherCAT master of the robot. See EcatMaster.h.
*****************************************************************************/
#include "EcatMaster.h"

namespace {
const uint16_t DC_ASSIGN_ACTIVATE = 0x0300;		// SYNC0
const uint16_t SYNC_MANAGER_PARAM = 0x1C32;		// output sync manager parameters
const uint8_t  SM_SYNC_TYPE = 0x01;				// synchronization type: 0 free run, 1 SM2 event, 2 DC SYNC0
const uint8_t  SM_SYNC_SUPPORTED = 0x04;		// synchronization types supported
}

/*************************************************************** slaves ***/

BOOL
CEcatSlave::ConfigureSlave(ec_master_t* apMaster, const ec_sync_info_t* apSyncs)
{
	m_pSc = ecrt_master_slave_config(apMaster, 0, m_usPosition, m_uVendor, m_uProduct);
	if (m_pSc == NULL)
	{
		DBG_LOG_ERROR("(CEcatSlave) [0:%u] %s: slave configuration 0x%08x:0x%08x failed", (unsigned)m_usPosition,
		              m_strSlaveName.c_str(), m_uVendor, m_uProduct);
		return FALSE;
	}
	if (0 != ecrt_slave_config_pdos(m_pSc, EC_END, apSyncs))
	{
		DBG_LOG_ERROR("(CEcatSlave) [0:%u] %s: PDO configuration failed", (unsigned)m_usPosition,
		              m_strSlaveName.c_str());
		return FALSE;
	}
	return TRUE;
}

int
CEcatSlave::RegisterEntry(ec_domain_t* apDomain, uint16_t ausIndex, uint8_t aucSub)
{
	unsigned int uBit = 0;
	const int nOff = ecrt_slave_config_reg_pdo_entry(m_pSc, ausIndex, aucSub, apDomain, &uBit);
	if (nOff < 0)
		DBG_LOG_ERROR("(CEcatSlave) [0:%u] %s: entry 0x%04x:%02x cannot be registered", (unsigned)m_usPosition,
		              m_strSlaveName.c_str(), (unsigned)ausIndex, (unsigned)aucSub);
	return nOff;
}

/*************************************************************** master ***/

CEcatMaster::CEcatMaster()
	: m_pMaster(NULL), m_pDomainIn(NULL), m_pDomainOut(NULL), m_pPdIn(NULL), m_pPdOut(NULL), m_bInit(FALSE),
	  m_uCycleNs(0), m_nSyncMode(eSyncFreeRun), m_nSync0Shift(0), m_uSyncDiff(0xFFFFFFFF)
{
	memset(&m_stMasterState, 0, sizeof(m_stMasterState));
	memset(&m_stDomainIn, 0, sizeof(m_stDomainIn));
	memset(&m_stDomainOut, 0, sizeof(m_stDomainOut));
}

CEcatMaster::~CEcatMaster()
{
	DeInit();
}

void
CEcatMaster::AddSlave(CEcatSlave* apSlave)
{
	apSlave->SetPosition((uint16_t)m_vSlaves.size());
	m_vSlaves.push_back(apSlave);
}

BOOL
CEcatMaster::Init(UINT32 auCycleNs, int anSyncMode, INT32 anSync0Shift)
{
	if (m_bInit)
		return TRUE;
	if (m_vSlaves.empty())
	{
		DBG_LOG_ERROR("(CEcatMaster) no slaves");
		return FALSE;
	}
	m_uCycleNs = auCycleNs;
	m_nSyncMode = anSyncMode;
	m_nSync0Shift = anSync0Shift;

	m_pMaster = ecrt_request_master(0);
	if (m_pMaster == NULL)
	{
		DBG_LOG_ERROR("(CEcatMaster) cannot request master 0 (is the EtherCAT master running? "
		              "systemctl status ethercat)");
		return FALSE;
	}
	m_pDomainIn = ecrt_master_create_domain(m_pMaster);
	m_pDomainOut = ecrt_master_create_domain(m_pMaster);
	if (m_pDomainIn == NULL || m_pDomainOut == NULL)
	{
		DBG_LOG_ERROR("(CEcatMaster) cannot create the domains");
		DeInit();
		return FALSE;
	}
	ecrt_master_state(m_pMaster, &m_stMasterState);

	/* the bus as the cfg describes it, and the synchronization it asks for */
	if (FALSE == CheckBus() || FALSE == CheckSyncSupport())
	{
		DeInit();
		return FALSE;
	}

	for (size_t n = 0; n < m_vSlaves.size(); n++)
	{
		CEcatSlave* pSl = m_vSlaves[n];
		if (FALSE == pSl->Configure(m_pMaster, m_pDomainIn, m_pDomainOut))
		{
			DeInit();
			return FALSE;
		}
		if (!pSl->IsDrive())
			continue;
		if (m_nSyncMode == eSyncSm && 0 != ecrt_slave_config_sdo16(pSl->GetConfig(), SYNC_MANAGER_PARAM,
		                                                           SM_SYNC_TYPE, 0x0001))
		{
			DBG_LOG_ERROR("(CEcatMaster) [0:%u] %s: cannot add the SDO 0x1C32:01 = 1", (unsigned)pSl->GetPosition(),
			              pSl->GetSlaveName().c_str());
			DeInit();
			return FALSE;
		}
		if (m_nSyncMode == eSyncDc && 0 != ecrt_slave_config_dc(pSl->GetConfig(), DC_ASSIGN_ACTIVATE, auCycleNs,
		                                                        anSync0Shift, 0, 0))
		{
			DBG_LOG_ERROR("(CEcatMaster) [0:%u] %s: cannot configure DC", (unsigned)pSl->GetPosition(),
			              pSl->GetSlaveName().c_str());
			DeInit();
			return FALSE;
		}
	}
	if (m_nSyncMode == eSyncDc)
	{
		/* the first joint drive is the reference clock */
		CEcatSlave* pRef = NULL;
		for (size_t n = 0; n < m_vSlaves.size() && pRef == NULL; n++)
			if (m_vSlaves[n]->IsDrive())
				pRef = m_vSlaves[n];
		if (pRef == NULL || 0 != ecrt_master_select_reference_clock(m_pMaster, pRef->GetConfig()))
		{
			DBG_LOG_ERROR("(CEcatMaster) cannot select the reference clock");
			DeInit();
			return FALSE;
		}
	}

	if (0 != ecrt_master_activate(m_pMaster))
	{
		DBG_LOG_ERROR("(CEcatMaster) cannot activate the master");
		DeInit();
		return FALSE;
	}
	m_pPdIn = ecrt_domain_data(m_pDomainIn);
	m_pPdOut = ecrt_domain_data(m_pDomainOut);
	if (m_pPdIn == NULL || m_pPdOut == NULL)
	{
		DBG_LOG_ERROR("(CEcatMaster) no process data of the domains");
		DeInit();
		return FALSE;
	}
	for (size_t n = 0; n < m_vSlaves.size(); n++)
		m_vSlaves[n]->SetDomainData(m_pPdIn, m_pPdOut);
	m_bInit = TRUE;

	if (m_nSyncMode == eSyncDc)
		DBG_LOG_INFO("(CEcatMaster) %u slaves, cycle %u ns, SYNC_MODE=DC: SYNC0 %d ns after the release, "
		             "reference clock: the first joint drive", GetSlaveCnt(), auCycleNs, (int)anSync0Shift);
	else
		DBG_LOG_INFO("(CEcatMaster) %u slaves, cycle %u ns, SYNC_MODE=%s%s", GetSlaveCnt(), auCycleNs,
		             CConfig::SyncModeName(m_nSyncMode),
		             m_nSyncMode == eSyncSm ? ": 0x1C32:01 = 1 written to the joint drives" :
		                                      ": no distributed clocks, nothing written to the drives");
	return TRUE;
}

/* the slaves on the bus against those of the cfg */
BOOL
CEcatMaster::CheckBus()
{
	ec_master_info_t stInfo;
	memset(&stInfo, 0, sizeof(stInfo));
	if (0 != ecrt_master(m_pMaster, &stInfo))
	{
		DBG_LOG_ERROR("(CEcatMaster) cannot read the master information");
		return FALSE;
	}
	if (!stInfo.link_up)
	{
		DBG_LOG_ERROR("(CEcatMaster) no Ethernet link (cable, power of the robot)");
		return FALSE;
	}
	if (stInfo.slave_count != m_vSlaves.size())
	{
		DBG_LOG_ERROR("(CEcatMaster) %u slaves on the bus, %u in the cfg (SLAVENUMBER)", stInfo.slave_count,
		              (unsigned)m_vSlaves.size());
		return FALSE;
	}
	BOOL bOk = TRUE;
	for (size_t n = 0; n < m_vSlaves.size(); n++)
	{
		ec_slave_info_t stSl;
		memset(&stSl, 0, sizeof(stSl));
		if (0 != ecrt_master_get_slave(m_pMaster, (uint16_t)n, &stSl))
		{
			DBG_LOG_ERROR("(CEcatMaster) cannot read slave %u", (unsigned)n);
			bOk = FALSE;
			continue;
		}
		const CEcatSlave* pSl = m_vSlaves[n];
		if (stSl.vendor_id != pSl->GetVendorId() || stSl.product_code != pSl->GetProductCode())
		{
			DBG_LOG_ERROR("(CEcatMaster) slave %u is 0x%08x:0x%08x (%s), the cfg has 0x%08x:0x%08x (%s)", (unsigned)n,
			              stSl.vendor_id, stSl.product_code, stSl.name, pSl->GetVendorId(), pSl->GetProductCode(),
			              pSl->GetSlaveName().c_str());
			bOk = FALSE;
		}
	}
	return bOk;
}

BOOL
CEcatMaster::SdoUpload16(uint16_t ausPos, uint16_t ausIndex, uint8_t aucSub, uint16_t* apusValue,
                         uint32_t* apuAbort)
{
	uint8_t aucBuf[4] = {0, 0, 0, 0};
	size_t uSize = 0;
	*apuAbort = 0;
	if (0 != ecrt_master_sdo_upload(m_pMaster, ausPos, ausIndex, aucSub, aucBuf, sizeof(aucBuf), &uSize, apuAbort) ||
	    uSize < 1)
		return FALSE;
	*apusValue = (uint16_t)(aucBuf[0] | (uSize > 1 ? (aucBuf[1] << 8) : 0));
	return TRUE;
}

/* SM or DC: 0x1C32:04 of every joint drive (bit 1: SM-synchronous, bits 2-3
 * = 01: DC SYNC0), read before the bus starts */
BOOL
CEcatMaster::CheckSyncSupport()
{
	if (m_nSyncMode == eSyncFreeRun)
		return TRUE;
	BOOL bOk = TRUE;
	for (size_t n = 0; n < m_vSlaves.size(); n++)
	{
		const CEcatSlave* pSl = m_vSlaves[n];
		if (!pSl->IsDrive())
			continue;
		uint16_t usTypes = 0;
		uint32_t uAbort = 0;
		if (FALSE == SdoUpload16((uint16_t)n, SYNC_MANAGER_PARAM, SM_SYNC_SUPPORTED, &usTypes, &uAbort))
		{
			fprintf(stderr, "[BUS] WARNING: slave %u (%s): 0x1C32:04 (synchronization types supported) cannot be "
			        "read (abort code 0x%08x) - SYNC_MODE=%s is not available; use SYNC_MODE=FREE_RUN\n",
			        (unsigned)n, pSl->GetSlaveName().c_str(), uAbort, CConfig::SyncModeName(m_nSyncMode));
			bOk = FALSE;
			continue;
		}
		const BOOL bSupported = (m_nSyncMode == eSyncSm) ? ((usTypes & 0x0002) != 0) : (((usTypes >> 2) & 0x3) == 0x1);
		printf("[BUS] slave %u (%s): 0x1C32:04 = 0x%04x (free run %s, SM %s, DC %s)\n", (unsigned)n,
		       pSl->GetSlaveName().c_str(), (unsigned)usTypes, (usTypes & 0x0001) ? "yes" : "no",
		       (usTypes & 0x0002) ? "yes" : "no", (((usTypes >> 2) & 0x3) == 0x1) ? "yes" : "no");
		if (!bSupported)
		{
			fprintf(stderr, "[BUS] WARNING: slave %u (%s) does not support SYNC_MODE=%s - the controller ends; "
			        "use SYNC_MODE=FREE_RUN\n", (unsigned)n, pSl->GetSlaveName().c_str(),
			        CConfig::SyncModeName(m_nSyncMode));
			bOk = FALSE;
		}
	}
	return bOk;
}

BOOL
CEcatMaster::CheckSyncInOp()
{
	if (m_nSyncMode == eSyncFreeRun)
		return TRUE;
	if (!m_bInit)
		return FALSE;
	const uint16_t usWant = (m_nSyncMode == eSyncSm) ? 1 : 2;
	BOOL bOk = TRUE;
	for (size_t n = 0; n < m_vSlaves.size(); n++)
	{
		const CEcatSlave* pSl = m_vSlaves[n];
		if (!pSl->IsDrive())
			continue;
		uint16_t usType = 0xFFFF;
		uint32_t uAbort = 0;
		if (FALSE == SdoUpload16((uint16_t)n, SYNC_MANAGER_PARAM, SM_SYNC_TYPE, &usType, &uAbort))
		{
			fprintf(stderr, "[BUS] WARNING: slave %u (%s): 0x1C32:01 cannot be read in OP (abort code 0x%08x)\n",
			        (unsigned)n, pSl->GetSlaveName().c_str(), uAbort);
			bOk = FALSE;
			continue;
		}
		if (usType != usWant)
		{
			fprintf(stderr, "[BUS] WARNING: slave %u (%s) runs with 0x1C32:01 = %u, SYNC_MODE=%s needs %u\n",
			        (unsigned)n, pSl->GetSlaveName().c_str(), (unsigned)usType, CConfig::SyncModeName(m_nSyncMode),
			        (unsigned)usWant);
			bOk = FALSE;
		}
	}
	if (bOk)
		printf("[BUS] SYNC_MODE=%s: every joint drive runs with 0x1C32:01 = %u\n", CConfig::SyncModeName(m_nSyncMode),
		       (unsigned)usWant);
	return bOk;
}

BOOL
CEcatMaster::IsAllSlavesOp() const
{
	for (size_t n = 0; n < m_vSlaves.size(); n++)
		if (FALSE == m_vSlaves[n]->IsOp())
			return FALSE;
	return TRUE;
}

/*********************************************************** the period ***/

BOOL
CEcatMaster::Receive()
{
	if (!m_bInit)
		return FALSE;
	ecrt_master_receive(m_pMaster);
	ecrt_domain_process(m_pDomainIn);
	ecrt_domain_process(m_pDomainOut);
	ecrt_domain_state(m_pDomainIn, &m_stDomainIn);
	ecrt_domain_state(m_pDomainOut, &m_stDomainOut);
	if (m_nSyncMode == eSyncDc)
		m_uSyncDiff = ecrt_master_sync_monitor_process(m_pMaster);

	/* an incomplete working counter: the inputs are not read */
	const BOOL bIn = (m_stDomainIn.wc_state == EC_WC_COMPLETE) ? TRUE : FALSE;
	if (bIn)
		for (size_t n = 0; n < m_vSlaves.size(); n++)
			m_vSlaves[n]->ReadFromSlave();
	return (bIn && m_stDomainOut.wc_state == EC_WC_COMPLETE) ? TRUE : FALSE;
}

void
CEcatMaster::Send(UINT64 aulReleaseNs)
{
	if (!m_bInit)
		return;
	if (m_stDomainOut.wc_state == EC_WC_COMPLETE)
		for (size_t n = 0; n < m_vSlaves.size(); n++)
			m_vSlaves[n]->WriteToSlave();
	/* the application time: the planned release, free of the wake-up jitter */
	ecrt_master_application_time(m_pMaster, aulReleaseNs);
	if (m_nSyncMode == eSyncDc)
	{
		ecrt_master_sync_reference_clock_to(m_pMaster, read_timer());
		ecrt_master_sync_slave_clocks(m_pMaster);
		ecrt_master_sync_monitor_queue(m_pMaster);
	}
	ecrt_domain_queue(m_pDomainIn);
	ecrt_domain_queue(m_pDomainOut);
	ecrt_master_send(m_pMaster);
}

void
CEcatMaster::UpdateStates()
{
	if (!m_bInit)
		return;
	ecrt_master_state(m_pMaster, &m_stMasterState);
	for (size_t n = 0; n < m_vSlaves.size(); n++)
		m_vSlaves[n]->UpdateConfigState();
}

/************************************************************* shutdown ***/

void
CEcatMaster::ReadSlaves()
{
	ecrt_master_receive(m_pMaster);
	ecrt_domain_process(m_pDomainIn);
	ecrt_master_state(m_pMaster, &m_stMasterState);
	ecrt_domain_state(m_pDomainIn, &m_stDomainIn);
	for (size_t n = 0; n < m_vSlaves.size(); n++)
	{
		m_vSlaves[n]->UpdateConfigState();
		if (m_stDomainIn.wc_state == EC_WC_COMPLETE)
			m_vSlaves[n]->ReadFromSlave();
	}
	ecrt_domain_queue(m_pDomainIn);
	ecrt_master_send(m_pMaster);
}

void
CEcatMaster::WriteSlaves(UINT64 aulAppTime)
{
	ecrt_master_receive(m_pMaster);
	ecrt_domain_process(m_pDomainOut);
	ecrt_domain_state(m_pDomainOut, &m_stDomainOut);
	for (size_t n = 0; n < m_vSlaves.size(); n++)
		if (m_stDomainOut.wc_state == EC_WC_COMPLETE)
			m_vSlaves[n]->WriteToSlave();
	ecrt_master_application_time(m_pMaster, aulAppTime);
	if (m_nSyncMode == eSyncDc)
	{
		ecrt_master_sync_reference_clock_to(m_pMaster, aulAppTime);
		ecrt_master_sync_slave_clocks(m_pMaster);
	}
	ecrt_domain_queue(m_pDomainOut);
	ecrt_master_send(m_pMaster);
}

void
CEcatMaster::DeInit()
{
	if (m_pMaster == NULL)
		return;
	if (m_bInit)
	{
		/* one last exchange with the commands of the shutdown (servo off) */
		ReadSlaves();
		for (size_t n = 0; n < m_vSlaves.size(); n++)
			m_vSlaves[n]->OnRelease();
		WriteSlaves(read_timer());
	}
	ecrt_release_master(m_pMaster);
	m_pMaster = NULL;
	m_pDomainIn = m_pDomainOut = NULL;
	m_pPdIn = m_pPdOut = NULL;
	m_bInit = FALSE;
	DBG_LOG_INFO("(CEcatMaster) master released");
}
