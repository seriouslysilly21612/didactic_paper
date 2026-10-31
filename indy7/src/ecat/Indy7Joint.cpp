/*****************************************************************************
*	Name: Indy7Joint.cpp
*	Description: A joint drive of the Indy7. See Indy7Joint.h.
*****************************************************************************/
#include "Indy7Joint.h"

namespace {
const double TWO_PI = 6.283185307179586;
}

CIndy7Joint::CIndy7Joint(const ST_CONFIG_ECAT_SLAVE& astCfg)
	: m_nOffCtrl(-1), m_nOffTpos(-1), m_nOffTvel(-1), m_nOffTtor(-1), m_nOffMode(-1),
	  m_nOffStatus(-1), m_nOffPos(-1), m_nOffVel(-1), m_nOffTor(-1), m_nOffModeDisp(-1),
	  m_usControl(0), m_nTargetPos(0), m_nTargetVel(0), m_sTargetTor(0), m_cMode(MODE_CYCLIC_TORQUE),
	  m_usRawStatus(0), m_usStatus(SW_NOT_READY), m_nActualPos(0), m_nActualVel(0), m_sActualTor(0),
	  m_cModeDisplay(0), m_bServoReq(FALSE), m_bServoOn(FALSE), m_bFaultTold(FALSE),
	  m_eServo(eServoDisconnected), m_eReady(eReadyInit), m_dPos(0.0), m_dVel(0.0), m_dTor(0.0)
{
	m_strName = astCfg.strName;
	m_strSlaveName = astCfg.strName;
	m_uVendor = astCfg.uVendorID;
	m_uProduct = astCfg.uProductCode;
	m_nDir = astCfg.bCCW ? 1 : -1;
	m_dRes = astCfg.dEncResolution;
	m_dGear = astCfg.dGearRatio;
	m_dTrans = astCfg.dTransRatio;
	m_dKt = astCfg.dTorqueConstant;
	m_dKappa = astCfg.dCurrentRatio;
	m_nHome = astCfg.nHomePositionOffset;

	/* Table 5: RxPDO 0x1600, TxPDO 0x1A00 */
	const ec_pdo_entry_info_t astE[10] = {
		{0x6040, 0x00, 16},		// controlword
		{0x607A, 0x00, 32},		// target position
		{0x60FF, 0x00, 32},		// target velocity
		{0x6071, 0x00, 16},		// target torque
		{0x6060, 0x00,  8},		// mode of operation
		{0x6041, 0x00, 16},		// statusword
		{0x6064, 0x00, 32},		// position actual value
		{0x606C, 0x00, 32},		// velocity actual value
		{0x6077, 0x00, 16},		// torque actual value
		{0x6061, 0x00,  8},		// mode of operation display
	};
	memcpy(m_astEntries, astE, sizeof(m_astEntries));
	m_astPdos[0].index = 0x1600;
	m_astPdos[0].n_entries = 5;
	m_astPdos[0].entries = m_astEntries + 0;
	m_astPdos[1].index = 0x1A00;
	m_astPdos[1].n_entries = 5;
	m_astPdos[1].entries = m_astEntries + 5;
	const ec_sync_info_t astS[5] = {
		{0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
		{1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
		{2, EC_DIR_OUTPUT, 1, m_astPdos + 0, EC_WD_ENABLE},
		{3, EC_DIR_INPUT, 1, m_astPdos + 1, EC_WD_DISABLE},
		{0xff, EC_DIR_INVALID, 0, NULL, EC_WD_DEFAULT},
	};
	memcpy(m_astSyncs, astS, sizeof(m_astSyncs));
}

BOOL
CIndy7Joint::Configure(ec_master_t* apMaster, ec_domain_t* apIn, ec_domain_t* apOut)
{
	if (FALSE == ConfigureSlave(apMaster, m_astSyncs))
		return FALSE;
	m_nOffCtrl     = RegisterEntry(apOut, 0x6040, 0x00);
	m_nOffTpos     = RegisterEntry(apOut, 0x607A, 0x00);
	m_nOffTvel     = RegisterEntry(apOut, 0x60FF, 0x00);
	m_nOffTtor     = RegisterEntry(apOut, 0x6071, 0x00);
	m_nOffMode     = RegisterEntry(apOut, 0x6060, 0x00);
	m_nOffStatus   = RegisterEntry(apIn, 0x6041, 0x00);
	m_nOffPos      = RegisterEntry(apIn, 0x6064, 0x00);
	m_nOffVel      = RegisterEntry(apIn, 0x606C, 0x00);
	m_nOffTor      = RegisterEntry(apIn, 0x6077, 0x00);
	m_nOffModeDisp = RegisterEntry(apIn, 0x6061, 0x00);
	return (m_nOffCtrl >= 0 && m_nOffTpos >= 0 && m_nOffTvel >= 0 && m_nOffTtor >= 0 && m_nOffMode >= 0 &&
	        m_nOffStatus >= 0 && m_nOffPos >= 0 && m_nOffVel >= 0 && m_nOffTor >= 0 && m_nOffModeDisp >= 0)
	       ? TRUE : FALSE;
}

UINT16
CIndy7Joint::DecodeStatus(UINT16 ausRaw)
{
	if ((ausRaw & 0x4F) == 0x00)
		return SW_NOT_READY;
	if ((ausRaw & 0x4F) == 0x08)
		return SW_FAULT;
	if ((ausRaw & 0x4F) == 0x40)
		return SW_SWITCH_ON_DISABLED;
	if ((ausRaw & 0x6F) == 0x27)
		return SW_OPERATION_ENABLED;
	if ((ausRaw & 0x6F) == 0x23)
		return SW_SWITCHED_ON;
	if ((ausRaw & 0x6F) == 0x21)
		return SW_READY_TO_SWITCH_ON;
	if ((ausRaw & 0x6F) == 0x07)
		return SW_QUICK_STOP_ACTIVE;
	if ((ausRaw & 0x4F) == 0x0F)
		return SW_FAULT_REACTION_ACTIVE;
	return SW_UNKNOWN;
}

/* a change of the state of the drive; the readiness of the joint follows
 * (Shutdown leaves it as it is) */
void
CIndy7Joint::SetServoStatus(eServo aeStatus)
{
	if (aeStatus == m_eServo)
		return;
	m_eServo = aeStatus;
	switch (aeStatus)
	{
		case eServoIdle:
		case eServoRunning:
			m_eReady = eReadyYes;
			break;
		case eServoFault:
			m_eReady = eReadyFault;
			break;
		case eServoDisconnected:
			m_eReady = eReadyDisconnected;
			DBG_LOG_ERROR("(CIndy7Joint) [0:%u] %s: check the EtherCAT connection", (unsigned)m_usPosition,
			              m_strName.c_str());
			break;
		case eServoOff:
		default:
			break;
	}
}

void
CIndy7Joint::UpdateSi()
{
	const INT32 nRel = m_nActualPos - m_nHome;
	m_dPos = TWO_PI * ((double)nRel / m_dRes / m_dGear / m_dTrans) * m_nDir;
	m_dVel = TWO_PI * ((double)m_nActualVel / m_dRes / m_dGear / m_dTrans) * m_nDir;
	const double dCurrent = (double)m_sActualTor / m_dKappa;
	m_dTor = dCurrent * m_dGear * m_dKt * m_nDir;
}

void
CIndy7Joint::ReadFromSlave()
{
	m_usRawStatus = EC_READ_U16(m_pIn + m_nOffStatus);
	if (FALSE == IsOnline())
		SetServoStatus(eServoDisconnected);
	else if (m_cModeDisplay == MODE_CYCLIC_TORQUE || m_cModeDisplay == MODE_CYCLIC_POSITION ||
	         m_cModeDisplay == MODE_CYCLIC_VELOCITY)
		/* in a cyclic mode (the display of the last read): bit 12 set, the
		 * drive follows the targets */
		SetServoStatus((m_usRawStatus & 0x1000) ? eServoRunning : eServoIdle);

	m_usStatus = DecodeStatus(m_usRawStatus);
	m_bServoOn = (m_usStatus == SW_OPERATION_ENABLED) ? TRUE : FALSE;
	m_nActualPos = EC_READ_S32(m_pIn + m_nOffPos);
	m_nActualVel = EC_READ_S32(m_pIn + m_nOffVel);
	m_sActualTor = EC_READ_S16(m_pIn + m_nOffTor);
	m_cModeDisplay = EC_READ_S8(m_pIn + m_nOffModeDisp);

	if (m_usStatus == SW_FAULT)
	{
		if (!m_bFaultTold)
			DBG_LOG_ERROR("(CIndy7Joint) [0:%u] %s: FAULT (statusword 0x%04x)", (unsigned)m_usPosition,
			              m_strName.c_str(), (unsigned)m_usRawStatus);
		m_bFaultTold = TRUE;
	}
	else if (m_usStatus == SW_SWITCH_ON_DISABLED)
		m_bFaultTold = FALSE;

	UpdateSi();
}

void
CIndy7Joint::WriteToSlave()
{
	if (m_bServoReq && !m_bServoOn)
	{
		/* the enable sequence: the next command for the state of the drive */
		switch (m_usStatus)
		{
			case SW_SWITCH_ON_DISABLED:
				SetServoStatus(eServoOff);
				m_usControl = CW_SHUTDOWN;
				break;
			case SW_READY_TO_SWITCH_ON:
				m_usControl = CW_SWITCH_ON;
				break;
			case SW_SWITCHED_ON:
				m_nTargetPos = m_nActualPos;
				m_nTargetVel = 0;
				m_sTargetTor = 0;
				m_usControl = CW_ENABLE_OPERATION;
				break;
			case SW_FAULT:
				SetServoStatus(eServoFault);
				m_usControl = CW_FAULT_RESET;
				break;
			case SW_OPERATION_ENABLED:
				SetServoStatus(eServoIdle);
				break;
			case SW_QUICK_STOP_ACTIVE:
				m_usControl = CW_DISABLE_VOLTAGE;
				break;
			case SW_FAULT_REACTION_ACTIVE:		// automatic transitions
			case SW_NOT_READY:
			default:
				break;
		}
	}
	else if (!m_bServoReq)
	{
		m_usControl = CW_SHUTDOWN;
		SetServoStatus(eServoOff);
	}

	EC_WRITE_U16(m_pOut + m_nOffCtrl, m_usControl);
	EC_WRITE_S32(m_pOut + m_nOffTpos, m_nTargetPos);
	EC_WRITE_S32(m_pOut + m_nOffTvel, m_nTargetVel);
	EC_WRITE_S16(m_pOut + m_nOffTtor, m_sTargetTor);
	EC_WRITE_S8(m_pOut + m_nOffMode, m_cMode);
}

void
CIndy7Joint::OnRelease()
{
	m_bServoReq = FALSE;
	m_usControl = CW_QUICK_STOP;
}

BOOL
CIndy7Joint::ServoOn()
{
	m_cMode = MODE_CYCLIC_TORQUE;
	if (!m_bServoOn)
		m_bServoReq = TRUE;
	return TRUE;
}

BOOL
CIndy7Joint::ServoOff()
{
	m_bServoReq = FALSE;
	return TRUE;
}

BOOL
CIndy7Joint::StopAxis()
{
	if (FALSE == IsMovable())
		return FALSE;
	m_usControl |= CW_HALT;
	return TRUE;
}

BOOL
CIndy7Joint::MoveTorque(double adTor)
{
	if (FALSE == IsMovable())
		return FALSE;
	if (m_cModeDisplay != MODE_CYCLIC_TORQUE && m_cModeDisplay != MODE_PROFILE_TORQUE)
		return FALSE;
	/* eq. (2): the motor current tau / (N K_t), in drive units */
	const double dCurrent = (adTor / m_dGear / m_dKt);
	m_sTargetTor = (INT16)(dCurrent * m_dKappa * m_nDir);
	return TRUE;
}
