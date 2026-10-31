/*****************************************************************************
*	Name: SimJoint.cpp
*	Description: A joint of the simulated arm. See SimJoint.h.
*****************************************************************************/
#include "SimJoint.h"

BOOL
CSimJoint::MoveTorque(double adTor)
{
	/* as on the drive: only in a torque mode */
	if (m_nDriveMode != 10 && m_nDriveMode != 4)
		return FALSE;
	m_dCmdTorque = adTor;
	return TRUE;
}

BOOL
CSimJoint::ServoOn()
{
	m_nDriveMode = 10;
	/* the rising edge of the request resets a fault, if the power is there;
	 * without power nothing is enabled, and the request waits */
	const BOOL bEdge = m_bRequest ? FALSE : TRUE;
	m_bRequest = TRUE;
	if (m_bPowerCut)
		return TRUE;
	if (m_bFault)
	{
		if (!bEdge)
			return TRUE;
		m_bFault = FALSE;
	}
	m_bServoOn = TRUE;
	return TRUE;
}

BOOL
CSimJoint::ServoOff()
{
	/* the last target torque stays, as in the process image of the bus */
	m_bRequest = FALSE;
	m_bServoOn = FALSE;
	return TRUE;
}

void
CSimJoint::SetPowerCut(BOOL abCut, int anReport)
{
	if (abCut)
	{
		m_bPowerCut = TRUE;
		m_nReport = anReport;
		m_bServoOn = FALSE;			// out of Operation enabled, the brake holds
		if (anReport == eReportFault)
			m_bFault = TRUE;		// with the request on, the reset edge comes now and is lost
		return;
	}
	m_bPowerCut = FALSE;
	if (!m_bFault && m_bRequest)
		m_bServoOn = TRUE;			// still requested: enabled by itself
}
