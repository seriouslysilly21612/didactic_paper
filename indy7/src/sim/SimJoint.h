/*****************************************************************************
*	Name: SimJoint.h
*	Description: A joint of the simulated arm (MuJoCo, CSimArm) with the
*	interface of a joint drive, so that the control tasks run unchanged in
*	simulation (-s) and with the bus beside the simulated arm (-b). It holds
*	the joint state in SI units, written by the simulation, and the target
*	torque in N m, read by it. As on a drive in cyclic synchronous torque
*	mode, the target torque acts only in Operation enabled and stays until
*	the next one; without the servo on, the brake of the joint holds it.
*
*	The models of the protective tests (SIM_FAULT_WHAT): a drive dropout
*	(the joint leaves Operation enabled by itself, SimDropout) and the
*	E-stop of the control box (SetPowerCut): while the power is cut a drive
*	cannot be enabled and reports Switch on disabled, Ready to switch on, or
*	Fault (SIM_ESTOP_REPORTS); a drive still requested on is enabled when
*	the power is back, as the enable sequence of the master does on the
*	robot; a Fault is reset only by a new servo-on request with the power
*	there. EtherCAT task only.
*****************************************************************************/
#ifndef G2T_SIM_JOINT_H
#define G2T_SIM_JOINT_H

#include "../ecat/Joint.h"

class CSimJoint : public CJoint
{
public:
	enum eEstopReport { eReportSod = 0, eReportReady = 1, eReportFault = 2 };

	CSimJoint(const TSTRING& astrName, int anDriveMode)
		: m_bServoOn(FALSE), m_bRequest(FALSE), m_bPowerCut(FALSE), m_bFault(FALSE), m_nReport(eReportSod),
		  m_dCmdTorque(0.0), m_nDriveMode(anDriveMode), m_dPos(0.0), m_dVel(0.0), m_dTor(0.0)
	{
		m_strName = astrName;
	}

	/* CJoint */
	virtual double	GetCurrentPos	(	) const { return m_dPos; }
	virtual double	GetCurrentVel	(	) const { return m_dVel; }
	virtual double	GetCurrentTor	(	) const { return m_dTor; }
	virtual BOOL	IsServoOn		(	) const { return m_bServoOn; }
	/* Operation enabled 0x0037, Ready to switch on 0x0031 (the master writes
	 * Shutdown), during an E-stop Switch on disabled 0x0040, Ready to switch
	 * on without voltage 0x0021, or Fault 0x0008 (0x0018 with the power back) */
	virtual UINT16	GetStatusWord	(	) const
	{
		if (m_bServoOn)
			return 0x0037;
		if (m_bFault)
			return m_bPowerCut ? 0x0008 : 0x0018;
		if (m_bPowerCut)
			return (m_nReport == eReportReady) ? 0x0021 : 0x0040;
		return 0x0031;
	}
	virtual BOOL	ServoOn			(	);
	virtual BOOL	ServoOff		(	);
	virtual BOOL	StopAxis		(	) { return TRUE; }	// the halt bit has no effect in CST
	virtual BOOL	MoveTorque		(double adTor);

	/* the simulation side */
	double	GetCommandTorque	(	) const { return m_bServoOn ? m_dCmdTorque : 0.0; }
	BOOL	IsBrakeReleased		(	) const { return m_bServoOn; }
	void	SetSimState			(double adPos, double adVel, double adTor) { m_dPos = adPos; m_dVel = adVel; m_dTor = adTor; }
	void	SimDropout			(	) { m_bServoOn = FALSE; }
	void	SetPowerCut			(BOOL abCut, int anReport);

private:
	BOOL	m_bServoOn;			// Operation enabled
	BOOL	m_bRequest;			// servo on requested (ServoOn .. ServoOff)
	BOOL	m_bPowerCut;
	BOOL	m_bFault;
	int		m_nReport;
	double	m_dCmdTorque;
	int		m_nDriveMode;
	double	m_dPos, m_dVel, m_dTor;
};

#endif // G2T_SIM_JOINT_H
