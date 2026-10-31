/*****************************************************************************
*	Name: Joint.h
*	Description: A joint of the arm as the control tasks see it: the joint
*	drive on the bus (CIndy7Joint) or a joint of the simulated arm
*	(CSimJoint). Called by the EtherCAT task only.
*****************************************************************************/
#ifndef G2T_JOINT_H
#define G2T_JOINT_H

#include <cmath>

#include "../base/Common.h"

class CJoint
{
public:
	virtual ~CJoint() {}

	/* the state of the joint [rad, rad/s, N m]: the measurement of the last
	 * receive */
	virtual double	GetCurrentPos	(	) const = 0;
	virtual double	GetCurrentVel	(	) const = 0;
	virtual double	GetCurrentTor	(	) const = 0;
	/* Operation enabled */
	virtual BOOL	IsServoOn		(	) const = 0;
	/* the CiA 402 statusword (0x6041) as received */
	virtual UINT16	GetStatusWord	(	) const = 0;
	BOOL			IsMoving		(	) const { return (std::fabs(GetCurrentVel()) > 0.0) ? TRUE : FALSE; }

	/* commands for the next send: the enable sequence in cyclic synchronous
	 * torque mode, the disable, the halt bit, and the target torque (eq. (2)) */
	virtual BOOL	ServoOn			(	) = 0;
	virtual BOOL	ServoOff		(	) = 0;
	virtual BOOL	StopAxis		(	) = 0;
	virtual BOOL	MoveTorque		(double adTor) = 0;

	/* drive units, for the logs (0 without a drive): torque actual value
	 * (0x6077), target torque (0x6071), position actual value (0x6064) */
	virtual INT32	GetActualTorRaw	(	) const { return 0; }
	virtual INT32	GetTargetTorRaw	(	) const { return 0; }
	virtual INT32	GetCurrentRawPos(	) const { return 0; }

	const TSTRING&	GetName			(	) const { return m_strName; }

protected:
	TSTRING	m_strName;
};

#endif // G2T_JOINT_H
