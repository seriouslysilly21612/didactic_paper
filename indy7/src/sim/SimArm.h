/*****************************************************************************
*	Name: SimArm.h
*	Description: MuJoCo in place of the EtherCAT exchange (-s, -b; paper,
*	Sec. IV and V).
*
*	The EtherCAT task calls GetMeasured() where the frame is received and
*	Step() where it is sent, one step of one period per cycle, so that the
*	controller runs with the timing of the robot and the simulation keeps
*	the delay of the pipeline (send before compute, Fig. 2). Two delays
*	[steps] stand for the drives: the state that the task receives in
*	period k is the state at the end of step k - a (SIM_STATE_DELAY_STEPS),
*	and the torque that it sends in period k acts during step k + D - a
*	(SIM_TORQUE_DELAY_STEPS = D >= a), held over the step, as the drives
*	hold it.
*	Servo off engages a brake: the joint keeps its position. Dry friction
*	(frictionloss) can be switched on with the breakaway torques of the
*	robot (SIM_FRICTION_ON, SIM_TAU_S), and the rotor inertias act as the
*	armature of the joints (SIM_ARMATURE; default ROTOR_INERTIA_n). The
*	model sets the integrator (RK4, see tools/urdf2mjcf.py).
*	Everything is allocated in Init(); Step() neither allocates nor blocks.
*****************************************************************************/
#ifndef G2T_SIM_ARM_H
#define G2T_SIM_ARM_H

#include "../base/Common.h"

struct mjModel_;
struct mjData_;

class CSimArm
{
public:
	enum { MAX_DOF = 8, MAX_DELAY = 8 };

	CSimArm();
	virtual ~CSimArm();

	BOOL	Init			(const char* astrMJCF, int anDof, int anStateDelay, int anTorqueDelay,
	                         const double* adQ0, BOOL abFriction, const double* adTauS, double adPeriod);
	BOOL	IsInit			(	) const { return m_bInit; }
	/* the rotor inertias of the simulated joints [kg m^2] as the armature of
	 * MuJoCo (paper, Sec. IV-A); after Init */
	BOOL	SetArmature		(const double* adB, int anDof);

	/* the state the controller receives in the cycle after the last Step() */
	void	GetMeasured		(double* adQ, double* adQd, double* adTau) const;
	/* the simulated angles now, at the end of the last Step() (for a viewer) */
	void	GetCurrentQ		(double* adQ) const;

	/* one control cycle: adTau = torques of the controller [N m], abReleased =
	 * brake released (servo on) per joint; then GetMeasured() for the next cycle */
	void	Step			(const double* adTau, const BOOL* abReleased);

	void	PrintSummary	(	) const;

private:
	struct mjModel_*	m_pM;
	struct mjData_*		m_pD;
	BOOL	m_bInit;
	int		m_nDof;
	int		m_anQ[MAX_DOF];				// qpos address per joint
	int		m_anV[MAX_DOF];				// dof address per joint
	int		m_nStateDelay;				// a
	int		m_nTorqueDelay;				// D
	long	m_nStep;					// steps done
	double	m_adHistQ[MAX_DELAY + 1][MAX_DOF];	// state at the end of each step (ring)
	double	m_adHistQd[MAX_DELAY + 1][MAX_DOF];
	double	m_adHistTau[MAX_DELAY + 1][MAX_DOF];	// torque that acted during that step
	double	m_adTauQueue[MAX_DELAY + 1][MAX_DOF];	// D - a extra steps of torque delay (ring)
	BOOL	m_abBraked[MAX_DOF];
	double	m_adBrakeQ[MAX_DOF];
	/* statistics, printed by PrintSummary(): steps that integrated the arm,
	 * and steps with every brake engaged (no integration) */
	UINT64	m_uStepNsMax;
	UINT64	m_uStepNsSum;
	long	m_nStepsBraked;
	double	m_adQRelease[MAX_DOF];		// position at the last brake release
	double	m_dMaxDisp;					// largest |q - q_release| of a released joint since then
	BOOL	m_bFriction;
};

#endif // G2T_SIM_ARM_H
