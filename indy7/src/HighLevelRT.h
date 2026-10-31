/*****************************************************************************
*	Name: HighLevelRT.h
*	Description: The high-level controller (paper, Sec. IV-C, eq. (9) and
*	Table 9). It turns the latest goal, a position of the tool, into a joint
*	reference that meets the reference contract, in four steps per cycle:
*	  1. goal admission: a goal replaces the accepted one only if it differs
*	     from it by more than delta, and then restarts the IK;
*	  2. inverse kinematics: damped least-squares steps with the 3x6 position
*	     Jacobian, from the ready posture q_r for every accepted goal, at most
*	     N_it per cycle and N_max per goal, every iterate clipped to the joint
*	     limits minus a margin, on a model instance of its own;
*	  3. trajectory generation: Ruckig, position interface, from the current
*	     state of the reference to the IK solution, within the limits on
*	     velocity, acceleration, and jerk; a new target changes the plan, not
*	     the state;
*	  4. lead bound: a joint whose reference leads the measurement by ell gets
*	     the measured state (q_i, qd_i, 0) in the generator, which replans.
*	The arm is at rest when the generator has reached its target and |qd| has
*	stayed <= 0.05 rad/s for 200 ms. TRACK follows goals; HOME moves to q_r
*	(or to a target of the operator, StartHomeTo).
*	Everything is allocated in Init(); Update() does not allocate (checked by
*	the malloc-count build) and takes a bounded time.
*	A template on the number of joints N of the IK model (the arm: N = 6,
*	CHighLevelRT; a hand of the humanoid: N = 7, paper, Sec. VI), with the
*	tool body as a parameter (Params::acTool, "tcp" for the arm).
*****************************************************************************/
#ifndef G2T_HIGH_LEVEL_RT_H
#define G2T_HIGH_LEVEL_RT_H

#include <rbdl/rbdl.h>
#include <ruckig/ruckig.hpp>
#include <Eigen/Dense>

#include "base/Common.h"

template <int N>
class CHighLevelRTN
{
public:
	enum { DOF = N };

	struct Params {
		double	dDelta;					// goal threshold [m]
		double	dLambda;				// DLS damping [m]
		double	dStepMax;				// step bound Delta_max [rad]
		double	dTol;					// IK tolerance e_tol [m]
		int		nIterCycle;				// N_it, iterations per cycle
		int		nIterMax;				// N_max, iterations per goal
		double	dVelMax, dAccMax, dJerkMax;		// joint limits of the generator
		double	dMargin;				// position-limit margin [rad]
		double	dLead;					// lead bound ell [rad]
		double	dRestQd;				// at rest: |qd| [rad/s] ...
		int		nRestCyc;				// ... for this many cycles
		double	adQr[DOF];				// ready posture [rad]
		double	adLimLo[DOF], adLimHi[DOF];		// joint limits [rad]
		bool	bStepRef;				// baseline, simulation only: the IK solution as a step,
										// without Step 3 (generator) and Step 4 (lead bound)
		char	acTool[48];				// body of the tool point in the URDF ("tcp")
	};
	static Params DefaultParams();

	/* what the last Update() did, and the state */
	struct Status {
		bool	bGoalAccepted;			// a goal replaced the accepted one (this cycle)
		bool	bIkConverged;			// the IK converged: new target (this cycle)
		bool	bIkRejected;			// the IK gave up after N_max (this cycle)
		bool	bIkActive;
		int		nIkIter;				// iterations for the current goal
		double	dIkErr;					// |e| at the last iterate [m]
		int		nOtg;					// ruckig::Result of this cycle
		bool	bOtgNewCalc;			// the generator planned anew (this cycle)
		double	dOtgDuration;			// duration of the current plan [s]
		double	dOtgCalcUs;				// time of its calculation [us]
		unsigned uLeadReset;			// bit i: the lead bound reset joint i (this cycle)
		bool	bAtRest;
		bool	bHasGoal;				// a goal has been accepted
		double	adGoal[3];				// accepted goal [m]
		double	adTarget[DOF];			// target of the generator [rad]
		long	nAccepted, nConverged, nRejected, nLeadResets;	// since Start()
		double	dUs;					// duration of Update() [us]
	};

	CHighLevelRTN();

	/* load the model of the IK from the URDF; Ts: control period [s] */
	BOOL	Init			(const char* astrURDF, double adTs, const Params& astParams);
	BOOL	IsInit			(	) const { return m_bInit; }
	void	SetDelta		(double adDelta) { m_p.dDelta = adDelta; }
	void	SetReady		(const double* adQr);
	const Params& GetParams	(	) const { return m_p; }

	/* every motion starts its reference from the measured state (q, qd, 0):
	 * TRACK holds the measured posture until a goal is accepted; HOME moves
	 * to q_r */
	void	StartTrack		(const double* adQ, const double* adQd);
	void	StartHome		(const double* adQ, const double* adQd);
	/* HOME to adTarget instead of q_r (the joint move of the operator) */
	void	StartHomeTo		(const double* adQ, const double* adQd, const double* adTarget);

	/* one cycle: adGoal = latest goal [m] or NULL, measured state -> reference.
	 * FALSE if the generator returned an error (the reference is then the
	 * last one, at rest is false). */
	BOOL	Update			(const double* adGoal, const double* adQ, const double* adQd,
	                         double* adQRef, double* adQdRef, double* adQddRef);
	const Status& GetStatus	(	) const { return m_st; }

	/* position of the tool [m] at adQ, on the model of the IK (RT-safe; the
	 * IK updates the kinematics at its own iterate before it uses them) */
	void	ToolPosition	(const double* adQ, double* adP);

	/* Step 2 alone, as Update() runs it, from q_r until it converges or
	 * gives up after N_max iterations. Not for the Trajectory task: the goals
	 * of high-level tests 1 and 2 are drawn with it before the tasks start,
	 * and the keyboard task checks the reach of a task move ('t') with it,
	 * each on an instance of its own. TRUE: converged; adQ (may be NULL): the
	 * solution, *pnIter: iterations, *pdErr: |e| at the last iterate [m] */
	BOOL	SolveIk			(const double* adGoal, double* adQ, int* pnIter, double* pdErr);

private:
	void	StartFrom		(const double* adQ, const double* adQd);
	void	SetTarget		(const RigidBodyDynamics::Math::VectorNd& aq);

	bool	m_bInit;
	bool	m_bHome;
	Params	m_p;
	double	m_dTs;
	RigidBodyDynamics::Model		m_model;		// the IK's own instance
	unsigned int					m_uTcp;
	RigidBodyDynamics::Math::Vector3d	m_vTcpPoint;
	RigidBodyDynamics::Math::VectorNd	m_vQk, m_vZero, m_vQ;
	RigidBodyDynamics::Math::MatrixNd	m_mG;		// 3 x n, filled by RBDL
	Eigen::Matrix<double, 3, DOF>		m_mJp;
	Eigen::LDLT<Eigen::Matrix3d>		m_ldlt;
	ruckig::Ruckig<DOF>					m_otg;
	ruckig::InputParameter<DOF>			m_in;
	ruckig::OutputParameter<DOF>		m_out;
	int		m_nRest;
	Status	m_st;
};

/* the arm (paper, Sec. IV-C); CHighLevelRTN<7>: a hand of the humanoid (Sec. VI) */
typedef CHighLevelRTN<6> CHighLevelRT;

#endif // G2T_HIGH_LEVEL_RT_H
