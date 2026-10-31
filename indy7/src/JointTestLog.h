/*****************************************************************************
*	Name: JointTestLog.h
*	Description: The log of the JOINT TEST (paper, Sec. IV-A and V-A),
*	written by CTestLog (TestLog.h): one CSV file per test, one row per
*	control cycle, and a .json file with the parameters and the result of
*	the test.
*
*	CSV columns (joint i = 0..5): t [s], phase (0 to q1, 1 at q1, 2 to q0,
*	3 at q0), q_ref_i, qd_ref_i, qdd_ref_i, q_i and qd_i (the measurement the
*	controller used), e_i = q_ref_i - q_i, tau_i (commanded, after the torque
*	limit), and the terms of the torque before the limit: tau_g_i = g(q),
*	tau_c_i = C(q,qd) qd, tau_mqdd_i = M(q) qdd_ref, tau_fb_i = M(q)(Kp e +
*	Kd ed) or D(Kp e + Kd ed) for PD + g, tau_ff_i = friction feedforward;
*	clamp = bit mask of the joints whose torque was limited.
*	Then tau_act_i (the torque the drive reports in this
*	cycle, 0x6077, in N m; the simulator: the torque it applied),
*	tau_act_raw_i and tau_cmd_raw_i (0x6077 and the target 0x6071 written in
*	this cycle, drive units; 0 in the simulation). Test "step" (law "step"):
*	the torque-step delay test, tau_ff_i = the step of this cycle, phase 1
*	while it is on, 0 off, 3 after the last step. Test "brk" (law "ramp"): the
*	breakaway test, tau_ff_i = the ramp, phase 0 ramp, 1 damped to rest, 3 done.
*	Then mii_i = M_ii(q) of the URDF without the rotor
*	inertia (CTC laws; for the identification of B, tools/identify_rotor.py),
*	and mrow_i = row j of M(q) of the URDF in the sine test of joint j (0 else);
*	tau_mqdd_i is (M(q) + B) qdd_ref. Test "sine" (law "ctc" or "ctcff"): the
*	sine test, phase 0 fade in, 1 steady, 2 fade out, 3 done.
*****************************************************************************/
#ifndef G2T_JOINT_TEST_LOG_H
#define G2T_JOINT_TEST_LOG_H

#include "FullDynControllerRT.h"
#include "TestLog.h"

struct JointTestFormat
{
	typedef CControllerFullDynamicsRT::JointTestSample Sample;
	enum { MAX_DOF = CControllerFullDynamicsRT::JT_MAX_DOF };

	/* what the control task knows when the test starts */
	struct Meta {
		char	acTest[8];				// "fast", "slow", "step"
		char	acLaw[8];				// "ctc", "ctcff", "pdg", "step"
		int		nRun;					// run of this test and law in this session
		int		nDof;
		double	adDq[MAX_DOF];
		double	dT, dDwell, dPeriod;
		double	adKp[MAX_DOF], adKd[MAX_DOF], adKf[MAX_DOF], adTauMax[MAX_DOF];
		BOOL	bSim, bSimFriction;
		int		nSimStateDelay, nSimTorqueDelay;
		double	adSimTauS[MAX_DOF];
		/* the torque-step test (test "step") */
		int		nStepJoint;				// 1-based, 0: not a step test
		double	dStepTau, dStepOn, dStepOff;	// [N m], [s], [s]
		int		nStepRep;
		/* the breakaway test (test "brk") */
		int		nBaJoint;				// 1-based, 0: not a breakaway test
		double	dBaRate, dBaMax, dBaQd;
		int		nBaRep;
		BOOL	bFfGate;				// the gate of eq. (7) (FALSE: the ablation without it)
		/* the rotor inertias of the controller, and the sine test (test "sine") */
		double	adB[MAX_DOF];			// [kg m^2]
		int		nSineJoint;				// 1-based, 0: not a sine test
		double	dSineAmp, dSineFreq;	// [rad], [Hz]
		int		nSinePeriods;
		double	dSineV0;				// [rad/s] sweep, 0: none
	};
	/* and at its end */
	struct Result {
		double	adQ0[MAX_DOF], adD[MAX_DOF];
		long	nTimingFaults;			// robot: nLate + nOverruns + nMissed; simulation: 0 (see Finish)
		long	nLate, nOverruns, nMissed, nWkc;	// during the test (Indy7Ctrl.h, RtStats)
		long	nAllocs;				// malloc-count build: allocations of the control law (-1: not counted)
		long	nCtrlFaults;			// controller faults during the test (Indy7Ctrl.h)
		long	nDriveDropouts;			// drive dropouts during the test (Indy7Ctrl.h)
	};

	void	Name	(const Meta& astM, const char* astrStamp, char* acOut, size_t auLen);
	void	Header	(FILE* fp, const Meta& astM);
	void	Begin	(	);
	void	Row		(FILE* fp, const Meta& astM, const Sample& astS);
	void	Finish	(FILE* apJson, const char* astrCsv, const Meta& astM, const Result& astR,
	                 const char* astrState, long anRows, unsigned anDrops);

	/* running statistics (writer thread) */
	double	adMaxE[MAX_DOF], adMaxTau[MAX_DOF];
	long	nClamped;
};

typedef CTestLog<JointTestFormat> CJointTestLog;

#endif // G2T_JOINT_TEST_LOG_H
