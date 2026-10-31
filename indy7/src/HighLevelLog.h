/*****************************************************************************
*	Name: HighLevelLog.h
*	Description: The log of the high-level tests (paper, Sec. V-B and V-C),
*	written by CTestLog (TestLog.h): one CSV file per test with one row per
*	cycle of TRACK, and a .json file with the parameters and the result.
*
*	CSV columns: t [s]; mode (0 TRACK, 1 HOME); flags (1 goal accepted, 2 IK
*	converged, 4 IK rejected, 8 IK active, 16 new plan, 32 at rest, 64 has a
*	goal); otg (ruckig::Result); lead_reset (bit mask); ik_iter, ik_err [m];
*	otg_dur (duration of the current plan) [s]; hl_us (time of the high-level
*	step); goal_age (cycles since the goal slot changed); goal_x..z (the goal
*	slot, empty when there is none), acc_x..z (accepted goal), tool_x..z
*	(tool position of the measurement) [m]; target_i (IK solution), q_ref_i,
*	qd_ref_i, qdd_ref_i, q_i and qd_i (the measurement the controller used),
*	tau_i (commanded, after the torque limit); clamp.
*	Written by the Trajectory task. tau_i and clamp are the command of the
*	Dynamics task of the previous period (Torque slot), which
*	acted on the previous reference; the other columns are of this period.
*****************************************************************************/
#ifndef G2T_HIGH_LEVEL_LOG_H
#define G2T_HIGH_LEVEL_LOG_H

#include "TrajectoryRT.h"       // the sample of a TRACK/HOME cycle (Trajectory task)
#include "TestLog.h"

struct HighLevelFormat
{
	typedef CTrajectoryRT::Sample Sample;
	enum { MAX_DOF = CTrajectoryRT::MAX_DOF };

	struct Meta {
		char	acTest[12];				// "fixed", "jump", "noise", "noise0", "refcheck"
		int		nRun;
		int		nDof;
		double	dPeriod;
		CHighLevelRT::Params stP;		// the parameters of the high-level controller (delta of this test)
		double	adP0[3];				// tool position at q_r, the origin of the goals [m]
		double	adGoalOffset[3];		// goal of the test relative to p0 [m]
		int		nGoal, nGoals;			// test 1 with random goals: index of this goal, number of goals (-1, 0: fixed goal)
		double	adJump[3];				// jump of the goal (test 2) [m]
		double	dTrigger;				// distance of the tool to the goal at the jump [m]
		double	dNoise, dNoiseHz;		// synthetic goal noise [m], rate [Hz]
		char	acNoiseFile[64];		// recorded noise replayed instead ("": synthetic)
		double	dDuration;				// fixed duration (noise, refcheck) [s], 0: until at rest
		BOOL	bFrictionFF;
		double	adKp[MAX_DOF], adKd[MAX_DOF], adKf[MAX_DOF], adTauMax[MAX_DOF];
		BOOL	bSim, bSimFriction;
		int		nSimStateDelay, nSimTorqueDelay;
		double	adSimTauS[MAX_DOF];
	};
	struct Result {
		long	nTimingFaults;			// robot: nLate + nOverruns + nMissed; simulation: 0
		long	nLate, nOverruns, nMissed, nWkc;	// during the test (Indy7Ctrl.h, RtStats)
		long	nAllocs;				// malloc-count build (-1: not counted)
		long	nCtrlFaults;			// controller faults during the test (Indy7Ctrl.h)
		long	nDriveDropouts;			// drive dropouts during the test (Indy7Ctrl.h)
		long	nAccepted, nConverged, nRejected, nLeadResets;
		double	dRestTime;				// first time at rest after the last goal event [s], -1: never
	};

	void	Name	(const Meta& astM, const char* astrStamp, char* acOut, size_t auLen);
	void	Header	(FILE* fp, const Meta& astM);
	void	Begin	(	);
	void	Row		(FILE* fp, const Meta& astM, const Sample& astS);
	void	Finish	(FILE* apJson, const char* astrCsv, const Meta& astM, const Result& astR,
	                 const char* astrState, long anRows, unsigned anDrops);

	/* running statistics (writer thread) */
	double	dMaxHlUs, dMaxLead;
	double	adLastTool[3], adLastAcc[3];
	bool	bHasPrev;
	double	adPrevQdd[MAX_DOF], adPrevTau[MAX_DOF];
	double	dMaxDQdd, dMaxDTau;
};

typedef CTestLog<HighLevelFormat> CHighLevelLog;

#endif // G2T_HIGH_LEVEL_LOG_H
