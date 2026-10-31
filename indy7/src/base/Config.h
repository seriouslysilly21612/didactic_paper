/*****************************************************************************
*	Name: Config.h
*	Description: The configuration file of the controller (INDY7.cfg).
*
*	Format: "[SECTION]" lines and "KEY=VALUE" lines; ';' or '#' starts a
*	comment, also after a value. Keys are not case-sensitive and belong to
*	the section above them. A key that is not given keeps the default below;
*	an unknown key is reported. A relative path in the file (URDF_PATH,
*	SIM_MJCF_PATH, JOINT_TEST_LOG_DIR, CALIB_DIR, HL_TEST_NOISE_FILE) is
*	relative to the directory of the file.
*
*	Sections: [SYSTEM] (the controller, the tests, the simulation, the
*	application), [RT_TASKS] and [TASK0]..[TASK3] (the tasks), [ECAT_MASTER]
*	(the bus) and [AXIS0]..[AXIS6] (the slaves in the order of the bus: the
*	six joint drives, then the end tool). Lengths of the high-level
*	controller are given in mm in the file and kept in m here.
*****************************************************************************/
#ifndef G2T_CONFIG_H
#define G2T_CONFIG_H

#include <vector>

#include "Common.h"

typedef struct stConfigTask
{
	BOOL	bEnabled;
	TSTRING	strName;
	INT32	nPriority;
	INT32	nStartDelay;	// [ns]
	INT32	nPeriod;		// [ns], 0: not periodic
	INT32	nCpu;			// -1: the CPUs of the process

	stConfigTask() : bEnabled(FALSE), strName(""), nPriority(0), nStartDelay(0), nPeriod(0), nCpu(-1) {}
} ST_CONFIG_TASK;

/* SYNC_MODE of [ECAT_MASTER] */
enum eSyncMode { eSyncFreeRun = 0, eSyncSm = 1, eSyncDc = 2 };

typedef struct stConfigEcatMaster
{
	INT32	nNoOfSlaves;
	INT32	nCycleTime;		// [ns], the period of TASK0
	INT32	nSendDeadline;	// t_b of eq. (3) [ns] after the release; 0: one period
	INT32	nSyncMode;		// eSyncMode
	INT32	nDcSync0Shift;	// SYNC_MODE=DC: SYNC0 this long after the release [ns]

	stConfigEcatMaster() : nNoOfSlaves(0), nCycleTime(0), nSendDeadline(0), nSyncMode(eSyncFreeRun),
	                       nDcSync0Shift(0) {}
} ST_CONFIG_ECAT_MASTER;

typedef struct stConfigEcatSlave
{
	TSTRING	strName;
	UINT32	uVendorID;
	UINT32	uProductCode;
	INT32	nSlaveType;			// 0: joint drive, 1: end tool
	BOOL	bCCW;				// MOVE_CCW: s_i = +1 (1) or -1 (0) in eq. (2)
	INT32	nDriveMode;			// 10: cyclic synchronous torque
	double	dEncResolution;		// C_i [counts per revolution]
	double	dGearRatio;			// N_i
	double	dTransRatio;		// further transmission (1 on the Indy7)
	double	dTorqueConstant;	// K_t,i [N m/A]
	double	dCurrentRatio;		// kappa_i [1/A]
	INT32	nHomePositionOffset;// c_0,i [counts]
	double	dTorLimitU;			// tau_max [N m]

	stConfigEcatSlave() : strName("EtherCAT slave"), uVendorID(0), uProductCode(0), nSlaveType(0), bCCW(FALSE),
	                      nDriveMode(10), dEncResolution(0.0), dGearRatio(0.0), dTransRatio(0.0),
	                      dTorqueConstant(0.0), dCurrentRatio(0.0), nHomePositionOffset(0), dTorLimitU(0.0) {}
} ST_CONFIG_ECAT_SLAVE;

typedef struct stConfigSystem
{
	TSTRING strName;
	BOOL	bSim;
	TSTRING strURDFPath;
	BOOL	bEnableControllerAtStartup;
	INT32	nRobotAxis;
	std::vector<double> vKp, vKd;		// gains of eq. (5)
	std::vector<double> vKf;			// friction feedforward K_f [N m] of eq. (7), 0: off
	std::vector<double> vRotorB;		// rotor inertias B_ii [kg m^2] (eq. (4))
	std::vector<double> vSimArmature;	// armature of the simulated arm (default: vRotorB)
	INT32	nFfGate;					// 1: eq. (7) with its gate, 0: without (ablation)
	/* simulation (-s, -b) */
	TSTRING strSimMJCFPath;
	INT32	nSimStateDelay;				// age of the state passed to the controller [steps]
	INT32	nSimTorqueDelay;			// the torque acts this many steps after that state
	BOOL	bSimFriction;				// dry friction (frictionloss) at SIM_TAU_S
	std::vector<double> vSimTauS;		// breakaway torques [N m]
	std::vector<double> vSimInitialQ;	// initial posture [rad]
	INT32	nSimFaultJoint, nSimFaultWhat, nSimFaultPersist, nSimEstopReports;
	/* live view (tools/sim_view.py), 0: off */
	INT32	nViewPort;
	double	dViewRateHz;
	/* JOINT TEST ('u') */
	std::vector<double> vJtFastDq, vJtSlowDq;
	double	dJtFastT, dJtSlowT, dJtDwell;
	TSTRING	strJtLogDir;
	INT32	nTsJoint;  double dTsTau, dTsOn, dTsOff;  INT32 nTsRep;		// 5: torque step
	INT32	nBaJoint;  double dBaRate, dBaMax, dBaQd; INT32 nBaRep;		// 6: breakaway
	INT32	nSineJoint; double dSineAmp, dSineFreq; INT32 nSinePeriods;	// 7: sine
	INT32	nSineFf;
	double	dSineV0;
	/* high-level controller (Table 9) and its tests ('y') */
	double	dHlDelta, dHlLambda, dHlStepMax, dHlTol;
	INT32	nHlIterCycle, nHlIterGoal;
	double	dHlVelMax, dHlAccMax, dHlJerkMax, dHlMargin, dHlLead;
	std::vector<double> vHlReadyQ;
	std::vector<double> vHlTestGoal, vHlTestJump;
	double	dHlTestTrigger, dHlTestNoise, dHlTestNoiseHz, dHlTestNoiseT, dHlTestRefT;
	INT32	nHlRandGoals, nHlRandSeed;
	std::vector<double> vHlRandBox;
	double	dHlRandRMax;
	INT32	nHlRandJumps;
	std::vector<double> vHlRandJumpLen, vHlRandJumpAt;
	INT32	nHlStepBaseline;
	TSTRING	strHlNoiseFile;
	/* drives */
	INT32	nServoOnTimeoutMs;
	/* hand-eye capture ('w', 's') */
	TSTRING	strCalibDir;
	/* the camera application ('p', 'y' 8) */
	INT32	nAppBridge;
	std::vector<double> vAppGoalBox;
	double	dAppGoalRMax, dAppGoalFloor, dAppHover, dAppKappa, dAppGapS, dAppLossS, dAppDurationS;

	stConfigSystem();
} ST_CONFIG_SYSTEM;

typedef std::vector<ST_CONFIG_TASK>			VEC_TASK_CONF;
typedef std::vector<ST_CONFIG_ECAT_SLAVE>	VEC_ECAT_SLAVE_CONF;

class CConfig
{
public:
	CConfig();

	/* the file: -f PATH, or ./INDY7.cfg, or INDY7.cfg next to the
	 * directory of the program (bin/..); the path that was found */
	static TSTRING	Find			(const TSTRING& astrGiven, const char* apcArgv0);
	BOOL			Load			(const TSTRING& astrPath, BOOL abForceSim);

	const ST_CONFIG_SYSTEM&			System	(	) const { return m_stSystem; }
	const ST_CONFIG_ECAT_MASTER&	Master	(	) const { return m_stMaster; }
	const VEC_ECAT_SLAVE_CONF&		Slaves	(	) const { return m_vSlaves; }
	const VEC_TASK_CONF&			Tasks	(	) const { return m_vTasks; }
	const TSTRING&					Path	(	) const { return m_strPath; }

	/* a path of the file: absolute as it is, otherwise relative to the
	 * directory of the file */
	TSTRING			Resolve			(const TSTRING& astrPath) const;
	static const char* SyncModeName	(int anMode);

private:
	BOOL	Parse			(	);

	TSTRING					m_strPath;
	ST_CONFIG_SYSTEM		m_stSystem;
	ST_CONFIG_ECAT_MASTER	m_stMaster;
	VEC_ECAT_SLAVE_CONF		m_vSlaves;
	VEC_TASK_CONF			m_vTasks;
};

#endif // G2T_CONFIG_H
