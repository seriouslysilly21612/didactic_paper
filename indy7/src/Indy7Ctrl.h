/*****************************************************************************
*	Name: Indy7Ctrl.h
*	Description: The controller of the Indy7 (paper, Fig. 1): the bus or the
*	simulated arm, the three periodic tasks, the slots between them, the
*	operator commands and tools, the tests of the paper, and the bridge of
*	the camera application.
*
*	Tasks (cfg [RT_TASKS], one release grid, in the order of their
*	priorities within a period; TaskSlots.h has the data flow):
*	  TASK0  EtherCAT task    TaskEtherCAT.cpp    receive -> State slot; Torque slot -> send;
*	                                              the drives, the LED, the timing log
*	  TASK1  Trajectory task  TaskTrajectory.cpp  goal slot -> high-level controller (IK,
*	                                              Ruckig) -> Reference slot; the high-level
*	                                              tests, the operator motions
*	  TASK2  Dynamics task    TaskDynamics.cpp    State + Reference slot -> control law
*	                                              (gravity compensation, CTC, JOINT TEST)
*	                                              -> Torque slot; the operator commands
*	  TASK3  keyboard task    Indy7Ctrl.cpp       the keys and the lines of the tools
*	The torque sent in a period was computed in the period before from the
*	state received in the period before (send before compute, Fig. 2).
*
*	Modes: the robot (the EtherCAT bus); -s the simulation (MuJoCo in place
*	of the bus, paper, Sec. IV and V); -b the bus of the robot with the
*	drives kept off beside the simulated arm (the host-timing test of
*	Sec. III-C); -t DIR the timing log of every period (TimingLog.h).
*****************************************************************************/
#ifndef G2T_INDY7_CTRL_H
#define G2T_INDY7_CTRL_H

#include <atomic>
#include <vector>

#include "base/Common.h"
#include "base/Config.h"
#include "base/RtTask.h"
#include "base/ViewStream.h"
#include "ecat/EcatMaster.h"
#include "ecat/Indy7Joint.h"
#include "ecat/Indy7EndTool.h"
#include "sim/SimArm.h"
#include "sim/SimJoint.h"
#include "FullDynControllerRT.h"
#include "TrajectoryRT.h"
#include "TaskSlots.h"
#include "CalibCapture.h"
#include "ROS2PickBridge.h"
#include "JointTestLog.h"
#include "HighLevelLog.h"
#include "GoalSlot.h"
#include "TimingLog.h"

class CRobotIndy7
{
public:
	CRobotIndy7(CConfig* apConfig);
	~CRobotIndy7();

	/* before Init: -b, the bus beside the simulated arm; -t DIR */
	void			SetHybridBus			(BOOL abOn) { m_bHybridBus = abOn; }
	void			SetTimingLogDir			(const TSTRING& astrDir) { m_strTimingDir = astrDir; }

	/* the controller, the bus or the simulation, the tasks; FALSE: the
	 * application ends (after StopTasks() and the end of the tasks) */
	BOOL			Init					(	);
	/* after Init, in main(): SYNC_MODE=SM or DC, once the bus is OP,
	 * 0x1C32:01 of the drives; FALSE: the application must end */
	BOOL			CheckSyncMode			(	);
	BOOL			CheckStopTask			(	) const { return m_bStopTask.load() ? TRUE : FALSE; }
	BOOL			AreTasksStarted			(	) const { return m_bTasksStarted; }
	void			StopTasks				(	) { m_bStopTask.store(true); }
	/* every real-time task has ended (the EtherCAT task after switching the
	 * drives off and releasing the master) */
	BOOL			IsControlTaskDone		(	) const
	{ return (m_bEcatDone.load() && m_bTrajDone.load() && m_bDynDone.load()) ? TRUE : FALSE; }
	/* after the tasks: the bridge, the live view, the log writers */
	void			StopServices			(	);
	void			PrintHelp				(	) const;

	/* the controller */
	CControllerFullDynamicsRT* GetController() { return m_pController; }
	BOOL			InitController			(	);
	BOOL			EnableController		(BOOL abEnable);
	BOOL			SetControllerMode		(CControllerFullDynamicsRT::eControlMode aeMode);
	BOOL			IsMoving				(	);
	int				GetTotalAxis			(	) const { return (int)m_vJoints.size(); }

private:
	BOOL			CheckConfig				(	);
	BOOL			InitEtherCAT			(	);
	BOOL			InitSimulation			(	);
	BOOL			InitTrajectory			(	);
	BOOL			InitRTTasks				(	);

	CConfig*					m_pConfig;
	TSTRING						m_strName;
	std::vector<CJoint*>		m_vJoints;		// the joints of the arm (drives or simulated)
	std::vector<CSimJoint*>		m_vSimJoints;	// the same, in simulation (-s, -b)
	std::vector<CIndy7Joint*>	m_vDrives;		// the joint drives on the bus (robot, -b)
	CIndy7EndTool*				m_pEndTool;		// on the bus (robot, -b), NULL otherwise
	CEcatMaster*				m_pcEcatMaster;	// the robot, and -b
	CSimArm*					m_pSim;			// the simulated arm (-s, -b), NULL on the robot
	CViewStream					m_cView;		// newest joint angles for an external viewer

	/* the tasks, in the order of the cfg ([TASK0]..[TASK3]) */
	enum { N_TASKS = 4 };
	CRtTask						m_aTask[N_TASKS];
	BOOL						m_bTasksStarted{FALSE};
	std::atomic<bool>			m_bStopTask{false};
	std::atomic<bool>			m_bBusOp{false};	// the EtherCAT task has seen the bus OP once

	/* operator commands: the key of the keyboard task, read by the Dynamics
	 * task (DoInput) after its torque is published */
	std::atomic<char>	m_cKeyPress{' '};
	void	DoInput						(	);
	// Stop every motion: the running JOINT TEST ends here, TRACK or HOME in
	// the Trajectory task (HL_CMD_STOP), the menus close, and the controller
	// holds in gravity compensation. The common first step of 'e' and 'x'.
	void	StopMotion					(	);

	friend void	proc_ethercat_control(void*);
	friend void	proc_trajectory_control(void*);
	friend void	proc_dynamics_control(void*);
	friend void	proc_keyboard_control(void*);

	/* the slots between the tasks (TaskSlots.h), one writer each */
	CAtomicSlot<task_slots::ST_STATE_SLOT>	m_stateSlot;	// EtherCAT task
	CAtomicSlot<task_slots::ST_REF_SLOT>	m_refSlot;		// Trajectory task
	CAtomicSlot<task_slots::ST_TORQUE_SLOT>	m_torqueSlot;	// Dynamics task
	CAtomicSlot<task_slots::ST_MOVE_REQ>	m_moveSlot;		// keyboard task ('m', 't')
	// and the command words: the drives (Dynamics -> EtherCAT, eServoCmd),
	// the high-level motions (Dynamics -> Trajectory, HL_CMD_* or 1..8)
	std::atomic<int>	m_nServoCmd{task_slots::eServoCmdNone};
	std::atomic<int>	m_nHlCmd{task_slots::HL_CMD_NONE};
	// the Dynamics task after a stop: the Reference slot is ignored until the
	// Trajectory task has acknowledged it (eRefNone), so that the stop acts in
	// the period of the key, not a period later
	BOOL				m_bRefInhibit{FALSE};
	long				m_nRefStale{0};		// periods in which the reference was too old to follow
	long				m_nRefRefused{0};	// references refused because a JOINT TEST was running
	// the copies of the Dynamics task (DoInput reads them)
	task_slots::ST_STATE_SLOT	m_stDynState{};
	task_slots::ST_REF_SLOT		m_stDynRef{};

	// the high-level controller, run by the Trajectory task
	CTrajectoryRT	m_cTraj;

	// -b: the bus of the robot beside the simulated arm. Its joint drives are
	// not joints of the arm: they stay in OP with the power stage off and the
	// brakes engaged (Shutdown in every period)
	BOOL			m_bHybridBus{FALSE};
	BOOL			IsAnyBusServoOn			(	) const;
	// the drives, on the command of the Dynamics task (EtherCAT task)
	void			ApplyServoCommand		(int anCmd);
	// a drive dropout (EtherCAT task): every joint servo off, the counts, and
	// the stop of the motion for the Dynamics task
	void			DriveDropout			(uint32_t auLost, uint32_t auPeriod);
	// a servo on that did not complete within SERVO_ON_TIMEOUT_MS (EtherCAT
	// task): withdrawn, as a dropout; auMissing: the joints not in Operation
	// enabled, apusStatus: their 0x6041
	void			ServoOnExpired			(uint32_t auMissing, const uint16_t* apusStatus, uint32_t auPeriod);

	// -t DIR: the timing of every period (TimingLog.h), started before the
	// tasks, stopped after them (StopServices)
	TSTRING			m_strTimingDir;
	CTimingLog		m_cTimingLog;
	void			StartTimingLog			(	);

	// JOINT TEST ('u', then a digit; paper, Sec. IV-A and V-A). The parameters
	// are copied from the cfg at Init; the log is written by a non-real-time
	// thread. Dynamics task.
	enum { JT_MENU = 7 };
	bool          m_bJtMenu{false};
	double        m_adJtDq[2][CControllerFullDynamicsRT::JT_MAX_DOF]{};   // fast, slow
	double        m_adJtT[2]{1.0, 3.0};
	double        m_dJtDwell{1.0};
	int           m_anJtRuns[JT_MENU]{};
	CJointTestLog::Meta m_stJtMeta{};    // the fields that do not change at run time
	CJointTestLog m_cJtLog;
	// high-level tests ('y', then a digit; paper, Sec. V-B and V-C):
	// 1 fixed goal, 2 goal jump, 3 goal noise with delta, 4 without delta,
	// 5 reference-check stream, 6 HOME, 7 record q_r, 8 the camera application.
	// Each test first moves to q_r (HOME), then runs TRACK with goals that a
	// test program writes into the goal slot before the high-level controller
	// reads it; the application reads the slot of the bridge. Trajectory task:
	// the digit arrives from DoInput through m_nHlCmd.
	enum { HL_MENU = 8, HL_APP = 8 };
	enum eHlTest { eHlTestIdle = 0, eHlTestHoming, eHlTestTracking };
	CGoalSlot     m_goalSlot;             // read by the high-level controller
	bool          m_bHlMenu{false};       // DoInput (Dynamics task)
	eHlTest       m_eHlTest{eHlTestIdle};
	int           m_nHlSel{0};
	long          m_nHlCycle{0};          // cycles of TRACK
	long          m_nHlLastEvent{0};      // cycle of the last goal change of the test
	long          m_nHlRestSince{-1};     // first cycle at rest after it
	bool          m_bHlJumped{false};
	double        m_adHlP0[3]{}, m_adHlBase[3]{}, m_adHlTool[3]{};
	bool          m_bHlGap{false};
	long          m_nHlNextSeg{0};
	unsigned long long m_uHlRng{1};
	// HOME done -> TRACK: the start waits until the Dynamics task reports
	// gravity compensation again (the period after); gives up after
	// HL_START_WAIT periods
	enum { HL_START_WAIT = 100 };
	bool          m_bHlTrackPending{false};
	int           m_nHlStartWait{0};
	// recorded goal noise (HL_TEST_NOISE_FILE), loaded before the tasks:
	// offsets [m] at times [s] from the first sample, replayed in a loop
	enum { HL_NOISE_MAX = 8192 };
	double        m_adHlNoiseRec[HL_NOISE_MAX][3]{};
	double        m_adHlNoiseRecT[HL_NOISE_MAX]{};
	int           m_nHlNoiseRec{0};
	int           m_nHlNoiseIdx{-1};
	char          m_acHlNoiseFile[64]{};
	void          LoadHighLevelNoise();
	int           m_anHlRuns[HL_MENU]{};
	/* from the cfg, copied at Init */
	double        m_adHlGoalOff[3]{}, m_adHlJump[3]{};
	double        m_dHlTrigger{0.15}, m_dHlNoise{0.001}, m_dHlNoiseHz{15.0}, m_dHlNoiseT{10.0}, m_dHlRefT{60.0};
	double        m_dHlDelta{0.006};
	bool          m_bHlFF{false};         // friction FF in TRACK/HOME: robot, or simulation with friction
	CHighLevelLog m_cHlLog;
	CHighLevelLog::Meta m_stHlMeta{};
	// the camera application (paper, Sec. V-C): the bridge (APP_BRIDGE=1, a
	// build with ROS 2) writes the goals from the camera into m_appGoalSlot,
	// of which its executor thread is the only writer; 'p' lists the objects
	// in view, a digit selects one and the arm follows it ('y', 8 runs the
	// same); the high-level controller adds the hover height to every
	// position that it reads. Started before the tasks, stopped after them.
	CROS2PickBridge* m_pBridge{NULL};
	CGoalSlot     m_appGoalSlot;
	double        m_dAppHover{0.05};
	double        m_dAppT{0.0};           // length of a run of the application [s], 0: until 'x'
	void          InitBridge              (	);
	unsigned long m_uHlAllocs0{0};
	/* test 1 with random goals (cfg HL_RAND_*): drawn before the tasks use
	 * them, inside the box and solved by the IK from q_r; 'y', 1 runs them
	 * one after another, each from q_r (HOME), until 'x' or a stop */
	enum { HL_GOALS_MAX = 1000 };
	double        m_adHlGoals[HL_GOALS_MAX][3]{};
	int           m_nHlGoals{0};          // 0: the fixed goal HL_TEST_GOAL
	int           m_nHlGoalIdx{0};        // goal of the current or the next run
	bool          m_bHlSeq{false};        // the goals (test 1) or jumps (test 2) run one after another
	bool          m_bHlSeqNext{false};    // start the next once the log is written
	/* test 2 with random goal jumps (cfg HL_RAND_JUMP*): a first goal drawn
	 * like those of test 1, a new goal HL_RAND_JUMP_LEN_M from it, and the
	 * fraction of the starting distance to the first goal at which the goal
	 * jumps; 'y', 2 runs them one after another, each from q_r (HOME) */
	double        m_adHlJ1[HL_GOALS_MAX][3]{}, m_adHlJ2[HL_GOALS_MAX][3]{}, m_adHlJAt[HL_GOALS_MAX]{};
	int           m_nHlJumps{0};          // 0: the fixed jump HL_TEST_JUMP
	int           m_nHlJumpIdx{0};        // jump of the current or the next run
	double        m_adHlJumpRun[3]{}, m_dHlTriggerRun{0.15};   // jump and trigger of the current run
	void          DrawHighLevelGoals       (	);
	void          DrawHighLevelJumps       (	);
	void          StopHighLevelSeq         (const char* apcWhen);
	void          InitHighLevelTests       (	);
	/* Trajectory task: the menu entry (from m_nHlCmd), the stop, the goal of
	 * the cycle, and what follows the high-level step */
	BOOL          StartHighLevelMenu       (int anSel, const task_slots::ST_STATE_SLOT& astState,
	                                        const task_slots::ST_TORQUE_SLOT& astTorque);
	BOOL          StartHighLevelTrack      (const task_slots::ST_STATE_SLOT& astState,
	                                        const task_slots::ST_TORQUE_SLOT& astTorque);
	void          StopHighLevel            (	);   // ends TRACK or HOME; the test closes in HighLevelTestAfterUpdate
	void          HighLevelTestBeforeUpdate(	);
	void          HighLevelTestAfterUpdate (const task_slots::ST_STATE_SLOT& astState,
	                                        const task_slots::ST_TORQUE_SLOT& astTorque);
	void          EndHighLevelTest         (int anState, const task_slots::ST_FAULTS& astFaults,
	                                        unsigned long auAllocs);

	/* the operator motions (keyboard task, 'm' and 't'): checked there
	 * against the joint limits and the reach of the IK, then run by the
	 * Trajectory task: 'm' moves to joint angles (HOME to a target), 't'
	 * moves the tool to a position (TRACK on goals of the operator, which
	 * stays on: every 't' is a new goal, 'x' ends it) */
	enum eUserMove { eUserNone = 0, eUserJoint, eUserTask };
	eUserMove     m_eUserMove{eUserNone};  // Trajectory task
	uint32_t      m_uMoveSeen{0};          // Trajectory task
	CHighLevelRT* m_pKeyIk{NULL};          // keyboard task: its own model of the IK
	BOOL          ReadJointMove            (	);   // keyboard task
	BOOL          ReadTaskMove             (	);   // keyboard task
	void          PassKey                  (char acKey);   // keyboard task: a key typed at their prompt
	void          StartUserMove            (const task_slots::ST_MOVE_REQ& astReq,
	                                        const task_slots::ST_STATE_SLOT& astState,
	                                        const task_slots::ST_TORQUE_SLOT& astTorque);
	BOOL          CheckMotionStart         (const char* apcWho, const task_slots::ST_STATE_SLOT& astState,
	                                        const task_slots::ST_TORQUE_SLOT& astTorque);

	// malloc-count build: heap allocations of the Dynamics task, in the
	// control law (Update) and in the whole cycle, and of the whole cycles of
	// the EtherCAT and the Trajectory task
	unsigned long m_uAllocUpdate{0}, m_uAllocCycle{0}, m_uAllocCycles{0}, m_uAllocUpdateMaxCycle{0};
	unsigned long m_uAllocEcat{0}, m_uAllocTraj{0};
	unsigned long m_uJtAllocs0{0};

	// timing of the EtherCAT task (paper, Sec. III-B, eq. (3)), written by the
	// EtherCAT task only: t_w from the planned release to the start of the
	// cycle, t_s from the start to the return of the send, t_e from the start
	// to the end of the cycle; the timing faults, and the incomplete working
	// counters
	struct RtStats {
		RTTIME tPeriod, tDeadline;     // [ns] period, send bound t_b after the release
		long   nCycles;                // cycles that ran
		long   nLate;                  // releases that started more than a period late
		long   nOverruns;              // cycles that ended after the next release
		long   nMissed;                // sends that returned after t_b
		long   nWkc;                   // incomplete working counters after the bus was up: no control
		RTTIME tWMax, tSMax, tEMax;    // [ns]
		double dWSum, dSSum, dESum;    // [ns]
		UINT32 uSyncDiffMax;           // SYNC_MODE=DC: largest system-time difference of the slaves (0x092C)
	};
	RtStats           m_stRt{};
	long              m_nTorqueStale{0};   // periods without a fresh torque while OP (the last target stayed)
	// the timing of the two other tasks, each written by itself
	task_slots::ST_TASK_TIMING m_stTrajTiming{}, m_stDynTiming{};
	// the faults at the start of a test (from the State slot), for the logs
	task_slots::ST_FAULTS m_stJtRt0{}, m_stHlRt0{};
	std::atomic<bool> m_bEcatDone{false}, m_bTrajDone{false}, m_bDynDone{false};
	// controller fault (TaskDynamics.cpp): the control law failed (Update()
	// FALSE: a non-finite measured state or torque). The Dynamics task sends
	// zero torque in that period and latches the fault once: the motion ends
	// and the drives go off in the next period, as with 'e'; 'h' is refused
	// until the operator has switched the controller off ('r', which clears
	// the fault) and on ('r'). Written by the Dynamics task only (the law,
	// DoInput); atomics for the Trajectory task (its test log) and the summary.
	std::atomic<bool> m_bCtrlFault{false};       // latched
	std::atomic<long> m_nCtrlFaults{0};          // latches so far
	uint32_t          m_uCtrlFaultFirst{0}, m_uCtrlFaultLast{0};   // EtherCAT periods of the first and the last latch
	long              m_nCtrlFaultCycles{0};     // cycles in which the law failed (also while latched)
	long              m_nJtCtrlFaults0{0}, m_nHlCtrlFaults0{0};    // latches at the start of a test, for the logs
	// drive dropout (TaskEtherCAT.cpp): while the servo-on command of the
	// operator ('h') is active, a joint that has reached Operation enabled
	// since that command leaves it without a command (the E-stop of the
	// control box, a fault of the drive). The EtherCAT task switches every
	// joint off in that period (DriveDropout; ServoOff clears the on-request
	// of every drive, so that none enables itself again when the E-stop is
	// released), as 'e' does; the Dynamics task stops the motion
	// (m_nDriveStop). 'h' enables the drives again, once the cause is gone.
	// A servo on that has not brought every joint to Operation enabled within
	// SERVO_ON_TIMEOUT_MS is withdrawn the same way (ServoOnExpired), and 'h'
	// is refused while a drive stays in Switch on disabled (DoInput).
	// Written by the EtherCAT task, except m_nServoRefused (Dynamics task).
	std::atomic<long> m_nDriveDropouts{0};       // dropouts so far
	uint32_t          m_uDropoutFirst{0}, m_uDropoutLast{0};   // EtherCAT periods of the first and the last
	std::atomic<int>  m_nDriveStop{0};           // EtherCAT -> Dynamics task: stop the motion
	long              m_nJtDropouts0{0}, m_nHlDropouts0{0};    // dropouts at the start of a test, for the logs
	int               m_nServoOnTimeoutMs{2000}; // SERVO_ON_TIMEOUT_MS, copied at Init
	long              m_nServoOnDone{0};         // servo ons that completed
	RTTIME            m_tServoOnMax{0};          // the longest of them, 'h' to every joint in Operation enabled [ns]
	long              m_nServoExpired{0};        // servo ons withdrawn after SERVO_ON_TIMEOUT_MS
	long              m_nServoRefused{0};        // 'h' refused: a drive in Switch on disabled (DoInput)
	// fault injection ('n'; cfg SIM_FAULT_*, simulation only, copied at
	// Init): the joint (0-based, -1: off), the torque (0) or the measured
	// angle (1), one cycle or every cycle until 'r' off; the angle is poisoned
	// by the Dynamics task before Update() (m_nStateInject: 1 the next cycle,
	// 2 every cycle), the torque by the controller (InjectTorqueFault). Or a
	// drive dropout (2): the EtherCAT task takes the simulated joint out of
	// Operation enabled once (m_nSimDropout: the joint, 1-based; 0: none)
	int               m_nSimFaultJoint{-1};
	int               m_nSimFaultWhat{0};
	bool              m_bSimFaultPersist{false};
	int               m_nStateInject{0};
	std::atomic<int>  m_nSimDropout{0};
	// the E-stop tests (3, SIM_FAULT_WHAT=estop): 'n' cuts the power of every
	// simulated drive, the next 'n' restores it (CSimJoint::SetPowerCut); the
	// Dynamics task keeps the toggle, the EtherCAT task applies it before the
	// state of its next period (m_nSimEstopCmd: 1 cut, 2 restore); the
	// drives report m_nSimEstopReports meanwhile
	bool              m_bSimEstop{false};
	int               m_nSimEstopReports{0};
	std::atomic<int>  m_nSimEstopCmd{0};
	// the timing counts of a test, from its start to now, as the EtherCAT
	// task counts them (State slot). In simulation the EtherCAT task steps the
	// simulator once per cycle that runs, so the loop keeps the timing of the
	// robot whatever the host does: no timing faults there (JointTestLog.cpp).
	// With -b the EtherCAT exchange is real, so its faults count as on the
	// robot. And the controller faults and the drive dropouts during the test
	// (latches since anCtrlFaults0 and anDropouts0), in every mode.
	template <class R> void FillRtResult(R& arResult, const task_slots::ST_FAULTS& astNow,
	                                     const task_slots::ST_FAULTS& astAt0, long anCtrlFaults0,
	                                     long anDropouts0) const
	{
		arResult.nLate     = astNow.nLate - astAt0.nLate;
		arResult.nOverruns = astNow.nOverruns - astAt0.nOverruns;
		arResult.nMissed   = astNow.nMissed - astAt0.nMissed;
		arResult.nWkc      = astNow.nWkc - astAt0.nWkc;
		arResult.nTimingFaults = (m_pSim != NULL && m_pcEcatMaster == NULL) ? 0 :
		                         arResult.nLate + arResult.nOverruns + arResult.nMissed;
		arResult.nCtrlFaults = m_nCtrlFaults.load(std::memory_order_acquire) - anCtrlFaults0;
		arResult.nDriveDropouts = m_nDriveDropouts.load(std::memory_order_acquire) - anDropouts0;
	}
	void              PrintRtSummary  (	) const;
	void          InitJointTest      (	);
	BOOL          StartJointTestMenu (int anSel, const task_slots::ST_STATE_SLOT& astState);

	/* the controller */
	CControllerFullDynamicsRT*	m_pController;
	BOOL m_bRTControllerEnabled;

	// control vectors (Dynamics task)
	std::vector<double> m_vCurrentPos;
	std::vector<double> m_vCurrentVel;
	std::vector<double> m_vCurrentTor;
	std::vector<double> m_vOutputTorque;

	// Hand-eye capture: 'w' arms it, then every 's' appends the tool pose to
	// robot_poses.csv and writes pose_rPe_<N>.yaml in CALIB_DIR of the cfg
	// (calib/eye_to_hand_calib.py). Off at every start: a stray 's' would
	// rewrite pose N of a dataset whose images it does not match.
	CalibCapture                    m_cCalib;
	CControllerFullDynamicsRT::Pose m_Pose;
	int                             m_nPoseCapture{0};
	bool                            m_bCalibMode{false};
	void SaveRobotPose();
};

#endif // G2T_INDY7_CTRL_H
