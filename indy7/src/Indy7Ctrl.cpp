/*****************************************************************************
*	Name: Indy7Ctrl.cpp
*	Description: The controller of the Indy7. See Indy7Ctrl.h.
*****************************************************************************/
#include "Indy7Ctrl.h"
#include "base/MallocCount.h"
#include "CallTiming.h"

#include <cmath>
#include <cstring>
#include <ctime>
#include <sys/stat.h>
#include <sys/utsname.h>

// the tasks, in the order of the cfg ([RT_TASKS]): TaskEtherCAT.cpp,
// TaskTrajectory.cpp, TaskDynamics.cpp, and the keyboard below
void proc_ethercat_control(void* apRobot);
void proc_trajectory_control(void* apRobot);
void proc_dynamics_control(void* apRobot);
void proc_keyboard_control(void* apRobot);

namespace {
/* the first releases come this long after the tasks are created */
const RTTIME GRID_LEAD_NS = 100000000ULL;
}

/****************************************************************************/
CRobotIndy7::CRobotIndy7(CConfig* apConfig)
	: m_pConfig(apConfig), m_pEndTool(NULL), m_pcEcatMaster(NULL), m_pSim(NULL), m_pController(NULL),
	  m_bRTControllerEnabled(FALSE)
{
	m_strName = m_pConfig->System().strName;
}

CRobotIndy7::~CRobotIndy7()
{
	delete m_pController;
	m_pController = NULL;
	delete m_pKeyIk;
	m_pKeyIk = NULL;
	delete m_pcEcatMaster;		// releases the master if it is still requested
	m_pcEcatMaster = NULL;
	for (CIndy7Joint* p : m_vDrives)
		delete p;
	m_vDrives.clear();
	delete m_pEndTool;
	m_pEndTool = NULL;
	for (CSimJoint* p : m_vSimJoints)
		delete p;
	m_vSimJoints.clear();
	m_vJoints.clear();
	delete m_pSim;
	m_pSim = NULL;
}

/* the values that the controller relies on, before anything starts */
BOOL
CRobotIndy7::CheckConfig()
{
	const ST_CONFIG_SYSTEM& stSys = m_pConfig->System();
	const ST_CONFIG_ECAT_MASTER& stM = m_pConfig->Master();
	const VEC_TASK_CONF& vT = m_pConfig->Tasks();
	BOOL bOk = TRUE;
	if (stSys.nRobotAxis != CHighLevelRT::DOF)
	{
		DBG_LOG_ERROR("(CRobotIndy7) ROBOT_DOF=%d: the Indy7 has %d joints", (int)stSys.nRobotAxis,
		              (int)CHighLevelRT::DOF);
		bOk = FALSE;
	}
	if (stSys.strURDFPath.empty())
	{
		DBG_LOG_ERROR("(CRobotIndy7) URDF_PATH is missing in [SYSTEM]");
		bOk = FALSE;
	}
	int nDrives = 0, nTools = 0;
	for (size_t n = 0; n < m_pConfig->Slaves().size(); n++)
	{
		const ST_CONFIG_ECAT_SLAVE& s = m_pConfig->Slaves()[n];
		if (s.nSlaveType == 1)
		{
			nTools++;
			continue;
		}
		if (s.nSlaveType != 0)
		{
			DBG_LOG_ERROR("(CRobotIndy7) [AXIS%u] SLAVE_TYPE=%d: 0 (joint drive) or 1 (end tool)", (unsigned)n,
			              (int)s.nSlaveType);
			bOk = FALSE;
			continue;
		}
		nDrives++;
		if (s.nDriveMode != CIndy7Joint::MODE_CYCLIC_TORQUE)
		{
			DBG_LOG_ERROR("(CRobotIndy7) [AXIS%u] DRIVE_MODE=%d: the controller runs the drives in cyclic "
			              "synchronous torque mode (10)", (unsigned)n, (int)s.nDriveMode);
			bOk = FALSE;
		}
		if (!(s.dEncResolution > 0.0 && s.dGearRatio > 0.0 && s.dTransRatio > 0.0 && s.dTorqueConstant > 0.0 &&
		      s.dCurrentRatio > 0.0 && s.dTorLimitU > 0.0))
		{
			DBG_LOG_ERROR("(CRobotIndy7) [AXIS%u] ENCODER_RESOLUTION, GEAR_RATIO, TRANSMISSION_RATIO, "
			              "TORQUE_CONSTANT, CURRENT_RATIO and TORQUE_LIMIT_U must be positive", (unsigned)n);
			bOk = FALSE;
		}
	}
	if (nDrives != stSys.nRobotAxis || nTools > 1)
	{
		DBG_LOG_ERROR("(CRobotIndy7) [ECAT_MASTER] SLAVENUMBER=%d: %d joint drives and %d end tools, %d and at "
		              "most 1 expected", (int)stM.nNoOfSlaves, nDrives, nTools, (int)stSys.nRobotAxis);
		bOk = FALSE;
	}
	if ((int)vT.size() != N_TASKS)
	{
		DBG_LOG_ERROR("(CRobotIndy7) [RT_TASKS] NO_OF_TASKS=%d: the controller has %d tasks (EtherCAT, "
		              "Trajectory, Dynamics, keyboard)", (int)vT.size(), (int)N_TASKS);
		return FALSE;
	}
	for (int n = 0; n < 3; n++)
	{
		if (!vT[n].bEnabled || vT[n].nPeriod <= 0 || vT[n].nPeriod != stM.nCycleTime)
		{
			DBG_LOG_ERROR("(CRobotIndy7) [TASK%d] must be enabled with TASK_PERIOD = CYCLE_TIME of [ECAT_MASTER] "
			              "(%d ns)", n, (int)stM.nCycleTime);
			bOk = FALSE;
		}
		if (n > 0 && vT[n].nPriority >= vT[n - 1].nPriority)
		{
			DBG_LOG_ERROR("(CRobotIndy7) the priorities must decrease from TASK0 to TASK2 (the order of the "
			              "pipeline within a period)");
			bOk = FALSE;
		}
	}
	if (!vT[3].bEnabled || vT[3].nPeriod != 0)
	{
		DBG_LOG_ERROR("(CRobotIndy7) [TASK3] (the keyboard) must be enabled, with TASK_PERIOD=0");
		bOk = FALSE;
	}
	if (stM.nSyncMode == eSyncDc && (stM.nDcSync0Shift <= 0 || stM.nDcSync0Shift >= stM.nCycleTime))
	{
		DBG_LOG_ERROR("(CRobotIndy7) SYNC_MODE=DC: DC_SYNC0_SHIFT=%d ns must lie inside the cycle",
		              (int)stM.nDcSync0Shift);
		bOk = FALSE;
	}
	/* the slots: 64-bit atomic words without a lock (paper, Sec. II-A) */
	if (!CAtomicSlot<task_slots::ST_STATE_SLOT>::IsLockFree())
	{
		DBG_LOG_ERROR("(CRobotIndy7) 64-bit atomics are not lock-free on this processor: the slots cannot be used");
		bOk = FALSE;
	}
	return bOk;
}

BOOL
CRobotIndy7::Init()
{
	const ST_CONFIG_SYSTEM& stSys = m_pConfig->System();
	const BOOL bSim = stSys.bSim;
	if (FALSE == CheckConfig())
		return FALSE;
	if (FALSE == InitController())
	{
		DBG_LOG_ERROR("(CRobotIndy7) cannot initialize the controller");
		return FALSE;
	}
	// the high-level controller of the Trajectory task (TRACK, HOME); without
	// it the two are refused
	if (FALSE == InitTrajectory())
		return FALSE;

	// SAFETY: the step baseline of the high-level controller commands steps
	// of the reference; only the simulation (-s) may run it
	if (stSys.nHlStepBaseline != 0 && (!bSim || m_bHybridBus))
	{
		DBG_LOG_ERROR("(CRobotIndy7) HL_STEP_BASELINE=1 is refused outside the simulation (-s)");
		return FALSE;
	}
	if (stSys.nHlStepBaseline != 0)
		printf("[HL] STEP BASELINE: the IK solution is the reference, as a step (no generator, no lead bound)\n");
	// SAFETY: the fault injection ('n') makes a torque or a measured angle
	// non-finite, takes a drive out of Operation enabled, or cuts the power
	// of the drives; only the simulation (-s) may have it
	if (stSys.nSimFaultJoint != 0)
	{
		if (!bSim || m_bHybridBus)
		{
			DBG_LOG_ERROR("(CRobotIndy7) SIM_FAULT_JOINT=%d is refused outside the simulation (-s)",
			              (int)stSys.nSimFaultJoint);
			return FALSE;
		}
		if (stSys.nSimFaultJoint < 0 || stSys.nSimFaultJoint > stSys.nRobotAxis || stSys.nSimFaultWhat < 0 ||
		    stSys.nSimEstopReports < 0)
		{
			DBG_LOG_ERROR("(CRobotIndy7) SIM_FAULT_JOINT=%d, SIM_FAULT_WHAT: a joint 1..%d (0: off), torque, state, "
			              "dropout or estop; SIM_ESTOP_REPORTS: sod, ready or fault", (int)stSys.nSimFaultJoint,
			              (int)stSys.nRobotAxis);
			return FALSE;
		}
		m_nSimFaultJoint = (int)stSys.nSimFaultJoint - 1;
		m_nSimFaultWhat = (int)stSys.nSimFaultWhat;
		m_bSimFaultPersist = (stSys.nSimFaultPersist != 0);
		m_nSimEstopReports = (int)stSys.nSimEstopReports;
		if (m_nSimFaultWhat == 3)
			printf("[FAULT] fault injection on 'n': the E-stop of the control box, every drive loses its power "
			       "('n' again restores it) and reports %s meanwhile\n",
			       m_nSimEstopReports == 1 ? "Ready to switch on" : m_nSimEstopReports == 2 ? "Fault" :
			       "Switch on disabled");
		else if (m_nSimFaultWhat == 2)
			printf("[FAULT] fault injection on 'n': J%d leaves Operation enabled once, with the servo on (the "
			       "drive-dropout test)\n", m_nSimFaultJoint + 1);
		else
			printf("[FAULT] fault injection on 'n': a non-finite %s of J%d, %s (the controller-fault test)\n",
			       m_nSimFaultWhat == 1 ? "measured angle" : "torque", m_nSimFaultJoint + 1,
			       m_bSimFaultPersist ? "in every cycle until 'r' switches the controller off" : "in one cycle");
	}
	// a servo on is withdrawn when it has not completed after this time
	m_nServoOnTimeoutMs = stSys.nServoOnTimeoutMs > 0 ? (int)stSys.nServoOnTimeoutMs : 2000;
	m_cCalib.SetDir(stSys.strCalibDir);

	// t_b of eq. (3), the send bound after the release (0: one period)
	const INT32 nDeadline = m_pConfig->Master().nSendDeadline;
	m_stRt.tDeadline = nDeadline > 0 ? (RTTIME)nDeadline : 0;

	// call timing of the library calls (CallTiming.h), from the environment;
	// a run with it measures the calls, not the timing of the tasks
	call_timing::Init();
	if (call_timing::g_bOn)
		printf("[RT] call timing ON (G2T_CALL_TIMING): the clock is read around the library calls of the "
		       "control law and of the high-level controller; this run measures the calls, not the timing of the tasks\n");

	// the IK of the task move ('t'): Step 2 of the high-level controller on a
	// model instance of its own, for the keyboard task, which checks the reach
	// of a position before the Trajectory task moves there
	if (m_cTraj.IsInit())
	{
		m_pKeyIk = new CHighLevelRT();
		if (FALSE == m_pKeyIk->Init(stSys.strURDFPath.c_str(), (double)m_pConfig->Tasks()[1].nPeriod * 1e-9,
		                            m_cTraj.HighLevel().GetParams()))
		{
			delete m_pKeyIk;
			m_pKeyIk = NULL;
		}
	}

	// the bridge of the camera application, the logs of the tests, the live
	// view: before the tasks use them
	InitBridge();
	InitJointTest();
	InitHighLevelTests();
	if (stSys.nViewPort > 0)
		m_cView.Start(stSys.nViewPort, stSys.dViewRateHz, (int)stSys.nRobotAxis);

	// the bus, or the simulated arm (with -b also the bus)
	if (FALSE == (bSim ? InitSimulation() : InitEtherCAT()))
		return FALSE;

	// -t: the log takes the first period of the EtherCAT task
	if (!m_strTimingDir.empty())
		StartTimingLog();

	if (FALSE == InitRTTasks())
		return FALSE;
	DBG_LOG_INFO("(CRobotIndy7) %s: %s", m_strName.c_str(),
	             m_bHybridBus ? "the bus of the robot (drives off) beside the simulated arm (-b)" :
	             bSim ? "simulation (MuJoCo in place of the bus)" : "the robot on the EtherCAT bus");
	return TRUE;
}

// the three periodic tasks on one grid and the keyboard task; the policy,
// priority and affinity of every task are read back by the task itself
BOOL
CRobotIndy7::InitRTTasks()
{
	const VEC_TASK_CONF& vT = m_pConfig->Tasks();
	const PTASKFCN apFcn[N_TASKS] = { &proc_ethercat_control, &proc_trajectory_control, &proc_dynamics_control,
	                                  &proc_keyboard_control };
	const RTTIME tOrigin = CRtTask::GridOrigin(GRID_LEAD_NS);
	for (int n = 0; n < N_TASKS; n++)
	{
		CRtTask::Spec stS;
		stS.strName = vT[n].strName;
		stS.nPriority = vT[n].nPriority;
		stS.nCpu = vT[n].nCpu;
		stS.tPeriod = (RTTIME)(vT[n].nPeriod > 0 ? vT[n].nPeriod : 0);
		stS.tStartDelay = (RTTIME)(vT[n].nStartDelay > 0 ? vT[n].nStartDelay : 0);
		if (FALSE == m_aTask[n].Create(stS, tOrigin))
			return FALSE;
	}
	for (int n = 0; n < N_TASKS; n++)
	{
		if (FALSE == m_aTask[n].Start(apFcn[n], this))
		{
			// the tasks that run end after the stop; those that did not
			// start count as ended
			StopTasks();
			if (!m_aTask[0].IsStarted())
				m_bEcatDone.store(true, std::memory_order_release);
			if (!m_aTask[1].IsStarted())
				m_bTrajDone.store(true, std::memory_order_release);
			if (!m_aTask[2].IsStarted())
				m_bDynDone.store(true, std::memory_order_release);
			return FALSE;
		}
		m_bTasksStarted = TRUE;
	}
	/* the read-back of every task, before the first release */
	for (int nWait = 0; nWait < 1000; nWait++)
	{
		int nPending = 0;
		for (int n = 0; n < N_TASKS; n++)
			if (m_aTask[n].GetCheck() == CRtTask::eCheckPending)
				nPending++;
		if (nPending == 0)
			break;
		usleep(1000);
	}
	for (int n = 0; n < N_TASKS; n++)
		if (m_aTask[n].GetCheck() != CRtTask::eCheckOk)
		{
			DBG_LOG_ERROR("(CRobotIndy7) task '%s' does not run as requested - the controller ends",
			              vT[n].strName.c_str());
			StopTasks();
			return FALSE;
		}
	return TRUE;
}

// SYNC_MODE=SM or DC: once the EtherCAT task has seen the bus OP, the
// synchronization type of every joint drive (main(), not real-time)
BOOL
CRobotIndy7::CheckSyncMode()
{
	if (m_pcEcatMaster == NULL || m_pcEcatMaster->GetSyncMode() == eSyncFreeRun)
		return TRUE;
	for (int nWait = 0; nWait < 10000 && !m_bBusOp.load() && !CheckStopTask(); nWait++)
		usleep(1000);
	if (CheckStopTask())
		return TRUE;
	if (!m_bBusOp.load())
	{
		fprintf(stderr, "[BUS] WARNING: the bus did not reach OP within 10 s with SYNC_MODE=%s - the controller "
		        "ends; check the drives, or use SYNC_MODE=FREE_RUN\n",
		        CConfig::SyncModeName(m_pcEcatMaster->GetSyncMode()));
		return FALSE;
	}
	if (FALSE == m_pcEcatMaster->CheckSyncInOp())
	{
		fprintf(stderr, "[BUS] WARNING: the drives do not run in SYNC_MODE=%s - the controller ends; use "
		        "SYNC_MODE=FREE_RUN\n", CConfig::SyncModeName(m_pcEcatMaster->GetSyncMode()));
		return FALSE;
	}
	return TRUE;
}

// the bridge of the camera application (paper, Sec. V-C; ROS2PickBridge.h):
// with APP_BRIDGE=1 in the cfg and a build with ROS 2, before the tasks
// start. Also in simulation (-s): the goals from the camera then move the
// simulated arm.
void
CRobotIndy7::InitBridge()
{
	const ST_CONFIG_SYSTEM& stSys = m_pConfig->System();
	m_dAppHover = stSys.dAppHover;
	m_dAppT = stSys.dAppDurationS;
	if (stSys.nAppBridge == 0)
		return;
	CROS2PickBridge::Config stCfg;
	for (int k = 0; k < 6; k++)
		stCfg.stFilter.adBox[k] = k < (int)stSys.vAppGoalBox.size() ? stSys.vAppGoalBox[k] : 0.0;
	stCfg.stFilter.dRMax  = stSys.dAppGoalRMax;
	stCfg.stFilter.dFloor = stSys.dAppGoalFloor;
	stCfg.stFilter.dHover = stSys.dAppHover;
	stCfg.stFilter.dKappa = stSys.dAppKappa;
	stCfg.stFilter.dGapS  = stSys.dAppGapS;
	stCfg.dLossS = stSys.dAppLossS;
	m_pBridge = new CROS2PickBridge();
	if (FALSE == m_pBridge->Init(stCfg, &m_appGoalSlot))
	{
		delete m_pBridge;
		m_pBridge = NULL;
		printf("[APP] no bridge - the camera application ('p', 'y' 8) is unavailable\n");
	}
}

// JOINT TEST: copy the parameters from the cfg and start the writer of the
// logs (not real-time, before the tasks use them)
void
CRobotIndy7::InitJointTest()
{
	const ST_CONFIG_SYSTEM& stSys = m_pConfig->System();
	const int nJ = CControllerFullDynamicsRT::JT_MAX_DOF;
	for (int i = 0; i < nJ; i++)
	{
		m_adJtDq[0][i] = i < (int)stSys.vJtFastDq.size() ? stSys.vJtFastDq[i] : 0.0;
		m_adJtDq[1][i] = i < (int)stSys.vJtSlowDq.size() ? stSys.vJtSlowDq[i] : 0.0;
	}
	m_adJtT[0] = stSys.dJtFastT;
	m_adJtT[1] = stSys.dJtSlowT;
	m_dJtDwell = stSys.dJtDwell;

	CJointTestLog::Meta& stM = m_stJtMeta;
	memset(&stM, 0, sizeof(stM));
	stM.nDof = (int)stSys.nRobotAxis < nJ ? (int)stSys.nRobotAxis : nJ;
	stM.dDwell = m_dJtDwell;
	stM.dPeriod = (double)m_pConfig->Tasks()[0].nPeriod * 1e-9;
	stM.bSim = stSys.bSim;
	stM.bSimFriction = stSys.bSimFriction;
	stM.nSimStateDelay = stSys.nSimStateDelay;
	stM.nSimTorqueDelay = stSys.nSimTorqueDelay;
	for (int i = 0; i < stM.nDof; i++)
		stM.adSimTauS[i] = (stSys.bSim && i < (int)stSys.vSimTauS.size()) ? stSys.vSimTauS[i] : 0.0;

	if (m_cJtLog.Start(stSys.strJtLogDir.c_str(), "jt-log"))
		DBG_LOG_INFO("(CRobotIndy7) test logs in %s", m_cJtLog.GetDir().c_str());
}

// start the JOINT TEST of menu entry anSel (Dynamics task, DoInput); the
// servo state and the fault counts come from the State slot of the period
BOOL
CRobotIndy7::StartJointTestMenu(int anSel, const task_slots::ST_STATE_SLOT& astState)
{
	typedef CControllerFullDynamicsRT C;
	static const struct { int nTest; C::eJointTestLaw eLaw; const char* pcLaw; const char* pcName; }
		s_astMenu[JT_MENU] = {
			{ 0, C::eJtPDG,   "pdg",   "fast, PD + g"   },
			{ 0, C::eJtCTC,   "ctc",   "fast, CTC"      },
			{ 1, C::eJtCTC,   "ctc",   "slow, CTC"      },
			{ 1, C::eJtCTCFF, "ctcff", "slow, CTC + FF" },
			{ 2, C::eJtStep,  "step",  "torque step (delay test)" },
			{ 3, C::eJtRamp,  "ramp",  "breakaway (tau_s, sign test)" },
			{ 4, C::eJtCTCFF, "ctcff", "sine (rotor inertia)" } };
	if (anSel < 0 || anSel >= JT_MENU)
		return FALSE;
	if (m_pController == NULL || !m_bRTControllerEnabled || !m_pController->IsEnabled())
	{
		printf("[JT] refused - controller not enabled ('r', then 'g')\n");
		return FALSE;
	}
	if (!astState.bServoOn)
	{
		printf("[JT] refused - servo OFF ('h' first)\n");
		return FALSE;
	}
	if (!m_cJtLog.IsIdle())
	{
		printf("[JT] refused - the log of the last test is still being written\n");
		return FALSE;
	}
	const int nT = s_astMenu[anSel].nTest;
	const bool bStep = (s_astMenu[anSel].eLaw == C::eJtStep);
	const bool bRamp = (s_astMenu[anSel].eLaw == C::eJtRamp);
	const bool bSine = (nT == 4);
	const ST_CONFIG_SYSTEM& stSys = m_pConfig->System();
	char acWhy[192];
	const BOOL bOk = bSine ?
		m_pController->StartSineTest(stSys.nSineJoint - 1, stSys.dSineAmp, stSys.dSineFreq, stSys.nSinePeriods,
		                             stSys.dSineV0, stSys.nSineFf ? C::eJtCTCFF : C::eJtCTC, acWhy, sizeof(acWhy)) :
		bStep ?
		m_pController->StartTorqueStep(stSys.nTsJoint - 1, stSys.dTsTau, stSys.dTsOn, stSys.dTsOff, stSys.nTsRep,
		                               acWhy, sizeof(acWhy)) :
		bRamp ?
		m_pController->StartBreakaway(stSys.nBaJoint - 1, stSys.dBaRate, stSys.dBaMax, stSys.dBaQd, stSys.nBaRep,
		                              acWhy, sizeof(acWhy)) :
		m_pController->StartJointTest(m_adJtDq[nT], m_adJtT[nT], m_dJtDwell, s_astMenu[anSel].eLaw,
		                              acWhy, sizeof(acWhy));
	if (FALSE == bOk)
	{
		printf("[JT] refused - %s\n", acWhy);
		return FALSE;
	}
	// what the log needs: the fixed part from Init, the rest as it is now
	CJointTestLog::Meta stM = m_stJtMeta;
	snprintf(stM.acTest, sizeof(stM.acTest), "%s",
	         bSine ? "sine" : bStep ? "step" : bRamp ? "brk" : (nT == 0 ? "fast" : "slow"));
	snprintf(stM.acLaw, sizeof(stM.acLaw), "%s", (bSine && !stSys.nSineFf) ? "ctc" : s_astMenu[anSel].pcLaw);
	stM.nRun = ++m_anJtRuns[anSel];
	stM.dT = (bStep || bRamp || bSine) ? 0.0 : m_adJtT[nT];
	stM.nSineJoint = bSine ? stSys.nSineJoint : 0;
	stM.dSineAmp = bSine ? stSys.dSineAmp : 0.0;
	stM.dSineFreq = bSine ? stSys.dSineFreq : 0.0;
	stM.nSinePeriods = bSine ? stSys.nSinePeriods : 0;
	stM.dSineV0 = bSine ? stSys.dSineV0 : 0.0;
	stM.nBaJoint = bRamp ? stSys.nBaJoint : 0;
	stM.dBaRate = bRamp ? stSys.dBaRate : 0.0;
	stM.dBaMax = bRamp ? stSys.dBaMax : 0.0;
	stM.dBaQd = bRamp ? stSys.dBaQd : 0.0;
	stM.nBaRep = bRamp ? stSys.nBaRep : 0;
	stM.bFfGate = m_pController->GetFrictionFFGate() ? TRUE : FALSE;
	stM.nStepJoint = bStep ? stSys.nTsJoint : 0;
	stM.dStepTau = bStep ? stSys.dTsTau : 0.0;
	stM.dStepOn = bStep ? stSys.dTsOn : 0.0;
	stM.dStepOff = bStep ? stSys.dTsOff : 0.0;
	stM.nStepRep = bStep ? stSys.nTsRep : 0;
	for (int i = 0; i < stM.nDof; i++)
	{
		stM.adDq[i] = (bStep || bRamp || bSine) ? 0.0 : m_adJtDq[nT][i];
		stM.adB[i] = m_pController->GetRotorInertiaAxis((UINT)i);
		m_pController->GetControlGain((UINT)i, &stM.adKp[i], &stM.adKd[i]);
		stM.adKf[i] = (s_astMenu[anSel].eLaw == C::eJtCTCFF && !(bSine && !stSys.nSineFf)) ?
		              m_pController->GetFrictionFFAxis((UINT)i) : 0.0;
		stM.adTauMax[i] = m_pController->GetTorqueLimit((UINT)i);
	}
	m_stJtRt0 = astState.stFaults;
	m_nJtCtrlFaults0 = m_nCtrlFaults.load(std::memory_order_relaxed);   // the Dynamics task: its own count
	m_nJtDropouts0 = m_nDriveDropouts.load(std::memory_order_acquire);  // the EtherCAT task's count
	m_uJtAllocs0 = m_uAllocUpdate;
	m_cJtLog.Open(stM);
	if (bRamp)
		printf("[JT] start: %s, run %d, J%d, ramp %.2f N m/s up to %.1f N m, moved above %.3f rad/s or %.0f mrad, "
		       "%d ramps each way%s - 'x' stops\n", s_astMenu[anSel].pcName, stM.nRun, stSys.nBaJoint, stSys.dBaRate,
		       stSys.dBaMax, stSys.dBaQd, 1e3 * CControllerFullDynamicsRT::BA_MOVE_RAD, stSys.nBaRep,
		       m_cJtLog.IsStarted() ? "" : " (not logged)");
	else if (bSine)
	{
		// the reach of the sweep: v0 (Tf/2 + Tc + 2 Tf/pi) + A, Tf two periods, Tc (N - 8)/2 periods
		const double dTf = 2.0 / stSys.dSineFreq, dTc = (stSys.nSinePeriods - 8) / (2.0 * stSys.dSineFreq);
		const double dReach = stSys.dSineAmp + (stSys.dSineV0 > 0.0 ? stSys.dSineV0 * (dTf / 2.0 + dTc + 2.0 * dTf / M_PI)
		                                                             : 0.0);
		printf("[JT] start: %s, run %d, J%d, %.3f rad at %.2f Hz on a sweep of +-%.2f rad/s, %d periods "
		       "(sine peak %.2f rad/s, %.1f rad/s^2; reach %.3f rad from q0)%s - 'x' stops\n", s_astMenu[anSel].pcName,
		       stM.nRun, stSys.nSineJoint, stSys.dSineAmp, stSys.dSineFreq, stSys.dSineV0,
		       stSys.nSinePeriods, stSys.dSineAmp * 2.0 * M_PI * stSys.dSineFreq,
		       stSys.dSineAmp * 4.0 * M_PI * M_PI * stSys.dSineFreq * stSys.dSineFreq, dReach,
		       m_cJtLog.IsStarted() ? "" : " (not logged)");
	}
	else if (bStep)
		printf("[JT] start: %s, run %d, J%d, step %.2f N m, on %.2f s, off %.2f s, %d steps%s - 'x' stops\n",
		       s_astMenu[anSel].pcName, stM.nRun, stSys.nTsJoint, stSys.dTsTau, stSys.dTsOn, stSys.dTsOff,
		       stSys.nTsRep, m_cJtLog.IsStarted() ? "" : " (not logged)");
	else
		printf("[JT] start: %s, run %d, T %.1f s, dwell %.1f s%s - 'x' stops\n", s_astMenu[anSel].pcName,
		       stM.nRun, stM.dT, m_dJtDwell, m_cJtLog.IsStarted() ? "" : " (not logged)");
	return TRUE;
}

void
CRobotIndy7::StopServices()
{
	// the bridge after the real-time tasks, the only callers of its RT methods
	if (m_pBridge != NULL)
	{
		m_pBridge->DeInit();
		delete m_pBridge;
		m_pBridge = NULL;
	}
	m_cView.Stop();
	m_cJtLog.Stop();
	m_cHlLog.Stop();
	m_cTimingLog.Stop();      // after the EtherCAT task has ended
}

// -t DIR: the header of the timing log (key=value lines, read by
// tools/timing_summary.py), then the writer. Before the tasks.
void
CRobotIndy7::StartTimingLog()
{
	const VEC_TASK_CONF& vTasks = m_pConfig->Tasks();
	const BOOL bSim = m_pConfig->System().bSim;
	const INT32 nPeriod = vTasks[0].nPeriod;
	char acHost[64] = "";
	gethostname(acHost, sizeof(acHost) - 1);
	struct utsname stU;
	memset(&stU, 0, sizeof(stU));
	uname(&stU);
	char acCmd[640] = "";
	FILE* fp = fopen("/proc/cmdline", "r");
	if (fp != NULL)
	{
		if (fgets(acCmd, sizeof(acCmd), fp) == NULL)
			acCmd[0] = '\0';
		fclose(fp);
		acCmd[strcspn(acCmd, "\n")] = '\0';
	}
	char acStamp[32];
	const time_t tNow = time(NULL);
	struct tm stTm;
	localtime_r(&tNow, &stTm);
	strftime(acStamp, sizeof(acStamp), "%Y-%m-%d %H:%M:%S %Z", &stTm);

	char acLine[768];
	TSTRING strHead = "format=g2t-timing-1\nrecord_bytes=60\n"
	                  "fields=period,w_ns,s_ns,e_ns,dyn_ns,hl_ns,sim_ns,sync_ns,flags,reserved,recv_ns,send_ns,"
	                  "traj_w_ns,traj_e_ns,dyn_w_ns,dyn_e_ns\n"
	                  "flag_bits=OP:1,WKC:2,LATE:4,MISSED:8,OVERRUN:16,LAW:32,HL:64,TRACK:128,STOP:256,"
	                  "SIM:512,BUS:1024,SERVO:2048,BUSSERVO:4096,TEST:8192\n";
	snprintf(acLine, sizeof(acLine), "mode=%s\nperiod_ns=%d\ndeadline_ns=%llu\ncpu=%d\npriority=%d\n",
	         m_bHybridBus ? "hybrid" : (bSim ? "simulation" : "robot"), (int)nPeriod,
	         (unsigned long long)(m_stRt.tDeadline > 0 ? m_stRt.tDeadline : (RTTIME)nPeriod),
	         (int)vTasks[0].nCpu, (int)vTasks[0].nPriority);
	strHead += acLine;
	// the tasks of the pipeline: name:priority:cpu of every periodic task, in the order of the cfg
	strHead += "tasks=";
	for (size_t n = 0, nAdded = 0; n < vTasks.size(); n++)
	{
		if (vTasks[n].nPeriod == 0 || !vTasks[n].bEnabled)
			continue;
		TSTRING strName = vTasks[n].strName;
		for (size_t k = 0; k < strName.size(); k++)
			if (strName[k] == ' ' || strName[k] == ',' || strName[k] == ':' || strName[k] == '\n')
				strName[k] = '_';
		snprintf(acLine, sizeof(acLine), "%s%s:%d:%d", nAdded ? "," : "", strName.c_str(),
		         (int)vTasks[n].nPriority, (int)vTasks[n].nCpu);
		strHead += acLine;
		nAdded++;
	}
	strHead += "\n";
	if (m_pcEcatMaster != NULL)
	{
		const ST_CONFIG_ECAT_MASTER& stM = m_pConfig->Master();
		snprintf(acLine, sizeof(acLine), "bus_slaves=%u\nsync_mode=%s\nsync0_shift_ns=%d\n",
		         m_pcEcatMaster->GetSlaveCnt(), CConfig::SyncModeName(stM.nSyncMode),
		         stM.nSyncMode == eSyncDc ? (int)stM.nDcSync0Shift : 0);
		strHead += acLine;
	}
	snprintf(acLine, sizeof(acLine), "cfg=%s\nhost=%s\nkernel=%s\ncreated=%s\ncmdline=%s\ncall_timing=%d\n",
	         m_pConfig->Path().c_str(), acHost, stU.release, acStamp, acCmd, call_timing::g_bOn ? 1 : 0);
	strHead += acLine;
	m_cTimingLog.Start(m_strTimingDir.c_str(), strHead);
}

BOOL
CRobotIndy7::IsAnyBusServoOn() const
{
	for (size_t i = 0; i < m_vDrives.size(); i++)
		if (m_vDrives[i]->IsServoOn())
			return TRUE;
	return FALSE;
}

// the drives, on the command word of the Dynamics task (DoInput: 'h', 'j',
// 'e', 'x'); EtherCAT task, before the send. The interlocks of the commands
// (controller enabled, gravity compensation) were checked where the key was
// read. With -b the drives of the bus are not joints of the arm: they stay off.
void
CRobotIndy7::ApplyServoCommand(int anCmd)
{
	switch (anCmd)
	{
		case task_slots::eServoCmdOn:
			for (CJoint* p : m_vJoints)
				p->ServoOn();
			break;
		case task_slots::eServoCmdOff:
			for (CJoint* p : m_vJoints)
				p->ServoOff();
			break;
		case task_slots::eServoCmdStop:
			for (CJoint* p : m_vJoints)
				p->StopAxis();
			break;
		default:
			break;
	}
}

// a drive dropout (EtherCAT task, before the send; see Indy7Ctrl.h): every
// joint servo off, which also clears the on-request of a drive that is
// already off, so that none enables itself again when the E-stop is
// released; then the counts, and the stop of the motion, which the Dynamics
// task makes in this period. One message per dropout: the detection rests
// until the next 'h'.
void
CRobotIndy7::DriveDropout(uint32_t auLost, uint32_t auPeriod)
{
	for (CJoint* p : m_vJoints)
		p->ServoOff();
	if (m_nDriveDropouts.load(std::memory_order_relaxed) == 0)
		m_uDropoutFirst = auPeriod;
	m_uDropoutLast = auPeriod;
	m_nDriveDropouts.fetch_add(1, std::memory_order_acq_rel);
	m_nDriveStop.store(1, std::memory_order_release);
	char acJ[64] = "";
	int nLen = 0;
	for (int i = 0; i < GetTotalAxis() && i < 32 && nLen < (int)sizeof(acJ) - 8; i++)
		if (auLost & (1u << i))
			nLen += snprintf(acJ + nLen, sizeof(acJ) - (size_t)nLen, "%sJ%d", nLen > 0 ? " " : "", i + 1);
	DBG_LOG_ERROR(">>> DRIVE DROPOUT (period %u): %s left Operation enabled during servo on (E-stop, or a fault "
	              "of the drive) - SERVO OFF (all joints, brakes), motion stopped; no drive enables by itself: "
	              "clear the cause (release the E-stop), then 'h'", (unsigned)auPeriod, acJ);
}

// a servo on that has not brought every joint to Operation enabled within
// SERVO_ON_TIMEOUT_MS (EtherCAT task, before the send): withdrawn as a
// dropout is, so that it cannot wait for the E-stop to be released and
// enable the drives then. The joints that are missing, with their 0x6041
// (Switch on disabled ..40/..50, Ready ..21/..31, Fault ..08/..18).
void
CRobotIndy7::ServoOnExpired(uint32_t auMissing, const uint16_t* apusStatus, uint32_t auPeriod)
{
	for (CJoint* p : m_vJoints)
		p->ServoOff();
	m_nServoExpired++;
	m_nDriveStop.store(1, std::memory_order_release);
	char acJ[160] = "";
	int nLen = 0;
	for (int i = 0; i < GetTotalAxis() && i < 32 && nLen < (int)sizeof(acJ) - 24; i++)
		if (auMissing & (1u << i))
			nLen += snprintf(acJ + nLen, sizeof(acJ) - (size_t)nLen, "%sJ%d (0x%04x)", nLen > 0 ? " " : "", i + 1,
			                 (unsigned)apusStatus[i]);
	DBG_LOG_ERROR(">>> SERVO ON NOT COMPLETE after %d ms (period %u): %s not in Operation enabled - the request "
	              "is withdrawn, SERVO OFF (all joints); clear the cause (E-stop, a fault of the drive), then 'h'",
	              m_nServoOnTimeoutMs, (unsigned)auPeriod, acJ);
}

// the timing of the tasks, printed when they have ended
void
CRobotIndy7::PrintRtSummary() const
{
	const RtStats& st = m_stRt;
	const double dN = st.nCycles > 0 ? (double)st.nCycles : 1.0;
	printf("[RT] EtherCAT task: %ld cycles of %.3f ms: t_w (release to start) mean %.1f us, max %.1f us; "
	       "t_s (start to send) mean %.1f us, max %.1f us; t_e (start to end) mean %.1f us, max %.1f us\n",
	       st.nCycles, (double)st.tPeriod * 1e-6, st.dWSum / dN * 1e-3, (double)st.tWMax * 1e-3,
	       st.dSSum / dN * 1e-3, (double)st.tSMax * 1e-3, st.dESum / dN * 1e-3, (double)st.tEMax * 1e-3);
	printf("[RT] EtherCAT task faults: %ld late releases (more than a period late), %ld overruns, %ld missed send "
	       "deadlines (t_w + t_s > %.0f us)\n", st.nLate, st.nOverruns, st.nMissed, (double)st.tDeadline * 1e-3);
	// the two other tasks of the pipeline
	const task_slots::ST_TASK_TIMING* apT[2] = { &m_stTrajTiming, &m_stDynTiming };
	const char* apcName[2] = { "Trajectory", "Dynamics" };
	for (int k = 0; k < 2; k++)
	{
		const task_slots::ST_TASK_TIMING& t = *apT[k];
		const double dM = t.nCycles > 0 ? (double)t.nCycles : 1.0;
		printf("[RT] %s task: %ld cycles of %.3f ms: t_w mean %.1f us, max %.1f us; t_e mean %.1f us, max %.1f us; "
		       "%ld late releases, %ld overruns\n", apcName[k], t.nCycles, (double)t.tPeriod * 1e-6,
		       t.dWSum / dM * 1e-3, (double)t.tWMax * 1e-3, t.dESum / dM * 1e-3, (double)t.tEMax * 1e-3, t.nLate,
		       t.nOverruns);
	}
	printf("[RT] pipeline: %ld periods without a fresh torque while OP (the last target stayed), %ld periods with a "
	       "reference too old to follow, %ld references refused during a JOINT TEST\n",
	       m_nTorqueStale, m_nRefStale, m_nRefRefused);
	if (m_pcEcatMaster != NULL)
	{
		if (m_pcEcatMaster->IsDc())
			printf("[RT] incomplete working counters: %ld; DC: largest system-time difference %u ns (while OP)\n",
			       st.nWkc, st.uSyncDiffMax);
		else
			printf("[RT] incomplete working counters: %ld\n", st.nWkc);
	}
	// the controller faults (TaskDynamics.cpp): latches, and the cycles in
	// which the law failed (zero torque)
	const long nCtrlFaults = m_nCtrlFaults.load(std::memory_order_acquire);
	if (nCtrlFaults > 0)
		printf("[RT] controller faults: %ld (first at period %u, last at period %u); the law failed in %ld cycle%s "
		       "(non-finite state or torque: zero torque, the drives off)\n", nCtrlFaults,
		       (unsigned)m_uCtrlFaultFirst, (unsigned)m_uCtrlFaultLast, m_nCtrlFaultCycles,
		       m_nCtrlFaultCycles == 1 ? "" : "s");
	else
		printf("[RT] controller faults: 0\n");
	// the drive dropouts and the servo ons (TaskEtherCAT.cpp)
	const long nDropouts = m_nDriveDropouts.load(std::memory_order_acquire);
	if (nDropouts > 0)
		printf("[RT] drive dropouts: %ld (first at period %u, last at period %u): a drive left Operation enabled "
		       "during servo on, every joint went off\n", nDropouts, (unsigned)m_uDropoutFirst,
		       (unsigned)m_uDropoutLast);
	else
		printf("[RT] drive dropouts: 0\n");
	printf("[RT] servo on: %ld completed, the longest in %.0f ms ('h' to every joint in Operation enabled); %ld "
	       "refused (a drive in Switch on disabled), %ld withdrawn (not complete after %d ms)\n", m_nServoOnDone,
	       (double)m_tServoOnMax * 1e-6, m_nServoRefused, m_nServoExpired, m_nServoOnTimeoutMs);
	if (malloc_count::Enabled())
		printf("[MALLOC] control law (Update): %lu heap allocations in %lu cycles (at most %lu in one cycle); "
		       "whole cycle of the Dynamics task: %lu; EtherCAT task: %lu; Trajectory task: %lu\n",
		       m_uAllocUpdate, m_uAllocCycles, m_uAllocUpdateMaxCycle, m_uAllocCycle, m_uAllocEcat, m_uAllocTraj);
	// call timing: the table, and with -t the file next to the timing log
	if (call_timing::g_bOn)
	{
		printf("[CALLS] time of the library calls of the control law (G2T_CALL_TIMING):\n");
		call_timing::Print(stdout);
		TSTRING strPath = m_cTimingLog.GetPath();
		const size_t uSlash = strPath.rfind('/');
		const size_t uPos = strPath.find("timing_", uSlash == TSTRING::npos ? 0 : uSlash);
		if (uPos != TSTRING::npos && strPath.size() > 4 && strPath.compare(strPath.size() - 4, 4, ".bin") == 0)
		{
			strPath.replace(strPath.size() - 4, 4, ".txt");
			strPath.replace(uPos, 7, "calls_");
			FILE* fp = fopen(strPath.c_str(), "w");
			if (fp != NULL)
			{
				fprintf(fp, "# call timing of the control law and the high-level controller (CallTiming.h), "
				        "%ld periods of the EtherCAT task\n", st.nCycles);
				call_timing::Print(fp);
				fclose(fp);
				printf("[CALLS] -> %s\n", strPath.c_str());
			}
			// the histograms next to it (calls_<stamp>.hist)
			strPath.replace(strPath.size() - 4, 4, ".hist");
			fp = fopen(strPath.c_str(), "w");
			if (fp != NULL)
			{
				call_timing::PrintHist(fp);
				fclose(fp);
				printf("[CALLS] -> %s\n", strPath.c_str());
			}
		}
	}
}

BOOL
CRobotIndy7::InitController()
{
	const ST_CONFIG_SYSTEM& stSys = m_pConfig->System();
	const unsigned int dof = (unsigned int)stSys.nRobotAxis;
	if (dof == 0)
	{
		DBG_LOG_ERROR("(CRobotIndy7) cannot initialize the controller: ROBOT_DOF = 0");
		return FALSE;
	}
	m_pController = new CControllerFullDynamicsRT(stSys.strURDFPath, dof);
	m_pController->EnableRTMode(TRUE);
	m_pController->SetDeadline(500000);    // 500 us

	// the controller starts in gravity compensation; the motions (JOINT
	// TEST, TRACK, HOME) start from there on a command, when their start
	// conditions hold
	m_pController->SetControlMode(CControllerFullDynamicsRT::eGravityCompensation);
	if (!m_pController->Init())
	{
		DBG_LOG_ERROR("(CRobotIndy7) cannot initialize the controller (URDF %s)", stSys.strURDFPath.c_str());
		delete m_pController;
		m_pController = NULL;
		return FALSE;
	}

	// the gains of eq. (5) (Table 7)
	std::vector<double> Kp(dof), Kd(dof);
	for (unsigned int i = 0; i < dof; i++)
	{
		Kp[i] = stSys.vKp[i];
		Kd[i] = stSys.vKd[i];
	}
	m_pController->SetControlGains(Kp, Kd);

	// the friction feedforward K_f of eq. (7) [N m] (KF_n; 0: off), the rotor
	// inertias B (ROTOR_INERTIA_n [kg m^2], eq. (4)), and the gate of eq. (7)
	{
		std::vector<double> Kf(dof, 0.0);
		for (unsigned int i = 0; i < dof && i < stSys.vKf.size(); i++)
			Kf[i] = stSys.vKf[i];
		m_pController->SetFrictionFF(Kf);
		std::vector<double> vB(dof, 0.0);
		for (unsigned int i = 0; i < dof && i < stSys.vRotorB.size(); i++)
			vB[i] = stSys.vRotorB[i];
		m_pController->SetRotorInertia(vB);
		if (dof == 6)
			printf("[B] rotor inertias [kg m^2]: %.5f %.5f %.5f %.5f %.5f %.5f%s\n", vB[0], vB[1], vB[2], vB[3],
			       vB[4], vB[5], vB[5] > 0.0 ? "" : " (J6: none - J6 cannot follow a reference)");
		m_pController->SetFrictionFFGate(stSys.nFfGate != 0);
		if (stSys.nFfGate == 0)
			printf("[FF] FRICTION_FF_GATE=0: the friction feedforward acts WITHOUT its gate (ablation)\n");
	}

	// the torque limit tau_max per joint: TORQUE_LIMIT_U of the joint drives,
	// in the order of the bus
	{
		std::vector<double> vTauMax;
		const VEC_ECAT_SLAVE_CONF& vSlaves = m_pConfig->Slaves();
		for (size_t i = 0; i < vSlaves.size() && vTauMax.size() < dof; i++)
			if (vSlaves[i].nSlaveType == 0)
				vTauMax.push_back(vSlaves[i].dTorLimitU);
		vTauMax.resize(dof, 0.0);
		m_pController->SetTorqueLimits(vTauMax);
	}

	// the vectors of the Dynamics task
	m_vCurrentPos.resize(dof, 0.0);
	m_vCurrentVel.resize(dof, 0.0);
	m_vCurrentTor.resize(dof, 0.0);
	m_vOutputTorque.resize(dof, 0.0);

	DBG_LOG_INFO("(CRobotIndy7) controller initialized, DOF %u", dof);
	DBG_LOG_INFO("(CRobotIndy7) gains Kp: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]", Kp[0], Kp[1], Kp[2], Kp[3], Kp[4],
	             Kp[5]);
	DBG_LOG_INFO("(CRobotIndy7) gains Kd: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]", Kd[0], Kd[1], Kd[2], Kd[3], Kd[4],
	             Kd[5]);

	if (TRUE == stSys.bEnableControllerAtStartup)
		EnableController(TRUE);
	return TRUE;
}

// the high-level controller of the Trajectory task (TRACK, HOME; paper,
// Sec. IV-C, Table 9): parameters from the cfg, a model instance of its own,
// and the goal slot that it reads. Its period is that of the Trajectory task
// (TASK1).
BOOL
CRobotIndy7::InitTrajectory()
{
	const ST_CONFIG_SYSTEM& stS = m_pConfig->System();
	const unsigned int dof = (m_pController != NULL) ? m_pController->GetDOF() : 0;
	const double dTs = (double)m_pConfig->Tasks()[1].nPeriod * 1e-9;

	CHighLevelRT::Params stP = CHighLevelRT::DefaultParams();
	stP.dDelta     = stS.dHlDelta;
	stP.dLambda    = stS.dHlLambda;
	stP.dStepMax   = stS.dHlStepMax;
	stP.dTol       = stS.dHlTol;
	stP.nIterCycle = stS.nHlIterCycle;
	stP.nIterMax   = stS.nHlIterGoal;
	stP.dVelMax    = stS.dHlVelMax;
	stP.dAccMax    = stS.dHlAccMax;
	stP.dJerkMax   = stS.dHlJerkMax;
	stP.dMargin    = stS.dHlMargin;
	stP.dLead      = stS.dHlLead;
	stP.bStepRef   = (stS.nHlStepBaseline != 0);
	for (int i = 0; i < CHighLevelRT::DOF && i < (int)stS.vHlReadyQ.size(); i++)
		stP.adQr[i] = stS.vHlReadyQ[i];
	if (dof != (unsigned int)CHighLevelRT::DOF || FALSE == m_cTraj.Init(stS.strURDFPath.c_str(), dTs, stP))
		DBG_LOG_WARN("(CRobotIndy7) high-level controller unavailable - TRACK and HOME are refused");
	m_cTraj.SetGoalSlot(&m_goalSlot);
	return TRUE;
}

BOOL
CRobotIndy7::EnableController(BOOL abEnable)
{
	if (m_pController == NULL)
	{
		DBG_LOG_ERROR("(CRobotIndy7) the controller is not initialized");
		return FALSE;
	}
	m_pController->Enable(abEnable);
	m_bRTControllerEnabled = abEnable;
	DBG_LOG_INFO("(CRobotIndy7) controller %s", abEnable ? "ENABLED" : "DISABLED");
	// the controller off ('r') clears a controller fault and ends the fault
	// injection: the operator has acted on it. The drives can then come on
	// again only after 'r' (on) and 'h'; should the law fail in its first
	// cycle, the fault latches again (TaskDynamics.cpp).
	if (!abEnable)
	{
		m_nStateInject = 0;
		m_pController->ClearFaultInjection();
		if (m_bCtrlFault.exchange(false, std::memory_order_acq_rel))
			DBG_LOG_WARN(">>> controller fault cleared (controller off): 'r' switches it on, then 'h'");
	}
	return TRUE;
}

BOOL
CRobotIndy7::SetControllerMode(CControllerFullDynamicsRT::eControlMode aeMode)
{
	if (m_pController == NULL)
	{
		DBG_LOG_ERROR("(CRobotIndy7) the controller is not initialized");
		return FALSE;
	}
	m_pController->SetControlMode(aeMode);
	const char* mode_names[] = {"Gravity Compensation", "JOINT TEST", "TRACK", "HOME"};
	if (aeMode < 4)
		DBG_LOG_INFO("(CRobotIndy7) mode: %s", mode_names[aeMode]);
	return TRUE;
}

// the bus of the robot: the joint drives and the end tool in the order of the
// cfg ([AXIS0]..). With -b the joint drives are not joints of the arm.
BOOL
CRobotIndy7::InitEtherCAT()
{
	const ST_CONFIG_ECAT_MASTER& stM = m_pConfig->Master();
	const VEC_ECAT_SLAVE_CONF& vSl = m_pConfig->Slaves();
	m_pcEcatMaster = new CEcatMaster();
	for (size_t n = 0; n < vSl.size(); n++)
	{
		if (vSl[n].nSlaveType == 0)
		{
			CIndy7Joint* pJ = new CIndy7Joint(vSl[n]);
			m_vDrives.push_back(pJ);
			m_pcEcatMaster->AddSlave(pJ);
		}
		else
		{
			m_pEndTool = new CIndy7EndTool(vSl[n]);
			m_pcEcatMaster->AddSlave(m_pEndTool);
		}
	}
	if (FALSE == m_pcEcatMaster->Init((UINT32)stM.nCycleTime, stM.nSyncMode, stM.nDcSync0Shift))
	{
		DBG_LOG_ERROR("(CRobotIndy7) cannot initialize the EtherCAT master");
		return FALSE;
	}
	if (!m_bHybridBus)
		for (CIndy7Joint* p : m_vDrives)
			m_vJoints.push_back(p);
	else
		DBG_LOG_INFO("(CRobotIndy7) -b: %u joint drives and %s of the robot; the drives stay off: OP, power stage "
		             "off, brakes engaged", (unsigned)m_vDrives.size(), m_pEndTool ? "the end tool" : "no end tool");
	return TRUE;
}

// the simulation in place of the bus (paper, Sec. IV and V): the same
// joints, simulated (CSimJoint), and CSimArm (MuJoCo) in place of the bus;
// with -b also the bus of the robot (InitEtherCAT)
BOOL
CRobotIndy7::InitSimulation()
{
	const ST_CONFIG_SYSTEM& stSys = m_pConfig->System();
	const VEC_ECAT_SLAVE_CONF& vSl = m_pConfig->Slaves();
	for (size_t n = 0; n < vSl.size(); n++)
		if (vSl[n].nSlaveType == 0)
		{
			CSimJoint* p = new CSimJoint(vSl[n].strName, vSl[n].nDriveMode);
			m_vSimJoints.push_back(p);
			m_vJoints.push_back(p);
		}
	const int nDof = GetTotalAxis();
	if (nDof != stSys.nRobotAxis || nDof > CSimArm::MAX_DOF)
	{
		DBG_LOG_ERROR("(CRobotIndy7) %d simulated joints, ROBOT_DOF=%d", nDof, (int)stSys.nRobotAxis);
		return FALSE;
	}
	const double dPeriod = (double)m_pConfig->Tasks()[0].nPeriod * 1e-9;
	m_pSim = new CSimArm();
	if (FALSE == m_pSim->Init(stSys.strSimMJCFPath.c_str(), nDof, stSys.nSimStateDelay, stSys.nSimTorqueDelay,
	                          stSys.vSimInitialQ.data(), stSys.bSimFriction, stSys.vSimTauS.data(), dPeriod))
		return FALSE;
	// the rotor inertias as the armature of the simulated arm
	if ((int)stSys.vSimArmature.size() >= nDof && FALSE == m_pSim->SetArmature(stSys.vSimArmature.data(), nDof))
		return FALSE;

	/* the controller sees the initial posture from its first cycle on */
	double adQ[CSimArm::MAX_DOF], adQd[CSimArm::MAX_DOF], adTau[CSimArm::MAX_DOF];
	m_pSim->GetMeasured(adQ, adQd, adTau);
	for (int i = 0; i < nDof; i++)
		m_vSimJoints[i]->SetSimState(adQ[i], adQd[i], adTau[i]);

	// -b: the bus of the robot beside the simulated arm
	if (TRUE == m_bHybridBus && FALSE == InitEtherCAT())
		return FALSE;
	return TRUE;
}

// Stop every motion (see Indy7Ctrl.h). Runs in the Dynamics task (DoInput):
// flag stores, one change of mode, which ends a running JOINT TEST, and the
// stop word for the Trajectory task, which ends TRACK or HOME, the running
// high-level test, and an operator motion (TaskTrajectory.cpp, StopHighLevel).
// Until the Trajectory task has acknowledged (eRefNone), the Dynamics task
// ignores the Reference slot: the stop acts in the period of the key.
void
CRobotIndy7::StopMotion()
{
	m_bJtMenu = false;
	m_bHlMenu = false;
	m_nHlCmd.store(task_slots::HL_CMD_STOP, std::memory_order_release);
	m_bRefInhibit = TRUE;
	if (m_pController != NULL)
		SetControllerMode(CControllerFullDynamicsRT::eGravityCompensation);
}

void
CRobotIndy7::PrintHelp() const
{
	printf("[KEYS] r controller on/off   g gravity compensation   h servo on   j servo off\n"
	       "       x stop (hold in gravity compensation)   e e-stop (servo off, brakes)   q quit\n"
	       "       s print the tool pose   w arm/disarm the pose capture of the hand-eye calibration\n"
	       "       k hold in gravity compensation on/off\n"
	       "[TOOLS] m joint move: absolute joint angles [deg]   t task move: tool position x y z [mm]\n"
	       "       p camera application: the objects in view, a digit follows one\n"
	       "       u joint tests and identification (1-7)   y high-level tests (1-8)\n"
	       "       ? this help%s\n", m_nSimFaultJoint >= 0 ? "   n fault injection (simulation)" : "");
}

// the operator commands, in the Dynamics task after its torque is published.
// The controller and the JOINT TEST are its own; the drives go to the
// EtherCAT task (m_nServoCmd), the high-level motions to the Trajectory task
// (m_nHlCmd); the state of the arm comes from its copy of the State slot
// (m_stDynState), that of the Trajectory task from the Reference slot.
void
CRobotIndy7::DoInput()
{
	using namespace task_slots;
	const char cKey = m_cKeyPress.exchange(' ');
	// JOINT TEST: the key after 'u' selects the test; while a test runs, only
	// the stops act ('x' stop, 'g' float, 'e' e-stop, 'j' servo off - each
	// ends the test)
	if (m_bJtMenu && cKey != ' ')
	{
		m_bJtMenu = false;
		if (cKey >= '1' && cKey < '1' + JT_MENU)
			StartJointTestMenu(cKey - '1', m_stDynState);
		else
			printf("[JT] menu closed\n");
		return;
	}
	// the key after 'y' selects a high-level test: the Trajectory task starts
	// it (its start conditions are checked there)
	if (m_bHlMenu && cKey != ' ')
	{
		m_bHlMenu = false;
		if (cKey >= '1' && cKey < '1' + HL_MENU)
			m_nHlCmd.store(cKey - '0', std::memory_order_release);
		else
			printf("[HL] menu closed\n");
		return;
	}
	// a digit for the object menu of the camera application ('p'), also
	// while the application runs
	if (m_pBridge != NULL && cKey != ' ' && m_pBridge->FeedDigit(cKey))
		return;
	const bool bHlTest = (m_stDynRef.bTest != FALSE);
	if (m_pController != NULL && cKey != ' ' && (m_pController->IsJointTestRunning() || bHlTest))
	{
		const bool bAppMenu = (cKey == 'p' || cKey == 'P') && bHlTest && m_stDynRef.nHlTest == HL_APP;
		switch (cKey)
		{
			case 'x': case 'X': case 'g': case 'G': case 'e': case 'E': case 'j': case 'J':
				break;
			case 'n': case 'N':
				// the fault injection acts during a test too (simulation)
				if (m_nSimFaultJoint >= 0)
					break;
				/* fall through */
			default:
				if (bAppMenu)       // the object menu while the application runs
					break;
				printf("[TEST] key '%c' ignored while a motion runs - "
				       "'x' stop, 'g' float, 'e' e-stop, 'j' servo off\n", cKey);
				return;
		}
	}

	switch (cKey)
	{
		// Servo on and off by the operator. Interlock: the drives enable in
		// cyclic torque mode at zero torque, so the controller must already
		// be enabled ('r') and in gravity compensation before the brakes
		// release. The EtherCAT task switches the drives before its next
		// send. A controller fault keeps them off until the controller has
		// been switched off and on ('r', 'r'). After a drive dropout (the
		// E-stop) 'h' enables them again, but not while a drive stays in
		// Switch on disabled: Shutdown is written in every period while a
		// drive is off, which brings a drive with power to Ready to switch
		// on; one that stays below is held off by the E-stop (or another
		// inhibit), and a request now would wait and enable it when the E-stop
		// is released (TaskEtherCAT.cpp). A request that does not complete
		// within SERVO_ON_TIMEOUT_MS is withdrawn there.
		case 'h':
		case 'H':
			if (m_bCtrlFault.load(std::memory_order_acquire))
			{
				printf("[ARM] refused - controller fault latched; press 'r' (off), 'r' (on), then 'h'\n");
				break;
			}
			if (m_stDynState.uNotReady != 0)
			{
				char acJ[160] = "";
				int nLen = 0;
				for (int i = 0; i < m_stDynState.nDof && i < 32 && nLen < (int)sizeof(acJ) - 24; i++)
					if (m_stDynState.uNotReady & (1u << i))
						nLen += snprintf(acJ + nLen, sizeof(acJ) - (size_t)nLen, "%sJ%d (0x%04x)",
						                 nLen > 0 ? " " : "", i + 1, (unsigned)m_stDynState.ausStatus[i]);
				m_nServoRefused++;
				printf("[ARM] refused - %s in Switch on disabled: E-stop pressed (or the drive inhibited)? "
				       "Release it, then 'h'\n", acJ);
				break;
			}
			if (!m_bRTControllerEnabled ||
			    m_pController->GetControlMode() != CControllerFullDynamicsRT::eGravityCompensation)
			{
				printf("[ARM] refused - press 'r' (controller) then 'g' (grav-comp) first\n");
				break;
			}
			DBG_LOG_WARN(">>> SERVO ON (all joints, CST) - brakes release, grav-comp holds");
			m_nServoCmd.store(eServoCmdOn, std::memory_order_release);
			break;
		case 'j':
		case 'J':
			// a motion does not go on against the brakes
			if (m_pController != NULL &&
			    (m_pController->IsJointTestRunning() || m_stDynRef.eMode != eRefNone || bHlTest))
				StopMotion();
			DBG_LOG_WARN(">>> SERVO OFF (all joints) - brakes engage");
			m_nServoCmd.store(eServoCmdOff, std::memory_order_release);
			break;
		// The stops. A quick stop of the drives alone does not stop this
		// robot: the CiA 402 state machine of the drive re-arms out of QUICK
		// STOP while the controller still commands its motion. Both stops
		// therefore end the motion first (StopMotion); then
		//   'e' latches servo off, which the state machine cannot undo: the
		//       motors de-energize and the brakes engage; re-arm with 'h'.
		//   'x' keeps the servo on and holds in gravity compensation.
		case 'e':
		case 'E':
			DBG_LOG_WARN(">>> E-STOP: motion stopped, SERVO OFF (brakes)");
			StopMotion();
			m_nServoCmd.store(eServoCmdOff, std::memory_order_release);
			break;
		case 'x':
		case 'X':
			DBG_LOG_WARN(">>> STOP: motion stopped - holding in grav-comp");
			StopMotion();
			// controller not running: the stop of the drives is all that is left
			if (!m_bRTControllerEnabled || m_pController == NULL || !m_pController->IsEnabled())
				m_nServoCmd.store(eServoCmdStop, std::memory_order_release);
			break;
		case 'r':
		case 'R':
			DBG_LOG_INFO(">>> controller: %s", m_bRTControllerEnabled ? "DISABLED" : "ENABLED");
			EnableController(!m_bRTControllerEnabled);   // off: clears a controller fault
			break;
		// fault injection (cfg SIM_FAULT_*, simulation only, refused at Init
		// elsewhere): the next cycle of the law, or every one until 'r'
		// switches the controller off, gets a non-finite torque or measured
		// angle on one joint; or, the drive-dropout test, the EtherCAT task
		// takes the joint out of Operation enabled in the next period; or,
		// the E-stop tests, it cuts or restores the power of every drive in
		// the next period
		case 'n':
		case 'N':
			if (m_nSimFaultJoint < 0 || m_pSim == NULL || m_pcEcatMaster != NULL)
			{
				printf("[FAULT] no fault injection (SIM_FAULT_JOINT=0, or not the simulation)\n");
				break;
			}
			if (m_nSimFaultWhat == 3)
			{
				m_bSimEstop = !m_bSimEstop;
				m_nSimEstopCmd.store(m_bSimEstop ? 1 : 2, std::memory_order_release);
				printf("[FAULT] E-stop %s (simulated): %s\n", m_bSimEstop ? "PRESSED" : "RELEASED",
				       m_bSimEstop ? "every drive loses its power ('n' releases it)" :
				                     "the power is back; a drive still requested on would enable by itself");
				break;
			}
			if (m_nSimFaultWhat == 2)
			{
				if (!m_stDynState.bServoOn)
				{
					printf("[FAULT] refused - servo OFF ('h' first): the dropout takes an enabled drive out of "
					       "Operation enabled\n");
					break;
				}
				m_nSimDropout.store(m_nSimFaultJoint + 1, std::memory_order_release);
				printf("[FAULT] injected: J%d leaves Operation enabled in the next period (the drive-dropout "
				       "test)\n", m_nSimFaultJoint + 1);
				break;
			}
			if (!m_bRTControllerEnabled || m_pController == NULL)
			{
				printf("[FAULT] refused - the controller is off ('r')\n");
				break;
			}
			if (m_nSimFaultWhat == 1)
				m_nStateInject = m_bSimFaultPersist ? 2 : 1;
			else
				m_pController->InjectTorqueFault(m_nSimFaultJoint, m_bSimFaultPersist ? TRUE : FALSE);
			printf("[FAULT] injected: a non-finite %s of J%d %s\n",
			       m_nSimFaultWhat == 1 ? "measured angle" : "torque", m_nSimFaultJoint + 1,
			       m_bSimFaultPersist ? "in every cycle until 'r' (off)" : "in the next cycle");
			break;
		case 'g':
		case 'G':
			// gravity compensation: the running motion ends (JOINT TEST here,
			// TRACK or HOME in the Trajectory task)
			DBG_LOG_INFO(">>> controller: gravity compensation");
			StopMotion();
			break;
		case 'w':
		case 'W':
			// arm or disarm the hand-eye capture ('s')
			m_bCalibMode = !m_bCalibMode;
			if (m_bCalibMode)
				printf("\n================= CALIB CAPTURE: ON =================\n"
				       " 's' now WRITES %s/ :\n"
				       "   robot_poses.csv     (capture #1 TRUNCATES it)\n"
				       "   pose_rPe_<N>.yaml   next N = %d\n"
				       " 'w' again to disarm.\n"
				       "=====================================================\n\n",
				       m_cCalib.GetDir().c_str(), m_cCalib.GetCount());
			else
				printf("[CALIB] capture DISARMED - 's' only prints the pose\n");
			break;
		// the keyboard task handles 'q', 'm', 't' and '?' itself
		case 's':
		case 'S':
			m_pController->GetCurrentPose(m_Pose);
			// armed ('w'): the only two writers of the dataset
			if (m_bCalibMode)
			{
				SaveRobotPose();
				m_cCalib.Capture(m_Pose);
			}
			printf("%s", m_bCalibMode ? "[TCP Pose] *** CAPTURED ***\n"
			                          : "[TCP Pose] (print only - 'w' arms capture)\n");
			printf("  t = [%.6f, %.6f, %.6f]\n", m_Pose.m_position[0], m_Pose.m_position[1], m_Pose.m_position[2]);
			printf("  R = [%.6f, %.6f, %.6f]\n"
			       "      [%.6f, %.6f, %.6f]\n"
			       "      [%.6f, %.6f, %.6f]\n",
			       m_Pose.m_rotation(0,0), m_Pose.m_rotation(0,1), m_Pose.m_rotation(0,2),
			       m_Pose.m_rotation(1,0), m_Pose.m_rotation(1,1), m_Pose.m_rotation(1,2),
			       m_Pose.m_rotation(2,0), m_Pose.m_rotation(2,1), m_Pose.m_rotation(2,2));
			break;
		case 'k':
		case 'K':
			if (m_pController != NULL)
			{
				const bool bEn = !m_pController->m_bStickyEnable.load();
				m_pController->m_bStickyEnable = bEn;
				DBG_LOG_INFO("[HOLD] grav-comp hold %s (dead-band %.0f mrad)", bEn ? "ON" : "OFF - pure float",
				             CControllerFullDynamicsRT::HOLD_DB_RAD * 1e3);
			}
			break;
		// the object menu of the camera application (paper, Sec. V-C): the
		// worker of the bridge prints the classes in view; a digit selects
		// one, and the arm follows it
		case 'p':
		case 'P':
			if (m_pBridge != NULL)
				m_pBridge->RequestMenu();
			else
				printf("[APP] no bridge (APP_BRIDGE=1 in the cfg, a build with ROS 2)\n");
			break;
		// JOINT TEST menu (paper, Sec. IV-A and V-A)
		case 'u':
		case 'U':
			printf("[JT] JOINT TEST - press 1-%d, any other key closes:\n"
			       "     1 fast, PD + g   2 fast, CTC   3 slow, CTC   4 slow, CTC + FF\n"
			       "     5 torque step (delay test)   6 breakaway (tau_s, sign test)   7 sine (rotor inertia)\n"
			       "     fast: dq [%.2f %.2f %.2f %.2f %.2f %.2f] rad, T %.1f s; "
			       "slow: dq [%.2f %.2f %.2f %.2f %.2f %.2f] rad, T %.1f s; dwell %.1f s\n",
			       (int)JT_MENU,
			       m_adJtDq[0][0], m_adJtDq[0][1], m_adJtDq[0][2], m_adJtDq[0][3], m_adJtDq[0][4], m_adJtDq[0][5],
			       m_adJtT[0],
			       m_adJtDq[1][0], m_adJtDq[1][1], m_adJtDq[1][2], m_adJtDq[1][3], m_adJtDq[1][4], m_adJtDq[1][5],
			       m_adJtT[1], m_dJtDwell);
			m_bJtMenu = true;
			break;
		// high-level tests (paper, Sec. V-B and V-C)
		case 'y':
		case 'Y':
			printf("[HL] high-level tests - press 1-%d, any other key closes:\n"
			       "     1 fixed goal   2 goal jump   3 goal noise (delta)   4 goal noise (delta = 0)\n"
			       "     5 reference check   6 HOME   7 record q_r (this session)\n"
			       "     8 camera application: goals from the camera through the bridge (%s), hover %.2f m\n"
			       "     goal [%.3f %.3f %.3f] m from the tool at q_r; jump [%.3f %.3f %.3f] m at %.2f m; "
			       "noise %.1f mm at %.0f Hz for %.0f s; delta %.1f mm; friction FF %s\n",
			       (int)HL_MENU, m_pBridge != NULL ? "up" : "not running", m_dAppHover,
			       m_adHlGoalOff[0], m_adHlGoalOff[1], m_adHlGoalOff[2],
			       m_adHlJump[0], m_adHlJump[1], m_adHlJump[2], m_dHlTrigger,
			       m_dHlNoise * 1e3, m_dHlNoiseHz, m_dHlNoiseT, m_dHlDelta * 1e3, m_bHlFF ? "on" : "off");
			if (m_nHlGoals > 0)
				printf("     1 runs %d random goals (HL_RAND_*), each from q_r, from goal %d on; 'x' stops\n",
				       m_nHlGoals, (m_nHlGoalIdx < m_nHlGoals ? m_nHlGoalIdx : 0) + 1);
			if (m_nHlJumps > 0)
				printf("     2 runs %d random goal jumps (HL_RAND_JUMP*), each from q_r, from jump %d on; 'x' stops\n",
				       m_nHlJumps, (m_nHlJumpIdx < m_nHlJumps ? m_nHlJumpIdx : 0) + 1);
			m_bHlMenu = true;
			break;
		default:
			break;
	}
}

BOOL
CRobotIndy7::IsMoving()
{
	for (CJoint* p : m_vJoints)
		if (p->IsMoving())
			return TRUE;
	return FALSE;
}

/*************************************************** the keyboard task ***/

namespace {
/* a line from the terminal, with the echo and the editing of the terminal */
bool ReadLine(char* apcBuf, size_t auLen)
{
	struct termios stOld, stNew;
	const bool bTty = (0 == tcgetattr(STDIN_FILENO, &stOld));
	if (bTty)
	{
		stNew = stOld;
		stNew.c_lflag |= (tcflag_t)(ICANON | ECHO);
		tcsetattr(STDIN_FILENO, TCSANOW, &stNew);
	}
	const bool bOk = (fgets(apcBuf, (int)auLen, stdin) != NULL);
	if (bTty)
		tcsetattr(STDIN_FILENO, TCSANOW, &stOld);
	return bOk;
}

/* a line of one letter (a key such as 'x' or 'e' typed at the prompt): the letter, else 0 */
char LineKey(const char* apcLine)
{
	const char* pc = apcLine;
	while (*pc == ' ' || *pc == '\t')
		pc++;
	const char c = *pc;
	if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z')))
		return 0;
	pc++;
	while (*pc == ' ' || *pc == '\t' || *pc == '\r' || *pc == '\n')
		pc++;
	return (*pc == '\0') ? c : 0;
}

/* up to anMax numbers of a line; the count */
int ParseNumbers(const char* apcLine, double* adOut, int anMax)
{
	int n = 0;
	const char* pc = apcLine;
	while (n < anMax)
	{
		while (*pc == ' ' || *pc == '\t' || *pc == ',')
			pc++;
		if (*pc == '\0' || *pc == '\n')
			break;
		char* pcEnd = NULL;
		const double d = strtod(pc, &pcEnd);
		if (pcEnd == pc)
			return -1;
		adOut[n++] = d;
		pc = pcEnd;
	}
	while (*pc == ' ' || *pc == '\t' || *pc == ',' || *pc == '\n')
		pc++;
	return (*pc == '\0') ? n : -1;
}
}

// a key letter typed at the prompt of 'm' or 't': the move is cancelled and
// the key acts ('q' ends the controller; 'm', 't' and '?' only cancel)
void
CRobotIndy7::PassKey(char acKey)
{
	printf("[MOVE] cancelled\n");
	if (acKey == 'q')
		StopTasks();
	else if (acKey != 'm' && acKey != 'M' && acKey != 't' && acKey != 'T' && acKey != '?')
		m_cKeyPress.store(acKey);
}

// 'm': six absolute joint angles [deg], each inside the joint limits of the
// URDF minus the margin of the high-level controller (HL_LIMIT_MARGIN); the
// Trajectory task moves there with the trajectory generator of HOME
BOOL
CRobotIndy7::ReadJointMove()
{
	if (!m_cTraj.IsInit())
	{
		printf("[MOVE] refused - the high-level controller is not available\n");
		return FALSE;
	}
	const CHighLevelRT::Params& stP = m_cTraj.HighLevel().GetParams();
	const int nDof = CHighLevelRT::DOF;
	printf("[MOVE] joint move: J1..J%d [deg], absolute; Enter alone cancels, a key letter (x, e, ...) acts as "
	       "the key\n", nDof);
	for (int i = 0; i < nDof; i++)
		printf("       J%d %+.1f .. %+.1f deg%s", i + 1, (stP.adLimLo[i] + stP.dMargin) * 180.0 / M_PI,
		       (stP.adLimHi[i] - stP.dMargin) * 180.0 / M_PI, (i % 3 == 2 || i == nDof - 1) ? "\n" : ";");
	printf("[MOVE] > ");
	fflush(stdout);
	char acLine[256];
	if (!ReadLine(acLine, sizeof(acLine)))
		return FALSE;
	if (const char cKey = LineKey(acLine))
	{
		PassKey(cKey);
		return FALSE;
	}
	double adDeg[CHighLevelRT::DOF];
	const int nRead = ParseNumbers(acLine, adDeg, nDof);
	if (nRead == 0)
	{
		printf("[MOVE] cancelled\n");
		return FALSE;
	}
	if (nRead != nDof)
	{
		printf("[MOVE] refused - %d numbers expected [deg]\n", nDof);
		return FALSE;
	}
	task_slots::ST_MOVE_REQ stReq;
	memset(&stReq, 0, sizeof(stReq));
	BOOL bOk = TRUE;
	for (int i = 0; i < nDof; i++)
	{
		const double dQ = adDeg[i] * M_PI / 180.0;
		const double dLo = stP.adLimLo[i] + stP.dMargin, dHi = stP.adLimHi[i] - stP.dMargin;
		if (!std::isfinite(dQ) || dQ < dLo || dQ > dHi)
		{
			printf("[MOVE] refused - J%d %.1f deg outside %+.1f .. %+.1f deg (the joint limit minus %.2f rad)\n",
			       i + 1, adDeg[i], dLo * 180.0 / M_PI, dHi * 180.0 / M_PI, stP.dMargin);
			bOk = FALSE;
		}
		stReq.adTarget[i] = dQ;
	}
	if (!bOk)
		return FALSE;
	stReq.eKind = task_slots::eMoveJoint;
	m_moveSlot.Write(stReq);
	return TRUE;
}

// 't': a position of the tool in the base frame [mm]; refused if the IK of
// the high-level controller does not reach it from q_r (the reach of the
// arm inside its joint limits minus the margin). The Trajectory task then
// runs TRACK on goals of the operator, as with a goal of the camera.
BOOL
CRobotIndy7::ReadTaskMove()
{
	if (m_pKeyIk == NULL)
	{
		printf("[MOVE] refused - the high-level controller is not available\n");
		return FALSE;
	}
	printf("[MOVE] task move: the tool position x y z [mm] in the base frame (orientation free); Enter alone "
	       "cancels, a key letter (x, e, ...) acts as the key\n[MOVE] > ");
	fflush(stdout);
	char acLine[256];
	if (!ReadLine(acLine, sizeof(acLine)))
		return FALSE;
	if (const char cKey = LineKey(acLine))
	{
		PassKey(cKey);
		return FALSE;
	}
	double adMm[3];
	const int nRead = ParseNumbers(acLine, adMm, 3);
	if (nRead == 0)
	{
		printf("[MOVE] cancelled\n");
		return FALSE;
	}
	if (nRead != 3 || !std::isfinite(adMm[0]) || !std::isfinite(adMm[1]) || !std::isfinite(adMm[2]))
	{
		printf("[MOVE] refused - x y z expected [mm]\n");
		return FALSE;
	}
	const double adGoal[3] = { adMm[0] * 1e-3, adMm[1] * 1e-3, adMm[2] * 1e-3 };
	double adQ[CHighLevelRT::DOF];
	int nIt = 0;
	double dErr = 0.0;
	if (FALSE == m_pKeyIk->SolveIk(adGoal, adQ, &nIt, &dErr))
	{
		printf("[MOVE] refused - (%.1f, %.1f, %.1f) mm is out of reach: the IK ends %.1f mm away after %d "
		       "iterations\n", adMm[0], adMm[1], adMm[2], dErr * 1e3, nIt);
		return FALSE;
	}
	task_slots::ST_MOVE_REQ stReq;
	memset(&stReq, 0, sizeof(stReq));
	for (int k = 0; k < 3; k++)
		stReq.adTarget[k] = adGoal[k];
	stReq.eKind = task_slots::eMoveTask;
	printf("[MOVE] reachable (IK: %d iterations, %.3f mm); joints [deg] %.1f %.1f %.1f %.1f %.1f %.1f\n", nIt,
	       dErr * 1e3, adQ[0] * 180.0 / M_PI, adQ[1] * 180.0 / M_PI, adQ[2] * 180.0 / M_PI, adQ[3] * 180.0 / M_PI,
	       adQ[4] * 180.0 / M_PI, adQ[5] * 180.0 / M_PI);
	m_moveSlot.Write(stReq);
	return TRUE;
}

void
proc_keyboard_control(void* apRobot)
{
	CRobotIndy7* pRobot = (CRobotIndy7*)apRobot;
	pRobot->PrintHelp();
	while (!pRobot->CheckStopTask())
	{
		const int nKey = getche();
		if (nKey == EOF)
		{
			printf("[KEY] the input has ended - the keys are off ('kill -INT' ends the controller)\n");
			break;
		}
		const char cKey = (char)nKey;
		// escape sequences (arrow keys, \033[X) are ignored
		if (cKey == '\033')
		{
			getche();
			getche();
			continue;
		}
		// a newline carries no command: a piped "u\n" (or Enter) would
		// otherwise overwrite the key before the Dynamics task reads it
		if (cKey == '\n' || cKey == '\r')
			continue;
		if (cKey == 'q')
		{
			pRobot->StopTasks();
			break;
		}
		// the tools that read a line: here, off the real-time tasks
		if (cKey == 'm' || cKey == 'M')
		{
			printf("\n");
			pRobot->ReadJointMove();
			continue;
		}
		if (cKey == 't' || cKey == 'T')
		{
			printf("\n");
			pRobot->ReadTaskMove();
			continue;
		}
		if (cKey == '?')
		{
			printf("\n");
			pRobot->PrintHelp();
			continue;
		}
		pRobot->m_cKeyPress.store(cKey);
	}
	DBG_LOG_INFO("(proc_keyboard_control) task ended");
}

void
CRobotIndy7::SaveRobotPose()
{
	char acPath[512];
	snprintf(acPath, sizeof(acPath), "%s/robot_poses.csv", m_cCalib.GetDir().c_str());
	mkdir(m_cCalib.GetDir().c_str(), 0777);
	m_nPoseCapture++;

	FILE* fp;
	if (m_nPoseCapture == 1)
	{
		fp = fopen(acPath, "w");
		if (!fp) { DBG_LOG_ERROR("SaveRobotPose: cannot open %s", acPath); m_nPoseCapture--; return; }
		fprintf(fp, "capture,tx,ty,tz,r00,r01,r02,r10,r11,r12,r20,r21,r22\n");
	}
	else
	{
		fp = fopen(acPath, "a");
		if (!fp) { DBG_LOG_ERROR("SaveRobotPose: cannot open %s", acPath); m_nPoseCapture--; return; }
	}
	const auto& p = m_Pose.m_position;
	const auto& R = m_Pose.m_rotation;
	fprintf(fp, "%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
	        m_nPoseCapture, p[0], p[1], p[2], R(0,0), R(0,1), R(0,2), R(1,0), R(1,1), R(1,2), R(2,0), R(2,1), R(2,2));
	fclose(fp);
	DBG_LOG_INFO("[SaveRobotPose] #%d saved: t=[%.4f, %.4f, %.4f]", m_nPoseCapture, p[0], p[1], p[2]);
	printf("[robot_poses] #%d saved -> %s\n", m_nPoseCapture, acPath);
}
