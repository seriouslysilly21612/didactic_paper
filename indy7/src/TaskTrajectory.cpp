/*****************************************************************************
*	Name: TaskTrajectory.cpp
*	Description: The Trajectory task (TASK1, paper, Sec. IV-C), the
*	high-level tests it runs (Sec. V-B and V-C), and the motions of the
*	operator ('m', 't').
*
*	In every period the task reads the State slot of the EtherCAT task and
*	the Torque slot of the Dynamics task, takes the command of the operator
*	for the high-level motions (m_nHlCmd: a menu entry of 'y', or the stop),
*	a motion of the operator (m_moveSlot) and a selection of the camera
*	application, lets the test program write the goal of the period into the
*	goal slot, runs the high-level controller (CTrajectoryRT: goal admission,
*	IK, Ruckig, lead bound) if TRACK or HOME is running, and publishes the
*	joint reference in the Reference slot, which the Dynamics task follows
*	with computed torque control. It also writes the log of the high-level
*	tests. The Reference slot is written in every period, with eRefNone when
*	no motion runs, so that the Dynamics task always sees a fresh word from
*	this task.
*****************************************************************************/
#include "Indy7Ctrl.h"
#include "base/MallocCount.h"

#include <cmath>
#include <cstring>

//============================================================================
// High-level tests (paper, Sec. V-B and V-C). Each test moves to q_r (HOME),
// then runs TRACK; a test program writes the goals into the goal slot in the
// Trajectory task, before the high-level controller reads them, so that a
// test is the same in every run, in the simulation and on the robot:
//   1 random goals (cfg HL_RAND_*, each from q_r); with HL_RAND_GOALS=0 the
//     fixed goal HL_TEST_GOAL_M from the tool at q_r
//   2 random goal jumps (cfg HL_RAND_JUMP*, each from q_r): a first goal and
//     a new goal near it, to which the goal jumps during the motion; with
//     HL_RAND_JUMPS=0 the goal of 1, moved by HL_TEST_JUMP_M when the tool is
//     HL_TEST_TRIGGER_M from it
//   3 the goal of 1 with goal noise at the rate of the camera (synthetic, or
//     recorded: HL_TEST_NOISE_FILE), delta of the cfg
//   4 the same with delta = 0
//   5 reference check: goals that change during the motion, goals out of
//     reach, repeated goals, and gaps, with noise, for HL_TEST_REF_S
//   6 HOME, 7 record q_r, 8 the camera application
//============================================================================
namespace {
/* xorshift64* and Box-Muller: the same sequence in every run, no allocation */
double HlUniform(unsigned long long& auS)
{
	auS ^= auS >> 12;
	auS ^= auS << 25;
	auS ^= auS >> 27;
	return (double)((auS * 2685821657736338717ULL) >> 11) * (1.0 / 9007199254740992.0);
}
double HlNormal(unsigned long long& auS)
{
	double dU1 = HlUniform(auS);
	if (dU1 < 1e-300)
		dU1 = 1e-300;
	const double dU2 = HlUniform(auS);
	return std::sqrt(-2.0 * std::log(dU1)) * std::cos(2.0 * M_PI * dU2);
}
const char* const s_apcHlName[] = { "", "fixed goal", "goal jump", "goal noise, delta", "goal noise, delta = 0",
                                    "reference check", "HOME", "record q_r", "camera application" };
const char* const s_apcHlId[]   = { "", "fixed", "jump", "noise", "noise0", "refcheck", "home", "qr", "app" };
}

void
CRobotIndy7::InitHighLevelTests()
{
	const ST_CONFIG_SYSTEM& stSys = m_pConfig->System();
	for (int k = 0; k < 3; k++)
	{
		m_adHlGoalOff[k] = k < (int)stSys.vHlTestGoal.size() ? stSys.vHlTestGoal[k] : 0.0;
		m_adHlJump[k]    = k < (int)stSys.vHlTestJump.size() ? stSys.vHlTestJump[k] : 0.0;
	}
	m_dHlTrigger = stSys.dHlTestTrigger;
	m_dHlNoise   = stSys.dHlTestNoise;
	m_dHlNoiseHz = stSys.dHlTestNoiseHz > 0.0 ? stSys.dHlTestNoiseHz : 15.0;
	m_dHlNoiseT  = stSys.dHlTestNoiseT;
	m_dHlRefT    = stSys.dHlTestRefT;
	LoadHighLevelNoise();
	m_dHlDelta   = stSys.dHlDelta;
	// the friction feedforward compensates friction: on the robot, and in a
	// simulation with friction only (paper, Sec. IV-A, eq. (7))
	m_bHlFF = (FALSE == stSys.bSim) || (TRUE == stSys.bSimFriction);

	CHighLevelLog::Meta& stM = m_stHlMeta;
	memset(&stM, 0, sizeof(stM));
	stM.nDof = m_stJtMeta.nDof;
	stM.dPeriod = m_stJtMeta.dPeriod;
	stM.bSim = m_stJtMeta.bSim;
	stM.bSimFriction = m_stJtMeta.bSimFriction;
	stM.nSimStateDelay = m_stJtMeta.nSimStateDelay;
	stM.nSimTorqueDelay = m_stJtMeta.nSimTorqueDelay;
	for (int i = 0; i < CControllerFullDynamicsRT::JT_MAX_DOF; i++)
		stM.adSimTauS[i] = m_stJtMeta.adSimTauS[i];
	for (int k = 0; k < 3; k++)
	{
		stM.adGoalOffset[k] = m_adHlGoalOff[k];
		stM.adJump[k] = m_adHlJump[k];
	}
	stM.dTrigger = m_dHlTrigger;
	stM.dNoiseHz = m_dHlNoiseHz;
	stM.bFrictionFF = m_bHlFF ? TRUE : FALSE;
	stM.nGoal = -1;
	stM.nGoals = 0;
	m_cHlLog.Start(m_cJtLog.GetDir().c_str(), "hl-log");
	DrawHighLevelGoals();
	DrawHighLevelJumps();
}

// The random goals of test 1 (cfg HL_RAND_*): uniform in the box, within
// HL_RAND_RMAX_M of the vertical axis of the base, and kept only if the IK
// converges from q_r, on an IK instance of its own (not real-time, at Init;
// the same seed gives the same goals in the simulation and on the robot).
// The list is also written to hl_goals_seed<S>.csv in the log directory.
void
CRobotIndy7::DrawHighLevelGoals()
{
	const ST_CONFIG_SYSTEM& stSys = m_pConfig->System();
	m_nHlGoals = 0;
	m_nHlGoalIdx = 0;
	const int nWant = stSys.nHlRandGoals < (int)HL_GOALS_MAX ? stSys.nHlRandGoals : (int)HL_GOALS_MAX;
	if (nWant <= 0 || !m_cTraj.IsInit())
		return;
	if (stSys.vHlRandBox.size() < 6)
	{
		printf("[HL] HL_RAND_BOX_M needs x0 x1 y0 y1 z0 z1 - test 1 keeps its fixed goal\n");
		return;
	}
	const double dT = m_stHlMeta.dPeriod > 0.0 ? m_stHlMeta.dPeriod : 1e-3;
	CHighLevelRT* pIk = new CHighLevelRT();
	if (FALSE == pIk->Init(stSys.strURDFPath.c_str(), dT, m_cTraj.HighLevel().GetParams()))
	{
		printf("[HL] cannot load the IK for the random goals - test 1 keeps its fixed goal\n");
		delete pIk;
		return;
	}
	const std::vector<double>& b = stSys.vHlRandBox;
	const double dR2 = stSys.dHlRandRMax * stSys.dHlRandRMax;
	unsigned long long uS = 0x9E3779B97F4A7C15ULL * (unsigned long long)(stSys.nHlRandSeed > 0 ? stSys.nHlRandSeed : 1);
	long nDraws = 0, nFar = 0, nNoIk = 0;
	int nItMax = 0;
	while (m_nHlGoals < nWant && nDraws < 1000L * nWant)
	{
		nDraws++;
		double adG[3];
		adG[0] = b[0] + (b[1] - b[0]) * HlUniform(uS);
		adG[1] = b[2] + (b[3] - b[2]) * HlUniform(uS);
		adG[2] = b[4] + (b[5] - b[4]) * HlUniform(uS);
		if (adG[0] * adG[0] + adG[1] * adG[1] > dR2)
		{
			nFar++;
			continue;
		}
		int nIt = 0;
		double dErr = 0.0;
		if (FALSE == pIk->SolveIk(adG, NULL, &nIt, &dErr))
		{
			nNoIk++;
			continue;
		}
		if (nIt > nItMax)
			nItMax = nIt;
		for (int k = 0; k < 3; k++)
			m_adHlGoals[m_nHlGoals][k] = adG[k];
		m_nHlGoals++;
	}
	delete pIk;
	printf("[HL] test 1: %d random goals (seed %d) in x [%.2f, %.2f], y [%.2f, %.2f], z [%.2f, %.2f] m, "
	       "r <= %.2f m: %ld draws, %ld beyond r, %ld not solved from q_r; IK iterations <= %d\n",
	       m_nHlGoals, stSys.nHlRandSeed, b[0], b[1], b[2], b[3], b[4], b[5], stSys.dHlRandRMax,
	       nDraws, nFar, nNoIk, nItMax);
	if (m_nHlGoals < nWant)
		printf("[HL] warning: only %d of %d goals found in the box\n", m_nHlGoals, nWant);
	char acPath[512];
	snprintf(acPath, sizeof(acPath), "%s/hl_goals_seed%d.csv", m_cJtLog.GetDir().c_str(), stSys.nHlRandSeed);
	FILE* fp = fopen(acPath, "w");
	if (fp != NULL)
	{
		fprintf(fp, "index,x,y,z\n");
		for (int n = 0; n < m_nHlGoals; n++)
			fprintf(fp, "%d,%.9f,%.9f,%.9f\n", n, m_adHlGoals[n][0], m_adHlGoals[n][1], m_adHlGoals[n][2]);
		fclose(fp);
	}
}

// The random goal jumps of test 2 (cfg HL_RAND_JUMP*): a first goal drawn
// like those of test 1 (box, radius, IK from q_r), at least 0.2 m from the
// tool at q_r; a new goal HL_RAND_JUMP_LEN_M from it in a direction uniform
// on the sphere, also in the box and solved from q_r; and the fraction
// HL_RAND_JUMP_AT of the distance from the tool at q_r to the first goal at
// which the goal jumps. Seeded apart from test 1, whose goals stay the same;
// written to hl_jumps_seed<S>.csv in the log directory.
void
CRobotIndy7::DrawHighLevelJumps()
{
	const ST_CONFIG_SYSTEM& stSys = m_pConfig->System();
	m_nHlJumps = 0;
	m_nHlJumpIdx = 0;
	const int nWant = stSys.nHlRandJumps < (int)HL_GOALS_MAX ? stSys.nHlRandJumps : (int)HL_GOALS_MAX;
	if (nWant <= 0 || !m_cTraj.IsInit())
		return;
	if (stSys.vHlRandBox.size() < 6 || stSys.vHlRandJumpLen.size() < 2 || stSys.vHlRandJumpAt.size() < 2)
	{
		printf("[HL] HL_RAND_BOX_M, HL_RAND_JUMP_LEN_M or HL_RAND_JUMP_AT incomplete - test 2 keeps its fixed jump\n");
		return;
	}
	const double dT = m_stHlMeta.dPeriod > 0.0 ? m_stHlMeta.dPeriod : 1e-3;
	CHighLevelRT* pIk = new CHighLevelRT();
	if (FALSE == pIk->Init(stSys.strURDFPath.c_str(), dT, m_cTraj.HighLevel().GetParams()))
	{
		printf("[HL] cannot load the IK for the random jumps - test 2 keeps its fixed jump\n");
		delete pIk;
		return;
	}
	const std::vector<double>& b = stSys.vHlRandBox;
	const double dR2 = stSys.dHlRandRMax * stSys.dHlRandRMax;
	const double dL0 = stSys.vHlRandJumpLen[0], dL1 = stSys.vHlRandJumpLen[1];
	const double dF0 = stSys.vHlRandJumpAt[0], dF1 = stSys.vHlRandJumpAt[1];
	const double dMinStart = 0.20;               // [m] first goal from the tool at q_r
	double adP0[3];
	pIk->ToolPosition(pIk->GetParams().adQr, adP0);
	unsigned long long uS = 0xD1B54A32D192ED03ULL * (unsigned long long)(stSys.nHlRandSeed > 0 ? stSys.nHlRandSeed : 1);
	long nDraws = 0, nOut = 0, nNear = 0, nNoIk = 0, nNoJump = 0;
	while (m_nHlJumps < nWant && nDraws < 1000L * nWant)
	{
		nDraws++;
		double adG1[3], adG2[3];
		adG1[0] = b[0] + (b[1] - b[0]) * HlUniform(uS);
		adG1[1] = b[2] + (b[3] - b[2]) * HlUniform(uS);
		adG1[2] = b[4] + (b[5] - b[4]) * HlUniform(uS);
		if (adG1[0] * adG1[0] + adG1[1] * adG1[1] > dR2)
		{
			nOut++;
			continue;
		}
		double dS2 = 0.0;
		for (int k = 0; k < 3; k++)
			dS2 += (adG1[k] - adP0[k]) * (adG1[k] - adP0[k]);
		if (dS2 < dMinStart * dMinStart)
		{
			nNear++;
			continue;
		}
		int nIt = 0;
		double dErr = 0.0;
		if (FALSE == pIk->SolveIk(adG1, NULL, &nIt, &dErr))
		{
			nNoIk++;
			continue;
		}
		bool bOk = false;
		for (int nTry = 0; nTry < 100 && !bOk; nTry++)
		{
			double adV[3], dN2 = 0.0;
			for (int k = 0; k < 3; k++)
			{
				adV[k] = HlNormal(uS);
				dN2 += adV[k] * adV[k];
			}
			const double dLen = dL0 + (dL1 - dL0) * HlUniform(uS);
			if (dN2 < 1e-12)
				continue;
			for (int k = 0; k < 3; k++)
				adG2[k] = adG1[k] + dLen * adV[k] / std::sqrt(dN2);
			if (adG2[0] < b[0] || adG2[0] > b[1] || adG2[1] < b[2] || adG2[1] > b[3] || adG2[2] < b[4] || adG2[2] > b[5]
			    || adG2[0] * adG2[0] + adG2[1] * adG2[1] > dR2)
				continue;
			if (FALSE == pIk->SolveIk(adG2, NULL, &nIt, &dErr))
				continue;
			bOk = true;
		}
		if (!bOk)
		{
			nNoJump++;
			continue;
		}
		for (int k = 0; k < 3; k++)
		{
			m_adHlJ1[m_nHlJumps][k] = adG1[k];
			m_adHlJ2[m_nHlJumps][k] = adG2[k];
		}
		m_adHlJAt[m_nHlJumps] = dF0 + (dF1 - dF0) * HlUniform(uS);
		m_nHlJumps++;
	}
	delete pIk;
	printf("[HL] test 2: %d random goal jumps (seed %d) of %.2f to %.2f m, at %.0f to %.0f %% of the distance to "
	       "the first goal: %ld draws, %ld beyond r, %ld within %.2f m of q_r, %ld not solved, %ld without a new goal\n",
	       m_nHlJumps, stSys.nHlRandSeed, dL0, dL1, 100.0 * dF0, 100.0 * dF1, nDraws, nOut, nNear, dMinStart,
	       nNoIk, nNoJump);
	if (m_nHlJumps < nWant)
		printf("[HL] warning: only %d of %d jumps found in the box\n", m_nHlJumps, nWant);
	char acPath[512];
	snprintf(acPath, sizeof(acPath), "%s/hl_jumps_seed%d.csv", m_cJtLog.GetDir().c_str(), stSys.nHlRandSeed);
	FILE* fp = fopen(acPath, "w");
	if (fp != NULL)
	{
		fprintf(fp, "index,x1,y1,z1,x2,y2,z2,at\n");
		for (int n = 0; n < m_nHlJumps; n++)
			fprintf(fp, "%d,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.6f\n", n, m_adHlJ1[n][0], m_adHlJ1[n][1], m_adHlJ1[n][2],
			        m_adHlJ2[n][0], m_adHlJ2[n][1], m_adHlJ2[n][2], m_adHlJAt[n]);
		fclose(fp);
	}
}

// a sequence of test 1 or 2 ends early; 'y' with its digit continues it
void
CRobotIndy7::StopHighLevelSeq(const char* apcWhen)
{
	m_bHlSeq = false;
	m_bHlSeqNext = false;
	if (m_nHlSel == 2)
		printf("[HL] random goal jumps stopped %s jump %d of %d; 'y', 2 continues with it\n", apcWhen,
		       m_nHlJumpIdx + 1, m_nHlJumps);
	else
		printf("[HL] random goals stopped %s goal %d of %d; 'y', 1 continues with it\n", apcWhen,
		       m_nHlGoalIdx + 1, m_nHlGoals);
}

// The start conditions of a motion of the high-level controller (a test, the
// camera application, a motion of the operator; Trajectory task): the state
// of the arm from the State slot, that of the controller from the Torque
// slot (its last cycle)
BOOL
CRobotIndy7::CheckMotionStart(const char* apcWho, const task_slots::ST_STATE_SLOT& astState,
                              const task_slots::ST_TORQUE_SLOT& astTorque)
{
	if (!astTorque.bEnabled)
	{
		printf("[%s] refused - controller not enabled ('r', then 'g')\n", apcWho);
		return FALSE;
	}
	if (!astState.bServoOn)
	{
		printf("[%s] refused - servo OFF ('h' first)\n", apcWho);
		return FALSE;
	}
	if (astTorque.bJtRunning || m_cTraj.IsRunning() || m_eHlTest != eHlTestIdle || m_eUserMove != eUserNone)
	{
		printf("[%s] refused - another motion is running ('x' first)\n", apcWho);
		return FALSE;
	}
	if (astTorque.eCtrlMode != (int)CControllerFullDynamicsRT::eGravityCompensation)
	{
		printf("[%s] refused - start from gravity compensation ('g')\n", apcWho);
		return FALSE;
	}
	return TRUE;
}

// menu entry anSel of 'y' (Trajectory task, from the command word of
// DoInput, or the selection of the camera application)
BOOL
CRobotIndy7::StartHighLevelMenu(int anSel, const task_slots::ST_STATE_SLOT& astState,
                                const task_slots::ST_TORQUE_SLOT& astTorque)
{
	if (anSel < 1 || anSel > HL_MENU)
		return FALSE;
	if (anSel == 7)
	{
		double adQ[CHighLevelRT::DOF];
		for (int i = 0; i < CHighLevelRT::DOF; i++)
			adQ[i] = astState.adQ[i];
		m_cTraj.HighLevel().SetReady(adQ);
		printf("[HL] q_r recorded for this session; for the cfg:\n"
		       "HL_READY_Q=%.6f %.6f %.6f %.6f %.6f %.6f\n", adQ[0], adQ[1], adQ[2], adQ[3], adQ[4], adQ[5]);
		// the IK starts every goal from q_r and clips its iterates to the
		// limits minus the margin: q_r should lie inside that range
		const CHighLevelRT::Params& stP = m_cTraj.HighLevel().GetParams();
		for (int i = 0; i < CHighLevelRT::DOF; i++)
			if (adQ[i] < stP.adLimLo[i] + stP.dMargin || adQ[i] > stP.adLimHi[i] - stP.dMargin)
				printf("[HL] warning: J%d is within %.2f rad of its limit - choose another q_r\n", i + 1, stP.dMargin);
		if (m_nHlGoals > 0)
			printf("[HL] the random goals of test 1 were solved from the q_r of the cfg: put this q_r into "
			       "HL_READY_Q and restart before test 1\n");
		printf("[HL] 't' still checks the reach of a position from the q_r of the cfg\n");
		return TRUE;
	}
	if (anSel == HL_APP && m_pBridge == NULL)
	{
		printf("[HL] refused - no bridge (APP_BRIDGE=1 in the cfg, a build with ROS 2, "
		       "/opt/ros/jazzy and ros2/install sourced before run.sh)\n");
		return FALSE;
	}
	if (FALSE == CheckMotionStart("HL", astState, astTorque))
		return FALSE;
	if (m_bHlSeqNext)
	{
		printf("[HL] refused - a sequence of test %d is between two runs ('x' first)\n", m_nHlSel);
		return FALSE;
	}
	if (!m_cHlLog.IsIdle())
	{
		printf("[HL] refused - the log of the last test is still being written\n");
		return FALSE;
	}
	char acWhy[192];
	if (FALSE == m_cTraj.Start(true, astState.adQ, astState.adQd, m_bHlFF ? TRUE : FALSE, acWhy, sizeof(acWhy)))
	{
		printf("[HL] refused - %s\n", acWhy);
		return FALSE;
	}
	m_nHlSel = anSel;
	m_eHlTest = eHlTestHoming;
	m_bHlTrackPending = false;
	if (anSel == 1 && m_nHlGoals > 0)
	{
		/* the random goals: from the next one on (after a stop, the stopped one) */
		if (m_nHlGoalIdx >= m_nHlGoals)
			m_nHlGoalIdx = 0;
		m_bHlSeq = true;
		printf("[HL] random goal %d of %d: HOME to q_r, then TRACK - 'x' stops\n", m_nHlGoalIdx + 1, m_nHlGoals);
	}
	else if (anSel == 2 && m_nHlJumps > 0)
	{
		if (m_nHlJumpIdx >= m_nHlJumps)
			m_nHlJumpIdx = 0;
		m_bHlSeq = true;
		printf("[HL] random goal jump %d of %d: HOME to q_r, then TRACK - 'x' stops\n", m_nHlJumpIdx + 1, m_nHlJumps);
	}
	else
		printf("[HL] %s: HOME to q_r%s - 'x' stops\n", s_apcHlName[anSel], anSel == 6 ? "" : ", then TRACK");
	return TRUE;
}

// TRACK after HOME (Trajectory task): the slot of this run, the goals of the
// test fixed in space, and the log. Called once the Dynamics task reports
// gravity compensation again after the end of HOME (HighLevelTestAfterUpdate).
BOOL
CRobotIndy7::StartHighLevelTrack(const task_slots::ST_STATE_SLOT& astState,
                                 const task_slots::ST_TORQUE_SLOT& astTorque)
{
	/* The slot of this run: that of the test program, or, for the camera
	 * application, that of the bridge, whose positions get the hover height
	 * (paper, Sec. V-C) */
	const bool bApp = (m_nHlSel == HL_APP);
	const double adOff[3] = {0.0, 0.0, bApp ? m_dAppHover : 0.0};
	m_cTraj.SetGoalSlot(bApp ? &m_appGoalSlot : &m_goalSlot);
	m_cTraj.SetGoalOffset(adOff);
	m_cTraj.HighLevel().SetDelta(m_nHlSel == 4 ? 0.0 : m_dHlDelta);
	char acWhy[192];
	if (FALSE == m_cTraj.Start(false, astState.adQ, astState.adQd, m_bHlFF ? TRUE : FALSE, acWhy, sizeof(acWhy)))
	{
		printf("[HL] TRACK refused - %s\n", acWhy);
		m_eHlTest = eHlTestIdle;
		if (m_bHlSeq)
			StopHighLevelSeq("at");
		return FALSE;
	}
	/* the goals are fixed in space, relative to the tool at q_r (p0), not to
	 * where HOME stopped: that differs from run to run by the error at rest
	 * (on the robot, the static-friction error), and the spread of the final
	 * positions must show only the repeatability; the random goals of test 1
	 * are fixed in the base frame */
	double adQ[CHighLevelRT::DOF];
	for (int i = 0; i < CHighLevelRT::DOF; i++)
		adQ[i] = astState.adQ[i];
	m_cTraj.HighLevel().ToolPosition(m_cTraj.HighLevel().GetParams().adQr, m_adHlP0);
	m_cTraj.HighLevel().ToolPosition(adQ, m_adHlTool);
	const bool bRandom = (m_nHlSel == 1 && m_nHlGoals > 0);
	const bool bRandJump = (m_nHlSel == 2 && m_nHlJumps > 0);
	double dStart2 = 0.0;
	for (int k = 0; k < 3; k++)
	{
		m_adHlBase[k] = bRandom ? m_adHlGoals[m_nHlGoalIdx][k] :
		                bRandJump ? m_adHlJ1[m_nHlJumpIdx][k] : m_adHlP0[k] + m_adHlGoalOff[k];
		m_adHlJumpRun[k] = bRandJump ? m_adHlJ2[m_nHlJumpIdx][k] - m_adHlJ1[m_nHlJumpIdx][k] : m_adHlJump[k];
		dStart2 += (m_adHlBase[k] - m_adHlP0[k]) * (m_adHlBase[k] - m_adHlP0[k]);
	}
	/* the trigger of a random jump: its fraction of the distance from the
	 * tool at q_r (not where HOME stopped) to the first goal */
	m_dHlTriggerRun = bRandJump ? m_adHlJAt[m_nHlJumpIdx] * std::sqrt(dStart2) : m_dHlTrigger;
	m_nHlCycle = 0;
	m_nHlLastEvent = 0;
	m_nHlRestSince = -1;
	m_bHlJumped = false;
	m_bHlGap = false;
	m_nHlNextSeg = 0;
	m_uHlRng = 0x9E3779B97F4A7C15ULL * (unsigned long long)m_nHlSel;	// the same noise in every run

	CHighLevelLog::Meta stM = m_stHlMeta;
	snprintf(stM.acTest, sizeof(stM.acTest), "%s", s_apcHlId[m_nHlSel]);
	stM.nRun = ++m_anHlRuns[m_nHlSel - 1];
	stM.stP = m_cTraj.HighLevel().GetParams();
	for (int k = 0; k < 3; k++)
	{
		stM.adP0[k] = m_adHlP0[k];
		if (bRandom || bRandJump)
			stM.adGoalOffset[k] = m_adHlBase[k] - m_adHlP0[k];
		if (bApp)                   // the application: the offset that the high-level controller adds (hover height)
			stM.adGoalOffset[k] = adOff[k];
		stM.adJump[k] = m_adHlJumpRun[k];
	}
	stM.dTrigger = m_dHlTriggerRun;
	stM.nGoal = bRandom ? m_nHlGoalIdx : bRandJump ? m_nHlJumpIdx : -1;
	stM.nGoals = bRandom ? m_nHlGoals : bRandJump ? m_nHlJumps : 0;
	stM.dNoise = (m_nHlSel >= 3 && m_nHlSel <= 5) ? m_dHlNoise : 0.0;
	snprintf(stM.acNoiseFile, sizeof(stM.acNoiseFile), "%s",
	         (m_nHlSel == 3 || m_nHlSel == 4) && m_nHlNoiseRec > 0 ? m_acHlNoiseFile : "");
	m_nHlNoiseIdx = -1;
	stM.dDuration = (m_nHlSel == 3 || m_nHlSel == 4) ? m_dHlNoiseT : (m_nHlSel == 5 ? m_dHlRefT :
	                (bApp ? m_dAppT : 0.0));
	for (int i = 0; i < stM.nDof; i++)
	{
		m_pController->GetControlGain((UINT)i, &stM.adKp[i], &stM.adKd[i]);
		stM.adKf[i] = m_bHlFF ? m_pController->GetFrictionFFAxis((UINT)i) : 0.0;
		stM.adTauMax[i] = m_pController->GetTorqueLimit((UINT)i);
	}
	m_stHlRt0 = astState.stFaults;
	m_nHlCtrlFaults0 = m_nCtrlFaults.load(std::memory_order_acquire);
	m_nHlDropouts0 = m_nDriveDropouts.load(std::memory_order_acquire);
	m_uHlAllocs0 = astTorque.uAllocUpdate;
	m_cHlLog.Open(stM);
	m_eHlTest = eHlTestTracking;
	printf("[HL] TRACK: %s, run %d, delta %.1f mm%s\n", s_apcHlName[m_nHlSel], stM.nRun, stM.stP.dDelta * 1e3,
	       m_cHlLog.IsStarted() ? "" : " (not logged)");
	return TRUE;
}

// The stop word of DoInput ('x', 'g', 'e', 'j'), or the stop of the
// application (Trajectory task): TRACK or HOME ends now; a running test then
// ends in HighLevelTestAfterUpdate of the same period, which closes its log,
// and a motion of the operator after the step.
void
CRobotIndy7::StopHighLevel()
{
	if (m_bHlSeqNext)		// between two random goals: no next one
		printf("[HL] random goals stopped before goal %d of %d; 'y', 1 continues with it\n",
		       m_nHlGoalIdx + 1, m_nHlGoals);
	m_bHlSeqNext = false;	// a running goal ends in HighLevelTestAfterUpdate, which ends the sequence
	if (m_bHlTrackPending)	// HOME done, TRACK not yet started: the test ends here
	{
		m_bHlTrackPending = false;
		printf("[HL] stopped before TRACK\n");
		m_eHlTest = eHlTestIdle;
		if (m_bHlSeq)
			StopHighLevelSeq("at");
	}
	if (m_eHlTest == eHlTestIdle)
		m_bHlSeq = false;
	if (m_cTraj.IsRunning())
		m_cTraj.End(CTrajectoryRT::eAborted);
}

// The recorded goal noise of tests 3 and 4 (HL_TEST_NOISE_FILE,
// tools/goal_noise_file.py): CSV with a header and rows t,dx,dy,dz [s, m];
// the times from the first sample on; read before the tasks start
void
CRobotIndy7::LoadHighLevelNoise()
{
	const ST_CONFIG_SYSTEM& stSys = m_pConfig->System();
	m_nHlNoiseRec = 0;
	m_acHlNoiseFile[0] = 0;
	if (stSys.strHlNoiseFile.empty())
		return;
	const TSTRING& strPath = stSys.strHlNoiseFile;      // relative to the cfg (Config.h)
	FILE* fp = fopen(strPath.c_str(), "r");
	if (fp == NULL)
	{
		printf("[HL] HL_TEST_NOISE_FILE %s cannot be read - tests 3 and 4 use the synthetic noise\n",
		       strPath.c_str());
		return;
	}
	char acLine[256];
	double dT0 = 0.0;
	while (fgets(acLine, sizeof(acLine), fp) != NULL && m_nHlNoiseRec < (int)HL_NOISE_MAX)
	{
		double dT, dX, dY, dZ;
		if (sscanf(acLine, "%lf,%lf,%lf,%lf", &dT, &dX, &dY, &dZ) != 4)
			continue;                       // the header, or a malformed line
		if (m_nHlNoiseRec == 0)
			dT0 = dT;
		if (m_nHlNoiseRec > 0 && dT - dT0 <= m_adHlNoiseRecT[m_nHlNoiseRec - 1])
			continue;                       // not increasing
		m_adHlNoiseRecT[m_nHlNoiseRec] = dT - dT0;
		m_adHlNoiseRec[m_nHlNoiseRec][0] = dX;
		m_adHlNoiseRec[m_nHlNoiseRec][1] = dY;
		m_adHlNoiseRec[m_nHlNoiseRec][2] = dZ;
		m_nHlNoiseRec++;
	}
	fclose(fp);
	const size_t uSlash = strPath.rfind('/');
	snprintf(m_acHlNoiseFile, sizeof(m_acHlNoiseFile), "%s",
	         strPath.c_str() + (uSlash == TSTRING::npos ? 0 : uSlash + 1));
	if (m_nHlNoiseRec < 2)
	{
		printf("[HL] HL_TEST_NOISE_FILE %s: fewer than 2 samples - synthetic noise\n", strPath.c_str());
		m_nHlNoiseRec = 0;
		return;
	}
	printf("[HL] tests 3 and 4: recorded goal noise %s, %d samples over %.1f s (%.1f Hz)\n", m_acHlNoiseFile,
	       m_nHlNoiseRec, m_adHlNoiseRecT[m_nHlNoiseRec - 1],
	       (m_nHlNoiseRec - 1) / m_adHlNoiseRecT[m_nHlNoiseRec - 1]);
}

// the test program: the goal of this TRACK cycle (Trajectory task, before
// the high-level controller reads the slot)
void
CRobotIndy7::HighLevelTestBeforeUpdate()
{
	if (m_eHlTest != eHlTestTracking)
		return;
	const long n = m_nHlCycle;
	const double dT = m_stHlMeta.dPeriod > 0.0 ? m_stHlMeta.dPeriod : 1e-3;
	const long nNoise = (long)std::lround(1.0 / (m_dHlNoiseHz * dT)) > 0 ? (long)std::lround(1.0 / (m_dHlNoiseHz * dT)) : 1;
	double adGoal[3] = { m_adHlBase[0], m_adHlBase[1], m_adHlBase[2] };
	bool bWrite = false, bEvent = false;
	switch (m_nHlSel)
	{
		case 1:
			bWrite = bEvent = (n == 0);
			break;
		case 2:
		{
			if (n == 0)
				bWrite = bEvent = true;
			else if (!m_bHlJumped)
			{
				double dD2 = 0.0;
				for (int k = 0; k < 3; k++)
					dD2 += (m_adHlTool[k] - m_adHlBase[k]) * (m_adHlTool[k] - m_adHlBase[k]);
				if (dD2 <= m_dHlTriggerRun * m_dHlTriggerRun)
				{
					for (int k = 0; k < 3; k++)
						adGoal[k] = m_adHlBase[k] += m_adHlJumpRun[k];
					m_bHlJumped = true;
					bWrite = bEvent = true;
				}
			}
			break;
		}
		case 3:
		case 4:
			if (m_nHlNoiseRec > 0)
			{
				// the recorded noise: the sample of this time, at the
				// recorded rate, the recording replayed in a loop
				const double dSpan = m_adHlNoiseRecT[m_nHlNoiseRec - 1] + 1.0 / m_dHlNoiseHz;
				const double dTm = std::fmod((double)n * dT, dSpan);
				int nIdx = (m_nHlNoiseIdx >= 0 && m_nHlNoiseIdx < m_nHlNoiseRec &&
				            m_adHlNoiseRecT[m_nHlNoiseIdx] <= dTm) ? m_nHlNoiseIdx : 0;
				while (nIdx + 1 < m_nHlNoiseRec && m_adHlNoiseRecT[nIdx + 1] <= dTm)
					nIdx++;
				if (n == 0 || nIdx != m_nHlNoiseIdx)
				{
					for (int k = 0; k < 3; k++)
						adGoal[k] = m_adHlBase[k] + m_adHlNoiseRec[nIdx][k];
					bWrite = true;
					bEvent = (n == 0);
				}
				m_nHlNoiseIdx = nIdx;
			}
			else if (n % nNoise == 0)
			{
				for (int k = 0; k < 3; k++)
					adGoal[k] = m_adHlBase[k] + m_dHlNoise * HlNormal(m_uHlRng);
				bWrite = true;
				bEvent = (n == 0);
			}
			break;
		case 5:
		{
			if (n >= m_nHlNextSeg)
			{
				const double dR = HlUniform(m_uHlRng);
				double dDur;
				m_bHlGap = false;
				if (dR < 0.6)			// a new goal, often while the arm still moves
				{
					m_adHlBase[0] = m_adHlP0[0] - 0.15 + 0.40 * HlUniform(m_uHlRng);
					m_adHlBase[1] = m_adHlP0[1] - 0.20 + 0.40 * HlUniform(m_uHlRng);
					m_adHlBase[2] = m_adHlP0[2] - 0.30 + 0.35 * HlUniform(m_uHlRng);
					dDur = 0.1 + 1.4 * HlUniform(m_uHlRng);
				}
				else if (dR < 0.7)		// out of reach: rejected, the target stays
				{
					m_adHlBase[0] = m_adHlP0[0] + 1.5;
					m_adHlBase[1] = m_adHlP0[1];
					m_adHlBase[2] = m_adHlP0[2];
					dDur = 0.3 + 1.2 * HlUniform(m_uHlRng);
				}
				else if (dR < 0.8)		// the same goal again
					dDur = 0.1 + 0.9 * HlUniform(m_uHlRng);
				else					// a gap: no goal arrives
				{
					m_bHlGap = true;
					dDur = 0.5 + 2.5 * HlUniform(m_uHlRng);
				}
				m_nHlNextSeg = n + (long)std::lround(dDur / dT);
			}
			if (!m_bHlGap && n % nNoise == 0)
			{
				for (int k = 0; k < 3; k++)
					adGoal[k] = m_adHlBase[k] + m_dHlNoise * HlNormal(m_uHlRng);
				bWrite = true;
			}
			break;
		}
		default:
			break;
	}
	if (bWrite)
		m_goalSlot.Write(adGoal);
	if (bEvent)
	{
		m_nHlLastEvent = n;
		m_nHlRestSince = -1;
	}
}

// After the high-level step: the log, HOME -> TRACK, the end of the test
// (Trajectory task). The torque of the sample is that of the Dynamics task's
// last cycle (Torque slot); TRACK after HOME starts once the Dynamics task
// reports gravity compensation again (the period after HOME ended), or gives
// up after HL_START_WAIT periods.
void
CRobotIndy7::HighLevelTestAfterUpdate(const task_slots::ST_STATE_SLOT& astState,
                                      const task_slots::ST_TORQUE_SLOT& astTorque)
{
	/* the random goals of test 1 (and the jumps of test 2): the next one once
	 * the log of the last one is written (a refusal, e.g. servo off, ends the
	 * sequence), and once the Dynamics task reports gravity compensation
	 * again: its Torque slot lags the end of the motion by a period or two */
	if (m_bHlSeqNext && m_eHlTest == eHlTestIdle)
	{
		if (m_cHlLog.IsIdle())
		{
			const bool bGrav = astTorque.eCtrlMode == (int)CControllerFullDynamicsRT::eGravityCompensation &&
			                   !astTorque.bJtRunning;
			if (bGrav)
			{
				m_bHlSeqNext = false;
				m_nHlStartWait = 0;
				if (FALSE == StartHighLevelMenu(m_nHlSel, astState, astTorque))
					StopHighLevelSeq("before");
			}
			else if (++m_nHlStartWait > (int)HL_START_WAIT)
			{
				m_bHlSeqNext = false;
				m_nHlStartWait = 0;
				printf("[HL] the controller did not return to gravity compensation\n");
				StopHighLevelSeq("before");
			}
		}
		return;
	}
	if (m_eHlTest == eHlTestIdle || m_pController == NULL)
		return;
	typedef CTrajectoryRT T;
	const T::eState eSt = m_cTraj.GetState();
	if (m_eHlTest == eHlTestHoming)
	{
		if (m_bHlTrackPending)
		{
			/* TRACK from q_r once the Dynamics task is back in gravity
			 * compensation, at rest */
			if (astTorque.eCtrlMode == (int)CControllerFullDynamicsRT::eGravityCompensation && !astTorque.bJtRunning)
			{
				m_bHlTrackPending = false;
				StartHighLevelTrack(astState, astTorque);
			}
			else if (++m_nHlStartWait > (int)HL_START_WAIT)
			{
				m_bHlTrackPending = false;
				printf("[HL] TRACK not started: the controller did not return to gravity compensation\n");
				m_eHlTest = eHlTestIdle;
				if (m_bHlSeq)
					StopHighLevelSeq("at");
			}
			return;
		}
		if (eSt == T::eRunning)
			return;
		if (eSt != T::eDoneAtRest)
		{
			printf("[HL] HOME stopped\n");
			m_eHlTest = eHlTestIdle;
			if (m_bHlSeq)
				StopHighLevelSeq("at");
			return;
		}
		if (m_nHlSel == 6)
		{
			printf("[HL] at q_r - gravity compensation\n");
			m_eHlTest = eHlTestIdle;
			return;
		}
		/* HOME has ended at rest: TRACK starts in the next period, when the
		 * Dynamics task has followed the Reference slot back to gravity
		 * compensation */
		m_bHlTrackPending = true;
		m_nHlStartWait = 0;
		return;
	}

	/* TRACK */
	if (m_cTraj.HasSample())
	{
		T::Sample& stS = m_cTraj.GetSample();
		// the torque of the Dynamics task's last cycle (Torque slot)
		for (int i = 0; i < T::MAX_DOF; i++)
			stS.adTau[i] = astTorque.bValid ? astTorque.adTau[i] : 0.0;
		stS.uClamp = astTorque.bValid ? astTorque.uClamp : 0;
		m_cHlLog.Push(stS);
		for (int k = 0; k < 3; k++)
			m_adHlTool[k] = stS.adTool[k];
		if ((stS.uFlags & T::HL_AT_REST) && m_nHlRestSince < 0)
			m_nHlRestSince = m_nHlCycle;
	}
	if (eSt != T::eRunning)		// a stop, or a generator error
	{
		EndHighLevelTest(eSt == T::eError ? eTestGenError : eTestAborted, astState.stFaults, astTorque.uAllocUpdate);
		return;
	}
	m_nHlCycle++;
	const double dT = m_stHlMeta.dPeriod > 0.0 ? m_stHlMeta.dPeriod : 1e-3;
	if (m_nHlSel <= 2)
	{
		/* until 1 s after the arm came to rest at the last goal (residual motion) */
		if (m_nHlRestSince >= 0 && (m_nHlSel == 1 || m_bHlJumped) && m_nHlCycle - m_nHlRestSince >= (long)std::lround(1.0 / dT))
			EndHighLevelTest(eTestCompleted, astState.stFaults, astTorque.uAllocUpdate);
		else if (m_nHlCycle >= (long)std::lround(20.0 / dT))
			EndHighLevelTest(eTestTimeout, astState.stFaults, astTorque.uAllocUpdate);
	}
	else if (m_nHlSel == HL_APP)
	{
		/* the application: until 'x' (or APP_DURATION_S) */
		if (m_dAppT > 0.0 && m_nHlCycle >= (long)std::lround(m_dAppT / dT))
			EndHighLevelTest(eTestCompleted, astState.stFaults, astTorque.uAllocUpdate);
	}
	else
	{
		const double dDur = (m_nHlSel == 5) ? m_dHlRefT : m_dHlNoiseT;
		if (m_nHlCycle >= (long)std::lround(dDur / dT))
			EndHighLevelTest(eTestCompleted, astState.stFaults, astTorque.uAllocUpdate);
	}
}

void
CRobotIndy7::EndHighLevelTest(int anState, const task_slots::ST_FAULTS& astFaults, unsigned long auAllocs)
{
	const CHighLevelRT::Status& st = m_cTraj.HighLevel().GetStatus();
	CHighLevelLog::Result stR;
	FillRtResult(stR, astFaults, m_stHlRt0, m_nHlCtrlFaults0, m_nHlDropouts0);
	stR.nAllocs = malloc_count::Enabled() ? (long)(auAllocs - m_uHlAllocs0) : -1;
	stR.nAccepted = st.nAccepted;
	stR.nConverged = st.nConverged;
	stR.nRejected = st.nRejected;
	stR.nLeadResets = st.nLeadResets;
	stR.dRestTime = (m_nHlRestSince >= 0) ? m_nHlRestSince * m_stHlMeta.dPeriod : -1.0;
	m_cHlLog.Close(anState, stR);
	m_eHlTest = eHlTestIdle;
	m_bHlTrackPending = false;
	// the motion ends with the test: the Reference slot of this period carries
	// no reference, and the Dynamics task returns to gravity compensation
	if (m_cTraj.IsRunning())
		m_cTraj.End(CTrajectoryRT::eAborted);
	DBG_LOG_INFO("[HL] %s %s - gravity compensation", s_apcHlName[m_nHlSel],
	             anState == eTestCompleted ? "done" : anState == eTestTimeout ? "ended: not at rest after 20 s" :
	             anState == eTestGenError ? "ended: trajectory generator error" : "stopped");
	/* the random goals of test 1 or the jumps of test 2: a completed run, or
	 * one that did not come to rest (a result), leads to the next; a stop or
	 * an error ends them */
	if (m_bHlSeq && (m_nHlSel == 1 || m_nHlSel == 2))
	{
		int& nIdx = (m_nHlSel == 2) ? m_nHlJumpIdx : m_nHlGoalIdx;
		const int nAll = (m_nHlSel == 2) ? m_nHlJumps : m_nHlGoals;
		if (anState == eTestCompleted || anState == eTestTimeout)
		{
			if (++nIdx < nAll)
			{
				m_bHlSeqNext = true;
				m_nHlStartWait = 0;
			}
			else
			{
				m_bHlSeq = false;
				nIdx = 0;
				printf("[HL] random goal%s: all %d done\n", m_nHlSel == 2 ? " jumps" : "s", nAll);
			}
		}
		else
			StopHighLevelSeq("at");
	}
}

//============================================================================
// The motions of the operator (keyboard task, Indy7Ctrl.cpp): 'm' a joint
// move to absolute joint angles (HOME with a target of the operator; the
// keyboard task checked them against the joint limits minus the margin),
// 't' a task move of the tool to a position (TRACK on goals of the operator;
// the keyboard task checked the reach with the IK). A task move stays on
// after the arm has arrived: every 't' is a new goal; 'x' ends it.
//============================================================================
void
CRobotIndy7::StartUserMove(const task_slots::ST_MOVE_REQ& astReq, const task_slots::ST_STATE_SLOT& astState,
                           const task_slots::ST_TORQUE_SLOT& astTorque)
{
	using namespace task_slots;
	if (astReq.eKind == eMoveTask && m_eUserMove == eUserTask && m_cTraj.IsRunning())
	{
		m_goalSlot.Write(astReq.adTarget);
		printf("[MOVE] new goal (%.1f, %.1f, %.1f) mm\n", astReq.adTarget[0] * 1e3, astReq.adTarget[1] * 1e3,
		       astReq.adTarget[2] * 1e3);
		return;
	}
	if (astReq.eKind != eMoveJoint && astReq.eKind != eMoveTask)
		return;
	if (FALSE == CheckMotionStart("MOVE", astState, astTorque))
		return;
	if (m_bHlSeqNext)
	{
		printf("[MOVE] refused - a sequence of test %d is between two runs ('x' first)\n", m_nHlSel);
		return;
	}
	char acWhy[192];
	if (astReq.eKind == eMoveJoint)
	{
		if (FALSE == m_cTraj.Start(true, astState.adQ, astState.adQd, m_bHlFF ? TRUE : FALSE, acWhy, sizeof(acWhy),
		                           astReq.adTarget))
		{
			printf("[MOVE] refused - %s\n", acWhy);
			return;
		}
		m_eUserMove = eUserJoint;
		const double d = 180.0 / M_PI;
		printf("[MOVE] joint move to [%.1f %.1f %.1f %.1f %.1f %.1f] deg - 'x' stops\n", astReq.adTarget[0] * d,
		       astReq.adTarget[1] * d, astReq.adTarget[2] * d, astReq.adTarget[3] * d, astReq.adTarget[4] * d,
		       astReq.adTarget[5] * d);
		return;
	}
	const double adOff[3] = {0.0, 0.0, 0.0};
	m_cTraj.SetGoalSlot(&m_goalSlot);
	m_cTraj.SetGoalOffset(adOff);
	m_cTraj.HighLevel().SetDelta(m_dHlDelta);
	if (FALSE == m_cTraj.Start(false, astState.adQ, astState.adQd, m_bHlFF ? TRUE : FALSE, acWhy, sizeof(acWhy)))
	{
		printf("[MOVE] refused - %s\n", acWhy);
		return;
	}
	// after the start: only goals written after it count
	m_goalSlot.Write(astReq.adTarget);
	m_eUserMove = eUserTask;
	printf("[MOVE] task move to (%.1f, %.1f, %.1f) mm: TRACK on the goals of the operator - 't' a new goal, "
	       "'x' ends\n", astReq.adTarget[0] * 1e3, astReq.adTarget[1] * 1e3, astReq.adTarget[2] * 1e3);
}

//============================================================================
// The Trajectory task (TASK1). In every period, after the release:
//   read     the State slot (the measurement of this period) and the Torque
//            slot (the state of the controller and its last torque);
//   command  the word of DoInput: a menu entry of 'y' starts HOME (then
//            TRACK), the stop ends the running motion; a motion of the
//            operator ('m', 't'); a selection of the camera application;
//   goal     the test program writes the goal of the period into the slot;
//   step     TRACK or HOME: the high-level controller turns the latest goal
//            and the measured state into the joint reference of this period
//            (paper, Sec. IV-C: goal admission, IK, trajectory generation,
//            lead bound); HOME at rest or a generator error ends the motion;
//   publish  the Reference slot, in every period (eRefNone without a
//            motion), with the time of the step and the timing of this cycle;
//   then     the log of the test and its state machine.
// It ends when the EtherCAT task has ended (after the stop).
//============================================================================
void
proc_trajectory_control(void* apRobot)
{
	using namespace task_slots;
	auto* pRobot = static_cast<CRobotIndy7*>(apRobot);
	CRtTask* const pTask = CRtTask::Self();
	DBG_LOG_INFO("(proc_trajectory_control) Trajectory task started (high-level controller %s)",
	             pRobot->m_cTraj.IsInit() ? "ready" : "unavailable: TRACK and HOME are refused");
	if (pTask == NULL)
	{
		pRobot->m_bTrajDone.store(true, std::memory_order_release);
		return;
	}

	ST_TASK_TIMING& st = pRobot->m_stTrajTiming;
	st.tPeriod = pTask->Period();
	ST_STATE_SLOT  stState;
	ST_TORQUE_SLOT stTorque;
	ST_REF_SLOT    stRef;
	ST_MOVE_REQ    stMove;
	memset(&stState, 0, sizeof(stState));
	memset(&stTorque, 0, sizeof(stTorque));
	memset(&stRef, 0, sizeof(stRef));
	memset(&stMove, 0, sizeof(stMove));
	uint32_t uLastPeriod = 0xFFFFFFFF;
	double adQ[CTrajectoryRT::MAX_DOF], adQd[CTrajectoryRT::MAX_DOF];
	double adQr[CTrajectoryRT::MAX_DOF], adQdr[CTrajectoryRT::MAX_DOF], adQddr[CTrajectoryRT::MAX_DOF];
	for (int i = 0; i < CTrajectoryRT::MAX_DOF; i++)
		adQ[i] = adQd[i] = adQr[i] = adQdr[i] = adQddr[i] = 0.0;

	while (TRUE)
	{
		/* release */
		const RTTIME tRelease = pTask->WaitNextRelease();
		const RTTIME tStart = read_timer();
		if (tStart - tRelease > st.tPeriod)
			st.nLate++;
		// the EtherCAT task has ended (1 s after the stop): so does this one
		if (pRobot->m_bEcatDone.load(std::memory_order_acquire))
			break;

		// malloc-count build: allocations of the last cycle of this task
		{
			static unsigned long s_uCycleStart = 0;
			const unsigned long uNow = malloc_count::Count();
			pRobot->m_uAllocTraj += uNow - s_uCycleStart;
			s_uCycleStart = uNow;
			malloc_count::Arm(true);
		}

		/* read */
		const bool bNewState = pRobot->m_stateSlot.Read(stState) && stState.uPeriod != uLastPeriod;
		pRobot->m_torqueSlot.Read(stTorque);
		if (bNewState)
		{
			uLastPeriod = stState.uPeriod;
			for (int i = 0; i < CTrajectoryRT::MAX_DOF && i < stState.nDof; i++)
			{
				adQ[i]  = stState.adQ[i];
				adQd[i] = stState.adQd[i];
			}
		}

		/* the command of the operator (DoInput, Dynamics task) */
		const int nCmd = pRobot->m_nHlCmd.exchange(HL_CMD_NONE, std::memory_order_acq_rel);
		if (nCmd == HL_CMD_STOP || (stState.bStop && pRobot->m_cTraj.IsRunning()))
			pRobot->StopHighLevel();
		else if (nCmd >= 1 && nCmd <= (int)CRobotIndy7::HL_MENU && bNewState)
			pRobot->StartHighLevelMenu(nCmd, stState, stTorque);
		if (bNewState && !stState.bStop)
		{
			/* a motion of the operator (keyboard task) */
			uint32_t uMoves = 0;
			if (pRobot->m_moveSlot.Read(stMove, &uMoves) && uMoves != pRobot->m_uMoveSeen)
			{
				pRobot->m_uMoveSeen = uMoves;
				if (nCmd == HL_CMD_STOP)
					printf("[MOVE] dropped: a stop came in the same period\n");
				else
					pRobot->StartUserMove(stMove, stState, stTorque);
			}
			/* the camera application: a selection of the object menu ('p',
			 * then a digit) makes the arm follow it */
			if (pRobot->m_pBridge != NULL && pRobot->m_pBridge->TakeFollowRequest())
			{
				if (pRobot->m_eHlTest != CRobotIndy7::eHlTestIdle && pRobot->m_nHlSel == CRobotIndy7::HL_APP)
					printf("[APP] the arm follows the new selection\n");
				else if (nCmd != HL_CMD_STOP)
					pRobot->StartHighLevelMenu(CRobotIndy7::HL_APP, stState, stTorque);
			}
		}

		/* the step: with a new state only (without one, the reference of the
		 * last state stands) */
		if (bNewState)
		{
			int32_t nHlNs = -1;
			int eMode = eRefNone;
			if (stState.bOp && !stState.bStop)
			{
				// the goal of this period, from the test program, into the slot
				pRobot->HighLevelTestBeforeUpdate();
				if (pRobot->m_cTraj.IsRunning())
				{
					const BOOL bOk = pRobot->m_cTraj.Update(adQ, adQd, adQr, adQdr, adQddr);
					nHlNs = Sat32((RTTIME)(pRobot->m_cTraj.HighLevel().GetStatus().dUs * 1e3));
					if (bOk)
						eMode = (pRobot->m_cTraj.GetMotion() == CTrajectoryRT::eMotionHome) ? eRefHome : eRefTrack;
					// HOME at rest, or a generator error: the motion ends with
					// this period, and the Dynamics task holds in gravity
					// compensation
					if (pRobot->m_cTraj.Pending() != CTrajectoryRT::eRunning)
					{
						pRobot->m_cTraj.End(pRobot->m_cTraj.Pending());
						eMode = eRefNone;
					}
				}
			}
			stRef.uPeriod = stState.uPeriod;
			stRef.eMode = eMode;
			stRef.bFrictionFF = pRobot->m_cTraj.GetFrictionFF();
			for (int i = 0; i < MAX_DOF; i++)
			{
				const bool bIn = (i < CTrajectoryRT::MAX_DOF);
				stRef.adQRef[i]   = (eMode != eRefNone && bIn) ? adQr[i] : 0.0;
				stRef.adQdRef[i]  = (eMode != eRefNone && bIn) ? adQdr[i] : 0.0;
				stRef.adQddRef[i] = (eMode != eRefNone && bIn) ? adQddr[i] : 0.0;
			}
			stRef.nHlNs = nHlNs;
		}

		/* the end of a motion of the operator */
		if (pRobot->m_eUserMove != CRobotIndy7::eUserNone && !pRobot->m_cTraj.IsRunning())
		{
			const CTrajectoryRT::eState eSt = pRobot->m_cTraj.GetState();
			printf("[MOVE] %s - gravity compensation\n",
			       eSt == CTrajectoryRT::eDoneAtRest ? "at the target" :
			       eSt == CTrajectoryRT::eError ? "ended: trajectory generator error" : "stopped");
			pRobot->m_eUserMove = CRobotIndy7::eUserNone;
		}

		/* publish: the reference of this period for the Dynamics task, in
		 * every cycle (the state of the test and the timing are of this cycle) */
		const RTTIME tEnd0 = read_timer();
		const bool bHlTest = (pRobot->m_eHlTest != CRobotIndy7::eHlTestIdle);
		stRef.bTest = (bHlTest || pRobot->m_eUserMove != CRobotIndy7::eUserNone) ? TRUE : FALSE;
		stRef.nHlTest = bHlTest ? pRobot->m_nHlSel : 0;
		stRef.nW = Sat32(tStart - tRelease);
		stRef.nE = Sat32(tEnd0 - tStart);
		pRobot->m_refSlot.Write(stRef);

		/* then: the log of the test, HOME -> TRACK, the end of the test */
		if (bNewState)
			pRobot->HighLevelTestAfterUpdate(stState, stTorque);

		/* the timing of this task */
		const RTTIME tEnd = read_timer();
		const RTTIME tW = tStart - tRelease, tE = tEnd - tStart;
		st.nCycles++;
		st.dWSum += (double)tW;
		st.dESum += (double)tE;
		if (tW > st.tWMax) st.tWMax = tW;
		if (tE > st.tEMax) st.tEMax = tE;
		if (tW + tE > st.tPeriod)
			st.nOverruns++;
	}
	// a test still open at the end: its log closes as stopped
	if (pRobot->m_eHlTest != CRobotIndy7::eHlTestIdle)
		pRobot->EndHighLevelTest(eTestAborted, stState.stFaults, stTorque.uAllocUpdate);
	DBG_LOG_INFO("(proc_trajectory_control) task ended");
	pRobot->m_bTrajDone.store(true, std::memory_order_release);
}
