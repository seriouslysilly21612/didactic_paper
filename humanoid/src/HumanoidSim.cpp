/*****************************************************************************
*	Name: HumanoidSim.cpp
*	Description: The computations of the controller reused on a humanoid in
*	simulation (paper, Sec. VI, Table 12, Fig. 11). The Unitree G1 with 29
*	joints, the pelvis fixed to the world: the computed-torque law (eq. (5))
*	for all joints with RBDL on the model of g1_fixed.urdf, and the
*	high-level controller of Sec. IV-C (CHighLevelRTN<7>, the code of the
*	arm, indy7/src/HighLevelRT.h) for each hand, with one IK model instance
*	per hand (g1_<side>_arm.urdf). The simulator (MuJoCo, the model converted
*	with tools/urdf2mjcf.py) takes the place of the EtherCAT exchange, with
*	the timing of the simulation of the arm: the state is 2 steps old, the
*	torque acts 2 steps after it. One periodic real-time task at 1 kHz on
*	RT-POSIX (indy7/src/base/RtTask.h: an absolute release grid, the timing
*	measured from the planned release, a late release not skipped).
*
*	  bin/humanoid_sim --models DIR [--mode check|joint|hl] [--out DIR]
*	                   [--cpu N] [--prio P] [--goals N] [--seed S] [--duration S]
*	  check  RBDL against MuJoCo over random states (both hands, M, h), and the
*	         IK models of the hands against the full model; no real time
*	  joint  the joint test on the arms: both arms, shoulder pitch and roll,
*	         elbow and wrist pitch, out in 2 s (a quintic polynomial from rest
*	         to rest), a rest of 1 s, back, a rest; the legs and the waist held;
*	         every period logged (joint_test.csv; humanoid/tools/hum_analyze.py
*	         compares it with eq. (8))
*	  hl     both hands follow random goals in front of the robot (drawn
*	         before the task, solvable from q_r), the next goal 0.3 s after the
*	         hand is at rest; per goal: time to rest and final distance; with
*	         --duration S the goal stream runs S seconds
*	Every mode with real time reports t_w, t_e (start to the end of the
*	control law, without the simulator step), the high-level controllers, the
*	control law, and the simulator step (median, 99.9, 99.999 %, max; Table
*	12), and the timing faults: late releases (t_w > T_s) and overruns
*	(t_w + t_e > T_s). G2T_CALL_TIMING=1: the times of the library calls
*	(indy7/src/CallTiming.h), in a separate run.
*****************************************************************************/
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <mujoco/mujoco.h>
#include <Eigen/Dense>

#include <sys/mman.h>
#include <cerrno>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include "base/Common.h"
#include "base/RtTask.h"
#include "CallTiming.h"
#include "HighLevelRT.h"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

namespace {

typedef CHighLevelRTN<7> HL;
const int    NA        = 7;          // joints per arm
const int    MAXJ      = 32;
const int    MAXD      = 8;          // delay buffers [steps]
const int    STATE_DELAY = 2, TORQUE_DELAY = 2;
const double T_S       = 0.001;      // [s]
const int    HBINS     = 5000;       // 1 us bins up to 5 ms

/* ---------------------------------------------------------------- histogram */
struct Hist
{
	const char* name;
	uint64_t n = 0, over = 0;
	double sum = 0.0, max = 0.0;
	uint32_t bins[HBINS] = {};
	void Add(double us)
	{
		n++;
		sum += us;
		if (us > max) max = us;
		const int b = (int)us;
		if (b >= 0 && b < HBINS) bins[b]++; else over++;
	}
	double Pct(double p) const		// upper edge of the bin of the p-quantile [us]
	{
		if (!n) return 0.0;
		const double r = p * (double)n;
		uint64_t c = 0;
		for (int b = 0; b < HBINS; b++)
		{
			c += bins[b];
			if ((double)c >= r && c > 0) return std::min((double)(b + 1), max);
		}
		return max;
	}
};

/* ---------------------------------------------------------------- the model */
struct Robot
{
	Model model;					// CTC: the whole robot, pelvis fixed
	int nj = 0;
	std::string jname[MAXJ];		// MuJoCo joint name of RBDL joint i (controller order)
	std::string link[MAXJ];			// child link (RBDL body name)
	double lo[MAXJ], hi[MAXJ], effort[MAXJ], kp[MAXJ], kd[MAXJ], qr[MAXJ];
	int arm[2][NA];					// controller index of the 7 joints of each arm (IK order)
	unsigned hand[2];				// RBDL body of each tool point
	const char* tool[2] = { "left_rubber_hand", "right_rubber_hand" };
};

/* ---------------------------------------------------------------- the simulator */
struct Sim
{
	mjModel* m = nullptr;
	mjData* d = nullptr;
	int qa[MAXJ], va[MAXJ];
	int nj = 0;
	long k = 0;
	double histQ[MAXD + 1][MAXJ], histQd[MAXD + 1][MAXJ], tauQ[MAXD + 1][MAXJ];
	static int Slot(long i) { return (int)(((i % (MAXD + 1)) + (MAXD + 1)) % (MAXD + 1)); }
	void Measured(double* q, double* qd) const
	{
		const int s = Slot(k - STATE_DELAY);
		for (int i = 0; i < nj; i++) { q[i] = histQ[s][i]; qd[i] = histQd[s][i]; }
	}
	void Step(const double* tau)
	{
		const int sIn = Slot(k);
		for (int i = 0; i < nj; i++) tauQ[sIn][i] = tau[i];
		const int sOut = Slot(k - (TORQUE_DELAY - STATE_DELAY));
		for (int i = 0; i < nj; i++) d->qfrc_applied[va[i]] = tauQ[sOut][i];
		mj_step(m, d);
		const int sNew = Slot(k + 1);
		for (int i = 0; i < nj; i++) { histQ[sNew][i] = d->qpos[qa[i]]; histQd[sNew][i] = d->qvel[va[i]]; }
		k++;
	}
};

/* ---------------------------------------------------------------- arguments */
struct Args
{
	std::string models = "models", mode = "hl", out = ".";
	int cpu = -1, prio = 97, goals = 50;
	unsigned seed = 1;
	double duration = 0.0;
};

/* the gains follow the rule of Table 7, Kp = wn^2, Kd = 2 zeta wn with zeta about 1:
 * wn = 20 rad/s for the legs, the waist, and the shoulders, 14.1 for the elbows, 10 for the wrists */
void Gains(const std::string& n, double* kp, double* kd)
{
	if (n.find("elbow") != std::string::npos)      { *kp = 200.0; *kd = 28.0; }
	else if (n.find("wrist") != std::string::npos) { *kp = 100.0; *kd = 20.0; }
	else                                           { *kp = 400.0; *kd = 40.0; }
}

/* the ready posture: arms bent in front of the robot, legs and waist at 0 */
double Ready(const std::string& n)
{
	const double s = (n.rfind("left", 0) == 0) ? 1.0 : -1.0;
	if (n.find("shoulder_pitch") != std::string::npos) return -0.3;
	if (n.find("shoulder_roll") != std::string::npos)  return 0.25 * s;
	if (n.find("elbow") != std::string::npos)          return 0.9;
	return 0.0;
}

bool LoadRobot(const Args& a, Robot& r, Sim& sim)
{
	const std::string urdf = a.models + "/g1_fixed.urdf", mjcf = a.models + "/g1_mujoco.xml";
	if (!Addons::URDFReadFromFile(urdf.c_str(), &r.model, false, false))
	{
		fprintf(stderr, "cannot read %s\n", urdf.c_str());
		return false;
	}
	r.nj = (int)r.model.qdot_size;
	if (r.nj > MAXJ)
	{
		fprintf(stderr, "%d joints, at most %d\n", r.nj, MAXJ);
		return false;
	}
	char err[1024] = {0};
	sim.m = mj_loadXML(mjcf.c_str(), nullptr, err, sizeof(err));
	if (!sim.m)
	{
		fprintf(stderr, "cannot load %s: %s\n", mjcf.c_str(), err);
		return false;
	}
	sim.m->opt.timestep = T_S;
	sim.d = mj_makeData(sim.m);
	if (sim.m->nv != r.nj)
	{
		fprintf(stderr, "RBDL %d joints, MuJoCo %d\n", r.nj, (int)sim.m->nv);
		return false;
	}
	sim.nj = r.nj;
	/* match the joints by the child link: RBDL body name = MuJoCo body name of the joint */
	int found = 0;
	for (unsigned b = 1; b < r.model.mBodies.size(); b++)
	{
		const unsigned qi = r.model.mJoints[b].q_index;
		if (r.model.mJoints[b].mDoFCount != 1)
			continue;
		const std::string name = r.model.GetBodyName(b);
		for (int j = 0; j < sim.m->njnt; j++)
		{
			const int body = sim.m->jnt_bodyid[j];
			const char* bn = mj_id2name(sim.m, mjOBJ_BODY, body);
			if (bn && name == bn)
			{
				r.link[qi] = name;
				r.jname[qi] = mj_id2name(sim.m, mjOBJ_JOINT, j);
				sim.qa[qi] = sim.m->jnt_qposadr[j];
				sim.va[qi] = sim.m->jnt_dofadr[j];
				r.lo[qi] = sim.m->jnt_range[2 * j];
				r.hi[qi] = sim.m->jnt_range[2 * j + 1];
				found++;
				break;
			}
		}
	}
	if (found != r.nj)
	{
		fprintf(stderr, "matched %d of %d joints between RBDL and MuJoCo\n", found, r.nj);
		return false;
	}
	/* effort limits from the URDF (MuJoCo does not keep them) */
	FILE* fp = fopen((a.models + "/g1_joints.txt").c_str(), "r");
	for (int i = 0; i < r.nj; i++) r.effort[i] = 0.0;
	if (fp)
	{
		char line[256];
		while (fgets(line, sizeof(line), fp))
		{
			int idx;
			char nm[64];
			double lo, hi, eff, vel;
			if (sscanf(line, "%d %63s %lf %lf %lf %lf", &idx, nm, &lo, &hi, &eff, &vel) == 6)
				for (int i = 0; i < r.nj; i++)
					if (r.jname[i] == nm) r.effort[i] = eff;
		}
		fclose(fp);
	}
	for (int i = 0; i < r.nj; i++)
	{
		if (!(r.effort[i] > 0.0))
		{
			fprintf(stderr, "no effort limit for %s (g1_joints.txt)\n", r.jname[i].c_str());
			return false;
		}
		Gains(r.jname[i], &r.kp[i], &r.kd[i]);
		r.qr[i] = Ready(r.jname[i]);
	}
	/* the arms in the order of their IK models */
	for (int h = 0; h < 2; h++)
	{
		const char* side = h == 0 ? "left" : "right";
		static const char* const parts[NA] = { "shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
		                                       "wrist_roll", "wrist_pitch", "wrist_yaw" };
		for (int k = 0; k < NA; k++)
		{
			r.arm[h][k] = -1;
			const std::string want = std::string(side) + "_" + parts[k] + "_joint";
			for (int i = 0; i < r.nj; i++)
				if (r.jname[i] == want) r.arm[h][k] = i;
			if (r.arm[h][k] < 0)
			{
				fprintf(stderr, "no joint %s\n", want.c_str());
				return false;
			}
		}
		r.hand[h] = r.model.GetBodyId(r.tool[h]);
		if (r.hand[h] == std::numeric_limits<unsigned int>::max())
		{
			fprintf(stderr, "no body %s in %s\n", r.tool[h], urdf.c_str());
			return false;
		}
	}
	/* the simulator starts at q_r, at rest */
	for (int i = 0; i < r.nj; i++) sim.d->qpos[sim.qa[i]] = r.qr[i];
	mju_zero(sim.d->qvel, sim.m->nv);
	mj_forward(sim.m, sim.d);
	for (int s = 0; s <= MAXD; s++)
		for (int i = 0; i < r.nj; i++) { sim.histQ[s][i] = r.qr[i]; sim.histQd[s][i] = 0.0; sim.tauQ[s][i] = 0.0; }
	return true;
}

bool InitHand(const Args& a, const Robot& r, int h, HL& hl)
{
	HL::Params p = HL::DefaultParams();		// delta, IK, limits of the generator, lead, rest: as the arm's
	snprintf(p.acTool, sizeof(p.acTool), "%s", r.tool[h]);
	for (int k = 0; k < NA; k++)
	{
		const int i = r.arm[h][k];
		p.adQr[k] = r.qr[i];
		p.adLimLo[k] = r.lo[i];
		p.adLimHi[k] = r.hi[i];
	}
	const std::string urdf = a.models + (h == 0 ? "/g1_left_arm.urdf" : "/g1_right_arm.urdf");
	return hl.Init(urdf.c_str(), T_S, p) == TRUE;
}

/* ---------------------------------------------------------------- check */
int Check(const Args& a, Robot& r, Sim& sim)
{
	const int n = r.nj;
	std::mt19937_64 rng(a.seed);
	std::uniform_real_distribution<double> u(0.0, 1.0);
	VectorNd q = VectorNd::Zero(n), qd = VectorNd::Zero(n), zero = VectorNd::Zero(n), h = VectorNd::Zero(n);
	MatrixNd M = MatrixNd::Zero(n, n);
	std::vector<double> Mm(sim.m->nv * sim.m->nv), vE(sim.m->nv, 0.0), vCol(sim.m->nv);
	double worst[4] = {0, 0, 0, 0};		// hands [m], M rel, h rel, IK models [m]
	HL hl[2];
	for (int hh = 0; hh < 2; hh++)
		if (!InitHand(a, r, hh, hl[hh])) return 2;
	const int N = 10000;
	for (int s = 0; s < N; s++)
	{
		for (int i = 0; i < n; i++)
		{
			q[i] = r.lo[i] + (r.hi[i] - r.lo[i]) * u(rng);
			qd[i] = -M_PI + 2.0 * M_PI * u(rng);
		}
		// the IK models assume the waist at 0: every second state has it there
		if (s % 2 == 0)
			for (int i = 0; i < n; i++)
				if (r.jname[i].rfind("waist", 0) == 0) q[i] = 0.0;
		for (int i = 0; i < n; i++) { sim.d->qpos[sim.qa[i]] = q[i]; sim.d->qvel[sim.va[i]] = qd[i]; }
		mj_forward(sim.m, sim.d);
		for (int j = 0; j < sim.m->nv; j++)		// M column by column (mj_mulM: the same in every MuJoCo 3)
		{
			vE[j] = 1.0;
			mj_mulM(sim.m, sim.d, vCol.data(), vE.data());
			vE[j] = 0.0;
			for (int i = 0; i < sim.m->nv; i++)
				Mm[i * sim.m->nv + j] = vCol[i];
		}
		UpdateKinematics(r.model, q, qd, zero);
		M.setZero();
		CompositeRigidBodyAlgorithm(r.model, q, M, false);
		NonlinearEffects(r.model, q, qd, h);
		double dM = 0, nM = 0, dh = 0, nh = 0;
		for (int i = 0; i < n; i++)
		{
			for (int j = 0; j < n; j++)
			{
				const double ref = Mm[sim.va[i] * sim.m->nv + sim.va[j]];
				dM += (M(i, j) - ref) * (M(i, j) - ref);
				nM += ref * ref;
			}
			const double ref = sim.d->qfrc_bias[sim.va[i]];
			dh += (h[i] - ref) * (h[i] - ref);
			nh += ref * ref;
		}
		worst[1] = std::max(worst[1], std::sqrt(dM / nM));
		worst[2] = std::max(worst[2], std::sqrt(dh / nh));
		for (int hh = 0; hh < 2; hh++)
		{
			const Vector3d p = CalcBodyToBaseCoordinates(r.model, q, r.hand[hh], Vector3d::Zero(), false);
			const int b = mj_name2id(sim.m, mjOBJ_BODY, r.tool[hh]);
			for (int k = 0; k < 3; k++) worst[0] = std::max(worst[0], std::fabs(p[k] - sim.d->xpos[3 * b + k]));
			if (s % 2 == 0)
			{
				double qa[NA], pa[3];
				for (int k = 0; k < NA; k++) qa[k] = q[r.arm[hh][k]];
				hl[hh].ToolPosition(qa, pa);
				for (int k = 0; k < 3; k++) worst[3] = std::max(worst[3], std::fabs(pa[k] - p[k]));
			}
		}
	}
	const bool ok = worst[0] < 1e-8 && worst[1] < 1e-7 && worst[2] < 1e-7 && worst[3] < 1e-8;
	printf("check: %d random states, %d joints (RBDL %s vs MuJoCo %s)\n", N, n, "g1_fixed.urdf", "g1_mujoco.xml");
	printf("  hand positions            %.3e m   (limit 1e-8)\n", worst[0]);
	printf("  M(q), relative            %.3e     (limit 1e-7)\n", worst[1]);
	printf("  h(q, qd), relative        %.3e     (limit 1e-7)\n", worst[2]);
	printf("  IK models of the hands    %.3e m   (waist at 0; limit 1e-8)\n", worst[3]);
	printf("check: %s\n", ok ? "PASS" : "FAIL");
	return ok ? 0 : 1;
}

/* ---------------------------------------------------------------- the task */
struct Joint			// the joint test on the arms
{
	int idx[8];
	double dq[8];
	int n = 0;
	double T = 2.0, dwell = 1.0;
};

struct GoalRun
{
	int hand, index;
	double t0, rest, dist;
	long accepted, rejected;
};

struct Task
{
	Args a;
	Robot* r;
	Sim* sim;
	HL hl[2];
	// state and reference, controller order (preallocated)
	VectorNd q, qd, qref, qdref, qddref, zero, qdd, h, tauM, tau;
	MatrixNd M;
	double tauOut[MAXJ];
	// high-level goals
	std::vector<std::array<double, 3>> goals[2];
	int goalIdx[2] = {0, 0};
	long tGoal[2] = {0, 0}, tRest[2] = {-1, -1};
	bool waiting[2] = {false, false};
	long nextAt[2] = {0, 0};
	std::vector<GoalRun> runs;
	// joint test
	Joint jt;
	std::vector<float> log;			// per period: t, then qref, qdref, qddref, q, e, tau for every joint
	int logCols = 0;
	long logRows = 0, logMax = 0;
	// timing
	Hist hw{"t_w (release to start)"}, he{"t_e (start to end of the control law)"}, hhl{"high-level, both hands"},
	     hctc{"control law (kinematics, M, h, torque)"}, hsim{"simulator step"}, hwe{"t_w + t_e"};
	long late = 0, overruns = 0, cycles = 0, clamped = 0;
	long n = 0, nEnd = 0;
	std::atomic<bool> done{false};		// the task is asked to end
	std::atomic<bool> ended{false};		// the task has ended
};

void Reference(Task& T, long n)
{
	Robot& r = *T.r;
	for (int i = 0; i < r.nj; i++) { T.qref[i] = r.qr[i]; T.qdref[i] = 0.0; T.qddref[i] = 0.0; }
	if (T.a.mode == "joint")
	{
		const double t = n * T_S, TT = T.jt.T, W = T.jt.dwell;
		double s = 0.0, sign = 0.0, base = 0.0;
		if (t < TT) { s = t / TT; sign = 1.0; }
		else if (t < TT + W) { base = 1.0; }
		else if (t < 2 * TT + W) { s = (t - TT - W) / TT; sign = -1.0; base = 1.0; }
		const double p = 10 * s * s * s - 15 * s * s * s * s + 6 * s * s * s * s * s;
		const double pd = (30 * s * s - 60 * s * s * s + 30 * s * s * s * s) / TT;
		const double pdd = (60 * s - 180 * s * s + 120 * s * s * s) / (TT * TT);
		for (int k = 0; k < T.jt.n; k++)
		{
			const int i = T.jt.idx[k];
			const double dq = T.jt.dq[k];
			T.qref[i] = r.qr[i] + (sign > 0 ? dq * p : (sign < 0 ? dq - dq * p : base * dq));
			T.qdref[i] = sign * dq * pd;
			T.qddref[i] = sign * dq * pdd;
		}
		return;
	}
	/* hl: the two hands; the legs and the waist hold q_r */
	for (int hh = 0; hh < 2; hh++)
	{
		double qa[NA], qda[NA], qr[NA], qdr[NA], qddr[NA];
		for (int k = 0; k < NA; k++) { qa[k] = T.q[r.arm[hh][k]]; qda[k] = T.qd[r.arm[hh][k]]; }
		const double* goal = nullptr;
		if (T.goalIdx[hh] < (int)T.goals[hh].size() && !T.waiting[hh] && n >= T.nextAt[hh])
			goal = T.goals[hh][T.goalIdx[hh]].data();
		T.hl[hh].Update(goal, qa, qda, qr, qdr, qddr);
		for (int k = 0; k < NA; k++)
		{
			const int i = r.arm[hh][k];
			T.qref[i] = qr[k]; T.qdref[i] = qdr[k]; T.qddref[i] = qddr[k];
		}
		const HL::Status& st = T.hl[hh].GetStatus();
		if (goal && st.bGoalAccepted) T.tGoal[hh] = n;
		if (goal && st.bHasGoal && st.bAtRest && n > T.tGoal[hh])
		{
			// at rest at this goal: the result, then the next goal after 0.3 s
			double pa[3];
			T.hl[hh].ToolPosition(qa, pa);
			const double* g = T.goals[hh][T.goalIdx[hh]].data();
			const double dist = std::sqrt((pa[0] - g[0]) * (pa[0] - g[0]) + (pa[1] - g[1]) * (pa[1] - g[1]) +
			                              (pa[2] - g[2]) * (pa[2] - g[2]));
			if (T.runs.size() < T.runs.capacity())
				T.runs.push_back({hh, T.goalIdx[hh], T.tGoal[hh] * T_S, (n - T.tGoal[hh]) * T_S, dist,
				                  st.nAccepted, st.nRejected});
			T.goalIdx[hh]++;
			T.nextAt[hh] = n + 300;
			T.hl[hh].StartTrack(qa, qda);		// a new motion from the measured state, as TRACK
		}
	}
}

void Control(Task& T)
{
	Robot& r = *T.r;
	const RTTIME t0 = call_timing::Now();
	UpdateKinematics(r.model, T.q, T.qd, T.zero);
	RTTIME tc = call_timing::Lap(call_timing::eCtcKin, t0);
	T.M.setZero();
	CompositeRigidBodyAlgorithm(r.model, T.q, T.M, false);
	tc = call_timing::Lap(call_timing::eCtcCrba, tc);
	NonlinearEffects(r.model, T.q, T.qd, T.h);
	call_timing::Lap(call_timing::eCtcNle, tc);
	for (int i = 0; i < r.nj; i++)
		T.qdd[i] = T.qddref[i] + r.kp[i] * (T.qref[i] - T.q[i]) + r.kd[i] * (T.qdref[i] - T.qd[i]);
	T.tauM.noalias() = T.M * T.qdd;
	T.tau = T.tauM + T.h;
	bool clamp = false;
	for (int i = 0; i < r.nj; i++)
	{
		double v = T.tau[i];
		if (!std::isfinite(v)) v = 0.0;
		if (v > r.effort[i]) { v = r.effort[i]; clamp = true; }
		if (v < -r.effort[i]) { v = -r.effort[i]; clamp = true; }
		T.tauOut[i] = v;
	}
	if (clamp) T.clamped++;
}

void TaskBody(void* arg)
{
	Task& T = *(Task*)arg;
	Robot& r = *T.r;
	CRtTask* self = CRtTask::Self();
	const double dPeriodUs = (double)self->Period() * 1e-3;
	double qm[MAXJ], qdm[MAXJ];
	while (!T.done.load())
	{
		const RTTIME tRel = self->WaitNextRelease();
		const RTTIME tStart = read_timer();
		const double tw = (double)(int64_t)(tStart - tRel) * 1e-3;
		if (tw > dPeriodUs) T.late++;
		T.sim->Measured(qm, qdm);
		for (int i = 0; i < r.nj; i++) { T.q[i] = qm[i]; T.qd[i] = qdm[i]; }
		const RTTIME tH0 = read_timer();
		const RTTIME tC0 = call_timing::Now();
		Reference(T, T.n);
		const RTTIME tH1 = read_timer();
		Control(T);
		call_timing::Lap(call_timing::eControl, tC0);
		const RTTIME tEnd = read_timer();
		T.sim->Step(T.tauOut);
		const RTTIME tSim = read_timer();
		const double te = (double)(tEnd - tStart) * 1e-3;
		T.hw.Add(tw);
		T.he.Add(te);
		T.hwe.Add(tw + te);
		T.hhl.Add((double)(tH1 - tH0) * 1e-3);
		T.hctc.Add((double)(tEnd - tH1) * 1e-3);
		T.hsim.Add((double)(tSim - tEnd) * 1e-3);
		if (tw + te > dPeriodUs) T.overruns++;
		T.cycles++;
		if (T.a.mode == "joint" && T.logRows < T.logMax)
		{
			float* row = &T.log[(size_t)T.logRows * T.logCols];
			int c = 0;
			row[c++] = (float)(T.n * T_S);
			for (int i = 0; i < r.nj; i++) row[c++] = (float)T.qref[i];
			for (int i = 0; i < r.nj; i++) row[c++] = (float)T.qdref[i];
			for (int i = 0; i < r.nj; i++) row[c++] = (float)T.qddref[i];
			for (int i = 0; i < r.nj; i++) row[c++] = (float)T.q[i];
			for (int i = 0; i < r.nj; i++) row[c++] = (float)(T.qref[i] - T.q[i]);
			for (int i = 0; i < r.nj; i++) row[c++] = (float)T.tauOut[i];
			T.logRows++;
		}
		T.n++;
		bool end = (T.nEnd > 0 && T.n >= T.nEnd);
		if (T.a.mode == "hl" && T.goalIdx[0] >= (int)T.goals[0].size() && T.goalIdx[1] >= (int)T.goals[1].size())
			end = true;
		if (end) T.done.store(true);
	}
	T.ended.store(true);
}

void DrawGoals(Task& T)
{
	Robot& r = *T.r;
	std::mt19937_64 rng(T.a.seed);
	std::uniform_real_distribution<double> u(-1.0, 1.0);
	for (int hh = 0; hh < 2; hh++)
	{
		HL ik;
		InitHand(T.a, r, hh, ik);
		double qa[NA], p0[3];
		for (int k = 0; k < NA; k++) qa[k] = r.qr[r.arm[hh][k]];
		ik.ToolPosition(qa, p0);
		long draws = 0, noik = 0;
		while ((int)T.goals[hh].size() < T.a.goals && draws < 1000L * T.a.goals)
		{
			draws++;
			// a box of 0.30 x 0.30 x 0.30 m around the hand at q_r, not across the middle plane
			std::array<double, 3> g = { p0[0] + 0.15 * u(rng), p0[1] + 0.15 * u(rng), p0[2] + 0.15 * u(rng) };
			if ((hh == 0 && g[1] < 0.05) || (hh == 1 && g[1] > -0.05))
				continue;
			int it = 0;
			double e = 0.0;
			if (ik.SolveIk(g.data(), nullptr, &it, &e) != TRUE) { noik++; continue; }
			T.goals[hh].push_back(g);
		}
		printf("goals of the %s hand: %zu (seed %u), around [%.3f %.3f %.3f] m, %ld draws, %ld not solved\n",
		       hh == 0 ? "left" : "right", T.goals[hh].size(), T.a.seed, p0[0], p0[1], p0[2], draws, noik);
	}
}

int Run(const Args& a, Robot& r, Sim& sim)
{
	Task* pT = new Task();
	Task& T = *pT;
	T.a = a;
	T.r = &r;
	T.sim = &sim;
	const int n = r.nj;
	T.q = VectorNd::Zero(n); T.qd = VectorNd::Zero(n); T.qref = VectorNd::Zero(n); T.qdref = VectorNd::Zero(n);
	T.qddref = VectorNd::Zero(n); T.zero = VectorNd::Zero(n); T.qdd = VectorNd::Zero(n); T.h = VectorNd::Zero(n);
	T.tauM = VectorNd::Zero(n); T.tau = VectorNd::Zero(n); T.M = MatrixNd::Zero(n, n);
	for (int hh = 0; hh < 2; hh++)
	{
		if (!InitHand(a, r, hh, T.hl[hh]))
			return 2;
		double qa[NA], z[NA] = {0};
		for (int k = 0; k < NA; k++) qa[k] = r.qr[r.arm[hh][k]];
		T.hl[hh].StartTrack(qa, z);
	}
	if (a.mode == "joint")
	{
		/* both arms: shoulder pitch -0.6 (forward), roll 0.3 (outward), elbow 0.6, wrist pitch 0.4 rad */
		const char* parts[4] = { "shoulder_pitch", "shoulder_roll", "elbow", "wrist_pitch" };
		const double dq[4] = { -0.6, 0.3, 0.6, 0.4 };
		for (int hh = 0; hh < 2; hh++)
			for (int k = 0; k < 4; k++)
			{
				const std::string want = std::string(hh == 0 ? "left_" : "right_") + parts[k] + "_joint";
				for (int i = 0; i < n; i++)
					if (r.jname[i] == want)
					{
						T.jt.idx[T.jt.n] = i;
						T.jt.dq[T.jt.n] = (k == 1 && hh == 1) ? -dq[k] : dq[k];
						T.jt.n++;
					}
			}
		T.nEnd = (long)std::lround((2 * T.jt.T + 2 * T.jt.dwell) / T_S);
		T.logCols = 1 + 6 * n;
		T.logMax = T.nEnd;
		T.log.assign((size_t)T.logMax * T.logCols, 0.0f);
	}
	else
	{
		DrawGoals(T);
		T.runs.reserve(2 * a.goals + 2);
		T.nEnd = a.duration > 0 ? (long)std::lround(a.duration / T_S) : 0;
	}
	call_timing::Init();
	if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
	{
		fprintf(stderr, "mlockall failed: %s (the memlock limit must be unlimited)\n", strerror(errno));
		return 2;
	}
	/* one periodic task, SCHED_FIFO a.prio, 1 ms, on a.cpu (-1: the CPUs of the process) */
	CRtTask task;
	CRtTask::Spec spec;
	spec.strName = "humanoid";
	spec.nPriority = a.prio;
	spec.nCpu = a.cpu;
	spec.tPeriod = 1000000;
	spec.tStartDelay = 0;
	if (!task.Create(spec, CRtTask::GridOrigin(100000000)) || !task.Start(TaskBody, &T))
	{
		fprintf(stderr, "the real-time task cannot start (the rtprio limit must be at least %d)\n", a.prio);
		return 2;
	}
	while (task.GetCheck() == CRtTask::eCheckPending)
		usleep(1000);
	if (task.GetCheck() != CRtTask::eCheckOk)
	{
		fprintf(stderr, "the real-time task does not run as requested (see above)\n");
		T.done.store(true);
	}
	while (!T.ended.load())
		usleep(100000);

	/* outputs */
	const std::string out = a.out;
	FILE* fs = fopen((out + "/summary_" + a.mode + ".txt").c_str(), "w");
	auto both = [&](const char* fmt, auto... args) {
		printf(fmt, args...);
		if (fs) fprintf(fs, fmt, args...);
	};
	both("humanoid (Unitree G1, %d joints, pelvis fixed), mode %s: %ld periods of 1 ms (cpu %d, prio %d)\n", n,
	     a.mode.c_str(), T.cycles, a.cpu, a.prio);
	both("%-40s %9s %9s %9s %9s %9s %9s\n", "[us]", "median", "99.9 %", "99.999 %", "max", "mean", "samples");
	for (Hist* hp : { &T.hw, &T.he, &T.hhl, &T.hctc, &T.hsim, &T.hwe })
		both("%-40s %9.1f %9.1f %9.1f %9.1f %9.2f %9llu\n", hp->name, hp->Pct(0.5), hp->Pct(0.999),
		     hp->Pct(0.99999), hp->max, hp->n ? hp->sum / hp->n : 0.0, (unsigned long long)hp->n);
	both("timing faults: %ld late releases (t_w above the period), %ld overruns (t_w + t_e above the period)\n",
	     T.late, T.overruns);
	both("periods with a torque at its limit: %ld\n", T.clamped);
	if (a.mode == "hl")
	{
		std::vector<double> d[2], tr[2];
		long rej = 0;
		FILE* fg = fopen((out + "/hl_goals.csv").c_str(), "w");
		if (fg) fprintf(fg, "hand,index,t_goal_s,time_to_rest_s,final_distance_m,accepted,rejected\n");
		for (const GoalRun& g : T.runs)
		{
			d[g.hand].push_back(g.dist * 1e3);
			tr[g.hand].push_back(g.rest);
			rej += g.rejected;
			if (fg) fprintf(fg, "%d,%d,%.3f,%.3f,%.9f,%ld,%ld\n", g.hand, g.index, g.t0, g.rest, g.dist, g.accepted,
			                g.rejected);
		}
		if (fg) fclose(fg);
		for (int hh = 0; hh < 2; hh++)
		{
			if (d[hh].empty()) continue;
			std::vector<double> v = d[hh];
			std::sort(v.begin(), v.end());
			both("%s hand: %zu goals reached of %zu, final distance median %.4f, max %.4f mm\n",
			     hh == 0 ? "left" : "right", d[hh].size(), T.goals[hh].size(), v[v.size() / 2], v.back());
		}
		both("IK rejections: %ld\n", rej);
	}
	if (a.mode == "joint")
	{
		FILE* fj = fopen((out + "/joint_test.csv").c_str(), "w");
		if (fj)
		{
			fprintf(fj, "t");
			static const char* const grp[6] = { "q_ref", "qd_ref", "qdd_ref", "q", "e", "tau" };
			for (int g = 0; g < 6; g++)
				for (int i = 0; i < n; i++) fprintf(fj, ",%s:%s", grp[g], r.jname[i].c_str());
			fprintf(fj, "\n");
			for (long k = 0; k < T.logRows; k++)
			{
				const float* row = &T.log[(size_t)k * T.logCols];
				fprintf(fj, "%.3f", row[0]);
				for (int c = 1; c < T.logCols; c++) fprintf(fj, ",%.8g", row[c]);
				fprintf(fj, "\n");
			}
			fclose(fj);
			both("joint test: %ld periods -> %s/joint_test.csv\n", T.logRows, out.c_str());
		}
		FILE* fm = fopen((out + "/joint_test_meta.txt").c_str(), "w");
		if (fm)
		{
			fprintf(fm, "T %.3f\ndwell %.3f\nstate_delay %d\ntorque_delay %d\n", T.jt.T, T.jt.dwell, STATE_DELAY,
			        TORQUE_DELAY);
			for (int i = 0; i < n; i++)
				fprintf(fm, "joint %s kp %.6g kd %.6g qr %.9g effort %.6g\n", r.jname[i].c_str(), r.kp[i], r.kd[i],
				        r.qr[i], r.effort[i]);
			for (int k = 0; k < T.jt.n; k++)
				fprintf(fm, "move %s %.6g\n", r.jname[T.jt.idx[k]].c_str(), T.jt.dq[k]);
			fclose(fm);
		}
	}
	if (call_timing::g_bOn)
	{
		both("%s", "library calls (G2T_CALL_TIMING; this run measures the calls, not the timing of the task):\n");
		call_timing::Print(stdout);
		if (fs) call_timing::Print(fs);
	}
	if (fs) fclose(fs);
	return 0;
}

} // namespace

int main(int argc, char** argv)
{
	Args a;
	for (int i = 1; i < argc; i++)
	{
		const std::string k = argv[i];
		auto val = [&](void) -> std::string { return (i + 1 < argc) ? argv[++i] : ""; };
		if (k == "--models") a.models = val();
		else if (k == "--mode") a.mode = val();
		else if (k == "--out") a.out = val();
		else if (k == "--cpu") a.cpu = atoi(val().c_str());
		else if (k == "--prio") a.prio = atoi(val().c_str());
		else if (k == "--goals") a.goals = atoi(val().c_str());
		else if (k == "--seed") a.seed = (unsigned)atoi(val().c_str());
		else if (k == "--duration") a.duration = atof(val().c_str());
		else
		{
			fprintf(stderr, "usage: %s --models DIR [--mode check|joint|hl] [--out DIR] [--cpu N] [--prio P] "
			        "[--goals N] [--seed S] [--duration S]\n", argv[0]);
			return 2;
		}
	}
	if (a.mode != "check" && a.mode != "joint" && a.mode != "hl")
	{
		fprintf(stderr, "mode %s: check, joint, or hl\n", a.mode.c_str());
		return 2;
	}
	Robot* r = new Robot();
	Sim* sim = new Sim();
	if (!LoadRobot(a, *r, *sim))
		return 2;
	printf("model: %d joints (%s .. %s), gains %g/%g (legs, waist, shoulders), %g/%g (elbows), %g/%g (wrists)\n",
	       r->nj, r->jname[0].c_str(), r->jname[r->nj - 1].c_str(), 400.0, 40.0, 200.0, 28.0, 100.0, 20.0);
	if (a.mode == "check")
		return Check(a, *r, *sim);
	return Run(a, *r, *sim);
}
