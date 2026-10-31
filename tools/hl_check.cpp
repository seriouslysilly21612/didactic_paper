/*****************************************************************************
*	Name: hl_check.cpp
*	Description: Offline check of the high-level controller (CHighLevelRT,
*	indy7/src/HighLevelRT.h; paper, Sec. IV-C, eq. (9) and Table 9) before it
*	drives the simulation or the robot.
*
*	The arm is ideal: it follows the reference exactly, and the controller
*	receives the state two periods old, as on the robot. Scenarios:
*	  A  HOME from another posture
*	  B  fixed goal 0.3 m away (the fixed-target task-space test)
*	  C  goal moved 0.15 m sideways when the tool is 0.15 m from it
*	  D  goal with noise (1 mm, every 67 periods), with delta and with delta = 0
*	  E  goal out of reach (rejected), then a goal within reach
*	  F  J2 blocked: the lead bound resets its reference
*	In every period it checks the reference contract: q_d is the integral of
*	qd_d (C2), the limits on velocity, acceleration and jerk except where the
*	lead bound reset a joint, and the joint limits (C3), the lead (C4); and
*	the final position of the tool, the time to reach the goal against the
*	planned duration, the turn toward a moved goal without a stop (C1), and,
*	in the malloc-count build, that Update() never allocates (C5).
*
*	    bin/hl_check models/indy7.urdf     exit code 0 when every check passes
*	(make tools; make MALLOC_COUNT=1 tools to count the allocations)
*****************************************************************************/
#include "HighLevelRT.h"
#include "base/MallocCount.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <random>
#include <vector>

namespace {

const int N = CHighLevelRT::DOF;
const double TS = 1e-3;
int g_nFail = 0;

void Check(bool abOk, const char* astrWhat, const char* astrDetail)
{
	printf("  %-4s %-58s %s\n", abOk ? "ok" : "FAIL", astrWhat, astrDetail);
	if (!abOk) g_nFail++;
}

/* closed loop with an ideal arm: measurement = reference two periods old */
struct Loop
{
	CHighLevelRT& hl;
	std::deque<std::vector<double>> qHist, qdHist;
	double q[N], qd[N], qr[N], qdr[N], qddr[N], qrPrev[N], qdrPrev[N], qddrPrev[N];
	bool bFirst;
	/* contract */
	double dMaxVel, dMaxAcc, dMaxJerk, dMaxInteg, dMaxLead, dMinMargin;
	long nCycles, nAllocs, nResets;
	double dMaxUs;
	unsigned uBlocked;		// joints whose measurement is frozen
	double adBlockedQ[N];

	explicit Loop(CHighLevelRT& aHl, const double* adQ0) : hl(aHl), bFirst(true), dMaxVel(0), dMaxAcc(0), dMaxJerk(0),
		dMaxInteg(0), dMaxLead(0), dMinMargin(1e9), nCycles(0), nAllocs(0), nResets(0), dMaxUs(0), uBlocked(0)
	{
		for (int i = 0; i < N; i++) { q[i] = adQ0[i]; qd[i] = 0; qr[i] = adQ0[i]; qdr[i] = 0; qddr[i] = 0; adBlockedQ[i] = adQ0[i]; }
		for (int k = 0; k < 2; k++) { qHist.push_back(std::vector<double>(q, q + N)); qdHist.push_back(std::vector<double>(N, 0.0)); }
	}

	bool Step(const double* adGoal)
	{
		/* the measurement of this period: the reference of two periods ago */
		for (int i = 0; i < N; i++) { q[i] = qHist.front()[i]; qd[i] = qdHist.front()[i]; }
		for (int i = 0; i < N; i++) if (uBlocked & (1u << i)) { q[i] = adBlockedQ[i]; qd[i] = 0.0; }
		memcpy(qrPrev, qr, sizeof(qr)); memcpy(qdrPrev, qdr, sizeof(qdr)); memcpy(qddrPrev, qddr, sizeof(qddr));
		const unsigned long uA0 = malloc_count::Count();
		malloc_count::Arm(true);
		const BOOL bOk = hl.Update(adGoal, q, qd, qr, qdr, qddr);
		malloc_count::Arm(false);
		if (nCycles > 0) nAllocs += (long)(malloc_count::Count() - uA0);
		const CHighLevelRT::Status& st = hl.GetStatus();
		if (st.dUs > dMaxUs && nCycles > 0) dMaxUs = st.dUs;
		const CHighLevelRT::Params& p = hl.GetParams();
		for (int i = 0; i < N; i++)
		{
			dMaxVel = std::max(dMaxVel, std::fabs(qdr[i]));
			dMaxAcc = std::max(dMaxAcc, std::fabs(qddr[i]));
			dMinMargin = std::min(dMinMargin, std::min(qr[i] - p.adLimLo[i], p.adLimHi[i] - qr[i]));
			dMaxLead = std::max(dMaxLead, std::fabs(qr[i] - q[i]));
			if (!bFirst && !(prevReset & (1u << i)))
			{
				dMaxJerk = std::max(dMaxJerk, std::fabs(qddr[i] - qddrPrev[i]) / TS);
				/* q_d is the integral of qd_d (trapezoid; Ruckig's jerk is piecewise constant) */
				dMaxInteg = std::max(dMaxInteg, std::fabs(qr[i] - qrPrev[i] - 0.5 * TS * (qdr[i] + qdrPrev[i])));
			}
		}
		prevReset = st.uLeadReset;
		if (st.uLeadReset) nResets++;
		bFirst = false;
		qHist.pop_front(); qdHist.pop_front();
		qHist.push_back(std::vector<double>(qr, qr + N)); qdHist.push_back(std::vector<double>(qdr, qdr + N));
		nCycles++;
		return bOk;
	}
	unsigned prevReset = 0;
};

double Norm3(const double* a, const double* b)
{
	return std::sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2]));
}

void ReportContract(const Loop& l, const CHighLevelRT::Params& p)
{
	char ac[160];
	snprintf(ac, sizeof(ac), "max |qd_d| %.4f rad/s (limit %.2f)", l.dMaxVel, p.dVelMax);
	Check(l.dMaxVel <= p.dVelMax + 1e-9, "C3 velocity limit", ac);
	snprintf(ac, sizeof(ac), "max |qdd_d| %.4f rad/s^2 (limit %.2f)", l.dMaxAcc, p.dAccMax);
	Check(l.dMaxAcc <= p.dAccMax + 1e-9, "C3 acceleration limit", ac);
	snprintf(ac, sizeof(ac), "max |dqdd_d|/T_s %.4f rad/s^3 (limit %.1f)", l.dMaxJerk, p.dJerkMax);
	Check(l.dMaxJerk <= p.dJerkMax * (1 + 1e-6) + 1e-6, "C3 jerk limit (outside lead-bound resets)", ac);
	snprintf(ac, sizeof(ac), "max |q_d(k)-q_d(k-1)-T(qd_d(k)+qd_d(k-1))/2| %.2e rad", l.dMaxInteg);
	Check(l.dMaxInteg < 1e-7, "C2 q_d is the integral of qd_d", ac);
	snprintf(ac, sizeof(ac), "closest to a joint limit %.4f rad", l.dMinMargin);
	Check(l.dMinMargin > 0.0, "C3 joint limits", ac);
	snprintf(ac, sizeof(ac), "Update(): %ld allocations in %ld periods%s; max %.1f us", l.nAllocs, l.nCycles,
	         malloc_count::Enabled() ? "" : " (not counted: build with make MALLOC_COUNT=1)", l.dMaxUs);
	Check(l.nAllocs == 0, "C5 no allocation in Update()", ac);
}

} // namespace

int main(int argc, char** argv)
{
	if (argc < 2) { fprintf(stderr, "usage: %s <urdf>   (models/indy7.urdf)\n", argv[0]); return 2; }
	CHighLevelRT::Params p = CHighLevelRT::DefaultParams();
	char ac[200];

	/* ---------------------------------------------------------------- A */
	{
		printf("A  HOME from another posture\n");
		CHighLevelRT hl;
		if (!hl.Init(argv[1], TS, p)) return 2;
		const double q0[N] = { 0.4, -0.3, -1.2, 0.3, -1.2, 0.5 };
		Loop l(hl, q0);
		hl.StartHome(q0, l.qd);
		int nRest = -1;
		for (int k = 0; k < 20000 && nRest < 0; k++) { l.Step(NULL); if (hl.GetStatus().bAtRest) nRest = k; }
		double dMax = 0;
		for (int i = 0; i < N; i++) dMax = std::max(dMax, std::fabs(l.q[i] - p.adQr[i]));
		snprintf(ac, sizeof(ac), "at rest after %.3f s, |q - q_r| %.1e rad", nRest * TS, dMax);
		Check(nRest > 0 && dMax < 1e-9, "reaches q_r and comes to rest", ac);
		ReportContract(l, p);
	}

	/* ---------------------------------------------------------------- B, C */
	for (int nCase = 0; nCase < 2; nCase++)
	{
		printf(nCase == 0 ? "B  fixed goal 0.3 m away\n" : "C  goal moved 0.15 m sideways when the tool is 0.15 m from it\n");
		CHighLevelRT hl;
		if (!hl.Init(argv[1], TS, p)) return 2;
		Loop l(hl, p.adQr);
		hl.StartTrack(p.adQr, l.qd);
		double p0[3], goal[3], tool[3];
		hl.ToolPosition(p.adQr, p0);
		const double d1[3] = { 0.2121320343559643, 0.0, -0.2121320343559643 };
		for (int k = 0; k < 3; k++) goal[k] = p0[k] + d1[k];
		bool bJumped = false;
		int nAccept = -1, nConv = -1, nRest = -1, nJump = -1;
		double dPlanned = 0, dMinSpeedAfterJump = 1e9;
		for (int k = 0; k < 20000; k++)
		{
			l.Step(goal);
			const CHighLevelRT::Status& st = hl.GetStatus();
			if (st.bGoalAccepted && nAccept < 0) nAccept = k;		// the first goal
			if (st.bIkConverged && nConv < 0) nConv = k;
			if (st.bOtgNewCalc) dPlanned = st.dOtgDuration;
			hl.ToolPosition(l.q, tool);
			if (nCase == 1 && !bJumped && Norm3(tool, goal) <= 0.15)
			{
				goal[1] += 0.15;
				bJumped = true;
				nJump = k;
			}
			if (nCase == 1 && bJumped && k <= nJump + 500)		// 0.5 s after the jump
			{
				double dS = 0;
				for (int i = 0; i < N; i++) dS += l.qdr[i] * l.qdr[i];
				dMinSpeedAfterJump = std::min(dMinSpeedAfterJump, std::sqrt(dS));
			}
			if (st.bAtRest && (nCase == 0 || bJumped)) { nRest = k; break; }
		}
		const CHighLevelRT::Status& st = hl.GetStatus();
		hl.ToolPosition(l.q, tool);
		snprintf(ac, sizeof(ac), "first goal accepted in period %d, IK converged in period %d; last goal: %d iterations",
		         nAccept, nConv, st.nIkIter);
		Check(nAccept == 0 && nConv == 0 && st.nIkIter < p.nIterCycle, "IK converges within the period of the goal", ac);
		snprintf(ac, sizeof(ac), "final distance of the tool %.4f mm (tolerance %.2f mm)", Norm3(tool, goal) * 1e3, p.dTol * 1e3);
		Check(Norm3(tool, goal) < p.dTol, "tool at the goal", ac);
		if (nCase == 0)
		{
			snprintf(ac, sizeof(ac), "at rest after %.3f s; planned duration %.3f s + 0.2 s rest window", nRest * TS, dPlanned);
			Check(nRest > 0 && std::fabs(nRest * TS - (dPlanned + 0.2 + 2 * TS)) < 0.01, "time to reach = planned duration", ac);
		}
		else
		{
			snprintf(ac, sizeof(ac), "jump at %.3f s; lowest joint speed in the next 0.5 s %.3f rad/s", nJump * TS, dMinSpeedAfterJump);
			Check(nJump > 0 && dMinSpeedAfterJump > 0.05, "C1 turns toward the new goal without stopping", ac);
		}
		snprintf(ac, sizeof(ac), "%ld accepted, %ld converged, %ld rejected, %ld lead resets", st.nAccepted, st.nConverged, st.nRejected, st.nLeadResets);
		Check(st.nRejected == 0 && st.nLeadResets == 0, "counters", ac);
		ReportContract(l, p);
	}

	/* ---------------------------------------------------------------- D */
	for (int nCase = 0; nCase < 2; nCase++)
	{
		const double dDelta = nCase == 0 ? 0.006 : 0.0;	// delta of Table 9: 6 sigma of the noise
		printf("D  goal with noise of 1 mm every 67 periods, delta = %.0f mm\n", dDelta * 1e3);
		CHighLevelRT::Params pd = p;
		pd.dDelta = dDelta;
		CHighLevelRT hl;
		if (!hl.Init(argv[1], TS, pd)) return 2;
		Loop l(hl, pd.adQr);
		hl.StartTrack(pd.adQr, l.qd);
		double p0[3], goal[3], g0[3], tool[3];
		hl.ToolPosition(pd.adQr, p0);
		const double d1[3] = { 0.2121320343559643, 0.0, -0.2121320343559643 };
		for (int k = 0; k < 3; k++) g0[k] = goal[k] = p0[k] + d1[k];
		std::mt19937 rng(1);
		std::normal_distribution<double> nd(0.0, 0.001);
		const int nT = 10000;
		double adMin[3] = { 1e9, 1e9, 1e9 }, adMax[3] = { -1e9, -1e9, -1e9 };
		for (int k = 0; k < nT; k++)
		{
			if (k % 67 == 0)
				for (int j = 0; j < 3; j++) goal[j] = g0[j] + nd(rng);
			l.Step(goal);
			if (k >= nT - 1000)
			{
				hl.ToolPosition(l.q, tool);
				for (int j = 0; j < 3; j++) { adMin[j] = std::min(adMin[j], tool[j]); adMax[j] = std::max(adMax[j], tool[j]); }
			}
		}
		double dPP = 0;
		for (int j = 0; j < 3; j++) dPP = std::max(dPP, adMax[j] - adMin[j]);
		const CHighLevelRT::Status& st = hl.GetStatus();
		snprintf(ac, sizeof(ac), "%ld goals accepted of %d samples; residual motion (last 1 s) %.3f mm", st.nAccepted, nT / 67 + 1, dPP * 1e3);
		if (nCase == 0)
			Check(dPP < 1e-6, "with delta: the tool comes to rest", ac);
		else
			Check(dPP > 1e-4, "without delta: the tool keeps moving", ac);
		ReportContract(l, pd);
	}

	/* ---------------------------------------------------------------- E */
	{
		printf("E  goal out of reach, then a goal within reach\n");
		CHighLevelRT hl;
		if (!hl.Init(argv[1], TS, p)) return 2;
		Loop l(hl, p.adQr);
		hl.StartTrack(p.adQr, l.qd);
		double p0[3], far[3], near_[3];
		hl.ToolPosition(p.adQr, p0);
		for (int k = 0; k < 3; k++) { far[k] = p0[k]; near_[k] = p0[k]; }
		far[0] += 2.0;
		near_[2] -= 0.1;
		int nRej = -1;
		for (int k = 0; k < 100 && nRej < 0; k++) { l.Step(far); if (hl.GetStatus().bIkRejected) nRej = k; }
		double dMaxQ = 0;
		for (int i = 0; i < N; i++) dMaxQ = std::max(dMaxQ, std::fabs(hl.GetStatus().adTarget[i] - p.adQr[i]));
		snprintf(ac, sizeof(ac), "rejected in period %d (%d iterations), target moved %.1e rad", nRej, p.nIterMax, dMaxQ);
		Check(nRej == p.nIterMax / p.nIterCycle && dMaxQ == 0.0, "rejected after N_max iterations, previous target kept", ac);
		int nRest = -1;
		for (int k = 0; k < 20000 && nRest < 0; k++) { l.Step(near_); if (hl.GetStatus().bAtRest) nRest = k; }
		double tool[3];
		hl.ToolPosition(l.q, tool);
		snprintf(ac, sizeof(ac), "then at rest after %.3f s, %.4f mm from the goal", nRest * TS, Norm3(tool, near_) * 1e3);
		Check(nRest > 0 && Norm3(tool, near_) < p.dTol, "a goal within reach is accepted afterwards", ac);
		ReportContract(l, p);
	}

	/* ---------------------------------------------------------------- F */
	{
		printf("F  J2 blocked while the goal needs it\n");
		CHighLevelRT hl;
		if (!hl.Init(argv[1], TS, p)) return 2;
		Loop l(hl, p.adQr);
		hl.StartTrack(p.adQr, l.qd);
		double p0[3], goal[3];
		hl.ToolPosition(p.adQr, p0);
		const double d1[3] = { 0.2121320343559643, 0.0, -0.2121320343559643 };
		for (int k = 0; k < 3; k++) goal[k] = p0[k] + d1[k];
		l.uBlocked = 1u << 1;
		l.adBlockedQ[1] = p.adQr[1];
		double dMaxLeadJ2 = 0;
		for (int k = 0; k < 3000; k++)
		{
			l.Step(goal);
			dMaxLeadJ2 = std::max(dMaxLeadJ2, std::fabs(l.qr[1] - l.q[1]));
		}
		snprintf(ac, sizeof(ac), "%ld resets of J2 in 3 s; largest lead of J2 %.4f rad (ell %.2f)", hl.GetStatus().nLeadResets, dMaxLeadJ2, p.dLead);
		Check(hl.GetStatus().nLeadResets > 0 && dMaxLeadJ2 < p.dLead + 0.01, "C4 the lead bound holds the reference of the blocked joint back", ac);
	}

	printf("\n%s: %d check(s) failed\n", g_nFail ? "FAIL" : "PASS", g_nFail);
	return g_nFail ? 1 : 0;
}
