/*****************************************************************************
*	Name: goal_filter_check.cpp
*	Description: Offline check of the goal checks and the smoothing of the
*	bridge (indy7/src/GoalFilter.h; paper, Sec. V-C), without ROS 2:
*	  A  every rejection: invalid, not finite, goal below the floor, goal
*	     outside the box (each face and the radius), and a goal inside
*	  B  goal at rest with noise: the standard deviation falls by the factor
*	     sqrt(kappa / (2 - kappa)) (0.5 for kappa = 0.4)
*	  C  moving goal: the lag of the average is (1 - kappa) / kappa samples
*	     (1.5 for kappa = 0.4)
*	  D  a single outlier moves the average by kappa times its jump; the
*	     next samples pull it back
*	  E  a gap of more than dGapS restarts the average at the new sample, a
*	     shorter gap does not
*	The box, floor and hover height are test values; those of the robot are
*	the APP_GOAL_* entries of indy7/INDY7.cfg.
*
*	    bin/goal_filter_check     exit code 0 when every check passes
*	(make tools)
*****************************************************************************/
#include "GoalFilter.h"

#include <cmath>
#include <cstdio>
#include <limits>
#include <random>

namespace {

int g_nFail = 0;

void Check(bool abOk, const char* apcWhat, const char* apcFmt, double a = 0.0, double b = 0.0)
{
	char acDetail[160];
	snprintf(acDetail, sizeof(acDetail), apcFmt, a, b);
	printf("  %-4s %-58s %s\n", abOk ? "ok" : "FAIL", apcWhat, acDetail);
	if (!abOk)
		g_nFail++;
}

CGoalFilter::Params TestParams()
{
	// box, radius, goal floor, hover height; kappa = 0.4 (as in the cfg);
	// restart after 1 s
	CGoalFilter::Params p{{0.30, 0.94, -0.37, 0.63, 0.10, 0.50}, 0.92, 0.25, 0.20, 0.4, 1.0};
	return p;
}

}  // namespace

int main()
{
	const double dT = 1.0 / 30.0;              // the camera rate
	CGoalFilter f;
	f.SetParams(TestParams());
	const CGoalFilter::Params& p = f.GetParams();
	double adOut[3];

	printf("A  rejections (goal = object + %.2f m in z; floor %.2f m; box x [%.2f, %.2f] y [%.2f, %.2f] "
	       "z [%.2f, %.2f], r <= %.2f m)\n", p.dHover, p.dFloor, p.adBox[0], p.adBox[1], p.adBox[2], p.adBox[3],
	       p.adBox[4], p.adBox[5], p.dRMax);
	{
		const double adIn[3] = {0.55, 0.10, 0.10};     // goal (0.55, 0.10, 0.30): inside
		Check(f.Check(true, adIn) == CGoalFilter::eAccepted, "object inside, goal inside the box", "");
		Check(f.Check(false, adIn) == CGoalFilter::eInvalid, "invalid sample (no object or no depth)", "");
		const double nan = std::numeric_limits<double>::quiet_NaN();
		const double adNan[3] = {0.55, nan, 0.10};
		Check(f.Check(true, adNan) == CGoalFilter::eNotFinite, "a coordinate not finite", "");
		const double adInf[3] = {std::numeric_limits<double>::infinity(), 0.10, 0.10};
		Check(f.Check(true, adInf) == CGoalFilter::eNotFinite, "a coordinate infinite", "");
		const double adLow[3] = {0.55, 0.10, 0.04};    // goal z 0.24 < 0.25: depth error below the table
		Check(f.Check(true, adLow) == CGoalFilter::eBelowFloor, "goal 1 cm below the floor", "");
		const double adOnFloor[3] = {0.55, 0.10, 0.05};
		Check(f.Check(true, adOnFloor) == CGoalFilter::eAccepted, "goal on the floor", "");
		const double adFaces[6][3] = {{0.29, 0.10, 0.10}, {0.95, 0.10, 0.10}, {0.55, -0.38, 0.10},
		                              {0.55, 0.64, 0.10}, {0.55, 0.10, 0.31}, {0.80, 0.50, 0.10}};
		const char* const apcFace[6] = {"goal beyond x0", "goal beyond x1", "goal beyond y0", "goal beyond y1",
		                                "goal above z1", "goal beyond the radius (r = 0.94 m)"};
		for (int i = 0; i < 6; i++)
			Check(f.Check(true, adFaces[i]) == CGoalFilter::eOutsideBox, apcFace[i], "");
		// a rejected sample leaves the average and its time alone
		f.Reset();
		f.Push(true, adIn, 0.0, adOut);
		f.Push(true, adLow, dT, adOut);
		Check(f.Count(CGoalFilter::eAccepted) == 1 && f.Count(CGoalFilter::eBelowFloor) == 1 &&
		      f.LastAcceptT() == 0.0, "a rejected sample does not touch the average", "");
	}

	printf("B  goal at rest with noise\n");
	{
		f.SetParams(TestParams());
		std::mt19937_64 rng(1);
		std::normal_distribution<double> noise(0.0, 0.002);
		const double adC[3] = {0.55, 0.10, 0.10};
		const int nN = 200000;
		double adSum[2] = {0, 0}, adSq[2] = {0, 0};
		long nUsed = 0;
		for (int i = 0; i < nN; i++)
		{
			double adZ[3];
			for (int k = 0; k < 3; k++)
				adZ[k] = adC[k] + noise(rng);
			f.Push(true, adZ, i * dT, adOut);
			if (i < 100)                      // settle
				continue;
			const double dIn = adZ[0] - adC[0], dOut = adOut[0] - adC[0];
			adSum[0] += dIn;  adSq[0] += dIn * dIn;
			adSum[1] += dOut; adSq[1] += dOut * dOut;
			nUsed++;
		}
		const double dSdIn = std::sqrt(adSq[0] / nUsed - (adSum[0] / nUsed) * (adSum[0] / nUsed));
		const double dSdOut = std::sqrt(adSq[1] / nUsed - (adSum[1] / nUsed) * (adSum[1] / nUsed));
		const double dPred = std::sqrt(p.dKappa / (2.0 - p.dKappa));
		Check(std::fabs(dSdOut / dSdIn - dPred) < 0.01, "standard deviation out / in",
		      "%.3f (predicted %.3f)", dSdOut / dSdIn, dPred);
		Check(f.Restarts() == 1, "one restart (the first sample)", "%.0f", (double)f.Restarts());
	}

	printf("C  moving goal\n");
	{
		f.SetParams(TestParams());
		const double dV = 0.0005;             // m per sample (15 mm/s at 30 Hz)
		double dLag = 0.0;
		for (int i = 0; i < 400; i++)
		{
			const double adZ[3] = {0.40 + dV * i, 0.10, 0.10};
			f.Push(true, adZ, i * dT, adOut);
			dLag = (adZ[0] - adOut[0]) / dV;
		}
		const double dPred = (1.0 - p.dKappa) / p.dKappa;
		Check(std::fabs(dLag - dPred) < 1e-6, "lag of the average [samples]", "%.4f (predicted %.4f)", dLag, dPred);
	}

	printf("D  single outlier\n");
	{
		f.SetParams(TestParams());
		const double adC[3] = {0.55, 0.10, 0.10};
		for (int i = 0; i < 50; i++)
			f.Push(true, adC, i * dT, adOut);
		const double dJump = 0.10;
		const double adO[3] = {0.55 + dJump, 0.10, 0.10};
		f.Push(true, adO, 50 * dT, adOut);
		const double dMoved = adOut[0] - adC[0];
		Check(std::fabs(dMoved - p.dKappa * dJump) < 1e-12 && !f.Restarted(), "the outlier moves the average by kappa times its jump",
		      "%.4f m (%.4f m), no restart", dMoved, p.dKappa * dJump);
		int nBack = 0;
		for (int i = 51; i < 200; i++)
		{
			f.Push(true, adC, i * dT, adOut);
			nBack++;
			if (std::fabs(adOut[0] - adC[0]) < 1e-4)
				break;
		}
		Check(nBack <= 15, "back within 0.1 mm after", "%.0f samples", (double)nBack);
	}

	printf("E  gaps\n");
	{
		f.SetParams(TestParams());
		const double adA[3] = {0.50, 0.10, 0.10}, adB[3] = {0.70, 0.20, 0.10};
		for (int i = 0; i < 10; i++)
			f.Push(true, adA, i * dT, adOut);
		double t = 9 * dT;
		t += 0.5;                              // shorter than dGapS: the average goes on
		f.Push(true, adB, t, adOut);
		Check(!f.Restarted() && std::fabs(adOut[0] - (0.50 + p.dKappa * 0.20)) < 1e-12, "gap of 0.5 s: no restart",
		      "x %.4f m", adOut[0]);
		t += 1.5;                              // longer than dGapS: restart at the new sample
		const double adC[3] = {0.40, -0.10, 0.12};
		f.Push(true, adC, t, adOut);
		Check(f.Restarted() && adOut[0] == adC[0] && adOut[1] == adC[1] && adOut[2] == adC[2],
		      "gap of 1.5 s: restart at the new sample", "x %.4f m", adOut[0]);
		// rejected samples do not bridge a gap: only accepted samples count
		const double adLow[3] = {0.55, 0.10, 0.00};
		for (int i = 1; i <= 60; i++)
			f.Push(true, adLow, t + i * dT, adOut);           // 2 s of samples below the floor
		f.Push(true, adA, t + 61 * dT, adOut);
		Check(f.Restarted(), "2 s of rejected samples count as a gap", "");
	}

	printf("%s: %d check(s) failed\n", g_nFail ? "FAIL" : "PASS", g_nFail);
	return g_nFail ? 1 : 0;
}
