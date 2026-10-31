/*****************************************************************************
*	Name: CallTiming.h
*	Description: The time of the library calls of the control law and of the
*	high-level controller (RBDL, Ruckig), a measurement tool. With
*	G2T_CALL_TIMING=1 in the environment of the controller, the tasks read
*	the clock around each call below and accumulate, per call, the count,
*	the sum, and the largest time: no allocation, no lock; each call is
*	timed by one task only. The table is printed when the tasks end and,
*	with -t DIR, written to DIR/calls_<stamp>.txt next to the timing log.
*
*	Every clock read costs some tens of nanoseconds and lengthens the cycle,
*	so a run with call timing measures the calls, not the timing of the
*	tasks. Without the variable, each site costs the test of one flag.
*
*	A histogram per call, 10 ns bins up to 10 us and 1 us bins up to 1 ms
*	(one more bin counts the rest; the largest time is kept exactly),
*	static, so the percentiles come from every call: the table gets the
*	50th, 99th, 99.9th, 99.99th and 99.999th percentiles, and
*	calls_<stamp>.hist the non-empty bins.
*****************************************************************************/
#ifndef G2T_CALL_TIMING_H
#define G2T_CALL_TIMING_H

#include <cstdint>
#include <cstdio>
#include <cstdlib>

#include "base/Common.h"

namespace call_timing
{
	/* the rows of the table, in the order of a cycle */
	enum eSlot
	{
		eIkKin = 0,		// IK, per iteration: UpdateKinematics
		eIkFk,			// IK, per iteration: CalcBodyToBaseCoordinates, p(q_k)
		eIkJac,			// IK, per iteration: CalcPointJacobian, J_p(q_k)
		eIkSolve,		// IK, per iteration: J J^T + lambda^2 I, LDLT (3x3), the step dq_k
		eIkIter,		// IK: one iteration, all of the above
		eOtg,			// per cycle: Ruckig<N>::update (N = 6: the arm; 7: a hand of the humanoid)
		eCtcKin,		// per cycle: UpdateKinematics of the control law
		eCtcCrba,		// per cycle: CompositeRigidBodyAlgorithm, M(q)
		eCtcNle,		// per cycle: NonlinearEffects, h(q, qd)
		eControl,		// per cycle: the whole control law (Update), high-level controller included
		eN
	};

	struct Stat
	{
		uint64_t uN;		// calls
		double   dSum;		// [ns]
		int64_t  nMax;		// [ns]
	};

	/* histogram: bins 0..999 of 10 ns (0 to 10 us), 1000..1989 of 1 us (10 us
	 * to 1 ms), bin 1990 for the rest */
	enum { HIST_FINE = 1000, HIST_COARSE = 990, HIST_BINS = HIST_FINE + HIST_COARSE + 1 };

	inline bool g_bOn = false;
	inline Stat g_aStat[eN] = {};
	inline uint32_t g_aHist[eN][HIST_BINS] = {};

	inline int Bin(int64_t n)
	{
		if (n < 0)
			return 0;
		if (n < 10000)
			return (int)(n / 10);
		if (n < 1000000)
			return HIST_FINE + (int)((n - 10000) / 1000);
		return HIST_BINS - 1;
	}

	/* lower edge of bin b [ns]; the last bin has no upper edge */
	inline int64_t BinLow(int b)
	{
		return b < HIST_FINE ? 10LL * b : 10000LL + 1000LL * (b - HIST_FINE);
	}

	inline int64_t BinHigh(int b)
	{
		return b < HIST_FINE ? 10LL * (b + 1) : (b < HIST_BINS - 1 ? 10000LL + 1000LL * (b - HIST_FINE + 1) : -1);
	}

	inline const char* Name(int e)
	{
		static const char* const s_apName[eN] = {
			"IK: UpdateKinematics", "IK: CalcBodyToBaseCoordinates", "IK: CalcPointJacobian",
			"IK: LDLT step (3x3)", "IK: iteration (all of the above)", "Ruckig<N>::update",
			"CTC: UpdateKinematics", "CTC: CompositeRigidBodyAlgorithm", "CTC: NonlinearEffects",
			"control law (Update), all" };
		return (e >= 0 && e < eN) ? s_apName[e] : "?";
	}

	/* before the control task: from the environment */
	inline void Init()
	{
		const char* p = getenv("G2T_CALL_TIMING");
		g_bOn = (p != NULL && atoi(p) != 0);
	}

	inline RTTIME Now()
	{
		return g_bOn ? read_timer() : 0;
	}

	inline void Add(eSlot e, int64_t n)
	{
		Stat& s = g_aStat[e];
		s.uN++;
		s.dSum += (double)n;
		if (n > s.nMax)
			s.nMax = n;
		g_aHist[e][Bin(n)]++;
	}

	/* the p-quantile of slot e [ns] from the histogram: the upper edge of the
	 * bin in which it lies (an upper bound, at most one bin above the true
	 * value), the exact largest time in the last bin; -1 without calls */
	inline double Quantile(int e, double p)
	{
		const Stat& s = g_aStat[e];
		if (s.uN == 0)
			return -1.0;
		const double dRank = p * (double)s.uN;
		uint64_t uCum = 0;
		for (int b = 0; b < HIST_BINS; b++)
		{
			uCum += g_aHist[e][b];
			if ((double)uCum >= dRank && uCum > 0)
			{
				const int64_t nHi = BinHigh(b);
				return (double)((nHi < 0 || nHi > s.nMax) ? s.nMax : nHi);
			}
		}
		return (double)s.nMax;
	}

	/* the time since t0 to the row e; returns the time now (0 when off) so
	 * that consecutive calls share the clock reads */
	inline RTTIME Lap(eSlot e, RTTIME t0)
	{
		if (!g_bOn)
			return 0;
		const RTTIME t = read_timer();
		Add(e, (int64_t)(t - t0));
		return t;
	}

	inline void Print(FILE* fp)
	{
		static const double s_adP[5] = {0.5, 0.99, 0.999, 0.9999, 0.99999};
		fprintf(fp, "%-36s %12s %10s %10s %9s %9s %9s %9s %9s\n", "call", "calls", "mean [us]", "max [us]",
				"p50", "p99", "p99.9", "p99.99", "p99.999");
		for (int e = 0; e < eN; e++)
		{
			const Stat& s = g_aStat[e];
			if (s.uN == 0)
			{
				fprintf(fp, "%-36s %12d %10s %10s\n", Name(e), 0, "-", "-");
				continue;
			}
			fprintf(fp, "%-36s %12llu %10.3f %10.3f", Name(e), (unsigned long long)s.uN,
					s.dSum / (double)s.uN * 1e-3, (double)s.nMax * 1e-3);
			for (int k = 0; k < 5; k++)
				fprintf(fp, " %9.3f", Quantile(e, s_adP[k]) * 1e-3);
			fprintf(fp, "\n");
		}
		fprintf(fp, "(percentiles in us: upper edge of the histogram bin, 10 ns up to 10 us, 1 us above)\n");
	}

	/* the non-empty bins: slot, call, lower and upper edge [ns] (-1: open), count */
	inline void PrintHist(FILE* fp)
	{
		fprintf(fp, "# g2t-call-hist 1: slot lo_ns hi_ns count  (hi_ns -1: the last bin, open)\n");
		for (int e = 0; e < eN; e++)
			fprintf(fp, "# slot %d: %s, %llu calls, max %lld ns\n", e, Name(e),
					(unsigned long long)g_aStat[e].uN, (long long)g_aStat[e].nMax);
		for (int e = 0; e < eN; e++)
			for (int b = 0; b < HIST_BINS; b++)
				if (g_aHist[e][b])
					fprintf(fp, "%d %lld %lld %u\n", e, (long long)BinLow(b), (long long)BinHigh(b), g_aHist[e][b]);
	}
}

#endif // G2T_CALL_TIMING_H
