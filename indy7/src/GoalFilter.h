/*****************************************************************************
*	Name: GoalFilter.h
*	Description: The goal checks and the smoothing of the bridge (paper,
*	Sec. V-C), without ROS 2, so that the same code runs in the bridge
*	(ROS2PickBridge.cpp) and in its offline check (tools/goal_filter_check.cpp).
*
*	A sample is the position of the object in the base frame [m] with the
*	validity flag of the pipeline (an object of the selected class and a
*	valid depth at its center). The goal of a sample is the point at the
*	hover height above it; the high-level controller adds that height to
*	every position that it reads from the goal slot
*	(CTrajectoryRT::SetGoalOffset), so the slot carries the object position
*	and the checks here use the goal.
*	A sample is rejected when it is invalid, not finite, its goal below the
*	goal floor, or its goal outside the workspace box (x, y, z ranges and
*	the distance from the vertical axis of the base). Accepted samples are
*	smoothed by an exponential moving average,
*	    p_k = p_{k-1} + kappa (z_k - p_{k-1}),
*	which restarts from the new sample after a gap of more than dGapS
*	between accepted samples. A jump is neither rejected nor a restart.
*	No allocation, no clock: the caller passes the time of each sample.
*****************************************************************************/
#ifndef G2T_GOAL_FILTER_H
#define G2T_GOAL_FILTER_H

#include <cmath>

class CGoalFilter
{
public:
	struct Params
	{
		double adBox[6];   // goal box x0 x1 y0 y1 z0 z1 [m], base frame
		double dRMax;      // largest distance of the goal from the vertical axis of the base [m]
		double dFloor;     // goal floor: smallest z of the goal [m]
		double dHover;     // hover height: goal = object + (0, 0, dHover) [m]
		double dKappa;     // weight of the newest sample (0 < kappa <= 1)
		double dGapS;      // restart after this gap between accepted samples [s]
	};
	enum eVerdict { eAccepted = 0, eInvalid, eNotFinite, eBelowFloor, eOutsideBox, eVerdicts };

	CGoalFilter() { Reset(); }

	void SetParams(const Params& astP)
	{
		m_stP = astP;
		if (!(m_stP.dKappa > 0.0) || m_stP.dKappa > 1.0)
			m_stP.dKappa = 1.0;   // filter off rather than a filter that never moves
		Reset();
	}
	const Params& GetParams() const { return m_stP; }

	void Reset()
	{
		m_bSeeded = false;
		m_bRestarted = false;
		m_dLastT = 0.0;
		for (int k = 0; k < 3; k++)
			m_adAvg[k] = 0.0;
		for (int v = 0; v < eVerdicts; v++)
			m_anCount[v] = 0;
		m_nRestarts = 0;
	}

	/* one sample at time adT [s] (any monotonic clock); on acceptance the
	 * smoothed object position (without the hover height) is in adOut */
	eVerdict Push(bool abValid, const double* adP, double adT, double* adOut)
	{
		const eVerdict eV = Check(abValid, adP);
		m_anCount[eV]++;
		if (eV != eAccepted)
			return eV;
		m_bRestarted = !m_bSeeded || (adT - m_dLastT) > m_stP.dGapS;
		for (int k = 0; k < 3; k++)
			m_adAvg[k] = m_bRestarted ? adP[k] : m_adAvg[k] + m_stP.dKappa * (adP[k] - m_adAvg[k]);
		if (m_bRestarted)
			m_nRestarts++;
		m_bSeeded = true;
		m_dLastT = adT;
		for (int k = 0; k < 3; k++)
			adOut[k] = m_adAvg[k];
		return eAccepted;
	}

	eVerdict Check(bool abValid, const double* adP) const
	{
		if (!abValid)
			return eInvalid;
		if (!std::isfinite(adP[0]) || !std::isfinite(adP[1]) || !std::isfinite(adP[2]))
			return eNotFinite;
		const double dGx = adP[0], dGy = adP[1], dGz = adP[2] + m_stP.dHover;
		if (dGz < m_stP.dFloor)
			return eBelowFloor;
		const double* b = m_stP.adBox;
		if (dGx < b[0] || dGx > b[1] || dGy < b[2] || dGy > b[3] || dGz < b[4] || dGz > b[5] ||
		    dGx * dGx + dGy * dGy > m_stP.dRMax * m_stP.dRMax)
			return eOutsideBox;
		return eAccepted;
	}

	bool   Restarted() const { return m_bRestarted; }     // the last accepted sample restarted the average
	double LastAcceptT() const { return m_dLastT; }
	bool   HasAccepted() const { return m_bSeeded; }
	long   Count(eVerdict aeV) const { return m_anCount[aeV]; }
	long   Restarts() const { return m_nRestarts; }
	static const char* Name(eVerdict aeV)
	{
		static const char* const s_apc[eVerdicts] = { "accepted", "invalid", "not finite", "below the goal floor",
		                                              "outside the workspace box" };
		return (aeV >= 0 && aeV < eVerdicts) ? s_apc[aeV] : "?";
	}

private:
	Params m_stP{{0.30, 0.94, -0.37, 0.63, 0.10, 0.50}, 0.92, 0.25, 0.20, 0.4, 1.0};
	bool   m_bSeeded, m_bRestarted;
	double m_dLastT;
	double m_adAvg[3];
	long   m_anCount[eVerdicts];
	long   m_nRestarts;
};

#endif // G2T_GOAL_FILTER_H
