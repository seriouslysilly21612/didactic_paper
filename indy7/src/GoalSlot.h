/*****************************************************************************
*	Name: GoalSlot.h
*	Description: The goal slot (paper, Sec. II and Fig. 1): the latest goal
*	of the tool [m], written by one source (the camera application through
*	its bridge, or a test program of the Trajectory task) and read by the
*	high-level controller in every period. An atomic slot (AtomicSlot.h):
*	the reader also learns how many goals have been written, so that it can
*	tell a new goal from the one it has seen and ignore goals written before
*	a motion started.
*****************************************************************************/
#ifndef G2T_GOAL_SLOT_H
#define G2T_GOAL_SLOT_H

#include "base/AtomicSlot.h"

class CGoalSlot
{
public:
	/* one writer */
	void Write(const double* adP)
	{
		Goal stG;
		for (int k = 0; k < 3; k++)
			stG.adP[k] = adP[k];
		m_cSlot.Write(stG);
	}

	/* the latest goal and the number of goals written so far; false if none
	 * has been written (or every attempt overlapped a write: adP unchanged) */
	bool Read(double* adP, unsigned int* auCount) const
	{
		Goal stG;
		uint32_t uCount = 0;
		if (!m_cSlot.Read(stG, &uCount))
			return false;
		for (int k = 0; k < 3; k++)
			adP[k] = stG.adP[k];
		*auCount = uCount;
		return true;
	}

	unsigned int Count() const { return m_cSlot.Count(); }

private:
	struct Goal { double adP[3]; };
	CAtomicSlot<Goal> m_cSlot;
};

#endif // G2T_GOAL_SLOT_H
