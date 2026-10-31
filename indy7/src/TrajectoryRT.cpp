/*****************************************************************************
*	Name: TrajectoryRT.cpp
*	Description: TRACK and HOME as the Trajectory task runs them. See
*	TrajectoryRT.h.
*****************************************************************************/
#include "TrajectoryRT.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>

CTrajectoryRT::CTrajectoryRT()
    : m_pGoalSlot(NULL)
    , m_eState(eIdle)
    , m_ePending(eIdle)
    , m_eMotion(eMotionNone)
    , m_bFrictionFF(FALSE)
    , m_uGoalCountStart(0)
    , m_uGoalCountSeen(0)
    , m_nGoalAge(0)
    , m_nCycle(0)
    , m_dTs(0.001)
    , m_bSample(FALSE)
{
    for (int k = 0; k < 3; k++)
        m_adGoalOffset[k] = 0.0;
    memset(&m_sample, 0, sizeof(m_sample));
}

BOOL
CTrajectoryRT::Init(const char* astrURDF, double adTs, const CHighLevelRT::Params& astParams)
{
    m_dTs = adTs;
    return m_hl.Init(astrURDF, adTs, astParams);
}

BOOL
CTrajectoryRT::Start(bool abHome, const double* adQ, const double* adQd, BOOL abFrictionFF,
                     char* astrWhy, size_t auWhyLen, const double* adHomeTarget)
{
    if (!m_hl.IsInit()) {
        snprintf(astrWhy, auWhyLen, "high-level controller not initialized (see the log at start-up)");
        return FALSE;
    }
    if (m_eState == eRunning) {
        snprintf(astrWhy, auWhyLen, "%s is running ('x' first)", m_eMotion == eMotionHome ? "HOME" : "TRACK");
        return FALSE;
    }
    double dQdSq = 0.0;
    for (int i = 0; i < MAX_DOF; ++i)
        dQdSq += adQd[i] * adQd[i];
    if (dQdSq > REST_QD * REST_QD) {
        snprintf(astrWhy, auWhyLen, "the arm is moving (|qd| %.3f rad/s > %.2f)", std::sqrt(dQdSq), REST_QD);
        return FALSE;
    }
    // a measured angle outside the range of its joint (wrong zero offset,
    // whole turns) would make the generator turn the joint toward a target
    // inside it
    const CHighLevelRT::Params& stP = m_hl.GetParams();
    for (int i = 0; i < MAX_DOF; ++i) {
        if (adQ[i] < stP.adLimLo[i] || adQ[i] > stP.adLimHi[i]) {
            snprintf(astrWhy, auWhyLen, "J%d at %.3f rad, outside its range [%.3f, %.3f]", i + 1, adQ[i],
                     stP.adLimLo[i], stP.adLimHi[i]);
            return FALSE;
        }
    }
    // the target of a joint move: inside the limits minus the margin, as
    // every target of the IK
    if (abHome && adHomeTarget != NULL) {
        for (int i = 0; i < MAX_DOF; ++i) {
            const double dLo = stP.adLimLo[i] + stP.dMargin, dHi = stP.adLimHi[i] - stP.dMargin;
            if (!std::isfinite(adHomeTarget[i]) || adHomeTarget[i] < dLo || adHomeTarget[i] > dHi) {
                snprintf(astrWhy, auWhyLen, "the target of J%d, %.3f rad, is outside [%.3f, %.3f]", i + 1,
                         adHomeTarget[i], dLo, dHi);
                return FALSE;
            }
        }
    }
    if (abHome && adHomeTarget != NULL)
        m_hl.StartHomeTo(adQ, adQd, adHomeTarget);
    else if (abHome)
        m_hl.StartHome(adQ, adQd);
    else
        m_hl.StartTrack(adQ, adQd);
    m_uGoalCountStart = (m_pGoalSlot != NULL) ? m_pGoalSlot->Count() : 0;
    m_uGoalCountSeen  = m_uGoalCountStart;
    m_nGoalAge   = 0;
    m_nCycle     = 0;
    m_ePending   = eRunning;
    m_bFrictionFF = abFrictionFF;
    m_eMotion    = abHome ? eMotionHome : eMotionTrack;
    m_eState     = eRunning;
    m_bSample    = FALSE;
    return TRUE;
}

void
CTrajectoryRT::End(eState aeState)
{
    if (m_eState != eRunning)
        return;
    m_eState   = aeState;
    m_ePending = aeState;
    m_eMotion  = eMotionNone;
}

BOOL
CTrajectoryRT::Update(const double* adQ, const double* adQd,
                      double* adQRef, double* adQdRef, double* adQddRef)
{
    m_bSample = FALSE;
    if (m_eState != eRunning)
        return FALSE;

    const int nN = MAX_DOF;
    double adGoal[3];
    // the goal slot: only goals written after the start of TRACK count
    const double* pGoal = NULL;
    unsigned int uCount = 0;
    const bool bSlot = (m_eMotion == eMotionTrack && m_pGoalSlot != NULL && m_pGoalSlot->Read(adGoal, &uCount));
    if (bSlot)                  // the goal p_g: the hover point above the object (application), or the goal itself
        for (int k = 0; k < 3; ++k)
            adGoal[k] += m_adGoalOffset[k];
    if (bSlot && uCount != m_uGoalCountSeen) {
        m_uGoalCountSeen = uCount;
        m_nGoalAge = 0;
    } else
        m_nGoalAge++;
    if (bSlot && uCount != m_uGoalCountStart)
        pGoal = adGoal;

    const BOOL bOk = m_hl.Update(pGoal, adQ, adQd, adQRef, adQdRef, adQddRef);

    // the cycle for the log
    const CHighLevelRT::Status& st = m_hl.GetStatus();
    Sample& s = m_sample;
    s.dT = m_nCycle * m_dTs;
    s.nMode = (m_eMotion == eMotionHome) ? 1 : 0;
    s.uFlags = (st.bGoalAccepted ? HL_GOAL_ACCEPTED : 0) | (st.bIkConverged ? HL_IK_CONVERGED : 0) |
               (st.bIkRejected ? HL_IK_REJECTED : 0) | (st.bIkActive ? HL_IK_ACTIVE : 0) |
               (st.bOtgNewCalc ? HL_OTG_NEW : 0) | (st.bAtRest ? HL_AT_REST : 0) | (st.bHasGoal ? HL_HAS_GOAL : 0);
    s.nOtg = st.nOtg;
    s.uLeadReset = st.uLeadReset;
    s.nIkIter = st.nIkIter;
    s.dIkErr = st.dIkErr;
    s.dOtgDuration = st.dOtgDuration;
    s.dHlUs = st.dUs;
    s.nGoalAge = m_nGoalAge;
    for (int k = 0; k < 3; ++k) {
        s.adGoal[k] = pGoal ? adGoal[k] : std::numeric_limits<double>::quiet_NaN();
        s.adAccepted[k] = st.adGoal[k];
    }
    m_hl.ToolPosition(adQ, s.adTool);
    for (int i = 0; i < nN; ++i) {
        s.adTarget[i] = st.adTarget[i];
        s.adQRef[i] = adQRef[i];
        s.adQdRef[i] = adQdRef[i];
        s.adQddRef[i] = adQddRef[i];
        s.adQ[i] = adQ[i];
        s.adQd[i] = adQd[i];
        s.adTau[i] = 0.0;           // the Trajectory task fills the torque from the TorqueSlot
    }
    s.uClamp = 0;
    m_bSample = TRUE;
    m_nCycle++;

    if (!bOk)
        m_ePending = eError;
    else if (m_eMotion == eMotionHome && st.bAtRest)
        m_ePending = eDoneAtRest;
    return bOk;
}
