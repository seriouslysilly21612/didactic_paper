/*****************************************************************************
*	Name: FullDynControllerRT.cpp
*	Author: Raimarius Delgado
*	Affiliation: RAIMLAB @ Myongji University
*	Description: Implementation of RT-optimized full dynamics controller
*	Copyright: RAIMLAB (2025)
*****************************************************************************/

#include "FullDynControllerRT.h"
#include "CallTiming.h"
#include <cstdlib>
#include <algorithm>
#include <cmath>
#include <climits>
#include <limits>
#include <errno.h>
#include <fstream>
#include <ctime>
#include <sys/stat.h>
/* SSE headers exist only on x86; the FTZ/DAZ intrinsics they serve are
 * __SSE__-guarded at the call sites as well. */
#if defined(__SSE__)
#include <xmmintrin.h>
#include <pmmintrin.h>
#endif

CControllerFullDynamicsRT::CControllerFullDynamicsRT(const TSTRING& astrURDFPath, unsigned int auDOF)
    : CController(eControllerFullDynamics, auDOF)  // Use MPC type for full dynamics
    , m_strURDFPath(astrURDFPath)
    , m_eControlMode(eGravityCompensation)
    , m_bRTMode(FALSE)
    , m_bRTOptimized(FALSE)
{
    // Initialize performance monitoring
    memset(&m_rt_perf, 0, sizeof(m_rt_perf));
    m_rt_perf.deadline_ns = 500000;  // 500us deadline for 1kHz control
    
    // Pre-allocate all RBDL vectors
    m_Q.resize(auDOF);
    m_Qd.resize(auDOF);
    m_Qdd.resize(auDOF);
    m_Q_ref.resize(auDOF);
    m_Qd_ref.resize(auDOF);
    m_Qdd_ref.resize(auDOF);
    m_zero_vector.resize(auDOF);
    // sized here: its first assignment in the control loop (the
    // grav-comp hold anchoring) resized it on the heap
    m_Q_hold.resize(auDOF);
    m_Q_hold.setZero();

    m_M.resize(auDOF, auDOF);
    m_h.resize(auDOF);
    m_g.resize(auDOF);
    
    m_tau_M.resize(auDOF);
    m_tau_feedback.resize(auDOF);
    m_tau_total.resize(auDOF);
    
    m_pos_error.resize(auDOF);
    m_vel_error.resize(auDOF);
    
    // Initialize all vectors to zero
    m_Q.setZero();
    m_Qd.setZero();
    m_Qdd.setZero();
    m_Q_ref.setZero();
    m_Qd_ref.setZero();
    m_Qdd_ref.setZero();
    m_zero_vector.setZero();

    m_M.setZero();
    m_h.setZero();
    m_g.setZero();
    
    m_tau_M.setZero();
    m_tau_feedback.setZero();
    m_tau_total.setZero();
    
    m_pos_error.setZero();
    m_vel_error.setZero();
    
    // Initialize control gains
    m_Kp.resize(auDOF, 100.0);  // Default position gains
    m_Kd.resize(auDOF, 10.0);   // Default velocity gains
    m_Kf.resize(auDOF, 0.0);    // Friction FF off unless the cfg says so
    m_B.resize(auDOF, 0.0);     // no rotor inertia unless the cfg says so

    // JOINT TEST and torque limits
    m_vJtMqdd.resize(auDOF);
    m_vJtMqdd.setZero();
    {
        // URDF effort limits; SetTorqueLimits lowers them to the cfg values
        static constexpr double s_adEffort[8] =
            {431.97, 431.97, 197.23, 79.79, 79.79, 79.79, 79.79, 79.79};
        for (int i = 0; i < 8; ++i)
            m_adTauMax[i] = s_adEffort[i];
    }

    DBG_LOG_INFO("(%s) Constructor completed for %u DOF", "CControllerFullDynamicsRT", auDOF);
}

CControllerFullDynamicsRT::~CControllerFullDynamicsRT()
{
    if (m_bRTMode && m_bRTOptimized) {
        // Unlock memory
        munlockall();
    }

    DBG_LOG_INFO("(%s) Destructor completed", "CControllerFullDynamicsRT");
}

BOOL 
CControllerFullDynamicsRT::Init()
{
    DBG_LOG_INFO("(%s) Initializing controller...", "CControllerFullDynamicsRT");
    
    // Load URDF model
    if (!LoadURDF(m_strURDFPath)) {
        DBG_LOG_ERROR("(%s) Failed to load URDF: %s", "CControllerFullDynamicsRT", m_strURDFPath.c_str());
        return FALSE;
    }
    
    // Set default gravity
    SetGravity(RigidBodyDynamics::Math::Vector3d(0.0, 0.0, -9.81));
    
    // Verify DOF consistency
    if (m_rbdlModel.qdot_size != m_uDOF) {
        DBG_LOG_ERROR("(%s) DOF mismatch: Model=%u, Expected=%u", 
                     "CControllerFullDynamicsRT", m_rbdlModel.qdot_size, m_uDOF);
        return FALSE;
    }
    
    // Initialize RT optimizations if enabled
    if (m_bRTMode) {
        if (!InitRTOptimizations()) {
            DBG_LOG_ERROR("(%s) Failed to initialize RT optimizations", "CControllerFullDynamicsRT");
            return FALSE;
        }
    }
    
    m_bInitialized = TRUE;
    DBG_LOG_INFO("(%s) Controller initialized successfully with %u DOF", 
                 "CControllerFullDynamicsRT", m_uDOF);
    
    // the tool frame of the URDF ("tcp", a massless child of the flange)
    m_body_id = m_rbdlModel.GetBodyId("tcp");

    return TRUE;
}

BOOL 
CControllerFullDynamicsRT::Update(const std::vector<double>& avCurrentPos, 
                                      const std::vector<double>& avCurrentVel,
                                      const std::vector<double>& avCurrentTor,
                                      std::vector<double>& avOutputTorque)
{
    m_bJtSample = FALSE;
    m_uDynNs = 0;
    if (!m_bInitialized || !m_bEnabled) {
        std::fill(avOutputTorque.begin(), avOutputTorque.end(), 0.0);
        return FALSE;
    }
    
    // Start RT timing
    uint64_t t_start = read_timer();
    
    // Input validation
    if (avCurrentPos.size() != m_uDOF || avCurrentVel.size() != m_uDOF || 
        avOutputTorque.size() != m_uDOF) {
        DBG_LOG_ERROR("(%s) Size mismatch: Pos=%zu, Vel=%zu, Output=%zu, DOF=%u", 
                     "CControllerFullDynamicsRT", avCurrentPos.size(), avCurrentVel.size(), 
                     avOutputTorque.size(), m_uDOF);
        return FALSE;
    }

    // the measured state must be finite: a NaN or an infinity
    // would go through RBDL into every torque (the backstop below would then
    // fail the cycle) and could stay in the state of the hold (its anchor).
    // Fail the cycle here instead, before the law, with zero torque, and say
    // which joint and which quantity. The measured torque is not checked: the
    // law does not use it.
    for (unsigned int i = 0; i < m_uDOF; ++i) {
        const bool bQ = std::isfinite(avCurrentPos[i]);
        if (!bQ || !std::isfinite(avCurrentVel[i])) {
            std::fill(avOutputTorque.begin(), avOutputTorque.end(), 0.0);
            static uint32_t s_uStateCnt = 0;
            if ((s_uStateCnt++ % 1000) == 0)
                DBG_LOG_ERROR("(FullDynRT) non-finite measured %s of J%u - "
                              "output zeroed (x%u)", bQ ? "velocity" : "angle", i + 1, s_uStateCnt);
            m_uLastClamp = 0;
            return FALSE;
        }
    }

    // Copy current state to RBDL vectors (RT-safe)
    for (unsigned int i = 0; i < m_uDOF; ++i) {
        m_Q[i] = avCurrentPos[i];
        m_Qd[i] = avCurrentVel[i];
    }

    // sticky-float anchor is only meaningful while grav-comp is the ACTIVE
    // mode - re-anchor fresh on the next grav-comp entry
    if (m_eControlMode != eGravityCompensation)
        m_bHoldAnchored = false;

    // Compute control based on selected mode
    BOOL result = FALSE;
    switch (m_eControlMode) {
        case eGravityCompensation:
            result = ComputeGravityCompensation(avOutputTorque);
            break;
        case eJointTest:
            // prescribed reference, then the law of the test
            UpdateJointTest();
            if (m_jt.eLaw == eJtPDG)
                result = ComputePDGravity(avOutputTorque);
            else if (m_jt.eLaw == eJtStep || m_jt.eLaw == eJtRamp)
                result = ComputeGravityStep(avOutputTorque);
            else
                result = ComputeComputedTorque(avOutputTorque);
            FillJointTestTerms();
            m_bJtSample = TRUE;
            break;
        case eTrack:
        case eHome:
            // CTC on the reference of the Trajectory task
            // (SetReference, this period); gravity compensation without one
            result = ComputeReferenceTracking(avOutputTorque);
            break;

        default:
            result = ComputeGravityCompensation(avOutputTorque);
            break;
    }

    // fault injection (InjectTorqueFault; simulation only)
    if (m_nInjectJoint >= 0 && m_nInjectJoint < (int)m_uDOF) {
        avOutputTorque[m_nInjectJoint] = std::numeric_limits<double>::quiet_NaN();
        if (!m_bInjectPersist)
            m_nInjectJoint = -1;
    }

    // Torque backstop: nothing downstream saturates (SaturateOutput is never
    // wired and MoveTorque writes drive current unchecked). Limit per joint:
    // the cfg TORQUE_LIMIT_U (tau_max of Table 7), never above the URDF effort
    // limit (SetTorqueLimits); any non-finite value means the dynamics are
    // poisoned - zero ALL joints and fail the cycle (the FALSE path of the
    // Dynamics task already writes zero torque) rather than emit full-scale
    // current. The Dynamics task latches a controller fault on the FALSE
    // (TaskDynamics.cpp): the motion ends, the drives go off.
    unsigned int uClamp = 0;
    {
        for (unsigned int i = 0; i < m_uDOF; ++i) {
            if (!std::isfinite(avOutputTorque[i])) {
                std::fill(avOutputTorque.begin(), avOutputTorque.end(), 0.0);
                static uint32_t s_uNanCnt = 0;
                if ((s_uNanCnt++ % 1000) == 0)
                    DBG_LOG_ERROR("(FullDynRT) non-finite torque of J%u - "
                                  "output zeroed (x%u)", i + 1, s_uNanCnt);
                result = FALSE;
                break;
            }
            const double dMax = (i < 8) ? m_adTauMax[i] : 79.79;
            if (avOutputTorque[i] >  dMax) { avOutputTorque[i] =  dMax; uClamp |= 1u << i; }
            if (avOutputTorque[i] < -dMax) { avOutputTorque[i] = -dMax; uClamp |= 1u << i; }
        }
    }

    // TRACK / HOME: the commanded torque and the limit flags go to the Torque
    // slot (the Dynamics task), where the Trajectory task takes
    // them for its log; HOME at rest and a generator error end the motion in
    // the Trajectory task, which then publishes no reference
    m_uLastClamp = uClamp;

    // JOINT TEST: the commanded torque for the log, and the end
    // of the test once the arm is at rest after the last dwell
    if (m_bJtSample) {
        for (unsigned int i = 0; i < m_uDOF && i < (unsigned int)JT_MAX_DOF; ++i)
            m_jtSample.adTau[i] = avOutputTorque[i];
        m_jtSample.uClamp = uClamp;
        const int nEnd = (m_jt.eLaw == eJtStep) ? m_jt.nStepEnd :
                         (m_jt.eLaw == eJtRamp) ? (m_jt.nPhase == 3 ? m_jt.nStepEnd : INT_MAX / 2) :
                         m_jt.bSine ? m_jt.nSineEnd :
                         2 * m_jt.nT + 2 * m_jt.nW;
        if (m_jt.eState == eJtRunning && m_jt.n >= nEnd) {
            if (m_jt.nRest >= JT_REST_CYC) {
                EndJointTest(eJtDoneAtRest);
                SetControlMode(eGravityCompensation);
            } else if (m_jt.n >= nEnd + (int)std::lround(JT_REST_TMAX / m_dt)) {
                EndJointTest(eJtDoneTimeout);
                SetControlMode(eGravityCompensation);
            }
        }
    }

    // RT performance monitoring
    uint64_t t_end = read_timer();
    uint64_t computation_time = t_end - t_start;
    
    // Update performance statistics
    m_rt_perf.total_compute_time_ns += computation_time;
    m_rt_perf.cycle_count++;
    
    if (computation_time > m_rt_perf.max_compute_time_ns) {
        m_rt_perf.max_compute_time_ns = computation_time;
    }
    
    m_rt_perf.avg_compute_time_ns = m_rt_perf.total_compute_time_ns / m_rt_perf.cycle_count;
    
    // Check for RT violations
    if (m_bRTMode) {
        CheckRTViolation(computation_time);
    }
    
    return result;
}

BOOL 
CControllerFullDynamicsRT::ComputeGravityCompensation(std::vector<double>& avOutputTorque)
{
    // Gravity compensation: tau = g(q)
    const uint64_t tDyn0 = read_timer();
    RigidBodyDynamics::InverseDynamics(m_rbdlModel, m_Q, m_zero_vector, m_zero_vector, m_g);
    m_uDynNs += read_timer() - tDyn0;

    for (unsigned int i = 0; i < m_uDOF; ++i) {
        avOutputTorque[i] = m_g[i];
    }

    // Sticky-float hold (see header). Only when grav-comp is
    // the ACTIVE mode - this function also serves as a fallback for other
    // modes, which must stay pure g(q).
    if (m_eControlMode == eGravityCompensation &&
        m_bStickyEnable.load(std::memory_order_relaxed))
    {
        if (!m_bHoldAnchored)
        {
            m_Q_hold = m_Q;
            m_bHoldAnchored = true;
        }
        for (unsigned int i = 0; i < m_uDOF; ++i)
        {
            if (std::fabs(m_Qd[i]) > HOLD_UNLOCK_QD)
            {
                // clearly being hand-guided - anchor follows, no spring
                m_Q_hold[i] = m_Q[i];
                continue;
            }
            double dErr = m_Q[i] - m_Q_hold[i];
            if (dErr > HOLD_DB_RAD)          // pushed past the dead-band:
                m_Q_hold[i] = m_Q[i] - HOLD_DB_RAD;   // the anchor drags along
            else if (dErr < -HOLD_DB_RAD)
                m_Q_hold[i] = m_Q[i] + HOLD_DB_RAD;
            dErr = m_Q[i] - m_Q_hold[i];
            avOutputTorque[i] += -HOLD_KP_FRAC * m_Kp[i] * dErr
                                 - HOLD_KD_FRAC * m_Kd[i] * m_Qd[i];
        }
    }

    return TRUE;
}

BOOL CControllerFullDynamicsRT::ComputeComputedTorque(std::vector<double>& avOutputTorque)
{
    // Computed torque control: tau = M(q)[qdd_ref + Kp*e_pos + Kd*e_vel] + h(q,qd)

    // Compute dynamics, without allocation (paper, Sec. IV-A): the
    // kinematics are updated once, CRBA uses them
    // (update_kinematics = false; with true, RBDL allocates a temporary
    // vector in every call), M is zeroed first because CRBA fills only its
    // nonzero entries, and NonlinearEffects, which has no such flag, does not
    // allocate.
    const uint64_t tDyn0 = read_timer();
    RigidBodyDynamics::UpdateKinematics(m_rbdlModel, m_Q, m_Qd, m_zero_vector);
    RTTIME tCt = call_timing::Lap(call_timing::eCtcKin, tDyn0);     // call timing (CallTiming.h)
    m_M.setZero();
    RigidBodyDynamics::CompositeRigidBodyAlgorithm(m_rbdlModel, m_Q, m_M, false);
    // the identified rotor inertias on the diagonal: M + B (paper, eqs. (4)
    // and (5)); B = 0 where none is configured
    for (unsigned int i = 0; i < m_uDOF; ++i)
        m_M(i, i) += m_B[i];
    tCt = call_timing::Lap(call_timing::eCtcCrba, tCt);
    RigidBodyDynamics::NonlinearEffects(m_rbdlModel, m_Q, m_Qd, m_h);
    call_timing::Lap(call_timing::eCtcNle, tCt);
    m_uDynNs += read_timer() - tDyn0;

    // Compute errors
    m_pos_error = m_Q_ref - m_Q;
    m_vel_error = m_Qd_ref - m_Qd;

    // Desired acceleration with feedback
    for (unsigned int i = 0; i < m_uDOF; ++i) {
        m_Qdd[i] = m_Qdd_ref[i] + m_Kp[i] * m_pos_error[i] + m_Kd[i] * m_vel_error[i];
    }
    
    // Computed torque: tau = M(q) * qdd_desired + h(q,qd)
    // noalias: a dynamic-size product assigned without it is
    // evaluated into a temporary, which allocates
    m_tau_M.noalias() = m_M * m_Qdd;
    m_tau_total = m_tau_M + m_h;

    // Friction feedforward, eq. (7) of the paper: K_f sat(qd_d/eps_v)
    // gamma with eps_v = 0.01 rad/s and gamma = clamp(e sign(qd_d)/eps_q, 0, 1),
    // eps_q = 2 mrad. Only a joint that lags its reference in the direction of
    // motion receives it (one that sticks or has not caught up); a joint that
    // has caught up or overtaken its reference receives none, and friction
    // brakes it. Exactly zero when the reference is at rest, so it cannot buzz
    // at rest or hunt like an integrator. m_dKfScale switches it per motion.
    for (unsigned int i = 0; i < m_uDOF; ++i) {
        if (m_Kf[i] > 0.0 && m_dKfScale > 0.0) {
            double dS = m_Qd_ref[i] * 100.0;            // /0.01 rad/s
            if (dS > 1.0) dS = 1.0; else if (dS < -1.0) dS = -1.0;
            const double dLag = (m_Q_ref[i] - m_Q[i]) * (dS >= 0.0 ? 1.0 : -1.0);
            double dG = dLag / 0.002;
            if (dG > 1.0) dG = 1.0; else if (dG < 0.0) dG = 0.0;
            if (!m_bKfGate)
                dG = 1.0;                               // the ablation: no gate
            m_tau_total[i] += m_Kf[i] * m_dKfScale * dS * dG;
        }
    }

    // Copy result to output
    for (unsigned int i = 0; i < m_uDOF; ++i) {
        avOutputTorque[i] = m_tau_total[i];
    }
    
    return TRUE;
}

//============================================================================
// JOINT TEST (paper, Sec. V-A). See the header.
//============================================================================
BOOL
CControllerFullDynamicsRT::StartJointTest(const double* adDq, double adT, double adDwell,
                                          eJointTestLaw aeLaw, char* astrWhy, size_t auWhyLen)
{
    // URDF joint limits (indy7.urdf): J1-J5 175 deg, J6 215 deg
    static const double s_adLim[JT_MAX_DOF] = {3.0543261909900767, 3.0543261909900767,
                                               3.0543261909900767, 3.0543261909900767,
                                               3.0543261909900767, 3.7524578917878086};
    if (m_uDOF > (unsigned int)JT_MAX_DOF) {
        snprintf(astrWhy, auWhyLen, "%u joints, at most %d", m_uDOF, JT_MAX_DOF);
        return FALSE;
    }
    if (m_eControlMode != eGravityCompensation) {
        snprintf(astrWhy, auWhyLen, "start from gravity compensation ('g')");
        return FALSE;
    }
    if (!(adT >= 0.2 && adT <= 20.0) || !(adDwell >= 0.0 && adDwell <= 10.0)) {
        snprintf(astrWhy, auWhyLen, "T %.3g s (0.2..20), dwell %.3g s (0..10)", adT, adDwell);
        return FALSE;
    }
    double dQdSq = 0.0;
    for (unsigned int i = 0; i < m_uDOF; ++i)
        dQdSq += m_Qd[i] * m_Qd[i];
    if (dQdSq > JT_REST_QD * JT_REST_QD) {
        snprintf(astrWhy, auWhyLen, "the arm is moving (|qd| %.3f rad/s > %.2f)",
                 std::sqrt(dQdSq), JT_REST_QD);
        return FALSE;
    }
    for (unsigned int i = 0; i < m_uDOF; ++i) {
        const double dLim = s_adLim[i] - JT_LIMIT_MARGIN;
        const double dQ1 = m_Q[i] + adDq[i];
        if (std::fabs(m_Q[i]) > dLim || std::fabs(dQ1) > dLim) {
            snprintf(astrWhy, auWhyLen, "J%u: q0 %.3f or q1 %.3f rad beyond +-%.3f (limit - %.1f)",
                     i + 1, m_Q[i], dQ1, dLim, JT_LIMIT_MARGIN);
            return FALSE;
        }
        // peak speed and acceleration of the quintic: 1.875 dq/T, 5.774 dq/T^2,
        // within the joint limits of the high-level controller
        const double dV = 1.875 * std::fabs(adDq[i]) / adT;
        const double dA = 5.7735026918962576 * std::fabs(adDq[i]) / (adT * adT);
        if (dV > JT_QD_PEAK + 1e-9 || dA > JT_QDD_PEAK + 1e-9) {
            snprintf(astrWhy, auWhyLen, "J%u: peak %.2f rad/s, %.2f rad/s^2 (limits %.1f, %.1f)",
                     i + 1, dV, dA, JT_QD_PEAK, JT_QDD_PEAK);
            return FALSE;
        }
    }
    // J6 follows a reference only with its rotor inertia in the model (paper,
    // Sec. IV-A)
    if (m_uDOF == 6 && adDq[5] != 0.0 && m_B[5] <= 0.0) {
        snprintf(astrWhy, auWhyLen, "J6 moves only with its rotor inertia (ROTOR_INERTIA_5)");
        return FALSE;
    }
    for (int i = 0; i < JT_MAX_DOF; ++i) {
        m_jt.adDq[i] = (i < (int)m_uDOF) ? adDq[i] : 0.0;
        m_jt.adQ0[i] = 0.0;
        m_jt.adD[i]  = 0.0;
    }
    m_jt.bSine  = false;
    m_jt.eLaw   = aeLaw;
    m_jt.nT     = (int)std::lround(adT / m_dt);
    m_jt.nW     = (int)std::lround(adDwell / m_dt);
    m_jt.n      = 0;
    m_jt.nPhase = 0;
    m_jt.nRest  = 0;
    m_jt.bFirst = true;
    m_jt.dKfScaleSaved = m_dKfScale;
    // the feedforward of eq. (7) only in CTC + FF
    m_dKfScale  = (aeLaw == eJtCTCFF) ? 1.0 : 0.0;
    m_jt.eState = eJtRunning;
    SetControlMode(eJointTest);
    return TRUE;
}

// the torque-step delay test
BOOL
CControllerFullDynamicsRT::StartTorqueStep(int anJoint, double adTau, double adOnS, double adOffS, int anRep,
                                           char* astrWhy, size_t auWhyLen)
{
    if (m_uDOF > (unsigned int)JT_MAX_DOF) {
        snprintf(astrWhy, auWhyLen, "%u joints, at most %d", m_uDOF, JT_MAX_DOF);
        return FALSE;
    }
    if (m_eControlMode != eGravityCompensation) {
        snprintf(astrWhy, auWhyLen, "start from gravity compensation ('g')");
        return FALSE;
    }
    if (anJoint < 0 || anJoint >= (int)m_uDOF) {
        snprintf(astrWhy, auWhyLen, "joint %d (1..%u)", anJoint + 1, m_uDOF);
        return FALSE;
    }
    const double dCap = std::min(TS_MAX_NM, 0.25 * m_adTauMax[anJoint]);
    if (!(std::fabs(adTau) > 0.0 && std::fabs(adTau) <= dCap)) {
        snprintf(astrWhy, auWhyLen, "step %.3g N m (0 < |step| <= %.3g)", adTau, dCap);
        return FALSE;
    }
    if (!(adOnS >= 0.02 && adOnS <= 2.0) || !(adOffS >= 0.02 && adOffS <= 2.0) || anRep < 1 || anRep > 200) {
        snprintf(astrWhy, auWhyLen, "on %.3g s, off %.3g s (0.02..2), %d steps (1..200)", adOnS, adOffS, anRep);
        return FALSE;
    }
    double dQdSq = 0.0;
    for (unsigned int i = 0; i < m_uDOF; ++i)
        dQdSq += m_Qd[i] * m_Qd[i];
    if (dQdSq > JT_REST_QD * JT_REST_QD) {
        snprintf(astrWhy, auWhyLen, "the arm is moving (|qd| %.3f rad/s > %.2f)", std::sqrt(dQdSq), JT_REST_QD);
        return FALSE;
    }
    for (int i = 0; i < JT_MAX_DOF; ++i) {
        m_jt.adDq[i] = 0.0;
        m_jt.adQ0[i] = 0.0;
        m_jt.adD[i]  = 0.0;
    }
    m_jt.eLaw       = eJtStep;
    m_jt.nT         = 0;
    m_jt.nW         = 0;
    m_jt.nStepJoint = anJoint;
    m_jt.dStepTau   = adTau;
    m_jt.nOn        = (int)std::lround(adOnS / m_dt);
    m_jt.nOff       = (int)std::lround(adOffS / m_dt);
    m_jt.nRep       = anRep;
    m_jt.nStepEnd   = anRep * (m_jt.nOn + m_jt.nOff);
    m_jt.dStepNow   = 0.0;
    m_jt.n          = 0;
    m_jt.nPhase     = 0;
    m_jt.nRest      = 0;
    m_jt.bFirst     = true;
    m_jt.dKfScaleSaved = m_dKfScale;
    m_dKfScale      = 0.0;
    m_jt.eState     = eJtRunning;
    SetControlMode(eJointTest);
    return TRUE;
}

// the breakaway test (tau_s, and the sign of the torque)
BOOL
CControllerFullDynamicsRT::StartBreakaway(int anJoint, double adRate, double adMax, double adQd, int anRep,
                                          char* astrWhy, size_t auWhyLen)
{
    if (m_uDOF > (unsigned int)JT_MAX_DOF) {
        snprintf(astrWhy, auWhyLen, "%u joints, at most %d", m_uDOF, JT_MAX_DOF);
        return FALSE;
    }
    if (m_eControlMode != eGravityCompensation) {
        snprintf(astrWhy, auWhyLen, "start from gravity compensation ('g')");
        return FALSE;
    }
    if (anJoint < 0 || anJoint >= (int)m_uDOF) {
        snprintf(astrWhy, auWhyLen, "joint %d (1..%u)", anJoint + 1, m_uDOF);
        return FALSE;
    }
    if (!(adMax > 0.0 && adMax <= 0.8 * m_adTauMax[anJoint]) || !(adRate >= 0.1 && adRate <= 10.0) ||
        !(adQd >= 0.005 && adQd <= 0.2) || anRep < 1 || anRep > 50) {
        snprintf(astrWhy, auWhyLen, "max %.3g N m (0..%.3g), rate %.3g N m/s (0.1..10), qd %.3g rad/s (0.005..0.2), "
                 "%d ramps (1..50)", adMax, 0.8 * m_adTauMax[anJoint], adRate, adQd, anRep);
        return FALSE;
    }
    double dQdSq = 0.0;
    for (unsigned int i = 0; i < m_uDOF; ++i)
        dQdSq += m_Qd[i] * m_Qd[i];
    if (dQdSq > JT_REST_QD * JT_REST_QD) {
        snprintf(astrWhy, auWhyLen, "the arm is moving (|qd| %.3f rad/s > %.2f)", std::sqrt(dQdSq), JT_REST_QD);
        return FALSE;
    }
    for (int i = 0; i < JT_MAX_DOF; ++i) {
        m_jt.adDq[i] = 0.0;
        m_jt.adQ0[i] = 0.0;
        m_jt.adD[i]  = 0.0;
    }
    m_jt.eLaw       = eJtRamp;
    m_jt.nT         = 0;
    m_jt.nW         = 0;
    m_jt.nStepJoint = anJoint;
    m_jt.dRampRate  = adRate;
    m_jt.dRampMax   = adMax;
    m_jt.dRampQd    = adQd;
    m_jt.nRep       = 2 * anRep;          // both directions
    m_jt.nRampDone  = 0;
    m_jt.nRampCyc   = 0;
    m_jt.nRampStill = 0;
    m_jt.dRampQ0    = m_Q[anJoint];
    m_jt.dStepNow   = 0.0;
    m_jt.n          = 0;
    m_jt.nPhase     = 0;
    m_jt.nRest      = 0;
    m_jt.bFirst     = true;
    m_jt.dKfScaleSaved = m_dKfScale;
    m_dKfScale      = 0.0;
    m_jt.eState     = eJtRunning;
    SetControlMode(eJointTest);
    return TRUE;
}

// tau = g(q) + the step of this cycle on the joint of the test; no feedback
BOOL
CControllerFullDynamicsRT::ComputeGravityStep(std::vector<double>& avOutputTorque)
{
    const uint64_t tDyn0 = read_timer();
    RigidBodyDynamics::InverseDynamics(m_rbdlModel, m_Q, m_zero_vector, m_zero_vector, m_g);
    m_uDynNs += read_timer() - tDyn0;
    for (unsigned int i = 0; i < m_uDOF; ++i) {
        m_pos_error[i] = m_Q_ref[i] - m_Q[i];
        m_vel_error[i] = -m_Qd[i];
        m_tau_feedback[i] = 0.0;
        avOutputTorque[i] = m_g[i] + ((int)i == m_jt.nStepJoint ? m_jt.dStepNow : 0.0);
    }
    // breakaway test, phase 1: the joint of the test is damped to
    // rest with the damping of CTC at q0 (diag M(q0) Kd)
    if (m_jt.eLaw == eJtRamp && m_jt.nPhase == 1) {
        const int j = m_jt.nStepJoint;
        avOutputTorque[j] -= m_jt.adD[j] * m_Kd[j] * m_Qd[j];
    }
    return TRUE;
}

void
CControllerFullDynamicsRT::EndJointTest(eJointTestState aeState)
{
    if (m_jt.eState != eJtRunning)
        return;
    m_jt.eState = aeState;
    m_dKfScale  = m_jt.dKfScaleSaved;
}

void
CControllerFullDynamicsRT::UpdateJointTest()
{
    JointTest& jt = m_jt;
    const unsigned int uN = m_uDOF;
    if (jt.bFirst) {
        // the reference starts from the measured state at rest, (q0, 0, 0)
        for (unsigned int i = 0; i < uN; ++i)
            jt.adQ0[i] = m_Q[i];
        RigidBodyDynamics::UpdateKinematics(m_rbdlModel, m_Q, m_Qd, m_zero_vector);
        m_M.setZero();
        RigidBodyDynamics::CompositeRigidBodyAlgorithm(m_rbdlModel, m_Q, m_M, false);
        for (unsigned int i = 0; i < uN; ++i)
            jt.adD[i] = m_M(i, i) + m_B[i];      // diag (M(q0) + B), for PD + g
        jt.bFirst = false;
    }
    // breakaway test: phase 0 ramp, 1 damped to rest, 3 done
    if (jt.eLaw == eJtRamp) {
        const int j = jt.nStepJoint;
        for (unsigned int i = 0; i < uN; ++i) {
            m_Q_ref[i]   = jt.adQ0[i];
            m_Qd_ref[i]  = 0.0;
            m_Qdd_ref[i] = 0.0;
        }
        if (std::fabs(m_Q[j] - jt.adQ0[j]) > BA_ABORT_RAD && jt.nPhase != 3) {
            jt.nPhase = 1;                      // too far: damp, then end
            jt.nRampDone = jt.nRep;
        }
        if (jt.nPhase == 0) {
            const double dSign = (jt.nRampDone % 2 == 0) ? 1.0 : -1.0;
            const double dExtra = jt.dRampRate * (jt.nRampCyc + 1) * m_dt;
            const bool bMoved = std::fabs(m_Qd[j]) > jt.dRampQd || std::fabs(m_Q[j] - jt.dRampQ0) > BA_MOVE_RAD;
            if (bMoved || dExtra > jt.dRampMax) {
                jt.dStepNow = 0.0;              // moved (or the maximum): remove the torque, damp
                jt.nPhase = 1;
                jt.nRampCyc = 0;
                jt.nRampStill = 0;
            } else {
                jt.dStepNow = dSign * dExtra;
                jt.nRampCyc++;
            }
        } else if (jt.nPhase == 1) {
            jt.dStepNow = 0.0;
            jt.nRampCyc++;
            jt.nRampStill = (std::fabs(m_Qd[j]) <= 0.01) ? jt.nRampStill + 1 : 0;
            if (jt.nRampStill >= JT_REST_CYC && jt.nRampCyc >= 500) {
                jt.nRampDone++;
                jt.nRampCyc = 0;
                jt.dRampQ0 = m_Q[j];
                jt.nPhase = (jt.nRampDone >= jt.nRep) ? 3 : 0;
                if (jt.nPhase == 3)
                    jt.nStepEnd = jt.n;         // the end check counts the rest from here
            }
        } else {
            jt.dStepNow = 0.0;
        }
        if (jt.nPhase == 3) {
            double dQdSq = 0.0;
            for (unsigned int i = 0; i < uN; ++i)
                dQdSq += m_Qd[i] * m_Qd[i];
            jt.nRest = (dQdSq <= JT_REST_QD * JT_REST_QD) ? jt.nRest + 1 : 0;
        }
        jt.n++;
        return;
    }
    // torque step: the reference stays at q0; phase 1 while the
    // step is on (sign alternating from step to step), 0 while off, 3 after
    // the last step (then at rest as below)
    if (jt.eLaw == eJtStep) {
        const int n = jt.n;
        const int nPer = jt.nOn + jt.nOff;
        int nPhase = 3;
        jt.dStepNow = 0.0;
        if (n < jt.nStepEnd && nPer > 0) {
            const int k = n % nPer, nStep = n / nPer;
            nPhase = (k >= jt.nOff) ? 1 : 0;
            if (nPhase == 1)
                jt.dStepNow = (nStep % 2 == 0) ? jt.dStepTau : -jt.dStepTau;
        }
        for (unsigned int i = 0; i < uN; ++i) {
            m_Q_ref[i]   = jt.adQ0[i];
            m_Qd_ref[i]  = 0.0;
            m_Qdd_ref[i] = 0.0;
        }
        if (nPhase == 3) {
            double dQdSq = 0.0;
            for (unsigned int i = 0; i < uN; ++i)
                dQdSq += m_Qd[i] * m_Qd[i];
            jt.nRest = (dQdSq <= JT_REST_QD * JT_REST_QD) ? jt.nRest + 1 : 0;
        }
        jt.nPhase = nPhase;
        jt.n = n + 1;
        return;
    }
    // sine test: joint j follows q0 + A e(t) sin(w t), with the
    // envelope e = (1 - cos(pi s / T_f)) / 2 over the first and the last T_f
    // (two periods) and e = 1 between; the other joints are held at q0.
    // Phases: 0 fade in, 1 steady, 2 fade out, 3 done (then at rest as below)
    if (jt.bSine) {
        const int n = jt.n, j = jt.nSineJoint;
        int nPhase = 3;
        double dQ = 0.0, dQd = 0.0, dQdd = 0.0;
        if (n < jt.nSineEnd) {
            const double t = n * m_dt, w = jt.dSineW, A = jt.dSineA;
            const double c = M_PI / (jt.nSineFade * m_dt);
            double e = 1.0, ed = 0.0, edd = 0.0;
            if (n < jt.nSineFade) {
                nPhase = 0;
                e = 0.5 * (1.0 - std::cos(c * t)); ed = 0.5 * c * std::sin(c * t); edd = 0.5 * c * c * std::cos(c * t);
            } else if (n >= jt.nSineEnd - jt.nSineFade) {
                nPhase = 2;
                const double s = (jt.nSineEnd - n) * m_dt;
                e = 0.5 * (1.0 - std::cos(c * s)); ed = -0.5 * c * std::sin(c * s); edd = 0.5 * c * c * std::cos(c * s);
            } else {
                nPhase = 1;
            }
            const double sn = std::sin(w * t), cs = std::cos(w * t);
            dQ   = A * e * sn;
            dQd  = A * (ed * sn + e * w * cs);
            dQdd = A * (edd * sn + 2.0 * ed * w * cs - e * w * w * sn);
            // the sweep: velocity v0 s(t), s up 0 -> 1 over Tf, 1, reversal 1 -> -1
            // over 2 Tf, -1, down -1 -> 0 over Tf (raised cosines); position in closed form
            if (jt.dSineV0 > 0.0) {
                const double v0 = jt.dSineV0, Tf = jt.nSineFade * m_dt;
                const double Tc = (jt.nSineEnd - 4 * jt.nSineFade) * m_dt / 2.0;
                const double t2 = Tf, t3 = Tf + Tc, t4 = 3.0 * Tf + Tc, t5 = 3.0 * Tf + 2.0 * Tc;
                double p, s, sd;
                if (t < t2) {
                    p = t / 2.0 - Tf / (2.0 * M_PI) * std::sin(M_PI * t / Tf);
                    s = 0.5 * (1.0 - std::cos(M_PI * t / Tf)); sd = 0.5 * M_PI / Tf * std::sin(M_PI * t / Tf);
                } else if (t < t3) {
                    p = Tf / 2.0 + (t - t2); s = 1.0; sd = 0.0;
                } else if (t < t4) {
                    const double u = (t - t3) / (2.0 * Tf);
                    p = Tf / 2.0 + Tc + 2.0 * Tf / M_PI * std::sin(M_PI * u);
                    s = std::cos(M_PI * u); sd = -M_PI / (2.0 * Tf) * std::sin(M_PI * u);
                } else if (t < t5) {
                    p = Tf / 2.0 + Tc - (t - t4); s = -1.0; sd = 0.0;
                } else {
                    const double u = (t - t5) / Tf;
                    p = Tf / 2.0 - (t - t5) / 2.0 - Tf / (2.0 * M_PI) * std::sin(M_PI * u);
                    s = -0.5 * (1.0 + std::cos(M_PI * u)); sd = 0.5 * M_PI / Tf * std::sin(M_PI * u);
                }
                dQ += v0 * p; dQd += v0 * s; dQdd += v0 * sd;
            }
        }
        for (unsigned int i = 0; i < uN; ++i) {
            m_Q_ref[i]   = jt.adQ0[i] + ((int)i == j ? dQ : 0.0);
            m_Qd_ref[i]  = ((int)i == j) ? dQd : 0.0;
            m_Qdd_ref[i] = ((int)i == j) ? dQdd : 0.0;
        }
        if (nPhase == 3) {
            double dQdSq = 0.0;
            for (unsigned int i = 0; i < uN; ++i)
                dQdSq += m_Qd[i] * m_Qd[i];
            jt.nRest = (dQdSq <= JT_REST_QD * JT_REST_QD) ? jt.nRest + 1 : 0;
        }
        jt.nPhase = nPhase;
        jt.n = n + 1;
        return;
    }
    // the quintic out (phase 0) and back (phase 2), integer cycle counts so
    // that every run has the same samples
    const int n = jt.n;
    int nPhase;
    double dS = 0.0, dSign = 0.0;
    bool bAtQ1 = false;
    if (n < jt.nT)                       { nPhase = 0; dS = (double)n / jt.nT; dSign =  1.0; }
    else if (n < jt.nT + jt.nW)          { nPhase = 1; bAtQ1 = true; }
    else if (n < 2 * jt.nT + jt.nW)      { nPhase = 2; dS = (double)(n - jt.nT - jt.nW) / jt.nT; dSign = -1.0; }
    else                                 { nPhase = 3; }
    const double dT  = jt.nT * m_dt;
    const double s2 = dS * dS, s3 = s2 * dS, s4 = s3 * dS, s5 = s4 * dS;
    const double dP   = 10.0 * s3 - 15.0 * s4 + 6.0 * s5;
    const double dPd  = (30.0 * s2 - 60.0 * s3 + 30.0 * s4) / dT;
    const double dPdd = (60.0 * dS - 180.0 * s2 + 120.0 * s3) / (dT * dT);
    for (unsigned int i = 0; i < uN; ++i) {
        const double dQ0 = jt.adQ0[i], dDq = jt.adDq[i];
        if (nPhase == 0) {
            m_Q_ref[i] = dQ0 + dDq * dP;
        } else if (nPhase == 2) {
            m_Q_ref[i] = dQ0 + dDq - dDq * dP;
        } else {
            m_Q_ref[i] = bAtQ1 ? dQ0 + dDq : dQ0;
        }
        m_Qd_ref[i]  = dSign * dDq * dPd;
        m_Qdd_ref[i] = dSign * dDq * dPdd;
    }
    // at rest: the reference has ended and |qd| <= 0.05 rad/s, for 200 ms
    if (nPhase == 3) {
        double dQdSq = 0.0;
        for (unsigned int i = 0; i < uN; ++i)
            dQdSq += m_Qd[i] * m_Qd[i];
        jt.nRest = (dQdSq <= JT_REST_QD * JT_REST_QD) ? jt.nRest + 1 : 0;
    }
    jt.nPhase = nPhase;
    jt.n = n + 1;
}

// tau = g(q) + D (Kp e + Kd ed), D = diag M(q0)
BOOL
CControllerFullDynamicsRT::ComputePDGravity(std::vector<double>& avOutputTorque)
{
    const uint64_t tDyn0 = read_timer();
    RigidBodyDynamics::InverseDynamics(m_rbdlModel, m_Q, m_zero_vector, m_zero_vector, m_g);
    m_uDynNs += read_timer() - tDyn0;
    for (unsigned int i = 0; i < m_uDOF; ++i) {
        m_pos_error[i] = m_Q_ref[i] - m_Q[i];
        m_vel_error[i] = m_Qd_ref[i] - m_Qd[i];
        m_tau_feedback[i] = m_jt.adD[i] * (m_Kp[i] * m_pos_error[i] + m_Kd[i] * m_vel_error[i]);
        avOutputTorque[i] = m_g[i] + m_tau_feedback[i];
    }
    return TRUE;
}

// the terms of the torque of this period, before the limit (the columns
// tau_g, tau_c, tau_mqdd, tau_fb, tau_ff of the joint-test log)
void
CControllerFullDynamicsRT::FillJointTestTerms()
{
    JointTestSample& s = m_jtSample;
    const unsigned int uN = m_uDOF;
    s.dT = (m_jt.n - 1) * m_dt;
    s.nPhase = m_jt.nPhase;
    if (m_jt.eLaw == eJtCTC || m_jt.eLaw == eJtCTCFF) {
        // CTC: m_M, m_h, m_tau_M = M a, m_tau_total = M a + h + ff are this
        // cycle's; g(q) only for the log
        RigidBodyDynamics::InverseDynamics(m_rbdlModel, m_Q, m_zero_vector, m_zero_vector, m_g);
        m_vJtMqdd.noalias() = m_M * m_Qdd_ref;
    }
    for (unsigned int i = 0; i < uN; ++i) {
        s.adQRef[i]   = m_Q_ref[i];
        s.adQdRef[i]  = m_Qd_ref[i];
        s.adQddRef[i] = m_Qdd_ref[i];
        s.adQ[i]      = m_Q[i];
        s.adQd[i]     = m_Qd[i];
        s.adE[i]      = m_Q_ref[i] - m_Q[i];
        s.adTauG[i]   = m_g[i];
        if (m_jt.eLaw == eJtPDG) {
            s.adTauC[i]    = 0.0;
            s.adTauMqdd[i] = 0.0;
            s.adTauFb[i]   = m_tau_feedback[i];
            s.adTauFf[i]   = 0.0;
        } else if (m_jt.eLaw == eJtStep || m_jt.eLaw == eJtRamp) {
            s.adTauC[i]    = 0.0;
            s.adTauMqdd[i] = 0.0;
            s.adTauFb[i]   = 0.0;
            s.adTauFf[i]   = ((int)i == m_jt.nStepJoint) ? m_jt.dStepNow : 0.0;
        } else {
            s.adTauC[i]    = m_h[i] - m_g[i];
            s.adTauMqdd[i] = m_vJtMqdd[i];
            s.adTauFb[i]   = m_tau_M[i] - m_vJtMqdd[i];
            s.adTauFf[i]   = m_tau_total[i] - m_tau_M[i] - m_h[i];
        }
        // M_ii(q) of the URDF without B, and in the sine test the row of
        // the tested joint j, for the identification of B: [M(q) qdd]_j includes the
        // coupling to the joints that are held but still move a little
        const bool bCtc = (m_jt.eLaw == eJtCTC || m_jt.eLaw == eJtCTCFF);
        s.adMii[i] = bCtc ? m_M(i, i) - m_B[i] : 0.0;
        const int j = m_jt.nSineJoint;
        s.adMRow[i] = (bCtc && m_jt.bSine) ? m_M(j, i) - (((int)i == j) ? m_B[j] : 0.0) : 0.0;
    }
}

//============================================================================
// TRACK and HOME: CTC on the reference of the Trajectory task (paper, Sec.
// IV-A and IV-C). The high-level controller itself runs in
// the Trajectory task (TrajectoryRT.h); the Dynamics task hands its reference
// to SetReference() before Update() in every period of the motion.
//============================================================================
void
CControllerFullDynamicsRT::SetReference(const Reference& astRef)
{
    m_ref = astRef;
    m_bRefValid = TRUE;
}

BOOL
CControllerFullDynamicsRT::ComputeReferenceTracking(std::vector<double>& avOutputTorque)
{
    if (!m_bRefValid) {
        // no reference of this period: hold in gravity compensation (the
        // mode is left to the Dynamics task, which follows the RefSlot)
        return ComputeGravityCompensation(avOutputTorque);
    }
    for (unsigned int i = 0; i < m_uDOF && i < 8; ++i) {
        m_Q_ref[i]   = m_ref.adQ[i];
        m_Qd_ref[i]  = m_ref.adQd[i];
        m_Qdd_ref[i] = m_ref.adQdd[i];
    }
    // the feedforward of eq. (7) only where the motion asks for it
    // (the robot, or the simulation with friction)
    m_dKfScale = m_ref.bFrictionFF ? 1.0 : 0.0;
    return ComputeComputedTorque(avOutputTorque);
}


void
CControllerFullDynamicsRT::SetTorqueLimits(const std::vector<double>& avTauMax)
{
    static constexpr double s_adEffort[8] =
        {431.97, 431.97, 197.23, 79.79, 79.79, 79.79, 79.79, 79.79};
    for (unsigned int i = 0; i < m_uDOF && i < 8 && i < avTauMax.size(); ++i)
        m_adTauMax[i] = (avTauMax[i] > 0.0 && avTauMax[i] < s_adEffort[i]) ? avTauMax[i] : s_adEffort[i];
    if (m_uDOF == 6)
        DBG_LOG_INFO("(%s) Torque limits [N m]: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                     "CControllerFullDynamicsRT", m_adTauMax[0], m_adTauMax[1], m_adTauMax[2],
                     m_adTauMax[3], m_adTauMax[4], m_adTauMax[5]);
}

//============================================================================
// Pose of the tool, and the model at an arbitrary state (offline tools)
//============================================================================
BOOL
CControllerFullDynamicsRT::GetCurrentPose(Pose& astCurrPose)
{
    RigidBodyDynamics::Math::Vector3d p_tcp;
    RigidBodyDynamics::Math::Matrix3d R_base_to_body, R_body_to_base;

    // tcp position 
    p_tcp = RigidBodyDynamics::CalcBodyToBaseCoordinates(m_rbdlModel, m_Q, m_body_id, tcp_local_point);
    
    // tcp orientation - (R_base_to_body : base -> body) 
    R_base_to_body = RigidBodyDynamics::CalcBodyWorldOrientation(m_rbdlModel, m_Q, m_body_id);
    R_body_to_base = R_base_to_body.transpose();
    
    // base coordinates
    astCurrPose.m_position = p_tcp;
    astCurrPose.m_rotation = R_body_to_base;
   
    return TRUE;
}

RigidBodyDynamics::Math::Matrix3d
CControllerFullDynamicsRT::ToolRotAt(const RigidBodyDynamics::Math::VectorNd& aq)
{
    // CalcBodyWorldOrientation returns base->body; Pose::m_rotation is
    // body->base (SetTargetPosePositionOnly transposes it back for RBDL).
    return RigidBodyDynamics::CalcBodyWorldOrientation(m_rbdlModel, aq,
                                                       m_body_id).transpose();
}

RigidBodyDynamics::Math::Vector3d
CControllerFullDynamicsRT::ToolPosAt(const RigidBodyDynamics::Math::VectorNd& aq)
{
    // Offline use only (model_check) - see the header note.
    return RigidBodyDynamics::CalcBodyToBaseCoordinates(m_rbdlModel, aq,
                                                        m_body_id,
                                                        tcp_local_point);
}

void
CControllerFullDynamicsRT::ModelTermsAt(const RigidBodyDynamics::Math::VectorNd& aq,
                                        const RigidBodyDynamics::Math::VectorNd& aqd,
                                        RigidBodyDynamics::Math::MatrixNd& aM,
                                        RigidBodyDynamics::Math::VectorNd& ah)
{
    // Offline use only (model_check) - see the header note.
    // CRBA writes only the nonzero entries, so M starts from zero.
    aM = RigidBodyDynamics::Math::MatrixNd::Zero(m_rbdlModel.qdot_size, m_rbdlModel.qdot_size);
    ah = RigidBodyDynamics::Math::VectorNd::Zero(m_rbdlModel.qdot_size);
    RigidBodyDynamics::CompositeRigidBodyAlgorithm(m_rbdlModel, aq, aM);
    RigidBodyDynamics::NonlinearEffects(m_rbdlModel, aq, aqd, ah);
}

BOOL
CControllerFullDynamicsRT::Reset()
{
    // Reset all vectors to zero
    m_Q.setZero();
    m_Qd.setZero();
    m_Qdd.setZero();
    m_Q_ref.setZero();
    m_Qd_ref.setZero();
    m_Qdd_ref.setZero();
    
    m_M.setZero();
    m_h.setZero();
    m_g.setZero();
    
    m_tau_M.setZero();
    m_tau_feedback.setZero();
    m_tau_total.setZero();
    
    m_pos_error.setZero();
    m_vel_error.setZero();
    
    // Reset performance counters
    ResetPerformance();
    
    DBG_LOG_INFO("(%s) Controller reset", "CControllerFullDynamicsRT");
    return TRUE;
}

void 
CControllerFullDynamicsRT::SetGains(const std::vector<double>& avGains)
{
    if (avGains.size() == m_uDOF * 2) {
        // Assume first half is Kp, second half is Kd
        for (unsigned int i = 0; i < m_uDOF; ++i) {
            m_Kp[i] = avGains[i];
            m_Kd[i] = avGains[i + m_uDOF];
        }
    }
    m_vGains = avGains;
}

BOOL
CControllerFullDynamicsRT::GetControlGain(UINT auAxis, double* adKp, double* adKd)
{
    if (auAxis >= m_uDOF) 
    {
        DBG_LOG_ERROR("(%s) Cannot get Control Gain - Invalid axis %u for DOF %u", "CControllerFullDynamicsRT", auAxis, m_uDOF);
        return FALSE;
    }
    *adKp = m_Kp[auAxis];
    *adKd = m_Kd[auAxis];
    return TRUE;
}

BOOL
CControllerFullDynamicsRT::SetControlGain(UINT auAxis, double adKp, double adKd)
{
    if (auAxis >= m_uDOF && auAxis != _ALL_AXIS) 
    {
        DBG_LOG_ERROR("(%s) Cannot set Control Gain - Invalid axis %u for DOF %u", "CControllerFullDynamicsRT", auAxis, m_uDOF);
        return FALSE;
    }
    else if (auAxis == _ALL_AXIS) 
    {
        // Set all axes to the same gains
        for (unsigned int i = 0; i < m_uDOF; ++i) 
        {
            m_Kp[i] = adKp;
            m_Kd[i] = adKd;
        }
        return TRUE;
    }
    m_Kp[auAxis] = adKp;
    m_Kd[auAxis] = adKd;
    return TRUE;
}

void 
CControllerFullDynamicsRT::SetControlGains(const std::vector<double>& avKp, const std::vector<double>& avKd)
{
    if (avKp.size() == m_uDOF && avKd.size() == m_uDOF) {
        m_Kp = avKp;
        m_Kd = avKd;
    }
}

void
CControllerFullDynamicsRT::SetFrictionFF(const std::vector<double>& avKf)
{
    if (avKf.size() != m_uDOF)
        return;
    // Clamp to [0, 12] Nm: a cfg typo must not become a torque source. 12 is
    // above the largest measured breakaway (j3 ~10.6 Nm) yet well under the
    // wrist axes' 21 Nm rated limit.
    bool bClamped = false;
    for (unsigned int i = 0; i < m_uDOF; ++i) {
        double d = avKf[i];
        if (d < 0.0 || d > 12.0) { d = (d < 0.0) ? 0.0 : 12.0; bClamped = true; }
        m_Kf[i] = d;
    }
    if (bClamped)
        DBG_LOG_WARN("(%s) Friction FF value out of [0,12] Nm - clamped",
                     "CControllerFullDynamicsRT");
    if (m_uDOF == 6)
        DBG_LOG_INFO("(%s) Friction FF [Nm]: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                     "CControllerFullDynamicsRT",
                     m_Kf[0], m_Kf[1], m_Kf[2], m_Kf[3], m_Kf[4], m_Kf[5]);
}

BOOL
CControllerFullDynamicsRT::SetFrictionFFAxis(UINT auAxis, double adKf)
{
    if (auAxis >= m_uDOF)
        return FALSE;
    m_Kf[auAxis] = adKf < 0.0 ? 0.0 : (adKf > 12.0 ? 12.0 : adKf);
    return TRUE;
}

double
CControllerFullDynamicsRT::GetFrictionFFAxis(UINT auAxis) const
{
    return (auAxis < m_uDOF) ? m_Kf[auAxis] : 0.0;
}

void
CControllerFullDynamicsRT::SetRotorInertia(const std::vector<double>& avB)
{
    if (avB.size() != m_uDOF)
        return;
    // a cfg typo must not change the dynamics by orders of magnitude: [0, 2] kg m^2
    bool bClamped = false;
    for (unsigned int i = 0; i < m_uDOF; ++i) {
        double d = avB[i];
        if (!(d >= 0.0 && d <= 2.0)) { d = (d > 2.0) ? 2.0 : 0.0; bClamped = true; }
        m_B[i] = d;
    }
    if (bClamped)
        DBG_LOG_WARN("(%s) rotor inertia out of [0, 2] kg m^2 - clamped", "CControllerFullDynamicsRT");
    if (m_uDOF == 6)
        DBG_LOG_INFO("(%s) rotor inertias B [kg m^2]: [%.5f, %.5f, %.5f, %.5f, %.5f, %.5f]",
                     "CControllerFullDynamicsRT", m_B[0], m_B[1], m_B[2], m_B[3], m_B[4], m_B[5]);
}

double
CControllerFullDynamicsRT::GetRotorInertiaAxis(UINT auAxis) const
{
    return (auAxis < m_uDOF) ? m_B[auAxis] : 0.0;
}

BOOL
CControllerFullDynamicsRT::StartSineTest(int anJoint, double adAmp, double adFreq, int anPeriods, double adV0,
                                         eJointTestLaw aeLaw, char* astrWhy, size_t auWhyLen)
{
    // URDF joint limits (indy7.urdf): J1-J5 175 deg, J6 215 deg
    static const double s_adLim[JT_MAX_DOF] = {3.0543261909900767, 3.0543261909900767,
                                               3.0543261909900767, 3.0543261909900767,
                                               3.0543261909900767, 3.7524578917878086};
    if (m_uDOF > (unsigned int)JT_MAX_DOF) {
        snprintf(astrWhy, auWhyLen, "%u joints, at most %d", m_uDOF, JT_MAX_DOF);
        return FALSE;
    }
    if (m_eControlMode != eGravityCompensation) {
        snprintf(astrWhy, auWhyLen, "start from gravity compensation ('g')");
        return FALSE;
    }
    if (anJoint < 0 || anJoint >= (int)m_uDOF) {
        snprintf(astrWhy, auWhyLen, "joint %d (SINE_TEST_JOINT is 1-based, 1..%u)", anJoint + 1, m_uDOF);
        return FALSE;
    }
    if (aeLaw != eJtCTC && aeLaw != eJtCTCFF) {
        snprintf(astrWhy, auWhyLen, "the sine test runs under CTC");
        return FALSE;
    }
    if (!(adAmp > 0.0 && adAmp <= 0.3) || !(adFreq >= 0.2 && adFreq <= 8.0) || anPeriods < 5 || anPeriods > 2000 ||
        !(adV0 >= 0.0 && adV0 <= 0.6) || (adV0 > 0.0 && anPeriods < 12)) {
        snprintf(astrWhy, auWhyLen, "amplitude %.3g rad (0..0.3), frequency %.3g Hz (0.2..8), %d periods (5..2000, "
                 "12 with a sweep), sweep %.3g rad/s (0..0.6)", adAmp, adFreq, anPeriods, adV0);
        return FALSE;
    }
    double dQdSq = 0.0;
    for (unsigned int i = 0; i < m_uDOF; ++i)
        dQdSq += m_Qd[i] * m_Qd[i];
    if (dQdSq > JT_REST_QD * JT_REST_QD) {
        snprintf(astrWhy, auWhyLen, "the arm is moving (|qd| %.3f rad/s > %.2f)", std::sqrt(dQdSq), JT_REST_QD);
        return FALSE;
    }
    // a whole number of cycles per period, so that the reference ends at q0
    const int nPer = (int)std::lround(1.0 / (adFreq * m_dt));
    const double dW = 2.0 * M_PI / (nPer * m_dt);
    const double dTf = 2 * nPer * m_dt, dTc = (anPeriods - 8) * nPer * m_dt / 2.0;
    // the sweep goes out to v0 (Tf/2 + Tc + 2 Tf/pi) and back (see UpdateJointTest)
    const double dReach = adAmp + ((adV0 > 0.0) ? adV0 * (dTf / 2.0 + dTc + 2.0 * dTf / M_PI) : 0.0);
    const double dLim = s_adLim[anJoint] - JT_LIMIT_MARGIN;
    if (m_Q[anJoint] + dReach > dLim || m_Q[anJoint] - adAmp < -dLim) {
        snprintf(astrWhy, auWhyLen, "J%d: q0 %.3f rad, reach %.3f rad beyond +-%.3f (limit - %.1f)",
                 anJoint + 1, m_Q[anJoint], dReach, dLim, JT_LIMIT_MARGIN);
        return FALSE;
    }
    const double dAccSweep = (adV0 > 0.0) ? adV0 * M_PI / dTf : 0.0;
    if (adV0 + adAmp * dW > SINE_QD_MAX + 1e-9 || adAmp * dW * dW + dAccSweep > SINE_QDD_MAX + 1e-9) {
        snprintf(astrWhy, auWhyLen, "peak %.2f rad/s, %.2f rad/s^2 (limits %.1f, %.1f)",
                 adV0 + adAmp * dW, adAmp * dW * dW + dAccSweep, SINE_QD_MAX, SINE_QDD_MAX);
        return FALSE;
    }
    for (int i = 0; i < JT_MAX_DOF; ++i) {
        m_jt.adDq[i] = 0.0;
        m_jt.adQ0[i] = 0.0;
        m_jt.adD[i]  = 0.0;
    }
    m_jt.bSine      = true;
    m_jt.nSineJoint = anJoint;
    m_jt.dSineA     = adAmp;
    m_jt.dSineW     = dW;
    m_jt.nSineFade  = 2 * nPer;
    m_jt.nSineEnd   = anPeriods * nPer;
    m_jt.dSineV0    = adV0;
    m_jt.eLaw   = aeLaw;
    m_jt.nT     = 0;
    m_jt.nW     = 0;
    m_jt.n      = 0;
    m_jt.nPhase = 0;
    m_jt.nRest  = 0;
    m_jt.bFirst = true;
    m_jt.dKfScaleSaved = m_dKfScale;
    m_dKfScale  = (aeLaw == eJtCTCFF) ? 1.0 : 0.0;
    m_jt.eState = eJtRunning;
    SetControlMode(eJointTest);
    return TRUE;
}

void 
CControllerFullDynamicsRT::SetGravity(const RigidBodyDynamics::Math::Vector3d& avGravity)
{
    m_rbdlModel.gravity = avGravity;
    DBG_LOG_INFO("(%s) Gravity set to [%.3f, %.3f, %.3f]", 
                 "CControllerFullDynamicsRT", avGravity[0], avGravity[1], avGravity[2]);
}

BOOL 
CControllerFullDynamicsRT::LoadURDF(const TSTRING& astrURDFPath)
{
    if (!RigidBodyDynamics::Addons::URDFReadFromFile(astrURDFPath.c_str(), &m_rbdlModel, false, true)) {
        DBG_LOG_ERROR("(%s) Failed to load URDF: %s", "CControllerFullDynamicsRT", astrURDFPath.c_str());
        return FALSE;
    }
    
    DBG_LOG_INFO("(%s) URDF loaded successfully: %s (DOF=%u)", 
                 "CControllerFullDynamicsRT", astrURDFPath.c_str(), m_rbdlModel.qdot_size);
    
    return TRUE;
}

BOOL 
CControllerFullDynamicsRT::InitRTOptimizations()
{
    if (m_bRTOptimized) {
        return TRUE;  // Already optimized
    }
    
    // Initialize RT memory optimizations
    if (!InitRTMemoryOptimizations()) {
        DBG_LOG_ERROR("(%s) Failed to initialize RT memory optimizations", "CControllerFullDynamicsRT");
        return FALSE;
    }
    
    // Initialize RT processor optimizations
    if (!InitRTProcessorOptimizations()) {
        DBG_LOG_ERROR("(%s) Failed to initialize RT processor optimizations", "CControllerFullDynamicsRT");
        return FALSE;
    }
    
    m_bRTOptimized = TRUE;
    DBG_LOG_INFO("(%s) RT optimizations initialized", "CControllerFullDynamicsRT");
    
    return TRUE;
}

BOOL
CControllerFullDynamicsRT::InitRTMemoryOptimizations()
{
    DBG_LOG_INFO("(%s) Prefaulting controller heap memory...", "CControllerFullDynamicsRT");
    
    // Prefault all pre-allocated RBDL vectors by touching them
    // This ensures they are in physical memory before RT loop starts
    for (unsigned int i = 0; i < m_uDOF; ++i) {
        // Touch all vector elements
        m_Q[i] = 0.0;
        m_Qd[i] = 0.0;
        m_Qdd[i] = 0.0;
        m_Q_ref[i] = 0.0;
        m_Qd_ref[i] = 0.0;
        m_Qdd_ref[i] = 0.0;
        m_zero_vector[i] = 0.0;
        
        m_h[i] = 0.0;
        m_g[i] = 0.0;
        m_tau_M[i] = 0.0;
        m_tau_feedback[i] = 0.0;
        m_tau_total[i] = 0.0;
        m_pos_error[i] = 0.0;
        m_vel_error[i] = 0.0;
        
        // Touch matrix elements (most important for RT performance)
        for (unsigned int j = 0; j < m_uDOF; ++j) {
            m_M(i, j) = 0.0;
        }
    }
    
    // Prefault std::vector containers
    std::fill(m_Kp.begin(), m_Kp.end(), 0.0);
    std::fill(m_Kd.begin(), m_Kd.end(), 0.0);
    
    // Force page faults for RBDL model data structures
    // This is crucial for RT performance
    if (m_rbdlModel.q_size > 0) {
        RigidBodyDynamics::Math::VectorNd temp_q(m_rbdlModel.q_size);
        RigidBodyDynamics::Math::VectorNd temp_qdot(m_rbdlModel.qdot_size);
        RigidBodyDynamics::Math::VectorNd temp_tau(m_rbdlModel.qdot_size);
        RigidBodyDynamics::Math::MatrixNd temp_M(m_rbdlModel.qdot_size, m_rbdlModel.qdot_size);
        
        temp_q.setZero();
        temp_qdot.setZero();
        temp_tau.setZero();
        temp_M.setZero();
        
        // Perform one dummy computation to prefault RBDL internal structures
        try {
            RigidBodyDynamics::CompositeRigidBodyAlgorithm(m_rbdlModel, temp_q, temp_M);
            RigidBodyDynamics::InverseDynamics(m_rbdlModel, temp_q, temp_qdot, temp_qdot, temp_tau);
            RigidBodyDynamics::NonlinearEffects(m_rbdlModel, temp_q, temp_qdot, temp_tau);
        } catch (...) {
            DBG_LOG_WARN("(%s) Dummy RBDL computation failed during prefault", "CControllerFullDynamicsRT");
        }
    }
    
    DBG_LOG_INFO("(%s) RT memory optimizations completed (using existing memory lock)", 
                 "CControllerFullDynamicsRT");
    return TRUE;
}

BOOL 
CControllerFullDynamicsRT::InitRTProcessorOptimizations()
{
    // 1. Eigen RT optimizations - disable malloc
    #if defined(EIGEN_RUNTIME_NO_MALLOC) && (__EIGEN_MAJOR_VERSION__>=3 && __EIGEN_MINOR_VERSION__>=4)
    Eigen::internal::set_is_malloc_allowed(false);
    DBG_LOG_INFO("(%s) Eigen malloc disabled for RT", "CControllerFullDynamicsRT");
    #endif
    
    // 2. SSE optimizations - flush-to-zero and denormals-are-zero
    #ifdef __SSE__
    _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
    _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);
    DBG_LOG_INFO("(%s) SSE optimizations enabled", "CControllerFullDynamicsRT");
    #endif
    
    return TRUE;
}

void 
CControllerFullDynamicsRT::CheckRTViolation(uint64_t computation_time_ns)
{
    if (computation_time_ns > m_rt_perf.deadline_ns) {
        m_rt_perf.rt_violations++;
        
        // Log only occasional violations to avoid RT impact
        if (m_rt_perf.rt_violations % 100 == 0) {
            DBG_LOG_WARN("(%s) RT violation #%llu: %llu us (deadline: %llu us)", 
                        "CControllerFullDynamicsRT", 
                        (unsigned long long)m_rt_perf.rt_violations,
                        (unsigned long long)(computation_time_ns / 1000),
                        (unsigned long long)(m_rt_perf.deadline_ns / 1000));
        }
    }
}
