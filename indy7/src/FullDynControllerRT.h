/*****************************************************************************
*	Name: FullDynControllerRT.h
*	Author: Raimarius Delgado
*	Affiliation: RAIMLAB @ Myongji University
*	Description: Real-time optimized full dynamics controller using RBDL
*	Copyright: RAIMLAB (2025)
*
*	The joint-space controller (paper, Sec. IV-A, eqs. (4), (5), (7) and
*	Table 7): gravity compensation with the hold, computed torque control
*	with the friction feedforward, the torque limit, and the three motions
*	TRACK, HOME, and JOINT TEST, on the RBDL model of the URDF.
*
*	Run by the Dynamics task. The high-level controller (goal admission, IK,
*	trajectory generation, lead bound) is not part of this class: the
*	Trajectory task runs it (TrajectoryRT.h) and publishes the joint
*	reference, which the Dynamics task hands to SetReference() before
*	Update() in TRACK and HOME. This class computes the torque only.
*****************************************************************************/

#ifndef G2T_FULL_DYN_CONTROLLER_RT_H
#define G2T_FULL_DYN_CONTROLLER_RT_H

#include "Controller.h"  // Include the parent class
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <rbdl/Kinematics.h>
#include <Eigen/Core>
#include <sys/mman.h>
#include <sched.h>
#include <atomic>
#include <vector>
#include <string>
#include <cstring>

#include "base/Common.h"

#define _ALL_AXIS 0xFFFFFFFF

class CControllerFullDynamicsRT : public CController
{
public:
    CControllerFullDynamicsRT(const TSTRING& astrURDFPath, unsigned int auDOF);
    virtual ~CControllerFullDynamicsRT();

    // Inherited from CController - must implement
    virtual BOOL Init() override;
    virtual BOOL Update(const std::vector<double>& avCurrentPos,
                       const std::vector<double>& avCurrentVel,
                       const std::vector<double>& avCurrentTor,
                       std::vector<double>& avOutputTorque) override;
    virtual BOOL Reset() override;
    virtual void SetGains(const std::vector<double>& avGains) override;

    void SetControlGains(const std::vector<double>& avKp, const std::vector<double>& avKd);
    // Friction feedforward K_f [N m] per joint, eq. (7) of the paper:
    // K_f sat(qd_d/0.01) gamma, applied OUTSIDE the M(q) product (friction is
    // a joint torque, not an acceleration). Values clamped to [0, 12] N m.
    void SetFrictionFF(const std::vector<double>& avKf);
    void SetGravity(const RigidBodyDynamics::Math::Vector3d& avGravity);
    void EnableRTMode(BOOL abEnable) { m_bRTMode = abEnable; }
    BOOL SetControlGain(UINT auAxis, double adKp, double adKd);
    BOOL GetControlGain(UINT auAxis, double* adKp, double* adKd);
    // per-axis friction FF: plain double store/load (RT-safe), same clamp
    BOOL   SetFrictionFFAxis(UINT auAxis, double adKf);
    double GetFrictionFFAxis(UINT auAxis) const;
    // rotor inertias B_ii [kg m^2] (paper, eq. (4)), added to the diagonal of
    // M(q) in CTC; values clamped to [0, 2] kg m^2 (Table 7)
    void   SetRotorInertia(const std::vector<double>& avB);
    double GetRotorInertiaAxis(UINT auAxis) const;
    UINT GetDOF() const { return m_uDOF; }

    // The hold of gravity compensation (Table 7: k_h, d_h, v_h, Delta_h):
    // grav-comp + a weak per-joint spring to an anchor that DRAGS when pushed
    // past the dead-band and freezes when released. Stops the residual-model
    // sink (the model cannot be perfect: link mass errors and friction) while
    // staying hand-guidable; the spring force is bounded by construction
    // (max = frac*Kp*DB per joint). Toggle with 'k' (default ON).
    static constexpr double HOLD_DB_RAD  = 0.03;   // dead-band [rad]
    static constexpr double HOLD_KP_FRAC = 0.15;   // of the joint Kp
    static constexpr double HOLD_KD_FRAC = 0.03;   // of the joint Kd (feel)
    // Per-joint velocity gate: while a joint is clearly being hand-guided
    // (|qd| above this) its anchor just follows - guiding feels like pure
    // grav-comp. Creep from model residual is far slower than this, so the
    // spring still catches sinking (an always-on spring made guiding feel
    // stiff).
    static constexpr double HOLD_UNLOCK_QD = 0.08; // [rad/s]
    std::atomic<bool> m_bStickyEnable{true};

    // Controller modes (paper, Sec. IV-C, the supervisory modes): gravity
    // compensation (with the hold) and the three motions under CTC
    enum eControlMode {
        eGravityCompensation = 0,
        eJointTest,               // 'u' : JOINT TEST, see below
        eTrack,                   // TRACK, CTC on the reference of the Trajectory task
        eHome                     // HOME to the ready posture q_r (or a joint move), the same
    };

    // eTrack and eHome are set by the Dynamics task while the Reference slot
    // carries a reference (TaskSlots.h), eGravityCompensation when it does
    // not or on a stop; eJointTest by the start of a JOINT TEST
    void SetControlMode(eControlMode aeMode)
    {
        // any change of mode ends a running JOINT TEST ('x', 'g', 'e', the
        // stops); the test itself ends in grav-comp
        if (m_eControlMode == eJointTest && aeMode != eJointTest)
            EndJointTest(eJtAborted);
        // leaving TRACK or HOME: the feedforward scale of the motion goes,
        // the reference is no longer valid
        if ((m_eControlMode == eTrack || m_eControlMode == eHome) && aeMode != eTrack && aeMode != eHome) {
            m_dKfScale = 1.0;
            m_bRefValid = FALSE;
        }
        m_eControlMode = aeMode;
    }
    eControlMode GetControlMode() const { return m_eControlMode; }

    // TRACK and HOME: the joint reference of this period, from the Trajectory
    // task (Reference slot), set before Update(). CTC then follows it (eq.
    // (5)), with the friction feedforward of eq. (7) when bFrictionFF is set
    // (the robot, or the simulation with friction). Without a valid
    // reference the two modes compute gravity compensation.
    struct Reference {
        double adQ[8], adQd[8], adQdd[8];
        BOOL   bFrictionFF;
    };
    void SetReference(const Reference& astRef);
    BOOL HasReference() const { return m_bRefValid; }
    // bit i: the torque limit acted on joint i in the last Update() (any mode)
    unsigned int GetLastClamp() const { return m_uLastClamp; }

    // JOINT TEST (paper, Sec. V-A): a prescribed joint reference, a quintic,
    // replaces the high-level controller. From the measured posture q0 at
    // rest the reference goes to q1 = q0 + dq in T, rests at q1 for the
    // dwell, returns to q0 in T, and rests at q0 for the dwell; the mode then
    // returns to grav-comp as soon as the arm is at rest (Table 9: |qd| <=
    // 0.05 rad/s for 200 ms after the reference has ended). Laws: CTC (eq.
    // (5)) without or with the friction feedforward (eq. (7)), or PD control
    // with gravity compensation, tau = g(q) + D (Kp e + Kd ed) with D = diag
    // M(q0), which has the feedback of CTC at q0 but no feedforward of qdd_d,
    // no Coriolis and centrifugal torques, and no inertial coupling. No lead
    // bound acts. Started from grav-comp only; Dynamics task only (DoInput /
    // Update).
    // eJtStep: the torque-step delay test, tau = g(q) plus a small torque
    // step on one joint, on and off with alternating sign, the reference held
    // at q0; the log shows the cycle in which the torque that the drive
    // reports (0x6077) follows the step (tools/jt_step_delay.py). No feedback
    // acts.
    // eJtRamp: the breakaway test (paper, Sec. IV-A), tau = g(q) plus a
    // torque ramp on one joint, until the joint moves (|qd| above a threshold
    // or 5 mrad from the start of the ramp) or the ramp reaches its maximum;
    // then the extra torque is removed and the joint is damped to rest, and
    // the next ramp runs in the other direction. The torque at the breakaway
    // is tau_s of Table 7, and the sign of the motion checks the sign of the
    // torque (tools/jt_breakaway.py). Ends early if the joint leaves q0 by
    // 0.1 rad.
    enum eJointTestLaw   { eJtCTC = 0, eJtCTCFF, eJtPDG, eJtStep, eJtRamp };
    enum eJointTestState { eJtIdle = 0, eJtRunning, eJtDoneAtRest, eJtDoneTimeout, eJtAborted };
    static constexpr int    JT_MAX_DOF      = 6;
    static constexpr double JT_REST_QD      = 0.05;   // [rad/s], norm of qd
    static constexpr int    JT_REST_CYC     = 200;    // 200 ms at 1 kHz
    static constexpr double JT_REST_TMAX    = 5.0;    // [s] after the last dwell, then stop anyway
    static constexpr double JT_LIMIT_MARGIN = 0.1;    // [rad] inside the joint limits (position-limit margin)
    static constexpr double JT_QD_PEAK      = 0.6;    // [rad/s] peak speed of the quintic (HL_VEL_MAX)
    static constexpr double JT_QDD_PEAK     = 2.0;    // [rad/s^2] peak acceleration (HL_ACC_MAX)
    // One cycle of the test, for the log: the reference, the measurement the
    // controller used, and the terms of the torque before the limit.
    struct JointTestSample {
        double       dT;                        // time since the start [s]
        int          nPhase;                    // 0 to q1, 1 at q1, 2 to q0, 3 at q0
        double       adQRef[JT_MAX_DOF], adQdRef[JT_MAX_DOF], adQddRef[JT_MAX_DOF];
        double       adQ[JT_MAX_DOF], adQd[JT_MAX_DOF];
        double       adE[JT_MAX_DOF];           // q_ref - q
        double       adTau[JT_MAX_DOF];         // commanded, after the limit
        double       adTauG[JT_MAX_DOF];        // g(q)
        double       adTauC[JT_MAX_DOF];        // C(q,qd) qd = h - g (CTC)
        double       adTauMqdd[JT_MAX_DOF];     // M(q) qdd_ref (CTC)
        double       adTauFb[JT_MAX_DOF];       // M(q)(Kp e + Kd ed), or D(Kp e + Kd ed)
        double       adTauFf[JT_MAX_DOF];       // friction feedforward, or the torque step (eJtStep)
        double       adMii[JT_MAX_DOF];         // M_ii(q) of the URDF, without B (CTC)
        double       adMRow[JT_MAX_DOF];        // sine test: row j of M(q) of the URDF (tested joint j)
        unsigned int uClamp;                    // bit i: the torque limit acted on joint i
        // filled by the Dynamics task from the drives, not by the controller:
        // the torque they report in this period (N m and drive units, 0x6077)
        // and the target written in this period (units, 0x6071), from the
        // State slot of the period: the target written in it is the torque of
        // the previous cycle of this task.
        double       adTauAct[JT_MAX_DOF];
        int          anTauActRaw[JT_MAX_DOF];
        int          anTauCmdRaw[JT_MAX_DOF];
    };
    // Checks the start conditions (grav-comp, arm at rest, q0 and q1 inside
    // the limits, peak speed and acceleration of the quintic within the joint
    // limits of the high-level controller) and starts; FALSE with the reason.
    BOOL StartJointTest(const double* adDq, double adT, double adDwell, eJointTestLaw aeLaw,
                        char* astrWhy, size_t auWhyLen);
    // the torque-step delay test: joint anJoint (0-based), step
    // adTau [N m], on adOnS and off adOffS [s], anRep steps with alternating
    // sign; refused unless in grav-comp at rest, |adTau| <= TS_MAX_NM and
    // <= a quarter of the torque limit of the joint
    static constexpr double TS_MAX_NM       = 3.0;
    BOOL StartTorqueStep(int anJoint, double adTau, double adOnS, double adOffS, int anRep,
                         char* astrWhy, size_t auWhyLen);
    // the breakaway test: joint anJoint (0-based), ramp adRate
    // [N m/s] up to adMax [N m] (at most 0.8 of the torque limit), motion
    // threshold adQd [rad/s], anRep ramps in each direction
    static constexpr double BA_MOVE_RAD     = 0.005;  // [rad] from the start of a ramp: moved
    static constexpr double BA_ABORT_RAD    = 0.1;    // [rad] from q0: the test ends
    BOOL StartBreakaway(int anJoint, double adRate, double adMax, double adQd, int anRep,
                        char* astrWhy, size_t auWhyLen);
    // the sine test (identification of the rotor inertias, paper, Sec. IV-A,
    // eq. (6)): joint anJoint (0-based) follows
    // q0 + A sin(2 pi f t), faded in and out over two periods, anPeriods
    // periods in all, while the other joints are held at q0, under CTC
    // (aeLaw eJtCTC or eJtCTCFF); refused unless in grav-comp at rest, with
    // q0 +- A inside the joint limits, A f 2 pi <= SINE_QD_MAX and
    // A (2 pi f)^2 <= SINE_QDD_MAX
    static constexpr double SINE_QD_MAX     = 1.0;    // [rad/s]
    static constexpr double SINE_QDD_MAX    = 12.0;   // [rad/s^2]
    // adV0 > 0: the sine rides on a sweep at +-adV0 [rad/s], out and back (up
    // over two periods, cruise, reversal over four, cruise, down over two),
    // so that the joint keeps sliding and friction changes sign only at the
    // reversal (A 2 pi f < adV0: the velocity does not cross zero while it
    // cruises); adV0 = 0: the sine alone, around q0
    BOOL StartSineTest(int anJoint, double adAmp, double adFreq, int anPeriods, double adV0, eJointTestLaw aeLaw,
                       char* astrWhy, size_t auWhyLen);
    eJointTestState GetJointTestState() const { return m_jt.eState; }
    BOOL  IsJointTestRunning() const { return m_jt.eState == eJtRunning; }
    // TRUE if the last Update() was a cycle of the test (its sample is valid)
    BOOL  HasJointTestSample() const { return m_bJtSample; }
    const JointTestSample& GetJointTestSample() const { return m_jtSample; }
    const double* GetJointTestQ0() const { return m_jt.adQ0; }
    const double* GetJointTestD() const { return m_jt.adD; }

    // torque limit per joint [N m], tau_max of Table 7: the cfg TORQUE_LIMIT_U
    // of each joint drive, never above the URDF effort limit, which stays the
    // limit where the cfg has none. Applied in every mode.
    void SetTorqueLimits(const std::vector<double>& avTauMax);
    double GetTorqueLimit(UINT auAxis) const { return auAxis < 8 ? m_adTauMax[auAxis] : 0.0; }
    // the ablation of the gate: the friction feedforward without it (cfg FRICTION_FF_GATE=0)
    void SetFrictionFFGate(bool abOn) { m_bKfGate = abOn; }
    bool GetFrictionFFGate() const { return m_bKfGate; }

    // Performance monitoring
    struct RTPerformance {
        uint64_t max_compute_time_ns;
        uint64_t avg_compute_time_ns;
        uint64_t total_compute_time_ns;
        uint64_t cycle_count;
        uint64_t rt_violations;
        uint64_t deadline_ns;

        void Reset() {
            max_compute_time_ns = 0;
            avg_compute_time_ns = 0;
            total_compute_time_ns = 0;
            cycle_count = 0;
            rt_violations = 0;
        }
    };

    RTPerformance GetPerformance() const { return m_rt_perf; }
    void ResetPerformance() { m_rt_perf.Reset(); }
    void SetDeadline(uint64_t deadline_ns) { m_rt_perf.deadline_ns = deadline_ns; }

    // time of the RBDL calls of the control law (g, M, h) in the last
    // Update() [ns], for the timing log (TimingLog.h)
    uint64_t GetDynamicsNs() const { return m_uDynNs; }

    // Update() fails (FALSE, all torques 0) on a non-finite
    // measured angle or velocity, checked before the law, and on a
    // non-finite torque, checked after it (the backstop); the application
    // latches a controller fault on it (TaskDynamics.cpp). Fault injection
    // for the test of that path, simulation only (cfg SIM_FAULT_*, refused
    // on the robot by the application): the torque of joint anJoint
    // (0-based) becomes NaN after the law and before the backstop, in the
    // next Update() (abPersist FALSE) or in every one until
    // ClearFaultInjection(). Dynamics task only, like Update().
    void InjectTorqueFault(int anJoint, BOOL abPersist)
    {
        m_nInjectJoint = (anJoint >= 0 && anJoint < (int)m_uDOF) ? anJoint : -1;
        m_bInjectPersist = abPersist;
    }
    void ClearFaultInjection() { m_nInjectJoint = -1; m_bInjectPersist = FALSE; }

    // RT optimization control
    BOOL InitRTOptimizations();

    // Pose of the tool (TCP body "tcp"): position and rotation body -> base
    struct Pose
    {
        RigidBodyDynamics::Math::Vector3d m_position;
        RigidBodyDynamics::Math::Matrix3d m_rotation;

        Pose()
        {
            m_position.setZero();
            m_rotation.setIdentity();
        }
    };
    RigidBodyDynamics::Math::Vector3d tcp_local_point{0.0, 0.0, 0.0};

    // Pose of the tool at the last measured state (hand-eye capture, 's').
    // Updates the kinematics of the model (allocates in RBDL): not for Update().
    BOOL GetCurrentPose(Pose& astCurrPose);
    // Tool attitude (body->base, the Pose::m_rotation convention) and tool
    // POSITION at an arbitrary joint vector, with the model and the TCP the
    // robot runs. Offline tools only (tools/model_check.cpp): allocates.
    RigidBodyDynamics::Math::Matrix3d ToolRotAt(
        const RigidBodyDynamics::Math::VectorNd& aq);
    RigidBodyDynamics::Math::Vector3d ToolPosAt(
        const RigidBodyDynamics::Math::VectorNd& aq);

    // M(q) and h(q, qd) = C(q, qd) qd + g(q) at an arbitrary
    // state, computed exactly as the CTC law computes them (same model
    // object, CompositeRigidBodyAlgorithm and NonlinearEffects). For the
    // offline model check against MuJoCo (tools/model_check.cpp), same
    // principle as ToolPosAt. Not for the RT loop (allocates).
    void ModelTermsAt(const RigidBodyDynamics::Math::VectorNd& aq,
                      const RigidBodyDynamics::Math::VectorNd& aqd,
                      RigidBodyDynamics::Math::MatrixNd& aM,
                      RigidBodyDynamics::Math::VectorNd& ah);

    static constexpr double m_dt = 0.001;        // 1kHz -> 1ms

private:
    // RBDL model and dynamics
    RigidBodyDynamics::Model m_rbdlModel;
    TSTRING m_strURDFPath;
    eControlMode m_eControlMode;
    unsigned int m_body_id;                             // body "tcp"

    // Pre-allocated RBDL vectors (RT-safe)
    RigidBodyDynamics::Math::VectorNd m_Q;              // Current position
    RigidBodyDynamics::Math::VectorNd m_Qd;             // Current velocity
    RigidBodyDynamics::Math::VectorNd m_Qdd;            // Desired acceleration (CTC)
    RigidBodyDynamics::Math::VectorNd m_Q_ref;          // Reference position
    RigidBodyDynamics::Math::VectorNd m_Qd_ref;         // Reference velocity
    RigidBodyDynamics::Math::VectorNd m_Qdd_ref;        // Reference acceleration
    RigidBodyDynamics::Math::VectorNd m_zero_vector;    // Zero vector

    // Dynamics components
    RigidBodyDynamics::Math::MatrixNd m_M;              // Inertia matrix M(q)
    RigidBodyDynamics::Math::VectorNd m_h;              // Nonlinear effects h(q,qd)
    RigidBodyDynamics::Math::VectorNd m_g;              // Gravity vector g(q)

    // Control terms
    RigidBodyDynamics::Math::VectorNd m_tau_M;          // M(q) * qdd_ref
    RigidBodyDynamics::Math::VectorNd m_tau_feedback;   // Feedback control
    RigidBodyDynamics::Math::VectorNd m_tau_total;      // Total control torque

    // Error vectors
    RigidBodyDynamics::Math::VectorNd m_pos_error;      // Position error
    RigidBodyDynamics::Math::VectorNd m_vel_error;      // Velocity error

    // Control gains
    std::vector<double> m_Kp;                           // Position gains
    std::vector<double> m_Kd;                           // Velocity gains
    std::vector<double> m_Kf;                           // Friction FF [Nm], 0 = off
    std::vector<double> m_B;                            // rotor inertias [kg m^2], 0 = none
    double m_dKfScale{1.0};                             // friction FF on (1) or off (0) for the running motion
    bool   m_bKfGate{true};                             // the gate of eq. (7); false: the ablation without it

    // sticky-float anchor (grav-comp only; reset on mode change)
    RigidBodyDynamics::Math::VectorNd m_Q_hold;
    bool m_bHoldAnchored{false};

    // JOINT TEST state (see the public part)
    struct JointTest {
        eJointTestState eState;
        eJointTestLaw   eLaw;
        double adDq[JT_MAX_DOF];
        double adQ0[JT_MAX_DOF];      // measured at the first cycle
        double adD[JT_MAX_DOF];       // diag M(q0), for PD + g
        int    nT, nW;                // leg and dwell [cycles]
        int    n;                     // cycles done
        int    nPhase;
        int    nRest;                 // consecutive cycles at rest after the reference ended
        bool   bFirst;
        double dKfScaleSaved;         // friction-FF scale outside the test
        // eJtStep
        int    nStepJoint;            // 0-based
        double dStepTau;              // [N m]
        int    nOn, nOff, nRep;       // [cycles], steps
        int    nStepEnd;              // nRep (nOn + nOff)
        double dStepNow;              // the step of this cycle [N m]
        // eJtRamp (nStepJoint, dStepNow, nRep shared)
        double dRampRate, dRampMax, dRampQd;
        int    nRampDone;             // ramps finished
        int    nRampCyc;              // cycles in the current phase
        int    nRampStill;            // consecutive cycles at rest (phase 1)
        double dRampQ0;               // joint angle at the start of the ramp
        // sine test (eJtCTC or eJtCTCFF with bSine)
        bool   bSine;
        int    nSineJoint;            // 0-based
        double dSineA, dSineW;        // [rad], [rad/s]
        int    nSineFade, nSineEnd;   // [cycles]
        double dSineV0;               // [rad/s] sweep velocity, 0: none
    };
    JointTest       m_jt{};
    JointTestSample m_jtSample{};
    BOOL            m_bJtSample{FALSE};
    RigidBodyDynamics::Math::VectorNd m_vJtMqdd;    // M(q) qdd_ref, preallocated
    void UpdateJointTest();
    void FillJointTestTerms();
    void EndJointTest(eJointTestState aeState);
    BOOL ComputePDGravity(std::vector<double>& avOutputTorque);
    BOOL ComputeGravityStep(std::vector<double>& avOutputTorque);

    // torque limit per joint (SetTorqueLimits)
    double m_adTauMax[8];

    // the reference of the Trajectory task (SetReference)
    Reference        m_ref{};
    BOOL             m_bRefValid{FALSE};
    unsigned int     m_uLastClamp{0};
    BOOL ComputeReferenceTracking(std::vector<double>& avOutputTorque);

    // fault injection (InjectTorqueFault): joint, -1 none
    int              m_nInjectJoint{-1};
    BOOL             m_bInjectPersist{FALSE};

    // RT performance monitoring
    RTPerformance m_rt_perf;
    BOOL m_bRTMode;
    BOOL m_bRTOptimized;
    uint64_t m_uDynNs{0};       // see GetDynamicsNs()

    // RT utility methods
    BOOL LoadURDF(const TSTRING& astrURDFPath);
    BOOL InitRTMemoryOptimizations();
    BOOL InitRTProcessorOptimizations();
    void CheckRTViolation(uint64_t computation_time_ns);

    // Control computation methods
    BOOL ComputeGravityCompensation(std::vector<double>& avOutputTorque);
    BOOL ComputeComputedTorque(std::vector<double>& avOutputTorque);
};

#endif // G2T_FULL_DYN_CONTROLLER_RT_H
