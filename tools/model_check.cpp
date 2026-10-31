/*****************************************************************************
*	Name: model_check.cpp
*	Description: Kinematic and dynamic check of the simulation model (paper,
*	Sec. IV-A, eq. (4)): the RBDL model of the controller against the MuJoCo
*	model converted from the same URDF (tools/urdf2mjcf.py), over random
*	joint states.
*
*	kinematic check   TCP position (and orientation) of the controller,
*	                  ToolPosAt/ToolRotAt, vs the "tcp" body of MuJoCo.
*	                  A difference beyond numerical precision reveals a wrong
*	                  joint order, tool body, or tool offset.
*	dynamic check     M(q) + B of the controller vs the M of MuJoCo, and h(q, qd) vs
*	                  qfrc_bias, relative to the MuJoCo value; h is also
*	                  split into gravity, h(q, 0), and the velocity part,
*	                  h(q, qd) - h(q, 0), so that the large gravity term
*	                  cannot hide an error in the Coriolis terms.
*
*	The controller side is the model object of the controller
*	(FullDynControllerRT.cpp: ToolPosAt, ToolRotAt, ModelTermsAt), with the
*	TCP definition the robot runs. Joints are matched by name (joint0..5).
*
*	    bin/model_check models/indy7.urdf models/indy7_mujoco.xml
*	                    [samples=10000] [seed=1] [out.csv] [B=b1,...,b6]
*	B= the rotor inertias [kg m^2] (ROTOR_INERTIA_n): added to M(q) of the
*	controller and set as the armature of the MuJoCo model, as in the
*	simulation. q uniform within the joint ranges of the MJCF (= the URDF
*	limits), qd uniform in [-pi, pi] rad/s.
*	Exit code 0 when every metric is below its limit (KIN_LIMIT_M,
*	KIN_LIMIT_RAD, DYN_LIMIT_REL below), 1 otherwise.
*	(make MUJOCO=1 tools)
*****************************************************************************/
#include "FullDynControllerRT.h"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include <vector>

using RigidBodyDynamics::Math::Matrix3d;
using RigidBodyDynamics::Math::MatrixNd;
using RigidBodyDynamics::Math::Vector3d;
using RigidBodyDynamics::Math::VectorNd;

namespace {

const int    DOF           = 6;
/* Limits. Floor with the Indy7 URDF (10^4 states): 3.4e-10 m, 5.8e-10 rad,
 * at most 1.4e-8 relative in M, h, g, c. It comes entirely from RBDL's URDF
 * reader, which recovers roll, pitch, and yaw from a quaternion with asin();
 * next to pitch = pi/2 (joint origins of joint1 and joint4) this keeps half
 * of the digits. Given the angles RBDL recovers, MuJoCo agrees to <= 1e-13.
 * Smallest faults seen (deliberately broken models): MJCF numbers rounded to
 * 6 digits, ~1e-6 relative; tool offset 1 mm, 1e-3 m; mass 1 %, 2e-3. */
const double KIN_LIMIT_M   = 1e-8;    // [m]   TCP position
const double KIN_LIMIT_RAD = 1e-8;    // [rad] TCP orientation
const double DYN_LIMIT_REL = 1e-7;    // relative difference of M, h, g, and c

struct Stat
{
    const char* pcName;
    double dLimit;
    std::vector<double> vValue;
    double Max() const { return vValue.empty() ? 0.0 : *std::max_element(vValue.begin(), vValue.end()); }
    double Median() const
    {
        if (vValue.empty()) return 0.0;
        std::vector<double> v = vValue;
        std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
        return v[v.size() / 2];
    }
    size_t ArgMax() const { return std::max_element(vValue.begin(), vValue.end()) - vValue.begin(); }
};

double RelDiff(double adNumNorm, double adRefNorm)
{
    return adNumNorm / std::max(adRefNorm, 1e-300);
}

} // namespace

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        fprintf(stderr, "usage: %s <urdf> <mjcf> [samples=10000] [seed=1] [out.csv] [B=b1,...,b6]\n", argv[0]);
        return 2;
    }
    // B=b1,...,b6 anywhere after the MJCF; the other arguments keep their positions
    double adB[DOF] = {0.0};
    {
        int nArgs = 0;
        for (int k = 1; k < argc; k++)
        {
            if (strncmp(argv[k], "B=", 2) == 0)
            {
                const char* pc = argv[k] + 2;
                for (int i = 0; i < DOF && *pc; i++)
                {
                    char* pcEnd = NULL;
                    adB[i] = strtod(pc, &pcEnd);
                    if (pcEnd == pc) break;
                    pc = (*pcEnd == ',') ? pcEnd + 1 : pcEnd;
                }
                continue;
            }
            argv[++nArgs] = argv[k];
        }
        argc = nArgs + 1;
    }
    const std::string strURDF = argv[1];
    const std::string strMJCF = argv[2];
    const long nSamples = (argc > 3) ? atol(argv[3]) : 10000;
    const unsigned long uSeed = (argc > 4) ? strtoul(argv[4], NULL, 10) : 1;
    FILE* pCSV = (argc > 5) ? fopen(argv[5], "w") : NULL;
    if (nSamples <= 0)
    {
        fprintf(stderr, "model_check: samples must be positive\n");
        return 2;
    }

    /* controller side: the robot's own model and TCP definition */
    CControllerFullDynamicsRT cCtrl(strURDF, DOF);
    if (!cCtrl.Init())
    {
        fprintf(stderr, "model_check: controller init failed (URDF %s)\n", strURDF.c_str());
        return 1;
    }

    /* MuJoCo side */
    char acErr[1024] = {0};
    mjModel* m = mj_loadXML(strMJCF.c_str(), NULL, acErr, sizeof(acErr));
    if (m == NULL)
    {
        fprintf(stderr, "model_check: cannot load %s: %s\n", strMJCF.c_str(), acErr);
        return 1;
    }
    mjData* d = mj_makeData(m);
    if (m->nv != DOF || m->nq != DOF)
    {
        fprintf(stderr, "model_check: MJCF has nq=%d nv=%d, expected %d\n", (int)m->nq, (int)m->nv, DOF);
        return 1;
    }
    int anQ[DOF], anV[DOF];
    double adLo[DOF], adHi[DOF];
    for (int i = 0; i < DOF; i++)
    {
        const std::string strJ = "joint" + std::to_string(i);
        const int nJ = mj_name2id(m, mjOBJ_JOINT, strJ.c_str());
        if (nJ < 0 || !m->jnt_limited[nJ])
        {
            fprintf(stderr, "model_check: joint %s missing or without range in the MJCF\n", strJ.c_str());
            return 1;
        }
        anQ[i] = m->jnt_qposadr[nJ];
        anV[i] = m->jnt_dofadr[nJ];
        adLo[i] = m->jnt_range[2 * nJ];
        adHi[i] = m->jnt_range[2 * nJ + 1];
    }
    for (int i = 0; i < DOF; i++)                             // B as the armature, as in the simulation
        m->dof_armature[anV[i]] = adB[i];
    mj_setConst(m, d);
    printf("model_check: rotor inertias B [kg m^2]: %g %g %g %g %g %g\n", adB[0], adB[1], adB[2], adB[3], adB[4], adB[5]);
    const int nTcp = mj_name2id(m, mjOBJ_BODY, "tcp");
    if (nTcp < 0)
    {
        fprintf(stderr, "model_check: body \"tcp\" missing in the MJCF\n");
        return 1;
    }

    Stat stPos  = {"TCP position [m]",           KIN_LIMIT_M,   {}};
    Stat stRot  = {"TCP orientation [rad]",      KIN_LIMIT_RAD, {}};
    Stat stM    = {"M(q) relative",              DYN_LIMIT_REL, {}};
    Stat stH    = {"h(q,qd) relative",           DYN_LIMIT_REL, {}};
    Stat stG    = {"gravity h(q,0) relative",    DYN_LIMIT_REL, {}};
    Stat stC    = {"velocity part relative",     DYN_LIMIT_REL, {}};
    Stat* apStat[] = {&stPos, &stRot, &stM, &stH, &stG, &stC};
    double dMaxAbsM = 0.0, dMaxAbsH = 0.0;

    std::mt19937_64 cRng(uSeed);
    std::vector<VectorNd> vQ;
    VectorNd q(DOF), qd(DOF), qZero = VectorNd::Zero(DOF);
    MatrixNd Mr, Mm(DOF, DOF), Mtmp;
    VectorNd hr, gr, hm(DOF), gm(DOF);
    std::vector<mjtNum> vFull(DOF * DOF), vE(DOF, 0.0), vCol(DOF);

    if (pCSV)
        fprintf(pCSV, "i,q0,q1,q2,q3,q4,q5,qd0,qd1,qd2,qd3,qd4,qd5,"
                      "e_pos_m,e_rot_rad,e_M_rel,e_h_rel,e_g_rel,e_c_rel\n");

    for (long n = 0; n < nSamples; n++)
    {
        for (int i = 0; i < DOF; i++)
        {
            q[i]  = std::uniform_real_distribution<double>(adLo[i], adHi[i])(cRng);
            qd[i] = std::uniform_real_distribution<double>(-M_PI, M_PI)(cRng);
        }
        vQ.push_back(q);

        /* controller */
        const Vector3d pr = cCtrl.ToolPosAt(q);
        const Matrix3d Rr = cCtrl.ToolRotAt(q);            // body -> base
        cCtrl.ModelTermsAt(q, qd, Mr, hr);
        cCtrl.ModelTermsAt(q, qZero, Mtmp, gr);
        for (int i = 0; i < DOF; i++)                         // M + B, eq. (4)
            Mr(i, i) += adB[i];

        /* MuJoCo, moving state: positions, M, bias forces */
        for (int i = 0; i < DOF; i++) { d->qpos[anQ[i]] = q[i]; d->qvel[anV[i]] = qd[i]; }
        mj_forward(m, d);
        for (int j = 0; j < m->nv; j++)                      // M(q) column by column (mj_mulM has
        {                                                    // the same signature in every MuJoCo 3)
            vE[j] = 1.0;
            mj_mulM(m, d, vCol.data(), vE.data());
            vE[j] = 0.0;
            for (int i = 0; i < m->nv; i++)
                vFull[i * m->nv + j] = vCol[i];
        }
        Vector3d pm(d->xpos[3 * nTcp], d->xpos[3 * nTcp + 1], d->xpos[3 * nTcp + 2]);
        Matrix3d Rm;                                         // body -> world (xmat is row-major)
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                Rm(r, c) = d->xmat[9 * nTcp + 3 * r + c];
        for (int i = 0; i < DOF; i++)
        {
            hm[i] = d->qfrc_bias[anV[i]];
            for (int j = 0; j < DOF; j++)
                Mm(i, j) = vFull[anV[i] * m->nv + anV[j]];
        }
        /* MuJoCo at rest: gravity */
        for (int i = 0; i < DOF; i++) d->qvel[anV[i]] = 0.0;
        mj_forward(m, d);
        for (int i = 0; i < DOF; i++) gm[i] = d->qfrc_bias[anV[i]];

        /* differences */
        /* rotation angle of Rr^T Rm: atan2(sin, cos) stays accurate for tiny
         * angles, where acos(cos) cannot resolve less than ~1.5e-8 rad */
        const Matrix3d dR = Rr.transpose() * Rm;
        const Vector3d w(dR(2, 1) - dR(1, 2), dR(0, 2) - dR(2, 0), dR(1, 0) - dR(0, 1));
        stPos.vValue.push_back((pr - pm).norm());
        stRot.vValue.push_back(std::atan2(0.5 * w.norm(), 0.5 * (dR.trace() - 1.0)));
        stM.vValue.push_back(RelDiff((Mr - Mm).norm(), Mm.norm()));
        stH.vValue.push_back(RelDiff((hr - hm).norm(), hm.norm()));
        stG.vValue.push_back(RelDiff((gr - gm).norm(), gm.norm()));
        stC.vValue.push_back(RelDiff(((hr - gr) - (hm - gm)).norm(), (hm - gm).norm()));
        dMaxAbsM = std::max(dMaxAbsM, (Mr - Mm).cwiseAbs().maxCoeff());
        dMaxAbsH = std::max(dMaxAbsH, (hr - hm).cwiseAbs().maxCoeff());

        if (pCSV)
        {
            fprintf(pCSV, "%ld", n);
            for (int i = 0; i < DOF; i++) fprintf(pCSV, ",%.17g", q[i]);
            for (int i = 0; i < DOF; i++) fprintf(pCSV, ",%.17g", qd[i]);
            for (Stat* p : apStat) fprintf(pCSV, ",%.3e", p->vValue.back());
            fprintf(pCSV, "\n");
        }
    }
    if (pCSV) fclose(pCSV);

    const int nRbdl = rbdl_get_api_version();
    printf("\nmodel_check: %ld random states (seed %lu)\n  URDF %s (RBDL %d.%d.%d)\n  MJCF %s (MuJoCo %s)\n",
           nSamples, uSeed, strURDF.c_str(), (nRbdl >> 16) & 0xff, (nRbdl >> 8) & 0xff, nRbdl & 0xff,
           strMJCF.c_str(), mj_versionString());
    printf("  %-26s %12s %12s %12s  %s\n", "", "median", "max", "limit", "worst state q [rad]");
    bool bPass = true;
    for (Stat* p : apStat)
    {
        const bool bOk = p->Max() <= p->dLimit;
        bPass = bPass && bOk;
        const VectorNd& qw = vQ[p->ArgMax()];
        printf("  %-26s %12.3e %12.3e %12.0e  [%.3f %.3f %.3f %.3f %.3f %.3f] %s\n",
               p->pcName, p->Median(), p->Max(), p->dLimit,
               qw[0], qw[1], qw[2], qw[3], qw[4], qw[5], bOk ? "ok" : "FAIL");
    }
    printf("  largest absolute difference: M %.3e kg m^2, h %.3e N m\n", dMaxAbsM, dMaxAbsH);
    printf("model_check: %s\n", bPass ? "PASS" : "FAIL");

    mj_deleteData(d);
    mj_deleteModel(m);
    return bPass ? 0 : 1;
}
