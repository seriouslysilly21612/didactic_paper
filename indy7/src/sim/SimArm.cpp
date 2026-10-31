/*****************************************************************************
*	Name: SimArm.cpp
*	Description: MuJoCo in place of the EtherCAT exchange. See SimArm.h.
*****************************************************************************/
#include "SimArm.h"
#include "../base/Common.h"

#include <cmath>
#include <cstdio>
#include <cstring>

#ifdef G2T_WITH_MUJOCO
#include <mujoco/mujoco.h>

/* MuJoCo warnings are counted, not printed, while the task runs */
static volatile int s_nMjWarnings = 0;
static char s_acMjWarning[256] = {0};
static void SimArmWarning(const char* astrMsg)
{
	s_nMjWarnings = s_nMjWarnings + 1;
	strncpy(s_acMjWarning, astrMsg, sizeof(s_acMjWarning) - 1);
}
#endif

namespace {
inline int Slot(long anIdx) { return (int)(((anIdx % (CSimArm::MAX_DELAY + 1)) + (CSimArm::MAX_DELAY + 1)) % (CSimArm::MAX_DELAY + 1)); }
}

CSimArm::CSimArm()
	: m_pM(NULL), m_pD(NULL), m_bInit(FALSE), m_nDof(0), m_nStateDelay(0), m_nTorqueDelay(0),
	  m_nStep(0), m_uStepNsMax(0), m_uStepNsSum(0), m_nStepsBraked(0), m_dMaxDisp(0.0), m_bFriction(FALSE)
{
	memset(m_anQ, 0, sizeof(m_anQ));
	memset(m_anV, 0, sizeof(m_anV));
	memset(m_adHistQ, 0, sizeof(m_adHistQ));
	memset(m_adHistQd, 0, sizeof(m_adHistQd));
	memset(m_adHistTau, 0, sizeof(m_adHistTau));
	memset(m_adTauQueue, 0, sizeof(m_adTauQueue));
	memset(m_abBraked, 0, sizeof(m_abBraked));
	memset(m_adBrakeQ, 0, sizeof(m_adBrakeQ));
	memset(m_adQRelease, 0, sizeof(m_adQRelease));
}

CSimArm::~CSimArm()
{
#ifdef G2T_WITH_MUJOCO
	if (m_pD) mj_deleteData((mjData*)m_pD);
	if (m_pM) mj_deleteModel((mjModel*)m_pM);
#endif
}

BOOL
CSimArm::Init(const char* astrMJCF, int anDof, int anStateDelay, int anTorqueDelay,
              const double* adQ0, BOOL abFriction, const double* adTauS, double adPeriod)
{
#ifdef G2T_WITH_MUJOCO
	if (anDof <= 0 || anDof > MAX_DOF)
	{
		DBG_LOG_ERROR("(CSimArm) %d joints, at most %d supported", anDof, (int)MAX_DOF);
		return FALSE;
	}
	if (anStateDelay < 0 || anTorqueDelay < anStateDelay || anTorqueDelay > MAX_DELAY)
	{
		DBG_LOG_ERROR("(CSimArm) delays: state %d, torque %d steps; need 0 <= state <= torque <= %d",
		              anStateDelay, anTorqueDelay, (int)MAX_DELAY);
		return FALSE;
	}
	char acErr[1024] = {0};
	mjModel* m = mj_loadXML(astrMJCF, NULL, acErr, sizeof(acErr));
	if (m == NULL)
	{
		DBG_LOG_ERROR("(CSimArm) cannot load %s: %s", astrMJCF, acErr);
		return FALSE;
	}
	if (std::fabs(m->opt.timestep - adPeriod) > 1e-12)
	{
		DBG_LOG_WARN("(CSimArm) timestep %g s of the model set to the control period %g s",
		             m->opt.timestep, adPeriod);
		m->opt.timestep = adPeriod;
	}
	mjData* d = mj_makeData(m);
	for (int i = 0; i < anDof; i++)
	{
		char acName[16];
		snprintf(acName, sizeof(acName), "joint%d", i);
		const int nJ = mj_name2id(m, mjOBJ_JOINT, acName);
		if (nJ < 0 || (m->jnt_type[nJ] != mjJNT_HINGE && m->jnt_type[nJ] != mjJNT_SLIDE))
		{
			DBG_LOG_ERROR("(CSimArm) %s: no hinge or slide joint \"%s\"", astrMJCF, acName);
			mj_deleteData(d);
			mj_deleteModel(m);
			return FALSE;
		}
		m_anQ[i] = m->jnt_qposadr[nJ];
		m_anV[i] = m->jnt_dofadr[nJ];
		m->dof_frictionloss[m_anV[i]] = abFriction ? adTauS[i] : 0.0;
		d->qpos[m_anQ[i]] = adQ0[i];
	}
	mju_zero(d->qvel, m->nv);
	mj_forward(m, d);
	mju_user_warning = SimArmWarning;

	m_pM = (struct mjModel_*)m;
	m_pD = (struct mjData_*)d;
	m_nDof = anDof;
	m_nStateDelay = anStateDelay;
	m_nTorqueDelay = anTorqueDelay;
	m_bFriction = abFriction;
	m_nStep = 0;
	for (int s = 0; s <= MAX_DELAY; s++)
		for (int i = 0; i < anDof; i++)
		{
			m_adHistQ[s][i] = adQ0[i];
			m_adHistQd[s][i] = 0.0;
			m_adHistTau[s][i] = 0.0;
			m_adTauQueue[s][i] = 0.0;
		}
	for (int i = 0; i < anDof; i++)
	{
		m_abBraked[i] = TRUE;			// servo off at start: brakes engaged
		m_adBrakeQ[i] = adQ0[i];
		m_adQRelease[i] = adQ0[i];
	}
	m_bInit = TRUE;
	static const char* const s_astrIntegrator[] = { "Euler", "RK4", "implicit", "implicitfast" };
	const int nInt = m->opt.integrator;
	DBG_LOG_INFO("(CSimArm) %s: %d joints, step %g s (%s), state %d steps old, torque %d steps after it, friction %s",
	             astrMJCF, anDof, m->opt.timestep, (nInt >= 0 && nInt < 4) ? s_astrIntegrator[nInt] : "?",
	             anStateDelay, anTorqueDelay, abFriction ? "on" : "off");
	if (nInt != mjINT_RK4)
		DBG_LOG_WARN("(CSimArm) integrator is not RK4: at a step of 1 ms, the tracking error includes an error "
		             "of the integrator (regenerate the model with tools/urdf2mjcf.py)");
	return TRUE;
#else
	(void)astrMJCF; (void)anDof; (void)anStateDelay; (void)anTorqueDelay;
	(void)adQ0; (void)abFriction; (void)adTauS; (void)adPeriod;
	DBG_LOG_ERROR("(CSimArm) built without MuJoCo (make MUJOCO=1 with MUJOCO_DIR)");
	return FALSE;
#endif
}

BOOL
CSimArm::SetArmature(const double* adB, int anDof)
{
#ifdef G2T_WITH_MUJOCO
	if (m_pM == NULL || anDof != m_nDof)
		return FALSE;
	mjModel* m = (mjModel*)m_pM;
	mjData* d = (mjData*)m_pD;
	for (int i = 0; i < anDof; i++)
		m->dof_armature[m_anV[i]] = (adB[i] > 0.0) ? adB[i] : 0.0;
	mj_setConst(m, d);		// the derived constants that depend on the armature
	mj_forward(m, d);
	return TRUE;
#else
	(void)adB; (void)anDof;
	return FALSE;
#endif
}

void
CSimArm::GetMeasured(double* adQ, double* adQd, double* adTau) const
{
	const int s = Slot(m_nStep - m_nStateDelay);
	for (int i = 0; i < m_nDof; i++)
	{
		adQ[i] = m_adHistQ[s][i];
		adQd[i] = m_adHistQd[s][i];
		adTau[i] = m_adHistTau[s][i];
	}
}

void
CSimArm::GetCurrentQ(double* adQ) const
{
	const int s = Slot(m_nStep);
	for (int i = 0; i < m_nDof; i++)
		adQ[i] = m_adHistQ[s][i];
}

void
CSimArm::Step(const double* adTau, const BOOL* abReleased)
{
#ifdef G2T_WITH_MUJOCO
	if (!m_bInit) return;
	const RTTIME tStart = read_timer();
	mjModel* m = (mjModel*)m_pM;
	mjData* d = (mjData*)m_pD;
	const long k = m_nStep;

	/* torque of this cycle; with D = a it acts during this step */
	const int sIn = Slot(k);
	for (int i = 0; i < m_nDof; i++)
		m_adTauQueue[sIn][i] = adTau[i];
	const int sOut = Slot(k - (m_nTorqueDelay - m_nStateDelay));

	BOOL bAllBraked = TRUE;
	BOOL bReleasedNow = FALSE;
	double adApplied[MAX_DOF];
	for (int i = 0; i < m_nDof; i++)
	{
		const BOOL bReleased = abReleased[i];
		if (!bReleased && !m_abBraked[i])
		{
			m_abBraked[i] = TRUE;				// brake engages where the joint is
			m_adBrakeQ[i] = d->qpos[m_anQ[i]];
		}
		else if (bReleased && m_abBraked[i])
		{
			m_abBraked[i] = FALSE;
			bReleasedNow = TRUE;
		}
		adApplied[i] = bReleased ? m_adTauQueue[sOut][i] : 0.0;
		d->qfrc_applied[m_anV[i]] = adApplied[i];
		if (m_abBraked[i])
			d->qvel[m_anV[i]] = 0.0;
		else
			bAllBraked = FALSE;
	}
	if (bReleasedNow)
	{
		for (int i = 0; i < m_nDof; i++)
			m_adQRelease[i] = d->qpos[m_anQ[i]];
		m_dMaxDisp = 0.0;
	}

	if (bAllBraked)
		d->time += m->opt.timestep;			// the arm rests on its brakes
	else
		mj_step(m, d);
	if (bAllBraked)
		m_nStepsBraked++;
	for (int i = 0; i < m_nDof; i++)
		if (m_abBraked[i])
		{
			d->qpos[m_anQ[i]] = m_adBrakeQ[i];
			d->qvel[m_anV[i]] = 0.0;
		}

	/* the state at the end of this step, and the torque that acted in it */
	const int sNew = Slot(k + 1);
	for (int i = 0; i < m_nDof; i++)
	{
		m_adHistQ[sNew][i] = d->qpos[m_anQ[i]];
		m_adHistQd[sNew][i] = d->qvel[m_anV[i]];
		m_adHistTau[sNew][i] = adApplied[i];
		if (!m_abBraked[i])
		{
			const double dDisp = std::fabs(m_adHistQ[sNew][i] - m_adQRelease[i]);
			if (dDisp > m_dMaxDisp) m_dMaxDisp = dDisp;
		}
	}
	m_nStep = k + 1;

	if (!bAllBraked)
	{
		const UINT64 uNs = (UINT64)(read_timer() - tStart);
		m_uStepNsSum += uNs;
		if (uNs > m_uStepNsMax) m_uStepNsMax = uNs;
	}
#else
	(void)adTau; (void)abReleased;
#endif
}

void
CSimArm::PrintSummary() const
{
	if (!m_bInit) return;
#ifdef G2T_WITH_MUJOCO
	const long nInt = m_nStep - m_nStepsBraked;
	printf("[SIM] %ld steps (%.3f s): %ld integrated, step time mean %.2f us, max %.2f us; "
	       "%ld with all brakes engaged; MuJoCo warnings %d\n",
	       m_nStep, ((mjData*)m_pD)->time, nInt,
	       nInt > 0 ? (double)m_uStepNsSum / (double)nInt / 1e3 : 0.0, (double)m_uStepNsMax / 1e3,
	       m_nStepsBraked, (int)s_nMjWarnings);
	if (s_nMjWarnings)
		printf("[SIM] last MuJoCo warning: %s\n", s_acMjWarning);
	printf("[SIM] largest joint displacement since the last brake release: %.3e rad; friction %s\n",
	       m_dMaxDisp, m_bFriction ? "on" : "off");
#endif
}
