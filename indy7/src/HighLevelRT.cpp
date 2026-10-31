/*****************************************************************************
*	Name: HighLevelRT.cpp
*	Description: The high-level controller (paper, Sec. IV-C). See
*	HighLevelRT.h.
*****************************************************************************/
#include "HighLevelRT.h"
#include "CallTiming.h"

#include <rbdl/addons/urdfreader/urdfreader.h>

#include <cmath>
#include <cstring>
#include <limits>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

template <int N>
typename CHighLevelRTN<N>::Params
CHighLevelRTN<N>::DefaultParams()
{
	Params p;
	p.dDelta     = 0.006;			// the values of Table 9; the cfg sets them (HL_* keys)
	p.dLambda    = 0.01;
	p.dStepMax   = 0.2;
	p.dTol       = 1e-4;
	p.nIterCycle = 10;
	p.nIterMax   = 100;
	p.dVelMax    = 0.6;
	p.dAccMax    = 2.0;
	p.dJerkMax   = 10.0;
	p.dMargin    = 0.26;			// above 0.253 rad, the largest modeled overshoot of a replanned target
	p.dLead      = 0.10;
	p.dRestQd    = 0.05;
	p.nRestCyc   = 200;
	/* the arm (N = 6): q_r and the URDF joint limits of indy7.urdf, J1-J5 175 deg, J6 215 deg; other N:
	 * zero and +-pi, to be set by the caller */
	const double adQr6[6] = { 0.0, 0.0, -M_PI / 2, 0.0, -M_PI / 2, 0.0 };
	for (int i = 0; i < DOF; i++)
	{
		p.adQr[i] = (N == 6) ? adQr6[i] : 0.0;
		p.adLimHi[i] = (N == 6) ? ((i == 5) ? 3.7524578917878086 : 3.0543261909900767) : M_PI;
		p.adLimLo[i] = -p.adLimHi[i];
	}
	snprintf(p.acTool, sizeof(p.acTool), "%s", "tcp");
	p.bStepRef   = false;			// the step baseline is off
	return p;
}

template <int N>
CHighLevelRTN<N>::CHighLevelRTN()
	: m_bInit(false), m_bHome(false), m_dTs(0.001), m_uTcp(0), m_otg(0.001), m_nRest(0)
{
	m_p = DefaultParams();
	memset(&m_st, 0, sizeof(m_st));
	m_vTcpPoint.setZero();
}

template <int N>
BOOL
CHighLevelRTN<N>::Init(const char* astrURDF, double adTs, const Params& astParams)
{
	if (!Addons::URDFReadFromFile(astrURDF, &m_model, false, false))
	{
		DBG_LOG_ERROR("(CHighLevelRT) cannot read %s", astrURDF);
		return FALSE;
	}
	if (m_model.qdot_size != (unsigned int)DOF)
	{
		DBG_LOG_ERROR("(CHighLevelRT) %u joints in %s, %d expected", m_model.qdot_size, astrURDF, (int)DOF);
		return FALSE;
	}
	m_uTcp = m_model.GetBodyId(astParams.acTool);
	if (m_uTcp == std::numeric_limits<unsigned int>::max())
	{
		DBG_LOG_ERROR("(CHighLevelRT) no body \"%s\" in %s", astParams.acTool, astrURDF);
		return FALSE;
	}
	if (!(adTs > 0.0))
		return FALSE;
	m_vQk  = VectorNd::Zero(DOF);
	m_vQ   = VectorNd::Zero(DOF);
	m_vZero = VectorNd::Zero(DOF);
	m_mG   = MatrixNd::Zero(3, DOF);
	m_dTs  = adTs;
	m_otg.delta_time = adTs;
	m_p = astParams;
	for (int i = 0; i < DOF; i++)
	{
		m_in.max_velocity[i]        = m_p.dVelMax;
		m_in.max_acceleration[i]    = m_p.dAccMax;
		m_in.max_jerk[i]            = m_p.dJerkMax;
		m_in.target_velocity[i]     = 0.0;
		m_in.target_acceleration[i] = 0.0;
	}
	m_in.synchronization = ruckig::Synchronization::Time;	// all joints arrive at once
	m_bInit = true;

	/* the ready posture must lie inside the limits minus the margin */
	for (int i = 0; i < DOF; i++)
		if (m_p.adQr[i] < m_p.adLimLo[i] + m_p.dMargin || m_p.adQr[i] > m_p.adLimHi[i] - m_p.dMargin)
			DBG_LOG_WARN("(CHighLevelRT) q_r of J%d (%.3f rad) is not inside the limits minus %.2f rad",
			             i + 1, m_p.adQr[i], m_p.dMargin);
	DBG_LOG_INFO("(CHighLevelRT) delta %.1f mm; IK lambda %.3g m, step %.2f rad, tol %.2f mm, %d/cycle, %d/goal; "
	             "limits %.2f rad/s, %.2f rad/s^2, %.1f rad/s^3; margin %.2f rad, lead %.2f rad",
	             m_p.dDelta * 1e3, m_p.dLambda, m_p.dStepMax, m_p.dTol * 1e3, m_p.nIterCycle, m_p.nIterMax,
	             m_p.dVelMax, m_p.dAccMax, m_p.dJerkMax, m_p.dMargin, m_p.dLead);
	return TRUE;
}

template <int N>
void
CHighLevelRTN<N>::SetReady(const double* adQr)
{
	for (int i = 0; i < DOF; i++)
		m_p.adQr[i] = adQr[i];
}

template <int N>
void
CHighLevelRTN<N>::StartFrom(const double* adQ, const double* adQd)
{
	memset(&m_st, 0, sizeof(m_st));		// no accepted goal, counters zero
	for (int i = 0; i < DOF; i++)
	{
		m_in.current_position[i]     = adQ[i];
		m_in.current_velocity[i]     = adQd[i];
		m_in.current_acceleration[i] = 0.0;
		m_in.target_position[i]      = adQ[i];		// hold until a goal is solved
		m_st.adTarget[i]             = adQ[i];
		m_out.new_position[i]        = adQ[i];
		m_out.new_velocity[i]        = adQd[i];
		m_out.new_acceleration[i]    = 0.0;
	}
	m_nRest = 0;
}

template <int N>
void
CHighLevelRTN<N>::StartTrack(const double* adQ, const double* adQd)
{
	m_bHome = false;
	StartFrom(adQ, adQd);
}

template <int N>
void
CHighLevelRTN<N>::StartHome(const double* adQ, const double* adQd)
{
	StartHomeTo(adQ, adQd, m_p.adQr);
}

template <int N>
void
CHighLevelRTN<N>::StartHomeTo(const double* adQ, const double* adQd, const double* adTarget)
{
	m_bHome = true;
	StartFrom(adQ, adQd);
	for (int i = 0; i < DOF; i++)
	{
		m_in.target_position[i] = adTarget[i];
		m_st.adTarget[i]        = adTarget[i];
	}
}

template <int N>
void
CHighLevelRTN<N>::SetTarget(const VectorNd& aq)
{
	for (int i = 0; i < DOF; i++)
	{
		m_in.target_position[i] = aq[i];
		m_st.adTarget[i]        = aq[i];
	}
}

template <int N>
BOOL
CHighLevelRTN<N>::Update(const double* adGoal, const double* adQ, const double* adQd,
                     double* adQRef, double* adQdRef, double* adQddRef)
{
	const RTTIME tStart = read_timer();
	m_st.bGoalAccepted = false;
	m_st.bIkConverged  = false;
	m_st.bIkRejected   = false;
	m_st.bOtgNewCalc   = false;
	m_st.uLeadReset    = 0;

	/* Step 1, goal admission: a goal within delta of the accepted goal is
	 * ignored; one farther away replaces it and restarts the IK from q_r */
	if (!m_bHome && adGoal != NULL)
	{
		double dD2 = 0.0;
		for (int k = 0; k < 3; k++)
			dD2 += (adGoal[k] - m_st.adGoal[k]) * (adGoal[k] - m_st.adGoal[k]);
		if (!m_st.bHasGoal || dD2 > m_p.dDelta * m_p.dDelta)
		{
			for (int k = 0; k < 3; k++)
				m_st.adGoal[k] = adGoal[k];
			for (int i = 0; i < DOF; i++)
				m_vQk[i] = m_p.adQr[i];
			m_st.bHasGoal      = true;
			m_st.bIkActive     = true;
			m_st.nIkIter       = 0;
			m_st.bGoalAccepted = true;
			m_st.nAccepted++;
			// the arm is not at rest at the new goal: the rest counter
			// restarts (it had kept counting while the IK solved it)
			m_nRest = 0;
			m_st.bAtRest = false;
		}
	}

	/* Step 2, inverse kinematics: DLS steps on the model of the IK, at most
	 * N_it per cycle; converged -> target of Step 3, N_max -> rejected (the
	 * previous target is kept) */
	const Vector3d vGoal(m_st.adGoal[0], m_st.adGoal[1], m_st.adGoal[2]);
	const double dL2 = m_p.dLambda * m_p.dLambda;
	for (int k = 0; m_st.bIkActive && k < m_p.nIterCycle; k++)
	{
		// call timing (CallTiming.h): 0 and no clock read when off
		const RTTIME tIt0 = call_timing::Now();
		UpdateKinematics(m_model, m_vQk, m_vZero, m_vZero);
		RTTIME tCt = call_timing::Lap(call_timing::eIkKin, tIt0);
		const Vector3d vE = vGoal - CalcBodyToBaseCoordinates(m_model, m_vQk, m_uTcp, m_vTcpPoint, false);
		tCt = call_timing::Lap(call_timing::eIkFk, tCt);
		m_st.dIkErr = vE.norm();
		if (m_st.dIkErr < m_p.dTol)
		{
			m_st.bIkActive    = false;
			m_st.bIkConverged = true;
			m_st.nConverged++;
			SetTarget(m_vQk);
		}
		else if (++m_st.nIkIter > m_p.nIterMax)
		{
			m_st.bIkActive   = false;
			m_st.bIkRejected = true;
			m_st.nRejected++;
		}
		else
		{
			m_mG.setZero();				// RBDL fills only the nonzero entries
			CalcPointJacobian(m_model, m_vQk, m_uTcp, m_vTcpPoint, m_mG, false);
			tCt = call_timing::Lap(call_timing::eIkJac, tCt);
			m_mJp = m_mG;				// fixed size from here on
			Eigen::Matrix3d mA = m_mJp * m_mJp.transpose();
			mA(0, 0) += dL2;
			mA(1, 1) += dL2;
			mA(2, 2) += dL2;
			m_ldlt.compute(mA);
			Eigen::Matrix<double, DOF, 1> vDq = m_mJp.transpose() * m_ldlt.solve(vE);
			const double dN = vDq.norm();
			if (dN > m_p.dStepMax)
				vDq *= m_p.dStepMax / dN;
			for (int i = 0; i < DOF; i++)
			{
				double dQ = m_vQk[i] + vDq[i];
				const double dLo = m_p.adLimLo[i] + m_p.dMargin, dHi = m_p.adLimHi[i] - m_p.dMargin;
				m_vQk[i] = dQ < dLo ? dLo : (dQ > dHi ? dHi : dQ);
			}
			call_timing::Lap(call_timing::eIkSolve, tCt);
		}
		call_timing::Lap(call_timing::eIkIter, tIt0);
	}

	/* the step baseline (simulation only, HL_STEP_BASELINE): the IK solution
	 * is the reference, as a step, without the generator (Step 3) and the
	 * lead bound (Step 4); the arm is at rest when |qd| has stayed small, as
	 * below */
	if (m_p.bStepRef)
	{
		double dQd2 = 0.0;
		for (int i = 0; i < DOF; i++)
		{
			adQRef[i]   = m_in.target_position[i];
			adQdRef[i]  = 0.0;
			adQddRef[i] = 0.0;
			m_in.current_position[i]     = m_in.target_position[i];
			m_in.current_velocity[i]     = 0.0;
			m_in.current_acceleration[i] = 0.0;
			dQd2 += adQd[i] * adQd[i];
		}
		m_st.nOtg = (int)ruckig::Result::Finished;
		m_nRest = (dQd2 <= m_p.dRestQd * m_p.dRestQd) ? m_nRest + 1 : 0;
		m_st.bAtRest = m_nRest >= m_p.nRestCyc;
		m_st.dUs = (double)(read_timer() - tStart) * 1e-3;
		return TRUE;
	}

	/* Step 3, trajectory generation from the current state of the reference */
	const RTTIME tOtg = call_timing::Now();
	const ruckig::Result eRes = m_otg.update(m_in, m_out);
	call_timing::Lap(call_timing::eOtg, tOtg);
	m_st.nOtg = (int)eRes;
	if (eRes < 0)
	{
		/* invalid input: keep the last reference; the caller stops the motion */
		for (int i = 0; i < DOF; i++)
		{
			adQRef[i]   = m_in.current_position[i];
			adQdRef[i]  = m_in.current_velocity[i];
			adQddRef[i] = m_in.current_acceleration[i];
		}
		m_st.bAtRest = false;
		m_nRest = 0;
		m_st.dUs = (double)(read_timer() - tStart) * 1e-3;
		return FALSE;
	}
	if (m_out.new_calculation)
	{
		m_st.bOtgNewCalc  = true;
		m_st.dOtgDuration = m_out.trajectory.get_duration();
		m_st.dOtgCalcUs   = m_out.calculation_duration;
	}
	m_out.pass_to_input(m_in);
	for (int i = 0; i < DOF; i++)
	{
		adQRef[i]   = m_out.new_position[i];
		adQdRef[i]  = m_out.new_velocity[i];
		adQddRef[i] = m_out.new_acceleration[i];
	}

	/* Step 4, lead bound: a joint whose reference leads by ell gets its
	 * measured state in the generator, which replans from there */
	for (int i = 0; i < DOF; i++)
		if (std::fabs(m_out.new_position[i] - adQ[i]) >= m_p.dLead)
		{
			m_in.current_position[i]     = adQ[i];
			m_in.current_velocity[i]     = adQd[i];
			m_in.current_acceleration[i] = 0.0;
			m_st.uLeadReset |= 1u << i;
			m_st.nLeadResets++;
		}

	/* at rest: the generator has reached its target and |qd| has stayed small */
	if (eRes == ruckig::Result::Finished)
	{
		double dQd2 = 0.0;
		for (int i = 0; i < DOF; i++)
			dQd2 += adQd[i] * adQd[i];
		m_nRest = (dQd2 <= m_p.dRestQd * m_p.dRestQd) ? m_nRest + 1 : 0;
	}
	else
		m_nRest = 0;
	m_st.bAtRest = m_nRest >= m_p.nRestCyc;
	m_st.dUs = (double)(read_timer() - tStart) * 1e-3;
	return TRUE;
}

template <int N>
void
CHighLevelRTN<N>::ToolPosition(const double* adQ, double* adP)
{
	for (int i = 0; i < DOF; i++)
		m_vQ[i] = adQ[i];
	UpdateKinematics(m_model, m_vQ, m_vZero, m_vZero);
	const Vector3d vP = CalcBodyToBaseCoordinates(m_model, m_vQ, m_uTcp, m_vTcpPoint, false);
	for (int k = 0; k < 3; k++)
		adP[k] = vP[k];
}

// the iterations of Step 2 in Update(), without the bound per cycle: the
// same start q_r, steps, clipping, tolerance, and N_max, so that a goal that
// converges here converges in TRACK
template <int N>
BOOL
CHighLevelRTN<N>::SolveIk(const double* adGoal, double* adQ, int* pnIter, double* pdErr)
{
	const Vector3d vGoal(adGoal[0], adGoal[1], adGoal[2]);
	const double dL2 = m_p.dLambda * m_p.dLambda;
	for (int i = 0; i < DOF; i++)
		m_vQk[i] = m_p.adQr[i];
	int nIter = 0;
	BOOL bOk = FALSE;
	double dErr = 0.0;
	for (;;)
	{
		UpdateKinematics(m_model, m_vQk, m_vZero, m_vZero);
		const Vector3d vE = vGoal - CalcBodyToBaseCoordinates(m_model, m_vQk, m_uTcp, m_vTcpPoint, false);
		dErr = vE.norm();
		if (dErr < m_p.dTol)
		{
			bOk = TRUE;
			break;
		}
		if (++nIter > m_p.nIterMax)
			break;
		m_mG.setZero();
		CalcPointJacobian(m_model, m_vQk, m_uTcp, m_vTcpPoint, m_mG, false);
		m_mJp = m_mG;
		Eigen::Matrix3d mA = m_mJp * m_mJp.transpose();
		mA(0, 0) += dL2;
		mA(1, 1) += dL2;
		mA(2, 2) += dL2;
		m_ldlt.compute(mA);
		Eigen::Matrix<double, DOF, 1> vDq = m_mJp.transpose() * m_ldlt.solve(vE);
		const double dN = vDq.norm();
		if (dN > m_p.dStepMax)
			vDq *= m_p.dStepMax / dN;
		for (int i = 0; i < DOF; i++)
		{
			double dQ = m_vQk[i] + vDq[i];
			const double dLo = m_p.adLimLo[i] + m_p.dMargin, dHi = m_p.adLimHi[i] - m_p.dMargin;
			m_vQk[i] = dQ < dLo ? dLo : (dQ > dHi ? dHi : dQ);
		}
	}
	if (adQ != NULL)
		for (int i = 0; i < DOF; i++)
			adQ[i] = m_vQk[i];
	if (pnIter != NULL)
		*pnIter = nIter;
	if (pdErr != NULL)
		*pdErr = dErr;
	return bOk;
}

/* the instances: the arm (6) and a hand of the humanoid (7) */
template class CHighLevelRTN<6>;
template class CHighLevelRTN<7>;
