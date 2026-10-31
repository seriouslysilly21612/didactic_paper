/*****************************************************************************
*	Name: HighLevelLog.cpp
*	Description: The format of the high-level test log. See HighLevelLog.h.
*****************************************************************************/
#include "HighLevelLog.h"

#include <cmath>

void
HighLevelFormat::Name(const Meta& astM, const char* astrStamp, char* acOut, size_t auLen)
{
	snprintf(acOut, auLen, "hl_%s_%s_r%02d_%s", astM.acTest,
	         astM.bSim ? (astM.bSimFriction ? "simfr" : "sim") : "robot", astM.nRun, astrStamp);
}

void
HighLevelFormat::Header(FILE* fp, const Meta& astM)
{
	const int nN = astM.nDof < (int)MAX_DOF ? astM.nDof : (int)MAX_DOF;
	fprintf(fp, "t,mode,flags,otg,lead_reset,ik_iter,ik_err,otg_dur,hl_us,goal_age,"
	            "goal_x,goal_y,goal_z,acc_x,acc_y,acc_z,tool_x,tool_y,tool_z");
	static const char* const s_apcGroups[] = { "target", "q_ref", "qd_ref", "qdd_ref", "q", "qd", "tau" };
	for (const char* pcG : s_apcGroups)
		for (int i = 0; i < nN; i++)
			fprintf(fp, ",%s%d", pcG, i);
	fprintf(fp, ",clamp\n");
}

void
HighLevelFormat::Begin()
{
	dMaxHlUs = dMaxLead = dMaxDQdd = dMaxDTau = 0.0;
	for (int k = 0; k < 3; k++)
		adLastTool[k] = adLastAcc[k] = 0.0;
	bHasPrev = false;
}

void
HighLevelFormat::Row(FILE* fp, const Meta& astM, const Sample& s)
{
	const int nN = astM.nDof < (int)MAX_DOF ? astM.nDof : (int)MAX_DOF;
	fprintf(fp, "%.3f,%d,%u,%d,%u,%d,%.3e,%.6f,%.2f,%ld", s.dT, s.nMode, s.uFlags, s.nOtg, s.uLeadReset,
	        s.nIkIter, s.dIkErr, s.dOtgDuration, s.dHlUs, s.nGoalAge);
	for (int k = 0; k < 3; k++)
	{
		if (std::isfinite(s.adGoal[k])) fprintf(fp, ",%.7f", s.adGoal[k]);
		else fputs(",", fp);
	}
	for (int k = 0; k < 3; k++) fprintf(fp, ",%.7f", s.adAccepted[k]);
	for (int k = 0; k < 3; k++) fprintf(fp, ",%.9f", s.adTool[k]);
	for (int i = 0; i < nN; i++) fprintf(fp, ",%.10f", s.adTarget[i]);
	for (int i = 0; i < nN; i++) fprintf(fp, ",%.10f", s.adQRef[i]);
	for (int i = 0; i < nN; i++) fprintf(fp, ",%.9f", s.adQdRef[i]);
	for (int i = 0; i < nN; i++) fprintf(fp, ",%.9f", s.adQddRef[i]);
	for (int i = 0; i < nN; i++) fprintf(fp, ",%.10f", s.adQ[i]);
	for (int i = 0; i < nN; i++) fprintf(fp, ",%.9f", s.adQd[i]);
	for (int i = 0; i < nN; i++) fprintf(fp, ",%.6f", s.adTau[i]);
	fprintf(fp, ",%u\n", s.uClamp);
	if (s.dHlUs > dMaxHlUs) dMaxHlUs = s.dHlUs;
	for (int k = 0; k < 3; k++)
	{
		adLastTool[k] = s.adTool[k];
		adLastAcc[k] = s.adAccepted[k];
	}
	for (int i = 0; i < nN; i++)
	{
		const double dLead = std::fabs(s.adQRef[i] - s.adQ[i]);
		if (dLead > dMaxLead) dMaxLead = dLead;
		if (bHasPrev && !(s.uLeadReset & (1u << i)))
		{
			const double dA = std::fabs(s.adQddRef[i] - adPrevQdd[i]);
			if (dA > dMaxDQdd) dMaxDQdd = dA;
		}
		if (bHasPrev)
		{
			const double dT = std::fabs(s.adTau[i] - adPrevTau[i]);
			if (dT > dMaxDTau) dMaxDTau = dT;
		}
		adPrevQdd[i] = s.adQddRef[i];
		adPrevTau[i] = s.adTau[i];
	}
	bHasPrev = true;
}

void
HighLevelFormat::Finish(FILE* fp, const char* astrCsv, const Meta& m, const Result& r,
                        const char* astrState, long anRows, unsigned anDrops)
{
	const int nN = m.nDof < (int)MAX_DOF ? m.nDof : (int)MAX_DOF;
	const char* pcBase = strrchr(astrCsv, '/');
	pcBase = pcBase ? pcBase + 1 : astrCsv;
	const double dDist = std::sqrt((adLastTool[0] - adLastAcc[0]) * (adLastTool[0] - adLastAcc[0]) +
	                               (adLastTool[1] - adLastAcc[1]) * (adLastTool[1] - adLastAcc[1]) +
	                               (adLastTool[2] - adLastAcc[2]) * (adLastTool[2] - adLastAcc[2]));
	if (fp != NULL)
	{
		const CHighLevelRT::Params& p = m.stP;
		fprintf(fp, "{\n");
		fprintf(fp, "  \"format\": \"g2t-high-level-test 1\",\n");
		fprintf(fp, "  \"csv\": \"%s\",\n", pcBase);
		fprintf(fp, "  \"test\": \"%s\",\n  \"run\": %d,\n", m.acTest, m.nRun);
		fprintf(fp, "  \"platform\": \"%s\",\n", m.bSim ? "simulation" : "robot");
		fprintf(fp, "  \"sim_friction\": %s,\n", m.bSim && m.bSimFriction ? "true" : "false");
		fprintf(fp, "  \"sim_state_delay_steps\": %d,\n  \"sim_torque_delay_steps\": %d,\n",
		        m.nSimStateDelay, m.nSimTorqueDelay);
		testlog::PrintVec(fp, "sim_tau_s", m.adSimTauS, nN, "%.6g");
		fprintf(fp, "  \"friction_ff\": %s,\n", m.bFrictionFF ? "true" : "false");
		fprintf(fp, "  \"period\": %.9g,\n", m.dPeriod);
		fprintf(fp, "  \"delta\": %.6g,\n  \"lambda\": %.6g,\n  \"step_max\": %.6g,\n  \"tol\": %.6g,\n",
		        p.dDelta, p.dLambda, p.dStepMax, p.dTol);
		fprintf(fp, "  \"iter_cycle\": %d,\n  \"iter_goal\": %d,\n", p.nIterCycle, p.nIterMax);
		fprintf(fp, "  \"vel_max\": %.6g,\n  \"acc_max\": %.6g,\n  \"jerk_max\": %.6g,\n", p.dVelMax, p.dAccMax, p.dJerkMax);
		fprintf(fp, "  \"margin\": %.6g,\n  \"lead\": %.6g,\n  \"rest_qd\": %.6g,\n  \"rest_cycles\": %d,\n",
		        p.dMargin, p.dLead, p.dRestQd, p.nRestCyc);
		fprintf(fp, "  \"step_baseline\": %s,\n", p.bStepRef ? "true" : "false");
		testlog::PrintVec(fp, "q_r", p.adQr, nN, "%.12g");
		testlog::PrintVec(fp, "lim_lo", p.adLimLo, nN, "%.12g");
		testlog::PrintVec(fp, "lim_hi", p.adLimHi, nN, "%.12g");
		testlog::PrintVec(fp, "p0", m.adP0, 3, "%.9g");
		testlog::PrintVec(fp, "goal_offset", m.adGoalOffset, 3, "%.9g");
		fprintf(fp, "  \"goal_index\": %d,\n  \"goals\": %d,\n", m.nGoal, m.nGoals);
		testlog::PrintVec(fp, "jump", m.adJump, 3, "%.9g");
		fprintf(fp, "  \"trigger\": %.6g,\n  \"noise\": %.6g,\n  \"noise_hz\": %.6g,\n  \"duration\": %.6g,\n",
		        m.dTrigger, m.dNoise, m.dNoiseHz, m.dDuration);
		fprintf(fp, "  \"noise_file\": \"%s\",\n", m.acNoiseFile);
		testlog::PrintVec(fp, "Kp", m.adKp, nN, "%.6g");
		testlog::PrintVec(fp, "Kd", m.adKd, nN, "%.6g");
		testlog::PrintVec(fp, "Kf", m.adKf, nN, "%.6g");
		testlog::PrintVec(fp, "tau_max", m.adTauMax, nN, "%.6g");
		fprintf(fp, "  \"result\": \"%s\",\n", astrState);
		fprintf(fp, "  \"cycles\": %ld,\n  \"dropped\": %u,\n", anRows, anDrops);
		/* as in JointTestLog.cpp: 0 in simulation */
		fprintf(fp, "  \"timing_faults\": %ld,\n", r.nTimingFaults);
		fprintf(fp, "  \"late_releases\": %ld,\n  \"overruns\": %ld,\n  \"missed_deadlines\": %ld,\n"
		        "  \"wkc_errors\": %ld,\n", r.nLate, r.nOverruns, r.nMissed, r.nWkc);
		fprintf(fp, "  \"allocations\": %ld,\n", r.nAllocs);
		/* as in JointTestLog.cpp */
		fprintf(fp, "  \"controller_faults\": %ld,\n", r.nCtrlFaults);
		fprintf(fp, "  \"drive_dropouts\": %ld,\n", r.nDriveDropouts);
		fprintf(fp, "  \"accepted\": %ld,\n  \"converged\": %ld,\n  \"rejected\": %ld,\n  \"lead_resets\": %ld,\n",
		        r.nAccepted, r.nConverged, r.nRejected, r.nLeadResets);
		fprintf(fp, "  \"rest_time\": %.6g,\n", r.dRestTime);
		fprintf(fp, "  \"final_distance\": %.6e,\n", dDist);
		fprintf(fp, "  \"max_lead\": %.6e,\n  \"max_dqdd_cycle\": %.6e,\n  \"max_dtau_cycle\": %.6e,\n",
		        dMaxLead, dMaxDQdd, dMaxDTau);
		fprintf(fp, "  \"max_hl_us\": %.3f\n", dMaxHlUs);
		fprintf(fp, "}\n");
	}
	printf("[HL] saved %s (%s, run %d: %s, %ld cycles, %u dropped%s)\n", astrCsv, m.acTest, m.nRun, astrState,
	       anRows, anDrops, r.nTimingFaults > 0 ? ", TIMING FAULTS" : "");
	if (r.nTimingFaults > 0 || r.nWkc > 0)
		printf("[HL] during the test: %ld late releases, %ld overruns, %ld missed send deadlines, "
		       "%ld incomplete working counters\n", r.nLate, r.nOverruns, r.nMissed, r.nWkc);
	if (r.nCtrlFaults > 0)
		printf("[HL] CONTROLLER FAULT during the test: it ended, the drives went off\n");
	if (r.nDriveDropouts > 0)
		printf("[HL] DRIVE DROPOUT during the test: it ended, every axis went off\n");
	printf("[HL] tool %.3f mm from the accepted goal; goals accepted %ld, rejected %ld; lead resets %ld; "
	       "max lead %.4f rad; max change of qdd_ref %.4f rad/s^2/cycle; at rest %s%.3f s; HL step max %.1f us%s\n",
	       dDist * 1e3, r.nAccepted, r.nRejected, r.nLeadResets, dMaxLead, dMaxDQdd,
	       r.dRestTime < 0 ? "never " : "after ", r.dRestTime < 0 ? 0.0 : r.dRestTime, dMaxHlUs,
	       r.nAllocs > 0 ? " - HEAP ALLOCATIONS" : "");
}
