/*****************************************************************************
*	Name: JointTestLog.cpp
*	Description: The format of the JOINT TEST log. See JointTestLog.h.
*****************************************************************************/
#include "JointTestLog.h"

#include <cmath>

void
JointTestFormat::Name(const Meta& astM, const char* astrStamp, char* acOut, size_t auLen)
{
	snprintf(acOut, auLen, "jt_%s_%s_%s_r%02d_%s", astM.acTest, astM.acLaw,
	         astM.bSim ? (astM.bSimFriction ? "simfr" : "sim") : "robot", astM.nRun, astrStamp);
}

void
JointTestFormat::Header(FILE* fp, const Meta& astM)
{
	const int nN = astM.nDof < (int)MAX_DOF ? astM.nDof : (int)MAX_DOF;
	static const char* const s_apcGroups[] = { "q_ref", "qd_ref", "qdd_ref", "q", "qd", "e", "tau",
	                                           "tau_g", "tau_c", "tau_mqdd", "tau_fb", "tau_ff" };
	static const char* const s_apcDrive[] = { "tau_act", "tau_act_raw", "tau_cmd_raw" };
	fprintf(fp, "t,phase");
	for (const char* pcG : s_apcGroups)
		for (int i = 0; i < nN; i++)
			fprintf(fp, ",%s%d", pcG, i);
	fprintf(fp, ",clamp");
	for (const char* pcG : s_apcDrive)
		for (int i = 0; i < nN; i++)
			fprintf(fp, ",%s%d", pcG, i);
	for (int i = 0; i < nN; i++)
		fprintf(fp, ",mii%d", i);
	for (int i = 0; i < nN; i++)
		fprintf(fp, ",mrow%d", i);
	fprintf(fp, "\n");
}

void
JointTestFormat::Begin()
{
	for (int i = 0; i < MAX_DOF; i++)
		adMaxE[i] = adMaxTau[i] = 0.0;
	nClamped = 0;
}

void
JointTestFormat::Row(FILE* fp, const Meta& astM, const Sample& s)
{
	const int nN = astM.nDof < (int)MAX_DOF ? astM.nDof : (int)MAX_DOF;
	fprintf(fp, "%.3f,%d", s.dT, s.nPhase);
	for (int i = 0; i < nN; i++) fprintf(fp, ",%.10f", s.adQRef[i]);
	for (int i = 0; i < nN; i++) fprintf(fp, ",%.9f", s.adQdRef[i]);
	for (int i = 0; i < nN; i++) fprintf(fp, ",%.9f", s.adQddRef[i]);
	for (int i = 0; i < nN; i++) fprintf(fp, ",%.10f", s.adQ[i]);
	for (int i = 0; i < nN; i++) fprintf(fp, ",%.9f", s.adQd[i]);
	for (int i = 0; i < nN; i++) fprintf(fp, ",%.6e", s.adE[i]);
	for (int i = 0; i < nN; i++) fprintf(fp, ",%.6f", s.adTau[i]);
	for (int i = 0; i < nN; i++) fprintf(fp, ",%.6f", s.adTauG[i]);
	for (int i = 0; i < nN; i++) fprintf(fp, ",%.6f", s.adTauC[i]);
	for (int i = 0; i < nN; i++) fprintf(fp, ",%.6f", s.adTauMqdd[i]);
	for (int i = 0; i < nN; i++) fprintf(fp, ",%.6f", s.adTauFb[i]);
	for (int i = 0; i < nN; i++) fprintf(fp, ",%.6f", s.adTauFf[i]);
	fprintf(fp, ",%u", s.uClamp);
	for (int i = 0; i < nN; i++) fprintf(fp, ",%.6f", s.adTauAct[i]);
	for (int i = 0; i < nN; i++) fprintf(fp, ",%d", s.anTauActRaw[i]);
	for (int i = 0; i < nN; i++) fprintf(fp, ",%d", s.anTauCmdRaw[i]);
	for (int i = 0; i < nN; i++) fprintf(fp, ",%.8f", s.adMii[i]);
	for (int i = 0; i < nN; i++) fprintf(fp, ",%.8f", s.adMRow[i]);
	fprintf(fp, "\n");
	for (int i = 0; i < nN; i++)
	{
		if (std::fabs(s.adE[i]) > adMaxE[i]) adMaxE[i] = std::fabs(s.adE[i]);
		if (std::fabs(s.adTau[i]) > adMaxTau[i]) adMaxTau[i] = std::fabs(s.adTau[i]);
	}
	if (s.uClamp)
		nClamped++;
}

void
JointTestFormat::Finish(FILE* fp, const char* astrCsv, const Meta& m, const Result& r,
                        const char* astrState, long anRows, unsigned anDrops)
{
	const int nN = m.nDof < (int)MAX_DOF ? m.nDof : (int)MAX_DOF;
	const char* pcBase = strrchr(astrCsv, '/');
	pcBase = pcBase ? pcBase + 1 : astrCsv;
	if (fp != NULL)
	{
		fprintf(fp, "{\n");
		fprintf(fp, "  \"format\": \"g2t-joint-test 1\",\n");
		fprintf(fp, "  \"csv\": \"%s\",\n", pcBase);
		fprintf(fp, "  \"test\": \"%s\",\n  \"law\": \"%s\",\n  \"run\": %d,\n", m.acTest, m.acLaw, m.nRun);
		fprintf(fp, "  \"platform\": \"%s\",\n", m.bSim ? "simulation" : "robot");
		fprintf(fp, "  \"sim_friction\": %s,\n", m.bSim && m.bSimFriction ? "true" : "false");
		fprintf(fp, "  \"sim_state_delay_steps\": %d,\n  \"sim_torque_delay_steps\": %d,\n",
		        m.nSimStateDelay, m.nSimTorqueDelay);
		testlog::PrintVec(fp, "sim_tau_s", m.adSimTauS, nN, "%.6g");
		testlog::PrintVec(fp, "dq", m.adDq, nN, "%.6g");
		fprintf(fp, "  \"T\": %.6g,\n  \"dwell\": %.6g,\n  \"period\": %.9g,\n", m.dT, m.dDwell, m.dPeriod);
		testlog::PrintVec(fp, "Kp", m.adKp, nN, "%.6g");
		testlog::PrintVec(fp, "Kd", m.adKd, nN, "%.6g");
		testlog::PrintVec(fp, "Kf", m.adKf, nN, "%.6g");
		fprintf(fp, "  \"ff_gate\": %s,\n", m.bFfGate ? "true" : "false");
		testlog::PrintVec(fp, "tau_max", m.adTauMax, nN, "%.6g");
		testlog::PrintVec(fp, "rotor_inertia", m.adB, nN, "%.8g");
		if (m.nSineJoint > 0)
			fprintf(fp, "  \"sine_joint\": %d,\n  \"sine_amp\": %.6g,\n  \"sine_freq\": %.6g,\n"
			            "  \"sine_periods\": %d,\n  \"sine_sweep\": %.6g,\n", m.nSineJoint, m.dSineAmp, m.dSineFreq,
			        m.nSinePeriods, m.dSineV0);
		if (m.nBaJoint > 0)
			fprintf(fp, "  \"breakaway_joint\": %d,\n  \"breakaway_rate\": %.6g,\n  \"breakaway_max\": %.6g,\n"
			            "  \"breakaway_qd\": %.6g,\n  \"breakaway_repeat\": %d,\n",
			        m.nBaJoint, m.dBaRate, m.dBaMax, m.dBaQd, m.nBaRep);
		if (m.nStepJoint > 0)
			fprintf(fp, "  \"step_joint\": %d,\n  \"step_tau\": %.6g,\n  \"step_on\": %.6g,\n"
			            "  \"step_off\": %.6g,\n  \"step_repeat\": %d,\n",
			        m.nStepJoint, m.dStepTau, m.dStepOn, m.dStepOff, m.nStepRep);
		testlog::PrintVec(fp, "q0", r.adQ0, nN, "%.12g");
		testlog::PrintVec(fp, "D", r.adD, nN, "%.12g");
		fprintf(fp, "  \"result\": \"%s\",\n", astrState);
		fprintf(fp, "  \"cycles\": %ld,\n  \"dropped\": %u,\n  \"clamped_cycles\": %ld,\n", anRows, anDrops, nClamped);
		/* timing faults during the test (paper, Sec. III-B, eq. (3)): late
		 * releases, overruns, missed send deadlines. In simulation the
		 * EtherCAT task steps the simulator once per cycle, so the loop
		 * keeps the timing of the robot whatever the host does:
		 * timing_faults is 0 there, and the counts are for information. */
		fprintf(fp, "  \"timing_faults\": %ld,\n", r.nTimingFaults);
		fprintf(fp, "  \"late_releases\": %ld,\n  \"overruns\": %ld,\n  \"missed_deadlines\": %ld,\n"
		        "  \"wkc_errors\": %ld,\n", r.nLate, r.nOverruns, r.nMissed, r.nWkc);
		fprintf(fp, "  \"allocations\": %ld,\n", r.nAllocs);
		/* controller faults (a non-finite state or torque: the test ended,
		 * the drives went off; Indy7Ctrl.h) and drive dropouts (a drive
		 * left Operation enabled: the test ended, every joint off) */
		fprintf(fp, "  \"controller_faults\": %ld,\n", r.nCtrlFaults);
		fprintf(fp, "  \"drive_dropouts\": %ld,\n", r.nDriveDropouts);
		testlog::PrintVec(fp, "max_abs_e", adMaxE, nN, "%.6e");
		testlog::PrintVec(fp, "max_abs_tau", adMaxTau, nN, "%.6g", true);
		fprintf(fp, "}\n");
	}
	printf("[JT] saved %s (%s %s, run %d: %s, %ld cycles, %u dropped, %ld limited%s)\n", astrCsv,
	       m.acTest, m.acLaw, m.nRun, astrState, anRows, anDrops, nClamped,
	       r.nTimingFaults > 0 ? ", TIMING FAULTS" : "");
	if (r.nTimingFaults > 0 || r.nWkc > 0)
		printf("[JT] during the test: %ld late releases, %ld overruns, %ld missed send deadlines, "
		       "%ld incomplete working counters\n", r.nLate, r.nOverruns, r.nMissed, r.nWkc);
	if (r.nAllocs > 0)
		printf("[JT] %ld heap allocations in the control law during the test\n", r.nAllocs);
	if (r.nCtrlFaults > 0)
		printf("[JT] CONTROLLER FAULT during the test: it ended, the drives went off\n");
	if (r.nDriveDropouts > 0)
		printf("[JT] DRIVE DROPOUT during the test: it ended, every axis went off\n");
	printf("[JT] max |e| [mrad]:");
	for (int i = 0; i < nN; i++)
		printf(" %.4f", adMaxE[i] * 1e3);
	printf("\n");
}
