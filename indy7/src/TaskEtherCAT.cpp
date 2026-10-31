/*****************************************************************************
*	Name: TaskEtherCAT.cpp
*	Description: The EtherCAT task (TASK0, paper, Sec. III-B and Fig. 2).
*	In every period, after the release:
*	  receive  the frame of the last period (in simulation: the state that
*	           the simulator returns, CSimArm::GetMeasured); both working
*	           counters and the OP state of the slaves;
*	  state    the measurement, its validity (bOp), the servo state and the
*	           fault counts of this period -> State slot, for the Trajectory
*	           and the Dynamics task;
*	  drives   the drive dropout (a joint that reached Operation enabled
*	           after 'h' leaves it without a command: every joint off) and
*	           the end of a servo on that did not complete (SERVO_ON_TIMEOUT_MS:
*	           every joint off); if the input is valid: the command of the
*	           operator (servo on, off, stop; m_nServoCmd from DoInput), then
*	           the torque of the Dynamics task's last cycle (Torque slot) as
*	           the target of this period; otherwise no new target, and the
*	           last one stays;
*	  send     the frame of this period (in simulation: one simulator step);
*	  then     the AL states, the live view, the joint state for the bridge,
*	           the LED of the end tool, and the timing of this period
*	           (TimingLog.h).
*	With -b both: the frame of the bus of the robot is received and sent
*	(its joint drives stay off), the simulator gives the state of the arm,
*	and it steps after the send, as the robot moves after it.
*	In the simulation (-s) the simulator steps here, after the send, once in
*	every period: the simulated loop runs on the release grid of the host,
*	and a period that runs late delays it as a late period would delay the
*	loop on the robot (the controller assumes the real-time host of the
*	paper, Table 3).
*
*	Timing (eq. (3)): t_w from the planned release of the period (the grid,
*	RtTask.h) to the start of the cycle, t_s from the start to the return of
*	the send, t_e from the start to the end. A release with t_w > T is late;
*	its cycle runs nevertheless (nothing is skipped), and the next releases
*	stay on the grid. A send with t_w + t_s > t_b (SEND_DEADLINE) is missed,
*	a cycle with t_w + t_e > T overruns.
*
*	After a stop ('q' or a signal), the drives are switched off for 1 s
*	while the exchange goes on; then the task releases the master (the slaves
*	go to PREOP without a watchdog alarm), waits for the two other tasks,
*	prints the summaries, and ends.
*
*	The servo-on request and the E-stop of the control box. While the
*	request of a drive is on, every period walks it toward Operation enabled
*	(CIndy7Joint::WriteToSlave); a drive that cannot be enabled (no power) is
*	walked on as soon as it can, by itself. So no request may outlive the
*	moment it was meant for:
*	  - a dropout (a drive that had reached Operation enabled after 'h' left
*	    it without a command) switches every joint off in that period, as 'e'
*	    does, and the motion ends; 'h' enables the drives again;
*	  - 'h' is refused while a drive stays in Switch on disabled (DoInput,
*	    State slot uNotReady), which a pressed E-stop may cause;
*	  - a request that has not brought every joint to Operation enabled
*	    within SERVO_ON_TIMEOUT_MS is withdrawn (every joint off): it covers
*	    what the refusal cannot tell from a drive that is ready (an E-stop
*	    reported as Ready to switch on or as Fault), and a fault that a reset
*	    did not clear, whose next reset then comes with the next 'h'.
*	An 'h' that arrives in the period of a dropout or of a withdrawal is
*	dropped: it was given before the drives went off.
*****************************************************************************/
#include "Indy7Ctrl.h"
#include "base/MallocCount.h"

#include <cstring>

void
proc_ethercat_control(void* apRobot)
{
	typedef CTimingLog TL;
	using namespace task_slots;
	auto* pRobot = static_cast<CRobotIndy7*>(apRobot);
	CRtTask* const pTask = CRtTask::Self();
	CSimArm* const pSim = pRobot->m_pSim;
	// the EtherCAT master: on the robot, and with -b beside the simulator
	CEcatMaster* const pMaster = pRobot->m_pcEcatMaster;
	const BOOL bHybrid = (pSim != NULL && pMaster != NULL) ? TRUE : FALSE;
	DBG_LOG_INFO("(proc_ethercat_control) EtherCAT task started (%s)",
	             bHybrid ? "the bus of the robot, drives off, beside the simulated arm" :
	             pSim != NULL ? "simulation: MuJoCo in place of the EtherCAT exchange" : "EtherCAT");

	const unsigned int udof = (unsigned int)pRobot->GetTotalAxis();
	if (pTask == NULL || udof == 0 || udof > (unsigned int)CSimArm::MAX_DOF || udof > (unsigned int)MAX_DOF ||
	    (pSim == NULL && pMaster == NULL))
	{
		DBG_LOG_ERROR("(proc_ethercat_control) %u joints, %s", udof, pMaster ? "EtherCAT" : "no EtherCAT master");
		pRobot->m_bEcatDone.store(true, std::memory_order_release);
		return;
	}

	CRobotIndy7::RtStats& st = pRobot->m_stRt;
	st.tPeriod = pTask->Period();
	if (st.tDeadline == 0)
		st.tDeadline = st.tPeriod;
	double adQ[CSimArm::MAX_DOF], adQd[CSimArm::MAX_DOF], adTauM[CSimArm::MAX_DOF], adTau[CSimArm::MAX_DOF];
	BOOL   abReleased[CSimArm::MAX_DOF];
	double adViewQ[CViewStream::MAX_DOF] = {0.0};
	BOOL   bBusUp = FALSE;        // the working counters were complete once
	BOOL   bOpSeen = FALSE;       // the bus was OP once (printed)
	BOOL   bBusAlarm = FALSE;     // -b: a drive of the bus was found enabled (printed once)
	BOOL   bStaleTold = FALSE;    // no fresh torque while OP (printed once)
	int    nCleanUp = 0;
	CTimingLog& cLog = pRobot->m_cTimingLog;
	const uint16_t usMode = (uint16_t)((pSim != NULL ? TL::TF_SIM : 0) | (pMaster != NULL ? TL::TF_BUS : 0));
	// the LED of the end tool shows the state of the controller on the robot
	// (with -b the end tool stays as it is)
	CIndy7EndTool* const pTool = bHybrid ? NULL : pRobot->m_pEndTool;

	// the slots: the state of this period (written here), the torque and the
	// reference of the other tasks (read here)
	ST_STATE_SLOT  stState;
	ST_TORQUE_SLOT stTorque;
	ST_REF_SLOT    stRef;
	memset(&stState, 0, sizeof(stState));
	memset(&stTorque, 0, sizeof(stTorque));
	memset(&stRef, 0, sizeof(stRef));
	stState.nDof = (int)udof;
	uint32_t uTorqueSent = 0xFFFFFFFF;    // the Torque slot period last written to the drives
	// the drive dropout: the servo-on command of the operator is active, and
	// the joints that have reached Operation enabled since it
	BOOL     bServoReq = FALSE;
	uint32_t uReached = 0;
	// the servo on: its period, done (every joint reached Operation enabled),
	// and SERVO_ON_TIMEOUT_MS in periods (at least one)
	uint32_t uServoReqPer = 0;
	BOOL     bServoDone = FALSE;
	const uint32_t uAll = (udof >= 32) ? 0xFFFFFFFFu : ((1u << udof) - 1u);
	const unsigned long long uTimeoutNs = (unsigned long long)(pRobot->m_nServoOnTimeoutMs > 0 ?
	                                                           pRobot->m_nServoOnTimeoutMs : 1) * 1000000ULL;
	const unsigned long long uPeriodNs = st.tPeriod > 0 ? (unsigned long long)st.tPeriod : 1000000ULL;
	const uint32_t uServoTimeout = (uint32_t)((uTimeoutNs + uPeriodNs - 1ULL) / uPeriodNs);

	// one simulator step with the torques and the brakes of the joints [ns]
	auto stepSim = [&]() -> int32_t
	{
		for (unsigned int i = 0; i < udof; i++)
		{
			adTau[i] = pRobot->m_vSimJoints[i]->GetCommandTorque();
			abReleased[i] = pRobot->m_vSimJoints[i]->IsBrakeReleased();
		}
		const RTTIME t0 = read_timer();
		pSim->Step(adTau, abReleased);
		return Sat32(read_timer() - t0);
	};

	while (TRUE)
	{
		/* release */
		const RTTIME tRelease = pTask->WaitNextRelease();
		const RTTIME tStart = read_timer();
		const uint32_t uPer = (uint32_t)pTask->ReleaseIndex();
		const RTTIME tW = tStart - tRelease;
		const BOOL bLate = (tW > st.tPeriod) ? TRUE : FALSE;
		if (bLate)
			st.nLate++;
		const BOOL bStop = pRobot->CheckStopTask();

		// malloc-count build: allocations of the last cycle of this task
		// (nothing is counted in the normal build)
		{
			static unsigned long s_uCycleStart = 0;
			const unsigned long uNow = malloc_count::Count();
			pRobot->m_uAllocEcat += uNow - s_uCycleStart;
			s_uCycleStart = uNow;
			malloc_count::Arm(true);
		}

		/* receive */
		BOOL   bOp = TRUE;
		BOOL   bWkcErr = FALSE;
		UINT32 uSyncDiff = 0xFFFFFFFF;
		if (pMaster != NULL)
		{
			const BOOL bWkc = pMaster->Receive();
			if (bWkc)
				bBusUp = TRUE;
			else if (bBusUp)
			{
				st.nWkc++;
				bWkcErr = TRUE;
			}
			bOp = (bWkc && pMaster->IsAllSlavesOp() && pMaster->IsMasterOp()) ? TRUE : FALSE;
			if (pMaster->IsDc())
			{
				uSyncDiff = pMaster->GetSyncDiff();
				// before OP the clocks are still being aligned
				if (bOp && uSyncDiff != 0xFFFFFFFF && uSyncDiff > st.uSyncDiffMax)
					st.uSyncDiffMax = uSyncDiff;
			}
		}
		if (pSim != NULL)
		{
			pSim->GetMeasured(adQ, adQd, adTauM);
			for (unsigned int i = 0; i < udof; i++)
				pRobot->m_vSimJoints[i]->SetSimState(adQ[i], adQd[i], adTauM[i]);
			// the drive-dropout test ('n', SIM_FAULT_WHAT=dropout; -s only):
			// the joint leaves Operation enabled
			if (pMaster == NULL && pRobot->m_nSimDropout.load(std::memory_order_relaxed) != 0)
			{
				const int nJ = pRobot->m_nSimDropout.exchange(0, std::memory_order_acq_rel);
				if (nJ >= 1 && nJ <= (int)udof)
					pRobot->m_vSimJoints[nJ - 1]->SimDropout();
			}
			// the E-stop tests ('n', SIM_FAULT_WHAT=estop): the power of every
			// simulated drive, as the status words of this period bring it
			if (pMaster == NULL && pRobot->m_nSimEstopCmd.load(std::memory_order_relaxed) != 0)
			{
				const int nE = pRobot->m_nSimEstopCmd.exchange(0, std::memory_order_acq_rel);
				for (unsigned int i = 0; i < udof; i++)
					pRobot->m_vSimJoints[i]->SetPowerCut(nE == 1 ? TRUE : FALSE, pRobot->m_nSimEstopReports);
			}
		}
		// the end of the receive phase: the inputs of the period are in
		const RTTIME tRecv = read_timer();

		/* the state of this period, for the Trajectory and the Dynamics task */
		BOOL bServo = TRUE;
		uint32_t uOe = 0;           // the joints in Operation enabled
		uint32_t uNotReady = 0;     // the joints in Switch on disabled / Not ready to switch on
		stState.uPeriod = uPer;
		stState.tRelease = tRelease;
		stState.bOp = bOp;
		stState.bWkcErr = bWkcErr;
		stState.bStop = bStop;
		for (unsigned int i = 0; i < udof; i++)
		{
			const CJoint* const pJ = pRobot->m_vJoints[i];
			stState.adQ[i]   = pJ->GetCurrentPos();
			stState.adQd[i]  = pJ->GetCurrentVel();
			stState.adTau[i] = pJ->GetCurrentTor();
			stState.anTauActRaw[i] = (int)pJ->GetActualTorRaw();
			if (!pJ->IsServoOn())
				bServo = FALSE;
			else if (i < 32)
				uOe |= 1u << i;
			// 0x6041, as received
			const uint16_t usSw = (uint16_t)pJ->GetStatusWord();
			stState.ausStatus[i] = usSw;
			if (i < 32 && ((usSw & 0x4F) == 0x40 || (usSw & 0x4F) == 0x00))
				uNotReady |= 1u << i;
		}
		stState.bServoOn = bServo;
		stState.uNotReady = uNotReady;
		stState.stFaults.nLate = st.nLate;
		stState.stFaults.nOverruns = st.nOverruns;
		stState.stFaults.nMissed = st.nMissed;
		stState.stFaults.nWkc = st.nWkc;

		/* the drives: the dropout, the command of the operator, then the
		 * torque of the Dynamics task's last cycle as the target of this
		 * period */
		BOOL bOffNow = FALSE;      // a dropout or a withdrawn servo on in this period
		if (bStop)
		{
			for (CJoint* p : pRobot->m_vJoints)
				p->ServoOff();
			bServoReq = FALSE;
		}
		// The drive dropout (Indy7Ctrl.h): after 'h', a joint that has
		// reached Operation enabled leaves it, and no command came ('j', 'e',
		// a controller fault, the stop): every joint off now, in this period.
		// Also while the bus is not OP: a frame that did not come back leaves
		// the last state in the process image (no false dropout), and a drive
		// that an E-stop took out of OP is caught when its state comes in
		// again, before the enable sequence gets through. Then the servo on
		// that has not completed within SERVO_ON_TIMEOUT_MS: withdrawn, every
		// joint off (see the top of this file).
		else if (bServoReq)
		{
			uReached |= uOe;
			const uint32_t uLost = uReached & ~uOe;
			if (uLost != 0)
			{
				pRobot->DriveDropout(uLost, uPer);
				bServoReq = FALSE;
				uReached = 0;
				bOffNow = TRUE;
			}
			else if (!bServoDone && (uReached & uAll) == uAll)
			{
				bServoDone = TRUE;
				const RTTIME tOn = (RTTIME)(uPer - uServoReqPer) * st.tPeriod;
				pRobot->m_nServoOnDone++;
				if (tOn > pRobot->m_tServoOnMax)
					pRobot->m_tServoOnMax = tOn;
				printf("[ARM] servo on: every joint in Operation enabled %.0f ms after 'h'\n", (double)tOn * 1e-6);
			}
			else if (!bServoDone && uPer - uServoReqPer >= uServoTimeout)
			{
				pRobot->ServoOnExpired(uAll & ~uOe, stState.ausStatus, uPer);
				bServoReq = FALSE;
				uReached = 0;
				bOffNow = TRUE;
			}
		}
		if (!bStop && bOp)
		{
			const int nCmd = pRobot->m_nServoCmd.exchange(eServoCmdNone, std::memory_order_acq_rel);
			if (nCmd == eServoCmdOn && bOffNow)
			{
				// an 'h' of the period before met the dropout or the withdrawal
				// of this one: dropped, the drives stay off
				printf("[ARM] the 'h' of the last period is dropped: the drives went off in this one\n");
			}
			else if (nCmd != eServoCmdNone)
			{
				pRobot->ApplyServoCommand(nCmd);
				if (nCmd == eServoCmdOn && !(bServoReq && bServoDone))
				{
					// a new request ('h' again while one waits restarts its time)
					bServoReq = TRUE;
					bServoDone = FALSE;
					uReached = 0;
					uServoReqPer = uPer;
				}
				else if (nCmd == eServoCmdOff)
					bServoReq = FALSE;
			}
			const bool bTorque = pRobot->m_torqueSlot.Read(stTorque);
			if (bTorque && stTorque.bValid && stTorque.uPeriod != uTorqueSent)
			{
				for (unsigned int i = 0; i < udof; i++)
					pRobot->m_vJoints[i]->MoveTorque(stTorque.adTau[i]);
				uTorqueSent = stTorque.uPeriod;
			}
			else if (bTorque && stTorque.bValid && stTorque.bEnabled && bServo)
			{
				// the controller is on, the drives are on, and no new torque
				// came since the last period: the last target stays in the
				// drives
				pRobot->m_nTorqueStale++;
				if (!bStaleTold)
				{
					printf("[BUS] WARNING: no fresh torque from the Dynamics task (period %u): the last target stays\n",
					       (unsigned)uPer);
					bStaleTold = TRUE;
				}
			}
		}
		for (unsigned int i = 0; i < udof; i++)
			stState.anTauCmdRaw[i] = (int)pRobot->m_vJoints[i]->GetTargetTorRaw();
		pRobot->m_stateSlot.Write(stState);

		/* send */
		int32_t nSim = -1;
		const RTTIME tProc = read_timer();       // the end of the process phase (the slots and the drives)
		if (pMaster != NULL)
			pMaster->Send(tRelease);
		else
			nSim = stepSim();
		const RTTIME tSent = read_timer();
		// -b: the simulated arm moves after the send, as the robot does
		if (bHybrid)
			nSim = stepSim();

		/* after the send */
		if (pMaster != NULL)
		{
			pMaster->UpdateStates();
			if (bOp && !bOpSeen)
			{
				bOpSeen = TRUE;
				pRobot->m_bBusOp.store(true, std::memory_order_release);
				printf("[BUS] OP: %u slaves, working counters complete\n", pMaster->GetSlaveCnt());
			}
		}
		// -b: the drives of the bus are never commanded on; should one report
		// Operation enabled, it is switched off (the flag of the log must stay 0)
		const BOOL bBusServo = bHybrid ? pRobot->IsAnyBusServoOn() : FALSE;
		if (bBusServo)
		{
			for (CIndy7Joint* p : pRobot->m_vDrives)
				p->ServoOff();
			if (!bBusAlarm)
				printf("[BUS] SAFETY: a drive of the bus reports Operation enabled with -b - servo off\n");
			bBusAlarm = TRUE;
		}

		// the newest angles for the live view (wait-free): in simulation the
		// arm now, on the robot the measurement of this period
		if (pSim != NULL)
			pSim->GetCurrentQ(adViewQ);
		else
			for (unsigned int i = 0; i < udof; i++)
				adViewQ[i] = stState.adQ[i];
		pRobot->m_cView.Publish(adViewQ);
		// and for /joint_states of the bridge (wait-free)
		if (pRobot->m_pBridge != NULL)
			pRobot->m_pBridge->PushJointState(adViewQ, stState.adQd, (int)udof);

		/* the LED of the end tool: the mode of the controller (Torque slot),
		 * blinking while the arm moves */
		if (!bStop && bOp && pTool != NULL)
		{
			const BOOL bBlink = pRobot->IsMoving();
			switch (stTorque.eCtrlMode)
			{
				case CControllerFullDynamicsRT::eGravityCompensation:
					pTool->LedGreen(bBlink);
					break;
				case CControllerFullDynamicsRT::eTrack:
				case CControllerFullDynamicsRT::eHome:
					pTool->LedBlue(bBlink);
					break;
				case CControllerFullDynamicsRT::eJointTest:
					pTool->LedYellow(bBlink);
					break;
				default:
					pTool->LedRed(bBlink);
					break;
			}
		}

		/* timing (eq. (3)) */
		const RTTIME tEnd = read_timer();
		const RTTIME tS = tSent - tStart, tE = tEnd - tStart;
		st.nCycles++;
		st.dWSum += (double)tW;
		st.dSSum += (double)tS;
		st.dESum += (double)tE;
		if (tW > st.tWMax) st.tWMax = tW;
		if (tS > st.tSMax) st.tSMax = tS;
		if (tE > st.tEMax) st.tEMax = tE;
		const BOOL bMissed = (tW + tS > st.tDeadline) ? TRUE : FALSE;
		const BOOL bOverrun = (tW + tE > st.tPeriod) ? TRUE : FALSE;
		if (bMissed)
			st.nMissed++;
		if (bOverrun)
			st.nOverruns++;

		// -t: the record of this period (no lock, no allocation). The fields
		// of the Trajectory and the Dynamics task are their cycle of the
		// period before, the last one complete now
		if (cLog.IsOn())
		{
			pRobot->m_refSlot.Read(stRef);
			const BOOL bTrajFresh = (stRef.uPeriod + 1 == uPer) ? TRUE : FALSE;
			const BOOL bDynFresh = (stTorque.uPeriod + 1 == uPer) ? TRUE : FALSE;
			const int32_t nDyn = (bDynFresh && stTorque.bValid) ? stTorque.nDynNs : -1;
			const int32_t nHl = (bTrajFresh && stRef.eMode != eRefNone) ? stRef.nHlNs : -1;
			const BOOL bTrack = (bTrajFresh && stRef.eMode == eRefTrack) ? TRUE : FALSE;
			const BOOL bTest = (bTrajFresh && stRef.bTest) || (bDynFresh && stTorque.bJtRunning);
			TL::Record stR;
			stR.uPeriod = uPer;
			stR.nW = Sat32(tW);
			stR.nS = Sat32(tS);
			stR.nE = Sat32(tE);
			stR.nDyn = nDyn;
			stR.nHl = nHl;
			stR.nSim = nSim;
			stR.uSyncDiff = uSyncDiff;
			stR.usFlags = (uint16_t)(usMode | (bOp ? TL::TF_OP : 0) | (bWkcErr ? TL::TF_WKC : 0) |
			                         (bLate ? TL::TF_LATE : 0) |
			                         (bMissed ? TL::TF_MISSED : 0) | (bOverrun ? TL::TF_OVERRUN : 0) |
			                         (nDyn >= 0 ? TL::TF_LAW : 0) | (nHl >= 0 ? TL::TF_HL : 0) |
			                         (bTrack ? TL::TF_TRACK : 0) | (bStop ? TL::TF_STOP : 0) |
			                         (bServo ? TL::TF_SERVO : 0) | (bBusServo ? TL::TF_BUSSERVO : 0) |
			                         (bTest ? TL::TF_TEST : 0));
			stR.usReserved = 0;
			stR.nRecv = Sat32(tRecv - tStart);
			stR.nSend = Sat32(tSent - tProc);
			stR.nTrajW = bTrajFresh ? stRef.nW : -1;
			stR.nTrajE = bTrajFresh ? stRef.nE : -1;
			stR.nDynW = bDynFresh ? stTorque.nW : -1;
			stR.nDynE = bDynFresh ? stTorque.nE : -1;
			cLog.Push(stR);
		}

		if (bStop && ++nCleanUp > 1000)
			break;
	}
	// release the master right after the last exchange: its deactivation
	// takes the slaves from OP to PREOP while the master's own thread keeps
	// the bus running. Released later, the drives would stay in OP without
	// process data, and their sync-manager watchdog would expire.
	if (pMaster != NULL)
	{
		const RTTIME tLast = read_timer();
		pMaster->DeInit();
		printf("[BUS] master released %.1f ms after the last cycle: the slaves go to PREOP\n",
		       (double)(read_timer() - tLast) * 1e-6);
	}
	// the two other tasks end when they see this flag; their timing is
	// complete once they have, so the summary waits for them
	pRobot->m_bEcatDone.store(true, std::memory_order_release);
	for (int nWait = 0; nWait < 500 && !(pRobot->m_bTrajDone.load() && pRobot->m_bDynDone.load()); nWait++)
		usleep(1000);
	if (pSim != NULL)
		pSim->PrintSummary();
	pRobot->PrintRtSummary();
	DBG_LOG_INFO("(proc_ethercat_control) task ended");
}
