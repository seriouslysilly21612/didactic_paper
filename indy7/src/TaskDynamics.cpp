/*****************************************************************************
*	Name: TaskDynamics.cpp
*	Description: The Dynamics task (TASK2, paper, Sec. IV-A). In every
*	period, after the release:
*	  read     the State slot of the EtherCAT task (the measurement of this
*	           period) and the Reference slot of the Trajectory task (the
*	           joint reference of this period, or none);
*	  mode     TRACK or HOME while the Reference slot carries a reference of
*	           this period or the one before, gravity compensation when it
*	           carries none, has gone stale, or after a stop until the
*	           Trajectory task has acknowledged it; a JOINT TEST keeps its own
*	           mode, and a reference that arrives during one is refused (the
*	           motion is stopped);
*	  law      if the state is valid and the controller is enabled: the
*	           control law (gravity compensation with the hold, CTC on the
*	           reference, eq. (5), or the JOINT TEST), the torque limit; the
*	           JOINT TEST log. A law that fails (a non-finite measured state
*	           or torque) gives zero torque and latches a controller fault:
*	           the motion ends, the drives go off in the next period, and they
*	           stay off until the operator switches the controller off and on
*	           ('r', 'r', 'h'); a drive dropout that the EtherCAT task found in
*	           this period, or a servo on that it withdrew (every joint off),
*	           ends the motion here, as 'e' does;
*	  publish  the Torque slot: the torque of this period, which the
*	           EtherCAT task sends in the next one, and the state of the
*	           controller;
*	  then     the operator commands (DoInput), off the path of the torque,
*	           and the Torque slot again with the state they changed.
*	It ends when the EtherCAT task has ended (after the stop).
*****************************************************************************/
#include "Indy7Ctrl.h"
#include "base/MallocCount.h"
#include "CallTiming.h"

#include <cstring>
#include <limits>

void
proc_dynamics_control(void* apRobot)
{
	using namespace task_slots;
	typedef CControllerFullDynamicsRT C;
	auto* pRobot = static_cast<CRobotIndy7*>(apRobot);
	CRtTask* const pTask = CRtTask::Self();
	const unsigned int udof = (unsigned int)pRobot->GetTotalAxis();
	C* const pCtl = pRobot->GetController();
	DBG_LOG_INFO("(proc_dynamics_control) Dynamics task started (controller %s)",
	             pCtl != NULL ? "available; 'r' enables it" : "NOT available - no torque");
	if (pTask == NULL || udof == 0 || udof > (unsigned int)MAX_DOF)
	{
		DBG_LOG_ERROR("(proc_dynamics_control) %u joints", udof);
		pRobot->m_bDynDone.store(true, std::memory_order_release);
		return;
	}

	ST_TASK_TIMING& st = pRobot->m_stDynTiming;
	st.tPeriod = pTask->Period();
	ST_STATE_SLOT& stState = pRobot->m_stDynState;      // DoInput reads these copies
	ST_REF_SLOT&   stRef = pRobot->m_stDynRef;
	ST_TORQUE_SLOT stTorque;
	memset(&stTorque, 0, sizeof(stTorque));
	C::Reference stReference;
	memset(&stReference, 0, sizeof(stReference));
	uint32_t uLastPeriod = 0xFFFFFFFF;
	BOOL bRefusedTold = FALSE;

	while (TRUE)
	{
		/* release */
		const RTTIME tRelease = pTask->WaitNextRelease();
		const RTTIME tStart = read_timer();
		if (tStart - tRelease > st.tPeriod)
			st.nLate++;
		// the EtherCAT task has ended (1 s after the stop): so does this one
		if (pRobot->m_bEcatDone.load(std::memory_order_acquire))
			break;

		// malloc-count build: allocations of the last cycle of this task
		// (nothing is counted in the normal build)
		{
			static unsigned long s_uCycleStart = 0;
			const unsigned long uNow = malloc_count::Count();
			pRobot->m_uAllocCycle += uNow - s_uCycleStart;
			s_uCycleStart = uNow;
			malloc_count::Arm(true);
		}

		/* read */
		const bool bNewState = pRobot->m_stateSlot.Read(stState) && stState.uPeriod != uLastPeriod;
		pRobot->m_refSlot.Read(stRef);
		int32_t nDynNs = -1;
		BOOL bValid = FALSE;
		unsigned int uClamp = 0;

		if (bNewState)
		{
			uLastPeriod = stState.uPeriod;
			// a drive dropout or a withdrawn servo on in this period
			// (TaskEtherCAT.cpp): the motion ends (a JOINT TEST here, TRACK or
			// HOME in the Trajectory task), as with 'e'; the drives are off
			// already
			if (pRobot->m_nDriveStop.load(std::memory_order_relaxed) != 0 &&
			    pRobot->m_nDriveStop.exchange(0, std::memory_order_acq_rel) != 0)
				pRobot->StopMotion();
			if (stState.bOp && !stState.bStop)
			{
				// the measurement of this period, in every cycle, also while
				// the controller is off: the tests and the hand-eye capture
				// use the measured posture
				for (unsigned int i = 0; i < udof; i++)
				{
					pRobot->m_vCurrentPos[i] = stState.adQ[i];
					pRobot->m_vCurrentVel[i] = stState.adQd[i];
					pRobot->m_vCurrentTor[i] = stState.adTau[i];
				}

				/* the mode: the reference of the Trajectory task, if any */
				if (pCtl != NULL)
				{
					// after a stop: nothing until the Trajectory task has acknowledged
					if (pRobot->m_bRefInhibit && stRef.eMode == eRefNone)
						pRobot->m_bRefInhibit = FALSE;
					const BOOL bHasRef = (stRef.eMode == eRefTrack || stRef.eMode == eRefHome) ? TRUE : FALSE;
					// of this period, or of the one before
					const BOOL bFresh = (stRef.uPeriod == stState.uPeriod || stRef.uPeriod + 1 == stState.uPeriod)
					                        ? TRUE : FALSE;
					const BOOL bJt = pCtl->IsJointTestRunning();
					BOOL bFollow = (bHasRef && bFresh && !pRobot->m_bRefInhibit && !bJt) ? TRUE : FALSE;
					if (bHasRef && !bFresh && !pRobot->m_bRefInhibit)
						pRobot->m_nRefStale++;
					if (bHasRef && bJt && !pRobot->m_bRefInhibit)
					{
						// a reference during a JOINT TEST: the test keeps the arm,
						// the motion of the Trajectory task is stopped
						pRobot->m_nRefRefused++;
						pRobot->m_nHlCmd.store(HL_CMD_STOP, std::memory_order_release);
						pRobot->m_bRefInhibit = TRUE;
						if (!bRefusedTold)
						{
							printf("[DYN] a reference arrived during a JOINT TEST: TRACK/HOME stopped\n");
							bRefusedTold = TRUE;
						}
					}
					const C::eControlMode eNow = pCtl->GetControlMode();
					if (bFollow)
					{
						const C::eControlMode eWant = (stRef.eMode == eRefHome) ? C::eHome : C::eTrack;
						if (eNow != eWant)
						{
							// from gravity compensation only (a JOINT TEST was excluded above)
							if (eNow == C::eGravityCompensation || eNow == C::eTrack || eNow == C::eHome)
								pRobot->SetControllerMode(eWant);
							else
								bFollow = FALSE;
						}
						if (bFollow)
						{
							for (unsigned int i = 0; i < udof; i++)
							{
								stReference.adQ[i]   = stRef.adQRef[i];
								stReference.adQd[i]  = stRef.adQdRef[i];
								stReference.adQdd[i] = stRef.adQddRef[i];
							}
							stReference.bFrictionFF = stRef.bFrictionFF;
							pCtl->SetReference(stReference);
						}
					}
					else if (eNow == C::eTrack || eNow == C::eHome)
					{
						// the reference is gone (the end of the motion, a stop,
						// or stale): gravity compensation
						pRobot->SetControllerMode(C::eGravityCompensation);
					}
				}

				/* the law */
				const BOOL bCtrlOn = (pCtl != NULL && pCtl->IsEnabled()) ? TRUE : FALSE;
				if (bCtrlOn)
				{
					// fault injection ('n', SIM_FAULT_WHAT=state; simulation
					// only): the measured angle of the joint is not finite in
					// this cycle (the next copy of the state restores it)
					if (pRobot->m_nStateInject != 0 && pRobot->m_nSimFaultJoint >= 0 &&
					    pRobot->m_nSimFaultJoint < (int)udof)
					{
						pRobot->m_vCurrentPos[pRobot->m_nSimFaultJoint] = std::numeric_limits<double>::quiet_NaN();
						if (pRobot->m_nStateInject == 1)
							pRobot->m_nStateInject = 0;
					}
					// the malloc-count build counts the allocations of the
					// control law, which must stay at zero
					const unsigned long uAlloc0 = malloc_count::Count();
					const RTTIME tCt = call_timing::Now();
					const BOOL bUpdated = pCtl->Update(pRobot->m_vCurrentPos, pRobot->m_vCurrentVel,
					                                   pRobot->m_vCurrentTor, pRobot->m_vOutputTorque);
					call_timing::Lap(call_timing::eControl, tCt);
					const unsigned long uAlloc = malloc_count::Count() - uAlloc0;
					pRobot->m_uAllocUpdate += uAlloc;
					pRobot->m_uAllocCycles++;
					if (uAlloc > pRobot->m_uAllocUpdateMaxCycle)
						pRobot->m_uAllocUpdateMaxCycle = uAlloc;
					nDynNs = Sat32(pCtl->GetDynamicsNs());
					uClamp = pCtl->GetLastClamp();
					if (!bUpdated)
					{
						// The controller failed: zero torque. Update() fails
						// only on a non-finite measured state or torque
						// (FullDynControllerRT.cpp; its other FALSE cases
						// cannot occur here: the controller is initialized,
						// enabled, and the vectors have its size). A controller
						// fault, latched once: the motion ends and the drives go
						// off in the next period, as with 'e' (the brakes
						// engage); 'h' is refused until 'r', 'r'. While it stays
						// latched, a cycle that fails again sends zero torque
						// and repeats nothing (the controller logs one failed
						// cycle in 1000).
						for (unsigned int i = 0; i < udof; i++)
							pRobot->m_vOutputTorque[i] = 0.0;
						pRobot->m_nCtrlFaultCycles++;
						if (!pRobot->m_bCtrlFault.load(std::memory_order_relaxed))
						{
							pRobot->m_bCtrlFault.store(true, std::memory_order_release);
							if (pRobot->m_nCtrlFaults.fetch_add(1, std::memory_order_acq_rel) == 0)
								pRobot->m_uCtrlFaultFirst = stState.uPeriod;
							pRobot->m_uCtrlFaultLast = stState.uPeriod;
							pRobot->StopMotion();
							pRobot->m_nServoCmd.store(eServoCmdOff, std::memory_order_release);
							DBG_LOG_ERROR(">>> CONTROLLER FAULT (non-finite state or torque): motion stopped, "
							              "SERVO OFF (brakes) - clear with 'r', 'r', then 'h' (period %u)",
							              (unsigned)stState.uPeriod);
						}
					}
					bValid = TRUE;
				}

				// JOINT TEST log: the sample of every cycle of the test, then
				// the end of the test (at rest, timeout, or a stop) closes it
				if (pRobot->m_cJtLog.IsOpen() && pCtl != NULL)
				{
					if (bCtrlOn && pCtl->HasJointTestSample())
					{
						// with what the drives report in this period (0x6077)
						// and the target written in this period (0x6071, the
						// torque of the last cycle of this task); in the
						// simulation the applied torque
						C::JointTestSample stS = pCtl->GetJointTestSample();
						for (int i = 0; i < (int)udof && i < C::JT_MAX_DOF; ++i)
						{
							stS.adTauAct[i] = stState.adTau[i];
							stS.anTauActRaw[i] = stState.anTauActRaw[i];
							stS.anTauCmdRaw[i] = stState.anTauCmdRaw[i];
						}
						pRobot->m_cJtLog.Push(stS);
					}
					const C::eJointTestState eJt = pCtl->GetJointTestState();
					if (eJt != C::eJtRunning)
					{
						JointTestFormat::Result stR;
						memcpy(stR.adQ0, pCtl->GetJointTestQ0(), sizeof(stR.adQ0));
						memcpy(stR.adD, pCtl->GetJointTestD(), sizeof(stR.adD));
						pRobot->FillRtResult(stR, stState.stFaults, pRobot->m_stJtRt0, pRobot->m_nJtCtrlFaults0,
						                     pRobot->m_nJtDropouts0);
						stR.nAllocs = malloc_count::Enabled() ? (long)(pRobot->m_uAllocUpdate - pRobot->m_uJtAllocs0) : -1;
						pRobot->m_cJtLog.Close((int)eJt, stR);
						DBG_LOG_INFO("[JT] %s - gravity compensation",
						             eJt == C::eJtDoneAtRest ? "done, the arm is at rest" :
						             eJt == C::eJtDoneTimeout ? "done, NOT at rest after 5 s" :
						             "stopped");
					}
				}
			}
		}

		/* publish: the torque of this period (sent by the EtherCAT task in the
		 * next one) and the state of the controller. Without a new state
		 * nothing is written: the torque of the last state stays in the slot
		 * for the EtherCAT task. */
		const RTTIME tEnd0 = read_timer();
		if (bNewState)
		{
			stTorque.uPeriod = stState.uPeriod;
			stTorque.bValid = bValid;
			for (unsigned int i = 0; i < udof; i++)
				stTorque.adTau[i] = bValid ? pRobot->m_vOutputTorque[i] : 0.0;
			stTorque.uClamp = uClamp;
			stTorque.eCtrlMode = (pCtl != NULL) ? (int)pCtl->GetControlMode() : (int)C::eGravityCompensation;
			stTorque.bEnabled = pRobot->m_bRTControllerEnabled;
			stTorque.bJtRunning = (pCtl != NULL && pCtl->IsJointTestRunning()) ? TRUE : FALSE;
			stTorque.nDynNs = nDynNs;
			stTorque.nW = Sat32(tStart - tRelease);
			stTorque.nE = Sat32(tEnd0 - tStart);
			stTorque.uAllocUpdate = pRobot->m_uAllocUpdate;
			pRobot->m_torqueSlot.Write(stTorque);
		}

		/* then: the operator commands, after the torque is out */
		if (bNewState && stState.bOp && !stState.bStop && pRobot->m_cKeyPress.load() != ' ')
		{
			pRobot->DoInput();
			// the state of the controller as the command left it, for the
			// Trajectory task of the next period (the torque is unchanged)
			stTorque.eCtrlMode = (pCtl != NULL) ? (int)pCtl->GetControlMode() : (int)C::eGravityCompensation;
			stTorque.bEnabled = pRobot->m_bRTControllerEnabled;
			stTorque.bJtRunning = (pCtl != NULL && pCtl->IsJointTestRunning()) ? TRUE : FALSE;
			pRobot->m_torqueSlot.Write(stTorque);
		}

		/* the timing of this task */
		const RTTIME tEnd = read_timer();
		const RTTIME tW = tStart - tRelease, tE = tEnd - tStart;
		st.nCycles++;
		st.dWSum += (double)tW;
		st.dESum += (double)tE;
		if (tW > st.tWMax) st.tWMax = tW;
		if (tE > st.tEMax) st.tEMax = tE;
		if (tW + tE > st.tPeriod)
			st.nOverruns++;
	}
	DBG_LOG_INFO("(proc_dynamics_control) task ended");
	pRobot->m_bDynDone.store(true, std::memory_order_release);
}
