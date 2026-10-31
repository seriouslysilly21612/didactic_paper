/*****************************************************************************
*	Name: TaskSlots.h
*	Description: The data that the three periodic tasks of the controller
*	exchange (paper, Sec. II and Fig. 1).
*
*	The tasks share one release grid (cfg [RT_TASKS]) and, within a period,
*	run in the order of their priorities:
*
*	    EtherCAT task    receive the frame            -> State slot
*	    (TASK0, 97)      Torque slot (last period)    -> send
*	    Trajectory task  State slot, goal slot        -> high-level controller
*	    (TASK1, 96)                                   -> Reference slot
*	    Dynamics task    State slot + Reference slot  -> control law
*	    (TASK2, 95)                                   -> Torque slot
*
*	so the torque that the EtherCAT task sends in period k+1 was computed by
*	the Dynamics task in period k from the state received in period k (send
*	before compute, Fig. 2). The simulation (-s, -b) is stepped by the
*	EtherCAT task in place of the exchange and keeps this delay.
*
*	Every slot is a CAtomicSlot (AtomicSlot.h, eq. (1)) with one writer and
*	carries the period identifier of the EtherCAT task (uPeriod), which every
*	reader checks before it uses the snapshot. The operator commands are read
*	by the Dynamics task (DoInput), which owns the controller; what belongs
*	to another task goes through a command word, an atomic that the reader
*	exchanges with 0: the drives to the EtherCAT task (eServoCmd), the
*	high-level motions to the Trajectory task (HL_CMD_*).
*****************************************************************************/
#ifndef G2T_TASK_SLOTS_H
#define G2T_TASK_SLOTS_H

#include <cstdint>

#include "base/Common.h"
#include "base/AtomicSlot.h"

namespace task_slots
{
	enum { MAX_DOF = 8 };

	/* the timing faults of the EtherCAT task so far (the test logs report
	 * the counts during a test as differences) */
	struct ST_FAULTS
	{
		long	nLate;			// releases that started more than a period late
		long	nOverruns;		// cycles that ended after the next release
		long	nMissed;		// sends that returned after the send deadline t_b
		long	nWkc;			// incomplete working counters after the bus was up once
	};

	/* EtherCAT task -> Trajectory and Dynamics tasks: what the frame of this
	 * period brought (in simulation: what the simulator returned) */
	struct ST_STATE_SLOT
	{
		uint32_t	uPeriod;		// release index of the EtherCAT task
		RTTIME		tRelease;		// its planned release [ns]
		int			nDof;
		BOOL		bOp;			// bus OP and both working counters complete (simulation: TRUE)
		BOOL		bWkcErr;		// incomplete working counter in this period (after the bus was up)
		BOOL		bServoOn;		// every joint in Operation enabled
		uint32_t	uNotReady;		// bit i: joint i in Switch on disabled or Not ready to switch on,
									// although Shutdown is written while it is off (an E-stop, or an
									// inhibit of the drive): 'h' is refused
		uint16_t	ausStatus[MAX_DOF];	// 0x6041 of this period
		BOOL		bStop;			// the application is stopping (the drives are being switched off)
		double		adQ[MAX_DOF];	// measured joint angles [rad]
		double		adQd[MAX_DOF];	// [rad/s]
		double		adTau[MAX_DOF];	// torque the drives report [N m] (simulation: the applied torque)
		int			anTauActRaw[MAX_DOF];	// 0x6077 of this period, drive units (0 in simulation)
		int			anTauCmdRaw[MAX_DOF];	// 0x6071 written in this period, drive units (0 in simulation)
		ST_FAULTS	stFaults;		// the faults of the EtherCAT task so far
	};

	/* Trajectory task -> Dynamics task: the joint reference of this period */
	enum eRefMode { eRefNone = 0, eRefTrack = 1, eRefHome = 2 };
	struct ST_REF_SLOT
	{
		uint32_t	uPeriod;		// the State slot period it was computed from
		int			eMode;			// eRefMode: eRefNone -> the Dynamics task holds in gravity compensation
		BOOL		bFrictionFF;	// the friction feedforward acts in this motion
		BOOL		bTest;			// a high-level test or an operator motion is running (timing log)
		int			nHlTest;		// the test (1..8 of the menu), 0: none
		double		adQRef[MAX_DOF], adQdRef[MAX_DOF], adQddRef[MAX_DOF];
		int32_t		nHlNs;			// time of the high-level step [ns] (-1: it did not run)
		int32_t		nW, nE;			// t_w, t_e of the Trajectory task's cycle [ns]
	};

	/* Dynamics task -> EtherCAT task (the torque) and Trajectory task (the
	 * state of the controller) */
	struct ST_TORQUE_SLOT
	{
		uint32_t	uPeriod;		// the State slot period it was computed from
		BOOL		bValid;			// the control law ran: adTau is the command of this period
		double		adTau[MAX_DOF];	// [N m], after the torque limit
		unsigned int uClamp;		// bit i: the torque limit acted on joint i
		int			eCtrlMode;		// CControllerFullDynamicsRT::eControlMode
		BOOL		bEnabled;		// the controller is enabled ('r')
		BOOL		bJtRunning;		// a JOINT TEST is running
		int32_t		nDynNs;			// RBDL calls of the law [ns] (-1: the law did not run)
		int32_t		nW, nE;			// t_w, t_e of the Dynamics task's cycle [ns]
		unsigned long uAllocUpdate;	// malloc-count build: allocations of the law so far
	};

	/* Dynamics task (DoInput) -> EtherCAT task: the drives */
	enum eServoCmd { eServoCmdNone = 0, eServoCmdOn = 1, eServoCmdOff = 2, eServoCmdStop = 3 };

	/* Dynamics task (DoInput) -> Trajectory task: the high-level motions.
	 * HL_CMD_STOP ends TRACK or HOME (and the test); 1..8 start menu entry n */
	enum { HL_CMD_NONE = 0, HL_CMD_STOP = -1 };

	/* keyboard task -> Trajectory task: a motion of the operator, checked by
	 * the keyboard task against the joint limits ('m': joint angles [rad])
	 * or the reach of the IK ('t': a position of the tool in the base frame
	 * [m]); the Trajectory task tells a new request by the count of the slot */
	enum eMoveKind { eMoveNone = 0, eMoveJoint = 1, eMoveTask = 2 };
	struct ST_MOVE_REQ
	{
		int			eKind;			// eMoveKind
		double		adTarget[MAX_DOF];
	};

	/* a time [ns] into an int32 field of the logs */
	inline int32_t Sat32(RTTIME at) { return at > 0x7FFFFFFFULL ? 0x7FFFFFFF : (int32_t)at; }

	/* the timing of one task, kept by the task itself */
	struct ST_TASK_TIMING
	{
		RTTIME	tPeriod;
		long	nCycles;
		long	nLate;				// releases that started more than a period late
		long	nOverruns;			// cycles that ended after the next release
		RTTIME	tWMax, tEMax;		// [ns]
		double	dWSum, dESum;		// [ns]
	};
}

#endif // G2T_TASK_SLOTS_H
