/*****************************************************************************
*	Name: RtTask.h
*	Description: A task of the application on RT-POSIX (third_party/rt-posix,
*	used as it is): SCHED_FIFO with an explicit priority, pinned to one CPU
*	(or, without a CPU, on the CPUs of the process), and for a periodic task
*	an absolute-time release grid.
*
*	The periodic tasks of the controller share one grid: Create() gets the
*	same origin for all of them, and release k of a task (k = 0, 1, ...) is
*	    t_k = origin + START_DELAY + (k + 1) T,
*	which is where RT-POSIX's wait_next_period() wakes the task (its first
*	deadline is start + T). WaitNextRelease() returns t_k, computed here from
*	the grid, so that the timing of every cycle (release-to-start delay t_w,
*	late release, overrun) is measured from the planned release and not from
*	the time of the wake-up. The return value of wait_next_period() is not
*	used, and a late release is not skipped: the cycle runs late and the
*	next releases stay on the grid.
*
*	In the new thread, before the task function: the scheduling policy, the
*	priority and the CPU affinity are read back and compared with the request
*	(the result is printed, and the application refuses to go on if they
*	differ), the stack is prefaulted, and SIGINT and SIGTERM are blocked, so
*	that main() handles them.
*****************************************************************************/
#ifndef G2T_RT_TASK_H
#define G2T_RT_TASK_H

#include <atomic>
#include <cstdint>

#include "Common.h"

class CRtTask
{
public:
	struct Spec
	{
		TSTRING	strName;
		int		nPriority;		// SCHED_FIFO 1..99
		int		nCpu;			// -1: the CPUs of the process
		RTTIME	tPeriod;		// [ns], 0: not periodic
		RTTIME	tStartDelay;	// [ns] after the common origin
	};
	enum eCheck { eCheckPending = 0, eCheckOk = 1, eCheckFailed = 2 };

	CRtTask();

	/* before Start(), in main(): the task (create_rt_task, set_cpu_affinity,
	 * set_task_period with atOrigin + START_DELAY) */
	BOOL	Create			(const Spec& astSpec, RTTIME atOrigin);
	BOOL	Start			(PTASKFCN apFcn, void* apArg);
	/* the read-back of the new thread: eCheckPending until it has run */
	eCheck	GetCheck		(	) const { return (eCheck)m_nCheck.load(); }
	const Spec& GetSpec		(	) const { return m_stSpec; }
	BOOL	IsStarted		(	) const { return m_bStarted; }

	/* in the task: the task object of the calling thread */
	static CRtTask*	Self	(	);
	/* the next release: waits for it and returns its planned time [ns] */
	RTTIME	WaitNextRelease	(	);
	RTTIME	Period			(	) const { return m_stSpec.tPeriod; }
	/* the index k of the current release (after the first WaitNextRelease) */
	uint64_t ReleaseIndex	(	) const { return m_uRelease; }

	/* a common origin for the grids of the tasks created after it: the
	 * first releases come atLeadNs after the call, so that every task has
	 * started (and prefaulted its stack) before its first release */
	static RTTIME	GridOrigin	(RTTIME atLeadNs) { return read_timer() + atLeadNs; }

private:
	static void		Entry		(void* apArg);
	BOOL			Verify		(	);

	POSIX_TASK		m_stTask;
	Spec			m_stSpec;
	char			m_acName[MAX_NAME_LENGTH];
	RTTIME			m_tFirst;		// release 0 [ns]
	uint64_t		m_uRelease;		// index of the current release
	BOOL			m_bWaited;		// WaitNextRelease() was called once
	BOOL			m_bCreated, m_bStarted;
	PTASKFCN		m_pFcn;
	void*			m_pArg;
	std::atomic<int> m_nCheck;
	cpu_set_t		m_stProcessCpus;	// the CPUs of the process at Create()
};

#endif // G2T_RT_TASK_H
