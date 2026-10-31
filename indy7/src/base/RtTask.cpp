/*****************************************************************************
*	Name: RtTask.cpp
*	Description: A task of the application on RT-POSIX. See RtTask.h.
*****************************************************************************/
#include "RtTask.h"

#include <alloca.h>
#include <cerrno>
#include <csignal>
#include <pthread.h>
#include <sched.h>

namespace {

const int TASK_STACK = 2 * 1024 * 1024;		// [bytes]
thread_local CRtTask* t_pSelf = NULL;

void NsToTimespec(RTTIME atNs, struct timespec* apTs)
{
	apTs->tv_sec  = (time_t)(atNs / 1000000000ULL);
	apTs->tv_nsec = (long)(atNs % 1000000000ULL);
}

/* "6" or "0-5,8-15" */
void CpuList(const cpu_set_t& astSet, char* apcOut, size_t auLen)
{
	int nLen = 0;
	apcOut[0] = '\0';
	for (int nCpu = 0; nCpu < CPU_SETSIZE && nLen < (int)auLen - 12; nCpu++)
	{
		if (!CPU_ISSET(nCpu, &astSet))
			continue;
		int nLast = nCpu;
		while (nLast + 1 < CPU_SETSIZE && CPU_ISSET(nLast + 1, &astSet))
			nLast++;
		nLen += snprintf(apcOut + nLen, auLen - (size_t)nLen, nLast > nCpu ? "%s%d-%d" : "%s%d",
		                 nLen > 0 ? "," : "", nCpu, nLast);
		nCpu = nLast;
	}
}

/* touch the stack once, so that the task never takes a page fault on it */
void __attribute__((noinline)) PrefaultStack(size_t auBytes)
{
	volatile unsigned char* pBuf = (volatile unsigned char*)alloca(auBytes);
	const long nPage = sysconf(_SC_PAGESIZE) > 0 ? sysconf(_SC_PAGESIZE) : 4096;
	for (size_t i = 0; i < auBytes; i += (size_t)nPage)
		pBuf[i] = 0;
}

} // namespace

CRtTask::CRtTask()
	: m_tFirst(0), m_uRelease(0), m_bWaited(FALSE), m_bCreated(FALSE), m_bStarted(FALSE),
	  m_pFcn(NULL), m_pArg(NULL), m_nCheck(eCheckPending)
{
	memset(&m_stTask, 0, sizeof(m_stTask));
	memset(m_acName, 0, sizeof(m_acName));
	m_stSpec.nPriority = 0;
	m_stSpec.nCpu = -1;
	m_stSpec.tPeriod = 0;
	m_stSpec.tStartDelay = 0;
	CPU_ZERO(&m_stProcessCpus);
}

BOOL
CRtTask::Create(const Spec& astSpec, RTTIME atOrigin)
{
	m_stSpec = astSpec;
	snprintf(m_acName, sizeof(m_acName), "%s", astSpec.strName.empty() ? "task" : astSpec.strName.c_str());
	if (sched_getaffinity(0, sizeof(m_stProcessCpus), &m_stProcessCpus) != 0)
	{
		DBG_LOG_ERROR("(CRtTask) '%s': sched_getaffinity: %s", m_acName, strerror(errno));
		return FALSE;
	}

	if (RET_SUCC != create_rt_task(&m_stTask, m_acName, TASK_STACK, astSpec.nPriority))
	{
		DBG_LOG_ERROR("(CRtTask) '%s': cannot create the task (SCHED_FIFO %d; priority 1..99, ulimit -r)",
		              m_acName, astSpec.nPriority);
		return FALSE;
	}
	if (astSpec.nCpu >= 0)
	{
		if (RET_SUCC != set_cpu_affinity(&m_stTask, astSpec.nCpu))
		{
			DBG_LOG_ERROR("(CRtTask) '%s': CPU %d cannot be set (is it online?)", m_acName, astSpec.nCpu);
			return FALSE;
		}
	}
	else
	{
		/* RT-POSIX puts a new task on CPU 0; without a CPU in the cfg the
		 * task runs on the CPUs of the process (the non-isolated cores) */
		m_stTask.stCpuAffinity = m_stProcessCpus;
		const int nRet = pthread_attr_setaffinity_np(&m_stTask.stThreadAttr, sizeof(m_stProcessCpus),
		                                             &m_stProcessCpus);
		if (nRet != 0)
		{
			DBG_LOG_ERROR("(CRtTask) '%s': affinity of the process: %s", m_acName, strerror(nRet));
			return FALSE;
		}
	}
	if (astSpec.tPeriod > 0)
	{
		const RTTIME tStart = atOrigin + astSpec.tStartDelay;
		if (RET_SUCC != set_task_period(&m_stTask, tStart, astSpec.tPeriod))
		{
			DBG_LOG_ERROR("(CRtTask) '%s': cannot set the period", m_acName);
			return FALSE;
		}
		m_tFirst = tStart + astSpec.tPeriod;		// the first deadline of RT-POSIX
	}
	m_bCreated = TRUE;
	return TRUE;
}

BOOL
CRtTask::Start(PTASKFCN apFcn, void* apArg)
{
	if (!m_bCreated || m_bStarted || apFcn == NULL)
		return FALSE;
	m_pFcn = apFcn;
	m_pArg = apArg;
	m_nCheck.store(eCheckPending);
	if (RET_SUCC != start_task(&m_stTask, &CRtTask::Entry, this))
	{
		DBG_LOG_ERROR("(CRtTask) '%s': cannot start the task (SCHED_FIFO %d, CPU %d)", m_acName,
		              m_stSpec.nPriority, m_stSpec.nCpu);
		return FALSE;
	}
	m_bStarted = TRUE;
	return TRUE;
}

CRtTask*
CRtTask::Self()
{
	return t_pSelf;
}

void
CRtTask::Entry(void* apArg)
{
	CRtTask* pTask = static_cast<CRtTask*>(apArg);
	t_pSelf = pTask;

	/* the signals of the operator go to main() */
	sigset_t stSet;
	sigemptyset(&stSet);
	sigaddset(&stSet, SIGINT);
	sigaddset(&stSet, SIGTERM);
	pthread_sigmask(SIG_BLOCK, &stSet, NULL);

	pTask->m_nCheck.store(pTask->Verify() ? eCheckOk : eCheckFailed);
	PrefaultStack((size_t)TASK_STACK / 2);

	pTask->m_pFcn(pTask->m_pArg);
}

/* policy, priority and affinity of the calling thread, read back */
BOOL
CRtTask::Verify()
{
	int nPolicy = -1;
	struct sched_param stParam;
	memset(&stParam, 0, sizeof(stParam));
	cpu_set_t stCpus;
	CPU_ZERO(&stCpus);
	const int nSched = pthread_getschedparam(pthread_self(), &nPolicy, &stParam);
	const int nAff = pthread_getaffinity_np(pthread_self(), sizeof(stCpus), &stCpus);

	cpu_set_t stWant;
	CPU_ZERO(&stWant);
	if (m_stSpec.nCpu >= 0)
		CPU_SET(m_stSpec.nCpu, &stWant);
	else
		stWant = m_stProcessCpus;

	char acHave[96], acWant[96];
	CpuList(stCpus, acHave, sizeof(acHave));
	CpuList(stWant, acWant, sizeof(acWant));
	const BOOL bOk = (nSched == 0 && nAff == 0 && nPolicy == SCHED_FIFO &&
	                  stParam.sched_priority == m_stSpec.nPriority && CPU_EQUAL(&stCpus, &stWant)) ? TRUE : FALSE;
	if (bOk)
		printf("[RT] task '%s': SCHED_FIFO %d on CPU %s%s (read back)\n", m_acName, stParam.sched_priority,
		       acHave, m_stSpec.tPeriod > 0 ? ", periodic" : "");
	else
		DBG_LOG_ERROR("(CRtTask) task '%s' runs %s %d on CPU %s; requested SCHED_FIFO %d on CPU %s",
		              m_acName, nPolicy == SCHED_FIFO ? "SCHED_FIFO" : nPolicy == SCHED_RR ? "SCHED_RR" :
		              nPolicy == SCHED_OTHER ? "SCHED_OTHER" : "?", stParam.sched_priority, acHave,
		              m_stSpec.nPriority, acWant);
	return bOk;
}

RTTIME
CRtTask::WaitNextRelease()
{
	wait_next_period(NULL);
	if (m_bWaited)
		m_uRelease++;
	else
		m_bWaited = TRUE;
	const RTTIME tRelease = m_tFirst + (RTTIME)m_uRelease * m_stSpec.tPeriod;
	/* a sleep that a signal ended early: on to the release */
	if (read_timer() < tRelease)
	{
		struct timespec stTs;
		NsToTimespec(tRelease, &stTs);
		while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &stTs, NULL) == EINTR)
			;
	}
	return tRelease;
}
