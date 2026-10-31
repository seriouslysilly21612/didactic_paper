/*****************************************************************************
*	Name: TestLog.h
*	Description: The writer of the test logs (JOINT TEST, high-level tests):
*	one CSV file per test, one row per control cycle, and a .json file with
*	the parameters and the result. The real-time task only fills a
*	preallocated ring buffer (wait-free: no lock, no allocation, no system
*	call); a thread that is not real-time opens the files, drains the ring
*	while the test runs, and closes them when the task reports the end of
*	the test.
*
*	Format F describes one kind of test:
*	  typedef ... Sample, Meta, Result;	// plain data
*	  void Name(const Meta&, const char* astrStamp, char* acOut, size_t)
*	  void Header(FILE*, const Meta&)		// first line of the CSV
*	  void Begin()							// reset the running statistics
*	  void Row(FILE*, const Meta&, const Sample&)	// one row, statistics
*	  void Finish(FILE* apJson, const char* astrCsv, const Meta&, const Result&,
*	              const char* astrState, long anRows, unsigned anDrops)
*	          // writes the .json (apJson may be NULL) and prints the summary
*****************************************************************************/
#ifndef G2T_TEST_LOG_H
#define G2T_TEST_LOG_H

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <string>
#include <thread>
#include <pthread.h>
#include <sys/stat.h>

#include "base/Common.h"
#include "base/RingBuffer.h"

namespace testlog {

/* mkdir -p */
inline bool MakeDirs(const std::string& astrDir)
{
	if (astrDir.empty())
		return false;
	size_t uPos = 0;
	while (uPos != std::string::npos)
	{
		uPos = astrDir.find('/', uPos + 1);
		const std::string strPath = astrDir.substr(0, uPos);
		if (!strPath.empty() && mkdir(strPath.c_str(), 0775) != 0 && errno != EEXIST)
			return false;
	}
	struct stat st;
	return stat(astrDir.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}

inline void PrintVec(FILE* fp, const char* astrKey, const double* adV, int anN, const char* astrFmt, bool abLast = false)
{
	fprintf(fp, "  \"%s\": [", astrKey);
	for (int i = 0; i < anN; i++)
	{
		if (i)
			fputs(", ", fp);
		fprintf(fp, astrFmt, adV[i]);
	}
	fprintf(fp, "]%s\n", abLast ? "" : ",");
}

} // namespace testlog

template <class F, size_t RING_N = 8192>
class CTestLog
{
public:
	typedef typename F::Sample Sample;
	typedef typename F::Meta   Meta;
	typedef typename F::Result Result;

	CTestLog() : m_nState(0), m_uDrops(0), m_bOpen(FALSE)
	{
		memset(&m_meta, 0, sizeof(m_meta));
		memset(&m_res, 0, sizeof(m_res));
		memset(m_acCsv, 0, sizeof(m_acCsv));
	}
	~CTestLog() { Stop(); }

	/* non-real-time: the writer thread, files in astrDir (created if missing) */
	BOOL Start(const char* astrDir, const char* astrThread)
	{
		if (IsStarted())
			return TRUE;
		m_strDir = astrDir ? astrDir : "";
		while (m_strDir.size() > 1 && m_strDir.back() == '/')
			m_strDir.pop_back();
		if (!testlog::MakeDirs(m_strDir))
		{
			DBG_LOG_ERROR("(CTestLog) cannot create %s: %s - tests are not logged", m_strDir.c_str(), strerror(errno));
			return FALSE;
		}
		snprintf(m_acThread, sizeof(m_acThread), "%s", astrThread ? astrThread : "test-log");
		m_bStop.store(false);
		m_th = std::thread(&CTestLog::Run, this);
		return TRUE;
	}
	void Stop()
	{
		m_bStop.store(true);
		if (m_th.joinable())
			m_th.join();
	}
	BOOL IsStarted() const { return m_th.joinable(); }
	const std::string& GetDir() const { return m_strDir; }

	/* control task */
	BOOL IsIdle() const		// the files of the last test are closed (always, without the writer)
	{
		return !IsStarted() ||
		       m_uDoneSeq.load(std::memory_order_acquire) == m_uOpenSeq.load(std::memory_order_relaxed);
	}
	BOOL IsOpen() const { return m_bOpen; }
	void Open(const Meta& astMeta)
	{
		if (!IsStarted())
			return;
		m_meta = astMeta;
		m_uDrops = 0;
		m_bOpen = TRUE;
		m_uOpenSeq.fetch_add(1, std::memory_order_release);
	}
	void Push(const Sample& astSample)
	{
		if (m_bOpen && !m_ring.push(astSample))
			m_uDrops++;
	}
	void Close(int anState, const Result& astRes)
	{
		if (!m_bOpen)
			return;
		m_res = astRes;
		m_nState = anState;
		m_bOpen = FALSE;
		m_uCloseSeq.store(m_uOpenSeq.load(std::memory_order_relaxed), std::memory_order_release);
	}

	F& Format() { return m_fmt; }		// the writer thread uses it while a test is written

private:
	void Run()
	{
		pthread_setname_np(pthread_self(), m_acThread);
		FILE* pCsv = NULL;
		Meta stMeta;
		unsigned int uSeq = 0;
		Sample stS;
		long nRows = 0;
		for (;;)
		{
			const bool bStop = m_bStop.load();
			if (pCsv == NULL)
			{
				const unsigned int uOpen = m_uOpenSeq.load(std::memory_order_acquire);
				if (uOpen != m_uDoneSeq.load(std::memory_order_relaxed))
				{
					uSeq = uOpen;
					stMeta = m_meta;
					char acStamp[32];
					const time_t tNow = time(NULL);
					struct tm stTm;
					localtime_r(&tNow, &stTm);
					strftime(acStamp, sizeof(acStamp), "%Y%m%d_%H%M%S", &stTm);
					char acName[256];
					m_fmt.Name(stMeta, acStamp, acName, sizeof(acName));
					snprintf(m_acCsv, sizeof(m_acCsv), "%s/%s.csv", m_strDir.c_str(), acName);
					pCsv = fopen(m_acCsv, "w");
					if (pCsv == NULL)
					{
						printf("[LOG] cannot write %s: %s - this test is not logged\n", m_acCsv, strerror(errno));
						fflush(stdout);
						while (m_uCloseSeq.load(std::memory_order_acquire) != uSeq && !m_bStop.load())
						{
							while (m_ring.pop(stS)) {}
							std::this_thread::sleep_for(std::chrono::milliseconds(5));
						}
						while (m_ring.pop(stS)) {}
						m_uDoneSeq.store(uSeq, std::memory_order_release);
						continue;
					}
					printf("[LOG] -> %s\n", m_acCsv);
					fflush(stdout);
					m_fmt.Header(pCsv, stMeta);
					m_fmt.Begin();
					nRows = 0;
				}
			}
			if (pCsv != NULL)
			{
				/* everything pushed before Close() has been read once the
				 * ring is empty after the close was seen */
				const bool bClosed = m_uCloseSeq.load(std::memory_order_acquire) == uSeq;
				while (m_ring.pop(stS))
				{
					m_fmt.Row(pCsv, stMeta, stS);
					nRows++;
				}
				if (bClosed || bStop)
				{
					fclose(pCsv);
					pCsv = NULL;
					char acJson[520];
					snprintf(acJson, sizeof(acJson), "%s", m_acCsv);
					char* pcDot = strrchr(acJson, '.');
					if (pcDot != NULL)
						strcpy(pcDot, ".json");
					FILE* pJson = fopen(acJson, "w");
					m_fmt.Finish(pJson, m_acCsv, stMeta, m_res, bClosed ? StateName(m_nState) : "application stopped",
					             nRows, m_uDrops);
					if (pJson != NULL)
						fclose(pJson);
					fflush(stdout);
					m_uDoneSeq.store(uSeq, std::memory_order_release);
				}
			}
			if (bStop && pCsv == NULL)
				break;
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
		}
	}

	static const char* StateName(int anState)
	{
		switch (anState)
		{
			case 2:  return "at rest";
			case 3:  return "timeout (not at rest)";
			case 4:  return "aborted";
			case 5:  return "completed";
			case 6:  return "generator error";
			default: return "unknown";
		}
	}

	F							m_fmt;
	RingBuffer<Sample, RING_N>	m_ring;
	std::thread					m_th;
	std::atomic<bool>			m_bStop{false};
	std::string					m_strDir;
	char						m_acThread[16];

	/* handshake: the control task writes m_meta, then m_uOpenSeq; m_res,
	 * m_nState and m_uDrops, then m_uCloseSeq (release); the writer reads them
	 * after acquiring the counters and sets m_uDoneSeq */
	Meta						m_meta;
	Result						m_res;
	int							m_nState;
	unsigned int				m_uDrops;
	BOOL						m_bOpen;			// control task only
	std::atomic<unsigned int>	m_uOpenSeq{0}, m_uCloseSeq{0}, m_uDoneSeq{0};
	char						m_acCsv[512];		// writer only
};

/* end states of a test (the JOINT TEST uses the values of the controller) */
enum eTestEnd { eTestAtRest = 2, eTestTimeout = 3, eTestAborted = 4, eTestCompleted = 5, eTestGenError = 6 };

#endif // G2T_TEST_LOG_H
