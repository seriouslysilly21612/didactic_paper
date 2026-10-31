/*****************************************************************************
*	Name: TimingLog.cpp
*	Description: The timing of every period of the EtherCAT task. See
*	TimingLog.h.
*****************************************************************************/
#include "TimingLog.h"

#include <cerrno>
#include <chrono>
#include <cstring>
#include <ctime>
#include <pthread.h>

#include "base/Common.h"
#include "TestLog.h"		// testlog::MakeDirs

bool
CTimingLog::Start(const char* astrDir, const std::string& astrHeader)
{
	if (IsStarted())
		return true;
	std::string strDir = (astrDir != NULL) ? astrDir : "";
	while (strDir.size() > 1 && strDir.back() == '/')
		strDir.pop_back();
	if (!testlog::MakeDirs(strDir))
	{
		DBG_LOG_ERROR("(CTimingLog) cannot create %s: %s - no timing log", strDir.c_str(), strerror(errno));
		return false;
	}
	char acStamp[32];
	const time_t tNow = time(NULL);
	struct tm stTm;
	localtime_r(&tNow, &stTm);
	strftime(acStamp, sizeof(acStamp), "%Y%m%d_%H%M%S", &stTm);
	m_strPath = strDir + "/timing_" + acStamp + ".bin";
	m_strTxt  = strDir + "/timing_" + acStamp + ".txt";

	m_fp = fopen(m_strPath.c_str(), "wb");
	if (m_fp == nullptr)
	{
		DBG_LOG_ERROR("(CTimingLog) cannot write %s: %s - no timing log", m_strPath.c_str(), strerror(errno));
		return false;
	}
	/* the header: the text, padded to HEADER_BYTES, the last byte a newline */
	std::string strHead = astrHeader;
	if (!strHead.empty() && strHead.back() != '\n')
		strHead += '\n';
	if (strHead.size() > (size_t)HEADER_BYTES - 1)
	{
		DBG_LOG_WARN("(CTimingLog) header of %zu bytes cut to %d", strHead.size(), HEADER_BYTES - 1);
		strHead.resize(HEADER_BYTES - 1);
	}
	strHead.resize(HEADER_BYTES - 1, ' ');
	strHead += '\n';
	if (fwrite(strHead.data(), 1, strHead.size(), m_fp) != strHead.size())
	{
		DBG_LOG_ERROR("(CTimingLog) cannot write %s: %s - no timing log", m_strPath.c_str(), strerror(errno));
		fclose(m_fp);
		m_fp = nullptr;
		return false;
	}
	m_uRecords = 0;
	m_uDrops.store(0);
	m_bStop.store(false);
	m_th = std::thread(&CTimingLog::Run, this);
	m_bOn.store(true, std::memory_order_release);
	DBG_LOG_INFO("(CTimingLog) timing of every cycle -> %s", m_strPath.c_str());
	return true;
}

void
CTimingLog::Stop()
{
	if (!IsStarted())
		return;
	m_bOn.store(false, std::memory_order_release);
	m_bStop.store(true);
	m_th.join();
	const unsigned long uDrops = m_uDrops.load();
	FILE* fp = fopen(m_strTxt.c_str(), "w");
	if (fp != nullptr)
	{
		fprintf(fp, "file=%s\nrecords=%lu\ndropped=%lu\nrecord_bytes=%zu\nheader_bytes=%d\n", m_strPath.c_str(),
		        m_uRecords, uDrops, sizeof(Record), (int)HEADER_BYTES);
		fclose(fp);
	}
	printf("[TIMING] %lu cycles written to %s%s\n", m_uRecords, m_strPath.c_str(),
	       uDrops ? " - RECORDS DROPPED, see the .txt" : "");
	if (uDrops)
		printf("[TIMING] %lu records dropped: the writer did not keep up\n", uDrops);
	fflush(stdout);
}

void
CTimingLog::Run()
{
	pthread_setname_np(pthread_self(), "timing-log");
	Record stR;
	auto tFlush = std::chrono::steady_clock::now();
	for (;;)
	{
		const bool bStop = m_bStop.load();
		while (m_ring.pop(stR))
		{
			if (fwrite(&stR, sizeof(stR), 1, m_fp) == 1)
				m_uRecords++;
		}
		const auto tNow = std::chrono::steady_clock::now();
		if (tNow - tFlush > std::chrono::seconds(1))
		{
			fflush(m_fp);		// the file grows while the application runs
			tFlush = tNow;
		}
		if (bStop)
			break;			// everything pushed before the stop has been popped
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	fclose(m_fp);
	m_fp = nullptr;
}
