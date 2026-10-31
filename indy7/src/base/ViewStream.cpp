/*****************************************************************************
*	Name: ViewStream.cpp
*	Description: Live joint angles for an external viewer. See ViewStream.h.
*****************************************************************************/
#include "ViewStream.h"

#include <arpa/inet.h>
#include <chrono>
#include <cerrno>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>

CViewStream::CViewStream()
	: m_bStop(false), m_nPort(0), m_nDof(0), m_dRateHz(50.0)
{
}

CViewStream::~CViewStream()
{
	Stop();
}

void
CViewStream::Publish(const double* adQ)
{
	View stV;
	for (int i = 0; i < MAX_DOF; i++)
		stV.adQ[i] = adQ[i];
	m_cSlot.Write(stV);
}

BOOL
CViewStream::Start(int anPort, double adRateHz, int anDof)
{
	if (anPort <= 0 || IsRunning())
		return FALSE;
	if (anDof <= 0 || anDof > MAX_DOF || adRateHz <= 0.0)
	{
		DBG_LOG_ERROR("(CViewStream) dof %d, rate %g Hz", anDof, adRateHz);
		return FALSE;
	}
	m_nPort = anPort;
	m_nDof = anDof;
	m_dRateHz = adRateHz;
	m_bStop.store(false);
	m_th = std::thread(&CViewStream::Run, this);
	return TRUE;
}

void
CViewStream::Stop()
{
	m_bStop.store(true);
	if (m_th.joinable())
		m_th.join();
}

void
CViewStream::Run()
{
	pthread_setname_np(pthread_self(), "view-stream");
	const int nListen = socket(AF_INET, SOCK_STREAM, 0);
	if (nListen < 0)
	{
		DBG_LOG_ERROR("(CViewStream) socket: %s", strerror(errno));
		return;
	}
	const int nOn = 1;
	setsockopt(nListen, SOL_SOCKET, SO_REUSEADDR, &nOn, sizeof(nOn));
	struct sockaddr_in stAddr;
	memset(&stAddr, 0, sizeof(stAddr));
	stAddr.sin_family = AF_INET;
	stAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	stAddr.sin_port = htons((uint16_t)m_nPort);
	if (bind(nListen, (struct sockaddr*)&stAddr, sizeof(stAddr)) < 0 || listen(nListen, 1) < 0)
	{
		DBG_LOG_ERROR("(CViewStream) port %d: %s", m_nPort, strerror(errno));
		close(nListen);
		return;
	}
	DBG_LOG_INFO("(CViewStream) live view on TCP port %d, %.0f Hz (tools/sim_view.py live)", m_nPort, m_dRateHz);

	const auto tStart = std::chrono::steady_clock::now();
	const auto tPeriod = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
		std::chrono::duration<double>(1.0 / m_dRateHz));
	int nClient = -1;
	char acLine[512];
	auto tNext = std::chrono::steady_clock::now();

	while (!m_bStop.load())
	{
		if (nClient < 0)
		{
			struct pollfd stPoll = { nListen, POLLIN, 0 };
			if (poll(&stPoll, 1, 200) <= 0)
				continue;
			struct sockaddr_in stPeer;
			socklen_t uLen = sizeof(stPeer);
			nClient = accept(nListen, (struct sockaddr*)&stPeer, &uLen);
			if (nClient < 0)
				continue;
			setsockopt(nClient, IPPROTO_TCP, TCP_NODELAY, &nOn, sizeof(nOn));
			DBG_LOG_INFO("(CViewStream) viewer %s connected", inet_ntoa(stPeer.sin_addr));
			const int nLen = snprintf(acLine, sizeof(acLine), "# g2t-view 1 dof=%d rate=%.0f\n", m_nDof, m_dRateHz);
			if (send(nClient, acLine, (size_t)nLen, MSG_NOSIGNAL) != nLen)
			{
				close(nClient);
				nClient = -1;
				continue;
			}
			tNext = std::chrono::steady_clock::now();
		}

		tNext += tPeriod;
		std::this_thread::sleep_until(tNext);
		View stV;
		if (!m_cSlot.Read(stV))
			continue;
		const double dT = std::chrono::duration<double>(std::chrono::steady_clock::now() - tStart).count();
		int nLen = snprintf(acLine, sizeof(acLine), "%.3f", dT);
		for (int i = 0; i < m_nDof && nLen < (int)sizeof(acLine) - 32; i++)
			nLen += snprintf(acLine + nLen, sizeof(acLine) - (size_t)nLen, " %.6f", stV.adQ[i]);
		nLen += snprintf(acLine + nLen, sizeof(acLine) - (size_t)nLen, "\n");
		/* a viewer that cannot keep up loses samples; it never blocks */
		const ssize_t nSent = send(nClient, acLine, (size_t)nLen, MSG_NOSIGNAL | MSG_DONTWAIT);
		if (nSent < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
		{
			DBG_LOG_INFO("(CViewStream) viewer disconnected");
			close(nClient);
			nClient = -1;
		}
	}
	if (nClient >= 0)
		close(nClient);
	close(nListen);
}
