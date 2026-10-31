/*****************************************************************************
*	Name: TimingLog.h
*	Description: The timing of every period of the EtherCAT task (paper,
*	Sec. III-C, eq. (3), Table 6 and Fig. 3). With the option -t DIR the
*	EtherCAT task pushes one record per period into a preallocated ring
*	(wait-free: no lock, no allocation, no system call), and a thread that
*	is not real-time appends the records to DIR/timing_<stamp>.bin.
*
*	File (format g2t-timing-1): a text header of HEADER_BYTES bytes
*	(key=value lines, padded with spaces, the last byte '\n'), then the
*	records (60 bytes, little endian, see Record). When the log stops,
*	DIR/timing_<stamp>.txt receives the counts (records written, records
*	dropped because the ring was full). tools/timing_summary.py reads it.
*	About 216 MB per hour at 1 kHz.
*
*	The EtherCAT task writes the record of every period: its own timing (t_w,
*	receive, send, t_e) is of this period; the fields of the Trajectory and
*	the Dynamics task (dyn_ns, hl_ns, their t_w and t_e) come from their
*	slots, i.e. from their cycle of the PREVIOUS period, the last one
*	complete when the record is written (the two tasks run after the
*	EtherCAT task within a period).
*****************************************************************************/
#ifndef G2T_TIMING_LOG_H
#define G2T_TIMING_LOG_H

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <string>
#include <thread>

#include "base/RingBuffer.h"

class CTimingLog
{
public:
	enum { HEADER_BYTES = 1024, RING_N = 16384 };	// 16 s of records at 1 kHz

	/* flags of a record */
	enum {
		TF_OP       = 1 << 0,	// bus OP and both working counters complete (always in simulation)
		TF_WKC      = 1 << 1,	// incomplete working counter after the bus was up once
		TF_LATE     = 1 << 2,	// the cycle started more than a period after its release (it ran)
		TF_MISSED   = 1 << 3,	// t_w + t_s > t_b: the send returned after the send deadline
		TF_OVERRUN  = 1 << 4,	// t_w + t_e > T: the cycle ended after the next release
		TF_LAW      = 1 << 5,	// the control law ran (controller enabled)
		TF_HL       = 1 << 6,	// the high-level controller ran (TRACK or HOME)
		TF_TRACK    = 1 << 7,	// TRACK (with TF_HL); HOME without it
		TF_STOP     = 1 << 8,	// the application is stopping: the drives are switched off
		TF_SIM      = 1 << 9,	// the arm is simulated (-s, -b)
		TF_BUS      = 1 << 10,	// an EtherCAT exchange ran (robot, -b)
		TF_SERVO    = 1 << 11,	// every joint of the arm is servo on
		TF_BUSSERVO = 1 << 12,	// -b: a drive of the bus reports Operation enabled (must stay 0)
		TF_TEST     = 1 << 13,	// a JOINT TEST, a high-level test or a motion of the operator runs
	};

#pragma pack(push, 1)
	struct Record {
		uint32_t uPeriod;		// release index k of the period (k = 0: the first release)
		int32_t  nW;			// t_w [ns]: planned release -> start of the cycle
		int32_t  nS;			// t_s [ns]: start -> return of the send
		int32_t  nE;			// t_e [ns]: start -> end of the cycle
		int32_t  nDyn;			// RBDL calls of the control law [ns] (-1: the law did not run)
		int32_t  nHl;			// high-level controller [ns] (-1: it did not run)
		int32_t  nSim;			// simulator step [ns] (-1: no simulator)
		uint32_t uSyncDiff;		// SYNC_MODE=DC: system-time difference of the slaves [ns]
						// (0xFFFFFFFF: not measured)
		uint16_t usFlags;
		uint16_t usReserved;		// 0
		/* the phases of t_s (receive + process + send = t_s) */
		int32_t  nRecv;			// receive [ns]: start -> the inputs of the cycle are in (the frame,
						// the working counters and states; with -s/-b the state of the
						// simulated arm)
		int32_t  nSend;			// send [ns]: the send call (outputs, application time, the frame);
						// with -s the simulator step instead
		/* the other tasks of the pipeline, their cycle of the previous
		 * period (see above); -1: that task did not run */
		int32_t  nTrajW;		// Trajectory task: t_w [ns], planned release -> start
		int32_t  nTrajE;		// Trajectory task: t_e [ns], start -> end of its cycle
		int32_t  nDynW;			// Dynamics task: t_w [ns]
		int32_t  nDynE;			// Dynamics task: t_e [ns]
	};
#pragma pack(pop)
	static_assert(sizeof(Record) == 60, "the record layout is part of the file format");

	CTimingLog() {}
	~CTimingLog() { Stop(); }

	/* non-real-time, before the real-time tasks start: create the directory,
	 * open the file, write the header (key=value lines), start the writer */
	bool Start(const char* astrDir, const std::string& astrHeader);
	/* non-real-time, after the real-time tasks have ended: drain the ring,
	 * close the file, write the counts; prints the path */
	void Stop();
	bool IsStarted() const { return m_th.joinable(); }
	const std::string& GetPath() const { return m_strPath; }

	/* the EtherCAT task */
	bool IsOn() const { return m_bOn.load(std::memory_order_relaxed); }
	void Push(const Record& astR)
	{
		if (m_bOn.load(std::memory_order_relaxed) && !m_ring.push(astR))
			m_uDrops.fetch_add(1, std::memory_order_relaxed);
	}

private:
	void Run();

	RingBuffer<Record, RING_N>	m_ring;
	std::thread					m_th;
	std::atomic<bool>			m_bOn{false};
	std::atomic<bool>			m_bStop{false};
	std::atomic<unsigned long>	m_uDrops{0};
	FILE*						m_fp{nullptr};		// writer
	unsigned long				m_uRecords{0};		// writer
	std::string					m_strPath, m_strTxt;
};

#endif // G2T_TIMING_LOG_H
