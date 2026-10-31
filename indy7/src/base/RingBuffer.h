/*****************************************************************************
*	Name: RingBuffer.h
*	Description: A single-producer, single-consumer ring of N - 1 entries
*	without a lock: a real-time task pushes (a full ring drops the entry
*	and never blocks), a thread that is not real-time pops. The logs of the
*	tests and of the timing use it (TestLog.h, TimingLog.h).
*****************************************************************************/
#ifndef G2T_RING_BUFFER_H
#define G2T_RING_BUFFER_H

#include <atomic>
#include <cstddef>

template <typename T, size_t N>
class RingBuffer
{
public:
	/* the producer: false if the ring is full (the entry is dropped) */
	bool push(const T& astItem)
	{
		const size_t uHead = m_uHead.load(std::memory_order_relaxed);
		const size_t uNext = (uHead + 1) % N;
		if (uNext == m_uTail.load(std::memory_order_acquire))
			return false;
		m_aBuf[uHead] = astItem;
		m_uHead.store(uNext, std::memory_order_release);
		return true;
	}
	/* the consumer: false if the ring is empty */
	bool pop(T& astItem)
	{
		const size_t uTail = m_uTail.load(std::memory_order_relaxed);
		if (uTail == m_uHead.load(std::memory_order_acquire))
			return false;
		astItem = m_aBuf[uTail];
		m_uTail.store((uTail + 1) % N, std::memory_order_release);
		return true;
	}

private:
	T m_aBuf[N];
	std::atomic<size_t> m_uHead{0}, m_uTail{0};
};

#endif // G2T_RING_BUFFER_H
