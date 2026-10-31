/*****************************************************************************
*	Name: AtomicSlot.h
*	Description: Single-writer atomic slot (paper, Sec. II-A, eq. (1)).
*
*	The payload T, a plain struct, is stored as fixed-size 64-bit atomic
*	words; every access to a shared word is an atomic load or store with
*	sequentially consistent ordering, so no thread ever reads a mutable
*	non-atomic object that another thread writes.
*
*	  writer  publishes an odd sequence number, stores the payload words,
*	          then publishes the next even number;
*	  reader  loads the sequence number s0, the payload words, and the
*	          sequence number s1, and accepts the copy only if
*	              accept(s0, s1)  <=>  s0 = s1  and  s0 > 0  and  s0 even,
*	          at most MAX_ATTEMPTS times; otherwise its previous copy (and
*	          the period tag in it) stays as it was.
*
*	One writer per slot. The writer never waits; the read takes a bounded
*	number of steps. No lock, no allocation, no system call. s0 / 2 is the
*	number of payloads published so far, which a reader uses to tell a new
*	payload from one it has seen. IsLockFree() must hold on the target
*	(checked when the application starts).
*****************************************************************************/
#ifndef G2T_ATOMIC_SLOT_H
#define G2T_ATOMIC_SLOT_H

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

template <class T>
class CAtomicSlot
{
	static_assert(std::is_trivially_copyable<T>::value, "the payload of a slot must be trivially copyable");

public:
	enum { WORDS = (sizeof(T) + sizeof(uint64_t) - 1) / sizeof(uint64_t), MAX_ATTEMPTS = 100 };

	CAtomicSlot()
	{
		m_uSeq.store(0);
		for (size_t i = 0; i < WORDS; i++)
			m_auWord[i].store(0);
	}
	CAtomicSlot(const CAtomicSlot&) = delete;
	CAtomicSlot& operator=(const CAtomicSlot&) = delete;

	/* the 64-bit words of the slot are lock-free on this processor */
	static bool IsLockFree()
	{
		std::atomic<uint64_t> uProbe(0);
		return uProbe.is_lock_free();
	}

	/* eq. (1) */
	static bool Accept(uint64_t auS0, uint64_t auS1)
	{
		return (auS0 == auS1) && (auS0 > 0) && (auS0 % 2 == 0);
	}

	/* the writer (one thread only) */
	void Write(const T& astData)
	{
		uint64_t auBuf[WORDS];
		memset(auBuf, 0, sizeof(auBuf));
		memcpy(auBuf, &astData, sizeof(T));
		const uint64_t uSeq = m_uSeq.load();
		m_uSeq.store(uSeq + 1);					// odd: being written
		for (size_t i = 0; i < WORDS; i++)
			m_auWord[i].store(auBuf[i]);
		m_uSeq.store(uSeq + 2);					// even: published
	}

	/* a reader: true and the newest payload in astData if a copy was
	 * accepted; false if nothing has been published yet or every attempt
	 * overlapped a write (astData is then unchanged). *apuCount: the number
	 * of payloads published up to the accepted copy. */
	bool Read(T& astData, uint32_t* apuCount = NULL) const
	{
		uint64_t auBuf[WORDS];
		for (int nAttempt = 0; nAttempt < (int)MAX_ATTEMPTS; nAttempt++)
		{
			const uint64_t uS0 = m_uSeq.load();
			if (uS0 == 0)
				return false;					// never published
			for (size_t i = 0; i < WORDS; i++)
				auBuf[i] = m_auWord[i].load();
			const uint64_t uS1 = m_uSeq.load();
			if (Accept(uS0, uS1))
			{
				memcpy(&astData, auBuf, sizeof(T));
				if (apuCount != NULL)
					*apuCount = (uint32_t)(uS0 / 2);
				return true;
			}
		}
		return false;
	}

	/* the number of payloads published so far */
	uint32_t Count() const { return (uint32_t)(m_uSeq.load() / 2); }

private:
	std::atomic<uint64_t>	m_uSeq;
	std::atomic<uint64_t>	m_auWord[WORDS];
};

#endif // G2T_ATOMIC_SLOT_H
