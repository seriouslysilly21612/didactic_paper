/*****************************************************************************
*	Name: slot_check.cpp
*	Description: The concurrent-reader test of the atomic slot (paper,
*	Sec. II-A, eq. (1); indy7/src/base/AtomicSlot.h).
*
*	    bin/slot_check [--snapshots N] [--readers R] [--words W] [--unsafe]
*
*	One writer publishes N snapshots (default 150000) into one slot while R
*	readers (default 3) read it as fast as they can. Snapshot k carries the
*	sequence number k in its first and its last word and k * (i + 1) in the
*	W words between (default 64): a copy that mixes two snapshots breaks one
*	of these cross-field invariants. Every accepted copy is checked, and so
*	is the order: the number of payloads published up to it (Read()'s count)
*	must equal k, and k must not decrease for a reader. The test passes if
*	no inconsistent snapshot was accepted.
*	--unsafe reads the payload words without the sequence check (accepting
*	every copy), to show that the test sees a torn copy when there is one.
*	Prints the accepted copies, the reads that gave up after 100 attempts,
*	and the inconsistent ones.
*****************************************************************************/
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>
#include <vector>

#include "base/AtomicSlot.h"

namespace {

enum { MAX_WORDS = 256 };

struct Payload
{
	uint64_t uK;				// the snapshot number
	uint64_t auV[MAX_WORDS];	// k * (i + 1)
	uint64_t uK2;				// k again
};

struct ReaderResult
{
	long nAccepted = 0, nFailed = 0, nBad = 0, nOrder = 0, nCount = 0;
	uint64_t uLast = 0;
};

/* the same loads as CAtomicSlot::Read(), without the check of eq. (1) */
template <class T>
bool UnsafeRead(const CAtomicSlot<T>& acSlot, T& astOut)
{
	struct Probe { std::atomic<uint64_t> seq; std::atomic<uint64_t> w[CAtomicSlot<T>::WORDS]; };
	const Probe* p = reinterpret_cast<const Probe*>(&acSlot);
	if (p->seq.load() == 0)
		return false;
	uint64_t au[CAtomicSlot<T>::WORDS];
	for (size_t i = 0; i < CAtomicSlot<T>::WORDS; i++)
		au[i] = p->w[i].load();
	memcpy(&astOut, au, sizeof(T));
	return true;
}

} // namespace

int main(int argc, char** argv)
{
	long nSnapshots = 150000;
	int nReaders = 3, nWords = 64;
	bool bUnsafe = false;
	for (int i = 1; i < argc; i++)
	{
		if (!strcmp(argv[i], "--snapshots") && i + 1 < argc) nSnapshots = atol(argv[++i]);
		else if (!strcmp(argv[i], "--readers") && i + 1 < argc) nReaders = atoi(argv[++i]);
		else if (!strcmp(argv[i], "--words") && i + 1 < argc) nWords = atoi(argv[++i]);
		else if (!strcmp(argv[i], "--unsafe")) bUnsafe = true;
		else
		{
			fprintf(stderr, "usage: %s [--snapshots N] [--readers R] [--words W (1..%d)] [--unsafe]\n", argv[0],
			        (int)MAX_WORDS);
			return 2;
		}
	}
	if (nWords < 1 || nWords > MAX_WORDS || nReaders < 1 || nSnapshots < 1)
		return 2;
	if (!CAtomicSlot<Payload>::IsLockFree())
	{
		printf("64-bit atomics are not lock-free on this processor\n");
		return 1;
	}

	static CAtomicSlot<Payload> s_cSlot;
	std::atomic<bool> bDone(false);
	std::vector<ReaderResult> vRes((size_t)nReaders);
	std::vector<std::thread> vThreads;
	for (int r = 0; r < nReaders; r++)
		vThreads.emplace_back([&, r]() {
			ReaderResult& res = vRes[(size_t)r];
			Payload st;
			while (!bDone.load(std::memory_order_relaxed))
			{
				uint32_t uCount = 0;
				const bool bOk = bUnsafe ? UnsafeRead(s_cSlot, st) : s_cSlot.Read(st, &uCount);
				if (!bOk)
				{
					if (s_cSlot.Count() > 0)
						res.nFailed++;
					continue;
				}
				res.nAccepted++;
				bool bBad = (st.uK2 != st.uK);
				for (int i = 0; i < nWords && !bBad; i++)
					if (st.auV[i] != st.uK * (uint64_t)(i + 1))
						bBad = true;
				if (bBad)
					res.nBad++;
				if (!bUnsafe && uCount != st.uK)
					res.nCount++;
				if (st.uK < res.uLast)
					res.nOrder++;
				res.uLast = st.uK;
			}
		});

	Payload stW;
	memset(&stW, 0, sizeof(stW));
	for (long k = 1; k <= nSnapshots; k++)
	{
		stW.uK = (uint64_t)k;
		for (int i = 0; i < nWords; i++)
			stW.auV[i] = (uint64_t)k * (uint64_t)(i + 1);
		stW.uK2 = (uint64_t)k;
		s_cSlot.Write(stW);
	}
	bDone.store(true);
	for (auto& t : vThreads)
		t.join();

	long nBad = 0, nOrder = 0, nCount = 0;
	printf("%s slot: 1 writer, %ld snapshots of %d + 2 words; %d readers\n", bUnsafe ? "UNSAFE (no check)" : "atomic",
	       nSnapshots, nWords, nReaders);
	for (int r = 0; r < nReaders; r++)
	{
		const ReaderResult& res = vRes[(size_t)r];
		printf("  reader %d: %ld copies accepted, %ld reads gave up after %d attempts, %ld inconsistent, "
		       "%ld out of order, %ld with a wrong count\n", r, res.nAccepted, res.nFailed,
		       (int)CAtomicSlot<Payload>::MAX_ATTEMPTS, res.nBad, res.nOrder, res.nCount);
		nBad += res.nBad;
		nOrder += res.nOrder;
		nCount += res.nCount;
	}
	const bool bPass = (nBad == 0 && nOrder == 0 && nCount == 0);
	printf("%s: %ld inconsistent snapshots accepted\n", bPass ? "PASS" : "FAIL", nBad);
	return bPass ? 0 : 1;
}
