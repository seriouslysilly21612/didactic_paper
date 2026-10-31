/*****************************************************************************
*	Name: MallocCount.h
*	Description: A test build that counts the heap allocations of the
*	real-time tasks, which must stay at zero while they run.
*
*	Built with G2T_MALLOC_COUNT (make MALLOC_COUNT=1), MallocCount.cpp
*	replaces malloc, calloc, realloc, memalign, posix_memalign and
*	aligned_alloc of the program by wrappers around the allocator of glibc
*	(__libc_malloc ...); operator new and Eigen allocate through them. A call
*	is counted only on a thread that has armed the counter, so that the other
*	threads do not disturb the count. Without G2T_MALLOC_COUNT the functions
*	below do nothing and Enabled() is false. Every counted call passes
*	through malloc_count_hit(): a breakpoint there in gdb shows where it came
*	from.
*****************************************************************************/
#ifndef G2T_MALLOC_COUNT_H
#define G2T_MALLOC_COUNT_H

namespace malloc_count {

bool			Enabled	();				// built with G2T_MALLOC_COUNT
void			Arm		(bool abOn);	// count the calls of the calling thread
unsigned long	Count	();				// calls counted on the calling thread so far

} // namespace malloc_count

#endif // G2T_MALLOC_COUNT_H
