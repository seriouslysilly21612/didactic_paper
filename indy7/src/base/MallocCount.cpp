/*****************************************************************************
*	Name: MallocCount.cpp
*	Description: The allocation counter of the test build. See MallocCount.h.
*****************************************************************************/
#include "MallocCount.h"

#ifdef G2T_MALLOC_COUNT

#include <cerrno>
#include <cstddef>

extern "C" {
void*	__libc_malloc	(size_t);
void	__libc_free		(void*);
void*	__libc_calloc	(size_t, size_t);
void*	__libc_realloc	(void*, size_t);
void*	__libc_memalign	(size_t, size_t);
}

/* initial-exec TLS of the executable: reading it never allocates */
static __thread int				t_nArmed = 0;
static __thread unsigned long	t_uCount = 0;

/* called for every counted allocation: to find where one comes from, run the
 * program in gdb with "break malloc_count_hit" and "bt" at the breakpoint */
extern "C" __attribute__((noinline)) void malloc_count_hit(void) { __asm__ __volatile__(""); }

static inline void Hit() { if (t_nArmed) { t_uCount++; malloc_count_hit(); } }

extern "C" void* malloc(size_t n)                    { Hit(); return __libc_malloc(n); }
extern "C" void  free(void* p)                       { __libc_free(p); }
extern "C" void* calloc(size_t n, size_t s)          { Hit(); return __libc_calloc(n, s); }
extern "C" void* realloc(void* p, size_t n)          { Hit(); return __libc_realloc(p, n); }
extern "C" void* memalign(size_t a, size_t n)        { Hit(); return __libc_memalign(a, n); }
extern "C" void* aligned_alloc(size_t a, size_t n)   { Hit(); return __libc_memalign(a, n); }
extern "C" int   posix_memalign(void** pp, size_t a, size_t n)
{
	Hit();
	if (a < sizeof(void*) || (a & (a - 1)) != 0)
		return EINVAL;
	void* p = __libc_memalign(a, n);
	if (p == NULL)
		return ENOMEM;
	*pp = p;
	return 0;
}

namespace malloc_count {
bool          Enabled()       { return true; }
void          Arm(bool abOn)  { t_nArmed = abOn ? 1 : 0; }
unsigned long Count()         { return t_uCount; }
}

#else

namespace malloc_count {
bool          Enabled()       { return false; }
void          Arm(bool)       { }
unsigned long Count()         { return 0; }
}

#endif
