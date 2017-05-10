/******************************************************************************
 *	Copyright (C) 2016 Marvell International Ltd.
 *
 *  If you received this File from Marvell, you may opt to use, redistribute
 *  and/or modify this File under the following licensing terms.
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *	* Redistributions of source code must retain the above copyright
 *	  notice, this list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the distribution.
 *
 *	* Neither the name of Marvell nor the names of its contributors may be
 *	  used to endorse or promote products derived from this software
 *	  without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#ifndef __MV_COMMON_H__
#define __MV_COMMON_H__

#include <unistd.h>
#include <malloc.h>
#include <stdlib.h>
#include "env/mv_errno.h"

#define PTR2INT(_p)		((uintptr_t)(_p))
#define INT2PTR(_i)		((void *)(uintptr_t)(_i))

#ifndef NOTUSED
#define NOTUSED(_a) ((_a) = (_a))
#endif /* !NOTUSED */

#ifndef MEMBER_OFFSET
/**
 * TODO
 *
 * @param[in]	_type	TODO
 * @param[in]	_member	TODO
 *
 * @return	TODO
 */
#define MEMBER_OFFSET(_type, _member)	\
	(PTR2INT(&((_type *)0)->_member))
#endif /* !MEMBER_OFFSET */

#ifndef ARRAY_SIZE
/**
 * TODO
 *
 * @param[in]	_arr	TODO
 *
 * @return	TODO
 */
#define ARRAY_SIZE(_arr)	\
	(sizeof(_arr) / sizeof((_arr)[0]))
#endif /* !ARRAY_SIZE */

#ifndef BIT
#define BIT(nr) (1UL << (nr))
#endif

#define CREATE_MASK(pos, len)		GENMASK((pos) + (len) - 1, (pos))
#define CREATE_MASK_ULL(pos, len)	GENMASK_ULL((pos) + (len) - 1, (pos))

#define AUTO_MASK(reg_name)		CREATE_MASK(reg_name##_OFFS, reg_name##_SIZE)

#define BIT_MASK(bits)			((1 << (bits)) - 1)
#define BIT_MASK_OFFS(offs, bits)	(BIT_MASK(bits) << (offs))

/*
 * min()/max()/clamp() macros that also do
 * strict type-checking.. See the
 * "unnecessary" pointer comparison.
 */
#ifndef min
#define min(x, y) ({                            \
	typeof(x) _min1 = (x);                  \
	typeof(y) _min2 = (y);                  \
	(void)(&_min1 == &_min2);              \
	_min1 < _min2 ? _min1 : _min2; })
#endif

#ifndef max
#define max(x, y) ({                            \
	typeof(x) _max1 = (x);                  \
	typeof(y) _max2 = (y);                  \
	(void)(&_max1 == &_max2);              \
	_max1 > _max2 ? _max1 : _max2; })
#endif

#define upper_32_bits(n) ((u32)(((n) >> 16) >> 16))
#define lower_32_bits(n) ((u32)(n))

/*
 * swap - swap value of @a and @b
 */
#ifndef swap
#define swap(a, b) \
	do { typeof(a) __tmp = (a); (a) = (b); (b) = __tmp; } while (0)
#endif

#ifndef swab16
#define swab16(x) ((u16)(			\
	(((u16)(x) & (u16)0x00ffU) << 8) |	\
	(((u16)(x) & (u16)0xff00U) >> 8)))
#endif

#ifndef swab32
#define swab32(x) ((u32)(			\
	(((x) & 0x000000FF) << 24) |		\
	(((x) & 0x0000FF00) <<  8) |		\
	(((x) & 0x00FF0000) >>  8) |		\
	(((x) & 0xFF000000) >> 24)))
#endif

#ifndef swab64
#define swab64(x) ((u64)(			\
	(((x) & 0x00000000000000FFULL) << 24) |	\
	(((x) & 0x000000000000FF00ULL) << 8)  |	\
	(((x) & 0x0000000000FF0000ULL) >> 8)  |	\
	(((x) & 0x00000000FF000000ULL) >> 24) |	\
	(((x) & 0x000000FF00000000ULL) << 24) |	\
	(((x) & 0x0000FF0000000000ULL) << 8)  |	\
	(((x) & 0x00FF000000000000ULL) >> 8)  |	\
	(((x) & 0xFF00000000000000ULL) >> 24)))
#endif

#define le32_to_cpu(x) le32toh(x)

#define in4_pton(src, srclen, dst, delim, end) inet_pton(AF_INET, src, dst)
#define in6_pton(src, srclen, dst, delim, end) inet_pton(AF_INET6, src, dst)

static inline int kstrtou8(const char *s, unsigned int base, u8 *res)
{
	char *endptr;
	unsigned long ores = strtoul(s, &endptr, base);

	if (endptr == s)
		return -EINVAL;
	*res = (u8) ores;
	return 0;
}

static inline int kstrtou16(const char *s, unsigned int base, u16 *res)
{
	char *endptr;
	unsigned long ores = strtoul(s, &endptr, base);

	if (endptr == s)
		return -EINVAL;
	*res = (u16) ores;
	return 0;
}

static inline int kstrtou32(const char *s, unsigned int base, u32 *res)
{
	char *endptr;
	unsigned long ores = strtoul(s, &endptr, base);

	if (endptr == s)
		return -EINVAL;
	*res = (u32) ores;
	return 0;
}

static inline int kstrtoint(const char *s, unsigned int base, int *res)
{
	char *endptr;
	long ores = strtol(s, &endptr, base);

	if (endptr == s)
		return -EINVAL;
	*res = (int) ores;
	return 0;
}

/*
 * roundup - roundup to nearest integer
 */
#ifndef roundup
#define roundup(x, y) ((((x) + ((y) - 1)) / (y)) * (y))
#endif

/*
 * Alignment check macro
 */
#ifndef IS_ALIGNED
#define IS_ALIGNED(val, align)	(((val) & ((typeof(val))(align) - 1)) == 0)
#endif

/*
 * Alignment update macro
 */
#ifndef ALIGN
#define ALIGN(x, a)		__ALIGN_MASK(x, (typeof(x))(a) - 1)
#define __ALIGN_MASK(x, mask)	(((x) + (mask)) & ~(mask))
#endif

#pragma GCC diagnostic ignored "-Wunused-function"

/*
 * deal with unrepresentable constant logarithms
 */
extern __attribute__((const, noreturn))
int __ilog2_undef(void);

/**
 * fls - find last (most-significant) bit set
 * @x: the word to search
 *
 * This is defined the same way as ffs.
 * Note fls(0) = 0, fls(1) = 1, fls(0x80000000) = 32.
 */
static __always_inline uint32_t fls(uint32_t x)
{
	return x ? sizeof(x) * 8 - __builtin_clz(x) : 0;
}

/*
 * non-constant log of base 2 calculators.
 * - the arch may override these in asm/bitops.h if they can be implemented
 *   more efficiently than using fls() and fls64()
 * - the arch is not required to handle n==0 if implementing the fallback
 */
static uint32_t __ilog2_uint32_t(uint32_t n)
{
	return fls(n) - 1;
}

/**
 * ilog2 - log of base 2 of 32-bit or a 64-bit unsigned value
 * @n - parameter
 *
 * constant-capable log of base 2 calculation
 * - this can be used to initialise global variables from constant data, hence
 *   the massive ternary operator construction
 *
 * selects the appropriately-sized optimised version depending on sizeof(n)
 */
#define ilog2(n)				\
(						\
	__builtin_constant_p(n) ? (		\
		(n) < 1 ? __ilog2_undef() :	\
		(n) & (1ULL << 63) ? 63 :	\
		(n) & (1ULL << 62) ? 62 :	\
		(n) & (1ULL << 61) ? 61 :	\
		(n) & (1ULL << 60) ? 60 :	\
		(n) & (1ULL << 59) ? 59 :	\
		(n) & (1ULL << 58) ? 58 :	\
		(n) & (1ULL << 57) ? 57 :	\
		(n) & (1ULL << 56) ? 56 :	\
		(n) & (1ULL << 55) ? 55 :	\
		(n) & (1ULL << 54) ? 54 :	\
		(n) & (1ULL << 53) ? 53 :	\
		(n) & (1ULL << 52) ? 52 :	\
		(n) & (1ULL << 51) ? 51 :	\
		(n) & (1ULL << 50) ? 50 :	\
		(n) & (1ULL << 49) ? 49 :	\
		(n) & (1ULL << 48) ? 48 :	\
		(n) & (1ULL << 47) ? 47 :	\
		(n) & (1ULL << 46) ? 46 :	\
		(n) & (1ULL << 45) ? 45 :	\
		(n) & (1ULL << 44) ? 44 :	\
		(n) & (1ULL << 43) ? 43 :	\
		(n) & (1ULL << 42) ? 42 :	\
		(n) & (1ULL << 41) ? 41 :	\
		(n) & (1ULL << 40) ? 40 :	\
		(n) & (1ULL << 39) ? 39 :	\
		(n) & (1ULL << 38) ? 38 :	\
		(n) & (1ULL << 37) ? 37 :	\
		(n) & (1ULL << 36) ? 36 :	\
		(n) & (1ULL << 35) ? 35 :	\
		(n) & (1ULL << 34) ? 34 :	\
		(n) & (1ULL << 33) ? 33 :	\
		(n) & (1ULL << 32) ? 32 :	\
		(n) & (1ULL << 31) ? 31 :	\
		(n) & (1ULL << 30) ? 30 :	\
		(n) & (1ULL << 29) ? 29 :	\
		(n) & (1ULL << 28) ? 28 :	\
		(n) & (1ULL << 27) ? 27 :	\
		(n) & (1ULL << 26) ? 26 :	\
		(n) & (1ULL << 25) ? 25 :	\
		(n) & (1ULL << 24) ? 24 :	\
		(n) & (1ULL << 23) ? 23 :	\
		(n) & (1ULL << 22) ? 22 :	\
		(n) & (1ULL << 21) ? 21 :	\
		(n) & (1ULL << 20) ? 20 :	\
		(n) & (1ULL << 19) ? 19 :	\
		(n) & (1ULL << 18) ? 18 :	\
		(n) & (1ULL << 17) ? 17 :	\
		(n) & (1ULL << 16) ? 16 :	\
		(n) & (1ULL << 15) ? 15 :	\
		(n) & (1ULL << 14) ? 14 :	\
		(n) & (1ULL << 13) ? 13 :	\
		(n) & (1ULL << 12) ? 12 :	\
		(n) & (1ULL << 11) ? 11 :	\
		(n) & (1ULL << 10) ? 10 :	\
		(n) & (1ULL <<  9) ?  9 :	\
		(n) & (1ULL <<  8) ?  8 :	\
		(n) & (1ULL <<  7) ?  7 :	\
		(n) & (1ULL <<  6) ?  6 :	\
		(n) & (1ULL <<  5) ?  5 :	\
		(n) & (1ULL <<  4) ?  4 :	\
		(n) & (1ULL <<  3) ?  3 :	\
		(n) & (1ULL <<  2) ?  2 :	\
		(n) & (1ULL <<  1) ?  1 :	\
		(n) & (1ULL <<  0) ?  0 :	\
		__ilog2_undef()			\
				   ) :		\
	(sizeof(n) <= 4) ?			\
	__ilog2_uint32_t(n) :			\
	__ilog2_undef()				\
)

 /*
 * round up to nearest power of two
 */
static inline __attribute__((const))
uint32_t __roundup_pow_of_two(uint32_t n)
{
	return 1UL << fls(n - 1);
}

 /**
 * roundup_pow_of_two - round the given value up to nearest power of two
 * @n - parameter
 *
 * round the given value up to the nearest power of two
 * - the result is undefined when n == 0
 * - this can be used to initialise global variables from constant data
 */
#define roundup_pow_of_two(n)			\
(						\
	__builtin_constant_p(n) ? (		\
		(n == 1) ? 1 :			\
		(1 << (ilog2((n) - 1) + 1))	\
				   ) :		\
	__roundup_pow_of_two(n)			\
)

/* Kernel Memory Allocation */
#define kmalloc(size, gfp) malloc(size)
#define kcalloc(num, size, gfp) calloc(num, size)
#define kfree(ptr) if(ptr) free(ptr)

/* Kernel Delay */
#define udelay(us) usleep(us)
#define usleep_range(us, range) usleep(us)

#endif /* __MV_COMMON_H__ */
