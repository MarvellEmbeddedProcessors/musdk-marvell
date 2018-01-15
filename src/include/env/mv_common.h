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

#ifndef BITS_PER_BYTE
#define BITS_PER_BYTE	(8)
#endif

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

/**
 * clamp - return a value clamped to a given range with strict type-checking
 * @val: current value
 * @lo: lowest allowable value
 * @hi: highest allowable value
 */
#ifndef clamp
#define clamp(val, lo, hi) min((typeof(val))max(val, lo), hi)
#endif

/*
 * min_t()/max_t()/clamp_t() macros that do no type-checking and uses temporary
 * variables of type 'type' to make all the comparisons.
 * should be used when min/max/clamp are called with type casting
 */
#ifndef min_t
#define min_t(type, x, y) ({			\
	type __min1 = (x);			\
	type __min2 = (y);			\
	__min1 < __min2 ? __min1 : __min2; })
#endif

#ifndef max_t
#define max_t(type, x, y) ({			\
	type __max1 = (x);			\
	type __max2 = (y);			\
	__max1 > __max2 ? __max1 : __max2; })
#endif

/**
 * clamp_t - return a value clamped to a given range using a given type
 * @type: the type of variable to use
 * @val: current value
 * @lo: minimum allowable value
 * @hi: maximum allowable value
 */
#define clamp_t(type, val, lo, hi) min_t(type, max_t(type, val, lo), hi)

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

#define be32_to_cpu(x) be32toh(x)
#define le32_to_cpu(x) le32toh(x)
#define cpu_to_le32(x) htole32(x)


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


static inline int kstrtou64(const char *s, unsigned int base, u64 *res)
{
	char *endptr;
	unsigned long ores = strtoull(s, &endptr, base);

	if (endptr == s)
		return -EINVAL;
	*res = (u64) ores;
	return 0;
}

static inline int bit_count(u64 mask)
{
	int count = 0, i;

	for (i = 0; i < 64; i++) {
		count += (mask & 1);
		mask >>= 1;
	}
	return count;
}

/*
 * roundup - roundup to nearest integer
 */
#ifndef roundup
#define roundup(x, y) ((((x) + ((y) - 1)) / (y)) * (y))
#endif

/* Returns the least number N such that N * Y >= X.  */
#ifndef ceil
#define ceil(x, y) ((((x) + ((y) - 1)) / (y)))
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


/**
 * fls - find last (most-significant) bit set
 * @x: the word to search
 *
 * This is defined the same way as ffs.
 * Note fls(0) = 0, fls(1) = 1, fls(0x80000000) = 32.
 */
static __always_inline u32 fls_32(u32 x)
{
	return x ? (32 - __builtin_clz(x)) : 0;
}


/* fls_64 relies on fls_32 to check for x=0 */
static __always_inline u32 fls_64(u64 x)
{
	return upper_32_bits(x) ? fls_32(upper_32_bits(x)) : fls_32(lower_32_bits(x));
}

#define fls(n)						\
(							\
	(sizeof(n) <= 4) ? fls_32(n) : fls_64(n)	\
)



/**
 * mvlog2 - log of base 2 of 32-bit or a 64-bit unsigned value
 * @n - parameter
 *
 */
#define mvlog2(n)				\
(						\
	fls(n) - 1				\
)


 /**
 * roundup_pow_of_two - round the given value up to nearest power of two
 * @n - parameter
 *
 * round the given value up to the nearest power of two
 * - the result is undefined when n == 0
 */
#define roundup_pow_of_two(n)			\
(						\
	1 << fls(n - 1)				\
)

/* Kernel Memory Allocation */
#define kmalloc(size, gfp) malloc(size)
#define kzalloc(size, gfp) kcalloc(1, size, gfp)
#define kcalloc(num, size, gfp) calloc(num, size)
#define kfree(ptr) if(ptr) free(ptr)

/* Kernel Delay */
#define udelay(us) usleep(us)
#define usleep_range(us, range) usleep(us)
#define mdelay(ms) usleep(ms*1000)

#endif /* __MV_COMMON_H__ */
