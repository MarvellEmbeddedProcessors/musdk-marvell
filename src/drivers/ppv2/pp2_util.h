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

/**
 * @file pp2_util.h
 *
 * PPDK - Utilities
 *
 */

#ifndef __PP2_UTIL_H__
#define __PP2_UTIL_H__

#include "pp2_types.h"

/* TODO: Keep these until classifier phase */
#pragma GCC diagnostic ignored "-Wunused-function"

/*
 * swap - swap value of @a and @b
 */
#define swap(a, b) \
	do { typeof(a) __tmp = (a); (a) = (b); (b) = __tmp; } while (0)

#define swab16(x) ((uint16_t)(				\
	(((uint16_t)(x) & (uint16_t)0x00ffU) << 8) |	\
	(((uint16_t)(x) & (uint16_t)0xff00U) >> 8)))

#define roundup(x, y) ((((x) + ((y) - 1)) / (y)) * (y))

/*
 * Return number of array's elements
 */
#define ELEM_OF(t)  (sizeof(t) / sizeof((t)[0]))

/*
 * Alignment check macro
 */
#ifndef IS_ALIGNED
#define IS_ALIGNED(val, align)  (!((uint32_t)(val) & (align - 1)))
#endif
/*
 * Alignment update macro
 */
#ifndef ALIGN
#define ALIGN(x,a)              __ALIGN_MASK(x,(typeof(x))(a)-1)
#define __ALIGN_MASK(x,mask)    (((x)+(mask))&~(mask))
#endif

/*
 * Brench prediction improvements
 */
#ifndef unlikely
#define unlikely(expr) __builtin_expect(!!(expr), 0)
#endif
#ifndef likely
#define likely(expr)   __builtin_expect(!!(expr), 1)
#endif

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


static inline bool mv_check_eaddr_mc(const uint8_t *eaddr)
{
	uint16_t e_16 = *(const uint16_t *)eaddr;
#ifdef __BIG_ENDIAN
	return 0x01 & e_16;
#else
	return 0x01 & (e_16 >> ((sizeof(e_16) * 8) - 8));
#endif
}


static inline bool mv_check_eaddr_uc(const uint8_t *addr)
{
   return !mv_check_eaddr_mc(addr);
}


static inline bool mv_check_eaddr_bc(const uint8_t *eaddr)
{
	return (*(const uint16_t *)(eaddr + 0) &
		*(const uint16_t *)(eaddr + 2) &
		*(const uint16_t *)(eaddr + 4)) == 0xffff;
}


static inline int mv_check_eaddr_zero(const uint8_t *eaddr)
{
   return !(eaddr[0] | eaddr[1] | eaddr[2] | eaddr[3] | eaddr[4] | eaddr[5]);
}


static inline bool mv_eaddr_identical(const uint8_t *eaddr1, const uint8_t *eaddr2)
{
   const uint16_t *e1_16 = (const uint16_t *)eaddr1;
   const uint16_t *e2_16 = (const uint16_t *)eaddr2;
   return ((e1_16[0] ^ e2_16[0]) | (e1_16[1] ^ e2_16[1]) | (e1_16[2] ^ e2_16[2])) == 0;
}

static inline int mv_check_eaddr_valid(const uint8_t *addr)
{
   return !mv_check_eaddr_mc(addr) && !mv_check_eaddr_zero(addr);
}

static inline void mv_cp_eaddr(uint8_t *dest, const uint8_t *source)
{
	 uint16_t *dst_16 = (uint16_t *)dest;
	 const uint16_t *src_16 = (const uint16_t *)source;

	 dst_16[0] = src_16[0];
	 dst_16[1] = src_16[1];
	 dst_16[2] = src_16[2];
}

#endif /* __PP2_UTIL_H__ */
