/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __MV_TYPES_H__
#define __MV_TYPES_H__

#ifndef __KERNEL__

#include <stdint.h>
#include <inttypes.h>

/* C99 does not allow redefining typedef, user may optionally take u8/u16/etc. from musdk. */
#ifdef MVCONF_TYPES_PUBLIC
typedef uint8_t		u8;
typedef uint16_t	u16;
typedef uint32_t	u32;
typedef uint64_t	u64;
typedef int8_t		s8;
typedef int16_t		s16;
typedef int32_t		s32;
typedef int64_t		s64;
#endif

/* C99 does not allow redefining typedef, user may optionally take dma_addr_t/phys_addr_t from musdk. */
#if MVCONF_DMA_PHYS_ADDR_T_SIZE == 64
	#ifdef MVCONF_DMA_PHYS_ADDR_T_PUBLIC
		typedef u64 dma_addr_t;
		typedef u64 phys_addr_t;
	#endif
	#define PRIdma         PRIx64
#else /* MVCONF_DMA_PHYS_ADDR_T_SIZE */
	#ifdef MVCONF_DMA_PHYS_ADDR_T_PUBLIC
		typedef u32 dma_addr_t;
		typedef u32 phys_addr_t;
	#endif
	#define PRIdma         PRIx32
#endif /* MVCONF_DMA_PHYS_ADDR_T_SIZE */


/* At this stage dma_addr_t/phys_addr_t have been defined, either externally or internally.
 * Following code checks that their sizes adhere to the requested compilation flag.
 * mv_types_check() is just a (necessary) container for BUILD_BUG_ON(). The fn is never called.
 * The macro is created and used locally, since mv_types.h needs to be self-sufficient.
 */
#define MV_TYPES_BUILD_BUG_ON(x) ((void)sizeof(char[1 - 2*!!(x)]))
static inline void mv_types_check(void)
{
	MV_TYPES_BUILD_BUG_ON(sizeof(dma_addr_t) != (MVCONF_DMA_PHYS_ADDR_T_SIZE >> 3));
	MV_TYPES_BUILD_BUG_ON(sizeof(phys_addr_t) != (MVCONF_DMA_PHYS_ADDR_T_SIZE >> 3));
}
#undef MV_TYPES_BUILD_BUG_ON


#endif /* __KERNEL__ */

#endif /* __MV_TYPES_H__ */
