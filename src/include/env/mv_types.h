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
