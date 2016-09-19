/**************************************************************************//**
	Copyright (C) 2016 Marvell International Ltd.
*//***************************************************************************/

#ifndef __TYPES_H__
#define __TYPES_H__

//#include <inttypes.h>
#include <stdint.h>
#include <stddef.h>

typedef uint8_t		u8;
typedef uint16_t	u16;
typedef uint32_t	u32;
typedef uint64_t	u64;

#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
typedef u64 dma_addr_t;
typedef u64 phys_addr_t;
#else
typedef u32 dma_addr_t;
typedef u32 phys_addr_t;
#endif /* CONFIG_ARCH_DMA_ADDR_T_64BIT */

#endif /* __TYPES_H__ */
