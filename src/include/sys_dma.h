/**************************************************************************//**
	Copyright (C) 2016 Marvell International Ltd.
*//***************************************************************************/

#ifndef __SYS_DMA_H__
#define __SYS_DMA_H__

#include "std.h"

/**
 * Internal physical base-address.
 *
 * this is exported only to allow the ptov/vtop functions (below) to be
 * implemented as inlines.
 */
extern phys_addr_t __dma_phys_base;
extern void *__dma_virt_base;


/**
 * Initialize the system DMA memory manager
 *
 * For an efficient conversion between user-space virtual address map(s) and bus
 * addresses required by hardware for DMA, we use a single contiguous mmap() on
 * the 'SYS_US_DEV_FILE_PATH' device, a pre-arranged physical base address.
 *
 * @param[in]	size	size of the request DMA memory.
 *
 * @retval	0 on success.
 */
int mv_sys_dma_mem_init(u64 size);

/**
 * Destroy the system DMA memory manager.
 */
void mv_sys_dma_mem_destroy(void);

/**
 * DMA memory allocation (Optimised for speed).
 *
 * @param[in]	size	size of the request DMA memory.
 * @param[in]	align	Alignment of the request DMA memory.
 *
 * @retval	A pointer to a DMA memory on success
 * @retval	<0 on failure
 */
void *mv_sys_dma_mem_alloc(size_t size, size_t align);

/**
 * Free an allocated DMA memory.
 *
 * @param[in]	ptr		A pointer to a DMA memory.
 */
void mv_sys_dma_mem_free(void *ptr);

/**
 * Physical to Virtual address translation of an allocated DMA memory.
 *
 * @param[in]	pa	Physical Address to be translated.
 *
 * @retval	A pointer to a virtual DMA memory on success
 * @retval	<0 on failure
 */
static __inline__ void * mv_sys_dma_mem_phys2virt(phys_addr_t pa)
{
	return (void *)((u64)(pa - __dma_phys_base) + (u64)__dma_virt_base);
}

/**
 * Virtual to Physical address translation of an allocated DMA memory.
 *
 * @param[in]	va	A pointer to a virtual DMA memory to be translated.
 *
 * @retval	Physical-address on success
 * @retval	<0 on failure
 */
static __inline__ phys_addr_t mv_sys_dma_mem_virt2phys(void *va)
{
	return ((u64)va - (u64)__dma_virt_base) + __dma_phys_base;
}

#endif /* __SYS_DMA_H__ */
