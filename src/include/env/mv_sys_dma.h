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

#ifndef __MV_SYS_DMA_H__
#define __MV_SYS_DMA_H__

#include "mv_types.h"

/** @addtogroup grp_sys_dma DMA Memory Manager
 *
 *  DMA Memory Manager API documentation
 *
 *  @{
 */

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
 * MUSDK system provides a mechanism for DMA-able memory allocation.
 * Assume this memory is a single, contiguously allocated region.
 * In some implementations (e.g. when using hugepages), it is only contiguous
 * in the virtual address space, while in other implementations (e.g. CMA),
 * it is allocated as a contiguous memory both in virtual as well as in
 * physical address space. It is created in order to support an efficient
 * conversion between user-space virtual address map(s) and bus addresses
 * required by hardware for DMA.
 *
 * @param[in]	size	size of the request DMA memory.
 *
 * @retval	0 on success.
 * @retval	error code on failure.
 */
int mv_sys_dma_mem_init(u64 size);

/**
 * Destroy the system DMA memory manager.
 */
void mv_sys_dma_mem_destroy(void);

/**
 * Allocate DMA memory slice from the system DMA memory.
 *
 * @param[in]	size	size of the request DMA memory.
 * @param[in]	align	Alignment of the request DMA memory.
 *
 * @retval	A pointer to a DMA memory on success
 * @retval	<0 on failure
 */
void *mv_sys_dma_mem_alloc(size_t size, size_t align);

/**
 * Free an allocated DMA memory slice of the system DMA memory.
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
#ifdef MVCONF_SYS_DMA_HUGE_PAGE
void *mv_sys_dma_mem_phys2virt(phys_addr_t pa);
#else /* !MVCONF_SYS_DMA_HUGE_PAGE */
static inline void *mv_sys_dma_mem_phys2virt(phys_addr_t pa)
{
	return (void *)((u64)(pa - __dma_phys_base) + (u64)__dma_virt_base);
}
#endif /* MVCONF_SYS_DMA_HUGE_PAGE */

/**
 * Virtual to Physical address translation of an allocated DMA memory.
 *
 * @param[in]	va	A pointer to a virtual DMA memory to be translated.
 *
 * @retval	Physical-address on success
 * @retval	<0 on failure
 */
#ifdef MVCONF_SYS_DMA_HUGE_PAGE
phys_addr_t mv_sys_dma_mem_virt2phys(void *va);
#else /* !MVCONF_SYS_DMA_HUGE_PAGE */
static inline phys_addr_t mv_sys_dma_mem_virt2phys(void *va)
{
	return ((u64)va - (u64)__dma_virt_base) + __dma_phys_base;
}
#endif /* MVCONF_SYS_DMA_HUGE_PAGE */

/** @} */ /* end of grp_pp2_hif */

#endif /* __MV_SYS_DMA_H__ */
