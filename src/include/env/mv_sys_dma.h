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

#define MV_SYS_DMA_MAX_NUM_MEM_ID		4
#define MV_SYS_DMA_MAX_MEM_ID			(MV_SYS_DMA_MAX_NUM_MEM_ID - 1)


struct mv_sys_dma_mem_region {
	void		*dma_virt_base;
	phys_addr_t	dma_phys_base;
	u64		size;
	int		manage;		/* Is this region managed by alloc and free functions */
	void		*priv;
	u32		mem_id;		/* Which DRAM will the region be allocated on. */
};

struct mv_sys_dma_mem_region_params {
	u64		size;
	int		manage;		/* Is this region managed by alloc and free functions */
	u32		mem_id;		/* Which DRAM will the region be allocated on. */
					/* TODO: change after understanding DRAM */
};

struct mv_sys_dma_mem_info {
	char		*name;
	size_t		 size;
	phys_addr_t	 paddr;
};

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
#ifdef __KERNEL__
int mv_sys_dma_mem_init(struct device *dev, size_t size);
#else
int mv_sys_dma_mem_init(size_t size);
#endif

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
 * Retrieve DMA associated info
 *
 * @param[out]	mem_info	DMA associated info
 */
int mv_sys_dma_mem_get_info(struct mv_sys_dma_mem_info *mem_info);

/**
 * Physical to Virtual address translation of an allocated DMA memory.
 *
 * @param[in]	pa	Physical Address to be translated.
 *
 * @retval	A pointer to a virtual DMA memory on success
 * @retval	<0 on failure
 */
void *mv_sys_dma_mem_phys2virt(phys_addr_t pa);


/**
 * Virtual to Physical address translation of an allocated DMA memory.
 *
 * @param[in]	va	A pointer to a virtual DMA memory to be translated.
 *
 * @retval	Physical-address on success
 * @retval	<0 on failure
 */
phys_addr_t mv_sys_dma_mem_virt2phys(void *va);

/**
 * Check validity of virtual address allocated from DMA memory.
 *
 * @param[in]	va	Virtual address to be checked.
 *
 * @retval	1 - virtual address is valid
 * @retval	0 - virtual address is invalid
 */
int mv_sys_dma_virt_is_valid(void *va);

/**
 * Check validity of physical address allocated from DMA memory.
 *
 * @param[in]	pa	Physical Address to be checked.
 *
 * @retval	1 - physical address is valid
 * @retval	0 - physical address is invalid
 */
int mv_sys_dma_phys_is_valid(phys_addr_t pa);


int mv_sys_dma_mem_region_init(struct mv_sys_dma_mem_region_params *params, struct mv_sys_dma_mem_region **mem);
void mv_sys_dma_mem_region_destroy(struct mv_sys_dma_mem_region *mem);
void *mv_sys_dma_mem_region_alloc(struct mv_sys_dma_mem_region *mem, size_t size, size_t align);
void mv_sys_dma_mem_region_free(struct mv_sys_dma_mem_region *mem, void *ptr);

static inline void *mv_sys_dma_mem_region_get_vaddr_base(struct mv_sys_dma_mem_region *mem)
{
	return mem->dma_virt_base;
}

static inline phys_addr_t mv_sys_dma_mem_region_get_paddr_base(struct mv_sys_dma_mem_region *mem)
{
	return mem->dma_phys_base;
}

#ifdef MVCONF_SYS_DMA_HUGE_PAGE
void *mv_sys_dma_mem_region_phys2virt(struct mv_sys_dma_mem_region *mem, phys_addr_t pa);
#else
static inline void *mv_sys_dma_mem_region_phys2virt(struct mv_sys_dma_mem_region *mem, phys_addr_t pa)
{
	if (!mem)
		return mv_sys_dma_mem_phys2virt(pa);
	return (void *)((pa - mem->dma_phys_base) + (phys_addr_t)mem->dma_virt_base);
}
#endif /* MVCONF_SYS_DMA_HUGE_PAGE */

#ifdef MVCONF_SYS_DMA_HUGE_PAGE
phys_addr_t mv_sys_dma_mem_region_virt2phys(struct mv_sys_dma_mem_region *mem, void *va);
#else
static inline phys_addr_t mv_sys_dma_mem_region_virt2phys(struct mv_sys_dma_mem_region *mem, void *va)
{
	if (!mem)
		return mv_sys_dma_mem_virt2phys(va);
	return ((phys_addr_t)va - (phys_addr_t)mem->dma_virt_base) + mem->dma_phys_base;
}
#endif /* MVCONF_SYS_DMA_HUGE_PAGE */

struct mv_sys_dma_mem_region *mv_sys_dma_mem_region_get(u32 mem_id);

int mv_sys_dma_mem_region_exist(u32 mem_id);
/** @} */ /* end of grp_sys_dma */

#endif /* __MV_SYS_DMA_H__ */
