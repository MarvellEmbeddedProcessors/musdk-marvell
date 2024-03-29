/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

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
	return (void *)(uintptr_t)((pa - mem->dma_phys_base) + (phys_addr_t)(uintptr_t)mem->dma_virt_base);
}
#endif /* MVCONF_SYS_DMA_HUGE_PAGE */

#ifdef MVCONF_SYS_DMA_HUGE_PAGE
phys_addr_t mv_sys_dma_mem_region_virt2phys(struct mv_sys_dma_mem_region *mem, void *va);
#else
static inline phys_addr_t mv_sys_dma_mem_region_virt2phys(struct mv_sys_dma_mem_region *mem, void *va)
{
	if (!mem)
		return mv_sys_dma_mem_virt2phys(va);
	return ((phys_addr_t)(uintptr_t)va - (phys_addr_t)(uintptr_t)mem->dma_virt_base) + mem->dma_phys_base;
}
#endif /* MVCONF_SYS_DMA_HUGE_PAGE */

struct mv_sys_dma_mem_region *mv_sys_dma_mem_region_get(u32 mem_id);

int mv_sys_dma_mem_region_exist(u32 mem_id);
/** @} */ /* end of grp_sys_dma */

#endif /* __MV_SYS_DMA_H__ */
