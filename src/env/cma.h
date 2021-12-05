/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file cma.h
 *
 * Contiguous memory allocator
 *
 */

#ifndef __CMA_H__
#define __CMA_H__

#include <stdint.h>

/**
 * Initialization routine for pp contiguous memory allocator
 *
 * @retval 0 Success
 * @retval non-0 Failure.
 *
 */
int cma_init(void);
int cma_init_region(int mem_id);
bool cma_region_exist(int mem_id);

/**
 * De-initialization routine for pp contiguous memory allocator
 *
 */
void cma_deinit(void);
void cma_deinit_region(int mem_id);

/**
 * Allocates physical zeroed contiguous memory
 *
 * @param    size            memory size
 *
 * @retval Handle to the allocated memory Success
 * @retval NULL Failure.
 *
 */
void *cma_calloc(size_t size);
void *cma_calloc_region(int mem_id, size_t size);


/**
 * Free physical memory
 *
 * @param    buf             Handle to buff
 *
 */
void cma_free(void *handle);
void cma_free_region(int mem_id, void *handle);

/**
 * Get virtual address from pp CMA
 *
 * @param    buf             pointer to buff
 *
 * @retval virtual address  Success
 * @retval NULL Failure.
 *
 */
void *cma_get_vaddr(void *handle);

/**
 * Get physical address from pp CMA
 *
 * @param    buf             pointer to buff
 *
 * @retval physical address  Success
 * @retval NULL Failure.
 *
 */
phys_addr_t cma_get_paddr(void *handle);

/**
 * Get device name from pp CMA
 *
 * @param    handle             CMA object handle
 *
 * @retval pointer to CMA object device name
 */
int cma_get_dev_name(char *dev_name);
int cma_get_dev_name_region(int mem_id, char *dev_name);

/**
 * Get size from pp CMA
 *
 * @param    handle             CMA object handle
 *
 * @retval size of allocated CMA object
 */
size_t cma_get_size(void *handle);

#endif /* __CMA_H__ */
