/**
 * @file cma.h
 *
 * Contiguous memory allocator
 *
 */

#ifndef __CMA_H__
#define __CMA_H__

#include "mv_std.h"

/**
 * Initialization routine for pp contiguous memory allocator
 *
 * @retval 0 Success
 * @retval non-0 Failure.
 *
 */
int cma_init(void);

/**
 * De-initialization routine for pp contiguous memory allocator
 *
 */
void cma_deinit(void);

/**
 * Allocates physical zeroed contiguous memory
 *
 * @param    size            memory size
 *
 * @retval Handle to the allocated memory Success
 * @retval NULL Failure.
 *
 */
uintptr_t cma_calloc(size_t size);

/**
 * Free physical memory
 *
 * @param    buf             Handle to buff
 *
 */
void cma_free(uintptr_t buf);

/**
 * Get virtual address from pp CMA
 *
 * @param    buf             pointer to buff
 *
 * @retval virtual address  Success
 * @retval NULL Failure.
 *
 */
uintptr_t cma_get_vaddr(uintptr_t buf);

/**
 * Get physical address from pp CMA
 *
 * @param    buf             pointer to buff
 *
 * @retval physical address  Success
 * @retval NULL Failure.
 *
 */
uintptr_t cma_get_paddr(uintptr_t buf);

#endif /* __CMA_H__ */
