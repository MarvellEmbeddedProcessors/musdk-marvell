/**
 * @file pp2_cma.h
 *
 * Contiguous memory allocator
 *
 */

#ifndef _PP2_CMA_H_
#define _PP2_CMA_H_

#include <stdint.h>

/**
 * Initialization routine for pp contiguous memory allocator
 *
 * @retval 0 Success
 * @retval non-0 Failure.
 *
 */
int pp2_cma_init(void);

/**
 * Allocates physical zeroed contiguous memory
 *
 * @param    size            memory size
 *
 * @retval Handle to the allocated memory Success
 * @retval NULL Failure.
 *
 */
uintptr_t pp2_cma_calloc(size_t size);

/**
 * Free physical memory
 *
 * @param    buf             Handle to buff
 *
 */
void pp2_cma_free(uintptr_t buf);

/**
 * De-initialization routine for pp contiguous memory allocator
 *
 */
void pp2_cma_deinit(void);

/**
 * Get virtual address from pp CMA
 *
 * @param    buf             pointer to buff
 *
 * @retval virtual address  Success
 * @retval NULL Failure.
 *
 */
uintptr_t pp2_cma_vaddr(uintptr_t buf);

/**
 * Get physical address from pp CMA
 *
 * @param    buf             pointer to buff
 *
 * @retval physical address  Success
 * @retval NULL Failure.
 *
 */
uintptr_t pp2_cma_paddr(uintptr_t buf);

#endif /* _PP2_CMA_H_ */
