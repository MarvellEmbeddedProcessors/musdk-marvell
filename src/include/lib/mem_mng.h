/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __MEM_MNG_H__
#define __MEM_MNG_H__

#include "env/mv_types.h"

/** Alignments between 2-128 are available where the maximum alignment defined
 * as 2^MEM_MNG_MAX_ALIGNMENT
 */
#define MEM_MNG_MAX_ALIGNMENT	20

/** TODO
 */
#define MEM_MNG_MAX_NAME_LEN	32
#define MEM_MNG_ILLEGAL_BASE	(-1)

struct mem_mng;

/**
 * Initializes a new MM object.
 *
 * It initializes a new memory block consisting of base address
 * and size of the available memory by calling to MemBlock_Init
 * routine. It is also initializes a new free block for each
 * by calling FreeBlock_Init routine, which is pointed to
 * the almost all memory started from the required alignment
 * from the base address and to the end of the memory.
 * The handle to the new MM object is returned via "MM"
 * argument (passed by reference).
 *
 * @param[in]	base	Base address of the MM.
 * @param[in]	size	Size of the MM block.
 * @param[out]	mm		Handle to the MM object.
 *
 * @retval	0 is returned on success.
 * @retval	ENOMEM is returned if the new MM object or a new free block
 *			can not be initialized.
 */
int mem_mng_init(u64 base, u64 size, struct mem_mng **mm);

/**
 * Releases memory allocated for MM object.
 *
 * @param[in]	mm		A handle to the MM object.
 */
void mem_mng_free(struct mem_mng *mm);

/**
 * Allocates a block of memory according to the given size and the alignment.
 *
 * The Alignment argument tells from which
 * free list allocate a block of memory. 2^alignment indicates
 * the alignment that the base address of the allocated block
 * should have. So, the only values 1, 2, 4, 8, 16, 32 and 64
 * are available for the alignment argument.
 * The routine passes through the specific free list of free
 * blocks and seeks for a first block that have anough memory
 * that  is required (best fit).
 * After the block is found and data is allocated, it calls
 * the internal mem_mng_CutFree routine to update all free lists
 * do not include a just allocated block. Of course, each
 * free list contains a free blocks with the same alignment.
 * It is also creates a busy block that holds
 * information about an allocated block.
 *
 * @param[in]	mm			A handle to the MM object.
 * @param[in]	size		Size of the memory requested.
 * @param[in]	alignment	Size of the memory requested.
 * @param[in]	name		The name that specifies an allocated block.
 *
 * @retval	base address of an allocated block.
 * @retval	MEM_MNG_ILLEGAL_BASE if can't allocate a block.
 */
u64 mem_mng_get(struct mem_mng *mm, u64 size, u64 alignment, const char *name);

/**
 * Puts a block of memory of the given base address back to the memory.
 *
 * It checks if there is a busy block with the given base address.
 * If not, it returns 0, that means can't free a block. Otherwise, it gets
 * parameters of the busy block and after it updates lists of free blocks,
 * removes that busy block from the list by calling to mem_mng_CutBusy routine.
 * After that it calls to mem_mng_AddFree routine to add a new free
 * block to the free lists.
 *
 * @param[in]	mm			A handle to the MM object.
 * @param[in]	base		Base address of the MM.
 *
 * @retval	base address of an allocated block.
 * @retval	MEM_MNG_ILLEGAL_BASE if can't allocate a block.
 */
u64 mem_mng_put(struct mem_mng *mm, u64 base);

/**
 * Checks if a specific address is in the memory range of the passed MM object.
 *
 * @param[in]	mm			A handle to the MM object.
 * @param[in]	addr		The address to be checked.
 *
 * @retval	1 if the address is in the address range of the block.
 * @retval	0 if the address is NOT in the address range of the block.
 */
int mem_mng_in_range(struct mem_mng *mm, u64 addr);

/**
 * Returns the size (in bytes) of free memory.
 *
 * @param[in]	mm			A handle to the MM object.
 *
 * @retval	Free memory size in bytes.
 */
u64 mem_mng_get_avail_mem(struct mem_mng *mm);

/**
 * Prints results of free and busy lists.
 *
 * @param[in]	mm			A handle to the MM object.
 */
void mem_mng_dump(struct mem_mng *mm);

#endif /* __MEM_MNG_H___ */
