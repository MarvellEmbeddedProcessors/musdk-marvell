/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file hugepage_mem.h
 *
 * Huge page memory allocation
 *
 */

#ifndef __HUGEPAGE_MEM_H__
#define __HUGEPAGE_MEM_H__

#include <stdint.h>

#define HUGE_PAGE_MAX_PAGE_COUNT	64
struct sys_hugepage;
/**
 * Initialization routine for pp huge page memory allocator
 *
 * @param    size		memory size
 * @param    dma_virt_base	pointer to virtual address base of allocated memory
 *
 * @retval Huge Page struct pointer as void: non-NULL Success
 * @retval NULL Failure.
 *
 */
void *hugepage_init_mem(u64 size, void **dma_virt_base);

/**
 * Free physical memory
 *
 * @param    hugepage		pointer to allocated hugepage struct
 *
 */
void hugepage_free_mem(struct sys_hugepage *hugepage);

/**
 * retrieve PA from VA, by scanning process page map for VA, and get it's PA
 *
 * @param    vaddr		requested virtual address
 * @retval   intptr_t 		pointer of Physical address for requested virtual address
 *
 */
intptr_t hugepage_virt2phys_scan_proc_pagemap(void *vaddr);

/**
 * Retrieve Huge Page size from VA, by parsing /proc/meminfo for Hugepagesize entry
 * and verify that there is enough pages to allocate requested size
 *
 * @param    size		requested memory size to be allocated
 *
 * @retval Huge Page size used by kernel Success
 * @retval 0 Failure.
 */
u64 hugepage_verify_hugepage_pagesize(u64 size);

#endif /* __HUGEPAGE_MEM_H__ */
