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
