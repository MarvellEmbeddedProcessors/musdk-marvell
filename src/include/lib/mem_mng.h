/*******************************************************************************
 * Copyright (C) Marvell International Ltd. and its affiliates
 *
 * This software file (the "File") is owned and distributed by Marvell
 * International Ltd. and/or its affiliates ("Marvell") under the following
 * alternative licensing terms.  Once you have made an election to distribute the
 * File under one of the following license alternatives, please (i) delete this
 * introductory statement regarding license alternatives, (ii) delete the three
 * license alternatives that you have not elected to use and (iii) preserve the
 * Marvell copyright notice above.
 *
 ********************************************************************************
 * Marvell Commercial License Option
 *
 * If you received this File from Marvell and you have entered into a commercial
 * license agreement (a "Commercial License") with Marvell, the File is licensed
 * to you under the terms of the applicable Commercial License.
 *
 ********************************************************************************
 * Marvell GPL License Option
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File in accordance with the terms and conditions of the General
 * Public License Version 2, June 1991 (the "GPL License"), a copy of which is
 * available along with the File in the license.txt file or by writing to the Free
 * Software Foundation, Inc., or on the worldwide web at http://www.gnu.org/licenses/gpl.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY
 * DISCLAIMED.  The GPL License provides additional details about this warranty
 * disclaimer.
 *
 ********************************************************************************
 * Marvell GNU General Public License FreeRTOS Exception
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File in accordance with the terms and conditions of the Lesser
 * General Public License Version 2.1 plus the following FreeRTOS exception.
 * An independent module is a module which is not derived from or based on
 * FreeRTOS.
 * Clause 1:
 * Linking FreeRTOS statically or dynamically with other modules is making a
 * combined work based on FreeRTOS. Thus, the terms and conditions of the GNU
 * General Public License cover the whole combination.
 * As a special exception, the copyright holder of FreeRTOS gives you permission
 * to link FreeRTOS with independent modules that communicate with FreeRTOS solely
 * through the FreeRTOS API interface, regardless of the license terms of these
 * independent modules, and to copy and distribute the resulting combined work
 * under terms of your choice, provided that:
 * 1. Every copy of the combined work is accompanied by a written statement that
 * details to the recipient the version of FreeRTOS used and an offer by yourself
 * to provide the FreeRTOS source code (including any modifications you may have
 * made) should the recipient request it.
 * 2. The combined work is not itself an RTOS, scheduler, kernel or related
 * product.
 * 3. The independent modules add significant and primary functionality to
 * FreeRTOS and do not merely extend the existing functionality already present in
 * FreeRTOS.
 * Clause 2:
 * FreeRTOS may not be used for any competitive or comparative purpose, including
 * the publication of any form of run time or compile time metric, without the
 * express permission of Real Time Engineers Ltd. (this is the norm within the
 * industry and is intended to ensure information accuracy).
 *
 ********************************************************************************
 * Marvell BSD License Option
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File under the following licensing terms.
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *	* Redistributions of source code must retain the above copyright notice,
 *	  this list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the distribution.
 *
 *	* Neither the name of Marvell nor the names of its contributors may be
 *	  used to endorse or promote products derived from this software without
 *	  specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************/

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
