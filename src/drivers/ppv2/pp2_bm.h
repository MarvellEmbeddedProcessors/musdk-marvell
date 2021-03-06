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

/**
 * @file pp2_bm.h
 *
 * Packet Processor Buffer Manager
 *
 * Presentation API for PPDK hardware managed memory resources
 */

#ifndef _PP2_BM_H_
#define _PP2_BM_H_

#include "std_internal.h"

#define BM_TYPE_SHORT_BUF_POOL (0x00)
#define BM_TYPE_LONG_BUF_POOL  (0x01)

/* Number of buffer references per BPPE unit. Client should
 * request a number of buffers divisible to this this value
 */
#define PP2_BPPE_UNIT_SIZE           (8)

/* Forward declarations for structures required by BM internals */
struct pp2;
struct pp2_bm_pool;
struct pp2_port;

/**
 * pp2_bm_if_param
 *
 * Buffer Manager interface object parameters
 *
 * @field	id    BM-IF ID. This ID should be assigned uniquely per
 *                BM software requestor.
 *
 *                Valid range: [0 - PP2_TOTAL_NUM_REGSPACES]
 *                See <pp2_plat.h>.
 */
struct pp2_bm_if_param {
	u32 id;
};

/**
 * bm_pool_param
 *
 * Buffer Manager pool parameters
 *
 * @field	id        BM Pool unique logical ID
 *
 *                    Valid range: [0 - PP2_TOTAL_NUM_BMPOOLS]
 *                    See <pp2_plat.h>
 *
 * @field	buf_num   Number of buffers that shall be managed
 *                    by BM for this buffer pool. This value should be
 *                    a multiple of PP2_BPPE_UNIT_SIZE
 *
 * @field   buf_size  Size of each buffer that resides
 *                    in this BM Pool
 */
struct bm_pool_param {
	u32 id;
	u32 pp2_id;
	u32 buf_num;
	u32 buf_size;
	int dummy_pool;
	struct mv_sys_dma_mem_region *likely_buffer_mem;
};

/**
 * pp2_bm_if_init
 *
 * Initialize (create) and prepare an exclusive (lockless)
 * BM-IF object for accessing BM resources
 *
 * @param ppdk       The PPDK handle. Based on the param input
 *                   the corresponding packet processor shall
 *                   be used
 * @param param      Parameters for this BM-IF object
 *
 * @retval           BM-IF object handle on success, NULL otherwise
 */
struct pp2_bm_if *pp2_bm_if_init(struct pp2 *ppdk, struct pp2_bm_if_param *param);

/**
 * pp2_bm_pool_create
 *
 * Create and associate a BM pool to the BM object provided as
 * input. Multiple BM objects can share the same BM pool.
 * The BM pool is not usable after this routines returns
 *
 * @param bm_if      BM-IF object handle
 * @param param      BM Pool parameters
 *
 * @retval           BM Pool handle on success, NULL otherwise
 */
int pp2_bm_pool_create(struct pp2 *pp2, struct bm_pool_param *param);

/**
 * pp2_bm_pool_alloc_memory
 *
 * Allocate a software chunk of memory and associate it to the
 * input BM pool handle.
 *
 * This should be spitted into buffers by caller and these buffers
 * have to be released to pool by calling pp2_bm_buf_put() on each buffer
 *
 * @param bm_pool    BM Pool handle
 * @param size       Size of the memory that shall be managed
 *
 * @retval           Valid handle to allocate memory, 0 otherwise
 */
uintptr_t pp2_bm_pool_alloc_memory(struct pp2_bm_pool *bm_pool,
				   size_t size);

/**
 * pp2_bm_pool_destroy
 *
 * Destroy this BM pool from the input BM-If object. After
 * this routine returns successfully, the BM pool is free
 * and can be re-initialized and setup against another
 * BM-IF object at a later time.
 *
 * @param bm_if      Original BM-IF object pool owner
 * @param bm_pool    BM Pool handle
 *
 * @retval           0 on success, error otherwise
 */
int pp2_bm_pool_destroy(uintptr_t cpu_slot,
			struct pp2_bm_pool *bm_pool);

/**
 * pp2_bm_if_deinit
 *
 * Deinitialize (destroy) a BM-IF object. After this
 * routine returns successfully, all BM pools associated
 * with it shall also be destroyed
 *
 * @param bm_if      BM-IF object handle
 *
 * @retval           0 on success, error otherwise
 */
void pp2_bm_if_deinit(struct pp2_bm_if *bm_if);

/**
 * pp2_bm_pool_get_id
 *
 * BM Pool get physical ID
 *
 * @param bm_pool    BM Pool handle
 *
 * @retval physical ID associated with this BM pool handle
 */
uint32_t pp2_bm_pool_get_id(struct pp2_bm_pool *bm_pool);

/**
 * pp2_bm_pool_get_pool_by_id
 *
 * Get pointer to buffer_pool configuration
 * in specified PP@ Instance, and pool_id
 *
 * @param pp2_inst   PP2 Instance handle
 * @param pool_id    pool id
 *
 * @retval           BM Pool handle, NULL if not found
 */
struct pp2_bm_pool *pp2_bm_pool_get_pool_by_id(struct pp2_inst *pp2_inst, uint32_t pool_id);

/**
 * pp2_bm_pool_assign
 *
 * Assign BM Pool to Port RXQ(s) for SWF
 * Associate port RXQ with this pool.
 * RxDMA shall construct RXDs using this BM pool
 *
 * TODO: Why this function stays here and it is not a part of RX queue code?
 *
 * @param port       pointer to PP port
 * @param pool_id    pool id
 * @param rxq_id     ingress physical queue id
 * @param type       pool type: long(jumbo)/short
 *
 */
void pp2_bm_pool_assign(struct pp2_port *port, uint32_t pool_id,
			u32 rxq_id, uint32_t type);

/**
 * pp2_bm_pa2va
 *
 * Translate physical address of buffer to virtual address
 *
 * @param bm_pool    BM Pool handle
 * @param pa         buffer physical address
 *
 * @retval           buffer virtual address
 */
uintptr_t pp2_bm_pa2va(struct pp2_bm_pool *bm_pool, uintptr_t pa);

/**
 * pp2_bm_va2pa
 *
 * Translate virtual address of buffer to physical address
 *
 * @param bm_pool    BM Pool handle
 * @param va         buffer virtual address
 *
 * @retval           buffer physical address
 */
uintptr_t pp2_bm_va2pa(struct pp2_bm_pool *bm_pool, uintptr_t va);

uint32_t pp2_bm_hw_pool_create(uintptr_t cpu_slot, uint32_t pool_id,
			       u32 bppe_num, uintptr_t pool_phys_addr);

void pp2_bm_port_add(struct pp2_inst *pp2_inst, u32 pool_id, u32 port_id, int pool_stop_bufs, int pool_start_bufs);
void pp2_bm_port_remove(struct pp2_inst *pp2_inst, u32 pool_id, u32 port_id, int pool_stop_bufs, int pool_start_bufs);
void pp2_bm_update_fc_thresh(struct pp2_inst *pp2_inst, u32 pool_id, int pool_stop_bufs, int pool_start_bufs);
void pp2_bm_pool_reset_fc(uintptr_t base, struct pp2_bm_pool *pool);


#endif /* _PP2_BM_H_ */
