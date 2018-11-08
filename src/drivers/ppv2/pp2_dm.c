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
 * @file pp2_dm.c
 *
 * Descriptor management and manipulation routines
 */
#include "std_internal.h"
#include "pp2_types.h"

#include "pp2_hw_type.h"
#include "pp2.h"
#include "pp2_dm.h"
#include "pp2_port.h"


static void dm_lock_create(struct pp2_dm_if *dm_if)
{
#ifdef MVCONF_PP2_LOCK
	dm_if->dm_lock.lock = spin_lock_create();
#endif
}

static void dm_lock_destroy(struct pp2_dm_if *dm_if)
{
#ifdef MVCONF_PP2_LOCK
	spin_lock_destroy(dm_if->dm_lock.lock);
#endif
}

static void pp2_dm_aggr_queue_config(struct pp2_dm_if *dm_if, uintptr_t addr, u32 size)
{
	pp2_reg_write(dm_if->cpu_slot, MVPP2_AGGR_TXQ_INIT(dm_if->id), 0x01);
	pp2_reg_write(dm_if->cpu_slot, MVPP2_AGGR_TXQ_DESC_ADDR_REG(dm_if->id), addr >> MVPP22_DESC_ADDR_SHIFT);
	pp2_reg_write(dm_if->cpu_slot, MVPP2_AGGR_TXQ_DESC_SIZE_REG(dm_if->id), size);
}

/* Internal. Creates a DM object */
int pp2_dm_if_init(struct pp2 *pp2, uint32_t dm_id, uint32_t pp2_id, uint32_t num_desc,
		   struct mv_sys_dma_mem_region *mem)
{
	struct pp2_inst *inst;
	struct pp2_dm_if *dm_if;

	/* Identify parent packet processor instance */
	inst = pp2->pp2_inst[pp2_id];

	dm_if = kcalloc(1, sizeof(struct pp2_dm_if), GFP_KERNEL);
	if (unlikely(!dm_if)) {
		pr_err("DM: cannot allocate DM object\n");
		return -ENOMEM;
	}
	dm_if->id = dm_id;
	dm_if->desc_total = num_desc;
	dm_if->mem = mem;

	/* Allocate a region via CMA for TXDs and setup their addresses */
	dm_if->desc_virt_arr = mv_sys_dma_mem_region_alloc(mem, (num_desc * MVPP2_DESC_ALIGNED_SIZE),
							    MVPP2_DESC_Q_ALIGN);
	if (unlikely(!dm_if->desc_virt_arr)) {
		pr_err("DM: cannot allocate DM region\n");
		kfree(dm_if);
		return -ENOMEM;
	}
	dm_if->desc_phys_arr = (uintptr_t)mv_sys_dma_mem_region_virt2phys(mem, dm_if->desc_virt_arr);
	if (!IS_ALIGNED(dm_if->desc_phys_arr, MVPP2_DESC_Q_ALIGN)) {
		pr_err("DM: Descriptor array must be %u-byte aligned\n",
			MVPP2_DESC_Q_ALIGN);
		mv_sys_dma_mem_region_free(mem, dm_if->desc_virt_arr);
		kfree(dm_if);
		return -EPERM;
	}
	/* Get register address space slot for this DM object */
	dm_if->cpu_slot = inst->hw.base[dm_id].va;

	/* Initialize the aggregation queue under this DM object */
	pp2_dm_aggr_queue_config(dm_if, dm_if->desc_phys_arr, dm_if->desc_total);
	dm_if->desc_next_idx = pp2_reg_read(dm_if->cpu_slot, MVPP2_AGGR_TXQ_INDEX_REG(dm_if->id));

	/* Save this DM object in its packet processor unique slot */
	dm_if->parent = inst;
	inst->dm_ifs[dm_id] = dm_if;
	inst->num_dm_ifs++;

	/* Create dm_lock for aggregation_queue locking */
	dm_lock_create(dm_if);
	pr_debug("DM:(AQ%u)(PP%u) created\n", dm_id, pp2_id);

	return 0;
}

/* External. Destroys a DM object */
void pp2_dm_if_deinit(struct pp2 *pp2, uint32_t dm_id, uint32_t pp2_id)
{
	struct pp2_dm_if *dm_if;
	struct pp2_inst *inst;

	inst = pp2->pp2_inst[pp2_id];
	dm_if = inst->dm_ifs[dm_id];

	if (!dm_if)
		return;

	/* Reset the aggregation queue under this DM object */
	pp2_dm_aggr_queue_config(dm_if, 0, 0);
	dm_lock_destroy(dm_if);

	pr_debug("DM: (AQ%u)(PP%u) destroyed\n", dm_if->id, inst->id);
	mv_sys_dma_mem_region_free(dm_if->mem, dm_if->desc_virt_arr);
	kfree(dm_if);
	inst->num_dm_ifs--;
	inst->dm_ifs[dm_id] = NULL;
}

/* External.
 */
uint32_t
pp2_dm_if_get_desc(struct pp2_dm_if *dm_if, struct pp2_desc **out_desc, uint32_t req_desc)
{
	u32 free_desc;
	u32 next_idx;
	u32 act_desc;
	u32 occ_desc;
	u32 total_desc;

	/* Number of occupied descriptors, if any, are present before the
	* descriptor next index in the aggregator
	*/
	occ_desc = pp2_reg_read(dm_if->cpu_slot,
				MVPP2_AGGR_TXQ_STATUS_REG(dm_if->id)) & MVPP2_AGGR_TXQ_PENDING_MASK;

	next_idx   = dm_if->desc_next_idx;
	total_desc = dm_if->desc_total;
	act_desc   = req_desc;

	free_desc = (next_idx > occ_desc) ? (total_desc - next_idx) : (total_desc - occ_desc);

	if (unlikely(req_desc > free_desc))
	act_desc = free_desc;

	dm_if->desc_next_idx = (next_idx + act_desc) % total_desc;

	*out_desc = (act_desc > 0) ? (dm_if->desc_virt_arr + next_idx) : NULL;

	return act_desc;
}

/* Descriptor layout */
void
pp2_dm_desc_dump(struct pp2_desc *desc)
{
	pr_debug("DESC (%p):\n", desc);
	pr_debug("CMD0:  0x%08X\n", desc->cmd0);
	pr_debug("CMD1:  0x%08X\n", desc->cmd1);
	pr_debug("CMD2:  0x%08X\n", desc->cmd2);
	pr_debug("CMD3:  0x%08X\n", desc->cmd3);
	pr_debug("CMD4:  0x%08X\n", desc->cmd4);
	pr_debug("CMD5:  0x%08X\n", desc->cmd5);
	pr_debug("CMD6:  0x%08X\n", desc->cmd6);
	pr_debug("CMD7:  0x%08X\n", desc->cmd7);
	}
