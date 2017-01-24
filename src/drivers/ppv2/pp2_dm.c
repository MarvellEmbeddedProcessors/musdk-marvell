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


/* Internal. Creates a DM object */
int pp2_dm_if_init(struct pp2 *pp2, uint32_t dm_id, uint32_t pp2_id, uint32_t num_desc)
{
    struct pp2_inst *inst;
    struct pp2_dm_if *dm_if;

    /* Identify parent packet processor instance */
    inst = pp2->pp2_inst[pp2_id];

    dm_if = kcalloc(1, sizeof(struct pp2_dm_if), GFP_KERNEL);
    if (unlikely(!dm_if)) {
        pp2_err("DM: cannot allocate DM object\n");
        return -ENOMEM;
    }
    dm_if->id = dm_id;
    dm_if->desc_total = num_desc;

    /* Allocate a region via CMA for TXDs and setup their addresses */
    dm_if->desc_virt_arr = (struct pp2_desc *)mv_sys_dma_mem_alloc((num_desc * MVPP2_DESC_ALIGNED_SIZE), MVPP2_DESC_Q_ALIGN);
    if (unlikely(!dm_if->desc_virt_arr)) {
        pp2_err("DM: cannot allocate DM region\n");
        kfree(dm_if);
        return -ENOMEM;
    }
    dm_if->desc_phys_arr = (uintptr_t)mv_sys_dma_mem_virt2phys(dm_if->desc_virt_arr);
    if (!IS_ALIGNED(dm_if->desc_phys_arr, MVPP2_DESC_Q_ALIGN)) {
        pp2_err("DM: Descriptor array must be %u-byte aligned\n",
                MVPP2_DESC_Q_ALIGN);
        mv_sys_dma_mem_free(dm_if->desc_virt_arr);
        kfree(dm_if);
        return -EPERM;
    }
    /* Get register address space slot for this DM object */
    dm_if->cpu_slot = inst->hw.base[dm_id].va;

    /* Initialize the aggregation queue under this DM object */
    pp2_reg_write(dm_if->cpu_slot, MVPP2_AGGR_TXQ_INIT(dm_if->id), 0x01);

    dm_if->desc_next_idx = pp2_reg_read(dm_if->cpu_slot,
            MVPP2_AGGR_TXQ_INDEX_REG(dm_if->id));
    pp2_reg_write(dm_if->cpu_slot, MVPP2_AGGR_TXQ_DESC_ADDR_REG(dm_if->id),
            dm_if->desc_phys_arr >> MVPP22_DESC_ADDR_SHIFT);
    pp2_reg_write(dm_if->cpu_slot, MVPP2_AGGR_TXQ_DESC_SIZE_REG(dm_if->id),
            dm_if->desc_total);

    /* Save this DM object in its packet processor unique slot */
    dm_if->parent = inst;
    inst->dm_ifs[dm_id] = dm_if;
    inst->num_dm_ifs++;

    pp2_dbg("DM:(AQ%u)(PP%u) created\n", dm_id, pp2_id);

    return 0;
}

/* External. Destroys a DM object */
void pp2_dm_if_deinit(struct pp2 *pp2, uint32_t dm_id, uint32_t pp2_id)
{
    struct pp2_dm_if *dm_if;
    struct pp2_inst *inst;

    inst = pp2->pp2_inst[pp2_id];
    dm_if = inst->dm_ifs[dm_id];

    if (NULL == dm_if)
        return;

    /* TODO: Destroy AQ in HW */

    pp2_dbg("DM: (AQ%u)(PP%u) destroyed\n", dm_if->id, inst->id);
    mv_sys_dma_mem_free(dm_if->desc_virt_arr);
    kfree(dm_if);
    inst->num_dm_ifs--;
    inst->dm_ifs[dm_id] = NULL;
}

/* External.
 */
uint32_t
pp2_dm_if_get_desc(struct pp2_dm_if *dm_if, struct pp2_desc **out_desc, uint32_t req_desc)
{
    uint32_t free_desc;
    uint32_t next_idx;
    uint32_t act_desc;
    uint32_t occ_desc;
    uint32_t total_desc;

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
    pp2_dbg("DESC (%p):\n", desc);
    pp2_dbg("CMD0:  0x%08X\n", desc->cmd0);
    pp2_dbg("CMD1:  0x%08X\n", desc->cmd1);
    pp2_dbg("CMD2:  0x%08X\n", desc->cmd2);
    pp2_dbg("CMD3:  0x%08X\n", desc->cmd3);
    pp2_dbg("CMD4:  0x%08X\n", desc->cmd4);
    pp2_dbg("CMD5:  0x%08X\n", desc->cmd5);
    pp2_dbg("CMD6:  0x%08X\n", desc->cmd6);
    pp2_dbg("CMD7:  0x%08X\n", desc->cmd7);
}
