/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

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
