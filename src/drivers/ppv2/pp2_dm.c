/**
 * @file pp2_dm.c
 *
 * Descriptor management and manipulation routines
 */
#include "pp2_types.h"

#include "pp2_hw_type.h"
#include "pp2.h"
#include "pp2_dm.h"
#include "pp2_port.h"

/* TODO: temporary we add the prototypes here until we use the musdk ones */
uintptr_t cma_calloc(size_t size);
void cma_free(uintptr_t buf);
uintptr_t cma_get_vaddr(uintptr_t buf);
uintptr_t cma_get_paddr(uintptr_t buf);

/* Internal. Creates a DM object */
int pp2_dm_if_init(struct pp2 *pp2, uint32_t dm_id, uint32_t pp2_id, uint32_t num_desc)
{
    struct pp2_inst *inst;
    struct pp2_dm_if *dm_if;

    /* Identify parent packet processor instance */
    inst = pp2->pp2_inst[pp2_id];

    dm_if = calloc(1, sizeof(struct pp2_dm_if));
    if (unlikely(!dm_if)) {
        pp2_err("DM: cannot allocate DM object\n");
        return -ENOMEM;
    }
    dm_if->id = dm_id;
    dm_if->desc_total = num_desc;

    /* Allocate a region via CMA for TXDs and setup their addresses */
    dm_if->cma_hdl = cma_calloc(num_desc * MVPP2_DESC_ALIGNED_SIZE);
    if (unlikely(!dm_if->cma_hdl)) {
        pp2_err("DM: cannot allocate DM region\n");
        free(dm_if);
        return -ENOMEM;
    }
    dm_if->desc_phys_arr = cma_get_paddr(dm_if->cma_hdl);
    if (!IS_ALIGNED(dm_if->desc_phys_arr, MVPP2_DESC_Q_ALIGN)) {
        pp2_err("DM: Descriptor array must be %u-byte aligned\n",
                MVPP2_DESC_Q_ALIGN);
        cma_free(dm_if->cma_hdl);
        free(dm_if);
        return -EPERM;
    }
    dm_if->desc_virt_arr = (struct pp2_desc *)cma_get_vaddr(dm_if->cma_hdl);
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
    cma_free(dm_if->cma_hdl);
    free(dm_if);
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
