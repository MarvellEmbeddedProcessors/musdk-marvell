/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "std_internal.h"

#include "pp2_hif.h"
#include "pp2.h"
#include "pp2_dm.h"
#include "lib/lib_misc.h"

static struct pp2_hif pp2_hif[PP2_NUM_REGSPACES];

int pp2_hif_init(struct pp2_hif_params *params, struct pp2_hif **hif)
{
	int rc;
	u8 hif_slot, pp2_id, i;
	struct pp2_ppio_desc *descs;

	if (mv_sys_match(params->match, "hif", 1, &hif_slot)) {
		pr_err("[%s] Invalid match string (%s)!\n", __func__, params->match);
		return(-ENXIO);
	}
	if (pp2_is_init() == false) {
		pr_err("[%s] pp2 is not initialized\n", __func__);
		return(-EPERM);
	}

	if (hif_slot >= PP2_NUM_REGSPACES) {
		pr_err("[%s] Invalid match string!\n", __func__);
		return(-ENXIO);
	}
	if (pp2_ptr->init.hif_reserved_map & (1 << hif_slot)) {
		pr_err("[%s] hif is reserved.\n", __func__);
		return(-EFAULT);
	}
	if (pp2_ptr->pp2_common.hif_slot_map & (1 << hif_slot)) {
		pr_err("[%s] hif already exists.\n", __func__);
		return(-EEXIST);
	}

	descs = kcalloc(PP2_NUM_PKT_PROC * PP2_MAX_NUM_PUT_BUFFS, sizeof(struct pp2_ppio_desc), GFP_KERNEL);
	if (!descs)
		return(-ENOMEM);

	/* Create AGGR_TXQ for each of the PPV2 instances. */
	for (pp2_id = 0; pp2_id < pp2_ptr->num_pp2_inst; pp2_id++) {
		rc = pp2_dm_if_init(pp2_ptr, hif_slot, pp2_id, params->out_size, params->mem);
		/* Rollback created instances */
		if (rc) {
			for (i = 0; i < pp2_id; i++)
				pp2_dm_if_deinit(pp2_ptr, hif_slot, i);
			return rc;
		}
	}
	pp2_hif[hif_slot].regspace_slot = hif_slot;

	pp2_ptr->pp2_common.hif_slot_map |= (1 << hif_slot);
	*hif = &pp2_hif[hif_slot];
	(*hif)->rel_descs = descs;

	return 0;
}

void pp2_hif_deinit(struct pp2_hif *hif)
{
	u8 pp2_id;
	u8 hif_slot = hif->regspace_slot;

	if (hif_slot >= PP2_NUM_REGSPACES) {
		pr_err("[%s] Invalid hif slot %d!\n", __func__, hif_slot);
		return;
	}

	if (!(pp2_ptr->pp2_common.hif_slot_map & (1 << hif_slot))) {
		pr_err("[%s] hif slot %d does not exist.\n", __func__, hif_slot);
		return;
	}

	kfree(hif->rel_descs);

	for (pp2_id = 0; pp2_id < pp2_ptr->num_pp2_inst; pp2_id++)
		pp2_dm_if_deinit(pp2_ptr, hif_slot, pp2_id);

	pp2_ptr->pp2_common.hif_slot_map &= ~(1 << hif_slot);
}

