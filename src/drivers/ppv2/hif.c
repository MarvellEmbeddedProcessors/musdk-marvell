/**************************************************************************//**
	Copyright (C) 2016 Marvell International Ltd.
*//***************************************************************************/
#include "mv_std.h"

#include "lib/misc.h"

#include "hif.h"
#include "pp2.h"
#include "pp2_dm.h"


static struct pp2_hif pp2_hif[PP2_NUM_REGSPACES];


int pp2_hif_init(struct pp2_hif_params *params, struct pp2_hif **hif)
{
	int rc;
	u8 hif_slot, pp2_id;

	if (mv_sys_match(params->match, "hif", 1, &hif_slot))
		return(-ENXIO);
	if (pp2_is_init() == false)
		return(-EPERM);

	if (hif_slot >= PP2_NUM_REGSPACES) {
		pr_err("[%s] Invalid match string!\n", __FUNCTION__);
		return(-ENXIO);
	}
	if (pp2_ptr->init.hif_reserved_map & (1<<hif_slot)) {
		pr_err("[%s] hif is reserved.\n", __FUNCTION__);
		return(-EFAULT);
	}
	if (pp2_ptr->pp2_common.hif_slot_map & (1<<hif_slot)) {
		pr_err("[%s] hif already exists.\n", __FUNCTION__);
		return(-EEXIST);
	}
	/* Create AGGR_TXQ for each of the PPV2 instances. */
	for (pp2_id = 0; pp2_id < pp2_ptr->num_pp2_inst; pp2_id++) {
		rc = pp2_dm_if_init(pp2_ptr, hif_slot, pp2_id, params->out_size);
		if (rc) {
			if (pp2_id == PP2_ID1)
				pp2_dm_if_deinit(pp2_ptr, hif_slot, PP2_ID0);
			return(rc);
		}
	}
	pp2_hif[hif_slot].regspace_slot = hif_slot;

	pp2_ptr->pp2_common.hif_slot_map |= (1<<hif_slot);
	*hif = &pp2_hif[hif_slot];
	return(0);
}


void pp2_hif_deinit(struct pp2_hif *hif)
{
	pr_err("[%s] routine not supported yet!\n", __FUNCTION__);
}


