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

#include "std_internal.h"

#include "pp2_hif.h"
#include "pp2.h"
#include "pp2_dm.h"
#include "lib/lib_misc.h"

static struct pp2_hif pp2_hif[PP2_NUM_REGSPACES];

int pp2_hif_init(struct pp2_hif_params *params, struct pp2_hif **hif)
{
	int rc;
	u8 hif_slot, pp2_id;

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
	/* Create AGGR_TXQ for each of the PPV2 instances. */
	for (pp2_id = 0; pp2_id < pp2_ptr->num_pp2_inst; pp2_id++) {
		rc = pp2_dm_if_init(pp2_ptr, hif_slot, pp2_id, params->out_size);
		if (rc) {
			if (pp2_id == PP2_ID1)
				pp2_dm_if_deinit(pp2_ptr, hif_slot, PP2_ID0);
			return rc;
		}
	}
	pp2_hif[hif_slot].regspace_slot = hif_slot;

	pp2_ptr->pp2_common.hif_slot_map |= (1 << hif_slot);
	*hif = &pp2_hif[hif_slot];
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

	for (pp2_id = 0; pp2_id < pp2_ptr->num_pp2_inst; pp2_id++)
		pp2_dm_if_deinit(pp2_ptr, hif_slot, pp2_id);

	pp2_ptr->pp2_common.hif_slot_map &= ~(1 << hif_slot);
}

