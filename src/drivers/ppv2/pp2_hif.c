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

