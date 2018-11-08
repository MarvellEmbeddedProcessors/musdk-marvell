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

/***********************/
/* c file declarations */
/***********************/
#include "std_internal.h"

#include "../pp2_types.h"
#include "../pp2.h"
#include "pp2_hw_cls.h"
#include "pp2_hw_cls_dbg.h"
#include "pp2_edrop.h"


/******************************************************************************
 * Function Definition
 ******************************************************************************/
static int pp2_cls_edrop_hw_entry_add(struct pp2_inst				*inst,
				      u8					edrop_id,
				      const struct pp2_cls_early_drop_params	*edrop_params)
{
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);
	int rc = 0;

	if (mv_pp2x_range_validate(edrop_id, 0, MVPP2_EDROP_MAX - 1)) {
		pr_err("invalid early-drop ID %d, out of range[%d, %d]\n", edrop_id, 0, MVPP2_EDROP_MAX - 1);
		return -EINVAL;
	}

	/* disable this early-drop */
	rc = mv_pp2x_plcr_hw_cpu_thresh_set(cpu_slot, edrop_id, edrop_params->threshold);
	if (rc) {
		pr_err("failed to set CPU threshold to HW\n");
		return rc;
	}

	return rc;
}

static int pp2_cls_edrop_hw_entry_del(struct pp2_inst *inst, u8 edrop_id)
{
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);
	int rc = 0;

	if (mv_pp2x_range_validate(edrop_id, 0, MVPP2_EDROP_MAX - 1)) {
		pr_err("invalid early-drop ID %d, out of range[%d, %d]\n", edrop_id, 0, MVPP2_EDROP_MAX - 1);
		return -EINVAL;
	}

	/* disable this early-drop */
	rc = mv_pp2x_plcr_hw_cpu_thresh_set(cpu_slot, edrop_id, MVPP2_EDROP_MAX_THESH);
	if (rc) {
		pr_err("failed to set CPU threshold to HW\n");
		return rc;
	}

	return rc;
}

static int pp2_cls_edrop_reset(struct pp2_inst *inst)
{
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);
	int i, rc = 0;

	/* init early drop DB */
	rc = pp2_cls_db_edrop_init(inst);
	if (rc) {
		pr_err("fail to init policer DB\n");
		return rc;
	}

	/* Set CPU threshold #0 and "non-reserved-entries" to maximum value
	 * to simulate "disable" mode
	 */
	pp2_cls_edrop_hw_entry_del(inst, 0);

	for (i = 0; i < MVPP2_EDROP_MAX - 1; i++) {
		if (!(pp2_ptr->init.early_drop_reserved_map & (1 << i)))
			pp2_cls_edrop_hw_entry_del(inst, (MVPP2_EDROP_MIN_ENTRY_ID + i));
	}

	/* enable early drop */
	rc = mv_pp2x_plcr_hw_early_drop_set(cpu_slot, true);
	if (rc) {
		pr_err("failed to set early drop to HW\n");
		return rc;
	}

	return rc;
}

static int pp2_cls_edrop_clear(struct pp2_inst *inst)
{
	struct pp2_cls_db_edrop_entry_t entry;
	int idx, rc = 0;

	for (idx = MVPP2_EDROP_MIN_ENTRY_ID; idx < MVPP2_EDROP_MAX; idx++) {
		/* get entry from DB */
		rc = pp2_cls_db_edrop_entry_get(inst, idx, &entry);
		if (rc) {
			pr_err("failed to get edrop entry from DB\n");
			return rc;
		}

		/* "delete" the early-drop if it is valid */
		if (entry.valid == MVPP2_PLCR_ENTRY_VALID_STATE) {
			rc = pp2_cls_edrop_entry_del(inst, idx);
			if (rc) {
				pr_err("failed to delete early-drop entry\n");
				return rc;
			}
		}
	}

	return rc;
}

int pp2_cls_edrop_entry_add(struct pp2_inst				*inst,
			    const struct pp2_cls_early_drop_params	*edrop_params,
			    u8						edrop_id)
{
	struct pp2_cls_db_edrop_entry_t entry;
	int rc = 0;

	if (mv_pp2x_ptr_validate(edrop_params))
		return -EFAULT;

	/* validate the input early-drop parmas */
	if (edrop_params->threshold > MVPP2_EDROP_MAX_THESH) {
		pr_err("threshold is above the maximum value %u\n", MVPP2_EDROP_MAX_THESH);
		return -EINVAL;
	}

	/* Add entry to HW */
	rc = pp2_cls_edrop_hw_entry_add(inst, edrop_id, edrop_params);
	if (rc) {
		pr_err("failed to add entry to HW\n");
		return rc;
	}

	/* Add entry to DB */
	MVPP2_MEMSET_ZERO(entry);
	entry.valid = MVPP2_PLCR_ENTRY_VALID_STATE;
	memcpy(&entry.edrop_entry, edrop_params, sizeof(struct pp2_cls_early_drop_params));
	rc = pp2_cls_db_edrop_entry_set(inst, edrop_id, &entry);
	if (rc) {
		pr_err("failed to save entry to DB\n");
		return rc;
	}

	return rc;
}

int pp2_cls_edrop_entry_del(struct pp2_inst *inst, u8 edrop_id)
{
	struct pp2_cls_db_edrop_entry_t entry;
	int rc = 0;

	if (mv_pp2x_range_validate(edrop_id, MVPP2_EDROP_MIN_ENTRY_ID, MVPP2_EDROP_MAX - 1)) {
		pr_err("invalid early-drop ID %d, out of range[%d, %d]\n", edrop_id,
			MVPP2_EDROP_MIN_ENTRY_ID, MVPP2_EDROP_MAX - 1);
		return -EINVAL;
	}

	/* check entry status */
	rc = pp2_cls_db_edrop_entry_get(inst, edrop_id, &entry);
	if (rc) {
		pr_err("failed to get early-drop entry from DB\n");
		return rc;
	}

	if (entry.valid == MVPP2_EDROP_ENTRY_INVALID_STATE) {
		pr_err("edrop_id(%d) is invalid, don't need to delete it\n", edrop_id);
		return -EINVAL;
	}

	if (entry.ref_cnt > 0) {
		pr_err("the early-drop is still used, ref_cnt(%d), could not be deleted\n", entry.ref_cnt);
		return -EINVAL;
	}

	/* Delete entry from HW */
	rc = pp2_cls_edrop_hw_entry_del(inst, edrop_id);
	if (rc) {
		pr_err("failed to delete early-drop entry from HW\n");
		return rc;
	}

	/* Delete entry from DB */
	entry.valid = MVPP2_PLCR_ENTRY_INVALID_STATE;
	rc = pp2_cls_db_edrop_entry_set(inst, edrop_id, &entry);
	if (rc) {
		pr_err("failed to delete early-drop entry from DB\n");
		return rc;
	}

	return rc;
}

int pp2_cls_edrop_entry_state_get(struct pp2_inst *inst, u8 edrop_id, enum pp2_cls_edrop_entry_state_t *state)
{
	struct pp2_cls_db_edrop_entry_t entry;
	int rc = 0;

	if (mv_pp2x_ptr_validate(state))
		return -EFAULT;

	if (mv_pp2x_range_validate(edrop_id, MVPP2_EDROP_MIN_ENTRY_ID, MVPP2_EDROP_MAX - 1)) {
		pr_err("invalid early-drop ID %d, out of range[%d, %d]\n", edrop_id,
			MVPP2_EDROP_MIN_ENTRY_ID, MVPP2_EDROP_MAX - 1);
		return -EINVAL;
	}

	/* check entry status */
	rc = pp2_cls_db_edrop_entry_get(inst, edrop_id, &entry);
	if (rc) {
		pr_err("failed to get early-drop entry from DB\n");
		return rc;
	}

	*state = entry.valid;

	return rc;
}

int pp2_cls_edrop_assign_qid(struct pp2_inst *inst, u8 edrop_id, u8 qid, int assign)
{
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);
	enum pp2_cls_edrop_ref_cnt_action_t cnt_action;
	int rc;

	if (mv_pp2x_range_validate(edrop_id, MVPP2_EDROP_MIN_ENTRY_ID, MVPP2_EDROP_MAX - 1)) {
		pr_err("invalid early-drop ID %d, out of range[%d, %d]\n", edrop_id,
			MVPP2_EDROP_MIN_ENTRY_ID, MVPP2_EDROP_MAX - 1);
		return -EINVAL;
	}

	if (assign) {
		cnt_action = MVPP2_EDROP_REF_CNT_INC;
		mv_pp2x_plcr_hw_rxq_thresh_set(cpu_slot, qid, edrop_id);
	} else {
		cnt_action = MVPP2_EDROP_REF_CNT_DEC;
		pp2_cls_edrop_bypass_assign_qid(inst, qid);
	}

	rc = pp2_cls_db_edrop_ref_cnt_update(inst, edrop_id, cnt_action);
	if (rc) {
		pr_err("failed to update early-drop ref count\n");
		return rc;
	}

	return rc;
}

void pp2_cls_edrop_bypass_assign_qid(struct pp2_inst *inst, u8 qid)
{
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);

	mv_pp2x_plcr_hw_rxq_thresh_set(cpu_slot, qid, MVPP2_EDROP_BYPASS_THRESH_ID);
}

int pp2_cls_edrop_start(struct pp2_inst *inst)
{
	if (pp2_cls_edrop_reset(inst) != 0) {
		pr_err("MVPP2 early-drop start failed\n");
		return -EINVAL;
	}

	return 0;
}

void pp2_cls_edrop_finish(struct pp2_inst *inst)
{
	if (pp2_cls_edrop_clear(inst) != 0)
		pr_debug("MVPP2 early-drop finish failed\n");

}

void pp2_cls_edrop_dump(struct pp2_inst *inst)
{
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);
	struct pp2_cls_db_edrop_entry_t entry;
	u32 i;
	int enable, thresh;

	mv_pp2x_plcr_hw_early_drop_get(cpu_slot, &enable);
	printk("Eraly Drop is %s\n", (enable) ? "Enable" : "Disable");
	print_horizontal_line(95, "=");
	printk("=                                     Early-Drop Thresholds Table                             =\n");
	print_horizontal_line(95, "=");
	printk("= Index | Thresh | Ref_Cnt  =\n");

	for (i = 0; i < MVPP2_EDROP_MAX; i++) {
		pp2_cls_db_edrop_entry_get(inst, i, &entry);
		mv_pp2x_plcr_hw_cpu_thresh_get(cpu_slot, i, &thresh);
		printk("= %5u | %6u | %7u  =\n",
			i, thresh, entry.ref_cnt);
	}

	print_horizontal_line(95, "=");
	printk("=				      Early-Drop Qid-Thresh Mapping			      =\n");
	print_horizontal_line(95, "=");
	printk("= Qid | Thresh-Idx =\n");

	for (i = 0; i < PP2_PPIO_MAX_NUM_INQS; i++) {
		mv_pp2x_plcr_hw_rxq_thresh_get(cpu_slot, i, &thresh);
		printk("= %3u | %10u =\n", i, thresh);
	}
}

