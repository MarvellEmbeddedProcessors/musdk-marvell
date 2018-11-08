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

#include "drivers/mv_pp2_cls.h"
#include "cls/pp2_cls_mng.h"
#include "pp2_types.h"
#include "pp2.h"
#include "pp2_hw_type.h"
#include "cls/pp2_hw_cls.h"
#include "cls/pp2_cls_db.h"
#include "lib/lib_misc.h" /* for mv_sys_match */

#define MVPP2_CLS_PROTO_SHIFT	MVPP2_CLS_PROTO_SHIFT
#define NOT_SUPPORTED_YET 255

int pp2_cls_tbl_init(struct pp2_cls_tbl_params *params, struct pp2_cls_tbl **tbl)
{
	u32 rc;

	/* Para check */
	if (mv_pp2x_ptr_validate(params))
		return -EINVAL;

	rc = pp2_cls_mng_tbl_init(params, tbl, MVPP2_CLS_LKP_MUSDK_CLS);
	if (rc) {
		pr_err("cls manager table init error\n");
		return rc;
	}

	return 0;
}

void pp2_cls_tbl_deinit(struct pp2_cls_tbl *tbl)
{
	int rc;

	/* TODO: check table type in all API functions, */

	if (mv_pp2x_ptr_validate(tbl)) {
		pr_err("%s(%d) fail, tbl = NULL\n", __func__, __LINE__);
		return;
	}

	rc = pp2_cls_mng_table_deinit(tbl);
	if (rc) {
		pr_err("cls manager table deinit error\n");
	}
}

int pp2_cls_qos_tbl_init(struct pp2_cls_qos_tbl_params *params, struct pp2_cls_tbl **tbl)
{
	u32 rc;

	/* Para check */
	if (mv_pp2x_ptr_validate(params)) {
		pr_err("%s(%d) fail, params = NULL\n", __func__, __LINE__);
		return -EINVAL;
	}

	rc = pp2_cls_mng_qos_tbl_init(params, tbl);
	if (rc) {
		pr_err("cls manager table init error\n");
		return rc;
	}

	return 0;
}

void pp2_cls_qos_tbl_deinit(struct pp2_cls_tbl *tbl)
{
	u32 rc;

	if (mv_pp2x_ptr_validate(tbl)) {
		pr_err("%s(%d) fail, tbl = NULL\n", __func__, __LINE__);
	}

	rc = pp2_cls_mng_qos_tbl_deinit(tbl);
	if (rc)
		pr_err("cls manager table init error: %d\n", rc);

	return;
}

int pp2_cls_tbl_add_rule(struct pp2_cls_tbl		*tbl,
			 struct pp2_cls_tbl_rule	*rule,
			 struct pp2_cls_tbl_action	*action)
{
	u32 rc;

	/* Para check */
	if (mv_pp2x_ptr_validate(tbl))
		return -EINVAL;

	if (mv_pp2x_ptr_validate(rule))
		return -EINVAL;

	if (mv_pp2x_ptr_validate(action))
		return -EINVAL;

	rc = pp2_cls_mng_rule_add(tbl, rule, action, MVPP2_CLS_LKP_MUSDK_CLS);
	if (rc)
		pr_err("cls mng: unable to add rule\n");

	return rc;
}

int pp2_cls_tbl_modify_rule(struct pp2_cls_tbl		*tbl,
			    struct pp2_cls_tbl_rule	*rule,
			    struct pp2_cls_tbl_action	*action)
{
	u32 rc;

	/* Para check */
	if (mv_pp2x_ptr_validate(tbl))
		return -EINVAL;

	if (mv_pp2x_ptr_validate(rule))
		return -EINVAL;

	if (mv_pp2x_ptr_validate(action))
		return -EINVAL;

	rc = pp2_cls_mng_rule_modify(tbl, rule, action);
	if (rc)
		pr_err("cls mng: unable to modify rule\n");

	return rc;
}

int pp2_cls_tbl_remove_rule(struct pp2_cls_tbl		*tbl,
			    struct pp2_cls_tbl_rule	*rule)
{
	u32 rc;

	/* Para check */
	if (mv_pp2x_ptr_validate(tbl))
		return -EINVAL;

	if (mv_pp2x_ptr_validate(rule))
		return -EINVAL;

	rc = pp2_cls_mng_rule_remove(tbl, rule);
	if (rc)
		pr_err("cls mng: unable to remove rule\n");

	return rc;
}

int pp2_cls_plcr_init(struct pp2_cls_plcr_params *params, struct pp2_cls_plcr **plcr)
{
	u8 match[2];
	int policer_id, pp2_id, rc;
	enum pp2_cls_plcr_entry_state_t state;

	/* Para check */
	if (mv_pp2x_ptr_validate(params))
		return -EINVAL;

	if (mv_sys_match(params->match, "policer", 2, match))
		return(-ENXIO);

	if (pp2_is_init() == false)
		return(-EPERM);

	pp2_id = match[0];
	policer_id = match[1];

	if (policer_id < 0 || policer_id >= PP2_CLS_PLCR_NUM) {
		pr_err("[%s] Invalid match string!\n", __func__);
		return(-ENXIO);
	}

	if (pp2_id < 0 || pp2_id >= pp2_ptr->num_pp2_inst) {
		pr_err("[%s] Invalid match string!\n", __func__);
		return(-ENXIO);
	}

	if (pp2_ptr->init.policers_reserved_map & (1 << policer_id)) {
		pr_err("[%s] policer-id is reserved.\n", __func__);
		return(-EFAULT);
	}

	rc = pp2_cls_plcr_entry_state_get(pp2_ptr->pp2_inst[pp2_id], policer_id+1, &state);
	if (rc || state == MVPP2_PLCR_ENTRY_VALID_STATE) {
		pr_err("[%s] policer already exists.\n", __func__);
		return(-EEXIST);
	}

	*plcr = kmalloc(sizeof(struct pp2_cls_plcr), GFP_KERNEL);
	if (!*plcr)
		return -ENOMEM;
	(*plcr)->pp2_id = pp2_id;
	(*plcr)->id = policer_id+1;
	rc = pp2_cls_plcr_entry_add(pp2_ptr->pp2_inst[pp2_id], params, policer_id+1);
	if (rc) {
		kfree(*plcr);
		*plcr = NULL;
	}

	return rc;
}

void pp2_cls_plcr_deinit(struct pp2_cls_plcr *plcr)
{
	int rc;

	if (mv_pp2x_ptr_validate(plcr))
		pr_err("%s(%d) fail, plcr = NULL\n", __func__, __LINE__);

	rc = pp2_cls_plcr_entry_del(pp2_ptr->pp2_inst[plcr->pp2_id], plcr->id);
	if (rc)
		pr_err("[%s] cls policer deinit error\n", __func__);
}

int pp2_cls_early_drop_init(struct pp2_cls_early_drop_params *params, struct pp2_cls_early_drop **edrop)
{
	u8 match[2];
	int ed_id, pp2_id, rc;
	enum pp2_cls_edrop_entry_state_t state;

	/* Para check */
	if (mv_pp2x_ptr_validate(params))
		return -EINVAL;

	if (mv_sys_match(params->match, "ed", 2, match)) {
		pr_err("[%s] Invalid match string!\n", __func__);
		return(-ENXIO);
	}

	if (pp2_is_init() == false)
		return(-EPERM);

	pp2_id = match[0];
	ed_id = match[1];

	if (ed_id < 0 || ed_id >= PP2_CLS_EARLY_DROP_NUM) {
		pr_err("[%s] Invalid early-drop id!\n", __func__);
		return(-ENXIO);
	}

	if (pp2_id < 0 || pp2_id >= pp2_ptr->num_pp2_inst) {
		pr_err("[%s] Invalid pp2-id string!\n", __func__);
		return(-ENXIO);
	}

	if (pp2_ptr->init.early_drop_reserved_map & (1 << ed_id)) {
		pr_err("[%s] early-drop id is reserved.\n", __func__);
		return(-EFAULT);
	}

	rc = pp2_cls_edrop_entry_state_get(pp2_ptr->pp2_inst[pp2_id], ed_id + 1, &state);
	if (rc || state == MVPP2_EDROP_ENTRY_VALID_STATE) {
		pr_err("[%s] early-drop already exists.\n", __func__);
		return(-EEXIST);
	}

	*edrop = kmalloc(sizeof(struct pp2_cls_early_drop), GFP_KERNEL);
	if (!*edrop)
		return -ENOMEM;
	(*edrop)->pp2_id = pp2_id;
	(*edrop)->id = ed_id + 1;
	rc = pp2_cls_edrop_entry_add(pp2_ptr->pp2_inst[pp2_id], params, (*edrop)->id);
	if (rc) {
		kfree(*edrop);
		*edrop = NULL;
	}

	return rc;
}

void pp2_cls_early_drop_deinit(struct pp2_cls_early_drop *edrop)
{
	int rc;

	if (mv_pp2x_ptr_validate(edrop))
		pr_err("%s(%d) fail, edrop = NULL\n", __func__, __LINE__);

	rc = pp2_cls_edrop_entry_del(pp2_ptr->pp2_inst[edrop->pp2_id], edrop->id);
	if (rc)
		pr_err("[%s] cls early-drop deinit error\n", __func__);
}

