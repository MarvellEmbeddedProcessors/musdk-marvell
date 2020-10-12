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
#include "mng/mv_nmp_guest_pp2_cls.h"

#include "lf/mng_cmd_desc.h"

#include "nmp_guest.h"

static void prepare_action(struct guest_pp2_cls_tbl_action *dst_action,
			   struct pp2_cls_tbl_action *src_action,
			   uintptr_t base)
{
	memcpy(&dst_action->action, src_action, sizeof(struct pp2_cls_tbl_action));
	if (src_action->cos) {
		memcpy(&dst_action->cos, src_action->cos, sizeof(struct pp2_cls_cos_desc));
		dst_action->action.cos = (void *)((uintptr_t)&dst_action->cos - base);
	}
}

static void prepare_rule(struct guest_pp2_cls_tbl_rule *dst_rule,
			   struct pp2_cls_tbl_rule *src_rule)
{
	int i;

	dst_rule->num_fields = src_rule->num_fields;
	for (i = 0; i < dst_rule->num_fields; i++) {
		dst_rule->fields[i].size = src_rule->fields[i].size;
		dst_rule->fields[i].key_valid = 0;
		dst_rule->fields[i].mask_valid = 0;
		if (src_rule->fields[i].key) {
			dst_rule->fields[i].key_valid = 1;
			memcpy(dst_rule->fields[i].key, src_rule->fields[i].key, dst_rule->fields[i].size);
		}
		if (src_rule->fields[i].mask) {
			dst_rule->fields[i].mask_valid = 1;
			memcpy(dst_rule->fields[i].mask, src_rule->fields[i].mask, dst_rule->fields[i].size);
		}
	}
}

int nmp_guest_pp2_cls_tbl_init(struct nmp_guest *guest,
			       struct pp2_cls_tbl_params *params,
			       struct pp2_cls_tbl **tbl)
{
	struct guest_pp2_cls_tbl_params tbl_params;
	struct guest_cmd_resp resp;
	int ret;

	memcpy(&tbl_params.params, params, sizeof(struct pp2_cls_tbl_params));
	prepare_action(&tbl_params.def_action, &params->default_act, (uintptr_t)&tbl_params);
	ret = send_internal_msg(guest, CDT_PF, guest->lf_master_id, MSG_F_GUEST_TABLE_INIT, 0,
		&tbl_params, sizeof(tbl_params), &resp, sizeof(resp));
	if (ret)
		return ret;
	if (resp.status == RESP_STATUS_FAIL)
		return -1;
	*tbl = (struct pp2_cls_tbl *)(uintptr_t)(resp.pp2_cls_resp.tbl_init.tbl_id + 1);

	return 0;
}

void nmp_guest_pp2_cls_tbl_deinit(struct nmp_guest *guest, struct pp2_cls_tbl *tbl)
{
	struct guest_cmd_resp resp;
	u32 tbl_id = ((u32)(uintptr_t)tbl) - 1;
	int ret;

	ret = send_internal_msg(guest, CDT_PF, guest->lf_master_id, MSG_F_GUEST_TABLE_DEINIT, 0,
		&tbl_id, sizeof(tbl_id), &resp, sizeof(resp));
	if (ret || (resp.status == RESP_STATUS_FAIL))
		pr_err("command MSG_F_GUEST_TABLE_INIT failed\n");
}

int nmp_guest_pp2_cls_tbl_add_rule(struct nmp_guest *guest,
				   struct pp2_cls_tbl *tbl,
				   struct pp2_cls_tbl_rule *rule,
				   struct pp2_cls_tbl_action *action)
{
	struct guest_pp2_cls_rule_add rule_add;
	struct guest_cmd_resp resp;
	int ret;

	rule_add.tbl_id = ((u32)(uintptr_t)tbl) - 1;
	prepare_action(&rule_add.action, action, (uintptr_t)&rule_add);
	prepare_rule(&rule_add.rule, rule);
	ret = send_internal_msg(guest, CDT_PF, guest->lf_master_id, MSG_F_GUEST_ADD_RULE, 0,
		&rule_add, sizeof(rule_add), &resp, sizeof(resp));
	if (ret)
		return ret;
	if (resp.status == RESP_STATUS_FAIL)
		return -1;
	return 0;
}

int nmp_guest_pp2_cls_tbl_modify_rule(struct nmp_guest *guest,
				      struct pp2_cls_tbl *tbl,
				      struct pp2_cls_tbl_rule *rule,
				      struct pp2_cls_tbl_action *action)
{
	struct guest_pp2_cls_rule_add rule_add;
	struct guest_cmd_resp resp;
	int ret;

	rule_add.tbl_id = ((u32)(uintptr_t)tbl) - 1;
	prepare_action(&rule_add.action, action, (uintptr_t)&rule_add);
	prepare_rule(&rule_add.rule, rule);
	ret = send_internal_msg(guest, CDT_PF, guest->lf_master_id, MSG_F_GUEST_MODIFY_RULE, 0,
		&rule_add, sizeof(rule_add), &resp, sizeof(resp));
	if (ret)
		return ret;
	if (resp.status == RESP_STATUS_FAIL)
		return -1;
	return 0;
}

int nmp_guest_pp2_cls_tbl_remove_rule(struct nmp_guest *guest,
				      struct pp2_cls_tbl *tbl,
				      struct pp2_cls_tbl_rule *rule)
{
	struct guest_pp2_cls_rule_remove rule_rem;
	struct guest_cmd_resp resp;
	int ret;

	rule_rem.tbl_id = ((u32)(uintptr_t)tbl) - 1;
	prepare_rule(&rule_rem.rule, rule);
	ret = send_internal_msg(guest, CDT_PF, guest->lf_master_id, MSG_F_GUEST_REMOVE_RULE, 0,
		&rule_rem, sizeof(rule_rem), &resp, sizeof(resp));
	if (ret)
		return ret;
	if (resp.status == RESP_STATUS_FAIL)
		return -1;
	return 0;
}

