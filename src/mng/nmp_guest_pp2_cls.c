/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
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

