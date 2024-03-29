/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

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

