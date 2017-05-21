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

#include "drivers/mv_pp2_cls.h"
#include "cls/pp2_cls_mng.h"
#include "pp2_types.h"
#include "pp2.h"
#include "pp2_hw_type.h"
#include "cls/pp2_hw_cls.h"
#include "cls/pp2_cls_db.h"

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
	if (mv_pp2x_ptr_validate(tbl)) {
		pr_err("%s(%d) fail, tbl = NULL\n", __func__, __LINE__);
	}
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
