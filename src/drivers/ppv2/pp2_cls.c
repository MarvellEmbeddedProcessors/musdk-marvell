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
#include "pp2_print.h"
#include "pp2_hw_type.h"
#include "pp2_hw_cls.h"

#define MVPP2_CLS_PROTO_SHIFT	MVPP2_CLS_PROTO_SHIFT
#define NOT_SUPPORTED_YET 255
#define PP2_CLS_MAX_NUM_TABLES	10

struct pp2_cls_tbl {
	int dummy;
	struct pp2_cls_tbl_params params;
};

struct pp2_cls_table_db {
	u32 idx;
	struct pp2_cls_tbl *table[PP2_CLS_MAX_NUM_TABLES];
};

static struct pp2_cls_table_db table_db;

int pp2_cls_tbl_init(struct pp2_cls_tbl_params *params, struct pp2_cls_tbl **tbl)
{
	struct pp2_cls_tbl *cls_table;
	u32 rc;

	/* Para check */
	if (mv_pp2x_ptr_validate(params))
		return -EINVAL;

	rc = pp2_cls_mng_tbl_init(params);
	if (rc) {
		pr_err("cls manager table init error\n");
		return rc;
	}

	if (table_db.idx < PP2_CLS_MAX_NUM_TABLES) {
		cls_table = kmalloc(sizeof(*cls_table), GFP_KERNEL);
		if (!cls_table) {
			pr_err("no mem for cls_table array!\n");
			return -ENOMEM;
		}

		*tbl = cls_table;
		table_db.table[table_db.idx] = cls_table;
		memcpy(&table_db.table[table_db.idx]->params, params, sizeof(struct pp2_cls_tbl_params));
		table_db.idx++;
	} else {
		pr_err("no more space to add another table!\n");
		return -ENOMEM;
	}

	return 0;
}

void pp2_cls_tbl_deinit(struct pp2_cls_tbl *tbl)
{
	u32 idx;

	for (idx = 0; idx < table_db.idx; idx++)
		kfree(table_db.table[table_db.idx]);
}

int pp2_cls_tbl_add_rule(struct pp2_cls_tbl		*tbl,
			 struct pp2_cls_tbl_rule	*rule,
			 struct pp2_cls_tbl_action	*action)
{
	u32 idx, rc;

	/* Para check */
	if (mv_pp2x_ptr_validate(tbl))
		return -EINVAL;

	if (mv_pp2x_ptr_validate(rule))
		return -EINVAL;

	if (mv_pp2x_ptr_validate(action))
		return -EINVAL;

	/* check if table exists in DB */
	for (idx = 0; idx < table_db.idx; idx++) {
		if (tbl == table_db.table[idx]) {
			pp2_info("table found at index %d\n", idx);
			break;
		}
	}

	if (idx == table_db.idx) {
		pp2_err("table not found\n");
		return -EIO;
	}

	rc = pp2_cls_mng_rule_add(&table_db.table[idx]->params, rule, action);
	if (rc) {
		pr_err("cls manager table init error\n");
		return rc;
	}

	return 0;
}

int pp2_cls_tbl_modify_rule(struct pp2_cls_tbl		*tbl,
			    struct pp2_cls_tbl_rule	*rule,
			    struct pp2_cls_tbl_action	*action)
{
	pr_err("[%s] routine not supported yet!\n", __FUNCTION__);
	return -ENOTSUP;
}

int pp2_cls_tbl_remove_rule(struct pp2_cls_tbl		*tbl,
			    struct pp2_cls_tbl_rule	*rule)
{
	pr_err("[%s] routine not supported yet!\n", __FUNCTION__);
	return -ENOTSUP;
}
