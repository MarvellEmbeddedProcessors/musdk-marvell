/**************************************************************************//**
	Copyright (C) 2016 Marvell International Ltd.
*//***************************************************************************/

#include "mv_std.h"

#include "mv_pp2_cls.h"


struct pp2_cls_tbl {
	int dummy;
};

int pp2_cls_tbl_init(struct pp2_cls_tbl_params *params, struct pp2_cls_tbl **tbl)
{
	pr_err("[%s] routine not supported yet!\n", __FUNCTION__);
	return -ENOTSUPP;
}

void pp2_cls_tbl_deinit(struct pp2_cls_tbl *tbl)
{
	pr_err("[%s] routine not supported yet!\n", __FUNCTION__);
}

int pp2_cls_tbl_add_rule(struct pp2_cls_tbl			*tbl,
			 struct pp2_cls_tbl_rule		*rule,
			 struct pp2_cls_tbl_entry_action	*action)
{
	pr_err("[%s] routine not supported yet!\n", __FUNCTION__);
	return -ENOTSUPP;
}

int pp2_cls_tbl_modify_rule(struct pp2_cls_tbl			*tbl,
			    struct pp2_cls_tbl_rule		*rule,
			    struct pp2_cls_tbl_entry_action	*action)
{
	pr_err("[%s] routine not supported yet!\n", __FUNCTION__);
	return -ENOTSUPP;
}

int pp2_cls_tbl_remove_rule(struct pp2_cls_tbl			*tbl,
			    struct pp2_cls_tbl_rule		*rule)
{
	pr_err("[%s] routine not supported yet!\n", __FUNCTION__);
	return -ENOTSUPP;
}
