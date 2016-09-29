/**************************************************************************//**
	Copyright (C) 2016 Marvell International Ltd.
*//***************************************************************************/

#include "mv_std.h"

#include "mv_pp2_bpool.h"


struct pp2_bpool {
	int dummy;
};


int pp2_bpool_init(struct pp2_bpool_params *params, struct pp2_bpool **bpool)
{
	pr_err("[%s] routine not supported yet!\n",__FUNCTION__);
	return -ENOTSUPP;
}

int pp2_bpool_get_buff(struct pp2_hif *hif, struct pp2_bpool *pool, struct pp2_buff_inf *buff)
{
	pr_err("[%s] routine not supported yet!\n",__FUNCTION__);
	return -ENOTSUPP;
}

int pp2_bpool_put_buff(struct pp2_hif *hif, struct pp2_bpool *pool, struct pp2_buff_inf *buff)
{
	pr_err("[%s] routine not supported yet!\n",__FUNCTION__);
	return -ENOTSUPP;
}
