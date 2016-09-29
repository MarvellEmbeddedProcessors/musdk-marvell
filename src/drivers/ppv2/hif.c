/**************************************************************************//**
	Copyright (C) 2016 Marvell International Ltd.
*//***************************************************************************/

#include "mv_std.h"

#include "hif.h"


struct pp2_hif {
	int dummy;
};

int pp2_hif_init(struct pp2_hif_params *params, struct pp2_hif **hif)
{
	pr_err("[%s] routine not supported yet!\n", __FUNCTION__);
	return -ENOTSUPP;
}
