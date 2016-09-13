/**************************************************************************//**
	Copyright (C) 2016 Marvell International Ltd.
*//***************************************************************************/

#ifndef __MV_PP2_HIF_H__
#define __MV_PP2_HIF_H__

#include "std.h"


struct pp2_hif;

struct pp2_hif_params {
	/** Used for DTS acc to find appropriate “physical” H-IF obj */
	char		*match;
	u32			 out_size; /* TX-Agg Q size */
};

int pp2_hif_init(struct pp2_hif_params *params, struct pp2_hif **hif);

#endif /* __MV_PP2_HIF_H__ */
