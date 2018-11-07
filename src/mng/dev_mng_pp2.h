/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#ifndef _DEV_MNG_PP2_H
#define _DEV_MNG_PP2_H

#include "std_internal.h"

#include "mng/mv_nmp.h"

struct netdev_if_params {
	char if_name[16];
	u32 admin_status;
	u8 ppio_id;
	u8 pp_id;
};

extern int pp2_netdev_if_info_get(struct netdev_if_params *netdev_params);

int dev_mng_pp2_init(struct nmp *nmp);
int dev_mng_pp2_terminate(struct nmp *nmp);

int dev_mng_pp2_find_free_bpool(struct nmp *nmp, u32 pp_id);

#endif /* _DEV_MNG_PP2_H */
