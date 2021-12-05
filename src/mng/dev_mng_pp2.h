/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _DEV_MNG_PP2_H
#define _DEV_MNG_PP2_H

#include "std_internal.h"

#include "mng/mv_nmp.h"

int dev_mng_pp2_init(struct nmp *nmp);
int dev_mng_pp2_terminate(struct nmp *nmp);

int dev_mng_pp2_find_free_bpool(struct nmp *nmp, u32 pp_id);

#endif /* _DEV_MNG_PP2_H */
