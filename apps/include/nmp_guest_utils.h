/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __MV_NMP_GUEST_UTILS_H__
#define __MV_NMP_GUEST_UTILS_H__

#include "mng/mv_nmp.h"
#include "mng/mv_nmp_guest.h"
#include "utils.h"
#include "pp2_utils.h"
#include "giu_utils.h"

/*
 * nmp_guest mode related functions.
 */
int app_read_nmp_cfg_file(char *cfg_file, struct nmp_params *params);

int app_guest_utils_build_all_giu_bpools(char *buff, struct nmp_guest_info *guest_info,
					 struct giu_bpools_desc *pools_desc);

int app_guest_utils_allocate_all_giu_bpools(struct giu_bpools_desc *pools_desc,
					    u32 num_buffs);

int app_nmp_guest_giu_port_init(char *buff, struct nmp_guest_info *guest_info, struct giu_port_desc *port);

int app_guest_utils_build_all_pp2_bpools(char *buff, struct nmp_guest_info *guest_info,
					 struct bpool_desc ***ppools,
					 struct pp2_glb_common_args *pp2_args,
					 struct bpool_inf infs[]);
int app_nmp_guest_pp2_port_init(char *buff, struct nmp_guest_info *guest_info, struct port_desc *port);


#endif /*__MV_NMP_GUEST_UTILS_H__*/

