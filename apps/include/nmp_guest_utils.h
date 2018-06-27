/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#ifndef __MV_NMP_GUEST_UTILS_H__
#define __MV_NMP_GUEST_UTILS_H__

#include "mng/mv_nmp.h"
#include "mng/mv_nmp_guest.h"
#include "utils.h"
#include "pp2_utils.h"

/*
 * nmp_guest mode related functions.
 */
int app_read_nmp_cfg_file(char *cfg_file, struct nmp_params *params);
int app_guest_utils_build_all_bpools(char *buff, struct nmp_guest_info *guest_info,
				     struct bpool_desc ***ppools,
				     struct pp2_glb_common_args *pp2_args,
				     struct bpool_inf infs[]);
int app_nmp_guest_port_init(char *buff, struct nmp_guest_info *guest_info, struct port_desc *port);


#endif /*__MV_NMP_GUEST_UTILS_H__*/

