/* Copyright (c) 2018 Marvell.
 * SPDX-License-Identifier: GPL-2.0-only
 */

#ifndef _MV_DP_SYSFS_MC_H_
#define _MV_DP_SYSFS_MC_H_


#ifdef __cplusplus
extern "C" {
#endif

int mv_dp_mc_sysfs_exit(struct kobject *ko);
int mv_dp_mc_sysfs_init(struct kobject *ko);


#ifdef __cplusplus
}
#endif


#endif
