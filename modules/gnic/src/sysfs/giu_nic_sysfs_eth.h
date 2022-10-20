/* Copyright (c) 2018 Marvell.
 * SPDX-License-Identifier: GPL-2.0-only
 */
#ifndef __AGNIC_ETH_SYSFS_H__
#define __AGNIC_ETH_SYSFS_H__

/* Subdirectories of agnic menu */

/* ETH Rx sysfs APIs */
int agnic_rx_sysfs_init(struct kobject *kobj);
int agnic_rx_sysfs_exit(struct kobject *kobj);

/* RSS sysfs APIs */
int agnic_rss_sysfs_init(struct kobject *kobj);
int agnic_rss_sysfs_exit(struct kobject *kobj);

/* params sysfs APIs */
int  agnic_metadata_sysfs_init(struct kobject *pp2_kobj);
int  agnic_metadata_sysfs_exit(struct kobject *pp2_kobj);

#endif /* __AGNIC_ETH_SYSFS_H__ */
