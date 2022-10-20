/* Copyright (c) 2018 Marvell.
 * SPDX-License-Identifier: GPL-2.0-only
 */
#ifndef __AGNIC_SYSFS_H__
#define __AGNIC_SYSFS_H__

#define DBG_MSG(fmt, args...)   printk(fmt, ## args)

int agnic_sysfs_init(void);
void agnic_sysfs_exit(void);

#endif /* __AGNIC_SYSFS_H__ */
