/* Copyright (c) 2018 Marvell.
 * SPDX-License-Identifier: GPL-2.0-only
 */
#ifndef __AGNIC_SYSFS_MDCLS_H__
#define __AGNIC_SYSFS_MDCLS_H__

#define META_DATA_LEN            16
#define META_DATA_STR_LEN       (META_DATA_LEN * 2)

/* Subdirectories of mv_nss menu */
int  agnic_metadata_classify_sysfs_init(struct kobject *gbe_kobj);
int  agnic_metadata_classify_sysfs_exit(struct kobject *gbe_kobj);

#endif /* __MV_NSS_SYSFS_MD_TBL_H__ */

