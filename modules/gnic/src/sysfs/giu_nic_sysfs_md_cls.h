/*
* ***************************************************************************
* Copyright (c) 2018 Marvell.
* ***************************************************************************
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation, either version 2 of the License, or any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
* ***************************************************************************
*/
#ifndef __AGNIC_SYSFS_MDCLS_H__
#define __AGNIC_SYSFS_MDCLS_H__

#define META_DATA_LEN            16
#define META_DATA_STR_LEN       (META_DATA_LEN * 2)

/* Subdirectories of mv_nss menu */
int  agnic_metadata_classify_sysfs_init(struct kobject *gbe_kobj);
int  agnic_metadata_classify_sysfs_exit(struct kobject *gbe_kobj);

#endif /* __MV_NSS_SYSFS_MD_TBL_H__ */

