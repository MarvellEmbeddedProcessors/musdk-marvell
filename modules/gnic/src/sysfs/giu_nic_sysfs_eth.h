/*
* ***************************************************************************
* Copyright (C) 2018 Marvell International Ltd.
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
