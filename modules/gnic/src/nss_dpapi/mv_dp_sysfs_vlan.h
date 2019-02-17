
/*******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is free software: you can redistribute it and/or
*  modify it under the terms of the GNU General Public License as
*  published by the Free Software Foundation, either version 2 of the
*  License, or any later version.
*
*  This program is distributed in the hope that it will be useful, but
*  WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
*  General Public License for more details.
*
*******************************************************************************/

#ifndef _MV_DP_SYSFS_VLAN_H_
#define _MV_DP_SYSFS_VLAN_H_


#ifdef __cplusplus
extern "C" {
#endif

int mv_dp_vlan_sysfs_exit(struct kobject *ko);
int mv_dp_vlan_sysfs_init(struct kobject *ko);


#ifdef __cplusplus
}
#endif


#endif
