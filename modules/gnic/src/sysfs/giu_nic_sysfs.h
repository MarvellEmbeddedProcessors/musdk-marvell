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
#ifndef __AGNIC_SYSFS_H__
#define __AGNIC_SYSFS_H__

#define DBG_MSG(fmt, args...)   printk(fmt, ## args)

int agnic_sysfs_init(void);
void agnic_sysfs_exit(void);

#endif /* __AGNIC_SYSFS_H__ */
