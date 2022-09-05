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

#ifndef _ARMADA_GIU_NIC_UIO_H_
#define _ARMADA_GIU_NIC_UIO_H_

int agnic_uio_probe(struct device *dev);
int agnic_uio_remove(struct device *dev);

void agnic_uio_notify(struct agnic_adapter *adapter, int tc);

#endif /* _ARMADA_GIU_NIC_UIO_H_ */
