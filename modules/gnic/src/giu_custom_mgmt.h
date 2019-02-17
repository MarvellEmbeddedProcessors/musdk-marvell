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

#ifndef _ARMADA_GIU_CUSTOM_MGMT_H_
#define _ARMADA_GIU_CUSTOM_MGMT_H_

int agnic_mgmt_custom_process(struct agnic_adapter *adapter, u8 client_type,
					u8 client_id, u16 cmd_idx, u8 cmd_code, void *msg, u16 len);

#endif /* _ARMADA_GIU_CUSTOM_MGMT_H_ */
