/* Copyright (c) 2018 Marvell.
 * SPDX-License-Identifier: GPL-2.0-only
 */

#ifndef _ARMADA_GIU_CUSTOM_MGMT_H_
#define _ARMADA_GIU_CUSTOM_MGMT_H_

int agnic_mgmt_custom_process(struct agnic_adapter *adapter, u8 client_type,
					u8 client_id, u16 cmd_idx, u8 cmd_code, void *msg, u16 len);

#endif /* _ARMADA_GIU_CUSTOM_MGMT_H_ */
