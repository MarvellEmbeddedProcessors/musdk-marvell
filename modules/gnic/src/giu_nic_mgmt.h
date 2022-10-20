/* Copyright (c) 2015 Marvell.
 * SPDX-License-Identifier: GPL-2.0-only
 */

#ifndef _ARMADA_GIU_NIC_MGMT_H_
#define _ARMADA_GIU_NIC_MGMT_H_

/* Message Parameters
 * cmd_idx	- Command Identifier, this field is OUTPUT param which will be will be
 *		  set by the lower layer.
 * cmd_code	- Command to be executed (out of enum agnic_cmd_codes)
 * client_id	- Client ID – PF / VF Id
 * client_type	- Client type – PF / VF
 * msg		- Message data (command parameter)
 * msg_len	- Message data size
 * timeout	- (not supported) Timeout for receiving reply
 * resp_msg	- Message response
 * resp_msg_len - Message response size
 *     Array of bytes, holding the serialized parameters/response list for a specific command.
 */
/* Make sure structure is portable along different systems. */
struct agnic_msg_params {
	u16	cmd_idx;
	u8	cmd_code;
	u8	client_id;
	u8	client_type;
	void	*msg;
	u16	msg_len;
	u32	timeout;
	void	*resp_msg;
	u16	resp_msg_len;
};


/* Forward declaration. */
struct agnic_adapter;

void agnic_mgmt_irq_disable(struct agnic_adapter *adapter);
void agnic_mgmt_irq_enable(struct agnic_adapter *adapter);
int agnic_mgmt_set_mgmt_queues(struct agnic_adapter *adapter);

int agnic_mgmt_command_send(struct agnic_adapter *adapter, struct agnic_msg_params *msg_params);
int agnic_mgmt_async_command_send(struct agnic_adapter *adapter, struct agnic_msg_params *msg_params);
int agnic_mgmt_notif_handle(struct agnic_adapter *adapter);

/* Notify Keep-alive timeout */
int agnic_mgmt_notify_down(struct agnic_adapter *adapter);

#endif /* _ARMADA_GIU_NIC_MGMT_H_ */
