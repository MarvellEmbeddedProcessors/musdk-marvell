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

#include <linux/jiffies.h>
#include "giu_nic.h"
#include "giu_nic_hw.h"
#include "mv_gnic_custom_mgmt.h"

static struct agnic_cb_msg_params cb_msg_params[MGMT_CMD_MAX_IDX];

/*
** agnic_register_custom: register call back function.
*/
int agnic_register_custom(struct net_device *netdev, u8 id, void *arg,
			void (*recv_custom_msg_cb)(void *arg, u8 code, u64 cookie, void *msg, u16 len))
{
	struct agnic_adapter *adapter = netdev_priv(netdev);

	/* register call back function handler */
	adapter->custom_info.id = id;
	adapter->custom_info.arg = arg;
	adapter->custom_info.f_recv_custom_msg_cb = recv_custom_msg_cb;

	/* Clear custom message params arrays */
	memset(cb_msg_params, 0, sizeof(struct agnic_cb_msg_params) * MGMT_CMD_MAX_IDX);

	return 0;
}
EXPORT_SYMBOL(agnic_register_custom);

/*
** agnic_send_custom_msg: send custom message.
*/
int agnic_send_custom_msg(struct net_device *netdev,
			struct agnic_inband_mng_msg_params *msg)
{
	int ret;
	struct agnic_msg_params msg_params;
	struct agnic_adapter *adapter = netdev_priv(netdev);

	/* Build message */
	msg_params.cmd_code     = msg->msg_code;
	msg_params.client_id    = adapter->custom_info.id;
	msg_params.client_type  = CDT_CUSTOM;
	msg_params.msg          = msg->msg;
	msg_params.msg_len      = msg->msg_len;
	msg_params.resp_msg     = msg->resp_msg;
	msg_params.resp_msg_len = msg->resp_msg_len;

	/* Send message */
	ret = agnic_mgmt_async_command_send(adapter, &msg_params);
	if (ret) {
		agnic_dev_err("Failed to send custom message\n");
		return ret;
	}

	/* Validate command index */
	if (cb_msg_params[msg_params.cmd_idx].status) {
		agnic_dev_err("Send custom message return invalid message index %d\n",
			msg_params.cmd_idx);
		ret = -1;
		return ret;
	}

	/* Save CB parameters message */
	cb_msg_params[msg_params.cmd_idx].status  = MGMT_CMD_IDX_OCCUPY;
	cb_msg_params[msg_params.cmd_idx].cookie  = msg->cookie;
	cb_msg_params[msg_params.cmd_idx].timeout = jiffies_to_msecs(jiffies) - (msg->timeout);

	return 0;
}
EXPORT_SYMBOL(agnic_send_custom_msg);

int agnic_mgmt_custom_process(struct agnic_adapter *adapter, u8 client_type, u8 client_id, u16 cmd_idx,
					u8 cmd_code, void *msg, u16 len)
{
	int ret = -1;
	u64 cookie;

	/* Custom notification/response handling */

	/* Validate Client Id */
	if (adapter->custom_info.id != client_id) {
		agnic_dev_err("Invalid Client Id, Registered Id %d != Message Id %d\n",
				(int)adapter->custom_info.id, (int)client_id);
		goto cmd_error;
	}

	/* Validate CB function */
	if (adapter->custom_info.f_recv_custom_msg_cb == NULL) {
		agnic_dev_err("No registeration of custom CB function\n");
		goto cmd_error;
	}

	if (cmd_idx == CMD_ID_NOTIFICATION) {
		cookie = -1; /* cookie '-1' is for notification */
	} else {
		/* Validate CB function parameters and command index */
		if (cb_msg_params[cmd_idx].status != MGMT_CMD_IDX_OCCUPY) {
			agnic_dev_err("Invalid cmd index %d cmd_code %d\n", cmd_idx, cmd_code);
			goto cmd_error;
		}

		/* Validate CB function timeout */
		if (jiffies_to_msecs(jiffies) < cb_msg_params[cmd_idx].timeout) {
			agnic_dev_err("custom CB function timeout expired (current %d, timeout %d)\n",
						jiffies_to_msecs(jiffies), cb_msg_params[cmd_idx].timeout);

			/* Only timeout validation at this stage */
		}
		cookie = cb_msg_params[cmd_idx].cookie;
	}
	adapter->custom_info.f_recv_custom_msg_cb(adapter->custom_info.arg,
			cmd_code, cookie, msg, len);

	ret = 0;

cmd_error:

	if (cmd_idx != CMD_ID_NOTIFICATION)
		/* Clear CB parameters array */
		cb_msg_params[cmd_idx].status = MGMT_CMD_IDX_FREE;

	return ret;
}

