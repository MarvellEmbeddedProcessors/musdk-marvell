/*
** Copyright (C) 2015-2016 Marvell International Ltd.
**
** This program is free software: you can redistribute it and/or
** modify it under the terms of the GNU General Public License as
** published by the Free Software Foundation, either version 2 of the
** License, or any later version.
**
** This program is distributed in the hope that it will be useful, but
** WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
** General Public License for more details.
*/

#include <linux/jiffies.h>
#include <linux/rtnetlink.h>
#include "giu_nic.h"
#include "giu_nic_hw.h"
#include "giu_custom_mgmt.h"
#include "mv_gnic_custom_mgmt.h"

/* this buffer is used for asyc notification (as no buffer was allocated for them)
 * Note: we assume that these notifications are handled one by one (otherwise, this
 *	 buffer will be overridden).
 */
#define AGNIC_MGMT_NOTIF_BUF_SIZE	1024
static char agnic_notif_buf[AGNIC_MGMT_NOTIF_BUF_SIZE];

/*
** agnic_mgmt_irq_disable: Stop interrupts generation.
*/
void agnic_mgmt_irq_disable(struct agnic_adapter *adapter)
{

}

/*
** agnic_mgmt_igmt_enable: Enable NIC's interrupts generation.
*/
void agnic_mgmt_irq_enable(struct agnic_adapter *adapter)
{

}

/*
 * Communicate management queues information with device side.
 * In case command / notification queue index is located on device memory
 * set ring producer / consumer to device memory
 * else set ring producer / consumer to notification area
 */
int agnic_mgmt_set_mgmt_queues(struct agnic_adapter *adapter)
{
	struct agnic_q_hw_info	*cmd_q_info, *notif_q_info;
	int timeout = 1000; /* ~1 second. */

	/* Set CMD queue base address & length. */
	cmd_q_info = &adapter->nic_cfg_base->cmd_q;
	cmd_q_info->q_addr = adapter->cmd_ring.dma;
	cmd_q_info->len  = adapter->cmd_ring.count;
	cmd_q_info->q_prod_offs = AGNIC_RING_PROD_INDX_LOCAL_PHYS(&adapter->cmd_ring);
	cmd_q_info->q_cons_offs = AGNIC_RING_CONS_INDX_LOCAL_PHYS(&adapter->cmd_ring);

	/* Set Notification queue base address & length. */
	notif_q_info = &adapter->nic_cfg_base->notif_q;
	notif_q_info->q_addr = adapter->notif_ring.dma;
	notif_q_info->len  = adapter->notif_ring.count;
	notif_q_info->q_prod_offs = AGNIC_RING_PROD_INDX_LOCAL_PHYS(&adapter->notif_ring);
	notif_q_info->q_cons_offs = AGNIC_RING_CONS_INDX_LOCAL_PHYS(&adapter->notif_ring);

	/* Make sure that upper writes are executed before notifying the
	 * end-point.
	 */
	wmb();

	/* Notify the AGNIC */
	adapter->nic_cfg_base->status |= AGNIC_CFG_STATUS_HOST_MGMT_READY;

#ifdef EMULATION_MODE
	timeout = 10;
#endif

	/* Wait for device to setup mgmt queues. */
	do {
		if (adapter->nic_cfg_base->status & AGNIC_CFG_STATUS_DEV_MGMT_READY)
			break;
		usleep_range(1000, 2000);
		timeout--;
	} while (timeout);

	if (timeout == 0) {
		agnic_dev_err("Timeout while waiting for device response.\n");
#ifndef EMULATION_MODE
		return -ETIMEDOUT;
#endif
	}

	return 0;
}

/* This function generated cmd_inx (for the cookie list)
 * Note: make sure that the input parameter (cmd_idx) is used
 *       within a LOCK so 2 different threads won't get the same id.
 */
static u16 agnic_mgmt_cmd_idx_gen(u16 cmd_idx, u32 ring_count)
{
	do {
		/* Increment command index */
		AGNIC_RING_PTR_INC(cmd_idx, 1, ring_count);
	} while (cmd_idx == CMD_ID_ILLEGAL || cmd_idx == CMD_ID_NOTIFICATION);

	return cmd_idx;
}

static void agnic_mgmt_wait_resp_pre(void)
{
	/* must be called under rtnl lock */
	ASSERT_RTNL();
	rtnl_unlock();
}

static void agnic_mgmt_wait_resp_post(void)
{
	rtnl_lock();
}

static int agnic_mgmt_command_send_process(struct agnic_adapter *adapter, struct agnic_msg_params *msg_params)
{
	struct agnic_ring *ring = &adapter->cmd_ring;
	struct agnic_cmd_desc *desc;
	struct agnic_mgmt_cookie *mgmt_buff;
	static u16 cmd_idx, desc_required, desc_free, desc_idx;
	int msg_buf_left, copy_len;
	int ret = 0, no_resp_req = msg_params->resp_msg ? 0 : 1;

	/* Check if length is set (only if message is not null) */
	if (msg_params->msg && !msg_params->msg_len) {
		agnic_dev_err("mgmt msg length is 0\n");
		return -EINVAL;
	}

	agnic_debug(adapter, "mgmt cmd issue (%d).\n", msg_params->cmd_code);

	/* Check how many descriptors are required for sending the message */
	desc_required = ceil(msg_params->msg_len, AGNIC_MGMT_DESC_DATA_LEN);
	if (!desc_required) /* even if there is no buffer to copy, 1 descriptor is need for the message */
		desc_required = 1;

	/* Hold the mgmt commands spin-lock, as we have a single ring that serves all CPUs */
	spin_lock_bh(&adapter->mgmt_lock);

	desc_free = AGNIC_RING_FREE(ring->tx_prod_shadow, readl(ring->consumer_p), ring->count);

	if (desc_free < desc_required) {
		agnic_dev_warn("Msg size is %d which requires %d descriptors and only %d are available\n",
				msg_params->msg_len, desc_required, desc_free);
		ret = -EBUSY;
		goto cmd_send_error;
	}

	/* Generate command index (for cookie list) */
	cmd_idx = agnic_mgmt_cmd_idx_gen(cmd_idx, ring->cookie_count);

	/* return cmd index to be used later to get the buffer */
	msg_params->cmd_idx = cmd_idx;

	mgmt_buff = &(ring->cookie_list[cmd_idx].mgmt);

	if (mgmt_buff->buf) {
		agnic_dev_warn("%s - No available cookie\n", __func__);
		ret = -ENOBUFS;
		goto cmd_send_error;
	}

	msg_buf_left = msg_params->msg_len;
	copy_len = AGNIC_MGMT_DESC_DATA_LEN;

	for (desc_idx = 0; desc_idx < desc_required; desc_idx++) {
		/* Get a pointer to the next Tx descriptor and relevant mgmt buffer info */
		desc = ((struct agnic_cmd_desc *)ring->desc) + ring->tx_prod_shadow;

		/* Identify the command once the response is received */
		desc->cmd_idx = cmd_idx;
		CMD_FLAGS_NO_RESP_SET(desc->flags, no_resp_req);

		/* Update the cmd_desc (HW descriptor) */
		desc->app_code = AC_PF_MANAGER;
		desc->cmd_code = msg_params->cmd_code;
		desc->client_id = msg_params->client_id;
		desc->client_type = msg_params->client_type;

		/* If command params exist, copy them to HW descriptor */
		if (msg_params->msg) {
			/* Adjust the copy size */
			copy_len = min(msg_buf_left, AGNIC_MGMT_DESC_DATA_LEN);

			memcpy(desc->data, msg_params->msg + (AGNIC_MGMT_DESC_DATA_LEN * desc_idx), copy_len);
			msg_buf_left -= copy_len;
		}

		/* Set the desc flag (Currently, external buffers is not supported) */
		if (desc_required == 1) /* Single descriptor */
			CMD_FLAGS_BUF_POS_SET(desc->flags, CMD_FLAG_BUF_POS_SINGLE);
		else if (desc_idx == (desc_required - 1)) /* Last descriptor */
			CMD_FLAGS_BUF_POS_SET(desc->flags, CMD_FLAG_BUF_POS_LAST);
		else /* First or Mid descriptor */
			CMD_FLAGS_BUF_POS_SET(desc->flags, CMD_FLAG_BUF_POS_FIRST_MID);

		/* Increment producer counter (Note that it's not written to HW yet) */
		AGNIC_RING_PTR_INC(ring->tx_prod_shadow, 1, ring->count);
	}

	/* Save buffer info (it will be used by notification handler
	 * to save the response data)
	 */
	mgmt_buff->buf = msg_params->resp_msg;
	mgmt_buff->buf_len = msg_params->resp_msg_len;

	/* Mark the command as sent, the condition for releasing the
	 * wait_event() below is that this field is set to != 0 by the
	 * notification handler
	 */
	mgmt_buff->condition = mgmt_buff_cmd_sent;


	/* Notify NIC about all written descriptors */
	writel(ring->tx_prod_shadow, ring->producer_p);

cmd_send_error:
	/* spin_lock can be released, as ring manipulation is over */
	spin_unlock_bh(&adapter->mgmt_lock);

	return ret;
}


static int agnic_mgmt_command_resp_handle(struct agnic_adapter *adapter, struct agnic_msg_params *msg_params)
{
	struct agnic_ring *ring = &adapter->cmd_ring;
	struct agnic_mgmt_cookie *mgmt_buff;
	u16 sync_cmd_idx = msg_params->cmd_idx;
	ulong timeout;
	int ret, result;

	/* Check if length is OK */
	if (!msg_params->resp_msg_len) {
		agnic_dev_err("mgmt response buf length is 0\n");
		result = -EINVAL;
		goto error;
	}

	/* Get a pointer to the next Tx descriptor and relevant mgmt buffer info */
	mgmt_buff = &(ring->cookie_list[sync_cmd_idx].mgmt);

	agnic_debug(adapter, "Calling wait_event.\n");
	timeout = msecs_to_jiffies(MGMT_CMD_TIMEOUT_MSECS);

	/* avoid holding rtnl-lock while waiting for even as it blocks the user-space
	 * operations which also use the rtnl-lock
	 */

	agnic_mgmt_wait_resp_pre();
	ret = wait_event_interruptible_timeout(adapter->mgmt_wait_q, mgmt_buff->condition, timeout);
	agnic_mgmt_wait_resp_post();
	if (ret < 0) {
		agnic_dev_warn("Interrupt while waiting for command %d completion.\n", msg_params->cmd_code);
		result = -ERESTARTSYS;
		goto error;
	} else if (ret == 0) {
		agnic_dev_err("Timeout while waiting for command %d completion.\n", msg_params->cmd_code);
		result = -ETIMEDOUT;
		goto error;
	}

	/* If we reached here, then No timeout, no signal, operation completed */
	/* =================================================================== */

	/* Sanity check first */
	if (mgmt_buff->condition == mgmt_buff_cmd_sent) {
		agnic_dev_err("Bad state, event triggered but condition is still 0x0.\n");
		result = -1;
		goto error;
	}

	/* Get notification data */
	result = mgmt_buff->result;

	/* update response size.
	 * Note that the data was copied to the buffer by notification handler
	 * so no need to copy it again.
	 */
	msg_params->resp_msg_len = sizeof(struct agnic_mgmt_cmd_resp);

error:
#ifdef EMULATION_MODE
	result = 0;
#endif
	return result;
}


/*
 * agnic_mgmt_command_send - Send simple control command, and wait for it's completion.
 * This function does not support multi-descriptor commands, or commands with
 * external data buffers. Such commands should be implemented by a different
 * function.
 *
 * TODO: Add support for multi descriptor commands.
 * TODO: Add support for external buffer commands.
 */
int agnic_mgmt_command_send(struct agnic_adapter *adapter, struct agnic_msg_params *msg_params)
{
	int ret;

	ret = agnic_mgmt_command_send_process(adapter, msg_params);
	if (ret) {
		agnic_dev_err("mgmt command send failed\n");
		return ret;
	}

	/* Check if response is required */
	if (!msg_params->resp_msg)
		return 0;

	ret = agnic_mgmt_command_resp_handle(adapter, msg_params);
	if (ret) {
		agnic_dev_err("mgmt command receive failed\n");
		return ret;
	}

	return 0;
}


/*
 * agnic_mgmt_async_command_send - Send command, do not wait for it's completion.
 *
 */
int agnic_mgmt_async_command_send(struct agnic_adapter *adapter, struct agnic_msg_params *msg_params)
{
	int ret;

	ret = agnic_mgmt_command_send_process(adapter, msg_params);
	if (ret) {
		agnic_dev_err("mgmt command send failed\n");
		return ret;
	}

	return 0;
}


static int agnic_mgmt_resp_process(struct agnic_adapter *adapter, u16 cmd_idx,
					u8 cmd_code, void *msg, u16 len)
{
	struct agnic_ring *cmd_ring = &adapter->cmd_ring;
	struct agnic_mgmt_cookie *mgmt_buff;
	struct agnic_mgmt_cmd_resp *cmd_resp = (struct agnic_mgmt_cmd_resp *)msg;
	int ret = 0;

	/* grab the commands buffer */
	mgmt_buff = &(cmd_ring->cookie_list[cmd_idx].mgmt);

	mgmt_buff->result = cmd_resp->status;

	/* For responses there is no need to handle the mgmt_buff if there was
	 * an error. we just trigger the event to wake the caller.
	 */
	if (mgmt_buff->result != AGNIC_NOTIF_STATUS_OK) {
		agnic_dev_err("notification failure (err %d cmd_code %d).\n",
				mgmt_buff->result, cmd_code);
		ret = -1;
	}

	/* Make sure all logic above (including buf copy in caller function), was observed before releasing
	* the command issuer.
	*/
	wmb();
	mgmt_buff->condition = mgmt_buff_notif_rcv;

	/* Wake the command issuer in order to receive the response */
	wake_up_interruptible(&adapter->mgmt_wait_q);

	return ret;
}


/* There are 3 response/notification types:
 * 1) netdev response: this is a response to a netdev command which
 *	was sent before. in this case we trigger the event that will wake-up
 *	the thread which sent the command
 *
 * 2) netdev notification: notification which are received asynchronously.
 *
 * 3) custom notification/response: notification/response which are part of
 *	the in-band management mechanism.
 */
static int agnic_mgmt_notif_dispatch(struct agnic_adapter *adapter, u8 client_type,
					u8 client_id, u16 cmd_idx, u8 cmd_code, void *msg, u16 len)
{

	/* Custom notification/response handling */
	if (client_type != CDT_PF)
		return agnic_mgmt_custom_process(adapter, client_type,
						client_id, cmd_idx, cmd_code, msg, len);

	/* Netdev notification/response handling */

	/* Netdev (async) notification handling */
	if (cmd_idx == CMD_ID_NOTIFICATION)
		return agnic_mgmt_notif_process(adapter, cmd_code, msg, len);

	/* Netdev response handling */
	return agnic_mgmt_resp_process(adapter, cmd_idx, cmd_code, msg, len);
}

/*
 * agnic_mgmt_notif_handle - Handle management notifications.
 * Called by Notification ISR or a timer callback in case working in polling
 * mode.
 *
 * TODO: Add support for multi descriptor notifications.
 * TODO: Add support for external buffer notifications.
 */
int agnic_mgmt_notif_handle(struct agnic_adapter *adapter)
{
	struct agnic_ring *ring = &adapter->notif_ring;
	struct agnic_ring *cmd_ring = &adapter->cmd_ring;
	struct agnic_cmd_desc *desc;
	struct agnic_mgmt_cookie *mgmt_buff = NULL;
	void *msg_buf;
	int desc_idx = 0, buf_pos;
	int msg_buf_len, msg_buf_left, msg_len, copy_len;
	int sg_msg; /* Indicates that S/G message is processed*/
	u16 cmd_idx;
	int ret = 0;

	/* Check if anything should be done. */
	if (AGNIC_RING_IS_EMPTY(readl(ring->producer_p), ring->rx_cons_shadow)) {
		agnic_debug(adapter, "Notification ring is empty.\n");
		return -ENOMSG;
	}

	/* Get a pointer to the next Rx descriptor and relevant mgmt buffer
	 * info.
	 */
	desc = ((struct agnic_cmd_desc *)ring->desc) + ring->rx_cons_shadow;

	cmd_idx = desc->cmd_idx;

	if (cmd_idx == CMD_ID_NOTIFICATION) {
		/* For async notifications, no one allocated a buffer. Therefore, we use pre-allocated buffer.
		 * Note: we assume that these notifications are handled one by one (otherwise, this
		 *	 buffer will be overridden).
		 */
		msg_buf = agnic_notif_buf;
		msg_buf_len = AGNIC_MGMT_NOTIF_BUF_SIZE;
	} else {
		/* Check that the cmd_idx is a valid one */
		if (cmd_idx > cmd_ring->cookie_count) {
			agnic_dev_err("Bad value in notification cmd_idx (0x%x).\n", cmd_idx);
			ret = -EOVERFLOW;
			goto notify_error;
		}

		/* Now grab the commands buffer */
		mgmt_buff = &(cmd_ring->cookie_list[cmd_idx].mgmt);

		/* Take allocated buffer */
		msg_buf = mgmt_buff->buf;
		msg_buf_len = mgmt_buff->buf_len;

		if (!msg_buf) {
			agnic_dev_err("cmd code %d (cmd idx %d) has NULL buffer. Skip this notification\n",
					desc->cmd_code, cmd_idx);
			/* Increment the consumer so this failed message is skipped */
			AGNIC_RING_PTR_INC(ring->rx_cons_shadow, 1, ring->count);

			ret = -1;
			goto notify_error;
		}
	}

	msg_buf_left = msg_buf_len;
	copy_len = AGNIC_MGMT_DESC_DATA_LEN;
	msg_len = 0;

	/* For S/G message, we loop the descriptors until we reach buf-pos with value of LAST.
	 * There are several error cases::
	 * 1. buff-pos is single or external
	 * 2. desc->cmd_indx != cmd_idx (i.e cmd_idx should be the same for all descriptors).
	 * 3. num_ext_desc != 0
	 * 4. Ring is empty before we reached a descriptor with buf-pos with value of LAST.
	 */

	/* Check if this is S/G message */
	sg_msg = (CMD_FLAGS_BUF_POS_GET(desc->flags) == CMD_FLAG_BUF_POS_FIRST_MID) ? 1 : 0;

	do {
		/* Check if there are not more descriptor in the ring.
		 * This is an error case as for S/G message, we expect all descriptors
		 * to be in the ring before sending the message.
		 * For single message, we already checked above that the ring is not
		 * empty so this is not a valid case.
		 */
		if (AGNIC_RING_IS_EMPTY(*ring->producer_p, ring->rx_cons_shadow)) {
			agnic_dev_err("ring is empty in the middle of S/G message\n");
			ret = -1;
			goto notify_error;
		}

		/* get next descriptor */
		desc = ((struct agnic_cmd_desc *)ring->desc) + ring->rx_cons_shadow;

		/* Check that the msg buffer is enough for copying the response into it
		 * if not, reduce the copy length to whatever space left
		 */
		if (msg_buf_left < AGNIC_MGMT_DESC_DATA_LEN) {
			agnic_debug(adapter, "space left < data size (%d < %d). reducing to left space\n",
					msg_buf_left, AGNIC_MGMT_DESC_DATA_LEN);
			copy_len = msg_buf_left;
		}

		/* If no space left at all and we still have descriptors to handle
		 * we iterate all remaining descriptors and increment the consumer
		 * pointer without handing the descriptors or copying the data.
		 */
		/* TODO: Assumption is that every response requires buffer (for notifications we have static buffer) */
		if (msg_buf_left == 0)
			agnic_debug(adapter, "Skipping remaining descriptors as no space left (cmd_code %d  cmd_idx %d)\n",
					desc->cmd_code, desc->cmd_idx);

		buf_pos = CMD_FLAGS_BUF_POS_GET(desc->flags);

		if (sg_msg) {
			/* Check that the descriptor has the same cmd_idx as the first one */
			if (desc->cmd_idx != cmd_idx) {
				agnic_dev_err("desc->cmd_idx (%d) != cmd_idx (%d)\n", desc->cmd_idx, cmd_idx);
				ret = -1;
			} else if ((buf_pos == CMD_FLAG_BUF_POS_SINGLE) || (buf_pos == CMD_FLAG_BUF_POS_EXT_BUF)) {
				agnic_dev_err("Invalid buf position (%d) during S/G message\n", buf_pos);
				ret = -1;
			} else if (CMD_FLAGS_NUM_EXT_DESC_GET(desc->flags) != 0) {
				agnic_dev_err("Invalid Ext desc number during S/G message\n");
				ret = -1;
			}

			if (ret)
				goto notify_error;
		}

		/* Copy the message data only if there was no error and there is space in the buffer */
		if (!ret && msg_buf_left) {
			memcpy(msg_buf + (AGNIC_MGMT_DESC_DATA_LEN * desc_idx), desc->data, copy_len);
			msg_buf_left -= copy_len;
			msg_len += copy_len;
			desc_idx++;
		}

		/* Increment consumer counter */
		AGNIC_RING_PTR_INC(ring->rx_cons_shadow, 1, ring->count);
	} while ((buf_pos != CMD_FLAG_BUF_POS_SINGLE) && (buf_pos != CMD_FLAG_BUF_POS_LAST));

	if (ret)
		goto notify_error;

	/* Notify NIC with new consumer counter */
	writel(ring->rx_cons_shadow, ring->consumer_p);

	/* Call dispatcher only if we didn't reach here due to error */
	ret = agnic_mgmt_notif_dispatch(adapter, desc->client_type, desc->client_id, desc->cmd_idx,
					desc->cmd_code, msg_buf, msg_len);

	/* The mgmt_buff 'buf' field should be reset to indicate that this
	 * cookie entry is free
	 */
	if (mgmt_buff)
		mgmt_buff->buf = NULL;

	return ret;

notify_error:
	/* in case of an error, the mgmt_buff 'buf' field should be reset to indicate that this
	 * cookie entry is free
	 */
	if (mgmt_buff)
		mgmt_buff->buf = NULL;

	/* Notify NIC with new consumer counter */
	writel(ring->rx_cons_shadow, ring->consumer_p);

	return ret;
}

/*
 * agnic_mgmt_notify_down - Notify Keep-alive timeout.
 *
 */
int agnic_mgmt_notify_down(struct agnic_adapter *adapter)
{
	return agnic_mgmt_custom_process(adapter, CDT_CUSTOM, adapter->custom_info.id,
					 CMD_ID_NOTIFICATION, AGNIC_CUSTOM_CODE_NOTIFY_DOWN, NULL, 0);
}
