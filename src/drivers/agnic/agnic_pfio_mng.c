/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "std_internal.h"
#include "lib/net.h"

#include "agnic_pfio.h"


/* this buffer is used for asyc notification (as no buffer was allocated for them)
 * Note: we assume that these notifications are handled one by one (otherwise, this
 *	 buffer will be overridden).
 */
#define AGNIC_MGMT_NOTIF_BUF_SIZE	1024
#define AGNIC_MGMT_CMD_RESP_TIMEOUT	5000 /* 5 secs */


static char agnic_notif_buf[AGNIC_MGMT_NOTIF_BUF_SIZE];


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

static int agnic_mgmt_command_send_process(struct agnic_pfio *pfio, struct agnic_msg_params *msg_params)
{
	struct agnic_ring *ring = &pfio->cmd_ring;
	struct agnic_cmd_desc *desc;
	struct agnic_mgmt_cookie *mgmt_buff;
	static u16 cmd_idx, desc_required, desc_free, desc_idx;
	int msg_buf_left, copy_len;
	int ret = 0, no_resp_req = msg_params->resp_msg ? 0 : 1;

	/* Check if length is set (only if message is not null) */
	if (msg_params->msg && !msg_params->msg_len) {
		pr_err("agnic mgmt msg length is 0\n");
		return -EINVAL;
	}

	pr_debug("mgmt cmd issue (%d).\n", msg_params->cmd_code);

	/* Check how many descriptors are required for sending the message */
	desc_required = ceil(msg_params->msg_len, AGNIC_MGMT_DESC_DATA_LEN);
	if (!desc_required) /* even if there is no buffer to copy, 1 descriptor is need for the message */
		desc_required = 1;

	/* Hold the mgmt commands spin-lock, as we have a single ring that serves all CPUs */
	spin_lock(&pfio->mgmt_lock);

	desc_free = AGNIC_RING_FREE(ring->tx_prod_shadow, readl(ring->consumer_p), ring->count);

	if (desc_free < desc_required) {
		pr_warn("Msg size is %d which requires %d descriptors and only %d are available\n",
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
		pr_warn("%s - No available cookie\n", __func__);
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
	spin_unlock(&pfio->mgmt_lock);

	return ret;
}

static int agnic_mgmt_command_resp_handle(struct agnic_pfio *pfio, struct agnic_msg_params *msg_params)
{
	struct agnic_ring *ring = &pfio->cmd_ring;
	struct agnic_mgmt_cookie *mgmt_buff;
	u16 sync_cmd_idx = msg_params->cmd_idx;
	ulong timeout;
	int result;

	/* Check if length is OK */
	if (!msg_params->resp_msg_len) {
		pr_err("agnic mgmt response buf length is 0\n");
		result = -EINVAL;
		goto error;
	}

	/* Get a pointer to the next Tx descriptor and relevant mgmt buffer info */
	mgmt_buff = &(ring->cookie_list[sync_cmd_idx].mgmt);

	timeout = AGNIC_MGMT_CMD_RESP_TIMEOUT;
	while ((mgmt_buff->condition == mgmt_buff_cmd_sent) && --timeout) {
		udelay(1000);
		agnic_pfio_poll_mgmt(pfio);
	}
	if (!timeout) {
		pr_err("Timeout while waiting for command %d completion.\n", msg_params->cmd_code);
		result = -ETIMEDOUT;
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
static int agnic_mgmt_command_send(struct agnic_pfio *pfio, struct agnic_msg_params *msg_params)
{
	int ret;

	ret = agnic_mgmt_command_send_process(pfio, msg_params);
	if (ret) {
		pr_err("agnic mgmt command send failed\n");
		return ret;
	}

	/* Check if response is required */
	if (!msg_params->resp_msg)
		return 0;

	ret = agnic_mgmt_command_resp_handle(pfio, msg_params);
	if (ret) {
		pr_err("agnic mgmt command recv failed\n");
		return ret;
	}

	return 0;
}

static int agnic_mgmt_resp_process(struct agnic_pfio *pfio, u16 cmd_idx,
					u8 cmd_code, void *msg, u16 len)
{
	struct agnic_ring *cmd_ring = &pfio->cmd_ring;
	struct agnic_mgmt_cookie *mgmt_buff;
	struct agnic_mgmt_cmd_resp *cmd_resp = (struct agnic_mgmt_cmd_resp *)msg;

	/* grab the commands buffer */
	mgmt_buff = &(cmd_ring->cookie_list[cmd_idx].mgmt);

	mgmt_buff->result = cmd_resp->status;

	/* For responses there is no need to handle the mgmt_buff if there was
	 * an error. we just trigger the event to wake the caller.
	 */
	if (mgmt_buff->result != AGNIC_NOTIF_STATUS_OK) {
		pr_err("netdev Notification status is failure (0x%x).\n", mgmt_buff->result);
		return -1;
	}

	/* Make sure all logic above (including buf copy in caller function), was observed before releasing
	* the command issuer.
	*/
	wmb();
	mgmt_buff->condition = mgmt_buff_notif_rcv;

	return 0;
}

static int agnic_mgmt_custom_process(struct agnic_pfio *pfio, u8 client_type, u8 client_id, u16 cmd_idx,
					u8 cmd_code, void *msg, u16 len)
{
	int ret = -1;
	u64 cookie;

	/* Custom notification/response handling */

	/* Validate Client Id */
	if (pfio->custom_info.id != client_id) {
		pr_err("Invalid Client Id, Registered Id %d != Message Id %d\n",
				(int)pfio->custom_info.id, (int)client_id);
		goto cmd_error;
	}

	/* Validate CB function */
	if (pfio->custom_info.f_recv_custom_msg_cb == NULL) {
		pr_err("No registeration of custom CB function\n");
		goto cmd_error;
	}

	if (cmd_idx == CMD_ID_NOTIFICATION)
		cookie = -1; /* cookie '-1' is for notification */
	else {
		/* Validate CB function parameters and command index */
		if (pfio->custom_info.cb_msg_params[cmd_idx].status != MGMT_CMD_IDX_OCCUPY) {
			pr_err("Invalid cmd index %d cmd_code %d\n", cmd_idx, cmd_code);
			goto cmd_error;
		}

		/* TODO: add support for timeout here ... */

		cookie = pfio->custom_info.cb_msg_params[cmd_idx].cookie;
	}
	pfio->custom_info.f_recv_custom_msg_cb(pfio->custom_info.arg,
			cmd_code, cookie, msg, len);

	ret = 0;

cmd_error:
	if (cmd_idx != CMD_ID_NOTIFICATION)
		/* Clear CB parameters array */
		pfio->custom_info.cb_msg_params[cmd_idx].status = MGMT_CMD_IDX_FREE;

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
static int agnic_mgmt_notif_dispatch(struct agnic_pfio *pfio, u8 client_type,
					u8 client_id, u16 cmd_idx, u8 cmd_code, void *msg, u16 len)
{
	/* (async) notification handling */
	if (cmd_idx == CMD_ID_NOTIFICATION) {
		if (client_type == CDT_PF) {
			int err;

			err = agnic_mgmt_notif_process(pfio, cmd_code, msg, len);
			if (err)
				return err;
		}

		/* Custom notification/response handling */
		return agnic_mgmt_custom_process(pfio, client_type,
						 client_id, cmd_idx, cmd_code, msg, len);
	}

	/* response handling */
	return agnic_mgmt_resp_process(pfio, cmd_idx, cmd_code, msg, len);
}


/*
 * agnic_init_pfio - Send the Rx / Tx / Control queues information into
 * the HW. This will actually send the required control messages to the giu-nic to
 * create the required "channels".
 */
int agnic_init_pfio(struct agnic_pfio *pfio)
{
	struct agnic_msg_params msg_params;
	struct agnic_mgmt_cmd_params cmd_params;
	struct agnic_mgmt_cmd_resp cmd_resp;
	struct agnic_ring *ring, *bp_ring;
	int msg_len = sizeof(struct agnic_mgmt_cmd_params);
	int resp_msg_len = sizeof(struct agnic_mgmt_cmd_resp);
	int ret, tc, i;

	pr_debug("Configure queues in GIU.\n");

	/* Management echo. */
	pr_debug("Sending mgmt-echo.\n");
	msg_params.cmd_code = CC_PF_MGMT_ECHO;
	msg_params.client_id = 0;
	msg_params.client_type = CDT_PF;
	msg_params.msg = NULL; /* No msg params */
	msg_params.msg_len = 0;
	msg_params.resp_msg = &cmd_resp;
	msg_params.resp_msg_len = resp_msg_len;

	ret = agnic_mgmt_command_send(pfio, &msg_params);
	if (ret)
		goto error;

	/* PF_INIT */
	pr_debug("Sending PF_INIT.\n");

	msg_params.cmd_code = CC_PF_INIT;
	msg_params.client_id = 0;
	msg_params.client_type = CDT_PF;

	cmd_params.pf_init.mtu_override = pfio->mtu;
	cmd_params.pf_init.num_host_egress_tc = pfio->num_out_tcs;
	cmd_params.pf_init.num_host_ingress_tc = pfio->num_in_tcs;
	cmd_params.pf_init.mru_override = pfio->mru;
	cmd_params.pf_init.egress_sched = ES_STRICT_SCHED;
	msg_params.msg = &cmd_params;
	msg_params.msg_len = msg_len;
	msg_params.resp_msg = &cmd_resp;
	msg_params.resp_msg_len = resp_msg_len;

	ret = agnic_mgmt_command_send(pfio, &msg_params);
	if (ret)
		goto error;

	/* PF_INGRESS_TC_ADD */
	pr_debug("Set ingress TC configuration.\n");
	msg_params.cmd_code = CC_PF_INGRESS_TC_ADD;
	msg_params.client_id = 0;
	msg_params.client_type = CDT_PF;
	msg_params.msg = &cmd_params;
	msg_params.msg_len = msg_len;
	msg_params.resp_msg = &cmd_resp;
	msg_params.resp_msg_len = resp_msg_len;

	for (i = 0; i < pfio->num_in_tcs; i++) {
		cmd_params.pf_ingress_tc_add.tc = i;
		cmd_params.pf_ingress_tc_add.num_queues = pfio->num_qs_per_tc;
		cmd_params.pf_ingress_tc_add.pkt_offset = pfio->in_tcs[i].pkt_offset;
		cmd_params.pf_ingress_tc_add.hash_type = pfio->hash_type;

		ret = agnic_mgmt_command_send(pfio, &msg_params);
		if (ret)
			goto error;
	}

	/* PF_INGRESS_DATA_QUEUE_ADD */
	msg_params.cmd_code = CC_PF_INGRESS_DATA_Q_ADD;
	msg_params.client_id = 0;
	msg_params.client_type = CDT_PF;
	msg_params.msg = &cmd_params;
	msg_params.msg_len = msg_len;
	msg_params.resp_msg = &cmd_resp;
	msg_params.resp_msg_len = resp_msg_len;

	/* TODO: support multiple TCs (i.e. iterate TCs as well) */
	tc = 0;

	for (i = 0; i < pfio->num_qs_per_tc; i++) {
		pr_debug("Add ingress queue #%d.\n", i);
		cmd_params.pf_ingress_data_q_add.tc = tc;

		ring = pfio->rx_ring[cmd_params.pf_ingress_data_q_add.tc * pfio->num_qs_per_tc + i];
		ring->tc = tc; /* update the TC this ring is assigned to */

		cmd_params.pf_ingress_data_q_add.q_phys_addr = ring->dma;
		cmd_params.pf_ingress_data_q_add.q_len = ring->count;
		cmd_params.pf_ingress_data_q_add.q_prod_offs = AGNIC_RING_PROD_INDX_LOCAL_PHYS(ring);
		cmd_params.pf_ingress_data_q_add.q_cons_offs = AGNIC_RING_CONS_INDX_LOCAL_PHYS(ring);

		/* MSI-X are not enabled */
		cmd_params.pf_ingress_data_q_add.msix_id = 0;

		bp_ring = &pfio->bp_ring[cmd_params.pf_ingress_data_q_add.tc * pfio->num_qs_per_tc + i];
		cmd_params.pf_ingress_data_q_add.bpool_q_phys_addr = bp_ring->dma;
		cmd_params.pf_ingress_data_q_add.bpool_q_prod_offs = AGNIC_RING_PROD_INDX_LOCAL_PHYS(bp_ring);
		cmd_params.pf_ingress_data_q_add.bpool_q_cons_offs = AGNIC_RING_CONS_INDX_LOCAL_PHYS(bp_ring);
		cmd_params.pf_ingress_data_q_add.q_buf_size = bp_ring->bp_frag_sz;

		ret = agnic_mgmt_command_send(pfio, &msg_params);
		if (ret)
			goto error;
	}

	/* PF_EGRESS_TC_ADD */
	pr_debug("Set egress TC configuration.\n");
	msg_params.cmd_code = CC_PF_EGRESS_TC_ADD;
	msg_params.client_id = 0;
	msg_params.client_type = CDT_PF;
	msg_params.msg = &cmd_params;
	msg_params.msg_len = msg_len;
	msg_params.resp_msg = &cmd_resp;
	msg_params.resp_msg_len = resp_msg_len;

	for (i = 0; i < pfio->num_out_tcs; i++) {
		cmd_params.pf_egress_tc_add.tc = i;
		cmd_params.pf_egress_tc_add.num_queues = pfio->num_qs_per_tc;

		ret = agnic_mgmt_command_send(pfio, &msg_params);

		if (ret)
			goto error;
	}

	/* PF_EGRESS_DATA_QUEUE_ADD */
	msg_params.cmd_code = CC_PF_EGRESS_DATA_Q_ADD;
	msg_params.client_id = 0;
	msg_params.client_type = CDT_PF;
	msg_params.msg = &cmd_params;
	msg_params.msg_len = msg_len;
	msg_params.resp_msg = &cmd_resp;
	msg_params.resp_msg_len = resp_msg_len;

	/* TODO: support multiple TCs (i.e. iterate TCs as well) */
	tc = 0;

	for (i = 0; i < pfio->num_qs_per_tc; i++) {
		pr_debug("Add egress queue #%d.\n", i);
		ring = pfio->tx_ring[i];
		ring->tc = tc; /* update the TC this ring is assigned to */

		cmd_params.pf_egress_q_add.q_phys_addr = ring->dma;
		cmd_params.pf_egress_q_add.q_len = ring->count;
		cmd_params.pf_egress_q_add.q_prod_offs = AGNIC_RING_PROD_INDX_LOCAL_PHYS(ring);
		cmd_params.pf_egress_q_add.q_cons_offs = AGNIC_RING_CONS_INDX_LOCAL_PHYS(ring);

		/* MSI-X are not enabled */
		cmd_params.pf_egress_q_add.msix_id = 0;

		/* Meanwhile, we support only strict prio */
		cmd_params.pf_egress_q_add.q_wrr_weight = 0;
		cmd_params.pf_egress_q_add.tc = tc;

		ret = agnic_mgmt_command_send(pfio, &msg_params);
		if (ret)
			goto error;
	}

	/* PF_INIT_DONE */
	pr_debug("Send INIT_DONE command.\n");
	msg_params.cmd_code = CC_PF_INIT_DONE;
	msg_params.client_id = 0;
	msg_params.client_type = CDT_PF;
	msg_params.msg = NULL; /* No msg params */
	msg_params.msg_len = 0;
	msg_params.resp_msg = &cmd_resp;
	msg_params.resp_msg_len = sizeof(cmd_resp);

	ret = agnic_mgmt_command_send(pfio, &msg_params);
	if (ret)
		goto error;

	return 0;
error:
	/* TODO: Define configuration rollback at NIC level, then implement
	 * here.
	 */
	pr_err("Failed to configure network queues into hardware.\n");
	return ret;
}

/*
 * agnic_deinit_pfio - Delete all queues configuration from the underlying
 * hardware.
 */
void agnic_deinit_pfio(struct agnic_pfio *pfio)
{
	pr_err("%s - Not implemented.\n", __func__);
	/* TODO: Need to define the ifdown process at the NIC side, and
	 * implement this function.
	 */
}


int agnic_pfio_register_custom_msg_cb(struct agnic_pfio *pfio,
				      u8 id,
				      void *arg,
				      void (*recv_custom_msg_cb)(void *arg, u8 code, u64 cookie, void *msg, u16 len))
{
	if (!pfio) {
		pr_err("no pfio obj!");
		return -EINVAL;
	}

	/* register call back function handler */
	pfio->custom_info.id = id;
	pfio->custom_info.arg = arg;
	pfio->custom_info.f_recv_custom_msg_cb = recv_custom_msg_cb;

	return 0;
}

int agnic_pfio_send_custom_msg(struct agnic_pfio *pfio,
			       struct agnic_pfio_inband_mng_msg_params *msg)
{
	int			 ret;
	struct agnic_msg_params	 msg_params;

	if (!pfio) {
		pr_err("no pfio obj!");
		return -EINVAL;
	}

	if (msg->timeout) {
		pr_err("message timeout is not supported yet!\n");
		return -EINVAL;
	}

	/* Build message */
	msg_params.cmd_code     = msg->msg_code;
	msg_params.client_id    = pfio->custom_info.id;
	msg_params.client_type  = CDT_CUSTOM;
	msg_params.msg          = msg->msg;
	msg_params.msg_len      = msg->msg_len;
	msg_params.resp_msg     = msg->resp_msg;
	msg_params.resp_msg_len = msg->resp_msg_len;

	/* Send message */
	ret = agnic_mgmt_command_send_process(pfio, &msg_params);
	if (ret) {
		pr_err("Failed to send custom message\n");
		return ret;
	}

	/* Validate command index */
	if (pfio->custom_info.cb_msg_params[msg_params.cmd_idx].status) {
		pr_err("Send custom message return invalid message index %d\n",
			msg_params.cmd_idx);
		ret = -1;
		return ret;
	}

	/* Save CB parameters message */
	pfio->custom_info.cb_msg_params[msg_params.cmd_idx].status  = MGMT_CMD_IDX_OCCUPY;
	pfio->custom_info.cb_msg_params[msg_params.cmd_idx].cookie  = msg->cookie;
	pfio->custom_info.cb_msg_params[msg_params.cmd_idx].timeout = 0;

	return 0;
}

/*
 * agnic_mgmt_notif_handle - Handle management notifications.
 * Called by Notification ISR or a timer callback in case working in polling
 * mode.
 *
 * TODO: Add support for multi descriptor notifications.
 * TODO: Add support for external buffer notifications.
 */
int agnic_pfio_poll_mgmt(struct agnic_pfio *pfio)
{
	struct agnic_ring *ring = &pfio->notif_ring;
	struct agnic_ring *cmd_ring = &pfio->cmd_ring;
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
		pr_debug("Notification ring is empty.\n");
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
			pr_err("Bad value in notification cmd_idx (0x%x).\n", cmd_idx);
			ret = -EOVERFLOW;
			goto notify_error;
		}

		/* Now grab the commands buffer */
		mgmt_buff = &(cmd_ring->cookie_list[cmd_idx].mgmt);

		/* Take allocated buffer */
		msg_buf = mgmt_buff->buf;
		msg_buf_len = mgmt_buff->buf_len;

		if (!msg_buf) {
			pr_err("cmd code %d (cmd idx %d) has NULL buffer. Skip this notification\n",
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
			pr_err("ring is empty in the middle of S/G message\n");
			ret = -1;
			goto notify_error;
		}

		/* get next descriptor */
		desc = ((struct agnic_cmd_desc *)ring->desc) + ring->rx_cons_shadow;

		/* Check that the msg buffer is enough for copying the response into it
		 * if not, reduce the copy length to whatever space left
		 */
		if (msg_buf_left < AGNIC_MGMT_DESC_DATA_LEN) {
			pr_debug("space left < data size (%d < %d). reducing to left space\n",
					msg_buf_left, AGNIC_MGMT_DESC_DATA_LEN);
			copy_len = msg_buf_left;
		}

		/* If no space left at all and we still have descriptors to handle
		 * we iterate all remaining descriptors and increment the consumer
		 * pointer without handing the descriptors or copying the data.
		 */
		/* TODO: Assumption is that every response requires buffer (for notifications we have static buffer) */
		if (msg_buf_left == 0)
			pr_debug("Skipping remaining descriptors as no space left (cmd_code %d  cmd_idx %d)\n",
					desc->cmd_code, desc->cmd_idx);

		buf_pos = CMD_FLAGS_BUF_POS_GET(desc->flags);

		if (sg_msg) {
			/* Check that the descriptor has the same cmd_idx as the first one */
			if (desc->cmd_idx != cmd_idx) {
				pr_err("desc->cmd_idx (%d) != cmd_idx (%d)\n", desc->cmd_idx, cmd_idx);
				ret = -1;
			} else if ((buf_pos == CMD_FLAG_BUF_POS_SINGLE) || (buf_pos == CMD_FLAG_BUF_POS_EXT_BUF)) {
				pr_err("Invalid buf position (%d) during S/G message\n", buf_pos);
				ret = -1;
			} else if (CMD_FLAGS_NUM_EXT_DESC_GET(desc->flags) != 0) {
				pr_err("Invalid Ext desc number during S/G message\n");
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
	ret = agnic_mgmt_notif_dispatch(pfio, desc->client_type, desc->client_id, desc->cmd_idx,
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

int agnic_pfio_enable(struct agnic_pfio *pfio)
{
	int ret;
	struct agnic_msg_params msg_params;
	struct agnic_mgmt_cmd_resp cmd_resp;

	if (!pfio) {
		pr_err("no pfio obj!");
		return -EINVAL;
	}

	msg_params.client_id = 0;
	msg_params.client_type = CDT_PF;
	msg_params.msg = NULL;    /* No msg params */
	msg_params.msg_len = 0;
	msg_params.cmd_code = CC_PF_ENABLE;
	msg_params.resp_msg = &cmd_resp;
	msg_params.resp_msg_len = sizeof(cmd_resp);

	ret = agnic_mgmt_command_send(pfio, &msg_params);
	if (ret) {
		pr_err("Failed to set %s command!\n", "enable");
		return ret;
	}

	return 0;
}

int agnic_pfio_disable(struct agnic_pfio *pfio)
{
	int ret;
	struct agnic_msg_params msg_params;

	if (!pfio) {
		pr_err("no pfio obj!");
		return -EINVAL;
	}

	msg_params.client_id = 0;
	msg_params.client_type = CDT_PF;
	msg_params.msg = NULL;    /* No msg params */
	msg_params.msg_len = 0;
	msg_params.cmd_code = CC_PF_DISABLE;
	msg_params.resp_msg = 0;    /* No response in disable */
	msg_params.resp_msg_len = 0;

	ret = agnic_mgmt_command_send(pfio, &msg_params);
	if (ret) {
		pr_err("Failed to set %s command!\n", "disable");
		return ret;
	}

	return 0;
}

int agnic_pfio_set_mac_addr(struct agnic_pfio *pfio, const eth_addr_t addr)
{
	struct agnic_msg_params msg_params;
	struct agnic_mgmt_cmd_params cmd_params;
	struct agnic_mgmt_cmd_resp cmd_resp;
	int ret;

	if (!pfio) {
		pr_err("no pfio obj!");
		return -EINVAL;
	}

	if (!mv_check_eaddr_valid(addr))
		return -EADDRNOTAVAIL;

	/* Management set-address. */
	memcpy(cmd_params.mac_addr, addr, MV_ETHADDR_LEN);

	pr_debug("Sending set-address\n");
	msg_params.cmd_code = CC_PF_MAC_ADDR;
	msg_params.client_id = 0;
	msg_params.client_type = CDT_PF;
	msg_params.msg = &cmd_params;
	msg_params.msg_len = sizeof(cmd_params);
	msg_params.resp_msg = &cmd_resp;
	msg_params.resp_msg_len = sizeof(cmd_resp);

	ret = agnic_mgmt_command_send(pfio, &msg_params);
	if (ret) {
		pr_err("Failed to set %s command!\n", "set-mac-addr");
		return ret;
	}

	memcpy(pfio->mac, addr, MV_ETHADDR_LEN);
	return 0;
}

int agnic_pfio_set_mtu(struct agnic_pfio *pfio, u16 mtu)
{
	int ret;
	struct agnic_msg_params msg_params;
	struct agnic_mgmt_cmd_params cmd_params;
	struct agnic_mgmt_cmd_resp cmd_resp;

	if (mtu > AGNIC_MAX_MTU)
		return -EINVAL;

	msg_params.cmd_code = CC_PF_MTU;
	msg_params.client_id = 0;
	msg_params.client_type = CDT_PF;
	msg_params.msg_len = sizeof(cmd_params);
	msg_params.msg = &cmd_params;

	msg_params.resp_msg = &cmd_resp;
	msg_params.resp_msg_len = sizeof(cmd_resp);

	cmd_params.pf_set_mtu.mtu = mtu;

	ret = agnic_mgmt_command_send(pfio, &msg_params);
	if (ret) {
		pr_err("Failed to set %s command!\n", "mtu");
		return ret;
	}

	pfio->mtu = mtu;

	return 0;
}

int agnic_pfio_get_link_state(struct agnic_pfio *pfio, int *en)
{
	if (!pfio) {
		pr_err("no pfio obj!");
		return -EINVAL;
	}

	*en = pfio->link;
	return 0;
}

/* TODO: implement ...
 *int agnic_pfio_get_mac_addr(struct agnic_pfio *pfio, eth_addr_t addr);
 *int agnic_pfio_get_mtu(struct agnic_pfio *pfio, u16 *mtu);
 *int agnic_pfio_set_mru(struct agnic_pfio *pfio, u16 len);
 *int agnic_pfio_get_mru(struct agnic_pfio *pfio, u16 *len);
 *int agnic_pfio_set_promisc(struct agnic_pfio *pfio, int en);
 *int agnic_pfio_get_promisc(struct agnic_pfio *pfio, int *en);
 *int agnic_pfio_set_mc_promisc(struct agnic_pfio *pfio, int en);
 *int agnic_pfio_get_mc_promisc(struct agnic_pfio *pfio, int *en);
 *int agnic_pfio_add_mac_addr(struct agnic_pfio *pfio, const eth_addr_t addr);
 *int agnic_pfio_remove_mac_addr(struct agnic_pfio *pfio, const eth_addr_t addr);
 *int agnic_pfio_flush_mac_addrs(struct agnic_pfio *pfio, int uc, int mc);
 *int agnic_pfio_get_statistics(struct agnic_pfio *pfio, struct agnic_pfio_statistics *stats);
 *int agnic_pfio_set_link_state(struct agnic_pfio *pfio, int en);
*/
