/******************************************************************************
 *	Copyright (C) 2017 Marvell International Ltd.
 *
 *  If you received this File from Marvell, you may opt to use, redistribute
 *  and/or modify this File under the following licensing terms.
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *	* Redistributions of source code must retain the above copyright
 *	  notice, this list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the distribution.
 *
 *	* Neither the name of Marvell nor the names of its contributors may be
 *	  used to endorse or promote products derived from this software
 *	  without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#define log_fmt(fmt) "dispatch: " fmt

#include "std_internal.h"
#include "drivers/mv_mqa.h"
#include "drivers/mv_mqa_queue.h"
#include "lf/mng_cmd_desc.h"
#include "lf/pf/pf_queue_topology.h"
#include "db.h"
#include "dispatch.h"

#define q_inc_idx(q, idx)	q_inc_idx_val(q, idx, 1)
#define q_inc_idx_val(q, idx, val)	((idx + val) & (q->len - 1))
#define q_rd_idx(idx)		(*((u32 *)idx))
#define q_wr_idx(idx, val)	(*((u32 *)idx) = val)
#define q_rd_cons(q)		q_rd_idx(q->cons_virt)
#define q_rd_prod(q)		q_rd_idx(q->prod_virt)
#define q_wr_cons(q, val)	q_wr_idx(q->cons_virt, val)
#define q_wr_prod(q, val)	q_wr_idx(q->prod_virt, val)

#define q_full(q, p, c)		(((p + 1) & (q->len - 1)) == c)
#define q_empty(p, c)		(p == c)
#define q_occupancy(q, p, c)	((p - c + q->len) & (q->len - 1))
#define q_space(q, p, c)	(q->len - q_occupancy(q, p, c) - 1)

#define MSG_WAS_RECV	1
#define MSG_Q_IS_EMPTY	0

static int nmdisp_msg_recv(struct nmdisp *nmdisp, struct mqa_q *q, struct nmdisp_msg *msg);
static int nmdisp_msg_transmit(struct mqa_q *q, int ext_desc_support, struct nmdisp_msg *msg);


/*
 *	nmdisp_client_id_get
 *
 *	This function return client index in dispatcher client array
 *
 *	@param[in]	nmdisp - pointer to dispatcher object
 *	@param[in]	client - client type
 *	@param[in]	id - client id
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
static inline int nmdisp_client_id_get(struct nmdisp *nmdisp_p, u8 client, u8 id)
{
	u32 client_idx;

	for (client_idx = 0; client_idx < NMDISP_MAX_CLIENTS; client_idx++) {
		if ((nmdisp_p->clients[client_idx].client_type == client) &&
			(nmdisp_p->clients[client_idx].client_id   == id)) {
			return client_idx;
		}
	}

	return -1;
}


/*
 *	nmdisp_free_client_get
 *
 *	This function next available index in dispatcher client array
 *
 *	@param[in]	nmdisp - pointer to dispatcher object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
static int nmdisp_free_client_get(struct nmdisp *nmdisp_p)
{
	u32 client_idx;

	for (client_idx = 0; client_idx < NMDISP_MAX_CLIENTS; client_idx++) {
		if (nmdisp_p->clients[client_idx].client_type == 0)
			return client_idx;
	}

	return -1;
}


/*
 *	nmdisp_free_client_q_get
 *
 *	This function return next available queue index in dispatcher client array
 *
 *	@param[in]	nmdisp - pointer to dispatcher object
 *	@param[in]	client - client type
 *	@param[in]	id - client id
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
static int nmdisp_free_client_q_get(struct nmdisp *nmdisp_p, u8 client, u8 id)
{
	int client_idx;
	u32 q_idx;
	struct nmdisp_q_pair_params *q;

	client_idx = nmdisp_client_id_get(nmdisp_p, client, id);
	if (client_idx < 0) {
		pr_err("Failed to find dispatcher client - client id-%d not found\n", id);
		return client_idx;
	}

	for (q_idx = 0; q_idx < MV_NMP_Q_PAIR_MAX; q_idx++) {
		q = &(nmdisp_p->clients[client_idx].client_q[q_idx]);
		if ((q->cmd_q == NULL) && (q->notify_q == NULL))
			return q_idx;
	}

	return -1;
}


/*
 *	nmdisp_init
 *
 *	This function initialize dispatcher handler
 *
 *	@param[in]	nmdisp - pointer to dispatcher object
 *	@param[in]	params - pointer to dispatcher parameters
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int nmdisp_init(struct nmdisp_params *params, struct nmdisp **nmdisp)
{
	params = params;

	*nmdisp = kcalloc(1, sizeof(struct nmdisp), GFP_KERNEL);
	if (*nmdisp == NULL) {
		pr_err("Failed to allocate dispatcher handler\n");
		return -1;
	}

	memset((void *)*nmdisp, 0, sizeof(struct nmdisp));

	(*nmdisp)->max_msg_size = 0;

	return 0;
}


/*
 *	nmdisp_deinit
 *
 *	This function release dispatcher handler
 *
 *	@param[in]	nmdisp - pointer to dispatcher object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
void nmdisp_deinit(struct nmdisp *nmdisp)
{
	kfree(nmdisp->cmd_msg);
	kfree(nmdisp);
}


/*
 *	nmdisp_register_client
 *
 *	This function register a client (PF/VF/..) to the dispatcher
 *
 *	@param[in]	nmdisp - pointer to dispatcher object
 *	@param[in]	params - pointer to dispatcher client parameters
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int nmdisp_register_client(struct nmdisp *nmdisp_p, struct nmdisp_client_params *params)
{
	int free_client_idx;

	free_client_idx = nmdisp_free_client_get(nmdisp_p);
	if (free_client_idx < 0) {
		pr_err("Failed to register dispatcher client - No free slot\n");
		return -1;
	}

	if (params->client_type > NMDISP_MAX_CLIENTS_TYPE) {
		pr_err("Failed to register dispatcher client - client-%d exceed max\n", params->client_type);
		return -1;
	}

	if (params->client_id > NMDISP_MAX_CLIENTS_ID) {
		pr_err("Failed to register dispatcher client - client id-%d exceed max\n", params->client_id);
		return -1;
	}

	/* configure dispatcher client table */
	nmdisp_p->clients[free_client_idx].client_type  = params->client_type;
	nmdisp_p->clients[free_client_idx].client_id    = params->client_id;
	nmdisp_p->clients[free_client_idx].client_ctrl_cb = params->f_client_ctrl_cb;
	nmdisp_p->clients[free_client_idx].client       = params->client;

	pr_debug("nmdisp_register_client idx %d, client type %d, client Id %d\n", free_client_idx,
			nmdisp_p->clients[free_client_idx].client_type,
			nmdisp_p->clients[free_client_idx].client_id);


	return 0;
}


/*
 *	nmdisp_deregister_client
 *
 *	This function deregister a client (PF/VF/..) from the dispatcher
 *
 *	@param[in]	nmdisp - pointer to dispatcher object
 *	@param[in]	client - client type
 *	@param[in]	id - client id
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int nmdisp_deregister_client(struct nmdisp *nmdisp_p, u8 client, u8 id)
{
	int client_idx;
	u32 q_idx;
	struct nmdisp_q_pair_params *q;

	if (client > NMDISP_MAX_CLIENTS) {
		pr_err("Failed to deregister dispatcher client - client-%d exceed max\n", client);
		return -1;
	}

	if (id > NMDISP_MAX_CLIENTS_ID) {
		pr_err("Failed to deregister dispatcher client - client id-%d exceed max\n", id);
		return -1;
	}

	client_idx = nmdisp_client_id_get(nmdisp_p, client, id);
	if (client_idx < 0) {
		pr_err("Failed to deregister dispatcher client - slot not found\n");
		return -1;
	}

	/* clear dispatcher client table */
	nmdisp_p->clients[client_idx].client_type  = 0;
	nmdisp_p->clients[client_idx].client_id    = 0;
	nmdisp_p->clients[client_idx].client_ctrl_cb = NULL;
	nmdisp_p->clients[client_idx].client       = NULL;

	for (q_idx = 0; q_idx < MV_NMP_Q_PAIR_MAX; q_idx++) {
		q = &(nmdisp_p->clients[client_idx].client_q[q_idx]);
		q->cmd_q->q_id    = 0;
		q->notify_q->q_id = 0;
	}

	return 0;
}

/*
 *	nmdisp_add_queue
 *
 *	This function add dispatcher queue pair to dispatcher client
 *
 *	@param[in]	nmdisp - pointer to dispatcher object
 *	@param[in]	client - client type
 *	@param[in]	id - client id
 *	@param[in]	q_params - client queue pair
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int nmdisp_add_queue(struct nmdisp *nmdisp_p, u8 client, u8 id, struct nmdisp_q_pair_params *q_params)
{
	u32 client_idx;
	u32 q_idx;
	struct nmdisp_q_pair_params *q;

	client_idx = nmdisp_client_id_get(nmdisp_p, client, id);
	if (client_idx < 0) {
		pr_err("Failed to ad queue to dispatcher - client not found\n");
		return -1;
	}

	q_idx = nmdisp_free_client_q_get(nmdisp_p, client, id);
	if (q_idx < 0) {
		pr_err("Failed to ad queue to dispatcher - no free queue\n");
		return -1;
	}

	/* configure dispatcher client queue table */
	q = &(nmdisp_p->clients[client_idx].client_q[q_idx]);
	q->cmd_q    = q_params->cmd_q;
	q->notify_q = q_params->notify_q;
	q->ext_desc_support = q_params->ext_desc_support;
	q->max_msg_size = q_params->max_msg_size;

	nmdisp_p->max_msg_size = max(nmdisp_p->max_msg_size, q->max_msg_size);

	pr_debug("nmdisp_add_queue client idx %d, q_idx %d, cmd_q %d, notify_q %d\n",
			client_idx, q_idx, q->cmd_q->q_id, q->notify_q->q_id);

	return 0;
}

/*
 *	nmdisp_dispatch_dump
 */
void nmdisp_dispatch_dump(struct nmdisp *nmdisp_p)
{
	u32 client_idx;
	u32 q_idx;
	struct nmdisp_client *client_p;
	struct nmdisp_q_pair_params *q;

	pr_info("nmdisp_dispatch Info\n");

	for (client_idx = 0; client_idx < NMDISP_MAX_CLIENTS; client_idx++) {
		client_p = &(nmdisp_p->clients[client_idx]);
		if (client_p->client_type != CDT_INVALID)
			pr_info("client idx = %d  type %d  id %d\n",
					client_idx, client_p->client_type, client_p->client_id);

		for (q_idx = 0; q_idx < MV_NMP_Q_PAIR_MAX; q_idx++) {
			q = &(nmdisp_p->clients[client_idx].client_q[q_idx]);

			if (q->cmd_q == NULL)
				continue;

			pr_info("	q_idx = %d, cmd %d, notify %d\n",
					q_idx, q->cmd_q->q_id, q->notify_q->q_id);
		}
	}
}


/*
 *	nmdisp_dispatch
 *
 *	This function execute dispatcher functionality
 *
 *	@param[in]	nmdisp - pointer to dispatcher object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int nmdisp_dispatch(struct nmdisp *nmdisp_p)
{
	int ret, dst_client_idx;
	u32 client_idx;
	u32 q_idx;
	struct nmdisp_client *client_p, *dst_client_p;
	struct nmdisp_q_pair_params *q;
	struct nmdisp_msg msg;

	/* only on first time need to allocate the message buffer according to the maximum message length */
	if (!nmdisp_p->cmd_msg) {
		nmdisp_p->cmd_msg = kcalloc(1, nmdisp_p->max_msg_size, GFP_KERNEL);
		if (nmdisp_p->cmd_msg == NULL) {
			pr_err("Failed to allocate message buffer\n");
			return -1;
		}
	}

	for (client_idx = 0; client_idx < NMDISP_MAX_CLIENTS; client_idx++) {

		client_p = &(nmdisp_p->clients[client_idx]);
		if (client_p->client_type != 0) {

			for (q_idx = 0; q_idx < MV_NMP_Q_PAIR_MAX; q_idx++) {
				q = &(nmdisp_p->clients[client_idx].client_q[q_idx]);

				if (q->cmd_q == NULL)
					continue;

				msg.src_client = client_p->client_type;
				msg.src_id = client_p->client_id;
				ret = nmdisp_msg_recv(nmdisp_p, q->cmd_q, &msg);
				if (ret == MSG_Q_IS_EMPTY)
					/* queue is empty */
					continue;
				if (ret < 0)
					return ret;

				pr_debug("recv: client idx %d q_idx %d cmd_q %d\n",
						client_idx, q_idx, q->cmd_q->q_id);
				pr_debug("      src_client %d src_id %d dst_client %d dst_id %d\n",
						msg.src_client, msg.src_id, msg.dst_client, msg.dst_id);

				dst_client_idx = nmdisp_client_id_get(nmdisp_p, msg.dst_client, msg.dst_id);
				if (dst_client_idx < 0) {
					pr_err("can't dispatch msg, dst-client (%d, %d) not found\n",
						msg.dst_client, msg.dst_id);
					return -1;
				}

				pr_debug("recv: dst_client_idx %d\n\n", dst_client_idx);

				dst_client_p = &nmdisp_p->clients[dst_client_idx];
				ret = dst_client_p->client_ctrl_cb(dst_client_p->client, &msg);
				if (ret < 0)
					return ret;
			}
		}
	}

	return 0;
}

static inline int nmdisp_msg_recv_ext_descs(struct nmdisp *nmdisp,
					    struct mqa_q *q,
					    struct nmdisp_msg *msg,
					    u32 *cons_idx,
					    u8 num_ext_descs,
					    struct cmd_desc *recv_desc)
{
	u16 len;
	int ret = MSG_WAS_RECV;

	len = sizeof(recv_desc->data) + sizeof(struct cmd_desc) * num_ext_descs;

	if (len > nmdisp->max_msg_size) {
		pr_err("message length (%u) must be up to %u\n", len, nmdisp->max_msg_size);
		/* Need to mark the descriptors as consumed and exit */
		ret = -1;
		goto exit;
	}
	/* In case there is a wrap around the descriptors are be stored to the
	 * end of the ring AND from the beginning of the desc ring.
	 * So in order to keep the non-wrap code the same, we first copy the descs from the beginning of
	 * the ring.
	 */
	if (unlikely((*cons_idx + (num_ext_descs + 1)) > q->len)) {
		u8 num_ext_desc_post_wrap = (*cons_idx + num_ext_descs + 1) - q->len;
		u16 len_post_wrap = sizeof(struct cmd_desc) * num_ext_desc_post_wrap;

		/* Update len "pre wrap" */
		len -= len_post_wrap;
		/* copy post wrap part */
		memcpy(&((u8 *)msg->msg)[len], q->virt_base_addr, len_post_wrap);
		msg->msg_len += len_post_wrap;
	}

	memcpy(msg->msg, recv_desc->data, len);
	msg->msg_len += len;
exit:
	*cons_idx = q_inc_idx_val(q, *cons_idx, 1 + num_ext_descs);

	return ret;
}

static inline int nmdisp_msg_recv_sg(struct nmdisp *nmdisp,
				     struct mqa_q *q,
				     struct nmdisp_msg *msg,
				     u32 *cons_idx,
				     u8 buf_pos,
				     struct cmd_desc *recv_desc)
{
	int ret = MSG_WAS_RECV, skip_copy = false;
	u32 prod_idx;

	prod_idx = q_rd_prod(q);

	/* This is a S/G message. Need to loop the descs until either one of the following:
	 * 1. buff-pos is LAST - S/G completion
	 * 2. buff-pos is single or external - S/G error
	 * 3. cmd_indx!=fisrt_cmd_indx - S/G error
	 * 4. num_ext_desc != 0 - S/G error
	 * 5. ring is empty - S/G error
	 */
	u16 first_cmd_idx = recv_desc->cmd_idx;

	while (1) {
		if ((msg->msg_len + sizeof(recv_desc->data)) > nmdisp->max_msg_size) {
			pr_err("message length (%u) must be up to %u\n",
			       (u32)(msg->msg_len + sizeof(recv_desc->data)), nmdisp->max_msg_size);

			/* need to mark the descriptors until the 'last' descriptor as consumed without
			 * copy it and return error
			 */
			skip_copy = true;
			msg->msg_len = 0; /* So we won't enter this code again */
			ret = -1;
		}
		if (!skip_copy) {
			memcpy(&((u8 *)msg->msg)[msg->msg_len], recv_desc->data, sizeof(recv_desc->data));
			msg->msg_len += sizeof(recv_desc->data);
		}
		*cons_idx = q_inc_idx(q, *cons_idx);
		if (buf_pos == CMD_FLAG_BUF_POS_LAST)
			break;
		recv_desc = ((struct cmd_desc *)q->virt_base_addr) + *cons_idx;
		buf_pos = CMD_FLAGS_BUF_POS_GET(recv_desc->flags);
		if ((first_cmd_idx != recv_desc->cmd_idx) ||
		    ((buf_pos ==  CMD_FLAG_BUF_POS_SINGLE) ||  (buf_pos == CMD_FLAG_BUF_POS_EXT_BUF)) ||
		    (CMD_FLAGS_NUM_EXT_DESC_GET(recv_desc->flags) != 0) ||
		    (q_empty(prod_idx, *cons_idx))) {
			return -1;
		}
	}

	return ret;

}

/*
 *	nmdisp_msg_recv
 *
 *	This function reads a message from control channel,
 *	The function handles control channel internals (producer / consumer)
 *
 *	@param[in]	queue_id - queue Id to receive message
 *	@param[in]	msg - management command
 *
 *	@retval	= 0 no message received
 *	@retval	= 1 yes, message received
 *	@retval	error-code otherwise
 */
static int nmdisp_msg_recv(struct nmdisp *nmdisp, struct mqa_q *q, struct nmdisp_msg *msg)
{
	struct cmd_desc *recv_desc;
	u8 num_ext_descs, buf_pos;
	u32 cons_idx, prod_idx;
	int ret = MSG_WAS_RECV;

	cons_idx = q_rd_cons(q);
	prod_idx = q_rd_prod(q);

	/* Check for pending message */
	if (q_empty(prod_idx, cons_idx))
		return MSG_Q_IS_EMPTY;

	/* Place message */
	recv_desc = ((struct cmd_desc *)q->virt_base_addr) + cons_idx;

	msg->ext = 1;
	msg->dst_client = recv_desc->client_type;
	msg->dst_id = recv_desc->client_id;
	msg->code = recv_desc->cmd_code;
	msg->indx = recv_desc->cmd_idx;
	msg->msg = nmdisp->cmd_msg;
	msg->msg_len = 0;

	num_ext_descs = CMD_FLAGS_NUM_EXT_DESC_GET(recv_desc->flags);
	buf_pos = CMD_FLAGS_BUF_POS_GET(recv_desc->flags);
	if (num_ext_descs) {
		ret = nmdisp_msg_recv_ext_descs(nmdisp, q, msg, &cons_idx, num_ext_descs, recv_desc);
	} else if (buf_pos == CMD_FLAG_BUF_POS_FIRST_MID) {
		ret = nmdisp_msg_recv_sg(nmdisp, q, msg, &cons_idx, buf_pos, recv_desc);
	} else if (buf_pos == CMD_FLAG_BUF_POS_EXT_BUF) {
		pr_err("No support for external buffer\n");
		cons_idx = q_inc_idx(q, cons_idx);
		ret = -1;
	} else {
		/* Single desc */
		memcpy(msg->msg, recv_desc->data, sizeof(recv_desc->data));
		msg->msg_len += sizeof(recv_desc->data);
		cons_idx = q_inc_idx(q, cons_idx);
	}

	/* Memory barrier */
	wmb();

	/* Increament queue consumer */
	q_wr_cons(q, cons_idx);

	return ret;
}


/*
 *	nmdisp_send
 *
 *	This function send a message to the notification channel,
 *	The transmitted message can be notification or response to command execution
 *
 *	@param[in]	nmdisp - pointer to dispatcher object
 *	@param[in]	client - client type
 *	@param[in]	id - client id
 *	@param[in]	msg - management reply
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int nmdisp_send_msg(struct nmdisp *nmdisp, u8 qid, struct nmdisp_msg *msg)
{
	int ret;
	u32 client_idx;
	struct nmdisp_client *client_p;

	client_idx = nmdisp_client_id_get(nmdisp, msg->dst_client, msg->dst_id);
	if (client_idx < 0) {
		pr_err("Failed to ad queue to dispatcher - client not found\n");
		return -1;
	}

	client_p = &(nmdisp->clients[client_idx]);

	if (msg->ext) {
		ret = nmdisp_msg_transmit(client_p->client_q[qid].notify_q,
					  client_p->client_q[qid].ext_desc_support,
					  msg);
	} else {
		ret = client_p->client_ctrl_cb(client_p->client, msg);
	}
	if (ret <= 0)
		return ret;

	return 0;
}

static inline int nmdisp_msg_transmit_ext_descs(struct mqa_q *q,
						struct nmdisp_msg *nmdisp_msg,
						u32 *prod_idx,
						u8 free_descs,
						struct cmd_desc *trans_desc)
{
	u8 num_ext_descs;

	/* 'num_ext_descs' represent the number of descriptors that are used as pure data.
	 * To calculate it we need to subtract the size of data portion of the first descriptor
	 * and to divide it by the size of command-descriptor.
	 * for more details please refer to the A8K_NMP_Descriptor_Format.xls
	 */
	num_ext_descs = ceil((nmdisp_msg->msg_len - MGMT_DESC_DATA_LEN), sizeof(struct cmd_desc));

	if (free_descs < (1 + num_ext_descs)) {
		pr_err("Not enogth free descriptors (%u) to hold the message length (%u).\n"
			, free_descs, nmdisp_msg->msg_len);
		return -ENOSPC;
	}

	CMD_FLAGS_NUM_EXT_DESC_SET(trans_desc->flags, num_ext_descs);

	/* In case there is a wrap around the descriptors are be stored to the
	 * end of the ring AND from the beginning of the desc ring.
	 * So in order to keep the non-wrap code the same, we first copy the descs from the beginning of
	 * the ring.
	 */
	if (unlikely((*prod_idx + (num_ext_descs + 1)) > q->len)) {
		u8 num_ext_desc_post_wrap = (*prod_idx + num_ext_descs + 1) - q->len;
		u16 len_pre_wrap = (num_ext_descs - num_ext_desc_post_wrap) * sizeof(struct cmd_desc) +
				   MGMT_DESC_DATA_LEN;
		u16 len_post_wrap = nmdisp_msg->msg_len - len_pre_wrap;

		/* Update len "pre wrap" */
		nmdisp_msg->msg_len = len_pre_wrap;
		/* copy post wrap part */
		memcpy(q->virt_base_addr, &((u8 *)nmdisp_msg->msg)[nmdisp_msg->msg_len], len_post_wrap);
	}

	memcpy(trans_desc->data, nmdisp_msg->msg, nmdisp_msg->msg_len);
	*prod_idx = q_inc_idx_val(q, *prod_idx, (1 + num_ext_descs));

	return 0;
}

static inline int nmdisp_msg_transmit_sg(struct mqa_q *q,
					 struct nmdisp_msg *nmdisp_msg,
					 u32 *prod_idx,
					 u8 free_descs,
					 struct cmd_desc *trans_desc)
{
	u8 num_descs, num_to_copy;
	u16 data_pos = 0;
	struct cmd_desc *first_desc = trans_desc;

	if (nmdisp_msg->indx == CMD_ID_NOTIFICATION) {
		pr_err("S/G cannot be applied on Notification message (as the CMD_indx is used to identify start of new S/G)\n");
		return -1;
	}

	/* 'num_descs' represent the number of descriptors that are used as command descriptor.
	 * To calculate it we need to divide the length by the size of descriptor data portion.
	 * for more details please refer to the A8K_NMP_Descriptor_Format.xls
	 */
	num_descs = ceil(nmdisp_msg->msg_len, MGMT_DESC_DATA_LEN);

	if (free_descs < num_descs) {
		pr_err("Not enogth free descriptors (%u) to hold the message length (%u).\n"
			, free_descs, nmdisp_msg->msg_len);
		return -ENOSPC;
	}

	CMD_FLAGS_BUF_POS_SET(first_desc->flags, CMD_FLAG_BUF_POS_FIRST_MID);

	do {
		trans_desc = ((struct cmd_desc *)q->virt_base_addr) + *prod_idx;
		memcpy(trans_desc, first_desc, 8);
		num_to_copy = min(MGMT_DESC_DATA_LEN, nmdisp_msg->msg_len - data_pos);
		memcpy(trans_desc->data, nmdisp_msg->msg + data_pos, num_to_copy);
		data_pos += num_to_copy;
		*prod_idx = q_inc_idx(q, *prod_idx);
	} while (data_pos < nmdisp_msg->msg_len);

	CMD_FLAGS_BUF_POS_SET(trans_desc->flags, CMD_FLAG_BUF_POS_LAST);

	return 0;
}

/*
 *	nmdisp_msg_transmit
 *
 *	This function writes a message to the notification channel,
 *	The transmitted message can be notification or response to command execution
 *	The function handles control channel internals (producer / consumer)
 *
 *	@param[in]	queue_id - queue Id to sent message
 *	@param[in]	msg - management command
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int nmdisp_msg_transmit(struct mqa_q *q, int ext_desc_support, struct nmdisp_msg *nmdisp_msg)
{
	struct cmd_desc *trans_desc;
	u32 cons_idx;
	u32 prod_idx;
	u8 free_descs;
	int ret;

	cons_idx = q_rd_cons(q);
	prod_idx = q_rd_prod(q);

	/* Check for free space */
	if (q_full(q, prod_idx, cons_idx))
		return 0;

	/* Place message */
	trans_desc = ((struct cmd_desc *)q->virt_base_addr) + prod_idx;
	trans_desc->cmd_idx = nmdisp_msg->indx;
/*	trans_desc->app_code =*/
	trans_desc->cmd_code = nmdisp_msg->code;
	trans_desc->client_type = nmdisp_msg->src_client;
	trans_desc->client_id = nmdisp_msg->src_id;
	trans_desc->flags = 0;

	if (nmdisp_msg->msg_len > MGMT_DESC_DATA_LEN) {
		/* S/G */
		/* Calculate number of free descriptors */
		free_descs = q_space(q, prod_idx, cons_idx);

		if (ext_desc_support) {
			ret = nmdisp_msg_transmit_ext_descs(q, nmdisp_msg, &prod_idx, free_descs, trans_desc);
			if (ret)
				return ret;
		} else {
			ret = nmdisp_msg_transmit_sg(q, nmdisp_msg, &prod_idx, free_descs, trans_desc);
			if (ret)
				return ret;
		}
	} else {
		/* Single desc */
		memcpy(trans_desc->data, nmdisp_msg->msg, nmdisp_msg->msg_len);
		prod_idx = q_inc_idx(q, prod_idx);
	}

	/* Memory barrier */
	wmb();

	/* Increament queue producer */
	q_wr_prod(q, prod_idx);

	return 0;
}

