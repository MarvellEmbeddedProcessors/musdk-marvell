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
#include "lf/pf/pf_mng_cmd_desc.h"
#include "lf/pf/pf_queue_topology.h"
#include "db.h"
#include "mng/mv_nmp_dispatch.h"
#include "dispatch.h"

#define q_inc_idx(q, idx)	((idx + 1) & (q->len - 1))
#define q_rd_idx(idx)		(*((u32 *)idx))
#define q_wr_idx(idx, val)	(*((u32 *)idx) = val)
#define q_rd_cons(q)		q_rd_idx(q->cons_virt)
#define q_rd_prod(q)		q_rd_idx(q->prod_virt)
#define q_wr_cons(q, val)	q_wr_idx(q->cons_virt, val)
#define q_wr_prod(q, val)	q_wr_idx(q->prod_virt, val)

#define q_full(q, p, c)		(((p + 1) & (q->len - 1)) == c)
#define q_empty(p, c)		(p == c)

static int nmdisp_msg_recv(u32 queue_id, struct cmd_desc *msg);
static int nmdisp_msg_transmit(u32 queue_id, struct notif_desc *msg);


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
static int nmdisp_client_id_get(struct nmdisp *nmdisp_p, u8 client, u8 id)
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
		if ((q->cmd_q.q_id == 0) && (q->notify_q.q_id == 0))
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
	nmdisp_p->clients[free_client_idx].client_sr_cb = params->client_sr_cb;
	nmdisp_p->clients[free_client_idx].client       = params->client;

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
	nmdisp_p->clients[client_idx].client_sr_cb = NULL;
	nmdisp_p->clients[client_idx].client       = NULL;

	for (q_idx = 0; q_idx < MV_NMP_Q_PAIR_MAX; q_idx++) {
		q = &(nmdisp_p->clients[client_idx].client_q[q_idx]);
		q->cmd_q.q_id    = 0;
		q->notify_q.q_id = 0;
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
	q->cmd_q.q_id    = q_params->cmd_q.q_id;
	q->notify_q.q_id = q_params->notify_q.q_id;

	return 0;
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
	int ret;
	u32 client_idx;
	u32 q_idx;
	struct nmdisp_client *client_p;
	struct nmdisp_q_pair_params *q;
	struct cmd_desc cmd;

	for (client_idx = 0; client_idx < NMDISP_MAX_CLIENTS; client_idx++) {

		client_p = &(nmdisp_p->clients[client_idx]);
		if (client_p->client_type != 0) {

			for (q_idx = 0; q_idx < MV_NMP_Q_PAIR_MAX; q_idx++) {
				q = &(nmdisp_p->clients[client_idx].client_q[q_idx]);

				ret = nmdisp_msg_recv(q->cmd_q.q_id, &cmd);
				if (ret <= 0)
					return ret;

				ret = client_p->client_sr_cb(client_p->client, cmd.cmd_code, (void *)&cmd);
				if (ret <= 0)
					return ret;
			}
		}
	}

	return 0;
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
int nmdisp_msg_recv(u32 queue_id, struct cmd_desc *msg)
{
	struct db_q *db_q;
	struct mqa_queue_params *queue_info;
	struct cmd_desc *recv_desc;
	u32 cons_idx;
	u32 prod_idx;

	/* Extract queue parameters */
	db_q = db_queue_get(queue_id);
	if (db_q == NULL) {
		pr_err("Failed to get queue %d parameters\n", queue_id);
		return -EINVAL;
	}

	queue_info = &db_q->params;

	cons_idx = q_rd_cons(queue_info);
	prod_idx = q_rd_prod(queue_info);

	/* Check for pending message */
	if (q_empty(prod_idx, cons_idx))
		return 0;

	/* Place message */
	recv_desc = ((struct cmd_desc *)queue_info->virt_base_addr) + cons_idx;
	memcpy(msg, recv_desc, sizeof(struct cmd_desc));

	/* Memory barrier */
	wmb();

	/* Increament queue consumer */
	q_wr_cons(queue_info, q_inc_idx(queue_info, cons_idx));

	return 1;
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
int nmdisp_send(struct nmdisp *nmdisp_p, u8 client, u8 id, u8 qid, void *msg)
{
	int ret;
	u32 client_idx;
	struct nmdisp_client *client_p;

	client_idx = nmdisp_client_id_get(nmdisp_p, client, id);
	if (client_idx < 0) {
		pr_err("Failed to ad queue to dispatcher - client not found\n");
		return -1;
	}

	client_p = &(nmdisp_p->clients[client_idx]);

	ret = nmdisp_msg_transmit(client_p->client_q[qid].notify_q.q_id, (struct notif_desc *)msg);
	if (ret <= 0)
		return ret;

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
int nmdisp_msg_transmit(u32 queue_id, struct notif_desc *msg)
{
	struct db_q *db_q;
	struct mqa_queue_params *queue_info;
	struct notif_desc *trans_desc;
	u32 cons_idx;
	u32 prod_idx;

	/* Retrieve queue parameters */
	db_q = db_queue_get(queue_id);
	if (db_q == NULL) {
		pr_err("Failed to get queue %d parameters\n", queue_id);
		return -EINVAL;
	}

	queue_info = &db_q->params;

	cons_idx = q_rd_cons(queue_info);
	prod_idx = q_rd_prod(queue_info);

	/* Check for free space */
	if (q_full(queue_info, prod_idx, cons_idx))
		return 0;

	/* Place message */
	trans_desc = ((struct notif_desc *)queue_info->virt_base_addr) + prod_idx;
	memcpy(trans_desc, msg, sizeof(struct notif_desc));

	/* Memory barrier */
	wmb();

	/* Increament queue producer */
	q_wr_prod(queue_info, q_inc_idx(queue_info, prod_idx));

	return 0;
}

