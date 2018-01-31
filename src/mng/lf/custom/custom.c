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

#define log_fmt(fmt) "pf: " fmt

#include "std_internal.h"
#include "drivers/mv_mqa.h"
#include "drivers/mv_mqa_queue.h"
#include "mng/lf/mng_cmd_desc.h"
#include "mng/db.h"
#include "mng/mv_nmp.h"
#include "mng/dispatch.h"
#include "lib/lib_misc.h"
#include "custom.h"

#define MAX_MSG_DATA_LEN	1024
#define CMD_QUEUE_SIZE	\
		(1 + ceil((MAX_MSG_DATA_LEN - MGMT_DESC_DATA_LEN), sizeof(struct cmd_desc)))

/*
 *	nmcstm_mng_chn_init
 *
 *	This function create CUSTOM management channel
 *
 */
static int nmcstm_mng_chn_init(struct nmcstm *nmcstm)
{
	u32 cmd_queue, notify_queue;
	int ret = 0;

	struct mqa_queue_params params;
	struct mqa_q *cmd_queue_p    = NULL;
	struct mqa_q *notify_queue_p = NULL;

	/* Allocate and Register Command queue in MQA */
	pr_info("Register Command Q\n");

	/* Allocate queue from MQA */
	ret = mqa_queue_alloc(nmcstm->mqa, &cmd_queue);
	if (ret < 0) {
		pr_err("Failed to allocate queue from MQA\n");
		goto exit_error;
	}

	memset(&params, 0, sizeof(struct mqa_queue_params));

	params.idx  = cmd_queue;
	params.len  = CMD_QUEUE_SIZE;
	params.size = sizeof(struct cmd_desc);
	params.attr = LOCAL_QUEUE | EGRESS_QUEUE;
	params.prio = 0;

	ret = mqa_queue_create(nmcstm->mqa, &params, &cmd_queue_p);
	if (ret < 0) {
		pr_info("Failed to create Custom Management CMD Q\n");
		goto exit_error;
	}

	nmcstm->mng_ctrl.cmd_queue = cmd_queue_p;

	/* Allocate and Register Local Notification queue in MQA */
	pr_info("Register Notification Q\n");

	/* Allocate queue from MQA */
	ret = mqa_queue_alloc(nmcstm->mqa, &notify_queue);
	if (ret < 0) {
		pr_err("Failed to allocate queue from MQA\n");
		goto exit_error;
	}
	memset(&params, 0, sizeof(struct mqa_queue_params));

	params.idx   = notify_queue;
	params.len   = CMD_QUEUE_SIZE;
	params.size  = sizeof(struct cmd_desc);
	params.attr  = LOCAL_QUEUE | INGRESS_QUEUE;
	params.prio  = 0;

	ret = mqa_queue_create(nmcstm->mqa, &params, &notify_queue_p);
	if (ret < 0) {
		pr_info("Failed to create Custom Management Notify Q\n");
		goto exit_error;
	}

	nmcstm->mng_ctrl.notify_queue = notify_queue_p;

	pr_debug("Custom CMD Queue Params:\n");
	pr_debug("\tdesc_ring_base phys %p\n", nmcstm->mng_ctrl.cmd_queue->phy_base_addr);
	pr_debug("\tdesc_ring_base virt %p\n", nmcstm->mng_ctrl.cmd_queue->virt_base_addr);
	pr_debug("\tcons_addr phys %p\n", nmcstm->mng_ctrl.cmd_queue->cons_phys);
	pr_debug("\tcons_addr phys %p\n", nmcstm->mng_ctrl.cmd_queue->cons_virt);
	pr_debug("\tprod_addr phys %p\n", nmcstm->mng_ctrl.cmd_queue->prod_phys);
	pr_debug("\tprod_addr virt %p\n", nmcstm->mng_ctrl.cmd_queue->prod_virt);
	pr_debug("\tlen 0x%x\n", nmcstm->mng_ctrl.cmd_queue->len);

	pr_debug("Custom NOTIFY Queue Params:\n");
	pr_debug("\tdesc_ring_base phys %p\n", nmcstm->mng_ctrl.notify_queue->phy_base_addr);
	pr_debug("\tdesc_ring_base virt %p\n", nmcstm->mng_ctrl.notify_queue->virt_base_addr);
	pr_debug("\tcons_addr phys %p\n", nmcstm->mng_ctrl.notify_queue->cons_phys);
	pr_debug("\tcons_addr phys %p\n", nmcstm->mng_ctrl.notify_queue->cons_virt);
	pr_debug("\tprod_addr phys %p\n", nmcstm->mng_ctrl.notify_queue->prod_phys);
	pr_debug("\tprod_addr virt %p\n", nmcstm->mng_ctrl.notify_queue->prod_virt);
	pr_debug("\tlen 0x%x\n", nmcstm->mng_ctrl.notify_queue->len);

	return 0;

exit_error:

	if (cmd_queue >= 0) {
		if (cmd_queue_p) {
			ret = mqa_queue_destroy(nmcstm->mqa, cmd_queue_p);
			if (ret < 0)
				pr_err("Failed to free Cmd Q %d in DB\n", cmd_queue);
			kfree(cmd_queue_p);
		}
		ret = mqa_queue_free(nmcstm->mqa, cmd_queue);
		if (ret < 0)
			pr_err("Failed to free Cmd Q %d in MQA\n", cmd_queue);
	}

	if (notify_queue >= 0) {
		if (notify_queue_p) {
			ret = mqa_queue_destroy(nmcstm->mqa, notify_queue_p);
			if (ret < 0)
				pr_err("Failed to free Notify Q %d in DB\n", notify_queue);
			kfree(notify_queue_p);
		}
		ret = mqa_queue_free(nmcstm->mqa, notify_queue);
		if (ret < 0)
			pr_err("Failed to free Notify Q %d in MQA\n", notify_queue);
	}

	return ret;
}

/*
 *	nmcstm_mng_chn_terminate
 *
 *	This function terminate CUSTOM management channel
 *
 */
static int nmcstm_mng_chn_terminate(struct nmcstm *nmcstm)
{
	int cmd_queue, notify_queue;
	int ret = 0;

	struct mqa_q *cmd_queue_p    = nmcstm->mng_ctrl.cmd_queue;
	struct mqa_q *notify_queue_p = nmcstm->mng_ctrl.notify_queue;

	if (cmd_queue_p) {
		cmd_queue = cmd_queue_p->q_id;
		ret = mqa_queue_destroy(nmcstm->mqa, cmd_queue_p);
		if (ret < 0)
			pr_err("Failed to free Cmd Q %d in DB\n", cmd_queue);
		ret = mqa_queue_free(nmcstm->mqa, cmd_queue);
		if (ret < 0)
			pr_err("Failed to free Cmd Q %d in MQA\n", cmd_queue);

		kfree(cmd_queue_p);
	}

	if (notify_queue_p) {
		notify_queue = notify_queue_p->q_id;
		ret = mqa_queue_destroy(nmcstm->mqa, notify_queue_p);
		if (ret < 0)
			pr_err("Failed to free Notify Q %d in DB\n", notify_queue);
		ret = mqa_queue_free(nmcstm->mqa, notify_queue);
		if (ret < 0)
			pr_err("Failed to free Notify Q %d in MQA\n", notify_queue);

		kfree(notify_queue_p);
	}

	return 0;
}

/*
 *	nmcstm_init
 *
 *	@param[in]	params - custom module parameters
 *	@param[in]	nmcstm - pointer to custom object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int nmcstm_init(struct nmcstm_params *params, struct nmcstm **nmcstm_ptr)
{
	int ret;
	struct nmdisp_client_params client_params;
	struct nmdisp_q_pair_params q_params;
	struct nmcstm *nmcstm;

	nmcstm = kzalloc(sizeof(struct nmcstm), GFP_KERNEL);
	if (nmcstm == NULL)
		return -ENOMEM;

	nmcstm->mqa = params->mqa;
	nmcstm->nmdisp = params->nmdisp;
	nmcstm->id = params->id;
	nmcstm->pf_id = params->pf_id;

	/* Initialize management queues */
	ret = nmcstm_mng_chn_init(nmcstm);
	if (ret)
		goto init_exit;

	/* Register NIC PF to dispatcher */
	client_params.client_type	= CDT_CUSTOM;
	client_params.client_id		= nmcstm->id;
	client_params.f_client_ctrl_cb	= nmcstm_process_command;
	client_params.client		= nmcstm;
	ret = nmdisp_register_client(nmcstm->nmdisp, &client_params);
	if (ret)
		goto init_exit;

	/* Add management queues to dispatcher */
	q_params.cmd_q    = nmcstm->mng_ctrl.cmd_queue;
	q_params.notify_q = nmcstm->mng_ctrl.notify_queue;
	q_params.ext_desc_support = 1;
	q_params.max_msg_size = MAX_MSG_DATA_LEN;
	ret = nmdisp_add_queue(nmcstm->nmdisp, client_params.client_type, client_params.client_id, &q_params);
	if (ret)
		goto init_exit;

	*nmcstm_ptr = nmcstm;

init_exit:
	if (ret) {
		nmcstm_deinit(nmcstm);
		kfree(nmcstm);
	}

	return ret;
}


/*
 *	nmcstm_deinit
 *
 *	@param[in]	nmcstm - pointer to custom object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int nmcstm_deinit(struct nmcstm *nmcstm)
{
	int ret;

	ret = nmdisp_deregister_client(nmcstm->nmdisp, CDT_CUSTOM, nmcstm->id);
	if (ret)
		return ret;

	ret = nmcstm_mng_chn_terminate(nmcstm);
	if (ret)
		return ret;

	pr_info("Terminating NIC PF\n");

	return 0;
}


/*
 *	nmcstm_process_command
 *
 *	This function process Custom commands
 *
 *	@param[in]	nmcstm - pointer to Custom object
 *	@param[in]	cmd_code
 *	@param[in]	cmd - pointer to cmd_desc object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int nmcstm_process_command(void *arg, struct nmdisp_msg *msg)
{
	struct nmcstm *nmcstm = (struct nmcstm *)arg;
	int ret = 0;

/*
 *	In case the command received from netdev inband-management (src-client/ID' is LF such as NIC-PF and 'ext'
 *	is set), it shall then validate the 'code' (against the registered custom codes) and then forward the message
 *	to its guest by calling 'nmdisp_send_msg' API with its 'src-client/ID', its 'dst-client/ID' and 'ext' is set.
 *
 *	In case the 'src-client/id' is LF (either NIC-PF or the NIC-PF netdev) and 'ext' is not set,
 *	the Custom shell validate the 'code' (against the registered LF codes) and then forward the message to its guest
 *	by calling 'nmdisp_send_msg' API with the LF 'src-client/ID', and 'ext' set.
 *
 *	In case the 'src-client' is external source of Custom (i.e. the Custom's guest), the Custom may either terminate
 *	the command or forward the message by calling 'nmdisp_send_msg' API with its own 'src-client/ID',
 *	'dst-client/id' of its registered LF (i.e. NIC-PF) and 'ext' is not set.
 */

	pr_debug("Custom-Lf got %s command code %d from client-type %d client-id %d msg: 0x%x\n",
		(msg->ext) ? "external":"internal", msg->code, msg->src_client, msg->src_id, *(u32 *)msg->msg);

	if (msg->ext) {
		if (msg->src_client == CDT_CUSTOM) {
			msg->dst_client = CDT_PF;
			msg->dst_id = nmcstm->pf_id;
			msg->ext = 0;
		} else {
			/* inband-management */
			msg->src_client = CDT_CUSTOM;
			msg->src_id = nmcstm->id;
		}
	} else {
		/* internal msg */
		/* TODO - add messeges filtering */
		msg->ext = 1;
	}

	ret = nmdisp_send_msg(nmcstm->nmdisp, 0, msg);
	if (ret) {
		pr_err("failed to send message\n");
		return ret;
	}

	return 0;
}

int nmcstm_serialize(struct nmcstm *nmcstm, char *buff, u32 size)
{
	size_t				pos = 0;
	u32				poffset;
	struct mv_sys_dma_mem_info	mem_info;
	char				dev_name[100];

	mem_info.name = dev_name;
	mv_sys_dma_mem_get_info(&mem_info);

	json_print_to_buffer(buff, size, 1, "\"custom-info\": {\n");
	json_print_to_buffer(buff, size, 2, "\"lf-master-id\": %u,\n", nmcstm->pf_id);
	json_print_to_buffer(buff, size, 2, "\"max-msg-len\": %u,\n", MAX_MSG_DATA_LEN);

	json_print_to_buffer(buff, size, 2, "\"cmd-queue\": {\n");
	poffset = (phys_addr_t)nmcstm->mng_ctrl.cmd_queue->phy_base_addr - mem_info.paddr;
	json_print_to_buffer(buff, size, 3, "\"base-poffset\": %#x,\n", poffset);
	poffset = (phys_addr_t)nmcstm->mng_ctrl.cmd_queue->cons_phys - mem_info.paddr;
	json_print_to_buffer(buff, size, 3, "\"cons-poffset\": %#x,\n", poffset);
	poffset = (phys_addr_t)nmcstm->mng_ctrl.cmd_queue->prod_phys - mem_info.paddr;
	json_print_to_buffer(buff, size, 3, "\"prod-poffset\": %#x,\n", poffset);
	json_print_to_buffer(buff, size, 3, "\"len\": %u,\n", nmcstm->mng_ctrl.cmd_queue->len);
	json_print_to_buffer(buff, size, 2, "},\n");

	json_print_to_buffer(buff, size, 2, "\"notify-queue\": {\n");
	poffset = (phys_addr_t)nmcstm->mng_ctrl.notify_queue->phy_base_addr - mem_info.paddr;
	json_print_to_buffer(buff, size, 3, "\"base-poffset\": %#x,\n", poffset);
	poffset = (phys_addr_t)nmcstm->mng_ctrl.notify_queue->cons_phys - mem_info.paddr;
	json_print_to_buffer(buff, size, 3, "\"cons-poffset\": %#x,\n", poffset);
	poffset = (phys_addr_t)nmcstm->mng_ctrl.notify_queue->prod_phys - mem_info.paddr;
	json_print_to_buffer(buff, size, 3, "\"prod-poffset\": %#x,\n", poffset);
	json_print_to_buffer(buff, size, 3, "\"len\": %u,\n", nmcstm->mng_ctrl.notify_queue->len);
	json_print_to_buffer(buff, size, 2, "},\n");

	json_print_to_buffer(buff, size, 1, "}\n");

	return pos;
}

