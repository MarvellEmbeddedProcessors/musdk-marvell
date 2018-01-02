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

#include "std_internal.h"
#include "mng/mv_nmp.h"
#include "mng/mv_nmp_guest.h"
#include "db.h"
#include "dev_mng.h"
#include "hw_emul/gie.h"
#include "mng/dispatch.h"
#include "config.h"
#include "lib/lib_misc.h"

#include "nmp_guest.h"

static int nmp_guest_wait_for_guest_file(struct nmp_guest *guest)
{
	char	file_name[SER_MAX_FILE_SIZE];
	int	timeout = guest->timeout;
	int	fd, ret;

	/* Map serial file*/
	snprintf(file_name, sizeof(file_name), "%s%s%d", SER_FILE_VAR_DIR, SER_FILE_NAME_PREFIX, guest->id);

	/* wait for guest file to be opened by NMP */
	do {
		fd = open(file_name, O_RDWR);
		if (fd > 0) {
			close(fd);
			break;
		}

		udelay(100);
	} while (fd < 0 && --timeout);

	if (!timeout) {
		pr_err("failed to find regfile %s. timeout exceeded.\n", file_name);
		return -EFAULT;
	}

	ret = read_file_to_buf(file_name, guest->prb_str, SER_MAX_FILE_SIZE);
	if (ret) {
		pr_err("read guest file failed for %s\n", file_name);
		return ret;
	}

	return 0;
}

static int nmp_guest_probe(struct nmp_guest *guest)
{
	guest->cmd_queue.base_addr_virt		= nmcstm_for_guest->mng_ctrl.cmd_queue->virt_base_addr;
	guest->cmd_queue.cons_virt		= nmcstm_for_guest->mng_ctrl.cmd_queue->cons_virt;
	guest->cmd_queue.prod_virt		= nmcstm_for_guest->mng_ctrl.cmd_queue->prod_virt;
	guest->cmd_queue.len			= nmcstm_for_guest->mng_ctrl.cmd_queue->len;
	guest->notify_queue.base_addr_virt	= nmcstm_for_guest->mng_ctrl.notify_queue->virt_base_addr;
	guest->notify_queue.cons_virt		= nmcstm_for_guest->mng_ctrl.notify_queue->cons_virt;
	guest->notify_queue.prod_virt		= nmcstm_for_guest->mng_ctrl.notify_queue->prod_virt;
	guest->notify_queue.len			= nmcstm_for_guest->mng_ctrl.notify_queue->len;

	return 0;
}

int nmp_guest_init(struct nmp_guest_params *params, struct nmp_guest **g)
{
	int	err;
	char	file_name[SER_MAX_FILE_NAME];

	*g = kcalloc(1, sizeof(struct nmp_guest), GFP_KERNEL);
	if (*g == NULL) {
		pr_err("Failed to allocate NMP handler\n");
		return -ENOMEM;
	}

	(*g)->prb_str = kcalloc(1, SER_MAX_FILE_SIZE, GFP_KERNEL);
	if ((*g)->prb_str == NULL)
		return -ENOMEM;

	(*g)->id = params->id;
	(*g)->timeout = params->timeout;
	(*g)->prb_str = kcalloc(1, SER_MAX_FILE_SIZE, GFP_KERNEL);
	if ((*g)->prb_str == NULL) {
		err = -ENOMEM;
		goto guest_init_err1;
	}

	err = nmp_guest_wait_for_guest_file(*g);
	if (err) {
		err = -EINVAL;
		pr_err("Guest file not available\n");
		goto guest_init_err2;
	}

	snprintf(file_name, sizeof(file_name), "%s%s%d", SER_FILE_VAR_DIR, SER_FILE_NAME_PREFIX, (*g)->id);
	err = read_file_to_buf(file_name, (*g)->prb_str, SER_MAX_FILE_SIZE);
	if (err) {
		pr_err("app_read_config_file failed for %s\n", file_name);
		goto guest_init_err2;
	}

	/* TODO - take max mesg length from guest-file */
	(*g)->msg = kcalloc(1, 1024, GFP_KERNEL);
	if ((*g)->msg == NULL) {
		pr_err("Failed to allocate message buffer\n");
		kfree((*g)->prb_str);
		kfree(*g);
		return -ENOMEM;
	}

	nmp_guest_probe(*g);

	pr_info("%s...done\n", __func__);

	return 0;

guest_init_err2:
	kfree((*g)->prb_str);
guest_init_err1:
	kfree((*g));
	return err;

}

void nmp_guest_deinit(struct nmp_guest *guest)
{
	kfree(guest->prb_str);
	kfree(guest);
	pr_info("%s...done\n", __func__);
}

int nmp_guest_get_probe_str(struct nmp_guest *guest, char **prb_str)
{
	*prb_str = guest->prb_str;
	pr_debug("prb_str: %s\n", *prb_str);
	return 0;
}

int nmp_guest_register_event_handler(struct nmp_guest *guest,
				     enum nmp_guest_lf_type lf_type,
				     u8 lf_id,
				     u64 ev_mask,
				     void *arg,
				     int (*guest_ev_cb)(void *arg, enum nmp_guest_lf_type client, u8 id, u8 code,
							u16 indx, void *msg, u16 len))
{
	if (guest->app_cb.guest_ev_cb) {
		pr_err("only one registration is allowed\n");
		return -1;
	}

	guest->app_cb.lf_type = lf_type;
	guest->app_cb.lf_id = lf_id;
	guest->app_cb.ev_mask = ev_mask;
	guest->app_cb.arg = arg;
	guest->app_cb.guest_ev_cb = guest_ev_cb;

	return 0;
}

int nmp_guest_schedule(struct nmp_guest *guest)
{
	struct cmd_desc *cmd;
	u32 prod_idx, cons_idx;
	u16 len;
	enum nmp_guest_lf_type lf_type = NMP_GUEST_LF_T_NONE;
	struct nmp_guest_queue *q = &guest->notify_queue;

	while (1) {
		prod_idx = q_rd_prod(q);
		cons_idx = q_rd_cons(q);

		/* Check for pending message */
		if (q_empty(prod_idx, cons_idx))
			return 0;

		/* Place message */
		cmd = q->base_addr_virt + cons_idx;

		if (guest->app_cb.guest_ev_cb) {
			if (cmd->dest_type == CDT_PF)
				lf_type = NMP_GUEST_LF_T_NICPF;
			len = sizeof(struct mgmt_cmd_params);
			memcpy(guest->msg, &cmd->params, len);
		}

		/* make sure all writes are done (i.e. descriptor were copied)
		 * before incrementing the consumer index
		 */
		wmb();

		/* Increament queue consumer */
		q_wr_cons(q, q_inc_idx(q, cons_idx));

		/* The filtering should be done on 'custom' part.
		 * As currently only one CB is supported there is no need for searching the correct CB.
		 */
		if (guest->app_cb.guest_ev_cb) {
			guest->app_cb.guest_ev_cb(guest->app_cb.arg,
						  lf_type,
						  cmd->dest_id,
						  cmd->cmd_code,
						  cmd->cmd_idx,
						  guest->msg,
						  len);
		}
	}

	return 0;
}

int nmp_guest_send_msg(struct nmp_guest *guest, u8 code, u16 indx, void *msg, u16 len)
{
	struct cmd_desc *cmd;
	u32 cons_idx, prod_idx;
	struct nmp_guest_queue *q = &guest->cmd_queue;

	/* TODO - remove once S/G supported */
	if (len > sizeof(struct mgmt_cmd_params)) {
		pr_err("message length is bigger than command body. Currently S/G not supported\n");
		return -1;
	}

	cons_idx = q_rd_cons(q);
	prod_idx = q_rd_prod(q);

	/* Check for free space */
	if (q_full(q, prod_idx, cons_idx))
		return -ENOSPC;

	/* Place message */
	cmd = q->base_addr_virt + prod_idx;
/*	cmd->app_code	=  */
	cmd->cmd_code	= code;
	cmd->cmd_idx	= indx;
	cmd->dest_type	= CDT_CUSTOM;
	cmd->dest_id	= guest->id;
	cmd->flags	= 0;
	memcpy(&cmd->params, msg, len);

	/* Memory barrier */
	wmb();

	/* Increament queue producer */
	q_wr_prod(q, q_inc_idx(q, prod_idx));

	return 0;
}


