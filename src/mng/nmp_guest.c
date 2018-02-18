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
	char				*sec = NULL;
	struct sys_iomem_params		 iomem_params;
	struct sys_iomem		*sys_iomem;
	char				 dev_name[FILE_MAX_LINE_CHARS];
	uintptr_t			 va;
	phys_addr_t			 paddr;
	size_t				 reg_size = 0;
	u32				 poffset = 0;
	char				*tmp_buff;

	tmp_buff = kcalloc(1, strlen(guest->prb_str), GFP_KERNEL);
	if (tmp_buff == NULL)
		return -ENOMEM;

	memcpy(tmp_buff, guest->prb_str, strlen(guest->prb_str));

	sec = strstr(tmp_buff, "dma-info");
	if (!sec) {
		pr_err("'dma-info' not found\n");
		return -EINVAL;
	}

	memset(dev_name, 0, FILE_MAX_LINE_CHARS);
	/* get the master DMA device name */
	json_buffer_to_input_str(sec, "file_name", dev_name);
	if (dev_name[0] == 0) {
		pr_err("'file_name' not found\n");
		return -EINVAL;
	}

	/* get the master DMA region size */
	json_buffer_to_input(sec, "region_size", reg_size);
	if (reg_size == 0) {
		pr_err("reg_size is 0\n");
		return -EINVAL;
	}

	/* get the master DMA physical address */
	json_buffer_to_input(sec, "phys_addr", paddr);
	if (!paddr) {
		pr_err("'phys_addr' not found\n");
		return -EINVAL;
	}

	iomem_params.type = SYS_IOMEM_T_SHMEM;
	iomem_params.devname = dev_name;
	iomem_params.index = 1;
	iomem_params.size = reg_size;

	if (sys_iomem_init(&iomem_params, &sys_iomem)) {
		pr_err("sys_iomem_init error\n");
		return -EINVAL;
	}

	/* Map the iomem physical address */
	if (sys_iomem_map(sys_iomem, NULL, (phys_addr_t *)&paddr,
			  (void **)&va)) {
		pr_err("sys_iomem_map error\n");
		sys_iomem_deinit(sys_iomem);
		return -EINVAL;
	}

	/* Search for the custom-info section */
	sec = strstr(sec, "custom-info");
	if (!sec) {
		pr_err("custom-info section not found\n");
		return -EINVAL;
	}

	json_buffer_to_input(sec, "lf-master-id", guest->lf_master_id);
	json_buffer_to_input(sec, "max-msg-len", guest->max_msg_len);

	sec = strstr(sec, "cmd-queue");
	if (!sec) {
		pr_err("cmd-queue section not found\n");
		return -EINVAL;
	}

	json_buffer_to_input(sec, "base-poffset", poffset);
	guest->cmd_queue.base_addr_phys = (void *)(paddr + poffset);
	guest->cmd_queue.base_addr_virt = (struct cmd_desc *)(va + poffset);
	json_buffer_to_input(sec, "cons-poffset", poffset);
	guest->cmd_queue.cons_phys = (void *)(paddr + poffset);
	guest->cmd_queue.cons_virt = (u32 *)(va + poffset);
	json_buffer_to_input(sec, "prod-poffset", poffset);
	guest->cmd_queue.prod_phys = (void *)(paddr + poffset);
	guest->cmd_queue.prod_virt = (u32 *)(va + poffset);
	json_buffer_to_input(sec, "len", guest->cmd_queue.len);

	sec = strstr(sec, "notify-queue");
	if (!sec) {
		pr_err("cmd-queue section not found\n");
		return -EINVAL;
	}

	json_buffer_to_input(sec, "base-poffset", poffset);
	guest->notify_queue.base_addr_phys = (void *)(paddr + poffset);
	guest->notify_queue.base_addr_virt = (struct cmd_desc *)(va + poffset);
	json_buffer_to_input(sec, "cons-poffset", poffset);
	guest->notify_queue.cons_phys = (void *)(paddr + poffset);
	guest->notify_queue.cons_virt = (u32 *)(va + poffset);
	json_buffer_to_input(sec, "prod-poffset", poffset);
	guest->notify_queue.prod_phys = (void *)(paddr + poffset);
	guest->notify_queue.prod_virt = (u32 *)(va + poffset);
	json_buffer_to_input(sec, "len", guest->notify_queue.len);

	pr_debug("NMP-GUEST CMD Queue Params:\n");
	pr_debug("\tdesc_ring_base phys %p\n", guest->cmd_queue.base_addr_phys);
	pr_debug("\tdesc_ring_base virt %p\n", guest->cmd_queue.base_addr_virt);
	pr_debug("\tcons_addr phys %p\n", guest->cmd_queue.cons_phys);
	pr_debug("\tcons_addr phys %p\n", guest->cmd_queue.cons_virt);
	pr_debug("\tprod_addr phys %p\n", guest->cmd_queue.prod_phys);
	pr_debug("\tprod_addr virt %p\n", guest->cmd_queue.prod_virt);
	pr_debug("\tlen 0x%x\n", guest->cmd_queue.len);

	pr_debug("NMP-GUEST NOTIFY Queue Params:\n");
	pr_debug("\tdesc_ring_base phys %p\n", guest->notify_queue.base_addr_phys);
	pr_debug("\tdesc_ring_base virt %p\n", guest->notify_queue.base_addr_virt);
	pr_debug("\tcons_addr phys %p\n", guest->notify_queue.cons_phys);
	pr_debug("\tcons_addr phys %p\n", guest->notify_queue.cons_virt);
	pr_debug("\tprod_addr phys %p\n", guest->notify_queue.prod_phys);
	pr_debug("\tprod_addr virt %p\n", guest->notify_queue.prod_virt);
	pr_debug("\tlen 0x%x\n", guest->notify_queue.len);

	return 0;
}

int nmp_guest_init(struct nmp_guest_params *params, struct nmp_guest **g)
{
	int	err;

	*g = kcalloc(1, sizeof(struct nmp_guest), GFP_KERNEL);
	if (*g == NULL) {
		pr_err("Failed to allocate NMP handler\n");
		return -ENOMEM;
	}

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

	nmp_guest_probe(*g);

	/* TODO - take max mesg length from guest-file */
	(*g)->msg = kcalloc(1, (*g)->max_msg_len, GFP_KERNEL);
	if ((*g)->msg == NULL) {
		err = -ENOMEM;
		pr_err("Failed to allocate message buffer\n");
		goto guest_init_err2;
	}


	pr_debug("%s...done\n", __func__);

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
	pr_debug("%s...done\n", __func__);
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
	u16 len, total_len = 0;
	u8 num_ext_descs;
	enum nmp_guest_lf_type lf_type = NMP_GUEST_LF_T_CUSTOM;
	struct nmp_guest_queue *q = &guest->notify_queue;

	while (1) {
		prod_idx = q_rd_prod(q);
		cons_idx = q_rd_cons(q);

		/* Check for pending message */
		if (q_empty(prod_idx, cons_idx))
			return 0;

		/* Place message */
		cmd = q->base_addr_virt + cons_idx;

		num_ext_descs = CMD_FLAGS_NUM_EXT_DESC_GET(cmd->flags);
		if (guest->app_cb.guest_ev_cb) {
			if (cmd->client_type == CDT_PF)
				lf_type = NMP_GUEST_LF_T_NICPF;
			/* if 'num_ext_descs' >= 1 means that first descriptor includes descriptor head + data,
			 * and rest of the descriptors are pure data.
			 * for more details please refer to the A8K_NMP_Descriptor_Format.xls
			 */
			len = sizeof(cmd->data) + sizeof(struct cmd_desc) * num_ext_descs;

			/* In case there is a wrap around the descriptors are be stored to the end of the ring AND
			 * from the beginning of the desc ring.
			 * So in order to keep the non-wrap code the same, we first copy the descs from the beginning of
			 * the ring.
			 */
			if (unlikely((cons_idx + (num_ext_descs + 1)) > q->len)) {
				u8 num_ext_desc_post_wrap = (cons_idx + num_ext_descs + 1) - q->len;
				u16 len_post_wrap = sizeof(struct cmd_desc) * num_ext_desc_post_wrap;

				/* Update len "pre wrap" */
				len -= len_post_wrap;
				/* copy post wrap part */
				memcpy(&((u8 *)guest->msg)[len], q->base_addr_virt, len_post_wrap);
				total_len += len_post_wrap;
			}

			memcpy(guest->msg, cmd->data, len);
			total_len += len;
		}

		/* make sure all writes are done (i.e. descriptor were copied)
		 * before incrementing the consumer index
		 */
		wmb();

		/* Increament queue consumer */
		q_wr_cons(q, q_inc_idx_val(q, cons_idx, 1 + num_ext_descs));

		/* The filtering should be done on 'custom' part.
		 * As currently only one CB is supported there is no need for searching the correct CB.
		 */
		if (guest->app_cb.guest_ev_cb) {
			guest->app_cb.guest_ev_cb(guest->app_cb.arg,
						  lf_type,
						  cmd->client_id,
						  cmd->cmd_code,
						  cmd->cmd_idx,
						  guest->msg,
						  total_len);
		}
	}

	return 0;
}

int nmp_guest_send_msg(struct nmp_guest *guest, u8 code, u16 indx, void *msg, u16 len)
{
	struct cmd_desc *cmd;
	u32 cons_idx, prod_idx;
	u8 num_ext_descs, free_descs;
	struct nmp_guest_queue *q = &guest->cmd_queue;

	cons_idx = q_rd_cons(q);
	prod_idx = q_rd_prod(q);

	/* Check for free space */
	if (q_full(q, prod_idx, cons_idx))
		return -ENOSPC;

	/* 'num_ext_descs' represent the number of descriptors that are used as pure data.
	 * To calculate it we need to subtract the size of data portion of the first descriptor and to divide it by
	 * the size of command-descriptor.
	 * for more details please refer to the A8K_NMP_Descriptor_Format.xls
	 */
	num_ext_descs = ceil((len - MGMT_DESC_DATA_LEN), sizeof(struct cmd_desc));

	/* Calculate number of free descriptors */
	free_descs = q_space(q, prod_idx, cons_idx);
	if (free_descs < (1 + num_ext_descs)) {
		pr_err("Not enogth free descriptors (%u) to hold the message length (%u).\n"
			, free_descs, len);
		return -ENOSPC;
	}

	/* Place message */
	cmd = q->base_addr_virt + prod_idx;
/*	cmd->app_code		=  */
	cmd->cmd_code		= code;
	cmd->cmd_idx		= indx;
	cmd->client_type	= CDT_CUSTOM;
	cmd->client_id		= guest->id;
	cmd->flags		= 0;
	CMD_FLAGS_NUM_EXT_DESC_SET(cmd->flags, num_ext_descs);

	/* In case there is a wrap around the descriptors are be stored to the
	 * end of the ring AND from the beginning of the desc ring.
	 * So in order to keep the non-wrap code the same, we first copy the descs from the beginning of
	 * the ring.
	 */
	if (unlikely((prod_idx + (num_ext_descs + 1)) > q->len)) {
		u8 num_ext_desc_post_wrap = (prod_idx + num_ext_descs + 1) - q->len;
		u16 len_pre_wrap = (num_ext_descs - num_ext_desc_post_wrap) * sizeof(struct cmd_desc) +
				   MGMT_DESC_DATA_LEN;
		u16 len_post_wrap = len - len_pre_wrap;

		/* Update len "pre wrap" */
		len = len_pre_wrap;
		/* copy post wrap part */
		memcpy(q->base_addr_virt, &((u8 *)msg)[len], len_post_wrap);
	}

	memcpy(cmd->data, msg, len);

	/* Memory barrier */
	wmb();

	/* Increament queue producer */
	q_wr_prod(q, q_inc_idx_val(q, prod_idx, (1 + num_ext_descs)));

	return 0;
}


