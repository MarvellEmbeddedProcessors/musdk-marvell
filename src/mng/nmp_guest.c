/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#include "std_internal.h"
#include "env/mv_autogen_comp_flags.h"
#ifdef MVCONF_NMP_BUILT
#include "mng/mv_nmp.h"
#endif /* MVCONF_NMP_BUILT */
#include "dev_mng.h"
#include "lf/mng_cmd_desc.h"

#include "nmp_guest.h"


#define NMP_MAX_BUF_STR_LEN	256


static int nmp_guest_wait_for_guest_file(struct nmp_guest *guest)
{
	char	file_name[SER_MAX_FILE_SIZE];
	int	timeout = guest->timeout;
	int	fd, ret;

	/* Map serial file*/
	snprintf(file_name, sizeof(file_name), "%s%s%d", SER_FILE_VAR_DIR, SER_FILE_NAME_PREFIX, guest->id);

	/* wait for guest file to be opened by NMP */
	do {
#ifdef MVCONF_NMP_BUILT
		if (guest->nmp)
			nmp_schedule(guest->nmp, NMP_SCHED_MNG, NULL);
#endif /* MVCONF_NMP_BUILT */

		fd = open(file_name, O_RDWR);
		if (fd > 0) {
			close(fd);
			break;
		}

		udelay(1000);
	} while (fd < 0 && --timeout);

	if (!timeout) {
		pr_err("failed to find regfile %s. timeout exceeded.\n", file_name);
		return -ETIMEDOUT;
	}

#ifdef MVCONF_NMP_BUILT
	/* Make sure that last command response is sent to host. */
	if (guest->nmp)
		nmp_schedule(guest->nmp, NMP_SCHED_MNG, NULL);
#endif /* MVCONF_NMP_BUILT */

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
	guest->cmd_queue.base_addr_phys = (void *)(uintptr_t)(paddr + poffset);
	guest->cmd_queue.base_addr_virt = (struct cmd_desc *)(va + poffset);
	json_buffer_to_input(sec, "cons-poffset", poffset);
	guest->cmd_queue.cons_phys = (void *)(uintptr_t)(paddr + poffset);
	guest->cmd_queue.cons_virt = (u32 *)(va + poffset);
	json_buffer_to_input(sec, "prod-poffset", poffset);
	guest->cmd_queue.prod_phys = (void *)(uintptr_t)(paddr + poffset);
	guest->cmd_queue.prod_virt = (u32 *)(va + poffset);
	json_buffer_to_input(sec, "len", guest->cmd_queue.len);

	sec = strstr(sec, "notify-queue");
	if (!sec) {
		pr_err("cmd-queue section not found\n");
		return -EINVAL;
	}

	json_buffer_to_input(sec, "base-poffset", poffset);
	guest->notify_queue.base_addr_phys = (void *)(uintptr_t)(paddr + poffset);
	guest->notify_queue.base_addr_virt = (struct cmd_desc *)(va + poffset);
	json_buffer_to_input(sec, "cons-poffset", poffset);
	guest->notify_queue.cons_phys = (void *)(uintptr_t)(paddr + poffset);
	guest->notify_queue.cons_virt = (u32 *)(va + poffset);
	json_buffer_to_input(sec, "prod-poffset", poffset);
	guest->notify_queue.prod_phys = (void *)(uintptr_t)(paddr + poffset);
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

static void check_ka_state(struct nmp_guest *guest)
{
	if (!guest->keep_alive_thresh || (guest->keep_alive_counter++ != guest->keep_alive_thresh))
		return;
	guest->keep_alive_counter = 0;

	/* Send Keep Alive notification message */
	nmp_guest_send_ka_msg(guest);
}

static int internal_ev_cb(void *arg, enum nmp_guest_lf_type client, u8 id, u8 code, u16 indx, void *msg, u16 len)
{
	struct nmp_guest *guest = (struct nmp_guest *)arg;
	enum cmd_dest_type client_type = CDT_CUSTOM;

	int ret = 0;

	/* check if msg is the required response */
	if ((client != NMP_GUEST_LF_T_NICPF) || (indx == CMD_IDX_NOTIFICATION))
		goto push_to_shadow_queue;

	/* msg is a response from NICPF */
	if ((code != guest->wait_for_resp.code) ||
	    (indx != guest->wait_for_resp.indx) ||
	    (len < guest->wait_for_resp.resp_len)) {
	    /* something went wrong, this is not the expected response */
		guest->wait_for_resp.got_resp = -1;
		return 0;
	}

	memcpy(guest->wait_for_resp.resp, msg, guest->wait_for_resp.resp_len);
	guest->wait_for_resp.got_resp = 1;
	return 0;

push_to_shadow_queue:
	if (client == NMP_GUEST_LF_T_NICPF)
		client_type = CDT_PF;
	ret = guest_push_msg_to_q(&guest->notify_shadow_queue,
				  client_type,
				  id,
				  code,
				  indx,
				  msg,
				  len,
				  0,
				  guest->notify_shadow_queue.cons_val,
				  &guest->notify_shadow_queue.prod_val);

	return ret;
}

static u16 guest_get_msg_from_q(struct nmp_guest *guest,
				struct nmp_guest_queue *q,
				u32 *cons_idx,
				struct cmd_desc **ret_cmd)
{
	struct cmd_desc *cmd;
	u16 len, total_len = 0;
	u8 num_ext_descs;

	/* Place message */
	cmd = q->base_addr_virt + *cons_idx;

	num_ext_descs = CMD_FLAGS_NUM_EXT_DESC_GET(cmd->flags);
	if (guest->app_cb.guest_ev_cb) {
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
		if (unlikely((*cons_idx + (num_ext_descs + 1)) > q->len)) {
			u8 num_ext_desc_post_wrap = (*cons_idx + num_ext_descs + 1) - q->len;
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
	/* Increment queue consumer */
	*cons_idx = q_inc_idx_val(q, *cons_idx, 1 + num_ext_descs);
	*ret_cmd = cmd;

	return total_len;
}

static int guest_send_msg(struct nmp_guest *guest,
			  enum cmd_dest_type client_type,
			  u8 client_id,
			  u8 code,
			  u16 indx,
			  void *msg,
			  u16 len,
			  int resp_required)
{
	u32 cons_idx, prod_idx;
	struct nmp_guest_queue *q = &guest->cmd_queue;
	int ret;

	cons_idx = q_rd_cons(q);
	prod_idx = q_rd_prod(q);

	/* Memory barrier */
	rmb();

	ret = guest_push_msg_to_q(&guest->cmd_queue,
				  client_type,
				  client_id,
				  code,
				  indx,
				  msg,
				  len,
				  resp_required,
				  cons_idx,
				  &prod_idx);
	if (ret)
		return ret;

	/* Memory barrier */
	wmb();

	/* Increment queue producer */
	q_wr_prod(q, prod_idx);

	return 0;
}

static int skip_str_relation_info(char *prb_str)
{
	char	*lbuff, *sec = NULL;
	char	*tmp_lb, *tmp_rb;
	int	 br_cnt = 0;

	lbuff = kcalloc(1, SER_MAX_FILE_SIZE, GFP_KERNEL);
	if (lbuff == NULL)
		return -ENOMEM;

	memcpy(lbuff, prb_str, SER_MAX_FILE_SIZE);

	sec = strstr(lbuff, "relations-info");
	if (!sec) {
		pr_err("'relations-info' not found\n");
		kfree(lbuff);
		return -EINVAL;
	}

	tmp_lb = strstr(sec, "{");
	tmp_rb = strstr(sec, "}");
	if (tmp_rb <= tmp_lb) {
		pr_err("Invalid probe-string!\n");
		return -EFAULT;
	}
	sec = tmp_lb + 1; /* next search will be from the left-brace */
	br_cnt++;
	do {
		tmp_lb = strstr(sec, "{");
		tmp_rb = strstr(sec, "}");
		if (tmp_rb < tmp_lb) {
			sec = tmp_rb + 1; /* next search will be from the right-brace */
			br_cnt--;
		} else {
			sec = tmp_lb + 1; /* next search will be from the left-brace */
			br_cnt++;
		}
	} while (br_cnt > 1);

	return sec - lbuff;
}


int guest_push_msg_to_q(struct nmp_guest_queue *q,
			enum cmd_dest_type client_type,
			u8 client_id,
			u8 code,
			u16 indx,
			void *msg,
			u16 len,
			int resp_required,
			u32 cons_idx,
			u32 *prod_idx)
{
	struct cmd_desc *cmd;
	u8 num_ext_descs, free_descs;

	/* Check for free space */
	if (q_full(q, *prod_idx, cons_idx))
		return -ENOSPC;

	/* 'num_ext_descs' represent the number of descriptors that are used as pure data.
	 * To calculate it we need to subtract the size of data portion of the first descriptor and to divide it by
	 * the size of command-descriptor.
	 * for more details please refer to the A8K_NMP_Descriptor_Format.xls
	 */
	num_ext_descs = ceil((len - MGMT_DESC_DATA_LEN), sizeof(struct cmd_desc));

	/* Calculate number of free descriptors */
	free_descs = q_space(q, *prod_idx, cons_idx);
	if (free_descs < (1 + num_ext_descs)) {
		pr_err("Not enogth free descriptors (%u) to hold the message length (%u).\n"
			, free_descs, len);
		return -ENOSPC;
	}

	/* Place message */
	cmd = q->base_addr_virt + *prod_idx;
/*	cmd->app_code		=  */
	cmd->cmd_code		= code;
	cmd->cmd_idx		= indx;
	cmd->client_type	= client_type;
	cmd->client_id		= client_id;
	cmd->flags		= 0;
	CMD_FLAGS_NO_RESP_SET(cmd->flags, !resp_required);
	CMD_FLAGS_NUM_EXT_DESC_SET(cmd->flags, num_ext_descs);

	/* In case there is a wrap around the descriptors are be stored to the
	 * end of the ring AND from the beginning of the desc ring.
	 * So in order to keep the non-wrap code the same, we first copy the descs from the beginning of
	 * the ring.
	 */
	if (unlikely((*prod_idx + (num_ext_descs + 1)) > q->len)) {
		u8 num_ext_desc_post_wrap = (*prod_idx + num_ext_descs + 1) - q->len;
		u16 len_pre_wrap = (num_ext_descs - num_ext_desc_post_wrap) * sizeof(struct cmd_desc) +
				   MGMT_DESC_DATA_LEN;
		u16 len_post_wrap = len - len_pre_wrap;

		/* Update len "pre wrap" */
		len = len_pre_wrap;
		/* copy post wrap part */
		memcpy(q->base_addr_virt, &((u8 *)msg)[len], len_post_wrap);
	}

	memcpy(cmd->data, msg, len);

	*prod_idx = q_inc_idx_val(q, *prod_idx, (1 + num_ext_descs));

	return 0;
}

int send_internal_msg(struct nmp_guest *guest,
		      u8 code,
		      u16 indx,
		      void *msg,
		      u16 len,
		      void *resp,
		      u16 resp_len)
{
	int ret, resp_req = resp ? 1 : 0;

	ret = guest_send_msg(guest, CDT_PF, guest->lf_master_id, code, indx, msg, len, resp_req);
	if (ret) {
		/* TODO - specific handling??? */
		return ret;
	}

	if (!resp || !resp_len)
		return 0;

	/* Sequence :
	 * replace CB with private version
	 * change mode of scheduling
	 * call nmp_guest_schedule till required response arrive
	 * on CB, save all "not-to-me" msg in shadow-q. on resp mark complete
	 * revert CB and scheduling mode
	 * copy response and return
	 */

	guest->internal_cb.arg = guest->app_cb.arg;
	guest->internal_cb.cb = guest->app_cb.guest_ev_cb;
	guest->app_cb.arg = guest;
	guest->app_cb.guest_ev_cb = internal_ev_cb;
	guest->internal_schedule = 1;
	guest->wait_for_resp.code = code;
	guest->wait_for_resp.indx = indx;
	guest->wait_for_resp.resp = resp;
	guest->wait_for_resp.resp_len = resp_len;
	guest->wait_for_resp.got_resp = 0;
	do {
		nmp_guest_schedule(guest);
	} while (guest->wait_for_resp.got_resp == 0); /* TODO - add timeout?? */

	ret = (guest->wait_for_resp.got_resp > 0) ? 0 : guest->wait_for_resp.got_resp;

	guest->app_cb.arg = guest->internal_cb.arg;
	guest->app_cb.guest_ev_cb = guest->internal_cb.cb;
	guest->internal_schedule = 0;

	return ret;
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
	(*g)->nmp = params->nmp;

	err = nmp_guest_wait_for_guest_file(*g);
	if (err) {
		pr_err("Guest file not available\n");
		goto guest_init_err2;
	}

	nmp_guest_probe(*g);

	(*g)->msg = kcalloc(1, (*g)->max_msg_len, GFP_KERNEL);
	if ((*g)->msg == NULL) {
		err = -ENOMEM;
		pr_err("Failed to allocate message buffer\n");
		goto guest_init_err2;
	}

	(*g)->notify_shadow_queue.len = (*g)->notify_queue.len;
	(*g)->notify_shadow_queue.base_addr_virt =
		kcalloc(1, (*g)->notify_shadow_queue.len * sizeof(struct cmd_desc), GFP_KERNEL);
	if ((*g)->notify_shadow_queue.base_addr_virt == NULL) {
		err = -ENOMEM;
		pr_err("Failed to allocate shadow queue\n");
		goto guest_init_err3;
	}
	(*g)->notify_shadow_queue.prod_val = 0;
	(*g)->notify_shadow_queue.cons_val = 0;

	pr_debug("%s...done\n", __func__);

	return 0;

guest_init_err3:
	kfree((*g)->msg);
guest_init_err2:
	kfree((*g)->prb_str);
guest_init_err1:
	kfree((*g));
	return err;

}

void nmp_guest_deinit(struct nmp_guest *guest)
{
	kfree(guest->notify_shadow_queue.base_addr_virt);
	kfree(guest->msg);
	kfree(guest->prb_str);
	kfree(guest);
	pr_debug("%s...done\n", __func__);
}

int nmp_guest_get_probe_str(struct nmp_guest *guest, char **prb_str)
{
	int skip = skip_str_relation_info(guest->prb_str);

	if (skip < 0) {
		pr_err("error in probe-string!\n");
		return -EFAULT;
	}

	/* We provide the user the probe-string without the relation-information */
	*prb_str = guest->prb_str + skip;
	pr_debug("prb_str: %s\n", *prb_str);

	return 0;
}

int nmp_guest_get_relations_info(struct nmp_guest *guest, struct nmp_guest_info *guest_info)
{
	u32	 i, j;
	char	*sec = NULL;
	int	 rc;
	char	*lbuff;
	char	 tmp_buf[NMP_MAX_BUF_STR_LEN];
	struct nmp_guest_port_info *giu_info = &guest_info->giu_info;
	struct nmp_guest_module_info *pp2_info = &guest_info->ports_info;

	lbuff = kcalloc(1, SER_MAX_FILE_SIZE, GFP_KERNEL);
	if (lbuff == NULL)
		return -ENOMEM;

	memcpy(lbuff, guest->prb_str, SER_MAX_FILE_SIZE);

	sec = strstr(lbuff, "relations-info");
	if (!sec) {
		pr_err("'relations-info' not found\n");
		rc = -EINVAL;
		goto rel_info_exit1;
	}

	memset(tmp_buf, 0, sizeof(tmp_buf));
	snprintf(tmp_buf, sizeof(tmp_buf), "giu-gpio");
	json_buffer_to_input_str(sec, tmp_buf, giu_info->port_name);
	pr_debug("giu-port: gpio_name %s\n", giu_info->port_name);

	json_buffer_to_input(sec, "num_bpools", giu_info->num_bpools);
	pr_debug("giu-port: num_pools %d\n", giu_info->num_bpools);

	giu_info->bpool_info = kcalloc(1, sizeof(struct nmp_guest_bpool_info) *
						    giu_info->num_bpools, GFP_KERNEL);
	if (giu_info->bpool_info == NULL) {
		rc = -ENOMEM;
		goto rel_info_exit1;
	}
	for (j = 0; j < giu_info->num_bpools; j++) {
		memset(tmp_buf, 0, sizeof(tmp_buf));
		snprintf(tmp_buf, sizeof(tmp_buf), "giu-bpool-%d", j);
		json_buffer_to_input_str(sec, tmp_buf, giu_info->bpool_info[j].bpool_name);
		pr_debug("giu-port: pool name %s\n", giu_info->bpool_info[j].bpool_name);
	}

	json_buffer_to_input(sec, "num_pp2_ports", pp2_info->num_ports);
	pr_debug("num_ports: %d\n", pp2_info->num_ports);

	if (pp2_info->num_ports == 0) {
		kfree(lbuff);
		return 0;
	}

	pp2_info->port_info = kcalloc(1, sizeof(struct nmp_guest_port_info) * pp2_info->num_ports, GFP_KERNEL);
	if (pp2_info->port_info == NULL) {
		rc = -ENOMEM;
		goto rel_info_exit2;
	}

	for (i = 0; i < pp2_info->num_ports; i++) {
		memset(tmp_buf, 0, sizeof(tmp_buf));
		snprintf(tmp_buf, sizeof(tmp_buf), "ppio-%d", i);
		json_buffer_to_input_str(sec, tmp_buf, pp2_info->port_info[i].port_name);
		pr_debug("port: %d, ppio_name %s\n", i, pp2_info->port_info[i].port_name);

		json_buffer_to_input(sec, "num_pp2_bpools", pp2_info->port_info[i].num_bpools);
		pr_debug("port: %d, num_pools %d\n", i, pp2_info->port_info[i].num_bpools);

		pp2_info->port_info[i].bpool_info = kcalloc(1, sizeof(struct nmp_guest_bpool_info) *
							    pp2_info->port_info[i].num_bpools, GFP_KERNEL);
		if (pp2_info->port_info[i].bpool_info == NULL) {
			rc = -ENOMEM;
			goto rel_info_exit3;
		}
		for (j = 0; j < pp2_info->port_info[i].num_bpools; j++) {
			memset(tmp_buf, 0, sizeof(tmp_buf));
			snprintf(tmp_buf, sizeof(tmp_buf), "bpool-%d", j);
			json_buffer_to_input_str(sec, tmp_buf, pp2_info->port_info[i].bpool_info[j].bpool_name);
			pr_debug("port: %d, pool name %s\n", i, pp2_info->port_info[i].bpool_info[j].bpool_name);
		}
	}
	kfree(lbuff);
	return 0;

rel_info_exit3:
	kfree(pp2_info->port_info);
rel_info_exit2:
	kfree(giu_info->bpool_info);
rel_info_exit1:
	kfree(lbuff);
	return rc;
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
	struct cmd_desc *cmd = NULL;
	u16 total_len = 0;
	u32 prod_idx, cons_idx;
	enum nmp_guest_lf_type lf_type = NMP_GUEST_LF_T_CUSTOM;

	check_ka_state(guest);

#ifdef MVCONF_NMP_BUILT
	if (guest->nmp)
		nmp_schedule(guest->nmp, NMP_SCHED_MNG, NULL);
#endif /* MVCONF_NMP_BUILT */

	prod_idx = q_rd_prod((&guest->notify_queue));
	cons_idx = q_rd_cons((&guest->notify_queue));

	/* Memory barrier */
	rmb();

	while (1) {
		if ((!guest->internal_schedule) &&
		    (!q_empty(guest->notify_shadow_queue.prod_val, guest->notify_shadow_queue.cons_val))) {
			/* First check shadow notify queue */
			total_len = guest_get_msg_from_q(guest,
							 &guest->notify_shadow_queue,
							 &guest->notify_shadow_queue.cons_val,
							 &cmd);
		} else {

			/* Check for pending message */
			if (q_empty(prod_idx, cons_idx))
				return 0;

			total_len = guest_get_msg_from_q(guest, &guest->notify_queue, &cons_idx, &cmd);

			/* make sure all writes are done (i.e. descriptor were copied)
			 * before incrementing the consumer index
			 */
			/* Memory barrier */
			wmb();

			q_wr_cons((&guest->notify_queue), cons_idx);
		}

		if (cmd->client_type == CDT_PF)
			lf_type = NMP_GUEST_LF_T_NICPF;

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

#ifdef MVCONF_NMP_BUILT
	if (guest->nmp)
		nmp_schedule(guest->nmp, NMP_SCHED_MNG, NULL);
#endif /* MVCONF_NMP_BUILT */

	return 0;
}

int nmp_guest_send_msg(struct nmp_guest *guest, u8 code, u16 indx, void *msg, u16 len)
{
	return guest_send_msg(guest, CDT_CUSTOM, guest->id, code, indx, msg, len, 0);
}

int nmp_guest_send_ka_msg(struct nmp_guest *guest)
{
	return send_internal_msg(guest, MSG_F_GUEST_KA, 0, NULL, 0, NULL, 0);
}

#ifdef MVCONF_NMP_BUILT
void nmp_guest_set_nmp(struct nmp_guest *guest, void *nmp)
{
	guest->nmp = nmp;
}
#endif /* MVCONF_NMP_BUILT */

