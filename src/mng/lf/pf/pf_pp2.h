/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#ifndef _PF_PP2_H
#define _PF_PP2_H

#define INITIAL_MAC_ADDR	{0x0, 0x1, 0x2, 0x3, 0x4, 0x5}

int nmnicpf_pp2_port_pp2_init(struct nmnicpf *nmnicpf);
int nmnicpf_pp2_port_init(struct nmnicpf *nmnicpf);
int nmnicpf_pp2_init_bpools(struct nmnicpf *nmnicpf);
int nmnicpf_pp2_init_ppio(struct nmnicpf *nmnicpf);
int nmnicpf_pp2_get_statistics(struct nmnicpf *nmnicpf,
			       struct mgmt_cmd_params *params,
			       struct mgmt_cmd_resp *resp_data);
void nmnicpf_pp2_get_mac_addr(struct nmnicpf *nmnicpf, u8 *mac_addr);

int nmnicpf_pp2_cls_table_init(struct nmnicpf *nmnicpf,
			       void *msg,
			       u16 msg_len,
			       struct guest_pp2_cls_cmd_resp *resp);
int nmnicpf_pp2_cls_table_deinit(struct nmnicpf *nmnicpf, void *msg, u16 msg_len);
int nmnicpf_pp2_cls_rule_add(struct nmnicpf *nmnicpf, void *msg, u16 msg_len);
int nmnicpf_pp2_cls_rule_modify(struct nmnicpf *nmnicpf, void *msg, u16 msg_len);
int nmnicpf_pp2_cls_rule_remove(struct nmnicpf *nmnicpf, void *msg, u16 msg_len);

#endif /* _PF_PP2_H */
