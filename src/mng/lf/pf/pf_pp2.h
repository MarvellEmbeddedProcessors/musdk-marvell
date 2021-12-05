/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PF_PP2_H
#define _PF_PP2_H

#include "std_internal.h"
#include "pf_topology.h"

#define INITIAL_MAC_ADDR	{0x0, 0x1, 0x2, 0x3, 0x4, 0x5}

int nmnicpf_pp2_port_pp2_init(struct nmnicpf *nmnicpf);
int nmnicpf_pp2_port_init(struct nmnicpf *nmnicpf);
int nmnicpf_pp2_init_bpools(struct nmnicpf *nmnicpf);
int nmnicpf_pp2_init_ppio(struct nmnicpf *nmnicpf);
int nmnicpf_pp2_get_statistics(struct nmnicpf *nmnicpf,
			       struct mgmt_cmd_params *params,
			       struct mgmt_cmd_resp *resp_data);
int nmnicpf_pp2_accumulate_statistics(struct nmnicpf *nmnicpf,
				      struct pp2_ppio_statistics *stats,
				      int    reset);
void nmnicpf_pp2_get_mac_addr(struct nmnicpf *nmnicpf, u8 *mac_addr);

int nmnicpf_pp2_cls_table_init(struct nmnicpf *nmnicpf,
			       void *msg,
			       u16 msg_len,
			       struct guest_pp2_cls_cmd_resp *resp);
int nmnicpf_pp2_cls_table_deinit(struct nmnicpf *nmnicpf, void *msg, u16 msg_len);
int nmnicpf_pp2_cls_rule_add(struct nmnicpf *nmnicpf, void *msg, u16 msg_len);
int nmnicpf_pp2_cls_rule_modify(struct nmnicpf *nmnicpf, void *msg, u16 msg_len);
int nmnicpf_pp2_cls_rule_remove(struct nmnicpf *nmnicpf, void *msg, u16 msg_len);

/**
 * Serialize the nmnicpf parameters
 *
 * @param[in]	nmnicpf		A nmnicpf handle.
 * @param[in]	buff		Pointer to a buffer where the resulting string is stored.
 *				The buffer should have a size of at least 'size' characters.
 * @param[in]	size		size of buffer.
 *
 * @retval	The number of characters that would have been written if 'size' had been sufficiently large
 * @retval	<0 on failure
 */
int nmnicpf_pp2_serialize_relation_inf(struct nmnicpf *nmnicpf, char *buff, u32 size, u8 depth);
int nmnicpf_pp2_serialize(struct nmnicpf *nmnicpf, char *buff, u32 size);

#endif /* _PF_PP2_H */
