/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PF_PROFILE_H
#define _PF_PROFILE_H

#include "std_internal.h"
#include "mng/mv_nmp.h"

#define PROFILE_NAME_LEN (32)
#define PROFILE_DESC_LEN (128)
#define NMP_MAX_BUF_STR_LEN		256


/* SNIC PF / VF Instance profile
 *
 * name               - profile name
 * desc               - profile name
 * match              - matching gpio name
 * pci_en             -	flag inidicating PCI interface is present
 * sg_en	      - flag inidicating S/G support is required
 * lcl_egress_q_num   - number of local egress data queues
 * lcl_egress_q_size  - size of local egress data queue
 * lcl_ingress_q_num  - number of local ingress data queues
 * lcl_ingress_q_size - size of local ingress data queues
 * lcl_bp_num       - number of local bm queues
 * lcl_bm_q_size      - size of bm queue
 */
struct pf_profile {
	char name[PROFILE_NAME_LEN];
	char desc[PROFILE_DESC_LEN];

	char match[NMP_MAX_BUF_STR_LEN];
	int pci_en;
	int sg_en;
	u32 keep_alive_thresh;
	u32 keep_alive_counter;
	int guest_ka_recv;
	u32 lcl_egress_q_num;
	u32 lcl_egress_q_size;
	u32 lcl_ingress_q_num;
	u32 lcl_ingress_q_size;
	u32 lcl_bp_num;
	struct {
		u32 lcl_bp_size;
		u32 lcl_bp_buf_size;
	} lcl_bp_params[GIU_GPIO_TC_MAX_NUM_BPOOLS];
	/* additions (including pp2)*/
	u16 dflt_pkt_offset;
	u8 max_num_tcs;
	u16 pp2_bm_pool_reserved_map;
	enum nmp_lf_nicpf_type port_type;
	char port_match[NMP_MAX_BUF_STR_LEN];
	struct nmp_lf_nicpf_pp2_port_params pp2_port;
};

#endif /* _PF_PROFILE_H */

