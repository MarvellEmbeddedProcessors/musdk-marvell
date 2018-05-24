/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#ifndef _PF_PROFILE_H
#define _PF_PROFILE_H

#define PROFILE_NAME_LEN (32)
#define PROFILE_DESC_LEN (128)
#define NMP_MAX_BUF_STR_LEN		256

#include "mng/mv_nmp.h"


/* SNIC PF / VF Instance profile
 *
 * name               - profile name
 * desc               - profile name
 * match              - matching gpio name
 * pci_en             -	flag inidicating PCI interface is present
 * lcl_egress_q_num   - number of local egress data queues
 * lcl_egress_q_size  - size of local egress data queue
 * lcl_ingress_q_num  - number of local ingress data queues
 * lcl_ingress_q_size - size of local ingress data queues
 * lcl_bm_q_num       - number of local bm queues
 * lcl_bm_q_size      - size of bm queue
 */
struct pf_profile {
	char name[PROFILE_NAME_LEN];
	char desc[PROFILE_DESC_LEN];

	char match[NMP_MAX_BUF_STR_LEN];
	int pci_en;
	u32 lcl_egress_q_num;
	u32 lcl_egress_q_size;
	u32 lcl_ingress_q_num;
	u32 lcl_ingress_q_size;
	u32 lcl_bm_q_num;
	u32 lcl_bm_q_size;
	u32 lcl_bm_buf_size;
	/* additions (including pp2)*/
	u16 dflt_pkt_offset;
	u8 max_num_tcs;
	u16 pp2_bm_pool_reserved_map;
	enum nmp_lf_nicpf_type port_type;
	struct nmp_lf_nicpf_pp2_port_params pp2_port;
};

#endif /* _PF_PROFILE_H */

