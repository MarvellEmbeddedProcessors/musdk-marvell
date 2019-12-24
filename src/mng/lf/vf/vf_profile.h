/******************************************************************************
*  Copyright (C) 2019 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#ifndef _VF_PROFILE_H
#define _VF_PROFILE_H

#include "std_internal.h"
#include "mng/mv_nmp.h"

#define PROFILE_NAME_LEN (32)
#define PROFILE_DESC_LEN (128)
#define NMP_MAX_BUF_STR_LEN		256


/* SNIC VF / VF Instance profile
 *
 * name               - profile name
 * desc               - profile name
 * match              - matching gpio name
 * pci_en             -	flag inidicating PCI interface is present
 * lcl_egress_q_num   - number of local egress data queues
 * lcl_egress_q_size  - size of local egress data queue
 * lcl_ingress_q_num  - number of local ingress data queues
 * lcl_ingress_q_size - size of local ingress data queues
 * lcl_bp_num       - number of local bm queues
 * lcl_bm_q_size      - size of bm queue
 */
struct vf_profile {
	char name[PROFILE_NAME_LEN];
	char desc[PROFILE_DESC_LEN];

	char match[NMP_MAX_BUF_STR_LEN];
	int pci_en;
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
	u16 dflt_pkt_offset;
	u8 max_num_tcs;
};

#endif /* _VF_PROFILE_H */

