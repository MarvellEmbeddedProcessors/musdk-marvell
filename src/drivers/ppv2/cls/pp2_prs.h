/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file pp2_cls_prs.h
 *
 * internal and external definitions for parser High level routines
 */

#ifndef _PP2_CLS_PRS_H_
#define _PP2_CLS_PRS_H_

#include "pp2_cls_types.h"
#include "pp2_cls_internal_types.h"
#include "pp2_cls_common.h"
#include "pp2_cls_utils.h"
#include "pp2_flow_rules.h"
#include "pp2_c3.h"
#include "pp2_c2.h"
#include "pp2_cls_db.h"

/********************************************************************************/
/*			MACROS							*/
/********************************************************************************/

/* HW_BYTE_OFFS
 * return HW byte offset in 4 bytes register
 * _offs_: native offset (LE)
 * LE example: HW_BYTE_OFFS(1) = 1
 * BE example: HW_BYTE_OFFS(1) = 2
 */

#if defined(__LITTLE_ENDIAN)
#define HW_BYTE_OFFS(_offs_) (_offs_)
#else
#define HW_BYTE_OFFS(_offs_) ((3 - ((_offs_) % 4)) + (((_offs_) / 4) * 4))
#endif

#define SRAM_BIT_TO_BYTE(_bit_) HW_BYTE_OFFS((_bit_) / 8)
#define SRAM_BIT_TO_WORD(_bit_) HW_BYTE_OFFS((_bit_) / 32)
#define SRAM_BIT_IN_WORD(_bit_) HW_BYTE_OFFS((_bit_) % 32)

#define TCAM_DATA_BYTE_OFFS_LE(_offs_)		(((_offs_) - \
	((_offs_) % 2)) * 2 + ((_offs_) % 2))
#define TCAM_DATA_MASK_OFFS_LE(_offs_) (((_offs_) * 2) - ((_offs_) % 2)  + 2)

/* TCAM_DATA_BYTE/MASK
 * tcam data divide into 4 bytes registers
 * each register include 2 bytes of data and 2 bytes of mask
 * the next macros calc data/mask offset in 4 bytes register
 * _offs_: native offset (LE) in data bytes array
 * relevant only for TCAM data bytes
 * used by PRS and CLS2
 */
#define TCAM_DATA_BYTE(_offs_) (HW_BYTE_OFFS(TCAM_DATA_BYTE_OFFS_LE(_offs_)))
#define TCAM_DATA_MASK(_offs_) (HW_BYTE_OFFS(TCAM_DATA_MASK_OFFS_LE(_offs_)))

#define MAX_LOOKUP	3
#define MAX_PROTO_NUM	3

/********************************************************************************/
/*			ENUMERATIONS						*/
/********************************************************************************/

/********************************************************************************/
/*			STRUCTURES						*/
/********************************************************************************/

struct mv_pp2x_iphdr {
	uint8_t	 ihl:4,
		 version:4;
	u8	 tos;
	u16 tot_len;
	u16 id;
	u16 frag_off;
	u8	 ttl;
	u8	 protocol;
	u16 check;
	u32 saddr;
	u32 daddr;
};

struct mv_pp2x_ipv6hdr {
	uint8_t	 priority:4,
		 version:4;
	u8	 flow_lbl[3];

	u16 payload_len;
	u8	 nexthdr;
	u8	 hop_limit;

	struct	 in6_addr saddr;
	struct	 in6_addr daddr;
};

/* Parser definition array */
struct pp2_prs_dynamic {
	u32	valid;
	u32	proto;
	u32	lookup;
	u32	sram_bits;
	u32	tcam_bits;
	u32	num_entries;
};

/* User udf id mapping to Parser udf */
struct pp2_prs_udf_map {
	int	user_udf_idx;	/* pp2_parse_udf_params array index */
	u8	prs_udf_id;	/* hw udf number */
	int	tid;		/* parser tcam index for 1st entry */
	int	tid2;		/* parser tcam index for 2nd entry */
};

/********************************************************************************/
/*			PROTOTYPE						*/
/********************************************************************************/
int pp2_cls_prs_init(struct pp2_inst *inst);
int pp2_prs_udf_init(struct pp2_inst *inst, struct pp2_parse_udfs *prs_udfs);
void pp2_cls_prs_deinit(struct pp2_inst *inst);
int mv_pp2x_prs_flow_id_attr_get(int flow_id);
int pp2_prs_eth_start_hdr_set(struct pp2_port *port, enum pp2_ppio_eth_start_hdr eth_start_hdr);
int pp2_prs_set_log_port(struct pp2_port *port, struct pp2_ppio_log_port_params *params);
int pp2_prs_eth_start_header_set(struct pp2_port *port, enum pp2_ppio_eth_start_hdr mode);
void mv_pp2x_prs_clear_active_vlans(struct pp2_port *port, uint32_t *vlans);
int mv_pp2x_prs_mac_da_accept(struct pp2_port *port, const u8 *da, bool add);
int mv_pp2x_prs_hw_dump(struct pp2_inst *inst);
int mv_pp2x_prs_hw_hits_dump(struct pp2_inst *inst);
int pp2_prs_uid_to_prs_udf(unsigned int uid);

#endif /*_PP2_CLS_PRS_H_*/

