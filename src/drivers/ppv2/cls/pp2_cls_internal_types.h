/******************************************************************************
 *	Copyright (C) 2016 Marvell International Ltd.
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

/**
 * @file pp2_cls_internal_types.h
 *
 *  Internal definitions shared by sub-modules
 */

/* TODO - Merge with pp2_cls_types.h */

/***********************/
/* h file declarations */
/***********************/
#ifndef _PP2_CLS_INTERNAL_TYPES_H_
#define _PP2_CLS_INTERNAL_TYPES_H_

/********************************************************************************/
/*			MACROS							*/
/********************************************************************************/
#define MVPP2_FALSE	(0)	/* MVPP2 false state	*/
#define MVPP2_TRUE	(1)	/* MVPP2 true state	*/

/* GET_NUM_BYTES() convert bits to bytes and rounds it up */
#define GET_NUM_BYTES(field_size)	(!!(field_size % BYTE_BITS) +  field_size / BYTE_BITS)

#define QOS_LOG_PORT_TABLE_OFF(port_id)	(port_id + 4)

/* additional match filed bm, internal use */
#define MVPP2_MATCH_TTL			0x1000000
#define MVPP2_MATCH_TCP_FLAG_RF		0x2000000
#define MVPP2_MATCH_TCP_FLAG_S		0x4000000
#define MVPP2_MATCH_MH			0x8000000

#define MAC_ADDR_SIZE			(6)
#define IPV4_ADDR_SIZE			(4)
#define IPV6_ADDR_SIZE			(16)
#define WORD_BYTES			(4)
#define SHORT_BITS			(16)
#define BYTE_BITS			(8)
#define BYTE_MASK			(0xFF)
#define IPV4_VER			(4)
#define IPV6_VER			(6)
#define VLAN_PBIT_OFF			(13)/* The offset of pbit on VLAN */
/* IPvx Multist related */
#define IP4_MC_MAC_2BYTE		(0x0100)
#define IP4_MC_MAC_2BYTE_MASK		(0xFF00)
#define IP6_MC_MAC_2BYTE		(0x3333)
#define IP6_MC_MAC_2BYTE_MASK		(0xFFFF)

#define MVPP2_FLOW_FIELD_COUNT_MAX	(4)
#define MVPP2_VIRT_PORT_ID_MAX		(MVPP2_CLS_GEM_VIRT_REGS_NUM - 1)

#define MVPP2_HWF_MOD_IPTR_MAX		(255)
#define MVPP2_HW_MOD_DPTR_MAX		(23552)/* Private data 41KB, 41K/2 */
#define MVPP2_POLICER_ID_MAX		MVPP2_CLS3_ACT_DUP_POLICER_MAX

/* WAY definition */
/*------------------------------------------------------------------------------*/
#define MVPP2_WAY_PON			(1) /* WAY of PON */
#define MVPP2_WAY_NON_PON		(0) /* WAY of NONE PON port */

/* VLAN tag definition */
/*------------------------------------------------------------------------------*/
#define MVPP2_NUM_MAX_TPID_COMBO	(16)	/* Maximum number of TPID combo up to dual tag */
#define MVPP2_NUM_VLAN_ETYPE_REGS	(3)	/* Maximum number of Ethernet type registers*/
#define MVPP2_NUM_MAX_FILTER_TYPE	(8)	/* Maximum number of filter type */
#define MVPP2_INVALID_TPID		(0xFFFF)/* Invalid TPID for VLAN modification, TPID combo */
#define MVPP2_MAX_VLAN_ID_VALUE		(4095)	/* Maximum legal VLAN ID value */
#define MVPP2_MIN_VLAN_ID_VALUE		(0)	/* Minum legal VLAN ID value*/
#define MVPP2_MAX_PBITS_VALUE		(7)	/* Maximum legal P-bits value*/
#define MVPP2_INVALID_VLAN_ID		(0xFFFF)/* Invalid VLAN ID value */
#define MVPP2_VID_MASK			(0xFFF)	/* VID mask */

/* MTU definition */
/*------------------------------------------------------------------------------*/
/*default layer 2 MTU size, default L3 MTU + MH + DMAC + SMAC + VLAN tag + VLAN tags	*/
#define MVPP2_DEFAULT_L2_MTU_SIZE		(1500 + 2 + 12 + 4 + 4)

/*
 * Pre-defined FlowId assignment
*/
#define FLOWID_DEF(_port_)	(_port_)
#define FLOWID_MASK		0x3F

/* Whether it is default LSP LU ID */
/*------------------------------------------------------------------------------*/
#define LUID_IS_LSP_RESERVED(luid)	(NULL)

/* IPV4 Multicast IP */
#define MVPP2_IPV4_MC_IP		(0xE0)
#define MVPP2_IPV4_MC_IP_MASK		(0xE0)
#define MVPP2_IGMP_MAC_ADDR_MASK	(0x7F)/* Mask the 25th bit of MAC address */

/* IPV6 Multicast IP */
#define MVPP2_IPV6_MC_IP		(0xFF)
#define MVPP2_IPV6_MC_IP_MASK		(0xFF)

/* Multicast MAC */
#define MVPP2_MC_MAC			(0x01)
#define MVPP2_MC_MAC_MASK		(0xFF)

/* Classifier User Defined TPID and CFI's offset bits based on the start of packet*/
#define MVPP2_OUT_TPID_OFFSET_BITS	(112)
#define MVPP2_IN_TPID_OFFSET_BITS	(144)
#define MVPP2_TPID_BITS			(16)
#define MVPP2_CFI_BITS			(1)

/* CFI's offset bits in VLAN*/
#define MVPP2_CFI_OFFSET_BITS		(3)

/********************************************************************************/
/*			ENUMERATIONS						*/
/********************************************************************************/

enum pp2_cls_cls_field_id_t {
	MH_FIELD_ID			= 0,
	GEM_PORT_ID_FIELD_ID		= 1,
	MH_UNTAGGED_PRI_FIELD_ID	= 2,
	MAC_DA_FIELD_ID			= 3,
	MAC_SA_FIELD_ID			= 4,
	OUT_VLAN_PRI_FIELD_ID		= 5,
	OUT_VLAN_ID_FIELD_ID		= 6,
	IN_VLAN_ID_FIELD_ID		= 7,
	ETH_TYPE_FIELD_ID		= 8,
	PPPOE_FIELD_ID			= 9,
	IP_VER_FIELD_ID			= 10,
	IPV4_DSCP_FIELD_ID		= 11,
	IPV4_ECN_FIELD_ID		= 12,
	IPV4_LEN_FIELD_ID		= 13,
	IPV4_TTL_FIELD_ID		= 14,
	IPV6_HL_FIELD_ID		= 14,
	IPV4_PROTO_FIELD_ID		= 15,
	IPV6_PROTO_FIELD_ID		= 15,
	IPV4_SA_FIELD_ID		= 16,
	IPV4_DA_FIELD_ID		= 17,
	IPV6_DSCP_FIELD_ID		= 18,
	IPV6_ECN_FIELD_ID		= 19,
	IPV6_FLOW_LBL_FIELD_ID		= 20,
	IPV6_PAYLOAD_LEN_FIELD_ID	= 21,
	IPV6_NH_FIELD_ID		= 22,
	IPV6_SA_FIELD_ID		= 23,
	IPV6_SA_PREF_FIELD_ID		= 24,
	IPV6_SA_SUFF_FIELD_ID		= 25,
	IPV6_DA_FIELD_ID		= 26,
	IPV6_DA_PREF_FIELD_ID		= 27,
	IPV6_DA_SUFF_FIELD_ID		= 28,
	L4_SRC_FIELD_ID			= 29,
	L4_DST_FIELD_ID			= 30,
	TCP_FLAGS_FIELD_ID		= 31,
	USER_DEFINED_START_FIELD_ID	= 32,
	OUT_TPID_FIELD_ID		= USER_DEFINED_START_FIELD_ID,
	OUT_VLAN_CFI_FIELD_ID		= 33,
	IN_TPID_FIELD_ID		= 34,
	IN_VLAN_CFI_FIELD_ID		= 35,
	ARP_IPV4_DA_FIELD_ID		= 48,
	IN_VLAN_PRI_FIELD_ID		= 49,
	PPPOE_PROTO_ID			= 50,
	CLS_FIELD_MAX			= 51,
};

enum pp2_cls_cls_field_size_t {/* unit: bits */
	MH_FIELD_SIZE			= 16,
	GEM_PORT_ID_FIELD_SIZE		= 12,
	MH_UNTAGGED_PRI_FIELD_SIZE	= 3,
	MAC_DA_FIELD_SIZE		= 48,
	MAC_SA_FIELD_SIZE		= 48,
	OUT_VLAN_PRI_FIELD_SIZE		= 3,
	OUT_VLAN_ID_FIELD_SIZE		= 12,
	IN_VLAN_ID_FIELD_SIZE		= 12,
	ETH_TYPE_FIELD_SIZE		= 16,
	PPPOE_FIELD_SIZE		= 16,
	IP_VER_FIELD_SIZE		= 4,
	IPV4_DSCP_FIELD_SIZE		= 6,
	IPV4_ECN_FIELD_SIZE		= 2,
	IPV4_LEN_FIELD_SIZE		= 16,
	IPV4_TTL_FIELD_SIZE		= 8,
	IPV4_PROTO_FIELD_SIZE		= 8,
	IPV4_SA_FIELD_SIZE		= 32,
	IPV4_DA_FIELD_SIZE		= 32,
	IPV6_PROTO_FIELD_SIZE		= 8,
	IPV6_DSCP_FIELD_SIZE		= 6,
	IPV6_ECN_FIELD_SIZE		= 2,
	IPV6_FLOW_LBL_FIELD_SIZE	= 20,
	IPV6_PAYLOAD_LEN_FIELD_SIZE	= 16,
	IPV6_NH_FIELD_SIZE		= 8,
	IPV6_HL_FIELD_SIZE		= 8,
	IPV6_SA_FIELD_SIZE		= 128,
	IPV6_SA_PREF_FIELD_SIZE		= 64,
	IPV6_SA_SUFF_FIELD_SIZE		= 64,
	IPV6_DA_FIELD_SIZE		= 128,
	IPV6_DA_PREF_FIELD_SIZE		= 64,
	IPV6_DA_SUFF_FIELD_SIZE		= 64,
	L4_SRC_FIELD_SIZE		= 16,
	L4_DST_FIELD_SIZE		= 16,
	TCP_FLAGS_FIELD_SIZE		= 8,
	OUT_TPID_FIELD_SIZE		= 16,
	OUT_VLAN_CFI_FIELD_SIZE		= 1,
	IN_TPID_FIELD_SIZE		= 16,
	IN_VLAN_CFI_FIELD_SIZE		= 1,
	ARP_IPV4_DA_FIELD_SIZE		= 32,
	IN_VLAN_PRI_FIELD_SIZE		= 3,
	PPPOE_PROTO_SIZE		= 16,

};

enum pp2_cls_cls_field_valid_t {
	MVPP2_FIELD_INVALID = 0,
	MVPP2_FIELD_VALID
};

enum pp2_cls_engine_no_t {
	MVPP2_ENGINE_C2 = 1,
	MVPP2_ENGINE_C3_A,
	MVPP2_ENGINE_C3_B,
	MVPP2_ENGINE_C4,
	MVPP2_ENGINE_C3_HA = 6,
	MVPP2_ENGINE_C3_HB,
};

enum pp2_cls_l4_type_t {
	MVPP2_L4_TYPE_NO_EXIST = 0,		/* L4 Info no exist */
	MVPP2_L4_TYPE_TCP,			/* TCP type */
	MVPP2_L4_TYPE_UDP,			/* UDP type */
	MVPP2_L4_TYPE_MAX = MVPP2_L4_TYPE_UDP,
};

enum pp2_cls_l4_checksum_gen_t {
	MVPP2_L4_CKSUM_NO_GEN = 0,		/* Not Generate L4 checksum */
	MVPP2_L4_CKSUM_GEN,			/* Generate L4 checksum */
};

enum pp2_cls_queue_type_t {
	MVPP2_CPU_RX_QUEUE = 0,	/* CPU RX queue		*/
	MVPP2_HWF_TX_QUEUE	/* HWF/CPU TX queue	*/
};

enum pp2_cls_action_num_t {
	MVPP2_ACT_NUM_IS_1 = 1,
	MVPP2_ACT_NUM_IS_2,
	MVPP2_ACT_NUM_IS_3,
	MVPP2_ACT_NUM_IS_4,
	MVPP2_ACT_NUM_IS_5,
	MVPP2_ACT_NUM_IS_6,
	MVPP2_ACT_NUM_IS_7,
	MVPP2_ACT_NUM_IS_8,
	MVPP2_ACT_NUM_MAX = MVPP2_ACT_NUM_IS_8
};

enum pp2_cls_action_index_t {
	MVPP2_ACT_IDX0,
	MVPP2_ACT_IDX1,
	MVPP2_ACT_IDX2,
	MVPP2_ACT_IDX3,
	MVPP2_ACT_IDX4,
	MVPP2_ACT_IDX5,
	MVPP2_ACT_IDX6,
	MVPP2_ACT_IDX7,
	MVPP2_ACT_IDX_MAX = MVPP2_ACT_IDX7
};

/* This bit selects the value of the Port ID forwareded to the C2, C3 engines */
enum pp2_cls_cls_port_id_sel_t {
	MVPP2_CLS_PORT_ID_FROM_TBL = 0,
	MVPP2_CLS_PORT_ID_FROM_PKT,
};

enum pp2_cls_cls_seq_ctrl_t {
	MVPP2_CLS_SEQ_CTRL_NORMAL = 0,
	MVPP2_CLS_SEQ_CTRL_FIRST_TYPE_1,
	MVPP2_CLS_SEQ_CTRL_FIRST_TYPE_2,
	MVPP2_CLS_SEQ_CTRL_LAST,
	MVPP2_CLS_SEQ_CTRL_MIDDLE,
};

/********************************************************************************/
/*			STRUCTURES						*/
/********************************************************************************/
struct pp2_cls_ntuple_info_t {
	u32				ntuple_field_bm;
	u32				rule_type;
	enum pp2_cls_vlan_num_enum_t	tag_num;
	u32				curr_field;
	u32				field_num;
	u32				field_match_bm[MVPP2_NUM_MAX_NTUPLE_FIELD_NUM];
};

struct pp2_cls_mng_pkt_key_t {
	struct pp2_cls_pkt_key_t	*pkt_key;
	struct pp2_cls_ntuple_info_t	*nt_info;
	u8				ttl;
	u8				tcp_flag;
	u8				tcp_flag_mask;
	u16				mh;
	u16				mh_mask;
};

struct pp2_cls_mng_pkt_key_db_t {
	struct pp2_cls_pkt_key_t	pkt_key;
	u8				ttl;
	u8				tcp_flag;
	u8				tcp_flag_mask;
	u8				dummy;
};

struct pp2_cls_field_int_value_t {
	u32	parsed_int_val;
	u32	parsed_int_val_mask;
};

struct pp2_cls_field_mac_addr_t {
	u8	parsed_mac_addr[MAC_ADDR_SIZE];
	u8	parsed_mac_addr_mask[MAC_ADDR_SIZE];
};

struct pp2_cls_field_ipv4_addr_t {
	u8	parsed_ipv4_addr[IPV4_ADDR_SIZE];
	u8	parsed_ipv4_addr_mask[IPV4_ADDR_SIZE];
};

struct pp2_cls_field_ipv6_addr_t {
	u8	parsed_ipv6_addr[IPV6_ADDR_SIZE];
	u8	parsed_ipv6_addr_mask[IPV6_ADDR_SIZE];
};

union pp2_cls_field_value_union_t {
	struct pp2_cls_field_int_value_t	int_data;
	struct pp2_cls_field_mac_addr_t		mac_addr;
	struct pp2_cls_field_ipv4_addr_t	ipv4_addr;
	struct pp2_cls_field_ipv6_addr_t	ipv6_addr;
};

struct pp2_cls_field_match_info {
	int					valid;
	u32					field_id;
	union pp2_cls_field_value_union_t	filed_value;
};

struct pp2_cls_hit_cnt_t {
	u32	log_idx;
	u32	phys_idx;
	u32	cntr_val;
};

struct pp2_cls_seq_instr_t {
	u32	instr_low;		/* bit 0-31   */
	u32	instr_high;		/* bit 32-37 */
};

struct pp2_cls_seq_instr_info_t {
	struct pp2_cls_seq_instr_t	instr_value;
	u32				bits_num;
};

struct pp2_cls_engine_sram_t {
	struct mv_pp2x_engine_qos_info		qos_info;	/* C2 QoS table info            */
	struct mv_pp2x_engine_pkt_action	action;		/* update&lock info		*/
	struct mv_pp2x_qos_value		qos_value;	/* qLow/qHigh			*/
	struct mv_pp2x_engine_pkt_mod		pkt_mod;	/* PMT cmd idx and data idx	*/
	struct mv_pp2x_duplicate_info		dup_info;	/* pkt duplication flow info    */
	struct pp2_cls_seq_instr_info_t		instr_info;
};

#endif /* _PP2_CLS_INTERNAL_TYPES_H_ */
