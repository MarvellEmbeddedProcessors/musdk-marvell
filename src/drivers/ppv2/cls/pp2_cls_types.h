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
 * @file pp2_cls_types.h
 *
 *  Internal definitions shared by sub-modules
 */

/* TODO - Merge with pp2_cls_internal_types.h */

/***********************/
/* h file declarations */
/***********************/
#ifndef _PP2_CLS_TYPES_H_
#define _PP2_CLS_TYPES_H_

/********************************************************************************/
/*			MACROS							*/
/********************************************************************************/
#define MVPP2_CFI_UPDATE	(0x10)		/* Update original CFI */
#define MVPP2_CFI_KEEP		(0x00)		/* Keep original CFI */
#define MVPP2_PBIT_UPDATE	(0x10)		/* Update the PBIT */
#define MVPP2_PBIT_KEEP		(0x00)		/* Keep original PBIT */
#define MVPP2_VID_UPDATE	(0x1000)	/* Update the VID */
#define MVPP2_VID_KEEP		(0x0000)	/* Keep original VID */
#define MVPP2_TPID_UPDATE	(0x00010000)	/* Update the TPID */
#define MVPP2_TPID_KEEP		(0x00000000)	/* Keep original TPID */
#define MVPP2_TRG_PORT_MAX	(6)		/* The max number of target port */
#define MVPP2_POLICER_INVALID	(0xff)
#define MVPP2_CNM_FIELD_MAX	(6)		/* The max number of fields in CnM rule */
#define MVPP2_RULE_ID_INVALID	(0)

#define MVPP2_NUM_MAX_NTUPLE_FIELD_NUM	(9)
#define MVPP2_INFI_SEQ_FIELD_SIZE	(0xFFFF)

/********************************************************************************/
/*			ENUMERATIONS						*/
/********************************************************************************/
enum pp2_cls_gmacs_enum_t {
	MVPP2_INVALID_GMAC = -1,
	MVPP2_ENUM_GMAC_0,
	MVPP2_ENUM_GMAC_1,
	MVPP2_ENUM_GMAC_LPK,
	MVPP2_ENUM_PMAC = 7,
	MVPP2_MAX_GMAC = MVPP2_ENUM_PMAC,
	MVPP2_MAX_NUM_GMACS
};

enum pp2_cls_field_match_t {
	/* L2 */
	MVPP2_MATCH_ETH_DST           = 0x000001,	/* Match Ethernet Destination Address */
	MVPP2_MATCH_ETH_SRC           = 0x000002,	/* Match Ethernet Source Address */
	MVPP2_MATCH_VID_OUTER         = 0x000004,	/* Match Outer VID */
	MVPP2_MATCH_PBITS_OUTER       = 0x000008,	/* Match Outer Pbits */
	MVPP2_MATCH_VID_INNER         = 0x000010,	/* Match Inner VID */
	MVPP2_MATCH_PBITS_INNER       = 0x000020,	/* Match Inner Pbits */
	MVPP2_MATCH_ETH_TYPE          = 0x000040,	/* Match Ethertype */

	/* PPPoE */
	MVPP2_MATCH_PPPOE_PROTO       = 0x000080,	/* Match PPPoE Protocol */
	MVPP2_MATCH_PPPOE_SES         = 0x000100,	/* Match PPPoE SessionId */

	/* IPV4/V6 */
	MVPP2_MATCH_IPV4_PKT          = 0x000200,	/* Match IPv4 Packet, used together with other*/
							/*  MVPP2_MATCH_IP_XX */
	MVPP2_MATCH_IPV6_PKT          = 0x000400,	/* Match IPv6 Packet, used together with other*/
							/*  MVPP2_MATCH_IP_XX */
	MVPP2_MATCH_IP_SRC            = 0x000800,	/* Match IPV4/6 Source Address */
	MVPP2_MATCH_IP_DST            = 0x001000,	/* Match IPV4/6 Destination Address */
	MVPP2_MATCH_IP_DSCP           = 0x002000,	/* Match DSCP */
	MVPP2_MATCH_IPV6_FLBL         = 0x004000,	/* Match IPV6 Flow Label */
	MVPP2_MATCH_IP_PROTO          = 0x008000,	/* Match IPv4_Proto/IPv6_NH field */
	MVPP2_MATCH_IP_VERSION        = 0x010000,	/* Match IP version field */
	MVPP2_MATCH_L4_SRC            = 0x020000,	/* Match L4 Source Port (UDP or TCP) */
	MVPP2_MATCH_L4_DST            = 0x040000,	/* Match L4 Destination Port (UDP or TCP) */
	MVPP2_MATCH_IPV6_PREF         = 0x080000,	/* Match IPV6 address prefix */
	MVPP2_MATCH_IPV6_SUFF         = 0x100000,	/* Match IPV6 address profix */
	MVPP2_MATCH_ARP_TRGT_IP_ADDR  = 0x200000,	/* Match ARP TARGET IP ADDR */

	/*TPID and CFI*/
	MVPP2_MATCH_TPID_OUTER        = 0x0400000, /* Match Outer TPID */
	MVPP2_MATCH_CFI_OUTER         = 0x0800000, /* Match Outer CFI */
	MVPP2_MATCH_TPID_INNER        = 0x1000000, /* Match Inner TPID */
	MVPP2_MATCH_CFI_INNER         = 0x2000000, /* Match Inner CFI */
};

#define MVPP2_MATCH_FIELD_ALL	(MVPP2_MATCH_ETH_DST | MVPP2_MATCH_ETH_SRC | MVPP2_MATCH_VID_OUTER \
				| MVPP2_MATCH_PBITS_OUTER | MVPP2_MATCH_VID_INNER \
				| MVPP2_MATCH_PBITS_INNER | MVPP2_MATCH_ETH_TYPE \
				| MVPP2_MATCH_PPPOE_PROTO | MVPP2_MATCH_PPPOE_SES \
				| MVPP2_MATCH_IPV4_PKT | MVPP2_MATCH_IPV6_PKT | MVPP2_MATCH_IP_SRC \
				| MVPP2_MATCH_IP_DST | MVPP2_MATCH_IP_DSCP \
				| MVPP2_MATCH_IPV6_FLBL | MVPP2_MATCH_IP_PROTO | MVPP2_MATCH_IP_VERSION\
				| MVPP2_MATCH_L4_SRC | MVPP2_MATCH_L4_DST | MVPP2_MATCH_IPV6_PREF\
				| MVPP2_MATCH_IPV6_SUFF | MVPP2_MATCH_ARP_TRGT_IP_ADDR\
				| MVPP2_MATCH_TPID_OUTER | MVPP2_MATCH_CFI_OUTER\
				| MVPP2_MATCH_TPID_INNER | MVPP2_MATCH_CFI_INNER) /* All MVPP2 fields*/

#define MVPP2_MATCH_FIELD_IP	(MVPP2_MATCH_IPV4_PKT | MVPP2_MATCH_IPV6_PKT | MVPP2_MATCH_IP_SRC \
				| MVPP2_MATCH_IP_DST | MVPP2_MATCH_IP_DSCP \
				| MVPP2_MATCH_IPV6_FLBL | MVPP2_MATCH_IP_PROTO | MVPP2_MATCH_IP_VERSION\
				| MVPP2_MATCH_L4_SRC | MVPP2_MATCH_L4_DST | MVPP2_MATCH_IPV6_PREF\
				| MVPP2_MATCH_IPV6_SUFF | MVPP2_MATCH_ARP_TRGT_IP_ADDR) /* All MVPP2 IP fields*/

enum pp2_cls_color_t {
	MVPP2_COL_GREEN		= 0x00,	/* Treat as green for Classifier */
	MVPP2_COL_YELLOW	= 0x01,	/* Treat as yellow for Classifier */
	MVPP2_COL_RED		= 0x02	/* drop packet */
};

enum pp2_cls_packet_mod_t {
/* L2 */
	MVPP2_ETH_DST_SET	= 0x00001,	/* Modify ETH DST */
	MVPP2_ETH_SRC_SET	= 0x00002,	/* Modify ETH SRC */
	MVPP2_VLAN_MOD		= 0x00004,	/* Modify VLAN Tag(s) */
	MVPP2_ETH_TYPE_SET	= 0x00008,	/* Modify Ethertype */
/* Note: MH_SET not to be implemented, unless proven necessary. */

	/* PPPoE */
	MVPP2_PPOE_ADD		= 0x00010,	/* Add PPPoE encapsulation */
	MVPP2_PPOE_DEL		= 0x00020,	/* Delete PPPoE encapsulation */

/* IPV4/V6 */
	MVPP2_IP_DSCP_SET	= 0x00100,	/* Set the DSCP value */
	MVPP2_TTL_HOPL_DEC	= 0x00200,	/* Decrease the TTL/HOPL value by 1 */
	MVPP2_IP_SRC_SET	= 0x00400,	/* Set the IPV4/6 Source Address */
	MVPP2_IP_DST_SET	= 0x00800,	/* Set the IPV4/6 Destination Address */
	MVPP2_L4_SRC_SET	= 0x01000,	/* Set the L4 Source Port (UDP or TCP) */
	MVPP2_L4_DST_SET	= 0x02000	/* Set the L4 Destination Port (UDP or TCP) */
};

enum pp2_cls_qos_sel_t {
	MVPP2_QOS_NONE		= 0,	/* Do not use QoS Table */
	MVPP2_QOS_DSCP		= 1,	/* Use DSCP to select QoS Table */
	MVPP2_QOS_PBIT		= 2	/* Use Pbits to select QoS Table */
};

enum pp2_cls_class_port_type_t {
	MVPP2_CLASS_PP_PORT_BM,
	MVPP2_CLASS_VIRT_PORT
};

enum pp2_cls_port_id_t {
	MVPP2_PORTID_SPEC	= 0x1000,	/* Use specific port_id */
	MVPP2_PORTID_ANY	= 0x0000	/* No specific port_id */
};

enum pp2_cls_vlan_id_t {
	MVPP2_ALL_VID	= 0xffff,	/* VID magic number for single/double tag */
	MVPP2_NO_VID	= 0x0000,	/* VID magic number for untag */
	MVPP2_SPEC_VID	= 0x1000,	/* Use specific vid */
};

enum pp2_cls_vlan_oper_t {
	MVPP2_VLANOP_NOOP,			/* no VLAN operation performed */
	MVPP2_VLANOP_EXT_TAG_MOD,		/* modify external tag */
	MVPP2_VLANOP_EXT_TAG_DEL,		/* delete external tag */
	MVPP2_VLANOP_EXT_TAG_INS,		/* insert(prepend) external tag */
	MVPP2_VLANOP_EXT_TAG_MOD_INS,		/* modify existing external tag and insert(prepend) new tag */
	MVPP2_VLANOP_INS_2TAG,			/* insert(prepend) 2 new tags */
	MVPP2_VLANOP_MOD_2TAG,			/* modify 2 tags */
	MVPP2_VLANOP_SWAP_TAGS,			/* swap internal and external tags */
	MVPP2_VLANOP_DEL_2TAG,			/* delete 2 existing tags */
	MVPP2_VLANOP_INT_TAG_MOD,		/* modify existing internal tag */
	MVPP2_VLANOP_EXT_TAG_DEL_INT_MOD,	/* delete existing external tag and modify internal tag */
	MVPP2_VLANOP_SPLIT_MOD_PBIT,		/* split mod stage 2, only modify p-bit */
	MVPP2_VLANOP_ILLEGAL			/* illegal VLAN operation */
};

union pp2_cls_ipvx_add {
	u8   ipv4[4];	/* IPV4 Address */
	u8   ipv6[16];	/* IPV6 Address */
};

/* Port definition */
/*------------------------------------------------------------------------------*/
enum pp2_cls_pp_port_t {
	MVPP2_PP_DROP     = 0x0000,	/* Drop packet */
	MVPP2_PP_GMAC0    = 0x0001,	/* Packet Processor GMAC0 */
	MVPP2_PP_GMAC1    = 0x0002,	/* Packet Processor GMAC1 */
	MVPP2_PP_PMAC     = 0x0004,	/* Packet Processor PON_MAC */
	MVPP2_PP_LPBK     = 0x0008,	/* Packet Processor Loopback Port */
	MVPP2_PP_CPU      = 0x0010	/* CPU */
};

/* Capability definition */
/*------------------------------------------------------------------------------*/
enum pp2_cls_vlan_num_enum_t {
	MVPP2_NO_VLAN,
	MVPP2_SINGLE_VLAN,
	MVPP2_DOUBLE_VLAN,
	MVPP2_TRIPLE_VLAN,
	MVPP2_NOT_DOUBLE_VLAN,			/* untag or single tag */
	MVPP2_ANY_VLAN,				/* vlan number is not relevant */
};

enum pp2_cls_bad_chksum_action_t {
	MVPP2_BAD_CHKSUM_NO_DROP,		/* do not drop bad L3/4 checksum packet */
	MVPP2_BAD_CHKSUM_DROP,			/* not drop bad L3/4 checksum packet */
};

/* Multicast definition */
/*------------------------------------------------------------------------------*/
enum pp2_cls_traffic_handler_t {
	MVPP2_HANDLER_GEN_CLAS,		/* general classification */
	MVPP2_HANDLER_EXACT_MATCH,	/* exact match */
	MVPP2_HANDLER_MC		/* multicast */
};

enum pp2_cls_mc_traffic_layer_t {
	MVPP2_MC_TRAFFIC_L2,		/* Multicast L2 */
	MVPP2_MC_TRAFFIC_L3,		/* Multicast L3 */
	MVPP2_MC_LAYER_MAX
};

enum pp2_cls_mc_flow_def_action_t {
	MVPP2_MC_FLOW_DEFAULT_DROP,	/* drop packets by default */
	MVPP2_MC_FLOW_DEFAULT_CPU	/* filter to CPU by default */
};

/* MISC definition */
/*------------------------------------------------------------------------------*/
enum pp2_cls_ipv6_support_t {
	MVPP2_IPV6_NOT_SUPPORTED = 0,	/* does not support IPv6		*/
	MVPP2_IPV6_SUPPORTED		/* support IPv6			*/
};

enum pp2_cls_tcp_flag_check_t {
	MVPP2_TCP_FLAG_NOT_CHECK = 0,	/* does not check TCP flag	*/
	MVPP2_TCP_FLAG_CHECK		/* check TCP flag		*/
};

/********************************************************************************/
/*			STRUCTURES						*/
/********************************************************************************/
struct pp2_cls_pppoe_key_t {
	u16	ppp_session;	/* PPPoE session */
	u16	ppp_proto;	/* PPPoE Protocol */
};

/* VLAN modification commands */
struct pp2_cls_addn_qual_t {
	u32	addn_qual_bm;	/* BM of additional qualifiers to handle (N-T) to match flow. */
						/* Refer to enum pp2_cls_field_match_t for values*/
	u16	ether_type;	/* ether_type: 0x0800(IPv4oE),0x86DD(IPv6oE),or 0x8864(PPPoE) */
	u16	ppp_proto;	/* ppp_proto values: 0x21(IPv4oPPPoE), 0x57(IPv6oPPPoE)*/
	u8	eth_dst[6];	/* Ethernet Destination Address */
	u8	eth_src[6];	/* Ethernet Source Address */
	u16	out_vid;	/* Outer VID, 0xffff->untagged */
	u8	out_pbit;	/* Outer pbits */
	u16	inn_vid;	/* Inner VID 0xffff->single tagged */
	u16	pppoe_ses;	/* 0xffff->no pppoe */
	u8	dscp;
};

struct pp2_cls_5t_t {
	u16			ip_ver;		/* IP version (4,6) */
	u16			l4_proto;	/* L4 protocol type, udp/tcp */
	u16			l4_src;		/* Source Port */
	u16			l4_dst;		/* Destination Port */
	union pp2_cls_ipvx_add	ip_src;		/* Source IP address */
	union pp2_cls_ipvx_add	ip_dst;		/* Destination IP address */
};

struct pp2_cls_class_port_t {
	enum pp2_cls_class_port_type_t	port_type;
	u32			class_port;
};

struct pp2_cls_exact_match_t {
	u32				rule_type;
	struct mv_pp2x_src_port		port;
	struct pp2_cls_5t_t		ipvx_five_t;	/* IPV4 or IPV6 five-tuple */
	struct pp2_cls_addn_qual_t	addn_qual;	/* Additional qualifiers (N-tuple) */
};

struct pp2_cls_vlan_mod_t {
	u32	tpid;		/* Set TPID_UPDATE to update the TPID */
	u16	vid;		/* Set VID_UPDATE to update the vid */
	u8	cfi;		/* Set CFI_UPDATE to update the cfi */
	u8	pbit;		/* Set PBIT_UPDATE to update the pbits */
};

struct pp2_cls_vlan_op_t {
	enum pp2_cls_vlan_oper_t	oper;		/* Vlan operation/modification command */
	struct pp2_cls_vlan_mod_t	out_vlan;	/* Structure for outer VLAN key */
	struct pp2_cls_vlan_mod_t	inn_vlan;	/* Structure for inner VLAN key */
};

struct pp2_cls_ipvx_mod_t {
	union pp2_cls_ipvx_add		ip_src;		/*IPV4/IPV6  source address for packet modification */
	union pp2_cls_ipvx_add		ip_dst;		/*IPV4/IPV6  dest address for packet modification */
	u16				l4_src;		/*TCP/UDP  source port for packet modification */
	u16				l4_dst;		/*TCP/UDP  dest port for packet modification */
	u8				dscp;		/*dscp value for packet modification */
};

struct pp2_cls_pkt_mod_t {
	u32				mod_bm;		/* Bitmap of modification fields (enum pp2_cls_packet_mod_t)*/
	u8				eth_dst[6];	/* Ethernet source address for packet modification */
	u8				eth_src[6];	/* Ethernet dest address for packet modification */
	struct pp2_cls_vlan_op_t	vlan_op;	/* Vlan Operation and data for packet modification */
	struct pp2_cls_pppoe_key_t	pppoe_mod;	/* PPPoe data for inserting PPPoE tunnel */
	struct pp2_cls_ipvx_mod_t	ipvx;		/* IPV4/IPV6 and TCP/UDP data for packet modification */
};

struct pp2_cls_eth_add_key_t {
	u8	eth_add[6];	/* Ethernet Address */
	u8	eth_add_mask[6];/* Ethernet Address Mask*/
};

struct pp2_cls_ipvx_add_key_t {
	union pp2_cls_ipvx_add	ip_add;		/* IPV4/IPV6 Address */
	union pp2_cls_ipvx_add	ip_add_mask;	/* IPV4/IPV6 Address Mask*/
};

struct pp2_cls_ipvx_key_t {
	u16				ip_ver;		/* IP version (4,6) */
	struct pp2_cls_ipvx_add_key_t	ip_src;		/* Maskable IPV4/IPV6 source address */
	struct pp2_cls_ipvx_add_key_t	ip_dst;		/* Maskable IPV4/IPV6 dest address */
	u8				dscp;		/* IPV4/IPV6 dscp */
	u8				dscp_mask;	/* IPV4/IPV6 dscp mask*/
	u8				ip_proto;	/* IP protocol */
	u32				flow_label;	/* ipv6 only */
	u32				flow_label_mask;/* ipv6 only */
};

struct pp2_cls_pkt_key_t {
	struct mv_pp2x_src_port		port;
	u32				rule_type;	/* rules with smaller value has higher priority being matched*/
							/* should be set together with cap/filter during MVPP2 INIT */
	u32				field_bm;	/* Bitmap of packet fields to match (pp2_cls_field_match_t) */
	u32				field_bm_mask;	/* Bitmap of packet fields to match (pp2_cls_field_match_t) */
	struct pp2_cls_eth_add_key_t	eth_dst;	/* Ethernet Destination address & mask */
	struct pp2_cls_eth_add_key_t	eth_src;	/* Ethernet Destination address & mask */
	u16				out_vid;	/* Outer Tag VID, 0x0000->untagged, 0xffff->tagged */
							/* 0x1ABC->specific Outer VID=ABC */
	u8				out_pbit;	/* Outer Tag pbits */
	u16				out_tpid;	/* Outer TPID */
	u8				out_cfi;	/* Outer Tag cfi */
	u16				inn_vid;	/* Inner VID,0x0000->not double-tagged,0xffff-> double-tagged */
							/* 0x1ABC->specific Inner VID=ABC */
	u8				inn_pbit;	/* Inner Tag pbits */
	u16				inn_tpid;	/* Inner TPID */
	u8				inn_cfi;	/* Inner Tag cfi */
	u16				ether_type;	/* Ethertype (after Vlan Tags)*/
	struct pp2_cls_pppoe_key_t	ppp_info;	/* PPPoE key (proto, ppp_session) */
	struct pp2_cls_ipvx_key_t	ipvx_add;	/*IPV4/IPV6 packet key */
	struct pp2_cls_ipvx_add_key_t	arp_ip_dst;	/* ARP IPV4 dest address */
	u16				l4_src;		/*UDP/TCP source port */
	u16				l4_dst;		/*UDP/TCP dest port */
};

#endif /* _PP2_CLS_TYPES_H_ */

