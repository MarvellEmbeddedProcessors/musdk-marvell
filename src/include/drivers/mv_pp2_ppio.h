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

#ifndef __MV_PP2_PPIO_H__
#define __MV_PP2_PPIO_H__

#include "mv_std.h"
#include "env/mv_sys_event.h"

#include "mv_pp2.h"
#include "mv_pp2_hif.h"
#include "mv_pp2_bpool.h"

/** @addtogroup grp_pp2_io Packet Processor: I/O
 *
 *  Packet Processor I/O API documentation
 *
 *  @{
 */

struct pp2_ppio {
	u32	pp2_id;			/* packet processor's Id */
	u32	port_id;		/* port Id */
	void	*internal_param;	/* parameters for internal use */
};

struct pp2_bpool;

#define PP2_PPIO_MAX_NUM_TCS		32 /**< Max. number of TCs per ppio. */
#define PP2_PPIO_MAX_NUM_INQS		32 /**< Max. number of inqs per ppio. */
#define PP2_PPIO_MAX_NUM_OUTQS		8 /**< Max. number of outqs per ppio. */
#define PP2_PPIO_TC_CLUSTER_MAX_POOLS	2 /**< Max. number of bpools per TC per mem_id. */
#define PP2_PPIO_TC_MAX_POOLS		\
	(PP2_PPIO_TC_CLUSTER_MAX_POOLS * MV_SYS_DMA_MAX_NUM_MEM_ID) /**< Max. number of bpools per TC. */

enum pp2_ppio_type {
	PP2_PPIO_T_LOG = 0,	/*  Logical-port is only a set of Out-Qs and In-TCs (i.e. no link, l2-filters) */
	PP2_PPIO_T_NIC		/*  NIC is a logical-port with link and l2-filters */
};

enum pp2_ppio_hash_type {
	PP2_PPIO_HASH_T_NONE = 0,	/* Invalid hash type (hashing mechanism is disabled) */
	PP2_PPIO_HASH_T_2_TUPLE,	/* IP-src, IP-dst */
	PP2_PPIO_HASH_T_5_TUPLE,	/* IP-src, IP-dst, IP-Prot, L4-src, L4-dst */
	PP2_PPIO_HASH_T_OUT_OF_RANGE
};

#define PP2_PPIO_MIN_CBS	64
#define PP2_PPIO_MIN_CIR	100

struct pp2_ppio_rate_limit_params {
	u32 cbs;	/* committed_burst_size, in kilobytes. Min: 64kB */
	u32 cir;	/* committed_information_rate, in kilobits per second. Min: 100kbps */
};

/**
 * The enum below defines the possible ethernet header formats
 */
enum pp2_ppio_eth_start_hdr {
	/** Normal ethernet header */
	PP2_PPIO_HDR_ETH = 0,
	/** Packets contain Marvell Header */
	/* TODO: PP2_PPIO_HDR_ETH_MH, */
	/** Packets contain Marvell DSA header of type ethtype (8 bytes)*/
	/* TODO: PP2_PPIO_HDR_ETH_MH_ETYPE_DSA, */
	/** Packets contain Marvell DSA header (4 bytes)*/
	PP2_PPIO_HDR_ETH_DSA,
	/** Packets contain Marvell extended DSA header (8 bytes)*/
	PP2_PPIO_HDR_ETH_EXT_DSA,
	PP2_PPIO_HDR_OUT_OF_RANGE
};

enum pp2_ppio_outqs_sched_mode {
	PP2_PPIO_SCHED_M_NONE = 0,
};

enum pp2_ppio_log_port_rule_type {
	PP2_RULE_TYPE_PROTO = 0,
	PP2_RULE_TYPE_PROTO_FIELD
};

/**
 * The enum below defines the logical port target
 */
enum pp2_ppio_cls_target {
	PP2_CLS_TARGET_LOCAL_PPIO = 0,	/* traffic forwarded to kernel and logical port rules to MUSDK */
	PP2_CLS_TARGET_OTHER		/* traffic forwarded to MUSDK and logical port rules to kernel */
};

enum pp2_ppio_color {
	PP2_PPIO_COLOR_GREEN	= 0,	/* Treat as green for Policer */
	PP2_PPIO_COLOR_YELLOW,		/* Treat as yellow for Policer */
	PP2_PPIO_COLOR_RED		/* drop packet */
};

/**
 * ppio inq parameters
 *
 */
struct pp2_ppio_inq_params {
	u32	size; /**< q_size in number of descriptors */
	struct mv_sys_dma_mem_region *mem;
	u8	tc_pools_mem_id_index;	/* 0..(MV_SYS_DMA_MAX_NUM_MEM_ID-1) */
};

/**
 * ppio inq statistics
 *
 * Note: Drop counters are 32 bit counters and they will be updated up to
 *       32/16 bit maximum value (0xFFFFFFFF/0xFFFF) and will not be wrapped around.
 *
 */
struct pp2_ppio_inq_statistics {
	u64	enq_desc;	/**< ppio inq descriptors enqueue counter */
	u32	drop_fullq;	/**< ppio inq full queue dropped packets counter */
	u16	drop_early;	/**< ppio inq early dropped packets counter */
	u16	drop_bm;	/**< ppio inq BM dropped packets counter */
};

/**
 * ppio tc parameters
 *
 */
struct pp2_ppio_tc_params {
	u16				 pkt_offset; /**< pkt offset, must be multiple of 32 bytes */
	u16				 num_in_qs;  /**< number of inqs .
							* Value greater than 1 assumes use of hashing mechanism
							*/
	struct pp2_ppio_inq_params	*inqs_params; /**< pointer to the tc's inq parameters */;
	/**< bpools used by the tc */
	struct pp2_bpool		*pools[MV_SYS_DMA_MAX_NUM_MEM_ID][PP2_PPIO_TC_CLUSTER_MAX_POOLS];
	enum pp2_ppio_color		default_color; /**< Default color for this TC */
};

/**
 * ppio inq related parameters
 *
 */
struct pp2_ppio_inqs_params {
	u16				num_tcs; /**< Number of tcs */
	struct pp2_ppio_tc_params	tcs_params[PP2_PPIO_MAX_NUM_TCS]; /**< Parameters for each tc */
	/** Indicate the type of hash to use in hashing mechanism according to pp2_ppio_hash_type.
	 * The hash type is common to all TC's in ppio
	 */
	enum pp2_ppio_hash_type		hash_type;
	struct pp2_cls_plcr		*plcr; /**< pointer to a default policer handle; if 'NULL' it doens't set */
};

/**
 * ppio outq parameters
 *
 */
enum pp2_ppio_outq_sched_mode {
	PP2_PPIO_SCHED_M_WRR = 0,
	PP2_PPIO_SCHED_M_SP
};

struct pp2_ppio_outq_params {
	u32	size;	/**< q_size in number of descriptors */

	enum pp2_ppio_outq_sched_mode	sched_mode;

	/** The weight is relative among the PP-IO out-Qs; this field is relevant only if
	 * PP2_PPIO_SCHED_M_WRR scheduler is selected for this out-Q.
	 * Weight is in the range 0-255.
	 */
	u8				weight;

	int				rate_limit_enable;

	/** this field is relevant only if this out-Q has rate-limit enabled */
	struct pp2_ppio_rate_limit_params rate_limit_params;
};

/**
 * ppio outq statistics
 *
 */
struct pp2_ppio_outq_statistics {
	u64	enq_desc;	/**< ppio outq descriptors enqueue counter */
	u64	enq_dec_to_ddr;	/**< ppio outq enqueue descriptors to DRAM counter */
	u64	enq_buf_to_ddr;	/**< ppio outq enqueue buffers to DRAM counter */
	u64	deq_desc;	/**< ppio outq packets dequeue counter */
};

/**
 * ppio outq related parameters
 *
 */
struct pp2_ppio_outqs_params {
	u16				 num_outqs; /**< Number of outqs */
	struct pp2_ppio_outq_params	 outqs_params[PP2_PPIO_MAX_NUM_OUTQS]; /**< Parameters for each outq */

	int				 sched_enable;
};

/**
* ppio "Logical-Port" rule parameters
*/
struct pp2_ppio_log_port_rule_params {
	/** Indicate whether the rule is a network protocol or a special protocol-field
	 * PP2_RULE_TYPE_PROTO, PP2_RULE_TYPE_PROTO_FIELD
	 */
	enum pp2_ppio_log_port_rule_type	rule_type;
	union {
		struct {
			/** Defines a network protocol to be supported by logical port
			 * The following network protocols are supported:
			 * - MV_NET_PROTO_VLAN,
			 * - MV_NET_PROTO_PPPOE,
			 * - MV_NET_PROTO_IP,
			 * - MV_NET_PROTO_IP4,
			 * - MV_NET_PROTO_IP6,
			 * - MV_NET_PROTO_TCP,
			 * - MV_NET_PROTO_UDP,
			 */
			enum mv_net_proto	proto;
			/** Indicates whether the selected network protocol is to be matched or the negated
			 * network protocol is to be matched
			* i.e. if VLAN protocol is specified:
			* val = 0 indicates all tagged frames are to be matched
			* val = 1 indicates all untagged frames are to be matched
			*/
			int			val;
		} proto_params;
		struct {
			/** Defines a network special protocol to be supported by logical port
			 * The following special protocols fields are supported:
			 * - proto:	MV_NET_PROTO_ETH_DSA
			 * - field:		MV_NET_ETH_F_DSA_TAG_MODE
			 */
			struct pp2_proto_field	proto_field;
			/** Defines the value of the specified special protocol field
			 * i.e. if DSA is supported and MV_NET_ETH_F_DSA_TAG_MODE is selected as special protocol field
			 * the available values are defined in mv_net.h under mv_net_eth_dsa_tag_mode_values
			 */
			u8			val;
		} proto_field_params;
	} u;
};

/**
* ppio "Logical-Port" related parameters
*/
struct pp2_ppio_log_port_params {
	struct {
		/** Indicate whether the network protocol rules defined below will be used for the NIC
		* or for the logical-port;
		* PP2_CLS_TARGET_LOCAL_PPIO, PP2_CLS_TARGET_OTHER
		 */
		enum pp2_ppio_cls_target			target;
		/** Number of network protocol and protocol-field rules supported by logical port */
		u8						num_proto_rule_sets;
		struct {
			/** Number of network protocol and protocol-field rules to match*/
			u8					num_rules;
			struct pp2_ppio_log_port_rule_params	rules[PP2_MAX_PROTO_SUPPORTED];
		} rule_sets[PP2_MAX_PROTO_SUPPORTED];
	} proto_based_target;
	u32	first_inq;
};

/**
 * ppio "NIC" related parameters
 *
 */
struct pp2_ppio_nic_params {
	/** Override the default MTU (1500B); '0' value means not to override */
	u16	override_mtu;
	/** Override the default MRU (1500B); '0' value means not to override */
	u16	override_mru;
};

/**
 * ppio parameters
 *
 */
struct pp2_ppio_params {
	/** Used for DTS acc to find appropriate "physical" PP-IO obj;
	 * E.g. "eth-0:0" means PPv2[0],port[0]
	 */
	const char			*match;

	enum pp2_ppio_type		 type; /**<  ppio type. TODO: currently only support "NIC" */
	struct pp2_ppio_inqs_params	 inqs_params; /**<  ppio inq parameters structure */
	struct pp2_ppio_outqs_params	 outqs_params; /**<  ppio outq parameters structure */
	int				 maintain_stats; /**< ppio statistics maintaining flag.
							  * If set, the statistics will be updated
							  * automatically when number of rx/tx packets
							  * reach defined threshold.
							  */
	union {
		/** relevant only if PPIO type is 'PP2_PPIO_T_LOG' */
		struct pp2_ppio_log_port_params	log_port_params;
		/** relevant only if PPIO type is 'PP2_PPIO_T_NIC' */
		struct pp2_ppio_nic_params	nic_params;
	} specific_type_params;

	enum pp2_ppio_eth_start_hdr		eth_start_hdr;

	int					rate_limit_enable;
	/** this field is relevant only if this PP-IO rate-limit is enabled.
	 * Please note that when PP-IO rate limit is enable, the entire port
	 * is impact by this setting (i.e. even in case of logical-port).
	 */
	struct pp2_ppio_rate_limit_params rate_limit_params;

/* TODO: do we need extra pools per port?
 *	struct pp2_bpool		*pools[PP2_PPIO_TC_MAX_POOLS];
 */
};

/**
 * ppio statistics
 *
 */
struct pp2_ppio_statistics {
	/* Rx port statistics */
	u64	rx_bytes;		/**< RX Bytes Counter;
					* relevant only if PPIO type is 'PP2_PPIO_T_NIC'
					*/
	u64	rx_packets;		/**< RX Packets Counter */
	u64	rx_unicast_packets;	/**< RX Unicast Packets Counter;
					* relevant only if PPIO type is 'PP2_PPIO_T_NIC'
					*/
	u64	rx_errors;		/**< RX MAC Errors Counter;
					* relevant only if PPIO type is 'PP2_PPIO_T_NIC'
					*/
	u64	rx_fullq_dropped;	/**< RX Full Queue Dropped Counter */
	u32	rx_bm_dropped;		/**< RX BM Dropped Counter */
	u32	rx_early_dropped;	/**< RX Early Dropped Counter  */
	u32	rx_fifo_dropped;	/**< RX FIFO Overrun Counter;
					* relevant only if PPIO type is 'PP2_PPIO_T_NIC'
					*/
	u32	rx_cls_dropped;		/**< RX CLS Dropped Counter;
					* relevant only if PPIO type is 'PP2_PPIO_T_NIC'
					*/
	/* Tx port statistics */
	u64	tx_bytes;		/**< TX Bytes Counter;
					* relevant only if PPIO type is 'PP2_PPIO_T_NIC'
					*/
	u64	tx_packets;		/**< TX Packets Counter */
	u64	tx_unicast_packets;	/**< TX Unicast Packets Counter;
					* relevant only if PPIO type is 'PP2_PPIO_T_NIC'
					*/
	u64	tx_errors;		/**< TX MAC Errors Counter;
					* relevant only if PPIO type is 'PP2_PPIO_T_NIC'
					*/
};

struct pp2_ppio_rxq_event_params {
	u32 pkt_coal;
	u32 usec_coal;
	u32 tc_inqs_mask[PP2_PPIO_MAX_NUM_TCS];
};

/**
 * Initialize a ppio
 *
 * @param[in]	params	A pointer to structure that contains all relevant parameters.
 * @param[out]	ppio	A pointer to opaque ppio handle of type 'struct pp2_ppio *'.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 */
int pp2_ppio_init(struct pp2_ppio_params *params, struct pp2_ppio **ppio);

/**
 * Destroy a ppio
 *
 * @param[in]	ppio	A ppio handle.
 *
 */
void pp2_ppio_deinit(struct pp2_ppio *ppio);

/****************************************************************************
 *	Run-time API
 ****************************************************************************/

#define PP2_PPIO_DESC_NUM_WORDS	8
#define PP2_PPIO_DESC_NUM_FRAGS	16 /* TODO: check if thereâs HW limitation */

struct pp2_ppio_desc {
	u32			 cmds[PP2_PPIO_DESC_NUM_WORDS];
};

struct pp2_ppio_sg_desc {
	u8			 num_frags;
	struct pp2_ppio_desc	 descs[PP2_PPIO_DESC_NUM_FRAGS];
};

enum pp2_outq_l3_type {
	PP2_OUTQ_L3_TYPE_IPV4 = 0,
	PP2_OUTQ_L3_TYPE_IPV6,
	PP2_OUTQ_L3_TYPE_OTHER
};

enum pp2_outq_l4_type {
	PP2_OUTQ_L4_TYPE_TCP = 0,
	PP2_OUTQ_L4_TYPE_UDP,
	PP2_OUTQ_L4_TYPE_OTHER
};

enum pp2_inq_l3_type {
	PP2_INQ_L3_TYPE_NA = 0,
	PP2_INQ_L3_TYPE_IPV4_NO_OPTS,	/* IPv4 with IHL=5, TTL>0 */
	PP2_INQ_L3_TYPE_IPV4_OK,	/* IPv4 with IHL>5, TTL>0 */
	PP2_INQ_L3_TYPE_IPV4_TTL_ZERO,	/* Other IPV4 packets */
	PP2_INQ_L3_TYPE_IPV6_NO_EXT,	/* IPV6 without extensions */
	PP2_INQ_L3_TYPE_IPV6_EXT,	/* IPV6 with extensions */
	PP2_INQ_L3_TYPE_ARP,		/* ARP */
	PP2_INQ_L3_TYPE_USER_DEFINED	/* User defined */
};

enum pp2_inq_l4_type {
	PP2_INQ_L4_TYPE_NA = 0,
	PP2_INQ_L4_TYPE_TCP = 1,
	PP2_INQ_L4_TYPE_UDP = 2,
	PP2_INQ_L4_TYPE_OTHER = 3
};

enum pp2_inq_vlan_tag {
	PP2_INQ_VLAN_TAG_NONE = 0,	/* No VLANs */
	PP2_INQ_VLAN_TAG_SINGLE,	/* Single VLAN */
	PP2_INQ_VLAN_TAG_DOUBLE,	/* Double VLANs */
	PP2_INQ_VLAN_TAG_TRIPLE,	/* Triple VLANs */
};

enum pp2_inq_l2_cast_type {
	PP2_INQ_L2_UNICAST = 0,		/* L2 Unicast */
	PP2_INQ_L2_MULTICAST,		/* L2 Multicast */
	PP2_INQ_L2_BROADCAST,		/* L2 Broadcast */
};

enum pp2_inq_l3_cast_type {
	PP2_INQ_L3_UNICAST = 0,		/* L3 Unicast */
	PP2_INQ_L3_MULTICAST,		/* L3 Multicast */
	PP2_INQ_L3_ANYCAST,		/* L3 Anycast */
	PP2_INQ_L3_BROADCAST,		/* L3 Broadcast */
};

enum pp2_inq_desc_status {
	PP2_DESC_ERR_OK = 0,
	PP2_DESC_ERR_MAC_CRC,		/* L2 MAC error (for example CRC Error) */
	PP2_DESC_ERR_MAC_OVERRUN,	/* L2 Overrun Error*/
	PP2_DESC_ERR_MAC_RESERVED,	/* L2 Reserved */
	PP2_DESC_ERR_MAC_RESOURCE,	/* L2 Resource Error (No buffers for multi-buffer frame) */
	PP2_DESC_ERR_IPV4_HDR,		/* L3 IPv4 Header error */
	PP2_DESC_ERR_L4_CHECKSUM	/* L4 checksum error */
};

/******** TXQ  ********/

/* TODO: Add PTP, PME, L4Icheck */

/* NOTE: Following functions must be called.
 * 'pp2_ppio_outq_desc_set_pkt_offset' is optimized for performance so it MUST be called before
 *	'pp2_ppio_outq_desc_set_pkt_len'
 *
 *	pp2_ppio_outq_desc_reset ()
 *	pp2_ppio_outq_desc_set_phys_addr()
 *	pp2_ppio_outq_desc_set_proto_info()
 *	pp2_ppio_outq_desc_set_pkt_offset()
 *	pp2_ppio_outq_desc_set_pkt_len()
 */

/******************** TxQ-desc *****************/
/* cmd 0 */
#define TXD_FIRST                  (0x2)
#define TXD_LAST                   (0x1)
#define TXD_FIRST_LAST             (0x3)
#define TXD_IP_CHK_DISABLE         (0x1)
#define TXD_IP_CHK_ENABLE          (0x0)
#define TXD_L4_CHK_ENABLE          (0x0)
#define TXD_L4_CHK_FRG_ENABLE      (0x1)
#define TXD_L4_CHK_DISABLE         (0x2)

#define TXD_L_MASK                 (0x10000000)
#define TXD_F_MASK                 (0x20000000)
#define TXD_FL_MASK                (TXD_F_MASK | TXD_L_MASK)
#define TXD_FORMAT_MASK            (0x40000000)
#define TXD_L3_TYPE_MASK           (0x0C000000)
#define TXD_L4_TYPE_MASK           (0x03000000)
#define TXD_PKT_OFF_EXT_MASK       (0x00100000)
#define TXD_POOL_ID_MASK           (0x000F0000)
#define TXD_GEN_L4_CHK_MASK        (0x00006000)
#define TXD_GEN_IP_CHK_MASK        (0x00008000)
#define TXD_IP_HEAD_LEN_MASK       (0x00001F00)

#define TXD_BUFMODE_MASK           (0x00000080)
#define TXD_L3_OFFSET_MASK         (0x0000007F)

/* cmd 1 */
#define TXD_PKT_OFF_MASK           (0x000000FF)
#define TXD_DEST_QID_MASK          (0x0000FF00)
#define TXD_BYTE_COUNT_MASK        (0xFFFF0000)
/* cmd 4 */
#define TXD_BUF_PHYS_LO_MASK       (0xFFFFFFFF)
/* cmd 5 */
#define TXD_BUF_PHYS_HI_MASK       (0x000000FF)
#define TXD_PTP_DESC_MASK          (0xFFFFFF00)
/* cmd 6 */
#define TXD_BUF_VIRT_LO_MASK       (0xFFFFFFFF)
/* cmd 7 */
#define TXD_BUF_VIRT_HI_MASK       (0x000000FF)

/******************** RxQ-desc *****************/
/* cmd 0 */
#define RXD_L3_OFF_MASK            (0x0000007F)
#define RXD_IPHDR_LEN_MASK         (0x00001F00)
#define RXD_EC_MASK                (0x00006000)
#define RXD_ES_MASK                (0x00008000)
#define RXD_POOL_ID_MASK           (0x000F0000)
#define RXD_HWF_SYNC_MASK          (0x00200000)
#define RXD_L4_CHK_OK_MASK         (0x00400000)
#define RXD_L3_IP_FRAG_MASK        (0x00800000)
#define RXD_L3_IP4_HDR_ERR_MASK    (0x01000000)
#define RXD_L4_PRS_INFO_MASK       (0x0E000000)
#define RXD_L3_PRS_INFO_MASK       (0x70000000)
#define RXD_BUF_HDR_MASK           (0x80000000)
/* cmd 1 */
#define RXD_BYTE_COUNT_MASK        (0xFFFF0000)
#define RXD_VLAN_INFO_MASK	   (0x0000C000)
#define RXD_L2_CAST_INFO_MASK	   (0x00003000)
#define RXD_L3_CAST_INFO_MASK	   (0x00000C00)
/* cmd 4 */
#define RXD_BUF_PHYS_LO_MASK       (0xFFFFFFFF)
/* cmd 5 */
#define RXD_BUF_PHYS_HI_MASK       (0x000000FF)
#define RXD_KEY_HASH_MASK          (0xFFFFFF00)
/* cmd 6 */
#define RXD_BUF_VIRT_LO_MASK       (0xFFFFFFFF)
/* cmd 7 */
#define RXD_BUF_VIRT_HI_MASK       (0x000000FF)

#define DM_RXD_GET_L3_OFF(desc)         (((desc)->cmds[0] & RXD_L3_OFF_MASK) >> 0)
#define DM_RXD_GET_IPHDR_LEN(desc)      (((desc)->cmds[0] & RXD_IPHDR_LEN_MASK) >> 8)
#define DM_RXD_GET_EC(desc)             (((desc)->cmds[0] & RXD_EC_MASK) >> 13)
#define DM_RXD_GET_ES(desc)             (((desc)->cmds[0] & RXD_ES_MASK) >> 15)
#define DM_RXD_GET_POOL_ID(desc)        (((desc)->cmds[0] & RXD_POOL_ID_MASK) >> 16)
#define DM_RXD_GET_HWF_SYNC(desc)       (((desc)->cmds[0] & RXD_HWF_SYNC_MASK) >> 21)
#define DM_RXD_GET_L4_CHK_OK(desc)      (((desc)->cmds[0] & RXD_L4_CHK_OK_MASK) >> 22)
#define DM_RXD_GET_L3_IP_FRAG(desc)     (((desc)->cmds[0] & RXD_L3_IP_FRAG_MASK) >> 23)
#define DM_RXD_GET_L3_IP4_HDR_ERR(desc) (((desc)->cmds[0] & RXD_L3_IP4_HDR_ERR_MASK) >> 24)
#define DM_RXD_GET_L4_PRS_INFO(desc)    (((desc)->cmds[0] & RXD_L4_PRS_INFO_MASK) >> 25)
#define DM_RXD_GET_L3_PRS_INFO(desc)    (((desc)->cmds[0] & RXD_L3_PRS_INFO_MASK) >> 28)
#define DM_RXD_GET_BUF_HDR(desc)        (((desc)->cmds[0] & RXD_BUF_HDR_MASK) >> 31)

#define DM_RXD_GET_VLAN_INFO(desc)	(((desc)->cmds[1] & RXD_VLAN_INFO_MASK) >> 14)
#define DM_RXD_GET_L2_CAST_INFO(desc)	(((desc)->cmds[1] & RXD_L2_CAST_INFO_MASK) >> 12)
#define DM_RXD_GET_L3_CAST_INFO(desc)	(((desc)->cmds[1] & RXD_L3_CAST_INFO_MASK) >> 10)

#define DM_TXD_SET_GEN_L4_CHK(desc, data)	\
	((desc)->cmds[0] = ((desc)->cmds[0] & ~TXD_GEN_L4_CHK_MASK) | (data << 13 & TXD_GEN_L4_CHK_MASK))
#define DM_TXD_SET_GEN_IP_CHK(desc, data)	\
	((desc)->cmds[0] = ((desc)->cmds[0] & ~TXD_GEN_IP_CHK_MASK) | (data << 15 & TXD_GEN_IP_CHK_MASK))
#define DM_TXD_SET_FIRST_LAST(desc, data)	\
	((desc)->cmds[0] = ((desc)->cmds[0] & ~TXD_FL_MASK) | (data << 28 & TXD_FL_MASK))
#define DM_TXD_SET_L3_OFF(desc, data)	\
		((desc)->cmds[0] = ((desc)->cmds[0] & ~TXD_L3_OFFSET_MASK) | (data << 0 & TXD_L3_OFFSET_MASK))
#define DM_TXD_SET_IP_HEAD_LEN(desc, data)	\
	((desc)->cmds[0] = ((desc)->cmds[0] & ~TXD_IP_HEAD_LEN_MASK) | (data << 8 & TXD_IP_HEAD_LEN_MASK))
#define DM_TXD_SET_L3_TYPE(desc, data)	\
	((desc)->cmds[0] = ((desc)->cmds[0] & ~TXD_L3_TYPE_MASK) | (data << 26 & TXD_L3_TYPE_MASK))
#define DM_TXD_SET_L4_TYPE(desc, data)	\
	((desc)->cmds[0] = ((desc)->cmds[0] & ~TXD_L4_TYPE_MASK) | (data << 24 & TXD_L4_TYPE_MASK))

/**
 * Reset an outq packet descriptor to default value.
 *
 * @param[out]	desc	A pointer to a packet descriptor structure.
 *
 */
static inline void pp2_ppio_outq_desc_reset(struct pp2_ppio_desc *desc)
{
	desc->cmds[0] = desc->cmds[1] = desc->cmds[2] = desc->cmds[3] =
	desc->cmds[5] = desc->cmds[7] = 0;

	/* Do not generate L4 nor IPv4 header checksum by default */
	DM_TXD_SET_GEN_IP_CHK(desc, TXD_IP_CHK_DISABLE);
	DM_TXD_SET_GEN_L4_CHK(desc, TXD_L4_CHK_DISABLE);
	DM_TXD_SET_FIRST_LAST(desc, TXD_FIRST_LAST);
}

/**
 * Set the physical address in an outq packet descriptor.
 *
 * @param[out]	desc	A pointer to a packet descriptor structure.
 * @param[in]	addr	Physical DMA address containing the packet to be sent.
 *
 */
static inline void pp2_ppio_outq_desc_set_phys_addr(struct pp2_ppio_desc *desc, dma_addr_t addr)
{
	desc->cmds[4] = (u32)addr;
	desc->cmds[5] = (desc->cmds[5] & ~TXD_BUF_PHYS_HI_MASK) | ((u64)addr >> 32 & TXD_BUF_PHYS_HI_MASK);
}

/**
 * Set the user specified cookie in an outq packet descriptor.
 *
 * @param[out]	desc	A pointer to a packet descriptor structure.
 * @param[in]	cookie	User specified cookie.
 *
 */
static inline void pp2_ppio_outq_desc_set_cookie(struct pp2_ppio_desc *desc, u64 cookie)
{
	desc->cmds[6] = (u32)cookie;
	desc->cmds[7] = (desc->cmds[7] & ~TXD_BUF_VIRT_HI_MASK) | (cookie >> 32 & TXD_BUF_VIRT_HI_MASK);
}

/**
 * Set the ppv2 pool to return the buffer to, after sending the packet.
 * Calling this API will cause PPV2 HW to return the buffer to the PPV2 Buffer Manager.
 *
 * @param[out]	desc	A pointer to a packet descriptor structure.
 * @param[in]	pool	A bpool handle.
 *
 */
void pp2_ppio_outq_desc_set_pool(struct pp2_ppio_desc *desc, struct pp2_bpool *pool);

/**
 * Set the protocol info in an outq packet descriptor.
 * This API must be called, if the PPV2 needs to generate l3 or l4 checksum.
 * Otherwise, it may, or may not, be called.
 *
 * @param[out]	desc		A pointer to a packet descriptor structure.
 * @param[in]	l3_type		The l3 type of the packet.
 * @param[in]	l4_type		The l4 type of the packet.
 * @param[in]	l3_offset	The l3 offset of the packet.
 * @param[in]	l4_offset	The l4 offset of the packet.
 * @param[in]	gen_l3_chk	Set to '1' to generate IPV4 checksum.
 * @param[in]	gen_l4_chk	Set to '1' to generate TCP/UDP checksum.
 *
 */
static inline void pp2_ppio_outq_desc_set_proto_info(struct pp2_ppio_desc *desc, enum pp2_outq_l3_type l3_type,
						     enum pp2_outq_l4_type l4_type, u8  l3_offset, u8 l4_offset,
						     int gen_l3_chk, int gen_l4_chk)
{
	DM_TXD_SET_L3_TYPE(desc, l3_type);
	DM_TXD_SET_L4_TYPE(desc, l4_type);
	DM_TXD_SET_L3_OFF(desc, l3_offset);
	DM_TXD_SET_IP_HEAD_LEN(desc, (l4_offset - l3_offset)/sizeof(u32));
	DM_TXD_SET_GEN_IP_CHK(desc, ((gen_l3_chk) ? TXD_IP_CHK_ENABLE : TXD_IP_CHK_DISABLE));
	DM_TXD_SET_GEN_L4_CHK(desc, ((gen_l4_chk) ? TXD_L4_CHK_ENABLE : TXD_L4_CHK_DISABLE));
}

/**
 * TODO - Add a Marvell DSA Tag to the packet.
 *
 * @param[out]	desc	A pointer to a packet descriptor structure.
 *
 */
void pp2_ppio_outq_desc_set_dsa_tag(struct pp2_ppio_desc *desc);

static inline void pp2_ppio_outq_desc_set_pkt_offset(struct pp2_ppio_desc *desc, u8  offset)
{
	desc->cmds[1] = (u32)offset;
}

/**
 * Set the packet length in an outq packet descriptor..
 *
 * @param[out]	desc	A pointer to a packet descriptor structure.
 * @param[in]	len	The packet length, not including CRC.
 *
 */
static inline void pp2_ppio_outq_desc_set_pkt_len(struct pp2_ppio_desc *desc, u16 len)
{
	desc->cmds[1] = (desc->cmds[1] & ~TXD_BYTE_COUNT_MASK) | (len << 16 & TXD_BYTE_COUNT_MASK);
}

/******** RXQ  ********/

/* TODO: Timestamp, L4IChk */

/**
 * Get the physical DMA address from an inq packet descriptor.
 *
 * @param[in]	desc	A pointer to a packet descriptor structure.
 *
 * @retval	physical dma address
 */
static inline dma_addr_t pp2_ppio_inq_desc_get_phys_addr(struct pp2_ppio_desc *desc)
{
	return ((u64)((desc->cmds[5] & RXD_BUF_PHYS_HI_MASK) >> 0) << 32) |
		((u64)((desc->cmds[4] & RXD_BUF_PHYS_LO_MASK) >> 0));
}

/**
 * Get the user defined cookie from an inq packet descriptor.
 *
 * @param[in]	desc	A pointer to a packet descriptor structure.
 *
 * @retval	cookie
 */
static inline u64 pp2_ppio_inq_desc_get_cookie(struct pp2_ppio_desc *desc)
{
	return ((u64)((desc->cmds[7] & RXD_BUF_VIRT_HI_MASK) >> 0) << 32) |
		((u64)((desc->cmds[6] & RXD_BUF_VIRT_LO_MASK) >> 0));
}

struct pp2_ppio *pp2_ppio_inq_desc_get_pp_io(struct pp2_ppio_desc *desc); /*Note: under _DEBUG_*/

/**
 * Get the packet length from an inq packet descriptor.
 *
 * @param[in]	desc	A pointer to a packet descriptor structure.
 *
 * @retval	packet length
 */
static inline u16 pp2_ppio_inq_desc_get_pkt_len(struct pp2_ppio_desc *desc)
{
	u16 len = (desc->cmds[1] & RXD_BYTE_COUNT_MASK) >> 16;
	len -= MV_MH_SIZE;
	return len;
}

/**
 * Get the Layer 3 information from an inq packet descriptor.
 *
 * @param[in]	desc	A pointer to a packet descriptor structure.
 * @param[out]	type	A pointer to l3 type.
 * @param[out]	offset	A pointer to l3 offset, relative to start of frame-on-the-wire (not including MH).
 *
 */
static inline void pp2_ppio_inq_desc_get_l3_info(struct pp2_ppio_desc *desc, enum pp2_inq_l3_type *type, u8 *offset)
{
	*type   = (desc->cmds[0] & RXD_L3_PRS_INFO_MASK) >> 28;
	*offset = (desc->cmds[0] & RXD_L3_OFF_MASK) >> 0;
	*offset -= MV_MH_SIZE;
}

/**
 * Get the Layer 4 information from an inq packet descriptor.
 *
 * @param[in]	desc	A pointer to a packet descriptor structure.
 * @param[out]	type	A pointer to l4 type.
 * @param[out]	offset	A pointer to l4 offset.
 *
 */
static inline void pp2_ppio_inq_desc_get_l4_info(struct pp2_ppio_desc *desc, enum pp2_inq_l4_type *type, u8 *offset)
{
	*type   = (desc->cmds[0] & RXD_L4_PRS_INFO_MASK) >> 25;
	*offset = ((desc->cmds[0] & RXD_L3_OFF_MASK) >> 0) + sizeof(u32)*((desc->cmds[0] & RXD_IPHDR_LEN_MASK) >> 8);
	*offset -= MV_MH_SIZE;
}


/**
 * Get the VLAN tag information from an inq packet descriptor.
 *
 * @param[in]	desc	A pointer to a packet descriptor structure.
 * @param[out]	tag	A pointer to vlan tag.
 *
 */
static inline void pp2_ppio_inq_desc_get_vlan_tag(struct pp2_ppio_desc *desc, enum pp2_inq_vlan_tag *tag)
{
	*tag = DM_RXD_GET_VLAN_INFO(desc);
}

/**
 * Get the Layer 2 casting information from an inq packet descriptor.
 *
 * @param[in]	desc	A pointer to a packet descriptor structure.
 * @param[out]	type	A pointer to cast type.
 *
 */
static inline void pp2_ppio_inq_desc_get_l2_cast_info(struct pp2_ppio_desc *desc, enum pp2_inq_l2_cast_type *type)
{
	*type = DM_RXD_GET_L2_CAST_INFO(desc);
}

/**
 * Get the Layer 3 casting information from an inq packet descriptor.
 *
 * @param[in]	desc	A pointer to a packet descriptor structure.
 * @param[out]	type	A pointer to cast type.
 *
 */
static inline void pp2_ppio_inq_desc_get_l3_cast_info(struct pp2_ppio_desc *desc, enum pp2_inq_l3_cast_type *type)
{
	*type = DM_RXD_GET_L3_CAST_INFO(desc);
}

/**
 * TODO - Check if there is IPV4 fragmentation in an inq packet descriptor.
 *
 * @param[in]	desc	A pointer to a packet descriptor structure.
 *
 * @retval	0 - not fragmented, 1 - fragmented.
 */
static inline int pp2_ppio_inq_desc_get_ip_isfrag(struct pp2_ppio_desc *desc)
{
	return DM_RXD_GET_L3_IP_FRAG(desc);
}

/**
 * Get the bpool of an inq packet descriptor.
 *
 * @param[in]	desc	A pointer to a packet descriptor structure.
 * @param[in]	ppio	A pointer to a PP-IO object.
 *
 * @retval	pointer to bpool
 */
static inline struct pp2_bpool *pp2_ppio_inq_desc_get_bpool(struct pp2_ppio_desc *desc, struct pp2_ppio *ppio)
{
	return &pp2_bpools[ppio->pp2_id][DM_RXD_GET_POOL_ID(desc)];
}



/**
 * Check if packet in inq packet descriptor has a L2 MAC (CRC) error condition.
 *
 * @param[in]	desc	A pointer to a packet descriptor structure.
 *
 * @retval	see enum pp2_inq_desc_status.
 */

static inline enum pp2_inq_desc_status pp2_ppio_inq_desc_get_l2_pkt_error(struct pp2_ppio_desc *desc)
{
	if (unlikely(DM_RXD_GET_ES(desc)))
		return (1 + DM_RXD_GET_EC(desc));
	return PP2_DESC_ERR_OK;
}

/**
 * Check if packet in inq packet descriptor has or L3 IPv4 error condition.
 *
 * @param[in]	desc	A pointer to a packet descriptor structure.
 *
 * @retval	see enum pp2_inq_desc_status.
 */
static inline enum pp2_inq_desc_status pp2_ppio_inq_desc_get_l3_pkt_error(struct pp2_ppio_desc *desc)
{
	enum pp2_inq_l3_type l3_info = DM_RXD_GET_L3_PRS_INFO(desc);

	if (unlikely((l3_info > PP2_INQ_L3_TYPE_NA) &&
		     (l3_info <= PP2_INQ_L3_TYPE_IPV4_TTL_ZERO) &&
		     DM_RXD_GET_L3_IP4_HDR_ERR(desc)))
		return PP2_DESC_ERR_IPV4_HDR;
	return PP2_DESC_ERR_OK;
}

/**
 * Check if packet in inq packet descriptor has IP/L4 (checksum) error condition.
 *
 * @param[in]	desc	A pointer to a packet descriptor structure.
 *
 * @retval	see enum pp2_inq_desc_status.
 */
static inline enum pp2_inq_desc_status pp2_ppio_inq_desc_get_l4_pkt_error(struct pp2_ppio_desc *desc)
{
	enum pp2_inq_l4_type l4_info = DM_RXD_GET_L4_PRS_INFO(desc);

	if (unlikely((l4_info == PP2_INQ_L4_TYPE_TCP || l4_info == PP2_INQ_L4_TYPE_UDP) &&
		     !DM_RXD_GET_L3_IP_FRAG(desc) && !DM_RXD_GET_L4_CHK_OK(desc)))
		return PP2_DESC_ERR_L4_CHECKSUM;
	return PP2_DESC_ERR_OK;
}

/**
 * Check if packet in inq packet descriptor has some error condition.
 *
 * @param[in]	desc	A pointer to a packet descriptor structure.
 *
 * @retval	see enum pp2_inq_desc_status.
 */
static inline enum pp2_inq_desc_status pp2_ppio_inq_desc_get_pkt_error(struct pp2_ppio_desc *desc)
{
	enum pp2_inq_desc_status status;

	status = pp2_ppio_inq_desc_get_l2_pkt_error(desc);
	if (unlikely(status))
		return status;

	status = pp2_ppio_inq_desc_get_l3_pkt_error(desc);
	if (unlikely(status))
		return status;

	return pp2_ppio_inq_desc_get_l4_pkt_error(desc);
}

/**
 * Send a batch of frames (single dscriptor) on an OutQ of PP-IO.
 *
 * The routine assumes that the BM-Pool is either freed by HW (by appropriate desc
 * setter) or by the MUSDK client SW.
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[in]		hif	A hif handle.
 * @param[in]		qid	out-Q id on which to send the frames.
 * @param[in]		descs	A pointer to an array of descriptors representing the
 *				frames to be sent.
 * @param[in,out]	num	input: number of frames to be sent; output: number of frames sent.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_ppio_send(struct pp2_ppio	*ppio,
		  struct pp2_hif	*hif,
		  u8			 qid,
		  struct pp2_ppio_desc	*descs,
		  u16			*num);

/**
 * TODO - Send a batch of S/G frames (single or multiple dscriptors) on an OutQ of PP-IO.
 *
 * The routine assumes that the BM-Pool is either freed by HW (by appropriate desc
 * setter) or by the MUSDK client SW.
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[in]		hif	A hif handle.
 * @param[in]		qid	out-Q id on which to send the frames.
 * @param[in]		descs	A pointer to an array of S/G-descriptors representing the
 *				frames to be sent.
 * @param[in,out]	num	input: number of frames to be sent; output: number of frames sent.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_ppio_send_sg(struct pp2_ppio		*ppio,
		     struct pp2_hif		*hif,
		     u8			 qid,
		     struct pp2_ppio_sg_desc	*descs,
		     u16			*num);

/**
 * Get number of packets sent on a queue, since last call of this API.
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[in]		hif	A hif handle.
 * @param[in]		qid	out-Q id on which to send the frames.
 * @param[out]		num	Number of frames that were sent.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_ppio_get_num_outq_done(struct pp2_ppio	*ppio,
			       struct pp2_hif	*hif,
			       u8		 qid,
			       u16		*num);

/**
 * Get out-Q statistics
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[in]		qid	out-Q id.
 * @param[out]		stats	out-Q statistics.
 * @param[in]		reset	A flag indicates if counters should be reset.
 *
 */
int pp2_ppio_outq_get_statistics(struct pp2_ppio *ppio, u8 qid,
				 struct pp2_ppio_outq_statistics *stats, int reset);


/**
 * Receive packets on a ppio.
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[in]		tc	traffic class on which to receive frames
 * @param[in]		qid	in-Q id on which to receive the frames.
 * @param[in]		descs	A pointer to an array of descriptors represents the
 *				received frames.
 * @param[in,out]	num	input: Max number of frames to receive;
 *				output: number of frames received.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_ppio_recv(struct pp2_ppio	*ppio,
		  u8			 tc,
		  u8			 qid,
		  struct pp2_ppio_desc	*descs,
		  u16			*num);

/**
 * Get in-Q statistics
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[in]		tc	traffic class on which to receive frames
 * @param[in]		qid	in-Q id.
 * @param[out]		stats	in-Q statistics.
 * @param[in]		reset	A flag indicates if counters should be reset.
 *
 */
int pp2_ppio_inq_get_statistics(struct pp2_ppio *ppio, u8 tc, u8 qid,
				struct pp2_ppio_inq_statistics *stats, int reset);


/****************************************************************************
 *	Run-time Control API
 ****************************************************************************/

/**
 * pp2 ppio in tc queue capabilities info
 *
 */
struct pp2_ppio_q_info {
	u16		size;
};

/**
 * pp2 ppio in tc capabilities info
 *
 */
struct pp2_ppio_intc_info {
	u16				pkt_offset;
	u8				num_inqs;
	struct pp2_ppio_q_info		inqs_infs[PP2_PPIO_MAX_NUM_INQS];
	struct pp2_bpool		pools[PP2_PPIO_TC_MAX_POOLS];
};

/**
 * pp2 ppio in tcs capabilities info
 *
 */
struct pp2_ppio_intcs_info {
	u8				num_intcs;
	struct pp2_ppio_intc_info	intcs_infs[PP2_PPIO_MAX_NUM_TCS];
};

/**
 * pp2 ppio out queues capabilities info
 *
 */
struct pp2_ppio_outqs_info {
	u8				num_outtcs;
	struct pp2_ppio_q_info		outqs_infs[PP2_PPIO_MAX_NUM_OUTQS];
};

/**
 * pp2 ppio capabilities
 *
 */
struct pp2_ppio_capabilities {
	u8	pp2_id;
	u8	id;
	struct pp2_ppio_intcs_info	intcs_inf;
	struct pp2_ppio_outqs_info	outqs_inf;
};

/**
 * Get ppio capabilities
 *
.* This API should be called by the user application in order to retrieve the information and capabilities needed
 * for a probed ppio object.
 *
 * @param[in]	ppio		ppio probed structure
 * @param[out]	capa		ppio information and capabilities
 *
 * @retval      0 on success
 * @retval      <0 on failure
 */
int pp2_ppio_get_capabilities(struct pp2_ppio *ppio, struct pp2_ppio_capabilities *capa);

/**
 * Enable a ppio
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_ppio_enable(struct pp2_ppio *ppio);

/**
 * Disable a ppio
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_ppio_disable(struct pp2_ppio *ppio);

/**
 * Set ppio Ethernet MAC address
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[in]		addr	Ethernet MAC address to configure .
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_ppio_set_mac_addr(struct pp2_ppio *ppio, const eth_addr_t addr);

/**
 * Get ppio Ethernet MAC address
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[out]		addr	Configured ppio Ethernet MAC address .
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_ppio_get_mac_addr(struct pp2_ppio *ppio, eth_addr_t addr);
int pp2_ppio_set_mtu(struct pp2_ppio *ppio, u16 mtu); /* For debug only, check mtu during  pkt_send() */
int pp2_ppio_get_mtu(struct pp2_ppio *ppio, u16 *mtu);
int pp2_ppio_set_mru(struct pp2_ppio *ppio, u16 len);
int pp2_ppio_get_mru(struct pp2_ppio *ppio, u16 *len);
int pp2_ppio_set_loopback(struct pp2_ppio *ppio, int en);
int pp2_ppio_get_loopback(struct pp2_ppio *ppio, int *en);

/**
 * Set ppio to promiscuous mode
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[in]		en	1 - enable, 0 - disable.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_ppio_set_promisc(struct pp2_ppio *ppio, int en);

/**
 * Get ppio promiscuous mode
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[out]		en	1 - enabled, 0 - disabled.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_ppio_get_promisc(struct pp2_ppio *ppio, int *en);

/**
 * Set ppio to listen to all multicast mode
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[in]		en	1 - enable, 0 - disable.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_ppio_set_mc_promisc(struct pp2_ppio *ppio, int en);

/**
 * Get ppio all multicast mode
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[out]		en	1 - enabled, 0 - disabled.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_ppio_get_mc_promisc(struct pp2_ppio *ppio, int *en);

/**
 * Add ppio Ethernet MAC address
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[in]		addr	Ethernet MAC address to add .
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_ppio_add_mac_addr(struct pp2_ppio *ppio, const eth_addr_t addr);

/**
 * Remove ppio Ethernet MAC address
 *
 * Allows to remove the mac_address set  by pp2_ppio_set_mac_addr().
  *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[in]		addr	Ethernet MAC address to remove .
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_ppio_remove_mac_addr(struct pp2_ppio *ppio, const eth_addr_t addr);

/**
 * Flush ppio Ethernet MAC address
 *
 * Does not flush the mac_address set  by pp2_ppio_set_mac_addr().
.*
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[in]		uc	1 - flush multicast list.
 * @param[in]		mc	1 - flush unicast list .
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_ppio_flush_mac_addrs(struct pp2_ppio *ppio, int uc, int mc);

int pp2_ppio_get_phys_in_q(struct pp2_ppio *ppio, u8 tc, u8 qid, u8 *pq);

/**
 * Add ppio filtering vlan id
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[in]		vlan	vlan id to add.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_ppio_add_vlan(struct pp2_ppio *ppio, u16 vlan);

/**
 * Remove ppio filtering vlan id
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[in]		vlan	vlan id to remove.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_ppio_remove_vlan(struct pp2_ppio *ppio, u16 vlan);

/**
 * Flush ppio filtering vlan id's
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_ppio_flush_vlan(struct pp2_ppio *ppio);

/**
 * Get ppio statistics
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[out]		stats	Port statistics.
 * @param[in]		reset	A flag indicates if counters should be reset.
 *
 */
int pp2_ppio_get_statistics(struct pp2_ppio *ppio, struct pp2_ppio_statistics *stats, int reset);

/**
 * Get link state
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[out]		en	link enabled.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_ppio_get_link_state(struct pp2_ppio *ppio, int *en);

/**
 * TODO : Set link state
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[in]		en	link enabled.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
/* int pp2_ppio_set_link_state(struct pp2_ppio *ppio, int en); */

/**
 * Set outq state
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[in]		qid	Out-Q id.
 * @param[in]		en	A flag indicates if queue should be
 *				enabled or disabled.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */

int pp2_ppio_set_outq_state(struct pp2_ppio *ppio, u8 qid, int en);
/**
 * Get outq state
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[in]		qid	Out-Q id.
 * @param[out]		en	A flag indicates if queue is
 *				enabled or disabled.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_ppio_get_outq_state(struct pp2_ppio *ppio, u8 qid, int *en);

/**
 * Enable/disable rx flow control
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[in]		en	A flag indicates if rx pause should be
 *				enabled or disabled.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_ppio_set_rx_pause(struct pp2_ppio *ppio, int en);

/**
 * Get rx flow control status
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[out]		en	A flag indicates if rx pause is
 *				enabled or disabled.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_ppio_get_rx_pause(struct pp2_ppio *ppio, int *en);

/**
 * Serialize the ppio parameters
 *
 * The serialization API is called by the 'master' user application to serialize a local ppio object.
 * The output string is created in a JSON format.
 * Below is how a ppio config-string looks like:
 *	ppio-<pp2_id>:<port-id>: {
 *	iomap_filename: <str>,	(TBD)
 *	pp2_id: <int>,
 *	port-id: <int>,
 *	num-in-tcs: <int>,		(used for in QoS according to #priorities)
 *	intc: {
 *		num-inqs : <int>,	(used for in RSS (according to remote side #cores))
 *		inqs : {[<int>>],,[<int>]}, ([q-size])
 *		bpool : <int>
 *	},
 *	
 *	num-out-qs: <int>,	(used for out QoS according to #priorities)
 *	outq: {
 *		outqs : {[<int>],,[<int>]}, ([q-size])
 *		}
 *	}
 *
 * The guest application can then access the buffer ppio object, and retrieve the ppio config string
 *
 * @param[in]	ppio		A ppio handle.
 * @param[in]	buff		Pointer to a buffer where the resulting string is stored.
 *				The buffer should have a size of at least 'size' characters.
 * @param[in]	size		size of buffer.
 *
 * @retval	The number of characters that would have been written if 'size' had been sufficiently large
 * @retval	<0 on failure
 */
int pp2_ppio_serialize(struct pp2_ppio *ppio, char buff[], u32 size);

/**
 * Probe a ppio
 *
 * The probe API should be called by the user application to create the buffer-ppio object for a guest application.
 *
 * @param[in]	match		The matching string to search for in the Buffer pool object.
 * @param[in]	buff		Buffer ppio object.
 * @param[out]	ppio		ppio structure containing the results of the match
 *
 * @retval      0 on success
 * @retval      <0 on failure
 */
int pp2_ppio_probe(char *match, char *buff, struct pp2_ppio **ppio);

/**
 * Remove a ppio
 *
 * @param[in]	ppio		ppio structure to remove
 *
 * @retval      0 on success
 * @retval      <0 on failure
 */
int pp2_ppio_remove(struct pp2_ppio *ppio);

/**
 * Create a ppio rx event
 *
 * The rx_event API is called to create a sys_event for a ppio, that
 * can later be polled through the mv_sys_event_poll() API.
 *
 * @param[in]	ppio		A pointer to a PP-IO object.
 * @param[in]	params		Parameters for the event.
 * @param[out]	ev		A pointer to event handle of type 'struct mv_sys_event *'.
 *
 * @retval      0 on success
 * @retval      <0 on failure
 */
int pp2_ppio_rx_create_event(struct pp2_ppio *ppio, struct pp2_ppio_rxq_event_params *params, struct mv_sys_event **ev);

/**
 * Delete a ppio rx event
 *
 * @param[in]	ev		A sys_event handle.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 */
int pp2_ppio_rx_delete_event(struct mv_sys_event *ev);

/**
 * Set a ppio rx event
 *
 * The rx_set_event API is called to enable the creation of events for the related ppio.
 *
 * @param[in]	ev		A sys_event handle.
 * @param[in]	en		1 - enable, 0 - disable.
 *
 * @retval      0 on success
 * @retval      <0 on failure
 */
int pp2_ppio_rx_set_event(struct mv_sys_event *ev, int en);


/** @} */ /* end of grp_pp2_io */

#endif /* __MV_PP2_PPIO_H__ */
