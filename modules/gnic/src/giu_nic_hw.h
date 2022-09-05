/*
** Copyright (c) 2015 Marvell.
** This program is free software: you can redistribute it and/or
** modify it under the terms of the GNU General Public License as
** published by the Free Software Foundation, either version 2 of the
** License, or any later version.
**
** This program is distributed in the hope that it will be useful, but
** WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
** General Public License for more details.
*/

/* This file includes the definitions of all data structures that should be
 * shared with the various drivers and modules on the Marvell Armada GIU NIC side.
 * The file should be copied AS-IS, in order to make sure that data-structures
 * on both sides are 100% aligned.
 */

#ifndef _ARMADA_GIU_NIC_HW_H_
#define _ARMADA_GIU_NIC_HW_H_

/*
** AGNIC Configuration space definition
*/

/*
** AGNIC configuration structure defined by the device, and used for
** initial communication between the host driver and the NIC.
** The below is still preliminary, till we finalize the interface between
** the host and the NIC.
** - q_addr: Physical address of the queue in host's memory.
** - q_prod_offs: Producer offset from BAR.
** - q_cons_offs: Consumer offset from BAR.
** - len: Number of elements in the queue.
*/
#pragma pack(1)
struct agnic_q_hw_info {
	u64	q_addr;
	u32	q_prod_offs;
	u32	q_cons_offs;
	u32	len;
	u32	res;
};

struct agnic_config_mem {
#define AGNIC_CFG_STATUS_DEV_READY		(0x1)
#define AGNIC_CFG_STATUS_HOST_MGMT_READY	(0x2)
#define AGNIC_CFG_STATUS_DEV_MGMT_READY		(0x4)
	u32	status;
	u8	mac_addr[6];
	u8	res1[6];
	struct agnic_q_hw_info cmd_q;
	struct agnic_q_hw_info notif_q;
	u8	res2[24];

	u32	dev_use_size;
	/* MSI-X table offset at BAR0 */
	u32     msi_x_tbl_offset;

	u8	res3[920]; /* complete to 1KB */
};
#pragma pack()

/*
 * Management descriptors definitions.
 */

#define AGNIC_MGMT_DESC_DATA_LEN		(56)

enum agnic_cmd_dest_type {
	CDT_INVALID = 0,
	CDT_PF,
	CDT_VF,
	CDT_CUSTOM
};

/*
 * agnic_app_code - Define the list of the different command receivers at the NIC
 * side, or the notification receive at the host side.
 */
enum agnic_app_codes {
	AC_HOST_AGNIC_NETDEV	= 0X1,
	AC_PF_MANAGER,

	APP_CODE_LAST		= 0XFFFF,
};

struct rate_limit_params {
	u32 cbs;	/* committed_burst_size, in kilobytes. Min: 64kB */
	u32 cir;	/* committed_information_rate, in kilobits per second. Min: 100kbps */
};

/*
 * agnic_cmd_codes - Define the list of commands that can be set from the host to
 * the agnic.
 */
enum agnic_cmd_codes {
	CC_PF_INIT = 0x1,
	CC_PF_INIT_DONE,
	CC_PF_EGRESS_TC_ADD,
	CC_PF_EGRESS_DATA_Q_ADD,
	CC_PF_INGRESS_TC_ADD,
	CC_PF_INGRESS_DATA_Q_ADD,
	CC_PF_ENABLE,
	CC_PF_DISABLE,

	CC_PF_MGMT_ECHO,
	CC_PF_LINK_STATUS,
	CC_PF_GET_STATISTICS,
	CC_PF_CLOSE,
	CC_PF_MAC_ADDR,
	CC_PF_PROMISC,
	CC_PF_MC_PROMISC,
	CC_PF_MTU,
	CC_PF_SET_LOOPBACK,
	CC_PF_ADD_VLAN,
	CC_PF_REMOVE_VLAN,
	CC_PF_GET_GP_STATS,
	CC_PF_GET_GP_QUEUE_STATS,
	CC_PF_MC_ADD_ADDR,
	CC_PF_MC_REMOVE_ADDR,
	CC_PF_MAC_FLUSH,
	CC_PF_LINK_INFO,
	CC_PF_PAUSE_SET,
	CC_PF_PAUSE_GET,
	CC_PF_PORT_RATE_LIMIT,
	CC_PF_QUEUE_RATE_LIMIT,
	CMD_CODE_LAST = 0XFF,
};

enum agnic_notif_codes {
	NC_PF_LINK_CHANGE = 0x1,
	NC_PF_KEEP_ALIVE  = 0x2,

	NOTIF_CODE_LAST = 0XFF,
};

/* Relevant only for pf_init command. */
enum agnic_egress_sched {
	ES_STRICT_SCHED = 0x1,
	ES_WRR_SCHED
};

/* Relevant only for ingress_tc_add command. */
enum agnic_ingress_hash_type {
	ING_HASH_TYPE_NONE = 0x0,
	ING_HASH_TYPE_2_TUPLE,
	ING_HASH_TYPE_5_TUPLE,
	ING_HASH_LAST
};

/* For performance considerations, assume descriptor fields where zeroed before
 * setup.
 */
#define SET_FIELD(word, val, off, msk)	((word) |= ((val & msk) << (off)))
#define GET_FIELD(word, off, msk)	((word >> off) & msk)



/* Transmit Descriptor
 * The below fields where defined according to the agnic spec.
 * Because of portaiblity issues, we cannot use bitfields for structure members
 * (different compilers represent bitfields differently), so bitfields where
 * grouped into 1/2Byte elements, with proper flags / masks.
 */

enum agnic_dec_ipv4_l4_err {
	DESC_ERR_OK = 0x0,
	DESC_ERR_IPV4_HDR,      /* L3 IPv4 Header error */
	DESC_ERR_L4_CHECKSUM,   /* L4 checksum error */
	DESC_ERR_IPV4_CHECKSUM, /* IPv4 checksum error */
};

/* Byte 0 */
#define TXD_FLAGS_L3_OFFSET_SET(flags, val)		SET_FIELD(flags, val, 0, 0x7F)
#define TXD_FLAGS_L3_OFFSET_GET(flags, val)		GET_FIELD(flags, 0, 0x7F)
#define TXD_FLAGS_BUF_MODE				(1 << 7)

/* Byte 1 */
#define TXD_FLAGS_IP_HDR_LEN_SET(flags, val)		SET_FIELD(flags, val, 8, 0x1F)
#define TXD_FLAGS_IP_HDR_LEN_GET(flags, val)		GET_FIELD(flags, 8, 0x1F)
#define TXD_FLAGS_GEN_L4_CSUM_MODE_SET(flags, val)	SET_FIELD(flags, val, 13, 0x3)
#define TXD_FLAGS_GEN_L4_CSUM_MODE_GET(flags)		GET_FIELD(flags, 13, 0x3)
#define TXD_FLAGS_GEN_L4_CSUM_MASK			(0x3 << 13)
#define TXD_FLAGS_GEN_L4_CSUM_EN			(0 << 13)
#define TXD_FLAGS_GEN_L4_CSUM_PARTIAL			(1 << 13)
#define TXD_FLAGS_GEN_L4_CSUM_NOT			(2 << 13)
#define TXD_FLAGS_GEN_IPV4_CSUM_DIS			(1 << 15)

/* Byte 2 */
#define TXD_FLAGS_POOL_IDX_SET(flags, val)	SET_FIELD(flags, val, 16, 0x1F)
#define TXD_FLAGS_POOL_IDX_GET(flags, val)	GET_FIELD(flags, 16, 0x1F)
#define TXD_FLAGS_MD_MODE			(1 << 22)
#define TXD_FLAGS_PADDING_DIS			(1 << 23)

/* Byte 3 */
#define TXD_FLAGS_L4_TYPE_GET(flags)		GET_FIELD(flags, 24, 0x3)
#define TXD_FLAGS_L4_TCP			(0 << 24)
#define TXD_FLAGS_L4_UDP			(1 << 24)
#define TXD_FLAGS_L4_OTHER			(2 << 24)
#define TXD_FLAGS_L3_INFO_GET(flags)		GET_FIELD(flags, 26, 0x3)
#define TXD_FLAGS_L3_INFO_IPV4			(0 << 26)
#define TXD_FLAGS_L3_INFO_IPV6			(1 << 26)
#define TXD_FLAGS_L3_INFO_OTHER			(2 << 26)
#define TXD_FLAGS_LAST				(1 << 28)
#define TXD_FLAGS_FIRST				(1 << 29)

#pragma pack(1)
struct agnic_tx_desc {
	/* 0x0 - 0x3
	 * fields order: msb ... lsb
	 * Byte 0: res:1 | l3_offset:7
	 * Byte 1: gen_ip4_csum:1 | gen_l4_csum:2 | ip_hdr_len:5
	 * Byte 2: l2_pad_en:1 | res2:1 | buf_release_mode:1 | pool_idx:5
	 * Byte 3: res3:2 | last:1 | first:1 | l3_info:2 | l4_type:2
	 */
	u32 flags;

	/* 0x4 - 0x7
	 * fields order: msb ... lsb
	 * Byte 0:   pkt_offset
	 * Byte 1:   vlan_info:2 | res4:6
	 * Byte 2-3: byte_cnt
	 */
	u8 pkt_offset;
	u8 res4: 6;
#define TX_VLAN_TAG_NONE	0 /* No VLANs */
#define TX_VLAN_TAG_SINGLE	1 /* Single VLAN */
#define TX_VLAN_TAG_DOUBLE	2 /* Double VLANs */
#define TX_VLAN_TAG_RESERVE	3 /* Reserve */
	u8 vlan_info: 2;
	u16 byte_cnt;

	/* 0x8 - 0xB */
	u16 res5;
	u16 l4_csum;

	/* 0xC - 0xF */
	u32 res6;

	/* 0x10 - 0x17 */
	u64 buffer_addr;

	/* 0x18 - 0x1F */
	u64 cookie;
};
#pragma pack()

/* Temporary definition: Receive Descriptor
 * The below fields where defined according to the agnic spec.
 * Because of portaiblity issues, we cannot use bitfields for structure members
 * (different compilers represent bitfields differently), so bitfields where
 * grouped into 1/2Byte elements, with proper flags / masks.
 */

enum agnic_rx_desc_l3_type {
	RX_DESC_L3_TYPE_NA = 0,
	RX_DESC_L3_TYPE_IPV4_NO_OPTS,	/* IPv4 with IHL=5, TTL>0 */
	RX_DESC_L3_TYPE_IPV4_OPTS,	/* IPv4 with IHL>5, TTL>0 */
	RX_DESC_L3_TYPE_IPV4_OTHER,	/* Other IPV4 packets */
	RX_DESC_L3_TYPE_IPV6_NO_EXT,	/* IPV6 without extensions */
	RX_DESC_L3_TYPE_IPV6_EXT,	/* IPV6 with extensions */
	RX_DESC_L3_TYPE_ARP,		/* ARP */
	RX_DESC_L3_TYPE_USER_DEFINED	/* User defined */
};

enum agnic_rx_desc_l4_type {
	RX_DESC_L4_TYPE_NA = 0,
	RX_DESC_L4_TYPE_TCP,
	RX_DESC_L4_TYPE_UDP,
	RX_DESC_L4_TYPE_OTHER
};

/* Byte 0 */
#define RXD_FLAGS_L3_OFFSET_SET(flags, val)	SET_FIELD(flags, val, 0, 0x7F)
#define RXD_FLAGS_L3_OFFSET_GET(flags)		GET_FIELD(flags, 0, 0x7F)

/* Byte 1 */
#define RXD_FLAGS_IP_HDR_LEN_SET(flags, val)	SET_FIELD(flags, val, 8, 0x1F)
#define RXD_FLAGS_IP_HDR_LEN_GET(flags)		GET_FIELD(flags, 8, 0x1F)
#define RXD_FLAGS_ERR_CODE_SET(flags, val)	SET_FIELD(flags, val, 13, 0x3)
#define RXD_FLAGS_ERR_CODE_GET(flags)		GET_FIELD(flags, 13, 0x3)
#define RXD_FLAGS_RX_ERROR			(1 << 15)

/* Byte 2 */
#define RXD_FLAGS_POOL_IDX_SET(flags, val)	SET_FIELD(flags, val, 16, 0x1F)
#define RXD_FLAGS_POOL_IDX_GET(flags)		GET_FIELD(flags, 16, 0x1F)
#define RXD_FLAGS_MD_MODE			(1 << 22)
#define RXD_FLAGS_L3_L4_ERR_GET(flags)		GET_FIELD(flags, 23, 0x3)

/* Byte 3 */
#define RXD_FLAGS_L4_TYPE_GET(flags)		GET_FIELD(flags, 25, 0x7)
#define RXD_FLAGS_L4_TCP			(1 << 25)
#define RXD_FLAGS_L4_UDP			(1 << 26)
#define RXD_FLAGS_L3_INFO_GET(flags)		GET_FIELD(flags, 28, 0x7)
#define RXD_FLAGS_L3_IP4			(1 << 28)
#define RXD_FLAGS_L3_IP6			(1 << 30)

#pragma pack(1)
struct agnic_rx_desc {
	/* 0x0 - 0x3
	 * fields order: msb ... lsb
	 * Byte 0: res:1 | l3offset:7
	 * Byte 1: err_sum:1 | err_code:2 | ip_hdr_len:5
	 * Byte 2: res:1 | l4_csum_ok:1 | res:1 | pool_idx:5
	 * Byte 3: res:1 | l3_info:3 | l4_info:3 | ipv4_hdr_err:1
	 */
	u32 flags;

	/* 0x4 - 0x7 */
	u8 pkt_offset;
	/* portnum_dp: port_num:3 | dp:2 | res:3 */
	u8 portnum_dp;
	u16 byte_cnt;

	/* 0x8 - 0xB */
	u16 res4;
	u16 l4_csum;

	/* 0xC - 0xF */
	u32 timestamp_hashkey;

	/* 0x10 - 0x17 */
	u64 buffer_addr;

	/* 0x18 - 0x1F */
	u64 cookie;
};
#pragma pack()

#define SET_MGMT_CMD_FIELD(word, val, off, msk)	(word = ((word & ~(msk << off)) | ((val & msk) << off)))
#define GET_MGMT_CMD_FIELD(word, off, msk)	((word >> off) & msk)

#define AGNIC_MAC_ADDR_LEN	6

/*
 * agnic_mgmt_cmd - Encapsulates all management control commands parameters.
 */
/* Make sure structure is portable along different systems. */
#pragma pack(1)
struct agnic_mgmt_cmd_params {
	union {
		struct {
			u32	num_host_egress_tc;
			u32	num_host_ingress_tc;
			u16	mtu_override;
			u16	mru_override;
			u8	egress_sched; /* enum agnic_egress_sched */
		} pf_init;

		struct {
			u32	tc;
			u32	num_queues;
		} pf_egress_tc_add;

		/* Used for BP & Tx queues. */
		struct {
			u64	q_phys_addr;
			u32	q_prod_offs;
			u32	q_cons_offs;
			u32	q_len;
			u32	q_wrr_weight;
			u32	tc; /* irrelevant for BP. */
			u32	msix_id;
		} pf_egress_q_add;

		struct {
			u32	tc;
			u32	num_queues;
			u32	pkt_offset;
			u8	hash_type; /* enum agnic_ingress_hash_type */
		} pf_ingress_tc_add;

		struct {
			u64	q_phys_addr;
			u32	q_prod_offs;
			u32	q_cons_offs;
			u64	bpool_q_phys_addr;
			u32	bpool_q_prod_offs;
			u32	bpool_q_cons_offs;
			u32	q_len;
			u32	msix_id;
			u32	tc;
			u32	q_buf_size;
		} pf_ingress_data_q_add;

		struct {
			u8	reset;
		} pf_get_statistics;

		struct {
			u8 out;
			u8 tc;
			u8 qid;
			u8	reset;
		} pf_q_get_statistics;

		struct {
			u32	mtu;
		} pf_set_mtu;

		struct {
			u8 loopback;
		} pf_set_loopback;

		struct {
			u16 vlan;
		} pf_vlan;

		struct {
			u8 uc;
			u8 mc;
		} pf_flush_addr;

		struct {
			u8 enable;
			u8 tc;
			u8 qid;
			struct rate_limit_params rate_limit;
		} pf_queue_rate_limit;

		struct {
			u8 enable;
			struct rate_limit_params rate_limit;
		} pf_port_rate_limit;

		/* CC_PF_MAC_ADDR */
		u8 mac_addr[AGNIC_MAC_ADDR_LEN];

		/* CC_PF_PROMISC */
#define AGNIC_PROMISC_ENABLE		(1)
#define AGNIC_PROMISC_DISABLE		(0)
		u8 promisc;

		/* CC_PF_MC_PROMISC */
#define AGNIC_MC_PROMISC_ENABLE		(1)
#define AGNIC_MC_PROMISC_DISABLE	(0)
		u8 mc_promisc;

		/* CC_PF_PAUSE_SET */
		struct {
			u8 rx;
			u8 tx;
		} pause_params;
	};

};
#pragma pack()

enum net_link_speed {
	NET_LINK_SPEED_AN = 0,
	NET_LINK_SPEED_10,
	NET_LINK_SPEED_100,
	NET_LINK_SPEED_1000,
	NET_LINK_SPEED_2500,
	NET_LINK_SPEED_10000,
};

enum net_link_duplex {
	NET_LINK_DUPLEX_AN = 0,
	NET_LINK_DUPLEX_HALF,
	NET_LINK_DUPLEX_FULL
};

enum net_phy_mode {
	NET_PHY_MODE_NONE = 0,
	NET_PHY_MODE_MII,
	NET_PHY_MODE_GMII,
	NET_PHY_MODE_SGMII,
	NET_PHY_MODE_TBI,
	NET_PHY_MODE_REVMII,
	NET_PHY_MODE_RMII,
	NET_PHY_MODE_RGMII,
	NET_PHY_MODE_RGMII_ID,
	NET_PHY_MODE_RGMII_RXID,
	NET_PHY_MODE_RGMII_TXID,
	NET_PHY_MODE_RTBI,
	NET_PHY_MODE_SMII,
	NET_PHY_MODE_XGMII,
	NET_PHY_MODE_MOCA,
	NET_PHY_MODE_QSGMII,
	NET_PHY_MODE_XAUI,
	NET_PHY_MODE_RXAUI,
	NET_PHY_MODE_KR,
	NET_PHY_MODE_OUT_OF_RANGE,
};

/*
 * agnic_mgmt_cmd_resp - Encapsulates the different responses that can be
 * received from the agnic as a result of a management command.
 * status - Command execution status (0 - Ok, 1 - Fail, 0xFF - Notification).
 */
/* Make sure structure is portable along different systems. */
#pragma pack(1)
struct agnic_mgmt_cmd_resp {
#define AGNIC_NOTIF_STATUS_OK		(0)
#define AGNIC_NOTIF_STATUS_FAIL		(1)
	u8 status;
	union {
		/* Use same response structure for all Q add operations. */
		struct {
#define AGNIC_Q_INF_STATUS_OK		(0)
#define AGNIC_Q_INF_STATUS_ERR		(1)
			u64	q_inf;
			u64	bpool_q_inf;
		} q_add_resp;

		/* CC_PF_LINK_STATUS */
		u32 link_status;

		/* CC_PF_PP2_STATISTICS */
		struct {
			u64 rx_bytes;
			u64 rx_packets;
			u64 rx_unicast_packets;
			u64 rx_errors;
			u64 rx_fullq_dropped;
			u64 rx_bm_dropped;
			u64 rx_early_dropped;
			u64 rx_fifo_dropped;
			u64 rx_cls_dropped;
			u64 tx_bytes;
			u64 tx_packets;
			u64 tx_unicast_packets;
			u64 tx_errors;
		} agnic_stats;

		struct {
			u64 gp_rx_packets;
			u64 gp_rx_fullq_dropped;
			u64 gp_tx_packets;
		} gp_stats;
		struct {
			u64 q_packets;
		} q_stats;

		/* CC_PF_LINK_INFO */
		struct {
			u8  link_up;
			u32 speed;
			u32 duplex;
			u32 phy_mode;
		} link_info;

		/* CC_PF_PAUSE_GET */
		struct {
			u8 rx;
			u8 tx;
		} pause_params;
	};
};
#pragma pack()

/*
 * agnic_mgmt_notification - Encapsulates the different notifications that can be
 * received from the SNIC.
 */
/* Make sure structure is portable along different systems. */
#pragma pack(1)
struct agnic_mgmt_notification {
	union {
		/* NC_PF_LINK_CHANGE */
		u32 link_status;

		/* NC_PF_KEEP_ALIVE */
#define MGMT_NOTIF_KEEP_ALIVE_FW        (0x1)
#define MGMT_NOTIF_KEEP_ALIVE_APP       (0x2)
		u32 keep_alive;
	};
};
#pragma pack()

/* Command Descriptor
 * cmd_idx - Command Identifier, this field will be copied to the response
 *   descriptor by the agnic, in order to correlate the response with the command.
 *	value 0xFFFF indicate a notification message.
 * app_code - Target application Id (out of enum agnic_app_codes)
 * cmd_code - Command to be executed (out of enum agnic_cmd_codes)
 * client_id - Destination ID – PF / VF Id
 * client_type - Destination type – PF / VF
 * flags - Bitmask of CMD_FLAGS_XX.
 * cmd_params/resp_data
 *     Array of bytes, holding the serialized parameters/response list for a specific command.
 */
/* Make sure structure is portable along different systems. */
#pragma pack(1)
struct agnic_cmd_desc {
#define CMD_ID_ILLEGAL			0
#define CMD_ID_NOTIFICATION		0xFFFF
	u16 cmd_idx;
	u16 app_code;
	u8 cmd_code;
	u8 client_id;
	u8 client_type;
	u8 flags;

	u8 data[AGNIC_MGMT_DESC_DATA_LEN];
};
#pragma pack()

/* indicates whether the descriptor is consturcted from multiple ones */
#define CMD_FLAGS_NUM_EXT_DESC_MASK		0x1F
#define CMD_FLAGS_NUM_EXT_DESC_SHIFT		0

#define CMD_FLAGS_NUM_EXT_DESC_SET(flags, val)	\
	SET_MGMT_CMD_FIELD(flags, val, CMD_FLAGS_NUM_EXT_DESC_SHIFT, CMD_FLAGS_NUM_EXT_DESC_MASK)
#define CMD_FLAGS_NUM_EXT_DESC_GET(flags)	\
	GET_MGMT_CMD_FIELD(flags, CMD_FLAGS_NUM_EXT_DESC_SHIFT, CMD_FLAGS_NUM_EXT_DESC_MASK)

/* No response is required for this cmd */
#define CMD_FLAGS_NO_RESP_MASK			0x1
#define CMD_FLAGS_NO_RESP_SHIFT			5
#define CMD_FLAGS_NO_RESP_SET(flags, val)	\
	SET_MGMT_CMD_FIELD(flags, val, CMD_FLAGS_NO_RESP_SHIFT, CMD_FLAGS_NO_RESP_MASK)
#define CMD_FLAGS_NO_RESP_GET(flags)		\
	GET_MGMT_CMD_FIELD(flags, CMD_FLAGS_NO_RESP_SHIFT, CMD_FLAGS_NO_RESP_MASK)

/* Indicates position of the command buffer- inline first, last or external buffer. */
#define CMD_FLAGS_BUF_POS_MASK			0x3
#define CMD_FLAGS_BUF_POS_SHIFT			6
#define CMD_FLAGS_BUF_POS_SET(flags, val)	\
	SET_MGMT_CMD_FIELD(flags, val, CMD_FLAGS_BUF_POS_SHIFT, CMD_FLAGS_BUF_POS_MASK)
#define CMD_FLAGS_BUF_POS_GET(flags)		\
	GET_MGMT_CMD_FIELD(flags, CMD_FLAGS_BUF_POS_SHIFT, CMD_FLAGS_BUF_POS_MASK)

#define CMD_FLAG_BUF_POS_SINGLE		0 /*	CMD_params is inline and this is a single buffer descriptor;
					   *	i.e. the parameters for this command are inline.
					   */
#define CMD_FLAG_BUF_POS_FIRST_MID	1 /*	CMD_params is inline and this is the first or a middle buffer out of a
					   *	sequence of buffers to come.
					   */
#define CMD_FLAG_BUF_POS_LAST		2 /*	CMD_params is inline and this is the last buffer out of a sequence that
					   *	previously arrived.
					   */
#define CMD_FLAG_BUF_POS_EXT_BUF	3 /*	CMD_params is a pointer to an external buffer; i.e. the parameters for
					   *	this command reside in external buffer.
					   */

/* Buffers Pool Descriptor */
#pragma pack(1)
struct agnic_bpool_desc {
	u64 buff_addr_phys;
	u64 buff_cookie;
};
#pragma pack()

#endif /* _ARMADA_GIU_NIC_HW_H_ */
