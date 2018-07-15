/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#ifndef _GIU_MNG_DESC_H_
#define _GIU_MNG_DESC_H_

/*
 * Management descriptors definitions.
 */

#define MGMT_DESC_DATA_LEN		(56)

enum cmd_dest_type {
	CDT_INVALID = 0,
	CDT_PF,
	CDT_VF,
	CDT_CUSTOM
};

/*
 * app_code - Define the list of the different command receivers at the NIC
 * side, or the notification receive at the host side.
 */
enum app_codes {
	AC_HOST_SNIC_NETDEV	= 0X1,
	AC_PF_MANAGER,

	APP_CODE_LAST		= 0XFFFF,
};

/*
 * aos_cmd_codes - Define the list of commands that can be set from the host to
 * the SNIC.
 */
enum cmd_codes {
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
	CC_PF_LOOPBACK,
	CC_PF_ADD_VLAN,
	CC_PF_REMOVE_VLAN,
	CC_PF_GET_GP_STATS,
	CC_PF_GET_GP_QUEUE_STATS,
	CC_PF_MC_ADD_ADDR,
	CC_PF_MC_REMOVE_ADDR,
	CC_PF_MAC_FLUSH,
	CC_PF_LINK_INFO,
	CMD_CODE_LAST = 0XFF,
};

enum notif_codes {
	NC_PF_LINK_CHANGE = 0x1,
	NC_PF_KEEP_ALIVE = 0x2,

	NOTIF_CODE_LAST = 0XFF,
};

/* Relevant only for pf_init command. */
enum egress_sched {
	ES_STRICT_SCHED = 0x1,
	ES_WRR_SCHED
};

/* Relevant only for ingress_tc_add command. */
enum ingress_hash_type {
	ING_HASH_TYPE_2_TUPLE = 0x1,
	ING_HASH_TYPE_5_TUPLE
};

#define SET_MGMT_CMD_FIELD(word, val, off, msk)	(word = ((word & ~(msk << off)) | ((val & msk) << off)))
#define GET_MGMT_CMD_FIELD(word, off, msk)	((word >> off) & msk)

#define MAC_ADDR_LEN	6
/*
 * mgmt_cmd - Encapsulates all management control commands parameters.
 */
/* Make sure structure is portable along different systems. */
struct mgmt_cmd_params {
	union {
		struct {
			u32	num_host_egress_tc;
			u32	num_host_ingress_tc;
			u16	mtu_override;
			u16	mru_override;
			u8	egress_sched; /* enum aos_egress_sched */
		} __packed pf_init;

		struct {
			u32	tc_prio;
			u32	num_queues_per_tc;
		} __packed pf_egress_tc_add;

		/* Used for BM & Tx queues. */
		struct {
			u64	q_phys_addr;
			u32	q_len;
			u64	q_cons_phys_addr;
			u32	q_wrr_weight;
			u32	tc_prio; /* irrelevant for BM. */
			u32	msix_id;
		} __packed pf_egress_q_add;

		struct {
			u32	tc_prio;
			u32	num_queues_per_tc;
			u32	pkt_offset;
			u8	hash_type; /* enum aos_ingress_hash_type */
		} __packed pf_ingress_tc_add;

		struct {
			u64	q_phys_addr;
			u64	q_prod_phys_addr;
			u64	bpool_q_phys_addr;
			u64	bpool_q_cons_phys_addr;
			u32	q_len;
			u32	msix_id;
			u32	tc_prio;
			u32	q_buf_size;
		} __packed pf_ingress_data_q_add;

		struct {
			u8	reset;
		} __packed pf_get_statistics;

		struct {
			u32	mtu;
		} __packed pf_set_mtu;

		struct {
			u8 loopback;
		} __packed pf_set_loopback;

		struct {
			u16 vlan;
		} __packed pf_vlan;

		struct {
			u8 uc;
			u8 mc;
		} __packed pf_flush_addr;

		struct {
			u8 out;
			u8 tc;
			u8 qid;
			u8 reset;
		} __packed pf_q_get_statistics;

		/* CC_PF_MAC_ADDR */
		u8 mac_addr[MAC_ADDR_LEN];

		/* CC_PF_PROMISC */
#define AGNIC_PROMISC_ENABLE		(1)
#define AGNIC_PROMISC_DISABLE		(0)
		u8 promisc;

		/* CC_PF_MC_PROMISC */
#define AGNIC_MC_PROMISC_ENABLE		(1)
#define AGNIC_MC_PROMISC_DISABLE	(0)
		u8 mc_promisc;
	};
} __packed;

/*
 * mgmt_cmd_resp - Encapsulates the different responses that can be
 * received from the SNIC as a result of a management command.
 * status - Command execution status (0 - Ok, 1 - Fail).
 */
/* Make sure structure is portable along different systems. */
struct mgmt_cmd_resp {
#define NOTIF_STATUS_OK	(0)
#define NOTIF_STATUS_FAIL	(1)
	u8 status;
	union {
		/* Use same response structure for all Q add operations.
		 * The prod_cons_phys_addr wil hold either the consumer or the
		 * producers address depending on the type of queue being
		 * created (ingress or egress).
		 */
		struct {
			u64	q_prod_cons_phys_addr;
			u64	bpool_q_prod_cons_phys_addr;
		} __packed q_add_resp;

		/* CC_PF_LINK_STATUS */
		u32 link_status;

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
		} __packed agnic_stats;

		struct {
			u64 gp_rx_packets;
			u64 gp_tx_packets;
		} __packed gp_stats;

		struct {
			u64 packets;
		} __packed gp_queue_stats;

		/* CC_PF_LINK_INFO */
		struct {
			u8  link_up;
			u32 speed;
			u32 duplex;
			u32 phy_mode;
		} __packed link_info;
	};
} __packed;

/*
 * mgmt_notification - Encapsulates the different notifications that can be
 * received from the SNIC.
 */
/* Make sure structure is portable along different systems. */
struct mgmt_notification {
	union {
		/* NC_PF_LINK_CHANGE */
		u32 link_status;

		/* NC_PF_KEEP_ALIVE */
#define MGMT_NOTIF_KEEP_ALIVE_FW	(0x1)
#define MGMT_NOTIF_KEEP_ALIVE_APP	(0x2)
		u32 keep_alive;
	};
} __packed;

/* Command Descriptor
 * cmd_idx - Command Identifier, this field will be copied to the response
 *   descriptor by the SNIC, in order to correlate the response with the command.
 *	value 0xFFFF indicate a notification message.
 * app_code - Target application Id (out of enum snic_app_codes)
 * cmd_code - Command to be executed (out of enum snic_cmd_codes)
 * client_id - Destination ID – PF / VF Id
 * client_type - Destination type – PF / VF
 * flags - Bitmask of CMD_FLAGS_XX.
 * cmd_params/resp_data
 *     Array of bytes, holding the serialized parameters/response list for a specific command.
 */
/* Make sure structure is portable along different systems. */
struct cmd_desc {
#define CMD_ID_ILLEGAL			0
#define CMD_ID_NOTIFICATION		0xFFFF
	u16 cmd_idx;
	u16 app_code;
	u8 cmd_code;
	u8 client_id;
	u8 client_type;
	u8 flags;

	u8 data[MGMT_DESC_DATA_LEN];
} __packed;

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
#endif /* _GIU_MNG_DESC_H_ */
