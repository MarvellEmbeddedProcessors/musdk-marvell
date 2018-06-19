/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#ifndef __AGNIC_PFIO_HW_H__
#define __AGNIC_PFIO_HW_H__

#define AGNIC_CONFIG_BAR_SIZE	(sizeof(struct agnic_config_bar))
#define AGNIC_CONFIG_BAR_ID	0

/*
** AGNIC Configuration space definition
*/

/*
** AGNIC configuration structure defined by the device, and used for
** initial communication between the host driver and the NIC.
** The below is still preliminary, till we finalize the interface between
** the host and the NIC.
** - q_addr: Physical address of the queue in host's memory.
** - consumer_idx_addr: Physical address of the queue consumer index.
** - producer_idx_addr: Physical address of the queue producer index.
** - len: Number of elements in the queue.
*/
struct agnic_q_hw_info {
	u64	q_addr;
	u64	consumer_idx_addr;
	u64	producer_idx_addr;
	u32	len;
	u32	res;
} __packed;

struct agnic_config_mem {
#define AGNIC_CFG_STATUS_DEV_READY		(0x1)
#define AGNIC_CFG_STATUS_HOST_MGMT_READY	(0x2)
#define AGNIC_CFG_STATUS_DEV_MGMT_READY		(0x4)
	u32	status;
	u8	mac_addr[6];
	u8	res[6];
	struct agnic_q_hw_info cmd_q;
	struct agnic_q_hw_info notif_q;
	/* Meanwhile, assume agnic's notification table is part of BAR-0.
	 * This is actually the offset of the prod / cons notification tables
	 * inside BAR0.
	 * Value N, meanes that the respective notification table starts at
	 * offset N-Bytes from the beginning of BAR-0.
	 * Consecutively, the _size parameter holds the size in bytes of the
	 * notification tables.
	 */
	u32	cons_notif_tbl_offset;
	u32	cons_notif_tbl_size;
	u32	prod_notif_tbl_offset;
	u32	prod_notif_tbl_size;
	/* This parameter defines where ring index are placed
	 * It can be set as remote = device memory, or
	 * It can be set as local = notification tables area
	 */
	u32	remote_index_location;

	/* MSI-X table offset at BAR0 */
	u32     msi_x_tbl_offset;
} __packed;

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

#define SET_MGMT_CMD_FIELD(word, val, off, msk)	(word = ((word & ~(msk << off)) | ((val & msk) << off)))
#define GET_MGMT_CMD_FIELD(word, off, msk)	((word >> off) & msk)

#define AGNIC_MAC_ADDR_LEN	6
/*
 * agnic_mgmt_cmd - Encapsulates all management control commands parameters.
 */
/* Make sure structure is portable along different systems. */
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
			u32	q_len;
			u64	q_cons_phys_addr;
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
			u64	q_prod_phys_addr;
			u64	bpool_q_phys_addr;
			u64	bpool_q_cons_phys_addr;
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
	};
} __packed;

/*
 * agnic_mgmt_cmd_resp - Encapsulates the different responses that can be
 * received from the agnic as a result of a management command.
 * status - Command execution status (0 - Ok, 1 - Fail, 0xFF - Notification).
 */
/* Make sure structure is portable along different systems. */
struct agnic_mgmt_cmd_resp {
#define AGNIC_NOTIF_STATUS_OK		(0)
#define AGNIC_NOTIF_STATUS_FAIL		(1)
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
			u64 gp_tx_packets;
		} gp_stats;
		struct {
			u64 q_packets;
		} q_stats;
	};
} __packed;

/*
 * agnic_mgmt_notification - Encapsulates the different notifications that can be
 * received from the SNIC.
 */
/* Make sure structure is portable along different systems. */
struct agnic_mgmt_notification {
	union {
		/* NC_PF_LINK_CHANGE */
		u32 link_status;
	};
} __packed;

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

/* Buffers Pool Descriptor */
struct agnic_bpool_desc {
	u64 buff_addr_phys;
	u64 buff_cookie;
} __packed;

#endif /* __AGNIC_PFIO_HW_H__ */
