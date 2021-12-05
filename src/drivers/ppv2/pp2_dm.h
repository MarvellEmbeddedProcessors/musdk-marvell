/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file pp2_dm.h
 *
 * Packet Processor Descriptor Manager
 *
 * Presentation API for PPDK egress and ingress descriptors
 */

#ifndef _PP2_DM_H_
#define _PP2_DM_H_

/* Forward declaration for the PPDK handle */
struct pp2;

/**
 * struct pp2_desc
 *
 * Descriptor Manager 32-byte structure DMA layout. Holds run-time
 * data per each descriptor set by egress requestor (TX descriptor)
 * or by packet processor (RX descriptor).
 *
 * NOTE: Indirectly constructed and accessed by clients using the
 * below TX/RX descripto set of macros.
 *
 * @field   cmd0   See TX/RX descriptor macros below for details
 * @field   cmd1   See TX/RX descriptor macros below for details
 * @field   cmd2   See TX/RX descriptor macros below for details
 * @field   cmd3   See TX/RX descriptor macros below for details
 * @field   cmd4   See TX/RX descriptor macros below for details
 * @field   cmd5   See TX/RX descriptor macros below for details
 * @field   cmd6   See TX/RX descriptor macros below for details
 * @field   cmd7   See TX/RX descriptor macros below for details
 *
 */
struct pp2_desc {
	u32 cmd0;
	u32 cmd1;
	u32 cmd2;
	u32 cmd3;
	u32 cmd4;
	u32 cmd5;
	u32 cmd6;
	u32 cmd7;
};

/**
 * RxDesc structure that maps on HW controlled RxDesc
 */
struct pp2_rx_desc {
	union {
	struct {
		uint8_t l3_offset : 7;
		uint8_t rsrv : 1;

		uint8_t ip_hdr_len : 5;
		uint8_t ec : 2;
		uint8_t es : 1;

		uint8_t pool_id : 4;
		uint8_t rsrv2 : 1;
		uint8_t hwf_sync : 1;
		uint8_t l4chk_ok : 1;
		uint8_t ip_frg : 1;

		uint8_t ipv4_hdr_err : 1;
		uint8_t l4_info : 3;
		uint8_t l3_info : 3;
		uint8_t buffer_header : 1;
	};
	u32 status;		 /* info about received packet */
	};

	union {
	struct {
		uint16_t lookup_id : 6;
		uint16_t pars_info : 10;
	};
	u16 rsrvd_parser;	/* parser_info (for future use, PnC) */
	};

	u16 data_size;		/* size of received packet in bytes */

	struct {
		uint16_t gem_port_id : 12;
		uint16_t packet_color : 2;
		uint16_t gop_sop_u : 1;
		uint16_t key_hash_ok : 1;
	};

	u16 l4csum;
	u32 timestamp;

	struct {
		uint64_t buf_phys_addr : 40;
		uint64_t key_hash : 24;
	};

	struct {
		uint64_t buf_virt_addr : 40;
		uint8_t  buffer_qset_no : 7;
		uint8_t  buffer_type : 1;
		uint8_t  mod_dscp : 6;
		uint8_t  mod_pri : 3;
		uint8_t  mdscp : 1;
		uint8_t  mpri : 1;
		uint8_t  mgpid : 1;
		uint8_t  res : 1;
		uint8_t  port_num : 3;
	};
};

/**
 * pp2_dm_if_param
 *
 * Descriptor Manager interface object parameters
 *
 * @field	dm_id	DM-IF ID. This ID should be
 *			assigned uniquely per DM software
 *			egress requestor.
 *
 *			Valid range: [0 - PP2_TOTAL_NUM_REGSPACES]
 *			See <pp2_plat.h>
 *
 * @field	dm_num	Number of transmit descriptors to
 *			allocate. An egress requestor can
 *			assign either a packet or multiple
 *			buffers from a fragmented packet
 *			to a transmit descriptor.
 */
struct pp2_dm_if_param {
	u32  dm_id;
	u32  dm_pp2_id;
	u32  dm_num;
};

/**
 * pp2_dm_if_init
 *
 * Initialize (create) and prepare an exclusive (lockless)
 * DM-IF object for enqueueing packets into a PP-PORT object.
 *
 * @param pp2		The PPDK handle. Based on the param input
 *			the corresponding packet processor shall
 *			be used
 *
 * @param param		Parameters for this DM-IF object
 *
 * @retval		DM-IF object handle on success, NULL otherwise
 */
int pp2_dm_if_init(struct pp2 *pp2, uint32_t dm_id, uint32_t pp2_id, uint32_t num_desc,
		   struct mv_sys_dma_mem_region *mem);

/**
 * pp2_dm_if_deinit
 *
 * Deinitialize (destroy) a DM-IF object. After this
 * routine returns successfully, all transmit descriptors
 * associated with this object shall be destroyed.
 *
 * @param dm_if		DM-IF object handle
 *
 * @retval		0 on success, error otherwise
 */
void pp2_dm_if_deinit(struct pp2 *pp2, uint32_t dm_id, uint32_t pp2_id);

/**
 * pp2_dm_if_get_desc
 *
 * After getting an DM-IF object, the egress requestor
 * must call this routine in order to check if enough
 * transmit descriptors are available for holding the
 * pre-enqueue packets and if positive, the first available
 * descriptor from the descriptor array associated with this
 * DM-If object shall be returned.
 *
 * NOTE: Client should check actual number of descriptors
 * returned, which can be lower than the number requested.
 * In this case, a second enqueue should be done afterwards
 * for the residual descriptors.
 *
 * NOTE: After this call successfully returns, client has
 * responsability for populating the descriptors with relevant
 * packet data by using the below setters/getters presented below.
 *
 * @param  dm_if	DM-IF object handle
 *
 * @output out_desc	Egress descriptor array
 *
 * @param  req_txds	Number of requested TX descriptors. This
 *			number usually reflects the number of packets
 *			or buffers
 *
 * @retval		Number of valid descriptors in the @out_desc array
 */
uint32_t pp2_dm_if_get_desc(struct pp2_dm_if *dm_if, struct pp2_desc **out_desc, uint32_t req_desc);

void pp2_dm_desc_dump(struct pp2_desc *desc);

static inline
struct pp2_desc *pp2_dm_if_next_desc_get(struct pp2_dm_if *dm_if)
{
	u32 tx_desc = dm_if->desc_next_idx;

	dm_if->desc_next_idx = ((unlikely(tx_desc == (dm_if->desc_total - 1))) ? 0 : (tx_desc + 1));

	return dm_if->desc_virt_arr + tx_desc;
}

static inline
struct pp2_desc *pp2_dm_if_next_desc_block_get(struct pp2_dm_if *dm_if, uint16_t num_desc, uint16_t *cont_desc)
{
	u32 tx_desc = dm_if->desc_next_idx;

	if (unlikely(num_desc >= (dm_if->desc_total - dm_if->desc_next_idx))) {
		*cont_desc = dm_if->desc_total - dm_if->desc_next_idx;
		dm_if->desc_next_idx = 0;
	} else {
		dm_if->desc_next_idx = tx_desc + num_desc;
		*cont_desc = num_desc;
	}

	return dm_if->desc_virt_arr + tx_desc;
}

/* TX Descriptor Getters */
#define DM_TXD_GET_L3_OFF(desc)		(((desc)->cmds[0] & TXD_L3_OFF_MASK) >> 0)
#define DM_TXD_GET_BUFMODE(desc)	(((desc)->cmds[0] & TXD_BUFMODE_MASK) >> 7)
#define DM_TXD_GET_IPHDR_LEN(desc)	(((desc)->cmds[0] & TXD_IPHDR_LEN_MASK) >> 8)
#define DM_TXD_GET_GEN_L4_CHK(desc)	(((desc)->cmds[0] & TXD_GEN_L4_CHK_MASK) >> 13)
#define DM_TXD_GET_GEN_IP_CHK(desc)	(((desc)->cmds[0] & TXD_GEN_IP_CHK_MASK) >> 15)
#define DM_TXD_GET_POOL_ID(desc)	(((desc)->cmds[0] & TXD_POOL_ID_MASK) >> 16)
#define DM_TXD_GET_PKT_OFF_EXT(desc)	(((desc)->cmds[0] & TXD_PKT_OFF_EXT_MASK) >> 20)
#define DM_TXD_GET_HWF_SYNC(desc)	(((desc)->cmds[0] & TXD_HWF_SYNC_MASK) >> 21)
#define DM_TXD_GET_HWF(desc)		(((desc)->cmds[0] & TXD_HWF_MASK) >> 22)
#define DM_TXD_GET_PADD_DSBL(desc)	(((desc)->cmds[0] & TXD_PADD_DSBL_MASK) >> 23)
#define DM_TXD_GET_L4_INFO(desc)	(((desc)->cmds[0] & TXD_L4_INFO_MASK) >> 24)
#define DM_TXD_GET_L3_INFO(desc)	(((desc)->cmds[0] & TXD_L3_INFO_MASK) >> 26)
#define DM_TXD_GET_L(desc)		(((desc)->cmds[0] & TXD_L_MASK) >> 28)
#define DM_TXD_GET_F(desc)		(((desc)->cmds[0] & TXD_F_MASK) >> 29)
#define DM_TXD_GET_FORMAT(desc)		(((desc)->cmds[0] & TXD_FORMAT_MASK) >> 30)
#define DM_TXD_GET_BUF_HDR(desc)	(((desc)->cmds[0] & TXD_BUF_HDR_MASK) >> 31)
#define DM_TXD_GET_PKT_OFF(desc)	(((desc)->cmds[1] & TXD_PKT_OFF_MASK) >> 0)
#define DM_TXD_GET_DEST_QID(desc)	(((desc)->cmds[1] & TXD_DEST_QID_MASK) >> 8)
#define DM_TXD_GET_BYTE_COUNT(desc)	(((desc)->cmds[1] & TXD_BYTE_COUNT_MASK) >> 16)
#define DM_TXD_GET_GEM_PID_PTT(desc)	(((desc)->cmds[2] & TXD_GEM_PID_PTT_MASK) >> 0)
#define DM_TXD_GET_DP_MASK(desc)	(((desc)->cmds[2] & TXD_DP_MASK) >> 12)
#define DM_TXD_GET_DSATAG(desc)		(((desc)->cmds[2] & TXD_DSATAG_MASK) >> 14)
#define DM_TXD_GET_L4_INIT_CHK(desc)	(((desc)->cmds[2] & TXD_L4_INIT_CHK_MASK) >> 16)
#define DM_TXD_GET_PME_DPTR(desc)	(((desc)->cmds[3] & TXD_PME_DPTR_MASK) >> 0)
#define DM_TXD_GET_PME_PROGRAM(desc)	(((desc)->cmds[3] & TXD_PME_PROGRAM_MASK) >> 16)
#define DM_TXD_GET_HWF_IDB(desc)	(((desc)->cmds[3] & TXD_HWF_IDB_MASK) >> 24)
#define DM_TXD_GET_GEM_OEM(desc)	(((desc)->cmds[3] & TXD_GEM_OEM_MASK) >> 25)
#define DM_TXD_GET_ERR_SUM(desc)	(((desc)->cmds[3] & TXD_ERR_SUM_MASK) >> 26)
#define DM_TXD_GET_PON_FEC(desc)	(((desc)->cmds[3] & TXD_PON_FEC_MASK) >> 27)
#define DM_TXD_GET_CPU_ID(desc)		(((desc)->cmds[3] & TXD_CPU_ID_MASK) >> 28)
#define DM_TXD_GET_PTP_DESC(desc)	(((desc)->cmds[5] & TXD_PTP_DESC_MASK) >> 8)
#define DM_TXD_GET_QSET_NO(desc)	(((desc)->cmds[7] & TXD_BUF_QSET_NO_MASK) >> 8)
#define DM_TXD_GET_TYPE(desc)		(((desc)->cmds[7] & TXD_BUF_TYPE_MASK) >> 15)
#define DM_TXD_GET_DSCP(desc)		(((desc)->cmds[7] & TXD_MOD_DSCP_MASK) >> 16)
#define DM_TXD_GET_PRI(desc)		(((desc)->cmds[7] & TXD_MOD_PRI_MASK) >> 22)
#define DM_TXD_GET_DSCP_EN(desc)	(((desc)->cmds[7] & TXD_MOD_DSCP_EN_MASK) >> 25)
#define DM_TXD_GET_PRI_EN(desc)		(((desc)->cmds[7] & TXD_MOD_PRI_EN_MASK) >> 26)
#define DM_TXD_GET_GEM_EN(desc)		(((desc)->cmds[7] & TXD_MOD_GEM_EN_MASK) >> 27)
#define DM_TXD_GET_PHYS_LO(desc)	(((desc)->cmds[4] & TXD_BUF_PHYS_LO_MASK) >> 0)
#define DM_TXD_GET_PHYS_HI(desc)	(((desc)->cmds[5] & TXD_BUF_PHYS_HI_MASK) >> 0)
#define DM_TXD_GET_VIRT_LO(desc)	(((desc)->cmds[6] & TXD_BUF_VIRT_LO_MASK) >> 0)
#define DM_TXD_GET_VIRT_HI(desc)	(((desc)->cmds[7] & TXD_BUF_VIRT_HI_MASK) >> 0)
#define DM_TXD_GET_PHYSADDR(desc)	(uintptr_t)(((uint64_t)DM_TXD_GET_PHYS_HI(desc) << 32) | \
					(uint64_t)DM_TXD_GET_PHYS_LO(desc))
#define DM_TXD_GET_VIRTADDR(desc)	(uintptr_t)(((uint64_t)DM_TXD_GET_VIRT_HI(desc) << 32) | \
					(uint64_t)DM_TXD_GET_VIRT_LO(desc))

/* RX Descriptor Getters */
#define DM_RXD_GET_L3_OFF(desc)		(((desc)->cmds[0] & RXD_L3_OFF_MASK) >> 0)
#define DM_RXD_GET_IPHDR_LEN(desc)	(((desc)->cmds[0] & RXD_IPHDR_LEN_MASK) >> 8)
#define DM_RXD_GET_EC(desc)		(((desc)->cmds[0] & RXD_EC_MASK) >> 13)
#define DM_RXD_GET_ES(desc)		(((desc)->cmds[0] & RXD_ES_MASK) >> 15)
#define DM_RXD_GET_POOL_ID(desc)	(((desc)->cmds[0] & RXD_POOL_ID_MASK) >> 16)
#define DM_RXD_GET_HWF_SYNC(desc)	(((desc)->cmds[0] & RXD_HWF_SYNC_MASK) >> 21)
#define DM_RXD_GET_L4_CHK_OK(desc)	(((desc)->cmds[0] & RXD_L4_CHK_OK_MASK) >> 22)
#define DM_RXD_GET_L3_IP_FRAG(desc)	(((desc)->cmds[0] & RXD_L3_IP_FRAG_MASK) >> 23)
#define DM_RXD_GET_L3_IP4_HDR_ERR(desc)	(((desc)->cmds[0] & RXD_L3_IP4_HDR_ERR_MASK) >> 24)
#define DM_RXD_GET_L4_PRS_INFO(desc)	(((desc)->cmds[0] & RXD_L4_PRS_INFO_MASK) >> 25)
#define DM_RXD_GET_L3_PRS_INFO(desc)	(((desc)->cmds[0] & RXD_L3_PRS_INFO_MASK) >> 28)
#define DM_RXD_GET_BUF_HDR(desc)	(((desc)->cmds[0] & RXD_BUF_HDR_MASK) >> 31)
#define DM_RXD_GET_LOOKUP_ID(desc)	(((desc)->cmds[1] & RXD_LOOKUP_ID_MASK) >> 0)
#define DM_RXD_GET_PRS_INFO(desc)	(((desc)->cmds[1] & RXD_PRS_INFO_MASK) >> 6)
#define DM_RXD_GET_BYTE_COUNT(desc)	(((desc)->cmds[1] & RXD_BYTE_COUNT_MASK) >> 16)
#define DM_RXD_GET_GEM_PID(desc)	(((desc)->cmds[2] & RXD_GEM_PID_MASK) >> 0)
#define DM_RXD_GET_GEM_DP(desc)		(((desc)->cmds[2] & RXD_GEM_DP_MASK) >> 12)
#define DM_RXD_GET_GOP_SOP(desc)	(((desc)->cmds[2] & RXD_GOP_SOP_MASK) >> 14)
#define DM_RXD_GET_KEY_HASH_EN(desc)	(((desc)->cmds[2] & RXD_KEY_HASH_EN_MASK) >> 15)
#define DM_RXD_GET_L4_CHK(desc)		(((desc)->cmds[2] & RXD_L4_CHK_MASK) >> 16)
#define DM_RXD_GET_TIMESTAMP(desc)	(((desc)->cmds[3] & RXD_TIMESTAMP_MASK) >> 0)
#define DM_RXD_GET_KEY_HASH(desc)	(((desc)->cmds[5] & RXD_KEY_HASH_MASK) >> 8)
#define DM_RXD_GET_QSET_NO(desc)	(((desc)->cmds[7] & RXD_QSET_NO_MASK) >> 8)
#define DM_RXD_GET_BUF_TYPE(desc)	(((desc)->cmds[7] & RXD_BUF_TYPE_MASK) >> 15)
#define DM_RXD_GET_MOD_DSCP(desc)	(((desc)->cmds[7] & RXD_MOD_DSCP_MASK) >> 16)
#define DM_RXD_GET_MOD_PRI(desc)	(((desc)->cmds[7] & RXD_MOD_PRI_MASK) >> 22)
#define DM_RXD_GET_MOD_DSCP_EN(desc)	(((desc)->cmds[7] & RXD_MOD_DSCP_EN_MASK) >> 25)
#define DM_RXD_GET_MOD_PRI_EN(desc)	(((desc)->cmds[7] & RXD_MOD_PRI_EN_MASK) >> 26)
#define DM_RXD_GET_GEM_PID_EN(desc)	(((desc)->cmds[7] & RXD_MOD_GEM_PID_EN_MASK) >> 27)
#define DM_RXD_GET_PORT_NUM(desc)	(((desc)->cmds[7] & RXD_PORT_NUM_MASK) >> 29)
#define DM_RXD_GET_PHYS_LO(desc)	(((desc)->cmds[4] & RXD_BUF_PHYS_LO_MASK) >> 0)
#define DM_RXD_GET_PHYS_HI(desc)	(((desc)->cmds[5] & RXD_BUF_PHYS_HI_MASK) >> 0)
#define DM_RXD_GET_VIRT_LO(desc)	(((desc)->cmds[6] & RXD_BUF_VIRT_LO_MASK) >> 0)
#define DM_RXD_GET_VIRT_HI(desc)	(((desc)->cmds[7] & RXD_BUF_VIRT_HI_MASK) >> 0)
#define DM_RXD_GET_PHYSADDR(desc)	(uintptr_t)(((uint64_t)DM_RXD_GET_PHYS_HI(desc) << 32) | \
					(uint64_t)DM_RXD_GET_PHYS_LO(desc))
#define DM_RXD_GET_VIRTADDR(desc)	(uintptr_t)(((uint64_t)DM_RXD_GET_VIRT_HI(desc) << 32) | \
					(uint64_t)DM_RXD_GET_VIRT_LO(desc))

/* TX Descriptor Setters */
/* TODO: Disabled because of collision wih mv_pp2_ppio.h, to be resolved with code unification */
/* #define DM_TXD_SET_L3_OFF(desc, data)	((desc)->cmds[0] = ((desc)->cmds[0] & ~TXD_L3_OFF_MASK) | \
 *						(data << 0 & TXD_L3_OFF_MASK))
 */
#define DM_TXD_SET_BUFMODE(desc, data)		((desc)->cmds[0] = ((desc)->cmds[0] & ~TXD_BUFMODE_MASK) | \
						(data << 7 & TXD_BUFMODE_MASK))
#define DM_TXD_SET_IPHDR_LEN(desc, data)	((desc)->cmds[0] = ((desc)->cmds[0] & ~TXD_IPHDR_LEN_MASK) | \
						(data << 8 & TXD_IPHDR_LEN_MASK))
#define DM_TXD_SET_GEN_L4_CHK(desc, data)	((desc)->cmds[0] = ((desc)->cmds[0] & ~TXD_GEN_L4_CHK_MASK) | \
						(data << 13 & TXD_GEN_L4_CHK_MASK))
#define DM_TXD_SET_GEN_IP_CHK(desc, data)	((desc)->cmds[0] = ((desc)->cmds[0] & ~TXD_GEN_IP_CHK_MASK) | \
						(data << 15 & TXD_GEN_IP_CHK_MASK))
#define DM_TXD_SET_POOL_ID(desc, data)		((desc)->cmds[0] = ((desc)->cmds[0] & ~TXD_POOL_ID_MASK) | \
						(data << 16 & TXD_POOL_ID_MASK))
#define DM_TXD_SET_PKT_OFF_EXT(desc, data)	((desc)->cmds[0] = ((desc)->cmds[0] & ~TXD_PKT_OFF_EXT_MASK) | \
						(data << 20 & TXD_PKT_OFF_EXT_MASK))
#define DM_TXD_SET_HWF_SYNC(desc, data)		((desc)->cmds[0] = ((desc)->cmds[0] & ~TXD_HWF_SYNC_MASK) | \
						(data << 21 & TXD_HWF_SYNC_MASK))
#define DM_TXD_SET_HWF(desc, data)		((desc)->cmds[0] = ((desc)->cmds[0] & ~TXD_HWF_MASK) | \
						(data << 22 & TXD_HWF_MASK))
#define DM_TXD_SET_PADD_DSBL(desc, data)	((desc)->cmds[0] = ((desc)->cmds[0] & ~TXD_PADD_DSBL_MASK) | \
						(data << 23 & TXD_PADD_DSBL_MASK))
#define DM_TXD_SET_L4_INFO(desc, data)		((desc)->cmds[0] = ((desc)->cmds[0] & ~TXD_L4_INFO_MASK) | \
						(data << 24 & TXD_L4_INFO_MASK))
#define DM_TXD_SET_L3_INFO(desc, data)		((desc)->cmds[0] = ((desc)->cmds[0] & ~TXD_L3_INFO_MASK) | \
						(data << 26 & TXD_L3_INFO_MASK))
#define DM_TXD_SET_L(desc, data)		((desc)->cmds[0] = ((desc)->cmds[0] & ~TXD_L_MASK) | \
						(data << 28 & TXD_L_MASK))
#define DM_TXD_SET_F(desc, data)		((desc)->cmds[0] = ((desc)->cmds[0] & ~TXD_F_MASK) | \
						(data << 29 & TXD_F_MASK))
#define DM_TXD_SET_FL(desc, data)		((desc)->cmds[0] = ((desc)->cmds[0] & ~TXD_FL_MASK) | \
						(data << 28 & TXD_FL_MASK))
#define DM_TXD_SET_FORMAT(desc, data)		((desc)->cmds[0] = ((desc)->cmds[0] & ~TXD_FORMAT_MASK) | \
						(data << 30 & TXD_FORMAT_MASK))
#define DM_TXD_SET_BUF_HDR(desc, data)		((desc)->cmds[0] = ((desc)->cmds[0] & ~TXD_BUF_HDR_MASK) | \
						(data << 31 & TXD_BUF_HDR_MASK))
#define DM_TXD_SET_PKT_OFF(desc, data)		((desc)->cmds[1] = ((desc)->cmds[1] & ~TXD_PKT_OFF_MASK) | \
						(data << 0 & TXD_PKT_OFF_MASK))
#define DM_TXD_SET_DEST_QID(desc, data)		((desc)->cmds[1] = ((desc)->cmds[1] & ~TXD_DEST_QID_MASK) | \
						(data << 8 & TXD_DEST_QID_MASK))
#define DM_TXD_SET_BYTE_COUNT(desc, data)	((desc)->cmds[1] = ((desc)->cmds[1] & ~TXD_BYTE_COUNT_MASK) | \
						(data << 16 & TXD_BYTE_COUNT_MASK))
#define DM_TXD_SET_GEM_PID_PTT(desc, data)	((desc)->cmds[2] = ((desc)->cmds[2] & ~TXD_GEM_PID_PTT_MASK) | \
						(data << 0 & TXD_GEM_PID_PTT_MASK))
#define DM_TXD_SET_DP(desc, data)		((desc)->cmds[2] = ((desc)->cmds[2] & ~TXD_DP_MASK) | \
						(data << 12 & TXD_DP_MASK))
#define DM_TXD_SET_DSATAG(desc, data)		((desc)->cmds[2] = ((desc)->cmds[2] & ~TXD_DSATAG_MASK) | \
						(data << 14 & TXD_DSATAG_MASK))
#define DM_TXD_SET_L4_INIT_CHK(desc, data)	((desc)->cmds[2] = ((desc)->cmds[2] & ~TXD_L4_INIT_CHK_MASK) | \
						(data << 16 & TXD_L4_INIT_CHK_MASK))
#define DM_TXD_SET_DPTR(desc, data)		((desc)->cmds[3] = ((desc)->cmds[3] & ~TXD_PME_DPTR_MASK) | \
						(data << 0 & TXD_PME_DPTR_MASK))
#define DM_TXD_SET_PME_PROGRAM(desc, data)	((desc)->cmds[3] = ((desc)->cmds[3] & ~TXD_PME_PROGRAM_MASK) | \
						(data << 16 & TXD_PME_PROGRAM_MASK))
#define DM_TXD_SET_HWF_IDB(desc, data)		((desc)->cmds[3] = ((desc)->cmds[3] & ~TXD_HWF_IDB_MASK) | \
						(data << 24 & TXD_HWF_IDB_MASK))
#define DM_TXD_SET_GEM_OEM(desc, data)		((desc)->cmds[3] = ((desc)->cmds[3] & ~TXD_GEM_OEM_MASK) | \
						(data << 25 & TXD_GEM_OEM_MASK))
#define DM_TXD_SET_ERR_SUM(desc, data)		((desc)->cmds[3] = ((desc)->cmds[3] & ~TXD_ERR_SUM_MASK) | \
						(data << 26 & TXD_ERR_SUM_MASK))
#define DM_TXD_SET_PON_FEC(desc, data)		((desc)->cmds[3] = ((desc)->cmds[3] & ~TXD_PON_FEC_MASK) | \
						(data << 27 & TXD_PON_FEC_MASK))
#define DM_TXD_SET_CPU_ID(desc, data)		((desc)->cmds[3] = ((desc)->cmds[3] & ~TXD_CPU_ID_MASK) | \
						(data << 28 & TXD_CPU_ID_MASK))
#define DM_TXD_SET_PTP_DESC(desc, data)		((desc)->cmds[5] = ((desc)->cmds[5] & ~TXD_PTP_DESC_MASK) | \
						(data << 8 & TXD_PTP_DESC_MASK))
#define DM_TXD_SET_QSET_NO(desc, data)		((desc)->cmds[7] = ((desc)->cmds[7] & ~TXD_BUF_QSET_NO_MASK) | \
						(data << 8 & TXD_BUF_QSET_NO_MASK))
#define DM_TXD_SET_TYPE(desc, data)		((desc)->cmds[7] = ((desc)->cmds[7] & ~TXD_BUF_TYPE_MASK) | \
						(data << 15 & TXD_BUF_TYPE_MASK))
#define DM_TXD_SET_DSCP(desc, data)		((desc)->cmds[7] = ((desc)->cmds[7] & ~TXD_MOD_DSCP_MASK) | \
						(data << 16 & TXD_MOD_DSCP_MASK))
#define DM_TXD_SET_PRI(desc, data)		((desc)->cmds[7] = ((desc)->cmds[7] & ~TXD_MOD_PRI_MASK) | \
						(data << 22 & TXD_MOD_PRI_MASK))
#define DM_TXD_SET_DSCP_EN(desc, data)		((desc)->cmds[7] = ((desc)->cmds[7] & ~TXD_MOD_DSCP_EN_MASK) | \
						(data << 25 & TXD_MOD_DSCP_EN_MASK))
#define DM_TXD_SET_PRI_EN(desc, data)		((desc)->cmds[7] = ((desc)->cmds[7] & ~TXD_MOD_PRI_EN_MASK) | \
						(data << 26 & TXD_MOD_PRI_EN_MASK))
#define DM_TXD_SET_GEM_EN(desc, data)		((desc)->cmds[7] = ((desc)->cmds[7] & ~TXD_MOD_GEM_EN_MASK) | \
						(data << 27 & TXD_MOD_GEM_EN_MASK))
#define DM_TXD_SET_PHYSADDR(desc, data) \
	do { \
		(desc)->cmds[4] = (uint32_t)(uintptr_t)data; \
		(desc)->cmds[5] = ((desc)->cmds[5] & ~TXD_BUF_PHYS_HI_MASK) | \
		((uintptr_t)data >> 32 & TXD_BUF_PHYS_HI_MASK); \
	} while (0)
#define DM_TXD_SET_VIRTADDR(desc, data)	  \
	do { \
		(desc)->cmds[6] = (uint32_t)(uintptr_t)data; \
		(desc)->cmds[7] = ((desc)->cmds[7] & ~TXD_BUF_VIRT_HI_MASK) | \
		((uintptr_t)data >> 32 & TXD_BUF_VIRT_HI_MASK); \
	} while (0)

/* Released by software */
#define TXD_RLS_SW					(0x00)
/* Released by BM */
#define TXD_RLS_BM					(0x01)
/* Enable L4 chk generation */
#define TXD_L4_CHK_EN					(0x00)
/* Disable L4 chl generation */
#define TXD_L4_CHK_DS					(0x02)
/* Enable L3 chk generation */
#define TXD_L3_CHK_EN					(0x00)
/* Disable L3 chk generation */
#define TXD_L3_CHK_DS					(0x01)
/* Enable padding */
#define TXD_PADD_EN					(0x00)
/* Disable padding */
#define TXD_PADD_DS					(0x01)
/* TCP packet */
#define TXD_L4_TCP					(0x00)
/* UDP packet */
#define TXD_L4_UDP					(0x01)
/* IPv4 packet */
#define TXD_L3_IPV4					(0x00)
/* IPv6 packet */
#define TXD_L3_IPV6					(0x01)
/* Last buffer */
/* Buffer format */
#define TXD_FMT_BUF					(0x00)
/* Packet format */
#define TXD_FMT_PKT					(0x01)
/* Packe default offset */
#define TXD_OFF_ZERO					(0x00)

/* TX Descriptor masks (Internal) */
/* cmd 0 */
#define TXD_L3_OFF_MASK			(0x0000007F)
#define TXD_BUFMODE_MASK		(0x00000080)
#define TXD_IPHDR_LEN_MASK		(0x00001F00)
#define TXD_GEN_L4_CHK_MASK		(0x00006000)
#define TXD_GEN_IP_CHK_MASK		(0x00008000)
#define TXD_POOL_ID_MASK		(0x000F0000)
#define TXD_PKT_OFF_EXT_MASK		(0x00100000)
#define TXD_HWF_SYNC_MASK		(0x00200000)
#define TXD_HWF_MASK			(0x00400000)
#define TXD_PADD_DSBL_MASK		(0x00800000)
#define TXD_L4_INFO_MASK		(0x03000000)
#define TXD_L3_INFO_MASK		(0x0C000000)
#define TXD_L_MASK			(0x10000000)
#define TXD_F_MASK			(0x20000000)
#define TXD_FL_MASK			(TXD_F_MASK | TXD_L_MASK)
#define TXD_FORMAT_MASK			(0x40000000)
#define TXD_BUF_HDR_MASK		(0x80000000)
/* cmd 1 */
#define TXD_PKT_OFF_MASK		(0x000000FF)
#define TXD_DEST_QID_MASK		(0x0000FF00)
#define TXD_BYTE_COUNT_MASK		(0xFFFF0000)
/* cmd 2 */
#define TXD_GEM_PID_PTT_MASK		(0x00000FFF)
#define TXD_DP_MASK			(0x00003000)
#define TXD_DSATAG_MASK			(0x0000C000)
#define TXD_L4_INIT_CHK_MASK		(0xFFFF0000)
/* cmd 3 */
#define TXD_PME_DPTR_MASK		(0x0000FFFF)
#define TXD_PME_PROGRAM_MASK		(0x00FF0000)
#define TXD_HWF_IDB_MASK		(0x01000000)
#define TXD_GEM_OEM_MASK		(0x02000000)
#define TXD_ERR_SUM_MASK		(0x04000000)
#define TXD_PON_FEC_MASK		(0x08000000)
#define TXD_CPU_ID_MASK			(0xF0000000)
/* cmd 4 */
#define TXD_BUF_PHYS_LO_MASK		(0xFFFFFFFF)
/* cmd 5 */
#define TXD_BUF_PHYS_HI_MASK		(0x000000FF)
#define TXD_PTP_DESC_MASK		(0xFFFFFF00)
/* cmd 6 */
#define TXD_BUF_VIRT_LO_MASK		(0xFFFFFFFF)
/* cmd 7 */
#define TXD_BUF_VIRT_HI_MASK		(0x000000FF)
#define TXD_BUF_QSET_NO_MASK		(0x00007F00)
#define TXD_BUF_TYPE_MASK		(0x00008000)
#define TXD_MOD_DSCP_MASK		(0x003F0000)
#define TXD_MOD_PRI_MASK		(0x01C00000)
#define TXD_MOD_DSCP_EN_MASK		(0x02000000)
#define TXD_MOD_PRI_EN_MASK		(0x04000000)
#define TXD_MOD_GEM_EN_MASK		(0x08000000)

/* RX Descriptor masks (Internal) */
/* cmd 0 */
#define RXD_L3_OFF_MASK			(0x0000007F)
#define RXD_IPHDR_LEN_MASK		(0x00001F00)
#define RXD_EC_MASK			(0x00006000)
#define RXD_ES_MASK			(0x00008000)
#define RXD_POOL_ID_MASK		(0x000F0000)
#define RXD_HWF_SYNC_MASK		(0x00200000)
#define RXD_L4_CHK_OK_MASK		(0x00400000)
#define RXD_L3_IP_FRAG_MASK		(0x00800000)
#define RXD_L3_IP4_HDR_ERR_MASK		(0x01000000)
#define RXD_L4_PRS_INFO_MASK		(0x0E000000)
#define RXD_L3_PRS_INFO_MASK		(0x70000000)
#define RXD_BUF_HDR_MASK		(0x80000000)
/* cmd 1 */
#define RXD_LOOKUP_ID_MASK		(0x0000003F)
#define RXD_PRS_INFO_MASK		(0x0000FFC0)
#define RXD_BYTE_COUNT_MASK		(0xFFFF0000)
/* cmd 2 */
#define RXD_GEM_PID_MASK		(0x00000FFF)
#define RXD_GEM_DP_MASK			(0x00003000)
#define RXD_GOP_SOP_MASK		(0x00004000)
#define RXD_KEY_HASH_EN_MASK		(0x00008000)
#define RXD_L4_CHK_MASK			(0xFFFF0000)
/* cmd 3 */
#define RXD_TIMESTAMP_MASK		(0xFFFFFFFF)
/* cmd 4 */
#define RXD_BUF_PHYS_LO_MASK		(0xFFFFFFFF)
/* cmd 5 */
#define RXD_BUF_PHYS_HI_MASK		(0x000000FF)
#define RXD_KEY_HASH_MASK		(0xFFFFFF00)
/* cmd 6 */
#define RXD_BUF_VIRT_LO_MASK		(0xFFFFFFFF)
/* cmd 7 */
#define RXD_BUF_VIRT_HI_MASK		(0x000000FF)
#define RXD_QSET_NO_MASK		(0x00007F00)
#define RXD_BUF_TYPE_MASK		(0x00008000)
#define RXD_MOD_DSCP_MASK		(0x003F0000)
#define RXD_MOD_PRI_MASK		(0x01C00000)
#define RXD_MOD_DSCP_EN_MASK		(0x02000000)
#define RXD_MOD_PRI_EN_MASK		(0x04000000)
#define RXD_MOD_GEM_PID_EN_MASK		(0x08000000)
#define RXD_PORT_NUM_MASK		(0xE0000000)

/* TODO: Revise and style above macros to 80 lines max. */
#endif /* _MVPP2X_DM_H_ */
