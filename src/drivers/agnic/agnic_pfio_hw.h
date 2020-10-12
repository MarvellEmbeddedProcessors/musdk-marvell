/*******************************************************************************
 * Copyright (C) Marvell International Ltd. and its affiliates
 *
 * This software file (the "File") is owned and distributed by Marvell
 * International Ltd. and/or its affiliates ("Marvell") under the following
 * alternative licensing terms.  Once you have made an election to distribute the
 * File under one of the following license alternatives, please (i) delete this
 * introductory statement regarding license alternatives, (ii) delete the three
 * license alternatives that you have not elected to use and (iii) preserve the
 * Marvell copyright notice above.
 *
 ********************************************************************************
 * Marvell Commercial License Option
 *
 * If you received this File from Marvell and you have entered into a commercial
 * license agreement (a "Commercial License") with Marvell, the File is licensed
 * to you under the terms of the applicable Commercial License.
 *
 ********************************************************************************
 * Marvell GPL License Option
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File in accordance with the terms and conditions of the General
 * Public License Version 2, June 1991 (the "GPL License"), a copy of which is
 * available along with the File in the license.txt file or by writing to the Free
 * Software Foundation, Inc., or on the worldwide web at http://www.gnu.org/licenses/gpl.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY
 * DISCLAIMED.  The GPL License provides additional details about this warranty
 * disclaimer.
 *
 ********************************************************************************
 * Marvell GNU General Public License FreeRTOS Exception
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File in accordance with the terms and conditions of the Lesser
 * General Public License Version 2.1 plus the following FreeRTOS exception.
 * An independent module is a module which is not derived from or based on
 * FreeRTOS.
 * Clause 1:
 * Linking FreeRTOS statically or dynamically with other modules is making a
 * combined work based on FreeRTOS. Thus, the terms and conditions of the GNU
 * General Public License cover the whole combination.
 * As a special exception, the copyright holder of FreeRTOS gives you permission
 * to link FreeRTOS with independent modules that communicate with FreeRTOS solely
 * through the FreeRTOS API interface, regardless of the license terms of these
 * independent modules, and to copy and distribute the resulting combined work
 * under terms of your choice, provided that:
 * 1. Every copy of the combined work is accompanied by a written statement that
 * details to the recipient the version of FreeRTOS used and an offer by yourself
 * to provide the FreeRTOS source code (including any modifications you may have
 * made) should the recipient request it.
 * 2. The combined work is not itself an RTOS, scheduler, kernel or related
 * product.
 * 3. The independent modules add significant and primary functionality to
 * FreeRTOS and do not merely extend the existing functionality already present in
 * FreeRTOS.
 * Clause 2:
 * FreeRTOS may not be used for any competitive or comparative purpose, including
 * the publication of any form of run time or compile time metric, without the
 * express permission of Real Time Engineers Ltd. (this is the norm within the
 * industry and is intended to ensure information accuracy).
 *
 ********************************************************************************
 * Marvell BSD License Option
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File under the following licensing terms.
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *	* Redistributions of source code must retain the above copyright notice,
 *	  this list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the distribution.
 *
 *	* Neither the name of Marvell nor the names of its contributors may be
 *	  used to endorse or promote products derived from this software without
 *	  specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************/

#ifndef __AGNIC_PFIO_HW_H__
#define __AGNIC_PFIO_HW_H__

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
struct agnic_q_hw_info {
	u64	q_addr;
	u32	q_prod_offs;
	u32	q_cons_offs;
	u32	len;
	u32	res;
} __packed;

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
		} __packed pf_init;

		struct {
			u32	tc;
			u32	num_queues;
		} __packed pf_egress_tc_add;

		/* Used for BP & Tx queues. */
		struct {
			u64	q_phys_addr;
			u32	q_prod_offs;
			u32	q_cons_offs;
			u32	q_len;
			u32	q_wrr_weight;
			u32	tc; /* irrelevant for BP. */
			u32	msix_id;
		} __packed pf_egress_q_add;

		struct {
			u32	tc;
			u32	num_queues;
			u32	pkt_offset;
			u8	hash_type; /* enum agnic_ingress_hash_type */
		} __packed pf_ingress_tc_add;

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
		} __packed pf_ingress_data_q_add;

		struct {
			u8	reset;
		} __packed pf_get_statistics;

		struct {
			u8 out;
			u8 tc;
			u8 qid;
			u8	reset;
		} __packed pf_q_get_statistics;

		struct {
			u32	mtu;
		} __packed pf_set_mtu;

		struct {
			u8 loopback;
		} __packed pf_set_loopback;

		struct {
			u16 vlan;
		} __packed pf_vlan;

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
#define AGNIC_NOTIF_STATUS_OK	(0)
#define AGNIC_NOTIF_STATUS_FAIL	(1)
	u8 status;
	union {
		/* Use same response structure for all Q add operations. */
		struct {
#define AGNIC_Q_INF_STATUS_OK	(0)
#define AGNIC_Q_INF_STATUS_ERR	(1)
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
