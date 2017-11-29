/******************************************************************************
 *	Copyright (C) 2017 Marvell International Ltd.
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

#ifndef _GIU_MNG_DESC_H_
#define _GIU_MNG_DESC_H_

/*
 * Management descriptors definitions.
 */

#define MGMT_DESC_DATA_LEN		(52)

enum cmd_dest_type {
	CDT_INVALID = 0,
	CDT_PF = 1,
	CDT_VF = 2
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
	CC_PF_BM_POOL_ADD,
	CC_PF_EGRESS_TC_ADD,
	CC_PF_EGRESS_DATA_Q_ADD,
	CC_PF_INGRESS_TC_ADD,
	CC_PF_INGRESS_DATA_Q_ADD,

	CC_PF_ENABLE,
	CC_PF_DISABLE,

	CC_PF_MGMT_ECHO,
	CC_PF_LINK_STATUS,
	CC_PF_CLOSE,
	CMD_CODE_LAST = 0XFF,
};

enum agnic_notif_codes {
	NC_PF_LINK_CHANGE = 0x1,

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


/*
 * mgmt_cmd - Encapsulates all management control commands parameters.
 */
/* Make sure structure is portable along different systems. */
#pragma pack(1)
struct mgmt_cmd_params {
	union {
		struct {
			u32	num_host_bm_pools;
			u32	num_host_egress_tc;
			u32	num_host_ingress_tc;
			u16	mtu_override;
			u16	mru_override;
			u8	egress_sched; /* enum aos_egress_sched */
		} pf_init;

		struct {
			u64	q_phys_addr;
			u32	q_len;
			u64	q_cons_phys_addr;
			u32	q_buf_size;
		} bm_pool_add;

		struct {
			u32	tc_prio;
			u32	num_queues_per_tc;
		} pf_egress_tc_add;

		/* Used for BM & Tx queues. */
		struct {
			u64	q_phys_addr;
			u32	q_len;
			u64	q_cons_phys_addr;
			u32	q_wrr_weight;
			u32	tc_prio; /* irrelevant for BM. */
		} pf_egress_q_add;

		struct {
			u32	tc_prio;
			u32	num_queues_per_tc;
			u32	pkt_offset;
			u8	hash_type; /* enum aos_ingress_hash_type */
		} pf_ingress_tc_add;

		struct {
			u64	q_phys_addr;
			u64	q_prod_phys_addr;
			u32	q_len;
			u32	msix_id;
			u32	tc_prio;
			u32	bm_pool_q_id_list[4];
		} pf_ingress_data_q_add;

		struct {
			u8 align[52];
		} align;
	};
};
#pragma pack()

/* Command Descriptor
 * cmd_idx - Command Identifier, this field will be copied to the response
 *   descriptor by the SNIC, in order to correlate the response with the command.
 * app_code - Target application Id (out of enum snic_app_codes)
 * cmd_code - Command to be executed (out of enum snic_cmd_codes)
 * dest_id - Destination ID – PF / VF Id
 * dest_type - Destination type – PF / VF
 * flags - Bitmask of AOS_CMD_FLAG_XX.
 * cmd_param_size - Size (Bytes) of the command parameters (Refers to the total
 *   size of the parameters, and not only the one included as part of this
 *   descriptor).
 * desc_param_size - Size (Bytes) of the command parameters transmitted as part
 *   of this descriptor
 * cmd_params - Command parameters
 *   if (flags & FLAG_EXT_BUFF):
 *     Command parameters include physical address of the buffer containing the
 *     command’s data.
 *   else:
 *     Command parameters include Array of bytes, holding the serialized
 *     parameters list for a specific command.
 */
/* Make sure structure is portable along different systems. */
#pragma pack(1)
struct cmd_desc {
	u16 cmd_idx;
	u16 app_code;
	u8 cmd_code;
	u8 dest_id;
	u8 dest_type;

#define CMD_FLAG_EXT_BUFF	BIT(3) /* Inline cmd params, or external buff */
#define CMD_FLAG_NO_RESP	BIT(2) /* No response is required for this cmd */
#define CMD_FLAG_FIRST	BIT(1) /* Marks the first descriptor in a series */
#define CMD_FLAG_LAST	BIT(0) /* Marks the last descriptor in a series */
	u8 flags;

	u16 cmd_param_size;
	u16 desc_param_size;

	struct mgmt_cmd_params params;
};
#pragma pack()


/*
 * mgmt_cmd_resp - Encapsulates the different responses that can be
 * received from the SNIC as a result of a management command.
 */
/* Make sure structure is portable along different systems. */
#pragma pack(1)
struct mgmt_cmd_resp {
	union {
		/* Use same response structure for all Q add operations.
		 * The prod_cons_phys_addr wil hold either the consumer or the
		 * producers address depending on the type of queue being
		 * created (ingress or egress).
		 */
		struct {
			u32	q_id;
			u64	q_prod_cons_phys_addr;
		} q_add_resp;

		u32 link_status;

		struct {
			u8 align[20];
		} align;
	};
};
#pragma pack()

/* Notifications Descriptor
 * cmd_idx - Command Identifier
 *   Command Reply – copied from the command descriptor by the SNIC.
 *   Notification – Notification identifier
 * app_code - Target application Id.
 * flags - Bitmask of AOS_NOTIF_FLAG_XX.
 * status - Command execution status (0 - Ok, 1 - Fail, 0xFF - Notification).
 * resp_param_size - Size (Bytes) of the response parameters (Refers to the total
 *   size of the parameters, and not only the one included as part of this
 *   descriptor).
 * desc_param_size - Size (Bytes) of the response parameters transmitted as part
 *   of this descriptor.
 * notif_params - Notification parameters
 *   if (flags & FLAG_EXT_BUFF):
 *     Notif parameters include physical address of the buffer containing the
 *     notification's data.
 *   else:
 *     Notif parameters include Array of bytes, holding the serialized
 *     parameters list for a specific notification.
 */
#pragma pack(1)
struct notif_desc {
	u16 cmd_idx;
	u16 app_code;

#define NOTIF_FLAG_EXT_BUFF	BIT(0) /* Inline notif params, or external buff */
	u8 flags;

#define NOTIF_STATUS_OK	(0)
#define NOTIF_STATUS_FAIL	(1)
#define NOTIF_STATUS_NOTIF	(0xFF)
	u8 status;
	u8 pad[2];

	u16 resp_param_size;
	u16 desc_param_size;
	struct mgmt_cmd_resp resp_data;

};
#pragma pack()

#endif /* _GIU_MNG_DESC_H_ */
