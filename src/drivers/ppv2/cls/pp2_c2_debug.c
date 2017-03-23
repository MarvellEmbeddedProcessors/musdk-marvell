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
 * @file pp2_c2_debug.c
 *
 * C3 High level debug routines (dump functions and external configuration)
 */

/***********************/
/* c file declarations */
/***********************/
#include "std_internal.h"
#include "../pp2_types.h"
#include "../pp2.h"
#include "../pp2_hw_type.h"
#include "../pp2_hw_cls.h"

/* source port type name string */
static char *pp2_cls_port_type_str_tbl[3] = {
	"PHY",
	"UNI",
	"VIRT"
};

/* C2 QOS table name string */
static char *pp2_cls_qos_table_str_tbl[2] = {
	"QOS_PBIT",
	"QOS_DSCP"
};

/* C2 QOS table name string */
static char *pp2_cls_qos_src_str_tbl[2] = {
	"ACTION",
	"DSCP_PBIT"
};

/* pp2_cls color action name string */
static char *pp2_cls_color_action_str[8] = {
	"No_Updt",
	"No_Updt_L",
	"Green",
	"",
	"Yellow",
	"",
	"Red",
	"Red_L"
};

/* pp2_cls common action name string */
static char *pp2_cls_common_action_str[4] = {
	"No_Upd",
	"No_Updt_L",
	"Update",
	"Update_L"
};

/* pp2_cls flowID enable action name string */
static char *pp2_cls_flowid_action_str[2] = {
	"Disable",
	"Enable"
};

/* pp2_cls forwarding action name string */
static char *pp2_cls_frwd_action_str[8] = {
	"No_Updt",
	"No_Updt_L",
	"SWF",
	"SWF_L",
	"HWF",
	"HWF_L",
	"HWF_Lat",
	"HWF_Lat_L"
};

/*******************************************************************************
 * pp2_cls_sfs_valid_c2_entry_header_dump
 *
 * DESCRIPTION: print header.
 * INPUTS:
 *           None
 *
 * OUTPUTS:
 *           None
 *
 * COMMENTS:
 *           None
 ******************************************************************************/
static void pp2_cls_valid_c2_entry_header_dump(void)
{
	print_horizontal_line(150, "=");
	printf("= LKP_Type | Priority | Port_Info |   Field_Name  |      Field_Value     |");
	printf("            QOS_Info            |    Action_Info     | Hit_Cnt |   INDX    =\n");
	print_horizontal_line(150, "=");
}

/*******************************************************************************
 * pp2_cls_valid_c2_entry_line_dump
 *
 * DESCRIPTION: Print one line of dump info.
 * INPUTS:
 *           None
 *
 * OUTPUTS:
 *           None
 *
 * COMMENTS:
 *           None
 ******************************************************************************/
static void pp2_cls_valid_c2_entry_line_dump(uintptr_t cpu_slot,
					     u32 lkp_type,
					     u32 dump_idx,
					     struct pp2_cls_c2_data_t	*c2_entry,
					     struct pp2_cls_c2_index_t	*c2_index_node)
{
	char *empty_str = "";
	char field_name_str[14] = "";
	char qos_info_str[30] = "";
	char action_str[18] = "";
	char port_info_str[8] = "";
	char field_value_str[48] = "";
	char field_value_mask_str[48] = "";
	char lookup_type_str[8] = "";
	char internal_pri_str[8] = "";
	char index_str[8] = "";
	char hit_cnt_str[8] = "";
	u32 hit_cnt = 0;
	u32 field_id;
	u32 field_match;
	struct pp2_cls_field_match_info field_match_info;
	struct pp2_cls_mng_pkt_key_t pp2_cls_pkt_key_t_tmp;
	struct pp2_cls_pkt_key_t	pkt_key_tmp;

	if (!dump_idx) {
		sprintf(lookup_type_str, "%02d", lkp_type);
		sprintf(internal_pri_str, "%03d", c2_entry->priority);
		mv_pp2x_c2_hit_cntr_read(cpu_slot, c2_index_node->c2_hw_idx, &hit_cnt);
		sprintf(hit_cnt_str, "%03d", hit_cnt);
	}

	/* port info, including type, value and mask */
	switch (dump_idx) {
	case MVPP2_PORT_DUMP_TYPE:
		sprintf(port_info_str, "T:%s", pp2_cls_port_type_str_tbl[c2_entry->port.port_type]);
		break;
	case MVPP2_PORT_DUMP_VALUE:
		sprintf(port_info_str, "V:0x%x", c2_entry->port.port_value);
		break;
	case MVPP2_PORT_DUMP_MASK:
		sprintf(port_info_str, "M:0x%x", c2_entry->port.port_mask);
		break;
	}

	/* Field match parse and set field value */
	if ((!c2_entry->field_bm) && (!dump_idx)) {
		sprintf(field_name_str, "%s", "NA");
		sprintf(field_value_str, "%s", "NA");
	}

	field_match = c2_entry->field_bm & (1 << dump_idx);

	/* skip IPv6 sub matches, need only one bit to print data */
	if (c2_entry->field_bm & MVPP2_MATCH_IPV6_PREF)
		field_match &= ~MVPP2_MATCH_IPV6_PREF;
	else if (c2_entry->field_bm & MVPP2_MATCH_IPV6_SUFF)
		field_match &= ~MVPP2_MATCH_IPV6_SUFF;

	if (field_match) {
		pp2_cls_pkt_key_t_tmp.ttl = c2_entry->mng_pkt_key.ttl;
		pp2_cls_pkt_key_t_tmp.tcp_flag = c2_entry->mng_pkt_key.tcp_flag;
		pp2_cls_pkt_key_t_tmp.tcp_flag_mask = c2_entry->mng_pkt_key.tcp_flag_mask;
		memcpy(&pkt_key_tmp, &c2_entry->mng_pkt_key.pkt_key, sizeof(struct pp2_cls_pkt_key_t));
		pp2_cls_pkt_key_t_tmp.pkt_key = &pkt_key_tmp;
		memset(&field_match_info, 0, sizeof(struct pp2_cls_field_match_info));

		/* set IPv6 sub matches cleared to get correct field_info readings */
		if (c2_entry->field_bm & MVPP2_MATCH_IPV6_PREF)
			field_match |= MVPP2_MATCH_IPV6_PREF;
		else if (c2_entry->field_bm & MVPP2_MATCH_IPV6_SUFF)
			field_match |= MVPP2_MATCH_IPV6_SUFF;

		if (pp2_cls_field_bm_to_field_info(field_match,
						   &pp2_cls_pkt_key_t_tmp,
						   1,
						   true,
						   &field_match_info))
			return;
		field_id = field_match_info.field_id;
		sprintf(field_name_str, "%s", pp2_cls_utils_field_id_str_get(field_id));

		switch (field_id) {
		case MAC_DA_FIELD_ID:
		case MAC_SA_FIELD_ID:
			sprintf(field_value_str,
				"V: %p", field_match_info.filed_value.mac_addr.parsed_mac_addr);
			sprintf(field_value_mask_str,
				"M: %p", field_match_info.filed_value.mac_addr.parsed_mac_addr_mask);
			break;

		case IPV4_SA_FIELD_ID:
		case IPV4_DA_FIELD_ID:
		case ARP_IPV4_DA_FIELD_ID:
			sprintf(field_value_str,
				"V: %d.%d.%d.%d",
				field_match_info.filed_value.ipv4_addr.parsed_ipv4_addr[0],
				field_match_info.filed_value.ipv4_addr.parsed_ipv4_addr[1],
				field_match_info.filed_value.ipv4_addr.parsed_ipv4_addr[2],
				field_match_info.filed_value.ipv4_addr.parsed_ipv4_addr[3]);
			sprintf(field_value_mask_str,
				"M: %02x.%02x.%02x.%02x",
				field_match_info.filed_value.ipv4_addr.parsed_ipv4_addr_mask[0],
				field_match_info.filed_value.ipv4_addr.parsed_ipv4_addr_mask[1],
				field_match_info.filed_value.ipv4_addr.parsed_ipv4_addr_mask[2],
				field_match_info.filed_value.ipv4_addr.parsed_ipv4_addr_mask[3]);
			break;

		case IPV6_SA_PREF_FIELD_ID:
		case IPV6_DA_PREF_FIELD_ID:
			sprintf(field_value_str,
				"V: %04x:%04x:%04x:%04x",
				field_match_info.filed_value.ipv6_addr.parsed_ipv6_addr[0],
				field_match_info.filed_value.ipv6_addr.parsed_ipv6_addr[2],
				field_match_info.filed_value.ipv6_addr.parsed_ipv6_addr[4],
				field_match_info.filed_value.ipv6_addr.parsed_ipv6_addr[6]);
			sprintf(field_value_mask_str,
				"M: %04x:%04x:%04x:%04x",
				field_match_info.filed_value.ipv6_addr.parsed_ipv6_addr_mask[0],
				field_match_info.filed_value.ipv6_addr.parsed_ipv6_addr_mask[2],
				field_match_info.filed_value.ipv6_addr.parsed_ipv6_addr_mask[4],
				field_match_info.filed_value.ipv6_addr.parsed_ipv6_addr_mask[6]);
			break;

		case IPV6_DA_SUFF_FIELD_ID:
		case IPV6_SA_SUFF_FIELD_ID:
			sprintf(field_value_str,
				"V: %04x:%04x:%04x:%04x",
				field_match_info.filed_value.ipv6_addr.parsed_ipv6_addr[8],
				field_match_info.filed_value.ipv6_addr.parsed_ipv6_addr[10],
				field_match_info.filed_value.ipv6_addr.parsed_ipv6_addr[12],
				field_match_info.filed_value.ipv6_addr.parsed_ipv6_addr[14]);
			sprintf(field_value_mask_str,
				"M: %04x:%04x:%04x:%04x",
				field_match_info.filed_value.ipv6_addr.parsed_ipv6_addr_mask[8],
				field_match_info.filed_value.ipv6_addr.parsed_ipv6_addr_mask[10],
				field_match_info.filed_value.ipv6_addr.parsed_ipv6_addr_mask[12],
				field_match_info.filed_value.ipv6_addr.parsed_ipv6_addr_mask[14]);
			break;

		case IPV6_SA_FIELD_ID:
		case IPV6_DA_FIELD_ID:
			sprintf(field_value_str,
				"V: %p",
				field_match_info.filed_value.ipv6_addr.parsed_ipv6_addr);
			sprintf(field_value_mask_str,
				"M: %p",
				field_match_info.filed_value.ipv6_addr.parsed_ipv6_addr_mask);
			break;

		default:
			sprintf(field_value_str,
				"V: 0x%x",
				field_match_info.filed_value.int_data.parsed_int_val);
			sprintf(field_value_mask_str,
				"M: 0x%x",
				field_match_info.filed_value.int_data.parsed_int_val_mask);
		}
	}

	/* QOS_Info */
	switch (dump_idx) {
	case MVPP2_QOS_DUMP_DSCP:
		if (c2_entry->qos_info.pri_dscp_src == MVPP2_QOS_SRC_DSCP_PBIT_TBL)
			sprintf(qos_info_str,
				"DSCP src: %s",
				pp2_cls_qos_src_str_tbl[c2_entry->qos_info.pri_dscp_src]);
		else
			sprintf(qos_info_str,
				"DSCP src: %s, DSCP=%d",
				pp2_cls_qos_src_str_tbl[c2_entry->qos_info.pri_dscp_src],
				c2_entry->qos_value.dscp);
		break;
	case MVPP2_QOS_DUMP_PBIT:
		if (c2_entry->qos_info.pri_dscp_src == MVPP2_QOS_SRC_DSCP_PBIT_TBL)
			sprintf(qos_info_str,
				"PBIT src: %s",
				pp2_cls_qos_src_str_tbl[c2_entry->qos_info.pri_dscp_src]);
		else
			sprintf(qos_info_str,
				"PBIT src: %s, PBIT=%d",
				pp2_cls_qos_src_str_tbl[c2_entry->qos_info.pri_dscp_src],
				c2_entry->qos_value.pri);
		break;
	case MVPP2_QOS_DUMP_GEMPORT:
		if (c2_entry->qos_info.gemport_src == MVPP2_QOS_SRC_DSCP_PBIT_TBL)
			sprintf(qos_info_str,
				"GEMP src: %s",
				pp2_cls_qos_src_str_tbl[c2_entry->qos_info.gemport_src]);
		else
			sprintf(qos_info_str,
				"GEMP src: %s, GEMP=%d",
				pp2_cls_qos_src_str_tbl[c2_entry->qos_info.gemport_src],
				c2_entry->qos_value.gemp);
		break;
	case MVPP2_QOS_DUMP_QUEUE_LOW:
		if (c2_entry->qos_info.gemport_src == MVPP2_QOS_SRC_DSCP_PBIT_TBL)
			sprintf(qos_info_str,
				"Q_L src: %s",
				pp2_cls_qos_src_str_tbl[c2_entry->qos_info.q_low_src]);
		else
			sprintf(qos_info_str,
				"Q_L src: %s, Q_L=%d",
				pp2_cls_qos_src_str_tbl[c2_entry->qos_info.q_low_src],
				c2_entry->qos_value.q_low);
		break;
	case MVPP2_QOS_DUMP_QUEUE_HIGH:
		if (c2_entry->qos_info.gemport_src == MVPP2_QOS_SRC_DSCP_PBIT_TBL)
			sprintf(qos_info_str,
				"Q_H src: %s",
				pp2_cls_qos_src_str_tbl[c2_entry->qos_info.q_high_src]);
		else
			sprintf(qos_info_str,
				"Q_H src: %s, Q_H=%d",
				pp2_cls_qos_src_str_tbl[c2_entry->qos_info.q_high_src],
				c2_entry->qos_value.q_high);
		break;
	case MVPP2_QOS_DUMP_COLOR:
		sprintf(qos_info_str,
			"Color src: %s",
			pp2_cls_qos_src_str_tbl[c2_entry->qos_info.color_src]);
		break;
	case MVPP2_QOS_DUMP_TABLE:
		if (c2_entry->qos_info.pri_dscp_src == MVPP2_QOS_SRC_DSCP_PBIT_TBL ||
		    c2_entry->qos_info.gemport_src == MVPP2_QOS_SRC_DSCP_PBIT_TBL ||
		    c2_entry->qos_info.q_low_src == MVPP2_QOS_SRC_DSCP_PBIT_TBL ||
		    c2_entry->qos_info.q_high_src == MVPP2_QOS_SRC_DSCP_PBIT_TBL ||
		    c2_entry->qos_info.color_src == MVPP2_QOS_SRC_DSCP_PBIT_TBL)
			sprintf(qos_info_str,
				"QOS tbl: %s, index=%d", pp2_cls_qos_table_str_tbl[c2_entry->qos_info.qos_tbl_type],
				c2_entry->qos_info.qos_tbl_index);
		break;
	case MVPP2_QOS_DUMP_POLICER:
		if (c2_entry->qos_info.policer_id != 0xFFFF)
			sprintf(qos_info_str,
				"Policer ID = %d",
				c2_entry->qos_info.policer_id);
		break;
	}

	/* Action */
	switch (dump_idx) {
	case MVPP2_ACT_DUMP_DSCP:
		sprintf(action_str, "DSCP: %s", pp2_cls_common_action_str[c2_entry->action.dscp_act]);
		break;
	case MVPP2_ACT_DUMP_PBIT:
		sprintf(action_str, "PBIT: %s", pp2_cls_common_action_str[c2_entry->action.pri_act]);
		break;
	case MVPP2_ACT_DUMP_GEMPORT:
		sprintf(action_str, "GEMP: %s", pp2_cls_common_action_str[c2_entry->action.gemp_act]);
		break;
	case MVPP2_ACT_DUMP_QUEUE_LOW:
		sprintf(action_str, "Q_LOW: %s", pp2_cls_common_action_str[c2_entry->action.q_low_act]);
		break;
	case MVPP2_ACT_DUMP_QUEUE_HIGH:
		sprintf(action_str, "Q_HIGH: %s", pp2_cls_common_action_str[c2_entry->action.q_high_act]);
		break;
	case MVPP2_ACT_DUMP_COLOR:
		sprintf(action_str, "COLOR: %s", pp2_cls_color_action_str[c2_entry->action.color_act]);
		break;
	case MVPP2_ACT_DUMP_POLICER:
		sprintf(action_str, "POLICER: %s", pp2_cls_common_action_str[c2_entry->action.policer_act]);
		break;
	case MVPP2_ACT_DUMP_FRWD:
		sprintf(action_str, "FRWD: %s", pp2_cls_frwd_action_str[c2_entry->action.frwd_act]);
		break;
	case MVPP2_ACT_DUMP_FLOWID:
		sprintf(action_str, "FLOWID: %s", pp2_cls_flowid_action_str[c2_entry->action.flowid_act]);
		break;
	}

	/* Entry index, including HW index, logical index,and DB index */
	switch (dump_idx) {
	case MVPP2_INDEX_DUMP_TCAM:
		sprintf(index_str, "HW:%03d", c2_index_node->c2_hw_idx);
		break;
	case MVPP2_INDEX_DUMP_LOGICAL:
		sprintf(index_str, "LOG:%04d", c2_index_node->c2_logic_idx);
		break;
	case MVPP2_INDEX_DUMP_DB:
		sprintf(index_str, "DB:%04d", c2_index_node->c2_data_db_idx);
		break;
	}

	if (dump_idx == 0 ||
	    dump_idx <= MVPP2_PORT_DUMP_MASK ||
	    field_match ||
	    dump_idx <= MVPP2_QOS_DUMP_POLICER ||
	    dump_idx <= MVPP2_ACT_DUMP_FRWD ||
	    dump_idx <= MVPP2_MOD_DUMP_L4_CHECKSUM ||
	    dump_idx <= MVPP2_FLOW_DUMP_CNT ||
	    dump_idx <= MVPP2_INDEX_DUMP_DB) {
		printf("+ %8s | %8s | %8s  | %13s | %20s | %30s | %18s | %7s | %8s  +\n",
		       lookup_type_str, internal_pri_str, port_info_str, field_name_str, field_value_str,
		       qos_info_str, action_str, hit_cnt_str, index_str);
		if (field_match)
			printf("+ %8s | %8s | %8s  | %13s | %20s | %30s | %18s | %7s | %9s +\n",
			       empty_str, empty_str, empty_str, empty_str, field_value_mask_str,
			       empty_str, empty_str, empty_str, empty_str);
	}
}

/*******************************************************************************
 * pp2_cls_c2_dump_all
 *
 * DESCRIPTION: The routine will dump the valid C2 entries from C2 sub-module
 *              internal DB.
 * INPUTS:
 *           lookup_type - lookup type entry to dump
 *
 * OUTPUTS:
 *           None
 *
 * COMMENTS:
 *           For debug, if lookup_type equal to 64, means dump all lookup types
 ******************************************************************************/
static void pp2_cls_c2_dump_all(struct pp2_inst *inst, u8 lookup_type)
{
	u32 lkp_type_idx;
	struct pp2_cls_c2_data_t c2_db_data;
	struct pp2_cls_c2_index_t *c2_index_node;
	u32 i;
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);

	/* Print header */
	pp2_cls_valid_c2_entry_header_dump();

	for (lkp_type_idx = 0; lkp_type_idx < MVPP2_C2_LKP_TYPE_MAX; lkp_type_idx++) {
		if (lkp_type_idx != lookup_type &&
		    lookup_type != MVPP2_C2_LKP_TYPE_MAX)
			continue;
		if (list_is_empty(pp2_cls_db_c2_lkp_type_list_head_get(inst, lkp_type_idx)))
			continue;
		/* Traverse lookup type list */
		LIST_FOR_EACH_OBJECT(c2_index_node, struct pp2_cls_c2_index_t,
				     pp2_cls_db_c2_lkp_type_list_head_get(inst, lkp_type_idx), list_node) {
			/* get C2 db entry data */
			if (pp2_cls_db_c2_data_get(inst, c2_index_node->c2_data_db_idx, &c2_db_data))
				continue;
			/* First line */
			for (i = 0; i < sizeof(c2_db_data.field_bm) * BYTE_BITS; i++) {
				pp2_cls_valid_c2_entry_line_dump(cpu_slot,
								 lkp_type_idx,
								 i,
								 &c2_db_data,
								 c2_index_node);
			}
			print_horizontal_line(150, "-");
			printf("\n");
		}
	}
}

/*******************************************************************************
 * pp2_cls_print_free_c2_tcam_dump_head
 *
 * DESCRIPTION: print header for free TCAM.
 * INPUTS:
 *           None
 *
 * OUTPUTS:
 *           None
 *
 * COMMENTS:
 *           None
 ******************************************************************************/
static void pp2_cls_print_free_c2_tcam_dump_head(void)
{
	print_horizontal_line(67, "=");
	printf("=    TOTAL    |                FREE_TCAM_IDX                      =\n");
	print_horizontal_line(67, "=");
}

/*******************************************************************************
 * pp2_cls_print_tcam_index_dump_line
 *
 * DESCRIPTION: print one line with TCAM index information.
 * INPUTS:
 *           common_int - tcam count or lkp_type
 *           tcam_idx   - TCAM index array
 *
 * OUTPUTS:
 *           None
 *
 * COMMENTS:
 *           None
 ******************************************************************************/
static void pp2_cls_print_tcam_index_dump_line(u32 common_int,
					       u8 *tcam_idx)
{
	char common_str[4] = "";
	char tcam_idx_str[40] = "";
	int i, j, tcam_cnt, ten_cnt;
	int line_int_num = 10;
	bool first_line = true;

	/* Para Check */
	if (!tcam_idx)
		return;

	sprintf(common_str, "%03d", common_int);
	for (i = 0; i < MVPP2_C2_ENTRY_MAX; i++) {
		if (tcam_idx[i] == MVPP2_C2_LAST_ENTRY)
			break;
	}
	tcam_cnt = i;
	if (tcam_cnt % line_int_num)
		ten_cnt = tcam_cnt / line_int_num + 1;
	else
		ten_cnt = tcam_cnt / line_int_num;
	/* Handle TCAM count zero */
	if (tcam_cnt == 0)
		ten_cnt = 1;
	for (i = 0; i < ten_cnt; i++) {
		sprintf(tcam_idx_str, "%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d", tcam_idx[i * line_int_num],
			tcam_idx[i * line_int_num + 1], tcam_idx[i * line_int_num + 2], tcam_idx[i * line_int_num + 3],
			tcam_idx[i * line_int_num + 4], tcam_idx[i * line_int_num + 5], tcam_idx[i * line_int_num + 6],
			tcam_idx[i * line_int_num + 7], tcam_idx[i * line_int_num + 8], tcam_idx[i * line_int_num + 9]);
		if ((tcam_cnt % line_int_num) && (i == tcam_cnt / line_int_num)) {
			for (j = ((tcam_cnt % line_int_num) * 4); j < 40; j++)
				tcam_idx_str[j] = 0;
			if (first_line) {
				first_line = false;
				printf("+    %4s     |     %40s      +\n", common_str, tcam_idx_str);
			} else {
				printf("+    %4s     |     %40s      +\n", "", tcam_idx_str);
			}
		} else {
			if (first_line) {
				first_line = false;
				if (tcam_cnt)
					printf("+    %4s     |     %40s      +\n", common_str, tcam_idx_str);
				else
					printf("+    %4s     |     %40s      +\n", common_str, "NA");
			} else {
				printf("+    %4s     |     %40s      +\n", "", tcam_idx_str);
			}
		}
	}
}

/*******************************************************************************
 * pp2_cls_c2_dump_freelist
 *
 * DESCRIPTION: The routine will dump all the free C2 entry number from C2
 *              sub-module internal DB
 * INPUTS:
 *           None
 *
 * OUTPUTS:
 *           None
 *
 * COMMENTS:
 *           For debug.
 ******************************************************************************/
static void pp2_cls_c2_dump_freelist(struct pp2_inst *inst)
{
	struct pp2_cls_c2_index_t *c2_index_node;
	u32 count = 0;
	u8 tcam_array[MVPP2_C2_ENTRY_MAX];

	pp2_cls_print_free_c2_tcam_dump_head();
	/* Traverse free list */
	MVPP2_MEMSET_FF(tcam_array);
	LIST_FOR_EACH_OBJECT(c2_index_node, struct pp2_cls_c2_index_t,
			     pp2_cls_db_c2_free_list_head_get(inst), list_node)
		tcam_array[count++] = c2_index_node->c2_hw_idx;
	/* Print Free TCAM info */
	pp2_cls_print_tcam_index_dump_line(count, tcam_array);
	print_horizontal_line(67, "=");
}

/*******************************************************************************
 * pp2_cls_print_valid_lkp_type_dump_head
 *
 * DESCRIPTION: print header for lookup type.
 * INPUTS:
 *           None
 *
 * OUTPUTS:
 *           None
 *
 * COMMENTS:
 *           None
 ******************************************************************************/
static void pp2_cls_print_valid_lkp_type_dump_head(void)
{
	print_horizontal_line(67, "=");
	printf("=   LKP_Type  |                TCAM_IDX                           =\n");
	print_horizontal_line(67, "=");
}

/*******************************************************************************
 * pp2_cls_c2_dump_lookup_type_list
 *
 * DESCRIPTION: The routine will dump all the valid lookup_type list from C2
 *              sub-module internal DB
 * INPUTS:
 *           None
 *
 * OUTPUTS:
 *           None
 *
 * COMMENTS:
 *           For debug.
 ******************************************************************************/
static void pp2_cls_c2_dump_lookup_type_list(struct pp2_inst *inst)
{
	u32 lkp_type_idx;
	u8 tcam_array[MVPP2_C2_ENTRY_MAX];
	int i;
	struct pp2_cls_c2_index_t *c2_index_node;

	pp2_cls_print_valid_lkp_type_dump_head();
	for (lkp_type_idx = 0; lkp_type_idx < MVPP2_C2_LKP_TYPE_MAX; lkp_type_idx++) {
		if (list_is_empty(pp2_cls_db_c2_lkp_type_list_head_get(inst, lkp_type_idx)))
			continue;
		MVPP2_MEMSET_FF(tcam_array);
		i = 0;
		LIST_FOR_EACH_OBJECT(c2_index_node, struct pp2_cls_c2_index_t,
				     pp2_cls_db_c2_lkp_type_list_head_get(inst, lkp_type_idx), list_node)
			tcam_array[i++] = (u8)(c2_index_node->c2_hw_idx);
		/* Print LKP_type info */
		pp2_cls_print_tcam_index_dump_line(lkp_type_idx, tcam_array);
		printf("+-----------------------------------------------------------------+\n");
	}
	print_horizontal_line(67, "=");
}

/*******************************************************************************
 * pp2_cls_cli_c2_lkp_type_entry_dump
 *
 * DESCRIPTION:
 *           This function dump C2 lookup type entry
 * INPUTS:
 *       buf - Shell parameters as char buffer
 *       len - Number of characters in buffer
 ******************************************************************************/
int pp2_cls_cli_c2_lkp_type_entry_dump(void *arg, int argc, char *argv[])
{
	struct pp2_inst *inst = (struct pp2_inst *)arg;
	int long_index = 0;
	char *ret_ptr;
	int option;
	u32 lookup_type = MVPP2_C2_LKP_TYPE_MAX;
	struct option long_options[] = {
		{"type", required_argument, 0, 't'},
		{0, 0, 0, 0}
	};

	if (argc != 1 && argc != 3) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	if (argc == 3) {
		optind = 0;
		option = getopt_long_only(argc, argv, "", long_options, &long_index);
		if (option == 't') {
			lookup_type = strtoul(optarg, &ret_ptr, 0);
			if ((optarg == ret_ptr) || (lookup_type < 0) || (lookup_type >= MVPP2_C2_LKP_TYPE_MAX)) {
				printf("parsing fail, wrong input for --type\n");
				return -EINVAL;
			}
		} else {
			printf("parsing fail, wrong input\n");
			return -EINVAL;
		}
	}
	pp2_cls_c2_dump_all(inst, lookup_type);

	return 0;
}

/*******************************************************************************
 * pp2_cls_cli_c2_free_tcam_dump
 *
 * DESCRIPTION:
 *           This function dump free C2 tcam entry
 * INPUTS:
 *       buf     - Shell parameters as char buffer
 ******************************************************************************/
int pp2_cls_cli_c2_free_tcam_dump(void *arg, int argc, char *argv[])
{
	int off = 0;
	struct pp2_inst *inst = (struct pp2_inst *)arg;

	pp2_cls_c2_dump_freelist(inst);

	return off;
}

/*******************************************************************************
 * pp2_cls_cli_c2_free_tcam_dump
 *
 * DESCRIPTION:
 *           This function dump C2 valid lookup type and its TCAM index
 * INPUTS:
 *       buf     - Shell parameters as char buffer
 ******************************************************************************/
int pp2_cls_cli_c2_valid_lkp_type_dump(void *arg, int argc, char *argv[])
{
	int off = 0;
	struct pp2_inst *inst = (struct pp2_inst *)arg;

	pp2_cls_c2_dump_lookup_type_list(inst);

	return off;
}

/*******************************************************************************
 * pp2_cls_cli_c2_dump
 *
 * DESCRIPTION:
 *       This function dumps C2 entries
 ******************************************************************************/
int pp2_cls_cli_c2_hw_dump(void *arg, int argc, char *argv[])
{
	struct pp2_inst *inst = (struct pp2_inst *)arg;
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);

	mv_pp2x_c2_hw_dump(cpu_slot);

	return 0;
}

/*******************************************************************************
 * pp2_cls_cli_c2_hit_dump
 *
 * DESCRIPTION:
 *       This function dumps C2 entries
 ******************************************************************************/
int pp2_cls_cli_c2_hw_hit_dump(void *arg, int argc, char *argv[])
{
	struct pp2_cls_hit_cnt_t cntr_info[MVPP2_CLS_C2_TCAM_SIZE];
	u32 num_of_counters = MVPP2_CLS_C2_TCAM_SIZE;
	struct pp2_inst *inst = (struct pp2_inst *)arg;
	int i;

	pp2_cls_c2_hit_cntr_all_get(inst, 0, cntr_info, &num_of_counters);

	for (i = 0; i < num_of_counters; i++) {
		if (cntr_info[i].cntr_val == 0)
			continue;
		pr_info("index = %d, phys_idx = %d, hits = %d\n",
			cntr_info[i].log_idx, cntr_info[i].phys_idx, cntr_info[i].cntr_val);
	}

	return 0;
}

int pp2_cls_cli_qos_dscp_dump(void *arg, int argc, char *argv[])
{
	struct pp2_inst *inst = (struct pp2_inst *)arg;
	int tbl_id, tbl_line, val;
	struct mv_pp2x_cls_c2_qos_entry qos;

	for (tbl_id = 0; tbl_id < MVPP2_CLS_C2_QOS_DSCP_TBL_NUM; tbl_id++) {
		printf("\n------------ DSCP TABLE %d ------------\n", tbl_id);
		printf("LINE	DSCP	COLOR	GEM_ID	QUEUE\n");
		for (tbl_line = 0; tbl_line < MVPP2_CLS_C2_QOS_DSCP_TBL_SIZE; tbl_line++) {
			mv_pp2x_cls_c2_qos_hw_read(&inst->hw, tbl_id, 1/*DSCP*/, tbl_line, &qos);
			printf("0x%2.2x\t", qos.tbl_line);
			mv_pp2x_cls_c2_qos_dscp_get(&qos, &val);
			printf("0x%2.2x\t", val);
			mv_pp2x_cls_c2_qos_color_get(&qos, &val);
			printf("0x%1.1x\t", val);
			mv_pp2x_cls_c2_qos_gpid_get(&qos, &val);
			printf("0x%3.3x\t", val);
			mv_pp2x_cls_c2_qos_queue_get(&qos, &val);
			printf("0x%2.2x", val);
			printf("\n");
		}
	}
	return MV_OK;
}

int pp2_cls_cli_qos_pcp_dump(void *arg, int argc, char *argv[])
{
	struct pp2_inst *inst = (struct pp2_inst *)arg;
	int tbl_id, tbl_line, val;
	struct mv_pp2x_cls_c2_qos_entry qos;

	for (tbl_id = 0; tbl_id < MVPP2_CLS_C2_QOS_PRIO_TBL_NUM; tbl_id++) {
		printf("\n-------- PRIORITY TABLE %d -----------\n", tbl_id);
		printf("LINE	PRIO	COLOR	GEM_ID	QUEUE\n");

		for (tbl_line = 0; tbl_line < MVPP2_CLS_C2_QOS_PRIO_TBL_SIZE; tbl_line++) {
			mv_pp2x_cls_c2_qos_hw_read(&inst->hw, tbl_id, 0/*PRIO*/, tbl_line, &qos);
			printf("0x%2.2x\t", qos.tbl_line);
			mv_pp2x_cls_c2_qos_prio_get(&qos, &val);
			printf("0x%1.1x\t", val);
			mv_pp2x_cls_c2_qos_color_get(&qos, &val);
			printf("0x%1.1x\t", val);
			mv_pp2x_cls_c2_qos_gpid_get(&qos, &val);
			printf("0x%3.3x\t", val);
			mv_pp2x_cls_c2_qos_queue_get(&qos, &val);
			printf("0x%2.2x", val);
			printf("\n");
		}
	}
	return MV_OK;
}
