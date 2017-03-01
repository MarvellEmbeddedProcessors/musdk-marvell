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
 * @file pp2_cls_utils.c
 *
 *  Internal utilities shared by cls sub-modules
 */

/***********************/
/* c file declarations */
/***********************/
#include "std_internal.h"

#include "../pp2_types.h"
#include "../pp2.h"
#include "../pp2_hw_type.h"
#include "../pp2_hw_cls.h"

char g_unknown_str[] = "<unknown>";

static struct pp2_cls_enum_str_t g_enum_eng_name[] = {
	{MVPP2_ENGINE_C2, "C2"},
	{MVPP2_ENGINE_C3_A, "C3_A"},
	{MVPP2_ENGINE_C3_B, "C3_B"},
	{MVPP2_ENGINE_C4, "C4"},
	{MVPP2_ENGINE_C3_HA, "C3_HA"},
	{MVPP2_ENGINE_C3_HB, "C3_HB"},
};

static struct pp2_cls_enum_str_t g_enum_valid[] = {
	{ 0, "no "},
	{ 1, "yes"},
};

static struct pp2_cls_enum_str_t g_enum_field_match[] = {
	{ MVPP2_MATCH_ETH_DST,		"ETH_DST"	},
	{ MVPP2_MATCH_ETH_SRC,		"ETH_SRC"	},
	{ MVPP2_MATCH_VID_OUTER,	"VID_OUTER"	},
	{ MVPP2_MATCH_PBITS_OUTER,	"PBITS_OUTER"	},
	{ MVPP2_MATCH_VID_INNER,	"VID_INNER"	},
	{ MVPP2_MATCH_PBITS_INNER,	"PBITS_INNER"	},
	{ MVPP2_MATCH_ETH_TYPE,		"ETH_TYPE"	},
	{ MVPP2_MATCH_PPPOE_PROTO,	"PPPOE_PROTO"	},
	{ MVPP2_MATCH_PPPOE_SES,	"PPPOE_SES"	},
	{ MVPP2_MATCH_IPV4_PKT,		"IPV4_PKT"	},
	{ MVPP2_MATCH_IPV6_PKT,		"IPV6_PKT"	},
	{ MVPP2_MATCH_IP_SRC,		"IP_SRC"	},
	{ MVPP2_MATCH_IP_DST,		"IP_DST"	},
	{ MVPP2_MATCH_IP_DSCP,		"IP_DSCP"	},
	{ MVPP2_MATCH_IPV6_FLBL,	"IPV6_FLBL"	},
	{ MVPP2_MATCH_IP_PROTO,		"IP_PROTO"	},
	{ MVPP2_MATCH_IP_VERSION,	"IP_VERSION"	},
	{ MVPP2_MATCH_L4_SRC,		"L4_SRC"	},
	{ MVPP2_MATCH_L4_DST,		"L4_DST"	},
	{ MVPP2_MATCH_IPV6_PREF,	"IPV6_PREF"	},
	{ MVPP2_MATCH_IPV6_SUFF,	"IPV6_SUFF"	},
	{ MVPP2_MATCH_ARP_TRGT_IP_ADDR,	"ARP_TRGT_IP_ADDR"},
	{ MVPP2_MATCH_TPID_OUTER,	"TPID_OUTER"	},
	{ MVPP2_MATCH_CFI_OUTER,	"CFI_OUTER"	},
	{ MVPP2_MATCH_TPID_INNER,	"TPID_INNER"	},
	{ MVPP2_MATCH_CFI_INNER,	"CFI_INNER"	},
};

static struct pp2_cls_enum_str_t g_enum_field_id[] = {
	{ MH_FIELD_ID,			"MV_HEADER"	},
	{ GEM_PORT_ID_FIELD_ID,		"GEM_PORT"	},
	{ MH_UNTAGGED_PRI_FIELD_ID,	"MH_PRI"	},
	{ MAC_DA_FIELD_ID,		"MAC_DA"	},
	{ MAC_SA_FIELD_ID,		"MAC_SA"	},
	{ OUT_VLAN_PRI_FIELD_ID,	"OUT_VLAN_PRI"	},
	{ OUT_VLAN_ID_FIELD_ID,		"OUT_VLAN_ID"	},
	{ IN_VLAN_ID_FIELD_ID,		"IN_VLAN_ID"	},
	{ ETH_TYPE_FIELD_ID,		"ETH_TYPE"	},
	{ PPPOE_FIELD_ID,		"PPPOE_SESS"	},
	{ IP_VER_FIELD_ID,		"IP_VER"	},
	{ IPV4_DSCP_FIELD_ID,		"IPV4_DSCP"	},
	{ IPV4_ECN_FIELD_ID,		"IPV4_ECN"	},
	{ IPV4_LEN_FIELD_ID,		"IPV4_LEN"	},
	{ IPV4_TTL_FIELD_ID,		"IPVx_TTL_HOPL"	},
	{ IPV4_PROTO_FIELD_ID,		"IPVx_PROTO"	},
	{ IPV4_SA_FIELD_ID,		"IPV4_SA"	},
	{ IPV4_DA_FIELD_ID,		"IPV4_DA"	},
	{ IPV6_DSCP_FIELD_ID,		"IPV6_DSCP"	},
	{ IPV6_ECN_FIELD_ID,		"IPV6_ECN"	},
	{ IPV6_FLOW_LBL_FIELD_ID,	"IPV6_FLOW_LBL"	},
	{ IPV6_PAYLOAD_LEN_FIELD_ID,	"IPV6_PL_LEN"	},
	{ IPV6_NH_FIELD_ID,		"IPV6_NH"	},
	{ IPV6_SA_FIELD_ID,		"IPV6_SA"	},
	{ IPV6_SA_PREF_FIELD_ID,	"IPV6_SA_PREF"	},
	{ IPV6_SA_SUFF_FIELD_ID,	"IPV6_SA_SUFF"	},
	{ IPV6_DA_FIELD_ID,		"IPV6_DA"	},
	{ IPV6_DA_PREF_FIELD_ID,	"IPV6_DA_PREF"	},
	{ IPV6_DA_SUFF_FIELD_ID,	"IPV6_DA_SUFF"	},
	{ L4_SRC_FIELD_ID,		"L4_SRC_PORT"	},
	{ L4_DST_FIELD_ID,		"L4_DST_PORT"	},
	{ TCP_FLAGS_FIELD_ID,		"TCP_FLAGS"	},
	{ OUT_TPID_FIELD_ID,		"OUT_TPID"	},
	{ OUT_VLAN_CFI_FIELD_ID,	"OUT_VLAN_CFI"	},
	{ IN_TPID_FIELD_ID,		"IN_TPID"	},
	{ IN_VLAN_CFI_FIELD_ID,		"IN_VLAN_CFI"	},
	{ ARP_IPV4_DA_FIELD_ID,		"ARP_IPV4_DA"	}
};

/* source port type string */
static struct pp2_cls_enum_str_t g_pp2_cls_port_type[] = {
	{MVPP2_SRC_PORT_TYPE_PHY,	"PHY"},
	{MVPP2_SRC_PORT_TYPE_UNI,	"UNI"},
	{MVPP2_SRC_PORT_TYPE_VIR,	"VIRT"}
};

/* layer 4 type string */
static struct pp2_cls_enum_str_t g_pp2_cls_l4_type[] = {
	{MVPP2_L4_TYPE_TCP,	"TCP"},
	{MVPP2_L4_TYPE_UDP,	"UDP"},
};

/* QoS action string */
static struct pp2_cls_enum_str_t g_pp2_cls_qos_action_type[] = {
	{MVPP2_COLOR_ACTION_TYPE_NO_UPDT,	"No_Updt"},
	{MVPP2_COLOR_ACTION_TYPE_NO_UPDT_LOCK,	"No_Updt_L"},
	{MVPP2_COLOR_ACTION_TYPE_GREEN,		"Green"},
	{MVPP2_COLOR_ACTION_TYPE_YELLOW,	"Yellow"},
	{MVPP2_COLOR_ACTION_TYPE_RED,		"Red"},
	{MVPP2_COLOR_ACTION_TYPE_RED_LOCK,	"Red_L"},
};

/* common action string */
static struct pp2_cls_enum_str_t g_pp2_cls_common_action_type[] = {
	{MVPP2_ACTION_TYPE_NO_UPDT,		"No_Updt"},
	{MVPP2_ACTION_TYPE_NO_UPDT_LOCK,	"No_Updt_L"},
	{MVPP2_ACTION_TYPE_UPDT,		"Update"},
	{MVPP2_ACTION_TYPE_UPDT_LOCK,		"Update_L"},
};

/* flow ID action string */
static struct pp2_cls_enum_str_t g_pp2_cls_flow_id_action_type[] = {
	{MVPP2_ACTION_FLOWID_DISABLE,	"Disable"},
	{MVPP2_ACTION_FLOWID_ENABLE,	"Enable"},
};

/* forwarding action string */
static struct pp2_cls_enum_str_t g_pp2_cls_frwd_action_type[] = {
	{MVPP2_FRWD_ACTION_TYPE_NO_UPDT,		"No_Updt"},
	{MVPP2_FRWD_ACTION_TYPE_NO_UPDT_LOCK,		"No_Updt_L"},
	{MVPP2_FRWD_ACTION_TYPE_SWF,			"SWF"},
	{MVPP2_FRWD_ACTION_TYPE_SWF_LOCK,		"SWF_L"},
	{MVPP2_FRWD_ACTION_TYPE_HWF,			"HWF"},
	{MVPP2_FRWD_ACTION_TYPE_HWF_LOCK,		"HWF_L"},
	{MVPP2_FRWD_ACTION_TYPE_HWF_LOW_LATENCY,	"HWF_Lat"},
	{MVPP2_FRWD_ACTION_TYPE_HWF_LOW_LATENCY_LOCK,	"HWF_Lat_L"},
};

/* scan mode */
static struct pp2_cls_enum_str_t g_pp2_cls_scan_mode[] = {
	{MVPP2_SCAN_BELOW_THRESHOLD,	"below threshold"},
	{MVPP2_SCAN_ABOVE_THRESHOLD,	"above threshold"},
};

static struct pp2_cls_enum_str_t g_enum_tbl_action_type[] = {
	{ PP2_CLS_TBL_ACT_DROP, "DROP"},
	{ PP2_CLS_TBL_ACT_DONE, "IN_QUEUE"},
};

/******************************************************************************
 *
 * Function   : common_mask_gen
 *
 * Description: generate mask according to bits number
 *
 * Parameters :
 * INPUT bit_num - bit num needed
 * OUTPUT None
 * Returns    : mask
 * Comments: for example, bit_num = 2, mask = 0x3(b00000011)
 *****************************************************************************/
u32 common_mask_gen(int bit_num)
{
	u32 temp = 0x1;
	int i;

	/* para check */
	if (bit_num < 0 ||
	    bit_num > (sizeof(u32) * BYTE_BITS))
		return 0;

	if (bit_num == 0)
		return 0;

	for (i = 1; i < bit_num; i++) {
		temp = temp << 1;
		temp |= 0x1;
	}

	return temp;
}

/******************************************************************************
 * lookup_enum_str()
 *
 * DESCRIPTION:
 *	This routine lookups enum string according to enum value
 *
 * INPUTS:
 *	enum_str   - enum string array
 *	enum_num   - enum number
 *	enum_value - the enum value to be matched
 *
 * OUTPUTS:
 *	None
 *
 * RETURNS:
 *	Enum string
 ******************************************************************************/
static const char *lookup_enum_str(struct pp2_cls_enum_str_t enum_str[], int enum_num, int enum_value)
{
	int idx;

	for (idx = 0; idx < enum_num; idx++) {
		if (enum_value == enum_str[idx].enum_value)
			return enum_str[idx].enum_str;
	}
	return g_unknown_str;
}

const char *pp2_utils_eng_no_str_get(int value)
{
	return lookup_enum_str(g_enum_eng_name, MVPP2_MEMBER_NUM(g_enum_eng_name), value);
}

const char *pp2_utils_field_id_str_get(int value)
{
	return lookup_enum_str(g_enum_field_id, MVPP2_MEMBER_NUM(g_enum_field_id), value);
}

const char *pp2_utils_valid_str_get(int value)
{
	return lookup_enum_str(g_enum_valid, MVPP2_MEMBER_NUM(g_enum_valid), value);
}

const char *pp2_cls_utils_field_match_str_get(int value)
{
	return lookup_enum_str(g_enum_field_match, MVPP2_MEMBER_NUM(g_enum_field_match), value);
}

const char *pp2_cls_utils_field_id_str_get(int value)
{
	return lookup_enum_str(g_enum_field_id, MVPP2_MEMBER_NUM(g_enum_field_id), value);
}

const char *pp2_cls_utils_port_type_str_get(int value)
{
	return lookup_enum_str(g_pp2_cls_port_type, MVPP2_MEMBER_NUM(g_pp2_cls_port_type), value);
}

const char *pp2_cls_utils_l4_type_str_get(int value)
{
	return lookup_enum_str(g_pp2_cls_l4_type, MVPP2_MEMBER_NUM(g_pp2_cls_l4_type), value);
}

const char *pp2_cls_utils_qos_action_str_get(int value)
{
	return lookup_enum_str(g_pp2_cls_qos_action_type, MVPP2_MEMBER_NUM(g_pp2_cls_qos_action_type), value);
}

const char *pp2_cls_utils_common_action_str_get(int value)
{
	return lookup_enum_str(g_pp2_cls_common_action_type, MVPP2_MEMBER_NUM(g_pp2_cls_common_action_type), value);
}

const char *pp2_cls_utils_flow_id_action_str_get(int value)
{
	return lookup_enum_str(g_pp2_cls_flow_id_action_type, MVPP2_MEMBER_NUM(g_pp2_cls_flow_id_action_type), value);
}

const char *pp2_cls_utils_frwd_action_str_get(int value)
{
	return lookup_enum_str(g_pp2_cls_frwd_action_type, MVPP2_MEMBER_NUM(g_pp2_cls_frwd_action_type), value);
}

const char *pp2_cls_utils_scan_mode_str_get(int value)
{
	return lookup_enum_str(g_pp2_cls_scan_mode, MVPP2_MEMBER_NUM(g_pp2_cls_scan_mode), value);
}

const char *pp2_cls_utils_tbl_action_type_str_get(int value)
{
	return lookup_enum_str(g_enum_tbl_action_type, MVPP2_MEMBER_NUM(g_enum_tbl_action_type), value);
}

void print_horizontal_line(u32 char_count, const char *char_val)
{
	u32 cnt;

	for (cnt = 0; cnt < char_count; cnt++)
		printf("%s", char_val);
	printf("\n");
}

