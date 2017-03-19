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

/***********************/
/* c file declarations */
/***********************/
#include <arpa/inet.h>
#include "std_internal.h"
#include "drivers/ppv2/pp2.h"
#include "drivers/ppv2/pp2_hw_type.h"
#include "drivers/ppv2/pp2_hw_cls.h"
#include "pp2_cls_types.h"
#include "pp2_cls_internal_types.h"
#include "pp2_cls_common.h"
#include "pp2_flow_rules.h"
#include "pp2_cls_db.h"
#include "drivers/mv_pp2_cls.h"
#include "../pp2_hw_cls.h"
#include "pp2_cls_mng.h"
#include "pp2_prs.h"


#define MVPP2_CLS_PROTO_SHIFT	MVPP2_CLS_PROTO_SHIFT
#define NOT_SUPPORTED_YET 255

struct cls_field_convert_t {
	u32 proto;
	u32 field;
	u32 field_to_config;
	u32 match_bm;
};

static struct cls_field_convert_t g_cls_field_convert[] = {
	/* ethernet, source address */
	{MV_NET_PROTO_ETH, MV_NET_ETH_F_SA, MAC_SA_FIELD_ID, MVPP2_MATCH_ETH_SRC},
	/* ethernet, destination address */
	{MV_NET_PROTO_ETH, MV_NET_ETH_F_DA, MAC_DA_FIELD_ID, MVPP2_MATCH_ETH_DST},
	/* ethernet, type */
	{MV_NET_PROTO_ETH, MV_NET_ETH_F_TYPE, ETH_TYPE_FIELD_ID, MVPP2_MATCH_ETH_TYPE},
	/* vlan, priority (outer vlan) */
	{MV_NET_PROTO_VLAN, MV_NET_VLAN_F_PRI, OUT_VLAN_PRI_FIELD_ID, MVPP2_MATCH_PBITS_OUTER},
	/* vlan, id (outer vlan) */
	{MV_NET_PROTO_VLAN, MV_NET_VLAN_F_ID, OUT_VLAN_ID_FIELD_ID, MVPP2_MATCH_VID_OUTER},
	/* vlan, tci */
	{MV_NET_PROTO_VLAN, MV_NET_VLAN_F_TCI, NOT_SUPPORTED_YET, 0},
	/* pppoe */
	{MV_NET_PROTO_PPPOE, 0, PPPOE_FIELD_ID, MVPP2_MATCH_PPPOE_PROTO},
	/* ip */
	{MV_NET_PROTO_IP, 0, IP_VER_FIELD_ID, MVPP2_MATCH_IP_VERSION},
	/* ipv4, tos  [AW: check] */
	{MV_NET_PROTO_IP4, MV_NET_IP4_F_TOS, IPV4_DSCP_FIELD_ID, MVPP2_MATCH_IP_DSCP},
	/* ipv4, souce address */
	{MV_NET_PROTO_IP4, MV_NET_IP4_F_SA, IPV4_SA_FIELD_ID, MVPP2_MATCH_IP_SRC},
	/* ipv4, destination address */
	{MV_NET_PROTO_IP4, MV_NET_IP4_F_DA, IPV4_DA_FIELD_ID, MVPP2_MATCH_IP_DST},
	/* ipv4, proto */
	{MV_NET_PROTO_IP4, MV_NET_IP4_F_PROTO, IPV4_PROTO_FIELD_ID, MVPP2_MATCH_IP_PROTO},
	/* ipv6, tc */
	{MV_NET_PROTO_IP6, MV_NET_IP6_F_TC, NOT_SUPPORTED_YET, 0},
	/* ipv6, souce address */
	{MV_NET_PROTO_IP6, MV_NET_IP6_F_SA, IPV6_SA_FIELD_ID, MVPP2_MATCH_IP_SRC},
	/* ipv6, destination address */
	{MV_NET_PROTO_IP6, MV_NET_IP6_F_DA, IPV6_DA_FIELD_ID, MVPP2_MATCH_IP_DST},
	/* ipv6, flow */
	{MV_NET_PROTO_IP6, MV_NET_IP6_F_FLOW, IPV6_FLOW_LBL_FIELD_ID, MVPP2_MATCH_IPV6_FLBL},
	 /* ipv6, next header */
	{MV_NET_PROTO_IP6, MV_NET_IP6_F_NEXT_HDR, IPV6_NH_FIELD_ID, MVPP2_MATCH_IP_PROTO},
	/* layer4, source port */
	{MV_NET_PROTO_L4, MV_NET_L4_F_SP, L4_SRC_FIELD_ID, MVPP2_MATCH_L4_SRC},
	/* layer4, destination port */
	{MV_NET_PROTO_L4, MV_NET_L4_F_DP, L4_DST_FIELD_ID, MVPP2_MATCH_L4_DST},
	/* layer4, checksum */
	{MV_NET_PROTO_L4, MV_NET_L4_F_CSUM, NOT_SUPPORTED_YET, 0},
	/* tcp, source port */
	{MV_NET_PROTO_TCP, MV_NET_TCP_F_SP, L4_SRC_FIELD_ID, MVPP2_MATCH_L4_SRC},
	/* tcp, destination port */
	{MV_NET_PROTO_TCP, MV_NET_TCP_F_DP, L4_DST_FIELD_ID, MVPP2_MATCH_L4_DST},
	/* tcp, checksum */
	{MV_NET_PROTO_TCP, MV_NET_TCP_F_CSUM, NOT_SUPPORTED_YET, 0},
	/* udp, source port */
	{MV_NET_PROTO_UDP, MV_NET_UDP_F_SP, L4_SRC_FIELD_ID, MVPP2_MATCH_L4_SRC},
	/* udp, destination port */
	{MV_NET_PROTO_UDP, MV_NET_UDP_F_DP, L4_DST_FIELD_ID, MVPP2_MATCH_L4_DST},
	/* udp, checksum */
	{MV_NET_PROTO_UDP, MV_NET_UDP_F_CSUM, NOT_SUPPORTED_YET, 0},
	/* icmp */
	{MV_NET_PROTO_ICMP, 0, NOT_SUPPORTED_YET, 0},
	/* arp */
	{MV_NET_PROTO_ARP, 0, ARP_IPV4_DA_FIELD_ID, MVPP2_MATCH_ARP_TRGT_IP_ADDR},
};

static u32 lookup_field_id(u32 proto, u32 field, u32 *field_id, u32 *match_bm)
{
	u32 idx;

	for (idx = 0; idx < MVPP2_MEMBER_NUM(g_cls_field_convert); idx++) {
		if ((proto == g_cls_field_convert[idx].proto) &&
		    (field == g_cls_field_convert[idx].field)) {
			*field_id = g_cls_field_convert[idx].field_to_config;
			*match_bm = g_cls_field_convert[idx].match_bm;
			return 0;
		}
	}
	return -ENOENT;
}

static int pp2_cls_add_non_ip_logical_id(u16 *select_logical_id)
{
	int i = 0;

	select_logical_id[i++] = MVPP2_PRS_FL_NON_IP_UNTAG;
	select_logical_id[i++] = MVPP2_PRS_FL_NON_IP_TAG;

	return i;
}

static int pp2_cls_add_ip4_logical_id(u16 *select_logical_id)
{
	int i = 0;

	select_logical_id[i++] = MVPP2_PRS_FL_IP4_UNTAG;
	select_logical_id[i++] = MVPP2_PRS_FL_IP4_TAG;

	return i;
}

static int pp2_cls_add_ip4_udp_logical_id(u16 *select_logical_id)
{
	int i = 0;

	select_logical_id[i++] = MVPP2_PRS_FL_IP4_UDP_NF_TAG;
	select_logical_id[i++] = MVPP2_PRS_FL_IP4_UDP_NF_UNTAG;
	select_logical_id[i++] = MVPP2_PRS_FL_IP4_UDP_FRAG_UNTAG;
	select_logical_id[i++] = MVPP2_PRS_FL_IP4_UDP_FRAG_TAG;

	return i;
}

static int pp2_cls_add_ip4_tcp_logical_id(u16 *select_logical_id)
{
	int i = 0;

	select_logical_id[i++] = MVPP2_PRS_FL_IP4_TCP_NF_UNTAG;
	select_logical_id[i++] = MVPP2_PRS_FL_IP4_TCP_NF_TAG;
	select_logical_id[i++] = MVPP2_PRS_FL_IP4_TCP_FRAG_UNTAG;
	select_logical_id[i++] = MVPP2_PRS_FL_IP4_TCP_FRAG_TAG;

	return i;
}

static int pp2_cls_add_ip6_logical_id(u16 *select_logical_id)
{
	int i = 0;

	select_logical_id[i++] = MVPP2_PRS_FL_IP6_UNTAG;
	select_logical_id[i++] = MVPP2_PRS_FL_IP6_TAG;

	return i;
}

static int pp2_cls_add_ip6_udp_logical_id(u16 *select_logical_id)
{
	int i = 0;

	select_logical_id[i++] = MVPP2_PRS_FL_IP6_UDP_NF_UNTAG;
	select_logical_id[i++] = MVPP2_PRS_FL_IP6_UDP_NF_TAG;
	select_logical_id[i++] = MVPP2_PRS_FL_IP6_UDP_FRAG_UNTAG;
	select_logical_id[i++] = MVPP2_PRS_FL_IP6_UDP_FRAG_TAG;

	return i;
}

static int pp2_cls_add_ip6_tcp_logical_id(u16 *select_logical_id)
{
	int i = 0;

	select_logical_id[i++] = MVPP2_PRS_FL_IP6_TCP_NF_UNTAG;
	select_logical_id[i++] = MVPP2_PRS_FL_IP6_TCP_NF_TAG;
	select_logical_id[i++] = MVPP2_PRS_FL_IP6_TCP_FRAG_UNTAG;
	select_logical_id[i++] = MVPP2_PRS_FL_IP6_TCP_FRAG_TAG;

	return i;
}

/*
 * pp2_cls_mng_add_default_flow()
 * add default flow&rule for all lookup id
 */
static int pp2_cls_mng_add_default_flow(struct pp2_ppio *ppio)
{
	struct pp2_cls_tbl_params tbl_params;
	struct pp2_cls_tbl_rule rule;
	struct pp2_cls_tbl *tbl;

	/* add default flow for all lkpid */
	tbl_params.type = PP2_CLS_TBL_MASKABLE;
	tbl_params.max_num_rules = 1;
	tbl_params.key.key_size = 0;
	tbl_params.key.num_fields = 0;

	tbl_params.default_act.cos = kmalloc((sizeof(*tbl_params.default_act.cos)), GFP_KERNEL);
	if (!tbl_params.default_act.cos)
		return -ENOMEM;

	tbl_params.default_act.type = PP2_CLS_TBL_ACT_DONE;
	tbl_params.default_act.cos->ppio = ppio;
	tbl_params.default_act.cos->tc = 0;

	pp2_cls_mng_tbl_init(&tbl_params, &tbl, MVPP2_CLS_LKP_MUSDK_LOG_PORT_DEF);

	/* add default c2 rule */
	rule.num_fields = 0;
	pp2_cls_mng_rule_add(tbl, &rule, &tbl_params.default_act, MVPP2_CLS_LKP_MUSDK_LOG_PORT_DEF);

	kfree(tbl_params.default_act.cos);

	return 0;
}

/*
 * pp2_cls_mng_set_logical_port_params()
 * configure parser and default flow for logical port
 */
int pp2_cls_mng_set_logical_port_params(struct pp2_ppio *ppio, struct pp2_ppio_params *params)
{
	struct pp2_port *port = GET_PPIO_PORT(ppio);
	int rc;

	rc = pp2_cls_mng_add_default_flow(ppio);
	if (rc) {
		pr_err("%s(%d) pp2_cls_mng_add_default_flow_for_log fail\n", __func__, __LINE__);
		return -EINVAL;
	}

	rc = pp2_prs_eth_start_hdr_set(port, params->eth_start_hdr);
	if (rc) {
		pr_err("%s(%d) pp2_prs_eth_start_hdr_set fail\n", __func__, __LINE__);
		return -EINVAL;
	}

	rc = pp2_prs_set_log_port(port, &params->specific_type_params.log_port_params);
	if (rc) {
		pr_err("%s(%d) pp2_prs_set_log_port fail\n", __func__, __LINE__);
		return -EINVAL;
	}

	return 0;
}

int pp2_cls_mng_tbl_init(struct pp2_cls_tbl_params *params, struct pp2_cls_tbl **tbl, int lkp_type)
{
	struct pp2_cls_fl_rule_list_t *fl_rls;
	struct pp2_ppio *ppio;
	struct pp2_port *port;
	struct pp2_inst *inst;
	struct pp2_cls_tbl *tbl_node = NULL;
	struct pp2_cls_cos_desc *cos;

	u32 idx;
	u32 field, match_bm;
	u32 rc = 0;
	u32 i, j = 0, field_index;
	u32 five_tuple = 0;
	u32 ipv4_flag = 0;
	u32 ipv6_flag = 0;
	u32 tcp_flag = 0;
	u32 udp_flag = 0;
	u32 l4_flag = 0;
	u16 select_logical_id[30];

	/* Para check */
	if (mv_pp2x_ptr_validate(params))
		return -EINVAL;

	if (mv_pp2x_range_validate(params->key.num_fields, 0, PP2_CLS_TBL_MAX_NUM_FIELDS))
		return -EINVAL;
	pr_debug("key.num_fields = %d\n", params->key.num_fields);

	fl_rls = kmalloc(sizeof(*fl_rls), GFP_KERNEL);
	if (!fl_rls)
		return -ENOMEM;

	/* get packet processor instance */
	ppio = params->default_act.cos->ppio;
	port = GET_PPIO_PORT(ppio);
	inst = port->parent;

	fl_rls->fl_len = 1;
	field_index = 0;

	/* parse the protocol and protocol fields */
	for (idx = 0; idx < params->key.num_fields; idx++) {
		rc = lookup_field_id(params->key.proto_field[idx].proto,
				     params->key.proto_field[idx].field.eth, &field, &match_bm);
		if (rc) {
			pr_err("%s(%d) lookup id error!\n", __func__, __LINE__);
			goto end;
		}
		if (params->key.proto_field[idx].proto == MV_NET_PROTO_IP4) {
			ipv4_flag = 1;
			if ((params->key.proto_field[idx].field.ipv4 == MV_NET_IP4_F_PROTO) &&
			    (params->key.num_fields == PP2_CLS_TBL_MAX_NUM_FIELDS))
				five_tuple = 1;
		} else if (params->key.proto_field[idx].proto == MV_NET_PROTO_IP6) {
			ipv6_flag = 1;
			if ((params->key.proto_field[idx].field.ipv6 == MV_NET_IP6_F_NEXT_HDR) &&
			    (params->key.num_fields == PP2_CLS_TBL_MAX_NUM_FIELDS))
				five_tuple = 1;
		}
		if (params->key.proto_field[idx].proto == MV_NET_PROTO_TCP)
			tcp_flag = 1;
		else if (params->key.proto_field[idx].proto == MV_NET_PROTO_UDP)
			udp_flag = 1;
		else if (params->key.proto_field[idx].proto == MV_NET_PROTO_L4)
			l4_flag = 1;

		fl_rls->fl[0].field_id[field_index++] = field;
	}

	/* engine selection */
	if (params->type == PP2_CLS_TBL_MASKABLE) {
		if (five_tuple) {
			pr_err("%s(%d) maskable engine doesn't support 5 tuples!\n", __func__, __LINE__);
			return -EINVAL;
		}
		fl_rls->fl[0].engine = MVPP2_CLS_ENGINE_C2;
	} else if (params->type == PP2_CLS_TBL_EXACT_MATCH) {
		if (five_tuple)
			fl_rls->fl[0].engine = MVPP2_CLS_ENGINE_C3B;
		else
			fl_rls->fl[0].engine = MVPP2_CLS_ENGINE_C3A;
	} else {
		pr_err("%s(%d) unknown engine type!\n", __func__, __LINE__);
		return -EINVAL;
	}

	/* port type - TODO fixed to PHY for now */
	fl_rls->fl[0].port_type = MVPP2_SRC_PORT_TYPE_PHY;

	/* port ID - TODO set it fixed to 1. this value is used only if
	 * PortIdSelect bit in CLS_FLOW_TBL1 register is set to 0
	 */
	fl_rls->fl[0].port_bm = (1 << port->id);

	/* lookup_type */
	fl_rls->fl[0].lu_type = lkp_type;
	fl_rls->fl[0].enabled = true;
	fl_rls->fl[0].prio = lkp_type == MVPP2_CLS_LKP_MUSDK_CLS ? MVPP2_CLS_MUSDK_CLS_PRIO : MVPP2_CLS_MUSDK_DEF_PRIO;
	fl_rls->fl[0].udf7 = port->type == PP2_PPIO_T_LOG ? MVPP2_CLS_MUSDK_LOG_UDF7 : MVPP2_CLS_MUSDK_NIC_UDF7;
	fl_rls->fl[0].seq_ctrl = MVPP2_CLS_DEF_SEQ_CTRL;
	fl_rls->fl[0].field_id_cnt = params->key.num_fields - (fl_rls->fl[0].engine == MVPP2_CLS_ENGINE_C3B);

	pr_debug("ipv4_flag = %d\n", ipv4_flag);
	pr_debug("ipv6_flag = %d\n", ipv6_flag);
	pr_debug("l4_flag = %d\n", l4_flag);
	pr_debug("udp_flag = %d\n", udp_flag);
	pr_debug("tcp_flag = %d\n", tcp_flag);

	/* find relevant lkpid for this flow */
	if (!ipv4_flag && !ipv6_flag && !l4_flag && !tcp_flag && !udp_flag) {
		j += pp2_cls_add_non_ip_logical_id(&select_logical_id[j]);
		j += pp2_cls_add_ip4_logical_id(&select_logical_id[j]);
		j += pp2_cls_add_ip4_tcp_logical_id(&select_logical_id[j]);
		j += pp2_cls_add_ip4_udp_logical_id(&select_logical_id[j]);
		j += pp2_cls_add_ip6_logical_id(&select_logical_id[j]);
		j += pp2_cls_add_ip6_tcp_logical_id(&select_logical_id[j]);
		j += pp2_cls_add_ip6_udp_logical_id(&select_logical_id[j]);
	} else if (ipv4_flag) {
		if (!tcp_flag && !udp_flag && !l4_flag) {
			j += pp2_cls_add_ip4_logical_id(&select_logical_id[j]);
			j += pp2_cls_add_ip4_tcp_logical_id(&select_logical_id[j]);
			j += pp2_cls_add_ip4_udp_logical_id(&select_logical_id[j]);
		} else if (l4_flag && !tcp_flag && !udp_flag) {
			j += pp2_cls_add_ip4_tcp_logical_id(&select_logical_id[j]);
			j += pp2_cls_add_ip4_udp_logical_id(&select_logical_id[j]);
		} else if (tcp_flag) {
			j += pp2_cls_add_ip4_tcp_logical_id(&select_logical_id[j]);
		} else if (udp_flag) {
			j += pp2_cls_add_ip4_udp_logical_id(&select_logical_id[j]);
		} else {
			pr_err("%s(%d), failed to calculate lkpid\n", __func__, __LINE__);
			return -EINVAL;
		}
	} else if (ipv6_flag) {
		if (!tcp_flag && !udp_flag && !l4_flag) {
			j += pp2_cls_add_ip6_logical_id(&select_logical_id[j]);
			j += pp2_cls_add_ip6_tcp_logical_id(&select_logical_id[j]);
			j += pp2_cls_add_ip6_udp_logical_id(&select_logical_id[j]);
		} else if (l4_flag && !tcp_flag && !udp_flag) {
			j += pp2_cls_add_ip6_tcp_logical_id(&select_logical_id[j]);
			j += pp2_cls_add_ip6_udp_logical_id(&select_logical_id[j]);
		} else if (tcp_flag) {
			j += pp2_cls_add_ip6_tcp_logical_id(&select_logical_id[j]);
		} else if (udp_flag) {
			j += pp2_cls_add_ip6_udp_logical_id(&select_logical_id[j]);
		} else {
			pr_err("%s(%d), failed to calculate lkpid\n", __func__, __LINE__);
			return -EINVAL;
		}
	} else if (l4_flag) {
		j += pp2_cls_add_ip4_tcp_logical_id(&select_logical_id[j]);
		j += pp2_cls_add_ip4_udp_logical_id(&select_logical_id[j]);
		j += pp2_cls_add_ip6_tcp_logical_id(&select_logical_id[j]);
		j += pp2_cls_add_ip6_udp_logical_id(&select_logical_id[j]);
	} else if (tcp_flag) {
		j += pp2_cls_add_ip4_tcp_logical_id(&select_logical_id[j]);
		j += pp2_cls_add_ip6_tcp_logical_id(&select_logical_id[j]);
	} else if (udp_flag) {
		j += pp2_cls_add_ip4_udp_logical_id(&select_logical_id[j]);
		j += pp2_cls_add_ip6_udp_logical_id(&select_logical_id[j]);
	} else {
		pr_err("%s(%d), failed to calculate lkpid\n", __func__, __LINE__);
		return -EINVAL;
	}
	select_logical_id[j] = 0;

	/* add current rule for all selected logical flow id */
	for (i = 0; select_logical_id[i] != 0; i++) {
		pr_debug("select_logical_id = %d\n", select_logical_id[i]);
		fl_rls->fl[0].fl_log_id = select_logical_id[i];

		/* Add flow rule */
		pp2_cls_lkp_dcod_set_and_disable(inst, select_logical_id[i]);
		rc = pp2_cls_fl_rule_add(inst, fl_rls);
		if (rc) {
			pr_err("failed to add cls flow rule\n");
			goto end;
		}
		pp2_cls_lkp_dcod_enable(inst, select_logical_id[i]);
	}

	/* add flow to list db */
	rc = pp2_cls_db_mng_tbl_add(&tbl_node);
	tbl_node->params.max_num_rules = params->max_num_rules;
	tbl_node->params.type = params->type;
	tbl_node->params.default_act.type = params->default_act.type;
	cos = kmalloc(sizeof(*cos), GFP_KERNEL);
	if (!cos) {
		pr_err("%s(%d) no mem for pp2_cls_cos_desc!\n", __func__, __LINE__);
		return -ENOMEM;
	}
	tbl_node->params.default_act.cos = cos;
	tbl_node->params.default_act.cos->ppio = params->default_act.cos->ppio;
	tbl_node->params.default_act.cos->tc = params->default_act.cos->tc;

	tbl_node->params.key.key_size = params->key.key_size;
	tbl_node->params.key.num_fields = params->key.num_fields;
	for (i = 0; i < params->key.num_fields; i++) {
		tbl_node->params.key.proto_field[i].proto = params->key.proto_field[i].proto;
		tbl_node->params.key.proto_field[i].field = params->key.proto_field[i].field;
	}
	*tbl = tbl_node;

end:
	kfree(fl_rls);

	return rc;
}

int pp2_cls_mng_table_deinit(struct pp2_cls_tbl *tbl)
{
	int i;
	struct pp2_cls_tbl_rule *rule = NULL;
	u32 rc;

	/* Remove configured rules in table */
	for (i = 0; i < tbl->params.max_num_rules; i++) {
		rc = pp2_cls_db_mng_tbl_rule_next_get(tbl, &rule);
		if (!rc)
			pp2_cls_mng_rule_remove(tbl, rule);
	}

	/*TODO remove flow from HW */

	/* Remove rules and table from database */
	kfree(tbl->params.default_act.cos);

	rc = pp2_cls_db_mng_tbl_remove(tbl);
	if (rc) {
		pr_err("%s(%d) Error while removing table from db!\n", __func__, __LINE__);
		return -EFAULT;
	}
	return 0;
}

static int pp2_cls_set_rule_info(struct pp2_cls_mng_pkt_key_t *mng_pkt_key,
				 struct mv_pp2x_engine_pkt_action *pkt_action,
				 struct mv_pp2x_qos_value *pkt_qos,
				 struct mv_pp2x_src_port *rule_port,
				 struct pp2_cls_tbl_params *params,
				 struct pp2_cls_tbl_rule *rule,
				 struct pp2_cls_tbl_action *action,
				 struct pp2_port *port)
{
	char *ret_ptr;
	char *mask_ptr;
	char mask_arr[3];
	int rc = 0, i;
	u32 proto_flag = 0;
	u32 ipv4_flag = 0;
	u32 ipv6_flag = 0;
	u32 field;
	u32 idx1, idx2;
	u32 field_bm = 0, bm = 0;
	u8 queue;

	rule_port->port_type = MVPP2_SRC_PORT_TYPE_PHY;
	rule_port->port_value = (1 << port->id);
	rule_port->port_mask = 0xff;

	/* parse the protocol and protocol fields */
	for (idx1 = 0; idx1 < params->key.num_fields; idx1++) {
		rc = lookup_field_id(params->key.proto_field[idx1].proto, params->key.proto_field[idx1].field.eth,
				     &field, &bm);
		if (rc) {
			pr_err("%s(%d) lookup id error!\n", __func__, __LINE__);
			return -EINVAL;
		}

		if (field == NOT_SUPPORTED_YET) {
			pr_err("%s(%d) protocol_field not supported yet - skipping!\n", __func__, __LINE__);
			continue;
		}
		if (params->key.proto_field[idx1].proto == MV_NET_PROTO_IP4) {
			ipv4_flag = 1;
			field_bm |= MVPP2_MATCH_IPV4_PKT;
		}
		if (params->key.proto_field[idx1].proto == MV_NET_PROTO_IP6) {
			ipv6_flag = 1;
			field_bm |= MVPP2_MATCH_IPV6_PKT;
		}
		field_bm |= bm;

		switch (field) {
		case MAC_SA_FIELD_ID:
			if (rule->fields[idx1].size != (GET_NUM_BYTES(MAC_DA_FIELD_SIZE))) {
				pr_err("%s(%d) field size does not match! %d\n", __func__, __LINE__,
				       rule->fields[idx1].size);
				return -EINVAL;
			}
			rc = mv_pp2x_parse_mac_address((char *)rule->fields[idx1].key,
						       &mng_pkt_key->pkt_key->eth_src.eth_add[0]);
			if (rc < 0) {
				pr_err("Unable to parse MAC SA\n");
				return -EINVAL;
			}
			pr_debug("MAC SA: %02x:%02x:%02x:%02x:%02x:%02x\n",
				 mng_pkt_key->pkt_key->eth_src.eth_add[0],
				 mng_pkt_key->pkt_key->eth_src.eth_add[1],
				 mng_pkt_key->pkt_key->eth_src.eth_add[2],
				 mng_pkt_key->pkt_key->eth_src.eth_add[3],
				 mng_pkt_key->pkt_key->eth_src.eth_add[4],
				 mng_pkt_key->pkt_key->eth_src.eth_add[5]);
			rc = mv_pp2x_parse_mac_address((char *)rule->fields[idx1].mask,
						       &mng_pkt_key->pkt_key->eth_src.eth_add_mask[0]);
			if (rc < 0) {
				pr_err("Unable to parse MAC SA mask\n");
				return -EINVAL;
			}
			pr_debug("MAC SA MASK: %02x:%02x:%02x:%02x:%02x:%02x\n",
				 mng_pkt_key->pkt_key->eth_src.eth_add_mask[0],
				 mng_pkt_key->pkt_key->eth_src.eth_add_mask[1],
				 mng_pkt_key->pkt_key->eth_src.eth_add_mask[2],
				 mng_pkt_key->pkt_key->eth_src.eth_add_mask[3],
				 mng_pkt_key->pkt_key->eth_src.eth_add_mask[4],
				 mng_pkt_key->pkt_key->eth_src.eth_add_mask[5]);
			break;
		case MAC_DA_FIELD_ID:
			if (rule->fields[idx1].size != (GET_NUM_BYTES(MAC_SA_FIELD_SIZE))) {
				pr_err("%s(%d) field size does not match! %d\n", __func__, __LINE__,
				       rule->fields[idx1].size);
				return -EINVAL;
			}
			rc = mv_pp2x_parse_mac_address((char *)rule->fields[idx1].key,
						       &mng_pkt_key->pkt_key->eth_dst.eth_add[0]);
			if (rc < 0) {
				pr_err("Unable to parse MAC DA\n");
				return -EINVAL;
			}
			pr_debug("MAC SA: %02x:%02x:%02x:%02x:%02x:%02x\n",
				 mng_pkt_key->pkt_key->eth_dst.eth_add[0],
				 mng_pkt_key->pkt_key->eth_dst.eth_add[1],
				 mng_pkt_key->pkt_key->eth_dst.eth_add[2],
				 mng_pkt_key->pkt_key->eth_dst.eth_add[3],
				 mng_pkt_key->pkt_key->eth_dst.eth_add[4],
				 mng_pkt_key->pkt_key->eth_dst.eth_add[5]);
			rc = mv_pp2x_parse_mac_address((char *)rule->fields[idx1].mask,
						       &mng_pkt_key->pkt_key->eth_dst.eth_add_mask[0]);
			if (rc < 0) {
				pr_err("Unable to parse MAC DA mask\n");
				return -EINVAL;
			}
			pr_debug("MAC SA MASK: %02x:%02x:%02x:%02x:%02x:%02x\n",
				 mng_pkt_key->pkt_key->eth_dst.eth_add_mask[0],
				 mng_pkt_key->pkt_key->eth_dst.eth_add_mask[1],
				 mng_pkt_key->pkt_key->eth_dst.eth_add_mask[2],
				 mng_pkt_key->pkt_key->eth_dst.eth_add_mask[3],
				 mng_pkt_key->pkt_key->eth_dst.eth_add_mask[4],
				 mng_pkt_key->pkt_key->eth_dst.eth_add_mask[5]);
			break;
		case OUT_VLAN_PRI_FIELD_ID:
			if (rule->fields[idx1].size != (GET_NUM_BYTES(OUT_VLAN_PRI_FIELD_SIZE))) {
				pr_err("%s(%d) field size does not match! %d\n", __func__, __LINE__,
				       rule->fields[idx1].size);
				return -EINVAL;
			}
			mng_pkt_key->pkt_key->out_pbit =
				strtol((char *)(rule->fields[idx1].key), NULL, 0);

			pr_debug("OUT_VLAN_PRI_FIELD_ID = %d\n", mng_pkt_key->pkt_key->out_pbit);
			break;
		case OUT_VLAN_ID_FIELD_ID:
			if (rule->fields[idx1].size != (GET_NUM_BYTES(OUT_VLAN_ID_FIELD_SIZE))) {
				pr_err("%s(%d) field size does not match! %d\n", __func__, __LINE__,
				       rule->fields[idx1].size);
				return -EINVAL;
			}
			mng_pkt_key->pkt_key->out_vid =
				strtol((char *)(rule->fields[idx1].key), NULL, 0);

			pr_debug("OUT_VLAN_ID_FIELD_ID = %d\n", mng_pkt_key->pkt_key->out_vid);
			break;
		case IPV4_DSCP_FIELD_ID:
			if (rule->fields[idx1].size != (GET_NUM_BYTES(IPV4_DSCP_FIELD_SIZE))) {
				pr_err("%s(%d) field size does not match! %d\n", __func__, __LINE__,
				       rule->fields[idx1].size);
				return -EINVAL;
			}
			mng_pkt_key->pkt_key->ipvx_add.dscp =
				strtol((char *)(rule->fields[idx1].key), NULL, 0);
			pr_debug("OUT_VLAN_ID_FIELD_ID = %d\n", mng_pkt_key->pkt_key->ipvx_add.dscp);

			mng_pkt_key->pkt_key->ipvx_add.dscp_mask =
				strtol((char *)(rule->fields[idx1].mask), NULL, 0);
			pr_debug("OUT_VLAN_ID_FIELD_ID mask = %d\n", mng_pkt_key->pkt_key->ipvx_add.dscp_mask);
			break;
		case IPV4_SA_FIELD_ID:
			if (rule->fields[idx1].size != (GET_NUM_BYTES(IPV4_SA_FIELD_SIZE))) {
				pr_err("%s(%d) field size does not match! %d\n", __func__, __LINE__,
				       rule->fields[idx1].size);
				return -EINVAL;
			}
			rc = inet_pton(AF_INET, (char *)rule->fields[idx1].key,
				       &mng_pkt_key->pkt_key->ipvx_add.ip_src.ip_add.ipv4[0]);
			if (rc <= 0) {
				pr_err("Unable to parse IPv4 SA\n");
				return -EINVAL;
			}
			/* convert mask */
			if (strncmp((char *)rule->fields[idx1].mask, "0x", 2) == 0)
				mask_ptr = (char *)((char *)rule->fields[idx1].mask + 2);
			else
				return -EINVAL;
			for (i = 0; i < 4; i++) {
				strncpy(mask_arr, mask_ptr, 2);
				mng_pkt_key->pkt_key->ipvx_add.ip_src.ip_add_mask.ipv4[i] =
						strtoul(mask_arr, &ret_ptr, 16);

				if (mask_arr == ret_ptr)
					return -EINVAL;
				mask_ptr += 2;
			}
			pr_debug("IPv4 SA: %d.%d.%d.%d\n",
				 mng_pkt_key->pkt_key->ipvx_add.ip_src.ip_add.ipv4[0],
				 mng_pkt_key->pkt_key->ipvx_add.ip_src.ip_add.ipv4[1],
				 mng_pkt_key->pkt_key->ipvx_add.ip_src.ip_add.ipv4[2],
				 mng_pkt_key->pkt_key->ipvx_add.ip_src.ip_add.ipv4[3]);
			break;
		case IPV4_DA_FIELD_ID:
			if (rule->fields[idx1].size != (GET_NUM_BYTES(IPV4_DA_FIELD_SIZE))) {
				pr_err("%s(%d) field size does not match! %d\n", __func__, __LINE__,
				       rule->fields[idx1].size);
				return -EINVAL;
			}
			rc = inet_pton(AF_INET, (char *)rule->fields[idx1].key,
				       &mng_pkt_key->pkt_key->ipvx_add.ip_dst.ip_add.ipv4[0]);
			if (rc <= 0) {
				pr_err("Unable to parse IPv4 DA\n");
				return -EINVAL;
			}
			/* convert mask */
			if (strncmp((char *)rule->fields[idx1].mask, "0x", 2) == 0)
				mask_ptr = (char *)((char *)rule->fields[idx1].mask + 2);
			else
				return -EINVAL;
			for (i = 0; i < 4; i++) {
				strncpy(mask_arr, mask_ptr, 2);
				mng_pkt_key->pkt_key->ipvx_add.ip_dst.ip_add_mask.ipv4[i] =
									strtoul(mask_arr, &ret_ptr, 16);
				if (mask_arr == ret_ptr)
					return -EINVAL;
				mask_ptr += 2;
			}

			pr_debug("IPv4 DA: %d.%d.%d.%d\n",
				 mng_pkt_key->pkt_key->ipvx_add.ip_dst.ip_add.ipv4[0],
				 mng_pkt_key->pkt_key->ipvx_add.ip_dst.ip_add.ipv4[1],
				 mng_pkt_key->pkt_key->ipvx_add.ip_dst.ip_add.ipv4[2],
				 mng_pkt_key->pkt_key->ipvx_add.ip_dst.ip_add.ipv4[3]);
			break;
		case IPV4_PROTO_FIELD_ID:
		case IPV6_NH_FIELD_ID:
			if (strtol((char *)(rule->fields[idx1].key), NULL, 0) == IPPROTO_UDP) {
				pr_debug("udp selected\n");
				proto_flag = 1;
				mng_pkt_key->pkt_key->ipvx_add.ip_proto = IPPROTO_UDP;
			} else if (strtol((char *)(rule->fields[idx1].key), NULL, 0) == IPPROTO_TCP) {
				pr_debug("tcp selected\n");
				proto_flag = 1;
				mng_pkt_key->pkt_key->ipvx_add.ip_proto = IPPROTO_TCP;
			}
			break;
		case IPV6_SA_FIELD_ID:
			if (rule->fields[idx1].size != (GET_NUM_BYTES(IPV6_SA_FIELD_SIZE))) {
				pr_err("%s(%d) field size does not match! %d\n", __func__, __LINE__,
				       rule->fields[idx1].size);
				return -EINVAL;
			}

			rc = inet_pton(AF_INET6, (char *)rule->fields[idx1].key,
				       &mng_pkt_key->pkt_key->ipvx_add.ip_src.ip_add.ipv6[0]);
			if (rc <= 0) {
				pr_err("Unable to parse IPv6 SA\n");
				return -EINVAL;
			}
			pr_debug("IPv6 SA: ");
			for (idx2 = 0; idx2 < 16; idx2 += 2) {
				pr_info("%x%x",
					mng_pkt_key->pkt_key->ipvx_add.ip_src.ip_add.ipv6[idx2],
					mng_pkt_key->pkt_key->ipvx_add.ip_src.ip_add.ipv6[idx2 + 1]);
				if (idx2 < 14)
					pr_info(":");
				else
					pr_info("\n");
			}
			break;
		case IPV6_DA_FIELD_ID:
			if (rule->fields[idx1].size != (GET_NUM_BYTES(IPV6_DA_FIELD_SIZE))) {
				pr_err("%s(%d) field size does not match! %d\n", __func__, __LINE__,
				       rule->fields[idx1].size);
				return -EINVAL;
			}

			rc = inet_pton(AF_INET6, (char *)rule->fields[idx1].key,
				       &mng_pkt_key->pkt_key->ipvx_add.ip_dst.ip_add.ipv6[0]);
			if (rc <= 0) {
				pr_err("Unable to parse IPv6 DA\n");
				return -EINVAL;
			}
			pr_debug("IPv6 DA: ");
			for (idx2 = 0; idx2 < 16; idx2 += 2) {
				pr_info("%x%x",
					mng_pkt_key->pkt_key->ipvx_add.ip_dst.ip_add.ipv6[idx2],
					mng_pkt_key->pkt_key->ipvx_add.ip_dst.ip_add.ipv6[idx2 + 1]);
				if (idx2 < 14)
					pr_info(":");
				else
					pr_info("\n");
			}
			break;
		case L4_SRC_FIELD_ID:
			if (rule->fields[idx1].size != (GET_NUM_BYTES(L4_SRC_FIELD_SIZE))) {
				pr_err("%s(%d) field size does not match! %d\n", __func__, __LINE__,
				       rule->fields[idx1].size);
				return -EINVAL;
			}
			mng_pkt_key->pkt_key->l4_src =
				strtol((char *)(rule->fields[idx1].key), NULL, 0);

			pr_debug("L4_SRC_FIELD_ID = %d\n", mng_pkt_key->pkt_key->l4_src);
			break;
		case L4_DST_FIELD_ID:
			if (rule->fields[idx1].size != (GET_NUM_BYTES(L4_DST_FIELD_SIZE))) {
				pr_err("%s(%d) field size does not match! %d\n", __func__, __LINE__,
				       rule->fields[idx1].size);
				return -EINVAL;
			}
			mng_pkt_key->pkt_key->l4_dst =
				strtol((char *)(rule->fields[idx1].key), NULL, 0);

			pr_debug("L4_DST_FIELD_ID = %d\n", mng_pkt_key->pkt_key->l4_dst);
			break;
		default:
			pr_err("%s(%d) protocol_field not supported yet!\n", __func__, __LINE__);
			return -EINVAL;
		}
	}

	if ((field_bm == (MVPP2_MATCH_IP_SRC | MVPP2_MATCH_IP_DST | MVPP2_MATCH_L4_SRC | MVPP2_MATCH_L4_DST |
			  MVPP2_MATCH_IPV4_PKT)) && (proto_flag))
		mng_pkt_key->pkt_key->field_bm = MVPP2_MATCH_IPV4_5T;
	else if ((field_bm == (IPV6_SA_FIELD_ID | IPV6_DA_FIELD_ID | L4_SRC_FIELD_ID | L4_DST_FIELD_ID |
		    MVPP2_MATCH_IPV6_PKT)) && (proto_flag))
		mng_pkt_key->pkt_key->field_bm = MVPP2_MATCH_IPV6_5T;
	else
		mng_pkt_key->pkt_key->field_bm = field_bm;

	mng_pkt_key->pkt_key->field_bm_mask = mng_pkt_key->pkt_key->field_bm;

	pr_debug("field_bm: %x\n", mng_pkt_key->pkt_key->field_bm);

	if (ipv4_flag)
		mng_pkt_key->pkt_key->ipvx_add.ip_ver = 4;
	else if (ipv6_flag)
		mng_pkt_key->pkt_key->ipvx_add.ip_ver = 6;

	if (action->type == PP2_CLS_TBL_ACT_DROP)
		pkt_action->color_act = MVPP2_COLOR_ACTION_TYPE_RED_LOCK;
	else
		pkt_action->color_act = MVPP2_COLOR_ACTION_TYPE_GREEN;
	pkt_action->policer_act = MVPP2_ACTION_TYPE_NO_UPDT;
	pkt_action->flowid_act = MVPP2_ACTION_FLOWID_DISABLE;
	pkt_action->frwd_act = MVPP2_ACTION_TYPE_NO_UPDT;

	if (action->cos->tc >= 0 && action->cos->tc < PP2_PPIO_MAX_NUM_TCS) {
		pkt_action->q_low_act = MVPP2_ACTION_TYPE_UPDT_LOCK;
		pkt_action->q_high_act = MVPP2_ACTION_TYPE_UPDT_LOCK;
		queue = port->tc[action->cos->tc].tc_config.first_rxq;
		pkt_qos->q_high = ((u16)queue) >> MVPP2_CLS2_ACT_QOS_ATTR_QL_BITS;
		pkt_qos->q_low = ((u16)queue) & ((1 << MVPP2_CLS2_ACT_QOS_ATTR_QL_BITS) - 1);
		pr_debug("q_low %d, q_high %d, queue %d, tc %d\n", pkt_qos->q_low,
			 pkt_qos->q_high, queue, action->cos->tc);
	} else {
		pkt_action->q_low_act = MVPP2_ACTION_TYPE_NO_UPDT;
		pkt_action->q_high_act = MVPP2_ACTION_TYPE_NO_UPDT;
	}

	return 0;
}

static int pp2_cls_mng_rule_update_db(struct pp2_cls_tbl_rule *rule, struct pp2_cls_tbl_rule *rule_db,
				      struct pp2_cls_tbl_action *action, struct pp2_cls_tbl_action *action_db)
{
	u8 *key, *mask;
	u32 i;

	rule_db->num_fields = rule->num_fields;

	for (i = 0; i < rule->num_fields; i++) {
		rule_db->fields[i].size = rule->fields[i].size;
		key = kmalloc(CLS_MNG_KEY_SIZE_MAX, GFP_KERNEL);
		if (!key) {
			pr_err("no mem for HEK in DB!\n");
			return -ENOMEM;
		}
		memcpy(key, rule->fields[i].key, CLS_MNG_KEY_SIZE_MAX);
		rule_db->fields[i].key = key;

		mask = kmalloc(CLS_MNG_KEY_SIZE_MAX, GFP_KERNEL);
		if (!mask) {
			kfree(key);
			pr_err("no mem for HEK in DB!\n");
			return -ENOMEM;
		}
		memcpy(mask, rule->fields[i].mask, CLS_MNG_KEY_SIZE_MAX);
		rule_db->fields[i].mask = mask;
	}

	action_db->cos = kmalloc(sizeof(*action_db->cos), GFP_KERNEL);
	if (!action_db->cos)
		return -ENOMEM;

	action_db->type = action->type;
	action_db->cos->tc = action->cos->tc;
	return 0;
}

int pp2_cls_mng_rule_add(struct pp2_cls_tbl *tbl, struct pp2_cls_tbl_rule *rule,
			 struct pp2_cls_tbl_action *action, int lkp_type)
{
	struct pp2_cls_pkt_key_t pkt_key;
	struct pp2_cls_mng_pkt_key_t mng_pkt_key;
	struct mv_pp2x_engine_pkt_action pkt_action;
	struct mv_pp2x_qos_value pkt_qos;
	struct mv_pp2x_src_port rule_port;
	struct pp2_port *port;
	struct pp2_inst *inst;
	u32 rc = 0, logic_idx;
	struct pp2_cls_tbl_params *params = &tbl->params;
	struct pp2_cls_tbl_rule *rule_db;
	struct pp2_cls_tbl_action *action_db;

	/* check if table exists in DB */
	rc = pp2_cls_db_mng_tbl_check(tbl);
	if (rc) {
		pr_err("table not found in db\n");
		return -EIO;
	}

	/* check rule is not duplicated */
	rc = pp2_cls_db_mng_rule_check(tbl, rule);
	if (rc) {
		pr_err("rule is duplicated in table\n");
		return -EFAULT;
	}

	/* init value */
	MVPP2_MEMSET_ZERO(pkt_key);
	MVPP2_MEMSET_ZERO(mng_pkt_key);
	mng_pkt_key.pkt_key = &pkt_key;

	port = GET_PPIO_PORT(params->default_act.cos->ppio);
	inst = port->parent;
	rc = pp2_cls_set_rule_info(&mng_pkt_key, &pkt_action, &pkt_qos, &rule_port,
				   params, rule, action, port);

	if (rc) {
		pr_err("%s(%d) setting pkt_mng params failed\n", __func__, __LINE__);
		return rc;
	}

	if (params->type == PP2_CLS_TBL_MASKABLE) {
		struct mv_pp2x_c2_add_entry c2_entry;

		MVPP2_MEMSET_ZERO(c2_entry);
		c2_entry.mng_pkt_key = &mng_pkt_key;
		c2_entry.mng_pkt_key->pkt_key = &pkt_key;
		c2_entry.lkp_type = lkp_type;
		c2_entry.lkp_type_mask = MVPP2_C2_HEK_LKP_TYPE_MASK >> MVPP2_C2_HEK_LKP_TYPE_OFFS;
		memcpy(&c2_entry.port, &rule_port, sizeof(rule_port));
		memcpy(&c2_entry.action, &pkt_action, sizeof(pkt_action));
		memcpy(&c2_entry.qos_value, &pkt_qos, sizeof(pkt_qos));

		/* add rule */
		rc = pp2_cls_c2_rule_add(inst, &c2_entry, &logic_idx);
		if (rc) {
			pr_err("fail to add C2 rule\n");
			return rc;
		}
		pr_debug("Rule added in C2: logic_idx: %d\n", logic_idx);
	} else if (params->type == PP2_CLS_TBL_EXACT_MATCH) {
		struct pp2_cls_c3_add_entry_t c3_entry;

		MVPP2_MEMSET_ZERO(c3_entry);
		c3_entry.mng_pkt_key = &mng_pkt_key;
		c3_entry.mng_pkt_key->pkt_key = &pkt_key;
		c3_entry.lkp_type = lkp_type;
		memcpy(&c3_entry.port, &rule_port, sizeof(rule_port));
		memcpy(&c3_entry.action, &pkt_action, sizeof(pkt_action));
		memcpy(&c3_entry.qos_value, &pkt_qos, sizeof(pkt_qos));

		/* add rule */
		rc = pp2_cls_c3_rule_add(inst, &c3_entry, &logic_idx);
		if (rc) {
			pr_err("fail to add C3 rule\n");
			return rc;
		}
		pr_debug("Rule added in C3: logic_idx: %d\n", logic_idx);
	} else {
		pr_err("%s(%d) unknown engine type!\n", __func__, __LINE__);
		return -EINVAL;
	}
	/* Update database */
	rc = pp2_cls_db_mng_tbl_rule_add(tbl, &rule_db, logic_idx, &action_db);
	if (rc)
		return -EFAULT;

	rc = pp2_cls_mng_rule_update_db(rule, rule_db, action, action_db);
	if (rc)
		return -EFAULT;

	return 0;
}

int pp2_cls_mng_rule_remove(struct pp2_cls_tbl *tbl, struct pp2_cls_tbl_rule *rule)
{
	struct pp2_port *port;
	u32 rc;
	struct pp2_cls_tbl_params *params = &tbl->params;
	u32 logic_index;
	struct pp2_inst *inst;

	/* check if table exists in DB */
	rc = pp2_cls_db_mng_tbl_check(tbl);
	if (rc) {
		pr_err("table not found in db\n");
		return -EFAULT;
	}

	port = GET_PPIO_PORT(params->default_act.cos->ppio);
	inst = port->parent;

	rc = pp2_cls_db_mng_tbl_rule_remove(tbl, rule, &logic_index);
	if (rc)
		return -EFAULT;

	pr_debug("logic_index %d\n", logic_index);
	if (params->type == PP2_CLS_TBL_MASKABLE) {
		pp2_cls_c2_rule_del(inst, logic_index);
	} else if (params->type == PP2_CLS_TBL_EXACT_MATCH) {
		pp2_cls_c3_rule_del(inst, logic_index);
	} else {
		pr_err("%s(%d) unknown engine type!\n", __func__, __LINE__);
		return -EINVAL;
	}
	return 0;
}

int pp2_cls_mng_rule_modify(struct pp2_cls_tbl *tbl, struct pp2_cls_tbl_rule *rule, struct pp2_cls_tbl_action *action)
{
	int rc;

	/* check if table exists in DB */
	rc = pp2_cls_db_mng_tbl_check(tbl);
	if (rc) {
		pr_err("table not found in db\n");
		return -EFAULT;
	}

	rc = pp2_cls_mng_rule_remove(tbl, rule);
	if (rc) {
		pr_err("cls manager rule modify - remove error\n");
		return rc;
	}

	rc = pp2_cls_mng_rule_add(tbl, rule, action, MVPP2_CLS_LKP_MUSDK_CLS);
	if (rc) {
		pr_err("cls manager rule modify - add error\n");
		return rc;
	}
	return 0;
}
void pp2_cls_mng_init(struct pp2_inst *inst)
{
	if (inst->cls_db)
		return;			/*Already initialized*/

	pp2_cls_db_init(inst);
	pp2_cls_prs_init(inst);
	pp2_cls_init(inst);
	pp2_cls_c2_start(inst);
	pp2_cls_c3_start(inst);
}

void pp2_cls_mng_deinit(struct pp2_inst *inst)
{
	pp2_cls_prs_deinit(inst);
}


