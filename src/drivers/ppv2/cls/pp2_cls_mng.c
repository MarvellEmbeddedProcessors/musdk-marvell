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
#include "pp2_cls_mng.h"

#define MVPP2_CLS_PROTO_SHIFT	MVPP2_CLS_PROTO_SHIFT
#define NOT_SUPPORTED_YET 255

struct pp2_ppio {
	struct pp2_port *port;
};

struct cls_field_convert_t {
	u32 proto;
	u32 field;
	u32 field_to_config;
	u32 match_bm;
};

static struct cls_field_convert_t g_cls_field_convert[] = {
	{1, 0, MAC_SA_FIELD_ID, MVPP2_MATCH_ETH_SRC},	        /* ethernet, source address */
	{1, 1, MAC_DA_FIELD_ID, MVPP2_MATCH_ETH_DST},	        /* ethernet, destination address */
	{1, 2, ETH_TYPE_FIELD_ID, MVPP2_MATCH_ETH_TYPE},	/* ethernet, type */
	{2, 0, OUT_VLAN_PRI_FIELD_ID, MVPP2_MATCH_PBITS_OUTER},	/* vlan, priority (outer vlan)*/
	{2, 1, OUT_VLAN_ID_FIELD_ID, MVPP2_MATCH_VID_OUTER},	/* vlan, id (outer vlan) */
	{2, 2, NOT_SUPPORTED_YET, 0},	                        /* vlan, tci */
	{3, 0, PPPOE_FIELD_ID, MVPP2_MATCH_PPPOE_PROTO},	/* pppoe*/
	{4, 0, IP_VER_FIELD_ID, MVPP2_MATCH_IP_VERSION},	/* ip */
	{5, 0, IPV4_DSCP_FIELD_ID, MVPP2_MATCH_IP_DSCP},	/* ipv4, tos  [AW: check] */
	{5, 1, IPV4_SA_FIELD_ID, MVPP2_MATCH_IP_SRC},	        /* ipv4, souce address */
	{5, 2, IPV4_DA_FIELD_ID, MVPP2_MATCH_IP_DST},	        /* ipv4, destination address */
	{5, 3, IPV4_PROTO_FIELD_ID, MVPP2_MATCH_IP_PROTO},	/* ipv4, proto */
	{6, 0, NOT_SUPPORTED_YET, 0},	                        /* ipv6, tc */
	{6, 1, IPV6_SA_FIELD_ID, MVPP2_MATCH_IP_SRC},	        /* ipv6, souce address */
	{6, 2, IPV6_DA_FIELD_ID, MVPP2_MATCH_IP_DST},	        /* ipv6, destination address */
	{6, 3, IPV6_FLOW_LBL_FIELD_ID, MVPP2_MATCH_IPV6_FLBL},	/* ipv6, flow */
	{6, 4, IPV6_NH_FIELD_ID, MVPP2_MATCH_IP_PROTO},	        /* ipv6, next header */
	{7, 0, L4_SRC_FIELD_ID, MVPP2_MATCH_L4_SRC},	        /* layer4, source port */
	{7, 1, L4_DST_FIELD_ID, MVPP2_MATCH_L4_DST},	        /* layer4, destination port */
	{7, 2, NOT_SUPPORTED_YET, 0},	                        /* layer4, checksum */
	{8, 0, L4_SRC_FIELD_ID, MVPP2_MATCH_L4_SRC},	        /* tcp, source port */
	{8, 1, L4_DST_FIELD_ID, MVPP2_MATCH_L4_DST},	        /* tcp, destination port */
	{8, 2, NOT_SUPPORTED_YET, 0},	                        /* tcp, checksum */
	{9, 0, L4_SRC_FIELD_ID, MVPP2_MATCH_L4_SRC},	        /* udp, source port */
	{9, 1, L4_DST_FIELD_ID, MVPP2_MATCH_L4_DST},	        /* udp, destination port */
	{9, 2, NOT_SUPPORTED_YET, 0},	                        /* udp, checksum */
	{10, 0, NOT_SUPPORTED_YET, 0},	                        /* icmp */
	{11, 0, ARP_IPV4_DA_FIELD_ID, MVPP2_MATCH_ARP_TRGT_IP_ADDR},	/* arp */
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

int pp2_cls_mng_tbl_init(struct pp2_cls_tbl_params *params)
{
	struct pp2_cls_fl_rule_list_t *fl_rls;
	struct pp2_ppio *ppio;
	struct pp2_port *port;
	uintptr_t cpu_slot;
	u32 idx;
	u32 field, match_bm;
	u32 rc = 0;
	u32 i, j, field_index;
	u32 proto_flag = 0;
	u32 ipv4_flag = 0;
	u32 ipv6_flag = 0;
	u16 select_log_id[10];

	/* Para check */
	if (mv_pp2x_ptr_validate(params))
		return -EINVAL;

	if (mv_pp2x_range_validate(params->key.num_fields, 1, PP2_CLS_TBL_MAX_NUM_FIELDS))
		return -EINVAL;
	pp2_dbg("key.num_fields = %d\n", params->key.num_fields);

	fl_rls = malloc(sizeof(struct pp2_cls_fl_rule_list_t));
	if (fl_rls == NULL) {
		pp2_err("%s(%d) Error allocating memory!\n", __func__, __LINE__);
		return -ENOMEM;
	}
	/* get cpu_slot */
	ppio = params->default_act.cos->ppio;
	port = ppio->port;
	cpu_slot = port->cpu_slot;

	fl_rls->fl_len = 1;
	field_index = 0;
	/* parse the protocol and protocol fields */
	for (idx = 0; idx < params->key.num_fields; idx++) {
		rc = lookup_field_id(params->key.proto_field[idx].proto,
			params->key.proto_field[idx].field.eth, &field, &match_bm);
		if (rc) {
			pp2_err("%s(%d) lookup id error!\n", __func__, __LINE__);
			goto end;
		}
		if (params->key.proto_field[idx].proto == MV_NET_PROTO_IP4) {
			ipv4_flag = 1;
			if (params->key.proto_field[idx].field.ipv4 == MV_NET_IP4_F_PROTO) {
				proto_flag = 1;
				if (params->key.num_fields == PP2_CLS_TBL_MAX_NUM_FIELDS)
					continue;
			}
		} else if (params->key.proto_field[idx].proto == MV_NET_PROTO_IP6) {
			ipv6_flag = 1;
			if (params->key.proto_field[idx].field.ipv6 == MV_NET_IP6_F_NEXT_HDR) {
				proto_flag = 1;
				if (params->key.num_fields == PP2_CLS_TBL_MAX_NUM_FIELDS)
					continue;
			}
		}
		fl_rls->fl[0].field_id[field_index++] = field;
	}

	/* engine selection */
	if (params->type == PP2_CLS_TBL_MASKABLE) {
		fl_rls->fl[0].engine = MVPP2_CLS_ENGINE_C2;
		pp2_err("maskable engine not supported\n");
		return -EINVAL;
	} else {
		if (proto_flag)
			fl_rls->fl[0].engine = MVPP2_CLS_ENGINE_C3B;
		else
			fl_rls->fl[0].engine = MVPP2_CLS_ENGINE_C3A;
	}

	/* port type - TODO fixed to PHY for now */
	fl_rls->fl[0].port_type = MVPP2_SRC_PORT_TYPE_PHY;

	/* port ID - TODO set it fixed to 1. this value is used only if
	 * PortIdSelect bit in CLS_FLOW_TBL1 register is set to 0
	 */
	fl_rls->fl[0].port_bm = 1;

	/* lookup_type */
	fl_rls->fl[0].lu_type = MVPP2_CLS_MUSDK_LKP_DEFAULT;
	fl_rls->fl[0].enabled = true;
	/* priotity - TODO - not implemented yet in API */
	fl_rls->fl[0].prio = MVPP2_CLS_DEF_PRIO;
	fl_rls->fl[0].seq_ctrl = MVPP2_CLS_DEF_SEQ_CTRL;
	fl_rls->fl[0].field_id_cnt = params->key.num_fields - (fl_rls->fl[0].engine == MVPP2_CLS_ENGINE_C3B);

	pp2_dbg("ipv4_flag = %d\n", ipv4_flag);
	pp2_dbg("ipv6_flag = %d\n", ipv6_flag);
	pp2_dbg("proto_flag = %d\n", proto_flag);

	/* find for which logical flow id to add this rule */
	j = 0;
	/* ipv4 */
	if (ipv4_flag && proto_flag) {
		select_log_id[j++] = MVPP2_PRS_FL_IP4_TCP_NF_UNTAG;
		select_log_id[j++] = MVPP2_PRS_FL_IP4_TCP_NF_TAG;
		select_log_id[j++] = MVPP2_PRS_FL_IP4_TCP_FRAG_UNTAG;
		select_log_id[j++] = MVPP2_PRS_FL_IP4_TCP_FRAG_TAG;
		select_log_id[j++] = MVPP2_PRS_FL_IP4_UDP_NF_TAG;
		select_log_id[j++] = MVPP2_PRS_FL_IP4_UDP_NF_UNTAG;
		select_log_id[j++] = MVPP2_PRS_FL_IP4_UDP_FRAG_UNTAG;
		select_log_id[j++] = MVPP2_PRS_FL_IP4_UDP_FRAG_TAG;
	/* ipv6 */
	} else if (ipv6_flag && proto_flag) {
		select_log_id[j++] = MVPP2_PRS_FL_IP6_TCP_NF_UNTAG;
		select_log_id[j++] = MVPP2_PRS_FL_IP6_TCP_NF_TAG;
		select_log_id[j++] = MVPP2_PRS_FL_IP6_TCP_FRAG_UNTAG;
		select_log_id[j++] = MVPP2_PRS_FL_IP6_TCP_FRAG_TAG;
		select_log_id[j++] = MVPP2_PRS_FL_IP6_UDP_NF_UNTAG;
		select_log_id[j++] = MVPP2_PRS_FL_IP6_UDP_NF_TAG;
		select_log_id[j++] = MVPP2_PRS_FL_IP6_UDP_FRAG_UNTAG;
		select_log_id[j++] = MVPP2_PRS_FL_IP6_UDP_FRAG_TAG;
	/* ipv4 */
	} else if (ipv4_flag) {
		select_log_id[j++] = MVPP2_PRS_FL_IP4_UNTAG;
		select_log_id[j++] = MVPP2_PRS_FL_IP4_TAG;
	/* ipv6 */
	} else if (ipv6_flag) {
		select_log_id[j++] = MVPP2_PRS_FL_IP6_UNTAG;
		select_log_id[j++] = MVPP2_PRS_FL_IP6_TAG;
	} else {
		select_log_id[j++] = MVPP2_PRS_FL_NON_IP_UNTAG;
		select_log_id[j++] = MVPP2_PRS_FL_NON_IP_TAG;
	}
	select_log_id[j] = 0;

	/* add current rule for all selected logical flow id */
	for (i = 0; select_log_id[i] != 0; i++) {
		pp2_dbg("select_log_id = %d\n", select_log_id[i]);
		fl_rls->fl[0].fl_log_id = select_log_id[i];

		/* Add flow rule */
		pp2_cls_lkp_dcod_set_and_disable(cpu_slot, select_log_id[i]);
		rc = pp2_cls_fl_rule_add(cpu_slot, fl_rls);
		if (rc) {
			pp2_err("failed to add cls flow rule\n");
			goto end;
		}
		pp2_cls_lkp_dcod_enable(cpu_slot, select_log_id[i]);
	}

end:
	free(fl_rls);

	return rc;
}

int pp2_cls_mng_rule_add(struct pp2_cls_tbl_params *params, struct pp2_cls_tbl_rule *rule,
			 struct pp2_cls_tbl_action *action)
{

	u32 idx1, idx2;
	struct pp2_cls_pkt_key_t pkt_key;
	struct pp2_cls_mng_pkt_key_t mng_pkt_key;
	struct pp2_cls_c3_add_entry_t c3_entry;
	u32 logic_idx;
	u32 rc = 0;
	u32 match_bm = 0, bm = 0;
	u32 proto_flag = 0;
	u32 ipv4_flag = 0;
	u32 ipv6_flag = 0;
	u32 field;
	struct pp2_ppio *ppio;
	struct pp2_port *port;

	/* init value */
	MVPP2_MEMSET_ZERO(pkt_key);
	MVPP2_MEMSET_ZERO(mng_pkt_key);
	MVPP2_MEMSET_ZERO(c3_entry);
	c3_entry.mng_pkt_key = &mng_pkt_key;
	c3_entry.mng_pkt_key->pkt_key = &pkt_key;

	ppio = params->default_act.cos->ppio;
	port = ppio->port;

	/* set value */
	c3_entry.port.port_type = MVPP2_SRC_PORT_TYPE_PHY;
	c3_entry.port.port_value = (1 << port->id);
	c3_entry.port.port_mask = 0xff;

	c3_entry.lkp_type = MVPP2_CLS_MUSDK_LKP_DEFAULT;

	/* parse the protocol and protocol fields */
	for (idx1 = 0; idx1 < params->key.num_fields; idx1++) {
		rc = lookup_field_id(params->key.proto_field[idx1].proto, params->key.proto_field[idx1].field.eth,
				     &field, &bm);
		if (rc) {
			pp2_err("%s(%d) lookup id error!\n", __func__, __LINE__);
			return -EINVAL;
		}

		if (field == NOT_SUPPORTED_YET) {
			pp2_err("%s(%d) protocol_field not supported yet - skipping!\n", __func__, __LINE__);
			continue;
		}
		if (params->key.proto_field[idx1].proto == MV_NET_PROTO_IP4) {
			ipv4_flag = 1;
			match_bm |= MVPP2_MATCH_IPV4_PKT;
		}
		if (params->key.proto_field[idx1].proto == MV_NET_PROTO_IP6) {
			ipv6_flag = 1;
			match_bm |= MVPP2_MATCH_IPV6_PKT;
		}

		match_bm |= bm;

		switch (field) {
		case IPV4_SA_FIELD_ID:
			if (rule->fields[idx1].size != 4) {
				pp2_err("%s(%d) field size does not match! %d\n", __func__, __LINE__,
					rule->fields[idx1].size);
				return -EINVAL;
			}
			rc = inet_pton(AF_INET, (char *)rule->fields[idx1].key,
				&c3_entry.mng_pkt_key->pkt_key->ipvx_add.ip_src.ip_add.ipv4[0]);
			if (rc <= 0) {
				pr_err("Unable to parse IPv4 SA\n");
				return rc;
			}
			pp2_dbg("IPv4 SA: %d.%d.%d.%d\n",
				c3_entry.mng_pkt_key->pkt_key->ipvx_add.ip_src.ip_add.ipv4[0],
				c3_entry.mng_pkt_key->pkt_key->ipvx_add.ip_src.ip_add.ipv4[1],
				c3_entry.mng_pkt_key->pkt_key->ipvx_add.ip_src.ip_add.ipv4[2],
				c3_entry.mng_pkt_key->pkt_key->ipvx_add.ip_src.ip_add.ipv4[3]);
			break;
		case IPV4_DA_FIELD_ID:
			if (rule->fields[idx1].size != 4) {
				pp2_err("%s(%d) field size does not match! %d\n", __func__, __LINE__,
					rule->fields[idx1].size);
				return -EINVAL;
			}
			rc = inet_pton(AF_INET, (char *)rule->fields[idx1].key,
				&c3_entry.mng_pkt_key->pkt_key->ipvx_add.ip_dst.ip_add.ipv4[0]);
			if (rc <= 0) {
				pr_err("Unable to parse IPv4 DA\n");
				return rc;
			}
			pp2_dbg("IPv4 DA: %d.%d.%d.%d\n",
				c3_entry.mng_pkt_key->pkt_key->ipvx_add.ip_dst.ip_add.ipv4[0],
				c3_entry.mng_pkt_key->pkt_key->ipvx_add.ip_dst.ip_add.ipv4[1],
				c3_entry.mng_pkt_key->pkt_key->ipvx_add.ip_dst.ip_add.ipv4[2],
				c3_entry.mng_pkt_key->pkt_key->ipvx_add.ip_dst.ip_add.ipv4[3]);
			break;
		case IPV4_PROTO_FIELD_ID:
		case IPV6_NH_FIELD_ID:
			if (strtol((char *)(rule->fields[idx1].key), NULL, 0) == IPPROTO_UDP) {
				pp2_err("udp selected\n");
				proto_flag = 1;
				c3_entry.mng_pkt_key->pkt_key->ipvx_add.ip_proto = IPPROTO_UDP;
			} else if (strtol((char *)(rule->fields[idx1].key), NULL, 0) == IPPROTO_TCP) {
				pp2_err("tcp selected\n");
				proto_flag = 1;
				c3_entry.mng_pkt_key->pkt_key->ipvx_add.ip_proto = IPPROTO_TCP;
			}
			break;
		case IPV6_SA_FIELD_ID:
			if (rule->fields[idx1].size != 16) {
				pp2_err("%s(%d) field size does not match! %d\n", __func__, __LINE__,
					rule->fields[idx1].size);
				return -EINVAL;
			}

			rc = inet_pton(AF_INET6, (char *)rule->fields[idx1].key,
				&c3_entry.mng_pkt_key->pkt_key->ipvx_add.ip_src.ip_add.ipv6[0]);
			if (rc <= 0) {
				pr_err("Unable to parse IPv6 SA\n");
				return rc;
			}
			pp2_dbg("IPv6 SA: ");
			for (idx2 = 0; idx2 < 16; idx2 += 2) {
				pr_info("%x%x",
					c3_entry.mng_pkt_key->pkt_key->ipvx_add.ip_src.ip_add.ipv6[idx2],
					c3_entry.mng_pkt_key->pkt_key->ipvx_add.ip_src.ip_add.ipv6[idx2 + 1]);
				if (idx2 < 14)
					pr_info(":");
				else
					pr_info("\n");
			}
			break;
		case IPV6_DA_FIELD_ID:
			if (rule->fields[idx1].size != 16) {
				pp2_err("%s(%d) field size does not match! %d\n", __func__, __LINE__,
					rule->fields[idx1].size);
				return -EINVAL;
			}

			rc = inet_pton(AF_INET6, (char *)rule->fields[idx1].key,
				&c3_entry.mng_pkt_key->pkt_key->ipvx_add.ip_dst.ip_add.ipv6[0]);
			if (rc <= 0) {
				pr_err("Unable to parse IPv6 DA\n");
				return rc;
			}
			pp2_dbg("IPv6 DA: ");
			for (idx2 = 0; idx2 < 16; idx2 += 2) {
				pr_info("%x%x",
					c3_entry.mng_pkt_key->pkt_key->ipvx_add.ip_dst.ip_add.ipv6[idx2],
					c3_entry.mng_pkt_key->pkt_key->ipvx_add.ip_dst.ip_add.ipv6[idx2 + 1]);
				if (idx2 < 14)
					pr_info(":");
				else
					pr_info("\n");
			}
			break;
		case L4_SRC_FIELD_ID:
			if (rule->fields[idx1].size != 2) {
				pp2_err("%s(%d) field size does not match! %d\n", __func__, __LINE__,
					rule->fields[idx1].size);
				return -EINVAL;
			}
			c3_entry.mng_pkt_key->pkt_key->l4_src =
				strtol((char *)(rule->fields[idx1].key), NULL, 16);
			break;
		case L4_DST_FIELD_ID:
			if (rule->fields[idx1].size != 2) {
				pp2_err("%s(%d) field size does not match! %d\n", __func__, __LINE__,
					rule->fields[idx1].size);
				return -EINVAL;
			}
			c3_entry.mng_pkt_key->pkt_key->l4_dst =
				strtol((char *)(rule->fields[idx1].key), NULL, 16);
			break;
		}
	}

	if ((match_bm == (MVPP2_MATCH_IP_SRC | MVPP2_MATCH_IP_DST | MVPP2_MATCH_L4_SRC | MVPP2_MATCH_L4_DST |
			  MVPP2_MATCH_IPV4_PKT)) && (proto_flag))
		c3_entry.mng_pkt_key->pkt_key->field_match_bm = MVPP2_MATCH_IPV4_5T;
	else if ((match_bm == (IPV6_SA_FIELD_ID | IPV6_DA_FIELD_ID | L4_SRC_FIELD_ID | L4_DST_FIELD_ID |
		    MVPP2_MATCH_IPV6_PKT)) && (proto_flag))
		c3_entry.mng_pkt_key->pkt_key->field_match_bm = MVPP2_MATCH_IPV6_5T;
	else
		c3_entry.mng_pkt_key->pkt_key->field_match_bm = match_bm;

	pp2_dbg("match_bm: %x\n", c3_entry.mng_pkt_key->pkt_key->field_match_bm);

	if (ipv4_flag)
		c3_entry.mng_pkt_key->pkt_key->ipvx_add.ip_ver = 4;
	else if (ipv6_flag)
		c3_entry.mng_pkt_key->pkt_key->ipvx_add.ip_ver = 6;

	c3_entry.qos_info.policer_id = 0;

	if (action->type == PP2_CLS_TBL_ACT_DROP)
		c3_entry.action.color_act = MVPP2_COLOR_ACTION_TYPE_RED_LOCK;
	else
		c3_entry.action.color_act = MVPP2_COLOR_ACTION_TYPE_NO_UPDT;
	c3_entry.action.q_low_act = MVPP2_ACTION_TYPE_NO_UPDT;
	c3_entry.action.q_high_act = MVPP2_ACTION_TYPE_NO_UPDT;
	c3_entry.action.policer_act = MVPP2_ACTION_TYPE_NO_UPDT;
	c3_entry.action.flowid_act = MVPP2_ACTION_FLOWID_DISABLE;
	c3_entry.action.frwd_act = MVPP2_ACTION_TYPE_NO_UPDT;

	c3_entry.qos_value.q_high = 6;
	c3_entry.qos_value.q_low = 2;

	pp2_dbg("cpu_slot2: %p\n", (void *)port->cpu_slot);
	/* add rule */
	rc = pp2_cls_c3_rule_add(port->cpu_slot, &c3_entry, &logic_idx);
	if (rc) {
		pp2_err("fail to add C3 rule\n");
		return rc;
	}
	pp2_dbg("Rule added in C3: logic_idx: %d\n", logic_idx);

	return 0;
}
