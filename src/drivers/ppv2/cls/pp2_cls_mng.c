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
#include "std_internal.h"
#include "drivers/ppv2/pp2.h"
#include "drivers/ppv2/pp2_hw_type.h"
#include "pp2_hw_cls.h"
#include "pp2_cls_types.h"
#include "pp2_cls_internal_types.h"
#include "pp2_cls_common.h"
#include "pp2_flow_rules.h"
#include "pp2_cls_db.h"
#include "drivers/mv_pp2_cls.h"
#include "pp2_hw_cls.h"
#include "pp2_cls_mng.h"
#include "pp2_prs.h"
#include "pp2_rss.h"

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
	/* ipv4, tos  [AW: check] */
	{MV_NET_PROTO_IP4, MV_NET_IP4_F_DSCP, IPV4_DSCP_FIELD_ID, MVPP2_MATCH_IP_DSCP},
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
	/* tcp, source port */
	{MV_NET_PROTO_TCP, MV_NET_TCP_F_SP, L4_SRC_FIELD_ID, MVPP2_MATCH_L4_SRC},
	/* tcp, destination port */
	{MV_NET_PROTO_TCP, MV_NET_TCP_F_DP, L4_DST_FIELD_ID, MVPP2_MATCH_L4_DST},
	/* udp, source port */
	{MV_NET_PROTO_UDP, MV_NET_UDP_F_SP, L4_SRC_FIELD_ID, MVPP2_MATCH_L4_SRC},
	/* udp, destination port */
	{MV_NET_PROTO_UDP, MV_NET_UDP_F_DP, L4_DST_FIELD_ID, MVPP2_MATCH_L4_DST},
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

int pp2_cls_mng_lkp_type_to_prio(int lkp_type)
{
	int prio;

	switch (lkp_type) {
	case MVPP2_CLS_LKP_HASH:
		prio = MVPP2_CLS_KERNEL_HASH_PRIO;
		break;
	case MVPP2_CLS_LKP_VLAN_PRI:
		prio = MVPP2_CLS_KERNEL_VLAN_PRIO;
		break;
	case MVPP2_CLS_LKP_DSCP_PRI:
		prio = MVPP2_CLS_KERNEL_DSCP_PRIO;
		break;
	case MVPP2_CLS_LKP_DEFAULT:
		prio = MVPP2_CLS_KERNEL_DEF_PRIO;
		break;
	case MVPP2_CLS_LKP_MUSDK_LOG_HASH:
		prio = MVPP2_CLS_MUSDK_HASH_PRIO;
		break;
	case MVPP2_CLS_LKP_MUSDK_VLAN_PRI:
		prio = MVPP2_CLS_MUSDK_VLAN_PRIO;
		break;
	case MVPP2_CLS_LKP_MUSDK_DSCP_PRI:
		prio = MVPP2_CLS_MUSDK_DSCP_PRIO;
		break;
	case MVPP2_CLS_LKP_MUSDK_LOG_PORT_DEF:
		prio = MVPP2_CLS_MUSDK_DEF_PRIO;
		break;
	case MVPP2_CLS_LKP_MUSDK_CLS:
		prio = MVPP2_CLS_MUSDK_CLS_PRIO;
		break;
	default:
		pr_err("unknown lkp type = %d\n", lkp_type);
		return -EINVAL;
	}

	return prio;
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

static int pp2_cls_mng_get_lkpid_for_flow_type(u16 *select_logical_id,
					       u32 ipv4_flag,
					       u32 ipv6_flag,
					       u32 tcp_flag,
					       u32 udp_flag,
					       u32 l4_flag)
{
	int num_lkpid = 0;

	/* find relevant lkpid for this flow */
	if (!ipv4_flag && !ipv6_flag && !l4_flag && !tcp_flag && !udp_flag) {
		num_lkpid += pp2_cls_add_non_ip_logical_id(&select_logical_id[num_lkpid]);
		num_lkpid += pp2_cls_add_ip4_logical_id(&select_logical_id[num_lkpid]);
		num_lkpid += pp2_cls_add_ip4_tcp_logical_id(&select_logical_id[num_lkpid]);
		num_lkpid += pp2_cls_add_ip4_udp_logical_id(&select_logical_id[num_lkpid]);
		num_lkpid += pp2_cls_add_ip6_logical_id(&select_logical_id[num_lkpid]);
		num_lkpid += pp2_cls_add_ip6_tcp_logical_id(&select_logical_id[num_lkpid]);
		num_lkpid += pp2_cls_add_ip6_udp_logical_id(&select_logical_id[num_lkpid]);
	} else if (ipv4_flag) {
		if (!tcp_flag && !udp_flag && !l4_flag) {
			num_lkpid += pp2_cls_add_ip4_logical_id(&select_logical_id[num_lkpid]);
			num_lkpid += pp2_cls_add_ip4_tcp_logical_id(&select_logical_id[num_lkpid]);
			num_lkpid += pp2_cls_add_ip4_udp_logical_id(&select_logical_id[num_lkpid]);
		} else if (l4_flag && !tcp_flag && !udp_flag) {
			num_lkpid += pp2_cls_add_ip4_tcp_logical_id(&select_logical_id[num_lkpid]);
			num_lkpid += pp2_cls_add_ip4_udp_logical_id(&select_logical_id[num_lkpid]);
		} else if (tcp_flag) {
			num_lkpid += pp2_cls_add_ip4_tcp_logical_id(&select_logical_id[num_lkpid]);
		} else if (udp_flag) {
			num_lkpid += pp2_cls_add_ip4_udp_logical_id(&select_logical_id[num_lkpid]);
		} else {
			pr_err("%s(%d), failed to calculate lkpid\n", __func__, __LINE__);
			return -EINVAL;
		}
	} else if (ipv6_flag) {
		if (!tcp_flag && !udp_flag && !l4_flag) {
			num_lkpid += pp2_cls_add_ip6_logical_id(&select_logical_id[num_lkpid]);
			num_lkpid += pp2_cls_add_ip6_tcp_logical_id(&select_logical_id[num_lkpid]);
			num_lkpid += pp2_cls_add_ip6_udp_logical_id(&select_logical_id[num_lkpid]);
		} else if (l4_flag && !tcp_flag && !udp_flag) {
			num_lkpid += pp2_cls_add_ip6_tcp_logical_id(&select_logical_id[num_lkpid]);
			num_lkpid += pp2_cls_add_ip6_udp_logical_id(&select_logical_id[num_lkpid]);
		} else if (tcp_flag) {
			num_lkpid += pp2_cls_add_ip6_tcp_logical_id(&select_logical_id[num_lkpid]);
		} else if (udp_flag) {
			num_lkpid += pp2_cls_add_ip6_udp_logical_id(&select_logical_id[num_lkpid]);
		} else {
			pr_err("%s(%d), failed to calculate lkpid\n", __func__, __LINE__);
			return -EINVAL;
		}
	} else if (l4_flag) {
		num_lkpid += pp2_cls_add_ip4_tcp_logical_id(&select_logical_id[num_lkpid]);
		num_lkpid += pp2_cls_add_ip4_udp_logical_id(&select_logical_id[num_lkpid]);
		num_lkpid += pp2_cls_add_ip6_tcp_logical_id(&select_logical_id[num_lkpid]);
		num_lkpid += pp2_cls_add_ip6_udp_logical_id(&select_logical_id[num_lkpid]);
	} else if (tcp_flag) {
		num_lkpid += pp2_cls_add_ip4_tcp_logical_id(&select_logical_id[num_lkpid]);
		num_lkpid += pp2_cls_add_ip6_tcp_logical_id(&select_logical_id[num_lkpid]);
	} else if (udp_flag) {
		num_lkpid += pp2_cls_add_ip4_udp_logical_id(&select_logical_id[num_lkpid]);
		num_lkpid += pp2_cls_add_ip6_udp_logical_id(&select_logical_id[num_lkpid]);
	} else {
		pr_err("%s(%d), failed to calculate lkpid\n", __func__, __LINE__);
		return -EINVAL;
	}

	return num_lkpid;
}

static int pp2_cls_mng_get_lkpid_for_lkp_type(int lkp_type, u16 *select_logical_id)
{
	int lkpid, lkpid_attr;
	int num_lkpid = 0;

	for (lkpid = MVPP2_PRS_FL_START; lkpid < MVPP2_PRS_FL_LAST; lkpid++) {
		/* Get lookup id attribute */
		lkpid_attr = mv_pp2x_prs_flow_id_attr_get(lkpid);
		/* For untagged IP packets, only need default
		 * rule and dscp rule
		 */
		if ((lkpid_attr & (MVPP2_PRS_FL_ATTR_IP4_BIT | MVPP2_PRS_FL_ATTR_IP6_BIT)) &&
		    (!(lkpid_attr & MVPP2_PRS_FL_ATTR_VLAN_BIT))) {
			if (lkp_type == MVPP2_CLS_LKP_DEFAULT ||
			    lkp_type == MVPP2_CLS_LKP_DSCP_PRI ||
			    lkp_type == MVPP2_CLS_LKP_MUSDK_LOG_PORT_DEF ||
			    lkp_type == MVPP2_CLS_LKP_MUSDK_DSCP_PRI) {
				select_logical_id[num_lkpid++] = lkpid;
				continue;
			}
		}

		/* For tagged IP packets, only need vlan rule and dscp rule */
		if ((lkpid_attr & (MVPP2_PRS_FL_ATTR_IP4_BIT | MVPP2_PRS_FL_ATTR_IP6_BIT)) &&
		    (lkpid_attr & MVPP2_PRS_FL_ATTR_VLAN_BIT)) {
			if (lkp_type == MVPP2_CLS_LKP_VLAN_PRI ||
			    lkp_type == MVPP2_CLS_LKP_DSCP_PRI ||
			    lkp_type == MVPP2_CLS_LKP_MUSDK_VLAN_PRI ||
			    lkp_type == MVPP2_CLS_LKP_MUSDK_DSCP_PRI) {
				select_logical_id[num_lkpid++] = lkpid;
				continue;
			}
		}

		/* For non-IP packets, only need default rule if untagged,
		 * vlan rule also needed if tagged
		 */
		if (!(lkpid_attr & (MVPP2_PRS_FL_ATTR_IP4_BIT | MVPP2_PRS_FL_ATTR_IP6_BIT))) {
			/* Default rule */
			if (lkp_type == MVPP2_CLS_LKP_DEFAULT ||
			    lkp_type == MVPP2_CLS_LKP_MUSDK_LOG_PORT_DEF) {
				select_logical_id[num_lkpid++] = lkpid;
				continue;
			}
			/* VLAN rule if tagged */
			if (lkpid_attr & MVPP2_PRS_FL_ATTR_VLAN_BIT) {
				if (lkp_type == MVPP2_CLS_LKP_VLAN_PRI ||
				    lkp_type == MVPP2_CLS_LKP_MUSDK_VLAN_PRI) {
					select_logical_id[num_lkpid++] = lkpid;
					continue;
				}
			}
		}
	}

	return num_lkpid;
}

static int pp2_cls_mng_get_lkpid_for_rss(int engine, u16 *select_logical_id, int ipv4_flag, int ipv6_flag)
{
	int lkpid, lkpid_attr;
	int num_lkpid = 0;

	for (lkpid = MVPP2_PRS_FL_START; lkpid < MVPP2_PRS_FL_LAST; lkpid++) {
		/* Get lookup id attribute */
		lkpid_attr = mv_pp2x_prs_flow_id_attr_get(lkpid);

		if (!((lkpid_attr & MVPP2_PRS_FL_ATTR_IP4_BIT && ipv4_flag) ||
		      (lkpid_attr & MVPP2_PRS_FL_ATTR_IP6_BIT && ipv6_flag)))
			continue;

		/* For frag packets or non-TCP & UDP, rss must be based on 2T */
		if ((engine == MVPP2_CLS_ENGINE_C3HA) &&
		    ((lkpid_attr & MVPP2_PRS_FL_ATTR_FRAG_BIT) ||
		    !(lkpid_attr & (MVPP2_PRS_FL_ATTR_TCP_BIT | MVPP2_PRS_FL_ATTR_UDP_BIT)))) {
			select_logical_id[num_lkpid++] = lkpid;
			continue;
		}

		if (!(lkpid_attr & MVPP2_PRS_FL_ATTR_FRAG_BIT)) {
			if (lkpid_attr & (MVPP2_PRS_FL_ATTR_TCP_BIT | MVPP2_PRS_FL_ATTR_UDP_BIT)) {
				select_logical_id[num_lkpid++] = lkpid;
				continue;
			} else if ((engine == MVPP2_CLS_ENGINE_C3HB) &&
				  (lkpid_attr & MVPP2_PRS_FL_ATTR_TCP_BIT)) {
				select_logical_id[num_lkpid++] = lkpid;
				continue;
			}
		}
	}

	return num_lkpid;
}

static int pp2_cls_single_flow_enable(struct pp2_port *port, u16 lkp_id, u16 lkp_type, int set)
{
	struct pp2_cls_fl_rule_list_t fl_rls;
	struct pp2_inst *inst = port->parent;
	int rc;

	if (set < 0 || set > 1) {
		pr_err("[%s] Invalid boolean value in 'set': %d.\n", __func__, set);
		return -EINVAL;
	}

	fl_rls.fl_len = 1;
	fl_rls.fl->port_type = MVPP2_SRC_PORT_TYPE_PHY;
	fl_rls.fl->port_bm = (1 << port->id);
	fl_rls.fl->lu_type = lkp_type;
	fl_rls.fl->enabled = set;

	fl_rls.fl->prio = pp2_cls_mng_lkp_type_to_prio(lkp_type);
	fl_rls.fl->engine = MVPP2_CLS_ENGINE_C2;
	fl_rls.fl->udf7 = (port->type == PP2_PPIO_T_LOG) ? MVPP2_CLS_MUSDK_LOG_UDF7 : MVPP2_CLS_MUSDK_NIC_UDF7;
	fl_rls.fl->seq_ctrl = MVPP2_CLS_DEF_SEQ_CTRL;
	fl_rls.fl->field_id_cnt = 0;
	fl_rls.fl->field_id[0] = 0;
	fl_rls.fl->field_id[1] = 0;
	fl_rls.fl->field_id[2] = 0;
	fl_rls.fl->field_id[3] = 0;
	fl_rls.fl->fl_log_id = lkp_id;

	pr_debug("lkp_id = %d, set = %d\n", lkp_id, set);

	rc = pp2_cls_fl_rule_enable(inst, &fl_rls);

	if (rc)
		pr_err("[%s] failed to %s cls flow id %u of type %u.\n",
		       __func__, set ? "enable" : "disable", lkp_id, lkp_type);

	return rc;
}

static int pp2_cls_dscp_flows_set(struct pp2_port *port, enum pp2_cls_qos_tbl_type qos_type)
{
	int lkp_type = (port->type == PP2_PPIO_T_LOG) ? MVPP2_CLS_LKP_MUSDK_DSCP_PRI : MVPP2_CLS_LKP_DSCP_PRI;
	int lkpid, lkpid_attr;
	int dscp_enable, dscp_pcp_enable;
	int enable_flow;
	int rc;

	if (qos_type == PP2_CLS_QOS_TBL_IP_PRI || qos_type == PP2_CLS_QOS_TBL_IP_VLAN_PRI) {
		dscp_enable = 1;
		dscp_pcp_enable = 1;
	} else if (qos_type == PP2_CLS_QOS_TBL_VLAN_IP_PRI) {
		dscp_enable = 1;
		dscp_pcp_enable = 0; /* Allow PCP to take priority over DSCP, where it exists */
	} else {
		dscp_enable = 0;
		dscp_pcp_enable = 0;
	}
	for (lkpid = MVPP2_PRS_FL_START; lkpid < MVPP2_PRS_FL_LAST; lkpid++) {
		/* Get lookup id attribute */
		lkpid_attr = mv_pp2x_prs_flow_id_attr_get(lkpid);

		if (!(lkpid_attr & (MVPP2_PRS_FL_ATTR_IP4_BIT | MVPP2_PRS_FL_ATTR_IP6_BIT)))
			continue; /* There is no dscp flow */

		if (lkpid_attr & (MVPP2_PRS_FL_ATTR_VLAN_BIT))
			enable_flow = dscp_pcp_enable;
		else
			enable_flow = dscp_enable;
		rc = pp2_cls_single_flow_enable(port, lkpid, lkp_type, enable_flow);

		if (rc)
			return rc;
	}

	return 0;
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
	struct pp2_cls_tbl *tbl_hash;
	struct pp2_port *port = GET_PPIO_PORT(ppio);

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
	tbl_params.default_act.plcr = NULL;

	pp2_cls_mng_tbl_init(&tbl_params, &tbl, MVPP2_CLS_LKP_MUSDK_VLAN_PRI);
	pp2_cls_mng_tbl_init(&tbl_params, &tbl, MVPP2_CLS_LKP_MUSDK_DSCP_PRI);
	pp2_cls_mng_tbl_init(&tbl_params, &tbl, MVPP2_CLS_LKP_MUSDK_LOG_PORT_DEF);

	/* add 2 tuple hash rule fo ipv4 */
	tbl_params.type = PP2_CLS_TBL_EXACT_MATCH;
	tbl_params.max_num_rules = 1;
	tbl_params.key.num_fields = 2;
	tbl_params.key.key_size = 8;
	tbl_params.key.proto_field[0].proto = MV_NET_PROTO_IP4;
	tbl_params.key.proto_field[0].field.eth = MV_NET_IP4_F_SA;
	tbl_params.key.proto_field[1].proto = MV_NET_PROTO_IP4;
	tbl_params.key.proto_field[1].field.eth = MV_NET_IP4_F_DA;
	pp2_cls_mng_tbl_init(&tbl_params, &tbl_hash, MVPP2_CLS_LKP_MUSDK_LOG_HASH);

	/* add 2 tuple hash rule fo ipv6 */
	tbl_params.type = PP2_CLS_TBL_EXACT_MATCH;
	tbl_params.max_num_rules = 1;
	tbl_params.key.num_fields = 2;
	tbl_params.key.key_size = 8;
	tbl_params.key.proto_field[0].proto = MV_NET_PROTO_IP6;
	tbl_params.key.proto_field[0].field.eth = MV_NET_IP6_F_SA;
	tbl_params.key.proto_field[1].proto = MV_NET_PROTO_IP6;
	tbl_params.key.proto_field[1].field.eth = MV_NET_IP6_F_DA;
	pp2_cls_mng_tbl_init(&tbl_params, &tbl_hash, MVPP2_CLS_LKP_MUSDK_LOG_HASH);

	/* add 5 tuple hash rule ipv4 */
	tbl_params.type = PP2_CLS_TBL_EXACT_MATCH;
	tbl_params.max_num_rules = 1;
	tbl_params.key.num_fields = PP2_CLS_TBL_MAX_NUM_FIELDS;
	tbl_params.key.key_size = 13;
	tbl_params.key.proto_field[0].proto = MV_NET_PROTO_IP4;
	tbl_params.key.proto_field[0].field.eth = MV_NET_IP4_F_SA;
	tbl_params.key.proto_field[1].proto = MV_NET_PROTO_IP4;
	tbl_params.key.proto_field[1].field.eth = MV_NET_IP4_F_DA;
	tbl_params.key.proto_field[2].proto = MV_NET_PROTO_L4;
	tbl_params.key.proto_field[2].field.eth = MV_NET_L4_F_SP;
	tbl_params.key.proto_field[3].proto = MV_NET_PROTO_L4;
	tbl_params.key.proto_field[3].field.eth = MV_NET_L4_F_DP;
	tbl_params.key.proto_field[4].proto = MV_NET_PROTO_IP4;
	tbl_params.key.proto_field[4].field.eth = MV_NET_IP4_F_PROTO;
	pp2_cls_mng_tbl_init(&tbl_params, &tbl_hash, MVPP2_CLS_LKP_MUSDK_LOG_HASH);

	/* add 5 tuple hash rule ipv6 */
	tbl_params.type = PP2_CLS_TBL_EXACT_MATCH;
	tbl_params.max_num_rules = 1;
	tbl_params.key.num_fields = PP2_CLS_TBL_MAX_NUM_FIELDS;
	tbl_params.key.key_size = 36;
	tbl_params.key.proto_field[0].proto = MV_NET_PROTO_IP6;
	tbl_params.key.proto_field[0].field.eth = MV_NET_IP6_F_SA;
	tbl_params.key.proto_field[1].proto = MV_NET_PROTO_IP6;
	tbl_params.key.proto_field[1].field.eth = MV_NET_IP6_F_DA;
	tbl_params.key.proto_field[2].proto = MV_NET_PROTO_L4;
	tbl_params.key.proto_field[2].field.eth = MV_NET_L4_F_SP;
	tbl_params.key.proto_field[3].proto = MV_NET_PROTO_L4;
	tbl_params.key.proto_field[3].field.eth = MV_NET_L4_F_DP;
	tbl_params.key.proto_field[4].proto = MV_NET_PROTO_IP6;
	tbl_params.key.proto_field[4].field.eth = MV_NET_IP6_F_NEXT_HDR;
	pp2_cls_mng_tbl_init(&tbl_params, &tbl_hash, MVPP2_CLS_LKP_MUSDK_LOG_HASH);

	/* add default c2 rule */
	rule.num_fields = 0;
	pp2_cls_mng_rule_add(tbl, &rule, &tbl_params.default_act, MVPP2_CLS_LKP_MUSDK_VLAN_PRI);
	pp2_cls_mng_rule_add(tbl, &rule, &tbl_params.default_act, MVPP2_CLS_LKP_MUSDK_DSCP_PRI);
	pp2_cls_mng_rule_add(tbl, &rule, &tbl_params.default_act, MVPP2_CLS_LKP_MUSDK_LOG_PORT_DEF);

	/* set rss mode */
	pp2_cls_rss_mode_flows_set(port, port->hash_type);

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

	rc = pp2_prs_set_log_port(port, &params->specific_type_params.log_port_params);
	if (rc) {
		pr_err("%s(%d) pp2_prs_set_log_port fail\n", __func__, __LINE__);
		return -EINVAL;
	}

	return 0;
}

/*
 * pp2_cls_mng_eth_start_header_params_set()
 * configure parser ethernet start header (for DSA)
 */
int pp2_cls_mng_eth_start_header_params_set(struct pp2_ppio *ppio,
					    enum pp2_ppio_eth_start_hdr eth_start_hdr)
{
	struct pp2_port *port = GET_PPIO_PORT(ppio);
	int rc;

	rc = pp2_prs_eth_start_header_set(port, eth_start_hdr);
	if (rc) {
		pr_err("%s(%d) pp2_prs_eth_start_header_set fail\n", __func__, __LINE__);
		return -EINVAL;
	}

	return 0;
}

/*
 * pp2_cls_mng_set_policing()
 * configure policer and color for defaults flows
 */
int pp2_cls_mng_set_default_policing(struct pp2_ppio *ppio, int clear)
{
	struct pp2_port *port = GET_PPIO_PORT(ppio);
	u32 ref_cnt;
	int rc;

	if (!port->default_plcr)
		return 0;

	if (!clear) {
		rc = pp2_cls_plcr_ref_cnt_get(pp2_ptr->pp2_inst[ppio->pp2_id], port->default_plcr->id, NULL, &ref_cnt);
		if (rc || ref_cnt) {
			pr_err("[%s] policer already in use by other ppio\n", __func__);
			return -EFAULT;
		}
	}

	rc = pp2_c2_set_default_policing(port, clear);
	if (rc) {
		pr_err("%s(%d) pp2_cls_mng_set_policing fail\n", __func__, __LINE__);
		return -EINVAL;
	}

	rc = pp2_cls_plcr_ref_cnt_update(pp2_ptr->pp2_inst[ppio->pp2_id],
					 port->default_plcr->id,
					 (clear) ? MVPP2_PLCR_REF_CNT_DEC : MVPP2_PLCR_REF_CNT_INC,
					 true);
	if (rc) {
		pr_err("%s(%d) pp2_cls_plcr_ref_cnt_update fail\n", __func__, __LINE__);
		return -EFAULT;
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
	u32 i, num_lkpid = 0, field_index;
	u32 five_tuple = 0;
	u32 ipv4_flag = 0;
	u32 ipv6_flag = 0;
	u32 tcp_flag = 0;
	u32 udp_flag = 0;
	u32 l4_flag = 0;
	u16 select_logical_id[30];

	if (mv_pp2x_ptr_validate(params)) {
		pr_err("%s(%d) fail, param = NULL\n", __func__, __LINE__);
		return -EINVAL;
	}

	/* get packet processor instance */
	ppio = params->default_act.cos->ppio;
	port = GET_PPIO_PORT(ppio);
	inst = port->parent;

	if ((params->type != PP2_CLS_TBL_EXACT_MATCH) && (params->type != PP2_CLS_TBL_MASKABLE)) {
		pr_err("%s(%d) fail, engine type = %d is out of range\n", __func__, __LINE__, params->type);
		return -EINVAL;
	}

	if (mv_pp2x_range_validate(params->default_act.cos->tc, 0, port->num_tcs)) {
		pr_err("%s(%d) fail, tc = %d is out of range\n", __func__, __LINE__, params->default_act.cos->tc);
		return -EINVAL;
	}

	if (mv_pp2x_range_validate(params->key.key_size, 0, CLS_MNG_KEY_SIZE_MAX)) {
		pr_err("%s(%d) fail, key_size = %d is out of range\n", __func__, __LINE__, params->key.key_size);
		return -EINVAL;
	}

	if (mv_pp2x_range_validate(params->key.num_fields, 0, PP2_CLS_TBL_MAX_NUM_FIELDS)) {
		pr_err("%s(%d) fail, num_fields = %d is out of range\n", __func__, __LINE__, params->key.num_fields);
		return -EINVAL;
	}

	if ((params->default_act.type != PP2_CLS_TBL_ACT_DROP) && (params->default_act.type != PP2_CLS_TBL_ACT_DONE)) {
		pr_err("%s(%d) fail, action type = %d is out of range\n", __func__, __LINE__, params->default_act.type);
		return -EINVAL;
	}

	fl_rls = kmalloc(sizeof(*fl_rls), GFP_KERNEL);
	if (!fl_rls)
		return -ENOMEM;

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
			fl_rls->fl[0].engine = (lkp_type == MVPP2_CLS_LKP_MUSDK_LOG_HASH) ?
						MVPP2_CLS_ENGINE_C3HB : MVPP2_CLS_ENGINE_C3B;
		else
			fl_rls->fl[0].engine = (lkp_type == MVPP2_CLS_LKP_MUSDK_LOG_HASH) ?
						MVPP2_CLS_ENGINE_C3HA : MVPP2_CLS_ENGINE_C3A;
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
	/* as default DSCP flows should be disabled */
	fl_rls->fl[0].enabled = (lkp_type != MVPP2_CLS_LKP_MUSDK_DSCP_PRI) ? true : false;

	fl_rls->fl[0].prio = pp2_cls_mng_lkp_type_to_prio(lkp_type);
	if (fl_rls->fl[0].prio < 0)
		return -EINVAL;

	fl_rls->fl[0].udf7 = (port->type == PP2_PPIO_T_LOG) ? MVPP2_CLS_MUSDK_LOG_UDF7 : MVPP2_CLS_MUSDK_NIC_UDF7;
	fl_rls->fl[0].seq_ctrl = MVPP2_CLS_DEF_SEQ_CTRL;
	fl_rls->fl[0].field_id_cnt = params->key.num_fields - five_tuple;

	pr_debug("ipv4_flag = %d\n", ipv4_flag);
	pr_debug("ipv6_flag = %d\n", ipv6_flag);
	pr_debug("l4_flag = %d\n", l4_flag);
	pr_debug("udp_flag = %d\n", udp_flag);
	pr_debug("tcp_flag = %d\n", tcp_flag);

	if (lkp_type == MVPP2_CLS_LKP_MUSDK_CLS) {
		num_lkpid = pp2_cls_mng_get_lkpid_for_flow_type(&select_logical_id[0], ipv4_flag, ipv6_flag,
								tcp_flag, udp_flag, l4_flag);
	} else if (lkp_type == MVPP2_CLS_LKP_MUSDK_LOG_HASH) {
		num_lkpid = pp2_cls_mng_get_lkpid_for_rss(fl_rls->fl[0].engine, &select_logical_id[0],
							  ipv4_flag, ipv6_flag);
	} else {
		num_lkpid = pp2_cls_mng_get_lkpid_for_lkp_type(lkp_type, &select_logical_id[0]);
	}

	/* add current rule for all selected logical flow id */
	for (i = 0; i < num_lkpid; i++) {
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
	tbl_node->type = PP2_CLS_FLOW_TBL;
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
	struct pp2_port *port;
	struct pp2_cls_tbl_rule *rule = NULL;
	u32 rc;
	struct pp2_cls_fl_rule_entry_t fl;
	int idx;
	u32 field, match_bm;
	u32 field_index;

	if (mv_pp2x_ptr_validate(tbl)) {
		pr_err("%s(%d) fail, tbl = NULL\n", __func__, __LINE__);
		return -EINVAL;
	}

	/* Remove configured rules in table */
	for (i = 0; i < tbl->params.max_num_rules; i++) {
		rc = pp2_cls_db_mng_tbl_rule_next_get(tbl, &rule);
		if (!rc)
			pp2_cls_mng_rule_remove(tbl, rule);
	}

	/* Remove flow from HW */
	port = GET_PPIO_PORT(tbl->params.default_act.cos->ppio);
	field_index = 0;
	fl.lu_type = MVPP2_CLS_LKP_MUSDK_CLS;
	fl.port_type = MVPP2_SRC_PORT_TYPE_PHY;
	fl.port_bm = (1 << port->id);

	for (idx = 0; idx < tbl->params.key.num_fields; idx++) {
		rc = lookup_field_id(tbl->params.key.proto_field[idx].proto,
				     tbl->params.key.proto_field[idx].field.eth, &field, &match_bm);
		if (rc) {
			pr_err("%s(%d) lookup id error!\n", __func__, __LINE__);
			return -EFAULT;
		}
		fl.field_id[field_index++] = field;
	}

	(tbl->params.key.num_fields < 5) ?
		(fl.field_id_cnt = tbl->params.key.num_fields) :
		(fl.field_id_cnt = tbl->params.key.num_fields - 1);
	pp2_cls_rule_disable(port, &fl);

	/* Remove rules and table from database */
	kfree(tbl->params.default_act.cos);

	rc = pp2_cls_db_mng_tbl_remove(tbl);
	if (rc) {
		pr_err("%s(%d) Error while removing table from db!\n", __func__, __LINE__);
		return -EFAULT;
	}
	return 0;
}

int pp2_cls_mng_qos_tbl_init(struct pp2_cls_qos_tbl_params *qos_params,
			     struct pp2_cls_tbl **tbl)
{
	int rc = 0;
	u32 i;
	u8 tc_array[MVPP2_QOS_TBL_LINE_NUM_DSCP];
	struct pp2_port *port;
	struct pp2_cls_tbl *tmp_tbl = NULL;

	/* Para check */
	if (mv_pp2x_ptr_validate(qos_params)) {
		pr_err("%s(%d) fail, params = NULL\n", __func__, __LINE__);
		return -EINVAL;
	}

	if (qos_params->type <= PP2_CLS_QOS_TBL_NONE ||
	    qos_params->type >= PP2_CLS_QOS_TBL_OUT_OF_RANGE) {
		pr_err("QoS type %u out of range\n", qos_params->type);
		return -EINVAL;
	}

	port = GET_PPIO_PORT(qos_params->dscp_cos_map[0].ppio);

	/*Configure dscp flows */
	pp2_cls_dscp_flows_set(port, qos_params->type);

	/* Set up QoS lookup tables for PCP*/
	if (qos_params->type == PP2_CLS_QOS_TBL_VLAN_PRI ||
	    qos_params->type == PP2_CLS_QOS_TBL_VLAN_IP_PRI ||
	    qos_params->type == PP2_CLS_QOS_TBL_IP_VLAN_PRI) {
		for (i = 0; i < MV_VLAN_PRIO_NUM; i++) {
			if (!qos_params->pcp_cos_map[i].ppio) {
				pr_err("PCP field %u has NULL ppio.", i);
				return -EINVAL;
			}
			if (qos_params->pcp_cos_map->tc < 0 ||
			    qos_params->pcp_cos_map->tc > port->num_tcs) {
				pr_err("pcp tc value out of range.%d",
					qos_params->pcp_cos_map->tc);
				return -EINVAL;
			}
			tc_array[i] = qos_params->pcp_cos_map[i].tc;
		}

		rc = mv_pp2x_cls_c2_qos_tbl_fill_array(port,
						  MVPP2_QOS_TBL_SEL_PRI,
						  tc_array);
		if (rc) {
			pr_err("mv_pp2x_cls_c2_qos_tbl_fill_array failed\n");
			return -EINVAL;
		}
	}

	/* Set up QoS lookup tables for DSCP*/
	if (qos_params->type == PP2_CLS_QOS_TBL_IP_PRI ||
	    qos_params->type == PP2_CLS_QOS_TBL_VLAN_IP_PRI ||
	    qos_params->type == PP2_CLS_QOS_TBL_IP_VLAN_PRI) {
		for (i = 0; i < MV_DSCP_NUM; i++) {
			if (!qos_params->dscp_cos_map[i].ppio) {
				pr_err("DSCP field %u has NULL ppio.", i);
				return -EINVAL;
			}

			if (qos_params->dscp_cos_map->tc < 0 ||
			    qos_params->dscp_cos_map->tc > port->num_tcs) {
				pr_err("dscp tc value out of range.%d",
					qos_params->dscp_cos_map->tc);
				return -EINVAL;
			}

			tc_array[i] = qos_params->dscp_cos_map[i].tc;
		}

		rc = mv_pp2x_cls_c2_qos_tbl_fill_array(port,
						  MVPP2_QOS_TBL_SEL_DSCP,
						  tc_array);
		if (rc) {
			pr_err("mv_pp2x_cls_c2_qos_tbl_fill_array failed\n");
			return -EINVAL;
		}
	}

	rc = pp2_cls_db_mng_tbl_add(&tmp_tbl);
	memcpy(&tmp_tbl->qos_params, qos_params, sizeof(tmp_tbl->qos_params));
	tmp_tbl->type = PP2_CLS_QOS_TBL;

	*tbl = tmp_tbl;

	return rc;
}

int pp2_cls_mng_qos_tbl_deinit(struct pp2_cls_tbl *tbl)
{
	int rc = 0;
	u32 i;
	struct pp2_port *port;
	u8 tc_array[MVPP2_QOS_TBL_LINE_NUM_DSCP];

	port = GET_PPIO_PORT(tbl->qos_params.dscp_cos_map[0].ppio);

	/*Disable flows */
	pp2_cls_dscp_flows_set(port, PP2_CLS_QOS_TBL_NONE);

	for (i = 0; i < MV_DSCP_NUM; i++)
		tc_array[i] = 0;

	rc = mv_pp2x_cls_c2_qos_tbl_fill_array(port,
					  MVPP2_QOS_TBL_SEL_DSCP,
					  tc_array);
	if (rc) {
		pr_err("mv_pp2x_cls_c2_qos_tbl_fill_array failed\n");
		return -EINVAL;
	}

	rc = mv_pp2x_cls_c2_qos_tbl_fill_array(port,
					  MVPP2_QOS_TBL_SEL_PRI,
					  tc_array);
	if (rc) {
		pr_err("mv_pp2x_cls_c2_qos_tbl_fill_array failed\n");
		return -EINVAL;
	}

	rc = pp2_cls_db_mng_tbl_remove(tbl);
	if (rc) {
		pr_err("removing qos table from db failed\n");
		return -EFAULT;
	}
	return 0;
}

static int pp2_cls_set_rule_info(struct pp2_cls_mng_pkt_key_t *mng_pkt_key,
				 struct mv_pp2x_src_port *rule_port,
				 struct pp2_cls_tbl_params *params,
				 struct pp2_cls_tbl_rule *rule,
				 struct pp2_port *port)
{
	char *mask_ptr;
	char mask_arr[3];
	int rc = 0, i;
	u32 proto_flag = 0;
	u32 ipv4_flag = 0;
	u32 ipv6_flag = 0;
	u32 field;
	u16 ipproto;
	u32 idx1, idx2;
	u32 field_bm = 0, bm = 0;

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
			pr_debug("MAC DA: %02x:%02x:%02x:%02x:%02x:%02x\n",
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
			pr_debug("MAC DA MASK: %02x:%02x:%02x:%02x:%02x:%02x\n",
				 mng_pkt_key->pkt_key->eth_dst.eth_add_mask[0],
				 mng_pkt_key->pkt_key->eth_dst.eth_add_mask[1],
				 mng_pkt_key->pkt_key->eth_dst.eth_add_mask[2],
				 mng_pkt_key->pkt_key->eth_dst.eth_add_mask[3],
				 mng_pkt_key->pkt_key->eth_dst.eth_add_mask[4],
				 mng_pkt_key->pkt_key->eth_dst.eth_add_mask[5]);
			break;
		case ETH_TYPE_FIELD_ID:
			if (rule->fields[idx1].size != (GET_NUM_BYTES(ETH_TYPE_FIELD_SIZE))) {
				pr_err("%s(%d) field size does not match! %d\n", __func__, __LINE__,
				       rule->fields[idx1].size);
				return -EINVAL;
			}
			rc = kstrtou16((char *)(rule->fields[idx1].key), 0, &mng_pkt_key->pkt_key->ether_type);
			if (rc) {
				pr_err("Failed to parse eth_type header.\n");
				return rc;
			}
			if (mng_pkt_key->pkt_key->ether_type > ((1 << ETH_TYPE_FIELD_SIZE) - 1)) {
				pr_err("eth_type exceeds max val!.\n");
				return rc;
			}
			pr_debug("ETH_TYPE_FIELD_ID = %d\n", mng_pkt_key->pkt_key->ether_type);
			break;
		case OUT_VLAN_PRI_FIELD_ID:
			if (rule->fields[idx1].size != (GET_NUM_BYTES(OUT_VLAN_PRI_FIELD_SIZE))) {
				pr_err("%s(%d) field size does not match! %d\n", __func__, __LINE__,
				       rule->fields[idx1].size);
				return -EINVAL;
			}
			rc = kstrtou8((char *)(rule->fields[idx1].key), 0, &mng_pkt_key->pkt_key->out_pbit);
			if (rc) {
				pr_err("Failed to parse PCP bits in VLAN header.\n");
				return rc;
			}
			if (mng_pkt_key->pkt_key->out_pbit >= MV_VLAN_PRIO_NUM) {
				pr_err("Key exceeds max val! %d\n", MV_VLAN_PRIO_NUM - 1);
				return -EINVAL;
			}
			pr_debug("OUT_VLAN_PRI_FIELD_ID = %d\n", mng_pkt_key->pkt_key->out_pbit);
			break;
		case OUT_VLAN_ID_FIELD_ID:
			if (rule->fields[idx1].size != (GET_NUM_BYTES(OUT_VLAN_ID_FIELD_SIZE))) {
				pr_err("%s(%d) field size does not match! %d\n", __func__, __LINE__,
				       rule->fields[idx1].size);
				return -EINVAL;
			}
			rc = kstrtou16((char *)(rule->fields[idx1].key), 0, &mng_pkt_key->pkt_key->out_vid);
			if (rc) {
				pr_err("Failed to parse VID in VLAN header.\n");
				return rc;
			}
			if (mng_pkt_key->pkt_key->out_vid > ((1 << OUT_VLAN_ID_FIELD_SIZE) - 1)) {
				pr_err("vlan_id exceeds max val!.\n");
				return rc;
			}
			pr_debug("OUT_VLAN_ID_FIELD_ID = %d\n", mng_pkt_key->pkt_key->out_vid);
			break;
		case IPV4_DSCP_FIELD_ID:
			if (rule->fields[idx1].size != (GET_NUM_BYTES(IPV4_DSCP_FIELD_SIZE))) {
				pr_err("%s(%d) field size does not match! %d\n", __func__, __LINE__,
				       rule->fields[idx1].size);
				return -EINVAL;
			}
			rc = kstrtou8((char *)(rule->fields[idx1].key), 0, &mng_pkt_key->pkt_key->ipvx_add.dscp);
			if (rc) {
				pr_err("Failed to parse DSCP field.\n");
				return rc;
			}
			if (mng_pkt_key->pkt_key->ipvx_add.dscp >= MV_DSCP_NUM) {
				pr_err("Key exceeds max val! %d\n", MV_DSCP_NUM - 1);
				return -EINVAL;
			}
			pr_debug("OUT_VLAN_ID_FIELD_ID = %d\n", mng_pkt_key->pkt_key->ipvx_add.dscp);
			rc = kstrtou8((char *)(rule->fields[idx1].mask), 0, &mng_pkt_key->pkt_key->ipvx_add.dscp_mask);
			if (rc) {
				pr_err("Failed to parse DSCP mask.\n");
				return rc;
			}
			pr_debug("OUT_VLAN_ID_FIELD_ID mask = %d\n", mng_pkt_key->pkt_key->ipvx_add.dscp_mask);
			break;
		case IPV4_SA_FIELD_ID:
			if (rule->fields[idx1].size != (GET_NUM_BYTES(IPV4_SA_FIELD_SIZE))) {
				pr_err("%s(%d) field size does not match! %d\n", __func__, __LINE__,
				       rule->fields[idx1].size);
				return -EINVAL;
			}
			rc = in4_pton((char *)rule->fields[idx1].key, strlen((char *)rule->fields[idx1].key),
				      &mng_pkt_key->pkt_key->ipvx_add.ip_src.ip_add.ipv4[0], '.', NULL);
			if (!rc) {
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
				mask_arr[2] = '\0';
				rc = kstrtou8(mask_arr, 16, &mng_pkt_key->pkt_key->ipvx_add.ip_src.ip_add_mask.ipv4[i]);
				if (rc) {
					pr_err("Unable to parse IPv4 SA mask\n");
					return rc;
				}
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
			rc = in4_pton((char *)rule->fields[idx1].key, strlen((char *)rule->fields[idx1].key),
				      &mng_pkt_key->pkt_key->ipvx_add.ip_dst.ip_add.ipv4[0], '.', NULL);
			if (!rc) {
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
				mask_arr[2] = '\0';
				rc = kstrtou8(mask_arr, 16,
					      &mng_pkt_key->pkt_key->ipvx_add.ip_dst.ip_add_mask.ipv4[i]);
				if (rc) {
					pr_err("Unable to parse IPv4 DA mask\n");
					return rc;
				}
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
			rc = kstrtou16((char *)(rule->fields[idx1].key), 0, &ipproto);
			if (rc) {
				pr_err("Unable to parse Ipv6 protocol");
				return rc;
			}
			mng_pkt_key->pkt_key->ipvx_add.ip_proto = ipproto;
			if ((ipproto == IPPROTO_UDP) || (ipproto == IPPROTO_TCP))
				/* Turn on flag indicating tcp/udp, used for detecting 5 tuple configuration */
				proto_flag = 1;
			pr_debug("protocol: %d\n", ipproto);
			break;
		case IPV6_SA_FIELD_ID:
			if (rule->fields[idx1].size != (GET_NUM_BYTES(IPV6_SA_FIELD_SIZE))) {
				pr_err("%s(%d) field size does not match! %d\n", __func__, __LINE__,
				       rule->fields[idx1].size);
				return -EINVAL;
			}

			rc = in6_pton((char *)rule->fields[idx1].key, strlen((char *)rule->fields[idx1].key),
				      &mng_pkt_key->pkt_key->ipvx_add.ip_src.ip_add.ipv6[0], ':', NULL);
			if (!rc) {
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

			rc = in6_pton((char *)rule->fields[idx1].key, strlen((char *)rule->fields[idx1].key),
				      &mng_pkt_key->pkt_key->ipvx_add.ip_dst.ip_add.ipv6[0], ':', NULL);
			if (!rc) {
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
		case IPV6_FLOW_LBL_FIELD_ID:
			if (rule->fields[idx1].size != (GET_NUM_BYTES(IPV6_FLOW_LBL_FIELD_SIZE))) {
				pr_err("%s(%d) field size does not match! %d\n", __func__, __LINE__,
				       rule->fields[idx1].size);
				return -EINVAL;
			}

			rc = kstrtou32((char *)(rule->fields[idx1].key), 0, &mng_pkt_key->pkt_key->ipvx_add.flow_label);
			if (rc) {
				pr_err("%s(%d)) Falied to parse IPv6 flow label.", __func__, __LINE__);
				return rc;
			}
			pr_debug("IPV6_FLOW_LBL_FIELD_ID = %x\n", mng_pkt_key->pkt_key->ipvx_add.flow_label);

			if (mng_pkt_key->pkt_key->ipvx_add.flow_label > (1 << IPV6_FLOW_LBL_FIELD_SIZE) - 1) {
				pr_err("%s(%d)) IPv6 flow label.value too big. Max value %x", __func__, __LINE__,
				       ((1 << IPV6_FLOW_LBL_FIELD_SIZE) - 1));
				return -EFAULT;
			}

			rc = kstrtou32((char *)(rule->fields[idx1].mask), 0,
					&mng_pkt_key->pkt_key->ipvx_add.flow_label_mask);
			if (rc) {
				pr_err("Failed to parse IPv6 flow label mask.\n");
				return rc;
			}
			pr_debug("IPV6_FLOW_LBL_FIELD_ID mask = %x\n", mng_pkt_key->pkt_key->ipvx_add.flow_label_mask);

			if (mng_pkt_key->pkt_key->ipvx_add.flow_label_mask > (1 << IPV6_FLOW_LBL_FIELD_SIZE) - 1) {
				pr_err("%s(%d)) IPv6 flow label.mask value too big. Max value %x", __func__, __LINE__,
				       ((1 << IPV6_FLOW_LBL_FIELD_SIZE) - 1));
				return -EFAULT;
			}
			break;
		case L4_SRC_FIELD_ID:
			if (rule->fields[idx1].size != (GET_NUM_BYTES(L4_SRC_FIELD_SIZE))) {
				pr_err("%s(%d) field size does not match! %d\n", __func__, __LINE__,
				       rule->fields[idx1].size);
				return -EINVAL;
			}
			rc = kstrtou16((char *)(rule->fields[idx1].key), 0, &mng_pkt_key->pkt_key->l4_src);
			if (rc) {
				pr_err("%s(%d)) Falied to parse L4 source port.", __func__, __LINE__);
				return rc;
			}

			pr_debug("L4_SRC_FIELD_ID = %d\n", mng_pkt_key->pkt_key->l4_src);
			break;
		case L4_DST_FIELD_ID:
			if (rule->fields[idx1].size != (GET_NUM_BYTES(L4_DST_FIELD_SIZE))) {
				pr_err("%s(%d) field size does not match! %d\n", __func__, __LINE__,
				       rule->fields[idx1].size);
				return -EINVAL;
			}
			rc = kstrtou16((char *)(rule->fields[idx1].key), 0, &mng_pkt_key->pkt_key->l4_dst);
			if (rc) {
				pr_err("%s(%d)) Falied to parse L4 destination port.", __func__, __LINE__);
				return rc;
			}

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

	return 0;
}

static void pp2_cls_mng_set_c2_action(struct pp2_port *port,
				   struct mv_pp2x_engine_qos_info *qos_info,
				   struct mv_pp2x_qos_value *pkt_qos,
				   struct mv_pp2x_engine_pkt_action *pkt_action,
				   struct pp2_cls_tbl_action *action,
				   int lkp_type)
{
	u8 queue;
	u8 tc_array[MVPP2_QOS_TBL_LINE_NUM_DSCP] = {0};

	if (action->type == PP2_CLS_TBL_ACT_DROP)
		pkt_action->color_act = MVPP2_COLOR_ACTION_TYPE_RED_LOCK;
	else {
		switch (port->tc[action->cos->tc].tc_config.default_color) {
		case PP2_PPIO_COLOR_GREEN:
			pkt_action->color_act = MVPP2_COLOR_ACTION_TYPE_GREEN;
			break;
		case PP2_PPIO_COLOR_YELLOW:
			pkt_action->color_act = MVPP2_COLOR_ACTION_TYPE_YELLOW;
			break;
		case PP2_PPIO_COLOR_RED:
			pkt_action->color_act = MVPP2_COLOR_ACTION_TYPE_RED;
			break;
		}
	}
	pkt_action->policer_act = MVPP2_ACTION_TYPE_NO_UPDT;
	pkt_action->flowid_act = MVPP2_ACTION_FLOWID_DISABLE;
	pkt_action->frwd_act = MVPP2_ACTION_TYPE_NO_UPDT;
	pkt_action->rss_act = MVPP2_ACTION_TYPE_UPDT_LOCK;

	if (action->plcr) {
		pkt_action->policer_act = MVPP2_ACTION_TYPE_UPDT_LOCK;
		qos_info->policer_id = action->plcr->id;
	}

	/* for qos rules (only activated when logical port is initialized) */
	if (lkp_type == MVPP2_CLS_LKP_MUSDK_DSCP_PRI ||
	    lkp_type == MVPP2_CLS_LKP_MUSDK_VLAN_PRI) {
		u8 tbl_sel;

		if (lkp_type == MVPP2_CLS_LKP_MUSDK_VLAN_PRI)
			tbl_sel = MVPP2_QOS_TBL_SEL_PRI;
		else if (lkp_type == MVPP2_CLS_LKP_MUSDK_DSCP_PRI)
			tbl_sel = MVPP2_QOS_TBL_SEL_DSCP;

		qos_info->qos_tbl_index = QOS_LOG_PORT_TABLE_OFF(port->id);
		qos_info->q_low_src = MVPP2_QOS_SRC_DSCP_PBIT_TBL;
		qos_info->q_high_src = MVPP2_QOS_SRC_DSCP_PBIT_TBL;
		qos_info->color_src = MVPP2_QOS_SRC_DSCP_PBIT_TBL;
		qos_info->qos_tbl_type = tbl_sel;
		pkt_action->q_low_act = MVPP2_ACTION_TYPE_UPDT_LOCK;
		pkt_action->q_high_act = MVPP2_ACTION_TYPE_UPDT_LOCK;

		mv_pp2x_cls_c2_qos_tbl_fill_array(port, tbl_sel, tc_array);
	} else {
	/* for classifier and default rules */
		if (action->cos->tc >= 0 && action->cos->tc < PP2_PPIO_MAX_NUM_TCS) {
			pkt_action->q_low_act = MVPP2_ACTION_TYPE_UPDT_LOCK;
			pkt_action->q_high_act = MVPP2_ACTION_TYPE_UPDT_LOCK;
			queue = port->tc[action->cos->tc].tc_config.first_rxq;
			pkt_qos->q_high = ((u16)queue) >> MVPP2_CLS2_ACT_QOS_ATTR_QL_BITS;
			pkt_qos->q_low = ((u16)queue) & ((1 << MVPP2_CLS2_ACT_QOS_ATTR_QL_BITS) - 1);
			qos_info->q_low_src = MVPP2_QOS_SRC_ACTION_TBL;
			qos_info->q_high_src = MVPP2_QOS_SRC_ACTION_TBL;
			pr_debug("q_low %d, q_high %d, queue %d, tc %d\n", pkt_qos->q_low,
				 pkt_qos->q_high, queue, action->cos->tc);
		} else {
			pkt_action->q_low_act = MVPP2_ACTION_TYPE_NO_UPDT;
			pkt_action->q_high_act = MVPP2_ACTION_TYPE_NO_UPDT;
		}
	}
}

static void pp2_cls_mng_set_c3_action(struct pp2_port *port,
				   struct mv_pp2x_engine_qos_info *qos_info,
				   struct mv_pp2x_qos_value *pkt_qos,
				   struct mv_pp2x_engine_pkt_action *pkt_action,
				   struct pp2_cls_tbl_action *action,
				   int lkp_type)
{
	u8 queue;

	if (action->type == PP2_CLS_TBL_ACT_DROP)
		pkt_action->color_act = MVPP2_COLOR_ACTION_TYPE_RED_LOCK;
	else {
		switch (port->tc[action->cos->tc].tc_config.default_color) {
		case PP2_PPIO_COLOR_GREEN:
			pkt_action->color_act = MVPP2_COLOR_ACTION_TYPE_GREEN;
			break;
		case PP2_PPIO_COLOR_YELLOW:
			pkt_action->color_act = MVPP2_COLOR_ACTION_TYPE_YELLOW;
			break;
		case PP2_PPIO_COLOR_RED:
			pkt_action->color_act = MVPP2_COLOR_ACTION_TYPE_RED;
			break;
		}
	}
	pkt_action->policer_act = MVPP2_ACTION_TYPE_NO_UPDT;
	pkt_action->flowid_act = MVPP2_ACTION_FLOWID_DISABLE;
	pkt_action->frwd_act = MVPP2_ACTION_TYPE_NO_UPDT;
	pkt_action->rss_act = MVPP2_ACTION_TYPE_UPDT_LOCK;

	if (action->plcr) {
		pkt_action->policer_act = MVPP2_ACTION_TYPE_UPDT_LOCK;
		qos_info->policer_id = action->plcr->id;
	}

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
	action_db->plcr = action->plcr;
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
	struct mv_pp2x_src_port rule_port;
	struct pp2_port *port;
	struct pp2_inst *inst;
	u32 rc = 0, logic_idx;
	struct pp2_cls_tbl_params *params = &tbl->params;
	struct pp2_cls_tbl_rule *rule_db;
	struct pp2_cls_tbl_action *action_db;

	/* check table type */
	if (tbl->type != PP2_CLS_FLOW_TBL) {
		pr_err("%s(%d) wrong table type inserted\n", __func__, __LINE__);
		return -EFAULT;
	}

	/* check if table exists in DB */
	rc = pp2_cls_db_mng_tbl_check(tbl);
	if (rc) {
		pr_err("table not found in db\n");
		return -EIO;
	}

	/* check rule is not duplicated */
	rc = pp2_cls_db_mng_rule_check(tbl, rule);
	if (rc) {
		pr_warn("duplicated rule, ignoring request\n");
		return 0;
	}

	/* init value */
	MVPP2_MEMSET_ZERO(pkt_key);
	MVPP2_MEMSET_ZERO(mng_pkt_key);
	mng_pkt_key.pkt_key = &pkt_key;

	port = GET_PPIO_PORT(params->default_act.cos->ppio);
	inst = port->parent;

	if (mv_pp2x_range_validate(rule->num_fields, 0, PP2_CLS_TBL_MAX_NUM_FIELDS)) {
		pr_err("%s(%d) fail, num_fields = %d is out of range\n", __func__, __LINE__, rule->num_fields);
		return -EINVAL;
	}

	if (mv_pp2x_range_validate(action->cos->tc, 0, port->num_tcs)) {
		pr_err("%s(%d) fail, tc = %d is out of range\n", __func__, __LINE__, action->cos->tc);
		return -EINVAL;
	}

	if ((action->type != PP2_CLS_TBL_ACT_DROP) && (action->type != PP2_CLS_TBL_ACT_DONE)) {
		pr_err("%s(%d) fail, action type = %d is out of range\n", __func__, __LINE__, action->type);
		return -EINVAL;
	}

	rc = pp2_cls_set_rule_info(&mng_pkt_key, &rule_port, params, rule, port);
	if (rc) {
		pr_err("%s(%d) pp2_cls_set_rule_info failed\n", __func__, __LINE__);
		return rc;
	}

	if (params->type == PP2_CLS_TBL_MASKABLE) {
		struct mv_pp2x_c2_add_entry c2_entry;

		MVPP2_MEMSET_ZERO(c2_entry);
		c2_entry.mng_pkt_key = &mng_pkt_key;
		c2_entry.mng_pkt_key->pkt_key = &pkt_key;
		c2_entry.lkp_type = lkp_type;
		c2_entry.lkp_type_mask = MVPP2_C2_HEK_LKP_TYPE_MASK >> MVPP2_C2_HEK_LKP_TYPE_OFFS;
		c2_entry.rss_en = port->rss_en;
		pp2_cls_mng_set_c2_action(port,
					  &c2_entry.qos_info,
					  &c2_entry.qos_value,
					  &c2_entry.action,
					  action,
					  lkp_type);

		memcpy(&c2_entry.port, &rule_port, sizeof(rule_port));

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
		c3_entry.rss_en = port->rss_en;
		pp2_cls_mng_set_c3_action(port,
					  &c3_entry.qos_info,
					  &c3_entry.qos_value,
					  &c3_entry.action,
					  action,
					  lkp_type);

		memcpy(&c3_entry.port, &rule_port, sizeof(rule_port));

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

	if (action->plcr) {
		rc = pp2_cls_plcr_ref_cnt_update(inst, action->plcr->id, MVPP2_PLCR_REF_CNT_INC, false);
		if (rc)
			return -EFAULT;
	}

	return 0;
}

int pp2_cls_mng_rule_remove(struct pp2_cls_tbl *tbl, struct pp2_cls_tbl_rule *rule)
{
	struct pp2_port *port;
	u32 rc;
	struct pp2_cls_tbl_params *params = &tbl->params;
	struct pp2_cls_tbl_action action;
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

	rc = pp2_cls_db_mng_tbl_rule_remove(tbl, rule, &logic_index, &action);
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

	if (action.plcr) {
		rc = pp2_cls_plcr_ref_cnt_update(inst, action.plcr->id, MVPP2_PLCR_REF_CNT_DEC, false);
		if (rc)
			return -EFAULT;
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

void pp2_cls_mng_rss_port_init(struct pp2_port *port, u16 rss_map)
{
	int rc, i;
	u32 num_queues = 0;

	port->rss_en = true;

	/* Check total number of TC's and number of in_queues per TC do
	 * not exceed maximum number of HW queues in port
	 */
	for (i = 0; i < port->num_tcs; i++)
		num_queues += port->tc[i].tc_config.num_in_qs;

	if (num_queues > PP2_PPIO_MAX_NUM_TCS) {
		pr_err("not enough hw queues to allocate %d TC's and RSS. Needed %d queues, available %d\n",
			port->num_tcs, num_queues, PP2_PPIO_MAX_NUM_TCS);
		pr_err("RSS is set to disabled\n");
		port->rss_en = false;
	}

	if (port->hash_type == PP2_PPIO_HASH_T_NONE)
		port->rss_en = false;
	else {
		/* calculate the required musdk rss table map (not including the kernel rss map) */
		rc = pp2_rss_musdk_map_get(port);
		if (rc) {
			pr_err("Error in pp2_rss_musdk_map_get\n");
			pr_err("RSS is set to disabled\n");
			port->rss_en = false;
		}
	}

	if (port->rss_en == true) {
		/* bind rxq to rss table for this port */
		if (pp22_cls_rss_rxq_set(port)) {
			pr_err("cannot allocate rss table for rxq\n");
			pr_err("RSS is set to disabled\n");
			port->rss_en = false;
		}

		/* Init RSS table */
		if (pp2_rss_hw_tbl_set(port)) {
			pr_err("cannot init rss hw table\n");
			pr_err("RSS is set to disabled\n");
			port->rss_en = false;
		}

		/* Configure hash type only for MUSDK port at this point (flows for logical port are not defined yet
		 *  at this point, so hash type is configured later for logical ports
		 */
		if (port->type == PP2_PPIO_T_NIC) {
			rc = pp2_cls_rss_mode_flows_set(port, port->hash_type);
			if (rc) {
				pr_err("cannot set hash type in flows\n");
				pr_err("RSS is set to disabled\n");
				port->rss_en = false;
			}
		}
	}

	/* Enable or disable RSS*/
	if (pp2_rss_enable(port, port->rss_en)) {
		pr_err("cannot enable rss\n");
		return;
	}
}

/*
 * Configure default cos queue for the specified port.
 * The cos queue configured is according to the first rx queue defined for the port.
 * Configuration is not needed for logical port
 * TODO review whenever API for configuring default cos queue is available
 */
void pp2_cls_mng_config_default_cos_queue(struct pp2_port *port)
{
	if (port->type == PP2_PPIO_T_NIC)
		pp2_c2_config_default_queue(port, port->first_rxq);
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
	pp2_cls_rss_init(inst);
	pp2_cls_plcr_start(inst);
}

void pp2_cls_mng_deinit(struct pp2_inst *inst)
{
	pp2_cls_prs_deinit(inst);
	pp2_cls_plcr_finish(inst);
}


