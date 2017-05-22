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

#include <string.h>
#include <stdio.h>
#include <getopt.h>
#include "mv_std.h"
#include "mvapp.h"
#include "mv_pp2.h"
#include "mv_pp2_ppio.h"

#include "drivers/mv_pp2_cls.h"
#include "src/drivers/ppv2/pp2.h"
#include "utils.h"
#include "cls_debug.h"

#include "lib/list.h"

#define CLS_APP_DMA_MEM_SIZE			(10 * 1024 * 1024)
#define CLS_APP_QS_MAP_MASK			0xFFFF
#define CLS_APP_MAX_NUM_TCS_PER_PORT		4
#define CLS_APP_FIRST_LOG_PORT_IN_QUEUE		4
#define CLS_APP_FIRST_MUSDK_IN_QUEUE		0
#define CLS_APP_STR_SIZE_MAX			40
#define CLS_APP_KEY_SIZE_MAX			37
#define CLS_APP_MAX_NUM_OF_RULES		20
#define CLS_APP_COMMAND_LINE_SIZE		256

#define CLS_APP_DEF_Q_SIZE			1024
#define CLS_APP_HIF_Q_SIZE			CLS_APP_DEF_Q_SIZE
#define CLS_APP_RX_Q_SIZE			CLS_APP_DEF_Q_SIZE
#define CLS_APP_TX_Q_SIZE			CLS_APP_DEF_Q_SIZE

#define CLS_APP_MAX_BURST_SIZE			(CLS_APP_RX_Q_SIZE >> 1)
#define CLS_APP_DFLT_BURST_SIZE			256

#define CLS_APP_MAX_NUM_QS_PER_CORE		MVAPPS_MAX_NUM_QS_PER_TC

#define CLS_APP_KEY_MEM_SIZE_MAX		(PP2_CLS_TBL_MAX_NUM_FIELDS * CLS_APP_STR_SIZE_MAX)

#define CLS_APP_PREFETCH_SHIFT			7
#define CLS_APP_PKT_ECHO_SUPPORT
#define CLS_APP_USE_APP_PREFETCH

#define CLS_APP_BPOOLS_INF		{ {2048, 1024} }

struct glob_arg {
	int			verbose;
	int			cli;
	int			cpus;	/* cpus used for running */
	int			echo;
	u32			hash_type;
	u64			qs_map;
	int			qs_map_shift;
	int			num_ports;
	int			pp2_num_inst;
	struct port_desc	ports_desc[MVAPPS_MAX_NUM_PORTS];
	struct pp2_hif		*hif;
	int			num_pools;
	struct bpool_desc	**pools_desc;
	struct pp2_init_params	pp2_params;
};

struct local_arg {
	struct tx_shadow_q	shadow_qs[CLS_APP_MAX_NUM_QS_PER_CORE];
	u64			 qs_map;

	struct pp2_hif		*hif;
	int			 num_ports;
	struct lcl_port_desc	*ports_desc;

	struct bpool_desc	**pools_desc;
	int			 echo;
	int			 id;

	struct glob_arg		*garg;
};

static struct glob_arg garg = {};

struct pp2_cls_table_node {
	u32				idx;
	struct	pp2_cls_tbl		*tbl;
	struct	pp2_cls_tbl_params	tbl_params;
	struct	pp2_cls_qos_tbl_params	qos_tbl_params;
	char				ppio_name[MVAPPS_PPIO_NAME_MAX];
	struct list			list_node;
};

static struct list cls_flow_tbl_head;
static struct list cls_qos_tbl_head;

static void app_print_horizontal_line(u32 char_count, const char *char_val)
{
	u32 cnt;

	for (cnt = 0; cnt < char_count; cnt++)
		printf("%s", char_val);
	printf("\n");
}

static int mv_pp2x_parse_mac_address(char *buf, u8 *macaddr_parts)
{
	if (sscanf(buf, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
		   &macaddr_parts[0], &macaddr_parts[1],
		   &macaddr_parts[2], &macaddr_parts[3],
		   &macaddr_parts[4], &macaddr_parts[5]) == ETH_ALEN)
		return 0;
	else
		return -EFAULT;
}

static int pp2_cls_convert_string_to_proto_and_field(u32 *proto, u32 *field)
{
	int key_size = -1;

	if (!strcmp(optarg, "eth_src")) {
		*proto = MV_NET_PROTO_ETH;
		*field = MV_NET_ETH_F_SA;
		key_size = 6;
	} else if (!strcmp(optarg, "eth_dst")) {
		*proto = MV_NET_PROTO_ETH;
		*field = MV_NET_ETH_F_DA;
		key_size = 6;
	} else if (!strcmp(optarg, "eth_type")) {
		*proto = MV_NET_PROTO_ETH;
		*field = MV_NET_ETH_F_TYPE;
		key_size = 1;
	} else if (!strcmp(optarg, "vlan_prio")) {
		*proto = MV_NET_PROTO_VLAN;
		*field = MV_NET_VLAN_F_PRI;
		key_size = 1;
	} else if (!strcmp(optarg, "vlan_id")) {
		*proto = MV_NET_PROTO_VLAN;
		*field = MV_NET_VLAN_F_ID;
		key_size = 2;
	} else if (!strcmp(optarg, "vlan_tci")) {
		*proto = MV_NET_PROTO_VLAN;
		*field = MV_NET_VLAN_F_TCI;
		key_size = 1;
	} else if (!strcmp(optarg, "pppoe")) {
		*proto = MV_NET_PROTO_PPPOE;
		*field = 0;
		key_size = 1;
	} else if (!strcmp(optarg, "ip")) {
		*proto = MV_NET_PROTO_IP;
		*field = 0;
		key_size = 1;
	} else if (!strcmp(optarg, "ip4_tos")) {
		*proto = MV_NET_PROTO_IP4;
		*field = MV_NET_IP4_F_TOS;
		key_size = 1;
	} else if (!strcmp(optarg, "ip4_src")) {
		*proto = MV_NET_PROTO_IP4;
		*field = MV_NET_IP4_F_SA;
		key_size = 4;
	} else if (!strcmp(optarg, "ip4_dst")) {
		*proto = MV_NET_PROTO_IP4;
		*field = MV_NET_IP4_F_DA;
		key_size = 4;
	} else if (!strcmp(optarg, "ip4_proto")) {
		*proto = MV_NET_PROTO_IP4;
		*field = MV_NET_IP4_F_PROTO;
		key_size = 1;
	} else if (!strcmp(optarg, "ip6_tc")) {
		*proto = MV_NET_PROTO_IP6;
		*field = MV_NET_IP6_F_TC;
		key_size = 1;
	} else if (!strcmp(optarg, "ip6_src")) {
		*proto = MV_NET_PROTO_IP6;
		*field = MV_NET_IP6_F_SA;
		key_size = 16;
	} else if (!strcmp(optarg, "ip6_dst")) {
		*proto = MV_NET_PROTO_IP6;
		*field = MV_NET_IP6_F_DA;
		key_size = 16;
	} else if (!strcmp(optarg, "ip6_flow")) {
		*proto = MV_NET_PROTO_IP6;
		*field = MV_NET_IP6_F_FLOW;
		key_size = 3;
	} else if (!strcmp(optarg, "ip6_next_hdr")) {
		*proto = MV_NET_PROTO_IP6;
		*field = MV_NET_IP6_F_NEXT_HDR;
		key_size = 1;
	} else if (!strcmp(optarg, "l4_src")) {
		*proto = MV_NET_PROTO_L4;
		*field = MV_NET_L4_F_SP;
		key_size = 2;
	} else if (!strcmp(optarg, "l4_dst")) {
		*proto = MV_NET_PROTO_L4;
		*field = MV_NET_L4_F_DP;
		key_size = 2;
	} else if (!strcmp(optarg, "l4_csum")) {
		*proto = MV_NET_PROTO_L4;
		*field = MV_NET_L4_F_CSUM;
		key_size = 2;
	} else if (!strcmp(optarg, "tcp_src")) {
		*proto = MV_NET_PROTO_TCP;
		*field = MV_NET_TCP_F_SP;
		key_size = 2;
	} else if (!strcmp(optarg, "tcp_dst")) {
		*proto = MV_NET_PROTO_TCP;
		*field = MV_NET_TCP_F_DP;
		key_size = 2;
	} else if (!strcmp(optarg, "tcp_csum")) {
		*proto = MV_NET_PROTO_TCP;
		*field = MV_NET_TCP_F_CSUM;
		key_size = 2;
	} else if (!strcmp(optarg, "udp_src")) {
		*proto = MV_NET_PROTO_UDP;
		*field = MV_NET_UDP_F_SP;
		key_size = 2;
	} else if (!strcmp(optarg, "udp_dst")) {
		*proto = MV_NET_PROTO_UDP;
		*field = MV_NET_UDP_F_DP;
		key_size = 2;
	} else if (!strcmp(optarg, "udp_csum")) {
		*proto = MV_NET_PROTO_UDP;
		*field = MV_NET_UDP_F_CSUM;
		key_size = 2;
	} else if (!strcmp(optarg, "icmp")) {
		*proto = MV_NET_PROTO_ICMP;
		*field = 0;
		key_size = 1;
	} else if (!strcmp(optarg, "arp")) {
		*proto = MV_NET_PROTO_ARP;
		*field = 0;
		key_size = 1;
	}
	return key_size;
}

static u32 pp2_cls_first_queue_get(void)
{
	/* TODO: need to read from kernel through sysfs function */
	return (garg.ports_desc[0].ppio_type == PP2_PPIO_T_LOG ?
		CLS_APP_FIRST_LOG_PORT_IN_QUEUE : CLS_APP_FIRST_MUSDK_IN_QUEUE);
}

/*
 * pp2_cls_table_next_index_get()
 * Get the next free table index in the list. The first index starts at 1.
 * in case entries were removed from list, this function returns the first free table index
 */
static int pp2_cls_table_next_index_get(struct list *cls_tbl_head)
{
	struct pp2_cls_table_node *tbl_node;
	int idx = 0;

	LIST_FOR_EACH_OBJECT(tbl_node, struct pp2_cls_table_node, cls_tbl_head, list_node) {
		if ((tbl_node->idx == 0) || ((tbl_node->idx - idx) > 1))
			return idx + 1;
		idx++;
	}
	return idx + 1;
}

/*
 * pp2_cls_table_get()
 * returns a pointer to the table for the provided table index
 */
static int pp2_cls_table_get(u32 tbl_idx, struct pp2_cls_tbl **tbl, struct list *cls_tbl_head)
{
	struct pp2_cls_table_node *tbl_node;

	LIST_FOR_EACH_OBJECT(tbl_node, struct pp2_cls_table_node, cls_tbl_head, list_node) {
		if (tbl_node->idx == tbl_idx) {
			*tbl = tbl_node->tbl;
			return 0;
		}
	}
	return -EFAULT;
}

static int pp2_cls_cli_table_add(void *arg, int argc, char *argv[])
{
	u32 idx = 0;
	int rc;
	int engine_type = -1;
	int key_size = 0;
	int num_fields;
	u32 proto[PP2_CLS_TBL_MAX_NUM_FIELDS];
	u32 field[PP2_CLS_TBL_MAX_NUM_FIELDS];
	char name[MVAPPS_PPIO_NAME_MAX];
	int traffic_class = -1;
	int action_type = PP2_CLS_TBL_ACT_DONE;
	char *ret_ptr;
	struct pp2_cls_table_node *tbl_node;
	struct pp2_cls_tbl_params *tbl_params;
	int i, option;
	int long_index = 0;
	struct option long_options[] = {
		{"engine_type", required_argument, 0, 'e'},
		{"key", required_argument, 0, 'k'},
		{"tc", required_argument, 0, 'q'},
		{"drop", no_argument, 0, 'd'},
		{0, 0, 0, 0}
	};

	if  (argc < 5 || argc > 16) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	/* every time starting getopt we should reset optind */
	optind = 0;
	for (i = 0; ((option = getopt_long_only(argc, argv, "", long_options, &long_index)) != -1); i++) {
		/* Get parameters */
		switch (option) {
		case 'e':
			if (!strcmp(optarg, "exact_match")) {
				engine_type = 0;
			} else if (!strcmp(optarg, "maskable")) {
				engine_type = 1;
			} else {
				printf("parsing fail, wrong input for engine_type\n");
				return -EINVAL;
			}
			break;
		case 'd':
			action_type = PP2_CLS_TBL_ACT_DROP;
			break;
		case 'q':
			traffic_class = strtoul(optarg, &ret_ptr, 0);
			if ((optarg == ret_ptr) || (traffic_class < 0) ||
			    (traffic_class >= CLS_APP_MAX_NUM_TCS_PER_PORT)) {
				printf("parsing fail, wrong input for --tc\n");
				return -EINVAL;
			}
			break;
		case 'k':
			rc = pp2_cls_convert_string_to_proto_and_field(&proto[idx], &field[idx]);
			if (rc < 0) {
				printf("parsing fail, wrong input for --key\n");
				return -EINVAL;
			}
			key_size += rc;
			idx++;
			break;
		default:
			printf("parsing fail, wrong input\n");
			return -EINVAL;
		}
	}
	/* check if all the fields are initialized */
	if (engine_type < 0) {
		printf("parsing fail, invalid --engine_type\n");
		return -EINVAL;
	}

	if (traffic_class < 0) {
		printf("parsing fail, invalid --tc\n");
		return -EINVAL;
	}

	if (key_size > CLS_APP_KEY_SIZE_MAX) {
		pr_err("key size out of range = %d\n", key_size);
		return -EINVAL;
	}

	num_fields = idx;
	if (num_fields > PP2_CLS_TBL_MAX_NUM_FIELDS) {
		pr_err("parsing fail, wrong input for --num_fields\n");
		return -EINVAL;
	}

	pr_debug("num_fields = %d, key_size = %d\n", num_fields, key_size);

	tbl_node = malloc(sizeof(*tbl_node));
	if (!tbl_node) {
		pr_err("%s no mem for new table!\n", __func__);
		return -ENOMEM;
	}
	memset(tbl_node, 0, sizeof(*tbl_node));

	/* add table to db */
	list_add_to_tail(&tbl_node->list_node, &cls_flow_tbl_head);

	tbl_node->idx = pp2_cls_table_next_index_get(&cls_flow_tbl_head);

	tbl_params = &tbl_node->tbl_params;
	tbl_params->type = engine_type;
	tbl_params->max_num_rules = CLS_APP_MAX_NUM_OF_RULES;
	tbl_params->key.key_size = key_size;
	tbl_params->key.num_fields = num_fields;
	for (idx = 0; idx < tbl_params->key.num_fields; idx++) {
		tbl_params->key.proto_field[idx].proto = proto[idx];
		tbl_params->key.proto_field[idx].field.eth = field[idx];
	}

	tbl_params->default_act.cos = malloc(sizeof(struct pp2_cls_cos_desc));
	if (!tbl_params->default_act.cos) {
		free(tbl_node);
		pr_err("%s(%d) no mem for pp2_cls_cos_desc!\n", __func__, __LINE__);
		return -ENOMEM;
	}

	tbl_params->default_act.type = action_type;
	tbl_params->default_act.cos->ppio = garg.ports_desc[0].ppio;
	tbl_params->default_act.cos->tc = traffic_class;

	memset(name, 0, sizeof(name));
	snprintf(name, sizeof(name), "ppio-%d:%d",
		 garg.ports_desc[0].pp_id, garg.ports_desc[0].ppio_id);

	strcpy(&tbl_node->ppio_name[0], name);

	if (!pp2_cls_tbl_init(tbl_params, &tbl_node->tbl))
		printf("OK\n");
	else
		printf("FAIL\n");

	return 0;
}

static int pp2_cls_table_remove(u32 tbl_idx, struct list *cls_tbl_head)
{
	struct pp2_cls_table_node *tbl_node;

	LIST_FOR_EACH_OBJECT(tbl_node, struct pp2_cls_table_node, cls_tbl_head, list_node) {
		pr_debug("tbl_node->idx %d, tbl_idx %d\n", tbl_node->idx, tbl_idx);

		if (tbl_node->idx == tbl_idx) {
			struct pp2_cls_tbl_params *tbl_ptr = &tbl_node->tbl_params;

			pp2_cls_tbl_deinit(tbl_node->tbl);
			list_del(&tbl_node->list_node);
			free(tbl_ptr->default_act.cos);
			free(tbl_node);
		}
	}
	return 0;
}

static int pp2_cls_cli_table_remove(void *arg, int argc, char *argv[])
{
	int tbl_idx = -1;
	char *ret_ptr;
	int option = 0;
	int long_index = 0;
	int rc;
	struct option long_options[] = {
		{"table_index", required_argument, 0, 't'},
		{0, 0, 0, 0}
	};

	if (argc != 3) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	/* every time starting getopt we should reset optind */
	optind = 0;
	/* Get parameters */
	while ((option = getopt_long_only(argc, argv, "", long_options, &long_index)) != -1) {
		switch (option) {
		case 't':
			tbl_idx = strtoul(optarg, &ret_ptr, 0);
			if ((optarg == ret_ptr) || (tbl_idx < 0) || (tbl_idx >= list_num_objs(&cls_flow_tbl_head))) {
				printf("parsing fail, wrong input for --table_index\n");
				return -EINVAL;
			}
			break;
		default:
			printf("parsing fail, wrong input, line = %d\n", __LINE__);
			return -EINVAL;
		}
	}

	/* check if all the fields are initialized */
	if (tbl_idx < 0) {
		printf("parsing fail, invalid --table_index\n");
		return -EINVAL;
	}

	rc = pp2_cls_table_remove(tbl_idx, &cls_flow_tbl_head);
	if (!rc)
		printf("OK\n");
	else
		printf("error removing table\n");
	return 0;
}

static int pp2_cls_cli_qos_table_add(void *arg, int argc, char *argv[])
{
	int type = -1;
	u32 pcp_dflt = 0;
	int pcp_idx = -1;
	int pcp_val = -1;
	u32 dscp_dflt = 0;
	int dscp_idx = -1;
	int dscp_val = -1;
	int pcp_map[MV_VLAN_PRIO_NUM];
	int dscp_map[MV_DSCP_NUM];
	char *ret_ptr;
	struct pp2_cls_table_node *tbl_node;
	struct pp2_cls_qos_tbl_params *qos_tbl_params;
	int i, option;
	int long_index = 0;
	struct option long_options[] = {
		{"type", required_argument, 0, 't'},
		{"pcp_dflt", required_argument, 0, 'p'},
		{"pcp_idx", required_argument, 0, 'i'},
		{"pcp_val", required_argument, 0, 'v'},
		{"dscp_dflt", required_argument, 0, 'q'},
		{"dscp_idx", required_argument, 0, 'j'},
		{"dscp_val", required_argument, 0, 'w'},
		{0, 0, 0, 0}
	};

	if (argc < 5 || argc > 145) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	/* fill pcp_table with invalid values*/
	for (i = 0; i < MV_VLAN_PRIO_NUM; i++)
		pcp_map[i] = -1;

	/* fill pcp_table with default values*/
	for (i = 0; i < MV_DSCP_NUM; i++)
		dscp_map[i] = -1;

	/* every time starting getopt we should reset optind */
	optind = 0;
	while ((option = getopt_long_only(argc, argv, "", long_options, &long_index)) != -1) {
		/* Get parameters */
		switch (option) {
		case 't':
			if (!strcmp(optarg, "none")) {
				type = PP2_CLS_QOS_TBL_NONE;
			} else if (!strcmp(optarg, "vlan")) {
				type = PP2_CLS_QOS_TBL_VLAN_PRI;
			} else if (!strcmp(optarg, "ip")) {
				type = PP2_CLS_QOS_TBL_IP_PRI;
			} else if (!strcmp(optarg, "vlan_ip")) {
				type = PP2_CLS_QOS_TBL_VLAN_IP_PRI;
			} else if (!strcmp(optarg, "ip_vlan")) {
				type = PP2_CLS_QOS_TBL_IP_VLAN_PRI;
			} else {
				printf("parsing fail, wrong input for qos type\n");
				return -EINVAL;
			}
			break;
		case 'p':
			pcp_dflt = strtoul(optarg, &ret_ptr, 0);
			if ((optarg == ret_ptr) || (pcp_dflt < 0) ||
			    (pcp_dflt > CLS_APP_MAX_NUM_TCS_PER_PORT)) {
				printf("parsing fail, wrong input for --pcp_dflt\n");
				return -EINVAL;
			}
			break;
		case 'i':
			pcp_idx = strtoul(optarg, &ret_ptr, 0);
			if ((optarg == ret_ptr) || (pcp_idx < 0) ||
			    (pcp_idx >= MV_VLAN_PRIO_NUM)) {
				printf("parsing fail, wrong input for pcp_idx\n");
				return -EINVAL;
			}
			option = getopt_long_only(argc, argv, "", long_options, &long_index);
			if (option == 'v') {
				pcp_val = strtoul(optarg, &ret_ptr, 0);
				if ((optarg == ret_ptr) || (pcp_val < 0) ||
				    (pcp_val >= CLS_APP_MAX_NUM_TCS_PER_PORT)) {
					printf("parsing fail, wrong input for pcp_val\n");
					return -EINVAL;
				}
			} else {
				printf("parsing fail, wrong input, line = %d\n", __LINE__);
				return -EINVAL;
			}
			pcp_map[pcp_idx] = pcp_val;
			break;
		case 'q':
			dscp_dflt = strtoul(optarg, &ret_ptr, 0);
			if ((optarg == ret_ptr) || (dscp_dflt < 0) ||
			    (dscp_dflt > CLS_APP_MAX_NUM_TCS_PER_PORT)) {
				printf("parsing fail, wrong input for --pcp_dflt\n");
				return -EINVAL;
			}
			break;
		case 'j':
			dscp_idx = strtoul(optarg, &ret_ptr, 0);
			if ((optarg == ret_ptr) || (dscp_idx < 0) ||
			    (dscp_idx >= MV_DSCP_NUM)) {
				printf("parsing fail, wrong input for dscp_idx\n");
				return -EINVAL;
			}
			option = getopt_long_only(argc, argv, "", long_options, &long_index);
			if (option == 'w') {
				dscp_val = strtoul(optarg, &ret_ptr, 0);
				if ((optarg == ret_ptr) || (dscp_val < 0) ||
				    (dscp_val >= CLS_APP_MAX_NUM_TCS_PER_PORT)) {
					printf("parsing fail, wrong input for dscp_val\n");
					return -EINVAL;
				}
			} else {
				printf("parsing fail, wrong input, line = %d\n", __LINE__);
				return -EINVAL;
			}
			dscp_map[dscp_idx] = dscp_val;
			break;
		default:
			printf("parsing fail, wrong input\n");
			return -EINVAL;
		}
	}

	/* check if all the fields are initialized */
	if (type < 0) {
		printf("parsing fail, invalid --type\n");
		return -EINVAL;
	}

	tbl_node = malloc(sizeof(*tbl_node));
	if (!tbl_node) {
		pr_err("%s no mem for new table!\n", __func__);
		return -ENOMEM;
	}
	memset(tbl_node, 0, sizeof(*tbl_node));

	/* add table to db */
	list_add_to_tail(&tbl_node->list_node, &cls_qos_tbl_head);

	tbl_node->idx = pp2_cls_table_next_index_get(&cls_qos_tbl_head);

	qos_tbl_params = &tbl_node->qos_tbl_params;
	qos_tbl_params->type = type;

	/* fill pcp_table wit default values*/
	for (i = 0; i < MV_VLAN_PRIO_NUM; i++) {
		if (pcp_map[i] == -1)
			qos_tbl_params->pcp_cos_map[i].tc = pcp_dflt;
		else
			qos_tbl_params->pcp_cos_map[i].tc = pcp_map[i];
		qos_tbl_params->pcp_cos_map[i].ppio = garg.ports_desc[0].ppio;
		pr_debug("pcp[%d] %d\n", i, qos_tbl_params->pcp_cos_map[i].tc);
	}

	/* fill pcp_table wit default values*/
	for (i = 0; i < MV_DSCP_NUM; i++) {
		if (dscp_map[i] == -1)
			qos_tbl_params->dscp_cos_map[i].tc = dscp_dflt;
		else
			qos_tbl_params->dscp_cos_map[i].tc = dscp_map[i];
		qos_tbl_params->dscp_cos_map[i].ppio = garg.ports_desc[0].ppio;
		pr_debug("dscp[%d] %d\n", i, qos_tbl_params->dscp_cos_map[i].tc);
	}

	if (!pp2_cls_qos_tbl_init(qos_tbl_params, &tbl_node->tbl))
		printf("OK\n");
	else
		printf("FAIL\n");

	return 0;
}

static int pp2_cls_qos_table_remove(u32 tbl_idx)
{
	struct pp2_cls_table_node *tbl_node;

	LIST_FOR_EACH_OBJECT(tbl_node, struct pp2_cls_table_node, &cls_qos_tbl_head, list_node) {
		pr_debug("tbl_node->idx %d, tbl_idx %d\n", tbl_node->idx, tbl_idx);

		if (tbl_node->idx == tbl_idx) {
			pp2_cls_qos_tbl_deinit(tbl_node->tbl);
			list_del(&tbl_node->list_node);
			free(tbl_node);
		}
	}
	return 0;
}

static int pp2_cls_cli_qos_table_remove(void *arg, int argc, char *argv[])
{
	int tbl_idx = -1;
	char *ret_ptr;
	int option = 0;
	int long_index = 0;
	int rc;
	struct option long_options[] = {
		{"qos_table_index", required_argument, 0, 't'},
		{0, 0, 0, 0}
	};

	if (argc != 3) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	/* every time starting getopt we should reset optind */
	optind = 0;
	/* Get parameters */
	while ((option = getopt_long_only(argc, argv, "", long_options, &long_index)) != -1) {
		switch (option) {
		case 't':
			tbl_idx = strtoul(optarg, &ret_ptr, 0);
			if ((optarg == ret_ptr) || (tbl_idx < 0) || (tbl_idx >= list_num_objs(&cls_qos_tbl_head))) {
				printf("parsing fail, wrong input for --table_index\n");
				return -EINVAL;
			}
			break;
		default:
			printf("parsing fail, wrong input, line = %d\n", __LINE__);
			return -EINVAL;
		}
	}

	/* check if all the fields are initialized */
	if (tbl_idx < 0) {
		printf("parsing fail, invalid --table_index\n");
		return -EINVAL;
	}

	rc = pp2_cls_qos_table_remove(tbl_idx);
	if (!rc)
		printf("OK\n");
	else
		printf("error removing table\n");
	return 0;
}

static int pp2_cls_cli_cls_rule_key(void *arg, int argc, char *argv[])
{
	u32 idx = 0;
	int tbl_idx = -1;
	int traffic_class = -1;
	int action_type = PP2_CLS_TBL_ACT_DONE;
	int rc;
	u32 num_fields;
	struct pp2_cls_tbl *tbl;
	struct pp2_cls_tbl_rule *rule;
	struct pp2_cls_tbl_action *action;
	u32 key_size[PP2_CLS_TBL_MAX_NUM_FIELDS];
	u8 key[PP2_CLS_TBL_MAX_NUM_FIELDS][CLS_APP_STR_SIZE_MAX] = {0};
	u8 mask[PP2_CLS_TBL_MAX_NUM_FIELDS][CLS_APP_STR_SIZE_MAX] = {0};
	char *ret_ptr;
	int option = 0;
	int long_index = 0;
	u32 cmd = 0;
	struct option long_options[] = {
		{"add", no_argument, 0, 'a'},
		{"modify", no_argument, 0, 'o'},
		{"remove", no_argument, 0, 'r'},
		{"size", required_argument, 0, 's'},
		{"key", required_argument, 0, 'k'},
		{"mask", required_argument, 0, 'm'},
		{"table_index", required_argument, 0, 't'},
		{"drop", no_argument, 0, 'd'},
		{"tc", required_argument, 0, 'q'},
		{0, 0, 0, 0}
	};

	if (argc < 3 || argc > CLS_APP_KEY_SIZE_MAX) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	/* every time starting getopt we should reset optind */
	optind = 0;
	/* Get parameters */
	while ((option = getopt_long_only(argc, argv, "", long_options, &long_index)) != -1) {
		switch (option) {
		case 'a':
			cmd = 1;
			break;
		case 'o':
			cmd = 2;
			break;
		case 'r':
			cmd = 3;
			break;
		case 't':
			tbl_idx = strtoul(optarg, &ret_ptr, 0);
			if ((optarg == ret_ptr) || (tbl_idx < 0)) {
				printf("parsing fail, wrong input for --table_index\n");
				return -EINVAL;
			}
			break;
		case 'd':
			action_type = PP2_CLS_TBL_ACT_DROP;
			break;
		case 'q':
			traffic_class = strtoul(optarg, &ret_ptr, 0);
			if ((optarg == ret_ptr) || (traffic_class < 0) ||
			    (traffic_class >= CLS_APP_MAX_NUM_TCS_PER_PORT)) {
				printf("parsing fail, wrong input for --tc\n");
				return -EINVAL;
			}
			break;
		case 's':
			key_size[idx] = strtoul(optarg, &ret_ptr, 0);
			if ((argv[2 + (idx * 3)] == ret_ptr) || (key_size[idx] < 0) ||
			    (key_size[idx] > CLS_APP_KEY_SIZE_MAX)) {
				printf("parsing fail, wrong input for ---size\n");
				return -EINVAL;
			}
			option = getopt_long_only(argc, argv, "", long_options, &long_index);
			if (option == 'k') {
				rc = sscanf(optarg, "%s", &key[idx][0]);
				if (rc <= 0) {
					printf("parsing fail, wrong input for --key\n");
					return -EINVAL;
				}
				option = getopt_long_only(argc, argv, "", long_options, &long_index);
				if (option == 'm') {
					rc = sscanf(optarg, "%s", &mask[idx][0]);
					if (rc <= 0) {
						printf("parsing fail, wrong input for --mask\n");
						return -EINVAL;
					}
				} else {
					printf("parsing fail, wrong input, line = %d\n", __LINE__);
					return -EINVAL;
				}
			} else {
				printf("parsing fail, wrong input, line = %d\n", __LINE__);
				return -EINVAL;
			}
			idx++;
			break;
		default:
			printf("parsing fail, wrong input, line = %d\n", __LINE__);
			return -EINVAL;
		}
	}

	/* check if all the fields are initialized */
	if (tbl_idx < 0) {
		printf("parsing fail, invalid --table_index\n");
		return -EINVAL;
	}
	if (traffic_class < 0) {
		printf("parsing fail, invalid --tc\n");
		return -EINVAL;
	}
	num_fields = idx;
	if (num_fields > PP2_CLS_TBL_MAX_NUM_FIELDS) {
		printf("num_fields = %d is too long, max num_fields is %d\n", num_fields, PP2_CLS_TBL_MAX_NUM_FIELDS);
		return -EINVAL;
	}

	if (cmd < 1 || cmd > 3) {
		printf("command not recognized\n");
		return -EINVAL;
	}

	rc = pp2_cls_table_get(tbl_idx, &tbl, &cls_flow_tbl_head);
	if (rc) {
		printf("table not found for index %d\n", tbl_idx);
		return -EINVAL;
	}

	rule = malloc(sizeof(*rule));
	if (!rule)
		goto rule_add_fail;

	rule->num_fields = num_fields;
	for (idx = 0; idx < num_fields; idx++) {
		rule->fields[idx].size = key_size[idx];
		rule->fields[idx].key = &key[idx][0];
		rule->fields[idx].mask = &mask[idx][0];
	}

	if (cmd == 3) {
		rc = pp2_cls_tbl_remove_rule(tbl, rule);
	} else {
		action = malloc(sizeof(*action));
		if (!action)
			goto rule_add_fail1;

		action->cos = malloc(sizeof(*action->cos));
		if (!action->cos)
			goto rule_add_fail2;

		action->type = action_type;
		action->cos->tc = traffic_class;
		action->cos->ppio = garg.ports_desc[0].ppio;

		if (cmd == 1)
			rc = pp2_cls_tbl_add_rule(tbl, rule, action);
		else
			rc = pp2_cls_tbl_modify_rule(tbl, rule, action);

		free(action->cos);
		free(action);
	}

	if (!rc)
		printf("OK\n");
	else
		printf("FAIL: unable to perform requested command to table index: %d\n", tbl_idx);
	free(rule);
	return 0;

rule_add_fail2:
	free(action);
rule_add_fail1:
	free(rule);
rule_add_fail:
	pr_err("%s no mem for new rule!\n", __func__);
	return -ENOMEM;
}

static int pp2_cls_cli_cls_table_dump(void *arg, int argc, char *argv[])
{
	u32 i, j;
	struct pp2_cls_table_node *tbl_node;
	u32 num_tables = list_num_objs(&cls_flow_tbl_head);

	printf("total indexes: %d\n", num_tables);
	if (num_tables > 0) {
		printf("|                  |     default_action   |               key\n");
		printf("|idx|type|num_rules|    port  |type|tc_num|key_size|num_fields|");

		for (i = 0; i < 5; i++)
			printf("proto,field|");
		printf("\n");
		app_print_horizontal_line(123, "=");

		LIST_FOR_EACH_OBJECT(tbl_node, struct pp2_cls_table_node, &cls_flow_tbl_head, list_node) {
			struct pp2_cls_tbl_params *tbl_ptr = &tbl_node->tbl_params;

			printf("|%3d|%4d|%9d|", tbl_node->idx, tbl_ptr->type,
			       tbl_ptr->max_num_rules);
			printf("%10s|%4d|%6d|", tbl_node->ppio_name, tbl_ptr->default_act.type,
			       tbl_ptr->default_act.cos->tc);

			printf("%8d|%10d|", tbl_ptr->key.key_size, tbl_ptr->key.num_fields);
			for (j = 0; j < tbl_ptr->key.num_fields; j++) {
				printf("%5d,%5d|", tbl_ptr->key.proto_field[j].proto,
				       tbl_ptr->key.proto_field[j].field.eth);
			}
			printf("\n");
			app_print_horizontal_line(123, "-");
		}
	}
	printf("OK\n");

	return 0;
}

static int pp2_cls_cli_mac_addr(void *arg, int argc, char *argv[])
{
	int rc;
	int i, option, uc, mc;
	int long_index = 0;
	u8 mac[ETH_ALEN];
	struct option long_options[] = {
		{"set", required_argument, 0, 's'},
		{"get", no_argument, 0, 'g'},
		{"add", required_argument, 0, 'a'},
		{"remove", required_argument, 0, 'r'},
		{"flush", no_argument, 0, 'f'},
		{"uc", no_argument, 0, 'u'},
		{"mc", no_argument, 0, 'm'},
		{0, 0, 0, 0}
	};

	if  (argc < 2 || argc > 4) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	/* every time starting getopt we should reset optind */
	optind = 0;
	for (i = 0; ((option = getopt_long_only(argc, argv, "", long_options, &long_index)) != -1); i++) {
		/* Get parameters */
		switch (option) {
		case 's':
			rc = mv_pp2x_parse_mac_address(optarg, mac);
			if (rc) {
				printf("parsing fail, wrong input for mac_address set\n");
				return -EINVAL;
			}
			printf("mac_addr set1 %x:%x:%x:%x:%x:%x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

			rc = pp2_ppio_set_mac_addr(garg.ports_desc[0].ppio, mac);
			if (rc) {
				printf("Unable to set mac address\n");
				return -EINVAL;
			}
			break;
		case 'g':
			rc = pp2_ppio_get_mac_addr(garg.ports_desc[0].ppio, mac);
			if (rc) {
				printf("Unable to get mac address\n");
				return -EINVAL;
			}

			printf("mac_addr get %x:%x:%x:%x:%x:%x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
			break;
		case 'a':
			rc = mv_pp2x_parse_mac_address(optarg, mac);
			if (rc) {
				printf("parsing fail, wrong input for mac_address add\n");
				return -EINVAL;
			}
			printf("mac_addr add %x:%x:%x:%x:%x:%x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

			rc = pp2_ppio_add_mac_addr(garg.ports_desc[0].ppio, mac);
			if (rc) {
				printf("Unable to add mac address\n");
				return -EINVAL;
			}
			break;
		case 'r':
			rc = mv_pp2x_parse_mac_address(optarg, mac);
			if (rc) {
				printf("parsing fail, wrong input for mac_address remove\n");
				return -EINVAL;
			}
			printf("mac_addr remove %x:%x:%x:%x:%x:%x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

			rc = pp2_ppio_remove_mac_addr(garg.ports_desc[0].ppio, mac);
			if (rc) {
				printf("Unable to remove mac address\n");
				return -EINVAL;
			}
			break;
		case 'f':
			uc = 0;
			mc = 0;
			option = getopt_long_only(argc, argv, "", long_options, &long_index);
			if (option == 'u') {
				uc = 1;
				option = getopt_long_only(argc, argv, "", long_options, &long_index);
				if (option == 'm')
					mc = 1;
			} else if (option == 'm') {
				mc = 1;
				option = getopt_long_only(argc, argv, "", long_options, &long_index);
				if (option == 'u')
					uc = 1;
			} else {
				printf("uc or mc flags not selected. Command ignored\n");
				return -EINVAL;
			}
			rc = pp2_ppio_flush_mac_addrs(garg.ports_desc[0].ppio, uc, mc);
			if (rc) {
				printf("Unable to flush mac address %d, %d\n", uc, mc);
				return -EINVAL;
			}
			break;
		default:
			printf("parsing fail, wrong input\n");
			return -EINVAL;
		}
	}
	return 0;
}

static int pp2_cls_cli_promisc_mode(void *arg, int argc, char *argv[])
{
	int rc;
	int i, option;
	int long_index = 0;
	int en = -1;
	int mode = -1;
	int cmd = -1;
	struct option long_options[] = {
		{"on", no_argument, 0, 'n'},
		{"off", no_argument, 0, 'f'},
		{"get", no_argument, 0, 'g'},
		{"uc", no_argument, 0, 'u'},
		{"mc", no_argument, 0, 'm'},
		{0, 0, 0, 0}
	};

	if  (argc < 2 || argc > 3) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	/* every time starting getopt we should reset optind */
	optind = 0;
	for (i = 0; ((option = getopt_long_only(argc, argv, "", long_options, &long_index)) != -1); i++) {
		/* Get parameters */
		switch (option) {
		case 'n':
			if (cmd < 0) {
				cmd = 1;
				en = 1;
			}
			break;
		case 'f':
			if (cmd < 0) {
				cmd = 1;
				en = 0;
			}
			break;
		case 'g':
			if (cmd < 0)
				cmd = 0;
			break;
		case 'u':
			if (mode < 0)
				mode = 1;
			break;
		case 'm':
			if (mode < 0)
				mode = 0;
			break;
		default:
			printf("parsing fail, wrong input\n");
			return -EINVAL;
		}
	}

	if ((mode < 0) || (cmd < 0)) {
		printf("Wrong inputs in promiscuous mode command\n");
		return -EINVAL;
	}

	if (cmd && mode) {
		rc = pp2_ppio_set_uc_promisc(garg.ports_desc[0].ppio, en);
		if (rc) {
			printf("Unable to enable unicast promiscuous mode\n");
			return -rc;
		}
	} else if (cmd && !mode) {
		rc = pp2_ppio_set_mc_promisc(garg.ports_desc[0].ppio, en);
		if (rc) {
			printf("Unable to enable all multicast mode\n");
			return -rc;
		}
	} else if (!cmd && mode) {
		rc = pp2_ppio_get_uc_promisc(garg.ports_desc[0].ppio, &en);
		if (rc) {
			printf("Unable to get unicast promiscuous mode\n");
			return -rc;
		}
		if (en)
			printf("unicast promiscuous mode: enabled\n");
		else
			printf("unicast promiscuous mode: disabled\n");
	} else if (!cmd && !mode) {
		rc = pp2_ppio_get_mc_promisc(garg.ports_desc[0].ppio, &en);
		if (rc) {
			printf("Unable to get all multicast mode\n");
			return -rc;
		}
		if (en)
			printf("all multicast mode: enabled\n");
		else
			printf("all multicast mode: disabled\n");
	} else {
		pr_err("Option not supported\n");
		return -EFAULT;
	}
	return 0;
}

static int pp2_cls_cli_vlan(void *arg, int argc, char *argv[])
{
	int rc;
	char *ret_ptr;
	int i, option;
	int long_index = 0;
	u16 vlan_id;
	int cmd = -1;
	struct option long_options[] = {
		{"set", required_argument, 0, 's'},
		{"remove", required_argument, 0, 'r'},
		{"flush", no_argument, 0, 'f'},
		{0, 0, 0, 0}
	};

	if  (argc < 2 || argc > 4) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	/* every time starting getopt we should reset optind */
	optind = 0;
	for (i = 0; ((option = getopt_long_only(argc, argv, "", long_options, &long_index)) != -1); i++) {
		/* Get parameters */
		switch (option) {
		case 's':
			vlan_id = strtoul(optarg, &ret_ptr, 0);
			cmd = 0;
			break;
		case 'r':
			vlan_id = strtoul(optarg, &ret_ptr, 0);
			cmd = 1;
			break;
		case 'f':
			cmd = 2;
			break;
		}
	}

	if (cmd < 0)
		printf("parsing fail, wrong input\n");

	if (cmd == 0 && vlan_id >= 0 && vlan_id <= 4095) {
		rc = pp2_ppio_add_vlan(garg.ports_desc[0].ppio, vlan_id);
		if (rc) {
			printf("Unable to add vlan id %d\n", vlan_id);
			return -EINVAL;
		}
	} else if (cmd == 1 && vlan_id >= 0 && vlan_id <= 4095) {
		rc = pp2_ppio_remove_vlan(garg.ports_desc[0].ppio, vlan_id);
		if (rc) {
			printf("Unable to remove vlan id %d\n", vlan_id);
			return -EINVAL;
		}
	} else if (cmd == 2) {
		rc = pp2_ppio_flush_vlan(garg.ports_desc[0].ppio);
		if (rc) {
			printf("Unable to flush vlans\n");
			return -EINVAL;
		}
	} else {
		printf("parsing fail, wrong input\n");
		return -EINVAL;
	}
	return 0;
}

static int pp2_cls_cli_ppio_tag_mode(struct glob_arg *garg, int argc, char *argv[])
{
	int i, option;
	int long_index = 0;
	enum pp2_ppio_eth_start_hdr eth_hdr = -1;
	struct pp2_ppio_params *port_params = &garg->ports_desc[0].port_params;
	struct option long_options[] = {
		{"none", no_argument, 0, 'n'},
		{"dsa", no_argument, 0, 'd'},
		{"extended_dsa", no_argument, 0, 'e'},
		{0, 0, 0, 0}
	};
	if (argc != 2) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	/* every time starting getopt we should reset optind */
	optind = 0;
	for (i = 0; ((option = getopt_long_only(argc, argv, "", long_options, &long_index)) != -1); i++) {
		/* Get parameters */
		switch (option) {
		case 'n':
			eth_hdr = PP2_PPIO_HDR_ETH;
			break;
		case 'd':
			eth_hdr = PP2_PPIO_HDR_ETH_DSA;
			break;
		case 'e':
			eth_hdr = PP2_PPIO_HDR_ETH_EXT_DSA;
			break;
		default:
			printf("parsing fail, wrong input\n");
			return -EINVAL;
		}
	}

	if (eth_hdr < 0 || eth_hdr >= PP2_PPIO_HDR_OUT_OF_RANGE) {
		printf("parsing fail, wrong input\n");
		return -EFAULT;
	}

	port_params->eth_start_hdr = eth_hdr;

	return 0;
}

static int pp2_cls_logical_port_params(struct glob_arg *garg, int argc, char *argv[])
{
	char *ret_ptr;
	int i, j, option;
	int long_index = 0;
	u32 r_idx = 0;
	struct pp2_ppio_params *port_params = &garg->ports_desc[0].port_params;
	struct pp2_ppio_log_port_params params;
	struct pp2_ppio_log_port_rule_params *rules_params;
	struct pp2_ppio_log_port_params *log_port_params =
					&port_params->specific_type_params.log_port_params;
	struct option long_options[] = {
		{"target", required_argument, 0, 'c'},
		{"num_proto_rule_sets", required_argument, 0, 'n'},
		{"num_rules", required_argument, 0, 'm'},
		{"rule_type", required_argument, 0, 'a'},
		{"proto", required_argument, 0, 't'},
		{"proto_val", required_argument, 0, 'l'},
		{"special_proto", required_argument, 0, 's'},
		{"special_fields", required_argument, 0, 'f'},
		{"field_val", required_argument, 0, 'v'},
		{0, 0, 0, 0}
	};

	if  (argc < 2 || argc > (6 + 2 * PP2_MAX_PROTO_SUPPORTED + 2 * PP2_MAX_FIELDS_SUPPORTED)) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	memset(&params, 0, sizeof(struct pp2_ppio_log_port_params));
	/* every time starting getopt we should reset optind */
	optind = 0;
	for (i = 0; ((option = getopt_long_only(argc, argv, "", long_options, &long_index)) != -1); i++) {
		/* Get parameters */
		switch (option) {
		case 'c':
			params.proto_based_target.target = strtoul(optarg, &ret_ptr, 0);
			if (params.proto_based_target.target != 0 && params.proto_based_target.target != 1) {
				printf("parsing fail, wrong input for --target\n");
				return -EINVAL;
			}
			break;
		case 'n':
			params.proto_based_target.num_proto_rule_sets = strtoul(optarg, &ret_ptr, 0);
			if (params.proto_based_target.num_proto_rule_sets <= 0 &&
			    params.proto_based_target.num_proto_rule_sets > PP2_MAX_PROTO_SUPPORTED) {
				printf("parsing fail, wrong input for --num_proto_rule_sets\n");
				return -EINVAL;
			}
			break;
		case 'm':
			params.proto_based_target.rule_sets[r_idx].num_rules = strtoul(optarg, &ret_ptr, 0);
			if (params.proto_based_target.rule_sets[r_idx].num_rules <= 0 &&
			    params.proto_based_target.rule_sets[r_idx].num_rules > PP2_MAX_PROTO_SUPPORTED) {
				printf("parsing fail, wrong input for --num_rules\n");
				return -EINVAL;
			}
			for (j = 0; j < params.proto_based_target.rule_sets[r_idx].num_rules &&
			     ((option = getopt_long_only(argc, argv, "", long_options, &long_index)) != -1); j++) {
				if (option != 'a') {
					printf("parsing fail, wrong input\n");
					return -EINVAL;
				}
				rules_params = &params.proto_based_target.rule_sets[r_idx].rules[j];
				rules_params->rule_type = strtoul(optarg, &ret_ptr, 0);
				if (rules_params->rule_type != 0 &&
				    rules_params->rule_type != 1) {
					printf("parsing fail, wrong input for --rule_type\n");
					return -EINVAL;
				}
				if (!rules_params->rule_type) {
					option = getopt_long_only(argc, argv, "", long_options, &long_index);
					if (option != 't') {
						printf("parsing fail, missing input --proto\n");
						return -EINVAL;
					}
					rules_params->u.proto_params.proto =
									strtoul(optarg, &ret_ptr, 0);
					if (rules_params->u.proto_params.proto < 0 ||
					    rules_params->u.proto_params.proto
					    >= MV_NET_PROTO_LAST) {
						printf("parsing fail, wrong input for --proto\n");
						return -EINVAL;
					}
					option = getopt_long_only(argc, argv, "", long_options, &long_index);
					if (option != 'l') {
						printf("parsing fail, missing input --proto_val\n");
						return -EINVAL;
					}
					rules_params->u.proto_params.val =
									strtoul(optarg, &ret_ptr, 0);
					if (rules_params->u.proto_params.val != 0 &&
					    rules_params->u.proto_params.val != 1) {
						printf("parsing fail, wrong input for --proto_val\n");
						return -EINVAL;
					}
				} else {
					option = getopt_long_only(argc, argv, "", long_options, &long_index);
					if (option != 's') {
						printf("parsing fail, missing --special_proto\n");
						return -EINVAL;
					}
					rules_params->u.proto_field_params.proto_field.proto
											= strtoul(optarg, &ret_ptr, 0);
					if (rules_params->u.proto_field_params.proto_field.proto
					    < 0 ||
					    rules_params->u.proto_field_params.proto_field.proto
					    >= PP2_MAX_PROTO_SUPPORTED) {
						printf("parsing fail, wrong input for --special_proto\n");
						return -EINVAL;
					}
					option = getopt_long_only(argc, argv, "", long_options, &long_index);
					if (option != 'f') {
						printf("parsing fail, missing --special_fields\n");
						return -EINVAL;
					}
					rules_params->u.proto_field_params.proto_field.field.eth =
											strtoul(optarg, &ret_ptr, 0);
					if (rules_params->u.proto_field_params.proto_field.field.eth
					    < 0 ||
					    rules_params->u.proto_field_params.proto_field.field.eth
					    >= PP2_MAX_FIELDS_SUPPORTED) {
						printf("parsing fail, wrong input for --special_fields\n");
						return -EINVAL;
					}
					option = getopt_long_only(argc, argv, "", long_options, &long_index);
					if (option != 'v') {
						printf("parsing fail, missing --field_val\n");
						return -EINVAL;
					}
					rules_params->u.proto_field_params.val =
									strtoul(optarg, &ret_ptr, 0);
					if (rules_params->u.proto_field_params.val < 0) {
						printf("parsing fail, wrong input for --field_val\n");
						return -EINVAL;
					}
				}
			}
			r_idx++;
			break;
		default:
			printf("(%d) parsing fail, wrong input\n", __LINE__);
			return -EINVAL;
		}
	}

	log_port_params->proto_based_target.target = params.proto_based_target.target;
	log_port_params->proto_based_target.num_proto_rule_sets =  params.proto_based_target.num_proto_rule_sets;
	for (i = 0; i < log_port_params->proto_based_target.num_proto_rule_sets; i++) {
		log_port_params->proto_based_target.rule_sets[i].num_rules =
					params.proto_based_target.rule_sets[i].num_rules;
		for (j = 0; j < params.proto_based_target.rule_sets[i].num_rules; j++)
			log_port_params->proto_based_target.rule_sets[i].rules[j] =
					params.proto_based_target.rule_sets[i].rules[j];
	}

	return 0;
}

#ifdef CLS_APP_PKT_ECHO_SUPPORT
#ifdef CLS_APP_USE_APP_PREFETCH
static inline void prefetch(const void *ptr)
{
	asm volatile("prfm pldl1keep, %a0\n" : : "p" (ptr));
}
#endif /* CLS_APP_USE_APP_PREFETCH */

#endif /* CLS_APP_PKT_ECHO_SUPPORT */

static inline int loop_sw_recycle(struct local_arg	*larg,
				  u8			 rx_ppio_id,
				  u8			 tx_ppio_id,
				  u8			 bpool_id,
				  u8			 tc,
				  u8			 qid,
				  u16			 num)
{
	struct pp2_bpool *bpool;
	struct tx_shadow_q *shadow_q;
	struct pp2_buff_inf *binf;
	struct pp2_ppio_desc descs[CLS_APP_MAX_BURST_SIZE];
	int err;
	u16 i;
#ifdef CLS_APP_PKT_ECHO_SUPPORT
	int prefetch_shift = CLS_APP_PREFETCH_SHIFT;
#endif /* CLS_APP_PKT_ECHO_SUPPORT */
	bpool = larg->pools_desc[larg->ports_desc[rx_ppio_id].pp_id][bpool_id].pool;
	shadow_q = &larg->ports_desc[tx_ppio_id].shadow_qs[tc];

	err = pp2_ppio_recv(larg->ports_desc[rx_ppio_id].ppio, tc, qid, descs, &num);

	for (i = 0; i < num; i++) {
		char		*buff = (char *)(uintptr_t)pp2_ppio_inq_desc_get_cookie(&descs[i]);
		dma_addr_t	 pa = pp2_ppio_inq_desc_get_phys_addr(&descs[i]);

		u16 len = pp2_ppio_inq_desc_get_pkt_len(&descs[i]) - PP2_MH_SIZE;

#ifdef CLS_APP_PKT_ECHO_SUPPORT
		if (likely(larg->echo)) {
			char *tmp_buff;
#ifdef CLS_APP_USE_APP_PREFETCH
			if (num - i > prefetch_shift) {
				tmp_buff = (char *)(uintptr_t)pp2_ppio_inq_desc_get_cookie(&descs[i + prefetch_shift]);
				tmp_buff += MVAPPS_PKT_EFEC_OFFS;
				pr_debug("tmp_buff_before(%p)\n", tmp_buff);
				tmp_buff = (char *)(((uintptr_t)tmp_buff) | sys_dma_high_addr);
				pr_debug("tmp_buff_after(%p)\n", tmp_buff);
				prefetch(tmp_buff);
			}
#endif /* CLS_APP_USE_APP_PREFETCH */
			tmp_buff = (char *)(((uintptr_t)(buff)) | sys_dma_high_addr);
			pr_debug("buff2(%p)\n", tmp_buff);
			tmp_buff += MVAPPS_PKT_EFEC_OFFS;
			swap_l2(tmp_buff);
			swap_l3(tmp_buff);
		}
#endif /* CLS_APP_PKT_ECHO_SUPPORT */
		pp2_ppio_outq_desc_reset(&descs[i]);
		pp2_ppio_outq_desc_set_phys_addr(&descs[i], pa);
		pp2_ppio_outq_desc_set_pkt_offset(&descs[i], MVAPPS_PKT_EFEC_OFFS);
		pp2_ppio_outq_desc_set_pkt_len(&descs[i], len);
		shadow_q->ents[shadow_q->write_ind].buff_ptr.cookie = (uintptr_t)buff;
		shadow_q->ents[shadow_q->write_ind].buff_ptr.addr = pa;
		pr_debug("buff_ptr.cookie(0x%lx)\n", (u64)shadow_q->ents[shadow_q->write_ind].buff_ptr.cookie);
		shadow_q->write_ind++;
		if (shadow_q->write_ind == CLS_APP_TX_Q_SIZE)
			shadow_q->write_ind = 0;
	}

	if (num) {
		err = pp2_ppio_send(larg->ports_desc[tx_ppio_id].ppio, larg->hif, tc, descs, &num);
		if (err) {
			pr_err("pp2_ppio_send\n");
			return err;
		}
	}

	pp2_ppio_get_num_outq_done(larg->ports_desc[tx_ppio_id].ppio, larg->hif, tc, &num);
	for (i = 0; i < num; i++) {
		binf = &shadow_q->ents[shadow_q->read_ind].buff_ptr;
		if (unlikely(!binf->cookie || !binf->addr)) {
			pr_err("Shadow memory @%d: cookie(%lx), pa(%lx)!\n",
			       shadow_q->read_ind, (u64)binf->cookie, (u64)binf->addr);
			continue;
		}
		pp2_bpool_put_buff(larg->hif,
				   bpool,
				   binf);
		shadow_q->read_ind++;
		if (shadow_q->read_ind == CLS_APP_TX_Q_SIZE)
			shadow_q->read_ind = 0;
	}
	return 0;
}

static int loop_1p(struct local_arg *larg, int *running)
{
	int err;
	u16 num;
	u8 tc = 0, qid = 0;

	if (!larg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	num = CLS_APP_DFLT_BURST_SIZE;

	while (*running) {
		/* Find next queue to consume */
		do {
			qid++;
			if (qid == MVAPPS_MAX_NUM_QS_PER_TC) {
				qid = 0;
				tc++;
				if (tc == CLS_APP_MAX_NUM_TCS_PER_PORT)
					tc = 0;
			}
		} while (!(larg->qs_map & (1 << ((tc * MVAPPS_MAX_NUM_QS_PER_TC) + qid))));
		err = loop_sw_recycle(larg, 0, 0, 0, tc, qid, num);
		if (err)
			return err;
	}

	return 0;
}

static int main_loop(void *arg, int *running)
{
	struct local_arg *larg = (struct local_arg *)arg;

	if (!larg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	if (larg->echo)
		return loop_1p(larg, running);

	while (*running)
		;

	return 0;
}

static int init_all_modules(void)
{
	struct pp2_init_params *pp2_params = &garg.pp2_params;
	int			 err;
	char			 file[PP2_MAX_BUF_STR_LEN];
	int			 num_rss_tables = 0;

	pr_info("Global initializations ...\n");

	err = mv_sys_dma_mem_init(CLS_APP_DMA_MEM_SIZE);
	if (err)
		return err;

	memset(pp2_params, 0, sizeof(*pp2_params));
	pp2_params->hif_reserved_map = MVAPPS_PP2_HIFS_RSRV;
	pp2_params->bm_pool_reserved_map = MVAPPS_PP2_BPOOLS_RSRV;

	sprintf(file, "%s/%s", PP2_SYSFS_RSS_PATH, PP2_SYSFS_RSS_NUM_TABLES_FILE);
	num_rss_tables = appp_pp2_sysfs_param_get(garg.ports_desc[0].name, file);
	if (num_rss_tables < 0) {
		pr_err("Failed to read kernel RSS tables. Please check mvpp2x_sysfs.ko is loaded\n");
		return -EFAULT;
	}
	pp2_params->rss_tbl_reserved_map = (1 << num_rss_tables) - 1;

	err = pp2_init(pp2_params);
	if (err)
		return err;

	pr_info("done\n");
	return 0;
}

static int init_local_modules(struct glob_arg *garg)
{
	int				err, port_index;
	struct bpool_inf		infs[] = CLS_APP_BPOOLS_INF;
	int				i;

	pr_info("Local initializations ...\n");

	err = app_hif_init(&garg->hif, CLS_APP_HIF_Q_SIZE);
	if (err)
		return err;

	garg->num_pools = ARRAY_SIZE(infs);
	err = app_build_all_bpools(&garg->pools_desc, garg->num_pools, infs, garg->hif);
	if (err)
		return err;

	for (port_index = 0; port_index < garg->num_ports; port_index++) {
		struct port_desc *port = &garg->ports_desc[port_index];

		err = app_find_port_info(port);
		if (!err) {
			port->num_tcs	= CLS_APP_MAX_NUM_TCS_PER_PORT;
			for (i = 0; i < port->num_tcs; i++)
				port->num_inqs[i] = MVAPPS_MAX_NUM_QS_PER_TC;
			port->inq_size	= CLS_APP_RX_Q_SIZE;
			port->num_outqs	= CLS_APP_MAX_NUM_TCS_PER_PORT;
			port->outq_size	= CLS_APP_TX_Q_SIZE;
			port->first_inq = CLS_APP_FIRST_MUSDK_IN_QUEUE;
			port->hash_type = garg->hash_type;

			err = app_port_init(port, garg->num_pools, garg->pools_desc[port->pp_id], DEFAULT_MTU);
			if (err) {
				pr_err("Failed to initialize port %d (pp_id: %d)\n", port_index,
				       port->pp_id);
				return err;
			}
		} else {
			return err;
		}
	}

	INIT_LIST(&cls_flow_tbl_head);
	INIT_LIST(&cls_qos_tbl_head);

	pr_info("done\n");
	return 0;
}

static void destroy_local_modules(struct glob_arg *garg)
{
	struct pp2_cls_table_node *tbl_node;

	app_disable_all_ports(garg->ports_desc, garg->num_ports);
	app_free_all_pools(garg->pools_desc, garg->num_pools, garg->hif);
	app_deinit_all_ports(garg->ports_desc, garg->num_ports);

	if (garg->hif)
		pp2_hif_deinit(garg->hif);

	LIST_FOR_EACH_OBJECT(tbl_node, struct pp2_cls_table_node, &cls_flow_tbl_head, list_node) {
		pp2_cls_table_remove(tbl_node->idx,  &cls_flow_tbl_head);
	}
	LIST_FOR_EACH_OBJECT(tbl_node, struct pp2_cls_table_node, &cls_qos_tbl_head, list_node) {
		pp2_cls_table_remove(tbl_node->idx,  &cls_qos_tbl_head);
	}
}

static void destroy_all_modules(void)
{
	pp2_deinit();
	mv_sys_dma_mem_destroy();
}

static int register_cli_cls_api_cmds(struct glob_arg *garg)
{
	struct cli_cmd_params cmd_params;

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_tbl_init";
	cmd_params.desc		= "create a classifier table according to key and default action";
	cmd_params.format	= "--engine_type --tc --drop(optional) --key\n"
				  "\t\t\t\t--engine_type	(string) exact_match, maskable\n"
				  "\t\t\t\t--tc			(dec) 1..8\n"
				  "\t\t\t\t--drop		(no argument)optional\n"
				  "\t\t\t\t--key		(string) the following keys are defined:\n"
				  "\t\t\t\t			eth_src - ethernet, source address\n"
				  "\t\t\t\t			eth_dst - ethernet, destination address\n"
				  "\t\t\t\t			eth_type - ethernet, type\n"
				  "\t\t\t\t			vlan_prio - vlan, priority\n"
				  "\t\t\t\t			vlan_id - vlan, id\n"
				  "\t\t\t\t			vlan_tci - vlan, tci\n"
				  "\t\t\t\t			pppoe - pppoe\n"
				  "\t\t\t\t			ip - ip\n"
				  "\t\t\t\t			ip4_tos - ipv4, tos\n"
				  "\t\t\t\t			ip4_src - ipv4, souce address\n"
				  "\t\t\t\t			ip4_dst - ipv4, destination address\n"
				  "\t\t\t\t			ip4_proto - ipv4, proto\n"
				  "\t\t\t\t			ip6_tc - ipv6, tc\n"
				  "\t\t\t\t			ip6_src - ipv6, source address\n"
				  "\t\t\t\t			ip6_dst - ipv6, destination address\n"
				  "\t\t\t\t			ip6_flow - ipv6, flow\n"
				  "\t\t\t\t			ip6_next_hdr - ipv6, next header\n"
				  "\t\t\t\t			l4_src - layer4, source port\n"
				  "\t\t\t\t			l4_dst - layer4, destination port\n"
				  "\t\t\t\t			l4_csum - layer4, checksum\n"
				  "\t\t\t\t			tcp_src - tcp, source port\n"
				  "\t\t\t\t			tcp_dst - tcp, destination port\n"
				  "\t\t\t\t			tcp_csum - tcp, checksum\n"
				  "\t\t\t\t			udp_src - udp, source port\n"
				  "\t\t\t\t			udp_dst - udp, destination port\n"
				  "\t\t\t\t			udp_csum - udp, checksum\n"
				  "\t\t\t\t			icmp - icmp\n"
				  "\t\t\t\t			arp - arp\n";
	cmd_params.cmd_arg	= garg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_table_add;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_rule_key";
	cmd_params.desc		= "add/modify/remove a classifier rule key to existing table";
	cmd_params.format	= "--add    --table_index --tc --drop(optional) --size --key --mask...\n"
				  "--modify --table_index --tc --drop(optional) --size --key --mask...\n"
				  "--remove --table_index --tc --drop(optional) --size --key --mask...\n"
				  "\t\t\t\t--table_index	(dec) index to existing table\n"
				  "\t\t\t\t--tc			(dec) 1..8\n"
				  "\t\t\t\t--drop		(optional)(no argument)\n"
				  "\t\t\t\t--size		(dec) size in bytes of the key\n"
				  "\t\t\t\t--key		(dec or hex) key\n"
				  "\t\t\t\t			   i.e ipv4: 192.168.10.5\n"
				  "\t\t\t\t			   i.e ipv6: 2605:2700:0:3::4713:93e3\n"
				  "\t\t\t\t			   i.e port: 0x1234\n"
				  "\t\t\t\t			   i.e udp: 17(IPPROTO_UDP)\n"
				  "\t\t\t\t			   i.e tcp: 6(IPPROTO_TCP)\n"
				  "\t\t\t\t--mask		(hex) mask for the key (if maskable is used)\n";
	cmd_params.cmd_arg	= garg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_cls_rule_key;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_tbl_dump";
	cmd_params.desc		= "display classifier defined tables in cls_demo application";
	cmd_params.format	= "";
	cmd_params.cmd_arg	= garg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_cls_table_dump;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_tbl_deinit";
	cmd_params.desc		= "remove a specified table";
	cmd_params.format	= "--table_index (dec) index to existing table\n";
	cmd_params.cmd_arg	= garg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_table_remove;
	mvapp_register_cli_cmd(&cmd_params);

	return 0;
}

static int register_cli_cls_api_qos_cmds(struct glob_arg *garg)
{
	struct cli_cmd_params cmd_params;

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_qos_tbl_init";
	cmd_params.desc		= "create a QoS classifier table";
	cmd_params.format	= "--type --pcp_map --dscp_map\n"
				  "\t\t\t\t--type	(string) none, vlan, ip, vlan_ip, ip_vlan\n"
				  "\t\t\t\t--pcp_default	TC number - default TC for all table values\n"
				  "\t\t\t\t			except for values defined in pcp_idx and pcp_val\n"
				  "\t\t\t\t--pcp_idx		index in pcp_map table\n"
				  "\t\t\t\t--pcp_val		TC value in pcp_map table\n"
				  "\t\t\t\t			pcp_idx and pcp_val need to be set together\n"
				  "\t\t\t\t--dscp_default	TC number - default TC for all table values\n"
				  "\t\t\t\t			except for values defined in pcp_idx and pcp_val\n"
				  "\t\t\t\t--dscp_idx		index in pcp_map table\n"
				  "\t\t\t\t--dscp_val		TC value in pcp_map table\n"
				  "\t\t\t\t			pcp_idx and pcp_val need to be set together\n";
	cmd_params.cmd_arg	= garg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_qos_table_add;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_qos_tbl_deinit";
	cmd_params.desc		= "remove a specified qos table";
	cmd_params.format	= "--qos_table_index (dec) index to existing table\n";
	cmd_params.cmd_arg	= garg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_qos_table_remove;
	mvapp_register_cli_cmd(&cmd_params);

	return 0;
}

static int register_cli_filter_cmds(struct glob_arg *garg)
{
	struct cli_cmd_params cmd_params;

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "mac_addr";
	cmd_params.desc		= "set/get/add/remove/flush ppio MAC address";
	cmd_params.format	= "--set <xx:xx:xx:xx:xx:xx>\n"
				  "\t\t\t\t\t\t--get\n"
				  "\t\t\t\t\t\t--add <xx:xx:xx:xx:xx:xx>\n"
				  "\t\t\t\t\t\t--remove <xx:xx:xx:xx:xx:xx>\n"
				  "\t\t\t\t\t\t--flush --uc --mc\n";
	cmd_params.cmd_arg	= garg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_mac_addr;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "promisc";
	cmd_params.desc		= "set/get ppio unicast/multicast promiscuous mode";
	cmd_params.format	= "--<uc/mc> --<on/off/get>\n";
	cmd_params.cmd_arg	= garg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_promisc_mode;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "vlan";
	cmd_params.desc		= "set/remove/flush ppio vlan filter";
	cmd_params.format	= "--set <vlan_id>\n"
				  "\t\t\t\t\t\t--remove <vlan_id>\n"
				  "\t\t\t\t\t\t--flush\n";
	cmd_params.cmd_arg	= garg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_vlan;
	mvapp_register_cli_cmd(&cmd_params);

	return 0;
}

static int register_cli_cmds(struct glob_arg *garg)
{
	struct pp2_ppio *ppio = garg->ports_desc[0].ppio;

	if (!garg->cli)
		return -EFAULT;

	register_cli_cls_api_cmds(garg);
	register_cli_cls_api_qos_cmds(garg);
	register_cli_filter_cmds(garg);
	register_cli_cls_cmds(ppio);
	register_cli_c3_cmds(ppio);
	register_cli_c2_cmds(ppio);
	register_cli_mng_cmds(ppio);
	app_register_cli_common_cmds(garg->ports_desc);

	return 0;
}

static int unregister_cli_cmds(struct glob_arg *garg)
{
	/* TODO: unregister cli cmds */
	return 0;
}

static int init_global(void *arg)
{
	struct glob_arg *garg = (struct glob_arg *)arg;
	int		 err;

	if (!garg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	err = init_all_modules();
	if (err)
		return err;

	err = init_local_modules(garg);
	if (err)
		return err;

	err = register_cli_cmds(garg);
	if (err)
		return err;

	return 0;
}

static void deinit_global(void *arg)
{
	struct glob_arg *garg = (struct glob_arg *)arg;

	if (!garg)
		return;
	if (garg->cli)
		unregister_cli_cmds(garg);

	destroy_local_modules(garg);
	destroy_all_modules();
}

static int init_local(void *arg, int id, void **_larg)
{
	struct glob_arg		*garg = (struct glob_arg *)arg;
	struct local_arg	*larg;
	int			 i, err;

	if (!garg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	larg = (struct local_arg *)malloc(sizeof(struct local_arg));
	if (!larg) {
		pr_err("No mem for local arg obj!\n");
		return -ENOMEM;
	}

	err = app_hif_init(&larg->hif, CLS_APP_HIF_Q_SIZE);
	if (err)
		return err;

	larg->id		= id;
	larg->echo		= garg->echo;
	larg->num_ports		= garg->num_ports;
	larg->ports_desc	= (struct lcl_port_desc *)malloc(larg->num_ports * sizeof(struct lcl_port_desc));
	if (!larg->ports_desc) {
		pr_err("no mem for local-port-desc obj!\n");
		return -ENOMEM;
	}
	memset(larg->ports_desc, 0, larg->num_ports * sizeof(struct lcl_port_desc));
	for (i = 0; i < larg->num_ports; i++)
		app_port_local_init(i, larg->id, &larg->ports_desc[i], &garg->ports_desc[i]);

	larg->pools_desc	= garg->pools_desc;
	larg->garg              = garg;

	larg->qs_map = garg->qs_map << (garg->qs_map_shift * id);

	*_larg = larg;
	return 0;
}

static void deinit_local(void *arg)
{
	struct local_arg *larg = (struct local_arg *)arg;
	int i;

	if (!larg)
		return;

	if (larg->ports_desc) {
		for (i = 0; i < larg->num_ports; i++)
			app_port_local_deinit(&larg->ports_desc[i]);
		free(larg->ports_desc);
	}

	if (larg->hif)
		pp2_hif_deinit(larg->hif);

	free(larg);
}

static void usage(char *progname)
{
	printf("\n"
		"MUSDK cls-demo application.\n"
		"\n"
		"Usage: %s OPTIONS\n"
		"  E.g. %s -i eth0\n"
		"\n"
		"Mandatory OPTIONS:\n"
		"\t-i, --interface <eth-interface>\n"
		"\n"
		"Optional OPTIONS:\n"
		"\t-e, --echo			(no argument) activate echo packets\n"
		"\t-b, --hash_type <none, 2-tuple, 5-tuple>\n"
		"\t--ppio_tag_mode		(no argument)configure ppio_tag_mode parameter\n"
		"\t--logical_port_params	(no argument)configure logical port parameters\n"
		"\n", MVAPPS_NO_PATH(progname), MVAPPS_NO_PATH(progname)
		);
}

static int parse_args(struct glob_arg *garg, int argc, char *argv[])
{
	int i = 1;
	int option;
	int long_index = 0;
	int ppio_tag_mode = 0;
	int logical_port_params = 0;
	char buff[CLS_APP_COMMAND_LINE_SIZE];
	int argc_cli;
	int rc;

	struct pp2_ppio_params	*port_params = &garg->ports_desc[0].port_params;
	struct pp2_init_params	*pp2_params = &garg->pp2_params;
	struct port_desc	*port = &garg->ports_desc[0];

	struct option long_options[] = {
		{"help", no_argument, 0, 'h'},
		{"interface", required_argument, 0, 'i'},
		{"echo", no_argument, 0, 'e'},
		{"hash_type", required_argument, 0, 'b'},
		{"ppio_tag_mode", no_argument, 0, 't'},
		{"logical_port_params", no_argument, 0, 'g'},
		{0, 0, 0, 0}
	};

	garg->cpus = 1;
	garg->qs_map = CLS_APP_QS_MAP_MASK;
	garg->qs_map_shift = CLS_APP_MAX_NUM_TCS_PER_PORT;
	garg->echo = 0;
	garg->hash_type = PP2_PPIO_HASH_T_2_TUPLE;
	garg->num_ports = 0;
	garg->cli = 1;

	memset(port_params, 0, sizeof(*port_params));
	memset(pp2_params, 0, sizeof(*pp2_params));
	port->ppio_type = PP2_PPIO_T_NIC;

	/* every time starting getopt we should reset optind */
	optind = 0;
	while ((option = getopt_long(argc, argv, "hi:b:etg", long_options, &long_index)) != -1) {
		switch (option) {
		case 'h':
			usage(argv[0]);
			exit(0);
			break;
		case 'i':
			snprintf(garg->ports_desc[garg->num_ports].name,
				 sizeof(garg->ports_desc[garg->num_ports].name), "%s", optarg);
			garg->num_ports++;
			/* currently supporting only 1 port */
			if (garg->num_ports > 1) {
				pr_err("too many ports specified (%d vs %d)\n",
				       garg->num_ports, 1);
				return -EINVAL;
			}
			break;
		case 'e':
			garg->echo = 1;
			break;
		case 'b':
			if (!strcmp(optarg, "none")) {
				garg->hash_type = PP2_PPIO_HASH_T_NONE;
			} else if (!strcmp(optarg, "2-tuple")) {
				garg->hash_type = PP2_PPIO_HASH_T_2_TUPLE;
			} else if (!strcmp(optarg, "5-tuple")) {
				garg->hash_type = PP2_PPIO_HASH_T_5_TUPLE;
			} else {
				printf("parsing fail, wrong input for hash\n");
				return -EINVAL;
			}
			break;
		case 't':
			ppio_tag_mode = true;
			break;
		case 'g':
			logical_port_params = true;
			port->ppio_type = PP2_PPIO_T_LOG;
			break;
		default:
			pr_err("argument (%s) not supported!\n", argv[i]);
			return -EINVAL;
		}
	}

	if (ppio_tag_mode) {
		rc = app_get_line("please enter tag_mode\n"
				  "\t\t\tno tag:--none			(no argument)\n"
				  "\t\t\tdsa tag:--dsa			(no argument)\n"
				  "\t\t\textended dsa tag:--extended_dsa(no argument)\n",
				  buff, sizeof(buff), &argc_cli, argv);
		if (rc) {
			pr_err("app_get_line failed!\n");
			return -EINVAL;
		}
		rc = pp2_cls_cli_ppio_tag_mode(garg, argc_cli, argv);
		if (rc) {
			pr_err("pp2_cls_cli_ppio_tag_mode failed!\n");
			return -EINVAL;
		}
	}

	if (logical_port_params) {
		rc = app_get_line("please enter logical port params:\n"
				  "\t\t\t--target		(dec)\n"
				  "\t\t\t--num_proto_rule_sets	(dec)\n"
				  "\t\t\t--num_rules		(dec)\n"
				  "\t\t\t--rule_type	(dec)\n"
				  "\t\t\t--proto		(dec)\n"
				  "\t\t\t--proto_val		(dec)\n"
				  "\t\t\t--special_proto	(dec)\n"
				  "\t\t\t--special_fields	(dec)\n"
				  "\t\t\t--field_val		(dec)\n",
				  buff, sizeof(buff), &argc_cli, argv);
		if (rc) {
			pr_err("app_get_line failed!\n");
			return -EINVAL;
		}
		rc = pp2_cls_logical_port_params(garg, argc_cli, argv);
		if (rc) {
			pr_err("pp2_cls_logical_port_params failed!\n");
			return -EINVAL;
		}
	}
	/* Now, check validity of all inputs */
	if (!garg->num_ports) {
		pr_err("No port defined!\n");
		return -EINVAL;
	}

	return 0;
}

int main(int argc, char *argv[])
{
	struct mvapp_params	mvapp_params;
	int			err;

	setbuf(stdout, NULL);

	pr_debug("pr_debug is enabled\n");

	err = parse_args(&garg, argc, argv);
	if (err)
		return err;

	garg.pp2_num_inst = pp2_get_num_inst();

	memset(&mvapp_params, 0, sizeof(mvapp_params));
	mvapp_params.use_cli		= garg.cli;
	mvapp_params.num_cores		= garg.cpus;
	mvapp_params.global_arg		= (void *)&garg;
	mvapp_params.init_global_cb	= init_global;
	mvapp_params.deinit_global_cb	= deinit_global;
	mvapp_params.init_local_cb	= init_local;
	mvapp_params.deinit_local_cb	= deinit_local;
	mvapp_params.main_loop_cb	= main_loop;
	return mvapp_go(&mvapp_params);
}
