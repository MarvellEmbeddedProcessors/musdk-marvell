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
#include "utils.h"
#include "drivers/mv_pp2_cls.h"
#include "src/drivers/ppv2/pp2.h"
#include "src/drivers/ppv2/cls/pp2_cls_types.h"
#include "src/drivers/ppv2/cls/pp2_cls_internal_types.h"
#include "src/drivers/ppv2/cls/pp2_c3.h"
#include "src/drivers/ppv2/cls/pp2_c2.h"
#include "src/drivers/ppv2/cls/pp2_flow_rules.h"
#include "src/drivers/ppv2/cls/pp2_cls_db.h"
#include "cls_debug.h"

#define CLS_APP_DMA_MEM_SIZE			(10 * 1024 * 1024)
#define CLS_APP_MAX_NUM_TCS_PER_PORT		4
#define CLS_APP_PP2_MAX_NUM_QS_PER_TC		1
#define CLS_APP_MAX_NUM_TBL			10
#define CLS_APP_MAX_NUM_RULES			10
#define CLS_APP_STR_SIZE_MAX			40
#define CLS_APP_KEY_SIZE_MAX			37
#define CLS_APP_PPIO_NAME_MAX			15
#define CLS_APP_PORT_NAME_MAX			15

#define CLS_APP_KEY_MEM_SIZE_MAX	(PP2_CLS_TBL_MAX_NUM_FIELDS * CLS_APP_STR_SIZE_MAX)

#define PREFETCH_SHIFT	7
#define PKT_ECHO_SUPPORT
#define USE_APP_PREFETCH

/** Get rid of path in filename - only for unix-type paths using '/' */
#define CLS_DBG_NO_PATH(file_name) (strrchr((file_name), '/') ? \
			    strrchr((file_name), '/') + 1 : (file_name))
#define CLS_TEST_MODULE_NAME_MAX 10

struct pp2_ppio {
	struct pp2_port *port;
};

struct lcl_port_desc {
	u32		 pp_id;
	u32		 ppio_id;
	struct pp2_ppio	*port;
};

struct port_desc {
	char		 name[CLS_APP_PORT_NAME_MAX];
	int		 pp_id;
	int		 ppio_id;
	struct pp2_ppio	*port;
};

struct tx_shadow_q_entry {
	struct pp2_buff_inf	buff_ptr;
};

struct tx_shadow_q {
	u16				 read_ind;
	u16				 write_ind;

	struct tx_shadow_q_entry	 ents[MVAPPS_Q_SIZE];
};

struct glob_arg {
	int			verbose;
	int			cli;
	int			cpus;	/* cpus used for running */
	int			echo;
	u64			qs_map;
	int			qs_map_shift;
	int			num_ports;
	int			pp2_num_inst;
	struct port_desc	ports_desc[MVAPPS_MAX_NUM_PORTS];
	struct pp2_hif		*hif;
	int			num_pools;
	struct pp2_bpool	***pools;
	struct pp2_buff_inf	***buffs_inf;
	char			test_module[CLS_TEST_MODULE_NAME_MAX];
	int			test_number;
	uintptr_t		cpu_slot;
};

struct local_arg {
	struct tx_shadow_q	shadow_qs[MVAPPS_MAX_NUM_QS_PER_CORE];
	u64			 qs_map;

	struct pp2_hif		*hif;
	int			 num_ports;
	struct lcl_port_desc	*ports_desc;

	struct pp2_bpool	***pools;
	int			 echo;
	int			 id;

	struct glob_arg		*garg;
};

static struct glob_arg garg = {};
static u8	pp2_num_inst;


struct pp2_cls_table_db_element {
	struct	pp2_cls_tbl		*tbl;
	struct	pp2_cls_tbl_params	tbl_params;
	char				ppio_name[CLS_APP_PPIO_NAME_MAX];
};

struct pp2_cls_table_db {
	u32				idx;
	struct pp2_cls_table_db_element	elem[CLS_APP_MAX_NUM_TBL];
};

struct pp2_cls_rule_key_element {
	struct pp2_cls_tbl_rule		rule_key;
	char				key[CLS_APP_KEY_MEM_SIZE_MAX];
	char				mask[CLS_APP_KEY_MEM_SIZE_MAX];
};

struct pp2_cls_rule_key_db {
	u32				idx;
	struct pp2_cls_rule_key_element	elem[CLS_APP_MAX_NUM_RULES];
};

struct pp2_cls_db {
	struct pp2_cls_table_db		table_db;
	struct pp2_cls_rule_key_db	rule_key_db;
};

static struct pp2_cls_db *cls_db;

static int find_port_info(struct port_desc *port_desc)
{
	char		 name[CLS_APP_PPIO_NAME_MAX];
	u8		 pp, ppio;
	int		 err;

	if (!port_desc->name) {
		pr_err("No port name given!\n");
		return -1;
	}

	memset(name, 0, sizeof(name));
	snprintf(name, sizeof(name), "%s", port_desc->name);
	err = pp2_netdev_get_port_info(name, &pp, &ppio);
	if (err) {
		pr_err("PP2 Port %s not found!\n", port_desc->name);
		return err;
	}

	port_desc->ppio_id = ppio;
	port_desc->pp_id = pp;

	return 0;
}

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
		key_size = 1;
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

static int pp2_cls_cli_table_add(void *arg, int argc, char *argv[])
{
	u32 idx = 0;
	int rc;
	int engine_type = -1;
	int key_size = 0;
	int num_fields;
	u32 proto[PP2_CLS_TBL_MAX_NUM_FIELDS];
	u32 field[PP2_CLS_TBL_MAX_NUM_FIELDS];
	char name[CLS_APP_PPIO_NAME_MAX];
	int traffic_class = -1;
	int action_type = PP2_CLS_TBL_ACT_DONE;
	char *ret_ptr;
	struct pp2_cls_tbl_params *tbl_ptr;
	struct pp2_cls_cos_desc *action_cos;
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

	action_cos = malloc(sizeof(struct pp2_cls_cos_desc));
	if (!action_cos) {
		pr_err("%s(%d) no mem for pp2_cls_cos_desc!\n", __func__, __LINE__);
		return -ENOMEM;
	}

	tbl_ptr = &cls_db->table_db.elem[cls_db->table_db.idx].tbl_params;

	tbl_ptr->type = engine_type;
	tbl_ptr->max_num_rules = MVPP2_CLS_DEF_FLOW_LEN;
	tbl_ptr->key.key_size = key_size;
	tbl_ptr->key.num_fields = num_fields;
	for (idx = 0; idx < tbl_ptr->key.num_fields; idx++) {
		tbl_ptr->key.proto_field[idx].proto = proto[idx];
		tbl_ptr->key.proto_field[idx].field.eth = field[idx];
	}
	tbl_ptr->default_act.type = action_type;
	tbl_ptr->default_act.cos = action_cos;
	tbl_ptr->default_act.cos->ppio = garg.ports_desc[0].port;
	tbl_ptr->default_act.cos->tc = traffic_class;

	memset(name, 0, sizeof(name));
	snprintf(name, sizeof(name), "ppio-%d:%d",
		 garg.ports_desc[0].pp_id, garg.ports_desc[0].ppio_id);

	strcpy(&cls_db->table_db.elem[cls_db->table_db.idx].ppio_name[0], name);
	if (!pp2_cls_tbl_init(tbl_ptr, &cls_db->table_db.elem[cls_db->table_db.idx].tbl)) {
		printf("OK\n");
		cls_db->table_db.idx++;
	} else {
		printf("FAIL\n");
	}

	free(action_cos);

	return 0;
}

static int pp2_cls_cli_cls_rule_key_add(void *arg, int argc, char *argv[])
{
	u32 idx = 0;
	int tbl_idx = -1;
	int traffic_class = -1;
	int action_type = PP2_CLS_TBL_ACT_DONE;
	int rc;
	u32 num_fields;
	struct pp2_cls_tbl *tbl;
	struct pp2_cls_tbl_action action;
	struct pp2_cls_cos_desc *action_cos;
	struct pp2_cls_rule_key_element *rule_key_ptr;
	u32 key_size[PP2_CLS_TBL_MAX_NUM_FIELDS];
	u8 key[PP2_CLS_TBL_MAX_NUM_FIELDS][CLS_APP_STR_SIZE_MAX] = {0};
	u8 mask[PP2_CLS_TBL_MAX_NUM_FIELDS][CLS_APP_STR_SIZE_MAX] = {0};
	char *ret_ptr;
	int option = 0;
	int long_index = 0;
	struct option long_options[] = {
		{"size", required_argument, 0, 's'},
		{"key", required_argument, 0, 'k'},
		{"mask", required_argument, 0, 'm'},
		{"table_index", required_argument, 0, 't'},
		{"drop", no_argument, 0, 'd'},
		{"tc", required_argument, 0, 'q'},
		{0, 0, 0, 0}
	};

	if (argc < 3 || argc > 36) {
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
			if ((optarg == ret_ptr) || (tbl_idx < 0) || (tbl_idx >= cls_db->table_db.idx)) {
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
			    (key_size[idx] > CLS_APP_MAX_NUM_RULES)) {
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

	/* adding new key into rule key table */
	if (cls_db->rule_key_db.idx < CLS_APP_MAX_NUM_RULES) {
		u32 field_idx;
		rule_key_ptr = &cls_db->rule_key_db.elem[cls_db->rule_key_db.idx];

		rule_key_ptr->rule_key.num_fields = num_fields;
		for (idx = 0; idx < num_fields; idx++) {
			field_idx = (CLS_APP_STR_SIZE_MAX * idx);
			rule_key_ptr->rule_key.fields[idx].size = key_size[idx];
			strcpy(&rule_key_ptr->key[field_idx], (char *)&key[idx][0]);
			rule_key_ptr->rule_key.fields[idx].key = (u8 *)&rule_key_ptr->key[field_idx];
			strcpy(&rule_key_ptr->mask[field_idx], (char *)&mask[idx][0]);
			rule_key_ptr->rule_key.fields[idx].mask = (u8 *)&rule_key_ptr->mask[field_idx];
			printf("1: | %4d , %26s , %26s |\n",
			       rule_key_ptr->rule_key.fields[idx].size,
			       rule_key_ptr->rule_key.fields[idx].key,
			       rule_key_ptr->rule_key.fields[idx].mask);
		}
		cls_db->rule_key_db.idx++;
	} else {
		printf("FAIL: rule_key table is full: %d\n", cls_db->rule_key_db.idx);
		return -EINVAL;
	}

	tbl = cls_db->table_db.elem[tbl_idx].tbl;

	action_cos = malloc(sizeof(struct pp2_cls_cos_desc));
	if (!action_cos) {
		pr_err("%s(%d) no mem for pp2_cls_cos_desc!\n", __func__, __LINE__);
		return -ENOMEM;
	}
	action.type = action_type;
	action_cos->tc = traffic_class;
	action.cos = action_cos;
	action.cos->ppio = garg.ports_desc[0].port;
	action.cos->tc = traffic_class;

	if (!pp2_cls_tbl_add_rule(tbl, &rule_key_ptr->rule_key, &action))
		printf("OK\n");
	else
		printf("FAIL: unable to add rule to table index: %d\n", tbl_idx);

	free(action_cos);

	return 0;
}

static int pp2_cls_cli_cls_rule_modify(void *arg, int argc, char *argv[])
{
	int tbl_idx = -1;
	int rule_idx = -1;
	int traffic_class = -1;
	int action_type = PP2_CLS_TBL_ACT_DONE;
	char *ret_ptr;
	struct pp2_cls_tbl *tbl;
	struct pp2_cls_tbl_rule	*rule;
	struct pp2_cls_tbl_action action;
	struct pp2_cls_cos_desc *action_cos;
	int i, option = 0;
	int long_index = 0;
	struct option long_options[] = {
		{"table_index", required_argument, 0, 't'},
		{"rule_index", required_argument, 0, 'r'},
		{"drop", no_argument, 0, 'd'},
		{"tc", required_argument, 0, 'q'},
		{0, 0, 0, 0}
	};

	if (argc != 7) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	/* every time starting getopt we should reset optind */
	optind = 0;

	/* Get parameters */
	for (i = 0; ((option = getopt_long_only(argc, argv, "", long_options, &long_index)) != -1); i++) {
		switch (option) {
		case 't':
			tbl_idx = strtoul(optarg, &ret_ptr, 0);
			if ((optarg == ret_ptr) || (tbl_idx < 0) || (tbl_idx >= cls_db->table_db.idx)) {
				printf("parsing fail, wrong input for --table_index\n");
				return -EINVAL;
			}
			break;
		case 'r':
			rule_idx = strtoul(optarg, &ret_ptr, 0);
			if ((optarg == ret_ptr) || (rule_idx < 0) || (rule_idx >= cls_db->rule_key_db.idx)) {
				printf("parsing fail, wrong input for argv[2] --rule_index\n");
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
		default:
			printf("parsing fail, wrong input\n");
			return -EINVAL;
		}
	}

	/* check if all the fields are initialized */
	if (rule_idx < 0) {
		printf("parsing fail, invalid --rule_index\n");
		return -EINVAL;
	}

	if (tbl_idx < 0) {
		printf("parsing fail, invalid --table_index\n");
		return -EINVAL;
	}

	if (traffic_class < 0) {
		printf("parsing fail, invalid --tc\n");
		return -EINVAL;
	}

	tbl = cls_db->table_db.elem[tbl_idx].tbl;
	rule = &cls_db->rule_key_db.elem[rule_idx].rule_key;

	action_cos = malloc(sizeof(struct pp2_cls_cos_desc));
	if (!action_cos) {
		pr_err("%s(%d) no mem for pp2_cls_cos_desc!\n", __func__, __LINE__);
		return -ENOMEM;
	}
	action.type = action_type;
	action_cos->tc = traffic_class;
	action.cos = action_cos;
	action.cos->ppio = garg.ports_desc[0].port;
	action.cos->tc = traffic_class;

	if (!pp2_cls_tbl_modify_rule(tbl, rule, &action))
		printf("OK\n");
	else
		printf("FAIL: unable to modify rule to table index: %d\n", tbl_idx);

	free(action_cos);

	return 0;
}

static int pp2_cls_cli_cls_rule_remove(void *arg, int argc, char *argv[])
{
	int tbl_idx = -1;
	int rule_idx = -1;
	char *ret_ptr;
	struct pp2_cls_tbl *tbl;
	struct pp2_cls_tbl_rule	*rule;
	int i, option = 0;
	int long_index = 0;
	struct option long_options[] = {
		{"table_index", required_argument, 0, 't'},
		{"rule_index", required_argument, 0, 'r'},
		{0, 0, 0, 0}
	};

	if (argc != 5) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	/* every time starting getopt we should reset optind */
	optind = 0;
	/* Get parameters */
	for (i = 0; ((option = getopt_long_only(argc, argv, "", long_options, &long_index)) != -1); i++) {
		switch (option) {
		case 't':
			tbl_idx = strtoul(optarg, &ret_ptr, 0);
			if ((optarg == ret_ptr) || (tbl_idx < 0) || (tbl_idx >= cls_db->table_db.idx)) {
				printf("parsing fail, wrong input for --table_index\n");
				return -EINVAL;
			}
			break;
		case 'r':
			rule_idx = strtoul(optarg, &ret_ptr, 0);
			if ((optarg == ret_ptr) || (rule_idx < 0) || (rule_idx >= cls_db->rule_key_db.idx)) {
				printf("parsing fail, wrong input for --rule_index\n");
				return -EINVAL;
			}
			break;
		default:
			printf("parsing fail, wrong input\n");
			return -EINVAL;
		}
	}

	/* check if all the fields are initialized */
	if (rule_idx < 0) {
		printf("parsing fail, invalid --rule_index\n");
		return -EINVAL;
	}

	if (tbl_idx < 0) {
		printf("parsing fail, invalid --table_index\n");
		return -EINVAL;
	}

	tbl = cls_db->table_db.elem[tbl_idx].tbl;
	rule = &cls_db->rule_key_db.elem[rule_idx].rule_key;
	if (!pp2_cls_tbl_remove_rule(tbl, rule))
		printf("OK\n");
	else
		printf("FAIL: unable to remove rule to table index: %d\n", tbl_idx);
	return 0;
}

static int pp2_cls_cli_cls_table_dump(void *arg, int argc, char *argv[])
{
	u32 idx1, idx2;

	printf("total indexes: %d\n", cls_db->table_db.idx);
	printf("|idx|type|num_rules|key_size|num_fields|proto,field|proto,field|proto,field|proto,field|proto,field|");
	printf("    port  |type|tc_num|\n");
	for (idx1 = 0; idx1 < cls_db->table_db.idx; idx1++) {
		struct pp2_cls_tbl_params *tbl_ptr = &cls_db->table_db.elem[idx1].tbl_params;

		printf("|%3d|%4d|%9d|", idx1, tbl_ptr->type, tbl_ptr->max_num_rules);
		printf("%8d|%10d|", tbl_ptr->key.key_size, tbl_ptr->key.num_fields);
		for (idx2 = 0; idx2 < tbl_ptr->key.num_fields; idx2++) {
			printf("%5d,%5d|", tbl_ptr->key.proto_field[idx2].proto,
			       tbl_ptr->key.proto_field[idx2].field.eth);
		}
		printf("%10s|%4d|%6d|\n", cls_db->table_db.elem[idx1].ppio_name, tbl_ptr->default_act.type,
		       tbl_ptr->default_act.cos->tc);
	}
	printf("OK\n");

	return 0;
}

static int pp2_cls_cli_cls_rule_key_dump(void *arg, int argc, char *argv[])
{
	u32 idx1, idx2;

	printf("total indexes: %d\n", cls_db->rule_key_db.idx);
	printf("| idx | num_fields |	size ,           key              ,            mask            |\n");
	app_print_horizontal_line(86, "=");
	for (idx1 = 0; idx1 < cls_db->rule_key_db.idx; idx1++) {
		struct pp2_cls_tbl_rule *rule_key_ptr = &cls_db->rule_key_db.elem[idx1].rule_key;

		printf("| %3d | %10d | 1: %4d , %26s , %26s |\n", idx1,
		       rule_key_ptr->num_fields,
		       rule_key_ptr->fields[0].size,
		       rule_key_ptr->fields[0].key,
		       rule_key_ptr->fields[0].mask);

		for (idx2 = 1; idx2 < rule_key_ptr->num_fields; idx2++) {
			printf("|     |            | %d: %4d , %26s , %26s |\n", idx2,
			       rule_key_ptr->fields[idx2].size,
			       rule_key_ptr->fields[idx2].key,
			       rule_key_ptr->fields[idx2].mask);
		}
		app_print_horizontal_line(86, "-");
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

			rc = pp2_ppio_set_mac_addr(garg.ports_desc[0].port, mac);
			if (rc) {
				printf("Unable to set mac address\n");
				return -EINVAL;
			}
			break;
		case 'g':
			rc = pp2_ppio_get_mac_addr(garg.ports_desc[0].port, mac);
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

			rc = pp2_ppio_add_mac_addr(garg.ports_desc[0].port, mac);
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

			rc = pp2_ppio_remove_mac_addr(garg.ports_desc[0].port, mac);
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
			rc = pp2_ppio_flush_mac_addrs(garg.ports_desc[0].port, uc, mc);
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

	if ((mode < 0) || (cmd < 0)){
		printf("Wrong inputs in promiscuous mode command\n");
		return -EINVAL;
	}

	if (cmd && mode) {
		rc = pp2_ppio_set_uc_promisc(garg.ports_desc[0].port, en);
		if (rc) {
			printf("Unable to enable unicast promiscuous mode\n");
			return -rc;
		}
	} else if (cmd && !mode) {
		rc = pp2_ppio_set_mc_promisc(garg.ports_desc[0].port, en);
		if (rc) {
			printf("Unable to enable all multicast mode\n");
			return -rc;
		}
	} else if (!cmd && mode) {
		rc = pp2_ppio_get_uc_promisc(garg.ports_desc[0].port, &en);
		if (rc) {
			printf("Unable to get unicast promiscuous mode\n");
			return -rc;
		}
		if (en)
			printf("unicast promiscuous mode: enabled\n");
		else
			printf("unicast promiscuous mode: disabled\n");
	}else if (!cmd && !mode) {
		rc = pp2_ppio_get_mc_promisc(garg.ports_desc[0].port, &en);
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
		rc = pp2_ppio_add_vlan(garg.ports_desc[0].port, vlan_id);
		if (rc) {
			printf("Unable to add vlan id %d\n", vlan_id);
			return -EINVAL;
		}
	} else if (cmd == 1 && vlan_id >= 0 && vlan_id <= 4095) {
		rc = pp2_ppio_remove_vlan(garg.ports_desc[0].port, vlan_id);
		if (rc) {
			printf("Unable to remove vlan id %d\n", vlan_id);
			return -EINVAL;
		}
	} else if (cmd == 2) {
		rc = pp2_ppio_flush_vlan(garg.ports_desc[0].port);
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


#ifdef PKT_ECHO_SUPPORT
#ifdef USE_APP_PREFETCH
static inline void prefetch(const void *ptr)
{
	asm volatile("prfm pldl1keep, %a0\n" : : "p" (ptr));
}
#endif /* USE_APP_PREFETCH */

#endif /* PKT_ECHO_SUPPORT */

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
	struct pp2_ppio_desc descs[MVAPPS_MAX_BURST_SIZE];
	int err;
	u16 i;
#ifdef PKT_ECHO_SUPPORT
	int			prefetch_shift = PREFETCH_SHIFT;
#endif /* PKT_ECHO_SUPPORT */
	bpool = larg->pools[larg->ports_desc[rx_ppio_id].pp_id][bpool_id];
	shadow_q = &(larg->shadow_qs[tc]);
	err = pp2_ppio_recv(larg->ports_desc[rx_ppio_id].port, tc, qid, descs, &num);

	for (i = 0; i < num; i++) {
		char		*buff = (char *)(uintptr_t)pp2_ppio_inq_desc_get_cookie(&descs[i]);
		dma_addr_t	 pa = pp2_ppio_inq_desc_get_phys_addr(&descs[i]);

		u16 len = pp2_ppio_inq_desc_get_pkt_len(&descs[i]) - PP2_MH_SIZE;

#ifdef PKT_ECHO_SUPPORT
		if (likely(larg->echo)) {
			char *tmp_buff;
#ifdef USE_APP_PREFETCH
			if (num-i > prefetch_shift) {
				tmp_buff = (char *)(uintptr_t)pp2_ppio_inq_desc_get_cookie(&descs[i + prefetch_shift]);
				tmp_buff += MVAPPS_PKT_EFEC_OFFS;
				pr_debug("tmp_buff_before(%p)\n", tmp_buff);
				tmp_buff = (char *)(((uintptr_t)tmp_buff) | app_get_sys_dma_high_addr());
				pr_debug("tmp_buff_after(%p)\n", tmp_buff);
				prefetch(tmp_buff);
			}
#endif /* USE_APP_PREFETCH */
			tmp_buff = (char *)(((uintptr_t)(buff)) | app_get_sys_dma_high_addr());
			pr_debug("buff2(%p)\n", tmp_buff);
			tmp_buff += MVAPPS_PKT_EFEC_OFFS;
			swap_l2(tmp_buff);
			swap_l3(tmp_buff);
		}
#endif /* PKT_ECHO_SUPPORT */
		pp2_ppio_outq_desc_reset(&descs[i]);
		pp2_ppio_outq_desc_set_phys_addr(&descs[i], pa);
		pp2_ppio_outq_desc_set_pkt_offset(&descs[i], MVAPPS_PKT_EFEC_OFFS);
		pp2_ppio_outq_desc_set_pkt_len(&descs[i], len);
		shadow_q->ents[shadow_q->write_ind].buff_ptr.cookie = (uintptr_t)buff;
		shadow_q->ents[shadow_q->write_ind].buff_ptr.addr = pa;
		pr_debug("buff_ptr.cookie(0x%lx)\n", (u64)shadow_q->ents[shadow_q->write_ind].buff_ptr.cookie);
		shadow_q->write_ind++;
		if (shadow_q->write_ind == MVAPPS_Q_SIZE)
			shadow_q->write_ind = 0;
	}

	if (num) {
		err = pp2_ppio_send(larg->ports_desc[tx_ppio_id].port, larg->hif, tc, descs, &num);
		if (err) {
			pr_err("pp2_ppio_send\n");
			return err;
		}
	}

	pp2_ppio_get_num_outq_done(larg->ports_desc[tx_ppio_id].port, larg->hif, tc, &num);
	for (i = 0; i < num; i++) {
		binf = &(shadow_q->ents[shadow_q->read_ind].buff_ptr);
		if (unlikely(!binf->cookie || !binf->addr)) {
			pr_err("Shadow memory @%d: cookie(%lx), pa(%lx)!\n",
				   shadow_q->read_ind, (u64)binf->cookie, (u64)binf->addr);
			continue;
		}
		pp2_bpool_put_buff(larg->hif,
				   bpool,
				   binf);
		shadow_q->read_ind++;
		if (shadow_q->read_ind == MVAPPS_Q_SIZE)
			shadow_q->read_ind = 0;
	}

	return 0;
}

static int loop_1p(struct local_arg *larg, volatile int *running)
{
	int err;
	u16 num;
	u8 tc = 0, qid = 0;

	if (!larg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	num = MVAPPS_DFLT_BURST_SIZE;

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

static int main_loop(void *arg, volatile int *running)
{
	struct local_arg *larg = (struct local_arg *)arg;

	if (!larg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	if (larg->echo)
		return loop_1p(larg, running);

	while (*running);

	return 0;
}

static int init_all_modules(void)
{
	struct pp2_init_params	 pp2_params;
	int			 err;

	pr_info("Global initializations ... ");

	err = mv_sys_dma_mem_init(CLS_APP_DMA_MEM_SIZE);
	if (err)
		return err;

	memset(&pp2_params, 0, sizeof(pp2_params));
	pp2_params.hif_reserved_map = MVAPPS_PP2_HIFS_RSRV;
	pp2_params.bm_pool_reserved_map = MVAPPS_PP2_BPOOLS_RSRV;
	/* Enable 10G port */
	pp2_params.ppios[0][0].is_enabled = 1;
	pp2_params.ppios[0][0].first_inq = 0;
	/* Enable 1G ports */
	pp2_params.ppios[0][1].is_enabled = 1;
	pp2_params.ppios[0][1].first_inq = 0;
	pp2_params.ppios[0][2].is_enabled = 1;
	pp2_params.ppios[0][2].first_inq = 0;

	if (garg.pp2_num_inst == 2) {
		/* Enable 10G port */
		pp2_params.ppios[1][0].is_enabled = 1;
		pp2_params.ppios[1][0].first_inq = 0;
		/* Enable 1G ports */
		pp2_params.ppios[1][1].is_enabled = 1;
		pp2_params.ppios[1][1].first_inq = 0;
		pp2_params.ppios[1][2].is_enabled = 1;
		pp2_params.ppios[1][2].first_inq = 0;
	}
	err = pp2_init(&pp2_params);
	if (err)
		return err;

	pr_info("done\n");
	return 0;
}

static int init_local_modules(struct glob_arg *garg)
{
	char				name[CLS_APP_PPIO_NAME_MAX];
	struct pp2_ppio_params		port_params;
	struct pp2_ppio_inq_params	inq_params;
	int				i = 0, j, err, port_index;
	struct bpool_inf		infs[] = MVAPPS_BPOOLS_INF;
	struct pp2_port *port;

	pr_info("Local initializations ... ");

	err = app_hif_init(&garg->hif);
	if (err)
		return err;

	garg->num_pools = ARRAY_SIZE(infs);
	err = app_build_all_bpools(&garg->pools, &garg->buffs_inf, garg->num_pools, infs, garg->hif);
	if (err)
		return err;

	for (port_index = 0; port_index < garg->num_ports; port_index++) {
		err = find_port_info(&garg->ports_desc[port_index]);
		if (err) {
			pr_err("Port info not found!\n");
			return err;
		}

		memset(name, 0, sizeof(name));
		snprintf(name, sizeof(name), "ppio-%d:%d",
			 garg->ports_desc[port_index].pp_id, garg->ports_desc[port_index].ppio_id);
		pr_debug("found port: %s\n", name);
		memset(&port_params, 0, sizeof(port_params));
		port_params.match = name;
		port_params.type = PP2_PPIO_T_NIC;
		port_params.inqs_params.num_tcs = CLS_APP_MAX_NUM_TCS_PER_PORT;
		for (i = 0; i < port_params.inqs_params.num_tcs; i++) {
			port_params.inqs_params.tcs_params[i].pkt_offset = MVAPPS_PKT_OFFS >> 2;
			port_params.inqs_params.tcs_params[i].num_in_qs = CLS_APP_PP2_MAX_NUM_QS_PER_TC;
			/* TODO: we assume here only one Q per TC; change it! */
			inq_params.size = MVAPPS_Q_SIZE;
			port_params.inqs_params.tcs_params[i].inqs_params = &inq_params;
			for (j = 0; j < garg->num_pools; j++)
				port_params.inqs_params.tcs_params[i].pools[j] =
					garg->pools[garg->ports_desc[port_index].pp_id][j];
		}
		port_params.outqs_params.num_outqs = CLS_APP_MAX_NUM_TCS_PER_PORT;
		for (i = 0; i < port_params.outqs_params.num_outqs; i++) {
			port_params.outqs_params.outqs_params[i].size = MVAPPS_Q_SIZE;
			port_params.outqs_params.outqs_params[i].weight = 1;
		}
		err = pp2_ppio_init(&port_params, &garg->ports_desc[port_index].port);
		if (err)
			return err;
		if (!garg->ports_desc[port_index].port) {
			pr_err("PP-IO init failed!\n");
			return -EIO;
		}

		err = pp2_ppio_enable(garg->ports_desc[port_index].port);
		if (err)
			return err;
	}
	/* cls memory allocations */
	cls_db = malloc(sizeof(*cls_db));
	if (!cls_db) {
		pr_err("no mem for cls_db array!\n");
		return -ENOMEM;
	}

	port = garg->ports_desc[0].port->port;
	garg->cpu_slot = port->cpu_slot;

	if (!garg->cpu_slot) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

#ifdef CLS_DEBUG
	if (strncmp(garg->test_module, "parser", 6) == 0) {
		pr_info("Parser tests not implemented yet\n");
		return -EINVAL;
	} else if (strncmp(garg->test_module, "issue", 5) == 0) {
	} else if (strncmp(garg->test_module, "c2", 2) == 0) {
		pr_info("c2 tests not implemented yet\n");
		return -EINVAL;
	} else if (strncmp(garg->test_module, "c3", 2) == 0) {
		if (garg->test_number < 1 || garg->test_number > 5) {
			pr_err("c3 test number not supported\n");
			return -EINVAL;
		}
		pr_info("*************start test c3************\n");
		pp2_cls_c3_test(garg->cpu_slot, garg->test_number);
	}
#endif

	pr_info("done\n");
	return 0;
}

static void destroy_local_modules(struct glob_arg *garg)
{
	int	i, j;
	pp2_num_inst = garg->pp2_num_inst;

	if (garg->ports_desc[0].port) {
		pp2_ppio_disable(garg->ports_desc[0].port);
		pp2_ppio_deinit(garg->ports_desc[0].port);
	}

	if (garg->pools) {
		for (i = 0; i < pp2_num_inst; i++) {
			if (garg->pools[i]) {
				for (j = 0; j < garg->num_pools; j++)
					if (garg->pools[i][j])
						pp2_bpool_deinit(garg->pools[i][j]);
				free(garg->pools[i]);
			}
		}
		free(garg->pools);
	}
	if (garg->buffs_inf) {
		for (i = 0; i < pp2_num_inst; i++) {
			if (garg->buffs_inf[i]) {
				for (j = 0; j < garg->num_pools; j++)
					if (garg->buffs_inf[i][j])
						free(garg->buffs_inf[i][j]);
				free(garg->buffs_inf[i]);
			}
		}
		free(garg->buffs_inf);
	}

	if (garg->hif)
		pp2_hif_deinit(garg->hif);

	if (cls_db)
		free(cls_db);
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
	cmd_params.name		= "cls_rule_key_add";
	cmd_params.desc		= "add a classifier rule key to existing table";
	cmd_params.format	= "--table_index --tc --drop(optional) --size --key --mask...\n"
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
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_cls_rule_key_add;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_rule_modify";
	cmd_params.desc		= "modify a classifier rule in specified table";
	cmd_params.format	= "\n"
				  "\t\t\t\t--table_index	(dec) index to existing table\n"
				  "\t\t\t\t--rule_index		(dec) index to existing rule in database\n"
				  "\t\t\t\t--tc			(dec) 1..8\n"
				  "\t\t\t\t--drop		(optional)(no argument)\n";
	cmd_params.cmd_arg	= garg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_cls_rule_modify;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_rule_remove";
	cmd_params.desc		= "modify a classifier rule in specified table";
	cmd_params.format	= "--table_index --rule_index\n"
				  "\t\t\t\t--table_index	(dec) index to existing table\n"
				  "\t\t\t\t--rule_index		(dec) index to existing rule in database\n";
	cmd_params.cmd_arg	= garg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_cls_rule_remove;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_table_dump";
	cmd_params.desc		= "display classifier defined tables";
	cmd_params.format	= "";
	cmd_params.cmd_arg	= garg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_cls_table_dump;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_rule_key_dump";
	cmd_params.desc		= "display classifier defined rule_keys";
	cmd_params.format	= "";
	cmd_params.cmd_arg	= garg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_cls_rule_key_dump;
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
	struct pp2_ppio *ppio = garg->ports_desc[0].port;
	struct pp2_port *port = ppio->port;

	if (!garg->cli)
		return -EFAULT;

	garg->cpu_slot = port->cpu_slot;

	register_cli_cls_api_cmds(garg);
	register_cli_filter_cmds(garg);
	register_cli_cls_cmds(garg->cpu_slot);
	register_cli_c3_cmds(garg->cpu_slot);
	register_cli_c2_cmds(garg->cpu_slot);

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

	err = app_hif_init(&larg->hif);
	if (err)
		return err;

	larg->id                = id;
	larg->echo              = garg->echo;
	larg->num_ports         = garg->num_ports;
	larg->ports_desc = (struct lcl_port_desc *)malloc(larg->num_ports*sizeof(struct lcl_port_desc));
	if (!larg->ports_desc) {
		pr_err("no mem for local-port-desc obj!\n");
		return -ENOMEM;
	}
	memset(larg->ports_desc, 0, larg->num_ports*sizeof(struct lcl_port_desc));
	for (i = 0; i < larg->num_ports; i++) {
		larg->ports_desc[i].pp_id = garg->ports_desc[i].pp_id;
		larg->ports_desc[i].ppio_id = garg->ports_desc[i].ppio_id;
		larg->ports_desc[i].port = garg->ports_desc[i].port;

	}
	larg->pools             = garg->pools;
	larg->garg              = garg;

	larg->qs_map = garg->qs_map << (garg->qs_map_shift * id);

	*_larg = larg;
	return 0;
}

static void deinit_local(void *arg)
{
	struct glob_arg *garg = (struct glob_arg *)arg;

	if (!garg)
		return;
}

static void usage(char *progname)
{
	printf("\n"
		"MUSDK cls-test application.\n"
		"\n"
		"Usage: %s OPTIONS\n"
	"  E.g. %s -i ppio-0:0\n"
	    "\n"
	    "Mandatory OPTIONS:\n"
		"\t-i, --interface <eth-interface>\n"
		"\n"
		"Optional OPTIONS:\n"
		"\t-e, --echo	(no argument) activate echo packets\n"
#ifdef CLS_DEBUG
		"\t-m <test module>	test module <parser,issue,c2,c3,rss>\n"
		"\t-n <test number>	select test number\n"
#endif
		"\n", CLS_DBG_NO_PATH(progname), CLS_DBG_NO_PATH(progname)
		);
}

static int parse_args(struct glob_arg *garg, int argc, char *argv[])
{
	int i = 1;
	int option;
	int long_index = 0;
	struct option long_options[] = {
		{"help", no_argument, 0, 'h'},
		{"interface", required_argument, 0, 'i'},
		{"echo", no_argument, 0, 'e'},
		{0, 0, 0, 0}
	};

	garg->cpus = 1;
	garg->qs_map = 0xf;
	garg->qs_map_shift = CLS_APP_MAX_NUM_TCS_PER_PORT;
	garg->echo = 0;
	garg->num_ports = 0;
	garg->cli = 1;
	garg->test_number = 0;

	/* every time starting getopt we should reset optind */
	optind = 0;
	while ((option = getopt_long(argc, argv, "hi:n:m:", long_options, &long_index)) != -1) {
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
#ifdef CLS_DEBUG
		case 'm':
			snprintf(garg->test_module, sizeof(garg->test_module), "%s", optarg);
			break;
		case 'n':
			garg->test_number = atoi(optarg);
			break;
#endif
		default:
			pr_err("argument (%s) not supported!\n", argv[i]);
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
