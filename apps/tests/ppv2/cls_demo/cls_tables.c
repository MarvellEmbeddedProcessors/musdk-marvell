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
#include "mvapp.h"
#include "cls_main.h"

#include "mv_pp2_ppio.h"

#define CLS_APP_KEY_SIZE_MAX			37
#define CLS_APP_STR_SIZE_MAX			40
#define CLS_APP_MAX_NUM_OF_RULES		20

static struct list cls_flow_tbl_head;

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
	} else if (!strcmp(optarg, "tcp_src")) {
		*proto = MV_NET_PROTO_TCP;
		*field = MV_NET_TCP_F_SP;
		key_size = 2;
	} else if (!strcmp(optarg, "tcp_dst")) {
		*proto = MV_NET_PROTO_TCP;
		*field = MV_NET_TCP_F_DP;
		key_size = 2;
	} else if (!strcmp(optarg, "udp_src")) {
		*proto = MV_NET_PROTO_UDP;
		*field = MV_NET_UDP_F_SP;
		key_size = 2;
	} else if (!strcmp(optarg, "udp_dst")) {
		*proto = MV_NET_PROTO_UDP;
		*field = MV_NET_UDP_F_DP;
		key_size = 2;
	}
	return key_size;
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
	struct port_desc *ports_desc = (struct port_desc *)arg;
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
				engine_type = PP2_CLS_TBL_EXACT_MATCH;
			} else if (!strcmp(optarg, "maskable")) {
				engine_type = PP2_CLS_TBL_MASKABLE;
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
	tbl_params->default_act.cos->ppio = ports_desc->ppio;
	tbl_params->default_act.cos->tc = traffic_class;

	memset(name, 0, sizeof(name));
	snprintf(name, sizeof(name), "ppio-%d:%d",
		 ports_desc->pp_id, ports_desc->ppio_id);

	strcpy(&tbl_node->ppio_name[0], name);

	if (!pp2_cls_tbl_init(tbl_params, &tbl_node->tbl))
		pr_info("table created, table_index %d\n", tbl_node->idx);
	else
		pr_info("FAIL\n");

	return 0;
}

int pp2_cls_table_remove(u32 tbl_idx, struct list *cls_tbl_head)
{
	struct pp2_cls_table_node *tbl_node;

	LIST_FOR_EACH_OBJECT(tbl_node, struct pp2_cls_table_node, cls_tbl_head, list_node) {
		pr_debug("tbl_node->idx %d, tbl_idx %d\n", tbl_node->idx, tbl_idx);

		if (tbl_node->idx == tbl_idx) {
			struct pp2_cls_tbl_params *tbl_ptr = &tbl_node->tbl_params;

			pr_info("Removing table %d\n", tbl_idx);
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
			if ((optarg == ret_ptr) || (tbl_idx <= 0) || (tbl_idx > list_num_objs(&cls_flow_tbl_head))) {
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

static int pp2_cls_cli_cls_rule_key(void *arg, int argc, char *argv[])
{
	struct port_desc *ports_desc = (struct port_desc *)arg;
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
		action->cos->ppio = ports_desc->ppio;

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

void unregister_cli_cls_api_cmds(void)
{
	struct pp2_cls_table_node *tbl_node;

	LIST_FOR_EACH_OBJECT(tbl_node, struct pp2_cls_table_node, &cls_flow_tbl_head, list_node) {
		pp2_cls_table_remove(tbl_node->idx,  &cls_flow_tbl_head);
	}
}

int register_cli_cls_api_cmds(struct port_desc *arg)
{
	struct cli_cmd_params cmd_params;

	INIT_LIST(&cls_flow_tbl_head);

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
				  "\t\t\t\t			tcp_src - tcp, source port\n"
				  "\t\t\t\t			tcp_dst - tcp, destination port\n"
				  "\t\t\t\t			udp_src - udp, source port\n"
				  "\t\t\t\t			udp_dst - udp, destination port\n";
	cmd_params.cmd_arg	= arg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_table_add;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_rule_key";
	cmd_params.desc		= "add/modify/remove a classifier rule key to existing table";
	cmd_params.format	= "\t--add    --table_index --tc --drop(optional) --size --key --mask...\n"
				  "\t\t\t\t\t--modify --table_index --tc --drop(optional) --size --key --mask...\n"
				  "\t\t\t\t\t--remove --table_index --tc --drop(optional) --size --key --mask...\n"
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
	cmd_params.cmd_arg	= arg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_cls_rule_key;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_tbl_deinit";
	cmd_params.desc		= "remove a specified table";
	cmd_params.format	= "--table_index (dec) index to existing table\n";
	cmd_params.cmd_arg	= arg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_table_remove;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_tbl_dump";
	cmd_params.desc		= "display classifier defined tables in cls_demo application";
	cmd_params.format	= "";
	cmd_params.cmd_arg	= arg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_cls_table_dump;
	mvapp_register_cli_cmd(&cmd_params);

	return 0;
}
