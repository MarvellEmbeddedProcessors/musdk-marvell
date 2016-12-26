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
#include "src/drivers/ppv2/cls/pp2_flow_rules.h"
#include "src/drivers/ppv2/cls/pp2_cls_db.h"
#include "cls_debug.h"

#define CLS_APP_PKT_OFFS			64
#define CLS_APP_DMA_MEM_SIZE			(4 * 1024 * 1024)
#define CLS_APP_PP2_MAX_NUM_TCS_PER_PORT	1
#define CLS_APP_PP2_MAX_NUM_QS_PER_TC		1
#define CLS_APP_MAX_NUM_TBL			10
#define CLS_APP_MAX_NUM_TBL_KEYS		10
#define CLS_APP_MAX_NUM_TBL_ACT			10
#define CLS_APP_MAX_NUM_RULES			10
#define CLS_APP_STR_SIZE_MAX			40
#define CLS_APP_KEY_SIZE_MAX			37
#define CLS_APP_PPIO_NAME_MAX			15
#define CLS_APP_KEY_MEM_SIZE_MAX	(PP2_CLS_TBL_MAX_NUM_FIELDS * CLS_APP_STR_SIZE_MAX)

#define CLS_DEBUG

/** Get rid of path in filename - only for unix-type paths using '/' */
#define CLS_DBG_NO_PATH(file_name) (strrchr((file_name), '/') ? \
			    strrchr((file_name), '/') + 1 : (file_name))
#define CLS_TEST_MODULE_NAME_MAX 10

struct pp2_ppio {
	struct pp2_port *port;
};

struct glob_arg {
	int			verbose;
	int			cli;
	char			port_name[CLS_APP_PPIO_NAME_MAX];
	struct pp2_hif		*hif;
	struct pp2_ppio		*ppio;
	int			num_pools;
	struct pp2_bpool	***pools;
	struct pp2_buff_inf	***buffs_inf;
	char			test_module[CLS_TEST_MODULE_NAME_MAX];
	int			test_number;
	uintptr_t		cpu_slot;
};

static struct glob_arg garg = {};

struct pp2_cls_tbl_key_db {
	u32			idx;
	struct pp2_cls_tbl_key	tbl_key[CLS_APP_MAX_NUM_TBL_KEYS];
};

struct pp2_cls_action_element {
	struct pp2_cls_tbl_action	tbl_action;
	char				ppio_name[CLS_APP_PPIO_NAME_MAX];
	struct pp2_cls_cos_desc		cos;
};

struct pp2_cls_action_db {
	u32				idx;
	struct pp2_cls_action_element	elem[CLS_APP_MAX_NUM_TBL_ACT];
};

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
	struct pp2_cls_tbl_key_db	key_db;
	struct pp2_cls_action_db	action_db;
	struct pp2_cls_table_db		table_db;
	struct pp2_cls_rule_key_db	rule_key_db;
};

static struct pp2_cls_db *cls_db;

static int app_find_pp_id(char *ppio_str, u32 *pp_id)
{
	char *pch;
	int val;

	pch = strchr(ppio_str, ':');
	val = *(pch + 1) - 0x30;
	if ((val < 0) || (val > PP2_NUM_PKT_PROC - 1))
		return -EINVAL;

	*pp_id = val;
	return 0;
}

static int app_find_ppio_id(char *ppio_str, u32 *ppio_id)
{
	char *pch;
	int val;

	pch = strchr(ppio_str, '-');
	val = *(pch + 1) - 0x30;
	if ((val < 0) || (val > PP2_NUM_ETH_PPIO - 1))
		return -EINVAL;

	*ppio_id = val;
	return 0;
}

static void app_print_horizontal_line(u32 char_count, const char *char_val)
{
	u32 cnt;

	for (cnt = 0; cnt < char_count; cnt++)
		printf("%s", char_val);
	printf("\n");
}

static int pp2_cls_cli_cls_key_add(void *arg, int argc, char *argv[])
{
	u32 idx;
	int key_size = -1;
	int num_fields = -1;
	u32 proto[PP2_CLS_TBL_MAX_NUM_FIELDS];
	u32 field[PP2_CLS_TBL_MAX_NUM_FIELDS];
	char *ret_ptr;
	int option;
	int long_index = 0;
	struct option long_options[] = {
		{"key_size", required_argument, 0, 'k'},
		{"num_fields", required_argument, 0, 'n'},
		{"field", required_argument, 0, 'f'},
		{"protocol", required_argument, 0, 'p'},
		{0, 0, 0, 0}
	};

	if ((argc > 25) || (argc <= 3)) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	/* every time starting getopt we should reset optind */
	optind = 0;
	/* Get parameters */
	option = getopt_long_only(argc, argv, "", long_options, &long_index);
	if (option == 'k') {
		key_size = strtoul(optarg, &ret_ptr, 0);
		if ((optarg == ret_ptr) || (key_size < 0) || (key_size > CLS_APP_KEY_SIZE_MAX)) {
			printf("parsing fail, wrong input for argv[1] - key_size\n");
			return -EINVAL;
		}
	} else {
		printf("parsing fail, wrong input for argv[1] - key_size\n");
		return -EINVAL;
	}
	option = getopt_long_only(argc, argv, "", long_options, &long_index);
	if (option == 'n') {
		num_fields = strtoul(optarg, &ret_ptr, 0);
		if ((optarg == ret_ptr) || (num_fields < 0) || (num_fields > PP2_CLS_TBL_MAX_NUM_FIELDS)) {
			printf("parsing fail, wrong input for argv[2] - num_fields\n");
			return -EINVAL;
		}
	} else {
		printf("parsing fail, wrong input for argv[2] - num_fields\n");
		return -EINVAL;
	}

	for (idx = 0; idx < num_fields; idx++) {
		option = getopt_long_only(argc, argv, "", long_options, &long_index);
		if (option == 'f') {
			proto[idx] = strtoul(optarg, &ret_ptr, 0);
			if ((optarg == ret_ptr) || (proto[idx] < 0) || (proto[idx] >= MV_NET_PROTO_LAST)) {
				printf("parsing fail, wrong input for argv[%d] - proto\n", 3 + (idx << 1));
				return -EINVAL;
			}
			option = getopt_long_only(argc, argv, "", long_options, &long_index);
			if (option == 'p') {
				field[idx] = strtoul(optarg, &ret_ptr, 0);
				if ((optarg == ret_ptr) || (field[idx] < 0) || (field[idx] >= MV_NET_IP6_F_NEXT_HDR)) {
					printf("parsing fail, wrong input for argv[%d] - field\n", 4 + (idx << 1));
					return -EINVAL;
				}
			}
		} else {
			printf("parsing fail, insert -s for key_size, and -n for number of fields\n");
			return -EINVAL;
		}
	}

	if (cls_db->key_db.idx < CLS_APP_MAX_NUM_TBL_KEYS) {
		struct pp2_cls_tbl_key *key_ptr = &cls_db->key_db.tbl_key[cls_db->key_db.idx];

		key_ptr->key_size = key_size;
		key_ptr->num_fields = num_fields;
		for (idx = 0; idx < num_fields; idx++) {
			key_ptr->proto_field[idx].proto = proto[idx];
			key_ptr->proto_field[idx].field.eth = field[idx];
		}
		cls_db->key_db.idx++;
		printf("OK\n");
	} else {
		printf("FAIL: keys table is full: %d\n", cls_db->key_db.idx);
	}

	return 0;
}

static int pp2_cls_cli_cls_action_add(void *arg, int argc, char *argv[])
{
	char name[CLS_APP_PPIO_NAME_MAX];
	u32 action_type;
	u32 traffic_class_num;
	char *ret_ptr;
	int rc;
	int option;
	int long_index = 0;
	struct option long_options[] = {
		{"interface", required_argument, 0, 'i'},
		{"action_type", required_argument, 0, 'a'},
		{"traffic_class", required_argument, 0, 't'},
		{0, 0, 0, 0}
	};

	if (argc != 7) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	/* every time starting getopt we should reset optind */
	optind = 0;

	option = getopt_long_only(argc, argv, "", long_options, &long_index);
	if (option == 'i') {
		/* Get parameters */
		rc = sscanf(optarg, "%s", &name[0]);
		if (rc <= 0) {
			printf("parsing fail, wrong input for argv[1] - interface_name\n");
			return -EINVAL;
		}
	} else {
		printf("parsing fail\n");
		return -EINVAL;
	}

	option = getopt_long_only(argc, argv, "", long_options, &long_index);
	if (option == 'a') {
		action_type = strtoul(optarg, &ret_ptr, 0);
		if ((optarg == ret_ptr) || (action_type < 0) || (action_type > PP2_CLS_TBL_ACT_DONE)) {
			printf("parsing fail, wrong input for argv[2] - action_type %d\n", action_type);
			return -EINVAL;
		}
	} else {
		printf("parsing fail\n");
		return -EINVAL;
	}
	option = getopt_long_only(argc, argv, "", long_options, &long_index);
	if (option == 't') {
		traffic_class_num = strtoul(optarg, &ret_ptr, 0);
		if ((optarg == ret_ptr) || (traffic_class_num < 0) || (traffic_class_num >= PP2_PPIO_MAX_NUM_TCS)) {
			printf("parsing fail, wrong input for argv[3] - traffic_class_num\n");
			return -EINVAL;
		}
	} else {
		printf("parsing fail\n");
		return -EINVAL;
	}

	if (cls_db->action_db.idx < CLS_APP_MAX_NUM_TBL_ACT) {
		struct pp2_cls_action_element *act_ptr = &cls_db->action_db.elem[cls_db->action_db.idx];

		act_ptr->tbl_action.type = action_type;
		act_ptr->cos.ppio = garg.ppio;
		act_ptr->cos.tc = traffic_class_num;
		act_ptr->tbl_action.cos = &act_ptr->cos;
		printf("found port: %s, %d %d\n", name, traffic_class_num, act_ptr->tbl_action.cos->tc);
		strcpy(&act_ptr->ppio_name[0], name);
		cls_db->action_db.idx++;
		printf("OK\n");
	} else {
		printf("FAIL: actions table is full: %d\n", cls_db->action_db.idx);
	}

	return 0;
}

static int pp2_cls_cli_table_add(void *arg, int argc, char *argv[])
{
	u32 idx;
	int engine_type = -1;
	int key_idx = -1;
	int action_idx = -1;
	char *ret_ptr;
	struct pp2_cls_tbl_params *tbl_ptr;
	struct pp2_cls_tbl_key *key_ptr;
	struct pp2_cls_tbl_action *act_ptr;
	int i, option;
	int long_index = 0;
	struct option long_options[] = {
		{"engine_type", required_argument, 0, 'e'},
		{"key_index", required_argument, 0, 'k'},
		{"action_index", required_argument, 0, 'a'},
		{0, 0, 0, 0}
	};

	if (argc != 7) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	optind = 0;/* every time starting getopt we should reset optind */
	for (i = 0; i < 3; i++) {
		/* Get parameters */
		option = getopt_long_only(argc, argv, "", long_options, &long_index);
		switch (option) {
		case 'e':
			engine_type = strtoul(optarg, &ret_ptr, 0);
			if ((optarg == ret_ptr) || (engine_type < 0) || (engine_type >= PP2_CLS_TBL_MASKABLE)) {
				printf("parsing fail, wrong input for argv[1] - engine_type\n");
				return -EINVAL;
			}
			break;
		case 'k':
			key_idx = strtoul(optarg, &ret_ptr, 0);
			if ((optarg == ret_ptr) || (key_idx < 0) || (key_idx >= cls_db->key_db.idx)) {
				printf("parsing fail, wrong input for argv[3] - key_idx\n");
				return -EINVAL;
			}
			break;
		case 'a':
			action_idx = strtoul(optarg, &ret_ptr, 0);
			if ((optarg == ret_ptr) || (action_idx < 0) || (action_idx >= cls_db->action_db.idx)) {
				printf("parsing fail, wrong input for argv[4] - action_idx\n");
				return -EINVAL;
			}
			break;
		default:
			printf("parsing fail, wrong input\n");
			return -EINVAL;
		}
	}
	/* check if all the fields are initialized */
	if (engine_type < 0) {
		printf("parsing fail, invalid engine_type\n");
		return -EINVAL;
	}

	if (key_idx < 0) {
		printf("parsing fail, invalid key_index\n");
		return -EINVAL;
	}

	if (action_idx < 0) {
		printf("parsing fail, invalid action_index\n");
		return -EINVAL;
	}

	tbl_ptr = &cls_db->table_db.elem[cls_db->table_db.idx].tbl_params;
	key_ptr = &cls_db->key_db.tbl_key[key_idx];
	act_ptr = &cls_db->action_db.elem[action_idx].tbl_action;

	tbl_ptr->type = engine_type;
	tbl_ptr->max_num_rules = MVPP2_CLS_DEF_FLOW_LEN;
	tbl_ptr->key.key_size = key_ptr->key_size;
	tbl_ptr->key.num_fields = key_ptr->num_fields;
	for (idx = 0; idx < tbl_ptr->key.num_fields; idx++) {
		tbl_ptr->key.proto_field[idx].proto = key_ptr->proto_field[idx].proto;
		tbl_ptr->key.proto_field[idx].field.eth = key_ptr->proto_field[idx].field.eth;
	}
	tbl_ptr->default_act.type = act_ptr->type;
	tbl_ptr->default_act.cos = act_ptr->cos;
	tbl_ptr->default_act.cos->tc = act_ptr->cos->tc;
	strcpy(&cls_db->table_db.elem[cls_db->table_db.idx].ppio_name[0],
	       &cls_db->action_db.elem[action_idx].ppio_name[0]);
	if (!pp2_cls_tbl_init(tbl_ptr, &cls_db->table_db.elem[cls_db->table_db.idx].tbl)) {
		printf("OK\n");
		cls_db->table_db.idx++;
	} else {
		printf("FAIL\n");
	}

	return 0;
}

static int pp2_cls_cli_cls_rule_key_add(void *arg, int argc, char *argv[])
{
	u32 idx;
	int rc;
	u32 num_fields;
	u32 key_size[PP2_CLS_TBL_MAX_NUM_FIELDS];
	u8 key[PP2_CLS_TBL_MAX_NUM_FIELDS][CLS_APP_STR_SIZE_MAX] = {0};
	u8 mask[PP2_CLS_TBL_MAX_NUM_FIELDS][CLS_APP_STR_SIZE_MAX] = {0};
	char *ret_ptr;
	int option = 0;
	int long_index = 0;
	struct option long_options[] = {
		{"num_fields", required_argument, 0, 'n'},
		{"size", required_argument, 0, 's'},
		{"key", required_argument, 0, 'k'},
		{"mask", required_argument, 0, 'm'},
		{"protocol", no_argument, 0, 'p'},
		{0, 0, 0, 0}
	};

	if (argc < 3 || argc > 32) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	/* every time starting getopt we should reset optind */
	optind = 0;
	/* Get parameters */
	option = getopt_long_only(argc, argv, "", long_options, &long_index);
	if (option == 'n') {
		num_fields = strtoul(optarg, &ret_ptr, 0);
		if ((argv[1] == ret_ptr) || (num_fields < 0) || (num_fields > PP2_CLS_TBL_MAX_NUM_FIELDS)) {
			printf("parsing fail, wrong input for argv[1] - num_fields\n");
			return -EINVAL;
		}
	} else {
		printf("parsing fail, wrong input for argv[1] - num_fields\n");
		return -EINVAL;
	}

	idx = 0;
	while (idx < num_fields) {
		option = getopt_long_only(argc, argv, "", long_options, &long_index);
		switch (option) {
		case 's':
			key_size[idx] = strtoul(optarg, &ret_ptr, 0);
			if ((argv[2 + (idx * 3)] == ret_ptr) || (key_size[idx] < 0) ||
			    (key_size[idx] > CLS_APP_MAX_NUM_RULES)) {
				printf("parsing fail, wrong input for argv[%d] - key_size\n", 2 + (idx * 3));
				return -EINVAL;
			}
			option = getopt_long_only(argc, argv, "", long_options, &long_index);
			if (option == 'k') {
				if (strncmp(optarg, "0x", 2) == 0) {
					rc = sscanf(optarg + 2, "%s", &key[idx][0]);
					if (rc <= 0) {
						printf("parsing fail, wrong input for argv[%d] - key_type\n",
							3 + (idx * 3));
						return -EINVAL;
					}
				} else {
					rc = sscanf(optarg, "%s", &key[idx][0]);
					if (rc <= 0) {
						printf("parsing fail, wrong input for argv[%d] - key_type\n",
							3 + (idx * 3));
						return -EINVAL;
					}
				}
				option = getopt_long_only(argc, argv, "", long_options, &long_index);
				if (option == 'm') {
					rc = sscanf(optarg, "%s", &mask[idx][0]);
					if (rc <= 0) {
						printf("parsing fail, wrong input for argv[%d] - key_mask\n",
							4 + (idx * 3));
						return -EINVAL;
					}
				} else if (option == 'p') {
					sprintf((char *)&mask[idx][0], "%d", 0);
				} else {
					printf("parsing fail, wrong input, line = %d\n", __LINE__);
					return -EINVAL;
				}
			} else {
				printf("parsing fail, wrong input, line = %d\n", __LINE__);
				return -EINVAL;
			}
			break;
		default:
			printf("parsing fail, wrong input, line = %d\n", __LINE__);
			return -EINVAL;
		}
		idx++;
	}
	printf("\n");

	if (cls_db->rule_key_db.idx < CLS_APP_MAX_NUM_RULES) {
		u32 field_idx;
		struct pp2_cls_rule_key_element *rule_key_ptr = &cls_db->rule_key_db.elem[cls_db->rule_key_db.idx];

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
		printf("OK\n");
	} else {
		printf("FAIL: rule_key table is full: %d\n", cls_db->rule_key_db.idx);
	}
	return 0;
}

static int pp2_cls_cli_cls_rule_add(void *arg, int argc, char *argv[])
{
	int tbl_idx = -1;
	int rule_idx = -1;
	int action_idx = -1;
	char *ret_ptr;
	struct pp2_cls_tbl *tbl;
	struct pp2_cls_tbl_rule	*rule;
	struct pp2_cls_tbl_action *action;
	int option = 0;
	int i;
	int long_index = 0;
	struct option long_options[] = {
		{"table_index", required_argument, 0, 't'},
		{"rule_index", required_argument, 0, 'r'},
		{"action_index", required_argument, 0, 'a'},
		{0, 0, 0, 0}
	};

	if (argc != 7) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	/* every time starting getopt we should reset optind */
	optind = 0;

	/* Get parameters */
	for (i = 0; i < 3; i++) {
		option = getopt_long_only(argc, argv, "", long_options, &long_index);
		switch (option) {
		case 't':
			tbl_idx = strtoul(optarg, &ret_ptr, 0);
			if ((optarg == ret_ptr) || (tbl_idx < 0) || (tbl_idx >= cls_db->table_db.idx)) {
				printf("parsing fail, wrong input for argv[1] - tbl_idx\n");
				return -EINVAL;
			}
			break;
		case 'r':
			rule_idx = strtoul(optarg, &ret_ptr, 0);
			if ((optarg == ret_ptr) || (rule_idx < 0) || (rule_idx >= cls_db->rule_key_db.idx)) {
				printf("parsing fail, wrong input for argv[2] - rule_idx\n");
				return -EINVAL;
			}
			break;
		case 'a':
			action_idx = strtoul(optarg, &ret_ptr, 0);
			if ((optarg == ret_ptr) || (action_idx < 0) || (action_idx >= cls_db->action_db.idx)) {
				printf("parsing fail, wrong input for argv[3] - action_idx\n");
				return -EINVAL;
			}
			break;
		default:
			printf("parsing fail, wrong input for argv[3] - action_idx\n");
			return -EINVAL;
		}
	}

	/* check if all the fields are initialized */
	if (tbl_idx < 0) {
		printf("parsing fail, invalid table_index\n");
		return -EINVAL;
	}
	if (rule_idx < 0) {
		printf("parsing fail, invalid rule_index\n");
		return -EINVAL;
	}
	if (action_idx < 0) {
		printf("parsing fail, invalid action_index\n");
		return -EINVAL;
	}

	tbl = cls_db->table_db.elem[tbl_idx].tbl;
	rule = &cls_db->rule_key_db.elem[rule_idx].rule_key;
	action = &cls_db->action_db.elem[action_idx].tbl_action;
	if (!pp2_cls_tbl_add_rule(tbl, rule, action))
		printf("OK\n");
	else
		printf("FAIL: unable to add rule to table index: %d\n", tbl_idx);
	return 0;
}

static int pp2_cls_cli_cls_rule_modify(void *arg, int argc, char *argv[])
{
	int tbl_idx = -1;
	int rule_idx = -1;
	int action_idx = -1;
	char *ret_ptr;
	struct pp2_cls_tbl *tbl;
	struct pp2_cls_tbl_rule	*rule;
	struct pp2_cls_tbl_action *action;
	int i, option = 0;
	int long_index = 0;
	struct option long_options[] = {
		{"table_index", required_argument, 0, 't'},
		{"rule_index", required_argument, 0, 'r'},
		{"action_index", required_argument, 0, 'a'},
		{0, 0, 0, 0}
	};

	if (argc != 7) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	/* every time starting getopt we should reset optind */
	optind = 0;

	/* Get parameters */
	for (i = 0; i < 3; i++) {
		option = getopt_long_only(argc, argv, "", long_options, &long_index);
		switch (option) {
		case 't':
			tbl_idx = strtoul(optarg, &ret_ptr, 0);
			if ((optarg == ret_ptr) || (tbl_idx < 0) || (tbl_idx >= cls_db->table_db.idx)) {
				printf("parsing fail, wrong input for argv[1] - tbl_idx\n");
				return -EINVAL;
			}
			break;
		case 'r':
			rule_idx = strtoul(optarg, &ret_ptr, 0);
			if ((optarg == ret_ptr) || (rule_idx < 0) || (rule_idx >= cls_db->rule_key_db.idx)) {
				printf("parsing fail, wrong input for argv[2] - rule_idx\n");
				return -EINVAL;
			}
			break;
		case 'a':
			action_idx = strtoul(optarg, &ret_ptr, 0);
			if ((optarg == ret_ptr) || (action_idx < 0) || (action_idx >= cls_db->action_db.idx)) {
				printf("parsing fail, wrong input for argv[3] - action_idx\n");
				return -EINVAL;
			}
			break;
		default:
			printf("parsing fail, wrong input for argv[3] - action_idx\n");
			return -EINVAL;
		}
	}

	/* check if all the fields are initialized */
	if (action_idx < 0) {
		printf("parsing fail, invalid action_index\n");
		return -EINVAL;
	}

	if (rule_idx < 0) {
		printf("parsing fail, invalid rule_index\n");
		return -EINVAL;
	}

	if (tbl_idx < 0) {
		printf("parsing fail, invalid table_index\n");
		return -EINVAL;
	}

	tbl = cls_db->table_db.elem[tbl_idx].tbl;
	rule = &cls_db->rule_key_db.elem[rule_idx].rule_key;
	action = &cls_db->action_db.elem[action_idx].tbl_action;
	if (!pp2_cls_tbl_modify_rule(tbl, rule, action))
		printf("OK\n");
	else
		printf("FAIL: unable to modify rule to table index: %d\n", tbl_idx);
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
	for (i = 0; i < 2; i++) {
		option = getopt_long_only(argc, argv, "", long_options, &long_index);
		switch (option) {
		case 't':
			tbl_idx = strtoul(optarg, &ret_ptr, 0);
			if ((optarg == ret_ptr) || (tbl_idx < 0) || (tbl_idx >= cls_db->table_db.idx)) {
				printf("parsing fail, wrong input for argv[1] - tbl_idx\n");
				return -EINVAL;
			}
			break;
		case 'r':
			rule_idx = strtoul(optarg, &ret_ptr, 0);
			if ((optarg == ret_ptr) || (rule_idx < 0) || (rule_idx >= cls_db->rule_key_db.idx)) {
				printf("parsing fail, wrong input for argv[2] - rule_idx\n");
				return -EINVAL;
			}
			break;
		default:
			printf("parsing fail, wrong input for argv[3] - action_idx\n");
			return -EINVAL;
		}
	}

	/* check if all the fields are initialized */
	if (rule_idx < 0) {
		printf("parsing fail, invalid rule_index\n");
		return -EINVAL;
	}

	if (tbl_idx < 0) {
		printf("parsing fail, invalid table_index\n");
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

static int pp2_cls_cli_cls_key_dump(void *arg, int argc, char *argv[])
{
	u32 idx1, idx2;

	printf("total indexes: %d\n", cls_db->key_db.idx);
	printf("| idx | key_size | num_fields | 1 proto,field| 2 proto, field| 3 proto,field| 4 proto,field|\n");

	for (idx1 = 0; idx1 < cls_db->key_db.idx; idx1++) {
		struct pp2_cls_tbl_key *key_ptr = &cls_db->key_db.tbl_key[idx1];

		printf("|%5d| %8d | %10d |", idx1, key_ptr->key_size, key_ptr->num_fields);
		for (idx2 = 0; idx2 < key_ptr->num_fields; idx2++)
			printf("   %5d,%5d|", key_ptr->proto_field[idx2].proto, key_ptr->proto_field[idx2].field.eth);
	printf("\n");
	}
	printf("\nOK\n");

	return 0;
}

static int pp2_cls_cli_cls_act_dump(void *arg, int argc, char *argv[])
{
	u32 idx;

	printf("total indexes: %d\n", cls_db->action_db.idx);
	printf("| idx |     port   | type | tc_num |\n");
	for (idx = 0; idx < cls_db->action_db.idx; idx++) {
		struct pp2_cls_action_element *act_ptr = &cls_db->action_db.elem[idx];

		printf("| %3d | %10s | %4d | %6d |\n", idx, act_ptr->ppio_name, act_ptr->tbl_action.type,
		       act_ptr->tbl_action.cos->tc);
	}
	printf("OK\n");
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

static int main_loop(void *arg, volatile int *running)
{
	struct glob_arg *garg = (struct glob_arg *)arg;
	struct pp2_ppio *ppio = garg->ppio;
	struct pp2_port *port = ppio->port;

	garg->cpu_slot = port->cpu_slot;

	if (!garg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}
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
	pp2_params.hif_reserved_map = 0;
	pp2_params.bm_pool_reserved_map = 0;
	pp2_params.ppios[0][0].is_enabled = 1;
	pp2_params.ppios[0][0].first_inq = 0;
	err = pp2_init(&pp2_params);
	if (err)
		return err;

	pr_info("done\n");
	return 0;
}

static int init_local_modules(struct glob_arg *garg)
{
	struct pp2_ppio_params		port_params;
	struct pp2_ppio_inq_params	inq_params;
	int				i, j, err;
	struct bpool_inf		infs[] = MVAPPS_BPOOLS_INF;
	u32				pp_id = 0, ppio_id = 0;

	pr_info("Local initializations ... ");

	err = app_hif_init(&garg->hif);
	if (err)
		return err;

	garg->num_pools = ARRAY_SIZE(infs);
	err = app_build_all_bpools(&garg->pools, &garg->buffs_inf, garg->num_pools, infs, garg->hif);
	if (err)
		return err;

	app_find_ppio_id(garg->port_name, &ppio_id);
	app_find_pp_id(garg->port_name, &pp_id);
	pr_info("interface: %s, pp_id: %d, ppio_id: %d\n", garg->port_name, pp_id, ppio_id);

	memset(&port_params, 0, sizeof(port_params));
	port_params.match = garg->port_name;
	port_params.type = PP2_PPIO_T_NIC;
	port_params.inqs_params.num_tcs = CLS_APP_PP2_MAX_NUM_TCS_PER_PORT;
	for (i = 0; i < port_params.inqs_params.num_tcs; i++) {
		port_params.inqs_params.tcs_params[0].pkt_offset = CLS_APP_PKT_OFFS >> 2;
		port_params.inqs_params.tcs_params[0].num_in_qs = CLS_APP_PP2_MAX_NUM_QS_PER_TC;
		/* TODO: we assume here only one Q per TC; change it! */
		inq_params.size = MVAPPS_Q_SIZE;
		port_params.inqs_params.tcs_params[0].inqs_params = &inq_params;
		for (j = 0; j < garg->num_pools; j++)
			port_params.inqs_params.tcs_params[0].pools[j] = garg->pools[0][j];
	}
	port_params.outqs_params.num_outqs = CLS_APP_PP2_MAX_NUM_TCS_PER_PORT;
	for (i = 0; i < port_params.outqs_params.num_outqs; i++) {
		port_params.outqs_params.outqs_params[0].size = MVAPPS_Q_SIZE;
		port_params.outqs_params.outqs_params[0].weight = 1;
	}
	err = pp2_ppio_init(&port_params, &garg->ppio);
	if (err)
		return err;
	if (!garg->ppio) {
		pr_err("PP-IO init failed!\n");
		return -EIO;
	}

	err = pp2_ppio_enable(garg->ppio);
	if (err)
		return err;
	err = pp2_ppio_set_uc_promisc(garg->ppio, 1);
	if (err)
		return err;

	/* cls memory allocations */
	cls_db = malloc(sizeof(*cls_db));
	if (!cls_db) {
		pr_err("no mem for cls_db array!\n");
		return -ENOMEM;
	}

	pr_info("done\n");
	return 0;
}

static void destroy_local_modules(struct glob_arg *garg)
{
	int	i, j;

	if (garg->ppio) {
		pp2_ppio_disable(garg->ppio);
		pp2_ppio_deinit(garg->ppio);
	}

	if (garg->pools) {
		for (i = 0; i < PP2_SOC_NUM_PACKPROCS; i++) {
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
		for (i = 0; i < PP2_SOC_NUM_PACKPROCS; i++) {
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
	cmd_params.name		= "cls_key_add";
	cmd_params.desc		= "add a classifier key to database";
	cmd_params.format	= "--key_size --num_fields --protocol --field...\n"
				  "\t\t\t\t--key_size	      (dec)size of the key in bytes (13 for 5 tuples)\n"
				  "\t\t\t\t--num_fields	      (dec)number of fields in key\n"
				  "\t\t\t\t--protocol --field	   (dec)the following protocols, fields are defined:\n"
				  "\t\t\t\t			   1 0 - ethernet, source address\n"
				  "\t\t\t\t			   1 1 - ethernet, destination address\n"
				  "\t\t\t\t			   1 2 - ethernet, type\n"
				  "\t\t\t\t			   2 0 - vlan, priority\n"
				  "\t\t\t\t			   2 1 - vlan, id\n"
				  "\t\t\t\t			   2 2 - vlan, tci\n"
				  "\t\t\t\t			   3 0 - pppoe\n"
				  "\t\t\t\t			   4 0 - ip\n"
				  "\t\t\t\t			   5 0 - ipv4, tos\n"
				  "\t\t\t\t			   5 1 - ipv4, souce address\n"
				  "\t\t\t\t			   5 2 - ipv4, destination address\n"
				  "\t\t\t\t			   5 3 - ipv4, proto\n"
				  "\t\t\t\t			   6 0 - ipv6, tc\n"
				  "\t\t\t\t			   6 1 - ipv6, source address\n"
				  "\t\t\t\t			   6 2 - ipv6, destination address\n"
				  "\t\t\t\t			   6 3 - ipv6, flow\n"
				  "\t\t\t\t			   6 4 - ipv6, next header\n"
				  "\t\t\t\t			   7 0 - layer4, source port\n"
				  "\t\t\t\t			   7 1 - layer4, destination port\n"
				  "\t\t\t\t			   7 2 - layer4, checksum\n"
				  "\t\t\t\t			   8 0 - tcp, source port\n"
				  "\t\t\t\t			   8 1 - tcp, destination port\n"
				  "\t\t\t\t			   8 2 - tcp, checksum\n"
				  "\t\t\t\t			   9 0 - udp, source port\n"
				  "\t\t\t\t			   9 1 - udp, destination port\n"
				  "\t\t\t\t			   9 2 - udp, checksum\n"
				  "\t\t\t\t			   10 0 - icmp\n"
				  "\t\t\t\t			   11 0 - arp\n";
	cmd_params.cmd_arg	= garg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_cls_key_add;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_action_add";
	cmd_params.desc		= "add a classifier action to database";
	cmd_params.format	= "--interface --action_type --traffic_class\n"
				  "\t\t\t\t--interface       ppio-0:0 - A7040/A8040 10Gb interface 1\n"
				  "\t\t\t\t                  ppio-0:1 - A7040/A8040  1Gb interface 1\n"
				  "\t\t\t\t                  ppio-0:2 - A7040        1Gb interface 2\n"
				  "\t\t\t\t                  ppio-1:0 - A8040       10Gb interface 2\n"
				  "\t\t\t\t                  ppio-1:1 - A8040        1Gb interface 2\n"
				  "\t\t\t\t--action_type     (dec) 0: drop, 1: done\n"
				  "\t\t\t\t--traffic_class   (dec) 1..8\n";
	cmd_params.cmd_arg	= garg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_cls_action_add;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_tbl_init";
	cmd_params.desc		= "create a classifier table according to key and default action";
	cmd_params.format	= "--engine_type --key_index --action_index\n"
				  "\t\t\t\t--engine_type	(dec) 0: exact_match, 1: maskable\n"
				  "\t\t\t\t--key_index		(dec) index to pre-defined key\n"
				  "\t\t\t\t--action_index	(dec) index to pre-defined default_action\n";
	cmd_params.cmd_arg	= garg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_table_add;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_rule_key_add";
	cmd_params.desc		= "add a classifier rule key";
	cmd_params.format	= "--num_fields --size --key --mask (or) --protocol...\n"
				  "\t\t\t\t--num_fields		(dec) 1..4\n"
				  "\t\t\t\t--size		(dec) size in bytes of the key\n"
				  "\t\t\t\t--key		(dec or hex) key\n"
				  "\t\t\t\t			   i.e ipv4: 192.168.10.5\n"
				  "\t\t\t\t			   i.e ipv6: 2605:2700:0:3::4713:93e3\n"
				  "\t\t\t\t			   i.e port: 0x1234\n"
				  "\t\t\t\t			   i.e udp: 17(IPPROTO_UDP)\n"
				  "\t\t\t\t			   i.e tcp: 6(IPPROTO_TCP)\n"
				  "\t\t\t\t--mask		(hex) mask for the key (if maskable is used)\n"
				  "\t\t\t\t--protocol		(dec) needed only while using protocol key\n";
	cmd_params.cmd_arg	= garg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_cls_rule_key_add;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_rule_add";
	cmd_params.desc		= "add a classifier rule to existing table";
	cmd_params.format	= "--table_index --rule_index --action_index\n"
				  "\t\t\t\t--table_index	(dec) index to existing table\n"
				  "\t\t\t\t--rule_index		(dec) index to existing rule in database\n"
				  "\t\t\t\t--action_index	(dec) index to existing action in database\n";
	cmd_params.cmd_arg	= garg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_cls_rule_add;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_rule_modify";
	cmd_params.desc		= "modify a classifier rule in specified table";
	cmd_params.format	= "--table_index --rule_index --action_index\n"
				  "\t\t\t\t--table_index	(dec) index to existing table\n"
				  "\t\t\t\t--rule_index		(dec) index to existing rule in database\n"
				  "\t\t\t\t--action_index	(dec) index to existing action in database\n";
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
	cmd_params.name		= "cls_key_dump";
	cmd_params.desc		= "display classifier defined keys";
	cmd_params.format	= "";
	cmd_params.cmd_arg	= garg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_cls_key_dump;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_action_dump";
	cmd_params.desc		= "display classifier defined default_actions";
	cmd_params.format	= "";
	cmd_params.cmd_arg	= garg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_cls_act_dump;
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

static int register_cli_cmds(struct glob_arg *garg)
{
	struct pp2_ppio *ppio = garg->ppio;
	struct pp2_port *port = ppio->port;

	if (!garg->cli)
		return -EFAULT;

	garg->cpu_slot = port->cpu_slot;

	register_cli_cls_api_cmds(garg);
#ifdef CLS_DEBUG
	register_cli_cls_cmds(garg->cpu_slot);
	register_cli_c3_cmds(garg->cpu_slot);
#endif
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

static int init_local(void *arg, int id, void **larg)
{
	struct glob_arg *garg = (struct glob_arg *)arg;

	if (!garg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	*larg = garg;
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
	"  E.g. %s -i eth0,eth1 -c 1\n"
	    "\n"
	    "Mandatory OPTIONS:\n"
		"\t-i, --interface <port-interface>\n"
		"\t	  ppio-0:0 - A7040/A8040 10Gb interface 1\n"
		"\t	  ppio-0:1 - A7040/A8040  1Gb interface 1\n"
		"\t	  ppio-0:2 - A7040        1Gb interface 2\n"
		"\t	  ppio-1:0 - A8040       10Gb interface 2\n"
		"\t	  ppio-1:1 - A8040       1Gb interface 2\n"
		"\n"
#ifdef CLS_DEBUG
	    "Optional OPTIONS:\n"
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
		{0, 0, 0, 0}
	};

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
			snprintf(garg->port_name, sizeof(garg->port_name), "%s", optarg);
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
	if (!garg->port_name[0]) {
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

	memset(&mvapp_params, 0, sizeof(mvapp_params));
	mvapp_params.use_cli		= garg.cli;
	mvapp_params.num_cores		= 1;
	mvapp_params.global_arg		= (void *)&garg;
	mvapp_params.init_global_cb	= init_global;
	mvapp_params.deinit_global_cb	= deinit_global;
	mvapp_params.init_local_cb	= init_local;
	mvapp_params.deinit_local_cb	= deinit_local;
	mvapp_params.main_loop_cb	= main_loop;
	return mvapp_go(&mvapp_params);
}
