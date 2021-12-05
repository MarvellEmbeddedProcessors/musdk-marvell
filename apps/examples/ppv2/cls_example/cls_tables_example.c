/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <string.h>
#include <stdio.h>
#include "mvapp.h"
#include "cls_main_example.h"

#define CLS_APP_STR_SIZE_MAX			40
#define CLS_APP_MAX_NUM_OF_RULES		20

static struct	pp2_cls_tbl		*flow_tbl;

/*
 * pp2_cls_cli_add_5_tuple_table()
 * an example for init 5-tuple table with exact match engine
 * 5-tuple: ip4 src, ip4 dst, l4 src, l4 dst and ip4 proto
 */
int pp2_cls_add_5_tuple_table(struct port_desc *ports_desc)
{
	struct pp2_cls_tbl_params *tbl_params;
	u32 idx = 0;
	u32 proto[PP2_CLS_TBL_MAX_NUM_FIELDS];
	u32 field[PP2_CLS_TBL_MAX_NUM_FIELDS];
	int num_fields = PP2_CLS_TBL_MAX_NUM_FIELDS;
	int engine_type = PP2_CLS_TBL_EXACT_MATCH;
	int key_size = 13; /* ip4_src + ip4_dst + l4_src + l4_dst + proto = 4 + 4 + 2 + 2 + 1 */
	int traffic_class = 0;
	int action_type = PP2_CLS_TBL_ACT_DONE;

	/* ip4 src */
	proto[0] = MV_NET_PROTO_IP4;
	field[0] = MV_NET_IP4_F_SA;
	/* ip4 dst */
	proto[1] = MV_NET_PROTO_IP4;
	field[1] = MV_NET_IP4_F_DA;
	/* l4 src */
	proto[2] = MV_NET_PROTO_L4;
	field[2] = MV_NET_L4_F_SP;
	/* l4 dst */
	proto[3] = MV_NET_PROTO_L4;
	field[3] = MV_NET_L4_F_DP;
	/* ip4 proto */
	proto[4] = MV_NET_PROTO_IP4;
	field[4] = MV_NET_IP4_F_PROTO;

	pr_debug("num_fields = %d, key_size = %d\n", num_fields, key_size);

	tbl_params = malloc(sizeof(*tbl_params));
	if (!tbl_params) {
		pr_err("%s no mem for new table!\n", __func__);
		return -ENOMEM;
	}
	memset(tbl_params, 0, sizeof(*tbl_params));

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
		free(tbl_params);
		pr_err("%s(%d) no mem for pp2_cls_cos_desc!\n", __func__, __LINE__);
		return -ENOMEM;
	}

	tbl_params->default_act.type = action_type;
	tbl_params->default_act.cos->ppio = ports_desc->ppio;
	tbl_params->default_act.cos->tc = traffic_class;

	if (!pp2_cls_tbl_init(tbl_params, &flow_tbl))
		printf("OK\n");
	else
		printf("FAIL\n");

	return 0;
}

/*
 * pp2_cls_example_rule_key()
 * add rule key with the 5-tuple table
 * 1. set tc = 0
 * 2. select 5-tuple table
 * 3. add the following key:	ip4_src = 1.1.1.10
 *				ip4_dst = 192.168.1.10
 *				l4_src = 0x400
 *				l4_dst = 0x400
 *				protocol = UDP
 */
int pp2_cls_example_rule_key(struct port_desc *ports_desc)
{
	u32 idx = 0;
	int rc;
	struct pp2_cls_tbl_rule *rule;
	struct pp2_cls_tbl_action *action;
	u32 key_size[PP2_CLS_TBL_MAX_NUM_FIELDS];
	u8 key[PP2_CLS_TBL_MAX_NUM_FIELDS][CLS_APP_STR_SIZE_MAX] = {0};
	u8 mask[PP2_CLS_TBL_MAX_NUM_FIELDS][CLS_APP_STR_SIZE_MAX] = {0};
	int action_type = PP2_CLS_TBL_ACT_DONE;
	u32 num_fields = PP2_CLS_TBL_MAX_NUM_FIELDS;
	int traffic_class = 0;

	if (!flow_tbl) {
		printf("fail, flow table should be initilaize first\n");
		return -EINVAL;
	}

	key_size[0] = 4;
	strcpy((char *)&key[0], "1.1.1.10");
	strcpy((char *)&mask[0], "0xffffffff");

	key_size[1] = 4;
	strcpy((char *)&key[1], "192.168.1.10");
	strcpy((char *)&mask[1], "0xffffffff");

	key_size[2] = 2;
	strcpy((char *)&key[2], "0x400");
	strcpy((char *)&mask[2], "0xffff");

	key_size[3] = 2;
	strcpy((char *)&key[3], "0x400");
	strcpy((char *)&mask[3], "0xffff");

	/* UDP protocol */
	key_size[4] = 1;
	strcpy((char *)&key[4], "17");
	strcpy((char *)&mask[4], "0xff");

	rule = malloc(sizeof(*rule));
	if (!rule)
		goto rule_add_fail;

	rule->num_fields = num_fields;
	for (idx = 0; idx < num_fields; idx++) {
		rule->fields[idx].size = key_size[idx];
		rule->fields[idx].key = &key[idx][0];
		rule->fields[idx].mask = &mask[idx][0];
	}

	action = malloc(sizeof(*action));
	if (!action)
		goto rule_add_fail1;

	action->cos = malloc(sizeof(*action->cos));
	if (!action->cos)
		goto rule_add_fail2;

	action->type = action_type;
	action->cos->tc = traffic_class;
	action->cos->ppio = ports_desc->ppio;

	rc = pp2_cls_tbl_add_rule(flow_tbl, rule, action);
	free(action->cos);
	free(action);

	if (!rc)
		printf("OK\n");
	else
		printf("FAIL\n");
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

