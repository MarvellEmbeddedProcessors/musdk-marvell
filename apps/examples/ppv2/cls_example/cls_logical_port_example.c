/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <string.h>
#include <stdio.h>
#include "cls_main_example.h"

/*
 * pp2_cls_logical_port_params_example()
 * example for logical port setting.
 * 1. set logical port
 * 2. set eth header to 'eth' type
 * 3. select that all TCP and UDP traffic will forward to musdk (other traffic to the kernel)
 */
int pp2_cls_logical_port_params_example(struct pp2_ppio_params *port_params, enum pp2_ppio_type *ppio_type)
{
	int i, j;
	struct pp2_ppio_log_port_params params;
	struct pp2_ppio_log_port_rule_params *rules_params;
	struct pp2_ppio_log_port_params *log_port_params =
					&port_params->specific_type_params.log_port_params;

	/* ppio type is logical port */
	*ppio_type = PP2_PPIO_T_LOG;

	/* use ETH header tag */
	port_params->eth_start_hdr = PP2_PPIO_HDR_ETH;

	memset(&params, 0, sizeof(struct pp2_ppio_log_port_params));

	/* handle all TCP & UDP traffic in musdk */
	params.proto_based_target.target = PP2_CLS_TARGET_LOCAL_PPIO;
	params.proto_based_target.num_proto_rule_sets = 1;
	params.proto_based_target.rule_sets[0].num_rules = 2;

	rules_params = &params.proto_based_target.rule_sets[0].rules[0];
	rules_params->rule_type = PP2_RULE_TYPE_PROTO;
	rules_params->u.proto_params.proto = MV_NET_PROTO_TCP;
	rules_params->u.proto_params.val = 0; /* indicates all tagged frames are to be matched */

	rules_params = &params.proto_based_target.rule_sets[0].rules[1];
	rules_params->rule_type = PP2_RULE_TYPE_PROTO;
	rules_params->u.proto_params.proto = MV_NET_PROTO_UDP;
	rules_params->u.proto_params.val = 0;

	log_port_params->proto_based_target.target = params.proto_based_target.target;
	log_port_params->proto_based_target.num_proto_rule_sets = params.proto_based_target.num_proto_rule_sets;
	for (i = 0; i < log_port_params->proto_based_target.num_proto_rule_sets; i++) {
		log_port_params->proto_based_target.rule_sets[i].num_rules =
					params.proto_based_target.rule_sets[i].num_rules;
		for (j = 0; j < params.proto_based_target.rule_sets[i].num_rules; j++)
			log_port_params->proto_based_target.rule_sets[i].rules[j] =
					params.proto_based_target.rule_sets[i].rules[j];
	}

	return 0;
}
