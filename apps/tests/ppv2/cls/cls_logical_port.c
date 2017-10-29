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
#include "mv_pp2_ppio.h"
#include "../pp2_tests_main.h"

int pp2_cls_cli_ppio_tag_mode(struct pp2_ppio_params *port_params, int argc, char *argv[])
{
	int i, option;
	int long_index = 0;
	enum pp2_ppio_eth_start_hdr eth_hdr = -1;
	struct option long_options[] = {
		{"eth", no_argument, 0, 'n'},
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

int pp2_cls_logical_port_params(struct pp2_ppio_params *port_params, int argc, char *argv[])
{
	char *ret_ptr;
	int i, j, option;
	int long_index = 0;
	u32 r_idx = 0;
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
			if ((params.proto_based_target.target != PP2_CLS_TARGET_LOCAL_PPIO) &&
			    (params.proto_based_target.target != PP2_CLS_TARGET_OTHER)) {
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
				if (rules_params->rule_type != PP2_RULE_TYPE_PROTO &&
				    rules_params->rule_type != PP2_RULE_TYPE_PROTO_FIELD) {
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
					    rules_params->u.proto_params.proto >= MV_NET_PROTO_LAST) {
						printf("parsing fail, wrong input for --proto\n");
						return -EINVAL;
					}
					option = getopt_long_only(argc, argv, "", long_options, &long_index);
					if (option != 'l') {
						printf("parsing fail, missing input --proto_val\n");
						return -EINVAL;
					}
					rules_params->u.proto_params.val = strtoul(optarg, &ret_ptr, 0);
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
