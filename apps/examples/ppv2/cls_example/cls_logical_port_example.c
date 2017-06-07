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
