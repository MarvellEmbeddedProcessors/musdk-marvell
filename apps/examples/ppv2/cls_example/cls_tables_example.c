/*******************************************************************************
 * Copyright (C) Marvell International Ltd. and its affiliates
 *
 * This software file (the "File") is owned and distributed by Marvell
 * International Ltd. and/or its affiliates ("Marvell") under the following
 * alternative licensing terms.  Once you have made an election to distribute the
 * File under one of the following license alternatives, please (i) delete this
 * introductory statement regarding license alternatives, (ii) delete the three
 * license alternatives that you have not elected to use and (iii) preserve the
 * Marvell copyright notice above.
 *
 ********************************************************************************
 * Marvell Commercial License Option
 *
 * If you received this File from Marvell and you have entered into a commercial
 * license agreement (a "Commercial License") with Marvell, the File is licensed
 * to you under the terms of the applicable Commercial License.
 *
 ********************************************************************************
 * Marvell GPL License Option
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File in accordance with the terms and conditions of the General
 * Public License Version 2, June 1991 (the "GPL License"), a copy of which is
 * available along with the File in the license.txt file or by writing to the Free
 * Software Foundation, Inc., or on the worldwide web at http://www.gnu.org/licenses/gpl.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY
 * DISCLAIMED.  The GPL License provides additional details about this warranty
 * disclaimer.
 *
 ********************************************************************************
 * Marvell GNU General Public License FreeRTOS Exception
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File in accordance with the terms and conditions of the Lesser
 * General Public License Version 2.1 plus the following FreeRTOS exception.
 * An independent module is a module which is not derived from or based on
 * FreeRTOS.
 * Clause 1:
 * Linking FreeRTOS statically or dynamically with other modules is making a
 * combined work based on FreeRTOS. Thus, the terms and conditions of the GNU
 * General Public License cover the whole combination.
 * As a special exception, the copyright holder of FreeRTOS gives you permission
 * to link FreeRTOS with independent modules that communicate with FreeRTOS solely
 * through the FreeRTOS API interface, regardless of the license terms of these
 * independent modules, and to copy and distribute the resulting combined work
 * under terms of your choice, provided that:
 * 1. Every copy of the combined work is accompanied by a written statement that
 * details to the recipient the version of FreeRTOS used and an offer by yourself
 * to provide the FreeRTOS source code (including any modifications you may have
 * made) should the recipient request it.
 * 2. The combined work is not itself an RTOS, scheduler, kernel or related
 * product.
 * 3. The independent modules add significant and primary functionality to
 * FreeRTOS and do not merely extend the existing functionality already present in
 * FreeRTOS.
 * Clause 2:
 * FreeRTOS may not be used for any competitive or comparative purpose, including
 * the publication of any form of run time or compile time metric, without the
 * express permission of Real Time Engineers Ltd. (this is the norm within the
 * industry and is intended to ensure information accuracy).
 *
 ********************************************************************************
 * Marvell BSD License Option
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File under the following licensing terms.
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *	* Redistributions of source code must retain the above copyright notice,
 *	  this list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the distribution.
 *
 *	* Neither the name of Marvell nor the names of its contributors may be
 *	  used to endorse or promote products derived from this software without
 *	  specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************/

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

