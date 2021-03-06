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
#include <getopt.h>
#include "mvapp.h"
#include "../pp2_tests_main.h"
#include "src/drivers/ppv2/pp2.h"
#include "src/drivers/ppv2/cls/pp2_cls_types.h"
#include "src/drivers/ppv2/cls/pp2_cls_internal_types.h"
#include "src/drivers/ppv2/cls/pp2_c3.h"
#include "src/drivers/ppv2/cls/pp2_c2.h"
#include "src/drivers/ppv2/cls/pp2_flow_rules.h"
#include "src/drivers/ppv2/cls/pp2_cls_db.h"

#include "mv_pp2_ppio.h"

struct edrop_node {
	int	valid;
	int	busy;
	struct	pp2_cls_early_drop		*edrop;
	struct	pp2_cls_early_drop_params	edrop_params;
};

static struct edrop_node edrops[PP2_CLS_EARLY_DROP_NUM];

static int cli_cls_edrop_get_edrop_node(int idx)
{
	int i;

	if (idx >= 0) {
		if (idx >= PP2_CLS_EARLY_DROP_NUM)
			return -EFAULT;

		/* "reserved" will be check in driver. this way we can do unit test fot the driver*/
		if (edrops[idx].busy) {
			printf("Occupied edrop-id\n");
			return -1;
		}
		return idx;
	}

	for (i = 0; i < PP2_CLS_EARLY_DROP_NUM; i++)
		if (edrops[i].valid && !edrops[i].busy)
			return i;
	printf("No free edrops-idx\n");

	return -1;
}

static int cli_cls_edrop_add(void *arg, int argc, char *argv[])
{
	struct port_desc *port_desc = (struct port_desc *)arg;
	struct pp2_cls_early_drop_params edrop_params;
	char name[15];
	char *ret_ptr;
	int edrop_idx = -1, option = 0;
	int long_index = 0;
	struct option long_options[] = {
		{"edrop_index", required_argument, 0, 'e'},
		{"threshold", required_argument, 0, 't'},
		{0, 0, 0, 0}
	};

	/* every time starting getopt we should reset optind */
	optind = 0;
	/* Get parameters */
	memset(&edrop_params, 0, sizeof(struct pp2_cls_early_drop_params));
	while ((option = getopt_long_only(argc, argv, "", long_options, &long_index)) != -1) {
		switch (option) {
		case 'e':
			edrop_idx = strtoul(optarg, &ret_ptr, 0);
			if ((optarg == ret_ptr) || (edrop_idx < 1) || (edrop_idx > PP2_CLS_EARLY_DROP_NUM)) {
				printf("parsing fail, wrong input for --edrop_index\n");
				return -EINVAL;
			}
			edrop_idx -= 1;
			break;
		case 't':
			edrop_params.threshold = strtoul(optarg, &ret_ptr, 0);
			break;
		default:
			printf("parsing fail, wrong input, line = %d\n", __LINE__);
			return -EINVAL;
		}
	}

	edrop_idx = cli_cls_edrop_get_edrop_node(edrop_idx);
	if (edrop_idx < 0) {
		printf("FAIL\n");
		return 0;
	}

	memcpy(&edrops[edrop_idx].edrop_params, &edrop_params, sizeof(struct pp2_cls_early_drop_params));
	memset(name, 0, sizeof(name));
	snprintf(name, sizeof(name), "ed-%d:%d", port_desc->pp_id, edrop_idx);
	edrops[edrop_idx].edrop_params.match = name;
	if (!pp2_cls_early_drop_init(&edrops[edrop_idx].edrop_params, &edrops[edrop_idx].edrop)) {
		printf("OK\n");
		edrops[edrop_idx].busy = 1;
	} else
		printf("FAIL\n");
	return 0;
}

static int cli_cls_edrop_remove(void *arg, int argc, char *argv[])
{
	int edrop_idx = -1;
	char *ret_ptr;
	int option = 0;
	int long_index = 0;
	struct option long_options[] = {
		{"edrop_index", required_argument, 0, 'e'},
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
		case 'e':
			edrop_idx = strtoul(optarg, &ret_ptr, 0);
			if ((optarg == ret_ptr) || (edrop_idx < 1) || (edrop_idx > PP2_CLS_EARLY_DROP_NUM)) {
				printf("parsing fail, wrong input for --edrop_index\n");
				return -EINVAL;
			}
			edrop_idx -= 1;
			break;
		default:
			printf("parsing fail, wrong input, line = %d\n", __LINE__);
			return -EINVAL;
		}
	}

	/* check if all the fields are initialized */
	if (edrop_idx < 0) {
		printf("parsing fail, invalid --edrop_index\n");
		return -EINVAL;
	}

	if (edrops[edrop_idx].valid &&
	    edrops[edrop_idx].busy &&
	    edrops[edrop_idx].edrop) {
		pp2_cls_early_drop_deinit(edrops[edrop_idx].edrop);
		printf("OK\n");
		edrops[edrop_idx].busy = 0;
		edrops[edrop_idx].edrop = NULL;
	} else
		printf("invalid state. can't remove edrop\n");

	return 0;
}

static int cli_cls_edrop_assign_qid(void *arg, int argc, char *argv[])
{
	struct port_desc *port_desc = (struct port_desc *)arg;
	char *ret_ptr;
	int edrop_idx = -1, option = 0, long_index = 0, assign = 1, tc = -1, qid = -1;
	struct option long_options[] = {
		{"edrop_index", required_argument, 0, 'e'},
		{"tc", required_argument, 0, 't'},
		{"qid", required_argument, 0, 'q'},
		{"unassign", no_argument, 0, 'a'},
		{0, 0, 0, 0}
	};

	/* every time starting getopt we should reset optind */
	optind = 0;
	/* Get parameters */
	while ((option = getopt_long_only(argc, argv, "", long_options, &long_index)) != -1) {
		switch (option) {
		case 'e':
			edrop_idx = strtoul(optarg, &ret_ptr, 0);
			if ((optarg == ret_ptr) || (edrop_idx < 1) || (edrop_idx > PP2_CLS_EARLY_DROP_NUM)) {
				printf("parsing fail, wrong input for --edrop_index\n");
				return -EINVAL;
			}
			edrop_idx -= 1;
			break;
		case 't':
			tc = strtoul(optarg, &ret_ptr, 0);
			break;
		case 'q':
			qid = strtoul(optarg, &ret_ptr, 0);
			break;
		case 'a':
			assign = 0;
			break;
		default:
			printf("parsing fail, wrong input, line = %d\n", __LINE__);
			return -EINVAL;
		}
	}

	/* check if all the fields are initialized */
	if (edrop_idx < 0) {
		printf("parsing fail, invalid --edrop_index\n");
		return -EINVAL;
	}

	if (tc < 0) {
		printf("parsing fail, invalid --tc\n");
		return -EINVAL;
	}

	if (qid < 0) {
		printf("parsing fail, invalid --qid\n");
		return -EINVAL;
	}

	if (edrops[edrop_idx].valid &&
	    edrops[edrop_idx].busy &&
	    edrops[edrop_idx].edrop &&
	    (!pp2_ppio_set_inq_early_drop(port_desc->ppio, tc, qid, assign, edrops[edrop_idx].edrop))) {
		printf("OK\n");
	} else
		printf("FAIL\n");

	return 0;
}

static int pp2_cls_cli_edrops_dump(struct port_desc *port_desc)
{
	struct pp2_port *port = GET_PPIO_PORT(port_desc->ppio);
	struct pp2_inst *inst = port->parent;

	pp2_cls_edrop_dump(inst);

	return 0;
}

void cli_cls_prepare_edrops_db(u32 edrops_reserved_map)
{
	int i;

	memset(edrops, 0, sizeof(edrops));
	for (i = 0; i < PP2_CLS_EARLY_DROP_NUM; i++)
		if (!(edrops_reserved_map & (1 << i)))
			edrops[i].valid = 1;
}

void unregister_cli_cls_api_edrop_cmds(void)
{
	int i;

	for (i = 0; i < PP2_CLS_EARLY_DROP_NUM; i++)
		if (edrops[i].valid && edrops[i].busy)
			pp2_cls_early_drop_deinit(edrops[i].edrop);
}

int register_cli_cls_api_edrop_cmds(struct port_desc *arg)
{
	struct cli_cmd_params cmd_params;

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_edrop_init";
	cmd_params.desc		= "create an early-drop entry";
	cmd_params.format	= "--edrop_index --threshold";
	cmd_params.cmd_arg	= arg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))cli_cls_edrop_add;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_edrop_deinit";
	cmd_params.desc		= "remove a specified early-drop entry";
	cmd_params.format	= "--edrop_index (dec) index to existing early-drop entry\n";
	cmd_params.cmd_arg	= arg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))cli_cls_edrop_remove;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_edrop_assign_qid";
	cmd_params.desc		= "un/assign qid to an early-drop entry";
	cmd_params.format	= "--edrop_index --tc --qid --unassign(opt)";
	cmd_params.cmd_arg	= arg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))cli_cls_edrop_assign_qid;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_edrop_dump";
	cmd_params.desc		= "dump edrops entries";
	cmd_params.format	= "";
	cmd_params.cmd_arg	= arg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_edrops_dump;
	mvapp_register_cli_cmd(&cmd_params);

	return 0;
}
