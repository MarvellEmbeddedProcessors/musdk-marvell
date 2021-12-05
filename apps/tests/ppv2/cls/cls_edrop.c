/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

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
