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

struct policer_node {
	int	valid;
	int	busy;
	struct	pp2_cls_plcr		*plcr;
	struct	pp2_cls_plcr_params	plcr_params;
};

static struct policer_node policers[PP2_CLS_PLCR_NUM];

static int cli_cls_plcr_get_policer_node(int idx)
{
	int i;

	if (idx >= 0) {
		if (idx >= PP2_CLS_PLCR_NUM)
			return -EFAULT;

		/* "reserved" will be check in driver. this way we can do unit test fot the driver*/
		if (policers[idx].busy) {
			printf("Occupied policer-id\n");
			return -1;
		}
		return idx;
	}

	for (i = 0; i < PP2_CLS_PLCR_NUM; i++)
		if (policers[i].valid && !policers[i].busy)
			return i;
	printf("No free policers-idx\n");

	return -1;
}

static int cli_cls_policer_add(void *arg, int argc, char *argv[])
{
	struct port_desc *ports_desc = (struct port_desc *)arg;
	struct pp2_cls_plcr_params policer_params;
	char name[15];
	char *ret_ptr;
	int plcr_idx = -1, option = 0;
	int long_index = 0;
	struct option long_options[] = {
		{"policer_index", required_argument, 0, 'p'},
		{"token_unit", required_argument, 0, 't'},
		{"color_mode", required_argument, 0, 'm'},
		{"cir", required_argument, 0, 'r'},
		{"cbs", required_argument, 0, 'c'},
		{"ebs", required_argument, 0, 'e'},
		{0, 0, 0, 0}
	};

	/* every time starting getopt we should reset optind */
	optind = 0;
	/* Get parameters */
	memset(&policer_params, 0, sizeof(struct pp2_cls_plcr_params));
	while ((option = getopt_long_only(argc, argv, "", long_options, &long_index)) != -1) {
		switch (option) {
		case 'p':
			plcr_idx = strtoul(optarg, &ret_ptr, 0);
			if ((optarg == ret_ptr) || (plcr_idx < 1) || (plcr_idx > PP2_CLS_PLCR_NUM)) {
				printf("parsing fail, wrong input for --policer_index\n");
				return -EINVAL;
			}
			plcr_idx -= 1;
			break;
		case 't':
			policer_params.token_unit =
				(enum pp2_cls_plcr_token_unit)strtoul(optarg, &ret_ptr, 0);
			break;
		case 'm':
			policer_params.color_mode =
				(enum pp2_cls_plcr_color_mode)strtoul(optarg, &ret_ptr, 0);
			break;
		case 'r':
			policer_params.cir = strtoul(optarg, &ret_ptr, 0);
			break;
		case 'c':
			policer_params.cbs = strtoul(optarg, &ret_ptr, 0);
			break;
		case 'e':
			policer_params.ebs = strtoul(optarg, &ret_ptr, 0);
			break;
		default:
			printf("parsing fail, wrong input, line = %d\n", __LINE__);
			return -EINVAL;
		}
	}

	plcr_idx = cli_cls_plcr_get_policer_node(plcr_idx);
	if (plcr_idx < 0) {
		printf("FAIL\n");
		return 0;
	}

	memcpy(&policers[plcr_idx].plcr_params, &policer_params, sizeof(struct pp2_cls_plcr_params));
	memset(name, 0, sizeof(name));
	snprintf(name, sizeof(name), "policer-%d:%d", ports_desc->pp_id, plcr_idx);
	policers[plcr_idx].plcr_params.match = name;
	if (!pp2_cls_plcr_init(&policers[plcr_idx].plcr_params, &policers[plcr_idx].plcr)) {
		printf("OK\n");
		policers[plcr_idx].busy = 1;
	} else
		printf("FAIL\n");
	return 0;
}

static int cli_cls_policer_remove(void *arg, int argc, char *argv[])
{
	int plcr_idx = -1;
	char *ret_ptr;
	int option = 0;
	int long_index = 0;
	struct option long_options[] = {
		{"policer_index", required_argument, 0, 'p'},
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
		case 'p':
			plcr_idx = strtoul(optarg, &ret_ptr, 0);
			if ((optarg == ret_ptr) || (plcr_idx < 1) || (plcr_idx > PP2_CLS_PLCR_NUM)) {
				printf("parsing fail, wrong input for --policer_index\n");
				return -EINVAL;
			}
			plcr_idx -= 1;
			break;
		default:
			printf("parsing fail, wrong input, line = %d\n", __LINE__);
			return -EINVAL;
		}
	}

	/* check if all the fields are initialized */
	if (plcr_idx < 0) {
		printf("parsing fail, invalid --policer_index\n");
		return -EINVAL;
	}

	if (policers[plcr_idx].valid &&
	    policers[plcr_idx].busy &&
	    policers[plcr_idx].plcr) {
		pp2_cls_plcr_deinit(policers[plcr_idx].plcr);
		printf("OK\n");
		policers[plcr_idx].busy = 0;
		policers[plcr_idx].plcr = NULL;
	} else
		printf("invalid state. can't remove policer\n");

	return 0;
}

static void pp2_cls_cli_policers_dump(struct port_desc *port_desc)
{
	struct pp2_port *port = GET_PPIO_PORT(port_desc->ppio);
	struct pp2_inst *inst = port->parent;

	pp2_cls_db_plcr_dump(inst);
}

int cli_cls_policer_params(struct port_desc *port_desc)
{
	int i, rc;

	rc = cli_cls_policer_add(port_desc, port_desc->plcr_argc, port_desc->plcr_argv);
	if (rc)
		return rc;

	/* As we don't know the policer-id, search for the first one */
	for (i = 0; i < PP2_CLS_PLCR_NUM; i++) {
		rc = cli_cls_policer_get(i, &port_desc->port_params.inqs_params.plcr);
		if (!rc)
			return rc;
	}

	return rc;
}

void cli_cls_prepare_policers_db(u32 policers_reserved_map)
{
	int i;

	memset(policers, 0, sizeof(policers));
	for (i = 0; i < PP2_CLS_PLCR_NUM; i++)
		if (!(policers_reserved_map & (1 << i)))
			policers[i].valid = 1;
}

int cli_cls_policer_get(u32 idx, struct pp2_cls_plcr **plcr)
{
	if ((idx < 0) || (idx >= PP2_CLS_PLCR_NUM))
		return -EFAULT;

	if (policers[idx].valid && policers[idx].busy) {
		*plcr = policers[idx].plcr;
		return 0;
	}

	return -EFAULT;
}

void unregister_cli_cls_api_plcr_cmds(void)
{
	int i;

	for (i = 0; i < PP2_CLS_PLCR_NUM; i++)
		if (policers[i].valid && policers[i].busy)
			pp2_cls_plcr_deinit(policers[i].plcr);
}

int register_cli_cls_api_plcr_cmds(struct port_desc *arg)
{
	struct cli_cmd_params cmd_params;

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_plcr_init";
	cmd_params.desc		= "create a policer profile";
	cmd_params.format	= "--policer_index --token_unit --color_mode --cir --cbs --ebs";
	cmd_params.cmd_arg	= arg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))cli_cls_policer_add;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_plcr_deinit";
	cmd_params.desc		= "remove a specified policer";
	cmd_params.format	= "--policer_index (dec) index to existing policer\n";
	cmd_params.cmd_arg	= arg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))cli_cls_policer_remove;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_plcr_dump";
	cmd_params.desc		= "dump cls policers profiles";
	cmd_params.format	= "";
	cmd_params.cmd_arg	= arg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_policers_dump;
	mvapp_register_cli_cmd(&cmd_params);

	return 0;
}
