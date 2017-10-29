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
#include "../pp2_tests_main.h"
#include "drivers/mv_pp2_cls.h"

static struct list cls_qos_tbl_head;

static int pp2_cls_cli_qos_table_add(void *arg, int argc, char *argv[])
{
	struct port_desc *ports_desc = (struct port_desc *)arg;
	int type = -1;
	u32 pcp_dflt = 0;
	int pcp_idx = -1;
	int pcp_val = -1;
	u32 dscp_dflt = 0;
	int dscp_idx = -1;
	int dscp_val = -1;
	int pcp_map[MV_VLAN_PRIO_NUM];
	int dscp_map[MV_DSCP_NUM];
	char *ret_ptr;
	struct pp2_cls_table_node *tbl_node;
	struct pp2_cls_qos_tbl_params *qos_tbl_params;
	struct list *node;
	int i, option;
	int long_index = 0;
	struct option long_options[] = {
		{"type", required_argument, 0, 't'},
		{"pcp_default", required_argument, 0, 'p'},
		{"pcp_idx", required_argument, 0, 'i'},
		{"pcp_val", required_argument, 0, 'v'},
		{"dscp_default", required_argument, 0, 'q'},
		{"dscp_idx", required_argument, 0, 'j'},
		{"dscp_val", required_argument, 0, 'w'},
		{0, 0, 0, 0}
	};

	if (argc < 3 || argc > 145) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	/* fill pcp_table with invalid values*/
	for (i = 0; i < MV_VLAN_PRIO_NUM; i++)
		pcp_map[i] = -1;

	/* fill pcp_table with default values*/
	for (i = 0; i < MV_DSCP_NUM; i++)
		dscp_map[i] = -1;

	/* every time starting getopt we should reset optind */
	optind = 0;
	while ((option = getopt_long_only(argc, argv, "", long_options, &long_index)) != -1) {
		/* Get parameters */
		switch (option) {
		case 't':
			if (!strcmp(optarg, "none")) {
				type = PP2_CLS_QOS_TBL_NONE;
			} else if (!strcmp(optarg, "vlan")) {
				type = PP2_CLS_QOS_TBL_VLAN_PRI;
			} else if (!strcmp(optarg, "ip")) {
				type = PP2_CLS_QOS_TBL_IP_PRI;
			} else if (!strcmp(optarg, "vlan_ip")) {
				type = PP2_CLS_QOS_TBL_VLAN_IP_PRI;
			} else if (!strcmp(optarg, "ip_vlan")) {
				type = PP2_CLS_QOS_TBL_IP_VLAN_PRI;
			} else {
				printf("parsing fail, wrong input for qos type\n");
				return -EINVAL;
			}
			break;
		case 'p':
			pcp_dflt = strtoul(optarg, &ret_ptr, 0);
			if ((optarg == ret_ptr) || (pcp_dflt < 0) ||
			    (pcp_dflt > ports_desc->num_tcs)) {
				printf("parsing fail, wrong input for --pcp_dflt\n");
				return -EINVAL;
			}
			break;
		case 'i':
			pcp_idx = strtoul(optarg, &ret_ptr, 0);
			if ((optarg == ret_ptr) || (pcp_idx < 0) ||
			    (pcp_idx >= MV_VLAN_PRIO_NUM)) {
				printf("parsing fail, wrong input for pcp_idx\n");
				return -EINVAL;
			}
			option = getopt_long_only(argc, argv, "", long_options, &long_index);
			if (option == 'v') {
				pcp_val = strtoul(optarg, &ret_ptr, 0);
				if ((optarg == ret_ptr) || (pcp_val < 0) ||
				    (pcp_val >= ports_desc->num_tcs)) {
					printf("parsing fail, wrong input for pcp_val\n");
					return -EINVAL;
				}
			} else {
				printf("parsing fail, wrong input, line = %d\n", __LINE__);
				return -EINVAL;
			}
			pcp_map[pcp_idx] = pcp_val;
			break;
		case 'q':
			dscp_dflt = strtoul(optarg, &ret_ptr, 0);
			if ((optarg == ret_ptr) || (dscp_dflt < 0) ||
			    (dscp_dflt > ports_desc->num_tcs)) {
				printf("parsing fail, wrong input for --pcp_dflt\n");
				return -EINVAL;
			}
			break;
		case 'j':
			dscp_idx = strtoul(optarg, &ret_ptr, 0);
			if ((optarg == ret_ptr) || (dscp_idx < 0) ||
			    (dscp_idx >= MV_DSCP_NUM)) {
				printf("parsing fail, wrong input for dscp_idx\n");
				return -EINVAL;
			}
			option = getopt_long_only(argc, argv, "", long_options, &long_index);
			if (option == 'w') {
				dscp_val = strtoul(optarg, &ret_ptr, 0);
				if ((optarg == ret_ptr) || (dscp_val < 0) ||
				    (dscp_val >= ports_desc->num_tcs)) {
					printf("parsing fail, wrong input for dscp_val\n");
					return -EINVAL;
				}
			} else {
				printf("parsing fail, wrong input, line = %d\n", __LINE__);
				return -EINVAL;
			}
			dscp_map[dscp_idx] = dscp_val;
			break;
		default:
			printf("parsing fail, wrong input\n");
			return -EINVAL;
		}
	}

	/* check if all the fields are initialized */
	if (type <= 0) {
		printf("parsing fail, invalid --type\n");
		return -EINVAL;
	}

	tbl_node = malloc(sizeof(*tbl_node));
	if (!tbl_node) {
		pr_err("%s no mem for new table!\n", __func__);
		return -ENOMEM;
	}
	memset(tbl_node, 0, sizeof(*tbl_node));

	/* add table to db */
	tbl_node->idx = pp2_cls_table_next_index_get(&cls_qos_tbl_head);
	node = pp2_cls_table_next_node_get(&cls_qos_tbl_head, tbl_node->idx);
	list_add_to_tail(&tbl_node->list_node, node);

	qos_tbl_params = &tbl_node->qos_tbl_params;
	qos_tbl_params->type = type;

	/* fill pcp_table wit default values*/
	for (i = 0; i < MV_VLAN_PRIO_NUM; i++) {
		if (pcp_map[i] == -1)
			qos_tbl_params->pcp_cos_map[i].tc = pcp_dflt;
		else
			qos_tbl_params->pcp_cos_map[i].tc = pcp_map[i];
		qos_tbl_params->pcp_cos_map[i].ppio = ports_desc->ppio;
		pr_debug("pcp[%d] %d\n", i, qos_tbl_params->pcp_cos_map[i].tc);
	}

	/* fill pcp_table wit default values*/
	for (i = 0; i < MV_DSCP_NUM; i++) {
		if (dscp_map[i] == -1)
			qos_tbl_params->dscp_cos_map[i].tc = dscp_dflt;
		else
			qos_tbl_params->dscp_cos_map[i].tc = dscp_map[i];
		qos_tbl_params->dscp_cos_map[i].ppio = ports_desc->ppio;
		pr_debug("dscp[%d] %d\n", i, qos_tbl_params->dscp_cos_map[i].tc);
	}

	if (!pp2_cls_qos_tbl_init(qos_tbl_params, &tbl_node->tbl))
		pr_info("table created, table_index %d\n", tbl_node->idx);
	else
		pr_info("FAIL\n");

	return 0;
}

static int pp2_cls_qos_table_remove(u32 tbl_idx)
{
	struct pp2_cls_table_node *tbl_node;

	LIST_FOR_EACH_OBJECT(tbl_node, struct pp2_cls_table_node, &cls_qos_tbl_head, list_node) {
		pr_debug("tbl_node->idx %d, tbl_idx %d\n", tbl_node->idx, tbl_idx);

		if (tbl_node->idx == tbl_idx) {
			pr_info("Removing table %d\n", tbl_idx);
			pp2_cls_qos_tbl_deinit(tbl_node->tbl);
			list_del(&tbl_node->list_node);
			free(tbl_node);
			return 0;
		}
	}
	return -EINVAL;
}

static int pp2_cls_cli_qos_table_remove(void *arg, int argc, char *argv[])
{
	int tbl_idx = -1;
	char *ret_ptr;
	int option = 0;
	int long_index = 0;
	int rc;
	struct option long_options[] = {
		{"qos_table_index", required_argument, 0, 't'},
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
			if (optarg == ret_ptr) {
				printf("parsing fail, wrong input for --qos_table_index\n");
				return -EINVAL;
			}
			break;
		default:
			printf("parsing fail, wrong input, line = %d\n", __LINE__);
			return -EINVAL;
		}
	}

	/* check if all the fields are initialized */
	if (tbl_idx <= 0) {
		printf("parsing fail, invalid --table_index\n");
		return -EINVAL;
	}

	rc = pp2_cls_qos_table_remove(tbl_idx);
	if (!rc)
		printf("OK\n");
	else
		printf("error removing table\n");
	return 0;
}

static int pp2_cls_cli_qos_cls_table_dump(void *arg, int argc, char *argv[])
{
	u32 i;
	struct pp2_cls_table_node *tbl_node;
	u32 num_tables = list_num_objs(&cls_qos_tbl_head);

	printf("total indexes: %d\n", num_tables);
	if (num_tables > 0) {
		app_print_horizontal_line(14, "=");

		LIST_FOR_EACH_OBJECT(tbl_node, struct pp2_cls_table_node, &cls_qos_tbl_head, list_node) {
			struct pp2_cls_qos_tbl_params *tbl_ptr = &tbl_node->qos_tbl_params;

			printf("table Index %d\n", tbl_node->idx);
			printf("|idx|dscp|pcp|\n");

			for (i = 0; i < MV_DSCP_NUM; i++) {
				if (i < MV_VLAN_PRIO_NUM)
					printf("|%3d|%4d|%3d|", i, tbl_ptr->dscp_cos_map[i].tc,
					       tbl_ptr->pcp_cos_map[i].tc);
				else
					printf("|%3d|%4d|   |", i, tbl_ptr->dscp_cos_map[i].tc);
				printf("\n");
				app_print_horizontal_line(14, "-");
			}
		}
	}
	printf("OK\n");

	return 0;
}

void unregister_cli_cls_api_qos_cmds(void)
{
	struct pp2_cls_table_node *tbl_node;

	LIST_FOR_EACH_OBJECT(tbl_node, struct pp2_cls_table_node, &cls_qos_tbl_head, list_node) {
		pp2_cls_qos_table_remove(tbl_node->idx);
	}
}

int register_cli_cls_api_qos_cmds(struct port_desc *arg)
{
	struct cli_cmd_params cmd_params;

	INIT_LIST(&cls_qos_tbl_head);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_qos_tbl_init";
	cmd_params.desc		= "create a QoS classifier table (one table per interface)\n";
	cmd_params.format	= "--type --pcp_map --dscp_map\n"
				  "\t\t\t\t--type	(string) vlan, ip, vlan_ip, ip_vlan\n"
				  "\t\t\t\t--pcp_default	TC number - default TC for all table values\n"
				  "\t\t\t\t			except for values defined in pcp_idx and pcp_val\n"
				  "\t\t\t\t--pcp_idx		index in pcp_map table\n"
				  "\t\t\t\t--pcp_val		TC value in pcp_map table\n"
				  "\t\t\t\t			pcp_idx and pcp_val need to be set together\n"
				  "\t\t\t\t--dscp_default	TC number - default TC for all table values\n"
				  "\t\t\t\t			except for values defined in pcp_idx and pcp_val\n"
				  "\t\t\t\t--dscp_idx		index in pcp_map table\n"
				  "\t\t\t\t--dscp_val		TC value in pcp_map table\n"
				  "\t\t\t\t			pcp_idx and pcp_val need to be set together\n";
	cmd_params.cmd_arg	= arg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_qos_table_add;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_qos_tbl_deinit";
	cmd_params.desc		= "remove a specified qos table";
	cmd_params.format	= "--qos_table_index (dec) index to existing table\n";
	cmd_params.cmd_arg	= NULL;
	cmd_params.do_cmd_cb	= (void *)pp2_cls_cli_qos_table_remove;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_qos_tbl_dump";
	cmd_params.desc		= "display classifier defined tables in cls_demo application";
	cmd_params.format	= "";
	cmd_params.cmd_arg	= NULL;
	cmd_params.do_cmd_cb	= (void *)pp2_cls_cli_qos_cls_table_dump;
	mvapp_register_cli_cmd(&cmd_params);

	return 0;
}
