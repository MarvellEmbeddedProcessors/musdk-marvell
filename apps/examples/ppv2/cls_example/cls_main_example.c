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
#include "pp2_utils.h"
#include "mv_pp2.h"

#include "cls_main_example.h"
#include "src/drivers/ppv2/pp2.h"
/*
 * including cls_debug.h is optional and used only for debugging purposes.
 * It is added here only to show cls capabilities but should be avoided in other applications.
 * i.e. when adding a 5-tuple rule (table and key), it allows to dump the flow tables and C3 tables.
 */
#include "../../../tests/ppv2/cls_demo/cls_debug.h"

#define CLS_APP_DMA_MEM_SIZE			(10 * 1024 * 1024)
#define CLS_APP_QS_MAP_MASK			0xFFFF
#define CLS_APP_FIRST_MUSDK_IN_QUEUE		0
#define CLS_APP_MAX_NUM_TCS_PER_PORT		4
#define CLS_APP_DEF_Q_SIZE			1024
#define CLS_APP_HIF_Q_SIZE			CLS_APP_DEF_Q_SIZE
#define CLS_APP_RX_Q_SIZE			CLS_APP_DEF_Q_SIZE
#define CLS_APP_TX_Q_SIZE			CLS_APP_DEF_Q_SIZE
#define CLS_APP_MAX_NUM_QS_PER_CORE		MVAPPS_PP2_MAX_NUM_QS_PER_TC
#define CLS_APP_BPOOLS_INF		{ {2048, 1024} }

struct glob_arg {
	struct glb_common_args	cmn_args;  /* Keep first */

	u32				hash_type;
	struct pp2_init_params		pp2_params;
	int				logical_port_flag;
	int				qos_flag;
	int				cls_table_flag;
	int				policer_flag;
	int				filter_flag;
	int				txsched_flag;
};

struct local_arg {
	struct local_common_args	cmn_args;  /* Keep first */
};

static struct glob_arg garg = {};

static int main_loop(void *arg, int *running)
{
	struct local_arg *larg = (struct local_arg *)arg;

	if (!larg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	while (*running)
	;

	return 0;
}

static int init_all_modules(void)
{
	struct pp2_init_params *pp2_params = &garg.pp2_params;
	struct pp2_glb_common_args *pp2_args = (struct pp2_glb_common_args *)garg.cmn_args.plat;
	int			 err;
	char			 file[PP2_MAX_BUF_STR_LEN];
	int			 num_rss_tables = 0;

	pr_info("Global initializations ...\n");

	err = mv_sys_dma_mem_init(CLS_APP_DMA_MEM_SIZE);
	if (err)
		return err;

	memset(pp2_params, 0, sizeof(*pp2_params));
	pp2_params->hif_reserved_map = MVAPPS_PP2_HIFS_RSRV;
	pp2_params->bm_pool_reserved_map = MVAPPS_PP2_BPOOLS_RSRV;
	pp2_params->policers_reserved_map = MVAPPS_PP2_POLICERSS_RSRV;

	sprintf(file, "%s/%s", PP2_SYSFS_RSS_PATH, PP2_SYSFS_RSS_NUM_TABLES_FILE);
	num_rss_tables = appp_pp2_sysfs_param_get(pp2_args->ports_desc[0].name, file);
	if (num_rss_tables < 0) {
		pr_err("Failed to read kernel RSS tables. Please check mvpp2x_sysfs.ko is loaded\n");
		return -EFAULT;
	}
	pp2_params->rss_tbl_reserved_map = (1 << num_rss_tables) - 1;

	err = pp2_init(pp2_params);
	if (err)
		return err;

	pr_info("done\n");
	return 0;
}

static int init_local_modules(struct glob_arg *garg)
{
	int				err, port_index;
	struct bpool_inf		infs[] = CLS_APP_BPOOLS_INF;
	struct pp2_glb_common_args *pp2_args = (struct pp2_glb_common_args *) garg->cmn_args.plat;
	int				i;

	pr_info("Local initializations ...\n");

	err = app_hif_init(&pp2_args->hif, CLS_APP_HIF_Q_SIZE);
	if (err)
		return err;

	pp2_args->num_pools = ARRAY_SIZE(infs);
	err = app_build_all_bpools(&pp2_args->pools_desc, pp2_args->num_pools, infs, pp2_args->hif);
	if (err)
		return err;

	for (port_index = 0; port_index < garg->cmn_args.num_ports; port_index++) {
		struct port_desc *port = &pp2_args->ports_desc[port_index];

		if (garg->logical_port_flag) {
			err = pp2_cls_logical_port_params_example(&pp2_args->ports_desc[port_index].port_params,
								  &port->ppio_type);
			if (err) {
				pr_err("pp2_cls_logical_port_params failed!\n");
				return -EINVAL;
			}
		}

		if (garg->policer_flag)
			pp2_cls_policer_params_example(port);

		if (garg->txsched_flag)
			pp2_cls_txsched_params_example(port);

		err = app_find_port_info(port);
		if (!err) {
			port->num_tcs	= CLS_APP_MAX_NUM_TCS_PER_PORT;
			for (i = 0; i < port->num_tcs; i++)
				port->num_inqs[i] = MVAPPS_PP2_MAX_NUM_QS_PER_TC;
			port->inq_size	= CLS_APP_RX_Q_SIZE;
			port->num_outqs	= CLS_APP_MAX_NUM_TCS_PER_PORT;
			port->outq_size	= CLS_APP_TX_Q_SIZE;
			port->first_inq = CLS_APP_FIRST_MUSDK_IN_QUEUE;
			port->hash_type = garg->hash_type;
			/* pkt_offset=0 not to be changed, before recoding rx_data_path to use pkt_offset as well */
			err = app_port_init(port, pp2_args->num_pools, pp2_args->pools_desc[port->pp_id],
					    DEFAULT_MTU, 0);
			if (err) {
				pr_err("Failed to initialize port %d (pp_id: %d)\n", port_index,
				       port->pp_id);
				return err;
			}
		} else {
			return err;
		}
	}
	if (garg->cls_table_flag) {
		pp2_cls_add_5_tuple_table(pp2_args->ports_desc);
		pp2_cls_example_rule_key(pp2_args->ports_desc);
	}
	if (garg->qos_flag)
		pp2_cls_qos_table_add_example(pp2_args->ports_desc[0].ppio);

	pr_info("done\n");
	return 0;
}


static int register_cli_cmds(struct glob_arg *garg)
{
	struct pp2_glb_common_args *pp2_args = (struct pp2_glb_common_args *) garg->cmn_args.plat;

	struct pp2_ppio *ppio = pp2_args->ports_desc[0].ppio;

	if (!garg->cmn_args.cli)
		return -EFAULT;

	if (garg->filter_flag)
		register_cli_filter_cmds(ppio);
	if (garg->qos_flag)
		register_cli_qos_cmds(ppio);
	if (garg->logical_port_flag)
		register_cli_c2_cmds(ppio);
	if (garg->cls_table_flag)
		register_cli_c3_cmds(ppio);

	app_register_cli_common_cmds(pp2_args->ports_desc);

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


static int init_local(void *arg, int id, void **_larg)
{
	struct glob_arg		*garg = (struct glob_arg *)arg;
	struct local_arg	*larg;
	struct pp2_glb_common_args *glb_pp2_args = (struct pp2_glb_common_args *) garg->cmn_args.plat;
	struct pp2_lcl_common_args *lcl_pp2_args;

	int			 i, err;

	if (!garg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	larg = (struct local_arg *)malloc(sizeof(struct local_arg));
	if (!larg) {
		pr_err("No mem for local arg obj!\n");
		return -ENOMEM;
	}
	memset(larg, 0, sizeof(struct local_arg));

	larg->cmn_args.plat = (struct pp2_lcl_common_args *)malloc(sizeof(struct pp2_lcl_common_args));
	if (!larg->cmn_args.plat) {
		pr_err("No mem for local plat arg obj!\n");
		free(larg);
		return -ENOMEM;
	}
	memset(larg->cmn_args.plat, 0, sizeof(struct pp2_lcl_common_args));
	lcl_pp2_args = (struct pp2_lcl_common_args *) larg->cmn_args.plat;

	larg->cmn_args.id		= id;
	larg->cmn_args.num_ports	= garg->cmn_args.num_ports;

	lcl_pp2_args->lcl_ports_desc	= (struct lcl_port_desc *)
					   malloc(larg->cmn_args.num_ports * sizeof(struct lcl_port_desc));
	if (!lcl_pp2_args->lcl_ports_desc) {
		pr_err("no mem for local-port-desc obj!\n");
		free(larg->cmn_args.plat);
		free(larg);
		return -ENOMEM;
	}
	memset(lcl_pp2_args->lcl_ports_desc, 0, larg->cmn_args.num_ports * sizeof(struct lcl_port_desc));

	err = app_hif_init(&lcl_pp2_args->hif, CLS_APP_HIF_Q_SIZE);
	if (err)
		return err;

	for (i = 0; i < larg->cmn_args.num_ports; i++)
		app_port_local_init(i, larg->cmn_args.id, &lcl_pp2_args->lcl_ports_desc[i],
				    &glb_pp2_args->ports_desc[i]);

	lcl_pp2_args->pools_desc	= glb_pp2_args->pools_desc;
	larg->cmn_args.garg              = garg;
	garg->cmn_args.largs[id] = larg;


	*_larg = larg;
	return 0;
}

static void usage(char *progname)
{
	printf("\n"
		"MUSDK cls-demo application.\n"
		"\n"
		"Usage: %s OPTIONS\n"
		"  E.g. %s -i eth0\n"
		"\n"
		"Mandatory OPTIONS:\n"
		"\t-i, --interface <eth-interface>\n"
		"\n"
		"Optional OPTIONS:\n"
		"\t--hash_type <none, 2-tuple, 5-tuple>\n"
		"\t--logical_port	(no argument) configure logical port parameters\n"
		"\t--cls_table	(no argument) init hardcode 5-tuple table & rule key, and add relevant cli\n"
		"\t--qos\t	(no argument) init hardcode qos table and add relevant cli\n"
		"\t--filter	(no argument) add filter cli\n"
		"\t--policer	(no argument) init hardcoded policer and add it to ppio\n"
		"\t--txsched	(no argument) init hardcoded txsched parameters\n"

		"\n", MVAPPS_NO_PATH(progname), MVAPPS_NO_PATH(progname)
		);
}

static int parse_args(struct glob_arg *garg, int argc, char *argv[])
{
	int i = 1;
	int option;
	int long_index = 0;
	struct pp2_glb_common_args *pp2_args = (struct pp2_glb_common_args *) garg->cmn_args.plat;
	struct pp2_ppio_params	*port_params = &pp2_args->ports_desc[0].port_params;
	struct pp2_init_params	*pp2_params = &garg->pp2_params;
	struct port_desc	*port = &pp2_args->ports_desc[0];

	struct option long_options[] = {
		{"help", no_argument, 0, 'h'},
		{"interface", required_argument, 0, 'i'},
		{"hash_type", required_argument, 0, 's'},
		{"logical_port", no_argument, 0, 'g'},
		{"cls_table", no_argument, 0, 't'},
		{"qos", no_argument, 0, 'q'},
		{"filter", no_argument, 0, 'f'},
		{"policer", no_argument, 0, 'p'},
		{"txsched", no_argument, 0, 'x'},
		{0, 0, 0, 0}
	};

	garg->cmn_args.cpus = 1;
	garg->hash_type = PP2_PPIO_HASH_T_2_TUPLE;
	garg->cmn_args.num_ports = 0;
	garg->cmn_args.cli = 1;

	memset(port_params, 0, sizeof(*port_params));
	memset(pp2_params, 0, sizeof(*pp2_params));
	port->ppio_type = PP2_PPIO_T_NIC;
	garg->logical_port_flag = false;
	garg->qos_flag = false;
	garg->cls_table_flag = false;
	garg->filter_flag = false;
	garg->policer_flag = false;
	garg->cmn_args.pkt_offset = 0;

	/* every time starting getopt we should reset optind */
	optind = 0;
	while ((option = getopt_long(argc, argv, "hi:b:etgpx", long_options, &long_index)) != -1) {
		switch (option) {
		case 'h':
			usage(argv[0]);
			exit(0);
			break;
		case 'i':
			snprintf(pp2_args->ports_desc[garg->cmn_args.num_ports].name,
				 sizeof(pp2_args->ports_desc[garg->cmn_args.num_ports].name), "%s", optarg);
			garg->cmn_args.num_ports++;
			/* currently supporting only 1 port */
			if (garg->cmn_args.num_ports > 1) {
				pr_err("too many ports specified (%d vs %d)\n",
				       garg->cmn_args.num_ports, 1);
				return -EINVAL;
			}
			break;
		case 's':
			if (!strcmp(optarg, "none")) {
				garg->hash_type = PP2_PPIO_HASH_T_NONE;
			} else if (!strcmp(optarg, "2-tuple")) {
				garg->hash_type = PP2_PPIO_HASH_T_2_TUPLE;
			} else if (!strcmp(optarg, "5-tuple")) {
				garg->hash_type = PP2_PPIO_HASH_T_5_TUPLE;
			} else {
				printf("parsing fail, wrong input for hash\n");
				return -EINVAL;
			}
			break;
		case 'g':
			garg->logical_port_flag = true;
			break;
		case 't':
			garg->cls_table_flag = true;
			break;
		case 'q':
			garg->qos_flag = true;
			break;
		case 'f':
			garg->filter_flag = true;
			break;
		case 'p':
			garg->policer_flag = true;
			break;
		case 'x':
			garg->txsched_flag = true;
			break;
		default:
			pr_err("argument (%s) not supported!\n", argv[i]);
			return -EINVAL;
		}
	}

	/* Now, check validity of all inputs */
	if (!garg->cmn_args.num_ports) {
		pr_err("No port defined!\n");
		return -EINVAL;
	}

	return 0;
}

int main(int argc, char *argv[])
{
	struct mvapp_params	mvapp_params;
	struct pp2_glb_common_args *pp2_args;
	int			err;

	setbuf(stdout, NULL);

	pr_debug("pr_debug is enabled\n");

	garg.cmn_args.plat = (struct pp2_glb_common_args *)malloc(sizeof(struct pp2_glb_common_args));

	if (!garg.cmn_args.plat) {
		pr_err("No mem for global plat arg obj!\n");
		return -ENOMEM;
	}

	pp2_args = (struct pp2_glb_common_args *) garg.cmn_args.plat;

	err = parse_args(&garg, argc, argv);
	if (err) {
		free(garg.cmn_args.plat);
		return err;
	}

	pp2_args->pp2_num_inst = pp2_get_num_inst();

	memset(&mvapp_params, 0, sizeof(mvapp_params));
	mvapp_params.use_cli		= garg.cmn_args.cli;
	mvapp_params.num_cores		= garg.cmn_args.cpus;
	mvapp_params.global_arg		= (void *)&garg;
	mvapp_params.init_global_cb	= init_global;
	mvapp_params.deinit_global_cb	= apps_pp2_deinit_global;
	mvapp_params.init_local_cb	= init_local;
	mvapp_params.deinit_local_cb	= apps_pp2_deinit_local;
	mvapp_params.main_loop_cb	= main_loop;
	return mvapp_go(&mvapp_params);
}
