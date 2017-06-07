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
	int			cli;
	int			cpus;	/* cpus used for running */
	u32			hash_type;
	int			num_ports;
	int			pp2_num_inst;
	struct port_desc	ports_desc[MVAPPS_PP2_MAX_NUM_PORTS];
	struct pp2_hif		*hif;
	int			num_pools;
	struct bpool_desc	**pools_desc;
	struct pp2_init_params	pp2_params;
	int			logical_port_flag;
};

struct local_arg {
	struct tx_shadow_q	shadow_qs[CLS_APP_MAX_NUM_QS_PER_CORE];
	struct pp2_hif		*hif;
	int			 num_ports;
	struct lcl_port_desc	*ports_desc;
	struct bpool_desc	**pools_desc;
	int			 id;
	struct glob_arg		*garg;
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

	sprintf(file, "%s/%s", PP2_SYSFS_RSS_PATH, PP2_SYSFS_RSS_NUM_TABLES_FILE);
	num_rss_tables = appp_pp2_sysfs_param_get(garg.ports_desc[0].name, file);
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
	int				i;

	pr_info("Local initializations ...\n");

	err = app_hif_init(&garg->hif, CLS_APP_HIF_Q_SIZE);
	if (err)
		return err;

	garg->num_pools = ARRAY_SIZE(infs);
	err = app_build_all_bpools(&garg->pools_desc, garg->num_pools, infs, garg->hif);
	if (err)
		return err;

	for (port_index = 0; port_index < garg->num_ports; port_index++) {
		struct port_desc *port = &garg->ports_desc[port_index];

		if (garg->logical_port_flag) {
			err = pp2_cls_logical_port_params_example(&garg->ports_desc[port_index].port_params,
								  &port->ppio_type);
			if (err) {
				pr_err("pp2_cls_logical_port_params failed!\n");
				return -EINVAL;
			}
		}

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

			err = app_port_init(port, garg->num_pools, garg->pools_desc[port->pp_id], DEFAULT_MTU);
			if (err) {
				pr_err("Failed to initialize port %d (pp_id: %d)\n", port_index,
				       port->pp_id);
				return err;
			}
		} else {
			return err;
		}
	}

	pr_info("done\n");
	return 0;
}

static void destroy_local_modules(struct glob_arg *garg)
{
	app_disable_all_ports(garg->ports_desc, garg->num_ports);
	app_free_all_pools(garg->pools_desc, garg->num_pools, garg->hif);
	app_deinit_all_ports(garg->ports_desc, garg->num_ports);

	if (garg->hif)
		pp2_hif_deinit(garg->hif);
}

static void destroy_all_modules(void)
{
	pp2_deinit();
	mv_sys_dma_mem_destroy();
}

static int register_cli_cmds(struct glob_arg *garg)
{
	struct pp2_ppio *ppio = garg->ports_desc[0].ppio;

	if (!garg->cli)
		return -EFAULT;

	register_cli_cls_api_cmds(garg->ports_desc);
	register_cli_cls_api_qos_cmds(ppio);
	register_cli_filter_cmds(ppio);
	register_cli_cls_cmds(ppio);
	register_cli_c3_cmds(ppio);
	register_cli_c2_cmds(ppio);
	register_cli_mng_cmds(ppio);
	app_register_cli_common_cmds(garg->ports_desc);

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

static void deinit_global(void *arg)
{
	struct glob_arg *garg = (struct glob_arg *)arg;

	if (!garg)
		return;
	if (garg->cli)
		unregister_cli_cmds(garg);

	destroy_local_modules(garg);
	destroy_all_modules();
}

static int init_local(void *arg, int id, void **_larg)
{
	struct glob_arg		*garg = (struct glob_arg *)arg;
	struct local_arg	*larg;
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

	err = app_hif_init(&larg->hif, CLS_APP_HIF_Q_SIZE);
	if (err)
		return err;

	larg->id		= id;
	larg->num_ports		= garg->num_ports;
	larg->ports_desc	= (struct lcl_port_desc *)malloc(larg->num_ports * sizeof(struct lcl_port_desc));
	if (!larg->ports_desc) {
		pr_err("no mem for local-port-desc obj!\n");
		return -ENOMEM;
	}
	memset(larg->ports_desc, 0, larg->num_ports * sizeof(struct lcl_port_desc));
	for (i = 0; i < larg->num_ports; i++)
		app_port_local_init(i, larg->id, &larg->ports_desc[i], &garg->ports_desc[i]);

	larg->pools_desc	= garg->pools_desc;
	larg->garg              = garg;

	*_larg = larg;
	return 0;
}

static void deinit_local(void *arg)
{
	struct local_arg *larg = (struct local_arg *)arg;
	int i;

	if (!larg)
		return;

	if (larg->ports_desc) {
		for (i = 0; i < larg->num_ports; i++)
			app_port_local_deinit(&larg->ports_desc[i]);
		free(larg->ports_desc);
	}

	if (larg->hif)
		pp2_hif_deinit(larg->hif);

	free(larg);
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
		"\t-s, --hash_type <none, 2-tuple, 5-tuple>\n"
		"\t--logical_port	(no argument)configure logical port parameters\n"
		"\n", MVAPPS_NO_PATH(progname), MVAPPS_NO_PATH(progname)
		);
}

static int parse_args(struct glob_arg *garg, int argc, char *argv[])
{
	int i = 1;
	int option;
	int long_index = 0;
	struct pp2_ppio_params	*port_params = &garg->ports_desc[0].port_params;
	struct pp2_init_params	*pp2_params = &garg->pp2_params;
	struct port_desc	*port = &garg->ports_desc[0];

	struct option long_options[] = {
		{"help", no_argument, 0, 'h'},
		{"interface", required_argument, 0, 'i'},
		{"hash_type", required_argument, 0, 's'},
		{"logical_port", no_argument, 0, 'g'},
		{0, 0, 0, 0}
	};

	garg->cpus = 1;
	garg->hash_type = PP2_PPIO_HASH_T_2_TUPLE;
	garg->num_ports = 0;
	garg->cli = 1;

	memset(port_params, 0, sizeof(*port_params));
	memset(pp2_params, 0, sizeof(*pp2_params));
	port->ppio_type = PP2_PPIO_T_NIC;
	garg->logical_port_flag = false;

	/* every time starting getopt we should reset optind */
	optind = 0;
	while ((option = getopt_long(argc, argv, "hi:b:etg", long_options, &long_index)) != -1) {
		switch (option) {
		case 'h':
			usage(argv[0]);
			exit(0);
			break;
		case 'i':
			snprintf(garg->ports_desc[garg->num_ports].name,
				 sizeof(garg->ports_desc[garg->num_ports].name), "%s", optarg);
			garg->num_ports++;
			/* currently supporting only 1 port */
			if (garg->num_ports > 1) {
				pr_err("too many ports specified (%d vs %d)\n",
				       garg->num_ports, 1);
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
		default:
			pr_err("argument (%s) not supported!\n", argv[i]);
			return -EINVAL;
		}
	}

	/* Now, check validity of all inputs */
	if (!garg->num_ports) {
		pr_err("No port defined!\n");
		return -EINVAL;
	}

	return 0;
}

int main(int argc, char *argv[])
{
	struct mvapp_params	mvapp_params;
	int			err;

	setbuf(stdout, NULL);

	pr_debug("pr_debug is enabled\n");

	err = parse_args(&garg, argc, argv);
	if (err)
		return err;

	garg.pp2_num_inst = pp2_get_num_inst();

	memset(&mvapp_params, 0, sizeof(mvapp_params));
	mvapp_params.use_cli		= garg.cli;
	mvapp_params.num_cores		= garg.cpus;
	mvapp_params.global_arg		= (void *)&garg;
	mvapp_params.init_global_cb	= init_global;
	mvapp_params.deinit_global_cb	= deinit_global;
	mvapp_params.init_local_cb	= init_local;
	mvapp_params.deinit_local_cb	= deinit_local;
	mvapp_params.main_loop_cb	= main_loop;
	return mvapp_go(&mvapp_params);
}
