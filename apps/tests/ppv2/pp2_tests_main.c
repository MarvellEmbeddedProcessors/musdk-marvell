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

#define _GNU_SOURCE         /* See feature_test_macros(7) */
#define CLS_DEBUG
#include <string.h>
#include <stdio.h>
#include <getopt.h>
#include <sched.h>


#include "mv_std.h"
#include "mvapp.h"
#include "env/io.h"


/* Following defines are used in pp2_utils.h */
#define APP_PKT_ECHO_SUPPORT
#define APP_USE_PREFETCH
#define APP_MAX_BURST_SIZE			1024
#define APP_TX_RETRY
#define USE_PP2_UTILS_LPBK_SW_RECYCLE

#define APP_DESCRIPTORS_DUMP

#include "pp2_utils.h"
#include "mv_pp2.h"

#include "src/drivers/ppv2/pp2.h"

#include "cls/cls_debug.h"
#include "pp2_tests_main.h"

#define CLS_APP_DMA_MEM_SIZE			(40 * 1024 * 1024)
#define CLS_APP_FIRST_MUSDK_IN_QUEUE		0
#define CLS_APP_COMMAND_LINE_SIZE		1024
#define CLS_APP_DEF_Q_SIZE			1024
#define CLS_APP_HIF_Q_SIZE			(8 * CLS_APP_DEF_Q_SIZE)
#define CLS_APP_RX_Q_SIZE			CLS_APP_DEF_Q_SIZE
#define CLS_APP_TX_Q_SIZE			(CLS_APP_DEF_Q_SIZE >> 1)

#define CLS_APP_DFLT_BURST_SIZE			64
#if CLS_APP_DFLT_BURST_SIZE > APP_MAX_BURST_SIZE
#error "Invalid CLS_APP_DFLT_BURST_SIZE"
#endif

#define CLS_APP_PREFETCH_SHIFT			7


#define CLS_APP_KEY_MEM_SIZE_MAX		(PP2_CLS_TBL_MAX_NUM_FIELDS * CLS_APP_STR_SIZE_MAX)


#define CLS_APP_BPOOLS_INF			{ {2048, 8192, 0, NULL} }

/* Structure containing a map of queues per core */
struct queue_map {
	int		tc_inq[PP2_PPIO_MAX_NUM_TCS];
};


struct glob_arg {
	struct glb_common_args		cmn_args;  /* Keep first */
	u32				hash_type;
	struct pp2_init_params		pp2_params;
	struct queue_map		core_queues[MVAPPS_MAX_NUM_CORES];
	int				num_tcs;
};

struct local_arg {
	struct local_common_args	cmn_args;  /* Keep first */
	struct queue_map		*core_queue;
	int				num_tcs;
};

static struct glob_arg garg = {};

/*
 * pp2_cls_table_next_index_get()
 * Get the next free table index in the list. The first index starts at 1.
 * in case entries were removed from list, this function returns the first free table index
 */
int pp2_cls_table_next_index_get(struct list *cls_tbl_head)
{
	struct pp2_cls_table_node *tbl_node;
	int idx = 0;

	LIST_FOR_EACH_OBJECT(tbl_node, struct pp2_cls_table_node, cls_tbl_head, list_node) {
		pr_debug("tbl_node->idx %d, idx %d\n", tbl_node->idx, idx);
		if ((tbl_node->idx == 0) || ((tbl_node->idx - idx) > 1))
			return idx + 1;
		idx++;
	}
	return idx + 1;
}

struct list *pp2_cls_table_next_node_get(struct list *cls_tbl_head, u32 index)
{
	int i;
	struct list *node = cls_tbl_head;

	for (i = 0; i < index; i++)
		node = LIST_NEXT(node);
	return node;
}

static int loop_1p(struct local_arg *larg, int *running)
{
	int err;
	u16 num;
	u8 tc = 0, tc_qid = 0, tx_qid = 0;
	struct pp2_lcl_common_args *pp2_args = (struct pp2_lcl_common_args *) larg->cmn_args.plat;

	if (!larg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	num = CLS_APP_DFLT_BURST_SIZE;
	if (pp2_args->lcl_ports_desc->first_txq >= PP2_PPIO_MAX_NUM_OUTQS) {
		pr_err("All TX queues in use by kernel. Exiting application\n");
		return -EINVAL;
	}
	tx_qid = pp2_args->lcl_ports_desc->first_txq;

	while (*running) {
		/* Find next queue to consume */
		tc++;
		if (tc == larg->num_tcs)
			tc = 0;
		tc_qid = larg->core_queue->tc_inq[tc];
		if (++tx_qid >= PP2_PPIO_MAX_NUM_OUTQS)
			tx_qid = pp2_args->lcl_ports_desc->first_txq;
		pr_debug("thread %d, tc %d, tc_qid %d tx_qid %d\n", larg->cmn_args.id, tc, tc_qid, tx_qid);
		err = loop_sw_recycle(&larg->cmn_args, 0, 0, tc, tc_qid, tx_qid, num);
		if (err)
			return err;
	}

	return 0;
}

static int main_loop(void *arg, int *running)
{
	struct local_arg *larg = (struct local_arg *)arg;

	if (!larg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	if (larg->cmn_args.echo)
		return loop_1p(larg, running);

	while (*running)
		;

	return 0;
}

static int init_all_modules(void)
{
	struct pp2_init_params	 pp2_params;
	struct glb_common_args *cmn_args = &garg.cmn_args;
	struct pp2_glb_common_args *pp2_args = (struct pp2_glb_common_args *)garg.cmn_args.plat;
	int			 err = 0;
	char			 file[PP2_MAX_BUF_STR_LEN];
	int			 num_rss_tables = 0;
	struct mv_sys_dma_mem_region_params params;

	pr_info("Global initializations ...\n");

	app_get_num_cpu_clusters(cmn_args);
	params.size = CLS_APP_DMA_MEM_SIZE;
	params.manage = 1;
	err = app_sys_dma_init(&params, cmn_args);
	if (err)
		return err;

	memset(&pp2_params, 0, sizeof(pp2_params));
	pp2_params.bm_pool_reserved_map = MVAPPS_PP2_BPOOLS_RSRV;

	if (garg.cmn_args.cpus > 1) {
		num_rss_tables = app_rss_num_tbl_get(pp2_args->ports_desc[0].name, file);
		if (num_rss_tables < 0)
			return -EFAULT;
	}

	pp2_params.rss_tbl_reserved_map = (1 << num_rss_tables) - 1;

	err = pp2_init(&pp2_params);
	if (err)
		return err;

	/* Must be after pp2_init */
	pp2_params.hif_reserved_map = pp2_get_kernel_hif_map();
	app_used_hifmap_init(pp2_params.hif_reserved_map);

	cli_cls_prepare_policers_db(pp2_params.policers_reserved_map);

	pr_info("done\n");
	return 0;
}

static int init_local_modules(struct glob_arg *garg)
{
	int				err, port_index;
	struct bpool_inf		std_inf[] = CLS_APP_BPOOLS_INF;
	struct bpool_inf		*jumbo_inf = NULL;
	struct bpool_inf		*infs;
	struct pp2_glb_common_args *pp2_args = (struct pp2_glb_common_args *) garg->cmn_args.plat;
	int				i = 0;

	pr_info("Local initializations ...\n");

	err = app_build_common_hifs(&garg->cmn_args, CLS_APP_HIF_Q_SIZE);
	if (err)
		return err;

	app_prepare_bpools(&garg->cmn_args, &infs, std_inf, ARRAY_SIZE(std_inf), jumbo_inf, ARRAY_SIZE(jumbo_inf));


	err = app_build_all_bpools(&pp2_args->pools_desc, pp2_args->num_pools, infs, pp2_args->hif);
	if (err)
		return err;

	for (port_index = 0; port_index < garg->cmn_args.num_ports; port_index++) {
		struct port_desc *port = &pp2_args->ports_desc[port_index];

		err = app_find_port_info(port);
		if (!err) {
			memcpy(port->mem_region, garg->cmn_args.mem_region, sizeof(garg->cmn_args.mem_region));
			port->ppio_type	= PP2_PPIO_T_NIC;
			port->num_tcs	= garg->num_tcs;
			for (i = 0; i < port->num_tcs; i++)
				port->num_inqs[i] = garg->cmn_args.cpus;
			port->inq_size	= CLS_APP_RX_Q_SIZE;
			port->num_outqs	= PP2_PPIO_MAX_NUM_OUTQS;
			port->outq_size	= CLS_APP_TX_Q_SIZE;
			port->first_inq = CLS_APP_FIRST_MUSDK_IN_QUEUE;
			if (garg->cmn_args.cpus == 1)
				port->hash_type = PP2_PPIO_HASH_T_NONE;
			else
				port->hash_type = garg->hash_type;

			if (port->plcr_argc) {
				err = cli_cls_policer_params(port);
				if (err) {
					pr_err("cli_cls_policer_params failed!\n");
					return -EINVAL;
				}
			}

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

	pr_info("done\n");
	return 0;
}

static int unregister_cli_cmds(void *arg)
{
	unregister_cli_cls_api_cmds();
	unregister_cli_cls_api_qos_cmds();

	/* TODO: unregister cli cmds */
	return 0;
}

static int register_cli_cmds(struct glob_arg *garg)
{
	struct pp2_glb_common_args *pp2_args = (struct pp2_glb_common_args *) garg->cmn_args.plat;

	struct pp2_ppio *ppio = pp2_args->ports_desc[0].ppio;

	if (!garg->cmn_args.cli)
		return -EFAULT;
	garg->cmn_args.cli_unregister_cb = unregister_cli_cmds;

	register_cli_cls_api_cmds(pp2_args->ports_desc);
	register_cli_mng_cmds(ppio);
	register_cli_cls_api_qos_cmds(pp2_args->ports_desc);
	register_cli_cls_api_plcr_cmds(pp2_args->ports_desc);
	register_cli_filter_cmds(ppio);
	register_cli_cls_cmds(ppio);
	register_cli_c3_cmds(ppio);
	register_cli_c2_cmds(ppio);
	register_cli_qos_cmds(ppio);
	register_cli_prs_cmds(ppio);
	app_register_cli_desc_cmds(pp2_args->ports_desc);
	app_register_cli_common_cmds(&garg->cmn_args);

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
	int mem_id;

	if (!garg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	mem_id = sched_getcpu() / MVAPPS_NUM_CORES_PER_AP;
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
	larg->num_tcs			= garg->num_tcs;
	lcl_pp2_args->lcl_ports_desc = (struct lcl_port_desc *)
					   malloc(larg->cmn_args.num_ports * sizeof(struct lcl_port_desc));
	if (!lcl_pp2_args->lcl_ports_desc) {
		pr_err("no mem for local-port-desc obj!\n");
		free(larg->cmn_args.plat);
		free(larg);
		return -ENOMEM;
	}
	memset(lcl_pp2_args->lcl_ports_desc, 0, larg->cmn_args.num_ports * sizeof(struct lcl_port_desc));

	err = app_hif_init_wrap(id, &garg->cmn_args.thread_lock, glb_pp2_args, lcl_pp2_args, CLS_APP_HIF_Q_SIZE,
				garg->cmn_args.mem_region[mem_id]);
	if (err)
		return err;

	larg->cmn_args.busy_wait			= garg->cmn_args.busy_wait;
	larg->cmn_args.echo		= garg->cmn_args.echo;
	larg->cmn_args.prefetch_shift	= garg->cmn_args.prefetch_shift;

	for (i = 0; i < larg->cmn_args.num_ports; i++)
		app_port_local_init(i, larg->cmn_args.id, &lcl_pp2_args->lcl_ports_desc[i],
				    &glb_pp2_args->ports_desc[i]);

	lcl_pp2_args->pools_desc	= glb_pp2_args->pools_desc;
	lcl_pp2_args->multi_buffer_release = glb_pp2_args->multi_buffer_release;
	larg->cmn_args.garg              = garg;
	garg->cmn_args.largs[id] = larg;
	for (i = 0; i < larg->cmn_args.num_ports; i++)
		glb_pp2_args->ports_desc[i].lcl_ports_desc[id] = &lcl_pp2_args->lcl_ports_desc[i];
	larg->core_queue = &garg->core_queues[larg->cmn_args.id];

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
		"\t-e, --echo			(no argument) activate echo packets\n"
		"\t-c, --cores <number>		Number of CPUs to use\n"
		"\t-a, --affinity <number>	Use set affinity (default is no affinity)\n"
		"\t-t, --num_tcs <number>	Number of Traffic classes (TCs) to use\n"
		"\t-b, --hash_type <none, 2-tuple, 5-tuple>\n"
		"\t--eth_start_hdr		(no argument)configure ethernet start header\n"
		"\t--logical_port_params	(no argument)configure logical port parameters\n"
		"\t-q, --egress_scheduler_params	(no argument)configure egress scheduler parameters\n"
		"\t--policers_range		(dec)-(dec) valid range [1-%d]\n"
		"\t--policer_params		(no argument)configure default policer parameters\n"
		"\n", MVAPPS_NO_PATH(progname), MVAPPS_NO_PATH(progname), PP2_CLS_PLCR_NUM
		);
}

static int parse_args(struct glob_arg *garg, int argc, char *argv[])
{
	int i = 1, j;
	int option;
	int long_index = 0;
	int ppio_tag_mode = 0;
	int logical_port_params = 0;
	int egress_scheduler_params = 0;
	int policer_params = 0;
	char buff[CLS_APP_COMMAND_LINE_SIZE];
	int argc_cli;
	int rc;
	char *ret_ptr;
	int num_tcs = 8;
	int num_cpus = 1;
	int affinity = MVAPPS_INVALID_AFFINITY;

	struct pp2_glb_common_args *pp2_args = (struct pp2_glb_common_args *) garg->cmn_args.plat;
	struct pp2_ppio_params	*port_params = &pp2_args->ports_desc[0].port_params;
	struct pp2_init_params	*pp2_params = &garg->pp2_params;
	struct port_desc	*port = &pp2_args->ports_desc[0];


	struct option long_options[] = {
		{"help", no_argument, 0, 'h'},
		{"interface", required_argument, 0, 'i'},
		{"echo", no_argument, 0, 'e'},
		{"cpu", required_argument, 0, 'c'},
		{"affinity", required_argument, 0, 'a'},
		{"num_tcs", required_argument, 0, 't'},
		{"hash_type", required_argument, 0, 'b'},
		{"eth_start_hdr", no_argument, 0, 's'},
		{"egress_scheduler_params", no_argument, 0, 'q'},
		{"policers_range", required_argument, 0, 'r'},
		{"logical_port_params", no_argument, 0, 'g'},
		{"policer_params", no_argument, 0, 'p'},
		{0, 0, 0, 0}
	};

	garg->cmn_args.pkt_offset = 0;
	garg->cmn_args.busy_wait	= 0;
	garg->cmn_args.echo = 1;
	garg->hash_type = PP2_PPIO_HASH_T_2_TUPLE;
	garg->cmn_args.num_ports = 0;
	garg->cmn_args.cli = 1;
	garg->cmn_args.prefetch_shift = CLS_APP_PREFETCH_SHIFT;

	pp2_args->multi_buffer_release = 1;

	memset(port_params, 0, sizeof(*port_params));
	memset(pp2_params, 0, sizeof(*pp2_params));

	pp2_params->policers_reserved_map = MVAPPS_PP2_POLICERSS_RSRV;
	port->ppio_type = PP2_PPIO_T_NIC;

	/* every time starting getopt we should reset optind */
	optind = 0;
	while ((option = getopt_long(argc, argv, "hi:b:c:t:r:a:epsgq", long_options, &long_index)) != -1) {
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
		case 'e':
			garg->cmn_args.echo = 1;
			break;
		case 'c':
			num_cpus = strtoul(optarg, &ret_ptr, 0);
			break;
		case 'a':
			affinity = strtoul(optarg, &ret_ptr, 0);
			break;
		case 'b':
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
		case 's':
			ppio_tag_mode = true;
			break;
		case 'g':
			logical_port_params = true;
			port->ppio_type = PP2_PPIO_T_LOG;
			break;
		case 'q':
			egress_scheduler_params = true;
			break;
		case 't':
			num_tcs = strtoul(optarg, &ret_ptr, 0);
			break;
		case 'p':
			policer_params = true;
			break;
		case 'r':
			{
				int ranges[2];
				char *ret_ptr, *token;

				token = strtok(optarg, "-");
				ranges[0] = strtoul(token, &ret_ptr, 0);
				token = strtok(NULL, "-");
				ranges[1] = strtoul(token, &ret_ptr, 0);
				if ((ranges[0] < 1 || ranges[0] > PP2_CLS_PLCR_NUM) ||
				    (ranges[1] < 1 || ranges[1] > PP2_CLS_PLCR_NUM) ||
				    (ranges[0] > ranges[1])) {
					printf("parsing fail, wrong input for policers ranges\n");
					return -EINVAL;
				}
				pp2_params->policers_reserved_map = (u32)~0;
				for (i = ranges[0]-1; i <= ranges[1]-1; i++)
					pp2_params->policers_reserved_map &= ~(1 << i);
			}
			break;
		default:
			pr_err("argument (%s) not supported!\n", argv[i]);
			return -EINVAL;
		}
	}

	if (ppio_tag_mode) {
		rc = app_get_line("please enter ethernet start header tag mode\n"
				  "\t\t\teth:--eth			(no argument)\n"
				  "\t\t\tdsa:--dsa			(no argument)\n"
				  "\t\t\textended dsa:--extended_dsa(no argument)\n",
				  buff, sizeof(buff), &argc_cli, argv);
		if (rc) {
			pr_err("app_get_line failed!\n");
			return -EINVAL;
		}
		rc = pp2_cls_cli_ppio_tag_mode(&pp2_args->ports_desc[0].port_params, argc_cli, argv);
		if (rc) {
			pr_err("pp2_cls_cli_ppio_tag_mode failed!\n");
			return -EINVAL;
		}
	}

	if (logical_port_params) {
		rc = app_get_line("please enter logical port params:\n"
				  "\t\t\t--target		(dec)\n"
				  "\t\t\t--num_proto_rule_sets	(dec)\n"
				  "\t\t\t--num_rules		(dec)\n"
				  "\t\t\t--rule_type	(dec)\n"
				  "\t\t\t--proto		(dec)\n"
				  "\t\t\t--proto_val		(dec)\n"
				  "\t\t\t--special_proto	(dec)\n"
				  "\t\t\t--special_fields	(dec)\n"
				  "\t\t\t--field_val		(dec)\n",
				  buff, sizeof(buff), &argc_cli, argv);
		if (rc) {
			pr_err("app_get_line failed!\n");
			return -EINVAL;
		}
		rc = pp2_cls_logical_port_params(&pp2_args->ports_desc[0].port_params, argc_cli, argv);
		if (rc) {
			pr_err("pp2_cls_logical_port_params failed!\n");
			return -EINVAL;
		}
	}

	if (egress_scheduler_params) {
		rc = app_get_line("please enter egress scheduler params:\n"
				  "\t\t\t--port_rate_limit_enable (no argument)\n"
				  "\t\t\t--port_rate_limit	  (kbps min: 100)\n"
				  "\t\t\t--port_burst_size	  (kB min: 64)\n"
				  "\t\t\t--txq_rate_limit_enable  (qid)\n"
				  "\t\t\t--txq_rate_limit	  (qid,kbps min: 100)\n"
				  "\t\t\t--txq_burst_size	  (qid,kB min: 64)\n"
				  "\t\t\t--txq_arb_mode		  (qid,0=wrr 1=fp)\n"
				  "\t\t\t--txq_wrr_weight	  (qid,1-255)\n",
				  buff, sizeof(buff), &argc_cli, argv);
		if (rc) {
			pr_err("egress scheduler app_get_line failed!\n");
			return -EINVAL;
		}
		rc = pp2_egress_scheduler_params(&pp2_args->ports_desc[0].port_params, argc_cli, argv);
		if (rc) {
			pr_err("pp2_egress_scheduler_params failed!\n");
			return -EINVAL;
		}
	}

	if (policer_params) {
		rc = app_get_line("please enter policer params:\n"
				  "\t\t\t--policer_index   (dec)\n"
				  "\t\t\t--token_unit	   (dec)\n"
				  "\t\t\t--color_mode	   (dec)\n"
				  "\t\t\t--cir		   (dec)\n"
				  "\t\t\t--cbs	           (dec)\n"
				  "\t\t\t--ebs		   (dec)\n",
				  buff, sizeof(buff), &argc_cli, argv);
		if (rc) {
			pr_err("app_get_line failed!\n");
			return -EINVAL;
		}

		pp2_args->ports_desc[0].plcr_argc = argc_cli;
		for (i = 0; i < argc_cli; i++)
			pp2_args->ports_desc[0].plcr_argv[i] = strdup(argv[i]);
	}

	/* Now, check validity of all inputs */
	if (!garg->cmn_args.num_ports) {
		pr_err("No port defined!\n");
		return -EINVAL;
	}

	/* Check num_cpus validity */
	if (num_cpus < 0 || num_cpus > system_ncpus()) {
		pr_err("Number of TC'exceeds maximum of %d\n", system_ncpus());
		return -EINVAL;
	}

	garg->cmn_args.cpus = num_cpus;
	pr_debug("cpus: %d\n", garg->cmn_args.cpus);

	/* Check affinity validity */
	if ((affinity != MVAPPS_INVALID_AFFINITY) &&
	    ((garg->cmn_args.cpus + affinity) > system_ncpus())) {
		pr_err("illegal num cores or affinity requested (%d,%d vs %d)!\n",
		       garg->cmn_args.cpus, affinity, system_ncpus());
		return -EINVAL;
	}

	garg->cmn_args.affinity = affinity;

	/* Check num_tcs validity */
	if (num_tcs > PP2_PPIO_MAX_NUM_TCS) {
		pr_err("Number of TC'exceeds maximum of %d\n", PP2_PPIO_MAX_NUM_TCS);
		return -EINVAL;
	}

	/* Check if number of TC's and number of cores are acceptable */
	if ((num_tcs * garg->cmn_args.cpus) > (PP2_PPIO_MAX_NUM_TCS)) {
		pr_err("not enough hw queues to allocate %d TCs and %d CPUs. Needed %d queues, available %d\n",
			num_tcs, garg->cmn_args.cpus, (num_tcs * garg->cmn_args.cpus), PP2_PPIO_MAX_NUM_TCS);
		pr_info("Allowed values: |   TCs  |  CPUs  |\n");
		pr_info("                |-----------------|\n");
		pr_info("                | (1..8) | (1..4) |\n");
		pr_info("                | (8.16) | (1..2) |\n");
		pr_info("                |(16..32)|   1    |\n");
		return -EINVAL;
	}

	garg->num_tcs = num_tcs;
	pr_debug("number of TC's: %d\n", garg->num_tcs);

	/* Fill the core queue map with default values */
	for (i = 0; i < garg->cmn_args.cpus; i++) {
		for (j = 0; j < garg->num_tcs; j++) {
			garg->core_queues[i].tc_inq[j] = i;
			pr_debug("core %d, tc %d, in_queue %d\n", i, j, garg->core_queues[i].tc_inq[j]);
		}
	}

	return 0;
}

int main(int argc, char *argv[])
{
	struct mvapp_params	mvapp_params;
	u64			cores_mask;
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

	cores_mask = apps_cores_mask_create(garg.cmn_args.cpus, garg.cmn_args.affinity);
	garg.cmn_args.cores_mask = cores_mask;

	memset(&mvapp_params, 0, sizeof(mvapp_params));
	mvapp_params.use_cli		= garg.cmn_args.cli;
	mvapp_params.num_cores		= garg.cmn_args.cpus;
	mvapp_params.cores_mask		= cores_mask;
	mvapp_params.global_arg		= (void *)&garg;
	mvapp_params.init_global_cb	= init_global;
	mvapp_params.deinit_global_cb	= apps_pp2_deinit_global;
	mvapp_params.init_local_cb	= init_local;
	mvapp_params.deinit_local_cb	= apps_pp2_deinit_local;
	mvapp_params.main_loop_cb	= main_loop;
	return mvapp_go(&mvapp_params);
}
