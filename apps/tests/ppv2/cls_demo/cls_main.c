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

#include "src/drivers/ppv2/pp2.h"

#include "cls_debug.h"
#include "cls_main.h"

#define CLS_APP_DMA_MEM_SIZE			(10 * 1024 * 1024)
#define CLS_APP_QS_MAP_MASK			0xFFFF
#define CLS_APP_FIRST_LOG_PORT_IN_QUEUE		4
#define CLS_APP_FIRST_MUSDK_IN_QUEUE		0
#define CLS_APP_COMMAND_LINE_SIZE		256
#define CLS_APP_DEF_Q_SIZE			1024
#define CLS_APP_HIF_Q_SIZE			CLS_APP_DEF_Q_SIZE
#define CLS_APP_RX_Q_SIZE			CLS_APP_DEF_Q_SIZE
#define CLS_APP_TX_Q_SIZE			CLS_APP_DEF_Q_SIZE

#define CLS_APP_MAX_BURST_SIZE			(CLS_APP_RX_Q_SIZE >> 1)
#define CLS_APP_DFLT_BURST_SIZE			256

#define CLS_APP_MAX_NUM_QS_PER_CORE		MVAPPS_PP2_MAX_NUM_QS_PER_TC

#define CLS_APP_KEY_MEM_SIZE_MAX		(PP2_CLS_TBL_MAX_NUM_FIELDS * CLS_APP_STR_SIZE_MAX)

#define CLS_APP_PREFETCH_SHIFT			7
#define CLS_APP_PKT_ECHO_SUPPORT
#define CLS_APP_USE_APP_PREFETCH

#define CLS_APP_BPOOLS_INF		{ {2048, 1024} }

struct glob_arg {
	struct glb_common_args	cmn_args;  /* Keep first */

	u32				hash_type;
	struct pp2_init_params		pp2_params;
};

struct local_arg {
	struct local_common_args	cmn_args;  /* Keep first */
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
		if ((tbl_node->idx == 0) || ((tbl_node->idx - idx) > 1))
			return idx + 1;
		idx++;
	}
	return idx + 1;
}


static inline int loop_sw_recycle(struct local_arg	*larg,
				  u8			 rx_ppio_id,
				  u8			 tx_ppio_id,
				  u8			 bpool_id,
				  u8			 tc,
				  u8			 qid,
				  u16			 num)
{
	struct pp2_bpool *bpool;
	struct tx_shadow_q *shadow_q;
	struct pp2_buff_inf *binf;
	struct pp2_ppio_desc descs[CLS_APP_MAX_BURST_SIZE];
	struct pp2_lcl_common_args *pp2_args = (struct pp2_lcl_common_args *) larg->cmn_args.plat;
	int err;
	u16 i;
#ifdef CLS_APP_PKT_ECHO_SUPPORT
	int prefetch_shift = CLS_APP_PREFETCH_SHIFT;
#endif /* CLS_APP_PKT_ECHO_SUPPORT */
	bpool = pp2_args->pools_desc[pp2_args->lcl_ports_desc[rx_ppio_id].pp_id][bpool_id].pool;
	shadow_q = &pp2_args->lcl_ports_desc[tx_ppio_id].shadow_qs[tc];

	err = pp2_ppio_recv(pp2_args->lcl_ports_desc[rx_ppio_id].ppio, tc, qid, descs, &num);

	for (i = 0; i < num; i++) {
		char		*buff = (char *)(uintptr_t)pp2_ppio_inq_desc_get_cookie(&descs[i]);
		dma_addr_t	 pa = pp2_ppio_inq_desc_get_phys_addr(&descs[i]);

		u16 len = pp2_ppio_inq_desc_get_pkt_len(&descs[i]) - MV_MH_SIZE;

#ifdef CLS_APP_PKT_ECHO_SUPPORT
		if (likely(larg->cmn_args.echo)) {
			char *tmp_buff;
#ifdef CLS_APP_USE_APP_PREFETCH
			if (num - i > prefetch_shift) {
				tmp_buff = (char *)(uintptr_t)pp2_ppio_inq_desc_get_cookie(&descs[i + prefetch_shift]);
				tmp_buff += MVAPPS_PP2_PKT_DEF_EFEC_OFFS;
				pr_debug("tmp_buff_before(%p)\n", tmp_buff);
				tmp_buff = (char *)(((uintptr_t)tmp_buff) | sys_dma_high_addr);
				pr_debug("tmp_buff_after(%p)\n", tmp_buff);
				prefetch(tmp_buff);
			}
#endif /* CLS_APP_USE_APP_PREFETCH */
			tmp_buff = (char *)(((uintptr_t)(buff)) | sys_dma_high_addr);
			pr_debug("buff2(%p)\n", tmp_buff);
			tmp_buff += MVAPPS_PP2_PKT_DEF_EFEC_OFFS;
			swap_l2(tmp_buff);
			swap_l3(tmp_buff);
		}
#endif /* CLS_APP_PKT_ECHO_SUPPORT */
		pp2_ppio_outq_desc_reset(&descs[i]);
		pp2_ppio_outq_desc_set_phys_addr(&descs[i], pa);
		pp2_ppio_outq_desc_set_pkt_offset(&descs[i], MVAPPS_PP2_PKT_DEF_EFEC_OFFS);
		pp2_ppio_outq_desc_set_pkt_len(&descs[i], len);
		shadow_q->ents[shadow_q->write_ind].buff_ptr.cookie = (uintptr_t)buff;
		shadow_q->ents[shadow_q->write_ind].buff_ptr.addr = pa;
		pr_debug("buff_ptr.cookie(0x%lx)\n", (u64)shadow_q->ents[shadow_q->write_ind].buff_ptr.cookie);
		shadow_q->write_ind++;
		if (shadow_q->write_ind == CLS_APP_TX_Q_SIZE)
			shadow_q->write_ind = 0;
	}

	if (num) {
		err = pp2_ppio_send(pp2_args->lcl_ports_desc[tx_ppio_id].ppio, pp2_args->hif,
				    tc, descs, &num);
		if (err) {
			pr_err("pp2_ppio_send\n");
			return err;
		}
	}

	pp2_ppio_get_num_outq_done(pp2_args->lcl_ports_desc[tx_ppio_id].ppio, pp2_args->hif, tc, &num);
	for (i = 0; i < num; i++) {
		binf = &shadow_q->ents[shadow_q->read_ind].buff_ptr;
		if (unlikely(!binf->cookie || !binf->addr)) {
			pr_err("Shadow memory @%d: cookie(%lx), pa(%lx)!\n",
			       shadow_q->read_ind, (u64)binf->cookie, (u64)binf->addr);
			continue;
		}
		pp2_bpool_put_buff(pp2_args->hif,
				   bpool,
				   binf);
		shadow_q->read_ind++;
		if (shadow_q->read_ind == CLS_APP_TX_Q_SIZE)
			shadow_q->read_ind = 0;
	}
	return 0;
}

static int loop_1p(struct local_arg *larg, int *running)
{
	int err;
	u16 num;
	u8 tc = 0, qid = 0;

	if (!larg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	num = CLS_APP_DFLT_BURST_SIZE;

	while (*running) {
		/* Find next queue to consume */
		do {
			qid++;
			if (qid == MVAPPS_PP2_MAX_NUM_QS_PER_TC) {
				qid = 0;
				tc++;
				if (tc == CLS_APP_MAX_NUM_TCS_PER_PORT)
					tc = 0;
			}
		} while (!(larg->cmn_args.qs_map & (1 << ((tc * MVAPPS_PP2_MAX_NUM_QS_PER_TC) + qid))));
		err = loop_sw_recycle(larg, 0, 0, 0, tc, qid, num);
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
	register_cli_cls_api_qos_cmds(ppio);
	register_cli_filter_cmds(ppio);
	register_cli_cls_cmds(ppio);
	register_cli_c3_cmds(ppio);
	register_cli_c2_cmds(ppio);
	register_cli_qos_cmds(ppio);
	register_cli_mng_cmds(ppio);
	app_register_cli_common_cmds(pp2_args->ports_desc);

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
	lcl_pp2_args->lcl_ports_desc = (struct lcl_port_desc *)
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

	larg->cmn_args.id		= id;
	larg->cmn_args.echo		= garg->cmn_args.echo;
	larg->cmn_args.num_ports		= garg->cmn_args.num_ports;
	memset(lcl_pp2_args->lcl_ports_desc, 0, larg->cmn_args.num_ports * sizeof(struct lcl_port_desc));
	for (i = 0; i < larg->cmn_args.num_ports; i++)
		app_port_local_init(i, larg->cmn_args.id, &lcl_pp2_args->lcl_ports_desc[i],
				    &glb_pp2_args->ports_desc[i]);

	lcl_pp2_args->pools_desc	= glb_pp2_args->pools_desc;
	larg->cmn_args.garg              = garg;
	garg->cmn_args.largs[id] = larg;
	larg->cmn_args.qs_map = garg->cmn_args.qs_map << (garg->cmn_args.qs_map_shift * id);

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
		"\t-b, --hash_type <none, 2-tuple, 5-tuple>\n"
		"\t--ppio_tag_mode		(no argument)configure ppio_tag_mode parameter\n"
		"\t--logical_port_params	(no argument)configure logical port parameters\n"
		"\n", MVAPPS_NO_PATH(progname), MVAPPS_NO_PATH(progname)
		);
}

static int parse_args(struct glob_arg *garg, int argc, char *argv[])
{
	int i = 1;
	int option;
	int long_index = 0;
	int ppio_tag_mode = 0;
	int logical_port_params = 0;
	char buff[CLS_APP_COMMAND_LINE_SIZE];
	int argc_cli;
	int rc;

	struct pp2_glb_common_args *pp2_args = (struct pp2_glb_common_args *) garg->cmn_args.plat;
	struct pp2_ppio_params	*port_params = &pp2_args->ports_desc[0].port_params;
	struct pp2_init_params	*pp2_params = &garg->pp2_params;
	struct port_desc	*port = &pp2_args->ports_desc[0];


	struct option long_options[] = {
		{"help", no_argument, 0, 'h'},
		{"interface", required_argument, 0, 'i'},
		{"echo", no_argument, 0, 'e'},
		{"hash_type", required_argument, 0, 'b'},
		{"ppio_tag_mode", no_argument, 0, 't'},
		{"logical_port_params", no_argument, 0, 'g'},
		{0, 0, 0, 0}
	};

	garg->cmn_args.cpus = 1;
	garg->cmn_args.qs_map = CLS_APP_QS_MAP_MASK;
	garg->cmn_args.qs_map_shift = CLS_APP_MAX_NUM_TCS_PER_PORT;
	garg->cmn_args.pkt_offset = 0;

	garg->cmn_args.echo = 0;
	garg->hash_type = PP2_PPIO_HASH_T_2_TUPLE;
	garg->cmn_args.num_ports = 0;
	garg->cmn_args.cli = 1;

	memset(port_params, 0, sizeof(*port_params));
	memset(pp2_params, 0, sizeof(*pp2_params));
	port->ppio_type = PP2_PPIO_T_NIC;

	/* every time starting getopt we should reset optind */
	optind = 0;
	while ((option = getopt_long(argc, argv, "hi:b:etg", long_options, &long_index)) != -1) {
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
		case 't':
			ppio_tag_mode = true;
			break;
		case 'g':
			logical_port_params = true;
			port->ppio_type = PP2_PPIO_T_LOG;
			break;
		default:
			pr_err("argument (%s) not supported!\n", argv[i]);
			return -EINVAL;
		}
	}

	if (ppio_tag_mode) {
		rc = app_get_line("please enter tag_mode\n"
				  "\t\t\tno tag:--none			(no argument)\n"
				  "\t\t\tdsa tag:--dsa			(no argument)\n"
				  "\t\t\textended dsa tag:--extended_dsa(no argument)\n",
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
