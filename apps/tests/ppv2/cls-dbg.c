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
#include "mv_std.h"
#include "mvapp.h"
#include "mv_pp2.h"
#include "mv_pp2_ppio.h"
#include "utils.h"
#include "src/drivers/ppv2/pp2.h"
#include "src/drivers/ppv2/cls/pp2_cls_types.h"
#include "src/drivers/ppv2/cls/pp2_cls_internal_types.h"
#include "src/drivers/ppv2/cls/pp2_c3.h"
#include "src/drivers/ppv2/cls/pp2_flow_rules.h"
#include "src/drivers/ppv2/cls/pp2_cls_db.h"

#define CLS_DBG_PKT_OFFS	64
#define CLS_DBG_DMA_MEM_SIZE	(4*1024*1024)
#define CLS_DBG_PP2_MAX_NUM_TCS_PER_PORT	1
#define CLS_DBG_PP2_MAX_NUM_QS_PER_TC		1

/** Get rid of path in filename - only for unix-type paths using '/' */
#define CLS_DBG_NO_PATH(file_name) (strrchr((file_name), '/') ? \
			    strrchr((file_name), '/') + 1 : (file_name))

struct pp2_ppio {
	struct pp2_port *port;
};

struct glob_arg {
	int			 verbose;
	int			 cli;
	char			 port_name[15];
	struct pp2_hif		*hif;
	struct pp2_ppio		*ppio;
	int			 num_pools;
	struct pp2_bpool	***pools;
	struct pp2_buff_inf	***buffs_inf;
	char			*test_module;
	int			test_number;
	uintptr_t		cpu_slot;
};

static struct glob_arg garg = {};

static int main_loop(void *arg, volatile int *running)
{
	struct glob_arg	*garg = (struct glob_arg *)arg;
	struct pp2_ppio *ppio = garg->ppio;
	struct pp2_port *port = ppio->port;

	garg->cpu_slot = port->cpu_slot;

	if (!garg->cpu_slot) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	if (!garg->test_module) {
		pr_err("no test module selected!\n");
	} else if (strncmp(garg->test_module, "parser", 6) == 0) {
		pr_info("Parser tests not implemented yet\n");
		return -EINVAL;
	} else if (strncmp(garg->test_module, "issue", 5) == 0) {
		pr_info("*************Initialize DB**************\n");
		pp2_cls_db_init();
		pr_info("*************Initialize cls - decoder & issue engine **************\n");
		pp2_cls_init(garg->cpu_slot);
	} else if (strncmp(garg->test_module, "c2", 2) == 0) {
		pr_info("c2 tests not implemented yet\n");
		return -EINVAL;
	} else if (strncmp(garg->test_module, "c3", 2) == 0) {
		if (garg->test_number < 1 || garg->test_number > 5) {
			pr_info("c3 test number not supported\n");
			return -EINVAL;
		}
		pr_info("*************start test c3************\n");
		pp2_cls_c3_test(port->cpu_slot, garg->test_number);
	} else {
		pr_info("wrong engine params\n");
	}

	while (*running);

	return 0;
}

static int init_all_modules(void)
{
	struct pp2_init_params	 pp2_params;
	int			 err;

	pr_info("Global initializations ... ");

	err = mv_sys_dma_mem_init(CLS_DBG_DMA_MEM_SIZE);
	if (err)
		return err;

	memset(&pp2_params, 0, sizeof(pp2_params));
	pp2_params.hif_reserved_map = 0;
	pp2_params.bm_pool_reserved_map = 0;
	pp2_params.ppios[0][0].is_enabled = 1;
	pp2_params.ppios[0][0].first_inq = 0;
	err = pp2_init(&pp2_params);
	if (err)
		return err;

	pr_info("done\n");
	return 0;
}

static int init_local_modules(struct glob_arg *garg)
{
	struct pp2_ppio_params		port_params;
	struct pp2_ppio_inq_params	inq_params;
	int				i, j, err;
	struct bpool_inf 		infs[] = MVAPPS_BPOOLS_INF;

	pr_info("Local initializations ... ");

	err = app_hif_init(&garg->hif);
	if (err)
		return err;

	garg->num_pools = ARRAY_SIZE(infs);
	err = app_build_all_bpools(&garg->pools, &garg->buffs_inf, garg->num_pools, infs, garg->hif);
	if (err)
		return err;

	pr_info("interface: %s\n", garg->port_name);
	memset(&port_params, 0, sizeof(port_params));
	port_params.match = garg->port_name;
	port_params.type = PP2_PPIO_T_NIC;
	port_params.inqs_params.num_tcs = CLS_DBG_PP2_MAX_NUM_TCS_PER_PORT;

	for (i = 0; i < port_params.inqs_params.num_tcs; i++) {
		port_params.inqs_params.tcs_params[0].pkt_offset = CLS_DBG_PKT_OFFS >> 2;
		port_params.inqs_params.tcs_params[0].num_in_qs = CLS_DBG_PP2_MAX_NUM_QS_PER_TC;
		/* TODO: we assume here only one Q per TC; change it! */
		inq_params.size = MVAPPS_Q_SIZE;
		port_params.inqs_params.tcs_params[0].inqs_params = &inq_params;
		for (j = 0; j < garg->num_pools; j++)
			port_params.inqs_params.tcs_params[0].pools[j] = garg->pools[0][j];
	}
	port_params.outqs_params.num_outqs = CLS_DBG_PP2_MAX_NUM_TCS_PER_PORT;
	for (i = 0; i < port_params.outqs_params.num_outqs; i++) {
		port_params.outqs_params.outqs_params[0].size = MVAPPS_Q_SIZE;
		port_params.outqs_params.outqs_params[0].weight = 1;
	}
	err = pp2_ppio_init(&port_params, &garg->ppio);
	if (err)
		return err;

	if (!garg->ppio) {
		pr_err("PP-IO init failed!\n");
		return -EIO;
	}

	err = pp2_ppio_enable(garg->ppio);
	if (err)
		return err;

	err = pp2_ppio_set_uc_promisc(garg->ppio, 1);
	if (err)
		return err;

	pr_info("done\n");
	return 0;
}

static void destroy_local_modules(struct glob_arg *garg)
{
	int	i, j;

	if (garg->ppio) {
		pp2_ppio_disable(garg->ppio);
		pp2_ppio_deinit(garg->ppio);
	}

	if (garg->pools) {
		for (i = 0; i < PP2_SOC_NUM_PACKPROCS; i++) {
			if (garg->pools[i]) {
				for (j = 0; j < garg->num_pools; j++)
					if (garg->pools[i][j])
						pp2_bpool_deinit(garg->pools[i][j]);
				free(garg->pools[i]);
			}
		}
		free(garg->pools);
	}
	if (garg->buffs_inf) {
		for (i = 0; i < PP2_SOC_NUM_PACKPROCS; i++) {
			if (garg->buffs_inf[i]) {
				for (j = 0; j < garg->num_pools; j++)
					if (garg->buffs_inf[i][j])
						free(garg->buffs_inf[i][j]);
				free(garg->buffs_inf[i]);
			}
		}
		free(garg->buffs_inf);
	}

	if (garg->hif)
		pp2_hif_deinit(garg->hif);
}

static void destroy_all_modules(void)
{
	pp2_deinit();
	mv_sys_dma_mem_destroy();
}

static int register_cli_cls_cmds(struct glob_arg *garg)
{
	struct cli_cmd_params cmd_params;
	struct pp2_ppio *ppio = garg->ppio;
	struct pp2_port *port = ppio->port;

	garg->cpu_slot = port->cpu_slot;

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "pp2_cls_lkp_dcod_entry_set";
	cmd_params.desc		= "sets the lookup ID structure in the global lookup decode table";
	cmd_params.format	= "[flow_log_id] [flow_len_max] [cpu_q]";
	cmd_params.cmd_arg	= (void *)garg->cpu_slot;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cli_cls_lkp_dcod_entry_set;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "pp2_cls_lkp_dcod_luid_set";
	cmd_params.desc		= "sets the lookup ID information in the global lookup decode table";
	cmd_params.format	= "[lookup id number ... lookup id number]";
	cmd_params.cmd_arg	= (void *)garg->cpu_slot;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cli_cls_lkp_dcod_luid_set;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "pp2_cls_lkp_dcod_add";
	cmd_params.desc		= "add the entry in the global lookup decode table";
	cmd_params.format	= "";
	cmd_params.cmd_arg	= (void *)garg->cpu_slot;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cli_cls_lkp_dcod_add;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "pp2_cls_lkp_dcod_ena";
	cmd_params.desc		= "enables flow lookup ID in the lookup decode table";
	cmd_params.format	= "[flow_log_id flow_log_id ... flow_log_id]";
	cmd_params.cmd_arg	= (void *)garg->cpu_slot;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cli_cls_lkp_dcod_ena;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "pp2_cls_lkp_dcod_dump";
	cmd_params.desc		= "dump all logical decode DB information";
	cmd_params.format	= "";
	cmd_params.cmd_arg	= (void *)garg->cpu_slot;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cli_cls_lkp_dcod_dump;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "pp2_cls_fl_rule_init";
	cmd_params.desc		= "initializes the global flow rules";
	cmd_params.format	= "";
	cmd_params.cmd_arg	= (void *)garg->cpu_slot;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cli_cls_fl_rule_init;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "pp2_cls_fl_rule_set";
	cmd_params.desc		= "sets a single flow rule in the global flow rules";
	cmd_params.format	= "[fl_log_id] [port_type] [port_bm]\n\t\t\t\t[enabled] [prio] [engine]\n"
				"\t\t\t\[field_id_cnt]\n\t\t\t\t[field_id0] [field_id1] [field_id2] [field_id3]";
	cmd_params.cmd_arg	= (void *)garg->cpu_slot;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cli_cls_fl_rule_set;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "pp2_cls_fl_rule_add";
	cmd_params.desc		= "adds the current global flow rules";
	cmd_params.format	= "";
	cmd_params.cmd_arg	= (void *)garg->cpu_slot;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cli_cls_fl_rule_add;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "pp2_cls_fl_rule_ena";
	cmd_params.desc		= "enables the current global flow rules";
	cmd_params.format	= "";
	cmd_params.cmd_arg	= (void *)garg->cpu_slot;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cli_cls_fl_rule_ena;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "pp2_cls_fl_rule_dis";
	cmd_params.desc		= "disables a logical rule according to input array";
	cmd_params.format	= "[disable number disable number ... disable number]";
	cmd_params.cmd_arg	= (void *)garg->cpu_slot;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cli_cls_fl_rule_dis;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "pp2_cls_lkp_hits_dump";
	cmd_params.desc		= "dump all hit decode entry and its DB information";
	cmd_params.format	= "";
	cmd_params.cmd_arg	= (void *)garg->cpu_slot;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cli_cls_lkp_hits_dump;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "pp2_cls_fl_hits_dump";
	cmd_params.desc		= "dump all hit flow table entry";
	cmd_params.format	= "";
	cmd_params.cmd_arg	= (void *)garg->cpu_slot;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cli_cls_fl_hits_dump;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "pp2_cls_fl_rls_dump";
	cmd_params.desc		= "dump all logical flow ID rules";
	cmd_params.format	= "";
	cmd_params.cmd_arg	= (void *)garg->cpu_slot;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cli_cls_fl_rls_dump;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "pp2_cls_fl_log_rls_dump";
	cmd_params.desc		= "dump all logical flow ID and rule offset";
	cmd_params.format	= "";
	cmd_params.cmd_arg	= (void *)garg->cpu_slot;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cli_cls_fl_log_rls_dump;
	mvapp_register_cli_cmd(&cmd_params);

	return 0;
}


static int register_cli_c3_cmds(struct glob_arg *garg)
{
	struct cli_cmd_params cmd_params;

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "rule_add";
	cmd_params.desc		= "add a C3 rule (fixed 5 tuples...no parameters for now";
	cmd_params.format	= "";
	cmd_params.cmd_arg	= garg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_c3_rule_add;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "rule_remove";
	cmd_params.desc		= "remove a C3 rule according to logic_index";
	cmd_params.format	= "[logix_index] (dec) logical index of the rule to be removed";
	cmd_params.cmd_arg	= garg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_c3_rule_delete;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "entry_dump";
	cmd_params.desc		= "dump C3 entries according to type and index";
	cmd_params.format	= "[type] [var]\n"
				  "\t\t\ttype (dec)C3 dump type, 0: logic idx, 1:hash idx, 2:lookup type, 3:all\n"
				  "\t\t\tvar (dec)value according to type, type0/1:idx, type 2: lookup type,type 3:0\n";
	cmd_params.cmd_arg	= garg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_c3_type_entry_dump;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "index_dump";
	cmd_params.desc		= " dump two index tables";
	cmd_params.format	= "[all] (dec)all index entry or only valid one, 0: valid, 1:all";
	cmd_params.cmd_arg	= garg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_c3_index_entry_dump;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "scan_param_set";
	cmd_params.desc		= "set scan mode";
	cmd_params.format	= "[clear] [lkp_type_en] [lkp_type] [mode] [start] [delay] [threshold]\n"
				  "\t\t\t\tclear        (dec)clear before scan, 0:not clear, 1:clear\n"
				  "\t\t\t\tlkp_type_en  (dec)scan by lookup type, 0:disable, 1:enable\n"
				  "\t\t\t\tlkp_type     (dec)lookup type value, from 0 to 15\n"
				  "\t\t\t\tmode         (dec)scan mode, 0:below threshold, 1:above threshold\n"
				  "\t\t\t\tdelay        (dec)scan delay time in unit of 256 core clock cycles\n"
				  "\t\t\t\tthreshold    (dec)scan threshold\n";
	cmd_params.cmd_arg	= garg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_c3_scan_param_set;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "scan_result_get";
	cmd_params.desc		= "dumps C3 entries scan result";
	cmd_params.format	= "[max_entry] (dec)max scan entry number";
	cmd_params.cmd_arg	= garg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_c3_scan_result_get;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "hit_count_get";
	cmd_params.desc		= "get hit counter by logical index";
	cmd_params.format	= "[logic_idx] (dec)logical index, 0...4095";
	cmd_params.cmd_arg	= garg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_c3_hit_count_get;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "search_depth_set";
	cmd_params.desc		= "set C3 cuckoo search depth (default 3)";
	cmd_params.format	= "[search_depth] (dec)cuckoo search depth, 1...8";
	cmd_params.cmd_arg	= garg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_c3_search_depth_set;
	mvapp_register_cli_cmd(&cmd_params);

	return 0;
}


static int register_cli_cmds(struct glob_arg *garg)
{
	if (!garg->cli)
		return -EFAULT;

	register_cli_cls_cmds(garg);
	register_cli_c3_cmds(garg);
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

static int init_local(void *arg, int id, void **larg)
{
	struct glob_arg *garg = (struct glob_arg *)arg;

	if (!garg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	*larg = garg;
	return 0;
}

static void deinit_local(void *arg)
{
	struct glob_arg *garg = (struct glob_arg *)arg;

	if (!garg)
		return;
}

static void usage(char *progname)
{
	printf("\n"
		"MUSDK cls-test application.\n"
		"\n"
		"Usage: %s OPTIONS\n"
	"  E.g. %s -i eth0,eth1 -c 1\n"
	    "\n"
	    "Mandatory OPTIONS:\n"
		"\t-i, --interface <port-interface>\n"
		"\t	  ppio-0:0 - A7040/A8040 10Gb interface 1\n"
		"\t	  ppio-0:1 - A7040/A8040  1Gb interface 1\n"
		"\t	  ppio-0:2 - A7040        1Gb interface 2\n"
		"\t	  ppio-1:0 - A8040       10Gb interface 2\n"
		"\t	  ppio-1:1 - A8040 	  1Gb interface 2\n"
		"\n"
	    "Optional OPTIONS:\n"
		"\t-m <test module>	test module <parser,issue,c2,c3,rss>\n"
		"\t-n <test number>	select test number\n"
		"\t--cli			use CLI\n"
		"\n", CLS_DBG_NO_PATH(progname), CLS_DBG_NO_PATH(progname)
		);
}

static int parse_args(struct glob_arg *garg, int argc, char *argv[])
{
	int	i = 1;

	garg->cli = 0;
	garg->test_module = NULL;
	garg->test_number = 0;

	while (i < argc) {
		if ((strcmp(argv[i], "?") == 0) ||
		    (strcmp(argv[i], "-h") == 0) ||
		    (strcmp(argv[i], "--help") == 0)) {
			usage(argv[0]);
			exit(0);
		} else if (strcmp(argv[i], "-i") == 0) {
			if (argc < (i+2)) {
				pr_err("Invalid number of arguments!\n");
				return -EINVAL;
			}
			if (argv[i+1][0] == '-') {
				pr_err("Invalid arguments format!\n");
				return -EINVAL;
			}
			snprintf(garg->port_name, sizeof(garg->port_name), "%s", argv[i+1]);
			i += 2;
		} else if (strcmp(argv[i], "-m") == 0) {
			if (argc < i + 2) {
				pr_err("Invalid number of arguments!\n");
				return -EINVAL;
			}
			if (argv[i + 1][0] == '-') {
				pr_err("Invalid arguments format!\n");
				return -EINVAL;
			}
			garg->test_module = &argv[i + 1][0];
			i += 2;
		} else if (strcmp(argv[i], "-n") == 0) {
			if (argc < i + 2) {
				pr_err("Invalid number of arguments!\n");
				return -EINVAL;
			}
			if (argv[i + 1][0] == '-') {
				pr_err("Invalid arguments format!\n");
				return -EINVAL;
			}
			garg->test_number = atoi(argv[i + 1]);
			i += 2;
		}  else if (strcmp(argv[i], "--cli") == 0) {
			garg->cli = 1;
			i += 1;
		} else {
			pr_err("argument (%s) not supported!\n", argv[i]);
			return -EINVAL;
		}
	}

	/* Now, check validity of all inputs */
	if (!garg->port_name[0]) {
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

	memset(&mvapp_params, 0, sizeof(mvapp_params));
	mvapp_params.use_cli		= garg.cli;
	mvapp_params.num_cores		= 1;
	mvapp_params.global_arg		= (void *)&garg;
	mvapp_params.init_global_cb	= init_global;
	mvapp_params.deinit_global_cb	= deinit_global;
	mvapp_params.init_local_cb	= init_local;
	mvapp_params.deinit_local_cb	= deinit_local;
	mvapp_params.main_loop_cb	= main_loop;
	return mvapp_go(&mvapp_params);
}
