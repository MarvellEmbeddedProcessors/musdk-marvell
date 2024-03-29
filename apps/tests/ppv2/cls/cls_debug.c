/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <string.h>
#include <stdio.h>
#include "mv_std.h"
#include "mvapp.h"
#include "mv_pp2.h"
#include "mv_pp2_ppio.h"
#include "src/drivers/ppv2/pp2.h"
#include "src/drivers/ppv2/cls/pp2_cls_types.h"
#include "src/drivers/ppv2/cls/pp2_cls_internal_types.h"
#include "src/drivers/ppv2/cls/pp2_c3.h"
#include "src/drivers/ppv2/cls/pp2_c2.h"
#include "src/drivers/ppv2/cls/pp2_flow_rules.h"
#include "src/drivers/ppv2/cls/pp2_cls_db.h"
#include "src/drivers/ppv2/cls/pp2_prs.h"
#include "src/drivers/ppv2/cls/pp2_rss.h"
#include "src/drivers/ppv2/pp2_port.h"
#include "cls_debug.h"

int register_cli_cls_cmds(struct pp2_ppio *ppio)
{
	struct cli_cmd_params cmd_params;
	struct pp2_port *port = GET_PPIO_PORT(ppio);
	struct pp2_inst *inst = port->parent;

#ifdef CLS_DEBUG
	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "pp2_cls_lkp_dcod_entry_set";
	cmd_params.desc		= "sets the lookup ID structure in the global lookup decode table";
	cmd_params.format	= "[flow_log_id] [flow_len_max] [cpu_q]";
	cmd_params.cmd_arg	= (void *)inst;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cli_cls_lkp_dcod_entry_set;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "pp2_cls_lkp_dcod_luid_set";
	cmd_params.desc		= "sets the lookup ID information in the global lookup decode table";
	cmd_params.format	= "[lookup id number ... lookup id number]";
	cmd_params.cmd_arg	= (void *)inst;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cli_cls_lkp_dcod_luid_set;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "pp2_cls_lkp_dcod_add";
	cmd_params.desc		= "add the entry in the global lookup decode table";
	cmd_params.format	= "";
	cmd_params.cmd_arg	= (void *)inst;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cli_cls_lkp_dcod_add;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "pp2_cls_lkp_dcod_ena";
	cmd_params.desc		= "enables flow lookup ID in the lookup decode table";
	cmd_params.format	= "[flow_log_id flow_log_id ... flow_log_id]";
	cmd_params.cmd_arg	= (void *)inst;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cli_cls_lkp_dcod_ena;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "pp2_cls_lkp_dcod_dump";
	cmd_params.desc		= "dump all logical decode DB information";
	cmd_params.format	= "";
	cmd_params.cmd_arg	= (void *)inst;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cli_cls_lkp_dcod_dump;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "pp2_cls_fl_rule_init";
	cmd_params.desc		= "initializes the global flow rules";
	cmd_params.format	= "";
	cmd_params.cmd_arg	= (void *)inst;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cli_cls_fl_rule_init;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "pp2_cls_fl_rule_set";
	cmd_params.desc		= "sets a single flow rule in the global flow rules";
	cmd_params.format	= "[fl_log_id] [port_type] [port_bm]\n\t\t\t\t[enabled] [prio] [engine]\n"
				"\t\t\t\[field_id_cnt]\n\t\t\t\t[field_id0] [field_id1] [field_id2] [field_id3]";
	cmd_params.cmd_arg	= (void *)inst;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cli_cls_fl_rule_set;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "pp2_cls_fl_rule_add";
	cmd_params.desc		= "adds the current global flow rules";
	cmd_params.format	= "";
	cmd_params.cmd_arg	= (void *)inst;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cli_cls_fl_rule_add;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "pp2_cls_fl_rule_ena";
	cmd_params.desc		= "enables the current global flow rules";
	cmd_params.format	= "";
	cmd_params.cmd_arg	= (void *)inst;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cli_cls_fl_rule_ena;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "pp2_cls_fl_rule_dis";
	cmd_params.desc		= "disables a logical rule according to input array";
	cmd_params.format	= "[disable number disable number ... disable number]";
	cmd_params.cmd_arg	= (void *)inst;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cli_cls_fl_rule_dis;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_fl_log_rls_dump";
	cmd_params.desc		= "dump all logical flow ID and rule offset";
	cmd_params.format	= "";
	cmd_params.cmd_arg	= (void *)inst;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cli_cls_fl_log_rls_dump;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "pp2_rss_mode";
	cmd_params.desc		= "set rss mode";
	cmd_params.format	= "2 tuple - 0\n"
				  "\t\t\t\t5 tuple - 1\n";
	cmd_params.cmd_arg	= (void *)port;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cli_cls_set_rss_mode;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_flow_dump";
	cmd_params.desc		= "dump all flows table entry";
	cmd_params.format	= "";
	cmd_params.cmd_arg	= (void *)inst;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cli_cls_flow_dump;
	mvapp_register_cli_cmd(&cmd_params);
#endif

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_lkp_hits_dump";
	cmd_params.desc		= "dump all hit decode entry and its DB information";
	cmd_params.format	= "";
	cmd_params.cmd_arg	= (void *)inst;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cli_cls_lkp_hits_dump;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_fl_hits_dump";
	cmd_params.desc		= "dump all hit flow table entry";
	cmd_params.format	= "";
	cmd_params.cmd_arg	= (void *)inst;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cli_cls_fl_hits_dump;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_fl_rls_dump";
	cmd_params.desc		= "dump all logical flow ID rules";
	cmd_params.format	= "";
	cmd_params.cmd_arg	= (void *)inst;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cli_cls_fl_rls_dump;
	mvapp_register_cli_cmd(&cmd_params);

	return 0;
}

int register_cli_c3_cmds(struct pp2_ppio *ppio)
{
	struct cli_cmd_params cmd_params;
	struct pp2_port *port = GET_PPIO_PORT(ppio);
	struct pp2_inst *inst = port->parent;

#ifdef CLS_DEBUG
	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "rule_add";
	cmd_params.desc		= "add a C3 rule (fixed 5 tuples...no parameters for now";
	cmd_params.format	= "";
	cmd_params.cmd_arg	= (void *)inst;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_c3_rule_add;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "rule_remove";
	cmd_params.desc		= "remove a C3 rule according to logic_index";
	cmd_params.format	= "[logix_index] (dec) logical index of the rule to be removed";
	cmd_params.cmd_arg	= (void *)inst;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_c3_rule_delete;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "index_dump";
	cmd_params.desc		= " dump two index tables";
	cmd_params.format	= "[all] (dec)all index entry or only valid one, 0: valid, 1:all";
	cmd_params.cmd_arg	= (void *)inst;
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
	cmd_params.cmd_arg	= (void *)inst;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_c3_scan_param_set;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "scan_result_get";
	cmd_params.desc		= "dumps C3 entries scan result";
	cmd_params.format	= "[max_entry] (dec)max scan entry number";
	cmd_params.cmd_arg	= (void *)inst;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_c3_scan_result_get;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "hit_count_get";
	cmd_params.desc		= "get hit counter by logical index";
	cmd_params.format	= "[logic_idx] (dec)logical index, 0...4095";
	cmd_params.cmd_arg	= (void *)inst;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_c3_hit_count_get;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "search_depth_set";
	cmd_params.desc		= "set C3 cuckoo search depth (default 3)";
	cmd_params.format	= "[search_depth] (dec)cuckoo search depth, 1...8";
	cmd_params.cmd_arg	= (void *)inst;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_c3_search_depth_set;
	mvapp_register_cli_cmd(&cmd_params);

#endif
	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_c3_rule_hit_dump";
	cmd_params.desc		= "dump C3 entries according to type and index";
	cmd_params.format       = "[type] [var] or no arguments\n"
				  "\t\t\t\ttype (dec) C3 dump type, 0: logic idx, 1:hash idx, 2:lookup type\n"
				  "\t\t\t\tvar  (dec) value according to type, type 0/1:idx, type 2: lookup type\n"
				  "\t\t\t\tno arguments -> dumping all flows\n";
	cmd_params.cmd_arg	= (void *)inst;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_c3_type_entry_dump;
	mvapp_register_cli_cmd(&cmd_params);

	return 0;
}

int register_cli_c2_cmds(struct pp2_ppio *ppio)
{
	struct cli_cmd_params cmd_params;
	struct pp2_port *port = GET_PPIO_PORT(ppio);
	struct pp2_inst *inst = port->parent;

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_c2_rule_hit_dump";
	cmd_params.desc		= "dump all entries with the lookup type";
	cmd_params.format       = "[type] or no arguments\n"
				  "\t\t\t\ttype (dec) lookup type number 0 - 64\n"
				  "\t\t\t\tno arguments -> dumping all flows\n";
	cmd_params.cmd_arg	= (void *)inst;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_c2_lkp_type_entry_dump;
	mvapp_register_cli_cmd(&cmd_params);


#ifdef CLS_DEBUG
	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_c2_valid_lkp_type_dump";
	cmd_params.desc		= "dump valid C2 lookup type, with its TCAM index";
	cmd_params.format	= "(no arguments)\n";
	cmd_params.cmd_arg	= (void *)inst;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_c2_valid_lkp_type_dump;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_c2_free_tcam_dump";
	cmd_params.desc		= "dump all free C2 TCAM entry index";
	cmd_params.format	= "(no arguments)\n";
	cmd_params.cmd_arg	= (void *)inst;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_c2_free_tcam_dump;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_c2_rule_hw_dump";
	cmd_params.desc		= "dump C2 entries";
	cmd_params.format	= "(no arguments)\n";
	cmd_params.cmd_arg	= (void *)inst;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_c2_hw_dump;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_c2_rule_hw_hit_dump";
	cmd_params.desc		= "dump C2 hits according";
	cmd_params.format	= "(no arguments)\n";
	cmd_params.cmd_arg	= (void *)inst;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_c2_hw_hit_dump;
	mvapp_register_cli_cmd(&cmd_params);
#endif
	return 0;
}

int register_cli_qos_cmds(struct pp2_ppio *ppio)
{
	struct cli_cmd_params cmd_params;
	struct pp2_port *port = GET_PPIO_PORT(ppio);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_qos_pcp_table_dump";
	cmd_params.desc		= "dump pcp qos table";
	cmd_params.format	= "(no arguments)\n";
	cmd_params.cmd_arg	= (void *)port;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_qos_pcp_dump;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_qos_dscp_table_dump";
	cmd_params.desc		= "dump dscp qos table";
	cmd_params.format	= "(no arguments)\n";
	cmd_params.cmd_arg	= (void *)port;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_qos_dscp_dump;
	mvapp_register_cli_cmd(&cmd_params);

	return 0;
}

int register_cli_mng_cmds(struct pp2_ppio *ppio)
{
	struct cli_cmd_params cmd_params;

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_mng_table_dump";
	cmd_params.desc		= "dump all tables and defined rules in manager db";
	cmd_params.format	= "(no arguments)\n";
	cmd_params.cmd_arg	= NULL;
	cmd_params.do_cmd_cb	= (void *)pp2_cls_db_mng_tbl_list_dump;
	mvapp_register_cli_cmd(&cmd_params);
	return 0;
}

static int pp2_cli_cls_prs_dump(void *arg, int argc, char *argv[])
{
	struct pp2_inst *inst = (struct pp2_inst *)arg;

	mv_pp2x_prs_hw_dump(inst);
	return 0;
}

static int pp2_cli_cls_prs_hits_dump(void *arg, int argc, char *argv[])
{
	struct pp2_inst *inst = (struct pp2_inst *)arg;

	mv_pp2x_prs_hw_hits_dump(inst);
	return 0;
}

int register_cli_prs_cmds(struct pp2_ppio *ppio)
{
	struct cli_cmd_params cmd_params;
	struct pp2_port *port = GET_PPIO_PORT(ppio);
	struct pp2_inst *inst = port->parent;
#ifdef CLS_DEBUG

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "prs_log_port_negated_proto_dump";
	cmd_params.desc		= "dumps negated protocols in logical port";
	cmd_params.format	= "(no arguments)\n";
	cmd_params.cmd_arg	= (void *)inst;
	cmd_params.do_cmd_cb	= (void *)pp2_cli_cls_db_prs_tcam_neg_proto_dump;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "prs_log_port_match_list_dump";
	cmd_params.desc		= "dumps existing parser entries which were matched when starting logical port";
	cmd_params.format	= "(no arguments)\n";
	cmd_params.cmd_arg	= (void *)inst;
	cmd_params.do_cmd_cb	= (void *)pp2_cli_cls_db_prs_match_list_dump;
	mvapp_register_cli_cmd(&cmd_params);
#endif

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "prs_dump";
	cmd_params.desc		= "dumps existing parser entries";
	cmd_params.format	= "(no arguments)\n";
	cmd_params.cmd_arg	= (void *)inst;
	cmd_params.do_cmd_cb	= (void *)pp2_cli_cls_prs_dump;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "prs_hits_dump";
	cmd_params.desc		= "dumps hits in parser entries";
	cmd_params.format	= "(no arguments)\n";
	cmd_params.cmd_arg	= (void *)inst;
	cmd_params.do_cmd_cb	= (void *)pp2_cli_cls_prs_hits_dump;
	mvapp_register_cli_cmd(&cmd_params);

	return 0;
}

static int pp2_cli_cls_rxq_counters_dump(void *arg, int argc, char *argv[])
{
	struct pp2_port *port = (struct pp2_port *)arg;
	int ret, i;
	u32 queue = 0;

	if (argc != 1 && argc != 2) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	if (argc == 2) {
		ret = kstrtou32(argv[1], 10, &queue);
		if (ret || (queue >= PP2_PPIO_MAX_NUM_INQS)) {
			pr_err("parsing fail, wrong input for arg[1]\n");
			return -EINVAL;
		}
		pp2_port_rxq_cntrs_dump(port, queue);
	}

	if (argc == 1) {
		for (i = 0; i < PP2_PPIO_MAX_NUM_INQS; i++)
			pp2_port_rxq_cntrs_dump(port, i);
	}

	return 0;
}

static int pp2_cli_cls_txq_counters_dump(void *arg, int argc, char *argv[])
{
	struct pp2_port *port = (struct pp2_port *)arg;
	int ret, i;
	u32 queue = 0;

	if (argc != 1 && argc != 2) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	if (argc == 2) {
		ret = kstrtou32(argv[1], 10, &queue);
		if (ret || (queue >= PP2_PPIO_MAX_NUM_OUTQS)) {
			pr_err("parsing fail, wrong input for arg[1]\n");
			return -EINVAL;
		}
		pp2_port_txq_cntrs_dump(port, queue);
	}

	if (argc == 1) {
		for (i = 0; i < PP2_PPIO_MAX_NUM_OUTQS; i++)
			pp2_port_txq_cntrs_dump(port, i);
	}

	return 0;
}

int register_cli_cntrs_cmds(struct pp2_ppio *ppio)
{
	struct cli_cmd_params cmd_params;
	struct pp2_port *port = GET_PPIO_PORT(ppio);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "hw_rx_counters";
	cmd_params.desc		= "dumps hw rx counters registers";
	cmd_params.format       = "[queue] or no arguments\n"
				  "\t\t\t\tqueue (dec) queue number 0 - 31\n"
				  "\t\t\t\tno arguments -> dumping all queues\n";
	cmd_params.cmd_arg	= (void *)port;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cli_cls_rxq_counters_dump;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "hw_tx_counters";
	cmd_params.desc		= "dumps hw tx counters registers";
	cmd_params.format       = "[queue] or no arguments\n"
				  "\t\t\t\tqueue (dec) queue number 0 - 7\n"
				  "\t\t\t\tno arguments -> dumping all queues\n";
	cmd_params.cmd_arg	= (void *)port;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cli_cls_txq_counters_dump;
	mvapp_register_cli_cmd(&cmd_params);


	return 0;
}

static int pp2_cli_cls_rss_dump(void *arg, int argc, char *argv[])
{
	struct pp2_inst *inst = (struct pp2_inst *)arg;

	pp2_cls_rss_hw_dump(inst);
	return 0;
}

static int pp2_cli_cls_rss_rxq_dump(void *arg, int argc, char *argv[])
{
	struct pp2_inst *inst = (struct pp2_inst *)arg;

	pp2_cls_rss_hw_rxq_tbl_dump(inst);
	return 0;
}

int register_cli_rss_cmds(struct pp2_ppio *ppio)
{
	struct cli_cmd_params cmd_params;
	struct pp2_port *port = GET_PPIO_PORT(ppio);
	struct pp2_inst *inst = port->parent;

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "rss_table_dump";
	cmd_params.desc		= "dumps hw rx counters registers";
	cmd_params.format	= "(no arguments)\n";
	cmd_params.cmd_arg	= (void *)inst;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cli_cls_rss_dump;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "rss_rxq_table_dump";
	cmd_params.desc		= "dumps hw rss rxq table";
	cmd_params.format	= "(no arguments)\n";
	cmd_params.cmd_arg	= (void *)inst;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cli_cls_rss_rxq_dump;
	mvapp_register_cli_cmd(&cmd_params);

	return 0;
}


