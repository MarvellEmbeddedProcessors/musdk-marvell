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
#include "src/drivers/ppv2/pp2.h"
#include "src/drivers/ppv2/cls/pp2_cls_types.h"
#include "src/drivers/ppv2/cls/pp2_cls_internal_types.h"
#include "src/drivers/ppv2/cls/pp2_c3.h"
#include "src/drivers/ppv2/cls/pp2_c2.h"
#include "src/drivers/ppv2/cls/pp2_flow_rules.h"
#include "src/drivers/ppv2/cls/pp2_cls_db.h"
#include "cls_debug.h"

int register_cli_cls_cmds(uintptr_t cpu_slot)
{
	struct cli_cmd_params cmd_params;


#ifdef CLS_DEBUG
	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "pp2_cls_lkp_dcod_entry_set";
	cmd_params.desc		= "sets the lookup ID structure in the global lookup decode table";
	cmd_params.format	= "[flow_log_id] [flow_len_max] [cpu_q]";
	cmd_params.cmd_arg	= (void *)cpu_slot;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cli_cls_lkp_dcod_entry_set;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "pp2_cls_lkp_dcod_luid_set";
	cmd_params.desc		= "sets the lookup ID information in the global lookup decode table";
	cmd_params.format	= "[lookup id number ... lookup id number]";
	cmd_params.cmd_arg	= (void *)cpu_slot;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cli_cls_lkp_dcod_luid_set;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "pp2_cls_lkp_dcod_add";
	cmd_params.desc		= "add the entry in the global lookup decode table";
	cmd_params.format	= "";
	cmd_params.cmd_arg	= (void *)cpu_slot;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cli_cls_lkp_dcod_add;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "pp2_cls_lkp_dcod_ena";
	cmd_params.desc		= "enables flow lookup ID in the lookup decode table";
	cmd_params.format	= "[flow_log_id flow_log_id ... flow_log_id]";
	cmd_params.cmd_arg	= (void *)cpu_slot;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cli_cls_lkp_dcod_ena;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "pp2_cls_lkp_dcod_dump";
	cmd_params.desc		= "dump all logical decode DB information";
	cmd_params.format	= "";
	cmd_params.cmd_arg	= (void *)cpu_slot;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cli_cls_lkp_dcod_dump;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "pp2_cls_fl_rule_init";
	cmd_params.desc		= "initializes the global flow rules";
	cmd_params.format	= "";
	cmd_params.cmd_arg	= (void *)cpu_slot;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cli_cls_fl_rule_init;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "pp2_cls_fl_rule_set";
	cmd_params.desc		= "sets a single flow rule in the global flow rules";
	cmd_params.format	= "[fl_log_id] [port_type] [port_bm]\n\t\t\t\t[enabled] [prio] [engine]\n"
				"\t\t\t\[field_id_cnt]\n\t\t\t\t[field_id0] [field_id1] [field_id2] [field_id3]";
	cmd_params.cmd_arg	= (void *)cpu_slot;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cli_cls_fl_rule_set;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "pp2_cls_fl_rule_add";
	cmd_params.desc		= "adds the current global flow rules";
	cmd_params.format	= "";
	cmd_params.cmd_arg	= (void *)cpu_slot;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cli_cls_fl_rule_add;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "pp2_cls_fl_rule_ena";
	cmd_params.desc		= "enables the current global flow rules";
	cmd_params.format	= "";
	cmd_params.cmd_arg	= (void *)cpu_slot;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cli_cls_fl_rule_ena;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "pp2_cls_fl_rule_dis";
	cmd_params.desc		= "disables a logical rule according to input array";
	cmd_params.format	= "[disable number disable number ... disable number]";
	cmd_params.cmd_arg	= (void *)cpu_slot;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cli_cls_fl_rule_dis;
	mvapp_register_cli_cmd(&cmd_params);
#endif
	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_fl_log_rls_dump";
	cmd_params.desc		= "dump all logical flow ID and rule offset";
	cmd_params.format	= "";
	cmd_params.cmd_arg	= (void *)cpu_slot;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cli_cls_fl_log_rls_dump;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_lkp_hits_dump";
	cmd_params.desc		= "dump all hit decode entry and its DB information";
	cmd_params.format	= "";
	cmd_params.cmd_arg	= (void *)cpu_slot;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cli_cls_lkp_hits_dump;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_fl_hits_dump";
	cmd_params.desc		= "dump all hit flow table entry";
	cmd_params.format	= "";
	cmd_params.cmd_arg	= (void *)cpu_slot;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cli_cls_fl_hits_dump;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_fl_rls_dump";
	cmd_params.desc		= "dump all logical flow ID rules";
	cmd_params.format	= "";
	cmd_params.cmd_arg	= (void *)cpu_slot;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cli_cls_fl_rls_dump;
	mvapp_register_cli_cmd(&cmd_params);

	return 0;
}

int register_cli_c3_cmds(uintptr_t cpu_slot)
{
	struct cli_cmd_params cmd_params;

#ifdef CLS_DEBUG
	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "rule_add";
	cmd_params.desc		= "add a C3 rule (fixed 5 tuples...no parameters for now";
	cmd_params.format	= "";
	cmd_params.cmd_arg	= (void *)cpu_slot;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_c3_rule_add;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "rule_remove";
	cmd_params.desc		= "remove a C3 rule according to logic_index";
	cmd_params.format	= "[logix_index] (dec) logical index of the rule to be removed";
	cmd_params.cmd_arg	= (void *)cpu_slot;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_c3_rule_delete;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "index_dump";
	cmd_params.desc		= " dump two index tables";
	cmd_params.format	= "[all] (dec)all index entry or only valid one, 0: valid, 1:all";
	cmd_params.cmd_arg	= (void *)cpu_slot;
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
	cmd_params.cmd_arg	= (void *)cpu_slot;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_c3_scan_param_set;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "scan_result_get";
	cmd_params.desc		= "dumps C3 entries scan result";
	cmd_params.format	= "[max_entry] (dec)max scan entry number";
	cmd_params.cmd_arg	= (void *)cpu_slot;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_c3_scan_result_get;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "hit_count_get";
	cmd_params.desc		= "get hit counter by logical index";
	cmd_params.format	= "[logic_idx] (dec)logical index, 0...4095";
	cmd_params.cmd_arg	= (void *)cpu_slot;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_c3_hit_count_get;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "search_depth_set";
	cmd_params.desc		= "set C3 cuckoo search depth (default 3)";
	cmd_params.format	= "[search_depth] (dec)cuckoo search depth, 1...8";
	cmd_params.cmd_arg	= (void *)cpu_slot;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_c3_search_depth_set;
	mvapp_register_cli_cmd(&cmd_params);

#endif
	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_c3_rule_hit_dump";
	cmd_params.desc		= "dump C3 entries according to type and index";
	cmd_params.format	= "--type --var (no arguments)\n"
				  "\t\t\t\t--type	(dec) C3 dump type, 0: logic idx, 1:hash idx, 2:lookup type\n"
				  "\t\t\t\t--var	(dec) value according to type, type 0/1:idx, type 2: lookup type\n"
				  "\t\t\t\tno arguments -> dumping all flows\n";
	cmd_params.cmd_arg	= (void *)cpu_slot;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_c3_type_entry_dump;
	mvapp_register_cli_cmd(&cmd_params);

	return 0;
}

int register_cli_c2_cmds(uintptr_t cpu_slot)
{
	struct cli_cmd_params cmd_params;

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_c2_lkp_type_entry_dump";
	cmd_params.desc		= "dump all entries with the lookup type";
	cmd_params.format	= "--type  (no arguments)\n"
				  "\t\t\t\t--type	(dec) lookup type number 0 - 64\n"
				  "\t\t\t\tno arguments -> dumping all flows\n";
	cmd_params.cmd_arg	= (void *)cpu_slot;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_c2_lkp_type_entry_dump;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_c2_free_tcam_dump";
	cmd_params.desc		= "dump all free C2 TCAM entry index";
	cmd_params.format	= "(no arguments)\n";
	cmd_params.cmd_arg	= (void *)cpu_slot;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_c2_free_tcam_dump;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_c2_valid_lkp_type_dump";
	cmd_params.desc		= "dump valid C2 lookup type, with its TCAM index";
	cmd_params.format	= "(no arguments)\n";
	cmd_params.cmd_arg	= (void *)cpu_slot;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_c2_valid_lkp_type_dump;
	mvapp_register_cli_cmd(&cmd_params);

#ifdef CLS_DEBUG
	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_c2_rule_hw_dump";
	cmd_params.desc		= "dump C2 entries";
	cmd_params.format	= "(no arguments)\n";
	cmd_params.cmd_arg	= (void *)cpu_slot;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_c2_hw_dump;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "cls_c2_rule_hw_hit_dump";
	cmd_params.desc		= "dump C2 hits according";
	cmd_params.format	= "(no arguments)\n";
	cmd_params.cmd_arg	= (void *)cpu_slot;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_c2_hw_hit_dump;
	mvapp_register_cli_cmd(&cmd_params);
#endif

	return 0;
}

