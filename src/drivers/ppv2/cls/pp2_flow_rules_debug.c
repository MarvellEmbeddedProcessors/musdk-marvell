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

#include "drivers/ppv2/pp2.h"
#include "drivers/ppv2/pp2_hw_type.h"
#include "drivers/ppv2/pp2_hw_cls.h"
#include "pp2_cls_types.h"
#include "pp2_cls_internal_types.h"
#include "pp2_cls_common.h"
#include "pp2_flow_rules.h"
#include "pp2_cls_db.h"
#include "pp2_cls_utils.h"
#include "pp2_cls_mng.h"

static struct pp2_cls_lkp_dcod_entry_t g_lkp_dcod_entry;
static struct pp2_cls_fl_rule_list_t g_fl_rls;

/*******************************************************************************
 * pp2_cli_cls_lkp_dcod_entry_set
 *
 * DESCRIPTION: The routine sets the lookup ID structure in the global lookup decode table
 *
 * INPUTS:
 *	arg - packet processor instance pointer
 *	argc - arguments count
 *	argv[] - arguments pointer
 *
 * RETURNS:
 *	On success, the function returns 0. On error different types are returned
 *	according to the case.
 ******************************************************************************/
int pp2_cli_cls_lkp_dcod_entry_set(void *arg, int argc, char *argv[])
{
	int rc;
	int flow_log_id;
	int cpu_q;
	int flow_len;
	int ret_val = 0;

	if (argc != 4) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	/* Get parameters */
	rc = kstrtoint(argv[1], 10, &flow_log_id);
	if (rc || flow_log_id < 0 || (flow_log_id >= MVPP2_MNG_FLOW_ID_MAX)) {
		pr_err("parsing fail, wrong input for argv[1] - flow_log_id");
		return -EINVAL;
	}
	rc = kstrtoint(argv[2], 10, &flow_len);
	if (rc || flow_len <= 0 || (flow_len >= MVPP2_CLS_FLOW_RULE_MAX)) {
		pr_err("parsing fail, wrong input for argv[2] - flow length");
		return -EINVAL;
	}
	rc = kstrtoint(argv[3], 10, &cpu_q);
	if (rc || cpu_q < 0 || (cpu_q >= ((1 << MVPP2_FLOWID_RXQ_BITS) - 1))) {
		pr_err("parsing fail, wrong input for argv[3] - cpu_q");
		return -EINVAL;
	}

	memset(&g_lkp_dcod_entry, 0, sizeof(g_lkp_dcod_entry));

	g_lkp_dcod_entry.cpu_q = cpu_q;
	g_lkp_dcod_entry.flow_len = flow_len;
	g_lkp_dcod_entry.flow_log_id = flow_log_id;
	g_lkp_dcod_entry.luid_num = 0;

	printk("flow_log_id = %d, flow_len_max = %d, cpu_q = %d\n", flow_log_id, flow_len, cpu_q);
	printk("success\n");

	return ret_val;
}

/*******************************************************************************
 * pp2_cli_cls_lkp_dcod_luid_set
 *
 * DESCRIPTION: The routine sets the lookup ID information in the global lookup decode table
 *
 * INPUTS:
 *	arg - packet processor instance pointer
 *	argc - arguments count
 *	argv[] - arguments pointer
 *
 * RETURNS:
 *	On success, the function returns 0. On error different types are returned
 *	according to the case.
 ******************************************************************************/
int pp2_cli_cls_lkp_dcod_luid_set(void *arg, int argc, char *argv[])
{
	int rc;
	int luid[MVPP2_CLS_LOG_FLOW_LUID_MAX], i, num_of_luid;
	int ret_val = 0;

	if (argc < 2 || argc > MVPP2_CLS_LOG_FLOW_LUID_MAX) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	num_of_luid = argc - 1;

	for (i = 0; i < num_of_luid; i++) {
		rc = kstrtoint(argv[1 + i], 10, &luid[i]);
		if (rc || luid[i] < 0 || luid[i] >= MVPP2_CLS_LKP_TBL_SIZE) {
			pr_err("parsing fail, wrong input for argv[2 + %d]", i);
			return -EINVAL;
		}
	}

	if ((num_of_luid + g_lkp_dcod_entry.luid_num) >= MVPP2_CLS_LOG_FLOW_LUID_MAX) {
		pr_err("too many luid per flow log id");
		return -EINVAL;
	}

	for (i = 0; i < num_of_luid; i++) {
		g_lkp_dcod_entry.luid_list[g_lkp_dcod_entry.luid_num].luid = luid[i];
		g_lkp_dcod_entry.luid_num++;
	}

	printk("success\n");

	return ret_val;
}

/*******************************************************************************
 * pp2_cli_cls_lkp_dcod_add
 *
 * DESCRIPTION: The routine add the entry in the global lookup decode table
 *
 * INPUTS:
 *	arg - packet processor instance pointer
 *	argc - arguments count
 *	argv[] - arguments pointer
 *
 * RETURNS:
 *	On success, the function returns 0. On error different types are returned
 *	according to the case.
 *******************************************************************************/
int pp2_cli_cls_lkp_dcod_add(void *arg, int argc, char *argv[])
{
	struct pp2_inst *inst = (struct pp2_inst *)arg;

	if (pp2_cls_lkp_dcod_set(inst, &g_lkp_dcod_entry))
		printk("%s fail\n", __func__);
	else
		printk("success\n");

	return 0;
}

/*******************************************************************************
 * pp2_cli_cls_lkp_dcod_ena
 *
 * DESCRIPTION: The routine enables a single lookup ID in the lookup decode table
 *
 * INPUTS:
 *	arg - packet processor instance pointer
 *	argc - arguments count
 *	argv[] - arguments pointer
 *
 * RETURNS:
 *	On success, the function returns 0. On error different types are returned
 *	according to the case.
 ******************************************************************************/
int pp2_cli_cls_lkp_dcod_ena(void *arg, int argc, char *argv[])
{
	int rc;
	struct pp2_inst *inst = (struct pp2_inst *)arg;
	int log_id_en[MVPP2_MNG_FLOW_ID_MAX], i, num_log_id;
	int ret_val = 0;

	if (argc < 2) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	num_log_id = argc - 1;

	/* Get parameters */
	for (i = 0; i < num_log_id; i++) {
		rc = kstrtoint(argv[1 + i], 10, &log_id_en[i]);
		if (rc || log_id_en[i] < 0 || log_id_en[i] >= MVPP2_MNG_FLOW_ID_MAX) {
			pr_err("parsing fail, wrong input for argv[2 + %d]", i);
			return -EINVAL;
		}
	}

	for (i = 0; i < num_log_id; i++) {
		if (pp2_cls_lkp_dcod_enable(inst, log_id_en[i]))
			printk("%s fail\n", __func__);
		else
			printk("success\n");
	}

	return ret_val;
}

/*******************************************************************************
 * pp2_cli_cls_fl_rule_init
 *
 * DESCRIPTION: The routine initializes the global flow rules to default
 *
 * INPUTS:
 *	arg - packet processor instance pointer
 *	argc - arguments count
 *	argv[] - arguments pointer
 *
 * RETURNS:
 *	On success, the function returns 0. On error different types are returned
 *	according to the case.
 ******************************************************************************/
int pp2_cli_cls_fl_rule_init(void *arg, int argc, char *argv[])
{
	memset(&g_fl_rls, 0, sizeof(g_fl_rls));
	printk("success\n");

	return 0;
}

/*******************************************************************************
 * pp2_cli_cls_fl_rule_set
 *
 * DESCRIPTION: The routine sets a single flow rule in the global flow rules
 *
 * INPUTS:
 *	arg - packet processor instance pointer
 *	argc - arguments count
 *	argv[] - arguments pointer
 *
 * RETURNS:
 *	On success, the function returns 0. On error different types are returned
 *	according to the case.
 ******************************************************************************/
int pp2_cli_cls_fl_rule_set(void *arg, int argc, char *argv[])
{
	int rc;
	int ret_val = 0;
	int flow_log_id, port_type, port_bm, enabled;
	int prio, engine, field_id_cnt;
	u8 field_id[MVPP2_FLOW_FIELD_COUNT_MAX];

	if (argc != 12) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	if (g_fl_rls.fl_len == (MVPP2_CLS_FLOW_RULE_MAX - 1)) {
		pr_err("flow is full of rules: %d\n", g_fl_rls.fl_len);
		return -EINVAL;
	}

	rc = kstrtoint(argv[1], 10, &flow_log_id);
	if (rc || flow_log_id < 0 || (flow_log_id >= MVPP2_MNG_FLOW_ID_MAX)) {
		pr_err("parsing fail, wrong input for argv[1] - flow_log_id");
		return -EINVAL;
	}

	rc = kstrtoint(argv[2], 10, &port_type);
	if (rc || port_type < 0 || port_type >= MVPP2_SRC_PORT_TYPE_MAX) {
		pr_err("parsing fail, wrong input for argv[2] - port_type");
		return -EINVAL;
	}
	rc = kstrtoint(argv[3], 10, &port_bm);
	if (rc ||  port_bm < 0 || port_bm > MVPP2_PORT_BM_INV) {
		pr_err("parsing fail, wrong input for argv[3] - port_bm");
		return -EINVAL;
	}

	rc = kstrtoint(argv[4], 10, &enabled);
	if (rc || enabled < 0 || enabled > 1) {
		pr_err("parsing fail, wrong input for argv[5] - enabled");
		return -EINVAL;
	}
	rc = kstrtoint(argv[5], 10, &prio);
	if (rc || prio < 0 || (prio >= ((1 << MVPP2_FLOW_FIELD_ID_BITS) - 1))) {
		pr_err("parsing fail, wrong input for argv[6] - priority");
		return -EINVAL;
	}
	rc = kstrtoint(argv[6], 10, &engine);
	if (rc || engine < 1 || engine > MVPP2_FLOW_ENGINE_MAX) {
		pr_err("parsing fail, wrong input for argv[7] - engine");
		return -EINVAL;
	}
	rc = kstrtoint(argv[7], 10, &field_id_cnt);
	if (rc || field_id_cnt < 0 || field_id_cnt > MVPP2_FLOW_FIELD_COUNT_MAX) {
		pr_err("parsing fail, wrong input for argv[8] - field_id_cnt");
		return -EINVAL;
	}
	rc = kstrtou8(argv[8], 10, &field_id[0]);
	if (rc || field_id[0] >= CLS_FIELD_MAX) {
		pr_err("parsing fail, wrong input for argv[9] - field_id[0]");
		return -EINVAL;
	}
	rc = kstrtou8(argv[9], 10, &field_id[1]);
	if (rc || field_id[1] >= CLS_FIELD_MAX) {
		pr_err("parsing fail, wrong input for argv[10] - field_id[1]");
		return -EINVAL;
	}
	rc = kstrtou8(argv[10], 10, &field_id[2]);
	if (rc || field_id[2] >= CLS_FIELD_MAX) {
		pr_err("parsing fail, wrong input for argv[11] - field_id[2]");
		return -EINVAL;
	}
	rc = kstrtou8(argv[11], 10, &field_id[3]);
	if (rc || field_id[3] >= CLS_FIELD_MAX) {
		pr_err("parsing fail, wrong input for argv[12] - field_id[3]");
		return -EINVAL;
	}

	g_fl_rls.fl[g_fl_rls.fl_len].fl_log_id	= flow_log_id;
	g_fl_rls.fl[g_fl_rls.fl_len].port_type	= port_type;
	g_fl_rls.fl[g_fl_rls.fl_len].port_bm	= port_bm;
	g_fl_rls.fl[g_fl_rls.fl_len].lu_type	= MVPP2_CLS_LKP_MUSDK_CLS;
	g_fl_rls.fl[g_fl_rls.fl_len].enabled	= enabled;
	g_fl_rls.fl[g_fl_rls.fl_len].prio		= prio;
	g_fl_rls.fl[g_fl_rls.fl_len].engine		= engine;
	g_fl_rls.fl[g_fl_rls.fl_len].field_id_cnt	= field_id_cnt;
	memcpy(g_fl_rls.fl[g_fl_rls.fl_len].field_id, field_id, sizeof(field_id));
	g_fl_rls.fl_len++;
	printk("success\n");

	return ret_val;
}

/*******************************************************************************
 * pp2_cli_cls_fl_rule_add
 *
 * DESCRIPTION: The routine adds the current global flow rules
 *
 * INPUTS:
 *	arg - packet processor instance pointer
 *	argc - arguments count
 *	argv[] - arguments pointer
 *
 * RETURNS:
 *	On success, the function returns 0. On error different types are returned
 *	according to the case.
 ******************************************************************************/
int pp2_cli_cls_fl_rule_add(void *arg, int argc, char *argv[])
{
	int rc;
	struct pp2_inst *inst = (struct pp2_inst *)arg;

	rc = pp2_cls_fl_rule_add(inst, &g_fl_rls);

	if (!rc)
		printk("success\n");
	else
		printk("%s fail\n", __func__);

	return 0;
}

/*******************************************************************************
 * pp2_cls_fl_rule_ena
 *
 * DESCRIPTION: The routine enables the current global flow rules
 *
 * INPUTS:
 *	arg - packet processor instance pointer
 *	argc - arguments count
 *	argv[] - arguments pointer
 *
 * RETURNS:
 *	On success, the function returns 0. On error different types are returned
 *	according to the case.
 ******************************************************************************/
int pp2_cli_cls_fl_rule_ena(void *arg, int argc, char *argv[])
{
	int rc, i;
	struct pp2_inst *inst = (struct pp2_inst *)arg;

	rc = pp2_cls_fl_rule_enable(inst, &g_fl_rls);

	if (!rc) {
		for (i = 0; i < g_fl_rls.fl_len; i++)
			printk("rule:%d assigned logical rule ID:%d\n", i, g_fl_rls.fl[i].rl_log_id);
	}

	if (!rc)
		printk("success\n");
	else
		printk("%s fail\n", __func__);

	return 0;
}

/*******************************************************************************
 * pp2_cli_cls_set_rss_mode
 *
 * DESCRIPTION: The routine set rss mode
 *
 * INPUTS:
 *	arg - packet port pointer
 *	argc - arguments count
 *	argv[] - arguments pointer
 *
 * RETURNS:
 *	On success, the function returns 0. On error different types are returned
 *	according to the case.
 ******************************************************************************/
int pp2_cli_cls_set_rss_mode(void *arg, int argc, char *argv[])
{
	int rc;
	u32 rss_mode;
	struct pp2_port *port = (struct pp2_port *)arg;

	if (argc != 2) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	rc = kstrtou32(argv[1], 10, &rss_mode);
	if (rc || (rss_mode >= PP2_PPIO_HASH_T_OUT_OF_RANGE)) {
		pr_err("parsing fail, wrong input for argv[1] - rss_mode");
		return -EINVAL;
	}

	rc = pp2_cls_rss_mode_flows_set(port, rss_mode);
	if (!rc)
		pr_err("success\n");
	else
		pr_err("%s fail\n", __func__);

	return 0;
}

/*******************************************************************************
 * pp2_cli_cls_fl_rule_dis
 *
 * DESCRIPTION: The routine disables a logical rule according to input array
 *
 * INPUTS:
 *	arg - packet processor instance pointer
 *	argc - arguments count
 *	argv[] - arguments pointer
 *
 * RETURNS:
 *	On success, the function returns 0. On error different types are returned
 *	according to the case.
 ******************************************************************************/
int pp2_cli_cls_fl_rule_dis(void *arg, int argc, char *argv[])
{
	int rc;
	int i;
	u16 rl_log_id[MVPP2_CLS_FLOW_RULE_MAX];
	u16 rl_log_id_len;
	int loop = 0;
	struct pp2_cls_class_port_t src_port;
	struct pp2_inst *inst = (struct pp2_inst *)arg;

	if (argc < 2 || argc > MVPP2_CLS_FLOW_RULE_MAX) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	rl_log_id_len = argc - 1;
	for (i = 0; i < rl_log_id_len; i++) {
		rc = kstrtou16(argv[2 + i], 10, &rl_log_id[i]);
		if (rc ||  rl_log_id[i] >= MVPP2_FLOW_TBL_SIZE) {
			pr_err("parsing fail, wrong input for argv[2 + %d]", i);
			return -EINVAL;
		}
	}

	src_port.port_type = (enum pp2_cls_class_port_type_t)g_fl_rls.fl[0].port_type;

	for (loop = 0; loop < MVPP2_MAX_NUM_GMACS; loop++) {
		if ((1 << loop) & g_fl_rls.fl[0].port_bm) {
			src_port.class_port = (1 << loop);
			if (pp2_cls_fl_rule_disable(inst, rl_log_id, rl_log_id_len, &src_port))
				printk("%s fail\n", __func__);
			else
				printk("success\n");
		}
	}

	return 0;
}

/*******************************************************************************
 * pp2_cli_cls_lkp_dcod_dump
 *
 * DESCRIPTION: The routine dump all logical decode DB information.
 *
 * INPUTS:
 *	arg - packet processor instance pointer
 *	argc - arguments count
 *	argv[] - arguments pointer
 *
 * RETURNS:
 *	On success, the function returns 0. On error different types are returned
 *	according to the case.
 *******************************************************************************/
int pp2_cli_cls_lkp_dcod_dump(void *arg, int argc, char *argv[])
{
	struct pp2_db_cls_lkp_dcod_t lkp_dcod_db;
	u32 i, j;
	int rc;
	struct pp2_inst *inst = (struct pp2_inst *)arg;

	printk("\nlog_id enabled  cpuq  alloc len off luid_no = luid_list (luid)\n");
	for (i = 0; i < MVPP2_MNG_FLOW_ID_MAX; i++) {
		rc = pp2_db_cls_lkp_dcod_get(inst, i, &lkp_dcod_db);
		if (rc != 0) {
			pr_err("pp2_db_cls_lkp_dcod_get returned error %d\n", rc);
			pr_err("%s fail\n", __func__);
			return -EINVAL;
		}
		printk("%6d %-7s  0x%02X %5d %3d %3d %7d = ",
		       i, pp2_utils_valid_str_get((int)lkp_dcod_db.enabled),
		       (int)lkp_dcod_db.cpu_q, (int)lkp_dcod_db.flow_alloc_len,
		       (int)lkp_dcod_db.flow_len, lkp_dcod_db.flow_off, lkp_dcod_db.luid_num);
		for (j = 0; j < lkp_dcod_db.luid_num; j++)
			printk("%2d", lkp_dcod_db.luid_list[j].luid);
		printk("\n");
	}

	return 0;
}

/*******************************************************************************
 * pp2_cli_cls_lkp_hits_dump
 *
 * DESCRIPTION: The routine dump all hit decode entry and its DB information.
 *
 * INPUTS:
 *	arg - packet processor instance pointer
 *	argc - arguments count
 *	argv[] - arguments pointer
 *
 * RETURNS:
 *	On success, the function returns 0. On error different types are returned
 *	according to the case.
 ******************************************************************************/
int pp2_cli_cls_lkp_hits_dump(void *arg, int argc, char *argv[])
{
	struct pp2_db_cls_lkp_dcod_t lkp_dcod_db;
	int rc = 0;
	u32 i, flow_log_id = 0, lkpid = 0, hit_cnt = 0;
	struct pp2_inst *inst = (struct pp2_inst *)arg;
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);

	printk("log_id  lkp_id hit_cnt\n");

	/* get the lkpid */
	for (lkpid = 0; lkpid < MVPP2_CLS_LKP_TBL_SIZE; lkpid++) {
		rc = mv_pp2x_cls_hw_lkp_hit_get(cpu_slot, lkpid, &hit_cnt);
		if (rc) {
			pr_err("lookup ID%d, hit cnt read fail\n", lkpid);
			return -EINVAL;
		}
		if (!hit_cnt)
			continue;

		/* get logical id */
		for (flow_log_id = 0; flow_log_id < MVPP2_MNG_FLOW_ID_MAX; flow_log_id++) {
			rc = pp2_db_cls_lkp_dcod_get(inst, flow_log_id, &lkp_dcod_db);
			if (rc) {
				pr_err("pp2_db_cls_lkp_dcod_get returned error %d\n", rc);
				return 0;
			}
			if (lkp_dcod_db.luid_num == 0)
				continue;
			for (i = 0; i < lkp_dcod_db.luid_num; i++) {
				if (lkp_dcod_db.luid_list[i].luid == lkpid) {
					printk("%3d      %3d    %3d\n",
					       flow_log_id, lkpid, hit_cnt);
					break;
				}
			}
			if (i < lkp_dcod_db.luid_num)
				break;
		}
		if (flow_log_id == MVPP2_MNG_FLOW_ID_MAX)
			pr_err("logical id not found for lkpid %d\n", lkpid);
		hit_cnt = 0;
	}

	return 0;
}

/*******************************************************************************
 * pp2_cli_cls_fl_hits_dump
 *
 * DESCRIPTION: The routine dump all hit flow table entry.
 *
 * INPUTS:
 *	arg - packet processor instance pointer
 *	argc - arguments count
 *	argv[] - arguments pointer
 *
 * RETURNS:
 *	On success, the function returns 0. On error different types are returned
 *	according to the case.
 ******************************************************************************/
int pp2_cli_cls_fl_hits_dump(void *arg, int argc, char *argv[])
{
	int rc;
	struct pp2_inst *inst = (struct pp2_inst *)arg;
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);

	rc = mv_pp2x_cls_hw_flow_hits_dump(cpu_slot);
	if (rc)
		pr_err("%s fail\n", __func__);

	return 0;
}

/*******************************************************************************
 * pp2_cli_cls_fl_rls_dump
 *
 * DESCRIPTION: The routine dump all logical flow ID rules.
 *
 * INPUTS:
 *	arg - packet processor instance pointer
 *	argc - arguments count
 *	argv[] - arguments pointer
 *
 * RETURNS:
 *	On success, the function returns 0. On error different types are returned
 *	according to the case.
 ******************************************************************************/
int pp2_cli_cls_fl_rls_dump(void *arg, int argc, char *argv[])
{
	struct pp2_db_cls_fl_rule_list_t *fl_rl_list_db;
	struct pp2_db_cls_lkp_dcod_t lkp_dcod_db;
	u32 i, j, k;
	u16 ref_sum = 0;
	int rc;
	struct pp2_inst *inst = (struct pp2_inst *)arg;

	fl_rl_list_db = kmalloc(sizeof(*fl_rl_list_db), GFP_KERNEL);
	if (!fl_rl_list_db) {
		return -ENOMEM;
	}
	memset(fl_rl_list_db, 0, sizeof(struct pp2_db_cls_fl_rule_list_t));

	printk("log_flow_id enabled offset engine logID lut portBm portType pri refCnt fid# = field_Ids\n");

	for (i = 0; i < MVPP2_MNG_FLOW_ID_MAX; i++) {
		rc = pp2_db_cls_lkp_dcod_get(inst, i, &lkp_dcod_db);
		if (rc) {
			printk("pp2_db_cls_lkp_dcod_get returned error %d\n", rc);
			kfree(fl_rl_list_db);
			return 0;
		}
		if (lkp_dcod_db.flow_len == 0)
			continue;

		printk("%-11d ", i);

		rc = pp2_db_cls_fl_rule_list_get(inst, lkp_dcod_db.flow_off,
						 lkp_dcod_db.flow_len, fl_rl_list_db->flow);
		if (rc) {
			pr_err("pp2_db_cls_fl_rule_list_get returned error %d\n", rc);
			kfree(fl_rl_list_db);
			return 0;
		}

		for (j = 0; j < lkp_dcod_db.flow_len; j++) {
			int proto_field = (fl_rl_list_db->flow[j].engine == MVPP2_CLS_ENGINE_C3B ||
					   fl_rl_list_db->flow[j].engine == MVPP2_CLS_ENGINE_C3HB);
			if (j != 0)
				printk("\t    ");

			printk("%-6s  %6d %-6s",
				pp2_utils_valid_str_get((int)fl_rl_list_db->flow[j].enabled),
				lkp_dcod_db.flow_off + j,
				pp2_utils_eng_no_str_get((int)fl_rl_list_db->flow[j].engine));
			ref_sum = 0;
			for (k = 0; k < MVPP2_MAX_NUM_GMACS; k++)
				ref_sum += fl_rl_list_db->flow[j].ref_cnt[k];

			printk(" %5d %3d %6d %4d %-8s %3d %6d",
				fl_rl_list_db->flow[j].rl_log_id,
				fl_rl_list_db->flow[j].lu_type,
				fl_rl_list_db->flow[j].port_bm,
				fl_rl_list_db->flow[j].udf7,
				pp2_cls_utils_port_type_str_get(fl_rl_list_db->flow[j].port_type),
				fl_rl_list_db->flow[j].prio,
				ref_sum);
			printk(" %4d = ", fl_rl_list_db->flow[j].field_id_cnt + proto_field);
			for (k = 0; k < fl_rl_list_db->flow[j].field_id_cnt; k++)
				printk("%s ", pp2_utils_field_id_str_get(fl_rl_list_db->flow[j].field_id[k]));
			if (proto_field)
				printk("%s ", pp2_utils_field_id_str_get(IPV4_PROTO_FIELD_ID));
			printk("\n");
		}
		if (lkp_dcod_db.flow_len)
			printk("\n");
	}

	kfree(fl_rl_list_db);
	return 0;
}

/*******************************************************************************
 * pp2_cli_cls_fl_log_rls_dump
 *
 * DESCRIPTION: The routine dump all logical flow ID and rule offset.
 *
 * INPUTS:
 *	arg - packet processor instance pointer
 *	argc - arguments count
 *	argv[] - arguments pointer
 *
 * RETURNS:
 *	On success, the function returns 0. On error different types are returned
 *	according to the case.
 ******************************************************************************/
int pp2_cli_cls_fl_log_rls_dump(void *arg, int argc, char *argv[])
{
	u32 i;
	int rc;
	u16 off;
	struct pp2_inst *inst = (struct pp2_inst *)arg;

	rc = pp2_db_cls_rl_off_get(inst, &off, 0);
	if (rc) {
		pr_err("%s returned error %d\n", __func__, rc);
		return 0;
	}

	printk("number of rules=%d\n", off - MVPP2_CLS_LOG2OFF_START);

	if ((off - MVPP2_CLS_LOG2OFF_START) > 0) {
		printk("log_rule_ID = flow_table_offset\n");

		for (i = 1; i < MVPP2_CLS_LOG2OFF_TBL_SIZE; i++) {
			rc = pp2_db_cls_rl_off_get(inst, &off, i);
			if (rc == -EINVAL) {
				return 0;
			} else if (rc) {
				pr_err("%s returned error %d\n", __func__, rc);
				return 0;
			}

			if (off != MVPP2_CLS_FREE_FL_LOG)
				printk("%11d = %17d\n", i, off);
		}
	}

	return 0;
}

int pp2_cls_print_rxq_counters(void *arg, int argc, char *argv[])
{
	struct pp2_port *port = (struct pp2_port *)arg;
	uintptr_t cpu_slot = port->cpu_slot;
	int i = 1, j;
	u32 tc_num;
	int rc;
	int phy_rxq;

	if  (argc < 2 || argc > (port->num_tcs + 1)) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	do {
		rc = kstrtou32(argv[1], 10, &tc_num);
		if (rc || (tc_num >= port->num_tcs)) {
			pr_err("parsing fail, wrong input for --tc\n");
			return -EINVAL;
		}

		for (j = 0; j < port->tc[tc_num].tc_config.num_in_qs; j++) {
			phy_rxq = port->tc[tc_num].tc_config.first_rxq + j;
			printk("\n------ [Port %s, TC %d, queue %d counters] -----\n",
				port->linux_name, tc_num, j);
			rc = mv_pp2x_cls_hw_rxq_counter_get(cpu_slot, phy_rxq);
			if (rc) {
				pr_err("%s(%d) read rxq counters failed\n", __func__, __LINE__);
				return -EINVAL;
			}
		}
	} while (++i < argc);

	return 0;
}
