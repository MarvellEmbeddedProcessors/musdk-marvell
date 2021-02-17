/*******************************************************************************
 * Copyright (C) Marvell International Ltd. and its affiliates
 *
 * This software file (the "File") is owned and distributed by Marvell
 * International Ltd. and/or its affiliates ("Marvell") under the following
 * alternative licensing terms.  Once you have made an election to distribute the
 * File under one of the following license alternatives, please (i) delete this
 * introductory statement regarding license alternatives, (ii) delete the three
 * license alternatives that you have not elected to use and (iii) preserve the
 * Marvell copyright notice above.
 *
 ********************************************************************************
 * Marvell Commercial License Option
 *
 * If you received this File from Marvell and you have entered into a commercial
 * license agreement (a "Commercial License") with Marvell, the File is licensed
 * to you under the terms of the applicable Commercial License.
 *
 ********************************************************************************
 * Marvell GPL License Option
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File in accordance with the terms and conditions of the General
 * Public License Version 2, June 1991 (the "GPL License"), a copy of which is
 * available along with the File in the license.txt file or by writing to the Free
 * Software Foundation, Inc., or on the worldwide web at http://www.gnu.org/licenses/gpl.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY
 * DISCLAIMED.  The GPL License provides additional details about this warranty
 * disclaimer.
 *
 ********************************************************************************
 * Marvell GNU General Public License FreeRTOS Exception
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File in accordance with the terms and conditions of the Lesser
 * General Public License Version 2.1 plus the following FreeRTOS exception.
 * An independent module is a module which is not derived from or based on
 * FreeRTOS.
 * Clause 1:
 * Linking FreeRTOS statically or dynamically with other modules is making a
 * combined work based on FreeRTOS. Thus, the terms and conditions of the GNU
 * General Public License cover the whole combination.
 * As a special exception, the copyright holder of FreeRTOS gives you permission
 * to link FreeRTOS with independent modules that communicate with FreeRTOS solely
 * through the FreeRTOS API interface, regardless of the license terms of these
 * independent modules, and to copy and distribute the resulting combined work
 * under terms of your choice, provided that:
 * 1. Every copy of the combined work is accompanied by a written statement that
 * details to the recipient the version of FreeRTOS used and an offer by yourself
 * to provide the FreeRTOS source code (including any modifications you may have
 * made) should the recipient request it.
 * 2. The combined work is not itself an RTOS, scheduler, kernel or related
 * product.
 * 3. The independent modules add significant and primary functionality to
 * FreeRTOS and do not merely extend the existing functionality already present in
 * FreeRTOS.
 * Clause 2:
 * FreeRTOS may not be used for any competitive or comparative purpose, including
 * the publication of any form of run time or compile time metric, without the
 * express permission of Real Time Engineers Ltd. (this is the norm within the
 * industry and is intended to ensure information accuracy).
 *
 ********************************************************************************
 * Marvell BSD License Option
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File under the following licensing terms.
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *	* Redistributions of source code must retain the above copyright notice,
 *	  this list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the distribution.
 *
 *	* Neither the name of Marvell nor the names of its contributors may be
 *	  used to endorse or promote products derived from this software without
 *	  specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************/

/***********************/
/* c file declarations */
/***********************/

#include "drivers/ppv2/pp2.h"
#include "drivers/ppv2/pp2_hw_type.h"
#include "pp2_hw_cls.h"
#include "pp2_cls_types.h"
#include "pp2_cls_internal_types.h"
#include "pp2_cls_common.h"
#include "pp2_flow_rules.h"
#include "pp2_cls_db.h"
#include "pp2_cls_mng.h"
#include "pp2_prs.h"

#define SAME_PRIO_ENABLED 0
#undef MVPP2_CLS_DEBUG

#ifdef MVPP2_CLS_DEBUG
void debug_dump_cls_fl(char *name, struct pp2_cls_fl_t *flow)
{
	int i;

	printk("dumping flow_log_id %d %s\n", flow->fl_log_id, name);

	for (i = 0; i < flow->fl_len; i++) {
		printk("en %d eng %d fid_cnt %d lut %2d port_bm %5d port_t %5d pri %2d udf7 %2d ref_cnt %2d skip %d\n",
			(int)flow->fl[i].enabled,
			(int)flow->fl[i].engine,
			(int)flow->fl[i].field_id_cnt,
			(int)flow->fl[i].lu_type,
			(int)flow->fl[i].port_bm,
			(int)flow->fl[i].port_type,
			(int)flow->fl[i].prio,
			(int)flow->fl[i].udf7,
			(int)flow->fl[i].ref_cnt,
			(int)flow->fl[i].skip);
	}
}

void debug_dump_lkp_dcod_db(char *name, struct pp2_db_cls_lkp_dcod_t *lkp_dcod_db)
{
	int i;

	printk("dumping pp2_db_cls_lkp_dcod_t %s\n", name);

	printk("CPUq %d enabled %d alloc_len %d flow_len %d flow_off %d luid_num %d\n",
		(int)lkp_dcod_db->cpu_q,
		(int)lkp_dcod_db->enabled,
		(int)lkp_dcod_db->flow_alloc_len,
		(int)lkp_dcod_db->flow_len,
		(int)lkp_dcod_db->flow_off,
		(int)lkp_dcod_db->luid_num);

	for (i = 0; i < lkp_dcod_db->luid_num; i++)
		pr_info("luid[%d]=%d/%d ",
			i, lkp_dcod_db->luid_list[i].luid);
	printk("\n");
}
#endif

#define RND_HIT_CNT(cnt, r) (cnt[r].c2 + cnt[r].c3 + cnt[r].c4)

/*******************************************************************************
 * pp2_cls_lkp_dcod_set
 *
 * DESCRIPTION: The API writes a single lookup decode entry to DB
 *
 * INPUTS:
 *	inst - packet processor instance
 *	lkp_dcod_conf - lookup decode structure information
 *
 * OUTPUTS:
 *	None
 *
 * RETURNS:
 *	0 on success, error-code otherwise
 ******************************************************************************/
int pp2_cls_lkp_dcod_set(struct pp2_inst *inst, struct pp2_cls_lkp_dcod_entry_t *lkp_dcod_conf)
{
	struct pp2_db_cls_lkp_dcod_t lkp_dcod_db;
	struct pp2_db_cls_fl_ctrl_t fl_ctrl;
	int rc;

	if (!lkp_dcod_conf) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}

	if (lkp_dcod_conf->flow_log_id >= MVPP2_MNG_FLOW_ID_MAX) {
		pr_err("Invalid input (flow_log_id = %d)\n", lkp_dcod_conf->flow_log_id);
		return -EINVAL;
	}

	if (lkp_dcod_conf->luid_num >= MVPP2_CLS_LOG_FLOW_LUID_MAX) {
		pr_err("Invalid input (luid_num = %d)\n", lkp_dcod_conf->luid_num);
		return -EINVAL;
	}

	if (lkp_dcod_conf->flow_len <= 0) {
		pr_err("Invalid input (flow_alloc_len = %d)\n", lkp_dcod_conf->flow_len);
		return -EINVAL;
	}

	/* get the DB entry for log flow ID */
	rc = pp2_db_cls_lkp_dcod_get(inst, lkp_dcod_conf->flow_log_id, &lkp_dcod_db);
	if (rc) {
		pr_err("failed to get lookup decode info for fl_log_id %d\n", lkp_dcod_conf->flow_log_id);
		return rc;
	}

	if (lkp_dcod_db.enabled) {
		pr_err("log flowid %d already configured\n", lkp_dcod_conf->flow_log_id);
		return -EFAULT;
	}

	if (lkp_dcod_db.flow_alloc_len) {
		pr_err("lkp for flow_log_id = %d already set\n", lkp_dcod_conf->flow_log_id);
		return 0;
	}

	rc = pp2_db_cls_fl_ctrl_get(inst, &fl_ctrl);
	if (rc) {
		pr_err("recvd ret_code(%d)\n", rc);
		return rc;
	}

	/* check that we have enough free space for flow */
	if (fl_ctrl.f_end - fl_ctrl.f_start <= lkp_dcod_conf->flow_len) {
		pr_err("not enough space for flow, request: %d, free:%d\n",
		       lkp_dcod_conf->flow_len, fl_ctrl.f_end - fl_ctrl.f_start);
		return -EPERM;
	}

	lkp_dcod_db.flow_off	= fl_ctrl.f_start;
	lkp_dcod_db.flow_alloc_len = lkp_dcod_conf->flow_len;
	lkp_dcod_db.flow_len	= 0;
	lkp_dcod_db.luid_num	= lkp_dcod_conf->luid_num;
	memcpy(lkp_dcod_db.luid_list, lkp_dcod_conf->luid_list,
	       sizeof(struct pp2_cls_luid_conf_t) * lkp_dcod_conf->luid_num);
	lkp_dcod_db.way	= 0; /* currently, always setting way to '0' */
	lkp_dcod_db.cpu_q	= lkp_dcod_conf->cpu_q;
	lkp_dcod_db.enabled	= false;

	rc = pp2_db_cls_lkp_dcod_set(inst, lkp_dcod_conf->flow_log_id, &lkp_dcod_db);
	if (rc) {
		pr_err("recvd ret_code(%d)\n", rc);
		return rc;
	}

	/* update the max flow length */
	if (lkp_dcod_conf->flow_len > fl_ctrl.fl_max_len)
		fl_ctrl.fl_max_len = lkp_dcod_conf->flow_len;

	/* update the control start index */
	fl_ctrl.f_start += lkp_dcod_conf->flow_len;

	rc = pp2_db_cls_fl_ctrl_set(inst, &fl_ctrl);
	if (rc) {
		pr_err("recvd ret_code(%d)\n", rc);
		return rc;
	}

	return 0;
}

/*******************************************************************************
 * pp2_cls_lkp_dcod_hw_set
 *
 * DESCRIPTION: The routine write to HW the lookup decode entry
 *
 * INPUTS:
 *	inst - packet processor instance
 *	fl - flow rules structure
 *
 * OUTPUTS:
 *	None
 *
 * RETURNS:
 *	0 on success, error-code otherwise
 *******************************************************************************/
static int pp2_cls_lkp_dcod_hw_set(struct pp2_inst *inst, struct pp2_cls_fl_t *fl)
{
	struct mv_pp2x_cls_lookup_entry fe;
	int rc;
	u16 luid, rl = 0;
	struct pp2_db_cls_lkp_dcod_t lkp_dcod_db;
	struct pp2_db_cls_fl_rule_list_t *fl_rl_db;
	u16 rl_off;
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);

	if (!fl) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}

	/* get the lookup DB for this logical flow ID */
	rc = pp2_db_cls_lkp_dcod_get(inst, fl->fl_log_id, &lkp_dcod_db);
	if (rc) {
		pr_err("failed to get lookup decode info for fl_log_id %d\n", fl->fl_log_id);
		return rc;
	}

	if (!lkp_dcod_db.enabled) {
		/* entry not enabled for this log_flow id */
		return 0;
	}

	/* get the rule list for this logical flow ID */
	fl_rl_db = kmalloc(sizeof(*fl_rl_db), GFP_KERNEL);
	if (!fl_rl_db)
		return -ENOMEM;

	memset(fl_rl_db, 0, sizeof(struct pp2_db_cls_fl_rule_list_t));
	rc = pp2_db_cls_fl_rule_list_get(inst, lkp_dcod_db.flow_off, lkp_dcod_db.flow_len, &fl_rl_db->flow[0]);
	if (rc) {
		pr_err("failed to get flow rule list fl_log_id=%d flow_off=%d flow_len=%d\n",
		       fl->fl_log_id, lkp_dcod_db.flow_off, lkp_dcod_db.flow_len);
		kfree(fl_rl_db);
		return rc;
	}

	/* iterate over all LUIDs */
	for (luid = 0; luid < lkp_dcod_db.luid_num; luid++) {
		/* Exclude MAC default LookupID by LSP */
		if (LUID_IS_LSP_RESERVED(lkp_dcod_db.luid_list[luid].luid))
			continue;

		/* found the rule, get it`s offset */
		rc = pp2_db_cls_rl_off_get(inst, &rl_off, fl_rl_db->flow[rl].rl_log_id);
		if (rc) {
			pr_err("recvd ret_code(%d)\n", rc);
			kfree(fl_rl_db);
			return rc;
		}

		/* updated the HW */
		mv_pp2x_cls_sw_lkp_clear(&fe);

		rc = mv_pp2x_cls_sw_lkp_flow_set(&fe, rl_off);
		if (rc) {
			pr_err("recvd ret_code(%d)\n", rc);
			kfree(fl_rl_db);
			return rc;
		}

		rc = mv_pp2x_cls_sw_lkp_rxq_set(&fe, lkp_dcod_db.cpu_q);
		if (rc) {
			pr_err("recvd ret_code(%d)\n", rc);
			kfree(fl_rl_db);
			return rc;
		}

		rc = mv_pp2x_cls_sw_lkp_en_set(&fe, lkp_dcod_db.enabled);
		if (rc) {
			pr_err("recvd ret_code(%d)\n", rc);
			kfree(fl_rl_db);
			return rc;
		}

		fe.way = lkp_dcod_db.way;
		fe.lkpid = lkp_dcod_db.luid_list[luid].luid;
		rc = mv_pp2x_cls_hw_lkp_write(cpu_slot, &fe);
		if (rc) {
			pr_err("recvd ret_code(%d)\n", rc);
			kfree(fl_rl_db);
			return rc;
		}

		pr_debug("fl_log_id[%2d] lid_nr[%2d] rl_log_id[%3d] prio[%2d] rl_off[%3d] luid[%2d]\n",
			 fl->fl_log_id, luid, fl_rl_db->flow[rl].rl_log_id,
			 fl_rl_db->flow[rl].prio,
			 rl_off,
			 lkp_dcod_db.luid_list[luid].luid);
	}

	kfree(fl_rl_db);
	return 0;
}

/*******************************************************************************
 * pp2_cls_lkp_dcod_set_and_disable
 *
 * DESCRIPTION: The API sets the decoder entry if not configured till now,
 *		and disables it if it was enabled
 *
 * INPUTS:
 *	inst - packet processor instance
 *	fl_log_id - the logical flow ID to perform the operation
 *
 * OUTPUTS:
 *	None
 *
 * RETURNS:
 *	0 on success, error-code otherwise
 ******************************************************************************/
int pp2_cls_lkp_dcod_set_and_disable(struct pp2_inst *inst,  u16 fl_log_id)
{
	struct pp2_cls_lkp_dcod_entry_t dcod;
	struct pp2_db_cls_lkp_dcod_t lkp_dcod_db;
	int rc;

	rc = pp2_db_cls_lkp_dcod_get(inst, fl_log_id, &lkp_dcod_db);
	if (rc) {
		pr_err("failed to get lookup decode info for fl_log_id %d\n", fl_log_id);
		return rc;
	}
	if (lkp_dcod_db.enabled) {
		pp2_cls_lkp_dcod_disable(inst, fl_log_id);
	/* if it isn't enabled it means that we should init decoder setting */
	} else {
		/* way - always 0*/
		dcod.way = MVPP2_CLS_DEF_WAY;

		/* currently every lkp id have specific log id */
		dcod.luid_num = 1;

		/* Flow log ID - Set to be the same as luid_list[0] TODO Ehud*/
		dcod.flow_log_id = fl_log_id;
		dcod.luid_list[0].luid = fl_log_id;

		dcod.flow_len = MVPP2_CLS_DEF_FLOW_LEN;

		/* Default queue - TODO not implemented yet in API */
		dcod.cpu_q = MVPP2_CLS_DEF_RXQ;

		/* Configure decoder table*/
		rc = pp2_cls_lkp_dcod_set(inst, &dcod);
		if (rc) {
			pr_err("failed to add in decoder table\n");
			return rc;
		}
	}

	return 0;
}

/*******************************************************************************
 * pp2_cls_lkp_dcod_disable
 *
 * DESCRIPTION: The API enables a lookup decode entry for a logical flow ID
 *
 * INPUTS:
 *	inst - packet processor instance
 *	fl_log_id - the logical flow ID to perform the operation
 *
 * OUTPUTS:
 *	None
 *
 * RETURNS:
 *	0 on success, error-code otherwise
 ******************************************************************************/
int pp2_cls_lkp_dcod_disable(struct pp2_inst *inst, u16 fl_log_id)
{
	struct mv_pp2x_cls_lookup_entry le;
	int rc;
	u16 luid;
	int way = 0; /* currently, always setting way to '0' */
	struct pp2_db_cls_lkp_dcod_t lkp_dcod_db;
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);

	/* get the lookup DB for this logical flow ID */
	rc = pp2_db_cls_lkp_dcod_get(inst, fl_log_id, &lkp_dcod_db);
	if (rc) {
		pr_err("failed to get lookup decode info for fl_log_id %d\n", fl_log_id);
		return rc;
	}

	if (!lkp_dcod_db.enabled) {
		/* entry disabled for this log_flow id */
		pr_warn("skipping disable of fl_log_id=%d, already disabled\n", fl_log_id);
		return 0;
	}

	/* iterate over all LUIDs */
	for (luid = 0; luid < lkp_dcod_db.luid_num; luid++) {
		/* Exclude MAC default LookupID by LSP */
		if (LUID_IS_LSP_RESERVED(lkp_dcod_db.luid_list[luid].luid))
			continue;

		/* updated the HW */
		mv_pp2x_cls_sw_lkp_clear(&le);

		rc = mv_pp2x_cls_hw_lkp_read(cpu_slot, luid, way, &le);
		if (rc)
			return rc;

		rc = mv_pp2x_cls_sw_lkp_en_set(&le, 0);
		if (rc) {
			pr_err("recvd ret_code(%d)\n", rc);
			return rc;
		}

		le.way = lkp_dcod_db.way;
		le.lkpid = lkp_dcod_db.luid_list[luid].luid;
		rc = mv_pp2x_cls_hw_lkp_write(cpu_slot, &le);
		if (rc) {
			pr_err("recvd ret_code(%d)\n", rc);
			return rc;
		}

		pr_debug("fl_log_id[%2d] luid_nr[%2d] luid[%2d]\n",
			 fl_log_id, luid,
			 lkp_dcod_db.luid_list[luid].luid);
	}

	/* update lkp_dcod DB */
	lkp_dcod_db.enabled = false;
	rc = pp2_db_cls_lkp_dcod_set(inst, fl_log_id, &lkp_dcod_db);
	if (rc) {
		pr_err("recvd ret_code(%d)\n", rc);
		return rc;
	}

	return 0;
}

/*******************************************************************************
 * pp2_cls_lkp_dcod_enable
 *
 * DESCRIPTION: The API enables a lookup decode entry for a logical flow ID
 *
 * INPUTS:
 *	inst - packet processor instance
 *	fl_log_id - the logical flow ID to perform the operation
 *
 * OUTPUTS:
 *	None
 *
 * RETURNS:
 *	0 on success, error-code otherwise
 ******************************************************************************/
int pp2_cls_lkp_dcod_enable(struct pp2_inst *inst, u16 fl_log_id)
{
	struct mv_pp2x_cls_lookup_entry fe;
	int rc;
	u16 luid, rl = 0;
	struct pp2_db_cls_lkp_dcod_t lkp_dcod_db;
	struct pp2_db_cls_fl_rule_list_t *fl_rl_db;
	u16 rl_off;
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);

	/* get the lookup DB for this logical flow ID */
	rc = pp2_db_cls_lkp_dcod_get(inst, fl_log_id, &lkp_dcod_db);
	if (rc) {
		pr_err("failed to get lookup decode info for fl_log_id %d\n", fl_log_id);
		return rc;
	}

	if (lkp_dcod_db.enabled) {
		/* entry enabled for this log_flow id */
		pr_warn("skipping enable of fl_log_id=%d, already enabled\n", fl_log_id);
		return 0;
	}

	if (!lkp_dcod_db.flow_len) {
		/* there are no flow rules */
		pr_warn("skipping enable of fl_log_id=%d, no rules in flow\n", fl_log_id);
		return 0;
	}

	/* get the rule list for this logical flow ID */
	fl_rl_db = kmalloc(sizeof(*fl_rl_db), GFP_KERNEL);
	if (!fl_rl_db)
		return -ENOMEM;

	memset(fl_rl_db, 0, sizeof(struct pp2_db_cls_fl_rule_list_t));
	rc = pp2_db_cls_fl_rule_list_get(inst, lkp_dcod_db.flow_off, lkp_dcod_db.flow_len, &fl_rl_db->flow[0]);
	if (rc) {
		pr_err("failed to get flow rule list fl_log_id=%d flow_off=%d flow_len=%d\n",
		       fl_log_id, lkp_dcod_db.flow_off, lkp_dcod_db.flow_len);
		kfree(fl_rl_db);
		return rc;
	}

	fl_rl_db->flow_len = lkp_dcod_db.flow_len;

	/* iterate over all LUIDs */
	for (luid = 0; luid < lkp_dcod_db.luid_num; luid++) {
		/* Exclude MAC default LookupID by LSP */
		if (LUID_IS_LSP_RESERVED(lkp_dcod_db.luid_list[luid].luid))
			continue;

		/* found the rule, get it`s offset */
		rc = pp2_db_cls_rl_off_get(inst, &rl_off, fl_rl_db->flow[rl].rl_log_id);
		if (rc) {
			pr_err("recvd ret_code(%d)\n", rc);
			kfree(fl_rl_db);
			return rc;
		}

		/* updated the HW */
		mv_pp2x_cls_sw_lkp_clear(&fe);

		rc = mv_pp2x_cls_sw_lkp_flow_set(&fe, rl_off);

		if (rc) {
			pr_err("recvd ret_code(%d)\n", rc);
			kfree(fl_rl_db);
			return rc;
		}

		rc = mv_pp2x_cls_sw_lkp_rxq_set(&fe, lkp_dcod_db.cpu_q);
		if (rc) {
			pr_err("recvd ret_code(%d)\n", rc);
			kfree(fl_rl_db);
			return rc;
		}

		rc = mv_pp2x_cls_sw_lkp_en_set(&fe, 1);
		if (rc) {
			pr_err("recvd ret_code(%d)\n", rc);
			kfree(fl_rl_db);
			return rc;
		}

		fe.way = lkp_dcod_db.way;
		fe.lkpid = lkp_dcod_db.luid_list[luid].luid;
		rc = mv_pp2x_cls_hw_lkp_write(cpu_slot, &fe);
		if (rc) {
			pr_err("recvd ret_code(%d)\n", rc);
			kfree(fl_rl_db);
			return rc;
		}

		pr_debug("fl_log_id[%2d] luid_nr[%2d] rl_log_id[%3d] prio[%2d] rl_off[%3d] luid[%2d]\n",
			 fl_log_id, luid, fl_rl_db->flow[rl].rl_log_id,
			 fl_rl_db->flow[rl].prio,
			 rl_off,
			 lkp_dcod_db.luid_list[luid].luid);
	}

	/* update lkp_dcod DB */
	lkp_dcod_db.enabled = true;
	rc = pp2_db_cls_lkp_dcod_set(inst, fl_log_id, &lkp_dcod_db);
	if (rc) {
		pr_err("recvd ret_code(%d)\n", rc);
		kfree(fl_rl_db);
		return rc;
	}

	kfree(fl_rl_db);
	return 0;
}

static int pp2_cls_lkp_dcod_enable_all(struct pp2_inst *inst)
{
	struct pp2_db_cls_lkp_dcod_t lkp_dcod_db;
	int fl_log_id;
	int rc;

	for (fl_log_id = 0; fl_log_id < MVPP2_MNG_FLOW_ID_MAX; fl_log_id++) {
		/* get the lookup DB for this logical flow ID */
		rc = pp2_db_cls_lkp_dcod_get(inst, fl_log_id, &lkp_dcod_db);
		if (rc)
			pr_err("failed to get lookup decode info for fl_log_id %d\n", fl_log_id);

		if ((!lkp_dcod_db.enabled) && (lkp_dcod_db.flow_alloc_len > 0)) {
			rc = pp2_cls_lkp_dcod_enable(inst, fl_log_id);
			if (rc)
				pr_err("fail fl_log_id %d\n", fl_log_id);
		}
	}

	return 0;
}

/*******************************************************************************
 * cmp_prio
 *
 * DESCRIPTION: The routine compares two rules according to the rule priority
 *
 * INPUTS:
 *	rl1 - the first rule
 *	rl2 - the second rule
 *
 * OUTPUTS:
 *	None
 *
 * RETURNS:
 *	if rl1<rl2 returns -1, if rl1>rl2 returns 1, else returns 0
 ******************************************************************************/
static int cmp_prio(const void *rl1, const void *rl2)
{
	if (((const struct pp2_cls_rl_entry_t *)rl1)->prio < ((const struct pp2_cls_rl_entry_t *)rl2)->prio)
		return -1;
	else if (((const struct pp2_cls_rl_entry_t *)rl1)->prio > ((const struct pp2_cls_rl_entry_t *)rl2)->prio)
		return 1;
	else
		return 0;
}

/*******************************************************************************
 * pp2_cls_fl_rls_sort
 *
 * DESCRIPTION: The routine sorts the classifier flow array according to the entry's priority
 *
 * INPUTS:
 *	fls - flows array
 *	fl_len - entries number
 *
 * OUTPUTS:
 *	None
 *
 * RETURNS:
 *	None
 ******************************************************************************/
static void pp2_cls_fl_rls_sort(struct pp2_cls_rl_entry_t fls[], u16 fl_len)
{
	struct pp2_cls_rl_entry_t temp;
	int i, j;

	for (i = 0; i < fl_len; i++) {
		for (j  = 0; j < fl_len - i - 1; j++) {
			if (cmp_prio(&fls[j], &fls[j + 1]) == 1) {
				temp = fls[j];
				fls[j] = fls[j + 1];
				fls[j + 1] = temp;
			}
		}
	}
}

/*******************************************************************************
 * pp2_cls_fl_rl_eng_cnt_upd
 *
 * DESCRIPTION: The routine updates the engine count according to input params
 *
 * INPUTS:
 *	op - the operation to perform (increment/decrement)
 *	eng - the engine to update
 *
 * OUTPUTS:
 *	eng_cnt - the engine structure which is updated
 *
 * RETURNS:
 *	0 on success, error-code otherwise
 ******************************************************************************/
static int pp2_cls_fl_rl_eng_cnt_upd(enum pp2_cls_rl_cnt_op_t op,
				     u16 eng,
				     struct pp2_cls_fl_eng_cnt_t *eng_cnt)
{
	if (!eng_cnt) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}

	switch (eng) {
	case MVPP2_ENGINE_C2:
		(op == MVPP2_CNT_DEC) ? eng_cnt->c2-- : eng_cnt->c2++;
		break;

	case MVPP2_ENGINE_C3_A:
	case MVPP2_ENGINE_C3_B:
	case MVPP2_ENGINE_C3_HA:
	case MVPP2_ENGINE_C3_HB:
		(op == MVPP2_CNT_DEC) ? eng_cnt->c3-- : eng_cnt->c3++;
		break;

	case MVPP2_ENGINE_C4:
		(op == MVPP2_CNT_DEC) ? eng_cnt->c4-- : eng_cnt->c4++;
		break;

	default:
		pr_err("Invalid input [engine=%d op=%d]\n", eng, op);
		return -EINVAL;
	}
	return 0;
}

/*******************************************************************************
 * pp2_cls_new_fl_rl_merge
 *
 * DESCRIPTION: The routine merges a single rule into a merged flow.
 *
 * INPUTS:
 *	new_rl_num - the rule number in the new flow
 *	new_fl_rls - the new flow that the new rule comes from
 *
 * OUTPUTS:
 *	mrg_fl_rls - the merged flow
 *
 * RETURNS:
 *	0 on success, error-code otherwise
 *******************************************************************************/
static int pp2_cls_new_fl_rl_merge(u16 new_rl_num,
				   struct pp2_cls_fl_t *new_fl_rls,
				   struct pp2_cls_fl_t *mrg_fl_rls)
{
	int rc;

	if (!new_fl_rls) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}
	if (!mrg_fl_rls) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}

	/* copy rule to merged flow */
	memcpy(&mrg_fl_rls->fl[mrg_fl_rls->fl_len],
	       &new_fl_rls->fl[new_rl_num],
	       sizeof(struct pp2_cls_rl_entry_t));

	/* set the skip flag in the source */
	new_fl_rls->fl[new_rl_num].skip = 1;

	/* update merged flow length */
	mrg_fl_rls->fl_len++;

	/* update merge engine count */
	rc = pp2_cls_fl_rl_eng_cnt_upd(MVPP2_CNT_INC,
				       new_fl_rls->fl[new_rl_num].engine,
				       &mrg_fl_rls->eng_cnt);
	if (rc) {
		pr_err("recvd ret_code(%d)\n", rc);
		return rc;
	}

	/* update new engine count */
	rc = pp2_cls_fl_rl_eng_cnt_upd(MVPP2_CNT_DEC,
				       new_fl_rls->fl[new_rl_num].engine,
				       &new_fl_rls->eng_cnt);
	if (rc) {
		pr_err("recvd ret_code(%d)\n", rc);
		return rc;
	}

	return 0;
}

/*******************************************************************************
 * pp2_cls_rl_hit_cnt_upd
 *
 * DESCRIPTION: The routine updates the engine hit counter for a single rule.
 *		It scans the flow for an identical rule, if does not find
 *		it increments the engine counter
 *
 * INPUTS:
 *	rl - rule to insert information
 *	fl_rls - flow rules to check the new rule
 *
 * OUTPUTS:
 *	eng_hit_cnt - the hit counter per engine structure which is updated
 *
 * RETURNS:
 *	0 on success, error-code otherwise
 ******************************************************************************/
static int pp2_cls_rl_hit_cnt_upd(struct pp2_cls_rl_entry_t *rl,
				  struct pp2_cls_fl_t *fl_rls,
				  struct pp2_cls_fl_eng_cnt_t *eng_hit_cnt)
{
	u16 rl_i;
	int rc;

	if (!rl) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}
	if (!fl_rls) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}
	if (!eng_hit_cnt) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}

	/* scan the flow for an identical rule */
	for (rl_i = 0; rl_i < fl_rls->fl_len; rl_i++) {
		if ((fl_rls->fl[rl_i].engine		== rl->engine)		&&
		    (fl_rls->fl[rl_i].lu_type		== rl->lu_type)		&&
		    (fl_rls->fl[rl_i].prio		== rl->prio)		&&
		    (fl_rls->fl[rl_i].udf7		== rl->udf7)		&&
		    (fl_rls->fl[rl_i].field_id_cnt	== rl->field_id_cnt)	&&
		    (!memcmp(fl_rls->fl[rl_i].field_id,
				rl->field_id,
				sizeof(rl->field_id)))) {
			return 0;
		}
	}

	/*
	 * did not find rule with same engine and priority
	 * increment hit counter
	 */
	rc = pp2_cls_fl_rl_eng_cnt_upd(MVPP2_CNT_INC,
				       rl->engine,
				       eng_hit_cnt);
	if (rc) {
		pr_err("recvd ret_code(%d)\n", rc);
		return rc;
	}

	return 0;
}

/*******************************************************************************
 * pp2_cls_rl_hit_cnt_upd
 *
 * DESCRIPTION: The routine updates the engine hit counter for a single rule in flow's cls nt rule reorder.
 *		It scans the flow for an identical rule, if does not find
 *		it increments the engine counter
 *
 * INPUTS:
 *	fl_rls - flow rules to check the new rule
 *	fl_idx - the rule index in flow which rule is used to check hit cnt updates
 *
 * OUTPUTS:
 *	eng_hit_cnt - the hit counter per engine structure which is updated
 *
 * RETURNS:
 *	On success, the function returns 0. On error different types are returned
 *	according to the case.
 ******************************************************************************/
static int pp2_cls_rl_hit_cnt_upd_reorder(struct pp2_cls_fl_t *fl_rls,
					  u32 fl_idx,
					  struct pp2_cls_fl_eng_cnt_t *eng_hit_cnt)
{
	u16 rl_i;
	int rc;
	struct pp2_cls_rl_entry_t *rl;

	if (!fl_rls) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}
	if (!eng_hit_cnt) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}
	if (((fl_idx) > (fl_rls->fl_len - 1)) || ((fl_idx) < 0)) {
		pr_err("(error) %s(%d) value (%d/0x%x) is out of range[%d, %d]\n",
		       __func__, __LINE__, (fl_idx), (fl_idx), 0, (fl_rls->fl_len - 1));
		return -EINVAL;
	}

	rl = &fl_rls->fl[fl_idx];

	/* scan the flow for an identical rule */
	for (rl_i = 0; rl_i < fl_idx; rl_i++) {
		if ((fl_rls->fl[rl_i].engine		== rl->engine)		&&
		    (fl_rls->fl[rl_i].lu_type		== rl->lu_type)		&&
		    (fl_rls->fl[rl_i].prio		== rl->prio)		&&
		    (fl_rls->fl[rl_i].udf7		== rl->udf7)		&&
		    (fl_rls->fl[rl_i].field_id_cnt	== rl->field_id_cnt)	&&
		    (fl_rls->fl[rl_i].port_type		== rl->port_type)	&&
		    (!memcmp(fl_rls->fl[rl_i].field_id,
				rl->field_id,
				sizeof(rl->field_id))))
			return 0;
	}

	/*
	 * did not find rule with same engine and priority
	 * increment hit counter
	 */
	rc = pp2_cls_fl_rl_eng_cnt_upd(MVPP2_CNT_INC,
				       rl->engine,
				       eng_hit_cnt);
	if (rc) {
		pr_err("recvd ret_code(%d)\n", rc);
		return rc;
	}

	return 0;
}

/*******************************************************************************
 * pp2_cls_rl_c4_validate
 *
 * DESCRIPTION: The routine validates C4 rule merge, searches for same priority rule
 *
 * INPUTS:
 *	rl - rule to insert information
 *	fl_rls - flow rules to check the new rule
 *
 * OUTPUTS:
 *	valid - returned validation indication
 *
 * RETURNS:
 *	0 on success, error-code otherwise
 ******************************************************************************/
static int pp2_cls_rl_c4_validate(struct pp2_cls_rl_entry_t *rl,
				  struct pp2_cls_fl_t *fl_rls,
				  bool *valid)
{
	u16 rl_i;

	if (!rl) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}
	if (!fl_rls) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}

	/* scan the flow for an identical rule */
	for (rl_i = 0; rl_i < fl_rls->fl_len; rl_i++) {
		if ((fl_rls->fl[rl_i].engine	== MVPP2_ENGINE_C4) &&
		    (fl_rls->fl[rl_i].prio	!= rl->prio)) {
			pr_err("add diff prio rule for C4 prohibited merged prio=%d new prio=%d fl_log_id=%d\n",
			       rl->prio, fl_rls->fl[rl_i].prio, fl_rls->fl_log_id);
			*valid = false;
			return 0;
		}
	}

	*valid = true;

	return 0;
}

/*******************************************************************************
 * pp2_cls_fl_rls_merge
 *
 * DESCRIPTION: The routine merges two flows (cur and new) of the same logical flow ID
 *		into a merged flow, the routine gets the current flow from DB and
 *		sorts the new flow ascending according to rule priority.
 *		Then it eliminates from the new flow rules that exist in the current flow.
 *		It then merges the two flows into one according to HW limitations
 *		taking into account allocation and engine limitation.
 *
 * INPUTS:
 *	inst - packet processor instance
 *	fl_log_id - merging flow logical rule ID
 *	cur_fl_rls - current flow rules
 *	new_fl_rls - new added flow rules
 *
 * OUTPUTS:
 *	mrg_fl_rls - merged flow rules
 *
 * RETURNS:
 *	0 on success, error-code otherwise
 ******************************************************************************/
static int pp2_cls_fl_rls_merge(struct pp2_inst *inst,
				u16 fl_log_id,
				struct pp2_cls_fl_t *cur_fl_rls,
				struct pp2_cls_fl_t *new_fl_rls,
				struct pp2_cls_fl_t *mrg_fl_rls)
{
	bool valid_c4;
	u16 rl;
	int rc;
	u16 cur_rl_cnt, new_rl_cnt;
	u16 rl_off;
	u16 rnd = 0;
	struct pp2_db_cls_lkp_dcod_t lkp_dcod_db;
	struct pp2_cls_fl_eng_cnt_t eng_hit_cnt[MVPP2_CLS_FLOW_RND_MAX]; /* two hit rounds */

	if (!cur_fl_rls) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}

	if (!new_fl_rls) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}

	if (!mrg_fl_rls) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}

	/* get the lookup DB for this logical flow ID */
	rc = pp2_db_cls_lkp_dcod_get(inst, fl_log_id, &lkp_dcod_db);
	if (rc) {
		pr_err("failed to get lookup decode info for fl_log_id %d\n", fl_log_id);
		return rc;
	}

	rl_off = lkp_dcod_db.flow_off;

	memset(mrg_fl_rls, 0, sizeof(struct pp2_cls_fl_t));
	memset(&eng_hit_cnt[0], 0, sizeof(eng_hit_cnt));

	mrg_fl_rls->fl_log_id = fl_log_id;
	cur_rl_cnt = cur_fl_rls->fl_len;
	new_rl_cnt = new_fl_rls->fl_len;

	/* max flow length check */
	if (cur_rl_cnt + new_rl_cnt > MVPP2_CLS_FLOW_RULE_MAX) {
		pr_err("cur_rl_cnt + new_rl_cnt too large [cur %d new %d]\n",
		       cur_rl_cnt, new_rl_cnt);
		return -EPERM;
	}

	/* merge while loop */
	while (cur_rl_cnt || new_rl_cnt) {
		u16 cur_rl = MVPP2_CLS_FLOW_RULE_MAX;
		u16 new_rl = MVPP2_CLS_FLOW_RULE_MAX;
		bool merge_new = false;
		bool is_seq = false;

#ifdef MVPP2_CLS_DEBUG
		debug_dump_cls_fl("curr", cur_fl_rls);
		debug_dump_cls_fl("new", new_fl_rls);
#endif
		/* update round number */
		if (rnd == 0 && (RND_HIT_CNT(eng_hit_cnt, 0) == MVPP2_CLS_FL_RND_SIZE))
			rnd = 1;

		cur_rl = MVPP2_CLS_FLOW_RULE_MAX;
		new_rl = MVPP2_CLS_FLOW_RULE_MAX;

		/* get first C4 rule from cur and new flows  */
		if (new_fl_rls->eng_cnt.c4) {
			for (rl = 0; rl < new_fl_rls->fl_len; rl++) {
				if (!new_fl_rls->fl[rl].skip &&
				    (new_fl_rls->fl[rl].engine == MVPP2_ENGINE_C4)) {
					break;
				}
			}
			if (rl == new_fl_rls->fl_len) {
				pr_err("eng count inconsistent new_rl=%d fl_len=%d\n",
				       rl, new_fl_rls->fl_len);
				return -EINVAL;
			}
			/* found a new c4 rule, save index */
			new_rl = rl;
		}

		if (cur_fl_rls->eng_cnt.c4) {
			for (rl = 0; rl < cur_fl_rls->fl_len; rl++) {
				if (!cur_fl_rls->fl[rl].skip &&
				    (cur_fl_rls->fl[rl].engine == MVPP2_ENGINE_C4)) {
					break;
				}
			}
			if (rl == cur_fl_rls->fl_len) {
				pr_err("eng count inconsistent new_rl=%d fl_len=%d\n",
				       rl, cur_fl_rls->fl_len);
				return -EINVAL;
			}
			/* found a current c4 rule, save index */
			cur_rl = rl;
		}

		/* no C4 rules, proceed with C3 */
		if ((new_fl_rls->eng_cnt.c4 == 0) && (cur_fl_rls->eng_cnt.c4 == 0))
			goto c3_ins;

		/* decide which rule to merge according to rule priority */
		if (cur_rl != MVPP2_CLS_FLOW_RULE_MAX)
			merge_new = false;
		else if (new_rl != MVPP2_CLS_FLOW_RULE_MAX)
			merge_new = true;

		/* perform merge validation */

		/* max engine hits validation */
		if (RND_HIT_CNT(eng_hit_cnt, 0) == MVPP2_CLS_FL_RND_SIZE * MVPP2_CLS_FLOW_RND_MAX
			+ RND_HIT_CNT(eng_hit_cnt, 1)) {
			pr_err("max CLS entries, failed to add\n");
			return -EPERM;
		}

		/* max allocated flow length validation */
		if (mrg_fl_rls->fl_len == lkp_dcod_db.flow_alloc_len) {
			pr_err("flow alloc length reached alloc_len=%d merge_fl_len=%d\n",
			       lkp_dcod_db.flow_alloc_len, mrg_fl_rls->fl_len);
			return -EPERM;
		}

		/* perform single priority C4 validation */
		if (merge_new)
			rc = pp2_cls_rl_c4_validate(&new_fl_rls->fl[new_rl],
						    mrg_fl_rls,
						    &valid_c4);
		else
			rc = pp2_cls_rl_c4_validate(&cur_fl_rls->fl[cur_rl],
						    mrg_fl_rls,
						    &valid_c4);

		if (rc) {
			pr_err("recvd ret_code(%d)\n", rc);
			return rc;
		}

		if (!valid_c4)
			return -EPERM;

		/* validation done, update engine counters */
		if (merge_new)
			rc = pp2_cls_rl_hit_cnt_upd(&new_fl_rls->fl[new_rl],
						    mrg_fl_rls,
						    &eng_hit_cnt[rnd]);
		else
			rc = pp2_cls_rl_hit_cnt_upd(&cur_fl_rls->fl[cur_rl],
						    mrg_fl_rls,
						    &eng_hit_cnt[rnd]);
		if (rc) {
			pr_err("recvd ret_code(%d)\n", rc);
			return rc;
		}

		/* merge the new rule */
		if (merge_new) {
			new_fl_rls->fl[new_rl].rl_off = rl_off;
			rc = pp2_cls_new_fl_rl_merge(new_rl, new_fl_rls,
						     mrg_fl_rls);
			if (rc) {
				pr_err("failed to merge new C4 rule offset=%d fl_log_id=%d\n",
				       rl_off, new_fl_rls->fl_log_id);
				return rc;
			}

			/* update new rule count */
			new_rl_cnt--;
		} else {
			cur_fl_rls->fl[cur_rl].rl_off = rl_off;
			rc = pp2_cls_new_fl_rl_merge(cur_rl, cur_fl_rls,
						     mrg_fl_rls);

			if (rc) {
				pr_err("failed to merge cur C4 rule offset=%d fl_log_id=%d\n",
				       rl_off, cur_fl_rls->fl_log_id);
				return rc;
			}

			/* update current rule count */
			cur_rl_cnt--;
		}

		rl_off++;
		continue;

c3_ins:
		/* get first C3 rule */

		/* no C3 rules, proceed with C2 */
		if (new_fl_rls->eng_cnt.c3 == 0 &&
		    cur_fl_rls->eng_cnt.c3 == 0)
			goto c2_ins;

		cur_rl = MVPP2_CLS_FLOW_RULE_MAX;
		new_rl = MVPP2_CLS_FLOW_RULE_MAX;

		/* get first C3 rule from cur and new flows  */
		if (new_fl_rls->eng_cnt.c3) {
			for (rl = 0; rl < new_fl_rls->fl_len; rl++) {
				if (!new_fl_rls->fl[rl].skip &&
				    (new_fl_rls->fl[rl].engine == MVPP2_ENGINE_C3_A ||
				     new_fl_rls->fl[rl].engine == MVPP2_ENGINE_C3_B ||
				     new_fl_rls->fl[rl].engine == MVPP2_ENGINE_C3_HA ||
				     new_fl_rls->fl[rl].engine == MVPP2_ENGINE_C3_HB)) {
					break;
				}
			}
			if (rl == new_fl_rls->fl_len) {
				pr_err("eng count inconsistent new_rl=%d fl_len=%d\n",
				       rl, new_fl_rls->fl_len);
				return -EINVAL;
			}
			/* found a new c3 rule, save index */
			new_rl = rl;
		}

		/* get first C3 rule */
		if (cur_fl_rls->eng_cnt.c3) {
			for (rl = 0; rl < cur_fl_rls->fl_len; rl++) {
				if (!cur_fl_rls->fl[rl].skip &&
				    (cur_fl_rls->fl[rl].engine == MVPP2_ENGINE_C3_A ||
				     cur_fl_rls->fl[rl].engine == MVPP2_ENGINE_C3_B ||
				     cur_fl_rls->fl[rl].engine == MVPP2_ENGINE_C3_HA ||
				     cur_fl_rls->fl[rl].engine == MVPP2_ENGINE_C3_HB)) {
					break;
				}
			}
			if (rl == cur_fl_rls->fl_len) {
				pr_err("eng count inconsistent new_rl=%d fl_len=%d\n",
				       rl, cur_fl_rls->fl_len);
				return -EINVAL;
			}
			/* found a current c3 rule, save index */
			cur_rl = rl;
		}

		/* no C3 rules, skip to C2 section */
		if (cur_rl == MVPP2_CLS_FLOW_RULE_MAX &&
		    new_rl == MVPP2_CLS_FLOW_RULE_MAX)
			goto c2_ins;

c3_seq_ins:
		/* decide which rule to merge according to rule priority */
		if (cur_rl != MVPP2_CLS_FLOW_RULE_MAX &&
		    new_rl != MVPP2_CLS_FLOW_RULE_MAX){
			if (cur_fl_rls->fl[cur_rl].prio > new_fl_rls->fl[new_rl].prio)
				merge_new = true;
			else
				merge_new = false;
#if SAME_PRIO_ENABLED
			else if (cur_fl_rls->fl[cur_rl].prio < new_fl_rls->fl[new_rl].prio)
				merge_new = false;
			else{
				pr_err("add same prio rule prio=%d fl_log_id=%d rule #=%d\n",
				       cur_fl_rls->fl[cur_rl].prio,
				       cur_fl_rls->fl[cur_rl].rl_log_id,
				       cur_rl);
				return -EINVAL;
			}
#endif
		} else if (cur_rl != MVPP2_CLS_FLOW_RULE_MAX) {
			merge_new = false;
		} else if (new_rl != MVPP2_CLS_FLOW_RULE_MAX) {
			merge_new = true;
		}

		/* perform merge validation */

		/* round 0 full of C2 and there are some C2 to merge */
		if (rnd == 0 && MVPP2_CLS_C3_RND_MAX == eng_hit_cnt[0].c3) {
			if (!is_seq && (new_fl_rls->eng_cnt.c2 || cur_fl_rls->eng_cnt.c2))
				goto c2_ins;
			rnd = 1;
		} else if (rnd == 1 && MVPP2_CLS_C3_RND_MAX == eng_hit_cnt[1].c3) {
			pr_err("max C3 entries (last round), failed to add\n");
			return -EPERM;
		}

		/* max C3 hits validation */
		if (MVPP2_CLS_C3_RND_MAX * MVPP2_CLS_FLOW_RND_MAX <
			(eng_hit_cnt[0].c3 + eng_hit_cnt[1].c3)) {
			pr_err("max C3 entries, failed to add\n");
			return -EPERM;
		}

		/* max engine hits validation */
		if (MVPP2_CLS_FL_RND_SIZE * MVPP2_CLS_FLOW_RND_MAX <
			(RND_HIT_CNT(eng_hit_cnt, 0) + RND_HIT_CNT(eng_hit_cnt, 1))) {
			pr_err("max CLS entries, failed to add\n");
			return -EPERM;
		}

		/* max allocated flow length validation */
		if (mrg_fl_rls->fl_len == lkp_dcod_db.flow_alloc_len) {
			pr_err("flow alloc length reached alloc_len=%d merge_fl_len=%d\n",
			       lkp_dcod_db.flow_alloc_len, mrg_fl_rls->fl_len);
			return -EPERM;
		}

		/* validation done, update engine counters */
		if (merge_new) {
			rc = pp2_cls_rl_hit_cnt_upd(&new_fl_rls->fl[new_rl],
						    mrg_fl_rls,
						    &eng_hit_cnt[rnd]);
			if (rc) {
				pr_err("recvd ret_code(%d)\n", rc);
				return rc;
			}
		} else {
			rc = pp2_cls_rl_hit_cnt_upd(&cur_fl_rls->fl[cur_rl],
						    mrg_fl_rls,
						    &eng_hit_cnt[rnd]);
			if (rc) {
				pr_err("recvd ret_code(%d)\n", rc);
				return rc;
			}
		}

		/* merge the new rule */
		if (merge_new) {
			new_fl_rls->fl[new_rl].rl_off = rl_off;
			rc = pp2_cls_new_fl_rl_merge(new_rl, new_fl_rls,
						     mrg_fl_rls);

			if (rc) {
				pr_err("failed to merge new C3 rule offset=%d fl_log_id=%d\n",
				       rl_off, new_fl_rls->fl_log_id);
				return rc;
			}

			/* update new rule count */
			new_rl_cnt--;
		} else {
			cur_fl_rls->fl[cur_rl].rl_off = rl_off;
			rc = pp2_cls_new_fl_rl_merge(cur_rl, cur_fl_rls,
						     mrg_fl_rls);

			if (rc) {
				pr_err("failed to merge cur C3 rule offset=%d fl_log_id=%d\n",
				       rl_off, cur_fl_rls->fl_log_id);
				return rc;
			}

			/* update current rule count */
			cur_rl_cnt--;
		}

		rl_off++;

		if (merge_new) {
			if (!is_seq) {
				if (new_fl_rls->fl[new_rl].seq_ctrl != MVPP2_CLS_SEQ_CTRL_NORMAL) {
					if (new_fl_rls->fl[new_rl].seq_ctrl != MVPP2_CLS_SEQ_CTRL_FIRST_TYPE_1) {
						pr_err("seqence control rule not start from first type (new_rl %d)\n"
						       , new_rl);
						return -EINVAL;
					}
					is_seq = true;
					cur_rl = MVPP2_CLS_FLOW_RULE_MAX;
				}
			} else {
				if (new_fl_rls->fl[new_rl].seq_ctrl == MVPP2_CLS_SEQ_CTRL_LAST)
					is_seq = false;
			}
			if (is_seq) {
				new_rl++;
				if (new_fl_rls->fl[new_rl].skip ||
				    new_fl_rls->fl[new_rl].seq_ctrl == MVPP2_CLS_SEQ_CTRL_NORMAL ||
				    new_fl_rls->fl[new_rl].seq_ctrl == MVPP2_CLS_SEQ_CTRL_FIRST_TYPE_1) {
					pr_err("seqence control rule is not continuous (new_rl %d)\n", new_rl);
					return -EINVAL;
				}
				if (new_fl_rls->fl[new_rl].engine == MVPP2_ENGINE_C3_A ||
				    new_fl_rls->fl[new_rl].engine == MVPP2_ENGINE_C3_B ||
				    new_fl_rls->fl[new_rl].engine == MVPP2_ENGINE_C3_HA ||
				    new_fl_rls->fl[new_rl].engine == MVPP2_ENGINE_C3_HB) {
					goto c3_seq_ins;
				} else if (new_fl_rls->fl[new_rl].engine == MVPP2_ENGINE_C2) {
					goto c2_seq_ins;
				} else {
					pr_err("seqence control not support c4 rule\n");
					return -EINVAL;
				}
			}
		} else {
			if (!is_seq) {
				if (cur_fl_rls->fl[cur_rl].seq_ctrl != MVPP2_CLS_SEQ_CTRL_NORMAL) {
					if (new_fl_rls->fl[new_rl].seq_ctrl != MVPP2_CLS_SEQ_CTRL_FIRST_TYPE_1) {
						pr_err("seqence control rule not start from first type (new_rl %d)\n",
						       new_rl);
						return -EINVAL;
					}
					is_seq = true;
					new_rl = MVPP2_CLS_FLOW_RULE_MAX;
				}
			} else {
				if (cur_fl_rls->fl[cur_rl].seq_ctrl == MVPP2_CLS_SEQ_CTRL_LAST)
					is_seq = false;
			}
			if (is_seq) {
				cur_rl++;
				if (cur_fl_rls->fl[cur_rl].skip ||
				    cur_fl_rls->fl[cur_rl].seq_ctrl == MVPP2_CLS_SEQ_CTRL_NORMAL ||
					cur_fl_rls->fl[cur_rl].seq_ctrl == MVPP2_CLS_SEQ_CTRL_FIRST_TYPE_1) {
					pr_err("seqence control rule is not continuous (cur_rl %d)\n", cur_rl);
					return -EINVAL;
				}
				if (cur_fl_rls->fl[cur_rl].engine == MVPP2_ENGINE_C3_A ||
				    cur_fl_rls->fl[cur_rl].engine == MVPP2_ENGINE_C3_B ||
					cur_fl_rls->fl[cur_rl].engine == MVPP2_ENGINE_C3_HA ||
					cur_fl_rls->fl[cur_rl].engine == MVPP2_ENGINE_C3_HB) {
					goto c3_seq_ins;
				} else if (cur_fl_rls->fl[cur_rl].engine == MVPP2_ENGINE_C2) {
					goto c2_seq_ins;
				} else {
					pr_err("seqence control not support c4 rule\n");
					return -EINVAL;
				}
			}
		}
		continue;

c2_ins:
		/* no C2 in both flow list, skip */
		if (new_fl_rls->eng_cnt.c2 == 0 &&
		    cur_fl_rls->eng_cnt.c2 == 0)
			continue;

		cur_rl = MVPP2_CLS_FLOW_RULE_MAX;
		new_rl = MVPP2_CLS_FLOW_RULE_MAX;

		/* get first C2 rule from cur and new flows  */
		if (new_fl_rls->eng_cnt.c2) {
			for (rl = 0; rl < new_fl_rls->fl_len; rl++) {
				if (!new_fl_rls->fl[rl].skip &&
				    new_fl_rls->fl[rl].engine == MVPP2_ENGINE_C2) {
					break;
				}
			}
			if (rl == new_fl_rls->fl_len) {
				pr_err("eng count inconsistent new_rl=%d fl_len=%d\n",
				       rl, new_fl_rls->fl_len);
				return -EINVAL;
			}
			/* found a new c2 rule, save index */
			new_rl = rl;
		}

		/* get first C2 rule */
		if (cur_fl_rls->eng_cnt.c2) {
			for (rl = 0; rl < cur_fl_rls->fl_len; rl++) {
				if (!cur_fl_rls->fl[rl].skip &&
				    (cur_fl_rls->fl[rl].engine == MVPP2_ENGINE_C2)) {
					break;
				}
			}
			if (rl == cur_fl_rls->fl_len) {
				pr_err("eng count inconsistent new_rl=%d fl_len=%d\n",
				       rl, cur_fl_rls->fl_len);
				return -EINVAL;
			}
			/* found a current c2 rule, save index */
			cur_rl = rl;
		}

		if (cur_rl == MVPP2_CLS_FLOW_RULE_MAX &&
		    new_rl == MVPP2_CLS_FLOW_RULE_MAX)
			continue;

c2_seq_ins:

		/* decide which rule to merge according to rule priority */
		if (cur_rl != MVPP2_CLS_FLOW_RULE_MAX &&
		    new_rl != MVPP2_CLS_FLOW_RULE_MAX){
			if (cur_fl_rls->fl[cur_rl].prio > new_fl_rls->fl[new_rl].prio)
				merge_new = true;
			else
				merge_new = false;
#if SAME_PRIO_ENABLED
			else if (cur_fl_rls->fl[cur_rl].prio < new_fl_rls->fl[new_rl].prio)
				merge_new = false;
			else{
				pr_err("add same prio rule prio=%d fl_log_id=%d rule #=%d\n",
				       cur_fl_rls->fl[cur_rl].prio,
				       cur_fl_rls->fl[cur_rl].rl_log_id,
				       cur_rl);
				return -EINVAL;
			}
#endif
		} else if (cur_rl != MVPP2_CLS_FLOW_RULE_MAX) {
			merge_new = false;
		} else if (new_rl != MVPP2_CLS_FLOW_RULE_MAX) {
			merge_new = true;
		}

		/* perform merge validation */

		/* running out of C3 rules, but next highest priority C2 rule is part of N-tuple - update round */
		if (merge_new) {
			if (!is_seq && new_fl_rls->fl[new_rl].seq_ctrl != MVPP2_CLS_SEQ_CTRL_NORMAL) {
				rnd = 1;
				continue;
			}
		} else {
			if (!is_seq && cur_fl_rls->fl[cur_rl].seq_ctrl != MVPP2_CLS_SEQ_CTRL_NORMAL) {
				rnd = 1;
				continue;
			}
		}

		/* update round number */
		if (rnd == 0 && MVPP2_CLS_C2_RND_MAX == eng_hit_cnt[0].c2) {
			rnd = 1;
		} else if (rnd == 1 && MVPP2_CLS_C2_RND_MAX == eng_hit_cnt[1].c2) {
			pr_err("max C2 entries (last round), failed to add\n");
			return -EPERM;
		}

		/* max C2 hits validation */
		if (MVPP2_CLS_C2_RND_MAX * MVPP2_CLS_FLOW_RND_MAX <
			(eng_hit_cnt[0].c2 + eng_hit_cnt[1].c2)) {
			pr_err("max C2 entries, failed to add\n");
			return -EPERM;
		}

		/* max engine hits validation */
		if (MVPP2_CLS_FL_RND_SIZE * MVPP2_CLS_FLOW_RND_MAX <
			(RND_HIT_CNT(eng_hit_cnt, 0) + RND_HIT_CNT(eng_hit_cnt, 1))) {
			pr_err("max CLS entries, failed to add\n");
			return -EPERM;
		}

		/* max allocated flow length validation */
		if (mrg_fl_rls->fl_len == lkp_dcod_db.flow_alloc_len) {
			pr_err("flow alloc length reached alloc_len=%d merge_fl_len=%d\n",
			       lkp_dcod_db.flow_alloc_len, mrg_fl_rls->fl_len);
			return -EPERM;
		}

		/* validation done, update engine counters */
		if (merge_new) {
			rc = pp2_cls_rl_hit_cnt_upd(&new_fl_rls->fl[new_rl],
						    mrg_fl_rls,
						    &eng_hit_cnt[rnd]);
			if (rc) {
				pr_err("recvd ret_code(%d)\n", rc);
				return rc;
			}
		} else {
			rc = pp2_cls_rl_hit_cnt_upd(&cur_fl_rls->fl[cur_rl],
						    mrg_fl_rls,
						    &eng_hit_cnt[rnd]);
			if (rc) {
				pr_err("recvd ret_code(%d)\n", rc);
				return rc;
			}
		}

		/* merge the new rule */
		if (merge_new) {
			new_fl_rls->fl[new_rl].rl_off = rl_off;
			rc = pp2_cls_new_fl_rl_merge(new_rl, new_fl_rls,
						     mrg_fl_rls);

			if (rc) {
				pr_err("failed to merge new C2 rule offset=%d fl_log_id=%d\n",
				       rl_off, new_fl_rls->fl_log_id);
				return rc;
			}

			/* update new rule count */
			new_rl_cnt--;
		} else {
			cur_fl_rls->fl[cur_rl].rl_off = rl_off;
			rc = pp2_cls_new_fl_rl_merge(cur_rl, cur_fl_rls,
						     mrg_fl_rls);

			if (rc) {
				pr_err("failed to merge cur C2 rule offset=%d fl_log_id=%d\n",
				       rl_off, cur_fl_rls->fl_log_id);
				return rc;
			}

			/* update current rule count */
			cur_rl_cnt--;
		}
		rl_off++;

		if (merge_new) {
			if (!is_seq) {
				if (new_fl_rls->fl[new_rl].seq_ctrl != MVPP2_CLS_SEQ_CTRL_NORMAL) {
					if (new_fl_rls->fl[new_rl].seq_ctrl != MVPP2_CLS_SEQ_CTRL_FIRST_TYPE_1) {
						pr_err("seqence control rule not start from first type (new_rl %d)\n",
						       new_rl);
						return -EINVAL;
					}
					is_seq = true;
					cur_rl = MVPP2_CLS_FLOW_RULE_MAX;
				}
			} else {
				if (new_fl_rls->fl[new_rl].seq_ctrl == MVPP2_CLS_SEQ_CTRL_LAST)
					is_seq = false;
			}
			if (is_seq) {
				new_rl++;
				if (new_fl_rls->fl[new_rl].skip ||
				    new_fl_rls->fl[new_rl].seq_ctrl == MVPP2_CLS_SEQ_CTRL_NORMAL ||
				    new_fl_rls->fl[new_rl].seq_ctrl == MVPP2_CLS_SEQ_CTRL_FIRST_TYPE_1) {
					pr_err("seqence control rule is not continuous (new_rl %d)\n", new_rl);
					return -EINVAL;
				}
				if (new_fl_rls->fl[new_rl].engine == MVPP2_ENGINE_C3_A ||
				    new_fl_rls->fl[new_rl].engine == MVPP2_ENGINE_C3_B ||
				    new_fl_rls->fl[new_rl].engine == MVPP2_ENGINE_C3_HA ||
				    new_fl_rls->fl[new_rl].engine == MVPP2_ENGINE_C3_HB) {
					goto c3_seq_ins;
				} else if (new_fl_rls->fl[new_rl].engine == MVPP2_ENGINE_C2) {
					goto c2_seq_ins;
				} else {
					pr_err("seqence control not support c4 rule\n");
					return -EINVAL;
				}
			}
		} else {
			if (!is_seq) {
				if (cur_fl_rls->fl[cur_rl].seq_ctrl != MVPP2_CLS_SEQ_CTRL_NORMAL) {
					if (new_fl_rls->fl[new_rl].seq_ctrl != MVPP2_CLS_SEQ_CTRL_FIRST_TYPE_1) {
						pr_err("seqence control rule not start from first type (new_rl %d)\n",
						       new_rl);
						return -EINVAL;
					}
					is_seq = true;
					new_rl = MVPP2_CLS_FLOW_RULE_MAX;
				}
			} else {
				if (cur_fl_rls->fl[cur_rl].seq_ctrl == MVPP2_CLS_SEQ_CTRL_LAST)
					is_seq = false;
			}
			if (is_seq) {
				cur_rl++;
				if (cur_fl_rls->fl[cur_rl].skip ||
				    cur_fl_rls->fl[cur_rl].seq_ctrl == MVPP2_CLS_SEQ_CTRL_NORMAL ||
				    cur_fl_rls->fl[cur_rl].seq_ctrl == MVPP2_CLS_SEQ_CTRL_FIRST_TYPE_1) {
					pr_err("seqence control rule is not continuous (cur_rl %d)\n", cur_rl);
					return -EINVAL;
				}
				if (cur_fl_rls->fl[cur_rl].engine == MVPP2_ENGINE_C3_A ||
				    cur_fl_rls->fl[cur_rl].engine == MVPP2_ENGINE_C3_B ||
				    cur_fl_rls->fl[cur_rl].engine == MVPP2_ENGINE_C3_HA ||
				    cur_fl_rls->fl[cur_rl].engine == MVPP2_ENGINE_C3_HB) {
					goto c3_seq_ins;
				} else if (cur_fl_rls->fl[cur_rl].engine == MVPP2_ENGINE_C2) {
					goto c2_seq_ins;
				} else {
					pr_err("seqence control not support c4 rule\n");
					return -EINVAL;
				}
			}
		}
	}

	return 0;
}

/*******************************************************************************
 * pp2_cls_fl_rl_db_set
 *
 * DESCRIPTION: The routine writes a flow rule to DB
 *
 * INPUTS:
 *	inst - packet processor instance
 *	rl - the rule to enable information structure
 *	fl_log_id - flow logical ID for the rule
 *
 * OUTPUTS:
 *	None
 *
 * RETURNS:
 *	0 on success, error-code otherwise
 ******************************************************************************/
static int pp2_cls_fl_rl_db_set(struct pp2_inst *inst,
				struct pp2_cls_rl_entry_t *rl, u16 fl_log_id)
{
	struct pp2_db_cls_fl_rule_t rl_db;
	int rc;
	u16 cur_rl_off;
	int loop;

	if (!rl) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}

	memset(&rl_db, 0, sizeof(rl_db));

	/* different handling for rule states */
	switch (rl->state) {
	case MVPP2_MRG_NEW:
		/* new rule, assign logical rule id */
		rc = pp2_db_cls_rl_off_free_set(inst, rl->rl_off, &rl->rl_log_id);
		if (rc) {
			pr_err("got error for rule offset %d\n", rl->rl_off);
			return rc;
		}

		/* save the new logical rule ID in DB */
		rl_db.rl_log_id = rl->rl_log_id;
		break;

	case MVPP2_MRG_NEW_EXISTS:
	case MVPP2_MRG_NOT_NEW:
		/* this new entry exists */
		rc = pp2_db_cls_rl_off_get(inst, &cur_rl_off, rl->rl_log_id);
		if (rc) {
			pr_err("could not get rl_log_id %d offset\n", rl->rl_log_id);
			return rc;
		}

		/* if logical rule offset changed, update new offset */
		if (cur_rl_off != rl->rl_off) {
			rc = pp2_db_cls_rl_off_set(inst, rl->rl_off, rl->rl_log_id);
			if (rc) {
				pr_err("could not set rule offset %d for rl_log_id %d\n",
				       rl->rl_off, rl->rl_log_id);
				return rc;
			}
		}

		/* this rule is not new, use the current ref_count */
		memcpy(&rl_db.ref_cnt[0], &rl->ref_cnt[0], PP2_NUM_PORTS * sizeof(u16));
		/* same rule logical id */
		rl_db.rl_log_id = rl->rl_log_id;
		break;

	default:
		 pr_err("Invalid rule state [rl->state=%d]\n", rl->state);
		 return -EINVAL;
	}

	memcpy(rl_db.field_id, rl->field_id, sizeof(rl->field_id));

	if (rl->state != MVPP2_MRG_NOT_NEW) {
		/* update ref_count for new entries only */
		for (loop = 0; loop < PP2_NUM_PORTS; loop++) {
			if ((1 << loop) & rl->port_bm)
				rl_db.ref_cnt[loop] = (rl->enabled) ? rl->ref_cnt[loop] + 1 : 0;
		}
	}

	rl_db.enabled		= rl->enabled;
	rl_db.engine		= rl->engine;
	rl_db.field_id_cnt	= rl->field_id_cnt;
	rl_db.lu_type		= rl->lu_type;
	rl_db.port_bm		= rl->port_bm;
	rl_db.port_type		= rl->port_type;
	rl_db.prio		= rl->prio;
	rl_db.udf7		= rl->udf7;

	rc = pp2_db_cls_fl_rule_set(inst, rl->rl_off, &rl_db);
	if (rc) {
		pr_err("recvd ret_code(%d)\n", rc);
		return rc;
	}

	return 0;
}

/*******************************************************************************
 * pp2_cls_fl_rl_hw_set
 *
 * DESCRIPTION: The routine writes a flow rule to HW
 *
 * INPUTS:
 *	rl - the rule to enable information structure
 *	is_last - a flag indicating if he rule is last in flow
 *
 * OUTPUTS:
 *	None
 *
 * RETURNS:
 *	0 on success, error-code otherwise
 ******************************************************************************/
static int pp2_cls_fl_rl_hw_set(uintptr_t cpu_slot,
				struct pp2_cls_rl_entry_t *rl,
				bool is_last)
{
	struct mv_pp2x_cls_flow_entry fe;
	int rc;
	u16 fid;
	/* enum pp2_init_us_2g_trunk_mode_t us_2g_trunk_support; */

	if (!rl) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}

	mv_pp2x_cls_sw_flow_clear(&fe);

	rc = mv_pp2x_cls_sw_flow_engine_set(&fe, rl->engine, is_last);
	if (rc) {
		pr_err("recvd ret_code(%d)\n", rc);
		return rc;
	}

	/* set the port_type and port_bm according to enable configuration */
	if (rl->enabled)
		rc = mv_pp2x_cls_sw_flow_port_set(&fe, rl->port_type, rl->port_bm);
	else
		rc = mv_pp2x_cls_sw_flow_port_set(&fe, 0, 0);

	if (rc) {
		pr_err("recvd ret_code(%d)\n", rc);
		return rc;
	}

	rc = mv_pp2x_cls_sw_flow_extra_set(&fe, rl->lu_type, rl->prio);
	if (rc) {
		pr_err("recvd ret_code(%d)\n", rc);
		return rc;
	}

	rc = mv_pp2x_cls_sw_flow_hek_num_set(&fe, rl->field_id_cnt);
	if (rc) {
		pr_err("recvd ret_code(%d)\n", rc);
		return rc;
	}

	/* Set Port ID Select, selects the value of the Port ID forwareded to the C2, C3 engines*/
	/* Check US 2G trunk is supported or not */
	/* rc = pp2_db_generic_param_get(MVPP2_DB_PARAM_US_2G_SUPPORT, &us_2g_trunk_support); */
	/* IF_ERROR_STR(TPM_MNG_MOD, rc, "get US 2G trunk value failed\n"); */

	/* for 2G US case, SRC port DS rules are all GMAC1 and PON
	 * so in classifier and C2/3, all src port are GMAC1 and PON too,
	 * and the src port value of C2/3 comes from classifier, not packet
	 */
	/* if (TPM_US_2G_TRUNK_SUPPORTED == us_2g_trunk_support)
	 *	rc = mv_pp2x_cls_sw_flow_portid_select(&fe, MVPP2_CLS_PORT_ID_FROM_TBL);
	 * else
	 *	rc = mv_pp2x_cls_sw_flow_portid_select(&fe, MVPP2_CLS_PORT_ID_FROM_PKT);
	 *
	 */

	rc = mv_pp2x_cls_sw_flow_portid_select(&fe, MVPP2_CLS_PORT_ID_FROM_PKT);
	if (rc) {
		pr_err("recvd ret_code(%d)\n", rc);
		return rc;
	}

	rc = mv_pp2x_cls_sw_flow_seq_ctrl_set(&fe, rl->seq_ctrl);
	if (rc) {
		pr_err("recvd ret_code(%d)\n", rc);
		return rc;
	}

	rc = mv_pp2x_cls_sw_flow_udf7_set(&fe, rl->udf7);
	if (rc) {
		pr_err("recvd ret_code(%d)\n", rc);
		return rc;
	}

	for (fid = 0; fid < rl->field_id_cnt; fid++) {
		rc = mv_pp2x_cls_sw_flow_hek_set(&fe, fid, rl->field_id[fid]);
		if (rc) {
			pr_err("recvd ret_code(%d)\n", rc);
			return rc;
		}
	}
	fe.index = rl->rl_off;
	rc = mv_pp2x_cls_hw_flow_write(cpu_slot, &fe);
	if (rc) {
		pr_err("recvd ret_code(%d)\n", rc);
		return rc;
	}

	return 0;
}

/*******************************************************************************
 * pp2_cls_fl_port_hw_read
 *
 * DESCRIPTION: Get flow port_bm from the hw
 *
 * INPUTS:
 *	inst - packet processor instance
 *	rl_log_id - allocated rule logical ID
 *
 * OUTPUTS:
 *	port_bm - return flow port_bm from the hw
 *
 * RETURNS:
 *	0 on success, error-code otherwise
 ******************************************************************************/
static int pp2_cls_fl_port_hw_read(struct pp2_inst *inst, int rl_log_id, int *port_bm)
{
	struct mv_pp2x_cls_flow_entry fe;
	int port_type;
	int rc;
	u16 off;
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);

	/* get the rule offset according to rule logical ID */
	rc = pp2_db_cls_rl_off_get(inst, &off, rl_log_id);
	if (rc) {
		pr_err("%s(%d): recvd ret_code(%d)\n", __func__, __LINE__, rc);
		return rc;
	}

	rc = mv_pp2x_cls_hw_flow_read(cpu_slot, off, &fe);
	if (rc) {
		pr_err("%s(%d): recvd ret_code(%d)\n", __func__, __LINE__, rc);
		return rc;
	}

	rc = mv_pp2x_cls_sw_flow_port_get(&fe, &port_type, port_bm);
	if (rc) {
		pr_err("%s(%d): recvd ret_code(%d)\n", __func__, __LINE__, rc);
		return rc;
	}

	return 0;
}

/*******************************************************************************
 * pp2_cls_fl_rl_hw_ena
 *
 * DESCRIPTION: The routine enables a flow rule in HW according to rule port_type
 *		and port_bm
 *
 * INPUTS:
 *	inst - packet processor instance
 *	rl_en - the rule to enable information structure
 *
 * OUTPUTS:
 *	None
 *
 * RETURNS:
 *	0 on success, error-code otherwise
 ******************************************************************************/
static int pp2_cls_fl_rl_hw_ena(struct pp2_inst *inst, struct pp2_cls_fl_rule_entry_t	*rl_en)
{
	struct mv_pp2x_cls_flow_entry fe;
	int rc;
	u16 off;
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);

	if (!rl_en) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}

	/* get the rule offset according to rule logical ID */
	rc = pp2_db_cls_rl_off_get(inst, &off, rl_en->rl_log_id);
	if (rc) {
		pr_err("recvd ret_code(%d)\n", rc);
		return rc;
	}

	rc = mv_pp2x_cls_hw_flow_read(cpu_slot, off, &fe);
	if (rc) {
		pr_err("recvd ret_code(%d)\n", rc);
		return rc;
	}

	/* enable the rule according to DB settings */
	rc = mv_pp2x_cls_sw_flow_port_set(&fe, rl_en->port_type, rl_en->port_bm);
	if (rc) {
		pr_err("recvd ret_code(%d)\n", rc);
		return rc;
	}

	fe.index = off;
	rc = mv_pp2x_cls_hw_flow_write(cpu_slot, &fe);
	if (rc) {
		pr_err("recvd ret_code(%d)\n", rc);
		return rc;
	}

	return 0;
}

/*******************************************************************************
 * pp2_cls_fl_rl_hw_dis
 *
 * DESCRIPTION: The routine disables a flow rule in HW, sets the port_type and
 *		port_bm to zero
 *
 * INPUTS:
 *	off - the offset of the rule
 *
 * OUTPUTS:
 *	None
 *
 * RETURNS:
 *	0 on success, error-code otherwise
 ******************************************************************************/
static int pp2_cls_fl_rl_hw_dis(uintptr_t cpu_slot, u16 off)
{
	struct mv_pp2x_cls_flow_entry	fe;
	int			rc;

	rc = mv_pp2x_cls_hw_flow_read(cpu_slot, off, &fe);
	if (rc) {
		pr_err("recvd ret_code(%d)\n", rc);
		return rc;
	}

	rc = mv_pp2x_cls_sw_flow_port_set(&fe, 0, 0);
	if (rc) {
		pr_err("recvd ret_code(%d)\n", rc);
		return rc;
	}

	fe.index = off;
	rc = mv_pp2x_cls_hw_flow_write(cpu_slot, &fe);
	if (rc) {
		pr_err("recvd ret_code(%d)\n", rc);
		return rc;
	}

	return 0;
}

/*******************************************************************************
 * pp2_cls_fl_rls_set
 *
 * DESCRIPTION: The routine sets the merged flow to HW and updated DBs
 *
 * INPUTS:
 *	inst - packet processor instance
 *	fl_rls - a list of merged rules of a single logical flow ID to set
 *
 * OUTPUTS:
 *	None
 *
 * RETURNS:
 *	0 on success, error-code otherwise
 ******************************************************************************/
static int pp2_cls_fl_rls_set(struct pp2_inst *inst, struct pp2_cls_fl_t *fl_rls)
{
	struct pp2_db_cls_lkp_dcod_t lkp_dcod_db;
	int rc;
	u32 free_db_cnt;
	u16 rl_idx;
	bool is_last;
	u16 new_rl_cnt = 0;
	struct pp2_cls_rl_entry_t *rl;
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);

	if (!fl_rls) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}

	/* get the lookup DB for this logical flow ID */
	rc = pp2_db_cls_lkp_dcod_get(inst, fl_rls->fl_log_id, &lkp_dcod_db);
	if (rc) {
		pr_err("failed to get lookup decode info for fl_log_id %d\n", fl_rls->fl_log_id);
		return rc;
	}

	/* count all new rules */
	for (rl_idx = 0; rl_idx < fl_rls->fl_len; rl_idx++) {
		rl = &fl_rls->fl[rl_idx];
		if (rl->rl_log_id == MVPP2_CLS_UNDF_FL_LOG_ID)
			new_rl_cnt++;
	}

	/* validate enough DB entries for new rules */
	if (new_rl_cnt) {
		rc = pp2_db_cls_rl_off_free_nr(inst, &free_db_cnt);
		if (rc) {
			pr_err("recvd ret_code(%d)\n", rc);
			return rc;
		}

		if (free_db_cnt < new_rl_cnt) {
			pr_err("not enough free rule DB[free_db_cnt=%d new_rl_cnt=%d]\n",
			       free_db_cnt, new_rl_cnt);
			return -EPERM;
		}
	}

	/* set the flow rules */
	for (rl_idx = 0; rl_idx < fl_rls->fl_len; rl_idx++) {
		rl = &fl_rls->fl[rl_idx];
		is_last = (rl_idx == fl_rls->fl_len - 1) ? true : false;

		/* write rule to HW */
		rc = pp2_cls_fl_rl_hw_set(cpu_slot, rl, is_last);
		if (rc) {
			pr_err("recvd ret_code(%d)\n", rc);
			return rc;
		}
	}

	/* set the lookup decode table */
	rc = pp2_cls_lkp_dcod_hw_set(inst, fl_rls);
	if (rc) {
		pr_err("recvd ret_code(%d)\n", rc);
		return rc;
	}

	/* update DB for rules and lookup decode */
	for (rl_idx = 0; rl_idx < fl_rls->fl_len; rl_idx++) {
		rl = &fl_rls->fl[rl_idx];

		/* write rule to DB */
		rc = pp2_cls_fl_rl_db_set(inst, rl, fl_rls->fl_log_id);
		if (rc) {
			pr_err("recvd ret_code(%d)\n", rc);
			return rc;
		}
	}

	/* update flow actual length */
	lkp_dcod_db.flow_len = fl_rls->fl_len;
	rc = pp2_db_cls_lkp_dcod_set(inst, fl_rls->fl_log_id, &lkp_dcod_db);
	if (rc) {
		pr_err("failed to set lookup decode info for fl_log_id %d\n", fl_rls->fl_log_id);
		return rc;
	}

	return 0;
}

/*******************************************************************************
 * pp2_cls_fl_cur_get
 *
 * DESCRIPTION: The routine returns the current flow rules for the flow logical ID
 *
 * INPUTS:
 *	inst - packet processor instance
 *	fl_log_id - the flow logical ID
 *	cur_fl	  - flow structure that holds all flows and additional information
 *
 * OUTPUTS:
 *	None
 *
 * RETURNS:
 *	0 on success, error-code otherwise
 ******************************************************************************/
static int pp2_cls_fl_cur_get(struct pp2_inst *inst,
			      u16 fl_log_id,
			      struct pp2_cls_fl_t *cur_fl)
{
	struct pp2_db_cls_fl_rule_list_t *fl_rl_db;
	struct pp2_db_cls_lkp_dcod_t lkp_dcod_db;
	int rc;
	u16 i;

	if (!cur_fl) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}

	/* get the lookup DB for this logical flow ID */
	rc = pp2_db_cls_lkp_dcod_get(inst, fl_log_id, &lkp_dcod_db);
	if (rc) {
		pr_err("failed to get lookup decode info for fl_log_id %d\n", fl_log_id);
		return rc;
	}

#ifdef MVPP2_CLS_DEBUG
	debug_dump_lkp_dcod_db("pp2_cls_fl_cur_get", &lkp_dcod_db);
#endif

	memset(cur_fl, 0, sizeof(struct pp2_cls_fl_t));
	fl_rl_db = kmalloc(sizeof(*fl_rl_db), GFP_KERNEL);
	if (!fl_rl_db) {
		pr_err("%s(%d) Error allocating memory!\n", __func__, __LINE__);
		return -ENOMEM;
	}
	memset(fl_rl_db, 0, sizeof(struct pp2_db_cls_fl_rule_list_t));

	/* update DB flow length */
	fl_rl_db->flow_len = lkp_dcod_db.flow_len;
	cur_fl->fl_log_id = fl_log_id;

	if (lkp_dcod_db.flow_len == 0) {
		kfree(fl_rl_db);
		return 0;
	}

	rc = pp2_db_cls_fl_rule_list_get(inst, lkp_dcod_db.flow_off,
					 lkp_dcod_db.flow_len, &fl_rl_db->flow[0]);

	if (rc) {
		pr_err("fail to get flow rule list DB data, fl_log_id=%d, flow_off=%d, flow_len=%d\n",
		       fl_log_id, lkp_dcod_db.flow_off, lkp_dcod_db.flow_len);
		kfree(fl_rl_db);
		return rc;
	}

	/* set the current flow additional configurations */
	cur_fl->fl_len = fl_rl_db->flow_len;

	for (i = 0; i < fl_rl_db->flow_len; i++) {
		cur_fl->fl[i].enabled	= fl_rl_db->flow[i].enabled;
		cur_fl->fl[i].engine	= fl_rl_db->flow[i].engine;
		cur_fl->fl[i].field_id_cnt = fl_rl_db->flow[i].field_id_cnt;

		memcpy(cur_fl->fl[i].field_id,
		       fl_rl_db->flow[i].field_id,
			sizeof(cur_fl->fl[i].field_id));

		cur_fl->fl[i].rl_off	= lkp_dcod_db.flow_off + i;
		cur_fl->fl[i].lu_type	= fl_rl_db->flow[i].lu_type;
		cur_fl->fl[i].port_bm	= fl_rl_db->flow[i].port_bm;
		cur_fl->fl[i].port_type = fl_rl_db->flow[i].port_type;
		cur_fl->fl[i].prio	= fl_rl_db->flow[i].prio;
		cur_fl->fl[i].udf7	= fl_rl_db->flow[i].udf7;
		memcpy(&cur_fl->fl[i].ref_cnt[0],
		       &fl_rl_db->flow[i].ref_cnt[0],
			PP2_NUM_PORTS * sizeof(u16));
		cur_fl->fl[i].rl_log_id = fl_rl_db->flow[i].rl_log_id;
		cur_fl->fl[i].state	= MVPP2_MRG_NOT_NEW;
		cur_fl->fl[i].skip	= 0;

		rc = pp2_cls_fl_rl_eng_cnt_upd(MVPP2_CNT_INC, fl_rl_db->flow[i].engine, &cur_fl->eng_cnt);
		if (rc) {
			pr_err("recvd ret_code(%d)\n", rc);
			kfree(fl_rl_db);
			return rc;
		}
	}

	kfree(fl_rl_db);
	return 0;
}

/*******************************************************************************
 * pp2_cls_fl_rls_log_rl_id_upd
 *
 * DESCRIPTION: The routine update the rl_log_id in the added rule list from
 *		the merged logical flow rules
 *
 * INPUTS:
 *	add_rls - the added rule list
 *	mrg_rls - the merged flow rules
 *
 * OUTPUTS:
 *	fl_rls->fl[].rl_log_id - allocated rule logical ID
 *
 * RETURNS:
 *	0 on success, error-code otherwise
 ******************************************************************************/
static int pp2_cls_fl_rls_log_rl_id_upd(struct pp2_cls_fl_rule_list_t *add_rls,
					struct pp2_cls_fl_t *mrg_rls)
{
	u16 i, j;

	if (!add_rls) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}

	if (!mrg_rls) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}

	for (i = 0; i < add_rls->fl_len; i++) {
		if (add_rls->fl[i].fl_log_id != mrg_rls->fl_log_id)
			continue;

		for (j = 0; j < mrg_rls->fl_len; j++) {
			if (add_rls->fl[i].engine	== mrg_rls->fl[j].engine	&&
			    add_rls->fl[i].field_id_cnt == mrg_rls->fl[j].field_id_cnt	&&
			    add_rls->fl[i].lu_type	== mrg_rls->fl[j].lu_type	&&
			    add_rls->fl[i].port_bm	== mrg_rls->fl[j].port_bm	&&
			    add_rls->fl[i].port_type	== mrg_rls->fl[j].port_type	&&
			    add_rls->fl[i].prio	== mrg_rls->fl[j].prio		&&
			    add_rls->fl[i].udf7	== mrg_rls->fl[j].udf7		&&
			    !memcmp(add_rls->fl[i].field_id, mrg_rls->fl[j].field_id,
				    sizeof(add_rls->fl[i].field_id))) {
				/* found the added rule, update logial rule ID */
				add_rls->fl[i].rl_log_id = mrg_rls->fl[j].rl_log_id;
				break;
			}
		}
		if (j == mrg_rls->fl_len) {
			pr_err("new rule #%d not found in merged flow\n", i);
			return -EINVAL;
		}
	}

	return 0;
}

static int pp2_cls_fl_nt_rule_reorder(struct pp2_cls_fl_t *fl_rls)
{
	int i, j, len;
	u16 rl_off;
	struct pp2_cls_fl_eng_cnt_t eng_cnt[MVPP2_CLS_FLOW_RND_MAX]; /* two hit rounds */

	memset(&eng_cnt[0], 0, sizeof(eng_cnt));

	rl_off = fl_rls->fl[0].rl_off;

	for (i = 0; i < fl_rls->fl_len; i++) {
		/* Skip the normal flow rules */
		if (fl_rls->fl[i].seq_ctrl == MVPP2_CLS_SEQ_CTRL_NORMAL)
			continue;
		/* Find the first type rule */
		if (fl_rls->fl[i].seq_ctrl != MVPP2_CLS_SEQ_CTRL_FIRST_TYPE_1) {
			pr_err("n-tuple rule (%d) not start from first type 1\n", i);
			return -EINVAL;
		}
		/* Get the n-tuple flow length */
		for (len = 1; len + i < fl_rls->fl_len; len++) {
			if (fl_rls->fl[len + i].seq_ctrl != MVPP2_CLS_SEQ_CTRL_MIDDLE &&
			    fl_rls->fl[len + i].seq_ctrl != MVPP2_CLS_SEQ_CTRL_LAST) {
				pr_err("unexpect rule (%d) sequence type (%d)\n", len + i,
				       fl_rls->fl[len + i].seq_ctrl);
				return -EINVAL;
			}
			if (fl_rls->fl[len + i].seq_ctrl == MVPP2_CLS_SEQ_CTRL_LAST)
				break;
		}
		if (len + i == fl_rls->fl_len) {
			pr_err("not found last sequence rule\n");
			return -EINVAL;
		}
		/* Reach the end of the flow rules - out */
		if (len + i + 1 == fl_rls->fl_len)
			break;
		/* Find the higher priority rule after the n-tuple flow rules */
		for (j = len + i + 1; j < fl_rls->fl_len; j++) {
			if (fl_rls->fl[j].prio < fl_rls->fl[i].prio)
				break;
		}
		/* Skip the entire n-tuple flow if not found the higher priority rule */
		if (j == fl_rls->fl_len) {
			i += len;
			continue;
		}
		/* Reorder the rules */
		pp2_cls_fl_rls_sort(&fl_rls->fl[i], j - i + 1);
	}

	/* Reassign the rule offset */
	for (i = 0, j = 0; i < fl_rls->fl_len; i++) {
		fl_rls->fl[i].rl_off = rl_off + i;
		if (fl_rls->fl[i].engine == MVPP2_ENGINE_C3_A ||
		    fl_rls->fl[i].engine == MVPP2_ENGINE_C3_B ||
			fl_rls->fl[i].engine == MVPP2_ENGINE_C3_HA ||
			fl_rls->fl[i].engine == MVPP2_ENGINE_C3_HB) {
			if (j == 0 && eng_cnt[0].c3 == MVPP2_CLS_C3_RND_MAX)
				j = 1;
		} else if (fl_rls->fl[i].engine == MVPP2_ENGINE_C2) {
			if (j == 0 && eng_cnt[0].c2 == MVPP2_CLS_C2_RND_MAX)
				j = 1;
		}
		pp2_cls_rl_hit_cnt_upd_reorder(fl_rls, i, &eng_cnt[j]);
		if (j == 0 && RND_HIT_CNT(eng_cnt, 0) == MVPP2_CLS_FL_RND_SIZE)
			j = 1;
	}

	/* Verify hit count of each engine */
	if (eng_cnt[1].c2 > MVPP2_CLS_C2_RND_MAX ||
	    eng_cnt[1].c3 > MVPP2_CLS_C3_RND_MAX) {
		pr_err("c2 hits (%d) or c3 hits (%d) exceed maximum number\n",
		       eng_cnt[1].c2, eng_cnt[1].c3);
		return -EINVAL;
	}

	return 0;
}

/*******************************************************************************
 * pp2_cls_fl_rule_add
 *
 * DESCRIPTION: The API adds all rules according to flow rule array
 *
 * INPUTS:
 *	inst - packet processor instance
 *	fl_rls - a list of rules to enable
 *
 * OUTPUTS:
 *	fl_rls->fl[].rl_log_id - allocated rule logical ID
 *
 * RETURNS:
 *	0 on success, error-code otherwise
 ******************************************************************************/
int pp2_cls_fl_rule_add(struct pp2_inst *inst, struct pp2_cls_fl_rule_list_t *fl_rls)
{
	struct pp2_cls_fl_t *new_fl;
	struct pp2_cls_fl_t *merge_fl;
	struct pp2_cls_fl_t *cur_fl;
	struct pp2_cls_fl_rule_entry_t *fl_rl;
	struct pp2_cls_rl_entry_t *fl;
	struct pp2_db_cls_lkp_dcod_t lkp_dcod_db;
	int rc;
	u16 i, j, fl_log_id;
	bool rule_found;

	if (!fl_rls) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}

	new_fl = kmalloc(sizeof(*new_fl), GFP_KERNEL);
	if (!new_fl)
		return -ENOMEM;

	merge_fl = kmalloc(sizeof(*merge_fl), GFP_KERNEL);
	if (!merge_fl) {
		rc = -ENOMEM;
		goto err1;
	}

	cur_fl = kmalloc(sizeof(*cur_fl), GFP_KERNEL);
	if (!cur_fl) {
		rc = -ENOMEM;
		goto err2;
	}

	/* populate all current flows from DB for flow_log_id */
	for (fl_log_id = 0; fl_log_id < MVPP2_MNG_FLOW_ID_MAX; fl_log_id++) {
		memset(cur_fl, 0, sizeof(struct pp2_cls_fl_t));

		/* get current flow_log_id rules */
		rc = pp2_cls_fl_cur_get(inst, fl_log_id, cur_fl);
		if (rc != 0) {
			pr_err("pp2_db_cls_lkp_dcod_get returned error\n");
			rc = -EFAULT;
			goto err3;
		}

		memset(new_fl, 0, sizeof(struct pp2_cls_fl_t));

		new_fl->fl_log_id = fl_log_id;
		new_fl->fl_len = 0;

		/* inset all new flow rules to logical flow based table */
		for (i = 0; i < fl_rls->fl_len; i++) {
			rule_found = false;
			fl_rl = &fl_rls->fl[i];

			/* handle only the current logical flow ID */
			if (fl_log_id != fl_rl->fl_log_id)
				continue;

			/* get the lookup DB for this logical flow ID */
			rc = pp2_db_cls_lkp_dcod_get(inst, fl_log_id, &lkp_dcod_db);
			if (rc) {
				pr_err("failed to get lookup decode info for fl_log_id %d\n", fl_log_id);
				goto err3;
			}

			/* logical flow not initialized */
			if (lkp_dcod_db.flow_alloc_len == 0) {
				pr_err("fl_log_id %d new rule #%d was not initialized\n",
				       fl_log_id, i);
				rc = -EFAULT;
				goto err3;
			}

			fl = &new_fl->fl[new_fl->fl_len];

			if (!fl_rl->enabled) {
				fl_rl->port_type = MVPP2_PORT_TYPE_INV;
				fl_rl->port_bm = MVPP2_PORT_BM_INV;
			}

			/* copy the data to new entry */
			fl->enabled	= fl_rl->enabled;
			fl->engine	= fl_rl->engine;
			fl->field_id_cnt = fl_rl->field_id_cnt;
			fl->lu_type	= fl_rl->lu_type;
			fl->port_bm	= fl_rl->port_bm;
			fl->port_type	= fl_rl->port_type;
			fl->prio	= fl_rl->prio;
			fl->udf7	= fl_rl->udf7;
			MVPP2_MEMSET_ZERO(fl->ref_cnt);
			fl->rl_log_id	= MVPP2_CLS_UNDF_FL_LOG_ID;
			fl->rl_off	= lkp_dcod_db.flow_off + i;
			fl->skip	= 0;
			fl->seq_ctrl	= fl_rl->seq_ctrl;
			fl->state	= MVPP2_MRG_NEW;
			memcpy(fl->field_id,
			       fl_rl->field_id,
				sizeof(fl_rl->field_id));
			/*
			 * code snippet is disabled since currently rules addition does
			 * not include valid port_type and port_bm and there is no
			 * way to differentiate two rules with same priority
			 */

			/* search for the rule within the current flow rules */
			for (j = 0; j < cur_fl->fl_len; j++) {
				if (cur_fl->fl[j].engine	== fl_rl->engine	&&
				    cur_fl->fl[j].field_id_cnt	== fl_rl->field_id_cnt	&&
				    cur_fl->fl[j].lu_type	== fl_rl->lu_type	&&
				    cur_fl->fl[j].prio		== fl_rl->prio		&&
				    cur_fl->fl[j].udf7		== fl_rl->udf7		&&
				    !memcmp(cur_fl->fl[j].field_id,
					    fl_rl->field_id,
					    sizeof(fl_rl->field_id))) {
					/* identical rule found */
					rule_found = true;

					/* update the logical rule id */
					fl_rl->rl_log_id = cur_fl->fl[j].rl_log_id;

					cur_fl->fl[j].state = MVPP2_MRG_NEW_EXISTS;

					if (cur_fl->fl[j].enabled == 1)
						break;

					/* enable the found rule if it's disabled */
					fl_rl->enabled = 1;
					cur_fl->fl[j].enabled = 1;
					cur_fl->fl[j].port_type = fl_rl->port_type;
					cur_fl->fl[j].port_bm = fl_rl->port_bm;

					rc = pp2_cls_fl_rl_hw_ena(inst, fl_rl);
					if (rc) {
						pr_err("pp2_cls_fl_rl_hw_ena ret_code(%d)\n", rc);
						goto err3;
					}

					rc = pp2_cls_fl_rl_db_set(inst, &cur_fl->fl[j], cur_fl->fl_log_id);
					if (rc) {
						pr_err("pp2_cls_fl_rl_db_set ret_code(%d)\n", rc);
						goto err3;
					}
					break;
				}
			}

			/* rule already exist in the current flow */
			if (rule_found)
				continue;
			/* increment the engine count per flow log id */
			rc = pp2_cls_fl_rl_eng_cnt_upd(MVPP2_CNT_INC, fl_rl->engine,
						       &new_fl->eng_cnt);
			if (rc) {
				pr_err("recvd ret_code(%d)\n", rc);
				goto err3;
			}

			new_fl->fl_len++;
		}

		/* did we find new rules for the logical flow ID? */
		if (new_fl->fl_len == 0)
			continue;

		/* merge the current and new flow rules together */

		/* sort the new flow according to prio */
		if (new_fl->fl_len > 1)
			pp2_cls_fl_rls_sort(new_fl->fl, new_fl->fl_len);

		/* merge the two flows (new & curr) */
		rc = pp2_cls_fl_rls_merge(inst, fl_log_id, cur_fl, new_fl, merge_fl);
		if (rc) {
			pr_err("recvd ret_code(%d)\n", rc);
			goto err3;
		}

		/* reorder the n-tuple flows by priority */
		rc = pp2_cls_fl_nt_rule_reorder(merge_fl);
		if (rc) {
			pr_err("recvd ret_code(%d)\n", rc);
			goto err3;
		}

		/* set the rules in DB and HW */
		rc = pp2_cls_fl_rls_set(inst, merge_fl);
		if (rc) {
			pr_err("recvd ret_code(%d)\n", rc);
			goto err3;
		}

		/* update logical rule ID in caller structure */
		rc = pp2_cls_fl_rls_log_rl_id_upd(fl_rls, merge_fl);
		if (rc) {
			pr_err("recvd ret_code(%d)\n", rc);
			goto err3;
		}
	}

	kfree(cur_fl);
	kfree(merge_fl);
	kfree(new_fl);

	return 0;
err3:
	kfree(cur_fl);
err2:
	kfree(merge_fl);
err1:
	kfree(new_fl);

	return rc;
}

/*******************************************************************************
 * pp2_cls_fl_rule_enable
 *
 * DESCRIPTION: The API enables all rules according to flow rule array
 *
 * INPUTS:
 *	inst - packet processor instance
 *	fl_rls - a list of rules to enable
 *
 * OUTPUTS:
 *	fl_rls->fl[].rl_log_id - rule logical ID according to matching rule in DB
 *
 * RETURNS:
 *	0 on success, error-code otherwise
 ******************************************************************************/
int pp2_cls_fl_rule_enable(struct pp2_inst *inst,
			   struct pp2_cls_fl_rule_list_t *fl_rls)
{
	u16 rl_off, i;
	struct pp2_db_cls_lkp_dcod_t lkp_dcod_db;
	struct pp2_db_cls_fl_rule_list_t *fl_rl_db;	/*use heap to reduce stack size*/
	struct pp2_cls_fl_rule_entry_t *rl_en;
	struct pp2_db_cls_fl_rule_t *rl_db = NULL;
	int rc;
	int loop;
	u16 port_bm = 0;
	u16 fl_rls_port_bm;

	if (!fl_rls) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}

	fl_rl_db = kmalloc(sizeof(*fl_rl_db), GFP_KERNEL);
	if (!fl_rl_db)
		return -ENOMEM;

	/* iterate over all rule list */
	for (i = 0; i < fl_rls->fl_len; i++) {
		/* get the lookup DB for this logical flow ID */
		rc = pp2_db_cls_lkp_dcod_get(inst, fl_rls->fl[i].fl_log_id, &lkp_dcod_db);
		if (rc) {
			pr_err("failed to get lookup decode DB data for fl_log_id %d\n",
			       fl_rls->fl[i].fl_log_id);
			return rc;
		}

		/* get all rules for this logical flow ID */
		memset(fl_rl_db, 0, sizeof(struct pp2_db_cls_fl_rule_list_t));
		rc = pp2_db_cls_fl_rule_list_get(inst, lkp_dcod_db.flow_off, lkp_dcod_db.flow_len, &fl_rl_db->flow[0]);
		if (rc) {
			pr_err("failed to get flow rule list, fl_log_id=%d flow_off=%d flow_len=%d\n",
			       fl_rls->fl[i].fl_log_id, lkp_dcod_db.flow_off, lkp_dcod_db.flow_len);
			kfree(fl_rl_db);
			return rc;
		}

		/* set the flow length in the DB entry */
		fl_rl_db->flow_len = lkp_dcod_db.flow_len;

		rl_en = &fl_rls->fl[i];

		/* search for enabled rule (valid port_type and port_bm) to enable */
		for (rl_off = 0; rl_off < fl_rl_db->flow_len; rl_off++) {
			rl_db = &fl_rl_db->flow[rl_off];
			if (rl_en->engine	== rl_db->engine	&&
			    rl_en->field_id_cnt	== rl_db->field_id_cnt	&&
			    rl_en->lu_type	== rl_db->lu_type	&&
			    rl_en->port_type	== rl_db->port_type	&&
			    rl_en->prio		== rl_db->prio		&&
			    rl_en->udf7		== rl_db->udf7		&&
			    !memcmp(rl_en->field_id, rl_db->field_id,
				    rl_en->field_id_cnt * sizeof(rl_en->field_id[0]))) {
				/* for virt port, port_id does not matter */
				if (rl_en->port_type != MVPP2_SRC_PORT_TYPE_VIR) {
					int read_port_bm;

					rc = pp2_cls_fl_port_hw_read(inst, rl_db->rl_log_id, &read_port_bm);
					if (rc) {
						pr_err("recvd ret_code(%d)\n", rc);
						kfree(fl_rl_db);
						return rc;
					}
					if (rl_en->enabled)
						rl_db->port_bm = read_port_bm | rl_en->port_bm;
					else
						rl_db->port_bm = read_port_bm & (~rl_en->port_bm);
					port_bm = rl_db->port_bm;

					if (!rl_db->enabled) {
						MVPP2_MEMSET_ZERO(rl_db->ref_cnt);
						rl_db->enabled = true;
					}
					fl_rls_port_bm = rl_en->port_bm;

					rl_en->port_bm = rl_db->port_bm;
					/* Update Port BM */
					rl_en->rl_log_id = rl_db->rl_log_id;
					rc = pp2_cls_fl_rl_hw_ena(inst, rl_en);
					if (rc) {
						pr_err("recvd ret_code(%d)\n", rc);
						kfree(fl_rl_db);
						return rc;
					}
					/* restore rl_en value */
					rl_en->port_bm = fl_rls_port_bm;
					/* update the logical rule id */
					rl_en->rl_log_id = rl_db->rl_log_id;
				}
				break;
			}
		}

		/* verify that we found a rule */
		if (rl_off == fl_rl_db->flow_len) {
			pr_err("failed to find flow rule #%d to enable\n", i);
			pr_err("fl_id(%d),port_type(%d),port_bm(%d),",
			       fl_rls->fl[i].fl_log_id, fl_rls->fl[i].port_type, fl_rls->fl[i].port_bm);
			pr_err("prio(%d),lu_type(%d),engine(%d),udf7(%d),field_id_cnt(%d)",
			       fl_rls->fl[i].prio, fl_rls->fl[i].lu_type, fl_rls->fl[i].engine, fl_rls->fl[i].udf7,
			       fl_rls->fl[i].field_id_cnt);
			pr_err("field_id_0(%x), field_id_1(%x),field_id_2(%x),field_id_3(%x)\n",
			       fl_rls->fl[i].field_id[0], fl_rls->fl[i].field_id[1], fl_rls->fl[i].field_id[2],
			       fl_rls->fl[i].field_id[3]);
			kfree(fl_rl_db);
			return -EFAULT;
		}

		/* update the logical rule id */
		rl_en->rl_log_id = rl_db->rl_log_id;

		/* found the rule we searched for */
		if (!rl_db->enabled) {
			u16 fl_rls_port_bm = rl_en->port_bm;
			u16 fl_rls_log_id = rl_en->rl_log_id;

			MVPP2_MEMSET_ZERO(rl_db->ref_cnt);
			rl_db->enabled = true;
			if (rl_en->enabled)
				rl_db->port_bm |= rl_en->port_bm;
			else
				rl_db->port_bm &= ~rl_en->port_bm;
			rl_en->port_bm = rl_db->port_bm;
			rl_en->rl_log_id = rl_db->rl_log_id;
			/* rule disabled, enable the HW */
			rc = pp2_cls_fl_rl_hw_ena(inst, rl_en);
			if (rc) {
				pr_err("recvd ret_code(%d)\n", rc);
				kfree(fl_rl_db);
				return rc;
			}
			/* restore rl_en value */
			rl_en->port_bm = fl_rls_port_bm;
			rl_en->rl_log_id = fl_rls_log_id;
		}

		/* increment the reference counter */
		for (loop = 0; loop < PP2_NUM_PORTS; loop++) {
			if (1 << loop & port_bm)
				rl_db->ref_cnt[loop]++;
		}

		pr_debug("enable: fl_log_id[%d] rl_log_id[%d] rl_off[%d] port_type[%d] port_bm[%d]",
			 fl_rls->fl[i].fl_log_id, rl_en->rl_log_id, rl_off,
			 fl_rls->fl[i].port_type, fl_rls->fl[i].port_bm);
		pr_debug("prio[%d] lu_type[%d] engine[%d] udf7[%d] field_id_cnt[%d]\n",
			 fl_rls->fl[i].prio, fl_rls->fl[i].lu_type, fl_rls->fl[i].engine, fl_rls->fl[i].udf7,
			 fl_rls->fl[i].field_id_cnt);

		/* update the DB */
		rc = pp2_db_cls_fl_rule_set(inst, lkp_dcod_db.flow_off + rl_off, rl_db);
		if (rc) {
			pr_err("recvd ret_code(%d)\n", rc);
			kfree(fl_rl_db);
			return rc;
		}
	}

	kfree(fl_rl_db);
	return 0;
}

/*******************************************************************************
 * pp2_cls_fl_hash_rule_set
 *
 * DESCRIPTION: The API enables hash rules according to flow rule array
 *
 * INPUTS:
 *	inst - packet processor instance
 *	fl_rls - a list of rules to enable
 *
 * OUTPUTS:
 *	fl_rls->fl[].rl_log_id - rule logical ID according to matching rule in DB
 *
 * RETURNS:
 *	0 on success, error-code otherwise
 ******************************************************************************/
int pp2_cls_fl_hash_rule_set(struct pp2_inst *inst, struct pp2_port *port, int lkpid)
{
	int rl_off;
	int engine, is_last, field_cnt, fid, lkpid_attr;
	int field[MVPP2_CLS_FLOWS_TBL_FIELDS_MAX];
	struct mv_pp2x_cls_flow_entry fe;
	int rc;
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);
	struct pp2_db_cls_lkp_dcod_t lkp_dcod_db;
	struct pp2_db_cls_fl_rule_list_t *fl_rl_db;	/*use heap to reduce stack size*/
	struct pp2_db_cls_fl_rule_t *rl_db = NULL;

	fl_rl_db = kmalloc(sizeof(*fl_rl_db), GFP_KERNEL);
	if (!fl_rl_db)
		return -ENOMEM;

	/* get the lookup DB for this logical flow ID */
	rc = pp2_db_cls_lkp_dcod_get(inst, lkpid, &lkp_dcod_db);
	if (rc) {
		pr_err("failed to get lookup decode DB data for fl_log_id %d\n", lkpid);
		return rc;
	}

	/* get all rules for this logical flow ID */
	memset(fl_rl_db, 0, sizeof(struct pp2_db_cls_fl_rule_list_t));
	rc = pp2_db_cls_fl_rule_list_get(inst, lkp_dcod_db.flow_off, lkp_dcod_db.flow_len, &fl_rl_db->flow[0]);
	if (rc) {
		pr_err("failed to get flow rule list, flow_off=%d flow_len=%d\n",
		       lkp_dcod_db.flow_off, lkp_dcod_db.flow_len);
		kfree(fl_rl_db);
		return rc;
	}

	/* set the flow length in the DB entry */
	fl_rl_db->flow_len = lkp_dcod_db.flow_len;

	/* search for enabled rule (valid port_type and port_bm) to enable */
	for (rl_off = 0; rl_off < fl_rl_db->flow_len; rl_off++) {
		rl_db = &fl_rl_db->flow[rl_off];

		if (((rl_db->engine == MVPP2_CLS_ENGINE_C3HA) ||
		    (rl_db->engine == MVPP2_CLS_ENGINE_C3HB)) &&
		    rl_db->port_bm & BIT(port->id))
			break;
	}

	if (rl_off == fl_rl_db->flow_len) {
		pr_err("failed to find flow rule\n");
		kfree(fl_rl_db);
		return -EFAULT;
	}

	/* read the rule from HW*/
	rc = mv_pp2x_cls_hw_flow_read(cpu_slot, rl_off + 1, &fe);
	if (rc) {
		pr_err("%s(%d): recvd ret_code(%d)\n", __func__, __LINE__, rc);
		return rc;
	}

	/* Get engine and is last*/
	rc = mv_pp2x_cls_sw_flow_engine_get(&fe, &engine, &is_last);
	if (rc) {
		pr_err("mv_pp2x_cls_sw_flow_engine_get fail rc = %d\n", rc);
		return rc;
	}

	if (engine == MVPP2_CLS_ENGINE_C3HA)
		field_cnt = 2;
	else
		field_cnt = 4;

	lkpid_attr = mv_pp2x_prs_flow_id_attr_get(lkpid);

	if (lkpid_attr & MVPP2_PRS_FL_ATTR_IP4_BIT) {
		field[0] = MVPP2_CLS_FIELD_IP4SA;
		field[1] = MVPP2_CLS_FIELD_IP4DA;
	} else if (lkpid_attr & MVPP2_PRS_FL_ATTR_IP6_BIT) {
		field[0] = MVPP2_CLS_FIELD_IP6SA;
		field[1] = MVPP2_CLS_FIELD_IP6DA;
	}
	field[2] = MVPP2_CLS_FIELD_L4SIP;
	field[3] = MVPP2_CLS_FIELD_L4DIP;

	/* update hek number field */
	rc = mv_pp2x_cls_sw_flow_hek_num_set(&fe, field_cnt);
	if (rc) {
		pr_err("mv_pp2x_cls_sw_flow_hek_num_set fail rc = %d\n", rc);
		return rc;
	}

	/* update hek */
	for (fid = 0; fid < field_cnt; fid++) {
		rc = mv_pp2x_cls_sw_flow_hek_set(&fe, fid, field[fid]);
		if (rc) {
			pr_err("mv_pp2x_cls_sw_flow_hek_set ret_code(%d)\n", rc);
			return rc;
		}
	}

	fe.index = rl_off + 1;
	rc = mv_pp2x_cls_hw_flow_write(cpu_slot, &fe);
	if (rc) {
		pr_err("mv_pp2x_cls_hw_flow_write ret_code(%d)\n", rc);
		return rc;
	}

	/* update DB */
	rl_db->field_id_cnt = field_cnt;
	for (fid = 0; fid < field_cnt; fid++)
		rl_db->field_id[fid] = field[fid];

	rc = pp2_db_cls_fl_rule_set(inst, lkp_dcod_db.flow_off + rl_off, rl_db);
	if (rc) {
		pr_err("pp2_db_cls_fl_rule_set recvd ret_code(%d)\n", rc);
		return rc;
	}

	/* read the rule from HW*/
	rc = mv_pp2x_cls_hw_flow_read(cpu_slot, rl_off + 1, &fe);
	if (rc) {
		pr_err("%s(%d): recvd ret_code(%d)\n", __func__, __LINE__, rc);
		return rc;
	}

	rc = mv_pp2x_cls_sw_flow_hek_get(&fe, &field_cnt, field);
	if (rc) {
		pr_err("mv_pp2x_cls_sw_flow_hek_get fail rc = %d\n", rc);
		return rc;
	}

	/* update the DB */
	rc = pp2_db_cls_fl_rule_set(inst, lkp_dcod_db.flow_off + rl_off, rl_db);
	if (rc) {
		pr_err("recvd ret_code(%d)\n", rc);
		return rc;
	}

	pr_debug("%s, after updating HEK\n", __func__);
	pr_debug("  ptype | bm | prio | lutype | eng | udf7 | cnt | 1 | 2 | 3 | 4\n");
	pr_debug("  %6d|%4x|%6d|%8d|%5d|%6d|%5d|%3x|%3x|%3x|%3x\n",
		rl_db->port_type,
		rl_db->port_bm,
		rl_db->prio,
		rl_db->lu_type,
		rl_db->engine,
		rl_db->udf7,
		rl_db->field_id_cnt,
		rl_db->field_id[0],
		rl_db->field_id[1],
		rl_db->field_id[2],
		rl_db->field_id[3]);

	return 0;
}

/*******************************************************************************
 * pp2_cls_fl_rule_disable
 *
 * DESCRIPTION: The API disables all rules according to logical rule ID array
 *
 * INPUTS:
 *	inst - packet processor instance
 *	rl_log_id - a list of the logical rule id
 *	rl_log_id_len - rl_log_id length
 *
 * OUTPUTS:
 *	None
 *
 * RETURNS:
 *	0 on success, error-code otherwise
 ******************************************************************************/
int pp2_cls_fl_rule_disable(struct pp2_inst *inst, u16 *rl_log_id,
			    u16 rl_log_id_len,
			    u32 port_id)
{
	u16 rl_off, i;
	struct pp2_db_cls_fl_rule_t rl_db;
	int rc;
	u16 ref_sum = 0;
	int loop;
	struct pp2_cls_fl_rule_entry_t rl_en;
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);

	if (!rl_log_id) {
		pr_err("%s: rl_log_id is null pointer\n", __func__);
		return -EFAULT;
	}

	MVPP2_MEMSET_ZERO(rl_en);

	/* iterate all logical rule IDs */
	for (i = 0; i < rl_log_id_len; i++) {
		/* get the offset for the rl_log_id */
		rc = pp2_db_cls_rl_off_get(inst, &rl_off, rl_log_id[i]);
		if (rc) {
			pr_err("recvd ret_code(%d)\n", rc);
			return rc;
		}

		/* get the rule DB entry for the offset */
		rc = pp2_db_cls_fl_rule_get(inst, rl_off, &rl_db);
		if (rc) {
			pr_err("recvd ret_code(%d)\n", rc);
			return rc;
		}

		/* rule already disabled, skip */
		if (!rl_db.enabled)
			pr_warn("rl_log_id=%d already disabled\n", rl_log_id[i]);

		/* last reference count, need to disable in HW */
		ref_sum = 0;
		for (loop = 0; loop < PP2_NUM_PORTS; loop++)
			ref_sum += rl_db.ref_cnt[loop];
		if (ref_sum == 1) {
			rc = pp2_cls_fl_rl_hw_dis(cpu_slot, rl_off);
			if (rc) {
				pr_err("recvd ret_code(%d)\n", rc);
				return rc;
			}

			rl_db.enabled = false;
			rl_db.port_type = MVPP2_PORT_TYPE_INV;
			rl_db.port_bm = MVPP2_PORT_BM_INV;
		}

		if (ref_sum > 1 && rl_db.ref_cnt[port_id] == 1) {
			rl_db.port_bm &= ~(1 << port_id);
			rl_en.enabled = rl_db.enabled;
			rl_en.engine = rl_db.engine;
			memcpy(rl_en.field_id, rl_db.field_id, MVPP2_FLOW_FIELD_COUNT_MAX * sizeof(u8));
			rl_en.field_id_cnt = rl_db.field_id_cnt;
			rl_en.lu_type = rl_db.lu_type;
			rl_en.port_bm = rl_db.port_bm;
			rl_en.port_type = rl_db.port_type;
			rl_en.prio = rl_db.prio;
			rl_en.udf7 = rl_db.udf7;
			rl_en.rl_log_id = rl_db.rl_log_id;
			rc = pp2_cls_fl_rl_hw_ena(inst, &rl_en);
			if (rc) {
				pr_err("recvd ret_code(%d)\n", rc);
				return rc;
			}
		}
		rl_db.ref_cnt[port_id]--;

		pr_debug("disable: rl_off[%d] rl_log_id[%d] port_type[%d] port_bm[%d] prio[%d]",
			 rl_off, rl_db.rl_log_id, rl_db.port_type, rl_db.port_bm, rl_db.prio);
		pr_debug("lu_type[%d] engine[%d] udf7[%d] field_id_cnt[%d]\n",
			 rl_db.lu_type, rl_db.engine, rl_db.udf7, rl_db.field_id_cnt);

		/* update rule entry in DB */
		rc = pp2_db_cls_fl_rule_set(inst, rl_off, &rl_db);
		if (rc) {
			pr_err("recvd ret_code(%d)\n", rc);
			return rc;
		}
	}
	return 0;
}

/*******************************************************************************
 * pp2_cls_rule_disable
 *
 * DESCRIPTION: The function disables all flows for specific lookup type and lookup fields
 *
 * INPUTS:
 *	port	- packet port
 *	fl	- flow rule entry including the lookup type and lookup fields to match
 *
 * OUTPUTS:
 *	None
 *
 * RETURNS:
 *	0 on success, error-code otherwise
 ******************************************************************************/
int pp2_cls_rule_disable(struct pp2_port *port, struct pp2_cls_fl_rule_entry_t *fl)
{
	int index;
	int rc, i;
	struct pp2_inst *inst = port->parent;
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);
	struct pp2_db_cls_fl_rule_t rl_db;
	u16 ref_sum = 0;
	int loop;
	struct pp2_cls_fl_rule_entry_t rl_en;

	for (index = 0; index < MVPP2_CLS_FLOWS_TBL_SIZE; index++) {
		/* get the rule DB entry for the offset */
		rc = pp2_db_cls_fl_rule_get(inst, index, &rl_db);
		if (rc) {
			pr_err("recvd ret_code(%d)\n", rc);
			return rc;
		}

		if (rl_db.lu_type != fl->lu_type)
			continue;

		if (rl_db.field_id_cnt !=  fl->field_id_cnt)
			continue;

		for (i = 0; i < rl_db.field_id_cnt; i++) {
			if (rl_db.field_id[i] != fl->field_id[i])
				continue;
		}

		pr_debug("index %d, type %d, count %d, fields %x %x %x %x\n", index,
			rl_db.lu_type,
			rl_db.field_id_cnt,
			rl_db.field_id[0],
			rl_db.field_id[1],
			rl_db.field_id[2],
			rl_db.field_id[3]);

		/* rule already disabled, skip */
		if (!rl_db.enabled) {
			pr_debug("index=%d already disabled\n", index);
			continue;
		}

		/* last reference count, need to disable in HW */
		ref_sum = 0;
		for (loop = 0; loop < PP2_NUM_PORTS; loop++)
			ref_sum += rl_db.ref_cnt[loop];
		if (ref_sum == 1) {
			rc = pp2_cls_fl_rl_hw_dis(cpu_slot, index);
			if (rc) {
				pr_err("recvd ret_code(%d)\n", rc);
				return rc;
			}

			rl_db.enabled = false;
			rl_db.port_type = MVPP2_PORT_TYPE_INV;
			rl_db.port_bm = MVPP2_PORT_BM_INV;
		}

		if (ref_sum > 1 && rl_db.ref_cnt[port->id] == 1) {
			rl_db.port_bm &= ~(1 << port->id);
			rl_en.enabled = rl_db.enabled;
			rl_en.engine = rl_db.engine;
			memcpy(rl_en.field_id, rl_db.field_id, MVPP2_FLOW_FIELD_COUNT_MAX * sizeof(u8));
			rl_en.field_id_cnt = rl_db.field_id_cnt;
			rl_en.lu_type = rl_db.lu_type;
			rl_en.port_bm = rl_db.port_bm;
			rl_en.port_type = rl_db.port_type;
			rl_en.prio = rl_db.prio;
			rl_en.udf7 = rl_db.udf7;
			rl_en.rl_log_id = rl_db.rl_log_id;
			rc = pp2_cls_fl_rl_hw_ena(inst, &rl_en);
			if (rc) {
				pr_err("recvd ret_code(%d)\n", rc);
				return rc;
			}
		}
		rl_db.ref_cnt[port->id]--;

		pr_debug("disable: rl_off[%d] rl_log_id[%d] port_type[%x] port_bm[%x] prio[%d]",
			 index, rl_db.rl_log_id, rl_db.port_type, rl_db.port_bm, rl_db.prio);
		pr_debug("lu_type[%d] engine[%d] udf7[%d] field_id_cnt[%d]\n",
			 rl_db.lu_type, rl_db.engine, rl_db.udf7, rl_db.field_id_cnt);

		/* update rule entry in DB */
		rc = pp2_db_cls_fl_rule_set(inst, index, &rl_db);
		if (rc) {
			pr_err("recvd ret_code(%d)\n", rc);
			return rc;
		}
	}
	return 0;
}

/*******************************************************************************
 * pp2_cls_set_hash_params
 *
 * DESCRIPTION: The function set fl_rls variables according to the inputs
 *
 * INPUTS:
 *	fl_rls - packet processor instance
 *	port - packet port
 *	engine - engine type
 *	lkpid - lookup id number
 *	lkpid_attr - parser attribute
 *	set - enable or disable
 *
 * OUTPUTS:
 *	fl_rls - modify fl_rls according to the inputs
 ******************************************************************************/
static void pp2_cls_set_hash_params(struct pp2_cls_fl_rule_list_t *fl_rls, struct pp2_port *port,
				    int engine, int lkpid, int lkpid_attr, int set)
{
	int lkp_type = (port->type == PP2_PPIO_T_LOG) ? MVPP2_CLS_LKP_MUSDK_LOG_HASH : MVPP2_CLS_LKP_HASH;

	fl_rls->fl_len = 1;
	fl_rls->fl->enabled = set;
	fl_rls->fl->fl_log_id = lkpid;
	fl_rls->fl->port_type = MVPP2_SRC_PORT_TYPE_PHY;
	fl_rls->fl->port_bm = (1 << port->id);
	fl_rls->fl->lu_type = lkp_type;
	fl_rls->fl->prio = pp2_cls_mng_lkp_type_to_prio(lkp_type);
	fl_rls->fl->engine = engine;
	fl_rls->fl->udf7 = (port->type == PP2_PPIO_T_LOG) ? MVPP2_CLS_MUSDK_LOG_UDF7 : MVPP2_CLS_MUSDK_NIC_UDF7;
	fl_rls->fl->seq_ctrl = MVPP2_CLS_DEF_SEQ_CTRL;

	if (engine == MVPP2_CLS_ENGINE_C3HA)
		fl_rls->fl->field_id_cnt = 2;
	else
		fl_rls->fl->field_id_cnt = 4;

	if (lkpid_attr & MVPP2_PRS_FL_ATTR_IP4_BIT) {
		fl_rls->fl->field_id[0] = MVPP2_CLS_FIELD_IP4SA;
		fl_rls->fl->field_id[1] = MVPP2_CLS_FIELD_IP4DA;
	} else if (lkpid_attr & MVPP2_PRS_FL_ATTR_IP6_BIT) {
		fl_rls->fl->field_id[0] = MVPP2_CLS_FIELD_IP6SA;
		fl_rls->fl->field_id[1] = MVPP2_CLS_FIELD_IP6DA;
	}
	fl_rls->fl->field_id[2] = MVPP2_CLS_FIELD_L4SIP;
	fl_rls->fl->field_id[3] = MVPP2_CLS_FIELD_L4DIP;
}

/*******************************************************************************
 * pp2_cls_rss_mode_flows_set
 *
 * DESCRIPTION: update flows bm according to rss mode.
 *
 * INPUTS:
 *	port - packet port
 *	rss_mode - for example 2/5 tuple
 *
 * RETURNS:
 *	0 on success, error-code otherwise
 ******************************************************************************/
int pp2_cls_rss_mode_flows_set(struct pp2_port *port, int rss_mode)
{
	int lkpid, lkpid_attr;
	int rc;
	struct pp2_inst *inst = port->parent;
	struct pp2_cls_fl_rule_list_t *fl_rls_hash;
	enum musdk_lnx_id lnx_id = lnx_id_get();

	if (rss_mode == PP2_PPIO_HASH_T_NONE)
		return 0;

	fl_rls_hash = kmalloc((sizeof(*fl_rls_hash)), GFP_KERNEL);
	if (!fl_rls_hash)
		return -ENOMEM;

	int lkp_type = (port->type == PP2_PPIO_T_LOG) ? MVPP2_CLS_LKP_MUSDK_LOG_HASH : MVPP2_CLS_LKP_HASH;

	for (lkpid = MVPP2_PRS_FL_START; lkpid < MVPP2_PRS_FL_LAST; lkpid++) {
		/* Get lookup id attribute */
		lkpid_attr = mv_pp2x_prs_flow_id_attr_get(lkpid);
		if ((lkpid_attr & (MVPP2_PRS_FL_ATTR_TCP_BIT | MVPP2_PRS_FL_ATTR_UDP_BIT)) &&
		    !(lkpid_attr & MVPP2_PRS_FL_ATTR_FRAG_BIT)) {
			if (rss_mode == PP2_PPIO_HASH_T_2_TUPLE) {
				/* For backwards compatibility to LK 4.4 */
				if (lnx_is_mainline(lnx_id) &&
				    (lkp_type == MVPP2_CLS_LKP_HASH)) {
					pp2_cls_fl_hash_rule_set(inst, port, lkpid);
				} else {
					pp2_cls_set_hash_params(fl_rls_hash, port, MVPP2_CLS_ENGINE_C3HA,
								lkpid, lkpid_attr, true);
					rc = pp2_cls_fl_rule_enable(inst, fl_rls_hash);
					pp2_cls_set_hash_params(fl_rls_hash, port, MVPP2_CLS_ENGINE_C3HB, lkpid,
								lkpid_attr, false);
					rc |= pp2_cls_fl_rule_enable(inst, fl_rls_hash);
				}
			} else if (rss_mode == PP2_PPIO_HASH_T_5_TUPLE) {
				/* For backwards compatibility to LK 4.4 */
				if (lnx_is_mainline(lnx_id) &&
				    (lkp_type == MVPP2_CLS_LKP_HASH)) {
					pp2_cls_fl_hash_rule_set(inst, port, lkpid);
				} else {
					pp2_cls_set_hash_params(fl_rls_hash, port, MVPP2_CLS_ENGINE_C3HA, lkpid,
								lkpid_attr, false);
					rc = pp2_cls_fl_rule_enable(inst, fl_rls_hash);
					pp2_cls_set_hash_params(fl_rls_hash, port, MVPP2_CLS_ENGINE_C3HB, lkpid,
								lkpid_attr, true);
					rc |= pp2_cls_fl_rule_enable(inst, fl_rls_hash);
				}
			} else {
				pr_err("%s(%d), unknown rss mode\n", __func__, __LINE__);
				kfree(fl_rls_hash);
				return -EINVAL;
			}
		} else if (lkpid_attr & (MVPP2_PRS_FL_ATTR_IP4_BIT | MVPP2_PRS_FL_ATTR_IP6_BIT)) {
			/* For backwards compatibility to LK 4.4 */
			if (lnx_is_mainline(lnx_id) &&
			    (lkp_type == MVPP2_CLS_LKP_HASH)) {
				pp2_cls_fl_hash_rule_set(inst, port, lkpid);
			} else {
				pp2_cls_set_hash_params(fl_rls_hash, port, MVPP2_CLS_ENGINE_C3HA,
							lkpid, lkpid_attr, true);
				rc = pp2_cls_fl_rule_enable(inst, fl_rls_hash);
			}
		}
	}

	kfree(fl_rls_hash);
	return 0;
}

/*******************************************************************************
 * pp2_cls_find_flows_for_lkp
 *
 * DESCRIPTION: searching for HW flows for lookup id  and adding them to flow list
 *
 * INPUTS:
 *	fl_rls - pointer to list of the flow rule
 *	flow_log_id - the logical flow ID to perform the operation
 *	flow_index - the index in the HW to looking for the flows
 *
 * OUTPUTS:
 *	fl_rls - adding new flows to fl_rls
 *
 * RETURNS:
 *	0 on success, error-code otherwise
 ******************************************************************************/
static int pp2_cls_find_flows_per_lkp(uintptr_t cpu_slot,
				      struct pp2_cls_fl_rule_list_t *fl_rls,
				      int flow_log_id, int flow_index)
{
	int rc;

	struct mv_pp2x_cls_flow_entry fe;
	int engine, is_last, num_of_fields, port_type, port_id, lkp_type, prio, seq_ctrl, tmp;
	int fields_arr[MVPP2_CLS_FLOWS_TBL_FIELDS_MAX];

	for (; flow_index < MVPP2_CLS_FLOWS_TBL_SIZE; flow_index++) {
		rc = mv_pp2x_cls_hw_flow_read(cpu_slot, flow_index, &fe);
		if (rc) {
			pr_err("mv_pp2x_cls_hw_flow_read fail rc = %d\n", rc);
			return rc;
		}

		rc = mv_pp2x_cls_sw_flow_engine_get(&fe, &engine, &is_last);
		if (rc) {
			pr_err("mv_pp2x_cls_sw_flow_engine_get fail rc = %d\n", rc);
			return rc;
		}

		if (!engine) {
			pr_err("didn't find any flows\n");
			break;
		}

		rc = mv_pp2x_cls_sw_flow_extra_get(&fe, &lkp_type, &tmp);
		if (rc) {
			pr_err("mv_pp2x_cls_sw_flow_extra_get fail rc = %d\n", rc);
			return rc;
		}

		/* add only kernel flows to db & hw */
		if (lkp_type > MVPP2_CLS_LKP_DEFAULT) {
			if (is_last) {
				pr_debug("found %d flows\n", fl_rls->fl_len);
				break;
			}
			continue;
		}
		prio = pp2_cls_mng_lkp_type_to_prio(lkp_type);
		if (prio < 0)
			return -EINVAL;

		rc = mv_pp2x_cls_sw_flow_port_get(&fe, &port_type, &port_id);
		if (rc) {
			pr_err("mv_pp2x_cls_sw_flow_port_get fail rc = %d\n", rc);
			return rc;
		}

		rc = mv_pp2x_cls_sw_flow_seq_ctrl_get(&fe, &seq_ctrl);
		if (rc) {
			pr_err("mv_pp2x_cls_sw_flow_seq_ctrl_get fail rc = %d\n", rc);
			return rc;
		}

		rc = mv_pp2x_cls_sw_flow_hek_get(&fe, &num_of_fields, fields_arr);
		if (rc) {
			pr_err("mv_pp2x_cls_sw_flow_hek_get fail rc = %d\n", rc);
			return rc;
		}

		fl_rls->fl[fl_rls->fl_len].fl_log_id = flow_log_id;
		fl_rls->fl[fl_rls->fl_len].engine = engine;
		fl_rls->fl[fl_rls->fl_len].port_type = port_type;
		fl_rls->fl[fl_rls->fl_len].port_bm = port_id;
		fl_rls->fl[fl_rls->fl_len].lu_type = lkp_type;
		fl_rls->fl[fl_rls->fl_len].enabled = true;
		fl_rls->fl[fl_rls->fl_len].prio = prio;
		fl_rls->fl[fl_rls->fl_len].udf7 = MVPP2_CLS_KERNEL_UDF7;
		fl_rls->fl[fl_rls->fl_len].seq_ctrl = seq_ctrl;
		fl_rls->fl[fl_rls->fl_len].field_id_cnt = (u8)num_of_fields;
		fl_rls->fl[fl_rls->fl_len].field_id[0] = (u8)fields_arr[0];
		fl_rls->fl[fl_rls->fl_len].field_id[1] = (u8)fields_arr[1];
		fl_rls->fl[fl_rls->fl_len].field_id[2] = (u8)fields_arr[2];
		fl_rls->fl[fl_rls->fl_len].field_id[3] = (u8)fields_arr[3];
		fl_rls->fl_len++;

		if (fl_rls->fl_len >= MVPP2_CLS_FLOW_RULE_MAX) {
			pr_err("too many flow found, fl_len = %d\n", fl_rls->fl_len);
			return -EFAULT;
		}

		if (is_last) {
			pr_debug("found %d flows\n", fl_rls->fl_len);
			break;
		}
	}

	return 0;
}

/*******************************************************************************
 * pp2_cls_add_lkpid_flow_to_db
 *
 * DESCRIPTION: find lkp which configured by kernel and add them to DB, find for each lkp its flow
 *	and returning them to fl_rls.
 *
 *	by calling pp2_cls_lkp_dcod_set setting decoder DB.
 *	returning all flows to fl_rls - we are returning them because calling to pp2_cls_fl_rule_add
 *	is setting DB & HW and we dont want do it before cleaning the HW
 *
 * INPUTS:
 *	inst - packet processor instance
 *
 * OUTPUTS:
 *           fl_rls - all flows found
 *
 * RETURN:
 *	0 on success, error-code otherwise
 ******************************************************************************/

static int pp2_cls_add_lkpid_and_flows_to_db(struct pp2_inst *inst,
					     struct pp2_cls_fl_rule_list_t *fl_rls)
{
	struct pp2_cls_lkp_dcod_entry_t  *dcod_entry;
	struct mv_pp2x_cls_lookup_entry le;
	int lkp_index, rxq, en, flow_index, mod;
	int rc = 0;
	int way = 0; /* currently, always setting way to '0' */
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);

	dcod_entry = kmalloc(sizeof(*dcod_entry), GFP_KERNEL);
	if (!dcod_entry)
		return -ENOMEM;

	pr_debug("\n");
	pr_debug("ID :	RXQ	EN	FLOW	MODE_BASE\n");
	for (lkp_index = 0; lkp_index < MVPP2_CLS_LKP_TBL_SIZE; lkp_index++) {
		rc = mv_pp2x_cls_hw_lkp_read(cpu_slot, lkp_index, way, &le);
		if (rc)
			goto end;
		rc = mv_pp2x_cls_sw_lkp_rxq_get(&le, &rxq);
		if (rc)
			goto end;
		rc = mv_pp2x_cls_sw_lkp_en_get(&le, &en);
		if (rc)
			goto end;
		rc = mv_pp2x_cls_sw_lkp_flow_get(&le, &flow_index);
		if (rc)
			goto end;
		rc = mv_pp2x_cls_sw_lkp_mod_get(&le, &mod);
		if (rc)
			goto end;
		if (en) {
			pr_debug(" 0x%2.2x\t 0x%2.2x\t %1.1d\t 0x%3.3x\t 0x%2.2x\n",
				 le.lkpid, rxq, en, flow_index, mod);
			memset(dcod_entry, 0, sizeof(struct pp2_cls_lkp_dcod_entry_t));
			dcod_entry->cpu_q = rxq;
			dcod_entry->way = way;
			dcod_entry->flow_len = MVPP2_CLS_DEF_FLOW_LEN;
			dcod_entry->flow_log_id = le.lkpid;
			dcod_entry->luid_num = 1;
			dcod_entry->luid_list[0].luid = le.lkpid;

			rc = pp2_cls_find_flows_per_lkp(cpu_slot, fl_rls, dcod_entry->flow_log_id, flow_index);
			if (rc)
				goto end;
			pp2_cls_lkp_dcod_set(inst, dcod_entry);
		}
	}

end:
	kfree(dcod_entry);

	return rc;
}

/*******************************************************************************
 * pp2_cls_udf_field_add
 *
 * DESCRIPTION: Add UDF to CLS (configures CLS_UDF register)
 *
 * INPUTS:
 *	inst	- packet processor instance
 *	udf	- udf number
 *	offset	- offset from the udf base offset passed by Parser in bytes
 *	size	- field size in bytes
 *
 * RETURN:
 *	0 on success, error-code otherwise
 ******************************************************************************/
int pp2_cls_udf_field_add(struct pp2_inst *inst, u8 udf_num, u8 offset, u8 size)
{
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);
	u32 reg, val;
	u32 size_bits, offset_bits;

	/* relative offset and field size resolution is in bits */
	size_bits = size * 8;
	offset_bits = offset * 8;

	/* Parameters check */
	if (mv_pp2x_range_validate(offset_bits, 0, MVPP2_CLS_UDF_REL_OFFSET_MAX))
		return -EINVAL;

	if (mv_pp2x_range_validate(size_bits, 1, MVPP2_CLS_UDF_SIZE_MAX))
		return -EINVAL;

	/* Check for available UDF numbers */
	if (!(udf_num == MVPP2_CLS_UDF_OFFSET_3 || udf_num == MVPP2_CLS_UDF_OFFSET_5 ||
	      udf_num == MVPP2_CLS_UDF_OFFSET_6))
		return -EINVAL;

	reg = MVPP2_CLS_UDF_REG(udf_num);
	val = ((udf_num << MVPP2_CLS_UDF_OFFSET_ID_OFFS) & MVPP2_CLS_UDF_OFFSET_ID_MASK) |
	      ((offset_bits << MVPP2_CLS_UDF_REL_OFFSET_OFFS) & MVPP2_CLS_UDF_REL_OFFSET_MASK) |
	      ((size_bits << MVPP2_CLS_UDF_SIZE_OFFS) & MVPP2_CLS_UDF_SIZE_MASK);

	pp2_reg_write(cpu_slot, reg, val);

	return 0;
}

/*******************************************************************************
 * pp2_cls_udf_field_remove
 *
 * DESCRIPTION: Remove UDF from CLS
 *
 * INPUTS:
 *	inst	- packet processor instance
 *	udf	- udf number
 *
 * RETURN:
 *	0 on success, error-code otherwise
 ******************************************************************************/
int pp2_cls_udf_field_remove(struct pp2_inst *inst, u8 udf_num)
{
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);
	u32 reg, val;

	/* Check for available UDF numbers */
	if (!(udf_num == MVPP2_CLS_UDF_OFFSET_3 || udf_num == MVPP2_CLS_UDF_OFFSET_5 ||
	      udf_num == MVPP2_CLS_UDF_OFFSET_6))
		return -EINVAL;

	/* CLS_UDF registers numbering starts from 0. Keep 1:1 PRS UDF to CLS_UDF match */
	reg = MVPP2_CLS_UDF_REG(udf_num);
	val = ((MVPP2_CLS_UDF_OFFSET_DISABLE << MVPP2_CLS_UDF_OFFSET_ID_OFFS) & MVPP2_CLS_UDF_OFFSET_ID_MASK) |
	      ((0 << MVPP2_CLS_UDF_REL_OFFSET_OFFS) & MVPP2_CLS_UDF_REL_OFFSET_MASK) |
	      ((MVPP2_CLS_UDF_SIZE_MIN << MVPP2_CLS_UDF_SIZE_OFFS) & MVPP2_CLS_UDF_SIZE_MASK);

	pp2_reg_write(cpu_slot, reg, val);

	return 0;
}

/*******************************************************************************
 * pp2_cls_init
 *
 * DESCRIPTION: The API will clean all the HW entries in CLS flow and lookup decode tables
 *              and also initialize the CLS DB to default
 *
 * INPUTS:
 *	inst - packet processor instance
 *
 * OUTPUTS:
 *           None
 *
 * RETURN:
 *	0 on success, error-code otherwise
 ******************************************************************************/
int pp2_cls_init(struct pp2_inst *inst)
{
	int rc = 0;
	struct pp2_cls_fl_rule_list_t *fl_rls;
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);

	rc = mv_pp2x_cls_hw_cls_enable(cpu_slot, true);
	if (rc) {
		pr_err("failed to enable clasifier\n");
		return rc;
	}

	pp2_db_cls_init(inst);

	fl_rls = kmalloc(sizeof(*fl_rls), GFP_KERNEL);
	if (!fl_rls)
		return -ENOMEM;
	memset(fl_rls, 0, sizeof(struct pp2_cls_fl_rule_list_t));

	rc = pp2_cls_add_lkpid_and_flows_to_db(inst, fl_rls);
	if (rc) {
		pr_err("pp2_cls_adding_db_current_flows fail rc = %d\n", rc);
		goto end;
	}

	//rc = mv_pp2x_cls_hw_lkp_clear_all(cpu_slot);
	//if (rc) {
	//	pr_err("mv_pp2x_cls_hw_lkp_clear_all fail rc = %d\n", rc);
	//	goto end;
	//}

	//rc = mv_pp2x_cls_hw_flow_clear_all(cpu_slot);
	//if (rc) {
	//	pr_err("mv_pp2x_cls_hw_flow_clear_all fail rc = %d\n", rc);
	//	goto end;
	//}

	/* add rules and set HW */
	if (fl_rls->fl_len)
		pp2_cls_fl_rule_add(inst, fl_rls);

	/* Enable lookup decoder */
	pp2_cls_lkp_dcod_enable_all(inst);

end:
	kfree(fl_rls);

	return rc;
}
