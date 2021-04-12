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

#include "../pp2.h"
#include "pp2_hw_cls.h"
#include "pp2_rss.h"

#include "pp2_cls_types.h"
#include "pp2_cls_internal_types.h"
#include "pp2_cls_common.h"
#include "pp2_cls_utils.h"
#include "pp2_flow_rules.h"
#include "pp2_c3.h"
#include "pp2_c2.h"
#include "pp2_cls_db.h"

int pp2_rss_musdk_map_get(struct pp2_port *port)
{
	u16 req_tbls = 0, used_tbls, avail_tbls;
	int i, idx, req_ind[MVPP22_RSS_TBL_NUM] = { 0 };
	struct pp2_inst *inst = port->parent;
	int hw_tbl;
#ifdef DEBUG
	u16 num_in_q;
#endif

	used_tbls = pp2_cls_db_rss_kernel_rsvd_tbl_get(inst) + pp2_cls_db_rss_num_musdk_tbl_get(inst);
	avail_tbls = MVPP22_RSS_TBL_NUM - used_tbls;

	/* Calculate number of TC's which require RSS */
	for (i = 0; i < port->num_tcs; i++) {
		if (port->tc[i].tc_config.num_in_qs == 1)
			continue;

		hw_tbl = pp2_cls_db_rss_get_hw_tbl_from_in_q(inst, port->tc[i].tc_config.num_in_qs);
		/* New hw_tbl required for this TC */
		if (hw_tbl < 0) {
			if (req_tbls >= avail_tbls) {
				pr_err("%s:Out of RSS tables\n", __func__);
				goto rollback;
			}
			/* entry in rss_tbl_map is empty. Fill dB with new values */
			idx = pp2_cls_db_rss_tbl_map_get_next_free_idx(inst);
			if (idx == MVPP22_RSS_TBL_NUM) {
				/* This should never happen */
				pr_err("%s: Unable to allocate new RSS table\n", __func__);
				goto rollback;
			}
			pp2_cls_db_rss_tbl_map_set(inst, idx,
						   pp2_cls_db_rss_kernel_rsvd_tbl_get(inst) + idx,
						   port->tc[i].tc_config.num_in_qs);
			req_ind[req_tbls] = idx;
			req_tbls++;
#ifdef DEBUG
			pp2_cls_db_rss_tbl_map_get(inst, idx, &hw_tbl, &num_in_q);
			pr_debug("%s: rss_db_ind:%d, rss_hw_tbl_id:%d, num_in_q:%d\n",
				 __func__, idx, hw_tbl, num_in_q);
#endif
		}
	}

	pp2_cls_db_rss_num_musdk_tbl_set(inst, (used_tbls + req_tbls));

	return 0;
rollback:
	for (i = 0; i < req_tbls; i++)
		pp2_cls_db_rss_tbl_map_set(inst, req_ind[i], 0, 0);
	return -ENOSPC;

}

int pp2_rss_hw_tbl_set(struct pp2_port *port)
{
	struct mv_pp22_rss_entry rss_entry;
	int i;
	int entry_idx;
	u16 width;
	struct pp2_inst *inst = port->parent;
	int hw_tbl;

	memset(&rss_entry, 0, sizeof(struct mv_pp22_rss_entry));
	rss_entry.sel = MVPP22_RSS_ACCESS_TBL;

	for (i = 0; i < port->num_tcs; i++) {
		hw_tbl = pp2_cls_db_rss_get_hw_tbl_from_in_q(inst, port->tc[i].tc_config.num_in_qs);
		if (hw_tbl < 0) {
			pr_err("%s RSS table index not found\n", __func__);
			return -EFAULT;
		}
		rss_entry.u.entry.tbl_id = hw_tbl;

		width = mvlog2(roundup_pow_of_two(port->tc[i].tc_config.num_in_qs));
		pr_debug("setting rss table %d, width %d\n", rss_entry.u.entry.tbl_id, width);
		rss_entry.u.entry.width = width;

		for (entry_idx = 0; entry_idx < MVPP22_RSS_TBL_LINE_NUM; entry_idx++) {
			rss_entry.u.entry.tbl_line = entry_idx;
			rss_entry.u.entry.rxq = entry_idx % port->tc[i].tc_config.num_in_qs;
			if (mv_pp22_rss_tbl_entry_set(&port->parent->hw, &rss_entry))
				return -1;
		}
	}
	return 0;
}

/* The function allocate a rss table for each phisical rxq,
 * they have same cos priority
 */
int pp22_cls_rss_rxq_set(struct pp2_port *port)
{
	int i, j;
	struct mv_pp22_rss_entry rss_entry;
	struct pp2_inst *inst = port->parent;
	int hw_tbl;

	memset(&rss_entry, 0, sizeof(struct mv_pp22_rss_entry));
	rss_entry.sel = MVPP22_RSS_ACCESS_POINTER;

	for (i = 0; i < port->num_tcs; i++) {
		/* Set the table index to be used according to rss_map */
		hw_tbl = pp2_cls_db_rss_get_hw_tbl_from_in_q(inst, port->tc[i].tc_config.num_in_qs);
		if (hw_tbl < 0) {
			pr_err("%s RSS table index not found %d. Check mvpp2x_sysfs.ko module is loaded\n", __func__,
				port->tc[i].tc_config.num_in_qs);
			return -EFAULT;
		}
		rss_entry.u.pointer.rss_tbl_ptr = hw_tbl;

		for (j = 0; j < port->tc[i].tc_config.num_in_qs; j++) {
			rss_entry.u.pointer.rxq_idx = port->tc[i].tc_config.first_rxq + j;
			pr_debug("%d rxq_idx %d rss_tbl %d\n", j,
				rss_entry.u.pointer.rxq_idx,
				rss_entry.u.pointer.rss_tbl_ptr);
			if (mv_pp22_rss_tbl_entry_set(&port->parent->hw, &rss_entry))
				return -EFAULT;
		}
	}
	return 0;
}

/* mv_pp22_rss_enable_set
*  -- The API enable or disable RSS on the port
*/
int pp2_rss_enable(struct pp2_port *port, int en)
{
	int rc;

	/* For logical port, there is no need to enable C2 since the flows are not shared with kernel */
	if (port->type == PP2_PPIO_T_LOG)
		return 0;

	rc = pp2_rss_c2_enable(port, en);
	if (rc)
		return -EINVAL;

	return 0;
}

/* pp2_rss_start
*  -- Initialize the RSS parameters and dB
*/
int pp2_cls_rss_init(struct pp2_inst *inst)
{
	int rc, i;
	u16 rss_k_map;
	u16 rss_k_tbls = 0;

	rc = pp2_cls_db_rss_init(inst);
	if (rc)
		return rc;

	rss_k_map = pp2_rss_map_get();
	/* Check RSS tables can fit number of TC's configured */
	for (i = 0; i < MVPP22_RSS_TBL_NUM; i++)
		rss_k_tbls += (rss_k_map >> i) & 0x1;

	if (rss_k_tbls >= MVPP22_RSS_TBL_NUM) {
		pr_err("Kernel is using all RSS tables\n");
		return -EFAULT;
	}

	pp2_cls_db_rss_kernel_rsvd_tbl_set(inst, rss_k_tbls);

	return 0;
}

int pp2_cls_rss_hw_dump(struct pp2_inst *inst)
{
	int tbl_id, tbl_line;

	struct mv_pp22_rss_entry rss_entry;

	memset(&rss_entry, 0, sizeof(struct mv_pp22_rss_entry));

	rss_entry.sel = MVPP22_RSS_ACCESS_TBL;

	for (tbl_id = 0; tbl_id < MVPP22_RSS_TBL_NUM; tbl_id++) {
		printk("\n-------- RSS TABLE %d-----------\n", tbl_id);
		printk("HASH	QUEUE	WIDTH\n");

		for (tbl_line = 0; tbl_line < MVPP22_RSS_TBL_LINE_NUM;
			tbl_line++) {
			rss_entry.u.entry.tbl_id = tbl_id;
			rss_entry.u.entry.tbl_line = tbl_line;
			pp2_rss_tbl_entry_get(&inst->hw, &rss_entry);
			printk("0x%2.2x\t", rss_entry.u.entry.tbl_line);
			printk("0x%2.2x\t", rss_entry.u.entry.rxq);
			printk("0x%2.2x", rss_entry.u.entry.width);
			printk("\n");
		}
	}
	return 0;
}

int pp2_cls_rss_hw_rxq_tbl_dump(struct pp2_inst *inst)
{
	int rxq, port;
	struct mv_pp22_rss_entry rss_entry;

	memset(&rss_entry, 0, sizeof(struct mv_pp22_rss_entry));

	rss_entry.sel = MVPP22_RSS_ACCESS_POINTER;

	for (port = 0; port < MVPP2_MAX_PORTS; port++) {
		printk("\n-------- RXQ TABLE PORT %d-----------\n", port);
		printk("QUEUE	RSS TBL\n");

		for (rxq = 0; rxq < MVPP22_RSS_TBL_LINE_NUM; rxq++) {
			rss_entry.u.pointer.rxq_idx = port * MVPP22_RSS_TBL_LINE_NUM + rxq;
			pp2_rss_tbl_entry_get(&inst->hw, &rss_entry);
			printk("0x%2.2x\t", rss_entry.u.pointer.rxq_idx);
			printk("0x%2.2x\t", rss_entry.u.pointer.rss_tbl_ptr);
			printk("\n");

		}
	}
	return 0;
}


