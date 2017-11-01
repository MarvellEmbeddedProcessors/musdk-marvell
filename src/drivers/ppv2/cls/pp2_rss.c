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
	u16 req_tbls = 0;
	int i, idx;
	int rss_en = false;
	struct pp2_inst *inst = port->parent;
	int hw_tbl;
	u16 num_in_q;

	/* Calculate number of TC's which require RSS */
	for (i = 0; i < port->num_tcs; i++) {
		if (port->tc[i].tc_config.num_in_qs > 1)
			rss_en = true;

		hw_tbl = pp2_cls_db_rss_get_hw_tbl_from_in_q(inst, port->tc[i].tc_config.num_in_qs);
		if (hw_tbl < 0) {
			/* entry in rss_tbl_map is empty. Fill dB with new values */
			idx = pp2_cls_db_rss_tbl_map_get_next_free_idx(inst);
			if (idx == MVPP22_RSS_TBL_NUM) {
				pr_err("Unable to allocate new RSS table\n");
				return -EFAULT;
			}
			pp2_cls_db_rss_tbl_map_set(inst, idx,
				pp2_cls_db_rss_kernel_rsvd_tbl_get(inst) + idx,
				port->tc[i].tc_config.num_in_qs);
			pp2_cls_db_rss_tbl_map_get(inst, req_tbls, &hw_tbl, &num_in_q);
			pr_debug("%d, tbl %d, num_in_q %d\n", idx, hw_tbl, num_in_q);
			req_tbls++;
		}
	}

	/* If none of the TC's require RSS, then set musdk map to 0 and return */
	if (!rss_en) {
		pp2_cls_db_rss_num_musdk_tbl_set(inst, 0);
		return 0;
	}

	pr_debug("rss_map %x, req_tbls %d\n", pp2_cls_db_rss_kernel_rsvd_tbl_get(inst), req_tbls);

	/* Check number of requested RSS tables + reserved tables is less or equal maximum available HW RSS tables */
	if (req_tbls > MVPP22_RSS_TBL_NUM -  pp2_cls_db_rss_kernel_rsvd_tbl_get(inst)) {
		pr_err("Unable to allocate RSS tables for requested TC's. Available RSS tables %d, requested %d\n",
		       MVPP22_RSS_TBL_NUM -  pp2_cls_db_rss_kernel_rsvd_tbl_get(inst), req_tbls);
		return -EFAULT;
	}

	pp2_cls_db_rss_num_musdk_tbl_set(inst, req_tbls);

	return 0;
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

