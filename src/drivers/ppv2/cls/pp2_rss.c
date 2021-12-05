/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

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


