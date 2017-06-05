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

/**
 * @file pp2_qos.h
 *
 * PPDK Quality of Service(QoS) assured by
 *      Parser, Classifier and Policer
 *
 */

#ifndef _PP2_HW_CLS_H_
#define _PP2_HW_CLS_H_

#include "std_internal.h"

#include "../pp2_types.h"
#include "../pp2_hw_type.h"

#include "pp2_cls_types.h"
#include "pp2_cls_internal_types.h"
#include "pp2_cls_common.h"
#include "pp2_cls_utils.h"
#include "pp2_flow_rules.h"
#include "pp2_c3.h"
#include "pp2_c2.h"
#include "pp2_cls_db.h"

#define MV_ERROR		(-1)
#define MV_OK			(0)

#define WAY_MAX			(1)
#define NOT_IN_USE		(-1)
#define IN_USE			(1)
#define DWORD_BITS_LEN		(32)
#define RETRIES_EXCEEDED	(15000)
#define MVPP2_CLS_C3_SC_RES_TBL_SIZE	(256)

enum mv_pp2x_cos_classifier {
	MVPP2_COS_CLS_VLAN,	/* CoS based on VLAN pri */
	MVPP2_COS_CLS_DSCP,
	MVPP2_COS_CLS_VLAN_DSCP,	/* CoS based on VLAN pri, */
	/*if untagged and IP, then based on DSCP */
	MVPP2_COS_CLS_DSCP_VLAN,
};

int mv_pp2x_c2_init(struct pp2_hw *hw);
void mv_pp2x_cls_oversize_rxq_set(struct pp2_port *port);
void mv_pp2x_cls_port_config(struct pp2_port *port);
int mv_pp2x_cls_sw_flow_port_set(struct mv_pp2x_cls_flow_entry *fe, int type, int portid);
int mv_pp2x_cls_sw_flow_port_get(struct mv_pp2x_cls_flow_entry *fe, int *type, int *portid);
int mv_pp2x_cls_sw_flow_portid_select(struct mv_pp2x_cls_flow_entry *fe, int from);
int mv_pp2x_cls_sw_flow_hek_num_set(struct mv_pp2x_cls_flow_entry *fe, int num_of_fields);
int mv_pp2x_cls_sw_flow_hek_get(struct mv_pp2x_cls_flow_entry *fe, int *num_of_fields, int field_ids[]);
int mv_pp2x_cls_sw_flow_hek_set(struct mv_pp2x_cls_flow_entry *fe, int field_index, int field_id);
int mv_pp2x_cls_sw_flow_udf7_set(struct mv_pp2x_cls_flow_entry *fe, int mode);
int mv_pp2x_cls_sw_flow_seq_ctrl_set(struct mv_pp2x_cls_flow_entry *fe, int mode);
int mv_pp2x_cls_sw_flow_seq_ctrl_get(struct mv_pp2x_cls_flow_entry *fe, int *mode);
int mv_pp2x_cls_sw_flow_engine_set(struct mv_pp2x_cls_flow_entry *fe, int engine, int is_last);
int mv_pp2x_cls_sw_flow_engine_get(struct mv_pp2x_cls_flow_entry *fe, int *engine, int *is_last);
int mv_pp2x_cls_sw_flow_extra_get(struct mv_pp2x_cls_flow_entry *fe, int *type, int *prio);
int mv_pp2x_cls_sw_flow_extra_set(struct mv_pp2x_cls_flow_entry *fe, int type, int prio);
int mv_pp2x_cls_hw_flow_write(uintptr_t cpu_slot, struct mv_pp2x_cls_flow_entry *fe);
int mv_pp2x_cls_hw_flow_read(uintptr_t cpu_slot, int index, struct mv_pp2x_cls_flow_entry *fe);
void mv_pp2x_cls_flow_write(struct pp2_hw *hw, struct mv_pp2x_cls_flow_entry *fe);
int mv_pp2x_cls_sw_flow_dump(struct mv_pp2x_cls_flow_entry *fe);
int mv_pp2x_cls_hw_flow_hits_dump(uintptr_t cpu_slot);
int mv_pp2x_cls_hw_lkp_hit_get(uintptr_t cpu_slot, int lkpid, unsigned int *cnt);
int mv_pp2x_cls_hw_rxq_counter_get(uintptr_t cpu_slot, int phy_rxq);
void mv_pp2x_cls_sw_flow_clear(struct mv_pp2x_cls_flow_entry *fe);
int mv_pp2x_cls_hw_lkp_clear_all(uintptr_t cpu_slot);
int mv_pp2x_cls_hw_flow_clear_all(uintptr_t cpu_slot);
void mv_pp2x_cls_lookup_read(struct pp2_hw *hw, int lkpid, int way, struct mv_pp2x_cls_lookup_entry *le);
int mv_pp2x_cls_hw_lkp_write(uintptr_t cpu_slot, struct mv_pp2x_cls_lookup_entry *fe);
int mv_pp2x_cls_hw_lkp_read(uintptr_t cpu_slot, int lkpid, int way, struct mv_pp2x_cls_lookup_entry *fe);
void mv_pp2x_cls_sw_lkp_clear(struct mv_pp2x_cls_lookup_entry *fe);
int mv_pp2x_cls_hw_lkp_clear(uintptr_t cpu_slot, int lkpid, int way);
int mv_pp2x_cls_c2_qos_hw_write(struct pp2_hw *hw, struct mv_pp2x_cls_c2_qos_entry *qos);
int mv_pp2x_cls_sw_lkp_rxq_get(struct mv_pp2x_cls_lookup_entry *lkp, int *rxq);
int mv_pp2x_cls_sw_lkp_rxq_set(struct mv_pp2x_cls_lookup_entry *lkp, int rxq);
int mv_pp2x_cls_sw_lkp_mod_get(struct mv_pp2x_cls_lookup_entry *le, int *mod_base);
int mv_pp2x_cls_sw_lkp_flow_get(struct mv_pp2x_cls_lookup_entry *le, int *flow_idx);
int mv_pp2x_cls_sw_lkp_flow_set(struct mv_pp2x_cls_lookup_entry *lkp, int flow_idx);
int mv_pp2x_cls_sw_lkp_en_get(struct mv_pp2x_cls_lookup_entry *le, int *en);
int mv_pp2x_cls_sw_lkp_en_set(struct mv_pp2x_cls_lookup_entry *lkp, int en);
int mv_pp2x_cls_hw_cls_enable(uintptr_t cpu_slot, uint32_t en);
int mv_pp2x_cls_hw_lkp_dump(uintptr_t cpu_slot);
int mv_pp2x_cls_hw_flow_dump(uintptr_t cpu_slot);

int mv_pp22_rss_tbl_entry_set(struct pp2_hw *hw,
			      struct mv_pp22_rss_entry *rss);
int pp2_rss_tbl_entry_get(struct pp2_hw *hw,
			  struct mv_pp22_rss_entry *rss);
int pp2_rss_c2_enable(struct pp2_port *port, int en);
int pp2_c2_set_default_policing(struct pp2_port *port, int clear);


/*-------------------------------------------------------------------------------*/
/*	c2 Common utilities							  */
/*-------------------------------------------------------------------------------*/
int mv_pp2x_cls_c2_tcam_byte_set(struct mv_pp2x_cls_c2_entry *c2,
				 unsigned int offs, unsigned char byte,
				 unsigned char enable);
int mv_pp2x_cls_c2_hw_inv(uintptr_t cpu_slot, int index);
void mv_pp2x_cls_c2_hw_inv_all(uintptr_t cpu_slot);
int mv_pp2x_c2_hw_dump(uintptr_t cpu_slot);
int mv_pp2x_cls_c2_hw_read(uintptr_t cpu_slot, int index, struct mv_pp2x_cls_c2_entry *c2);
int mv_pp2x_cls_c2_hw_write(uintptr_t cpu_slot, int index,
			    struct mv_pp2x_cls_c2_entry *c2);
int mv_pp2x_cls_c2_qos_tbl_set(struct mv_pp2x_cls_c2_entry *c2,
			       int tbl_id, int tbl_sel);
int mv_pp2x_cls_c2_prio_set(struct mv_pp2x_cls_c2_entry *c2, int cmd,
			    int prio, int from);
int mv_pp2x_cls_c2_color_set(struct mv_pp2x_cls_c2_entry *c2, int cmd, int from);
int mv_pp2x_cls_c2_dscp_set(struct mv_pp2x_cls_c2_entry *c2,
			    int cmd, int dscp, int from);
int mv_pp2x_cls_c2_gpid_set(struct mv_pp2x_cls_c2_entry *c2, int cmd, int gpid, int from);
int mv_pp2x_cls_c2_queue_low_set(struct mv_pp2x_cls_c2_entry *c2,
				 int cmd, int queue, int from);
int mv_pp2x_cls_c2_queue_high_set(struct mv_pp2x_cls_c2_entry *c2,
				  int cmd, int queue, int from);
int mv_pp2x_cls_c2_rss_set(struct mv_pp2x_cls_c2_entry *c2, int cmd, int rss_en);
int mv_pp2x_cls_c2_forward_set(struct mv_pp2x_cls_c2_entry *c2, int cmd);
int mv_pp2x_cls_c2_policer_set(struct mv_pp2x_cls_c2_entry *c2, int cmd, int policer_id, int bank);
int mv_pp2x_cls_c2_flow_id_en(struct mv_pp2x_cls_c2_entry *c2, int flow_id_en);
int mv_pp2x_cls_c2_mod_set(struct mv_pp2x_cls_c2_entry *c2, int data_ptr, int instr_offs, int l4_csum);
int mv_pp2x_cls_c2_dup_set(struct mv_pp2x_cls_c2_entry *c2, int dupid, int count);
int mv_pp2x_cls_c2_seq_set(struct mv_pp2x_cls_c2_entry *c2, int miss, int id);
void mv_pp2x_c2_sw_clear(struct mv_pp2x_cls_c2_entry *c2);
int mv_pp2x_c2_hit_cntr_read(uintptr_t cpu_slot, int index, u32 *cntr);
int mv_pp2x_cls_c2_qos_tbl_fill_array(struct pp2_port *port,
				       u8 tbl_sel, uint8_t tc_values[]);
int mv_pp2x_cls_c2_qos_hw_read(struct pp2_hw *hw, int tbl_id, int tbl_sel,
			       int tbl_line, struct mv_pp2x_cls_c2_qos_entry *qos);
int mv_pp2x_cls_c2_qos_prio_get(struct mv_pp2x_cls_c2_qos_entry *qos, int *prio);
int mv_pp2x_cls_c2_qos_dscp_get(struct mv_pp2x_cls_c2_qos_entry *qos, int *dscp);
int mv_pp2x_cls_c2_qos_color_get(struct mv_pp2x_cls_c2_qos_entry *qos, int *color);
int mv_pp2x_cls_c2_qos_gpid_get(struct mv_pp2x_cls_c2_qos_entry *qos, int *gpid);
int mv_pp2x_cls_c2_qos_queue_get(struct mv_pp2x_cls_c2_qos_entry *qos, int *queue);
int pp2_c2_config_default_queue(struct pp2_port *port, u16 queue);

/*-------------------------------------------------------------------------------*/
/*	C3 Common utilities							  */
/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_init(uintptr_t cpu_slot);
void pp2_cls_c3_shadow_init(void);
int pp2_cls_c3_shadow_free_get(void);
int pp2_cls_c3_shadow_ext_free_get(void);
int pp2_cls_c3_shadow_ext_status_get(int index);
void pp2_cls_c3_shadow_clear(int index);
void pp2_cls_c3_shadow_get(int index, int *hek_size, int *ext_index);

/*-------------------------------------------------------------------------------*/
/*	APIs for Classification C3 engine						  */
/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_hw_read(uintptr_t cpu_slot, struct pp2_cls_c3_entry *c3, int index);
int pp2_cls_c3_hw_add(uintptr_t cpu_slot, struct pp2_cls_c3_entry *c3, int index, int ext_index);
int pp2_cls_c3_hw_miss_add(uintptr_t cpu_slot, struct pp2_cls_c3_entry *c3, int lkp_type);
int pp2_cls_c3_hw_del(uintptr_t cpu_slot, int index);
int pp2_cls_c3_hw_del_all(uintptr_t cpu_slot);
void pp2_cls_c3_sw_clear(struct pp2_cls_c3_entry *c3);
void pp2_cls_c3_hw_init_ctr_set(int cnt_val);
int pp2_cls_c3_hw_query(uintptr_t cpu_slot, struct pp2_cls_c3_entry *c3, u8 *occupied_bmp, int index[]);
int pp2_cls_c3_hw_query_add(uintptr_t cpu_slot, struct pp2_cls_c3_entry *c3, int max_search_depth,
			    struct pp2_cls_c3_hash_pair *hash_pair_arr);
int pp2_cls_c3_hw_miss_read(uintptr_t cpu_slot, struct pp2_cls_c3_entry *c3, int lkp_type);

/*-------------------------------------------------------------------------------*/
/*	APIs for Classification C3 key fields						  */
/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_sw_l4_info_set(struct pp2_cls_c3_entry *c3, int l4info);
int pp2_cls_c3_sw_lkp_type_set(struct pp2_cls_c3_entry *c3, int lkp_type);
int pp2_cls_c3_sw_port_id_set(struct pp2_cls_c3_entry *c3, int type, int portid);
int pp2_cls_c3_sw_hek_size_set(struct pp2_cls_c3_entry *c3, int hek_size);
int pp2_cls_c3_sw_hek_byte_set(struct pp2_cls_c3_entry *c3, u32 offs, u8 byte);
int pp2_cls_c3_sw_hek_word_set(struct pp2_cls_c3_entry *c3, u32 offs, u32 word);

/*-------------------------------------------------------------------------------*/
/*	APIs for Classification C3 action table fields					  */
/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_color_set(struct pp2_cls_c3_entry *c3, int cmd);
int pp2_cls_c3_queue_high_set(struct pp2_cls_c3_entry *c3, int cmd, int q);
int pp2_cls_c3_queue_low_set(struct pp2_cls_c3_entry *c3, int cmd, int q);
int pp2_cls_c3_queue_set(struct pp2_cls_c3_entry *c3, int cmd, int queue);
int pp2_cls_c3_forward_set(struct pp2_cls_c3_entry *c3, int cmd);
int pp2_cls_c3_policer_set(struct pp2_cls_c3_entry *c3, int cmd, int policer_id, int bank);
int pp2_cls_c3_flow_id_en(struct pp2_cls_c3_entry *c3, int flowid_en);
int pp2_cls_c3_rss_set(struct pp2_cls_c3_entry *c3, int cmd, int rss_en);
int pp2_cls_c3_mtu_set(struct pp2_cls_c3_entry *c3, int mtu_inx);
int pp2_cls_c3_mod_set(struct pp2_cls_c3_entry *c3, int data_ptr, int instr_offs, int l4_csum);
int pp2_cls_c3_dup_set(struct pp2_cls_c3_entry *c3, int dupid, int count);
int pp2_cls_c3_seq_set(struct pp2_cls_c3_entry *c3, int id, int bits_offs, int bits);

/*-------------------------------------------------------------------------------*/
/*	APIs for Classification C3 Hit counters management			 */
/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_hit_cntrs_read(uintptr_t cpu_slot, int index, u32 *cntr);
int pp2_cls_c3_hit_cntrs_clear_all(uintptr_t cpu_slot);
int pp2_cls_c3_hit_cntrs_read_all(uintptr_t cpu_slot);
int pp2_cls_c3_hit_cntrs_clear(uintptr_t cpu_slot, int lkp_type);
int pp2_cls_c3_hit_cntrs_miss_read(uintptr_t cpu_slot, int lkp_type, u32 *cntr);

/*-------------------------------------------------------------------------------*/
/*	 APIs for Classification C3 hit counters scan fields operation		 */
/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_scan_start(uintptr_t cpu_slot);
int pp2_cls_c3_scan_thresh_set(uintptr_t cpu_slot, int mode, int thresh);
int pp2_cls_c3_scan_clear_before_en_set(uintptr_t cpu_slot, int en);
int pp2_cls_c3_scan_lkp_type_set(uintptr_t cpu_slot, int type);
int pp2_cls_c3_scan_start_index_set(uintptr_t cpu_slot, int idx);
int pp2_cls_c3_scan_delay_set(uintptr_t cpu_slot, u32 time);
int pp2_cls_c3_scan_res_read(uintptr_t cpu_slot, int index, int *addr, int *cnt);
int pp2_cls_c3_scan_num_of_res_get(uintptr_t cpu_slot, int *res_num);

/*-------------------------------------------------------------------------------*/
/*	 APIs for Policer							 */
/*-------------------------------------------------------------------------------*/
void mv_pp2x_plcr_hw_regs(uintptr_t cpu_slot);
void mv_pp2x_plcr_hw_dump_all(uintptr_t cpu_slot);
void mv_pp2x_plcr_hw_dump_single(uintptr_t cpu_slot, int plcr);
void mv_pp2x_plcr_tb_cnt_dump(uintptr_t cpu_slot, int plcr);
int mv_pp2x_plcr_hw_base_period_set(uintptr_t cpu_slot, int period);
int mv_pp2x_plcr_hw_base_rate_gen_enable(uintptr_t cpu_slot, int enable);
int mv_pp2x_plcr_hw_enable(uintptr_t cpu_slot, int plcr, int enable);
int mv_pp2x_plcr_hw_mode(uintptr_t cpu_slot, int mode);
int mv_pp2x_plcr_hw_min_pkt_len(uintptr_t cpu_slot, int bytes);
int mv_pp2x_plcr_hw_early_drop_set(uintptr_t cpu_slot, int enable);
int mv_pp2x_plcr_hw_token_config(uintptr_t cpu_slot, int plcr, int unit, int type);
int mv_pp2x_plcr_hw_token_value(uintptr_t cpu_slot, int plcr, int value);
int mv_pp2x_plcr_hw_color_mode_set(uintptr_t cpu_slot, int plcr, int enable);
int mv_pp2x_plcr_hw_bucket_size_set(uintptr_t cpu_slot, int plcr, int commit, int excess);
int mv_pp2x_plcr_hw_cpu_thresh_set(uintptr_t cpu_slot, int idx, int threshold);
int mv_pp2x_plcr_hw_hwf_thresh_set(uintptr_t cpu_slot, int idx, int threshold);
int mv_pp2x_plcr_hw_rxq_thresh_set(uintptr_t cpu_slot, int rxq, int idx);
int mv_pp2x_plcr_hw_txq_thresh_set(uintptr_t cpu_slot, int txq, int idx);

#endif /* _PP2_HW_CLS_H_ */
