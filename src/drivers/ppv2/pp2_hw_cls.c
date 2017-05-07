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

#include "std_internal.h"

#include "pp2_types.h"

#include "pp2.h"
#include "pp2_hw_cls.h"
#include "pp2_hw_type.h"
#include "cls/pp2_prs.h"

/* TODO: Keep these until classifier phase */
#pragma GCC diagnostic ignored "-Wmissing-prototypes"

/*	C3 declarations	*/
struct pp2_cls_c3_shadow_hash_entry pp2_cls_c3_shadow_tbl[MVPP2_CLS_C3_HASH_TBL_SIZE];
int pp2_cls_c3_shadow_ext_tbl[MVPP2_CLS_C3_EXT_TBL_SIZE];
static int sw_init_cnt_set;

/* Parser configuration routines */

/********************************************************************/
/***************** Classifier Top Public lkpid table APIs ********************/
/********************************************************************/

/*------------------------------------------------------------------*/

int mv_pp2x_cls_hw_lkp_read(uintptr_t cpu_slot, int lkpid, int way,
			    struct mv_pp2x_cls_lookup_entry *fe)
{
	unsigned int reg_val = 0;

	if (mv_pp2x_ptr_validate(fe) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_range_validate(way, 0, WAY_MAX) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_range_validate(lkpid, 0,
				   MVPP2_CLS_FLOWS_TBL_SIZE) == MV_ERROR)
		return MV_ERROR;

	/* write index reg */
	reg_val = (way << MVPP2_CLS_LKP_INDEX_WAY_OFFS) |
	    (lkpid << MVPP2_CLS_LKP_INDEX_LKP_OFFS);
	pp2_reg_write(cpu_slot, MVPP2_CLS_LKP_INDEX_REG, reg_val);

	fe->way = way;
	fe->lkpid = lkpid;

	fe->data = pp2_reg_read(cpu_slot, MVPP2_CLS_LKP_TBL_REG);

	return 0;
}

int mv_pp2x_cls_hw_lkp_write(uintptr_t cpu_slot, struct mv_pp2x_cls_lookup_entry *fe)
{
	u32 reg_val = 0;

	if (mv_pp2x_ptr_validate(fe) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_range_validate(fe->way, 0, 1) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_range_validate(fe->lkpid, 0,
				   MVPP2_CLS_FLOWS_TBL_SIZE) == MV_ERROR)
		return MV_ERROR;

	/* write index reg */
	reg_val = (fe->way << MVPP2_CLS_LKP_INDEX_WAY_OFFS) |
	    (fe->lkpid << MVPP2_CLS_LKP_INDEX_LKP_OFFS);
	pp2_reg_write(cpu_slot, MVPP2_CLS_LKP_INDEX_REG, reg_val);

	/* write flow_id reg */
	pp2_reg_write(cpu_slot, MVPP2_CLS_LKP_TBL_REG, fe->data);

	return 0;
}

void mv_pp2x_cls_sw_lkp_clear(struct mv_pp2x_cls_lookup_entry *fe)
{
	memset(fe, 0, sizeof(struct mv_pp2x_cls_lookup_entry));
}

int mv_pp2x_cls_hw_lkp_clear(uintptr_t cpu_slot, int lkpid, int way)
{
	struct mv_pp2x_cls_lookup_entry fe;

	if (mv_pp2x_range_validate(lkpid, 0,
				   MVPP2_CLS_FLOWS_TBL_SIZE) == MV_ERROR)
		return -EINVAL;
	if (mv_pp2x_range_validate(way, 0, 1) == MV_ERROR)
		return -EINVAL;

	/* clear entry */
	mv_pp2x_cls_sw_lkp_clear(&fe);
	fe.lkpid = lkpid;
	fe.way = way;
	mv_pp2x_cls_hw_lkp_write(cpu_slot, &fe);

	return 0;
}

int mv_pp2x_cls_hw_lkp_clear_all(uintptr_t cpu_slot)
{
	int lkpid;

	for (lkpid = 0; lkpid < MVPP2_CLS_LKP_TBL_SIZE; lkpid++) {
		if (mv_pp2x_cls_hw_lkp_clear(cpu_slot, lkpid, 0))
			return -EINVAL;
		if (mv_pp2x_cls_hw_lkp_clear(cpu_slot, lkpid, 1))
			return -EINVAL;
	}
	return 0;
}

int mv_pp2x_cls_hw_cls_enable(uintptr_t cpu_slot, uint32_t en)
{
	if (mv_pp2x_range_validate(en, 0, 1) == MV_ERROR)
		return -EINVAL;

	/* Enable classifier */
	pp2_reg_write(cpu_slot, MVPP2_CLS_MODE_REG, en);

	return 0;
}

/*----------------------------------------------------------------------*/

int mv_pp2x_cls_sw_lkp_rxq_get(struct mv_pp2x_cls_lookup_entry *lkp, int *rxq)
{
	if (mv_pp2x_ptr_validate(lkp) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_ptr_validate(rxq) == MV_ERROR)
		return MV_ERROR;

	*rxq = (lkp->data & MVPP2_FLOWID_RXQ_MASK) >> MVPP2_FLOWID_RXQ;
	return 0;
}

int mv_pp2x_cls_sw_lkp_rxq_set(struct mv_pp2x_cls_lookup_entry *lkp, int rxq)
{
	if (mv_pp2x_ptr_validate(lkp) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_range_validate(rxq, 0,
				   (1 << MVPP2_FLOWID_RXQ_BITS) - 1) ==
	    MV_ERROR)
		return MV_ERROR;

	lkp->data &= ~MVPP2_FLOWID_RXQ_MASK;
	lkp->data |= (rxq << MVPP2_FLOWID_RXQ);

	return 0;
}

int mv_pp2x_cls_sw_lkp_mod_get(struct mv_pp2x_cls_lookup_entry *le, int *mod_base)
{
	if (mv_pp2x_ptr_validate(le) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_ptr_validate(mod_base) == MV_ERROR)
		return MV_ERROR;

	*mod_base = (le->data & MVPP2_FLOWID_MODE_MASK) >> MVPP2_FLOWID_MODE;

	return 0;
}

int mv_pp2x_cls_sw_lkp_flow_get(struct mv_pp2x_cls_lookup_entry *le, int *flow_idx)
{
	if (mv_pp2x_ptr_validate(le) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_ptr_validate(flow_idx) == MV_ERROR)
		return MV_ERROR;

	*flow_idx = (le->data & MVPP2_FLOWID_FLOW_MASK) >> MVPP2_FLOWID_FLOW;

	return 0;
}

int mv_pp2x_cls_sw_lkp_flow_set(struct mv_pp2x_cls_lookup_entry *lkp,
				int flow_idx)
{
	if (mv_pp2x_ptr_validate(lkp) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_range_validate(flow_idx, 0,
				   MVPP2_CLS_FLOWS_TBL_SIZE) == MV_ERROR)
		return MV_ERROR;

	lkp->data &= ~MVPP2_FLOWID_FLOW_MASK;
	lkp->data |= (flow_idx << MVPP2_FLOWID_FLOW);

	return 0;
}

int mv_pp2x_cls_sw_lkp_en_get(struct mv_pp2x_cls_lookup_entry *le, int *en)
{
	if (mv_pp2x_ptr_validate(le) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_ptr_validate(en) == MV_ERROR)
		return MV_ERROR;

	*en = (le->data & MVPP2_FLOWID_EN_MASK) >> MVPP2_FLOWID_EN;

	return 0;
}

int mv_pp2x_cls_sw_lkp_en_set(struct mv_pp2x_cls_lookup_entry *lkp, int en)
{
	if (mv_pp2x_ptr_validate(lkp) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_range_validate(en, 0, 1) == MV_ERROR)
		return MV_ERROR;

	lkp->data &= ~MVPP2_FLOWID_EN_MASK;
	lkp->data |= (en << MVPP2_FLOWID_EN);

	return 0;
}

/* Update classification lookup table register */
static void mv_pp2x_cls_lookup_write(struct pp2_hw *hw,
				     struct mv_pp2x_cls_lookup_entry *le)
{
	u32 val;

	val = (le->way << MVPP2_CLS_LKP_INDEX_WAY_OFFS) | le->lkpid;
	pp2_reg_write(hw->base[0].va, MVPP2_CLS_LKP_INDEX_REG, val);
	pp2_reg_write(hw->base[0].va, MVPP2_CLS_LKP_TBL_REG, le->data);
}

/* Init lookup decoding table with lookup id */
void mv_pp2x_cls_lookup_tbl_config(struct pp2_hw *hw)
{
	int index, flow_idx;
	int data[3];
	struct mv_pp2x_cls_lookup_entry le;
	struct mv_pp2x_cls_flow_info *flow_info;

	memset(&le, 0, sizeof(struct mv_pp2x_cls_lookup_entry));
	/* Enable classifier engine */
	mv_pp2x_cls_sw_lkp_en_set(&le, 1);

	for (index = 0; index < (MVPP2_PRS_FL_LAST - MVPP2_PRS_FL_START);
	     index++) {
		flow_info = &hw->cls_shadow->flow_info[index];
		data[0] = MVPP2_FLOW_TBL_SIZE;
		data[1] = MVPP2_FLOW_TBL_SIZE;
		data[2] = MVPP2_FLOW_TBL_SIZE;
		le.lkpid = hw->cls_shadow->flow_info[index].lkpid;
		/* Find the min non-zero one in flow_entry_dflt,
		 * flow_entry_vlan, and flow_entry_dscp
		 */
		if (flow_info->flow_entry_dflt)
			data[0] = flow_info->flow_entry_dflt;
		if (flow_info->flow_entry_vlan)
			data[1] = flow_info->flow_entry_vlan;
		if (flow_info->flow_entry_dscp)
			data[2] = flow_info->flow_entry_dscp;
		flow_idx = min(data[0], min(data[1], data[2]));

		/* Set flow pointer index */
		mv_pp2x_cls_sw_lkp_flow_set(&le, flow_idx);

		/* Set initial rx queue */
		mv_pp2x_cls_sw_lkp_rxq_set(&le, 0x0);

		le.way = 0;

		/* Update lookup ID table entry */
		mv_pp2x_cls_lookup_write(hw, &le);

		le.way = 1;

		/* Update lookup ID table entry */
		mv_pp2x_cls_lookup_write(hw, &le);
	}
}

/* Update the flow index for flow of lkpid */
void mv_pp2x_cls_lkp_flow_set(struct pp2_hw *hw, int lkpid, int way,
			      int flow_idx)
{
	struct mv_pp2x_cls_lookup_entry le;

	mv_pp2x_cls_lookup_read(hw, lkpid, way, &le);
	mv_pp2x_cls_sw_lkp_flow_set(&le, flow_idx);
	mv_pp2x_cls_lookup_write(hw, &le);
}

int mv_pp2x_cls_c2_hw_inv(uintptr_t cpu_slot, int index)
{
	if (!cpu_slot || index >= MVPP2_CLS_C2_TCAM_SIZE)
		return -EINVAL;

	/* write index reg */
	pp2_reg_write(cpu_slot, MVPP2_CLS2_TCAM_IDX_REG, index);

	/* set invalid bit */
	pp2_reg_write(cpu_slot, MVPP2_CLS2_TCAM_INV_REG,
		     (1 << MVPP2_CLS2_TCAM_INV_INVALID_OFF));

	/* trigger */
	pp2_reg_write(cpu_slot, MVPP2_CLS2_TCAM_DATA_REG(4), 0);

	return 0;
}

void mv_pp2x_cls_c2_hw_inv_all(uintptr_t cpu_slot)
{
	int index;

	for (index = 0; index < MVPP2_CLS_C2_TCAM_SIZE; index++)
		mv_pp2x_cls_c2_hw_inv(cpu_slot, index);
}

static void mv_pp2x_cls_c2_qos_hw_clear_all(struct pp2_hw *hw)
{
	struct mv_pp2x_cls_c2_qos_entry qos;

	memset(&qos, 0, sizeof(struct mv_pp2x_cls_c2_qos_entry));

	/* clear DSCP tables */
	qos.tbl_sel = MVPP2_QOS_TBL_SEL_DSCP;
	for (qos.tbl_id = 0; qos.tbl_id < MVPP2_QOS_TBL_NUM_DSCP;
	     qos.tbl_id++) {
		for (qos.tbl_line = 0; qos.tbl_line <
		     MVPP2_QOS_TBL_LINE_NUM_DSCP; qos.tbl_line++) {
			mv_pp2x_cls_c2_qos_hw_write(hw, &qos);
		}
	}

	/* clear PRIO tables */
	qos.tbl_sel = MVPP2_QOS_TBL_SEL_PRI;
	for (qos.tbl_id = 0; qos.tbl_id < MVPP2_QOS_TBL_NUM_PRI; qos.tbl_id++)
		for (qos.tbl_line = 0; qos.tbl_line <
		     MVPP2_QOS_TBL_LINE_NUM_PRI; qos.tbl_line++) {
			mv_pp2x_cls_c2_qos_hw_write(hw, &qos);
		}
}

/* C2 TCAM init */
int mv_pp2x_c2_init(struct pp2_hw *hw)
{
	int i;
	uintptr_t cpu_slot = hw->base[PP2_DEFAULT_REGSPACE].va;

	/* Invalid all C2 and QoS entries */
	mv_pp2x_cls_c2_hw_inv_all(cpu_slot);

	mv_pp2x_cls_c2_qos_hw_clear_all(hw);

	/* Set CLSC2_TCAM_CTRL to enable C2, or C2 does not work */
	pp2_reg_write(cpu_slot, MVPP2_CLS2_TCAM_CTRL_REG,
		      MVPP2_CLS2_TCAM_CTRL_EN_MASK);

	/* Allocate mem for c2 shadow */
	hw->c2_shadow = kcalloc(1, sizeof(struct mv_pp2x_c2_shadow), GFP_KERNEL);
	if (!hw->c2_shadow)
		return -ENOMEM;

	/* Init the rule idx to invalid value */
	for (i = 0; i < 8; i++) {
		hw->c2_shadow->rule_idx_info[i].vlan_pri_idx =
		    MVPP2_CLS_C2_TCAM_SIZE;
		hw->c2_shadow->rule_idx_info[i].dscp_pri_idx =
		    MVPP2_CLS_C2_TCAM_SIZE;
		hw->c2_shadow->rule_idx_info[i].default_rule_idx =
		    MVPP2_CLS_C2_TCAM_SIZE;
	}
	hw->c2_shadow->c2_tcam_free_start = 0;

	return 0;
}

void mv_pp2x_cls_oversize_rxq_set(struct pp2_port *port)
{
	uintptr_t cpu_slot = port->cpu_slot;

	pp2_reg_write(cpu_slot, MVPP2_CLS_OVERSIZE_RXQ_LOW_REG(port->id),
		      port->first_rxq);
}

void mv_pp2x_cls_port_config(struct pp2_port *port)
{
	struct mv_pp2x_cls_lookup_entry le;
	struct pp2_hw *hw = &port->parent->hw;
	uintptr_t cpu_slot = port->cpu_slot;
	u32 val;

	/* Set way for the port */
	val = pp2_reg_read(cpu_slot, MVPP2_CLS_PORT_WAY_REG);
	val &= ~MVPP2_CLS_PORT_WAY_MASK(port->id);
	pp2_reg_write(cpu_slot, MVPP2_CLS_PORT_WAY_REG, val);

	/* Pick the entry to be accessed in lookup ID decoding table
	 * according to the way and lkpid.
	 */
	le.lkpid = port->id;
	le.way = 0;
	le.data = 0;

	/* Set initial CPU queue for receiving packets */
	le.data &= ~MVPP2_CLS_LKP_TBL_RXQ_MASK;
	le.data |= port->first_rxq;

	/* Disable classification engines */
	le.data &= ~MVPP2_CLS_LKP_TBL_LOOKUP_EN_MASK;

	/* Update lookup ID table entry */
	mv_pp2x_cls_lookup_write(hw, &le);
}

/* The function get the number of cpu online */
static inline int mv_pp2x_num_online_cpu_get(struct pp2_inst *pp)
{
	u8 num_online_cpus = 0;
	u16 x = pp->cpu_map;

	while (x) {
		x &= (x - 1);
		num_online_cpus++;
	}

	return num_online_cpus;
}

/* The function calculate the width, such as cpu width, cos queue width */
static inline void mv_pp2x_width_calc(struct pp2_inst *pp2, uint32_t *cpu_width,
				      u32 *cos_width,
				      uint32_t *port_rxq_width)
{
	if (pp2) {
		/* Calculate CPU width */
		if (cpu_width)
			*cpu_width =
			    ilog2(roundup_pow_of_two
				  (mv_pp2x_num_online_cpu_get(pp2)));

		/* Calculate cos queue width */
		if (cos_width)
			*cos_width =
			    ilog2(roundup_pow_of_two
				  (pp2->pp2_cfg.cos_cfg.num_cos_queues));
		/* Calculate rx queue width on the port */
		if (port_rxq_width)
			*port_rxq_width =
			    ilog2(roundup_pow_of_two
				  (pp2->pp2xdata.pp2x_max_port_rxqs));
	}
}

/*********************************************************************/
/***************** Classifier Top Public flows table APIs  ********************/
/********************************************************************/

int mv_pp2x_cls_hw_flow_write(uintptr_t cpu_slot,
			      struct mv_pp2x_cls_flow_entry *fe)
{
	if (mv_pp2x_range_validate(fe->index, 0,
				   MVPP2_CLS_FLOWS_TBL_SIZE) == MV_ERROR)
		return -EINVAL;

	/* write index */
	pp2_reg_write(cpu_slot, MVPP2_CLS_FLOW_INDEX_REG, fe->index);

	pp2_reg_write(cpu_slot, MVPP2_CLS_FLOW_TBL0_REG, fe->data[0]);
	pp2_reg_write(cpu_slot, MVPP2_CLS_FLOW_TBL1_REG, fe->data[1]);
	pp2_reg_write(cpu_slot, MVPP2_CLS_FLOW_TBL2_REG, fe->data[2]);

	return 0;
}

int mv_pp2x_cls_hw_flow_read(uintptr_t cpu_slot, int index,
			     struct mv_pp2x_cls_flow_entry *fe)
{
	if (mv_pp2x_ptr_validate(fe) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_range_validate(index, 0,
				   MVPP2_CLS_FLOWS_TBL_SIZE) == MV_ERROR)
		return MV_ERROR;

	fe->index = index;

	/*write index */
	pp2_reg_write(cpu_slot, MVPP2_CLS_FLOW_INDEX_REG, index);

	fe->data[0] = pp2_reg_read(cpu_slot, MVPP2_CLS_FLOW_TBL0_REG);
	fe->data[1] = pp2_reg_read(cpu_slot, MVPP2_CLS_FLOW_TBL1_REG);
	fe->data[2] = pp2_reg_read(cpu_slot, MVPP2_CLS_FLOW_TBL2_REG);

	return 0;
}

int mv_pp2x_cls_sw_flow_hek_get(struct mv_pp2x_cls_flow_entry *fe,
				int *num_of_fields, int field_ids[])
{
	int index;

	if (mv_pp2x_ptr_validate(fe) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_ptr_validate(num_of_fields) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_ptr_validate(field_ids) == MV_ERROR)
		return MV_ERROR;

	*num_of_fields = (fe->data[1] &
			  MVPP2_FLOW_FIELDS_NUM_MASK) >> MVPP2_FLOW_FIELDS_NUM;

	for (index = 0; index < (*num_of_fields); index++)
		field_ids[index] = ((fe->data[2] &
				     MVPP2_FLOW_FIELD_MASK(index)) >>
				    MVPP2_FLOW_FIELD_ID(index));

	return 0;
}

int mv_pp2x_cls_sw_flow_port_get(struct mv_pp2x_cls_flow_entry *fe,
				 int *type, int *portid)
{
	if (mv_pp2x_ptr_validate(fe) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_ptr_validate(type) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_ptr_validate(portid) == MV_ERROR)
		return MV_ERROR;

	*type = (fe->data[0] & MVPP2_FLOW_PORT_TYPE_MASK) >>
	    MVPP2_FLOW_PORT_TYPE;
	*portid = (fe->data[0] & MVPP2_FLOW_PORT_ID_MASK) >> MVPP2_FLOW_PORT_ID;

	return 0;
}

int mv_pp2x_cls_sw_flow_port_set(struct mv_pp2x_cls_flow_entry *fe,
				 int type, int portid)
{
	if (mv_pp2x_ptr_validate(fe) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_range_validate(type, 0,
				   ((1 << MVPP2_FLOW_PORT_TYPE_BITS) - 1)) ==
	    MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_range_validate(portid, 0,
				   ((1 << MVPP2_FLOW_PORT_ID_BITS) - 1)) ==
	    MV_ERROR)
		return MV_ERROR;

	fe->data[0] &= ~MVPP2_FLOW_PORT_ID_MASK;
	fe->data[0] &= ~MVPP2_FLOW_PORT_TYPE_MASK;

	fe->data[0] |= (portid << MVPP2_FLOW_PORT_ID);
	fe->data[0] |= (type << MVPP2_FLOW_PORT_TYPE);

	return 0;
}

int mv_pp2x_cls_sw_flow_portid_select(struct mv_pp2x_cls_flow_entry *fe,
				      int from)
{
	if (mv_pp2x_ptr_validate(fe) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_range_validate(from, 0, 1) == MV_ERROR)
		return MV_ERROR;

	if (from)
		fe->data[0] |= MVPP2_FLOW_PORT_ID_SEL_MASK;
	else
		fe->data[0] &= ~MVPP2_FLOW_PORT_ID_SEL_MASK;

	return 0;
}

int mv_pp2x_cls_sw_flow_pppoe_set(struct mv_pp2x_cls_flow_entry *fe, int mode)
{
	if (mv_pp2x_ptr_validate(fe) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_range_validate(mode, 0, MVPP2_FLOW_PPPOE_MAX) == MV_ERROR)
		return MV_ERROR;

	fe->data[0] &= ~MVPP2_FLOW_PPPOE_MASK;
	fe->data[0] |= (mode << MVPP2_FLOW_PPPOE);
	return 0;
}

int mv_pp2x_cls_sw_flow_vlan_set(struct mv_pp2x_cls_flow_entry *fe, int mode)
{
	if (mv_pp2x_ptr_validate(fe) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_range_validate(mode, 0, MVPP2_FLOW_VLAN_MAX) == MV_ERROR)
		return MV_ERROR;

	fe->data[0] &= ~MVPP2_FLOW_VLAN_MASK;
	fe->data[0] |= (mode << MVPP2_FLOW_VLAN);
	return 0;
}

int mv_pp2x_cls_sw_flow_macme_set(struct mv_pp2x_cls_flow_entry *fe, int mode)
{
	if (mv_pp2x_ptr_validate(fe) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_range_validate(mode, 0, MVPP2_FLOW_MACME_MAX) == MV_ERROR)
		return MV_ERROR;

	fe->data[0] &= ~MVPP2_FLOW_MACME_MASK;
	fe->data[0] |= (mode << MVPP2_FLOW_MACME);
	return 0;
}

int mv_pp2x_cls_sw_flow_udf7_set(struct mv_pp2x_cls_flow_entry *fe, int mode)
{
	if (mv_pp2x_ptr_validate(fe) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_range_validate(mode, 0, MVPP2_FLOW_UDF7_MAX) == MV_ERROR)
		return MV_ERROR;

	fe->data[0] &= ~MVPP2_FLOW_UDF7_MASK;
	fe->data[0] |= (mode << MVPP2_FLOW_UDF7);
	return 0;
}

int mv_pp2x_cls_sw_flow_seq_ctrl_set(struct mv_pp2x_cls_flow_entry *fe,
				     int mode)
{
	if (mv_pp2x_ptr_validate(fe) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_range_validate(mode, 0, MVPP2_FLOW_ENGINE_MAX) == MV_ERROR)
		return MV_ERROR;

	fe->data[1] &= ~MVPP2_FLOW_SEQ_CTRL_MASK;
	fe->data[1] |= (mode << MVPP2_FLOW_SEQ_CTRL);

	return 0;
}

int mv_pp2x_cls_sw_flow_seq_ctrl_get(struct mv_pp2x_cls_flow_entry *fe,
				     int *mode)
{
	if (mv_pp2x_ptr_validate(fe) == MV_ERROR)
		return MV_ERROR;
	if (mv_pp2x_ptr_validate(mode) == MV_ERROR)
		return MV_ERROR;

	*mode = (fe->data[1] & MVPP2_FLOW_SEQ_CTRL_MASK) >> MVPP2_FLOW_SEQ_CTRL;

	return 0;
}

int mv_pp2x_cls_sw_flow_engine_get(struct mv_pp2x_cls_flow_entry *fe,
				   int *engine, int *is_last)
{
	if (mv_pp2x_ptr_validate(fe) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_ptr_validate(engine) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_ptr_validate(is_last) == MV_ERROR)
		return MV_ERROR;

	*engine = (fe->data[0] & MVPP2_FLOW_ENGINE_MASK) >> MVPP2_FLOW_ENGINE;
	*is_last = fe->data[0] & MVPP2_FLOW_LAST_MASK;

	return 0;
}

int mv_pp2x_cls_sw_flow_engine_set(struct mv_pp2x_cls_flow_entry *fe,
				   int engine, int is_last)
{
	if (mv_pp2x_ptr_validate(fe) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_range_validate(is_last, 0, 1) == MV_ERROR)
		return MV_ERROR;

	fe->data[0] &= ~MVPP2_FLOW_LAST_MASK;
	fe->data[0] &= ~MVPP2_FLOW_ENGINE_MASK;

	fe->data[0] |= is_last;
	fe->data[0] |= (engine << MVPP2_FLOW_ENGINE);

	return 0;
}

int mv_pp2x_cls_sw_flow_extra_get(struct mv_pp2x_cls_flow_entry *fe,
				  int *type, int *prio)
{
	if (mv_pp2x_ptr_validate(fe) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_ptr_validate(type) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_ptr_validate(prio) == MV_ERROR)
		return MV_ERROR;

	*type = (fe->data[1] & MVPP2_FLOW_LKP_TYPE_MASK) >> MVPP2_FLOW_LKP_TYPE;
	*prio = (fe->data[1] & MVPP2_FLOW_FIELD_PRIO_MASK) >>
	    MVPP2_FLOW_FIELD_PRIO;

	return 0;
}

int mv_pp2x_cls_sw_flow_extra_set(struct mv_pp2x_cls_flow_entry *fe,
				  int type, int prio)
{
	if (mv_pp2x_ptr_validate(fe) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_range_validate(type, 0, MVPP2_FLOW_PORT_ID_MAX) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_range_validate(prio, 0,
				   ((1 << MVPP2_FLOW_FIELD_ID_BITS) - 1)) == MV_ERROR)
		return MV_ERROR;

	fe->data[1] &= ~MVPP2_FLOW_LKP_TYPE_MASK;
	fe->data[1] |= (type << MVPP2_FLOW_LKP_TYPE);

	fe->data[1] &= ~MVPP2_FLOW_FIELD_PRIO_MASK;
	fe->data[1] |= (prio << MVPP2_FLOW_FIELD_PRIO);

	return 0;
}

/* Classifier configuration routines */

/* Update classification flow table registers */
void mv_pp2x_cls_flow_write(struct pp2_hw *hw,
			    struct mv_pp2x_cls_flow_entry *fe)
{
	pp2_reg_write(hw->base[0].va, MVPP2_CLS_FLOW_INDEX_REG, fe->index);
	pp2_reg_write(hw->base[0].va, MVPP2_CLS_FLOW_TBL0_REG, fe->data[0]);
	pp2_reg_write(hw->base[0].va, MVPP2_CLS_FLOW_TBL1_REG, fe->data[1]);
	pp2_reg_write(hw->base[0].va, MVPP2_CLS_FLOW_TBL2_REG, fe->data[2]);
}

static void mv_pp2x_cls_flow_read(struct pp2_hw *hw, int index,
				  struct mv_pp2x_cls_flow_entry *fe)
{
	fe->index = index;
	/*write index */
	pp2_reg_write(hw->base[0].va, MVPP2_CLS_FLOW_INDEX_REG, index);

	fe->data[0] = pp2_reg_read(hw->base[0].va, MVPP2_CLS_FLOW_TBL0_REG);
	fe->data[1] = pp2_reg_read(hw->base[0].va, MVPP2_CLS_FLOW_TBL1_REG);
	fe->data[2] = pp2_reg_read(hw->base[0].va, MVPP2_CLS_FLOW_TBL2_REG);
}

int mv_pp2x_cls_sw_flow_dump(struct mv_pp2x_cls_flow_entry *fe)
{
	int int32bit_1, int32bit_2, i;
	int fields_arr[MVPP2_CLS_FLOWS_TBL_FIELDS_MAX];
	int status = 0;

	if (mv_pp2x_ptr_validate(fe) == MV_ERROR)
		return MV_ERROR;

	printf("INDEX: F[0] F[1] F[2] F[3] PRT[T  ID] ENG LAST LKP_TYP  PRIO\n");

	/* index */
	printf("%5d  ", fe->index);

	/* filed[0] filed[1] filed[2] filed[3] */
	status |= mv_pp2x_cls_sw_flow_hek_get(fe, &int32bit_1, fields_arr);

	for (i = 0 ; i < MVPP2_CLS_FLOWS_TBL_FIELDS_MAX; i++)
		if (i < int32bit_1)
			printf("0x%2.2x ", fields_arr[i]);
		else
			printf(" NA  ");

	/* port_type port_id */
	status |= mv_pp2x_cls_sw_flow_port_get(fe, &int32bit_1, &int32bit_2);
	printf("[%1d  0x%3.3x]  ", int32bit_1, int32bit_2);

	/* engine_num last_bit */
	status |= mv_pp2x_cls_sw_flow_engine_get(fe, &int32bit_1, &int32bit_2);
	printf("%1d   %1d    ", int32bit_1, int32bit_2);

	/* lookup_type priority */
	status |= mv_pp2x_cls_sw_flow_extra_get(fe, &int32bit_1, &int32bit_2);
	printf("0x%2.2x    0x%2.2x", int32bit_1, int32bit_2);

	printf("\n       PPPEO   VLAN   MACME   UDF7   SELECT SEQ_CTRL\n");
	printf("         %1d      %1d      %1d       %1d      %1d      %1d\n",
	       (u32)((fe->data[0] & MVPP2_FLOW_PPPOE_MASK) >> MVPP2_FLOW_PPPOE),
	       (u32)((fe->data[0] & MVPP2_FLOW_VLAN_MASK) >> MVPP2_FLOW_VLAN),
	       (u32)((fe->data[0] & MVPP2_FLOW_MACME_MASK) >> MVPP2_FLOW_MACME),
	       (u32)((fe->data[0] & MVPP2_FLOW_UDF7_MASK) >> MVPP2_FLOW_UDF7),
	       (u32)((fe->data[0] & MVPP2_FLOW_PORT_ID_SEL_MASK) >> MVPP2_FLOW_PORT_ID_SEL),
	       (u32)((fe->data[1] & MVPP2_FLOW_SEQ_CTRL_MASK) >> MVPP2_FLOW_SEQ_CTRL));
	printf("\n");

	return 0;
}

void mv_pp2x_cls_sw_flow_clear(struct mv_pp2x_cls_flow_entry *fe)
{
	memset(fe, 0, sizeof(struct mv_pp2x_cls_flow_entry));
}

int mv_pp2x_cls_hw_flow_clear_all(uintptr_t cpu_slot)
{
	int index;

	struct mv_pp2x_cls_flow_entry fe;

	mv_pp2x_cls_sw_flow_clear(&fe);

	for (index = 0; index < MVPP2_CLS_FLOWS_TBL_SIZE ; index++) {
		fe.index = index;
		if (mv_pp2x_cls_hw_flow_write(cpu_slot, &fe))
			return -EINVAL;
	}
	return 0;
}

static int mv_pp2x_cls_hw_flow_hit_get(uintptr_t cpu_slot, int index, unsigned int *cnt)
{
	if (mv_pp2x_range_validate(index, 0, MVPP2_CLS_FLOWS_TBL_SIZE) == MV_ERROR)
		return -EINVAL;

	/*set index */
	pp2_reg_write(cpu_slot, MVPP2_CNT_IDX_REG, MVPP2_CNT_IDX_FLOW(index));

	if (cnt)
		*cnt = pp2_reg_read(cpu_slot, MVPP2_CLS_FLOW_TBL_HIT_REG);
	else
		printf("HITS = %d\n", pp2_reg_read(cpu_slot, MVPP2_CLS_FLOW_TBL_HIT_REG));

	return 0;
}

int mv_pp2x_cls_hw_lkp_hit_get(uintptr_t cpu_slot, int lkpid, u32 *cnt)
{
	int way = 0;

	if (mv_pp2x_range_validate(lkpid, 0, MVPP2_CLS_LKP_TBL_SIZE) == MV_ERROR)
		return -EINVAL;

	/*set index */
	pp2_reg_write(cpu_slot, MVPP2_CNT_IDX_REG, MVPP2_CNT_IDX_LKP(lkpid, way));

	if (cnt)
		*cnt = pp2_reg_read(cpu_slot, MVPP2_CLS_LKP_TBL_HIT_REG);
	else
		printf("HITS: %d\n", pp2_reg_read(cpu_slot, MVPP2_CLS_LKP_TBL_HIT_REG));

	return 0;
}

int mv_pp2x_cls_hw_rxq_counter_get(uintptr_t cpu_slot, int phy_rxq)
{
	u32 dropped;

	pp2_reg_write(cpu_slot, MVPP2_CNT_IDX_REG, phy_rxq);

	dropped = pp2_reg_read(cpu_slot, MVPP2_RX_PKT_FULLQ_DROP_REG);
	dropped += pp2_reg_read(cpu_slot, MVPP2_RX_PKT_EARLY_DROP_REG);
	dropped += pp2_reg_read(cpu_slot, MVPP2_RX_PKT_BM_DROP_REG);
	pr_info("rx dropped packet = 0x%8.8x\n", dropped);
	pr_info("rx enqueue packet = 0x%8.8x\n", pp2_reg_read(cpu_slot, MVPP2_RX_DESC_ENQ_REG));

	return 0;
}

int mv_pp2x_cls_hw_flow_dump(uintptr_t cpu_slot)
{
	int index;

	struct mv_pp2x_cls_flow_entry fe;

	for (index = 0; index < MVPP2_CLS_FLOWS_TBL_SIZE; index++) {
		mv_pp2x_cls_hw_flow_read(cpu_slot, index, &fe);
		mv_pp2x_cls_sw_flow_dump(&fe);
		mv_pp2x_cls_hw_flow_hit_get(cpu_slot, index, NULL);
		printf("\n");
	}

	return 0;
}

int mv_pp2x_cls_hw_flow_hits_dump(uintptr_t cpu_slot)
{
	struct mv_pp2x_cls_flow_entry fe;
	int index;
	u32 cnt = 0;

	for (index = 0; index < MVPP2_CLS_FLOWS_TBL_SIZE; index++) {
		mv_pp2x_cls_hw_flow_hit_get(cpu_slot, index, &cnt);
		if (cnt != 0) {
			mv_pp2x_cls_hw_flow_read(cpu_slot, index, &fe);
			mv_pp2x_cls_sw_flow_dump(&fe);
			printf("HITS = %d\n", cnt);
		}
	}

	return 0;
}

int mv_pp2x_cls_hw_lkp_hits_dump(uintptr_t cpu_slot)
{
	int index, way = 0;
	u32 cnt;

	printf("< ID  WAY >:	HITS\n");
	for (index = 0; index < MVPP2_CLS_LKP_TBL_SIZE; index++) {
		mv_pp2x_cls_hw_lkp_hit_get(cpu_slot, index, &cnt);
		if (cnt != 0)
			printf(" 0x%2.2x  %1.1d\t0x%8.8x\n", index, way, cnt);
	}
	return 0;
}

int mv_pp2x_cls_hw_lkp_dump(uintptr_t cpu_slot)
{
	int index, way = 0, rxq, en, flow, mod;
	u32 uint32bit;
	struct mv_pp2x_cls_lookup_entry le;

	printf("\n ID :	RXQ	EN	FLOW	MODE_BASE  HITS\n");
	for (index = 0; index < MVPP2_CLS_LKP_TBL_SIZE; index++) {
		mv_pp2x_cls_hw_lkp_read(cpu_slot, index, way, &le);
		printf(" 0x%2.2x \t", le.lkpid);
		mv_pp2x_cls_sw_lkp_rxq_get(&le, &rxq);
		printf("0x%2.2x\t", rxq);
		mv_pp2x_cls_sw_lkp_en_get(&le, &en);
		printf("%1.1d\t", en);
		mv_pp2x_cls_sw_lkp_flow_get(&le, &flow);
		printf("0x%3.3x\t", flow);
		mv_pp2x_cls_sw_lkp_mod_get(&le, &mod);
		printf(" 0x%2.2x\t", mod);
		mv_pp2x_cls_hw_lkp_hit_get(cpu_slot, index, &uint32bit);
		printf(" 0x%8.8x\n", 0);
		printf("\n");
	}
	return 0;
}

/* Operations on flow entry */
int mv_pp2x_cls_sw_flow_hek_num_set(struct mv_pp2x_cls_flow_entry *fe,
				    int num_of_fields)
{
	if (mv_pp2x_ptr_validate(fe) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_range_validate(num_of_fields, 0,
				   MVPP2_CLS_FLOWS_TBL_FIELDS_MAX) == MV_ERROR)
		return MV_ERROR;

	fe->data[1] &= ~MVPP2_FLOW_FIELDS_NUM_MASK;
	fe->data[1] |= (num_of_fields << MVPP2_FLOW_FIELDS_NUM);

	return 0;
}

int mv_pp2x_cls_sw_flow_hek_set(struct mv_pp2x_cls_flow_entry *fe,
				int field_index, int field_id)
{
	int num_of_fields;

	/* get current num_of_fields */
	num_of_fields = ((fe->data[1] &
			  MVPP2_FLOW_FIELDS_NUM_MASK) >> MVPP2_FLOW_FIELDS_NUM);

	if (num_of_fields < (field_index + 1)) {
		pr_debug("%s:num of heks=%d ,idx(%d) out of range\n",
			__func__, num_of_fields, field_index);
		return -1;
	}

	fe->data[2] &= ~MVPP2_FLOW_FIELD_MASK(field_index);
	fe->data[2] |= (field_id << MVPP2_FLOW_FIELD_ID(field_index));

	return 0;
}

static void mv_pp2x_cls_sw_flow_eng_set(struct mv_pp2x_cls_flow_entry *fe,
					int engine, int is_last)
{
	fe->data[0] &= ~MVPP2_FLOW_LAST_MASK;
	fe->data[0] &= ~MVPP2_FLOW_ENGINE_MASK;

	fe->data[0] |= is_last;
	fe->data[0] |= (engine << MVPP2_FLOW_ENGINE);
	fe->data[0] |= MVPP2_FLOW_PORT_ID_SEL_MASK;
}

void mv_pp2x_cls_lookup_read(struct pp2_hw *hw, int lkpid, int way,
			     struct mv_pp2x_cls_lookup_entry *le)
{
	unsigned int val = 0;

	/* write index reg */
	val = (way << MVPP2_CLS_LKP_INDEX_WAY_OFFS) | lkpid;
	pp2_reg_write(hw->base[0].va, MVPP2_CLS_LKP_INDEX_REG, val);
	le->way = way;
	le->lkpid = lkpid;
	le->data = pp2_reg_read(hw->base[0].va, MVPP2_CLS_LKP_TBL_REG);
}

void mv_pp2x_cls_flow_port_add(struct pp2_hw *hw, int index, int port_id)
{
	u32 data;

	/* Write flow index */
	pp2_reg_write(hw->base[0].va, MVPP2_CLS_FLOW_INDEX_REG, index);
	/* Read first data with port info */
	data = pp2_reg_read(hw->base[0].va, MVPP2_CLS_FLOW_TBL0_REG);
	/* Add the port */
	data |= ((1 << port_id) << MVPP2_FLOW_PORT_ID);
	/* Update the register */
	pp2_reg_write(hw->base[0].va, MVPP2_CLS_FLOW_TBL0_REG, data);
}

void mv_pp2x_cls_flow_port_del(struct pp2_hw *hw, int index, int port_id)
{
	u32 data;

	/* Write flow index */
	pp2_reg_write(hw->base[0].va, MVPP2_CLS_FLOW_INDEX_REG, index);
	/* Read first data with port info */
	data = pp2_reg_read(hw->base[0].va, MVPP2_CLS_FLOW_TBL0_REG);
	/* Delete the port */
	data &= ~(((1 << port_id) << MVPP2_FLOW_PORT_ID));
	/* Update the register */
	pp2_reg_write(hw->base[0].va, MVPP2_CLS_FLOW_TBL0_REG, data);
}

/* The function prepare a temporary flow table for lkpid flow,
 * in order to change the original one
 */
void mv_pp2x_cls_flow_tbl_temp_copy(struct pp2_hw *hw, int lkpid,
				    int *temp_flow_idx)
{
	struct mv_pp2x_cls_flow_entry fe;
	int index = lkpid - MVPP2_PRS_FL_START;
	int flow_start = hw->cls_shadow->flow_free_start;
	struct mv_pp2x_cls_flow_info *flow_info;

	flow_info = &hw->cls_shadow->flow_info[index];

	if (flow_info->flow_entry_dflt) {
		mv_pp2x_cls_flow_read(hw, flow_info->flow_entry_dflt, &fe);
		fe.index = flow_start++;
		mv_pp2x_cls_flow_write(hw, &fe);
	}
	if (flow_info->flow_entry_vlan) {
		mv_pp2x_cls_flow_read(hw, flow_info->flow_entry_vlan, &fe);
		fe.index = flow_start++;
		mv_pp2x_cls_flow_write(hw, &fe);
	}
	if (flow_info->flow_entry_dscp) {
		mv_pp2x_cls_flow_read(hw, flow_info->flow_entry_dscp, &fe);
		fe.index = flow_start++;
		mv_pp2x_cls_flow_write(hw, &fe);
	}
	if (flow_info->flow_entry_rss1) {
		mv_pp2x_cls_flow_read(hw, flow_info->flow_entry_rss1, &fe);
		fe.index = flow_start++;
		mv_pp2x_cls_flow_write(hw, &fe);
	}
	if (flow_info->flow_entry_rss2) {
		mv_pp2x_cls_flow_read(hw, flow_info->flow_entry_rss2, &fe);
		fe.index = flow_start++;
		mv_pp2x_cls_flow_write(hw, &fe);
	}

	*temp_flow_idx = hw->cls_shadow->flow_free_start;
}

/* To init flow table waccording to different flow */
static inline void mv_pp2x_cls_flow_cos(struct pp2_hw *hw,
					struct mv_pp2x_cls_flow_entry *fe,
					int lkpid, int cos_type)
{
	int hek_num, field_id, lkp_type, is_last;
	int entry_idx = hw->cls_shadow->flow_free_start;

	switch (cos_type) {
	case MVPP2_COS_TYPE_VLAN:
		lkp_type = MVPP2_CLS_LKP_VLAN_PRI;
		break;
	case MVPP2_COS_TYPE_DSCP:
		lkp_type = MVPP2_CLS_LKP_DSCP_PRI;
		break;
	default:
		lkp_type = MVPP2_CLS_LKP_DEFAULT;
		break;
	}
	hek_num = 0;
	if ((lkpid == MVPP2_PRS_FL_NON_IP_UNTAG &&
	     cos_type == MVPP2_COS_TYPE_DEF) ||
	    (lkpid == MVPP2_PRS_FL_NON_IP_TAG &&
	     cos_type == MVPP2_COS_TYPE_VLAN))
		is_last = 1;
	else
		is_last = 0;

	/* Set SW */
	memset(fe, 0, sizeof(struct mv_pp2x_cls_flow_entry));
	mv_pp2x_cls_sw_flow_hek_num_set(fe, hek_num);
	if (hek_num)
		mv_pp2x_cls_sw_flow_hek_set(fe, 0, field_id);
	mv_pp2x_cls_sw_flow_eng_set(fe, MVPP2_CLS_ENGINE_C2, is_last);
	mv_pp2x_cls_sw_flow_extra_set(fe, lkp_type, MVPP2_CLS_FL_COS_PRI);
	fe->index = entry_idx;

	/* Write HW */
	mv_pp2x_cls_flow_write(hw, fe);

	/* Update Shadow */
	if (cos_type == MVPP2_COS_TYPE_DEF)
		hw->cls_shadow->flow_info[lkpid -
					  MVPP2_PRS_FL_START].flow_entry_dflt =
		    entry_idx;
	else if (cos_type == MVPP2_COS_TYPE_VLAN)
		hw->cls_shadow->flow_info[lkpid -
					  MVPP2_PRS_FL_START].flow_entry_vlan =
		    entry_idx;
	else
		hw->cls_shadow->flow_info[lkpid -
					  MVPP2_PRS_FL_START].flow_entry_dscp =
		    entry_idx;

	/* Update first available flow entry */
	hw->cls_shadow->flow_free_start++;
}

int mv_pp2x_cls_c2_qos_hw_read(struct pp2_hw *hw, int tbl_id, int tbl_sel,
			       int tbl_line, struct mv_pp2x_cls_c2_qos_entry *qos)
{
	u32 reg_val = 0;

	if (mv_pp2x_ptr_validate(hw) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_ptr_validate(qos) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_range_validate(tbl_sel, 0, 1) == MV_ERROR)
		return MV_ERROR;

	if (tbl_sel == 1) {
		/*dscp*/
		if (mv_pp2x_range_validate(tbl_id, 0, MVPP2_QOS_TBL_NUM_DSCP) == MV_ERROR)
			return MV_ERROR;

		if (mv_pp2x_range_validate(tbl_line, 0, MVPP2_QOS_TBL_LINE_NUM_DSCP) == MV_ERROR)
			return MV_ERROR;
	} else {
		/*pri*/
		if (mv_pp2x_range_validate(tbl_id, 0, MVPP2_QOS_TBL_NUM_PRI) == MV_ERROR)
			return MV_ERROR;

		if (mv_pp2x_range_validate(tbl_line, 0, MVPP2_QOS_TBL_LINE_NUM_PRI) == MV_ERROR)
			return MV_ERROR;
	}

	qos->tbl_id = tbl_id;
	qos->tbl_sel = tbl_sel;
	qos->tbl_line = tbl_line;

	/* write index reg */
	reg_val |= (tbl_line << MVPP2_CLS2_DSCP_PRI_INDEX_LINE_OFF);
	reg_val |= (tbl_sel << MVPP2_CLS2_DSCP_PRI_INDEX_SEL_OFF);
	reg_val |= (tbl_id << MVPP2_CLS2_DSCP_PRI_INDEX_TBL_ID_OFF);

	pp2_reg_write(hw->base[0].va, MVPP2_CLS2_DSCP_PRI_INDEX_REG, reg_val);

	/* read data reg*/
	qos->data = pp2_reg_read(hw->base[0].va, MVPP2_CLS2_QOS_TBL_REG);

	return MV_OK;
}

int mv_pp2x_cls_c2_qos_hw_write(struct pp2_hw *hw,
				struct mv_pp2x_cls_c2_qos_entry *qos)
{
	unsigned int reg_val = 0;

	if (!qos || qos->tbl_sel > MVPP2_QOS_TBL_SEL_DSCP)
		return -EINVAL;

	if (qos->tbl_sel == MVPP2_QOS_TBL_SEL_DSCP) {
		/*dscp */
		if (qos->tbl_id >= MVPP2_QOS_TBL_NUM_DSCP ||
		    qos->tbl_line >= MVPP2_QOS_TBL_LINE_NUM_DSCP)
			return -EINVAL;
	} else {
		/*pri */
		if (qos->tbl_id >= MVPP2_QOS_TBL_NUM_PRI ||
		    qos->tbl_line >= MVPP2_QOS_TBL_LINE_NUM_PRI)
			return -EINVAL;
	}
	/* write index reg */
	reg_val |= (qos->tbl_line << MVPP2_CLS2_DSCP_PRI_INDEX_LINE_OFF);
	reg_val |= (qos->tbl_sel << MVPP2_CLS2_DSCP_PRI_INDEX_SEL_OFF);
	reg_val |= (qos->tbl_id << MVPP2_CLS2_DSCP_PRI_INDEX_TBL_ID_OFF);
	pp2_reg_write(hw->base[0].va, MVPP2_CLS2_DSCP_PRI_INDEX_REG, reg_val);

	/* write data reg */
	pp2_reg_write(hw->base[0].va, MVPP2_CLS2_QOS_TBL_REG, qos->data);

	return 0;
}

int mv_pp2x_cls_c2_qos_queue_set(struct mv_pp2x_cls_c2_qos_entry *qos,
				 uint8_t queue)
{
	if (!qos || queue >= (1 << MVPP2_CLS2_QOS_TBL_QUEUENUM_BITS))
		return -EINVAL;

	qos->data &= ~MVPP2_CLS2_QOS_TBL_QUEUENUM_MASK;
	qos->data |= (((uint32_t)queue) << MVPP2_CLS2_QOS_TBL_QUEUENUM_OFF);
	return 0;
}

int mv_pp2x_cls_c2_qos_prio_get(struct mv_pp2x_cls_c2_qos_entry *qos, int *prio)
{
	if (mv_pp2x_ptr_validate(qos) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_ptr_validate(prio) == MV_ERROR)
		return MV_ERROR;

	*prio = (qos->data & MVPP2_CLS2_QOS_TBL_PRI_MASK) >> MVPP2_CLS2_QOS_TBL_PRI_OFF;
	return MV_OK;
}

int mv_pp2x_cls_c2_qos_dscp_get(struct mv_pp2x_cls_c2_qos_entry *qos, int *dscp)
{
	if (mv_pp2x_ptr_validate(qos) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_ptr_validate(dscp) == MV_ERROR)
		return MV_ERROR;

	*dscp = (qos->data & MVPP2_CLS2_QOS_TBL_DSCP_MASK) >> MVPP2_CLS2_QOS_TBL_DSCP_OFF;
	return MV_OK;
}

int mv_pp2x_cls_c2_qos_color_get(struct mv_pp2x_cls_c2_qos_entry *qos, int *color)
{
	if (mv_pp2x_ptr_validate(qos) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_ptr_validate(color) == MV_ERROR)
		return MV_ERROR;

	*color = (qos->data & MVPP2_CLS2_QOS_TBL_COLOR_MASK) >> MVPP2_CLS2_QOS_TBL_COLOR_OFF;
	return MV_OK;
}

int mv_pp2x_cls_c2_qos_gpid_get(struct mv_pp2x_cls_c2_qos_entry *qos, int *gpid)
{
	if (mv_pp2x_ptr_validate(qos) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_ptr_validate(gpid) == MV_ERROR)
		return MV_ERROR;

	*gpid = (qos->data & MVPP2_CLS2_QOS_TBL_GEMPORT_MASK) >> MVPP2_CLS2_QOS_TBL_GEMPORT_OFF;
	return MV_OK;
}

int mv_pp2x_cls_c2_qos_queue_get(struct mv_pp2x_cls_c2_qos_entry *qos, int *queue)
{
	if (mv_pp2x_ptr_validate(qos) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_ptr_validate(queue) == MV_ERROR)
		return MV_ERROR;

	*queue = (qos->data & MVPP2_CLS2_QOS_TBL_QUEUENUM_MASK) >> MVPP2_CLS2_QOS_TBL_QUEUENUM_OFF;
	return MV_OK;
}

/* Fill the qos table with queue array supplied by user */
void mv_pp2x_cls_c2_qos_tbl_fill_array(struct pp2_port *port,
				       u8 tbl_sel, uint8_t cos_values[],
				       uint8_t start_queue)
{
	struct mv_pp2x_cls_c2_qos_entry qos_entry;
	u32 pri, line_num;
	u8 queue;

	if (tbl_sel == MVPP2_QOS_TBL_SEL_PRI)
		line_num = MVPP2_QOS_TBL_LINE_NUM_PRI;
	else
		line_num = MVPP2_QOS_TBL_LINE_NUM_DSCP;

	memset(&qos_entry, 0, sizeof(struct mv_pp2x_cls_c2_qos_entry));
	if (port->type == PP2_PPIO_T_LOG)
		qos_entry.tbl_id = QOS_LOG_PORT_TABLE_OFF(port->id);
	else
		qos_entry.tbl_id = port->id;
	qos_entry.tbl_sel = tbl_sel;

	/* Fill the QoS dscp/pbit table */
	for (pri = 0; pri < line_num; pri++) {
		qos_entry.tbl_line = pri;
		/* map cos queue to physical queue */
		/* Physical queue contains 2 parts: port ID and CPU ID,
		* CPU ID will be used in RSS
		*/
		queue = start_queue + cos_values[pri];

		pr_debug("%d start_queue %d, queue %d\n", pri, start_queue, queue);

		mv_pp2x_cls_c2_qos_queue_set(&qos_entry, queue);
		mv_pp2x_cls_c2_qos_hw_write(&port->parent->hw, &qos_entry);
	}
}

u8 mv_pp2x_cosval_queue_map(struct pp2_port *port, uint8_t cos_value)
{
	int cos_width, cos_mask;

	cos_width = ilog2(roundup_pow_of_two(port->num_tcs));
	cos_mask = (1 << cos_width) - 1;
	return ((port->parent->pp2_cfg.cos_cfg.pri_map >> (cos_value * 4)) & cos_mask);
}

/* CoS API */

/* mv_pp2x_cos_classifier_set
*  -- The API supplies interface to config cos classifier:
*     0: cos based on vlan pri;
*     1: cos based on dscp;
*     2: cos based on vlan for tagged packets,
*		and based on dscp for untagged IP packets;
*     3: cos based on dscp for IP packets, and based on vlan for non-IP packets;
*/
/* Fill the qos table with queue */
void mv_pp2x_cls_c2_qos_tbl_fill(struct pp2_port *port,
				 u8 tbl_sel, uint8_t start_queue)
{
	struct mv_pp2x_cls_c2_qos_entry qos_entry;
	u32 pri, line_num;
	u8 queue;

	if (tbl_sel == MVPP2_QOS_TBL_SEL_PRI)
		line_num = MVPP2_QOS_TBL_LINE_NUM_PRI;
	else
		line_num = MVPP2_QOS_TBL_LINE_NUM_DSCP;

	memset(&qos_entry, 0, sizeof(struct mv_pp2x_cls_c2_qos_entry));
	qos_entry.tbl_id = QOS_LOG_PORT_TABLE_OFF(port->id);
	qos_entry.tbl_sel = tbl_sel;

	/* Fill the QoS dscp/pbit table */
	for (pri = 0; pri < line_num; pri++) {
		qos_entry.tbl_line = pri;
		queue = start_queue;
		mv_pp2x_cls_c2_qos_queue_set(&qos_entry, queue);
		mv_pp2x_cls_c2_qos_hw_write(&port->parent->hw, &qos_entry);
	}
}

int mv_pp2x_cls_c2_qos_tbl_set(struct mv_pp2x_cls_c2_entry *c2,
			       int tbl_id, int tbl_sel)
{
	if (!c2 || tbl_sel > 1)
		return -EINVAL;

	if (tbl_sel == 1) {
		/*dscp */
		if (tbl_id >= MVPP2_QOS_TBL_NUM_DSCP)
			return -EINVAL;
	} else {
		/*pri */
		if (tbl_id >= MVPP2_QOS_TBL_NUM_PRI)
			return -EINVAL;
	}
	c2->sram.regs.action_tbl = (tbl_id <<
				    MVPP2_CLS2_ACT_DATA_TBL_ID_OFF) |
	    (tbl_sel << MVPP2_CLS2_ACT_DATA_TBL_SEL_OFF);

	return 0;
}

int mv_pp2x_cls_c2_color_set(struct mv_pp2x_cls_c2_entry *c2, int cmd, int from)
{
	if (!c2 || cmd > MVPP2_COLOR_ACTION_TYPE_RED_LOCK)
		return -EINVAL;

	c2->sram.regs.actions &= ~MVPP2_CLS2_ACT_COLOR_MASK;
	c2->sram.regs.actions |= (cmd << MVPP2_CLS2_ACT_COLOR_OFF);

	if (from == 1)
		c2->sram.regs.action_tbl |=
		    (1 << MVPP2_CLS2_ACT_DATA_TBL_COLOR_OFF);
	else
		c2->sram.regs.action_tbl &=
		    ~(1 << MVPP2_CLS2_ACT_DATA_TBL_COLOR_OFF);

	return 0;
}

int mv_pp2x_cls_c2_prio_set(struct mv_pp2x_cls_c2_entry *c2, int cmd,
			    int prio, int from)
{
	if (!c2 || cmd > MVPP2_ACTION_TYPE_UPDT_LOCK ||
	    prio >= MVPP2_QOS_TBL_LINE_NUM_PRI)
		return -EINVAL;

	/*set command */
	c2->sram.regs.actions &= ~MVPP2_CLS2_ACT_PRI_MASK;
	c2->sram.regs.actions |= (cmd << MVPP2_CLS2_ACT_PRI_OFF);

	/*set modify priority value */
	c2->sram.regs.qos_attr &= ~MVPP2_CLS2_ACT_QOS_ATTR_PRI_MASK;
	c2->sram.regs.qos_attr |= ((prio << MVPP2_CLS2_ACT_QOS_ATTR_PRI_OFF) &
				   MVPP2_CLS2_ACT_QOS_ATTR_PRI_MASK);

	if (from == 1)
		c2->sram.regs.action_tbl |=
		    (1 << MVPP2_CLS2_ACT_DATA_TBL_PRI_DSCP_OFF);
	else
		c2->sram.regs.action_tbl &=
		    ~(1 << MVPP2_CLS2_ACT_DATA_TBL_PRI_DSCP_OFF);

	return 0;
}

int mv_pp2x_cls_c2_dscp_set(struct mv_pp2x_cls_c2_entry *c2,
			    int cmd, int dscp, int from)
{
	if (!c2 || cmd > MVPP2_ACTION_TYPE_UPDT_LOCK ||
	    dscp >= MVPP2_QOS_TBL_LINE_NUM_DSCP)
		return -EINVAL;

	/*set command */
	c2->sram.regs.actions &= ~MVPP2_CLS2_ACT_DSCP_MASK;
	c2->sram.regs.actions |= (cmd << MVPP2_CLS2_ACT_DSCP_OFF);

	/*set modify DSCP value */
	c2->sram.regs.qos_attr &= ~MVPP2_CLS2_ACT_QOS_ATTR_DSCP_MASK;
	c2->sram.regs.qos_attr |= ((dscp <<
				    MVPP2_CLS2_ACT_QOS_ATTR_DSCP_OFF) &
				   MVPP2_CLS2_ACT_QOS_ATTR_DSCP_MASK);

	if (from == 1)
		c2->sram.regs.action_tbl |=
		    (1 << MVPP2_CLS2_ACT_DATA_TBL_PRI_DSCP_OFF);
	else
		c2->sram.regs.action_tbl &=
		    ~(1 << MVPP2_CLS2_ACT_DATA_TBL_PRI_DSCP_OFF);

	return 0;
}

int mv_pp2x_cls_c2_gpid_set(struct mv_pp2x_cls_c2_entry *c2, int cmd, int gpid, int from)
{
	if (mv_pp2x_ptr_validate(c2) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_range_validate(cmd, 0,
				   MVPP2_ACTION_TYPE_UPDT_LOCK) == MV_ERROR)
		return MV_ERROR;
	if (mv_pp2x_range_validate(gpid, 0,
				   MVPP2_CLS2_ACT_QOS_ATTR_GEM_MAX) == MV_ERROR)
		return MV_ERROR;
	/*set command*/
	c2->sram.regs.actions &= ~MVPP2_CLS2_ACT_GEM_MASK;
	c2->sram.regs.actions |= (cmd << MVPP2_CLS2_ACT_GEM_OFF);

	/*set modify DSCP value*/
	c2->sram.regs.qos_attr &= ~MVPP2_CLS2_ACT_QOS_ATTR_GEM_MASK;
	c2->sram.regs.qos_attr |= (gpid << MVPP2_CLS2_ACT_QOS_ATTR_GEM_OFF);

	if (from == 1)
		c2->sram.regs.action_tbl |= (1 << MVPP2_CLS2_ACT_DATA_TBL_GEM_ID_OFF);
	else
		c2->sram.regs.action_tbl &= ~(1 << MVPP2_CLS2_ACT_DATA_TBL_GEM_ID_OFF);

	return 0;
}

int mv_pp2x_cls_c2_queue_low_set(struct mv_pp2x_cls_c2_entry *c2,
				 int cmd, int queue, int from)
{
	if (!c2 || cmd > MVPP2_ACTION_TYPE_UPDT_LOCK ||
	    queue >= (1 << MVPP2_CLS2_ACT_QOS_ATTR_QL_BITS))
		return -EINVAL;

	/*set command */
	c2->sram.regs.actions &= ~MVPP2_CLS2_ACT_QL_MASK;
	c2->sram.regs.actions |= (cmd << MVPP2_CLS2_ACT_QL_OFF);

	/*set modify Low queue value */
	c2->sram.regs.qos_attr &= ~MVPP2_CLS2_ACT_QOS_ATTR_QL_MASK;
	c2->sram.regs.qos_attr |= ((queue <<
				    MVPP2_CLS2_ACT_QOS_ATTR_QL_OFF) &
				   MVPP2_CLS2_ACT_QOS_ATTR_QL_MASK);

	if (from == 1)
		c2->sram.regs.action_tbl |=
		    (1 << MVPP2_CLS2_ACT_DATA_TBL_LOW_Q_OFF);
	else
		c2->sram.regs.action_tbl &=
		    ~(1 << MVPP2_CLS2_ACT_DATA_TBL_LOW_Q_OFF);

	return 0;
}

int mv_pp2x_cls_c2_queue_high_set(struct mv_pp2x_cls_c2_entry *c2,
				  int cmd, int queue, int from)
{
	if (!c2 || cmd > MVPP2_ACTION_TYPE_UPDT_LOCK ||
	    queue >= (1 << MVPP2_CLS2_ACT_QOS_ATTR_QH_BITS))
		return -EINVAL;

	/*set command */
	c2->sram.regs.actions &= ~MVPP2_CLS2_ACT_QH_MASK;
	c2->sram.regs.actions |= (cmd << MVPP2_CLS2_ACT_QH_OFF);

	/*set modify High queue value */
	c2->sram.regs.qos_attr &= ~MVPP2_CLS2_ACT_QOS_ATTR_QH_MASK;
	c2->sram.regs.qos_attr |= ((queue <<
				    MVPP2_CLS2_ACT_QOS_ATTR_QH_OFF) &
				   MVPP2_CLS2_ACT_QOS_ATTR_QH_MASK);

	if (from == 1)
		c2->sram.regs.action_tbl |=
		    (1 << MVPP2_CLS2_ACT_DATA_TBL_HIGH_Q_OFF);
	else
		c2->sram.regs.action_tbl &=
		    ~(1 << MVPP2_CLS2_ACT_DATA_TBL_HIGH_Q_OFF);

	return 0;
}

int mv_pp2x_cls_c2_forward_set(struct mv_pp2x_cls_c2_entry *c2, int cmd)
{
	if (!c2 || cmd > MVPP2_ACTION_TYPE_UPDT_LOCK)
		return -EINVAL;

	c2->sram.regs.actions &= ~MVPP2_CLS2_ACT_FRWD_MASK;
	c2->sram.regs.actions |= (cmd << MVPP2_CLS2_ACT_FRWD_OFF);

	return 0;
}

int mv_pp2x_cls_c2_policer_set(struct mv_pp2x_cls_c2_entry *c2, int cmd, int policer_id, int bank)
{
	if (mv_pp2x_ptr_validate(c2) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_range_validate(cmd, 0,
				   MVPP2_ACTION_TYPE_UPDT_LOCK) == MV_ERROR)
		return MV_ERROR;
	if (mv_pp2x_range_validate(policer_id, 0,
				   MVPP2_CLS2_ACT_DUP_ATTR_PLCRID_MAX) == MV_ERROR)
		return MV_ERROR;
	c2->sram.regs.actions &= ~MVPP2_CLS2_ACT_PLCR_MASK;
	c2->sram.regs.actions |= (cmd << MVPP2_CLS2_ACT_PLCR_OFF);

	c2->sram.regs.rss_attr &= ~MVPP2_CLS2_ACT_DUP_ATTR_PLCRID_MASK;
	c2->sram.regs.rss_attr |= (policer_id << MVPP2_CLS2_ACT_DUP_ATTR_PLCRID_OFF);

	if (bank)
		c2->sram.regs.rss_attr |= MVPP2_CLS2_ACT_DUP_ATTR_PLCRBK_MASK;
	else
		c2->sram.regs.rss_attr &= ~MVPP2_CLS2_ACT_DUP_ATTR_PLCRBK_MASK;

	return 0;
}

int mv_pp2x_cls_c2_rss_set(struct mv_pp2x_cls_c2_entry *c2, int cmd, int rss_en)
{
	if (!c2 || cmd > MVPP2_ACTION_TYPE_UPDT_LOCK || rss_en >=
	    (1 << MVPP2_CLS2_ACT_DUP_ATTR_RSSEN_BITS))
		return -EINVAL;

	c2->sram.regs.actions &= ~MVPP2_CLS2_ACT_RSS_MASK;
	c2->sram.regs.actions |= (cmd << MVPP2_CLS2_ACT_RSS_OFF);

	c2->sram.regs.rss_attr &= ~MVPP2_CLS2_ACT_DUP_ATTR_RSSEN_MASK;
	c2->sram.regs.rss_attr |= (rss_en << MVPP2_CLS2_ACT_DUP_ATTR_RSSEN_OFF);

	return 0;
}

int mv_pp2x_cls_c2_flow_id_en(struct mv_pp2x_cls_c2_entry *c2, int flow_id_en)
{
	if (!c2)
		return -EINVAL;

	/*set Flow ID enable or disable */
	if (flow_id_en)
		c2->sram.regs.actions |= (1 << MVPP2_CLS2_ACT_FLD_EN_OFF);
	else
		c2->sram.regs.actions &= ~(1 << MVPP2_CLS2_ACT_FLD_EN_OFF);

	return 0;
}

int mv_pp2x_cls_c2_mod_set(struct mv_pp2x_cls_c2_entry *c2, int data_ptr, int instr_offs, int l4_csum)
{
	if (mv_pp2x_ptr_validate(c2) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_range_validate(data_ptr, 0,
				   MVPP2_CLS2_ACT_HWF_ATTR_DPTR_MAX) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_range_validate(instr_offs, 0,
				   MVPP2_CLS2_ACT_HWF_ATTR_IPTR_MAX) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_range_validate(l4_csum, 0, 1) == MV_ERROR)
		return MV_ERROR;

	c2->sram.regs.hwf_attr &= ~MVPP2_CLS2_ACT_HWF_ATTR_DPTR_MASK;
	c2->sram.regs.hwf_attr &= ~MVPP2_CLS2_ACT_HWF_ATTR_IPTR_MASK;
	c2->sram.regs.hwf_attr &= ~MVPP2_CLS2_ACT_HWF_ATTR_L4CHK_MASK;

	c2->sram.regs.hwf_attr |= (data_ptr << MVPP2_CLS2_ACT_HWF_ATTR_DPTR_OFF);
	c2->sram.regs.hwf_attr |= (instr_offs << MVPP2_CLS2_ACT_HWF_ATTR_IPTR_OFF);
	c2->sram.regs.hwf_attr |= (l4_csum << MVPP2_CLS2_ACT_HWF_ATTR_L4CHK_OFF);

	return 0;
}


int mv_pp2x_cls_c2_dup_set(struct mv_pp2x_cls_c2_entry *c2, int dupid, int count)
{
	if (mv_pp2x_ptr_validate(c2) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_range_validate(count, 0,
				   MVPP2_CLS2_ACT_DUP_ATTR_DUPCNT_MAX) == MV_ERROR)
		return MV_ERROR;
	if (mv_pp2x_range_validate(dupid, 0,
				   MVPP2_CLS2_ACT_DUP_ATTR_DUPID_MAX) == MV_ERROR)
		return MV_ERROR;

	/*set flowid and count*/
	c2->sram.regs.rss_attr &= ~(MVPP2_CLS2_ACT_DUP_ATTR_DUPID_MASK | MVPP2_CLS2_ACT_DUP_ATTR_DUPCNT_MASK);
	c2->sram.regs.rss_attr |= (dupid << MVPP2_CLS2_ACT_DUP_ATTR_DUPID_OFF);
	c2->sram.regs.rss_attr |= (count << MVPP2_CLS2_ACT_DUP_ATTR_DUPCNT_OFF);

	return 0;
}

int mv_pp2x_cls_c2_seq_set(struct mv_pp2x_cls_c2_entry *c2, int miss, int id)
{
	if (mv_pp2x_ptr_validate(c2) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_range_validate(miss, 0,
				   1) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_range_validate(id, 0,
				   MVPP21_CLS2_ACT_SEQ_ATTR_ID_MAX) == MV_ERROR)
		return MV_ERROR;

	c2->sram.regs.seq_attr = 0;
	c2->sram.regs.seq_attr = ((id << MVPP21_CLS2_ACT_SEQ_ATTR_ID) | (miss << MVPP21_CLS2_ACT_SEQ_ATTR_MISS_OFF));

	return 0;
}

int mv_pp2x_cls_c2_tcam_byte_set(struct mv_pp2x_cls_c2_entry *c2,
				 unsigned int offs, unsigned char byte,
				 unsigned char enable)
{
	if (!c2 || offs >= MVPP2_CLS_C2_TCAM_DATA_BYTES)
		return -EINVAL;

	c2->tcam.bytes[TCAM_DATA_BYTE(offs)] = byte;
	c2->tcam.bytes[TCAM_DATA_MASK(offs)] = enable;

	return 0;
}

int mv_pp2x_cls_c2_tcam_byte_get(struct mv_pp2x_cls_c2_entry *c2,
				unsigned int offs, unsigned char *byte,
				unsigned char *enable)
{
	if (mv_pp2x_ptr_validate(c2) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_ptr_validate(byte) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_ptr_validate(enable) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_range_validate(offs, 0, 8) == MV_ERROR)
		return MV_ERROR;

	*byte = c2->tcam.bytes[TCAM_DATA_BYTE(offs)];
	*enable = c2->tcam.bytes[TCAM_DATA_MASK(offs)];

	return 0;
}

u8 pp2_cls_c2_tcam_port_get(struct mv_pp2x_cls_c2_entry *c2)
{
	return ((c2->tcam.words[4] >> 8) & 0xFF);
}

/* C2 rule and Qos table */
int mv_pp2x_cls_c2_hw_write(uintptr_t cpu_slot, int index,
			    struct mv_pp2x_cls_c2_entry *c2)
{
	int tcm_idx;

	if (!c2 || index >= MVPP2_CLS_C2_TCAM_SIZE)
		return -EINVAL;

	c2->index = index;

	/* write index reg */
	pp2_reg_write(cpu_slot, MVPP2_CLS2_TCAM_IDX_REG, index);

	/* write valid bit */
	c2->inv = 0;
	pp2_reg_write(cpu_slot, MVPP2_CLS2_TCAM_INV_REG,
		     ((c2->inv) << MVPP2_CLS2_TCAM_INV_INVALID_OFF));

	for (tcm_idx = 0; tcm_idx < MVPP2_CLS_C2_TCAM_WORDS; tcm_idx++)
		pp2_reg_write(cpu_slot, MVPP2_CLS2_TCAM_DATA_REG(tcm_idx),
			     c2->tcam.words[tcm_idx]);

	/* write action_tbl CLSC2_ACT_DATA */
	pp2_reg_write(cpu_slot, MVPP2_CLS2_ACT_DATA_REG,
		     c2->sram.regs.action_tbl);

	/* write actions CLSC2_ACT */
	pp2_reg_write(cpu_slot, MVPP2_CLS2_ACT_REG, c2->sram.regs.actions);

	/* write qos_attr CLSC2_ATTR0 */
	pp2_reg_write(cpu_slot, MVPP2_CLS2_ACT_QOS_ATTR_REG,
		     c2->sram.regs.qos_attr);

	/* write hwf_attr CLSC2_ATTR1 */
	pp2_reg_write(cpu_slot, MVPP2_CLS2_ACT_HWF_ATTR_REG,
		     c2->sram.regs.hwf_attr);

	/* write rss_attr CLSC2_ATTR2 */
	pp2_reg_write(cpu_slot, MVPP2_CLS2_ACT_DUP_ATTR_REG,
		     c2->sram.regs.rss_attr);

	return 0;
}

/*
 * note: error is not returned if entry is invalid
 * user should check c2->valid afer returned from this func
 */
int mv_pp2x_cls_c2_hw_read(uintptr_t cpu_slot, int index, struct mv_pp2x_cls_c2_entry *c2)
{
	unsigned int reg_val = 0;
	int tcm_idx;

	if (mv_pp2x_ptr_validate(c2) == MV_ERROR)
		return MV_ERROR;

	c2->index = index;

	/* write index reg */
	pp2_reg_write(cpu_slot, MVPP2_CLS2_TCAM_IDX_REG, index);

	/* read invalid bit */
	reg_val = pp2_reg_read(cpu_slot, MVPP2_CLS2_TCAM_INV_REG);

	c2->inv = (reg_val & MVPP2_CLS2_TCAM_INV_INVALID_MASK) >> MVPP2_CLS2_TCAM_INV_INVALID_OFF;

	if (c2->inv)
		return 0;

	for (tcm_idx = 0; tcm_idx < MVPP2_CLS_C2_TCAM_WORDS; tcm_idx++)
		c2->tcam.words[tcm_idx] = pp2_reg_read(cpu_slot, MVPP2_CLS2_TCAM_DATA_REG(tcm_idx));

	c2->sram.regs.action_tbl = pp2_reg_read(cpu_slot, MVPP2_CLS2_ACT_DATA_REG);
	c2->sram.regs.actions = pp2_reg_read(cpu_slot, MVPP2_CLS2_ACT_REG);
	c2->sram.regs.qos_attr = pp2_reg_read(cpu_slot, MVPP2_CLS2_ACT_QOS_ATTR_REG);
	c2->sram.regs.hwf_attr = pp2_reg_read(cpu_slot, MVPP2_CLS2_ACT_HWF_ATTR_REG);
	c2->sram.regs.rss_attr = pp2_reg_read(cpu_slot, MVPP2_CLS2_ACT_DUP_ATTR_REG);
	c2->sram.regs.seq_attr = pp2_reg_read(cpu_slot, MVPP21_CLS2_ACT_SEQ_ATTR_REG);

	return 0;
}

static int mv_pp2x_c2_tcam_set(uintptr_t cpu_slot,
			       struct mv_pp2x_c2_add_entry *c2_add_entry,
			       unsigned int c2_hw_idx)
{
	int ret_code;
	struct mv_pp2x_cls_c2_entry c2_entry;
	int hek_offs;
	unsigned char hek_byte[MVPP2_CLS_C2_HEK_OFF_MAX],
	    hek_byte_mask[MVPP2_CLS_C2_HEK_OFF_MAX];

	if (!c2_add_entry || !cpu_slot || c2_hw_idx >= MVPP2_CLS_C2_TCAM_SIZE)
		return -EINVAL;

	/* Clear C2 sw data */
	memset(&c2_entry, 0, sizeof(struct mv_pp2x_cls_c2_entry));

	/* Set QOS table, selection and ID */
	ret_code = mv_pp2x_cls_c2_qos_tbl_set(&c2_entry,
					      c2_add_entry->qos_info.
					      qos_tbl_index,
					      c2_add_entry->qos_info.
					      qos_tbl_type);
	if (ret_code)
		return ret_code;

	/* Set color, cmd and source */
	ret_code = mv_pp2x_cls_c2_color_set(&c2_entry,
					    c2_add_entry->action.color_act,
					    c2_add_entry->qos_info.color_src);
	if (ret_code)
		return ret_code;

	/* Set priority(pbit), cmd, value(not from qos table) and source */
	ret_code = mv_pp2x_cls_c2_prio_set(&c2_entry,
					   c2_add_entry->action.pri_act,
					   c2_add_entry->qos_value.pri,
					   c2_add_entry->qos_info.pri_dscp_src);
	if (ret_code)
		return ret_code;

	/* Set DSCP, cmd, value(not from qos table) and source */
	ret_code = mv_pp2x_cls_c2_dscp_set(&c2_entry,
					   c2_add_entry->action.dscp_act,
					   c2_add_entry->qos_value.dscp,
					   c2_add_entry->qos_info.pri_dscp_src);
	if (ret_code)
		return ret_code;

	/* Set queue low, cmd, value, and source */
	ret_code = mv_pp2x_cls_c2_queue_low_set(&c2_entry,
						c2_add_entry->action.q_low_act,
						c2_add_entry->qos_value.q_low,
						c2_add_entry->qos_info.
						q_low_src);
	if (ret_code)
		return ret_code;

	/* Set queue high, cmd, value and source */
	ret_code = mv_pp2x_cls_c2_queue_high_set(&c2_entry,
						 c2_add_entry->action.
						 q_high_act,
						 c2_add_entry->qos_value.q_high,
						 c2_add_entry->qos_info.
						 q_high_src);
	if (ret_code)
		return ret_code;

	/* Set forward */
	ret_code = mv_pp2x_cls_c2_forward_set(&c2_entry,
					      c2_add_entry->action.frwd_act);
	if (ret_code)
		return ret_code;

	/* Set RSS */
	ret_code = mv_pp2x_cls_c2_rss_set(&c2_entry,
					  c2_add_entry->action.rss_act,
					  c2_add_entry->rss_en);
	if (ret_code)
		return ret_code;

	/* Set flow_id(not for multicast) */
	ret_code = mv_pp2x_cls_c2_flow_id_en(&c2_entry,
					    c2_add_entry->action.flowid_act);
	if (ret_code)
		return ret_code;

	/* Set C2 HEK */
	memset(hek_byte, 0, MVPP2_CLS_C2_HEK_OFF_MAX);
	memset(hek_byte_mask, 0, MVPP2_CLS_C2_HEK_OFF_MAX);

	/* HEK offs 8, lookup type, port type */
	hek_byte[MVPP2_CLS_C2_HEK_OFF_LKP_PORT_TYPE] =
	    (c2_add_entry->port.port_type <<
	     MVPP2_CLS_C2_HEK_PORT_TYPE_OFFS) |
	    (c2_add_entry->lkp_type << MVPP2_CLS_C2_HEK_LKP_TYPE_OFFS);
	hek_byte_mask[MVPP2_CLS_C2_HEK_OFF_LKP_PORT_TYPE] =
	    MVPP2_CLS_C2_HEK_PORT_TYPE_MASK |
	    ((c2_add_entry->lkp_type_mask <<
	      MVPP2_CLS_C2_HEK_LKP_TYPE_OFFS) & MVPP2_CLS_C2_HEK_LKP_TYPE_MASK);
	/* HEK offs 9, port ID */
	hek_byte[MVPP2_CLS_C2_HEK_OFF_PORT_ID] = c2_add_entry->port.port_value;
	hek_byte_mask[MVPP2_CLS_C2_HEK_OFF_PORT_ID] =
	    c2_add_entry->port.port_mask;

	for (hek_offs = MVPP2_CLS_C2_HEK_OFF_PORT_ID; hek_offs >=
	     MVPP2_CLS_C2_HEK_OFF_BYTE0; hek_offs--) {
		ret_code = mv_pp2x_cls_c2_tcam_byte_set(&c2_entry,
							hek_offs,
							hek_byte[hek_offs],
							hek_byte_mask
							[hek_offs]);
		if (ret_code)
			return ret_code;
	}

	/* Write C2 entry data to HW */
	ret_code = mv_pp2x_cls_c2_hw_write(cpu_slot, c2_hw_idx, &c2_entry);
	if (ret_code)
		return ret_code;

	return 0;
}

static int mv_pp2x_c2_rule_add(struct pp2_port *port,
			       struct mv_pp2x_c2_add_entry *c2_add_entry)
{
	int ret, lkp_type, c2_index = 0;
	bool first_free_update = false;
	struct mv_pp2x_c2_rule_idx *rule_idx;
	uintptr_t cpu_slot;
	struct pp2_hw *hw;

	rule_idx = &port->parent->hw.c2_shadow->rule_idx_info[port->id];

	if (!port || !c2_add_entry)
		return -EINVAL;

	lkp_type = c2_add_entry->lkp_type;
	/* Write rule in C2 TCAM */
	if (lkp_type == MVPP2_CLS_LKP_VLAN_PRI) {
		if (rule_idx->vlan_pri_idx == MVPP2_CLS_C2_TCAM_SIZE) {
			/* If the C2 rule is new, apply a free c2 rule index */
			c2_index = port->parent->hw.c2_shadow->c2_tcam_free_start;
			first_free_update = true;
		} else {
			/* If the C2 rule is exist one,
			 * take the C2 index from shadow
			 */
			c2_index = rule_idx->vlan_pri_idx;
			first_free_update = false;
		}
	} else if (lkp_type == MVPP2_CLS_LKP_DSCP_PRI) {
		if (rule_idx->dscp_pri_idx == MVPP2_CLS_C2_TCAM_SIZE) {
			c2_index = port->parent->hw.c2_shadow->c2_tcam_free_start;
			first_free_update = true;
		} else {
			c2_index = rule_idx->dscp_pri_idx;
			first_free_update = false;
		}
	} else if (lkp_type == MVPP2_CLS_LKP_DEFAULT) {
		if (rule_idx->default_rule_idx == MVPP2_CLS_C2_TCAM_SIZE) {
			c2_index = port->parent->hw.c2_shadow->c2_tcam_free_start;
			first_free_update = true;
		} else {
			c2_index = rule_idx->default_rule_idx;
			first_free_update = false;
		}
	} else {
		return -EINVAL;
	}

	hw = &port->parent->hw;
	cpu_slot = hw->base[PP2_DEFAULT_REGSPACE].va;

	/* Write C2 TCAM HW */
	ret = mv_pp2x_c2_tcam_set(cpu_slot, c2_add_entry, c2_index);
	if (ret)
		return ret;

	/* Update first free rule */
	if (first_free_update)
		port->parent->hw.c2_shadow->c2_tcam_free_start++;

	/* Update shadow */
	if (lkp_type == MVPP2_CLS_LKP_VLAN_PRI)
		rule_idx->vlan_pri_idx = c2_index;
	else if (lkp_type == MVPP2_CLS_LKP_DSCP_PRI)
		rule_idx->dscp_pri_idx = c2_index;
	else if (lkp_type == MVPP2_CLS_LKP_DEFAULT)
		rule_idx->default_rule_idx = c2_index;

	return 0;
}


int mv_pp2x_c2_hit_cntr_read(uintptr_t cpu_slot, int index, u32 *cntr)
{
	u32 value = 0;

	/* write index reg */
	pp2_reg_write(cpu_slot, MVPP2_CLS2_TCAM_IDX_REG, index);

	value = pp2_reg_read(cpu_slot, MVPP2_CLS2_HIT_CTR_REG);

	if (cntr)
		*cntr = value;
	else
		pr_info("INDEX: 0x%8.8X	VAL: 0x%8.8X\n", index, value);

	return MV_OK;
}

void mv_pp2x_c2_sw_clear(struct mv_pp2x_cls_c2_entry *c2)
{
	memset(c2, 0, sizeof(struct mv_pp2x_cls_c2_entry));
}

int mv_pp2x_c2_sw_words_dump(struct mv_pp2x_cls_c2_entry *c2)
{
	int i;

	if (mv_pp2x_ptr_validate(c2) == MV_ERROR)
		return MV_ERROR;

	/* TODO check size */
	/* hw entry id */
	printf("[0x%3.3x] ", c2->index);

	i = MVPP2_CLS_C2_TCAM_WORDS - 1;

	while (i >= 0)
		printf("%4.4x ", (c2->tcam.words[i--]) & 0xFFFF);

	/* tcam inValid bit */
	printf("\n%s\n\t", (c2->inv == 1) ? "[inv]" : "[valid]");

	i = MVPP2_CLS_C2_TCAM_WORDS - 1;

	while (i >= 0)
		printf("%4.4x ", ((c2->tcam.words[i--] >> 16)  & 0xFFFF));

	printf("\n");

	return 0;
}

int mv_pp2x_c2_sw_dump(struct mv_pp2x_cls_c2_entry *c2)
{
	int id, sel, type, gemid, low_q, high_q, color, value;

	if (mv_pp2x_ptr_validate(c2) == MV_ERROR)
		return MV_ERROR;

	mv_pp2x_c2_sw_words_dump(c2);

	printf("\n");

	/*------------------------------*/
	/*	action_tbl 0x1B30	*/
	/*------------------------------*/

	id = ((c2->sram.regs.action_tbl & (MVPP2_CLS2_ACT_DATA_TBL_ID_MASK)) >>
	       MVPP2_CLS2_ACT_DATA_TBL_ID_OFF);
	sel = ((c2->sram.regs.action_tbl & (MVPP2_CLS2_ACT_DATA_TBL_SEL_MASK)) >>
		MVPP2_CLS2_ACT_DATA_TBL_SEL_OFF);
	type = ((c2->sram.regs.action_tbl & (MVPP2_CLS2_ACT_DATA_TBL_PRI_DSCP_MASK)) >>
		 MVPP2_CLS2_ACT_DATA_TBL_PRI_DSCP_OFF);
	gemid = ((c2->sram.regs.action_tbl & (MVPP2_CLS2_ACT_DATA_TBL_GEM_ID_MASK)) >>
		  MVPP2_CLS2_ACT_DATA_TBL_GEM_ID_OFF);
	low_q = ((c2->sram.regs.action_tbl & (MVPP2_CLS2_ACT_DATA_TBL_LOW_Q_MASK)) >>
		  MVPP2_CLS2_ACT_DATA_TBL_LOW_Q_OFF);
	high_q = ((c2->sram.regs.action_tbl & (MVPP2_CLS2_ACT_DATA_TBL_HIGH_Q_MASK)) >>
		   MVPP2_CLS2_ACT_DATA_TBL_HIGH_Q_OFF);
	color = ((c2->sram.regs.action_tbl & (MVPP2_CLS2_ACT_DATA_TBL_COLOR_MASK)) >>
		  MVPP2_CLS2_ACT_DATA_TBL_COLOR_OFF);

	printf("FROM_QOS_%s_TBL[%2.2d]:  ", sel ? "DSCP" : "PRI", id);
	type ? printf("%s	", sel ? "DSCP" : "PRIO") : 0;
	color ? printf("COLOR	") : 0;
	gemid ? printf("GEMID	") : 0;
	low_q ? printf("LOW_Q	") : 0;
	high_q ? printf("HIGH_Q	") : 0;
	printf("\n");

	printf("FROM_ACT_TBL:		");
	(type == 0) ? printf("%s	", sel ? "DSCP" : "PRI") : 0;
	(gemid == 0) ? printf("GEMID	") : 0;
	(low_q == 0) ? printf("LOW_Q	") : 0;
	(high_q == 0) ? printf("HIGH_Q	") : 0;
	(color == 0) ? printf("COLOR	") : 0;
	printf("\n\n");

	/*------------------------------*/
	/*	actions 0x1B60		*/
	/*------------------------------*/

	printf("ACT_CMD:		COLOR	PRIO	DSCP	GEMID	LOW_Q	HIGH_Q	FWD	POLICER	FID	RSS\n");
	printf("			");

	printf("%1.1d\t%1.1d\t%1.1d\t%1.1d\t%1.1d\t%1.1d\t%1.1d\t%1.1d\t%1.1d\t%1.1d\t",
	       ((c2->sram.regs.actions & MVPP2_CLS2_ACT_COLOR_MASK) >> MVPP2_CLS2_ACT_COLOR_OFF),
	       ((c2->sram.regs.actions & MVPP2_CLS2_ACT_PRI_MASK) >> MVPP2_CLS2_ACT_PRI_OFF),
	       ((c2->sram.regs.actions & MVPP2_CLS2_ACT_DSCP_MASK) >> MVPP2_CLS2_ACT_DSCP_OFF),
	       ((c2->sram.regs.actions & MVPP2_CLS2_ACT_GEM_MASK) >> MVPP2_CLS2_ACT_GEM_OFF),
	       ((c2->sram.regs.actions & MVPP2_CLS2_ACT_QL_MASK) >> MVPP2_CLS2_ACT_QL_OFF),
	       ((c2->sram.regs.actions & MVPP2_CLS2_ACT_QH_MASK) >> MVPP2_CLS2_ACT_QH_OFF),
	       ((c2->sram.regs.actions & MVPP2_CLS2_ACT_FRWD_MASK) >> MVPP2_CLS2_ACT_FRWD_OFF),
	       ((c2->sram.regs.actions & MVPP2_CLS2_ACT_PLCR_MASK) >> MVPP2_CLS2_ACT_PLCR_OFF),
	       ((c2->sram.regs.actions & MVPP2_CLS2_ACT_FLD_EN_MASK) >> MVPP2_CLS2_ACT_FLD_EN_OFF),
	       ((c2->sram.regs.actions & MVPP2_CLS2_ACT_RSS_MASK) >> MVPP2_CLS2_ACT_RSS_OFF));
	printf("\n\n");


	/*------------------------------*/
	/*	qos_attr 0x1B64		*/
	/*------------------------------*/
	printf("ACT_ATTR:		PRIO	DSCP	GEMID	LOW_Q	HIGH_Q	QUEUE\n");
	printf("		");
	/* modify priority */
	value = ((c2->sram.regs.qos_attr & MVPP2_CLS2_ACT_QOS_ATTR_PRI_MASK) >> MVPP2_CLS2_ACT_QOS_ATTR_PRI_OFF);
	printf("	%1.1d\t", value);

	/* modify dscp */
	value = ((c2->sram.regs.qos_attr & MVPP2_CLS2_ACT_QOS_ATTR_DSCP_MASK) >> MVPP2_CLS2_ACT_QOS_ATTR_DSCP_OFF);
	printf("0x%2.2x\t", value);

	/* modify gemportid */
	value = ((c2->sram.regs.qos_attr & MVPP2_CLS2_ACT_QOS_ATTR_GEM_MASK) >> MVPP2_CLS2_ACT_QOS_ATTR_GEM_OFF);
	printf("0x%4.4x\t", value);

	/* modify low Q */
	value = ((c2->sram.regs.qos_attr & MVPP2_CLS2_ACT_QOS_ATTR_QL_MASK) >> MVPP2_CLS2_ACT_QOS_ATTR_QL_OFF);
	printf("0x%1.1x\t", value);

	/* modify high Q */
	value = ((c2->sram.regs.qos_attr & MVPP2_CLS2_ACT_QOS_ATTR_QH_MASK) >> MVPP2_CLS2_ACT_QOS_ATTR_QH_OFF);
	printf("0x%2.2x\t", value);

	/*modify queue*/
	value = ((c2->sram.regs.qos_attr & (MVPP2_CLS2_ACT_QOS_ATTR_QL_MASK | MVPP2_CLS2_ACT_QOS_ATTR_QH_MASK)));
	value >>= MVPP2_CLS2_ACT_QOS_ATTR_QL_OFF;

	printf("0x%2.2x\t", value);
	printf("\n\n");

	/*------------------------------*/
	/*	hwf_attr 0x1B68		*/
	/*------------------------------*/
	printf("HWF_ATTR:		IPTR	DPTR	CHKSM   MTU_IDX\n");
	printf("			");

	/* HWF modification instraction pointer */
	value = ((c2->sram.regs.hwf_attr & MVPP2_CLS2_ACT_HWF_ATTR_DPTR_MASK) >>
		     MVPP2_CLS2_ACT_HWF_ATTR_DPTR_OFF);
	printf("0x%1.1x\t", value);

	/* HWF modification data pointer */
	value = ((c2->sram.regs.hwf_attr & MVPP2_CLS2_ACT_HWF_ATTR_IPTR_MASK) >>
		     MVPP2_CLS2_ACT_HWF_ATTR_IPTR_OFF);
	printf("0x%4.4x\t", value);

	/* HWF modification instraction pointer */
	value = ((c2->sram.regs.hwf_attr & MVPP2_CLS2_ACT_HWF_ATTR_L4CHK_MASK) >>
		     MVPP2_CLS2_ACT_HWF_ATTR_L4CHK_OFF);
	printf("%s\t", value ? "ENABLE " : "DISABLE");

	/* mtu index */
	value = ((c2->sram.regs.hwf_attr & MVPP2_CLS2_ACT_HWF_ATTR_MTUIDX_MASK) >>
		     MVPP2_CLS2_ACT_HWF_ATTR_MTUIDX_OFF);
	printf("0x%1.1x\t", value);
	printf("\n\n");

	/*------------------------------*/
	/*	rss_attr 0x1B6C		*/
	/*------------------------------*/
	printf("RSS_ATTR:		FID	COUNT	POLICER [id    bank]	RSS\n");
	printf("			0x%2.2x\t0x%1.1x\t\t[0x%2.2x   0x%1.1x]\t0x%1.1x\n",
	       ((c2->sram.regs.rss_attr & MVPP2_CLS2_ACT_DUP_ATTR_DUPID_MASK) >> MVPP2_CLS2_ACT_DUP_ATTR_DUPID_OFF),
	       ((c2->sram.regs.rss_attr & MVPP2_CLS2_ACT_DUP_ATTR_DUPCNT_MASK) >> MVPP2_CLS2_ACT_DUP_ATTR_DUPCNT_OFF),
	       ((c2->sram.regs.rss_attr & MVPP2_CLS2_ACT_DUP_ATTR_PLCRID_MASK) >> MVPP2_CLS2_ACT_DUP_ATTR_PLCRID_OFF),
	       ((c2->sram.regs.rss_attr & MVPP2_CLS2_ACT_DUP_ATTR_PLCRBK_MASK) >> MVPP2_CLS2_ACT_DUP_ATTR_PLCRBK_OFF),
	       ((c2->sram.regs.rss_attr & MVPP2_CLS2_ACT_DUP_ATTR_RSSEN_MASK) >> MVPP2_CLS2_ACT_DUP_ATTR_RSSEN_OFF));
	printf("\n");
	/*------------------------------*/
	/*	seq_attr 0x1B70		*/
	/*------------------------------*/
	/*PPv2.1 new feature MAS 3.14*/
	printf("SEQ_ATTR:		ID	MISS\n");
	printf("			0x%2.2x    0x%2.2x\n",
	      ((c2->sram.regs.seq_attr & MVPP21_CLS2_ACT_SEQ_ATTR_ID_MASK) >> MVPP21_CLS2_ACT_SEQ_ATTR_ID),
	      ((c2->sram.regs.seq_attr & MVPP21_CLS2_ACT_SEQ_ATTR_MISS_MASK) >> MVPP21_CLS2_ACT_SEQ_ATTR_MISS_OFF));

	printf("\n\n");


	return 0;
}

int mv_pp2x_c2_hw_dump(uintptr_t cpu_slot)
{
	int index;
	int c2_status;
	struct mv_pp2x_cls_c2_entry c2;

	c2_status = pp2_reg_read(cpu_slot, MVPP2_CLS2_TCAM_CTRL_REG);
	if (c2_status)
		pr_info("c2 is running\n");
	else {
		pr_info("c2 is off\n");
		return -EINVAL;
	}

	mv_pp2x_c2_sw_clear(&c2);

	for (index = 0; index < MVPP2_CLS_C2_TCAM_SIZE; index++) {
		mv_pp2x_cls_c2_hw_read(cpu_slot, index, &c2);
		if (c2.inv == 0)
			mv_pp2x_c2_sw_dump(&c2);
	}
	return 0;
}

/* RSS */
/* The function will set rss table entry */
int mv_pp22_rss_tbl_entry_set(struct pp2_hw *hw,
			      struct mv_pp22_rss_entry *rss)
{
	unsigned int reg_val = 0;

	if (!rss || rss->sel > MVPP22_RSS_ACCESS_TBL)
		return -EINVAL;

	if (rss->sel == MVPP22_RSS_ACCESS_POINTER) {
		if (rss->u.pointer.rss_tbl_ptr >= MVPP22_RSS_TBL_NUM)
			return -EINVAL;
		/* Write index */
		reg_val |= rss->u.pointer.rxq_idx << MVPP22_RSS_IDX_RXQ_NUM_OFF;
		pp2_reg_write(hw->base[0].va, MVPP22_RSS_IDX_REG, reg_val);
		pr_debug("rss queue %d, reg_val %x", rss->u.pointer.rxq_idx, reg_val);
		/* Write entry */
		reg_val = 0;
		reg_val &= (~MVPP22_RSS_RXQ2RSS_TBL_POINT_MASK);
		reg_val |= rss->u.pointer.rss_tbl_ptr <<
		    MVPP22_RSS_RXQ2RSS_TBL_POINT_OFF;
		pp2_reg_write(hw->base[0].va, MVPP22_RSS_RXQ2RSS_TBL_REG,
			      reg_val);
		pr_debug(", table %d, reg_val %x\n", rss->u.pointer.rss_tbl_ptr, reg_val);
	} else if (rss->sel == MVPP22_RSS_ACCESS_TBL) {
		if (rss->u.entry.tbl_id >= MVPP22_RSS_TBL_NUM ||
		    rss->u.entry.tbl_line >= MVPP22_RSS_TBL_LINE_NUM ||
		    rss->u.entry.width >= MVPP22_RSS_WIDTH_MAX)
			return -EINVAL;
		/* Write index */
		reg_val |= (rss->u.entry.tbl_line <<
			   MVPP22_RSS_IDX_ENTRY_NUM_OFF |
			   rss->u.entry.tbl_id << MVPP22_RSS_IDX_TBL_NUM_OFF);
		pp2_reg_write(hw->base[0].va, MVPP22_RSS_IDX_REG, reg_val);
		/* Write entry */
		reg_val &= (~MVPP22_RSS_TBL_ENTRY_MASK);
		reg_val |= (rss->u.entry.rxq << MVPP22_RSS_TBL_ENTRY_OFF);
		pp2_reg_write(hw->base[0].va, MVPP22_RSS_TBL_ENTRY_REG, reg_val);
		reg_val &= (~MVPP22_RSS_WIDTH_MASK);
		reg_val |= (rss->u.entry.width << MVPP22_RSS_WIDTH_OFF);
		pp2_reg_write(hw->base[0].va, MVPP22_RSS_WIDTH_REG, reg_val);
	}
	return 0;
}

/* The function will get rss table entry */
int pp2_rss_tbl_entry_get(struct pp2_hw *hw,
			  struct mv_pp22_rss_entry *rss)
{
	unsigned int reg_val = 0;

	if (!rss || rss->sel > MVPP22_RSS_ACCESS_TBL)
		return -EINVAL;

	if (rss->sel == MVPP22_RSS_ACCESS_POINTER) {
		/* Read entry */
		reg_val |= rss->u.pointer.rxq_idx << MVPP22_RSS_IDX_RXQ_NUM_OFF;
		pp2_reg_write(hw->base[0].va, MVPP22_RSS_IDX_REG, reg_val);
		rss->u.pointer.rss_tbl_ptr =
			pp2_reg_read(hw->base[0].va, MVPP22_RSS_RXQ2RSS_TBL_REG) &
				     MVPP22_RSS_RXQ2RSS_TBL_POINT_MASK;
	} else if (rss->sel == MVPP22_RSS_ACCESS_TBL) {
		if (rss->u.entry.tbl_id >= MVPP22_RSS_TBL_NUM ||
		    rss->u.entry.tbl_line >= MVPP22_RSS_TBL_LINE_NUM)
			return -EINVAL;
		/* Read index */
		reg_val |= (rss->u.entry.tbl_line <<
				MVPP22_RSS_IDX_ENTRY_NUM_OFF |
			   rss->u.entry.tbl_id <<
				MVPP22_RSS_IDX_TBL_NUM_OFF);
		pp2_reg_write(hw->base[0].va, MVPP22_RSS_IDX_REG, reg_val);
		/* Read entry */
		rss->u.entry.rxq = pp2_reg_read(hw->base[0].va,
						MVPP22_RSS_TBL_ENTRY_REG) &
						MVPP22_RSS_TBL_ENTRY_MASK;
		rss->u.entry.width = pp2_reg_read(hw->base[0].va,
						  MVPP22_RSS_WIDTH_REG) &
						  MVPP22_RSS_WIDTH_MASK;
	}
	return 0;
}

/* Go over C2 table and enable RSS in default flows */
int pp2_rss_c2_enable(struct pp2_port *port, int en)
{
	int index;
	int c2_status;
	int rc;
	u8 port_id;
	struct mv_pp2x_cls_c2_entry c2;
	struct pp2_hw *hw = &port->parent->hw;

	c2_status = pp2_reg_read(hw->base[0].va, MVPP2_CLS2_TCAM_CTRL_REG);
	if (!c2_status) {
		pr_info("c2 is off\n");
		return -EINVAL;
	}

	mv_pp2x_c2_sw_clear(&c2);

	for (index = 0; index < MVPP2_CLS_C2_TCAM_SIZE; index++) {
		mv_pp2x_cls_c2_hw_read(hw->base[0].va, index, &c2);
		port_id = pp2_cls_c2_tcam_port_get(&c2);

		if (c2.inv == 0 && port_id == (1 << port->id)) {
			/* Set RSS */
			rc = mv_pp2x_cls_c2_rss_set(&c2, MVPP2_ACTION_TYPE_UPDT_LOCK, en);
			if (rc)
				return rc;
			mv_pp2x_cls_c2_hw_write(hw->base[0].va, index, &c2);

			mv_pp2x_c2_sw_clear(&c2);
			mv_pp2x_cls_c2_hw_read(hw->base[0].va, index, &c2);
		}
	}
	return 0;
}

/* C3 engine */

/********************************************************************************/
/*		C3 Common utilities						*/
/********************************************************************************/
static void pp2_cls_c3_shadow_set(int hek_size, int index, int ext_index)
{
	pp2_cls_c3_shadow_tbl[index].size = hek_size;

	if (hek_size > MVPP2_CLS_C3_HEK_BYTES) {
		pp2_cls_c3_shadow_tbl[index].ext_ptr = ext_index;
		pp2_cls_c3_shadow_ext_tbl[ext_index] = IN_USE;
	} else {
		pp2_cls_c3_shadow_tbl[index].ext_ptr = NOT_IN_USE;
	}
}

/*-----------------------------------------------------------------------------*/
void pp2_cls_c3_shadow_get(int index, int *hek_size, int *ext_index)
{
	*hek_size = pp2_cls_c3_shadow_tbl[index].size;

	if (pp2_cls_c3_shadow_tbl[index].size > MVPP2_CLS_C3_HEK_BYTES)
		*ext_index = pp2_cls_c3_shadow_tbl[index].ext_ptr;
	else
		*ext_index = 0;
}

/*-----------------------------------------------------------------------------*/
void pp2_cls_c3_shadow_init(void)
{
	/* clear hash shadow and extension shadow */
	int index;

	for (index = 0; index < MVPP2_CLS_C3_HASH_TBL_SIZE; index++) {
		pp2_cls_c3_shadow_tbl[index].size = 0;
		pp2_cls_c3_shadow_tbl[index].ext_ptr = NOT_IN_USE;
	}

	for (index = 0; index < MVPP2_CLS_C3_EXT_TBL_SIZE; index++)
		pp2_cls_c3_shadow_ext_tbl[index] = NOT_IN_USE;
}

/*-----------------------------------------------------------------------------*/
int pp2_cls_c3_shadow_free_get(void)
{
	int index;

	/* Go through the all entires from first to last */
	for (index = 0; index < MVPP2_CLS_C3_HASH_TBL_SIZE; index++) {
		if (!pp2_cls_c3_shadow_tbl[index].size)
			break;
	}
	return index;
}

/*-----------------------------------------------------------------------------*/
int pp2_cls_c3_shadow_ext_free_get(void)
{
	int index;

	/* Go through the all entires from first to last */
	for (index = 0; index < MVPP2_CLS_C3_EXT_TBL_SIZE; index++) {
		if (pp2_cls_c3_shadow_ext_tbl[index] == NOT_IN_USE)
			break;
	}
	return index;
}

/*-----------------------------------------------------------------------------*/
int pp2_cls_c3_shadow_ext_status_get(int index)
{
	return pp2_cls_c3_shadow_ext_tbl[index];
}

/*-----------------------------------------------------------------------------*/
void pp2_cls_c3_shadow_clear(int index)
{
	int ext_ptr;

	pp2_cls_c3_shadow_tbl[index].size = 0;
	ext_ptr = pp2_cls_c3_shadow_tbl[index].ext_ptr;

	if (ext_ptr != NOT_IN_USE)
		pp2_cls_c3_shadow_ext_tbl[ext_ptr] = NOT_IN_USE;

	pp2_cls_c3_shadow_tbl[index].ext_ptr = NOT_IN_USE;
}

/*-------------------------------------------------------------------------------*/
/* retun 1 scan procedure completed							  */
/*-------------------------------------------------------------------------------*/
static int pp2_cls_c3_scan_complete(uintptr_t cpu_slot)
{
	u32 reg_val;

	reg_val = pp2_reg_read(cpu_slot, MVPP2_CLS3_STATE_REG);
	reg_val &= MVPP2_CLS3_STATE_SC_DONE_MASK;
	reg_val >>= MVPP2_CLS3_STATE_SC_DONE;

	return reg_val;
}

/*-------------------------------------------------------------------------------*/
/* return 1 if that the last CPU access (Query,Add or Delete) was completed			  */
/*-------------------------------------------------------------------------------*/
static int pp2_cls_c3_cpu_done(uintptr_t cpu_slot)
{
	u32 reg_val;

	reg_val = pp2_reg_read(cpu_slot, MVPP2_CLS3_STATE_REG);
	reg_val &= MVPP2_CLS3_STATE_CPU_DONE_MASK;
	reg_val >>= MVPP2_CLS3_STATE_CPU_DONE;
	return reg_val;
}

/*-------------------------------------------------------------------------------*/
/* 0x0  "ScanCompleted"  scan completed and the scan results are ready in hardware		  */
/* 0x1  "HitCountersClear"  The engine is clearing the Hit Counters				  */
/* 0x2  "ScanWait"  The engine waits for the scan delay timer				  */
/* 0x3  "ScanInProgress"  The scan process is in progress					  */
/*-------------------------------------------------------------------------------*/
static int pp2_cls_c3_scan_state_get(uintptr_t cpu_slot, u32 *state)
{
	u32 reg_val;

	if (mv_pp2x_ptr_validate(state))
		return -EINVAL;

	reg_val = pp2_reg_read(cpu_slot, MVPP2_CLS3_STATE_REG);
	reg_val &= MVPP2_CLS3_STATE_SC_STATE_MASK;
	reg_val >>= MVPP2_CLS3_STATE_SC_STATE;
	*state = reg_val;

	return 0;
}

/*-------------------------------------------------------------------------------*/
/* return 1 if counters clearing is completed						  */
/*-------------------------------------------------------------------------------*/

static int pp2_cls_c3_hit_cntr_clear_done(uintptr_t cpu_slot)
{
	u32 reg_val;

	reg_val = pp2_reg_read(cpu_slot, MVPP2_CLS3_STATE_REG);
	reg_val &= MVPP2_CLS3_STATE_CLEAR_CTR_DONE_MASK;
	reg_val >>= MVPP2_CLS3_STATE_CLEAR_CTR_DONE;
	return reg_val;
}

/*-------------------------------------------------------------------------------*/
void pp2_cls_c3_sw_clear(struct pp2_cls_c3_entry *c3)
{
	memset(c3, 0, sizeof(struct pp2_cls_c3_entry));
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_init(uintptr_t cpu_slot)
{
	int rc;

	pp2_cls_c3_shadow_init();
	rc = pp2_cls_c3_hit_cntrs_clear_all(cpu_slot);
	return rc;
}

/*-------------------------------------------------------------------------------*/
/* Add entry to hash table								  */
/* ext_index used only if hek size < 12						  */
/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_hw_add(uintptr_t cpu_slot, struct pp2_cls_c3_entry *c3, int index, int ext_index)
{
	int reg_start_ind, hek_size, iter = 0;
	u32 reg_val = 0;

	if (mv_pp2x_ptr_validate(c3))
		return -EINVAL;

	if (mv_pp2x_range_validate(index, 0, MVPP2_CLS3_HASH_OP_TBL_ADDR_MAX))
		return -EINVAL;

	c3->index = index;

	/* write key control */
	pp2_reg_write(cpu_slot, MVPP2_CLS3_KEY_CTRL_REG, c3->key.key_ctrl);

	hek_size = ((c3->key.key_ctrl & KEY_CTRL_HEK_SIZE_MASK) >> KEY_CTRL_HEK_SIZE);

	if (hek_size > MVPP2_CLS_C3_HEK_BYTES) {
		/* Extension */

		if (mv_pp2x_range_validate(ext_index, 0, MVPP2_CLS3_HASH_OP_EXT_TBL_ADDR_MAX))
			return -EINVAL;

		c3->ext_index = ext_index;
		reg_val |= (ext_index << MVPP2_CLS3_HASH_OP_EXT_TBL_ADDR);

		/* write 9 hek registers */
		reg_start_ind = 0;
	} else
		/* write 3 hek registers */
		reg_start_ind = 6;

	for (; reg_start_ind < MVPP2_CLS_C3_EXT_HEK_WORDS; reg_start_ind++)
		pp2_reg_write(cpu_slot, MVPP2_CLS3_KEY_HEK_REG(reg_start_ind),
			      c3->key.hek.words[reg_start_ind]);

	reg_val |= (index << MVPP2_CLS3_HASH_OP_TBL_ADDR);
	reg_val &= ~MVPP2_CLS3_MISS_PTR_MASK; /*set miss bit to 0*/
	reg_val |= (1 << MVPP2_CLS3_HASH_OP_ADD);

	/* set hit counter init value */
	pp2_reg_write(cpu_slot, MVPP2_CLS3_INIT_HIT_CNT_REG, sw_init_cnt_set << MVPP2_CLS3_INIT_HIT_CNT_OFFS),
	/*trigger ADD operation*/
	pp2_reg_write(cpu_slot, MVPP2_CLS3_HASH_OP_REG, reg_val);

	/* wait to cpu access done bit */
	while (!pp2_cls_c3_cpu_done(cpu_slot))
		if (++iter >= RETRIES_EXCEEDED) {
			pr_err("%s:Error - retries exceeded.\n", __func__);
			return -EBUSY;
		}

	/* write action table registers */
	pp2_reg_write(cpu_slot, MVPP2_CLS3_ACT_REG, c3->sram.regs.actions);
	pp2_reg_write(cpu_slot, MVPP2_CLS3_ACT_QOS_ATTR_REG, c3->sram.regs.qos_attr);
	pp2_reg_write(cpu_slot, MVPP2_CLS3_ACT_HWF_ATTR_REG, c3->sram.regs.hwf_attr);
	pp2_reg_write(cpu_slot, MVPP2_CLS3_ACT_DUP_ATTR_REG, c3->sram.regs.dup_attr);
	pp2_reg_write(cpu_slot, MVPP2_CLS3_ACT_SEQ_L_ATTR_REG, c3->sram.regs.seq_l_attr);
	pp2_reg_write(cpu_slot, MVPP2_CLS3_ACT_SEQ_H_ATTR_REG, c3->sram.regs.seq_h_attr);
	/* set entry as valid, extesion pointer in use only if size > 12*/
	pp2_cls_c3_shadow_set(hek_size, index, ext_index);

	return 0;
}

/*-------------------------------------------------------------------------------*/
/*	Add entry to miss hash table							  */
/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_hw_miss_add(uintptr_t cpu_slot, struct pp2_cls_c3_entry *c3, int lkp_type)
{
	u32 reg_val = 0;

	if (mv_pp2x_ptr_validate(c3))
		return -EINVAL;

	if (mv_pp2x_range_validate(lkp_type, 0, MVPP2_CLS_C3_MISS_TBL_SIZE - 1))
		return -EINVAL;

	c3->index = lkp_type;

	reg_val |= (lkp_type << MVPP2_CLS3_HASH_OP_TBL_ADDR);
	reg_val |= (1 << MVPP2_CLS3_HASH_OP_ADD);
	reg_val |= MVPP2_CLS3_MISS_PTR_MASK;/*set miss bit to 1*/

	/*index to miss table */
	pp2_reg_write(cpu_slot, MVPP2_CLS3_HASH_OP_REG, reg_val);

	/* write action table registers */
	pp2_reg_write(cpu_slot, MVPP2_CLS3_ACT_REG, c3->sram.regs.actions);
	pp2_reg_write(cpu_slot, MVPP2_CLS3_ACT_QOS_ATTR_REG, c3->sram.regs.qos_attr);
	pp2_reg_write(cpu_slot, MVPP2_CLS3_ACT_HWF_ATTR_REG, c3->sram.regs.hwf_attr);
	pp2_reg_write(cpu_slot, MVPP2_CLS3_ACT_DUP_ATTR_REG, c3->sram.regs.dup_attr);
	pp2_reg_write(cpu_slot, MVPP2_CLS3_ACT_SEQ_L_ATTR_REG, c3->sram.regs.seq_l_attr);
	pp2_reg_write(cpu_slot, MVPP2_CLS3_ACT_SEQ_H_ATTR_REG, c3->sram.regs.seq_h_attr);
	/*clear hit counter, clear on read */
	pp2_cls_c3_hit_cntrs_miss_read(cpu_slot, lkp_type, &reg_val);

	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_hw_del(uintptr_t cpu_slot, int index)
{
	u32 reg_val = 0;
	int iter = 0;

	if (mv_pp2x_range_validate(index, 0, MVPP2_CLS3_HASH_OP_TBL_ADDR_MAX))
		return -EINVAL;

	reg_val |= (index << MVPP2_CLS3_HASH_OP_TBL_ADDR);
	reg_val |= (1 << MVPP2_CLS3_HASH_OP_DEL);
	reg_val &= ~MVPP2_CLS3_MISS_PTR_MASK;/*set miss bit to 1*/

	/*trigger del operation*/
	pp2_reg_write(cpu_slot, MVPP2_CLS3_HASH_OP_REG, reg_val);

	/* wait to cpu access done bit */
	while (!pp2_cls_c3_cpu_done(cpu_slot))
		if (++iter >= RETRIES_EXCEEDED) {
			pr_err("%s:Error - retries exceeded.\n", __func__);
			return -EBUSY;
		}

	/* delete form shadow and extension shadow if exist */
	pp2_cls_c3_shadow_clear(index);

	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_hw_del_all(uintptr_t cpu_slot)
{
	int index, status;

	for (index = 0; index < MVPP2_CLS_C3_HASH_TBL_SIZE; index++) {
		status = pp2_cls_c3_hw_del(cpu_slot, index);
		if (status != 0)
			return status;
	}
	return 0;
}

/*-------------------------------------------------------------------------------*/
void pp2_cls_c3_hw_init_ctr_set(int cnt_val)
{
	sw_init_cnt_set = cnt_val;
}

/*-------------------------------------------------------------------------------*/
static int pp2_cls_c3_hw_query_add_relocate(uintptr_t cpu_slot, int new_idx, int max_depth, int cur_depth,
					    struct pp2_cls_c3_hash_pair *hash_pair_arr)
{
	int ret_val = 0, index_free, idx = 0;
	u8 occupied_bmp;
	struct pp2_cls_c3_entry local_c3;
	int used_index[MVPP2_CLS3_HASH_BANKS_NUM] = {0};

	if (cur_depth >= max_depth)
		return -EINVAL;

	pp2_cls_c3_sw_clear(&local_c3);

	ret_val = pp2_cls_c3_hw_read(cpu_slot, &local_c3, new_idx);
	if (ret_val) {
		pr_err("%s could not get key for index [0x%x]\n", __func__, new_idx);
		return ret_val;
	}

	ret_val = pp2_cls_c3_hw_query(cpu_slot, &local_c3, &occupied_bmp, used_index);
	if (ret_val) {
		pr_err("%s: pp2_cls_c3_hw_query failed, depth = %d\n", __func__, cur_depth);
		return ret_val;
	}

	/* fill in indices for this key */
	for (idx = 0; idx < MVPP2_CLS3_HASH_BANKS_NUM; idx++) {
		/* if new index is in the bank index, skip it */
		if (new_idx == used_index[idx]) {
			used_index[idx] = 0;
			continue;
		}

		/* found a vacant index */
		if (!(occupied_bmp & (1 << idx))) {
			index_free = used_index[idx];
			break;
		}
	}

	/* no free index, recurse and relocate another key */
	if (idx == MVPP2_CLS3_HASH_BANKS_NUM) {
#ifdef MV_DEBUG
		pr_debug("new[0x%.3x]:%.1d ", new_idx, cur_depth);
		for (idx = 0; idx < MVPP2_CLS3_HASH_BANKS_NUM; idx++)
			pr_debug("0x%.3x ", used_index[idx]);
		pr_debug("\n");
#endif

		/* recurse over all valid indices */
		for (idx = 0; idx < MVPP2_CLS3_HASH_BANKS_NUM; idx++) {
			if (used_index[idx] == 0)
				continue;

			if (pp2_cls_c3_hw_query_add_relocate(cpu_slot, used_index[idx], max_depth, cur_depth + 1,
							     hash_pair_arr) == 0)
				break;
		}

		/* tried relocate, no valid entries found */
		if (idx == MVPP2_CLS3_HASH_BANKS_NUM)
			return -EIO;
	}

	/* if we reached here, we found a valid free index */
	index_free = used_index[idx];

	/* new_idx del is not necessary */

	/*We do not chage extension tabe*/
	ret_val = pp2_cls_c3_hw_add(cpu_slot, &local_c3, index_free, local_c3.ext_index);

	/* update the hash pair */
	if (!hash_pair_arr) {
		hash_pair_arr->old_idx[hash_pair_arr->pair_num] = new_idx;
		hash_pair_arr->new_idx[hash_pair_arr->pair_num] = index_free;
		hash_pair_arr->pair_num++;
	}

	if (ret_val != 0) {
		pr_err("%s:Error - pp2_cls_c3_hw_add failed, depth = %d\\n", __func__, cur_depth);
		return ret_val;
	}

	pr_info("key relocated  0x%.3x->0x%.3x\n", new_idx, index_free);

	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_hw_query_add(uintptr_t cpu_slot, struct pp2_cls_c3_entry *c3, int max_search_depth,
			    struct pp2_cls_c3_hash_pair *hash_pair_arr)
{
	int used_index[MVPP2_CLS3_HASH_BANKS_NUM] = {0};
	u8 occupied_bmp;
	int idx, index_free, hek_size, ret_val, ext_index = 0;

	ret_val = pp2_cls_c3_hw_query(cpu_slot, c3, &occupied_bmp, used_index);
	if (ret_val != 0) {
		pr_err("%s:Error - pp2_cls_c3_hw_query failed\n", __func__);
		return ret_val;
	}

	/* Select available entry index */
	for (idx = 0; idx < MVPP2_CLS3_HASH_BANKS_NUM; idx++) {
		if (!(occupied_bmp & (1 << idx)))
			break;
	}

	/* Available index did not found, try to relocate another key */
	if (idx == MVPP2_CLS3_HASH_BANKS_NUM) {
		for (idx = 0; idx < MVPP2_CLS3_HASH_BANKS_NUM; idx++) {
			if (pp2_cls_c3_hw_query_add_relocate(cpu_slot, used_index[idx], max_search_depth,
							     0 /*curren depth*/, hash_pair_arr) == 0)
				break;
		}

		if (idx == MVPP2_CLS3_HASH_BANKS_NUM) {
			/* Available index did not found*/
			pr_err("%s:Error - HASH table is full.\n", __func__);
			return -EIO;
		}
	}

	index_free = used_index[idx];

	hek_size = ((c3->key.key_ctrl & KEY_CTRL_HEK_SIZE_MASK) >> KEY_CTRL_HEK_SIZE);

	if (hek_size > MVPP2_CLS_C3_HEK_BYTES) {
		/* Get Free Extension Index */
		ext_index = pp2_cls_c3_shadow_ext_free_get();

		if (ext_index == MVPP2_CLS_C3_EXT_TBL_SIZE) {
			pr_err("%s:Error - Extension table is full.\n", __func__);
			return -EIO;
		}
	}

	ret_val = pp2_cls_c3_hw_add(cpu_slot, c3, index_free, ext_index);
	if (ret_val != 0) {
		pr_err("%s:Error - pp2_cls_c3_hw_add failed\n", __func__);
		return ret_val;
	}

	if (hek_size > MVPP2_CLS_C3_HEK_BYTES)
		pr_info("Added C3 entry @ index=0x%.3x ext=0x%.3x\n", index_free, ext_index);
	else
		pr_info("Added C3 entry @ index=0x%.3x\n", index_free);

	return 0;
}

/*-------------------------------------------------------------------------------*/
/*	if index or occupied_bmp is NULL dump the data					  */
/*	index[] size must be 8							  */
/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_hw_query(uintptr_t cpu_slot, struct pp2_cls_c3_entry *c3, u8 *occupied_bmp, int index[])
{
	int idx = 0;
	u32 reg_val = 0;

	if (mv_pp2x_ptr_validate(c3))
		return -EINVAL;

	/* write key control */
	pp2_reg_write(cpu_slot, MVPP2_CLS3_KEY_CTRL_REG, c3->key.key_ctrl);

	/* write hek */
	for (idx = 0; idx < MVPP2_CLS_C3_EXT_HEK_WORDS; idx++)
		pp2_reg_write(cpu_slot, MVPP2_CLS3_KEY_HEK_REG(idx), c3->key.hek.words[idx]);

	/*trigger query operation*/
	pp2_reg_write(cpu_slot, MVPP2_CLS3_QRY_ACT_REG, (1 << MVPP2_CLS3_QRY_ACT));

	idx = 0;
	while (!pp2_cls_c3_cpu_done(cpu_slot))
		if (++idx >= RETRIES_EXCEEDED) {
			pr_err("%s:Error - retries exceeded.\n", __func__);
			return -EBUSY;
		}

	reg_val = pp2_reg_read(cpu_slot, MVPP2_CLS3_STATE_REG) & MVPP2_CLS3_STATE_OCCIPIED_MASK;
	reg_val = reg_val >> MVPP2_CLS3_STATE_OCCIPIED;

	if ((!occupied_bmp) || (!index)) {
		/* print to screen - call from sysfs*/
		for (idx = 0; idx < MVPP2_CLS3_HASH_BANKS_NUM; idx++)
			pr_info("0x%8.8x	%s\n", pp2_reg_read(cpu_slot, MVPP2_CLS3_QRY_RES_HASH_REG(idx)),
				 (reg_val & (1 << idx)) ? "OCCUPIED" : "FREE");
		return 0;
	}

	*occupied_bmp = reg_val;
	for (idx = 0; idx < MVPP2_CLS3_HASH_BANKS_NUM; idx++)
		index[idx] = pp2_reg_read(cpu_slot, MVPP2_CLS3_QRY_RES_HASH_REG(idx));

	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_hw_read(uintptr_t cpu_slot, struct pp2_cls_c3_entry *c3, int index)
{
	int i, is_ext;
	int reg_val = 0;
	u32 hash_data[MVPP2_CLS3_HASH_DATA_REG_NUM];
	u32 hash_ext_data[MVPP2_CLS3_HASH_EXT_DATA_REG_NUM];

	if (mv_pp2x_ptr_validate(c3))
		return -EINVAL;

	if (mv_pp2x_range_validate(index, 0, MVPP2_CLS3_HASH_OP_TBL_ADDR_MAX))
		return -EINVAL;

	pp2_cls_c3_sw_clear(c3);

	c3->index = index;
	c3->ext_index = NOT_IN_USE;

	/* write index */
	pp2_reg_write(cpu_slot, MVPP2_CLS3_DB_INDEX_REG, index);

	reg_val |= (index << MVPP2_CLS3_HASH_OP_TBL_ADDR);
	pp2_reg_write(cpu_slot, MVPP2_CLS3_HASH_OP_REG, reg_val);

	/* read action table */
	c3->sram.regs.actions = pp2_reg_read(cpu_slot, MVPP2_CLS3_ACT_REG);
	c3->sram.regs.qos_attr = pp2_reg_read(cpu_slot, MVPP2_CLS3_ACT_QOS_ATTR_REG);
	c3->sram.regs.hwf_attr = pp2_reg_read(cpu_slot, MVPP2_CLS3_ACT_HWF_ATTR_REG);
	c3->sram.regs.dup_attr = pp2_reg_read(cpu_slot, MVPP2_CLS3_ACT_DUP_ATTR_REG);

	c3->sram.regs.seq_l_attr = pp2_reg_read(cpu_slot, MVPP2_CLS3_ACT_SEQ_L_ATTR_REG);
	c3->sram.regs.seq_h_attr = pp2_reg_read(cpu_slot, MVPP2_CLS3_ACT_SEQ_H_ATTR_REG);

	/* read hash data*/
	for (i = 0; i < MVPP2_CLS3_HASH_DATA_REG_NUM; i++)
		hash_data[i] = pp2_reg_read(cpu_slot, MVPP2_CLS3_HASH_DATA_REG(i));

	if (pp2_cls_c3_shadow_tbl[index].size == 0)
		/* entry not in use */
		return 0;

	c3->key.key_ctrl = 0;

	if (pp2_cls_c3_shadow_tbl[index].ext_ptr == NOT_IN_USE) {
		is_ext = 0;
		/* TODO REMOVE NEXT LINES- ONLY FOR INTERNAL VALIDATION */
		if ((pp2_cls_c3_shadow_tbl[index].size == 0) || (pp2_cls_c3_shadow_tbl[index].ext_ptr != NOT_IN_USE)) {
			pr_err("%s: SW internal error.\n", __func__);
			return -EIO;
		}

		/*read Multihash entry data*/
		c3->key.hek.words[6] = hash_data[0]; /* hek 0*/
		c3->key.hek.words[7] = hash_data[1]; /* hek 1*/
		c3->key.hek.words[8] = hash_data[2]; /* hek 2*/

		/* write key control data to SW */
		c3->key.key_ctrl |= (((hash_data[3] & KEY_PRT_ID_MASK(is_ext)) >>
					(KEY_PRT_ID(is_ext) % DWORD_BITS_LEN)) << KEY_CTRL_PRT_ID);

		c3->key.key_ctrl |= (((hash_data[3] & KEY_PRT_ID_TYPE_MASK(is_ext)) >>
					(KEY_PRT_ID_TYPE(is_ext) % DWORD_BITS_LEN)) << KEY_CTRL_PRT_ID_TYPE);

		c3->key.key_ctrl |= (((hash_data[3] & KEY_LKP_TYPE_MASK(is_ext)) >>
					(KEY_LKP_TYPE(is_ext) % DWORD_BITS_LEN)) << KEY_CTRL_LKP_TYPE);

		c3->key.key_ctrl |= (((hash_data[3] & KEY_L4_INFO_MASK(is_ext)) >>
					(KEY_L4_INFO(is_ext) % DWORD_BITS_LEN)) << KEY_CTRL_L4);

	} else {
		is_ext = 1;
		/* TODO REMOVE NEXT LINES- ONLY FOR INTERNAL VALIDATION */
		if ((pp2_cls_c3_shadow_tbl[index].size == 0) || (pp2_cls_c3_shadow_tbl[index].ext_ptr == NOT_IN_USE)) {
			pr_err("%s: SW internal error.\n", __func__);
			return -EIO;
		}
		c3->ext_index = pp2_cls_c3_shadow_tbl[index].ext_ptr;

		/* write extension index */
		pp2_reg_write(cpu_slot, MVPP2_CLS3_DB_INDEX_REG, pp2_cls_c3_shadow_tbl[index].ext_ptr);

		/* read hash extesion data*/
		for (i = 0; i < MVPP2_CLS3_HASH_EXT_DATA_REG_NUM; i++)
			hash_ext_data[i] = pp2_reg_read(cpu_slot, MVPP2_CLS3_HASH_EXT_DATA_REG(i));

		/* heks bytes 35 - 32 */
		c3->key.hek.words[8] = ((hash_data[2] & 0x00FFFFFF) << 8) | ((hash_data[1] & 0xFF000000) >> 24);

		/* heks bytes 31 - 28 */
		c3->key.hek.words[7] = ((hash_data[1] & 0x00FFFFFF) << 8) | ((hash_data[0] & 0xFF000000) >> 24);

		/* heks bytes 27 - 24 */
		c3->key.hek.words[6] = ((hash_data[0] & 0x00FFFFFF) << 8) | (hash_ext_data[6] & 0x000000FF);

		c3->key.hek.words[5] = hash_ext_data[5]; /* heks bytes 23 - 20 */
		c3->key.hek.words[4] = hash_ext_data[4]; /* heks bytes 19 - 16 */
		c3->key.hek.words[3] = hash_ext_data[3]; /* heks bytes 15 - 12 */
		c3->key.hek.words[2] = hash_ext_data[2]; /* heks bytes 11 - 8  */
		c3->key.hek.words[1] = hash_ext_data[1]; /* heks bytes 7 - 4   */
		c3->key.hek.words[0] = hash_ext_data[0]; /* heks bytes 3 - 0   */

		/* write key control data to SW*/

		c3->key.key_ctrl |= (((hash_data[3] & KEY_PRT_ID_MASK(is_ext)) >>
					(KEY_PRT_ID(is_ext) % DWORD_BITS_LEN)) << KEY_CTRL_PRT_ID);

		/* PPv2.1 (feature MAS 3.16) LKP_TYPE size and offset changed */

		c3->key.key_ctrl |= (((hash_data[3] & KEY_PRT_ID_TYPE_MASK(is_ext)) >>
					(KEY_PRT_ID_TYPE(is_ext) % DWORD_BITS_LEN)) << KEY_CTRL_PRT_ID_TYPE);

		c3->key.key_ctrl |= ((((hash_data[2] & 0xf8000000) >> 27) |
					((hash_data[3] & 0x1) << 5)) << KEY_CTRL_LKP_TYPE);

		c3->key.key_ctrl |= (((hash_data[2] & KEY_L4_INFO_MASK(is_ext)) >>
					(KEY_L4_INFO(is_ext) % DWORD_BITS_LEN)) << KEY_CTRL_L4);
	}

	/* update hek size */
	c3->key.key_ctrl |= ((pp2_cls_c3_shadow_tbl[index].size << KEY_CTRL_HEK_SIZE) & KEY_CTRL_HEK_SIZE_MASK);

	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_hw_miss_read(uintptr_t cpu_slot, struct pp2_cls_c3_entry *c3, int lkp_type)
{
	u32 reg_val = 0;

	if (mv_pp2x_ptr_validate(c3))
		return -EINVAL;

	if (mv_pp2x_range_validate(lkp_type, 0, MVPP2_CLS_C3_MISS_TBL_SIZE - 1))
		return -EINVAL;

	pp2_cls_c3_sw_clear(c3);

	c3->index = lkp_type;
	c3->ext_index = NOT_IN_USE;

	reg_val = (lkp_type << MVPP2_CLS3_HASH_OP_TBL_ADDR) | MVPP2_CLS3_MISS_PTR_MASK;
	pp2_reg_write(cpu_slot, MVPP2_CLS3_HASH_OP_REG, reg_val);

	/* read action table */
	c3->sram.regs.actions = pp2_reg_read(cpu_slot, MVPP2_CLS3_ACT_REG);
	c3->sram.regs.qos_attr = pp2_reg_read(cpu_slot, MVPP2_CLS3_ACT_QOS_ATTR_REG);
	c3->sram.regs.hwf_attr = pp2_reg_read(cpu_slot, MVPP2_CLS3_ACT_HWF_ATTR_REG);
	c3->sram.regs.dup_attr = pp2_reg_read(cpu_slot, MVPP2_CLS3_ACT_DUP_ATTR_REG);
	c3->sram.regs.seq_l_attr = pp2_reg_read(cpu_slot, MVPP2_CLS3_ACT_SEQ_L_ATTR_REG);
	c3->sram.regs.seq_h_attr = pp2_reg_read(cpu_slot, MVPP2_CLS3_ACT_SEQ_H_ATTR_REG);

	return 0;
}

/*-------------------------------------------------------------------------------*/
/*	APIs for Classification C3 key fields						  */
/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_sw_l4_info_set(struct pp2_cls_c3_entry *c3, int l4info)
{
	if (mv_pp2x_ptr_validate(c3))
		return -EINVAL;

	if (mv_pp2x_range_validate(l4info, 0, KEY_CTRL_L4_MAX))
		return -EINVAL;

	c3->key.key_ctrl &= ~KEY_CTRL_L4_MASK;
	c3->key.key_ctrl |= (l4info << KEY_CTRL_L4);
	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_sw_lkp_type_set(struct pp2_cls_c3_entry *c3, int lkp_type)
{
	if (mv_pp2x_ptr_validate(c3))
		return -EINVAL;

	if (mv_pp2x_range_validate(lkp_type, 0, KEY_CTRL_LKP_TYPE_MAX))
		return -EINVAL;

	c3->key.key_ctrl &= ~KEY_CTRL_LKP_TYPE_MASK;
	c3->key.key_ctrl |= (lkp_type << KEY_CTRL_LKP_TYPE);
	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_sw_port_id_set(struct pp2_cls_c3_entry *c3, int type, int portid)
{
	if (mv_pp2x_ptr_validate(c3))
		return -EINVAL;

	if (mv_pp2x_range_validate(portid, 0, KEY_CTRL_PRT_ID_MAX))
		return -EINVAL;

	if (mv_pp2x_range_validate(type, 0, KEY_CTRL_PRT_ID_TYPE_MAX))
		return -EINVAL;

	c3->key.key_ctrl &= ~(KEY_CTRL_PRT_ID_MASK | KEY_CTRL_PRT_ID_TYPE_MASK);
	c3->key.key_ctrl |= ((portid << KEY_CTRL_PRT_ID) | (type << KEY_CTRL_PRT_ID_TYPE));

	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_sw_hek_size_set(struct pp2_cls_c3_entry *c3, int hek_size)
{
	if (mv_pp2x_ptr_validate(c3))
		return -EINVAL;

	if (mv_pp2x_range_validate(hek_size, 0, KEY_CTRL_HEK_SIZE_MAX))
		return -EINVAL;

	c3->key.key_ctrl &= ~KEY_CTRL_HEK_SIZE_MASK;
	c3->key.key_ctrl |= (hek_size << KEY_CTRL_HEK_SIZE);
	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_sw_hek_byte_set(struct pp2_cls_c3_entry *c3, u32 offs, u8 byte)
{
	if (mv_pp2x_ptr_validate(c3))
		return -EINVAL;

	if (mv_pp2x_range_validate(offs, 0, ((MVPP2_CLS_C3_EXT_HEK_WORDS * 4) - 1)))
		return -EINVAL;

	c3->key.hek.bytes[HW_BYTE_OFFS(offs)] = byte;

	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_sw_hek_word_set(struct pp2_cls_c3_entry *c3, u32 offs, u32 word)
{
	if (mv_pp2x_ptr_validate(c3))
		return -EINVAL;

	if (mv_pp2x_range_validate(offs, 0, ((MVPP2_CLS_C3_EXT_HEK_WORDS) - 1)))
		return -EINVAL;

	c3->key.hek.words[offs] = word;

	return 0;
}

/*-------------------------------------------------------------------------------*/
/*	APIs for Classification C3 action table fields					  */
/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_color_set(struct pp2_cls_c3_entry *c3, int cmd)
{
	if (mv_pp2x_ptr_validate(c3))
		return -EINVAL;

	if (mv_pp2x_range_validate(cmd, 0, MVPP2_COLOR_ACTION_TYPE_RED_LOCK))
		return -EINVAL;

	c3->sram.regs.actions &= ~MVPP2_CLS3_ACT_COLOR_MASK;
	c3->sram.regs.actions |= (cmd << MVPP2_CLS3_ACT_COLOR);

	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_queue_high_set(struct pp2_cls_c3_entry *c3, int cmd, int queue)
{
	if (mv_pp2x_ptr_validate(c3))
		return -EINVAL;

	if (mv_pp2x_range_validate(cmd, 0, MVPP2_ACTION_TYPE_UPDT_LOCK))
		return -EINVAL;

	if (mv_pp2x_range_validate(queue, 0, MVPP2_CLS3_ACT_QOS_ATTR_HIGH_Q_MAX))
		return -EINVAL;

	/*set command*/
	c3->sram.regs.actions &= ~MVPP2_CLS3_ACT_HIGH_Q_MASK;
	c3->sram.regs.actions |= (cmd << MVPP2_CLS3_ACT_HIGH_Q);

	/*set modify High queue value*/
	c3->sram.regs.qos_attr &= ~MVPP2_CLS3_ACT_QOS_ATTR_HIGH_Q_MASK;
	c3->sram.regs.qos_attr |= (queue << MVPP2_CLS3_ACT_QOS_ATTR_HIGH_Q);

	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_queue_low_set(struct pp2_cls_c3_entry *c3, int cmd, int queue)
{
	if (mv_pp2x_ptr_validate(c3))
		return -EINVAL;

	if (mv_pp2x_range_validate(cmd, 0, MVPP2_ACTION_TYPE_UPDT_LOCK))
		return -EINVAL;

	if (mv_pp2x_range_validate(queue, 0, MVPP2_CLS3_ACT_QOS_ATTR_LOW_Q_MAX))
		return -EINVAL;

	/*set command*/
	c3->sram.regs.actions &= ~MVPP2_CLS3_ACT_LOW_Q_MASK;
	c3->sram.regs.actions |= (cmd << MVPP2_CLS3_ACT_LOW_Q);

	/*set modify High queue value*/
	c3->sram.regs.qos_attr &= ~MVPP2_CLS3_ACT_QOS_ATTR_LOW_Q_MASK;
	c3->sram.regs.qos_attr |= (queue << MVPP2_CLS3_ACT_QOS_ATTR_LOW_Q);

	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_queue_set(struct pp2_cls_c3_entry *c3, int cmd, int queue)
{
	int status = 0;
	int q_high, q_low;

	if (mv_pp2x_ptr_validate(c3))
		return -EINVAL;

	if (mv_pp2x_range_validate(queue, 0, MVPP2_CLS3_ACT_QOS_ATTR_Q_MAX))
		return -EINVAL;

	/* cmd validation in set functions */

	q_high = (queue & MVPP2_CLS3_ACT_QOS_ATTR_HIGH_Q_MASK) >> MVPP2_CLS3_ACT_QOS_ATTR_HIGH_Q;
	q_low = (queue & MVPP2_CLS3_ACT_QOS_ATTR_LOW_Q_MASK) >> MVPP2_CLS3_ACT_QOS_ATTR_LOW_Q;

	status |= pp2_cls_c3_queue_low_set(c3, cmd, q_high);
	status |= pp2_cls_c3_queue_high_set(c3, cmd, q_low);

	return status;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_forward_set(struct pp2_cls_c3_entry *c3, int cmd)
{
	if (mv_pp2x_ptr_validate(c3))
		return -EINVAL;

	if (mv_pp2x_range_validate(cmd, 0, MVPP2_FRWD_ACTION_TYPE_HWF_LOW_LATENCY_LOCK))
		return -EINVAL;

	c3->sram.regs.actions &= ~MVPP2_CLS3_ACT_FWD_MASK;
	c3->sram.regs.actions |= (cmd << MVPP2_CLS3_ACT_FWD);
	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_policer_set(struct pp2_cls_c3_entry *c3, int cmd, int policer_id, int bank)
{
	if (mv_pp2x_ptr_validate(c3))
		return -EINVAL;

	if (mv_pp2x_range_validate(cmd, 0, MVPP2_ACTION_TYPE_UPDT_LOCK))
		return -EINVAL;

	if (mv_pp2x_range_validate(policer_id, 0, MVPP2_CLS3_ACT_DUP_POLICER_MAX))
		return -EINVAL;

	if (mv_pp2x_range_validate(bank, 0, 1))
		return -EINVAL;

	c3->sram.regs.actions &= ~MVPP2_CLS3_ACT_POLICER_SELECT_MASK;
	c3->sram.regs.actions |= (cmd << MVPP2_CLS3_ACT_POLICER_SELECT);

	c3->sram.regs.dup_attr &= ~MVPP2_CLS3_ACT_DUP_POLICER_MASK;
	c3->sram.regs.dup_attr |= (policer_id << MVPP2_CLS3_ACT_DUP_POLICER_ID);

	if (bank)
		c3->sram.regs.dup_attr |= MVPP2_CLS3_ACT_DUP_POLICER_BANK_MASK;
	else
		c3->sram.regs.dup_attr &= ~MVPP2_CLS3_ACT_DUP_POLICER_BANK_MASK;

	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_flow_id_en(struct pp2_cls_c3_entry *c3, int flowid_en)
{
	 if (mv_pp2x_ptr_validate(c3))
		return -EINVAL;

	/*set Flow ID enable or disable*/
	if (flowid_en)
		c3->sram.regs.actions |= (1 << MVPP2_CLS3_ACT_FLOW_ID_EN);
	else
		c3->sram.regs.actions &= ~(1 << MVPP2_CLS3_ACT_FLOW_ID_EN);

	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_rss_set(struct pp2_cls_c3_entry *c3, int cmd, int rss_en)
{
	 if (mv_pp2x_ptr_validate(c3))
		return -EINVAL;

	 if (mv_pp2x_range_validate(cmd, 0, MVPP2_ACTION_TYPE_UPDT_LOCK))
		return -EINVAL;

	 if (mv_pp2x_range_validate(rss_en, 0, 1))
		return -EINVAL;

	 c3->sram.regs.actions &= ~MVPP2_CLS3_ACT_RSS_EN_MASK;
	 c3->sram.regs.actions |= (cmd << MVPP2_CLS3_ACT_RSS_EN);

	 c3->sram.regs.dup_attr &= ~MVPP2_CLS3_ACT_DUP_RSS_EN_MASK;
	 c3->sram.regs.dup_attr |= (rss_en << MVPP2_CLS3_ACT_DUP_RSS_EN_BIT);

	 return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_mod_set(struct pp2_cls_c3_entry *c3, int data_ptr, int instr_offs, int l4_csum)
{
	if (mv_pp2x_ptr_validate(c3))
		return -EINVAL;

	if (mv_pp2x_range_validate(data_ptr, 0, MVPP2_CLS3_ACT_HWF_ATTR_DPTR_MAX))
		return -EINVAL;

	if (mv_pp2x_range_validate(instr_offs, 0, MVPP2_CLS3_ACT_HWF_ATTR_IPTR_MAX))
		return -EINVAL;

	if (mv_pp2x_range_validate(l4_csum, 0, 1))
		return -EINVAL;

	c3->sram.regs.hwf_attr &= ~MVPP2_CLS3_ACT_HWF_ATTR_DPTR_MASK;
	c3->sram.regs.hwf_attr &= ~MVPP2_CLS3_ACT_HWF_ATTR_IPTR_MASK;
	c3->sram.regs.hwf_attr &= ~MVPP2_CLS3_ACT_HWF_ATTR_CHKSM_EN_MASK;

	c3->sram.regs.hwf_attr |= (data_ptr << MVPP2_CLS3_ACT_HWF_ATTR_DPTR);
	c3->sram.regs.hwf_attr |= (instr_offs << MVPP2_CLS3_ACT_HWF_ATTR_IPTR);
	c3->sram.regs.hwf_attr |= (l4_csum << MVPP2_CLS3_ACT_HWF_ATTR_CHKSM_EN);

	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_mtu_set(struct pp2_cls_c3_entry *c3, int mtu_inx)
{
	if (mv_pp2x_ptr_validate(c3))
		return -EINVAL;

	if (mv_pp2x_range_validate(mtu_inx, 0, MVPP2_CLS3_ACT_HWF_ATTR_MTU_INX_MAX))
		return -EINVAL;

	c3->sram.regs.hwf_attr &= ~MVPP2_CLS3_ACT_HWF_ATTR_MTU_INX_MASK;
	c3->sram.regs.hwf_attr |= (mtu_inx << MVPP2_CLS3_ACT_HWF_ATTR_MTU_INX);
	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_dup_set(struct pp2_cls_c3_entry *c3, int dupid, int count)
{
	if (mv_pp2x_ptr_validate(c3))
		return -EINVAL;

	if (mv_pp2x_range_validate(count, 0, MVPP2_CLS3_ACT_DUP_COUNT_MAX))
		return -EINVAL;

	if (mv_pp2x_range_validate(dupid, 0, MVPP2_CLS3_ACT_DUP_FID_MAX))
		return -EINVAL;

	/*set flowid and count*/
	c3->sram.regs.dup_attr &= ~(MVPP2_CLS3_ACT_DUP_FID_MASK | MVPP2_CLS3_ACT_DUP_COUNT_MASK);
	c3->sram.regs.dup_attr |= (dupid << MVPP2_CLS3_ACT_DUP_FID);
	c3->sram.regs.dup_attr |= (count << MVPP2_CLS3_ACT_DUP_COUNT);

	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_seq_set(struct pp2_cls_c3_entry *c3, int id,  int bits_offs,  int bits)
{
	u32 low_bits, high_bits = 0;

	if (mv_pp2x_ptr_validate(c3))
		return -EINVAL;

	if (mv_pp2x_range_validate(bits, 0, MVPP2_CLS_SEQ_SIZE_MAX))
		return -EINVAL;

	if (mv_pp2x_range_validate(id, 0, (1 << bits) - 1))
		return -EINVAL;

	if (mv_pp2x_range_validate(bits_offs + bits, 0, MVPP2_CLS3_ACT_SEQ_SIZE))
		return -EINVAL;

	if (bits_offs >= DWORD_BITS_LEN)
		high_bits = bits;

	else if (bits_offs + bits > DWORD_BITS_LEN)
		high_bits = (bits_offs + bits) % DWORD_BITS_LEN;

	low_bits = bits - high_bits;

	/*
	* high_bits hold the num of bits that we need to write in seq_h_attr
	* low_bits hold the num of bits that we need to write in seq_l_attr
	*/

	if (low_bits) {
		/* mask and set new value in seq_l_attr*/
		c3->sram.regs.seq_l_attr &= ~(((1 << low_bits) - 1)  << bits_offs);
		c3->sram.regs.seq_l_attr |= (id  << bits_offs);
	}

	if (high_bits) {
		int high_id = id >> low_bits;
		int high_offs = (low_bits == 0) ? (bits_offs % DWORD_BITS_LEN) : 0;

		/* mask and set new value in seq_h_attr*/
		c3->sram.regs.seq_h_attr &= ~(((1 << high_bits) - 1)  << high_offs);
		c3->sram.regs.seq_h_attr |= (high_id << high_offs);
	}

	return 0;
}

/*-------------------------------------------------------------------------------*/
/*	APIs for Classification C3 Hit counters management				  */
/*-------------------------------------------------------------------------------*/

int pp2_cls_c3_hit_cntrs_clear(uintptr_t cpu_slot, int lkp_type)
{
	/* clear all counters that entry lookup type corresponding to lkp_type */
	int iter = 0;

	if (mv_pp2x_range_validate(lkp_type, 0, KEY_CTRL_LKP_TYPE_MAX))
		return -EINVAL;

	pp2_reg_write(cpu_slot, MVPP2_CLS3_CLEAR_COUNTERS_REG, lkp_type);

	/* wait to clear het counters done bit */
	while (!pp2_cls_c3_hit_cntr_clear_done(cpu_slot))
		if (++iter >= RETRIES_EXCEEDED) {
			pr_err("%s:Error - retries exceeded.\n", __func__);
			return -EBUSY;
		}

	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_hit_cntrs_clear_all(uintptr_t cpu_slot)
{
	int iter = 0;

	pp2_reg_write(cpu_slot, MVPP2_CLS3_CLEAR_COUNTERS_REG, MVPP2_CLS3_CLEAR_ALL);
	/* wait to clear het counters done bit */
	while (!pp2_cls_c3_hit_cntr_clear_done(cpu_slot))
		if (++iter >= RETRIES_EXCEEDED) {
			pr_err("%s:Error - retries exceeded.\n", __func__);
			return -EBUSY;
		}

	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_hit_cntrs_read(uintptr_t cpu_slot, int index, u32 *cntr)
{
	u32 counter;

	if (mv_pp2x_range_validate(index, 0, MVPP2_CLS3_HASH_OP_TBL_ADDR_MAX))
		return -EINVAL;

	/*write entry index*/
	pp2_reg_write(cpu_slot, MVPP2_CLS3_DB_INDEX_REG, index);

	/*counter read*/
	counter = pp2_reg_read(cpu_slot, MVPP2_CLS3_HIT_COUNTER_REG) & MVPP2_CLS3_HIT_COUNTER_MASK;

	if (!cntr)
		pr_info("ADDR:0x%3.3x	COUNTER VAL:0x%6.6x\n", index, counter);
	else
		*cntr = counter;
	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_hit_cntrs_miss_read(uintptr_t cpu_slot, int lkp_type, u32 *cntr)
{
	u32 counter;
	int index;

	if (mv_pp2x_range_validate(lkp_type, 0, MVPP2_CLS_C3_MISS_TBL_SIZE - 1))
		return -EINVAL;

	/*set miss bit to 1, ppv2.1 mas 3.16*/
	index = (lkp_type | MVPP2_CLS3_DB_MISS_MASK);

	/*write entry index*/
	pp2_reg_write(cpu_slot, MVPP2_CLS3_DB_INDEX_REG, index);

	/*counter read*/
	counter = pp2_reg_read(cpu_slot, MVPP2_CLS3_HIT_COUNTER_REG) & MVPP2_CLS3_HIT_COUNTER_MASK;

	if (!cntr)
		pr_info("LKPT:0x%3.3x	COUNTER VAL:0x%6.6x\n", lkp_type, counter);
	else
		*cntr = counter;
	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_hit_cntrs_read_all(uintptr_t cpu_slot)
{
	u32 counter, index;

	for (index = 0; index < MVPP2_CLS_C3_HASH_TBL_SIZE; index++) {
		pp2_cls_c3_hit_cntrs_read(cpu_slot, index, &counter);

		/* skip initial counter value */
		if (counter == 0)
			continue;

		pr_info("ADDR:0x%3.3x	COUNTER VAL:0x%6.6x\n", index, counter);
	}

	for (index = 0; index < MVPP2_CLS_C3_MISS_TBL_SIZE; index++) {
		pp2_cls_c3_hit_cntrs_miss_read(cpu_slot, index, &counter);

		/* skip initial counter value */
		if (counter == 0)
			continue;

		pr_info("LKPT:0x%3.3x	COUNTER VAL:0x%6.6x\n", index, counter);
	}
	return 0;
}

/*-------------------------------------------------------------------------------*/
/*	 APIs for Classification C3 hit counters scan fields operation			  */
/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_scan_start(uintptr_t cpu_slot)
{
	int complete, iter = 0;

	/* trigger scan operation */
	pp2_reg_write(cpu_slot, MVPP2_CLS3_SC_ACT_REG, (1 << MVPP2_CLS3_SC_ACT));

	do {
		complete = pp2_cls_c3_scan_complete(cpu_slot);

	} while ((!complete) && ((iter++) < RETRIES_EXCEEDED));/*scan compleated*/

	if (iter >= RETRIES_EXCEEDED)
		return -EBUSY;

	return 0;
}

/*-------------------------------------------------------------------------------*/
/*mod = 0 below th . mode = 1 above threshold*/
int pp2_cls_c3_scan_thresh_set(uintptr_t cpu_slot, int mode, int thresh)
{
	u32 reg_val;

	if (mv_pp2x_range_validate(mode, 0, 1))
		return -EINVAL;

	if (mv_pp2x_range_validate(thresh, 0, MVPP2_CLS3_SC_TH_MAX))
		return -EINVAL;

	reg_val = pp2_reg_read(cpu_slot, MVPP2_CLS3_SC_PROP_REG);
	reg_val &= ~MVPP2_CLS3_SC_PROP_TH_MODE_MASK;
	reg_val |= (mode << MVPP2_CLS3_SC_PROP_TH_MODE);
	pp2_reg_write(cpu_slot, MVPP2_CLS3_SC_PROP_REG, reg_val);

	reg_val = pp2_reg_read(cpu_slot, MVPP2_CLS3_SC_TH_REG);
	reg_val &= ~MVPP2_CLS3_SC_TH_MASK;
	reg_val |= (thresh << MVPP2_CLS3_SC_TH);
	pp2_reg_write(cpu_slot, MVPP2_CLS3_SC_TH_REG, reg_val);

	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_scan_lkp_type_set(uintptr_t cpu_slot, int type)
{
	u32 prop;

	if (mv_pp2x_range_validate(type, -1, MVPP2_CLS3_SC_PROP_LKP_TYPE_MAX))
		return -EINVAL;

	prop = pp2_reg_read(cpu_slot, MVPP2_CLS3_SC_PROP_REG);

	if (type == -1)
		/* scan all entries */
		prop &= ~(1 << MVPP2_CLS3_SC_PROP_LKP_TYPE_EN);
	else {
		/* scan according to lookup type */
		prop |= (1 << MVPP2_CLS3_SC_PROP_LKP_TYPE_EN);
		prop &= ~MVPP2_CLS3_SC_PROP_LKP_TYPE_MASK;
		prop |= (type << MVPP2_CLS3_SC_PROP_LKP_TYPE);
	}

	pp2_reg_write(cpu_slot, MVPP2_CLS3_SC_PROP_REG, prop);

	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_scan_clear_before_en_set(uintptr_t cpu_slot, int en)
{
	u32 prop;

	if (mv_pp2x_range_validate(en, 0, 1))
		return -EINVAL;

	prop = pp2_reg_read(cpu_slot, MVPP2_CLS3_SC_PROP_REG);

	prop &= ~MVPP2_CLS3_SC_PROP_CLEAR_MASK;
	prop |= (en << MVPP2_CLS3_SC_PROP_CLEAR);

	pp2_reg_write(cpu_slot, MVPP2_CLS3_SC_PROP_REG, prop);

	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_scan_start_index_set(uintptr_t cpu_slot, int idx)
{
	u32 prop;

	if (mv_pp2x_range_validate(idx, 0, MVPP2_CLS3_HASH_OP_TBL_ADDR_MAX))
		return -EINVAL;

	prop = pp2_reg_read(cpu_slot, MVPP2_CLS3_SC_PROP_REG);

	prop &= ~MVPP2_CLS3_SC_PROP_START_ENTRY_MASK;
	prop |= (idx << MVPP2_CLS3_SC_PROP_START_ENTRY);

	pp2_reg_write(cpu_slot, MVPP2_CLS3_SC_PROP_REG, prop);

	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_scan_delay_set(uintptr_t cpu_slot, u32 time)
{
	u32 prop_val;

	if (mv_pp2x_range_validate(time, 0, MVPP2_CLS3_SC_PROP_VAL_DELAY_MAX))
		return -EINVAL;

	prop_val = pp2_reg_read(cpu_slot, MVPP2_CLS3_SC_PROP_VAL_REG);
	prop_val &= ~MVPP2_CLS3_SC_PROP_VAL_DELAY_MASK;
	prop_val |= (time << MVPP2_CLS3_SC_PROP_VAL_DELAY);
	pp2_reg_write(cpu_slot, MVPP2_CLS3_SC_PROP_VAL_REG, prop_val);

	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_scan_res_read(uintptr_t cpu_slot, int index, int *addr, int *cnt)
{
	u32 reg_val, sc_state, address, counter;
	int iter = 0;

	if (mv_pp2x_range_validate(index, 0, MVPP2_CLS_C3_SC_RES_TBL_SIZE - 1))
		return -EINVAL;

	do {
		pp2_cls_c3_scan_state_get(cpu_slot, &sc_state);
	} while (sc_state != 0 && ((iter++) < RETRIES_EXCEEDED));/*scan compleated*/

	if (iter >= RETRIES_EXCEEDED) {
		pr_err("%s:Error - retries exceeded.\n", __func__);
		return -EBUSY;
	}

	/*write index*/
	pp2_reg_write(cpu_slot, MVPP2_CLS3_SC_INDEX_REG, index);

	/*read date*/
	reg_val = pp2_reg_read(cpu_slot, MVPP2_CLS3_SC_RES_REG);
	address = (reg_val & MVPP2_CLS3_SC_RES_ENTRY_MASK) >> MVPP2_CLS3_SC_RES_ENTRY;
	counter = (reg_val & MVPP2_CLS3_SC_RES_CTR_MASK) >> MVPP2_CLS3_SC_RES_CTR;
	/* if one of parameters is null - func call from sysfs*/
	if ((!addr) | (!cnt)) {
		pr_info("INDEX:0x%2.2x	ADDR:0x%3.3x	COUNTER VAL:0x%6.6x\n", index, address, counter);
	} else {
		*addr = address;
		*cnt = counter;
	}

	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_scan_num_of_res_get(uintptr_t cpu_slot, int *res_num)
{
	u32 reg_val, sc_state;
	int iter = 0;

	do {
		pp2_cls_c3_scan_state_get(cpu_slot, &sc_state);
	} while (sc_state != 0 && ((iter++) < RETRIES_EXCEEDED));/*scan compleated*/

	if (iter >= RETRIES_EXCEEDED) {
		pr_err("%s:Error - retries exceeded.\n", __func__);
		return -EBUSY;
	}

	reg_val = pp2_reg_read(cpu_slot, MVPP2_CLS3_STATE_REG);
	reg_val &= MVPP2_CLS3_STATE_NO_OF_SC_RES_MASK;
	reg_val >>= MVPP2_CLS3_STATE_NO_OF_SC_RES;
	*res_num = reg_val;
	return 0;
}

/*-------------------------------------------------------------------------------*/

/* *INDENT-ON* */
