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
 * @file pp2_hw_cls_debug.c
 *
 * PPDK container structures and packet processor initialization
 */

#include "std_internal.h"

#include "../pp2.h"
#include "pp2_hw_cls.h"
#include "pp2_hw_cls_dbg.h"
#include "../pp2_hw_type.h"

/*-------------------------------------------------------------------------------*/
/*	C3 DUMPS functions							  */
/*-------------------------------------------------------------------------------*/
static int pp2_cls_c3_sw_act_dump(struct pp2_cls_c3_entry *c3);

/*-------------------------------------------------------------------------------*/
/*
* All miss entries are valid,
* the key+heks in miss entries are hot in use and this is the
* reason that we dump onlt action table fields
*/
int pp2_cls_c3_hw_miss_dump(uintptr_t cpu_slot)
{
	int index;
	struct pp2_cls_c3_entry c3;

	pp2_cls_c3_sw_clear(&c3);

	for (index = 0; index < MVPP2_CLS_C3_MISS_TBL_SIZE; index++) {
		pp2_cls_c3_hw_miss_read(cpu_slot, &c3, index);
		printk("INDEX[0x%3.3X]\n", index);
		pp2_cls_c3_sw_act_dump(&c3);
		printk("----------------------------------------------------------------------\n");
	}

	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_hw_ext_dump(uintptr_t cpu_slot)
{
	int index, i;
	u32 hash_ext_data[MVPP2_CLS3_HASH_EXT_DATA_REG_NUM];

	printk("INDEX    DATA\n");

	for (index = 0; index <  MVPP2_CLS_C3_EXT_TBL_SIZE; index++)
		if (pp2_cls_c3_shadow_ext_status_get(index) == IN_USE) {
			/* write extension index */
			pp2_reg_write(cpu_slot, MVPP2_CLS3_DB_INDEX_REG, index);

			/* read hash extesion data*/
			for (i = 0; i < MVPP2_CLS3_HASH_EXT_DATA_REG_NUM; i++)
				hash_ext_data[i] = pp2_reg_read(cpu_slot, MVPP2_CLS3_HASH_EXT_DATA_REG(i));

			printk("[0x%2.2x] %8.8x %8.8x %8.8x %8.8x %8.8x %8.8x %8.8x\n",
				index, hash_ext_data[6], hash_ext_data[5], hash_ext_data[4],
				 hash_ext_data[3], hash_ext_data[2], hash_ext_data[1], hash_ext_data[0]);
		} /* if */

	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_sw_dump(struct pp2_cls_c3_entry *c3)
{
	int hek_size;

	if (mv_pp2x_ptr_validate(c3))
		return -EINVAL;

	printk("\n");
	printk("INDEX[0x%3.3x] ", c3->index);

	hek_size = ((c3->key.key_ctrl & KEY_CTRL_HEK_SIZE_MASK) >> KEY_CTRL_HEK_SIZE);

	/* print extension index if exist*/
	if (hek_size > MVPP2_CLS_C3_HEK_BYTES)
		/* extension */
		printk("EXT_INDEX[0x%2.2x] ", c3->ext_index);
	else
		/* without extension */
		printk("EXT_INDEX[ NA ] ");

	printk("SIZE[0x%2.2x] ", hek_size);
	printk("PRT[ID = 0x%2.2x,TYPE = 0x%1.1x] ", ((c3->key.key_ctrl & KEY_CTRL_PRT_ID_MASK) >> KEY_CTRL_PRT_ID),
		((c3->key.key_ctrl & KEY_CTRL_PRT_ID_TYPE_MASK) >> KEY_CTRL_PRT_ID_TYPE));

	printk("LKP_TYPE[0x%1.1x] ", ((c3->key.key_ctrl & KEY_CTRL_LKP_TYPE_MASK) >> KEY_CTRL_LKP_TYPE));

	printk("L4INFO[0x%1.1x] ", ((c3->key.key_ctrl & KEY_CTRL_L4_MASK) >> KEY_CTRL_L4));

	printk("\n\n");
	printk("HEK	");
	if (hek_size > MVPP2_CLS_C3_HEK_BYTES)
		/* extension */
		printk(HEK_EXT_FMT, HEK_EXT_VAL(c3->key.hek.words));
	else
		/* without extension */
		printk(HEK_FMT, HEK_VAL(c3->key.hek.words));
	printk("\n");
	return pp2_cls_c3_sw_act_dump(c3);
}

/*-------------------------------------------------------------------------------*/
static int pp2_cls_c3_sw_act_dump(struct pp2_cls_c3_entry *c3)
{
	if (mv_pp2x_ptr_validate(c3))
		return -EINVAL;

	printk("\n");

	/*------------------------------*/
	/*	actions 0x1D40		*/
	/*------------------------------*/

	printk("ACT_TBL: COLOR   LOW_Q   HIGH_Q     FWD   POLICER  FID\n");
	printk("CMD:     [%1d]      [%1d]    [%1d]        [%1d]   [%1d]      [%1d]\n",
		(u32)((c3->sram.regs.actions & (MVPP2_CLS3_ACT_COLOR_MASK)) >> MVPP2_CLS3_ACT_COLOR),
		(u32)((c3->sram.regs.actions & (MVPP2_CLS3_ACT_LOW_Q_MASK)) >> MVPP2_CLS3_ACT_LOW_Q),
		(u32)((c3->sram.regs.actions & (MVPP2_CLS3_ACT_HIGH_Q_MASK)) >> MVPP2_CLS3_ACT_HIGH_Q),
		(u32)((c3->sram.regs.actions & MVPP2_CLS3_ACT_FWD_MASK) >> MVPP2_CLS3_ACT_FWD),
		(u32)((c3->sram.regs.actions & (MVPP2_CLS3_ACT_POLICER_SELECT_MASK)) >> MVPP2_CLS3_ACT_POLICER_SELECT),
		(u32)((c3->sram.regs.actions & MVPP2_CLS3_ACT_FLOW_ID_EN_MASK) >> MVPP2_CLS3_ACT_FLOW_ID_EN));

	printk("VAL:              [%1d]    [0x%x]\n",
		((c3->sram.regs.qos_attr & (MVPP2_CLS3_ACT_QOS_ATTR_LOW_Q_MASK)) >> MVPP2_CLS3_ACT_QOS_ATTR_LOW_Q),
		 ((c3->sram.regs.qos_attr & (MVPP2_CLS3_ACT_QOS_ATTR_HIGH_Q_MASK)) >> MVPP2_CLS3_ACT_QOS_ATTR_HIGH_Q));

	printk("\n");
	/*------------------------------*/
	/*	hwf_attr 0x1D48		*/
	/*------------------------------*/

	printk("HWF_ATTR: IPTR	DPTR	 CHKSM     MTU_IDX\n");
	printk("          0x%1.1x   0x%4.4x   %s   0x%1.1x\n",
		(u32)((c3->sram.regs.hwf_attr & MVPP2_CLS3_ACT_HWF_ATTR_IPTR_MASK) >> MVPP2_CLS3_ACT_HWF_ATTR_IPTR),
		(u32)((c3->sram.regs.hwf_attr & MVPP2_CLS3_ACT_HWF_ATTR_DPTR_MASK) >> MVPP2_CLS3_ACT_HWF_ATTR_DPTR),
		((u32)((c3->sram.regs.hwf_attr & MVPP2_CLS3_ACT_HWF_ATTR_CHKSM_EN_MASK) >>
		MVPP2_CLS3_ACT_HWF_ATTR_CHKSM_EN) ? "ENABLE" : "DISABLE"),
		(u32)((c3->sram.regs.hwf_attr & MVPP2_CLS3_ACT_HWF_ATTR_MTU_INX_MASK) >>
		MVPP2_CLS3_ACT_HWF_ATTR_MTU_INX));
	printk("\n");
	/*------------------------------*/
	/*	dup_attr 0x1D4C		*/
	/*------------------------------*/
	printk("DUP_ATTR:FID	COUNT	POLICER [id    bank]\n");
	printk("         0x%2.2x\t0x%1.1x\t\t[0x%2.2x   0x%1.1x]\n",
		(u32)((c3->sram.regs.dup_attr & MVPP2_CLS3_ACT_DUP_FID_MASK) >> MVPP2_CLS3_ACT_DUP_FID),
		(u32)((c3->sram.regs.dup_attr & MVPP2_CLS3_ACT_DUP_COUNT_MASK) >> MVPP2_CLS3_ACT_DUP_COUNT),
		(u32)((c3->sram.regs.dup_attr & MVPP2_CLS3_ACT_DUP_POLICER_MASK) >> MVPP2_CLS3_ACT_DUP_POLICER_ID),
		(u32)((c3->sram.regs.dup_attr & MVPP2_CLS3_ACT_DUP_POLICER_BANK_MASK) >>
		   MVPP2_CLS3_ACT_DUP_POLICER_BANK_BIT));
	printk("\n");
	printk("SEQ_ATTR: HIGH[32:37] LOW[0:31]\n");
	printk("          0x%2.2x        0x%8.8x", c3->sram.regs.seq_h_attr, c3->sram.regs.seq_l_attr);

	printk("\n\n");

	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_hw_dump(uintptr_t cpu_slot)
{
	int index;
	struct pp2_cls_c3_entry c3;

	pp2_cls_c3_sw_clear(&c3);

	for (index = 0; index < MVPP2_CLS_C3_HASH_TBL_SIZE; index++) {
		int size, ext_idx;

		pp2_cls_c3_shadow_get(index, &size, &ext_idx);
		if (size > 0) {
			pp2_cls_c3_hw_read(cpu_slot, &c3, index);
			pp2_cls_c3_sw_dump(&c3);
			printk("----------------------------------------------------------------------\n");
		}
	}

	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_scan_regs_dump(uintptr_t cpu_slot)
{
	u32 prop, prop_val;
	u32 treshhold;

	treshhold = pp2_reg_read(cpu_slot, MVPP2_CLS3_SC_TH_REG);

	prop = pp2_reg_read(cpu_slot, MVPP2_CLS3_SC_PROP_REG);
	prop_val = pp2_reg_read(cpu_slot, MVPP2_CLS3_SC_PROP_VAL_REG);

	printk("%-32s: 0x%x = 0x%08x\n", "MVPP2_CLS3_SC_PROP_REG", MVPP2_CLS3_SC_PROP_REG, prop);
	printk("%-32s: 0x%x = 0x%08x\n", "MVPP2_CLS3_SC_PROP_VAL_REG", MVPP2_CLS3_SC_PROP_VAL_REG, prop_val);
	printk("\n");

	printk("MODE      = %s\n", ((MVPP2_CLS3_SC_PROP_TH_MODE_MASK & prop) == 0) ? "Below" : "Above");
	printk("CLEAR     = %s\n", ((MVPP2_CLS3_SC_PROP_CLEAR_MASK & prop) == 0) ? "NoClear" : "Clear  ");

	/* lookup type */
	if ((MVPP2_CLS3_SC_PROP_LKP_TYPE_EN_MASK & prop) == 0)
		printk("LKP_TYPE  = NA\n");
	else
		printk("LKP_TYPE  = 0x%x\n",
			((MVPP2_CLS3_SC_PROP_LKP_TYPE_MASK & prop) >> MVPP2_CLS3_SC_PROP_LKP_TYPE));

	/* start index */
	printk("START     = 0x%x\n", (MVPP2_CLS3_SC_PROP_START_ENTRY_MASK & prop) >> MVPP2_CLS3_SC_PROP_START_ENTRY);
	/* threshold */
	printk("THRESHOLD = 0x%x\n", (MVPP2_CLS3_SC_TH_MASK & treshhold) >> MVPP2_CLS3_SC_TH);

	/* delay value */
	printk("DELAY     = 0x%x\n\n",
		(MVPP2_CLS3_SC_PROP_VAL_DELAY_MASK & prop_val) >> MVPP2_CLS3_SC_PROP_VAL_DELAY);

	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_scan_res_dump(uintptr_t cpu_slot)
{
	int addr, cnt, res_num, index;

	pp2_cls_c3_scan_num_of_res_get(cpu_slot, &res_num);

	printk("INDEX	ADDRESS		COUNTER\n");
	for (index = 0; index < res_num; index++) {
		pp2_cls_c3_scan_res_read(cpu_slot, index, &addr, &cnt);
		printk("[0x%2.2x]\t[0x%3.3x]\t[0x%6.6x]\n", index, addr, cnt);
	}

	return 0;
}

/* *INDENT-ON* */
