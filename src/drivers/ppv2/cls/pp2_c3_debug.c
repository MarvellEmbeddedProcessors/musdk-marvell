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
 * @file pp2_c3_debug.c
 *
 * C3 High level debug routines (dump functions and external configuration)
 */

/***********************/
/* c file declarations */
/***********************/
#include "std_internal.h"
#include "../pp2_types.h"
#include "../pp2.h"
#include "../pp2_hw_type.h"
#include "pp2_hw_cls.h"

void pp2_cls_c3_entry_header_dump(void)
{
	print_horizontal_line(100, "=");
	printk("LkTp |PrtInf|L4_Inf|   HEK                  |   Action_Info    | QOS_Info    |   Idx   |Hit_Cnt\n");
	print_horizontal_line(100, "=");
}

/*******************************************************************************
 * pp2_cls_c3_entry_convert
 *
 * DESCRIPTION: The routine will convert PP2 C3 entry to management C3 entry.
 * INPUTS:
 *	pp2_entry - PP2 C3 entry
 *
 * OUTPUTS:
 *	mng_entry - management C3 entry
 *
 ******************************************************************************/
void pp2_cls_c3_entry_convert(struct pp2_cls_c3_entry *pp2_entry, struct pp2_cls_c3_data_t *mng_entry)
{
	int idx;
	/* clear */
	memset(mng_entry, 0, sizeof(struct pp2_cls_c3_data_t));

	/* convert key control and value field */
	mng_entry->port.port_type   = (pp2_entry->key.key_ctrl & KEY_CTRL_PRT_ID_TYPE_MASK) >> KEY_CTRL_PRT_ID_TYPE;
	mng_entry->port.port_value  = (pp2_entry->key.key_ctrl & KEY_CTRL_PRT_ID_MASK) >> KEY_CTRL_PRT_ID;
	mng_entry->lkp_type         = (pp2_entry->key.key_ctrl & KEY_CTRL_LKP_TYPE_MASK) >> KEY_CTRL_LKP_TYPE;
	mng_entry->l4_type          = (pp2_entry->key.key_ctrl & KEY_CTRL_L4_MASK) >> KEY_CTRL_L4;
	mng_entry->hek_len          = (pp2_entry->key.key_ctrl & KEY_CTRL_HEK_SIZE_MASK) >> KEY_CTRL_HEK_SIZE;
	for (idx = 0; idx < MVPP2_C3_MAX_HASH_KEY_SIZE; idx++)
		mng_entry->hek[MVPP2_C3_MAX_HASH_KEY_SIZE - 1 - idx] = pp2_entry->key.hek.bytes[idx];

	/* convert action filed */
	mng_entry->action.color_act   = (pp2_entry->sram.regs.actions & MVPP2_CLS3_ACT_COLOR_MASK) >>
					 MVPP2_CLS3_ACT_COLOR;
	mng_entry->action.q_low_act   = (pp2_entry->sram.regs.actions & MVPP2_CLS3_ACT_LOW_Q_MASK) >>
					 MVPP2_CLS3_ACT_LOW_Q;
	mng_entry->action.q_high_act  = (pp2_entry->sram.regs.actions & MVPP2_CLS3_ACT_HIGH_Q_MASK) >>
					 MVPP2_CLS3_ACT_HIGH_Q;
	mng_entry->action.policer_act = (pp2_entry->sram.regs.actions & MVPP2_CLS3_ACT_POLICER_SELECT_MASK) >>
					 MVPP2_CLS3_ACT_POLICER_SELECT;
	mng_entry->action.flowid_act  = (pp2_entry->sram.regs.actions & MVPP2_CLS3_ACT_FLOW_ID_EN_MASK) >>
					 MVPP2_CLS3_ACT_FLOW_ID_EN;
	mng_entry->action.frwd_act    = (pp2_entry->sram.regs.actions & MVPP2_CLS3_ACT_FWD_MASK) >>
					 MVPP2_CLS3_ACT_FWD;

	/* convert queue low, queue high, policer ID */
	mng_entry->qos_value.q_low    = (pp2_entry->sram.regs.qos_attr & MVPP2_CLS3_ACT_QOS_ATTR_LOW_Q_MASK) >>
					 MVPP2_CLS3_ACT_QOS_ATTR_LOW_Q;
	mng_entry->qos_value.q_high   = (pp2_entry->sram.regs.qos_attr & MVPP2_CLS3_ACT_QOS_ATTR_HIGH_Q_MASK) >>
					 MVPP2_CLS3_ACT_QOS_ATTR_HIGH_Q;

	/* convert modification */
	mng_entry->pkt_mod.mod_data_idx = (pp2_entry->sram.regs.hwf_attr & MVPP2_CLS3_ACT_HWF_ATTR_DPTR_MASK) >>
					   MVPP2_CLS3_ACT_HWF_ATTR_DPTR;
	mng_entry->pkt_mod.mod_cmd_idx = (pp2_entry->sram.regs.hwf_attr & MVPP2_CLS3_ACT_HWF_ATTR_IPTR_MASK) >>
					  MVPP2_CLS3_ACT_HWF_ATTR_IPTR;
	mng_entry->pkt_mod.l4_chksum_update_flag = (pp2_entry->sram.regs.hwf_attr &
						    MVPP2_CLS3_ACT_HWF_ATTR_CHKSM_EN_MASK) >>
						    MVPP2_CLS3_ACT_HWF_ATTR_CHKSM_EN;

	/* convert duplication */
	mng_entry->policer_id = (pp2_entry->sram.regs.dup_attr & MVPP2_CLS3_ACT_DUP_POLICER_MASK) >>
				 MVPP2_CLS3_ACT_DUP_POLICER_ID;
	mng_entry->dup_info.flow_id = (pp2_entry->sram.regs.dup_attr & MVPP2_CLS3_ACT_DUP_FID_MASK) >>
				       MVPP2_CLS3_ACT_DUP_FID;
	mng_entry->dup_info.flow_cnt = (pp2_entry->sram.regs.dup_attr & MVPP2_CLS3_ACT_DUP_COUNT_MASK) >>
					MVPP2_CLS3_ACT_DUP_COUNT;
}

/*******************************************************************************
 * pp2_cls_c3_entry_line_dump
 *
 * DESCRIPTION: Print one line of C3 entry info.
 * INPUTS:
 *	dump_idx  - index used to dump field value
 *	hash_iex  - multihash index of C3 HW
 *	logic_idx - logical index
 *	hit_count - hit counter
 *	c3_entry  - C3 entry information
 *
 * OUTPUTS:
 *           None
 ******************************************************************************/
void pp2_cls_c3_entry_line_dump(u32 dump_idx, u32 hash_idx, u32 logic_idx,
				u32 hit_count, struct pp2_cls_c3_data_t *c3_entry)
{
	char lookup_type_str[4] = "";
	char port_info_str[7] = "";
	char l4_info_str[7] = "";
	char hek_str[26] = "";
	char qos_info_str[13] = "";
	char action_str[18] = "";
	char index_str[9] = "";
	char hit_cnt_str[8] = "";
	u32 len;
	u32 idx;

	/* set lkp_Type, L4_Info, Hit_Count for the first time */
	if (dump_idx == 0) {
		sprintf(lookup_type_str, "%d", c3_entry->lkp_type);
		sprintf(l4_info_str, "%s", pp2_cls_utils_l4_type_str_get(c3_entry->l4_type));
		sprintf(hit_cnt_str, "%d", hit_count);
	}

	/* port */
	switch (dump_idx) {
	case MVPP2_C3_PORT_DUMP_TYPE:
		sprintf(port_info_str, "T:%s", pp2_cls_utils_port_type_str_get(c3_entry->port.port_type));
		break;
	case MVPP2_C3_PORT_DUMP_VALUE:
		sprintf(port_info_str, "V:0x%x", c3_entry->port.port_value);
		break;
	default:
		break;
	}

	/* HEK info */
	switch (dump_idx) {
	case MVPP2_C3_HEK_DUMP_LEN:
		sprintf(hek_str, "Len:%d", c3_entry->hek_len);
		break;
	case MVPP2_C3_HEK_DUMP_KEY1:
		len = (c3_entry->hek_len > MVPP2_CLS_C3_HEK_BYTES) ? MVPP2_CLS_C3_HEK_BYTES : c3_entry->hek_len;
		for (idx = 0; idx < len; idx++)
			sprintf(hek_str + 2 * idx, "%02x", c3_entry->hek[idx]);

		break;
	case MVPP2_C3_HEK_DUMP_KEY2:
		if (c3_entry->hek_len > MVPP2_CLS_C3_HEK_BYTES) {
			len = ((c3_entry->hek_len - MVPP2_CLS_C3_HEK_BYTES) > MVPP2_CLS_C3_HEK_BYTES) ?
				MVPP2_CLS_C3_HEK_BYTES : (c3_entry->hek_len - MVPP2_CLS_C3_HEK_BYTES);
			for (idx = 0; idx < len; idx++)
				sprintf(hek_str + 2 * idx, "%02x", c3_entry->hek[idx + MVPP2_CLS_C3_HEK_BYTES]);
		}
		break;
	case MVPP2_C3_HEK_DUMP_KEY3:
		if (c3_entry->hek_len > 2 * MVPP2_CLS_C3_HEK_BYTES) {
			len = ((c3_entry->hek_len - 2 * MVPP2_CLS_C3_HEK_BYTES) > MVPP2_CLS_C3_HEK_BYTES) ?
				MVPP2_CLS_C3_HEK_BYTES : (c3_entry->hek_len - 2 * MVPP2_CLS_C3_HEK_BYTES);
			for (idx = 0; idx < len; idx++)
				sprintf(hek_str + 2 * idx, "%02x", c3_entry->hek[idx + 2 * MVPP2_CLS_C3_HEK_BYTES]);
		}
		break;
	default:
		break;
	}

	/* action */
	switch (dump_idx) {
	case MVPP2_C3_ACT_DUMP_COLOR:
		sprintf(action_str, "COLOR: %s", pp2_cls_utils_qos_action_str_get(c3_entry->action.color_act));
		break;
	case MVPP2_C3_ACT_DUMP_QUEUE_LOW:
		sprintf(action_str, "Q_LOW: %s", pp2_cls_utils_common_action_str_get(c3_entry->action.q_low_act));
		break;
	case MVPP2_C3_ACT_DUMP_QUEUE_HIGH:
		sprintf(action_str, "Q_HIGH: %s", pp2_cls_utils_common_action_str_get(c3_entry->action.q_high_act));
		break;
	case MVPP2_C3_ACT_DUMP_FRWD:
		sprintf(action_str, "FRWD: %s", pp2_cls_utils_frwd_action_str_get(c3_entry->action.frwd_act));
		break;
	case MVPP2_C3_ACT_DUMP_POLICER:
		sprintf(action_str, "POLICER: %s", pp2_cls_utils_common_action_str_get(c3_entry->action.policer_act));
		break;
	case MVPP2_C3_ACT_DUMP_FLOWID:
		sprintf(action_str, "FLOWID: %s", pp2_cls_utils_flow_id_action_str_get(c3_entry->action.flowid_act));
		break;
	default:
		break;
	}

	/* QoS */
	switch (dump_idx) {
	case MVPP2_C3_QOS_DUMP_QUEUE_LOW:
		sprintf(qos_info_str, "Q_LOW=%d", c3_entry->qos_value.q_low);
		break;
	case MVPP2_C3_QOS_DUMP_QUEUE_HIGH:
		sprintf(qos_info_str, "Q_HIGH=%d", c3_entry->qos_value.q_high);
		break;
	case MVPP2_C3_QOS_DUMP_POLICER_ID:
		sprintf(qos_info_str, "POLICER_ID=%hd", c3_entry->policer_id);
		break;
	default:
		break;
	}

	/* Entry index, including HW index, logical index */
	switch (dump_idx) {
	case MVPP2_C3_INDEX_DUMP_HASH:
		sprintf(index_str, "HW:%04d", hash_idx);
		break;
	case MVPP2_C3_INDEX_DUMP_LOGICAL:
		sprintf(index_str, "LOG:%04d", logic_idx);
		break;
	default:
		break;
	}

	if (dump_idx <= MVPP2_C3_HEK_DUMP_KEY3 ||
	    dump_idx <= MVPP2_C3_ACT_DUMP_FLOWID ||
	    dump_idx <= MVPP2_C3_QOS_DUMP_QUEUE_HIGH ||
	    dump_idx <= MVPP2_C3_MOD_DUMP_L4_CHECKSUM ||
	    dump_idx <= MVPP2_C3_FLOW_DUMP_CNT ||
	    dump_idx <= MVPP2_INDEX_DUMP_LOGICAL) {
		printk("+%4s|%6s|%6s|%24s|%18s|%13s|%9s|%8s+\n",
		       lookup_type_str, port_info_str, l4_info_str, hek_str,
		       action_str, qos_info_str, index_str, hit_cnt_str);
	}
}

/*******************************************************************************
 * pp2_cls_c3_entry_dump
 *
 * DESCRIPTION: The routine will dump the valid C3 entries from C3 sub-module
 *              internal DB.
 * INPUTS:
 *	type  - dump type
 *	value - value according to type
 *
 * OUTPUTS:
 *	None
 ******************************************************************************/
static int pp2_cls_c3_entry_dump(struct pp2_inst *inst, u32 type, u32 value)
{
	struct pp2_cls_c3_entry c3;
	u32 idx;
	u32 num = 0;
	u32 dump_idx;
	u32 hash_idx;
	int logic_idx;
	struct pp2_cls_c3_data_t c3_entry;
	u32 hit_count;
	int rc = 0;
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);

	/* param verification */
	if (mv_pp2x_range_validate(type, 0, MVPP2_C3_ENTRY_DUMP_ALL))
		return -EINVAL;

	if ((type == MVPP2_C3_ENTRY_DUMP_LOGIC_IDX) || (type == MVPP2_C3_ENTRY_DUMP_HASH_IDX)) {
		if (mv_pp2x_range_validate(value, 0, MVPP2_CLS_C3_HASH_TBL_SIZE))
			return -EINVAL;
	} else if (type == MVPP2_C3_ENTRY_DUMP_LU_TYPE) {
		if (mv_pp2x_range_validate(value, 0, KEY_CTRL_LKP_TYPE_MAX))
			return -EINVAL;
	}

	if ((type == MVPP2_C3_ENTRY_DUMP_LOGIC_IDX) || (type == MVPP2_C3_ENTRY_DUMP_HASH_IDX)) {
		if (type == MVPP2_C3_ENTRY_DUMP_LOGIC_IDX) {
			rc = pp2_cls_db_c3_hash_idx_get(inst, value, &hash_idx);
			/* skip invalid entry */
			if (rc) {
				pr_err("logical index(%d) is invalid\n", value);
				return -EIO;
			}
			logic_idx = value;
		} else if (type == MVPP2_C3_ENTRY_DUMP_HASH_IDX) {
			rc = pp2_cls_db_c3_logic_idx_get(inst, value, &logic_idx);
			/* skip invalid entry */
			if (rc) {
				pr_err("hash index(%d) is invalid\n", value);
				return -EIO;
			}
			hash_idx = value;
		}

		/* print header */
		pp2_cls_c3_entry_header_dump();

		/* read multihash entry */
		rc = pp2_cls_c3_hw_read(cpu_slot, &c3, hash_idx);
		if (rc) {
			pr_err("failed to read C3 entry from HW\n");
			return -EIO;
		}

		/* convert entry */
		pp2_cls_c3_entry_convert(&c3, &c3_entry);

		/* read hit counter */
		rc = pp2_cls_c3_hit_count_get(inst, logic_idx, &hit_count);
		if (rc) {
			pr_err("failed to read hit counter\n");
			return -EIO;
		}

		/* print C3 entry */
		for (dump_idx = 0; dump_idx < sizeof(u32) * BYTE_BITS; dump_idx++)
			pp2_cls_c3_entry_line_dump(dump_idx, c3.index, logic_idx, hit_count, &c3_entry);

		print_horizontal_line(100, "-");
		printk("+");
	}	else if ((type == MVPP2_C3_ENTRY_DUMP_LU_TYPE) || (type == MVPP2_C3_ENTRY_DUMP_ALL)) {
		/* print header */
		pp2_cls_c3_entry_header_dump();

		/* read valid C3 entry from HW and dump them by lkp_type */
		for (idx = 0; idx < MVPP2_CLS_C3_HASH_TBL_SIZE; idx++) {
			rc = pp2_cls_db_c3_logic_idx_get(inst, idx, &logic_idx);
			/* skip invalid entry */
			if (rc)
				continue;

			/* read multihash entry */
			rc = pp2_cls_c3_hw_read(cpu_slot, &c3, idx);
			if (rc) {
				pr_err("failed to read hit counter\n");
				return -EIO;
			}

			/* convert entry */
			pp2_cls_c3_entry_convert(&c3, &c3_entry);

			/* skip C3 entry by lkp_type */
			if ((value != c3_entry.lkp_type) && (type != MVPP2_C3_ENTRY_DUMP_ALL))
				continue;

			/* read hit counter */
			rc = pp2_cls_c3_hit_count_get(inst, logic_idx, &hit_count);
			if (rc) {
				pr_err("failed to read hit counter\n");
				return -EIO;
			}

			/* print C3 entry */
			for (dump_idx = 0; dump_idx < sizeof(u32) * BYTE_BITS; dump_idx++)
				pp2_cls_c3_entry_line_dump(dump_idx, c3.index, logic_idx, hit_count, &c3_entry);

			print_horizontal_line(100, "-");
			printk("+\n");

			num++;
		}

		printk("Total Number=%d\n", num);
	}

	return 0;
}

/*******************************************************************************
 * pp2_cls_cli_c3_type_entry_dump
 *
 * DESCRIPTION:
 *       This function dumps C3 entries according to type
 ******************************************************************************/
int pp2_cls_cli_c3_type_entry_dump(void *arg, int argc, char *argv[])
{
	u32 type = MVPP2_C3_ENTRY_DUMP_ALL;
	u32 value = 0;
	struct pp2_inst *inst = (struct pp2_inst *)arg;
	int rc;

	if (argc != 1 && argc != 3) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	if (argc == 3) {
		rc = kstrtou32(argv[1], 10, &type);
		if (rc || (type > MVPP2_C3_DUMP_TYPE_MAX)) {
			pr_err("parsing fail, wrong input for argv[1] - type");
			return -EINVAL;
		}

		rc = kstrtou32(argv[2], 10, &value);
		if (rc) {
			pr_err("parsing fail, wrong input for argv[2] - value");
			return -EINVAL;
		}

		/* check if all the fields are initialized */
		if (type < 0) {
			pr_err("parsing fail, invalid --type\n");
			return -EINVAL;
		}
		if (value < 0) {
			pr_err("parsing fail, invalid --value\n");
			return -EINVAL;
		}
	}

	if (!pp2_cls_c3_entry_dump(inst, type, value))
		printk("OK\n");
	else
		printk("FAIL\n");

	return 0;
}

/*******************************************************************************
 * pp2_cls_c3_index_dump
 *
 * DESCRIPTION:
 *       This function dumps C3 index entries entries scan result
 ******************************************************************************/
static int pp2_cls_c3_index_dump(struct pp2_inst *inst, u32 type)
{
	int idx;
	u32 num = 0;
	char index_str[8] = "";
	char hash_idx_str[8] = "";
	char logic_idx_str[8] = "";
	u32 hash_idx;
	int logic_idx;
	int rc;

	/* dump multihash index table */
	print_horizontal_line(42, "=");
	printk("=         Multihash Index Table          =\n");
	print_horizontal_line(42, "=");
	printk("= Index  | Logic_Idx | Hash_Idx |  Valid =\n");

	for (idx = 0; idx < MVPP2_CLS_C3_HASH_TBL_SIZE; idx++) {
		rc = pp2_cls_db_c3_hash_idx_get(inst, idx, &hash_idx);
		if (rc == 0) {
			sprintf(index_str, "%d", num);
			sprintf(logic_idx_str, "%d", idx);
			sprintf(hash_idx_str, "%d", hash_idx);
			printk("= %6s | %9s | %8s |  %4s  =\n", index_str, logic_idx_str, hash_idx_str, "En");
			num++;
		} else if (type == MVPP2_C3_TABLE_DUMP_ALL) {
			sprintf(index_str, "%d", num);
			sprintf(logic_idx_str, "%d", idx);
			sprintf(hash_idx_str, "%d", 0);
			printk("= %6s | %9s | %8s |  %4s  =\n", index_str, logic_idx_str, hash_idx_str, "Dis");
			num++;
		}
	}
	print_horizontal_line(42, "=");
	printk("Total number=%d\n", num);
	printk("\n\n");

	print_horizontal_line(42, "=");
	printk("=         Logical Index Table            =\n");
	print_horizontal_line(42, "=");
	printk("= Index  | Hash_Idx | Logic_Idx |  Valid =\n");
	num = 0;
	for (idx = 0; idx < MVPP2_CLS_C3_HASH_TBL_SIZE; idx++) {
		rc = pp2_cls_db_c3_logic_idx_get(inst, idx, &logic_idx);
		if (rc == 0) {
			sprintf(index_str, "%d", num);
			sprintf(hash_idx_str, "%d", idx);
			sprintf(logic_idx_str, "%d", logic_idx);
			printk("= %6s | %8s | %9s |  %4s  =\n",	index_str, hash_idx_str, logic_idx_str,	"En");
			num++;
		} else if (type == MVPP2_C3_TABLE_DUMP_ALL) {
			sprintf(index_str, "%d", num);
			sprintf(hash_idx_str, "%d", idx);
			sprintf(logic_idx_str, "%d", 0);
			printk("= %6s | %8s | %9s |  %4s  =\n", index_str, hash_idx_str, logic_idx_str, "Dis");
			num++;
		}
	}
	print_horizontal_line(42, "=");
	printk("Total number=%d\n", num);
	return 0;
}

/*******************************************************************************
 * pp2_cls_cli_c3_index_entry_dump
 *
 * DESCRIPTION:
 *       This function dumps C3 index entries according to type
 ******************************************************************************/
int pp2_cls_cli_c3_index_entry_dump(void *arg, int argc, char *argv[])
{
	u32 index_entry;
	int rc;
	struct pp2_inst *inst = (struct pp2_inst *)arg;

	if (argc != 2) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	/* Get parameters */
	rc = kstrtou32(argv[1], 10, &index_entry);
	if (rc || (index_entry > MVPP2_C3_DUMP_INDEX_MAX)) {
		pr_err("parsing fail, wrong input for argv[1] - index_entry.\n");
		return -EINVAL;
	}

	if (!pp2_cls_c3_index_dump(inst, index_entry))
		printk("OK\n");
	else
		printk("FAIL\n");

	return 0;
}

/*******************************************************************************
 * pp2_cls_cli_c3_scan_param_set
 *
 * DESCRIPTION:
 *       This function set C3 scan parameters
 ******************************************************************************/
int pp2_cls_cli_c3_scan_param_set(void *arg, int argc, char *argv[])
{
	int rc;
	u32 clear;
	u32 lkp_type_en;
	u32 lkp_type;
	u32 mode;
	u32 start;
	u32 delay;
	u32 threshold;
	struct pp2_cls_c3_scan_config_t scan_config;
	struct pp2_inst *inst = (struct pp2_inst *)arg;

	if (argc != 8) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	/* Get parameters */
	rc = kstrtou32(argv[1], 10, &clear);
	if (rc || (clear > MVPP2_C3_SCAN_CLEAR_MAX)) {
		pr_err("parsing fail, wrong input for argv[1] - clear.\n");
		return -EINVAL;
	}

	rc = kstrtou32(argv[2], 10, &lkp_type_en);
	if (rc || (lkp_type_en > MVPP2_C3_SCAN_LKP_EN_MAX)) {
		pr_err("parsing fail, wrong input for argv[2] - lkp_type_en.\n");
		return -EINVAL;
	}

	rc = kstrtou32(argv[3], 10, &lkp_type);
	if (rc || (lkp_type > MVPP2_C3_SCAN_LKP_TYPE_MAX)) {
		pr_err("parsing fail, wrong input for argv[3] - lkp_type.\n");
		return -EINVAL;
	}

	rc = kstrtou32(argv[4], 10, &mode);
	if (rc || (mode > MVPP2_C3_SCAN_MODE_MAX)) {
		pr_err("parsing fail, wrong input for argv[4] - mode.\n");
		return -EINVAL;
	}

	rc = kstrtou32(argv[5], 10, &start);
	if (rc) {
		pr_err("parsing fail, wrong input for argv[5] - start.\n");
		return -EINVAL;
	}

	rc = kstrtou32(argv[6], 10, &delay);
	if (rc) {
		pr_err("parsing fail, wrong input for argv[6] - delay.\n");
		return -EINVAL;
	}

	rc = kstrtou32(argv[7], 10, &threshold);
	if (rc) {
		pr_err("parsing fail, wrong input for argv[7] - threshold.\n");
		return -EINVAL;
	}

	scan_config.clear_before_scan = clear;
	scan_config.lkp_type_scan = lkp_type_en;
	scan_config.lkp_type = lkp_type;
	scan_config.scan_mode = mode;
	scan_config.start_entry = start;
	scan_config.scan_delay = delay;
	scan_config.scan_threshold = threshold;
	if (!pp2_cls_c3_scan_param_set(inst, &scan_config))
		printk("OK\n");
	else
		printk("FAIL\n");

	return 0;
}

/*******************************************************************************
 * pp2_cls_c3_scan_result_dump
 *
 * DESCRIPTION:
 *       This function dumps C3 entries scan result
 ******************************************************************************/
static int pp2_cls_c3_scan_result_dump(struct pp2_inst *inst, u32 max_num)
{
	u32 idx;
	u32 entry_num;
	struct pp2_cls_c3_scan_entry_t *result_entry;
	char index_str[8] = "";
	char hash_idx_str[8] = "";
	char logic_idx_str[8] = "";
	char hit_cnt_str[8] = "";
	int rc;

	result_entry = kmalloc(128 * sizeof(struct pp2_cls_c3_scan_entry_t), GFP_KERNEL);
	if (!result_entry)
		return -ENOMEM;
	memset(result_entry, 0, 128 * sizeof(struct pp2_cls_c3_scan_entry_t));

	/* trigger and get scan result */
	rc = pp2_cls_c3_scan_result_get(inst, max_num, &entry_num, result_entry);
	if (rc) {
		pr_err("fail to get scan result\n");
		kfree(result_entry);
		return -EIO;
	}

	/* dump scan result info */
	print_horizontal_line(100, "=");
	printk("= Index | Hash_Idx | Logic_Idx |   Hit_Cnt  =\n");
	print_horizontal_line(100, "=");

	for (idx = 0; idx < entry_num; idx++) {
		sprintf(index_str, "%d", idx);
		sprintf(hash_idx_str, "%d", result_entry[idx].hash_idx);
		sprintf(logic_idx_str, "%d", result_entry[idx].logic_idx);
		sprintf(hit_cnt_str, "%d", result_entry[idx].hit_cnt);
		printk("= %8s | %8s | %8s |  %8s  =\n", index_str, hash_idx_str, logic_idx_str,	hit_cnt_str);
	}
	print_horizontal_line(100, "=");
	printk("Total Number:%d\n", entry_num);

	kfree(result_entry);
	return 0;
}

/*******************************************************************************
 * pp2_cls_cli_c3_scan_result_get
 *
 * DESCRIPTION:
 *       This function dumps C3 entries scan result
 ******************************************************************************/
int pp2_cls_cli_c3_scan_result_get(void *arg, int argc, char *argv[])
{
	int rc;
	u32 scan_num;
	struct pp2_inst *inst = (struct pp2_inst *)arg;

	if (argc != 2) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	/* Get parameters */
	rc = kstrtou32(argv[1], 10, &scan_num);
	if (rc) {
		pr_err("parsing fail, wrong input for argv[1] - scan_num");
		return -EINVAL;
	}

	if (!pp2_cls_c3_scan_result_dump(inst, scan_num))
		printk("OK\n");
	else
		printk("FAIL\n");
	return 0;
}

/*******************************************************************************
 * pp2_cls_cli_c3_hit_count_get
 *
 * DESCRIPTION:
 *       This function get C3 entry hit counter w/ specific logic index
 ******************************************************************************/
int pp2_cls_cli_c3_hit_count_get(void *arg, int argc, char *argv[])
{
	int rc;
	u32 logic_idx;
	u32 hit_cnt;
	struct pp2_inst *inst = (struct pp2_inst *)arg;

	if (argc != 2) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	/* Get parameters */
	rc = kstrtou32(argv[1], 10, &logic_idx);
	if (rc || (logic_idx > MVPP2_C3_INDEX_MAX)) {
		pr_err("parsing fail, wrong input for argv[1] - logic_idx");
		return -EINVAL;
	}

	if (!pp2_cls_c3_hit_count_get(inst, logic_idx, &hit_cnt)) {
		printk("Logical index(%d), hit counter=%d\n", logic_idx, hit_cnt);
		printk("%s success\n", __func__);
	} else {
		printk("%s fail\n", __func__);
	}

	return 0;
}

/*******************************************************************************
 * pp2_cls_cli_c3_search_depth_set
 *
 * DESCRIPTION:
 *       This function set C3 cuckoo search depth
 ******************************************************************************/
int pp2_cls_cli_c3_search_depth_set(void *arg, int argc, char *argv[])
{
	int rc;
	u32 search_depth;
	struct pp2_inst *inst = (struct pp2_inst *)arg;

	if (argc != 2) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	/* Get parameters */
	rc = kstrtou32(argv[1], 10, &search_depth);
	if (rc || (search_depth > MVPP2_C3_SEARCH_DEPTHX_MAX)) {
		pr_err("parsing fail, wrong input for argv[1] - search_depth");
		return -EINVAL;
	}

	if (!pp2_cls_db_c3_search_depth_set(inst, search_depth))
		printk("%s success\n", __func__);
	else
		printk("%s fail\n", __func__);

	return 0;
}

/*******************************************************************************
 * pp2_cls_cli_c3_rule_delete
 *
 * DESCRIPTION:
 *       This function removes an entry
 ******************************************************************************/
int pp2_cls_cli_c3_rule_delete(void *arg, int argc, char *argv[])
{
	int rc;
	u32 logic_idx;
	struct pp2_inst *inst = (struct pp2_inst *)arg;

	if (argc != 2) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	/* Get parameters */
	rc = kstrtou32(argv[1], 10, &logic_idx);
	if (rc || (logic_idx > MVPP2_C3_INDEX_MAX)) {
		pr_err("parsing fail, wrong input for argv[1] - logic_idx");
		return -EINVAL;
	}

	if (!pp2_cls_c3_rule_del(inst, logic_idx))
		printk("%s success\n", __func__);
	else
		printk("%s fail\n", __func__);

	return 0;
}

/*******************************************************************************
 * pp2_cls_cli_c3_rule_add
 *
 * DESCRIPTION:
 *       This function removes an entry
 ******************************************************************************/
int pp2_cls_cli_c3_rule_add(void *arg, int argc, char *argv[])
{
	struct pp2_inst *inst = (struct pp2_inst *)arg;
	int idx;
	struct pp2_cls_pkt_key_t pkt_key;
	struct pp2_cls_mng_pkt_key_t mng_pkt_key;
	struct pp2_cls_c3_add_entry_t c3_entry;
	u32 logic_idx;
	int rc = 0;

	/* init value */
	MVPP2_MEMSET_ZERO(pkt_key);
	MVPP2_MEMSET_ZERO(mng_pkt_key);
	MVPP2_MEMSET_ZERO(c3_entry);
	c3_entry.mng_pkt_key = &mng_pkt_key;
	c3_entry.mng_pkt_key->pkt_key = &pkt_key;

	/* set value */
	c3_entry.port.port_type = MVPP2_SRC_PORT_TYPE_PHY;
	c3_entry.port.port_value = 1;

	c3_entry.lkp_type = 1;
	c3_entry.mng_pkt_key->pkt_key->field_bm = MVPP2_MATCH_IPV4_5T;
		/*PP2_CLS_MATCH_IPV6_PKT | */
		/* MVPP2_MATCH_IP_PROTO | MVPP2_MATCH_IP_SRC | MVPP2_MATCH_IP_DST | MVPP2_MATCH_L4_SRC */
		/*| MVPP2_MATCH_L4_DST; */
	c3_entry.mng_pkt_key->pkt_key->ipvx_add.ip_proto = IPPROTO_TCP;
	c3_entry.mng_pkt_key->pkt_key->ipvx_add.ip_ver = 4;
	for (idx = 0; idx < 4; idx++)
		c3_entry.mng_pkt_key->pkt_key->ipvx_add.ip_src.ip_add.ipv4[idx] = idx;

	for (idx = 0; idx < 4; idx++)
		c3_entry.mng_pkt_key->pkt_key->ipvx_add.ip_dst.ip_add.ipv4[idx] = 0xf0 + idx;

	c3_entry.mng_pkt_key->pkt_key->l4_dst = 0x1234;
	c3_entry.mng_pkt_key->pkt_key->l4_src = 0x5678;

	c3_entry.qos_info.policer_id = 0;

	c3_entry.action.color_act = MVPP2_COLOR_ACTION_TYPE_NO_UPDT;
	c3_entry.action.q_low_act = MVPP2_ACTION_TYPE_UPDT_LOCK;
	c3_entry.action.q_high_act = MVPP2_ACTION_TYPE_UPDT_LOCK;
	c3_entry.action.policer_act = MVPP2_ACTION_TYPE_UPDT_LOCK;
	c3_entry.action.flowid_act = MVPP2_ACTION_FLOWID_ENABLE;
	c3_entry.action.frwd_act = MVPP2_FRWD_ACTION_TYPE_HWF_LOCK;

	c3_entry.qos_value.q_high = 5;
	c3_entry.qos_value.q_low = 2;

	/* add rule */
	for (idx = 0; idx < 1; idx++) {
		rc = pp2_cls_c3_rule_add(inst, &c3_entry, &logic_idx);
		if (rc) {
			pr_err("fail to add C3 rule\n");
			return rc;
		}
		pr_debug("Rule added in C3: logic_idx: %d\n", logic_idx);
	}
	return 0;
}

/*******************************************************************************
 * pp2_cls_c3_test
 * this routine is for UT, will be removed in future
 ******************************************************************************/
int pp2_cls_c3_test(struct pp2_inst *inst, int num)
{
	switch (num) {
	case 1:
	{
		int idx;
		struct pp2_cls_pkt_key_t pkt_key;
		struct pp2_cls_mng_pkt_key_t mng_pkt_key;
		struct pp2_cls_c3_add_entry_t c3_entry;
		u32 logic_idx;
		int rc = 0;

		pr_debug("**************C3 Test#1***********************\n");
		/* init value */
		MVPP2_MEMSET_ZERO(pkt_key);
		MVPP2_MEMSET_ZERO(mng_pkt_key);
		MVPP2_MEMSET_ZERO(c3_entry);
		c3_entry.mng_pkt_key = &mng_pkt_key;
		c3_entry.mng_pkt_key->pkt_key = &pkt_key;

		/* set value */
		c3_entry.port.port_type = MVPP2_SRC_PORT_TYPE_PHY;
		c3_entry.port.port_value = 1;

		c3_entry.lkp_type = 1;
		c3_entry.mng_pkt_key->pkt_key->field_bm = MVPP2_MATCH_VID_OUTER;
		c3_entry.mng_pkt_key->pkt_key->out_vid = 1000;

		c3_entry.qos_info.policer_id = 0;

		c3_entry.action.color_act = MVPP2_COLOR_ACTION_TYPE_NO_UPDT;
		c3_entry.action.q_low_act = MVPP2_ACTION_TYPE_UPDT_LOCK;
		c3_entry.action.q_high_act = MVPP2_ACTION_TYPE_UPDT_LOCK;
		c3_entry.action.policer_act = MVPP2_ACTION_TYPE_UPDT_LOCK;
		c3_entry.action.flowid_act = MVPP2_ACTION_FLOWID_ENABLE;
		c3_entry.action.frwd_act = MVPP2_FRWD_ACTION_TYPE_HWF_LOCK;

		c3_entry.qos_value.q_high = 0;
		c3_entry.qos_value.q_low = 5;

		/* add rule */
		for (idx = 0; idx < 1; idx++) {
			rc = pp2_cls_c3_rule_add(inst, &c3_entry, &logic_idx);
			if (rc) {
				pr_err("fail to add C3 rule\n");
				return rc;
			}
		}
		break;
	}

	case 2:
	{
		int idx;
		struct pp2_cls_pkt_key_t pkt_key;
		struct pp2_cls_mng_pkt_key_t mng_pkt_key;
		struct pp2_cls_c3_add_entry_t c3_entry;
		u32 logic_idx;
		int rc = 0;

		pr_debug("**************C3 Test#2***********************\n");
		/* init value */
		MVPP2_MEMSET_ZERO(pkt_key);
		MVPP2_MEMSET_ZERO(mng_pkt_key);
		MVPP2_MEMSET_ZERO(c3_entry);
		c3_entry.mng_pkt_key = &mng_pkt_key;
		c3_entry.mng_pkt_key->pkt_key = &pkt_key;

		/* set value */
		c3_entry.port.port_type = MVPP2_SRC_PORT_TYPE_PHY;
		c3_entry.port.port_value = 3;

		c3_entry.lkp_type = 1;
		c3_entry.mng_pkt_key->pkt_key->field_bm = MVPP2_MATCH_VID_OUTER;
		c3_entry.mng_pkt_key->pkt_key->out_vid = 1000;

		c3_entry.qos_info.policer_id = 0;

		c3_entry.action.color_act = MVPP2_COLOR_ACTION_TYPE_NO_UPDT;
		c3_entry.action.q_low_act = MVPP2_ACTION_TYPE_UPDT_LOCK;
		c3_entry.action.q_high_act = MVPP2_ACTION_TYPE_UPDT_LOCK;
		c3_entry.action.policer_act = MVPP2_ACTION_TYPE_UPDT_LOCK;
		c3_entry.action.flowid_act = MVPP2_ACTION_FLOWID_ENABLE;
		c3_entry.action.frwd_act = MVPP2_FRWD_ACTION_TYPE_HWF_LOCK;

		c3_entry.qos_value.q_high = 0;
		c3_entry.qos_value.q_low = 5;

		/* add rule */
		for (idx = 0; idx < 4096; idx++) {
			c3_entry.mng_pkt_key->pkt_key->out_vid = idx;
			rc = pp2_cls_c3_rule_add(inst, &c3_entry, &logic_idx);
			if (rc) {
				pr_err("fail to add C3 rule\n");
				return rc;
			}
		}
		break;
	}

	case 3:
	{
		int idx;
		struct pp2_cls_pkt_key_t pkt_key;
		struct pp2_cls_mng_pkt_key_t mng_pkt_key;
		struct pp2_cls_c3_add_entry_t c3_entry;
		u32 logic_idx;
		int rc = 0;

		pr_debug("**************C3 Test#3***********************\n");
		/* init value */
		MVPP2_MEMSET_ZERO(pkt_key);
		MVPP2_MEMSET_ZERO(mng_pkt_key);
		MVPP2_MEMSET_ZERO(c3_entry);
		c3_entry.mng_pkt_key = &mng_pkt_key;
		c3_entry.mng_pkt_key->pkt_key = &pkt_key;

		/* set value */
		c3_entry.port.port_type = MVPP2_SRC_PORT_TYPE_UNI;
		c3_entry.port.port_value = 3;

		c3_entry.lkp_type = 1;
		c3_entry.mng_pkt_key->pkt_key->field_bm = MVPP2_MATCH_IPV6_5T;
			/*PP2_CLS_MATCH_IPV6_PKT | */
			/* MVPP2_MATCH_IP_PROTO | MVPP2_MATCH_IP_SRC | MVPP2_MATCH_IP_DST | MVPP2_MATCH_L4_SRC */
			/*| MVPP2_MATCH_L4_DST; */
		c3_entry.mng_pkt_key->pkt_key->ipvx_add.ip_proto = IPPROTO_TCP;
		c3_entry.mng_pkt_key->pkt_key->ipvx_add.ip_ver = 6;
		c3_entry.mng_pkt_key->pkt_key->ipvx_add.ip_src.ip_add.ipv6[0] = 0x20;
		for (idx = 0; idx < 15; idx++)
			c3_entry.mng_pkt_key->pkt_key->ipvx_add.ip_src.ip_add.ipv6[1 + idx] = idx;

		c3_entry.mng_pkt_key->pkt_key->ipvx_add.ip_dst.ip_add.ipv6[0] = 0x20;
		for (idx = 0; idx < 15; idx++)
			c3_entry.mng_pkt_key->pkt_key->ipvx_add.ip_dst.ip_add.ipv6[1 + idx] = 0x10 + idx;

		c3_entry.mng_pkt_key->pkt_key->l4_dst = 0x1000;
		c3_entry.mng_pkt_key->pkt_key->l4_src = 0x2000;

		c3_entry.qos_info.policer_id = 0;

		c3_entry.action.color_act = MVPP2_COLOR_ACTION_TYPE_NO_UPDT;
		c3_entry.action.q_low_act = MVPP2_ACTION_TYPE_UPDT_LOCK;
		c3_entry.action.q_high_act = MVPP2_ACTION_TYPE_UPDT_LOCK;
		c3_entry.action.policer_act = MVPP2_ACTION_TYPE_UPDT_LOCK;
		c3_entry.action.flowid_act = MVPP2_ACTION_FLOWID_ENABLE;
		c3_entry.action.frwd_act = MVPP2_FRWD_ACTION_TYPE_HWF_LOCK;

		c3_entry.qos_value.q_high = 0;
		c3_entry.qos_value.q_low = 5;

		/* add rule */
		for (idx = 0; idx < 1; idx++) {
			rc = pp2_cls_c3_rule_add(inst, &c3_entry, &logic_idx);
			if (rc) {
				pr_err("fail to add C3 rule\n");
				return rc;
			}
		}
		break;
	}

	case 4:
	{
		int idx;
		struct pp2_cls_pkt_key_t pkt_key;
		struct pp2_cls_mng_pkt_key_t mng_pkt_key;
		struct pp2_cls_c3_add_entry_t c3_entry;
		u32 logic_idx;
		int rc = 0;

		pr_debug("**************C3 Test#4***********************\n");
		/* init value */
		MVPP2_MEMSET_ZERO(pkt_key);
		MVPP2_MEMSET_ZERO(mng_pkt_key);
		MVPP2_MEMSET_ZERO(c3_entry);
		c3_entry.mng_pkt_key = &mng_pkt_key;
		c3_entry.mng_pkt_key->pkt_key = &pkt_key;

		/* set value */
		c3_entry.port.port_type = MVPP2_SRC_PORT_TYPE_UNI;
		c3_entry.port.port_value = 3;

		c3_entry.lkp_type = 1;
		c3_entry.mng_pkt_key->pkt_key->field_bm = MVPP2_MATCH_IPV4_5T;
			/*PP2_CLS_MATCH_IPV6_PKT | */
			/* MVPP2_MATCH_IP_PROTO | MVPP2_MATCH_IP_SRC | MVPP2_MATCH_IP_DST | MVPP2_MATCH_L4_SRC */
			/*| MVPP2_MATCH_L4_DST; */
		c3_entry.mng_pkt_key->pkt_key->ipvx_add.ip_proto = IPPROTO_TCP;
		c3_entry.mng_pkt_key->pkt_key->ipvx_add.ip_ver = 4;
		for (idx = 0; idx < 4; idx++)
			c3_entry.mng_pkt_key->pkt_key->ipvx_add.ip_src.ip_add.ipv4[idx] = idx;

		for (idx = 0; idx < 4; idx++)
			c3_entry.mng_pkt_key->pkt_key->ipvx_add.ip_dst.ip_add.ipv4[idx] = 0x10 + idx;

		c3_entry.mng_pkt_key->pkt_key->l4_dst = 0x1000;
		c3_entry.mng_pkt_key->pkt_key->l4_src = 0x2000;

		c3_entry.qos_info.policer_id = 0;

		c3_entry.action.color_act = MVPP2_COLOR_ACTION_TYPE_NO_UPDT;
		c3_entry.action.q_low_act = MVPP2_ACTION_TYPE_UPDT_LOCK;
		c3_entry.action.q_high_act = MVPP2_ACTION_TYPE_UPDT_LOCK;
		c3_entry.action.policer_act = MVPP2_ACTION_TYPE_UPDT_LOCK;
		c3_entry.action.flowid_act = MVPP2_ACTION_FLOWID_ENABLE;
		c3_entry.action.frwd_act = MVPP2_FRWD_ACTION_TYPE_HWF_LOCK;

		c3_entry.qos_value.q_high = 0;
		c3_entry.qos_value.q_low = 5;

		/* add rule */
		for (idx = 0; idx < 1; idx++) {
			rc = pp2_cls_c3_rule_add(inst, &c3_entry, &logic_idx);
			if (rc) {
				pr_err("fail to add C3 rule\n");
				return rc;
			}
		}
		break;
	}

	case 5:
	{
		int idx;
		struct pp2_cls_pkt_key_t pkt_key;
		struct pp2_cls_mng_pkt_key_t mng_pkt_key;
		struct pp2_cls_c3_add_entry_t c3_entry;
		u32 logic_idx;
		int rc = 0;

		pr_debug("**************C3 Test#5***********************\n");
		/* init value */
		MVPP2_MEMSET_ZERO(pkt_key);
		MVPP2_MEMSET_ZERO(mng_pkt_key);
		MVPP2_MEMSET_ZERO(c3_entry);
		c3_entry.mng_pkt_key = &mng_pkt_key;
		c3_entry.mng_pkt_key->pkt_key = &pkt_key;

		/* set value */
		c3_entry.port.port_type = MVPP2_SRC_PORT_TYPE_UNI;
		c3_entry.port.port_value = 1;

		c3_entry.lkp_type = 1;
		c3_entry.mng_pkt_key->pkt_key->field_bm = MVPP2_MATCH_IPV4_5T;
			/*PP2_CLS_MATCH_IPV6_PKT | */
			/* MVPP2_MATCH_IP_PROTO | MVPP2_MATCH_IP_SRC | MVPP2_MATCH_IP_DST | MVPP2_MATCH_L4_SRC */
			/*| MVPP2_MATCH_L4_DST; */
		c3_entry.mng_pkt_key->pkt_key->ipvx_add.ip_proto = IPPROTO_TCP;
		c3_entry.mng_pkt_key->pkt_key->ipvx_add.ip_ver = 4;
		for (idx = 0; idx < 4; idx++)
			c3_entry.mng_pkt_key->pkt_key->ipvx_add.ip_src.ip_add.ipv4[idx] = idx;

		for (idx = 0; idx < 4; idx++)
			c3_entry.mng_pkt_key->pkt_key->ipvx_add.ip_dst.ip_add.ipv4[idx] = 0xf0 + idx;

		c3_entry.mng_pkt_key->pkt_key->l4_dst = 0x1234;
		c3_entry.mng_pkt_key->pkt_key->l4_src = 0x5678;

		c3_entry.qos_info.policer_id = 0;

		c3_entry.action.color_act = MVPP2_COLOR_ACTION_TYPE_NO_UPDT;
		c3_entry.action.q_low_act = MVPP2_ACTION_TYPE_UPDT_LOCK;
		c3_entry.action.q_high_act = MVPP2_ACTION_TYPE_UPDT_LOCK;
		c3_entry.action.policer_act = MVPP2_ACTION_TYPE_UPDT_LOCK;
		c3_entry.action.flowid_act = MVPP2_ACTION_FLOWID_ENABLE;
		c3_entry.action.frwd_act = MVPP2_FRWD_ACTION_TYPE_HWF_LOCK;

		c3_entry.qos_value.q_high = 5;
		c3_entry.qos_value.q_low = 2;

		/* add rule */
		for (idx = 0; idx < 1; idx++) {
			rc = pp2_cls_c3_rule_add(inst, &c3_entry, &logic_idx);
			if (rc) {
				pr_err("fail to add C3 rule\n");
				return rc;
			}
			pr_debug("Rule added in C3: logic_idx: %d\n", logic_idx);
		}

		/*pp2_cls_c3_entry_dump(cpu_slot, MVPP2_C3_ENTRY_DUMP_ALL, 0); */

		/* delete rule */
		/*
		* rc = pp2_cls_c3_rule_del(cpu_slot, logic_idx);
		* if (rc) {
		*	pr_err("fail to delete C3 rule\n");
		*	return rc;
		* }
		*/
		break;
	}
	default:
		pr_err("C3 test number not available\n");
	}
	return 0;
}
