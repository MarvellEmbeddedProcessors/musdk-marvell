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
/* h file declarations */
/***********************/
#ifndef _PP2_C2_H_
#define _PP2_C2_H_

#include "std_internal.h"
#include "lib/list.h"

/******************************************************************************/
/*                                 MACROS                                     */
/******************************************************************************/
#define MVPP2_C2_FIRST_ENTRY		16 /* reserve 0-15 entries for kernel usage */
#define MVPP2_C2_LAST_ENTRY		255
#define MVPP2_C2_ENTRY_MAX		(MVPP2_C2_LAST_ENTRY + 1)
#define MVPP2_C2_ENTRY_INVALID_IDX	MVPP2_C2_ENTRY_MAX
#define MVPP2_C2_LKP_TYPE_MAX		64
#define MVPP2_C2_LKP_TYPE_INVALID_PRI	0xFF
#define MVPP2_C2_TCAM_KEY_LEN_MAX	8
#define MVPP2_C2_LOGIC_IDX_BASE		1000

#define MVPP2_C2_HEK_LKP_TYPE_OFFS	0
#define MVPP2_C2_HEK_LKP_TYPE_BITS	6
#define MVPP2_C2_HEK_LKP_TYPE_MASK	(0x3F << MVPP2_C2_HEK_LKP_TYPE_OFFS)

#define MVPP2_C2_HEK_PORT_TYPE_OFFS	6
#define MVPP2_C2_HEK_PORT_TYPE_BITS	2
#define MVPP2_C2_HEK_PORT_TYPE_MASK	(0x3 << MVPP2_C2_HEK_PORT_TYPE_OFFS)

/******************************************************************************/
/*                              ENUMERATIONS                                  */
/******************************************************************************/
enum pp2_cls_c2_entry_free_t {
	MVPP2_C2_ENTRY_FREE_TRUE = 0,
	MVPP2_C2_ENTRY_FREE_FALSE,
};

enum pp2_cls_c2_db_entry_valid_t {
	MVPP2_C2_ENTRY_INVALID = 0,
	MVPP2_C2_ENTRY_VALID
};

enum pp2_cls_c2_hek_offs_t {
	MVPP2_C2_HEK_OFF_BYTE0 = 0,
	MVPP2_C2_HEK_OFF_BYTE1,
	MVPP2_C2_HEK_OFF_BYTE2,
	MVPP2_C2_HEK_OFF_BYTE3,
	MVPP2_C2_HEK_OFF_BYTE4,
	MVPP2_C2_HEK_OFF_BYTE5,
	MVPP2_C2_HEK_OFF_BYTE6,
	MVPP2_C2_HEK_OFF_BYTE7,
	MVPP2_C2_HEK_OFF_LKP_PORT_TYPE,
	MVPP2_C2_HEK_OFF_PORT_ID,
	MVPP2_C2_HEK_OFF_MAX
};

enum pp2_cls_port_dump_idx_t {
	MVPP2_PORT_DUMP_TYPE = 0,
	MVPP2_PORT_DUMP_VALUE,
	MVPP2_PORT_DUMP_MASK
};

enum pp2_cls_act_dump_idx_t {
	MVPP2_ACT_DUMP_DSCP = 0,
	MVPP2_ACT_DUMP_PBIT,
	MVPP2_ACT_DUMP_GEMPORT,
	MVPP2_ACT_DUMP_QUEUE_LOW,
	MVPP2_ACT_DUMP_QUEUE_HIGH,
	MVPP2_ACT_DUMP_COLOR,
	MVPP2_ACT_DUMP_RSS,
	MVPP2_ACT_DUMP_POLICER,
	MVPP2_ACT_DUMP_FRWD,
	MVPP2_ACT_DUMP_FLOWID
};

enum pp2_cls_qos_dump_index_t {
	MVPP2_QOS_DUMP_DSCP = 0,
	MVPP2_QOS_DUMP_PBIT,
	MVPP2_QOS_DUMP_GEMPORT,
	MVPP2_QOS_DUMP_QUEUE_LOW,
	MVPP2_QOS_DUMP_QUEUE_HIGH,
	MVPP2_QOS_DUMP_COLOR,
	MVPP2_QOS_DUMP_TABLE,
	MVPP2_QOS_DUMP_POLICER
};

enum pp2_cls_mod_dump_idx_t {
	MVPP2_MOD_DUMP_IPTR = 0,
	MVPP2_MOD_DUMP_DPTR,
	MVPP2_MOD_DUMP_L4_CHECKSUM
};

enum pp2_cls_flow_dump_idx_t {
	MVPP2_FLOW_DUMP_ID = 0,
	MVPP2_FLOW_DUMP_CNT
};

enum pp2_cls_index_dump_idx_t {
	MVPP2_INDEX_DUMP_TCAM = 0,
	MVPP2_INDEX_DUMP_LOGICAL,
	MVPP2_INDEX_DUMP_DB
};

/******************************************************************************/
/*                               STRUCTURES                                   */
/******************************************************************************/
struct pp2_cls_c2_data_t {
	u32			valid;		/* Indicate the data is a real TCAM entry or not */
	u32			priority;	/* priority in this look_type */
	struct mv_pp2x_src_port		port;
	u8			lkp_type;
	u8			lkp_type_mask;
	u32			field_bm;	/* bitmap of relevant fields*/
	struct pp2_cls_mng_pkt_key_db_t	mng_pkt_key;	/* pkt key value */
	struct mv_pp2x_engine_qos_info	qos_info;	/* all the qos input */
	struct mv_pp2x_engine_pkt_action	action;		/* update&lock info */
	struct mv_pp2x_qos_value		qos_value;	/* pri/dscp/gemport/qLow/qHigh */
	struct mv_pp2x_engine_pkt_mod	pkt_mod;	/* PMT cmd_idx and data_idx */
	struct mv_pp2x_duplicate_info		flow_info;	/* pkt duplication flow info */
};

struct pp2_cls_c2_index_t {
	u32		valid;		/* Indicate the node is in list(valid), free or lookup up type list */
	u32		c2_logic_idx;	/* logical index, unique inentifier, used for delete C2 entry */
	u32		c2_hw_idx;	/* HW entry index in C2 engine */
	u32		c2_data_db_idx;	/* data index in db */
	struct list	list_node;	/* list node */
};

int pp2_cls_cli_c2_lkp_type_entry_dump(void *arg, int argc, char *argv[]);
int pp2_cls_cli_c2_free_tcam_dump(void *arg, int argc, char *argv[]);
int pp2_cls_cli_c2_valid_lkp_type_dump(void *arg, int argc, char *argv[]);
int pp2_cls_cli_c2_hw_dump(void *arg, int argc, char *argv[]);
int pp2_cls_cli_c2_hw_hit_dump(void *arg, int argc, char *argv[]);
int pp2_cls_cli_qos_dscp_dump(void *arg, int argc, char *argv[]);
int pp2_cls_cli_qos_pcp_dump(void *arg, int argc, char *argv[]);
int pp2_cls_cli_rss_rxq_bind_dump(void *arg, int argc, char *argv[]);


/******************************************************************************/
/*                                PROTOTYPE                                   */
/******************************************************************************/
int pp2_cls_c2_get_hw_idx_from_logic_idx(struct pp2_inst *inst, u32 logic_idx, u32 *c2_hw_idx, u32 *c2_db_idx);
int pp2_cls_c2_free_entry_number_get(struct pp2_inst *inst, u32 *free_entry_number);
int pp2_cls_c2_rule_add(struct pp2_inst *inst, struct mv_pp2x_c2_add_entry *c2_entry, u32 *c2_logic_index);
int pp2_cls_c2_rule_del(struct pp2_inst *inst, u32 c2_logic_index);
int pp2_cls_c2_rule_sram_get(struct pp2_inst *inst, u32 logic_index, struct pp2_cls_engine_sram_t *sram);
int pp2_cls_c2_rule_sram_update(struct pp2_inst *inst, u32 logic_index, struct pp2_cls_engine_sram_t *sram);
int pp2_cls_c2_reset(struct pp2_inst *inst);
int pp2_cls_c2_start(struct pp2_inst *inst);
int pp2_cls_c2_hit_cntr_clear_all(uintptr_t cpu_slot);
int pp2_cls_c2_hit_cntr_all_get(struct pp2_inst *inst, int hit_low_thresh, struct pp2_cls_hit_cnt_t cntr_info[],
				u32 *num_of_cntrs);
int pp2_cls_c2_hit_cntr_get(struct pp2_inst *inst, int c2_id, u32 *cntr);
int pp2_cls_c2_tcam_hek_get(u32 field_bm, struct mv_pp2x_c2_add_entry *c2_entry, u8 hek[], u8 hek_mask[]);

#endif /* _PP2_C2_H_ */
