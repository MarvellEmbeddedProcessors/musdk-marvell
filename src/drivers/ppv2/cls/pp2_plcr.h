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
#ifndef _PP2_PLCR_H_
#define _PP2_PLCR_H_

#include "std_internal.h"
#include "drivers/mv_pp2_cls.h"

struct pp2_cls_plcr {
	int	pp2_id;		/* PP2 Instance */
	int	id;		/* policer id */
};


/******************************************************************************/
/*                                 MACROS                                     */
/******************************************************************************/
#define MVPP2_PLCR_MAX			48

/* minimium policer ID, start with 1, since 0 is used for all C2/3 enties without Policer configuration */
#define MVPP2_PLCR_MIN_ENTRY_ID		(1)
#define ACT_DUP_POLICER_MAX		(31)
#define MVPP2_PLCR_BANK0_DEFAULT_ENTRY_ID	(0)/*first policer id of bank 0, it is the reservered default policer*/
#define MVPP2_PLCR_BANK1_DEFAULT_ENTRY_ID	(ACT_DUP_POLICER_MAX + 1)/*first policer id of bank 1, reservered*/
#define MVPP2_TOKEN_PERIOD_400_CORE_CLOCK	(400)	/* 400 core clock		*/
#define MVPP2_TOKEN_PERIOD_480_CORE_CLOCK	(480)	/* 480 core clock		*/
#define MVPP2_TOKEN_PERIOD_600_CORE_CLOCK	(600)	/* 600 core clock		*/
#define MVPP2_TOKEN_PERIOD_800_CORE_CLOCK	(800)	/* 800 core clock		*/
#define MVPP2_PLCR_MIN_PKT_LEN		(0)	/* default min packet length	*/
#define MVPP2_PLCR_CIR_NO_LIMIT		(0)	/* do not limit CIR		*/
#define MVPP2_PLCR_BURST_SIZE_NO_LIMIT	(0)	/* maximum burst size		*/
#define MVPP2_PLCR_MAX_TOKEN_VALUE	((1 << MVPP2_PLCR_TOKEN_VALUE_BITS) - 1)/* maximum token value	*/

#define MVPP2_POLICER_2_BANK(policer)	(policer / (ACT_DUP_POLICER_MAX + 1))

/******************************************************************************/
/*                              ENUMERATIONS                                  */
/******************************************************************************/
enum pp2_cls_plcr_entry_state_t {
	MVPP2_PLCR_ENTRY_INVALID_STATE = 0,	/* invalid policer entry	*/
	MVPP2_PLCR_ENTRY_VALID_STATE		/* valid policer entry		*/
};

enum pp2_cls_plcr_rate_state_t {
	MVPP2_PLCR_BASE_RATE_DISABLE = 0,	/* disable base rate generation	*/
	MVPP2_PLCR_BASE_RATE_ENABLE		/* enable base rate generation	*/
};

enum pp2_cls_plcr_mode_t {
	/* Bank#0 and Bank#1 policers are operating in serial mode (with the Bank#0 policer first)	*/
	MVPP2_PLCR_MODE_SERIAL_BANK_0_1 = 0,
	/* Bank#0 and Bank#1 policers are operating in serial mode (with the Bank#1 policer first)	*/
	MVPP2_PLCR_MODE_SERIAL_BANK_1_0,
	/* Bank#0 and Bank#1 policers are operating in parallel. The resulting color is as follows:
	* If one of the policers generates red color, then the packet color is red.
	* Else, if one of the policers generates yellow color, then the packet color is yellow.
	* Else, the packet color is green.
	*/
	MVPP2_PLCR_MODE_PARALLEL,
	/* only Bank#0 is used */
	MVPP2_PLCR_MODE_ONLY_BANK_0
};

enum pp2_cls_plcr_ref_cnt_action_t {
	MVPP2_PLCR_REF_CNT_INC = 0,	/* increase reference counter by 1	*/
	MVPP2_PLCR_REF_CNT_DEC,		/* decrease reference counter by 1	*/
	MVPP2_PLCR_REF_CNT_CLEAR	/* clear reference counter to be 0	*/
};

enum pp2_cls_plcr_token_update_type_t {
	MVPP2_PLCR_TOKEN_RATE_TYPE_1KBPS_1B = 0,/* rate/burst bucket resolution:1KBPS/1B	*/
	MVPP2_PLCR_TOKEN_RATE_TYPE_10KBPS_8B,	/* rate/burst bucket resolution:10KBPS/8B	*/
	MVPP2_PLCR_TOKEN_RATE_TYPE_100KBPS_64B,	/* rate/burst bucket resolution:100KBPS/64B	*/
	MVPP2_PLCR_TOKEN_RATE_TYPE_1MBPS_512B,	/* rate/burst bucket resolution:1MBPS/512B	*/
	MVPP2_PLCR_TOKEN_RATE_TYPE_10MBPS_4KB,	/* rate/burst bucket resolution:10MBPS/4KB	*/
};

/******************************************************************************/
/*                               STRUCTURES                                   */
/******************************************************************************/
struct pp2_cls_plcr_gen_cfg_t {
	enum pp2_cls_plcr_rate_state_t	rate_state;	/* enable or disable base rate generation	*/
	enum pp2_cls_plcr_mode_t mode;			/* operation mode				*/
	u16	base_period;				/* token update period in units of core clock	*/
	u8	min_pkt_len;				/* minium packet length allowed by policer	*/
};

struct pp2_cls_plcr_token_type_t {
	enum pp2_cls_plcr_token_update_type_t token_type;	/* token type	*/
	u32	rate_resl;	/* rate resolution in units of pkt/s or kbps		*/
	u32	min_rate;	/* minimum rate supported by this token type		*/
	u32	max_rate;	/* maximum rate supported by this token type		*/
	u32	burst_size_resl;/* burst size resolution in units of pkt or bytes	*/
	u32	min_burst_size;	/* minimum burst size supported by this token type	*/
	u32	max_burst_size;	/* maximum burst size supported by this token type	*/
};

/******************************************************************************/
/*                                PROTOTYPE                                   */
/******************************************************************************/
int pp2_cls_plcr_entry_add(struct pp2_inst *inst,
			   const struct pp2_cls_plcr_params *policer_entry,
			   u8 policer_id);
int pp2_cls_plcr_entry_del(struct pp2_inst *inst, u8 policer_id);
int pp2_cls_plcr_ref_cnt_update(struct pp2_inst *inst,
				u8 policer_id,
				enum pp2_cls_plcr_ref_cnt_action_t cnt_action,
				int update_ppio);
int pp2_cls_plcr_entry_state_get(struct pp2_inst *inst, u8 policer_id, enum pp2_cls_plcr_entry_state_t *state);
int pp2_cls_plcr_ref_cnt_get(struct pp2_inst *inst, u8 policer_id, u32 *rules_ref, u32 *ppios_ref);
int pp2_cls_plcr_entry_clear(struct pp2_inst *inst);
int pp2_cls_plcr_gen_cfg_set(struct pp2_inst *inst, struct pp2_cls_plcr_gen_cfg_t *gen_cfg);
int pp2_cls_plcr_reset(struct pp2_inst *inst);
int pp2_cls_plcr_start(struct pp2_inst *inst);
void pp2_cls_plcr_finish(struct pp2_inst *inst);

#endif /* _PP2_PLCR_H_ */
