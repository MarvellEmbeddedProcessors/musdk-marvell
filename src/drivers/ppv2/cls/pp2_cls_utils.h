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

/**
 * @file pp2_cls_utils.h
 *
 *  api protypes and macro of pp2_cls_utils.c
 */

/***********************/
/* h file declarations */
/***********************/
#ifndef _PP2_CLS_UTILS_H_
#define _PP2_CLS_UTILS_H_

/********************************************************************************/
/*			MACROS							*/
/********************************************************************************/
#define MVPP2_MEMBER_NUM(array)	ARRAY_SIZE(array)

/********************************************************************************/
/*			STRUCTURES						*/
/********************************************************************************/

struct pp2_cls_enum_str_t {
	int enum_value;	/* the value of enum */
	const char *enum_str;	/* the string name of enum	*/
};

struct pp2_cls_enum_array_t {
	int enum_num;				/* total enum number	*/
	struct pp2_cls_enum_str_t *enum_array;	/* enum array		*/
};

/********************************************************************************/
/*			PROTOTYPE						*/
/********************************************************************************/
u32 common_mask_gen(int bit_num);
const char *pp2_utils_eng_no_str_get(int value);
const char *pp2_utils_field_id_str_get(int value);
const char *pp2_utils_valid_str_get(int value);
const char *pp2_cls_utils_field_match_str_get(int value);
const char *pp2_cls_utils_field_id_str_get(int value);
const char *pp2_cls_utils_port_type_str_get(int value);
const char *pp2_cls_utils_l4_type_str_get(int value);
const char *pp2_cls_utils_qos_action_str_get(int value);
const char *pp2_cls_utils_common_action_str_get(int value);
const char *pp2_cls_utils_flow_id_action_str_get(int value);
const char *pp2_cls_utils_frwd_action_str_get(int value);
const char *pp2_cls_utils_scan_mode_str_get(int value);
const char *pp2_cls_utils_tbl_action_type_str_get(int value);
const char *pp2_g_enum_prs_proto_num_str_get(int value);
const char *pp2_g_enum_prs_net_proto_str_get(int value);
const char *pp2_g_enum_prs_net_proto_field_str_get(int value);
const char *pp2_g_enum_prs_lookup_str_get(int value);
const char *pp2_g_enum_prs_log_port_str_get(int value);
const char *pp2_cls_utils_plcr_state_str_get(int value);
const char *pp2_cls_utils_plcr_token_type_str_get(int value);
const char *pp2_cls_utils_plcr_color_mode_str_get(int value);
void print_horizontal_line(u32 char_count, const char *char_val);
int mv_pp2x_ptr_validate(const void *ptr);
int mv_pp2x_range_validate(int value, int min, int max);
int mv_pp2x_parse_mac_address(char *buf, u8 *macaddr_parts);

#endif /* _PP2_CLS_UTILS_H_ */
