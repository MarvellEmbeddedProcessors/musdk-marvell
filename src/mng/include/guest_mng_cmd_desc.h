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

#ifndef _GUEST_MNG_CMD_H_
#define _GUEST_MNG_CMD_H_

#include "drivers/mv_pp2_cls.h"

#define GUEST_PP2_CLS_KEY_SIZE_MAX	40	/* Max possible size of HEK key */

/*
 * msg_from_guest_codes - Define the list of messages codes that can be send by guest.
 */
enum msg_from_guest_codes {
	MSG_F_GUEST_NONE = 0,
	MSG_F_GUEST_TABLE_INIT,
	MSG_F_GUEST_TABLE_DEINIT,
	MSG_F_GUEST_ADD_RULE,
	MSG_F_GUEST_MODIFY_RULE,
	MSG_F_GUEST_REMOVE_RULE,
	MSG_F_GUEST_KA,
	MSG_F_GUEST_GPIO_ENABLE,
	MSG_F_GUEST_GPIO_DISABLE,
	MSG_F_GUEST_GPIO_GET_LINK_STATE,
	MSG_F_GUEST_GPIO_RESET,
	MSG_F_GUEST_LAST,
};

struct guest_pp2_cls_tbl_action {
	struct pp2_cls_tbl_action action;
	struct pp2_cls_cos_desc cos;
};

struct guest_pp2_cls_tbl_params {
	struct pp2_cls_tbl_params params;
	struct guest_pp2_cls_tbl_action def_action;
};

struct guest_pp2_cls_rule_key_field {
	u8	size;
	int	key_valid;
	u8	key[GUEST_PP2_CLS_KEY_SIZE_MAX];
	int	mask_valid;
	u8	mask[GUEST_PP2_CLS_KEY_SIZE_MAX];
};

struct guest_pp2_cls_tbl_rule {
	u8	num_fields;
	struct guest_pp2_cls_rule_key_field fields[PP2_CLS_TBL_MAX_NUM_FIELDS];
};

struct guest_pp2_cls_rule_add {
	u32 tbl_id;
	struct guest_pp2_cls_tbl_rule rule;
	struct guest_pp2_cls_tbl_action action;
};

struct guest_pp2_cls_rule_remove {
	u32 tbl_id;
	struct guest_pp2_cls_tbl_rule rule;
};

struct guest_pp2_cls_cmd_resp {
	union {
		struct {
			u32	tbl_id;
		} tbl_init;
	};
};

struct guest_giu_cmd_resp {
	union {
		u8 link_state;
	};
};

struct guest_cmd_resp {
#define RESP_STATUS_OK		(0)
#define RESP_STATUS_FAIL	(1)
	u8 status;
	union {
		struct guest_pp2_cls_cmd_resp pp2_cls_resp;
		struct guest_giu_cmd_resp giu_resp;
	};
};

#endif /* _GUEST_MNG_CMD_H_ */
