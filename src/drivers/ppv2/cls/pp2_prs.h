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
 * @file pp2_cls_prs.h
 *
 * internal and external definitions for parser High level routines
 */

#ifndef _PP2_CLS_PRS_H_
#define _PP2_CLS_PRS_H_

#include "pp2_cls_types.h"
#include "pp2_cls_internal_types.h"
#include "pp2_cls_common.h"
#include "pp2_cls_utils.h"
#include "pp2_flow_rules.h"
#include "pp2_c3.h"
#include "pp2_c2.h"
#include "pp2_cls_db.h"

/********************************************************************************/
/*			MACROS							*/
/********************************************************************************/

/* HW_BYTE_OFFS
 * return HW byte offset in 4 bytes register
 * _offs_: native offset (LE)
 * LE example: HW_BYTE_OFFS(1) = 1
 * BE example: HW_BYTE_OFFS(1) = 2
 */

#if defined(__LITTLE_ENDIAN)
#define HW_BYTE_OFFS(_offs_) (_offs_)
#else
#define HW_BYTE_OFFS(_offs_) ((3 - ((_offs_) % 4)) + (((_offs_) / 4) * 4))
#endif

#define SRAM_BIT_TO_BYTE(_bit_) HW_BYTE_OFFS((_bit_) / 8)

#define TCAM_DATA_BYTE_OFFS_LE(_offs_)		(((_offs_) - \
	((_offs_) % 2)) * 2 + ((_offs_) % 2))
#define TCAM_DATA_MASK_OFFS_LE(_offs_) (((_offs_) * 2) - ((_offs_) % 2)  + 2)

/* TCAM_DATA_BYTE/MASK
 * tcam data divide into 4 bytes registers
 * each register include 2 bytes of data and 2 bytes of mask
 * the next macros calc data/mask offset in 4 bytes register
 * _offs_: native offset (LE) in data bytes array
 * relevant only for TCAM data bytes
 * used by PRS and CLS2
 */
#define TCAM_DATA_BYTE(_offs_) (HW_BYTE_OFFS(TCAM_DATA_BYTE_OFFS_LE(_offs_)))
#define TCAM_DATA_MASK(_offs_) (HW_BYTE_OFFS(TCAM_DATA_MASK_OFFS_LE(_offs_)))

#define MAX_LOOKUP	3
#define MAX_PROTO_NUM	3

/********************************************************************************/
/*			ENUMERATIONS						*/
/********************************************************************************/

/********************************************************************************/
/*			STRUCTURES						*/
/********************************************************************************/

struct mv_pp2x_iphdr {
	uint8_t	 ihl:4,
		 version:4;
	u8	 tos;
	u16 tot_len;
	u16 id;
	u16 frag_off;
	u8	 ttl;
	u8	 protocol;
	u16 check;
	u32 saddr;
	u32 daddr;
};

struct mv_pp2x_ipv6hdr {
	uint8_t	 priority:4,
		 version:4;
	u8	 flow_lbl[3];

	u16 payload_len;
	u8	 nexthdr;
	u8	 hop_limit;

	struct	 in6_addr saddr;
	struct	 in6_addr daddr;
};

/* Parser definition array */
struct pp2_prs_dynamic {
	u32	valid;
	u32	proto;
	u32	lookup;
	u32	sram_bits;
	u32	tcam_bits;
	u32	num_entries;
};

/********************************************************************************/
/*			PROTOTYPE						*/
/********************************************************************************/
int pp2_cls_prs_init(struct pp2_inst *inst);
void pp2_cls_prs_deinit(struct pp2_inst *inst);
int mv_pp2x_prs_flow_id_attr_get(int flow_id);
int pp2_prs_eth_start_hdr_set(struct pp2_port *port, enum pp2_ppio_eth_start_hdr eth_start_hdr);
int pp2_prs_set_log_port(struct pp2_port *port, struct pp2_ppio_log_port_params *params);
int pp2_prs_eth_start_header_set(struct pp2_port *port, enum pp2_ppio_eth_start_hdr mode);
void mv_pp2x_prs_clear_active_vlans(struct pp2_port *port, uint32_t *vlans);
int mv_pp2x_prs_mac_da_accept(struct pp2_port *port, const u8 *da, bool add);
int mv_pp2x_prs_hw_dump(struct pp2_port *port);
int mv_pp2x_prs_hw_hits_dump(struct pp2_port *port);

#endif /*_PP2_CLS_PRS_H_*/

