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

#ifndef _GIU_BUFF_DESC_H_
#define _GIU_BUFF_DESC_H_

#include "std_internal.h"

/* This file includes the definitions of all data structures that should be
 * shared with the various drivers and modules on the SNIC side.
 * The file should be copied AS-IS, in order to make sure that data-structures
 * on both sides are 100% aligned.
 */

#define HOST_FORMAT_DIRECT_SG	2
#define HOST_NUM_SG_ENT_MASK	0x1F


#define HOST_TXD_FORMAT_MASK	(0x30000000)
#define HOST_TXD_FORMAT_SHIFT	28


struct host_tx_desc {
	/* 0x0 - 0x3
	 * fields order: msb ... lsb
	 * Byte 0: res:1 | l3_offset:7
	 * Byte 1: gen_ip4_csum:1 | gen_l4_csum:2 | ip_hdr_len:5
	 * Byte 2: l2_pad_en:1 | res2:1 | buf_release_mode:1 | pool_idx:5
	 * Byte 3: res3:2 | last:1 | first:1 | l3_info:2 | l4_type:2
	 */
	u32 flags;

	/* 0x4 - 0x7 */
	u8 pkt_offset;
	u8 res4;
	u16 byte_cnt;

	/* 0x8 - 0xB */
	u16 res5;
	u8 num_sg_ent;
	u8 res;

	/* 0xC - 0xF */
	u32 res6;

	/* 0x10 - 0x17 */
	u64 buffer_addr;

	/* 0x18 - 0x1F */
	u64 cookie;
}  __packed;

#define HOST_RXD_FORMAT_MASK	(0x00180000)
#define HOST_RXD_FORMAT_SHIFT	19

struct host_rx_desc {
	/* 0x0 - 0x3
	 * fields order: msb ... lsb
	 * Byte 0: res:1 | l3offset:7
	 * Byte 1: err_sum:1 | err_code:2 | ip_hdr_len:5
	 * Byte 2: res:1 | l4_csum_ok:1 | res:1 | pool_idx:5
	 * Byte 3: res:1 | l3_info:3 | l4_info:3 | ipv4_hdr_err:1
	 */
	u32 flags;

	/* 0x4 - 0x7 */
	u8 pkt_offset;
	/* portnum_dp: port_num:3 | dp:2 | res:3 */
	u8 portnum_dp;
	u16 byte_cnt;

	/* 0x8 - 0xB */
	u16 res4;
	u8 num_sg_ent; /* currently 5bits, 0x1f */
	u8 res;

	/* 0xC - 0xF */
	u32 timestamp_hashkey;

	/* 0x10 - 0x17 */
	u64 buffer_addr;

	/* 0x18 - 0x1F */
	u64 cookie;
} __packed;

/* Buffers Pool Descriptor */
struct host_bpool_desc {
	u64 buff_addr_phys;
	u64 buff_cookie;
} __packed;

#endif /* _GIU_BUFF_DESC_H_ */
