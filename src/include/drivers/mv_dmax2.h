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

#ifndef __MV_DMAX2_H__
#define __MV_DMAX2_H__

#include "mv_std.h"

/* DMA-XOR v2 Initialization API documentation */

#define DMAX2_BURST_SIZE	2048
#define MV_XOR_V2_DESC_RESERVED_SIZE	12
#define MV_XOR_V2_DESC_BUFF_D_ADDR_SIZE	12

/* According to the spec, descriptors list base address must be aligned to 256
 * Bytes (addr[7:0] == 0x0).
 */
#define DMAX2_DESC_ADDR_ALIGN	0x100


struct dmax2;

/** @addtogroup grp_dmax2 DMA v2 engine
 *
 *  DMA Copy Engine API documentation
 *
 *  @{
 */

enum dmax2_mem_direction {
	DMAX2_TRANS_MEM_ATTR_NONE = 0,
	DMAX2_TRANS_MEM_ATTR_NOT_CACHABLE,
	DMAX2_TRANS_MEM_ATTR_CACHABLE,
	DMAX2_TRANS_MEM_ATTR_CACHABLE_STASH,
	DMAX2_TRANS_MEM_ATTR_IO,
	DMAX2_TRANS_MEM_ATTR_CACHABLE_WR_THROUGH_NO_ALLOC,
	DMAX2_TRANS_MEM_ATTR_OUT_OF_RANGE
};

enum dmax2_trans_location {
	DMAX2_TRANS_LOCATION_SRC = 0,
	DMAX2_TRANS_LOCATION_DST,
	DMAX2_TRANS_LOCATION_SRC_AND_DST,
	DMAX2_TRANS_LOCATION_OUT_OF_RANGE
};

/**
 * struct dmax2_desc - DMA HW descriptor
 * @desc_id: used by S/W and is not affected by H/W.
 * @flags: error and status flags
 * @crc32_result: CRC32 calculation result
 * @desc_ctrl: operation mode and control flags
 * @buff_size: amount of bytes to be processed
 * @src_addr: Fill-Pattern or Source-Address and AW-Attributes
 * @dst_addr: Destination-Address
 * @data_buff_addr: Source (and might be RAID6 destination)
 * addresses of data buffers in RAID5 and RAID6
 * @reserved: reserved
 */
struct dmax2_desc {
	u16 desc_id;
	u16 flags;
#define DESC_FLAGS_SYNC		BIT(8)
	u32 crc32_result;
	u32 desc_ctrl;

	/* Definitions for desc_ctrl */
#define DESC_NUM_ACTIVE_D_BUF_SHIFT	22
#define DESC_OP_MODE_SHIFT		28
#define DESC_OP_MODE_NOP		0       /* Idle operation */
#define DESC_OP_MODE_MEMCPY		1       /* Pure-DMA operation */
#define DESC_OP_MODE_MEMSET		2       /* Mem-Fill operation */
#define DESC_OP_MODE_MEMINIT		3       /* Mem-Init operation */
#define DESC_OP_MODE_MEM_COMPARE	4       /* Mem-Compare operation */
#define DESC_OP_MODE_CRC32		5       /* CRC32 calculation */
#define DESC_OP_MODE_XOR		6       /* RAID5 (XOR) operation */
#define DESC_OP_MODE_RAID6		7       /* RAID6 P&Q-generation */
#define DESC_OP_MODE_RAID6_REC		8       /* RAID6 Recovery */
#define DESC_Q_BUFFER_ENABLE		BIT(16)
#define DESC_P_BUFFER_ENABLE		BIT(17)
#define DESC_IOD			BIT(27)

	u32 buff_size;
	u64 src_addr;
	u64 dst_addr;
	u32 data_buff_addr[MV_XOR_V2_DESC_BUFF_D_ADDR_SIZE]; /* not relevant for MEMCPY */
	u32 reserved[MV_XOR_V2_DESC_RESERVED_SIZE];		/* not in use */
};

/**
 * struct dmax2_trans_complete_desc - DMA completed transfer descriptor
 * @status: descriptor status/flags
 * @res: currently unused
 * @desc_id: descriptor ID (cookie)
 */
struct dmax2_trans_complete_desc {
	u16	status;
	u16	desc_id;
};

/**
 * struct dmax2_trans_complete_desc - DMA completed transfer descriptor
 * @match: engine name string
 * @size: queue size
 */
struct dmax2_params {
	/* Used for DTS acc to find appropriate "physical" DMA-XOR2 obj;
	 * E.g. "dmax2-0" means DMA-XOR2[0]
	 */
	const char	*match;
	u32		 queue_size;
};

/**
 * Initialize DMA engine
 *
 * @param    params	required parameters struct for initialization
 * @param    dmax2	pointer for DMA engine struct
 *
 * @retval 0 Success.
 */
int dmax2_init(struct dmax2_params *params, struct dmax2 **dmax2);

/**
 * Free DMA engine resources
 *
 * @param    dmax2	pointer for DMA engine struct
 *
 * @retval 0 Success.
 */
int dmax2_deinit(struct dmax2 *dmax2);

/**
 * configure memory attributes for DMA source/destination
 *
 * @param    dmax2	pointer for DMA engine struct
 * @param    location	location settings (Source / Destination / Source & Destination)
 * @param    mem_attr	attribute type settings
 *
 * @retval 0 Success.
 *
 */
int dmax2_set_mem_attributes(struct dmax2		*dmax2,
			     enum dmax2_trans_location	 location,
			     enum dmax2_mem_direction	 mem_attr);

/**
 * Enqueue descriptors to DMA HW queue
 *
 * @param    dmax2	pointer for DMA engine struct
 * @param    descs	Pointer to requested descriptor's array
 * @param    num	Pointer for number of requested descriptors to enqueue
 *
 * @retval 0 Success
 */
int dmax2_enq(struct dmax2 *dmax2, struct dmax2_desc *descs, u16 *num);

/**
 * Dequeue descriptors to DMA HW queue
 *
 * @param    dmax2	pointer for DMA engine struct
 * @param    descs	Pointer to Completed descriptor's array
 * @param    num	Pointer for number of requested descriptors to dequeue
 * @param    verify	Boolean to indicate if need to verify and return descriptor's content & status
 *
 * @retval 0 Success
 */
int dmax2_deq(struct dmax2 *dmax2, struct dmax2_trans_complete_desc *descs, u16 *num, int verify);

/**
 * Get free space in DMA descriptor's HW Queue (How many descriptors can be queued?)
 *
 * @param    dmax2	pointer for DMA engine struct
 */
int dmax2_get_enq_num_available(struct dmax2 *dmax2);

/**
 * Get count of descriptors ready to be dequeued (How many descriptors Engine finished processing)
 *
 * @param    dmax2	pointer for DMA engine struct
 */
int dmax2_get_deq_num_available(struct dmax2 *dmax2);

/** @} */ /* end of grp_dmax2 */

#endif /* __MV_DMAX2_H__ */
