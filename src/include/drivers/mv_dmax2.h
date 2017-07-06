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

#ifndef __MV_DMAX2_H__
#define __MV_DMAX2_H__

#include "mv_std.h"

/* DMA-XOR v2 Initialization API documentation */

#define DMAX2_BURST_SIZE	2048
#define MV_XOR_V2_DESC_RESERVED_SIZE	12
#define MV_XOR_V2_DESC_BUFF_D_ADDR_SIZE	12

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
	DMAX2_TRANS_MEM_ATTR_OUT_OF_RANGE
};

enum dmax2_trans_location {
	DMAX2_TRANS_LOCATION_SRC = 0,
	DMAX2_TRANS_LOCATION_DST,
	DMAX2_TRANS_LOCATION_SRC_AND_DST
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
 * @param    dmax2	void pointer for DMA engine struct
 *
 * @retval 0 Success.
 */
int dmax2_init(struct dmax2_params *params, void **dmax2);

/**
 * Free DMA engine resources
 *
 * @param    dmax2	void pointer for DMA engine struct
 *
 * @retval 0 Success.
 */
int dmax2_deinit(void *dmax2);

/**
 * configure memory attributes for DMA source/destination
 *
 * @param    dmax2	void pointer for DMA engine struct
 * @param    location	location settings (Source / Destination / Source & Destination)
 * @param    mem_attr	attribute type settings
 *
 * @retval 0 Success.
 *
 */
int dmax2_set_mem_attributes(void *dmax2,
			     enum dmax2_trans_location	location,
			     enum dmax2_mem_direction	mem_attr);

/**
 * Enqueue descriptors to DMA HW queue
 *
 * @param    dmax2	void pointer for DMA engine struct
 * @param    descs	Pointer to requested descriptor's array
 * @param    num	Pointer for number of requested descriptors to enqueue
 *
 * @retval 0 Success
 */
int dmax2_enq(void *dmax2, struct dmax2_desc *descs, u16 *num);

/**
 * Dequeue descriptors to DMA HW queue
 *
 * @param    dmax2	void pointer for DMA engine struct
 * @param    descs	Pointer to Completed descriptor's array
 * @param    num	Pointer for number of requested descriptors to dequeue
 * @param    verify	Boolean to indicate if need to verify and return descriptor's content & status
 *
 * @retval 0 Success
 */
int dmax2_deq(void *dmax2, struct dmax2_trans_complete_desc *descs, u16 *num, int verify);

/**
 * Get free space in DMA descriptor's HW Queue (How many descriptors can be queued?)
 *
 * @param    dmax2	void pointer for DMA engine struct
 */
int dmax2_get_enq_num_available(void *dmax2);

/**
 * Get count of descriptors ready to be dequeued (How many descriptors Engine finished processing)
 *
 * @param    dmax2	void pointer for DMA engine struct
 */
int dmax2_get_deq_num_available(void *dmax2);

/** @} */ /* end of grp_dmax2 */

#endif /* __MV_DMAX2_H__ */
