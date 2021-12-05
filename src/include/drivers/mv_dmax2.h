/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

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
