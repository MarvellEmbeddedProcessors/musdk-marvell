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

#ifndef __DMAX2_H__
#define __DMAX2_H__

#include "drivers/mv_dmax2.h"
#include "std_internal.h"

/* DMA Engine Registers */
#define DMA_DESQ_BALR_OFF		0x000
#define DMA_DESQ_BAHR_OFF		0x004
#define DMA_DESQ_SIZE_OFF		0x008
#define DMA_DESQ_DONE_OFF		0x00C
#define DMA_DESQ_DONE_PENDING_MASK	0x1FFF
#define DMA_DESQ_DONE_PENDING_SHIFT	0
#define DMA_DESQ_DONE_READ_PTR_MASK	0x1FFF
#define DMA_DESQ_DONE_READ_PTR_SHIFT	16
#define DMA_DESQ_ATTR_CACHE_MASK	0x3F3F
#define DMA_DESQ_ATTR_OUTER_SHAREABLE	0x202
#define DMA_DESQ_ATTR_CACHEABLE		0x3C3C
#define DMA_IMSG_THRD_OFF		0x018
#define DMA_IMSG_THRD_MASK		0x7FFF
#define DMA_IMSG_THRD_SHIFT		0x0
#define DMA_IMSG_EVT_MASK		(1 << 16)
#define DMA_IMSG_TMR_MASK		(1 << 18)

/* ARCACHE & ARDOMAIN definitions */
#define DMA_DESQ_ARATTR_OFF		0x010
#define DMA_DESQ_AWATTR_OFF		0x01C
#define DMA_DESC_ATTR_ARDOMAIN_MASK	(0x3 << 0)
#define DMA_DESC_ATTR_ARCACHE_OFFSET	2
#define DMA_DESC_ATTR_ARCACHE_MASK	(0xF << DMA_DESC_ATTR_ARCACHE_OFFSET)

#define DMA_ATTR_ARDOMAIN_DEVICE_SYSTEM_SHAREABLE	0x3
#define DMA_ATTR_ARDOMAIN_OUTER_SHAREABLE		0x2
#define DMA_ATTR_ARCACHE_BUFFERABLE_OFFSET	0
#define DMA_ATTR_ARCACHE_MODIFIABLE_OFFSET	1
#define DMA_ATTR_ARCACHE_OTHER_ALLOCATE_OFFSET	2
#define DMA_ATTR_ARCACHE_ALLOCATE_OFFSET	3

  /* Same flags as DMA_DESQ_ARATTR_OFF */
#define DMA_DESQ_ALLOC_OFF		0x04C
#define DMA_DESQ_ALLOC_WRPTR_MASK	0xFFFF
#define DMA_DESQ_ALLOC_WRPTR_SHIFT	16
#define DMA_DESQ_ALLOC_HW_PENDING_MASK	0x1FFF
#define DMA_DESQ_ALLOC_HW_PENDING_SHIFT	0
#define DMA_DESQ_CTRL_OFF		0x100
#define DMA_DESQ_CTRL_32B		1
#define DMA_DESQ_CTRL_128B		7
#define DMA_DESQ_STOP_OFF		0x800
#define DMA_DESQ_DEALLOC_OFF		0x804
#define DMA_DESQ_ADD_OFF		0x808

#define DMA_DESQ_STATUS_MASK		0xFE00

/* XOR Global registers */
#define GLOB_BW_CTRL			0x4
#define GLOB_BW_CTRL_NUM_OSTD_RD_SHIFT	0
#define GLOB_BW_CTRL_NUM_OSTD_RD_VAL	64
#define GLOB_BW_CTRL_NUM_OSTD_WR_SHIFT	8
#define GLOB_BW_CTRL_NUM_OSTD_WR_VAL	8
#define GLOB_BW_CTRL_RD_BURST_LEN_SHIFT	12
#define GLOB_BW_CTRL_RD_BURST_LEN_VAL	4
#define GLOB_BW_CTRL_WR_BURST_LEN_SHIFT	16
#define GLOB_BW_CTRL_WR_BURST_LEN_VAL	4
#define GLOB_PAUSE			0x014
#define GLOB_PAUSE_AXI_TIME_DIS_VAL	0x8

#define MV_XOR_V2_CMD_LINE_NUM_MAX_D_BUF	8

#define MV_XOR_V2_DESC_RESERVED_SIZE	12
#define MV_XOR_V2_DESC_BUFF_D_ADDR_SIZE	12

#define DMAX2_Q_OCCUPANCY(_d)	((_d->desc_push_idx - _d->desc_pop_idx + _d->desc_q_size) & (_d->desc_q_size - 1))
#define DMAX2_Q_SPACE(_d)		(_d->desc_q_size - DMAX2_Q_OCCUPANCY(_d) - 1)


/**
 * struct dmax2 - implements a xor device
 * @sw_ll_lock: serializes enqueue/dequeue operations to the sw
 * descriptors pool
 * @push_lock: serializes enqueue operations to the DESCQ
 * @dma_base: memory mapped DMA register base
 * @glob_base: memory mapped global register base
 * @hw_desq: HW descriptors queue
 * @hw_desq_virt: virtual address of DESCQ
*/
struct dmax2 {
	int	id;
	struct sys_iomem *iomem;
	void __iomem *dma_base;
	void __iomem *glob_base;
	dma_addr_t hw_desq;
	struct dmax2_desc *hw_desq_virt;
	int num_of_pending;
	u16 desc_push_idx;
	u16 desc_pop_idx;
	int desc_q_size;
};

int init_dmax2_mem(struct dmax2 *dmax2);
void deinit_dmax2_mem(struct dmax2 *dmax2);

#endif /* __DMAX2_H__ */
