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

#include "std_internal.h"
#include "lib/lib_misc.h"

#include "dmax2.h"

/* Engine general configuration */
static int mv_xor_v2_descq_init(struct dmax2 *dmax2)
{
	u32 reg;

	/* write descriptor size */
	writel(DMA_DESQ_CTRL_128B, dmax2->dma_base + DMA_DESQ_CTRL_OFF);

	/* write the descriptor Q size to the DMA engine */
	writel(dmax2->desc_q_size, dmax2->dma_base + DMA_DESQ_SIZE_OFF);

	/* write the DESQ address to the DMA engine*/
	writel(dmax2->hw_desq & 0xFFFFFFFF,
	       dmax2->dma_base + DMA_DESQ_BALR_OFF);
	writel((dmax2->hw_desq & 0xFFFF00000000) >> 32,
	       dmax2->dma_base + DMA_DESQ_BAHR_OFF);

	/* enable the DMA engine */
	/* Reset the XOR state machine. */
	writel(1, dmax2->dma_base + DMA_DESQ_STOP_OFF);
	writel(3, dmax2->dma_base + DMA_DESQ_STOP_OFF);
	writel(0, dmax2->dma_base + DMA_DESQ_STOP_OFF);

	/* Set both descriptors as well as data as: CACHABLE (i.e. cachable, allocate) */
	/* TODO: un-like the data, the descriptors cannot be changed right now (no API) */
	reg = (DMA_DESQ_ATTR_PROT |
		(DMA_ATTR_AR_CACHABLE | (DMA_ATTR_AR_CACHABLE << DMA_DESQ_ATTR_DESC_OFF)));
	writel(reg, dmax2->dma_base + DMA_DESQ_ARATTR_OFF);

	/* Set both descriptors as well as data as: CACHABLE (i.e. cachable, write-through, allocate) */
	/* TODO: un-like the data, the descriptors cannot be changed right now (no API) */
	reg = (DMA_DESQ_ATTR_PROT |
		(DMA_ATTR_AW_CACHABLE | (DMA_ATTR_AW_CACHABLE << DMA_DESQ_ATTR_DESC_OFF)));
	writel(reg, dmax2->dma_base + DMA_DESQ_AWATTR_OFF);

	/* BW CTRL - set values to optimize the XOR performance:
	 *
	 *  - Set WrBurstLen & RdBurstLen - the unit will issue
	 *    maximum of 256B write/read transactions.
	 * -  Limit the number of outstanding write & read data
	 *    (OBB/IBB) requests to the maximal value.
	*/
	reg = (GLOB_BW_CTRL_NUM_OSTD_RD_VAL << GLOB_BW_CTRL_NUM_OSTD_RD_SHIFT) |
		(GLOB_BW_CTRL_NUM_OSTD_WR_VAL << GLOB_BW_CTRL_NUM_OSTD_WR_SHIFT) |
		(GLOB_BW_CTRL_RD_BURST_LEN_VAL << GLOB_BW_CTRL_RD_BURST_LEN_SHIFT) |
		(GLOB_BW_CTRL_WR_BURST_LEN_VAL << GLOB_BW_CTRL_WR_BURST_LEN_SHIFT);
	writel(reg, dmax2->glob_base + GLOB_BW_CTRL);

	/* Disable the AXI timer feature */
	reg = readl(dmax2->glob_base + GLOB_PAUSE);
	reg |= GLOB_PAUSE_AXI_TIME_DIS_VAL;
	writel(reg, dmax2->glob_base + GLOB_PAUSE);

	/* Disable the timer and event interrupts */
	reg = readl(dmax2->dma_base + DMA_IMSG_THRD_OFF);
	reg &= ~(DMA_IMSG_EVT_MASK | DMA_IMSG_TMR_MASK);
	writel(reg, dmax2->dma_base + DMA_IMSG_THRD_OFF);

	return 0;
}

int dmax2_init(struct dmax2_params *params, struct dmax2 **dmax2)
{
	struct dmax2 *dmax2_lcl;
	int ret = 0;
	u8 dmax2_slot;

	if (mv_sys_match(params->match, "dmax2", 1, &dmax2_slot)) {
		pr_err("dmax2 engine registration failure\n");
		return -ENXIO;
	}

	dmax2_lcl = kmalloc(sizeof(struct dmax2), GFP_KERNEL);
	if (!dmax2_lcl)
		return -ENOMEM;

	memset(dmax2_lcl, 0, sizeof(struct dmax2));

	dmax2_lcl->id = dmax2_slot;

	ret = init_dmax2_mem(dmax2_lcl);
	if (ret)
		goto free_dev;

	if ((params->queue_size  & (params->queue_size - 1)) != 0) {
		pr_err("DMAX2 Queue size must be power of 2 (requested Queue size: %d)\n", params->queue_size);
		ret = -EINVAL;
		goto free_dev_mem;
	}

	dmax2_lcl->desc_q_size = params->queue_size;

	/*
	 * allocate coherent memory for hardware descriptors
	 * note: writecombine gives slightly better performance, but
	 * requires that we explicitly flush the writes
	 */
	dmax2_lcl->hw_desq_virt =
		mv_sys_dma_mem_alloc((sizeof(struct dmax2_desc) * dmax2_lcl->desc_q_size),
				DMAX2_DESC_ADDR_ALIGN);
	if (!dmax2_lcl->hw_desq_virt) {
		ret = -ENOMEM;
		goto free_dev_mem;
	}
	dmax2_lcl->hw_desq = mv_sys_dma_mem_virt2phys(dmax2_lcl->hw_desq_virt);
	memset(dmax2_lcl->hw_desq_virt, 0, sizeof(struct dmax2_desc) * dmax2_lcl->desc_q_size);

	mv_xor_v2_descq_init(dmax2_lcl);

	*dmax2 = dmax2_lcl;
	return 0;

free_dev_mem:
	deinit_dmax2_mem(dmax2_lcl);
free_dev:
	if (dmax2_lcl->hw_desq_virt)
		mv_sys_dma_mem_free(dmax2_lcl->hw_desq_virt);

	kfree(dmax2_lcl);

	return ret;
}

int dmax2_deinit(struct dmax2 *dmax2)
{
	if (!dmax2)
		return -EINVAL;

	if (dmax2->hw_desq_virt)
		mv_sys_dma_mem_free(dmax2->hw_desq_virt);

	deinit_dmax2_mem(dmax2);

	kfree(dmax2);
	return 0;
}

int dmax2_set_mem_attributes(struct dmax2		*dmax2,
			     enum dmax2_trans_location	location,
			     enum dmax2_mem_direction	mem_attr)
{
	u32	reg, reg_mask, src_attr, dst_attr;

	/* Prepare mask and value for memory attributes */
	src_attr = dst_attr = DMA_ATTR_NONE;
	reg_mask = (DMA_DESC_ATTR_ARDOMAIN_MASK | DMA_DESC_ATTR_ARCACHE_MASK);
	switch (mem_attr) {
	case (DMAX2_TRANS_MEM_ATTR_CACHABLE_STASH):
		src_attr = dst_attr = DMA_ATTR_CACHABLE_STASH;
		break;
	case (DMAX2_TRANS_MEM_ATTR_NOT_CACHABLE):
	case (DMAX2_TRANS_MEM_ATTR_IO):
		src_attr = dst_attr = DMA_ATTR_IO_N_NOT_CACHABLE;
		break;
	case (DMAX2_TRANS_MEM_ATTR_CACHABLE):
		src_attr = DMA_ATTR_AR_CACHABLE;
		dst_attr = DMA_ATTR_AW_CACHABLE;
		break;
	case (DMAX2_TRANS_MEM_ATTR_CACHABLE_WR_THROUGH_NO_ALLOC):
		src_attr = DMA_ATTR_AR_WR_NO_ALLOC;
		dst_attr = DMA_ATTR_AW_WR_NO_ALLOC;
		break;
	default:
		pr_err("DMAX2: requested unsupported memory attribute (%d)\n", mem_attr);
		return -ENOTSUP;
	}

	if (location == DMAX2_TRANS_LOCATION_SRC ||
	    location == DMAX2_TRANS_LOCATION_SRC_AND_DST) {
		reg = readl(dmax2->dma_base + DMA_DESQ_ARATTR_OFF);
		reg &= ~reg_mask;
		reg |= src_attr;
		writel(reg, dmax2->dma_base + DMA_DESQ_ARATTR_OFF);
	}
	if (location == DMAX2_TRANS_LOCATION_DST ||
	    location == DMAX2_TRANS_LOCATION_SRC_AND_DST) {
		reg = readl(dmax2->dma_base + DMA_DESQ_AWATTR_OFF);
		reg &= ~reg_mask;
		reg |= dst_attr;
		writel(reg, dmax2->dma_base + DMA_DESQ_AWATTR_OFF);
	}

	return 0;
}

/* return number of DESCs in the queue that are pending and ready SW processing */
int dmax2_get_deq_num_available(struct dmax2 *dmax2)
{
	int completed;
	u32 reg;

	reg = mv_readl_relaxed(dmax2->dma_base + DMA_DESQ_DONE_OFF);
	completed = ((reg >> DMA_DESQ_DONE_PENDING_SHIFT) & DMA_DESQ_DONE_PENDING_MASK);

	return completed;
}

/* return DMA Queue space: How many descriptors available be pushed to Queue */
int dmax2_get_enq_num_available(struct dmax2 *dmax2)
{
	return DMAX2_Q_SPACE(dmax2);
}

int dmax2_enq(struct dmax2 *dmax2, struct dmax2_desc *descs, u16 *num)
{
	struct dmax2_desc	*desc;
	u16			desc_copy_num, queue_space = DMAX2_Q_SPACE(dmax2);

	if (unlikely(*num > queue_space)) {
		pr_debug("not enough space to queue %d descriptors (queue space = %d)\n", *num, queue_space);
		*num = queue_space;
		if (unlikely(!*num))
			return 0;
	}

	desc_copy_num = *num;
	/* if Requested descriptor count is going to reach end of HW-Q, than need to split memcpy:
	 * 1. Copy descriptors to HW-Q, up to end of HW-Q,
	 * 2. Point push index to start of HW-Q, and copy rest of descriptors to start of HW-Q
	 */
	if (dmax2->desc_push_idx + *num > dmax2->desc_q_size) {
		desc = &dmax2->hw_desq_virt[dmax2->desc_push_idx];

		/* copy descriptors only up to end of HW-Q */
		desc_copy_num = dmax2->desc_q_size - dmax2->desc_push_idx;
		__builtin_memcpy(desc, descs, desc_copy_num * sizeof(struct dmax2_desc));

		/* point push index to start of HW-Q */
		dmax2->desc_push_idx = 0;

		/* update array & number of descriptors left to copy to start of HW-Q */
		descs += desc_copy_num;
		desc_copy_num = *num - desc_copy_num;
	}

	/* copy to where push index points to */
	desc = &dmax2->hw_desq_virt[dmax2->desc_push_idx];
	__builtin_memcpy(desc, descs, desc_copy_num * sizeof(struct dmax2_desc));

	dmax2->desc_push_idx = (dmax2->desc_push_idx + desc_copy_num) & (dmax2->desc_q_size  - 1);

	writel(*num, dmax2->dma_base + DMA_DESQ_ADD_OFF);

	return 0;
}

int dmax2_deq(struct dmax2 *dmax2, struct dmax2_trans_complete_desc *descs, u16 *num, int verify)
{
	struct dmax2_desc	*desc;
	u16	i;
	u32	reg = mv_readl_relaxed(dmax2->dma_base + DMA_DESQ_DONE_OFF);
	u16	hw_pending_num = ((reg >> DMA_DESQ_DONE_PENDING_SHIFT) & DMA_DESQ_DONE_PENDING_MASK);

	/* can dequeue only pending descriptors */
	if (hw_pending_num < *num)
		*num = hw_pending_num;

	/* no verify (Fast mode): only update pop index that these descriptors have been released */
	if (!verify)
		dmax2->desc_pop_idx =  (dmax2->desc_pop_idx + *num) & (dmax2->desc_q_size  - 1);
	else {
		/* Else, return every descriptor's cookie & status, so it could be verified if needed */
		/* barrier here before accessing to read the descriptors */
		rmb();
		for (i = 0; i < *num; i++) {
			desc = &dmax2->hw_desq_virt[dmax2->desc_pop_idx];
			if (mv_readw_relaxed(&desc->flags) & DESC_FLAGS_SYNC)
				break;
			descs[i].desc_id = desc->desc_id;
			descs[i].status = (desc->flags & DMA_DESQ_STATUS_MASK);
			dmax2->desc_pop_idx = (dmax2->desc_pop_idx + 1) & (dmax2->desc_q_size  - 1);
		}
		*num = i;
	}

	mv_writel_relaxed(*num, dmax2->dma_base + DMA_DESQ_DEALLOC_OFF);
	return 0;
}
