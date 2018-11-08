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

#ifndef _MVNETA_BM_H_
#define _MVNETA_BM_H_

#include "mv_std.h"

/* BM Configuration Register */
#define MVNETA_BM_CONFIG_REG			0x0
#define    MVNETA_BM_STATUS_MASK		0x30
#define    MVNETA_BM_ACTIVE_MASK		BIT(4)
#define    MVNETA_BM_MAX_IN_BURST_SIZE_MASK	0x60000
#define    MVNETA_BM_MAX_IN_BURST_SIZE_16BP	BIT(18)
#define    MVNETA_BM_EMPTY_LIMIT_MASK		BIT(19)

/* BM Activation Register */
#define MVNETA_BM_COMMAND_REG			0x4
#define    MVNETA_BM_START_MASK			BIT(0)
#define    MVNETA_BM_STOP_MASK			BIT(1)
#define    MVNETA_BM_PAUSE_MASK			BIT(2)

/* BM Xbar interface Register */
#define MVNETA_BM_XBAR_01_REG			0x8
#define MVNETA_BM_XBAR_23_REG			0xc
#define MVNETA_BM_XBAR_POOL_REG(pool)		\
		(((pool) < 2) ? MVNETA_BM_XBAR_01_REG : MVNETA_BM_XBAR_23_REG)
#define     MVNETA_BM_TARGET_ID_OFFS(pool)	(((pool) & 1) ? 16 : 0)
#define     MVNETA_BM_TARGET_ID_MASK(pool)	\
		(0xf << MVNETA_BM_TARGET_ID_OFFS(pool))
#define     MVNETA_BM_TARGET_ID_VAL(pool, id)	\
		((id) << MVNETA_BM_TARGET_ID_OFFS(pool))
#define     MVNETA_BM_XBAR_ATTR_OFFS(pool)	(((pool) & 1) ? 20 : 4)
#define     MVNETA_BM_XBAR_ATTR_MASK(pool)	\
		(0xff << MVNETA_BM_XBAR_ATTR_OFFS(pool))
#define     MVNETA_BM_XBAR_ATTR_VAL(pool, attr)	\
		((attr) << MVNETA_BM_XBAR_ATTR_OFFS(pool))

/* Address of External Buffer Pointers Pool Register */
#define MVNETA_BM_POOL_BASE_REG(pool)		(0x10 + ((pool) << 4))
#define     MVNETA_BM_POOL_ENABLE_MASK		BIT(0)

/* External Buffer Pointers Pool RD pointer Register */
#define MVNETA_BM_POOL_READ_PTR_REG(pool)	(0x14 + ((pool) << 4))
#define     MVNETA_BM_POOL_SET_READ_PTR_MASK	0xfffc
#define     MVNETA_BM_POOL_GET_READ_PTR_OFFS	16
#define     MVNETA_BM_POOL_GET_READ_PTR_MASK	0xfffc0000

/* External Buffer Pointers Pool WR pointer */
#define MVNETA_BM_POOL_WRITE_PTR_REG(pool)	(0x18 + ((pool) << 4))
#define     MVNETA_BM_POOL_SET_WRITE_PTR_OFFS	0
#define     MVNETA_BM_POOL_SET_WRITE_PTR_MASK	0xfffc
#define     MVNETA_BM_POOL_GET_WRITE_PTR_OFFS	16
#define     MVNETA_BM_POOL_GET_WRITE_PTR_MASK	0xfffc0000

/* External Buffer Pointers Pool Size Register */
#define MVNETA_BM_POOL_SIZE_REG(pool)		(0x1c + ((pool) << 4))
#define     MVNETA_BM_POOL_SIZE_MASK		0x3fff

/* BM Interrupt Cause Register */
#define MVNETA_BM_INTR_CAUSE_REG		(0x50)

/* BM interrupt Mask Register */
#define MVNETA_BM_INTR_MASK_REG			(0x54)

/* Other definitions */
#define MVNETA_BM_SHORT_PKT_SIZE		256
#define MVNETA_BM_POOLS_NUM			4
#define MVNETA_BM_POOL_CAP_MIN			128
#define MVNETA_BM_POOL_CAP_DEF			2048
#define MVNETA_BM_POOL_CAP_MAX			\
		(16 * 1024 - MVNETA_BM_POOL_CAP_ALIGN)
#define MVNETA_BM_POOL_CAP_ALIGN		32
#define MVNETA_BM_POOL_PTR_ALIGN		32

#define MVNETA_BM_POOL_ACCESS_OFFS		8

#define MVNETA_BM_BPPI_SIZE			0x100000

enum mvneta_bm_type {
	MVNETA_BM_FREE,
	MVNETA_BM_LONG,
	MVNETA_BM_SHORT
};

struct mvneta_bm {
	uintptr_t		reg_base;  /* virtual address for BM registers */
	phys_addr_t		paddr;     /* physical address for BM registers */
	struct sys_iomem	*sys_iomem;
	/* BPPI virtual base address */
	void			*bppi_virt_addr;
	/* BPPI physical base address */
	dma_addr_t		 bppi_phys_addr;

	/* BM pools */
	struct mvneta_bm_pool	*bm_pools;
};

struct mvneta_bm_pool {
	int id;
	enum mvneta_bm_type type;
	/* Packet size */
	int pkt_size;
	/* Size of the buffer acces through DMA*/
	u32 buf_size;
	/* BPPE virtual base address */
	u32 *virt_addr;
	/* BPPE physical base address */
	dma_addr_t phys_addr;
	/* Ports using BM pool */
	u8 port_map;
	/* Max buffers number */
	int capacity;
	/* current buffers number */
	int buffs_num;

	struct mvneta_bm *priv;
};

/* Declarations and definitions */
void *mvneta_frag_alloc(unsigned int frag_size);
void mvneta_frag_free(unsigned int frag_size, void *data);
void mvneta_bm_pool_destroy(struct mvneta_bm *priv, struct mvneta_bm_pool *bm_pool);
void mvneta_bm_bufs_free(struct mvneta_bm *priv, struct mvneta_bm_pool *bm_pool);

int mvneta_bm_pool_refill(struct mvneta_bm *priv, struct mvneta_bm_pool *bm_pool);

static inline void mvneta_bm_pool_put_bp(struct mvneta_bm *priv,
					 struct mvneta_bm_pool *bm_pool,
					 dma_addr_t buf_phys_addr)
{
	writel(buf_phys_addr, priv->bppi_virt_addr + (bm_pool->id << MVNETA_BM_POOL_ACCESS_OFFS));
}

static inline u32 mvneta_bm_pool_get_bp(struct mvneta_bm *priv,
					struct mvneta_bm_pool *bm_pool)
{
	return readl(priv->bppi_virt_addr + (bm_pool->id << MVNETA_BM_POOL_ACCESS_OFFS));
}
#endif
