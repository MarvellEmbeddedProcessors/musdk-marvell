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

#ifndef _MQA_DEF_H
#define _MQA_DEF_H

#include "drivers/mv_mqa.h"

/**
 * MQA Queue Definition
 */

struct mqa_queue_msix_inf {
	u32		 data;
	u32		 mask_value;
	phys_addr_t	 pa;
	void		*va; /**< If NULL disabled */
	void		*mask_address;
};

/** MQA extended queue parameters */
struct mqa_queue_ext {
	u32 bm_queue[MQA_BM_QUEUE_ARRAY];

};

/** MQA common queue parameters  */
struct mqa_queue {
	u64 ring_phy_addr;	/** Ring physical base address */
	u64 ring_virt_addr;	/** Ring virtual base address - relevant for local queues */
	u64 prod_phys;		/** Queue producer physical address */
	u64 prod_virt;		/** Queue producer virtual address */
	u64 cons_phys;		/** Queue consumer physical address */
	u64 cons_virt;		/** Queue consumer virtual address */
	u64 host_remap;		/** Remap address in case the queue is on host side */
	u32 ring_size;		/** Ring size */
	u32 entry_size;		/** Ring element size */
	u32 queue_prio;		/** queue priority */
#define MQA_QFLAGS_COPY_BUF	(1 << 0) /** copy the queue payload */
#define MQA_QFLAGS_SG		(1 << 1) /** scatter-gather */
	u32 flags;		/** queue flags */

	struct mqa_queue_ext queue_ext;
	struct mqa_queue_msix_inf msix_inf;
};

/**
 * MQA Tables Definition
 */


/** MQA QPT entry parameters */
struct mqa_queue_qpt_spec {
	u64 reserved;
};

struct mqa_qpt_entry {
	struct mqa_queue common;	/** Queue common parameters */
	struct mqa_queue_qpt_spec spec;	/** QPT Queue specific parameters */

};

/** MQA QCT entry parameters */
struct mqa_queue_qct_spec {
	u32 dest_queue_id;		/** Queue desstination */
};

struct mqa_qct_entry {
	struct mqa_queue common;	/** Queue common parameters */
	struct mqa_queue_qct_spec spec;	/** QPT Queue specific parameters */

};

#define MQA_QPT_ENTRY_SIZE	(sizeof(struct mqa_qpt_entry))
#define MQA_QCT_ENTRY_SIZE	(sizeof(struct mqa_qct_entry))

#endif /* _MQA_DEF_H */

