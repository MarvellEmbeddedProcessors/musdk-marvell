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

#ifndef _MQA_INTERNAL_H
#define _MQA_INTERNAL_H

#include "drivers/mv_mqa.h"
#include "drivers/mqa_def.h"
#include "drivers/mv_mqa_queue.h"

#define MQA_REGION_MAX			(16)	/** Max number of regions in MQA tables */
#define MQA_REGION_FREE			(-1)

#define MQA_REGION_INIT_COUNT		(0)

#define MQA_QNxT_ALIGN			(64)

/* MQA Queue attributes definitions */
#define EGRESS_MQA_QUEUE_INGRESS_BIT_FIELD_ATTR	(MQA_QUEUE_EGRESS | MQA_QUEUE_INGRESS)
#define LOCAL_MQA_QUEUE_REMOTE_BIT_FIELD_ATTR	(MQA_QUEUE_LOCAL | MQA_QUEUE_REMOTE)

/* MQA Queue attributes checking status */
#define IS_QUEUE_INGRESS(x)	(((x) & EGRESS_MQA_QUEUE_INGRESS_BIT_FIELD_ATTR) == MQA_QUEUE_INGRESS)
#define IS_QUEUE_LOCAL(x)	(((x) & LOCAL_MQA_QUEUE_REMOTE_BIT_FIELD_ATTR) == MQA_QUEUE_LOCAL)

/**
 * MQA Tables Global Definition
 *
 * qpt_base	queue producer context table base
 * qct_base	queue consumer context table base
 * qnpt_base	phys base of producer notification table
 * qnct_base	phys base of consumer notification table
 * qnpt_virt	virt address of qnpt_base
 * qnct_virt	virt address of qpct_base
 * size		size of the tables
 */
struct mqa {
	void *qpt_base;
	void *qct_base;
	dma_addr_t qnpt_phys;
	dma_addr_t qnct_phys;
	struct mqa_qnpt_entry *qnpt_virt;
	struct mqa_qnct_entry *qnct_virt;
	u32 size;
};

/** MQA Table entry parameters */
struct mqa_table_entry {

	u32 mqa_queue_attr;

	struct mqa_queue		common;
	struct mqa_queue_qpt_spec	qpt;
	struct mqa_queue_qct_spec	qct;

};

/**
 * MQA Region definition
 */
struct mqa_region_params {
	u32 region_id;		/** MQA region Id */
	u32 region_start;	/** MQA region start index */
	u32 region_size;	/** MQA region size */

	u32 queue_alloc_count;	/** Number of allocated queue from the region */
	u32 queue_free_index;	/** Index of next free queue in the region */

};

/**
 * MQA Queue definition
 */
struct mqa_q {
	u32 q_id;
	u32 len;
	void *phy_base_addr;
	void *virt_base_addr;
	void *prod_phys;
	void *cons_phys;
	void *prod_virt;
	void *cons_virt;
};

/** MQA GNPT entry parameters */
struct mqa_qnpt_entry {
	u64 producer_address;		/** Queue producer index */
};

/** MQA GNCT entry parameters */
struct mqa_qnct_entry {
	u64 consumer_address;		/** Queue consumer index */
};


int queue_alloc(struct mqa *mqa, u32 *q);
int queue_free(u32 phy_queue_id);
int queue_config(struct mqa *mqa, u32 phy_queue_id, struct mqa_table_entry *q_params);
int queue_associate_pair(struct mqa *mqa, u32 phy_queue_id, u32 dest_queue_id);
int queue_associate_notify_intr(struct mqa *mqa, u32 phy_queue_id, struct mqa_queue_msix_params *params);
int queue_associate_bpool(struct mqa *mqa, u32 phy_queue_id, u32 bpool_num, u32 *bpool);

#endif /* _MQA_INTERNAL_H */

