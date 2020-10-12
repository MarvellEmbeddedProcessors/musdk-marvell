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

#define log_fmt(fmt, ...) "mqa_queue: " fmt, ##__VA_ARGS__

#include "std_internal.h"
#include "mqa_internal.h"
#include "drivers/mv_mqa_queue.h"

/*
 *	mqa_queue_setup
 *
 *	This function allocates memory for queue ring from MUSDK via cma_calloc
 *	It allocates a queue from NIC PF region, and configures queue.
 *	It uses the queue_params to return the queue base and indexes mapping.
 *
 *	@param[in]	mqa - pointer to MQA object
 *	@param[in]	local_queue_virt_addr
 *	@param[in/out]	queue_params - pointer to queue object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise (< 0)
 */

static int mqa_queue_setup(struct mqa *mqa, void *local_queue_virt_addr, struct mqa_queue_params *queue_params)
{
	int	ret = 0;
	struct	mqa_table_entry mqa_table_entry_int;

	/* **************************************** */
	/* Operations to configure queue in MQA     */
	/* **************************************** */

	/* Update Queue basic params */
	mqa_table_entry_int.mqa_queue_attr	= queue_params->attr;
	mqa_table_entry_int.common.ring_size	= queue_params->len;
	mqa_table_entry_int.common.entry_size	= queue_params->size;
	mqa_table_entry_int.common.queue_prio	= queue_params->prio;
	mqa_table_entry_int.common.host_remap	= (u64)queue_params->host_remap;
	mqa_table_entry_int.common.flags	= 0;
	if (queue_params->copy_payload)
		mqa_table_entry_int.common.flags |= MQA_QFLAGS_COPY_BUF;
	if (queue_params->sg_en)
		mqa_table_entry_int.common.flags |= MQA_QFLAGS_SG;

	/* Update Queue according it's type */
	if (IS_QUEUE_LOCAL(queue_params->attr)) {
		mqa_table_entry_int.common.ring_phy_addr = (u64) mv_sys_dma_mem_virt2phys(local_queue_virt_addr);
		mqa_table_entry_int.common.ring_virt_addr  = (u64) local_queue_virt_addr;

		/* Update the DB entry */
		queue_params->virt_base_addr = (void *) mqa_table_entry_int.common.ring_virt_addr;
		queue_params->phy_base_addr  = (void *) mqa_table_entry_int.common.ring_phy_addr;
	} else {
		/* Remote Queue */
		mqa_table_entry_int.common.ring_virt_addr = (u64) 0;
		mqa_table_entry_int.common.ring_phy_addr  = (u64) queue_params->remote_phy_addr;

		/* No need to update the DB type - should be updated by higher mgmt layer */
	}
	/* Question: What i need to do with host_remap ???*/

	/* Update Producer/Consumer params */
	mqa_table_entry_int.common.prod_phys = (u64)queue_params->prod_phys;
	mqa_table_entry_int.common.cons_phys = (u64)queue_params->cons_phys;
	mqa_table_entry_int.common.prod_virt = (u64)queue_params->prod_virt;
	mqa_table_entry_int.common.cons_virt = (u64)queue_params->cons_virt;

	/* Commit the queue configuration */
	ret = queue_config(mqa, queue_params->idx, &mqa_table_entry_int);
	if (ret != 0) {
		pr_err("Failed to configure MQA queue\n");
		return ret;
	}

	/* Update back the DB entry with the new cons/prod addresses */
	queue_params->prod_phys = (void *)mqa_table_entry_int.common.prod_phys;
	queue_params->prod_virt = (void *)mqa_table_entry_int.common.prod_virt;
	queue_params->cons_phys = (void *)mqa_table_entry_int.common.cons_phys;
	queue_params->cons_virt = (void *)mqa_table_entry_int.common.cons_virt;

	return 0;
}

/*
 *	mqa_queue_free
 *
 *	This function frees the queue config in the MQA
 *
 *	@param[in]	mqa		A pointer to MQA object
 *	@param[in]	queue_id	queue Id
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */

int mqa_queue_free(struct mqa *mqa, u32 queue_id)
{
	/* Free the Queue in MQA */
	return queue_free(queue_id);
}

/*
 *	mqa_queue_alloc
 *
 *	@param[in]	mqa - pointer to MQA object
 *	@param[in]	q - pointer to MQA queue object
 *
 *	@retval	q_Id on success
 *	@retval	error-code otherwise
 */
int mqa_queue_alloc(struct mqa *mqa, u32 *q)
{
	return queue_alloc(mqa, q);
}

/*
 *	mqa_queue_create
 *
 *	This function is a wrapper function for queue allocation
 *	parameters received from management commands
 *	The function updates return parameters if required via
 *	q_params object
 *
 *	@param[in]	mqa - pointer to MQA object
 *	@param[in/out]	queue_params - pointer to queue object
 *	@param[out]	q - pointer to MQA queue object
 *
 *	@retval	q_Id on success
 *	@retval	error-code otherwise
 */
int mqa_queue_create(struct mqa *mqa, struct mqa_queue_params *queue_params, struct mqa_q **q)
{
	int	ret = 0;
	void	*local_queue_virt_addr = NULL;

	/* Allocate queue from MQA */
	*q = kcalloc(1, sizeof(struct mqa_q), GFP_KERNEL);
	if (*q == NULL) {
		pr_err("Failed to allocate MQA handler\n");
		return -ENOMEM;
	}

	if ((queue_params->len  & (queue_params->len - 1)) != 0) {
		queue_params->len = roundup_pow_of_two(queue_params->len);
		pr_warn("MQA size must be power of 2. round it up to %d\n", queue_params->len);
	}

	/* Need to allocate memory  only for local Queues */
	if (IS_QUEUE_LOCAL(queue_params->attr)) {

		/* Allocate memory from CMA allocator */
		local_queue_virt_addr = mv_sys_dma_mem_alloc((queue_params->len * queue_params->size),
						queue_params->size);
		if (local_queue_virt_addr == NULL) {
			pr_err("Failed to allocate memory for Queue (%d) ring\n", queue_params->idx);
			return -ENOMEM;
		}
	}

	/* Commit the queue setup in the MQA */
	ret = mqa_queue_setup(mqa, local_queue_virt_addr, queue_params);
	if (ret < 0) {
		pr_err("Failed to setup Queue (%d) in MQA\n", queue_params->idx);
		if (local_queue_virt_addr != NULL)
			mv_sys_dma_mem_free(local_queue_virt_addr);
		return ret;
	}

	(*q)->q_id = queue_params->idx;
	(*q)->len  = queue_params->len;
	(*q)->phy_base_addr  = queue_params->phy_base_addr;
	(*q)->virt_base_addr = queue_params->virt_base_addr;
	(*q)->prod_phys = queue_params->prod_phys;
	(*q)->cons_phys = queue_params->cons_phys;
	(*q)->prod_virt = queue_params->prod_virt;
	(*q)->cons_virt = queue_params->cons_virt;

	/* Operations for Queue association in MQA */
	/* ======================================= */
	if (queue_params->peer_id > 0) {

		ret = queue_associate_pair(mqa, queue_params->idx, queue_params->peer_id);
		if (ret) {
			pr_err("Failed to associate Command queues (Src %d Dest %d)\n",
				queue_params->idx, queue_params->peer_id);
		}
	}

	/* Operations for Queue association in MQA */
	/* ======================================= */

	/* Association of MSI-X interrupt Id Notification */
	ret = queue_associate_notify_intr(mqa, queue_params->idx, &queue_params->msix_inf);
	if (ret != 0) {
		/* ***Check*** We may need deconfig??? to remove the mqa_queue_config activity */
		pr_err("Failed to associate Notification MSI-X interrupt Id (Phy Q: %d, MSI-X interrupt Id: %d)\n",
			  queue_params->idx, queue_params->msix_inf.id);
	}

	if (ret != 0) {
		mqa_queue_free(mqa, (u32)queue_params->idx);
		return ret;
	}

	/* Operations for Bpool association */
	/* ================================ */

	if (queue_params->bpool_num > 0) {

		ret = queue_associate_bpool(mqa, queue_params->idx, queue_params->bpool_num,
						(u32 *)queue_params->bpool_qids);
		if (ret != 0) {
			/* ***Check*** We may need deconfig??? to remove the mqa_queue_config activity */
			pr_err("Failed to associate bpools\n");
		}
	}

	if (ret != 0) {
		mqa_queue_free(mqa, (u32)queue_params->idx);
		return ret;
	}

	return 0;
}


/*
 *	q_associate_pair
 *
 *	@param[in]	mqa - pointer to MQA object
 *	@param[in]	src_queue_id - source queue
 *	@param[in]	dest_queue_id - destination queue
 *
 *	This function associate src queue with target dest queue Id
 */
int mqa_queue_associate_pair(struct mqa *mqa, u32 src_queue_id, u32 dest_queue_id)
{
	int ret = 0;

	/* Queue destination association */
	ret = queue_associate_pair(mqa, src_queue_id, dest_queue_id);
	if (ret != 0) {
		pr_err("Failed to associate Queue pair (Phy Q: %d, Dest Q: %d)\n",
			 src_queue_id, dest_queue_id);
	}

	return ret;
}


/*
 *	mqa_queue_destroy
 *
 *	This function release all queue allocated memory and return queue Id to MQA
 *
 *	@param[in]	mqa - pointer to MQA object
 *	@param[in]	q - pointer to MQA queue object
 *
 *	@retval	q_Id on success
 *	@retval	error-code otherwise
 */
int mqa_queue_destroy(struct mqa *mqa, struct mqa_q *q)
{
	if (q == NULL)
		return -EINVAL;

	/* Free the allocated memory */
	if (q->virt_base_addr)
		mv_sys_dma_mem_free(q->virt_base_addr);

	return 0;
}

int mqa_queue_get_id(struct mqa_q *q, u32 *queue_id)
{
	if (q == NULL)
		return -EINVAL;

	*queue_id = q->q_id;

	return 0;
}

int mqa_queue_get_info(struct mqa_q *q, struct mqa_queue_info *info)
{
	if (q == NULL)
		return -EINVAL;

	info->q_id		= q->q_id;
	info->len		= q->len;
	info->phy_base_addr	= q->phy_base_addr;
	info->virt_base_addr	= q->virt_base_addr;
	info->prod_phys		= q->prod_phys;
	info->prod_virt		= q->prod_virt;
	info->cons_phys		= q->cons_phys;
	info->cons_virt		= q->cons_virt;

	return 0;
}

