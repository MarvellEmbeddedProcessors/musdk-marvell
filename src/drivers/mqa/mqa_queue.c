/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#define log_fmt(fmt) "queue: " fmt

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
	queue_free(queue_id);

	return 0;
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
	ret = queue_associate_notify_intr(mqa, queue_params->idx, queue_params->msix_id);
	if (ret != 0) {
		/* ***Check*** We may need deconfig??? to remove the mqa_queue_config activity */
		pr_err("Failed to associate Notification MSI-X interrupt Id (Phy Q: %d, MSI-X interrupt Id: %d)\n",
			  queue_params->idx, queue_params->msix_id);
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
	if (q->phy_base_addr != NULL)
		mv_sys_dma_mem_free(q->phy_base_addr);

	return 0;
}
