/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#ifndef _MV_MQA_QUEUE_H
#define _MV_MQA_QUEUE_H

#include "drivers/mv_mqa.h"

/** @addtogroup grp_mqa_queue MQA Queue
 *
 *  MQA Queue API documentation
 *
 *  @{
 */

#define QUEUE_BPOOL_ARRAY	(3) /**< Number of BM pools associated with data queue */
#define QUEUE_UNUSED_PARAM	(-1)/**< Number of BM pools associated with data queue */

/**
 * struct mqa_queue_params - MQA Queue Params
 */
struct mqa_queue_params {

	u32 idx;	/**< Queue index */
	u32 len;	/**< Ring length */
	u32 size;	/**< Ring element size */

	/**< Attributes for Queue definition (bitwise):.  */
	/**<   EGRESS_QUEUE  - To define as Egress Queue. */
	/**<   INGRESS_QUEUE - To define as Ingress Queue.*/
	/**<   LOCAL_QUEUE   - To define as Local Queue.  */
	/**<   REMOTE_QUEUE  - To define as Remote Queue. */

	u32 attr;
	u32 prio;	/**< Priority   */

	void *phy_base_addr;	/**< Ring physical base address */
	void *virt_base_addr;   /**< Ring virtual base address */
	void *prod_phys;	/**< Producer physical address */
	void *cons_phys;        /**< Consumer physical address */
	void *prod_virt;        /**< Producer virtual address */
	void *cons_virt;        /**< Consumer virtual address */
	void *remote_phy_addr;	/**< Remote Physical address (== NULL if local queue) */
	void *host_remap;

	u32 msix_id;	/**< MSI-X interrupt Id (0 = unused) */
	u32 msi_id;	/**< MSI interrupt Id   (0 = unused) */
	u32 peer_id;	/**< Peer Id            (0 = unused) */
	u32 bpool_num;	/**< Number of BPools   (0 = unused) */

	/** List of BPool queue Id  (0 = unused) */
	s32 bpool_qids[MQA_BM_QUEUE_ARRAY];

	/** Whether to copy the payload or not */
	int copy_payload;
};

struct mqa_q;

/**
 *	Allocate MQA queue.
 *
 *	@param[in]	mqa	A pointer to MQA object
 *	@param[in]	q	A pointer to MQA queue object
 *
 *	@retval	q_Id on success
 *	@retval	error-code otherwise
 */
int mqa_queue_alloc(struct mqa *mqa, u32 *q);

/**
 *	Destroy a queue config in the MQA.
 *
 *	@param[in]	mqa		A pointer to MQA object
 *	@param[in]	queue_id	queue Id
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int mqa_queue_free(struct mqa *mqa, u32 queue_id);

/**
 *	A wrapper function for queue allocation.
 *	parameters received from management commands.
 *	The function updates return parameters if required via
 *	q_params object.
 *
 *	@param[in]	mqa		A pointer to MQA object
 *	@param[in/out]	queue_params	A pointer to queue object
 *	@param[out]	q		A pointer to MQA queue object
 *
 *	@retval	q_Id on success
 *	@retval	error-code otherwise
 */
int mqa_queue_create(struct mqa *mqa, struct mqa_queue_params *q_params, struct mqa_q **q);

/**
 *	Release all queue allocated memory and return queue Id to MQA.
 *
 *	@param[in]	mqa	A pointer to MQA object
 *	@param[in]	q	A pointer to MQA queue object
 *
 *	@retval	q_Id on success
 *	@retval	error-code otherwise
 */
int mqa_queue_destroy(struct mqa *mqa, struct mqa_q *q);

/**
 *	Associate src queue with target dest queue Id.
 *
 *	@param[in]	mqa		A pointer to MQA object
 *	@param[in]	src_queue_id	source queue
 *	@param[in]	dest_queue_id	destination queue
 *
 *	@retval	0 on success
 *	@retval	<0 on failure
 */
int mqa_queue_associate_pair(struct mqa *mqa, u32 src_queue_id, u32 dest_queue_id);

/** @} */ /* end of grp_mqa_queue */

#endif /* _MV_MQA_QUEUE_H */

