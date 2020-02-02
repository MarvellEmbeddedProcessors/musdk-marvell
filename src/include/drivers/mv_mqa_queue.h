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

/* MQA Queue attributes */
#define MQA_QUEUE_EGRESS	(0 << 0)
#define MQA_QUEUE_INGRESS	(1 << 0)
#define MQA_QUEUE_LOCAL		(0 << 1)
#define MQA_QUEUE_REMOTE	(1 << 1)

/**
 * struct mqa_queue_msix_params - MQA Queue MSI-X Params
 */
struct mqa_queue_msix_params {
	u32		 id;	/**< MSI interrupt Id   (0 = unused) */
	u32		 data;
	phys_addr_t	 pa;
	void		*va;
};

/**
 * struct mqa_queue_params - MQA Queue Params
 */
struct mqa_queue_params {

	u32 idx;	/**< Queue index */
	u32 len;	/**< Ring length */
	u32 size;	/**< Ring element size */

	/**< Attributes for Queue definition (bitwise):.  */
	/**<   MQA_QUEUE_EGRESS  - To define as Egress Queue. */
	/**<   MQA_QUEUE_INGRESS - To define as Ingress Queue.*/
	/**<   MQA_QUEUE_LOCAL   - To define as Local Queue.  */
	/**<   MQA_QUEUE_REMOTE  - To define as Remote Queue. */

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

	struct mqa_queue_msix_params msix_inf;	/**< MSI-X interrupt information */
	u32 peer_id;	/**< Peer Id            (0 = unused) */
	u32 bpool_num;	/**< Number of BPools   (0 = unused) */

	/** List of BPool queue Id  (0 = unused) */
	s32 bpool_qids[MQA_BM_QUEUE_ARRAY];

	/** Whether to copy the payload or not */
	int copy_payload;

	/** S/G support */
	int sg_en;
};

/**
 * struct mqa_queue_info - MQA Queue Info
 */
struct mqa_queue_info {
	u32 q_id;
	u32 len;
	void *phy_base_addr;
	void *virt_base_addr;
	void *prod_phys;
	void *cons_phys;
	void *prod_virt;
	void *cons_virt;
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

/**
 *	Retrieve queue-id from MQA queue object.
 *
 *	@param[in]	mqa	A pointer to MQA object
 *	@param[in]	q	A pointer to MQA queue object
 *	@param[out]	queue_id	queue Id
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int mqa_queue_get_id(struct mqa_q *q, u32 *queue_id);

/**
 *	Retrieve queue's info from MQA queue object.
 *
 *	@param[in]	q	A pointer to MQA queue object
 *	@param[out]	info	queue's info
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int mqa_queue_get_info(struct mqa_q *q, struct mqa_queue_info *info);

static inline u32 mqa_queue_inc_idx_val(struct mqa_queue_info *info, u32 idx, u32 val)
{
	return ((idx + val) & (info->len - 1));
}

static inline u32 mqa_queue_rd_cons(struct mqa_queue_info *info)
{
	return readl_relaxed(info->cons_virt);
}

static inline u32 mqa_queue_rd_prod(struct mqa_queue_info *info)
{
	return readl_relaxed(info->prod_virt);
}

static inline void mqa_queue_wr_cons(struct mqa_queue_info *info, u32 val)
{
	writel_relaxed(val, info->cons_virt);
}

static inline void mqa_queue_wr_prod(struct mqa_queue_info *info, u32 val)
{
	writel_relaxed(val, info->prod_virt);
}

static inline int mqa_queue_is_full(struct mqa_queue_info *info, u32 prod_val, u32 cons_val)
{
	return (((prod_val + 1) & (info->len - 1)) == cons_val);
}

static inline int mqa_queue_is_empty(struct mqa_queue_info *info, u32 prod_val, u32 cons_val)
{
	return (prod_val == cons_val);
}

static inline u32 mqa_queue_occupancy(struct mqa_queue_info *info, u32 prod_val, u32 cons_val)
{
	return ((prod_val - cons_val + info->len) & (info->len - 1));
}

static inline u32 mqa_queue_space(struct mqa_queue_info *info, u32 prod_val, u32 cons_val)
{
	return (info->len - mqa_queue_occupancy(info, prod_val, cons_val) - 1);
}

/** @} */ /* end of grp_mqa_queue */

#endif /* _MV_MQA_QUEUE_H */

