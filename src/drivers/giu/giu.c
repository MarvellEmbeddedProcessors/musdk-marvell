/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#define log_fmt(fmt) "giu: " fmt

#include "std_internal.h"
#include "drivers/mv_mqa.h"
#include "drivers/mv_mqa_queue.h"
#include "drivers/mv_giu_gpio.h"
#include "hw_emul/gie.h"
#include "giu_queue_topology.h"
#include "giu_internal.h"

/*
 *	giu_free_tc_queues
 */
int giu_free_tc_queues(struct mqa *mqa, union giu_gpio_q_params *giu_gpio_q_p,
					 u32 queue_num, u32 queue_type, void *gie)
{
	u32 q_idx;
	u32 q_num;
	int ret;

	for (q_idx = 0; q_idx < queue_num; q_idx++) {

		(queue_type == GIU_LCL_Q_IND) ?
			(q_num = giu_gpio_q_p->lcl_q.q_id) :
			(q_num = giu_gpio_q_p->rem_q.q_id);

		mqa_queue_free(mqa, q_num);

		/* If needed, unregister the queue from GIU polling */
		if (gie) {
			ret = gie_remove_queue(gie, q_num);
			if (ret)
				pr_err("Failed to remove queue Idx %x from GIU TX\n", q_num);
		}
	}

	return 0;
}

/*
 *	nic_pf_queue_remove
 */
int giu_queue_remove(struct mqa *mqa, struct mqa_q *q,
					 enum queue_type queue_type, void *giu_handle)
{
	int ret = 0;
	u32 qid;

	pr_debug("Remove queue %d (type %d)\n", q->q_id, queue_type);

	mqa_queue_get_id(q, &qid);
	if (giu_handle) {
		/* Un-register Q from GIU */
		if (queue_type == LOCAL_BM_QUEUE ||
		    queue_type == HOST_BM_QUEUE)
			ret = gie_remove_bm_queue(giu_handle, qid);
		else
			ret = gie_remove_queue(giu_handle, qid);
		if (ret)
			pr_err("Failed to remove queue Idx %x from GIU\n", qid);
	}

	/* For local queue: destroy the queue (as it was allocated by the NIC */
	if (queue_type == LOCAL_INGRESS_DATA_QUEUE ||
	    queue_type == LOCAL_EGRESS_DATA_QUEUE ||
	    queue_type == LOCAL_BM_QUEUE) {
		ret = mqa_queue_destroy(mqa, q);
		if (ret)
			pr_err("Failed to free queue Idx %x in DB\n", qid);
	}

	/* Free the MQA resource */
	ret = mqa_queue_free(mqa, qid);
	if (ret)
		pr_err("Failed to free queue Idx %x in MQA\n", qid);

	return ret;
}


