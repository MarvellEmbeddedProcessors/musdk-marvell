/******************************************************************************
 *	Copyright (C) 2017 Marvell International Ltd.
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

#define log_fmt(fmt) "giu: " fmt

#include "std_internal.h"
#include "drivers/mv_mqa.h"
#include "drivers/mv_mqa_queue.h"
#include "hw_emul/gie.h"
#include "giu_queue_topology.h"
#include "giu_internal.h"

int giu_free_tc_queues(struct mqa *mqa, struct giu_gpio_q *giu_gpio__q, u32 queue_num, void *gie)
{
	u32 q_idx;
	int ret;
	struct giu_gpio_q *giu_gpio_q_p;

	for (q_idx = 0; q_idx < queue_num; q_idx++) {
		giu_gpio_q_p = &(giu_gpio__q[q_idx]);
		if (giu_gpio_q_p != NULL) {
			mqa_queue_free(mqa, (u32)giu_gpio_q_p->params.idx);

			/* If needed, unregister the queue from GIU polling */
			if (gie) {
				ret = gie_remove_queue(gie, giu_gpio_q_p->params.idx);
				if (ret)
					pr_err("Failed to remove queue Idx %x from GIU TX\n", giu_gpio_q_p->params.idx);
			}

			memset(giu_gpio_q_p, 0, sizeof(struct giu_gpio_q));
		}
	}

	return 0;
}

/*
 *	nic_pf_queue_remove
 */
int giu_queue_remove(struct mqa *mqa, struct giu_gpio_q *giu_gpio_q_p,
					 enum queue_type queue_type, void *giu_handle)
{
	u32 q_id = giu_gpio_q_p->params.idx;
	int ret = 0;

	pr_debug("Remove queue %d (type %d)\n", q_id, queue_type);

	if (giu_handle) {
		/* Un-register Q from GIU */
		if (queue_type == LOCAL_BM_QUEUE ||
		    queue_type == HOST_BM_QUEUE)
			ret = gie_remove_bm_queue(giu_handle, q_id);
		else
			ret = gie_remove_queue(giu_handle, q_id);
		if (ret)
			pr_err("Failed to remove queue Idx %x from GIU\n", q_id);
	}

	/* For local queue: destroy the queue (as it was allocated by the NIC */
	if (queue_type == LOCAL_INGRESS_DATA_QUEUE ||
	    queue_type == LOCAL_EGRESS_DATA_QUEUE ||
	    queue_type == LOCAL_BM_QUEUE) {
		ret = mqa_queue_destroy(mqa, giu_gpio_q_p->q);
		if (ret)
			pr_err("Failed to free queue Idx %x in DB\n", q_id);
	}

	/* Free the MQA resource */
	ret = mqa_queue_free(mqa, q_id);
	if (ret)
		pr_err("Failed to free queue Idx %x in MQA\n", q_id);

	memset(giu_gpio_q_p, 0, sizeof(struct giu_gpio_q));

	return ret;
}


