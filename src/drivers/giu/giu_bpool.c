/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#include "std_internal.h"
#include "drivers/mv_mqa.h"
#include "drivers/mv_mqa_queue.h"
#include "drivers/mv_giu_gpio_init.h"
#include "drivers/mv_giu_bpool.h"
#include "drivers/mqa/mqa_internal.h"
#include "hw_emul/gie.h"
#include "giu_queue_topology.h"
#include "giu_internal.h"
#include "lib/lib_misc.h"

struct giu_bpool giu_bpools[GIU_BPOOL_NUM_POOLS] = {0};

int giu_bpool_init(struct giu_gpio_init_params *init_params, struct giu_bpool **bpool)
{
	int ret;
	u32 bm_idx, tc_idx;
	struct mqa_queue_params params;
	struct giu_gpio_outtc_params *outtc;
	struct giu_gpio_intc_params *intc;
	union giu_gpio_q_params *giu_gpio_q_p;

	*bpool = kcalloc(1, sizeof(struct giu_bpool), GFP_KERNEL);
	if (*bpool == NULL) {
		pr_err("Failed to allocate GIU BPOOL handler\n");
		goto error;
	}

	(*bpool)->params = init_params;

	pr_debug("Initializing Local BM queues\n");

	/* Create Local BM queues */
	for (tc_idx = 0; tc_idx < init_params->intcs_params.num_intcs; tc_idx++) {

		intc = &(init_params->intcs_params.intc_params[tc_idx]);

		for (bm_idx = 0; bm_idx < intc->num_inpools; bm_idx++) {

			giu_gpio_q_p = &(intc->pools[bm_idx]);

			memset(&params, 0, sizeof(struct mqa_queue_params));

			params.idx  = giu_gpio_q_p->lcl_q.q_id;
			params.len  = giu_gpio_q_p->lcl_q.len;
			params.size = gie_get_desc_size(BUFF_DESC);
			params.attr = LOCAL_QUEUE | EGRESS_QUEUE;

			ret = mqa_queue_create(init_params->mqa, &params, &(giu_gpio_q_p->lcl_q.q));
			if (ret < 0) {
				pr_err("Failed to allocate Local BM queue %d\n", params.idx);
				goto lcl_queue_error;
			}

			/* Register Local BM Queue to GIU */
			ret = gie_add_bm_queue(init_params->gie->tx_gie, params.idx,
									intc->pool_buf_size, GIU_LCL_Q_IND);
			if (ret) {
				pr_err("Failed to register BM Queue %d to GIU\n", params.idx);
				goto lcl_queue_error;
			}
			pr_debug("Local BM[%d], queue Id %d, Registered to GIU TX\n\n", bm_idx, params.idx);
		}
	}


	pr_debug("Initializing Remote BM queues\n");

	/* Create Remote BM queues */
	for (tc_idx = 0; tc_idx < init_params->outtcs_params.num_outtcs; tc_idx++) {

		outtc = &(init_params->outtcs_params.outtc_params[tc_idx]);

		for (bm_idx = 0; bm_idx < outtc->num_rem_inqs; bm_idx++) {

			giu_gpio_q_p = &(outtc->rem_poolqs_params[bm_idx]);

			memset(&params, 0, sizeof(struct mqa_queue_params));

			params.idx             = giu_gpio_q_p->rem_q.q_id;
			params.len             = giu_gpio_q_p->rem_q.len;
			params.size            = giu_gpio_q_p->rem_q.size;
			params.attr            = (u32)(REMOTE_QUEUE | EGRESS_QUEUE);
			params.remote_phy_addr = (void *)giu_gpio_q_p->rem_q.q_base_pa;
			params.cons_phys       = (void *)giu_gpio_q_p->rem_q.cons_base_pa;
			params.cons_virt       = giu_gpio_q_p->rem_q.cons_base_va;
			params.host_remap      = giu_gpio_q_p->rem_q.host_remap;

			ret = mqa_queue_create(init_params->mqa, &params,  &(giu_gpio_q_p->rem_q.q));
			if (ret < 0) {
				pr_err("Failed to allocate Host BM queue %d\n", params.idx);
				goto host_queue_error;
			}

			/* Register Host BM Queue to GIU */
			ret = gie_add_bm_queue(init_params->gie->rx_gie, params.idx, params.size, GIU_REM_Q_IND);
			if (ret) {
				pr_err("Failed to register BM Queue %d to GIU\n", params.idx);
				goto host_queue_error;
			}
			pr_debug("Host TC[%d] BM-pool[%d], queue Id %d, Registered to GIU RX\n\n",
				 tc_idx, bm_idx, params.idx);
		}
	}

	return 0;

host_queue_error:

	for (tc_idx = 0; tc_idx < init_params->outtcs_params.num_outtcs; tc_idx++) {

		outtc = &(init_params->outtcs_params.outtc_params[tc_idx]);

		for (bm_idx = 0; bm_idx < outtc->num_rem_inqs; bm_idx++) {

			giu_gpio_q_p = &(outtc->rem_poolqs_params[bm_idx]);

			if (giu_gpio_q_p->rem_q.q_id) {
				ret = gie_remove_bm_queue(init_params->gie->rx_gie, giu_gpio_q_p->rem_q.q_id);
				if (ret)
					pr_err("Failed to remove queue Idx %x from GIU TX\n", giu_gpio_q_p->rem_q.q_id);

				ret = mqa_queue_destroy(init_params->mqa, giu_gpio_q_p->rem_q.q);
				if (ret)
					pr_err("Failed to free queue Idx %x in DB\n", giu_gpio_q_p->rem_q.q_id);

				ret = mqa_queue_free(init_params->mqa, (u32)giu_gpio_q_p->rem_q.q_id);
				if (ret)
					pr_err("Failed to free queue Idx %x in MQA\n", giu_gpio_q_p->rem_q.q_id);
			}
		}
	}

lcl_queue_error:

	for (tc_idx = 0; tc_idx < init_params->outtcs_params.num_outtcs; tc_idx++) {

		intc = &(init_params->intcs_params.intc_params[tc_idx]);

		for (bm_idx = 0; bm_idx < intc->num_inpools; bm_idx++) {

			giu_gpio_q_p = &(intc->pools[bm_idx]);

			if (giu_gpio_q_p->lcl_q.q_id) {
				ret = gie_remove_bm_queue(init_params->gie->tx_gie, giu_gpio_q_p->lcl_q.q_id);
				if (ret)
					pr_err("Failed to remove queue Idx %x from GIU TX\n", giu_gpio_q_p->lcl_q.q_id);

				ret = mqa_queue_destroy(init_params->mqa, giu_gpio_q_p->lcl_q.q);
				if (ret)
					pr_err("Failed to free queue Idx %x in DB\n", giu_gpio_q_p->lcl_q.q_id);

				ret = mqa_queue_free(init_params->mqa, (u32)giu_gpio_q_p->lcl_q.q_id);
				if (ret)
					pr_err("Failed to free queue Idx %x in MQA\n", giu_gpio_q_p->lcl_q.q_id);
			}
		}
	}

error:

	return -ENOMEM;
}

void giu_bpool_deinit(struct giu_bpool *bpool)
{
	int ret;
	u32 bm_idx, tc_idx;
	union giu_gpio_q_params *giu_gpio_q_p;
	struct giu_gpio_outtc_params *outtc;
	struct giu_gpio_intc_params *intc;
	struct giu_gpio_init_params *init_params = (struct giu_gpio_init_params *)(bpool->params);

	pr_debug("De-initializing Remote BM queues\n");

	for (tc_idx = 0; tc_idx < init_params->outtcs_params.num_outtcs; tc_idx++) {

		outtc = &(init_params->outtcs_params.outtc_params[tc_idx]);

		for (bm_idx = 0; bm_idx < outtc->num_rem_inqs; bm_idx++) {

			giu_gpio_q_p = &(outtc->rem_poolqs_params[bm_idx]);
			if (giu_gpio_q_p->rem_q.q_id) {
				ret = giu_queue_remove(init_params->mqa, giu_gpio_q_p->rem_q.q,
								HOST_BM_QUEUE, init_params->gie->rx_gie);
				if (ret)
					pr_err("Failed to remove queue Idx %x\n", giu_gpio_q_p->rem_q.q_id);
			}
		}
	}

	pr_debug("De-initializing Local BM queues\n");

	for (tc_idx = 0; tc_idx < init_params->intcs_params.num_intcs; tc_idx++) {

		intc = &(init_params->intcs_params.intc_params[tc_idx]);

		for (bm_idx = 0; bm_idx < intc->num_inpools; bm_idx++) {

			giu_gpio_q_p = &(intc->pools[bm_idx]);
			if (giu_gpio_q_p) {
				ret = giu_queue_remove(init_params->mqa, giu_gpio_q_p->lcl_q.q,
								LOCAL_BM_QUEUE, init_params->gie->tx_gie);
				if (ret)
					pr_err("Failed to remove queue Idx %x\n", giu_gpio_q_p->lcl_q.q_id);
			}
		}
	}

	kfree(bpool);
}


/**
 * Probe the Buffer Pool (bpool)
 */
int giu_bpool_probe(char *match, char *regfile_name, struct giu_bpool **bpool)
{
	u8 match_params[2];
	struct giu_gpio_params *giu_gpio_params;
	struct giu_gpio_queue *bpq;
	struct giu_bpool *pool;
	int giu_id, bpool_id;
	int err;

	if (mv_sys_match(match, "giu_pool", 2, match_params))
		return(-ENXIO);

	giu_id = match_params[0];
	bpool_id = match_params[1];

	if (bpool_id >= GIU_BPOOL_NUM_POOLS) {
		pr_err("[%s] Cannot allocate Pool. No free BPool\n", __func__);
		return(-ENODEV);
	}

	if (giu_bpools[bpool_id].queue != NULL) {
		pr_err("[%s] BPool id %d is already in use\n", __func__, bpool_id);
		return (-EEXIST);
	}

	pr_debug("[%s] giu_id(%d) pool_id(%d)\n", __func__, giu_id, bpool_id);

	/* Init queue topology */
	err = giu_gpio_init_topology(giu_id, regfile_name);
	if (err) {
		pr_err("[%s] GIU topology init failed (%d)\n", __func__, err);
		return -1;
	}

	giu_gpio_params = giu_gpio_get_topology(giu_id);
	if (giu_gpio_params == NULL) {
		pr_err("[%s] queue topology was not initialized for GIU %d\n", __func__, giu_id);
		return -1;
	}

	bpq = &giu_gpio_params->bpool;

	pool = &giu_bpools[bpool_id];
	pool->id = bpool_id;
	pool->giu_id = giu_id;
	pool->buff_len = bpq->buff_len;
	pool->queue = bpq;

	*bpool = pool;

	pr_debug("giu_bpool_probe pool->id %d\n", pool->id);

	return 0;
}

/**
 * Remove a Buffer Pool (bpool)
 */
void giu_bpool_remove(struct giu_bpool *pool)
{
	memset(&giu_bpools[pool->id], 0, sizeof(struct giu_bpool));
}

/**
 * Add a buffer to a giu buffer pool.
 */
int giu_bpool_put_buff(struct giu_bpool *pool, struct giu_buff_inf *buff)
{
	u16 buf_num = 1;

	if (unlikely(giu_bpool_put_buffs(pool, buff, &buf_num)))
		goto error;

	/* Check that the buffer was added */
	if (buf_num != 1)
		return -EBUSY;

	return 0;

error:
	pr_err("GIU BPool: Failed to add a buffer to BP %d\n", pool->id);
	return -1;
}

/**
 * Add bulk of buffers to a giu buffer pool.
 */
int giu_bpool_put_buffs(struct giu_bpool *pool, struct giu_buff_inf buff_entry[], u16 *num)
{
	struct giu_gpio_queue *bpq;
	struct giu_buff_inf *buf_desc;
	u16 num_bpds = *num, desc_remain;
	u16 block_size, index;
	u32 free_count, cons_val;

	/* Get queue params */
	bpq = pool->queue;

	/* Read consumer index */
	cons_val = readl(bpq->cons_addr);

	/* Calculate number of free descriptors */
	free_count = QUEUE_SPACE(bpq->prod_val_shadow, cons_val, bpq->desc_total);

	if (unlikely(free_count < num_bpds)) {
		pr_debug("num_bpds(%d), free_count(%d) (BPool %d)\n", num_bpds, free_count, pool->id);
		num_bpds = free_count;
	}

	if (unlikely(!num_bpds)) {
		pr_debug("BPool full\n");
		*num = 0;
		return 0;
	}

	/* In case there is a wrap-around, handle the number of desc till the end of queue */
	block_size = min(num_bpds, (u16)(bpq->desc_total - bpq->prod_val_shadow));

	desc_remain = num_bpds;
	index = 0; /* index in source descriptor array */

	/* Get ring base */
	buf_desc = (struct giu_buff_inf *)bpq->desc_ring_base;

	/* In case there is a wrap-around, the first iteration will handle the
	 * descriptors till the end of queue. The rest will be handled at the
	 * following iteration.
	 * Note that there should be no more than 2 iterations.
	 **/
	do {
		/* Copy bulk of descriptors to descriptor ring */
		memcpy(&buf_desc[bpq->prod_val_shadow], &buff_entry[index], sizeof(struct giu_buff_inf) * block_size);

		/* Increment producer index, update remaining descriptors count and block size */
		bpq->prod_val_shadow = QUEUE_INDEX_INC(bpq->prod_val_shadow, block_size, bpq->desc_total);
		desc_remain -= block_size;
		index = block_size;	/* next desc index in source array */
		block_size = desc_remain; /* next desc index in target ring */
	} while (desc_remain > 0);

	/* Update Producer index in GNPT */
	/* make sure all writes are done (i.e. descriptor were copied)
	 * before incrementing the producer index
	 */
	writel(bpq->prod_val_shadow, bpq->prod_addr);

	/* Update number of updated descriptors */
	*num = num_bpds;

	return 0;
}

/**
 * Get the number of buffers in giu buffer pool.
 */
int giu_bpool_get_num_buffs(struct giu_bpool *pool, u32 *num_buffs)
{
	struct giu_gpio_queue *bpq = pool->queue;

	*num_buffs = QUEUE_OCCUPANCY(readl_relaxed(bpq->prod_addr), readl_relaxed(bpq->cons_addr), bpq->desc_total);

	return 0;
}

/**
 * Get the buffer pool capabilities.
 */
int giu_bpool_get_capabilities(struct giu_bpool *bpool, struct giu_bpool_capabilities *capa)
{
	struct giu_gpio_queue *bpq = bpool->queue;

	capa->buff_len = bpool->buff_len;
	capa->max_num_buffs = bpq->desc_total - 1;

	return 0;
}
