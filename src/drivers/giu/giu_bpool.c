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

#include "std_internal.h"
#include "drivers/mv_giu_bpool.h"
#include "giu_queue_topology.h"

#include "lib/lib_misc.h"

struct giu_bpool giu_bpools[GIU_BPOOL_NUM_POOLS] = {0};

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

	if (giu_bpool_put_buffs(pool, buff, &buf_num))
		goto error;

	/* Check that the buffer was added */
	if (buf_num != 1)
		goto error;

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
#ifdef GIU_GPIO_DEBUG
	u32 free_count, cons_val;
#endif

	if (!num_bpds) {
#ifdef GIU_GPIO_DEBUG
		pr_debug("num_bpds is zero\n");
#endif
		return 0;
	}

	/* Get queue params */
	bpq = pool->queue;

	/* Get ring base */
	buf_desc = (struct giu_buff_inf *)bpq->desc_ring_base;

#ifdef GIU_GPIO_DEBUG
	/* Read producer index */
	cons_val = *bpq->cons_addr;

	if (num_bpds > bpq->desc_total) {
		pr_debug("More tx_descs(%u) than txq_len(%u) (BPool %d)\n",
			num_bpds, bpq->desc_total, pool->id);
	}

	/* Calculate number of free descriptors */
	free_count = QUEUE_SPACE(bpq->prod_val_shadow, cons_val, bpq->desc_total);

	if (QUEUE_FULL(bpq->prod_val_shadow, cons_val, bpq->desc_total)) {
		pr_err("GIU BPool %d: No free descriptors for transmitting the packets\n", pool->id);
		return -1;
	}

	if (unlikely(free_count < num_bpds)) {
		pr_debug("num_bpds(%d), free_count(%d) (BPool %d)\n", num_bpds, free_count, pool->id);

		num_bpds = free_count;
	}
#endif

	/* In case there is a wrap-around, handle the number of desc till the end of queue */
	block_size = min(num_bpds, (u16)(bpq->desc_total - bpq->prod_val_shadow));

	desc_remain = num_bpds;
	index = 0; /* index in source descriptor array */

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

	/* make sure all writes are done (i.e. descriptor were copied)
	 * before incrementing the producer index
	 */
	wmb();

	/* Update Producer index in GNPT */
	*bpq->prod_addr = bpq->prod_val_shadow;

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

	return QUEUE_OCCUPANCY(*bpq->prod_addr, *bpq->cons_addr, bpq->desc_total);
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
