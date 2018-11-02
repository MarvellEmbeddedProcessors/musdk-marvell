/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#include "std_internal.h"
#include "drivers/mv_mqa.h"
#include "drivers/mv_mqa_queue.h"
#include "drivers/mv_giu_gpio.h"
#include "drivers/mv_giu_bpool.h"
#include "hw_emul/gie.h"
#include "lib/lib_misc.h"

#include "giu_queue_topology.h"
#include "giu_internal.h"


struct giu_bpool_int {
	u8		 giu_id;
	u8		 id;

	struct mqa	*mqa;
	struct giu	*giu;

	u16		 num_buffs; /**< number of buffers */
	u16		 buff_len; /**< buffer length */

	u32		 q_id;
	struct mqa_q	*mqa_q;

	/* Buffer Pool Q parameters */
	struct giu_gpio_queue	 queue;
};

/* Support only a single GIU */
struct giu_bpool giu_bpools[GIU_BPOOL_NUM_POOLS] = {0};


int giu_bpool_alloc(void)
{
	int i;

	for (i = 0; i < GIU_BPOOL_NUM_POOLS; i++)
		if (!giu_bpools[i].internal_param)
			return i;

	return -ENOENT;
}

int giu_bpool_init(struct giu_bpool_params *params, struct giu_bpool **bpool)
{
	struct giu_bpool_int		*bp_int;
	struct mqa_queue_params		 mqa_params;
	struct mqa_queue_info		 queue_info;
	u8				 match_params[2];
	int				 err, giu_id, bpool_id;

	if (mv_sys_match(params->match, "giu_pool", 2, match_params))
		return(-ENXIO);

	giu_id = match_params[0];
	bpool_id = match_params[1];

	if (bpool_id >= GIU_BPOOL_NUM_POOLS) {
		pr_err("[%s] Cannot allocate Pool. No free BPool\n", __func__);
		return(-ENODEV);
	}

	*bpool = &giu_bpools[bpool_id];
	if ((*bpool)->internal_param) {
		pr_err("Failed to allocate GIU BPOOL handler\n");
		return -EBUSY;
	}

	(*bpool)->giu_id = giu_id;
	(*bpool)->id = bpool_id;
	(*bpool)->buff_len = params->buff_len;

	bp_int = kcalloc(1, sizeof(struct giu_bpool_int), GFP_KERNEL);
	if (bp_int == NULL)
		return -ENOMEM;

	bp_int->giu_id = (*bpool)->giu_id;
	bp_int->id = (*bpool)->id;
	bp_int->buff_len = (*bpool)->buff_len;

	bp_int->mqa = params->mqa;
	bp_int->giu = params->giu;
	bp_int->num_buffs = params->num_buffs;

	pr_debug("Initializing Local BM queues\n");

	/* Allocate queue from MQA */
	err = mqa_queue_alloc(bp_int->mqa, &bp_int->q_id);
	if (err) {
		pr_err("Failed to allocate queue from MQA\n");
		kfree(bp_int);
		return err;
	}

	memset(&mqa_params, 0, sizeof(struct mqa_queue_params));

	mqa_params.idx  = bp_int->q_id;
	mqa_params.len  = bp_int->num_buffs;
	mqa_params.size = gie_get_desc_size(BUFF_DESC);
	mqa_params.attr = MQA_QUEUE_LOCAL | MQA_QUEUE_EGRESS;

	err = mqa_queue_create(bp_int->mqa, &mqa_params, &(bp_int->mqa_q));
	if (err) {
		pr_err("Failed to allocate Local BM queue %d\n", mqa_params.idx);
		mqa_queue_free(bp_int->mqa, bp_int->q_id);
		kfree(bp_int);
		return err;
	}

	/* Register Local BM Queue to GIU */
	err = gie_add_bm_queue(giu_get_gie_handle(bp_int->giu, GIU_ENG_IN),
			mqa_params.idx, bp_int->buff_len, GIU_LCL_Q_IND);
	if (err) {
		pr_err("Failed to register BM Queue %d to GIU\n", mqa_params.idx);
		mqa_queue_destroy(bp_int->mqa, bp_int->mqa_q);
		mqa_queue_free(bp_int->mqa, bp_int->q_id);
		kfree(bp_int);
		return err;
	}

	mqa_queue_get_info(bp_int->mqa_q, &queue_info);

	bp_int->queue.desc_total = queue_info.len;
	bp_int->queue.desc_ring_base = queue_info.virt_base_addr;
	bp_int->queue.last_cons_val = 0;
	bp_int->queue.prod_addr = queue_info.prod_virt;
	bp_int->queue.cons_addr = queue_info.cons_virt;
	bp_int->queue.buff_len = bp_int->buff_len;

	pr_debug("Local BM[%d], queue Id %d, Registered to GIU TX\n\n", bm_idx, mqa_params.idx);

	(*bpool)->internal_param = bp_int;

	return 0;
}

void giu_bpool_deinit(struct giu_bpool *bpool)
{
	struct giu_bpool_int	*bp_int = (struct giu_bpool_int *)bpool->internal_param;

	pr_debug("De-initializing Local BM queues\n");

	gie_remove_bm_queue(giu_get_gie_handle(bp_int->giu, GIU_ENG_IN), bp_int->q_id);
	mqa_queue_destroy(bp_int->mqa, bp_int->mqa_q);
	mqa_queue_free(bp_int->mqa, bp_int->q_id);
	kfree(bp_int);
	giu_bpools[bpool->id].internal_param = NULL;
}

int giu_bpool_serialize(struct giu_bpool *bpool, void **file_map)
{
	struct giu_bpool_int		*bp_int = (struct giu_bpool_int *)bpool->internal_param;
	struct mv_sys_dma_mem_info	 mem_info;
	struct mqa_queue_info		 queue_info;
	struct giu_queue		 reg_giu_queue;
	char				 dev_name[100];

	mem_info.name = dev_name;
	mv_sys_dma_mem_get_info(&mem_info);

	mqa_queue_get_info(bp_int->mqa_q, &queue_info);

	reg_giu_queue.hw_id		= queue_info.q_id;
	/** TODO - change params naming - change reg_giu_queue.size to reg_giu_queue.len*/
	reg_giu_queue.size		= queue_info.len;
	reg_giu_queue.type		= QUEUE_BP;
	reg_giu_queue.buff_len		= bp_int->buff_len;

	reg_giu_queue.phy_base_offset	= (phys_addr_t)(uintptr_t)(queue_info.phy_base_addr - mem_info.paddr);
	/* Prod/Cons addr are Virtual. Needs to translate them to offset */
	reg_giu_queue.prod_offset = (phys_addr_t)(uintptr_t)(queue_info.prod_phys - mem_info.paddr);
	reg_giu_queue.cons_offset = (phys_addr_t)(uintptr_t)(queue_info.cons_phys - mem_info.paddr);

	memcpy(*file_map, &reg_giu_queue, sizeof(struct giu_queue));
	*file_map += sizeof(struct giu_queue);

	/* mark this bpool as use but not for guest */
	giu_bpools[bpool->id].internal_param = (void *)(-1);

	return 0;
}

/**
 * Probe the Buffer Pool (bpool)
 */
int giu_bpool_probe(char *match, char *regfile_name, struct giu_bpool **bpool)
{
	struct giu_gpio_probe_params	*gpio_probe_params;
	struct giu_bpool_int		*bp_int;
	struct giu_gpio_queue		*bpq;
	struct giu_bpool		*pool;
	u8				 match_params[2];
	int				 err, giu_id, bpool_id;

	if (mv_sys_match(match, "giu_pool", 2, match_params))
		return(-ENXIO);

	giu_id = match_params[0];
	bpool_id = match_params[1];

	if (bpool_id >= GIU_BPOOL_NUM_POOLS) {
		pr_err("[%s] Cannot allocate Pool. No free BPool\n", __func__);
		return(-ENODEV);
	}

	pool = &giu_bpools[bpool_id];

	if (pool->internal_param &&
	    (pool->internal_param != (void *)(-1))) {
		pr_err("[%s] BPool id %d is already in use\n", __func__, bpool_id);
		return (-EEXIST);
	}

	pr_debug("[%s] giu_id(%d) pool_id(%d)\n", __func__, giu_id, bpool_id);

	pool->giu_id = giu_id;
	pool->id = bpool_id;

	bp_int = kcalloc(1, sizeof(struct giu_bpool_int), GFP_KERNEL);
	if (bp_int == NULL)
		return -ENOMEM;

	bp_int->giu_id = pool->giu_id;
	bp_int->id = pool->id;

	/* Init queue topology */
	err = giu_gpio_init_topology(bp_int->giu_id, regfile_name);
	if (err) {
		pr_err("[%s] GIU topology init failed (%d)\n", __func__, err);
		return -1;
	}

	gpio_probe_params = giu_gpio_get_topology(bp_int->giu_id);
	if (gpio_probe_params == NULL) {
		pr_err("[%s] queue topology was not initialized for GIU %d\n", __func__, bp_int->giu_id);
		return -1;
	}

	bpq = &gpio_probe_params->bpool;

	memcpy(&bp_int->queue, bpq, sizeof(struct giu_queue));

	bp_int->buff_len = bp_int->queue.buff_len;
	bp_int->num_buffs = bp_int->queue.desc_total;

	pool->internal_param = bp_int;

	*bpool = pool;

	pr_debug("giu_bpool_probe pool->id %d\n", pool->id);

	return 0;
}

/**
 * Remove a Buffer Pool (bpool)
 */
void giu_bpool_remove(struct giu_bpool *bpool)
{
	struct giu_bpool_int	*bp_int = (struct giu_bpool_int *)bpool->internal_param;

	/* mark this bpool as use but not for guest */
	giu_bpools[bpool->id].internal_param = (void *)(-1);
	kfree(bp_int);
}

/**
 * Add a buffer to a giu buffer pool.
 */
int giu_bpool_put_buff(struct giu_bpool *bpool, struct giu_buff_inf *buff)
{
	u16 buf_num = 1;

	if (unlikely(giu_bpool_put_buffs(bpool, buff, &buf_num)))
		goto error;

	/* Check that the buffer was added */
	if (buf_num != 1)
		return -EBUSY;

	return 0;

error:
	pr_err("GIU BPool: Failed to add a buffer to BP %d\n", bpool->id);
	return -1;
}

/**
 * Add bulk of buffers to a giu buffer pool.
 */
int giu_bpool_put_buffs(struct giu_bpool *bpool, struct giu_buff_inf buff_entry[], u16 *num)
{
	struct giu_bpool_int	*bp_int = (struct giu_bpool_int *)bpool->internal_param;
	struct giu_gpio_queue	*bpq = &bp_int->queue;
	struct giu_buff_inf	*buf_desc;
	u16 num_bpds = *num, desc_remain;
	u16 block_size, index;
	u32 free_count, cons_val, prod_val;

	/* Read consumer index */
	cons_val = readl(bpq->cons_addr);
	/* Read producer index */
	prod_val = readl(bpq->prod_addr);

	/* Calculate number of free descriptors */
	free_count = QUEUE_SPACE(prod_val, cons_val, bpq->desc_total);

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
	block_size = min(num_bpds, (u16)(bpq->desc_total - prod_val));

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
		memcpy(&buf_desc[prod_val], &buff_entry[index], sizeof(struct giu_buff_inf) * block_size);

		/* Increment producer index, update remaining descriptors count and block size */
		prod_val = QUEUE_INDEX_INC(prod_val, block_size, bpq->desc_total);
		desc_remain -= block_size;
		index = block_size;	/* next desc index in source array */
		block_size = desc_remain; /* next desc index in target ring */
	} while (desc_remain > 0);

	/* Update Producer index in GNPT */
	/* make sure all writes are done (i.e. descriptor were copied)
	 * before incrementing the producer index
	 */
	writel(prod_val, bpq->prod_addr);

	/* Update number of updated descriptors */
	*num = num_bpds;

	return 0;
}

/**
 * Get the number of buffers in giu buffer pool.
 */
int giu_bpool_get_num_buffs(struct giu_bpool *bpool, u32 *num_buffs)
{
	struct giu_bpool_int	*bp_int = (struct giu_bpool_int *)bpool->internal_param;
	struct giu_gpio_queue	*bpq = &bp_int->queue;

	*num_buffs = QUEUE_OCCUPANCY(readl_relaxed(bpq->prod_addr), readl_relaxed(bpq->cons_addr), bpq->desc_total);

	return 0;
}

/**
 * Get the buffer pool capabilities.
 */
int giu_bpool_get_capabilities(struct giu_bpool *bpool, struct giu_bpool_capabilities *capa)
{
	struct giu_bpool_int	*bp_int = (struct giu_bpool_int *)bpool->internal_param;
	struct giu_gpio_queue	*bpq = &bp_int->queue;

	capa->buff_len = bp_int->buff_len;
	capa->max_num_buffs = bpq->desc_total - 1;

	return 0;
}

int giu_bpool_get_mqa_q_id(struct giu_bpool *bpool)
{
	struct giu_bpool_int		*bp_int = (struct giu_bpool_int *)bpool->internal_param;

	return bp_int->q_id;
}
