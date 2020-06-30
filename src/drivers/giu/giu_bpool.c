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
#include "lib/lib_misc.h"

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
	void		*bm_queue_ref;

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
	u8				 match_params[2], egress_num_dma_engines;
	int				 err, giu_id, bpool_id, i;

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

	bp_int = kcalloc(1, sizeof(struct giu_bpool_int), GFP_KERNEL);
	if (bp_int == NULL)
		return -ENOMEM;

	bp_int->giu_id = (*bpool)->giu_id;
	bp_int->id = (*bpool)->id;
	bp_int->buff_len = params->buff_len;

	bp_int->mqa = params->mqa;
	bp_int->giu = params->giu;
	bp_int->num_buffs = params->num_buffs;

	pr_debug("Initializing Local BM queues\n");

	err = giu_get_num_dma_engines(bp_int->giu, GIU_ENG_IN, &egress_num_dma_engines);
	if (err) {
		pr_err("Failed to retrieve number of dma-engines\n");
		return err;
	}

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
	err = gie_add_bm_queue(giu_get_gie_handle(bp_int->giu, GIU_ENG_IN, 0),
			mqa_params.idx, bp_int->buff_len, GIU_LCL_Q, &bp_int->bm_queue_ref);
	if (err) {
		pr_err("Failed to register BM Queue %d to GIU\n", mqa_params.idx);
		mqa_queue_destroy(bp_int->mqa, bp_int->mqa_q);
		mqa_queue_free(bp_int->mqa, bp_int->q_id);
		kfree(bp_int);
		return err;
	}

	for (i = 1; i < egress_num_dma_engines; i++)
		gie_add_bm_queue_reference(giu_get_gie_handle(bp_int->giu, GIU_ENG_IN, i), bp_int->bm_queue_ref);

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
	u8			 egress_num_dma_engines;
	int			 i;

	pr_debug("De-initializing Local BM queues\n");

	giu_get_num_dma_engines(bp_int->giu, GIU_ENG_IN, &egress_num_dma_engines);

	for (i = 1; i < egress_num_dma_engines; i++)
		gie_remove_bm_queue_refernce(giu_get_gie_handle(bp_int->giu, GIU_ENG_IN, i), bp_int->bm_queue_ref);

	gie_remove_bm_queue(giu_get_gie_handle(bp_int->giu, GIU_ENG_IN, 0), bp_int->q_id);
	mqa_queue_destroy(bp_int->mqa, bp_int->mqa_q);
	mqa_queue_free(bp_int->mqa, bp_int->q_id);
	kfree(bp_int);
	giu_bpools[bpool->id].internal_param = NULL;
}

int giu_bpool_serialize(struct giu_bpool *bpool, char *buff, u32 size, u8 depth)
{
	struct giu_bpool_int		*bp_int;
	size_t				 pos = 0;
	struct mv_sys_dma_mem_info	 mem_info;
	char				 dev_name[100];
	struct mqa_queue_info		 queue_info;
	u32				 offs;

	if (!bpool) {
		pr_err("invalid bpool handle!\n");
		return -EINVAL;
	}
	bp_int = (struct giu_bpool_int *)bpool->internal_param;
	if (!bp_int) {
		pr_err("invalid bpool handle!\n");
		return -EINVAL;
	}

	mem_info.name = dev_name;
	mv_sys_dma_mem_get_info(&mem_info);

	json_print_to_buffer(buff, size, depth, "\"giu_pool-%d:%d\": {\n",
			     bpool->giu_id, bpool->id);
	json_print_to_buffer(buff, size, depth + 1, "\"giu_id\": %d,\n", bpool->giu_id);
	json_print_to_buffer(buff, size, depth + 1, "\"id\": %d,\n", bpool->id);
	json_print_to_buffer(buff, size, depth + 1, "\"dma_dev_name\": \"%s\",\n", mem_info.name);
	json_print_to_buffer(buff, size, depth + 1, "\"num_buffs\": %u,\n", bp_int->num_buffs);
	json_print_to_buffer(buff, size, depth + 1, "\"buff_len\": %u,\n", bp_int->buff_len);
	mqa_queue_get_info(bp_int->mqa_q, &queue_info);
	offs = (phys_addr_t)(uintptr_t)queue_info.phy_base_addr - mem_info.paddr;
	json_print_to_buffer(buff, size, depth + 1, "\"phy_base_offset\": %#x,\n", offs);
	offs = (phys_addr_t)(uintptr_t)queue_info.prod_phys - mem_info.paddr;
	json_print_to_buffer(buff, size, depth + 1, "\"prod_offset\": %#x,\n", offs);
	offs = (phys_addr_t)(uintptr_t)queue_info.cons_phys - mem_info.paddr;
	json_print_to_buffer(buff, size, depth + 1, "\"cons_offset\": %#x,\n", offs);
	json_print_to_buffer(buff, size, depth, "},\n");

	return pos;
}

int giu_bpool_probe(char *match, char *buff, struct giu_bpool **bpool)
{
	struct giu_bpool		*_bpool;
	struct giu_bpool_int		*bp_int;
	struct sys_iomem_params		 iomem_params;
	struct sys_iomem_info		 sys_iomem_info;
	char				*lbuff, *sec = NULL;
	char				 dev_name[FILE_MAX_LINE_CHARS];
	u8				 match_params[2];
	u32				 offs = 0;
	u8				 giu_id = 0, bpool_id = 0;

	if (!match) {
		pr_err("no match string found!\n");
		return -EFAULT;
	}

	lbuff = kcalloc(1, strlen(buff), GFP_KERNEL);
	if (lbuff == NULL)
		return -ENOMEM;

	memcpy(lbuff, buff, strlen(buff));
	sec = lbuff;

	/* Search for match (giu_pool-x:x) */
	sec = strstr(sec, match);
	if (!sec) {
		pr_err("match not found %s\n", match);
		kfree(lbuff);
		return -ENXIO;
	}

	if (mv_sys_match(match, "giu_pool", 2, match_params)) {
		kfree(lbuff);
		return -ENXIO;
	}

	/* Retireve giu_id and pool-id */
	json_buffer_to_input(sec, "giu_id", giu_id);
	json_buffer_to_input(sec, "id", bpool_id);

	if ((giu_id != match_params[0]) || (bpool_id != match_params[1])) {
		pr_err("IDs mismatch!\n");
		kfree(lbuff);
		return -EFAULT;
	}
	if (bpool_id >= GIU_BPOOL_NUM_POOLS) {
		pr_err("[%s] Cannot allocate Pool. No free BPool\n", __func__);
		kfree(lbuff);
		return -ENODEV;
	}

	_bpool = &giu_bpools[bpool_id];

	if (_bpool->internal_param) {
		pr_err("[%s] BPool id %d is already in use\n", __func__, bpool_id);
		kfree(lbuff);
		return -EEXIST;
	}

	pr_debug("probing bpool %d for giu id: %d.\n", giu_id, bpool_id);

	_bpool->giu_id = giu_id;
	_bpool->id = bpool_id;

	bp_int = kcalloc(1, sizeof(struct giu_bpool_int), GFP_KERNEL);
	if (bp_int == NULL) {
		kfree(lbuff);
		return -ENOMEM;
	}

	bp_int->giu_id = _bpool->giu_id;
	bp_int->id = _bpool->id;

	memset(dev_name, 0, FILE_MAX_LINE_CHARS);
	json_buffer_to_input_str(sec, "dma_dev_name", dev_name);
	if (dev_name[0] == 0) {
		pr_err("'dma_dev_name' not found\n");
		kfree(lbuff);
		return -EFAULT;
	}

	iomem_params.type = SYS_IOMEM_T_SHMEM;
	iomem_params.devname = dev_name;
	iomem_params.index = 1;

	if (sys_iomem_get_info(&iomem_params, &sys_iomem_info)) {
		pr_err("sys_iomem_get_info error\n");
		kfree(lbuff);
		return -EFAULT;
	}

	/* Retireve BM params */
	json_buffer_to_input(sec, "num_buffs", bp_int->num_buffs);
	bp_int->queue.desc_total = bp_int->num_buffs;
	json_buffer_to_input(sec, "buff_len", bp_int->buff_len);
	bp_int->queue.buff_len = bp_int->buff_len;
	json_buffer_to_input(sec, "phy_base_offset", offs);
	bp_int->queue.desc_ring_base =
		(struct giu_gpio_desc *)((uintptr_t)sys_iomem_info.u.shmem.va + offs);
	json_buffer_to_input(sec, "prod_offset", offs);
	bp_int->queue.prod_addr =
		(u32 *)((uintptr_t)sys_iomem_info.u.shmem.va + offs);
	json_buffer_to_input(sec, "cons_offset", offs);
	bp_int->queue.cons_addr =
		(u32 *)((uintptr_t)sys_iomem_info.u.shmem.va + offs);
	bp_int->queue.last_cons_val = 0;

	pr_debug("q_len %d, buff_len %d, desc_ring_base %p, prod_addr %p, cons_addr %p\n",
		bp_int->num_buffs, bp_int->buff_len, (unsigned int *)bp_int->queue.desc_ring_base,
		bp_int->queue.prod_addr, bp_int->queue.cons_addr);

	kfree(lbuff);
	_bpool->internal_param = bp_int;
	*bpool = _bpool;

	pr_debug("giu_bpool_probe pool->id %d\n", _bpool->id);

	return 0;
}

/**
 * Remove a Buffer Pool (bpool)
 */
void giu_bpool_remove(struct giu_bpool *bpool)
{
	struct giu_bpool_int	*bp_int = (struct giu_bpool_int *)bpool->internal_param;

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
	struct giu_bpool_int	*bp_int = (struct giu_bpool_int *)bpool->internal_param;

	return bp_int->q_id;
}

void giu_bpool_reset(struct giu_bpool *bpool)
{
	struct giu_bpool_int	*bp_int = (struct giu_bpool_int *)bpool->internal_param;
	struct giu_gpio_queue	*bpq = &bp_int->queue;

	writel(0, bpq->cons_addr);
	writel(0, bpq->prod_addr);
}
