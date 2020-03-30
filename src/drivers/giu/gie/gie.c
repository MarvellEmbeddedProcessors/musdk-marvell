/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#define log_fmt(fmt) "gie: " fmt

#include "std_internal.h"
#include "env/trace/trc_pf.h"
#include "drivers/mqa_def.h"
#include "drivers/mv_dmax2.h"

#include "gie_int.h"


static struct gie_q_pair *gie_find_q_pair(struct gie *gie, u16 qid)
{
	int p, q;

	for (p = 0; p < GIE_MAX_PRIOS; p++) {
		for (q = 0; q < GIE_MAX_Q_PER_PRIO; q++) {
			if ((gie->prios[p].qpairs[q].flags & GIE_QPAIR_VALID) &&
				((gie->prios[p].qpairs[q].src_q.qid == qid) ||
				(gie->prios[p].qpairs[q].dst_q.qid == qid)))
				return &gie->prios[p].qpairs[q];
		}
	}

	return NULL;
}

static struct gie_bpool *gie_find_bpool(struct gie_bpool *pools, int max, u16 qid)
{
	int i;

	for (i = 0; i < max; i++) {
		if ((pools[i].buf_size != 0) && (pools[i].src_q.qid == qid))
			return &pools[i];
	}
	return NULL;
}

static void gie_show_q_indices(struct gie_queue *q)
{
	pr_debug("-------------\n");
	pr_debug("tail:      %d\n", q->tail);
	pr_debug("qe_tail:   %d\n", q->qe_tail);
	pr_debug("qe_head:   %d\n", q->qe_head);
	pr_debug("head:      %d\n", q->head);
	pr_debug("-------------\n");
}

static void gie_show_queue(struct gie_queue *q)
{
	/* To prevent compilation warning in non-debug mode. */
	q = q;

	pr_debug("queue details\n");
	pr_debug("-------------\n");
	pr_debug("qid:		%d\n", q->qid);
	pr_debug("priority:		%d\n", q->prio);
	pr_debug("ring_phys:		%p\n", (void *)q->ring_phys);
	pr_debug("ring_virt:		%p\n", (void *)q->ring_virt);
	pr_debug("msg_head_phys:	%p\n", (void *)q->msg_head_phys);
	pr_debug("msg_tail_phys:	%p\n", (void *)q->msg_tail_phys);
	pr_debug("msg_head_virt:	%p\n", (void *)q->msg_head_virt);
	pr_debug("msg_tail_virt:	%p\n", (void *)q->msg_tail_virt);
	pr_debug("host_remap:		%p\n", (void *)q->host_remap);
	pr_debug("qlen:			0x%x\n", q->qlen);
	pr_debug("qesize:		0x%x\n", q->qesize);
}

static int gie_init_queue(struct gie *gie, struct gie_queue *q, struct mqa_queue *mqa, int is_remote, u16 qid)
{
	q->ring_phys = mqa->ring_phy_addr;
	q->ring_virt = mqa->ring_virt_addr;
	q->msg_head_phys = mqa->cons_phys;
	q->msg_tail_phys = mqa->prod_phys;
	q->msg_head_virt = mqa->cons_virt;
	q->msg_tail_virt = mqa->prod_virt;
	q->host_remap = mqa->host_remap;
	q->qlen = mqa->ring_size;
	q->qesize = mqa->entry_size;
	q->prio = mqa->queue_prio;
	q->packets = 0;
	q->msi_virt_addr = 0;
	q->msi_data = 0;
	q->qid = qid;

	q->idx_ring_virt = mv_sys_dma_mem_alloc(sizeof(u16) * q->qlen, sizeof(u16));
	if (!q->idx_ring_virt)
		return -ENOMEM;

	q->idx_ring_phys = (u16 *)(uintptr_t)mv_sys_dma_mem_virt2phys(q->idx_ring_virt);
	q->idx_ring_size = q->qlen;
	q->idx_ring_ptr = 0;

	q->qe_tail = q->qe_head = 0;
	q->tail = q->head = 0;

	/* remap the ring phys address once */
	q->ring_phys += q->host_remap;

	/* set the offset to important parts in the desc
	 * This allows to generalize the handling of rx & tx queues.
	 * use the remote flag to distinguish rx/tx
	 * remote == 1 - equals host tx queue
	 * remote == 0 - equals local rx queue
	 */
	if (is_remote) {
		q->desc_len_pos = offsetof(struct host_tx_desc, byte_cnt);
		q->desc_addr_pos = offsetof(struct host_tx_desc, buffer_addr);
		q->desc_cookie_pos = offsetof(struct host_tx_desc, cookie);
		q->desc_num_sg_ent_pos = offsetof(struct host_tx_desc, num_sg_ent);
		q->desc_format_mask = HOST_TXD_FORMAT_MASK;
		q->desc_format_offset = HOST_TXD_FORMAT_SHIFT;
	} else {
		q->desc_len_pos = offsetof(struct host_rx_desc, byte_cnt);
		q->desc_addr_pos = offsetof(struct host_rx_desc, buffer_addr);
		q->desc_cookie_pos = offsetof(struct host_rx_desc, cookie);
		q->desc_num_sg_ent_pos = offsetof(struct host_rx_desc, num_sg_ent);
		q->desc_format_mask = HOST_RXD_FORMAT_MASK;
		q->desc_format_offset = HOST_RXD_FORMAT_SHIFT;
	}

	gie_show_queue(q);

	return 0;
}

static int gie_alloc_bpool_shadow(struct gie *gie,  struct gie_bpool *pool)
{
	struct gie_queue *shadow = &pool->shadow;
	struct gie_queue *src_q = &pool->src_q;

	shadow->ring_virt = (u64)mv_sys_dma_mem_alloc(src_q->qesize * src_q->qlen, src_q->qesize);
	if (!shadow->ring_virt)
		return -ENOMEM;

	shadow->ring_phys = mv_sys_dma_mem_virt2phys((void *)shadow->ring_virt);
	shadow->qesize = src_q->qesize;
	shadow->qlen = src_q->qlen;
	shadow->qe_tail = shadow->qe_head = 0;
	shadow->tail = shadow->head = 0;

	/* set a fake qid for the shadow queue, so we can identify it */
	shadow->qid = UINT16_MAX - src_q->qid;

	pr_debug("allocated bpool shadow at phys %p virt %p\n", (void *)shadow->ring_phys, (void *)shadow->ring_virt);

	return 0;
}

static void gie_dealloc_bpool_shadow(struct gie_bpool *pool)
{
	struct gie_queue *shadow = &pool->shadow;

	if (shadow->ring_virt)
		mv_sys_dma_mem_free((void *)shadow->ring_virt);
}

/* Find the next queue to service according to the
 * scheduling algorithm and queue counters
 * We currently implement simple round robin on all Qs of all Priorities
 */
static struct gie_q_pair *gie_get_next_q(struct gie *gie, u16 *scanned_prios, u16 *scanned_qs)
{
	struct gie_prio		*prio;
	struct gie_q_pair	*qp;
	u16			 i = *scanned_prios;
	u16			 j = *scanned_qs;

	/* Iterate all the priorities and look for the next to serve */
	while (i < GIE_MAX_PRIOS) {
		prio = &gie->prios[gie->curr_prio];
		/* Iterate all the queues in this priority and look for the next to serve */
		while (j < prio->q_cnt) {
			/* If priority is not active, skip it */
			if (!(prio->flags & GIE_PRIO_ACTIVE))
				break;
			qp = &prio->qpairs[prio->curr_q];
			prio->curr_q++;
			if (prio->curr_q >= prio->q_cnt)
				prio->curr_q = 0;
			j++;
			/* If we found an active queue within the priority, return it */
			if (qp->flags & GIE_QPAIR_ACTIVE) {
				/* save how many prios/qs we scanned so next time we'll know when to bail out */
				*scanned_prios = i;
				*scanned_qs = j;
				/* Move to next priority for next round */
				/* TODO: this will create simple round robin between prios. in future, add here
				 * a mechanism for WRR/SRR/WFQ/etc.
				 */
				gie->curr_prio++;
				if (gie->curr_prio == GIE_MAX_PRIOS)
					gie->curr_prio = 0;
				return qp;
			}
		}
		/* we scanned all queues in this priority; try next one */
		j = 0;
		gie->curr_prio++;
		if (gie->curr_prio == GIE_MAX_PRIOS)
			gie->curr_prio = 0;
		i++;
	}
	/* scanned all prios, set prio to 0 */
	if (i == GIE_MAX_PRIOS)
		i = 0;
	/* save how many prios/qs we scanned so next time we'll know when to bail out */
	*scanned_prios = i;
	*scanned_qs = j;

	/* no more active queues */
	return NULL;
}

static inline void gie_clean_mem(struct dma_job_info *job, struct gie_queue *src_q,
			struct gie_queue *dst_q)
{
	writel(job->cookie_head, (void *)(src_q->msg_head_virt));
	writel(job->cookie_tail, (void *)(dst_q->msg_tail_virt));

	/* Send MSI to remote side (if configured) */
	/* if the transaction was indices, this is local-2-remote;
	 * need to send MSI to dest-Q.
	 */
	if ((job->flags & DMA_FLAGS_PRODUCE_IDX) && (dst_q->msi_virt_addr))
		writel(dst_q->msi_data, (void *)dst_q->msi_virt_addr);
	/* if the transaction was buffers, this is remote-2-local-2;
	 * need to send MSI to source-Q.
	 */
	else if ((job->flags & DMA_FLAGS_BUFFER_IDX) && (src_q->msi_virt_addr))
		writel(src_q->msi_data, (void *)src_q->msi_virt_addr);
}

static void gie_clean_dma_jobs(struct dma_info *dma)
{
#ifdef GIE_VERIFY_DMA_OP
	struct dmax2_trans_complete_desc	 dmax2_res_descs[GIE_MAX_QES_IN_BATCH];
#endif /* GIE_VERIFY_DMA_OP */
	struct dma_job_info			*jobi;
	u16					 completed, clean, elements;
	int					 i;

#ifdef GIE_VERIFY_DMA_OP
	completed = GIE_MAX_QES_IN_BATCH;
	dmax2_deq(dma->dmax2, dmax2_res_descs, &completed, 1);
#else
	completed = dmax2_get_deq_num_available(dma->dmax2);
#endif /* GIE_VERIFY_DMA_OP */
	if (!completed)
		return;
	/* TODO: iterate all result-descriptor and verify no errors occurred */

	i = dma->head;
	/* barrier here to ensure following references to job-head */
	rmb();
	clean = completed;
	while (clean) {
		/* a single job can clean multiple descriptors, up-to the amounts dequed from DMA.
		 * if the job includes more descriptors, they are left for cleanup next time
		 */
		jobi = dma->dma_jobs + i;
		elements = min_t(u16, clean, jobi->desc_count);
		clean -= elements;
		jobi->desc_count -= elements;

		/* do not advance job_info queue if desc are left in the job */
		if (jobi->desc_count == 0) {
			if (jobi->flags & DMA_FLAGS_BUFFER_IDX) {
				struct gie_queue *src_q, *dst_q;

				src_q = (struct gie_queue *)jobi->cookie;
				dst_q = (struct gie_queue *)jobi->cookie_dst;
				gie_clean_mem(jobi, src_q, dst_q);
			}
			q_idx_add(i, 1, dma->qlen);
		}

		if (jobi->flags & DMA_FLAGS_UPDATE_IDX) {
			struct gie_queue *src_q;

			src_q = (struct gie_queue *)jobi->cookie;
			q_idx_add(src_q->qe_head, elements * jobi->elements_per_desc, src_q->qlen);
		}

		if (jobi->flags & DMA_FLAGS_PRODUCE_IDX) {
			struct gie_queue *src_q, *dst_q;

			src_q = (struct gie_queue *)jobi->cookie;
			dst_q = (struct gie_queue *)jobi->cookie_dst;
			/* Update the prod/cons index by dma */
			/* We cannot allow this copy-indexes to fail, so we will try to
			 * cleanup the DMA queue in case of failure.
			 */
			gie_clean_mem(jobi, src_q, dst_q);
		}
	}

	dma->head = i;
#ifndef GIE_VERIFY_DMA_OP
	dmax2_deq(dma->dmax2, NULL, &completed, 0);
#endif /* !GIE_VERIFY_DMA_OP */
}

/* create a dma copy job using a single dma transaction */
static int gie_dma_copy_single(struct dma_info *dma, struct dma_job *job)
{
	struct dma_job_info *job_info;
	struct dmax2_desc desc;
	int timeout = 100;
	u16 cnt;

	/* log the job in the dma job queue */
	job_info = dma->dma_jobs + dma->tail;

	job_info->cookie = job->cookie;
	job_info->cookie_dst = job->cookie_dst;
	job_info->cookie_head = job->cookie_head;
	job_info->cookie_tail = job->cookie_tail;
	job_info->elements_per_desc = job->element_cnt;
	job_info->desc_count = 1;
	job_info->flags = job->flags;

	/* create the dma job to submit */
#ifdef GIE_VERIFY_DMA_OP
	desc.flags = DESC_FLAGS_SYNC;
#else
	desc.flags = 0;
#endif /* GIE_VERIFY_DMA_OP */
	desc.desc_ctrl = DESC_OP_MODE_MEMCPY << DESC_OP_MODE_SHIFT;
	desc.src_addr = job->src;
	desc.dst_addr = job->dst;
	desc.buff_size = job->element_size * job->element_cnt;

	tracepoint(gie, dma, (void *)job->src, (void *)job->dst, desc.buff_size);

	/* Copy single is used for copying descriptors, queue pointers
	 * (indexes), and triggering interrupts.
	 * We cannot allow these to fail, so we will try to cleanup the DMA
	 * queue in case of failure.
	 */
	do {
		cnt = 1;
		dmax2_enq(dma->dmax2, &desc, &cnt);
		if (cnt == 0) {
			/* Try to cleanup the DMA queue. */
			gie_clean_dma_jobs(dma);
			timeout--;
			usleep(1);
		}
	} while ((!cnt) && (timeout));

	if (!timeout) {
		/* Not much to do in case of failure, something went wrong. */
		pr_err("BUG: Could not cleanup DMA after copy_single failure.");
		exit(1);
	}

	q_idx_add(dma->tail, 1, dma->qlen);

	return 0;
}

/* create a backup of the tail/head idx to be used for DMA copy
 * this allows us to continue changing the main tail/head during DMA operation
 */
static u64 gie_idx_backup(struct gie_queue *q, u16 idx_val)
{
	u64 phys_bkp;

	q->idx_ring_virt[q->idx_ring_ptr] = idx_val;
	phys_bkp = (u64)(q->idx_ring_phys + q->idx_ring_ptr);

	q_idx_add(q->idx_ring_ptr, 1, q->idx_ring_size);

	return phys_bkp;
}

static void gie_copy_index(struct dma_info *dma, u64 src, u64 dst)
{
	struct dma_job job;

	job.cookie = NULL;
	job.cookie_dst = 0;
	job.cookie_head = 0;
	job.cookie_tail = 0;
	job.element_size = sizeof(u16);
#ifdef GIE_VERIFY_DMA_OP
	job.flags = DESC_FLAGS_SYNC;
#else
	job.flags = 0;
#endif /* GIE_VERIFY_DMA_OP */

	job.src = src;
	job.dst = dst;
	job.element_cnt = 1;
	gie_dma_copy_single(dma, &job);
}

static void gie_copy_qes(struct dma_info *dma, struct gie_queue *src_q, struct gie_queue *dst_q,
				int qes_to_copy, u16 first_qe, u32 flags)
{
	struct dma_job job;
	int qesize = src_q->qesize;
	u32 first_batch, second_batch;

	/* check if this copy wraps */
	if (first_qe + qes_to_copy > src_q->qlen) {
		first_batch = src_q->qlen - first_qe;
		second_batch = qes_to_copy - first_batch;
	} else {
		first_batch = qes_to_copy;
		second_batch = 0;
	}

	job.cookie = src_q;
	job.cookie_dst = (uintptr_t)dst_q;
	job.cookie_head = 0;
	job.cookie_tail = 0;
	job.element_size = qesize;
	job.flags = flags;

	if ((flags & DMA_FLAGS_PRODUCE_IDX) && second_batch)
		job.flags &= ~DMA_FLAGS_PRODUCE_IDX;

	job.src = src_q->ring_phys + first_qe * qesize;
	job.dst = dst_q->ring_phys + first_qe * qesize;
	job.element_cnt = first_batch;

	if (flags & DMA_FLAGS_PRODUCE_IDX) {
		q_idx_add(src_q->head, qes_to_copy, src_q->qlen);
		q_idx_add(dst_q->tail, qes_to_copy, dst_q->qlen);
		job.cookie_head = src_q->head;
		job.cookie_tail = dst_q->tail;
		src_q->packets += qes_to_copy;
		dst_q->packets += qes_to_copy;
	}

	gie_dma_copy_single(dma, &job);

	if (second_batch) {
		if (flags & DMA_FLAGS_PRODUCE_IDX)
			job.flags |= DMA_FLAGS_PRODUCE_IDX;
		job.src = src_q->ring_phys;
		job.dst = dst_q->ring_phys;
		job.element_cnt = second_batch;

		gie_dma_copy_single(dma, &job);
	}

	tracepoint(gie, queue, "QE copy", qes_to_copy, src_q->qid, first_qe, src_q->tail,
		   dst_q->qid, dst_q->head, dst_q->tail);
}

static void gie_bpool_fill_shadow(struct dma_info *dma, struct gie_bpool *pool)
{
	struct gie_queue *src_q = &pool->src_q;
	struct gie_queue *shadow_q = &pool->shadow;
	int fill_size = GIE_SHADOW_FILL_SIZE;
	int src_q_fill;
	u32 qes_copied;

	/* did we already submit a copy ? if yes, wait till it's done */
	if (qes_in_copy(src_q))
		return;

	qes_copied = qes_copied(src_q);
	if (qes_copied) {
		/* previous copy completed, update pointers */
		q_idx_add(shadow_q->tail, qes_copied, shadow_q->qlen);
		q_idx_add(src_q->head, qes_copied, src_q->qlen);
		writel(src_q->head, (void *)(src_q->msg_head_virt));
		return;
	}

	/* okay, let's submit a copy */
	src_q->tail = readl((void *)(src_q->msg_tail_virt));
	src_q_fill = q_occupancy(src_q);
	if (!src_q_fill)
		/* TODO: Add a trancepoint. */
		return;

	/* Check the size available at the source queue.
	 * No need to check the space of the local queue since
	 * if we got here, there is space
	 */
	fill_size = min(fill_size, src_q_fill);

	/* submit a background copy of QEs */
	gie_copy_qes(dma, src_q, shadow_q, fill_size, src_q->qe_tail, DMA_FLAGS_UPDATE_IDX);
	q_idx_add(src_q->qe_tail, fill_size, src_q->qlen);
}

static int gie_get_rem_bpool_bufs(struct dma_info *dma, struct gie_q_pair *qp, int buf_cnt)
{
	struct gie_queue *bpool_q;
	int bufs_avail;

	bpool_q = &qp->dst_bpools[0]->shadow;

	bufs_avail = q_occupancy(bpool_q);
	/* in case we don't have enough buffers in the local shadow BP, we need
	 * to fill it from the remote BP
	 */
	if ((bufs_avail < GIE_SHADOW_FILL_THRESH) || (bufs_avail < buf_cnt))
		gie_bpool_fill_shadow(dma, qp->dst_bpools[0]);

	if (unlikely(!bufs_avail))
		return 0;

	if (unlikely(buf_cnt > bufs_avail))
		/* TODO: Add a trancepoint. */
		buf_cnt = bufs_avail;

	return buf_cnt;
}

static inline struct host_bpool_desc *gie_get_shadow_bpool_buf(struct gie_q_pair *qp)
{
	struct gie_queue	*bpool_q;
	struct host_bpool_desc	*bp_buf;

	bpool_q = &qp->dst_bpools[0]->shadow;

	if (unlikely(!q_occupancy(bpool_q)))
		return NULL;

	bp_buf = (struct host_bpool_desc *)bpool_q->ring_virt + bpool_q->head;
	q_idx_add(bpool_q->head, 1, bpool_q->qlen);

	return bp_buf;
}

static inline u32 gie_get_shadow_bpool_occupancy(struct gie_q_pair *qp)
{
	struct gie_queue *bpool_q = &qp->dst_bpools[0]->shadow;

	return q_occupancy(bpool_q);
}

static inline int gie_copy_interim_qes(struct gie_queue		*src_q,
					struct gie_queue	*dst_q,
					struct gie_q_pair	*qp,
					struct gie_queue	*qes_q,
					int			 qes_to_copy,
					struct dmax2_desc	*descs,
					int			 sg_en)
{
	struct host_bpool_desc	*bp_buf = NULL;
	void	*qe;
	u64	*qe_cookie, *qe_buff;
	u64	 src_remap, dst_remap;
	u16	*qe_byte_cnt;
	u16	 addr_pos, cookie_pos, len_pos;
	u16	 cnt = 0;
	int	 i = 0;

	src_remap = src_q->host_remap;
	dst_remap = dst_q->host_remap;
	addr_pos = src_q->desc_addr_pos;
	cookie_pos = src_q->desc_cookie_pos;
	len_pos = src_q->desc_len_pos;

	i = src_q->head;
	/* barrier here before we read the QEs to make sure they're in memory */
	rmb();
	while (qes_to_copy--) {
		qe = (void *)qes_q->ring_virt + i * qes_q->qesize;
		qe_buff = qe + addr_pos;
		qe_cookie = qe + cookie_pos;
		qe_byte_cnt = qe + len_pos;

		if (sg_en) {
			u8 format = ((*(u32 *)qe) & src_q->desc_format_mask) >> src_q->desc_format_offset;

			if (format == HOST_FORMAT_DIRECT_SG) {
				u8 num_sg_ent = (*(u8 *)(qe + src_q->desc_num_sg_ent_pos) & HOST_NUM_SG_ENT_MASK) + 2;

				if ((num_sg_ent - 1) > qes_to_copy)
					/* Not enoght buffers for all s/g entries. will be handled next time */
					break;

				if (gie_get_shadow_bpool_occupancy(qp) < num_sg_ent)
					/* Not enoght buffers for all s/g entries. will be handled next time */
					break;
			}
		}

		bp_buf = gie_get_shadow_bpool_buf(qp);
		if (bp_buf == NULL)
			break;

		/* create the dma job to submit */
		descs[cnt].flags = 0;
		descs[cnt].desc_ctrl = DESC_OP_MODE_MEMCPY << DESC_OP_MODE_SHIFT;
		descs[cnt].src_addr = src_remap + *qe_buff;
		descs[cnt].dst_addr = dst_remap + bp_buf->buff_addr_phys;
		descs[cnt].buff_size = *qe_byte_cnt;

		tracepoint(gie, dma, (void *)descs[cnt].src_addr, (void *)descs[cnt].dst_addr, descs[cnt].buff_size);
		cnt++;

		/* update qe buffer with host buffer*/
		*qe_buff = bp_buf->buff_addr_phys;
		*qe_cookie = bp_buf->buff_cookie;
		q_idx_add(i, 1, src_q->qlen);
	}
	/* barrier here after we access the QEs to make sure all above are written before the next dma trans */
	wmb();

	return cnt;
}

static int gie_copy_buffers_l2r(struct dma_info *dma, struct gie_q_pair *qp, struct gie_queue *src_q,
			     struct gie_queue *dst_q, struct gie_queue *qes_q, int bufs_to_copy, int sg_en)
{
	struct dma_job_info	*job_info;
	struct dmax2_desc	 descs[GIE_MAX_QES_IN_BATCH];
	u16			 cnt = 0;

	/* fetch buffers from the bpool. the returned pointer points directly to the
	 * bpool queue elements to avoid copying them. therefore, when the queue is
	 * about the wrap, we only get the last items in the queue, so we dont need to
	 * handle the wrap-around in the loop. That's why we may call the function
	 * multiple times, first to get the last n elements in the queue, and then
	 * to get bufs_to_copy-n elements.
	 */
	bufs_to_copy = gie_get_rem_bpool_bufs(dma, qp, bufs_to_copy);
	if (unlikely(!bufs_to_copy))
		return 0;

	cnt = gie_copy_interim_qes(src_q, dst_q, qp, qes_q, bufs_to_copy, descs, sg_en);

	/* we can reach here after 1 pass or no pass at all */
	if (cnt) {
		u16 tmp_cnt, tmp_left;
		int timeout = 1000;

		/* log the job in the dma job queue */
		job_info = dma->dma_jobs + dma->tail;

		job_info->cookie = src_q;
		job_info->flags = 0;
		job_info->elements_per_desc = 1;
		job_info->desc_count = cnt;

		tmp_cnt = tmp_left = cnt;
		do {
			dmax2_enq(dma->dmax2, descs, &tmp_cnt);
			if (tmp_cnt == 0)
				gie_clean_dma_jobs(dma);
			tmp_left -= tmp_cnt;
			tmp_cnt = tmp_left;
			if (!tmp_cnt)
				break;
			timeout--;
			usleep(1);
		} while (timeout);

		if (!timeout) {
			/* Not much to do in case of failure, something went wrong. */
			pr_err("BUG: Could not cleanup DMA after copy_single failure.");
			exit(1);
		}

		q_idx_add(dma->tail, 1, dma->qlen);

		tracepoint(gie, queue, "buffer copy", cnt, src_q->qid, src_q->head, src_q->tail,
			   dst_q->qid, dst_q->head, dst_q->tail);
	}

	return cnt;
}

static inline int gie_get_lcl_bpool_buf(struct gie_q_pair *qp, u32 min_buf_size,
			       struct host_bpool_desc **bp_buf)
{
	struct gie_queue	*bpool_q;
	int			 i;

	/* Find the bpool that matches the min buffer size */
	for (i = 0; i < GIE_MAX_BPOOLS; i++)
		if (min_buf_size <= qp->dst_bpools[i]->buf_size)
			break;

	if (unlikely(i == GIE_MAX_BPOOLS)) {
		pr_err("Failed to find bpool for buffer size %d\n", min_buf_size);
		return 0;
	}

	/* For remote queues, fill the shadow. For locals, update the tail pointer */
	bpool_q = &qp->dst_bpools[i]->src_q;
	bpool_q->tail = readl((void *)(bpool_q->msg_tail_virt));

	if (unlikely(!q_occupancy(bpool_q)))
		return 0;

	/* increment an internal index to indicate these buffers are already used we still
	 * don't update the remote head until the user is done with the buffers to avoid override
	 */
	*bp_buf = (struct host_bpool_desc *)bpool_q->ring_virt + bpool_q->head;
	q_idx_add(bpool_q->head, 1, bpool_q->qlen);
	writel(bpool_q->head, (void *)(bpool_q->msg_head_virt));

	return 1;
}

static inline u32 gie_get_lcl_bpool_occupancy(struct gie_q_pair *qp, u8 pool_id)
{
	struct gie_queue *bpool_q;

	bpool_q = &qp->dst_bpools[pool_id]->src_q;
	bpool_q->tail = readl((void *)(bpool_q->msg_tail_virt));

	return q_occupancy(bpool_q);
}

static int gie_copy_buffers_r2l(struct dma_info *dma, struct gie_q_pair *qp, struct gie_queue *src_q,
			     struct gie_queue *dst_q, struct gie_queue *qes_q, int bufs_to_copy, int sg_en)
{
	struct dmax2_desc	 desc[GIE_MAX_QES_IN_BATCH];
	struct host_bpool_desc	*bp_buf;
	void			*qe;
	u64			*qe_cookie, *qe_buff;
	u16			*qe_byte_cnt;
	u64			 src_remap, dst_remap;
	u16			 addr_pos, cookie_pos, len_pos, cnt = 0;
	int			 i = 0;

	src_remap = src_q->host_remap;
	dst_remap = dst_q->host_remap;
	addr_pos = src_q->desc_addr_pos;
	cookie_pos = src_q->desc_cookie_pos;
	len_pos = src_q->desc_len_pos;

	i = src_q->head;
	/* barrier here before we read the QEs to make sure they're in memory */
	rmb();
	while (bufs_to_copy--) {
		qe = (void *)qes_q->ring_virt + i * qes_q->qesize;
		qe_buff = qe + addr_pos;
		qe_cookie = qe + cookie_pos;
		qe_byte_cnt = qe + len_pos;

		if (sg_en) {
			u8 format = ((*(u32 *)qe) & src_q->desc_format_mask) >> src_q->desc_format_offset;

			if (format == HOST_FORMAT_DIRECT_SG) {
				u8 num_sg_ent = (*(u8 *)(qe + src_q->desc_num_sg_ent_pos) & HOST_NUM_SG_ENT_MASK) + 2;

				if ((num_sg_ent - 1) > bufs_to_copy)
					/* Not all S/G entries were copied. will be handled next time */
					break;

				if (gie_get_lcl_bpool_occupancy(qp, 0) < num_sg_ent)
					/* Not enoght buffers for all s/g entries. will be handled next time */
					break;
			}
		}

		/* TODO: take dst-Q pkt-offset into acount here! */
		if (!gie_get_lcl_bpool_buf(qp, *qe_byte_cnt, &bp_buf))
			break;

		/* create the dma job to submit */
		desc[cnt].flags = 0;
		desc[cnt].desc_ctrl = DESC_OP_MODE_MEMCPY << DESC_OP_MODE_SHIFT;
		/* TODO: take src-Q pkt-offset into acount here! */
		desc[cnt].src_addr = src_remap + *qe_buff;
		/* TODO: take dst-Q pkt-offset into acount here! */
		desc[cnt].dst_addr = dst_remap + bp_buf->buff_addr_phys;
		desc[cnt].buff_size = *qe_byte_cnt;

		tracepoint(gie, dma, (void *)desc[cnt].src_addr, (void *)desc[cnt].dst_addr, desc[cnt].buff_size);
		cnt++;

		/* update qe buffer with host buffer*/
		*qe_buff = bp_buf->buff_addr_phys;
		*qe_cookie = bp_buf->buff_cookie;
		q_idx_add(i, 1, src_q->qlen);
	}
	/* barrier here after we access the QEs to make sure all above are written before the next dma trans */
	wmb();

	/* we can reach here after 1 pass or no pass at all */
	if (cnt) {
		struct dma_job_info	*job_info;
		u16			 tmp_cnt, tmp_left;
		int			 timeout = 1000;

		/* log the job in the dma job queue */
		job_info = dma->dma_jobs + dma->tail;

		job_info->cookie = src_q;
		job_info->cookie_dst = (uintptr_t)dst_q;
		q_idx_add(src_q->head, cnt, src_q->qlen);
		q_idx_add(dst_q->tail, cnt, dst_q->qlen);
		job_info->cookie_head = src_q->head;
		job_info->cookie_tail = dst_q->tail;
		src_q->packets += cnt;
		dst_q->packets += cnt;
		job_info->flags = DMA_FLAGS_BUFFER_IDX;
		job_info->elements_per_desc = 1;
		job_info->desc_count = cnt;

		tmp_cnt = tmp_left = cnt;
		do {
			dmax2_enq(dma->dmax2, desc, &tmp_cnt);
			if (tmp_cnt == 0)
				gie_clean_dma_jobs(dma);
			tmp_left -= tmp_cnt;
			tmp_cnt = tmp_left;
			if (!tmp_cnt)
				break;
			timeout--;
			usleep(1);
		} while (timeout);

		if (!timeout) {
			/* Not much to do in case of failure, something went wrong. */
			pr_err("BUG: Could not cleanup DMA after copy_single failure.");
			exit(1);
		}

		q_idx_add(dma->tail, 1, dma->qlen);

		tracepoint(gie, queue, "buffer copy", cnt, src_q->qid, src_q->head, src_q->tail,
			   dst_q->qid, dst_q->head, dst_q->tail);
	}

	return cnt;
}

static void gie_produce_q(struct dma_info *dma, struct gie_queue *src_q, struct gie_queue *dst_q, int qes_completed)
{
	q_idx_add(src_q->head, qes_completed, src_q->qlen);
	q_idx_add(dst_q->tail, qes_completed, dst_q->qlen);
	writel(src_q->head, (void *)(src_q->msg_head_virt));
	writel(dst_q->tail, (void *)(dst_q->msg_tail_virt));
	src_q->packets += qes_completed;
	dst_q->packets += qes_completed;

	tracepoint(gie, queue, "QE produce", qes_completed, src_q->qid, src_q->head, src_q->tail,
		   dst_q->qid, dst_q->head, dst_q->tail);
}

static int gie_clip_batch(struct gie_queue *dst_q, int required_copy)
{
	int dst_space = q_space(dst_q);

	/* clip to maximum batch size */
	required_copy = min(required_copy, GIE_MAX_QES_IN_BATCH);

	/* clip to destination size */
	if (required_copy > dst_space)
		/* TODO: Add a trancepoint. */
		required_copy = dst_space;

	return required_copy;
}

static int gie_process_remote_q(struct dma_info *dma, struct gie_q_pair *qp, int qe_limit)
{
	struct gie_queue *src_q = &qp->src_q;
	struct gie_queue *dst_q = &qp->dst_q;
	int copy_payload = qp->flags & GIE_QPAIR_CP_PAYLOAD;
	int sg_en = qp->flags & GIE_QPAIR_SG;
	int qes_copied, qes_to_copy, qes_copy_space;
	int completed = 0;

	/* TODO - consider this limitation in the future */
	(void)qe_limit;

	/* Get the updated tail & head from the notification area */
	src_q->tail = readl((void *)(src_q->msg_tail_virt));
	dst_q->head = readl((void *)(dst_q->msg_head_virt));

	/* check for any pending work (qes, buffers, etc) */
	if (q_empty(src_q))
		return 0;

	/* First phase - Copy the QEs to destination queue */
	qes_to_copy = qes_to_copy(src_q);
	qes_copy_space = qes_copy_space(dst_q);
	qes_to_copy = min(qes_to_copy, qes_copy_space);
	if (qes_to_copy) {
		qes_to_copy = gie_clip_batch(dst_q, qes_to_copy);
		gie_copy_qes(dma, src_q, dst_q, qes_to_copy, src_q->qe_tail, DMA_FLAGS_UPDATE_IDX);
		q_idx_add(src_q->qe_tail, qes_to_copy, src_q->qlen);
		q_idx_add(dst_q->qe_tail, qes_to_copy, dst_q->qlen);
	}

	/* Second phase - copy the buffers and update prod/cons index */
	qes_copied = qes_copied(src_q);
	if (qes_copied) {
		qes_copied = gie_clip_batch(dst_q, qes_copied);
		if (copy_payload)
			completed = gie_copy_buffers_r2l(dma, qp, src_q, dst_q, dst_q, qes_copied, sg_en);
		else
			completed = qes_copied;
	}

	/* Last phase - update the remote indices to indicate production/consumption */
	if ((completed) && (!copy_payload))
		gie_produce_q(dma, src_q, dst_q, completed);

	return completed;
}

static int gie_process_local_q(struct dma_info *dma, struct gie_q_pair *qp, int qe_limit)
{
	struct gie_queue *src_q = &qp->src_q;
	struct gie_queue *dst_q = &qp->dst_q;
	int copy_payload = qp->flags & GIE_QPAIR_CP_PAYLOAD;
	int sg_en = qp->flags & GIE_QPAIR_SG;
	int qes;

	/* TODO - consider this limitation in the future */
	(void)qe_limit;

	/* Get the updated tail & head from the notification area */
	src_q->tail = readl((void *)(src_q->msg_tail_virt));
	dst_q->head = readl((void *)(dst_q->msg_head_virt));

	/* check for any pending work (qes, buffers, etc) */
	if (q_empty(src_q))
		return 0;

	/* for data queues, first copy  the buffers.
	 * not all buffers might be copied due to lack of bpools
	 */
	qes = q_occupancy(src_q);
	qes = gie_clip_batch(dst_q, qes);

	if (copy_payload)
		qes = gie_copy_buffers_l2r(dma, qp, src_q, dst_q, src_q, qes, sg_en);

	/* if bpools are empty, no buffer copy will occur.
	 * If so, skip the QE copy as well
	 */
	if (!qes)
		return 0;

	/* Copy the QEs */
	gie_copy_qes(dma, src_q, dst_q, qes, src_q->head, DMA_FLAGS_PRODUCE_IDX);

	/* We'll update the prod/cons index only once both the buffers as well as the QEs were copied */
	/*gie_produce_q(dma, src_q, dst_q, qes);*/

	return qes;
}

static int gie_event_validate(void *driver_data)
{
	struct gie_event_data *event_data = driver_data;

	return event_data->enable;
}


/* Initialize the emulator. Basically the emulator only needs
 * the register base and the DMA engine from the caller.
 * The rest is initializations of local data structures
 */
int gie_init(struct gie_params *gie_params, struct gie **gie)
{
	struct dmax2_params dmax2_params;
	struct dma_info *dma;
	struct gie_regfile *gie_regs;
	int err = 0;

	pr_debug("Initializing %s-giu instance\n", gie_params->name_match);

	/* allocate GIU emulator handler */
	*gie = kcalloc(1, sizeof(struct gie), GFP_KERNEL);
	if (*gie == NULL)
		return -ENOMEM;

	/* allocate pseudo regfile for the GIU emulator */
	gie_regs = kcalloc(1, sizeof(struct gie_regfile), GFP_KERNEL);
	if (gie_regs == NULL) {
		kfree(*gie);
		return -ENOMEM;
	}

	strncpy((*gie)->name, gie_params->name_match, GIE_MAX_NAME);
	dma = &((*gie)->dma);

	/* Open a MUSDK DMAx2 channel */
	dmax2_params.match = gie_params->dmax_match;
	dmax2_params.queue_size = GIE_DMAX2_Q_SIZE;
	err = dmax2_init(&dmax2_params, &(dma->dmax2));
	if (err) {
		pr_err("Failed to initialize MUSDK dmax2\n");
		goto init_error;
	}

	err = dmax2_set_mem_attributes(dma->dmax2, DMAX2_TRANS_LOCATION_SRC_AND_DST, DMAX2_TRANS_MEM_ATTR_CACHABLE);
	if (err) {
		pr_err("Failed to set dmax2 attributes\n");
		goto attr_error;
	}

	gie_regs->gct_base	= gie_params->gct_base;
	gie_regs->gpt_base	= gie_params->gpt_base;
	/* Set MSI registers (for sending MSI messages) Phy/Virt addresses */
	gie_regs->msi_regs_phys	= gie_params->msi_regs_phys;
	gie_regs->msi_regs_virt	= gie_params->msi_regs_virt;

	(*gie)->regs = gie_regs;

	/* reset the dma job queue */
	dma->tail = dma->head = 0;
	dma->qlen = GIE_MAX_DMA_JOBS;

	return 0;

attr_error:
	dmax2_deinit(dma->dmax2);

init_error:
	kfree(*gie);
	kfree(gie_regs);

	return err;
}

/* Close the GIU emulator device.
 * At this point we assume that there is no active traffic.
 * We basically Close the MUSDK DMAx2 channel and free memory
 */
int gie_terminate(struct gie *gie)
{
	int ret;

	pr_debug("Terminating %s-giu instance\n", gie->name);

	/* TODO
	 * - check that all DMA transactions are closed
	 *   - remove all queues and free shadow bpools
	 */

	if (gie->dma.dmax2 == NULL) {
		pr_err("Invalid DMA handle in %s-giu\n", gie->name);
		return -ENODEV;
	}

	ret = dmax2_deinit(gie->dma.dmax2);
	if (ret)
		pr_warn("Failed to close MUSDK DMA channel\n");

	kfree(gie->regs);

	kfree(gie);
	return ret;
}

void *gie_regs(void *giu)
{
	return (void *)(((struct gie *)giu)->regs);
}

int gie_add_queue(void *giu, u16 qid, int is_remote)
{
	struct gie *gie = (struct gie *)giu;
	struct gie_q_pair *qp = NULL;
	struct mqa_qct_entry *qcd;
	struct mqa_qpt_entry *qpd;
	struct gie_bpool *bpool;
	int i, copy_payload, ret;
	u32 prio;

	pr_debug("adding qid %d to %s-giu\n", qid, gie->name);

	qcd = (struct mqa_qct_entry *)gie->regs->gct_base + qid;
	qpd = (struct mqa_qpt_entry *)gie->regs->gpt_base + qcd->spec.dest_queue_id;
	prio = qcd->common.queue_prio;
	copy_payload = qcd->common.flags & MQA_QFLAGS_COPY_BUF;

	/* Find an empty q_pair slot */
	for (i = 0; i < GIE_MAX_Q_PER_PRIO; i++) {
		if (!(gie->prios[prio].qpairs[i].flags & GIE_QPAIR_VALID)) {
			qp = &gie->prios[prio].qpairs[i];
			break;
		}
	}

	if (qp == NULL) {
		pr_err("No space to add queue %d\n", qid);
		return -ENOSPC;
	}

	ret = gie_init_queue(gie, &qp->src_q, &qcd->common, is_remote, qid);
	if (ret)
		return ret;
	qp->src_q.qid = qid;
	qp->qcd = qcd;

	ret = gie_init_queue(gie, &qp->dst_q, &qpd->common, is_remote, qcd->spec.dest_queue_id);
	if (ret)
		return ret;
	qp->dst_q.qid = qcd->spec.dest_queue_id;
	qp->qpd = qpd;

	/* Set MSI-X message info for dest Q of remote side */
	if (qpd->common.msix_inf.va) {
		/* Set message info */
		qp->dst_q.msi_virt_addr = (u64)qpd->common.msix_inf.va;
		qp->dst_q.msi_data = qpd->common.msix_inf.data;

		pr_debug("MSI-X for dst-Q %d: phys 0x%"PRIx64" virt 0x%lx data 0x%x\n",
			 qp->dst_q.qid, qpd->common.msix_inf.pa, qp->dst_q.msi_virt_addr, qp->dst_q.msi_data);
	}
	/* Set MSI-X message info for source Q of remote side */
	if (qcd->common.msix_inf.va) {
		/* Set message info */
		qp->src_q.msi_virt_addr = (u64)qcd->common.msix_inf.va;
		qp->src_q.msi_data = qcd->common.msix_inf.data;

		pr_debug("MSI-X for src-Q %d: phys 0x%"PRIx64" virt 0x%lx data 0x%x\n",
			 qp->src_q.qid, qcd->common.msix_inf.pa, qp->src_q.msi_virt_addr, qp->src_q.msi_data);
	}

	/* Find the bpools and keep local pointers */
	if (copy_payload) {
		for (i = 0; i < GIE_MAX_BM_PER_Q; i++) {
			bpool = gie_find_bpool(gie->bpools, gie->bp_cnt, qpd->common.queue_ext.bm_queue[i]);
			if (bpool == NULL) {
				pr_err("Failed to find bpool for destination queue %d\n", qid);
				return -ENODEV;
			}
			qp->dst_bpools[i] = bpool;
		}
	}

	qp->flags = GIE_QPAIR_VALID | GIE_QPAIR_ACTIVE;
	if (is_remote)
		qp->flags |= GIE_QPAIR_REMOTE;
	if (copy_payload)
		qp->flags |= GIE_QPAIR_CP_PAYLOAD;
	if (qcd->common.flags & MQA_QFLAGS_SG)
		qp->flags |= GIE_QPAIR_SG;
	gie->prios[prio].flags = GIE_PRIO_VALID | GIE_PRIO_ACTIVE;

	gie->prios[prio].q_cnt++;
	pr_debug("Added %s Q %d on GIE-%s\n", (is_remote?"rem":"lcl"), qp->src_q.qid, gie->name);
	pr_debug("Added %s Q %d on GIE-%s\n", (!is_remote?"rem":"lcl"), qp->dst_q.qid, gie->name);

	return 0;
}

/* Add a buffer pool queue to the GIE local structures.
 * The bpool will be associated with a produced queue and used
 * by the GIE to get destination buffers for payload copy
 */
int gie_add_bm_queue(void *giu, u16 qid, int buf_size, int is_remote)
{
	struct gie *gie = (struct gie *)giu;
	struct gie_bpool *pool = NULL;
	struct gie_queue *src_q;
	struct mqa_qct_entry *qcd;
	int ret, i;

	pr_debug("adding bpool qid %d of size %d to %s-giu\n", qid, buf_size, gie->name);

	if (buf_size == 0) {
		pr_err("invalid buf_size %d for bpool qid %d\n", buf_size, qid);
		return -EINVAL;
	}

	/* Find an empty bpool slot */
	for (i = 0; i < GIE_MAX_BPOOLS; i++) {
		if (gie->bpools[i].buf_size == 0) {
			pool = &gie->bpools[i];
			break;
		}
	}
	if (pool == NULL) {
		pr_warn("No space to add bpool queue %d\n", qid);
		return -ENOSPC;
	}

	src_q = &pool->src_q;
	qcd = (struct mqa_qct_entry *)gie->regs->gct_base + qid;

	ret = gie_init_queue(gie, src_q, &qcd->common, is_remote, qid);
	if (ret)
		return ret;

	src_q->qid = qid;
	src_q->qesize = sizeof(struct host_bpool_desc);
	pool->buf_size = buf_size;
	pool->flags = 0;

	/* Host queues cannot be accessed directly by the CPU, so
	 * we must copy them to our local memory first, using a shadow queue
	 */
	if (is_remote) {
		pool->flags |= GIE_BPOOL_REMOTE;
		ret = gie_alloc_bpool_shadow(gie, pool);
		if (ret) {
			pr_warn("Failed to allocate BM shadow queue\n");
			return ret;
		}
	}

	gie->bp_cnt++;
	pr_debug("Added BM-Q %d on GIE-%s\n", qid, gie->name);

	return 0;
}

int gie_remove_queue(void *giu, u16 qid)
{
	struct gie *gie = (struct gie *)giu;
	struct gie_q_pair *qp;

	qp = gie_find_q_pair(gie, qid);
	if (qp == NULL) {
		pr_warn("Cannot find queue %d to remove\n", qid);
		return -ENODEV;
	}
	/* we don't allow removing Q if it is not the SRC-Q! */
	if (qp->src_q.qid != qid) {
		pr_warn("Cannot find queue %d to remove (as src-q)!\n", qid);
		return -ENODEV;
	}

	gie->prios[qp->src_q.prio].q_cnt--;
	if (!gie->prios[qp->src_q.prio].q_cnt)
		gie->prios[qp->src_q.prio].flags = 0;
	qp->flags = 0;

	return 0;
}

int gie_remove_bm_queue(void *giu, u16 qid)
{
	struct gie *gie = (struct gie *)giu;
	struct gie_bpool *pool;

	pool = gie_find_bpool(gie->bpools, gie->bp_cnt, qid);
	if (pool == NULL) {
		pr_warn("Cannot find BM queue %d to remove\n", qid);
		return -ENODEV;
	}

	if (pool->flags & GIE_BPOOL_REMOTE)
		gie_dealloc_bpool_shadow(pool);

	pool->flags = 0;
	pool->buf_size = 0;
	gie->bp_cnt--;
	return 0;
}

int gie_schedule(void *giu, u64 time_limit, u64 qe_limit, u16 *pending)
{
	struct gie *gie = (struct gie *)giu;
	struct gie_q_pair *qp;
	int qes = 0;
	u16 scanned_prios = 0, scanned_qs = 0;

	if (qe_limit == 0)
		qe_limit = UINT64_MAX;
	if (time_limit == 0)
		time_limit = UINT64_MAX;

	tracepoint(gie, flow, "start scheduling", gie->name);

	gie_clean_dma_jobs(&gie->dma);

	while (qes < qe_limit) {
		/* TODO - consider time as stop condition */
		qp = gie_get_next_q(gie, &scanned_prios, &scanned_qs);
		if (qp == NULL)
			break;

		if (qp->flags & GIE_QPAIR_REMOTE)
			qes += gie_process_remote_q(&gie->dma, qp, qe_limit);
		else
			qes += gie_process_local_q(&gie->dma, qp, qe_limit);
	}

	if (pending)
		*pending = q_occupancy((&gie->dma));

	return qes;
}

int gie_get_desc_size(enum gie_desc_type type)
{
	switch (type) {
	case TX_DESC:
		return sizeof(struct host_tx_desc);
	case RX_DESC:
		return sizeof(struct host_rx_desc);
	case BUFF_DESC:
		return sizeof(struct host_bpool_desc);
	default:
		pr_err("Invalid gie desc typr %d\n", type);
	}

	return 0;
}

int gie_create_event(struct gie *gie, struct gie_event_params *params, struct mv_sys_event **ev)
{
	struct mv_sys_event_params ev_params = {0};
	struct gie_event_data *event_data;
	int err;
	u8 tc_num = 0;

	if (params->tc_mask != 0x01) {
		pr_err("(%s) Currently only TC0 is supported\n", __func__);
		return -EINVAL;
	}

	if (params->pkt_coal || params->usec_coal) {
		pr_err("(%s) Currently coalescing is not supported\n", __func__);
		return -EINVAL;
	}

	event_data = kcalloc(1, sizeof(struct gie_event_data), GFP_KERNEL);
	if (!event_data)
		return -ENOMEM;

	snprintf(ev_params.name, sizeof(ev_params.name), GIE_AGNIC_UIO_STRING, tc_num);
	ev_params.event_validate = gie_event_validate;
	ev_params.driver_data = event_data;

	err = mv_sys_event_create(&ev_params, ev);
	if (err) {
		pr_err("(%s) Can't open GIE event: %s\n", __func__, ev_params.name);
		kfree(event_data);
		return err;
	}
	(*ev)->events = MV_SYS_EVENT_POLLIN;

	memcpy(&event_data->ev_params, params, sizeof(*params));
	event_data->enable = 0;

	return 0;

}

int gie_delete_event(struct mv_sys_event *ev)
{
	int err;
	struct gie_event_data *event_data;

	err = mv_sys_event_get_driver_data(ev, (void **)&event_data);
	if (err) {
		pr_err("(%s) could not get event driver_data event_ptr(%p)\n", __func__, ev);
		return -EINVAL;
	}

	mv_sys_event_destroy(ev);
	kfree(event_data);

	return 0;
}

int gie_set_event(struct mv_sys_event *ev, int en)
{
	int err;
	struct gie_event_data *event_data;

	err = mv_sys_event_get_driver_data(ev, (void **)&event_data);
	if (err) {
		pr_err("(%s) could not get event driver_data event_ptr(%p)\n", __func__, ev);
		return -EINVAL;
	}

	event_data->enable = en;

	return 0;
}

int gie_get_queue_stats(void *giu, u16 qid, u64 *pkt_cnt, int reset)
{
	struct gie *gie = (struct gie *)giu;
	struct gie_q_pair *qp;

	qp = gie_find_q_pair(gie, qid);
	if (qp == NULL) {
		pr_err("Cannot find queue %d for stats\n", qid);
		return -ENODEV;
	}

	if (qp->src_q.qid == qid) {
		*pkt_cnt = qp->src_q.packets;
		if (reset)
			qp->src_q.packets = 0;
	} else {
		*pkt_cnt = qp->dst_q.packets;
		if (reset)
			qp->dst_q.packets = 0;
	}

	return 0;
}
