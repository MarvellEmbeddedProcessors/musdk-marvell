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

#define log_fmt(fmt) "gie: " fmt

#include "std_internal.h"
#include "hw_emul/gie.h"
#include "env/trace/trc_pf.h"
#include "drivers/mqa/mqa_internal.h"
#include "gie_buff_desc.h"

/* MUSDK includes */
#include "drivers/mv_dmax2.h"

#define GIE_MAX_NAME		256
#define GIE_MAX_DMA_JOBS	1024
#define GIE_MAX_QES_IN_BATCH	32

#define GIE_MAX_TCS		8
#define GIE_MAX_Q_PER_TC	128
#define GIE_MAX_BPOOLS		16
#define GIE_MAX_BM_PER_Q	1
#define GIE_SHADOW_FILL_SIZE	256
#define GIE_SHADOW_FILL_THRESH	128

#define GIE_MAX_QID		(UINT16_MAX + 1)

/* This structure describes a queue from the GIE perspective
 * it may be used for buffer pool queues and data queues alike
 * some of the members are copied from the QCE .This is done to
 * avoid accessing the QCEs in runtime which might cause thrashing.
 *
 * ring_phys		physical address of ring base
 * ring_virt		virtual address of ring base
 * msg_head_phys	physical address to read/write head updates to/from
 * msg_tail_phys	physical address to read/write tail updates to/from
 * msg_head_virt	virtual address to read/write head updates to/from
 * msg_tail_virt	virtual address to read/write tail updates to/from
 * host_remap		base address of the host memory in local memory space
 * packets		number of packets processed by this queue
 *
 * idx_ring_virt	the following four parameters describe the index ring.
 * idx_ring_phys	A ring that temporarily holds head/tail values until
 * idx_ring_size	the DMA engine copies them to the destination
 * idx_ring_ptr		_virt/_phys = address of ring. _ptr = location in ring
 *
 * tc			the traffic class associated with the ring
 * qlen			size of queue in elements
 * qesize		size of queue element
 * qid			the global queue id
 * tail			local tail value
 * head			local head value
 * qe_tail		next descriptor to copy
 * qe_head		next descriptor to process (already copied)
 * desc_len_pos		position of len field in descriptor
 * desc_addr_pos	position of buffer address field in descriptor
 * desc_cookie_pos	position of cookie field in descriptor
 *
 *
 * in general an GIU queue has the following indices.
 * The internal indices (qe_*, buf_*) are currently only maintained for src queues.
 *
 * tail:	is updated by the producing entity (never GIU for source queues)
 * qe_tail:	updated every time a QE copy is submitted to dma
 * qe_head:	updated every time a QE copy completea
 * head:	updated when processing completes on the element.
 *		(buffer copy in egress path, QE copy in ingress path)
 *
 * The diagram below shows a possible queue state for egress and ingress path
 * in egress path, the QE are copied first and then the buffers.
 * In ingress the opposite flow occurs.
 *
 *		 Egress Q state		Ingress Q state
 *		-----------------	-----------------
 *		|		|	|		|
 *		|    empty	|	|     empty	|
 *		|		|	|		|
 *		|		|	|		|
 *	tail -> +---------------+	+---------------+ <- tail
 *		|   QEs		|	|		|
 *		|   to copy	|	|		|
 *   qe_tail ->	+---------------+       |		|
 *		|   QEs	        |       |		|
 *		|   under copy  |	|  QEs to copy	|
 *   qe_head ->	+---------------+       |		|
 *		|  bufs to copy	|       |		|
 *		|		|       |		|
 *		|		|       |		|
 *      head ->	+---------------+	+---------------+ <- head
 *		|		|	|		|
 *		|    empty	|	|     empty	|
 *		|		|	|		|
 *		-----------------	-----------------
 */
struct gie_queue {
	u64	ring_phys;
	u64	ring_virt;
	u64	msg_head_phys;
	u64	msg_tail_phys;
	u64	msg_head_virt;
	u64	msg_tail_virt;
	u64	host_remap;
	u64	packets;
	u16	*idx_ring_virt;
	u16	*idx_ring_phys;
	u32	idx_ring_size;
	u32	idx_ring_ptr;
	u32	tc;
	u16	qlen;
	u16	qid;
	u16	tail;
	u16	head;
	u16	qe_tail;
	u16	qe_head;
	u16	desc_len_pos;
	u16	desc_addr_pos;
	u16	desc_cookie_pos;
	u8	qesize;
};

/* Structure describing an GIE buffer pool
 *
 * src_q	the bpool queue to consume
 * shadow	Local BM shadow queue used to copy buffer entries from host
 *		Only relevant for host bpools
 * buf_size	the size of the buffers in the buffer pool
 * flags	misc flags
 *		GIE_BPOOL_REMOTE	the ring resides on host memory
 */
struct gie_bpool {
	struct	gie_queue src_q;
	struct	gie_queue shadow;
	u32	buf_size;
#define GIE_BPOOL_REMOTE	(1 << 0)
	u32	flags;
};

/* variables specific to data queues (i.e. RX/TX)
 *
 * qpd		queue producer descriptor from QPT
 * qcd		queue consumer descriptor from QCT
 * src_q	the source queue to read from
 * dst_q	the destination queue to write to
 * flags	misc flags
 *		GIE_QPAIR_VALID	queue pair is valid
 *		GIE_QPAIR_ACTIVE	queue pair should be processed
 *		GIE_QPAIR_CP_PAYLOAD	queue pair has payload
 *		GIE_QPAIR_REMOTE	source ring is on host memory
 * dst_bpools	The bpools of the destination queues
 */
struct gie_q_pair {
	struct mqa_qct_entry	*qcd;
	struct mqa_qpt_entry	*qpd;
	struct gie_queue	src_q;
	struct gie_queue	dst_q;
#define		GIE_QPAIR_VALID	(1 << 0)
#define		GIE_QPAIR_ACTIVE	(1 << 1)
#define		GIE_QPAIR_CP_PAYLOAD	(1 << 4)
#define		GIE_QPAIR_REMOTE	(1 << 5)
	u32	flags;
	struct	gie_bpool *dst_bpools[GIE_MAX_BM_PER_Q];
};

/* Describes transactions submitted to dma
 * To reduce overehead, a single dma_job_info struct
 * can desribe multiple dma descriptors submitted for processing
 *
 * cookie		use to resture the queue or queue pair
 * elements_per_desc	total elements a single dma descriptor copies
 * desc_count		total descriptors in entire job
 * flags
 *	DMA_FLAG_IS_QE	this job describes qe copy and not buffer copy
 */
struct dma_job_info {
	void	*cookie;
	u32	elements_per_desc;
	u32	desc_count;
#define DMA_FLAGS_UPDATE_IDX		(1 << 0)
	u32	flags;
};

/* holds persistent dma information associated with and GIU emulator
 *
 * dma_jobs	an array containing dma_job_info elements
 *		updated on job submission and read when handling
 *		completed dma jobs from dma engine
 * dmax2	handle to HW DMA engine
 * job_tail	index to next element to add
 * job_head	index of next element to remove
 * job_qlen	size in elements of job queue
 */
struct dma_info {
	struct dma_job_info	dma_jobs[GIE_MAX_DMA_JOBS];
	struct dmax2		*dmax2;
	u16			job_tail;
	u16			job_head;
	u16			job_qlen;
};

/* used to describe dma copy jobs to dma wrapper functions
 *
 * cookie	used to recover queue information
 * src		source physical pointer
 * dst		destination physical pointer
 * element_size size of each element to copy
 * flags	job flags, same as in job_info
 * element_cnt	number of elements to copy
 */
struct dma_job {
	void	*cookie;
	u64	src;
	u64	dst;
	u32	element_size;
	u32	flags;
	u16	element_cnt;
};

/*
 * The main gie data structure. We have one per emulator.
 * This structure is internal and used only by the emulator
 * to track it's information
 *
 * name		name of the GIU instance
 * dmax2	a handle to the MUSDK dma engine
 * regs		the GIU register file
 * qpairs	queue pair descriptors
 * bpools	BM queue descriptors
 * q_cnt	queue pair count per TC
 * bp_cnt	buffer pool count
 * curr_q	the current queue to schedule
 * curr_tc	the current tc to schedule
 */
struct gie {
	char			name[GIE_MAX_NAME];
	struct dma_info		dma;
	struct gie_regfile	*regs;
	struct gie_q_pair	qpairs[GIE_MAX_TCS][GIE_MAX_Q_PER_TC];
	struct gie_bpool	bpools[GIE_MAX_BPOOLS];
	u16			q_cnt[GIE_MAX_TCS];
	u16			bp_cnt;
	u16			curr_q;
	u16			curr_tc;
};

#define map_host_addr(host_addr, q)	(q->host_remap + host_addr)

/* queue handling macros. assumes q size is always power of 2 */
#define q_occupancy(q)		((q->tail - q->head + q->qlen) & (q->qlen - 1))
#define q_space(q)		(q->qlen - q_occupancy(q) - 1)
#define q_empty(q)		(q->tail == q->head)
#define q_wraps(q)		(q->tail < q->head)

#define qes_to_copy(q)			((q->tail - q->qe_tail + q->qlen) & (q->qlen - 1))
/* How many "empty" QEs we have in queue.
 * i.e. How many QE DMA operations we can perform on the "dest" queue before we
 * reach the constumer pointer of that queue.
 */
#define qes_copy_space(q)		((q->head - q->qe_tail + q->qlen - 1) & (q->qlen - 1))
#define qes_in_copy(q)			((q->qe_tail - q->qe_head + q->qlen) & (q->qlen - 1))
#define qes_copied(q)			((q->qe_head - q->head + q->qlen) & (q->qlen - 1))
#define bufs_to_copy(q)			qes_copied(q)

#define q_idx_add(idx, val, qlen) (idx = (idx + val) & (qlen - 1))
#define q_read_idx(idx)                (*((u16 *)idx))


/* forward function declarations. */
static void gie_clean_dma_jobs(struct dma_info *dma);

/* Initialize the emulator. Basically the emulator only needs
 * the register base and the DMA engine from the caller.
 * The rest is initializations of local data structures
 */
void *gie_init(void *gie_regs, int dma_id, char *name)
{
	struct dmax2_params dmax2_params;
	struct dma_info *dma;
	struct gie *gie;
	char dma_name[16];
	int err;

	pr_info("Initializing %s-giu instance\n", name);

	gie = kcalloc(1, sizeof(struct gie), GFP_KERNEL);
	if (gie == NULL)
		return NULL;

	strncpy(gie->name, name, GIE_MAX_NAME);
	dma = &gie->dma;

	/* Open a MUSDK DMAx2 channel */
	sprintf(dma_name, "dmax2-%d", dma_id);
	dmax2_params.match = dma_name;
	dmax2_params.queue_size = DMAX2_BURST_SIZE;
	err = dmax2_init(&dmax2_params, &dma->dmax2);
	if (err) {
		pr_err("Failed to initialize MUSDK dmax2\n");
		goto init_error;
	}

	err = dmax2_set_mem_attributes(dma->dmax2, DMAX2_TRANS_LOCATION_SRC_AND_DST, DMAX2_TRANS_MEM_ATTR_CACHABLE);
	if (err) {
		pr_err("Failed to set dmax2 attributes\n");
		goto attr_error;
	}

	gie->regs = (struct gie_regfile *)gie_regs;

	/* reset the dma job queue */
	dma->job_tail = dma->job_head = 0;
	dma->job_qlen = GIE_MAX_DMA_JOBS;

	return gie;

attr_error:
	dmax2_deinit(dma->dmax2);
init_error:
	kfree(gie);
	return NULL;
}

/* Close the GIU emulator device.
 * At this point we assume that there is no active traffic.
 * We basically Close the MUSDK DMAx2 channel and free memory
 */
int gie_terminate(void *giu)
{
	struct gie *gie = (struct gie *)giu;
	int ret;

	pr_info("Terminating %s-giu instance\n", gie->name);

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

	kfree(gie);
	return ret;
}

void *gie_regs(void *giu)
{
	return (void *)(((struct gie *)giu)->regs);
}

static struct gie_q_pair *gie_find_q_pair(struct gie_q_pair *vec, int max, u16 qid)
{
	int i;

	for (i = 0; i < max; i++) {
		if ((vec[i].flags & GIE_QPAIR_VALID) && (vec[i].src_q.qid == qid))
			return &vec[i];
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
	pr_debug("tc:			%d\n", q->tc);
	pr_debug("ring_phys:		%p\n", (void *)q->ring_phys);
	pr_debug("ring_virt:		%p\n", (void *)q->ring_virt);
	pr_debug("msg_head_phys:	%p\n", (void *)q->msg_head_phys);
	pr_debug("msg_tail_phys:	%p\n", (void *)q->msg_tail_phys);
	pr_debug("msg_head_virt:	%p\n", (void *)q->msg_head_virt);
	pr_debug("msg_tail_virt:	%p\n", (void *)q->msg_tail_virt);
	pr_debug("host_remap:		%p\n", (void *)q->host_remap);
	pr_debug("qlen:		0x%x\n", q->qlen);
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
	q->tc = mqa->queue_prio;
	q->packets = 0;

	q->idx_ring_virt = mv_sys_dma_mem_alloc(sizeof(u16) * q->qlen, sizeof(u16));
	if (!q->idx_ring_virt)
		return -ENOMEM;

	q->idx_ring_phys = (u16 *)mv_sys_dma_mem_virt2phys(q->idx_ring_virt);
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
	} else {
		q->desc_len_pos = offsetof(struct host_rx_desc, byte_cnt);
		q->desc_addr_pos = offsetof(struct host_rx_desc, buffer_addr);
		q->desc_cookie_pos = offsetof(struct host_rx_desc, cookie);
	}

	gie_show_queue(q);

	return 0;
}

int gie_add_queue(void *giu, u16 qid, int is_remote)
{
	struct gie *gie = (struct gie *)giu;
	struct gie_q_pair *qp = NULL;
	struct mqa_qct_entry *qcd;
	struct mqa_qpt_entry *qpd;
	struct gie_bpool *bpool;
	int i, copy_payload, ret;
	u32 tc;

	pr_debug("adding qid %d to %s-giu\n", qid, gie->name);

	qcd = (struct mqa_qct_entry *)gie->regs->gct_base + qid;
	qpd = (struct mqa_qpt_entry *)gie->regs->gpt_base + qcd->spec.dest_queue_id;
	tc = qcd->common.queue_prio;
	copy_payload = qcd->common.flags & MQA_QFLAGS_COPY_BUF;

	/* Find an empty q_pair slot */
	for (i = 0; i < GIE_MAX_Q_PER_TC; i++) {
		if (!(gie->qpairs[tc][i].flags & GIE_QPAIR_VALID)) {
			qp = &gie->qpairs[tc][i];
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

	gie->q_cnt[tc]++;

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
	return 0;
}

int gie_remove_queue(void *giu, u16 qid)
{
	struct gie *gie = (struct gie *)giu;
	struct gie_q_pair *qp;

	qp = gie_find_q_pair(&gie->qpairs[0][0], GIE_MAX_TCS * GIE_MAX_Q_PER_TC, qid);
	if (qp == NULL) {
		pr_warn("Cannot find queue %d to remove\n", qid);
		return -ENODEV;
	}

	qp->flags = 0;
	gie->q_cnt[qp->src_q.tc]--;
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

static void gie_start_schedule(struct gie *gie)
{
	gie->curr_tc = 0;
	gie->curr_q = 0;
}

/* Find the next queue to service according to the
 * scheduling algorithm and queue counters
 * TODO - we currently implement a dumb round robin
 * on TC 0 only.
 */
static struct gie_q_pair *gie_get_next_q(struct gie *gie)
{
	struct gie_q_pair *qp;
	int tc = 0;

	while (gie->curr_q < gie->q_cnt[tc]) {
		qp = &gie->qpairs[tc][gie->curr_q];
		gie->curr_q++;
		if (qp->flags | GIE_QPAIR_ACTIVE)
			return qp;
	}

	/* no more active queues */
	return NULL;
}

/* create a dma copy job using a single dma transaction */
static int gie_dma_copy_single(struct dma_info *dma, struct dma_job *job)
{
	struct dma_job_info *job_info;
	struct dmax2_desc desc;
	int timeout = 100;
	u16 cnt;

	/* log the job in the dma job queue */
	job_info = dma->dma_jobs + dma->job_tail;
	q_idx_add(dma->job_tail, 1, dma->job_qlen);

	job_info->cookie = job->cookie;
	job_info->elements_per_desc = job->element_cnt;
	job_info->desc_count = 1;
	job_info->flags = job->flags;

	/* create the dma job to submit */
	desc.flags = 0x0;
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

	return 0;
}

static void gie_copy_index(struct dma_info *dma, u64 src, u64 dst)
{
	struct dma_job job;

	job.cookie = NULL;
	job.element_size = sizeof(u16);
	job.flags = 0;

	job.src = src;
	job.dst = dst;
	job.element_cnt = 1;
	gie_dma_copy_single(dma, &job);
}

static void gie_copy_qes(struct dma_info *dma, struct gie_queue *src_q, struct gie_queue *dst_q,
			  int qes_to_copy, u16 first_qe, int remote)
{
	struct dma_job job;
	int qesize = src_q->qesize;
	u32 first_batch, second_batch;
	int dma_cnt = 0;

	/* check if this copy wraps */
	if (first_qe + qes_to_copy > src_q->qlen) {
		first_batch = src_q->qlen - first_qe;
		second_batch = qes_to_copy - first_batch;
	} else {
		first_batch = qes_to_copy;
		second_batch = 0;
	}

	job.cookie = src_q;
	job.element_size = qesize;
	job.flags = 0;
	if (remote)
		job.flags |= DMA_FLAGS_UPDATE_IDX;

	job.src = src_q->ring_phys + first_qe * qesize;
	job.dst = dst_q->ring_phys + first_qe * qesize;
	job.element_cnt = first_batch;
	gie_dma_copy_single(dma, &job);
	dma_cnt++;

	if (second_batch) {
		job.src = src_q->ring_phys;
		job.dst = dst_q->ring_phys;
		job.element_cnt = second_batch;
		gie_dma_copy_single(dma, &job);
		dma_cnt++;
	}

	tracepoint(gie, queue, "QE copy", qes_to_copy, src_q->qid, first_qe, src_q->tail,
		   dst_q->qid, dst_q->head, dst_q->tail);
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

static void gie_bpool_fill_shadow(struct dma_info *dma, struct gie_bpool *pool)
{
	struct gie_queue *src_q = &pool->src_q;
	struct gie_queue *shadow_q = &pool->shadow;
	int fill_size = GIE_SHADOW_FILL_SIZE;
	int src_q_fill;
	u32 qes_copied;
	u64 head_bkp;

	/* did we already submit a copy ? if yes, wait till it's done */
	if (qes_in_copy(src_q))
		return;

	qes_copied = qes_copied(src_q);
	if (qes_copied) {
		/* previous copy completed, update pointers */
		q_idx_add(shadow_q->tail, qes_copied, shadow_q->qlen);
		q_idx_add(src_q->head, qes_copied, src_q->qlen);

		head_bkp = gie_idx_backup(src_q, src_q->head);
		gie_copy_index(dma, head_bkp, src_q->msg_head_phys);
		return;
	}

	/* okay, let's submit a copy */
	src_q->tail = q_read_idx(src_q->msg_tail_virt);
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
	gie_copy_qes(dma, src_q, shadow_q, fill_size, src_q->qe_tail, 1);
	q_idx_add(src_q->qe_tail, fill_size, src_q->qlen);
}

static int gie_get_bpool_bufs(struct dma_info *dma, struct gie_q_pair *qp, u32 min_buf_size,
			       struct host_bpool_desc **bp_bufs, int buf_cnt)
{
	struct gie_bpool *pool;
	struct gie_queue *bpool_q;
	int i, bufs_avail;

	/* Find the bpool that matches the min buffer size */
	for (i = 0; i < GIE_MAX_BPOOLS; i++) {
		pool = qp->dst_bpools[i];
		if (min_buf_size <= pool->buf_size)
			break;
	}

	if (i == GIE_MAX_BPOOLS) {
		pr_err("Failed to find bpool for buffer size %d\n", min_buf_size);
		return 0;
	}

	/* For remote queues, fill the shadow. For locals, update the tail pointer */
	if (pool->flags & GIE_BPOOL_REMOTE) {
		bpool_q = &pool->shadow;
		bufs_avail = q_occupancy(bpool_q);
		if ((bufs_avail < GIE_SHADOW_FILL_THRESH) || (bufs_avail < buf_cnt))
			gie_bpool_fill_shadow(dma, pool);
	} else {
		bpool_q = &pool->src_q;
		bpool_q->tail = q_read_idx(bpool_q->msg_tail_virt);
		bufs_avail = q_occupancy(bpool_q);
	}

	if (buf_cnt > bufs_avail)
		/* TODO: Add a trancepoint. */
		buf_cnt = bufs_avail;

	/* since we return a pointer to the queue, we can only return the amount
	 * of buffers until wrap around, so the caller doesn't need to wrap
	 */
	if (q_wraps(bpool_q))
		buf_cnt = min(buf_cnt, bpool_q->qlen - bpool_q->head);

	/* increment an internal index to indicate these buffers are already used we still
	 * don't update the remote head until the user is done with the buffers to avoid override
	 */
	*bp_bufs = (struct host_bpool_desc *)bpool_q->ring_virt + bpool_q->head;
	q_idx_add(bpool_q->head, buf_cnt, bpool_q->qlen);

	return buf_cnt;
}

static void gie_bpool_consume(struct dma_info *dma, struct gie_q_pair *qp, u32 min_buf_size)
{
	struct gie_bpool *pool;
	struct gie_queue *bpool_q;
	u64 head_bkp;
	int i;

	/* Find the bpool that matches the min buffer size */
	for (i = 0; i < GIE_MAX_BPOOLS; i++) {
		pool = qp->dst_bpools[i];
		if (min_buf_size <= pool->buf_size)
			break;
	}

	if (i == GIE_MAX_BPOOLS) {
		pr_err("Failed to find bpool for buffer size %d\n", min_buf_size);
		return;
	}

	/* remote bpools indixes are managed in refill routine */
	if (!(pool->flags & GIE_BPOOL_REMOTE)) {
		bpool_q = &pool->src_q;
		head_bkp = gie_idx_backup(bpool_q, bpool_q->head);
		gie_copy_index(dma, head_bkp, bpool_q->msg_head_phys);
	}
}

static int gie_copy_buffers(struct dma_info *dma, struct gie_q_pair *qp, struct gie_queue *src_q,
			     struct gie_queue *dst_q, struct gie_queue *qes_q, int bufs_to_copy)
{
	struct dmax2_desc desc[GIE_MAX_QES_IN_BATCH];
	struct host_bpool_desc *bp_buf;
	struct dma_job_info *job_info;
	u16 addr_pos, cookie_pos, len_pos;
	u64 src_remap, dst_remap;
	u64 *qe_cookie, *qe_buff;
	u16 *qe_byte_cnt;
	int i = 0, buf_cnt;
	void *qe;
	u16 cnt = 0;

	src_remap = src_q->host_remap;
	dst_remap = dst_q->host_remap;
	addr_pos = src_q->desc_addr_pos;
	cookie_pos = src_q->desc_cookie_pos;
	len_pos = src_q->desc_len_pos;

	i = src_q->head;

another_pass:
	/* fetch buffers from the bpool. the returned pointer points directly to the
	 * bpool queue elements to avoid copying them. therefore, when the queue is
	 * about the wrap, we only get the last items in the queue, so we dont need to
	 * handle the wrap-around in the loop. That's why we may call the function
	 * multiple times, first to get the last n elements in the queue, and then
	 * to get bufs_to_copy-n elements.
	 */
	buf_cnt = gie_get_bpool_bufs(dma, qp, 1500, &bp_buf, bufs_to_copy);
	if (!buf_cnt)
		goto bpool_empty;

	bufs_to_copy -= buf_cnt;

	while (buf_cnt--) {
		qe = (void *)qes_q->ring_virt + i * qes_q->qesize;
		qe_buff = qe + addr_pos;
		qe_cookie = qe + cookie_pos;
		qe_byte_cnt = qe + len_pos;

		/* create the dma job to submit */
		desc[cnt].flags = 0x0;
		desc[cnt].desc_ctrl = DESC_OP_MODE_MEMCPY << DESC_OP_MODE_SHIFT;
		desc[cnt].src_addr = src_remap + *qe_buff;
		desc[cnt].dst_addr = dst_remap + bp_buf->buff_addr_phys;
		desc[cnt].buff_size = *qe_byte_cnt;

		tracepoint(gie, dma, (void *)desc[cnt].src_addr, (void *)desc[cnt].dst_addr, desc[cnt].buff_size);
		cnt++;

		/* update qe buffer with host buffer*/
		*qe_buff = bp_buf->buff_addr_phys;
		*qe_cookie = bp_buf->buff_cookie;
		q_idx_add(i, 1, src_q->qlen);
		bp_buf++;
	}

	if (bufs_to_copy)
		goto another_pass;

bpool_empty:
	/* we can reach here after 1 pass or no pass at all */
	if (cnt) {
		/* log the job in the dma job queue */
		job_info = dma->dma_jobs + dma->job_tail;
		q_idx_add(dma->job_tail, 1, dma->job_qlen);

		job_info->cookie = src_q;
		job_info->elements_per_desc = 1;
		job_info->flags = 0;

		dmax2_enq(dma->dmax2, desc, &cnt);
		job_info->desc_count = cnt;

		/* release the bpool elements only after we used them
		 * to avoid override by producer
		 */
		gie_bpool_consume(dma, qp, 1500);

		tracepoint(gie, queue, "buffer copy", cnt, src_q->qid, src_q->head, src_q->tail,
			   dst_q->qid, dst_q->head, dst_q->tail);
	}

	return cnt;
}

static void gie_produce_q(struct dma_info *dma, struct gie_queue *src_q, struct gie_queue *dst_q, int qes_completed)
{
	u64 head_bkp, tail_bkp;

	q_idx_add(src_q->head, qes_completed, src_q->qlen);
	q_idx_add(dst_q->tail, qes_completed, dst_q->qlen);

	head_bkp = gie_idx_backup(src_q, src_q->head);
	tail_bkp = gie_idx_backup(dst_q, dst_q->tail);

	gie_copy_index(dma, head_bkp, src_q->msg_head_phys);
	gie_copy_index(dma, tail_bkp, dst_q->msg_tail_phys);
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
	int qes_copied, qes_to_copy, qes_copy_space;
	int completed = 0;

	/* TODO - consider this limitation in the future */
	(void)qe_limit;

	/* Get the updated tail & head from the notification area */
	src_q->tail = q_read_idx(src_q->msg_tail_virt);
	dst_q->head = q_read_idx(dst_q->msg_head_virt);

	/* check for any pending work (qes, buffers, etc) */
	if (q_empty(src_q))
		return 0;

	/* First phase - Copy the QEs to destination queue */
	qes_to_copy = qes_to_copy(src_q);
	qes_copy_space = qes_copy_space(dst_q);
	qes_to_copy = min(qes_to_copy, qes_copy_space);
	if (qes_to_copy) {
		qes_to_copy = gie_clip_batch(dst_q, qes_to_copy);
		gie_copy_qes(dma, src_q, dst_q, qes_to_copy, src_q->qe_tail, 1);
		q_idx_add(src_q->qe_tail, qes_to_copy, src_q->qlen);
		q_idx_add(dst_q->qe_tail, qes_to_copy, dst_q->qlen);
	}

	/* Second phase - copy the buffers and update prod/cons index */
	qes_copied = qes_copied(src_q);
	if (qes_copied) {
		qes_copied = gie_clip_batch(dst_q, qes_copied);
		if (copy_payload)
			completed = gie_copy_buffers(dma, qp, src_q, dst_q, dst_q, qes_copied);
		else
			completed = qes_copied;
	}

	/* Last phase - update the remote indices to indicate production/consumption */
	if (completed)
		gie_produce_q(dma, src_q, dst_q, completed);

	return completed;
}

static int gie_process_local_q(struct dma_info *dma, struct gie_q_pair *qp, int qe_limit)
{
	struct gie_queue *src_q = &qp->src_q;
	struct gie_queue *dst_q = &qp->dst_q;
	int copy_payload = qp->flags & GIE_QPAIR_CP_PAYLOAD;
	int qes;

	/* TODO - consider this limitation in the future */
	(void)qe_limit;

	/* Get the updated tail & head from the notification area */
	src_q->tail = q_read_idx(src_q->msg_tail_virt);
	dst_q->head = q_read_idx(dst_q->msg_head_virt);

	/* check for any pending work (qes, buffers, etc) */
	if (q_empty(src_q))
		return 0;

	/* for data queues, first copy  the buffers.
	 * not all buffers might be copied due to lack of bpools
	 */
	qes = q_occupancy(src_q);
	qes = gie_clip_batch(dst_q, qes);

	if (copy_payload)
		qes = gie_copy_buffers(dma, qp, src_q, dst_q, src_q, qes);

	/* if bpools are empty, no buffer copy will occur.
	 * If so, skip the QE copy as well
	 */
	if (!qes)
		return 0;

	/* Copy the QEs */
	gie_copy_qes(dma, src_q, dst_q, qes, src_q->head, 0);

	/* Update the prod/cons index by dma */
	gie_produce_q(dma, src_q, dst_q, qes);

	return qes;
}

static void gie_clean_dma_jobs(struct dma_info *dma)
{
	struct dma_job_info *jobi;
	struct gie_queue *src_q;
	u16 completed, release, elements, q_add;
	int i;

	completed = dmax2_get_deq_num_available(dma->dmax2);
	if (!completed)
		return;

	i = dma->job_head;
	release = completed;
	while (completed) {

		/* a single job can clean multiple descriptors, up-to the amounts dequed from DMA.
		 * if the job includes more descriptors, they are left for cleanup next time
		 */
		jobi = dma->dma_jobs + i;
		elements = min_t(u16, completed, jobi->desc_count);
		completed -= elements;
		jobi->desc_count -= elements;

		/* do not advance job_info queue if desc are left in the job */
		if (jobi->desc_count == 0)
			q_idx_add(i, 1, dma->job_qlen);

		if (!(jobi->flags & DMA_FLAGS_UPDATE_IDX))
			continue;

		src_q = (struct gie_queue *)jobi->cookie;
		q_add = elements * jobi->elements_per_desc;
		q_idx_add(src_q->qe_head, q_add, src_q->qlen);
	}
	dma->job_head = i;

	dmax2_deq(dma->dmax2, NULL, &release, 0);
}

int gie_schedule(void *giu, u64 time_limit, u64 qe_limit)
{
	struct gie *gie = (struct gie *)giu;
	struct gie_q_pair *qp;
	int qes;

	if (qe_limit == 0)
		qe_limit = UINT64_MAX;
	if (time_limit == 0)
		time_limit = UINT64_MAX;

	tracepoint(gie, flow, "start scheduling", gie->name);

	gie_clean_dma_jobs(&gie->dma);

	gie_start_schedule(gie);

	while (1) {

		/* TODO - consider time and qe limit as stop conditions */
		qp = gie_get_next_q(gie);
		if (qp == NULL)
			break;

		if (qp->flags & GIE_QPAIR_REMOTE)
			qes = gie_process_remote_q(&gie->dma, qp, qe_limit);
		else
			qes = gie_process_local_q(&gie->dma, qp, qe_limit);

		qe_limit -= qes;
	}

	return 0;
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

