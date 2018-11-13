/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#ifndef _GIU_INT_H_
#define _GIU_INT_H_

#include "std_internal.h"
#include "../include/gie.h"
#include "gie_buff_desc.h"

#define GIE_MAX_NAME		256
#define GIE_MAX_DMA_JOBS	4096
#define GIE_DMAX2_Q_SIZE	4096

#define GIE_MAX_QES_IN_BATCH	64

#define GIE_MAX_PRIOS		8
#define GIE_MAX_Q_PER_PRIO	128
#define GIE_MAX_BPOOLS		16
#define GIE_MAX_BM_PER_Q	1
#define GIE_SHADOW_FILL_SIZE	256
#define GIE_SHADOW_FILL_THRESH	128

/* #define GIE_VERIFY_DMA_OP */

#define GIE_MAX_QID		(UINT16_MAX + 1)

#define GIE_AGNIC_UIO_STRING	"agnic_tc_%u"

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

struct gie_event_data {
	struct gie *gie;
	int enable;
	struct gie_event_params ev_params;
};

struct gie_regfile {
	u32	ctrl;
	u32	status;
	u64	gct_base;
	u64	gpt_base;

	/* MSI phys/virt register base */
	u64 msi_regs_phys;
	u64 msi_regs_virt;
};

struct msix_table_entry {
	u64 msg_addr;
	u32 msg_data;
	u32 vector_ctrl;
};

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
 * prio			the priority associated with the ring
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
	u32	prio;
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
	/* MSI message info */
	u64	msi_virt_addr;
	u32	msi_data;
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
#define		GIE_QPAIR_VALID		(1 << 0)
#define		GIE_QPAIR_ACTIVE	(1 << 1)
#define		GIE_QPAIR_CP_PAYLOAD	(1 << 4)
#define		GIE_QPAIR_REMOTE	(1 << 5)
	u32	flags;
	struct	gie_bpool *dst_bpools[GIE_MAX_BM_PER_Q];
};

/* variables specific to priorities
 *
 * flags	misc flags
 *		GIE_PRIO_VALID	priority is valid
 *		GIE_PRIO_ACTIVE	priority should be processed
 * q_cnt	indicate how many queues available within this priority
 * curr_q	the current queue to schedule
 * qpairs	queue pair descriptors
 */
struct gie_prio {
#define		GIE_PRIO_VALID	(1 << 0)
#define		GIE_PRIO_ACTIVE	(1 << 1)
	u32			flags;
	u16			q_cnt;
	u16			curr_q;
	struct gie_q_pair	qpairs[GIE_MAX_Q_PER_PRIO];
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
	u64	cookie_dst;
	u64	cookie_head;
	u64	cookie_tail;
	u32	elements_per_desc;
	u32	desc_count;
#define DMA_FLAGS_NONE			(0)
#define DMA_FLAGS_UPDATE_IDX		(1 << 0)
#define DMA_FLAGS_PRODUCE_IDX		(1 << 1)
#define DMA_FLAGS_BUFFER_IDX		(1 << 2)
	u32	flags;
};

/* holds persistent dma information associated with and GIU emulator
 *
 * dma_jobs	an array containing dma_job_info elements
 *		updated on job submission and read when handling
 *		completed dma jobs from dma engine
 * dmax2	handle to HW DMA engine
 * tail		index to next element to add
 * head		index of next element to remove
 * qlen		size in elements of job queue
 */
struct dma_info {
	struct dma_job_info	dma_jobs[GIE_MAX_DMA_JOBS];
	struct dmax2		*dmax2;
	u16			tail;
	u16			head;
	u16			qlen;
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
	u64	cookie_dst;
	u64	cookie_head;
	u64	cookie_tail;
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
 * bpools	BM queue descriptors
 * q_cnt	queue pair count per priority
 * bp_cnt	buffer pool count
 * curr_prio	the current priority to schedule
 */
struct gie {
	char			name[GIE_MAX_NAME];
	struct dma_info		dma;
	struct gie_regfile	*regs;
	struct gie_prio		prios[GIE_MAX_PRIOS];
	struct gie_bpool	bpools[GIE_MAX_BPOOLS];
	u16			bp_cnt;
	u16			curr_prio;
};

#endif /* _GIU_INT_H_ */
