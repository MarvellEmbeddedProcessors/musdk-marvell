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

#ifndef _MQA_INTERNAL_H
#define _MQA_INTERNAL_H

#include "drivers/mv_mqa.h"

#define MQA_REGION_MAX			(16)	/** Max number of regions in MQA tables */
#define MQA_REGION_FREE			(-1)

#define MQA_REGION_INIT_COUNT	(0)

/* MQA Queue attributes */
#define EGRESS_QUEUE    (0 << 0)
#define INGRESS_QUEUE   (1 << 0)
#define LOCAL_QUEUE     (0 << 1)
#define REMOTE_QUEUE    (1 << 1)

/* MQA Queue attributes definitions */
#define EGRESS_INGRESS_QUEUE_BIT_FIELD_ATTR	(EGRESS_QUEUE | INGRESS_QUEUE)
#define LOCAL_REMOTE_QUEUE_BIT_FIELD_ATTR	(LOCAL_QUEUE | REMOTE_QUEUE)

/* MQA Queue attributes checking status */
#define IS_QUEUE_INGRESS(x)	(((x) & EGRESS_INGRESS_QUEUE_BIT_FIELD_ATTR) == INGRESS_QUEUE)
#define IS_QUEUE_LOCAL(x)	(((x) & LOCAL_REMOTE_QUEUE_BIT_FIELD_ATTR) == LOCAL_QUEUE)

/**
 * MQA Tables Global Definition
 *
 * qpt_base	queue producer context table base
 * qct_base	queue consumer context table base
 * qnpt_base	phys base of producer notification table
 * qnct_base	phys base of consumer notification table
 * qnpt_virt	virt address of qnpt_base
 * qnct_virt	virt address of qpct_base
 * size		size of the tables
 */
struct mqa {
	void *qpt_base;
	void *qct_base;
	void *qnpt_base;
	void *qnct_base;
	void *qnpt_virt;
	void *qnct_virt;
	u32 size;
};


/**
 * MQA Queue Definition
 */

/** MQA extended queue parameters */
struct mqa_queue_ext {
	u32 msi_x_id;		/** MSI-X interrupt Id */
	u32 bm_queue[MQA_BM_QUEUE_ARRAY];

};

/** MQA common queue parameters  */
struct mqa_queue {

	u64 ring_phy_addr;	/** Ring physical base address */
	u64 ring_virt_addr;	/** Ring virtual base address - relevant for local queues */
	u64 prod_phys;		/** Queue producer physical address */
	u64 prod_virt;		/** Queue producer virtual address */
	u64 cons_phys;		/** Queue consumer physical address */
	u64 cons_virt;		/** Queue consumer virtual address */
	u64 host_remap;		/** Remap address in case the queue is on host side */
	u32 ring_size;		/** Ring size */
	u32 entry_size;		/** Ring element size */
	u32 queue_prio;		/** queue priority */
#define MQA_QFLAGS_COPY_BUF	(1 << 0) /** copy the queue payload */
	u32 flags;		/** queue flags */

	struct mqa_queue_ext queue_ext;
};


/**
 * MQA Tables Definition
 */


/** MQA QPT entry parameters */
struct mqa_queue_qpt_spec {
	u64 reserved;
};

struct mqa_qpt_entry {
	struct mqa_queue common;	/** Queue common parameters */
	struct mqa_queue_qpt_spec spec;	/** QPT Queue specific parameters */

};

/** MQA QCT entry parameters */
struct mqa_queue_qct_spec {
	u32 dest_queue_id;		/** Queue desstination */
};

struct mqa_qct_entry {
	struct mqa_queue common;	/** Queue common parameters */
	struct mqa_queue_qct_spec spec;	/** QPT Queue specific parameters */

};

/** MQA GNPT entry parameters */
struct mqa_qnpt_entry {
	u32 producer_index;		/** Queue producer index */

};

/** MQA GNCT entry parameters */
struct mqa_qnct_entry {
	u32 consumer_index;		/** Queue consumer index */

};

/** MQA Table entry parameters */
struct mqa_table_entry {

	u32 mqa_queue_attr;

	struct mqa_queue		common;
	struct mqa_queue_qpt_spec	qpt;
	struct mqa_queue_qct_spec	qct;

};

#define MQA_QPT_ENTRY_SIZE	(sizeof(struct mqa_qpt_entry))
#define MQA_QCT_ENTRY_SIZE	(sizeof(struct mqa_qct_entry))
#define MQA_QNPT_ENTRY_SIZE	(sizeof(struct mqa_qnpt_entry))
#define MQA_QNCT_ENTRY_SIZE	(sizeof(struct mqa_qnct_entry))


/**
 * MQA Region definition
 */
struct mqa_region_params {
	u32 region_id;		/** MQA region Id */
	u32 region_start;	/** MQA region start index */
	u32 region_size;	/** MQA region size */

	u32 queue_alloc_count;	/** Number of allocated queue from the region */
	u32 queue_free_index;	/** Index of next free queue in the region */

};

/**
 * MQA Queue definition
 */
struct mqa_q {
	u32 q_id;
	u32 len;
	void *phy_base_addr;
	void *virt_base_addr;
	void *prod_phys;
	void *cons_phys;
	void *prod_virt;
	void *cons_virt;
};


int queue_alloc(struct mqa *mqa, u32 *q);
int queue_free(u32 phy_queue_id);
int queue_config(struct mqa *mqa, u32 phy_queue_id, struct mqa_table_entry *q_params);
int queue_associate_pair(struct mqa *mqa, u32 phy_queue_id, u32 dest_queue_id);
int queue_associate_notify_intr(struct mqa *mqa, u32 phy_queue_id, u32 msi_x_id);
int queue_associate_bpool(struct mqa *mqa, u32 phy_queue_id, u32 bpool_num, u32 *bpool);

#endif /* _MQA_INTERNAL_H */

