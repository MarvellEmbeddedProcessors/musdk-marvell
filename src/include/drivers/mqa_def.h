/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#ifndef _MQA_DEF_H
#define _MQA_DEF_H

#include "drivers/mv_mqa.h"

/**
 * MQA Queue Definition
 */

struct mqa_queue_msix_inf {
	u32		 id;	/**< MSI interrupt Id   (0 = unused) */
	u32		 data;
	phys_addr_t	 pa;
	void		*va;
};

/** MQA extended queue parameters */
struct mqa_queue_ext {
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
	struct mqa_queue_msix_inf msix_inf;
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

#define MQA_QPT_ENTRY_SIZE	(sizeof(struct mqa_qpt_entry))
#define MQA_QCT_ENTRY_SIZE	(sizeof(struct mqa_qct_entry))

#endif /* _MQA_DEF_H */

