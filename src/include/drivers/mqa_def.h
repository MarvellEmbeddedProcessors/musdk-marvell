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

/** MQA GNPT entry parameters */
struct mqa_qnpt_entry {
	u32 producer_index;		/** Queue producer index */

};

/** MQA GNCT entry parameters */
struct mqa_qnct_entry {
	u32 consumer_index;		/** Queue consumer index */

};

#define MQA_QPT_ENTRY_SIZE	(sizeof(struct mqa_qpt_entry))
#define MQA_QCT_ENTRY_SIZE	(sizeof(struct mqa_qct_entry))
#define MQA_QNPT_ENTRY_SIZE	(sizeof(struct mqa_qnpt_entry))
#define MQA_QNCT_ENTRY_SIZE	(sizeof(struct mqa_qnct_entry))

#endif /* _MQA_DEF_H */

