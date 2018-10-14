/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#ifndef _MQA_INTERNAL_H
#define _MQA_INTERNAL_H

#include "drivers/mv_mqa.h"
#include "drivers/mqa_def.h"
#include "drivers/mv_mqa_queue.h"

#define MQA_REGION_MAX			(16)	/** Max number of regions in MQA tables */
#define MQA_REGION_FREE			(-1)

#define MQA_REGION_INIT_COUNT	(0)

/* MQA Queue attributes definitions */
#define EGRESS_MQA_QUEUE_INGRESS_BIT_FIELD_ATTR	(MQA_QUEUE_EGRESS | MQA_QUEUE_INGRESS)
#define LOCAL_MQA_QUEUE_REMOTE_BIT_FIELD_ATTR	(MQA_QUEUE_LOCAL | MQA_QUEUE_REMOTE)

/* MQA Queue attributes checking status */
#define IS_QUEUE_INGRESS(x)	(((x) & EGRESS_MQA_QUEUE_INGRESS_BIT_FIELD_ATTR) == MQA_QUEUE_INGRESS)
#define IS_QUEUE_LOCAL(x)	(((x) & LOCAL_MQA_QUEUE_REMOTE_BIT_FIELD_ATTR) == MQA_QUEUE_LOCAL)

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
	u32 remote_index_location;
};

/** MQA Table entry parameters */
struct mqa_table_entry {

	u32 mqa_queue_attr;

	struct mqa_queue		common;
	struct mqa_queue_qpt_spec	qpt;
	struct mqa_queue_qct_spec	qct;

};

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
int queue_associate_notify_intr(struct mqa *mqa, u32 phy_queue_id, struct mqa_queue_msix_params *params);
int queue_associate_bpool(struct mqa *mqa, u32 phy_queue_id, u32 bpool_num, u32 *bpool);

#endif /* _MQA_INTERNAL_H */

