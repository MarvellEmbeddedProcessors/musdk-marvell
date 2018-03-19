
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

#define log_fmt(fmt) "mqa: " fmt

#include "std_internal.h"
#include "mqa_internal.h"
#include "drivers/mv_mqa.h"
#include "drivers/mv_mqa_queue.h"

#define MQA_GIU_REGION		(0)	/** NIC PF Region Id */
#define MQA_GIU_REGION_BASE		(1)	/** NIC PF Region Base */
#define MQA_GIU_REGION_SIZE		(128)	/** NIC PF Region Size */

#define MQA_RESERVED_REGION		(1)	/** Reserved Region Id */
#define MQA_RESERVED_REGION_BASE	(MQA_GIU_REGION_BASE + MQA_GIU_REGION_SIZE)
#define MQA_RESERVED_REGION_SIZE	(MQA_QUEUE_MAX - MQA_RESERVED_REGION_BASE)

/* MQA global information shadow for internal use */
#define MQA_FREE_QUEUE	(-1)

/* MQA validate queue */
#define INVALID_QUEUE(queue_id) (queue_id >= MQA_QUEUE_MAX)
#define INVALID_REGION(region_id) (region_id >= MQA_REGION_MAX)
#define UN_INITIALIZED_REGION(region_id) (region_id == MQA_REGION_FREE)

/* MQA Region Map
 * MQA support multiple regions, these array ststaicly defines
 * MQA regions and include region control informantion for queue
 * allocation from NIC-PF region
 */
struct mqa_region_params mqa_region_map[MQA_REGION_MAX];

/* MQA Queue state */
s8 *mqa_queue_state;

/* Debug API */
static void queue_dbg_globals_dump(struct mqa *mqa);
static void queue_dbg_region_dump(struct mqa_region_params *region);
static void queue_dbg_queue_dump(struct mqa_region_params *region, u32 queue_id);
static void queue_dbg_table_entry_dump(struct mqa *mqa, u32 queue_id);

/*
 *	queue_table_init
 *
 *	This function allocates MQA tables - QPT, QCT, QNPT, and QNCT
 *	the number of entries in each table is set to MQA_QUEUE_MAX = 1024
 *
 *	@param[in]	params - pointer to MQA parameters
 *	@param[in]	mqa - pointer to MQA object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
static int queue_table_init(struct mqa_params *params, struct mqa **mqa)
{
	pr_info("Initializing MQA tables\n");

	*mqa = kcalloc(1, sizeof(struct mqa), GFP_KERNEL);
	if (*mqa == NULL) {
		pr_err("Failed to allocate MQA handler\n");
		goto error;
	}

	(*mqa)->qpt_base = kcalloc(params->num_qs, MQA_QPT_ENTRY_SIZE, GFP_KERNEL);
	if ((*mqa)->qpt_base == NULL) {
		pr_err("Failed to allocate MQA qpt table\n");
		goto error;
	}

	(*mqa)->qct_base = kcalloc(params->num_qs, MQA_QCT_ENTRY_SIZE, GFP_KERNEL);
	if ((*mqa)->qct_base == NULL) {
		pr_err("Failed to allocate MQA qct table\n");
		goto error;
	}

	(*mqa)->size = params->num_qs;

	(*mqa)->qnpt_base = (void *)params->notif_tbl.qnpt_pa;
	(*mqa)->qnct_base = (void *)params->notif_tbl.qnct_pa;
	(*mqa)->qnpt_virt = (void *)params->notif_tbl.qnpt_va;
	(*mqa)->qnct_virt = (void *)params->notif_tbl.qnct_va;

	mqa_queue_state = kmalloc((sizeof(s8) * params->num_qs), GFP_KERNEL);
	if (mqa_queue_state == NULL)
		goto error;

	memset(mqa_queue_state, MQA_FREE_QUEUE, (sizeof(s8) * params->num_qs));

	/* Debug information */
	queue_dbg_globals_dump(*mqa);

	return 0;

error:
	/* No need to check pointer validity before kfree() */
	kfree((*mqa)->qpt_base);
	kfree((*mqa)->qct_base);
	kfree(mqa_queue_state);

	return -ENOMEM;
}


/*
 *	queue_region_map_init
 *
 *	This function map MQA regions
 *	It is statically defined and currently includes two regions
 *
 *	GIU Region Base		0	+-----------+
 *					|    GIU    +
 *					|           +
 *	GIU Region End		127	+-----------+
 *	Reserved Region Base	128	+-----------+
 *					| Reserved  +
 *					|           +
 *					|           +
 *					|           +
 *					|           +
 *	Reserved Region End	1023	+-----------+
 *
 *	Future regions will be added under reserved section
 *
 *	@retval	0 on success
 */
static int queue_region_map_init(void)
{
	struct mqa_region_params *reg_params;

	pr_info("Initializing MQA regions\n");

	/* Initialize MQA Region tables */
	memset(mqa_region_map, 0, sizeof(mqa_region_map));

	/* NIC PF Region Definition */
	/* ======================== */
	reg_params = &(mqa_region_map[MQA_GIU_REGION]);

	reg_params->region_id         = MQA_GIU_REGION;
	reg_params->region_start      = MQA_GIU_REGION_BASE;
	reg_params->region_size       = MQA_GIU_REGION_SIZE;
	reg_params->queue_alloc_count = MQA_REGION_INIT_COUNT;
	reg_params->queue_free_index  = MQA_GIU_REGION_BASE;

	/* Debug information */
	queue_dbg_region_dump(reg_params);

	/* Reserved Region Definition */
	/* ========================== */
	reg_params = &(mqa_region_map[MQA_RESERVED_REGION]);

	reg_params->region_id         = MQA_RESERVED_REGION;
	reg_params->region_start      = MQA_RESERVED_REGION_BASE;
	reg_params->region_size       = MQA_RESERVED_REGION_SIZE;
	reg_params->queue_alloc_count = MQA_REGION_INIT_COUNT;
	reg_params->queue_free_index  = MQA_RESERVED_REGION_BASE;

	/* Debug information */
	queue_dbg_region_dump(reg_params);

	return 0;
}


/*
 *	mqa_init
 *
 *	This function initializes MQA infrastructure
 *	MQA tables - GPT, GCT, GNPT, and GNCT are allocated and initialized
 *	MQA region map is statically initialized and divide MQA tables into dedicated regions
 *
 *	@param[in]	params - pointer to MQA parameters
 *	@param[in]	mqa - pointer to MQA object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int mqa_init(struct mqa_params *params, struct mqa **mqa)
{
	int ret;

	ret = queue_table_init(params, mqa);
	if (ret) {
		pr_err("Failed to initialize MQA tables\n");
		return ret;
	}

	ret = queue_region_map_init();
	if (ret) {
		pr_err("Failed to initialize MQA regions\n");
		return ret;
	}

	return 0;
}


/*
 *	mqa_terminate
 *
 *	This function release MQA tables - GPT, GCT, GNPT, and GNCT
 *	It clear MQA Region tables
 *
 *	@param[in]	mqa - pointer to MQA object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int mqa_deinit(struct mqa *mqa)
{
	/* Clear MQA region tables */
	memset(mqa_region_map, 0, sizeof(mqa_region_map));

	/* Release MQA tables */
	/* No need to check pointer validity before kfree() */
	kfree(mqa->qpt_base);
	kfree(mqa->qct_base);
	kfree(mqa_queue_state);

	return 0;
}


/*
 *	queue_alloc
 *
 *	This function allocates a queue from MQA region
 *	The region is scanned for the next available queue in the region
 *	Once allocated it is marked
 *
 *	@param[in]	mqa - pointer to MQA object
 *	@param[in]	q - pointer to MQA queue object
 *
 *	@retval	physical_queue_Id on success
 *	@retval	error-code otherwise
 */
int queue_alloc(struct mqa *mqa, u32 *q)
{
	u32 index;
	u32 queue_id;
	u32 region_id = MQA_GIU_REGION;
	struct mqa_region_params *reg_params;

	pr_debug("Allocating Queue, Region - %d\n", region_id);

	/* Validate region */
	if (INVALID_REGION(region_id)) {
		pr_err("Failed to allocate queue, invalid MQA region %d\n", region_id);
		return -EINVAL;
	}

	reg_params = &(mqa_region_map[region_id]);

	/* Validate if all queueus in the region were allocated */
	if (reg_params->queue_alloc_count >= reg_params->region_size)
		return -EBUSY;

	/* Scan region for next available free entry */
	for (index = 0; index < reg_params->region_size; index++) {

		if (mqa_queue_state[reg_params->queue_free_index] == MQA_FREE_QUEUE) {

			/* Marked entry as allocated */
			mqa_queue_state[reg_params->queue_free_index] = (s8)region_id;

			/* Increment region queue count */
			reg_params->queue_alloc_count++;

			/* Return queue Id */
			queue_id = reg_params->queue_free_index;

			/* Increment to next available queue */
			reg_params->queue_free_index++;
			if (reg_params->queue_free_index >= reg_params->region_start + reg_params->region_size)
				reg_params->queue_free_index = reg_params->region_start;

			/* Debug information */
			queue_dbg_queue_dump(reg_params, queue_id);

			*q = queue_id;

			return 0;
		}

		/* Roll over incase entry index exceeds region boundaries */
		reg_params->queue_free_index++;
		if (reg_params->queue_free_index >= reg_params->region_start + reg_params->region_size)
			reg_params->queue_free_index = reg_params->region_start;
	}

	pr_err("Failed to allocate queue from MQA region %d\n", region_id);

	return -ENOSPC;
}


/*
 *	queue_free
 *
 *	This function release a queue to MQA region
 *
 *	@param[in]	queue_id - queue Id
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int queue_free(u32 queue_id)
{
	s8 region_id;
	struct mqa_region_params *reg_params;

	/* Validate queue Id */
	if (INVALID_QUEUE(queue_id)) {
		pr_err("Failed to free queue, invalid MQA queue %d\n", queue_id);
		return -EINVAL;
	}

	region_id = mqa_queue_state[queue_id];

	/* Validate region */
	if ((INVALID_REGION(region_id)) ||
		(UN_INITIALIZED_REGION(region_id))) {
		pr_err("Failed to release queue, invalid MQA region %d\n", region_id);
		return -EINVAL;
	}

	pr_debug("Relase Queue %d from Region - %d\n", queue_id, region_id);

	reg_params = &(mqa_region_map[region_id]);

	/* Marked entry as free */
	mqa_queue_state[queue_id] = MQA_FREE_QUEUE;

	/* Decrement region queue count */
	reg_params->queue_alloc_count--;

	/* Debug information */
	queue_dbg_queue_dump(reg_params, queue_id);

	return 0;
}


/*
 *	queue_entry_config
 *
 *	This function initialize QPT, QCT queue entries in MQA tables
 *
 *	@param[in]	entry - pointer to QPT / QCT entry
 *	@param[in]	queue_params - pointer to queue parameters object
 */
static void queue_entry_config(struct mqa_queue *entry, struct mqa_table_entry *queue_params)
{
	memcpy(entry, &(queue_params->common), sizeof(struct mqa_queue));
}


/*
 *	queue_config
 *
 *	This function configure queue with input queue params
 *
 *	@param[in]	mqa - pointer to MQA object
 *	@param[in]	queue_id - queue Id
 *	@param[in]	queue_params - pointer to queue parameters object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int queue_config(struct mqa *mqa, u32 queue_id, struct mqa_table_entry *queue_params)
{
	s8 region_id;
	struct mqa_region_params *reg_params;
	struct mqa_qpt_entry *qpt;
	struct mqa_qct_entry *qct;
	struct mqa_qnpt_entry *qnpt_phys, *qnpt_virt;
	struct mqa_qnct_entry *qnct_phys, *qnct_virt;

	/* Validate queue */
	if (INVALID_QUEUE(queue_id)) {
		pr_err("Failed to config queue, invalid MQA queue %d\n", queue_id);
		return -EINVAL;
	}

	region_id = mqa_queue_state[queue_id];

	/* Validate region */
	if ((INVALID_REGION(region_id)) ||
		(UN_INITIALIZED_REGION(region_id))) {
		pr_err("Failed to config queue, invalid MQA region %d\n", region_id);
		return -EINVAL;
	}

	reg_params = &(mqa_region_map[region_id]);

	/* Validate queue boundaries */
	if ((queue_id < (reg_params->region_start)) ||
		(queue_id > (reg_params->region_start + reg_params->region_size))) {
		pr_err("Failed to config queue, invalid MQA queue %d\n", queue_id);
		return -EINVAL;
	}

	/* Calculate queue entries in MQA tables */
	qpt  = ((struct mqa_qpt_entry *)mqa->qpt_base) + queue_id;
	qct  = ((struct mqa_qct_entry *)mqa->qct_base) + queue_id;
	qnpt_phys = ((struct mqa_qnpt_entry *)mqa->qnpt_base) + queue_id;
	qnct_phys = ((struct mqa_qnct_entry *)mqa->qnct_base) + queue_id;
	qnpt_virt = ((struct mqa_qnpt_entry *)mqa->qnpt_virt) + queue_id;
	qnct_virt = ((struct mqa_qnct_entry *)mqa->qnct_virt) + queue_id;

	/* Initialize QPT, QCT queue entries in MQA tables
	*  Both tables include same information regarding queue structure
	*  Information is common to all queue types
	*/
	queue_entry_config(&(qpt->common), queue_params);
	queue_entry_config(&(qct->common), queue_params);

	*(u32 *)qnpt_virt = 0;
	*(u32 *)qnct_virt = 0;

	switch (queue_params->mqa_queue_attr) {

	case EGRESS_QUEUE | LOCAL_QUEUE:
	case INGRESS_QUEUE | LOCAL_QUEUE:
		qct->common.cons_phys = (u64)qnct_phys;
		qpt->common.cons_phys = (u64)qnct_phys;
		qct->common.prod_phys = (u64)qnpt_phys;
		qpt->common.prod_phys = (u64)qnpt_phys;
		qct->common.cons_virt = (u64)qnct_virt;
		qpt->common.cons_virt = (u64)qnct_virt;
		qct->common.prod_virt = (u64)qnpt_virt;
		qpt->common.prod_virt = (u64)qnpt_virt;
		queue_params->common.prod_phys = (u64)qnpt_phys;
		queue_params->common.cons_phys = (u64)qnct_phys;
		queue_params->common.prod_virt = (u64)qnpt_virt;
		queue_params->common.cons_virt = (u64)qnct_virt;
		break;

	case EGRESS_QUEUE | REMOTE_QUEUE:
		qct->common.prod_phys = (u64)qnpt_phys;
		qpt->common.prod_phys = (u64)qnpt_phys;
		qct->common.prod_virt = (u64)qnpt_virt;
		qpt->common.prod_virt = (u64)qnpt_virt;
		queue_params->common.prod_phys = (u64)qnpt_phys;
		queue_params->common.prod_virt = (u64)qnpt_virt;

		if (mqa->remote_index_location) {
			qct->common.cons_phys = queue_params->common.cons_phys;
			qpt->common.cons_phys = queue_params->common.cons_phys;
			qct->common.cons_virt = queue_params->common.cons_virt;
			qpt->common.cons_virt = queue_params->common.cons_virt;
		} else {
			qct->common.cons_phys = (u64)qnct_phys;
			qpt->common.cons_phys = (u64)qnct_phys;
			qct->common.cons_virt = (u64)qnct_virt;
			qpt->common.cons_virt = (u64)qnct_virt;
			queue_params->common.cons_phys = (u64)qnct_phys;
			queue_params->common.cons_virt = (u64)qnct_virt;
		}
		break;

	case INGRESS_QUEUE | REMOTE_QUEUE:
		qct->common.cons_phys = (u64)qnct_phys;
		qpt->common.cons_phys = (u64)qnct_phys;
		qct->common.cons_virt = (u64)qnct_virt;
		qpt->common.cons_virt = (u64)qnct_virt;
		queue_params->common.cons_phys = (u64)qnct_phys;
		queue_params->common.cons_virt = (u64)qnct_virt;

		if (mqa->remote_index_location) {
			qct->common.prod_phys = queue_params->common.prod_phys;
			qpt->common.prod_phys = queue_params->common.prod_phys;
			qct->common.prod_virt = queue_params->common.prod_virt;
			qpt->common.prod_virt = queue_params->common.prod_virt;
		} else {
			qct->common.prod_phys = (u64)qnpt_phys;
			qpt->common.prod_phys = (u64)qnpt_phys;
			qct->common.prod_virt = (u64)qnpt_virt;
			qpt->common.prod_virt = (u64)qnpt_virt;
			queue_params->common.prod_phys = (u64)qnpt_phys;
			queue_params->common.prod_virt = (u64)qnpt_virt;
		}
		break;
	}

	queue_dbg_table_entry_dump(mqa, queue_id);

	return 0;
}


/*
 *	queue_associate_pair
 *
 *	This function associate queue with target queue Id
 *
 *	@param[in]	mqa - pointer to MQA object
 *	@param[in]	queue_id - queue Id
 *	@param[in]	dest_queue_id - target queue Id
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int queue_associate_pair(struct mqa *mqa, u32 queue_id, u32 dest_queue_id)
{
	struct mqa_region_params *reg_params;
	struct mqa_qct_entry *qct;

	/* Validate queue */
	if ((INVALID_QUEUE(queue_id)) || (INVALID_QUEUE(dest_queue_id))) {
		pr_err("Failed to associate queue, invalid MQA queue %d, %d\n", queue_id, dest_queue_id);
		return -EINVAL;
	}

	/* Validate region */
	if ((UN_INITIALIZED_REGION(mqa_queue_state[queue_id])) ||
		(UN_INITIALIZED_REGION(mqa_queue_state[dest_queue_id]))) {
		pr_err("Failed to associate queue, invalid MQA region\n");
		return -EINVAL;
	}

	reg_params = &(mqa_region_map[mqa_queue_state[queue_id]]);

	/* Validate destination queue is in the boundaries of physical queue region */
	if ((dest_queue_id < (reg_params->region_start)) ||
		(dest_queue_id > (reg_params->region_start + reg_params->region_size))) {
		pr_err("Failed to config queue, invalid MQA queue %d\n", dest_queue_id);
		return -EINVAL;
	}

	pr_debug("Associate Queue - %d, Dest - %d\n", queue_id, dest_queue_id);

	/* Calculate queue entries in MQA tables */
	qct = ((struct mqa_qct_entry *)mqa->qct_base) + queue_id;

	qct->spec.dest_queue_id = dest_queue_id;

	return 0;
}


/*
 *	queue_associate_notify_intr
 *
 *	This function associate queue with notification interrupt Id
 *
 *	@param[in]	mqa - pointer to MQA object
 *	@param[in]	queue_id - queue Id
 *	@param[in]	msi_x_id - MSI-X Interrupt
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int queue_associate_notify_intr(struct mqa *mqa, u32 queue_id, u32 msi_x_id)
{
	struct mqa_qpt_entry *qpt;

	/* Validate queue */
	if (INVALID_QUEUE(queue_id)) {
		pr_err("Failed to associate queue, invalid MQA queue %d\n", queue_id);
		return -EINVAL;
	}

	/* Validate region */
	if (UN_INITIALIZED_REGION(mqa_queue_state[queue_id])) {
		pr_err("Failed to associate queue, invalid MQA region\n");
		return -EINVAL;
	}

	pr_debug("Associate Queue - %d, Notify Interrupt - %d\n", queue_id, msi_x_id);

	/* Calculate queue entries in MQA tables */
	qpt = ((struct mqa_qpt_entry *)mqa->qpt_base) + queue_id;

	qpt->common.queue_ext.msi_x_id = msi_x_id;

	return 0;
}


/*
 *	queue_associate_bpool
 *
 *	This function associate queue with target queue Id
 *
 *	@param[in]	mqa - pointer to MQA object
 *	@param[in]	queue_id - queue Id
 *	@param[in]	bpool_num - number of buffer pools
 *	@param[in]	bpool_qid_list - array of buffer pools queue Id
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int queue_associate_bpool(struct mqa *mqa, u32 queue_id, u32 bpool_num, u32 *bpool_qid_list)
{
	struct mqa_qpt_entry *qpt;

	/* Validate queue */
	if (INVALID_QUEUE(queue_id)) {
		pr_err("Failed to associate queue, invalid MQA queue %d\n", queue_id);
		return -EINVAL;
	}

	/* Validate region */
	if (UN_INITIALIZED_REGION(mqa_queue_state[queue_id])) {
		pr_err("Failed to associate queue, invalid MQA region\n");
		return -EINVAL;
	}

	/* Validate input */
	if (bpool_num >= MQA_BM_QUEUE_ARRAY) {
		pr_err("Failed to associate queue, invalid number of bpools %d\n", bpool_num);
		return -EINVAL;
	}

	pr_debug("Associate Queue - %d, Num of Bpools - %d\n", queue_id, bpool_num);

	/* Calculate queue entries in MQA tables */
	qpt = ((struct mqa_qpt_entry *)mqa->qpt_base) + queue_id;

	memcpy(&(qpt->common.queue_ext.bm_queue[0]), bpool_qid_list, (bpool_num * sizeof(u32)));

	return 0;
}


/*
 *	queue_dbg_globals_dump
 *
 *	This function print MQA table information
 *
 *	@param[in]	mqa - pointer to MQA object
 */
static void queue_dbg_globals_dump(struct mqa *mqa)
{
	/* Avoid compilation error */
	mqa = mqa;

	pr_debug("QPT Base   0x%lx\n", (u64)mqa->qpt_base);
	pr_debug("QCT Base   0x%lx\n", (u64)mqa->qct_base);
	pr_debug("QNPT Base  0x%lx\n", (u64)mqa->qnpt_base);
	pr_debug("QNCT Base  0x%lx\n", (u64)mqa->qnct_base);
	pr_debug("Table Size 0x%x\n\n", mqa->size);
}


/*
 *	queue_dbg_region_dump
 *
 *	This function print MQA region specific information
 *
 *	@param[in]	region - pointer to MQA region object
 */
static void queue_dbg_region_dump(struct mqa_region_params *region)
{
	/* Avoid compilation error */
	region = region;

	pr_debug("Region Id    0x%lx\n", (u64)region->region_id);
	pr_debug("Alloc count  0x%lx\n", (u64)region->queue_alloc_count);
	pr_debug("Free_index   0x%lx\n", (u64)region->queue_free_index);
}


/*
 *	queue_dbg_queue_dump
 *
 *	This function print MQA queue information
 *
 *	@param[in]	region - pointer to MQA region object
 *	@param[in]	queue_id - queue Id
 */
static void queue_dbg_queue_dump(struct mqa_region_params *region, u32 queue_id)
{
	/* Avoid compilation error */
	region = region;
	queue_id = queue_id;

	pr_debug("MQA Queue - %d\n", queue_id);
	pr_debug("Queue state       %s\n", (mqa_queue_state[queue_id] != MQA_FREE_QUEUE) ? ("occupy") : ("free"));
	pr_debug("Next free entry   %d\n", region->queue_free_index);
	pr_debug("Alloc queue count %d\n\n", region->queue_alloc_count);
}


/*
 *	queue_dbg_table_entry_dump
 *
 *	This function print MQA queue information
 *
 *  @param[in]	queue_id - queue Id
 */
static void queue_dbg_table_entry_dump(struct mqa *mqa, u32 queue_id)
{
	struct mqa_qpt_entry *qpt;

	/* Calculate queue entries in MQA tables */
	qpt = ((struct mqa_qpt_entry *)mqa->qpt_base) + queue_id;

	/* Avoid compilation error */
	mqa = mqa;
	queue_id = queue_id;
	qpt = qpt;

	pr_debug("MQA Queue      %d\n", queue_id);
	pr_debug("Ring_phy_addr  0x%lx\n", qpt->common.ring_phy_addr);
	pr_debug("Ring_virt_addr 0x%lx\n", qpt->common.ring_virt_addr);
	pr_debug("Ring_size      %d\n", qpt->common.ring_size);
	pr_debug("Entry_size     %d\n",  qpt->common.entry_size);
	pr_debug("Consumer_phys  0x%lx\n", qpt->common.cons_phys);
	pr_debug("Producer_phys  0x%lx\n", qpt->common.prod_phys);
	pr_debug("Consumer_virt  0x%lx\n", qpt->common.cons_virt);
	pr_debug("Producer_virt  0x%lx\n", qpt->common.prod_virt);
	pr_debug("Queue flags    0x%x\n", qpt->common.flags);
}

