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

#define log_fmt(fmt) "db: " fmt

#include "std_internal.h"
#include "env/trace/trc_pf.h"
#include "db.h"

struct nic_pf *nic_pf;
struct db_q *pf_giu_queue_table;


int db_init(struct nmp *nmp)
{
	u32 q_idx;

	nic_pf = &(nmp->nic_pf);

	/* Allocate queue batabase */
	pf_giu_queue_table = kmalloc((MQA_QUEUE_MAX * (sizeof(struct db_q))), GFP_KERNEL);

	/* Clear queue batabase */
	memset(pf_giu_queue_table, 0, MQA_QUEUE_MAX * (sizeof(struct db_q)));

	/* Mark queue as free in batabase */
	for (q_idx = 0; q_idx < MQA_QUEUE_MAX; q_idx++)
		pf_giu_queue_table[q_idx].params.idx = QUEUE_FREE_STATUS;

	/* Clear queue topology batabase */
	memset(&(nic_pf->topology_data), 0, sizeof(struct pf_queue_topology));

	return 0;
}


/*
 *	db_queue_set
 *
 *	This function sets queue parameters in snic database
 *
 *	@param[in]	queue_Id - queue identifier
 *	@param[in]	db_q - pointer to db queue object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int db_queue_set(u32 queue_Id, struct db_q *db_q)
{
	if (queue_Id >= MQA_QUEUE_MAX) {
		pr_err("Failed to set queue parameters, invalid queue %d\n", queue_Id);
		return -EINVAL;
	}

	memcpy(&(pf_giu_queue_table[queue_Id]), db_q, sizeof(struct db_q));

	return 0;
}


/*
 *	db_queue_reset
 *
 *	This function resets queue parameters in snic database
 *
 *	@param[in]	queue_Id - queue identifier
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int db_queue_reset(u32 queue_Id)
{
	if (queue_Id >= MQA_QUEUE_MAX) {
		pr_err("Failed to reset queue parameters, invalid queue %d\n", queue_Id);
		return -EINVAL;
	}

	memset(&(pf_giu_queue_table[queue_Id]), 0, sizeof(struct db_q));

	return 0;
}


/*
 *	db_queue_get
 *
 *	This function retrieve queue parameters from snic database
 *
 *	@param[in]	queue_Id - queue identifier
 *	@param[out]	db_q - pointer to db queue object
 *
 *	@retval	pointer to queue parameters object on success
 *	@retval	NULL in case of an error
 */
struct db_q *db_queue_get(u32 queue_Id)
{
	if (queue_Id >= MQA_QUEUE_MAX) {
		pr_err("Failed to retrieve queue, invalid queue %d\n", queue_Id);
		return NULL;
	}

	return &(pf_giu_queue_table[queue_Id]);
}

/*
 *	db_queue_dump
 *
 *	This function print queue parameters
 *
 *	@param[in]	queue_Id - queue identifier
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int db_queue_dump(u32 queue_Id)
{
	struct mqa_queue_params *q;

	if (queue_Id >= MQA_QUEUE_MAX) {
		pr_err("Failed to dump queue parameters, invalid queue %d\n", queue_Id);
		return -EINVAL;
	}

	q = &(pf_giu_queue_table[queue_Id].params);

	q = q;

	pr_debug("Queue-%d Parameters\n", q->idx);
	pr_debug("len %d, size %d, attr %x, tc %d\n", q->len, q->size, q->attr, q->tc);
	pr_debug("phy %p, virt %p, prod_p %p, cons_p %p, prod_c %p, cons_c %p, rem_p %p host %p\n",
			q->phy_base_addr, q->virt_base_addr, q->prod_phys, q->cons_phys, q->prod_virt,
			q->cons_virt, q->remote_phy_addr, q->host_remap);
	pr_debug("msix %d, msi %d, peer %d, bpool_num %d copy_payload %d\n",
			q->msix_id, q->msi_id, q->peer_id, q->bpool_num, q->copy_payload);
	pr_debug("bpool_qids %d, %d, %d\n", q->bpool_qids[0], q->bpool_qids[1], q->bpool_qids[2]);

	return 0;
}

/*
 *	db_tc_get
 *
 *	This function return TC params object pointer from queue topology database
 *
 *	@param[in]	tc_type - traffic class type
 *
 *	@retval	pointer to TC params object on success
 *	@retval	NULL otherwise
 */
static struct tc_params **db_tc_get(u32 tc_type)
{
	struct pf_queue_topology *q_top = &(nic_pf->topology_data);

	switch (tc_type) {
	case DB_LCL_EG_TC:
		return &(q_top->lcl_eg_tcs);

	case DB_LCL_ING_TC:
		return &(q_top->lcl_ing_tcs);

	case DB_HOST_EG_TC:
		return &(q_top->host_eg_tcs);

	case DB_HOST_ING_TC:
		return &(q_top->host_ing_tcs);
	}

	return NULL;
}

/*
 *	db_tc_num_set
 *
 *	This function set TC num in queue topology database
 *
 *	@param[in]	tc_type - traffic class type
 *	@param[in]	tc_num - number of traffic classes
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
static int db_tc_num_set(u32 tc_type, u32 tc_num)
{
	struct pf_queue_topology *q_top = &(nic_pf->topology_data);

	switch (tc_type) {
	case DB_LCL_EG_TC:
		q_top->lcl_eg_tcs_num = tc_num;
		break;

	case DB_LCL_ING_TC:
		q_top->lcl_ing_tcs_num = tc_num;
		break;

	case DB_HOST_EG_TC:
		q_top->host_eg_tcs_num = tc_num;
		break;

	case DB_HOST_ING_TC:
		q_top->host_ing_tcs_num = tc_num;
		break;
	}

	return 0;
}

/*
 *	db_tc_init
 *
 *	This function initilaize TC params object in queue topology database
 *
 *	@param[in]	tc_type - traffic class type
 *	@param[in]	tc_num - number of traffic classes
 *	@param[in]	q_num - number of queues in traffic class
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int db_tc_init(u32 tc_type, u32 tc_num, u32 q_num)
{
	int ret;
	u32 tc_idx;
	struct tc_params **tc_p;
	struct tc_params *tc;

	pr_debug("Initializing DB TC type - %d\n", tc_type);
	pr_debug("Num of TCs - %d, Num of queues - %d\n", tc_num, q_num);

	tc = kcalloc(tc_num, sizeof(struct tc_params), GFP_KERNEL);
	if (tc == NULL)
		return -ENOMEM;

	tc_p = db_tc_get(tc_type);
	*tc_p = tc;
	pr_debug("TCs Array addr - %p\n", *tc_p);

	for (tc_idx = 0; tc_idx < tc_num; tc_idx++) {

		tc[tc_idx].tc_id = tc_idx;
		tc[tc_idx].num_of_queues = q_num;
		tc[tc_idx].rss_type = 0; /* Not used */
		/* TC init for Host queues is based in PF_INIT parameters which include
		 * Ingress / Egress TC number, but does not include queue number per TC
		 * Therefore q_num == 0, is valid
		 */
		if (q_num != 0) {
			tc[tc_idx].tc_queues_idx = kcalloc(q_num, sizeof(u32), GFP_KERNEL);
			if (tc[tc_idx].tc_queues_idx == NULL)
				goto tc_error;
		}

		pr_debug("TC[%d], Num of queues %d, Q-Array addr - %p\n",
				tc_idx, tc[tc_idx].num_of_queues, tc[tc_idx].tc_queues_idx);
	}

	ret = db_tc_num_set(tc_type, tc_num);
	if (ret)
		return ret;

	return 0;

tc_error:

	for (tc_idx = 0; tc_idx < tc_num; tc_idx++) {
		if (tc[tc_idx].tc_queues_idx != NULL)
			kfree(tc[tc_idx].tc_queues_idx);
	}

	kfree(tc);

	return -ENOMEM;
}

/*
 *	db_tc_free
 *
 *	This function release TC params object in queue topology database
 *
 *	@param[in]	tc_type - traffic class type
 *	@param[in]	tc_num - number of traffic classes
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int db_tc_free(u32 tc_type, u32 tc_num)
{
	u32 tc_idx;
	struct tc_params *tc;
	struct tc_params **tc_p;

	tc_p = db_tc_get(tc_type);
	tc = *tc_p;
	if (tc == NULL)
		return -ENOENT;

	for (tc_idx = 0; tc_idx < tc_num; tc_idx++) {
		if (tc[tc_idx].tc_queues_idx != NULL)
			kfree(tc[tc_idx].tc_queues_idx);
	}

	kfree(tc);

	return -ENOMEM;
}


/*
 *	db_bm_get
 *
 *	This function return BM params object pointer from queue topology database
 *
 *	@param[in]	bm_type - buffer pool type
 *
 *	@retval	pointer to BM params object on success
 *	@retval	NULL otherwise
 */
static u32 **db_bm_get(u32 bm_type)
{
	struct pf_queue_topology *q_top = &(nic_pf->topology_data);

	switch (bm_type) {
	case DB_LCL_BM:
		return &(q_top->lcl_bm_qs_idx);

	case DB_HOST_BM:
		return &(q_top->host_bm_qs_idx);
	}

	return NULL;
}

/*
 *	db_bm_init
 *
 *	This function initilaize BM params object in queue topology database
 *
 *	@param[in]	bm_type - buffer pool type
 *	@param[in]	bm_num - number of buffer pools
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int db_bm_init(u32 bm_type, u32 bm_num)
{
	u32 **bm_p;
	u32 *bm;
	struct pf_queue_topology *q_top = &(nic_pf->topology_data);

	pr_debug("Initializing DB BM type - %d\n", bm_type);
	pr_debug("Num of BM - %d\n", bm_num);

	bm = kcalloc(bm_num, sizeof(u32), GFP_KERNEL);
	if (bm == NULL)
		return -ENOMEM;

	bm_p = db_bm_get(bm_type);
	*bm_p = bm;
	pr_debug("BMs Array addr - %p\n", *bm_p);

	/* Setting number of bm queues is relevant for local queues
	 * remote queues are update upon message arrival
	 */
	if (bm_type == DB_LCL_BM)
		q_top->lcl_bm_qs_num = bm_num;

	return 0;
}

/*
 *	db_bm_free
 *
 *	This function release BM params object in queue topology database
 *
 *	@param[in]	bm_type - buffer pool type
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int db_bm_free(u32 bm_type)
{
	u32 **bm_p;

	bm_p = db_bm_get(bm_type);
	if (*bm_p == NULL)
		return -ENOENT;

	kfree(*bm_p);

	return 0;
}

