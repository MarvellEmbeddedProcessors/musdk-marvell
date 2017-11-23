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

#define log_fmt(fmt) "pf_topoogy: " fmt

#include "std_internal.h"
#include "env/trace/trc_pf.h"
#include "mng/db.h"
#include "pf.h"
#include "pf_topology.h"

struct nic_pf *nic_pf;


int pf_topology_init(struct nmp *nmp)
{
	int ret;

	nic_pf = &(nmp->nic_pf);

	/* Initialize the NIC-PF */
	ret = nic_pf_init(&nmp->nic_pf);
	if (ret)
		return ret;

	return 0;
}

int pf_topology_terminate(struct nmp *nmp)
{
	int ret;

	/* Initialize the NIC-PF */
	ret = nic_pf_terminate(&nmp->nic_pf);
	if (ret)
		return ret;

	return 0;
}

/*
 *	tc_queue_get
 *
 *	This function return TC params object pointer from queue topology
 *
 *	@param[in]	tc_type - traffic class type
 *
 *	@retval	pointer to TC params object on success
 *	@retval	NULL otherwise
 */
static struct tc_params **tc_queue_get(u32 tc_type)
{
	struct giu_queue_topology *q_top = &(nic_pf->topology_data);

	switch (tc_type) {
	case LCL_EG_TC:
		return &(q_top->lcl_eg_tcs);

	case LCL_ING_TC:
		return &(q_top->lcl_ing_tcs);

	case HOST_EG_TC:
		return &(q_top->host_eg_tcs);

	case HOST_ING_TC:
		return &(q_top->host_ing_tcs);
	}

	return NULL;
}

/*
 *	tc_queue_num_set
 *
 *	This function set TC num in queue topology
 *
 *	@param[in]	tc_type - traffic class type
 *	@param[in]	tc_num - number of traffic classes
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
static int tc_queue_num_set(u32 tc_type, u32 tc_num)
{
	struct giu_queue_topology *q_top = &(nic_pf->topology_data);

	switch (tc_type) {
	case LCL_EG_TC:
		q_top->lcl_eg_tcs_num = tc_num;
		break;

	case LCL_ING_TC:
		q_top->lcl_ing_tcs_num = tc_num;
		break;

	case HOST_EG_TC:
		q_top->host_eg_tcs_num = tc_num;
		break;

	case HOST_ING_TC:
		q_top->host_ing_tcs_num = tc_num;
		break;
	}

	return 0;
}

/*
 *	pf_tc_queue_init
 *
 *	This function initilaize TC params object in queue topology
 *
 *	@param[in]	tc_type - traffic class type
 *	@param[in]	tc_num - number of traffic classes
 *	@param[in]	q_num - number of queues in traffic class
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int pf_tc_queue_init(u32 tc_type, u32 tc_num, u32 q_num)
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

	tc_p = tc_queue_get(tc_type);
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
			tc[tc_idx].tc_queue_params = kcalloc(q_num, sizeof(struct giu_gpio_q), GFP_KERNEL);
			if (tc[tc_idx].tc_queue_params == NULL)
				goto tc_error;
		}

		pr_debug("TC[%d], Num of queues %d, Q-Array addr - %p\n",
				tc_idx, tc[tc_idx].num_of_queues, tc[tc_idx].tc_queue_params);
	}

	ret = tc_queue_num_set(tc_type, tc_num);
	if (ret)
		return ret;

	return 0;

tc_error:

	for (tc_idx = 0; tc_idx < tc_num; tc_idx++) {
		if (tc[tc_idx].tc_queue_params != NULL)
			kfree(tc[tc_idx].tc_queue_params);
	}

	kfree(tc);

	return -ENOMEM;
}

/*
 *	pf_tc_queue_free
 *
 *	This function release TC params object in queue topology
 *
 *	@param[in]	tc_type - traffic class type
 *	@param[in]	tc_num - number of traffic classes
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int pf_tc_queue_free(u32 tc_type, u32 tc_num)
{
	u32 tc_idx;
	struct tc_params *tc;
	struct tc_params **tc_p;

	tc_p = tc_queue_get(tc_type);
	tc = *tc_p;
	if (tc == NULL)
		return -ENOENT;

	for (tc_idx = 0; tc_idx < tc_num; tc_idx++) {
		if (tc[tc_idx].tc_queue_params != NULL)
			kfree(tc[tc_idx].tc_queue_params);
	}

	kfree(tc);

	return -ENOMEM;
}


/*
 *	pf_bm_queue_get
 *
 *	This function return BM params object pointer from queue topology
 *
 *	@param[in]	bm_type - buffer pool type
 *
 *	@retval	pointer to BM params object on success
 *	@retval	NULL otherwise
 */
static struct giu_gpio_q **pf_bm_queue_get(u32 bm_type)
{
	struct giu_queue_topology *q_top = &(nic_pf->topology_data);

	switch (bm_type) {
	case LCL_BM:
		return &(q_top->lcl_bm_qs_params);

	case HOST_BM:
		return &(q_top->host_bm_qs_params);
	}

	return NULL;
}

/*
 *	pf_bm_queue_init
 *
 *	This function initilaize BM params object in queue topology
 *
 *	@param[in]	bm_type - buffer pool type
 *	@param[in]	bm_num - number of buffer pools
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int pf_bm_queue_init(u32 bm_type, u32 bm_num)
{
	struct giu_gpio_q **bm_p;
	struct giu_gpio_q *bm;
	struct giu_queue_topology *q_top = &(nic_pf->topology_data);

	pr_debug("Initializing DB BM type - %d\n", bm_type);
	pr_debug("Num of BM - %d\n", bm_num);

	bm = kcalloc(bm_num, sizeof(struct giu_gpio_q), GFP_KERNEL);
	if (bm == NULL)
		return -ENOMEM;

	bm_p = pf_bm_queue_get(bm_type);
	*bm_p = bm;
	pr_debug("BMs Array addr - %p\n", *bm_p);

	/* Setting number of bm queues is relevant for local queues
	 * remote queues are update upon message arrival
	 */
	if (bm_type == LCL_BM)
		q_top->lcl_bm_qs_num = bm_num;

	return 0;
}

/*
 *	pf_bm_queue_free
 *
 *	This function release BM params object in queue topology database
 *
 *	@param[in]	bm_type - buffer pool type
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int pf_bm_queue_free(u32 bm_type)
{
	struct giu_gpio_q **bm_p;

	bm_p = pf_bm_queue_get(bm_type);
	if (*bm_p == NULL)
		return -ENOENT;

	kfree(*bm_p);

	return 0;
}

