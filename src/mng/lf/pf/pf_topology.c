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
 *	pf_outtc_queue_init
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
int pf_outtc_queue_init(u32 type, u32 tc_num, u32 q_num)
{
	u32 tc_idx;
	struct giu_gpio_outtc_params *outtc_p;
	struct giu_gpio_init_params *q_top = &(nic_pf->topology_data);

	if (q_top->outtcs_params.outtc_params == NULL) {
		q_top->outtcs_params.outtc_params = kcalloc(tc_num, sizeof(struct giu_gpio_outtc_params), GFP_KERNEL);
		if (q_top->outtcs_params.outtc_params == NULL)
			return -ENOMEM;
	}

	outtc_p = &(q_top->outtcs_params.outtc_params[0]);

	for (tc_idx = 0; tc_idx < tc_num; tc_idx++) {

		if (type == LCL) {
			outtc_p[tc_idx].tc_id = tc_idx;
			outtc_p[tc_idx].num_outqs = q_num;
			/*outtc_p[tc_idx].rss_type = 0; *//* Not used */
			/* TC init for Host queues is based in PF_INIT parameters which include
			 * Ingress / Egress TC number, but does not include queue number per TC
			 * Therefore q_num == 0, is valid
			 */
			if (q_num != 0) {
				outtc_p[tc_idx].outqs_params =
						kcalloc(q_num, sizeof(union giu_gpio_q_params), GFP_KERNEL);
				if (outtc_p[tc_idx].outqs_params == NULL)
					goto tc_error;

				pr_info("Out TC[%d]\n"
						"Num of output queues %d, Q-Array addr - %p\n",
						tc_idx, outtc_p[tc_idx].num_outqs, outtc_p[tc_idx].outqs_params);
			}
		} else if (type == REM) {

			outtc_p[tc_idx].num_rem_inqs = q_num;
			if (q_num != 0) {
				outtc_p[tc_idx].rem_inqs_params =
						kcalloc(q_num, sizeof(union giu_gpio_q_params), GFP_KERNEL);
				if (outtc_p[tc_idx].rem_inqs_params == NULL)
					goto tc_error;

				pr_info("Out TC[%d]\n"
						"Num of remote input queues %d, Q-Array addr - %p\n",
						tc_idx, outtc_p[tc_idx].num_rem_inqs, outtc_p[tc_idx].rem_inqs_params);
			}
		}
	}

	return 0;

tc_error:

	for (tc_idx = 0; tc_idx < tc_num; tc_idx++) {
		if (type == LCL) {
			if (outtc_p[tc_idx].outqs_params != NULL)
				kfree(outtc_p[tc_idx].outqs_params);
		} else if (type == REM) {
			if (outtc_p[tc_idx].rem_inqs_params != NULL)
				kfree(outtc_p[tc_idx].rem_inqs_params);
		}
	}

	kfree(outtc_p);

	return -ENOMEM;
}

/*
 *	pf_intc_queue_init
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
int pf_intc_queue_init(u32 type, u32 tc_num, u32 q_num)
{
	u32 tc_idx;
	struct giu_gpio_intc_params *intc_p;
	struct giu_gpio_init_params *q_top = &(nic_pf->topology_data);

	if (q_top->intcs_params.intc_params == NULL) {
		q_top->intcs_params.intc_params =
				kcalloc(tc_num, sizeof(struct giu_gpio_intc_params), GFP_KERNEL);
		if (q_top->intcs_params.intc_params == NULL)
			return -ENOMEM;
	}

	intc_p = &(q_top->intcs_params.intc_params[0]);

	for (tc_idx = 0; tc_idx < tc_num; tc_idx++) {

		if (type == LCL) {
			intc_p[tc_idx].tc_id = tc_idx;
			intc_p[tc_idx].num_inqs = q_num;
			/*intc_p[tc_idx].rss_type = 0; *//* Not used */
			/* TC init for Host queues is based in PF_INIT parameters which include
			 * Ingress / Egress TC number, but does not include queue number per TC
			 * Therefore q_num == 0, is valid
			 */
			if (q_num != 0) {
				intc_p[tc_idx].inqs_params =
						kcalloc(q_num, sizeof(union giu_gpio_q_params), GFP_KERNEL);
				if (intc_p[tc_idx].inqs_params == NULL)
					goto tc_error;

				pr_debug("In TC[%d]\n"
						 "Num of input queues %d, Q-Array addr - %p\n",
						 tc_idx, intc_p[tc_idx].num_inqs, intc_p[tc_idx].inqs_params);
			}
		} else if (type == REM) {
			intc_p[tc_idx].num_rem_outqs = q_num;

			if (q_num != 0) {
				intc_p[tc_idx].rem_outqs_params =
						kcalloc(q_num, sizeof(union giu_gpio_q_params), GFP_KERNEL);
				if (intc_p[tc_idx].rem_outqs_params == NULL)
					goto tc_error;

				pr_debug("In TC[%d]\n"
						 "Num of remote output queues %d, Q-Array addr - %p\n",
						 tc_idx, intc_p[tc_idx].num_rem_outqs, intc_p[tc_idx].rem_outqs_params);
			}
		}
	}

	return 0;

tc_error:

	for (tc_idx = 0; tc_idx < tc_num; tc_idx++) {
		if (type == LCL) {
			if (intc_p[tc_idx].inqs_params != NULL)
				kfree(intc_p[tc_idx].inqs_params);
		} else if (type == REM) {
			if (intc_p[tc_idx].rem_outqs_params != NULL)
				kfree(intc_p[tc_idx].rem_outqs_params);
		}
	}

	kfree(intc_p);

	return -ENOMEM;
}



/*
 *	pf_outtc_queue_free
 *
 *	This function release TC params object in queue topology
 *
 *	@param[in]	tc_type - traffic class type
 *	@param[in]	tc_num - number of traffic classes
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int pf_outtc_queue_free(u32 type, u32 tc_num)
{
	u32 tc_idx;
	static u32 clear_outtc;

	struct giu_gpio_init_params *q_top = &(nic_pf->topology_data);
	struct giu_gpio_outtc_params *outtc_p = q_top->outtcs_params.outtc_params;

	for (tc_idx = 0; tc_idx < tc_num; tc_idx++) {
		if (type == LCL) {
			if (outtc_p[tc_idx].outqs_params != NULL) {
				kfree(outtc_p[tc_idx].outqs_params);
				clear_outtc++;
			}
		} else if (type == REM) {
			if (outtc_p[tc_idx].rem_inqs_params != NULL) {
				kfree(outtc_p[tc_idx].rem_inqs_params);
				clear_outtc++;
			}
		}
	}

	if (clear_outtc >= 2) {
		kfree(outtc_p);
		clear_outtc = 0;
	}


	return 0;
}


/*
 *	pf_intc_queue_free
 *
 *	This function release TC params object in queue topology
 *
 *	@param[in]	tc_type - traffic class type
 *	@param[in]	tc_num - number of traffic classes
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int pf_intc_queue_free(u32 type, u32 tc_num)
{
	u32 tc_idx;
	static u32 clear_intc;

	struct giu_gpio_init_params *q_top = &(nic_pf->topology_data);
	struct giu_gpio_intc_params *intc_p = q_top->intcs_params.intc_params;

	for (tc_idx = 0; tc_idx < tc_num; tc_idx++) {
		if (type == LCL) {
			if (intc_p[tc_idx].inqs_params != NULL) {
				kfree(intc_p[tc_idx].inqs_params);
				clear_intc++;
			}
		} else if (type == REM) {
			if (intc_p[tc_idx].rem_outqs_params != NULL) {
				kfree(intc_p[tc_idx].rem_outqs_params);
				clear_intc++;
			}
		}
	}

	if (clear_intc >= 2) {
		kfree(intc_p);
		clear_intc = 0;
	}

	return 0;
}


/*
 *	pf_outtc_bm_queue_init
 *
 *	This function initilaize BM params object in queue topology
 *
 *	@param[in]	bm_type - buffer pool type
 *	@param[in]	bm_num - number of buffer pools
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int pf_outtc_bm_queue_init(u32 bm_num)
{
	u32 tc_idx;
	struct giu_gpio_init_params *q_top = &(nic_pf->topology_data);
	struct giu_gpio_outtc_params *outtc_p = q_top->outtcs_params.outtc_params;

	for (tc_idx = 0; tc_idx < q_top->outtcs_params.num_outtcs; tc_idx++) {

		outtc_p[tc_idx].host_bm_qs_num = bm_num;
		if (bm_num != 0) {
			outtc_p[tc_idx].rem_poolqs_params =
						kcalloc(bm_num, sizeof(union giu_gpio_q_params), GFP_KERNEL);
			if (outtc_p[tc_idx].rem_poolqs_params == NULL)
				goto bm_error;
		}
	}

	return 0;

bm_error:

	for (tc_idx = 0; tc_idx < q_top->outtcs_params.num_outtcs; tc_idx++) {
		if (outtc_p[tc_idx].rem_poolqs_params != NULL)
			kfree(outtc_p[tc_idx].rem_poolqs_params);
	}

	return -ENOMEM;
}


/*
 *	pf_intc_bm_queue_init
 *
 *	This function initilaize BM params object in queue topology
 *
 *	@param[in]	bm_type - buffer pool type
 *	@param[in]	bm_num - number of buffer pools
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int pf_intc_bm_queue_init(u32 bm_num)
{
	u32 tc_idx;

	struct giu_gpio_init_params *q_top = &(nic_pf->topology_data);
	struct giu_gpio_intc_params *intc_p = q_top->intcs_params.intc_params;

	for (tc_idx = 0; tc_idx < q_top->intcs_params.num_intcs; tc_idx++) {

		intc_p[tc_idx].num_inpools = bm_num;
		if (bm_num != 0) {
			intc_p[tc_idx].pools =
						kcalloc(bm_num, sizeof(union giu_gpio_q_params), GFP_KERNEL);
			if (intc_p[tc_idx].pools == NULL)
				goto bm_error;
		}
	}

	return 0;

bm_error:

	for (tc_idx = 0; tc_idx < q_top->intcs_params.num_intcs; tc_idx++) {
		if (intc_p[tc_idx].pools != NULL)
			kfree(intc_p[tc_idx].pools);
	}

	return -ENOMEM;
}


/*
 *	pf_outtc_bm_queue_free
 *
 *	This function release BM params object in queue topology
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int pf_outtc_bm_queue_free(void)
{
	u32 tc_idx;
	struct giu_gpio_init_params *q_top = &(nic_pf->topology_data);
	struct giu_gpio_outtc_params *outtc_p = q_top->outtcs_params.outtc_params;

	for (tc_idx = 0; tc_idx < q_top->outtcs_params.num_outtcs; tc_idx++) {
		if (outtc_p[tc_idx].rem_poolqs_params != NULL)
			kfree(outtc_p[tc_idx].rem_poolqs_params);
	}

	return 0;
}


/*
 *	pf_intc_bm_queue_free
 *
 *	This function release BM params object in queue topology
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int pf_intc_bm_queue_free(void)
{
	u32 tc_idx;

	struct giu_gpio_init_params *q_top = &(nic_pf->topology_data);
	struct giu_gpio_intc_params *intc_p = q_top->intcs_params.intc_params;

	for (tc_idx = 0; tc_idx < q_top->intcs_params.num_intcs; tc_idx++) {
		if (intc_p[tc_idx].pools != NULL)
			kfree(intc_p[tc_idx].pools);
	}

	return 0;
}


