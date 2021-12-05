/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#define log_fmt(fmt, ...) "pf_topology#%d: " fmt, nmnicpf->pf_id, ##__VA_ARGS__

#include "std_internal.h"
#include "env/trace/trc_pf.h"

#include "pf_topology.h"


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
int pf_outtc_queue_init(struct nmnicpf *nmnicpf, u32 type, u32 tc_num, u32 q_num)
{
	u32				 tc_idx;

	for (tc_idx = 0; tc_idx < tc_num; tc_idx++) {
		if (type == LCL)
			nmnicpf->gpio_params.outtcs_params[tc_idx].num_outqs = q_num;
		else if (type == REM)
			nmnicpf->gpio_rem_params.outtcs_params[tc_idx].num_rem_inqs = q_num;
	}

	return 0;
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
int pf_intc_queue_init(struct nmnicpf *nmnicpf, u32 type, u32 tc_num, u32 q_num)
{
	u32				 tc_idx;

	for (tc_idx = 0; tc_idx < tc_num; tc_idx++) {
		if (type == LCL)
			nmnicpf->gpio_params.intcs_params[tc_idx].num_inqs = q_num;
		else if (type == REM)
			nmnicpf->gpio_rem_params.intcs_params[tc_idx].num_rem_outqs = q_num;
	}

	return 0;
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
int pf_outtc_queue_free(struct nmnicpf *nmnicpf, u32 type, u32 tc_num)
{
	u32				 tc_idx;

	for (tc_idx = 0; tc_idx < tc_num; tc_idx++) {
		if (type == LCL)
			nmnicpf->gpio_params.outtcs_params[tc_idx].num_outqs = 0;
		else if (type == REM)
			nmnicpf->gpio_rem_params.outtcs_params[tc_idx].num_rem_inqs = 0;
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
int pf_intc_queue_free(struct nmnicpf *nmnicpf, u32 type, u32 tc_num)
{
	u32				 tc_idx;

	for (tc_idx = 0; tc_idx < tc_num; tc_idx++) {
		if (type == LCL)
			nmnicpf->gpio_params.intcs_params[tc_idx].num_inqs = 0;
		else if (type == REM)
			nmnicpf->gpio_rem_params.intcs_params[tc_idx].num_rem_outqs = 0;
	}

	return 0;
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
int pf_intc_bm_queue_init(struct nmnicpf *nmnicpf, u32 bm_num)
{
	u32 tc_idx;

	for (tc_idx = 0; tc_idx < nmnicpf->gpio_params.num_intcs; tc_idx++)
		nmnicpf->gpio_params.intcs_params[tc_idx].num_inpools = bm_num;

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
int pf_intc_bm_queue_free(struct nmnicpf *nmnicpf)
{
	u32 tc_idx;

	for (tc_idx = 0; tc_idx < nmnicpf->gpio_params.num_intcs; tc_idx++)
		nmnicpf->gpio_params.intcs_params[tc_idx].num_inpools = 0;

	return 0;
}
