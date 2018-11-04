/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#ifndef _PF_TOPOLOGY_H
#define _PF_TOPOLOGY_H

#include "std_internal.h"
#include "mng/db.h"
#include "pf.h"

int pf_outtc_queue_init(struct nmnicpf *nmnicpf, u32 type, u32 tc_num, u32 q_num);

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
int pf_intc_queue_init(struct nmnicpf *nmnicpf, u32 type, u32 tc_num, u32 q_num);

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
int pf_outtc_queue_free(struct nmnicpf *nmnicpf, u32 type, u32 tc_num);

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
int pf_intc_queue_free(struct nmnicpf *nmnicpf, u32 type, u32 tc_num);

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
int pf_intc_bm_queue_init(struct nmnicpf *nmnicpf, u32 bm_num);

/*
 *	pf_intc_bm_queue_free
 *
 *	This function release BM params object in queue topology
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int pf_intc_bm_queue_free(struct nmnicpf *nmnicpf);

#endif /* _PF_TOPOLOGY_H */
