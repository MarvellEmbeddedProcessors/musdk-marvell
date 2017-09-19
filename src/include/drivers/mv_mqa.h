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

#ifndef _MV_MQA_H
#define _MV_MQA_H

#include "mv_std.h"

#define MQA_QUEUE_MAX		(1024)	/** Max number of queues in MQA */
#define MQA_BM_QUEUE_ARRAY	(3)	/** Number of BM pools associated with data queue */

struct mqa;

struct notif_tbl_params {
	void		*qnpt_va;
	phys_addr_t	 qnpt_pa;
	void		*qnct_va;
	phys_addr_t	 qnct_pa;

};

struct mqa_params {
	char *match;
	u16  num_qs;
	struct notif_tbl_params notif_tbl;

};

/**
 *	Initialize MQA infrastructure.
 *	MQA tables - GPT, GCT, GNPT, and GNCT are allocated and initialized.
 *	MQA region map is statically initialized and divide MQA tables into dedicated regions.
 *
 *	@param[in]	params	A pointer to MQA parameters
 *	@param[in]	mqa	A pointer to MQA object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int mqa_init(struct mqa_params *params, struct mqa **mqa);

/**
 *	Release MQA tables - GPT, GCT, GNPT, and GNCT.
 *	Clear MQA Region tables.
 *
 *	@param[in]	mqa	A pointer to MQA object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int mqa_deinit(struct mqa *mqa);

#endif /* _MV_MQA_H */



