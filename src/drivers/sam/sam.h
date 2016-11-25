/******************************************************************************
 *	Copyright (C) 2016 Marvell International Ltd.
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

#ifndef _SAM_H_
#define _SAM_H_

#include <drivers/mv_sam.h>
#include "std_internal.h"

#include "cs_driver.h"
#include "api_pec.h"
#include "api_dmabuf.h"
#include "api_driver197_init.h"
#include "sa_builder.h"
#include "sa_builder_basic.h"
#include "token_builder.h"
#include "firmware_eip207_api_cmd.h"

#define SAM_DMABUF_ALIGN	4 /* cache line */

#define SAM_DMA_BANK_PKT	0 /* dynamic bank */

#define SAM_DMA_BANK_SA		1 /* static bank */


#define SAM_HW_RING_NUM		DRIVER_MAX_NOF_RING_TO_USE
#define SAM_HW_RING_SIZE	DRIVER_PEC_MAX_PACKETS
#define SAM_HW_SA_NUM		DRIVER_PEC_MAX_SAS

#define SAM_HW_RING_RETRY_COUNT	(1000)
#define SAM_HW_RING_RETRY_US	(10)

#define SAM_AAD_IN_TOKEN_MAX_SIZE	(64)

/* max token size in bytes */
#define SAM_TOKEN_DMABUF_SIZE		(64 * 4)

/* max SA buffer size in bytes */
#define SAM_SA_DMABUF_SIZE		(64 * 4)

/* max TCR data size in bytes */
#define SAM_TCR_DATA_SIZE		(9 * 4)

struct sam_dmabuf {
	u32                     size;
	DMABuf_Handle_t		hndl;
	DMABuf_HostAddress_t	host_addr;
};

struct sam_cio_op {
	bool is_valid;
	struct sam_sa *sa;
	u32  num_bufs;        /* number of output buffers */
	struct sam_buf_info out_frags[SAM_CIO_MAX_FRAGS]; /* array of output buffers */
	u32  auth_icv_offset; /* offset of ICV in the buffer (in bytes) */
	void *cookie;
	struct sam_dmabuf token_dmabuf; /* DMA buffer for token  */
	struct sam_dmabuf data_dmabuf;  /* DMA buffer for data */
};


struct sam_cio {
	struct sam_cio_params params;
	struct sam_cio_op *operations;    /* array of operations */
	u32 next_request;
	u32 next_result;
};

struct sam_sa {
	bool is_valid;
	struct sam_session_params	params;
	struct sam_cio			*cio;
	/* Fields needed for EIP197 HW */
	SABuilder_Params_Basic_t	basic_params;
	SABuilder_Params_t		sa_params;
	struct sam_dmabuf		sa_dmabuf;
	u32				sa_words;
	u8				tcr_data[SAM_TCR_DATA_SIZE];
	u32				tcr_words;
	u32				token_words;
};

static inline u32 sam_cio_next_idx(struct sam_cio *cio, u32 idx)
{
	idx++;
	if (idx == cio->params.size)
		idx = 0;

	return idx;
}

static inline u32 sam_cio_prev_idx(struct sam_cio *cio, u32 idx)
{
	if (idx == 0)
		idx = cio->params.size - 1;
	else
		idx--;

	return idx;
}

/* next_request + 1 == next_result -> Full */
static inline bool sam_cio_is_full(struct sam_cio *cio)
{
	return (sam_cio_next_idx(cio, cio->next_request) == cio->next_result);
}

/* next_request == next_result -> Empty */
static inline bool sam_cio_is_empty(struct sam_cio *cio)
{
	return (cio->next_request == cio->next_result);
}

static inline int sam_max_check(int value, int limit, const char *name)
{
	if ((value < 0) || (value >= limit)) {
		pr_err("%s %d is out of range [0..%d]\n",
			name ? name : "value", value, (limit - 1));
		return 1;
	}
	return 0;
}

/* Debug functions */
void print_sa_params(SABuilder_Params_t *params);

void print_basic_sa_params(SABuilder_Params_Basic_t *params);

void print_cmd_desc(PEC_CommandDescriptor_t *desc);

void print_result_desc(PEC_ResultDescriptor_t *desc);

void print_pkt_params(PEC_PacketParams_t *pkt);

void print_token_params(TokenBuilder_Params_t *token);

void print_sam_cio_op_params(struct sam_cio_op_params *request);
void print_sam_sa_params(struct sam_session_params *sa_params);

#endif /* _SAM_H_ */
