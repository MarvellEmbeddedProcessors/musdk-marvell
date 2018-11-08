/*******************************************************************************
 * Copyright (C) Marvell International Ltd. and its affiliates
 *
 * This software file (the "File") is owned and distributed by Marvell
 * International Ltd. and/or its affiliates ("Marvell") under the following
 * alternative licensing terms.  Once you have made an election to distribute the
 * File under one of the following license alternatives, please (i) delete this
 * introductory statement regarding license alternatives, (ii) delete the three
 * license alternatives that you have not elected to use and (iii) preserve the
 * Marvell copyright notice above.
 *
 ********************************************************************************
 * Marvell Commercial License Option
 *
 * If you received this File from Marvell and you have entered into a commercial
 * license agreement (a "Commercial License") with Marvell, the File is licensed
 * to you under the terms of the applicable Commercial License.
 *
 ********************************************************************************
 * Marvell GPL License Option
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File in accordance with the terms and conditions of the General
 * Public License Version 2, June 1991 (the "GPL License"), a copy of which is
 * available along with the File in the license.txt file or by writing to the Free
 * Software Foundation, Inc., or on the worldwide web at http://www.gnu.org/licenses/gpl.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY
 * DISCLAIMED.  The GPL License provides additional details about this warranty
 * disclaimer.
 *
 ********************************************************************************
 * Marvell GNU General Public License FreeRTOS Exception
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File in accordance with the terms and conditions of the Lesser
 * General Public License Version 2.1 plus the following FreeRTOS exception.
 * An independent module is a module which is not derived from or based on
 * FreeRTOS.
 * Clause 1:
 * Linking FreeRTOS statically or dynamically with other modules is making a
 * combined work based on FreeRTOS. Thus, the terms and conditions of the GNU
 * General Public License cover the whole combination.
 * As a special exception, the copyright holder of FreeRTOS gives you permission
 * to link FreeRTOS with independent modules that communicate with FreeRTOS solely
 * through the FreeRTOS API interface, regardless of the license terms of these
 * independent modules, and to copy and distribute the resulting combined work
 * under terms of your choice, provided that:
 * 1. Every copy of the combined work is accompanied by a written statement that
 * details to the recipient the version of FreeRTOS used and an offer by yourself
 * to provide the FreeRTOS source code (including any modifications you may have
 * made) should the recipient request it.
 * 2. The combined work is not itself an RTOS, scheduler, kernel or related
 * product.
 * 3. The independent modules add significant and primary functionality to
 * FreeRTOS and do not merely extend the existing functionality already present in
 * FreeRTOS.
 * Clause 2:
 * FreeRTOS may not be used for any competitive or comparative purpose, including
 * the publication of any form of run time or compile time metric, without the
 * express permission of Real Time Engineers Ltd. (this is the norm within the
 * industry and is intended to ensure information accuracy).
 *
 ********************************************************************************
 * Marvell BSD License Option
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File under the following licensing terms.
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *	* Redistributions of source code must retain the above copyright notice,
 *	  this list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the distribution.
 *
 *	* Neither the name of Marvell nor the names of its contributors may be
 *	  used to endorse or promote products derived from this software without
 *	  specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************/

#ifndef _SAM_H_
#define _SAM_H_

#include <drivers/mv_sam.h>
#include "std_internal.h"

#include "sa_builder.h"
#include "sa_builder_basic.h"
#include "sa_builder_ipsec.h"
#include "sa_builder_ssltls.h"
#include "token_builder.h"

#include "sam_hw.h"

/** Maximum number of supported crypto engines */
#define SAM_HW_DEVICE_NUM	        2

/** Maximum number of supported rings per crypto engines */
#define SAM_HW_RING_NUM			8

#define SAM_AAD_IN_TOKEN_MAX_SIZE	(64)

/* max token size in bytes */
#define SAM_TOKEN_DMABUF_SIZE		(64 * 4)

/* max SA buffer size in bytes */
#define SAM_SA_DMABUF_SIZE		(64 * 4)

/* max TCR data size in bytes */
#define SAM_TCR_DATA_SIZE		(9 * 4)

/* default packets ISR coalescing value */
#define SAM_ISR_PKTS_COAL_DEF		16

/* default time ISR coalescing value in usecs */
#define SAM_ISR_TIME_COAL_DEF		100

struct sam_cio_op {
	bool is_valid;
	struct sam_sa *sa;
	u32 num_bufs_in;     /* number of input buffers */
	u32 num_bufs_out;    /* number of output buffers */
	struct sam_buf_info out_frags[SAM_CIO_MAX_FRAGS]; /* array of output buffers */
	u32  auth_icv_offset; /* offset of ICV in the buffer (in bytes) */
	void *cookie;
	struct sam_buf_info token_buf; /* DMA buffer for token  */
	u32 token_header_word;
	u32 token_words;
	u32 copy_len;
};

struct sam_cio {
	u8  idx;			/* index in the sam_rings array */
	struct sam_cio_params params;
#ifdef MVCONF_SAM_STATS
	struct sam_cio_stats stats;	/* cio statistics */
#endif
	struct sam_cio_op *operations;	/* array of operations */
	struct sam_hw_ring hw_ring;
	u32 next_request;
	u32 next_result;
	u32 pkt_coal;
	u32 usec_coal;
};

struct sam_sa {
	bool is_valid;
	struct sam_session_params	params;
	struct sam_cio			*ctr_cio; /* control path cio */
	struct sam_cio			*cio;     /* data path cio */
	/* Fields needed for EIP197 HW */
	SABuilder_Params_t		sa_params;
	union {
		SABuilder_Params_Basic_t basic_params;
		SABuilder_Params_IPsec_t ipsec_params;
		SABuilder_Params_SSLTLS_t ssltls_params;
	} u;
	struct sam_buf_info		sa_buf;		/* DMA buffer for SA */
	u32				sa_words;
	u8				tcr_data[SAM_TCR_DATA_SIZE];
	u32				tcr_words;
	u32				token_words;
	u32				nonce;
	u8				auth_inner[64]; /* authentication inner block */
	u8				auth_outer[64]; /* authentication outer block */
	u8				tunnel_header[40]; /* Maximum needed place for tunnel header */
	void	(*post_proc_cb)(struct sam_cio_op *operation, struct sam_hw_res_desc *res_desc,
				struct sam_cio_op_result *result);
};

#ifdef MVCONF_SAM_STATS
#define SAM_STATS(c) c
#else
#define SAM_STATS(c)
#endif

#if __BYTE_ORDER == __BIG_ENDIAN
static inline void sam_htole32_multi(u32 *vaddr, u32 words)
{
	int i;
	u32 data;

	for (i = 0; i < words; i++) {
		data = *vaddr;
		*vaddr = htole32(data);
		vaddr++;
	}
}
#else
static inline void sam_htole32_multi(u32 *vaddr, u32 words)
{
}
#endif

static inline u32 sam_cio_next_idx(struct sam_cio *cio, u32 idx)
{
	idx++;
	if (unlikely(idx == cio->params.size))
		idx = 0;

	return idx;
}

static inline u32 sam_cio_prev_idx(struct sam_cio *cio, u32 idx)
{
	if (unlikely(idx == 0))
		idx = cio->params.size - 1;
	else
		idx--;

	return idx;
}

static inline bool sam_cio_is_free_slot(struct sam_cio *cio, u32 num_bufs)
{
	/* check free slot in operations array */
	if (sam_cio_next_idx(cio, cio->next_request) == cio->next_result)
		return false;
	/* check free slot in hw CDR/RDR */
	return sam_hw_cmd_is_free_slot(&cio->hw_ring, num_bufs);
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

extern u32 sam_debug_flags;

void sam_ipsec_prepare_tunnel_header(struct sam_session_params *params, u8 *tunnel_header);
void sam_ipsec_ip4_transport_in_post_proc(struct sam_cio_op *operation,
			struct sam_hw_res_desc *res_desc, struct sam_cio_op_result *result);
void sam_ipsec_ip6_transport_in_post_proc(struct sam_cio_op *operation,
			struct sam_hw_res_desc *res_desc, struct sam_cio_op_result *result);
void sam_ipsec_ip4_tunnel_in_post_proc(struct sam_cio_op *operation,
			struct sam_hw_res_desc *res_desc, struct sam_cio_op_result *result);
void sam_ipsec_ip6_tunnel_in_post_proc(struct sam_cio_op *operation,
			struct sam_hw_res_desc *res_desc, struct sam_cio_op_result *result);
void sam_ipsec_ip4_tunnel_out_post_proc(struct sam_cio_op *operation,
			struct sam_hw_res_desc *res_desc, struct sam_cio_op_result *result);
void sam_ipsec_ip6_tunnel_out_post_proc(struct sam_cio_op *operation,
			struct sam_hw_res_desc *res_desc, struct sam_cio_op_result *result);

u16 sam_ssltls_version_convert(enum sam_ssltls_version version);
void sam_dtls_ip4_post_proc(struct sam_cio_op *operation, struct sam_hw_res_desc *res_desc,
			       struct sam_cio_op_result *result);
void sam_dtls_ip6_post_proc(struct sam_cio_op *operation, struct sam_hw_res_desc *res_desc,
			       struct sam_cio_op_result *result);

/* Debug functions */
void print_sam_sa(struct sam_sa *session);
void print_sa_builder_params(struct sam_sa *session);

void print_token_params(TokenBuilder_Params_t *token);

void print_sam_cio_op_params(struct sam_cio_op_params *request);
void print_sam_cio_ipsec_params(struct sam_cio_ipsec_params *request);
void print_sam_cio_ssltls_params(struct sam_cio_ssltls_params *request);
void print_sam_sa_params(struct sam_session_params *sa_params);
void print_sam_cio_operation_info(struct sam_cio_op *operation);

#endif /* _SAM_H_ */
