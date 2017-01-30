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

#ifndef __MV_SAM_CIO_H__
#define __MV_SAM_CIO_H__

#include "mv_std.h"
#include "mv_sam_session.h"

/** @addtogroup grp_sam_cio Security Acceleration Module: Crypto I/O
 *
 *  Security Acceleration Module Crypto I/O API documentation
 *
 *  @{
 */

/** Maximum number of input/output buffers for one crypto operation */
#define SAM_CIO_MAX_FRAGS	20

/** parameters for CIO instance */
struct sam_cio_params {
	const char *match; /**< SAM HW string in DTS file. e.g. "cio-0:0" */
	u32 size;          /**< ring size in number of descriptors */
	u32 num_sessions;  /**< number of supported sessions */
	u32 max_buf_size;  /**< maximum buffer size [in bytes] */
};

/** DMAable buffer representation */
struct sam_buf_info {
	void       *vaddr; /**< virtual address of the buffer */
	dma_addr_t paddr;  /**< physical address of the buffer */
	u32        len;    /**< buffer size in bytes */
};

/** Possible crypto operation errors */
enum sam_cio_op_status {
	SAM_CIO_OK = 0,  /**< No errors */
	SAM_CIO_ERR_HW,	 /**< Unexpected error returned by HW */
	SAM_CIO_ERR_ICV, /**< ICV value mismatch */
	SAM_CIO_ERR_LAST
};

/**
 * Crypto operation parameters
 *
 * Notes:
 *	- "num_bufs" must be in range from 1 to SAM_CIO_MAX_FRAGS.
 *	- "src" and "dst" buffers must be valid until crypto operation is completed.
 *	- "cipher_iv" and "cipher_offset" are valid only if "cipher_iv" field
 *	in "struct sam_session_params" is NULL and "crypto_mode" requires IV.
 *	- if "cipher_iv" != NULL, IV will be taken from external buffer pointed by "cipher_iv".
 *	- if "cipher_iv" == NULL, IV will be taken from "src" buffer accordingly with
 *	"cipher_iv_offset".
 *	- size of "cipher_iv" buffer is derived from cipher algorithm.
 *	- "auth_aad_offset" and "auth_aad" fields are valid only when "crypto_mode" is GCM or GMAC.
 *	- if "auth_aad" != NULL, AAD will be taken from external buffer pointed by "auth_aad".
 *	- if "auth_aad" == NULL, AAD will be taken from "src" buffer accordingly with
 *	"auth_aad_offset".
 */
struct sam_cio_op_params {
	struct sam_sa *sa;    /**< session handler */
	void *cookie;         /**< caller cookie to be return unchanged */
	u32  num_bufs;        /**< number of input/output buffers */
	struct sam_buf_info *src; /**< array of input buffers */
	struct sam_buf_info *dst; /**< array of output buffers */
	u32  cipher_iv_offset;/**< IV offset in the buffer (in bytes) */
	u8   *cipher_iv;      /**< pointer to external IV buffer */
	u32  cipher_offset;   /**< start of data for encryption (in bytes) */
	u32  cipher_len;      /**< size of data for encryption (in bytes) */
	u32  auth_aad_offset; /**< start of AAD in the buffer (in bytes) */
	u8   *auth_aad;       /**< pointer to external AAD buffer */
	u32  auth_offset;     /**< start of data for authentication (in bytes) */
	u32  auth_len;        /**< size of data for authentication (in bytes) */
	u32  auth_icv_offset; /**< offset of ICV in the buffer (in bytes) */
};

/** Crypto operation result */
struct sam_cio_op_result {
	void			*cookie; /**< caller cookie passed from request */
	u32			out_len; /**< output data length */
	enum sam_cio_op_status	status;  /**< status of crypto operation. */
};

/**
 * Create and initialize crypto IO instance
 *
 * @param[in]	params    - pointer to structure with crypto IO parameters.
 * @param[out]	cio       - address of place to save handler of new created crypto IO instance.
 *
 * @retval	0         - success
 * @retval	Negative  - failure
 */
int sam_cio_init(struct sam_cio_params *params, struct sam_cio **cio);

/**
 * Delete crypto IO instance
 *
 * @param[in]	cio	  - crypto IO instance handler.
 *
 * @retval	0         - success
 * @retval	Negative  - failure
 */
int sam_cio_deinit(struct sam_cio *cio);

/**
 * Enqueue single or multiple crypto operations to crypto IO instance
 *
 * @param[in]	  cio      - crypto IO instance handler.
 * @param[in]	  requests - pointer to parameters of one or more crypto operations
 * @param[in,out] num      - input:  number of requests to enqueue
 *                           output: number of requests successfully enqueued
 *
 * @retval	0          - all requests are successfully enqueued.
 * @retval	Negative   - enqueue of one or more requests failed.
 */
int sam_cio_enq(struct sam_cio *cio, struct sam_cio_op_params *requests, u16 *num);

/**
 * Enqueue single or multiple crypto operations to crypto IO instance
 *
 * @param[in]	  cio      - crypto IO instance handler.
 * @param[in]	  results  - pointer to results of one or more crypto operations
 * @param[in,out] num      - input:  number of results to dequeue
 *                           output: number of results successfully dequeued
 *
 * @retval	0          - all results are successfully dequeued.
 * @retval	Negative   - dequeue of one or more results failed.
 */
int sam_cio_deq(struct sam_cio *cio, struct sam_cio_op_result *results, u16 *num);

int sam_cio_enable(struct sam_cio *cio);
int sam_cio_disable(struct sam_cio *cio);

/** @} */ /* end of grp_sam_cio */

#endif /* __MV_SAM_CIO_H__ */
