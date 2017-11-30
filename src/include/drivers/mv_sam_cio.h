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
#include "env/mv_sys_event.h"

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
};

struct sam_cio_stats {
	u64 enq_pkts;   /**< Number of enqueued packet */
	u64 enq_bytes;  /**< Number of enqueued bytes */
	u64 enq_full;   /**< Number of times when ring was full on enqueue */
	u64 deq_pkts;   /**< Number of dequeued packet */
	u64 deq_bytes;  /**< Number of dequeued bytes */
	u64 deq_empty;  /**< Number of times ring was empty on dequeue */
};

/** DMAable buffer representation */
struct sam_buf_info {
	void       *vaddr; /**< virtual address of the buffer */
	dma_addr_t paddr;  /**< physical address of the buffer */
	u32        len;    /**< buffer size in bytes */
};

/** Possible crypto operation errors */
enum sam_cio_op_status {
	SAM_CIO_OK = 0,		/**< No errors */
	SAM_CIO_ERR_HW,		/**< Unexpected error returned by HW */
	SAM_CIO_ERR_ICV,	/**< ICV value mismatch */
	SAM_CIO_ERR_PROTO,      /**< Protocol error. Not a valid ESP or AH packet */
	SAM_CIO_ERR_SA_LOOKUP,  /**< SA lookup failed */
	SAM_CIO_ERR_ANTIREPLAY, /**< Anti-replay check failed */
	SAM_CIO_ERR_LAST
};

/** Possible registers to be printed */
enum sam_cio_regs {
	SAM_CIO_REGS_ALL = 0,	/**< Show all registers */
	SAM_CIO_REGS_CDR,	/**< Show CDR (Command Descriptors Ring) registers */
	SAM_CIO_REGS_RDR,	/**< Show RDR (Result Descriptors Ring) registers */
	SAM_CIO_REGS_LAST
};

struct sam_cio_event_params {
	u32 pkt_coal;
	u32 usec_coal;
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
 * IPSEC operation request
 *
 * Notes:
 *	- "src" and "dst" structures can be local.
 *	- "src->buf[i].vaddr" and "dst->buf[i].vaddr" must be valid until crypto operation is completed.
 */
struct sam_cio_ipsec_params {
	struct sam_sa       *sa;	/**< IPSEC session handler */
	void                *cookie;	/**< caller cookie to be return unchanged */
	u32 num_bufs;			/**< number of input/output buffers */
	struct sam_buf_info *src;	/**< array of input buffers */
	struct sam_buf_info *dst;	/**< array of output buffers */
	u32 l3_offset;                  /**< L3 header offset from beginning of src/dst buffer */
	u32 pkt_size;                   /**< packet size from beginning of src/dst buffer */
};

/** DTLS content type for outbound packet flow */
enum sam_cio_dtls_type {
	SAM_DTLS_CHANGE,
	SAM_DTLS_ALERT,
	SAM_DTLS_HANDSHAKE,
	SAM_DTLS_DATA,
	SAM_DTLS_TYPE_LAST
};

/**
 * SSL/TLS/DTLS operation request
 *
 * Notes:
 *	- "src" and "dst" structures can be local.
 *	- "src->buf[i].vaddr" and "dst->buf[i].vaddr" must be valid until crypto operation is completed.
 */
struct sam_cio_ssltls_params {
	struct sam_sa       *sa;	/**< session handler */
	void                *cookie;	/**< caller cookie to be return unchanged */
	u32 num_bufs;			/**< number of input/output buffers */
	struct sam_buf_info *src;	/**< array of input buffers */
	struct sam_buf_info *dst;	/**< array of output buffers */
	u32 l3_offset;                  /**< L3 header offset from beginning of src/dst buffer */
	u32 pkt_size;                   /**< packet size from beginning of src/dst buffer */
	enum sam_cio_dtls_type type;	/**< DTLS content type for outbound packet flow */
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
 * @retval	0          - success
 * @retval	Negative   - failure
 */
int sam_cio_enq(struct sam_cio *cio, struct sam_cio_op_params *requests, u16 *num);

/**
 * Dequeue single or multiple crypto operations to crypto IO instance
 *
 * @param[in]	  cio      - crypto IO instance handler.
 * @param[in]	  results  - pointer to results of one or more crypto operations
 * @param[in,out] num      - input:  number of results to dequeue
 *                           output: number of results successfully dequeued
 *
 * @retval	0          - success
 * @retval	Negative   - failed
 */
int sam_cio_deq(struct sam_cio *cio, struct sam_cio_op_result *results, u16 *num);

/**
 * Enqueue single or multiple crypto IPSEC operations to crypto IO instance
 *
 * @param[in]	  cio      - crypto IO instance handler.
 * @param[in]	  requests - pointer to parameters of one or more crypto IPSEC operations
 * @param[in,out] num      - input:  number of requests to enqueue
 *                           output: number of requests successfully enqueued
 *
 * @retval	0          - all requests are successfully enqueued.
 * @retval	Negative   - enqueue of one or more requests failed.
 */
int sam_cio_enq_ipsec(struct sam_cio *cio, struct sam_cio_ipsec_params *requests, u16 *num);

int sam_cio_enq_ssltls(struct sam_cio *cio, struct sam_cio_ssltls_params *requests, u16 *num);

/**
 * Flush crypto IO instance. All pending requests/results will be discarded.
 *
 * @param[in]     cio      - crypto IO instance handler.
 *
 * @retval      0          - no pending requests/results got crypto IO instance.
 * @retval      Negative   - failed on discard one or more pending requests/results.
 */
int sam_cio_flush(struct sam_cio *cio);

/**
 * Get statistics collected for crypto IO instance.
 *
 * To enable collect statistics capability of the SAM driver,
 *	use "--enable-sam-statistics" flag during ./configure
 *
 * @param[in]     cio      - crypto IO instance handler.
 * @param[out]    stats    - pointer to copy statistics.
 * @param[in]     reset    - 0    : don't reset statistics after copy.
 *			     other: reset statistics after copy.
 *
 * @retval      0          - Success
 * @retval	-ENOTSUP   - Debug capability is not supported
 */
int sam_cio_get_stats(struct sam_cio *cio, struct sam_cio_stats *stats, int reset);

/**
 * Show (Command Descriptors Ring) registers for crypto IO instance.
 *
 * To enable debug capability of the SAM driver,
 *	use "--enable-sam-debug" flag during ./configure
 *
 * @param[in]     cio      - crypto IO instance handler.
 * @param[in]     regs     - group(s) of registers to be shown
 *
 * @retval      0          - Success
 * @retval	-ENOTSUP   - Debug capability is not supported
 */
int sam_cio_show_regs(struct sam_cio *cio, enum sam_cio_regs regs);


/**
 * Create crypto IO event.
 *
 * @param[in]	cio        - crypto IO instance handler.
 * @param[in]	params     - event parameters
 * @param[out]  ev         - address of place to save handler of new created crypto IO event.
 *
 * @retval      0          - success
 * @retval      Negative   - failure
 */
int sam_cio_create_event(struct sam_cio *cio, struct sam_cio_event_params *params,
			 struct mv_sys_event **ev);

/**
 * Delete crypto IO event.
 *
 * @param[in]	cio        - crypto IO instance handler.
 * @param[in]	ev         - event handler.
 *
 * @retval      0          - success.
 * @retval      Negative   - failure.
 */
int sam_cio_delete_event(struct sam_cio *cio, struct mv_sys_event *ev);

/**
 * Enable/Disable crypto IO event.
 *
 * @param[in]	cio        - crypto IO instance handler.
 * @param[in]	ev         - event handler.
 * @param[in]	en         - 0 - disable, 1 -enable.
 *
 * @retval      0          - success.
 * @retval      Negative   - failure.
 */
int sam_cio_set_event(struct sam_cio *cio, struct mv_sys_event *ev, int en);

int sam_cio_enable(struct sam_cio *cio);
int sam_cio_disable(struct sam_cio *cio);

/** @} */ /* end of grp_sam_cio */

#endif /* __MV_SAM_CIO_H__ */
