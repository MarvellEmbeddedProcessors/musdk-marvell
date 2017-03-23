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

#include "std_internal.h"
#include "drivers/mv_sam.h"
#include "lib/lib_misc.h"

#include "sam.h"

#define SAM_MAX_CIO_NUM		(SAM_HW_RING_NUM * SAM_HW_ENGINE_NUM)

static bool		sam_initialized;
static int		sam_active_cios;
static struct sam_cio	*sam_cios[SAM_MAX_CIO_NUM];

static int sam_cio_free_idx_get(void)
{
	int i;

	for (i = 0; i < SAM_MAX_CIO_NUM; i++) {
		if (sam_cios[i] == NULL)
			return i;
	}
	return -1;
}

int sam_dma_buf_alloc(u32 buf_size, struct sam_buf_info *dma_buf)
{
	dma_buf->vaddr = mv_sys_dma_mem_alloc(buf_size, 256);
	if (!dma_buf->vaddr) {
		pr_err("Can't allocate input DMA buffer of %d bytes\n", buf_size);
		return -ENOMEM;
	}
	dma_buf->paddr = mv_sys_dma_mem_virt2phys(dma_buf->vaddr);
	dma_buf->len = buf_size;

	return 0;
}

void sam_dma_buf_free(struct sam_buf_info *dma_buf)
{
	if (dma_buf && dma_buf->vaddr)
		mv_sys_dma_mem_free(dma_buf->vaddr);
}

static struct sam_sa *sam_session_alloc(struct sam_cio *cio)
{
	int i;

	for (i = 0; i < cio->params.num_sessions; i++) {
		if (!cio->sessions[i].is_valid) {
			cio->sessions[i].is_valid = true;
			return &cio->sessions[i];
		}
	}
	pr_err("All sessions are busy\n");
	return NULL;
}

static void sam_session_free(struct sam_sa *sa)
{
	sa->is_valid = false;
}

static int sam_session_crypto_init(struct sam_session_params *params,
				   SABuilder_Params_Basic_t *basic_params,
				   SABuilder_Params_t *sa_params)
{
	if (params->cipher_alg == SAM_CIPHER_NONE)
		return 0;

	/* Check validity of crypto parameters */
	if (sam_max_check((int)params->cipher_alg, SAM_CIPHER_ALG_LAST, "cipher_alg"))
		return -EINVAL;

	if (sam_max_check((int)params->cipher_mode, SAM_CIPHER_MODE_LAST, "cipher_mode"))
		return -EINVAL;

	/* GCM and GMAC cipher modes are supported only for AES */
	if ((params->cipher_mode == SAM_CIPHER_GCM) || (params->cipher_mode == SAM_CIPHER_GMAC)) {
		if (params->cipher_alg != SAM_CIPHER_AES) {
			pr_err("GCM and GMAC cipher modes are supported only for AES, mode = %d, alg = %d\n",
				params->cipher_mode, params->cipher_alg);
			return -EINVAL;
		}
	}
	sa_params->CryptoAlgo = (SABuilder_Crypto_t)params->cipher_alg;
	sa_params->CryptoMode = (SABuilder_Crypto_Mode_t)params->cipher_mode;
	sa_params->KeyByteCount = params->cipher_key_len;
	sa_params->Key_p        = params->cipher_key;

	sa_params->IVSrc  = SAB_IV_SRC_TOKEN;

	return 0;
}

static int sam_session_auth_init(struct sam_session_params *params,
				 SABuilder_Params_Basic_t *basic_params,
				 SABuilder_Params_t *sa_params)
{
	if (params->auth_alg == SAM_AUTH_NONE)
		return 0;

	/* Check validity of crypto parameters */
	if (sam_max_check((int)params->auth_alg, SAM_AUTH_ALG_LAST, "auth_alg"))
		return -EINVAL;

	sa_params->AuthAlgo = (SABuilder_Auth_t)params->auth_alg;
	sa_params->AuthKey1_p   = params->auth_inner;
	sa_params->AuthKey2_p   = params->auth_outer;

	basic_params->ICVByteCount = params->auth_icv_len;
	if (params->dir == SAM_DIR_DECRYPT)
		basic_params->BasicFlags |= SAB_BASIC_FLAG_EXTRACT_ICV;

	if ((params->auth_alg == SAM_AUTH_AES_GCM) || (params->auth_alg == SAM_AUTH_AES_GMAC))
		sa_params->flags |= SAB_FLAG_SUPPRESS_HEADER;

	return 0;
}

static int sam_hw_cmd_token_build(struct sam_cio_op_params *request,
				  struct sam_cio_op *operation)
{
	struct sam_sa *session = request->sa;
	TokenBuilder_Params_t token_params;
	u32 copylen;
	TokenBuilder_Status_t rc;

	memset(&token_params, 0, sizeof(token_params));
	if (request->auth_len) {
		copylen = request->auth_len;
		if (session->params.dir == SAM_DIR_DECRYPT)
			copylen += session->params.auth_icv_len;
	} else {
		/* chipher only */
		copylen = request->cipher_len;
	}
	if (request->cipher_iv)
		token_params.IV_p = request->cipher_iv;

	if (request->auth_len)
		token_params.BypassByteCount = request->auth_offset;
	else
		token_params.BypassByteCount = request->cipher_offset;

	copylen += token_params.BypassByteCount;

	/* process AAD */
	if (request->auth_aad) {
		token_params.AAD_p = request->auth_aad;
		token_params.AdditionalValue = session->params.auth_aad_len;
	} else if (request->auth_len && request->cipher_len)
		token_params.AdditionalValue = request->auth_len - request->cipher_len;

#ifdef MVCONF_SAM_DEBUG
	if (session->cio->debug_flags & SAM_CIO_DEBUG_FLAG)
		print_token_params(&token_params);
#endif /* MVCONF_SAM_DEBUG */

	rc = TokenBuilder_BuildToken(session->tcr_data, request->src->vaddr,
				     copylen, &token_params,
				     operation->token_buf.vaddr,
				     &operation->token_words, &operation->token_header_word);
	if (rc != TKB_STATUS_OK) {
		pr_err("%s: TokenBuilder_BuildToken failed, rc = %d\n",
			__func__, rc);
		return -EINVAL;
	}
	/* Swap Token data if needed */
	sam_htole32_multi(operation->token_buf.vaddr, operation->token_words);

	/* Enable Context Reuse auto detect if no new SA */
	operation->token_header_word &= ~SAM_TOKEN_REUSE_CONTEXT_MASK;
	if (session->is_first)
		session->is_first = false;
	else
		operation->token_header_word |= SAM_TOKEN_REUSE_AUTO_MASK;

	operation->copy_len = copylen;

#ifdef MVCONF_SAM_DEBUG
	if (session->cio->debug_flags & SAM_CIO_DEBUG_FLAG) {
		print_sam_cio_operation_info(operation);

		printf("\nToken DMA buffer: %d bytes\n", operation->token_words * 4);
		mv_mem_dump(operation->token_buf.vaddr, operation->token_words * 4);
	}
#endif /* MVCONF_SAM_DEBUG */

	return 0;
}

/*********************** Public functions implementation *******************/

int sam_cio_init(struct sam_cio_params *params, struct sam_cio **cio)
{
	int i, engine, ring, cio_idx, scanned;
	struct sam_cio	*sam_ring;

	/* Load SAM HW engine */
	if (!sam_initialized) {
		if (sam_hw_engine_load())
			return -EINVAL;
	}
	sam_initialized = true;

	/* Parse match string to ring number */
	scanned = sscanf(params->match, "cio-%d:%d\n", &engine, &ring);
	if (scanned != 2) {
		pr_err("Invalid match string %s. Expected: cio-0:X\n",
			params->match);
		return -EINVAL;
	}
	cio_idx = sam_cio_free_idx_get();
	if (cio_idx < 0) {
		pr_err("No free place for new CIO: active_cios = %d, max_cios = %d\n",
			sam_active_cios, SAM_MAX_CIO_NUM);
		return -EINVAL;
	}

	/* Allocate single sam_cio structure */
	sam_ring = kcalloc(1, sizeof(struct sam_cio), GFP_KERNEL);
	if (!sam_ring) {
		pr_err("Can't allocate %lu bytes for sam_cio structure\n",
			 sizeof(struct sam_cio));
		return -ENOMEM;
	}

	/* Initialize HW ring */
	if (sam_hw_ring_init(engine, ring, params, &sam_ring->hw_ring))
		goto err;

	/* Save configured CIO params */
	sam_ring->params = *params;

	sam_cios[cio_idx] = sam_ring;
	sam_ring->idx = cio_idx;
	sam_active_cios++;

	/* Allocate configured number of sam_sa structures */
	sam_ring->sessions = kcalloc(params->num_sessions, sizeof(struct sam_sa), GFP_KERNEL);
	if (!sam_ring->sessions) {
		pr_err("Can't allocate %u * %lu bytes for sam_sa structures\n",
			params->num_sessions, sizeof(struct sam_sa));
		goto err;
	}

	/* Allocate DMA buffer for each session */
	for (i = 0; i < params->num_sessions; i++) {
		if (sam_dma_buf_alloc(SAM_SA_DMABUF_SIZE, &sam_ring->sessions[i].sa_buf)) {
			pr_err("Can't allocate DMA buffer (%d bytes) for Session #%d\n",
				SAM_SA_DMABUF_SIZE, i);
			goto err;
		}
	}
	pr_info("DMA buffers allocated for %d sessions (%d bytes)\n",
		params->num_sessions, SAM_SA_DMABUF_SIZE);

	/* Allocate array of sam_cio_op structures in size of CIO ring */
	sam_ring->operations = kcalloc(params->size, sizeof(struct sam_cio_op), GFP_KERNEL);
	if (!sam_ring->operations) {
		pr_err("Can't allocate %u * %lu bytes for sam_cio_op structures\n",
			params->size, sizeof(struct sam_cio_op));
		goto err;
	}

	/* Allocate DMA buffers for Tokens (one per operation) */
	for (i = 0; i < params->size; i++) {
		if (sam_dma_buf_alloc(SAM_TOKEN_DMABUF_SIZE, &sam_ring->operations[i].token_buf)) {
			pr_err("Can't allocate DMA buffer (%d bytes) for Token #%d\n",
				SAM_TOKEN_DMABUF_SIZE, i);
			goto err;
		}
	}
	pr_info("DMA buffers allocated for %d operations. Tokens - %d bytes\n",
		i, SAM_TOKEN_DMABUF_SIZE);

	*cio = sam_ring;

	return 0;

err:
	/* Release all allocated resources */
	sam_cio_deinit(sam_ring);

	return -ENOMEM;
}

int sam_cio_flush(struct sam_cio *cio)
{
	int rc;
	u32 count = 1000;
	u16 num;

	/* Wait for completion of all operations */
	while (!sam_cio_is_empty(cio)) {
		num = cio->params.size;
		rc = sam_cio_deq(cio, NULL, &num);
		if (rc) {
			pr_err("%s: dequeue error %d\n", __func__, rc);
			return rc;
		}

		if (num) /* restart counter */
			count = 1000;

		if (count-- == 0) {
			pr_err("%s: Timeout\n", __func__);
			return -EINVAL;
		}
	}
	return 0;
}

int sam_cio_deinit(struct sam_cio *cio)
{
	int i;

	if (!cio)
		return 0;

	if (cio->operations) {
		/* Wait for completion of all operations */
		sam_cio_flush(cio);

		for (i = 0; i < cio->params.size; i++) {
			sam_dma_buf_free(&cio->operations[i].token_buf);
		}
		kfree(cio->operations);
	}

	if (cio->sessions) {
		for (i = 0; i < cio->params.num_sessions; i++) {
			if (cio->sessions[i].is_valid)
				sam_session_destroy(&cio->sessions[i]);

			sam_dma_buf_free(&cio->sessions[i].sa_buf);
		}

		kfree(cio->sessions);
		cio->sessions = NULL;
	}

	if (sam_cios[cio->idx]) {
		sam_hw_ring_deinit(&cio->hw_ring);
		sam_cios[cio->idx] = NULL;
		sam_active_cios--;
	}

	if (sam_initialized && (sam_active_cios == 0)) {
		sam_hw_engine_unload();
		sam_initialized = false;
	}
	kfree(cio);

	return 0;
}

int sam_session_create(struct sam_cio *cio, struct sam_session_params *params, struct sam_sa **sa)
{
	SABuilder_Direction_t direction = (SABuilder_Direction_t)params->dir;
	struct sam_sa *session;
	int rc;

	/* Find free session structure */
	session = sam_session_alloc(cio);
	if (!session) {
		pr_err("%s: Can't get free session\n", __func__);
		return -EBUSY;
	}
	/* Clear session structure */
	memset(&session->sa_params, 0, sizeof(session->sa_params));
	memset(&session->basic_params, 0, sizeof(session->basic_params));

#ifdef MVCONF_SAM_DEBUG
	if (cio->debug_flags & SAM_SA_DEBUG_FLAG) {
		print_sam_sa_params(params);
		if (params->cipher_key) {
			printf("\nCipher Key: %d bytes\n", params->cipher_key_len);
			mv_mem_dump(params->cipher_key, params->cipher_key_len);
		}
		if (params->auth_inner) {
			printf("\nAuthentication Inner: %d bytes\n", 64);
			mv_mem_dump(params->auth_inner, 64);
		}
		if (params->auth_outer) {
			printf("\nAuthentication Outer: %d bytes\n", 64);
			mv_mem_dump(params->auth_outer, 64);
		}
	}
#endif /* MVCONF_SAM_DEBUG */

	/* Save session params */
	session->params = *params;

	/* Initialize sa_params and basic_params */
	SABuilder_Init_Basic(&session->sa_params, &session->basic_params, direction);

	/* Update sa_params and basic_params with session information */
	if (sam_session_crypto_init(params, &session->basic_params, &session->sa_params))
		goto error_session;

	if (sam_session_auth_init(params, &session->basic_params, &session->sa_params))
		goto error_session;

	/* Sanity check for SA and TCR size */
	rc = SABuilder_GetSizes(&session->sa_params, &session->sa_words, NULL, NULL);
	if (rc != 0) {
		pr_err("%s: SA not created because of error, rc = %d\n", __func__, rc);
		goto error_session;
	}
	if (session->sa_words > SAM_SA_DMABUF_SIZE / 4) {
		pr_err("%s: SA size %d words is too big. Maximum = %d words\n",
			__func__, session->sa_words, SAM_SA_DMABUF_SIZE / 4);
		goto error_session;
	}
	/* Clear DMA buffer allocated for SA */
	memset(session->sa_buf.vaddr, 0, 4 * session->sa_words);

	rc = TokenBuilder_GetContextSize(&session->sa_params, &session->tcr_words);
	if (rc != 0) {
		pr_err("%s: TokenBuilder_GetContextSize return error, rc = %d\n",
			__func__, rc);
		goto error_session;
	}
	if (session->tcr_words > SAM_TCR_DATA_SIZE / 4) {
		pr_err("%s: TCR size %d words is too big. Maximum = %d words\n",
			__func__, session->tcr_words, SAM_TCR_DATA_SIZE / 4);
		goto error_session;
	}
	/* Clear TCR data buffer used allocated for SA */
	memset(session->tcr_data, 0, 4 * session->tcr_words);

	rc = TokenBuilder_BuildContext(&session->sa_params, session->tcr_data);
	if (rc != 0) {
		pr_err("%s: TokenBuilder_BuildContext failed, rc = %d\n", __func__, rc);
		goto error_session;
	}

	rc = TokenBuilder_GetSize(session->tcr_data, &session->token_words);
	if (rc != 0) {
		pr_err("%s:: TokenBuilder_GetSize failed.\n", __func__);
		goto error_session;
	}

	if (session->token_words > SAM_TOKEN_DMABUF_SIZE / 4) {
		pr_err("%s: Token size %d words is too big. Maximum = %d words\n",
			__func__, session->token_words, SAM_TOKEN_DMABUF_SIZE / 4);
		goto error_session;
	}

	/* build the SA data and init according the parameters */
	rc = SABuilder_BuildSA(&session->sa_params, (u32 *)session->sa_buf.vaddr, NULL, NULL);
	if (rc != 0) {
		pr_err("%s: SABuilder_BuildSA failed, rc = %d\n", __func__, rc);
		goto error_session;
	}
	/* Swap session data if needed */
	sam_htole32_multi(session->sa_buf.vaddr, session->sa_words);

#ifdef MVCONF_SAM_DEBUG
	if (cio->debug_flags & SAM_SA_DEBUG_FLAG) {
		print_sa_params(&session->sa_params);
		print_basic_sa_params(&session->basic_params);
		printf("\nSA DMA buffer: %d bytes\n", session->sa_words * 4);
			mv_mem_dump(session->sa_buf.vaddr, session->sa_words * 4);
	}
#endif /* MVCONF_SAM_DEBUG */

	SAM_STATS(cio->stats.sa_add++);

	session->is_first = true;
	session->cio = cio;
	*sa = session;

	return 0;

error_session:
	sam_session_free(session);
	return -EINVAL;
}

int sam_session_destroy(struct sam_sa *session)
{
	struct sam_cio *cio = session->cio;
	struct sam_cio_op *operation;

	if (cio->hw_ring.type == HW_EIP197) {
		/* Check maximum number of pending requests */
		if (sam_cio_is_full(cio)) {
			SAM_STATS(cio->stats.enq_full++);
			return -EINVAL;
		}

		/* Get next operation structure */
		operation = &cio->operations[cio->next_request];
		operation->sa = session;
		operation->num_bufs = 0;

		/* Invalidate session in HW */
		sam_hw_session_invalidate(&cio->hw_ring, &session->sa_buf, cio->next_request);
		SAM_STATS(cio->stats.sa_inv++);

		cio->next_request = sam_cio_next_idx(cio, cio->next_request);
	} else {
		sam_session_free(session);
		SAM_STATS(cio->stats.sa_del++);
	}
	return 0;
}

static int sam_cio_check_op_params(struct sam_cio_op_params *request)
{
	if (request->num_bufs != 1) {
		/* Multiple buffers not supported */
		return -ENOTSUP;
	}

	if (request->src == NULL) {
		/* One source buffer is mandatory */
		return -ENOTSUP;
	}

	if (request->dst == NULL) {
		/* One destination buffer is mandatory */
		return -ENOTSUP;
	}

	return 0;
}

int sam_cio_enq(struct sam_cio *cio, struct sam_cio_op_params *requests, u16 *num)
{
	struct sam_cio_op *operation;
	struct sam_cio_op_params *request;
	int i, j, err, todo;

	todo = *num;
	if (todo >= cio->params.size)
		todo = cio->params.size - 1;

	for (i = 0; i < todo; i++) {
		request = &requests[i];

		/* Check request validity */
		err = sam_cio_check_op_params(request);
		if (err)
			return err;

		/* Check maximum number of pending requests */
		if (sam_cio_is_full(cio)) {
			SAM_STATS(cio->stats.enq_full++);
			break;
		}
#ifdef MVCONF_SAM_DEBUG
		if (cio->debug_flags & SAM_CIO_DEBUG_FLAG)
			print_sam_cio_op_params(request);
#endif /* MVCONF_SAM_DEBUG */

		/* Get next operation structure */
		operation = &cio->operations[cio->next_request];

		if (sam_hw_cmd_token_build(request, operation))
			goto error_enq;

		/* Save some fields from request needed for result processing */
		operation->sa = request->sa;
		operation->cookie = request->cookie;
		operation->num_bufs = request->num_bufs;
		operation->auth_icv_offset = request->auth_icv_offset;
		for (j = 0;  j < request->num_bufs; j++) {
			operation->out_frags[j].vaddr = request->dst[j].vaddr;
			operation->out_frags[j].paddr = request->dst[j].paddr;
			operation->out_frags[j].len = request->dst[j].len;
		}
		if (cio->hw_ring.type == HW_EIP197) {
			sam_hw_ring_desc_write(&cio->hw_ring, cio->next_request,
					request->src, request->dst, operation->copy_len,
					&request->sa->sa_buf, &operation->token_buf,
					operation->token_header_word, operation->token_words);
		} else {
			sam_hw_ring_basic_desc_write(&cio->hw_ring, cio->next_request,
					request->src, request->dst, operation->copy_len,
					&request->sa->sa_buf, &operation->token_buf,
					operation->token_header_word, operation->token_words);
		}

#ifdef MVCONF_SAM_DEBUG
		if (cio->debug_flags & SAM_CIO_DEBUG_FLAG) {
			struct sam_hw_cmd_desc *cmd_desc = sam_hw_cmd_desc_get(&cio->hw_ring, cio->next_result);

			print_cmd_desc(cmd_desc);
			printf("\nInput DMA buffer: %d bytes, physAddr = %p\n",
				operation->copy_len, (void *)request->src->paddr);
			mv_mem_dump(request->src->vaddr, operation->copy_len);
		}
#endif /* MVCONF_SAM_DEBUG */

		cio->next_request = sam_cio_next_idx(cio, cio->next_request);
		SAM_STATS(cio->stats.enq_bytes += operation->copy_len);
	}
	/* submit requests */
	if (i) {
		sam_hw_ring_submit(&cio->hw_ring, i);
		SAM_STATS(cio->stats.enq_pkts += i);
	}
	*num = (u16)i;

	return 0;

error_enq:
	return -EINVAL;
}

/* Process crypto operation result */
int sam_cio_deq(struct sam_cio *cio, struct sam_cio_op_result *results, u16 *num)
{
	unsigned int i, count, todo, done, out_len;
	struct sam_cio_op *operation;
	struct sam_hw_res_desc *res_desc;
	struct sam_cio_op_result *result;

	/* Try to get the processed packet from the RDR */
	done = sam_hw_ring_ready_get(&cio->hw_ring);
	if (!done) {
		SAM_STATS(cio->stats.deq_empty++);
		*num = 0;
		return 0;
	}
	todo = *num;

	result = results;
	i = 0;
	count = 0;
	while ((i < done) && (count < todo)) {
		res_desc = sam_hw_res_desc_get(&cio->hw_ring, cio->next_result);

#ifdef MVCONF_SAM_DEBUG
		if (cio->debug_flags & SAM_CIO_DEBUG_FLAG)
			print_result_desc(res_desc);
#endif
		i++;
		operation = &cio->operations[cio->next_result];
		if (operation->num_bufs == 0) {
			/* Inalidate cache entry finished - free session */
			sam_session_free(operation->sa);
			SAM_STATS(cio->stats.sa_del++);
			cio->next_result = sam_cio_next_idx(cio, cio->next_result);
			continue;
		}
		sam_hw_res_desc_read(res_desc, result);
		out_len = result->out_len;

		SAM_STATS(cio->stats.deq_bytes += out_len);

		/* Increment next result index */
		cio->next_result = sam_cio_next_idx(cio, cio->next_result);

		result->cookie = operation->cookie;

		/* Check output buffer size */
		if (operation->out_frags[0].len < out_len) {
			pr_err("%s: out_len %d bytes is larger than output buffer %d bytes\n",
				__func__, out_len, operation->out_frags[0].len);
		}

#ifdef MVCONF_SAM_DEBUG
		if (cio->debug_flags & SAM_CIO_DEBUG_FLAG) {
			printf("\nOutput DMA buffer: %d bytes\n", out_len);
			mv_mem_dump(operation->out_frags[0].vaddr, out_len);
		}
#endif /* MVCONF_SAM_DEBUG */

		result++;
		count++;
	}
	/* Update RDR registers */
	sam_hw_ring_update(&cio->hw_ring, i);

	SAM_STATS(cio->stats.deq_pkts += count);

	*num = (u16)count;

	return 0;
}

int sam_cio_debug_flags_set(struct sam_cio *cio, u32 debug_flags)
{
#ifdef MVCONF_SAM_DEBUG
	cio->debug_flags = debug_flags;
	return 0;
#else
	return -ENOTSUP;
#endif /* MVCONF_SAM_STATS */
}

int sam_cio_stats_get(struct sam_cio *cio, struct sam_cio_stats *stats, int reset)
{
#ifdef MVCONF_SAM_STATS
	memcpy(stats, &cio->stats, sizeof(cio->stats));

	if (reset)
		memset(&cio->stats, 0, sizeof(cio->stats));

	return 0;
#else
	return -ENOTSUP;
#endif /* MVCONF_SAM_STATS */
}
