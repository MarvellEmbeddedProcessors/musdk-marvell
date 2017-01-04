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

/*#define SAM_CIO_DEBUG*/
/*#define SAM_SA_DEBUG*/

static bool		sam_initialized;
static int		sam_active_rings;
static struct sam_cio	*sam_rings[SAM_HW_RING_NUM];

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

	/* Special processing for GCM/GMAC modes */

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

	return 0;
}

static int sam_hw_cmd_desc_init(struct sam_cio_op_params *request,
				struct sam_cio_op *operation,
				PEC_CommandDescriptor_t *cmd_desc,
				PEC_PacketParams_t *pkt_params)
{
	struct sam_sa *session = request->sa;
	TokenBuilder_Params_t token_params;
	u32 copylen, token_header_word, token_words;
	TokenBuilder_Status_t rc;
	PEC_Status_t pec_rc;

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

	if (request->auth_len && request->cipher_len)
		token_params.AdditionalValue = request->auth_len - request->cipher_len;

	if (request->auth_len)
		token_params.BypassByteCount = request->auth_offset;
	else
		token_params.BypassByteCount = request->cipher_offset;

	copylen += token_params.BypassByteCount;

	/* process AAD  - TBD */

#ifdef SAM_CIO_DEBUG
	print_token_params(&token_params);

	printf("\nInput DMA buffer: %d bytes, physAddr = %p\n",
		copylen, (void *)request->src->paddr);
	mv_mem_dump(request->src->vaddr, copylen);
#endif

	rc = TokenBuilder_BuildToken(session->tcr_data, request->src->vaddr,
				     copylen, &token_params,
				     operation->token_buf.vaddr,
				     &token_words, &token_header_word);
	if (rc != TKB_STATUS_OK) {
		pr_err("%s: TokenBuilder_BuildToken failed, rc = %d\n",
			__func__, rc);
		return -EINVAL;
	}

	cmd_desc->SA_Handle1.p     = (void *)session->sa_buf.paddr;
	cmd_desc->SA_WordCount     = session->sa_words;
	cmd_desc->SA_Handle2       = DMABuf_NULLHandle;
	cmd_desc->Token_Handle.p   = (void *)operation->token_buf.paddr;
	cmd_desc->SrcPkt_Handle.p  = (void *)request->src->paddr;
	cmd_desc->DstPkt_Handle.p  = (void *)request->dst->paddr;
	cmd_desc->User_p           = (void *)operation;
	cmd_desc->SrcPkt_ByteCount = copylen;
	cmd_desc->Token_WordCount  = token_words;
	pkt_params->HW_Services  = FIRMWARE_EIP207_CMD_PKT_LAC;
	pkt_params->TokenHeaderWord = token_header_word;

	pec_rc = PEC_CD_Control_Write(cmd_desc, pkt_params);
	if (pec_rc != PEC_STATUS_OK) {
		pr_err("%s: PEC_CD_Control_Write failed, rc = %d\n",
			__func__, pec_rc);
		return -EINVAL;
	}
	return 0;
}

/*********************** Public functions implementation *******************/

int sam_cio_init(struct sam_cio_params *params, struct sam_cio **cio)
{
	int i, engine, ring, scanned;
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
	/* Check valid range for CIO ring */
	if (ring >= SAM_HW_RING_NUM) {
		pr_err("sam_cio ring #%d is out range: 0 ... %d\n",
			ring, SAM_HW_RING_NUM);
		return -EINVAL;
	}
	/* Each instance of CIO can be used only once */
	if (sam_rings[ring] != NULL) {
		pr_err("sam_cio_init for ring #is already called\n");
		return -EEXIST;
	}

	/* Check ring size and number of sessions with HW max values */
	if (params->size > SAM_HW_RING_SIZE) {
		/* SW value can't exceed HW restriction */
		pr_warning("Warning! Ring size %d is too large. Set to maximum = %d\n",
			params->size, SAM_HW_RING_SIZE);
		params->size = SAM_HW_RING_SIZE;
	}
	params->size += 1;

	if (params->num_sessions > SAM_HW_SA_NUM) {
		/* SW value can't exceed HW restriction */
		pr_warning("Warning! Number of sessions %d is too large. Set to maximum = %d\n",
			params->num_sessions, SAM_HW_SA_NUM);
		params->num_sessions = SAM_HW_SA_NUM;
	}

	/* Allocate single sam_cio structure */
	sam_ring = kcalloc(1, sizeof(struct sam_cio), GFP_KERNEL);
	if (!sam_ring) {
		pr_err("Can't allocate %lu bytes for sam_cio structure\n",
			 sizeof(struct sam_cio));
		return -ENOMEM;
	}
	/* Save configured CIO params */
	sam_ring->params = *params;
	sam_ring->id = ring;

	/* Initialize HW ring */
	if (sam_hw_ring_init(ring, params->size, &sam_ring->hw_ring))
		goto err;

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
	sam_ring->hw_ring.cmd_desc = kcalloc(params->size, sizeof(PEC_CommandDescriptor_t), GFP_KERNEL);
	if (!sam_ring->hw_ring.cmd_desc) {
		pr_err("Can't allocate %u * %lu bytes for PEC_CommandDescriptor_t structures\n",
			params->size, sizeof(PEC_CommandDescriptor_t));
		goto err;
	}
	sam_ring->hw_ring.result_desc = kcalloc(params->size, sizeof(PEC_ResultDescriptor_t), GFP_KERNEL);
	if (!sam_ring->hw_ring.result_desc) {
		pr_err("Can't allocate %u * %lu bytes for PEC_ResultDescriptor_t structures\n",
			params->size, sizeof(PEC_ResultDescriptor_t));
		goto err;
	}

	/* Allocate DMA buffers for Token and Data for each operation */
	for (i = 0; i < params->size; i++) {
		if (sam_dma_buf_alloc(SAM_TOKEN_DMABUF_SIZE, &sam_ring->operations[i].token_buf)) {
			pr_err("Can't allocate DMA buffer (%d bytes) for Token #%d\n",
				SAM_TOKEN_DMABUF_SIZE, i);
			goto err;
		}
	}
	pr_info("DMA buffers allocated for %d operations. Tokens - %d bytes, Buffers - %d bytes\n",
		i, SAM_TOKEN_DMABUF_SIZE, params->max_buf_size);

	sam_rings[ring] = sam_ring;
	sam_active_rings++;

	*cio = sam_ring;

	return 0;

err:
	/* Release all allocated resources */
	sam_cio_deinit(sam_ring);

	return -ENOMEM;
}

int sam_cio_deinit(struct sam_cio *cio)
{
	int i;

	if (!cio)
		return 0;

	sam_hw_ring_deinit(&cio->hw_ring);

	if (cio->operations) {
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

	kfree(cio->hw_ring.cmd_desc);
	cio->hw_ring.cmd_desc = NULL;

	kfree(cio->hw_ring.result_desc);
	cio->hw_ring.result_desc = NULL;

	sam_rings[cio->id] = NULL;
	kfree(cio);

	sam_active_rings--;

	if (sam_active_rings == 0) {
		sam_hw_engine_unload();
		sam_initialized = false;
	}
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

#ifdef SAM_SA_DEBUG
	print_sam_sa_params(params);
#endif

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

#ifdef SAM_SA_DEBUG
	print_sa_params(&session->sa_params);
	print_basic_sa_params(&session->basic_params);
#endif

	session->cio = cio;
	*sa = session;

	return 0;

error_session:
	sam_session_free(session);
	return -EINVAL;
}

int sam_session_destroy(struct sam_sa *session)
{
	sam_session_free(session);

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
	struct sam_hw_cmd_desc *cmd_desc;
	struct sam_hw_res_desc *res_desc;
	PEC_CommandDescriptor_t *cmd;
	PEC_PacketParams_t pkt_params;
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
			/*pr_warning("SAM cio %d is full\n", cio->id);*/
			break;
		}
#ifdef SAM_CIO_DEBUG
		print_sam_cio_op_params(request);
#endif

		/* Get next operation structure */
		operation = &cio->operations[cio->next_request];

		/* Prepare request for submit */
		memset(&pkt_params, 0, sizeof(PEC_PacketParams_t));

		cmd = &cio->hw_ring.cmd_desc[i];
		memset(cmd, 0, sizeof(PEC_CommandDescriptor_t));
		if (sam_hw_cmd_desc_init(request, operation, cmd, &pkt_params))
			goto error_enq;

#ifdef SAM_CIO_DEBUG
		print_pkt_params(&pkt_params);
		print_cmd_desc(cmd);
#endif

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
		cmd_desc = sam_hw_cmd_desc_get(&cio->hw_ring, cio->next_request);
		res_desc = sam_hw_res_desc_get(&cio->hw_ring, cio->next_request);

		sam_hw_ring_desc_write(cmd_desc, res_desc, cmd);

		cio->next_request = sam_cio_next_idx(cio, cio->next_request);
	}
	/* submit requests */
	sam_hw_ring_submit(&cio->hw_ring, i);

	*num = (u16)i;

	return 0;

error_enq:
	return -EINVAL;
}

/* Process crypto operation result */
int sam_cio_deq(struct sam_cio *cio, struct sam_cio_op_result *results, u16 *num)
{
	PEC_Status_t rc;
	PEC_ResultStatus_t result_status;
	PEC_ResultDescriptor_t *result_desc;
	unsigned int i, count, todo, done;
	struct sam_cio_op *operation;
	struct sam_hw_res_desc *res_desc;

	/* Try to get the processed packet from the RDR */
	done = sam_hw_ring_ready_get(&cio->hw_ring);

	todo = *num;
	count = min(todo, done);

	result_desc = cio->hw_ring.result_desc;
	for (i = 0; i < count; i++) {
		struct sam_cio_op_result *result = &results[i];

		res_desc = sam_hw_res_desc_get(&cio->hw_ring, cio->next_result);
		sam_hw_res_desc_read(res_desc, result_desc);

#ifdef SAM_CIO_DEBUG
		print_result_desc(result_desc);
#endif

		operation = &cio->operations[cio->next_result];

		result->cookie = operation->cookie;
		result->out_len = result_desc->DstPkt_ByteCount;

		/* Increment next result index */
		cio->next_result = sam_cio_next_idx(cio, cio->next_result);

		rc = PEC_RD_Status_Read(result_desc, &result_status);
		if (rc != PEC_STATUS_OK) {
			pr_err("%s: PEC_RD_Status_Read failed, rc = %d\n",
				__func__, rc);
			return -EINVAL;
		}

		if (result_status.errors == 0)
			result->status = SAM_CIO_OK;
		else if (PEC_PKT_ERROR_AUTH & result_status.errors)
			result->status = SAM_CIO_ERR_ICV;
		else {
			result->status = SAM_CIO_ERR_HW;
			printf("%s: HW error 0x%x\n", __func__, result_status.errors);
			return -EINVAL;
		}

		/* Copy output data to user buffer */
		if (operation->out_frags[0].len < result_desc->DstPkt_ByteCount) {
			pr_err("%s: DstPkt_ByteCount %d bytes is larger than output buffer %d bytes\n",
				__func__, result_desc->DstPkt_ByteCount, operation->out_frags[0].len);
			return -EINVAL;
		}

#ifdef SAM_CIO_DEBUG
		printf("\nOutput DMA buffer: %d bytes\n", result_desc->DstPkt_ByteCount);
		mv_mem_dump(operation->out_frags[0].vaddr, result_desc->DstPkt_ByteCount);
#endif
		result_desc++;
	}
	/* Update RDR registers */
	sam_hw_ring_update(&cio->hw_ring, count);

	/*PEC_CDR_Regs(cio->id);*/
	/*PEC_RDR_Regs(cio->id);*/

	*num = (u16)count;

	return 0;
}
