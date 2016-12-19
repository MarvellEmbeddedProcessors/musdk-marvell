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

/*#define SAM_DMA_DEBUG*/
/*#define SAM_CIO_DEBUG*/
/*#define SAM_SA_DEBUG*/

static struct sam_cio	*sam_ring;
static struct sam_sa	*sam_sessions;


static void sam_dmabuf_free(struct sam_dmabuf *dmabuf)
{
#ifdef SAM_DMA_DEBUG
	pr_info("free DMA buffer: size = %d bytes, vaddr = %p, handle = %p\n",
		dmabuf->size, dmabuf->host_addr.p, dmabuf->hndl.p);
#endif
	if (dmabuf->size) {
		DMABuf_Release(dmabuf->hndl);
		dmabuf->size = 0;
	}
}

static int sam_dmabuf_alloc(u32 buf_size, struct sam_dmabuf *dmabuf)
{
	DMABuf_Status_t dma_status;
	DMABuf_Properties_t dma_properties = {0, 0, 0, 0};

	dma_properties.fCached   = true;
	dma_properties.Alignment = SAM_DMABUF_ALIGN;
	dma_properties.Size      = buf_size;
	dma_properties.Bank      = SAM_DMA_BANK_PKT;

	dma_status = DMABuf_Alloc(dma_properties,
				&dmabuf->host_addr,
				&dmabuf->hndl);
	if (dma_status != DMABUF_STATUS_OK) {
		pr_err("Failed to allocate DMA buffer of %d bytes. error = %d\n",
			buf_size, dma_status);
		return -ENOMEM;
	}
	dmabuf->size = buf_size;

#ifdef SAM_DMA_DEBUG
	pr_info("allocate DMA buffer: size = %d bytes, vaddr = %p, handle = %p\n",
		buf_size, dmabuf->host_addr.p, dmabuf->hndl.p);
#endif
	return 0;
}

static int sam_hw_engine_load(void)
{
	pr_info("Load SAM HW engine\n");

	if (Driver197_Init()) {
		pr_err("Can't init eip197 driver\n");
		return -ENODEV;
	}
	return 0;
}

static int sam_hw_engine_unload(void)
{
	pr_info("Unload SAM HW engine\n");
	Driver197_Exit();

	return 0;
}

static int sam_hw_ring_init(u32 ring)
{
	PEC_Status_t status;
	PEC_InitBlock_t init_block = {0, 0};
	u32 count = SAM_HW_RING_RETRY_COUNT;

	while (count > 0) {
		status = PEC_Init(ring, &init_block);
		if (status == PEC_STATUS_OK) {
			pr_info("EIP197 ring #%d loaded\n", ring);
			return 0;
		}

		if (status == PEC_STATUS_BUSY) {
			udelay(SAM_HW_RING_RETRY_US);
			count--;
			continue;
		}
		pr_err("Can't initialize ring #%d. error = %d\n", ring, status);
		return -ENODEV;
	} /* while */

	/* Timeout */
	pr_err("Can't initialize HW ring #%d. %d msec timeout expired\n",
		ring, SAM_HW_RING_RETRY_US * SAM_HW_RING_RETRY_COUNT / 1000);

	return -ENODEV;
}

static int sam_hw_ring_deinit(int ring)
{
	PEC_Status_t status;
	u32 count = SAM_HW_RING_RETRY_COUNT;

	while (count > 0) {
		status = PEC_UnInit(ring);
		if (status == PEC_STATUS_OK) {
			pr_info("EIP197 ring #%d unloaded\n", ring);
			return 0;
		}

		if (status == PEC_STATUS_BUSY) {
			udelay(SAM_HW_RING_RETRY_US);
			count--;
			continue;
		}
		pr_err("Can't un-initialize ring #%d. error = %d\n", ring, status);
		return -ENODEV;
	} /* while */

	/* Timeout */
	pr_err("Can't un-initialize ring #%d. %d msec timeout expired\n",
		ring, SAM_HW_RING_RETRY_US * SAM_HW_RING_RETRY_COUNT / 1000);

	return -ENODEV;
}

static struct sam_sa *sam_session_alloc(struct sam_cio *cio)
{
	int i;

	for (i = 0; i < cio->params.num_sessions; i++) {
		if (!sam_sessions[i].is_valid) {
			sam_sessions[i].is_valid = true;
			return &sam_sessions[i];
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

	/* process AAD  - TBD */

	/* Copy data from user buffer to DMA buffer */
	memcpy(operation->data_dmabuf.host_addr.p, request->src->vaddr, copylen);

#ifdef SAM_CIO_DEBUG
	print_token_params(&token_params);

	printf("\nInput DMA buffer: %d bytes\n", copylen);
	mv_mem_dump(operation->data_dmabuf.host_addr.p, copylen);
#endif

	rc = TokenBuilder_BuildToken(session->tcr_data, (u8 *)operation->data_dmabuf.host_addr.p,
				     copylen, &token_params,
				     operation->token_dmabuf.host_addr.p,
				     &token_words, &token_header_word);
	if (rc != TKB_STATUS_OK) {
		pr_err("%s: TokenBuilder_BuildToken failed, rc = %d\n",
			__func__, rc);
		return -EINVAL;
	}

	cmd_desc->SA_Handle1       = session->sa_dmabuf.hndl;
	cmd_desc->SA_WordCount     = session->sa_words;
	cmd_desc->SA_Handle2       = DMABuf_NULLHandle;
	cmd_desc->Token_Handle     = operation->token_dmabuf.hndl;
	cmd_desc->SrcPkt_Handle    = operation->data_dmabuf.hndl;
	cmd_desc->DstPkt_Handle    = operation->data_dmabuf.hndl;
	cmd_desc->User_p           = (void *)operation;
	cmd_desc->SrcPkt_ByteCount = copylen;
	cmd_desc->Token_WordCount  = token_words;
	pkt_params->HW_Services  = FIRMWARE_EIP207_CMD_PKT_LAC;
	pkt_params->TokenHeaderWord = token_header_word;

	return 0;
}

/*********************** Public functions implementation *******************/

int sam_cio_init(struct sam_cio_params *params, struct sam_cio **cio)
{
	int i;

	/* Load SAM HW engine */
	if (sam_hw_engine_load())
		return -EINVAL;

	/* Only one instance of CIO is supported */
	if (sam_ring != NULL) {
		pr_err("sam_cio_init is already called\n");
		return -EEXIST;
	}
	/* Check valid range for CIO ring */
	if (params->id >= SAM_HW_RING_NUM) {
		pr_err("sam_cio ring #%d is out range: 0 ... %d\n",
			params->id, SAM_HW_RING_NUM);
		return -EINVAL;
	}
	/* Check ring size and number of sessions with HW max values */
	if (params->size > SAM_HW_RING_SIZE) {
		/* SW value can't exceed HW restriction */
		pr_warning("Warning! Ring size %d is too large. Set to maximum = %d\n",
			params->size, SAM_HW_RING_SIZE);
		params->size = SAM_HW_RING_SIZE;
	}

	if (params->num_sessions > SAM_HW_SA_NUM) {
		/* SW value can't exceed HW restriction */
		pr_warning("Warning! Number of sessions %d is too large. Set to maximum = %d\n",
			params->num_sessions, SAM_HW_SA_NUM);
		params->num_sessions = SAM_HW_SA_NUM;
	}

	/* Initialize HW ring */
	if (sam_hw_ring_init(params->id))
		goto err;

	/* Allocate single sam_cio structure */
	sam_ring = kcalloc(1, sizeof(struct sam_cio), GFP_KERNEL);
	if (!sam_ring) {
		pr_err("Can't allocate %lu bytes for sam_cio structure\n",
			 sizeof(struct sam_cio));
		goto err;
	}
	/* Save configured CIO params */
	sam_ring->params = *params;

	/* Allocate configured number of sam_sa structures */
	sam_sessions = kcalloc(params->num_sessions, sizeof(struct sam_sa), GFP_KERNEL);
	if (!sam_sessions) {
		pr_err("Can't allocate %u * %lu bytes for sam_sa structures\n",
			params->num_sessions, sizeof(struct sam_sa));
		goto err;
	}

	/* Allocate DMA buffer for each session */
	for (i = 0; i < params->num_sessions; i++) {
		if (sam_dmabuf_alloc(SAM_SA_DMABUF_SIZE, &sam_sessions[i].sa_dmabuf)) {
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

	/* Allocate DMA buffers for Token and Data for each operation */
	for (i = 0; i < params->size; i++) {
		if (sam_dmabuf_alloc(SAM_TOKEN_DMABUF_SIZE, &sam_ring->operations[i].token_dmabuf)) {
			pr_err("Can't allocate DMA buffer (%d bytes) for Token #%d\n",
				SAM_TOKEN_DMABUF_SIZE, i);
			goto err;
		}

		if (sam_dmabuf_alloc(params->max_buf_size, &sam_ring->operations[i].data_dmabuf)) {
			pr_err("Can't allocate DMA buffer (%d bytes) for Buffer #%d\n",
				params->max_buf_size, i);
			goto err;
		}
	}
	pr_info("DMA buffers allocated for %d operations. Tokens - %d bytes, Buffers - %d bytes\n",
		i, SAM_TOKEN_DMABUF_SIZE, params->max_buf_size);

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

	sam_hw_ring_deinit(cio->params.id);

	if (sam_sessions) {
		for (i = 0; i < cio->params.num_sessions; i++)
			sam_dmabuf_free(&sam_sessions[i].sa_dmabuf);

		kfree(sam_sessions);
		sam_sessions = NULL;
	}

	if (cio->operations) {
		for (i = 0; i < cio->params.size; i++) {
			sam_dmabuf_free(&cio->operations[i].token_dmabuf);
			sam_dmabuf_free(&cio->operations[i].data_dmabuf);
		}
		kfree(sam_ring->operations);
	}
	kfree(cio);
	sam_ring = NULL;

	sam_hw_engine_unload();

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
	memset(session->sa_dmabuf.host_addr.p, 0, 4 * session->sa_words);

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
	rc = SABuilder_BuildSA(&session->sa_params,
				(u32 *)session->sa_dmabuf.host_addr.p, NULL, NULL);
	if (rc != 0) {
		pr_err("%s: SABuilder_BuildSA failed, rc = %d\n", __func__, rc);
		goto error_session;
	}

	/* Register the SA. */
	rc = PEC_SA_Register(cio->params.id, session->sa_dmabuf.hndl,
			     DMABuf_NULLHandle, DMABuf_NULLHandle);
	if (rc != PEC_STATUS_OK) {
		pr_err("%s: PEC_SA_Register on ring %d failed. rc = %d\n",
			__func__, cio->params.id, rc);
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
	struct sam_cio *cio = session->cio;
	PEC_Status_t rc;

	/* Unregister the SA. */
	rc = PEC_SA_UnRegister(cio->params.id, session->sa_dmabuf.hndl,
				DMABuf_NULLHandle, DMABuf_NULLHandle);
	if (rc != PEC_STATUS_OK) {
		pr_err("%s: PEC_SA_UnRegister on ring %d failed. rc = %d\n",
			__func__, cio->params.id, rc);
		return -EINVAL;
	}
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

int sam_cio_enq(struct sam_cio *cio, struct sam_cio_op_params *request, u16 *num)
{
	struct sam_cio_op *operation;
	PEC_CommandDescriptor_t cmd;
	PEC_PacketParams_t pkt_params;
	PEC_Status_t rc;
	unsigned int count = 0;
	int i, err;

	/* Check request validity */
	err = sam_cio_check_op_params(request);
	if (err)
		return err;

	/* Check maximum number of pending requests */
	if (sam_cio_is_full(cio)) {
		pr_warning("SAM cio %d is full\n", cio->params.id);
		return -EBUSY;
	}
#ifdef SAM_CIO_DEBUG
	print_sam_cio_op_params(request);
#endif

	/* Get next operation structure */
	operation = &cio->operations[cio->next_request];

	cio->next_request = sam_cio_next_idx(cio, cio->next_request);

	/* Prepare request for submit */
	memset(&cmd, 0, sizeof(cmd));
	memset(&pkt_params, 0, sizeof(pkt_params));

	if (sam_hw_cmd_desc_init(request, operation, &cmd, &pkt_params))
		goto error_enq;

#ifdef SAM_CIO_DEBUG
	print_pkt_params(&pkt_params);
	print_cmd_desc(&cmd);
#endif

	/* Save some fields from request needed for result processing */
	operation->sa = request->sa;
	operation->cookie = request->cookie;
	operation->num_bufs = request->num_bufs;
	operation->auth_icv_offset = request->auth_icv_offset;
	for (i = 0;  i < request->num_bufs; i++) {
		operation->out_frags[i].vaddr = request->dst[i].vaddr;
		operation->out_frags[i].len = request->dst[i].len;
	}

	/* submit request */
	rc = PEC_CD_Control_Write(&cmd, &pkt_params);
	if (rc != PEC_STATUS_OK) {
		pr_err("%s: PEC_CD_Control_Write failed, rc = %d\n",
			__func__, rc);
		goto error_enq;
	}

	rc = PEC_Packet_Put(cio->params.id, &cmd, 1, &count);
	if (rc != PEC_STATUS_OK) {
		pr_err("%s: PEC_Packet_Put failed, rc = %d, count = %d\n",
			__func__, rc, count);
		goto error_enq;
	}
	*num = (u16)count;

	return 0;

error_enq:
	return -EINVAL;
}

/* Process crypto operation result */
int sam_cio_deq(struct sam_cio *cio, struct sam_cio_op_result *result, u16 *num)
{
	PEC_Status_t rc;
	PEC_ResultDescriptor_t result_desc;
	PEC_ResultStatus_t result_status;
	unsigned int count;
	struct sam_cio_op *operation;

	/* Try to get the processed packet from the RDR */
	rc = PEC_Packet_Get(cio->params.id, &result_desc, 1, &count);
	if (rc != PEC_STATUS_OK) {
		pr_err("%s: PEC_Packet_Get failed, rc = %d\n", __func__, rc);
		return -EINVAL;
	}

	if (count == 0) /* No results are ready */
		return -EBUSY;

#ifdef SAM_CIO_DEBUG
	print_result_desc(&result_desc);
#endif
	*num = (u16)count;

	operation = result_desc.User_p;
	rc = PEC_RD_Status_Read(&result_desc, &result_status);
	if (rc != PEC_STATUS_OK) {
		pr_err("%s: PEC_RD_Status_Read failed, rc = %d\n",
			__func__, rc);
		return -EINVAL;
	}
	result->cookie = operation->cookie;

	/* Increment next result index */
	cio->next_result = sam_cio_next_idx(cio, cio->next_result);

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
	if (operation->out_frags[0].len < result_desc.DstPkt_ByteCount) {
		pr_err("%s: DstPkt_ByteCount %d bytes is larger than output buffer %d bytes\n",
			__func__, result_desc.DstPkt_ByteCount, operation->out_frags[0].len);
		return -EINVAL;
	}
	memcpy(operation->out_frags[0].vaddr, result_desc.DstPkt_p,
		result_desc.DstPkt_ByteCount);

#ifdef SAM_CIO_DEBUG
	printf("\nOutput DMA buffer: %d bytes\n", result_desc.DstPkt_ByteCount);
	mv_mem_dump(result_desc.DstPkt_p, result_desc.DstPkt_ByteCount);
#endif

	return 0;
}

