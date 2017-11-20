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

#include "drivers/mv_sam.h"
#include "crypto/mv_md5.h"
#include "crypto/mv_sha1.h"
#include "crypto/mv_sha2.h"
#include "crypto/mv_aes.h"

#include "sam.h"

/* Maximum number of supported CIOs for all devices */
#define SAM_MAX_CIO_NUM		(SAM_HW_RING_NUM * SAM_HW_DEVICE_NUM)

u32 sam_debug_flags;

static bool		sam_initialized;
static int		sam_num_instances;
static int		sam_active_cios;
static struct sam_cio	*sam_cios[SAM_MAX_CIO_NUM];
static int		sam_num_sessions;
static struct sam_sa	*sam_sessions;
#ifdef MVCONF_SAM_STATS
static struct sam_session_stats sam_sa_stats;
#endif /* MVCONF_SAM_STATS */


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
	memset(dma_buf->vaddr, 0, buf_size);
	dma_buf->paddr = mv_sys_dma_mem_virt2phys(dma_buf->vaddr);
	dma_buf->len = buf_size;

	return 0;
}

void sam_dma_buf_free(struct sam_buf_info *dma_buf)
{
	if (dma_buf && dma_buf->vaddr)
		mv_sys_dma_mem_free(dma_buf->vaddr);
}

static struct sam_sa *sam_session_alloc(void)
{
	int i;

	for (i = 0; i < sam_num_sessions; i++) {
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

static void sam_hmac_create_iv(enum sam_auth_alg auth_alg, unsigned char key[], int key_len,
				unsigned char inner[], unsigned char outer[])
{
	if (auth_alg == SAM_AUTH_HMAC_MD5)
		mv_md5_hmac_iv(key, key_len, inner, outer);
	else if (auth_alg == SAM_AUTH_HMAC_SHA1)
		mv_sha1_hmac_iv(key, key_len, inner, outer);
	else if (auth_alg == SAM_AUTH_HMAC_SHA2_256)
		mv_sha256_hmac_iv(key, key_len, inner, outer);
	else if (auth_alg == SAM_AUTH_HMAC_SHA2_384)
		mv_sha384_hmac_iv(key, key_len, inner, outer);
	else if (auth_alg == SAM_AUTH_HMAC_SHA2_512)
		mv_sha512_hmac_iv(key, key_len, inner, outer);
	else
		printf("\n%s: Unexpected authentication algorithm - %d\n", __func__, auth_alg);
}

static void sam_gcm_create_auth_key(u8 *key, int key_len, u8 inner[])
{
	u8 key_input[16] = {0};
	u32 *ptr32 = (u32 *)inner;
	int i;

	mv_aes_ecb_encrypt(key_input, key, inner, key_len * 8);

	for (i = 0; i < sizeof(key_input) / 4; i++) {
		u32 val32;

		val32 = *ptr32;
		*ptr32++ = __bswap_32(val32);
	}
}

static int sam_session_crypto_init(struct sam_sa *session,
				   SABuilder_Params_t *sa_params)
{
	struct sam_session_params *params = &session->params;

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

	if (params->proto == SAM_PROTO_NONE)
		sa_params->IVSrc  = SAB_IV_SRC_TOKEN;

	return 0;
}

static int sam_session_auth_init(struct sam_sa *session,
				 SABuilder_Params_t *sa_params)
{
	struct sam_session_params *params = &session->params;

	if (params->auth_alg == SAM_AUTH_NONE)
		return 0;

	/* Check validity of crypto parameters */
	if (sam_max_check((int)params->auth_alg, SAM_AUTH_ALG_LAST, "auth_alg"))
		return -EINVAL;

	if ((params->auth_alg == SAM_AUTH_AES_GCM) || (params->auth_alg == SAM_AUTH_AES_GMAC)) {
		/* Generate authenticationn key from cipher key */
		sam_gcm_create_auth_key(params->cipher_key, params->cipher_key_len, session->auth_inner);
		sa_params->AuthKey1_p = session->auth_inner;
	} else {
		if (params->auth_key) {
			sam_hmac_create_iv(params->auth_alg, params->auth_key, params->auth_key_len,
				   session->auth_inner, session->auth_outer);
			sa_params->AuthKey1_p = session->auth_inner;
			sa_params->AuthKey2_p = session->auth_outer;
		}
	}
	sa_params->AuthAlgo = (SABuilder_Auth_t)params->auth_alg;

	if ((params->auth_alg == SAM_AUTH_AES_GCM) || (params->auth_alg == SAM_AUTH_AES_GMAC))
		sa_params->flags |= SAB_FLAG_SUPPRESS_HEADER;

	return 0;
}

static void sam_session_basic_init(struct sam_sa *session, SABuilder_Params_Basic_t *basic_params)
{
	struct sam_session_params *params = &session->params;

	session->post_proc_cb = NULL;
	if (params->auth_alg != SAM_AUTH_NONE) {
		basic_params->ICVByteCount = params->u.basic.auth_icv_len;
		if (params->dir == SAM_DIR_DECRYPT)
			basic_params->BasicFlags |= SAB_BASIC_FLAG_EXTRACT_ICV;

		if ((params->cipher_alg != SAM_CIPHER_NONE) && params->u.basic.auth_then_encrypt)
			basic_params->BasicFlags |= SAB_BASIC_FLAG_ENCRYPT_AFTER_HASH;
	}
}

static void sam_session_ipsec_init(struct sam_sa *session, SABuilder_Params_IPsec_t *ipsec_params)
{
	static u32 context_ref = 1;
	struct sam_session_params *params = &session->params;

	/* Create a reference to the header processor context. */
	ipsec_params->ContextRef = context_ref++;

	ipsec_params->IPsecFlags |= SAB_IPSEC_PROCESS_IP_HEADERS;
	ipsec_params->SeqNum = params->u.ipsec.seq;
	if (params->u.ipsec.is_tunnel) {
		ipsec_params->SrcIPAddr_p = params->u.ipsec.tunnel.u.ipv4.sip;
		ipsec_params->DestIPAddr_p = params->u.ipsec.tunnel.u.ipv4.dip;
	}
	ipsec_params->PadAlignment = 0;

	session->post_proc_cb = NULL;
	if ((!params->u.ipsec.is_tunnel) && params->dir == SAM_DIR_DECRYPT) {
		if (params->u.ipsec.is_ip6)
			session->post_proc_cb = sam_ipsec_ip6_transport_in_post_proc;
		else
			session->post_proc_cb = sam_ipsec_ip4_transport_in_post_proc;
	} else if ((params->u.ipsec.is_tunnel) && params->dir == SAM_DIR_ENCRYPT) {
		sam_ipsec_prepare_tunnel_header(params, session->tunnel_header);
		if (params->u.ipsec.is_ip6)
			session->post_proc_cb = sam_ipsec_ip6_tunnel_out_post_proc;
		else
			session->post_proc_cb = sam_ipsec_ip4_tunnel_out_post_proc;
	} else if ((params->u.ipsec.is_tunnel) && params->dir == SAM_DIR_DECRYPT) {
		if (params->u.ipsec.is_ip6)
			session->post_proc_cb = sam_ipsec_ip6_tunnel_in_post_proc;
		else {
			if (params->u.ipsec.tunnel.copy_dscp || params->u.ipsec.tunnel.copy_df)
				session->post_proc_cb = sam_ipsec_ip4_tunnel_in_post_proc;
		}
	}
}

static int sam_token_context_build(struct sam_sa *session)
{
	int rc;

	rc = TokenBuilder_GetContextSize(&session->sa_params, &session->tcr_words);
	if (rc != 0) {
		pr_err("%s: TokenBuilder_GetContextSize return error, rc = %d\n",
			__func__, rc);
		return rc;
	}
	if (session->tcr_words > SAM_TCR_DATA_SIZE / 4) {
		pr_err("%s: TCR size %d words is too big. Maximum = %d words\n",
			__func__, session->tcr_words, SAM_TCR_DATA_SIZE / 4);
		return rc;
	}
	/* Clear TCR data buffer used allocated for SA */
	memset(session->tcr_data, 0, 4 * session->tcr_words);

	rc = TokenBuilder_BuildContext(&session->sa_params, session->tcr_data);
	if (rc != 0) {
		pr_err("%s: TokenBuilder_BuildContext failed, rc = %d\n", __func__, rc);
		return rc;
	}

	rc = TokenBuilder_GetSize(session->tcr_data, &session->token_words);
	if (rc != 0) {
		pr_err("%s:: TokenBuilder_GetSize failed.\n", __func__);
		return rc;
	}

	if (session->token_words > SAM_TOKEN_DMABUF_SIZE / 4) {
		pr_err("%s: Token size %d words is too big. Maximum = %d words\n",
			__func__, session->token_words, SAM_TOKEN_DMABUF_SIZE / 4);
		return -EINVAL;
	}
	return 0;
}

static int sam_hw_cmd_token_build(struct sam_cio *cio, struct sam_cio_op_params *request,
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
			copylen += session->params.u.basic.auth_icv_len;
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
		token_params.AdditionalValue = session->params.u.basic.auth_aad_len;
	} else if (request->auth_len && request->cipher_len)
		token_params.AdditionalValue = request->auth_len - request->cipher_len;

#ifdef MVCONF_SAM_DEBUG
	if (sam_debug_flags & SAM_CIO_DEBUG_FLAG)
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
	if (session->cio != cio) {
#ifdef MVCONF_SAM_DEBUG
		if (session->cio != NULL)
			pr_warn("Session is moved from cio=%d:%d to cio=%d:%d\n",
				session->cio->hw_ring.device, session->cio->hw_ring.ring,
				cio->hw_ring.device, cio->hw_ring.ring);
#endif
		session->cio = cio;
	} else
		operation->token_header_word |= SAM_TOKEN_REUSE_AUTO_MASK;

	operation->copy_len = copylen;

#ifdef MVCONF_SAM_DEBUG
	if (sam_debug_flags & SAM_CIO_DEBUG_FLAG) {
		print_sam_cio_operation_info(operation);

		printf("\nToken DMA buffer: %d words, physAddr = %p\n",
			operation->token_words, (void *)operation->token_buf.paddr);
		mv_mem_dump_words(operation->token_buf.vaddr, operation->token_words, 0);
	}
#endif /* MVCONF_SAM_DEBUG */

	return 0;
}

/*********************** Public functions implementation *******************/

int sam_get_capability(struct sam_capability *capa)
{
	capa->cipher_algos = BIT(SAM_CIPHER_3DES) | BIT(SAM_CIPHER_AES);
	capa->cipher_modes = BIT(SAM_CIPHER_ECB) | BIT(SAM_CIPHER_CBC) | BIT(SAM_CIPHER_GCM);

	/* HASH only */
	capa->auth_algos = BIT(SAM_AUTH_HASH_MD5) | BIT(SAM_AUTH_HASH_SHA1) |
			   BIT(SAM_AUTH_HASH_SHA2_224) | BIT(SAM_AUTH_HASH_SHA2_256) |
			   BIT(SAM_AUTH_HASH_SHA2_384) | BIT(SAM_AUTH_HASH_SHA2_512);
	/* HMAC */
	capa->auth_algos |= BIT(SAM_AUTH_HMAC_MD5) | BIT(SAM_AUTH_HMAC_SHA1) |
			    BIT(SAM_AUTH_HMAC_SHA2_224) | BIT(SAM_AUTH_HMAC_SHA2_256) |
			    BIT(SAM_AUTH_HMAC_SHA2_384) | BIT(SAM_AUTH_HMAC_SHA2_512);
	/* AES-GCM */
	capa->auth_algos |= BIT(SAM_AUTH_AES_GCM);

	return 0;
}

u32 sam_get_num_cios(u32 inst)
{
	return sam_hw_get_rings_num(inst);
}

u8 sam_get_num_inst(void)
{
	int i;
	u8 num = 0;

	if (sam_num_instances)
		return sam_num_instances;

	for (i = 0; i < SAM_HW_DEVICE_NUM; i++) {
		if (sam_hw_device_exist(i, NULL))
			num++;
	}
	sam_num_instances = num;

	return sam_num_instances;
}

int sam_init(struct sam_init_params *params)
{
	int i;
	u32 num_sessions = params->max_num_sessions;

	if (sam_initialized) {
		pr_warn("SAM driver already initialized\n");
		return -EINVAL;
	}
	/* Allocate configured number of sam_sa structures */
	sam_sessions = kcalloc(num_sessions, sizeof(struct sam_sa), GFP_KERNEL);
	if (!sam_sessions)
		goto err;

	/* Allocate DMA buffer for each session */
	for (i = 0; i < num_sessions; i++) {
		if (sam_dma_buf_alloc(SAM_SA_DMABUF_SIZE, &sam_sessions[i].sa_buf)) {
			pr_err("DMA buffers (%d bytes) allocated only for %d of %d sessions\n",
				SAM_SA_DMABUF_SIZE, i, num_sessions);
			num_sessions = i;
			break;
		}
	}
	pr_debug("DMA buffers allocated for %d sessions (%d bytes)\n",
		num_sessions, SAM_SA_DMABUF_SIZE);

	sam_num_sessions = num_sessions;
	sam_initialized = true;
	return 0;
err:
	/* Release all allocated resources */
	sam_deinit();

	return -ENOMEM;
}

void sam_deinit(void)
{
	int i;

	if (sam_sessions) {
		for (i = 0; i < sam_num_sessions; i++)
			if (sam_sessions[i].is_valid)
				pr_warn("All sessions must be deleted before %s called\n", __func__);
			sam_dma_buf_free(&sam_sessions[i].sa_buf);

		kfree(sam_sessions);
	}
	sam_initialized = true;
}

int sam_cio_init(struct sam_cio_params *params, struct sam_cio **cio)
{
	int i, device, ring, cio_idx, scanned;
	struct sam_cio	*local_cio;

	/* Parse match string to ring number */
	scanned = sscanf(params->match, "cio-%d:%d\n", &device, &ring);
	if (scanned != 2) {
		pr_err("Invalid match string %s. Expected: cio-0:X\n",
			params->match);
		return -EINVAL;
	}
	/* Check validity of device and ring values */
	if (device >= sam_get_num_inst()) {
		pr_err("SAM device #%d is out of valid range [0 .. %d]",
			device, sam_get_num_inst() - 1);
		return -EINVAL;
	}
	if (ring >= sam_get_num_cios(device)) {
		pr_err("SAM CIO #%d is out of valid range [0 .. %d]",
			device, sam_get_num_cios(device) - 1);
		return -EINVAL;
	}
	cio_idx = sam_cio_free_idx_get();
	if (cio_idx < 0) {
		pr_err("No free place for new CIO: active_cios = %d, max_cios = %d\n",
			sam_active_cios, SAM_MAX_CIO_NUM);
		return -EINVAL;
	}

	/* Allocate single sam_cio structure */
	local_cio = kcalloc(1, sizeof(struct sam_cio), GFP_KERNEL);
	if (!local_cio)
		return -ENOMEM;

	/* Initialize HW ring */
	if (sam_hw_ring_init(device, ring, params, &local_cio->hw_ring))
		goto err;

	/* Save configured CIO params */
	local_cio->params = *params;

	local_cio->idx = cio_idx;

	/* Allocate array of sam_cio_op structures in size of CIO ring */
	local_cio->operations = kcalloc(params->size, sizeof(struct sam_cio_op), GFP_KERNEL);
	if (!local_cio->operations)
		goto err;

	/* Allocate DMA buffers for Tokens (one per operation) */
	for (i = 0; i < params->size; i++) {
		if (sam_dma_buf_alloc(SAM_TOKEN_DMABUF_SIZE, &local_cio->operations[i].token_buf)) {
			pr_err("Can't allocate DMA buffer (%d bytes) for Token #%d\n",
				SAM_TOKEN_DMABUF_SIZE, i);
			goto err;
		}
	}
	pr_debug("DMA buffers allocated for %d operations. Tokens - %d bytes\n",
		i, SAM_TOKEN_DMABUF_SIZE);

	*cio = local_cio;
	sam_cios[cio_idx] = local_cio;
	sam_active_cios++;

	return 0;

err:
	/* Release all allocated resources */
	sam_cio_deinit(local_cio);

	return -ENOMEM;
}

int sam_cio_flush(struct sam_cio *cio)
{
	int rc;
	u32 count = 1000;
	u16 num, total = 0;

	/* Wait for completion of all operations */
	while (!sam_cio_is_empty(cio)) {
		num = cio->params.size;
		rc = sam_cio_deq(cio, NULL, &num);
		if (rc) {
			pr_err("%s: dequeue error %d\n", __func__, rc);
			return rc;
		}

		if (num) {
			total += num;
			count = 1000; /* restart counter */
		}

		if (count-- == 0) {
			pr_err("%s: Timeout\n", __func__);
			return -EINVAL;
		}
	}
	if (total)
		printf("%s: %d results were flushed\n", cio->params.match, total);

	return 0;
}

int sam_cio_deinit(struct sam_cio *cio)
{
	int i;

	if (!cio)
		return 0;

	if (sam_cios[cio->idx]) {
		sam_cio_flush(cio);

		sam_hw_ring_deinit(&cio->hw_ring);
		sam_cios[cio->idx] = NULL;
		sam_active_cios--;
	}

	if (cio->operations) {
		for (i = 0; i < cio->params.size; i++)
			sam_dma_buf_free(&cio->operations[i].token_buf);

		kfree(cio->operations);
		cio->operations = NULL;
	}
	kfree(cio);

	return 0;
}

int sam_session_create(struct sam_session_params *params, struct sam_sa **sa)
{
	SABuilder_Direction_t direction = (SABuilder_Direction_t)params->dir;
	struct sam_sa *session;
	int rc;

	/* Find free session structure */
	session = sam_session_alloc();
	if (!session) {
		pr_err("%s: Can't get free session\n", __func__);
		return -EBUSY;
	}
	/* Clear session structure */
	memset(&session->sa_params, 0, sizeof(session->sa_params));
	memset(&session->u, 0, sizeof(session->u));

	/* Save session params */
	session->params = *params;

	if (params->proto == SAM_PROTO_NONE) {
		/* Initialize sa_params and basic_params */
		if (SABuilder_Init_Basic(&session->sa_params, &session->u.basic_params, direction) != SAB_STATUS_OK)
			goto error_session;

		/* Update basic params with session information */
		sam_session_basic_init(session, &session->u.basic_params);
	} else if (params->proto == SAM_PROTO_IPSEC) {
		if (SABuilder_Init_ESP(&session->sa_params, &session->u.ipsec_params, params->u.ipsec.spi,
				params->u.ipsec.is_tunnel ? SAB_IPSEC_TUNNEL : SAB_IPSEC_TRANSPORT,
				params->u.ipsec.is_ip6 ? SAB_IPSEC_IPV6 : SAB_IPSEC_IPV4, direction) != SAB_STATUS_OK)
			goto error_session;

		/* Update ipsec params with session information */
		sam_session_ipsec_init(session, &session->u.ipsec_params);
	} else {
		pr_err("Unexpected protocol %d\n", params->proto);
		goto error_session;
	}

	/* Update sa_params with session information */
	if (sam_session_crypto_init(session, &session->sa_params))
		goto error_session;

	if (sam_session_auth_init(session, &session->sa_params))
		goto error_session;

#ifdef MVCONF_SAM_DEBUG
	if (sam_debug_flags & SAM_SA_DEBUG_FLAG)
		print_sam_sa(session);
#endif /* MVCONF_SAM_DEBUG */

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

	if (params->proto == SAM_PROTO_NONE)
		sam_token_context_build(session);

	/* Clear SA DMA buffer first */
	memset(session->sa_buf.vaddr, 0, session->sa_words * 4);

	/* build the SA data and init according the parameters */
	rc = SABuilder_BuildSA(&session->sa_params, (u32 *)session->sa_buf.vaddr, NULL, NULL);
	if (rc != 0) {
		pr_err("%s: SABuilder_BuildSA failed, rc = %d\n", __func__, rc);
		goto error_session;
	}
	/* Swap session data if needed */
	sam_htole32_multi(session->sa_buf.vaddr, session->sa_words);

#ifdef MVCONF_SAM_DEBUG
	if (sam_debug_flags & SAM_SA_DEBUG_FLAG)
		print_sa_builder_params(session);
#endif /* MVCONF_SAM_DEBUG */

	SAM_STATS(sam_sa_stats.sa_add++);

	session->cio = NULL;
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

	if (cio && (cio->hw_ring.type == HW_EIP197)) {
		/* Check maximum number of pending requests */
		if (sam_cio_is_full(cio)) {
			SAM_STATS(session->cio->stats.enq_full++);
			return -EINVAL;
		}

		/* Get next operation structure */
		operation = &cio->operations[cio->next_request];
		operation->sa = session;
		operation->num_bufs = 0;

		/* Invalidate session in HW */
		sam_hw_session_invalidate(&session->cio->hw_ring, &session->sa_buf, cio->next_request);
		SAM_STATS(sam_sa_stats.sa_inv++);

		cio->next_request = sam_cio_next_idx(cio, cio->next_request);
	} else {
		sam_session_free(session);
		SAM_STATS(sam_sa_stats.sa_del++);
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
	int i, j, todo, err = 0;

	todo = *num;
	if (todo >= cio->params.size)
		todo = cio->params.size - 1;

	for (i = 0; i < todo; i++) {
		request = &requests[i];

		/* Check request validity */
		err = sam_cio_check_op_params(request);
		if (err)
			break;

		/* Check maximum number of pending requests */
		if (sam_cio_is_full(cio)) {
			SAM_STATS(cio->stats.enq_full++);
			break;
		}
#ifdef MVCONF_SAM_DEBUG
		if (sam_debug_flags & SAM_CIO_DEBUG_FLAG)
			print_sam_cio_op_params(request);
#endif /* MVCONF_SAM_DEBUG */

		/* Get next operation structure */
		operation = &cio->operations[cio->next_request];

		err = sam_hw_cmd_token_build(cio, request, operation);
		if (err)
			break;

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
		if (sam_debug_flags & SAM_CIO_DEBUG_FLAG) {
			struct sam_hw_cmd_desc *cmd_desc = sam_hw_cmd_desc_get(&cio->hw_ring, cio->next_request);
			struct sam_hw_res_desc *res_desc = sam_hw_res_desc_get(&cio->hw_ring, cio->next_request);

			print_result_desc(res_desc, 1);
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

	return err;
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

		i++;
		operation = &cio->operations[cio->next_result];
		if (operation->num_bufs == 0) {
			/* Inalidate cache entry finished - free session */
			sam_session_free(operation->sa);
			SAM_STATS(sam_sa_stats.sa_del++);
			cio->next_result = sam_cio_next_idx(cio, cio->next_result);
			continue;
		}
#ifdef MVCONF_SAM_DEBUG
		if (sam_debug_flags & SAM_CIO_DEBUG_FLAG)
			print_result_desc(res_desc, 0);
#endif
		/* Increment next result index */
		cio->next_result = sam_cio_next_idx(cio, cio->next_result);
		if (!result) /* Flush cio */
			continue;

		sam_hw_res_desc_read(res_desc, result);
		if (operation->sa->post_proc_cb)
			operation->sa->post_proc_cb(operation, res_desc, result);

		out_len = result->out_len;

		SAM_STATS(cio->stats.deq_bytes += out_len);

		result->cookie = operation->cookie;

		/* Check output buffer size */
		if (operation->out_frags[0].len < out_len) {
			pr_err("%s: out_len %d bytes is larger than output buffer %d bytes\n",
				__func__, out_len, operation->out_frags[0].len);
			out_len = operation->out_frags[0].len;
		}

#ifdef MVCONF_SAM_DEBUG
		if (sam_debug_flags & SAM_CIO_DEBUG_FLAG) {
			printf("\nOutput DMA buffer: %d bytes, physAddr = %p\n",
				out_len, (void *)operation->out_frags[0].paddr);
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

int sam_cio_create_event(struct sam_cio *cio, struct sam_cio_event_params *params, struct mv_sys_event **ev)
{
	int err;
	struct mv_sys_event_params ev_params;

	snprintf(ev_params.name, sizeof(ev_params.name), "%s_%d:%d",
			(cio->hw_ring.type == HW_EIP197) ? "eip197" : "eip97",
			cio->hw_ring.device, cio->hw_ring.ring);

	err = mv_sys_event_create(&ev_params, ev);
	if (err) {
		pr_err("Can't open CIO event: %s\n", ev_params.name);
		return err;
	}
	if (params) {
		cio->pkt_coal = params->pkt_coal;
		cio->usec_coal = params->usec_coal;
	} else {
		cio->pkt_coal = SAM_ISR_PKTS_COAL_DEF;
		cio->usec_coal = SAM_ISR_TIME_COAL_DEF;
	}
	/* IRQ will not occurred until triggered */
	sam_hw_ring_enable_irq(&cio->hw_ring);
	sam_hw_ring_prepare_rdr_thresh(&cio->hw_ring, cio->pkt_coal, cio->usec_coal);

	pr_info("CIO event created for %s\n", ev_params.name);
	return 0;
}

int sam_cio_delete_event(struct sam_cio *cio, struct mv_sys_event *ev)
{
	sam_hw_ring_disable_irq(&cio->hw_ring);

	return mv_sys_event_destroy(ev);
}

int sam_cio_set_event(struct sam_cio *cio, struct mv_sys_event *ev, int en)
{
	if (en)
		sam_hw_ring_trigger_irq(&cio->hw_ring);
	else {
		/* Untrigger IRQ */
		sam_hw_ring_prepare_rdr_thresh(&cio->hw_ring, 0, 0);
		sam_hw_ring_trigger_irq(&cio->hw_ring);
		sam_hw_ring_ack_irq(&cio->hw_ring);
	}

	return 0;
}

int sam_set_debug_flags(u32 debug_flags)
{
#ifdef MVCONF_SAM_DEBUG
	sam_debug_flags = debug_flags;
	return 0;
#else
	return -ENOTSUP;
#endif /* MVCONF_SAM_DEBUG */
}

int sam_cio_show_regs(struct sam_cio *cio, enum sam_cio_regs regs)
{
#ifdef MVCONF_SAM_DEBUG
	switch (regs) {
	case SAM_CIO_REGS_ALL:
		sam_hw_cdr_regs_show(&cio->hw_ring);
		sam_hw_rdr_regs_show(&cio->hw_ring);
		break;
	case SAM_CIO_REGS_CDR:
		sam_hw_cdr_regs_show(&cio->hw_ring);
		break;
	case SAM_CIO_REGS_RDR:
		sam_hw_rdr_regs_show(&cio->hw_ring);
		break;
	default:
		pr_err("Unsupported group of CIO registers - %u\n", regs);
		break;
	}
	return 0;
#else
	return -ENOTSUP;
#endif /* MVCONF_SAM_DEBUG */
}

int sam_session_get_stats(struct sam_session_stats *stats, int reset)
{
#ifdef MVCONF_SAM_STATS
	memcpy(stats, &sam_sa_stats, sizeof(sam_sa_stats));

	if (reset)
		memset(&sam_sa_stats, 0, sizeof(sam_sa_stats));

	return 0;
#else
	return -ENOTSUP;
#endif /* MVCONF_SAM_STATS */
}

int sam_cio_get_stats(struct sam_cio *cio, struct sam_cio_stats *stats, int reset)
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
