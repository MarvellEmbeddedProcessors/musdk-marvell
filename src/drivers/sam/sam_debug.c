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
#include "lib/lib_misc.h"
#include "drivers/mv_sam.h"
#include "sam.h"
#include "sam_hw.h"

static void print_basic_sa_params(SABuilder_Params_Basic_t *params)
{
	pr_info("\n");
	pr_info("----------- SABuilder_Params_Basic_t basic_params ---------\n");
	pr_info("BasicFlags              = %d\n", params->BasicFlags);
	pr_info("DigestBlockCount        = %d\n", params->DigestBlockCount);
	pr_info("ICVByteCount            = %d\n", params->ICVByteCount);
	pr_info("bearer                  = %d\n", params->bearer);
	pr_info("direction               = %d\n", params->direction);
	pr_info("fresh                   = %d\n", params->fresh);
	pr_info("\n");
}

static void print_ipsec_sa_params(SABuilder_Params_IPsec_t *params)
{
	pr_info("\n");
	pr_info("----------- SABuilder_Params_IPsec_t ipsec_params ---------\n");
	pr_info("spi                     = 0x%x\n", params->spi);
	pr_info("IPsecFlags              = 0x%x\n", params->IPsecFlags);
	pr_info("SeqNum                  = 0x%x\n", params->SeqNum);
	pr_info("SeqNumHi                = 0x%x\n", params->SeqNumHi);
	pr_info("PadAlignment            = %d\n", params->PadAlignment);
	pr_info("ICVByteCount            = %d\n", params->ICVByteCount);
	pr_info("SrcIPAddr_p             = %p\n", params->SrcIPAddr_p);
	pr_info("DestIPAddr_p            = %p\n", params->DestIPAddr_p);
	pr_info("\n");
}

void print_cmd_desc(struct sam_hw_cmd_desc *cmd_desc)
{
	pr_info("\n");
	pr_info("----------- CDR descriptor ---------\n");
	pr_info("0x%8p + %2d * 4 = 0x%08x\n", cmd_desc, 0, readl_relaxed(&cmd_desc->words[0]));
	pr_info("0x%8p + %2d * 4 = 0x%08x\n", cmd_desc, 1, readl_relaxed(&cmd_desc->words[1]));
	pr_info("0x%8p + %2d * 4 = 0x%08x\n", cmd_desc, 2, readl_relaxed(&cmd_desc->words[2]));
	pr_info("0x%8p + %2d * 4 = 0x%08x\n", cmd_desc, 3, readl_relaxed(&cmd_desc->words[3]));
	pr_info("0x%8p + %2d * 4 = 0x%08x\n", cmd_desc, 4, readl_relaxed(&cmd_desc->words[4]));
	pr_info("0x%8p + %2d * 4 = 0x%08x\n", cmd_desc, 5, readl_relaxed(&cmd_desc->words[5]));
	pr_info("0x%8p + %2d * 4 = 0x%08x\n", cmd_desc, 6, readl_relaxed(&cmd_desc->words[6]));
	pr_info("0x%8p + %2d * 4 = 0x%08x\n", cmd_desc, 7, readl_relaxed(&cmd_desc->words[7]));
	pr_info("0x%8p + %2d * 4 = 0x%08x\n", cmd_desc, 8, readl_relaxed(&cmd_desc->words[8]));
	pr_info("0x%8p + %2d * 4 = 0x%08x\n", cmd_desc, 9, readl_relaxed(&cmd_desc->words[9]));
	pr_info("0x%8p + %2d * 4 = 0x%08x\n", cmd_desc, 10, readl_relaxed(&cmd_desc->words[10]));
	pr_info("0x%8p + %2d * 4 = 0x%08x\n", cmd_desc, 11, readl_relaxed(&cmd_desc->words[11]));
	pr_info("\n");
}

void print_sam_sa_params(struct sam_session_params *sa_params)
{
	pr_info("\n");
	pr_info("----------- struct sam_sa_params *session ---------\n");
	pr_info("dir                  = %d\n", sa_params->dir);
	pr_info("cipher_alg           = %d\n", sa_params->cipher_alg);
	pr_info("cipher_mode          = %d\n", sa_params->cipher_mode);
	/*pr_info("cipher_iv            = %p\n", sa_params->cipher_iv);*/
	pr_info("cipher_key           = %p\n", sa_params->cipher_key);
	pr_info("cipher_key_len       = %d\n", sa_params->cipher_key_len);
	pr_info("auth_alg             = %d\n", sa_params->auth_alg);
	pr_info("auth_key             = %p\n", sa_params->auth_key);
	pr_info("auth_key_len         = %d\n", sa_params->auth_key_len);
	pr_info("protocol             = %d\n", sa_params->proto);
	if (sa_params->proto == SAM_PROTO_NONE) {
		pr_info("basic.auth_icv_len   = %d\n", sa_params->u.basic.auth_icv_len);
		pr_info("basic.auth_aad_len   = %d\n", sa_params->u.basic.auth_aad_len);
	} else if (sa_params->proto == SAM_PROTO_IPSEC) {
		pr_info("ipsec.is_esp         = %d\n", sa_params->u.ipsec.is_esp);
		pr_info("ipsec.is_ip6         = %d\n", sa_params->u.ipsec.is_ip6);
		pr_info("ipsec.is_tunnel      = %d\n", sa_params->u.ipsec.is_tunnel);
		pr_info("ipsec.spi            = 0x%x\n", sa_params->u.ipsec.spi);
		pr_info("ipsec.seq            = 0x%lx\n", sa_params->u.ipsec.seq);
		if (sa_params->u.ipsec.is_tunnel) {
			pr_info("ipsec.sip            = %p\n", sa_params->u.ipsec.tunnel.u.ipv4.sip);
			mv_mem_dump(sa_params->u.ipsec.tunnel.u.ipv4.sip, 4);
			pr_info("ipsec.dip            = %p\n", sa_params->u.ipsec.tunnel.u.ipv4.dip);
			mv_mem_dump(sa_params->u.ipsec.tunnel.u.ipv4.dip, 4);
		}
	}
	pr_info("\n");
}

void print_sam_cio_op_params(struct sam_cio_op_params *request)
{
	pr_info("\n");
	pr_info("----------- struct sam_cio_op_params *request ---------\n");
	pr_info("request->sa                     = %p\n", request->sa);
	pr_info("request->cookie                 = %p\n", request->cookie);
	pr_info("request->num_bufs               = %d\n", request->num_bufs);

	pr_info("request->src[0].vaddr           = %p\n", request->src[0].vaddr);
	pr_info("request->dst[0].vaddr           = %p\n", request->dst[0].vaddr);

	/*pr_info("request->cipher_iv_offset       = %d\n", request->cipher_iv_offset);*/
	pr_info("request->cipher_iv              = %p\n", request->cipher_iv);
	pr_info("request->cipher_offset          = %d\n", request->cipher_offset);
	pr_info("request->cipher_len             = %d\n", request->cipher_len);
	/*pr_info("request->auth_aad_offset        = %d\n", request->auth_aad_offset);*/
	pr_info("request->auth_aad               = %p\n", request->auth_aad);
	pr_info("request->auth_offset            = %d\n", request->auth_offset);
	pr_info("request->auth_len               = %d\n", request->auth_len);
	pr_info("request->auth_icv_offset        = %d\n", request->auth_icv_offset);
	pr_info("\n");
}

void print_sam_cio_ipsec_params(struct sam_cio_ipsec_params *request)
{
	pr_info("\n");
	pr_info("----------- struct sam_cio_ipsec_params *request ---------\n");
	pr_info("request->sa                     = %p\n", request->sa);
	pr_info("request->cookie                 = %p\n", request->cookie);
	pr_info("request->num_bufs               = %d\n", request->num_bufs);

	pr_info("request->src[0].vaddr           = %p\n", request->src[0].vaddr);
	pr_info("request->dst[0].vaddr           = %p\n", request->dst[0].vaddr);
	pr_info("request->l3_offset              = %d\n", request->l3_offset);
	pr_info("request->pkt_size               = %d\n", request->pkt_size);
}

void print_sam_cio_operation_info(struct sam_cio_op *operation)
{
	pr_info("\n");
	pr_info("----------- struct sam_cio_op *operation ---------\n");
	pr_info("operation->cookie               = %p\n", operation->cookie);
	pr_info("operation->token_header_word    = 0x%08x\n", operation->token_header_word);
	pr_info("operation->token_words          = %d\n", operation->token_words);
	pr_info("operation->copy_len             = %d\n", operation->copy_len);
	pr_info("\n");
}

void print_sam_sa(struct sam_sa *session)
{
	struct sam_session_params *params = &session->params;

	print_sam_sa_params(params);

	if (params->cipher_key) {
		printf("\nCipher Key: %d bytes\n", params->cipher_key_len);
		mv_mem_dump(params->cipher_key, params->cipher_key_len);
	}
	if (params->auth_key) {
		printf("\nAuthentication Key: %d bytes\n", params->auth_key_len);
		mv_mem_dump(params->auth_key, params->auth_key_len);

		printf("\nAuthentication Inner: %d bytes\n", (int)sizeof(session->auth_inner));
		mv_mem_dump(session->auth_inner, sizeof(session->auth_inner));

		printf("\nAuthentication Outer: %d bytes\n", (int)sizeof(session->auth_outer));
		mv_mem_dump(session->auth_outer, sizeof(session->auth_outer));
	}
}

void print_sa_builder_params(struct sam_sa *session)
{
	SABuilder_Params_t *params = &session->sa_params;

	pr_info("\n");
	pr_info("----------- SABuilder_Params_t sa_params ---------\n");
	pr_info("CryptoAlgo              = %d\n", params->CryptoAlgo);
	pr_info("AuthAlgo                = %d\n", params->AuthAlgo);
	pr_info("AuthKey1_p              = %p\n", params->AuthKey1_p);
	pr_info("AuthKey2_p              = %p\n", params->AuthKey2_p);
	pr_info("CryptoMode              = %d\n", params->CryptoMode);
	pr_info("CW0                     = 0x%08x\n", params->CW0);
	pr_info("CW1                     = 0x%08x\n", params->CW1);
	pr_info("direction               = %d\n", params->direction);
	pr_info("flags                   = 0x%x\n", params->flags);
	pr_info("IVSrc                   = %d\n", params->IVSrc);
	pr_info("IVWord32Count           = %d\n", params->IVWord32Count);
	pr_info("KeyByteCount            = %d\n", params->KeyByteCount);
	/*pr_info("OffsetARC4State         = %d\n", params->OffsetARC4State);*/
	/*pr_info("OffsetARC4StateRecord   = %d\n", params->OffsetARC4StateRecord);*/
	pr_info("OffsetDigest0           = %d\n", params->OffsetDigest0);
	pr_info("OffsetDigest1           = %d\n", params->OffsetDigest1);
	pr_info("OffsetIV                = %d\n", params->OffsetIV);
	pr_info("OffsetSeqMask           = 0x%x\n", params->OffsetSeqMask);
	pr_info("OffsetSeqNum            = %d\n", params->OffsetSeqNum);
	pr_info("protocol                = %d\n", params->protocol);
	pr_info("SeqMaskWord32Count      = %d\n", params->SeqMaskWord32Count);
	pr_info("SeqNumWord32Count       = %d\n", params->SeqNumWord32Count);
	pr_info("\n");

	if (session->params.proto == SAM_PROTO_NONE)
		print_basic_sa_params(&session->u.basic_params);
	else if (session->params.proto == SAM_PROTO_IPSEC)
		print_ipsec_sa_params(&session->u.ipsec_params);

	printf("\nSA DMA buffer: %d words, physAddr = %p\n",
			session->sa_words, (void *)session->sa_buf.paddr);
	mv_mem_dump_words(session->sa_buf.vaddr, session->sa_words, 0);
}

void print_result_desc(struct sam_hw_res_desc *res_desc, int is_prepared)
{
	pr_info("\n");
	pr_info("----------- %s RDR descriptor ---------\n", is_prepared ? "Prepared" : "Result");
	pr_info("0x%8p + %2d * 4 = 0x%08x\n", res_desc, 0, readl_relaxed(&res_desc->words[0]));
	pr_info("0x%8p + %2d * 4 = 0x%08x\n", res_desc, 1, readl_relaxed(&res_desc->words[1]));
	pr_info("0x%8p + %2d * 4 = 0x%08x\n", res_desc, 2, readl_relaxed(&res_desc->words[2]));
	pr_info("0x%8p + %2d * 4 = 0x%08x\n", res_desc, 3, readl_relaxed(&res_desc->words[3]));

	if (is_prepared)
		return;

	pr_info("0x%8p + %2d * 4 = 0x%08x\n", res_desc, 4, readl_relaxed(&res_desc->words[4]));
	pr_info("0x%8p + %2d * 4 = 0x%08x\n", res_desc, 5, readl_relaxed(&res_desc->words[5]));
	pr_info("0x%8p + %2d * 4 = 0x%08x\n", res_desc, 6, readl_relaxed(&res_desc->words[6]));
	pr_info("0x%8p + %2d * 4 = 0x%08x\n", res_desc, 7, readl_relaxed(&res_desc->words[7]));
	pr_info("0x%8p + %2d * 4 = 0x%08x\n", res_desc, 8, readl_relaxed(&res_desc->words[8]));
	pr_info("0x%8p + %2d * 4 = 0x%08x\n", res_desc, 9, readl_relaxed(&res_desc->words[9]));
	pr_info("0x%8p + %2d * 4 = 0x%08x\n", res_desc, 10, readl_relaxed(&res_desc->words[10]));
	pr_info("0x%8p + %2d * 4 = 0x%08x\n", res_desc, 11, readl_relaxed(&res_desc->words[11]));
	pr_info("\n");

#if 0
	/* Descriptor parsing - TBD */
	/* Word 0 - Control Word */
	control_word = readl_relaxed(&res_desc->words[0]);

	/* Word 2 & 3 - Destination Packet Data Buffer Address */
	(void *)((u64)readl_relaxed(&res_desc->words[2]) |
	       ((u64)readl_relaxed(&res_desc->words[3]) << 32));
#endif
}

void print_token_params(TokenBuilder_Params_t *token)
{
	pr_info("\n");
	pr_info("----------- TokenBuilder_Params_t token ---------\n");
	pr_info("token->PacketFlags              = 0x%x\n", token->PacketFlags);
	pr_info("token->BypassByteCount          = %d\n", token->BypassByteCount);
	pr_info("token->PadByte                  = %d\n", token->PadByte);
	pr_info("token->IV_p                     = %p\n", token->IV_p);
	pr_info("token->AAD_p                    = %p\n", token->AAD_p);
	pr_info("token->AdditionalValue          = %d\n", token->AdditionalValue);
	pr_info("\n");
}
