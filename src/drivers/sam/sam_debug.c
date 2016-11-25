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
#include "sam.h"

void print_sam_sa_params(struct sam_session_params *sa_params)
{
	pr_info("\n");
	pr_info("----------- struct sam_sa_params *session ---------\n");
	pr_info("sa_params->dir                  = %d\n", sa_params->dir);
	pr_info("sa_params->cipher_alg           = %d\n", sa_params->cipher_alg);
	pr_info("sa_params->cipher_mode          = %d\n", sa_params->cipher_mode);
	/*pr_info("sa_params->cipher_iv            = %p\n", sa_params->cipher_iv);*/
	pr_info("sa_params->cipher_key           = %p\n", sa_params->cipher_key);
	pr_info("sa_params->cipher_key_len       = %d\n", sa_params->cipher_key_len);
	pr_info("sa_params->auth_alg             = %d\n", sa_params->auth_alg);
	pr_info("sa_params->auth_inner           = %p\n", sa_params->auth_inner);
	pr_info("sa_params->auth_outer           = %p\n", sa_params->auth_outer);
	pr_info("sa_params->auth_icv_len         = %d\n", sa_params->auth_icv_len);
	pr_info("sa_params->auth_aad_len         = %d\n", sa_params->auth_aad_len);
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

void print_sa_params(SABuilder_Params_t *params)
{
	pr_info("\n");
	pr_info("----------- SABuilder_Params_t params ---------\n");
	pr_info("params->CryptoAlgo              = %d\n", params->CryptoAlgo);
	pr_info("params->AuthAlgo                = %d\n", params->AuthAlgo);
	pr_info("params->AuthKey1_p              = %p\n", params->AuthKey1_p);
	pr_info("params->AuthKey2_p              = %p\n", params->AuthKey2_p);
	pr_info("params->CryptoMode              = %d\n", params->CryptoMode);
	pr_info("params->CW0                     = 0x%08x\n", params->CW0);
	pr_info("params->CW1                     = 0x%08x\n", params->CW1);
	pr_info("params->direction               = %d\n", params->direction);
	pr_info("params->flags                   = 0x%x\n", params->flags);
	pr_info("params->IVSrc                   = %d\n", params->IVSrc);
	pr_info("params->IVWord32Count           = %d\n", params->IVWord32Count);
	pr_info("params->KeyByteCount            = %d\n", params->KeyByteCount);
	/*pr_info("params->OffsetARC4State         = %d\n", params->OffsetARC4State);*/
	/*pr_info("params->OffsetARC4StateRecord   = %d\n", params->OffsetARC4StateRecord);*/
	pr_info("params->OffsetDigest0           = %d\n", params->OffsetDigest0);
	pr_info("params->OffsetDigest1           = %d\n", params->OffsetDigest1);
	pr_info("params->OffsetIV                = %d\n", params->OffsetIV);
	pr_info("params->OffsetSeqMask           = 0x%x\n", params->OffsetSeqMask);
	pr_info("params->OffsetSeqNum            = %d\n", params->OffsetSeqNum);
	pr_info("params->protocol                = %d\n", params->protocol);
	pr_info("params->SeqMaskWord32Count      = %d\n", params->SeqMaskWord32Count);
	pr_info("params->SeqNumWord32Count       = %d\n", params->SeqNumWord32Count);
	pr_info("\n");
}

void print_basic_sa_params(SABuilder_Params_Basic_t *params)
{
	pr_info("\n");
	pr_info("----------- SABuilder_Params_Basic_t params ---------\n");
	pr_info("params->BasicFlags              = %d\n", params->BasicFlags);
	pr_info("params->DigestBlockCount        = %d\n", params->DigestBlockCount);
	pr_info("params->ICVByteCount            = %d\n", params->ICVByteCount);
	pr_info("params->bearer                  = %d\n", params->bearer);
	pr_info("params->direction               = %d\n", params->direction);
	pr_info("params->fresh                   = %d\n", params->fresh);
	pr_info("\n");
}

void print_cmd_desc(PEC_CommandDescriptor_t *desc)
{
	pr_info("\n");
	pr_info("----------- PEC_CommandDescriptor_t desc ---------\n");
	pr_info("desc->SA_Handle1                = %p\n", desc->SA_Handle1.p);
	pr_info("desc->SA_Handle2                = %p\n", desc->SA_Handle2.p);
	pr_info("desc->SA_WordCount              = %d\n", desc->SA_WordCount);
	pr_info("desc->Token_Handle              = %p\n", desc->Token_Handle.p);
	pr_info("desc->Token_WordCount           = %d\n", desc->Token_WordCount);
	pr_info("desc->Bypass_WordCount          = %d\n", desc->Bypass_WordCount);
	pr_info("desc->DstPkt_Handle             = %p\n", desc->DstPkt_Handle.p);
	pr_info("desc->SrcPkt_Handle             = %p\n", desc->SrcPkt_Handle.p);
	pr_info("desc->SrcPkt_ByteCount          = %d\n", desc->SrcPkt_ByteCount);
	pr_info("desc->User_p                    = %p\n", desc->User_p);
	pr_info("desc->Control1                  = 0x%x\n", desc->Control1);
	pr_info("desc->Control2                  = 0x%x\n", desc->Control2);
	pr_info("desc->Control3                  = 0x%x\n", desc->Control3);
	pr_info("desc->Control4                  = 0x%x\n", desc->Control4);
	pr_info("\n");
}

void print_result_desc(PEC_ResultDescriptor_t *desc)
{
	pr_info("\n");
	pr_info("----------- PEC_ResultDescriptor_t desc ---------\n");
	pr_info("desc->SrcPkt_Handle             = %p\n", desc->SrcPkt_Handle.p);
	pr_info("desc->DstPkt_Handle             = %p\n", desc->DstPkt_Handle.p);
	pr_info("desc->DstPkt_ByteCount          = %d\n", desc->DstPkt_ByteCount);
	pr_info("desc->DstPkt_p                  = %p\n", desc->DstPkt_p);
	pr_info("desc->User_p                    = %p\n", desc->User_p);
	pr_info("desc->Status1                   = 0x%x\n", desc->Status1);
	pr_info("desc->Status2                   = 0x%x\n", desc->Status2);
	pr_info("desc->Status3                   = 0x%x\n", desc->Status3);
	pr_info("desc->Status4                   = 0x%x\n", desc->Status4);
	pr_info("desc->Status5                   = 0x%x\n", desc->Status5);
	pr_info("desc->Status6                   = 0x%x\n", desc->Status6);
	pr_info("desc->Status7                   = 0x%x\n", desc->Status7);
	pr_info("desc->Status8                   = 0x%x\n", desc->Status8);
	pr_info("\n");
}

void print_pkt_params(PEC_PacketParams_t *pkt)
{
	pr_info("\n");
	pr_info("----------- PEC_PacketParams_t pkt ---------\n");
	pr_info("pkt->HW_Services                = %d\n", pkt->HW_Services);
	pr_info("pkt->Offset                     = %d\n", pkt->Offset);
	pr_info("pkt->PadBoundary                = %d\n", pkt->PadBoundary);
	pr_info("pkt->PadByte                    = %d\n", pkt->PadByte);
	pr_info("pkt->TokenHeaderWord            = 0x%x\n", pkt->TokenHeaderWord);
	pr_info("pkt->flags                      = %d\n", pkt->flags);
	pr_info("\n");
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
