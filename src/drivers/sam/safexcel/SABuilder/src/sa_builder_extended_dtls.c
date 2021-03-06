/* sa_builder_extended_dtls.c
 *
 * DTLS specific functions (for initialization of SABuilder_Params_t
 * structures and for building the DTLS specifc part of an SA.) in the
 * Extended use case.
 */

/*****************************************************************************
Copyright (c) 2008-2016 INSIDE Secure B.V.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
  * Neither the name of INSIDE Secure B.V. nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*****************************************************************************/

/*----------------------------------------------------------------------------
 * This module implements (provides) the following interface(s):
 */
#include "c_sa_builder.h"
#ifdef SAB_ENABLE_DTLS_EXTENDED
#include "sa_builder_extended_internal.h"

/*----------------------------------------------------------------------------
 * This module uses (requires) the following interface(s):
 */
#include "basic_defs.h"
#include "log.h"
#include "sa_builder_internal.h" /* SABuilder_SetDTLSParams */
#include "sa_builder_ssltls.h"

/*----------------------------------------------------------------------------
 * Definitions and macros
 */

/*----------------------------------------------------------------------------
 * Local variables
 */


/*----------------------------------------------------------------------------
 * SABuilder_SetExtendedDTLSParams
 *
 * Fill in DTLS-specific extensions into the SA.for Extended.
 *
 * SAParams_p (input)
 *   The SA parameters structure from which the SA is derived.
 * SAState_p (input, output)
 *   Variables containing information about the SA being generated/
 * SABuffer_p (input, output).
 *   The buffer in which the SA is built. If NULL, no SA will be built, but
 *   state variables in SAState_p will still be updated.
 *
 * Return:
 * SAB_STATUS_OK on success
 * SAB_INVALID_PARAMETER when SAParams_p is invalid, or if any of
 *    the buffer arguments  is a null pointer while the corresponding buffer
 *    would be required for the operation.
 * SAB_UNSUPPORTED_FEATURE when SAParams_p describes an operations that
 *    is not supported on the hardware for which this SA builder
 *    is configured.
 */
SABuilder_Status_t
SABuilder_SetExtendedDTLSParams(SABuilder_Params_t *const SAParams_p,
                         SABuilder_State_t * const SAState_p,
                         uint32_t * const SABuffer_p)
{
    SABuilder_Params_SSLTLS_t *SAParamsSSLTLS_p =
        (SABuilder_Params_SSLTLS_t *)(SAParams_p->ProtocolExtension_p);
    uint32_t TokenHeaderWord = SAB_HEADER_DEFAULT;
    SABuilder_ESPProtocol_t ESPProto;
    SABuilder_HeaderProtocol_t HeaderProto;
    uint8_t PadBlockByteCount;
    uint8_t IVByteCount;
    uint8_t ICVByteCount;
    uint8_t SeqOffset;
    uint8_t AntiReplay;
    uint32_t flags = 0;
    uint32_t VerifyInstructionWord, CtxInstructionWord;

    IDENTIFIER_NOT_USED(SAState_p);

    if (SAParamsSSLTLS_p == NULL)
    {
        LOG_CRIT("SABuilder: SSLTLS extension pointer is null\n");
        return SAB_INVALID_PARAMETER;
    }

    if ((SAParamsSSLTLS_p->version != SAB_DTLS_VERSION_1_0 &&
         SAParamsSSLTLS_p->version != SAB_DTLS_VERSION_1_2) ||
        SAParams_p->CryptoAlgo != SAB_CRYPTO_AES)
    {
        LOG_CRIT("SABuilder: SSLTLS record only for look-aside\n");
        // No extended transform record can be created, however it can
        // still be valid for host look-aside.
        return SAB_STATUS_OK;
    }

    if ((SAParamsSSLTLS_p->SSLTLSFlags & SAB_DTLS_NO_ANTI_REPLAY) != 0)
        AntiReplay = 0;
    else
        AntiReplay = 1;

    if (SAParams_p->direction == SAB_DIRECTION_OUTBOUND)
    {
        if (SAParams_p->AuthAlgo == SAB_AUTH_AES_GCM)
        {
            ESPProto = SAB_DTLS_PROTO_OUT_GCM;
            PadBlockByteCount = 0;
            IVByteCount = 8;
        }
        else
        {
            switch (SAParams_p->IVSrc)
            {
            case SAB_IV_SRC_DEFAULT:
            case SAB_IV_SRC_PRNG:
                TokenHeaderWord |=
                    SAB_HEADER_IV_PRNG;
                break;
            case SAB_IV_SRC_SA: /* No action required */
            case SAB_IV_SRC_TOKEN:
                break;
            default:
                LOG_CRIT("SABuilder_BuildSA:"
                         "Unsupported IV source\n");
                return SAB_INVALID_PARAMETER;
            }
            ESPProto = SAB_DTLS_PROTO_OUT_CBC;
            PadBlockByteCount = 16;
            IVByteCount = 16;
        }

        if ((SAParamsSSLTLS_p->SSLTLSFlags & SAB_DTLS_IPV6) !=0)
        {
            if ((SAParamsSSLTLS_p->SSLTLSFlags & SAB_DTLS_CAPWAP) !=0)
            {
                HeaderProto = SAB_HDR_IPV6_OUT_DTLS_CAPWAP;
            }
            else
            {
                HeaderProto = SAB_HDR_IPV6_OUT_DTLS;
            }
        }
        else
        {
            if ((SAParamsSSLTLS_p->SSLTLSFlags & SAB_DTLS_CAPWAP) !=0)
            {
                HeaderProto = SAB_HDR_IPV4_OUT_DTLS_CAPWAP;
            }
            else
            {
                HeaderProto = SAB_HDR_IPV4_OUT_DTLS;
            }
        }
    }
    else
    {
        if (SAParams_p->AuthAlgo == SAB_AUTH_AES_GCM)
        {
            ESPProto = SAB_DTLS_PROTO_IN_GCM;
            PadBlockByteCount = 0;
            IVByteCount = 8;
        }
        else
        {
            ESPProto = SAB_DTLS_PROTO_IN_CBC;
            PadBlockByteCount = 16;
            TokenHeaderWord |= SAB_HEADER_PAD_VERIFY;
            IVByteCount = 16;
        }

        if ((SAParamsSSLTLS_p->SSLTLSFlags & SAB_DTLS_IPV6) !=0)
        {
            if ((SAParamsSSLTLS_p->SSLTLSFlags & SAB_DTLS_CAPWAP) !=0)
            {
                HeaderProto = SAB_HDR_IPV6_IN_DTLS_CAPWAP;
            }
            else
            {
                HeaderProto = SAB_HDR_IPV6_IN_DTLS;
            }
        }
        else
        {
            if ((SAParamsSSLTLS_p->SSLTLSFlags & SAB_DTLS_CAPWAP) !=0)
            {
                HeaderProto = SAB_HDR_IPV4_IN_DTLS_CAPWAP;
            }
            else
            {
                HeaderProto = SAB_HDR_IPV4_IN_DTLS;
            }
        }

        if ((SAParamsSSLTLS_p->SSLTLSFlags & SAB_DTLS_MASK_128) != 0)
            AntiReplay *= 4;
        else if ((SAParamsSSLTLS_p->SSLTLSFlags & SAB_DTLS_MASK_32) == 0)
                    AntiReplay *= 2;
    }
    SeqOffset = SAParams_p->OffsetSeqNum;

    if (SAParams_p->direction == SAB_DIRECTION_OUTBOUND &&
        SAParamsSSLTLS_p->PadAlignment >
        PadBlockByteCount &&
        SAParamsSSLTLS_p->PadAlignment <= 256)
        PadBlockByteCount =
            SAParamsSSLTLS_p->PadAlignment;

    switch(SAParams_p->AuthAlgo)
    {
    case SAB_AUTH_HMAC_MD5:
        ICVByteCount = 16;
        break;
    case SAB_AUTH_HMAC_SHA1:
        ICVByteCount = 20;
        break;
    case SAB_AUTH_HMAC_SHA2_256:
        ICVByteCount = 32;
        break;
    case SAB_AUTH_HMAC_SHA2_384:
        ICVByteCount = 48;
        break;
    case SAB_AUTH_HMAC_SHA2_512:
        ICVByteCount = 64;
        break;
    case SAB_AUTH_AES_GCM:
        ICVByteCount = 16;
        break;
    break;
    default:
        LOG_CRIT("SABuilder_BuildSA: unsupported authentication algorithm\n");
        return SAB_UNSUPPORTED_FEATURE;
    }

    /* Flags variable */
    if ((SAParamsSSLTLS_p->SSLTLSFlags & SAB_DTLS_IPV6) !=0)
        flags |= BIT_8;
    if ((SAParamsSSLTLS_p->SSLTLSFlags & SAB_DTLS_PROCESS_IP_HEADERS) !=0)
        flags |= BIT_19;
    if ((SAParamsSSLTLS_p->SSLTLSFlags & SAB_DTLS_PLAINTEXT_HEADERS) !=0)
        flags |= BIT_29;

    /* Take care of the VERIFY and CTX token instructions */
    if (SAParams_p->direction == SAB_DIRECTION_OUTBOUND)
    {
        VerifyInstructionWord = SAB_VERIFY_NONE;
        CtxInstructionWord = SAB_CTX_OUT_SEQNUM +
            ((unsigned int)(2<<24)) + SeqOffset;
    }
    else
    {
        VerifyInstructionWord = SAB_VERIFY_PAD;
        if (ICVByteCount > 0)
        {
            VerifyInstructionWord += SAB_VERIFY_BIT_H + ICVByteCount;
        }
        if (AntiReplay > 0)
        {
            VerifyInstructionWord += SAB_VERIFY_BIT_SEQ;
        }
        CtxInstructionWord = SAB_CTX_SEQNUM +
            ((unsigned int)(2+AntiReplay)<<24) + SeqOffset;
    }
    /* Write all parameters to their respective offsets */
    if (SABuffer_p != NULL)
    {
#ifdef SAB_ENABLE_TWO_FIXED_RECORD_SIZES
        if (SAParams_p->OffsetSeqNum > SAB_SEQNUM_LO_FIX_OFFSET)
        {
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_FLAGS_WORD_OFFSET_LARGE] = flags;
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_HDRPROC_CTX_WORD_OFFSET_LARGE] =
                SAParamsSSLTLS_p->ContextRef;
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_BYTE_PARAM_WORD_OFFSET_LARGE] =
                SAB_PACKBYTES(IVByteCount,ICVByteCount,HeaderProto,ESPProto);
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_TK_HDR_WORD_OFFSET_LARGE] = TokenHeaderWord;
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_PAD_ALIGN_WORD_OFFSET_LARGE] =
                SAB_PACKBYTES(PadBlockByteCount/2, 0, 0, 0);
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_CCM_SALT_WORD_OFFSET_LARGE] =0;
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_TK_VFY_INST_WORD_OFFSET_LARGE] =
                VerifyInstructionWord;
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_TK_CTX_INST_WORD_OFFSET_LARGE] =
                CtxInstructionWord;
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_TIME_STAMP_LO_WORD_OFFSET_LARGE] = 0;
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_TIME_STAMP_HI_WORD_OFFSET_LARGE] = 0;
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_STAT_OCT_LO_WORD_OFFSET_LARGE] = 0;
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_STAT_OCT_HI_WORD_OFFSET_LARGE] = 0;
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_STAT_PKT_WORD_OFFSET_LARGE] = 0;
        }
        else
#endif /* SAB_ENABLE_TWO_FIXED_RECORD_SIZES */
        {
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_FLAGS_WORD_OFFSET] = flags;
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_HDRPROC_CTX_WORD_OFFSET] =
                SAParamsSSLTLS_p->ContextRef;
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_BYTE_PARAM_WORD_OFFSET] =
                SAB_PACKBYTES(IVByteCount,ICVByteCount,HeaderProto,ESPProto);
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_TK_HDR_WORD_OFFSET] = TokenHeaderWord;
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_PAD_ALIGN_WORD_OFFSET] =
                SAB_PACKBYTES(PadBlockByteCount/2, 0, 0, 0);
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_CCM_SALT_WORD_OFFSET] = 0;
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_TK_VFY_INST_WORD_OFFSET] =
                VerifyInstructionWord;
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_TK_CTX_INST_WORD_OFFSET] =
                CtxInstructionWord;
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_TIME_STAMP_LO_WORD_OFFSET] = 0;
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_TIME_STAMP_HI_WORD_OFFSET] = 0;
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_STAT_OCT_LO_WORD_OFFSET] = 0;
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_STAT_OCT_HI_WORD_OFFSET] = 0;
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_STAT_PKT_WORD_OFFSET] = 0;
        }
    }
    return SAB_STATUS_OK;
}


#endif /* SAB_ENABLE_DTLS_EXTENDED */


/* end of file sa_builder_extended_dtls.c */
