/* sa_builder_extended_ipsec.c
 *
 * IPsec specific functions (for initialization of SABuilder_Params_t
 * structures and for building the IPSec specifc part of an SA.) in the
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

#ifdef SAB_ENABLE_IPSEC_EXTENDED
#include "sa_builder_extended_internal.h"

/*----------------------------------------------------------------------------
 * This module uses (requires) the following interface(s):
 */
#include "basic_defs.h"
#include "log.h"
#include "sa_builder_internal.h" /* SABuilder_SetIpsecParams */
#include "sa_builder_ipsec.h"

/*----------------------------------------------------------------------------
 * Definitions and macros
 */
#define ESP_HDR_LEN 8
#define IPV4_HDR_LEN 20
#define IPV6_HDR_LEN 40

/*----------------------------------------------------------------------------
 * Local variables
 */


#ifdef SAB_ENABLE_EXTENDED_TUNNEL_HEADER
/*----------------------------------------------------------------------------
 * get16
 *
 * Read 16-bit value from byte array not changing the byte order.
 */
static uint16_t
get16no(
        uint8_t *p,
        unsigned int offs)
{
    return (p[offs+1]<<8) | p[offs];
}
#endif

/*----------------------------------------------------------------------------
 * SABuilder_SetExtendedIPsecParams
 *
 * Fill in IPsec-specific extensions into the SA.for Extended.
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
SABuilder_SetExtendedIPsecParams(SABuilder_Params_t *const SAParams_p,
                         SABuilder_State_t * const SAState_p,
                         uint32_t * const SABuffer_p)
{
    SABuilder_Params_IPsec_t *SAParamsIPsec_p =
        (SABuilder_Params_IPsec_t *)(SAParams_p->ProtocolExtension_p);
    uint32_t TokenHeaderWord = SAB_HEADER_DEFAULT;
    SABuilder_ESPProtocol_t ESPProto;
    SABuilder_HeaderProtocol_t HeaderProto;
    uint8_t PadBlockByteCount;
    uint8_t IVByteCount;
    uint8_t ICVByteCount;
    uint8_t SeqOffset;
    uint8_t ExtSeq = 0;
    uint8_t AntiReplay;
    uint32_t CCMSalt = 0;
    uint32_t flags = 0;
    uint32_t VerifyInstructionWord, CtxInstructionWord;
#ifdef SAB_ENABLE_EXTENDED_TUNNEL_HEADER
    uint32_t MTUDiscount = 0;
    uint32_t CheckSum = 0;
#endif
    IDENTIFIER_NOT_USED(SAState_p);

    if (SAParamsIPsec_p == NULL)
    {
        LOG_CRIT("SABuilder: IPsec extension pointer is null\n");
        return SAB_INVALID_PARAMETER;
    }

    if ((SAParamsIPsec_p->IPsecFlags & SAB_IPSEC_ESP) == 0)
    {
        LOG_CRIT("SABuilder: IPsec only supports ESP.\n");
        return SAB_INVALID_PARAMETER;
    }


    if ((SAParamsIPsec_p->IPsecFlags & SAB_IPSEC_NO_ANTI_REPLAY) != 0)
        AntiReplay = 0;
    else
        AntiReplay = 1;

    if (SAParams_p->direction == SAB_DIRECTION_OUTBOUND)
    {
        ESPProto = SAB_ESP_PROTO_OUT_CBC;
        PadBlockByteCount = 4;
        if ((SAParamsIPsec_p->IPsecFlags & SAB_IPSEC_IPV6) !=0)
        {
            if ((SAParamsIPsec_p->IPsecFlags & SAB_IPSEC_PROCESS_IP_HEADERS) !=0)
            {
                if ((SAParamsIPsec_p->IPsecFlags & SAB_IPSEC_TUNNEL) !=0)
                {
                    HeaderProto = SAB_HDR_IPV6_OUT_TUNNEL;
                }
                else
                {
                    HeaderProto = SAB_HDR_IPV6_OUT_TRANSP;
                }
            }
            else
            {
                HeaderProto = SAB_HDR_IPV6_OUT_TRANSP_HDRBYPASS;
            }
        }
        else
        {
            if ((SAParamsIPsec_p->IPsecFlags & SAB_IPSEC_PROCESS_IP_HEADERS) !=0)
            {
                if ((SAParamsIPsec_p->IPsecFlags & SAB_IPSEC_TUNNEL) !=0)
                {
                    HeaderProto = SAB_HDR_IPV4_OUT_TUNNEL;
                }
                else
                {
                    HeaderProto = SAB_HDR_IPV4_OUT_TRANSP;
                }
            }
            else
            {
                HeaderProto = SAB_HDR_IPV4_OUT_TRANSP_HDRBYPASS;
            }
        }

        if ((SAParamsIPsec_p->IPsecFlags & SAB_IPSEC_LONG_SEQ) != 0)
            ExtSeq = 1;
    }
    else
    {
        ESPProto = SAB_ESP_PROTO_IN_CBC;
        PadBlockByteCount = 4;
        TokenHeaderWord |= SAB_HEADER_PAD_VERIFY;

        if ((SAParamsIPsec_p->IPsecFlags & SAB_IPSEC_IPV6) !=0)
        {
            if ((SAParamsIPsec_p->IPsecFlags & SAB_IPSEC_PROCESS_IP_HEADERS) !=0)
            {
                if ((SAParamsIPsec_p->IPsecFlags & SAB_IPSEC_TUNNEL) !=0)
                {
                    HeaderProto = SAB_HDR_IPV6_IN_TUNNEL;
                }
                else
                {
                    HeaderProto = SAB_HDR_IPV6_IN_TRANSP;
                    TokenHeaderWord |= SAB_HEADER_UPD_HDR;
                }
            }
            else
            {
                HeaderProto = SAB_HDR_IPV6_IN_TRANSP_HDRBYPASS;
            }
        }
        else
        {
            if ((SAParamsIPsec_p->IPsecFlags & SAB_IPSEC_PROCESS_IP_HEADERS) !=0)
            {
                if ((SAParamsIPsec_p->IPsecFlags & SAB_IPSEC_TUNNEL) !=0)
                {
                    HeaderProto = SAB_HDR_IPV4_IN_TUNNEL;
                }
                else
                {
                    HeaderProto = SAB_HDR_IPV4_IN_TRANSP;
                    TokenHeaderWord |= SAB_HEADER_UPD_HDR;
                }
            }
            else
            {
                HeaderProto = SAB_HDR_IPV4_IN_TRANSP_HDRBYPASS;
            }
        }

        if ((SAParamsIPsec_p->IPsecFlags & SAB_IPSEC_LONG_SEQ) != 0)
            ExtSeq = 1;
        if ((SAParamsIPsec_p->IPsecFlags & SAB_IPSEC_MASK_384) != 0)
            AntiReplay *= 12;
        else if ((SAParamsIPsec_p->IPsecFlags & SAB_IPSEC_MASK_256) != 0)
            AntiReplay *= 8;
        else if ((SAParamsIPsec_p->IPsecFlags & SAB_IPSEC_MASK_128) != 0)
            AntiReplay *= 4;
        else if ((SAParamsIPsec_p->IPsecFlags & SAB_IPSEC_MASK_32) == 0)
                    AntiReplay *= 2;
    }
    SeqOffset = SAParams_p->OffsetSeqNum;

    switch (SAParams_p->CryptoAlgo)
    {
    case SAB_CRYPTO_NULL:
        IVByteCount = 0;
                break;
    case SAB_CRYPTO_DES:
    case SAB_CRYPTO_3DES:
        IVByteCount = 8;
        PadBlockByteCount = 8;
        break;
    case SAB_CRYPTO_AES:
        if (SAParams_p->CryptoMode == SAB_CRYPTO_MODE_CBC)
        {
            IVByteCount = 16;
            PadBlockByteCount = 16;
        }
        else
        {
            if (SAParams_p->direction == SAB_DIRECTION_OUTBOUND)
                ESPProto = SAB_ESP_PROTO_OUT_CTR;
            else
                ESPProto = SAB_ESP_PROTO_IN_CTR;

            IVByteCount = 8;
        }
        break;
    default:
            LOG_CRIT("SABuilder_BuildSA:"
                     "Unsupported Crypto algorithm\n");
            return SAB_INVALID_PARAMETER;
        ;
    }

    /* For all inbound and CTR mode outbound packets there is
       only one supported way to obtain the IV, which is already
       taken care of. Now handle outbound CBC. */
    if(SAParams_p->CryptoMode == SAB_CRYPTO_MODE_CBC &&
       SAParams_p->direction == SAB_DIRECTION_OUTBOUND &&
       SAParams_p->CryptoAlgo != SAB_CRYPTO_NULL)
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
    }

    if (SAParams_p->direction == SAB_DIRECTION_OUTBOUND &&
        SAParamsIPsec_p->PadAlignment >
        PadBlockByteCount &&
        SAParamsIPsec_p->PadAlignment <= 256)
        PadBlockByteCount =
            SAParamsIPsec_p->PadAlignment;

    switch(SAParams_p->AuthAlgo)
    {
    case SAB_AUTH_NULL:
        ICVByteCount = 0;
        ExtSeq = 0;
        if (SAParams_p->direction == SAB_DIRECTION_OUTBOUND)
            ESPProto = SAB_ESP_PROTO_OUT_NULLAUTH;
        else
            ESPProto = SAB_ESP_PROTO_IN_NULLAUTH;

        break;
            case SAB_AUTH_HMAC_MD5:
    case SAB_AUTH_HMAC_SHA1:
    case SAB_AUTH_AES_XCBC_MAC:
    case SAB_AUTH_AES_CMAC_128:
        ICVByteCount = 12;
        break;
    case SAB_AUTH_HMAC_SHA2_224:
    case SAB_AUTH_HMAC_SHA2_256:
        ICVByteCount = 16;
        break;
    case SAB_AUTH_HMAC_SHA2_384:
        ICVByteCount = 24;
        break;
    case SAB_AUTH_HMAC_SHA2_512:
        ICVByteCount = 32;
        break;
    case SAB_AUTH_AES_CCM:
    case SAB_AUTH_AES_GCM:
    case SAB_AUTH_AES_GMAC:
        // All these protocols have a selectable ICV length.
        if (SAParamsIPsec_p->ICVByteCount == 8 ||
            SAParamsIPsec_p->ICVByteCount == 12 ||
            SAParamsIPsec_p->ICVByteCount == 16)
        {
            ICVByteCount =
                        SAParamsIPsec_p->ICVByteCount;
        }
        else
        {
            ICVByteCount = 16;
        }
        switch (SAParams_p->AuthAlgo)
        {
            /* These protocols need specialized protocol codes
               for the token generator.*/
        case SAB_AUTH_AES_CCM:
            if (SAParams_p->direction == SAB_DIRECTION_OUTBOUND)
                ESPProto = SAB_ESP_PROTO_OUT_CCM;
            else
                ESPProto = SAB_ESP_PROTO_IN_CCM;

            CCMSalt =
                (SAParams_p->Nonce_p[0] << 8) |
                (SAParams_p->Nonce_p[1] << 16) |
                (SAParams_p->Nonce_p[2] << 24) |
                SAB_CCM_FLAG_ADATA_L4 |
                ((ICVByteCount-2)*4);
            break;
        case SAB_AUTH_AES_GCM:
            if (SAParams_p->direction == SAB_DIRECTION_OUTBOUND)
                ESPProto = SAB_ESP_PROTO_OUT_GCM;
            else
                ESPProto = SAB_ESP_PROTO_IN_GCM;
            break;
        case SAB_AUTH_AES_GMAC:
            if (SAParams_p->direction == SAB_DIRECTION_OUTBOUND)
                        ESPProto = SAB_ESP_PROTO_OUT_GMAC;
            else
                ESPProto = SAB_ESP_PROTO_IN_GMAC;
            break;
        default:
            ;
        }
        break;
    default:
        LOG_CRIT("SABuilder_BuildSA: unsupported authentication algorithm\n");
        return SAB_UNSUPPORTED_FEATURE;
    }


    /* Flags variable */
    if ((SAParamsIPsec_p->IPsecFlags & SAB_IPSEC_IPV6) !=0)
        flags |= BIT_8;
    if ((SAParamsIPsec_p->IPsecFlags & SAB_IPSEC_PROCESS_IP_HEADERS) !=0)
        flags |= BIT_19;
    if (ExtSeq !=0)
        flags |= BIT_29;
    if ((SAParamsIPsec_p->IPsecFlags & SAB_IPSEC_CLEAR_DF) != 0)
        flags |= BIT_20;
    if ((SAParamsIPsec_p->IPsecFlags & SAB_IPSEC_SET_DF) != 0)
        flags |= BIT_21;
    if ((SAParamsIPsec_p->IPsecFlags & SAB_IPSEC_REPLACE_DSCP) != 0)
        flags |= BIT_22;
    if ((SAParamsIPsec_p->IPsecFlags & SAB_IPSEC_CLEAR_ECN) != 0)
        flags |= BIT_23;
    if ((SAParamsIPsec_p->IPsecFlags & SAB_IPSEC_APPEND_SEQNUM) != 0)
    {
        if ((SAParamsIPsec_p->IPsecFlags & SAB_IPSEC_LONG_SEQ) != 0)
            flags |= BIT_25;
        else
            flags |= BIT_24;
    }

    /* Take care of the VERIFY and CTX token instructions */
    if (SAParams_p->direction == SAB_DIRECTION_OUTBOUND)
    {
        VerifyInstructionWord = SAB_VERIFY_NONE;
        CtxInstructionWord = SAB_CTX_OUT_SEQNUM +
            ((unsigned int)(ExtSeq+1)<<24) + SeqOffset;
    }
    else
    {
        VerifyInstructionWord = SAB_VERIFY_PADSPI;
        if (ICVByteCount > 0)
        {
            VerifyInstructionWord += SAB_VERIFY_BIT_H + ICVByteCount;
        }
        if (AntiReplay > 0 &&
            (SAParamsIPsec_p->IPsecFlags & SAB_IPSEC_APPEND_SEQNUM) == 0)
        {
            /* Skip verification of sequence number in sequence number append
               mode. */
            VerifyInstructionWord += SAB_VERIFY_BIT_SEQ;
        }
        if (ICVByteCount == 0 || AntiReplay == 0)
        {
#ifdef SAB_ENABLE_TWO_FIXED_RECORD_SIZES
            if (SAParams_p->OffsetSeqNum > SAB_SEQNUM_LO_FIX_OFFSET)
            {
                CtxInstructionWord = SAB_CTX_NONE + sab_record_word_count_large - 1;
            }
            else
#endif
            {
                CtxInstructionWord = SAB_CTX_NONE + sab_record_word_count - 1;
            }
        }
        else if (ExtSeq != 0 ||
                 (AntiReplay != 0 &&
                  SAParams_p->OffsetSeqNum + 2 == SAParams_p->OffsetSeqMask))
        {
            CtxInstructionWord = SAB_CTX_SEQNUM +
                ((unsigned int)(2+AntiReplay)<<24) + SeqOffset;
        }
        else
        {
            CtxInstructionWord = SAB_CTX_INSEQNUM +
                ((unsigned int)(1+AntiReplay)<<24) + SeqOffset;
        }
    }

#ifdef SAB_ENABLE_EXTENDED_TUNNEL_HEADER
    /* Compute the maximum amount by which the packet can be enlarged,
       so discount that from the output MTU to judge whether a packet can
       be processed without fragmentation. */
    if (SAParams_p->direction == SAB_DIRECTION_OUTBOUND)
    {
        MTUDiscount = ESP_HDR_LEN + 1 + PadBlockByteCount +
            IVByteCount + ICVByteCount;

        if ((SAParamsIPsec_p->IPsecFlags & SAB_IPSEC_TUNNEL) !=0)
        {
            if ((SAParamsIPsec_p->IPsecFlags & SAB_IPSEC_IPV4) !=0)
                MTUDiscount += IPV4_HDR_LEN;
            else
                MTUDiscount += IPV6_HDR_LEN;

            // for IPv4 tunnel, pre-calculate checksum on IP addresses and store them in the transform record
            // this checksum does not include the final inversion and is performed on data
            // as they stored in the memory
            if ((SAParamsIPsec_p->IPsecFlags & SAB_IPSEC_IPV4) !=0)
            {
                // protection against NULL pointers
                if ((SAParamsIPsec_p->SrcIPAddr_p != NULL)&&
                    (SAParamsIPsec_p->DestIPAddr_p != NULL))
                {
                    // add the addresses (in order they are stored in the memory)
                    CheckSum += get16no(SAParamsIPsec_p->SrcIPAddr_p, 0);
                    CheckSum += get16no(SAParamsIPsec_p->SrcIPAddr_p, 2);
                    CheckSum += get16no(SAParamsIPsec_p->DestIPAddr_p, 0);
                    CheckSum += get16no(SAParamsIPsec_p->DestIPAddr_p, 2);

                    // process the carries
                    while ((CheckSum>>16) != 0)
                        CheckSum = (CheckSum>>16) + (CheckSum & 0xffff);
                }
            }
        }
    }
#endif

    /* If NAT-T selected, select other header protocol range */
    if ((SAParamsIPsec_p->IPsecFlags & SAB_IPSEC_NATT) != 0)
        HeaderProto += (SAB_HDR_IPV4_OUT_TRANSP_HDRBYPASS_NATT -
                        SAB_HDR_IPV4_OUT_TRANSP_HDRBYPASS);

    /* Write all parameters to their respective offsets */
    if (SABuffer_p != NULL)
    {
#ifdef SAB_ENABLE_TWO_FIXED_RECORD_SIZES
        if (SAParams_p->OffsetSeqNum > SAB_SEQNUM_LO_FIX_OFFSET)
        {
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_FLAGS_WORD_OFFSET_LARGE] = flags;
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_HDRPROC_CTX_WORD_OFFSET_LARGE] =
                SAParamsIPsec_p->ContextRef;
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_BYTE_PARAM_WORD_OFFSET_LARGE] =
                SAB_PACKBYTES(IVByteCount,ICVByteCount,HeaderProto,ESPProto);
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_TK_HDR_WORD_OFFSET_LARGE] = TokenHeaderWord;
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_PAD_ALIGN_WORD_OFFSET_LARGE] =
                SAB_PACKBYTES(PadBlockByteCount/2,
                              0,
                              SAParamsIPsec_p->TTL,
                              SAParamsIPsec_p->DSCP);
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_CCM_SALT_WORD_OFFSET_LARGE] = CCMSalt;
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_TK_VFY_INST_WORD_OFFSET_LARGE] =
                VerifyInstructionWord;
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_TK_CTX_INST_WORD_OFFSET_LARGE] =
                CtxInstructionWord;
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_TIME_STAMP_LO_WORD_OFFSET_LARGE] = 0;
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_TIME_STAMP_HI_WORD_OFFSET_LARGE] = 0;
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_STAT_OCT_LO_WORD_OFFSET_LARGE] = 0;
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_STAT_OCT_HI_WORD_OFFSET_LARGE] = 0;
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_STAT_PKT_WORD_OFFSET_LARGE] = 0;
#ifdef SAB_ENABLE_EXTENDED_TUNNEL_HEADER
            SABuffer_p[FIMRWARE_EIP207_CS_FLOW_TR_NATT_PORTS_WORD_OFFSET_LARGE] =
                SAB_PACKBYTES(SAParamsIPsec_p->NATTSrcPort >> 8,
                              SAParamsIPsec_p->NATTSrcPort & 0xff,
                              SAParamsIPsec_p->NATTDestPort >> 8,
                              SAParamsIPsec_p->NATTDestPort & 0xff);

            if (HeaderProto == SAB_HDR_IPV4_OUT_TUNNEL ||
                HeaderProto == SAB_HDR_IPV6_OUT_TUNNEL ||
                HeaderProto == SAB_HDR_IPV4_OUT_TUNNEL_NATT ||
                HeaderProto == SAB_HDR_IPV6_OUT_TUNNEL_NATT)
            {
#ifdef SAB_STRICT_ARGS_CHECK
                if (SAParamsIPsec_p->SrcIPAddr_p == NULL ||
                    SAParamsIPsec_p->DestIPAddr_p == NULL)
                {
                    LOG_CRIT("SABuilder: NULL pointer tunnel address.\n");
                    return SAB_INVALID_PARAMETER;
                }
#endif
                SABuilderLib_CopyKeyMat(SABuffer_p,
                                        FIRMWARE_EIP207_CS_FLOW_TR_TUNNEL_SRC_WORD_OFFSET_LARGE,
                                        SAParamsIPsec_p->SrcIPAddr_p,
                                        (SAParamsIPsec_p->IPsecFlags&SAB_IPSEC_IPV4)!=0?4:16);
                SABuilderLib_CopyKeyMat(SABuffer_p,
                                        FIRMWARE_EIP207_CS_FLOW_TR_TUNNEL_DST_WORD_OFFSET_LARGE,
                                        SAParamsIPsec_p->DestIPAddr_p,
                                        (SAParamsIPsec_p->IPsecFlags&SAB_IPSEC_IPV4)!=0?4:16);

#ifdef FIRMWARE_EIP207_CS_FLOW_TR_CHECKSUM_WORD_OFFSET
                // checksum (only for IPv4)
                if ((SAParamsIPsec_p->IPsecFlags & SAB_IPSEC_IPV4) !=0)
                    SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_CHECKSUM_WORD_OFFSET_LARGE] = CheckSum;
#endif
            }

            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_PATH_MTU_WORD_OFFSET_LARGE] =  MTUDiscount;
#endif /* SAB_ENABLE_EXTENDED_TUNNEL_HEADER */
        }
        else
#endif /* SAB_ENABLE_TWO_FIXED_RECORD_SIZES */
        {
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_FLAGS_WORD_OFFSET] = flags;
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_HDRPROC_CTX_WORD_OFFSET] =
                SAParamsIPsec_p->ContextRef;
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_BYTE_PARAM_WORD_OFFSET] =
                SAB_PACKBYTES(IVByteCount,ICVByteCount,HeaderProto,ESPProto);
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_TK_HDR_WORD_OFFSET] = TokenHeaderWord;
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_PAD_ALIGN_WORD_OFFSET] =
                SAB_PACKBYTES(PadBlockByteCount/2,
                              0,
                              SAParamsIPsec_p->TTL,
                              SAParamsIPsec_p->DSCP);
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_CCM_SALT_WORD_OFFSET] = CCMSalt;
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_TK_VFY_INST_WORD_OFFSET] =
                VerifyInstructionWord;
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_TK_CTX_INST_WORD_OFFSET] =
                CtxInstructionWord;
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_TIME_STAMP_LO_WORD_OFFSET] = 0;
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_TIME_STAMP_HI_WORD_OFFSET] = 0;
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_STAT_OCT_LO_WORD_OFFSET] = 0;
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_STAT_OCT_HI_WORD_OFFSET] = 0;
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_STAT_PKT_WORD_OFFSET] = 0;
#ifdef SAB_ENABLE_EXTENDED_TUNNEL_HEADER
            SABuffer_p[FIMRWARE_EIP207_CS_FLOW_TR_NATT_PORTS_WORD_OFFSET] =
                SAB_PACKBYTES(SAParamsIPsec_p->NATTSrcPort >> 8,
                              SAParamsIPsec_p->NATTSrcPort & 0xff,
                              SAParamsIPsec_p->NATTDestPort >> 8,
                              SAParamsIPsec_p->NATTDestPort & 0xff);

            if (HeaderProto == SAB_HDR_IPV4_OUT_TUNNEL ||
                HeaderProto == SAB_HDR_IPV6_OUT_TUNNEL ||
                HeaderProto == SAB_HDR_IPV4_OUT_TUNNEL_NATT ||
                HeaderProto == SAB_HDR_IPV6_OUT_TUNNEL_NATT)
            {
#ifdef SAB_STRICT_ARGS_CHECK
                if (SAParamsIPsec_p->SrcIPAddr_p == NULL ||
                    SAParamsIPsec_p->DestIPAddr_p == NULL)
                {
                    LOG_CRIT("SABuilder: NULL pointer tunnel address.\n");
                    return SAB_INVALID_PARAMETER;
                }
#endif
                SABuilderLib_CopyKeyMat(SABuffer_p,
                                        FIRMWARE_EIP207_CS_FLOW_TR_TUNNEL_SRC_WORD_OFFSET,
                                        SAParamsIPsec_p->SrcIPAddr_p,
                                        (SAParamsIPsec_p->IPsecFlags&SAB_IPSEC_IPV4)!=0?4:16);
                SABuilderLib_CopyKeyMat(SABuffer_p,
                                        FIRMWARE_EIP207_CS_FLOW_TR_TUNNEL_DST_WORD_OFFSET,
                                        SAParamsIPsec_p->DestIPAddr_p,
                                        (SAParamsIPsec_p->IPsecFlags&SAB_IPSEC_IPV4)!=0?4:16);

#ifdef FIRMWARE_EIP207_CS_FLOW_TR_CHECKSUM_WORD_OFFSET
                // checksum (only for IPv4)
                if ((SAParamsIPsec_p->IPsecFlags & SAB_IPSEC_IPV4) !=0)
                    SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_CHECKSUM_WORD_OFFSET] = CheckSum;
#endif

            }
            SABuffer_p[FIRMWARE_EIP207_CS_FLOW_TR_PATH_MTU_WORD_OFFSET] =  MTUDiscount;
#endif /* SAB_ENABLE_EXTENDED_TUNNEL_HEADER */
        }
    }
    return SAB_STATUS_OK;
}


#endif /* SAB_ENABLE_IPSEC_EXTENDED */


/* end of file sa_builder_extended_ipsec.c */
