/* sa_builder_params_ipsec.h
 *
 * IPsec specific extension to the SABuilder_Params_t type.
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


#ifndef SA_BUILDER_PARAMS_IPSEC_H_
#define SA_BUILDER_PARAMS_IPSEC_H_


/*----------------------------------------------------------------------------
 * This module uses (requires) the following interface(s):
 */

#include "sa_builder_params.h"

// Driver Framework Basic Definitions API
#include "basic_defs.h"


/*----------------------------------------------------------------------------
 * Definitions and macros
 */

/* Flag bits for the IPsecFlags field. Combine any values using a
   bitwise or.
   Of SAB_IPSEC_ESP and SAB_AH, exactly one must be set.
   Of SAB_IPSEC_TUNNEL and SAB_IPSEC_TRANSPORT, exactly one must be set.
   Of SAB_IPSEC_IPV4 and SAB_IPSEC_IPV6, exactly one must be set.
*/

#define SAB_IPSEC_ESP             BIT_0
#define SAB_IPSEC_AH              BIT_1

#define SAB_IPSEC_TUNNEL          BIT_2
#define SAB_IPSEC_TRANSPORT       BIT_3

#define SAB_IPSEC_IPV4            BIT_4
#define SAB_IPSEC_IPV6            BIT_5

#define SAB_IPSEC_LONG_SEQ        BIT_6 /* Use 64-bit extended seq. number */
#define SAB_IPSEC_NO_ANTI_REPLAY  BIT_7 /* Disable anti-replay protection */
#define SAB_IPSEC_MASK_128        BIT_8 /* Use 128-bit anti-replay mask instead of 64-bit*/
#define SAB_IPSEC_MASK_32         BIT_9 /* Use 32-bit anti-replay mask instead of 64-bit*/

#define SAB_IPSEC_PROCESS_IP_HEADERS BIT_10 /* Perform header processing */
#define SAB_IPSEC_CLEAR_DF        BIT_11 /* Clear DF on outer tunnel header */
#define SAB_IPSEC_SET_DF          BIT_12 /* Set DF on outer tunnel header */
#define SAB_IPSEC_NATT            BIT_13 /* Encapsulate ESP in UDP for nat traversal */
#define SAB_IPSEC_REPLACE_DSCP    BIT_14 /* Copy DSCP from transform record */
#define SAB_IPSEC_MASK_384        BIT_15 /* Use 384-bit anti-replay mask instead of 64-bit */
#define SAB_IPSEC_APPEND_SEQNUM   BIT_16 /* Cause the hardware to append sequence number to output */
#define SAB_IPSEC_CLEAR_ECN       BIT_17 /* Clear ECN bits in tunnel header for compatibility mode */
#define SAB_IPSEC_MASK_256        BIT_18 /* Use 256-bit anti-replay mask instead of 64-bit */

/* Extension record for SAParams_t. Protocol_Extension_p must point
   to this structure when the IPsec protocol is used.

   SABuilder_Iinit_ESP() will fill all fields in this structure  with
   sensible defaults.
 */
typedef struct
{
    uint32_t spi;
    uint32_t IPsecFlags;    /* See SAB_IPSEC_* flag bits above */

    uint32_t SeqNum;       /* Initialize with zero */
    uint32_t SeqNumHi;     /* Only valid if SAB_IPSEC_LONG_SEQ is set */
    uint32_t SeqMask[12];  /* Up to 384-bit mask window
                              Only used with inbound operations.
                              Initialize first word with 1, others with 0. */
    uint32_t PadAlignment; /* Align padding to specified multiple of bytes.
                              This must be a power of two between 4 and 256.
                              If zero, default pad alignment is used.*/
    uint32_t ICVByteCount; /* Length of ICV in bytes. If left zero, a default
                              value is used, compatible with the authentication                               algorithm, */
    uint8_t *SrcIPAddr_p;  /* Source IP address for tunnel header.
                                 4 bytes for IPv4, 16 bytes for IPv6*/
    uint8_t *DestIPAddr_p; /* Destination IP address for tunnel header.
                                 4 bytes for IPv4, 16 bytes for IPv6*/
    uint16_t NATTSrcPort;  /* UDP source port when using NAT-T */
    uint16_t NATTDestPort; /* UDP destination port when using NAT-T */
    uint32_t ContextRef; /* Reference to application context */
    uint8_t  TTL;        /* Time-to-live/Hop Limit in outer header */
    uint8_t  DSCP;       /* DSCP/traffic class field in outer header
                            if copied from SA */
} SABuilder_Params_IPsec_t;


#endif /* SA_BUILDER_PARAMS_IPSEC_H_ */


/* end of file sa_builder_params_ipsec.h */
