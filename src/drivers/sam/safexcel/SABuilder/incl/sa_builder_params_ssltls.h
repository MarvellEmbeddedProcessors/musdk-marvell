/* sa_builder_params_ssltls.h
 *
 * SSL/TLS/DTLS specific extension to the SABuilder_Params_t type.
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


#ifndef SA_BUILDER_PARAMS_SSLTLS_H_
#define SA_BUILDER_PARAMS_SSLTLS_H_


/*----------------------------------------------------------------------------
 * This module uses (requires) the following interface(s):
 */

#include "sa_builder_params.h"

// Driver Framework Basic Definitions API
#include "basic_defs.h"


/*----------------------------------------------------------------------------
 * Definitions and macros
 */

/* Flag bits for the SSLTLSFlags field. Combine any values using a
   bitwise or.
 */
#define SAB_DTLS_NO_ANTI_REPLAY  BIT_0 /* Disable anti-replay protection */
#define SAB_DTLS_MASK_128        BIT_1 /* Use 128-bit anti-replay mask instead of 64-bit*/
#define SAB_DTLS_MASK_32         BIT_2 /* Use 32-bit anti-replay mask instead of 64-bit*/
#define SAB_DTLS_IPV4            BIT_3 /* DLTS transported over UDP IPv4 */
#define SAB_DTLS_IPV6            BIT_4 /* DTLS transported over UDP IPv6 */
#define SAB_DTLS_UDPLITE         BIT_5 /* Use UDPLite instead of UDP */
#define SAB_DTLS_CAPWAP          BIT_6 /* Use CAPWAP/DTLS */
#define SAB_DTLS_PROCESS_IP_HEADERS BIT_7 /* Perform header processing */
#define SAB_DTLS_PLAINTEXT_HEADERS BIT_8 /* Expect DTLS headers in plaintext packets */

/* SSL, TLS and DTLS versions */
#define SAB_SSL_VERSION_3_0   0x0300
#define SAB_TLS_VERSION_1_0   0x0301
#define SAB_TLS_VERSION_1_1   0x0302
#define SAB_TLS_VERSION_1_2   0x0303
#define SAB_DTLS_VERSION_1_0  0xFEFF
#define SAB_DTLS_VERSION_1_2  0xFEFD

/* Extension record for SAParams_t. Protocol_Extension_p must point
   to this structure when the SSL, TLS or DTLS  protocol is used.

   SABuilder_Iinit_SSLTLS() will fill all fields in this structure  with
   sensible defaults.
 */
typedef struct
{
    uint32_t SSLTLSFlags;  /* See SAB_{SSL,TLS,DTLS}_* flag bits above*/
    uint16_t version;
    uint16_t epoch;        /* for DTLS only */

    uint32_t SeqNum;       /* Least significant part of sequence number.*/
    uint32_t SeqNumHi;     /* Most significant part of sequence number. */

    uint32_t SeqMask[4];   /* Up to 128-bit mask window
                              Only used with inbound DTLS.
                              Initialize first word with 1, others with 0. */

    uint32_t PadAlignment; /* Align padding to specified multiple of bytes.
                              This must be a power of two between 4 and 256.
                              If zero, default pad alignment is used.*/
    uint32_t ContextRef; /* Reference to application context */
} SABuilder_Params_SSLTLS_t;


#endif /* SA_BUILDER_PARAMS_TLS_H_ */


/* end of file sa_builder_params_ssltls.h */
