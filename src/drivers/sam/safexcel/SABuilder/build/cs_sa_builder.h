/* cs_sa_builder.h
 *
 * Product-specific configuration file for the SA Builder.
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

/* Specify a specific version of the EIP-96, specify exactly one */
//#define SAB_EIP96_VERSION_I
//#define SAB_EIP96_VERSION_IE
//#define SAB_EIP96_VERSION_IS
#define SAB_EIP96_VERSION_IES
//#define SAB_EIP96_VERSION_IW
//#define SAB_EIP96_VERSION_IEW
//#define SAB_EIP96_VERSION_IESW
//#define SAB_EIP96_VERSION_IEX
//#define SAB_EIP96_VERSION_IESWX
//#define SAB_EIP96_VERSION_IESWXK

/* Enable this if the hardware supports 384-bit sequence number masks. */
#define SAB_ENABLE_384BIT_SEQMASK


// Set this if there are two fixed record sizes:
// SAB_RECORD_WORD_COUNT and SAB_RECORD_WORD_COUNT_LARGE,
// select the large one if the context size exceeds
// SAB_LARGE_RECORD_TRHESHOLD_WORD_COUNT.
//#define SAB_ENABLE_TWO_FIXED_RECORD_SIZES

/* Which protocol families are enabled? */
#define SAB_ENABLE_PROTO_BASIC
#define SAB_ENABLE_PROTO_IPSEC
#define SAB_ENABLE_PROTO_SSLTLS
//#define SAB_ENABLE_PROTO_MACSEC
//#define SAB_ENABLE_PROTO_SRTP

/* Which protocol-specific options are enabled? */
#define SAB_ENABLE_IPSEC_ESP
//#define SAB_ENABLE_IPSEC_AH

/* Enable if the SA Builder must support extended use case for IPsec
   processing */
#define SAB_ENABLE_IPSEC_EXTENDED

// Set this if tunnel header fields are to be copied into the transform.
// for extended use case.
#define SAB_ENABLE_EXTENDED_TUNNEL_HEADER

/* Enable if the SA Builder must support extended use case for DTLS
   processing */
#define SAB_ENABLE_DTLS_EXTENDED

/* Enable if the SA Builder must support an engine with fixed SA records
   (e.g. for a record cache) */
//#define SAB_ENABLE_FIXED_RECORD_SIZE
//#define SAB_ENABLE_TWO_FIXED_RECORD_SIZES

/* Enable this if you desire to include the ARC4 state in the SA
   record. This requires the hardware to be configured for relative
   ARC4 state offsets */
#define SAB_ARC4_STATE_IN_SA

/* When the ARC4 state is included in the SA record, specify the
   alignment requirements for this state.
 */
#define SAB_ARC4_STATE_ALIGN_BYTE_COUNT 8

/* Enable single-pass inbound SSL/TLS, only for HW2.2 */
#define SAB_ENABLE_SINGLE_PASS_SSLTLS

/* Strict checking of function arguments if enabled */
#define SAB_STRICT_ARGS_CHECK

/* log level for the token builder.
   choose from LOG_SEVERITY_INFO, LOG_SEVERITY_WARN, LOG_SEVERITY_CRIT */
#define LOG_SEVERITY_MAX LOG_SEVERITY_WARN


/* end of file cs_sa_builder.h */
