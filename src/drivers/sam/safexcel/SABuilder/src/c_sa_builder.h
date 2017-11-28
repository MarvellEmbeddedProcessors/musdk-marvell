/* c_sa_builder.h
 *
 * Default configuration file for the SA Builder
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
 * Import the product specific configuration.
 */
#include "cs_sa_builder.h"

/* Specify a specific version of the EIP-96, specify exactly one */
//#define SAB_EIP96_VERSION_I
//#define SAB_EIP96_VERSION_IE
//#define SAB_EIP96_VERSION_IS
//#define SAB_EIP96_VERSION_IES
//#define SAB_EIP96_VERSION_IW
//#define SAB_EIP96_VERSION_IEW
//#define SAB_EIP96_VERSION_IESW
//#define SAB_EIP96_VERSION_IEX
//#define SAB_EIP96_VERSION_IESWX
//#define SAB_EIP96_VERSION_IESWXK

/* Which protocol families are enabled? Enable the desired protocols, for the
   I and IE version, the SSLTLS protocol is automatically disabled.*/
//#define SAB_ENABLE_PROTO_BASIC
//#define SAB_ENABLE_PROTO_IPSEC
//#define SAB_ENABLE_PROTO_SSLTLS
//#define SAB_ENABLE_PROTO_MACSEC
//#define SAB_ENABLE_PROTO_SRTP

/* Which protocol-specific options are enabled? */
//#define SAB_ENABLE_IPSEC_ESP
//#define SAB_ENABLE_IPSEC_AH

/* Enable if the SA Builder must support extended use case for IPsec
   processing */
//#define SAB_ENABLE_IPSEC_EXTENDED

/* Enable if the SA Builder must support extended use case for DTLS
   processing */
//#define SAB_ENABLE_DTLS_EXTENDED

/* Enable if the SA Builder must support extended use case for MACsec
   processing */
//#define SAB_ENABLE_MACSEC_EXTENDED

/* Enable if the SA Builder must support extended use case for Basic
   processing */
//#define SAB_ENABLE_BASIC_EXTENDED

/* Enable if the SA Builder must support an engine with fixed SA records
   (e.g. for a record cache) */
//#define SAB_ENABLE_FIXED_RECORD_SIZE

#if defined(SAB_ENABLE_IPSEC_EXTENDED) || defined(SAB_ENABLE_DTLS_EXTENDED) || defined(SAB_ENABLE_MACSEC_EXTENDED) || defined(SAB_ENABLE_BASIC_EXTENDED)

#ifdef SAB_ENABLE_IPSEC_EXTENDED
#if !defined(SAB_ENABLE_PROTO_IPSEC) || !defined(SAB_ENABLE_IPSEC_ESP)
#error "IPsec extended use case requires IPSEC and ESP protocols."
#endif
#endif

#ifdef SAB_ENABLE_DTLS_EXTENDED
#ifndef SAB_ENABLE_PROTO_SSLTLS
#error "DTLS extended use case requires SSL/TLS protocols."
#endif
#endif

#ifdef SAB_ENABLE_MACSEC_EXTENDED
#ifndef SAB_ENABLE_PROTO_MACSEC
#error "MACSEC extended use case requires MACSEC protocols"
#endif
#endif

// In the extended use case, always use a fixed record size, but do
// not provide a value of that size here.
#define SAB_ENABLE_FIXED_RECORD_SIZE

// Set this if tunnel header fields are to be copied into the transform.
// for extended use case.
//#define SAB_ENABLE_EXTENDED_TUNNEL_HEADER

#else
// Look-aside use case.
#ifdef SAB_ENABLE_FIXED_RECORD_SIZE
#ifndef SAB_RECORD_WORD_COUNT
// Specify the number of words of an SA record when the record size is fixed.
#define SAB_RECORD_WORD_COUNT 64
#endif
#endif
#endif

// Set this if there are two fixed record sizes:
// SAB_RECORD_WORD_COUNT and SAB_RECORD_WORD_COUNT_LARGE,
// select the large one if the context size exceeds
// SAB_LARGE_RECORD_TRHESHOLD_WORD_COUNT.
//#define SAB_ENABLE_TWO_FIXED_RECORD_SIZES


/* Enable specific crypto and authentication algorithms. The correct ones
   are already set by the VERSION define.*/
//#define SAB_ENABLE_CRYPTO_AES
//#define SAB_ENABLE_CRYPTO_GCM
//#define SAB_ENABLE_CRYPTO_3DES
//#define SAB_ENABLE_CRYPTO_ARCFOUR
//#define SAB_ENABLE_CRYPTO_XTS
//#define SAB_ENABLE_CRYPTO_KASUMI
//#define SAB_ENABLE_CRYPTO_SNOW
//#define SAB_ENABLE_CRYPTO_ZUC
//#define SAB_ENABLE_AUTH_MD5
//#define SAB_ENABLE_AUTH_SHA1
//#define SAB_ENABLE_AUTH_SHA2_256
//#define SAB_ENABLE_AUTH_SHA2_512
//#define SAB_ENABLE_AUTH_SHA3

#ifdef SAB_EIP96_VERSION_I
#define SAB_ENABLE_CRYPTO_AES
#define SAB_ENABLE_CRYPTO_GCM
#define SAB_ENABLE_CRYPTO_3DES
#undef SAB_ENABLE_CRYPTO_ARCFOUR
#define SAB_ENABLE_AUTH_MD5
#define SAB_ENABLE_AUTH_SHA1
#define SAB_ENABLE_AUTH_SHA2_256
#undef SAB_ENABLE_AUTH_SHA2_512
#undef SAB_ENABLE_AUTH_SHA3
#undef SAB_ENABLE_CRYPTO_XTS
#undef SAB_ENABLE_CRYPTO_KASUMI
#undef SAB_ENABLE_CRYPTO_SNOW
#undef SAB_ENABLE_CRYPTO_ZUC
#endif

#ifdef SAB_EIP96_VERSION_IE
#define SAB_ENABLE_CRYPTO_AES
#define SAB_ENABLE_CRYPTO_GCM
#define SAB_ENABLE_CRYPTO_3DES
#undef SAB_ENABLE_CRYPTO_ARCFOUR
#define SAB_ENABLE_AUTH_MD5
#define SAB_ENABLE_AUTH_SHA1
#define SAB_ENABLE_AUTH_SHA2_256
#define SAB_ENABLE_AUTH_SHA2_512
#undef SAB_ENABLE_AUTH_SHA3
#undef SAB_ENABLE_CRYPTO_XTS
#undef SAB_ENABLE_CRYPTO_KASUMI
#undef SAB_ENABLE_CRYPTO_SNOW
#undef SAB_ENABLE_CRYPTO_ZUC
#endif

#ifdef SAB_EIP96_VERSION_IS
#define SAB_ENABLE_CRYPTO_AES
#define SAB_ENABLE_CRYPTO_GCM
#define SAB_ENABLE_CRYPTO_3DES
#define SAB_ENABLE_CRYPTO_ARCFOUR
#define SAB_ENABLE_AUTH_MD5
#define SAB_ENABLE_AUTH_SHA1
#define SAB_ENABLE_AUTH_SHA2_256
#undef SAB_ENABLE_AUTH_SHA2_512
#undef SAB_ENABLE_AUTH_SHA3
#undef SAB_ENABLE_CRYPTO_XTS
#undef SAB_ENABLE_CRYPTO_KASUMI
#undef SAB_ENABLE_CRYPTO_SNOW
#undef SAB_ENABLE_CRYPTO_ZUC
#endif

#ifdef SAB_EIP96_VERSION_IES
#define SAB_ENABLE_CRYPTO_AES
#define SAB_ENABLE_CRYPTO_GCM
#define SAB_ENABLE_CRYPTO_3DES
#define SAB_ENABLE_CRYPTO_ARCFOUR
#define SAB_ENABLE_AUTH_MD5
#define SAB_ENABLE_AUTH_SHA1
#define SAB_ENABLE_AUTH_SHA2_256
#define SAB_ENABLE_AUTH_SHA2_512
#undef SAB_ENABLE_AUTH_SHA3
#undef SAB_ENABLE_CRYPTO_XTS
#undef SAB_ENABLE_CRYPTO_KASUMI
#undef SAB_ENABLE_CRYPTO_SNOW
#undef SAB_ENABLE_CRYPTO_ZUC
#endif


#ifdef SAB_EIP96_VERSION_IW
#define SAB_ENABLE_CRYPTO_AES
#define SAB_ENABLE_CRYPTO_GCM
#define SAB_ENABLE_CRYPTO_3DES
#undef SAB_ENABLE_CRYPTO_ARCFOUR
#define SAB_ENABLE_AUTH_MD5
#define SAB_ENABLE_AUTH_SHA1
#define SAB_ENABLE_AUTH_SHA2_256
#undef SAB_ENABLE_AUTH_SHA2_512
#undef SAB_ENABLE_AUTH_SHA3
#undef SAB_ENABLE_CRYPTO_XTS
#define SAB_ENABLE_CRYPTO_KASUMI
#define SAB_ENABLE_CRYPTO_SNOW
#define SAB_ENABLE_CRYPTO_ZUC
#endif

#ifdef SAB_EIP96_VERSION_IEW
#define SAB_ENABLE_CRYPTO_AES
#define SAB_ENABLE_CRYPTO_GCM
#define SAB_ENABLE_CRYPTO_3DES
#undef SAB_ENABLE_CRYPTO_ARCFOUR
#define SAB_ENABLE_AUTH_MD5
#define SAB_ENABLE_AUTH_SHA1
#define SAB_ENABLE_AUTH_SHA2_256
#define SAB_ENABLE_AUTH_SHA2_512
#undef SAB_ENABLE_AUTH_SHA3
#undef SAB_ENABLE_CRYPTO_XTS
#define SAB_ENABLE_CRYPTO_KASUMI
#define SAB_ENABLE_CRYPTO_SNOW
#define SAB_ENABLE_CRYPTO_ZUC
#endif

#ifdef SAB_EIP96_VERSION_IESW
#define SAB_ENABLE_CRYPTO_AES
#define SAB_ENABLE_CRYPTO_GCM
#define SAB_ENABLE_CRYPTO_3DES
#define SAB_ENABLE_CRYPTO_ARCFOUR
#define SAB_ENABLE_AUTH_MD5
#define SAB_ENABLE_AUTH_SHA1
#define SAB_ENABLE_AUTH_SHA2_256
#define SAB_ENABLE_AUTH_SHA2_512
#undef SAB_ENABLE_AUTH_SHA3
#undef SAB_ENABLE_CRYPTO_XTS
#define SAB_ENABLE_CRYPTO_KASUMI
#define SAB_ENABLE_CRYPTO_SNOW
#define SAB_ENABLE_CRYPTO_ZUC
#endif

#ifdef SAB_EIP96_VERSION_IEX
#define SAB_ENABLE_CRYPTO_AES
#define SAB_ENABLE_CRYPTO_GCM
#define SAB_ENABLE_CRYPTO_3DES
#undef SAB_ENABLE_CRYPTO_ARCFOUR
#define SAB_ENABLE_AUTH_MD5
#define SAB_ENABLE_AUTH_SHA1
#define SAB_ENABLE_AUTH_SHA2_256
#define SAB_ENABLE_AUTH_SHA2_512
#undef SAB_ENABLE_AUTH_SHA3
#define SAB_ENABLE_CRYPTO_XTS
#undef SAB_ENABLE_CRYPTO_KASUMI
#undef SAB_ENABLE_CRYPTO_SNOW
#undef SAB_ENABLE_CRYPTO_ZUC
#endif

#ifdef SAB_EIP96_VERSION_IESWX
#define SAB_ENABLE_CRYPTO_AES
#define SAB_ENABLE_CRYPTO_GCM
#define SAB_ENABLE_CRYPTO_3DES
#define SAB_ENABLE_CRYPTO_ARCFOUR
#define SAB_ENABLE_AUTH_MD5
#define SAB_ENABLE_AUTH_SHA1
#define SAB_ENABLE_AUTH_SHA2_256
#define SAB_ENABLE_AUTH_SHA2_512
#undef SAB_ENABLE_AUTH_SHA3
#define SAB_ENABLE_CRYPTO_XTS
#define SAB_ENABLE_CRYPTO_KASUMI
#define SAB_ENABLE_CRYPTO_SNOW
#define SAB_ENABLE_CRYPTO_ZUC
#endif

#ifdef SAB_EIP96_VERSION_IESWXK
#define SAB_ENABLE_CRYPTO_AES
#define SAB_ENABLE_CRYPTO_GCM
#define SAB_ENABLE_CRYPTO_3DES
#define SAB_ENABLE_CRYPTO_ARCFOUR
#define SAB_ENABLE_AUTH_MD5
#define SAB_ENABLE_AUTH_SHA1
#define SAB_ENABLE_AUTH_SHA2_256
#define SAB_ENABLE_AUTH_SHA2_512
#define SAB_ENABLE_AUTH_SHA3
#define SAB_ENABLE_CRYPTO_XTS
#define SAB_ENABLE_CRYPTO_KASUMI
#define SAB_ENABLE_CRYPTO_SNOW
#define SAB_ENABLE_CRYPTO_ZUC
#endif


/* Enable this if the hardware supports 384-bit sequence number masks. */
//#define SAB_ENABLE_384BIT_SEQMASK

/* Enable this to support building an SA for LTE firmware*/
//#define SAB_ENABLE_LTE_FIRMWARE

/* Enable this if you desire to include the ARC4 state in the SA
   record. This requires the hardware to be configured for relative
   ARC4 state offsets */
//#define SAB_ARC4_STATE_IN_SA


/* When the ARC4 state is included in the SA record, specify the
   alignment requirements for this state.
 */
#ifndef SAB_ARC4_STATE_ALIGN_BYTE_COUNT
#define SAB_ARC4_STATE_ALIGN_BYTE_COUNT 4
#endif


/* Enable single-pass inbound SSL/TLS, only for HW2.2 and higher */
//#define SAB_ENABLE_SINGLE_PASS_SSLTLS

/* Strict checking of function arguments if enabled */
//#define SAB_STRICT_ARGS_CHECK


/* log level for the SA builder.
   choose from LOG_SEVERITY_INFO, LOG_SEVERITY_WARN, LOG_SEVERITY_CRIT */
#ifndef LOG_SEVERITY_MAX
#define LOG_SEVERITY_MAX LOG_SEVERITY_CRIT
#endif


/* end of file c_sa_builder.h */
