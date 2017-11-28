/* c_token_builder.h
 *
 * Default configuration file for the Token Builder
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
#include "cs_token_builder.h"


/* Uncomment this option if the EIP-96 does not use an input packet pointer.
   This is the case when it is used in a system in which packet data is fetched
   outside the control of the EIP-96. Whether this setting is needed depends
   on the hardware environment in which the EIP-96 is integrated.
 */
//#define TKB_NO_INPUT_PACKET_POINTER

/* Uncomment this option if context reuse must be auto-detected. This
   is only supported in EIP-96 HW2.1 and higher.
 */
//#define TKB_AUTODETECT_CONTEXT_REUSE

/* Define this to a nonzero value if the generated token must contain a header
   that includes the Token Header Word. Any other fields are filled with zero.
   This parameter specifies the header size in words.
 */
#ifndef TKB_TOKEN_HEADER_WORD_COUNT
#define TKB_TOKEN_HEADER_WORD_COUNT 0
#endif


/* Specify a specific version of the EIP-96, specify exactly one */
//#define TKB_EIP96_VERSION_I
//#define TKB_EIP96_VERSION_IE
//#define TKB_EIP96_VERSION_IS
//#define TKB_EIP96_VERSION_IES
//#define TKB_EIP96_VERSION_IW
//#define TKB_EIP96_VERSION_IEW
//#define TKB_EIP96_VERSION_IESW
//#define TKB_EIP96_VERSION_IEX
//#define TKB_EIP96_VERSION_IESWX
//#define TKB_EIP96_VERSION_IESWXK

/* Which protocol families are enabled? */
//#define TKB_ENABLE_PROTO_BASIC
//#define TKB_ENABLE_PROTO_IPSEC
//#define TKB_ENABLE_PROTO_SSLTLS
//#define TKB_ENABLE_PROTO_MACSEC
//#define TKB_ENABLE_PROTO_SRTP

/* Which protocol-specific options are enabled? */
//#define TKB_ENABLE_IPSEC_ESP
//#define TKB_ENABLE_IPSEC_AH

//#define TKB_ENABLE_CRYPTO_WIRELESS
//#define TKB_ENABLE_CRYPTO_XTS


#if defined(TKB_EIP96_VERSION_I) || defined(TKB_EIP96_VERSION_IE) || \
    defined(TKB_EIP96_VERSION_IS) || defined(TKB_EIP96_VERSION_IES) || \
    defined(TKB_EIP96_VERSION_IEX)
#undef TKB_ENABLE_CRYPTO_WIRELESS
#endif


#ifdef TKB_ENABLE_SINGLE_PASS_SSLTLS
#define TKB_SINGLE_PASS_SSLTLS 1
#else
#define TKB_SINGLE_PASS_SSLTLS 0
#endif

#ifdef TKB_ENABLE_PROTO_BASIC
#define TKB_HAVE_PROTO_BASIC 1
#else
#define TKB_HAVE_PROTO_BASIC 0
#endif

#ifdef TKB_ENABLE_PROTO_IPSEC
#define TKB_HAVE_PROTO_IPSEC 1
#else
#define TKB_HAVE_PROTO_IPSEC 0
#endif

#ifdef TKB_ENABLE_PROTO_SSLTLS
#define TKB_HAVE_PROTO_SSLTLS 1
#else
#define TKB_HAVE_PROTO_SSLTLS 0
#endif

#ifdef TKB_ENABLE_PROTO_SRTP
#define TKB_HAVE_PROTO_SRTP 1
#else
#define TKB_HAVE_PROTO_SRTP 0
#endif

#ifdef TKB_ENABLE_CRYPTO_WIRELESS
#define TKB_HAVE_CRYPTO_WIRELESS 1
#else
#define TKB_HAVE_CRYPTO_WIRELESS 0
#endif

#ifdef TKB_ENABLE_CRYPTO_XTS
#define TKB_HAVE_CRYPTO_XTS 1
#else
#define TKB_HAVE_CRYPTO_XTS 0
#endif


/* Strict checking of function arguments if enabled */
//#define TKB_STRICT_ARGS_CHECK

/* log level for the token builder.
   choose from LOG_SEVERITY_INFO, LOG_SEVERITY_WARN, LOG_SEVERITY_CRIT */
#ifndef LOG_SEVERITY_MAX
#define LOG_SEVERITY_MAX LOG_SEVERITY_CRIT
#endif


/* end of file c_sa_builder.h */
