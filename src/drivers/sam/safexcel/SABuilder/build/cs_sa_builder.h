/* cs_sa_builder.h
 *
 * Product-specific configuration file for the SA Builder.
 */

/*****************************************************************************
* Copyright (c) 2011-2016 INSIDE Secure B.V. All Rights Reserved.
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 2 of the License, or
* any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
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

// Set this if tunnel header fields are to be copied into the transform.
// for extended use case.
//#define SAB_ENABLE_EXTENDED_TUNNEL_HEADER


/* Which protocol families are enabled? */
#define SAB_ENABLE_PROTO_BASIC
#define SAB_ENABLE_PROTO_IPSEC
//#define SAB_ENABLE_PROTO_SSLTLS
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
//#define SAB_ENABLE_DTLS_EXTENDED

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
