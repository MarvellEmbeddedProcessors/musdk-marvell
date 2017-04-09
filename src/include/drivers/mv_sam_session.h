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

#ifndef __MV_SAM_SESSION_H__
#define __MV_SAM_SESSION_H__

#include "mv_std.h"

/** @addtogroup grp_sam_se Security Acceleration Module: Session
 *
 *  Security Acceleration Module Session API documentation
 *
 *  @{
 */

/** Crypto operation direction */
enum sam_dir {
	SAM_DIR_ENCRYPT = 0, /**< encrypt and/or generate signature */
	SAM_DIR_DECRYPT,     /**< decrypt and/or verify signature */
	SAM_DIR_LAST,
};

/** Cipher algorithm for encryption/decryption */
enum sam_cipher_alg {
	SAM_CIPHER_NONE = 0,
	SAM_CIPHER_DES,
	SAM_CIPHER_3DES,   /* block size = 64 bits */
	SAM_CIPHER_AES,	   /* block size = 128 bits */
	SAM_CIPHER_ALG_LAST,
};

/** Cipher mode for encryption/decryption */
enum sam_cipher_mode {
	SAM_CIPHER_ECB = 0,
	SAM_CIPHER_CBC,
	SAM_CIPHER_OFB,
	SAM_CIPHER_CFB,
	SAM_CIPHER_CFB1,
	SAM_CIPHER_CFB8,
	SAM_CIPHER_CTR,
	SAM_CIPHER_ICM,
	SAM_CIPHER_CCM,    /* Used only with AES. */
	SAM_CIPHER_GCM,	   /* Used only with AES. */
	SAM_CIPHER_GMAC,   /* Used only with AES. */
	SAM_CIPHER_MODE_LAST,
};

/** Authentication algorithm */
enum sam_auth_alg {
	SAM_AUTH_NONE = 0,
	SAM_AUTH_HASH_MD5,
	SAM_AUTH_HASH_SHA1,
	SAM_AUTH_HASH_SHA2_224,
	SAM_AUTH_HASH_SHA2_256,
	SAM_AUTH_HASH_SHA2_384,
	SAM_AUTH_HASH_SHA2_512,
	SAM_AUTH_SSLMAC_MD5,
	SAM_AUTH_SSLMAC_SHA1,
	SAM_AUTH_HMAC_MD5,
	SAM_AUTH_HMAC_SHA1,
	SAM_AUTH_HMAC_SHA2_224,
	SAM_AUTH_HMAC_SHA2_256,
	SAM_AUTH_HMAC_SHA2_384,
	SAM_AUTH_HMAC_SHA2_512,
	SAM_AUTH_AES_XCBC_MAC,
	SAM_AUTH_AES_CMAC_128,
	SAM_AUTH_AES_CMAC_192,
	SAM_AUTH_AES_CMAC_256,
	SAM_AUTH_AES_CCM,
	SAM_AUTH_AES_GCM,
	SAM_AUTH_AES_GMAC,
	SAM_AUTH_ALG_LAST,
};

enum sam_crypto_protocol {
	SAM_PROTO_NONE,
	SAM_PROTO_IPSEC,
	SAM_PROTO_LAST,
};

struct sam_session_basic_params {
	u32 auth_icv_len;                /**< Integrity Check Value (ICV) size (in bytes) */
	u32 auth_aad_len;                /**< Additional Data (AAD) size (in bytes) */
};

/** IPSEC tunnel parameters: supports IPv4 and IPv6 tunnels */
struct sam_sa_ipsec_tunnel {
	union {
		/** IPv4 header parameters */
		struct {
			u8 *sip; /** 4 bytes of IPv4 source address (NETWORK ENDIAN) */
			u8 *dip; /** 4 bytes of IPv4 destination address (NETWORK ENDIAN) */
			u8 dscp; /** IPv4 Differentiated Services Code Point */
			u8 ttl;  /** IPv4 Time To Live */
			u8 flags;
		} ipv4;
		/** IPv6 header parameters */
		struct {
			u8 *sip;
			u8 *dip;
			/* TBD */
		} ipv6;
	} u;
};

/** NAT-T encapsulation parameters */
struct sam_sa_ipsec_natt {
	u16 udp_sport;
	u16 udp_dport;
};

struct sam_session_ipsec_params {
	int is_esp;				/**< true - ESP protocol. must be true. AH is not supported */
	int is_ip6;				/**< true - IPv6, false - IPv4 */
	int is_tunnel;				/**< true - tunnel mode, false - transport mode */
	struct sam_sa_ipsec_tunnel tunnel;	/**< Parameters for tunnel mode */
	int is_natt;				/**< true - NAT-Traversal is required and "natt" field is valid */
	struct sam_sa_ipsec_natt natt;		/**< NAT-Traversal parameters */
	u64 seq;				/**< Initial sequence number */
	u32 spi;				/**< SPI value */
};


/**
 * Crypto session parameters
 *
 * Notes:
 *	- "cipher_iv" is valid only if "crypto_mode" requires IV.
 *	If "cipher_iv" != NULL, IV for all crypto operations for this session will
 *	be derived from the "cipher_iv". "cipher_iv" and "cipher_iv_offset" fields in
 *	"struct sam_cio_op_params" will be ignored.
 *	If "cipher_iv" == NULL, IV value must be valid in "struct sam_cio_op_params".
 *	- Size of "cipher_iv" buffer is derived from cipher algorithm.
 *	- "auth_aad_len" field is valid only when "crypto_mode" is GCM or GMAC.
 *	- "auth_inner" and "auth_outer" are valid only if authentication algorithm
 *	requires key. Size of "auth_inner" and "auth_outer" buffers is derived from
 *	authentication algorithm.
 */
struct sam_session_params {
	enum sam_dir dir;                /**< operation direction: encode/decode */
	enum sam_cipher_alg cipher_alg;  /**< cipher algorithm */
	enum sam_cipher_mode cipher_mode;/**< cipher mode */
	u8  *cipher_iv;                  /**< session cipher IV */
	u8  *cipher_key;                 /**< cipher key */
	u32 cipher_key_len;              /**< cipher key size (in bytes) */
	enum sam_auth_alg auth_alg;      /**< authentication algorithm */
	u8  *auth_key;                   /**< authentication key */
	u32 auth_key_len;                /**< authentication key size (in bytes) */
	enum sam_crypto_protocol proto;  /**< prococol: None/IPSec(ESP)/Others */
	union {
		struct sam_session_basic_params basic; /**< Parameters for basic crypto */
		struct sam_session_ipsec_params ipsec; /**< Parameters for IPSec offload */
	} u;
};

/**
 * Create new crypto session
 *
 * @param[in]	cio       - crypto IO instance handler.
 * @param[in]	params    - pointer to structure with crypto session parameters.
 * @param[out]	sa        - address of place to save handler of new created crypto session.
 *
 * @retval	0         - success
 * @retval	Negative  - failure
 */
int sam_session_create(struct sam_cio *cio, struct sam_session_params *params,
		       struct sam_sa **sa);

/**
 * Delete existing crypto session
 *
 * @param[in]	sa	  - crypto session handler.
 *
 * @retval	0         - success
 * @retval	Negative  - failure
 */
int sam_session_destroy(struct sam_sa *sa);

/** @} */ /* end of grp_sam_se */

#endif /* __MV_SAM_SESSION_H__ */
