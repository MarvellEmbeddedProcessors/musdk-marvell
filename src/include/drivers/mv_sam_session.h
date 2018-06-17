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
	SAM_PROTO_SSLTLS,
	SAM_PROTO_LAST,
};

enum sam_ssltls_version {
	SAM_SSL_VERSION_3_0,
	SAM_TLS_VERSION_1_0,
	SAM_TLS_VERSION_1_1,
	SAM_TLS_VERSION_1_2,
	SAM_DTLS_VERSION_1_0,
	SAM_DTLS_VERSION_1_2,
	SAM_SSLTLS_VERSION_LAST,
};

enum sam_anti_replay_size {
	SAM_ANTI_REPLY_MASK_NONE, /**< 0 - Disable anti-replay protection */
	SAM_ANTI_REPLY_MASK_32B,  /**< 1 - Use 32-bit anti-replay mask */
	SAM_ANTI_REPLY_MASK_64B,  /**< 2 - Use 64-bit anti-replay mask (default) */
	SAM_ANTI_REPLY_MASK_128B, /**< 3 - Use 128-bit anti-replay mask */
	SAM_ANTI_REPLY_MASK_256B, /**< 3 - Use 256-bit anti-replay mask (IPsec only) */
	SAM_ANTI_REPLY_MASK_384B, /**< 3 - Use 384-bit anti-replay mask (IPsec only) */
	SAM_ANTI_REPLY_MASK_LAST
};

struct sam_session_basic_params {
	int auth_then_encrypt;  /**< Operations order:
				 * 0 - encrypt_then_auth and auth_then_decrypt
				 * 1 - auth_then_encrypt and decrypt_then_auth
				 */
	u32 auth_icv_len;       /**< Integrity Check Value (ICV) size (in bytes) */
	u32 auth_aad_len;       /**< Additional Data (AAD) size (in bytes) */
};

struct sam_session_stats {
	u64 sa_add;	/**< Number of added sessions */
	u64 sa_del;	/**< Number of deleted sessions */
	u64 sa_inv;	/**< Number of invalidated sessions */
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
			u8 df;   /** IPv4 Don't Fragment bit */
		} ipv4;
		/** IPv6 header parameters */
		struct {
			u8 *sip;     /** 16 bytes of IPv6 source address (NETWORK ENDIAN) */
			u8 *dip;     /** 16 bytes of IPv6 destination address (NETWORK ENDIAN) */
			u8 dscp;     /** IPv6 Differentiated Services Code Point */
			u8 hlimit;   /** IPv6 hop limit */
			u32 flabel;  /** IPv6 flow label */
		} ipv6;
	} u;
	/** Flags control copy fields between IP inner and outer headers.
	  *   1 - copy field from inner to outer header on outbound and vice versa on inbound.
	  *   0 - use values from ipsec tunnel parameters above.
	  */
	int copy_dscp;   /** 1 - copy DSCP field from IP header, 0 - use "dscp" value above */
	int copy_flabel; /** 1 - copy flabel field from IP header, 0 - use "flabel" value above  */
	int copy_df;     /** 1 - copy df field from IP header, 0 - use "df" value above  */
};

/** NAT-T encapsulation parameters */
struct sam_sa_ipsec_natt {
	u16 udp_sport;
	u16 udp_dport;
};

struct sam_session_ipsec_params {
	int is_esp;				/**< 1 - ESP protocol. Must be 1. AH is not supported */
	int is_ip6;				/**< 1 - IPv6, 0 - IPv4 */
	int is_tunnel;				/**< 1 - tunnel mode, 0 - transport mode */
	int is_esn;				/**< 1 - extended (64 bits) sequence number, 0 - 32 bits */
	enum sam_anti_replay_size seq_mask_size;/**< anti-replay seq mask size */
	struct sam_sa_ipsec_tunnel tunnel;	/**< Parameters for tunnel mode */
	int is_natt;				/**< true - NAT-Traversal is required and "natt" field is valid */
	struct sam_sa_ipsec_natt natt;		/**< NAT-Traversal parameters */
	u64 seq;				/**< Initial sequence number */
	u32 spi;				/**< SPI value */
};

/** SSL/TLS/DTLS session parameters */
struct sam_session_ssltls_params {
	enum sam_ssltls_version version;	/**< SSL, TLS and DTLS versions */
	u16 epoch;				/**< for DTLS only */
	u64 seq;				/**< Initial sequence number */
	int is_ip6;				/**< DTLS transported over: 1 - UDP/IPv6, 0 - UDP/IPv4 */
	int is_udp_lite;			/**< 1 - use UDPLite, 0 - use UDP */
	int is_capwap;				/**< 1 - use CAPWAP/DTLS, 0 - use DTLS */
	enum sam_anti_replay_size seq_mask_size;/**< anti-replay seq mask size */
	u32 seq_mask[4];			/**< up to 128-bit mask window used with inbound DTLS */
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
  *	- "ctr_cio" is a ring handler that (if defined) will be used for session delete
 *	operation. Other ways, session operation ring will be used.
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
	struct sam_cio *ctr_cio;         /**< control plan cio */
	union {
		struct sam_session_basic_params  basic; /**< Parameters for basic crypto */
		struct sam_session_ipsec_params  ipsec; /**< Parameters for IPSec offload */
		struct sam_session_ssltls_params ssltls; /**< Parameters for SSL/TLS offload */
	} u;
};

/**
 * sam_session_get_block_size
 *
 * @param[in]	algo      - cipher algorithm.
 *
 * @retval	0         - unknown cipher algorithm
 * @retval	Positive  - block size for the cipher algorithm in bytes
 */
u32 sam_session_get_block_size(enum sam_cipher_alg algo);

/**
 * Create new crypto session
 *
 * @param[in]	params    - pointer to structure with crypto session parameters.
 * @param[out]	sa        - address of place to save handler of new created crypto session.
 *
 * @retval	0         - success
 * @retval	Negative  - failure
 */
int sam_session_create(struct sam_session_params *params,
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

/**
 * Get statistics collected for sessions.
 *
 * To enable collect statistics capability of the SAM driver,
 *	use "--enable-sam-statistics" flag during ./configure
 *
 * @param[out]    stats    - pointer to copy statistics.
 * @param[in]     reset    - 0    : don't reset statistics after copy.
 *			     other: reset statistics after copy.
 *
 * @retval      0          - statistics are valid
 * @retval	-ENOTSUP   - statistics are not supported
 */
int sam_session_get_stats(struct sam_session_stats *stats, int reset);

/** @} */ /* end of grp_sam_se */

#endif /* __MV_SAM_SESSION_H__ */
