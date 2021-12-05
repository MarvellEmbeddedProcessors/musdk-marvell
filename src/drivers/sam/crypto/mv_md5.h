/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*
 * This code implements the MD5 message-digest algorithm.
 * The algorithm is due to Ron Rivest.  This code was
 * written by Colin Plumb in 1993, no copyright is claimed.
 * This code is in the public domain; do with it what you wish.
 *
 * Equivalent code is available from RSA Data Security, Inc.
 * This code has been tested against that, and is equivalent,
 * except that you don't need to include two pages of legalese
 * with every copy.
 *
 * To compute the message digest of a chunk of bytes, declare an
 * MD5Context structure, pass it to MD5Init, call MD5Update as
 * needed on buffers full of bytes, and then call MD5Final, which
 * will fill a supplied 16-byte array with the digest.
 */

#ifndef __MV_MD5_h__
#define __MV_MD5_h__

#ifdef __cplusplus
extern "C" {
#endif


#define MV_MD5_MAC_LEN		(16)	/* bytes */

	typedef struct {
		unsigned int buf[4];
		unsigned int bits[2];
		unsigned char in[64];
	} MV_MD5_CONTEXT;

void mv_md5_init(MV_MD5_CONTEXT *context);
void mv_md5_update(MV_MD5_CONTEXT *context, unsigned char const *buf, unsigned len);
void mv_md5_final(unsigned char digest[16], MV_MD5_CONTEXT *context);
void mv_md5_digest(unsigned char digest[16], MV_MD5_CONTEXT *context);

void mv_md5(unsigned char const *buf, unsigned len, unsigned char *digest);

void mv_hmac_md5(unsigned char const *text, int text_len,
	       unsigned char const *key, int key_len, unsigned char *digest);

void mv_md5_hmac_iv(unsigned char key[], int key_len,
		     unsigned char inner[], unsigned char outer[]);

#ifdef __cplusplus
}
#endif

#endif	/* __MV_MD5_H__ */
