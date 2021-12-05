/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*
 * SHA1 hash implementation and interface functions
 * Copyright (c) 2003-2005, Jouni Malinen <jkmaline@cc.hut.fi>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Alternatively, this software may be distributed under the terms of BSD
 * license.
 *
 * See README and COPYING for more details.
 */

#ifndef __MV_SHA1_h__
#define __MV_SHA1_h__

#ifdef __cplusplus
extern "C" {
#endif

#define MV_SHA1_DIGEST_SIZE 20

	typedef struct {
		unsigned int state[5];
		unsigned int count[2];
		unsigned char buffer[64];
	} MV_SHA1_CTX;

void mv_sha1_init(MV_SHA1_CTX *context);
void mv_sha1_result_copy(MV_SHA1_CTX *context, uint8_t digest[]);
void mv_sha1_update(MV_SHA1_CTX *context, unsigned char const *buf, unsigned int len);
void mv_sha1_final(unsigned char *digest, MV_SHA1_CTX *context);
void mv_sha1(unsigned char const *buf, unsigned int len, unsigned char *digest);
void mv_sha1_hmac_iv(unsigned char key[], int key_len,
		     unsigned char inner[], unsigned char outer[]);

#ifdef __cplusplus
}
#endif

#endif	/* __MV_SHA1_H__ */
