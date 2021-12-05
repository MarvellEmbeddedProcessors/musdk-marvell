/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*
 * sha2.h
 *
 * Version 1.0.0beta1
 *
 * Written by Aaron D. Gifford <me@aarongifford.com>
 *
 * Copyright 2000 Aaron D. Gifford.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR(S) AND CONTRIBUTOR(S) ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR(S) OR CONTRIBUTOR(S) BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */
#ifndef __MV_SHA2_H__
#define __MV_SHA2_H__

#ifdef __cplusplus
extern "C" {
#endif


/*** SHA-224/256/384/512 Various Length Definitions ***********************/
#define SHA224_BLOCK_LENGTH		64
#define SHA224_DIGEST_LENGTH		28
#define SHA256_BLOCK_LENGTH		64
#define SHA256_DIGEST_LENGTH		32
#define SHA384_BLOCK_LENGTH		128
#define SHA384_DIGEST_LENGTH		48
#define SHA512_BLOCK_LENGTH		128
#define SHA512_DIGEST_LENGTH		64

/*** SHA-256/384/512 Context Structures *******************************/
typedef struct _SHA256_CTX {
	uint32_t	state[8];
	uint64_t	bitcount;
	uint8_t	buffer[SHA256_BLOCK_LENGTH];
} SHA256_CTX;

typedef struct _SHA512_CTX {
	uint64_t	state[8];
	uint64_t	bitcount[2];
	uint8_t	buffer[SHA512_BLOCK_LENGTH];
} SHA512_CTX;

/*** SHA-224/256/384/512 Function Prototypes ******************************/
void mv_sha224_init(SHA256_CTX *ctx);
void mv_sha224(const uint8_t *data, size_t len, uint8_t digest[SHA224_DIGEST_LENGTH]);
void mv_sha224_hmac_iv(uint8_t key[], int key_len, uint8_t inner[], uint8_t outer[]);

void mv_sha256_init(SHA256_CTX *ctx);
void mv_sha256_update(SHA256_CTX *ctx, const uint8_t *input, size_t length);
void mv_sha256_result_copy(SHA256_CTX *ctx, uint8_t digest[]);
void mv_sha256_final(SHA256_CTX *context, uint8_t digest[], int digest_size);
void mv_sha256(const uint8_t *buf, size_t len, uint8_t digest[SHA256_DIGEST_LENGTH]);
void mv_sha256_hmac_iv(uint8_t key[], int key_len, uint8_t inner[], uint8_t outer[]);

void mv_sha384_init(SHA512_CTX *ctx);
void mv_sha384(const uint8_t *buf, size_t len, uint8_t digest[SHA384_DIGEST_LENGTH]);
void mv_sha384_hmac_iv(uint8_t key[], int key_len, uint8_t inner[], uint8_t outer[]);

void mv_sha512_init(SHA512_CTX *ctx);
void mv_sha512_update(SHA512_CTX *ctx, const uint8_t *input, size_t length);
void mv_sha512_final(SHA512_CTX *ctx, uint8_t digest[], int digest_size);
void mv_sha512_result_copy(SHA512_CTX *context, uint8_t digest[]);
void mv_sha512(const uint8_t *data, size_t len, uint8_t digest[SHA512_DIGEST_LENGTH]);
void mv_sha512_hmac_iv(uint8_t key[], int key_len, uint8_t inner[], uint8_t outer[]);

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif /* __MV_SHA2_H__ */

