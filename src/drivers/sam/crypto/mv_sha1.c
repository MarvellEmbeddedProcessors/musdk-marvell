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

#include <std_internal.h>
#include "mv_sha1.h"

#define SHA1HANDSOFF

typedef union {
	unsigned char c[64];
	unsigned int l[16];

} CHAR64LONG16;

static void mv_sha1_transform(unsigned int state[5], const unsigned char *buffer);

#define rol(value, bits) (((value) << (bits)) | ((value) >> (32 - (bits))))

#if __BYTE_ORDER == __LITTLE_ENDIAN
#define blk0(i) (block->l[i] = (rol(block->l[i], 24) & 0xFF00FF00) | \
		(rol(block->l[i], 8) & 0x00FF00FF))
#else
#define blk0(i) (block->l[i])
#endif /* __BYTE_ORDER == __LITTLE_ENDIAN */

#define blk(i) (block->l[i & 15] = rol(block->l[(i + 13) & 15] ^ \
		block->l[(i + 8) & 15] ^ block->l[(i + 2) & 15] ^ block->l[i & 15], 1))

/* (R0+R1), R2, R3, R4 are the different operations used in SHA1 */
#define R0(v, w, x, y, z, i) \
do { \
	z += ((w & (x ^ y)) ^ y) + blk0(i) + 0x5A827999 + rol(v, 5); \
	w = rol(w, 30); \
} while (0)

#define R1(v, w, x, y, z, i) \
do { \
	z += ((w & (x ^ y)) ^ y) + blk(i) + 0x5A827999 + rol(v, 5); \
	w = rol(w, 30); \
} while (0)

#define R2(v, w, x, y, z, i) \
do { \
	z += (w ^ x ^ y) + blk(i) + 0x6ED9EBA1 + rol(v, 5); \
	w = rol(w, 30); \
} while (0)

#define R3(v, w, x, y, z, i) \
do { \
	z += (((w | x) & y) | (w & x)) + blk(i) + 0x8F1BBCDC + rol(v, 5); \
	w = rol(w, 30); \
} while (0)

#define R4(v, w, x, y, z, i) \
do { \
	z += (w ^ x ^ y) + blk(i) + 0xCA62C1D6 + rol(v, 5); \
	w = rol(w, 30); \
} while (0)

/* Hash a single 512-bit block. This is the core of the algorithm. */
static void mv_sha1_transform(unsigned int state[5], const unsigned char *buffer)
{
	unsigned int a, b, c, d, e;
	CHAR64LONG16 *block;

#ifdef SHA1HANDSOFF
	static unsigned int workspace[16];

	block = (CHAR64LONG16 *) workspace;
	memcpy(block, buffer, 64);
#else
	block = (CHAR64LONG16 *) buffer;
#endif
	/* Copy context->state[] to working vars */
	a = state[0];
	b = state[1];
	c = state[2];
	d = state[3];
	e = state[4];
	/* 4 rounds of 20 operations each. Loop unrolled. */
	R0(a, b, c, d, e, 0);
	R0(e, a, b, c, d, 1);
	R0(d, e, a, b, c, 2);
	R0(c, d, e, a, b, 3);
	R0(b, c, d, e, a, 4);
	R0(a, b, c, d, e, 5);
	R0(e, a, b, c, d, 6);
	R0(d, e, a, b, c, 7);
	R0(c, d, e, a, b, 8);
	R0(b, c, d, e, a, 9);
	R0(a, b, c, d, e, 10);
	R0(e, a, b, c, d, 11);
	R0(d, e, a, b, c, 12);
	R0(c, d, e, a, b, 13);
	R0(b, c, d, e, a, 14);
	R0(a, b, c, d, e, 15);
	R1(e, a, b, c, d, 16);
	R1(d, e, a, b, c, 17);
	R1(c, d, e, a, b, 18);
	R1(b, c, d, e, a, 19);
	R2(a, b, c, d, e, 20);
	R2(e, a, b, c, d, 21);
	R2(d, e, a, b, c, 22);
	R2(c, d, e, a, b, 23);
	R2(b, c, d, e, a, 24);
	R2(a, b, c, d, e, 25);
	R2(e, a, b, c, d, 26);
	R2(d, e, a, b, c, 27);
	R2(c, d, e, a, b, 28);
	R2(b, c, d, e, a, 29);
	R2(a, b, c, d, e, 30);
	R2(e, a, b, c, d, 31);
	R2(d, e, a, b, c, 32);
	R2(c, d, e, a, b, 33);
	R2(b, c, d, e, a, 34);
	R2(a, b, c, d, e, 35);
	R2(e, a, b, c, d, 36);
	R2(d, e, a, b, c, 37);
	R2(c, d, e, a, b, 38);
	R2(b, c, d, e, a, 39);
	R3(a, b, c, d, e, 40);
	R3(e, a, b, c, d, 41);
	R3(d, e, a, b, c, 42);
	R3(c, d, e, a, b, 43);
	R3(b, c, d, e, a, 44);
	R3(a, b, c, d, e, 45);
	R3(e, a, b, c, d, 46);
	R3(d, e, a, b, c, 47);
	R3(c, d, e, a, b, 48);
	R3(b, c, d, e, a, 49);
	R3(a, b, c, d, e, 50);
	R3(e, a, b, c, d, 51);
	R3(d, e, a, b, c, 52);
	R3(c, d, e, a, b, 53);
	R3(b, c, d, e, a, 54);
	R3(a, b, c, d, e, 55);
	R3(e, a, b, c, d, 56);
	R3(d, e, a, b, c, 57);
	R3(c, d, e, a, b, 58);
	R3(b, c, d, e, a, 59);
	R4(a, b, c, d, e, 60);
	R4(e, a, b, c, d, 61);
	R4(d, e, a, b, c, 62);
	R4(c, d, e, a, b, 63);
	R4(b, c, d, e, a, 64);
	R4(a, b, c, d, e, 65);
	R4(e, a, b, c, d, 66);
	R4(d, e, a, b, c, 67);
	R4(c, d, e, a, b, 68);
	R4(b, c, d, e, a, 69);
	R4(a, b, c, d, e, 70);
	R4(e, a, b, c, d, 71);
	R4(d, e, a, b, c, 72);
	R4(c, d, e, a, b, 73);
	R4(b, c, d, e, a, 74);
	R4(a, b, c, d, e, 75);
	R4(e, a, b, c, d, 76);
	R4(d, e, a, b, c, 77);
	R4(c, d, e, a, b, 78);
	R4(b, c, d, e, a, 79);
	/* Add the working vars back into context.state[] */
	state[0] += a;
	state[1] += b;
	state[2] += c;
	state[3] += d;
	state[4] += e;
	/* Wipe variables */
	a = b = c = d = e = 0;
}

void mv_sha1_result_copy(MV_SHA1_CTX *context, uint8_t digest[])
{
	int i;

	for (i = 0; i < MV_SHA1_DIGEST_SIZE; i++) {
		digest[i] = (unsigned char)
		    ((context->state[i >> 2] >> ((3 - (i & 3)) * 8)) & 255);
	}
}

void mv_sha1_init(MV_SHA1_CTX *context)
{
	/* SHA1 initialization constants */
	context->state[0] = 0x67452301;
	context->state[1] = 0xEFCDAB89;
	context->state[2] = 0x98BADCFE;
	context->state[3] = 0x10325476;
	context->state[4] = 0xC3D2E1F0;
	context->count[0] = context->count[1] = 0;
}

/* Run your data through this. */
void mv_sha1_update(MV_SHA1_CTX *context, unsigned char const *data, unsigned int len)
{
	unsigned int i, j;

	j = (context->count[0] >> 3) & 63;
	context->count[0] += len << 3;
	if (context->count[0] < (len << 3))
		context->count[1]++;
	context->count[1] += (len >> 29);
	if ((j + len) > 63) {
		memcpy(&context->buffer[j], data, (i = 64 - j));
		mv_sha1_transform(context->state, context->buffer);
		for (; i + 63 < len; i += 64)
			mv_sha1_transform(context->state, &data[i]);
		j = 0;
	} else {
		i = 0;
	}
	memcpy(&context->buffer[j], &data[i], len - i);
}

void mv_sha1_final(unsigned char *digest, MV_SHA1_CTX *context)
{
	unsigned int i;
	unsigned char finalcount[8];

	for (i = 0; i < 8; i++)
		finalcount[i] = (unsigned char)((context->count[(i >= 4 ? 0 : 1)] >> ((3 - (i & 3)) * 8)) & 255);
	/* Endian independent */

	mv_sha1_update(context, (const unsigned char *)"\200", 1);
	while ((context->count[0] & 504) != 448)
		mv_sha1_update(context, (const unsigned char *)"\0", 1);

	mv_sha1_update(context, finalcount, 8);	/* Should cause a mv_sha1_Transform() */

	mv_sha1_result_copy(context, digest);

	/* Wipe variables */
	i = 0;
	memset(context->buffer, 0, 64);
	memset(context->state, 0, 20);
	memset(context->count, 0, 8);
	memset(finalcount, 0, 8);

#ifdef SHA1HANDSOFF		/* make SHA1Transform overwrite it's own static vars */
	mv_sha1_transform(context->state, context->buffer);
#endif
}

void mv_sha1(unsigned char const *buf, unsigned int len, unsigned char *digest)
{
	MV_SHA1_CTX ctx;

	mv_sha1_init(&ctx);
	mv_sha1_update(&ctx, buf, len);
	mv_sha1_final(digest, &ctx);
}

void mv_sha1_hmac_iv(unsigned char key[], int key_len,
		     unsigned char inner[], unsigned char outer[])
{
	unsigned char   in[64];
	unsigned char   out[64];
	unsigned char   key_buf[64];
	int             i, max_key_len;
	MV_SHA1_CTX	ctx;

	max_key_len = 64;

	if (key_len > max_key_len) {
		/* Hash Key first */
		memset(key_buf, 0, sizeof(key_buf));
		mv_sha1(key, key_len, key_buf);
		key = key_buf;
		key_len = max_key_len;
	}
	for (i = 0; i < key_len; i++) {
		in[i] = 0x36 ^ key[i];
		out[i] = 0x5c ^ key[i];
	}
	for (i = key_len; i < max_key_len; i++) {
		in[i] = 0x36;
		out[i] = 0x5c;
	}

	memset(&ctx, 0, sizeof(ctx));
	mv_sha1_init(&ctx);
	mv_sha1_update(&ctx, in, max_key_len);
	mv_sha1_result_copy(&ctx, inner);

	memset(&ctx, 0, sizeof(ctx));
	mv_sha1_init(&ctx);
	mv_sha1_update(&ctx, out, max_key_len);
	mv_sha1_result_copy(&ctx, outer);
}
