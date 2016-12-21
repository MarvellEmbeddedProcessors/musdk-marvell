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

#include <stdlib.h>
#include <string.h>

#include "mv_sam.h"

static struct sam_cio *cio_hndl;
static struct sam_sa  *sa_hndl;

/*
 * Case #1: Encrypting 16 bytes (1 block) using AES-CBC with 128-bit key
 * Key       : 0x06a9214036b8a15b512e03d534120006
 * IV        : 0x3dafba429d9eb430b422da802c9fac41
 * Plaintext : "Single block msg"
 * Ciphertext: 0xe353779c1079aeb82708942dbe77181a
 */

static uint8_t RFC3602_AES128_CBC_T1_KEY[] = {
	0x06, 0xa9, 0x21, 0x40, 0x36, 0xb8, 0xa1, 0x5b,
	0x51, 0x2e, 0x03, 0xd5, 0x34, 0x12, 0x00, 0x06
};

static uint8_t RFC3602_AES128_CBC_T1_IV[] = {
	0x3d, 0xaf, 0xba, 0x42, 0x9d, 0x9e, 0xb4, 0x30,
	0xb4, 0x22, 0xda, 0x80, 0x2c, 0x9f, 0xac, 0x41
};

static uint8_t RFC3602_AES128_CBC_T1_PT[] = { /* "Single block msg" */
	0x53, 0x69, 0x6E, 0x67, 0x6C, 0x65, 0x20, 0x62,
	0x6C, 0x6F, 0x63, 0x6B, 0x20, 0x6D, 0x73, 0x67
};

static uint8_t RFC3602_AES128_CBC_T1_CT[] = { /* Expected result */
	0xe3, 0x53, 0x77, 0x9c, 0x10, 0x79, 0xae, 0xb8,
	0x27, 0x08, 0x94, 0x2d, 0xbe, 0x77, 0x18, 0x1a
};

static struct sam_buf_info aes128_t1_buf = {
	.vaddr = RFC3602_AES128_CBC_T1_PT,
	.paddr = 0,
	.len = sizeof(RFC3602_AES128_CBC_T1_PT),
};

static struct sam_session_params aes_encypt_sa = {
	.dir = SAM_DIR_ENCRYPT,   /* operation direction: encode/decode */
	.cipher_alg = SAM_CIPHER_AES,  /* cipher algorithm */
	.cipher_mode = SAM_CIPHER_CBC, /* cipher mode */
	.cipher_iv = NULL,     /* default IV */
	.cipher_key = RFC3602_AES128_CBC_T1_KEY,    /* cipher key */
	.cipher_key_len = sizeof(RFC3602_AES128_CBC_T1_KEY), /* cipher key size (in bytes) */
	.auth_alg = SAM_AUTH_NONE, /* authentication algorithm */
	.auth_inner = NULL,    /* pointer to authentication inner block */
	.auth_outer = NULL,    /* pointer to authentication outer block */
	.auth_icv_len = 0,   /* Integrity Check Value (ICV) size (in bytes) */
	.auth_aad_len = 0,   /* Additional Data (AAD) size (in bytes) */
};

static struct sam_cio_op_params aes128_cbc_t1 = {
	.sa = NULL,
	.cookie = (void *)0x12345678,
	.num_bufs = 1,
	.src = &aes128_t1_buf,
	.dst = &aes128_t1_buf,
	.cipher_iv_offset = 0,
	.cipher_iv = RFC3602_AES128_CBC_T1_IV,
	.cipher_offset = 0,
	.cipher_len = sizeof(RFC3602_AES128_CBC_T1_PT),
	/* all auth fields are zero */
};

static void dump_buf(const unsigned char *p, unsigned int len)
{
	unsigned int i = 0, j;

	while (i < len) {
		j = 0;
		printf("\n%p: ", p + i);
		for (j = 0 ; j < 32 && i < len ; j++) {
			printf("%02x ", p[i]);
			i++;
		}
	}
	printf("\n");
}
static int create_session(struct sam_sa **hndl, const char *name)
{
	struct sam_session_params *sa_params;

	sa_params = &aes_encypt_sa;
	if (sam_session_create(cio_hndl, sa_params, hndl)) {
		printf("%s: failed\n", __func__);
		return 1;
	}
	return 0;
}

static int poll_results(struct sam_cio *cio, struct sam_cio_op_result *result,
			u16 *num)
{
	int rc;
	int count = 1000;

	while (count--) {
		rc = sam_cio_deq(cio, result, num);
		if (rc != -EBUSY)
			return rc;
	}
	/* Timeout */
	pr_err("%s: Timeout\n", __func__);
	return -EINVAL;
}

static int check_result(struct sam_sa *sa, struct sam_cio_op_params *request,
			struct sam_cio_op_result *result)
{
	/* Check result cookie */
	if (request->cookie != result->cookie) {
		pr_err("%s: Wrong cookie value: %p != %p\n",
			__func__, request->cookie, result->cookie);
		return -EINVAL;
	}
	printf("\nOutput buffer:");
	dump_buf(request->dst->vaddr, result->out_len);
	printf("\nExpected buffer:");
	dump_buf(RFC3602_AES128_CBC_T1_CT, sizeof(RFC3602_AES128_CBC_T1_CT));

	/* Compare output and expected data */
	if (memcmp(request->dst->vaddr, RFC3602_AES128_CBC_T1_CT, result->out_len)) {
		pr_err("%s: Test failed\n", __func__);
		return -EINVAL;
	}
	return 0;
}

int main(int argc, char **argv)
{
	struct sam_cio_params cio_params;
	struct sam_cio_op_result result;
	u16 num;
	int rc = 0;

	cio_params.match = "cio-0:0";
	cio_params.size = 32;
	cio_params.num_sessions = 64;
	cio_params.max_buf_size = 2048;

	if (sam_cio_init(&cio_params, &cio_hndl)) {
		printf("%s: initialization failed\n", argv[0]);
		return 1;
	}
	printf("%s successfully loaded\n", argv[0]);

	if (create_session(&sa_hndl, "aes_encrypt"))
		goto exit;

	pr_info("aes_encrypt session successfully created\n");

	printf("\nInput buffer:");
	dump_buf(RFC3602_AES128_CBC_T1_PT, sizeof(RFC3602_AES128_CBC_T1_PT));

	num = 1;
	aes128_cbc_t1.sa = sa_hndl;
	rc = sam_cio_enq(cio_hndl, &aes128_cbc_t1, &num);
	if ((rc != 0) || (num != 1)) {
		printf("%s: sam_cio_enq failed. num = %d, rc = %d\n",
			__func__, num, rc);
		goto exit;
	}
	/* polling for result */
	rc = poll_results(cio_hndl, &result, &num);
	if ((rc == 0) && (num == 1)) {
		/* check result */
		check_result(sa_hndl, &aes128_cbc_t1, &result);
	} else
		pr_err("No result: rc = %d, num = %d\n", rc, num);

exit:
	if (sa_hndl)
		sam_session_destroy(sa_hndl);

	if (sam_cio_deinit(cio_hndl)) {
		printf("%s: un-initialization failed\n", argv[0]);
		return 1;
	}
	printf("%s successfully unloaded\n", argv[0]);

	return 0;
}
