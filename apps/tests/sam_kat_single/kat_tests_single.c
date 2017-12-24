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

#include "env/mv_sys_dma.h"

#include "mv_sam.h"

#define SAM_DMA_MEM_SIZE	(1 * 1024 * 1204) /* 1 MBytes */

static struct sam_cio *cio_hndl;
static struct sam_cio *cio_hndl_1;

static struct sam_sa  *sa_hndl;
static struct sam_sa  *sa_hndl_1;

/*
 * Case #1: Encrypting 16 bytes (1 block) using AES-CBC with 128-bit key
 * Key       : 0x06a9214036b8a15b512e03d534120006
 * IV        : 0x3dafba429d9eb430b422da802c9fac41
 * Plaintext : "Single block msg"
 * Ciphertext: 0xe353779c1079aeb82708942dbe77181a
 */

static u8 RFC3602_AES128_CBC_T1_KEY[] = {
	0x06, 0xa9, 0x21, 0x40, 0x36, 0xb8, 0xa1, 0x5b,
	0x51, 0x2e, 0x03, 0xd5, 0x34, 0x12, 0x00, 0x06
};

static u8 RFC3602_AES128_CBC_T1_IV[] = {
	0x3d, 0xaf, 0xba, 0x42, 0x9d, 0x9e, 0xb4, 0x30,
	0xb4, 0x22, 0xda, 0x80, 0x2c, 0x9f, 0xac, 0x41
};

static u8 RFC3602_AES128_CBC_T1_PT[] = { /* "Single block msg" */
	0x53, 0x69, 0x6E, 0x67, 0x6C, 0x65, 0x20, 0x62,
	0x6C, 0x6F, 0x63, 0x6B, 0x20, 0x6D, 0x73, 0x67
};

static u8 RFC3602_AES128_CBC_T1_CT[] = { /* Expected result */
	0xe3, 0x53, 0x77, 0x9c, 0x10, 0x79, 0xae, 0xb8,
	0x27, 0x08, 0x94, 0x2d, 0xbe, 0x77, 0x18, 0x1a
};

static struct sam_buf_info aes128_t1_buf;

static struct sam_session_params aes_cbc_sa = {
	.dir = SAM_DIR_ENCRYPT,   /* operation direction: encode/decode */
	.cipher_alg = SAM_CIPHER_AES,  /* cipher algorithm */
	.cipher_mode = SAM_CIPHER_CBC, /* cipher mode */
	.cipher_iv = NULL,     /* default IV */
	.cipher_key = RFC3602_AES128_CBC_T1_KEY,    /* cipher key */
	.cipher_key_len = sizeof(RFC3602_AES128_CBC_T1_KEY), /* cipher key size (in bytes) */
	.auth_alg = SAM_AUTH_NONE, /* authentication algorithm */
	.auth_key = NULL,    /* authentication key */
	.auth_key_len = 0,   /* authentication key size (in bytes) */
	.proto = SAM_PROTO_NONE, /* Basic crypto session */
	.u.basic.auth_icv_len = 0,   /* Integrity Check Value (ICV) size (in bytes) */
	.u.basic.auth_aad_len = 0,   /* Additional Data (AAD) size (in bytes) */
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
	int rc;
	struct sam_session_params *sa_params;

	if (!strcmp(name, "aes_cbc_encrypt")) {
		sa_params = &aes_cbc_sa;
		sa_params->dir = SAM_DIR_ENCRYPT;
	} else if (!strcmp(name, "aes_cbc_decrypt")) {
		sa_params = &aes_cbc_sa;
		sa_params->dir = SAM_DIR_DECRYPT;
	} else {
		printf("%s: unknown session name - %s\n", __func__, name);
		return -EINVAL;
	}

	rc = sam_session_create(sa_params, hndl);
	if (rc) {
		printf("%s: failed - rc = %d\n", __func__, rc);
		return rc;
	}
	return 0;
}

static int poll_results(struct sam_cio *cio, struct sam_cio_op_result *result,
			u16 *num)
{
	int rc;
	int count = 1000;
	u16 to_deq = *num;

	while (count--) {
		rc = sam_cio_deq(cio, result, num);
		if (rc) {
			printf("%s: sam_cio_deq failed. num = %d, rc = %d\n",
					__func__, *num, rc);
			return rc;
		}
		if (*num)
			return 0;

		*num = to_deq;
	}
	/* Timeout */
	pr_err("%s: Timeout\n", __func__);
	return -EINVAL;
}

static int check_result(enum sam_dir dir, struct sam_cio_op_params *request,
			struct sam_cio_op_result *result)
{
	u8 *expected_buf, *input_buf;
	int expected_size, input_size;
	char *name;

	if (dir == SAM_DIR_ENCRYPT) {
		input_buf = RFC3602_AES128_CBC_T1_PT;
		input_size = sizeof(RFC3602_AES128_CBC_T1_PT);
		expected_buf = RFC3602_AES128_CBC_T1_CT;
		expected_size = sizeof(RFC3602_AES128_CBC_T1_CT);
		name = "Encrypt";
	} else {
		input_buf = RFC3602_AES128_CBC_T1_CT;
		input_size = sizeof(RFC3602_AES128_CBC_T1_CT);
		expected_buf = RFC3602_AES128_CBC_T1_PT;
		expected_size = sizeof(RFC3602_AES128_CBC_T1_PT);
		name = "Decrypt";
	}
	if (result->status != SAM_CIO_OK) {
		printf("%s: Error! result->status = %d\n",
			__func__, result->status);
		return -EINVAL;
	}

	printf("\nInput buffer:");
	dump_buf(input_buf, input_size);

	/* Check result cookie */
	if (request->cookie != result->cookie) {
		pr_err("%s: Wrong cookie value: %p != %p\n",
			__func__, request->cookie, result->cookie);
		return -EINVAL;
	}
	printf("\nOutput buffer:");
	dump_buf(request->dst->vaddr, result->out_len);

	printf("\nExpected buffer:");
	dump_buf(expected_buf, expected_size);

	/* Compare output and expected data */
	if (memcmp(request->dst->vaddr, expected_buf, result->out_len)) {
		pr_err("%s: Test failed\n", __func__);
		return -EINVAL;
	}
	printf("\n");
	printf("%s test: success\n", name);
	printf("\n");

	return 0;
}

int main(int argc, char **argv)
{
	struct sam_cio_params cio_params;
	struct sam_init_params init_params;
	struct sam_cio_op_result result;
	u16 num;
	int rc = 0;

	rc = mv_sys_dma_mem_init(SAM_DMA_MEM_SIZE);
	if (rc) {
		pr_err("Can't initialize %d KBytes of DMA memory area, rc = %d\n", SAM_DMA_MEM_SIZE, rc);
		return rc;
	}

	aes128_t1_buf.len = 2048;
	aes128_t1_buf.vaddr = mv_sys_dma_mem_alloc(aes128_t1_buf.len, 16);
	aes128_t1_buf.paddr = mv_sys_dma_mem_virt2phys(aes128_t1_buf.vaddr);

	if (!aes128_t1_buf.vaddr || !aes128_t1_buf.paddr) {
		pr_err("Can't allocate DMA buffer of %d bytes\n", aes128_t1_buf.len);
		goto exit;
	}

	pr_info("DMA Buffer %d bytes allocated: vaddr = %p, paddr = %p\n",
		aes128_t1_buf.len, aes128_t1_buf.vaddr, (void *)aes128_t1_buf.paddr);

	init_params.max_num_sessions = 64;
	sam_init(&init_params);

	cio_params.match = "cio-0:0";
	cio_params.size = 32;

	if (sam_cio_init(&cio_params, &cio_hndl)) {
		printf("%s: initialization failed\n", argv[0]);
		return 1;
	}

	cio_params.match = "cio-0:1";
	cio_params.size = 32;

	if (sam_cio_init(&cio_params, &cio_hndl_1)) {
		printf("%s: initialization failed\n", argv[0]);
		return 1;
	}

	memset(aes128_t1_buf.vaddr, 0, aes128_t1_buf.len);
	memcpy(aes128_t1_buf.vaddr, RFC3602_AES128_CBC_T1_PT, sizeof(RFC3602_AES128_CBC_T1_PT));

	if (create_session(&sa_hndl, "aes_cbc_encrypt"))
		goto exit;

	pr_info("aes_cbc_encrypt session created\n");

	if (create_session(&sa_hndl_1, "aes_cbc_decrypt"))
		goto exit;

	pr_info("aes_cbc_decrypt session created\n");

	/* Do encrypt */
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
		check_result(SAM_DIR_ENCRYPT, &aes128_cbc_t1, &result);
	} else
		pr_err("No result: rc = %d, num = %d\n", rc, num);

	/* Do decrypt */
	num = 1;
	aes128_cbc_t1.sa = sa_hndl_1;
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
		check_result(SAM_DIR_DECRYPT, &aes128_cbc_t1, &result);
	} else
		pr_err("No result: rc = %d, num = %d\n", rc, num);

exit:
	if (aes128_t1_buf.vaddr)
		mv_sys_dma_mem_free(aes128_t1_buf.vaddr);

	if (sa_hndl)
		sam_session_destroy(sa_hndl);

	if (sa_hndl_1)
		sam_session_destroy(sa_hndl_1);

	if (cio_hndl) {
		if (sam_cio_deinit(cio_hndl)) {
			printf("%s: un-initialization failed\n", argv[0]);
			return 1;
		}
	}
	if (cio_hndl_1) {
		if (sam_cio_deinit(cio_hndl_1)) {
			printf("%s: un-initialization failed\n", argv[0]);
			return 1;
		}
	}
	sam_deinit();

	return 0;
}
