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

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <linux/ip.h>
#include <linux/udp.h>

#include "mv_std.h"
#include "env/mv_sys_dma.h"
#include "lib/lib_misc.h"
#include "lib/net.h"
#include "utils.h"
#include "sam_utils.h"

#include "mv_sam.h"

#define SAM_DMA_MEM_SIZE	(1 * 1024 * 1204) /* 1 MBytes */

#define MAX_BUF_SIZE		2048

static struct sam_cio *cio_hndl;
static struct sam_cio *cio_hndl_1;

static struct sam_sa  *sa_hndl;
static struct sam_sa  *sa_hndl_1;

static char *test_names[] = {
	/* 0 */ "ip4_dtls_aes_cbc_sha1",
	/* 1 */ "ip4_dtls_aes_gcm",
	/* 2 */ "ip4_capwap_dtls_aes_cbc_sha1",
};

static int test_id;
static int num_pkts = 1;
static u32 debug_flags;
static u32 verbose;

static u8 example_l2_header[] = {
	0x00, 0x01, 0x01, 0x01, 0x55, 0x66,
	0x00, 0x02, 0x02, 0x02, 0x77, 0x88,
	0x08, 0x00
};

static u8 example_ip4_header[] = {
	0x45, 0x00, 0x00, 0x00,
	0x00, 0x01, 0x00, 0x00,
	0x40, 0x11, 0x00, 0x00,
	0x50, 0x00, 0x00, 0x0A,
	0x3C, 0x00, 0x00, 0x0A
};

static u8 example_udp_header[] = {
	0x00, 0x63, 0x00, 0x64,
	0x00, 0x00, 0x00, 0x00
};

static u8 ExampleNonce[] = {
	0x40, 0xd6, 0xc1, 0xa7
};

static u8 example_aes_key[] = {
	0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
	0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f, 0x50
};

static u8 example_hmac_key[] = {
	0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b,
	0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b,
	0x0b, 0x0b, 0x0b, 0x0b
};

static u8 aes128_cbc_t1_pt[] = { /* "Single block msg" */
	0x53, 0x69, 0x6E, 0x67, 0x6C, 0x65, 0x20, 0x62,
	0x6C, 0x6F, 0x63, 0x6B, 0x20, 0x6D, 0x73, 0x67
};

static struct sam_buf_info aes128_t1_buf;
static u8 expected_data[MAX_BUF_SIZE];
static u32 expected_data_size;
static u64 enc_seq = 0x34; /* Default initial sequence number for encrypt */
static u64 dec_seq = 0x34; /* Default initial sequence number for decrypt */

static struct sam_session_params dtls_aes_cbc_sha1_sa = {
	.dir = SAM_DIR_ENCRYPT,				/* operation direction: encode/decode */
	.cipher_alg = SAM_CIPHER_AES,			/* cipher algorithm */
	.cipher_mode = SAM_CIPHER_CBC,			/* cipher mode */
	.cipher_key = example_aes_key,			/* cipher key */
	.cipher_key_len = sizeof(example_aes_key),	/* cipher key size (in bytes) */
	.auth_alg = SAM_AUTH_HMAC_SHA1,			/* authentication algorithm */
	.auth_key = example_hmac_key,			/* pointer to authentication key */
	.auth_key_len = sizeof(example_hmac_key),	/* authentication key size (in bytes) */
	.proto = SAM_PROTO_SSLTLS,
	.u.ssltls.version = SAM_DTLS_VERSION_1_0,	/* DTLS 1.0 version */
	.u.ssltls.epoch = 0x0D,				/* 13 - for DTLS only */
	.u.ssltls.is_ip6 = 0,				/* DTLS transported over: 1 - UDP/IPv6, 0 - UDP/IPv4 */
	.u.ssltls.is_udp_lite = 0,			/* 1 - use UDPLite, 0 - use UDP */
	.u.ssltls.is_capwap = 0,			/* 1 - use CAPWAP/DTLS, 0 - use DTLS */
	.u.ssltls.seq_mask_size = SAM_DTLS_MASK_64B,	/* anti-replay seq mask size */
	.u.ssltls.seq_mask[0] = 0x0,			/* up to 128-bit mask window used with inbound DTLS */
};

static struct sam_session_params dtls_aes_gcm_sa = {
	.dir = SAM_DIR_ENCRYPT,				/* operation direction: encode/decode */
	.cipher_alg = SAM_CIPHER_AES,			/* cipher algorithm */
	.cipher_mode = SAM_CIPHER_GCM,			/* cipher mode */
	.cipher_key = example_aes_key,			/* cipher key */
	.cipher_key_len = sizeof(example_aes_key),	/* cipher key size (in bytes) */
	.cipher_iv = ExampleNonce,			/* Nonce - first four bytes of IV */
	.auth_alg = SAM_AUTH_AES_GCM,			/* authentication algorithm */
	.auth_key = NULL,				/* pointer to authentication key */
	.auth_key_len = 0,				/* authentication key size (in bytes) */
	.proto = SAM_PROTO_SSLTLS,
	.u.ssltls.version = SAM_DTLS_VERSION_1_2,	/* DTLS 1.2 version */
	.u.ssltls.epoch = 0x0D,				/* 13 - for DTLS only */
	.u.ssltls.is_ip6 = 0,				/* DTLS transported over: 1 - UDP/IPv6, 0 - UDP/IPv4 */
	.u.ssltls.is_udp_lite = 0,			/* 1 - use UDPLite, 0 - use UDP */
	.u.ssltls.is_capwap = 0,			/* 1 - use CAPWAP/DTLS, 0 - use DTLS */
	.u.ssltls.seq_mask_size = SAM_DTLS_MASK_64B,	/* anti-replay seq mask size */
	.u.ssltls.seq_mask[0] = 0x0,			/* up to 128-bit mask window used with inbound DTLS */
};

static struct sam_cio_ssltls_params aes128_cbc_t1 = {
	.sa = NULL,
	.cookie = (void *)0x12345678,
	.num_bufs = 1,
	.src = &aes128_t1_buf,
	.dst = &aes128_t1_buf,
	/* all auth fields are zero */
};

static int create_session(struct sam_sa **hndl, const char *name, enum sam_dir dir)
{
	int rc;
	struct sam_session_params *sa_params;

	if (!strcmp(name, "ip4_dtls_aes_cbc_sha1")) {
		sa_params = &dtls_aes_cbc_sha1_sa;
	} else if (!strcmp(name, "ip4_dtls_aes_gcm")) {
		sa_params = &dtls_aes_gcm_sa;
	} else if (!strcmp(name, "ip4_capwap_dtls_aes_cbc_sha1")) {
		sa_params = &dtls_aes_cbc_sha1_sa;
		sa_params->u.ssltls.is_capwap = 1;
	} else {
		printf("%s - unknown session name\n", name);
		return -EINVAL;
	}
	sa_params->dir = dir;
	if (dir == SAM_DIR_ENCRYPT)
		sa_params->u.ssltls.seq = enc_seq;
	else if (dir == SAM_DIR_DECRYPT)
		sa_params->u.ssltls.seq = dec_seq;
	else {
		printf("%s: Unexpected direction %d\n", __func__, dir);
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

/* Build UDP packet and return total packet length */
static int build_udp_pkt_for_encrypt(struct sam_buf_info *buf_info)
{
	int offset, l3_offset, l4_offset;
	u8 *src_buf = buf_info->vaddr;
	struct iphdr *iph;
	struct udphdr *udph;

	offset = 0;
	memcpy(src_buf + offset, example_l2_header, sizeof(example_l2_header));
	offset = sizeof(example_l2_header);
	l3_offset = offset;

	memcpy(src_buf + offset, example_ip4_header, sizeof(example_ip4_header));
	offset += sizeof(example_ip4_header);
	l4_offset = offset;

	memcpy(src_buf + offset, example_udp_header, sizeof(example_udp_header));
	offset += sizeof(example_udp_header);

	memcpy(src_buf + offset, aes128_cbc_t1_pt, sizeof(aes128_cbc_t1_pt));
	offset += sizeof(aes128_cbc_t1_pt);

	iph = (struct iphdr *)(src_buf + l3_offset);

	/* Set IP length */
	iph->tot_len = htobe16(offset - l3_offset);

	/* Set IP checksum */
	iph->check = 0;
	iph->check = mv_ip4_csum((u16 *)iph, iph->ihl);

	udph = (struct udphdr *)(src_buf + l4_offset);

	/* Set UDP payload length */
	udph->len = htobe16(offset - l4_offset);

	/* Set UDP checksum */
	/* udph->check =  */

	/* Save buffer for compare after loopback */
	expected_data_size = offset;
	memcpy(expected_data, src_buf, offset);

	return offset;
}

static int check_udp_pkt_after_encrypt(struct sam_buf_info *buf_info, struct sam_cio_op_result *result)
{
	if (result->status != SAM_CIO_OK) {
		printf("%s: Error! result->status = %d\n",
			__func__, result->status);
		return -EFAULT;
	}
	if (result->out_len > buf_info->len) {
		printf("%s: result->out_len  = %u > buf_info->len = %u\n",
			__func__, result->out_len, buf_info->len);
		return -EINVAL;
	}
	if (verbose) {
		printf("\nAfter encryption: %d bytes\n", result->out_len);
		mv_mem_dump(buf_info->vaddr, result->out_len);
	}

	return 0;
}

static int check_udp_pkt_after_decrypt(struct sam_buf_info *buf_info, struct sam_cio_op_result *result)
{
	int err = 0;

	if (result->status != SAM_CIO_OK) {
		printf("%s: Error! result->status = %d\n",
			__func__, result->status);
		return -EFAULT;
	}
	if (result->out_len > buf_info->len) {
		printf("%s: result->out_len  = %u > buf_info->len = %u\n",
			__func__, result->out_len, buf_info->len);
		return -EINVAL;
	}
	if (verbose) {
		printf("\nAfter decryption: %d bytes\n", result->out_len);
		mv_mem_dump(buf_info->vaddr, result->out_len);

		printf("\nExpected data: %d bytes\n", expected_data_size);
		mv_mem_dump(expected_data, expected_data_size);
	}
	/* Compare output and expected data */
	if (result->out_len != expected_data_size) {
		printf("Error: out_len = %u != expected_data_size = %u\n",
			result->out_len, expected_data_size);
		err = -EINVAL;
	} else if (memcmp(buf_info->vaddr, expected_data, result->out_len)) {
		printf("Error: out_data != expected_data\n");
		err = -EFAULT;
	}
	return err;
}

static void usage(char *progname)
{
	int id;

	printf("Usage: %s [OPTIONS]\n", MVAPPS_NO_PATH(progname));
	printf("OPTIONS are optional:\n");
	printf("\t-t   <number>    - Test ID (default: %d)\n", test_id);
	for (id = 0; id < ARRAY_SIZE(test_names); id++)
		printf("\t                 %d - %s\n", id, test_names[id]);
	printf("\t-n   <number>    - Number of packets (default: %d)\n", num_pkts);
	printf("\t-seq <enc> <dec> - Initial sequence id for encrypt and decrypt\n");
	printf("\t-f   <bitmask>   - Debug flags: 0x%x - SA, 0x%x - CIO. (default: 0x%x)\n",
					SAM_SA_DEBUG_FLAG, SAM_CIO_DEBUG_FLAG, debug_flags);
	printf("\t-v               - Increase verbose level (default is 0).\n");
}

static int parse_args(int argc, char *argv[])
{
	int i = 1;

	while (i < argc) {
		if ((strcmp(argv[i], "?") == 0) ||
		    (strcmp(argv[i], "-h") == 0) ||
		    (strcmp(argv[i], "--help") == 0)) {
			usage(argv[0]);
			exit(0);
		}
		if (strcmp(argv[i], "-t") == 0) {
			if (argc < (i + 2)) {
				pr_err("Invalid number of arguments!\n");
				return -EINVAL;
			}
			if (argv[i + 1][0] == '-') {
				pr_err("Invalid arguments format!\n");
				return -EINVAL;
			}
			test_id = atoi(argv[i + 1]);
			i += 2;
		} else if (strcmp(argv[i], "-n") == 0) {
			if (argc < (i + 2)) {
				pr_err("Invalid number of arguments!\n");
				return -EINVAL;
			}
			if (argv[i + 1][0] == '-') {
				pr_err("Invalid arguments format!\n");
				return -EINVAL;
			}
			num_pkts = atoi(argv[i + 1]);
			i += 2;
		} else if (strcmp(argv[i], "-seq") == 0) {
			if (argc < (i + 3)) {
				pr_err("Invalid number of arguments!\n");
				return -EINVAL;
			}
			if (argv[i + 1][0] == '-') {
				pr_err("Invalid arguments format!\n");
				return -EINVAL;
			}
			if (argv[i + 2][0] == '-') {
				pr_err("Invalid arguments format!\n");
				return -EINVAL;
			}
			enc_seq = atoi(argv[i + 1]);
			dec_seq = atoi(argv[i + 2]);
			i += 3;
		} else if (strcmp(argv[i], "-f") == 0) {
			int scanned;

			if (argc < (i + 2)) {
				pr_err("Invalid number of arguments!\n");
				return -EINVAL;
			}
			if (argv[i + 1][0] == '-') {
				pr_err("Invalid arguments format!\n");
				return -EINVAL;
			}
			scanned = sscanf(argv[i + 1], "0x%x", &debug_flags);
			if (scanned != 1) {
				pr_err("Invalid number if scanned arguments: %d != 1\n",
					scanned);
				return -EINVAL;
			}
			i += 2;
		} else if (strcmp(argv[i], "-v") == 0) {
			verbose++;
			i += 1;
		} else {
			pr_err("argument (%s) not supported!\n", argv[i]);
			return -EINVAL;
		}
	}
	if (test_id >= ARRAY_SIZE(test_names)) {
		pr_err("test_id (%d) is out of range [0 .. %d]\n",
			test_id, (unsigned)ARRAY_SIZE(test_names));

		return -EINVAL;
	}

	/* Print all inputs arguments */
	printf("Test ID     : %d\n", test_id);
	printf("Test name   : %s\n", test_names[test_id]);
	printf("Debug flags : 0x%x\n", debug_flags);
	printf("SeqId       : 0x%lx -> 0x%lx\n", enc_seq, dec_seq);

	return 0;
}

int main(int argc, char **argv)
{
	struct sam_init_params init_params;
	struct sam_cio_params cio_params;
	struct sam_cio_op_result result;
	u16 num;
	int rc = 0;
	int input_size, i;

	rc = parse_args(argc, argv);
	if (rc)
		return rc;

	rc = mv_sys_dma_mem_init(SAM_DMA_MEM_SIZE);
	if (rc) {
		pr_err("Can't initialize %d KBytes of DMA memory area, rc = %d\n", SAM_DMA_MEM_SIZE, rc);
		return rc;
	}

	aes128_t1_buf.len = MAX_BUF_SIZE;
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
	sam_set_debug_flags(debug_flags);

	if (create_session(&sa_hndl, test_names[test_id], SAM_DIR_ENCRYPT))
		goto exit;

	pr_info("%s encrypt session created on %s\n", test_names[test_id], "cio-0:0");

	if (create_session(&sa_hndl_1, test_names[test_id], SAM_DIR_DECRYPT))
		goto exit;

	pr_info("%s decrypt session created on %s\n", test_names[test_id], "cio-0:1");

	memset(aes128_t1_buf.vaddr, 0, aes128_t1_buf.len);

	/* Build input packet for encryption */
	input_size = build_udp_pkt_for_encrypt(&aes128_t1_buf);

	if (verbose) {
		printf("\nInput buffer    : %d bytes\n", input_size);
		mv_mem_dump(aes128_t1_buf.vaddr, input_size);
	}
	for (i = 0; i < num_pkts; i++) {
		/* Do encrypt */
		num = 1;
		aes128_cbc_t1.sa = sa_hndl;
		aes128_cbc_t1.l3_offset = sizeof(example_l2_header);
		aes128_cbc_t1.pkt_size = input_size;
		aes128_cbc_t1.type = SAM_DTLS_DATA;

		rc = sam_cio_enq_ssltls(cio_hndl, &aes128_cbc_t1, &num);
		if ((rc != 0) || (num != 1)) {
			printf("%s: sam_cio_enq failed. num = %d, rc = %d\n",
				__func__, num, rc);
			goto exit;
		}
		/* polling for result */
		rc = poll_results(cio_hndl, &result, &num);
		if ((rc != 0) || (num != 1)) {
			pr_err("No result: rc = %d, num = %d\n", rc, num);
			goto exit;
		}
		if (check_udp_pkt_after_encrypt(&aes128_t1_buf, &result))
			goto exit;

		/* Do decrypt */
		num = 1;
		aes128_cbc_t1.sa = sa_hndl_1;
		aes128_cbc_t1.pkt_size = result.out_len;

		rc = sam_cio_enq_ssltls(cio_hndl_1, &aes128_cbc_t1, &num);
		if ((rc != 0) || (num != 1)) {
			printf("%s: sam_cio_enq failed. num = %d, rc = %d\n",
				__func__, num, rc);
			goto exit;
		}
		/* polling for result */
		rc = poll_results(cio_hndl_1, &result, &num);
		if ((rc != 0) || (num != 1)) {
			pr_err("No result: rc = %d, num = %d\n", rc, num);
			goto exit;
		}
		if (check_udp_pkt_after_decrypt(&aes128_t1_buf, &result))
			goto exit;
	}
	printf("\n%s: success\n\n", argv[0]);

exit:
	if (sa_hndl) {
		if (sam_session_destroy(sa_hndl))
			printf("Can't destroy sa_hndl session");
	}
	if (sam_cio_flush(cio_hndl))
		printf("%s: sam_cio_flush failed for cio_hndl\n", argv[0]);

	if (sa_hndl_1) {
		if (sam_session_destroy(sa_hndl_1))
			printf("Can't destroy sa_hndl_1 session");
	}
	if (sam_cio_flush(cio_hndl_1))
		printf("%s: sam_cio_flush failed for cio_hndl_1\n", argv[0]);

	if (aes128_t1_buf.vaddr)
		mv_sys_dma_mem_free(aes128_t1_buf.vaddr);

	app_sam_show_cio_stats(cio_hndl, "encrypt", 1);
	app_sam_show_cio_stats(cio_hndl_1, "decrypt", 1);
	app_sam_show_stats(1);

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
