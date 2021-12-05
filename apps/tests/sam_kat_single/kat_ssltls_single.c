/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <netinet/if_ether.h>
#include <netinet/ip.h>
#include <netinet/ip6.h>
#include <netinet/udp.h>
#include <sys/time.h>

#include "mv_std.h"
#include "env/mv_sys_dma.h"
#include "lib/lib_misc.h"
#include "lib/net.h"
#include "utils.h"
#include "sam_utils.h"

#include "mv_sam.h"

#define SAM_DMA_MEM_SIZE	(1 * 1024 * 1204) /* 1 MBytes */

#define MAX_BUF_SIZE			2048
#define NUM_CONCURRENT_REQUESTS		128
#define NUM_BUFS			(NUM_CONCURRENT_REQUESTS * 2)
#define MAX_BURST_SIZE			(NUM_CONCURRENT_REQUESTS / 4)

static char cio_enc_match[16] = "cio-0:0";
static char cio_dec_match[16] = "cio-0:1";
static struct sam_cio *cio_enc;
static struct sam_cio *cio_dec;

static struct sam_sa  *sa_enc;
static struct sam_sa  *sa_dec;

static bool sa_is_ip6;

static char *test_names[] = {
	/* 0 */ "ip4_dtls_aes_cbc_sha1",
	/* 1 */ "ip4_dtls_aes_gcm",
	/* 2 */ "ip4_capwap_dtls_aes_cbc_sha1",
	/* 3 */ "ip6_dtls_aes_cbc_sha1",
	/* 4 */ "ip6_dtls_aes_gcm",
	/* 5 */ "ip6_capwap_dtls_aes_cbc_sha1",
};


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

static u8 example_ip6_header[] = {
	0x60, 0x30, 0x00, 0x00, 0x05, 0xA2, 0x11, 0x40,
	0xFE, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x02, 0x00, 0x01, 0xFF, 0xFE, 0x00, 0x01, 0x00,
	0x35, 0x55, 0x55, 0x55, 0x66, 0x66, 0x66, 0x66,
	0x77, 0x77, 0x77, 0x77, 0x88, 0x88, 0x88, 0x88
};

static u8 example_udp_header[] = {
	0x00, 0x63, 0x00, 0x64,
	0x00, 0x00, 0x00, 0x00
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

static u8 ExampleNonce[] = {
	0x40, 0xd6, 0xc1, 0xa7
};

static struct sam_buf_info	input_buf;
static struct sam_buf_info	output_bufs[NUM_BUFS];
int				next_out_buf;

static struct sam_cio_ssltls_params  enc_requests[MAX_BURST_SIZE];
static struct sam_cio_ssltls_params  dec_requests[MAX_BURST_SIZE];

static u8	expected_data[MAX_BUF_SIZE];
static u32	expected_data_size;
static u32	total_errors;

static u64 enc_seq = 0x34; /* Default initial sequence number for encrypt */
static u64 dec_seq = 0x34; /* Default initial sequence number for decrypt */
static int test_id = 1;
static int num_pkts = 1;
static int payload_size = sizeof(aes128_cbc_t1_pt);
static u32 debug_flags;
static u32 verbose;
static int loopback = 1;
static int num_checked;
static int num_to_check = 10;
static bool running;
static int num_to_print_mod = 1;

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
	.u.ssltls.seq_mask_size = SAM_ANTI_REPLY_MASK_64B,	/* anti-replay seq mask size */
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
	.u.ssltls.epoch = 0x000D,			/* 13 - for DTLS only */
	.u.ssltls.is_ip6 = 0,				/* DTLS transported over: 1 - UDP/IPv6, 0 - UDP/IPv4 */
	.u.ssltls.is_udp_lite = 0,			/* 1 - use UDPLite, 0 - use UDP */
	.u.ssltls.is_capwap = 0,			/* 1 - use CAPWAP/DTLS, 0 - use DTLS */
	.u.ssltls.seq_mask_size = SAM_ANTI_REPLY_MASK_64B,	/* anti-replay seq mask size */
	.u.ssltls.seq_mask[0] = 0x0,			/* up to 128-bit mask window used with inbound DTLS */
};

static void sigint_h(int sig)
{
	printf("\nInterrupted by signal #%d\n", sig);
	running = false;
	signal(SIGINT, SIG_DFL);
}

static void print_results(int test, char *test_name, int input_size,
			int operations, int errors,
			struct timeval *tv_start, struct timeval *tv_end)
{
	u32 usecs, secs, msecs, kpps, mbps;

	secs = tv_end->tv_sec - tv_start->tv_sec;
	if (tv_end->tv_usec < tv_start->tv_usec) {
		secs -= 1;
		usecs = (tv_end->tv_usec + 1000000) - tv_start->tv_usec;
	} else
		usecs = tv_end->tv_usec - tv_start->tv_usec;

	msecs = secs * 1000 + usecs / 1000;

	kpps = operations / msecs;
	mbps = kpps * input_size * 8 / 1000;

	printf("\n");
	if (errors == 0)
		printf("%2d. %-32s: passed %d times * %d Bytes        - %u.%03u secs\n",
			test, test_name, operations, input_size, secs, usecs / 1000);
	else
		printf("%2d. %-32s: failed %d of %d times * %d Bytes  - %u.%03u secs\n",
			test, test_name, errors, operations, input_size, secs, usecs / 1000);

	printf("    %32s: %u Kpps, %u Mbps\n", "Rate ", kpps, mbps);
}

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
	} else if (!strcmp(name, "ip6_dtls_aes_cbc_sha1")) {
		sa_params = &dtls_aes_cbc_sha1_sa;
		sa_params->u.ssltls.is_ip6 = 1;
		sa_is_ip6 = true;
	} else if (!strcmp(name, "ip6_dtls_aes_gcm")) {
		sa_params = &dtls_aes_gcm_sa;
		sa_params->u.ssltls.is_ip6 = 1;
		sa_is_ip6 = true;
	} else if (!strcmp(name, "ip6_capwap_dtls_aes_cbc_sha1")) {
		sa_params = &dtls_aes_cbc_sha1_sa;
		sa_params->u.ssltls.is_capwap = 1;
		sa_params->u.ssltls.is_ip6 = 1;
		sa_is_ip6 = true;
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
static int build_udp_pkt_for_encrypt(struct sam_buf_info *buf_info, int payload_size, bool is_ip6)
{
	int offset, l3_offset, l4_offset;
	u8 *src_buf = buf_info->vaddr;
	struct ether_header *ethh;
	struct iphdr *iph;
	struct udphdr *udph;

	offset = 0;
	memcpy(src_buf, example_l2_header, sizeof(example_l2_header));
	ethh = (struct ether_header *)src_buf;
	offset = sizeof(example_l2_header);
	l3_offset = offset;

	if (is_ip6) {
		ethh->ether_type = htobe16(ETHERTYPE_IPV6);
		memcpy(src_buf + offset, example_ip6_header, sizeof(example_ip6_header));
		offset += sizeof(example_ip6_header);
	} else {
		ethh->ether_type = htobe16(ETHERTYPE_IP);
		memcpy(src_buf + offset, example_ip4_header, sizeof(example_ip4_header));
		offset += sizeof(example_ip4_header);
	}
	l4_offset = offset;

	memcpy(src_buf + offset, example_udp_header, sizeof(example_udp_header));
	offset += sizeof(example_udp_header);

	/* Copy payload */
	while (payload_size) {
		int copy_size = min_t(int, payload_size, sizeof(aes128_cbc_t1_pt));

		memcpy(src_buf + offset, aes128_cbc_t1_pt, copy_size);
		offset += copy_size;
		payload_size -= copy_size;
	}

	if (is_ip6) {

		struct ip6_hdr *ip6h = (struct ip6_hdr *)(src_buf + l3_offset);

		/* Set IP length */
		ip6h->ip6_plen = htobe16(offset - l4_offset);
		ip6h->ip6_nxt = IPPROTO_UDP;
	} else {
		iph = (struct iphdr *)(src_buf + l3_offset);

		/* Set IP length */
		iph->tot_len = htobe16(offset - l3_offset);
		iph->protocol = IPPROTO_UDP;

		/* Set IP checksum */
		iph->check = 0;
		iph->check = mv_ip4_csum((u16 *)iph, iph->ihl);
	}
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

static int check_results_after_encrypt(struct sam_cio_op_result *results, int idx, int num)
{
	int i, err = 0;
	struct sam_buf_info *buf_info;

	for (i = 0; i < num; i++) {
		if (results[i].status != SAM_CIO_OK) {
			printf("%s: Error! result->status = %d\n",
				__func__, results[i].status);
			err++;
			continue;
		}
		buf_info = (struct sam_buf_info *)results[i].cookie;
		if (results[i].out_len > buf_info->len) {
			printf("%s: result->out_len  = %u > buf_info->len = %u\n",
				__func__, results[i].out_len, buf_info->len);
			err++;
			continue;
		}
		if (verbose && (((idx + i) % num_to_print_mod) == 0)) {
			printf("\nAfter encryption #%u: %d bytes\n", idx + i, results[i].out_len);
			mv_mem_dump(buf_info->vaddr, results[i].out_len);
		}
	}
	return err;
}

static int check_results_after_decrypt(struct sam_cio_op_result *results, int idx, int num)
{
	int i, to_print, new_err = 0, err = 0;
	struct sam_buf_info *buf_info;

	for (i = 0; i < num; i++) {
		to_print = (verbose && (((idx + i) % num_to_print_mod) == 0));
		if (results[i].status != SAM_CIO_OK) {
			printf("%s: Error! result->status = %d\n",
				__func__, results[i].status);
			err++;
			continue;
		}
		buf_info = (struct sam_buf_info *)results[i].cookie;

		if (results[i].out_len > buf_info->len) {
			printf("%s: result->out_len  = %u > buf_info->len = %u\n",
				__func__, results[i].out_len, buf_info->len);
			results[i].out_len = buf_info->len;
			err++;
			continue;
		}
		if (results[i].out_len != expected_data_size) {
			printf("Error: out_len = %u != expected_data_size = %u\n",
				results[i].out_len, expected_data_size);
			new_err = 1;
		}
		if (num_checked < num_to_check) {
			if (memcmp(buf_info->vaddr, expected_data, results[i].out_len)) {
				/* Compare output and expected data */
				printf("Error: out_data != expected_data\n");
				new_err = 1;
			}
			num_checked++;
		}
		if (to_print || new_err) {
			printf("\nAfter decryption #%u: %d bytes\n", idx + i, results[i].out_len);
			mv_mem_dump(buf_info->vaddr, results[i].out_len);
		}
		if (new_err) {
			new_err = 0;
			err++;
			printf("\nExpected data: %d bytes\n", expected_data_size);
			mv_mem_dump(expected_data, expected_data_size);
		}
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
	printf("\t-s   <number>    - Payload size (default: %u)\n",
		(unsigned)sizeof(aes128_cbc_t1_pt));
	printf("\t-n   <number>    - Number of packets (default: %d)\n", num_pkts);
	printf("\t-c   <number>    - Number of packets to check (default: %d)\n", num_to_check);
	printf("\t-seq <enc> <dec> - Initial sequence id (default: 0x%" PRIx64 " 0x%" PRIx64 ")\n",
		enc_seq, dec_seq);
	printf("\t-f   <bitmask>   - Debug flags: 0x%x - SA, 0x%x - CIO. (default: 0x%x)\n",
				     SAM_SA_DEBUG_FLAG, SAM_CIO_DEBUG_FLAG, debug_flags);
	printf("\t-v [num]         - Enable print of plain and cipher data each <num> packets\n");
	printf("\t--enc            - Do encrypt only (default: enc+dec)\n");
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
		} else if (strcmp(argv[i], "--enc") == 0) {
			loopback = 0;
			i += 1;
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
		} else if (strcmp(argv[i], "-s") == 0) {
			if (argc < (i + 2)) {
				pr_err("Invalid number of arguments!\n");
				return -EINVAL;
			}
			if (argv[i + 1][0] == '-') {
				pr_err("Invalid arguments format!\n");
				return -EINVAL;
			}
			payload_size = atoi(argv[i + 1]);
			i += 2;
		} else if (strcmp(argv[i], "-c") == 0) {
			if (argc < (i + 2)) {
				pr_err("Invalid number of arguments!\n");
				return -EINVAL;
			}
			if (argv[i + 1][0] == '-') {
				pr_err("Invalid arguments format!\n");
				return -EINVAL;
			}
			num_to_check = atoi(argv[i + 1]);
			i += 2;
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

			if ((argc > i) && (argv[i][0] != '-')) {
				num_to_print_mod = atoi(argv[i]);
				i += 1;
			}
		} else {
			pr_err("argument (%s) not supported!\n", argv[i]);
			return -EINVAL;
		}
	}
	if (test_id >= ARRAY_SIZE(test_names)) {
		pr_err("test_id (%d) is out of range [0 .. %d]\n",
			test_id, (unsigned)ARRAY_SIZE(test_names) - 1);

		return -EINVAL;
	}

	/* Print all inputs arguments */
	printf("Test ID                 : %d\n", test_id);
	printf("Test name               : %s\n", test_names[test_id]);
	printf("Payload size            : %d\n", payload_size);
	printf("Number of packets       : %d\n", num_pkts);
	printf("Number to check         : %u\n", num_to_check);
	printf("Debug flags             : 0x%x\n", debug_flags);
	printf("Initial sequence numbers: 0x%" PRIx64 " -> 0x%" PRIx64 "\n",
		enc_seq, dec_seq);
	if (verbose)
		printf("Print data each         : %u packets\n", num_to_print_mod);

	return 0;
}

int main(int argc, char **argv)
{
	struct sam_init_params init_params;
	struct sam_cio_params cio_params;
	struct sam_cio_op_result results[NUM_CONCURRENT_REQUESTS];
	u16 num, num_done;
	int num_bufs, pkt_size, to_enc, rc = 0;
	int i, enc_in_progress, dec_in_progress, not_completed;
	struct timeval tv_start, tv_end;

	rc = parse_args(argc, argv);
	if (rc)
		return rc;

	rc = mv_sys_dma_mem_init(SAM_DMA_MEM_SIZE);
	if (rc) {
		pr_err("Can't initialize %d KBytes of DMA memory area, rc = %d\n", SAM_DMA_MEM_SIZE, rc);
		return rc;
	}

	input_buf.len = MAX_BUF_SIZE;
	input_buf.vaddr = mv_sys_dma_mem_alloc(input_buf.len, 16);
	input_buf.paddr = mv_sys_dma_mem_virt2phys(input_buf.vaddr);
	if (!input_buf.vaddr || !input_buf.paddr) {
		pr_err("Can't allocate DMA buffer of %d bytes\n", input_buf.len);
		goto exit;
	}

	pr_info("DMA Buffer %d bytes allocated: vaddr = %p, paddr = %p\n",
		input_buf.len, input_buf.vaddr, (void *)(uintptr_t)input_buf.paddr);

	num_bufs = min(num_pkts, NUM_BUFS);
	for (i = 0; i < num_bufs; i++) {
		output_bufs[i].len = MAX_BUF_SIZE;
		output_bufs[i].vaddr = mv_sys_dma_mem_alloc(output_bufs[i].len, 16);
		output_bufs[i].paddr = mv_sys_dma_mem_virt2phys(output_bufs[i].vaddr);
		if (!output_bufs[i].vaddr || !output_bufs[i].paddr) {
			pr_err("Can't allocate DMA buffer of %d bytes\n", output_bufs[i].len);
			goto exit;
		}
		memset(output_bufs[i].vaddr, 0, MAX_BUF_SIZE);
	}

	init_params.max_num_sessions = 64;
	sam_init(&init_params);

	cio_params.match = cio_enc_match;
	cio_params.size = NUM_CONCURRENT_REQUESTS;

	if (sam_cio_init(&cio_params, &cio_enc)) {
		printf("%s: initialization failed\n", argv[0]);
		return 1;
	}
	sam_set_debug_flags(debug_flags);

	if (create_session(&sa_enc, test_names[test_id], SAM_DIR_ENCRYPT))
		goto exit;

	pr_info("%s encrypt session created\n", test_names[test_id]);

	if (loopback) {
		cio_params.match = cio_dec_match;
		cio_params.size = NUM_CONCURRENT_REQUESTS;

		if (sam_cio_init(&cio_params, &cio_dec)) {
			printf("%s: initialization failed\n", argv[0]);
			return 1;
		}
	}
	if (create_session(&sa_dec, test_names[test_id], SAM_DIR_DECRYPT))
		goto exit;

	pr_info("%s decrypt session created\n", test_names[test_id]);

	/* Build input packet for encryption */
	pkt_size = build_udp_pkt_for_encrypt(&input_buf, payload_size, sa_is_ip6);

	if (verbose) {
		printf("\nInput packet    : %d bytes\n", pkt_size);
		mv_mem_dump(input_buf.vaddr, pkt_size);
	}

	/* Prepare requests */
	for (i = 0; i < MAX_BURST_SIZE; i++) {
		enc_requests[i].sa = sa_enc;
		enc_requests[i].l3_offset = sizeof(example_l2_header);
		enc_requests[i].pkt_size = pkt_size;
		enc_requests[i].num_bufs = 1;
		enc_requests[i].src = &input_buf;
		enc_requests[i].dst = NULL;
		enc_requests[i].cookie = NULL;

		dec_requests[i].sa = sa_dec;
		dec_requests[i].l3_offset = sizeof(example_l2_header);
		dec_requests[i].pkt_size = 0;
		dec_requests[i].num_bufs = 1;
		dec_requests[i].src = NULL;
		dec_requests[i].dst = NULL;
		dec_requests[i].cookie = NULL;
	}

	to_enc = num_pkts;
	not_completed = num_pkts;
	enc_in_progress = dec_in_progress = 0;
	i = 0;

	signal(SIGINT, sigint_h);
	running = true;

	gettimeofday(&tv_start, NULL);
	while (running && (not_completed > 0)) {
		/* Enqueue encrypt */
		num = min(to_enc, MAX_BURST_SIZE);
		if (num && (enc_in_progress + num) < NUM_CONCURRENT_REQUESTS) {
			for (i = 0; i < num; i++) {
				enc_requests[i].cookie = &output_bufs[next_out_buf];
				enc_requests[i].dst = &output_bufs[next_out_buf];
				next_out_buf++;
				if (next_out_buf == NUM_BUFS)
					next_out_buf = 0;
			}
			num_done = num;
			rc = sam_cio_enq_ssltls(cio_enc, enc_requests, &num_done);
			if (rc != 0) {
				printf("Encrypt enqueue failed. num = %d, num_done = %d, rc = %d\n",
					num, num_done, rc);
				goto exit;
			}
			to_enc -= num_done;
			enc_in_progress += num_done;
			if (num > num_done) {
				pr_warn("Unexpected case for Enc enq: num %u > num_done %u\n",
					num, num_done);
			}
		}

		/* dequeue encrypt */
		num = min(enc_in_progress, MAX_BURST_SIZE);
		if (num && (dec_in_progress + num) < NUM_CONCURRENT_REQUESTS) {
			rc = sam_cio_deq(cio_enc, results, &num);
			if (rc) {
				printf("Encryption dequeue failed: enc_in_progress = %d, num = %d, rc = %d\n",
					dec_in_progress, num, rc);
				goto exit;
			}
			total_errors += check_results_after_encrypt(results,
					num_pkts - to_enc - enc_in_progress, num);
			enc_in_progress -= num;
		} else
			num = 0;

		if (!loopback) {
			not_completed -= num;
			continue;
		}
		/* For loopback - do decryption too */
		/* Enqueue decrypt */
		if (num) {
			for (i = 0; i < num; i++) {
				dec_requests[i].cookie = results[i].cookie;
				dec_requests[i].src = results[i].cookie;
				dec_requests[i].dst = results[i].cookie;
				dec_requests[i].pkt_size = results[i].out_len;
			}
			num_done = num;
			rc = sam_cio_enq_ssltls(cio_dec, dec_requests, &num_done);
			if (rc) {
				printf("Decrypt enqueue failed. num = %d, num_done = %d, rc = %d\n",
					num, num_done, rc);
				goto exit;
			}
			dec_in_progress += num_done;
			if (num > num_done) {
				pr_warn("Unexpected case for Dec enq: num %u > num_done %u\n",
					num, num_done);
			}
		}
		/* dequeue decrypt */
		num = min(dec_in_progress, MAX_BURST_SIZE);
		if (num) {
			rc = sam_cio_deq(cio_dec, results, &num);
			if (rc) {
				printf("Decryption dequeue failed: dec_in_progress = %d, num = %d, rc = %d\n",
					dec_in_progress, num, rc);
				goto exit;
			}

			total_errors += check_results_after_decrypt(results,
					num_pkts - not_completed, num);

			dec_in_progress -= num;
			not_completed -= num;
		}
	}
	gettimeofday(&tv_end, NULL);

	if (not_completed) {
		pr_warn("%u operations are not completed\n", not_completed);
		/*
		 * sam_cio_show_regs(cio_enc, SAM_CIO_REGS_ALL);
		 * if (cio_dec)
		 *	sam_cio_show_regs(cio_dec, SAM_CIO_REGS_ALL);
		 */
	}
	print_results(test_id, test_names[test_id], pkt_size,
			num_pkts - not_completed, total_errors, &tv_start, &tv_end);

	if (!total_errors)
		printf("\n%s: success\n\n", argv[0]);

exit:
	if (sa_enc) {
		if (sam_session_destroy(sa_enc))
			printf("Can't destroy sa_hndl session");
	}
	if (sam_cio_flush(cio_enc))
		printf("%s: sam_cio_flush failed for cio_enc\n", argv[0]);

	if (sa_dec) {
		if (sam_session_destroy(sa_dec))
			printf("Can't destroy sa_hndl_1 session");
	}
	if (cio_dec) {
		if (sam_cio_flush(cio_dec))
			printf("%s: sam_cio_flush failed for cio_dec\n", argv[0]);
	}
	if (input_buf.vaddr)
		mv_sys_dma_mem_free(input_buf.vaddr);

	for (i = 0; i < NUM_BUFS; i++) {
		if (output_bufs[i].vaddr)
			mv_sys_dma_mem_free(output_bufs[i].vaddr);
	}

	app_sam_show_cio_stats(cio_enc, cio_enc_match, 1);
	if (cio_dec && (cio_dec != cio_enc))
		app_sam_show_cio_stats(cio_dec, cio_dec_match, 1);
	app_sam_show_stats(1);

	if (cio_dec) {
		if (sam_cio_deinit(cio_dec)) {
			printf("%s: un-initialization failed\n", argv[0]);
			return 1;
		}
	}
	if (cio_enc) {
		if (sam_cio_deinit(cio_enc)) {
			printf("%s: un-initialization failed\n", argv[0]);
			return 1;
		}
	}
	sam_deinit();

	mv_sys_dma_mem_destroy();

	return 0;
}
