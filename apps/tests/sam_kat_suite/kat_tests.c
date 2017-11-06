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
#include <signal.h>
#include <sys/time.h>

#include "mv_std.h"
#include "lib/lib_misc.h"
#include "env/mv_sys_event.h"
#include "mv_sam.h"
#include "utils.h"
#include "sam_utils.h"
#include "fileSets.h"
#include "encryptedBlock.h"

#define SAM_DMA_MEM_SIZE		(1 * 1024 * 1204) /* 1 MBytes */

#define NUM_CONCURRENT_SESSIONS		1024
#define NUM_CONCURRENT_REQUESTS		127
#define MAX_BUFFER_SIZE			2048 /* bytes */
#define MAX_CIPHER_KEY_SIZE		32 /* 256 Bits = 32 Bytes */
#define MAX_CIPHER_BLOCK_SIZE		32 /* Bytes */
#define AUTH_BLOCK_SIZE_64B		64 /* Bytes */
#define MAX_AUTH_KEY_SIZE		1024 /* Bytes */
#define MAX_AUTH_ICV_SIZE		64 /* Bytes */
#define MAX_AAD_SIZE			64 /* Bytes */

static struct sam_cio		*cio_hndl;
static struct sam_sa		*sa_hndl[NUM_CONCURRENT_SESSIONS];
static struct sam_session_params sa_params[NUM_CONCURRENT_SESSIONS];

static char *sam_match_str;
static char *sam_tests_file;
static struct sam_buf_info	in_buf;
static u32			in_data_size;
static struct sam_buf_info	out_bufs[NUM_CONCURRENT_REQUESTS];
static int			next_request;
static u8			cipher_iv[MAX_CIPHER_BLOCK_SIZE];
static u8			auth_aad[MAX_AAD_SIZE];
static u8			auth_icv[MAX_AUTH_ICV_SIZE];
static u32			auth_icv_size;
static u8			expected_data[MAX_BUFFER_SIZE];
static u32			expected_data_size;
static bool			same_bufs;
static int			num_to_check = 10;
static int			num_checked;
static int			num_to_print = 1;
static int			num_printed;
static int			num_requests_per_enq = 32;
static int			num_requests_before_deq = 32;
static int			num_requests_per_deq = 32;
static u32			debug_flags;
static bool			use_events;
static int			ev_pkts_coal = 16;
static int			ev_usec_coal = 100;

static bool			running;
static generic_list		test_db;

static void sigint_h(int sig)
{
	printf("\nInterrupted by signal #%d\n", sig);
	running = false;
	signal(SIGINT, SIG_DFL);
}

static enum sam_dir direction_str_to_val(char *data)
{
	/* Direction must be valid */
	if (!data || (data[0] == '\0')) {
		printf("Direction is not defined\n");
		return SAM_DIR_LAST;
	}
	if (strcmp(data, "encryption") == 0)
		return SAM_DIR_ENCRYPT;

	if (strcmp(data, "decryption") == 0)
		return SAM_DIR_DECRYPT;

	printf("Syntax error in Direction: %s is unknown\n", data);
	return SAM_DIR_LAST;
}

static enum sam_cipher_alg cipher_algorithm_str_to_val(char *data)
{
	if (!data || (data[0] == '\0'))
		return SAM_CIPHER_NONE;

	if (strcmp(data, "DES") == 0)
		return SAM_CIPHER_DES;

	if (strcmp(data, "3DES") == 0)
		return SAM_CIPHER_3DES;

	if (strcmp(data, "AES") == 0)
		return SAM_CIPHER_AES;

	if (strcmp(data, "NULL") == 0)
		return SAM_CIPHER_NONE;

	printf("Syntax error in Algorithm: %s is unknown\n", data);
	return SAM_CIPHER_NONE;
}

static enum sam_cipher_mode cipher_mode_str_to_val(char *data)
{
	if (!data || (data[0] == '\0'))
		return SAM_CIPHER_MODE_LAST;

	if (strcmp(data, "ECB") == 0)
		return SAM_CIPHER_ECB;

	if (strcmp(data, "CBC") == 0)
		return SAM_CIPHER_CBC;

	if (strcmp(data, "CTR") == 0)
		return SAM_CIPHER_CTR;

	if (strcmp(data, "GCM") == 0)
		return SAM_CIPHER_GCM;

	if (strcmp(data, "GMAC") == 0)
		return SAM_CIPHER_GMAC;

	printf("Syntax error in Mode: %s is unknown\n", data);
	return SAM_CIPHER_MODE_LAST;
}

static enum sam_auth_alg auth_algorithm_str_to_val(char *data, int auth_key_len)
{
	if (!data || (data[0] == '\0'))
		return SAM_AUTH_NONE;

	if (strcmp(data, "NULL") == 0)
		return SAM_AUTH_NONE;

	if (strcmp(data, "AES_GCM") == 0)
		return SAM_AUTH_AES_GCM;

	if (strcmp(data, "AES_GMAC") == 0)
		return SAM_AUTH_AES_GMAC;

	if (auth_key_len > 0) {
		if (strcmp(data, "MD5") == 0)
			return SAM_AUTH_HMAC_MD5;

		if (strcmp(data, "SHA1") == 0)
			return SAM_AUTH_HMAC_SHA1;

		if (strcmp(data, "SHA224") == 0)
			return SAM_AUTH_HMAC_SHA2_224;

		if (strcmp(data, "SHA256") == 0)
			return SAM_AUTH_HMAC_SHA2_256;

		if (strcmp(data, "SHA384") == 0)
			return SAM_AUTH_HMAC_SHA2_384;

		if (strcmp(data, "SHA512") == 0)
			return SAM_AUTH_HMAC_SHA2_512;
	} else {
		if (strcmp(data, "MD5") == 0)
			return SAM_AUTH_HASH_MD5;

		if (strcmp(data, "SHA1") == 0)
			return SAM_AUTH_HASH_SHA1;

		if (strcmp(data, "SHA224") == 0)
			return SAM_AUTH_HASH_SHA2_224;

		if (strcmp(data, "SHA256") == 0)
			return SAM_AUTH_HASH_SHA2_256;

		if (strcmp(data, "SHA384") == 0)
			return SAM_AUTH_HASH_SHA2_384;

		if (strcmp(data, "SHA512") == 0)
			return SAM_AUTH_HASH_SHA2_512;
	}

	printf("Syntax error in Auth algorithm: %s is unknown\n", data);
	return SAM_AUTH_NONE;
}

static int delete_sessions(void)
{
	int rc, i, count = 0;

	i = 0;
	while (i < NUM_CONCURRENT_SESSIONS) {
		if (sa_hndl[i]) {
			if (sam_session_destroy(sa_hndl[i])) {
				/* flush CIO instanse */
				rc = sam_cio_flush(cio_hndl);
				if (rc)
					break;

				continue;
			}
			count++;
		}
		i++;
	}
	printf("%d sessions deleted\n", count);

	rc = sam_cio_flush(cio_hndl);
	if (rc)
		return rc;

	return 0;
}

static int create_sessions(generic_list tests_db)
{
	EncryptedBlockPtr block;
	int i, num_tests, auth_key_len;
	u8 cipher_key[MAX_CIPHER_KEY_SIZE];

	num_tests = generic_list_get_size(tests_db);
	if (num_tests > NUM_CONCURRENT_SESSIONS)
		num_tests = NUM_CONCURRENT_SESSIONS;

	block = generic_list_get_first(tests_db);
	for (i = 0; i < num_tests; i++) {
		if (i > 0)
			block = generic_list_get_next(tests_db);

		if (!block)
			return -1;

		sa_params[i].dir = direction_str_to_val(encryptedBlockGetDirection(block));
		sa_params[i].proto = SAM_PROTO_NONE;

		sa_params[i].cipher_alg = cipher_algorithm_str_to_val(encryptedBlockGetAlgorithm(block));
		if (sa_params[i].cipher_alg != SAM_CIPHER_NONE) {
			sa_params[i].cipher_key_len = encryptedBlockGetKeyLen(block);
			if (sa_params[i].cipher_key_len > sizeof(cipher_key)) {
				printf("Cipher key size is too long: %d bytes > %d bytes\n",
					sa_params[i].cipher_key_len, (int)sizeof(cipher_key));
				return -1;
			}
			encryptedBlockGetKey(block, sa_params[i].cipher_key_len, cipher_key);

			sa_params[i].cipher_mode = cipher_mode_str_to_val(encryptedBlockGetMode(block));
			sa_params[i].cipher_iv = NULL;
			sa_params[i].cipher_key = cipher_key;
			/* encryptedBlockGetIvLen(block, idx); - Check IV length with cipher algorithm */
		}

		auth_key_len = encryptedBlockGetAuthKeyLen(block);
		sa_params[i].auth_alg = auth_algorithm_str_to_val(encryptedBlockGetAuthAlgorithm(block), auth_key_len);
		if (sa_params[i].auth_alg != SAM_AUTH_NONE) {
			sa_params[i].u.basic.auth_then_encrypt = 0;
			sa_params[i].u.basic.auth_icv_len = encryptedBlockGetIcbLen(block, 0);

			if ((sa_params[i].auth_alg == SAM_AUTH_AES_GCM) ||
			    (sa_params[i].auth_alg == SAM_AUTH_AES_GMAC)) {
				/* cipher key used for authentication too */
				sa_params[i].u.basic.auth_aad_len = encryptedBlockGetAadLen(block, 0);
			} else {
				if (auth_key_len > 0) {
					u8 auth_key[MAX_AUTH_KEY_SIZE];

					if (auth_key_len > MAX_AUTH_KEY_SIZE) {
						printf("auth_key_len %d bytes is too big. Maximum is %d bytes\n",
							auth_key_len, MAX_AUTH_KEY_SIZE);
						return -EINVAL;
					}
					if (encryptedBlockGetAuthKey(block, auth_key_len, auth_key) !=
										ENCRYPTEDBLOCK_SUCCESS) {
						printf("Can't get authentication key of %d bytes\n", auth_key_len);
						return -EINVAL;
					}

					sa_params[i].auth_key = auth_key;
					sa_params[i].auth_key_len = auth_key_len;
				}
			}
		}
		if (sam_session_create(&sa_params[i], &sa_hndl[i])) {
			printf("%s: failed\n", __func__);
			return -1;
		}
	}
	printf("%d of %d sessions created successfully\n", i, num_tests);

	return 0;
}

static int check_results(struct sam_session_params *session_params,
			struct sam_cio_op_result *result, u16 num)
{
	int i, errors = 0;
	u8 *out_data;

	for (i = 0; i < num; i++) {
		/* cookie is pointer to output buffer */
		out_data = result->cookie;
		if (!out_data) {
			pr_err("%s: Wrong cookie value: %p\n",
				__func__, out_data);
			return errors;
		}
		if (num_printed++ < num_to_print) {
			printf("\nInput buffer: %d bytes\n", in_data_size);
			mv_mem_dump(in_buf.vaddr, in_data_size);

			printf("\nOutput buffer: %d bytes\n", result->out_len);
			mv_mem_dump(out_data, result->out_len);

			printf("\nExpected buffer: %d bytes\n", expected_data_size);
			mv_mem_dump(expected_data, expected_data_size);

			if (auth_icv_size) {
				printf("\nICV expected value: %d bytes\n", auth_icv_size);
				mv_mem_dump(auth_icv, auth_icv_size);

				if (session_params->dir == SAM_DIR_DECRYPT)
					printf("\nICV verified by HW\n");
			}
			printf("\n");
		}

		if (result->status != SAM_CIO_OK) {
			errors++;
			printf("Error: result->status = %d\n", result->status);
		} else if (memcmp(out_data, expected_data, result->out_len)) {
			/* Compare output and expected data (including ICV for encryption) */
			errors++;
			printf("Error: out_data != expected_data\n");
/*
			printf("\nOutput buffer: %d bytes\n", result->out_len);
			mv_mem_dump(out_data, result->out_len);

			printf("\nExpected buffer: %d bytes\n", expected_data_size);
			mv_mem_dump(expected_data, expected_data_size);
*/
		}
	}
	return errors;
}

static void print_results(int test, char *test_name, int operations, int errors,
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
	mbps = kpps * in_data_size * 8 / 1000;

	if (errors == 0)
		printf("%2d. FINISHED %-32s: passed %d times * %d Bytes        - %u.%03u secs\n",
			test, test_name, operations, in_data_size, secs, usecs / 1000);
	else
		printf("%2d. FINISHED %-32s: failed %d of %d times * %d Bytes  - %u.%03u secs\n",
			test, test_name, errors, operations, in_data_size, secs, usecs / 1000);

	printf("      Rate: %u Kpps, %u Mbps\n", kpps, mbps);
}

static void free_bufs(void)
{
	int i;

	if (in_buf.vaddr)
		mv_sys_dma_mem_free(in_buf.vaddr);

	for (i = 0; i < NUM_CONCURRENT_REQUESTS; i++) {
		if (out_bufs[i].vaddr)
			mv_sys_dma_mem_free(out_bufs[i].vaddr);
	}
}


static int allocate_bufs(int buf_size)
{
	int i;

	in_buf.vaddr = mv_sys_dma_mem_alloc(buf_size, 16);
	if (!in_buf.vaddr) {
		pr_err("Can't allocate input DMA buffer of %d bytes\n", buf_size);
		return -ENOMEM;
	}
	in_buf.paddr = mv_sys_dma_mem_virt2phys(in_buf.vaddr);
	in_buf.len = buf_size;

	for (i = 0; i < NUM_CONCURRENT_REQUESTS; i++) {
		out_bufs[i].vaddr = mv_sys_dma_mem_alloc(buf_size, 16);
		if (!out_bufs[i].vaddr) {
			pr_err("Can't allocate output %d DMA buffer of %d bytes\n", i, buf_size);
			return -ENOMEM;
		}
		out_bufs[i].paddr = mv_sys_dma_mem_virt2phys(out_bufs[i].vaddr);
		out_bufs[i].len = buf_size;
	}
	return 0;
}

static void prepare_bufs(EncryptedBlockPtr block, struct sam_session_params *session_params, int num)
{
	int i;

	if (num > NUM_CONCURRENT_REQUESTS)
		num = NUM_CONCURRENT_REQUESTS;

	if ((session_params->cipher_alg != SAM_CIPHER_NONE) && (session_params->cipher_mode != SAM_CIPHER_GMAC)) {
		/* plain text and cipher text must be valid */
		if (session_params->dir == SAM_DIR_ENCRYPT) {
			expected_data_size = encryptedBlockGetCipherTextLen(block, 0);
			encryptedBlockGetCipherText(block, expected_data_size, expected_data, 0);

			in_data_size = encryptedBlockGetPlainTextLen(block, 0);
			encryptedBlockGetPlainText(block, in_data_size, in_buf.vaddr, 0);

			if (same_bufs) {
				for (i = 0; i < num; i++)
					encryptedBlockGetPlainText(block, in_data_size, out_bufs[i].vaddr, 0);
			}
		} else if (session_params->dir == SAM_DIR_DECRYPT) {
			expected_data_size = encryptedBlockGetPlainTextLen(block, 0);
			encryptedBlockGetPlainText(block, expected_data_size, expected_data, 0);

			in_data_size = encryptedBlockGetCipherTextLen(block, 0);
			encryptedBlockGetCipherText(block, in_data_size, in_buf.vaddr, 0);
			if (same_bufs) {
				for (i = 0; i < num; i++)
					encryptedBlockGetCipherText(block, in_data_size, out_bufs[i].vaddr, 0);
			}
		}
	} else if (session_params->auth_alg != SAM_AUTH_NONE) {
		/* Authentication only */
		in_data_size = encryptedBlockGetPlainTextLen(block, 0);
		encryptedBlockGetPlainText(block, in_data_size, in_buf.vaddr, 0);

		if (same_bufs) {
			for (i = 0; i < num; i++)
				encryptedBlockGetPlainText(block, in_data_size, out_bufs[i].vaddr, 0);
		}
		/* Data must left the same */
		expected_data_size = in_data_size;
		encryptedBlockGetPlainText(block, expected_data_size, expected_data, 0);
	} else {
		/* Nothing to do */
		printf("Warning: cipher_alg and auth_alg are NONE both\n");
	}

	if (session_params->u.basic.auth_icv_len > 0) {
		auth_icv_size = session_params->u.basic.auth_icv_len;
		encryptedBlockGetIcb(block, auth_icv_size, auth_icv, 0);
		if (session_params->dir == SAM_DIR_DECRYPT) {
			/* copy ICV to end of input buffer */
			if (same_bufs) {
				for (i = 0; i < num; i++)
					memcpy((out_bufs[i].vaddr + in_data_size), auth_icv,
						auth_icv_size);
			} else {
				memcpy((in_buf.vaddr + in_data_size), auth_icv, auth_icv_size);
			}
			in_data_size += auth_icv_size;
		} else {
			/* copy ICV to end of expected buffer */
			memcpy((expected_data + expected_data_size), auth_icv, auth_icv_size);
			expected_data_size += auth_icv_size;
		}
	} else
		auth_icv_size = 0;
}

static void prepare_requests(EncryptedBlockPtr block, struct sam_session_params *session_params,
			     struct sam_sa *session_hndl, struct sam_cio_op_params *requests, int num)
{
	int i, iv_len, aad_len;
	struct sam_cio_op_params *request;

	for (i = 0; i < num; i++) {
		request = &requests[i];

		request->sa = session_hndl;
		request->num_bufs = 1;

		if (session_params->cipher_alg != SAM_CIPHER_NONE) {
			request->cipher_iv_offset = 0; /* not supported */
			request->cipher_offset = encryptedBlockGetCryptoOffset(block, 0);
			request->cipher_len = encryptedBlockGetPlainTextLen(block, 0) - request->cipher_offset;

			iv_len = encryptedBlockGetIvLen(block, 0);
			if (iv_len) {
				encryptedBlockGetIv(block, iv_len, cipher_iv, 0);
				request->cipher_iv = cipher_iv;
			}
		}
		if (session_params->auth_alg != SAM_AUTH_NONE) {
			request->auth_aad_offset = 0; /* not supported */
			aad_len = encryptedBlockGetAadLen(block, 0);
			if (aad_len) {
				encryptedBlockGetAad(block, aad_len, auth_aad, 0);
				request->auth_aad = auth_aad;
			}
			request->auth_offset = encryptedBlockGetAuthOffset(block, 0);
			request->auth_len = encryptedBlockGetPlainTextLen(block, 0) - request->auth_offset;
			request->auth_icv_offset = request->auth_offset + request->auth_len;
		}
		request++;
	}
};

/* There are few parameters can be configured:
 * - number of requests per enqueue/dequeue call: [1..NUM_CONCURRENT_REQUESTS] [default = 1]
 * - src/dst are different/same buffers [default: src != dst]
 */
static int run_tests(generic_list tests_db)
{
	EncryptedBlockPtr block;
	int i, test, num_tests, err = 0;
	int total_enqs, total_deqs, in_process, to_enq, num_enq, to_deq;
	u16 num;
	char *test_name;
	struct sam_cio_op_params requests[NUM_CONCURRENT_REQUESTS];
	struct sam_cio_op_result results[NUM_CONCURRENT_REQUESTS];
	int rc, count, total_passed, total_errors, errors;
	struct timeval tv_start, tv_end;
	struct mv_sys_event *ev = NULL;
	struct sam_cio_event_params ev_params;

	if (use_events) {
		ev_params.pkt_coal = ev_pkts_coal;
		ev_params.usec_coal = ev_usec_coal;
		err = sam_cio_create_event(cio_hndl, &ev_params, &ev);
		if (err) {
			printf("Can't create CIO event");
			return -EINVAL;
		}
		ev->events = MV_SYS_EVENT_POLLIN;
	}

	num_tests = generic_list_get_size(tests_db);
	if (num_tests > NUM_CONCURRENT_SESSIONS)
		num_tests = NUM_CONCURRENT_SESSIONS;

	block = generic_list_get_first(tests_db);
	total_passed = total_errors = 0;
	for (test = 0; test < num_tests; test++) {
		if (!running)
			break;

		if (test > 0)
			block = generic_list_get_next(tests_db);

		if (!block) {
			printf("Test %d of %d - invalid block\n", test, num_tests);
			return -1;
		}

		test_name = encryptedBlockGetName(block);

		count = total_enqs = total_deqs = encryptedBlockGetTestCounter(block);

		prepare_bufs(block, &sa_params[test], count);

		memset(requests, 0, sizeof(requests));
		memset(results, 0, sizeof(results));
		num_printed = 0;
		num_checked = 0;

		prepare_requests(block, &sa_params[test], sa_hndl[test], requests, num_requests_per_enq);

		/* Check plain_len == cipher_len */
		errors = 0;
		in_process = 0;
		next_request = 0;
		gettimeofday(&tv_start, NULL);
		while (total_deqs) {
			to_enq = min(total_enqs, num_requests_before_deq);
			while (in_process < to_enq) {
				if (!running)
					goto sig_exit;

				num_enq = min(num_requests_per_enq, (to_enq - in_process));
				for (i = 0; i < num_enq; i++) {
					if (same_bufs) {
						/* Input buffers are different pre request */
						requests[i].src = requests[i].dst = &out_bufs[next_request];
					} else {
						/* Output buffers are different per request */
						requests[i].src = &in_buf;
						requests[i].dst = &out_bufs[next_request];
					}
					requests[i].cookie = requests[i].dst->vaddr;

					/* Increment next_request */
					next_request++;
					if (next_request == NUM_CONCURRENT_REQUESTS)
						next_request = 0;
				}
				num = (u16)num_enq;
				rc = sam_cio_enq(cio_hndl, requests, &num);
				if (rc) {
					printf("%s: sam_cio_enq failed. to_enq = %d, num = %d, rc = %d\n",
						__func__, num_enq, num, rc);
					return rc;
				}
				if (num) {
					in_process += num;
					total_enqs -= num;
				} else
					break;
			}

			/* Trigger ISR */
			if (ev) {
				sam_cio_set_event(cio_hndl, ev, 1);

				rc = mv_sys_event_poll(ev, 1, -1);
				if ((rc != 1) || (ev->revents != MV_SYS_EVENT_POLLIN))
					pr_warn("Error during event poll: rc = %d, revents=0x%x\n",
						rc, ev->revents);

				/*pr_debug("mv_sys_event_poll returned - %d\n", rc);*/
			}

			/* Get all ready results together */
			to_deq = min(in_process, num_requests_per_deq);
			num = (u16)to_deq;
			rc = sam_cio_deq(cio_hndl, results, &num);
			if (rc) {
				printf("%s: sam_cio_deq failed. to_deq = %d, num = %d, rc = %d\n",
					__func__, to_deq, num, rc);
				return rc;
			}
			if (num) {
				in_process -= num;
				total_deqs -= num;

				/* check result */
				if (num_checked < num_to_check) {
					num = min((num_to_check - num_checked), (int)num);
					errors += check_results(&sa_params[test], results, num);
					num_checked += num;
				}
			}
		}
sig_exit:
		gettimeofday(&tv_end, NULL);

		count -= total_deqs;
		print_results(test, test_name, count, errors, &tv_start, &tv_end);

		total_errors += errors;
		total_passed += (count - errors);
	}
	if (ev) {
		sam_cio_set_event(cio_hndl, ev, 0);
		sam_cio_delete_event(cio_hndl, ev);
	}

	printf("\n");
	printf("SAM tests passed:   %d\n", total_passed);
	printf("SAM tests failed:   %d\n", total_errors);
	printf("\n");

	return err;
}

static void usage(char *progname)
{
	printf("Usage: %s <match> <test_file> [OPTIONS]\n", MVAPPS_NO_PATH(progname));
	printf("<match> string format is cio-0:0\n");
	printf("OPTIONS are optional:\n");
	printf("\t-c <number>      - Number of requests to check (default: %d)\n", num_to_check);
	printf("\t-p <number>      - Number of requests to print (default: %d)\n", num_to_print);
	printf("\t-e <number>      - Maximum burst for enqueue (default: %d)\n", num_requests_per_enq);
	printf("\t-d <number>      - Maximum burst for dequeue (default: %d)\n", num_requests_per_deq);
	printf("\t-f <bitmask>     - Debug flags: 0x%x - SA, 0x%x - CIO. (default: 0x0)\n",
					SAM_SA_DEBUG_FLAG, SAM_CIO_DEBUG_FLAG);
	printf("\t--same_bufs      - Use the same buffer as src and dst (default: %s)\n",
		same_bufs ? "same" : "different");
	printf("\t--use_events     - Use events to wait for requests completed (default: %s)\n",
		use_events ? "events" : "polling");
	printf("\t-pkts <number>   - Packets coalescing (default: %d)\n", ev_pkts_coal);
	printf("\t-time <number>   - Time coalescing in usecs (default: %d)\n", ev_usec_coal);
}

static int parse_args(int argc, char *argv[])
{
	int	i = 3;

	/* First 2 arguments are mandatory */
	if (argc < 3) {
		usage(argv[0]);
		return -1;
	}
	sam_match_str = argv[1];

	sam_tests_file = argv[2];

	while (i < argc) {
		if ((strcmp(argv[i], "?") == 0) ||
		    (strcmp(argv[i], "-h") == 0) ||
		    (strcmp(argv[i], "--help") == 0)) {
			usage(argv[0]);
			exit(0);
		}
		if (strcmp(argv[i], "-c") == 0) {
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
		} else if (strcmp(argv[i], "-p") == 0) {
			if (argc < (i + 2)) {
				pr_err("Invalid number of arguments!\n");
				return -EINVAL;
			}
			if (argv[i + 1][0] == '-') {
				pr_err("Invalid arguments format!\n");
				return -EINVAL;
			}
			num_to_print = atoi(argv[i + 1]);
			i += 2;
		} else if (strcmp(argv[i], "-e") == 0) {
			if (argc < (i + 2)) {
				pr_err("Invalid number of arguments!\n");
				return -EINVAL;
			}
			if (argv[i + 1][0] == '-') {
				pr_err("Invalid arguments format!\n");
				return -EINVAL;
			}
			 num_requests_per_enq = atoi(argv[i + 1]);
			i += 2;
		} else if (strcmp(argv[i], "-d") == 0) {
			if (argc < (i + 2)) {
				pr_err("Invalid number of arguments!\n");
				return -EINVAL;
			}
			if (argv[i + 1][0] == '-') {
				pr_err("Invalid arguments format!\n");
				return -EINVAL;
			}
			 num_requests_per_deq = atoi(argv[i + 1]);
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
		} else if (strcmp(argv[i], "--same_bufs") == 0) {
			same_bufs = true;
			i += 1;
		} else if (strcmp(argv[i], "--use-events") == 0) {
			use_events = true;
			i += 1;
		} else if (strcmp(argv[i], "-pkts") == 0) {
			if (argc < (i + 2)) {
				pr_err("Invalid number of arguments!\n");
				return -EINVAL;
			}
			if (argv[i + 1][0] == '-') {
				pr_err("Invalid arguments format!\n");
				return -EINVAL;
			}
			ev_pkts_coal = atoi(argv[i + 1]);
			i += 2;
		} else if (strcmp(argv[i], "-time") == 0) {
			if (argc < (i + 2)) {
				pr_err("Invalid number of arguments!\n");
				return -EINVAL;
			}
			if (argv[i + 1][0] == '-') {
				pr_err("Invalid arguments format!\n");
				return -EINVAL;
			}
			ev_usec_coal = atoi(argv[i + 1]);
			i += 2;
		} else {
			pr_err("argument (%s) not supported!\n", argv[i]);
			return -EINVAL;
		}
	}
	/* Check validity */
	if (ev_pkts_coal > num_requests_before_deq)
		ev_pkts_coal = num_requests_before_deq;

	/* Print all inputs arguments */
	printf("CIO match_str  : %s\n", sam_match_str);
	printf("Tests file name: %s\n", sam_tests_file);
	printf("Number to check: %u\n", num_to_check);
	printf("Number to print: %u\n", num_to_print);
	printf("Number per enq : %u\n", num_requests_per_enq);
	printf("Number per deq : %u\n", num_requests_per_deq);
	printf("Debug flags    : 0x%x\n", debug_flags);
	printf("src / dst bufs : %s\n", same_bufs ? "same" : "different");
	printf("Wait mode      : %s\n", use_events ? "events" : "polling");
	if (use_events) {
		printf("Pkts coalesing : %u pkts\n", ev_pkts_coal);
		printf("Time coalesing : %u usecs\n", ev_usec_coal);
	}
	return 0;
}

int main(int argc, char **argv)
{
	struct sam_init_params init_params;
	struct sam_cio_params cio_params;
	int rc;

	rc = parse_args(argc, argv);
	if (rc)
		return rc;

	rc = mv_sys_dma_mem_init(SAM_DMA_MEM_SIZE);
	if (rc) {
		pr_err("Can't initialize %d KBytes of DMA memory area, rc = %d\n", SAM_DMA_MEM_SIZE, rc);
		return rc;
	}

	test_db = generic_list_create(fileSetsEncryptedBlockCopyForList,
				      fileSetsEncryptedBlockDestroyForList);
	if (test_db == NULL) {
		printf("generic_list_create failed\n");
		return -1;
	}

	if (fileSetsReadBlocksFromFile(sam_tests_file, test_db) != FILE_SUCCESS) {
		printf("Can't read tests from file %s\n", sam_tests_file);
		return -1;
	}
	init_params.max_num_sessions = NUM_CONCURRENT_SESSIONS;
	sam_init(&init_params);

	cio_params.match = sam_match_str;
	cio_params.size = NUM_CONCURRENT_REQUESTS;

	if (sam_cio_init(&cio_params, &cio_hndl)) {
		printf("%s: initialization failed\n", argv[0]);
		return 1;
	}
	printf("%s successfully loaded\n", argv[0]);

	sam_set_debug_flags(debug_flags);

	if (create_sessions(test_db))
		goto exit;

	/* allocate in_buf and out_bufs */
	if (allocate_bufs(MAX_BUFFER_SIZE))
		goto exit;

	signal(SIGINT, sigint_h);
	running = true;
	if (run_tests(test_db))
		goto exit;

exit:
	delete_sessions();

	free_bufs();

	app_sam_show_cio_stats(cio_hndl, sam_match_str, 1);
	app_sam_show_stats(1);

	if (sam_cio_deinit(cio_hndl)) {
		printf("%s: un-initialization failed\n", argv[0]);
		return 1;
	}
	sam_deinit();

	printf("%s successfully unloaded\n", argv[0]);
	return 0;
}
