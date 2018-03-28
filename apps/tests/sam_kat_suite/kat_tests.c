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

#define _GNU_SOURCE         /* See feature_test_macros(7) */
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <sched.h>
#include <sys/time.h>

#include "mvapp.h"
#include "mv_std.h"
#include "lib/lib_misc.h"
#include "env/mv_sys_event.h"
#include "mv_sam.h"
#include "utils.h"
#include "sam_utils.h"
#include "fileSets.h"
#include "encryptedBlock.h"

#define SAM_DMA_MEM_SIZE		(10 * 1024 * 1204) /* 10 MBytes */

#define NUM_CONCURRENT_SESSIONS		1024
#define NUM_CONCURRENT_REQUESTS		127
#define MAX_BUFFER_SIZE			2048 /* bytes */
#define MAX_CIPHER_KEY_SIZE		32 /* 256 Bits = 32 Bytes */
#define MAX_CIPHER_BLOCK_SIZE		32 /* Bytes */
#define AUTH_BLOCK_SIZE_64B		64 /* Bytes */
#define MAX_AUTH_KEY_SIZE		1024 /* Bytes */
#define MAX_AUTH_ICV_SIZE		64 /* Bytes */
#define MAX_AAD_SIZE			64 /* Bytes */

static bool			same_bufs;
static int			num_to_check = 10;
static int			num_checked;
static int			num_to_print = 1;
static int			num_printed;
static int			test_burst_size = 32;
static u32			debug_flags;
static int			ev_pkts_coal = 16;
static int			ev_usec_coal = 100;

struct glob_arg {
	struct glb_common_args		cmn_args;  /* Keep first */
	char				sam_match_str[15];
	char				*sam_tests_file;
	EncryptedBlockPtr		test_db[NUM_CONCURRENT_SESSIONS];
	pthread_mutex_t			trd_lock;
	int				num_bufs;
	int                             num_devs;
	int				first_dev;
	int				first_cio;
	u32                             *free_cios; /* per device */
	bool				use_events;
	u64				total_passed;
	u64				total_errors;
};

struct local_arg {
	struct local_common_args	cmn_args;  /* Keep first */

	char				sam_match_str[15];
	struct sam_buf_info		in_buf;
	u32				in_data_size;
	struct sam_buf_info		out_bufs[NUM_CONCURRENT_REQUESTS];
	int				next_request;
	u8				expected_data[MAX_BUFFER_SIZE];
	u32				expected_data_size;
	struct mv_stack			*stack_hndl;
	struct sam_cio			*cio_hndl;
	struct sam_sa			*sa_hndl[NUM_CONCURRENT_SESSIONS];
	struct sam_session_params	sa_params[NUM_CONCURRENT_SESSIONS];
	u8				cipher_iv[MAX_CIPHER_BLOCK_SIZE];
	u8				auth_aad[MAX_AAD_SIZE];
	u8				auth_icv[MAX_AUTH_ICV_SIZE];
	u32				auth_icv_size;
	u64				total_passed;
	u64				total_errors;
	u32				almost_full;
};

static struct	glob_arg garg = {};

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

static int delete_sessions(struct local_arg *larg)
{
	int rc, i, count = 0;

	i = 0;
	while (i < NUM_CONCURRENT_SESSIONS) {
		if (larg->sa_hndl[i]) {
			pthread_mutex_lock(&larg->cmn_args.garg->trd_lock);
			if (sam_session_destroy(larg->sa_hndl[i])) {
				/* flush CIO instanse */
				rc = sam_cio_flush(larg->cio_hndl);
				if (rc)
					break;

				continue;
			}
			pthread_mutex_unlock(&larg->cmn_args.garg->trd_lock);
			count++;
		}
		i++;
	}
	pr_debug("thread #%d (cpu=%d): %d sessions deleted\n", larg->cmn_args.id, sched_getcpu(), count);

	rc = sam_cio_flush(larg->cio_hndl);
	if (rc)
		return rc;

	return 0;
}

static int create_sessions(struct local_arg *larg, EncryptedBlockPtr *tests_db)
{
	EncryptedBlockPtr block;
	int i, auth_key_len;
	u8 cipher_key[MAX_CIPHER_KEY_SIZE];

	for (i = 0; i < NUM_CONCURRENT_SESSIONS; i++) {

		block = tests_db[i];
		if (!block)
			break;

		larg->sa_params[i].dir = direction_str_to_val(encryptedBlockGetDirection(block));
		larg->sa_params[i].proto = SAM_PROTO_NONE;

		larg->sa_params[i].cipher_alg = cipher_algorithm_str_to_val(encryptedBlockGetAlgorithm(block));
		if (larg->sa_params[i].cipher_alg != SAM_CIPHER_NONE) {
			larg->sa_params[i].cipher_key_len = encryptedBlockGetKeyLen(block);
			if (larg->sa_params[i].cipher_key_len > sizeof(cipher_key)) {
				printf("thread #%d (cpu=%d): Cipher key size is too long: %d bytes > %d bytes\n",
					larg->cmn_args.id, sched_getcpu(),
					larg->sa_params[i].cipher_key_len, (int)sizeof(cipher_key));
				return -1;
			}
			encryptedBlockGetKey(block, larg->sa_params[i].cipher_key_len, cipher_key);

			larg->sa_params[i].cipher_mode = cipher_mode_str_to_val(encryptedBlockGetMode(block));
			larg->sa_params[i].cipher_iv = NULL;
			larg->sa_params[i].cipher_key = cipher_key;
			/* encryptedBlockGetIvLen(block, idx); - Check IV length with cipher algorithm */
		}

		auth_key_len = encryptedBlockGetAuthKeyLen(block);
		larg->sa_params[i].auth_alg = auth_algorithm_str_to_val(encryptedBlockGetAuthAlgorithm(block),
									auth_key_len);
		if (larg->sa_params[i].auth_alg != SAM_AUTH_NONE) {
			larg->sa_params[i].u.basic.auth_then_encrypt = 0;
			larg->sa_params[i].u.basic.auth_icv_len = encryptedBlockGetIcbLen(block, 0);

			if ((larg->sa_params[i].auth_alg == SAM_AUTH_AES_GCM) ||
			    (larg->sa_params[i].auth_alg == SAM_AUTH_AES_GMAC)) {
				/* cipher key used for authentication too */
				larg->sa_params[i].u.basic.auth_aad_len = encryptedBlockGetAadLen(block, 0);
			} else {
				if (auth_key_len > 0) {
					u8 auth_key[MAX_AUTH_KEY_SIZE];

					if (auth_key_len > MAX_AUTH_KEY_SIZE) {
						printf("thread #%d (cpu=%d): auth_key_len %d bytes is too big\n",
							larg->cmn_args.id, sched_getcpu(), auth_key_len);
						return -EINVAL;
					}
					if (encryptedBlockGetAuthKey(block, auth_key_len, auth_key) !=
										ENCRYPTEDBLOCK_SUCCESS) {
						printf("thread #%d (cpu=%d): Can't get auth_key of %d bytes\n",
							larg->cmn_args.id, sched_getcpu(), auth_key_len);
						return -EINVAL;
					}

					larg->sa_params[i].auth_key = auth_key;
					larg->sa_params[i].auth_key_len = auth_key_len;
				}
			}
		}
		if (sam_session_create(&larg->sa_params[i], &larg->sa_hndl[i])) {
			printf("thread #%d (cpu=%d): %s: failed\n", larg->cmn_args.id, sched_getcpu(), __func__);
			return -1;
		}
	}
	pr_debug("thread #%d (cpu=%d): %d sessions created\n",
		larg->cmn_args.id, sched_getcpu(), i);

	return 0;
}

static int check_results(struct local_arg *larg, struct sam_session_params *session_params,
			struct sam_cio_op_result *result, u16 num)
{
	int i, errors = 0;
	u8 *out_data;

	for (i = 0; i < num; i++) {
		/* cookie is pointer to output buffer */
		out_data = result->cookie;
		if (!out_data) {
			pr_err("thread #%d (cpu=%d): %s: Wrong cookie value: %p\n",
				larg->cmn_args.id, sched_getcpu(), __func__, out_data);
			return errors;
		}
		if (num_printed++ < num_to_print) {
			printf("\nthread #%d (cpu=%d): Input buffer: %d bytes\n",
				larg->cmn_args.id, sched_getcpu(), larg->in_data_size);
			mv_mem_dump(larg->in_buf.vaddr, larg->in_data_size);

			printf("\nthread #%d (cpu=%d): Output buffer: %d bytes\n",
				larg->cmn_args.id, sched_getcpu(), result->out_len);
			mv_mem_dump(out_data, result->out_len);

			printf("\nthread #%d (cpu=%d): Expected buffer: %d bytes\n",
				larg->cmn_args.id, sched_getcpu(), larg->expected_data_size);
			mv_mem_dump(larg->expected_data, larg->expected_data_size);

			if (larg->auth_icv_size) {
				printf("\nthread #%d (cpu=%d): ICV expected value: %d bytes\n",
					larg->cmn_args.id, sched_getcpu(), larg->auth_icv_size);
				mv_mem_dump(larg->auth_icv, larg->auth_icv_size);

				if (session_params->dir == SAM_DIR_DECRYPT)
					printf("\nthread #%d (cpu=%d): ICV verified by HW\n",
						larg->cmn_args.id, sched_getcpu());
			}
			printf("\n");
		}

		if (result->status != SAM_CIO_OK) {
			errors++;
			printf("thread #%d (cpu=%d): Error: result->status = %d\n",
				larg->cmn_args.id, sched_getcpu(), result->status);
		} else if (memcmp(out_data, larg->expected_data, result->out_len)) {
			/* Compare output and expected data (including ICV for encryption) */
			errors++;
			printf("thread #%d (cpu=%d): Error: out_data != expected_data\n",
				larg->cmn_args.id, sched_getcpu());
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

static void print_results(int test, char *test_name, int operations, int errors, u32 in_data_size,
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
		printf("cpu=%d: %2d. %-32s: passed %d times * %d Bytes        - %u.%03u secs\n",
			sched_getcpu(), test, test_name, operations, in_data_size, secs, usecs / 1000);
	else
		printf("cpu=%d: %2d. %-32s: failed %d of %d times * %d Bytes  - %u.%03u secs\n",
			sched_getcpu(), test, test_name, errors, operations, in_data_size, secs, usecs / 1000);

	printf("           %32s: %u Kpps, %u Mbps\n\n", "Rate", kpps, mbps);
}

static void free_bufs(struct local_arg *larg)
{
	int i;

	if (larg->in_buf.vaddr)
		mv_sys_dma_mem_free(larg->in_buf.vaddr);

	for (i = 0; i < NUM_CONCURRENT_REQUESTS; i++) {
		if (larg->out_bufs[i].vaddr)
			mv_sys_dma_mem_free(larg->out_bufs[i].vaddr);
	}
}


static int allocate_bufs(struct local_arg *larg, int buf_size)
{
	int i;

	larg->in_buf.vaddr = mv_sys_dma_mem_alloc(buf_size, 16);
	if (!larg->in_buf.vaddr) {
		pr_err("Can't allocate input DMA buffer of %d bytes\n", buf_size);
		return -ENOMEM;
	}
	larg->in_buf.paddr = mv_sys_dma_mem_virt2phys(larg->in_buf.vaddr);
	larg->in_buf.len = buf_size;

	for (i = 0; i < NUM_CONCURRENT_REQUESTS; i++) {
		larg->out_bufs[i].vaddr = mv_sys_dma_mem_alloc(buf_size, 16);
		if (!larg->out_bufs[i].vaddr) {
			pr_err("Can't allocate output %d DMA buffer of %d bytes\n", i, buf_size);
			return -ENOMEM;
		}
		larg->out_bufs[i].paddr = mv_sys_dma_mem_virt2phys(larg->out_bufs[i].vaddr);
		larg->out_bufs[i].len = buf_size;
	}
	return 0;
}

static void prepare_bufs(struct local_arg *larg, EncryptedBlockPtr block,
			 struct sam_session_params *session_params, int num)
{
	int i;

	if (num > NUM_CONCURRENT_REQUESTS)
		num = NUM_CONCURRENT_REQUESTS;

	if ((session_params->cipher_alg != SAM_CIPHER_NONE) && (session_params->cipher_mode != SAM_CIPHER_GMAC)) {
		/* plain text and cipher text must be valid */
		if (session_params->dir == SAM_DIR_ENCRYPT) {
			larg->expected_data_size = encryptedBlockGetCipherTextLen(block, 0);
			encryptedBlockGetCipherText(block, larg->expected_data_size, larg->expected_data, 0);

			larg->in_data_size = encryptedBlockGetPlainTextLen(block, 0);
			encryptedBlockGetPlainText(block, larg->in_data_size, larg->in_buf.vaddr, 0);

			if (same_bufs) {
				for (i = 0; i < num; i++)
					encryptedBlockGetPlainText(block, larg->in_data_size,
								   larg->out_bufs[i].vaddr, 0);
			}
		} else if (session_params->dir == SAM_DIR_DECRYPT) {
			larg->expected_data_size = encryptedBlockGetPlainTextLen(block, 0);
			encryptedBlockGetPlainText(block, larg->expected_data_size, larg->expected_data, 0);

			larg->in_data_size = encryptedBlockGetCipherTextLen(block, 0);
			encryptedBlockGetCipherText(block, larg->in_data_size, larg->in_buf.vaddr, 0);
			if (same_bufs) {
				for (i = 0; i < num; i++)
					encryptedBlockGetCipherText(block, larg->in_data_size,
								    larg->out_bufs[i].vaddr, 0);
			}
		}
	} else if (session_params->auth_alg != SAM_AUTH_NONE) {
		/* Authentication only */
		larg->in_data_size = encryptedBlockGetPlainTextLen(block, 0);
		encryptedBlockGetPlainText(block, larg->in_data_size, larg->in_buf.vaddr, 0);

		if (same_bufs) {
			for (i = 0; i < num; i++)
				encryptedBlockGetPlainText(block, larg->in_data_size,
							   larg->out_bufs[i].vaddr, 0);
		}
		/* Data must left the same */
		larg->expected_data_size = larg->in_data_size;
		encryptedBlockGetPlainText(block, larg->expected_data_size, larg->expected_data, 0);
	} else {
		/* Nothing to do */
		printf("thread #%d (cpu=%d): Warning: cipher_alg and auth_alg are NONE both\n",
			larg->cmn_args.id, sched_getcpu());
	}

	if (session_params->u.basic.auth_icv_len > 0) {
		larg->auth_icv_size = session_params->u.basic.auth_icv_len;
		encryptedBlockGetIcb(block, larg->auth_icv_size, larg->auth_icv, 0);
		if (session_params->dir == SAM_DIR_DECRYPT) {
			/* copy ICV to end of input buffer */
			if (same_bufs) {
				for (i = 0; i < num; i++)
					memcpy((larg->out_bufs[i].vaddr + larg->in_data_size),
						larg->auth_icv, larg->auth_icv_size);
			} else {
				memcpy((larg->in_buf.vaddr + larg->in_data_size), larg->auth_icv,
					larg->auth_icv_size);
			}
			larg->in_data_size += larg->auth_icv_size;
		} else {
			/* copy ICV to end of expected buffer */
			memcpy((larg->expected_data + larg->expected_data_size), larg->auth_icv,
				larg->auth_icv_size);
			larg->expected_data_size += larg->auth_icv_size;
		}
	} else
		larg->auth_icv_size = 0;
}

static void prepare_requests(struct local_arg *larg,
			     EncryptedBlockPtr block, struct sam_session_params *session_params,
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
				encryptedBlockGetIv(block, iv_len, larg->cipher_iv, 0);
				request->cipher_iv = larg->cipher_iv;
			}
		}
		if (session_params->auth_alg != SAM_AUTH_NONE) {
			request->auth_aad_offset = 0; /* not supported */
			aad_len = encryptedBlockGetAadLen(block, 0);
			if (aad_len) {
				encryptedBlockGetAad(block, aad_len, larg->auth_aad, 0);
				request->auth_aad = larg->auth_aad;
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
static int run_tests(void *arg, int *running)
{
	struct local_arg *larg = (struct local_arg *)arg;

	EncryptedBlockPtr block;
	int i, test, err = 0;
	int total_enqs, total_deqs, in_process, to_enq;
	u16 num;
	char *test_name;
	struct sam_cio_op_params requests[NUM_CONCURRENT_REQUESTS];
	struct sam_cio_op_result results[NUM_CONCURRENT_REQUESTS];
	int rc, count, errors;
	struct timeval tv_start, tv_end;
	struct mv_sys_event *ev = NULL;
	struct sam_cio_event_params ev_params;

	if (garg.use_events) {
		ev_params.pkt_coal = ev_pkts_coal;
		ev_params.usec_coal = ev_usec_coal;
		err = sam_cio_create_event(larg->cio_hndl, &ev_params, &ev);
		if (err) {
			printf("thread #%d (cpu=%d): Can't create CIO event\n",
				larg->cmn_args.id, sched_getcpu());
			return -EINVAL;
		}
		ev->events = MV_SYS_EVENT_POLLIN;
	}

	larg->total_passed = 0;
	larg->total_errors = 0;
	larg->almost_full = 0;

	for (test = 0; test < NUM_CONCURRENT_SESSIONS; test++) {
		if (!*running)
			break;

		block = larg->cmn_args.garg->test_db[test];
		if (!block)
			break;

		test_name = encryptedBlockGetName(block);

		count = total_enqs = total_deqs = encryptedBlockGetTestCounter(block);

		prepare_bufs(larg, block, &larg->sa_params[test], count);

		memset(requests, 0, sizeof(requests));
		memset(results, 0, sizeof(results));
		num_printed = 0;
		num_checked = 0;

		prepare_requests(larg, block, &larg->sa_params[test], larg->sa_hndl[test], requests,
				 test_burst_size);

		/* Check plain_len == cipher_len */
		errors = 0;
		in_process = 0;
		larg->next_request = 0;
		gettimeofday(&tv_start, NULL);
		while (total_deqs) {
			if (!*running)
				goto sig_exit;

			to_enq = min(total_enqs, test_burst_size);
			if (to_enq && (in_process + to_enq) < NUM_CONCURRENT_REQUESTS) {
				for (i = 0; i < to_enq; i++) {
					if (same_bufs) {
						/* Input buffers are different pre request */
						requests[i].src = &larg->out_bufs[larg->next_request];
						requests[i].dst = &larg->out_bufs[larg->next_request];
					} else {
						/* Output buffers are different per request */
						requests[i].src = &larg->in_buf;
						requests[i].dst = &larg->out_bufs[larg->next_request];
					}
					requests[i].cookie = requests[i].dst->vaddr;

					/* Increment next_request */
					larg->next_request++;
					if (larg->next_request == NUM_CONCURRENT_REQUESTS)
						larg->next_request = 0;
				}
				num = (u16)to_enq;
				rc = sam_cio_enq(larg->cio_hndl, requests, &num);
				if (rc) {
					printf("thread #%d (cpu=%d): %s: sam_cio_enq failed.",
						larg->cmn_args.id, sched_getcpu(), __func__);
					printf(" to_enq = %d, num = %d, rc = %d\n", to_enq, num, rc);
					return rc;
				}
				if (num) {
					in_process += num;
					total_enqs -= num;
				} else
					pr_err("Unexpected CIO full: to_enq = %d\n", to_enq);
			} else {
				if (to_enq)
					larg->almost_full++;
			}
			/* Trigger ISR */
			if (ev) {
				sam_cio_set_event(larg->cio_hndl, ev, 1);

				rc = mv_sys_event_poll(&ev, 1, -1);
				if ((rc != 1) || (ev->revents != MV_SYS_EVENT_POLLIN))
					pr_warn("Error during event poll: rc = %d, revents=0x%x\n",
						rc, ev->revents);

				/*pr_debug("mv_sys_event_poll returned - %d\n", rc);*/
			}

			/* Get all ready results together */
			num = (u16)in_process;
			rc = sam_cio_deq(larg->cio_hndl, results, &num);
			if (rc) {
				printf("thread #%d (cpu=%d): %s: sam_cio_deq failed. to_deq = %d, num = %d, rc = %d\n",
					larg->cmn_args.id, sched_getcpu(), __func__, in_process, num, rc);
				return rc;
			}
			if (num) {
				in_process -= num;
				total_deqs -= num;

				/* check result */
				if (num_checked < num_to_check) {
					num = min((num_to_check - num_checked), (int)num);
					errors += check_results(larg, &larg->sa_params[test], results, num);
					num_checked += num;
				}
			}
		}
sig_exit:
		gettimeofday(&tv_end, NULL);

		count -= total_deqs;

		pthread_mutex_lock(&larg->cmn_args.garg->trd_lock);
		print_results(test, test_name, count, errors, larg->in_data_size, &tv_start, &tv_end);
		pthread_mutex_unlock(&larg->cmn_args.garg->trd_lock);

		larg->total_errors += errors;
		larg->total_passed += (count - errors);
	}
	if (ev) {
		sam_cio_set_event(larg->cio_hndl, ev, 0);
		sam_cio_delete_event(larg->cio_hndl, ev);
	}

	/* return 1 to exit from main loop */
	return 1;
}

static void usage(char *progname)
{
	printf("Usage: %s <match> <test_file> [OPTIONS]\n", MVAPPS_NO_PATH(progname));
	printf("<match> string format is cio-0:0\n");
	printf("OPTIONS are optional:\n");
	printf("\t-c <number>      - Number of requests to check (default: %d)\n", num_to_check);
	printf("\t-p <number>      - Number of requests to print (default: %d)\n", num_to_print);
	printf("\t-b <number>      - Maximum burst size (default: %d)\n", test_burst_size);
	printf("\t-f <bitmask>     - Debug flags: 0x%x - SA, 0x%x - CIO. (default: 0x0)\n",
					SAM_SA_DEBUG_FLAG, SAM_CIO_DEBUG_FLAG);
	printf("\t--same_bufs      - Use the same buffer as src and dst (default: %s)\n",
		same_bufs ? "same" : "different");
	printf("\t--use-events     - Use events to wait for requests completed (default: polling)\n");
	printf("\t-pkts <number>   - Packets coalescing (default: %d)\n", ev_pkts_coal);
	printf("\t-time <number>   - Time coalescing in usecs (default: %d)\n", ev_usec_coal);
	printf("\t-cores <number>  - Number of CPUs to use (default %d)\n", garg.cmn_args.cpus);
}

static int parse_args(int argc, char *argv[])
{
	int	num, i = 3;

	/* First 2 arguments are mandatory */
	if (argc < 3) {
		usage(argv[0]);
		return -1;
	}

	garg.cmn_args.verbose = 0;
	garg.cmn_args.cli = 0;
	garg.cmn_args.cpus = 1;
	garg.cmn_args.affinity = MVAPPS_INVALID_AFFINITY;
	garg.cmn_args.echo = 1;
	strcpy(garg.sam_match_str, argv[1]);
	num = sscanf(garg.sam_match_str, "cio-%d:%d\n", &garg.first_dev, &garg.first_cio);
	if (num != 2) {
		pr_err("Invalid match string %s\n", garg.sam_match_str);
		return -1;
	}
	garg.sam_tests_file = argv[2];

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
		} else if (strcmp(argv[i], "-b") == 0) {
			if (argc < (i + 2)) {
				pr_err("Invalid number of arguments!\n");
				return -EINVAL;
			}
			if (argv[i + 1][0] == '-') {
				pr_err("Invalid arguments format!\n");
				return -EINVAL;
			}
			 test_burst_size = atoi(argv[i + 1]);
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
			garg.use_events = true;
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
		} else if (strcmp(argv[i], "-cores") == 0) {
			if (argc < (i + 2)) {
				pr_err("Invalid number of arguments!\n");
				return -EINVAL;
			}
			if (argv[i + 1][0] == '-') {
				pr_err("Invalid arguments format!\n");
				return -EINVAL;
			}
			garg.cmn_args.cpus = atoi(argv[i + 1]);
			i += 2;
		} else {
			pr_err("argument (%s) not supported!\n", argv[i]);
			return -EINVAL;
		}
	}
	/* Check validity */
	if (ev_pkts_coal > test_burst_size)
		ev_pkts_coal = test_burst_size;

	/* Print all inputs arguments */
	printf("CIO match_str  : %s\n", garg.sam_match_str);
	printf("Tests file name: %s\n", garg.sam_tests_file);
	printf("Number to check: %u\n", num_to_check);
	printf("Number to print: %u\n", num_to_print);
	printf("Burst size     : %u\n", test_burst_size);
	printf("Debug flags    : 0x%x\n", debug_flags);
	printf("src / dst bufs : %s\n", same_bufs ? "same" : "different");
	printf("Wait mode      : %s\n", garg.use_events ? "events" : "polling");
	if (garg.use_events) {
		printf("Pkts coalesing : %u pkts\n", ev_pkts_coal);
		printf("Time coalesing : %u usecs\n", ev_usec_coal);
	}
	return 0;
}

static int init_global(void *arg)
{
	struct glob_arg *garg = (struct glob_arg *)arg;
	struct sam_init_params init_params;
	int		err, dev;

	if (!garg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}
	pr_info("Global initializations ...\n");

	if (fileSetsReadBlocksFromFile(garg->sam_tests_file, garg->test_db,
				       NUM_CONCURRENT_SESSIONS) != FILE_SUCCESS) {
		printf("Can't read tests from file %s\n", garg->sam_tests_file);
		return -1;
	}

	if (pthread_mutex_init(&garg->trd_lock, NULL) != 0) {
		pr_err("init lock failed!\n");
		return -EIO;
	}
	garg->num_devs = sam_get_num_inst();
	garg->free_cios = calloc(garg->num_devs, sizeof(u32));
	if (!garg->free_cios)
		return -ENOMEM;

	for (dev = 0; dev < garg->num_devs; dev++)
		sam_get_available_cios(dev, &garg->free_cios[dev]);

	/* Check if first CIO is free */
	if ((garg->free_cios[garg->first_dev] & (1 << garg->first_cio)) == 0) {
		pr_err("cio-%d:%d is busy\n", garg->first_dev, garg->first_cio);
		return -EIO;
	}

	err = mv_sys_dma_mem_init(SAM_DMA_MEM_SIZE);
	if (err) {
		pr_err("Can't initialize %d KBytes of DMA memory area, err = %d\n", SAM_DMA_MEM_SIZE, err);
		return err;
	}

	init_params.max_num_sessions = NUM_CONCURRENT_SESSIONS;
	sam_init(&init_params);

	return 0;
}

static void deinit_global(void *arg)
{
	struct glob_arg *garg = (struct glob_arg *)arg;

	if (!garg)
		return;

	app_sam_show_stats(1);

	sam_deinit();

	if (garg->free_cios)
		free(garg->free_cios);

	printf("SAM tests passed:   %lu\n", garg->total_passed);
	printf("SAM tests failed:   %lu\n", garg->total_errors);
	printf("\n");
}

static int find_free_cio(struct glob_arg *garg, u8 device)
{
	int	i;
	u32	cios_map = garg->free_cios[device];

	if (cios_map == 0) {
		pr_err("no free CIO found!\n");
		return -ENOSPC;
	}

	/* we want to start from specified ring number */
	i = garg->first_cio;
	while (cios_map != 0) {
		if ((u32)(1 << i) & cios_map) {
			cios_map &= ~(u32)(1 << i);
			break;
		}
		i++;
		if (i == sam_get_num_cios(device))
			i = 0;
	}
	garg->free_cios[device] = cios_map;
	return i;
}

static int init_local(void *arg, int id, void **_larg)
{
	struct glob_arg		*garg = (struct glob_arg *)arg;
	struct local_arg	*larg;
	struct sam_cio_params	cio_params;
	int			err, cio_id, sam_device;

	pr_debug("Local initializations for thread %d\n", id);

	if (!garg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	larg = (struct local_arg *)malloc(sizeof(struct local_arg));
	if (!larg) {
		pr_err("No mem for local arg obj!\n");
		return -ENOMEM;
	}
	memset(larg, 0, sizeof(struct local_arg));

	pthread_mutex_lock(&garg->trd_lock);
	sam_device = (garg->first_dev + id) % garg->num_devs;
	cio_id = find_free_cio(garg, sam_device);
	if (cio_id < 0) {
		pr_err("free CIO not found!\n");
		pthread_mutex_unlock(&garg->trd_lock);
		return cio_id;
	}
	memset(&cio_params, 0, sizeof(cio_params));
	snprintf(larg->sam_match_str, sizeof(larg->sam_match_str), "cio-%d:%d", sam_device, cio_id);
	cio_params.match = larg->sam_match_str;
	cio_params.size = NUM_CONCURRENT_REQUESTS;
	err = sam_cio_init(&cio_params, &larg->cio_hndl);
	pthread_mutex_unlock(&garg->trd_lock);
	if (err != 0) {
		pr_err("CIO init failed!\n");
		return err;
	}
	sam_set_debug_flags(debug_flags);

	larg->cmn_args.id               = id;
	larg->cmn_args.verbose		= garg->cmn_args.verbose;
	larg->cmn_args.burst		= garg->cmn_args.burst;
	larg->cmn_args.echo             = garg->cmn_args.echo;
	larg->cmn_args.prefetch_shift   = garg->cmn_args.prefetch_shift;
	larg->cmn_args.garg		= garg;

	if (create_sessions(larg, garg->test_db))
		return -1;

	/* allocate in_buf and out_bufs */
	if (allocate_bufs(larg, MAX_BUFFER_SIZE))
		return -1;

	garg->cmn_args.largs[id] = larg;

	pr_info("Local thread #%d (cpu #%d): %s\n", id, sched_getcpu(), larg->sam_match_str);

	*_larg = larg;
	return 0;
}

static void deinit_local(void *arg)
{
	struct local_arg *larg = (struct local_arg *)arg;

	if (!larg)
		return;

	pthread_mutex_lock(&larg->cmn_args.garg->trd_lock);
	larg->cmn_args.garg->total_passed += larg->total_passed;
	larg->cmn_args.garg->total_errors += larg->total_errors;
	pthread_mutex_unlock(&larg->cmn_args.garg->trd_lock);

	delete_sessions(larg);

	free_bufs(larg);

	app_sam_show_cio_stats(larg->cio_hndl, larg->sam_match_str, 1);
	printf("Almost full                 : %" PRIu32 " packets\n\n", larg->almost_full);

	if (sam_cio_deinit(larg->cio_hndl)) {
		printf("thread #%d (cpu=%d): un-initialization failed\n",
			larg->cmn_args.id, sched_getcpu());
		return;
	}

	free(larg);
}

int main(int argc, char **argv)
{
	struct mvapp_params	mvapp_params;
	u64			cores_mask;
	int			rc;

	setbuf(stdout, NULL);
	pr_debug("pr_debug is enabled\n");

	rc = parse_args(argc, argv);
	if (rc)
		return rc;

	cores_mask = apps_cores_mask_create(garg.cmn_args.cpus, garg.cmn_args.affinity);
	garg.cmn_args.cores_mask = cores_mask;

	memset(&mvapp_params, 0, sizeof(mvapp_params));
	mvapp_params.num_cores		= garg.cmn_args.cpus;
	mvapp_params.cores_mask		= cores_mask;
	mvapp_params.global_arg		= (void *)&garg;
	mvapp_params.init_global_cb	= init_global;
	mvapp_params.deinit_global_cb	= deinit_global;
	mvapp_params.init_local_cb	= init_local;
	mvapp_params.deinit_local_cb	= deinit_local;
	mvapp_params.main_loop_cb	= run_tests;

	return mvapp_go(&mvapp_params);
}
