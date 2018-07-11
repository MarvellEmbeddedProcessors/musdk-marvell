/******************************************************************************
 *	Copyright (C) 2016 Marvell International Ltd.
 *
 *	If you received this File from Marvell, you may opt to use, redistribute
 *	and/or modify this File under the following licensing terms.
 *	Redistribution and use in source and binary forms, with or without
 *	modification, are permitted provided that the following conditions are met:
 *
 *		* Redistributions of source code must retain the above copyright notice,
 *		  this list of conditions and the following disclaimer.
 *
 *		* Redistributions in binary form must reproduce the above copyright
 *		  notice, this list of conditions and the following disclaimer in the
 *		  documentation and/or other materials provided with the distribution.
 *
 *		* Neither the name of Marvell nor the names of its contributors may be
 *		  used to endorse or promote products derived from this software without
 *		  specific prior written permission.
 *
 *	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *	AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *	IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *	ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *	LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *	CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *	SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *	INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *	CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *	ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *	POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#define _GNU_SOURCE         /* See feature_test_macros(7) */
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <pthread.h>
#include <sched.h>
#include <sys/time.h>
#include <netinet/ip.h>

#include "mv_std.h"
#include "lib/lib_misc.h"
#include "mv_stack.h"
#include "env/mv_sys_dma.h"
#include "env/io.h"
#include "env/mv_sys_event.h"

#include "mv_neta.h"
#include "mv_neta_ppio.h"
#include "mv_sam.h"

#include "../neta_utils.h"
#include "sam_utils.h"
#include "mvapp.h"
#include "perf_mon_emu.h"

#define CRYPT_APP_DEF_Q_SIZE		256
#define CRYPT_APP_RX_Q_SIZE		(2 * CRYPT_APP_DEF_Q_SIZE)
#define CRYPT_APP_TX_Q_SIZE		(2 * CRYPT_APP_RX_Q_SIZE)
#define CRYPT_APP_CIO_Q_SIZE		CRYPT_APP_DEF_Q_SIZE

#define CRYPT_APP_MAX_BURST_SIZE	(CRYPT_APP_RX_Q_SIZE >> 2)
#define CRYPT_APP_DFLT_BURST_SIZE	64
#define CRYPT_APP_CTRL_DFLT_THR		10000 /* 10 sec */
#define CRYPT_APP_DMA_MEM_SIZE		(48 * 1024 * 1024)

#define CRYPT_APP_CIOS_RSRV		0x0

#define MAX_AUTH_BLOCK_SIZE	128 /* Bytes */
#define AUTH_BLOCK_SIZE_64B	64  /* Bytes */
#define ICV_LEN			12  /* Bytes */

#define CRYPT_APP_FIRST_INQ			0
#define CRYPT_APP_MAX_NUM_TCS_PER_PORT		1
#define CRYPT_APP_MAX_NUM_QS_PER_CORE		CRYPT_APP_MAX_NUM_TCS_PER_PORT
#define CRYPT_APP_MAX_NUM_SESSIONS		32

/*#define CRYPT_APP_VERBOSE_CHECKS*/
#define CRYPT_APP_VERBOSE_DEBUG
#define CRYPT_APP_PKT_ECHO_SUPPORT
#define CRYPT_APP_PREFETCH_SHIFT	4

static u8 rfc3602_aes128_cbc_t1_key[] = {
	0x06, 0xa9, 0x21, 0x40, 0x36, 0xb8, 0xa1, 0x5b,
	0x51, 0x2e, 0x03, 0xd5, 0x34, 0x12, 0x00, 0x06
};

static u8 rfc3602_3des_cbc_t1_key[] = {
	0x06, 0xa9, 0x21, 0x40, 0x36, 0xb8, 0xa1, 0x5b,
	0x06, 0xa9, 0x21, 0x40, 0x36, 0xb8, 0xa1, 0x5b,
	0x51, 0x2e, 0x03, 0xd5, 0x34, 0x12, 0x00, 0x06
};

static u8 rfc3602_aes128_cbc_t1_iv[] = {
	0x3d, 0xaf, 0xba, 0x42, 0x9d, 0x9e, 0xb4, 0x30,
	0xb4, 0x22, 0xda, 0x80, 0x2c, 0x9f, 0xac, 0x41
};

static u8 rfc3602_sha1_t1_auth_key[] = {
	0x02, 0x81, 0xBC, 0xF7, 0xDC, 0xDE, 0xA1, 0xAD,
	0x3B, 0xBB, 0x77, 0xED, 0x1C, 0xAC, 0x20, 0x86,
	0xFF, 0x65, 0x15, 0x3D
};

/* Masks for mdata flags field */
#define MDATA_FLAGS_IP4_SEQID_MASK	BIT(0)   /* Set seqid field in IP header before send */

#define MDATA_FLAGS_IP4_CSUM_MASK	BIT(14)  /* Recalculate IPv4 checksum on TX */
#define MDATA_FLAGS_L4_CSUM_MASK	BIT(15)  /* Recalculate L4 checksum on TX */

#define CRYPT_APP_STATS_SUPPORT
#ifdef CRYPT_APP_STATS_SUPPORT
struct crypto_echo_stats {
	u64 rx_pkts[MVAPPS_NETA_MAX_NUM_PORTS];
	u64 rx_drop[MVAPPS_NETA_MAX_NUM_PORTS];
	u64 enc_enq;
	u64 enc_drop;
	u64 deq_cnt;
	u64 deq_err;
	u64 dec_enq;
	u64 dec_drop;
	u64 tx_pkts[MVAPPS_NETA_MAX_NUM_PORTS];
	u64 tx_drop[MVAPPS_NETA_MAX_NUM_PORTS];
	u64 tx_done[MVAPPS_NETA_MAX_NUM_PORTS];
};
#define CRYPT_STATS(c) c
#else
#define CRYPT_STATS(c)
#endif /* CRYPT_APP_STATS_SUPPORT */

struct pkt_mdata {
	u8 state; /* as defined in enum pkt_state */
	u8 rx_port;
	u8 rx_queue;
	u8 tx_port;
	u8 data_offs;
	u8 pad;
	u16 flags;
	void *buf_vaddr;
};

enum pkt_state {
	PKT_STATE_FREE = 0,
	PKT_STATE_RX,
	PKT_STATE_TX,
	PKT_STATE_ENC,
	PKT_STATE_DEC,
	PKT_STATE_LAST
};

enum crypto_dir {
	CRYPTO_ENC = 0,
	CRYPTO_DEC,
	CRYPTO_LB,
	CRYPTO_LAST
};

struct local_arg;

struct glob_arg {
	struct glb_common_args		cmn_args;  /* Keep first */
	enum crypto_dir			dir;
	enum sam_cipher_alg		cipher_alg;
	enum sam_cipher_mode		cipher_mode;
	enum sam_auth_alg		auth_alg;
	pthread_mutex_t			trd_lock;
	int				num_bufs;
	struct port_desc		ports_desc[MVAPPS_NETA_MAX_NUM_PORTS];
	int                             num_devs;
	u32                             *free_cios; /* per device */
	bool				use_events;
};

struct local_arg {
	struct local_common_args	cmn_args;  /* Keep first */

	struct mv_stack			*stack_hndl;
	char				enc_name[15];
	struct sam_cio			*enc_cio;
	struct sam_sa			*enc_sa;
	u32				seq_id[MVAPPS_NETA_MAX_NUM_PORTS];
	char				dec_name[15];
	struct sam_cio			*dec_cio;
#ifdef CRYPT_APP_STATS_SUPPORT
	struct crypto_echo_stats	stats;
#endif
	struct sam_sa			*dec_sa;
	enum crypto_dir			dir;
	struct lcl_port_desc		*lcl_ports_desc;

	u32				tx_in_cntr[MVAPPS_NETA_MAX_NUM_PORTS];
	u32				enc_in_cntr;
	u32				dec_in_cntr;

	struct mv_sys_event		*enc_ev;
	struct mv_sys_event		*dec_ev;
};

static struct glob_arg garg = {};
static int    ev_pkts_coal = 8;
static int    ev_usec_coal = 20;

#define CHECK_CYCLES
#ifdef CHECK_CYCLES
static int pme_ev_cnt_rx = -1, pme_ev_cnt_enq = -1, pme_ev_cnt_deq = -1, pme_ev_cnt_tx = -1;

#define START_COUNT_CYCLES(_ev_cnt)		pme_ev_cnt_start(_ev_cnt)
#define STOP_COUNT_CYCLES(_ev_cnt, _num)	pme_ev_cnt_stop(_ev_cnt, _num)

#else
#define START_COUNT_CYCLES(_ev_cnt)
#define STOP_COUNT_CYCLES(_ev_cnt, _num)
#endif /* CHECK_CYCLES */

#ifdef CRYPT_APP_STATS_SUPPORT
static void print_local_stats(struct local_arg *larg, int cpu, int reset)
{
	int i;

	printf("\n-------- Crypto-echo local CPU #%d statistics ------\n", cpu);
	for (i = 0; i < larg->cmn_args.num_ports; i++) {
		if (larg->stats.rx_pkts[i])
			printf("RX packets (port #%d)       : %" PRIu64 " packets\n", i, larg->stats.rx_pkts[i]);
		if (larg->stats.rx_drop[i])
			printf("RX drops (port #%d)         : %" PRIu64 " packets\n", i, larg->stats.rx_drop[i]);
	}
	if (larg->stats.enc_enq)
		printf("Encrypt enqueue            : %" PRIu64 " packets\n", larg->stats.enc_enq);
	if (larg->stats.enc_drop)
		printf("Encrypt drop               : %" PRIu64 " packets\n", larg->stats.enc_drop);
	if (larg->stats.dec_enq)
		printf("Decrypt enqueue            : %" PRIu64 " packets\n", larg->stats.dec_enq);
	if (larg->stats.dec_drop)
		printf("Decrypt drop               : %" PRIu64 " packets\n", larg->stats.dec_drop);
	if (larg->stats.deq_cnt)
		printf("Dequeue count              : %" PRIu64 " packets\n", larg->stats.deq_cnt);
	if (larg->stats.deq_err)
		printf("Dequeue errors             : %" PRIu64 " packets\n", larg->stats.deq_err);

	for (i = 0; i < larg->cmn_args.num_ports; i++) {
		if (larg->stats.tx_pkts[i])
			printf("TX packets (port #%d)       : %" PRIu64 " packets\n", i, larg->stats.tx_pkts[i]);
		if (larg->stats.tx_done[i])
			printf("TX done    (port #%d)       : %" PRIu64 " packets\n", i, larg->stats.tx_done[i]);
		if (larg->stats.tx_drop[i])
			printf("TX drops   (port #%d)       : %" PRIu64 " packets\n", i, larg->stats.tx_drop[i]);

	}
	printf("\n");

	if (reset)
		memset(&larg->stats, 0, sizeof(larg->stats));
}
#endif /* CRYPT_APP_STATS_SUPPORT */

static inline int free_buf_from_tx_shadow(struct local_arg *larg,
					  struct lcl_port_desc *tx_port,
					  u8 tc, u16 idx)
{
	struct neta_buff_inf	*binf;
	struct tx_shadow_q	*shadow_q;
	struct lcl_port_desc	*rx_port;
	u8 rp, rq;
	u16 bufs_num = 1;

	shadow_q = &tx_port->shadow_qs[tc];
	binf = &shadow_q->ents[idx].buff_ptr;
	rp = shadow_q->ents[idx].rx_port;
	rq = shadow_q->ents[idx].rx_queue;
	rx_port = &larg->lcl_ports_desc[rp];

	neta_ppio_inq_put_buffs(rx_port->ppio, rq, binf, &bufs_num);
	return 0;
}

static inline int free_sent_buffers(struct local_arg *larg, u8 tp, u8 tc)
{
	struct lcl_port_desc *tx_port = &larg->lcl_ports_desc[tp];
	u16 tx_num = 0;
	int i, idx;

	neta_ppio_get_num_outq_done(tx_port->ppio, tc, &tx_num);

	if (unlikely(!tx_num))
		return 0;

	idx = tx_port->shadow_qs[tc].read_ind;
	for (i = 0; i < tx_num; i++) {
		free_buf_from_tx_shadow(larg, tx_port, tc, idx);
		idx++;
		if (idx == tx_port->shadow_q_size)
			idx = 0;
	}
	tx_port->shadow_qs[tc].read_ind = idx;
	larg->tx_in_cntr[tp] -= tx_num;

#ifdef CRYPT_APP_VERBOSE_DEBUG
	if (larg->cmn_args.verbose && tx_num)
		printf("tx_done %d buffs on port #%d, tc %d\n", tx_num, tp, tc);
#endif /* CRYPT_APP_VERBOSE_DEBUG */

	return tx_num;
}

static void free_buf_from_sam_cookie(struct local_arg *larg, void *cookie)
{
	struct pkt_mdata	*mdata = (struct pkt_mdata *)cookie;
	struct lcl_port_desc	*rx_port;
	char			*buff = mdata->buf_vaddr;
	dma_addr_t		pa;
	u16			bufs_num;
	u8			rp;
	struct neta_buff_inf	binf;

	pa = mv_sys_dma_mem_virt2phys(buff);
	binf.addr = pa;
	binf.cookie = lower_32_bits((uintptr_t)(buff));
	rp = mdata->rx_port;
	rx_port = &larg->lcl_ports_desc[rp];

	bufs_num = 1;
	neta_ppio_inq_put_buffs(rx_port->ppio, mdata->rx_queue, &binf, &bufs_num);
	mv_stack_push(larg->stack_hndl, mdata);
}

#ifdef CRYPT_APP_PKT_ECHO_SUPPORT
static inline void echo_pkts(struct local_arg		*larg,
			     struct sam_cio_op_result	*sam_res_descs,
			     u16			 num)
{
	char	*tmp_buff;
	int	prefetch_shift = larg->cmn_args.prefetch_shift;
	u16	i;
	struct pkt_mdata *mdata;

	for (i = 0; i < num; i++) {
		if (num - i > prefetch_shift) {
			mdata = sam_res_descs[i + prefetch_shift].cookie;
			tmp_buff = mdata->buf_vaddr;
			prefetch(tmp_buff + MVAPPS_NETA_PKT_EFEC_OFFS);
		}
		/* pointer to MAC header */
		mdata = sam_res_descs[i].cookie;
		tmp_buff = (char *)mdata->buf_vaddr + MVAPPS_NETA_PKT_EFEC_OFFS;

#ifdef CRYPT_APP_VERBOSE_DEBUG
		if (larg->cmn_args.verbose > 1) {
			printf("pkt before echo (len %d):\n",
			       sam_res_descs[i].out_len);
			mv_mem_dump((u8 *)tmp_buff, sam_res_descs[i].out_len);
		}
#endif /* CRYPT_APP_VERBOSE_DEBUG */
		swap_l2(tmp_buff);
		swap_l3(tmp_buff);
		/*printf("pkt after echo:\n");*/
		/*mem_disp(tmp_buff, sam_res_descs[i].out_len  - MVAPPS_NETA_PKT_EFEC_OFFS);*/
	}
}
#endif /* CRYPT_APP_PKT_ECHO_SUPPORT */

static inline int proc_rx_pkts(struct local_arg *larg,
			   u8 rx_ppio_id, u8 tx_ppio_id, u8 rx_q,
			   struct neta_ppio_desc *descs, u16 num)
{
	struct sam_cio_op_params sam_descs[CRYPT_APP_MAX_BURST_SIZE];
	struct sam_buf_info	 src_buf_infs[CRYPT_APP_MAX_BURST_SIZE];
	struct sam_buf_info	 dst_buf_infs[CRYPT_APP_MAX_BURST_SIZE];
	struct pkt_mdata	*mdata;
	struct sam_sa		*sa;
	struct sam_cio		*cio;
	enum pkt_state		state;
	int			err;
	u16			i, buff_len, num_got;
	u32			block_size, pad_size;
	struct perf_cmn_cntrs	*perf_cntrs = &larg->cmn_args.perf_cntrs;

#ifdef CRYPT_APP_VERBOSE_DEBUG
	if (larg->cmn_args.verbose)
		pr_info("%s: %d packets received. %d -> %d\n", __func__, num, rx_ppio_id, tx_ppio_id);
#endif /* CRYPT_APP_VERBOSE_DEBUG */

	/* TODO: Get current buffer size */
	buff_len = larg->cmn_args.garg->cmn_args.mtu + 64;

	memset(sam_descs, 0, sizeof(sam_descs));

	if ((larg->dir == CRYPTO_ENC) || (larg->dir == CRYPTO_LB)) {
		cio = larg->enc_cio;
		sa = larg->enc_sa;
		state = PKT_STATE_ENC;
	} else {
		cio = larg->dec_cio;
		sa = larg->dec_sa;
		state = PKT_STATE_DEC;
	}

	for (i = 0; i < num; i++) {
		char *vaddr;

		vaddr = (char *)((uintptr_t)neta_ppio_inq_desc_get_cookie(&descs[i]) | neta_sys_dma_high_addr);

		mdata = (struct pkt_mdata *)mv_stack_pop(larg->stack_hndl);
		if (!mdata) {
			printf("Can't get mdata from stack for packet %d of %d\n", i, num);
			break;
		}

		mdata->state = state;
		mdata->rx_port = rx_ppio_id;
		mdata->rx_queue = rx_q;
		mdata->tx_port = tx_ppio_id;
		mdata->buf_vaddr = vaddr;
		mdata->flags = 0;
		mdata->pad = 0;

		/* Set vaddr and paddr to MAC address of the packet */
		vaddr += MVAPPS_NETA_PKT_EFEC_OFFS;
		src_buf_infs[i].vaddr = (char *)((uintptr_t)vaddr);

		src_buf_infs[i].paddr = neta_ppio_inq_desc_get_phys_addr(&descs[i]) + MVAPPS_NETA_PKT_EFEC_OFFS;

		/* Exclude MH from packet length */
		src_buf_infs[i].len = neta_ppio_inq_desc_get_pkt_len(&descs[i]);

#ifdef CRYPT_APP_VERBOSE_DEBUG
		if (larg->cmn_args.verbose > 1) {
			printf("Received packet (va:%p, pa 0x%08" PRIdma ", len %d):\n",
			       src_buf_infs[i].vaddr,
			       src_buf_infs[i].paddr,
			       src_buf_infs[i].len);
			mv_mem_dump(src_buf_infs[i].vaddr, src_buf_infs[i].len);
		}
#endif /* CRYPT_APP_VERBOSE_DEBUG */

		dst_buf_infs[i].vaddr = src_buf_infs[i].vaddr;
		dst_buf_infs[i].paddr = src_buf_infs[i].paddr;
		dst_buf_infs[i].len = buff_len;

		/* data_offs - L2 offset */
		mdata->data_offs = ETH_HLEN;

		sam_descs[i].sa = sa;
		sam_descs[i].cookie = mdata;

		sam_descs[i].num_bufs = 1;
		sam_descs[i].src = &src_buf_infs[i];
		sam_descs[i].dst = &dst_buf_infs[i];
		sam_descs[i].cipher_iv = rfc3602_aes128_cbc_t1_iv;

		sam_descs[i].cipher_offset = mdata->data_offs;
		sam_descs[i].cipher_len = src_buf_infs[i].len - sam_descs[i].cipher_offset;
		/* cipher_len must be block size aligned. Block size is always power of 2 */
		block_size = sam_session_get_block_size(larg->cmn_args.garg->cipher_alg);
		if (block_size && (sam_descs[i].cipher_len & (block_size - 1))) {
			pad_size = block_size - (sam_descs[i].cipher_len & (block_size - 1));
			/* clear padding data */
			memset(src_buf_infs[i].vaddr + sam_descs[i].cipher_offset + sam_descs[i].cipher_len,
			       0, pad_size);
			sam_descs[i].cipher_len += pad_size;
			mdata->pad = pad_size;
#ifdef CRYPT_APP_VERBOSE_DEBUG
			if (larg->cmn_args.verbose > 1)
				pr_info("%s: cipher_len after padding = %d bytes, pad_size = %d bytes\n",
					__func__, sam_descs[i].cipher_len, pad_size);
#endif
		}
		if (larg->cmn_args.garg->auth_alg != SAM_AUTH_NONE) {
			if (larg->dir == CRYPTO_DEC)
				sam_descs[i].cipher_len -= ICV_LEN;
			sam_descs[i].auth_len = sam_descs[i].cipher_len;
			sam_descs[i].auth_offset = sam_descs[i].cipher_offset;
			sam_descs[i].auth_icv_offset = sam_descs[i].auth_offset + sam_descs[i].auth_len;
		}
	}
	num_got = i;

START_COUNT_CYCLES(pme_ev_cnt_enq);
	err = sam_cio_enq(cio, sam_descs, &num_got);
STOP_COUNT_CYCLES(pme_ev_cnt_enq, num_got);

	if (cio == larg->enc_cio)
		larg->enc_in_cntr += num_got;
	else
		larg->dec_in_cntr += num_got;

	if (state == PKT_STATE_ENC)
		perf_cntrs->enc_cnt += num_got;
	else
		perf_cntrs->dec_cnt += num_got;

	if (unlikely(err)) {
		pr_err("SAM enqueue failed (%d)!\n", err);
		return -EFAULT;
	}
	if (num_got < num) {
		for (i = num_got; i < num; i++)
			free_buf_from_sam_cookie(larg, sam_descs[i].cookie);

		/*pr_warn("%s rx_ppio=%d: %d packets dropped\n", __func__, rx_ppio_id, num - num_got);*/
		perf_cntrs->drop_cnt += (num - num_got);
	}
	return 0;
}

static inline int dec_pkts(struct local_arg		*larg,
			   struct sam_cio_op_result	*sam_res_descs,
			   u16				 num)
{
	struct sam_cio_op_params sam_descs[CRYPT_APP_MAX_BURST_SIZE];
	struct sam_buf_info	 src_buf_infs[CRYPT_APP_MAX_BURST_SIZE];
	struct sam_buf_info	 dst_buf_infs[CRYPT_APP_MAX_BURST_SIZE];
	struct pkt_mdata	*mdata;
	int			 err;
	u16			 i, buff_len, num_got;
	u8			 data_offs;
	struct perf_cmn_cntrs	*perf_cntrs = &larg->cmn_args.perf_cntrs;

	/* TODO: is this enough?!?!?! */
	buff_len = larg->cmn_args.garg->cmn_args.mtu + 64;

	memset(sam_descs, 0, sizeof(sam_descs));

	/* For decryption crypto parameters are caclulated from cookie and out_len */
	for (i = 0; i < num; i++) {
		mdata = sam_res_descs[i].cookie;
		data_offs = mdata->data_offs;

		src_buf_infs[i].vaddr = (char *)mdata->buf_vaddr + MVAPPS_NETA_PKT_EFEC_OFFS;
		src_buf_infs[i].paddr = mv_sys_dma_mem_virt2phys(src_buf_infs[i].vaddr);
		src_buf_infs[i].len = sam_res_descs[i].out_len;

		dst_buf_infs[i].vaddr = src_buf_infs[i].vaddr;
		dst_buf_infs[i].paddr = src_buf_infs[i].paddr;
		dst_buf_infs[i].len = buff_len;

		sam_descs[i].cookie = mdata;
		sam_descs[i].sa = larg->dec_sa;
		sam_descs[i].num_bufs = 1;
		sam_descs[i].src = &src_buf_infs[i];
		sam_descs[i].dst = &dst_buf_infs[i];
		sam_descs[i].cipher_iv = rfc3602_aes128_cbc_t1_iv;
		sam_descs[i].cipher_offset = data_offs;
		sam_descs[i].cipher_len = src_buf_infs[i].len - sam_descs[i].cipher_offset;
		if (larg->cmn_args.garg->auth_alg != SAM_AUTH_NONE) {
			sam_descs[i].cipher_len -= ICV_LEN;
			sam_descs[i].auth_len = sam_descs[i].cipher_len;
			sam_descs[i].auth_offset = sam_descs[i].cipher_offset;
			sam_descs[i].auth_icv_offset = sam_descs[i].auth_offset + sam_descs[i].auth_len;
		}

#ifdef CRYPT_APP_VERBOSE_DEBUG
		if (larg->cmn_args.verbose > 1) {
			printf("Encrypted packet (va:%p, pa 0x%08" PRIdma ", len %d):\n",
			       src_buf_infs[i].vaddr,
			       src_buf_infs[i].paddr,
			       src_buf_infs[i].len);
			mv_mem_dump(src_buf_infs[i].vaddr, src_buf_infs[i].len);
		}
#endif /* CRYPT_APP_VERBOSE_DEBUG */
	}
	num_got = num;

START_COUNT_CYCLES(pme_ev_cnt_enq);
	err = sam_cio_enq(larg->dec_cio, sam_descs, &num_got);
STOP_COUNT_CYCLES(pme_ev_cnt_enq, num_got);

	if (unlikely(err)) {
		pr_err("SAM EnQ (DeC) failed (%d)!\n", err);
		return -EFAULT;
	}
	if (larg->dec_cio == larg->enc_cio)
		larg->enc_in_cntr += num_got;
	else
		larg->dec_in_cntr += num_got;

	perf_cntrs->dec_cnt += num_got;
	if (num_got < num) {
		for (i = num_got; i < num; i++)
			free_buf_from_sam_cookie(larg, sam_descs[i].cookie);

		/*pr_warn("%s: %d packets dropped\n", __func__, num - num_got);*/
		perf_cntrs->drop_cnt += (num - num_got);
	}
	return 0;
}

static inline int send_pkts(struct local_arg *larg,
			    struct sam_cio_op_result *sam_res_descs, u16 num)
{
	struct tx_shadow_q	*shadow_q;
	struct neta_ppio_desc	*desc;
	struct neta_ppio_desc	 descs[MVAPPS_NETA_MAX_NUM_PORTS][CRYPT_APP_MAX_BURST_SIZE];
	int			 err, tc = 0, shadow_q_size[MVAPPS_NETA_MAX_NUM_PORTS];
	u16			 i, tp, num_got, port_nums[MVAPPS_NETA_MAX_NUM_PORTS];
	struct perf_cmn_cntrs	*perf_cntrs = &larg->cmn_args.perf_cntrs;

	for (tp = 0; tp < larg->cmn_args.num_ports; tp++) {
		port_nums[tp] = 0;
		shadow_q_size[tp] = larg->lcl_ports_desc[tp].shadow_q_size;
	}

	for (i = 0; i < num; i++) {
		char			*buff;
		dma_addr_t		pa;
		u16			len = sam_res_descs[i].out_len;
		struct pkt_mdata	*mdata;

		mdata = (struct pkt_mdata *)(uintptr_t)sam_res_descs[i].cookie;
		buff = mdata->buf_vaddr;
		pa = mv_sys_dma_mem_virt2phys(buff);

		tp = mdata->tx_port;
		desc = &descs[tp][port_nums[tp]];
		shadow_q = &larg->lcl_ports_desc[tp].shadow_qs[tc];

		if (mdata->flags & MDATA_FLAGS_IP4_SEQID_MASK) {
			/* Set SeqID to IPv4 header of the packet */
			struct iphdr *iph = (struct iphdr *)
					((char *)mdata->buf_vaddr + MVAPPS_NETA_PKT_EFEC_OFFS + mdata->data_offs);

			iph->id = htobe16(larg->seq_id[tp]++);
		}

		neta_ppio_outq_desc_reset(desc);
		neta_ppio_outq_desc_set_phys_addr(desc, pa);
		neta_ppio_outq_desc_set_pkt_offset(desc, MVAPPS_NETA_PKT_EFEC_OFFS);
		neta_ppio_outq_desc_set_pkt_len(desc, len);

		if (mdata->flags & MDATA_FLAGS_IP4_CSUM_MASK) {
			/* Recalculate IPv4 checksum */
			neta_ppio_outq_desc_set_proto_info(desc, NETA_OUTQ_L3_TYPE_IPV4,
						  NETA_OUTQ_L4_TYPE_OTHER,
						  mdata->data_offs,
						  mdata->data_offs + sizeof(struct iphdr), 1, 0);
		}
#ifdef CRYPT_APP_VERBOSE_DEBUG
		if (larg->cmn_args.verbose > 1) {
			printf("Sending packet (va:%p, pa 0x%08x, len %d):\n",
			       buff, (unsigned int)pa, len);
			mv_mem_dump((u8 *)buff + MVAPPS_NETA_PKT_EFEC_OFFS, len);
		}
#endif /* CRYPT_APP_VERBOSE_DEBUG */

		shadow_q->ents[shadow_q->write_ind].buff_ptr.cookie = (uintptr_t)buff;
		shadow_q->ents[shadow_q->write_ind].buff_ptr.addr = pa;
		shadow_q->ents[shadow_q->write_ind].rx_port = mdata->rx_port;
		shadow_q->ents[shadow_q->write_ind].rx_queue = mdata->rx_queue;
		shadow_q->write_ind++;
		if (shadow_q->write_ind == shadow_q_size[tp])
			shadow_q->write_ind = 0;
		port_nums[tp]++;

		/* We don't need metadata more */
		mv_stack_push(larg->stack_hndl, mdata);
	}

	for (tp = 0; tp < larg->cmn_args.num_ports; tp++) {
		if (port_nums[tp] == 0)
			continue;

		num = num_got = port_nums[tp];
		shadow_q = &larg->lcl_ports_desc[tp].shadow_qs[tc];

START_COUNT_CYCLES(pme_ev_cnt_tx);
		err = neta_ppio_send(larg->lcl_ports_desc[tp].ppio, tc,
					    descs[tp], &num_got);
STOP_COUNT_CYCLES(pme_ev_cnt_tx, num_got);
		if (err)
			return err;

		if (num_got) {
			perf_cntrs->tx_cnt += num_got;
			larg->tx_in_cntr[tp] += num_got;
		}
#ifdef CRYPT_APP_VERBOSE_DEBUG
		if (larg->cmn_args.verbose) {
			printf("thread #%d (cpu=%d): sent %d of %d pkts on tx_port=%d, tc=%d\n",
				larg->cmn_args.id, sched_getcpu(),
				num_got, num, tp, tc);
		}
#endif /* CRYPT_APP_VERBOSE_DEBUG */

		if (num_got < num) {
			for (i = 0; i < num - num_got; i++) {
				if (shadow_q->write_ind == 0)
					shadow_q->write_ind = shadow_q_size[tp];
				shadow_q->write_ind--;
				free_buf_from_tx_shadow(larg, &larg->lcl_ports_desc[tp],
							tc, shadow_q->write_ind);
			}
			/*pr_warn("%s tc_port=%d: %d packets dropped\n", __func__, tp, num - num_got);*/
			perf_cntrs->drop_cnt += (num - num_got);
		}
	}
	return 0;
}

static inline int deq_crypto_pkts(struct local_arg	*larg,
				  struct sam_cio	*cio)
{
	struct sam_cio_op_result res_descs[CRYPT_APP_MAX_BURST_SIZE];
	struct sam_cio_op_result res_descs_to_dec[CRYPT_APP_MAX_BURST_SIZE];
	struct sam_cio_op_result res_descs_to_send[CRYPT_APP_MAX_BURST_SIZE];
	int			 err;
	u16			 i, num, num_to_send, num_to_dec;
	struct perf_cmn_cntrs	*perf_cntrs = &larg->cmn_args.perf_cntrs;

	num = CRYPT_APP_MAX_BURST_SIZE;
START_COUNT_CYCLES(pme_ev_cnt_deq);
	err = sam_cio_deq(cio, res_descs, &num);
STOP_COUNT_CYCLES(pme_ev_cnt_deq, num);
	if (unlikely(err)) {
		pr_err("SAM dequeue failed (%d)!\n", err);
		return -EFAULT;
	}
	if (cio == larg->enc_cio)
		larg->enc_in_cntr -= num;
	else
		larg->dec_in_cntr -= num;

	num_to_send = num_to_dec = 0;
	for (i = 0; i < num; i++) {
		struct pkt_mdata *mdata;

		if (unlikely(!res_descs[i].cookie)) {
			pr_err("SAM operation failed (no cookie: %d,%d)!\n", i, res_descs[i].out_len);
			perf_cntrs->drop_cnt++;
			continue;
		}

		mdata = (struct pkt_mdata *)res_descs[i].cookie;
		if (res_descs[i].status != SAM_CIO_OK) {

			pr_warn("SAM operation (%s) %d of %d failed! status = %d, len = %d\n",
				(mdata->state == (u8)PKT_STATE_ENC) ? "EnC" : "DeC",
				i, num, res_descs[i].status, res_descs[i].out_len);

#ifdef CRYPT_APP_VERBOSE_DEBUG
			if (larg->cmn_args.verbose) {
				char *tmp_buff = (char *)mdata->buf_vaddr + MVAPPS_NETA_PKT_EFEC_OFFS;
				mv_mem_dump((u8 *)tmp_buff, res_descs[i].out_len);
			}
#endif
			/* Free buffer */
			free_buf_from_sam_cookie(larg, res_descs[i].cookie);
			perf_cntrs->drop_cnt++;
			continue;
		}
		if ((larg->dir == CRYPTO_LB) && (mdata->state == (u8)PKT_STATE_ENC)) {
			mdata->state = (u8)PKT_STATE_DEC;
			res_descs_to_dec[num_to_dec++] = res_descs[i];
		} else {
			if ((mdata->state == (u8)PKT_STATE_DEC) && mdata->pad) {
				if (likely(res_descs[i].out_len > mdata->pad))
					res_descs[i].out_len -= mdata->pad;
			}
			res_descs_to_send[num_to_send++] = res_descs[i];
		}
	}

	if (num_to_dec) {
		err = dec_pkts(larg, res_descs_to_dec, num_to_dec);
		if (unlikely(err))
			return err;
	}

	if (num_to_send) {
#ifdef CRYPT_APP_PKT_ECHO_SUPPORT
		if (likely(larg->cmn_args.echo))
			echo_pkts(larg, res_descs_to_send, num_to_send);
#endif /* CRYPT_APP_PKT_ECHO_SUPPORT */

		return send_pkts(larg, res_descs_to_send, num_to_send);
	}
	return 0;
}

static inline int loop_sw_recycle(struct local_arg	*larg,
				  u8			 rx_ppio_id,
				  u8			 tx_ppio_id,
				  u8			 qid,
				  u16			 num)
{
	struct neta_ppio_desc	 descs[CRYPT_APP_MAX_BURST_SIZE];
	struct perf_cmn_cntrs	*perf_cntrs = &larg->cmn_args.perf_cntrs;
	int			 err;

/*pr_info("tid %d check on tc %d, qid %d\n", larg->cmn_args.id, tc, qid);*/
START_COUNT_CYCLES(pme_ev_cnt_rx);
	err = neta_ppio_recv(larg->lcl_ports_desc[rx_ppio_id].ppio, qid, descs, &num);
STOP_COUNT_CYCLES(pme_ev_cnt_rx, num);

#ifdef CRYPT_APP_VERBOSE_DEBUG
	if (larg->cmn_args.verbose && num)
		printf("recv %d pkts on ppio %d, qid %d\n", num, rx_ppio_id, qid);
#endif /* CRYPT_APP_VERBOSE_DEBUG */

	if (num) {
		perf_cntrs->rx_cnt += num;

		err = proc_rx_pkts(larg, rx_ppio_id, tx_ppio_id, qid, descs, num);
		if (unlikely(err))
			return err;
	}
	if (larg->enc_in_cntr) {
		if (larg->enc_ev) {
			sam_cio_set_event(larg->enc_cio, larg->enc_ev, 1);
			err = mv_sys_event_poll(&larg->enc_ev, 1, 100);
			if (err <= 0)
				pr_warn("Error during event poll: rc = %d, revents=0x%x\n",
					err, larg->enc_ev->revents);
		}
		err = deq_crypto_pkts(larg, larg->enc_cio);
		if (unlikely(err))
			return err;
	}

	if (larg->dec_in_cntr) {
		if (larg->dec_ev) {
			sam_cio_set_event(larg->dec_cio, larg->dec_ev, 1);
			err = mv_sys_event_poll(&larg->dec_ev, 1, 100);
			if (err <= 0)
				pr_warn("Error during event poll: rc = %d, revents=0x%x\n",
					err, larg->dec_ev->revents);
		}
		return deq_crypto_pkts(larg, larg->dec_cio);
	}

	return 0;
}

static int find_free_cio(struct glob_arg *garg, u8 device)
{
	int	i;
	u32	cios_map = garg->free_cios[device];

	if (cios_map == 0) {
		pr_err("no free CIO found!\n");
		return -ENOSPC;
	}

	i = 0;
	while (cios_map != 0) {
		if ((u32)(1 << i) & cios_map) {
			cios_map &= ~(u32)(1 << i);
			break;
		}
		i++;
	}
	garg->free_cios[device] = cios_map;
	return i;
}

static void get_next_qid(struct local_arg *larg, u8 *qid)
{
	u8 lqid;

	lqid = *qid;
	/* Find next queue to consume */
	do {
		lqid++;
		if (lqid == MVAPPS_NETA_MAX_NUM_QS_PER_TC)
			lqid = 0;
	} while (!(larg->cmn_args.qs_map & (1 << lqid)));

	*qid = lqid;
}

static int main_loop_cb(void *arg, int *running)
{
	struct local_arg	*larg = (struct local_arg *)arg;
	u8 qid;
	u16 num;
	int i, err, num_ports = 0;
	u8 src_ports[MVAPPS_NETA_MAX_NUM_PORTS];
	u8 dst_ports[MVAPPS_NETA_MAX_NUM_PORTS];

	if (!larg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}
	num = larg->cmn_args.burst;
	qid = 0;

	num_ports = larg->cmn_args.num_ports / garg.cmn_args.cpus;
	if ((num_ports < 1) || (num_ports > 2)) {
		*running = 0;
		pr_err("Bad configuration: ports = %d, cpus = %d\n",
			larg->cmn_args.num_ports, garg.cmn_args.cpus);

		return -EINVAL;
	}
	for (i = 0; i < num_ports; i++) {
		src_ports[i] = i + larg->cmn_args.id;
		dst_ports[i] = (src_ports[i] + 1) % larg->cmn_args.num_ports;
		pr_info("thread #%d (cpu #%d): port %d -> port %d\n",
			larg->cmn_args.id, sched_getcpu(), src_ports[i], dst_ports[i]);
	}
	while (*running) {
		get_next_qid(larg, &qid);
		for (i = 0; i < num_ports; i++) {
			u8 src = src_ports[i];
			u8 dst = dst_ports[i];

			err = loop_sw_recycle(larg, src, dst, qid, num);
			if (err) {
				*running = 0;
				return err;
			}

			if (larg->tx_in_cntr[dst])
				free_sent_buffers(larg, dst, qid);
		}
		if (larg->enc_in_cntr) {
			err = deq_crypto_pkts(larg, larg->enc_cio);
			if (unlikely(err))
				break;
		}
		if (larg->dec_in_cntr && (larg->dec_cio != larg->enc_cio)) {
			err = deq_crypto_pkts(larg, larg->dec_cio);
			if (unlikely(err))
				break;
		}
	}
	*running = 0;
	return err;
}


static int init_all_modules(void)
{
	int			err;

	pr_info("Global initializations ...\n");

	err = mv_sys_dma_mem_init(CRYPT_APP_DMA_MEM_SIZE);
	if (err)
		return err;

	err = neta_init();
	if (err)
		return err;

	pr_info("done\n");
	return 0;
}

static int create_sam_sessions(struct sam_sa **enc_sa, struct sam_sa **dec_sa,
			       enum sam_cipher_alg cipher_alg, enum sam_cipher_mode cipher_mode,
			       enum sam_auth_alg auth_alg)
{
	struct sam_session_params	 sa_params;
	int				 err;

	memset(&sa_params, 0, sizeof(sa_params));
	sa_params.proto = SAM_PROTO_NONE;

	sa_params.cipher_alg = cipher_alg;  /* cipher algorithm */
	sa_params.cipher_mode = cipher_mode; /* cipher mode */
	sa_params.cipher_iv = NULL;     /* default IV */
	if (sa_params.cipher_alg == SAM_CIPHER_3DES) {
		sa_params.cipher_key = rfc3602_3des_cbc_t1_key;    /* cipher key */
		sa_params.cipher_key_len = sizeof(rfc3602_3des_cbc_t1_key); /* cipher key size (in bytes) */
	} else if (sa_params.cipher_alg == SAM_CIPHER_AES) {
		sa_params.cipher_key = rfc3602_aes128_cbc_t1_key;    /* cipher key */
		sa_params.cipher_key_len = sizeof(rfc3602_aes128_cbc_t1_key); /* cipher key size (in bytes) */
	} else {
		pr_err("Unknown cipher alg (%d)!\n", sa_params.cipher_alg);
		return -EINVAL;
	}

	sa_params.auth_alg = auth_alg; /* authentication algorithm */
	if (sa_params.auth_alg != SAM_AUTH_NONE) {
		if (sa_params.auth_alg == SAM_AUTH_HMAC_SHA1) {
			sa_params.auth_key = rfc3602_sha1_t1_auth_key;    /* auth key */
			sa_params.auth_key_len = sizeof(rfc3602_sha1_t1_auth_key); /* auth key size (in bytes) */
		} else {
			pr_err("Unknown authetication alg (%d)!\n", sa_params.auth_alg);
			return -EINVAL;
		}
	} else {
		sa_params.auth_key = NULL;    /* auth key */
		sa_params.auth_key = 0;
	}
	sa_params.u.basic.auth_aad_len = 0;   /* Additional Data (AAD) size (in bytes) */
	if (sa_params.auth_alg != SAM_AUTH_NONE)
		sa_params.u.basic.auth_icv_len = ICV_LEN;
	else
		sa_params.u.basic.auth_icv_len = 0;

	sa_params.dir = SAM_DIR_ENCRYPT;   /* operation direction: encode */
	err = sam_session_create(&sa_params, enc_sa);
	if (err) {
		pr_err("EnC SA creation failed (%d)!\n", err);
		return err;
	}
	if (!*enc_sa) {
		pr_err("EnC SA creation failed!\n");
		return -EFAULT;
	}

	sa_params.dir = SAM_DIR_DECRYPT;   /* operation direction: decode */
	err = sam_session_create(&sa_params, dec_sa);
	if (err) {
		pr_err("DeC SA creation failed (%d)!\n", err);
		return err;
	}
	if (!*dec_sa) {
		pr_err("DeC SA creation failed!\n");
		return -EFAULT;
	}
	return 0;
}

static void destroy_sam_sessions(struct sam_cio		*enc_cio,
				struct sam_cio		*dec_cio,
				struct sam_sa		*enc_sa,
				struct sam_sa		*dec_sa)
{
	if (enc_sa) {
		if (sam_session_destroy(enc_sa))
			pr_err("EnC SA destroy failed!\n");
	}

	if (dec_sa) {
		if (sam_session_destroy(dec_sa))
			pr_err("DeC SA destroy failed!\n");
	}

	if (enc_cio) {
		if (sam_cio_flush(enc_cio))
			pr_err("EnC CIO flush failed!\n");
	}

	if (dec_cio && (enc_cio != dec_cio)) {
		if (sam_cio_flush(dec_cio))
			pr_err("DeC CIO flush failed!\n");
	}
}

static int port_inq_fill(struct port_desc *port, u16 mtu, u16 qid)
{
	struct neta_buff_inf buffs_inf[CRYPT_APP_RX_Q_SIZE];
	void *buff_virt_addr;
	u16 buff_size;
	u16 i, num_of_buffs;

	if (port->inq_size > CRYPT_APP_RX_Q_SIZE)
		port->inq_size = CRYPT_APP_RX_Q_SIZE;

	buff_size = MVAPPS_MTU_TO_MRU(port->port_params.inqs_params.mtu) + MVAPPS_NETA_PKT_OFFS;
	/* do it only once to store high 32 bits of memory address */
	if (!neta_sys_dma_high_addr) {
		void *tmp_addr;

		tmp_addr = mv_sys_dma_mem_alloc(buff_size, NETA_PPIO_RX_BUF_ALIGN);
		if (!tmp_addr) {
			pr_err("failed to allocate mem for neta_sys_dma_high_addr!\n");
			return 0;
		}
		neta_sys_dma_high_addr = ((u64)tmp_addr) & (~((1ULL << 32) - 1));
		mv_sys_dma_mem_free(tmp_addr);
	}
	for (i = 0; i < port->inq_size; i++) {
		buff_virt_addr = mv_sys_dma_mem_alloc(buff_size, NETA_PPIO_RX_BUF_ALIGN);
		if (!buff_virt_addr) {
			pr_err("port %d: queue %d: failed to allocate buffer memory (%d)!\n",
				port->ppio_id, qid, i);
			break;
		}
		buffs_inf[i].addr = (neta_dma_addr_t)mv_sys_dma_mem_virt2phys(buff_virt_addr);
		/* cookie contains lower_32_bits of the va */
		buffs_inf[i].cookie = lower_32_bits((u64)buff_virt_addr);
	}
	num_of_buffs = i;

	neta_ppio_inq_put_buffs(port->ppio, qid, buffs_inf, &num_of_buffs);

	pr_debug("port %d: queue %d: %d of %d buffers added.\n",
		port->ppio_id, qid, num_of_buffs, port->inq_size);

	/* Not all buffers are put to queue - free them */
	while (i > num_of_buffs) {
		i--;
		buff_virt_addr = (void *)(((uintptr_t)buffs_inf[i].cookie) | neta_sys_dma_high_addr);
		mv_sys_dma_mem_free(buff_virt_addr);
	}
	return num_of_buffs;
}

static int port_inqs_fill(struct port_desc *port, u16 mtu)
{
	int num = 0;
	u16 tc, qid;

	for (tc = 0; tc < port->num_tcs; tc++) {
		for (qid = 0; qid < port->num_inqs[tc]; qid++)
			num += port_inq_fill(port, mtu, qid);
	}

	return num;
}

static int init_local_modules(struct glob_arg *garg)
{
	struct sam_init_params		init_params;
	int				err, port_index;
	int				i;

	pr_info("Specific modules initializations\n");

	garg->num_bufs = 0;
	for (port_index = 0; port_index < garg->cmn_args.num_ports; port_index++) {
		struct port_desc *port = &garg->ports_desc[port_index];

		err = app_find_port_info(port);
		if (err)
			return err;

		port->num_tcs	= CRYPT_APP_MAX_NUM_TCS_PER_PORT;
		for (i = 0; i < port->num_tcs; i++)
			port->num_inqs[i] = CRYPT_APP_MAX_NUM_TCS_PER_PORT;

		port->inq_size	= CRYPT_APP_RX_Q_SIZE;
		port->num_outqs	= CRYPT_APP_MAX_NUM_TCS_PER_PORT;
		port->outq_size	= CRYPT_APP_TX_Q_SIZE;
		port->first_inq	= CRYPT_APP_FIRST_INQ;

		/* pkt_offset=0 not to be changed, before recoding rx_data_path to use pkt_offset as well */
		err = app_port_init(port, garg->cmn_args.mtu, MVAPPS_NETA_PKT_OFFS);
		if (err) {
			pr_err("Failed to initialize port %d\n", port_index);
			return err;
		}
		/* fill RX queues with buffers */
		garg->num_bufs += port_inqs_fill(port, garg->cmn_args.mtu);

		/* enable port */
		err = app_port_enable(port);
	}

	/* SAM driver global initialization */
	init_params.max_num_sessions = CRYPT_APP_MAX_NUM_SESSIONS;
	sam_init(&init_params);

#ifdef CRYPT_APP_VERBOSE_DEBUG
	if (garg->cmn_args.verbose > 2)
		sam_set_debug_flags(0x3);
#endif
	return 0;
}

static void destroy_local_modules(struct glob_arg *garg)
{
	app_disable_all_ports(garg->ports_desc, garg->cmn_args.num_ports);

	/* Free all RX buffers */
	app_deinit_all_ports(garg->ports_desc, garg->cmn_args.num_ports);
}

static void destroy_all_modules(void)
{
	sam_deinit();
	neta_deinit();
	mv_sys_dma_mem_destroy();
}

static int prefetch_cmd_cb(void *arg, int argc, char *argv[])
{
	struct glob_arg *garg = (struct glob_arg *)arg;

	if (!garg) {
		pr_err("no garg obj passed!\n");
		return -EINVAL;
	}
	if ((argc != 1) && (argc != 2)) {
		pr_err("Invalid number of arguments for prefetch cmd!\n");
		return -EINVAL;
	}

	if (argc == 1) {
		printf("%d\n", garg->cmn_args.prefetch_shift);
		return 0;
	}

	garg->cmn_args.prefetch_shift = atoi(argv[1]);

	return 0;
}

static int perf_cmd_cb(void *arg, int argc, char *argv[])
{
	struct glob_arg *garg = (struct glob_arg *)arg;

	if (!garg) {
		pr_err("no garg obj passed!\n");
		return -EINVAL;
	}
	if (argc != 1) {
		pr_err("Invalid number of arguments for perf cmd!\n");
		return -EINVAL;
	}

	return apps_perf_dump(&garg->cmn_args);
}

#ifdef CRYPT_APP_STATS_SUPPORT
static int stats_cmd_cb(void *arg, int argc, char *argv[])
{
	struct glob_arg *garg = (struct glob_arg *)arg;
	int i, reset = 0;
	int thread, cpu = -1;

	if (!garg) {
		pr_err("no garg obj passed!\n");
		return -EINVAL;
	}
	i = 1;
	while (i < argc) {
		if (strcmp(argv[i], "--cpu") == 0) {
			if (argc < (i + 2)) {
				pr_err("Invalid number of arguments!\n");
				return -EINVAL;
			}
			if (argv[i + 1][0] == '-') {
				pr_err("Invalid arguments format!\n");
				return -EINVAL;
			}
			cpu = atoi(argv[i + 1]);
			i += 2;
		} else if (strcmp(argv[i], "--reset") == 0) {
			reset = 1;
			i += 1;
		} else {
			pr_err("argument (%s) not supported!\n", argv[i]);
			return -EINVAL;
		}
	}

	if (cpu >= 0) {
		thread = apps_cpu_to_thread(&garg->cmn_args, cpu);
		if (thread < 0) {
			pr_err("Invalid CPU number #%d\n", cpu);
			return -EINVAL;
		}
		print_local_stats(garg->cmn_args.largs[thread], cpu, reset);
	} else {
		for (thread = 0; thread < garg->cmn_args.cpus; thread++) {
			if (garg->cmn_args.largs[thread]) {
				cpu = apps_thread_to_cpu(&garg->cmn_args, thread);
				print_local_stats(garg->cmn_args.largs[thread], cpu, reset);
			}
		}
	}
	return 0;
}
#endif

#ifdef CHECK_CYCLES
static int pme_cmd_cb(void *arg, int argc, char *argv[])
{
	struct glob_arg *garg = (struct glob_arg *)arg;

	if (!garg) {
		pr_err("no garg obj passed!\n");
		return -EINVAL;
	}
	if (argc != 1) {
		pr_err("Invalid number of arguments for PME cmd!\n");
		return -EINVAL;
	}

	pme_ev_cnt_dump(pme_ev_cnt_rx, 1);
	pme_ev_cnt_dump(pme_ev_cnt_enq, 1);
	pme_ev_cnt_dump(pme_ev_cnt_deq, 1);
	pme_ev_cnt_dump(pme_ev_cnt_tx, 1);
	return 0;
}
#endif /* CHECK_CYCLES */

static int unregister_cli_cmds(struct glob_arg *garg)
{
	/* TODO: unregister cli cmds */
	return 0;
}

static int register_cli_cmds(struct glob_arg *garg)
{
	struct cli_cmd_params	 cmd_params;

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "prefetch";
	cmd_params.desc		= "Prefetch ahead shift (number of buffers)";
	cmd_params.format	= "<shift>";
	cmd_params.cmd_arg	= garg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))apps_prefetch_cmd_cb;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "perf";
	cmd_params.desc		= "Dump performance statistics";
	cmd_params.format	= NULL;
	cmd_params.cmd_arg	= garg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))perf_cmd_cb;
	mvapp_register_cli_cmd(&cmd_params);

#ifdef CRYPT_APP_STATS_SUPPORT
	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "stats";
	cmd_params.desc		= "Show crypto statistics";
	cmd_params.format	= "--cpu <id>  --reset\n"
				  "\t\t--cpu <id> - CPU core number, if not specified, apply to all cores\n"
				  "\t\t--reset    - reset statistics after show";
	cmd_params.cmd_arg	= garg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))stats_cmd_cb;
	mvapp_register_cli_cmd(&cmd_params);
#endif

#ifdef CHECK_CYCLES
	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "pme";
	cmd_params.desc		= "Performance Montitor Emulator";
	cmd_params.format	= NULL;
	cmd_params.cmd_arg	= garg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pme_cmd_cb;
	mvapp_register_cli_cmd(&cmd_params);
#endif /* CHECK_CYCLES */

	app_register_cli_common_cmds(&garg->cmn_args);

	return 0;
}

static int init_global(void *arg)
{
	struct glob_arg *garg = (struct glob_arg *)arg;
	int		 err, dev;

	if (!garg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	if (pthread_mutex_init(&garg->trd_lock, NULL) != 0) {
		pr_err("init lock failed!\n");
		return -EIO;
	}
	garg->num_devs = sam_get_num_inst();
	garg->free_cios = calloc(garg->num_devs, sizeof(u32));
	if (!garg->free_cios)
		return -ENOMEM;

	for (dev = 0; dev < garg->num_devs; dev++) {
		sam_get_available_cios(dev, &garg->free_cios[dev]);
		garg->free_cios[dev] &= ~CRYPT_APP_CIOS_RSRV;
	}

	err = init_all_modules();
	if (err)
		return err;

	err = init_local_modules(garg);
	if (err)
		return err;

	if (garg->cmn_args.cli) {
		err = register_cli_cmds(garg);
		if (err)
			return err;
	}

	gettimeofday(&garg->cmn_args.ctrl_trd_last_time, NULL);

#ifdef CHECK_CYCLES
	pme_ev_cnt_rx = pme_ev_cnt_create("PP-IO Recv", 1000000, 0);
	if (pme_ev_cnt_rx < 0) {
		pr_err("PME failed!\n");
		return pme_ev_cnt_rx;
	}
	pme_ev_cnt_enq = pme_ev_cnt_create("SAM EnQ", 1000000, 0);
	if (pme_ev_cnt_enq < 0) {
		pr_err("PME failed!\n");
		return pme_ev_cnt_enq;
	}
	pme_ev_cnt_deq = pme_ev_cnt_create("SAM DeQ", 1000000, 0);
	if (pme_ev_cnt_deq < 0) {
		pr_err("PME failed!\n");
		return pme_ev_cnt_deq;
	}
	pme_ev_cnt_tx = pme_ev_cnt_create("PP-IO Send", 1000000, 0);
	if (pme_ev_cnt_tx < 0) {
		pr_err("PME failed!\n");
		return pme_ev_cnt_tx;
	}
#endif /* CHECK_CYCLES */

	return 0;
}

static void deinit_global(void *arg)
{
	struct glob_arg *garg = (struct glob_arg *)arg;

	if (!garg)
		return;
	if (garg->cmn_args.cli)
		unregister_cli_cmds(garg);

#ifdef MVCONF_SAM_STATS
	app_sam_show_stats(1);
#endif
	destroy_local_modules(garg);
	destroy_all_modules();

	if (garg->free_cios)
		free(garg->free_cios);
}

static int init_local(void *arg, int id, void **_larg)
{
	struct glob_arg		*garg = (struct glob_arg *)arg;
	struct local_arg	*larg;
	struct sam_cio_params	cio_params;
	struct sam_cio		*cio;
	int			 i, err, cio_id, sam_device;
	struct sam_cio_event_params ev_params;

	pr_info("Local initializations for thread %d\n", id);

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

	larg->cmn_args.num_ports	= garg->cmn_args.num_ports;

	larg->lcl_ports_desc = (struct lcl_port_desc *)
					   malloc(larg->cmn_args.num_ports * sizeof(struct lcl_port_desc));
	if (!larg->lcl_ports_desc) {
		pr_err("no mem for local-port-desc obj!\n");
		free(larg->cmn_args.plat);
		free(larg);
		return -ENOMEM;
	}
	memset(larg->lcl_ports_desc, 0, larg->cmn_args.num_ports * sizeof(struct lcl_port_desc));

	pthread_mutex_lock(&garg->trd_lock);
	sam_device = id % sam_get_num_inst();
	cio_id = find_free_cio(garg, sam_device);
	if (cio_id < 0) {
		pr_err("free CIO not found!\n");
		pthread_mutex_unlock(&garg->trd_lock);
		return cio_id;
	}
	memset(larg->enc_name, 0, sizeof(larg->enc_name));
	snprintf(larg->enc_name, sizeof(larg->enc_name), "cio-%d:%d", sam_device, cio_id);
	memset(&cio_params, 0, sizeof(cio_params));
	cio_params.match = larg->enc_name;
	cio_params.size = CRYPT_APP_CIO_Q_SIZE;
	err = sam_cio_init(&cio_params, &cio);
	pthread_mutex_unlock(&garg->trd_lock);
	if (err != 0) {
		pr_err("CIO init failed!\n");
		return err;
	}
	if (garg->dir == CRYPTO_ENC)
		larg->enc_cio = cio;
	else if (garg->dir == CRYPTO_DEC)
		larg->dec_cio = cio;
	else if (garg->dir == CRYPTO_LB) {

		larg->enc_cio = cio;

		/* If number of Cores > number of available CIOs then use the same cio or
		 * encrypt and decrypt
		 */
		if (2 * garg->cmn_args.cpus > sam_get_num_inst() * sam_get_num_cios(sam_device)) {
			strcpy(larg->dec_name, larg->enc_name);
			larg->dec_cio = larg->enc_cio;
		} else {
			/* Take other device if available */
			sam_device = (sam_device + 1) % sam_get_num_inst();

			pthread_mutex_lock(&garg->trd_lock);
			cio_id = find_free_cio(garg, sam_device);
			pthread_mutex_unlock(&garg->trd_lock);

			if (cio_id < 0) {
				pr_err("free CIO not found!\n");
				return cio_id;
			}
			memset(larg->dec_name, 0, sizeof(larg->dec_name));
			snprintf(larg->dec_name, sizeof(larg->dec_name), "cio-%d:%d", sam_device, cio_id);
			memset(&cio_params, 0, sizeof(cio_params));
			cio_params.match = larg->dec_name;
			cio_params.size = CRYPT_APP_CIO_Q_SIZE;
			err = sam_cio_init(&cio_params, &larg->dec_cio);
			if (err != 0)
				return err;
		}
	}
	larg->cmn_args.id               = id;
	larg->cmn_args.verbose		= garg->cmn_args.verbose;
	larg->cmn_args.burst		= garg->cmn_args.burst;
	larg->cmn_args.echo             = garg->cmn_args.echo;
	larg->cmn_args.prefetch_shift   = garg->cmn_args.prefetch_shift;
	larg->cmn_args.num_ports        = garg->cmn_args.num_ports;

	larg->dir			= garg->dir;

	larg->lcl_ports_desc = (struct lcl_port_desc *)
					malloc(larg->cmn_args.num_ports * sizeof(struct lcl_port_desc));
	if (!larg->lcl_ports_desc) {
		pr_err("no mem for local-port-desc obj!\n");
		return -ENOMEM;
	}
	memset(larg->lcl_ports_desc, 0, larg->cmn_args.num_ports * sizeof(struct lcl_port_desc));
	for (i = 0; i < larg->cmn_args.num_ports; i++)
		app_port_local_init(i, larg->cmn_args.id,
				    &larg->lcl_ports_desc[i], &garg->ports_desc[i]);

	err = mv_stack_create(garg->num_bufs, &larg->stack_hndl);
	if (err) {
		pr_err("failed to create Stack for %d elements!\n", garg->num_bufs);
		return -EFAULT;
	}
	/* Allocate packet metadata structures for all buffers */
	for (i = 0; i < garg->num_bufs; i++) {
		struct pkt_mdata *mdata = malloc(sizeof(struct pkt_mdata));

		if (!mdata) {
			pr_warn("Can't allocate all metadata structures\n");
			break;
		}
		mv_stack_push(larg->stack_hndl, mdata);
	}
	pr_debug("%d of %d metadata structures allocated\n", i, garg->num_bufs);

	err = create_sam_sessions(&larg->enc_sa, &larg->dec_sa,
				  garg->cipher_alg, garg->cipher_mode,
				  garg->auth_alg);
	if (err)
		return err;

	if (!larg->enc_sa || !larg->dec_sa) {
		pr_err("failed to create SAM sesions!\n");
		return -EFAULT;
	}

	larg->cmn_args.garg = garg;
	garg->cmn_args.largs[id] = larg;

	larg->cmn_args.qs_map = garg->cmn_args.qs_map;

	if (garg->use_events) {
		ev_params.pkt_coal = ev_pkts_coal;
		ev_params.usec_coal = ev_usec_coal;
		if (larg->enc_cio) {
			err = sam_cio_create_event(larg->enc_cio, &ev_params, &larg->enc_ev);
			if (err) {
				printf("Can't create CIO event");
				return -EINVAL;
			}
			larg->enc_ev->events = MV_SYS_EVENT_POLLIN;
		}
		if (larg->dec_cio) {
			err = sam_cio_create_event(larg->dec_cio, &ev_params, &larg->dec_ev);
			if (err) {
				printf("Can't create CIO event");
				return -EINVAL;
			}
			larg->dec_ev->events = MV_SYS_EVENT_POLLIN;
		}
	}

	pr_info("Local thread #%d (cpu #%d): Encrypt - %s, Decrypt - %s, qs_map = 0x%lx\n",
		id, sched_getcpu(), larg->enc_name,
		larg->dec_cio ? larg->dec_name : "None", larg->cmn_args.qs_map);

	*_larg = larg;
	return 0;
}

static void deinit_local(void *arg)
{
	struct local_arg *larg = (struct local_arg *)arg;
	struct pkt_mdata *mdata;
	int i;

	if (!larg)
		return;

	destroy_sam_sessions(larg->enc_cio, larg->dec_cio, larg->enc_sa, larg->dec_sa);

	pthread_mutex_lock(&larg->cmn_args.garg->trd_lock);
#ifdef CRYPT_APP_STATS_SUPPORT
	print_local_stats(larg, sched_getcpu(), 0);
#endif

#ifdef MVCONF_SAM_STATS
	if (larg->enc_cio) {
		pr_info("cpu #%d: %s\n", sched_getcpu(), larg->enc_name);
		app_sam_show_cio_stats(larg->enc_cio, "encrypt", 1);
	}

	if (larg->dec_cio && (larg->dec_cio != larg->enc_cio)) {
		pr_info("cpu #%d: %s\n", sched_getcpu(), larg->dec_name);
		app_sam_show_cio_stats(larg->dec_cio, "decrypt", 1);
	}
#endif /* MVCONF_SAM_STATS */
	pthread_mutex_unlock(&larg->cmn_args.garg->trd_lock);

	if (larg->lcl_ports_desc) {
		for (i = 0; i < larg->cmn_args.num_ports; i++)
			app_port_local_deinit(&larg->lcl_ports_desc[i]);
		free(larg->lcl_ports_desc);
	}
	/* Free all pkt_mdata structures and delete stack instance */
	i = 0;
	while (!mv_stack_is_empty(larg->stack_hndl)) {
		mdata = mv_stack_pop(larg->stack_hndl);
		if (!mdata)
			break;
		i++;
		free(mdata);
	}
	pr_debug("%d of %d metadata structures freed\n", i, larg->cmn_args.garg->num_bufs);

	mv_stack_delete(larg->stack_hndl);

	if (larg->dec_cio && (larg->dec_cio != larg->enc_cio))
		sam_cio_deinit(larg->dec_cio);
	if (larg->enc_cio)
		sam_cio_deinit(larg->enc_cio);

	free(larg);
}

static void usage(char *progname)
{
	printf("\n"
	       "MUSDK crypto-echo application.\n"
	       "\n"
	       "Usage: %s OPTIONS\n"
	       "  E.g. %s -i eth0,eth1 -c 1\n"
	       "\n"
	       "Mandatory OPTIONS:\n"
	       "\t-i, --interface <Eth-interfaces> (comma-separated, no spaces)\n"
	       "                  Interface count min 1, max %i\n"
	       "\n"
	       "Optional OPTIONS:\n"
	       "\t-b             <size>    Burst size (default is %d)\n"
	       "\t-c, --cores    <number>  Number of CPUs to use.\n"
	       "\t-a, --affinity <number>  Use setaffinity (default is no affinity).\n"
	       "\t-t             <mtu>     Set MTU (default is %d)\n"
#ifdef CRYPT_APP_VERBOSE_DEBUG
	       "\t-v                       Increase verbose debug (default is 0).\n"
	       "\t                         With every '-v', the debug is increased by one.\n"
	       "\t                         0 - none, 1 - pkts sent/recv indication, 2 - full pkt dump\n"
#endif /* CRYPT_APP_VERBOSE_DEBUG */
	       "\t--cipher-alg   <alg>     Cipher algorithm. Support: [none, aes128, 3des]. (default: aes128).\n"
	       "\t--cipher-mode  <alg>     Cipher mode. Support: [cbc, ecb]. (default: cbc).\n"
	       "\t--auth-alg     <alg>     Authentication algorithm. Support: [none, sha1]. (default: sha1).\n"
	       "\t--dir          <dir>     Operation direction. Support: [enc, dec, lb]. (default: lb)\n"
	       "\t--no-echo                No Echo packets\n"
	       "\t--use-events             Use events to wait for requests completed (default: polling)\n"
	       "\t--cli                    Use CLI\n"
	       "\t?, -h, --help            Display help and exit.\n\n"
	       "\n", MVAPPS_NO_PATH(progname), MVAPPS_NO_PATH(progname),
	       MVAPPS_NETA_MAX_NUM_PORTS, CRYPT_APP_MAX_BURST_SIZE, DEFAULT_MTU);
}

static int parse_args(struct glob_arg *garg, int argc, char *argv[])
{
	int	i = 1;

	garg->cmn_args.verbose = 0;
	garg->cmn_args.cli = 0;
	garg->cmn_args.cpus = 1;
	garg->cmn_args.affinity = -1;
	garg->cmn_args.burst = CRYPT_APP_DFLT_BURST_SIZE;
	garg->cmn_args.mtu = DEFAULT_MTU;
	garg->cmn_args.echo = 1;
	garg->dir = CRYPTO_LB;
	garg->cipher_alg = SAM_CIPHER_AES;
	garg->cipher_mode = SAM_CIPHER_CBC;
	garg->auth_alg = SAM_AUTH_HMAC_SHA1;
	garg->cmn_args.qs_map = 0;
	garg->cmn_args.qs_map_shift = 0;
	garg->cmn_args.pkt_offset = 0;
	garg->cmn_args.prefetch_shift = CRYPT_APP_PREFETCH_SHIFT;
	garg->cmn_args.ctrl_thresh = CRYPT_APP_CTRL_DFLT_THR;

	while (i < argc) {
		if ((strcmp(argv[i], "?") == 0) ||
		    (strcmp(argv[i], "-h") == 0) ||
		    (strcmp(argv[i], "--help") == 0)) {
			usage(argv[0]);
			exit(0);
		} else if (strcmp(argv[i], "-i") == 0) {
			char *token;

			if (argc < (i + 2)) {
				pr_err("Invalid number of arguments!\n");
				return -EINVAL;
			}
			if (argv[i + 1][0] == '-') {
				pr_err("Invalid interface arguments format!\n");
				return -EINVAL;
			}

			/* count the number of tokens separated by ',' */
			for (token = strtok(argv[i + 1], ","), garg->cmn_args.num_ports = 0;
			     token;
			     token = strtok(NULL, ","), garg->cmn_args.num_ports++) {
				int tmp;

				snprintf(garg->ports_desc[garg->cmn_args.num_ports].name,
					 sizeof(garg->ports_desc[garg->cmn_args.num_ports].name),
					 "%s", token);
				tmp = sscanf(garg->ports_desc[garg->cmn_args.num_ports].name, "eth%d",
				       &garg->ports_desc[garg->cmn_args.num_ports].ppio_id);
				if ((tmp != 1) ||
				    (garg->ports_desc[garg->cmn_args.num_ports].ppio_id >
				     MVAPPS_NETA_MAX_NUM_PORTS)) {
					pr_err("Invalid interface number %d!\n",
						garg->ports_desc[garg->cmn_args.num_ports].ppio_id);
					return -EINVAL;
				}
			}

			if (garg->cmn_args.num_ports == 0) {
				pr_err("Invalid interface arguments format!\n");
				return -EINVAL;
			} else if (garg->cmn_args.num_ports > MVAPPS_NETA_MAX_NUM_PORTS) {
				pr_err("too many ports specified (%d vs %d)\n",
				       garg->cmn_args.num_ports, MVAPPS_NETA_MAX_NUM_PORTS);
				return -EINVAL;
			}
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
			garg->cmn_args.burst = atoi(argv[i + 1]);
			i += 2;
		} else if (strcmp(argv[i], "-t") == 0) {
			if (argc < (i + 2)) {
				pr_err("Invalid number of arguments!\n");
				return -EINVAL;
			}
			if (argv[i + 1][0] == '-') {
				pr_err("Invalid arguments format!\n");
				return -EINVAL;
			}
			garg->cmn_args.mtu = atoi(argv[i + 1]);
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
			garg->cmn_args.cpus = atoi(argv[i + 1]);
			i += 2;
		} else if (strcmp(argv[i], "-a") == 0) {
			garg->cmn_args.affinity = atoi(argv[i + 1]);
			i += 2;
		} else if (strcmp(argv[i], "-m") == 0) {
			char *token;

			if (argc < (i + 2)) {
				pr_err("Invalid number of arguments!\n");
				return -EINVAL;
			}
			if (argv[i + 1][0] == '-') {
				pr_err("Invalid arguments format!\n");
				return -EINVAL;
			}
			token = strtok(argv[i + 1], ":");
			garg->cmn_args.qs_map = strtoul(token, NULL, 16);
			token = strtok(NULL, "");
			garg->cmn_args.qs_map_shift = atoi(token);
			i += 2;
#ifdef CRYPT_APP_VERBOSE_DEBUG
		} else if (strcmp(argv[i], "-v") == 0) {
			garg->cmn_args.verbose++;
			i += 1;
#endif /* CRYPT_APP_VERBOSE_DEBUG */
		} else if (strcmp(argv[i], "--no-echo") == 0) {
			garg->cmn_args.echo = 0;
			i += 1;
		} else if (strcmp(argv[i], "--dir") == 0) {
			if (strcmp(argv[i+1], "enc") == 0)
				garg->dir = CRYPTO_ENC;
			else if (strcmp(argv[i+1], "dec") == 0)
				garg->dir = CRYPTO_DEC;
			else if (strcmp(argv[i+1], "lb") == 0)
				garg->dir = CRYPTO_LB;
			else {
				pr_err("Direction (%s) not supported!\n", argv[i+1]);
				return -EINVAL;
			}
			i += 2;
		} else if (strcmp(argv[i], "--cipher-alg") == 0) {
			if (strcmp(argv[i+1], "aes128") == 0)
				garg->cipher_alg = SAM_CIPHER_AES;
			else if (strcmp(argv[i+1], "3des") == 0)
				garg->cipher_alg = SAM_CIPHER_3DES;
			else if (strcmp(argv[i+1], "none") == 0)
				garg->cipher_alg = SAM_CIPHER_NONE;
			else {
				pr_err("Cipher alg (%s) not supported!\n", argv[i+1]);
				return -EINVAL;
			}
			i += 2;
		} else if (strcmp(argv[i], "--cipher-mode") == 0) {
			if (strcmp(argv[i+1], "cbc") == 0)
				garg->cipher_alg = SAM_CIPHER_CBC;
			else if (strcmp(argv[i+1], "ecb") == 0)
				garg->cipher_alg = SAM_CIPHER_ECB;
			else {
				pr_err("Cipher mode (%s) not supported!\n", argv[i+1]);
				return -EINVAL;
			}
			i += 2;
		} else if (strcmp(argv[i], "--auth-alg") == 0) {
			if (strcmp(argv[i+1], "sha1") == 0)
				garg->auth_alg = SAM_AUTH_HMAC_SHA1;
			else if (strcmp(argv[i+1], "sha256") == 0)
				garg->auth_alg = SAM_AUTH_HMAC_SHA2_256;
			else if (strcmp(argv[i+1], "md5") == 0)
				garg->auth_alg = SAM_AUTH_HMAC_MD5;
			else if (strcmp(argv[i+1], "none") == 0)
				garg->auth_alg = SAM_AUTH_NONE;
			else {
				pr_err("Auth alg (%s) not supported!\n", argv[i+1]);
				return -EINVAL;
			}
			i += 2;
		} else if (strcmp(argv[i], "--use-events") == 0) {
			garg->use_events = true;
			i += 1;
		} else if (strcmp(argv[i], "--cli") == 0) {
			garg->cmn_args.cli = 1;
			i += 1;
		} else {
			pr_err("argument (%s) not supported!\n", argv[i]);
			return -EINVAL;
		}
	}

	/* Now, check validity of all inputs */
	if (!garg->cmn_args.num_ports ||
	    !garg->ports_desc[0].name) {
		pr_err("No port defined!\n");
		return -EINVAL;
	}
	if (garg->cmn_args.burst > CRYPT_APP_MAX_BURST_SIZE) {
		pr_err("illegal burst size requested (%d vs %d)!\n",
		       garg->cmn_args.burst, CRYPT_APP_MAX_BURST_SIZE);
		return -EINVAL;
	}
	if (garg->cmn_args.cpus > MVAPPS_MAX_NUM_CORES) {
		pr_err("illegal num cores requested (%d vs %d)!\n",
		       garg->cmn_args.cpus, MVAPPS_MAX_NUM_CORES);
		return -EINVAL;
	}
	if ((garg->cmn_args.affinity != -1) &&
	    ((garg->cmn_args.cpus + garg->cmn_args.affinity) > MVAPPS_MAX_NUM_CORES)) {
		pr_err("illegal num cores or affinity requested (%d,%d vs %d)!\n",
		       garg->cmn_args.cpus, garg->cmn_args.affinity, MVAPPS_MAX_NUM_CORES);
		return -EINVAL;
	}

	if (garg->cmn_args.qs_map &&
	    (MVAPPS_NETA_MAX_NUM_QS_PER_TC == 1) &&
	    (CRYPT_APP_MAX_NUM_TCS_PER_PORT == 1)) {
		pr_warn("no point in queues-mapping; ignoring.\n");
		garg->cmn_args.qs_map = 1;
		garg->cmn_args.qs_map_shift = 1;
	} else if (!garg->cmn_args.qs_map) {
		garg->cmn_args.qs_map = 1;
		garg->cmn_args.qs_map_shift = CRYPT_APP_MAX_NUM_TCS_PER_PORT;
	}

	if ((garg->cmn_args.cpus != 1) &&
	    (garg->cmn_args.qs_map & (garg->cmn_args.qs_map << garg->cmn_args.qs_map_shift))) {
		pr_err("Invalid queues-mapping (ovelapping CPUs)!\n");
		return -EINVAL;
	}

	if (garg->cmn_args.prefetch_shift > garg->cmn_args.burst)
		garg->cmn_args.prefetch_shift = garg->cmn_args.burst - 1;

	/* Print all inputs arguments */
	pr_info("cpus          : %d\n", garg->cmn_args.cpus);
	pr_info("ports         : %d\n", garg->cmn_args.num_ports);
	pr_info("burst         : %d\n", garg->cmn_args.burst);

	pr_info("direction     : %d\n", garg->dir);
	pr_info("swap addr     : %s\n", garg->cmn_args.echo ? "Swap" : "Not swap");
	pr_info("cipher-alg    : %d\n", garg->cipher_alg);
	pr_info("cipher-mode   : %d\n", garg->cipher_mode);
	pr_info("auth-alg      : %d\n", garg->auth_alg);

	return 0;
}

int main(int argc, char *argv[])
{
	struct mvapp_params	mvapp_params;
	u64			cores_mask;
	int			i, err;

	setbuf(stdout, NULL);

	pr_debug("pr_debug is enabled\n");

	err = parse_args(&garg, argc, argv);
	if (err) {
		free(garg.cmn_args.plat);
		return err;
	}

	cores_mask = 0;
	for (i = 0; i < garg.cmn_args.cpus; i++, cores_mask <<= 1, cores_mask |= 1)
		;
	cores_mask <<= (garg.cmn_args.affinity != -1) ? garg.cmn_args.affinity : 0;

	memset(&mvapp_params, 0, sizeof(mvapp_params));
	mvapp_params.use_cli		= garg.cmn_args.cli;
	mvapp_params.num_cores		= garg.cmn_args.cpus;
	mvapp_params.cores_mask		= cores_mask;
	mvapp_params.global_arg		= (void *)&garg;
	mvapp_params.init_global_cb	= init_global;
	mvapp_params.deinit_global_cb	= deinit_global;
	mvapp_params.init_local_cb	= init_local;
	mvapp_params.deinit_local_cb	= deinit_local;
	mvapp_params.main_loop_cb	= main_loop_cb;
	if (!mvapp_params.use_cli)
		mvapp_params.ctrl_cb	= app_ctrl_cb;
	return mvapp_go(&mvapp_params);
}
