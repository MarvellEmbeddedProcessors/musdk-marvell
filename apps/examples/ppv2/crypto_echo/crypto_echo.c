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
#include "lib/net.h"

#include "mv_stack.h"
#include "env/mv_sys_dma.h"
#include "env/io.h"

#include "mv_pp2.h"
#include "mv_pp2_hif.h"
#include "mv_pp2_bpool.h"
#include "mv_pp2_ppio.h"
#include "mv_sam.h"

#include "pp2_utils.h"
#include "sam_utils.h"
#include "mvapp.h"
#include "perf_mon_emu.h"

#define CRYPT_APP_DEF_Q_SIZE		256/*1024*/
#define CRYPT_APP_HIF_Q_SIZE		CRYPT_APP_DEF_Q_SIZE
#define CRYPT_APP_RX_Q_SIZE		CRYPT_APP_DEF_Q_SIZE
#define CRYPT_APP_TX_Q_SIZE		(2 * CRYPT_APP_DEF_Q_SIZE)
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
#define CRYPT_APP_STATS_SUPPORT
#define CRYPT_APP_PREFETCH_SHIFT		4
#define CRYPT_APP_MAX_NUM_FLOWS			64


#define PP2_COOKIE_SET_ALL_INFO(_c, _rp, _tp, _bp, _d)	\
	((_c) = (((_c) & ~0x3d) | \
		(((_d) << 0) | ((_rp) << 2) | ((_tp) << 3) | ((_bp) << 4))))

#define PP2_COOKIE_CLEAR_ALL_INFO(_c)	((_c) = ((_c) & ~0x3d))

/*#define CRYPT_APP_BPOOLS_INF	{ {384, 4096}, {2048, 1024} }*/
#define CRYPT_APP_BPOOLS_INF	{ {2048, 4096} }
/*#define CRYPT_APP_BPOOLS_JUMBO_INF	{ {2048, 4096}, {10240, 512} }*/
#define CRYPT_APP_BPOOLS_JUMBO_INF	{ {10240, 512} }

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

static u8 md5_auth_key[] = {
	0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b,
	0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b
};

static u32 t1_spi = 0x1234;
static u64 t1_seq = 0x6;

static u8 tunnel_src_ip4[] = {192, 168, 2, 3};
static u8 tunnel_dst_ip4[] = {192, 168, 2, 5};

static u8 tunnel_src_ip6[] = {
	0x20, 0x00, 0x1f, 0x3c, 0x55, 0x18, 0x00, 0x00,
	0x00, 0x00, 0x1f, 0x33, 0x44, 0x55, 0x66, 0x77
};
static u8 tunnel_dst_ip6[] = {
	0x20, 0x00, 0x1f, 0x3c, 0x55, 0x18, 0x00, 0x00,
	0x00, 0x00, 0x1f, 0x33, 0x44, 0x55, 0x66, 0x88
};

static u16 dtls_epoch = 0xD;			/**< for DTLS only */
static u64 ssl_seq = 0x34;			/**< Initial sequence number */

/* Masks for mdata flags field */
#define MDATA_FLAGS_IP4_SEQID_MASK	BIT(0)   /* Set seqid field in IP header before send */

#define MDATA_FLAGS_IP4_CSUM_MASK	BIT(14)  /* Recalculate IPv4 checksum on TX */
#define MDATA_FLAGS_L4_CSUM_MASK	BIT(15)  /* Recalculate L4 checksum on TX */

#ifdef CRYPT_APP_STATS_SUPPORT
struct crypto_echo_stats {
	u64 rx_pkts[MVAPPS_PP2_MAX_NUM_PORTS];
	u64 rx_drop[MVAPPS_PP2_MAX_NUM_PORTS];
	u64 enc_enq;
	u64 enc_drop;
	u64 deq_cnt;
	u64 deq_err;
	u64 dec_enq;
	u64 dec_drop;
	u64 tx_pkts[MVAPPS_PP2_MAX_NUM_PORTS];
	u64 tx_drop[MVAPPS_PP2_MAX_NUM_PORTS];
};
#define CRYPT_STATS(c) c
#else
#define CRYPT_STATS(c)
#endif /* CRYPT_APP_STATS_SUPPORT */

struct pkt_mdata {
	u8 state; /* as defined in enum pkt_state */
	u8 rx_port;
	u8 tx_port;
	u8 data_offs;
	struct crypto_flow *flow;
	void *buf_vaddr;
	struct pp2_bpool *bpool;
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

enum crypto_flow_mode {
	DEFAULT_FLOW,
	PACKET_FLOW
};

struct crypto_params {
	int				valid;
	enum crypto_dir			dir;
	enum sam_crypto_protocol	crypto_proto;
	enum sam_cipher_alg		cipher_alg;
	enum sam_cipher_mode		cipher_mode;
	u8				*cipher_key;
	int				cipher_key_len;
	enum sam_auth_alg		auth_alg;
	u8				*auth_key;
	int				auth_key_len;
	enum sam_ssltls_version         ssl_version;
	int				tunnel;
	int				ip6;
	int				seq64;
	int				capwap;
};

struct flow_params {
	int			valid;
	u32			thread_id;
	struct mv_2tuple	pkt_params;
	u8			tx_ppio_id;
};

struct crypto_flow {
	struct flow_params	*flow_params;
	struct crypto_params	*crypto_params;
	u8			txp;
	u8			txq;
	u16			flags;
	struct sam_sa		*enc_sa;
	struct sam_sa		*dec_sa;
};

struct local_arg;

struct glob_arg {
	struct glb_common_args		cmn_args;  /* Keep first */
	pthread_mutex_t			trd_lock;
	int				num_bufs;
	int                             num_devs;
	u32                             *free_cios; /* per device */
	enum crypto_flow_mode		flow_mode;
	struct crypto_params		def_crypto_params;
	struct crypto_params		crypto_params[CRYPT_APP_MAX_NUM_SESSIONS];
	struct flow_params		flow_params[CRYPT_APP_MAX_NUM_SESSIONS];
	int				last_flow_param;
};

struct local_arg {
	struct local_common_args	cmn_args;  /* Keep first */

	struct mv_stack			*stack_hndl;
	char				enc_name[15];
	struct sam_cio			*enc_cio;
	u32				seq_id[MVAPPS_PP2_MAX_NUM_PORTS];
	char				dec_name[15];
	struct sam_cio			*dec_cio;
#ifdef CRYPT_APP_STATS_SUPPORT
	struct crypto_echo_stats	stats;
#endif
	struct crypto_flow		def_flows[MVAPPS_PP2_MAX_NUM_PORTS];
	struct crypto_flow		flows[CRYPT_APP_MAX_NUM_SESSIONS];
	int				last_flow;
};

static struct glob_arg garg = {};

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
		printf("RX packets (port #%d)       : %" PRIu64 " packets\n", i, larg->stats.rx_pkts[i]);
		printf("RX drops (port #%d)         : %" PRIu64 " packets\n", i, larg->stats.rx_drop[i]);
	}

	printf("Encrypt enqueue            : %" PRIu64 " packets\n", larg->stats.enc_enq);
	printf("Encrypt drop               : %" PRIu64 " packets\n", larg->stats.enc_drop);
	printf("Dequeue count              : %" PRIu64 " packets\n", larg->stats.deq_cnt);
	printf("Dequeue errors             : %" PRIu64 " packets\n", larg->stats.deq_err);
	printf("Decrypt enqueue            : %" PRIu64 " packets\n", larg->stats.dec_enq);
	printf("Decrypt drop               : %" PRIu64 " packets\n", larg->stats.dec_drop);

	for (i = 0; i < larg->cmn_args.num_ports; i++) {
		printf("TX packets (port #%d)       : %" PRIu64 " packets\n", i, larg->stats.tx_pkts[i]);
		printf("TX drops (port #%d)         : %" PRIu64 " packets\n", i, larg->stats.tx_drop[i]);
	}
	printf("\n");
}
#endif /* CRYPT_APP_STATS_SUPPORT */

static void free_buf_from_sam_cookie(struct local_arg *larg, void *cookie)
{
	struct pkt_mdata	*mdata = (struct pkt_mdata *)cookie;
	struct pp2_lcl_common_args *pp2_args = (struct pp2_lcl_common_args *) larg->cmn_args.plat;

	char			*buff = mdata->buf_vaddr;
	dma_addr_t		pa;
	struct pp2_bpool	*bpool;
	struct pp2_buff_inf	binf;

	pa = mv_sys_dma_mem_virt2phys(buff);
	bpool = mdata->bpool;
	binf.addr = pa;
	binf.cookie = (u64)((uintptr_t)(buff));
	pp2_bpool_put_buff(pp2_args->hif, bpool, &binf);
	mv_stack_push(larg->stack_hndl, mdata);
}

static struct crypto_flow *get_packet_flow(struct local_arg *larg, u8 rx_port, void *pkt)
{
	struct crypto_flow *flow = NULL;
	int i;

	/* look local database first - TBD */
	for (i = 0; i <= larg->last_flow; i++) {
		flow = &larg->flows[i];
		if (!flow->crypto_params)
			continue;
	}
	/* Check global database for crypto_flow_params and create new flow */
	if (!flow) {
		struct glob_arg *garg = larg->cmn_args.garg;

		for (i = 0; i <= garg->last_flow_param; i++) {
			struct flow_params *params = &garg->flow_params[i];

			if ((params->valid == 0) || (params->thread_id != larg->cmn_args.id))
				continue;
		}
	}
	return flow;
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
			prefetch(tmp_buff + MVAPPS_PP2_PKT_DEF_OFFS);
		}
		/* pointer to MAC header */
		mdata = sam_res_descs[i].cookie;
		tmp_buff = (char *)mdata->buf_vaddr + MVAPPS_PP2_PKT_DEF_EFEC_OFFS;

#ifdef CRYPT_APP_VERBOSE_DEBUG
		if (larg->cmn_args.verbose > 1) {
			printf("pkt before echo (len %d):\n",
			       sam_res_descs[i].out_len);
			mv_mem_dump((u8 *)tmp_buff, sam_res_descs[i].out_len);
		}
#endif /* CRYPT_APP_VERBOSE_DEBUG */
		swap_l2(tmp_buff);
	}
}
#endif /* CRYPT_APP_PKT_ECHO_SUPPORT */

static inline int proc_rx_pkts(struct local_arg *larg,
			   u8 rx_ppio_id, struct pp2_ppio_desc *descs, u16 num)
{
	struct pp2_lcl_common_args *pp2_args = (struct pp2_lcl_common_args *) larg->cmn_args.plat;
	struct sam_cio_op_params sam_descs[CRYPT_APP_MAX_BURST_SIZE];
	struct sam_cio_ipsec_params ipsec_descs[CRYPT_APP_MAX_BURST_SIZE];
	struct sam_cio_ssltls_params ssltls_descs[CRYPT_APP_MAX_BURST_SIZE];
	struct sam_buf_info	 src_buf_infs[CRYPT_APP_MAX_BURST_SIZE];
	struct sam_buf_info	 dst_buf_infs[CRYPT_APP_MAX_BURST_SIZE];
	struct pkt_mdata	*mdata;
	struct sam_sa		*sa;
	struct sam_cio		*cio;
	enum pkt_state		state;
	int			err;
	u16			i, bpool_buff_len, num_got;
	u8			data_offs;
	struct perf_cmn_cntrs	*perf_cntrs = &larg->cmn_args.perf_cntrs;
	struct crypto_flow	*flow = NULL;

	/* TODO: Get current buffer size */
	bpool_buff_len = larg->cmn_args.garg->cmn_args.mtu + 64;

	memset(sam_descs, 0, sizeof(sam_descs));

	/* Only DEFAULT_FLOW mode is supported */
	flow = &larg->def_flows[rx_ppio_id];
	if ((flow->crypto_params->dir == CRYPTO_ENC) || (flow->crypto_params->dir == CRYPTO_LB)) {
		cio = larg->enc_cio;
		sa = flow->enc_sa;
		state = PKT_STATE_ENC;
	} else {
		cio = larg->dec_cio;
		sa = flow->dec_sa;
		state = PKT_STATE_DEC;
	}
	for (i = 0; i < num; i++) {
		char *vaddr;
		struct pp2_bpool *bpool;

		vaddr = (char *)((uintptr_t)pp2_ppio_inq_desc_get_cookie(&descs[i]));
		bpool = pp2_ppio_inq_desc_get_bpool(&descs[i], pp2_args->lcl_ports_desc[rx_ppio_id].ppio);

		mdata = (struct pkt_mdata *)mv_stack_pop(larg->stack_hndl);
		if (!mdata) {
			printf("Can't get mdata from stack for packet %d of %d\n", i, num);
			break;
		}
		mdata->rx_port = rx_ppio_id;
		mdata->bpool = bpool;
		mdata->buf_vaddr = vaddr;

		/* Set vaddr and paddr to MAC address of the packet */
		vaddr += MVAPPS_PP2_PKT_DEF_EFEC_OFFS;

		mdata->state = state;
		mdata->tx_port = flow->txp;
		mdata->flow = flow;

		src_buf_infs[i].vaddr = (char *)((uintptr_t)vaddr);
		src_buf_infs[i].paddr = pp2_ppio_inq_desc_get_phys_addr(&descs[i]) + MVAPPS_PP2_PKT_DEF_EFEC_OFFS;

		/* Exclude MH from packet length */
		src_buf_infs[i].len = pp2_ppio_inq_desc_get_pkt_len(&descs[i]);

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
		dst_buf_infs[i].len = bpool_buff_len;

		if (flow->crypto_params->crypto_proto == SAM_PROTO_NONE) {
			enum pp2_inq_l4_type type;
			u32 block_size, pad_size;

			/* SAM_PROTO_NONE: data_offs - L4 offset */
			pp2_ppio_inq_desc_get_l4_info(&descs[i], &type, &data_offs);
			mdata->data_offs = data_offs;

			sam_descs[i].sa = sa;
			sam_descs[i].cookie = mdata;

			sam_descs[i].num_bufs = 1;
			sam_descs[i].src = &src_buf_infs[i];
			sam_descs[i].dst = &dst_buf_infs[i];
			sam_descs[i].cipher_iv = rfc3602_aes128_cbc_t1_iv;

			sam_descs[i].cipher_offset = data_offs;
			sam_descs[i].cipher_len = src_buf_infs[i].len - sam_descs[i].cipher_offset;

			/* cipher_len must be block size aligned. Block size is always power of 2 */
			block_size = app_sam_cipher_block_size(flow->crypto_params->cipher_alg);
			if (block_size && (sam_descs[i].cipher_len & (block_size - 1))) {
				pad_size = block_size - (sam_descs[i].cipher_len & (block_size - 1));
				/* clear padding data */
				memset(src_buf_infs[i].vaddr + sam_descs[i].cipher_offset + sam_descs[i].cipher_len,
				       0, pad_size);
				sam_descs[i].cipher_len += pad_size;
#ifdef CRYPT_APP_VERBOSE_DEBUG
				if (larg->cmn_args.verbose > 1)
					pr_info("%s: cipher_len after padding = %d bytes, pad_size = %d bytes\n",
						__func__, sam_descs[i].cipher_len, pad_size);
#endif
			}
			if (flow->crypto_params->auth_alg != SAM_AUTH_NONE) {
				if (state == PKT_STATE_DEC)
					sam_descs[i].cipher_len -= ICV_LEN;

				sam_descs[i].auth_len = sam_descs[i].cipher_len;
				sam_descs[i].auth_offset = sam_descs[i].cipher_offset;
				sam_descs[i].auth_icv_offset = sam_descs[i].auth_offset + sam_descs[i].auth_len;
			}
		} else if (flow->crypto_params->crypto_proto == SAM_PROTO_IPSEC) {
			enum pp2_inq_l3_type type;

			/* SAM_PROTO_IPsec: data_offs - L3 offset */
			pp2_ppio_inq_desc_get_l3_info(&descs[i], &type, &data_offs);
			mdata->data_offs = data_offs;

			ipsec_descs[i].sa = sa;
			ipsec_descs[i].cookie = mdata;

			ipsec_descs[i].num_bufs = 1;
			ipsec_descs[i].src = &src_buf_infs[i];
			ipsec_descs[i].dst = &dst_buf_infs[i];

			ipsec_descs[i].l3_offset = data_offs;
			ipsec_descs[i].pkt_size = src_buf_infs[i].len;
		} else if (flow->crypto_params->crypto_proto == SAM_PROTO_SSLTLS) {
			enum pp2_inq_l3_type type;

			/* SAM_PROTO_IPsec: data_offs - L3 offset */
			pp2_ppio_inq_desc_get_l3_info(&descs[i], &type, &data_offs);
			mdata->data_offs = data_offs;

			ssltls_descs[i].sa = sa;
			ssltls_descs[i].cookie = mdata;

			ssltls_descs[i].num_bufs = 1;
			ssltls_descs[i].src = &src_buf_infs[i];
			ssltls_descs[i].dst = &dst_buf_infs[i];

			ssltls_descs[i].l3_offset = data_offs;
			ssltls_descs[i].pkt_size = src_buf_infs[i].len;
			ssltls_descs[i].type = SAM_DTLS_DATA;
		} else {
			pr_err("Unknown crypto_proto = %d\n", flow->crypto_params->crypto_proto);
			return -EFAULT;
		}
	}
	num_got = i;

	START_COUNT_CYCLES(pme_ev_cnt_enq);
	if (flow->crypto_params->crypto_proto == SAM_PROTO_NONE)
		err = sam_cio_enq(cio, sam_descs, &num_got);
	else if (flow->crypto_params->crypto_proto == SAM_PROTO_IPSEC)
		err = sam_cio_enq_ipsec(cio, ipsec_descs, &num_got);
	else
		err = sam_cio_enq_ssltls(cio, ssltls_descs, &num_got);
	STOP_COUNT_CYCLES(pme_ev_cnt_enq, num_got);

#ifdef CRYPT_APP_STATS_SUPPORT
	larg->stats.rx_pkts[rx_ppio_id] += num;
	larg->stats.rx_drop[rx_ppio_id] += num - i;

	if (cio == larg->enc_cio) {
		larg->stats.enc_enq += num_got;
		larg->stats.enc_drop += (i - num_got);
	} else {
		larg->stats.enc_enq += num_got;
		larg->stats.enc_drop += (i - num_got);
	}
#endif /* CRYPT_APP_STATS_SUPPORT*/

	if (unlikely(err)) {
		pr_err("SAM enqueue failed (%d)!\n", err);
		return -EFAULT;
	}
	if (num_got < num) {
		struct pp2_bpool	*bpool;
		struct pp2_buff_inf	 binf;

		for (i = num_got; i < num; i++) {
			binf.addr = pp2_ppio_inq_desc_get_phys_addr(&descs[i]);
			binf.cookie = pp2_ppio_inq_desc_get_cookie(&descs[i]);
			bpool = pp2_ppio_inq_desc_get_bpool(&descs[i], pp2_args->lcl_ports_desc[rx_ppio_id].ppio);
			pp2_bpool_put_buff(pp2_args->hif, bpool, &binf);
			if (flow->crypto_params->crypto_proto == SAM_PROTO_NONE)
				mdata = sam_descs[i].cookie;
			else if (flow->crypto_params->crypto_proto == SAM_PROTO_IPSEC)
				mdata = ipsec_descs[i].cookie;
			else
				mdata = ssltls_descs[i].cookie;

			if (mdata)
				mv_stack_push(larg->stack_hndl, mdata);
		}
		/*pr_warn("%s: %d packets dropped\n", __func__, num - num_got);*/
		perf_cntrs->drop_cnt += (num - num_got);
	}
	return 0;
}

static inline int dec_pkts(struct local_arg		*larg,
			   struct sam_cio_op_result	*sam_res_descs,
			   u16				 num)
{
	struct sam_cio_op_params sam_descs[CRYPT_APP_MAX_BURST_SIZE];
	struct sam_cio_ipsec_params ipsec_descs[CRYPT_APP_MAX_BURST_SIZE];
	struct sam_cio_ssltls_params ssltls_descs[CRYPT_APP_MAX_BURST_SIZE];
	struct sam_buf_info	 src_buf_infs[CRYPT_APP_MAX_BURST_SIZE];
	struct sam_buf_info	 dst_buf_infs[CRYPT_APP_MAX_BURST_SIZE];
	struct crypto_flow	*flow;
	struct pkt_mdata	*mdata;
	int			 err;
	u16			 i, bpool_buff_len, num_got;
	u8			 data_offs;
	struct perf_cmn_cntrs	*perf_cntrs = &larg->cmn_args.perf_cntrs;

	/* TODO: is this enough?!?!?! */
	bpool_buff_len = larg->cmn_args.garg->cmn_args.mtu + 64;

	memset(sam_descs, 0, sizeof(sam_descs));

	/* For decryption crypto parameters are caclulated from cookie and out_len */
	for (i = 0; i < num; i++) {
		mdata = sam_res_descs[i].cookie;
		flow = mdata->flow;
		data_offs = mdata->data_offs;

		src_buf_infs[i].vaddr = (char *)mdata->buf_vaddr + MVAPPS_PP2_PKT_DEF_EFEC_OFFS;
		src_buf_infs[i].paddr = mv_sys_dma_mem_virt2phys(src_buf_infs[i].vaddr);
		src_buf_infs[i].len = sam_res_descs[i].out_len;

		dst_buf_infs[i].vaddr = src_buf_infs[i].vaddr;
		dst_buf_infs[i].paddr = src_buf_infs[i].paddr;
		dst_buf_infs[i].len = bpool_buff_len;

		if (flow->crypto_params->crypto_proto == SAM_PROTO_NONE) {
			sam_descs[i].cookie = mdata;
			sam_descs[i].sa = flow->dec_sa;
			sam_descs[i].num_bufs = 1;
			sam_descs[i].src = &src_buf_infs[i];
			sam_descs[i].dst = &dst_buf_infs[i];
			sam_descs[i].cipher_iv = rfc3602_aes128_cbc_t1_iv;
			sam_descs[i].cipher_offset = data_offs;
			sam_descs[i].cipher_len = src_buf_infs[i].len - sam_descs[i].cipher_offset;
			if (flow->crypto_params->auth_alg != SAM_AUTH_NONE) {
				sam_descs[i].cipher_len -= ICV_LEN;
				sam_descs[i].auth_len = sam_descs[i].cipher_len;
				sam_descs[i].auth_offset = sam_descs[i].cipher_offset;
				sam_descs[i].auth_icv_offset = sam_descs[i].auth_offset + sam_descs[i].auth_len;
			}
		} else if (flow->crypto_params->crypto_proto == SAM_PROTO_IPSEC) {
			ipsec_descs[i].sa = flow->dec_sa;
			ipsec_descs[i].cookie = mdata;

			ipsec_descs[i].num_bufs = 1;
			ipsec_descs[i].src = &src_buf_infs[i];
			ipsec_descs[i].dst = &dst_buf_infs[i];
			ipsec_descs[i].l3_offset = data_offs;
			ipsec_descs[i].pkt_size = src_buf_infs[i].len;
		} else if (flow->crypto_params->crypto_proto == SAM_PROTO_SSLTLS) {
			ssltls_descs[i].sa = flow->dec_sa;
			ssltls_descs[i].cookie = mdata;

			ssltls_descs[i].num_bufs = 1;
			ssltls_descs[i].src = &src_buf_infs[i];
			ssltls_descs[i].dst = &dst_buf_infs[i];

			ssltls_descs[i].l3_offset = data_offs;
			ssltls_descs[i].pkt_size = src_buf_infs[i].len;
			ssltls_descs[i].type = SAM_DTLS_DATA;
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
	if (flow->crypto_params->crypto_proto == SAM_PROTO_NONE)
		err = sam_cio_enq(larg->dec_cio, sam_descs, &num_got);
	else if (flow->crypto_params->crypto_proto == SAM_PROTO_IPSEC)
		err = sam_cio_enq_ipsec(larg->dec_cio, ipsec_descs, &num_got);
	else
		err = sam_cio_enq_ssltls(larg->dec_cio, ssltls_descs, &num_got);
	STOP_COUNT_CYCLES(pme_ev_cnt_enq, num_got);

	if (unlikely(err)) {
		pr_err("SAM EnQ (DeC) failed (%d)!\n", err);
		return -EFAULT;
	}

	CRYPT_STATS(larg->stats.dec_enq += num);
	CRYPT_STATS(larg->stats.dec_drop += num - num_got);

	if (num_got < num) {
		for (i = num_got; i < num; i++) {
			if (flow->crypto_params->crypto_proto == SAM_PROTO_NONE)
				free_buf_from_sam_cookie(larg, sam_descs[i].cookie);
			else if (flow->crypto_params->crypto_proto == SAM_PROTO_IPSEC)
				free_buf_from_sam_cookie(larg, ipsec_descs[i].cookie);
			else
				free_buf_from_sam_cookie(larg, ssltls_descs[i].cookie);
		}
		/*pr_warn("%s: %d packets dropped\n", __func__, num - num_got);*/
		perf_cntrs->drop_cnt += (num - num_got);
	}
	return 0;
}

static inline void prepare_tx_csum(struct pkt_mdata *mdata, struct pp2_ppio_desc *desc)
{
	enum pp2_outq_l4_type l4_type = PP2_OUTQ_L4_TYPE_OTHER;
	enum pp2_outq_l3_type l3_type = PP2_OUTQ_L3_TYPE_IPV4;
	int gen_l3_chk = 0, gen_l4_chk = 0;

	if (mdata->flow->flags & MDATA_FLAGS_L4_CSUM_MASK) {
		/* Recalculate L4 checksum */
		gen_l4_chk = 1;
		l4_type = PP2_OUTQ_L4_TYPE_UDP;
	}
	if (mdata->flow->flags & MDATA_FLAGS_IP4_CSUM_MASK) {
		/* Recalculate IPv4 checksum */
		gen_l3_chk = 1;
	}
	if (gen_l3_chk || gen_l4_chk) {
		pp2_ppio_outq_desc_set_proto_info(desc, l3_type, l4_type,
						  mdata->data_offs, mdata->data_offs + sizeof(struct iphdr),
						  gen_l3_chk, gen_l4_chk);
	}
}

static inline int send_pkts(struct local_arg *larg, u8 tc,
			    struct sam_cio_op_result *sam_res_descs, u16 num)
{
	struct tx_shadow_q	*shadow_q;
	struct pp2_bpool	*bpool;
	struct pp2_buff_inf	*binf;
	struct pp2_ppio_desc	*desc;
	struct pp2_ppio_desc	 descs[MVAPPS_PP2_MAX_NUM_PORTS][CRYPT_APP_MAX_BURST_SIZE];
	int			 err, shadow_q_size[MVAPPS_PP2_MAX_NUM_PORTS];
	u16			 i, tp, num_got, num_done, port_nums[MVAPPS_PP2_MAX_NUM_PORTS];
	struct pp2_lcl_common_args *pp2_args = (struct pp2_lcl_common_args *) larg->cmn_args.plat;
	struct perf_cmn_cntrs	*perf_cntrs = &larg->cmn_args.perf_cntrs;

	for (tp = 0; tp < larg->cmn_args.num_ports; tp++) {
		port_nums[tp] = 0;
		shadow_q_size[tp] = pp2_args->lcl_ports_desc[tp].shadow_q_size;
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
		bpool = mdata->bpool;
		desc = &descs[tp][port_nums[tp]];
		shadow_q = &pp2_args->lcl_ports_desc[tp].shadow_qs[tc];

		if (mdata->flow->flags & MDATA_FLAGS_IP4_SEQID_MASK) {
			/* Set SeqID to IPv4 header of the packet */
			struct iphdr *iph = (struct iphdr *)
					((char *)mdata->buf_vaddr + MVAPPS_PP2_PKT_DEF_EFEC_OFFS + mdata->data_offs);

			iph->id = htobe16(larg->seq_id[tp]++);
		}

		pp2_ppio_outq_desc_reset(desc);
		pp2_ppio_outq_desc_set_phys_addr(desc, pa);
		pp2_ppio_outq_desc_set_pkt_offset(desc, MVAPPS_PP2_PKT_DEF_EFEC_OFFS);
		pp2_ppio_outq_desc_set_pkt_len(desc, len);

		prepare_tx_csum(mdata, desc);

#ifdef CRYPT_APP_VERBOSE_DEBUG
		if (larg->cmn_args.verbose > 1) {
			printf("Sending packet (va:%p, pa 0x%08x, len %d):\n",
			       buff, (unsigned int)pa, len);
			mv_mem_dump((u8 *)buff + MVAPPS_PP2_PKT_DEF_EFEC_OFFS, len);
		}
#endif /* CRYPT_APP_VERBOSE_DEBUG */

		shadow_q->ents[shadow_q->write_ind].buff_ptr.cookie = (uintptr_t)buff;
		shadow_q->ents[shadow_q->write_ind].buff_ptr.addr = pa;
		shadow_q->ents[shadow_q->write_ind].bpool = bpool;
		shadow_q->write_ind++;
		if (shadow_q->write_ind == shadow_q_size[tp])
			shadow_q->write_ind = 0;
		port_nums[tp]++;

		/* We don't need metadata more */
		mv_stack_push(larg->stack_hndl, mdata);
	}

	for (tp = 0; tp < larg->cmn_args.num_ports; tp++) {
		struct pp2_ppio	*ppio = pp2_args->lcl_ports_desc[tp].ppio;

		num = num_got = port_nums[tp];
		shadow_q = &pp2_args->lcl_ports_desc[tp].shadow_qs[tc];

		if (num_got) {

			START_COUNT_CYCLES(pme_ev_cnt_tx);
			err = pp2_ppio_send(ppio, pp2_args->hif, tc, descs[tp], &num_got);
			STOP_COUNT_CYCLES(pme_ev_cnt_tx, num_got);
			if (err)
				return err;

			CRYPT_STATS(larg->stats.tx_pkts[tp] += num_got);
			CRYPT_STATS(larg->stats.tx_drop[tp] += (num - num_got));

#ifdef CRYPT_APP_VERBOSE_DEBUG
			if (larg->cmn_args.verbose) {
				printf("thread #%d (cpu=%d): sent %d of %d pkts on tx_port=%d, tc=%d, hw_port=%d:%d\n",
					larg->cmn_args.id, sched_getcpu(),
					num_got, num, tp, tc, ppio->pp2_id, ppio->port_id);
			}
#endif /* CRYPT_APP_VERBOSE_DEBUG */


			if (num_got < num) {
				for (i = 0; i < num - num_got; i++) {
					if (shadow_q->write_ind == 0)
						shadow_q->write_ind = shadow_q_size[tp];
					shadow_q->write_ind--;
					binf = &shadow_q->ents[shadow_q->write_ind].buff_ptr;
					if (unlikely(!binf->cookie || !binf->addr)) {
						pr_warn("TX drop: bad buff - num=%d, num_got=%d, i=%d, write=%d, read=%d\n",
							num, num_got, i, shadow_q->write_ind, shadow_q->read_ind);
						continue;
					}
					bpool = shadow_q->ents[shadow_q->write_ind].bpool;
					pp2_bpool_put_buff(pp2_args->hif, bpool, binf);

				}
				/*pr_warn("%s: %d packets dropped\n", __func__, num - num_got);*/
				perf_cntrs->drop_cnt += (num - num_got);
			}
		}
		pp2_ppio_get_num_outq_done(pp2_args->lcl_ports_desc[tp].ppio, pp2_args->hif, tc, &num_done);
		for (i = 0; i < num_done; i++) {
			binf = &shadow_q->ents[shadow_q->read_ind].buff_ptr;
			bpool = shadow_q->ents[shadow_q->read_ind].bpool;

			shadow_q->read_ind++;
			if (shadow_q->read_ind == shadow_q_size[tp])
				shadow_q->read_ind = 0;

			if (unlikely(!binf->cookie || !binf->addr)) {
				pr_warn("TX done: bad buff - num_done=%d, i=%d, write=%d, read=%d\n",
					num_done, i, shadow_q->write_ind, shadow_q->read_ind);
				continue;
			}
			pp2_bpool_put_buff(pp2_args->hif, bpool, binf);
		}
		perf_cntrs->tx_cnt += num_done;
	}
	return 0;
}

static inline int deq_crypto_pkts(struct local_arg	*larg,
				  struct sam_cio	*cio,
				  u8			 tc)
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
	CRYPT_STATS(larg->stats.deq_cnt += num);

	num_to_send = num_to_dec = 0;
	for (i = 0; i < num; i++) {
		struct pkt_mdata *mdata;

		if (unlikely(!res_descs[i].cookie)) {
			pr_err("SAM operation failed (no cookie: %d,%d)!\n", i, res_descs[i].out_len);
			perf_cntrs->drop_cnt++;
			CRYPT_STATS(larg->stats.deq_err++);
			continue;
		}

		mdata = (struct pkt_mdata *)res_descs[i].cookie;
		if (res_descs[i].status != SAM_CIO_OK) {

#ifdef CRYPT_APP_VERBOSE_DEBUG
			if (larg->cmn_args.verbose) {
				char *tmp_buff = (char *)mdata->buf_vaddr + MVAPPS_PP2_PKT_DEF_EFEC_OFFS;

				pr_warn("SAM operation (%s) %d of %d failed! status = %d, len = %d\n",
					(mdata->state == (u8)PKT_STATE_ENC) ? "EnC" : "DeC",
					i, num, res_descs[i].status, res_descs[i].out_len);

				mv_mem_dump((u8 *)tmp_buff, res_descs[i].out_len);
			}
#endif
			/* Free buffer */
			free_buf_from_sam_cookie(larg, res_descs[i].cookie);
			perf_cntrs->drop_cnt++;
			CRYPT_STATS(larg->stats.deq_err++);
			continue;
		}
		if ((mdata->flow->crypto_params->dir == CRYPTO_LB) && (mdata->state == (u8)PKT_STATE_ENC)) {
			mdata->state = (u8)PKT_STATE_DEC;
			res_descs_to_dec[num_to_dec++] = res_descs[i];
		} else {
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

		return send_pkts(larg, tc, res_descs_to_send, num_to_send);
	}
	return 0;
}

static inline int loop_sw_recycle(struct local_arg	*larg,
				  u8			 rx_ppio_id,
				  u8			 tc,
				  u8			 qid,
				  u16			 num)
{
	struct pp2_ppio_desc	 descs[CRYPT_APP_MAX_BURST_SIZE];
	struct pp2_lcl_common_args *pp2_args = (struct pp2_lcl_common_args *) larg->cmn_args.plat;
	struct perf_cmn_cntrs	*perf_cntrs = &larg->cmn_args.perf_cntrs;
	int			 err;
	struct pp2_ppio		*ppio = pp2_args->lcl_ports_desc[rx_ppio_id].ppio;

	/*pr_info("tid %d check on tc %d, qid %d\n", larg->cmn_args.id, tc, qid);*/
	START_COUNT_CYCLES(pme_ev_cnt_rx);
	err = pp2_ppio_recv(ppio, tc, qid, descs, &num);
	STOP_COUNT_CYCLES(pme_ev_cnt_rx, num);

#ifdef CRYPT_APP_VERBOSE_DEBUG
	if (larg->cmn_args.verbose && num) {
		printf("thread #%d (cpu=%d): recv %d pkts on rx_port=%d, tc=%d, qid=%d, hw_port=%d:%d\n",
			larg->cmn_args.id, sched_getcpu(),
			num, rx_ppio_id, tc, qid, ppio->pp2_id, ppio->port_id);
	}
#endif /* CRYPT_APP_VERBOSE_DEBUG */

	if (num) {
		perf_cntrs->rx_cnt += num;

		err = proc_rx_pkts(larg, rx_ppio_id, descs, num);
		if (unlikely(err))
			return err;
	}
	if (larg->enc_cio) {
		err = deq_crypto_pkts(larg, larg->enc_cio, tc);
		if (unlikely(err))
			return err;
	}

	if (larg->dec_cio && (larg->dec_cio != larg->enc_cio))
		return deq_crypto_pkts(larg, larg->dec_cio, tc);

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

/* get next tc (traffic class) and RX queue to process */
static void get_next_tc_and_rxq(struct local_arg *larg, u8 *tc, u8 *rxq)
{
	u8 lrxq, ltc;

	lrxq = *rxq;
	ltc = *tc;
	/* Find next queue to consume */
	do {
		lrxq++;
		if (lrxq == mvapp_pp2_max_num_qs_per_tc) {
			lrxq = 0;
			ltc++;
			if (ltc == CRYPT_APP_MAX_NUM_TCS_PER_PORT)
				ltc = 0;
		}
	} while (!(larg->cmn_args.qs_map & (1 << ((ltc * mvapp_pp2_max_num_qs_per_tc) + lrxq))));

	*tc = ltc;
	*rxq = lrxq;
}

static int main_loop_cb(void *arg, int *running)
{
	struct local_arg *larg = (struct local_arg *)arg;
	u8 tc, rxq;
	u16 num;
	int err;

	int i;

	if (!larg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}
	num = larg->cmn_args.burst;
	rxq = 0;
	tc = 0;
	while (*running) {
		get_next_tc_and_rxq(larg, &tc, &rxq);
		for (i = 0; i < larg->cmn_args.num_ports; i++) {

			err = loop_sw_recycle(larg, i, tc, rxq, num);
			if (err != 0)
				return err;
		}
	}
	return err;
}


static int init_all_modules(void)
{
	struct pp2_init_params	 pp2_params;
	struct pp2_glb_common_args *pp2_args = (struct pp2_glb_common_args *)garg.cmn_args.plat;

	int			 err;
	char			 file[PP2_MAX_BUF_STR_LEN];
	int			 num_rss_tables = 0;

	pr_info("Global initializations ...\n");

	err = mv_sys_dma_mem_init(CRYPT_APP_DMA_MEM_SIZE);
	if (err)
		return err;

	memset(&pp2_params, 0, sizeof(pp2_params));
	pp2_params.hif_reserved_map = pp2_get_kernel_hif_map();
	app_used_hifmap_init(pp2_params.hif_reserved_map);
	pp2_params.bm_pool_reserved_map = MVAPPS_PP2_BPOOLS_RSRV;

	/* Check how many RSS tables are in use by kernel. This parameter is needed for configuring RSS */
	/* Relevant only if cpus is bigger than 1 */
	if (garg.cmn_args.cpus > 1) {
		sprintf(file, "%s/%s", PP2_SYSFS_RSS_PATH, PP2_SYSFS_RSS_NUM_TABLES_FILE);
		num_rss_tables = appp_pp2_sysfs_param_get(pp2_args->ports_desc[0].name, file);
		if (num_rss_tables < 0) {
			pr_err("Failed to read kernel RSS tables. Please check mvpp2x_sysfs.ko is loaded\n");
			return -EFAULT;
		}
	}

	pp2_params.rss_tbl_reserved_map = (1 << num_rss_tables) - 1;

	err = pp2_init(&pp2_params);
	if (err)
		return err;

	pr_info("done\n");
	return 0;
}

static int create_sam_sessions(struct crypto_params *params, struct crypto_flow *flow)
{
	struct sam_session_params	 sa_params;
	int				 err;

	memset(&sa_params, 0, sizeof(sa_params));

	sa_params.proto = params->crypto_proto;
	sa_params.cipher_alg = params->cipher_alg;  /* cipher algorithm */
	sa_params.cipher_mode = params->cipher_mode; /* cipher mode */
	sa_params.cipher_iv = NULL;     /* default IV */
	sa_params.cipher_key = params->cipher_key;    /* cipher key */
	sa_params.cipher_key_len = params->cipher_key_len; /* cipher key size (in bytes) */

	sa_params.auth_alg = params->auth_alg; /* authentication algorithm */
	if (sa_params.auth_alg != SAM_AUTH_NONE) {
		sa_params.auth_key = params->auth_key;    /* cipher key */
		sa_params.auth_key_len = params->auth_key_len; /* cipher key size (in bytes) */
	} else {
		sa_params.auth_key = NULL;    /* auth key */
		sa_params.auth_key = 0;
	}
	if (params->crypto_proto == SAM_PROTO_IPSEC) {
		sa_params.u.ipsec.is_esp = 1;
		sa_params.u.ipsec.is_natt = 0;
		sa_params.u.ipsec.spi = t1_spi;
		sa_params.u.ipsec.seq = t1_seq;
		sa_params.u.ipsec.is_esn = params->seq64;

		if (params->tunnel) {
			sa_params.u.ipsec.is_tunnel = 1;
			if (params->ip6) {
				sa_params.u.ipsec.is_ip6 = 1;
				sa_params.u.ipsec.tunnel.u.ipv6.sip = tunnel_src_ip6;
				sa_params.u.ipsec.tunnel.u.ipv6.dip = tunnel_dst_ip6;
				sa_params.u.ipsec.tunnel.u.ipv6.dscp = 0;
				sa_params.u.ipsec.tunnel.u.ipv6.hlimit = 64;
				sa_params.u.ipsec.tunnel.u.ipv6.flabel = 0;
			} else {
				sa_params.u.ipsec.is_ip6 = 0;
				sa_params.u.ipsec.tunnel.u.ipv4.sip = tunnel_src_ip4;
				sa_params.u.ipsec.tunnel.u.ipv4.dip = tunnel_dst_ip4;
				sa_params.u.ipsec.tunnel.u.ipv4.dscp = 0;
				sa_params.u.ipsec.tunnel.u.ipv4.ttl = 64;
				sa_params.u.ipsec.tunnel.u.ipv4.df = 0;
			}
			sa_params.u.ipsec.tunnel.copy_flabel = 0;
			sa_params.u.ipsec.tunnel.copy_dscp = 0;
			sa_params.u.ipsec.tunnel.copy_df = 0;
		}
	} else if (params->crypto_proto == SAM_PROTO_SSLTLS) {
		if (params->ip6)
			sa_params.u.ssltls.is_ip6 = 1;
		sa_params.u.ssltls.is_capwap = params->capwap;
		sa_params.u.ssltls.is_udp_lite = 0;
		sa_params.u.ssltls.version = params->ssl_version;
		sa_params.u.ssltls.seq = ssl_seq;
		sa_params.u.ssltls.epoch = dtls_epoch;
	} else {
		sa_params.u.basic.auth_aad_len = 0;   /* Additional Data (AAD) size (in bytes) */
		if (sa_params.auth_alg != SAM_AUTH_NONE)
			sa_params.u.basic.auth_icv_len = ICV_LEN;
		else
			sa_params.u.basic.auth_icv_len = 0;
	}

	sa_params.dir = SAM_DIR_ENCRYPT;   /* operation direction: encode */
	err = sam_session_create(&sa_params, &flow->enc_sa);
	if (err) {
		pr_err("EnC SA creation failed (%d)!\n", err);
		return err;
	}
	if (!flow->enc_sa) {
		pr_err("EnC SA creation failed!\n");
		return -EFAULT;
	}

	sa_params.dir = SAM_DIR_DECRYPT;   /* operation direction: decode */
	err = sam_session_create(&sa_params, &flow->dec_sa);
	if (err) {
		pr_err("DeC SA creation failed (%d)!\n", err);
		return err;
	}
	if (!flow->dec_sa) {
		pr_err("DeC SA creation failed!\n");
		return -EFAULT;
	}
	flow->crypto_params = params;

	return 0;
}

static void destroy_sam_sessions(struct local_arg *larg)
{
	int i;

	/* Destroy flow sessions - TBD */

	/* Destroy default sessions */
	for (i = 0; i < larg->cmn_args.num_ports; i++) {
		struct crypto_flow *flow = &larg->def_flows[i];

		if (flow->enc_sa) {
			if (sam_session_destroy(flow->enc_sa))
				pr_err("EnC SA destroy failed!\n");
		}

		if (flow->dec_sa) {
			if (sam_session_destroy(flow->dec_sa))
				pr_err("DeC SA destroy failed!\n");
		}

		if (larg->enc_cio) {
			if (sam_cio_flush(larg->enc_cio))
				pr_err("EnC CIO flush failed!\n");
		}

		if (larg->dec_cio && (larg->enc_cio != larg->dec_cio)) {
			if (sam_cio_flush(larg->dec_cio))
				pr_err("DeC CIO flush failed!\n");
		}
	}
}

static int init_local_modules(struct glob_arg *garg)
{
	struct sam_init_params	init_params;
	int			err, port_index;
	struct pp2_glb_common_args *pp2_args = (struct pp2_glb_common_args *) garg->cmn_args.plat;
	struct bpool_inf	std_infs[] = CRYPT_APP_BPOOLS_INF;
	struct bpool_inf	jumbo_infs[] = CRYPT_APP_BPOOLS_JUMBO_INF;
	struct bpool_inf	*infs;
	int			i;

	pr_info("Specific modules initializations\n");

	err = app_hif_init(&pp2_args->hif, CRYPT_APP_HIF_Q_SIZE);
	if (err)
		return err;

	if (garg->cmn_args.mtu > DEFAULT_MTU) {
		infs = jumbo_infs;
		pp2_args->num_pools = ARRAY_SIZE(jumbo_infs);
	} else {
		infs = std_infs;
		pp2_args->num_pools = ARRAY_SIZE(std_infs);
	}
	/* Calculate total number of buffers in the pools */
	garg->num_bufs = 0;
	for (i = 0; i < pp2_args->num_pools; i++)
		garg->num_bufs += infs[i].num_buffs;

	err = app_build_all_bpools(&pp2_args->pools_desc, pp2_args->num_pools, infs, pp2_args->hif);
	if (err)
		return err;

	for (port_index = 0; port_index < garg->cmn_args.num_ports; port_index++) {
		struct port_desc *port = &pp2_args->ports_desc[port_index];

		err = app_find_port_info(port);
		if (!err) {
			port->ppio_type	= PP2_PPIO_T_NIC;
			port->num_tcs	= CRYPT_APP_MAX_NUM_TCS_PER_PORT;
			for (i = 0; i < port->num_tcs; i++)
				port->num_inqs[i] = garg->cmn_args.cpus;
			port->inq_size	= CRYPT_APP_RX_Q_SIZE;
			port->num_outqs	= CRYPT_APP_MAX_NUM_TCS_PER_PORT;
			port->outq_size	= CRYPT_APP_TX_Q_SIZE;
			port->first_inq	= CRYPT_APP_FIRST_INQ;
			if (garg->cmn_args.cpus == 1)
				port->hash_type = PP2_PPIO_HASH_T_NONE;
			else
				port->hash_type = PP2_PPIO_HASH_T_2_TUPLE;

			/* pkt_offset=0 not to be changed, before recoding rx_data_path to use pkt_offset as well */
			err = app_port_init(port, pp2_args->num_pools,
					    pp2_args->pools_desc[port->pp_id], garg->cmn_args.mtu, 0);
			if (err) {
				pr_err("Failed to initialize port %d (pp_id: %d)\n", port_index,
				       port->pp_id);
				return err;
			}
		} else {
			return err;
		}
	}

	/* Sam driver global initializations
	 */
	init_params.max_num_sessions = CRYPT_APP_MAX_NUM_SESSIONS;
	sam_init(&init_params);

	return 0;
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

#ifdef MVCONF_SAM_STATS
	app_sam_show_stats(1);
#endif
	sam_deinit();

	if (garg->free_cios)
		free(garg->free_cios);

	apps_pp2_deinit_global(garg);
}

static int init_local(void *arg, int id, void **_larg)
{
	struct glob_arg		*garg = (struct glob_arg *)arg;
	struct local_arg	*larg;
	struct pp2_glb_common_args *glb_pp2_args = (struct pp2_glb_common_args *) garg->cmn_args.plat;
	struct pp2_lcl_common_args *lcl_pp2_args;
	struct sam_cio_params	cio_params;
	struct sam_cio		*cio;
	int			 i, err, cio_id, sam_device;

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

	larg->cmn_args.plat = (struct pp2_lcl_common_args *)malloc(sizeof(struct pp2_lcl_common_args));
	if (!larg) {
		pr_err("No mem for local plat arg obj!\n");
		free(larg);
		return -ENOMEM;
	}

	lcl_pp2_args = (struct pp2_lcl_common_args *) larg->cmn_args.plat;
	larg->cmn_args.num_ports	= garg->cmn_args.num_ports;

	lcl_pp2_args->lcl_ports_desc = (struct lcl_port_desc *)
					   malloc(larg->cmn_args.num_ports * sizeof(struct lcl_port_desc));
	if (!lcl_pp2_args->lcl_ports_desc) {
		pr_err("no mem for local-port-desc obj!\n");
		free(larg->cmn_args.plat);
		free(larg);
		return -ENOMEM;
	}
	memset(lcl_pp2_args->lcl_ports_desc, 0, larg->cmn_args.num_ports * sizeof(struct lcl_port_desc));

	pthread_mutex_lock(&garg->trd_lock);
	err = app_hif_init(&lcl_pp2_args->hif, CRYPT_APP_HIF_Q_SIZE);
	pthread_mutex_unlock(&garg->trd_lock);
	if (err)
		return err;

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
	if (garg->def_crypto_params.dir == CRYPTO_ENC)
		larg->enc_cio = cio;
	else if (garg->def_crypto_params.dir == CRYPTO_DEC)
		larg->dec_cio = cio;
	else if (garg->def_crypto_params.dir == CRYPTO_LB) {
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
	larg->cmn_args.garg		= garg;

	for (i = 0; i < larg->cmn_args.num_ports; i++)
		app_port_local_init(i, larg->cmn_args.id,
				    &lcl_pp2_args->lcl_ports_desc[i], &glb_pp2_args->ports_desc[i]);

	lcl_pp2_args->pools_desc = glb_pp2_args->pools_desc;

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

	/* Create default sessions */
	for (i = 0; i < larg->cmn_args.num_ports; i++) {
		u8 tx_port;
		struct crypto_flow *flow = &larg->def_flows[i];

		err = create_sam_sessions(&garg->def_crypto_params, flow);
		if (err)
			return err;

		/* Set flow flags */
		if (flow->crypto_params->crypto_proto == SAM_PROTO_SSLTLS) {
			/* packets must be IPv4/IPv6 + UDP/UDPLite */
			flow->flags = MDATA_FLAGS_IP4_CSUM_MASK | MDATA_FLAGS_L4_CSUM_MASK;
		} else if (flow->crypto_params->crypto_proto == SAM_PROTO_IPSEC) {
			if ((flow->crypto_params->dir == CRYPTO_ENC) &&
			     flow->crypto_params->tunnel &&
			     !flow->crypto_params->ip6) {
				flow->flags = MDATA_FLAGS_IP4_CSUM_MASK | MDATA_FLAGS_IP4_SEQID_MASK;
			}
		}
		/* Use next port as TX */
		tx_port = i + 1;
		if (tx_port == larg->cmn_args.num_ports)
			tx_port = 0;

		flow->txp = tx_port;
	}

	garg->cmn_args.largs[id] = larg;

	larg->cmn_args.qs_map = garg->cmn_args.qs_map << (garg->cmn_args.qs_map_shift * id);

	pr_info("Local thread #%d (cpu #%d): Encrypt - %s, Decrypt - %s, qs_map = 0x%lx\n",
		id, sched_getcpu(), larg->enc_name,
		larg->dec_cio ? larg->dec_name : "None", larg->cmn_args.qs_map);

	*_larg = larg;
	return 0;
}

static void deinit_local(void *arg)
{
	struct local_arg *larg = (struct local_arg *)arg;
	struct pp2_lcl_common_args *pp2_args = (struct pp2_lcl_common_args *) larg->cmn_args.plat;
	struct pkt_mdata *mdata;
	int i;

	if (!larg)
		return;

	destroy_sam_sessions(larg);

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

	if (pp2_args->lcl_ports_desc) {
		for (i = 0; i < larg->cmn_args.num_ports; i++)
			app_port_local_deinit(&pp2_args->lcl_ports_desc[i]);
		free(pp2_args->lcl_ports_desc);
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

	if (pp2_args->hif)
		pp2_hif_deinit(pp2_args->hif);
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
	       "\t--crypto-proto <proto>   Crypto protocol. Support: [none, esp, ssl]. (default: none).\n"
	       "\t--tunnel                 IPSec tunnel mode. (default: transport)\n"
	       "\t--seq64                  Use 64-bits extended sequence number. (default: 32-bits)\n"
	       "\t--capwap                 DTLS with capwap mode. (default: no capwap)\n"
	       "\t--ssl_version <ver>      SSL/TLS version. (default: dtls_1_0)\n"
	       "\t--ip6                    ESP/SSL over IPv6. (default: ESP/SSL over IPv4)\n"
	       "\t--cipher-alg   <alg>     Cipher algorithm. Support: [none, aes128, 3des]. (default: aes128).\n"
	       "\t--cipher-mode  <alg>     Cipher mode. Support: [cbc, ecb]. (default: cbc).\n"
	       "\t--auth-alg     <alg>     Authentication algorithm. Support: [none, sha1]. (default: sha1).\n"
	       "\t--dir          <dir>     Operation direction. Support: [enc, dec, lb]. (default: lb)\n"
	       "\t--no-echo                No Echo packets\n"
	       "\t--cli                    Use CLI\n"
	       "\t?, -h, --help            Display help and exit.\n\n"
	       "\n", MVAPPS_NO_PATH(progname), MVAPPS_NO_PATH(progname),
	       MVAPPS_PP2_MAX_NUM_PORTS, CRYPT_APP_MAX_BURST_SIZE, DEFAULT_MTU);
}

static int parse_args(struct glob_arg *garg, int argc, char *argv[])
{
	int	i = 1;
	struct pp2_glb_common_args *pp2_args = (struct pp2_glb_common_args *) garg->cmn_args.plat;

	garg->cmn_args.verbose = 0;
	garg->cmn_args.cli = 0;
	garg->cmn_args.cpus = 1;
	garg->cmn_args.affinity = MVAPPS_INVALID_AFFINITY;
	garg->cmn_args.burst = CRYPT_APP_DFLT_BURST_SIZE;
	garg->cmn_args.mtu = DEFAULT_MTU;
	garg->cmn_args.echo = 1;
	garg->flow_mode = DEFAULT_FLOW;
	garg->def_crypto_params.dir = CRYPTO_LB;
	garg->def_crypto_params.tunnel = 0;
	garg->def_crypto_params.seq64 = 0;
	garg->def_crypto_params.ip6 = 0;
	garg->def_crypto_params.capwap = 0;
	garg->def_crypto_params.ssl_version = SAM_DTLS_VERSION_1_0;
	garg->def_crypto_params.crypto_proto = SAM_PROTO_NONE;
	garg->def_crypto_params.cipher_alg = SAM_CIPHER_AES;
	garg->def_crypto_params.cipher_mode = SAM_CIPHER_CBC;
	garg->def_crypto_params.cipher_key = rfc3602_aes128_cbc_t1_key;
	garg->def_crypto_params.cipher_key_len = sizeof(rfc3602_aes128_cbc_t1_key);
	garg->def_crypto_params.auth_alg = SAM_AUTH_HMAC_SHA1;
	garg->def_crypto_params.auth_key = rfc3602_sha1_t1_auth_key;
	garg->def_crypto_params.auth_key_len = sizeof(rfc3602_sha1_t1_auth_key);
	garg->cmn_args.qs_map = 0;
	garg->cmn_args.qs_map_shift = 0;
	garg->cmn_args.pkt_offset = 0;
	garg->cmn_args.prefetch_shift = CRYPT_APP_PREFETCH_SHIFT;
	pp2_args->pp2_num_inst = pp2_get_num_inst();
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
				snprintf(pp2_args->ports_desc[garg->cmn_args.num_ports].name,
					 sizeof(pp2_args->ports_desc[garg->cmn_args.num_ports].name),
					 "%s", token);
			}

			if (garg->cmn_args.num_ports == 0) {
				pr_err("Invalid interface arguments format!\n");
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
		} else if (strcmp(argv[i], "--tunnel") == 0) {
			garg->def_crypto_params.tunnel = 1;
			i += 1;
		} else if (strcmp(argv[i], "--ip6") == 0) {
			garg->def_crypto_params.ip6 = 1;
			i += 1;
		} else if (strcmp(argv[i], "--seq64") == 0) {
			garg->def_crypto_params.seq64 = 1;
			i += 1;
		} else if (strcmp(argv[i], "--capwap") == 0) {
			garg->def_crypto_params.capwap = 1;
			i += 1;
		} else if (strcmp(argv[i], "--dir") == 0) {
			if (strcmp(argv[i+1], "enc") == 0)
				garg->def_crypto_params.dir = CRYPTO_ENC;
			else if (strcmp(argv[i+1], "dec") == 0)
				garg->def_crypto_params.dir = CRYPTO_DEC;
			else if (strcmp(argv[i+1], "lb") == 0)
				garg->def_crypto_params.dir = CRYPTO_LB;
			else {
				pr_err("Direction (%s) not supported!\n", argv[i+1]);
				return -EINVAL;
			}
			i += 2;
		} else if (strcmp(argv[i], "--crypto-proto") == 0) {
			if (strcmp(argv[i+1], "esp") == 0)
				garg->def_crypto_params.crypto_proto = SAM_PROTO_IPSEC;
			else if (strcmp(argv[i+1], "ssl") == 0)
				garg->def_crypto_params.crypto_proto = SAM_PROTO_SSLTLS;
			else if (strcmp(argv[i+1], "none") == 0)
				garg->def_crypto_params.crypto_proto = SAM_PROTO_NONE;
			else {
				pr_err("Crypto protocol (%s) not supported!\n", argv[i+1]);
				return -EINVAL;
			}
			i += 2;
		} else if (strcmp(argv[i], "--cipher-alg") == 0) {
			if (strcmp(argv[i+1], "aes128") == 0) {
				garg->def_crypto_params.cipher_alg = SAM_CIPHER_AES;
				garg->def_crypto_params.cipher_key = rfc3602_aes128_cbc_t1_key;
				garg->def_crypto_params.cipher_key_len = sizeof(rfc3602_aes128_cbc_t1_key);
			} else if (strcmp(argv[i+1], "3des") == 0) {
				garg->def_crypto_params.cipher_alg = SAM_CIPHER_3DES;
				garg->def_crypto_params.cipher_key = rfc3602_3des_cbc_t1_key;
				garg->def_crypto_params.cipher_key_len = sizeof(rfc3602_3des_cbc_t1_key);
			} else if (strcmp(argv[i+1], "none") == 0) {
				garg->def_crypto_params.cipher_alg = SAM_CIPHER_NONE;
				garg->def_crypto_params.cipher_key = NULL;
				garg->def_crypto_params.cipher_key_len = 0;
			} else {
				pr_err("Cipher alg (%s) not supported!\n", argv[i+1]);
				return -EINVAL;
			}
			i += 2;
		} else if (strcmp(argv[i], "--cipher-mode") == 0) {
			if (strcmp(argv[i+1], "cbc") == 0)
				garg->def_crypto_params.cipher_alg = SAM_CIPHER_CBC;
			else if (strcmp(argv[i+1], "ecb") == 0)
				garg->def_crypto_params.cipher_alg = SAM_CIPHER_ECB;
			else {
				pr_err("Cipher mode (%s) not supported!\n", argv[i+1]);
				return -EINVAL;
			}
			i += 2;
		} else if (strcmp(argv[i], "--auth-alg") == 0) {
			if (strcmp(argv[i+1], "sha1") == 0) {
				garg->def_crypto_params.auth_alg = SAM_AUTH_HMAC_SHA1;
				garg->def_crypto_params.auth_key = rfc3602_sha1_t1_auth_key;
				garg->def_crypto_params.auth_key_len = sizeof(rfc3602_sha1_t1_auth_key);
			} else if (strcmp(argv[i+1], "sha256") == 0) {
				garg->def_crypto_params.auth_alg = SAM_AUTH_HMAC_SHA2_256;
				garg->def_crypto_params.auth_key = rfc3602_sha1_t1_auth_key;
				garg->def_crypto_params.auth_key_len = sizeof(rfc3602_sha1_t1_auth_key);
			} else if (strcmp(argv[i+1], "md5") == 0) {
				garg->def_crypto_params.auth_alg = SAM_AUTH_HMAC_MD5;
				garg->def_crypto_params.auth_key = md5_auth_key;
				garg->def_crypto_params.auth_key_len = sizeof(md5_auth_key);
			} else if (strcmp(argv[i+1], "none") == 0) {
				garg->def_crypto_params.auth_alg = SAM_AUTH_NONE;
				garg->def_crypto_params.auth_key = NULL;
				garg->def_crypto_params.auth_key_len = 0;
			} else {
				pr_err("Auth alg (%s) not supported!\n", argv[i+1]);
				return -EINVAL;
			}
			i += 2;
		} else if (strcmp(argv[i], "--ssl-version") == 0) {
			if (strcmp(argv[i+1], "dtls_1_0") == 0)
				garg->def_crypto_params.ssl_version = SAM_DTLS_VERSION_1_0;
			else if (strcmp(argv[i+1], "dtls_1_2") == 0)
				garg->def_crypto_params.ssl_version = SAM_DTLS_VERSION_1_2;
			else {
				pr_err("SSL version (%s) not supported!\n", argv[i+1]);
				return -EINVAL;
			}
			i += 2;
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
	    !pp2_args->ports_desc[0].name) {
		pr_err("No port defined!\n");
		return -EINVAL;
	}
	if (garg->cmn_args.burst > CRYPT_APP_MAX_BURST_SIZE) {
		pr_err("illegal burst size requested (%d vs %d)!\n",
		       garg->cmn_args.burst, CRYPT_APP_MAX_BURST_SIZE);
		return -EINVAL;
	}
	if (garg->cmn_args.cpus > system_ncpus()) {
		pr_err("illegal num cores requested (%d vs %d)!\n",
		       garg->cmn_args.cpus, system_ncpus());
		return -EINVAL;
	}
	if ((garg->cmn_args.affinity != -1) &&
	    ((garg->cmn_args.cpus + garg->cmn_args.affinity) > system_ncpus())) {
		pr_err("illegal num cores or affinity requested (%d,%d vs %d)!\n",
		       garg->cmn_args.cpus, garg->cmn_args.affinity, system_ncpus());
		return -EINVAL;
	}

	if (garg->cmn_args.qs_map &&
	    (mvapp_pp2_max_num_qs_per_tc == 1) &&
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
	pr_info("affinity      : %d\n", garg->cmn_args.affinity);
	pr_info("ports         : %d\n", garg->cmn_args.num_ports);
	pr_info("burst         : %d\n", garg->cmn_args.burst);

	pr_info("direction     : %d\n", garg->def_crypto_params.dir);
	pr_info("swap addr     : %s\n", garg->cmn_args.echo ? "Swap" : "Not swap");
	pr_info("cipher-alg    : %d\n", garg->def_crypto_params.cipher_alg);
	pr_info("cipher-mode   : %d\n", garg->def_crypto_params.cipher_mode);
	pr_info("auth-alg      : %d\n", garg->def_crypto_params.auth_alg);

	if (garg->def_crypto_params.crypto_proto == SAM_PROTO_IPSEC) {
		pr_info("crypto-proto  : %s\n", "IPSec");
		pr_info("IPSec mode    : %s\n", garg->def_crypto_params.tunnel ? "Tunnel" : "Transport");
		pr_info("seq number    : %u bits\n", garg->def_crypto_params.seq64 ? 64 : 32);
	} else if (garg->def_crypto_params.crypto_proto == SAM_PROTO_SSLTLS) {
		pr_info("crypto-proto  : %s\n", "SSL/TLS");
		pr_info("SSL version   : %d\n", garg->def_crypto_params.ssl_version);
		pr_info("Capwap mode   : %s\n", garg->def_crypto_params.capwap ? "Yes" : "No");
	}
	return 0;
}

int main(int argc, char *argv[])
{
	struct mvapp_params	mvapp_params;
	u64			cores_mask;
	int			err;

	setbuf(stdout, NULL);
	app_set_max_num_qs_per_tc();

	pr_debug("pr_debug is enabled\n");

	garg.cmn_args.plat = (struct pp2_glb_common_args *)malloc(sizeof(struct pp2_glb_common_args));
	if (!garg.cmn_args.plat) {
		pr_err("No mem for global plat arg obj!\n");
		return -ENOMEM;
	}
	err = parse_args(&garg, argc, argv);
	if (err) {
		free(garg.cmn_args.plat);
		return err;
	}
	cores_mask = apps_cores_mask_create(garg.cmn_args.cpus, garg.cmn_args.affinity);

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
