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
#include <string.h>
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

#include "mv_neta.h"
#include "mv_neta_bpool.h"
#include "mv_neta_ppio.h"
#include "mv_sam.h"

#include "../neta_utils.h"
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
#define CRYPT_APP_CTRL_DFLT_THR		2000
#define CRYPT_APP_DMA_MEM_SIZE		(48 * 1024 * 1024)

#define CRYPT_APP_CIOS_RSRV		{0x0, 0x0}

#define MAX_AUTH_BLOCK_SIZE	128 /* Bytes */
#define AUTH_BLOCK_SIZE_64B	64  /* Bytes */
#define ICV_LEN			12  /* Bytes */

#define CRYPT_APP_FIRST_INQ			0
#define CRYPT_APP_MAX_NUM_TCS_PER_PORT		1
#define CRYPT_APP_MAX_NUM_QS_PER_CORE		CRYPT_APP_MAX_NUM_TCS_PER_PORT
#define CRYPT_APP_MAX_NUM_SESSIONS		32

/* TODO: find more generic way to get the following parameters */
#define CRYPT_APP_TOTAL_NUM_CIOS	4

/*#define CRYPT_APP_VERBOSE_CHECKS*/
#define CRYPT_APP_VERBOSE_DEBUG
#define CRYPT_APP_PKT_ECHO_SUPPORT
#define CRYPT_APP_PREFETCH_SHIFT	4

#define CRYPT_APP_BPOOLS_INF		{ {256, CRYPT_APP_TX_Q_SIZE * 2}, {1600, CRYPT_APP_TX_Q_SIZE} }
#define CRYPT_APP_BPOOLS_JUMBO_INF	{ {384, CRYPT_APP_TX_Q_SIZE}, {4096, CRYPT_APP_TX_Q_SIZE} }

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

/* Masks for mdata flags field */
#define MDATA_FLAGS_IP4_SEQID_MASK	BIT(0)   /* Set seqid field in IP header before send */

#define MDATA_FLAGS_IP4_CSUM_MASK	BIT(14)  /* Recalculate IPv4 checksum on TX */
#define MDATA_FLAGS_L4_CSUM_MASK	BIT(15)  /* Recalculate L4 checksum on TX */

struct pkt_mdata {
	u8 state; /* as defined in enum pkt_state */
	u8 rx_port;
	u8 tx_port;
	u8 data_offs;
	u16 flags;
	u16 reserved;
	void *buf_vaddr;
	struct neta_bpool *bpool;
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
	enum sam_crypto_protocol	crypto_proto;
	enum sam_cipher_alg		cipher_alg;
	enum sam_cipher_mode		cipher_mode;
	enum sam_auth_alg		auth_alg;
	int				tunnel;
	int				ip6;
	pthread_mutex_t			trd_lock;
	int				num_bufs;
	int				num_ports;
	struct port_desc		ports_desc[MVAPPS_NETA_MAX_NUM_PORTS];
	int				num_pools;
	struct bpool_desc		*pools_desc[MVAPPS_NETA_MAX_NUM_PORTS];

};

struct local_arg {
	struct local_common_args	cmn_args;  /* Keep first */

	struct mv_stack			*stack_hndl;
	struct sam_cio			*enc_cio;
	struct sam_sa			*enc_sa;
	u32				seq_id[MVAPPS_NETA_MAX_NUM_PORTS];
	struct sam_cio			*dec_cio;
	struct sam_sa			*dec_sa;
	enum crypto_dir			dir;
	int				num_ports;
	struct lcl_port_desc		*lcl_ports_desc;
	struct bpool_desc		*pools_desc;
};

static struct glob_arg garg = {};
static u8	used_cios[] = CRYPT_APP_CIOS_RSRV;

#define CHECK_CYCLES
#ifdef CHECK_CYCLES
static int pme_ev_cnt_rx = -1, pme_ev_cnt_enq = -1, pme_ev_cnt_deq = -1, pme_ev_cnt_tx = -1;

#define START_COUNT_CYCLES(_ev_cnt)		pme_ev_cnt_start(_ev_cnt)
#define STOP_COUNT_CYCLES(_ev_cnt, _num)	pme_ev_cnt_stop(_ev_cnt, _num)

#else
#define START_COUNT_CYCLES(_ev_cnt)
#define STOP_COUNT_CYCLES(_ev_cnt, _num)
#endif /* CHECK_CYCLES */


static void free_buf_from_sam_cookie(struct local_arg *larg, void *cookie)
{
	struct pkt_mdata	*mdata = (struct pkt_mdata *)cookie;

	char			*buff = mdata->buf_vaddr;
	dma_addr_t		pa;
	struct neta_bpool	*bpool;
	struct neta_buff_inf	binf;

	pa = mv_sys_dma_mem_virt2phys(buff);
	bpool = mdata->bpool;
	binf.addr = pa;
	binf.cookie = lower_32_bits((uintptr_t)(buff));
	neta_bpool_put_buff(bpool, &binf);
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
			   u8 rx_ppio_id, u8 tx_ppio_id,
			   struct neta_ppio_desc *descs, u16 num)
{
	struct sam_cio_op_params sam_descs[CRYPT_APP_MAX_BURST_SIZE];
	struct sam_cio_ipsec_params ipsec_descs[CRYPT_APP_MAX_BURST_SIZE];
	struct sam_buf_info	 src_buf_infs[CRYPT_APP_MAX_BURST_SIZE];
	struct sam_buf_info	 dst_buf_infs[CRYPT_APP_MAX_BURST_SIZE];
	struct pkt_mdata	*mdata;
	struct sam_sa		*sa;
	struct sam_cio		*cio;
	enum pkt_state		state;
	int			err;
	u16			i, bpool_buff_len, num_got, flags;
	u8			data_offs;
	struct perf_cmn_cntrs	*perf_cntrs = &larg->cmn_args.perf_cntrs;

#ifdef CRYPT_APP_VERBOSE_DEBUG
	if (larg->cmn_args.verbose)
		pr_info("%s: %d packets received. %d -> %d\n", __func__, num, rx_ppio_id, tx_ppio_id);
#endif /* CRYPT_APP_VERBOSE_DEBUG */

	/* TODO: Get current buffer size */
	bpool_buff_len = larg->cmn_args.garg->cmn_args.mtu + 64;

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
	if ((larg->dir == CRYPTO_ENC) && larg->cmn_args.garg->tunnel && !larg->cmn_args.garg->ip6)
		flags = MDATA_FLAGS_IP4_CSUM_MASK | MDATA_FLAGS_IP4_SEQID_MASK;
	else
		flags = 0;

	for (i = 0; i < num; i++) {
		char *vaddr;
		struct neta_bpool *bpool;

		vaddr = (char *)((uintptr_t)neta_ppio_inq_desc_get_cookie(&descs[i]) | neta_sys_dma_high_addr);
		bpool = neta_ppio_inq_desc_get_bpool(&descs[i], larg->lcl_ports_desc[rx_ppio_id].ppio);

		mdata = (struct pkt_mdata *)mv_stack_pop(larg->stack_hndl);
		if (!mdata)
			break;

		mdata->state = state;
		mdata->rx_port = rx_ppio_id;
		mdata->tx_port = tx_ppio_id;
		mdata->bpool = bpool;
		mdata->buf_vaddr = vaddr;
		mdata->flags = flags;

		/* Set vaddr and paddr to MAC address of the packet */
		vaddr += MVAPPS_NETA_PKT_EFEC_OFFS;
		src_buf_infs[i].vaddr = (char *)((uintptr_t)vaddr);

		src_buf_infs[i].paddr = neta_ppio_inq_desc_get_phys_addr(&descs[i]) + MVAPPS_NETA_PKT_EFEC_OFFS;

		/* Exclude MH from packet length */
		src_buf_infs[i].len = neta_ppio_inq_desc_get_pkt_len(&descs[i]);

#ifdef CRYPT_APP_VERBOSE_DEBUG
		if (larg->cmn_args.verbose > 1) {
			printf("Received packet (va:%p, pa 0x%08x, len %d):\n",
			       src_buf_infs[i].vaddr,
			       (unsigned int)src_buf_infs[i].paddr,
			       src_buf_infs[i].len);
			mv_mem_dump(src_buf_infs[i].vaddr, src_buf_infs[i].len);
		}
#endif /* CRYPT_APP_VERBOSE_DEBUG */

		dst_buf_infs[i].vaddr = src_buf_infs[i].vaddr;
		dst_buf_infs[i].paddr = src_buf_infs[i].paddr;
		dst_buf_infs[i].len = bpool_buff_len;

		if (larg->cmn_args.garg->crypto_proto == SAM_PROTO_NONE) {
			enum neta_inq_l4_type type;

			/* SAM_PROTO_NONE: data_offs - L4 offset */
			neta_ppio_inq_desc_get_l4_info(&descs[i], &type, &data_offs);
			mdata->data_offs = data_offs;

			sam_descs[i].sa = sa;
			sam_descs[i].cookie = mdata;

			sam_descs[i].num_bufs = 1;
			sam_descs[i].src = &src_buf_infs[i];
			sam_descs[i].dst = &dst_buf_infs[i];
			sam_descs[i].cipher_iv = rfc3602_aes128_cbc_t1_iv;

			sam_descs[i].cipher_offset = data_offs;
			sam_descs[i].cipher_len = src_buf_infs[i].len - sam_descs[i].cipher_offset;
		} else if (larg->cmn_args.garg->crypto_proto == SAM_PROTO_IPSEC) {
			enum neta_inq_l3_type type;

			/* SAM_PROTO_IPsec: data_offs - L3 offset */
			neta_ppio_inq_desc_get_l3_info(&descs[i], &type, &data_offs);
			mdata->data_offs = data_offs;

			ipsec_descs[i].sa = sa;
			ipsec_descs[i].cookie = mdata;

			ipsec_descs[i].num_bufs = 1;
			ipsec_descs[i].src = &src_buf_infs[i];
			ipsec_descs[i].dst = &dst_buf_infs[i];

			ipsec_descs[i].l3_offset = data_offs;
			ipsec_descs[i].pkt_size = src_buf_infs[i].len;
		} else {
			pr_err("Unknown crypto_proto = %d\n", larg->cmn_args.garg->crypto_proto);
			return -EFAULT;
		}
	}
	num_got = i;

START_COUNT_CYCLES(pme_ev_cnt_enq);
	if (larg->cmn_args.garg->crypto_proto == SAM_PROTO_NONE)
		err = sam_cio_enq(cio, sam_descs, &num_got);
	else
		err = sam_cio_enq_ipsec(cio, ipsec_descs, &num_got);
STOP_COUNT_CYCLES(pme_ev_cnt_enq, num_got);

	if (unlikely(err)) {
		pr_err("SAM EnQ (EnC) failed (%d)!\n", err);
		return -EFAULT;
	}
	if (num_got < num) {
		struct neta_bpool	*bpool;
		struct neta_buff_inf	 binf;

		for (i = num_got; i < num; i++) {
			binf.addr = neta_ppio_inq_desc_get_phys_addr(&descs[i]);
			binf.cookie = neta_ppio_inq_desc_get_cookie(&descs[i]);
			bpool = neta_ppio_inq_desc_get_bpool(&descs[i], larg->lcl_ports_desc[rx_ppio_id].ppio);
			neta_bpool_put_buff(bpool, &binf);
			if (larg->cmn_args.garg->crypto_proto == SAM_PROTO_NONE)
				mdata = sam_descs[i].cookie;
			else
				mdata = ipsec_descs[i].cookie;

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
	struct sam_buf_info	 src_buf_infs[CRYPT_APP_MAX_BURST_SIZE];
	struct sam_buf_info	 dst_buf_infs[CRYPT_APP_MAX_BURST_SIZE];
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
		data_offs = mdata->data_offs;

		src_buf_infs[i].vaddr = (char *)mdata->buf_vaddr + MVAPPS_NETA_PKT_EFEC_OFFS;
		src_buf_infs[i].paddr = mv_sys_dma_mem_virt2phys(src_buf_infs[i].vaddr);
		src_buf_infs[i].len = sam_res_descs[i].out_len;

		dst_buf_infs[i].vaddr = src_buf_infs[i].vaddr;
		dst_buf_infs[i].paddr = src_buf_infs[i].paddr;
		dst_buf_infs[i].len = bpool_buff_len;

		if (larg->cmn_args.garg->crypto_proto == SAM_PROTO_NONE) {
			sam_descs[i].cookie = mdata;
			sam_descs[i].sa = larg->dec_sa;
			sam_descs[i].num_bufs = 1;
			sam_descs[i].src = &src_buf_infs[i];
			sam_descs[i].dst = &dst_buf_infs[i];
			sam_descs[i].cipher_iv = rfc3602_aes128_cbc_t1_iv;
			sam_descs[i].cipher_offset = data_offs;
			sam_descs[i].cipher_len = src_buf_infs[i].len - sam_descs[i].cipher_offset;
		} else if (larg->cmn_args.garg->crypto_proto == SAM_PROTO_IPSEC) {
			ipsec_descs[i].sa = larg->dec_sa;
			ipsec_descs[i].cookie = mdata;

			ipsec_descs[i].num_bufs = 1;
			ipsec_descs[i].src = &src_buf_infs[i];
			ipsec_descs[i].dst = &dst_buf_infs[i];
			ipsec_descs[i].l3_offset = data_offs;
			ipsec_descs[i].pkt_size = src_buf_infs[i].len;
		}

#ifdef CRYPT_APP_VERBOSE_DEBUG
		if (larg->cmn_args.verbose > 1) {
			printf("Encrypted packet (va:%p, pa 0x%08x, len %d):\n",
			       src_buf_infs[i].vaddr,
			       (unsigned int)src_buf_infs[i].paddr,
			       src_buf_infs[i].len);
			mv_mem_dump(src_buf_infs[i].vaddr, src_buf_infs[i].len);
		}
#endif /* CRYPT_APP_VERBOSE_DEBUG */
	}
	num_got = num;

START_COUNT_CYCLES(pme_ev_cnt_enq);
	if (larg->cmn_args.garg->crypto_proto == SAM_PROTO_NONE)
		err = sam_cio_enq(larg->dec_cio, sam_descs, &num_got);
	else
		err = sam_cio_enq_ipsec(larg->dec_cio, ipsec_descs, &num_got);
STOP_COUNT_CYCLES(pme_ev_cnt_enq, num_got);

	if (unlikely(err)) {
		pr_err("SAM EnQ (DeC) failed (%d)!\n", err);
		return -EFAULT;
	}
	if (num_got < num) {
		for (i = num_got; i < num; i++) {
			if (larg->cmn_args.garg->crypto_proto == SAM_PROTO_NONE)
				free_buf_from_sam_cookie(larg, sam_descs[i].cookie);
			else
				free_buf_from_sam_cookie(larg, ipsec_descs[i].cookie);
		}
		/*pr_warn("%s: %d packets dropped\n", __func__, num - num_got);*/
		perf_cntrs->drop_cnt += (num - num_got);
	}
	return 0;
}

static inline int send_pkts(struct local_arg *larg,
			    struct sam_cio_op_result *sam_res_descs, u16 num)
{
	struct tx_shadow_q	*shadow_q;
	struct neta_bpool	*bpool;
	struct neta_buff_inf	*binf;
	struct neta_ppio_desc	*desc;
	struct neta_ppio_desc	 descs[MVAPPS_NETA_MAX_NUM_PORTS][CRYPT_APP_MAX_BURST_SIZE];
	int			 err, tc = 0;
	u16			 i, tp, num_got, port_nums[MVAPPS_NETA_MAX_NUM_PORTS];
	struct perf_cmn_cntrs	*perf_cntrs = &larg->cmn_args.perf_cntrs;

	for (tp = 0; tp < larg->cmn_args.num_ports; tp++)
		port_nums[tp] = 0;

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
		shadow_q->ents[shadow_q->write_ind].bpool = bpool;
		shadow_q->write_ind++;
		if (shadow_q->write_ind == CRYPT_APP_TX_Q_SIZE)
			shadow_q->write_ind = 0;
		port_nums[tp]++;

		/* We don't need metadata more */
		mv_stack_push(larg->stack_hndl, mdata);
	}

	for (tp = 0; tp < larg->cmn_args.num_ports; tp++) {
		num = num_got = port_nums[tp];
		shadow_q = &larg->lcl_ports_desc[tp].shadow_qs[tc];

START_COUNT_CYCLES(pme_ev_cnt_tx);
		if (num_got) {
			err = neta_ppio_send(larg->lcl_ports_desc[tp].ppio, tc,
					    descs[tp], &num_got);
			if (err)
				return err;
		}
STOP_COUNT_CYCLES(pme_ev_cnt_tx, num_got);

#ifdef CRYPT_APP_VERBOSE_DEBUG
		if (larg->cmn_args.verbose && num_got)
			printf("sent %d pkts on ppio %d, tc %d\n", num_got, tp, tc);
#endif /* CRYPT_APP_VERBOSE_DEBUG */

		if (num_got < num) {
			for (i = 0; i < num - num_got; i++) {
				if (shadow_q->write_ind == 0)
					shadow_q->write_ind = CRYPT_APP_TX_Q_SIZE;
				shadow_q->write_ind--;
				binf = &shadow_q->ents[shadow_q->write_ind].buff_ptr;
				if (unlikely(!binf->cookie || !binf->addr)) {
					pr_warn("Shadow memory @%d: cookie(%lx), pa(%lx)!\n",
						shadow_q->write_ind, (u64)binf->cookie, (u64)binf->addr);
					continue;
				}
				bpool = shadow_q->ents[shadow_q->write_ind].bpool;
				neta_bpool_put_buff(bpool, binf);
			}
			/*pr_warn("%s: %d packets dropped\n", __func__, num - num_got);*/
			perf_cntrs->drop_cnt += (num - num_got);
		}

		neta_ppio_get_num_outq_done(larg->lcl_ports_desc[tp].ppio, tc, &num);
		for (i = 0; i < num; i++) {
			binf = &shadow_q->ents[shadow_q->read_ind].buff_ptr;
			if (unlikely(!binf->cookie || !binf->addr)) {
				pr_warn("Shadow memory @%d: cookie(%lx), pa(%lx)!\n",
					shadow_q->read_ind, (u64)binf->cookie, (u64)binf->addr);
				continue;
			}
			bpool = shadow_q->ents[shadow_q->read_ind].bpool;
			neta_bpool_put_buff(bpool, binf);

			shadow_q->read_ind++;
			if (shadow_q->read_ind == CRYPT_APP_TX_Q_SIZE)
				shadow_q->read_ind = 0;
		}
		perf_cntrs->tx_cnt += num;
	}
	return 0;
}

static inline int deq_crypto_pkts(struct local_arg	*larg,
				  struct sam_cio	*cio)
{
	struct sam_cio_op_result res_descs[CRYPT_APP_MAX_BURST_SIZE];
	struct sam_cio_op_result res_descs_to_dec[CRYPT_APP_MAX_BURST_SIZE];
	struct sam_cio_op_result res_descs_to_send[CRYPT_APP_MAX_BURST_SIZE];
	struct sam_cio_op_result *to_send_ptr, *to_dec_ptr;
	int			 err;
	u16			 i, num, num_to_send, num_to_dec;
	struct perf_cmn_cntrs	*perf_cntrs = &larg->cmn_args.perf_cntrs;

	num = CRYPT_APP_MAX_BURST_SIZE;
START_COUNT_CYCLES(pme_ev_cnt_deq);
	err = sam_cio_deq(cio, res_descs, &num);
STOP_COUNT_CYCLES(pme_ev_cnt_deq, num);
	if (unlikely(err)) {
		pr_err("SAM DeQ (EnC) failed (%d)!\n", err);
		return -EFAULT;
	}
	num_to_send = num_to_dec = 0;
	if (larg->dir == CRYPTO_LB) {
		for (i = 0; i < num; i++) {
			struct pkt_mdata *mdata;

			if (unlikely(!res_descs[i].cookie)) {
				pr_err("SAM operation (EnC) failed (no cookie: %d,%d)!\n", i, res_descs[i].out_len);
				perf_cntrs->drop_cnt++;
				continue;
			}

			if (res_descs[i].status != SAM_CIO_OK) {
				pr_warn("SAM operation (EnC) failed (%d)!\n", res_descs[i].status);
				/* Free buffer */
				free_buf_from_sam_cookie(larg, res_descs[i].cookie);
				perf_cntrs->drop_cnt++;
				continue;
			}
			mdata = (struct pkt_mdata *)res_descs[i].cookie;
			if (mdata->state == (u8)PKT_STATE_ENC) {
				mdata->state = (u8)PKT_STATE_DEC;
				res_descs_to_dec[num_to_dec++] = res_descs[i];
			} else {
				res_descs_to_send[num_to_send++] = res_descs[i];
			}
			to_dec_ptr = res_descs_to_dec;
			to_send_ptr = res_descs_to_send;
		}
	} else {
		to_send_ptr = res_descs;
		num_to_send = num;
	}

	if (num_to_dec) {
		err = dec_pkts(larg, to_dec_ptr, num_to_dec);
		if (unlikely(err))
			return err;
	}

	if (num_to_send) {
#ifdef CRYPT_APP_PKT_ECHO_SUPPORT
		if (likely(larg->cmn_args.echo))
			echo_pkts(larg, to_send_ptr, num_to_send);
#endif /* CRYPT_APP_PKT_ECHO_SUPPORT */

		return send_pkts(larg, to_send_ptr, num_to_send);
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

		err = proc_rx_pkts(larg, rx_ppio_id, tx_ppio_id, descs, num);
		if (unlikely(err))
			return err;
	}
	if (larg->enc_cio) {
		err = deq_crypto_pkts(larg, larg->enc_cio);
		if (unlikely(err))
			return err;
	}

	if (larg->dec_cio && (larg->dec_cio != larg->enc_cio))
		return deq_crypto_pkts(larg, larg->dec_cio);

	return 0;
}

static int find_free_cio(u8 eng)
{
	int	i;

	for (i = 0; i < CRYPT_APP_TOTAL_NUM_CIOS; i++) {
		if (!((uint64_t)(1 << i) & used_cios[eng])) {
			used_cios[eng] |= (uint64_t)(1 << i);
			break;
		}
	}
	if (i == CRYPT_APP_TOTAL_NUM_CIOS) {
		pr_err("no free CIO found!\n");
		return -ENOSPC;
	}
	return i;
}

static int loop_1p(struct local_arg *larg, int *running)
{
	int			 err;
	u16			 num;
	u8			 tc = 0, qid = 0;

	if (!larg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	num = larg->cmn_args.burst;

	while (*running) {
		/* Find next queue to consume */
		do {
			qid++;
			if (qid == MVAPPS_NETA_MAX_NUM_QS_PER_TC) {
				qid = 0;
				tc++;
				if (tc == CRYPT_APP_MAX_NUM_TCS_PER_PORT)
					tc = 0;
			}
		} while (!(larg->cmn_args.qs_map & (1 << ((tc * MVAPPS_NETA_MAX_NUM_QS_PER_TC) + qid))));

		err = loop_sw_recycle(larg, 0, 0, qid, num);
		if (err != 0)
			return err;
	}

	return 0;
}

static int loop_2ps(struct local_arg *larg, int *running)
{
	int			 err;
	u16			 num;
	u8			 tc = 0, qid = 0;

	if (!larg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	num = larg->cmn_args.burst;

	while (*running) {
		/* Find next queue to consume */
		do {
			qid++;
			if (qid == MVAPPS_NETA_MAX_NUM_QS_PER_TC) {
				qid = 0;
				tc++;
				if (tc == CRYPT_APP_MAX_NUM_TCS_PER_PORT)
					tc = 0;
			}
		} while (!(larg->cmn_args.qs_map & (1 << ((tc * MVAPPS_NETA_MAX_NUM_QS_PER_TC) + qid))));

		err  = loop_sw_recycle(larg, 0, 1, qid, num);
		err |= loop_sw_recycle(larg, 1, 0, qid, num);
		if (err != 0)
			return err;
	}

	return 0;
}

static int main_loop_cb(void *arg, int *running)
{
	struct local_arg	*larg = (struct local_arg *)arg;

	if (!larg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	if (larg->cmn_args.num_ports == 1)
		return loop_1p(larg, running);
	return loop_2ps(larg, running);
}


static int init_all_modules(void)
{
	struct neta_init_params	params;
	int			err;

	pr_info("Global initializations ...\n");

	err = mv_sys_dma_mem_init(CRYPT_APP_DMA_MEM_SIZE);
	if (err)
		return err;

	memset(&params, 0, sizeof(params));
	params.bm_pool_reserved_map = MVAPPS_NETA_BPOOLS_RSRV;

	err = neta_init(&params);
	if (err)
		return err;

	pr_info("done\n");
	return 0;
}

static int create_sam_sessions(struct sam_sa **enc_sa, struct sam_sa **dec_sa,
			       enum sam_crypto_protocol crypto_proto, int tunnel, int ip6,
			       enum sam_cipher_alg cipher_alg, enum sam_cipher_mode cipher_mode,
			       enum sam_auth_alg auth_alg)
{
	struct sam_session_params	 sa_params;
	int				 err;

	memset(&sa_params, 0, sizeof(sa_params));
	sa_params.proto = crypto_proto;

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
	if (crypto_proto == SAM_PROTO_IPSEC) {
		sa_params.u.ipsec.is_esp = 1;
		sa_params.u.ipsec.is_natt = 0;
		sa_params.u.ipsec.spi = t1_spi;
		sa_params.u.ipsec.seq = t1_seq;

		if (tunnel) {
			sa_params.u.ipsec.is_tunnel = 1;
			if (ip6) {
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
	} else {
		sa_params.u.basic.auth_aad_len = 0;   /* Additional Data (AAD) size (in bytes) */
		if (sa_params.auth_alg != SAM_AUTH_NONE)
			sa_params.u.basic.auth_icv_len = ICV_LEN;
		else
			sa_params.u.basic.auth_icv_len = 0;
	}

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

static int init_local_modules(struct glob_arg *garg)
{
	struct sam_init_params		init_params;
	int				err, port_index;
	struct bpool_inf		std_infs[] = CRYPT_APP_BPOOLS_INF;
	struct bpool_inf		jumbo_infs[] = CRYPT_APP_BPOOLS_JUMBO_INF;
	struct bpool_inf		*infs;
	int				i;

	pr_info("Specific modules initializations\n");

	if (garg->cmn_args.mtu > DEFAULT_MTU) {
		infs = jumbo_infs;
		garg->num_pools = ARRAY_SIZE(jumbo_infs);
	} else {
		infs = std_infs;
		garg->num_pools = ARRAY_SIZE(std_infs);
	}
	/* Calculate total number of buffers in the pools */
	garg->num_bufs = 0;
	for (i = 0; i < garg->num_pools; i++)
		garg->num_bufs += infs[i].num_buffs;

	for (port_index = 0; port_index < garg->cmn_args.num_ports; port_index++) {
		struct port_desc *port = &garg->ports_desc[port_index];

		err = app_find_port_info(port);
		if (err)
			return err;

		err = app_build_port_bpools(&garg->pools_desc[port_index], garg->num_pools, infs);
		if (err)
			return err;

		port->num_tcs	= CRYPT_APP_MAX_NUM_TCS_PER_PORT;
		for (i = 0; i < port->num_tcs; i++)
			port->num_inqs[i] = garg->cmn_args.cpus;

		port->inq_size	= CRYPT_APP_RX_Q_SIZE;
		port->num_outqs	= CRYPT_APP_MAX_NUM_TCS_PER_PORT;
		port->outq_size	= CRYPT_APP_TX_Q_SIZE;
		port->first_inq	= CRYPT_APP_FIRST_INQ;

		/* pkt_offset=0 not to be changed, before recoding rx_data_path to use pkt_offset as well */
		err = app_port_init(port, garg->num_pools, garg->pools_desc[port_index],
				    garg->cmn_args.mtu, MVAPPS_NETA_PKT_OFFS);
		if (err) {
			pr_err("Failed to initialize port %d\n", port_index);
			return err;
		}
	}

	/* In that stage, we're initializaing CIO for global-arg just in order
	 * to enforce the initialization of the engine.
	 * TODO: in the future, replace the below code with appropraite initialization of the engine.
	 */

	init_params.max_num_sessions = CRYPT_APP_MAX_NUM_SESSIONS;
	sam_init(&init_params);

	return 0;
}

static void destroy_local_modules(struct glob_arg *garg)
{
	int port_index;

	app_disable_all_ports(garg->ports_desc, garg->cmn_args.num_ports);

	for (port_index = 0; port_index < garg->num_ports; port_index++)
		app_free_all_pools(garg->pools_desc[port_index], garg->num_pools);

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

#ifdef CHECK_CYCLES
	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "pme";
	cmd_params.desc		= "Performance Montitor Emulator";
	cmd_params.format	= NULL;
	cmd_params.cmd_arg	= garg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pme_cmd_cb;
	mvapp_register_cli_cmd(&cmd_params);
#endif /* CHECK_CYCLES */

	app_register_cli_common_cmds(garg->ports_desc);


	return 0;
}

static int init_global(void *arg)
{
	struct glob_arg *garg = (struct glob_arg *)arg;
	int		 err;

	if (!garg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	if (pthread_mutex_init(&garg->trd_lock, NULL) != 0) {
		pr_err("init lock failed!\n");
		return -EIO;
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
	destroy_local_modules(garg);
	destroy_all_modules();
}

static int init_local(void *arg, int id, void **_larg)
{
	struct glob_arg		*garg = (struct glob_arg *)arg;
	struct local_arg	*larg;
	struct sam_cio_params	cio_params;
	struct sam_cio		*cio;
	char			 name[15];
	int			 i, err, cio_id;

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
	cio_id = find_free_cio(0);
	if (cio_id < 0) {
		pr_err("free CIO not found!\n");
		pthread_mutex_unlock(&garg->trd_lock);
		return cio_id;
	}
	memset(name, 0, sizeof(name));
	snprintf(name, sizeof(name), "cio-%d:%d", 0, cio_id);
	pr_info("found cio: %s\n", name);
	memset(&cio_params, 0, sizeof(cio_params));
	cio_params.match = name;
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
		int sam_engine;

		larg->enc_cio = cio;

		if (sam_get_num_inst() == 2)
			sam_engine = 1;
		else
			sam_engine = 0;

		pthread_mutex_lock(&garg->trd_lock);
		cio_id = find_free_cio(sam_engine);
		pthread_mutex_unlock(&garg->trd_lock);

		if (cio_id < 0) {
			pr_err("free CIO not found!\n");
			return cio_id;
		}
		memset(name, 0, sizeof(name));
		snprintf(name, sizeof(name), "cio-%d:%d", sam_engine, cio_id);
		pr_info("found cio: %s\n", name);
		memset(&cio_params, 0, sizeof(cio_params));
		cio_params.match = name;
		cio_params.size = CRYPT_APP_CIO_Q_SIZE;
		err = sam_cio_init(&cio_params, &larg->dec_cio);
		if (err != 0)
			return err;
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

	larg->pools_desc = garg->pools_desc[larg->cmn_args.id];

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
	pr_info("%d of %d metadata structures allocated\n", i, garg->num_bufs);

	err = create_sam_sessions(&larg->enc_sa, &larg->dec_sa,
				  garg->crypto_proto, garg->tunnel, garg->ip6,
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

	larg->cmn_args.qs_map = garg->cmn_args.qs_map << (garg->cmn_args.qs_map_shift * id);
	pr_info("thread %d (cpu %d) mapped to Qs %llx using %s\n",
		larg->cmn_args.id, sched_getcpu(), (unsigned int long long)larg->cmn_args.qs_map, name);

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

#ifdef MVCONF_SAM_STATS
	if (larg->enc_cio)
		app_sam_show_cio_stats(larg->enc_cio, "encrypt", 1);

	if (larg->dec_cio && (larg->dec_cio != larg->enc_cio))
		app_sam_show_cio_stats(larg->dec_cio, "decrypt", 1);

	app_sam_show_stats(1);
#endif /* MVCONF_SAM_STATS */

	if (larg->lcl_ports_desc) {
		for (i = 0; i < larg->num_ports; i++)
			app_port_local_deinit(&larg->lcl_ports_desc[i]);
		free(larg->lcl_ports_desc);
	}
	/* Free all pkt_mdata structures and delete stack instance */
	while (!mv_stack_is_empty(larg->stack_hndl)) {
		mdata = mv_stack_pop(larg->stack_hndl);
		if (!mdata)
			break;

		free(mdata);
	}
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
	       "\t--crypto-proto <proto>   Crypto protocol. Support: [none, esp]. (default: none).\n"
	       "\t--tunnel                 IPSec tunnel mode. (default: transport)\n"
	       "\t--ip6                    ESP over IPv6. (default: ESP over IPv4)\n"
	       "\t--cipher-alg   <alg>     Cipher algorithm. Support: [none, aes128, 3des]. (default: aes128).\n"
	       "\t--cipher-mode  <alg>     Cipher mode. Support: [cbc, ecb]. (default: cbc).\n"
	       "\t--auth-alg     <alg>     Authentication algorithm. Support: [none, sha1]. (default: sha1).\n"
	       "\t--dir          <dir>     Operation direction. Support: [enc, dec, lb]. (default: lb)\n"
	       "\t--no-echo                No Echo packets\n"
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
	garg->tunnel = 0;
	garg->ip6 = 0;
	garg->crypto_proto = SAM_PROTO_NONE;
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
			     token = strtok(NULL, ","), garg->cmn_args.num_ports++)
				snprintf(garg->ports_desc[garg->cmn_args.num_ports].name,
					 sizeof(garg->ports_desc[garg->cmn_args.num_ports].name),
					 "%s", token);

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
		} else if (strcmp(argv[i], "--tunnel") == 0) {
			garg->tunnel = 1;
			i += 1;
		} else if (strcmp(argv[i], "--ip6") == 0) {
			garg->ip6 = 1;
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
		} else if (strcmp(argv[i], "--crypto-proto") == 0) {
			if (strcmp(argv[i+1], "esp") == 0)
				garg->crypto_proto = SAM_PROTO_IPSEC;
			else if (strcmp(argv[i+1], "none") == 0)
				garg->crypto_proto = SAM_PROTO_NONE;
			else {
				pr_err("Crypto protocol (%s) not supported!\n", argv[i+1]);
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
	pr_info("crypto-proto  : %s\n", garg->crypto_proto == SAM_PROTO_IPSEC ? "IPSec" : "None");

	if (garg->crypto_proto == SAM_PROTO_IPSEC)
		pr_info("IPSec mode    : %s\n", garg->tunnel ? "Tunnel" : "Transport");

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
