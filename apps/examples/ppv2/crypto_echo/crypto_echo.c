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

#include "mv_std.h"
#include "lib/lib_misc.h"
#include "env/mv_sys_dma.h"

#include "mv_pp2.h"
#include "mv_pp2_hif.h"
#include "mv_pp2_bpool.h"
#include "mv_pp2_ppio.h"
#include "mv_sam.h"

#include "utils.h"
#include "mvapp.h"
#include "perf_mon_emu.h"

#define CRYPT_APP_DEF_Q_SIZE		256/*1024*/
#define CRYPT_APP_HIF_Q_SIZE		CRYPT_APP_DEF_Q_SIZE
#define CRYPT_APP_RX_Q_SIZE		CRYPT_APP_DEF_Q_SIZE
#define CRYPT_APP_TX_Q_SIZE		CRYPT_APP_DEF_Q_SIZE
#define CRYPT_APP_CIO_Q_SIZE		CRYPT_APP_DEF_Q_SIZE

#define CRYPT_APP_MAX_BURST_SIZE	((CRYPT_APP_RX_Q_SIZE)>>2)
#define CRYPT_APP_DFLT_BURST_SIZE	64
#define CRYPT_APP_CTRL_DFLT_THR		1000
#define CRYPT_APP_DMA_MEM_SIZE		(48*1024*1024)

#define CRYPT_APP_CIOS_RSRV		{ 0x0, 0x0 }

#define CRYPT_APP_MAX_NUM_PORTS			2
#define CRYPT_APP_MAX_NUM_TCS_PER_PORT		1
#define CRYPT_APP_MAX_NUM_QS_PER_CORE		CRYPT_APP_MAX_NUM_TCS_PER_PORT
#define CRYPT_APP_MAX_NUM_SESSIONS_PER_RING	4
#define CRYPT_APP_SAM_TWO_ENG_SUPPORT

/* TODO: find more generic way to get the following parameters */
#define CRYPT_APP_TOTAL_NUM_CIOS	4

/*#define CRYPT_APP_VERBOSE_CHECKS*/
/*#define CRYPT_APP_VERBOSE_DEBUG*/
#define CRYPT_APP_PKT_ECHO_SUPPORT
#define CRYPT_APP_PREFETCH_SHIFT	4

#define COOKIE_SET_DIR(_c, _d)		((_c) = (void *)(((uintptr_t)(_c) & ~0x1) | ((_d) << 0)))
#define COOKIE_SET_RX_PORT(_c, _p)	((_c) = (void *)(((uintptr_t)(_c) & ~0x4) | ((_p) << 2)))
#define COOKIE_SET_TX_PORT(_c, _p)	((_c) = (void *)(((uintptr_t)(_c) & ~0x8) | ((_p) << 3)))
#define COOKIE_SET_ALL_INFO(_c, _rp, _tp, _bp, _d)	\
	((_c) = (void *)(((uintptr_t)(_c) & ~0x3d) | \
		(((_d) << 0) | ((_rp) << 2) | ((_tp) << 3) | ((_bp) << 4))))
#define PP2_COOKIE_SET_ALL_INFO(_c, _rp, _tp, _bp, _d)	\
	((_c) = (((_c) & ~0x3d) | \
		(((_d) << 0) | ((_rp) << 2) | ((_tp) << 3) | ((_bp) << 4))))
#define COOKIE_GET_DIR(_c)		((uintptr_t)(_c) & 0x1)
#define COOKIE_GET_RX_PORT(_c)		(((uintptr_t)(_c) & 0x4) >> 2)
#define COOKIE_GET_TX_PORT(_c)		(((uintptr_t)(_c) & 0x8) >> 3)
#define COOKIE_GET_BPOOL(_c)		(((uintptr_t)(_c) & 0x30) >> 4)
#define COOKIE_CLEAR_ALL_INFO(_c)	((_c) = (void *)((uintptr_t)(_c) & ~0x3d))
#define PP2_COOKIE_CLEAR_ALL_INFO(_c)	((_c) = ((_c) & ~0x3d))

/*#define CRYPT_APP_BPOOLS_INF	{ {384, 4096}, {2048, 1024} }*/
#define CRYPT_APP_BPOOLS_INF	{ {2048, 4096} }
/*#define CRYPT_APP_BPOOLS_JUMBO_INF	{ {2048, 4096}, {10240, 512} }*/
#define CRYPT_APP_BPOOLS_JUMBO_INF	{ {10240, 512} }

#define RFC3602_AES128_CBC_T1_KEY {			\
	0x06, 0xa9, 0x21, 0x40, 0x36, 0xb8, 0xa1, 0x5b,	\
	0x51, 0x2e, 0x03, 0xd5, 0x34, 0x12, 0x00, 0x06	\
}

#define RFC3602_AES128_CBC_T1_IV {			\
	0x3d, 0xaf, 0xba, 0x42, 0x9d, 0x9e, 0xb4, 0x30,	\
	0xb4, 0x22, 0xda, 0x80, 0x2c, 0x9f, 0xac, 0x41	\
}

struct local_arg;

struct glob_arg {
#ifdef CRYPT_APP_VERBOSE_DEBUG
	int			 verbose;
#endif /* CRYPT_APP_VERBOSE_DEBUG */
	int			 cli;
	int			 cpus;	/* cpus used for running */
	u16			 burst;
	u16			 mtu;
	int			 affinity;
	int			 loopback;
	int			 echo;
	u64			 qs_map;
	int			 qs_map_shift;
	int			 prefetch_shift;
	int			 pp2_num_inst;
	int			 num_ports;
	struct port_desc	 ports_desc[CRYPT_APP_MAX_NUM_PORTS];

	pthread_mutex_t		 trd_lock;

	struct pp2_hif		*hif;

	int			 num_pools;
	struct bpool_desc	**pools_desc;

	int			 ctrl_thresh;
	struct timeval		 ctrl_trd_last_time;
	u64			 lst_rx_cnt;
	u64			 lst_tx_cnt;

	struct sam_cio		*cio;
	struct local_arg	*largs[MVAPPS_MAX_NUM_CORES];
};

struct local_arg {
	u64			 qs_map;

	struct pp2_hif		*hif;
	int			 num_ports;
	struct lcl_port_desc	*ports_desc;

	struct bpool_desc	**pools_desc;

	struct sam_cio		*enc_cio;
	struct sam_sa		*enc_sa;
	struct sam_cio		*dec_cio;
	struct sam_sa		*dec_sa;

	u64			 rx_cnt;
	u64			 tx_cnt;
	u64			 drop_cnt;

	u16			 burst;
	int			 echo;
	int			 id;

	struct glob_arg		*garg;
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

static inline void prefetch(const void *ptr)
{
	asm volatile("prfm pldl1keep, %a0\n" : : "p" (ptr));
}

#ifdef CRYPT_APP_PKT_ECHO_SUPPORT
static inline void echo_pkts(struct local_arg		*larg,
			     struct sam_cio_op_result	*sam_res_descs,
			    u16				 num)
{
	char			*tmp_buff;
	int			 prefetch_shift = larg->garg->prefetch_shift;
	u16			 i;

	for (i = 0; i < num; i++) {
		if (num - i > prefetch_shift) {
			tmp_buff = sam_res_descs[i + prefetch_shift].cookie;
			COOKIE_CLEAR_ALL_INFO(tmp_buff);
			prefetch(tmp_buff);
		}
		tmp_buff = sam_res_descs[i].cookie;
		COOKIE_CLEAR_ALL_INFO(tmp_buff);
#ifdef CRYPT_APP_VERBOSE_DEBUG
		if (larg->garg->verbose) {
			printf("pkt before echo (len %d):\n",
			       sam_res_descs[i].out_len - MVAPPS_PKT_EFEC_OFFS);
			mem_disp(tmp_buff, sam_res_descs[i].out_len - MVAPPS_PKT_EFEC_OFFS);
		}
#endif /* CRYPT_APP_VERBOSE_DEBUG */
		swap_l2(tmp_buff);
		swap_l3(tmp_buff);
		/*printf("pkt after echo:\n");*/
		/*mem_disp(tmp_buff, sam_res_descs[i].out_len  - MVAPPS_PKT_EFEC_OFFS);*/
	}
}
#endif /* CRYPT_APP_PKT_ECHO_SUPPORT */

static inline int enc_pkts(struct local_arg		*larg,
			   u8				 rx_ppio_id,
			   u8				 tx_ppio_id,
			   struct pp2_ppio_desc	*descs,
			   u16				 num)
{
	struct sam_cio_op_params sam_descs[CRYPT_APP_MAX_BURST_SIZE];
	struct sam_buf_info	 src_buf_infs[CRYPT_APP_MAX_BURST_SIZE];
	struct sam_buf_info	 dst_buf_infs[CRYPT_APP_MAX_BURST_SIZE];
	int			 err;
	u16			 i, bpool_buff_len, num_got;
	u8			 cipher_iv[] = RFC3602_AES128_CBC_T1_IV;
	u8			 l4_offs = (14 + 20); /* TODO: hardcoded for eth/ipv4! */

	/* TODO: is this enough?!?!?! */
	bpool_buff_len = larg->garg->mtu + 64;

	larg->rx_cnt += num;

	memset(sam_descs, 0, sizeof(sam_descs));

	for (i = 0; i < num; i++) {
		src_buf_infs[i].vaddr = (char *)(uintptr_t)pp2_ppio_inq_desc_get_cookie(&descs[i]);
		src_buf_infs[i].paddr = pp2_ppio_inq_desc_get_phys_addr(&descs[i]);
		/* source buffer length is received packet size + headroom size */
		src_buf_infs[i].len = pp2_ppio_inq_desc_get_pkt_len(&descs[i]) + MVAPPS_PKT_EFEC_OFFS;

		src_buf_infs[i].vaddr += MVAPPS_PKT_EFEC_OFFS;
		src_buf_infs[i].vaddr = (char *)(((uintptr_t)(src_buf_infs[i].vaddr)) | sys_dma_high_addr);

#ifdef CRYPT_APP_VERBOSE_DEBUG
		if (larg->garg->verbose) {
			printf("Received packet (va:%p, pa 0x%08x, len %d):\n",
			       src_buf_infs[i].vaddr,
			       (unsigned int)src_buf_infs[i].paddr,
			       src_buf_infs[i].len - MVAPPS_PKT_EFEC_OFFS);
			mem_disp(src_buf_infs[i].vaddr, src_buf_infs[i].len - MVAPPS_PKT_EFEC_OFFS);
		}
#endif /* CRYPT_APP_VERBOSE_DEBUG */

		/* Mark the cookie with '1' (first bit) so we know it is for encrypt */
		dst_buf_infs[i].vaddr = src_buf_infs[i].vaddr;
		dst_buf_infs[i].paddr = src_buf_infs[i].paddr;
		dst_buf_infs[i].len = bpool_buff_len;

		sam_descs[i].sa = larg->enc_sa;
		sam_descs[i].cookie = dst_buf_infs[i].vaddr;
		/* TODO: find appropriate BPool-id! */
		COOKIE_SET_ALL_INFO(sam_descs[i].cookie, rx_ppio_id, tx_ppio_id, 0, 1);
		sam_descs[i].num_bufs = 1;
		sam_descs[i].src = &src_buf_infs[i];
		sam_descs[i].dst = &dst_buf_infs[i];
		sam_descs[i].cipher_iv = cipher_iv;
		sam_descs[i].cipher_offset = MVAPPS_PKT_EFEC_OFFS + l4_offs;
		sam_descs[i].cipher_len = src_buf_infs[i].len - sam_descs[i].cipher_offset;
	}

	num_got = num;
START_COUNT_CYCLES(pme_ev_cnt_enq);
	err = sam_cio_enq(larg->enc_cio, sam_descs, &num_got);
STOP_COUNT_CYCLES(pme_ev_cnt_enq, num_got);
	if (unlikely(err)) {
		pr_err("SAM EnQ (EnC) failed (%d)!\n", err);
		return -EFAULT;
	}
	if (num_got < num) {
		struct pp2_bpool	*bpool;
		struct pp2_buff_inf	 binf;

		for (i = num_got; i < num; i++) {
			char			*buff = (char *)(uintptr_t)sam_descs[i].cookie;
			dma_addr_t		 pa;

			COOKIE_CLEAR_ALL_INFO(buff);
			buff -= MVAPPS_PKT_EFEC_OFFS;
			pa = mv_sys_dma_mem_virt2phys(buff);
			bpool = larg->pools_desc[larg->ports_desc[COOKIE_GET_RX_PORT(sam_descs[i].cookie)].pp_id]
							[COOKIE_GET_BPOOL(sam_descs[i].cookie)].pool;
			binf.addr = pa;
			binf.cookie = lower_32_bits((uintptr_t)(buff));
			pp2_bpool_put_buff(larg->hif, bpool, &binf);
		}
		larg->drop_cnt += num - num_got;
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
	int			 err;
	u16			 i, bpool_buff_len, num_got;
	u8			 cipher_iv[] = RFC3602_AES128_CBC_T1_IV;
	u8			 l4_offs = (14 + 20); /* TODO: hardcoded for eth/ipv4! */

	/* TODO: is this enough?!?!?! */
	bpool_buff_len = larg->garg->mtu + 64;

	memset(sam_descs, 0, sizeof(sam_descs));

	/* For decryption crypto parameters are caclulated from cookie and out_len */
	for (i = 0; i < num; i++) {
		sam_descs[i].cookie = sam_res_descs[i].cookie;
		src_buf_infs[i].vaddr = (char *)(uintptr_t)sam_descs[i].cookie;
		COOKIE_CLEAR_ALL_INFO(src_buf_infs[i].vaddr);
		src_buf_infs[i].paddr = mv_sys_dma_mem_virt2phys(src_buf_infs[i].vaddr - MVAPPS_PKT_EFEC_OFFS);
		src_buf_infs[i].len = sam_res_descs[i].out_len;

		dst_buf_infs[i].vaddr = src_buf_infs[i].vaddr;
		dst_buf_infs[i].paddr = src_buf_infs[i].paddr;
		dst_buf_infs[i].len = bpool_buff_len;

		sam_descs[i].sa = larg->dec_sa;
		sam_descs[i].num_bufs = 1;
		sam_descs[i].src = &src_buf_infs[i];
		sam_descs[i].dst = &dst_buf_infs[i];
		sam_descs[i].cipher_iv = cipher_iv;
		sam_descs[i].cipher_offset = MVAPPS_PKT_EFEC_OFFS + l4_offs;
		sam_descs[i].cipher_len = src_buf_infs[i].len - sam_descs[i].cipher_offset;
	}
	num_got = num;
START_COUNT_CYCLES(pme_ev_cnt_enq);
	err = sam_cio_enq(larg->dec_cio, sam_descs, &num_got);
STOP_COUNT_CYCLES(pme_ev_cnt_enq, num_got);
	if (unlikely(err)) {
		pr_err("SAM EnQ (DeC) failed (%d)!\n", err);
		return -EFAULT;
	}
	if (num_got < num) {
		struct pp2_bpool	*bpool;
		struct pp2_buff_inf	 binf;

		for (i = num_got; i < num; i++) {
			char			*buff = (char *)(uintptr_t)sam_descs[i].cookie;
			dma_addr_t		 pa;

			COOKIE_CLEAR_ALL_INFO(buff);
			buff -= MVAPPS_PKT_EFEC_OFFS;
			pa = mv_sys_dma_mem_virt2phys(buff);
			bpool = larg->pools_desc[larg->ports_desc[COOKIE_GET_RX_PORT(sam_descs[i].cookie)].pp_id]
							[COOKIE_GET_BPOOL(sam_descs[i].cookie)].pool;
			binf.addr = pa;
			binf.cookie = lower_32_bits((uintptr_t)(buff));
			pp2_bpool_put_buff(larg->hif, bpool, &binf);
		}
		larg->drop_cnt += num - num_got;
	}

	return 0;
}

static inline int send_pkts(struct local_arg		*larg,
			    u8				 tc,
			    struct sam_cio_op_result	*sam_res_descs,
			    u16				 num)
{
	struct tx_shadow_q	*shadow_q;
	struct pp2_bpool	*bpool;
	struct pp2_buff_inf	*binf;
	struct pp2_ppio_desc	*desc;
	struct pp2_ppio_desc	 descs[CRYPT_APP_MAX_NUM_PORTS][CRYPT_APP_MAX_BURST_SIZE];
	int			 err;
	u16			 i, rp, tp, bp, num_got, port_nums[CRYPT_APP_MAX_NUM_PORTS];

	for (tp = 0; tp < larg->garg->num_ports; tp++)
		port_nums[tp] = 0;

	for (i = 0; i < num; i++) {
		char			*buff = (char *)(uintptr_t)sam_res_descs[i].cookie;
		dma_addr_t		 pa;
		/* TODO: size is incorrect!!! */
		u16			 len = sam_res_descs[i].out_len - MVAPPS_PKT_EFEC_OFFS;

		COOKIE_CLEAR_ALL_INFO(buff);
		pa = mv_sys_dma_mem_virt2phys(buff - MVAPPS_PKT_EFEC_OFFS);

#ifdef CRYPT_APP_VERBOSE_DEBUG
		if (larg->garg->verbose) {
			printf("Sending packet (va:%p, pa 0x%08x, len %d):\n",
			       buff, (unsigned int)pa, len);
			mem_disp(buff, len);
		}
#endif /* CRYPT_APP_VERBOSE_DEBUG */

		buff -= MVAPPS_PKT_EFEC_OFFS;

		rp = COOKIE_GET_RX_PORT(sam_res_descs[i].cookie);
		tp = COOKIE_GET_TX_PORT(sam_res_descs[i].cookie);
		bp = COOKIE_GET_BPOOL(sam_res_descs[i].cookie);
		desc = &descs[tp][port_nums[tp]];
		shadow_q = &larg->ports_desc[tp].shadow_qs[tc];
		pp2_ppio_outq_desc_reset(desc);
		pp2_ppio_outq_desc_set_phys_addr(desc, pa);
		pp2_ppio_outq_desc_set_pkt_offset(desc, MVAPPS_PKT_EFEC_OFFS);
		pp2_ppio_outq_desc_set_pkt_len(desc, len);
		shadow_q->ents[shadow_q->write_ind].buff_ptr.cookie = (uintptr_t)buff;
		PP2_COOKIE_SET_ALL_INFO(shadow_q->ents[shadow_q->write_ind].buff_ptr.cookie, rp, tp, bp, 0);
		shadow_q->ents[shadow_q->write_ind].buff_ptr.addr = pa;
		shadow_q->write_ind++;
		if (shadow_q->write_ind == CRYPT_APP_TX_Q_SIZE)
			shadow_q->write_ind = 0;
		port_nums[tp]++;
	}

	for (tp = 0; tp < larg->garg->num_ports; tp++) {
		num = num_got = port_nums[tp];
		shadow_q = &larg->ports_desc[tp].shadow_qs[tc];
START_COUNT_CYCLES(pme_ev_cnt_tx);
		if (num_got) {
			err = pp2_ppio_send(larg->ports_desc[tp].ppio, larg->hif, tc, descs[tp], &num_got);
			if (err)
				return err;
		}
STOP_COUNT_CYCLES(pme_ev_cnt_tx, num_got);
#ifdef CRYPT_APP_VERBOSE_DEBUG
		if (larg->garg->verbose && num_got)
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
				bpool = larg->pools_desc[larg->ports_desc[COOKIE_GET_RX_PORT(binf->cookie)].pp_id]
								[COOKIE_GET_BPOOL(binf->cookie)].pool;
				PP2_COOKIE_CLEAR_ALL_INFO(binf->cookie);
				pp2_bpool_put_buff(larg->hif,
						   bpool,
						   binf);
			}
			larg->drop_cnt += num - num_got;
		}

		pp2_ppio_get_num_outq_done(larg->ports_desc[tp].ppio, larg->hif, tc, &num);
		for (i = 0; i < num; i++) {
			binf = &shadow_q->ents[shadow_q->read_ind].buff_ptr;
			if (unlikely(!binf->cookie || !binf->addr)) {
				pr_warn("Shadow memory @%d: cookie(%lx), pa(%lx)!\n",
					shadow_q->read_ind, (u64)binf->cookie, (u64)binf->addr);
				continue;
			}
			bpool = larg->pools_desc[larg->ports_desc[COOKIE_GET_RX_PORT(binf->cookie)].pp_id]
							[COOKIE_GET_BPOOL(binf->cookie)].pool;
			PP2_COOKIE_CLEAR_ALL_INFO(binf->cookie);
			pp2_bpool_put_buff(larg->hif,
					   bpool,
					   binf);
			shadow_q->read_ind++;
			if (shadow_q->read_ind == CRYPT_APP_TX_Q_SIZE)
				shadow_q->read_ind = 0;
		}
		larg->tx_cnt += num;
	}

	return 0;
}

static inline int deq_n_proc_pkts(struct local_arg	*larg,
				  struct sam_cio	*cio,
				  u8			 tc)
{
	struct sam_cio_op_result sam_res_descs[CRYPT_APP_MAX_BURST_SIZE];
	struct sam_cio_op_result sam_enc_res_descs[CRYPT_APP_MAX_BURST_SIZE];
	struct sam_cio_op_result sam_dec_res_descs[CRYPT_APP_MAX_BURST_SIZE];
	int			 err;
	u16			 i, num, num_enc, num_dec;

	num = CRYPT_APP_MAX_BURST_SIZE;
START_COUNT_CYCLES(pme_ev_cnt_deq);
	err = sam_cio_deq(cio, sam_res_descs, &num);
STOP_COUNT_CYCLES(pme_ev_cnt_deq, num);
	if (unlikely(err)) {
		pr_err("SAM DeQ (EnC) failed (%d)!\n", err);
		return -EFAULT;
	}
#ifdef CRYPT_APP_VERBOSE_CHECKS
	for (i = 0; i < num; i++) {
		if (sam_res_descs[i].status != SAM_CIO_OK) {
			struct pp2_bpool	*bpool;
			struct pp2_buff_inf	 binf;
			char			*buff = (char *)(uintptr_t)sam_res_descs[i].cookie;
			dma_addr_t		 pa;

			pr_warn("SAM operation (EnC) failed (%d)!\n", sam_res_descs[i].status);

			COOKIE_CLEAR_ALL_INFO(buff);
			pa = mv_sys_dma_mem_virt2phys(buff - MVAPPS_PKT_EFEC_OFFS);
			bpool = larg->pools_desc[larg->ports_desc[COOKIE_GET_RX_PORT(sam_res_descs[i].cookie)].pp_id]
							[COOKIE_GET_BPOOL(sam_res_descs[i].cookie)].pool;
			binf.addr = pa;
			binf.cookie = lower_32_bits((uintptr_t)(buff));
			pp2_bpool_put_buff(larg->hif, bpool, &binf);
			if (i < (num - 1)) {
				memcpy(&sam_res_descs[i],
				       &sam_res_descs[i + 1],
				       sizeof(struct sam_cio_op_result) * (num - i - 1));
				i--;
				num--;
			}
			larg->drop_cnt++;

			continue;
		}
		if (unlikely(!sam_res_descs[i].cookie)) {
			pr_err("SAM operation (EnC) failed (no cookie: %d,%d)!\n", i, sam_res_descs[i].out_len);
			return -EFAULT;
		}
	}
#endif /* CRYPT_APP_VERBOSE_CHECKS */

	for (i = num_enc = num_dec = 0; i < num; i++) {
		if (COOKIE_GET_DIR(sam_res_descs[i].cookie) == 0x1) {
			COOKIE_SET_DIR(sam_res_descs[i].cookie, 0);
			sam_enc_res_descs[num_enc++] = sam_res_descs[i];
		} else {
			sam_dec_res_descs[num_dec++] = sam_res_descs[i];
		}
	}

	if (num_enc) {
#ifdef CRYPT_APP_PKT_ECHO_SUPPORT
		if (likely(larg->echo))
			echo_pkts(larg, sam_enc_res_descs, num_enc);
#endif /* CRYPT_APP_PKT_ECHO_SUPPORT */

		err = dec_pkts(larg, sam_enc_res_descs, num_enc);
		if (unlikely(err))
			return err;
	}

	if (num_dec)
		return send_pkts(larg, tc, sam_dec_res_descs, num_dec);

	return 0;
}

static inline int loop_sw_recycle(struct local_arg	*larg,
				  u8			 rx_ppio_id,
				  u8			 tx_ppio_id,
				  u8			 tc,
				  u8			 qid,
				  u16			 num)
{
	struct pp2_ppio_desc	 descs[CRYPT_APP_MAX_BURST_SIZE];
	int			 err;

/*pr_info("tid %d check on tc %d, qid %d\n", larg->id, tc, qid);*/
START_COUNT_CYCLES(pme_ev_cnt_rx);
	err = pp2_ppio_recv(larg->ports_desc[rx_ppio_id].ppio, tc, qid, descs, &num);
STOP_COUNT_CYCLES(pme_ev_cnt_rx, num);
#ifdef CRYPT_APP_VERBOSE_DEBUG
	if (larg->garg->verbose && num)
		printf("got %d pkts on ppio %d, tc %d, qid %d\n", num, rx_ppio_id, tc, qid);
#endif /* CRYPT_APP_VERBOSE_DEBUG */

	if (num) {
		err = enc_pkts(larg, rx_ppio_id, tx_ppio_id, descs, num);
		if (unlikely(err))
			return err;
	}

	err = deq_n_proc_pkts(larg, larg->enc_cio, tc);
	if (unlikely(err))
		return err;
	if (larg->dec_cio != larg->enc_cio)
		return deq_n_proc_pkts(larg, larg->dec_cio, tc);
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

	num = larg->burst;

	while (*running) {
		/* Find next queue to consume */
		do {
			qid++;
			if (qid == MVAPPS_MAX_NUM_QS_PER_TC) {
				qid = 0;
				tc++;
				if (tc == CRYPT_APP_MAX_NUM_TCS_PER_PORT)
					tc = 0;
			}
		} while (!(larg->qs_map & (1 << ((tc * MVAPPS_MAX_NUM_QS_PER_TC) + qid))));

		err = loop_sw_recycle(larg, 0, 0, tc, qid, num);
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

	num = larg->burst;

	while (*running) {
		/* Find next queue to consume */
		do {
			qid++;
			if (qid == MVAPPS_MAX_NUM_QS_PER_TC) {
				qid = 0;
				tc++;
				if (tc == CRYPT_APP_MAX_NUM_TCS_PER_PORT)
					tc = 0;
			}
		} while (!(larg->qs_map & (1 << ((tc * MVAPPS_MAX_NUM_QS_PER_TC) + qid))));

		err  = loop_sw_recycle(larg, 0, 1, tc, qid, num);
		err |= loop_sw_recycle(larg, 1, 0, tc, qid, num);
		if (err != 0)
			return err;
	}

	return 0;
}

static int dump_perf(struct glob_arg *garg)
{
	struct timeval	 curr_time;
	u64		 tmp_time_inter;
	u32		 tmp_rx_cnt, tmp_tx_cnt, drop_cnt;
	int		 i;

	gettimeofday(&curr_time, NULL);
	tmp_time_inter = (curr_time.tv_sec - garg->ctrl_trd_last_time.tv_sec) * 1000;
	tmp_time_inter += (curr_time.tv_usec - garg->ctrl_trd_last_time.tv_usec) / 1000;

	drop_cnt = 0;
	for (i = 0; i < MVAPPS_MAX_NUM_CORES; i++)
		if (garg->largs[i])
			drop_cnt += garg->largs[i]->drop_cnt;
	tmp_rx_cnt = tmp_tx_cnt = 0;
	for (i = 0; i < MVAPPS_MAX_NUM_CORES; i++)
		if (garg->largs[i]) {
			tmp_rx_cnt += garg->largs[i]->rx_cnt;
			tmp_tx_cnt += garg->largs[i]->tx_cnt;
		}
	printf("Perf: %dKpps (Rx: %dKpps)",
	       (int)((tmp_tx_cnt - garg->lst_tx_cnt) / tmp_time_inter),
		(int)((tmp_rx_cnt - garg->lst_rx_cnt) / tmp_time_inter));
	garg->lst_rx_cnt = tmp_rx_cnt;
	garg->lst_tx_cnt = tmp_tx_cnt;
	if (drop_cnt)
		printf(", drop: %ull", drop_cnt);
	printf("\n");
	gettimeofday(&garg->ctrl_trd_last_time, NULL);

	return 0;
}

static int main_loop_cb(void *arg, int *running)
{
	struct local_arg	*larg = (struct local_arg *)arg;

	if (!larg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	if (larg->num_ports == 1)
		return loop_1p(larg, running);
	return loop_2ps(larg, running);
}

static int ctrl_cb(void *arg)
{
	struct glob_arg *garg = (struct glob_arg *)arg;
	struct timeval	 curr_time;
	u64		 tmp_time_inter;

	if (!garg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	gettimeofday(&curr_time, NULL);
	tmp_time_inter = (curr_time.tv_sec - garg->ctrl_trd_last_time.tv_sec) * 1000;
	tmp_time_inter += (curr_time.tv_usec - garg->ctrl_trd_last_time.tv_usec) / 1000;
	if (tmp_time_inter >= garg->ctrl_thresh)
		return dump_perf(garg);
	return 0;
}

static int init_all_modules(void)
{
	struct pp2_init_params	 pp2_params;
	int			 err;

	pr_info("Global initializations ...\n");

	err = mv_sys_dma_mem_init(CRYPT_APP_DMA_MEM_SIZE);
	if (err)
		return err;

	memset(&pp2_params, 0, sizeof(pp2_params));
	pp2_params.hif_reserved_map = MVAPPS_PP2_HIFS_RSRV;
	pp2_params.bm_pool_reserved_map = MVAPPS_PP2_BPOOLS_RSRV;
	/* Enable 10G port */
	pp2_params.ppios[0][0].is_enabled = 1;
	pp2_params.ppios[0][0].first_inq = 0;
	/* Enable 1G ports according to DTS files */
	if (garg.pp2_num_inst == 1) {
		pp2_params.ppios[0][2].is_enabled = 1;
		pp2_params.ppios[0][2].first_inq = 0;
	}
	if (garg.pp2_num_inst == 2) {
		/* Enable 10G port */
		pp2_params.ppios[1][0].is_enabled = 1;
		pp2_params.ppios[1][0].first_inq = 0;
		/* Enable 1G ports */
		pp2_params.ppios[1][1].is_enabled = 1;
		pp2_params.ppios[1][1].first_inq = 0;
	}
	err = pp2_init(&pp2_params);
	if (err)
		return err;

	pr_info("done\n");
	return 0;
}

static int create_sam_sessions(struct sam_cio		*enc_cio,
			       struct sam_cio		*dec_cio,
			       struct sam_sa		**enc_sa,
			       struct sam_sa		**dec_sa)
{
	struct sam_session_params	 sa_params;
	int				 err;
	u8				 cipher_key[] = RFC3602_AES128_CBC_T1_KEY;

	memset(&sa_params, 0, sizeof(sa_params));
	sa_params.dir = SAM_DIR_ENCRYPT;   /* operation direction: encode */
	sa_params.cipher_alg = SAM_CIPHER_AES;  /* cipher algorithm */
	sa_params.cipher_mode = SAM_CIPHER_CBC; /* cipher mode */
	sa_params.cipher_iv = NULL;     /* default IV */
	sa_params.cipher_key = cipher_key;    /* cipher key */
	sa_params.cipher_key_len = sizeof(cipher_key); /* cipher key size (in bytes) */
	sa_params.auth_alg = SAM_AUTH_NONE; /* authentication algorithm */
	sa_params.auth_inner = NULL;    /* pointer to authentication inner block */
	sa_params.auth_outer = NULL;    /* pointer to authentication outer block */
	sa_params.auth_icv_len = 0;   /* Integrity Check Value (ICV) size (in bytes) */
	sa_params.auth_aad_len = 0;   /* Additional Data (AAD) size (in bytes) */
	err = sam_session_create(enc_cio, &sa_params, enc_sa);
	if (err) {
		pr_err("EnC SA creation failed (%d)!\n", err);
		return err;
	}
	if (!*enc_sa) {
		pr_err("EnC SA creation failed!\n");
		return -EFAULT;
	}

	sa_params.dir = SAM_DIR_DECRYPT;   /* operation direction: decode */
	err = sam_session_create(dec_cio, &sa_params, dec_sa);
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

static int destroy_sam_sessions(struct sam_cio		*enc_cio,
				struct sam_cio		*dec_cio,
				struct sam_sa		*enc_sa,
				struct sam_sa		*dec_sa)
{
	int err;

	if (sam_session_destroy(enc_sa))
		pr_err("DeC SA destroy failed!\n");
	if (sam_session_destroy(dec_sa))
		pr_err("EnC SA destroy failed!\n");

	err = sam_cio_flush(enc_cio);
	err = sam_cio_flush(dec_cio);

	return err;
}

static int init_local_modules(struct glob_arg *garg)
{
	struct sam_cio_params		cio_params;
	char				name[15];
	int				err, port_index, cio_id;
	struct bpool_inf		std_infs[] = CRYPT_APP_BPOOLS_INF;
	struct bpool_inf		jumbo_infs[] = CRYPT_APP_BPOOLS_JUMBO_INF;
	struct bpool_inf		*infs;

	pr_info("Specific modules initializations\n");

	err = app_hif_init(&garg->hif, CRYPT_APP_HIF_Q_SIZE);
	if (err)
		return err;

	if (garg->mtu > DEFAULT_MTU) {
		infs = jumbo_infs;
		garg->num_pools = ARRAY_SIZE(jumbo_infs);
	} else {
		infs = std_infs;
		garg->num_pools = ARRAY_SIZE(std_infs);
	}

	err = app_build_all_bpools(&garg->pools_desc, garg->num_pools, infs, garg->hif);
	if (err)
		return err;

	for (port_index = 0; port_index < garg->num_ports; port_index++) {
		struct port_desc *port = &garg->ports_desc[port_index];

		err = app_find_port_info(port);
		if (!err) {
			port->ppio_type	= PP2_PPIO_T_NIC;
			port->num_tcs	= CRYPT_APP_MAX_NUM_TCS_PER_PORT;
			port->num_inqs	= MVAPPS_MAX_NUM_QS_PER_TC;
			port->inq_size	= CRYPT_APP_RX_Q_SIZE;
			port->num_outqs	= CRYPT_APP_MAX_NUM_TCS_PER_PORT;
			port->outq_size	= CRYPT_APP_TX_Q_SIZE;

			err = app_port_init(port, garg->num_pools, garg->pools_desc[port->pp_id], garg->mtu);
			if (err) {
				pr_err("Failed to initialize port %d (pp_id: %d)\n", port_index,
				       port->pp_id);
				return err;
			}
		} else {
			return err;
		}
	}

	/* In that stage, we're initializaing CIO for global-arg just in order
	 * to enforce the initialization of the engine.
	 * TODO: in the future, replace the below code with appropraite initialization of the engine.
	 */
	cio_id = find_free_cio(0);
	if (cio_id < 0) {
		pr_err("free CIO not found!\n");
		return cio_id;
	}
	memset(name, 0, sizeof(name));
	snprintf(name, sizeof(name), "cio-%d:%d", 0, cio_id);
	pr_debug("found cio: %s\n", name);
	memset(&cio_params, 0, sizeof(cio_params));
	cio_params.match = name;
	cio_params.size = CRYPT_APP_CIO_Q_SIZE;
	cio_params.num_sessions = CRYPT_APP_MAX_NUM_SESSIONS_PER_RING;
	cio_params.max_buf_size = garg->mtu + 64;
	err = sam_cio_init(&cio_params, &garg->cio);
	if (err != 0)
		return err;
	if (!garg->cio) {
		pr_err("CIO init failed!\n");
		return -EIO;
	}

	return 0;
}

static void destroy_local_modules(struct glob_arg *garg)
{
	app_disable_all_ports(garg->ports_desc, garg->num_ports,
			      CRYPT_APP_MAX_NUM_TCS_PER_PORT, MVAPPS_MAX_NUM_QS_PER_TC);
	app_free_all_pools(garg->pools_desc, garg->num_pools, garg->hif);
	app_deinit_all_ports(garg->ports_desc, garg->num_ports);

	if (garg->hif)
		pp2_hif_deinit(garg->hif);
}

static void destroy_all_modules(void)
{
	pp2_deinit();
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
		printf("%d\n", garg->prefetch_shift);
		return 0;
	}

	garg->prefetch_shift = atoi(argv[1]);

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

	return dump_perf(garg);
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
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))prefetch_cmd_cb;
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

	if (garg->cli) {
		err = register_cli_cmds(garg);
		if (err)
			return err;
	}

	gettimeofday(&garg->ctrl_trd_last_time, NULL);

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
	if (garg->cli)
		unregister_cli_cmds(garg);
	destroy_local_modules(garg);
	destroy_all_modules();
}

static int init_local(void *arg, int id, void **_larg)
{
	struct glob_arg		*garg = (struct glob_arg *)arg;
	struct local_arg	*larg;
	struct sam_cio_params	 cio_params;
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

	pthread_mutex_lock(&garg->trd_lock);
	err = app_hif_init(&larg->hif, CRYPT_APP_HIF_Q_SIZE);
	pthread_mutex_unlock(&garg->trd_lock);
	if (err)
		return err;

	if (id == 0) {
		larg->enc_cio = garg->cio;
	} else {
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
		cio_params.num_sessions = CRYPT_APP_MAX_NUM_SESSIONS_PER_RING;
		cio_params.max_buf_size = garg->mtu + 64;
		err = sam_cio_init(&cio_params, &larg->enc_cio);
		pthread_mutex_unlock(&garg->trd_lock);
		if (err != 0)
			return err;
	}
	if (!larg->enc_cio) {
		pr_err("CIO init failed!\n");
		return -EIO;
	}
#ifdef CRYPT_APP_SAM_TWO_ENG_SUPPORT
	if (garg->pp2_num_inst == 2) {
		pthread_mutex_lock(&garg->trd_lock);
		cio_id = find_free_cio(1);
		if (cio_id < 0) {
			pr_err("free CIO not found!\n");
			pthread_mutex_unlock(&garg->trd_lock);
			return cio_id;
		}
		memset(name, 0, sizeof(name));
		snprintf(name, sizeof(name), "cio-%d:%d", 1, cio_id);
		pr_info("found cio: %s\n", name);
		memset(&cio_params, 0, sizeof(cio_params));
		cio_params.match = name;
		cio_params.size = CRYPT_APP_CIO_Q_SIZE;
		cio_params.num_sessions = CRYPT_APP_MAX_NUM_SESSIONS_PER_RING;
		cio_params.max_buf_size = garg->mtu + 64;
		err = sam_cio_init(&cio_params, &larg->dec_cio);
		pthread_mutex_unlock(&garg->trd_lock);
		if (err != 0)
			return err;
	} else
#endif /* CRYPT_APP_SAM_TWO_ENG_SUPPORT */
		larg->dec_cio = larg->enc_cio;

	larg->id                = id;
	larg->burst		= garg->burst;
	larg->echo              = garg->echo;
	larg->num_ports         = garg->num_ports;
	larg->ports_desc = (struct lcl_port_desc *)malloc(larg->num_ports * sizeof(struct lcl_port_desc));
	if (!larg->ports_desc) {
		pr_err("no mem for local-port-desc obj!\n");
		return -ENOMEM;
	}
	memset(larg->ports_desc, 0, larg->num_ports * sizeof(struct lcl_port_desc));
	for (i = 0; i < larg->num_ports; i++)
		app_port_local_init(i, larg->id, &larg->ports_desc[i], &garg->ports_desc[i]);

	larg->pools_desc             = garg->pools_desc;

	err = create_sam_sessions(larg->enc_cio, larg->dec_cio, &larg->enc_sa, &larg->dec_sa);
	if (err)
		return err;
	if (!larg->enc_sa || !larg->dec_sa) {
		pr_err("failed to create SAM sesions!\n");
		return -EFAULT;
	}

	larg->garg              = garg;
	garg->largs[larg->id]   = larg;

	larg->qs_map = garg->qs_map << (garg->qs_map_shift * id);
	pr_info("thread %d (cpu %d) mapped to Qs %llx using %s\n",
		larg->id, sched_getcpu(), (unsigned int long long)larg->qs_map, name);

	*_larg = larg;
	return 0;
}

static void deinit_local(void *arg)
{
	struct local_arg *larg = (struct local_arg *)arg;
	struct sam_cio_stats cio_stats;
	int i;

	if (!larg)
		return;

	destroy_sam_sessions(larg->enc_cio, larg->dec_cio, larg->enc_sa, larg->dec_sa);

	if (!sam_cio_stats_get(larg->enc_cio, &cio_stats, 1)) {
		printf("Enqueue packets             : %lu packets\n", cio_stats.enq_pkts);
		printf("Enqueue bytes               : %lu bytes\n", cio_stats.enq_bytes);
		printf("Enqueue full                : %lu times\n", cio_stats.enq_full);
		printf("Dequeue packets             : %lu packets\n", cio_stats.deq_pkts);
		printf("Dequeue bytes               : %lu bytes\n", cio_stats.deq_bytes);
		printf("Dequeue empty               : %lu times\n", cio_stats.deq_empty);
		printf("Created sessions            : %lu\n", cio_stats.sa_add);
		printf("Deleted sessions:	    : %lu\n", cio_stats.sa_del);
		printf("Invalidated sessions:	    : %lu\n", cio_stats.sa_inv);
	} else {
		printf("Failed to get sam_cio_stats_get!!!\n");
	}

	if (larg->ports_desc) {
		for (i = 0; i < larg->num_ports; i++)
			app_port_local_deinit(&larg->ports_desc[i]);
		free(larg->ports_desc);
	}

	if (larg->dec_cio != larg->enc_cio)
		sam_cio_deinit(larg->dec_cio);
	if (larg->enc_cio)
		sam_cio_deinit(larg->enc_cio);

	if (larg->hif)
		pp2_hif_deinit(larg->hif);
	free(larg);
}

static void usage(char *progname)
{
	printf("\n"
	       "MUSDK packet-echo application.\n"
	       "\n"
	       "Usage: %s OPTIONS\n"
	       "  E.g. %s -i eth0,eth1 -c 1\n"
	       "\n"
	       "Mandatory OPTIONS:\n"
	       "\t-i, --interface <Eth-interfaces> (comma-separated, no spaces)\n"
	       "                  Interface count min 1, max %i\n"
	       "\n"
	       "Optional OPTIONS:\n"
	       "\t-b <size>                Burst size (default is %d)\n"
	       "\t-c, --cores <number>     Number of CPUs to use.\n"
	       "\t-a, --affinity <number>  Use setaffinity (default is no affinity).\n"
	       "\t-t <mtu>                 Set MTU (default is %d)\n"
#ifdef CRYPT_APP_VERBOSE_DEBUG
	       "\t-v                       Enable verbose debug\n"
#endif /* CRYPT_APP_VERBOSE_DEBUG */
	       "\t--no-echo                No Echo packets\n"
	       "\t--cli                    Use CLI\n"
	       "\t?, -h, --help            Display help and exit.\n\n"
	       "\n", MVAPPS_NO_PATH(progname), MVAPPS_NO_PATH(progname),
	       CRYPT_APP_MAX_NUM_PORTS, CRYPT_APP_MAX_BURST_SIZE, DEFAULT_MTU);
}

static int parse_args(struct glob_arg *garg, int argc, char *argv[])
{
	int	i = 1;

#ifdef CRYPT_APP_VERBOSE_DEBUG
	garg->verbose = 0;
#endif /* CRYPT_APP_VERBOSE_DEBUG */
	garg->cli = 0;
	garg->cpus = 1;
	garg->affinity = -1;
	garg->burst = CRYPT_APP_DFLT_BURST_SIZE;
	garg->mtu = DEFAULT_MTU;
	garg->echo = 1;
	garg->qs_map = 0;
	garg->qs_map_shift = 0;
	garg->prefetch_shift = CRYPT_APP_PREFETCH_SHIFT;
	garg->pp2_num_inst = pp2_get_num_inst();
	garg->ctrl_thresh = CRYPT_APP_CTRL_DFLT_THR;

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
			for (token = strtok(argv[i + 1], ","), garg->num_ports = 0;
			     token;
			     token = strtok(NULL, ","), garg->num_ports++)
				snprintf(garg->ports_desc[garg->num_ports].name,
					 sizeof(garg->ports_desc[garg->num_ports].name),
					 "%s", token);

			if (garg->num_ports == 0) {
				pr_err("Invalid interface arguments format!\n");
				return -EINVAL;
			} else if (garg->num_ports > CRYPT_APP_MAX_NUM_PORTS) {
				pr_err("too many ports specified (%d vs %d)\n",
				       garg->num_ports, CRYPT_APP_MAX_NUM_PORTS);
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
			garg->burst = atoi(argv[i + 1]);
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
			garg->mtu = atoi(argv[i + 1]);
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
			garg->cpus = atoi(argv[i + 1]);
			i += 2;
		} else if (strcmp(argv[i], "-a") == 0) {
			garg->affinity = atoi(argv[i + 1]);
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
			garg->qs_map = strtoul(token, NULL, 16);
			token = strtok(NULL, "");
			garg->qs_map_shift = atoi(token);
			i += 2;
#ifdef CRYPT_APP_VERBOSE_DEBUG
		} else if (strcmp(argv[i], "-v") == 0) {
			garg->verbose = 1;
			i += 1;
#endif /* CRYPT_APP_VERBOSE_DEBUG */
		} else if (strcmp(argv[i], "--no-echo") == 0) {
			garg->echo = 0;
			i += 1;
		} else if (strcmp(argv[i], "--cli") == 0) {
			garg->cli = 1;
			i += 1;
		} else {
			pr_err("argument (%s) not supported!\n", argv[i]);
			return -EINVAL;
		}
	}

	/* Now, check validity of all inputs */
	if (!garg->num_ports ||
	    !garg->ports_desc[0].name) {
		pr_err("No port defined!\n");
		return -EINVAL;
	}
	if (garg->burst > CRYPT_APP_MAX_BURST_SIZE) {
		pr_err("illegal burst size requested (%d vs %d)!\n",
		       garg->burst, CRYPT_APP_MAX_BURST_SIZE);
		return -EINVAL;
	}
	if (garg->cpus > MVAPPS_MAX_NUM_CORES) {
		pr_err("illegal num cores requested (%d vs %d)!\n",
		       garg->cpus, MVAPPS_MAX_NUM_CORES);
		return -EINVAL;
	}
	if ((garg->affinity != -1) &&
	    ((garg->cpus + garg->affinity) > MVAPPS_MAX_NUM_CORES)) {
		pr_err("illegal num cores or affinity requested (%d,%d vs %d)!\n",
		       garg->cpus, garg->affinity, MVAPPS_MAX_NUM_CORES);
		return -EINVAL;
	}

	if (garg->qs_map &&
	    (MVAPPS_MAX_NUM_QS_PER_TC == 1) &&
	    (CRYPT_APP_MAX_NUM_TCS_PER_PORT == 1)) {
		pr_warn("no point in queues-mapping; ignoring.\n");
		garg->qs_map = 1;
		garg->qs_map_shift = 1;
	} else if (!garg->qs_map) {
		garg->qs_map = 1;
		garg->qs_map_shift = CRYPT_APP_MAX_NUM_TCS_PER_PORT;
	}

	if ((garg->cpus != 1) &&
	    (garg->qs_map & (garg->qs_map << garg->qs_map_shift))) {
		pr_err("Invalid queues-mapping (ovelapping CPUs)!\n");
		return -EINVAL;
	}

	if (garg->prefetch_shift > garg->burst)
		garg->prefetch_shift = garg->burst - 1;

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
	if (err)
		return err;

	cores_mask = 0;
	for (i = 0; i < garg.cpus; i++, cores_mask <<= 1, cores_mask |= 1)
		;
	cores_mask <<= (garg.affinity != -1) ? garg.affinity : 0;

	memset(&mvapp_params, 0, sizeof(mvapp_params));
	mvapp_params.use_cli		= garg.cli;
	mvapp_params.num_cores		= garg.cpus;
	mvapp_params.cores_mask		= cores_mask;
	mvapp_params.global_arg		= (void *)&garg;
	mvapp_params.init_global_cb	= init_global;
	mvapp_params.deinit_global_cb	= deinit_global;
	mvapp_params.init_local_cb	= init_local;
	mvapp_params.deinit_local_cb	= deinit_local;
	mvapp_params.main_loop_cb	= main_loop_cb;
	if (!mvapp_params.use_cli)
		mvapp_params.ctrl_cb	= ctrl_cb;
	return mvapp_go(&mvapp_params);
}
