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

#include "mvapp.h"
#include "perf_mon_emu.h"

#define Q_SIZE		256/*1024*/
#define MAX_BURST_SIZE	((Q_SIZE)>>2)
#define DFLT_BURST_SIZE	64
#define CTRL_DFLT_THR	1000
#define PKT_OFFS	64
#define PKT_EFEC_OFFS	(PKT_OFFS+PP2_MH_SIZE)
#define MAX_NUM_CORES	4
#define DMA_MEM_SIZE	(48*1024*1024)
#define PP2_BPOOLS_RSRV	0x7
#define PP2_HIFS_RSRV	0xF
#define SAM_CIOS_RSRV	{ 0x0, 0x0 }
#define PP2_MAX_NUM_PORTS		2
#define PP2_MAX_NUM_TCS_PER_PORT	1
#define PP2_MAX_NUM_QS_PER_TC		MAX_NUM_CORES
#define MAX_NUM_QS_PER_CORE		PP2_MAX_NUM_TCS_PER_PORT
#define SAM_MAX_NUM_SESSIONS_PER_RING	4
#define SAM_TWO_ENG_SUPPORT

/* TODO: find more generic way to get the following parameters */
#define PP2_TOTAL_NUM_BPOOLS	16
#define PP2_TOTAL_NUM_HIFS	9
#define SAM_TOTAL_NUM_CIOS	4

/*#define VERBOSE_CHECKS*/
#define PKT_ECHO_SUPPORT
#define PREFETCH_SHIFT	4
/*#define VERBOSE_DEBUG*/

#define DEFAULT_MTU			1500
#define VLAN_HLEN			4
#define ETH_HLEN			14
#define ETH_FCS_LEN			4

#define MVPP2_MTU_TO_MRU(mtu) \
	((mtu) + PP2_MH_SIZE + VLAN_HLEN + \
	ETH_HLEN + ETH_FCS_LEN)

#define MVPP2_MRU_TO_MTU(mru) \
	((mru) - PP2_MH_SIZE - VLAN_HLEN - \
	ETH_HLEN - ETH_FCS_LEN)

/** Get rid of path in filename - only for unix-type paths using '/' */
#define NO_PATH(file_name) (strrchr((file_name), '/') ? \
			    strrchr((file_name), '/') + 1 : (file_name))

/*#define BPOOLS_INF	{ {384, 4096}, {2048, 1024} }*/
#define BPOOLS_INF	{ {2048, 4096} }
/*#define BPOOLS_JUMBO_INF	{ {2048, 4096}, {10240, 512} }*/
#define BPOOLS_JUMBO_INF	{ {10240, 512} }

#define RFC3602_AES128_CBC_T1_KEY {			\
	0x06, 0xa9, 0x21, 0x40, 0x36, 0xb8, 0xa1, 0x5b,	\
	0x51, 0x2e, 0x03, 0xd5, 0x34, 0x12, 0x00, 0x06	\
}

#define RFC3602_AES128_CBC_T1_IV {			\
	0x3d, 0xaf, 0xba, 0x42, 0x9d, 0x9e, 0xb4, 0x30,	\
	0xb4, 0x22, 0xda, 0x80, 0x2c, 0x9f, 0xac, 0x41	\
}


struct port_desc {
	char		 name[15];
	int		 pp_id;
	int		 ppio_id;
	struct pp2_ppio	*port;
};

struct lcl_port_desc {
	int		 pp_id;
	int		 ppio_id;
	struct pp2_ppio	*port;
};

struct bpool_inf {
	int	buff_size;
	int	num_buffs;
};

struct tx_shadow_q_entry {
	struct pp2_buff_inf	buff_ptr;
};

struct tx_shadow_q {
	u16				 read_ind;
	u16				 write_ind;

	struct tx_shadow_q_entry	 ents[Q_SIZE];
};

struct local_arg;

struct glob_arg {
#ifdef VERBOSE_DEBUG
	int			 verbose;
#endif /* VERBOSE_DEBUG */
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
	struct port_desc	 ports_desc[PP2_MAX_NUM_PORTS];

	pthread_mutex_t		 trd_lock;

	struct pp2_hif		*hif;

	int			 num_pools;
	struct pp2_bpool	***pools;
	struct pp2_buff_inf	***buffs_inf;

	int			 ctrl_thresh;
	struct timeval		 ctrl_trd_last_time;
	u64			 lst_rx_cnt;
	u64			 lst_tx_cnt;

	struct sam_cio		*cio;
	struct local_arg	*largs[MAX_NUM_CORES];
};

struct local_arg {
	struct tx_shadow_q	 shadow_qs[MAX_NUM_QS_PER_CORE];
	u64			 qs_map;

	struct pp2_hif		*hif;
	int			 num_ports;
	struct lcl_port_desc	*ports_desc;

	struct pp2_bpool	***pools;

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
static u64 sys_dma_high_addr;

static u16	used_bpools = PP2_BPOOLS_RSRV;
static u16	used_hifs = PP2_HIFS_RSRV;
static u8	used_cios[] = SAM_CIOS_RSRV;

#define CHECK_CYCLES
#ifdef CHECK_CYCLES
static int pme_ev_cnt_rx = -1, pme_ev_cnt_enq = -1, pme_ev_cnt_deq = -1, pme_ev_cnt_tx = -1;

#define START_COUNT_CYCLES(_ev_cnt)		pme_ev_cnt_start(_ev_cnt)
#define STOP_COUNT_CYCLES(_ev_cnt, _num)	pme_ev_cnt_stop(_ev_cnt, _num)

#else
#define START_COUNT_CYCLES(_ev_cnt)
#define STOP_COUNT_CYCLES(_ev_cnt, _num)
#endif /* CHECK_CYCLES */


#ifdef PKT_ECHO_SUPPORT
static inline void swap_l2(char *buf)
{
	uint16_t *eth_hdr;

	register uint16_t tmp;

	eth_hdr = (uint16_t *)buf;
	tmp = eth_hdr[0];
	eth_hdr[0] = eth_hdr[3];
	eth_hdr[3] = tmp;
	tmp = eth_hdr[1];
	eth_hdr[1] = eth_hdr[4];
	eth_hdr[4] = tmp;
	tmp = eth_hdr[2];
	eth_hdr[2] = eth_hdr[5];
	eth_hdr[5] = tmp;
}

static inline void swap_l3(char *buf)
{
	register uint32_t tmp32;

	buf += 14 + 12;
	tmp32 = ((uint32_t *)buf)[0];
	((uint32_t *)buf)[0] = ((uint32_t *)buf)[1];
	((uint32_t *)buf)[1] = tmp32;
}
#endif /* PKT_ECHO_SUPPORT */

static inline void prefetch(const void *ptr)
{
	asm volatile("prfm pldl1keep, %a0\n" : : "p" (ptr));
}

static inline void free_not_sent_buffers(struct local_arg	*larg,
					 struct pp2_bpool	*bpool,
					 struct pp2_ppio_desc	*descs,
					 u16			num)
{
	int i;
	struct pp2_buff_inf binf;

	for (i = 0; i < num; i++) {
		binf.addr = pp2_ppio_inq_desc_get_phys_addr(&descs[i]);
		binf.cookie = pp2_ppio_inq_desc_get_cookie(&descs[i]);
		pp2_bpool_put_buff(larg->hif, bpool, &binf);
	}
	larg->drop_cnt += num;
}

#ifdef PKT_ECHO_SUPPORT
static inline void echo_pkts(struct local_arg		*larg,
			    struct sam_cio_op_result	*sam_res_descs,
			    u16				 num)
{
	char			*tmp_buff;
	int			 prefetch_shift = larg->garg->prefetch_shift;
	u16			 i;

	for (i = 0; i < num; i++) {
		if (num-i > prefetch_shift)
			prefetch((char *)(uintptr_t)sam_res_descs[i+prefetch_shift].cookie);
		tmp_buff = (char *)(uintptr_t)sam_res_descs[i].cookie;
#ifdef VERBOSE_DEBUG
		if (larg->garg->verbose) {
			printf("pkt before echo (len %d):\n",
				sam_res_descs[i].out_len - PKT_EFEC_OFFS);
			mem_disp(tmp_buff, sam_res_descs[i].out_len - PKT_EFEC_OFFS);
		}
#endif /* VERBOSE_DEBUG */
		swap_l2(tmp_buff);
		swap_l3(tmp_buff);
		/*printf("pkt after echo:\n");*/
		/*mem_disp(tmp_buff, sam_res_descs[i].out_len  - PKT_EFEC_OFFS);*/
	}
}
#endif /* PKT_ECHO_SUPPORT */

static inline int enc_pkts(struct local_arg		*larg,
			    u8				 rx_ppio_id,
			    u8				 bpool_id,
			    struct pp2_ppio_desc	*descs,
			    u16				 num)
{
	struct sam_cio_op_params sam_descs[MAX_BURST_SIZE];
	struct sam_buf_info	 src_buf_infs[MAX_BURST_SIZE];
	struct sam_buf_info	 dst_buf_infs[MAX_BURST_SIZE];
	int			 err;
	u16			 i, bpool_buff_len, num_got;
	u8			 cipher_iv[] = RFC3602_AES128_CBC_T1_IV;
	u8			 l4_offs = (14 + 20); /* TODO: hardcoded for eth/ipv4! */

	/* TODO: is this enough?!?!?! */
	bpool_buff_len = larg->garg->mtu+64;

	larg->rx_cnt += num;

	memset(sam_descs, 0, sizeof(sam_descs));

	for (i = 0; i < num; i++) {
		src_buf_infs[i].vaddr = (char *)(uintptr_t)pp2_ppio_inq_desc_get_cookie(&descs[i]);
		src_buf_infs[i].paddr = pp2_ppio_inq_desc_get_phys_addr(&descs[i]);
		/* source buffer length is received packet size + headroom size */
		src_buf_infs[i].len = pp2_ppio_inq_desc_get_pkt_len(&descs[i]) + PKT_OFFS;

		src_buf_infs[i].vaddr += PKT_EFEC_OFFS;
		src_buf_infs[i].vaddr = (char *)(((uintptr_t)(src_buf_infs[i].vaddr))|sys_dma_high_addr);

#ifdef VERBOSE_DEBUG
		if (larg->garg->verbose) {
			printf("Received packet (va:%p, pa 0x%08x, len %d):\n",
				src_buf_infs[i].vaddr,
				(unsigned int)src_buf_infs[i].paddr,
				src_buf_infs[i].len-PKT_EFEC_OFFS);
			mem_disp(src_buf_infs[i].vaddr, src_buf_infs[i].len-PKT_EFEC_OFFS);
		}
#endif /* VERBOSE_DEBUG */

		/* Mark the cookie with '1' (first bit) so we know it is for encrypt */
		dst_buf_infs[i].vaddr = src_buf_infs[i].vaddr;
		dst_buf_infs[i].paddr = src_buf_infs[i].paddr;
		dst_buf_infs[i].len = bpool_buff_len;

		sam_descs[i].sa = larg->enc_sa;
		sam_descs[i].cookie = dst_buf_infs[i].vaddr + 1;
		sam_descs[i].num_bufs = 1;
		sam_descs[i].src = &src_buf_infs[i];
		sam_descs[i].dst = &dst_buf_infs[i];
		sam_descs[i].cipher_iv = cipher_iv;
		sam_descs[i].cipher_offset = PKT_EFEC_OFFS + l4_offs;
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

		bpool = larg->pools[larg->ports_desc[rx_ppio_id].pp_id][bpool_id];
		free_not_sent_buffers(larg, bpool, &descs[num_got], num-num_got);
	}

	return 0;
}

static inline int dec_pkts(struct local_arg		*larg,
			    u8				 rx_ppio_id,
			    u8				 bpool_id,
			    struct sam_cio_op_result	*sam_res_descs,
			    u16				 num)
{
	struct sam_cio_op_params sam_descs[MAX_BURST_SIZE];
	struct sam_buf_info	 src_buf_infs[MAX_BURST_SIZE];
	struct sam_buf_info	 dst_buf_infs[MAX_BURST_SIZE];
	int			 err;
	u16			 i, bpool_buff_len, num_got;
	u8			 cipher_iv[] = RFC3602_AES128_CBC_T1_IV;
	u8			 l4_offs = (14 + 20); /* TODO: hardcoded for eth/ipv4! */

	/* TODO: is this enough?!?!?! */
	bpool_buff_len = larg->garg->mtu+64;

	memset(sam_descs, 0, sizeof(sam_descs));

	/* For decryption crypto parameters are caclulated from cookie and out_len */
	for (i = 0; i < num; i++) {
		src_buf_infs[i].vaddr = (char *)(uintptr_t)sam_res_descs[i].cookie;
		src_buf_infs[i].paddr = mv_sys_dma_mem_virt2phys(src_buf_infs[i].vaddr-PKT_EFEC_OFFS);
		src_buf_infs[i].len = sam_res_descs[i].out_len;

		dst_buf_infs[i].vaddr = src_buf_infs[i].vaddr;
		dst_buf_infs[i].paddr = src_buf_infs[i].paddr;
		dst_buf_infs[i].len = bpool_buff_len;

		sam_descs[i].sa = larg->dec_sa;
		sam_descs[i].cookie = dst_buf_infs[i].vaddr;
		sam_descs[i].num_bufs = 1;
		sam_descs[i].src = &src_buf_infs[i];
		sam_descs[i].dst = &dst_buf_infs[i];
		sam_descs[i].cipher_iv = cipher_iv;
		sam_descs[i].cipher_offset = PKT_EFEC_OFFS + l4_offs;
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
		struct pp2_ppio_desc	 descs[MAX_BURST_SIZE];

		bpool = larg->pools[larg->ports_desc[rx_ppio_id].pp_id][bpool_id];
		for (i = 0; i < num - num_got; i++) {
			char			*buff = (char *)(uintptr_t)sam_descs[i].cookie;
			dma_addr_t		 pa = mv_sys_dma_mem_virt2phys(buff-PKT_EFEC_OFFS);

			pp2_ppio_outq_desc_set_phys_addr(&descs[i], pa);
			pp2_ppio_outq_desc_set_cookie(&descs[i], lower_32_bits((uintptr_t)(buff)));
		}
		free_not_sent_buffers(larg, bpool, descs, num-num_got);
	}

	return 0;
}

static inline int send_pkts(struct local_arg		*larg,
			    u8				 rx_ppio_id,
			    u8				 tx_ppio_id,
			    u8				 bpool_id,
			    u8				 tc,
			    struct sam_cio_op_result	*sam_res_descs,
			    u16				 num)
{
	struct pp2_bpool	*bpool;
	struct tx_shadow_q	*shadow_q;
	struct pp2_buff_inf	*binf;
	struct pp2_ppio_desc	 descs[MAX_BURST_SIZE];
	int			 err;
	u16			 i, num_got;

	bpool = larg->pools[larg->ports_desc[rx_ppio_id].pp_id][bpool_id];
	shadow_q = &(larg->shadow_qs[tc]);

	for (i = 0; i < num; i++) {
		char			*buff = (char *)(uintptr_t)sam_res_descs[i].cookie;
		dma_addr_t		 pa = mv_sys_dma_mem_virt2phys(buff-PKT_EFEC_OFFS);
		/* TODO: size is incorrect!!! */
		u16			 len = sam_res_descs[i].out_len - PKT_EFEC_OFFS;

#ifdef VERBOSE_DEBUG
		if (larg->garg->verbose) {
			printf("Sending packet (va:%p, pa 0x%08x, len %d):\n",
				buff, (unsigned int)pa, len);
			mem_disp(buff, len);
		}
#endif /* VERBOSE_DEBUG */

		buff -= PKT_EFEC_OFFS;

		pp2_ppio_outq_desc_reset(&descs[i]);
		pp2_ppio_outq_desc_set_phys_addr(&descs[i], pa);
		pp2_ppio_outq_desc_set_pkt_offset(&descs[i], PKT_EFEC_OFFS);
		pp2_ppio_outq_desc_set_pkt_len(&descs[i], len);
		shadow_q->ents[shadow_q->write_ind].buff_ptr.cookie = (uintptr_t)buff;
		shadow_q->ents[shadow_q->write_ind].buff_ptr.addr = pa;
		shadow_q->write_ind++;
		if (shadow_q->write_ind == Q_SIZE)
			shadow_q->write_ind = 0;
	}

	num_got = num;
#ifdef VERBOSE_DEBUG
	if (larg->garg->verbose && num_got)
		printf("send %d pkts on ppio %d, tc %d\n", num_got, tx_ppio_id, tc);
#endif /* VERBOSE_DEBUG */
START_COUNT_CYCLES(pme_ev_cnt_tx);
	if (num_got) {
		err = pp2_ppio_send(larg->ports_desc[tx_ppio_id].port, larg->hif, tc, descs, &num_got);
		if (err)
			return err;
	}
STOP_COUNT_CYCLES(pme_ev_cnt_tx, num_got);
	if (num_got < num)
		free_not_sent_buffers(larg, bpool, &descs[num_got], num-num_got);

	pp2_ppio_get_num_outq_done(larg->ports_desc[tx_ppio_id].port, larg->hif, tc, &num);
	for (i = 0; i < num; i++) {
		binf = &(shadow_q->ents[shadow_q->read_ind].buff_ptr);
		if (unlikely(!binf->cookie || !binf->addr)) {
			pr_warn("Shadow memory @%d: cookie(%lx), pa(%lx)!\n",
				   shadow_q->read_ind, (u64)binf->cookie, (u64)binf->addr);
			continue;
		}
		pp2_bpool_put_buff(larg->hif,
				   bpool,
				   binf);
		shadow_q->read_ind++;
		if (shadow_q->read_ind == Q_SIZE)
			shadow_q->read_ind = 0;
	}
	larg->tx_cnt += num;

	return 0;
}

static inline int deq_n_proc_pkts(struct local_arg	*larg,
				  struct sam_cio	*cio,
				  u8			 rx_ppio_id,
				  u8			 tx_ppio_id,
				  u8			 bpool_id,
				  u8			 tc)
{
	struct sam_cio_op_result sam_res_descs[MAX_BURST_SIZE];
	struct sam_cio_op_result sam_enc_res_descs[MAX_BURST_SIZE];
	struct sam_cio_op_result sam_dec_res_descs[MAX_BURST_SIZE];
	int			 err;
	u16			 i, num, num_enc, num_dec;

	num = MAX_BURST_SIZE;
START_COUNT_CYCLES(pme_ev_cnt_deq);
	err = sam_cio_deq(cio, sam_res_descs, &num);
STOP_COUNT_CYCLES(pme_ev_cnt_deq, num);
	if (unlikely(err)) {
		pr_err("SAM DeQ (EnC) failed (%d)!\n", err);
		return -EFAULT;
	}
#ifdef VERBOSE_CHECKS
	for (i = 0; i < num; i++) {
		if (sam_res_descs[i].status != SAM_CIO_OK) {
			pr_err("SAM operation (EnC) failed (%d)!\n", sam_res_descs[i].status);
			return -EFAULT;
		}
		if (!sam_res_descs[i].cookie) {
			pr_err("SAM operation (EnC) failed (no cookie: %d,%d)!\n", i, sam_res_descs[i].out_len);
			return -EFAULT;
		}
	}
#endif /* VERBOSE_CHECKS */

	for (i = num_enc = num_dec = 0; i < num; i++) {
		if ((uintptr_t)sam_res_descs[i].cookie & 0x1) {
			sam_res_descs[i].cookie = (void *)((uintptr_t)sam_res_descs[i].cookie & ~0x1);
			sam_enc_res_descs[num_enc++] = sam_res_descs[i];
		} else
			sam_dec_res_descs[num_dec++] = sam_res_descs[i];
	}

	if (num_enc) {
#ifdef PKT_ECHO_SUPPORT
		if (likely(larg->echo))
			echo_pkts(larg, sam_enc_res_descs, num_enc);
#endif /* PKT_ECHO_SUPPORT */

		err = dec_pkts(larg, rx_ppio_id, bpool_id, sam_enc_res_descs, num_enc);
		if (unlikely(err))
			return err;
	}

	if (num_dec)
		return send_pkts(larg, rx_ppio_id, tx_ppio_id, bpool_id, tc, sam_dec_res_descs, num_dec);

	return 0;
}

static inline int loop_sw_recycle(struct local_arg	*larg,
				  u8			 rx_ppio_id,
				  u8			 tx_ppio_id,
				  u8			 bpool_id,
				  u8			 tc,
				  u8			 qid,
				  u16			 num)
{
	struct pp2_ppio_desc	 descs[MAX_BURST_SIZE];
	int			 err;

/*pr_info("tid %d check on tc %d, qid %d\n", larg->id, tc, qid);*/
START_COUNT_CYCLES(pme_ev_cnt_rx);
	err = pp2_ppio_recv(larg->ports_desc[rx_ppio_id].port, tc, qid, descs, &num);
STOP_COUNT_CYCLES(pme_ev_cnt_rx, num);
#ifdef VERBOSE_DEBUG
	if (larg->garg->verbose && num)
		printf("got %d pkts on ppio %d, tc %d, qid %d\n", num, rx_ppio_id, tc, qid);
#endif /* VERBOSE_DEBUG */

	if (num) {
		err = enc_pkts(larg, rx_ppio_id, bpool_id, descs, num);
		if (unlikely(err))
			return err;
	}

	err = deq_n_proc_pkts(larg, larg->enc_cio, rx_ppio_id, tx_ppio_id, bpool_id, tc);
	if (unlikely(err))
		return err;
	if (larg->dec_cio != larg->enc_cio)
		return deq_n_proc_pkts(larg, larg->dec_cio, rx_ppio_id, tx_ppio_id, bpool_id, tc);
	return 0;
}

static int find_port_info(struct port_desc *port_desc)
{
	char		 name[20];
	u8		 pp, ppio;
	int		 err;

	if (!port_desc->name) {
		pr_err("No port name given!\n");
		return -1;
	}

	memset(name, 0, sizeof(name));
	snprintf(name, sizeof(name), "%s", port_desc->name);
	err = pp2_netdev_get_port_info(name, &pp, &ppio);
	if (err) {
		pr_err("PP2 Port %s not found!\n", port_desc->name);
		return err;
	}

	port_desc->ppio_id = ppio;
	port_desc->pp_id = pp;

	return 0;
}

static int find_free_bpool(void)
{
	int	i;

	for (i = 0; i < PP2_TOTAL_NUM_BPOOLS; i++) {
		if (!((uint64_t)(1<<i) & used_bpools)) {
			used_bpools |= (uint64_t)(1<<i);
			break;
		}
	}
	if (i == PP2_TOTAL_NUM_BPOOLS) {
		pr_err("no free BPool found!\n");
		return -ENOSPC;
	}
	return i;
}

static int find_free_hif(void)
{
	int	i;

	for (i = 0; i < PP2_TOTAL_NUM_HIFS; i++) {
		if (!((uint64_t)(1<<i) & used_hifs)) {
			used_hifs |= (uint64_t)(1<<i);
			break;
		}
	}
	if (i == PP2_TOTAL_NUM_HIFS) {
		pr_err("no free HIF found!\n");
		return -ENOSPC;
	}
	return i;
}

static int find_free_cio(u8 eng)
{
	int	i;

	for (i = 0; i < SAM_TOTAL_NUM_CIOS; i++) {
		if (!((uint64_t)(1<<i) & used_cios[eng])) {
			used_cios[eng] |= (uint64_t)(1<<i);
			break;
		}
	}
	if (i == SAM_TOTAL_NUM_CIOS) {
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
			if (qid == PP2_MAX_NUM_QS_PER_TC) {
				qid = 0;
				tc++;
				if (tc == PP2_MAX_NUM_TCS_PER_PORT)
					tc = 0;
			}
		} while (!(larg->qs_map & (1<<((tc*PP2_MAX_NUM_QS_PER_TC)+qid))));

		err = loop_sw_recycle(larg, 0, 0, 0, tc, qid, num);
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
			if (qid == PP2_MAX_NUM_QS_PER_TC) {
				qid = 0;
				tc++;
				if (tc == PP2_MAX_NUM_TCS_PER_PORT)
					tc = 0;
			}
		} while (!(larg->qs_map & (1<<((tc*PP2_MAX_NUM_QS_PER_TC)+qid))));

		err  = loop_sw_recycle(larg, 0, 1, 0, tc, qid, num);
		err |= loop_sw_recycle(larg, 1, 0, 0, tc, qid, num);
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
	tmp_time_inter += (curr_time.tv_usec - garg->ctrl_trd_last_time.tv_usec)/1000;

	drop_cnt = 0;
	for (i = 0; i < MAX_NUM_CORES; i++)
		if (garg->largs[i])
			drop_cnt += garg->largs[i]->drop_cnt;
	tmp_rx_cnt = tmp_tx_cnt = 0;
	for (i = 0; i < MAX_NUM_CORES; i++)
		if (garg->largs[i]) {
			tmp_rx_cnt += garg->largs[i]->rx_cnt;
			tmp_tx_cnt += garg->largs[i]->tx_cnt;
		}
	printf("Perf: %dKpps (Rx: %dKpps)",
		(int)((tmp_tx_cnt - garg->lst_tx_cnt)/tmp_time_inter),
		(int)((tmp_rx_cnt - garg->lst_rx_cnt)/tmp_time_inter));
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
	tmp_time_inter += (curr_time.tv_usec - garg->ctrl_trd_last_time.tv_usec)/1000;
	if (tmp_time_inter >= garg->ctrl_thresh)
		return dump_perf(garg);
	return 0;
}

static int init_all_modules(void)
{
	struct pp2_init_params	 pp2_params;
	int			 err;

	pr_info("Global initializations ... ");

	err = mv_sys_dma_mem_init(DMA_MEM_SIZE);
	if (err)
		return err;

	memset(&pp2_params, 0, sizeof(pp2_params));
	pp2_params.hif_reserved_map = PP2_HIFS_RSRV;
	pp2_params.bm_pool_reserved_map = PP2_BPOOLS_RSRV;
	/* Enable 10G port */
	pp2_params.ppios[0][0].is_enabled = 1;
	pp2_params.ppios[0][0].first_inq = 0;
	/* Enable 1G ports */
	pp2_params.ppios[0][1].is_enabled = 1;
	pp2_params.ppios[0][1].first_inq = 0;
	pp2_params.ppios[0][2].is_enabled = 1;
	pp2_params.ppios[0][2].first_inq = 0;
	if (garg.pp2_num_inst == 2) {
		/* Enable 10G port */
		pp2_params.ppios[1][0].is_enabled = 1;
		pp2_params.ppios[1][0].first_inq = 0;
		/* Enable 1G ports */
		pp2_params.ppios[1][1].is_enabled = 1;
		pp2_params.ppios[1][1].first_inq = 0;
		pp2_params.ppios[1][2].is_enabled = 1;
		pp2_params.ppios[1][2].first_inq = 0;
	}
	err = pp2_init(&pp2_params);
	if (err)
		return err;

	pr_info("done\n");
	return 0;
}

static int create_sam_sessions(struct sam_cio		*cio,
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
	err = sam_session_create(cio, &sa_params, enc_sa);
	if (err) {
		pr_err("EnC SA creation failed (%d)!\n", err);
		return err;
	}
	if (!*enc_sa) {
		pr_err("EnC SA creation failed!\n");
		return -EFAULT;
	}

	sa_params.dir = SAM_DIR_DECRYPT;   /* operation direction: decode */
	err = sam_session_create(cio, &sa_params, dec_sa);
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

static int build_all_bpools(struct glob_arg *garg)
{
	struct pp2_bpool_params		bpool_params;
	int				i, j, k, err, pool_id;
	struct bpool_inf		std_infs[] = BPOOLS_INF;
	struct bpool_inf		jumbo_infs[] = BPOOLS_JUMBO_INF;
	struct bpool_inf		*infs;
	char				name[15];
	int				pp2_num_inst = garg->pp2_num_inst;

	if (garg->mtu > DEFAULT_MTU) {
		infs = jumbo_infs;
		garg->num_pools = ARRAY_SIZE(jumbo_infs);
	} else {
		infs = std_infs;
		garg->num_pools = ARRAY_SIZE(std_infs);
	}

	garg->pools = (struct pp2_bpool ***)malloc(pp2_num_inst*sizeof(struct pp2_bpool **));
	if (!garg->pools) {
		pr_err("no mem for bpools array!\n");
		return -ENOMEM;
	}
	garg->buffs_inf =
		(struct pp2_buff_inf ***)malloc(pp2_num_inst*sizeof(struct pp2_buff_inf **));
	if (!garg->buffs_inf) {
		pr_err("no mem for bpools-inf array!\n");
		return -ENOMEM;
	}
	garg->num_pools = ARRAY_SIZE(infs);
	/* TODO: temporary W/A until we have map routines of bpools to ppios */
	if (garg->num_pools > PP2_PPIO_TC_MAX_POOLS) {
		pr_err("only %d pools allowed!\n", PP2_PPIO_TC_MAX_POOLS);
		return -EINVAL;
	}

	for (i = 0; i < pp2_num_inst; i++) {
		garg->pools[i] = (struct pp2_bpool **)malloc(garg->num_pools*sizeof(struct pp2_bpool *));
		if (!garg->pools[i]) {
			pr_err("no mem for bpools array!\n");
			return -ENOMEM;
		}
		garg->buffs_inf[i] =
			(struct pp2_buff_inf **)malloc(garg->num_pools*sizeof(struct pp2_buff_inf *));
		if (!garg->buffs_inf[i]) {
			pr_err("no mem for bpools-inf array!\n");
			return -ENOMEM;
		}

		for (j = 0; j < garg->num_pools; j++) {
			pool_id = find_free_bpool();
			if (pool_id < 0) {
				pr_err("free bpool not found!\n");
				return pool_id;
			}
			memset(name, 0, sizeof(name));
			snprintf(name, sizeof(name), "pool-%d:%d", i, pool_id);
			pr_debug("found bpool:  %s\n", name);
			memset(&bpool_params, 0, sizeof(bpool_params));
			bpool_params.match = name;
			bpool_params.buff_len = infs[j].buff_size;
			err = pp2_bpool_init(&bpool_params, &garg->pools[i][j]);
			if (err)
				return err;
			if (!garg->pools[i][j]) {
				pr_err("BPool init failed!\n");
				return -EIO;
			}

			garg->buffs_inf[i][j] =
				(struct pp2_buff_inf *)malloc(infs[j].num_buffs*sizeof(struct pp2_buff_inf));
			if (!garg->buffs_inf[i][j]) {
				pr_err("no mem for bpools-inf array!\n");
				return -ENOMEM;
			}

			for (k = 0; k < infs[j].num_buffs; k++) {
				void *buff_virt_addr;

				buff_virt_addr = mv_sys_dma_mem_alloc(infs[j].buff_size, 4);
				if (!buff_virt_addr) {
					pr_err("failed to allocate mem (%d)!\n", k);
					return -1;
				}
				if (k == 0) {
					sys_dma_high_addr = ((u64)buff_virt_addr) & (~((1ULL<<32) - 1));
					pr_debug("sys_dma_high_addr (0x%lx)\n", sys_dma_high_addr);
				}
				if ((upper_32_bits((u64)buff_virt_addr)) != (sys_dma_high_addr >> 32)) {
					pr_err("buff_virt_addr(%p)  upper out of range; skipping this buff\n",
						buff_virt_addr);
					continue;
				}
				garg->buffs_inf[i][j][k].addr =
					(bpool_dma_addr_t)mv_sys_dma_mem_virt2phys(buff_virt_addr);
				/* cookie contains lower_32_bits of the va */
				garg->buffs_inf[i][j][k].cookie = lower_32_bits((u64)buff_virt_addr);
			}
			for (k = 0; k < infs[j].num_buffs; k++) {
				struct pp2_buff_inf	tmp_buff_inf;
				/* Don't add first buffer into BPool */
				if (k == 0)
					continue;
				tmp_buff_inf.cookie = garg->buffs_inf[i][j][k].cookie;
				tmp_buff_inf.addr   = garg->buffs_inf[i][j][k].addr;
				err = pp2_bpool_put_buff(garg->hif, garg->pools[i][j], &tmp_buff_inf);
				if (err)
					return err;
			}
		}
	}

	return 0;
}

static int init_local_modules(struct glob_arg *garg)
{
	struct pp2_hif_params		hif_params;
	struct pp2_ppio_params		port_params;
	struct pp2_ppio_inq_params	inq_params;
	struct sam_cio_params		cio_params;
	char				name[15];
	int				i, j, err, port_index, hif_id, cio_id;
	u16				mtu;

	pr_info("Specific modules initializations\n");

	hif_id = find_free_hif();
	if (hif_id < 0) {
		pr_err("free HIF not found!\n");
		return hif_id;
	}
	memset(name, 0, sizeof(name));
	snprintf(name, sizeof(name), "hif-%d", hif_id);
	pr_debug("found hif: %s\n", name);
	memset(&hif_params, 0, sizeof(hif_params));
	hif_params.match = name;
	hif_params.out_size = Q_SIZE;
	err = pp2_hif_init(&hif_params, &garg->hif);
	if (err)
		return err;
	if (!garg->hif) {
		pr_err("HIF init failed!\n");
		return -EIO;
	}

	err = build_all_bpools(garg);
	if (err)
		return err;

	for (port_index = 0; port_index < garg->num_ports; port_index++) {
		err = find_port_info(&garg->ports_desc[port_index]);
		if (err) {
			pr_err("Port info not found!\n");
			return err;
		}

		memset(name, 0, sizeof(name));
		snprintf(name, sizeof(name), "ppio-%d:%d",
			 garg->ports_desc[port_index].pp_id, garg->ports_desc[port_index].ppio_id);
		pr_debug("found port: %s\n", name);
		memset(&port_params, 0, sizeof(port_params));
		port_params.match = name;
		port_params.type = PP2_PPIO_T_NIC;
		port_params.inqs_params.num_tcs = PP2_MAX_NUM_TCS_PER_PORT;
		for (i = 0; i < port_params.inqs_params.num_tcs; i++) {
			port_params.inqs_params.tcs_params[i].pkt_offset = PKT_OFFS>>2;
			port_params.inqs_params.tcs_params[i].num_in_qs = PP2_MAX_NUM_QS_PER_TC;
			/* TODO: we assume here only one Q per TC; change it! */
			inq_params.size = Q_SIZE;
			port_params.inqs_params.tcs_params[i].inqs_params = &inq_params;
			for (j = 0; j < garg->num_pools; j++)
				port_params.inqs_params.tcs_params[i].pools[j] =
					garg->pools[garg->ports_desc[port_index].pp_id][j];
		}
		port_params.outqs_params.num_outqs = PP2_MAX_NUM_TCS_PER_PORT;
		for (i = 0; i < port_params.outqs_params.num_outqs; i++) {
			port_params.outqs_params.outqs_params[i].size = Q_SIZE;
			port_params.outqs_params.outqs_params[i].weight = 1;
		}
		err = pp2_ppio_init(&port_params, &garg->ports_desc[port_index].port);
		if (err)
			return err;
		if (!garg->ports_desc[port_index].port) {
			pr_err("PP-IO init failed!\n");
			return -EIO;
		}

		pp2_ppio_get_mtu(garg->ports_desc[port_index].port, &mtu);
		if (mtu != garg->mtu) {
			pp2_ppio_set_mtu(garg->ports_desc[port_index].port, garg->mtu);
			pp2_ppio_set_mru(garg->ports_desc[port_index].port, MVPP2_MTU_TO_MRU(garg->mtu));
			pr_debug("Set port ppio-%d:%d MTU to %d\n",
				 garg->ports_desc[port_index].pp_id,
				 garg->ports_desc[port_index].ppio_id,
				 garg->mtu);
		}

		err = pp2_ppio_enable(garg->ports_desc[port_index].port);
		if (err)
			return err;
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
	cio_params.size = Q_SIZE;
	cio_params.num_sessions = SAM_MAX_NUM_SESSIONS_PER_RING;
	cio_params.max_buf_size = garg->mtu+64;
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
	int	i, j;
	int	pp2_num_inst = garg->pp2_num_inst;

	if (garg->ports_desc[0].port) {
		pp2_ppio_disable(garg->ports_desc[0].port);
		pp2_ppio_deinit(garg->ports_desc[0].port);
	}

	if (garg->pools) {
		for (i = 0; i < pp2_num_inst; i++) {
			if (garg->pools[i]) {
				for (j = 0; j < garg->num_pools; j++)
					if (garg->pools[i][j])
						pp2_bpool_deinit(garg->pools[i][j]);
				free(garg->pools[i]);
			}
		}
		free(garg->pools);
	}
	if (garg->buffs_inf) {
		for (i = 0; i < pp2_num_inst; i++) {
			if (garg->buffs_inf[i]) {
				for (j = 0; j < garg->num_pools; j++)
					if (garg->buffs_inf[i][j])
						free(garg->buffs_inf[i][j]);
				free(garg->buffs_inf[i]);
			}
		}
		free(garg->buffs_inf);
	}

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
		; /* TODO: unregister cli cmds */
	destroy_local_modules(garg);
	destroy_all_modules();
}

static int init_local(void *arg, int id, void **_larg)
{
	struct glob_arg		*garg = (struct glob_arg *)arg;
	struct local_arg	*larg;
	struct pp2_hif_params	 hif_params;
	struct sam_cio_params	 cio_params;
	char			 name[15];
	int			 i, err, hif_id, cio_id;

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
	hif_id = find_free_hif();
	if (hif_id < 0) {
		pr_err("free HIF not found!\n");
		pthread_mutex_unlock(&garg->trd_lock);
		return hif_id;
	}
	memset(name, 0, sizeof(name));
	snprintf(name, sizeof(name), "hif-%d", hif_id);
	pr_debug("found hif: %s\n", name);
	memset(&hif_params, 0, sizeof(hif_params));
	hif_params.match = name;
	hif_params.out_size = Q_SIZE;
	err = pp2_hif_init(&hif_params, &larg->hif);
	pthread_mutex_unlock(&garg->trd_lock);
	if (err != 0)
		return err;
	if (!larg->hif) {
		pr_err("HIF init failed!\n");
		return -EIO;
	}

	if (id == 0)
		larg->enc_cio = garg->cio;
	else {
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
		cio_params.size = Q_SIZE;
		cio_params.num_sessions = SAM_MAX_NUM_SESSIONS_PER_RING;
		cio_params.max_buf_size = garg->mtu+64;
		err = sam_cio_init(&cio_params, &larg->enc_cio);
		pthread_mutex_unlock(&garg->trd_lock);
		if (err != 0)
			return err;
	}
	if (!larg->enc_cio) {
		pr_err("CIO init failed!\n");
		return -EIO;
	}
#ifdef SAM_TWO_ENG_SUPPORT
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
		cio_params.size = Q_SIZE;
		cio_params.num_sessions = SAM_MAX_NUM_SESSIONS_PER_RING;
		cio_params.max_buf_size = garg->mtu+64;
		err = sam_cio_init(&cio_params, &larg->dec_cio);
		pthread_mutex_unlock(&garg->trd_lock);
		if (err != 0)
			return err;
	} else
#endif /* SAM_TWO_ENG_SUPPORT */
		larg->dec_cio = larg->enc_cio;

	larg->id                = id;
	larg->burst		= garg->burst;
	larg->echo              = garg->echo;
	larg->num_ports         = garg->num_ports;
	larg->ports_desc = (struct lcl_port_desc *)malloc(larg->num_ports*sizeof(struct lcl_port_desc));
	if (!larg->ports_desc) {
		pr_err("no mem for local-port-desc obj!\n");
		return -ENOMEM;
	}
	memset(larg->ports_desc, 0, larg->num_ports*sizeof(struct lcl_port_desc));
	for (i = 0; i < larg->num_ports; i++) {
		larg->ports_desc[i].pp_id = garg->ports_desc[i].pp_id;
		larg->ports_desc[i].ppio_id = garg->ports_desc[i].ppio_id;
		larg->ports_desc[i].port = garg->ports_desc[i].port;
	}
	larg->pools             = garg->pools;

	err = create_sam_sessions(larg->enc_cio, &larg->enc_sa, &larg->dec_sa);
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

	if (!larg)
		return;

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
#ifdef VERBOSE_DEBUG
	       "\t-v                       Enable verbose debug\n"
#endif /* VERBOSE_DEBUG */
	       "\t--no-echo                No Echo packets\n"
	       "\t--cli                    Use CLI\n"
	       "\t?, -h, --help            Display help and exit.\n\n"
	       "\n", NO_PATH(progname), NO_PATH(progname), PP2_MAX_NUM_PORTS, MAX_BURST_SIZE, DEFAULT_MTU
	       );
}

static int parse_args(struct glob_arg *garg, int argc, char *argv[])
{
	int	i = 1;

#ifdef VERBOSE_DEBUG
	garg->verbose = 0;
#endif /* VERBOSE_DEBUG */
	garg->cli = 0;
	garg->cpus = 1;
	garg->affinity = -1;
	garg->burst = DFLT_BURST_SIZE;
	garg->mtu = DEFAULT_MTU;
	garg->echo = 1;
	garg->qs_map = 0;
	garg->qs_map_shift = 0;
	garg->prefetch_shift = PREFETCH_SHIFT;
	garg->pp2_num_inst = pp2_get_num_inst();
	garg->ctrl_thresh = CTRL_DFLT_THR;

	while (i < argc) {
		if ((strcmp(argv[i], "?") == 0) ||
		    (strcmp(argv[i], "-h") == 0) ||
		    (strcmp(argv[i], "--help") == 0)) {
			usage(argv[0]);
			exit(0);
		} else if (strcmp(argv[i], "-i") == 0) {
			char *token;

			if (argc < (i+2)) {
				pr_err("Invalid number of arguments!\n");
				return -EINVAL;
			}
			if (argv[i+1][0] == '-') {
				pr_err("Invalid interface arguments format!\n");
				return -EINVAL;
			}

			/* count the number of tokens separated by ',' */
			for (token = strtok(argv[i+1], ","), garg->num_ports = 0;
			     token != NULL;
			     token = strtok(NULL, ","), garg->num_ports++)
				snprintf(garg->ports_desc[garg->num_ports].name,
					 sizeof(garg->ports_desc[garg->num_ports].name),
					 "%s", token);

			if (garg->num_ports == 0) {
				pr_err("Invalid interface arguments format!\n");
				return -EINVAL;
			} else if (garg->num_ports > PP2_MAX_NUM_PORTS) {
				pr_err("too many ports specified (%d vs %d)\n",
				       garg->num_ports, PP2_MAX_NUM_PORTS);
				return -EINVAL;
			}
			i += 2;
		} else if (strcmp(argv[i], "-b") == 0) {
			if (argc < (i+2)) {
				pr_err("Invalid number of arguments!\n");
				return -EINVAL;
			}
			if (argv[i+1][0] == '-') {
				pr_err("Invalid arguments format!\n");
				return -EINVAL;
			}
			garg->burst = atoi(argv[i+1]);
			i += 2;
		} else if (strcmp(argv[i], "-t") == 0) {
			if (argc < (i+2)) {
				pr_err("Invalid number of arguments!\n");
				return -EINVAL;
			}
			if (argv[i+1][0] == '-') {
				pr_err("Invalid arguments format!\n");
				return -EINVAL;
			}
			garg->mtu = atoi(argv[i+1]);
			i += 2;
		} else if (strcmp(argv[i], "-c") == 0) {
			if (argc < (i+2)) {
				pr_err("Invalid number of arguments!\n");
				return -EINVAL;
			}
			if (argv[i+1][0] == '-') {
				pr_err("Invalid arguments format!\n");
				return -EINVAL;
			}
			garg->cpus = atoi(argv[i+1]);
			i += 2;
		} else if (strcmp(argv[i], "-a") == 0) {
			garg->affinity = atoi(argv[i+1]);
			i += 2;
		} else if (strcmp(argv[i], "-m") == 0) {
			char *token;

			if (argc < (i+2)) {
				pr_err("Invalid number of arguments!\n");
				return -EINVAL;
			}
			if (argv[i+1][0] == '-') {
				pr_err("Invalid arguments format!\n");
				return -EINVAL;
			}
			token = strtok(argv[i+1], ":");
			garg->qs_map = strtoul(token, NULL, 16);
			token = strtok(NULL, "");
			garg->qs_map_shift = atoi(token);
			i += 2;
#ifdef VERBOSE_DEBUG
		} else if (strcmp(argv[i], "-v") == 0) {
			garg->verbose = 1;
			i += 1;
#endif /* VERBOSE_DEBUG */
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
	if (garg->burst > MAX_BURST_SIZE) {
		pr_err("illegal burst size requested (%d vs %d)!\n",
			garg->burst, MAX_BURST_SIZE);
		return -EINVAL;
	}
	if (garg->cpus > MAX_NUM_CORES) {
		pr_err("illegal num cores requested (%d vs %d)!\n",
			garg->cpus, MAX_NUM_CORES);
		return -EINVAL;
	}
	if ((garg->affinity != -1) &&
	    ((garg->cpus + garg->affinity) > MAX_NUM_CORES)) {
		pr_err("illegal num cores or affinity requested (%d,%d vs %d)!\n",
			garg->cpus, garg->affinity, MAX_NUM_CORES);
		return -EINVAL;
	}

	if (garg->qs_map &&
	    (PP2_MAX_NUM_QS_PER_TC == 1) &&
	    (PP2_MAX_NUM_TCS_PER_PORT == 1)) {
		pr_warn("no point in queues-mapping; ignoring.\n");
		garg->qs_map = 1;
		garg->qs_map_shift = 1;
	} else if (!garg->qs_map) {
		garg->qs_map = 1;
		garg->qs_map_shift = PP2_MAX_NUM_TCS_PER_PORT;
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
