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
#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <sched.h>
#include <fcntl.h>
#include <arpa/inet.h>	/* ntohs */
#include <netinet/ip.h>
#include <netinet/udp.h>
#include <getopt.h>
#include <sys/time.h>

#include "mv_std.h"
#include <stdbool.h>
#include "lib/net.h"
#include "lib/lib_misc.h"
#include "env/mv_sys_dma.h"
#include "env/io.h"
#include "mv_dmax2.h"

#include "mvapp.h"
#include "utils.h"


#define PKT_GEN_APP_VERBOSE_DEBUG

#define PKT_GEN_APP_DEF_Q_SIZE		1024
#define PKT_GEN_APP_DMAX2_Q_SIZE	(2 * PKT_GEN_APP_DEF_Q_SIZE)
#define PKT_GEN_APP_DSHADOW_Q_SIZE	(PKT_GEN_APP_DMAX2_Q_SIZE)

#define PKT_GEN_APP_MAX_BURST_SIZE	((PKT_GEN_APP_DMAX2_Q_SIZE) >> 1)
/* as DMAX2 is the bottleneck, set the burst size to DMAX2_Q_SIZE / 4 */
#define PKT_GEN_APP_DFLT_BURST_SIZE	(PKT_GEN_APP_DMAX2_Q_SIZE >> 2)

#define PKT_GEN_APP_BUFF_POOL_SIZE	8192
#define PKT_GEN_APP_MAX_NUM_CPUS	3

#define PKT_GEN_APP_DMA_MEM_SIZE	(80 * 1024 * 1024)
#define PKT_GEN_APP_STATS_DFLT_THR	1000

#define PKT_GEN_APP_NUM_ENGS		2
#define PKT_GEN_APP_ENG_GEN		0
#define PKT_GEN_APP_ENG_ANL		1

#define PKT_GEN_APP_MAX_TOTAL_FRM_CNT	UINT32_MAX

#define PKT_GEN_APP_PREFETCH_SHIFT	4

#define PKT_GEN_NUM_CNTS		2
#define PKT_GEN_CNT_PKTS		0
#define PKT_GEN_CNT_BYTES		1

#define next_q_idx(_idx, _size) (((_idx+1) == _size) ? 0 : (_idx+1))
#define q_occupancy(prod, cons, q_size)	\
	(((prod) - (cons) + (q_size)) & ((q_size) - 1))
#define q_space(prod, cons, q_size)	\
	((q_size) - q_occupancy((prod), (cons), (q_size)) - 1)

#define MAX_BODYSIZE	(DEFAULT_MTU - IPV4_HDR_LEN - UDP_HDR_LEN)
#define MIN_PKT_SIZE			(60)
#define MAX_PKT_SIZE			(DEFAULT_MTU + ETH_HLEN)

#define MAX_BUFF_SIZE			2048

#define DEFAULT_REPORT_TIME 1 /* 1 second */
#define DEFAULT_RATE_USECS  0
#define DEFAULT_SRC_IP 0x0a000001
#define DEFAULT_DST_IP 0x0a000002
#define DEFAULT_SRC_PORT 1024
#define DEFAULT_DST_PORT 1024


struct pkt {
	struct ether_header	 eh;
	struct ip		 ip;
	struct udphdr		 udp;
	/* using 'empty array' as placeholder; the real size will be determine by the implementation
	 */
	u8			 body[];
} __packed;

struct dmax2_shadow_ent {
	struct buffer_desc	*src;
	struct buffer_desc	 dst;
	u16			 pkt_len;
	u16			 res;
};

struct dmax2_shadow_q {
	u16			 read_ind;	/* read index */
	u16			 write_ind;	/* write index */
	u16			 inter_ind;	/* intermmediate index */
	struct dmax2_shadow_ent	 ents[PKT_GEN_APP_DSHADOW_Q_SIZE];
	void			*buffer;
};

struct dmax2_eng_desc {
	u16			 id;
	char			 name[15];	/* Port name */
	struct dmax2		*dmax2;
	struct mv_stack		*stack_hndl;
	struct dmax2_shadow_q	 shadow_q;
};

struct local_arg;

struct glob_arg {
	struct glb_common_args	 cmn_args; /* Keep first */

	pthread_mutex_t		 trd_lock;

	int			 rx;
	int			 tx;
	u32			 total_frm_cnt;

	struct ip_range		 src_ip;
	struct ip_range		 dst_ip;
	eth_addr_t		 dst_mac;
	eth_addr_t		 src_mac;
	int			 pkt_size;
	u32			 report_time;

	int			 num_bufs;

	u8			 base_engine;

	void			*buffer;

	struct buffer_desc	 buf_dec[PKT_GEN_APP_BUFF_POOL_SIZE];
	struct dmax2_eng_desc	 eng_descs[PKT_GEN_APP_NUM_ENGS];
};

struct traffic_counters {
	u64		tx_pkts;
	u64		tx_bytes;
	u64		tx_drop;
	u64		rx_pkts;
	u64		rx_bytes;
};

struct local_arg {
	struct local_common_args	 cmn_args; /* Keep first */

	u16				 curr_frm;

	struct traffic_counters		 trf_cntrs;

	struct buffer_desc		*buf_dec;
	struct dmax2_eng_desc		*eng_descs;
};


eth_addr_t default_src_mac = {0x00, 0x00, 0x00, 0x00, 0x00, 0x01};
eth_addr_t default_dst_mac = {0x00, 0x00, 0x00, 0x00, 0x00, 0x02};

/* globals for ingress/egress packet rate statistics */
struct timeval		 ctrl_trd_last_time;
u64			 lst_rx_cnts[PKT_GEN_NUM_CNTS];
u64			 lst_tx_cnts[PKT_GEN_NUM_CNTS];

static struct glob_arg garg = {};

#ifdef PKT_GEN_APP_VERBOSE_DEBUG
static u8 dummy_arr[48*1024] = {};
#endif /* PKT_GEN_APP_VERBOSE_DEBUG */


static int build_dmax2_shadow(struct dmax2_shadow_q *shadow_q, u16 num_buffs, u16 buff_len)
{
	void	*buff_phys_addr;
	int	 i;

	shadow_q->buffer = mv_sys_dma_mem_alloc(buff_len * num_buffs, 4);
	if (!shadow_q->buffer) {
		pr_err("failed to allocate dmax2 bpool mem!\n");
		return -ENOMEM;
	}

	buff_phys_addr = (void *)mv_sys_dma_mem_virt2phys(shadow_q->buffer);

	for (i = 0; i < num_buffs; i++) {
		shadow_q->ents[i].dst.phy_addr = (u64)buff_phys_addr + (i * buff_len);
		shadow_q->ents[i].dst.virt_addr = shadow_q->buffer + (i * buff_len);
	}

	return 0;
}

static int dump_perf(struct glob_arg *garg)
{
	struct timeval	 curr_time;
	u64		 tmp_time_inter;
	u64 pkts, bytes, drops, pps, bps;
	u8 i;

	gettimeofday(&curr_time, NULL);
	tmp_time_inter = (curr_time.tv_sec - ctrl_trd_last_time.tv_sec) * 1000;
	tmp_time_inter += (curr_time.tv_usec - ctrl_trd_last_time.tv_usec) / 1000;

	printf("\r");
	pkts = bytes = 0;
	for (i = 0; i < garg->cmn_args.cpus; i++) {
		pkts  += (garg->cmn_args.largs[i])->trf_cntrs.rx_pkts;
		bytes += (garg->cmn_args.largs[i])->trf_cntrs.rx_bytes;
	}
	pps = pkts - lst_rx_cnts[PKT_GEN_CNT_PKTS];
	bps = bytes - lst_rx_cnts[PKT_GEN_CNT_BYTES];
	lst_rx_cnts[PKT_GEN_CNT_PKTS] = pkts;
	lst_rx_cnts[PKT_GEN_CNT_BYTES] = bytes;
	pps /= tmp_time_inter;
	bps = bps * 8 / tmp_time_inter;

	printf("RX: %" PRIu64 " Kpps, %" PRIu64 " Kbps\t", pps, bps);

	drops = pkts = bytes = 0;
	for (i = 0; i < garg->cmn_args.cpus; i++) {
		pkts  += (garg->cmn_args.largs[i])->trf_cntrs.tx_pkts;
		bytes += (garg->cmn_args.largs[i])->trf_cntrs.tx_bytes;
		drops += (garg->cmn_args.largs[i])->trf_cntrs.tx_drop;
	}
	pps = pkts - lst_tx_cnts[PKT_GEN_CNT_PKTS];
	bps = bytes - lst_tx_cnts[PKT_GEN_CNT_BYTES];
	lst_tx_cnts[PKT_GEN_CNT_PKTS] = pkts;
	lst_tx_cnts[PKT_GEN_CNT_BYTES] = bytes;
	pps /= tmp_time_inter;
	bps = bps * 8 / tmp_time_inter;

	printf("TX: %" PRIu64 " Kpps, %" PRIu64 " Kbps, %" PRIu64 " Kdrops\t",
		pps, bps, drops / 1000);

	gettimeofday(&ctrl_trd_last_time, NULL);

	return 0;
}

static int maintain_stats(void *arg)
{
	struct glob_arg *garg = (struct glob_arg *)arg;
	struct timeval	 curr_time;
	u64		 tmp_time_inter;

	if (!garg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	gettimeofday(&curr_time, NULL);
	tmp_time_inter = (curr_time.tv_sec - ctrl_trd_last_time.tv_sec) * 1000;
	tmp_time_inter += (curr_time.tv_usec - ctrl_trd_last_time.tv_usec) / 1000;
	if (tmp_time_inter >= garg->cmn_args.ctrl_thresh)
		return dump_perf(garg);

	return 0;
}

static inline int loop_echo(struct local_arg *larg)
{
	struct dmax2_trans_complete_desc dmax2_res_descs[PKT_GEN_APP_MAX_BURST_SIZE];
	struct dmax2_desc		 dmax2_descs[PKT_GEN_APP_MAX_BURST_SIZE];
	struct dmax2_eng_desc		*eng_desc_gen = &(larg->eng_descs[PKT_GEN_APP_ENG_GEN]);
	struct dmax2_eng_desc		*eng_desc_anl = &(larg->eng_descs[PKT_GEN_APP_ENG_ANL]);
	struct dmax2_shadow_q		*shadow_q_gen, *shadow_q_anl;
#ifdef PKT_GEN_APP_VERBOSE_DEBUG
	struct pkt			*pkt;
	u32				*buff;
#endif /* PKT_GEN_APP_VERBOSE_DEBUG */
	u16				 write_ind, inter_ind;
	u16				 i, num, num_got, free_count;
	int				 err;
	u16				 prefetch_idx;
	int				 prefetch_shift = larg->cmn_args.prefetch_shift;

	shadow_q_gen = &eng_desc_gen->shadow_q;
	shadow_q_anl = &eng_desc_anl->shadow_q;

	num = PKT_GEN_APP_MAX_BURST_SIZE;
	write_ind = shadow_q_anl->write_ind;
	free_count = q_space(write_ind, mv_readw_relaxed(&shadow_q_anl->read_ind), PKT_GEN_APP_DSHADOW_Q_SIZE);
	if (num > free_count)
		num = free_count;
	if (!num)
		return 0;

	err = dmax2_deq(eng_desc_gen->dmax2, dmax2_res_descs, &num, 1);
	if (unlikely(err)) {
		pr_err("DMAX2 DeQ (EnC) failed (%d)!\n", err);
		return -EFAULT;
	}
	if (!num)
		return 0;
#ifdef PKT_GEN_APP_VERBOSE_DEBUG
	if (unlikely(larg->cmn_args.verbose && num))
		printf("recv %d pkts on generator\n", num);
#endif /* PKT_GEN_APP_VERBOSE_DEBUG */

	inter_ind = shadow_q_gen->inter_ind;
	prefetch_idx = inter_ind + prefetch_shift;
	if (prefetch_idx >= PKT_GEN_APP_DSHADOW_Q_SIZE)
		prefetch_idx -= PKT_GEN_APP_DSHADOW_Q_SIZE;

	for (i = 0; i < num; i++) {
		if (unlikely(!shadow_q_gen->ents[inter_ind].src ||
			!shadow_q_gen->ents[inter_ind].src->virt_addr)) {
			pr_warn("Shadow memory @%d: cookie(%p), pa(%lx)!\n",
				inter_ind,
				shadow_q_gen->ents[inter_ind].src,
				shadow_q_gen->ents[inter_ind].src ?
					(u64)shadow_q_gen->ents[inter_ind].src->virt_addr : 0);
			continue;
		}

#ifdef PKT_GEN_APP_VERBOSE_DEBUG
		if (unlikely(dmax2_res_descs[i].status))
			pr_err("Generator desc - Illegal status (%x)!\n", dmax2_res_descs[i].status);
		if (unlikely(dmax2_res_descs[i].desc_id != inter_ind))
			pr_err("Generator desc - Illegal id (%d vs %d)!\n", dmax2_res_descs[i].desc_id, inter_ind);

		pkt = (struct pkt *)shadow_q_gen->ents[inter_ind].dst.virt_addr;
		buff = (u32 *)pkt->body;
		if (unlikely((buff[0] == MVAPPS_PLD_WATERMARK) || (buff[0] != inter_ind)))
			pr_info("\r[ERROR] Generator: Illegal PLD: %x\n", buff[0]);
		else if (unlikely((pkt->ip.ip_v == IPVERSION) && (pkt->eh.ether_type == htons(0x800)))) {
			u16 tot_len = swab16(pkt->ip.ip_len) + sizeof(pkt->eh); /* assuming eth header */

			if (unlikely(tot_len != shadow_q_gen->ents[inter_ind].pkt_len))
				pr_info("\r[ERROR] Generator: length mismatch (frame %u vs rx-desc %u @%d)!\n",
					tot_len, shadow_q_gen->ents[inter_ind].pkt_len, inter_ind);
		}
#endif /* PKT_GEN_APP_VERBOSE_DEBUG */

		if (num - i > prefetch_shift) {
			prefetch(shadow_q_gen->ents[prefetch_idx].dst.virt_addr);
			prefetch_idx = next_q_idx(prefetch_idx, PKT_GEN_APP_DSHADOW_Q_SIZE);
		}

#ifdef PKT_GEN_APP_VERBOSE_DEBUG
		if (unlikely(larg->cmn_args.verbose > 1)) {
			printf("received gen packet (va:%p, pa 0x%08x, len %d):\n",
			       shadow_q_gen->ents[inter_ind].dst.virt_addr,
			       (unsigned int)shadow_q_gen->ents[inter_ind].dst.phy_addr,
			       shadow_q_gen->ents[inter_ind].pkt_len);
			mem_disp(shadow_q_gen->ents[inter_ind].dst.virt_addr,
				shadow_q_gen->ents[inter_ind].pkt_len);
		}
#endif /* PKT_GEN_APP_VERBOSE_DEBUG */

		swap_l2(shadow_q_gen->ents[inter_ind].dst.virt_addr);
		swap_l3(shadow_q_gen->ents[inter_ind].dst.virt_addr);

		/* Prepare descriptors */
		dmax2_descs[i].desc_ctrl = DESC_OP_MODE_MEMCPY << DESC_OP_MODE_SHIFT;
		dmax2_descs[i].buff_size = shadow_q_gen->ents[inter_ind].pkt_len;
		dmax2_descs[i].src_addr = shadow_q_gen->ents[inter_ind].dst.phy_addr;
		dmax2_descs[i].dst_addr = shadow_q_anl->ents[write_ind].dst.phy_addr;
		dmax2_descs[i].flags = DESC_FLAGS_SYNC;
		/* desc_id/cookie = descriptor number, for easy comparison after Dequeue */
		dmax2_descs[i].desc_id = write_ind;
		dmax2_descs[i].data_buff_addr[0] = write_ind;

		shadow_q_anl->ents[write_ind].src = &shadow_q_gen->ents[inter_ind].dst;
		shadow_q_anl->ents[write_ind].pkt_len = shadow_q_gen->ents[inter_ind].pkt_len;

#ifdef PKT_GEN_APP_VERBOSE_DEBUG
		if (unlikely(larg->cmn_args.verbose > 1)) {
			printf("Echo packet (va:%p, pa 0x%08x, len %d):\n",
			       shadow_q_gen->ents[inter_ind].dst.virt_addr,
			       (unsigned int)shadow_q_gen->ents[inter_ind].dst.phy_addr,
			       shadow_q_gen->ents[inter_ind].pkt_len);
			mem_disp(shadow_q_gen->ents[inter_ind].dst.virt_addr,
				shadow_q_gen->ents[inter_ind].pkt_len);
		}
#endif /* PKT_GEN_APP_VERBOSE_DEBUG */

		write_ind = next_q_idx(write_ind, PKT_GEN_APP_DSHADOW_Q_SIZE);
		inter_ind = next_q_idx(inter_ind, PKT_GEN_APP_DSHADOW_Q_SIZE);
	}
	writew(write_ind, &shadow_q_anl->write_ind);
	num_got = i;

	err = dmax2_enq(eng_desc_anl->dmax2, dmax2_descs, &num_got);

	if (num_got < num) {
		pr_err("failed to enq to analayzer\n");
		if (unlikely(err)) {
			pr_err("DMAX2 EnQ (EnC) failed (%d)!\n", err);
			return -EFAULT;
		}

		for (i = num_got; i < num; i++) {
			if (write_ind == 0)
				write_ind = PKT_GEN_APP_DSHADOW_Q_SIZE;
			write_ind--;
			shadow_q_anl->ents[write_ind].src = NULL;
			if (inter_ind == 0)
				inter_ind = PKT_GEN_APP_DSHADOW_Q_SIZE;
			inter_ind--;
		}
		writew(write_ind, &shadow_q_anl->write_ind);
	}
	writew(inter_ind, &shadow_q_gen->inter_ind);
	larg->trf_cntrs.tx_drop += (num - num_got);
#ifdef PKT_GEN_APP_VERBOSE_DEBUG
	if (unlikely(larg->cmn_args.verbose && num_got))
		printf("sent %d pkts on analayzer\n", num_got);
#endif /* PKT_GEN_APP_VERBOSE_DEBUG */

	return num_got;
}

static inline int loop_rx(struct local_arg *larg)
{
	struct dmax2_trans_complete_desc dmax2_res_descs[PKT_GEN_APP_MAX_BURST_SIZE];
	struct dmax2_eng_desc		*eng_desc_gen = &(larg->eng_descs[PKT_GEN_APP_ENG_GEN]);
	struct dmax2_eng_desc		*eng_desc_anl = &(larg->eng_descs[PKT_GEN_APP_ENG_ANL]);
	struct dmax2_shadow_q		*shadow_q_gen, *shadow_q_anl;
#ifdef PKT_GEN_APP_VERBOSE_DEBUG
	struct pkt			*pkt;
	u32				*buff;
#endif /* PKT_GEN_APP_VERBOSE_DEBUG */
	u16				 read_ind, gen_read_ind;
	u16				 i, num;
	int				 err;
	u16				 prefetch_idx;
	int				 prefetch_shift = larg->cmn_args.prefetch_shift;

	shadow_q_gen = &eng_desc_gen->shadow_q;
	shadow_q_anl = &eng_desc_anl->shadow_q;

	num = PKT_GEN_APP_MAX_BURST_SIZE;

	err = dmax2_deq(eng_desc_anl->dmax2, dmax2_res_descs, &num, 1);
	if (unlikely(err)) {
		pr_err("DMAX2 DeQ (EnC) failed (%d)!\n", err);
		return -EFAULT;
	}
	if (!num)
		return 0;
#ifdef PKT_GEN_APP_VERBOSE_DEBUG
	if (unlikely(larg->cmn_args.verbose && num))
		printf("recv %d pkts on analayzer\n", num);
#endif /* PKT_GEN_APP_VERBOSE_DEBUG */
	larg->trf_cntrs.rx_pkts += num;

	gen_read_ind = shadow_q_gen->read_ind;
	read_ind = shadow_q_anl->read_ind;
	prefetch_idx = read_ind + prefetch_shift;
	if (prefetch_idx >= PKT_GEN_APP_DSHADOW_Q_SIZE)
		prefetch_idx -= PKT_GEN_APP_DSHADOW_Q_SIZE;

	for (i = 0; i < num; i++) {
		if (unlikely(!shadow_q_anl->ents[read_ind].src ||
			!shadow_q_anl->ents[read_ind].src->virt_addr)) {
			pr_warn("Shadow memory @%d: cookie(%p), pa(%lx)!\n",
				read_ind,
				shadow_q_anl->ents[read_ind].src,
				shadow_q_anl->ents[read_ind].src ?
					(u64)shadow_q_anl->ents[read_ind].src->virt_addr : 0);
			continue;
		}

#ifdef PKT_GEN_APP_VERBOSE_DEBUG
		if (unlikely(dmax2_res_descs[i].status))
			pr_err("Analayzer desc - Illegal status (%x)!\n", dmax2_res_descs[i].status);
		if (unlikely(dmax2_res_descs[i].desc_id != read_ind))
			pr_err("Analayzer desc - Illegal id (%d vs %d)!\n", dmax2_res_descs[i].desc_id, read_ind);

		pkt = (struct pkt *)shadow_q_anl->ents[read_ind].dst.virt_addr;
		buff = (u32 *)pkt->body;
		if (unlikely((buff[0] == MVAPPS_PLD_WATERMARK) || (buff[0] != read_ind)))
			pr_info("\r[ERROR] Analayzer: Illegal PLD: %x\n", buff[0]);
		else if (unlikely((pkt->ip.ip_v == IPVERSION) && (pkt->eh.ether_type == htons(0x800)))) {
			u16 tot_len = swab16(pkt->ip.ip_len) + sizeof(pkt->eh); /* assuming eth header */

			if (unlikely(tot_len != shadow_q_anl->ents[read_ind].pkt_len))
				pr_info("\r[ERROR] Analayzer: length mismatch (frame %u vs rx-desc %u @%d)!\n",
					tot_len, shadow_q_anl->ents[read_ind].pkt_len, read_ind);
		}

		/* Before return the buffer, mark it with watermark */
		pkt = (struct pkt *)shadow_q_gen->ents[gen_read_ind].dst.virt_addr;
		buff = (u32 *)pkt->body;
		buff[0] = MVAPPS_PLD_WATERMARK;
#endif /* PKT_GEN_APP_VERBOSE_DEBUG */

		if (num - i > prefetch_shift) {
			prefetch(shadow_q_anl->ents[prefetch_idx].dst.virt_addr);
			prefetch_idx = next_q_idx(prefetch_idx, PKT_GEN_APP_DSHADOW_Q_SIZE);
		}

#ifdef PKT_GEN_APP_VERBOSE_DEBUG
		if (unlikely(larg->cmn_args.verbose > 1)) {
			printf("received packet (va:%p, pa 0x%08x, len %d):\n",
			       shadow_q_anl->ents[read_ind].dst.virt_addr,
			       (unsigned int)shadow_q_anl->ents[read_ind].dst.phy_addr,
			       shadow_q_anl->ents[read_ind].pkt_len);
			mem_disp(shadow_q_anl->ents[read_ind].dst.virt_addr,
				shadow_q_anl->ents[read_ind].pkt_len);
		}
#endif /* PKT_GEN_APP_VERBOSE_DEBUG */

		larg->trf_cntrs.rx_bytes += shadow_q_anl->ents[read_ind].pkt_len;
		read_ind = next_q_idx(read_ind, PKT_GEN_APP_DSHADOW_Q_SIZE);
		gen_read_ind = next_q_idx(gen_read_ind, PKT_GEN_APP_DSHADOW_Q_SIZE);
	}
	writew(read_ind, &shadow_q_anl->read_ind);
	writew(gen_read_ind, &shadow_q_gen->read_ind);

	return num;
}

static int loop_tx(struct local_arg *larg, u16 num)
{
	struct dmax2_desc		 dmax2_descs[PKT_GEN_APP_MAX_BURST_SIZE];
	struct dmax2_eng_desc		*eng_desc = &(larg->eng_descs[PKT_GEN_APP_ENG_GEN]);
	struct buffer_desc		*buf_dec;
	struct dmax2_shadow_q		*shadow_q_gen;
#ifdef PKT_GEN_APP_VERBOSE_DEBUG
	struct pkt			*pkt;
	u32				*dat;
#endif /* PKT_GEN_APP_VERBOSE_DEBUG */
	u16				 write_ind;
	u16				 i, num_got, free_count;
	int				 err;

	shadow_q_gen = &eng_desc->shadow_q;

	write_ind = shadow_q_gen->write_ind;
	free_count = q_space(write_ind, mv_readw_relaxed(&shadow_q_gen->read_ind), PKT_GEN_APP_DSHADOW_Q_SIZE);
	if (num > free_count)
		num = free_count;
	if (!num)
		return 0;

	for (i = 0; i < num; i++) {
		buf_dec = &larg->buf_dec[larg->curr_frm];
		if (++larg->curr_frm == PKT_GEN_APP_BUFF_POOL_SIZE)
			larg->curr_frm = 0;

		/* Prepare descriptors */
		dmax2_descs[i].desc_ctrl = DESC_OP_MODE_MEMCPY << DESC_OP_MODE_SHIFT;
		dmax2_descs[i].buff_size = buf_dec->size;
		dmax2_descs[i].src_addr = buf_dec->phy_addr;
		dmax2_descs[i].dst_addr = shadow_q_gen->ents[write_ind].dst.phy_addr;
		dmax2_descs[i].flags = DESC_FLAGS_SYNC;
		/* desc_id/cookie = descriptor number, for easy comparison after Dequeue */
		dmax2_descs[i].desc_id = write_ind;
		dmax2_descs[i].data_buff_addr[0] = write_ind;

		shadow_q_gen->ents[write_ind].src = buf_dec;
		shadow_q_gen->ents[write_ind].pkt_len = buf_dec->size;

#ifdef PKT_GEN_APP_VERBOSE_DEBUG
		pkt = (struct pkt *)buf_dec->virt_addr;
		dat = (u32 *)pkt->body;
{
	int k;

	/* Write some watermark first */
	dat[0] = MVAPPS_PLD_WATERMARK;
	/* Now write some 'junk' in the size of L1C in order to make sure the previous
	 * write is in L2C already
	 */
	for (k = 0; k < sizeof(dummy_arr); k++)
		dummy_arr[k] = i;
}

		dat[0] = write_ind;
#endif /* PKT_GEN_APP_VERBOSE_DEBUG */

#ifdef PKT_GEN_APP_VERBOSE_DEBUG
		if (unlikely(larg->cmn_args.verbose > 1)) {
			printf("Sending packet (va:%p, pa 0x%08x, len %d):\n",
			       buf_dec->virt_addr,
			       (unsigned int)buf_dec->phy_addr,
			       buf_dec->size);
			mem_disp(buf_dec->virt_addr, buf_dec->size);
		}
#endif /* PKT_GEN_APP_VERBOSE_DEBUG */

		write_ind = next_q_idx(write_ind, PKT_GEN_APP_DSHADOW_Q_SIZE);
	}
	writew(write_ind, &shadow_q_gen->write_ind);
	num_got = i;

	err = dmax2_enq(eng_desc->dmax2, dmax2_descs, &num_got);

	if (num_got < num) {
		pr_err("failed to enq to generator\n");
		if (unlikely(err)) {
			pr_err("DMAX2 EnQ (EnC) failed (%d)!\n", err);
			return -EFAULT;
		}

		for (i = num_got; i < num; i++) {
			if (write_ind == 0)
				write_ind = PKT_GEN_APP_DSHADOW_Q_SIZE;
			write_ind--;
			shadow_q_gen->ents[write_ind].src = NULL;
		}
		writew(write_ind, &shadow_q_gen->write_ind);
	}
	larg->trf_cntrs.tx_pkts += num_got;
	for (i = 0; i < num_got; i++)
		larg->trf_cntrs.tx_bytes += dmax2_descs[i].buff_size;
	larg->trf_cntrs.tx_drop += (num - num_got);
#ifdef PKT_GEN_APP_VERBOSE_DEBUG
	if (unlikely(larg->cmn_args.verbose && num_got))
		printf("sent %d pkts on generator\n", num_got);
#endif /* PKT_GEN_APP_VERBOSE_DEBUG */

	return num_got;
}

static int main_loop_cb(void *arg, int *running)
{
	struct local_arg	*larg = (struct local_arg *)arg;
	int			err;
	int			rx = 0, tx = 0, echo = 0;
	u32			*total_frm_cnt;
	u16			num;

	if (!larg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	total_frm_cnt = &larg->cmn_args.garg->total_frm_cnt;
	if (larg->cmn_args.garg->cmn_args.cpus == 1)
		tx = rx = echo = 1;
	else if (larg->cmn_args.garg->cmn_args.cpus == 2) {
		if (larg->cmn_args.id == 0)
			tx = rx = 1;
		else
			echo = 1;
	} else if (larg->cmn_args.garg->cmn_args.cpus == 3) {
		if (larg->cmn_args.id == 0)
			tx = 1;
		else if (larg->cmn_args.id == 1)
			echo = 1;
		else
			rx = 1;
	}

	while (*running) {
		if (rx) {
			err = loop_rx(larg);
			if (unlikely(err < 0))
				return err;
			num = err;
			if (unlikely(*total_frm_cnt &&
				(*total_frm_cnt != PKT_GEN_APP_MAX_TOTAL_FRM_CNT))) {
				if (*total_frm_cnt < num)
					num = *total_frm_cnt;
				*total_frm_cnt -= num;
				if (!*total_frm_cnt)
					*total_frm_cnt = PKT_GEN_APP_MAX_TOTAL_FRM_CNT;
			}
		}

		if (unlikely(*total_frm_cnt == PKT_GEN_APP_MAX_TOTAL_FRM_CNT)) {
			*running = 0;
			continue;
		}

		if (echo) {
			err = loop_echo(larg);
			if (unlikely(err < 0))
				return err;
		}

		if (tx) {
			num = larg->cmn_args.burst;
			if (unlikely(*total_frm_cnt && (*total_frm_cnt < num)))
				num = *total_frm_cnt;
			err = loop_tx(larg, num);
			if (unlikely(err < 0))
				return err;
		}
	}

	return 0;
}

static int ctrl_loop_cb(void *arg)
{
	struct glob_arg *garg = (struct glob_arg *)arg;

	if (!garg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	if (!garg->cmn_args.cli)
		maintain_stats(garg);

	return 0;
}

static int init_global(void *arg)
{
	struct glob_arg		*garg = (struct glob_arg *)arg;
	struct dmax2_params	 dmax2_params;
	char			 name[15];
	int			 err;

	if (!garg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	if (pthread_mutex_init(&garg->trd_lock, NULL) != 0) {
		pr_err("init lock failed!\n");
		return -EIO;
	}

	pr_info("Global initializations ...\n");

	err = mv_sys_dma_mem_init(PKT_GEN_APP_DMA_MEM_SIZE);
	if (err)
		return err;

	/* Init the DMA-XORv2 modules */
	garg->eng_descs[PKT_GEN_APP_ENG_GEN].id = garg->base_engine;
	sprintf(name, "dmax2-%d", garg->eng_descs[PKT_GEN_APP_ENG_GEN].id);
	dmax2_params.match = name;
	dmax2_params.queue_size = PKT_GEN_APP_DMAX2_Q_SIZE;
	err = dmax2_init(&dmax2_params, &garg->eng_descs[PKT_GEN_APP_ENG_GEN].dmax2);
	if (err)
		return err;

	err = build_dmax2_shadow(&garg->eng_descs[PKT_GEN_APP_ENG_GEN].shadow_q,
				PKT_GEN_APP_DSHADOW_Q_SIZE,
				MAX_BUFF_SIZE);
	if (err)
		return err;

	garg->eng_descs[PKT_GEN_APP_ENG_ANL].id = garg->base_engine + 1;
	sprintf(name, "dmax2-%d", garg->eng_descs[PKT_GEN_APP_ENG_ANL].id);
	dmax2_params.match = name;
	err = dmax2_init(&dmax2_params, &garg->eng_descs[PKT_GEN_APP_ENG_ANL].dmax2);
	if (err)
		return err;

	err = build_dmax2_shadow(&garg->eng_descs[PKT_GEN_APP_ENG_ANL].shadow_q,
				PKT_GEN_APP_DSHADOW_Q_SIZE,
				MAX_BUFF_SIZE);
	if (err)
		return err;

	err = app_build_pkt_pool(&garg->buffer,
				garg->buf_dec,
				PKT_GEN_APP_BUFF_POOL_SIZE,
				MIN_PKT_SIZE,
				MAX_PKT_SIZE,
				garg->pkt_size,
				&garg->src_ip,
				&garg->dst_ip,
				garg->src_mac,
				garg->dst_mac);
	if (err)
		return err;

	pr_info("done\n");
	return 0;
}

static int init_local(void *arg, int id, void **_larg)
{
	struct glob_arg		*garg = (struct glob_arg *)arg;
	struct local_arg	*larg;

	if (!garg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	pr_info("Local TH%d initializations ...\n", id);

	larg = (struct local_arg *)malloc(sizeof(struct local_arg));
	if (!larg) {
		pr_err("No mem for local arg obj!\n");
		return -ENOMEM;
	}
	memset(larg, 0, sizeof(struct local_arg));

	larg->cmn_args.id		= id;
	larg->cmn_args.garg             = garg;
	larg->cmn_args.burst		= garg->cmn_args.burst;
	larg->cmn_args.busy_wait	= garg->cmn_args.busy_wait;
	larg->cmn_args.echo             = garg->cmn_args.echo;
	larg->cmn_args.prefetch_shift	= garg->cmn_args.prefetch_shift;
	larg->cmn_args.verbose		= garg->cmn_args.verbose;

	larg->buf_dec = garg->buf_dec;
	larg->eng_descs = garg->eng_descs;

	garg->cmn_args.largs[id] = larg;

	*_larg = larg;

	pr_info("done\n");
	return 0;
}

static void usage(char *progname)
{
	printf("\n"
	       "MUSDK DMA-XOR v2 packet-gen application.\n"
	       "\n"
	       "Usage: %s OPTIONS\n"
	       "  E.g. %s -i eth0,eth1 -c 1\n"
	       "\n"
	       "Mandatory OPTIONS:\n"
	       "\t-i, --interface <Eth-interface>\n"
	       "Optional OPTIONS:\n"
	       "\t-b, --burst <size>          burst size, num_pkts handled in a batch.(default is %d)\n"
	       "\t-l, --size <pkt-size|inc|rand>   packet size in bytes (excluding CRC) or the following:\n"
	       "\t                            inc - incremental; increase every pkt by 4B.\n"
	       "\t                            rand - random packet sizes.\n"
	       "\t                            (default is fixed - %d)\n"
	       "\t-d, --dst-ip <d.d.d.d[:port[-d.d.d.d:port]]>\n"
	       "\t                            destination IP address or range\n"
	       "\t-s, --src-ip <d.d.d.d[:port[-d.d.d.d:port]]>\n"
	       "\t                            source IP address or range\n"
	       "\t-D, --dst-mac <XX:XX:XX:XX:XX:XX>    destination MAC address\n"
	       "\t-S, --src-mac <XX:XX:XX:XX:XX:XX>    source MAC address\n"
	       "\t-R, --rate-limit <pps>      maximum rate to generate in packets-per-second (default is none).\n"
	       "\t                            'K'/'M' may be used for KILO/MEGA (e.g. 100K for 100000pps).\n"
	       "\t-C, --count <num-frms>      maximum number of frame to be sent (default is infinite).\n"
	       "Common OPTIONS:\n"
	       "\t-T, --report-time <second>  time in seconds between reports.(default is %ds)\n"
	       "\t-c, --cores <number>        number of CPUs to use\n"
	       "\t-a, --affinity <number>     first CPU ID for setaffinity (default is no affinity)\n"
#ifdef PKT_GEN_APP_VERBOSE_DEBUG
	       "\t-v, --verbose               Increase verbose debug (default is 0).\n"
	       "\t                            With every '-v', the debug is increased by one.\n"
	       "\t                            0 - none, 1 - pkts sent/recv indication, 2 - full pkt dump\n"
#endif /* PKT_GEN_APP_VERBOSE_DEBUG */
	       "\t--cli                       Use CLI\n"
	       "\t-h, --help                  Display help and exit\n\n"
	       "\n", MVAPPS_NO_PATH(progname), MVAPPS_NO_PATH(progname),
	       PKT_GEN_APP_DFLT_BURST_SIZE, MAX_PKT_SIZE, DEFAULT_REPORT_TIME
	       );
}

/*
 * extract the extremes from a range of ipv4 addresses.
 * addr_lo[-addr_hi][:port_lo[-port_hi]]
 */
static int extract_ip_range(struct ip_range *r, char *argv)
{
	char *ap, *pp;
	struct in_addr a;

	r->port0 = r->port1 = 0;
	r->start = r->end = 0;

	/* the first - splits start/end of range */
	ap = index(argv, '-');	/* do we have ports ? */
	if (ap)
		*ap++ = '\0';
	/* grab the initial values (mandatory) */
	pp = index(argv, ':');
	if (pp) {
		*pp++ = '\0';
		r->port0 = r->port1 = strtol(pp, NULL, 0);
	};
	inet_aton(argv, &a);
	r->start = r->end = ntohl(a.s_addr);
	if (ap) {
		pp = index(ap, ':');
		if (pp) {
			*pp++ = '\0';
			if (*pp)
				r->port1 = strtol(pp, NULL, 0);
		}
		if (*ap) {
			inet_aton(ap, &a);
			r->end = ntohl(a.s_addr);
		}
	}
	if (r->port0 > r->port1) {
		u16 tmp = r->port0;

		r->port0 = r->port1;
		r->port1 = tmp;
	}
	if (r->start > r->end) {
		u32 tmp = r->start;

		r->start = r->end;
		r->end = tmp;
	}
	{
		struct in_addr a;
		char buf1[16]; /* one ip address */

		a.s_addr = htonl(r->end);
		strncpy(buf1, inet_ntoa(a), sizeof(buf1));
		a.s_addr = htonl(r->start);
		pr_debug("range is %s:%d to %s:%d\n", inet_ntoa(a), r->port0, buf1, r->port1);
	}

	r->port_curr = r->port0;
	r->curr = r->start;

	return 0;
}

static int parse_args(struct glob_arg *garg, int argc, char *argv[])
{
	int option;
	int long_index = 0;
	int rv;
	int mult;
	int max_cpus;
	const char short_options[] = "hi:b:l:c:a:m:T:w:R:C:S:D:s:d:rtv";
	struct option long_options[] = {
		{"help", no_argument, 0, 'h'},
		{"burst", required_argument, 0, 'b'},
		{"size", required_argument, 0, 'l'},
		{"cores", required_argument, 0, 'c'},
		{"affinity", required_argument, 0, 'a'},
		{"report-time", required_argument, 0, 'T'},
		{"rate-limit", required_argument, 0, 'R'},
		{"count", required_argument, 0, 'C'},
		{"src-mac", required_argument, 0, 'S'},
		{"dst-mac", required_argument, 0, 'D'},
		{"src-ip", required_argument, 0, 's'},
		{"dst-ip", required_argument, 0, 'd'},
		{"verbose", no_argument, 0, 'v'},
		{"cli", no_argument, &garg->cmn_args.cli, 1},
		{0, 0, 0, 0}
	};
#ifdef PKT_GEN_APP_VERBOSE_DEBUG
	garg->cmn_args.verbose = 0;
#endif /* PKT_GEN_APP_VERBOSE_DEBUG */
	garg->rx = 0;
	garg->tx = 0;
	garg->total_frm_cnt = 0;
	garg->cmn_args.cli = 0;
	garg->cmn_args.cpus = 1;
	garg->cmn_args.affinity = MVAPPS_INVALID_AFFINITY;
	garg->cmn_args.burst = PKT_GEN_APP_DFLT_BURST_SIZE;
	garg->cmn_args.mtu = DEFAULT_MTU;
	garg->cmn_args.prefetch_shift = PKT_GEN_APP_PREFETCH_SHIFT;
	garg->cmn_args.ctrl_thresh = PKT_GEN_APP_STATS_DFLT_THR;
	garg->cmn_args.pkt_offset = 0;
	garg->cmn_args.busy_wait	= DEFAULT_RATE_USECS;
	garg->report_time = DEFAULT_REPORT_TIME;
	garg->pkt_size = MAX_PKT_SIZE;
	garg->src_ip.start = garg->src_ip.end = garg->src_ip.curr = DEFAULT_SRC_IP;
	garg->src_ip.port0 = garg->src_ip.port1 = garg->src_ip.port_curr = DEFAULT_SRC_PORT;
	garg->dst_ip.start = garg->dst_ip.end = garg->dst_ip.curr = DEFAULT_DST_IP;
	garg->dst_ip.port0 = garg->dst_ip.port1 = garg->dst_ip.port_curr = DEFAULT_DST_PORT;
	memcpy(garg->dst_mac, default_dst_mac, MV_ETH_ALEN);
	memcpy(garg->src_mac, default_src_mac, MV_ETH_ALEN);

	/* TODO: init hardcoded ports?!?!?! */
	garg->cmn_args.num_ports = 1;

	optind = 0;
	while ((option = getopt_long(argc, argv, short_options, long_options, &long_index)) != -1) {
		switch (option) {
		case 0:
			/* only options which set a flag return zero, so do nothing. */
			break;
		case 'h':
			usage(argv[0]);
			exit(0);
			break;
		case 'b':
			garg->cmn_args.burst = atoi(optarg);
			pr_debug("Set burst size to %d\n", garg->cmn_args.burst);
			break;
		case 'l':
			if (strcmp(optarg, "inc") == 0) {
				garg->pkt_size = MVAPPS_PKT_SIZE_INC;
				pr_debug("Set packet size to incremental\n");
			} else if (strcmp(optarg, "rand") == 0) {
				garg->pkt_size = MVAPPS_PKT_SIZE_RAND;
				pr_debug("Set packet size to incremental\n");
			} else if (strcmp(optarg, "imix") == 0) {
				garg->pkt_size = MVAPPS_PKT_SIZE_IMIX;
				pr_debug("Set packet size to incremental\n");
			} else {
				garg->pkt_size = atoi(optarg);
				pr_debug("Set packet size to %d\n", garg->pkt_size);
			}
			break;
		case 'c':
			garg->cmn_args.cpus = atoi(optarg);
			pr_debug("Set number of cores to %d\n", garg->cmn_args.cpus);
			break;
		case 'a':
			garg->cmn_args.affinity = atoi(optarg);
			pr_debug("Set first cpu id to %d\n", garg->cmn_args.affinity);
			break;
		case 'T':
			garg->report_time = atoi(optarg);
			pr_debug("Set report_time to %d\n", garg->report_time);
			break;
		case 'R':
			mult = 1;
			if (optarg[strlen(optarg)-1] == 'K') {
				optarg[strlen(optarg)-1] = '\0';
				mult = 1000;
			} else if (optarg[strlen(optarg)-1] == 'M') {
				optarg[strlen(optarg)-1] = '\0';
				mult = 1000000;
			}
			garg->cmn_args.busy_wait = atoi(optarg) * mult;
			break;
		case 'C':
			garg->total_frm_cnt = atoi(optarg);
			pr_debug("Set total frame Count to %d\n", garg->total_frm_cnt);
			break;
		case 'S':
			rv = sscanf(optarg, "%02hhx:%02hhx:%02hhx:%02hhx:%02hhx:%02hhx",
				    &garg->src_mac[0], &garg->src_mac[1], &garg->src_mac[2],
				&garg->src_mac[3], &garg->src_mac[4], &garg->src_mac[5]);
			if (rv != 6) {
				pr_err("Failed to parse -S parameter (%d)\n", rv);
				return -EINVAL;
			}
			pr_debug("Set src_mac to: %.2x:%.2x:%.2x:%.2x:%.2x:%.2x\n",
				garg->src_mac[0], garg->src_mac[1], garg->src_mac[2],
				garg->src_mac[3], garg->src_mac[4], garg->src_mac[5]);
			break;
		case 'D':
			rv = sscanf(optarg, "%02hhx:%02hhx:%02hhx:%02hhx:%02hhx:%02hhx",
				    &garg->dst_mac[0], &garg->dst_mac[1], &garg->dst_mac[2],
				&garg->dst_mac[3], &garg->dst_mac[4], &garg->dst_mac[5]);
			if (rv != 6) {
				pr_err("Failed to parse -D parameter (%d)\n", rv);
				return -EINVAL;
			}
			pr_debug("Set dst_mac to: %.2x:%.2x:%.2x:%.2x:%.2x:%.2x\n",
				garg->dst_mac[0], garg->dst_mac[1], garg->dst_mac[2],
				garg->dst_mac[3], garg->dst_mac[4], garg->dst_mac[5]);
			break;
		case 's':
			if (extract_ip_range(&garg->src_ip, optarg) < 0)
				return -EINVAL;
			pr_debug("Set src_ip to: %.4X\n", garg->src_ip.start);
			break;
		case 'd':
			if (extract_ip_range(&garg->dst_ip, optarg) < 0)
				return -EINVAL;
			pr_debug("Set dst_ip to: %.4X\n", garg->dst_ip.start);
			break;
		case 'v':
			garg->cmn_args.verbose++;
			pr_debug("Set verbose to %d\n", garg->cmn_args.verbose);
			break;
		default:
			pr_err("argument (%c) not supported!\n", option);
			return -EINVAL;
		}
	}

	if (garg->cmn_args.burst > PKT_GEN_APP_MAX_BURST_SIZE) {
		pr_err("illegal burst size requested (%d vs %d)!\n",
		       garg->cmn_args.burst, PKT_GEN_APP_MAX_BURST_SIZE);
		return -EINVAL;
	}
	max_cpus = PKT_GEN_APP_MAX_NUM_CPUS;
	if (max_cpus > system_ncpus())
		max_cpus = system_ncpus();
	if (garg->cmn_args.cpus > max_cpus) {
		pr_err("illegal num cores requested (%d vs %d)!\n",
		       garg->cmn_args.cpus, max_cpus);
		return -EINVAL;
	}
	if ((garg->cmn_args.affinity != -1) &&
	    ((garg->cmn_args.cpus + garg->cmn_args.affinity) > max_cpus)) {
		pr_err("illegal num cores or affinity requested (%d,%d vs %d)!\n",
		       garg->cmn_args.cpus, garg->cmn_args.affinity, max_cpus);
		return -EINVAL;
	}

	if (garg->pkt_size > 0) {
		if (garg->pkt_size > MAX_PKT_SIZE) {
			pr_err("illegal packet size (%d vs %d)!\n",
			       garg->pkt_size, MAX_PKT_SIZE);
			return -EINVAL;
		}
		if (garg->pkt_size < MIN_PKT_SIZE) {
			pr_err("illegal packet size (%d vs %d)!\n",
			       garg->pkt_size, MIN_PKT_SIZE);
			return -EINVAL;
		}
	} else if ((garg->pkt_size != MVAPPS_PKT_SIZE_INC) &&
		(garg->pkt_size != MVAPPS_PKT_SIZE_RAND) &&
		(garg->pkt_size != MVAPPS_PKT_SIZE_IMIX)) {
		pr_err("illegal packet size!\n");
		return -EINVAL;
	}

	if (garg->total_frm_cnt >= PKT_GEN_APP_MAX_TOTAL_FRM_CNT) {
		pr_err("illegal total frames count (%d vs %d)!\n",
		       garg->total_frm_cnt, PKT_GEN_APP_MAX_TOTAL_FRM_CNT);
		return -EINVAL;
	}

	/* in case rate-limit was requested, convert here from pkts-per-second given
	 * by the user to busy-wait time in u-secs
	 */
	if (garg->cmn_args.busy_wait) {
		/* we need to calculate here how much time we need to wait in u-secs
		 * per burst. so, the formula is: wait = (1000000/PPS)*burst
		 */
		u64 tmp = 1000000 * garg->cmn_args.burst;

		garg->cmn_args.busy_wait = tmp / garg->cmn_args.busy_wait;
	}

	return 0;
}


int main(int argc, char *argv[])
{
	struct mvapp_params	mvapp_params;
	int			err;

	setbuf(stdout, NULL);

	pr_info("DMAX2 pkt-gen is started\n");
	pr_debug("pr_debug is enabled\n");

	err = parse_args(&garg, argc, argv);
	if (err)
		return err;

	garg.cmn_args.cores_mask = apps_cores_mask_create(garg.cmn_args.cpus, garg.cmn_args.affinity);

	memset(&mvapp_params, 0, sizeof(mvapp_params));
	mvapp_params.use_cli		= garg.cmn_args.cli;
	mvapp_params.num_cores		= garg.cmn_args.cpus;
	mvapp_params.cores_mask		= garg.cmn_args.cores_mask;
	mvapp_params.global_arg		= (void *)&garg;
	mvapp_params.init_global_cb	= init_global;
	mvapp_params.deinit_global_cb	= NULL;
	mvapp_params.init_local_cb	= init_local;
	mvapp_params.deinit_local_cb	= NULL;
	mvapp_params.main_loop_cb	= main_loop_cb;
	mvapp_params.ctrl_cb		= ctrl_loop_cb;

	return mvapp_go(&mvapp_params);
}
