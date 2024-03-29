/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#define _GNU_SOURCE         /* See feature_test_macros(7) */
#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <sched.h>
#include <arpa/inet.h>	/* ntohs */
#include <netinet/if_ether.h>
#include <netinet/ip.h>
#include <netinet/udp.h>
#include <getopt.h>
#include <sys/time.h>

#include "mv_std.h"
#include "lib/lib_misc.h"
#include "env/mv_sys_dma.h"
#include "env/io.h"

#include "mvapp.h"
#include "mv_pp2.h"
#include "mv_pp2_hif.h"
#include "mv_pp2_bpool.h"
#include "mv_pp2_ppio.h"

#include "pp2_utils.h"

#define PKT_GEN_APP_TX_RETRY_WAIT		1
#define PKT_GEN_APP_DEF_Q_SIZE			1024
#define PKT_GEN_APP_HIF_Q_SIZE			(8 * PKT_GEN_APP_DEF_Q_SIZE)
#define PKT_GEN_APP_RX_Q_SIZE			(2 * PKT_GEN_APP_DEF_Q_SIZE)
#define PKT_GEN_APP_TX_Q_SIZE			(2 * PKT_GEN_APP_DEF_Q_SIZE)

#define PKT_GEN_APP_MAX_BURST_SIZE		((PKT_GEN_APP_RX_Q_SIZE) >> 1)
#define PKT_GEN_APP_DFLT_BURST_SIZE		256

#define PKT_GEN_APP_BUFF_POOL_SIZE		8192

#define PKT_GEN_APP_DMA_MEM_SIZE		(80 * 1024 * 1024)
/* 1 sec. threshold that is used for the perf statistics prints */
#define PKT_GEN_APP_STATS_DFLT_THR		1000

#define PKT_GEN_APP_FIRST_INQ			0
#define PKT_GEN_APP_MAX_NUM_TCS_PER_PORT	1
#define PKT_GEN_APP_MAX_NUM_QS_PER_CORE	PKT_GEN_APP_MAX_NUM_TCS_PER_PORT

#define PKT_GEN_APP_DIR_RX			0x1
#define PKT_GEN_APP_DIR_TX			0x2

#define PKT_GEN_APP_LINK_UP_THR			2000 /* 2 secs */

#define PKT_GEN_APP_MAX_TOTAL_FRM_CNT		UINT32_MAX

#define PKT_GEN_APP_LTNC_FRM_CNT		10000000
#define PKT_GEN_APP_LTNC_BURST			16
#define PKT_GEN_APP_LTNC_MINE			15

#define PKT_GEN_APP_USE_HW_RATE_LMT
#define PKT_GEN_APP_RATE_LMT_CBS		1000000

#define  PKT_GEN_APP_HW_TX_CHKSUM_CALC
#ifdef PKT_GEN_APP_HW_TX_CHKSUM_CALC
#define PKT_GEN_APP_HW_TX_L4_CHKSUM_CALC	1
#define PKT_GEN_APP_HW_TX_IPV4_CHKSUM_CALC	1
#endif /* PKT_GEN_APP_HW_TX_CHKSUM_CALC */

#define PKT_GEN_APP_PREFETCH_SHIFT		4

#define PKT_GEN_APP_BPOOLS_INF		{ {384, 4096, 0, NULL}, {2048, 4096, 0, NULL} }
#define PKT_GEN_APP_BPOOLS_JUMBO_INF	{ {2048, 4096, 0, NULL}, {10240, 512, 0, NULL} }

#define MAX_BODYSIZE			(DEFAULT_MTU - IPV4_HDR_LEN - UDP_HDR_LEN)
#define MIN_PKT_SIZE			(40)
#define MAX_PKT_SIZE			(DEFAULT_MTU + ETH_HLEN)

/* The following macro is need in order to extract pkt length out from
 * the pp2 outQ descriptor; see pp2_ppio_outq_desc_set_pkt_len() in mv_pp2_ppio.h
 */
#define PP2_OUTQ_DESC_PKT_LEN(d)	((d)->cmds[1] >> 16)

#define DEFAULT_REPORT_TIME 1 /* 1 second */
#define DEFAULT_RATE_USECS  0
#define DEFAULT_SRC_IP 0x0a000001
#define DEFAULT_DST_IP 0x0a000002
#define DEFAULT_SRC_PORT 1024
#define DEFAULT_DST_PORT 1024

#define PKT_GEN_NUM_CNTS	2
#define PKT_GEN_CNT_PKTS	0
#define PKT_GEN_CNT_BYTES	1


struct pkt {
	struct ether_header	 eh;
	struct ip		 ip;
	struct udphdr		 udp;
	/* using 'empty array' as placeholder; the real size will be determine by the implementation
	 */
	u8			 body[];
} __packed;

struct glob_arg {
	struct glb_common_args	cmn_args; /* Keep first */

	u16			rxq_size;
	int			loopback;
	int			maintain_stats;
	pthread_mutex_t		trd_lock;

	int			running;
	int			rx;
	int			tx;
	int			latency;
	u32			total_frm_cnt;
	u8			max_rx_cpu;
	u8			min_tx_cpu;

	struct ip_range		src_ip;
	struct ip_range		dst_ip;
	eth_addr_t		dst_mac;
	eth_addr_t		src_mac;
	int			pkt_size;
	u32			report_time;
};

struct traffic_counters {
	u64		tx_pkts;
	u64		tx_bytes;
	u64		tx_drop;
	u64		rx_pkts;
	u64		rx_bytes;
};

struct local_arg {
	struct local_common_args	cmn_args; /* Keep first */

	u32				total_frm_cnt;
	u16				curr_frm;

	struct buffer_desc		buf_dec[PKT_GEN_APP_BUFF_POOL_SIZE];

	struct traffic_counters		trf_cntrs;

	int				latency;
	u64				latency_agg_time;
	u64				latency_num_pkts;

	void				*buffer;
	int				pkt_size;
};


eth_addr_t default_src_mac = {0x00, 0x00, 0x00, 0x00, 0x00, 0x01};
eth_addr_t default_dst_mac = {0x00, 0x00, 0x00, 0x00, 0x00, 0x02};

/* globals for ingress/egress packet rate statistics */
u64			 lst_rx_cnts[PKT_GEN_NUM_CNTS][MVAPPS_PP2_MAX_NUM_PORTS];
u64			 lst_tx_cnts[PKT_GEN_NUM_CNTS][MVAPPS_PP2_MAX_NUM_PORTS];

static struct glob_arg garg = {};


/*
 * increment the addressed in the packet,
 * starting from the least significant field.
 *	DST_IP DST_PORT SRC_IP SRC_PORT
 */
static void update_addresses(struct pkt *pkt, struct glob_arg *garg)
{
	do {
		/* XXX for now it doesn't handle non-random src, random dst */
		pkt->udp.uh_sport = htons(garg->src_ip.port_curr++);
		if (garg->src_ip.port_curr >= garg->src_ip.port1)
			garg->src_ip.port_curr = garg->src_ip.port0;

		pkt->ip.ip_src.s_addr = htonl(garg->src_ip.curr++);
		if (garg->src_ip.curr >= garg->src_ip.end)
			garg->src_ip.curr = garg->src_ip.start;

		pkt->udp.uh_dport = htons(garg->dst_ip.port_curr++);
		if (garg->dst_ip.port_curr >= garg->dst_ip.port1)
			garg->dst_ip.port_curr = garg->dst_ip.port0;

		pkt->ip.ip_dst.s_addr = htonl(garg->dst_ip.curr++);
		if (garg->dst_ip.curr >= garg->dst_ip.end)
			garg->dst_ip.curr = garg->dst_ip.start;
	} while (0);
	/* update checksum */
}

static int dump_latency(struct local_arg *larg)
{
	struct glob_arg *garg;
	u64 pkts, tmp_time_inter;
	u8 i, j;

	if (unlikely(!larg)) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	/* make sure we calc the latency only once on thread #0 */
	if (larg->cmn_args.id != 0)
		return 0;
	garg = larg->cmn_args.garg;

	for (j = 0; j < MVAPPS_PP2_MAX_NUM_PORTS; j++) {
		/* TODO: temporary, we don't realy support multiple-ports statistics
		 * since we have only one set of counters
		 */
		if (j)
			break;
		pkts = tmp_time_inter = 0;
		for (i = 0; i < garg->cmn_args.cpus; i++) {
			pkts		+= (garg->cmn_args.largs[i])->latency_num_pkts;
			tmp_time_inter	+= (garg->cmn_args.largs[i])->latency_agg_time;
		}
	}
	tmp_time_inter = tmp_time_inter / pkts;
	tmp_time_inter -= larg->cmn_args.busy_wait;
	tmp_time_inter -= PKT_GEN_APP_LTNC_MINE;

	printf("Latency: %d u-secs\n", (int)tmp_time_inter);

	return 0;
}

static int dump_perf(struct glob_arg *garg)
{
	struct timeval	 curr_time;
	u64		 tmp_time_inter;
	u64 pkts, bytes, drops, pps, bps;
	u8 i, j;

	gettimeofday(&curr_time, NULL);
	tmp_time_inter = (curr_time.tv_sec - garg->cmn_args.ctrl_trd_last_time.tv_sec) * 1000;
	tmp_time_inter += (curr_time.tv_usec - garg->cmn_args.ctrl_trd_last_time.tv_usec) / 1000;

	printf("\r");
	if (garg->rx) {
		pps = bps = 0;
		for (j = 0; j < MVAPPS_PP2_MAX_NUM_PORTS; j++) {
			/* TODO: temporary, we don't realy support multiple-ports statistics
			 * since we have only one set of counters
			 */
			if (j)
				break;
			pkts = bytes = 0;
			for (i = 0; i < garg->cmn_args.cpus; i++) {
				pkts  += (garg->cmn_args.largs[i])->trf_cntrs.rx_pkts;
				bytes += (garg->cmn_args.largs[i])->trf_cntrs.rx_bytes;
			}
			pps += pkts - lst_rx_cnts[PKT_GEN_CNT_PKTS][j];
			bps += bytes - lst_rx_cnts[PKT_GEN_CNT_BYTES][j];
			lst_rx_cnts[PKT_GEN_CNT_PKTS][j] = pkts;
			lst_rx_cnts[PKT_GEN_CNT_BYTES][j] = bytes;
		}
		pps /= tmp_time_inter;
		bps = bps * 8 / tmp_time_inter;

		printf("RX: %" PRIu64 " Kpps, %" PRIu64 " Kbps\t", pps, bps);
	}

	if (garg->tx) {
		drops = pps = bps = 0;

		for (j = 0; j < MVAPPS_PP2_MAX_NUM_PORTS; j++) {
			/* TODO: temporary, we don't realy support multiple-ports statistics
			 * since we have only one set of counters
			 */
			if (j)
				break;
			pkts = bytes = 0;
			for (i = 0; i < garg->cmn_args.cpus; i++) {
				pkts  += (garg->cmn_args.largs[i])->trf_cntrs.tx_pkts;
				bytes += (garg->cmn_args.largs[i])->trf_cntrs.tx_bytes;
				drops += (garg->cmn_args.largs[i])->trf_cntrs.tx_drop;
			}
			pps += pkts - lst_tx_cnts[PKT_GEN_CNT_PKTS][j];
			bps += bytes - lst_tx_cnts[PKT_GEN_CNT_BYTES][j];
			lst_tx_cnts[PKT_GEN_CNT_PKTS][j] = pkts;
			lst_tx_cnts[PKT_GEN_CNT_BYTES][j] = bytes;
		}
		pps /= tmp_time_inter;
		bps = bps * 8 / tmp_time_inter;

		printf("TX: %" PRIu64 " Kpps, %" PRIu64 " Kbps (%" PRIu64 " Kdrops)\t",
			pps, bps, drops / 1000);
	}

	gettimeofday(&garg->cmn_args.ctrl_trd_last_time, NULL);

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
	tmp_time_inter = (curr_time.tv_sec - garg->cmn_args.ctrl_trd_last_time.tv_sec) * 1000;
	tmp_time_inter += (curr_time.tv_usec - garg->cmn_args.ctrl_trd_last_time.tv_usec) / 1000;
	if (tmp_time_inter >= garg->cmn_args.ctrl_thresh)
		return dump_perf(garg);

	return 0;
}

static int maintain_cnts(struct glob_arg *garg)
{
	int completed = 0;
	u64 r_pkts = 0, t_pkts = 0, r_bytes = 0, t_bytes = 0;
	u8 i, j;

	if (!garg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	if (likely(!garg->total_frm_cnt))
		return 0;

	if (!garg->running)
		return 1;

	if (garg->rx) {
		for (j = 0; j < MVAPPS_PP2_MAX_NUM_PORTS; j++) {
			/* TODO: temporary, we don't realy support multiple-ports statistics
			 * since we have only one set of counters
			 */
			if (j)
				break;
			for (i = 0; i < garg->cmn_args.cpus; i++) {
				r_pkts  += (garg->cmn_args.largs[i])->trf_cntrs.rx_pkts;
				r_bytes += (garg->cmn_args.largs[i])->trf_cntrs.rx_bytes;
			}
		}
		if (r_pkts >= garg->total_frm_cnt)
			completed = 1;
	}

	if (garg->tx) {
		for (j = 0; j < MVAPPS_PP2_MAX_NUM_PORTS; j++) {
			/* TODO: temporary, we don't realy support multiple-ports statistics
			 * since we have only one set of counters
			 */
			if (j)
				break;
			for (i = 0; i < garg->cmn_args.cpus; i++) {
				t_pkts  += (garg->cmn_args.largs[i])->trf_cntrs.tx_pkts;
				t_bytes += (garg->cmn_args.largs[i])->trf_cntrs.tx_bytes;
			}
		}
		if (t_pkts >= garg->total_frm_cnt) {
			garg->running++;
			/* stop running if either no RX or we wait enough time to receive traffic */
			if (!garg->rx || (garg->running >= 20))
				completed = 1;
			if (completed)
				printf("\nRX: %" PRIu64 " pkts, %" PRIu64 " bytes\t"
					"TX: %" PRIu64 " pkts, %" PRIu64 " bytes\n",
					 r_pkts, r_bytes, t_pkts, t_bytes);
		}
	}

	if (completed) {
		garg->running = 0;
		return 1;
	}

	return 0;
}

static inline int loop_rx(struct local_arg	*larg,
			  u8			 rx_ppio_id,
			  u8			 tc,
			  u8			 qid,
			  u16			 num)
{
	struct tx_shadow_q	*shadow_q;
	int			 shadow_q_size;
	struct pp2_ppio_desc	 descs[PKT_GEN_APP_MAX_BURST_SIZE];
	struct pp2_lcl_common_args *pp2_args = (struct pp2_lcl_common_args *)larg->cmn_args.plat;
	u16			 i, j;

	shadow_q = &pp2_args->lcl_ports_desc[rx_ppio_id].shadow_qs[tc];
	shadow_q_size = pp2_args->lcl_ports_desc[rx_ppio_id].shadow_q_size;

	pp2_ppio_recv(pp2_args->lcl_ports_desc[rx_ppio_id].ppio, tc, qid, descs, &num);
	if (unlikely(larg->cmn_args.verbose && num))
		printf("recv %d pkts on ppio %d, tc %d, qid %d\n", num, rx_ppio_id, tc, qid);

	larg->trf_cntrs.rx_pkts += num;

	for (i = 0; i < num; i++) {
		char *buff = (char *)(app_get_high_addr() | (uintptr_t)pp2_ppio_inq_desc_get_cookie(&descs[i]));
		dma_addr_t pa = pp2_ppio_inq_desc_get_phys_addr(&descs[i]);
		u16 len = pp2_ppio_inq_desc_get_pkt_len(&descs[i]);
		struct pp2_bpool *bpool = pp2_ppio_inq_desc_get_bpool(&descs[i],
								      pp2_args->lcl_ports_desc[rx_ppio_id].ppio);

		if (unlikely(larg->cmn_args.verbose > 1)) {
			char *tmp_buff;

			tmp_buff = buff;
			pr_debug("buff(%p)\n", tmp_buff);
			tmp_buff += MVAPPS_PP2_PKT_DEF_EFEC_OFFS;
			printf("Received packet (va:%p, pa 0x%08x, len %d):\n",
			       tmp_buff,
			       (unsigned int)pa,
			       len);
			mem_disp(tmp_buff, len);
		}

		if (unlikely(larg->latency)) {
			char		*tmp_buff;
			u64		*dat;

			tmp_buff = buff;
			tmp_buff += MVAPPS_PP2_PKT_DEF_EFEC_OFFS;
			dat = (u64 *)((struct pkt *)tmp_buff)->body;
			if (unlikely(dat[0])) {
				struct timeval	 curr_time;
				u64		 tmp_time_inter;

				gettimeofday(&curr_time, NULL);
				/* Allow maximum of 10 secs latency */
				if (likely(((u64)curr_time.tv_sec - dat[0]) <= 10)) {
					tmp_time_inter = ((u64)curr_time.tv_sec - dat[0]) * 1000000;
					if (unlikely(tmp_time_inter && (curr_time.tv_usec < dat[1])))
						tmp_time_inter -= dat[1] - (u64)curr_time.tv_usec;
					else
						tmp_time_inter += (u64)curr_time.tv_usec - (suseconds_t)dat[1];
					larg->latency_agg_time += tmp_time_inter;
					larg->latency_num_pkts++;
				}
			}
		}

		larg->trf_cntrs.rx_bytes += (len + ETH_FCS_LEN + ETH_IPG_LEN);

		shadow_q->ents[shadow_q->write_ind].buff_ptr.cookie = (uintptr_t)buff;
		shadow_q->ents[shadow_q->write_ind].buff_ptr.addr = pa;
		shadow_q->ents[shadow_q->write_ind].bpool = bpool;
		pr_debug("buff_ptr.cookie(0x%lx)\n", shadow_q->ents[shadow_q->write_ind].buff_ptr.cookie);
		shadow_q->write_ind++;
		if (shadow_q->write_ind == shadow_q_size)
			shadow_q->write_ind = 0;

		/* Below condition should never happen, if shadow_q size is large enough */
		if (unlikely(shadow_q->write_ind == shadow_q->read_ind)) {
			pr_err("%s: port(%d), tc(%d) qid(%d), shadow_q size=%d is too small, perform emergency drops\n",
				__func__, rx_ppio_id, tc, qid, shadow_q_size);
			/* Drop all following packets, and also this packet */
			for (j = i; j < num; j++) {
				struct pp2_buff_inf binf;

				binf.cookie = app_get_high_addr() | (uintptr_t)pp2_ppio_inq_desc_get_cookie(&descs[j]);
				binf.addr = pp2_ppio_inq_desc_get_phys_addr(&descs[j]);
				pp2_bpool_put_buff(pp2_args->hif, bpool, &binf);
			}
			/* Rollback write_index by 1 */
			if (shadow_q->write_ind > 0)
				shadow_q->write_ind--;
			else
				shadow_q->write_ind = shadow_q_size - 1;
			/* Update num of packets that may be handled */
			num = i;
			break;
		}


	}

	shadow_q->read_ind = free_buffers(&pp2_args->lcl_ports_desc[rx_ppio_id], &pp2_args->lcl_ports_desc[rx_ppio_id],
					  pp2_args->hif, shadow_q->read_ind, num, tc);

	return 0;
}

static int loop_tx(struct local_arg	*larg,
		   u8			 tx_ppio_id,
		   u8			 tc,
		   u16			 num)
{
	struct pp2_ppio_desc	 descs[PKT_GEN_APP_MAX_BURST_SIZE];
	struct pp2_lcl_common_args *pp2_args = (struct pp2_lcl_common_args *)larg->cmn_args.plat;
	struct buffer_desc	*buf_dec;
	u16			 i, tx_num;

#ifdef PKT_GEN_APP_HW_TX_CHKSUM_CALC
	enum pp2_inq_l3_type     l3_type = PP2_INQ_L3_TYPE_IPV4_OK;
	enum pp2_inq_l4_type     l4_type = PP2_INQ_L4_TYPE_UDP;
	u8			 l3_offset = sizeof(struct ether_header), l4_offset = (l3_offset + sizeof(struct ip));
#endif /* PKT_GEN_APP_HW_TX_CHKSUM_CALC */

	for (i = 0; i < num; i++) {
		buf_dec = &larg->buf_dec[larg->curr_frm];
		if (++larg->curr_frm == PKT_GEN_APP_BUFF_POOL_SIZE)
			larg->curr_frm = 0;

		if (unlikely(larg->latency)) {
			u64		*dat;

			dat = (u64 *)((struct pkt *)buf_dec->virt_addr)->body;

			if (i == (num >> 1)) {
				struct timeval	 curr_time;

				gettimeofday(&curr_time, NULL);
				dat[0] = (u64)curr_time.tv_sec;
				dat[1] = (u64)curr_time.tv_usec;
			} else
				dat[0] = 0;
		}

		pp2_ppio_outq_desc_reset(&descs[i]);
#ifdef PKT_GEN_APP_HW_TX_CHKSUM_CALC
#if (PKT_GEN_APP_HW_TX_IPV4_CHKSUM_CALC || PKT_GEN_APP_HW_TX_L4_CHKSUM_CALC)
		pp2_ppio_outq_desc_set_proto_info(&descs[i], pp2_l3_type_inq_to_outq(l3_type),
						  pp2_l4_type_inq_to_outq(l4_type), l3_offset,
						  l4_offset, PKT_GEN_APP_HW_TX_IPV4_CHKSUM_CALC,
						  PKT_GEN_APP_HW_TX_L4_CHKSUM_CALC);
#endif /* (PKT_GEN_APP_HW_TX_IPV4_CHKSUM_CALC ||  ... */
#endif /* PKT_GEN_APP_HW_TX_CHKSUM_CALC */
		pp2_ppio_outq_desc_set_phys_addr(&descs[i], buf_dec->phy_addr);
		pp2_ppio_outq_desc_set_pkt_offset(&descs[i], 0);
		pp2_ppio_outq_desc_set_pkt_len(&descs[i], buf_dec->size);

		if (unlikely(larg->cmn_args.verbose > 1)) {
			printf("Sending packet (va:%p, pa 0x%08x, len %d):\n",
			       buf_dec->virt_addr,
			       (unsigned int)buf_dec->phy_addr,
			       buf_dec->size);
			mem_disp(buf_dec->virt_addr, buf_dec->size);
		}
	}

	/* TX does not use BM buffers, therefore there is no issue to "drop" packets due to tx_queue_full */
	if (num) {
		tx_num = num;

		pp2_ppio_send(pp2_args->lcl_ports_desc[tx_ppio_id].ppio, pp2_args->hif, tc, descs, &tx_num);
		if (unlikely(larg->cmn_args.verbose && tx_num))
			printf("sent %d pkts on ppio %d, tc %d\n", tx_num, tx_ppio_id, tc);

		larg->trf_cntrs.tx_pkts += tx_num;
		for (i = 0; i < tx_num; i++)
			larg->trf_cntrs.tx_bytes += (PP2_OUTQ_DESC_PKT_LEN(&descs[i]) + ETH_FCS_LEN + ETH_IPG_LEN);
		larg->trf_cntrs.tx_drop += (num - tx_num);
	}

	if (unlikely(larg->cmn_args.busy_wait))
		udelay(larg->cmn_args.busy_wait);

	return 0;
}

static int main_loop_cb(void *arg, int *running)
{
	struct local_arg		*larg = (struct local_arg *)arg;
	struct pp2_glb_common_args	*pp2_args = (struct pp2_glb_common_args *)larg->cmn_args.garg->cmn_args.plat;
	int				err;
	u16				num;
	u8				port_index, tc = 0, qid = 0;

	if (!larg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	port_index = 0;

	while (*running) {
		if (!larg->cmn_args.garg->running) {
			if (larg->latency)
				dump_latency(larg);
			*running = 0;
			return 0;
		}

		num = larg->cmn_args.burst;
		if ((larg->cmn_args.id < larg->cmn_args.garg->max_rx_cpu) &&
			(pp2_args->ports_desc[port_index].traffic_dir & PKT_GEN_APP_DIR_RX)) {
			/* Find next queue to consume */
			do {
				qid++;
				if (qid == mvapp_pp2_max_num_qs_per_tc) {
					qid = 0;
					tc++;
					if (tc == PKT_GEN_APP_MAX_NUM_TCS_PER_PORT)
						tc = 0;
				}
			} while (!(larg->cmn_args.qs_map & (1 << ((tc * mvapp_pp2_max_num_qs_per_tc) + qid))));
			err = loop_rx(larg, port_index, tc, qid, num);
			if (unlikely(err))
				return err;
		}

		if ((larg->cmn_args.id >= larg->cmn_args.garg->min_tx_cpu) &&
			(pp2_args->ports_desc[port_index].traffic_dir & PKT_GEN_APP_DIR_TX)) {
			if (unlikely(larg->total_frm_cnt)) {
				if (unlikely(larg->trf_cntrs.tx_pkts >= larg->total_frm_cnt))
					num = 0;
				else if (unlikely((larg->total_frm_cnt - larg->trf_cntrs.tx_pkts) < num))
					num = larg->total_frm_cnt - larg->trf_cntrs.tx_pkts;
			}
			if (likely(num)) {
				err = loop_tx(larg, port_index, 0, num);
				if (unlikely(err))
					return err;
			}
		}
		port_index++;
		if (port_index >= larg->cmn_args.num_ports)
			port_index = 0;
	}

	return 0;
}

static int ctrl_loop_cb(void *arg)
{
	struct glob_arg	*garg = (struct glob_arg *)arg;

	if (!garg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	if (maintain_cnts(garg) > 0)
		return 0;

	if (!garg->cmn_args.cli)
		maintain_stats(garg);

	return 0;
}

static int init_all_modules(void)
{
	struct pp2_init_params	 pp2_params;
	struct pp2_glb_common_args *pp2_args = (struct pp2_glb_common_args *)garg.cmn_args.plat;

	int			 err;
	char			 file[PP2_MAX_BUF_STR_LEN];
	int			 num_rss_tables = 0;

	pr_info("Global initializations ...\n");

	err = mv_sys_dma_mem_init(PKT_GEN_APP_DMA_MEM_SIZE);
	if (err)
		return err;

	memset(&pp2_params, 0, sizeof(pp2_params));

	if (garg.cmn_args.cpus > 1) {
		num_rss_tables = app_rss_num_tbl_get(pp2_args->ports_desc[0].name, file);
		if (num_rss_tables < 0)
			return -EFAULT;
	}

	pp2_params.rss_tbl_reserved_map = (1 << num_rss_tables) - 1;
	pp2_params.res_maps_auto_detect_map = PP2_RSRVD_MAP_HIF_AUTO | PP2_RSRVD_MAP_BM_POOL_AUTO;

	err = pp2_init(&pp2_params);
	if (err)
		return err;

	/* Must be after pp2_init */
	app_used_hifmap_init(pp2_params.hif_reserved_map);
	app_used_bm_pool_map_init(pp2_params.bm_pool_reserved_map);

	pr_info("done\n");
	return 0;
}

static int init_local_modules(struct glob_arg *garg)
{
	int				err, port_index;
	struct bpool_inf		std_infs[] = PKT_GEN_APP_BPOOLS_INF;
	struct bpool_inf		jumbo_infs[] = PKT_GEN_APP_BPOOLS_JUMBO_INF;
	struct bpool_inf		*infs;
	struct pp2_glb_common_args *pp2_args = (struct pp2_glb_common_args *)garg->cmn_args.plat;
	int				i;

	pr_info("Local initializations ...\n");

	err = app_build_common_hifs(&garg->cmn_args, PKT_GEN_APP_HIF_Q_SIZE);
	if (err)
		return err;

	if (garg->cmn_args.mtu > DEFAULT_MTU) {
		infs = jumbo_infs;
		pp2_args->num_pools = ARRAY_SIZE(jumbo_infs);
	} else {
		infs = std_infs;
		pp2_args->num_pools = ARRAY_SIZE(std_infs);
	}

	err = app_build_all_bpools(&pp2_args->pools_desc, pp2_args->num_pools, infs, pp2_args->hif);
	if (err)
		return err;

	for (port_index = 0; port_index < garg->cmn_args.num_ports; port_index++) {
		struct port_desc *port = &pp2_args->ports_desc[port_index];

		err = app_find_port_info(port);
		if (!err) {
			port->ppio_type	= PP2_PPIO_T_NIC;
			port->num_tcs	= PKT_GEN_APP_MAX_NUM_TCS_PER_PORT;
			for (i = 0; i < port->num_tcs; i++)
				port->num_inqs[i] =  garg->max_rx_cpu;
			port->inq_size	= garg->rxq_size;
			port->num_outqs	= PKT_GEN_APP_MAX_NUM_TCS_PER_PORT;
			port->outq_size	= PKT_GEN_APP_TX_Q_SIZE;
			port->first_inq	= PKT_GEN_APP_FIRST_INQ;
			if (port->num_inqs[0] == 1)
				port->hash_type = PP2_PPIO_HASH_T_NONE;
			else
				port->hash_type = PP2_PPIO_HASH_T_5_TUPLE;

#ifdef PKT_GEN_APP_USE_HW_RATE_LMT
			if (garg->cmn_args.busy_wait) {
				port->port_params.rate_limit.rate_limit_enable = 1;
				port->port_params.rate_limit.rate_limit_params.cbs = PKT_GEN_APP_RATE_LMT_CBS;
				port->port_params.rate_limit.rate_limit_params.cir = garg->cmn_args.busy_wait;
				garg->cmn_args.busy_wait = 0;
			}
#endif /* PKT_GEN_APP_USE_HW_RATE_LMT */

			/* pkt_offset=0 not to be changed, before recoding rx_data_path to use pkt_offset as well */
			err = app_port_init(port, pp2_args->num_pools, pp2_args->pools_desc[port->pp_id],
					    garg->cmn_args.mtu, 0);
			if (err) {
				pr_err("Failed to initialize port %d (pp_id: %d)\n", port_index,
				       port->pp_id);
				return err;
			}
			/* Put the port in promisc so it will be able to get all frames received */
			err = pp2_ppio_set_promisc(port->ppio, 1);
			if (err) {
				pr_err("Failed to enter promisc port %d (pp_id: %d)\n", port_index,
				       port->pp_id);
				return err;
			}

			if (garg->loopback) {
				err = pp2_ppio_set_loopback(port->ppio, 1);
				if (err) {
					pr_err("Failed to enter promisc port %d (pp_id: %d)\n", port_index,
					       port->pp_id);
					return err;
				}
			} else {
				/* Wait for couple of seconds for the link to be up; after that, abort */
				int timeout = PKT_GEN_APP_LINK_UP_THR;

				do {
					err = pp2_ppio_get_link_state(port->ppio, &i);
					if (err) {
						pr_err("Link error (port: %d, pp_id: %d)\n", port_index,
						       port->pp_id);
						return -EFAULT;
					}
					udelay(1000);
				} while (!i && --timeout);
				if (!i) {
					pr_err("Link is down (port: %d, pp_id: %d)\n", port_index,
					       port->pp_id);
					return -EFAULT;
				}
			}
		} else {
			return err;
		}
	}

	pr_info("done\n");
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

	return dump_perf(arg);
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
	cmd_params.name		= "stat";
	cmd_params.desc		= "Show app statistics";
	cmd_params.format	= "<reset>";
	cmd_params.cmd_arg	= garg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))apps_pp2_stat_cmd_cb;
	mvapp_register_cli_cmd(&cmd_params);
	app_register_cli_common_cmds(&garg->cmn_args);

	/* statistics command */
	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "perf";
	cmd_params.desc		= "Dump performance statistics";
	cmd_params.format	= NULL;
	cmd_params.cmd_arg	= garg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))perf_cmd_cb;
	mvapp_register_cli_cmd(&cmd_params);

	return 0;
}

static int unregister_cli_cmds(struct glob_arg *garg)
{
	/* TODO: unregister cli cmds */
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

	garg->running = 1;

	return 0;
}

static int init_local(void *arg, int id, void **_larg)
{
	struct glob_arg		*garg = (struct glob_arg *)arg;
	struct local_arg	*larg;
	struct pp2_glb_common_args *glb_pp2_args = (struct pp2_glb_common_args *)garg->cmn_args.plat;
	struct pp2_lcl_common_args *lcl_pp2_args;
	int			 i, err;

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
	lcl_pp2_args = (struct pp2_lcl_common_args *)larg->cmn_args.plat;
	larg->cmn_args.id		= id;
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
	err = app_hif_init_wrap(id, &garg->cmn_args.thread_lock, glb_pp2_args, lcl_pp2_args,
				PKT_GEN_APP_HIF_Q_SIZE, NULL);
	if (err)
		return err;

	larg->cmn_args.id               = id;
	larg->cmn_args.burst		= garg->cmn_args.burst;
	larg->cmn_args.busy_wait	= garg->cmn_args.busy_wait;
	if (garg->total_frm_cnt) {
		/* split the frame count evenly among all cores */
		larg->total_frm_cnt	=
			garg->total_frm_cnt / (garg->cmn_args.cpus - garg->min_tx_cpu);
		/* let the first core send the residual frames */
		if (larg->cmn_args.id == 0)
			larg->total_frm_cnt +=
				garg->total_frm_cnt % (garg->cmn_args.cpus - garg->min_tx_cpu);
	}
	larg->cmn_args.echo              = garg->cmn_args.echo;
	larg->cmn_args.prefetch_shift	= garg->cmn_args.prefetch_shift;
	larg->cmn_args.num_ports         = garg->cmn_args.num_ports;
	lcl_pp2_args->lcl_ports_desc = (struct lcl_port_desc *)
					malloc(larg->cmn_args.num_ports * sizeof(struct lcl_port_desc));
	for (i = 0; i < larg->cmn_args.num_ports; i++)
		app_port_local_init(i, larg->cmn_args.id, &lcl_pp2_args->lcl_ports_desc[i],
				    &glb_pp2_args->ports_desc[i]);

	lcl_pp2_args->pools_desc	= glb_pp2_args->pools_desc;
	larg->cmn_args.garg             = garg;
	larg->pkt_size			= garg->pkt_size;
	larg->cmn_args.verbose		= garg->cmn_args.verbose;
	larg->latency			= garg->latency;

	larg->cmn_args.qs_map = garg->cmn_args.qs_map << (garg->cmn_args.qs_map_shift * id);
	lcl_pp2_args->multi_buffer_release = glb_pp2_args->multi_buffer_release;

	garg->cmn_args.largs[id] = larg;

	for (i = 0; i < larg->cmn_args.num_ports; i++)
		glb_pp2_args->ports_desc[i].lcl_ports_desc[id] = &lcl_pp2_args->lcl_ports_desc[i];
	pr_debug("thread %d (cpu %d) mapped to Qs %llx\n",
		 larg->cmn_args.id, sched_getcpu(), (unsigned long long)larg->cmn_args.qs_map);

	err = app_build_pkt_pool(&larg->buffer,
			larg->buf_dec,
			PKT_GEN_APP_BUFF_POOL_SIZE,
			MIN_PKT_SIZE,
			MAX_PKT_SIZE,
			larg->pkt_size,
			&larg->cmn_args.garg->src_ip,
			&larg->cmn_args.garg->dst_ip,
			larg->cmn_args.garg->src_mac,
			larg->cmn_args.garg->dst_mac);
	if (err)
		return err;

	*_larg = larg;

	return 0;
}

static void usage(char *progname)
{
	printf("\n"
	       "MUSDK packet-gen application.\n"
	       "\n"
	       "Usage: %s OPTIONS\n"
	       "  E.g. %s -i eth0,eth1 -c 1\n"
	       "\n"
	       "Mandatory OPTIONS:\n"
	       "\t-i, --interface <Eth-interface>\n"
	       "Optional OPTIONS:\n"
	       "\t-r, --rx                    enables port Rx mode (default: disabled)\n"
	       "\t-t, --tx                    enables port Tx (traffic generator) mode (default: disabled)\n"
	       "\tUsage: --rx/--tx options can be used per specific interface or for all interfaces:\n"
	       "\t\t%s -i eth0 --rx -i eth1 --tx\t[opens port eth0 in Rx mode and eth1 in Tx mode]\n"
	       "\t\t%s -i eth0,eth1 --rx --tx\t[opens eth0 and eth1 ports in Rx and Tx mode]\n"
	       "OPTIONS for Tx mode:\n"
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
	       "\t-L, --latency               Run latency measurments\n"
	       "\t--loopback                  set port loopback; may be useful for testing.\n"
#ifdef PKT_GEN_APP_VERBOSE_DEBUG
	       "\t-v, --verbose               Increase verbose debug (default is 0).\n"
	       "\t                            With every '-v', the debug is increased by one.\n"
	       "\t                            0 - none, 1 - pkts sent/recv indication, 2 - full pkt dump\n"
#endif /* PKT_GEN_APP_VERBOSE_DEBUG */
	       "\t--cli                       Use CLI\n"
	       "\t-h, --help                  Display help and exit\n\n"
	       "\n", MVAPPS_NO_PATH(progname), MVAPPS_NO_PATH(progname), MVAPPS_NO_PATH(progname),
	       MVAPPS_NO_PATH(progname), PKT_GEN_APP_DFLT_BURST_SIZE, MAX_PKT_SIZE, DEFAULT_REPORT_TIME
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
	struct pp2_glb_common_args *pp2_args = (struct pp2_glb_common_args *)garg->cmn_args.plat;
	char *token;
	int i = 1;
	int option;
	int long_index = 0;
	int rv, curr_port_index = 0;
	int port_dir[MVAPPS_PP2_MAX_I_OPTION_PORTS] = {0};
	int common_dir = 0;
	int mult;
	const char short_options[] = "hrtvLi:b:l:c:a:m:T:w:R:C:S:D:s:d:";
	struct option long_options[] = {
		{"help", no_argument, 0, 'h'},
		{"interface", required_argument, 0, 'i'},
		{"rx", no_argument, 0, 'r'},
		{"tx", no_argument, 0, 't'},
		{"latency", no_argument, 0, 'L'},
		{"burst", required_argument, 0, 'b'},
		{"size", required_argument, 0, 'l'},
		{"cores", required_argument, 0, 'c'},
		{"affinity", required_argument, 0, 'a'},
		{"qmap", required_argument, 0, 'm'},
		{"report-time", required_argument, 0, 'T'},
		{"rate-limit", required_argument, 0, 'R'},
		{"count", required_argument, 0, 'C'},
		{"src-mac", required_argument, 0, 'S'},
		{"dst-mac", required_argument, 0, 'D'},
		{"src-ip", required_argument, 0, 's'},
		{"dst-ip", required_argument, 0, 'd'},
		{"verbose", no_argument, 0, 'v'},
		{"cli", no_argument, &garg->cmn_args.cli, 1},
		{"loopback", no_argument, &garg->loopback, 1},
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
	garg->cmn_args.qs_map = 0;
	garg->cmn_args.qs_map_shift = 0;
	garg->cmn_args.prefetch_shift = PKT_GEN_APP_PREFETCH_SHIFT;
	garg->cmn_args.ctrl_thresh = PKT_GEN_APP_STATS_DFLT_THR;
	garg->cmn_args.pkt_offset = 0;
	garg->cmn_args.busy_wait	= DEFAULT_RATE_USECS;
	garg->rxq_size = PKT_GEN_APP_RX_Q_SIZE;
	garg->maintain_stats = 0;
	garg->report_time = DEFAULT_REPORT_TIME;
	garg->pkt_size = MAX_PKT_SIZE;
	garg->src_ip.start = garg->src_ip.end = garg->src_ip.curr = DEFAULT_SRC_IP;
	garg->src_ip.port0 = garg->src_ip.port1 = garg->src_ip.port_curr = DEFAULT_SRC_PORT;
	garg->dst_ip.start = garg->dst_ip.end = garg->dst_ip.curr = DEFAULT_DST_IP;
	garg->dst_ip.port0 = garg->dst_ip.port1 = garg->dst_ip.port_curr = DEFAULT_DST_PORT;
	memcpy(garg->dst_mac, default_dst_mac, MV_ETH_ALEN);
	memcpy(garg->src_mac, default_src_mac, MV_ETH_ALEN);

	pp2_args->multi_buffer_release = 1;


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
		case 'i':
			/* count the number of tokens separated by ',' */
			for (token = strtok(optarg, ","), garg->cmn_args.num_ports = 0;
			     token;
			     token = strtok(NULL, ","), garg->cmn_args.num_ports++)
				snprintf(pp2_args->ports_desc[garg->cmn_args.num_ports].name,
					 sizeof(pp2_args->ports_desc[garg->cmn_args.num_ports].name),
					 "%s", token);
			break;
		case 'r':
			pr_debug("Set RX mode for port %s\n", pp2_args->ports_desc[curr_port_index].name);
			port_dir[curr_port_index] |= PKT_GEN_APP_DIR_RX;
			common_dir |= PKT_GEN_APP_DIR_RX;
			garg->rx = 1;
			break;
		case 't':
			pr_debug("Set TX mode for port %s\n", pp2_args->ports_desc[curr_port_index].name);
			port_dir[curr_port_index] |= PKT_GEN_APP_DIR_TX;
			common_dir |= PKT_GEN_APP_DIR_TX;
			garg->tx = 1;
			break;
		case 'L':
			pr_debug("Set Latency check\n");
			garg->latency = 1;
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
		case 'm':
			rv = sscanf(optarg, "%x:%x", (unsigned int *)&garg->cmn_args.qs_map,
				    &garg->cmn_args.qs_map_shift);
			if (rv != 2) {
				pr_err("Failed to parse -m parameter\n");
				return -EINVAL;
			}
			pr_debug("Set qs_map=0x%.16lx, qs_map_shift=%d\n", garg->cmn_args.qs_map,
				garg->cmn_args.qs_map_shift);
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
	/* Now, check validity of all inputs */
	if (!garg->cmn_args.num_ports ||
	    !pp2_args->ports_desc[0].name) {
		pr_err("No port defined!\n");
		return -EINVAL;
	}
	if (garg->cmn_args.num_ports > MVAPPS_PP2_MAX_NUM_PORTS) {
		pr_err("too many ports specified (%d vs %d)\n",
		       garg->cmn_args.num_ports, MVAPPS_PP2_MAX_NUM_PORTS);
		return -EINVAL;
	}
	/* Update traffic direction for all ports */
	for (i = 0; i < garg->cmn_args.num_ports; i++) {
		if (port_dir[i])
			pp2_args->ports_desc[i].traffic_dir = port_dir[i];
		else if (common_dir)
			pp2_args->ports_desc[i].traffic_dir = common_dir;
		else {
			pp2_args->ports_desc[i].traffic_dir = PKT_GEN_APP_DIR_RX;
			garg->rx = 1;
		}
	}

	if (garg->cmn_args.burst > PKT_GEN_APP_MAX_BURST_SIZE) {
		pr_err("illegal burst size requested (%d vs %d)!\n",
		       garg->cmn_args.burst, PKT_GEN_APP_MAX_BURST_SIZE);
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
	    (PKT_GEN_APP_MAX_NUM_TCS_PER_PORT == 1)) {
		pr_warn("no point in queues-mapping; ignoring.\n");
		garg->cmn_args.qs_map = 1;
		garg->cmn_args.qs_map_shift = 1;
	} else if (!garg->cmn_args.qs_map) {
		garg->cmn_args.qs_map = 1;
		garg->cmn_args.qs_map_shift = PKT_GEN_APP_MAX_NUM_TCS_PER_PORT;
	}

	if ((garg->cmn_args.cpus != 1) &&
	    (garg->cmn_args.qs_map & (garg->cmn_args.qs_map << garg->cmn_args.qs_map_shift))) {
		pr_err("Invalid queues-mapping (overlapping CPUs)!\n");
		return -EINVAL;
	}

	garg->max_rx_cpu = garg->cmn_args.cpus / 2;
	if (garg->cmn_args.cpus % 2)
		garg->max_rx_cpu++;
	garg->min_tx_cpu = 0;
	if (garg->cmn_args.cpus > 1) {
		garg->min_tx_cpu = garg->cmn_args.cpus - garg->max_rx_cpu;
		if (garg->cmn_args.cpus % 2)
			garg->min_tx_cpu++;
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

	if (!garg->rx && !garg->tx) {
		pr_err("Trying to run with neither RX nor TX!\n");
		return -EINVAL;
	}

	if (garg->latency) {
		if (!(garg->rx && garg->tx)) {
			pr_err("Trying to run latency test neither RX or TX!\n");
			return -EINVAL;
		}
		garg->total_frm_cnt = PKT_GEN_APP_LTNC_FRM_CNT;
		garg->cmn_args.burst = PKT_GEN_APP_LTNC_BURST;
	}

#ifndef PKT_GEN_APP_USE_HW_RATE_LMT
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
#endif /* !PKT_GEN_APP_USE_HW_RATE_LMT */

	return 0;
}

int main(int argc, char *argv[])
{
	struct mvapp_params	mvapp_params;
	u64			cores_mask;
	struct pp2_glb_common_args *pp2_args;
	int			err;

	setbuf(stdout, NULL);
	app_set_max_num_qs_per_tc();

	pr_info("pkt-gen is started\n");
	pr_debug("pr_debug is enabled\n");

	garg.cmn_args.plat = (struct pp2_glb_common_args *)malloc(sizeof(struct pp2_glb_common_args));
	if (!garg.cmn_args.plat) {
		pr_err("No mem for global plat arg obj!\n");
		return -ENOMEM;
	}
	pp2_args = (struct pp2_glb_common_args *)garg.cmn_args.plat;
	err = parse_args(&garg, argc, argv);
	if (err) {
		free(garg.cmn_args.plat);
		return err;
	}

	pp2_args->pp2_num_inst = pp2_get_num_inst();

	cores_mask = apps_cores_mask_create(garg.cmn_args.cpus, garg.cmn_args.affinity);
	garg.cmn_args.cores_mask = cores_mask;

	memset(&mvapp_params, 0, sizeof(mvapp_params));
	mvapp_params.use_cli		= garg.cmn_args.cli;
	mvapp_params.num_cores		= garg.cmn_args.cpus;
	mvapp_params.cores_mask		= cores_mask;
	mvapp_params.global_arg		= (void *)&garg;
	mvapp_params.init_global_cb	= init_global;
	mvapp_params.deinit_global_cb	= apps_pp2_deinit_global;
	mvapp_params.init_local_cb	= init_local;
	mvapp_params.deinit_local_cb	= apps_pp2_deinit_local;
	mvapp_params.main_loop_cb	= main_loop_cb;
	mvapp_params.ctrl_cb		= ctrl_loop_cb;

	return mvapp_go(&mvapp_params);
}
