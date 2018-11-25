/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#define _GNU_SOURCE         /* See feature_test_macros(7) */
#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <sched.h>
#include <fcntl.h>
#include <arpa/inet.h>	/* ntohs */
#include <netinet/if_ether.h>
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
#include "mv_giu_gpio.h"
#include "mng/mv_nmp.h"
#include "mng/mv_nmp_guest.h"

#include "mvapp.h"

#include "utils.h"
#include "nmp_guest_utils.h"
#include "giu_utils.h"


#define PKT_GEN_APP_VERBOSE_DEBUG

#define PKT_GEN_APP_GIU_BP_SIZE			2048

/* as GIU is the bottleneck, set the burst size to GIU_Q_SIZE / 4 */
#define PKT_GEN_APP_DFLT_BURST_SIZE		128
#define PKT_GEN_APP_MAX_BURST_SIZE		256

#define PKT_GEN_APP_BUFF_POOL_SIZE		8192

#define PKT_GEN_APP_DMA_MEM_SIZE		(80 * 1024 * 1024)
#define PKT_GEN_APP_STATS_DFLT_THR		1000

#define PKT_GEN_APP_MAX_NUM_TCS_PER_PORT	1

#define PKT_GEN_APP_DIR_RX			0x1
#define PKT_GEN_APP_DIR_TX			0x2

#define PKT_GEN_APP_MAX_TOTAL_FRM_CNT		UINT32_MAX

#define PKT_GEN_APP_PREFETCH_SHIFT		4

#define PKT_GEN_NUM_CNTS	2
#define PKT_GEN_CNT_PKTS	0
#define PKT_GEN_CNT_BYTES	1

/* NMP Guest ID */
#define PKT_ECHO_APP_NMP_GUEST_ID	2
/* NMP Guest Timeout (ms)*/
#define PKT_ECHO_APP_NMP_GUEST_TIMEOUT	1000

/* The following macro is need in order to extract pkt length out from
 * the giu outQ descriptor; see giu_gpio_outq_desc_set_pkt_len() in mv_giu_gpio.h
 */
#define GIU_OUTQ_DESC_PKT_LEN(d)	((d)->cmds[1] >> 16)

#define MAX_BODYSIZE			(DEFAULT_MTU - IPV4_HDR_LEN - UDP_HDR_LEN)
#define MIN_PKT_SIZE			(60)
#define MAX_PKT_SIZE			(DEFAULT_MTU + ETH_HLEN)

#define DEFAULT_REPORT_TIME 1 /* 1 second */
#define DEFAULT_RATE_USECS  0
#define DEFAULT_SRC_IP 0x0a000001
#define DEFAULT_DST_IP 0x0a000002
#define DEFAULT_SRC_PORT 1024
#define DEFAULT_DST_PORT 1024

#define PLD_WATERMARK	0xcafecafe


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

	int			loopback;
	int			maintain_stats;
	pthread_mutex_t		trd_lock;

	int			rx;
	int			tx;
	u32			total_frm_cnt;

	struct ip_range		src_ip;
	struct ip_range		dst_ip;
	eth_addr_t		dst_mac;
	eth_addr_t		src_mac;
	int			pkt_size;
	u32			report_time;

	struct giu_bpools_desc	 giu_bpools_desc;
	struct giu_port_desc	 giu_port_desc;
	struct nmp		*nmp;
	struct nmp_guest	*nmp_guest;
	struct nmp_guest_info	guest_info;
	char			*prb_str;
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

	u32				traffic_dir;	/* Traffic direction (1 - Rx, 2 - Tx, 3 - Rx+Tx) */
	u32				total_frm_cnt;
	u16				curr_frm;

	struct buffer_desc		buf_dec[PKT_GEN_APP_BUFF_POOL_SIZE];

	struct traffic_counters		trf_cntrs;

	void				*buffer;
	int				pkt_size;
	struct lcl_giu_port_desc	giu_ports_desc[MVAPPS_GIU_MAX_NUM_PORTS];
};


eth_addr_t default_src_mac = {0x00, 0x00, 0x00, 0x00, 0x00, 0x01};
eth_addr_t default_dst_mac = {0x00, 0x00, 0x00, 0x00, 0x00, 0x02};

/* globals for ingress/egress packet rate statistics */
u64			 lst_rx_cnts[PKT_GEN_NUM_CNTS][MVAPPS_PP2_MAX_NUM_PORTS];
u64			 lst_tx_cnts[PKT_GEN_NUM_CNTS][MVAPPS_PP2_MAX_NUM_PORTS];

static struct glob_arg garg = {};


static inline u16 giu_free_buffers(struct lcl_giu_port_desc	*rx_port,
				   u16				start_idx,
				   u16				num,
				   u8				tc)
{
	u16			i, free_cnt = 0, idx = start_idx;
	struct giu_bpool	*bpool;
	struct giu_buff_inf	*binf;
	struct giu_tx_shadow_q	*shadow_q;

	shadow_q = &rx_port->shadow_qs[tc];

	for (i = 0; i < num; i++) {
		bpool = shadow_q->ents[idx].bpool;
		binf = (struct giu_buff_inf *)&shadow_q->ents[idx].buff_inf;
		if (unlikely(!binf->cookie || !binf->addr || !bpool)) {
			pr_warn("Shadow memory @%d: cookie(%lx), pa(%lx), pool(%lx)!\n",
				i, (u64)binf->cookie, (u64)binf->addr, (u64)bpool);
			continue;
		}
		giu_bpool_put_buff(bpool, binf);
		free_cnt++;

		if (++idx == rx_port->shadow_q_size)
			idx = 0;
	}

	return idx;
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

		printf("TX: %" PRIu64 " Kpps, %" PRIu64 " Kbps, %" PRIu64 " Kdrops\t",
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

static int loop_rx(struct local_arg	*larg,
		   u8			 rx_ppio_id,
		   u8			 tc,
		   u8			 qid,
		   u16			 num)
{
	struct giu_tx_shadow_q		*shadow_q;
	struct lcl_giu_port_desc	*giu_port_desc = &(larg->giu_ports_desc[rx_ppio_id]);
	struct giu_gpio_desc		 descs[PKT_GEN_APP_MAX_BURST_SIZE];
#ifdef PKT_GEN_APP_VERBOSE_DEBUG
	struct pkt			*pkt;
	u32				*dat;
#endif /* PKT_GEN_APP_VERBOSE_DEBUG */
	u16				 i;
	int				 shadow_q_size;
	int				 prefetch_shift = larg->cmn_args.prefetch_shift;

	/* Use GIU Shadow Q to save allocated buffers */
	shadow_q = &giu_port_desc->shadow_qs[tc];
	shadow_q_size = giu_port_desc->shadow_q_size;

	giu_gpio_recv(giu_port_desc->gpio, tc, qid, descs, &num);
	if (unlikely(larg->cmn_args.verbose && num))
		printf("recv %d pkts on ppio %d, tc %d, qid %d\n", num, rx_ppio_id, tc, qid);

	larg->trf_cntrs.rx_pkts += num;

	for (i = 0; i < num; i++) {
		char *buff = (char *)(app_get_high_addr() | (uintptr_t)giu_gpio_inq_desc_get_cookie(&descs[i]));
		dma_addr_t pa = giu_gpio_inq_desc_get_phys_addr(&descs[i]);
		u16 len = giu_gpio_inq_desc_get_pkt_len(&descs[i]);
		void *bpool = giu_gpio_inq_desc_get_bpool(&descs[i], giu_port_desc->gpio);

#ifdef PKT_GEN_APP_VERBOSE_DEBUG
		pkt = (struct pkt *)buff;
		dat = (u32 *)pkt->body;
		if (unlikely((dat[0] == MVAPPS_PLD_WATERMARK)/* TODO: || (dat[0] != read_ind)*/))
			pr_info("\r[ERROR] Analayzer: Illegal PLD: %x\n", dat[0]);
		else if (unlikely((pkt->eh.ether_type == htons(ETHERTYPE_IP)) && (pkt->ip.ip_v == IPVERSION))) {
			u16 tot_len = swab16(pkt->ip.ip_len) + sizeof(pkt->eh); /* assuming eth header */

			if (unlikely(tot_len != len))
				pr_info("\r[ERROR] Analayzer: length mismatch (frame %u vs rx-desc %u @%d)!\n",
					tot_len, len, shadow_q->write_ind);
		}

		dat[0] = MVAPPS_PLD_WATERMARK;

		if (unlikely(larg->cmn_args.verbose > 1)) {
			char *tmp_buff;

			tmp_buff = buff;
			pr_debug("buff(%p)\n", tmp_buff);
			printf("Received packet (va:%p, pa 0x%08x, len %d):\n",
			       tmp_buff,
			       (unsigned int)pa,
			       len);
			mem_disp(tmp_buff, len);
		}
#endif /* PKT_GEN_APP_VERBOSE_DEBUG */

		if (num - i > prefetch_shift)
			prefetch((char *)(app_get_high_addr() |
					(uintptr_t)giu_gpio_inq_desc_get_cookie(&descs[i + prefetch_shift])));

		larg->trf_cntrs.rx_bytes += len;

		shadow_q->ents[shadow_q->write_ind].buff_inf.cookie = (uintptr_t)buff;
		shadow_q->ents[shadow_q->write_ind].buff_inf.addr = pa;
		shadow_q->ents[shadow_q->write_ind].bpool = bpool;

		shadow_q->write_ind++;
		if (shadow_q->write_ind == shadow_q_size)
			shadow_q->write_ind = 0;
	}

	if (num)
		shadow_q->read_ind = giu_free_buffers(giu_port_desc, shadow_q->read_ind, num, tc);

	return 0;
}

static int loop_tx(struct local_arg	*larg,
		   u8			 tx_ppio_id,
		   u8			 tc,
		   u16			 num)
{
	struct giu_gpio_desc		 descs[PKT_GEN_APP_MAX_BURST_SIZE];
	struct lcl_giu_port_desc	*giu_port_desc = &(larg->giu_ports_desc[tx_ppio_id]);
	struct buffer_desc		*buf_dec;
#ifdef PKT_GEN_APP_VERBOSE_DEBUG
	u32				*dat;
#endif /* PKT_GEN_APP_VERBOSE_DEBUG */
	enum giu_outq_l3_type		 l3_type = GIU_OUTQ_L3_TYPE_IPV4_OK;
	enum giu_outq_l4_type		 l4_type = GIU_OUTQ_L4_TYPE_UDP;
	u16				 i, tx_num;
	u8				 l3_offset =
		sizeof(struct ether_header), l4_offset = (l3_offset + sizeof(struct ip));

	for (i = 0; i < num; i++) {
		buf_dec = &larg->buf_dec[larg->curr_frm];

#ifdef PKT_GEN_APP_VERBOSE_DEBUG
		dat = (u32 *)((struct pkt *)buf_dec->virt_addr)->body;
		dat[0] = larg->curr_frm;
#endif /* PKT_GEN_APP_VERBOSE_DEBUG */

		if (++larg->curr_frm == PKT_GEN_APP_BUFF_POOL_SIZE)
			larg->curr_frm = 0;

		giu_gpio_outq_desc_reset(&descs[i]);
		giu_gpio_outq_desc_set_proto_info(&descs[i],
						GIU_DESC_ERR_OK,
						GIU_OUTQ_L2_UNICAST,
						GIU_VLAN_TAG_NONE,
						GIU_OUTQ_L3_UNICAST,
						l3_type,
						l3_offset,
						l4_type,
						l4_offset);
		giu_gpio_outq_desc_set_phys_addr(&descs[i], buf_dec->phy_addr);
		giu_gpio_outq_desc_set_pkt_offset(&descs[i], 0);
		giu_gpio_outq_desc_set_pkt_len(&descs[i], buf_dec->size);

#ifdef PKT_GEN_APP_VERBOSE_DEBUG
		if (unlikely(larg->cmn_args.verbose > 1)) {
			printf("Sending packet (va:%p, pa 0x%08x, len %d):\n",
			       buf_dec->virt_addr,
			       (unsigned int)buf_dec->phy_addr,
			       buf_dec->size);
			mem_disp(buf_dec->virt_addr, buf_dec->size);
		}
#endif /* PKT_GEN_APP_VERBOSE_DEBUG */
	}

	giu_gpio_get_num_outq_done(giu_port_desc->gpio, tc, 0, &tx_num);
	/* TX does not use BM buffers, therefore there is no issue to "drop" packets due to tx_queue_full */
	if (num) {
		tx_num = num;

		giu_gpio_send(giu_port_desc->gpio, tc, 0, descs, &tx_num);
		if (unlikely(larg->cmn_args.verbose && tx_num))
			printf("sent %d pkts on ppio %d, tc %d\n", tx_num, tx_ppio_id, tc);

		larg->trf_cntrs.tx_pkts += tx_num;
		for (i = 0; i < tx_num; i++)
			larg->trf_cntrs.tx_bytes += GIU_OUTQ_DESC_PKT_LEN(&descs[i]);
		larg->trf_cntrs.tx_drop += (num - tx_num);
	}

	if (unlikely(larg->cmn_args.busy_wait))
		udelay(larg->cmn_args.busy_wait);

	return 0;
}

static int main_loop_cb(void *arg, int *running)
{
	struct local_arg	*larg = (struct local_arg *)arg;
	int			err;
	u16			num;
	u8			port_index, tc = 0, qid = 0;

	if (!larg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	num = larg->cmn_args.burst;
	port_index = 0;
	while (*running) {
		/* Schedule GIE execution. schedule it only if we're running on
		 * single-core or in case this thread is the first core.
		 */
		if ((larg->cmn_args.garg->cmn_args.cpus == 1) ||
			(larg->cmn_args.id == 0)) {
			nmp_schedule(garg.nmp, NMP_SCHED_TX, NULL);
			nmp_schedule(garg.nmp, NMP_SCHED_RX, NULL);
			if (larg->cmn_args.garg->cmn_args.cpus != 1)
				continue;
		}

		if (larg->traffic_dir & PKT_GEN_APP_DIR_RX) {
			/* Find next queue to consume */
			do {
				qid++;
				if (qid == mvapp_giu_max_num_qs_per_tc) {
					qid = 0;
					tc++;
					if (tc == PKT_GEN_APP_MAX_NUM_TCS_PER_PORT)
						tc = 0;
				}
			} while (!(larg->cmn_args.qs_map & (1 << ((tc * mvapp_giu_max_num_qs_per_tc) + qid))));
			err = loop_rx(larg, port_index, tc, qid, num);
			if (err != 0)
				return err;
		}

		if (larg->traffic_dir & PKT_GEN_APP_DIR_TX) {
			if (larg->total_frm_cnt == PKT_GEN_APP_MAX_TOTAL_FRM_CNT)
				num = 0;
			else if (larg->total_frm_cnt) {
				if (larg->total_frm_cnt < num)
					num = larg->total_frm_cnt;
				larg->total_frm_cnt -= num;
				if (!larg->total_frm_cnt)
					larg->total_frm_cnt = PKT_GEN_APP_MAX_TOTAL_FRM_CNT;
			}
			if (num) {
				err = loop_tx(larg, port_index, 0, num);
				if (err != 0)
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
	struct glob_arg *garg = (struct glob_arg *)arg;

	if (!garg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	if (garg->prb_str)
		nmp_guest_schedule(garg->nmp_guest);

	if (!garg->cmn_args.cli)
		maintain_stats(garg);

	return 0;
}

static int init_all_modules(void)
{
	int		err;
	struct		nmp_params nmp_params;
	struct		nmp_guest_params nmp_guest_params;

	pr_info("Global initializations ...\n");

	err = mv_sys_dma_mem_init(PKT_GEN_APP_DMA_MEM_SIZE);
	if (err)
		return err;

	/* NMP initializations */
	memset(&nmp_params, 0, sizeof(nmp_params));
	err = app_read_nmp_cfg_file(garg.cmn_args.nmp_cfg_location, &nmp_params);
	if (err) {
		pr_err("NMP preinit failed with error %d\n", err);
		return -EIO;
	}
	err = nmp_init(&nmp_params, &(garg.nmp));
	if (err)
		return err;

	/* NMP Guest initializations */
	memset(&nmp_guest_params, 0, sizeof(nmp_guest_params));
	nmp_guest_params.id = garg.cmn_args.guest_id;
	nmp_guest_params.timeout = PKT_ECHO_APP_NMP_GUEST_TIMEOUT;
	nmp_guest_params.nmp =  (void *)garg.nmp;
	err = nmp_guest_init(&nmp_guest_params, &garg.nmp_guest);
	if (err)
		return err;

	nmp_guest_get_probe_str(garg.nmp_guest, &garg.prb_str);

	pr_info("done\n");
	return 0;
}

static int init_local_modules(struct glob_arg *garg)
{
	int			err;

	pr_info("Local initializations ...\n");

	nmp_guest_get_relations_info(garg->nmp_guest, &garg->guest_info);

	err = app_guest_utils_build_all_giu_bpools(garg->prb_str,
			&garg->guest_info,
			&garg->giu_bpools_desc,
			PKT_GEN_APP_GIU_BP_SIZE);
	if (err)
		return err;

	err = app_nmp_guest_giu_port_init(garg->prb_str,
			&garg->guest_info,
			&garg->giu_port_desc);
	if (err)
		return err;

	garg->cmn_args.num_ports = garg->guest_info.ports_info.num_ports;

	pr_info("done\n");
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

	return 0;
}

static int init_local(void *arg, int id, void **_larg)
{
	struct glob_arg		*garg = (struct glob_arg *)arg;
	struct local_arg	*larg;
	int			giu_port_id = 0;
	int			giu_id = 0; /* TODO: this should not be hard-coded */
	int			err;

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

	larg->cmn_args.id		= id;
	larg->cmn_args.num_ports	= garg->cmn_args.num_ports;

	larg->cmn_args.id               = id;
	larg->cmn_args.burst		= garg->cmn_args.burst;
	larg->cmn_args.busy_wait	= garg->cmn_args.busy_wait;
	if (garg->total_frm_cnt) {
		/* split the frame count evenly among all cores */
		larg->total_frm_cnt		= garg->total_frm_cnt/garg->cmn_args.cpus;
		/* let the first core send the residual frames */
		if (larg->cmn_args.id == 0)
			larg->total_frm_cnt += garg->total_frm_cnt%garg->cmn_args.cpus;
	}
	larg->cmn_args.echo             = garg->cmn_args.echo;
	larg->cmn_args.prefetch_shift	= garg->cmn_args.prefetch_shift;
	larg->cmn_args.num_ports        = garg->cmn_args.num_ports;
	larg->cmn_args.garg             = garg;
	larg->pkt_size			= garg->pkt_size;
	larg->cmn_args.verbose		= garg->cmn_args.verbose;

	larg->cmn_args.qs_map = garg->cmn_args.qs_map << (garg->cmn_args.qs_map_shift * id);

	garg->cmn_args.largs[id] = larg;

	if (garg->rx)
		larg->traffic_dir |= PKT_GEN_APP_DIR_RX;
	if (garg->tx)
		larg->traffic_dir |= PKT_GEN_APP_DIR_TX;

	pr_debug("thread %d (cpu %d) mapped to Qs %llx\n",
		 larg->cmn_args.id, sched_getcpu(), (unsigned long long)larg->cmn_args.qs_map);

	app_giu_port_local_init(giu_port_id,
		larg->cmn_args.id,
		giu_id,
		&larg->giu_ports_desc[giu_id],
		garg->giu_port_desc.gpio);

	*_larg = larg;

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

	return 0;
}

static void usage(char *progname)
{
	printf("\n"
	       "MUSDK GIU packet-gen application.\n"
	       "\n"
	       "Usage: %s OPTIONS\n"
	       "  E.g. %s -i eth0,eth1 -c 1\n"
	       "\n"
	       "Mandatory OPTIONS:\n"
	       "\t-i, --interface <Eth-interface>\n"
	       "Optional OPTIONS:\n"
	       "\t-r, --rx                    enables port Rx mode (default: enabled)\n"
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
	       "\t-f, --file                  Location and name of the nmp-config file to load\n"
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
	int option;
	int long_index = 0;
	int rv;
	int mult;
	const char short_options[] = "hi:b:l:c:a:m:T:w:R:C:S:D:s:d:f:rtv";
	struct option long_options[] = {
		{"help", no_argument, 0, 'h'},
		{"rx", no_argument, 0, 'r'},
		{"tx", no_argument, 0, 't'},
		{"file", required_argument, 0, 'f'},
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
	garg->maintain_stats = 0;
	garg->cmn_args.guest_id = PKT_ECHO_APP_NMP_GUEST_ID;
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
		case 'f':
			strcpy(garg->cmn_args.nmp_cfg_location, optarg);
			break;
		case 'r':
			pr_debug("Set RX mode\n");
			garg->rx = 1;
			break;
		case 't':
			pr_debug("Set TX mode\n");
			garg->tx = 1;
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
	    (mvapp_giu_max_num_qs_per_tc == 1) &&
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
	u64			cores_mask;
	int			err;

	setbuf(stdout, NULL);
	app_giu_set_gen_max_num_qs_per_tc();

	pr_info("pkt-gen is started\n");
	pr_debug("pr_debug is enabled\n");

	err = parse_args(&garg, argc, argv);
	if (err)
		return err;

	cores_mask = apps_cores_mask_create(garg.cmn_args.cpus, garg.cmn_args.affinity);
	garg.cmn_args.cores_mask = cores_mask;

	memset(&mvapp_params, 0, sizeof(mvapp_params));
	mvapp_params.use_cli		= garg.cmn_args.cli;
	mvapp_params.num_cores		= garg.cmn_args.cpus;
	mvapp_params.cores_mask		= cores_mask;
	mvapp_params.global_arg		= (void *)&garg;
	mvapp_params.init_global_cb	= init_global;
	mvapp_params.deinit_global_cb	= NULL;
	mvapp_params.init_local_cb	= init_local;
	mvapp_params.deinit_local_cb	= NULL;
	mvapp_params.main_loop_cb	= main_loop_cb;
	mvapp_params.ctrl_cb		= ctrl_loop_cb;

	return mvapp_go(&mvapp_params);
}
