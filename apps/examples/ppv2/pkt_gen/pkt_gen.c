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
#include <arpa/inet.h>	/* ntohs */
#include <netinet/ip.h>
#include <netinet/udp.h>
#include <getopt.h>

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

#define PKT_GEN_APP_DMA_MEM_SIZE		(40 * 1024 * 1024)

#define PKT_GEN_APP_FIRST_INQ			0
#define PKT_GEN_APP_MAX_NUM_TCS_PER_PORT	1
#define PKT_GEN_APP_MAX_NUM_QS_PER_CORE	PKT_GEN_APP_MAX_NUM_TCS_PER_PORT

#define PKT_GEN_APP_DIR_RX			0x1
#define PKT_GEN_APP_DIR_TX			0x2

#define  PKT_GEN_APP_HW_TX_CHKSUM_CALC
#ifdef PKT_GEN_APP_HW_TX_CHKSUM_CALC
#define PKT_GEN_APP_HW_TX_L4_CHKSUM_CALC	1
#define PKT_GEN_APP_HW_TX_IPV4_CHKSUM_CALC	1
#endif /* PKT_GEN_APP_HW_TX_CHKSUM_CALC */

#define PKT_GEN_APP_PREFETCH_SHIFT	4

#define PKT_GEN_APP_BPOOLS_INF		{ {384, 4096}, {2048, 4096} }
#define PKT_GEN_APP_BPOOLS_JUMBO_INF	{ {2048, 4096}, {10240, 512} }

#define MAX_BODYSIZE			(DEFAULT_MTU - IPV4_HDR_LEN - UDP_HDR_LEN)
#define DEFAULT_PKT_SIZE		DEFAULT_MTU

#define DEFAULT_REPORT_TIME 1 /* 1 second */
#define DEFAULT_WAIT_CYCLE  0
#define DEFAULT_SRC_IP 0x0a000001
#define DEFAULT_DST_IP 0x0a000002
#define DEFAULT_SRC_PORT 1024
#define DEFAULT_DST_PORT 1024

#define ETHERTYPE_IP	0x800

struct buffer_dec {
	void		*virt_addr;
	dma_addr_t	 phy_addr;
};

struct ip_range {
	u32 start, end, curr; /* same as struct in_addr */
	u16 port0, port1, port_curr;
};

struct mac_range {
	eth_addr_t start;
	eth_addr_t end;
};

struct ether_header {
	eth_addr_t	ether_dmac;
	eth_addr_t	ether_smac;
	u16		ether_type;
} __packed;

struct pkt {
	struct ether_header eh;
	struct ip ip;
	struct udphdr udp;
	u8 body[MAX_BODYSIZE];
} __attribute__((__packed__));

struct glob_arg {
	struct glb_common_args	cmn_args; /* Keep first */

	u16			rxq_size;
	u32			busy_wait;
	int			multi_buffer_release;
	int			loopback;
	int			maintain_stats;
	pthread_mutex_t		trd_lock;

	int			rx;
	int			tx;

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

	u32				busy_wait;
	int				multi_buffer_release;
	void				*buffer;
	struct buffer_dec		buf_dec[PKT_GEN_APP_MAX_BURST_SIZE];
	int				pkt_size;

	struct traffic_counters	trf_cntrs;
};

eth_addr_t default_src_mac = {0x00, 0x00, 0x00, 0x00, 0x00, 0x01};
eth_addr_t default_dst_mac = {0x00, 0x00, 0x00, 0x00, 0x00, 0x02};

static struct glob_arg garg = {};

/*
 * increment the addressed in the packet,
 * starting from the least significant field.
 *	DST_IP DST_PORT SRC_IP SRC_PORT
 */
static void update_addresses(struct pkt *pkt, struct glob_arg *garg)
{
	struct ip *ip = &pkt->ip;
	struct udphdr *udp = &pkt->udp;

	do {
		/* XXX for now it doesn't handle non-random src, random dst */
		udp->uh_sport = htons(garg->src_ip.port_curr++);
		if (garg->src_ip.port_curr >= garg->src_ip.port1)
			garg->src_ip.port_curr = garg->src_ip.port0;

		ip->ip_src.s_addr = htonl(garg->src_ip.curr++);
		if (garg->src_ip.curr >= garg->src_ip.end)
			garg->src_ip.curr = garg->src_ip.start;

		udp->uh_dport = htons(garg->dst_ip.port_curr++);
		if (garg->dst_ip.port_curr >= garg->dst_ip.port1)
			garg->dst_ip.port_curr = garg->dst_ip.port0;

		ip->ip_dst.s_addr = htonl(garg->dst_ip.curr++);
		if (garg->dst_ip.curr >= garg->dst_ip.end)
			garg->dst_ip.curr = garg->dst_ip.start;
	} while (0);
	/* update checksum */
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
	u16			 i;

	shadow_q = &pp2_args->lcl_ports_desc[rx_ppio_id].shadow_qs[tc];
	shadow_q_size = pp2_args->lcl_ports_desc[rx_ppio_id].shadow_q_size;

	pp2_ppio_recv(pp2_args->lcl_ports_desc[rx_ppio_id].ppio, tc, qid, descs, &num);
	if (unlikely(larg->cmn_args.verbose && num))
		printf("recv %d pkts on ppio %d, tc %d, qid %d\n", num, rx_ppio_id, tc, qid);

	larg->trf_cntrs.rx_pkts += num;

	for (i = 0; i < num; i++) {
		char		*buff = (char *)(uintptr_t)pp2_ppio_inq_desc_get_cookie(&descs[i]);
		dma_addr_t	 pa = pp2_ppio_inq_desc_get_phys_addr(&descs[i]);
		u16 len = pp2_ppio_inq_desc_get_pkt_len(&descs[i]);
		struct pp2_bpool *bpool = pp2_ppio_inq_desc_get_bpool(&descs[i],
								      pp2_args->lcl_ports_desc[rx_ppio_id].ppio);

		if (unlikely(larg->cmn_args.verbose > 1)) {
			char *tmp_buff;

			tmp_buff = (char *)(((uintptr_t)(buff)) | sys_dma_high_addr);
			pr_debug("buff2(%p)\n", tmp_buff);
			tmp_buff += MVAPPS_PP2_PKT_DEF_EFEC_OFFS;
			printf("Received packet (va:%p, pa 0x%08x, len %d):\n",
			       tmp_buff,
			       (unsigned int)pa,
			       len);
			mem_disp(tmp_buff, len);
		}
		larg->trf_cntrs.rx_bytes += (len + ETH_FCS_LEN + ETH_IPG_LEN);

		shadow_q->ents[shadow_q->write_ind].buff_ptr.cookie = (uintptr_t)buff;
		shadow_q->ents[shadow_q->write_ind].buff_ptr.addr = pa;
		shadow_q->ents[shadow_q->write_ind].bpool = bpool;
		pr_debug("buff_ptr.cookie(0x%lx)\n", (u64)shadow_q->ents[shadow_q->write_ind].buff_ptr.cookie);
		shadow_q->write_ind++;
		if (shadow_q->write_ind == shadow_q_size)
			shadow_q->write_ind = 0;
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
	u16			 i, tx_num;
	int			 mycyc;

#ifdef PKT_GEN_APP_HW_TX_CHKSUM_CALC
	enum pp2_inq_l3_type     l3_type = PP2_INQ_L3_TYPE_IPV4_OK;
	enum pp2_inq_l4_type     l4_type = PP2_INQ_L4_TYPE_UDP;
	u8			 l3_offset = sizeof(struct ether_header), l4_offset = (l3_offset + sizeof(struct ip));
#endif /* PKT_GEN_APP_HW_TX_CHKSUM_CALC */

	for (i = 0; i < num; i++) {
		pp2_ppio_outq_desc_reset(&descs[i]);
#ifdef PKT_GEN_APP_HW_TX_CHKSUM_CALC
#if (PKT_GEN_APP_HW_TX_IPV4_CHKSUM_CALC || PKT_GEN_APP_HW_TX_L4_CHKSUM_CALC)
		pp2_ppio_outq_desc_set_proto_info(&descs[i], pp2_l3_type_inq_to_outq(l3_type),
						  pp2_l4_type_inq_to_outq(l4_type), l3_offset,
						  l4_offset, PKT_GEN_APP_HW_TX_IPV4_CHKSUM_CALC,
						  PKT_GEN_APP_HW_TX_L4_CHKSUM_CALC);
#endif /* (PKT_GEN_APP_HW_TX_IPV4_CHKSUM_CALC ||  ... */
#endif /* PKT_GEN_APP_HW_TX_CHKSUM_CALC */
		pp2_ppio_outq_desc_set_phys_addr(&descs[i], larg->buf_dec[i].phy_addr);
		pp2_ppio_outq_desc_set_pkt_offset(&descs[i], 0);
		pp2_ppio_outq_desc_set_pkt_len(&descs[i], larg->pkt_size);

		update_addresses((struct pkt *)larg->buf_dec[i].virt_addr, larg->cmn_args.garg);

		if (unlikely(larg->cmn_args.verbose > 1)) {
			printf("Sending packet (va:%p, pa 0x%08x, len %d):\n",
			       larg->buf_dec[i].virt_addr,
			       (unsigned int)larg->buf_dec[i].phy_addr,
			       larg->pkt_size);
			mem_disp(larg->buf_dec[i].virt_addr, larg->pkt_size);
		}
	}

	/* TX does not use BM buffers, therefore there is no issue to "drop" packets due to tx_queue_full */
	if (num) {
		tx_num = num;

		pp2_ppio_send(pp2_args->lcl_ports_desc[tx_ppio_id].ppio, pp2_args->hif, tc, descs, &tx_num);
		if (unlikely(larg->cmn_args.verbose && tx_num))
			printf("sent %d pkts on ppio %d, tc %d\n", tx_num, tx_ppio_id, tc);

		larg->trf_cntrs.tx_pkts += tx_num;
		larg->trf_cntrs.tx_bytes += tx_num * (larg->pkt_size + ETH_FCS_LEN + ETH_IPG_LEN);
		larg->trf_cntrs.tx_drop += (num - tx_num);
	}

	for (mycyc = 0; mycyc < larg->busy_wait; mycyc++)
		asm volatile("");

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

	num = larg->cmn_args.burst;
	port_index = 0;
	while (*running) {
		if (pp2_args->ports_desc[port_index].traffic_dir & PKT_GEN_APP_DIR_RX) {
			/* Find next queue to consume */
			do {
				qid++;
				if (qid == MVAPPS_PP2_MAX_NUM_QS_PER_TC) {
					qid = 0;
					tc++;
					if (tc == PKT_GEN_APP_MAX_NUM_TCS_PER_PORT)
						tc = 0;
				}
			} while (!(larg->cmn_args.qs_map & (1 << ((tc * MVAPPS_PP2_MAX_NUM_QS_PER_TC) + qid))));
			err = loop_rx(larg, port_index, tc, qid, num);
			if (err != 0)
				return err;
		}

		if (pp2_args->ports_desc[port_index].traffic_dir & PKT_GEN_APP_DIR_TX) {
			err = loop_tx(larg, port_index, 0, num);
			if (err != 0)
				return err;
		}
		port_index++;
		if (port_index >= larg->cmn_args.num_ports)
			port_index = 0;
	}

	return 0;
}

static int stat_loop_cb(void *arg)
{
	struct glob_arg *garg = (struct glob_arg *)arg;
	u64 pkts, bytes, drops, pps, bps;
	u64 tx_pkts_prev = 0, tx_bytes_prev = 0, rx_pkts_prev = 0, rx_bytes_prev = 0;
	u8 i;
	u8 timeout = garg->report_time;

	do {
		sleep(timeout);

		if (garg->rx) {
			pkts = 0;
			bytes = 0;

			for (i = 0; i < garg->cmn_args.cpus; i++) {
				pkts += (garg->cmn_args.largs[i])->trf_cntrs.rx_pkts;
				bytes += (garg->cmn_args.largs[i])->trf_cntrs.rx_bytes;
			}
			pps = (pkts - rx_pkts_prev) / timeout;
			bps = (bytes - rx_bytes_prev) / timeout * 8;

			printf("RX: %" PRIu64 " kpps, %" PRIu64 " kbps\t", pps / 1000, bps / 1000);

			rx_pkts_prev = pkts;
			rx_bytes_prev = bytes;
		}

		if (garg->tx) {
			pkts = 0;
			bytes = 0;
			drops = 0;

			for (i = 0; i < garg->cmn_args.cpus; i++) {
				pkts += (garg->cmn_args.largs[i])->trf_cntrs.tx_pkts;
				bytes += (garg->cmn_args.largs[i])->trf_cntrs.tx_bytes;
				drops += (garg->cmn_args.largs[i])->trf_cntrs.tx_drop;
			}
			pps = (pkts - tx_pkts_prev) / timeout;
			bps = (bytes - tx_bytes_prev) / timeout * 8;

			printf("TX: %" PRIu64 " kpps, %" PRIu64 " kbps, %" PRIu64 " kdrops\t",
				pps / 1000, bps / 1000, drops / 1000);

			tx_pkts_prev = pkts;
			tx_bytes_prev = bytes;
		}

		printf("\n");
	} while (1);

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
	pp2_params.hif_reserved_map = MVAPPS_PP2_HIFS_RSRV;
	pp2_params.bm_pool_reserved_map = MVAPPS_PP2_BPOOLS_RSRV;

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

static int init_local_modules(struct glob_arg *garg)
{
	int				err, port_index;
	struct bpool_inf		std_infs[] = PKT_GEN_APP_BPOOLS_INF;
	struct bpool_inf		jumbo_infs[] = PKT_GEN_APP_BPOOLS_JUMBO_INF;
	struct bpool_inf		*infs;
	struct pp2_glb_common_args *pp2_args = (struct pp2_glb_common_args *)garg->cmn_args.plat;
	int				i;

	pr_info("Local initializations ...\n");

	err = app_hif_init(&pp2_args->hif, PKT_GEN_APP_HIF_Q_SIZE);
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
				port->num_inqs[i] =  garg->cmn_args.cpus;
			port->inq_size	= garg->rxq_size;
			port->num_outqs	= PKT_GEN_APP_MAX_NUM_TCS_PER_PORT;
			port->outq_size	= PKT_GEN_APP_TX_Q_SIZE;
			port->first_inq	= PKT_GEN_APP_FIRST_INQ;
			if (garg->cmn_args.cpus == 1)
				port->hash_type = PP2_PPIO_HASH_T_NONE;
			else
				port->hash_type = PP2_PPIO_HASH_T_5_TUPLE;

			/* pkt_offset=0 not to be changed, before recoding rx_data_path to use pkt_offset as well */
			err = app_port_init(port, pp2_args->num_pools, pp2_args->pools_desc[port->pp_id],
					    garg->cmn_args.mtu, 0);
			if (err) {
				pr_err("Failed to initialize port %d (pp_id: %d)\n", port_index,
				       port->pp_id);
				return err;
			}
#ifdef CRYPT_APP_ONE_PORT_LOOP
			if (port_index) {
				err = pp2_ppio_set_promisc(port->ppio, 1);
				if (err)
					return err;
			}
#endif /* CRYPT_APP_ONE_PORT_LOOP */
		} else {
			return err;
		}
	}

	pr_info("done\n");
	return 0;
}

static int register_cli_cmds(struct glob_arg *garg)
{
	struct cli_cmd_params	 cmd_params;
	struct pp2_glb_common_args *pp2_args = (struct pp2_glb_common_args *)garg->cmn_args.plat;

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
	app_register_cli_common_cmds(pp2_args->ports_desc);

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
	pthread_mutex_lock(&garg->trd_lock);
	err = app_hif_init(&lcl_pp2_args->hif, PKT_GEN_APP_HIF_Q_SIZE);
	pthread_mutex_unlock(&garg->trd_lock);
	if (err)
		return err;

	larg->cmn_args.id               = id;
	larg->cmn_args.burst		= garg->cmn_args.burst;
	larg->busy_wait			= garg->busy_wait;
	larg->multi_buffer_release = garg->multi_buffer_release;
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

	larg->cmn_args.qs_map = garg->cmn_args.qs_map << (garg->cmn_args.qs_map_shift * id);

	garg->cmn_args.largs[id] = larg;

	for (i = 0; i < larg->cmn_args.num_ports; i++)
		glb_pp2_args->ports_desc[i].lcl_ports_desc[id] = lcl_pp2_args->lcl_ports_desc;
	pr_debug("thread %d (cpu %d) mapped to Qs %llx\n",
		 larg->cmn_args.id, sched_getcpu(), (unsigned long long)larg->cmn_args.qs_map);
	*_larg = larg;

	larg->buffer = mv_sys_dma_mem_alloc(DEFAULT_PKT_SIZE * PKT_GEN_APP_MAX_BURST_SIZE, 4);
	for (i = 0; i < PKT_GEN_APP_MAX_BURST_SIZE; i++) {
		struct pkt *header;

		larg->buf_dec[i].virt_addr = (void *)((unsigned char *)larg->buffer + i *  DEFAULT_PKT_SIZE);
		larg->buf_dec[i].phy_addr = mv_sys_dma_mem_virt2phys(larg->buf_dec[i].virt_addr);

		header = (struct pkt *)larg->buf_dec[i].virt_addr;
		memcpy(header->eh.ether_dmac, larg->cmn_args.garg->dst_mac, MV_ETH_ALEN);
		memcpy(header->eh.ether_smac, larg->cmn_args.garg->src_mac, MV_ETH_ALEN);

		header->eh.ether_type = htons(ETHERTYPE_IP);

		header->ip.ip_src.s_addr = htonl(larg->cmn_args.garg->src_ip.start);
		header->ip.ip_dst.s_addr = htonl(larg->cmn_args.garg->dst_ip.start);
		header->ip.ip_v = IPVERSION;
		header->ip.ip_hl = 5;
		header->ip.ip_id = 0;
		header->ip.ip_tos = IPTOS_LOWDELAY;
		header->ip.ip_len = htons(larg->pkt_size - sizeof(struct ether_header));
		header->ip.ip_id = 0;
		header->ip.ip_off = htons(IP_DF); /* Don't fragment */
		header->ip.ip_ttl = IPDEFTTL;
		header->ip.ip_p = IPPROTO_UDP;
		header->ip.ip_sum = 0;

		header->udp.uh_sport = htons(larg->cmn_args.garg->src_ip.port0);
		header->udp.uh_dport = htons(larg->cmn_args.garg->dst_ip.port0);
		header->udp.uh_ulen = htons(larg->pkt_size - sizeof(struct ether_header) - sizeof(struct ip));
		header->udp.uh_sum = 0;
	}

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
	       "\t-r, --rx                    enables port Rx mode (default: enabled)\n"
	       "\t-t, --tx                    enables port Tx (traffic generator) mode (default: disabled)\n"
	       "\tUsage: --rx/--tx options can be used per specific interface or for all interfaces:\n"
	       "\t\t%s -i eth0 --rx -i eth1 --tx\t[opens port eth0 in Rx mode and eth1 in Tx mode]\n"
	       "\t\t%s -i eth0,eth1 --rx --tx\t[opens eth0 and eth1 ports in Rx and Tx mode]\n"
	       "OPTIONS for Tx mode:\n"
	       "\t-b, --burst <size>          burst size, num_pkts handled in a batch.(default is %d)\n"
	       "\t-l, --size <pkt_size>       packet size in bytes excluding CRC.(default is %d)\n"
	       "\t-d, --dst-ip <d.d.d.d[:port[-d.d.d.d:port]]>\n"
	       "\t                            destination IP address or range\n"
	       "\t-s, --src-ip <d.d.d.d[:port[-d.d.d.d:port]]>\n"
	       "\t                            source IP address or range\n"
	       "\t-D, --dst-mac <XX:XX:XX:XX:XX:XX>    destination MAC address\n"
	       "\t-S, --src-mac <XX:XX:XX:XX:XX:XX>    source MAC address\n"
	       "\t-w, --wait-time <cycles>    cycles between tx bursts.(default is no wait)\n"
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
	       "\n", MVAPPS_NO_PATH(progname), MVAPPS_NO_PATH(progname), MVAPPS_NO_PATH(progname),
	       MVAPPS_NO_PATH(progname), PKT_GEN_APP_DFLT_BURST_SIZE, DEFAULT_PKT_SIZE, DEFAULT_REPORT_TIME
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
	int	i = 1;
	int option;
	int long_index = 0;
	int rv, curr_port_index = 0;
	int port_dir[MVAPPS_PP2_MAX_NUM_PORTS] = {0};
	int common_dir = 0;
	const char short_options[] = "hi:b:l:c:a:m:T:w:S:D:s:d:rtv";
	struct option long_options[] = {
		{"help", no_argument, 0, 'h'},
		{"interface", required_argument, 0, 'i'},
		{"rx", no_argument, 0, 'r'},
		{"tx", no_argument, 0, 't'},
		{"burst", required_argument, 0, 'b'},
		{"size", required_argument, 0, 'l'},
		{"cores", required_argument, 0, 'c'},
		{"affinity", required_argument, 0, 'a'},
		{"qmap", required_argument, 0, 'm'},
		{"report-time", required_argument, 0, 'T'},
		{"wait-time", required_argument, 0, 'w'},
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
	garg->cmn_args.cli = 0;
	garg->cmn_args.cpus = 1;
	garg->cmn_args.affinity = -1;
	garg->cmn_args.burst = PKT_GEN_APP_DFLT_BURST_SIZE;
	garg->cmn_args.mtu = DEFAULT_MTU;
	garg->cmn_args.qs_map = 0;
	garg->cmn_args.qs_map_shift = 0;
	garg->cmn_args.prefetch_shift = PKT_GEN_APP_PREFETCH_SHIFT;
	garg->cmn_args.pkt_offset = 0;
	garg->busy_wait	= DEFAULT_WAIT_CYCLE;
	garg->rxq_size = PKT_GEN_APP_RX_Q_SIZE;
	garg->multi_buffer_release = 1;
	garg->maintain_stats = 0;
	garg->report_time = DEFAULT_REPORT_TIME;
	garg->pkt_size = DEFAULT_PKT_SIZE;
	garg->src_ip.start = garg->src_ip.end = garg->src_ip.curr = DEFAULT_SRC_IP;
	garg->src_ip.port0 = garg->src_ip.port1 = garg->src_ip.port_curr = DEFAULT_SRC_PORT;
	garg->dst_ip.start = garg->dst_ip.end = garg->dst_ip.curr = DEFAULT_DST_IP;
	garg->dst_ip.port0 = garg->dst_ip.port1 = garg->dst_ip.port_curr = DEFAULT_DST_PORT;
	memcpy(garg->dst_mac, default_dst_mac, MV_ETH_ALEN);
	memcpy(garg->src_mac, default_src_mac, MV_ETH_ALEN);

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
			snprintf(pp2_args->ports_desc[garg->cmn_args.num_ports].name,
				 sizeof(pp2_args->ports_desc[garg->cmn_args.num_ports].name), "%s", optarg);
			curr_port_index = garg->cmn_args.num_ports;
			pr_debug("Set port %d: %s\n", curr_port_index, pp2_args->ports_desc[curr_port_index].name);
			garg->cmn_args.num_ports++;
			/* currently supporting only 1 port */
			if (garg->cmn_args.num_ports == 0) {
				pr_err("Invalid interface arguments format!\n");
				return -EINVAL;
			} else if (garg->cmn_args.num_ports > MVAPPS_PP2_MAX_NUM_PORTS) {
				pr_err("too many ports specified (%d vs %d)\n",
				       garg->cmn_args.num_ports, MVAPPS_PP2_MAX_NUM_PORTS);
				return -EINVAL;
			}
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
		case 'b':
			garg->cmn_args.burst = atoi(optarg);
			pr_debug("Set burst size to %d\n", garg->cmn_args.burst);
			break;
		case 'l':
			garg->pkt_size = atoi(optarg);
			pr_debug("Set packet size to %d\n", garg->pkt_size);
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
		case 'w':
			garg->busy_wait = atoi(optarg);
			pr_debug("Set busy_wait time to %d\n", garg->busy_wait);
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
	    (MVAPPS_PP2_MAX_NUM_QS_PER_TC == 1) &&
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
		pr_err("Invalid queues-mapping (ovelapping CPUs)!\n");
		return -EINVAL;
	}

	if (garg->pkt_size > DEFAULT_PKT_SIZE) {
		pr_err("illegal packet size (%d vs %d)!\n",
		       garg->pkt_size, DEFAULT_PKT_SIZE);
		return -EINVAL;
	}

	return 0;
}

int main(int argc, char *argv[])
{
	struct mvapp_params	mvapp_params;
	u64			cores_mask;
	struct pp2_glb_common_args *pp2_args;
	int			i, err;

	setbuf(stdout, NULL);
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
	mvapp_params.deinit_global_cb	= apps_pp2_deinit_global;
	mvapp_params.init_local_cb	= init_local;
	mvapp_params.deinit_local_cb	= apps_pp2_deinit_local;
	mvapp_params.main_loop_cb	= main_loop_cb;
	mvapp_params.ctrl_cb		= stat_loop_cb;

	return mvapp_go(&mvapp_params);
}
