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
#include <sys/sysinfo.h>

#include "mv_std.h"
#include "lib/lib_misc.h"
#include "env/io.h"
#include "env/mv_sys_dma.h"


#include "mvapp.h"
#include "mv_pp2.h"
#include "mv_pp2_hif.h"
#include "mv_pp2_bpool.h"
#include "mv_pp2_ppio.h"


/* Following defines are used in pp2_utils.h */
/* #define APP_HW_TX_CHKSUM_CALC */
#ifdef APP_HW_TX_CHKSUM_CALC
#define APP_HW_TX_CHKSUM_CALC			1
#define APP_HW_TX_IPV4_CHKSUM_CALC		1
#endif
#define APP_PKT_ECHO_SUPPORT
#define APP_USE_PREFETCH
#define APP_MAX_BURST_SIZE			1024
#define APP_TX_RETRY
/* #define PORTS_LOOPBACK */
/* #define HW_BUFF_RECYLCE */

#define USE_PP2_UTILS_LPBK_SW_RECYCLE
#include "pp2_utils.h"



#ifdef HW_BUFF_RECYLCE
static const char buf_release_str[] = "HW buffer release";
#else
static const char buf_release_str[] = "SW buffer release";
#endif

#ifdef PORTS_LOOPBACK
static const char app_mode_str[] = "Loopback mode";
#else
static const char app_mode_str[] = "Bridge mode";
#endif

#ifdef APP_TX_RETRY
static const char tx_retry_str[] = "Tx Retry enabled";
#else
static const char tx_retry_str[] = "Tx Retry disabled";
#endif

#define PKT_ECHO_APP_TX_RETRY_WAIT		1
#define PKT_ECHO_APP_DEF_Q_SIZE			1024
#define PKT_ECHO_APP_HIF_Q_SIZE			(8 * PKT_ECHO_APP_DEF_Q_SIZE)
#define PKT_ECHO_APP_RX_Q_SIZE			(2 * PKT_ECHO_APP_DEF_Q_SIZE)
#define PKT_ECHO_APP_TX_Q_SIZE			(2 * PKT_ECHO_APP_DEF_Q_SIZE)

#define PKT_ECHO_APP_DFLT_BURST_SIZE		256
#if PKT_ECHO_APP_DFLT_BURST_SIZE > APP_MAX_BURST_SIZE
#error "Invalid PKT_ECHO_APP_DFLT_BURST_SIZE"
#endif

#define PKT_ECHO_APP_DMA_MEM_SIZE		(40 * 1024 * 1024)
#define PKT_ECHO_APP_CTRL_DFLT_THR		1000

#define PKT_ECHO_APP_FIRST_INQ			0
#define PKT_ECHO_APP_MAX_NUM_TCS_PER_PORT	1
#define PKT_ECHO_APP_MAX_NUM_QS_PER_CORE	PKT_ECHO_APP_MAX_NUM_TCS_PER_PORT


#define PKT_ECHO_APP_PREFETCH_SHIFT		7

#define PKT_ECHO_APP_BPOOLS_INF		{ {384, 4096, 0, NULL}, {2048, 1024, 0, NULL} }
#define PKT_ECHO_APP_BPOOLS_JUMBO_INF	{ {2048, 4096, 0, NULL}, {10240, 512, 0, NULL} }

#define PKT_ECHO_MAX_FLOW_REQUESTS		64

struct wired_data_flow {
	u8 rx_ppio_id;
	u8 tc;
	u8 tc_qid;
	u8 tx_ppio_id;
	u8 tx_qid;
};

struct local_thr_params {
	int lcl_id;
	int num_flows;
	struct wired_data_flow flows[PKT_ECHO_MAX_FLOW_REQUESTS];
	/* TODO: Make use of following two fields in local_thread */
	u32 rx_port_mask; /* Bitmap of rx_ports, each bit represents a port of the garg */
	u32 tx_port_mask; /* Bitmap of tx_ports, each bit represents a port of the garg */
};

struct flow_request {
	u32 cpu_mask;		/* cpu_mask, not thread_mask */
	int rx_port;
	int tx_port;
};


struct glob_arg {
	struct glb_common_args	cmn_args; /* Keep first */

	u16				rxq_size;
	int				loopback;
	int				maintain_stats;
	pthread_mutex_t			trd_lock;

	/* Following fields are related to multi-port option (hard-nailed flows) */
	u16				num_flows;
	/* Flows in user requested format */
	struct flow_request		flows[PKT_ECHO_MAX_FLOW_REQUESTS];
	/* Number of rx_queues per port */
	int				port_num_inqs[MVAPPS_PP2_MULTI_PORT_MAX_NUM_PORTS];
	/* Parameters for the local_threads */
	struct local_thr_params		*lcl_params;
};

struct local_arg {
	struct local_common_args	cmn_args; /* Keep first */
	int				num_flows;
	struct wired_data_flow		*flows;
};

static struct glob_arg garg;



#ifdef HW_BUFF_RECYLCE
static inline void free_buffers(struct pp2_lcl_common_args *pp2_args,
				struct pp2_ppio_desc	*descs,
				u16			num,
				u8			ppio_id)
{
	int i;
	struct pp2_buff_inf binf;

	for (i = 0; i < num; i++) {
		struct pp2_bpool *bpool = pp2_ppio_inq_desc_get_bpool(&descs[i],
								      pp2_args->lcl_ports_desc[ppio_id].ppio);

		binf.addr = pp2_ppio_inq_desc_get_phys_addr(&descs[i]);
		binf.cookie = pp2_ppio_inq_desc_get_cookie(&descs[i]);
		pp2_bpool_put_buff(pp2_args->hif, bpool, &binf);
	}
	INC_FREE_COUNT(&pp2_args->lcl_ports_desc[ppio_id], num);
}
static inline int loop_hw_recycle(struct local_common_args *larg_cmn,
				  u8			 rx_ppio_id,
				  u8			 tx_ppio_id,
				  u8			 tc,
				  u8			 tc_qid,
				  u8			 tx_qid,
				  u16			 num)
{
	struct pp2_ppio_desc	 descs[APP_MAX_BURST_SIZE];
	struct pp2_lcl_common_args *pp2_args = (struct pp2_lcl_common_args *) larg_cmn->plat;
	struct perf_cmn_cntrs	*perf_cntrs = &larg_cmn->perf_cntrs;
	struct lcl_port_desc	*rx_lcl_port_desc = &(pp2_args->lcl_ports_desc[rx_ppio_id]);
	struct lcl_port_desc	*tx_lcl_port_desc = &(pp2_args->lcl_ports_desc[tx_ppio_id]);
	u16			 i, tx_num;
#ifdef APP_TX_RETRY
	u16			 desc_idx = 0, cnt = 0;
#endif
#ifdef APP_PKT_ECHO_SUPPORT
	int			 prefetch_shift = larg_cmn->prefetch_shift;
#endif /* APP_PKT_ECHO_SUPPORT */
	u16 pkt_offset = MVAPPS_PP2_PKT_EFEC_OFFS(rx_lcl_port_desc->pkt_offset[tc]);

	pp2_ppio_recv(rx_lcl_port_desc->ppio, tc, tc_qid, descs, &num);
	perf_cntrs->rx_cnt += num;
	INC_RX_COUNT(rx_lcl_port_desc, num);

	for (i = 0; i < num; i++) {
		char		*buff = (char *)(uintptr_t)pp2_ppio_inq_desc_get_cookie(&descs[i]);
		dma_addr_t	 pa = pp2_ppio_inq_desc_get_phys_addr(&descs[i]);
		u16 len = pp2_ppio_inq_desc_get_pkt_len(&descs[i]);
		struct pp2_bpool *bpool = pp2_ppio_inq_desc_get_bpool(&descs[i], rx_lcl_port_desc->ppio);

#ifdef APP_PKT_ECHO_SUPPORT
		if (likely(larg_cmn->echo)) {
			char *tmp_buff;
#ifdef APP_USE_PREFETCH
			if (num - i > prefetch_shift) {
				tmp_buff = (char *)(uintptr_t)pp2_ppio_inq_desc_get_cookie(&descs[i + prefetch_shift]);
				tmp_buff += pkt_offset;
				prefetch(tmp_buff);
			}
#endif /* APP_USE_PREFETCH */
			tmp_buff = buff;
			pr_debug("buff(%p)\n", tmp_buff);
			tmp_buff += pkt_offset;
			swap_l2(tmp_buff);
			swap_l3(tmp_buff);
		}
#endif /* APP_PKT_ECHO_SUPPORT */

		pp2_ppio_outq_desc_reset(&descs[i]);
		pp2_ppio_outq_desc_set_phys_addr(&descs[i], pa);
		pp2_ppio_outq_desc_set_pkt_offset(&descs[i], pkt_offset);
		pp2_ppio_outq_desc_set_pkt_len(&descs[i], len);
		pp2_ppio_outq_desc_set_cookie(&descs[i], (uintptr_t)(buff));
		pp2_ppio_outq_desc_set_pool(&descs[i], bpool);
	}

	SET_MAX_BURST(rx_lcl_port_desc, num);
#ifdef APP_TX_RETRY
	while (num) {
		tx_num = num;
		pp2_ppio_send(tx_lcl_port_desc->ppio, pp2_args->hif,
			      tx_qid, &descs[desc_idx], &tx_num);

		if (num > tx_num) {
			if (!cnt)
				INC_TX_RETRY_COUNT(rx_lcl_port_desc, num - tx_num);
			cnt++;
			usleep(PKT_ECHO_APP_TX_RETRY_WAIT);
		}
		desc_idx += tx_num;
		num -= tx_num;
		INC_TX_COUNT(rx_lcl_port_desc, tx_num);
		perf_cntrs->drop_cnt += tx_num;
	}
	SET_MAX_RESENT(rx_lcl_port_desc, cnt);
#else
	if (num) {
		tx_num = num;
		pp2_ppio_send(tx_lcl_port_desc->ppio, pp2_args->hif, tx_qid, descs, &tx_num);
		if (num > tx_num) {
			u16 not_sent = num - tx_num;

			free_buffers(pp2_args, &descs[tx_num], not_sent, rx_ppio_id);
			INC_TX_DROP_COUNT(lcl_port_desc, not_sent);
			perf_cntrs->drop_cnt += not_sent;
		}
		INC_TX_COUNT(rx_lcl_port_desc, tx_num);
		perf_cntrs->tx_cnt += tx_num;
	}
#endif /* APP_TX_RETRY */
	return 0;
}

#else
#endif /* HW_BUFF_RECYLCE */


static int loop_flows(struct local_arg *larg, int *running)
{
	int			err;
	u16			num;
	u8			tc = 0, tc_qid = 0, tx_qid = 0;
	u8			flow = 0, rx_ppio, tx_ppio;

	if (!larg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	num = larg->cmn_args.burst;

	while (*running) {
		/* Find next flow to consume */
		rx_ppio = larg->flows[flow].rx_ppio_id;
		tc	= larg->flows[flow].tc;
		tc_qid	= larg->flows[flow].tc_qid;
		tx_ppio = larg->flows[flow].tx_ppio_id;
		tx_qid	= larg->flows[flow].tx_qid;
		pr_debug("thr_id(%d) flow(%d) rx_ppio(%d) tc(%d) tc_qid(%d) tx_ppio(%d) tx_qid(%d)\n",
			 larg->cmn_args.id, flow, rx_ppio, tc, tc_qid, tx_ppio, tx_qid);
		flow++;
		if (flow == larg->num_flows)
			flow = 0;

#ifdef HW_BUFF_RECYLCE
		err = loop_hw_recycle(&larg->cmn_args, rx_ppio, tx_ppio, tc, tc_qid, tx_qid, num);
#else
		err = loop_sw_recycle(&larg->cmn_args, rx_ppio, tx_ppio, tc, tc_qid, tx_qid, num);
#endif /* HW_BUFF_RECYLCE */
		if (err != 0)
			return err;
	}

	return 0;
}




static int loop_1p(struct local_arg *larg, int *running)
{
	int			 err;
	u16			 num;
	u8			 tc = 0, tc_qid = 0, tx_qid = 0;

	if (!larg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	num = larg->cmn_args.burst;

	while (*running) {
		/* Find next queue to consume */
		do {
			tc_qid++;
			if (tc_qid == mvapp_pp2_max_num_qs_per_tc) {
				tc_qid = 0;
				tc++;
				if (tc == PKT_ECHO_APP_MAX_NUM_TCS_PER_PORT)
					tc = 0;
			}
		} while (!(larg->cmn_args.qs_map & (1 << ((tc * mvapp_pp2_max_num_qs_per_tc) + tc_qid))));

		tx_qid = tc % PP2_PPIO_MAX_NUM_OUTQS;

#ifdef HW_BUFF_RECYLCE
		err = loop_hw_recycle(&larg->cmn_args, 0, 0, tc, tc_qid, tx_qid, num);
#else
		err = loop_sw_recycle(&larg->cmn_args, 0, 0, tc, tc_qid, tx_qid, num);
#endif /* HW_BUFF_RECYLCE */
		if (err != 0)
			return err;
	}

	return 0;
}

static int loop_2ps(struct local_arg *larg, int *running)
{
	int			 err;
	u16			 num;
	u8			 tc = 0, tc_qid = 0, tx_qid = 0;
#ifdef PORTS_LOOPBACK
	int			 port_id = 0;
#endif
	if (!larg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

#ifdef PORTS_LOOPBACK
	if (larg->cmn_args.garg->cmn_args.cpus == 4) {
		/* For 4 cores: 2 first cores will work with port0, 2 other cores with port1,
		 * 2 first queues will be used for each port
		 */
		port_id = (larg->cmn_args.id < 2) ? 0 : 1;
		tc_qid = larg->cmn_args.id & 1;
		pr_debug("loop_2ps: for cpu %d (%d), port %d, queue %d\n", larg->cmn_args.id,
			 larg->cmn_args.garg->cmn_args.cpus, port_id, tc_qid);
	}
#endif

	num = larg->cmn_args.burst;
	while (*running) {
#ifndef PORTS_LOOPBACK
		/* Find next queue to consume */
		do {
			tc_qid++;
			if (tc_qid == mvapp_pp2_max_num_qs_per_tc) {
				tc_qid = 0;
				tc++;
				if (tc == PKT_ECHO_APP_MAX_NUM_TCS_PER_PORT)
					tc = 0;
			}
		} while (!(larg->cmn_args.qs_map & (1 << ((tc * mvapp_pp2_max_num_qs_per_tc) + tc_qid))));
#endif
		tx_qid = tc % PP2_PPIO_MAX_NUM_OUTQS;
#ifdef HW_BUFF_RECYLCE
#ifdef PORTS_LOOPBACK
		if (larg->cmn_args.garg->cmn_args.cpus == 4) {
			/* Use port_id and queue calculated in beginning of this function */
			err  = loop_hw_recycle(&larg->cmn_args, port_id, port_id, tc, tc_qid, tx_qid, num);
		} else {
			/* Set larg->cmn_args.id as rx_ppio, tx_ppio, for quick 2xloopback implementation
			 * and queue 0 only
			 */
			err  = loop_hw_recycle(&larg->cmn_args, larg->cmn_args.id, larg->cmn_args.id, tc, 0, num);
		}
#else
		err  = loop_hw_recycle(&larg->cmn_args, 0, 1, tc, tc_qid, tx_qid, num);
		err |= loop_hw_recycle(&larg->cmn_args, 1, 0, tc, tc_qid, tx_qid, num);
#endif /* PORTS_LOOPBACK */
#else
#ifdef PORTS_LOOPBACK

		if (larg->cmn_args.garg->cmn_args.cpus == 4) {
			/* Use port_id and queue calculated in beginning of this function */
			err  = loop_sw_recycle(&larg->cmn_args, port_id, port_id, tc, tc_qid, num);
		} else {
			/* Set larg->cmn_args.id as rx_ppio, tx_ppio, for quick 2xloopback implementation
			 * and queue 0 only
			 */
			err  = loop_sw_recycle(&larg->cmn_args, larg->cmn_args.id, larg->cmn_args.id, tc, tc_qid, num);
		}
#else
		err  = loop_sw_recycle(&larg->cmn_args, 0, 1, tc, tc_qid, tx_qid, num);
		err |= loop_sw_recycle(&larg->cmn_args, 1, 0, tc, tc_qid, tx_qid, num);
#endif /* PORTS_LOOPBACK */
#endif /* HW_BUFF_RECYLCE */
		if (err != 0)
			return err;
	}

	return 0;
}

static int main_loop(void *arg, int *running)
{
	struct local_arg	*larg = (struct local_arg *)arg;

	if (!larg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	if (larg->num_flows)
		return loop_flows(larg, running);

	if (larg->cmn_args.num_ports == 1)
		return loop_1p(larg, running);
	return loop_2ps(larg, running);
}

static int init_all_modules(void)
{
	struct pp2_init_params	 pp2_params;
	struct glb_common_args *cmn_args = &garg.cmn_args;
	struct pp2_glb_common_args *pp2_args = (struct pp2_glb_common_args *)garg.cmn_args.plat;
	int			 err = 0;
	char			 file[PP2_MAX_BUF_STR_LEN];
	int			 num_rss_tables = 0;
	struct mv_sys_dma_mem_region_params params;

	pr_info("Global initializations ...\n");

	app_get_num_cpu_clusters(cmn_args);

	params.size = PKT_ECHO_APP_DMA_MEM_SIZE;
	params.manage = 1;

	err = app_sys_dma_init(&params, cmn_args);
	if (err)
		return err;

	memset(&pp2_params, 0, sizeof(pp2_params));
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

	/* Must be after pp2_init */
	pp2_params.hif_reserved_map = pp2_get_kernel_hif_map();
	app_used_hifmap_init(pp2_params.hif_reserved_map);

	pr_info("done\n");
	return 0;
}


static void set_local_flows(struct glob_arg *garg)
{
	int i;

	garg->lcl_params = (struct local_thr_params *) malloc((sizeof(struct local_thr_params))*garg->cmn_args.cpus);
	memset(garg->lcl_params, 0, (sizeof(struct local_thr_params)) * garg->cmn_args.cpus);

	for (i = 0; i < garg->num_flows; i++) {
		int thr_id = 0;
		u32 garg_cores_mask = garg->cmn_args.cores_mask;
		u32 flow_cores_mask = garg->flows[i].cpu_mask;
		int rx_port = garg->flows[i].rx_port;
		int tx_port = garg->flows[i].tx_port;

		while (flow_cores_mask) {
			/* Copy flow into the local_threads that participate in the flow_cpu_mask.
			 * Per port, first N rx_qs are allocated to flow#0, next M rx_qs to flow#1, etc.
			*/
			if ((flow_cores_mask & BIT(0))) {
				struct local_thr_params *lcl_param = &garg->lcl_params[thr_id];

				lcl_param->lcl_id = thr_id;
				lcl_param->rx_port_mask |= BIT(rx_port);
				lcl_param->tx_port_mask |= BIT(tx_port);

				lcl_param->flows[lcl_param->num_flows].rx_ppio_id = rx_port;
				lcl_param->flows[lcl_param->num_flows].tc = 0; /* Hardcoded to 0, TODO make define */
				lcl_param->flows[lcl_param->num_flows].tc_qid = garg->port_num_inqs[rx_port];

				lcl_param->flows[lcl_param->num_flows].tx_ppio_id = tx_port;
				lcl_param->flows[lcl_param->num_flows].tx_qid = 0; /* TODO: Decide which options */

				lcl_param->num_flows++;
				garg->port_num_inqs[rx_port]++;
			}

			/* Thread(n) affinity is the N-th 1-BIT of garg_cores_mask.
			 * ==> thr_id is increased, only if garg_cores_mask bit(n)=1.
			*/
			if (garg_cores_mask & BIT(0))
				thr_id++;

			/* Both masks get shifted each iteration. */
			garg_cores_mask = (garg_cores_mask >> 1);
			flow_cores_mask = (flow_cores_mask >> 1);

		}

	}
}



static int init_local_modules(struct glob_arg *garg)
{
	int				err, port_index;
	struct bpool_inf		std_inf[] = PKT_ECHO_APP_BPOOLS_INF, jumbo_inf[] = PKT_ECHO_APP_BPOOLS_INF;
	struct bpool_inf		*infs;
	struct pp2_glb_common_args *pp2_args = (struct pp2_glb_common_args *) garg->cmn_args.plat;
	int				i = 0, j;

	pr_info("Local initializations ...\n");

	err = app_build_common_hifs(&garg->cmn_args, PKT_ECHO_APP_HIF_Q_SIZE);
	if (err)
		return err;

	app_prepare_bpools(&garg->cmn_args, &infs, std_inf, ARRAY_SIZE(std_inf), jumbo_inf, ARRAY_SIZE(jumbo_inf));

	err = app_build_all_bpools(&pp2_args->pools_desc, pp2_args->num_pools, infs, pp2_args->hif);
	if (err)
		return err;

	/* Fill in local_thread flows */
	if (garg->num_flows) {
		set_local_flows(garg);

		/* Debug, TODO: Replace with pr_debug after verification */
		for (i = 0; i < garg->cmn_args.cpus; i++) {
			struct local_thr_params *lcl_param = &garg->lcl_params[i];

			printf(" thr_id(%d), num_flows(%d), rx_mask(0x%x) tx_mask(0x%x)\n",
				i, lcl_param->num_flows, lcl_param->rx_port_mask, lcl_param->tx_port_mask);
			for (j = 0; j < lcl_param->num_flows; j++) {
				struct wired_data_flow *flow = &lcl_param->flows[j];

				printf("\t flow(%2d): rx_port(%u), tc(%u) tc_qid(%2u) tx_port(%u) txq(%u)\n",
					j, flow->rx_ppio_id, flow->tc, flow->tc_qid, flow->tx_ppio_id, flow->tx_qid);
			}
		}
	}
	for (port_index = 0; port_index < garg->cmn_args.num_ports; port_index++) {
		struct port_desc *port = &pp2_args->ports_desc[port_index];

		err = app_find_port_info(port);
		if (!err) {
			memcpy(port->mem_region, garg->cmn_args.mem_region, sizeof(garg->cmn_args.mem_region));
			port->ppio_type	= PP2_PPIO_T_NIC;
			port->num_tcs	= PKT_ECHO_APP_MAX_NUM_TCS_PER_PORT;
			if ((port->num_tcs > 1) && garg->port_num_inqs[port_index]) {
				pr_err("Multiple TCs not supported with this option\n");
				return -EIO;
			}
			for (i = 0; i < port->num_tcs; i++) {
				int config_inqs = garg->port_num_inqs[port_index];

				port->num_inqs[i] = (config_inqs) ? config_inqs : garg->cmn_args.cpus;
				if (garg->num_flows)
					printf("port(%d-%d-tc(%d)), inqs(%d)\n",
					       port->pp_id, port->ppio_id, i, port->num_inqs[i]);
			}
			port->inq_size	= garg->rxq_size;
			port->num_outqs	= PKT_ECHO_APP_MAX_NUM_TCS_PER_PORT;
			port->outq_size	= PKT_ECHO_APP_TX_Q_SIZE;
			port->first_inq	= PKT_ECHO_APP_FIRST_INQ;
			if (garg->cmn_args.cpus == 1)
				port->hash_type = PP2_PPIO_HASH_T_NONE;
			else
				port->hash_type = PP2_PPIO_HASH_T_2_TUPLE;
			/* Create shared shadow_qs for all shared_hifs
			 * TODO: Refine this check, to create shared_shadow_qs only for those threads that
			 *       have shared_hif w/other thread, which are also sending on this port.
			*/
			if (garg->cmn_args.shared_hifs) {
				int num_shadow_qs = port->num_outqs;

				i = 0;
				while (i < MVAPPS_PP2_TOTAL_NUM_HIFS && pp2_args->app_hif[i]) {
					int shadow_qsize, num_threads;
					u32 thr_mask = pp2_args->app_hif[i]->local_thr_id_mask;

					num_threads = __builtin_popcount(thr_mask);
					shadow_qsize = port->outq_size + port->num_tcs * num_threads * port->inq_size;
					app_port_shared_shadowq_create(&(port->shared_qs[i]),
						thr_mask, num_shadow_qs, shadow_qsize);
					i++;
				}
			}
			if (0)
				;/* TODO Handle flows */
			else
				app_port_inqs_mask_by_affinity(&garg->cmn_args, port);

			err = app_port_init(port, pp2_args->num_pools, pp2_args->pools_desc[port->pp_id],
					    garg->cmn_args.mtu, garg->cmn_args.pkt_offset);
			if (err) {
				pr_err("Failed to initialize port %d (pp_id: %d)\n", port_index,
				       port->pp_id);
				return err;
			}
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
	cmd_params.cmd_arg	= &garg->cmn_args;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))apps_pp2_stat_cmd_cb;
	mvapp_register_cli_cmd(&cmd_params);
	app_register_cli_common_cmds(&garg->cmn_args);

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

	if (pthread_mutex_init(&garg->cmn_args.thread_lock, NULL) != 0) {
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


static void local_params_update(struct local_thr_params *lcl_param, struct local_arg *lcl_arg)
{
	lcl_arg->num_flows = lcl_param->num_flows;
	lcl_arg->flows = malloc(lcl_arg->num_flows * sizeof(struct wired_data_flow));
	memcpy(lcl_arg->flows, lcl_param->flows, lcl_param->num_flows * sizeof(struct wired_data_flow));
}

static int init_local(void *arg, int id, void **_larg)
{
	struct glob_arg		*garg = (struct glob_arg *)arg;
	struct local_arg	*larg;
	struct pp2_glb_common_args *glb_pp2_args = (struct pp2_glb_common_args *) garg->cmn_args.plat;
	struct pp2_lcl_common_args *lcl_pp2_args;
	int			 i, err;
	int mem_id;

	if (!garg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	mem_id = sched_getcpu() / MVAPPS_NUM_CORES_PER_AP;
	larg = (struct local_arg *)malloc(sizeof(struct local_arg));
	if (!larg) {
		pr_err("No mem for local arg obj!\n");
		return -ENOMEM;
	}
	memset(larg, 0, sizeof(struct local_arg));

	larg->cmn_args.plat = (struct pp2_lcl_common_args *)malloc(sizeof(struct pp2_lcl_common_args));
	if (!larg->cmn_args.plat) {
		pr_err("No mem for local plat arg obj!\n");
		free(larg);
		return -ENOMEM;
	}
	memset(larg->cmn_args.plat, 0, sizeof(struct pp2_lcl_common_args));
	lcl_pp2_args = (struct pp2_lcl_common_args *) larg->cmn_args.plat;

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

	err = app_hif_init_wrap(id, &garg->cmn_args.thread_lock, glb_pp2_args, lcl_pp2_args, PKT_ECHO_APP_HIF_Q_SIZE,
				garg->cmn_args.mem_region[mem_id]);
	if (err)
		return err;

	larg->cmn_args.burst		= garg->cmn_args.burst;
	larg->cmn_args.busy_wait		= garg->cmn_args.busy_wait;
	larg->cmn_args.echo              = garg->cmn_args.echo;
	larg->cmn_args.prefetch_shift	= garg->cmn_args.prefetch_shift;

	if (garg->lcl_params)
		local_params_update(&garg->lcl_params[id], larg);

	for (i = 0; i < larg->cmn_args.num_ports; i++)
		app_port_local_init(i, larg->cmn_args.id, &lcl_pp2_args->lcl_ports_desc[i],
				    &glb_pp2_args->ports_desc[i]);

	lcl_pp2_args->pools_desc	= glb_pp2_args->pools_desc;
	lcl_pp2_args->multi_buffer_release = glb_pp2_args->multi_buffer_release;
	larg->cmn_args.garg             = garg;

	larg->cmn_args.qs_map = garg->cmn_args.qs_map << (garg->cmn_args.qs_map_shift * id);

	/* Update garg refs to local */
	garg->cmn_args.largs[id] = larg;
	for (i = 0; i < larg->cmn_args.num_ports; i++)
		glb_pp2_args->ports_desc[i].lcl_ports_desc[id] = &lcl_pp2_args->lcl_ports_desc[i];

	pr_debug("thread %d (cpu %d) (mem_id %d) mapped to Qs %llx\n",
		 larg->cmn_args.id, sched_getcpu(), mem_id, (unsigned long long)larg->cmn_args.qs_map);

	*_larg = larg;
	return 0;
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
	       "\tOptional Sub-Options for -i option:\n"
	       "\t\t--cmask <number> cores_mask, create wired_flows between the 2 interfaces for each cpu in cmask\n"
	       "\t\t-u      unidir, create flows only from first(rx) interface to second (tx)interface\n"
	       "\t\tNote: When --cmask suboption is added, the -i option may be added multiple times\n"
	       "\t\t      e.g. -i eth0,eth2 --cmask 0x3 -i eth2,eth3 --cmask 0xc -u\n"
	       "\n"
	       "Optional OPTIONS:\n"
	       "\t-b <size>                Burst size, num_pkts handled in a batch.(default is %d)\n"
	       "\t--mtu <mtu>              Set MTU (default is %d)\n"
	       "\t\t-c, --cores <number>     Number of CPUs to use\n"
	       "\t\t-a, --affinity <number>  Use setaffinity (default is no affinity)\n"
	       "\t-s                       Maintain statistics\n"
	       "\t-w <cycles>              Cycles to busy_wait between recv&send, simulating app behavior (default=0)\n"
	       "\t--rxq <size>             Size of rx_queue (default is %d)\n"
	       "\t--pkt-offset <size>      Packet offset in buffer, must be multiple of 32-byte (default is %d)\n"
	       "\t--mem-regions <number>   Number of mv_sys_dma_mem_regions (default=0)\n"
	       "\t--old-tx-desc-release    Use pp2_bpool_put_buff(), instead of NEW pp2_bpool_put_buffs() API\n"
	       "\t--no-echo                Don't perform 'pkt_echo', N/A w/o define APP_PKT_ECHO_SUPPORT\n"
	       "\t--cli                    Use CLI\n"
	       "\t?, -h, --help            Display help and exit.\n\n"
	       "\n", MVAPPS_NO_PATH(progname), MVAPPS_NO_PATH(progname),
	       MVAPPS_PP2_MAX_I_OPTION_PORTS, APP_MAX_BURST_SIZE, DEFAULT_MTU, PKT_ECHO_APP_RX_Q_SIZE,
	       MVAPPS_PP2_PKT_DEF_OFFS);
}

static int parse_args(struct glob_arg *garg, int argc, char *argv[])
{
	int	i = 1;
	struct pp2_glb_common_args *pp2_args = (struct pp2_glb_common_args *) garg->cmn_args.plat;
	bool flow_based = false, port_based = false;

	garg->cmn_args.cli = 0;
	garg->cmn_args.cpus = 1;
	garg->cmn_args.affinity = MVAPPS_INVALID_AFFINITY;
	garg->cmn_args.burst = PKT_ECHO_APP_DFLT_BURST_SIZE;
	garg->cmn_args.mtu = DEFAULT_MTU;
	garg->cmn_args.busy_wait	= 0;
	garg->rxq_size = PKT_ECHO_APP_RX_Q_SIZE;
	garg->cmn_args.echo = 1;
	garg->cmn_args.qs_map = 0;
	garg->cmn_args.qs_map_shift = 0;
	garg->cmn_args.pkt_offset = 0;
	garg->cmn_args.prefetch_shift = PKT_ECHO_APP_PREFETCH_SHIFT;
	garg->cmn_args.ctrl_thresh = PKT_ECHO_APP_CTRL_DFLT_THR;
	garg->cmn_args.num_mem_regions = MVAPPS_INVALID_MEMREGIONS;
	garg->maintain_stats = 0;

	pp2_args->multi_buffer_release = 1;

	while (i < argc) {
		if ((strcmp(argv[i], "?") == 0) ||
		    (strcmp(argv[i], "-h") == 0) ||
		    (strcmp(argv[i], "--help") == 0)) {
			usage(argv[0]);
			exit(0);
		} else if (strcmp(argv[i], "-i") == 0) {
			char *token;
			int port_found[MVAPPS_PP2_MAX_I_OPTION_PORTS] = {-1, -1};
			int j, port;
			u32 cmask = 0;
			bool unidir = false, current_flow_based = false;

			if (argc < (i + 2)) {
				pr_err("Invalid number of arguments!\n");
				return -EINVAL;
			}
			if (argv[i + 1][0] == '-') {
				pr_err("Invalid interface arguments format!\n");
				return -EINVAL;
			}

			/* count the number of tokens separated by ',' */
			for (token = strtok(argv[i + 1], ","), j = 0; token; token = strtok(NULL, ",")) {

				if (j == MVAPPS_PP2_MAX_I_OPTION_PORTS)  {
					pr_err("too many ports specified in -i option, max(%d)\n",
						MVAPPS_PP2_MAX_I_OPTION_PORTS);
					return -EINVAL;
				}
				/* Check if port was added in previous -i option */
				for (port = 0; port < garg->cmn_args.num_ports; port++) {
					if (strcmp(pp2_args->ports_desc[port].name, token) == 0) {
						port_found[j] = port;
						break;
					}
				}
				/* A new port */
				if (port_found[j] < 0) {
					if (garg->cmn_args.num_ports >= MVAPPS_PP2_MULTI_PORT_MAX_NUM_PORTS) {
						pr_err("too many ports specified, more than %d\n",
						       MVAPPS_PP2_MULTI_PORT_MAX_NUM_PORTS);
						return -EINVAL;
					}
					snprintf(pp2_args->ports_desc[garg->cmn_args.num_ports].name,
						 sizeof(pp2_args->ports_desc[garg->cmn_args.num_ports].name),
						 "%s", token);
					port_found[j] = garg->cmn_args.num_ports;
					garg->cmn_args.num_ports++;
				}
				j++;
			}
			i += 2;
			/* Check "--cmask" and "-u" suboptions */
			while (i < argc) {
				if (strcmp(argv[i], "--cmask") == 0) {
					flow_based = true;
					current_flow_based = true;
					cmask = strtol(argv[i + 1], NULL, 0);
					i += 2;
				} else if (strcmp(argv[i], "-u") == 0) {
					unidir = true;
					i += 1;
				} else {
					if (!current_flow_based)
						port_based = true;
					break;
				}
			}
			if (port_based && flow_based) {
				pr_err("Invalid to use -i option both with and without --cmask option\n");
				return -EINVAL;
			}
			if (port_based && garg->cmn_args.num_ports > MVAPPS_PP2_MAX_NUM_PORTS) {
				pr_err("too many ports specified (%d vs %d)\n",
				       garg->cmn_args.num_ports, MVAPPS_PP2_MAX_NUM_PORTS);
				return -EINVAL;
			}
			if (current_flow_based) {
				struct flow_request *flow;
				/* Single port means there is only a single flow */
				if (port_found[1] < 0) {
					unidir = true;
					port_found[1] = port_found[0];
				}
				flow = &garg->flows[garg->num_flows];
				flow->cpu_mask = cmask;
				flow->rx_port = port_found[0];
				flow->tx_port = port_found[1];
				garg->num_flows++;
				if (!unidir) {
					/* Add reverse direction */
					flow++;
					flow->cpu_mask = cmask;
					flow->rx_port = port_found[1];
					flow->tx_port = port_found[0];
					garg->num_flows++;
				}
			}
		} else if (strcmp(argv[i], "--mtu") == 0) {
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
		} else if (strcmp(argv[i], "-s") == 0) {
			garg->maintain_stats = 1;
			i += 1;
		} else if (strcmp(argv[i], "-w") == 0) {
			garg->cmn_args.busy_wait = atoi(argv[i + 1]);
			i += 2;
		} else if (strcmp(argv[i], "--rxq") == 0) {
			garg->rxq_size = atoi(argv[i + 1]);
			i += 2;
		} else if (strcmp(argv[i], "--pkt-offset") == 0) {
			garg->cmn_args.pkt_offset = atoi(argv[i + 1]);
			i += 2;
		} else if (strcmp(argv[i], "--old-tx-desc-release") == 0) {
			pp2_args->multi_buffer_release = 0;
			i += 1;
		} else if (strcmp(argv[i], "-m") == 0) {
			int rv;

			if (argc < (i + 2)) {
				pr_err("Invalid number of arguments!\n");
				return -EINVAL;
			}
			if (argv[i + 1][0] == '-') {
				pr_err("Invalid arguments format!\n");
				return -EINVAL;
			}
			rv = sscanf(argv[i + 1], "%x:%x", (unsigned int *)&garg->cmn_args.qs_map,
				    &garg->cmn_args.qs_map_shift);
			if (rv != 2) {
				pr_err("Failed to parse -m parameter\n");
				return -EINVAL;
			}
			i += 2;
		} else if (strcmp(argv[i], "--no-echo") == 0) {
			garg->cmn_args.echo = 0;
			i += 1;
		} else if (strcmp(argv[i], "--cli") == 0) {
			garg->cmn_args.cli = 1;
			i += 1;
		} else if (strcmp(argv[i], "--mem-regions") == 0) {
			garg->cmn_args.num_mem_regions = atoi(argv[i + 1]);
			i += 2;
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
	if (garg->cmn_args.burst > APP_MAX_BURST_SIZE) {
		pr_err("illegal burst size requested (%d vs %d)!\n",
		       garg->cmn_args.burst, APP_MAX_BURST_SIZE);
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
	    (PKT_ECHO_APP_MAX_NUM_TCS_PER_PORT == 1)) {
		pr_warn("no point in queues-mapping; ignoring.\n");
		garg->cmn_args.qs_map = 1;
		garg->cmn_args.qs_map_shift = 1;
	} else if (!garg->cmn_args.qs_map) {
		garg->cmn_args.qs_map = 1;
		garg->cmn_args.qs_map_shift = PKT_ECHO_APP_MAX_NUM_TCS_PER_PORT;
	}

	if ((garg->cmn_args.cpus != 1) &&
	    (garg->cmn_args.qs_map & (garg->cmn_args.qs_map << garg->cmn_args.qs_map_shift))) {
		pr_err("Invalid queues-mapping (ovelapping CPUs)!\n");
		return -EINVAL;
	}

	return 0;
}

int main(int argc, char *argv[])
{
	struct mvapp_params	mvapp_params;
	u64			cores_mask = 0;
	struct pp2_glb_common_args *pp2_args;

	int			err, i;

	setbuf(stdout, NULL);
	app_set_max_num_qs_per_tc();

	pr_info("pkt-echo is started in %s - %s - %s\n", app_mode_str, buf_release_str, tx_retry_str);
	pr_debug("pr_debug is enabled\n");
	garg.cmn_args.plat = (struct pp2_glb_common_args *)malloc(sizeof(struct pp2_glb_common_args));
	if (!garg.cmn_args.plat) {
		pr_err("No mem for global plat arg obj!\n");
		return -ENOMEM;
	}
	pp2_args = (struct pp2_glb_common_args *) garg.cmn_args.plat;
	err = parse_args(&garg, argc, argv);
	if (err) {
		free(garg.cmn_args.plat);
		return err;
	}

	pp2_args->pp2_num_inst = pp2_get_num_inst();

	if (garg.num_flows) {
#ifndef MVCONF_PP2_LOCK
		pr_err("For multi-port pkt_echo (using --cmask), musdk must be compiled with --enable-pp2-lock\n");
		return -EPERM;
#endif
		for (i = 0; i < garg.num_flows; i++)
			cores_mask |= garg.flows[i].cpu_mask;
		garg.cmn_args.cpus = bit_count(cores_mask);

		pr_debug("cores_mask(0x%lx) cpus(%d)\n", cores_mask, garg.cmn_args.cpus);
	} else
		cores_mask = apps_cores_mask_create(garg.cmn_args.cpus, garg.cmn_args.affinity);

	garg.cmn_args.cores_mask = cores_mask;

	/* Debug_code. TODO: Replace with pr_debug. */
	for (i = 0; i < garg.num_flows; i++) {
		struct pp2_glb_common_args *pp2_args = (struct pp2_glb_common_args *) garg.cmn_args.plat;

		printf("flow(%d), rx_port(%d,%s), tx_port(%d,%s) cpu_mask(0x%x)\n",
			i, garg.flows[i].rx_port, pp2_args->ports_desc[garg.flows[i].rx_port].name,
			garg.flows[i].tx_port, pp2_args->ports_desc[garg.flows[i].tx_port].name,
			garg.flows[i].cpu_mask);
	}

	memset(&mvapp_params, 0, sizeof(mvapp_params));
	mvapp_params.use_cli		= garg.cmn_args.cli;
	mvapp_params.num_cores		= garg.cmn_args.cpus;
	mvapp_params.cores_mask		= cores_mask;
	mvapp_params.global_arg		= (void *)&garg;
	mvapp_params.init_global_cb	= init_global;
	mvapp_params.deinit_global_cb	= apps_pp2_deinit_global;
	mvapp_params.init_local_cb	= init_local;
	mvapp_params.deinit_local_cb	= apps_pp2_deinit_local;
	mvapp_params.main_loop_cb	= main_loop;
	if (!mvapp_params.use_cli)
		mvapp_params.ctrl_cb	= app_ctrl_cb;

	return mvapp_go(&mvapp_params);
}
