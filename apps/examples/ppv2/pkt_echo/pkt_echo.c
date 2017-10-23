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

#include "mv_std.h"
#include "lib/lib_misc.h"
#include "env/io.h"
#include "env/mv_sys_dma.h"

#include "mvapp.h"
#include "mv_pp2.h"
#include "mv_pp2_hif.h"
#include "mv_pp2_bpool.h"
#include "mv_pp2_ppio.h"

#include "pp2_utils.h"


/* #define HW_BUFF_RECYLCE */
/* #define PORTS_LOOPBACK */
#define APP_TX_RETRY


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

#define PKT_ECHO_APP_MAX_BURST_SIZE		((PKT_ECHO_APP_RX_Q_SIZE) >> 1)
#define PKT_ECHO_APP_DFLT_BURST_SIZE		256

#define PKT_ECHO_APP_DMA_MEM_SIZE		(40 * 1024 * 1024)
#define PKT_ECHO_APP_CTRL_DFLT_THR		1000

#define PKT_ECHO_APP_FIRST_INQ			0
#define PKT_ECHO_APP_MAX_NUM_TCS_PER_PORT	1
#define PKT_ECHO_APP_MAX_NUM_QS_PER_CORE	PKT_ECHO_APP_MAX_NUM_TCS_PER_PORT

/* #define PKT_ECHO_APP_HW_TX_CHKSUM_CALC */
#ifdef PKT_ECHO_APP_HW_TX_CHKSUM_CALC
#define PKT_ECHO_APP_HW_TX_CHKSUM_CALC		1
#define PKT_ECHO_APP_HW_TX_IPV4_CHKSUM_CALC	1
#endif
#define PKT_ECHO_APP_PKT_ECHO_SUPPORT
#define PKT_ECHO_APP_USE_PREFETCH
#define PKT_ECHO_APP_PREFETCH_SHIFT		7

#define PKT_ECHO_APP_BPOOLS_INF		{ {384, 4096}, {2048, 1024} }
#define PKT_ECHO_APP_BPOOLS_JUMBO_INF	{ {2048, 4096}, {10240, 512} }

struct glob_arg {
	struct glb_common_args	cmn_args; /* Keep first */

	u16				rxq_size;
	u32				busy_wait;
	int				multi_buffer_release;
	int				loopback;
	int				maintain_stats;
	pthread_mutex_t			trd_lock;
};

struct local_arg {
	struct local_common_args	cmn_args; /* Keep first */

	u32				busy_wait;
	int				multi_buffer_release;
};

static struct glob_arg garg = {};



#ifdef HW_BUFF_RECYLCE
static inline void free_buffers(struct local_arg	*larg,
				struct pp2_ppio_desc	*descs,
				u16			num,
				u8			ppio_id)
{
	int i;
	struct pp2_buff_inf binf;

	for (i = 0; i < num; i++) {
		struct pp2_bpool *bpool = pp2_ppio_inq_desc_get_bpool(&descs[i],
								      larg->pp2_args.lcl_ports_desc[ppio_id].ppio);

		binf.addr = pp2_ppio_inq_desc_get_phys_addr(&descs[i]);
		binf.cookie = pp2_ppio_inq_desc_get_cookie(&descs[i]);
		pp2_bpool_put_buff(larg->pp2_args.hif, bpool, &binf);
	}
	INC_FREE_COUNT(&larg->pp2_args.lcl_ports_desc[ppio_id], num);
}
#endif


#ifdef HW_BUFF_RECYLCE
static inline int loop_hw_recycle(struct local_arg	*larg,
				  u8			 rx_ppio_id,
				  u8			 tx_ppio_id,
				  u8			 tc,
				  u8			 qid,
				  u16			 num)
{
	struct pp2_ppio_desc	 descs[PKT_ECHO_APP_MAX_BURST_SIZE];
	struct pp2_lcl_common_args *pp2_args = (struct pp2_lcl_common_args *) larg->cmn_args.plat;
	struct perf_cmn_cntrs	*perf_cntrs = &larg->cmn_args.perf_cntrs;
	u16			 i, tx_num;
#ifdef APP_TX_RETRY
	u16			 desc_idx = 0, cnt = 0;
#endif
#ifdef PKT_ECHO_APP_PKT_ECHO_SUPPORT
	int			 prefetch_shift = larg->cmn_args.prefetch_shift;
#endif /* PKT_ECHO_APP_PKT_ECHO_SUPPORT */
	struct lcl_port_desc	*lcl_port_desc = &(pp2_args->lcl_ports_desc[rx_ppio_id]);
	u16 pkt_offset = MVAPPS_PP2_PKT_EFEC_OFFS(lcl_port_desc->pkt_offset[tc]);

	pp2_ppio_recv(lcl_port_desc->ppio, tc, qid, descs, &num);
	perf_cntrs->rx_cnt += num;
	INC_RX_COUNT(lcl_port_desc->ppio, num);

	for (i = 0; i < num; i++) {
		char		*buff = (char *)(uintptr_t)pp2_ppio_inq_desc_get_cookie(&descs[i]);
		dma_addr_t	 pa = pp2_ppio_inq_desc_get_phys_addr(&descs[i]);
		u16 len = pp2_ppio_inq_desc_get_pkt_len(&descs[i]);
		struct pp2_bpool *bpool = pp2_ppio_inq_desc_get_bpool(&descs[i], lcl_port_desc->ppio);

#ifdef PKT_ECHO_APP_PKT_ECHO_SUPPORT
		if (likely(larg->cmn_args.echo)) {
			char *tmp_buff;
#ifdef PKT_ECHO_APP_USE_PREFETCH
			if (num - i > prefetch_shift) {
				tmp_buff = (char *)(uintptr_t)pp2_ppio_inq_desc_get_cookie(&descs[i + prefetch_shift]);
				tmp_buff += pkt_offset;
				prefetch(tmp_buff);
			}
#endif /* PKT_ECHO_APP_USE_PREFETCH */
			tmp_buff = buff;
			pr_debug("buff(%p)\n", tmp_buff);
			tmp_buff += pkt_offset;
			swap_l2(tmp_buff);
			swap_l3(tmp_buff);
		}
#endif /* PKT_ECHO_APP_PKT_ECHO_SUPPORT */

		pp2_ppio_outq_desc_reset(&descs[i]);
		pp2_ppio_outq_desc_set_phys_addr(&descs[i], pa);
		pp2_ppio_outq_desc_set_pkt_offset(&descs[i], pkt_offset);
		pp2_ppio_outq_desc_set_pkt_len(&descs[i], len);
		pp2_ppio_outq_desc_set_cookie(&descs[i], (uintptr_t)(buff));
		pp2_ppio_outq_desc_set_pool(&descs[i], bpool);
	}

	SET_MAX_BURST(lcl_port_desc, num);
#ifdef APP_TX_RETRY
	while (num) {
		tx_num = num;
		pp2_ppio_send(pp2_args->lcl_ports_desc[tx_ppio_id].ppio, pp2_args->hif,
			      tc, &descs[desc_idx], &tx_num);

		if (num > tx_num) {
			if (!cnt)
				INC_TX_RETRY_COUNT(lcl_port_desc, num - tx_num);
			cnt++;
			usleep(PKT_ECHO_APP_TX_RETRY_WAIT);
		}
		desc_idx += tx_num;
		num -= tx_num;
		INC_TX_COUNT(lcl_port_desc, tx_num);
		perf_cntrs->drop_cnt += tx_num;
	}
	SET_MAX_RESENT(lcl_port_desc, cnt);
#else
	if (num) {
		tx_num = num;
		pp2_ppio_send(pp2_args->lcl_ports_desc[tx_ppio_id].ppio, pp2_args->hif, tc, descs, &tx_num);
		if (num > tx_num) {
			free_buffers(larg, &descs[tx_num], num - tx_num, rx_ppio_id);
			INC_TX_DROP_COUNT(lcl_port_desc, num - tx_num);
			perf_cntrs->drop_cnt += (num - tx_num);
		}
		INC_TX_COUNT(lcl_port_desc, tx_num);
		perf_cntrs->tx_cnt += tx_num;
	}
#endif /* APP_TX_RETRY */
	return 0;
}

#else
static inline int loop_sw_recycle(struct local_arg	*larg,
				  u8			 rx_ppio_id,
				  u8			 tx_ppio_id,
				  u8			 tc,
				  u8			 qid,
				  u16			 num)
{
	struct tx_shadow_q	*shadow_q;
	int			 shadow_q_size;
	struct pp2_ppio_desc	 descs[PKT_ECHO_APP_MAX_BURST_SIZE];
	struct pp2_lcl_common_args *pp2_args = (struct pp2_lcl_common_args *) larg->cmn_args.plat;
	struct perf_cmn_cntrs	*perf_cntrs = &larg->cmn_args.perf_cntrs;
	u16			 i, j, tx_num;
	int			 mycyc;
#ifdef APP_TX_RETRY
	u16			 desc_idx = 0, cnt = 0;
#endif

#ifdef PKT_ECHO_APP_PKT_ECHO_SUPPORT
	int			 prefetch_shift = larg->cmn_args.prefetch_shift;
#endif /* PKT_ECHO_APP_PKT_ECHO_SUPPORT */
#ifdef PKT_ECHO_APP_HW_TX_CHKSUM_CALC
	enum pp2_inq_l3_type     l3_type;
	enum pp2_inq_l4_type     l4_type;
	u8                       l3_offset, l4_offset;
#endif /* PKT_ECHO_APP_HW_TX_CHKSUM_CALC */
	struct lcl_port_desc	*lcl_port_desc = &(pp2_args->lcl_ports_desc[rx_ppio_id]);
	u16 pkt_offset = MVAPPS_PP2_PKT_EFEC_OFFS(lcl_port_desc->pkt_offset[tc]);


	shadow_q = &pp2_args->lcl_ports_desc[tx_ppio_id].shadow_qs[tc];
	shadow_q_size = pp2_args->lcl_ports_desc[tx_ppio_id].shadow_q_size;

	pp2_ppio_recv(lcl_port_desc->ppio, tc, qid, descs, &num);
	perf_cntrs->rx_cnt += num;
	INC_RX_COUNT(lcl_port_desc, num);

	for (i = 0; i < num; i++) {
		char		*buff = (char *)(uintptr_t)pp2_ppio_inq_desc_get_cookie(&descs[i]);
		dma_addr_t	 pa = pp2_ppio_inq_desc_get_phys_addr(&descs[i]);
		u16 len = pp2_ppio_inq_desc_get_pkt_len(&descs[i]);
		struct pp2_bpool *bpool = pp2_ppio_inq_desc_get_bpool(&descs[i], lcl_port_desc->ppio);

#ifdef PKT_ECHO_APP_PKT_ECHO_SUPPORT
		if (likely(larg->cmn_args.echo)) {
			char *tmp_buff;
#ifdef PKT_ECHO_APP_USE_PREFETCH
			if (num - i > prefetch_shift) {
				tmp_buff = (char *)(uintptr_t)pp2_ppio_inq_desc_get_cookie(&descs[i + prefetch_shift]);
				tmp_buff += pkt_offset;
				prefetch(tmp_buff);
			}
#endif /* PKT_ECHO_APP_USE_PREFETCH */
			tmp_buff = buff;
			pr_debug("buff(%p)\n", tmp_buff);
			tmp_buff += pkt_offset;
			swap_l2(tmp_buff);
			swap_l3(tmp_buff);
		}
#endif /* PKT_ECHO_APP_PKT_ECHO_SUPPORT */
#ifdef PKT_ECHO_APP_HW_TX_CHKSUM_CALC
		pp2_ppio_inq_desc_get_l3_info(&descs[i], &l3_type, &l3_offset);
		pp2_ppio_inq_desc_get_l4_info(&descs[i], &l4_type, &l4_offset);
#endif /* PKT_ECHO_APP_HW_TX_CHKSUM_CALC */

		pp2_ppio_outq_desc_reset(&descs[i]);
#ifdef PKT_ECHO_APP_HW_TX_CHKSUM_CALC
#if (PKT_ECHO_APP_HW_TX_IPV4_CHKSUM_CALC || PKT_ECHO_APP_HW_TX_CHKSUM_CALC)
		pp2_ppio_outq_desc_set_proto_info(&descs[i], pp2_l3_type_inq_to_outq(l3_type),
						  pp2_l4_type_inq_to_outq(l4_type), l3_offset,
						  l4_offset, PKT_ECHO_APP_HW_TX_IPV4_CHKSUM_CALC,
						  PKT_ECHO_APP_HW_TX_CHKSUM_CALC);
#endif /* (PKT_ECHO_APP_HW_TX_IPV4_CHKSUM_CALC ||  ... */
#endif /* PKT_ECHO_APP_HW_TX_CHKSUM_CALC */
		pp2_ppio_outq_desc_set_phys_addr(&descs[i], pa);
		pp2_ppio_outq_desc_set_pkt_offset(&descs[i], pkt_offset);
		pp2_ppio_outq_desc_set_pkt_len(&descs[i], len);
		shadow_q->ents[shadow_q->write_ind].buff_ptr.cookie = (uintptr_t)buff;
		shadow_q->ents[shadow_q->write_ind].buff_ptr.addr = pa;
		shadow_q->ents[shadow_q->write_ind].bpool = bpool;
		pr_debug("buff_ptr.cookie(0x%lx)\n", (u64)shadow_q->ents[shadow_q->write_ind].buff_ptr.cookie);
		shadow_q->write_ind++;
		if (shadow_q->write_ind == shadow_q_size)
			shadow_q->write_ind = 0;

		/* Below condition should never happen, if shadow_q size is large enough */
		if (unlikely(shadow_q->write_ind == shadow_q->read_ind)) {
			pr_err("%s: port(%d), tc(%d), shadow_q size=%d is too small, performing emergency drops\n",
				__func__, tx_ppio_id, tc, shadow_q_size);
			/* Drop all following packets, and also this packet */
			for (j = i; j < num; j++) {
				struct pp2_buff_inf binf;

				binf.cookie = pp2_ppio_inq_desc_get_cookie(&descs[j]);
				binf.addr = pp2_ppio_inq_desc_get_phys_addr(&descs[j]);
				pp2_bpool_put_buff(pp2_args->hif, bpool, &binf);
			}
			/* Rollback write_index by 1 */
			if (shadow_q->write_ind > 0)
				shadow_q->write_ind--;
			else
				shadow_q->write_ind = shadow_q_size - 1;
			/* Update num of packets that may be sent */
			num = i;
			break;
		}

	}
	SET_MAX_BURST(lcl_port_desc, num);
	for (mycyc = 0; mycyc < larg->busy_wait; mycyc++)
		asm volatile("");
#ifdef APP_TX_RETRY
	do {
		tx_num = num;
		if (num) {
			pp2_ppio_send(pp2_args->lcl_ports_desc[tx_ppio_id].ppio, pp2_args->hif, tc,
				      &descs[desc_idx], &tx_num);
			if (num > tx_num) {
				if (!cnt)
					INC_TX_RETRY_COUNT(lcl_port_desc, num - tx_num);
				cnt++;
			}
			desc_idx += tx_num;
			num -= tx_num;
			INC_TX_COUNT(lcl_port_desc, tx_num);
			perf_cntrs->tx_cnt += tx_num;
		}
		free_sent_buffers(lcl_port_desc, &pp2_args->lcl_ports_desc[tx_ppio_id], pp2_args->hif,
				  tc, larg->multi_buffer_release);
	} while (num);
	SET_MAX_RESENT(lcl_port_desc, cnt);
#else
	if (num) {
		tx_num = num;
		pp2_ppio_send(pp2_args->lcl_ports_desc[tx_ppio_id].ppio, pp2_args->hif, tc, descs, &tx_num);
		if (num > tx_num) {
			u16 not_sent = num - tx_num;
			/* Free not sent buffers */
			shadow_q->write_ind = (shadow_q->write_ind < not_sent) ?
						(shadow_q_size - not_sent + shadow_q->write_ind) :
						shadow_q->write_ind - not_sent;
			free_buffers(lcl_port_desc,
				     &pp2_args->lcl_ports_desc[tx_ppio_id], pp2_args->hif,
				     shadow_q->write_ind, not_sent, tc);
			INC_TX_DROP_COUNT(lcl_port_desc, not_sent);
			perf_cntrs->drop_cnt += not_sent;
		}
		INC_TX_COUNT(lcl_port_desc, tx_num);
		perf_cntrs->tx_cnt += tx_num;
	}
	free_sent_buffers(lcl_port_desc, &pp2_args->lcl_ports_desc[tx_ppio_id], pp2_args->hif, tc,
			  larg->multi_buffer_release);
#endif /* APP_TX_RETRY */

	return 0;
}
#endif /* HW_BUFF_RECYLCE */

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
			if (qid == MVAPPS_PP2_MAX_NUM_QS_PER_TC) {
				qid = 0;
				tc++;
				if (tc == PKT_ECHO_APP_MAX_NUM_TCS_PER_PORT)
					tc = 0;
			}
		} while (!(larg->cmn_args.qs_map & (1 << ((tc * MVAPPS_PP2_MAX_NUM_QS_PER_TC) + qid))));

#ifdef HW_BUFF_RECYLCE
		err = loop_hw_recycle(larg, 0, 0, tc, qid, num);
#else
		err = loop_sw_recycle(larg, 0, 0, tc, qid, num);
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
	u8			 tc = 0, qid = 0;
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
		qid = larg->cmn_args.id & 1;
		pr_debug("loop_2ps: for cpu %d (%d), port %d, queue %d\n", larg->cmn_args.id,
			 larg->cmn_args.garg->cmn_args.cpus, port_id, qid);
	}
#endif

	num = larg->cmn_args.burst;
	while (*running) {
#ifndef PORTS_LOOPBACK
		/* Find next queue to consume */
		do {
			qid++;
			if (qid == MVAPPS_PP2_MAX_NUM_QS_PER_TC) {
				qid = 0;
				tc++;
				if (tc == PKT_ECHO_APP_MAX_NUM_TCS_PER_PORT)
					tc = 0;
			}
		} while (!(larg->cmn_args.qs_map & (1 << ((tc * MVAPPS_PP2_MAX_NUM_QS_PER_TC) + qid))));
#endif

#ifdef HW_BUFF_RECYLCE
#ifdef PORTS_LOOPBACK
		if (larg->cmn_args.garg->cmn_args.cpus == 4) {
			/* Use port_id and queue calculated in beginning of this function */
			err  = loop_hw_recycle(larg, port_id, port_id, tc, qid, num);
		} else {
			/* Set larg->cmn_args.id as rx_ppio, tx_ppio, for quick 2xloopback implementation
			 * and queue 0 only
			 */
			err  = loop_hw_recycle(larg, larg->cmn_args.id, larg->cmn_args.id, tc, 0, num);
		}
#else
		err  = loop_hw_recycle(larg, 0, 1, tc, qid, num);
		err |= loop_hw_recycle(larg, 1, 0, tc, qid, num);
#endif /* PORTS_LOOPBACK */
#else
#ifdef PORTS_LOOPBACK

		if (larg->cmn_args.garg->cmn_args.cpus == 4) {
			/* Use port_id and queue calculated in beginning of this function */
			err  = loop_sw_recycle(larg, port_id, port_id, tc, qid, num);
		} else {
			/* Set larg->cmn_args.id as rx_ppio, tx_ppio, for quick 2xloopback implementation
			 * and queue 0 only
			 */
			err  = loop_sw_recycle(larg, larg->cmn_args.id, larg->cmn_args.id, tc, qid, num);
		}
#else
		err  = loop_sw_recycle(larg, 0, 1, tc, qid, num);
		err |= loop_sw_recycle(larg, 1, 0, tc, qid, num);
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

	if (larg->cmn_args.num_ports == 1)
		return loop_1p(larg, running);
	return loop_2ps(larg, running);
}

static int init_all_modules(void)
{
	struct pp2_init_params	 pp2_params;
	struct pp2_glb_common_args *pp2_args = (struct pp2_glb_common_args *)garg.cmn_args.plat;
	int			 err;
	char			 file[PP2_MAX_BUF_STR_LEN];
	int			 num_rss_tables = 0;

	pr_info("Global initializations ...\n");

	err = mv_sys_dma_mem_init(PKT_ECHO_APP_DMA_MEM_SIZE);
	if (err)
		return err;

	memset(&pp2_params, 0, sizeof(pp2_params));
	pp2_params.hif_reserved_map = MVAPPS_PP2_HIFS_RSRV;
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

static int init_local_modules(struct glob_arg *garg)
{
	int				err, port_index;
	struct bpool_inf		std_infs[] = PKT_ECHO_APP_BPOOLS_INF;
	struct bpool_inf		jumbo_infs[] = PKT_ECHO_APP_BPOOLS_JUMBO_INF;
	struct bpool_inf		*infs;
	struct pp2_glb_common_args *pp2_args = (struct pp2_glb_common_args *) garg->cmn_args.plat;
	int				i;

	pr_info("Local initializations ...\n");

	err = app_hif_init(&pp2_args->hif, PKT_ECHO_APP_HIF_Q_SIZE);
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
			port->num_tcs	= PKT_ECHO_APP_MAX_NUM_TCS_PER_PORT;
			for (i = 0; i < port->num_tcs; i++)
				port->num_inqs[i] =  garg->cmn_args.cpus;
			port->inq_size	= garg->rxq_size;
			port->num_outqs	= PKT_ECHO_APP_MAX_NUM_TCS_PER_PORT;
			port->outq_size	= PKT_ECHO_APP_TX_Q_SIZE;
			port->first_inq	= PKT_ECHO_APP_FIRST_INQ;
			if (garg->cmn_args.cpus == 1)
				port->hash_type = PP2_PPIO_HASH_T_NONE;
			else
				port->hash_type = PP2_PPIO_HASH_T_2_TUPLE;

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
	struct pp2_glb_common_args *pp2_args = (struct pp2_glb_common_args *) garg->cmn_args.plat;


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
	struct pp2_glb_common_args *glb_pp2_args = (struct pp2_glb_common_args *) garg->cmn_args.plat;
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

	pthread_mutex_lock(&garg->trd_lock);
	err = app_hif_init(&lcl_pp2_args->hif, PKT_ECHO_APP_HIF_Q_SIZE);
	pthread_mutex_unlock(&garg->trd_lock);
	if (err)
		return err;

	larg->cmn_args.id                = id;
	larg->cmn_args.burst		= garg->cmn_args.burst;
	larg->busy_wait		= garg->busy_wait;
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

	larg->cmn_args.qs_map = garg->cmn_args.qs_map << (garg->cmn_args.qs_map_shift * id);

	/* Update garg refs to local */
	garg->cmn_args.largs[id] = larg;
	for (i = 0; i < larg->cmn_args.num_ports; i++)
		glb_pp2_args->ports_desc[i].lcl_ports_desc[id] = lcl_pp2_args->lcl_ports_desc;

	pr_debug("thread %d (cpu %d) mapped to Qs %llx\n",
		 larg->cmn_args.id, sched_getcpu(), (unsigned long long)larg->cmn_args.qs_map);

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
	       "\n"
	       "Optional OPTIONS:\n"
	       "\t-b <size>                Burst size, num_pkts handled in a batch.(default is %d)\n"
	       "\t--mtu <mtu>              Set MTU (default is %d)\n"
	       "\t-c, --cores <number>     Number of CPUs to use\n"
	       "\t-a, --affinity <number>  Use setaffinity (default is no affinity)\n"
	       "\t-s                       Maintain statistics\n"
	       "\t-w <cycles>              Cycles to busy_wait between recv&send, simulating app behavior (default=0)\n"
	       "\t--rxq <size>             Size of rx_queue (default is %d)\n"
	       "\t--pkt-offset <size>      Packet offset in buffer, must be multiple of 32-byte (default is %d)\n"
	       "\t--old-tx-desc-release    Use pp2_bpool_put_buff(), instead of NEW pp2_bpool_put_buffs() API\n"
	       "\t--no-echo                Don't perform 'pkt_echo', N/A w/o define PKT_ECHO_APP_PKT_ECHO_SUPPORT\n"
	       "\t--cli                    Use CLI\n"
	       "\t?, -h, --help            Display help and exit.\n\n"
	       "\n", MVAPPS_NO_PATH(progname), MVAPPS_NO_PATH(progname),
	       MVAPPS_PP2_MAX_NUM_PORTS, PKT_ECHO_APP_MAX_BURST_SIZE, DEFAULT_MTU, PKT_ECHO_APP_RX_Q_SIZE,
	       MVAPPS_PP2_PKT_DEF_OFFS);
}

static int parse_args(struct glob_arg *garg, int argc, char *argv[])
{
	int	i = 1;
	struct pp2_glb_common_args *pp2_args = (struct pp2_glb_common_args *) garg->cmn_args.plat;


	garg->cmn_args.cli = 0;
	garg->cmn_args.cpus = 1;
	garg->cmn_args.affinity = -1;
	garg->cmn_args.burst = PKT_ECHO_APP_DFLT_BURST_SIZE;
	garg->cmn_args.mtu = DEFAULT_MTU;
	garg->busy_wait	= 0;
	garg->rxq_size = PKT_ECHO_APP_RX_Q_SIZE;
	garg->multi_buffer_release = 1;
	garg->cmn_args.echo = 1;
	garg->cmn_args.qs_map = 0;
	garg->cmn_args.qs_map_shift = 0;
	garg->cmn_args.pkt_offset = 0;
	garg->cmn_args.prefetch_shift = PKT_ECHO_APP_PREFETCH_SHIFT;
	garg->cmn_args.ctrl_thresh = PKT_ECHO_APP_CTRL_DFLT_THR;
	garg->maintain_stats = 0;


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
				snprintf(pp2_args->ports_desc[garg->cmn_args.num_ports].name,
					 sizeof(pp2_args->ports_desc[garg->cmn_args.num_ports].name),
					 "%s", token);

			if (garg->cmn_args.num_ports == 0) {
				pr_err("Invalid interface arguments format!\n");
				return -EINVAL;
			} else if (garg->cmn_args.num_ports > MVAPPS_PP2_MAX_NUM_PORTS) {
				pr_err("too many ports specified (%d vs %d)\n",
				       garg->cmn_args.num_ports, MVAPPS_PP2_MAX_NUM_PORTS);
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
			garg->busy_wait = atoi(argv[i + 1]);
			i += 2;
		} else if (strcmp(argv[i], "--rxq") == 0) {
			garg->rxq_size = atoi(argv[i + 1]);
			i += 2;
		} else if (strcmp(argv[i], "--pkt-offset") == 0) {
			garg->cmn_args.pkt_offset = atoi(argv[i + 1]);
			i += 2;
		} else if (strcmp(argv[i], "--old-tx-desc-release") == 0) {
			garg->multi_buffer_release = 0;
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
	if (garg->cmn_args.burst > PKT_ECHO_APP_MAX_BURST_SIZE) {
		pr_err("illegal burst size requested (%d vs %d)!\n",
		       garg->cmn_args.burst, PKT_ECHO_APP_MAX_BURST_SIZE);
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
	u64			cores_mask;
	struct pp2_glb_common_args *pp2_args;

	int			i, err;

	setbuf(stdout, NULL);
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
	mvapp_params.main_loop_cb	= main_loop;
	if (!mvapp_params.use_cli)
		mvapp_params.ctrl_cb	= app_ctrl_cb;

	return mvapp_go(&mvapp_params);
}
