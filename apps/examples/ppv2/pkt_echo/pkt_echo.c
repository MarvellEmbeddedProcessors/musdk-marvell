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
#define SHOW_STATISTICS

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

#define PKT_ECHO_APP_FIRST_INQ			0
#define PKT_ECHO_APP_MAX_NUM_TCS_PER_PORT	1
#define PKT_ECHO_APP_MAX_NUM_QS_PER_CORE	PKT_ECHO_APP_MAX_NUM_TCS_PER_PORT

/* #define  PKT_ECHO_APP_HW_TX_CHKSUM_CALC */
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
	int			 verbose;
	int			 cli;
	int			 cpus;	/* cpus used for running */
	u16			 burst;
	u16			 mtu;
	u16			 rxq_size;
	u32			 busy_wait;
	int			 multi_buffer_release;
	int			 affinity;
	int			 loopback;
	int			 echo;
	int			 maintain_stats;
	u64			 qs_map;
	int			 qs_map_shift;
	int			 prefetch_shift;
	int			 num_ports;
	int			 pp2_num_inst;
	struct port_desc	 ports_desc[MVAPPS_PP2_MAX_NUM_PORTS];

	pthread_mutex_t		 trd_lock;

	struct pp2_hif		*hif;

	int			 num_pools;
	struct bpool_desc	**pools_desc;
};

struct local_arg {
	int			 prefetch_shift;
	u64			 qs_map;

	struct pp2_hif		*hif;
	int			 num_ports;
	struct lcl_port_desc	*ports_desc;

	struct bpool_desc	**pools_desc;

	u16			 burst;
	u32			 busy_wait;
	int			 echo;
	int			 id;
	int			 multi_buffer_release;

	struct glob_arg		*garg;
};

static struct glob_arg garg = {};

#ifdef SHOW_STATISTICS
#define INC_RX_COUNT(core, port, cnt)		(rx_buf_cnt[core][port] += cnt)
#define INC_TX_COUNT(core, port, cnt)		(tx_buf_cnt[core][port] += cnt)
#define INC_TX_RETRY_COUNT(core, port, cnt)	(tx_buf_retry[core][port] += cnt)
#define INC_TX_DROP_COUNT(core, port, cnt)	(tx_buf_drop[core][port] += cnt)
#define INC_FREE_COUNT(core, port, cnt)		(free_buf_cnt[core][port] += cnt)
#define SET_MAX_RESENT(core, port, cnt)		\
	{ if (cnt > tx_max_resend[core][port]) tx_max_resend[core][port] = cnt; }

#define SET_MAX_BURST(core, port, burst)	\
	{ if (burst > tx_max_burst[core][port]) tx_max_burst[core][port] = burst; }

u32 rx_buf_cnt[MVAPPS_PP2_MAX_NUM_CORES][MVAPPS_PP2_MAX_NUM_PORTS];
u32 free_buf_cnt[MVAPPS_PP2_MAX_NUM_CORES][MVAPPS_PP2_MAX_NUM_PORTS];
u32 tx_buf_cnt[MVAPPS_PP2_MAX_NUM_CORES][MVAPPS_PP2_MAX_NUM_PORTS];
u32 tx_buf_drop[MVAPPS_PP2_MAX_NUM_CORES][MVAPPS_PP2_MAX_NUM_PORTS];
u32 tx_buf_retry[MVAPPS_PP2_MAX_NUM_CORES][MVAPPS_PP2_MAX_NUM_PORTS];
u32 tx_max_resend[MVAPPS_PP2_MAX_NUM_CORES][MVAPPS_PP2_MAX_NUM_PORTS];
u32 tx_max_burst[MVAPPS_PP2_MAX_NUM_CORES][MVAPPS_PP2_MAX_NUM_PORTS];

#else
#define INC_RX_COUNT(core, port, cnt)
#define INC_TX_COUNT(core, port, cnt)
#define INC_TX_RETRY_COUNT(core, port, cnt)
#define INC_TX_DROP_COUNT(core, port, cnt)
#define INC_FREE_COUNT(core, port, cnt)
#define SET_MAX_RESENT(core, port, cnt)
#define SET_MAX_BURST(core, port, cnt)
#endif /* SHOW_STATISTICS */

#ifdef PKT_ECHO_APP_PKT_ECHO_SUPPORT
#ifdef PKT_ECHO_APP_USE_PREFETCH
static inline void prefetch(const void *ptr)
{
	asm volatile("prfm pldl1keep, %a0\n" : : "p" (ptr));
}
#endif /* PKT_ECHO_APP_USE_PREFETCH */

#endif /* PKT_ECHO_APP_PKT_ECHO_SUPPORT */

#ifdef PKT_ECHO_APP_HW_TX_CHKSUM_CALC
static inline enum mv_outq_l3_type pp2_l3_type_inq_to_outq(enum mv_inq_l3_type l3_inq)
{
	if (unlikely(l3_inq & MV_INQ_L3_TYPE_IPV6_NO_EXT))
		return MV_OUTQ_L3_TYPE_IPV6;

	if (likely(l3_inq != MV_INQ_L3_TYPE_NA))
		return MV_OUTQ_L3_TYPE_IPV4;

	return MV_OUTQ_L3_TYPE_OTHER;
}

static inline enum mv_outq_l4_type pp2_l4_type_inq_to_outq(enum mv_inq_l4_type l4_inq)
{
	if (likely(l4_inq == MV_INQ_L4_TYPE_TCP || l4_inq == MV_INQ_L4_TYPE_UDP))
		return (l4_inq - 1);

	return MV_OUTQ_L4_TYPE_OTHER;
}
#endif

#ifdef HW_BUFF_RECYLCE
static inline void free_buffers(struct local_arg	*larg,
				struct pp2_ppio_desc	*descs,
				u16			num,
				u8			ppio_id)
{
	int i;
	struct pp2_buff_inf binf;

	for (i = 0; i < num; i++) {
		struct pp2_bpool *bpool = pp2_ppio_inq_desc_get_bpool(&descs[i], larg->ports_desc[ppio_id].ppio);

		binf.addr = pp2_ppio_inq_desc_get_phys_addr(&descs[i]);
		binf.cookie = pp2_ppio_inq_desc_get_cookie(&descs[i]);
		pp2_bpool_put_buff(larg->hif, bpool, &binf);
	}
	INC_FREE_COUNT(larg->id, ppio_id, num);
}
#else
static inline u16 free_buffers(struct lcl_port_desc	*rx_port,
			       struct lcl_port_desc	*tx_port,
			       struct pp2_hif		*hif,
			       u16			 start_idx,
			       u16			 num,
			       u8			 tc)
{
	u16			i, free_cnt = 0, idx = start_idx;
	struct pp2_buff_inf	*binf;
	struct tx_shadow_q	*shadow_q;

	shadow_q = &tx_port->shadow_qs[tc];

	for (i = 0; i < num; i++) {
		struct pp2_bpool *bpool = shadow_q->ents[idx].bpool;

		binf = &shadow_q->ents[idx].buff_ptr;
		if (unlikely(!binf->cookie || !binf->addr || !bpool)) {
			pr_warn("Shadow memory @%d: cookie(%lx), pa(%lx), pool(%lx)!\n",
				i, (u64)binf->cookie, (u64)binf->addr, (u64)bpool);
			continue;
		}
		pp2_bpool_put_buff(hif, bpool, binf);
		free_cnt++;

		if (++idx == tx_port->shadow_q_size)
			idx = 0;
	}

	INC_FREE_COUNT(rx_port->lcl_id, rx_port->id, free_cnt);
	return idx;
}

static inline u16 free_multi_buffers(struct lcl_port_desc	*rx_port,
				     struct lcl_port_desc	*tx_port,
				     struct pp2_hif		*hif,
				     u16			 start_idx,
				     u16			 num,
				     u8			 tc)
{
	u16			idx = start_idx;
	u16			cont_in_shadow, req_num;
	struct tx_shadow_q	*shadow_q;

	shadow_q = &tx_port->shadow_qs[tc];

	cont_in_shadow = tx_port->shadow_q_size - start_idx;

	if (num <= cont_in_shadow) {
		req_num = num;
		pp2_bpool_put_buffs(hif, (struct buff_release_entry *)&shadow_q->ents[idx], &req_num);
		idx = idx + num;
		if (idx == tx_port->shadow_q_size)
			idx = 0;
	} else {
		req_num = cont_in_shadow;
		pp2_bpool_put_buffs(hif, (struct buff_release_entry *)&shadow_q->ents[idx], &req_num);

		req_num = num - cont_in_shadow;
		pp2_bpool_put_buffs(hif, (struct buff_release_entry *)&shadow_q->ents[0], &req_num);
		idx = num - cont_in_shadow;
	}

	INC_FREE_COUNT(rx_port->lcl_id, rx_port->id, num);

	return idx;
}

static inline void free_sent_buffers(struct lcl_port_desc	*rx_port,
				     struct lcl_port_desc	*tx_port,
				     struct pp2_hif		*hif,
				     u8				 tc,
				     int			 multi_buffer_release)
{
	u16 tx_num;

	pp2_ppio_get_num_outq_done(tx_port->ppio, hif, tc, &tx_num);

	if (multi_buffer_release)
		tx_port->shadow_qs[tc].read_ind = free_multi_buffers(rx_port, tx_port, hif,
								     tx_port->shadow_qs[tc].read_ind, tx_num, tc);
	else
		tx_port->shadow_qs[tc].read_ind = free_buffers(rx_port, tx_port, hif,
							       tx_port->shadow_qs[tc].read_ind, tx_num, tc);
}

#endif /* HW_BUFF_RECYLCE */

#ifdef HW_BUFF_RECYLCE
static inline int loop_hw_recycle(struct local_arg	*larg,
				  u8			 rx_ppio_id,
				  u8			 tx_ppio_id,
				  u8			 tc,
				  u8			 qid,
				  u16			 num)
{
	struct pp2_ppio_desc	 descs[PKT_ECHO_APP_MAX_BURST_SIZE];
	u16			 i, tx_num;
#ifdef APP_TX_RETRY
	u16			 desc_idx = 0, cnt = 0;
#endif
#ifdef PKT_ECHO_APP_PKT_ECHO_SUPPORT
	int			 prefetch_shift = larg->prefetch_shift;
#endif /* PKT_ECHO_APP_PKT_ECHO_SUPPORT */

/* pr_info("tid %d check on tc %d, qid %d\n", larg->id, tc, qid); */
/* pthread_mutex_lock(&larg->garg->trd_lock); */
	pp2_ppio_recv(larg->ports_desc[rx_ppio_id].ppio, tc, qid, descs, &num);
/* pthread_mutex_unlock(&larg->garg->trd_lock); */
/* if (num) pr_info("got %d pkts on thd %d, tc %d, qid %d\n", num, larg->id, tc, qid); */
	INC_RX_COUNT(larg->id, rx_ppio_id, num);

	for (i = 0; i < num; i++) {
		char		*buff = (char *)(uintptr_t)pp2_ppio_inq_desc_get_cookie(&descs[i]);
		dma_addr_t	 pa = pp2_ppio_inq_desc_get_phys_addr(&descs[i]);
		u16 len = pp2_ppio_inq_desc_get_pkt_len(&descs[i]);
		struct pp2_bpool *bpool = pp2_ppio_inq_desc_get_bpool(&descs[i], larg->ports_desc[rx_ppio_id].ppio);

#ifdef PKT_ECHO_APP_PKT_ECHO_SUPPORT
		if (likely(larg->echo)) {
			char *tmp_buff;
#ifdef PKT_ECHO_APP_USE_PREFETCH
			if (num - i > prefetch_shift) {
				tmp_buff = (char *)(uintptr_t)pp2_ppio_inq_desc_get_cookie(&descs[i + prefetch_shift]);
				tmp_buff += MVAPPS_PP2_PKT_EFEC_OFFS;
				pr_debug("tmp_buff_before(%p)\n", tmp_buff);
				tmp_buff = (char *)(((uintptr_t)tmp_buff) | sys_dma_high_addr);
				pr_debug("tmp_buff_after(%p)\n", tmp_buff);
				prefetch(tmp_buff);
			}
#endif /* PKT_ECHO_APP_USE_PREFETCH */
			tmp_buff = (char *)(((uintptr_t)(buff)) | sys_dma_high_addr);
			pr_debug("buff2(%p)\n", tmp_buff);
			tmp_buff += MVAPPS_PP2_PKT_EFEC_OFFS;
			/* printf("packet:\n"); mem_disp(tmp_buff, len); */
			swap_l2(tmp_buff);
			swap_l3(tmp_buff);
			/* printf("packet:\n"); mem_disp(tmp_buff, len); */
		}
#endif /* PKT_ECHO_APP_PKT_ECHO_SUPPORT */

		pp2_ppio_outq_desc_reset(&descs[i]);
		pp2_ppio_outq_desc_set_phys_addr(&descs[i], pa);
		pp2_ppio_outq_desc_set_pkt_offset(&descs[i], MVAPPS_PP2_PKT_EFEC_OFFS);
		pp2_ppio_outq_desc_set_pkt_len(&descs[i], len);
		pp2_ppio_outq_desc_set_cookie(&descs[i], (uintptr_t)(buff));
		pp2_ppio_outq_desc_set_pool(&descs[i], bpool);
	}

	SET_MAX_BURST(larg->id, rx_ppio_id, num);
#ifdef APP_TX_RETRY
	while (num) {
		tx_num = num;
		pp2_ppio_send(larg->ports_desc[tx_ppio_id].ppio, larg->hif, tc, &descs[desc_idx], &tx_num);

		if (num > tx_num) {
			if (!cnt)
				INC_TX_RETRY_COUNT(larg->id, rx_ppio_id, num - tx_num);
			cnt++;
			usleep(PKT_ECHO_APP_TX_RETRY_WAIT);
		}
		desc_idx += tx_num;
		num -= tx_num;
		INC_TX_COUNT(larg->id, rx_ppio_id, tx_num);
	}
	SET_MAX_RESENT(larg->id, rx_ppio_id, cnt);
#else
	if (num) {
		tx_num = num;
		pp2_ppio_send(larg->ports_desc[tx_ppio_id].ppio, larg->hif, tc, descs, &tx_num);
		if (num > tx_num) {
			free_buffers(larg, &descs[tx_num], num - tx_num, rx_ppio_id);
			INC_TX_DROP_COUNT(larg->id, rx_ppio_id, num - tx_num);
		}
		INC_TX_COUNT(larg->id, rx_ppio_id, tx_num);
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
	u16			 i, tx_num;
	int			 mycyc;
#ifdef APP_TX_RETRY
	u16			 desc_idx = 0, cnt = 0;
#endif

#ifdef PKT_ECHO_APP_PKT_ECHO_SUPPORT
	int			 prefetch_shift = larg->prefetch_shift;
#endif /* PKT_ECHO_APP_PKT_ECHO_SUPPORT */
#ifdef PKT_ECHO_APP_HW_TX_CHKSUM_CALC
	enum mv_inq_l3_type     l3_type;
	enum mv_inq_l4_type     l4_type;
	u8                      l3_offset, l4_offset;
#endif /* PKT_ECHO_APP_HW_TX_CHKSUM_CALC */

	shadow_q = &larg->ports_desc[tx_ppio_id].shadow_qs[tc];
	shadow_q_size = larg->ports_desc[tx_ppio_id].shadow_q_size;

/* pr_info("tid %d check on tc %d, qid %d\n", larg->id, tc, qid); */
/* pthread_mutex_lock(&larg->garg->trd_lock); */
	pp2_ppio_recv(larg->ports_desc[rx_ppio_id].ppio, tc, qid, descs, &num);
/* pthread_mutex_unlock(&larg->garg->trd_lock); */
/* if (num) pr_info("got %d pkts on tc %d, qid %d\n", num, tc, qid); */
	INC_RX_COUNT(larg->id, rx_ppio_id, num);

	for (i = 0; i < num; i++) {
		char		*buff = (char *)(uintptr_t)pp2_ppio_inq_desc_get_cookie(&descs[i]);
		dma_addr_t	 pa = pp2_ppio_inq_desc_get_phys_addr(&descs[i]);
		u16 len = pp2_ppio_inq_desc_get_pkt_len(&descs[i]);
		struct pp2_bpool *bpool = pp2_ppio_inq_desc_get_bpool(&descs[i], larg->ports_desc[rx_ppio_id].ppio);

#ifdef PKT_ECHO_APP_PKT_ECHO_SUPPORT
		if (likely(larg->echo)) {
			char *tmp_buff;
#ifdef PKT_ECHO_APP_USE_PREFETCH
			if (num - i > prefetch_shift) {
				tmp_buff = (char *)(uintptr_t)pp2_ppio_inq_desc_get_cookie(&descs[i + prefetch_shift]);
				tmp_buff += MVAPPS_PP2_PKT_EFEC_OFFS;
				pr_debug("tmp_buff_before(%p)\n", tmp_buff);
				tmp_buff = (char *)(((uintptr_t)tmp_buff) | sys_dma_high_addr);
				pr_debug("tmp_buff_after(%p)\n", tmp_buff);
				prefetch(tmp_buff);
			}
#endif /* PKT_ECHO_APP_USE_PREFETCH */
			tmp_buff = (char *)(((uintptr_t)(buff)) | sys_dma_high_addr);
			pr_debug("buff2(%p)\n", tmp_buff);
			tmp_buff += MVAPPS_PP2_PKT_EFEC_OFFS;
			/* printf("packet:\n"); mem_disp(tmp_buff, len); */
			swap_l2(tmp_buff);
			swap_l3(tmp_buff);
			/* printf("packet:\n"); mem_disp(tmp_buff, len); */
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
		pp2_ppio_outq_desc_set_pkt_offset(&descs[i], MVAPPS_PP2_PKT_EFEC_OFFS);
		pp2_ppio_outq_desc_set_pkt_len(&descs[i], len);
		shadow_q->ents[shadow_q->write_ind].buff_ptr.cookie = (uintptr_t)buff;
		shadow_q->ents[shadow_q->write_ind].buff_ptr.addr = pa;
		shadow_q->ents[shadow_q->write_ind].bpool = bpool;
		pr_debug("buff_ptr.cookie(0x%lx)\n", (u64)shadow_q->ents[shadow_q->write_ind].buff_ptr.cookie);
		shadow_q->write_ind++;
		if (shadow_q->write_ind == shadow_q_size)
			shadow_q->write_ind = 0;
	}
	SET_MAX_BURST(larg->id, rx_ppio_id, num);
	for (mycyc = 0; mycyc < larg->busy_wait; mycyc++)
		asm volatile("");
#ifdef APP_TX_RETRY
	do {
		tx_num = num;
		if (num) {
			pp2_ppio_send(larg->ports_desc[tx_ppio_id].ppio, larg->hif, tc, &descs[desc_idx], &tx_num);
			if (num > tx_num) {
				if (!cnt)
					INC_TX_RETRY_COUNT(larg->id, rx_ppio_id, num - tx_num);
				cnt++;
			}
			desc_idx += tx_num;
			num -= tx_num;
			INC_TX_COUNT(larg->id, rx_ppio_id, tx_num);
		}
		free_sent_buffers(&larg->ports_desc[rx_ppio_id], &larg->ports_desc[tx_ppio_id],
				  larg->hif, tc, larg->multi_buffer_release);
	} while (num);
	SET_MAX_RESENT(larg->id, rx_ppio_id, cnt);
#else
	if (num) {
		tx_num = num;
		pp2_ppio_send(larg->ports_desc[tx_ppio_id].ppio, larg->hif, tc, descs, &tx_num);
		if (num > tx_num) {
			u16 not_sent = num - tx_num;
			/* Free not sent buffers */
			shadow_q->write_ind = (shadow_q->write_ind < not_sent) ?
						(shadow_q_size - not_sent + shadow_q->write_ind) :
						shadow_q->write_ind - not_sent;
			free_buffers(&larg->ports_desc[rx_ppio_id], &larg->ports_desc[tx_ppio_id],
				     larg->hif, shadow_q->write_ind, not_sent, tc);
			INC_TX_DROP_COUNT(larg->id, rx_ppio_id, not_sent);
		}
		INC_TX_COUNT(larg->id, rx_ppio_id, tx_num);
	}
	free_sent_buffers(&larg->ports_desc[rx_ppio_id], &larg->ports_desc[tx_ppio_id], larg->hif,
			  tc, larg->multi_buffer_release);
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

	num = larg->burst;

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
		} while (!(larg->qs_map & (1 << ((tc * MVAPPS_PP2_MAX_NUM_QS_PER_TC) + qid))));

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
	if (larg->garg->cpus == 4) {
		/* For 4 cores: 2 first cores will work with port0, 2 other cores with port1,
		 * 2 first queues will be used for each port
		 */
		port_id = (larg->id < 2) ? 0 : 1;
		qid = larg->id & 1;
		pr_debug("loop_2ps: for cpu %d (%d), port %d, queue %d\n", larg->id, larg->garg->cpus, port_id, qid);
	}
#endif

	num = larg->burst;
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
		} while (!(larg->qs_map & (1 << ((tc * MVAPPS_PP2_MAX_NUM_QS_PER_TC) + qid))));
#endif

#ifdef HW_BUFF_RECYLCE
#ifdef PORTS_LOOPBACK
		if (larg->garg->cpus == 4) {
			/* Use port_id and queue calculated in beginning of this function */
			err  = loop_hw_recycle(larg, port_id, port_id, tc, qid, num);
		} else {
			/* Set larg->id as rx_ppio, tx_ppio, for quick 2xloopback implementation
			 * and queue 0 only
			 */
			err  = loop_hw_recycle(larg, larg->id, larg->id, tc, 0, num);
		}
#else
		err  = loop_hw_recycle(larg, 0, 1, tc, qid, num);
		err |= loop_hw_recycle(larg, 1, 0, tc, qid, num);
#endif /* PORTS_LOOPBACK */
#else
#ifdef PORTS_LOOPBACK

		if (larg->garg->cpus == 4) {
			/* Use port_id and queue calculated in beginning of this function */
			err  = loop_sw_recycle(larg, port_id, port_id, tc, qid, num);
		} else {
			/* Set larg->id as rx_ppio, tx_ppio, for quick 2xloopback implementation
			 * and queue 0 only
			 */
			err  = loop_sw_recycle(larg, larg->id, larg->id, tc, qid, num);
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

	if (larg->num_ports == 1)
		return loop_1p(larg, running);
	return loop_2ps(larg, running);
}

static int init_all_modules(void)
{
	struct pp2_init_params	 pp2_params;
	int			 err;
	char			 file[PP2_MAX_BUF_STR_LEN];
	u32			 num_rss_tables;

	pr_info("Global initializations ...\n");

	err = mv_sys_dma_mem_init(PKT_ECHO_APP_DMA_MEM_SIZE);
	if (err)
		return err;

	memset(&pp2_params, 0, sizeof(pp2_params));
	pp2_params.hif_reserved_map = MVAPPS_PP2_HIFS_RSRV;
	pp2_params.bm_pool_reserved_map = MVAPPS_PP2_BPOOLS_RSRV;

	sprintf(file, "%s/%s", PP2_SYSFS_RSS_PATH, PP2_SYSFS_RSS_NUM_TABLES_FILE);
	num_rss_tables = appp_pp2_sysfs_param_get(garg.ports_desc[0].name, file);
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
	int				i;

	pr_info("Local initializations ...\n");

	err = app_hif_init(&garg->hif, PKT_ECHO_APP_HIF_Q_SIZE);
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
			port->num_tcs	= PKT_ECHO_APP_MAX_NUM_TCS_PER_PORT;
			for (i = 0; i < port->num_tcs; i++)
				port->num_inqs[i] =  garg->cpus;
			port->inq_size	= garg->rxq_size;
			port->num_outqs	= PKT_ECHO_APP_MAX_NUM_TCS_PER_PORT;
			port->outq_size	= PKT_ECHO_APP_TX_Q_SIZE;
			port->first_inq	= PKT_ECHO_APP_FIRST_INQ;
			if (garg->cpus == 1)
				port->hash_type = PP2_PPIO_HASH_T_NONE;
			else
				port->hash_type = PP2_PPIO_HASH_T_2_TUPLE;

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

	pr_info("done\n");
	return 0;
}

static void destroy_local_modules(struct glob_arg *garg)
{
	app_disable_all_ports(garg->ports_desc, garg->num_ports);
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

#ifdef SHOW_STATISTICS
static int stat_cmd_cb(void *arg, int argc, char *argv[])
{
	int i, j, reset = 0;
	struct glob_arg *garg = (struct glob_arg *)arg;

	if (argc > 1)
		reset = 1;
	printf("reset stats: %d\n", reset);
	for (j = 0; j < garg->num_ports; j++) {
		printf("Port%d stats:\n", j);
		for (i = 0; i < garg->cpus; i++) {
			printf("cpu%d: rx_bufs=%u, tx_bufs=%u, free_bufs=%u, tx_resend=%u, max_retries=%u, ",
			       i, rx_buf_cnt[i][j], tx_buf_cnt[i][j], free_buf_cnt[i][j],
				tx_buf_retry[i][j], tx_max_resend[i][j]);
			printf(" tx_drops=%u, max_burst=%u\n", tx_buf_drop[i][j], tx_max_burst[i][j]);
			if (reset) {
				rx_buf_cnt[i][j] = 0;
				tx_buf_cnt[i][j] = 0;
				free_buf_cnt[i][j] = 0;
				tx_buf_retry[i][j] = 0;
				tx_buf_drop[i][j] = 0;
				tx_max_burst[i][j] = 0;
				tx_max_resend[i][j] = 0;
			}
		}
	}
	return 0;
}
#endif

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
#ifdef SHOW_STATISTICS
	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "stat";
	cmd_params.desc		= "Show app statistics";
	cmd_params.format	= "<reset>";
	cmd_params.cmd_arg	= garg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))stat_cmd_cb;
	mvapp_register_cli_cmd(&cmd_params);
#endif
	app_register_cli_common_cmds(garg->ports_desc);

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

	if (garg->cli) {
		err = register_cli_cmds(garg);
		if (err)
			return err;
	}

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

	pthread_mutex_lock(&garg->trd_lock);
	err = app_hif_init(&larg->hif, PKT_ECHO_APP_HIF_Q_SIZE);
	pthread_mutex_unlock(&garg->trd_lock);
	if (err)
		return err;

	larg->id                = id;
	larg->burst		= garg->burst;
	larg->busy_wait		= garg->busy_wait;
	larg->multi_buffer_release = garg->multi_buffer_release;
	larg->echo              = garg->echo;
	larg->prefetch_shift	= garg->prefetch_shift;
	larg->num_ports         = garg->num_ports;
	larg->ports_desc = (struct lcl_port_desc *)malloc(larg->num_ports * sizeof(struct lcl_port_desc));
	if (!larg->ports_desc) {
		pr_err("no mem for local-port-desc obj!\n");
		return -ENOMEM;
	}
	memset(larg->ports_desc, 0, larg->num_ports * sizeof(struct lcl_port_desc));
	for (i = 0; i < larg->num_ports; i++)
		app_port_local_init(i, larg->id, &larg->ports_desc[i], &garg->ports_desc[i]);

	larg->pools_desc	= garg->pools_desc;
	larg->garg              = garg;

	larg->qs_map = garg->qs_map << (garg->qs_map_shift * id);
	pr_debug("thread %d (cpu %d) mapped to Qs %llx using %s\n",
		 larg->id, sched_getcpu(), (unsigned long long)larg->qs_map, name);

	*_larg = larg;
	return 0;
}

static void deinit_local(void *arg)
{
	struct local_arg *larg = (struct local_arg *)arg;
	int i;

	if (!larg)
		return;

	if (larg->ports_desc) {
		for (i = 0; i < larg->num_ports; i++)
			app_port_local_deinit(&larg->ports_desc[i]);
		free(larg->ports_desc);
	}

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
	       "\t-b <size>                Burst size, num_pkts handled in a batch.(default is %d)\n"
	       "\t--mtu <mtu>              Set MTU (default is %d)\n"
	       "\t-c, --cores <number>     Number of CPUs to use\n"
	       "\t-a, --affinity <number>  Use setaffinity (default is no affinity)\n"
	       "\t-s                       Maintain statistics\n"
	       "\t-w <cycles>              Cycles to busy_wait between recv&send, simulating app behavior (default=0)\n"
	       "\t--rxq <size>             Size of rx_queue (default is %d)\n"
	       "\t--old-tx-desc-release    Use pp2_bpool_put_buff(), instead of NEW pp2_bpool_put_buffs() API\n"
	       "\t--no-echo                Don't perform 'pkt_echo', N/A w/o define PKT_ECHO_APP_PKT_ECHO_SUPPORT\n"
	       "\t--cli                    Use CLI\n"
	       "\t?, -h, --help            Display help and exit.\n\n"
	       "\n", MVAPPS_NO_PATH(progname), MVAPPS_NO_PATH(progname),
	       MVAPPS_PP2_MAX_NUM_PORTS, PKT_ECHO_APP_MAX_BURST_SIZE, DEFAULT_MTU, PKT_ECHO_APP_RX_Q_SIZE);
}

static int parse_args(struct glob_arg *garg, int argc, char *argv[])
{
	int	i = 1;

	garg->cli = 0;
	garg->cpus = 1;
	garg->affinity = -1;
	garg->burst = PKT_ECHO_APP_DFLT_BURST_SIZE;
	garg->mtu = DEFAULT_MTU;
	garg->busy_wait	= 0;
	garg->rxq_size = PKT_ECHO_APP_RX_Q_SIZE;
	garg->multi_buffer_release = 1;
	garg->echo = 1;
	garg->qs_map = 0;
	garg->qs_map_shift = 0;
	garg->prefetch_shift = PKT_ECHO_APP_PREFETCH_SHIFT;
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
			for (token = strtok(argv[i + 1], ","), garg->num_ports = 0;
			     token;
			     token = strtok(NULL, ","), garg->num_ports++)
				snprintf(garg->ports_desc[garg->num_ports].name,
					 sizeof(garg->ports_desc[garg->num_ports].name),
					 "%s", token);

			if (garg->num_ports == 0) {
				pr_err("Invalid interface arguments format!\n");
				return -EINVAL;
			} else if (garg->num_ports > MVAPPS_PP2_MAX_NUM_PORTS) {
				pr_err("too many ports specified (%d vs %d)\n",
				       garg->num_ports, MVAPPS_PP2_MAX_NUM_PORTS);
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
		} else if (strcmp(argv[i], "--mtu") == 0) {
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
		} else if (strcmp(argv[i], "-s") == 0) {
			garg->maintain_stats = 1;
			i += 1;
		} else if (strcmp(argv[i], "-w") == 0) {
			garg->busy_wait = atoi(argv[i + 1]);
			i += 2;
		} else if (strcmp(argv[i], "--rxq") == 0) {
			garg->rxq_size = atoi(argv[i + 1]);
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
			rv = sscanf(argv[i + 1], "%x:%x", (unsigned int *)&garg->qs_map, &garg->qs_map_shift);
			if (rv != 2) {
				pr_err("Failed to parse -m parameter\n");
				return -EINVAL;
			}
			i += 2;
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
	if (garg->burst > PKT_ECHO_APP_MAX_BURST_SIZE) {
		pr_err("illegal burst size requested (%d vs %d)!\n",
		       garg->burst, PKT_ECHO_APP_MAX_BURST_SIZE);
		return -EINVAL;
	}
	if (garg->cpus > MVAPPS_PP2_MAX_NUM_CORES) {
		pr_err("illegal num cores requested (%d vs %d)!\n",
		       garg->cpus, MVAPPS_PP2_MAX_NUM_CORES);
		return -EINVAL;
	}
	if ((garg->affinity != -1) &&
	    ((garg->cpus + garg->affinity) > MVAPPS_PP2_MAX_NUM_CORES)) {
		pr_err("illegal num cores or affinity requested (%d,%d vs %d)!\n",
		       garg->cpus, garg->affinity, MVAPPS_PP2_MAX_NUM_CORES);
		return -EINVAL;
	}

	if (garg->qs_map &&
	    (MVAPPS_PP2_MAX_NUM_QS_PER_TC == 1) &&
	    (PKT_ECHO_APP_MAX_NUM_TCS_PER_PORT == 1)) {
		pr_warn("no point in queues-mapping; ignoring.\n");
		garg->qs_map = 1;
		garg->qs_map_shift = 1;
	} else if (!garg->qs_map) {
		garg->qs_map = 1;
		garg->qs_map_shift = PKT_ECHO_APP_MAX_NUM_TCS_PER_PORT;
	}

	if ((garg->cpus != 1) &&
	    (garg->qs_map & (garg->qs_map << garg->qs_map_shift))) {
		pr_err("Invalid queues-mapping (ovelapping CPUs)!\n");
		return -EINVAL;
	}

	return 0;
}

int main(int argc, char *argv[])
{
	struct mvapp_params	mvapp_params;
	u64			cores_mask;
	int			i, err;

	setbuf(stdout, NULL);
	pr_info("pkt-echo is started in %s - %s - %s\n", app_mode_str, buf_release_str, tx_retry_str);

	pr_debug("pr_debug is enabled\n");

	err = parse_args(&garg, argc, argv);
	if (err)
		return err;

	garg.pp2_num_inst = pp2_get_num_inst();

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
	mvapp_params.main_loop_cb	= main_loop;

#ifdef SHOW_STATISTICS
	for (i = 0; i < MVAPPS_PP2_MAX_NUM_CORES; i++) {
		int j;

		for (j = 0; j < garg.num_ports; j++) {
			rx_buf_cnt[i][j] = 0;
			tx_buf_cnt[i][j] = 0;
			free_buf_cnt[i][j] = 0;
			tx_buf_retry[i][j] = 0;
			tx_buf_drop[i][j] = 0;
			tx_max_burst[i][j] = 0;
			tx_max_resend[i][j] = 0;
		}
	}
#endif
	return mvapp_go(&mvapp_params);
}
