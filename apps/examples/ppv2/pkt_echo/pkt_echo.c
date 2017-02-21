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

//#define HW_BUFF_RECYLCE
//#define SW_BUFF_RECYLCE
//#define PORTS_LOOPBACK
#define APP_TX_RETRY
#define SHOW_STATISTICS


#ifdef HW_BUFF_RECYLCE
static char buf_release_str[] = "HW buffer release";
#else
static char buf_release_str[] = "SW buffer release";
#endif

#ifdef PORTS_LOOPBACK
static char app_mode_str[] = "Loopback mode";
#else
static char app_mode_str[] = "Bridge mode";
#endif

#ifdef APP_TX_RETRY
static char tx_retry_str[] = "Tx Retry enabled";
#else
static char tx_retry_str[] = "Tx Retry disabled";
#endif

#define TX_RETRY_WAIT	1
#define MAX_NUM_BUFFS	8192
#define Q_SIZE		1024
#define MAX_BURST_SIZE	(Q_SIZE)>>1
#define DFLT_BURST_SIZE	256
#define PKT_OFFS	64
#define PKT_EFEC_OFFS	(PKT_OFFS+PP2_MH_SIZE)
#define MAX_PPIOS	1
#define MAX_NUM_CORES	4
#define DMA_MEM_SIZE 	(40*1024*1024)
#define PP2_NUM_BPOOLS_RSRV		3
#define PP2_BPOOLS_RSRV			((1 << PP2_NUM_BPOOLS_RSRV) - 1)
#define PP2_HIFS_RSRV			0xF
#define PP2_MAX_NUM_PORTS		2
#define PP2_MAX_NUM_TCS_PER_PORT	1
#define PP2_MAX_NUM_QS_PER_TC		MAX_NUM_CORES
#define MAX_NUM_QS_PER_CORE		PP2_MAX_NUM_TCS_PER_PORT

/* TODO: find more generic way to get the following parameters */
#define PP2_TOTAL_NUM_BPOOLS	16
#define PP2_TOTAL_NUM_HIFS	9
#define PP2_MAX_NUM_BPOOLS	min(PP2_PPIO_TC_MAX_POOLS*PP2_PPIO_MAX_NUM_TCS, \
				PP2_TOTAL_NUM_BPOOLS - PP2_NUM_BPOOLS_RSRV)


/* TODO: Move to internal .h file */
#define upper_32_bits(n) ((u32)(((n) >> 16) >> 16))
#define lower_32_bits(n) ((u32)(n))

#ifdef HW_BUFF_RECYLCE
/* sw-buuf-recycle is not allowed when using hw-buff-recycle! */
#undef SW_BUFF_RECYLCE
#endif /* HW_BUFF_RECYLCE */
//#define  HW_TX_CHKSUM_CALC
#ifdef HW_TX_CHKSUM_CALC
#define HW_TX_L4_CHKSUM_CALC	1
#define HW_TX_IPV4_CHKSUM_CALC	1
#endif
#define PKT_ECHO_SUPPORT
#define USE_APP_PREFETCH
#define PREFETCH_SHIFT	7

/** Get rid of path in filename - only for unix-type paths using '/' */
#define NO_PATH(file_name) (strrchr((file_name), '/') ? \
			    strrchr((file_name), '/') + 1 : (file_name))

#define BPOOLS_INF	{ {384, 4096}, {2048, 1024} }
/* #define BPOOLS_INF	{{2048, 8192}} */

#ifdef SW_BUFF_RECYLCE
#define COOKIE_BUILD(_pp, _bpool, _indx)	\
	(u32)(((u32)_pp<<30) | ((u32)_bpool<<24) | ((u32)_indx))
#define COOKIE_GET_PP(_cookie)		(_cookie>>30)
#define COOKIE_GET_BPOOL(_cookie)	((_cookie>>26) & 0x3f)
#define COOKIE_GET_INDX(_cookie)	(_cookie & 0xffffff)
#endif /* SW_BUFF_RECYLCE */

#if 0
#include <sys/time.h>   // for gettimeofday()
#define CLK_MHZ	1300
static int usecs1 = 0;
#endif /* 0 */


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

#ifndef HW_BUFF_RECYLCE
struct tx_shadow_q_entry {
#ifdef SW_BUFF_RECYLCE
	u32		 	buff_ptr;
#else
	struct pp2_buff_inf	buff_ptr;
#endif /* SW_BUFF_RECYLCE */
	struct pp2_bpool	*bpool;
};

struct tx_shadow_q {
	u16				 read_ind;
	u16				 write_ind;

	struct tx_shadow_q_entry	 ents[Q_SIZE];
};
#endif /* !HW_BUFF_RECYLCE */

struct glob_arg {
	int			 verbose;
	int			 cli;
	int			 cpus;	/* cpus used for running */
	u16			 burst;
	int			 affinity;
	int			 loopback;
	int			 echo;
	u64			 qs_map;
	int			 qs_map_shift;
	int			 prefetch_shift;
	int			 num_ports;
	int			 pp2_num_inst;
	struct port_desc	 ports_desc[PP2_MAX_NUM_PORTS];

	pthread_mutex_t		 trd_lock;

	struct pp2_hif		*hif;

	int			 num_pools;
	struct pp2_bpool	***pools;
	struct pp2_buff_inf	***buffs_inf;
};

struct local_arg {
#ifndef HW_BUFF_RECYLCE
#ifdef SW_BUFF_RECYLCE
	struct pp2_buff_inf	***buffs_inf;
#endif /* SW_BUFF_RECYLCE */

	struct tx_shadow_q	shadow_qs[MAX_NUM_QS_PER_CORE];
#endif /* !HW_BUFF_RECYLCE */
	int			 prefetch_shift;
	u64			 qs_map;

	struct pp2_hif		*hif;
	int			 num_ports;
	struct lcl_port_desc	*ports_desc;

	struct pp2_bpool	***pools;


	u16			 burst;
	int			 echo;
	int			 id;

	struct glob_arg		*garg;
};


static struct glob_arg garg = {};
static u64 sys_dma_high_addr = 0;

static u16	used_bpools[PP2_NUM_PKT_PROC] = {PP2_BPOOLS_RSRV, PP2_BPOOLS_RSRV};
static u16	used_hifs = PP2_HIFS_RSRV;

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

u32 rx_buf_cnt[MAX_NUM_CORES][PP2_MAX_NUM_PORTS];
u32 free_buf_cnt[MAX_NUM_CORES][PP2_MAX_NUM_PORTS];
u32 tx_buf_cnt[MAX_NUM_CORES][PP2_MAX_NUM_PORTS];
u32 tx_buf_drop[MAX_NUM_CORES][PP2_MAX_NUM_PORTS];
u32 tx_buf_retry[MAX_NUM_CORES][PP2_MAX_NUM_PORTS];
u32 tx_max_resend[MAX_NUM_CORES][PP2_MAX_NUM_PORTS];
u32 tx_max_burst[MAX_NUM_CORES][PP2_MAX_NUM_PORTS];

#else
#define INC_RX_COUNT(core, port, cnt)
#define INC_TX_COUNT(core, port, cnt)
#define INC_TX_RETRY_COUNT(core, port, cnt)
#define INC_TX_DROP_COUNT(core, port, cnt)
#define INC_FREE_COUNT(core, port, cnt)
#define SET_MAX_RESENT(core, port, cnt)
#define SET_MAX_BURST(core, port, cnt)
#endif /* SHOW_STATISTICS */

#ifdef PKT_ECHO_SUPPORT
#ifdef USE_APP_PREFETCH
static inline void prefetch(const void *ptr)
{
	asm volatile("prfm pldl1keep, %a0\n" : : "p" (ptr));
}
#endif /* USE_APP_PREFETCH */

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

#ifdef HW_TX_CHKSUM_CALC
static inline enum pp2_outq_l3_type pp2_l3_type_inq_to_outq(enum pp2_inq_l3_type l3_inq)
{
	if (unlikely(l3_inq & PP2_INQ_L3_TYPE_IPV6_NO_EXT))
		return(PP2_OUTQ_L3_TYPE_IPV6);

	if (likely(l3_inq != PP2_INQ_L3_TYPE_NA))
		return(PP2_OUTQ_L3_TYPE_IPV4);

	return(PP2_OUTQ_L3_TYPE_OTHER);
}

static inline enum pp2_outq_l4_type pp2_l4_type_inq_to_outq(enum pp2_inq_l4_type l4_inq)
{
	if (likely(l4_inq == PP2_INQ_L4_TYPE_TCP || l4_inq == PP2_INQ_L4_TYPE_UDP))
		return(l4_inq - 1);

	return(PP2_OUTQ_L4_TYPE_OTHER);
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
		struct pp2_bpool *bpool = pp2_ppio_inq_desc_get_bpool(&descs[i], larg->ports_desc[ppio_id].port);
		binf.addr = pp2_ppio_inq_desc_get_phys_addr(&descs[i]);
		binf.cookie = pp2_ppio_inq_desc_get_cookie(&descs[i]);
		pp2_bpool_put_buff(larg->hif, bpool, &binf);
	}
	INC_FREE_COUNT(larg->id, ppio_id, num);
}
#else
static inline u16 free_buffers(struct local_arg	*larg,
				u16		start_idx,
				u16		num,
				u8		ppio_id,
				u8		tc)
{
	u16			i, idx = start_idx;
	struct pp2_buff_inf	*binf;
	struct tx_shadow_q	*shadow_q;

	shadow_q = &(larg->shadow_qs[tc]);


	for (i = 0; i < num; i++) {
		struct pp2_bpool *bpool = shadow_q->ents[idx].bpool;
#ifdef SW_BUFF_RECYLCE
		struct pp2_buff_inf	 tmp_buff_inf;
		u32	ck = shadow_q->ents[idx].buff_ptr;
		tmp_buff_inf.cookie = ck;
		tmp_buff_inf.addr = larg->buffs_inf[COOKIE_GET_PP(ck)][COOKIE_GET_BPOOL(ck)][COOKIE_GET_INDX(ck)].addr;
		binf = &tmp_buff_inf;
#else
		binf = &(shadow_q->ents[idx].buff_ptr);
#endif /* SW_BUFF_RECYLCE */
		if (unlikely(!binf->cookie || !binf->addr || !bpool)) {
			pr_warn("Shadow memory @%d: cookie(%lx), pa(%lx), pool(%lx)!\n",
					i, (u64)binf->cookie, (u64)binf->addr, (u64)bpool);
			continue;
		}
		pp2_bpool_put_buff(larg->hif,
					bpool,
					binf);
		INC_FREE_COUNT(larg->id, ppio_id, 1);

		if (++idx == Q_SIZE)
			idx = 0;

	}
	return idx;
}
static inline void free_sent_buffers(struct local_arg	*larg,
				     u8			tx_ppio_id,
				     u8			rx_ppio_id,
				     u8			tc)
{
	u16			tx_num;

	pp2_ppio_get_num_outq_done(larg->ports_desc[tx_ppio_id].port, larg->hif, tc, &tx_num);
	larg->shadow_qs[tc].read_ind = free_buffers(larg, larg->shadow_qs[tc].read_ind, tx_num, rx_ppio_id, tc);
}

#endif //HW_BUFF_RECYLCE

#ifdef HW_BUFF_RECYLCE
static inline int loop_hw_recycle(struct local_arg	*larg,
				  u8			 rx_ppio_id,
				  u8			 tx_ppio_id,
				  u8			 tc,
				  u8			 qid,
				  u16			 num)
{
	struct pp2_ppio_desc	 descs[MAX_BURST_SIZE];
	u16			 i, tx_num;
#ifdef APP_TX_RETRY
	u16			 desc_idx = 0, cnt = 0;
#endif
#ifdef PKT_ECHO_SUPPORT
	int			 prefetch_shift = larg->prefetch_shift;
#endif /* PKT_ECHO_SUPPORT */


//pr_info("tid %d check on tc %d, qid %d\n", larg->id, tc, qid);
//pthread_mutex_lock(&larg->garg->trd_lock);
	pp2_ppio_recv(larg->ports_desc[rx_ppio_id].port, tc, qid, descs, &num);
//pthread_mutex_unlock(&larg->garg->trd_lock);
//if (num) pr_info("got %d pkts on thd %d, tc %d, qid %d\n", num, larg->id, tc, qid);
	INC_RX_COUNT(larg->id, rx_ppio_id, num);

	for (i=0; i<num; i++) {
		char		*buff = (char *)(uintptr_t)pp2_ppio_inq_desc_get_cookie(&descs[i]);
		dma_addr_t	 pa = pp2_ppio_inq_desc_get_phys_addr(&descs[i]);
		u16 len = pp2_ppio_inq_desc_get_pkt_len(&descs[i]);
		struct pp2_bpool *bpool = pp2_ppio_inq_desc_get_bpool(&descs[i], larg->ports_desc[rx_ppio_id].port);

#ifdef PKT_ECHO_SUPPORT
		if (likely(larg->echo)) {
			char *tmp_buff;
#ifdef USE_APP_PREFETCH
			if (num-i > prefetch_shift) {
				tmp_buff = (char *)(uintptr_t)pp2_ppio_inq_desc_get_cookie(&descs[i+prefetch_shift]);
				tmp_buff +=PKT_EFEC_OFFS;
				pr_debug("tmp_buff_before(%p)\n", tmp_buff);
				tmp_buff = (char *)(((uintptr_t)tmp_buff)|sys_dma_high_addr);
				pr_debug("tmp_buff_after(%p)\n", tmp_buff);
				prefetch(tmp_buff);
			}
#endif /* USE_APP_PREFETCH */
			tmp_buff = (char *)(((uintptr_t)(buff))|sys_dma_high_addr);
			pr_debug("buff2(%p)\n", tmp_buff);
			tmp_buff += PKT_EFEC_OFFS;
			//printf("packet:\n"); mem_disp(tmp_buff, len);
			swap_l2(tmp_buff);
			swap_l3(tmp_buff);
			//printf("packet:\n"); mem_disp(tmp_buff, len);
		}
#endif /* PKT_ECHO_SUPPORT */

		pp2_ppio_outq_desc_reset(&descs[i]);
		pp2_ppio_outq_desc_set_phys_addr(&descs[i], pa);
		pp2_ppio_outq_desc_set_pkt_offset(&descs[i], PKT_EFEC_OFFS);
		pp2_ppio_outq_desc_set_pkt_len(&descs[i], len);
		pp2_ppio_outq_desc_set_cookie(&descs[i], (uintptr_t)(buff));
		pp2_ppio_outq_desc_set_pool(&descs[i], bpool);
	}

	SET_MAX_BURST(larg->id, rx_ppio_id, num);
#ifdef APP_TX_RETRY
	while (num) {
		tx_num = num;
		pp2_ppio_send(larg->ports_desc[tx_ppio_id].port, larg->hif, tc, &descs[desc_idx], &tx_num);

		if (num > tx_num) {
			if (!cnt)
				INC_TX_RETRY_COUNT(larg->id, rx_ppio_id, num - tx_num);
			cnt++;
			usleep(TX_RETRY_WAIT);
		}
		desc_idx += tx_num;
		num -= tx_num;
		INC_TX_COUNT(larg->id, rx_ppio_id, tx_num);
	}
	SET_MAX_RESENT(larg->id, rx_ppio_id, cnt);
#else
	if (num) {
		tx_num = num;
		pp2_ppio_send(larg->ports_desc[tx_ppio_id].port, larg->hif, tc, descs, &tx_num);
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
	struct pp2_ppio_desc	 descs[MAX_BURST_SIZE];
	u16			 i, tx_num;
#ifdef APP_TX_RETRY
	u16			 desc_idx = 0, cnt = 0;
#endif

#ifdef PKT_ECHO_SUPPORT
	int			 prefetch_shift = larg->prefetch_shift;
#endif /* PKT_ECHO_SUPPORT */
#ifdef HW_TX_CHKSUM_CALC
	enum pp2_inq_l3_type     l3_type;
	enum pp2_inq_l4_type     l4_type;
	u8                       l3_offset, l4_offset;
#endif /* HW_TX_CHKSUM_CALC */

	shadow_q = &(larg->shadow_qs[tc]);

//pr_info("tid %d check on tc %d, qid %d\n", larg->id, tc, qid);
//pthread_mutex_lock(&larg->garg->trd_lock);
	pp2_ppio_recv(larg->ports_desc[rx_ppio_id].port, tc, qid, descs, &num);
//pthread_mutex_unlock(&larg->garg->trd_lock);
//if (num) pr_info("got %d pkts on tc %d, qid %d\n", num, tc, qid);
	INC_RX_COUNT(larg->id, rx_ppio_id, num);

	for (i=0; i<num; i++) {
#ifdef SW_BUFF_RECYLCE
		u32		 ck = pp2_ppio_inq_desc_get_cookie(&descs[i]);
		char		*buff = (char *)(uintptr_t)larg->buffs_inf[COOKIE_GET_PP(ck)][COOKIE_GET_BPOOL(ck)][COOKIE_GET_INDX(ck)].cookie;
		dma_addr_t	 pa = larg->buffs_inf[COOKIE_GET_PP(ck)][COOKIE_GET_BPOOL(ck)][COOKIE_GET_INDX(ck)].addr;
#else
		char		*buff = (char *)(uintptr_t)pp2_ppio_inq_desc_get_cookie(&descs[i]);
		dma_addr_t	 pa = pp2_ppio_inq_desc_get_phys_addr(&descs[i]);
#endif /* SW_BUFF_RECYLCE */
		u16 len = pp2_ppio_inq_desc_get_pkt_len(&descs[i]);
		struct pp2_bpool *bpool = pp2_ppio_inq_desc_get_bpool(&descs[i], larg->ports_desc[rx_ppio_id].port);

#ifdef PKT_ECHO_SUPPORT
		if (likely(larg->echo)) {
			char *tmp_buff;
#ifdef USE_APP_PREFETCH
			if (num-i > prefetch_shift) {
#ifdef SW_BUFF_RECYLCE
				u32 ck = pp2_ppio_inq_desc_get_cookie(&descs[i+prefetch_shift]);
				tmp_buff = (char *)(uintptr_t)larg->buffs_inf[COOKIE_GET_PP(ck)][COOKIE_GET_BPOOL(ck)][COOKIE_GET_INDX(ck)].cookie;
#else
				tmp_buff = (char *)(uintptr_t)pp2_ppio_inq_desc_get_cookie(&descs[i+prefetch_shift]);
#endif /* SW_BUFF_RECYLCE */
				tmp_buff +=PKT_EFEC_OFFS;
				pr_debug("tmp_buff_before(%p)\n", tmp_buff);
				tmp_buff = (char *)(((uintptr_t)tmp_buff)|sys_dma_high_addr);
				pr_debug("tmp_buff_after(%p)\n", tmp_buff);
				prefetch(tmp_buff);
			}
#endif /* USE_APP_PREFETCH */
			tmp_buff = (char *)(((uintptr_t)(buff))|sys_dma_high_addr);
			pr_debug("buff2(%p)\n", tmp_buff);
			tmp_buff += PKT_EFEC_OFFS;
			//printf("packet:\n"); mem_disp(tmp_buff, len);
			swap_l2(tmp_buff);
			swap_l3(tmp_buff);
			//printf("packet:\n"); mem_disp(tmp_buff, len);
		}
#endif /* PKT_ECHO_SUPPORT */
#ifdef HW_TX_CHKSUM_CALC
		pp2_ppio_inq_desc_get_l3_info(&descs[i], &l3_type, &l3_offset);
		pp2_ppio_inq_desc_get_l4_info(&descs[i], &l4_type, &l4_offset);
#endif /* HW_TX_CHKSUM_CALC */

		pp2_ppio_outq_desc_reset(&descs[i]);
#ifdef HW_TX_CHKSUM_CALC
#if (HW_TX_IPV4_CHKSUM_CALC || HW_TX_L4_CHKSUM_CALC)
		pp2_ppio_outq_desc_set_proto_info(&descs[i], pp2_l3_type_inq_to_outq(l3_type),
						  pp2_l4_type_inq_to_outq(l4_type), l3_offset,
						  l4_offset, HW_TX_IPV4_CHKSUM_CALC, HW_TX_L4_CHKSUM_CALC);
#endif /* (HW_TX_IPV4_CHKSUM_CALC ||  ... */
#endif /* HW_TX_CHKSUM_CALC */
		pp2_ppio_outq_desc_set_phys_addr(&descs[i], pa);
		pp2_ppio_outq_desc_set_pkt_offset(&descs[i], PKT_EFEC_OFFS);
		pp2_ppio_outq_desc_set_pkt_len(&descs[i], len);
#ifdef SW_BUFF_RECYLCE
		shadow_q->ents[shadow_q->write_ind].buff_ptr = ck;
#else
		shadow_q->ents[shadow_q->write_ind].buff_ptr.cookie = (uintptr_t)buff;
		shadow_q->ents[shadow_q->write_ind].buff_ptr.addr = pa;
#endif /* SW_BUFF_RECYLCE */
		shadow_q->ents[shadow_q->write_ind].bpool = bpool;
		pr_debug("buff_ptr.cookie(0x%lx)\n", (u64)shadow_q->ents[shadow_q->write_ind].buff_ptr.cookie);
		shadow_q->write_ind++;
		if (shadow_q->write_ind == Q_SIZE)
			shadow_q->write_ind = 0;
	}
	SET_MAX_BURST(larg->id, rx_ppio_id, num);
#ifdef APP_TX_RETRY
	do {
		tx_num = num;
		if (num) {
			pp2_ppio_send(larg->ports_desc[tx_ppio_id].port, larg->hif, tc, &descs[desc_idx], &tx_num);
			if (num > tx_num) {
				if (!cnt)
					INC_TX_RETRY_COUNT(larg->id, rx_ppio_id, num - tx_num);
				cnt++;
			}
			desc_idx += tx_num;
			num -= tx_num;
			INC_TX_COUNT(larg->id, rx_ppio_id, tx_num);
		}
		free_sent_buffers(larg, tx_ppio_id, rx_ppio_id, tc);
	} while (num);
	SET_MAX_RESENT(larg->id, rx_ppio_id, cnt);
#else
	if (num) {
		tx_num = num;
		pp2_ppio_send(larg->ports_desc[tx_ppio_id].port, larg->hif, tc, descs, &tx_num);
		if (num > tx_num) {
			u16 not_sent = num - tx_num;
			// Free not sent buffers
			shadow_q->write_ind = (shadow_q->write_ind < not_sent) ?
						(Q_SIZE - not_sent + shadow_q->write_ind) :
						shadow_q->write_ind - not_sent;
			free_buffers(larg, shadow_q->write_ind, not_sent, rx_ppio_id, tc);
			INC_TX_DROP_COUNT(larg->id, rx_ppio_id, not_sent);
		}
		INC_TX_COUNT(larg->id, rx_ppio_id, tx_num);
	}
	free_sent_buffers(larg, tx_ppio_id, rx_ppio_id, tc);
#endif /* APP_TX_RETRY */

	return 0;
}
#endif /* HW_BUFF_RECYLCE */

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
	if ((err = pp2_netdev_get_port_info(name,
					    &pp,
					    &ppio)) != 0) {
		pr_err("PP2 Port %s not found!\n", port_desc->name);
		return err;
	}

	port_desc->ppio_id = ppio;
	port_desc->pp_id = pp;

	return 0;
}

static int find_free_bpool(u32 pp_id)
{
	int	i;

	for (i=0; i<PP2_TOTAL_NUM_BPOOLS; i++) {
		if (!((1 << i) & used_bpools[pp_id])) {
			used_bpools[pp_id] |= (1 << i);
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

	for (i=0; i<PP2_TOTAL_NUM_HIFS; i++) {
		if (!((1 << i) & used_hifs)) {
			used_hifs |= (1 << i);
			break;
		}
	}
	if (i == PP2_TOTAL_NUM_HIFS) {
		pr_err("no free HIF found!\n");
		return -ENOSPC;
	}
	return i;
}

static int loop_1p(struct local_arg *larg, volatile int *running)
{
	int			 err;
	u16			 num;
	u8 			 tc = 0, qid = 0;

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

#ifdef HW_BUFF_RECYLCE
		err = loop_hw_recycle(larg, 0, 0, tc, qid, num);
#else
		err = loop_sw_recycle(larg, 0, 0, tc, qid, num);
#endif /* HW_BUFF_RECYLCE */
		if (err != 0) return err;
	}

	return 0;
}

static int loop_2ps(struct local_arg *larg, volatile int *running)
{
	int			 err;
	u16			 num;
	u8 			 tc = 0, qid = 0;
#ifdef PORTS_LOOPBACK
	int 			 port_id = 0;
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
		pr_debug("loop_2ps: for cpu %d (%d), port %d, queue %d \n", larg->id, larg->garg->cpus, port_id, qid);
	}
#endif

	num = larg->burst;
	while (*running) {
#ifndef PORTS_LOOPBACK
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
#endif

#ifdef HW_BUFF_RECYLCE
#ifdef PORTS_LOOPBACK
		if (larg->garg->cpus == 4) {
			/* Use port_id and queue calculated in beginning of this function */
			err  = loop_hw_recycle(larg, port_id, port_id, tc, qid, num);
		}
		else {
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
		}
		else {
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
		if (err != 0) return err;
	}

	return 0;
}

static int main_loop(void *arg, volatile int *running)
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

	pr_info("Global initializations ... \n");



	if ((err = mv_sys_dma_mem_init(DMA_MEM_SIZE)) != 0)
		return err;

	memset(&pp2_params, 0, sizeof(pp2_params));
	pp2_params.hif_reserved_map = PP2_HIFS_RSRV;
	pp2_params.bm_pool_reserved_map = PP2_BPOOLS_RSRV;
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
	if ((err = pp2_init(&pp2_params)) != 0)
		return err;

	pr_info("done\n");
	return 0;
}

static int build_all_bpools(struct glob_arg *garg)
{
	struct pp2_bpool_params	 	bpool_params;
	int			 	i, j, k, err, pool_id;
	struct bpool_inf		infs[] = BPOOLS_INF;
	char				name[15];
	int 				pp2_num_inst = garg->pp2_num_inst;

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
	if (garg->num_pools > PP2_MAX_NUM_BPOOLS) {
		pr_err("only %d pools allowed!\n", PP2_MAX_NUM_BPOOLS);
		return -EINVAL;
	}

	for (i=0; i<pp2_num_inst; i++) {
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

		for (j=0; j<garg->num_pools; j++) {
#if 0
struct timeval t1, t2;
#endif /* 0 */
			pool_id = find_free_bpool(i);
			if (pool_id < 0) {
				pr_err("free bpool not found!\n");
				return pool_id;
			}
			memset(name, 0, sizeof(name));
			snprintf(name, sizeof(name), "pool-%d:%d", i, pool_id);
			pr_debug("found bpool:  %s\n", name);
			memset(&bpool_params, 0, sizeof(bpool_params));
			bpool_params.match = name;
			bpool_params.max_num_buffs = MAX_NUM_BUFFS;
			bpool_params.buff_len = infs[j].buff_size;
			if ((err = pp2_bpool_init(&bpool_params, &garg->pools[i][j])) != 0)
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

			for (k=0; k<infs[j].num_buffs; k++) {
				void * buff_virt_addr;
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
					pr_err("buff_virt_addr(%p)  upper out of range; skipping this buff\n", buff_virt_addr);
					continue;
				}
				garg->buffs_inf[i][j][k].addr =
					(bpool_dma_addr_t)mv_sys_dma_mem_virt2phys(buff_virt_addr);
				garg->buffs_inf[i][j][k].cookie =
					lower_32_bits((u64)buff_virt_addr); /* cookie contains lower_32_bits of the va */
			}
#if 0
// start timer
gettimeofday(&t1, NULL);
#endif /* 0 */
			for (k=0; k<infs[j].num_buffs; k++) {
				struct pp2_buff_inf	tmp_buff_inf;
				/* Don't add first buffer into BPool */
				if (k == 0) continue;
#ifdef SW_BUFF_RECYLCE
				tmp_buff_inf.cookie = COOKIE_BUILD(i, j, k);
#else
				tmp_buff_inf.cookie = garg->buffs_inf[i][j][k].cookie;
#endif /* SW_BUFF_RECYLCE */
				tmp_buff_inf.addr   = garg->buffs_inf[i][j][k].addr;
				if ((err = pp2_bpool_put_buff(garg->hif,
							      garg->pools[i][j],
							      &tmp_buff_inf)) != 0)
					return err;
			}
#if 0
// stop timer
gettimeofday(&t2, NULL);
// compute and print the elapsed time in millisec
usecs1 = (t2.tv_sec - t1.tv_sec) * 1000000.0;      // sec to us
usecs1 += (t2.tv_usec - t1.tv_usec);
printf("pp2-bpool-put count: %d cycles  =================\n", usecs1*CLK_MHZ/infs[j].num_buffs);
#endif /* 0 */
		}
	}

	return 0;
}

static int init_local_modules(struct glob_arg *garg)
{
	struct pp2_hif_params	 	hif_params;
	struct pp2_ppio_params	 	port_params;
	struct pp2_ppio_inq_params	inq_params;
	char				name[15];
	int			 	i, j, err, port_index, hif_id;

	pr_info("Local initializations ... ");

	if ((hif_id = find_free_hif()) < 0) {
		pr_err("free HIF not found!\n");
		return hif_id;
	}
	memset(name, 0, sizeof(name));
	snprintf(name, sizeof(name), "hif-%d", hif_id);
	pr_debug("found hif: %s\n", name);
	memset(&hif_params, 0, sizeof(hif_params));
	hif_params.match = name;
	hif_params.out_size = Q_SIZE;
	if ((err = pp2_hif_init(&hif_params, &garg->hif)) != 0)
		return err;
	if (!garg->hif) {
		pr_err("HIF init failed!\n");
		return -EIO;
	}

	if ((err = build_all_bpools(garg)) != 0)
		return err;

	for (port_index = 0; port_index < garg->num_ports; port_index++) {
		if ((err = find_port_info(&garg->ports_desc[port_index])) != 0) {
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
		for (i=0; i<port_params.inqs_params.num_tcs; i++) {
			port_params.inqs_params.tcs_params[i].pkt_offset = PKT_OFFS>>2;
			port_params.inqs_params.tcs_params[i].num_in_qs = PP2_MAX_NUM_QS_PER_TC;
			/* TODO: we assume here only one Q per TC; change it! */
			inq_params.size = Q_SIZE;
			port_params.inqs_params.tcs_params[i].inqs_params = &inq_params;
			for (j=0; j<garg->num_pools; j++)
				port_params.inqs_params.tcs_params[i].pools[j] =
					garg->pools[garg->ports_desc[port_index].pp_id][j];
		}
		port_params.outqs_params.num_outqs = PP2_MAX_NUM_TCS_PER_PORT;
		for (i=0; i<port_params.outqs_params.num_outqs; i++) {
			port_params.outqs_params.outqs_params[i].size = Q_SIZE;
			port_params.outqs_params.outqs_params[i].weight = 1;
		}
		if ((err = pp2_ppio_init(&port_params, &garg->ports_desc[port_index].port)) != 0)
			return err;
		if (!garg->ports_desc[port_index].port) {
			pr_err("PP-IO init failed!\n");
			return -EIO;
		}

		if ((err = pp2_ppio_enable(garg->ports_desc[port_index].port)) != 0)
			return err;
	}

	pr_info("done\n");
	return 0;
}

static void destroy_local_modules(struct glob_arg *garg)
{
	int	i, j;
	int 	pp2_num_inst = garg->pp2_num_inst;

	if (garg->ports_desc[0].port) {
		pp2_ppio_disable(garg->ports_desc[0].port);
		pp2_ppio_deinit(garg->ports_desc[0].port);
	}

	if (garg->pools) {
		for (i=0; i<pp2_num_inst; i++) {
			if (garg->pools[i]) {
				for (j=0; j<garg->num_pools; j++)
					if (garg->pools[i][j])
						pp2_bpool_deinit(garg->pools[i][j]);
				free(garg->pools[i]);
			}
		}
		free(garg->pools);
	}
	if (garg->buffs_inf) {
		for (i=0; i<pp2_num_inst; i++) {
			if (garg->buffs_inf[i]) {
				for (j=0; j<garg->num_pools; j++)
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
		for (i = 0; i < MAX_NUM_CORES; i++) {
			printf("cpu%d: rx_bufs=%d, tx_bufs=%d, free_bufs=%d, tx_resend=%d, max_retries=%d, ",
				i, rx_buf_cnt[i][j], tx_buf_cnt[i][j], free_buf_cnt[i][j],
				tx_buf_retry[i][j], tx_max_resend[i][j]);
			printf(" tx_drops=%d, max_burst=%d\n", tx_buf_drop[i][j], tx_max_burst[i][j]);
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
	cmd_params.desc		= "Show statistics";
	cmd_params.format	= "<reset>";
	cmd_params.cmd_arg	= garg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))stat_cmd_cb;
	mvapp_register_cli_cmd(&cmd_params);
#endif
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

	if ((err = init_all_modules()) != 0)
		return err;

	if ((err = init_local_modules(garg)) != 0)
		return err;

	if (garg->cli && ((err = register_cli_cmds(garg)) != 0))
		return err;

	return 0;
}

static void deinit_global(void *arg)
{
	struct glob_arg *garg = (struct glob_arg *)arg;

	if (!garg)
		return;
	if (garg->cli)
		/* TODO: unregister cli cmds */;
	destroy_local_modules(garg);
	destroy_all_modules();
}

static int init_local(void *arg, int id, void **_larg)
{
	struct glob_arg		*garg = (struct glob_arg *)arg;
	struct local_arg	*larg;
	struct pp2_hif_params	 hif_params;
	char			 name[15];
	int			 i, err, hif_id;

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
	if ((hif_id = find_free_hif()) < 0) {
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

	larg->id                = id;
	larg->burst		= garg->burst;
	larg->echo              = garg->echo;
	larg->prefetch_shift	= garg->prefetch_shift;
	larg->num_ports         = garg->num_ports;
	larg->ports_desc = (struct lcl_port_desc *)malloc(larg->num_ports*sizeof(struct lcl_port_desc));
	if (!larg->ports_desc) {
		pr_err("no mem for local-port-desc obj!\n");
		return -ENOMEM;
	}
	memset(larg->ports_desc, 0, larg->num_ports*sizeof(struct lcl_port_desc));
	for (i=0; i<larg->num_ports; i++) {
		larg->ports_desc[i].pp_id = garg->ports_desc[i].pp_id;
		larg->ports_desc[i].ppio_id = garg->ports_desc[i].ppio_id;
		larg->ports_desc[i].port = garg->ports_desc[i].port;
	}
	larg->pools             = garg->pools;
#ifndef HW_BUFF_RECYLCE
#ifdef SW_BUFF_RECYLCE
	larg->buffs_inf         = garg->buffs_inf;
#endif /* SW_BUFF_RECYLCE */
#endif /* !HW_BUFF_RECYLCE */
	larg->garg              = garg;

	larg->qs_map = garg->qs_map << (garg->qs_map_shift * id);
	pr_debug("thread %d (cpu %d) mapped to Qs %llx using %s\n",
		 larg->id, sched_getcpu(), (long long unsigned int)larg->qs_map, name);

	*_larg = larg;
	return 0;
}

static void deinit_local(void *arg)
{
	struct local_arg *larg = (struct local_arg *)arg;

	if (!larg)
		return;

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
	       "\t--no-echo                No Echo packets\n"
	       "\t--cli                    Use CLI\n"
	       "\t?, -h, --help            Display help and exit.\n\n"
	       "\n", NO_PATH(progname), NO_PATH(progname), MAX_PPIOS, MAX_BURST_SIZE
	       );
}

static int parse_args(struct glob_arg *garg, int argc, char *argv[])
{
	int	i = 1;

	garg->cli = 0;
	garg->cpus = 1;
	garg->affinity = -1;
	garg->burst = DFLT_BURST_SIZE;
	garg->echo = 1;
	garg->qs_map = 0;
	garg->qs_map_shift = 0;
	garg->prefetch_shift = PREFETCH_SHIFT;

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
			sscanf(token,"%x", (unsigned int *)&garg->qs_map);
			token = strtok(NULL, "");
			garg->qs_map_shift = atoi(token);
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

	return 0;
}


int main (int argc, char *argv[])
{
	struct mvapp_params	mvapp_params;
	u64			cores_mask;
	int			i, err;

	setbuf(stdout, NULL);
	pr_info("pkt-echo is started in %s - %s - %s\n", app_mode_str, buf_release_str, tx_retry_str);

	pr_debug("pr_debug is enabled\n");

	if ((err = parse_args(&garg, argc, argv)) != 0)
		return err;

	garg.pp2_num_inst = pp2_get_num_inst();

	cores_mask = 0;
	for (i=0; i<garg.cpus; i++, cores_mask<<=1, cores_mask|=1) ;
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
	for (i = 0; i < MAX_NUM_CORES; i++) {
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
