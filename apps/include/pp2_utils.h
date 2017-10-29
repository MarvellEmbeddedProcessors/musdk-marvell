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

#ifndef __MV_PP2_UTILS_H__
#define __MV_PP2_UTILS_H__

#include "mv_std.h"
#include "utils.h"
#include "mv_pp2_hif.h"
#include "mv_pp2_ppio.h"
#include "mv_pp2_bpool.h"

/* pkt offset, must be multiple of 32 bytes */
#define MVAPPS_PP2_PKT_DEF_OFFS			64
/* pkt offset including Marvell header */
#define MVAPPS_PP2_PKT_DEF_EFEC_OFFS		(MVAPPS_PP2_PKT_DEF_OFFS + MV_MH_SIZE)
#define MVAPPS_PP2_PKT_EFEC_OFFS(pkt_offset)	(pkt_offset + MV_MH_SIZE)

/* Maximum number of packet processors used by application */
#define MVAPPS_PP2_MAX_PKT_PROC		2
/* Maximum number of ports used by application */
#define MVAPPS_PP2_MAX_NUM_PORTS		2

/* Maximum number of queues per TC */
#define MVAPPS_PP2_MAX_NUM_QS_PER_TC	MVAPPS_MAX_NUM_CORES
/* Number of BM pools reserved by kernel */
#define MVAPPS_PP2_NUM_BPOOLS_RSRV	3
/* Reserved BM pools mask */
#define MVAPPS_PP2_BPOOLS_RSRV		((1 << MVAPPS_PP2_NUM_BPOOLS_RSRV) - 1)
/* Maximum number of pools per packet processor */
#define MVAPPS_PP2_MAX_NUM_BPOOLS	(PP2_BPOOL_NUM_POOLS - MVAPPS_PP2_NUM_BPOOLS_RSRV)
/* Total number of HIFs supported */
#define MVAPPS_PP2_TOTAL_NUM_HIFS	9 /* PP2_NUM_REGSPACES - move to API h file */
/* Number of HIFs reserved by kernel */
#define MVAPPS_PP2_NUM_HIFS_RSRV	4
/* Reserved HIFs mask */
#define MVAPPS_PP2_HIFS_RSRV		((1 << MVAPPS_PP2_NUM_HIFS_RSRV) - 1)

/* Number of policers reserved by kernel */
#define MVAPPS_PP2_NUM_POLICERS_RSRV	0
/* Reserved policers mask */
#define MVAPPS_PP2_POLICERSS_RSRV	((1 << MVAPPS_PP2_NUM_POLICERS_RSRV) - 1)

/* sysfs path for reading relevant parameters from kernel driver */
#define PP2_SYSFS_MUSDK_PATH		"/sys/devices/platform/pp2/musdk"
#define PP2_SYSFS_DEBUG_PORT_SET_FILE	"sysfs_current_port"
#define PP2_SYSFS_RX_FIRST_RXQ_FILE	"first_rxq"
#define PP2_SYSFS_RX_NUM_RXQ_FILE	"num_rx_queues"
#define PP2_SYSFS_TX_NUM_TXQ_FILE	"num_tx_queues"

#define PP2_SYSFS_RSS_PATH		"/sys/devices/platform/pp2/rss"
#define PP2_SYSFS_RSS_NUM_TABLES_FILE	"num_rss_tables"

#define PP2_MAX_BUF_STR_LEN		MV_MAX_BUF_STR_LEN


/* Macroes to handle PP2 counters */
#define INC_RX_COUNT(lcl_port_desc, cnt)	((lcl_port_desc)->cntrs.rx_buf_cnt += cnt)
#define INC_TX_COUNT(lcl_port_desc, cnt)	((lcl_port_desc)->cntrs.tx_buf_cnt += cnt)
#define INC_TX_RETRY_COUNT(lcl_port_desc, cnt)	((lcl_port_desc)->cntrs.tx_buf_retry += cnt)
#define INC_TX_DROP_COUNT(lcl_port_desc, cnt)	((lcl_port_desc)->cntrs.tx_buf_drop += cnt)
#define INC_FREE_COUNT(lcl_port_desc, cnt)	((lcl_port_desc)->cntrs.free_buf_cnt += cnt)
#define SET_MAX_RESENT(lcl_port_desc, cnt)		\
	{ if (cnt > (lcl_port_desc)->cntrs.tx_max_resend) \
		(lcl_port_desc)->cntrs.tx_max_resend = cnt; }

#define SET_MAX_BURST(lcl_port_desc, burst)	\
	{ if (burst > (lcl_port_desc)->cntrs.tx_max_burst) \
		(lcl_port_desc)->cntrs.tx_max_burst = burst; }


/*
 * Tx shadow queue entry
 */
struct tx_shadow_q_entry {
	struct pp2_buff_inf	 buff_ptr;	/* pointer to the buffer object */
	struct pp2_bpool	*bpool;		/* pointer to the bpool object */
};

/*
 * Tx shadow queue
 */
struct tx_shadow_q {
	u16				read_ind;	/* read index */
	u16				write_ind;	/* write index */

	struct tx_shadow_q_entry	*ents;		/* array of entries */
};



struct pp2_counters {
	u32		rx_buf_cnt;
	u32		free_buf_cnt;
	u32		tx_buf_cnt;
	u32		tx_buf_drop;
	u32		tx_buf_retry;
	u32		tx_max_resend;
	u32		tx_max_burst;
};



/*
 * General port parameters
 */
struct port_desc {
	char			 name[15];	/* Port name */
	int			 initialized;	/* Flag indicated is port was initialized */
	int			 pp_id;		/* Packet Processor ID */
	int			 ppio_id;	/* PPIO port ID */
	enum pp2_ppio_type	 ppio_type;	/* PPIO type */
	u32			 first_inq;	/* First RXQ - relative to the Port's first RXQ */
	u16			 num_tcs;	/* Number of TCs */
	u16			 num_inqs[PP2_PPIO_MAX_NUM_TCS];	/* Number of Rx queues per TC*/
	u16			 num_outqs;	/* Number of Tx queues */
	u32			 inq_size;	/* Rx queue size */
	u32			 outq_size;	/* Tx queue size */
	u32			 hash_type;	/* Hash type */
	u32			 first_rss_tbl;	/* First RSS table */
	u32			 traffic_dir;	/* Traffic direction (1 - Rx, 2 - Tx, 3 - Rx+Tx) */
	int			 plcr_argc;	/* policer params argc */
	char			 *plcr_argv[10];/* policer params argv */
	struct pp2_ppio		 *ppio;		/* PPIO object returned by pp2_ppio_init() */
	struct pp2_ppio_params	 port_params;	/* PPIO configuration parameters */
	struct lcl_port_desc	*lcl_ports_desc[MVAPPS_MAX_NUM_CORES];
};

/*
 * Local thread port parameters
 */
struct lcl_port_desc {
	int			id;		/* Local port ID*/
	int			lcl_id;		/* Local thread ID*/
	int			pp_id;		/* Packet Processor ID */
	int			ppio_id;	/* PPIO port ID */
	struct pp2_ppio		*ppio;		/* PPIO object returned by pp2_ppio_init() */
	int			num_shadow_qs;	/* Number of Tx shadow queues */
	int			shadow_q_size;	/* Size of Tx shadow queue */
	struct tx_shadow_q	*shadow_qs;	/* Tx shadow queue */
	struct pp2_counters	cntrs;
	u16			pkt_offset[PP2_PPIO_MAX_NUM_TCS];
};

/*
 * BM pool size parameters
 */
struct bpool_inf {
	int	buff_size;	/* buffer size */
	int	num_buffs;	/* number of buffers */
};

/*
 * BM pool parameters
 */
struct bpool_desc {
	struct pp2_bpool	*pool;		/* pointer to the bpool object */
	struct pp2_buff_inf	*buffs_inf;	/* array of buffer objects */
	int			 num_buffs;	/* number of buffers */
};


struct pp2_lcl_common_args {
	struct pp2_hif		*hif;
	struct lcl_port_desc	*lcl_ports_desc;
	struct bpool_desc	**pools_desc;
	int			multi_buffer_release;
};


struct pp2_glb_common_args {
	struct port_desc	ports_desc[MVAPPS_PP2_MAX_NUM_PORTS]; /* TODO: dynamic_size, as lcl_ports_desc */
	int			num_pools;
	int			pp2_num_inst;
	struct pp2_hif		*hif;
	struct bpool_desc	**pools_desc;
	int			multi_buffer_release;
};


static inline enum pp2_outq_l3_type pp2_l3_type_inq_to_outq(enum pp2_inq_l3_type l3_inq)
{
	if (unlikely(l3_inq & PP2_INQ_L3_TYPE_IPV6_NO_EXT))
		return PP2_OUTQ_L3_TYPE_IPV6;

	if (likely(l3_inq != PP2_INQ_L3_TYPE_NA))
		return PP2_OUTQ_L3_TYPE_IPV4;

	return PP2_OUTQ_L3_TYPE_OTHER;
}

static inline enum pp2_outq_l4_type pp2_l4_type_inq_to_outq(enum pp2_inq_l4_type l4_inq)
{
	if (likely(l4_inq == PP2_INQ_L4_TYPE_TCP || l4_inq == PP2_INQ_L4_TYPE_UDP))
		return (l4_inq - 1);

	return PP2_OUTQ_L4_TYPE_OTHER;
}

#ifndef HW_BUFF_RECYLCE
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
			pr_warn("Shadow memory @%d: cookie(0x%" PRIx64 "), pa(0x%" PRIdma "), pool(%p)!\n",
				i, binf->cookie, binf->addr, bpool);
			continue;
		}
		pp2_bpool_put_buff(hif, bpool, binf);
		free_cnt++;

		if (++idx == tx_port->shadow_q_size)
			idx = 0;
	}

	INC_FREE_COUNT(rx_port, free_cnt);
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

	INC_FREE_COUNT(rx_port, num);

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
#endif

#if defined USE_PP2_UTILS_LPBK_SW_RECYCLE && !defined HW_BUFF_RECYLCE
static inline int loop_sw_recycle(struct local_common_args *larg_cmn,
				  u8			 rx_ppio_id,
				  u8			 tx_ppio_id,
				  u8			 tc,
				  u8			 tc_qid,
				  u8			 tx_qid,
				  u16			 num)
{
	struct tx_shadow_q	*shadow_q;
	int			 shadow_q_size;
	struct pp2_ppio_desc	 descs[APP_MAX_BURST_SIZE];
	struct pp2_lcl_common_args *pp2_args = larg_cmn->plat;
	struct perf_cmn_cntrs	*perf_cntrs = &larg_cmn->perf_cntrs;
	struct lcl_port_desc	*rx_lcl_port_desc = &(pp2_args->lcl_ports_desc[rx_ppio_id]);
	struct lcl_port_desc	*tx_lcl_port_desc = &(pp2_args->lcl_ports_desc[tx_ppio_id]);
	u16			 i, j, tx_num;
	int			 mycyc;
#ifdef APP_TX_RETRY
	u16			 desc_idx = 0, cnt = 0;
#endif

#ifdef APP_PKT_ECHO_SUPPORT
	int			 prefetch_shift = larg_cmn->prefetch_shift;
#endif /* APP_PKT_ECHO_SUPPORT */
#ifdef APP_HW_TX_CHKSUM_CALC
	enum pp2_inq_l3_type     l3_type;
	enum pp2_inq_l4_type     l4_type;
	u8                       l3_offset, l4_offset;
#endif /* APP_HW_TX_CHKSUM_CALC */
	u16 pkt_offset = MVAPPS_PP2_PKT_EFEC_OFFS(rx_lcl_port_desc->pkt_offset[tc]);


	shadow_q = &tx_lcl_port_desc->shadow_qs[tx_qid];
	shadow_q_size = tx_lcl_port_desc->shadow_q_size;

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
#ifdef APP_HW_TX_CHKSUM_CALC
		pp2_ppio_inq_desc_get_l3_info(&descs[i], &l3_type, &l3_offset);
		pp2_ppio_inq_desc_get_l4_info(&descs[i], &l4_type, &l4_offset);
#endif /* APP_HW_TX_CHKSUM_CALC */

		pp2_ppio_outq_desc_reset(&descs[i]);
#ifdef APP_HW_TX_CHKSUM_CALC
#if (APP_HW_TX_IPV4_CHKSUM_CALC || APP_HW_TX_CHKSUM_CALC)
		pp2_ppio_outq_desc_set_proto_info(&descs[i], pp2_l3_type_inq_to_outq(l3_type),
						  pp2_l4_type_inq_to_outq(l4_type), l3_offset,
						  l4_offset, APP_HW_TX_IPV4_CHKSUM_CALC,
						  APP_HW_TX_CHKSUM_CALC);
#endif /* (APP_HW_TX_IPV4_CHKSUM_CALC ||  ... */
#endif /* APP_HW_TX_CHKSUM_CALC */
		pp2_ppio_outq_desc_set_phys_addr(&descs[i], pa);
		pp2_ppio_outq_desc_set_pkt_offset(&descs[i], pkt_offset);
		pp2_ppio_outq_desc_set_pkt_len(&descs[i], len);
		shadow_q->ents[shadow_q->write_ind].buff_ptr.cookie = (uintptr_t)buff;
		shadow_q->ents[shadow_q->write_ind].buff_ptr.addr = pa;
		shadow_q->ents[shadow_q->write_ind].bpool = bpool;
		pr_debug("buff_ptr.cookie(0x%lx)\n", shadow_q->ents[shadow_q->write_ind].buff_ptr.cookie);
		shadow_q->write_ind++;
		if (shadow_q->write_ind == shadow_q_size)
			shadow_q->write_ind = 0;

		/* Below condition should never happen, if shadow_q size is large enough */
		if (unlikely(shadow_q->write_ind == shadow_q->read_ind)) {
			pr_err("%s: port(%d), txq_id(%d), shadow_q size=%d is too small, performing emergency drops\n",
				__func__, tx_ppio_id, tx_qid, shadow_q_size);
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
	SET_MAX_BURST(rx_lcl_port_desc, num);
	for (mycyc = 0; mycyc < larg_cmn->busy_wait; mycyc++)
		asm volatile("");
#ifdef APP_TX_RETRY
	do {
		tx_num = num;
		if (num) {
			pp2_ppio_send(pp2_args->lcl_ports_desc[tx_ppio_id].ppio, pp2_args->hif, tc,
				      &descs[desc_idx], &tx_num);
			if (num > tx_num) {
				if (!cnt)
					INC_TX_RETRY_COUNT(rx_lcl_port_desc, num - tx_num);
				cnt++;
			}
			desc_idx += tx_num;
			num -= tx_num;
			INC_TX_COUNT(rx_lcl_port_desc, tx_num);
			perf_cntrs->tx_cnt += tx_num;
		}
		free_sent_buffers(rx_lcl_port_desc, tx_lcl_port_desc, pp2_args->hif,
				  tx_qid, pp2_args->multi_buffer_release);
	} while (num);
	SET_MAX_RESENT(rx_lcl_port_desc, cnt);
#else
	if (num) {
		tx_num = num;
		pp2_ppio_send(tx_lcl_port_desc->ppio, pp2_args->hif, tx_qid, descs, &tx_num);
		if (num > tx_num) {
			u16 not_sent = num - tx_num;
			/* Free not sent buffers */
			shadow_q->write_ind = (shadow_q->write_ind < not_sent) ?
						(shadow_q_size - not_sent + shadow_q->write_ind) :
						shadow_q->write_ind - not_sent;
			free_buffers(rx_lcl_port_desc, tx_lcl_port_desc,
				     pp2_args->hif, shadow_q->write_ind, not_sent, tx_qid);
			INC_TX_DROP_COUNT(rx_lcl_port_desc, not_sent);
			perf_cntrs->drop_cnt += not_sent;
		}
		INC_TX_COUNT(rx_lcl_port_desc, tx_num);
		perf_cntrs->tx_cnt += tx_num;
	}
	free_sent_buffers(rx_lcl_port_desc, tx_lcl_port_desc, pp2_args->hif,
			  tx_qid, pp2_args->multi_buffer_release);
#endif /* APP_TX_RETRY */

	return 0;
}
#endif

/*
 * Init HIF object
 */
int app_hif_init(struct pp2_hif **hif, u32 queue_size);
/*
 * Build all pools
 */
int app_build_all_bpools(struct bpool_desc ***ppools, int num_pools, struct bpool_inf infs[], struct pp2_hif *hif);
/*
 * Free all pools
 */
void app_free_all_pools(struct bpool_desc **pools, int num_pools, struct pp2_hif *hif);

/*
 * Parse port pp_id and ppio_id from port name
 */
int app_find_port_info(struct port_desc *port_desc);
/*
 * Init port
 */
int app_port_init(struct port_desc *port, int num_pools, struct bpool_desc *pools, u16 mtu, u16 pkt_offset);

/*
 * Init local port object per thread according to port parameters
 */
void app_port_local_init(int id, int lcl_id, struct lcl_port_desc *lcl_port, struct port_desc *port);
/*
 * Deinit all ports
 */
void app_deinit_all_ports(struct port_desc *ports, int num_ports);
/*
 * Deinit local port object
 */
void app_port_local_deinit(struct lcl_port_desc *lcl_port);
/*
 * Disable all ports
 */
void app_disable_all_ports(struct port_desc *ports, int num_ports);

/* TODO: may be need to move to other file */
/*
 * Get line
 */
int app_get_line(char *prmpt, char *buff, size_t sz, int *argc, char *argv[]);

/*
 * Show rx queue statistics
 */
void app_show_rx_queue_stat(struct port_desc *port_desc, u8 tc, u8 q_start, int num_qs, int reset);
/*
 * Show tx queue statistics
 */
void app_show_tx_queue_stat(struct port_desc *port_desc, int reset);
/*
 * Show port statistics
 */
void app_show_port_stat(struct port_desc *port_desc, int reset);
/*
 * Register common CLI commands (currently show queue and port statistics)
 */
int app_register_cli_common_cmds(struct port_desc *port_desc);

/*
 * Get sysfs parameter from kernel driver
 */
u32 appp_pp2_sysfs_param_get(char *if_name, char *file);

/*
 * Standard deinit_local, for local_threads deinitialization.
 */
void apps_pp2_deinit_local(void *arg);
/*
 * Standard destroy_local_modules.
 */
void apps_pp2_destroy_local_modules(void *arg);

/*
 * Standard destroy_all_modules.
 */
void apps_pp2_destroy_all_modules(void);
/*
 * Standard deinit_global, for global deinitialization.
 */
void apps_pp2_deinit_global(void *arg);

int apps_pp2_stat_cmd_cb(void *arg, int argc, char *argv[]);







#endif /*__MVUTILS_H__*/

