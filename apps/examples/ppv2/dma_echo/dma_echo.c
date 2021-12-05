/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#define _GNU_SOURCE         /* See feature_test_macros(7) */
#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <sched.h>
#include <sys/time.h>
#include <netinet/ip.h>

#include "mv_std.h"
#include "lib/lib_misc.h"
#include "lib/mv_pme.h"
#include "env/io.h"

#include "mv_stack.h"

#include "mv_pp2.h"
#include "mv_pp2_hif.h"
#include "mv_pp2_bpool.h"
#include "mv_pp2_ppio.h"
#include "mv_dmax2.h"

#include "pp2_utils.h"
#include "mvapp.h"


#define DMA_ECHO_APP_DEF_Q_SIZE		1024
#define DMA_ECHO_APP_HIF_Q_SIZE		(8 * DMA_ECHO_APP_DEF_Q_SIZE)
#define DMA_ECHO_APP_RX_Q_SIZE		(2 * DMA_ECHO_APP_DEF_Q_SIZE)
#define DMA_ECHO_APP_TX_Q_SIZE		(2 * DMA_ECHO_APP_DEF_Q_SIZE)
#define DMA_ECHO_APP_DMAX2_Q_SIZE	(2 * DMA_ECHO_APP_DEF_Q_SIZE)
#define DMA_ECHO_APP_DSHADOW_Q_SIZE	(DMA_ECHO_APP_DMAX2_Q_SIZE)

#define DMA_ECHO_APP_MAX_BURST_SIZE	((DMA_ECHO_APP_RX_Q_SIZE) >> 2)
#define DMA_ECHO_APP_DFLT_BURST_SIZE	256
#define DMA_ECHO_APP_CTRL_DFLT_THR	1000
#define DMA_ECHO_APP_DMA_MEM_SIZE	(48 * 1024 * 1024)

#define DMA_ECHO_APP_FIRST_INQ			0
#define DMA_ECHO_APP_MAX_NUM_TCS_PER_PORT	1
#define DMA_ECHO_APP_MAX_NUM_QS_PER_CORE	DMA_ECHO_APP_MAX_NUM_TCS_PER_PORT
#define DMA_ECHO_APP_MAX_NUM_SESSIONS		32

#define DMAX2_INTERFACE_COUNT		4

/*#define DMA_ECHO_APP_VERBOSE_CHECKS*/
#define DMA_ECHO_APP_VERBOSE_DEBUG
#define DMA_ECHO_APP_PKT_ECHO_SUPPORT
#define DMA_ECHO_APP_PREFETCH_SHIFT	4

#define PP2_COOKIE_SET_ALL_INFO(_c, _rp, _tp, _bp, _d)	\
	((_c) = (((_c) & ~0x3d) | \
		(((_d) << 0) | ((_rp) << 2) | ((_tp) << 3) | ((_bp) << 4))))

#define PP2_COOKIE_CLEAR_ALL_INFO(_c)	((_c) = ((_c) & ~0x3d))

#define DMA_ECHO_APP_BPOOLS_INF		{ {2048, 4096, 0, NULL} }
#define DMA_ECHO_APP_BPOOLS_JUMBO_INF	{ {10240, 512, 0, NULL} }

#define next_q_idx(_idx, _size) (((_idx+1) == _size) ? 0 : (_idx+1))
#define q_occupancy(prod, cons, q_size)	\
	(((prod) - (cons) + (q_size)) & ((q_size) - 1))
#define q_space(prod, cons, q_size)	\
	((q_size) - q_occupancy((prod), (cons), (q_size)) - 1)

#define START_COUNT_CYCLES(_ev_cnt)		pme_ev_cnt_start(_ev_cnt)
#define STOP_COUNT_CYCLES(_ev_cnt, _num)	pme_ev_cnt_stop(_ev_cnt, _num)


enum pkt_state {
	PKT_STATE_FREE = 0,
	PKT_STATE_RX,
	PKT_STATE_TX,
	PKT_STATE_DMA,
	PKT_STATE_LAST
};

struct pkt_mdata {
	u8			 state; /* as defined in enum pkt_state */
	u8			 rx_port;
	u8			 tx_port;
	u8			 data_offs;
	u16			 flags;
	u16			 len;
	void			*buf_vaddr;
	struct pp2_bpool	*bpool;
};

struct dmax2_shadow_q {
	u16			 read_ind;	/* read index */
	u16			 write_ind;	/* write index */
	struct pkt_mdata	*ents[DMA_ECHO_APP_DSHADOW_Q_SIZE];
};

struct local_arg;

struct glob_arg {
	struct glb_common_args	 cmn_args;  /* Keep first */

	u16			 engine;
	pthread_mutex_t		 trd_lock;

	int			 num_bufs;
};

struct local_arg {
	struct local_common_args	cmn_args;  /* Keep first */

	struct dmax2			*dmax2;
	struct mv_stack			*stack_hndl;
	struct dmax2_shadow_q		 dmax2_shadow_q;
};


static struct glob_arg garg = {};

static int pme_ev_cnt_rx = -1, pme_ev_cnt_enq = -1, pme_ev_cnt_deq = -1, pme_ev_cnt_tx = -1;


static inline u16 free_all_buffers(struct lcl_port_desc	*tx_port,
				   struct pp2_hif	*hif,
				   u8			 tc)
{
	u16			cont_in_shadow, idx, num, req_num;
	struct tx_shadow_q	*shadow_q;

	pp2_ppio_get_num_outq_done(tx_port->ppio, hif, tc, &num);
	if (!num)
		return 0;

	shadow_q = &tx_port->shadow_qs[tc];

	idx = shadow_q->read_ind;
	cont_in_shadow = DMA_ECHO_APP_TX_Q_SIZE - idx;

	if (num <= cont_in_shadow) {
		req_num = num;
		pp2_bpool_put_buffs(hif, (struct buff_release_entry *)&shadow_q->ents[idx], &req_num);
		idx = idx + num;
		if (idx == DMA_ECHO_APP_TX_Q_SIZE)
			idx = 0;
	} else {
		req_num = cont_in_shadow;
		pp2_bpool_put_buffs(hif, (struct buff_release_entry *)&shadow_q->ents[idx], &req_num);

		req_num = num - cont_in_shadow;
		pp2_bpool_put_buffs(hif, (struct buff_release_entry *)&shadow_q->ents[0], &req_num);
		idx = num - cont_in_shadow;
	}
	shadow_q->read_ind = idx;

	return num;
}

static inline int proc_rx_pkts(struct local_arg		*larg,
			       u8			 rx_ppio_id,
			       u8			 tx_ppio_id,
			       struct pp2_ppio_desc	*descs,
			       u16			 num)
{
	struct pp2_lcl_common_args	*pp2_args = (struct pp2_lcl_common_args *) larg->cmn_args.plat;
	struct perf_cmn_cntrs		*perf_cntrs = &larg->cmn_args.perf_cntrs;
	struct dmax2_shadow_q		*dmax2_shadow_q;
	struct pkt_mdata		*mdata;
	struct dmax2_desc		 dmax2_descs[DMA_ECHO_APP_MAX_BURST_SIZE];
	u16				 i, num_got, tmp_num;
	int				 err;

	dmax2_shadow_q = &larg->dmax2_shadow_q;

#ifdef DMA_ECHO_APP_VERBOSE_DEBUG
	if (larg->cmn_args.verbose)
		pr_info("%s: %d packets received. %d -> %d\n", __func__, num, rx_ppio_id, tx_ppio_id);
#endif /* DMA_ECHO_APP_VERBOSE_DEBUG */

	memset(dmax2_descs, 0, sizeof(dmax2_descs));

	for (i = 0; i < num; i++) {
		char *vaddr;
		struct pp2_bpool *bpool;

		if (next_q_idx(dmax2_shadow_q->write_ind, DMA_ECHO_APP_DSHADOW_Q_SIZE) == dmax2_shadow_q->read_ind) {
			pr_warn("overlapped (%d,%d,%d)!\n",
				dmax2_shadow_q->write_ind,
				next_q_idx(dmax2_shadow_q->write_ind,
				DMA_ECHO_APP_DSHADOW_Q_SIZE),
				dmax2_shadow_q->read_ind);
			break;
		}

		mdata = (struct pkt_mdata *)mv_stack_pop(larg->stack_hndl);
		if (!mdata) {
			pr_debug("run out of mdata obj!\n");
			break;
		}

		vaddr = (char *)(uintptr_t)pp2_ppio_inq_desc_get_cookie(&descs[i]);
		bpool = pp2_ppio_inq_desc_get_bpool(&descs[i], pp2_args->lcl_ports_desc[rx_ppio_id].ppio);

		mdata->state = PKT_STATE_DMA;
		mdata->rx_port = rx_ppio_id;
		mdata->tx_port = tx_ppio_id;
		mdata->bpool = bpool;
		mdata->buf_vaddr = vaddr;
		mdata->len = pp2_ppio_inq_desc_get_pkt_len(&descs[i]);

		/* Set vaddr and paddr to MAC address of the packet */
		vaddr += MVAPPS_PP2_PKT_DEF_EFEC_OFFS;

		/* Prepare descriptors */
		dmax2_descs[i].desc_ctrl = DESC_OP_MODE_MEMCPY << DESC_OP_MODE_SHIFT;
		dmax2_descs[i].buff_size = pp2_ppio_inq_desc_get_pkt_len(&descs[i]);
		dmax2_descs[i].src_addr = pp2_ppio_inq_desc_get_phys_addr(&descs[i]) + MVAPPS_PP2_PKT_DEF_EFEC_OFFS;
		dmax2_descs[i].dst_addr = dmax2_descs[i].src_addr;
		dmax2_descs[i].flags = DESC_FLAGS_SYNC;
#ifdef DMA_ECHO_APP_VERBOSE_DEBUG
		/* desc_id/cookie = descriptor number, for easy comparison after Dequeue */
		dmax2_descs[i].desc_id = dmax2_shadow_q->write_ind;
#endif /* DMA_ECHO_APP_VERBOSE_DEBUG */

		dmax2_shadow_q->ents[dmax2_shadow_q->write_ind] = mdata;
		dmax2_shadow_q->write_ind = next_q_idx(dmax2_shadow_q->write_ind, DMA_ECHO_APP_DSHADOW_Q_SIZE);

#ifdef DMA_ECHO_APP_VERBOSE_DEBUG
		if (larg->cmn_args.verbose > 1) {
			printf("Received packet (va:%p, pa 0x%08x, len %d):\n",
			       vaddr,
			       (unsigned int)dmax2_descs[i].src_addr,
			       dmax2_descs[i].buff_size);
			mem_disp((char *)vaddr, dmax2_descs[i].buff_size);
		}
#endif /* DMA_ECHO_APP_VERBOSE_DEBUG */
	}
	tmp_num = num_got = i;

START_COUNT_CYCLES(pme_ev_cnt_enq);
		err = dmax2_enq(larg->dmax2, dmax2_descs, &num_got);
STOP_COUNT_CYCLES(pme_ev_cnt_enq, num_got);

	if (num_got < num) {
		struct pp2_bpool	*bpool;
		struct pp2_buff_inf	 binf;

		if (unlikely(err)) {
			pr_err("DMAX2 EnQ (EnC) failed (%d)!\n", err);
			return -EFAULT;
		}

		for (i = num_got; i < num; i++) {
			if (i < tmp_num) {
				if (dmax2_shadow_q->write_ind == 0)
					dmax2_shadow_q->write_ind = DMA_ECHO_APP_DSHADOW_Q_SIZE;
				dmax2_shadow_q->write_ind--;
				mdata = dmax2_shadow_q->ents[dmax2_shadow_q->write_ind];
				mv_stack_push(larg->stack_hndl, mdata);
			}
			binf.addr = pp2_ppio_inq_desc_get_phys_addr(&descs[i]);
			binf.cookie = pp2_ppio_inq_desc_get_cookie(&descs[i]);
			bpool = pp2_ppio_inq_desc_get_bpool(&descs[i], pp2_args->lcl_ports_desc[rx_ppio_id].ppio);
			pp2_bpool_put_buff(pp2_args->hif, bpool, &binf);
		}
		/*pr_warn("%s: %d packets dropped\n", __func__, num - num_got);*/
		perf_cntrs->drop_cnt += (num - num_got);
	}

	return 0;
}

static inline int deq_n_send_pkts(struct local_arg *larg, u8 tc)
{
	struct pp2_lcl_common_args	*pp2_args = (struct pp2_lcl_common_args *) larg->cmn_args.plat;
	struct perf_cmn_cntrs		*perf_cntrs = &larg->cmn_args.perf_cntrs;
	struct tx_shadow_q		*pp2_shadow_q;
	struct dmax2_shadow_q		*dmax2_shadow_q;
	struct pp2_bpool		*bpool;
	struct pp2_buff_inf		*binf;
	struct pp2_ppio_desc		*desc;
	struct pkt_mdata		*mdata;
	struct dmax2_trans_complete_desc dmax2_res_descs[DMA_ECHO_APP_MAX_BURST_SIZE];
	struct pp2_ppio_desc		 descs[MVAPPS_PP2_MAX_NUM_PORTS][DMA_ECHO_APP_MAX_BURST_SIZE];
	u16				 i, tp, num, num_got, port_nums[MVAPPS_PP2_MAX_NUM_PORTS];
	int				 err;
#ifdef DMA_ECHO_APP_PKT_ECHO_SUPPORT
	char				*tmp_buff;
	u16				 prefetch_idx;
	int				 prefetch_shift = larg->cmn_args.prefetch_shift;
#endif /* DMA_ECHO_APP_PKT_ECHO_SUPPORT */

	num = DMA_ECHO_APP_MAX_BURST_SIZE;
START_COUNT_CYCLES(pme_ev_cnt_deq);
	err = dmax2_deq(larg->dmax2, dmax2_res_descs, &num, 1);
STOP_COUNT_CYCLES(pme_ev_cnt_deq, num);
	if (unlikely(err)) {
		pr_err("DMAX2 DeQ (EnC) failed (%d)!\n", err);
		return -EFAULT;
	}

	for (tp = 0; tp < larg->cmn_args.num_ports; tp++)
		port_nums[tp] = 0;

	dmax2_shadow_q = &larg->dmax2_shadow_q;
#ifdef DMA_ECHO_APP_PKT_ECHO_SUPPORT
	prefetch_idx = dmax2_shadow_q->read_ind + prefetch_shift;
	if (prefetch_idx >= MVAPPS_PP2_PKT_DEF_EFEC_OFFS)
		prefetch_idx -= MVAPPS_PP2_PKT_DEF_EFEC_OFFS;
#endif /* DMA_ECHO_APP_PKT_ECHO_SUPPORT */

	for (i = 0; i < num; i++) {
		char			*buff;
		dma_addr_t		pa;
		u16			len;

		mdata = dmax2_shadow_q->ents[dmax2_shadow_q->read_ind];
		if (unlikely(!mdata || !mdata->buf_vaddr)) {
			pr_warn("Shadow memory @%d: cookie(%p), pa(%lx)!\n",
				dmax2_shadow_q->read_ind, mdata, (u64)mdata->buf_vaddr);
			continue;
		}

#ifdef DMA_ECHO_APP_VERBOSE_DEBUG
		if (dmax2_res_descs[i].status)
			pr_err("Illegal status (%x)!\n", dmax2_res_descs[i].status);
		if (dmax2_res_descs[i].desc_id != dmax2_shadow_q->read_ind)
			pr_err("Illegal id (%d vs %d)!\n", dmax2_res_descs[i].desc_id, dmax2_shadow_q->read_ind);
#endif /* DMA_ECHO_APP_VERBOSE_DEBUG */

		dmax2_shadow_q->read_ind = next_q_idx(dmax2_shadow_q->read_ind, DMA_ECHO_APP_DSHADOW_Q_SIZE);

		buff = mdata->buf_vaddr;
		pa = mv_sys_dma_mem_virt2phys(buff);
		len = mdata->len;

		tp = mdata->tx_port;
		bpool = mdata->bpool;
		desc = &descs[tp][port_nums[tp]];

#ifdef DMA_ECHO_APP_PKT_ECHO_SUPPORT
		if (num - i > prefetch_shift) {
			struct pkt_mdata *prefetch_mdata = dmax2_shadow_q->ents[prefetch_idx];

			prefetch(prefetch_mdata->buf_vaddr + MVAPPS_PP2_PKT_DEF_OFFS);
			prefetch_idx = next_q_idx(prefetch_idx, DMA_ECHO_APP_DSHADOW_Q_SIZE);
		}

		/* pointer to MAC header */
		tmp_buff = (char *)mdata->buf_vaddr + MVAPPS_PP2_PKT_DEF_EFEC_OFFS;
		tmp_buff = (char *)((uintptr_t)tmp_buff | app_get_high_addr());

#ifdef DMA_ECHO_APP_VERBOSE_DEBUG
		if (larg->cmn_args.verbose > 1) {
			printf("pkt before echo (len %d):\n", len);
			mem_disp(tmp_buff, len);
		}
#endif /* DMA_ECHO_APP_VERBOSE_DEBUG */
		swap_l2(tmp_buff);
		swap_l3(tmp_buff);
#endif /* DMA_ECHO_APP_PKT_ECHO_SUPPORT */

		pp2_ppio_outq_desc_reset(desc);
		pp2_ppio_outq_desc_set_phys_addr(desc, pa);
		pp2_ppio_outq_desc_set_pkt_offset(desc, MVAPPS_PP2_PKT_DEF_EFEC_OFFS);
		pp2_ppio_outq_desc_set_pkt_len(desc, len);

#ifdef DMA_ECHO_APP_VERBOSE_DEBUG
		if (larg->cmn_args.verbose > 1) {
			printf("Sending packet (va:%p, pa 0x%08x, len %d):\n",
			       buff, (unsigned int)pa, len);
			mem_disp(buff + MVAPPS_PP2_PKT_DEF_EFEC_OFFS, len);
		}
#endif /* DMA_ECHO_APP_VERBOSE_DEBUG */

		pp2_shadow_q = &pp2_args->lcl_ports_desc[tp].shadow_qs[tc];
		pp2_shadow_q->ents[pp2_shadow_q->write_ind].buff_ptr.cookie = (uintptr_t)buff;
		pp2_shadow_q->ents[pp2_shadow_q->write_ind].buff_ptr.addr = pa;
		pp2_shadow_q->ents[pp2_shadow_q->write_ind].bpool = bpool;

		pp2_shadow_q->write_ind = next_q_idx(pp2_shadow_q->write_ind, DMA_ECHO_APP_TX_Q_SIZE);
		port_nums[tp]++;

		/* We don't need the metadata anymore */
		mv_stack_push(larg->stack_hndl, mdata);
	}

	for (tp = 0; tp < larg->cmn_args.num_ports; tp++) {
		num = num_got = port_nums[tp];
		pp2_shadow_q = &pp2_args->lcl_ports_desc[tp].shadow_qs[tc];

START_COUNT_CYCLES(pme_ev_cnt_tx);
		if (num_got) {
			err = pp2_ppio_send(pp2_args->lcl_ports_desc[tp].ppio, pp2_args->hif, tc,
					    descs[tp], &num_got);
			if (err)
				return err;
		}
STOP_COUNT_CYCLES(pme_ev_cnt_tx, num_got);

#ifdef DMA_ECHO_APP_VERBOSE_DEBUG
		if (larg->cmn_args.verbose && num_got)
			printf("sent %d pkts on ppio %d, tc %d\n", num_got, tp, tc);
#endif /* DMA_ECHO_APP_VERBOSE_DEBUG */

		if (num_got < num) {
			for (i = 0; i < num - num_got; i++) {
				if (pp2_shadow_q->write_ind == 0)
					pp2_shadow_q->write_ind = DMA_ECHO_APP_TX_Q_SIZE;
				pp2_shadow_q->write_ind--;
				binf = &pp2_shadow_q->ents[pp2_shadow_q->write_ind].buff_ptr;
				if (unlikely(!binf->cookie || !binf->addr)) {
					pr_warn("Shadow memory @%d: cookie(%lx), pa(%lx)!\n",
						pp2_shadow_q->write_ind, (u64)binf->cookie, (u64)binf->addr);
					continue;
				}
				bpool = pp2_shadow_q->ents[pp2_shadow_q->write_ind].bpool;
				pp2_bpool_put_buff(pp2_args->hif,
						   bpool,
						   binf);
			}
			/*pr_warn("%s: %d packets dropped\n", __func__, num - num_got);*/
			perf_cntrs->drop_cnt += (num - num_got);
		}
		perf_cntrs->tx_cnt += num;

		free_all_buffers(&pp2_args->lcl_ports_desc[tp], pp2_args->hif, tc);
	}

	return 0;
}

static inline int loop_sw_recycle(struct local_arg	*larg,
				  u8			 rx_ppio_id,
				  u8			 tx_ppio_id,
				  u8			 tc,
				  u8			 qid,
				  u16			 num)
{
	struct pp2_ppio_desc		 descs[DMA_ECHO_APP_MAX_BURST_SIZE];
	struct pp2_lcl_common_args	*pp2_args = (struct pp2_lcl_common_args *) larg->cmn_args.plat;
	struct perf_cmn_cntrs		*perf_cntrs = &larg->cmn_args.perf_cntrs;
	u16				 free_count;
	int				 err;

	/* Protect from overflow on the DMAX2 Q */
	free_count = q_space(larg->dmax2_shadow_q.write_ind,
			larg->dmax2_shadow_q.read_ind,
			DMA_ECHO_APP_DSHADOW_Q_SIZE);
	if (num > free_count)
		num = free_count;

/*pr_info("tid %d check on tc %d, qid %d\n", larg->cmn_args.id, tc, qid);*/
START_COUNT_CYCLES(pme_ev_cnt_rx);
	err = pp2_ppio_recv(pp2_args->lcl_ports_desc[rx_ppio_id].ppio, tc, qid, descs, &num);
STOP_COUNT_CYCLES(pme_ev_cnt_rx, num);

#ifdef DMA_ECHO_APP_VERBOSE_DEBUG
	if (larg->cmn_args.verbose && num)
		printf("recv %d pkts on ppio %d, tc %d, qid %d\n", num, rx_ppio_id, tc, qid);
#endif /* DMA_ECHO_APP_VERBOSE_DEBUG */

	if (num) {
		perf_cntrs->rx_cnt += num;

		err = proc_rx_pkts(larg, rx_ppio_id, tx_ppio_id, descs, num);
		if (unlikely(err))
			return err;
	}

	return deq_n_send_pkts(larg, tc);
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

	num = larg->cmn_args.burst;

	while (*running) {
		/* Find next queue to consume */
		do {
			qid++;
			if (qid == mvapp_pp2_max_num_qs_per_tc) {
				qid = 0;
				tc++;
				if (tc == DMA_ECHO_APP_MAX_NUM_TCS_PER_PORT)
					tc = 0;
			}
		} while (!(larg->cmn_args.qs_map & (1 << ((tc * mvapp_pp2_max_num_qs_per_tc) + qid))));

		err = loop_sw_recycle(larg, 0, 0, tc, qid, num);
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

	num = larg->cmn_args.burst;

	while (*running) {
		/* Find next queue to consume */
		do {
			qid++;
			if (qid == mvapp_pp2_max_num_qs_per_tc) {
				qid = 0;
				tc++;
				if (tc == DMA_ECHO_APP_MAX_NUM_TCS_PER_PORT)
					tc = 0;
			}
		} while (!(larg->cmn_args.qs_map & (1 << ((tc * mvapp_pp2_max_num_qs_per_tc) + qid))));

		err  = loop_sw_recycle(larg, 0, 1, tc, qid, num);
		err |= loop_sw_recycle(larg, 1, 0, tc, qid, num);
		if (err != 0)
			return err;
	}

	return 0;
}

static int main_loop_cb(void *arg, int *running)
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
	struct pp2_glb_common_args	*pp2_args = (struct pp2_glb_common_args *)garg.cmn_args.plat;
	struct pp2_init_params		 pp2_params;
	char				 file[PP2_MAX_BUF_STR_LEN];
	int				 err;
	int				 num_rss_tables = 0;

	pr_info("Global initializations ...\n");

	err = mv_sys_dma_mem_init(DMA_ECHO_APP_DMA_MEM_SIZE);
	if (err)
		return err;

	memset(&pp2_params, 0, sizeof(pp2_params));

	/* Check how many RSS tables are in use by kernel. This parameter is needed for configuring RSS */
	/* Relevant only if cpus is bigger than 1 */
	if (garg.cmn_args.cpus > 1) {
		sprintf(file, "%s/%s", PP2_SYSFS_RSS_PATH, PP2_SYSFS_RSS_NUM_TABLES_FILE);
		num_rss_tables = app_pp2_sysfs_param_get(pp2_args->ports_desc[0].name, file);
		if (num_rss_tables < 0) {
			pr_err("Failed to read kernel RSS tables. Please check mvpp2x_sysfs.ko is loaded\n");
			return -EFAULT;
		}
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
	struct pp2_glb_common_args	*pp2_args = (struct pp2_glb_common_args *) garg->cmn_args.plat;
	struct bpool_inf		 std_infs[] = DMA_ECHO_APP_BPOOLS_INF;
	struct bpool_inf		 jumbo_infs[] = DMA_ECHO_APP_BPOOLS_JUMBO_INF;
	struct bpool_inf		*infs;
	int				 err, port_index;
	int				 i;

	pr_info("Specific modules initializations\n");

	err = app_hif_init(&pp2_args->hif, DMA_ECHO_APP_HIF_Q_SIZE, NULL);
	if (err)
		return err;

	if (garg->cmn_args.mtu > DEFAULT_MTU) {
		infs = jumbo_infs;
		pp2_args->num_pools = ARRAY_SIZE(jumbo_infs);
	} else {
		infs = std_infs;
		pp2_args->num_pools = ARRAY_SIZE(std_infs);
	}
	/* Calculate total number of buffers in the pools */
	garg->num_bufs = 0;
	for (i = 0; i < pp2_args->num_pools; i++)
		garg->num_bufs += infs[i].num_buffs;

	err = app_build_all_bpools(&pp2_args->pools_desc, pp2_args->num_pools, infs, pp2_args->hif);
	if (err)
		return err;

	for (port_index = 0; port_index < garg->cmn_args.num_ports; port_index++) {
		struct port_desc *port = &pp2_args->ports_desc[port_index];

		err = app_find_port_info(port);
		if (!err) {
			port->ppio_type	= PP2_PPIO_T_NIC;
			port->num_tcs	= DMA_ECHO_APP_MAX_NUM_TCS_PER_PORT;
			for (i = 0; i < port->num_tcs; i++)
				port->num_inqs[i] = garg->cmn_args.cpus;
			port->inq_size	= DMA_ECHO_APP_RX_Q_SIZE;
			port->num_outqs	= DMA_ECHO_APP_MAX_NUM_TCS_PER_PORT;
			port->outq_size	= DMA_ECHO_APP_TX_Q_SIZE;
			port->first_inq	= DMA_ECHO_APP_FIRST_INQ;
			if (garg->cmn_args.cpus == 1)
				port->hash_type = PP2_PPIO_HASH_T_NONE;
			else
				port->hash_type = PP2_PPIO_HASH_T_5_TUPLE;

			/* pkt_offset=0 not to be changed, before recoding rx_data_path to use pkt_offset as well */
			err = app_port_init(port, pp2_args->num_pools,
					    pp2_args->pools_desc[port->pp_id], garg->cmn_args.mtu, 0);
			if (err) {
				pr_err("Failed to initialize port %d (pp_id: %d)\n", port_index,
				       port->pp_id);
				return err;
			}
		} else {
			return err;
		}
	}

	return 0;
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
		printf("%d\n", garg->cmn_args.prefetch_shift);
		return 0;
	}

	garg->cmn_args.prefetch_shift = atoi(argv[1]);

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

	return apps_perf_dump(&garg->cmn_args);
}

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

static int unregister_cli_cmds(struct glob_arg *garg)
{
	/* TODO: unregister cli cmds */
	return 0;
}

static int register_cli_cmds(struct glob_arg *garg)
{
	struct cli_cmd_params	 cmd_params;

	app_register_cli_common_cmds(&garg->cmn_args);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "prefetch";
	cmd_params.desc		= "Prefetch ahead shift (number of buffers)";
	cmd_params.format	= "<shift>";
	cmd_params.cmd_arg	= garg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))apps_prefetch_cmd_cb;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "perf";
	cmd_params.desc		= "Dump performance statistics";
	cmd_params.format	= NULL;
	cmd_params.cmd_arg	= garg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))perf_cmd_cb;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "pme";
	cmd_params.desc		= "Performance Montitor Emulator";
	cmd_params.format	= NULL;
	cmd_params.cmd_arg	= garg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pme_cmd_cb;
	mvapp_register_cli_cmd(&cmd_params);

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

	gettimeofday(&garg->cmn_args.ctrl_trd_last_time, NULL);

	pme_ev_cnt_rx = pme_ev_cnt_create("PP-IO Recv", 1000000, 0);
	if (pme_ev_cnt_rx < 0) {
		pr_err("PME failed!\n");
		return pme_ev_cnt_rx;
	}
	pme_ev_cnt_enq = pme_ev_cnt_create("DMAX2 EnQ", 1000000, 0);
	if (pme_ev_cnt_enq < 0) {
		pr_err("PME failed!\n");
		return pme_ev_cnt_enq;
	}
	pme_ev_cnt_deq = pme_ev_cnt_create("DMAX2 DeQ", 1000000, 0);
	if (pme_ev_cnt_deq < 0) {
		pr_err("PME failed!\n");
		return pme_ev_cnt_deq;
	}
	pme_ev_cnt_tx = pme_ev_cnt_create("PP-IO Send", 1000000, 0);
	if (pme_ev_cnt_tx < 0) {
		pr_err("PME failed!\n");
		return pme_ev_cnt_tx;
	}

	return 0;
}

static int init_local(void *arg, int id, void **_larg)
{
	struct glob_arg			*garg = (struct glob_arg *)arg;
	struct local_arg		*larg;
	struct pp2_glb_common_args	*glb_pp2_args = (struct pp2_glb_common_args *) garg->cmn_args.plat;
	struct pp2_lcl_common_args	*lcl_pp2_args;
	struct dmax2_params		 dmax2_params;
	char				 name[15];
	int				 i, err;

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
	memset(larg, 0, sizeof(struct local_arg));

	larg->cmn_args.plat = (struct pp2_lcl_common_args *)malloc(sizeof(struct pp2_lcl_common_args));
	if (!larg) {
		pr_err("No mem for local plat arg obj!\n");
		free(larg);
		return -ENOMEM;
	}

	lcl_pp2_args = (struct pp2_lcl_common_args *) larg->cmn_args.plat;
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
	err = app_hif_init(&lcl_pp2_args->hif, DMA_ECHO_APP_HIF_Q_SIZE, NULL);
	pthread_mutex_unlock(&garg->trd_lock);
	if (err)
		return err;

	larg->cmn_args.id               = id;
	larg->cmn_args.verbose		= garg->cmn_args.verbose;
	larg->cmn_args.burst		= garg->cmn_args.burst;
	larg->cmn_args.echo             = garg->cmn_args.echo;
	larg->cmn_args.prefetch_shift   = garg->cmn_args.prefetch_shift;
	larg->cmn_args.num_ports        = garg->cmn_args.num_ports;

	lcl_pp2_args->lcl_ports_desc = (struct lcl_port_desc *)
					malloc(larg->cmn_args.num_ports * sizeof(struct lcl_port_desc));
	if (!lcl_pp2_args->lcl_ports_desc) {
		pr_err("no mem for local-port-desc obj!\n");
		return -ENOMEM;
	}
	memset(lcl_pp2_args->lcl_ports_desc, 0, larg->cmn_args.num_ports * sizeof(struct lcl_port_desc));
	for (i = 0; i < larg->cmn_args.num_ports; i++)
		app_port_local_init(i, larg->cmn_args.id,
				    &lcl_pp2_args->lcl_ports_desc[i], &glb_pp2_args->ports_desc[i]);

	lcl_pp2_args->pools_desc = glb_pp2_args->pools_desc;

	err = mv_stack_create(garg->num_bufs, &larg->stack_hndl);
	if (err) {
		pr_err("failed to create Stack for %d elements!\n", garg->num_bufs);
		return -EFAULT;
	}
	/* Allocate packet metadata structures for all buffers */
	for (i = 0; i < garg->num_bufs; i++) {
		struct pkt_mdata *mdata = malloc(sizeof(struct pkt_mdata));

		if (!mdata) {
			pr_warn("Can't allocate all metadata structures\n");
			break;
		}
		mv_stack_push(larg->stack_hndl, mdata);
	}
	pr_debug("%d of %d metadata structures allocated\n", i, garg->num_bufs);

	larg->cmn_args.garg = garg;
	garg->cmn_args.largs[larg->cmn_args.id] = larg;

	larg->cmn_args.qs_map = garg->cmn_args.qs_map << (garg->cmn_args.qs_map_shift * larg->cmn_args.id);
	pr_debug("thread %d (cpu %d) mapped to Qs %llx\n",
		larg->cmn_args.id, sched_getcpu(), (unsigned int long long)larg->cmn_args.qs_map);

	/* Init the DMA-XORv2 module */
	sprintf(name, "dmax2-%d", garg->engine + larg->cmn_args.id);
	dmax2_params.match = name;
	dmax2_params.queue_size = DMA_ECHO_APP_DMAX2_Q_SIZE;
	err = dmax2_init(&dmax2_params, &larg->dmax2);
	if (err)
		return err;

	*_larg = larg;
	return 0;
}

static void deinit_local(void *arg)
{
	struct local_arg		*larg = (struct local_arg *)arg;
	struct pp2_lcl_common_args	*pp2_args = (struct pp2_lcl_common_args *) larg->cmn_args.plat;
	int				 i;

	if (!larg)
		return;

	if (larg->dmax2)
		dmax2_deinit(larg->dmax2);

	if (larg->stack_hndl) {
		/* Free all pkt_mdata structures and delete stack instance */
		while (!mv_stack_is_empty(larg->stack_hndl)) {
			struct pkt_mdata *mdata = mv_stack_pop(larg->stack_hndl);

			if (!mdata)
				break;

			free(mdata);
		}
		mv_stack_delete(larg->stack_hndl);
	}

	if (pp2_args->lcl_ports_desc) {
		for (i = 0; i < larg->cmn_args.num_ports; i++)
			app_port_local_deinit(&pp2_args->lcl_ports_desc[i]);
		free(pp2_args->lcl_ports_desc);
	}

	if (pp2_args->hif)
		pp2_hif_deinit(pp2_args->hif);
	free(larg);
}

static void usage(char *progname)
{
	printf("\n"
	       "MUSDK crypto-echo application.\n"
	       "\n"
	       "Usage: %s OPTIONS\n"
	       "  E.g. %s -i eth0,eth1 -c 1\n"
	       "\n"
	       "Mandatory OPTIONS:\n"
	       "\t-i, --interface <Eth-interfaces> (comma-separated, no spaces)\n"
	       "                  Interface count min 1, max %i\n"
	       "\n"
	       "Optional OPTIONS:\n"
	       "\t-b             <size>    Burst size (default is %d)\n"
	       "\t-c, --cores    <number>  Number of CPUs to use.\n"
	       "\t-a, --affinity <number>  Use setaffinity (default is no affinity).\n"
	       "\t-t             <mtu>     Set MTU (default is %d)\n"
#ifdef DMA_ECHO_APP_VERBOSE_DEBUG
	       "\t-v                       Increase verbose debug (default is 0).\n"
	       "\t                         With every '-v', the debug is increased by one.\n"
	       "\t                         0 - none, 1 - pkts sent/recv indication, 2 - full pkt dump\n"
#endif /* DMA_ECHO_APP_VERBOSE_DEBUG */
	       "\t-e, --engine <DMA-engine-#> DMA-XOR-v2 Interface number: min 0, max %i (default 0)\n"
	       "\t--no-echo                No Echo packets\n"
	       "\t--cli                    Use CLI\n"
	       "\t?, -h, --help            Display help and exit.\n\n"
	       "\n", MVAPPS_NO_PATH(progname), MVAPPS_NO_PATH(progname),
	       MVAPPS_PP2_MAX_NUM_PORTS, DMA_ECHO_APP_MAX_BURST_SIZE, DEFAULT_MTU, DMAX2_INTERFACE_COUNT-1);
}

static int parse_args(struct glob_arg *garg, int argc, char *argv[])
{
	int	i = 1;
	struct pp2_glb_common_args *pp2_args = (struct pp2_glb_common_args *) garg->cmn_args.plat;

	garg->cmn_args.verbose = 0;
	garg->cmn_args.cli = 0;
	garg->cmn_args.cpus = 1;
	garg->cmn_args.affinity = -1;
	garg->cmn_args.burst = DMA_ECHO_APP_DFLT_BURST_SIZE;
	garg->cmn_args.mtu = DEFAULT_MTU;
	garg->cmn_args.echo = 1;
	garg->cmn_args.qs_map = 0;
	garg->cmn_args.qs_map_shift = 0;
	garg->cmn_args.pkt_offset = 0;
	garg->cmn_args.prefetch_shift = DMA_ECHO_APP_PREFETCH_SHIFT;
	pp2_args->pp2_num_inst = pp2_get_num_inst();
	garg->cmn_args.ctrl_thresh = DMA_ECHO_APP_CTRL_DFLT_THR;
	garg->engine = 0;

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
		} else if (strcmp(argv[i], "-t") == 0) {
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
		} else if (strcmp(argv[i], "-m") == 0) {
			char *token;

			if (argc < (i + 2)) {
				pr_err("Invalid number of arguments!\n");
				return -EINVAL;
			}
			if (argv[i + 1][0] == '-') {
				pr_err("Invalid arguments format!\n");
				return -EINVAL;
			}
			token = strtok(argv[i + 1], ":");
			garg->cmn_args.qs_map = strtoul(token, NULL, 16);
			token = strtok(NULL, "");
			garg->cmn_args.qs_map_shift = atoi(token);
			i += 2;
#ifdef DMA_ECHO_APP_VERBOSE_DEBUG
		} else if (strcmp(argv[i], "-v") == 0) {
			garg->cmn_args.verbose++;
			i += 1;
#endif /* DMA_ECHO_APP_VERBOSE_DEBUG */
		} else if ((strcmp(argv[i], "-e") == 0) || (strcmp(argv[i], "--engine") == 0)) {
			if (argc < (i+2)) {
				pr_err("Invalid number of arguments!\n");
				return -EINVAL;
			}
			garg->engine = atoi(argv[i+1]);
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
	if (garg->cmn_args.burst > DMA_ECHO_APP_MAX_BURST_SIZE) {
		pr_err("illegal burst size requested (%d vs %d)!\n",
		       garg->cmn_args.burst, DMA_ECHO_APP_MAX_BURST_SIZE);
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
	    (mvapp_pp2_max_num_qs_per_tc == 1) &&
	    (DMA_ECHO_APP_MAX_NUM_TCS_PER_PORT == 1)) {
		pr_warn("no point in queues-mapping; ignoring.\n");
		garg->cmn_args.qs_map = 1;
		garg->cmn_args.qs_map_shift = 1;
	} else if (!garg->cmn_args.qs_map) {
		garg->cmn_args.qs_map = 1;
		garg->cmn_args.qs_map_shift = DMA_ECHO_APP_MAX_NUM_TCS_PER_PORT;
	}

	if ((garg->cmn_args.cpus != 1) &&
	    (garg->cmn_args.qs_map & (garg->cmn_args.qs_map << garg->cmn_args.qs_map_shift))) {
		pr_err("Invalid queues-mapping (ovelapping CPUs)!\n");
		return -EINVAL;
	}

	if (garg->cmn_args.prefetch_shift > garg->cmn_args.burst)
		garg->cmn_args.prefetch_shift = garg->cmn_args.burst - 1;

	return 0;
}


int main(int argc, char *argv[])
{
	struct mvapp_params	mvapp_params;
	u64			cores_mask;
	int			i, err;

	setbuf(stdout, NULL);
	app_set_max_num_qs_per_tc();

	pr_debug("pr_debug is enabled\n");

	garg.cmn_args.plat = (struct pp2_glb_common_args *)malloc(sizeof(struct pp2_glb_common_args));
	if (!garg.cmn_args.plat) {
		pr_err("No mem for global plat arg obj!\n");
		return -ENOMEM;
	}
	err = parse_args(&garg, argc, argv);
	if (err) {
		free(garg.cmn_args.plat);
		return err;
	}

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
	mvapp_params.deinit_local_cb	= deinit_local;
	mvapp_params.main_loop_cb	= main_loop_cb;
	if (!mvapp_params.use_cli)
		mvapp_params.ctrl_cb	= app_ctrl_cb;
	return mvapp_go(&mvapp_params);
}
