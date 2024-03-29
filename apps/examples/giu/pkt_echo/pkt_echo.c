/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#define _GNU_SOURCE         /* See feature_test_macros(7) */
#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <sched.h>
#include <fcntl.h>
#include <sys/time.h>

#include "mv_std.h"
#include "lib/lib_misc.h"
#include "lib/file_utils.h"
#include "env/mv_sys_dma.h"

#include "mvapp.h"
#include "lib/mv_pme.h"
#include "mv_pp2.h"
#include "mv_pp2_hif.h"
#include "mv_pp2_bpool.h"
#include "mv_pp2_ppio.h"

#include "pp2_utils.h"

#include "nmp_guest_utils.h"
#include "mv_giu_gpio.h"
#include "giu_utils.h"
#include "mng/mv_nmp.h"
#include "mng/mv_nmp_guest.h"


#define PKT_ECHO_APP_MAX_NUM_CORES		3

#define PKT_ECHO_APP_TX_RETRY_MAX		3
#define PKT_ECHO_APP_DEF_Q_SIZE			1024
#define PKT_ECHO_APP_HIF_Q_SIZE			(8 * PKT_ECHO_APP_DEF_Q_SIZE)
#define PKT_ECHO_APP_RX_Q_SIZE			(2 * PKT_ECHO_APP_DEF_Q_SIZE)
#define PKT_ECHO_APP_TX_Q_SIZE			(2 * PKT_ECHO_APP_DEF_Q_SIZE)

#define PKT_ECHO_APP_GIU_BP_SIZE		4096

#define PKT_ECHO_APP_NET_INTERFACE		"eth0"

#define PKT_ECHO_APP_MAX_BURST_SIZE		((PKT_ECHO_APP_RX_Q_SIZE) >> 2)
/* as GIU is the bottleneck, set the burst size to GIU_Q_SIZE / 4 */
#define PKT_ECHO_APP_DFLT_BURST_SIZE		(PKT_ECHO_APP_MAX_BURST_SIZE >> 1)

#define PKT_ECHO_APP_DMA_MEM_SIZE		(80 * 1024 * 1024)
#define PKT_ECHO_APP_STATS_DFLT_THRESH		1000
#define PKT_ECHO_APP_CTRL_TRD_THRESH		200

#define PKT_ECHO_APP_FIRST_INQ			0
#define PKT_ECHO_APP_MAX_NUM_TCS_PER_PORT	1
#define PKT_ECHO_APP_MAX_NUM_QS_PER_CORE	PKT_ECHO_APP_MAX_NUM_TCS_PER_PORT

#define PKT_ECHO_APP_PKT_ECHO_SUPPORT
#define PKT_ECHO_APP_PREFETCH_SHIFT		7

#define PKT_ECHO_APP_BPOOLS_INF		{ {384, 4096, 0, NULL}, {2048, 4096, 0, NULL} }
#define PKT_ECHO_APP_BPOOLS_JUMBO_INF	{ {2048, 4096}, {10240, 512} }

#define QUEUE_OCCUPANCY(prod, cons, q_size)	\
	(((prod) - (cons) + (q_size)) & ((q_size) - 1))

#define QUEUE_SPACE(prod, cons, q_size)	\
	((q_size) - QUEUE_OCCUPANCY((prod), (cons), (q_size)) - 1)

/* NMP Guest ID */
#define PKT_ECHO_APP_NMP_GUEST_ID	2
/* NMP Guest Timeout (ms)*/
#define PKT_ECHO_APP_NMP_GUEST_TIMEOUT	10000

#define CHECK_CYCLES
#ifdef CHECK_CYCLES
#define START_COUNT_CYCLES(_ev_cnt)		pme_ev_cnt_start(_ev_cnt)
#define STOP_COUNT_CYCLES(_ev_cnt, _num)	pme_ev_cnt_stop(_ev_cnt, _num)
#else
#define START_COUNT_CYCLES(_ev_cnt)
#define STOP_COUNT_CYCLES(_ev_cnt, _num)
#endif /* CHECK_CYCLES */


/* When pkt-echo is running as part of the management application
 * the egress/ingress loop code is running as part of the management scheduling.
 * Therefore, we would like to limit it to a single pass/loop over the
 * ingress/egress (and not infinite loop at it should be in standalone app)
 */

struct glob_arg {
	struct glb_common_args		 cmn_args; /* Keep first */

	u16				 rxq_size;
	int				 loopback;
	int				 maintain_stats;
	int				 pkt_rate_stats;
	pthread_mutex_t			 trd_lock;

	struct giu_bpools_desc		 giu_bpools_desc;
	struct giu_port_desc		 giu_port_desc;
	struct nmp			*nmp;
	struct nmp_guest		*nmp_guest;
	struct nmp_guest_info		guest_info;
	char				*prb_str;
};

struct local_arg {
	struct local_common_args	 cmn_args; /* Keep first */

	struct lcl_giu_port_desc	 giu_ports_desc[MVAPPS_GIU_MAX_NUM_PORTS];
};

static struct glob_arg garg = {};

#ifdef CHECK_CYCLES
static int pme_ev_cnt_pp2_rx = -1, pme_ev_cnt_pp2_tx = -1, pme_ev_cnt_pp2_txd = -1;
static int pme_ev_cnt_giu_rx = -1, pme_ev_cnt_giu_tx = -1, pme_ev_cnt_giu_txd = -1;
static int pme_ev_cnt_gie_ingr = -1, pme_ev_cnt_gie_egr = -1;
#endif /* CHECK_CYCLES */

#define PKT_ECHO_APP_INC_RX_COUNT(core, port, cnt)		(rx_buf_cnt[core][port] += cnt)
#define PKT_ECHO_APP_INC_TX_COUNT(core, port, cnt)		(tx_buf_cnt[core][port] += cnt)
#define PKT_ECHO_APP_INC_TX_RETRY_COUNT(core, port, cnt)	(tx_buf_retry[core][port] += cnt)
#define PKT_ECHO_APP_INC_TX_DROP_COUNT(core, port, cnt)	(tx_buf_drop[core][port] += cnt)
#define PKT_ECHO_APP_INC_FREE_COUNT(core, port, cnt)		(free_buf_cnt[core][port] += cnt)
#define PKT_ECHO_APP_SET_MAX_RESENT(core, port, cnt)		\
	{ if (cnt > tx_max_resend[core][port]) tx_max_resend[core][port] = cnt; }

#define PKT_ECHO_APP_SET_MAX_BURST(core, port, burst)	\
	{ if (burst > tx_max_burst[core][port]) tx_max_burst[core][port] = burst; }

u32 rx_buf_cnt[MVAPPS_MAX_NUM_CORES][MVAPPS_PP2_MAX_NUM_PORTS];
u32 free_buf_cnt[MVAPPS_MAX_NUM_CORES][MVAPPS_PP2_MAX_NUM_PORTS];
u32 tx_buf_cnt[MVAPPS_MAX_NUM_CORES][MVAPPS_PP2_MAX_NUM_PORTS];
u32 tx_buf_drop[MVAPPS_MAX_NUM_CORES][MVAPPS_PP2_MAX_NUM_PORTS];
u32 tx_buf_retry[MVAPPS_MAX_NUM_CORES][MVAPPS_PP2_MAX_NUM_PORTS];
u32 tx_max_resend[MVAPPS_MAX_NUM_CORES][MVAPPS_PP2_MAX_NUM_PORTS];
u32 tx_max_burst[MVAPPS_MAX_NUM_CORES][MVAPPS_PP2_MAX_NUM_PORTS];

/* globals for ingress/egress packet rate statistics */
struct timeval		 ctrl_trd_last_time;
u64			 lst_rx_cnt[MVAPPS_PP2_MAX_NUM_PORTS];
u64			 lst_tx_cnt[MVAPPS_PP2_MAX_NUM_PORTS];


static int dump_perf(struct glob_arg *garg)
{
	struct timeval	 curr_time;
	u64		 tmp_time_inter;
	u32		 tmp_rx_cnt[MVAPPS_PP2_MAX_NUM_PORTS];
	u32		 tmp_tx_cnt[MVAPPS_PP2_MAX_NUM_PORTS];
	u32		 drop_cnt[MVAPPS_PP2_MAX_NUM_PORTS];
	int		 i, j;

	gettimeofday(&curr_time, NULL);
	tmp_time_inter = (curr_time.tv_sec - ctrl_trd_last_time.tv_sec) * 1000;
	tmp_time_inter += (curr_time.tv_usec - ctrl_trd_last_time.tv_usec) / 1000;

	for (j = 0; j < 2; j++) {
		drop_cnt[j] = 0;
		for (i = 0; i < garg->cmn_args.cpus; i++)
			drop_cnt[j] += tx_buf_drop[i][j];
	}
	for (j = 0; j < 2; j++) {
		tmp_rx_cnt[j] = tmp_tx_cnt[j] = 0;
		for (i = 0; i < garg->cmn_args.cpus; i++) {
			tmp_rx_cnt[j] += rx_buf_cnt[i][j];
			tmp_tx_cnt[j] += tx_buf_cnt[i][j];
		}
	}
	printf("Perf:");
	for (j = 0; j < 2; j++) {
		printf("%s: %dKpps (Rx: %dKpps)\t",
			j ? " egress" : " ingress",
			(int)((tmp_tx_cnt[j] - lst_tx_cnt[j]) / tmp_time_inter),
			(int)((tmp_rx_cnt[j] - lst_rx_cnt[j]) / tmp_time_inter));
		lst_rx_cnt[j] = tmp_rx_cnt[j];
		lst_tx_cnt[j] = tmp_tx_cnt[j];
		if (drop_cnt[j])
			printf(", drop: %u", drop_cnt[j]);
	}
	printf("\n");
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

#ifdef CHECK_CYCLES
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

	printf("Ingress:\n");
	pme_ev_cnt_dump(pme_ev_cnt_pp2_rx, 1);
	pme_ev_cnt_dump(pme_ev_cnt_giu_tx, 1);
	pme_ev_cnt_dump(pme_ev_cnt_pp2_txd, 1);
	pme_ev_cnt_dump(pme_ev_cnt_gie_ingr, 1);
	printf("Egress:\n");
	pme_ev_cnt_dump(pme_ev_cnt_gie_egr, 1);
	pme_ev_cnt_dump(pme_ev_cnt_giu_rx, 1);
	pme_ev_cnt_dump(pme_ev_cnt_pp2_tx, 1);
	pme_ev_cnt_dump(pme_ev_cnt_giu_txd, 1);
	return 0;
}
#endif /* CHECK_CYCLES */

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

	PKT_ECHO_APP_INC_FREE_COUNT(rx_port->lcl_id, rx_port->id, free_cnt);
	return idx;
}

/* During ingress flow, the PP2 (Rx side) saves the buffers in its shadow Q.
 * These buffers are taken from PP2 during receive and used by GIU during transmit.
 * In addition, the shadow Q also contains the PP2 BPool.
 *
 * In this function the transmitted buffers are being placed back to the PP2 BPool.
 */
static inline u16 pp2_free_multi_buffers(struct lcl_port_desc		*rx_port,
					 struct lcl_giu_port_desc	*tx_port,
					 struct pp2_hif			*hif,
					 u16				start_idx,
					 u16				num,
					 u8				tc)
{
	u16			idx = start_idx;
	u16			cont_in_shadow, req_num;
	struct tx_shadow_q	*shadow_q;

	shadow_q = &rx_port->shadow_qs[tc];

	cont_in_shadow = rx_port->shadow_q_size - start_idx;

	if (num <= cont_in_shadow) {
		req_num = num;
		pp2_bpool_put_buffs(hif, (struct buff_release_entry *)&shadow_q->ents[idx], &req_num);
		idx = idx + num;
		if (idx == rx_port->shadow_q_size)
			idx = 0;
	} else {
		req_num = cont_in_shadow;
		pp2_bpool_put_buffs(hif, (struct buff_release_entry *)&shadow_q->ents[idx], &req_num);

		req_num = num - cont_in_shadow;
		pp2_bpool_put_buffs(hif, (struct buff_release_entry *)&shadow_q->ents[0], &req_num);
		idx = num - cont_in_shadow;
	}

	PKT_ECHO_APP_INC_FREE_COUNT(rx_port->lcl_id, rx_port->id, num);

	return idx;
}

/* This function is called by during Ingress flow.
 * In this flow the PP2 is the Rx side (as it reads the packets from the network)
 * and the GIU is the Tx side (Transmits the packet to the host over PCI).
 * The PP2 is allocating the buffers (which GIU uses in its packets) so the
 * buffers should be put back to the PP2 BPool.
 * Therefore, the flow is:
 *	- Read how many packets the GIU has transmitted (since last time it
 *	  was sampled).
 *	- Use the shadow Q (of Rx side) to get the (PP2) BPool and buffers.
 *	- Put back the buffers to the PP2 BPool.
 *
 * Note: some of the functionality described above is done in the inner functions
 *	 (called by this function).
 */
static inline void pp2_free_sent_buffers(struct lcl_port_desc		*rx_port,
					 struct lcl_giu_port_desc	*tx_port,
					 struct pp2_hif			*hif,
					 u8				tc,
					 u8				qid)
{
	u16 tx_num;

	START_COUNT_CYCLES(pme_ev_cnt_pp2_txd);
	/* Read from giu how many packets were already transmitted
	 * so their buffers can be put back to the pp2 bpool
	 */
	giu_gpio_get_num_outq_done(tx_port->gpio, tc, qid, &tx_num);

	/* No buffer to release */
	if (tx_num == 0) {
		STOP_COUNT_CYCLES(pme_ev_cnt_pp2_txd, 0);
		return;
	}

	/* Return back the buffers to pp2 */
	rx_port->shadow_qs[tc].read_ind =
		pp2_free_multi_buffers(rx_port, tx_port, hif, rx_port->shadow_qs[tc].read_ind, tx_num, tc);

	STOP_COUNT_CYCLES(pme_ev_cnt_pp2_txd, tx_num);
}

/* This function is called by during Egress flow.
 * In this flow the GIU is the Rx side (as it reads the host packets over PCI)
 * and the PP2 is the Tx side (Transmits the packet to the network).
 * The GIU is allocating the buffers (which PP2 uses in its packets) so the
 * buffers should be put back to the GIU BPool.
 * Therefore, the flow is:
 *	- Read how many packets the PP2 has transmitted (since last time it
 *	  was sampled).
 *	- Use the shadow Q (of Rx side) to get the (GIU) BPool and buffers
 *	- Put back the buffers to the GIU BPool.
 *
 * Note: some of the functionality described above is done in the inner functions
 *	 (called by this function).
 */
static inline void giu_free_sent_buffers(struct lcl_giu_port_desc	*rx_port,
					 struct lcl_port_desc		*tx_port,
					 struct pp2_hif			*hif,
					 u8				tc)
{
	u16 tx_num;

	START_COUNT_CYCLES(pme_ev_cnt_giu_txd);
	/* Read from pp2 how many packets were already transmitted
	 * so their buffers can be put back to the giu bpool
	 */
	pp2_ppio_get_num_outq_done(tx_port->ppio, hif, tc, &tx_num);

	/* No buffer to release */
	if (tx_num == 0) {
		STOP_COUNT_CYCLES(pme_ev_cnt_giu_txd, 0);
		return;
	}

	/* Return back the buffers to giu */
	rx_port->shadow_qs[tc].read_ind =
		giu_free_buffers(rx_port, rx_port->shadow_qs[tc].read_ind, tx_num, tc);

	STOP_COUNT_CYCLES(pme_ev_cnt_giu_txd, tx_num);
}

/* In Ingress flow, the PP2 is the Rx side (as it reads the packets from the network)
 * and the GIU is the Tx side (Transmits the packet to the host over PCI).
 *
 * After the PP2 receives the packets (and allocates the buffers), the descriptors are
 * changed to fit GIU format and then the GIU can transmit them to the host.
 *
 * At the end of the flow the transmitted buffers (may also be from previous egress)
 * are placed back in the PP2 BPool.
 *
 * Therefore, the flow is:
 *	- PP2 receives packets from network.
 *	- The descriptors are manipulated to fit GIU format.
 *	- GIU transmits the packets (and saves the buffers in its shadow).
 *	- The transmitted buffers are placed back to the PP2 BPool.
 *
 * The PP2 (Rx side) shadow Q is used to save the BPool and buffers.
 * After they are transmitted, they can be released.
 */
static inline int loop_sw_ingress(struct local_arg	*larg,
				  u8			 rx_ppio_id,
				  u8			 tx_ppio_id,
				  u8			 tc,
				  u8			 qid,
				  u16			 num)
{
	struct tx_shadow_q	 *shadow_q;
	int			 shadow_q_size;
	struct pp2_ppio_desc	 pp2_descs[PKT_ECHO_APP_MAX_BURST_SIZE]; /* TODO - remove from stack to malloc */
	struct pp2_lcl_common_args	*pp2_args = (struct pp2_lcl_common_args *) larg->cmn_args.plat;
	struct lcl_port_desc		*pp2_port_desc = &(pp2_args->lcl_ports_desc[rx_ppio_id]);
	struct lcl_giu_port_desc	*giu_port_desc = &(larg->giu_ports_desc[tx_ppio_id]);
	struct giu_gpio_desc		*giu_descs;
	u16			 i, tx_num, free_count;
	u16			 desc_idx = 0, cnt = 0;
	u16			 pkt_offset = MVAPPS_PP2_PKT_EFEC_OFFS(pp2_port_desc->pkt_offset[tc]);
	enum pp2_inq_vlan_tag	 vlan_tag;
	enum pp2_inq_l2_cast_type l2_cast;
	enum pp2_inq_l3_cast_type l3_cast;
	enum pp2_inq_l3_type     l3_type;
	enum pp2_inq_l4_type     l4_type;
	enum pp2_inq_desc_status desc_status;
	enum giu_outq_ip_status ip_status = GIU_OUTQ_IPV4_CHECKSUM_UNKNOWN;
	enum giu_outq_l4_status l4_status = GIU_OUTQ_L4_CHECKSUM_UNKNOWN;
	u8                       l3_offset, l4_offset;

	/* Note: PP2 descriptors and GIU descriptors has similar
	 *	 structure so it's ok to use the same descriptors
	 *	 for both interfaces.
	 */
	giu_descs = (struct giu_gpio_desc *)pp2_descs;

	/* Use PP2 Shadow Q to save allocated buffers */
	shadow_q = &pp2_port_desc->shadow_qs[tc];
	shadow_q_size = pp2_port_desc->shadow_q_size;

	/* pr_info("ingress: tid %d check on tc %d, qid %d\n", larg->cmn_args.id, tc, qid); */
	/* pthread_mutex_lock(&larg->garg->trd_lock); */

	/* Make sure that we are not trying to send more packets than the space
	 * we have in the tx-shadow queue.
	 * In this case, simply clip the number of sent packets to the space
	 * available in the tx-shadow queue.
	 */
	free_count = QUEUE_SPACE(shadow_q->write_ind, shadow_q->read_ind, shadow_q_size);
	if (num > free_count)
		num = free_count;

	if (num) {
		START_COUNT_CYCLES(pme_ev_cnt_pp2_rx);
		pp2_ppio_recv(pp2_port_desc->ppio, tc, qid, pp2_descs, &num);
		STOP_COUNT_CYCLES(pme_ev_cnt_pp2_rx, num);
	}

	/* pthread_mutex_unlock(&larg->garg->trd_lock); */
	/* if (num) pr_info("ingress: got %d pkts on tc %d, qid %d\n", num, tc, qid); */

	PKT_ECHO_APP_INC_RX_COUNT(larg->cmn_args.id, 0, num);

	for (i = 0; i < num; i++) {
		char *buff    = (void *)(app_get_high_addr() | pp2_ppio_inq_desc_get_cookie(&pp2_descs[i]));
		dma_addr_t pa = pp2_ppio_inq_desc_get_phys_addr(&pp2_descs[i]);
		u16 len       = pp2_ppio_inq_desc_get_pkt_len(&pp2_descs[i]);

		/* Get pp2 bpool (as the received buffers should be returned to it) */
		void *bpool = pp2_ppio_inq_desc_get_bpool(&pp2_descs[i], pp2_port_desc->ppio);

#if 0
		/* This code displays the packet' buffer */
		char *tmp_buff = (char *)((uintptr_t)(buff));

		tmp_buff += pkt_offset;
		printf("In packet: (@%p,0x%x)\n", buff, pa); mem_disp(tmp_buff, len);
#endif
		/* Get L2 error state */
		desc_status = pp2_ppio_inq_desc_get_l2_pkt_error(&pp2_descs[i]);

		ip_status = GIU_OUTQ_IP_OK;

		/* If no L2 error ,check L3 error state */
		if (!desc_status)
			desc_status = pp2_ppio_inq_desc_get_l3_pkt_error(&pp2_descs[i]);
		if (desc_status)
			ip_status = GIU_OUTQ_IPV4_CHECKSUM_ERR;

		l4_status = GIU_OUTQ_IP_OK;
		/* If no L3 error ,check L4 error state */
		if (!desc_status)
			desc_status = pp2_ppio_inq_desc_get_l4_pkt_error(&pp2_descs[i]);
		if (desc_status)
			l4_status = GIU_OUTQ_L4_CHECKSUM_ERR;

		pp2_ppio_inq_desc_get_vlan_tag(&pp2_descs[i], &vlan_tag);
		pp2_ppio_inq_desc_get_l2_cast_info(&pp2_descs[i], &l2_cast);
		pp2_ppio_inq_desc_get_l3_cast_info(&pp2_descs[i], &l3_cast);

		pp2_ppio_inq_desc_get_l3_info(&pp2_descs[i], &l3_type, &l3_offset);
		pp2_ppio_inq_desc_get_l4_info(&pp2_descs[i], &l4_type, &l4_offset);

		/* Reset the descriptor */
		giu_gpio_outq_desc_reset(&giu_descs[i]);

		/* Update relevant fields in the descriptor */
		giu_gpio_outq_desc_set_phys_addr(&giu_descs[i], pa + pkt_offset);
		giu_gpio_outq_desc_set_pkt_offset(&giu_descs[i], 0);
		giu_gpio_outq_desc_set_pkt_len(&giu_descs[i], len);
		giu_gpio_outq_desc_set_proto_info(&giu_descs[i], ip_status, l4_status, l2_cast, vlan_tag,
							l3_cast, l3_type, l3_offset, l4_type, l4_offset);

		shadow_q->ents[shadow_q->write_ind].buff_ptr.cookie = (uintptr_t)buff;
		shadow_q->ents[shadow_q->write_ind].buff_ptr.addr = pa;
		shadow_q->ents[shadow_q->write_ind].bpool = bpool;
		pr_debug("buff_ptr.cookie(0x%lx)\n", (u64)shadow_q->ents[shadow_q->write_ind].buff_ptr.cookie);
		shadow_q->write_ind++;
		if (shadow_q->write_ind == shadow_q_size)
			shadow_q->write_ind = 0;
	}
	PKT_ECHO_APP_SET_MAX_BURST(larg->cmn_args.id, 0, num);

	/* Work in retry mode: Try to send the packets till all packets were transmitted */
	do {
		tx_num = num;
		if (num) {
			START_COUNT_CYCLES(pme_ev_cnt_giu_tx);
			giu_gpio_send(giu_port_desc->gpio, tc, qid, &giu_descs[desc_idx], &tx_num);
			STOP_COUNT_CYCLES(pme_ev_cnt_giu_tx, tx_num);
			if (num > tx_num) {
				if (!cnt)
					PKT_ECHO_APP_INC_TX_RETRY_COUNT(larg->cmn_args.id, 0, num - tx_num);
				cnt++;
				/* When working in single CPU mode and with a single thread, live lock might
				 * occur in a case there are no free buffers and there are still packets to sent.
				 * This might happen because the pkt-echo tries to send them endlessly and
				 * the buffer free code cannot be called.
				 * Therefore we use a transmit limit and, in case it was reached, we free the buffers
				 * which were not sent.
				 */
				if ((larg->cmn_args.garg->cmn_args.cpus == 1) && (cnt >= PKT_ECHO_APP_TX_RETRY_MAX)) {
					u16 not_sent = num - tx_num;
					/* Free un-sent buffers */
					shadow_q->write_ind = (shadow_q->write_ind < not_sent) ?
							(shadow_q_size - not_sent + shadow_q->write_ind) :
							shadow_q->write_ind - not_sent;
					pp2_free_multi_buffers(pp2_port_desc,
							       giu_port_desc,
							       pp2_args->hif,
							       shadow_q->write_ind,
							       not_sent,
							       tc);
					/* Update statistics */
					PKT_ECHO_APP_INC_TX_COUNT(larg->cmn_args.id, 0, tx_num);
					PKT_ECHO_APP_INC_TX_DROP_COUNT(larg->cmn_args.id, 0, not_sent);
					break;
				}
			}
			desc_idx += tx_num;
			num -= tx_num;
			PKT_ECHO_APP_INC_TX_COUNT(larg->cmn_args.id, 0, tx_num);
		}

		pp2_free_sent_buffers(pp2_port_desc, giu_port_desc, pp2_args->hif, tc, qid);
	} while (num);
	PKT_ECHO_APP_SET_MAX_RESENT(larg->cmn_args.id, 0, cnt);

	return 0;
}

/* In Egress flow, the GIU is the Rx side (as it reads the packets from host over PCI)
 * and the PP2 is the Tx side (Transmits the packet to the network).
 *
 * After the GIU receives the packets (and allocates the buffers), the descriptors are
 * changed to fit PP2 format and then the PP2 can transmit them out.
 *
 * At the end of the flow the transmitted buffers (may also be from previous egress)
 * are placed back in the PP2 BPool.
 *
 * Therefore, the flow is:
 *	- GIU receives packets from host.
 *	- The descriptors are manipulated to fit PP2 format.
 *	- PP2 transmits the packets (and saves the buffers in its shadow).
 *	- The transmitted buffers are placed back to the GIU BPool.
 *
 * The GIU (Rx side) shadow Q is used to save the BPool and buffers.
 * After they are transmitted, they can be released.
 */
static inline int loop_sw_egress(struct local_arg	*larg,
				 u8			 rx_ppio_id,
				 u8			 tx_ppio_id,
				 u8			 tc,
				 u8			 qid,
				 u16			 num)
{
	struct giu_tx_shadow_q	 *shadow_q;
	int			 shadow_q_size;
	struct pp2_ppio_desc	 pp2_descs[PKT_ECHO_APP_MAX_BURST_SIZE]; /* TODO - remove from stack to malloc */
	struct pp2_lcl_common_args	*pp2_args = (struct pp2_lcl_common_args *) larg->cmn_args.plat;
	struct lcl_port_desc		*pp2_port_desc = &(pp2_args->lcl_ports_desc[tx_ppio_id]);
	struct lcl_giu_port_desc	*giu_port_desc = &(larg->giu_ports_desc[rx_ppio_id]);
	struct giu_gpio_desc	*giu_descs;
	u16			 i, tx_num;
	u16			 desc_idx = 0, cnt = 0;
	/* L3/L4 info parameters */
	u32			 l3_type, l4_type;
	u8			 l3_offset, l4_offset;
	int			 gen_l3_chk, gen_l4_chk;

	/* Note: PP2 descriptors and GIU descriptors has similar
	 *	 structure so it's ok to use the same descriptors
	 *	 for both interfaces.
	 */
	giu_descs = (struct giu_gpio_desc *)pp2_descs;

	/* Use GIU Shadow Q to save allocated buffers */
	shadow_q = &giu_port_desc->shadow_qs[tc];
	shadow_q_size = giu_port_desc->shadow_q_size;

	/* pr_info("egress: tid %d check on tc %d, qid %d\n", larg->cmn_args.id, tc, qid); */
	/* pthread_mutex_lock(&larg->garg->trd_lock); */
	START_COUNT_CYCLES(pme_ev_cnt_giu_rx);
	giu_gpio_recv((struct giu_gpio *)giu_port_desc->gpio, tc, qid, giu_descs, &num);
	STOP_COUNT_CYCLES(pme_ev_cnt_giu_rx, num);

	/* pthread_mutex_unlock(&larg->garg->trd_lock); */
	/* if (num) pr_info("egress: got %d pkts on tc %d, qid %d\n", num, tc, qid); */

	PKT_ECHO_APP_INC_RX_COUNT(larg->cmn_args.id, 1, num);

	for (i = 0; i < num; i++) {
		char *buff    = (void *)(app_get_high_addr() | giu_gpio_inq_desc_get_cookie(&giu_descs[i]));
		dma_addr_t pa = giu_gpio_inq_desc_get_phys_addr(&giu_descs[i]);
		u16 len       = giu_gpio_inq_desc_get_pkt_len(&giu_descs[i]);

		/* Get giu bpool (as the received buffers should be returned to it) */
		void *bpool = giu_gpio_inq_desc_get_bpool(&giu_descs[i], giu_port_desc->gpio);

		/* Read L3 info */
		giu_gpio_inq_desc_get_l3_info(&giu_descs[i], &l3_type, &l3_offset, &gen_l3_chk);

		/* Read L4 info */
		giu_gpio_inq_desc_get_l4_info(&giu_descs[i], &l4_type, &l4_offset, &gen_l4_chk);

		/* printf("packet:\n"); mem_disp(tmp_buff, len); */

		/* Reset the descriptor */
		pp2_ppio_outq_desc_reset(&pp2_descs[i]);

		/* Update relevant fields in the descriptor */
		pp2_ppio_outq_desc_set_phys_addr(&pp2_descs[i], pa);
		pp2_ppio_outq_desc_set_pkt_offset(&pp2_descs[i], 0);
		pp2_ppio_outq_desc_set_pkt_len(&pp2_descs[i], len);
		pp2_ppio_outq_desc_set_proto_info(&pp2_descs[i], l3_type, l4_type, l3_offset,
							l4_offset, gen_l3_chk, gen_l4_chk);

		shadow_q->ents[shadow_q->write_ind].buff_inf.cookie = (uintptr_t)buff;
		shadow_q->ents[shadow_q->write_ind].buff_inf.addr = pa;
		shadow_q->ents[shadow_q->write_ind].bpool = bpool;
		pr_debug("buff_ptr.cookie(0x%lx)\n", (u64)shadow_q->ents[shadow_q->write_ind].buff_inf.cookie);

		shadow_q->write_ind++;
		if (shadow_q->write_ind == shadow_q_size)
			shadow_q->write_ind = 0;
	}
	PKT_ECHO_APP_SET_MAX_BURST(larg->cmn_args.id, 1, num);

	/* Work in retry mode: Try to send the packets till all packets were transmitted */
	do {
		tx_num = num;
		if (num) {
			START_COUNT_CYCLES(pme_ev_cnt_pp2_tx);
			pp2_ppio_send(pp2_port_desc->ppio, pp2_args->hif,
				      tc, &pp2_descs[desc_idx], &tx_num);
			STOP_COUNT_CYCLES(pme_ev_cnt_pp2_tx, tx_num);
			if (num > tx_num) {
				if (!cnt)
					PKT_ECHO_APP_INC_TX_RETRY_COUNT(larg->cmn_args.id, 1, num - tx_num);
				cnt++;
			}
			desc_idx += tx_num;
			num -= tx_num;
			PKT_ECHO_APP_INC_TX_COUNT(larg->cmn_args.id, 1, tx_num);
		}
		giu_free_sent_buffers(giu_port_desc, pp2_port_desc, pp2_args->hif, tc);
	} while (num);
	PKT_ECHO_APP_SET_MAX_RESENT(larg->cmn_args.id, 1, cnt);

	return 0;
}

static int main_loop_cb(void *arg, int *running)
{
	struct local_arg	*larg = (struct local_arg *)arg;
	int			 err = 0;
	u16			 num, tmp_num;
	u8			 tc = 0, qid = 0;
	u8			 pp2_port_id = 0, giu_port_id = 0;

	if (!larg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	num = larg->cmn_args.burst;
	while (*running) {
		/* TODO: Find next TC to consume */

		if (larg->cmn_args.garg->cmn_args.cpus == 1) {
			/* when using single CPU, run all routines on it */
			/* PP2 is Rx port and GIU is Tx port */
			err  = loop_sw_ingress(larg, pp2_port_id, giu_port_id, tc, qid, num);
			/* Schedule GIE execution */
			START_COUNT_CYCLES(pme_ev_cnt_gie_egr);
			tmp_num = nmp_schedule(garg.nmp, NMP_SCHED_TX, NULL);
			STOP_COUNT_CYCLES(pme_ev_cnt_gie_egr, tmp_num);
			START_COUNT_CYCLES(pme_ev_cnt_gie_ingr);
			tmp_num = nmp_schedule(garg.nmp, NMP_SCHED_RX, NULL);
			STOP_COUNT_CYCLES(pme_ev_cnt_gie_ingr, tmp_num);
			/* GIU is Rx port and PP2 is Tx port */
			err |= loop_sw_egress(larg, giu_port_id, pp2_port_id, tc, qid, num);
		} else if (larg->cmn_args.id == 0) {
			/* In this case, we either use 2 or 3 CPUs and it is thread0,
			 * run GIE on thread0 and the other routines on the other CPUs.
			 */
			/* Schedule GIE execution */
			START_COUNT_CYCLES(pme_ev_cnt_gie_egr);
			tmp_num = nmp_schedule(garg.nmp, NMP_SCHED_TX, NULL);
			STOP_COUNT_CYCLES(pme_ev_cnt_gie_egr, tmp_num);
			START_COUNT_CYCLES(pme_ev_cnt_gie_ingr);
			tmp_num = nmp_schedule(garg.nmp, NMP_SCHED_RX, NULL);
			STOP_COUNT_CYCLES(pme_ev_cnt_gie_ingr, tmp_num);
		} else {
			/* In this case, we either use 2 or 3 CPUs (but not thread0) so we handle
			 * the second/third thread.
			 */
			if ((larg->cmn_args.garg->cmn_args.cpus == 2) ||
			    (larg->cmn_args.id == 1))
				/* PP2 is Rx port and GIU is Tx port */
				err  = loop_sw_ingress(larg, pp2_port_id, giu_port_id, tc, qid, num);
			if ((larg->cmn_args.garg->cmn_args.cpus == 2) ||
			    (larg->cmn_args.id == 2))
				/* GIU is Rx port and PP2 is Tx port */
				err |= loop_sw_egress(larg, giu_port_id, pp2_port_id, tc, qid, num);
		}
		if (err != 0)
			return err;
	}

	return 0;
}

static int ctrl_cb(void *arg)
{
	struct glob_arg *garg = (struct glob_arg *)arg;

	if (!garg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	nmp_guest_schedule(garg->nmp_guest);

	if (!garg->cmn_args.cli && garg->pkt_rate_stats)
		maintain_stats(garg);

	return 0;
}

static int guest_ev_cb(void *arg, enum nmp_guest_lf_type client, u8 id, u8 code, u16 indx, void *msg, u16 len)
{
	int ret = 0;

	pr_debug("guest_ev_cb was called with: client %d, id %d, code %d, indx %d len %d msg 0x%x\n",
		 client, id, code, indx, len, *(u32 *)msg);
#ifdef DEBUG
	*(u32 *)msg = 0xCDCDCDCD;
#endif /* DEBUG */

	pr_debug("guest_ev_cb Sent Notification msg 0x%x\n", *(u32 *)msg);

	if (client == NMP_GUEST_LF_T_CUSTOM)
		ret = nmp_guest_send_msg(garg.nmp_guest, code, indx, msg, len);

	return ret;
}

static int init_all_modules(void)
{
	struct pp2_init_params	 pp2_params;
	struct pp2_glb_common_args *pp2_args = (struct pp2_glb_common_args *)garg.cmn_args.plat;
	int			err;
	char			file[PP2_MAX_BUF_STR_LEN];
	int			num_rss_tables = 0;
	struct			nmp_params nmp_params;
	struct			nmp_guest_params nmp_guest_params;

	pr_info("Global initializations ...\n");

	err = mv_sys_dma_mem_init(PKT_ECHO_APP_DMA_MEM_SIZE);
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

	/* PP2 initializations */
	memset(&pp2_params, 0, sizeof(pp2_params));

	/* Check how many RSS tables are in use by kernel. This parameter is needed for configuring RSS */
	/* Relevant only if cpus is bigger than 1 */
	if (garg.cmn_args.cpus > 1) {
		num_rss_tables = app_rss_num_tbl_get(pp2_args->ports_desc[0].name, file);
		if (num_rss_tables < 0)
			return -EFAULT;
	}

	pp2_params.rss_tbl_reserved_map = (1 << num_rss_tables) - 1;
	pp2_params.res_maps_auto_detect_map = PP2_RSRVD_MAP_HIF_AUTO | PP2_RSRVD_MAP_BM_POOL_AUTO;

	pp2_params.skip_hw_init = 1;

	err = pp2_init(&pp2_params);
	if (err)
		return err;

	/* Must be after pp2_init */
	app_used_hifmap_init(pp2_params.hif_reserved_map);
	app_used_bm_pool_map_init(pp2_params.bm_pool_reserved_map);

	nmp_guest_get_probe_str(garg.nmp_guest, &garg.prb_str);

	nmp_guest_register_event_handler(garg.nmp_guest,
					 NMP_GUEST_LF_T_NICPF,
					 0,
					 (NMP_GUEST_EV_NICPF_MTU | NMP_GUEST_EV_NICPF_MAC_ADDR),
					 &garg,
					 guest_ev_cb);

	return 0;
}

static int init_local_modules(struct glob_arg *garg)
{
	int				err;
	struct bpool_inf		std_infs[] = PKT_ECHO_APP_BPOOLS_INF;
	struct bpool_inf		jumbo_infs[] = PKT_ECHO_APP_BPOOLS_JUMBO_INF;
	struct bpool_inf		*infs;
	struct pp2_glb_common_args	*pp2_args = (struct pp2_glb_common_args *) garg->cmn_args.plat;

	pr_info("Local initializations ...\n");

	/**************************/
	/* PP2 Port Init	  */
	/**************************/
	err = app_hif_init(&pp2_args->hif, PKT_ECHO_APP_HIF_Q_SIZE, NULL);
	if (err)
		return err;

	if (garg->cmn_args.mtu > DEFAULT_MTU) {
		infs = jumbo_infs;
		pp2_args->num_pools = ARRAY_SIZE(jumbo_infs);
	} else {
		infs = std_infs;
		pp2_args->num_pools = ARRAY_SIZE(std_infs);
	}

	nmp_guest_get_relations_info(garg->nmp_guest, &garg->guest_info);

	err = app_guest_utils_build_all_giu_bpools(garg->prb_str,
			&garg->guest_info,
			&garg->giu_bpools_desc);
	if (err)
		return err;

	err = app_nmp_guest_giu_port_init(garg->prb_str,
			&garg->guest_info,
			&garg->giu_port_desc);
	if (err)
		return err;

	err = app_guest_utils_allocate_all_giu_bpools(&garg->giu_bpools_desc,
			PKT_ECHO_APP_GIU_BP_SIZE);
	if (err)
		return err;

	err = app_guest_utils_build_all_pp2_bpools(garg->prb_str,
			&garg->guest_info,
			&pp2_args->pools_desc,
			pp2_args,
			infs);
	if (err)
		return err;

	err = app_nmp_guest_pp2_port_init(garg->prb_str, &garg->guest_info, &pp2_args->ports_desc[0]);
	if (err)
		return err;

	garg->cmn_args.num_ports = garg->guest_info.ports_info.num_ports;

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

	/* statistics command */
	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "perf";
	cmd_params.desc		= "Dump performance statistics";
	cmd_params.format	= NULL;
	cmd_params.cmd_arg	= garg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))perf_cmd_cb;
	mvapp_register_cli_cmd(&cmd_params);

#ifdef CHECK_CYCLES
	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "pme";
	cmd_params.desc		= "Performance Montitor Emulator";
	cmd_params.format	= NULL;
	cmd_params.cmd_arg	= garg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pme_cmd_cb;
	mvapp_register_cli_cmd(&cmd_params);
#endif /* CHECK_CYCLES */

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

	gettimeofday(&garg->cmn_args.ctrl_trd_last_time, NULL);

#ifdef CHECK_CYCLES
	pme_ev_cnt_pp2_rx = pme_ev_cnt_create("PP2-PPIO Recv", 1000000, 0);
	if (pme_ev_cnt_pp2_rx < 0) {
		pr_err("PME failed!\n");
		return pme_ev_cnt_pp2_rx;
	}
	pme_ev_cnt_pp2_tx = pme_ev_cnt_create("PP2-PPIO Send", 1000000, 0);
	if (pme_ev_cnt_pp2_tx < 0) {
		pr_err("PME failed!\n");
		return pme_ev_cnt_pp2_tx;
	}
	pme_ev_cnt_pp2_txd = pme_ev_cnt_create("PP2-PPIO Done", 1000000, 0);
	if (pme_ev_cnt_pp2_txd < 0) {
		pr_err("PME failed!\n");
		return pme_ev_cnt_pp2_txd;
	}
	pme_ev_cnt_giu_rx = pme_ev_cnt_create("GIU-GPIO Recv", 1000000, 0);
	if (pme_ev_cnt_giu_rx < 0) {
		pr_err("PME failed!\n");
		return pme_ev_cnt_giu_rx;
	}
	pme_ev_cnt_giu_tx = pme_ev_cnt_create("GIU-GPIO Send", 1000000, 0);
	if (pme_ev_cnt_giu_tx < 0) {
		pr_err("PME failed!\n");
		return pme_ev_cnt_giu_tx;
	}
	pme_ev_cnt_giu_txd = pme_ev_cnt_create("GIU-GPIO Done", 1000000, 0);
	if (pme_ev_cnt_giu_txd < 0) {
		pr_err("PME failed!\n");
		return pme_ev_cnt_giu_tx;
	}
	pme_ev_cnt_gie_ingr = pme_ev_cnt_create("GIE Ingress", 1000000, 0);
	if (pme_ev_cnt_gie_ingr < 0) {
		pr_err("PME failed!\n");
		return pme_ev_cnt_gie_ingr;
	}
	pme_ev_cnt_gie_egr = pme_ev_cnt_create("GIE Egress", 1000000, 0);
	if (pme_ev_cnt_gie_egr < 0) {
		pr_err("PME failed!\n");
		return pme_ev_cnt_gie_egr;
	}
#endif /* CHECK_CYCLES */

	return 0;
}

static void deinit_global(void *arg)
{
	apps_pp2_deinit_global(arg);
}

static int init_local(void *arg, int id, void **_larg)
{
	struct glob_arg		*garg = (struct glob_arg *)arg;
	struct local_arg	*larg;
	struct pp2_glb_common_args *glb_pp2_args = (struct pp2_glb_common_args *) garg->cmn_args.plat;
	struct pp2_lcl_common_args *lcl_pp2_args;
	int			giu_port_id = 0;
	int			giu_id = 0; /* TODO: this should not be hard-coded */
	int			i, err;

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
	larg->cmn_args.burst		= garg->cmn_args.burst;
	larg->cmn_args.busy_wait	= garg->cmn_args.busy_wait;
	larg->cmn_args.echo		= garg->cmn_args.echo;
	larg->cmn_args.prefetch_shift	= garg->cmn_args.prefetch_shift;

	larg->cmn_args.plat = (struct pp2_lcl_common_args *)malloc(sizeof(struct pp2_lcl_common_args));
	if (!larg) {
		pr_err("No mem for local plat arg obj!\n");
		free(larg);
		return -ENOMEM;
	}
	lcl_pp2_args = (struct pp2_lcl_common_args *) larg->cmn_args.plat;
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
	err = app_hif_init(&lcl_pp2_args->hif, PKT_ECHO_APP_HIF_Q_SIZE, NULL);
	pthread_mutex_unlock(&garg->trd_lock);
	if (err)
		return err;

	for (i = 0; i < larg->cmn_args.num_ports; i++)
		app_port_local_init(i, larg->cmn_args.id, &lcl_pp2_args->lcl_ports_desc[i],
				    &glb_pp2_args->ports_desc[i]);

	lcl_pp2_args->pools_desc	= glb_pp2_args->pools_desc;
	lcl_pp2_args->multi_buffer_release = glb_pp2_args->multi_buffer_release;

	larg->cmn_args.garg = garg;
	larg->cmn_args.qs_map = garg->cmn_args.qs_map << (garg->cmn_args.qs_map_shift * id);

	/* Update garg refs to local */
	garg->cmn_args.largs[id] = larg;
	for (i = 0; i < larg->cmn_args.num_ports; i++)
		glb_pp2_args->ports_desc[i].lcl_ports_desc[id] = &lcl_pp2_args->lcl_ports_desc[i];

	pr_debug("thread %d (cpu %d) mapped to Qs %llx\n",
		 larg->cmn_args.id, sched_getcpu(), (unsigned long long)larg->cmn_args.qs_map);

	/* TODO: create and use GIU global port descriptor (similar to PP2 port local init) */
	app_giu_port_local_init(giu_port_id,
		larg->cmn_args.id,
		giu_id,
		&larg->giu_ports_desc[giu_id],
		garg->giu_port_desc.gpio);

	*_larg = larg;
	return 0;
}

static void deinit_local(void *arg)
{
	apps_pp2_deinit_local(arg);
}

static void usage(char *progname)
{
	printf("\n"
	       "MUSDK packet-echo application.\n"
	       "\n"
	       "Usage: %s OPTIONS\n"
	       "  E.g. %s -i eth1 -c 1\n"
	       "\n"
	       "Mandatory OPTIONS:\n"
	       "\t-g, --guestid <id>      Guest ID for retrieving parameters from cfg file.\n"
	       "\n"
	       "Optional OPTIONS:\n"
	       "\t-b <size>                Burst size, num_pkts handled in a batch.(default is %d)\n"
	       "\t-i <interface name>      Network Interface to use.(default is %s)\n"
	       "\t--mtu <mtu>              Set MTU (default is %d)\n"
	       "\t-c, --cores <number>     Number of CPUs to use\n"
	       "\t-a, --affinity <number>  Use setaffinity (default is no affinity)\n"
	       "\t-s                       Maintain statistics\n"
	       "\t- f, --file              Location and name of the nmp-config file to load\n"
	       "\t-w <cycles>              Cycles to busy_wait between recv&send, simulating app behavior (default=0)\n"
	       "\t--rxq <size>             Size of rx_queue (default is %d)\n"
	       "\t--pkt-offset <size>      Packet offset in buffer, must be multiple of 32-byte (default is %d)\n"
	       "\t--no-echo                Don't perform 'pkt_echo', N/A w/o define PKT_ECHO_APP_PKT_ECHO_SUPPORT\n"
	       "\t--cli                    Use CLI\n"
	       "\t--no-stat                Disable the packet's runtime statistics display\n"
	       "\t?, -h, --help            Display help and exit.\n\n"
	       "\n", MVAPPS_NO_PATH(progname), MVAPPS_NO_PATH(progname),
	       PKT_ECHO_APP_MAX_BURST_SIZE, PKT_ECHO_APP_NET_INTERFACE,
	       DEFAULT_MTU, PKT_ECHO_APP_RX_Q_SIZE, MVAPPS_PP2_PKT_DEF_OFFS);
}

static int parse_args(struct glob_arg *garg, int argc, char *argv[])
{
	int	i = 1;
	struct pp2_glb_common_args *pp2_args = (struct pp2_glb_common_args *) garg->cmn_args.plat;

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
	garg->cmn_args.ctrl_thresh = PKT_ECHO_APP_STATS_DFLT_THRESH;
	garg->maintain_stats = 0;
	garg->pkt_rate_stats = 1;
	garg->cmn_args.guest_id = PKT_ECHO_APP_NMP_GUEST_ID;

	pp2_args->multi_buffer_release = 1;

	/* TODO: init hardcoded ports?!?!?! */
	garg->cmn_args.num_ports = 1;
	snprintf(pp2_args->ports_desc[0].name,
		sizeof(pp2_args->ports_desc[0].name),
		"%s", PKT_ECHO_APP_NET_INTERFACE);

	while (i < argc) {
		if ((strcmp(argv[i], "?") == 0) ||
		    (strcmp(argv[i], "-h") == 0) ||
		    (strcmp(argv[i], "--help") == 0)) {
			usage(argv[0]);
			exit(0);
		} else if (strcmp(argv[i], "-g") == 0) {
			if (argc < (i + 2)) {
				pr_err("Invalid number of arguments!\n");
				return -EINVAL;
			}
			if (argv[i + 1][0] == '-') {
				pr_err("Invalid arguments format!\n");
				return -EINVAL;
			}
			garg->cmn_args.guest_id = atoi(argv[i + 1]);
			i += 2;
		} else if (strcmp(argv[i], "-f") == 0) {
			if (argc < (i + 2)) {
				pr_err("Invalid number of arguments!\n");
				return -EINVAL;
			}
			if (argv[i + 1][0] == '-') {
				pr_err("Invalid arguments format!\n");
				return -EINVAL;
			}

			strcpy(garg->cmn_args.nmp_cfg_location, argv[i + 1]);
			i += 2;
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
			garg->cmn_args.busy_wait = atoi(argv[i + 1]);
			i += 2;
		} else if (strcmp(argv[i], "--rxq") == 0) {
			garg->rxq_size = atoi(argv[i + 1]);
			i += 2;
		} else if (strcmp(argv[i], "--pkt-offset") == 0) {
			garg->cmn_args.pkt_offset = atoi(argv[i + 1]);
			i += 2;
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
		} else if (strcmp(argv[i], "--no-stat") == 0) {
			garg->pkt_rate_stats = 0;
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
	if (garg->cmn_args.cpus > PKT_ECHO_APP_MAX_NUM_CORES) {
		pr_err("illegal num cores requested (%d vs %d)!\n",
		       garg->cmn_args.cpus, PKT_ECHO_APP_MAX_NUM_CORES);
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

static void init_app_params(struct mvapp_params *mvapp_params, u64 cores_mask)
{
	memset(mvapp_params, 0, sizeof(struct mvapp_params));
	mvapp_params->use_cli		= garg.cmn_args.cli;
	mvapp_params->num_cores		= garg.cmn_args.cpus;
	mvapp_params->cores_mask	= cores_mask;
	mvapp_params->global_arg	= (void *)&garg;
	mvapp_params->init_global_cb	= init_global;
	mvapp_params->deinit_global_cb	= deinit_global;
	mvapp_params->init_local_cb	= init_local;
	mvapp_params->deinit_local_cb	= deinit_local;
	mvapp_params->main_loop_cb	= main_loop_cb;
	mvapp_params->ctrl_cb		= ctrl_cb;
	mvapp_params->ctrl_cb_threshold	= PKT_ECHO_APP_CTRL_TRD_THRESH;
}

static void reset_statistics(void)
{
	int i;

	for (i = 0; i < MVAPPS_MAX_NUM_CORES; i++) {
		int j;

		for (j = 0; j < garg.cmn_args.num_ports; j++) {
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


int main(int argc, char *argv[])
{
	struct mvapp_params		 mvapp_params;
	struct pp2_glb_common_args	*pp2_args;
	u64				 cores_mask;
	int				 err;

	setbuf(stdout, NULL);
	app_set_max_num_qs_per_tc();

	pr_info("GIU pkt-echo is started\n");
	pr_debug("pr_debug is enabled\n");

	garg.cmn_args.plat = (struct pp2_glb_common_args *)malloc(sizeof(struct pp2_glb_common_args));
	if (!garg.cmn_args.plat) {
		pr_err("No mem for global plat arg obj!\n");
		return -ENOMEM;
	}
	pp2_args = (struct pp2_glb_common_args *) garg.cmn_args.plat;

	err = parse_args(&garg, argc, argv);
	if (err)
		return err;

	pp2_args->pp2_num_inst = pp2_get_num_inst();

	cores_mask = apps_cores_mask_create(garg.cmn_args.cpus, garg.cmn_args.affinity);

	init_app_params(&mvapp_params, cores_mask);

	reset_statistics();

	return mvapp_go(&mvapp_params);
}

