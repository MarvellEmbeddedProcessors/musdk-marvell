/******************************************************************************=port->outq_size;
 *      Copyright (C) 2016 Marvell International Ltd.
 *
 *  If you received this File from Marvell, you may opt to use, redistribute
 *  and/or modify this File under the following licensing terms.
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *
 *      * Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *
 *      * Neither the name of Marvell nor the names of its contributors may be
 *        used to endorse or promote products derived from this software
 *        without specific prior written permission.
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

#define MVCONF_DMA_PHYS_ADDR_T_SIZE 64
#define APP_TX_RETRY
#define PKT_ECHO_APP_PKT_ECHO_SUPPORT
#define PKT_ECHO_APP_USE_PREFETCH

#include <linux/types.h>
#include <linux/stddef.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "mv_std.h"
#include "drivers/mv_pp2.h"
#include "drivers/mv_pp2_hif.h"
#include "drivers/mv_pp2_bpool.h"
#include "drivers/mv_pp2_ppio.h"

#define DRIVER_NAME     "pkt_echo"
#define DRIVER_VERSION  "0.1"
#define DRIVER_AUTHOR   "Marvell"
#define DRIVER_DESC     "Marvell User Space Development Kit Packet Echo"

#define MISC_DEV_NAME   "pkt_echo"

#define MVAPPS_MAX_NUM_PORTS		  2
#define DEFAULT_MTU                       1500

#define MVAPPS_MAX_PKT_PROC               2

#define MVAPPS_PP2_NUM_BPOOLS_RSRV        3
#define MVAPPS_PP2_BPOOLS_RSRV            ((1 << MVAPPS_PP2_NUM_BPOOLS_RSRV) - 1)
#define MVAPPS_PP2_MAX_NUM_BPOOLS	  (PP2_BPOOL_NUM_POOLS - MVAPPS_PP2_NUM_BPOOLS_RSRV)

#define MVAPPS_MAX_NUM_QS_PER_TC	  1
#define MVAPPS_MAX_NUM_TCS_PER_PORT	  1
#define BPOOLS_INF			  { {384, 4096}, {2048, 1024} }
#define MAX_NUM_QS_PER_CORE		  MVAPPS_MAX_NUM_QS_PER_TC

#define MVAPPS_DEF_Q_SIZE			1024
#define MVAPPS_HIF_Q_SIZE			(8 * MVAPPS_DEF_Q_SIZE)
#define MVAPPS_RX_Q_SIZE			(2 * MVAPPS_DEF_Q_SIZE)
#define MVAPPS_TX_Q_SIZE			(2 * MVAPPS_DEF_Q_SIZE)
#define MVAPPS_FIRST_MUSDK_IN_QUEUE		0

#define MUSDK_PKT_ECHO_HIF_Q_SIZE	  (8 * MVAPPS_DEF_Q_SIZE)
#define MUSDK_PKT_ECHO_PP2_TOTAL_NUM_HIFS  9
#define MUSDK_PKT_ECHO_PP2_NUM_HIFS_RSRV   4
#define MUSDK_PKT_ECHO_PP2_HIFS_RSRV	  ((1 << MUSDK_PKT_ECHO_PP2_NUM_HIFS_RSRV) - 1)

#define MVAPPS_PPIO_NAME_MAX		  20
#define PP2_MAX_BUF_STR_LEN		  256

#define MVAPPS_PKT_OFFS			64
#define VLAN_HLEN			4
#define ETH_HLEN			14
#define ETH_FCS_LEN			4
#define MVAPPS_MRU_TO_MTU(mru) \
	((mru) - MV_MH_SIZE - VLAN_HLEN - \
	ETH_HLEN - ETH_FCS_LEN)
#define MVAPPS_MTU_TO_MRU(mtu) \
	((mtu) + MV_MH_SIZE + VLAN_HLEN + \
	ETH_HLEN + ETH_FCS_LEN)

#define MAX_BURST_SIZE			(MVAPPS_RX_Q_SIZE >> 1)
#define DFLT_BURST_SIZE                 256
#define PREFETCH_SHIFT                  7
#define MVAPPS_PKT_EFEC_OFFS            (MVAPPS_PKT_OFFS + MV_MH_SIZE)
#define INC_FREE_COUNT(port, cnt)	(free_buf_cnt[port] += cnt)
#define INC_RX_COUNT(port, cnt)		(rx_buf_cnt[port] += cnt)
#define INC_TX_DROP_COUNT(port, cnt)	(tx_buf_drop[port] += cnt)
#define INC_TX_COUNT(port, cnt)		(tx_buf_cnt[port] += cnt)
#define INC_TX_RETRY_COUNT(port, cnt)	(tx_buf_retry[port] += cnt)
#define SET_MAX_BURST(port, burst)	\
	{ if (burst > tx_max_burst[port]) tx_max_burst[port] = burst; }
#define SET_MAX_RESENT(port, cnt)

char *interfaces[] = {"eth0", "eth2"};

struct tx_shadow_q_entry {
	struct pp2_buff_inf	buff_ptr;	/* pointer to the buffer object */
	struct pp2_bpool	*bpool;		/* pointer to the bpool object */
};

struct tx_shadow_q {
	u16				read_ind;	/* read index */
	u16				write_ind;	/* write index */
	struct tx_shadow_q_entry	*ents;		/* array of entries */
};

struct bpool_inf {
	int	buff_size;	/* buffer size */
	int	num_buffs;	/* number of buffers */
};

struct bpool_desc {
	struct pp2_bpool	*pool;		/* pointer to the bpool object */
	struct pp2_buff_inf	*buffs_inf;	/* array of buffer objects */
	int			 num_buffs;	/* number of buffers */
};

struct port_desc {
	char			 name[15];	/* Port name */
	int			 initialized;	/* Flag indicated is port was initialized */
	int			 pp_id;		/* Packet Processor ID */
	int			 ppio_id;	/* PPIO port ID */
	int			 id;
	enum pp2_ppio_type	 ppio_type;	/* PPIO type */
	u32			 first_inq;	/* First RXQ - relative to the Port's first RXQ */
	u16			 num_tcs;	/* Number of TCs */
	u16			 num_inqs[MVAPPS_MAX_NUM_QS_PER_TC];	/* Number of Rx queues */
	u16			 num_outqs;	/* Number of Tx queues */
	u32			 inq_size;	/* Rx queue size */
	u32			 outq_size;	/* Tx queue size */
	u32			 hash_type;	/* Hash type */
	u32			 first_rss_tbl;	/* First RSS table */
	struct pp2_ppio		 *ppio;		/* PPIO object returned by pp2_ppio_init() */
	struct pp2_ppio_params	 port_params;	/* PPIO configuration parameters */
	struct tx_shadow_q	 shadow_qs[MAX_NUM_QS_PER_CORE];
	u32			 shadow_q_size;
};

struct {
	int			num_pools;
	struct bpool_desc	**pools_desc;
	struct port_desc	ports_desc[MVAPPS_MAX_NUM_PORTS];
	struct tx_shadow_q	shadow_qs[MAX_NUM_QS_PER_CORE];
	struct pp2_hif          *hif;
	u8			num_ports;
	u32			hash_type;
	u32			busy_wait;
} app_data;

static u16 used_hifs = MUSDK_PKT_ECHO_PP2_HIFS_RSRV;
static u64 buf_alloc_cnt;
static u16 used_bpools[MVAPPS_MAX_PKT_PROC] = {MVAPPS_PP2_BPOOLS_RSRV, MVAPPS_PP2_BPOOLS_RSRV};
static u64 sys_dma_high_addr;
static struct pp2_ppio_desc descs[MAX_BURST_SIZE];
static u32 free_buf_cnt[MVAPPS_MAX_NUM_PORTS];
static u32 rx_buf_cnt[MVAPPS_MAX_NUM_PORTS];
static u32 tx_max_burst[MVAPPS_MAX_NUM_PORTS];
static u32 tx_buf_cnt[MVAPPS_MAX_NUM_PORTS];
static u32 tx_buf_retry[MVAPPS_MAX_NUM_PORTS];

static inline void swap_l2(char *buf)
{
	u16 *eth_hdr;

	register u16 tmp;

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
	register u32 tmp32;

	buf += 14 + 12;
	tmp32 = ((uint32_t *)buf)[0];
	((uint32_t *)buf)[0] = ((uint32_t *)buf)[1];
	((uint32_t *)buf)[1] = tmp32;
}

static inline u16 free_buffers(struct port_desc		*rx_port,
			       struct port_desc		*tx_port,
			       struct pp2_hif		*hif,
			       u16			 start_idx,
			       u16			 num,
			       u8			 tc)
{
	u16			i, free_cnt = 0, idx = start_idx;
	struct pp2_buff_inf	*binf;
	struct tx_shadow_q	*shadow_q;
	struct pp2_bpool	*bpool;

	shadow_q = &tx_port->shadow_qs[tc];

	for (i = 0; i < num; i++) {
		bpool = shadow_q->ents[idx].bpool;

		binf = &shadow_q->ents[idx].buff_ptr;
		if (unlikely(!binf->cookie || !binf->addr || !bpool)) {
			pr_warn("Shadow memory @%d: cookie(%llx), pa(%llx), pool(%llx)!\n",
				i, (u64)binf->cookie, (u64)binf->addr, (u64)bpool);
			continue;
		}
		pp2_bpool_put_buff(hif, bpool, binf);
		free_cnt++;

		if (++idx == tx_port->shadow_q_size)
			idx = 0;
	}

	INC_FREE_COUNT(rx_port->id, free_cnt);
	return idx;
}

static inline u16 free_multi_buffers(struct port_desc	*rx_port,
				     struct port_desc	*tx_port,
				     struct pp2_hif	*hif,
				     u16		start_idx,
				     u16		num,
				     u8			tc)
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

	INC_FREE_COUNT(rx_port->id, num);

	return idx;
}

static inline void free_sent_buffers(struct port_desc		*rx_port,
				     struct port_desc		*tx_port,
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

int musdk_pkt_echo_port_init(struct port_desc *port, int num_pools, struct bpool_desc *pools, u16 mtu)
{
	struct pp2_ppio_params		*port_params = &port->port_params;
	struct pp2_ppio_inq_params	inq_params;
	char				name[MVAPPS_PPIO_NAME_MAX];
	int				i, j, err = 0;
	u16				curr_mtu;
	static int			id;

	memset(name, 0, sizeof(name));
	snprintf(name, sizeof(name), "ppio-%d:%d",
		 port->pp_id, port->ppio_id);
	pr_debug("found port: %s\n", name);
	port_params->match = name;
	port_params->type = port->ppio_type;
	port_params->inqs_params.num_tcs = port->num_tcs;
	port_params->inqs_params.hash_type = port->hash_type;
	pr_debug("%s: port_params: match=%s type=%d.\n", __func__, name, port->ppio_type);

	port_params->specific_type_params.log_port_params.first_inq = port->first_inq;

	for (i = 0; i < port->num_tcs; i++) {
		port_params->inqs_params.tcs_params[i].pkt_offset = MVAPPS_PKT_OFFS >> 2;
		port_params->inqs_params.tcs_params[i].num_in_qs = port->num_inqs[i];
		inq_params.size = port->inq_size;
		port_params->inqs_params.tcs_params[i].inqs_params = &inq_params;

		for (j = 0; j < num_pools; j++)
			port_params->inqs_params.tcs_params[i].pools[j] = pools[j].pool;
	}
	port_params->outqs_params.num_outqs = port->num_outqs;
	for (i = 0; i < port->num_outqs; i++) {
		port_params->outqs_params.outqs_params[i].size = port->outq_size;
		port_params->outqs_params.outqs_params[i].weight = 1;
	}

	port->shadow_q_size = port->outq_size;
	for (i = 0; i < MAX_NUM_QS_PER_CORE; i++) {
		port->shadow_qs[i].read_ind = 0;
		port->shadow_qs[i].write_ind = 0;
		port->shadow_qs[i].ents = kmalloc_array(port->outq_size, sizeof(struct tx_shadow_q_entry), GFP_KERNEL);
	}

	err = pp2_ppio_init(port_params, &port->ppio);
	if (err) {
		pr_err("PP-IO init failed (error: %d)!\n", err);
		return err;
	}

	if (!port->ppio) {
		pr_err("PP-IO init failed!\n");
		return -EIO;
	}

	if (mtu) {
		/* Change port MTU if needed */
		pp2_ppio_get_mtu(port->ppio, &curr_mtu);
		if (curr_mtu != mtu) {
			pp2_ppio_set_mtu(port->ppio, mtu);
			pp2_ppio_set_mru(port->ppio, MVAPPS_MTU_TO_MRU(mtu));
			pr_info("Set port ppio-%d:%d MTU to %d\n",
				port->pp_id, port->ppio_id, mtu);
		}
	}

	err = pp2_ppio_enable(port->ppio);
	port->initialized = 1;
	port->id = id++;

	return err;
}

int musdk_pkt_echo_find_port_info(struct port_desc *port_desc)
{
	char		name[20];
	u8		pp, ppio;
	int		err;

	if (!port_desc->name || !strlen(port_desc->name)) {
		pr_err("No port name given!\n");
		return -1;
	}

	memset(name, 0, sizeof(name));
	snprintf(name, sizeof(name), "%s", port_desc->name);

	err = pp2_netdev_get_ppio_info(name, &pp, &ppio);
	if (err) {
		pr_err("PP2 Port %s not found!\n", port_desc->name);
		return err;
	}

	port_desc->ppio_id = ppio;
	port_desc->pp_id = pp;
	port_desc->ppio_type = PP2_PPIO_T_NIC;

	return 0;
}

static int musdk_pkt_echo_pp2_init(void)
{
	struct pp2_init_params init_params;

	init_params.hif_reserved_map = 0x000f;
	init_params.bm_pool_reserved_map = 0x0007;
	init_params.rss_tbl_reserved_map = 0x1;

	return pp2_init(&init_params);
}

static int musdk_pkt_echo_find_free_hif(void)
{
	int i;

	for (i = 0; i < MUSDK_PKT_ECHO_PP2_TOTAL_NUM_HIFS; i++) {
		if (!((1 << i) & used_hifs)) {
			used_hifs |= (1 << i);
			break;
		}
	}
	if (i == MUSDK_PKT_ECHO_PP2_TOTAL_NUM_HIFS) {
		pr_err("no free HIF found!\n");
		return -ENOSPC;
	}

	return i;
}

int musdk_pkt_echo_hif_init(struct pp2_hif **hif, u32 queue_size)
{
	int hif_id;
	char name[15];
	struct pp2_hif_params hif_params;
	int err;

	hif_id = musdk_pkt_echo_find_free_hif();
	if (hif_id < 0) {
		pr_err("free HIF not found!\n");
		return hif_id;
	}

	memset(name, 0, sizeof(name));
	snprintf(name, sizeof(name), "hif-%d", hif_id);
	pr_debug("found hif: %s\n", name);
	memset(&hif_params, 0, sizeof(hif_params));
	hif_params.match = name;
	hif_params.out_size = queue_size;
	err = pp2_hif_init(&hif_params, hif);
	if (err) {
		pr_err("%s: Error initialiazing HIF: %d\n", __func__, err);
		return err;
	}
	if (!hif) {
		pr_err("%s: HIF init failed!\n", __func__);
		return -EIO;
	}

	pr_debug("%s: HIF Initialized.\n", __func__);

	return 0;
}

static int find_free_bpool(u32 pp_id)
{
	int i;

	for (i = 0; i < PP2_BPOOL_NUM_POOLS; i++) {
		if (!((1 << i) & used_bpools[pp_id])) {
			used_bpools[pp_id] |= (1 << i);
			break;
		}
	}
	if (i == PP2_BPOOL_NUM_POOLS) {
		pr_err("no free BPool found!\n");
		return -ENOSPC;
	}
	return i;
}

int musdk_pkt_echo_build_all_bpools(struct bpool_desc ***ppools, int num_pools,
				   struct bpool_inf infs[], struct pp2_hif *hif)
{
	struct pp2_bpool_params		bpool_params;
	int				i, j, k, err, pool_id;
	char				name[15];
	struct bpool_desc		**pools = NULL;
	struct pp2_buff_inf		*buffs_inf = NULL;
	u8  pp2_num_inst = pp2_get_num_inst();

	buf_alloc_cnt = 0;

	if (num_pools > MVAPPS_PP2_MAX_NUM_BPOOLS) {
		pr_err("only %d pools allowed!\n", MVAPPS_PP2_MAX_NUM_BPOOLS);
		return -EINVAL;
	}

	/* TODO: release memory on error */
	pools = kmalloc_array(pp2_num_inst, sizeof(struct bpool_desc *), GFP_KERNEL);
	if (!pools)
		return -ENOMEM;

	*ppools = pools;

	for (i = 0; i < pp2_num_inst; i++) {
		pools[i] = kmalloc_array(num_pools, sizeof(struct bpool_desc), GFP_KERNEL);
		if (!pools[i]) {
			pr_err("no mem for bpool_desc array!\n");
			return -ENOMEM;
		}

		for (j = 0; j < num_pools; j++) {
			pool_id = find_free_bpool(i);
			if (pool_id < 0) {
				pr_err("free bpool not found!\n");
				return pool_id;
			}
			memset(name, 0, sizeof(name));
			snprintf(name, sizeof(name), "pool-%d:%d", i, pool_id);
			memset(&bpool_params, 0, sizeof(bpool_params));
			bpool_params.match = name;
			bpool_params.buff_len = infs[j].buff_size;

			pr_info("%s: buff_size %d, num_buffs %d\n", name, infs[j].buff_size, infs[j].num_buffs);
			err = pp2_bpool_init(&bpool_params, &pools[i][j].pool);
			if (err)
				return err;

			if (!pools[i][j].pool) {
				pr_err("BPool id%d init failed!\n", pool_id);
				return -EIO;
			}

			pools[i][j].buffs_inf =
				kmalloc_array(infs[j].num_buffs, sizeof(struct pp2_buff_inf), GFP_KERNEL);

			if (!pools[i][j].buffs_inf) {
				pr_err("no mem for bpools-inf array!\n");
				return -ENOMEM;
			}

			buffs_inf = pools[i][j].buffs_inf;
			pools[i][j].num_buffs = infs[j].num_buffs;

			for (k = 0; k < infs[j].num_buffs; k++) {
				void *buff_virt_addr;

				buff_virt_addr = mv_sys_dma_mem_alloc(infs[j].buff_size, 4);
				if (!buff_virt_addr) {
					pr_err("failed to allocate mem (%d)!\n", k);
					return -1;
				}
				buffs_inf[k].addr = mv_sys_dma_mem_virt2phys(buff_virt_addr);
				buffs_inf[k].cookie = (uintptr_t)buff_virt_addr;
			}

			for (k = 0; k < infs[j].num_buffs; k++) {
				struct pp2_buff_inf	tmp_buff_inf;

				tmp_buff_inf.cookie = buffs_inf[k].cookie;
				tmp_buff_inf.addr   = buffs_inf[k].addr;
				err = pp2_bpool_put_buff(hif, pools[i][j].pool, &tmp_buff_inf);
				if (err)
					return err;
				buf_alloc_cnt++;
			}
		}
	}
	return 0;
}

static int musdk_pkt_echo_init_musdk(void)
{
	int rc, port_index, i;
	struct bpool_inf infs[] = BPOOLS_INF;
	struct port_desc *port;

	rc = musdk_pkt_echo_pp2_init();
	if (rc) {
		pr_err("%s: musdk_pkt_echo_pp2_init error: %d\n", __func__, rc);
		return rc;
	}

	rc = musdk_pkt_echo_hif_init(&app_data.hif, MUSDK_PKT_ECHO_HIF_Q_SIZE);
	if (rc) {
		pr_err("%s: musdk_pkt_echo_hif_init error: %d\n", __func__, rc);
		return rc;
	}

	app_data.num_pools = ARRAY_SIZE(infs);
	rc = musdk_pkt_echo_build_all_bpools(&app_data.pools_desc, app_data.num_pools, infs, app_data.hif);

	app_data.num_ports = ARRAY_SIZE(interfaces);
	for (i = 0; i < ARRAY_SIZE(interfaces); i++)
		strcpy(app_data.ports_desc[i].name, interfaces[i]);

	for (port_index = 0; port_index < app_data.num_ports; port_index++) {
		port = &app_data.ports_desc[port_index];

		rc = musdk_pkt_echo_find_port_info(port);
		if (!rc) {
			port->num_tcs   = MVAPPS_MAX_NUM_TCS_PER_PORT;
			for (i = 0; i < port->num_tcs; i++)
				port->num_inqs[i] = MVAPPS_MAX_NUM_QS_PER_TC;
			port->inq_size  = MVAPPS_RX_Q_SIZE;
			port->num_outqs = MVAPPS_MAX_NUM_TCS_PER_PORT;
			port->outq_size = MVAPPS_TX_Q_SIZE;
			port->first_inq = MVAPPS_FIRST_MUSDK_IN_QUEUE;
			port->hash_type = app_data.hash_type;

			rc = musdk_pkt_echo_port_init(port, app_data.num_pools,
						     app_data.pools_desc[port->pp_id], DEFAULT_MTU);
			if (rc) {
				pr_err("Failed to initialize port %d (pp_id: %d)\n", port_index,
					port->pp_id);
				return rc;
			}
		} else {
			return rc;
		}
	}

	return 0;
}

static inline int loop_sw_recycle(u8			 rx_ppio_id,
				  u8			 tx_ppio_id,
				  u8			 tc,
				  u8			 qid,
				  u16			 num)
{
	struct tx_shadow_q	*shadow_q;
	int			 shadow_q_size;
	u16			 i, tx_num;
	int			 mycyc;
#ifdef APP_TX_RETRY
	u16			 desc_idx = 0, cnt = 0;
#endif

#ifdef PKT_ECHO_APP_PKT_ECHO_SUPPORT
	int			 prefetch_shift = PREFETCH_SHIFT;
#endif /* PKT_ECHO_APP_PKT_ECHO_SUPPORT */
#ifdef PKT_ECHO_APP_HW_TX_CHKSUM_CALC
	enum pp2_inq_l3_type     l3_type;
	enum pp2_inq_l4_type     l4_type;
	u8                       l3_offset, l4_offset;
#endif /* PKT_ECHO_APP_HW_TX_CHKSUM_CALC */

	shadow_q = &app_data.ports_desc[tx_ppio_id].shadow_qs[tc];
	shadow_q_size = app_data.ports_desc[tx_ppio_id].shadow_q_size;

/* pr_info("tid %d check on tc %d, qid %d\n", larg->id, tc, qid); */
/* pthread_mutex_lock(&larg->garg->trd_lock); */
	pp2_ppio_recv(app_data.ports_desc[rx_ppio_id].ppio, tc, qid, descs, &num);
/* pthread_mutex_unlock(&larg->garg->trd_lock); */
/* if (num) pr_info("got %d pkts on tc %d, qid %d\n", num, tc, qid); */
	INC_RX_COUNT(rx_ppio_id, num);

	for (i = 0; i < num; i++) {
		char		*buff = (char *)(uintptr_t)pp2_ppio_inq_desc_get_cookie(&descs[i]);
		dma_addr_t	 pa = pp2_ppio_inq_desc_get_phys_addr(&descs[i]);
		u16 len = pp2_ppio_inq_desc_get_pkt_len(&descs[i]);
		struct pp2_bpool *bpool = pp2_ppio_inq_desc_get_bpool(&descs[i], app_data.ports_desc[rx_ppio_id].ppio);

#ifdef PKT_ECHO_APP_PKT_ECHO_SUPPORT
		char *tmp_buff;
#ifdef PKT_ECHO_APP_USE_PREFETCH
		if (num - i > prefetch_shift) {
			tmp_buff = (char *)(uintptr_t)pp2_ppio_inq_desc_get_cookie(&descs[i + prefetch_shift]);
			tmp_buff += MVAPPS_PKT_EFEC_OFFS;
			prefetch(tmp_buff);
		}
#endif /* PKT_ECHO_APP_USE_PREFETCH */
		tmp_buff = buff;
		pr_debug("buff(%p)\n", tmp_buff);
		tmp_buff += MVAPPS_PKT_EFEC_OFFS;
		swap_l2(tmp_buff);
		swap_l3(tmp_buff);
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
		pp2_ppio_outq_desc_set_pkt_offset(&descs[i], MVAPPS_PKT_EFEC_OFFS);
		pp2_ppio_outq_desc_set_pkt_len(&descs[i], len);
		shadow_q->ents[shadow_q->write_ind].buff_ptr.cookie = (uintptr_t)buff;
		shadow_q->ents[shadow_q->write_ind].buff_ptr.addr = pa;
		shadow_q->ents[shadow_q->write_ind].bpool = bpool;
		pr_debug("buff_ptr.cookie(0x%llx)\n", (u64)shadow_q->ents[shadow_q->write_ind].buff_ptr.cookie);
		shadow_q->write_ind++;
		if (shadow_q->write_ind == shadow_q_size)
			shadow_q->write_ind = 0;
	}
	SET_MAX_BURST(rx_ppio_id, num);
	for (mycyc = 0; mycyc < app_data.busy_wait; mycyc++)
		asm volatile("");
#ifdef APP_TX_RETRY
	do {
		tx_num = num;
		if (num) {
			pp2_ppio_send(app_data.ports_desc[tx_ppio_id].ppio,
				      app_data.hif, tc, &descs[desc_idx], &tx_num);
			if (num > tx_num) {
				if (!cnt)
					INC_TX_RETRY_COUNT(rx_ppio_id, num - tx_num);
				cnt++;
			}
			desc_idx += tx_num;
			num -= tx_num;
			INC_TX_COUNT(rx_ppio_id, tx_num);
		}
		free_sent_buffers(&app_data.ports_desc[rx_ppio_id], &app_data.ports_desc[tx_ppio_id],
				  app_data.hif, tc, true);
	} while (num);
	SET_MAX_RESENT(rx_ppio_id, cnt);
#else
	if (num) {
		tx_num = num;
		pp2_ppio_send(app_data.ports_desc[tx_ppio_id].ppio, app_data.hif, tc, descs, &tx_num);
		if (num > tx_num) {
			u16 not_sent = num - tx_num;
			/* Free not sent buffers */
			shadow_q->write_ind = (shadow_q->write_ind < not_sent) ?
						(shadow_q_size - not_sent + shadow_q->write_ind) :
						shadow_q->write_ind - not_sent;
			free_buffers(&app_data.ports_desc[rx_ppio_id], &app_data.ports_desc[tx_ppio_id],
				     app_data.hif, shadow_q->write_ind, not_sent, tc);
			INC_TX_DROP_COUNT(rx_ppio_id, not_sent);
		}
		INC_TX_COUNT(rx_ppio_id, tx_num);
	}
	free_sent_buffers(&app_data.ports_desc[rx_ppio_id], &app_data.ports_desc[tx_ppio_id], app_data.hif,
			  tc, true);
#endif /* APP_TX_RETRY */

	return 0;
}

static int loop_1p(int *running)
{
	u16 num;
	u8 tc = 0, qid = 0;
	u64 qs_map = 0xFFFFFFFFFFFFFFFF;
	int err;

	num = DFLT_BURST_SIZE;

	while (*running) {
		/* Find next queue to consume */
		do {
			qid++;
			if (qid == MVAPPS_MAX_NUM_QS_PER_TC) {
				qid = 0;
				tc++;
				if (tc == MVAPPS_MAX_NUM_TCS_PER_PORT)
					tc = 0;
			}
		} while (!(/*larg->*/qs_map & (1 << ((tc * MVAPPS_MAX_NUM_QS_PER_TC) + qid))));

		err = loop_sw_recycle(0, 0, /*0, */tc, qid, num);
		if (err)
			return err;
	}

	return 0;
}

static int loop_2ps(int *running)
{
	u16 num;
	u8 tc = 0, qid = 0;
	u64 qs_map = 0xFFFFFFFFFFFFFFFF;
	int err;

	num = DFLT_BURST_SIZE;

	while (*running) {
		/* Find next queue to consume */
		do {
			qid++;
			if (qid == MVAPPS_MAX_NUM_QS_PER_TC) {
				qid = 0;
				tc++;
				if (tc == MVAPPS_MAX_NUM_TCS_PER_PORT)
					tc = 0;
			}
		} while (!(qs_map & (1 << ((tc * MVAPPS_MAX_NUM_QS_PER_TC) + qid))));

		err  = loop_sw_recycle(0, 1, /*0, */tc, qid, num);
		err |= loop_sw_recycle(1, 0, /*0, */tc, qid, num);

		if (err != 0)
			return err;
	}

	return 0;
}


int main_loop(int *running)
{
	while (*running) {
		if (app_data.num_ports == 1)
			loop_1p(running);
		else
			loop_2ps(running);
	}
	return 0;
}

static int musdk_pkt_echo_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int err;
	static int instances_probed;
	int running = 1;

	if (!dev->of_node) {
		dev_err(dev, "Device Tree does not contain a \"marvell,musdk-uio\" node.\n");
		return -EINVAL;
	}

	instances_probed++;

	pr_debug("%s: Probing instance no. %d/%d\n", __func__, instances_probed, pp2_get_num_inst());

	if (instances_probed == pp2_get_num_inst()) {

		err = musdk_pkt_echo_init_musdk();

		if (err)
			return err;

		main_loop(&running);
	}

	return 0;
}

static int musdk_pkt_echo_remove(struct platform_device *pdev)
{
	pp2_deinit();

	return 0;
}

static const struct of_device_id musdk_pkt_echo_of_match[] = {
	{ .compatible   = "marvell,mv-pp-uio", },
	{ }
};

static struct platform_driver musdk_pkt_echo_driver = {
	.driver = {
		.owner		= THIS_MODULE,
		.name		= DRIVER_NAME,
		.of_match_table = musdk_pkt_echo_of_match,
	},
	.probe  = musdk_pkt_echo_probe,
	.remove = musdk_pkt_echo_remove,
};

module_platform_driver(musdk_pkt_echo_driver);

MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_ALIAS("platform:" DRIVER_NAME);
