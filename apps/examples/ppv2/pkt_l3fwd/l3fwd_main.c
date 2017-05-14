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
#include <getopt.h>
#include <pthread.h>
#include <sched.h>
#include <stdbool.h>

#include "mv_std.h"
#include "lib/lib_misc.h"
#include "lib/net.h"
#include "env/mv_sys_dma.h"

#include "mvapp.h"
#include "mv_pp2.h"
#include "mv_pp2_hif.h"
#include "mv_pp2_bpool.h"
#include "mv_pp2_ppio.h"

#include "utils.h"
#include "byteorder_inlines.h"
#include "l3fwd_db.h"
#include "l3fwd_lpm.h"
#include "ezxml.h"

#define SHOW_STATISTICS
static const char buf_release_str[] = "SW buffer release";

#ifndef LPM_FRWD
static const char app_mode_str[] = "l3 forwarding - hash mode";
#else
static const char app_mode_str[] = "l3 forwarding - lpm mode";
#endif

static const char tx_retry_str[] = "Tx Retry enabled";

#define PKT_FWD_APP_DEF_Q_SIZE			1024
#define PKT_FWD_APP_HIF_Q_SIZE			(8 * PKT_FWD_APP_DEF_Q_SIZE)
#define PKT_FWD_APP_RX_Q_SIZE			(2 * PKT_FWD_APP_DEF_Q_SIZE)
#define PKT_FWD_APP_TX_Q_SIZE			(2 * PKT_FWD_APP_DEF_Q_SIZE)

#define PKT_FWD_APP_MAX_BURST_SIZE		((PKT_FWD_APP_RX_Q_SIZE) >> 1)
#define PKT_FWD_APP_DFLT_BURST_SIZE		256

#define PKT_FWD_APP_DMA_MEM_SIZE		(40 * 1024 * 1024)

#define PKT_FWD_APP_FIRST_INQ			0
#define PKT_FWD_APP_MAX_NUM_TCS_PER_PORT	1
#define PKT_FWD_APP_MAX_NUM_QS_PER_CORE	PKT_FWD_APP_MAX_NUM_TCS_PER_PORT

/* #define  PKT_FWD_APP_HW_TX_CHKSUM_CALC */
#ifdef PKT_FWD_APP_HW_TX_CHKSUM_CALC
#define PKT_FWD_APP_HW_TX_CHKSUM_CALC		1
#define PKT_FWD_APP_HW_TX_IPV4_CHKSUM_CALC	1
#endif
#define PKT_FWD_APP_USE_PREFETCH
#define PKT_FWD_APP_PREFETCH_SHIFT		7

#define PKT_FWD_APP_BPOOLS_INF		{ {384, 4096}, {2048, 1024} }
#define PKT_FWD_APP_BPOOLS_JUMBO_INF	{ {2048, 4096}, {10240, 512} }

#define MAX_NB_PKTIO	4
#define MAX_NB_ROUTE	32
#define INPUT_FILE_SIZE	256

typedef struct {
	char if_name[OIF_LEN];
	pp2h_ethaddr_t nexthop_mac_addr;
} if_mac_t;

typedef struct {
	char *if_names[MAX_NB_PKTIO];
	int if_count;
	char *route_str[MAX_NB_ROUTE];
	u8 hash_mode; /* 1:hash, 0:lpm */
	u8 dest_mac_changed[MAX_NB_PKTIO]; /* 1: dest mac from cmdline */
} app_args_t;

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
	int			 maintain_stats;
	u64			 qs_map;
	int			 qs_map_shift;
	int			 prefetch_shift;
	int			 num_ports;
	int			 pp2_num_inst;
	struct port_desc	 ports_desc[MVAPPS_MAX_NUM_PORTS];

	pthread_mutex_t		 trd_lock;

	struct pp2_hif		*hif;

	int			 num_pools;
	struct bpool_desc	**pools_desc;
	app_args_t args;
	pp2h_ethaddr_t		eth_src_mac[MAX_NB_PKTIO];
	pp2h_ethaddr_t		eth_dest_mac[MAX_NB_PKTIO];
	char			mac_dst_file[INPUT_FILE_SIZE];
	char			routing_file[INPUT_FILE_SIZE];
	/* forward func, hash or lpm */
	int (*fwd_func)(char *pkt, int l3_off, int l4_off);
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
	int			 id;
	int			 multi_buffer_release;

	struct glob_arg		*garg;
};

static struct glob_arg garg = {};

/**
 * Decrement TTL and incrementally update checksum
 *
 * @param ip  IPv4 header
 */
static inline void ipv4_dec_ttl_csum_update(pp2h_ipv4hdr_t *ip)
{
	u16 a = ~pp2_cpu_to_be_16(1 << 8);

	ip->ttl--;
	if (ip->chksum >= a)
		ip->chksum -= a;
	else
		ip->chksum += pp2_cpu_to_be_16(1 << 8);
}

static inline int l3fwd_pkt_lpm(char *pkt, int l3_off, int l4_off)
{
	pp2h_ipv4hdr_t *ip;
	pp2h_ethhdr_t *eth;
	int dif;
	int ret;

	ip = (pp2h_ipv4hdr_t *)(pkt + l3_off);
	ipv4_dec_ttl_csum_update(ip);
	eth = (pp2h_ethhdr_t *)pkt;

	/* network byte order maybe different from host */
	ret = fib_tbl_lookup(pp2_be_to_cpu_32(ip->dst_addr), &dif);
	if (ret)
		return -EINVAL;

	eth->dst = garg.eth_dest_mac[dif];
	eth->src = garg.eth_src_mac[dif];

	return dif;
}

static inline int l3fwd_pkt_hash(char *pkt, int l3_off, int l4_off)
{
	fwd_db_entry_t *entry;
	tuple5_t key;
	pp2h_ethhdr_t *eth;
	pp2h_ipv4hdr_t *ip;
	int dif;
#ifndef LPM_FRWD
	pp2h_udphdr_t  *udp;
#endif
#ifdef IPV6_ENABLED
	pp2h_ipv6hdr_t *ip6;
#endif
	memset(&key, 0, sizeof(tuple5_t));
	ip = (pp2h_ipv4hdr_t *)(pkt + l3_off);

#ifdef IPV6_ENABLED
	if (likely(IPV4_HDR_VER(ip->ver_ihl) == IP_VERSION_4)) {
#endif
		key.u5t.ipv4_5t.dst_ip = pp2_be_to_cpu_32(ip->dst_addr);
		if (unlikely((ip->ttl <= 1) || !((ip->proto == IP_PROTOCOL_UDP) ||
						 (ip->proto == IP_PROTOCOL_TCP)))) {
			/* drop zero TTL or not TCP/UDP traffic */
			return -EINVAL;
		}

#ifndef LPM_FRWD
		key.ip_protocol = IP_VERSION_4;
		key.u5t.ipv4_5t.src_ip = pp2_be_to_cpu_32(ip->src_addr);
		key.u5t.ipv4_5t.proto = ip->proto;

		udp = (pp2h_udphdr_t *)(pkt + l4_off);
		key.u5t.ipv4_5t.src_port = pp2_be_to_cpu_16(udp->src_port);
		key.u5t.ipv4_5t.dst_port = pp2_be_to_cpu_16(udp->dst_port);
#endif
		entry = find_fwd_db_entry(&key);
		if (likely(entry))
			ipv4_dec_ttl_csum_update(ip);

#ifdef IPV6_ENABLED
	} else {
		ip6 = (pp2h_ipv6hdr_t *)(pkt + l3_off);
		key.ip_protocol = IP_VERSION_6;
		if (unlikely((ip6->hop_limit <= 1) || !((ip6->next_hdr == IP_PROTOCOL_UDP) ||
							(ip6->next_hdr == IP_PROTOCOL_TCP)))) {
			/* drop zero TTL or not TCP/UDP traffic */
			return -EINVAL;
		}

		key.u5t.ipv6_5t.proto = ip6->next_hdr;
		memcpy(&key.u5t.ipv6_5t.dst_ipv6, ip6->dst_addr, IPV6_ADDR_LEN);
		memcpy(&key.u5t.ipv6_5t.src_ipv6, ip6->src_addr, IPV6_ADDR_LEN);
		udp = (pp2h_udphdr_t *)(pkt + l4_off);
		key.u5t.ipv6_5t.src_port = pp2_be_to_cpu_16(udp->src_port);
		key.u5t.ipv6_5t.dst_port = pp2_be_to_cpu_16(udp->dst_port);

		entry = find_fwd_db_entry(&key);
		if (likely(entry))
			ip6->hop_limit--;
	}
#endif

	if (likely(entry)) {
		eth = (pp2h_ethhdr_t *)pkt;
		eth->src = entry->src_mac;
		eth->dst = entry->dst_mac;
		dif = entry->oif_id;
	} else {
		/* no route, drop */
		pr_debug("packet is dropped\n");
		return -EINVAL;
	}
	pr_debug("dif = %d\n", dif);

	return dif;
}

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

u32 rx_buf_cnt[MVAPPS_MAX_NUM_CORES][MVAPPS_MAX_NUM_PORTS];
u32 free_buf_cnt[MVAPPS_MAX_NUM_CORES][MVAPPS_MAX_NUM_PORTS];
u32 tx_buf_cnt[MVAPPS_MAX_NUM_CORES][MVAPPS_MAX_NUM_PORTS];
u32 tx_buf_drop[MVAPPS_MAX_NUM_CORES][MVAPPS_MAX_NUM_PORTS];
u32 tx_buf_retry[MVAPPS_MAX_NUM_CORES][MVAPPS_MAX_NUM_PORTS];
u32 tx_max_resend[MVAPPS_MAX_NUM_CORES][MVAPPS_MAX_NUM_PORTS];
u32 tx_max_burst[MVAPPS_MAX_NUM_CORES][MVAPPS_MAX_NUM_PORTS];

#else
#define INC_RX_COUNT(core, port, cnt)
#define INC_TX_COUNT(core, port, cnt)
#define INC_TX_RETRY_COUNT(core, port, cnt)
#define INC_TX_DROP_COUNT(core, port, cnt)
#define INC_FREE_COUNT(core, port, cnt)
#define SET_MAX_RESENT(core, port, cnt)
#define SET_MAX_BURST(core, port, cnt)
#endif /* SHOW_STATISTICS */

#ifdef PKT_FWD_APP_USE_PREFETCH
static inline void prefetch(const void *ptr)
{
	asm volatile("prfm pldl1keep, %a0\n" : : "p" (ptr));
}
#endif /* PKT_FWD_APP_USE_PREFETCH */

#ifdef PKT_FWD_APP_HW_TX_CHKSUM_CALC
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
#endif

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

static struct buff_release_entry drop_buff[PKT_FWD_APP_DFLT_BURST_SIZE];

static inline int loop_sw_recycle(struct local_arg	*larg,
				  u8			 rx_ppio_id,
				  u8			 tc,
				  u8			 qid,
				  u16			 num)
{
	struct pp2_ppio_desc	descs[PKT_FWD_APP_MAX_BURST_SIZE];
	struct tx_shadow_q	*shadow_q;
	struct pp2_ppio_desc	*desc_ptr;
	struct pp2_ppio_desc	*desc_ptr_cur;
	struct pp2_ppio		*tx_ppio;
	struct pp2_hif		*hif = larg->hif;
	struct pp2_bpool	*bpool;
	int			shadow_q_size;
	int			dif, dst_port, mycyc;
	int			prefetch_shift = larg->prefetch_shift;
	u16			i, tx_num, tx_count, len;
	u16			num_drops = 0, desc_idx = 0, cnt = 0;
	char			*tmp_buff, *buff;
	dma_addr_t		pa;
	enum pp2_inq_l3_type	l3_type;
#ifndef LPM_FRWD
	enum pp2_inq_l4_type	l4_type;
#endif
	u8			l3_offset, l4_offset = 0;

	pp2_ppio_recv(larg->ports_desc[rx_ppio_id].ppio, tc, qid, descs, &num);
	INC_RX_COUNT(larg->id, rx_ppio_id, num);

	if (num < 1)
		return 0;

	desc_ptr = &descs[0];
	buff = (char *)(uintptr_t)pp2_ppio_inq_desc_get_cookie(desc_ptr);
	pa = pp2_ppio_inq_desc_get_phys_addr(desc_ptr);
	len = pp2_ppio_inq_desc_get_pkt_len(desc_ptr);
	bpool = pp2_ppio_inq_desc_get_bpool(desc_ptr, larg->ports_desc[rx_ppio_id].ppio);
#ifdef PKT_FWD_APP_USE_PREFETCH
	if (num > prefetch_shift) {
		tmp_buff = (char *)(uintptr_t)pp2_ppio_inq_desc_get_cookie(desc_ptr + prefetch_shift);
		tmp_buff += MVAPPS_PKT_EFEC_OFFS;
		tmp_buff = (char *)(((uintptr_t)tmp_buff) | sys_dma_high_addr);
		prefetch(tmp_buff);
	}
#endif /* PKT_FWD_APP_USE_PREFETCH */
	tmp_buff = (char *)(((uintptr_t)(buff)) | sys_dma_high_addr);
	tmp_buff += MVAPPS_PKT_EFEC_OFFS;

	pp2_ppio_inq_desc_get_l3_info(desc_ptr, &l3_type, &l3_offset);
#ifndef LPM_FRWD
	pp2_ppio_inq_desc_get_l4_info(desc_ptr, &l4_type, &l4_offset);
#endif

	dif = garg.fwd_func(tmp_buff, l3_offset, l4_offset);
	if (dif < 0) {
		drop_buff[num_drops].buff.cookie = (uintptr_t)buff;
		drop_buff[num_drops].buff.addr = pa;
		drop_buff[num_drops].bpool = bpool;
		num_drops++;
		tx_count = 0;
		pr_debug("drop\n");
	} else {
		pp2_ppio_outq_desc_reset(desc_ptr);
		pp2_ppio_outq_desc_set_phys_addr(desc_ptr, pa);
		pp2_ppio_outq_desc_set_pkt_offset(desc_ptr, MVAPPS_PKT_EFEC_OFFS);
		pp2_ppio_outq_desc_set_pkt_len(desc_ptr, len);

		shadow_q = &larg->ports_desc[dif].shadow_qs[tc];
		shadow_q_size = larg->ports_desc[dif].shadow_q_size;

		shadow_q->ents[shadow_q->write_ind].buff_ptr.cookie = (uintptr_t)buff;
		shadow_q->ents[shadow_q->write_ind].buff_ptr.addr = pa;
		shadow_q->ents[shadow_q->write_ind].bpool = bpool;
		shadow_q->write_ind++;
		if (shadow_q->write_ind == shadow_q_size)
			shadow_q->write_ind = 0;
		tx_count = 1;
	}

	do {
		/*
		 * advance until the packets are forwarded
		 * to the same if and send them at once
		 */
		dst_port = dif;
		for (i = 1; i < num; i++) {
			desc_ptr_cur = desc_ptr + i;
			buff = (char *)(uintptr_t)pp2_ppio_inq_desc_get_cookie(desc_ptr_cur);
			pa = pp2_ppio_inq_desc_get_phys_addr(desc_ptr_cur);
			len = pp2_ppio_inq_desc_get_pkt_len(desc_ptr_cur);
			bpool = pp2_ppio_inq_desc_get_bpool(desc_ptr_cur, larg->ports_desc[rx_ppio_id].ppio);

#ifdef PKT_FWD_APP_USE_PREFETCH
			if (num - i > prefetch_shift) {
				tmp_buff =
					(char *)(uintptr_t)pp2_ppio_inq_desc_get_cookie(desc_ptr_cur + prefetch_shift);
				tmp_buff += MVAPPS_PKT_EFEC_OFFS;
				tmp_buff = (char *)(((uintptr_t)tmp_buff) | sys_dma_high_addr);
				prefetch(tmp_buff);
			}
#endif /* PKT_FWD_APP_USE_PREFETCH */
			tmp_buff = (char *)(((uintptr_t)(buff)) | sys_dma_high_addr);
			tmp_buff += MVAPPS_PKT_EFEC_OFFS;

			pp2_ppio_inq_desc_get_l3_info(desc_ptr_cur, &l3_type, &l3_offset);
#ifndef LPM_FRWD
			pp2_ppio_inq_desc_get_l4_info(desc_ptr_cur, &l4_type, &l4_offset);
#endif

			dif = garg.fwd_func(tmp_buff, l3_offset, l4_offset);
			if (dif < 0) {
				drop_buff[num_drops].buff.cookie = (uintptr_t)buff;
				drop_buff[num_drops].buff.addr = pa;
				drop_buff[num_drops].bpool = bpool;
				num_drops++;
				continue;
			}
			pp2_ppio_outq_desc_reset(desc_ptr_cur);
			pp2_ppio_outq_desc_set_phys_addr(desc_ptr_cur, pa);
			pp2_ppio_outq_desc_set_pkt_offset(desc_ptr_cur, MVAPPS_PKT_EFEC_OFFS);
			pp2_ppio_outq_desc_set_pkt_len(desc_ptr_cur, len);
			shadow_q = &larg->ports_desc[dif].shadow_qs[tc];
			shadow_q_size = larg->ports_desc[dif].shadow_q_size;
			shadow_q->ents[shadow_q->write_ind].buff_ptr.cookie = (uintptr_t)buff;
			shadow_q->ents[shadow_q->write_ind].buff_ptr.addr = pa;
			shadow_q->ents[shadow_q->write_ind].bpool = bpool;
			pr_debug("buff_ptr.cookie(0x%lx)\n", (u64)shadow_q->ents[shadow_q->write_ind].buff_ptr.cookie);
			shadow_q->write_ind = (shadow_q->write_ind + 1 == shadow_q_size) ? 0 : shadow_q->write_ind + 1;

			if (dif != dst_port)
				break;
			tx_count++;
		}

		SET_MAX_BURST(larg->id, rx_ppio_id, tx_count);
		for (mycyc = 0; mycyc < larg->busy_wait; mycyc++)
			asm volatile("");
		while (tx_count) {
			desc_idx = 0;
			tx_ppio = larg->ports_desc[dst_port].ppio;
			tx_num = tx_count;
			if (tx_count) {
				pp2_ppio_send(tx_ppio, hif, tc, desc_ptr + desc_idx, &tx_num);
				if (tx_count > tx_num) {
					if (!cnt)
						INC_TX_RETRY_COUNT(larg->id, rx_ppio_id, tx_count - tx_num);
					cnt++;
				}
				desc_idx += tx_num;
				tx_count -= tx_num;
				INC_TX_COUNT(larg->id, rx_ppio_id, tx_num);
			}
			free_sent_buffers(&larg->ports_desc[rx_ppio_id], &larg->ports_desc[dst_port],
					  hif, tc, larg->multi_buffer_release);
		}
		desc_ptr += i;
		num -= i;
		if (dif >= 0)
			tx_count = 1;

	} while (num > 0);

	/* free all rx buffer drops */
	if (num_drops)
		pp2_bpool_put_buffs(hif, &drop_buff[0], &num_drops);

	return 0;
}

static int l3fw(struct local_arg *larg, int *running)
{
	int			err = 0;
	u16			num;
	u8			tc = 0, qid = 0;
	int			num_ports = larg->num_ports;
	u64			qs_map = larg->qs_map;
	int i;

	if (!larg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	num = larg->burst;
	while (*running) {
		/* Find next queue to consume */
		do {
			qid++;
			if (qid == MVAPPS_MAX_NUM_QS_PER_TC) {
				qid = 0;
				tc++;
				if (tc == PKT_FWD_APP_MAX_NUM_TCS_PER_PORT)
					tc = 0;
			}
		} while (!(qs_map & (1 << ((tc * MVAPPS_MAX_NUM_QS_PER_TC) + qid))));

		for (i = 0; i < num_ports; i++)
			err |= loop_sw_recycle(larg, i, tc, qid, num);

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

	return l3fw(larg, running);
}

static int init_all_modules(void)
{
	struct pp2_init_params	 pp2_params;
	int			 err;
	char			 file[PP2_MAX_BUF_STR_LEN];
	u32			 num_rss_tables;

	pr_info("Global initializations ...\n");

	err = mv_sys_dma_mem_init(PKT_FWD_APP_DMA_MEM_SIZE);
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

static int find_port_id_by_name(char *name, app_args_t *args)
{
	int i;

	if (!name)
		return -1;

	for (i = 0; i < args->if_count; i++) {
		if (!strcmp(name, args->if_names[i]))
			return i;
	}

	return -1;
}

static void init_mac_table(if_mac_t *if_mac_table, app_args_t *args)
{
	ezxml_t nhop;
	int i, j;
	ezxml_t mac_xml = ezxml_parse_file((const char *)&garg.mac_dst_file);

	if (!mac_xml)
		return;

	pr_info("Parsing next hop table xml\n");
	for (i = 0, nhop = ezxml_child(mac_xml, "Nexthop"); nhop && (i < MAX_NB_PKTIO); nhop = nhop->next,
	     i++) {
		if (ezxml_child(nhop, "If") && ezxml_child(nhop, "Mac")) {
			strcpy(if_mac_table[i].if_name, ezxml_child(nhop, "If")->txt);
			if (pp2h_eth_addr_parse(&if_mac_table[i].nexthop_mac_addr,
						ezxml_child(nhop, "Mac")->txt) != 0) {
				pr_err("init_mac_table: invalid nexthop MAC address\n");
				exit(EXIT_FAILURE);
			}
		}
	}
	ezxml_free(mac_xml);

	for (i = 0; i < MAX_NB_PKTIO; i++) {
		for (j = 0; j < MAX_NB_PKTIO; j++) {
			if (args->if_names[i]) {
				if (strcmp(args->if_names[i], if_mac_table[j].if_name) == 0) {
					mv_cp_eaddr(garg.eth_dest_mac[i].addr, if_mac_table[j].nexthop_mac_addr.addr);
					break;
				}
			}
		}
	}
}

#ifdef LPM_FRWD
static void parse_routing_xml(app_args_t *args)
{
	ezxml_t rentry;
	ezxml_t lpm;
	int outif;
	fwd_db_entry_t *entry;
	int i = 0;
	ezxml_t route_xml = ezxml_parse_file((const char *)&garg.routing_file);

	if (!route_xml)
		return;

	pr_info("Parsing routing xml\n");

	for (rentry = ezxml_child(route_xml, "RouteEntry"); rentry; rentry = rentry->next) {
		if (ezxml_child(rentry, "Lpm") && ezxml_child(rentry, "OutIf")) {
			lpm = ezxml_child(rentry, "Lpm");
			if (!(ezxml_child(lpm, "Subnet"))) {
				pr_err("%s: invalid LPM\n", __func__);
				exit(EXIT_FAILURE);
			}

			if (fwd_db->index >= MAX_DB) {
				pr_err("%s: out of space\n", __func__);
				return;
			}

			entry = &fwd_db->array[fwd_db->index];
			if (!entry) {
				pr_err("%s: not initialized entry\n", __func__);
				exit(EXIT_FAILURE);
			}
			if (parse_ipv4_string(ezxml_child(lpm, "Subnet")->txt,
					      &entry->subnet.addr, &entry->subnet.depth) == -1) {
				printf("create_fwd_db_entry: invalid IP address\n");
				return;
			}
			strncpy(entry->oif, ezxml_child(rentry, "OutIf")->txt, OIF_LEN - 1);
			entry->oif[OIF_LEN - 1] = 0;
			outif = find_port_id_by_name(entry->oif, args);
			if (outif == -1) {
				pr_warn("Skipping xml entry: port %s is not used.\n", entry->oif);
				memset(entry, 0, sizeof(fwd_db_entry_t));
				continue;
			}

			/* Add route to the list */
			fwd_db->index++;
			entry->next = fwd_db->list;
			fwd_db->list = entry;
			++i;
		}
	}
	ezxml_free(route_xml);
	pr_info("Routing xml: found %d entries\n", i);
}

#else
static void parse_routing_xml(app_args_t *args)
{
	ezxml_t rentry;
	ezxml_t tuple;
	int outif;
	fwd_db_entry_t *entry;
	int i = 0;
	ezxml_t route_xml = ezxml_parse_file((const char *)&garg.routing_file);

	if (!route_xml)
		return;

	pr_info("Parsing routing xml\n");

	for (rentry = ezxml_child(route_xml, "RouteEntry"); rentry; rentry = rentry->next) {
		if (ezxml_child(rentry, "Tuple") && ezxml_child(rentry, "OutIf")) {
			tuple = ezxml_child(rentry, "Tuple");
			if (!(ezxml_child(tuple, "SrcIP") &&
			      ezxml_child(tuple, "DstIP") &&
			      ezxml_child(tuple, "SrcPort") &&
			      ezxml_child(tuple, "DstPort") &&
			      ezxml_child(tuple, "Protocol"))) {
				pr_err("%s: invalid 5-Tuple\n", __func__);
				exit(EXIT_FAILURE);
			}

			if (fwd_db->index >= MAX_DB) {
				pr_err("%s: out of space\n", __func__);
				return;
			}

			entry = &fwd_db->array[fwd_db->index];
			if (!entry) {
				pr_err("%s: not initialized entry\n", __func__);
				exit(EXIT_FAILURE);
			}
			if (parse_ipv4_string(ezxml_child(tuple, "SrcIP")->txt, &entry->u.ipv4.src_subnet.addr,
					      &entry->u.ipv4.src_subnet.depth) == -1) {
				if (parse_ipv6_string(ezxml_child(tuple, "SrcIP")->txt,
						      &entry->u.ipv6.src_subnet.addr.u64.ipv6_hi,
						      &entry->u.ipv6.src_subnet.addr.u64.ipv6_lo,
						      &entry->u.ipv6.src_subnet.prefix) == -1) {
					pr_warn("Skipping xml entry: invalid SrcIp\n");
					memset(entry, 0, sizeof(fwd_db_entry_t));
					continue;
				} else {
					entry->ip_protocol = IP_VERSION_6;
				}
			} else {
				entry->ip_protocol = IP_VERSION_4;
			}
			if (entry->ip_protocol == IP_VERSION_4) {
				if (parse_ipv4_string(ezxml_child(tuple, "DstIP")->txt, &entry->u.ipv4.dst_subnet.addr,
						      &entry->u.ipv4.dst_subnet.depth) == -1) {
					pr_warn("Skipping xml entry: invalid DstIp\n");
					memset(entry, 0, sizeof(fwd_db_entry_t));
					continue;
				}
			} else {
				if (parse_ipv6_string(ezxml_child(tuple, "DstIP")->txt,
						      &entry->u.ipv6.dst_subnet.addr.u64.ipv6_hi,
						      &entry->u.ipv6.dst_subnet.addr.u64.ipv6_lo,
						      &entry->u.ipv6.dst_subnet.prefix) == -1) {
					pr_warn("Skipping xml entry: invalid DstIp\n");
					memset(entry, 0, sizeof(fwd_db_entry_t));
					continue;
				}
			}
			if (entry->ip_protocol == IP_VERSION_4) {
				entry->u.ipv4.src_port = atoi(ezxml_child(tuple, "SrcPort")->txt);
				entry->u.ipv4.dst_port = atoi(ezxml_child(tuple, "DstPort")->txt);
			} else {
				entry->u.ipv6.src_port = atoi(ezxml_child(tuple, "SrcPort")->txt);
				entry->u.ipv6.dst_port = atoi(ezxml_child(tuple, "DstPort")->txt);
			}
			if (strcmp(ezxml_child(tuple, "Protocol")->txt, "UDP") == 0) {
				if (entry->ip_protocol == IP_VERSION_4)
					entry->u.ipv4.protocol = IP_PROTOCOL_UDP;
				else
					entry->u.ipv6.protocol = IP_PROTOCOL_UDP;
			} else {
				if (strcmp(ezxml_child(tuple, "Protocol")->txt, "TCP") == 0) {
					if (entry->ip_protocol == IP_VERSION_4)
						entry->u.ipv4.protocol = IP_PROTOCOL_TCP;
					else
						entry->u.ipv6.protocol = IP_PROTOCOL_TCP;
				} else {
					pr_warn("Skipping xml entry: Invalid protocol %s\n",
						ezxml_child(tuple, "Protocol")->txt);
					memset(entry, 0, sizeof(fwd_db_entry_t));
					continue;
				}
			}

			strncpy(entry->oif, ezxml_child(rentry, "OutIf")->txt, OIF_LEN - 1);
			entry->oif[OIF_LEN - 1] = 0;
			outif = find_port_id_by_name(entry->oif, args);
			if (outif == -1) {
				pr_warn("Skipping xml entry: port %s is not used.\n", entry->oif);
				memset(entry, 0, sizeof(fwd_db_entry_t));
				continue;
			}

#ifndef IPV6_ENABLED
			/* skip IPv6 entries when IPv6 is disabled */
			if (entry->ip_protocol == IP_VERSION_4) {
#endif
			/* Add route to the list */
			fwd_db->index++;
			entry->next = fwd_db->list;
			fwd_db->list = entry;
			++i;
#ifndef IPV6_ENABLED
			}
#endif
		}
	}
	ezxml_free(route_xml);
	pr_info("Routing xml: found %d entries\n", i);
}
#endif

static void setup_fwd_db(void)
{
	fwd_db_entry_t *entry;
	int if_idx;
	app_args_t *args;

	args = &garg.args;
	if (args->hash_mode)
		init_fwd_hash_cache();
	else
		fib_tbl_init();

	for (entry = fwd_db->list; entry; entry = entry->next) {
		if_idx = entry->oif_id;
#ifdef LPM_FRWD
		/*LPM mode*/
		if (!args->hash_mode) {
			fib_tbl_insert(entry->subnet.addr, if_idx,
				       entry->subnet.depth);
		}
#endif
		if (args->dest_mac_changed[if_idx])
			garg.eth_dest_mac[if_idx] = entry->dst_mac;
		else
			entry->dst_mac = garg.eth_dest_mac[if_idx];
	}
}

static int init_fp(struct glob_arg *garg)
{
	u8 *dst_mac;
	u8 mac[ETH_ALEN];
	int i, j, rc;
	app_args_t *args = &garg->args;
	if_mac_t if_mac_table[MAX_NB_PKTIO];
	char *oif;

	/* Decide ip lookup method */
	if (args->hash_mode)
		garg->fwd_func = l3fwd_pkt_hash;
	else
		garg->fwd_func = l3fwd_pkt_lpm;

	args->if_count = garg->num_ports;
	for (i = 0; i < args->if_count; i++)
		args->if_names[i] = (char *)&garg->ports_desc[i].name;

	/* initialize the dest mac as 2:0:0:0:0:x */
	memset(mac, 0, ETH_ALEN);
	mac[0] = 2;
	for (i = 0; i < MAX_NB_PKTIO; i++) {
		mac[ETH_ALEN - 1] = (u8)i;
		mv_cp_eaddr(garg->eth_dest_mac[i].addr, mac);
	}

	/* read MACs table file and configure interface MAC addresses */
	memset(if_mac_table, 0, sizeof(if_mac_t) * MAX_NB_PKTIO);

	init_mac_table(if_mac_table, args);
	init_fwd_db();

	/* Add route into table */
	for (i = 0; i < MAX_NB_ROUTE; i++) {
		if (args->route_str[i]) {
			if (create_fwd_db_entry(args->route_str[i], &oif, &dst_mac) == -1) {
				pr_err("Error: fail to create route entry.\n");
				exit(1);
			}

			if (!oif) {
				pr_err("Error: fail to create route entry.\n");
				exit(1);
			}

			j = find_port_id_by_name(oif, args);
			if (j == -1) {
				pr_err("Error: port %s not used.\n", oif);
				exit(1);
			}

			if (dst_mac)
				args->dest_mac_changed[j] = 1;
		}
	}

	/* routing xml entries are added to the fw db after command line configuration */
	parse_routing_xml(args);

	/* Resolve fwd db*/
	for (i = 0; i < args->if_count; i++) {
		char *if_name;

		if_name = args->if_names[i];
		rc = pp2_ppio_get_mac_addr(garg->ports_desc[i].ppio, mac);
		if (rc) {
			pr_err("Unable to get mac address\n");
			return -EINVAL;
		}
		mv_cp_eaddr(garg->eth_src_mac[i].addr, mac);
		resolve_fwd_db(if_name, i, mac);
	}

	setup_fwd_db();

	dump_fwd_db();

	return 0;
}

static int init_local_modules(struct glob_arg *garg)
{
	int				err, port_index;
	struct bpool_inf		std_infs[] = PKT_FWD_APP_BPOOLS_INF;
	struct bpool_inf		jumbo_infs[] = PKT_FWD_APP_BPOOLS_JUMBO_INF;
	struct bpool_inf		*infs;
	int				i;

	pr_info("Local initializations ...\n");

	err = app_hif_init(&garg->hif, PKT_FWD_APP_HIF_Q_SIZE);
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
			port->num_tcs	= PKT_FWD_APP_MAX_NUM_TCS_PER_PORT;
			for (i = 0; i < port->num_tcs; i++)
				port->num_inqs[i] =  garg->cpus;
			port->inq_size	= garg->rxq_size;
			port->num_outqs	= PKT_FWD_APP_MAX_NUM_TCS_PER_PORT;
			port->outq_size	= PKT_FWD_APP_TX_Q_SIZE;
			port->first_inq	= PKT_FWD_APP_FIRST_INQ;
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

	err = init_fp(garg);
	if (err)
		return err;

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
	err = app_hif_init(&larg->hif, PKT_FWD_APP_HIF_Q_SIZE);
	pthread_mutex_unlock(&garg->trd_lock);
	if (err)
		return err;

	larg->id                = id;
	larg->burst		= garg->burst;
	larg->busy_wait		= garg->busy_wait;
	larg->multi_buffer_release = garg->multi_buffer_release;
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
	       "MUSDK l3fwd application.\n"
	       "\n"
	       "Usage: %s OPTIONS\n"
#ifdef LPM_FRWD
	       "  E.g. %s -i eth0,eth1 -r 1.1.1.0/24,eth0 -r 2.2.2.0/24,eth1\n"
	       " In the above example,\n"
	       " eth0 will send pkts to eth1 and vice versa\n"
	       "\n"
	       "Mandatory OPTIONS:\n"
	       "  -i, --interface eth interfaces (comma-separated, no spaces)\n"
	       "  -r, --route SubNet,Intf[,NextHopMAC]\n"
	       "	NextHopMAC can be optional\n"
	       "\n"
#else
	       "  E.g. %s -i eth0,eth1 -r 1.1.1.0/24,2.2.2.0/24,2002,2003,0,eth1\n"
	       " In the above example,\n"
	       " eth0 will send pkts to eth1\n"
	       "\n"
	       "Mandatory OPTIONS:\n"
	       "  -i, --interface eth interfaces (comma-separated, no spaces)\n"
	       "  -r, --route Src_SubNet,Dst_SubNet,Src_Port,Dst_Port,0-UDP/1-TCP,Dst_Intf[,NextHopMAC]\n"
	       "	NextHopMAC can be optional\n"
	       "\n"
#endif
	       "Optional OPTIONS:\n"
	       "\t-b <size>                Burst size, num_pkts handled in a batch.(default is %d)\n"
	       "\t--mtu <mtu>              Set MTU (default is %d)\n"
	       "\t-c, --cores <number>     Number of CPUs to use\n"
	       "\t-a, --affinity <number>  Use setaffinity (default is no affinity)\n"
	       "\t-s                       Maintain statistics\n"
	       "\t-w <cycles>              Cycles to busy_wait between recv&send, simulating app behavior (default=0)\n"
	       "\t--rxq <size>             Size of rx_queue (default is %d)\n"
	       "\t--cli                    Use CLI\n"
	       "\t--routing-file           Use *.xml file\n"
	       "\t--mac-dst-file           Use *.xml file\n"
	       "\t?, -h, --help            Display help and exit.\n\n"
	       "\n", MVAPPS_NO_PATH(progname), MVAPPS_NO_PATH(progname),
	       PKT_FWD_APP_MAX_BURST_SIZE, DEFAULT_MTU, PKT_FWD_APP_RX_Q_SIZE);
}

static int parse_args(struct glob_arg *garg, int argc, char *argv[])
{
	char *local;
	char *token;
	int rc;
	int route_index = 0;
	int option;
	int long_index = 0;
	struct option long_options[] = {
		{"help", no_argument, 0, 'h'},
		{"interface", required_argument, 0, 'i'},
		{"route", required_argument, 0, 'r'},
		{"routing-file", required_argument, 0, 'f'},
		{"mac-dst-file", required_argument, 0, 'd'},
		{"burst", required_argument, 0, 'b'},
		{"mtu", required_argument, 0, 'u'},
		{"core", required_argument, 0, 'c'},
		{"affinity", required_argument, 0, 'a'},
		{"stats", no_argument, 0, 's'},
		{"wait", required_argument, 0, 'w'},
		{"rxq", required_argument, 0, 'q'},
		{"qs-map", no_argument, 0, 'm'},
		{"cli", no_argument, 0, 'l'},
		{0, 0, 0, 0}
	};

	garg->cli = 0;
	garg->cpus = 1;
	garg->affinity = -1;
	garg->burst = PKT_FWD_APP_DFLT_BURST_SIZE;
	garg->mtu = DEFAULT_MTU;
	garg->busy_wait	= 0;
	garg->rxq_size = PKT_FWD_APP_RX_Q_SIZE;
	garg->multi_buffer_release = 1;
	garg->qs_map = 0;
	garg->qs_map_shift = 0;
	garg->prefetch_shift = PKT_FWD_APP_PREFETCH_SHIFT;
	garg->maintain_stats = 0;

#ifdef LPM_FRWD
	garg->args.hash_mode = 0;
#else
	garg->args.hash_mode = 1;
#endif

	/* every time starting getopt we should reset optind */
	optind = 0;
	while ((option = getopt_long(argc, argv, "hi:r:f:d:b:u:c:a:swq:zmi", long_options, &long_index)) != -1) {
		switch (option) {
		case 'h':
			usage(argv[0]);
			exit(0);
			break;
		case 'i':
			/* count the number of tokens separated by ',' */
			for (token = strtok(optarg, ","), garg->num_ports = 0;
			     token;
			     token = strtok(NULL, ","), garg->num_ports++)
				snprintf(garg->ports_desc[garg->num_ports].name,
					 sizeof(garg->ports_desc[garg->num_ports].name),
					 "%s", token);

			if (garg->num_ports == 0) {
				pr_err("Invalid interface arguments format!\n");
				return -EINVAL;
			} else if (garg->num_ports > MVAPPS_MAX_NUM_PORTS) {
				pr_err("too many ports specified (%d vs %d)\n",
				       garg->num_ports, MVAPPS_MAX_NUM_PORTS);
				return -EINVAL;
			}
			break;
		case 'r':
			if (route_index >= MAX_NB_ROUTE) {
				pr_err("No more routes can be added\n");
				break;
			}

			local = calloc(1, strlen(optarg) + 1);
			if (!local) {
				pr_err("calloc fail!\n");
				return -EINVAL;
			}
			memcpy(local, optarg, strlen(optarg));
			local[strlen(optarg)] = '\0';
			garg->args.route_str[route_index++] = local;
			break;
		case 'f':
			strncpy(&garg->routing_file[0], optarg, sizeof(garg->routing_file));
			break;
		case 'd':
			strncpy(&garg->mac_dst_file[0], optarg, sizeof(garg->mac_dst_file));
			break;
		case 'b':
			garg->burst = atoi(optarg);
			break;
		case 'u':
			garg->mtu = atoi(optarg);
			break;
		case 'c':
			garg->cpus = atoi(optarg);
			break;
		case 'a':
			garg->affinity = atoi(optarg);
			break;
		case 's':
			garg->maintain_stats = 1;
			break;
		case 'w':
			garg->busy_wait = atoi(optarg);
			break;
		case 'q':
			garg->rxq_size = atoi(optarg);
			break;
		case 'm':
			rc = sscanf(optarg, "%x:%x", (unsigned int *)&garg->qs_map, &garg->qs_map_shift);
			if (rc != 2) {
				pr_err("Failed to parse -m parameter\n");
				return -EINVAL;
			}
			break;
		case 'l':
			garg->cli = 1;
			break;
		default:
			pr_err("parsing fail, wrong input\n");
			return -EINVAL;
		}
	}

	/* Now, check validity of all inputs */
	if (!garg->num_ports ||
	    !garg->ports_desc[0].name) {
		pr_err("No port defined!\n");
		return -EINVAL;
	}
	if (garg->burst > PKT_FWD_APP_MAX_BURST_SIZE) {
		pr_err("illegal burst size requested (%d vs %d)!\n",
		       garg->burst, PKT_FWD_APP_MAX_BURST_SIZE);
		return -EINVAL;
	}
	if (garg->cpus > MVAPPS_MAX_NUM_CORES) {
		pr_err("illegal num cores requested (%d vs %d)!\n",
		       garg->cpus, MVAPPS_MAX_NUM_CORES);
		return -EINVAL;
	}
	if ((garg->affinity != -1) &&
	    ((garg->cpus + garg->affinity) > MVAPPS_MAX_NUM_CORES)) {
		pr_err("illegal num cores or affinity requested (%d,%d vs %d)!\n",
		       garg->cpus, garg->affinity, MVAPPS_MAX_NUM_CORES);
		return -EINVAL;
	}

	if (garg->qs_map &&
	    (MVAPPS_MAX_NUM_QS_PER_TC == 1) &&
	    (PKT_FWD_APP_MAX_NUM_TCS_PER_PORT == 1)) {
		pr_warn("no point in queues-mapping; ignoring.\n");
		garg->qs_map = 1;
		garg->qs_map_shift = 1;
	} else if (!garg->qs_map) {
		garg->qs_map = 1;
		garg->qs_map_shift = PKT_FWD_APP_MAX_NUM_TCS_PER_PORT;
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
	pr_info("pkt-l3fwd is started:\n\t%s\n\t%s\n\t%s\n", app_mode_str, buf_release_str, tx_retry_str);

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
	for (i = 0; i < MVAPPS_MAX_NUM_CORES; i++) {
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
