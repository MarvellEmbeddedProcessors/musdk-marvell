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
#include <sys/sysinfo.h>
#include <limits.h>


#include "mv_std.h"
#include "lib/lib_misc.h"
#include "lib/net.h"
#include "env/mv_sys_dma.h"
#include "env/io.h"

#include "mvapp.h"
#include "mv_pp2.h"
#include "mv_pp2_hif.h"
#include "mv_pp2_bpool.h"
#include "mv_pp2_ppio.h"

#include "pp2_utils.h"
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
#define PKT_FWD_APP_CTRL_DFLT_THR		1000


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

#define PKT_FWD_APP_BPOOLS_INF		{ {384, 4096, 0, NULL}, {2048, 1024, 0, NULL} }
#define PKT_FWD_APP_BPOOLS_JUMBO_INF	{ {2048, 4096, 0, NULL}, {10240, 512, 0, NULL} }

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
	struct glb_common_args	cmn_args;  /* Keep first */

	u16				rxq_size;
	int				maintain_stats;

	pthread_mutex_t			trd_lock;

	app_args_t			args;

	pp2h_ethaddr_t			eth_src_mac[MAX_NB_PKTIO];
	pp2h_ethaddr_t			eth_dest_mac[MAX_NB_PKTIO];
	char				mac_dst_file[INPUT_FILE_SIZE];
	char				routing_file[INPUT_FILE_SIZE];

	/* forward func, hash or lpm */
	int (*fwd_func)(char *pkt, int l3_off, int l4_off);
};

struct local_arg {
	struct local_common_args	cmn_args;  /* Keep first */
	struct buff_release_entry	dropped_buffers[PKT_FWD_APP_DFLT_BURST_SIZE];
};

static struct glob_arg garg = {};

/**
 * Decrement TTL and incrementally update checksum
 *
 * @param ip  IPv4 header
 */
static inline void ipv4_dec_ttl_csum_update(pp2h_ipv4hdr_t *ip)
{
	u16 a = ~htobe16(1 << 8);

	ip->ttl--;
	if (ip->chksum >= a)
		ip->chksum -= a;
	else
		ip->chksum += htobe16(1 << 8);
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
	ret = fib_tbl_lookup(be32toh(ip->dst_addr), &dif);
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
		key.u5t.ipv4_5t.dst_ip = be32toh(ip->dst_addr);
		if (unlikely((ip->ttl <= 1) || !((ip->proto == IP_PROTOCOL_UDP) ||
						 (ip->proto == IP_PROTOCOL_TCP)))) {
			/* drop zero TTL or not TCP/UDP traffic */
			return -EINVAL;
		}

#ifndef LPM_FRWD
		key.ip_protocol = IP_VERSION_4;
		key.u5t.ipv4_5t.src_ip = be32toh(ip->src_addr);
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


static inline int loop_sw_recycle(struct local_arg	*larg,
				  u8			 rx_ppio_id,
				  u8			 tc,
				  u8			 qid,
				  u16			 num)
{
	struct pp2_ppio_desc	descs[PKT_FWD_APP_MAX_BURST_SIZE];
	struct pp2_lcl_common_args *pp2_args = (struct pp2_lcl_common_args *) larg->cmn_args.plat;
	struct perf_cmn_cntrs	*perf_cntrs = &larg->cmn_args.perf_cntrs;
	struct tx_shadow_q	*shadow_q;
	struct buff_release_entry *drop_buff = &larg->dropped_buffers[0];
	struct pp2_ppio_desc	*desc_ptr, *desc_ptr_cur, *desc_ptr_send_cur;
	struct pp2_ppio		*tx_ppio;
	struct pp2_hif		*hif = pp2_args->hif;
	struct pp2_bpool	*bpool;
	int			dif, dst_port, shadow_q_size, max_write;
	int			prefetch_shift = larg->cmn_args.prefetch_shift;
	u16			i, tx_num, tx_count, tx_count_dup, len, read_ind, write_ind, write_start_ind;
	u16			num_drops = 0, desc_idx = 0, cnt = 0;
	char			*tmp_buff, *buff;
	dma_addr_t		pa;
	enum pp2_inq_l3_type	l3_type;
#ifndef LPM_FRWD
	enum pp2_inq_l4_type	l4_type;
#endif
	u8			l3_offset, l4_offset = 0;

	pp2_ppio_recv(pp2_args->lcl_ports_desc[rx_ppio_id].ppio, tc, qid, descs, &num);
	INC_RX_COUNT(&pp2_args->lcl_ports_desc[rx_ppio_id], num);
	perf_cntrs->rx_cnt += num;

	if (num == 0)
		return 0;

	dif = -1;
	desc_ptr = &descs[0];
	tx_count = 0;

	do {
		/*
		 * advance until the packets are forwarded
		 * to the same if and send them at once
		 */
		dst_port = dif;
		desc_ptr_send_cur = desc_ptr;
		max_write = INT_MAX;
		for (i = 0; i < num; i++) {
			desc_ptr_cur = desc_ptr + i;

			buff = (char *)(app_get_high_addr() |
					(uintptr_t)pp2_ppio_inq_desc_get_cookie(desc_ptr_cur));
			pa = pp2_ppio_inq_desc_get_phys_addr(desc_ptr_cur);
			len = pp2_ppio_inq_desc_get_pkt_len(desc_ptr_cur);
			bpool = pp2_ppio_inq_desc_get_bpool(desc_ptr_cur,
							    pp2_args->lcl_ports_desc[rx_ppio_id].ppio);

#ifdef PKT_FWD_APP_USE_PREFETCH
			if (num - i > prefetch_shift) {
				tmp_buff = (char *)(app_get_high_addr() | (uintptr_t)
						    pp2_ppio_inq_desc_get_cookie(desc_ptr_cur + prefetch_shift));
				tmp_buff += MVAPPS_PP2_PKT_DEF_EFEC_OFFS;
				prefetch(tmp_buff);
			}
#endif /* PKT_FWD_APP_USE_PREFETCH */
			tmp_buff = buff;
			tmp_buff += MVAPPS_PP2_PKT_DEF_EFEC_OFFS;

			pp2_ppio_inq_desc_get_l3_info(desc_ptr_cur, &l3_type, &l3_offset);
#ifndef LPM_FRWD
			pp2_ppio_inq_desc_get_l4_info(desc_ptr_cur, &l4_type, &l4_offset);
#endif

			dif = garg.fwd_func(tmp_buff, l3_offset, l4_offset);
			if (dif != dst_port && likely(dif >= 0)) {
				if (unlikely(dst_port == -1)) /* destination has never been set yet */
					dst_port = dif;
				else
					break; /* descriptor is handled in next occurrence of this for-loop */
			}
			if (unlikely((dif < 0) || (max_write == 0))) {
				drop_buff[num_drops].buff.cookie = (uintptr_t)buff;
				drop_buff[num_drops].buff.addr = pa;
				drop_buff[num_drops].bpool = bpool;
				num_drops++;
				if (max_write == 0)
					pr_err("%s: port:%d, txq_id:%d, shadow_q size:%d too small, emerg. drops\n",
						__func__, dst_port, tc, shadow_q_size);
				continue;
			}

			pp2_ppio_outq_desc_reset(desc_ptr_send_cur);
			pp2_ppio_outq_desc_set_phys_addr(desc_ptr_send_cur, pa);
			pp2_ppio_outq_desc_set_pkt_offset(desc_ptr_send_cur, MVAPPS_PP2_PKT_DEF_EFEC_OFFS);
			pp2_ppio_outq_desc_set_pkt_len(desc_ptr_send_cur, len);

			if (tx_count == 0) {
				shadow_q = &pp2_args->lcl_ports_desc[dif].shadow_qs[tc];
				shadow_q_size = pp2_args->lcl_ports_desc[dif].shadow_q_size;

				/* read_index need not be locked, just sampled */
				read_ind = shadow_q->read_ind;

				/* write_index will be updated */
				if (shadow_q->shared_q)
					spin_lock(&shadow_q->write_lock);

				/* Work with local copy */
				write_ind = write_start_ind = shadow_q->write_ind;

				/* Calculate indexes, and possible overflow conditions */
				if (read_ind > write_ind)
					max_write = (read_ind - 1) - write_ind;
				else
					max_write = (shadow_q_size - write_ind) + (read_ind - 1);
			}
			shadow_q->ents[write_ind].buff_ptr.cookie = (uintptr_t)buff;
			shadow_q->ents[write_ind].buff_ptr.addr = pa;
			shadow_q->ents[write_ind].bpool = bpool;
			pr_debug("buff_ptr.cookie(0x%lx)\n", (u64)shadow_q->ents[write_ind].buff_ptr.cookie);

			if (++write_ind == shadow_q_size)
				write_ind = 0;
			max_write--;
			tx_count++;
			desc_ptr_send_cur++;
		}
		/* tx_count confirms write_ind was updated */
		if (tx_count) {
			shadow_q->write_ind = write_ind;
			if (shadow_q->shared_q)
				spin_unlock(&shadow_q->write_lock);
		}

		SET_MAX_BURST(&pp2_args->lcl_ports_desc[rx_ppio_id], tx_count);

		if (tx_count) {
			if (shadow_q->shared_q) {
				/* CPU's w/ shared_q can't send simultaneously, since buffers must be freed in order */
				while (shadow_q->send_ind != write_start_ind)
					cpu_relax(); /* mem-barrier not needed, cpu_relax() ensures send_ind reloads */

				spin_lock(&shadow_q->send_lock);
			}

			desc_idx = 0;
			tx_count_dup = tx_count;
			while (tx_count) {
				tx_ppio = pp2_args->lcl_ports_desc[dst_port].ppio;
				tx_num = tx_count;

				pp2_ppio_send(tx_ppio, hif, tc, &desc_ptr[desc_idx], &tx_num);
				if (tx_count > tx_num) {
					if (!cnt)
					INC_TX_RETRY_COUNT(&pp2_args->lcl_ports_desc[rx_ppio_id], num - tx_num);
					cnt++;
				}
				desc_idx += tx_num;
				tx_count -= tx_num;
				INC_TX_COUNT(&pp2_args->lcl_ports_desc[rx_ppio_id], tx_num);
				perf_cntrs->tx_cnt += tx_num;

				free_sent_buffers(&pp2_args->lcl_ports_desc[rx_ppio_id],
						  &pp2_args->lcl_ports_desc[dst_port], hif, tc, 1);
			}
			if (((shadow_q_size - 1) - shadow_q->send_ind) >= tx_count_dup)
				shadow_q->send_ind += tx_count_dup;
			else
				shadow_q->send_ind = tx_count_dup - (shadow_q_size - shadow_q->send_ind);

			if (shadow_q->shared_q)
				spin_unlock(&shadow_q->send_lock);
		}

		desc_ptr += i;
		num -= i;
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
	int			num_ports = larg->cmn_args.num_ports;
	u64			qs_map = larg->cmn_args.qs_map;
	int i;

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
				if (tc == PKT_FWD_APP_MAX_NUM_TCS_PER_PORT)
					tc = 0;
			}
		} while (!(qs_map & (1 << ((tc * mvapp_pp2_max_num_qs_per_tc) + qid))));

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
	struct glb_common_args *cmn_args = &garg.cmn_args;
	struct pp2_glb_common_args *pp2_args = (struct pp2_glb_common_args *)garg.cmn_args.plat;
	int			 err = 0;
	char			 file[PP2_MAX_BUF_STR_LEN];
	int			 num_rss_tables = 0;
	struct mv_sys_dma_mem_region_params params;

	pr_info("Global initializations ...\n");

	app_get_num_cpu_clusters(cmn_args);

	params.size = PKT_FWD_APP_DMA_MEM_SIZE;
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
	struct pp2_glb_common_args *pp2_args = (struct pp2_glb_common_args *) garg->cmn_args.plat;
	char *oif;

	/* Decide ip lookup method */
	if (args->hash_mode)
		garg->fwd_func = l3fwd_pkt_hash;
	else
		garg->fwd_func = l3fwd_pkt_lpm;

	args->if_count = garg->cmn_args.num_ports;
	for (i = 0; i < args->if_count; i++)
		args->if_names[i] = (char *)&pp2_args->ports_desc[i].name;

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
		rc = pp2_ppio_get_mac_addr(pp2_args->ports_desc[i].ppio, mac);
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
	struct bpool_inf                std_inf[] = PKT_FWD_APP_BPOOLS_INF, jumbo_inf[] = PKT_FWD_APP_BPOOLS_JUMBO_INF;
	struct bpool_inf		*infs;
	struct pp2_glb_common_args *pp2_args = (struct pp2_glb_common_args *) garg->cmn_args.plat;
	int				i = 0;

	pr_info("Local initializations ...\n");

	err = app_build_common_hifs(&garg->cmn_args, PKT_FWD_APP_HIF_Q_SIZE);
	if (err)
		return err;

	app_prepare_bpools(&garg->cmn_args, &infs, std_inf, ARRAY_SIZE(std_inf), jumbo_inf, ARRAY_SIZE(jumbo_inf));

	err = app_build_all_bpools(&pp2_args->pools_desc, pp2_args->num_pools, infs, pp2_args->hif);
	if (err)
		return err;

	for (port_index = 0; port_index < garg->cmn_args.num_ports; port_index++) {
		struct port_desc *port = &pp2_args->ports_desc[port_index];

		err = app_find_port_info(port);
		if (!err) {
			memcpy(port->mem_region, garg->cmn_args.mem_region, sizeof(garg->cmn_args.mem_region));
			port->ppio_type	= PP2_PPIO_T_NIC;
			port->num_tcs	= PKT_FWD_APP_MAX_NUM_TCS_PER_PORT;
			for (i = 0; i < port->num_tcs; i++)
				port->num_inqs[i] =  garg->cmn_args.cpus;
			port->inq_size	= garg->rxq_size;
			port->num_outqs	= PKT_FWD_APP_MAX_NUM_TCS_PER_PORT;
			port->outq_size	= PKT_FWD_APP_TX_Q_SIZE;
			port->first_inq	= PKT_FWD_APP_FIRST_INQ;
			if (garg->cmn_args.cpus == 1)
				port->hash_type = PP2_PPIO_HASH_T_NONE;
			else
				port->hash_type = PP2_PPIO_HASH_T_2_TUPLE;
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
			app_port_inqs_mask_by_affinity(&garg->cmn_args, port);

			/* pkt_offset=0 not to be changed, before recoding rx_data_path to use pkt_offset as well */
			err = app_port_init(port, pp2_args->num_pools, pp2_args->pools_desc[port->pp_id],
					    garg->cmn_args.mtu, 0);
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

	err = app_hif_init_wrap(id, &garg->cmn_args.thread_lock, glb_pp2_args, lcl_pp2_args, PKT_FWD_APP_HIF_Q_SIZE,
				garg->cmn_args.mem_region[mem_id]);
	if (err)
		return err;

	larg->cmn_args.burst		= garg->cmn_args.burst;
	larg->cmn_args.prefetch_shift	= garg->cmn_args.prefetch_shift;
	larg->cmn_args.num_ports         = garg->cmn_args.num_ports;
	lcl_pp2_args->lcl_ports_desc = (struct lcl_port_desc *)
					malloc(larg->cmn_args.num_ports * sizeof(struct lcl_port_desc));
	for (i = 0; i < larg->cmn_args.num_ports; i++)
		app_port_local_init(i, larg->cmn_args.id, &lcl_pp2_args->lcl_ports_desc[i],
				    &glb_pp2_args->ports_desc[i]);

	lcl_pp2_args->pools_desc	= glb_pp2_args->pools_desc;
	larg->cmn_args.garg              = garg;
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
	struct pp2_glb_common_args *pp2_args = (struct pp2_glb_common_args *) garg->cmn_args.plat;
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
		{"rxq", required_argument, 0, 'q'},
		{"qs-map", no_argument, 0, 'm'},
		{"cli", no_argument, 0, 'l'},
		{0, 0, 0, 0}
	};

	garg->cmn_args.cli = 0;
	garg->cmn_args.cpus = 1;
	garg->cmn_args.affinity = MVAPPS_INVALID_AFFINITY;
	garg->cmn_args.burst = PKT_FWD_APP_DFLT_BURST_SIZE;
	garg->cmn_args.mtu = DEFAULT_MTU;
	garg->rxq_size = PKT_FWD_APP_RX_Q_SIZE;
	garg->cmn_args.qs_map = 0;
	garg->cmn_args.qs_map_shift = 0;
	garg->cmn_args.pkt_offset = 0;
	garg->cmn_args.prefetch_shift = PKT_FWD_APP_PREFETCH_SHIFT;
	garg->cmn_args.ctrl_thresh = PKT_FWD_APP_CTRL_DFLT_THR;
	garg->cmn_args.num_mem_regions = MVAPPS_INVALID_MEMREGIONS;
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
			for (token = strtok(optarg, ","), garg->cmn_args.num_ports = 0;
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
			garg->cmn_args.burst = atoi(optarg);
			break;
		case 'u':
			garg->cmn_args.mtu = atoi(optarg);
			break;
		case 'c':
			garg->cmn_args.cpus = atoi(optarg);
			break;
		case 'a':
			garg->cmn_args.affinity = atoi(optarg);
			break;
		case 's':
			garg->maintain_stats = 1;
			break;
		case 'q':
			garg->rxq_size = atoi(optarg);
			break;
		case 'm':
			rc = sscanf(optarg, "%x:%x", (unsigned int *)&garg->cmn_args.qs_map,
				    &garg->cmn_args.qs_map_shift);
			if (rc != 2) {
				pr_err("Failed to parse -m parameter\n");
				return -EINVAL;
			}
			break;
		case 'l':
			garg->cmn_args.cli = 1;
			break;
		default:
			pr_err("parsing fail, wrong input\n");
			return -EINVAL;
		}
	}

	/* Now, check validity of all inputs */
	if (!garg->cmn_args.num_ports ||
	    !pp2_args->ports_desc[0].name) {
		pr_err("No port defined!\n");
		return -EINVAL;
	}
	if (garg->cmn_args.burst > PKT_FWD_APP_MAX_BURST_SIZE) {
		pr_err("illegal burst size requested (%d vs %d)!\n",
		       garg->cmn_args.burst, PKT_FWD_APP_MAX_BURST_SIZE);
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
	    (PKT_FWD_APP_MAX_NUM_TCS_PER_PORT == 1)) {
		pr_warn("no point in queues-mapping; ignoring.\n");
		garg->cmn_args.qs_map = 1;
		garg->cmn_args.qs_map_shift = 1;
	} else if (!garg->cmn_args.qs_map) {
		garg->cmn_args.qs_map = 1;
		garg->cmn_args.qs_map_shift = PKT_FWD_APP_MAX_NUM_TCS_PER_PORT;
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
	int			err;

	setbuf(stdout, NULL);
	app_set_max_num_qs_per_tc();

	pr_info("pkt-l3fwd is started:\n\t%s\n\t%s\n\t%s\n", app_mode_str, buf_release_str, tx_retry_str);

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

	cores_mask = apps_cores_mask_create(garg.cmn_args.cpus, garg.cmn_args.affinity);
	garg.cmn_args.cores_mask = cores_mask;

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
