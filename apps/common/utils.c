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

#include <string.h>
#include <stdio.h>
#include <getopt.h>
#include <sys/time.h>
#include <arpa/inet.h>	/* ntohs */
#include <netinet/ip.h>
#include <netinet/udp.h>

#include "mv_std.h"
#include "lib/lib_misc.h"
#include "env/mv_sys_dma.h"
#include <stdbool.h>
#include "lib/net.h"

#include "utils.h"


#define ETHERTYPE_IP	0x800


struct pkt {
	struct ether_header	 eh;
	struct ip		 ip;
	struct udphdr		 udp;
	/* using 'empty array' as placeholder; the real size will be determine by the specific application
	 */
	u8			 body[];
} __packed;


uintptr_t cookie_high_bits = MVAPPS_INVALID_COOKIE_HIGH_BITS;


static void rand_permute(int *arr, int n)
{
	int i;

	for (i = 0; i < n; i++) {
		int rand_idx = rand() % n;
		int t;

		/* swap arr[i] with arr[rand_idx] */
		t = arr[i];
		arr[i] = arr[rand_idx];
		arr[rand_idx] = t;
	}
}

/*
 * increment the addressed in the packet,
 * starting from the least significant field.
 *	DST_IP DST_PORT SRC_IP SRC_PORT
 */
static void update_addresses(struct pkt *pkt, struct ip_range *src_ipr, struct ip_range *dst_ipr)
{
	struct ip *ip = &pkt->ip;
	struct udphdr *udp = &pkt->udp;

	do {
		/* XXX for now it doesn't handle non-random src, random dst */
		udp->uh_sport = htons(src_ipr->port_curr++);
		if (src_ipr->port_curr >= src_ipr->port1)
			src_ipr->port_curr = src_ipr->port0;

		ip->ip_src.s_addr = htonl(src_ipr->curr++);
		if (src_ipr->curr >= src_ipr->end)
			src_ipr->curr = src_ipr->start;

		udp->uh_dport = htons(dst_ipr->port_curr++);
		if (dst_ipr->port_curr >= dst_ipr->port1)
			dst_ipr->port_curr = dst_ipr->port0;

		ip->ip_dst.s_addr = htonl(dst_ipr->curr++);
		if (dst_ipr->curr >= dst_ipr->end)
			dst_ipr->curr = dst_ipr->start;
	} while (0);
	/* update checksum */
}


int app_build_pkt_pool(void			**mem,
		       struct buffer_desc	*buffs,
		       u16			 num_buffs,
		       u16			 min_pkt_size,
		       u16			 max_pkt_size,
		       int			 pkt_size,
		       struct ip_range		*src_ipr,
		       struct ip_range		*dst_ipr,
		       eth_addr_t		 src_mac,
		       eth_addr_t		 dst_mac
		       )
{
	u8	*buffer;
	int	i;
	int	imix_sizes_arr[] = {60, 60, 60, 60, 60, 60, 60, 566, 566, 566, 566, 1514};
	int	num_imix_vars = sizeof(imix_sizes_arr)/sizeof(int);

	if (min_pkt_size < MVAPPS_PLD_MIN_SIZE) {
		pr_err("illegal minimum pkt len (%d vs %d)!\n", min_pkt_size, MVAPPS_PLD_MIN_SIZE);
		return -EINVAL;
	}

	buffer = mv_sys_dma_mem_alloc(max_pkt_size * num_buffs, 4);
	if (!buffer) {
		pr_err("no mem for local packets buffer!\n");
		return -ENOMEM;
	}

	for (i = 0; i < num_buffs; i++) {
		struct pkt	*pkt;
		u32		*dat;

		pkt = (struct pkt *)(buffer + i *  max_pkt_size);
		buffs[i].virt_addr = (void *)pkt;
		buffs[i].phy_addr = mv_sys_dma_mem_virt2phys(buffs[i].virt_addr);

		if (pkt_size == MVAPPS_PKT_SIZE_INC)
			buffs[i].size = (min_pkt_size + 4 * i) % max_pkt_size;
		else if (pkt_size == MVAPPS_PKT_SIZE_RAND)
			buffs[i].size = min_pkt_size + (rand() % (max_pkt_size - min_pkt_size));
		else if (pkt_size == MVAPPS_PKT_SIZE_IMIX) {
			if ((i % num_imix_vars) == 0)
				rand_permute(imix_sizes_arr, num_imix_vars);
			buffs[i].size = imix_sizes_arr[i % num_imix_vars];
		} else
			buffs[i].size = pkt_size;
		pr_debug("got pkt size: %d\n", buffs[i].size);

		memcpy(pkt->eh.ether_dmac, dst_mac, MV_ETH_ALEN);
		memcpy(pkt->eh.ether_smac, src_mac, MV_ETH_ALEN);

		pkt->eh.ether_type = htons(ETHERTYPE_IP);

		pkt->ip.ip_src.s_addr = htonl(src_ipr->start);
		pkt->ip.ip_dst.s_addr = htonl(dst_ipr->start);
		pkt->ip.ip_v = IPVERSION;
		pkt->ip.ip_hl = 5;
		pkt->ip.ip_id = 0;
		pkt->ip.ip_tos = IPTOS_LOWDELAY;
		pkt->ip.ip_len = htons(buffs[i].size - sizeof(struct ether_header));
		pkt->ip.ip_id = 0;
		pkt->ip.ip_off = htons(IP_DF); /* Don't fragment */
		pkt->ip.ip_ttl = IPDEFTTL;
		pkt->ip.ip_p = IPPROTO_UDP;

		pkt->udp.uh_sport = htons(src_ipr->port0);
		pkt->udp.uh_dport = htons(dst_ipr->port0);
		pkt->udp.uh_ulen = htons(buffs[i].size - sizeof(struct ether_header) - sizeof(struct ip));
		pkt->udp.uh_sum = 0;

		update_addresses((struct pkt *)buffs[i].virt_addr, src_ipr, dst_ipr);

		/* Set IP checksum */
		pkt->ip.ip_sum = 0;
		pkt->ip.ip_sum = mv_ip4_csum((u16 *)&pkt->ip, pkt->ip.ip_hl);

		dat = (u32 *)pkt->body;
		dat[0] = MVAPPS_PLD_WATERMARK;
		dat[1] = i;
	}

	*mem = buffer;
	return 0;
}

int apps_cores_mask_create(int cpus, int affinity)
{
	u64 cores_mask_orig = 0, max_cores_mask, cores_mask;

	max_cores_mask = (1 << system_ncpus());
	cores_mask_orig = (1 << cpus) - 1;

	cores_mask = cores_mask_orig;
	cores_mask <<= (affinity != MVAPPS_INVALID_AFFINITY) ? affinity : MVAPPS_DEFAULT_AFFINITY;

	/* cores_mask must stay in CPU boundaries */
	if (cores_mask > max_cores_mask)
		cores_mask = cores_mask_orig;

	pr_debug("%s: cpus:%d, affinity:%d, cores_mask:0x%lx\n", __func__, cpus, affinity, cores_mask);
	return cores_mask;
}

int apps_thread_to_cpu(struct glb_common_args *cmn_args, int thread)
{
	int j = 0, i = 0;
	u64 core_mask;

	if (cmn_args->cores_mask)
		core_mask = cmn_args->cores_mask;
	else
		core_mask = apps_cores_mask_create(cmn_args->cpus, cmn_args->affinity);

	core_mask = apps_cores_mask_create(cmn_args->cpus, cmn_args->affinity);

	while (core_mask) {
		for (; !((1 << j) & core_mask); j++)
			;

		if (i++ == thread)
			return j;

		core_mask &= ~(1 << j);
	}
	printf("CPU not found for thread #%d\n", thread);
	return -1;
}

int apps_cpu_to_thread(struct glb_common_args *cmn_args, int cpu)
{
	int j = 0, i = 0;
	u64 core_mask;

	if (cmn_args->cores_mask)
		core_mask = cmn_args->cores_mask;
	else
		core_mask = apps_cores_mask_create(cmn_args->cpus, cmn_args->affinity);

	while (core_mask) {
		for (; !((1 << j) & core_mask); j++)
			;

		if (j == cpu)
			return i;

		i++;
		core_mask &= ~(1 << j);
	}
	printf("Thread not found for CPU #%d\n", cpu);
	return -1;

}

int apps_perf_dump(struct glb_common_args *cmn_args)
{
	struct timeval	curr_time;
	u64		tmp_time_inter;
	u64		tmp_rx_cnt = 0, tmp_tx_cnt = 0, drop_cnt = 0;
	u64		tmp_enc_cnt = 0, tmp_dec_cnt = 0;
	int		i;
	struct perf_cmn_cntrs *perf_cntrs;

	gettimeofday(&curr_time, NULL);
	tmp_time_inter = (curr_time.tv_sec - cmn_args->ctrl_trd_last_time.tv_sec) * 1000;
	tmp_time_inter += (curr_time.tv_usec - cmn_args->ctrl_trd_last_time.tv_usec) / 1000;

	for (i = 0; i < MVAPPS_MAX_NUM_CORES; i++) {
		if (cmn_args->largs[i]) {
			perf_cntrs = &((struct local_common_args *)cmn_args->largs[i])->perf_cntrs;
			drop_cnt   += perf_cntrs->drop_cnt;
			tmp_rx_cnt += perf_cntrs->rx_cnt;
			tmp_tx_cnt += perf_cntrs->tx_cnt;
			tmp_enc_cnt += perf_cntrs->enc_cnt;
			tmp_dec_cnt += perf_cntrs->dec_cnt;
		}
	}
	printf("Perf: RX: %d Kpps",
	       (int)((tmp_rx_cnt - cmn_args->last_rx_cnt) / tmp_time_inter));

	if (tmp_enc_cnt - cmn_args->last_enc_cnt)
		printf(" -> Enc: %d Kpps",
		       (int)((tmp_enc_cnt - cmn_args->last_enc_cnt) / tmp_time_inter));

	if (tmp_dec_cnt - cmn_args->last_dec_cnt)
		printf(" -> Dec: %d Kpps",
		       (int)((tmp_dec_cnt - cmn_args->last_dec_cnt) / tmp_time_inter));

	printf(" -> TX: %d Kpps",
	       (int)((tmp_tx_cnt - cmn_args->last_tx_cnt) / tmp_time_inter));

	cmn_args->last_rx_cnt = tmp_rx_cnt;
	cmn_args->last_tx_cnt = tmp_tx_cnt;
	cmn_args->last_enc_cnt = tmp_enc_cnt;
	cmn_args->last_dec_cnt = tmp_dec_cnt;
	if (drop_cnt)
		printf(", drop: %"PRIu64"", drop_cnt);
	printf("\n");
	gettimeofday(&cmn_args->ctrl_trd_last_time, NULL);

	return 0;
}

int app_ctrl_cb(void *arg)
{
	struct glb_common_args	*cmn_args = (struct glb_common_args *)arg;
	struct timeval	 curr_time;
	u64		 tmp_time_inter;

	if (!cmn_args) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	gettimeofday(&curr_time, NULL);
	tmp_time_inter = (curr_time.tv_sec - cmn_args->ctrl_trd_last_time.tv_sec) * 1000;
	tmp_time_inter += (curr_time.tv_usec - cmn_args->ctrl_trd_last_time.tv_usec) / 1000;
	if (tmp_time_inter >= cmn_args->ctrl_thresh)
		return apps_perf_dump(cmn_args);
	return 0;
}

int apps_prefetch_cmd_cb(void *arg, int argc, char *argv[])
{
	struct glb_common_args *g_cmn_args = (struct glb_common_args *)arg;

	if (!g_cmn_args) {
		pr_err("no garg obj passed!\n");
		return -EINVAL;
	}
	if ((argc != 1) && (argc != 2)) {
		pr_err("Invalid number of arguments for prefetch cmd!\n");
		return -EINVAL;
	}

	if (argc == 1) {
		printf("%d\n", g_cmn_args->prefetch_shift);
		return 0;
	}

	g_cmn_args->prefetch_shift = atoi(argv[1]);

	return 0;
}

void app_print_horizontal_line(u32 char_count, const char *char_val)
{
	u32 cnt;

	for (cnt = 0; cnt < char_count; cnt++)
		printf("%s", char_val);
	printf("\n");
}

int app_range_validate(int value, int min, int max)
{
	if (((value) > (max)) || ((value) < (min))) {
		pr_err("%s: value 0x%X (%d) is out of range [0x%X , 0x%X].\n",
			__func__, (value), (value), (min), (max));
		return -EFAULT;
	}
	return 0;
}

int app_parse_mac_address(char *buf, u8 *macaddr_parts)
{
	if (sscanf(buf, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
		   &macaddr_parts[0], &macaddr_parts[1],
		   &macaddr_parts[2], &macaddr_parts[3],
		   &macaddr_parts[4], &macaddr_parts[5]) == ETH_ALEN)
		return 0;
	else
		return -EFAULT;
}
