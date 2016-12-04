/*******************************************************************************
	Copyright (C) 2016 Marvell International Ltd.

	If you received this File from Marvell, you may opt to use, redistribute
	and/or modify this File under the following licensing terms.
	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are met:

		* Redistributions of source code must retain the above copyright notice,
		  this list of conditions and the following disclaimer.

		* Redistributions in binary form must reproduce the above copyright
		  notice, this list of conditions and the following disclaimer in the
		  documentation and/or other materials provided with the distribution.

		* Neither the name of Marvell nor the names of its contributors may be
		  used to endorse or promote products derived from this software without
		  specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
	AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
	IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
	ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
	LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
	CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
	SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
	INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
	CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
	ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#include <string.h>
#include <stdio.h>

#include "mv_std.h"
#include "lib/lib_misc.h"
#include "env/mv_sys_dma.h"

#include "mvapp.h"
#include "mv_pp2.h"
#include "mv_pp2_hif.h"
#include "mv_pp2_bpool.h"
#include "mv_pp2_ppio.h"


#define MAX_NUM_BUFFS	8192
#define Q_SIZE		1024
#define MAX_BURST_SIZE	(Q_SIZE)>>1
#define DFLT_BURST_SIZE	256
#define PKT_OFFS	64
#define PP2_MH_SIZE	(2) /* TODO: take this from ppio definitions.*/
#define PKT_EFEC_OFFS	(PKT_OFFS+PP2_MH_SIZE)
#define MAX_PPIOS	1
#define MAX_NUM_CORES	1
#define MAX_NUM_QS	1
#define DMA_MEM_SIZE 	(4*1024*1024)
#define PP2_BPOOLS_RSRV	0x3
#define PP2_HIFS_RSRV	0xF
#define PP2_MAX_NUM_TCS_PER_PORT	1
#define PP2_MAX_NUM_QS_PER_TC		1

/* TODO: find more generic way to get the following parameters */
#define PP2_TOTAL_NUM_BPOOLS	16
#define PP2_TOTAL_NUM_HIFS	9

/* TODO: Move to internal .h file */
#define upper_32_bits(n) ((u32)(((n) >> 16) >> 16))
#define lower_32_bits(n) ((u32)(n))

//#define HW_BUFF_RECYLCE
//#define SW_BUFF_RECYLCE
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

//#define BPOOLS_INF	{{384,4096},{2048,1024}}
#define BPOOLS_INF	{{2048,1024}}

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
	const char	*name;
	int		 pp_id;
	int		 ppio_id;
};

struct bpool_inf {
	int	buff_size;
	int	num_buffs;
};

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
	char			 port_name[15];

	struct pp2_hif		*hif;
	struct pp2_ppio		*port;

	int			 num_pools;
	struct pp2_bpool	***pools;
	struct pp2_buff_inf	***buffs_inf;
};

struct local_arg {
	struct glob_arg		*garg;
	struct pp2_hif		*hif;
};

#ifndef HW_BUFF_RECYLCE
struct tx_shadow_q_entry {
#ifdef SW_BUFF_RECYLCE
	u32		 	buff_ptr;
#else
	struct pp2_buff_inf	buff_ptr;
#endif /* SW_BUFF_RECYLCE */
};

struct tx_shadow_q {
	u16				 read_ind;
	u16				 write_ind;

	struct tx_shadow_q_entry	 ents[Q_SIZE];
};
#endif /* !HW_BUFF_RECYLCE */


static struct glob_arg garg = {};
static u64 sys_dma_high_addr = 0;
#ifndef HW_BUFF_RECYLCE
static struct tx_shadow_q shadow_qs[MAX_NUM_CORES][MAX_NUM_QS];
#endif /* !HW_BUFF_RECYLCE */
static u16	used_bpools = 0;
static u16	used_hifs = 0;


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
	if (unlikely(l4_inq == PP2_INQ_L4_TYPE_OTHER))
		return(PP2_OUTQ_L4_TYPE_OTHER);

	return(l4_inq - 1);
}
#endif

/* TODO: find a better way to map the ports!!! */
static int find_port_info(struct port_desc *port_desc)
{
/* TODO: temporary A7040 table! */
#define DEV2PORTS_MAP	\
{			\
	{"eth1", 0, 0},	\
	{"eth2", 0, 1},	\
	{"eth3", 0, 2},	\
}
	struct port_desc ports_map[] = DEV2PORTS_MAP;
	int		 i, num;

	if (!port_desc->name) {
		pr_err("No port name given!\n");
		return -EINVAL;
	}

	num = sizeof(ports_map)/sizeof(struct port_desc);

	for (i=0; i<num; i++)
		if (strcmp(port_desc->name, ports_map[i].name) == 0) {
			port_desc->pp_id = ports_map[i].pp_id;
			port_desc->ppio_id = ports_map[i].ppio_id;
			break;
		}

	if (i == num) {
		pr_err("port (%s) not found!\n", port_desc->name);
		return -ENODEV;
	}
	return 0;
}

static int find_free_bpool(void)
{
	int	i;

	for (i=0; i<PP2_TOTAL_NUM_BPOOLS; i++) {
		if (!((1<<i) & PP2_BPOOLS_RSRV) &&
		    !((uint64_t)(1<<i) & used_bpools)) {
			    used_bpools |= (uint64_t)(1<<i);
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
		if (!((1<<i) & PP2_HIFS_RSRV) &&
		    !((uint64_t)(1<<i) & used_hifs)) {
			    used_hifs |= (uint64_t)(1<<i);
			    break;
		    }
	}
	if (i == PP2_TOTAL_NUM_HIFS) {
		pr_err("no free HIF found!\n");
		return -ENOSPC;
	}
	return i;
}

static int main_loop(void *arg, volatile int *running)
{
	struct glob_arg		*garg = (struct glob_arg *)arg;
	struct pp2_ppio_desc	 descs[MAX_BURST_SIZE];
	int			 err;
	u16			 i,num;
#ifdef HW_TX_CHKSUM_CALC
	enum pp2_inq_l3_type     l3_type;
	enum pp2_inq_l4_type     l4_type;
	u8                       l3_offset, l4_offset;
#endif
	if (!garg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}
	while (*running) {
		struct pp2_bpool	*pool = garg->pools[0][0];
#ifndef HW_BUFF_RECYLCE
		struct tx_shadow_q	*shadow_q = &(shadow_qs[0][0]);
#endif /* !HW_BUFF_RECYLCE */
#ifdef PKT_ECHO_SUPPORT
		int			 prefetch_shift = garg->prefetch_shift;
#endif /* PKT_ECHO_SUPPORT */

		num = garg->burst;
		err = pp2_ppio_recv(garg->port, 0, 0, descs, &num);

		for (i=0; i<num; i++) {
#ifdef SW_BUFF_RECYLCE
			u32		 ck = pp2_ppio_inq_desc_get_cookie(&descs[i]);
			char		*buff = (char *)(uintptr_t)garg->buffs_inf[COOKIE_GET_PP(ck)][COOKIE_GET_BPOOL(ck)][COOKIE_GET_INDX(ck)].cookie;
			dma_addr_t	 pa = garg->buffs_inf[COOKIE_GET_PP(ck)][COOKIE_GET_BPOOL(ck)][COOKIE_GET_INDX(ck)].addr;
#else
			char		*buff = (char *)(uintptr_t)pp2_ppio_inq_desc_get_cookie(&descs[i]);
			dma_addr_t	 pa = pp2_ppio_inq_desc_get_phys_addr(&descs[i]);
#endif /* SW_BUFF_RECYLCE */
			u16 len = pp2_ppio_inq_desc_get_pkt_len(&descs[i]) - PP2_MH_SIZE;

#ifdef PKT_ECHO_SUPPORT
			if (likely(garg->echo)) {
				char *tmp_buff;
#ifdef USE_APP_PREFETCH
				if (num-i > prefetch_shift) {
#ifdef SW_BUFF_RECYLCE
					u32 ck = pp2_ppio_inq_desc_get_cookie(&descs[i+prefetch_shift]);
					tmp_buff = (char *)(uintptr_t)garg->buffs_inf[COOKIE_GET_PP(ck)][COOKIE_GET_BPOOL(ck)][COOKIE_GET_INDX(ck)].cookie;
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
#endif

			pp2_ppio_outq_desc_reset(&descs[i]);
#ifdef HW_TX_CHKSUM_CALC
#if (HW_TX_IPV4_CHKSUM_CALC || HW_TX_L4_CHKSUM_CALC)
			pp2_ppio_outq_desc_set_proto_info(&descs[i], pp2_l3_type_inq_to_outq(l3_type),
							  pp2_l4_type_inq_to_outq(l4_type), (l3_offset - PP2_MH_SIZE),
							  (l4_offset - PP2_MH_SIZE), HW_TX_IPV4_CHKSUM_CALC, HW_TX_L4_CHKSUM_CALC);
#endif
#endif
			pp2_ppio_outq_desc_set_phys_addr(&descs[i], pa);
			pp2_ppio_outq_desc_set_pkt_offset(&descs[i], PKT_EFEC_OFFS);
			pp2_ppio_outq_desc_set_pkt_len(&descs[i], len);
#ifdef HW_BUFF_RECYLCE
			pp2_ppio_outq_desc_set_cookie(&descs[i], (uintptr_t)(buff));
			pp2_ppio_outq_desc_set_pool(&descs[i], pool);
#else
#ifdef SW_BUFF_RECYLCE
			pp2_ppio_outq_desc_set_cookie(&descs[i], ck);
			shadow_q->ents[shadow_q->write_ind].buff_ptr = ck;
#else
			shadow_q->ents[shadow_q->write_ind].buff_ptr.cookie = (uintptr_t)buff;
			shadow_q->ents[shadow_q->write_ind].buff_ptr.addr = pa;
#endif /* SW_BUFF_RECYLCE */
			pr_debug("buff_ptr.cookie(0x%lx)\n", (u64)shadow_q->ents[shadow_q->write_ind].buff_ptr.cookie);
			shadow_q->write_ind++;
			if (shadow_q->write_ind == Q_SIZE)
				shadow_q->write_ind = 0;
#endif /* HW_BUFF_RECYLCE */
		}
		if (num && ((err = pp2_ppio_send(garg->port, garg->hif, 0, descs, &num)) != 0))
			return err;

#ifndef HW_BUFF_RECYLCE
		pp2_ppio_get_num_outq_done(garg->port, garg->hif, 0, &num);
		for (i=0; i<num; i++) {
			struct pp2_buff_inf	*binf;
#ifdef SW_BUFF_RECYLCE
			struct pp2_buff_inf	 tmp_buff_inf;
			u32	ck = shadow_q->ents[shadow_q->read_ind].buff_ptr;
			tmp_buff_inf.cookie = ck;
			tmp_buff_inf.addr   = garg->buffs_inf[COOKIE_GET_PP(ck)][COOKIE_GET_BPOOL(ck)][COOKIE_GET_INDX(ck)].addr;
			binf = &tmp_buff_inf;
#else
			binf = &(shadow_q->ents[shadow_q->read_ind].buff_ptr);
#endif /* SW_BUFF_RECYLCE */
			if (unlikely(!binf->cookie || !binf->addr)) {
				pr_err("[%s] shadow memory cookie(%lux) addr(%lux)!\n", __FUNCTION__,
					(u64)binf->cookie, (u64)binf->addr);
				sleep(1);
				return -1;
			}
			pp2_bpool_put_buff(garg->hif, pool, binf);
			shadow_q->read_ind++;
			if (shadow_q->read_ind == Q_SIZE)
				shadow_q->read_ind = 0;
		}
#endif /* !HW_BUFF_RECYLCE */
	}

	return 0;
}

static int init_all_modules(void)
{
	struct pp2_init_params	 pp2_params;
	int			 err;

	pr_info("Global initializations ... ");

	if ((err = mv_sys_dma_mem_init(DMA_MEM_SIZE)) != 0)
		return err;

	memset(&pp2_params, 0, sizeof(pp2_params));
	pp2_params.hif_reserved_map = 0;
	pp2_params.bm_pool_reserved_map = 0;
	pp2_params.ppios[0][0].is_enabled = 1;
	pp2_params.ppios[0][0].first_inq = 0;
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

	garg->pools = (struct pp2_bpool ***)malloc(PP2_SOC_NUM_PACKPROCS*sizeof(struct pp2_bpool **));
	if (!garg->pools) {
		pr_err("no mem for bpools array!\n");
		return -ENOMEM;
	}
	garg->buffs_inf =
		(struct pp2_buff_inf ***)malloc(PP2_SOC_NUM_PACKPROCS*sizeof(struct pp2_buff_inf **));
	if (!garg->buffs_inf) {
		pr_err("no mem for bpools-inf array!\n");
		return -ENOMEM;
	}
	for (i=0; i<PP2_SOC_NUM_PACKPROCS; i++) {
		garg->num_pools = ARRAY_SIZE(infs);
		/* TODO: temporary W/A until we have map routines of bpools to ppios */
		if (garg->num_pools > PP2_PPIO_TC_MAX_POOLS) {
			pr_err("only %d pools allowed!\n", PP2_PPIO_TC_MAX_POOLS);
			return -EINVAL;
		}
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
			if ((pool_id = find_free_bpool()) < 0) {
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
#ifdef SW_BUFF_RECYLCE
				/* Don't add first buffer into BPool */
				if (k == 0) continue;
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
	struct port_desc		port_desc;
	char				name[15];
	int			 	i, j, err, hif_id;

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

	memset(&port_desc, 0, sizeof(port_desc));
	memset(name, 0, sizeof(name));
	snprintf(name, sizeof(name), "%s", garg->port_name);
	port_desc.name = name;
	if ((err = find_port_info(&port_desc)) != 0) {
		pr_err("Port info not found!\n");
		return err;
	}

	memset(name, 0, sizeof(name));
	snprintf(name, sizeof(name), "ppio-%d:%d", port_desc.pp_id, port_desc.ppio_id);
	pr_debug("found port: %s\n", name);
	memset(&port_params, 0, sizeof(port_params));
	port_params.match = name;
	port_params.type = PP2_PPIO_T_NIC;
	port_params.inqs_params.num_tcs = PP2_MAX_NUM_TCS_PER_PORT;
	for (i=0; i<port_params.inqs_params.num_tcs; i++) {
		port_params.inqs_params.tcs_params[0].pkt_offset = PKT_OFFS>>2;
		port_params.inqs_params.tcs_params[0].num_in_qs = PP2_MAX_NUM_QS_PER_TC;
		/* TODO: we assume here only one Q per TC; change it! */
		inq_params.size = Q_SIZE;
		port_params.inqs_params.tcs_params[0].inqs_params = &inq_params;
		for (j=0; j<garg->num_pools; j++)
			port_params.inqs_params.tcs_params[0].pools[j] = garg->pools[0][j];
	}
	port_params.outqs_params.num_outqs = PP2_MAX_NUM_TCS_PER_PORT;
	for (i=0; i<port_params.outqs_params.num_outqs; i++) {
		port_params.outqs_params.outqs_params[0].size = Q_SIZE;
		port_params.outqs_params.outqs_params[0].weight = 1;
	}
	if ((err = pp2_ppio_init(&port_params, &garg->port)) != 0)
		return err;
	if (!garg->port) {
		pr_err("PP-IO init failed!\n");
		return -EIO;
	}

	if ((err = pp2_ppio_enable(garg->port)) != 0)
		return err;
	if ((err = pp2_ppio_set_uc_promisc(garg->port, 1)) != 0)
		return err;

	pr_info("done\n");
	return 0;
}

static void destroy_local_modules(struct glob_arg *garg)
{
	int	i, j;

	if (garg->port) {
		pp2_ppio_disable(garg->port);
		pp2_ppio_deinit(garg->port);
	}

	if (garg->pools) {
		for (i=0; i<PP2_SOC_NUM_PACKPROCS; i++) {
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
		for (i=0; i<PP2_SOC_NUM_PACKPROCS; i++) {
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

static int init_local(void *arg, void **larg)
{
	struct glob_arg *garg = (struct glob_arg *)arg;

	if (!garg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	*larg = garg;
	return 0;
}

static void deinit_local(void *arg)
{
	struct glob_arg *garg = (struct glob_arg *)arg;

	if (!garg)
		return;
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
			if (argc < (i+2)) {
				pr_err("Invalid number of arguments!\n");
				return -EINVAL;
			}
			if (argv[i+1][0] == '-') {
				pr_err("Invalid arguments format!\n");
				return -EINVAL;
			}
			snprintf(garg->port_name, sizeof(garg->port_name), "%s", argv[i+1]);
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
	if (!garg->port_name[0]) {
		pr_err("No port defined!\n");
		return -EINVAL;
	}
	if (garg->burst > MAX_BURST_SIZE) {
		pr_err("illegal burst size requested (%d vs %d)!\n",
			garg->burst, MAX_BURST_SIZE);
		return -EINVAL;
	}
	if (garg->cpus > MAX_NUM_CORES) {
		pr_err("illegal burst size requested (%d vs %d)!\n",
			garg->cpus, MAX_NUM_CORES);
		return -EINVAL;
	}
	if (garg->qs_map || garg->qs_map_shift) {
		pr_err("Queues mapping not supported yet!\n");
		return -ENOTSUP;
	}
	if (garg->affinity != -1) {
		pr_err("Affinity not supported yet!\n");
		return -ENOTSUP;
	}

	return 0;
}


int main (int argc, char *argv[])
{
	struct mvapp_params	mvapp_params;
	int			err;

	setbuf(stdout, NULL);

	pr_debug("pr_debug is enabled\n");

	if ((err = parse_args(&garg, argc, argv)) != 0)
		return err;

	memset(&mvapp_params, 0, sizeof(mvapp_params));
	mvapp_params.use_cli		= garg.cli;
	mvapp_params.num_cores		= garg.cpus;
	mvapp_params.global_arg		= (void *)&garg;
	mvapp_params.init_global_cb	= init_global;
	mvapp_params.deinit_global_cb	= deinit_global;
	mvapp_params.init_local_cb	= init_local;
	mvapp_params.deinit_local_cb	= deinit_local;
	mvapp_params.main_loop_cb	= main_loop;
	return mvapp_go(&mvapp_params);
}
