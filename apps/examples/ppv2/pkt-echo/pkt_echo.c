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
#include "lib/misc.h"
#include "sys_dma.h"
#include "mvapp.h"

#include "mv_pp2.h"
#include "mv_pp2_hif.h"
#include "mv_pp2_bpool.h"
#include "mv_pp2_ppio.h"


#define BUF_LEN		2048
#define Q_SIZE		1024
#define BURST_SIZE	16
#define PKT_OFFS	64
#define PP2_MH_SIZE	(2) /* TODO: take this from ppio definitions.*/
#define PKT_EFEC_OFFS	(PKT_OFFS+PP2_MH_SIZE)

#define DO_PKT_ECHO
#define USE_APP_PREFETCH
#define PREFETCH_SHIFT	4


struct glob_arg {
	int			 verbose;
	struct pp2_hif		*hif;
	struct pp2_bpool	*pool;
	struct pp2_ppio		*port;
};

struct local_arg {
	struct glob_arg		*garg;
	struct pp2_hif		*hif;
};


static struct glob_arg garg = {};


#ifdef USE_APP_PREFETCH
static inline void prefetch(const void *ptr)
{
	asm volatile("prfm pldl1keep, %a0\n" : : "p" (ptr));
}
#endif /* USE_APP_PREFETCH */

#ifdef DO_PKT_ECHO
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
#endif /* DO_PKT_ECHO */


static int main_loop(void *arg, volatile int *running)
{
	struct glob_arg		*garg = (struct glob_arg *)arg;
	struct pp2_ppio_desc	 descs[BURST_SIZE];
	int			 err;
	u16			 i,num;

	if (!garg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	while (*running) {
		num = BURST_SIZE;
		err = pp2_ppio_recv(garg->port, 0, 0, descs, &num);

		for (i=0; i<num; i++) {
			char *buff = (char *)(uintptr_t)pp2_ppio_inq_desc_get_cookie(&descs[i])+PKT_EFEC_OFFS;
			dma_addr_t pa = pp2_ppio_inq_desc_get_phys_addr(&descs[i]);
			u16 len = pp2_ppio_inq_desc_get_pkt_len(&descs[i]) - PP2_MH_SIZE;

//printf("packet:\n"); mem_disp(buff, len);
#ifdef DO_PKT_ECHO
#ifdef USE_APP_PREFETCH
			if (num-i > PREFETCH_SHIFT) {
				char *tmp_buff = (char *)(uintptr_t)pp2_ppio_inq_desc_get_cookie(&descs[i+PREFETCH_SHIFT]);
				tmp_buff += PKT_EFEC_OFFS;
				prefetch(tmp_buff);
			}
#endif /* USE_APP_PREFETCH */

			swap_l2(buff);
			swap_l3(buff);
//printf("packet:\n"); mem_disp(buff, len);
#endif /* DO_PKT_ECHO */

			pp2_ppio_outq_desc_reset(&descs[i]);
			pp2_ppio_outq_desc_set_phys_addr(&descs[i], pa);
			pp2_ppio_outq_desc_set_cookie(&descs[i], (uintptr_t)(buff-PKT_EFEC_OFFS));
			pp2_ppio_outq_desc_set_pkt_offset(&descs[i], PKT_EFEC_OFFS);
			pp2_ppio_outq_desc_set_pkt_len(&descs[i], len);
			pp2_ppio_outq_desc_set_pool(&descs[i], garg->pool);
/*
printf("tx-desc:\n");
printf("desc@0:\n\t%08X %08X %08X %08X\n\t\%08X %08X %08X %08X\n",
descs[i].cmds[0],
descs[i].cmds[1],
descs[i].cmds[2],
descs[i].cmds[3],
descs[i].cmds[4],
descs[i].cmds[5],
descs[i].cmds[6],
descs[i].cmds[7]);
*/
		}

		if (num && ((err = pp2_ppio_send(garg->port, garg->hif, 0, descs, &num)) != 0))
			return err;

/*
		if ((err = pp2_ppio_get_num_outq_done(garg->port, garg->hif, 0, &num)) != 0)
			return err;
*/
	}

	return 0;
}

static int init_all_modules(void)
{
	struct pp2_init_params	 pp2_params;
	int			 err;

	pr_info("Global initializations ... ");

	if ((err = mv_sys_dma_mem_init(4*1024*1024)) != 0)
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

static int init_local_modules(struct glob_arg *garg)
{
	struct pp2_hif_params	 	hif_params;
	struct pp2_bpool_params	 	bpool_params;
	struct pp2_ppio_params	 	port_params;
	struct pp2_ppio_inq_params	inq_params;
	int			 	i, err;

	pr_info("Local initializations ... ");

	memset(&hif_params, 0, sizeof(hif_params));
	hif_params.match = "hif-0";
	hif_params.out_size = Q_SIZE;
	if ((err = pp2_hif_init(&hif_params, &garg->hif)) != 0)
		return err;
	if (!garg->hif) {
		pr_err("HIF init failed!\n");
		return -EIO;
	}

	memset(&bpool_params, 0, sizeof(bpool_params));
	bpool_params.match = "pool-0:0";
	bpool_params.max_num_buffs = 8192;
	bpool_params.buff_len = BUF_LEN;
	if ((err = pp2_bpool_init(&bpool_params, &garg->pool)) != 0)
		return err;
	if (!garg->pool) {
		pr_err("BPool init failed!\n");
		return -EIO;
	}
	for (i=0; i<1024; i++) {
		struct pp2_buff_inf buff;
		buff.cookie = (uintptr_t)mv_sys_dma_mem_alloc(BUF_LEN, 4);
		if (!buff.cookie) {
			pr_err("failed to allocate mem (%d)!\n", i);
			return -1;
		}
		buff.addr = mv_sys_dma_mem_virt2phys((void*)(uintptr_t)buff.cookie);
		if ((err = pp2_bpool_put_buff(garg->hif, garg->pool, &buff)) != 0)
			return err;
	}

	memset(&port_params, 0, sizeof(port_params));
	port_params.match = "ppio-0:0";
	port_params.type = PP2_PPIO_T_NIC;
	port_params.inqs_params.num_tcs = 1;
	port_params.inqs_params.tcs_params[0].pkt_offset = PKT_OFFS>>2;
	port_params.inqs_params.tcs_params[0].num_in_qs = 1;
	inq_params.size = Q_SIZE;
	port_params.inqs_params.tcs_params[0].inqs_params = &inq_params;
	port_params.inqs_params.tcs_params[0].pools[0] = garg->pool;
	port_params.outqs_params.num_outqs = 1;
	port_params.outqs_params.outqs_params[0].size = Q_SIZE;
	port_params.outqs_params.outqs_params[0].weight = 1;
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
	if (garg->port) {
		pp2_ppio_disable(garg->port);
		pp2_ppio_deinit(garg->port);
	}
	if (garg->pool)
		pp2_bpool_deinit(garg->pool);
	if (garg->hif)
		pp2_hif_deinit(garg->hif);
}

static void destroy_all_modules(void)
{
	pp2_deinit();
	mv_sys_dma_mem_destroy();
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

	return 0;
}

static void deinit_global(void *arg)
{
	struct glob_arg *garg = (struct glob_arg *)arg;

	if (!garg)
		return;
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


int main (int argc, char *argv[])
{
	struct mvapp_params	mvapp_params;

	setbuf(stdout, NULL);

	printf("Marvell Armada US (Build: %s %s)\n", __DATE__, __TIME__);

	memset(&mvapp_params, 0, sizeof(mvapp_params));
	mvapp_params.use_cli		= 1;
	mvapp_params.num_cores		= 1;
	mvapp_params.global_arg		= (void *)&garg;
	mvapp_params.init_global_cb	= init_global;
	mvapp_params.deinit_global_cb	= deinit_global;
	mvapp_params.init_local_cb	= init_local;
	mvapp_params.deinit_local_cb	= deinit_local;
	mvapp_params.main_loop_cb	= main_loop;
	return mvapp_go(&mvapp_params);
}
