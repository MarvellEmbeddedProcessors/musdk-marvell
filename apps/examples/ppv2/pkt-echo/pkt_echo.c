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


#define BUF_LEN		2048
#define MAX_NUM_BUFFS	8192
#define NUM_BUFFS	1024
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

/* TODO: Move to internal .h file */
#define upper_32_bits(n) ((u32)(((n) >> 16) >> 16))
#define lower_32_bits(n) ((u32)(n))

//#define HW_BUFF_RECYLCE
#define PKT_ECHO_SUPPORT
#define USE_APP_PREFETCH
#define PREFETCH_SHIFT	7

/** Get rid of path in filename - only for unix-type paths using '/' */
#define NO_PATH(file_name) (strrchr((file_name), '/') ? \
			    strrchr((file_name), '/') + 1 : (file_name))



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

	struct pp2_hif		*hif;
	struct pp2_bpool	*pool;
	struct pp2_ppio		*port;
};


struct local_arg {
	struct glob_arg		*garg;
	struct pp2_hif		*hif;
};

struct tx_shadow_q_entry {
	u16		 buff_inf;
	u16		 res;
	struct pp2_buff_inf buff_ptr;
};

struct tx_shadow_q {
	u16				 read_ind;
	u16				 write_ind;

	struct tx_shadow_q_entry	 ents[Q_SIZE];
};
static struct glob_arg garg = {};
static struct tx_shadow_q shadow_qs[MAX_NUM_CORES][MAX_NUM_QS];
u64 sys_dma_high_addr = 0;


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

static int main_loop(void *arg, volatile int *running)
{
	struct glob_arg		*garg = (struct glob_arg *)arg;
	struct pp2_ppio_desc	 descs[MAX_BURST_SIZE];
	int			 err;
	u16			 i,num;

	if (!garg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}
	while (*running) {
#ifdef PKT_ECHO_SUPPORT
		int			 prefetch_shift = garg->prefetch_shift;
#endif /* PKT_ECHO_SUPPORT */

		num = garg->burst;
		struct tx_shadow_q * shadow_q = &(shadow_qs[0][0]);


		err = pp2_ppio_recv(garg->port, 0, 0, descs, &num);

		for (i=0; i<num; i++) {
			char *buff = (char *)(uintptr_t)pp2_ppio_inq_desc_get_cookie(&descs[i]);
			dma_addr_t pa = pp2_ppio_inq_desc_get_phys_addr(&descs[i]);
			u16 len = pp2_ppio_inq_desc_get_pkt_len(&descs[i]) - PP2_MH_SIZE;
#ifdef PKT_ECHO_SUPPORT
			char *buff2;
			if (garg->echo) {
#ifdef USE_APP_PREFETCH
				if (num-i > prefetch_shift) {
					char *tmp_buff = (char *)(uintptr_t)pp2_ppio_inq_desc_get_cookie(&descs[i+prefetch_shift]);
					tmp_buff +=PKT_EFEC_OFFS;
					pr_debug("tmp_buff_before(%p)\n", tmp_buff);
					tmp_buff = (char *)(((uintptr_t)tmp_buff)|sys_dma_high_addr);
					pr_debug("tmp_buff_after(%p)\n", tmp_buff);
					prefetch(tmp_buff);
				}
#endif /* USE_APP_PREFETCH */
				//printf("packet:\n"); mem_disp(buff, len);
				buff2 = (char *)(((uintptr_t)(buff))|sys_dma_high_addr);
				pr_debug("buff2(%p)\n", buff2);
				buff2 += PKT_EFEC_OFFS;
				swap_l2(buff2);
				swap_l3(buff2);
			}
#endif /* PKT_ECHO_SUPPORT */

			pp2_ppio_outq_desc_reset(&descs[i]);
			pp2_ppio_outq_desc_set_phys_addr(&descs[i], pa);
			pp2_ppio_outq_desc_set_pkt_offset(&descs[i], PKT_EFEC_OFFS);
			pp2_ppio_outq_desc_set_pkt_len(&descs[i], len);
#ifdef HW_BUFF_RECYLCE
			pp2_ppio_outq_desc_set_cookie(&descs[i], (uintptr_t)(buff-PKT_EFEC_OFFS)); /* TODO : Update this for HW_BUFF_RECY*/
			pp2_ppio_outq_desc_set_pool(&descs[i], garg->pool);
#else
			shadow_q->ents[shadow_q->write_ind].buff_inf = (0 << 15) | (0 << 11);
			shadow_q->ents[shadow_q->write_ind].buff_ptr.cookie = lower_32_bits((uintptr_t)buff);
			pr_debug("buff_ptr.cookie(0x%lx)\n", (u64)shadow_q->ents[shadow_q->write_ind].buff_ptr.cookie);
			shadow_q->ents[shadow_q->write_ind].buff_ptr.addr = pa;
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
			struct pp2_buff_inf *binf;
			binf = &(shadow_q->ents[shadow_q->read_ind].buff_ptr);
			if (unlikely(!binf->cookie || !binf->addr)) {
				pr_err("[%s] shadow memory cookie(%lux) addr(%lux)!\n", __FUNCTION__,
					(u64)binf->cookie, (u64)binf->addr);
				sleep(1);
				return -1;
			}
			pp2_bpool_put_buff(garg->hif, garg->pool, binf);
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
	bpool_params.max_num_buffs = MAX_NUM_BUFFS;
	bpool_params.buff_len = BUF_LEN;
	if ((err = pp2_bpool_init(&bpool_params, &garg->pool)) != 0)
		return err;
	if (!garg->pool) {
		pr_err("BPool init failed!\n");
		return -EIO;
	}
	for (i = 0; i < NUM_BUFFS; i++) {
		struct pp2_buff_inf buff;
		void * buff_virt_addr;
		buff_virt_addr = mv_sys_dma_mem_alloc(BUF_LEN, 4);
		if (!buff_virt_addr) {
			pr_err("failed to allocate mem (%d)!\n", i);
			return -1;
		}
		if (i == 0) {
			sys_dma_high_addr = ((u64)buff_virt_addr) & (~((1ULL<<32) - 1));
			pr_err("sys_dma_high_addr (0x%lx)\n", sys_dma_high_addr);
		}
		if ((upper_32_bits((u64)buff_virt_addr)) != (sys_dma_high_addr >> 32))
			pr_err("buff_virt_addr(%p)  upper out of range\n", buff_virt_addr);
		/* TOO: Currently implicitly assumed that sys_dma (dma_coherent)  memory is 32-bit addr. */
		buff.addr = (u32)mv_sys_dma_mem_virt2phys(buff_virt_addr);
		buff.cookie = lower_32_bits((u64)buff_virt_addr); /* cookie contains lower_32_bits of the va */
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
			/* TODO: complete .... */
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
