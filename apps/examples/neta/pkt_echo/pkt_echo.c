/*******************************************************************************
 * Copyright (C) Marvell International Ltd. and its affiliates
 *
 * This software file (the "File") is owned and distributed by Marvell
 * International Ltd. and/or its affiliates ("Marvell") under the following
 * alternative licensing terms.  Once you have made an election to distribute the
 * File under one of the following license alternatives, please (i) delete this
 * introductory statement regarding license alternatives, (ii) delete the three
 * license alternatives that you have not elected to use and (iii) preserve the
 * Marvell copyright notice above.
 *
 ********************************************************************************
 * Marvell Commercial License Option
 *
 * If you received this File from Marvell and you have entered into a commercial
 * license agreement (a "Commercial License") with Marvell, the File is licensed
 * to you under the terms of the applicable Commercial License.
 *
 ********************************************************************************
 * Marvell GPL License Option
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File in accordance with the terms and conditions of the General
 * Public License Version 2, June 1991 (the "GPL License"), a copy of which is
 * available along with the File in the license.txt file or by writing to the Free
 * Software Foundation, Inc., or on the worldwide web at http://www.gnu.org/licenses/gpl.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY
 * DISCLAIMED.  The GPL License provides additional details about this warranty
 * disclaimer.
 *
 ********************************************************************************
 * Marvell GNU General Public License FreeRTOS Exception
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File in accordance with the terms and conditions of the Lesser
 * General Public License Version 2.1 plus the following FreeRTOS exception.
 * An independent module is a module which is not derived from or based on
 * FreeRTOS.
 * Clause 1:
 * Linking FreeRTOS statically or dynamically with other modules is making a
 * combined work based on FreeRTOS. Thus, the terms and conditions of the GNU
 * General Public License cover the whole combination.
 * As a special exception, the copyright holder of FreeRTOS gives you permission
 * to link FreeRTOS with independent modules that communicate with FreeRTOS solely
 * through the FreeRTOS API interface, regardless of the license terms of these
 * independent modules, and to copy and distribute the resulting combined work
 * under terms of your choice, provided that:
 * 1. Every copy of the combined work is accompanied by a written statement that
 * details to the recipient the version of FreeRTOS used and an offer by yourself
 * to provide the FreeRTOS source code (including any modifications you may have
 * made) should the recipient request it.
 * 2. The combined work is not itself an RTOS, scheduler, kernel or related
 * product.
 * 3. The independent modules add significant and primary functionality to
 * FreeRTOS and do not merely extend the existing functionality already present in
 * FreeRTOS.
 * Clause 2:
 * FreeRTOS may not be used for any competitive or comparative purpose, including
 * the publication of any form of run time or compile time metric, without the
 * express permission of Real Time Engineers Ltd. (this is the norm within the
 * industry and is intended to ensure information accuracy).
 *
 ********************************************************************************
 * Marvell BSD License Option
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File under the following licensing terms.
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *	* Redistributions of source code must retain the above copyright notice,
 *	  this list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the distribution.
 *
 *	* Neither the name of Marvell nor the names of its contributors may be
 *	  used to endorse or promote products derived from this software without
 *	  specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************/

#define _GNU_SOURCE         /* See feature_test_macros(7) */
#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <sched.h>

#ifndef DEBUG
#define DEBUG
#endif

#include "mv_std.h"
#include "lib/lib_misc.h"
#include "env/mv_sys_dma.h"
#include "env/io.h"

#include "mvapp.h"
#include "mv_neta.h"
#include "mv_neta_ppio.h"

#include "../neta_utils.h"

static const char buf_release_str[] = "SW buffer release";
static const char tx_retry_str[] = "Tx Retry disabled";

#define PKT_ECHO_APP_TX_RETRY_WAIT		1
#define PKT_ECHO_APP_DEF_Q_SIZE			512
#define PKT_ECHO_APP_RX_Q_SIZE			(PKT_ECHO_APP_DEF_Q_SIZE)
#define PKT_ECHO_APP_TX_Q_SIZE			(2 * PKT_ECHO_APP_DEF_Q_SIZE)

#define PKT_ECHO_APP_MAX_BURST_SIZE		((PKT_ECHO_APP_RX_Q_SIZE) >> 1)
#define PKT_ECHO_APP_DFLT_BURST_SIZE		128

#define PKT_ECHO_APP_DMA_MEM_SIZE		(40 * 1024 * 1024)

#define PKT_ECHO_APP_FIRST_INQ			0
#define PKT_ECHO_APP_MAX_NUM_TCS_PER_PORT	1
#define PKT_ECHO_APP_MAX_NUM_QS_PER_CORE	PKT_ECHO_APP_MAX_NUM_TCS_PER_PORT

#define PKT_ECHO_APP_PREFETCH_SHIFT		4

#define PKT_ECHO_APP_BPOOLS_INF		{ {256, PKT_ECHO_APP_TX_Q_SIZE}, {1600, PKT_ECHO_APP_TX_Q_SIZE} }
#define PKT_ECHO_APP_BPOOLS_JUMBO_INF	{ {384, PKT_ECHO_APP_TX_Q_SIZE}, {4096, PKT_ECHO_APP_TX_Q_SIZE} }

struct glob_arg {
	struct glb_common_args		cmn_args;  /* Keep first */
	u16				rxq_size;
	int				loopback;
	int				maintain_stats;
	struct port_desc		ports_desc[MVAPPS_NETA_MAX_NUM_PORTS];

	pthread_mutex_t			trd_lock;
};

struct local_arg {
	struct local_common_args	cmn_args;  /* Keep first */
	struct lcl_port_desc		*ports_desc;
};

static struct glob_arg garg = {};


static inline u16 free_buffers(struct lcl_port_desc	*rx_port,
			       struct lcl_port_desc	*tx_port,
			       u16			 start_idx,
			       u32			 num,
			       u8			 tc)
{
	u16			idx = start_idx;
	u16			free_cnt = 0;
	struct neta_buff_inf	*binf;
	struct tx_shadow_q	*shadow_q;
	u16 bufs_num, i;

	shadow_q = &tx_port->shadow_qs[tc];
	bufs_num = 1;

	for (i = 0; i < num; i++) {
		binf = &shadow_q->ents[idx++].buff_ptr;
		neta_ppio_inq_put_buffs(rx_port->ppio, tc, binf, &bufs_num);
		if (idx == tx_port->shadow_q_size)
			idx = 0;
	}

	free_cnt += num;
	INC_FREE_COUNT(rx_port, free_cnt);

	return idx;
}

static inline void free_sent_buffers(struct lcl_port_desc	*rx_port,
				     struct lcl_port_desc	*tx_port,
				     u8				 tc)
{
	u16			idx;
	u16 tx_num = 0;

	neta_ppio_get_num_outq_done(tx_port->ppio, tc, &tx_num);

	if (unlikely(!tx_num))
		return;

	idx = free_buffers(rx_port, tx_port, tx_port->shadow_qs[tc].read_ind, tx_num, tc);
	tx_port->shadow_qs[tc].read_ind = idx;
}

#ifdef DEBUG_NETA
void neta_packet_dump(char *buff, u16 len)
{
	int i;

	printf("packet buffer at %p length %d:\n", buff, len);
	for (i = 0; i < len; i++) {
		printf("%0x8x ", buff[i]);
		if ((i % 32) == 0)
			printf("\n");
	}
}
#endif

static inline int loop_sw_recycle(struct local_arg	*larg,
				  u8			 rx_ppio_id,
				  u8			 tx_ppio_id,
				  u8			 qid,
				  u16			 num)
{
	struct neta_ppio_desc	 descs[PKT_ECHO_APP_MAX_BURST_SIZE];
	u16			 i, tx_num = 0;
	struct tx_shadow_q	*shadow_q;
	int			 shadow_q_size;
	int			 prefetch_shift = larg->cmn_args.prefetch_shift;

	shadow_q = &larg->ports_desc[tx_ppio_id].shadow_qs[qid];
	shadow_q_size = larg->ports_desc[tx_ppio_id].shadow_q_size;

	neta_ppio_recv(larg->ports_desc[rx_ppio_id].ppio, qid, descs, &num);
	INC_RX_COUNT(&larg->ports_desc[rx_ppio_id], num);

	for (i = 0; i < num; i++) {
		char		*buff = (char *)(uintptr_t)neta_ppio_inq_desc_get_cookie(&descs[i]);
		dma_addr_t	 pa = neta_ppio_inq_desc_get_phys_addr(&descs[i]);
		u16 len = neta_ppio_inq_desc_get_pkt_len(&descs[i]);

		if (!buff  || !pa) {
			pr_err("Receive empty descriptor on port %d\n", rx_ppio_id);
			continue;
		}
		INC_RX_BYTES_COUNT(&larg->ports_desc[rx_ppio_id], len);

		if (likely(larg->cmn_args.echo)) {
			char *tmp_buff;

			if ((num - i) > prefetch_shift) {
				tmp_buff = (char *)(uintptr_t)neta_ppio_inq_desc_get_cookie(&descs[i + prefetch_shift]);
				tmp_buff = (char *)(((uintptr_t)tmp_buff) | neta_sys_dma_high_addr);
				tmp_buff += MVAPPS_NETA_PKT_EFEC_OFFS;
				prefetch(tmp_buff);
			}

			tmp_buff = (char *)(((uintptr_t)(buff)) | neta_sys_dma_high_addr);
			tmp_buff += MVAPPS_NETA_PKT_EFEC_OFFS;
			pr_debug("buffer pointer is %p, tmp_buff for swap (%p)\n", buff, tmp_buff);
			swap_l2(tmp_buff);
			swap_l3(tmp_buff);
		}
		neta_ppio_outq_desc_reset(&descs[i]);
		neta_ppio_outq_desc_set_phys_addr(&descs[i], pa);
		neta_ppio_outq_desc_set_pkt_offset(&descs[i], MVAPPS_NETA_PKT_EFEC_OFFS);
		neta_ppio_outq_desc_set_pkt_len(&descs[i], len);
		INC_TX_BYTES_COUNT(&larg->ports_desc[tx_ppio_id], len);

		shadow_q->ents[shadow_q->write_ind].buff_ptr.cookie = (uintptr_t)buff;
		shadow_q->ents[shadow_q->write_ind].buff_ptr.addr = pa;
		pr_debug("buff_ptr.cookie(0x%lx)\n", (u64)shadow_q->ents[shadow_q->write_ind].buff_ptr.cookie);
		shadow_q->write_ind++;
		if (shadow_q->write_ind == shadow_q_size)
			shadow_q->write_ind = 0;
	}

	if (num) {
		tx_num = num;
		neta_ppio_send(larg->ports_desc[tx_ppio_id].ppio, qid, descs, &tx_num);
		if (num > tx_num) {
			u16 not_sent = num - tx_num;

			pr_warn("Cannot sent %d packets from %d.\n", not_sent, num);
			/* Free not sent buffers */
			shadow_q->write_ind = (shadow_q->write_ind < not_sent) ?
						(shadow_q_size - not_sent + shadow_q->write_ind) :
						shadow_q->write_ind - not_sent;
			shadow_q->read_ind = free_buffers(&larg->ports_desc[rx_ppio_id],
							  &larg->ports_desc[tx_ppio_id],
							  shadow_q->write_ind, not_sent, qid);

			INC_TX_DROP_COUNT(&larg->ports_desc[tx_ppio_id], not_sent);
		}
		INC_TX_COUNT(&larg->ports_desc[tx_ppio_id], tx_num);
	}

	free_sent_buffers(&larg->ports_desc[rx_ppio_id], &larg->ports_desc[tx_ppio_id], qid);
	return 0;
}

static int loop_1p(struct local_arg *larg, int *running)
{
	int			err;
	u16			num;
	u8			qid = 0;

	if (!larg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	num = larg->cmn_args.burst;

	while (*running) {
		/* Find next queue to consume */
		do {
			qid++;
			if (qid == MVAPPS_NETA_MAX_NUM_QS_PER_TC)
				qid = 0;
		} while (!(larg->cmn_args.qs_map & (1 << qid)));

		err = loop_sw_recycle(larg, 0, 0, qid, num);
		if (err != 0)
			return err;
	}

	return 0;
}

static int loop_2ps(struct local_arg *larg, int *running)
{
	int			 err;
	u16			 num;
	u8			 qid = 0;

	if (!larg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	num = larg->cmn_args.burst;
	while (*running) {

		/* Find next queue to consume */
		do {
			qid++;
			if (qid == MVAPPS_NETA_MAX_NUM_QS_PER_TC)
				qid = 0;
		} while (!(larg->cmn_args.qs_map & (1 << qid)));

		err  = loop_sw_recycle(larg, 0, 1, qid, num);
		err |= loop_sw_recycle(larg, 1, 0, qid, num);
		if (err != 0)
			return err;
	}

	return 0;
}

static int loop_2ps_2cs(struct local_arg *larg, int *running)
{
	int			 err;
	u16			 num;
	u8			 qid = 0;
	u8			 src_port, dst_port;

	if (!larg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	num = larg->cmn_args.burst;
	src_port = larg->cmn_args.id;
	dst_port = 1 - src_port;
	pr_info("Run loop 2 ports: port %d -> port %d\n", src_port, dst_port);

	while (*running) {

		/* Find next queue to consume */
		do {
			qid++;
			if (qid == MVAPPS_NETA_MAX_NUM_QS_PER_TC)
				qid = 0;
		} while (!(larg->cmn_args.qs_map & (1 << qid)));

		err  = loop_sw_recycle(larg, src_port, dst_port, qid, num);
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

	if (larg->cmn_args.num_ports == 1)
		return loop_1p(larg, running);

	if (garg.cmn_args.cpus == 1)
		return loop_2ps(larg, running);
	else
		return loop_2ps_2cs(larg, running);
}

static int init_all_modules(void)
{
	int			err;

	pr_info("Global initializations ...\n");

	err = mv_sys_dma_mem_init(PKT_ECHO_APP_DMA_MEM_SIZE);
	if (err)
		return err;

	err = neta_init();
	if (err)
		return err;

	pr_info("done\n");
	return 0;
}

static int port_inq_fill(struct port_desc *port, u16 mtu, u16 qid)
{
	struct neta_buff_inf buffs_inf[PKT_ECHO_APP_RX_Q_SIZE];
	u16 buff_size;
	u16 i, num_of_buffs;

	buff_size = MVAPPS_MTU_TO_MRU(port->port_params.inqs_params.mtu) + MVAPPS_NETA_PKT_OFFS;
	/* do it only once to store high 32 bits of memory address */
	if (!neta_sys_dma_high_addr) {
		void *tmp_addr;

		tmp_addr = mv_sys_dma_mem_alloc(buff_size, NETA_PPIO_RX_BUF_ALIGN);
		if (!tmp_addr) {
			pr_err("failed to allocate mem for neta_sys_dma_high_addr!\n");
			return -1;
		}
		neta_sys_dma_high_addr = ((u64)tmp_addr) & (~((1ULL << 32) - 1));
		mv_sys_dma_mem_free(tmp_addr);
	}
	for (i = 0; i < port->inq_size; i++) {
		void *buff_virt_addr;

		buff_virt_addr = mv_sys_dma_mem_alloc(buff_size, NETA_PPIO_RX_BUF_ALIGN);
		if (!buff_virt_addr) {
			pr_err("port %d: queue %d: failed to allocate buffer memory (%d)!\n",
				port->ppio_id, qid, i);
			return -1;
		}
		buffs_inf[i].addr = (neta_dma_addr_t)mv_sys_dma_mem_virt2phys(buff_virt_addr);
		/* cookie contains lower_32_bits of the va */
		buffs_inf[i].cookie = lower_32_bits((u64)buff_virt_addr);
	}
	num_of_buffs = i;

	neta_ppio_inq_put_buffs(port->ppio, qid, buffs_inf, &num_of_buffs);
	pr_debug("port %d: queue %d: %d buffers added.\n", port->ppio_id, qid, num_of_buffs);

	return 0;
}

static int port_inqs_fill(struct port_desc *port, u16 mtu)
{
	int err = 0;
	u16 tc, qid;

	for (tc = 0; tc < port->num_tcs; tc++) {
		for (qid = 0; qid < port->num_inqs[tc]; qid++)
			err |= port_inq_fill(port, mtu, qid);
	}

	return err;
}

static int init_local_modules(struct glob_arg *garg)
{
	int				err, port_index;
	int				i;

	pr_info("Local initializations ...\n");

	for (port_index = 0; port_index < garg->cmn_args.num_ports; port_index++) {
		struct port_desc *port = &garg->ports_desc[port_index];

		err = app_find_port_info(port);
		if (err)
			return err;

		port->num_tcs = PKT_ECHO_APP_MAX_NUM_TCS_PER_PORT;
		for (i = 0; i < port->num_tcs; i++)
			port->num_inqs[i] = PKT_ECHO_APP_MAX_NUM_QS_PER_CORE;

		port->inq_size	= garg->rxq_size;
		port->num_outqs	= PKT_ECHO_APP_MAX_NUM_TCS_PER_PORT;
		port->outq_size	= PKT_ECHO_APP_TX_Q_SIZE;
		port->first_inq	= PKT_ECHO_APP_FIRST_INQ;

		err = app_port_init(port, garg->cmn_args.mtu, MVAPPS_NETA_PKT_OFFS);
		if (err) {
			pr_err("Failed to initialize port %d (pp_id: %d)\n", port_index,
			       port->ppio_id);
			return err;
		}
		/* fill RX queues with buffers */
		port_inqs_fill(port, garg->cmn_args.mtu);

		/* enable port */
		err = app_port_enable(port);
	}

	pr_info("done\n");
	return 0;
}

static void destroy_local_modules(struct glob_arg *garg)
{
	app_disable_all_ports(garg->ports_desc, garg->cmn_args.num_ports);
	app_deinit_all_ports(garg->ports_desc, garg->cmn_args.num_ports);
}

static void destroy_all_modules(void)
{
	neta_deinit();
	mv_sys_dma_mem_destroy();
}

static int neta_stat_cmd_cb(void *arg, int argc, char *argv[])
{

	int i, j, reset = 0;
	struct glb_common_args *g_cmn_args = (struct glb_common_args *)arg;
	struct neta_counters *cntrs;

	if (argc > 1)
		reset = 1;
	printf("reset stats: %d\n", reset);
	for (j = 0; j < g_cmn_args->num_ports; j++) {
		printf("Port%d stats:\n", j);
		for (i = 0; i < g_cmn_args->cpus; i++) {
			cntrs = &(g_cmn_args->largs[i]->ports_desc[j].cntrs);

			printf("cpu%d: rx_bufs=%u (%lu bytes), tx_bufs=%u (%lu bytes), free_bufs=%u, tx_drops=%u\n",
			       i, cntrs->rx_buf_cnt, cntrs->rx_bytes_cnt, cntrs->tx_buf_cnt,
			       cntrs->tx_bytes_cnt, cntrs->free_buf_cnt, cntrs->tx_buf_drop);

			if (reset) {
				cntrs->rx_buf_cnt = 0;
				cntrs->tx_buf_cnt = 0;
				cntrs->free_buf_cnt = 0;
				cntrs->rx_bytes_cnt = 0;
				cntrs->tx_bytes_cnt = 0;
				cntrs->tx_buf_drop = 0;
			}
		}
	}
	return 0;
}

static int register_cli_cmds(struct glob_arg *garg)
{
	struct cli_cmd_params	 cmd_params;

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "prefetch";
	cmd_params.desc		= "Prefetch ahead shift (number of buffers)\n";
	cmd_params.format	= "<shift>";
	cmd_params.cmd_arg	= garg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))apps_prefetch_cmd_cb;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "stat";
	cmd_params.desc		= "Show app statistics\n";
	cmd_params.format	= "<reset>";
	cmd_params.cmd_arg	= &garg->cmn_args;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))neta_stat_cmd_cb;
	mvapp_register_cli_cmd(&cmd_params);

	app_register_cli_common_cmds(&garg->cmn_args);
	register_cli_ftr_cmds(&garg->cmn_args);

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

	return 0;
}

static void deinit_global(void *arg)
{
	struct glob_arg *garg = (struct glob_arg *)arg;

	if (!garg)
		return;
	if (garg->cmn_args.cli)
		unregister_cli_cmds(garg);
	destroy_local_modules(garg);
	destroy_all_modules();
}

static int init_local(void *arg, int id, void **_larg)
{
	struct glob_arg		*garg = (struct glob_arg *)arg;
	struct local_arg	*larg;
	int			 i;

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
	larg->cmn_args.burst		= garg->cmn_args.burst;
	larg->cmn_args.echo		= garg->cmn_args.echo;
	larg->cmn_args.prefetch_shift	= garg->cmn_args.prefetch_shift;
	larg->cmn_args.num_ports	= garg->cmn_args.num_ports;
	larg->ports_desc = (struct lcl_port_desc *)malloc(larg->cmn_args.num_ports * sizeof(struct lcl_port_desc));
	if (!larg->ports_desc) {
		pr_err("no mem for local-port-desc obj!\n");
		return -ENOMEM;
	}
	garg->cmn_args.plat = garg->ports_desc;
	memset(larg->ports_desc, 0, larg->cmn_args.num_ports * sizeof(struct lcl_port_desc));
	for (i = 0; i < larg->cmn_args.num_ports; i++)
		app_port_local_init(i, larg->cmn_args.id, &larg->ports_desc[i], &garg->ports_desc[i]);

	larg->cmn_args.garg              = garg;

	larg->cmn_args.qs_map = garg->cmn_args.qs_map;
	pr_info("thread %d (cpu %d) mapped to Qs %llx using %s\n",
		 larg->cmn_args.id, sched_getcpu(), (unsigned long long)larg->cmn_args.qs_map, "neta");

	garg->cmn_args.largs[sched_getcpu()] = larg;

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
		for (i = 0; i < larg->cmn_args.num_ports; i++)
			app_port_local_deinit(&larg->ports_desc[i]);
		free(larg->ports_desc);
	}

	free(larg);
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
	       "\t-b <size>                Burst size, num_pkts handled in a batch.(default is %d)\n"
	       "\t--mtu <mtu>              Set MTU (default is %d)\n"
	       "\t-c, --cores <number>     Number of CPUs to use\n"
	       "\t-a, --affinity <number>  Use setaffinity (default is no affinity)\n"
	       "\t-s                       Maintain statistics\n"
	       "\t--rxq <size>             Size of rx_queue (default is %d)\n"
	       "\t--old-tx-desc-release    Use neta_bpool_put_buff(), instead of NEW neta_bpool_put_buffs() API\n"
	       "\t--no-echo                Don't perform 'pkt_echo', N/A w/o define PKT_ECHO_APP_PKT_ECHO_SUPPORT\n"
	       "\t--cli                    Use CLI\n"
	       "\t?, -h, --help            Display help and exit.\n\n"
	       "\n", MVAPPS_NO_PATH(progname), MVAPPS_NO_PATH(progname),
	       MVAPPS_NETA_MAX_NUM_PORTS, PKT_ECHO_APP_MAX_BURST_SIZE, DEFAULT_MTU, PKT_ECHO_APP_RX_Q_SIZE);
}

static int parse_args(struct glob_arg *garg, int argc, char *argv[])
{
	int	i = 1;

	garg->cmn_args.cli = 0;
	garg->cmn_args.cpus = 1;
	garg->cmn_args.affinity = -1;
	garg->cmn_args.burst = PKT_ECHO_APP_DFLT_BURST_SIZE;
	garg->cmn_args.mtu = DEFAULT_MTU;
	garg->rxq_size = PKT_ECHO_APP_RX_Q_SIZE;
	garg->cmn_args.echo = 1;
	garg->cmn_args.qs_map = 0;
	garg->cmn_args.qs_map_shift = 0;
	garg->cmn_args.prefetch_shift = PKT_ECHO_APP_PREFETCH_SHIFT;
	garg->maintain_stats = 0;

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
			     token = strtok(NULL, ","), garg->cmn_args.num_ports++) {
				int tmp;

				snprintf(garg->ports_desc[garg->cmn_args.num_ports].name,
					 sizeof(garg->ports_desc[garg->cmn_args.num_ports].name),
					 "%s", token);
				tmp = sscanf(garg->ports_desc[garg->cmn_args.num_ports].name, "eth%d",
				       &garg->ports_desc[garg->cmn_args.num_ports].ppio_id);
				if ((tmp != 1) ||
				    (garg->ports_desc[garg->cmn_args.num_ports].ppio_id >
				     MVAPPS_NETA_MAX_NUM_PORTS)) {
					pr_err("Invalid interface number %d!\n",
						garg->ports_desc[garg->cmn_args.num_ports].ppio_id);
					return -EINVAL;
				}
			}
			if (garg->cmn_args.num_ports == 0) {
				pr_err("Invalid interface arguments format!\n");
				return -EINVAL;
			} else if (garg->cmn_args.num_ports > MVAPPS_NETA_MAX_NUM_PORTS) {
				pr_err("too many ports specified (%d vs %d)\n",
				       garg->cmn_args.num_ports, MVAPPS_NETA_MAX_NUM_PORTS);
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
		} else if (strcmp(argv[i], "--rxq") == 0) {
			garg->rxq_size = atoi(argv[i + 1]);
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
		} else {
			pr_err("argument (%s) not supported!\n", argv[i]);
			return -EINVAL;
		}
	}

	/* Now, check validity of all inputs */
	if (!garg->cmn_args.num_ports ||
	    !garg->ports_desc[0].name) {
		pr_err("No port defined!\n");
		return -EINVAL;
	}
	if (garg->cmn_args.burst > PKT_ECHO_APP_MAX_BURST_SIZE) {
		pr_err("illegal burst size requested (%d vs %d)!\n",
		       garg->cmn_args.burst, PKT_ECHO_APP_MAX_BURST_SIZE);
		return -EINVAL;
	}
	if (garg->cmn_args.cpus > MVAPPS_NETA_MAX_NUM_CORES) {
		pr_err("illegal num cores requested (%d vs %d)!\n",
		       garg->cmn_args.cpus, MVAPPS_NETA_MAX_NUM_CORES);
		return -EINVAL;
	}
	if (garg->cmn_args.cpus > garg->cmn_args.num_ports) {
		pr_warn("this application doesn't support number of cores (%d) > number of ports (%d)\n",
		       garg->cmn_args.cpus, garg->cmn_args.num_ports);
		garg->cmn_args.cpus = garg->cmn_args.num_ports;
	}
	if ((garg->cmn_args.affinity != -1) &&
	    ((garg->cmn_args.cpus + garg->cmn_args.affinity) > MVAPPS_NETA_MAX_NUM_CORES)) {
		pr_err("illegal num cores or affinity requested (%d,%d vs %d)!\n",
		       garg->cmn_args.cpus, garg->cmn_args.affinity, MVAPPS_NETA_MAX_NUM_CORES);
		return -EINVAL;
	}

	if (garg->cmn_args.qs_map &&
	    (MVAPPS_NETA_MAX_NUM_QS_PER_TC == 1) &&
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

int main(int argc, char *argv[])
{
	struct mvapp_params	mvapp_params;
	u64			cores_mask;
	int			i, err;

	setbuf(stdout, NULL);
	pr_info("NETA pkt-echo is started in %s - %s\n", buf_release_str, tx_retry_str);

	pr_debug("pr_debug is enabled\n");

	err = parse_args(&garg, argc, argv);
	if (err)
		return err;

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
	mvapp_params.deinit_global_cb	= deinit_global;
	mvapp_params.init_local_cb	= init_local;
	mvapp_params.deinit_local_cb	= deinit_local;
	mvapp_params.main_loop_cb	= main_loop;

	return mvapp_go(&mvapp_params);
}
