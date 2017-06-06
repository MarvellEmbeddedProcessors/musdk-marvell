/******************************************************************************
 *	Copyright (C) 2017 Marvell International Ltd.
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

#include "mv_std.h"
#include "lib/lib_misc.h"
#include "env/mv_sys_dma.h"

#include "mvapp.h"
#include "mv_neta.h"
#include "mv_neta_bpool.h"
#include "neta_utils.h"

u64 neta_sys_dma_high_addr;

static u16 used_bpools = MVAPPS_NETA_BPOOLS_RSRV;

static u64 buf_alloc_cnt;
static u64 buf_free_cnt;
static u64 hw_rxq_buf_free_cnt;
static u64 hw_bm_buf_free_cnt;
static u64 hw_buf_free_cnt;
static u64 tx_shadow_q_buf_free_cnt[MVAPPS_NETA_MAX_NUM_CORES];

void app_show_queue_stat(struct port_desc *port_desc, u8 tc, u8 q_start, int num_qs, int reset)
{
	/* TBD: show port queue statistics */
}

void app_show_port_stat(struct port_desc *port_desc, int reset)
{
	/* TBD: show port statistics */
}

static int queue_stat_cmd_cb(void *arg, int argc, char *argv[])
{
	/* TBD: show port queue statistics */

	return 0;
}

static int port_stat_cmd_cb(void *arg, int argc, char *argv[])
{
	/* TBD: show port statistics */

	return 0;
}

int app_register_cli_common_cmds(struct port_desc *port_desc)
{
	/* TBD: NETA CLI commands */

	return 0;
}

static int find_free_bpool(void)
{
	int i;

	for (i = 0; i < MVAPPS_NETA_TOTAL_NUM_BPOOLS; i++) {
		if (!((1 << i) & used_bpools)) {
			used_bpools |= (1 << i);
			break;
		}
	}
	if (i == MVAPPS_NETA_TOTAL_NUM_BPOOLS) {
		pr_err("no free BPool found!\n");
		return -ENOSPC;
	}
	return i;
}

int app_build_port_bpools(struct bpool_desc **ppools, int num_pools, struct bpool_inf infs[])
{
	struct neta_bpool_params	bpool_params;
	int				j, k, err, pool_id;
	char				name[15];
	struct bpool_desc		*pools = NULL;
	struct neta_buff_inf		*buffs_inf = NULL;

	if (num_pools > MVAPPS_NETA_MAX_NUM_BPOOLS) {
		pr_err("only %d pools allowed!\n", MVAPPS_NETA_MAX_NUM_BPOOLS);
		return -EINVAL;
	}

	neta_bm_init();

	pools = (struct bpool_desc *)malloc(sizeof(struct bpool_desc) * num_pools);
	if (!pools) {
		pr_err("no mem for bpool_desc array!\n");
		return -ENOMEM;
	}

	*ppools = pools;

	for (j = 0; j < num_pools; j++) {
		pool_id = find_free_bpool();
		if (pool_id < 0) {
			pr_err("free bpool not found!\n");
			return pool_id;
		}
		memset(name, 0, sizeof(name));
		snprintf(name, sizeof(name), "pool-%d", pool_id);
		memset(&bpool_params, 0, sizeof(bpool_params));
		bpool_params.match = name;
		bpool_params.buff_len = infs[j].buff_size;
		bpool_params.pool_capacity = infs[j].num_buffs;

		pr_info("%s: buff_size %d, num_buffs %d\n", name, infs[j].buff_size, infs[j].num_buffs);
		err = neta_bpool_init(&bpool_params, &pools[j].pool);
		if (err)
			return err;

		if (!pools[j].pool) {
			pr_err("pool id%d init failed!\n", pool_id);
			return -EIO;
		}

		pools[j].buffs_inf =
			(struct neta_buff_inf *)malloc(infs[j].num_buffs * sizeof(struct neta_buff_inf));

		if (!pools[j].buffs_inf) {
			pr_err("no mem for bpools-inf array!\n");
			return -ENOMEM;
		}

		buffs_inf = pools[j].buffs_inf;
		pools[j].num_buffs = infs[j].num_buffs;

		for (k = 0; k < infs[j].num_buffs; k++) {
			void *buff_virt_addr;

			buff_virt_addr = mv_sys_dma_mem_alloc(infs[j].buff_size, 4);
			if (!buff_virt_addr) {
				pr_err("failed to allocate mem (%d)!\n", k);
				return -1;
			}
			if (k == 0) {
				neta_sys_dma_high_addr = ((u64)buff_virt_addr) & (~((1ULL << 32) - 1));
				pr_debug("neta_sys_dma_high_addr (0x%lx)\n", neta_sys_dma_high_addr);
			}
			if ((upper_32_bits((u64)buff_virt_addr)) != (neta_sys_dma_high_addr >> 32)) {
				pr_err("buff_virt_addr(%p)  upper out of range; skipping this buff\n",
				       buff_virt_addr);
				continue;
			}
			buffs_inf[k].addr = (bpool_dma_addr_t)mv_sys_dma_mem_virt2phys(buff_virt_addr);
			/* cookie contains lower_32_bits of the va */
			buffs_inf[k].cookie = lower_32_bits((u64)buff_virt_addr);
			*(u32 *)buff_virt_addr = buffs_inf[k].cookie;
		}

		err = neta_bpool_put_buffs(pools[j].pool, buffs_inf, &infs[j].num_buffs);
		if (err)
			return err;
		buf_alloc_cnt += infs[j].num_buffs;
	}
	return 0;
}

int app_find_port_info(struct port_desc *port_desc)
{
	/* TBD: get port info */
	return 0;
}

int app_port_init(struct port_desc *port, int num_pools, struct bpool_desc *pools, u16 mtu)
{
	struct neta_ppio_params		*port_params = &port->port_params;
	char				name[MVAPPS_PPIO_NAME_MAX];
	int				i, j, err = 0;

	memset(name, 0, sizeof(name));
	snprintf(name, sizeof(name), "neta-%d", port->ppio_id);
	pr_debug("found port: %s\n", name);
	port_params->match = name;
	port_params->inqs_params.num_tcs = port->num_tcs;

	for (i = 0; i < port->num_tcs; i++) {
		port_params->inqs_params.tcs_params[i].pkt_offset = MVAPPS_NETA_PKT_OFFS >> 2;
		port_params->inqs_params.tcs_params[i].size = port->inq_size;

		for (j = 0; j < num_pools; j++)
			port_params->inqs_params.pools[j] = pools[j].pool;
	}
	port_params->outqs_params.num_outqs = port->num_outqs;
	for (i = 0; i < port->num_outqs; i++) {
		port_params->outqs_params.outqs_params[i].size = port->outq_size;
		port_params->outqs_params.outqs_params[i].weight = 1;
	}

	err = neta_ppio_init(port_params, &port->ppio);
	if (err) {
		pr_err("PP-IO init failed (error: %d)!\n", err);
		return err;
	}

	if (!port->ppio) {
		pr_err("PP-IO init failed!\n");
		return -EIO;
	}
	/* TBD: Change port MTU if needed */

	err = neta_ppio_enable(port->ppio);
	port->initialized = 1;

	return err;
}

void app_port_local_init(int id, int lcl_id, struct lcl_port_desc *lcl_port, struct port_desc *port)
{
	int i;

	lcl_port->lcl_id	= lcl_id;
	lcl_port->port_id	= port->ppio_id;
	lcl_port->ppio		= port->ppio;

	lcl_port->num_shadow_qs = port->num_outqs;
	lcl_port->shadow_q_size	= port->outq_size;
	lcl_port->shadow_qs = (struct tx_shadow_q *)malloc(port->num_outqs * sizeof(struct tx_shadow_q));

	for (i = 0; i < lcl_port->num_shadow_qs; i++) {
		lcl_port->shadow_qs[i].read_ind = 0;
		lcl_port->shadow_qs[i].write_ind = 0;

		lcl_port->shadow_qs[i].ents =
			(struct tx_shadow_q_entry *)malloc(port->outq_size * sizeof(struct tx_shadow_q_entry));
	}
}

void app_port_local_deinit(struct lcl_port_desc *lcl_port)
{
	int i, cnt = 0;

	for (i = 0; i < lcl_port->num_shadow_qs; i++) {
		struct tx_shadow_q *shadow_q = &lcl_port->shadow_qs[i];

		if (shadow_q->read_ind > shadow_q->write_ind) {
			cnt = lcl_port->shadow_q_size - shadow_q->read_ind + shadow_q->write_ind;
			tx_shadow_q_buf_free_cnt[lcl_port->lcl_id] += cnt;
		} else {
			cnt = shadow_q->write_ind - shadow_q->read_ind;
			tx_shadow_q_buf_free_cnt[lcl_port->lcl_id] += cnt;
		}
		pr_debug("Release %d buffers from shadow_q-%d port:%d:%d\n",
			 cnt, i, lcl_port->pp_id, lcl_port->ppio_id);
		shadow_q->read_ind = 0;
		shadow_q->write_ind = 0;
		free(shadow_q->ents);
	}

	free(lcl_port->shadow_qs);
}

static void free_rx_queues(struct neta_ppio *port, u16 num_tcs, u16 num_inqs[])
{
	struct neta_ppio_desc	descs[MVAPPS_MAX_BURST_SIZE];
	u8			tc = 0, qid = 0;
	u16			num;

	for (tc = 0; tc < num_tcs; tc++) {
		for (qid = 0; qid < num_inqs[tc]; qid++) {
			num = MVAPPS_MAX_BURST_SIZE;
			while (num) {
				neta_ppio_recv(port, qid, descs, &num);
				hw_rxq_buf_free_cnt += num;
			}
		}
	}
}

void app_disable_all_ports(struct port_desc *ports, int num_ports)
{
	int i;

	for (i = 0;  i < num_ports; i++) {
		if (ports[i].ppio) {
			neta_ppio_disable(ports[i].ppio);
			free_rx_queues(ports[i].ppio, ports[i].num_tcs, ports[i].num_inqs);
		}
	}
}

void app_deinit_all_ports(struct port_desc *ports, int num_ports)
{
	int i;

	for (i = 0;  i < num_ports; i++) {
		if (ports[i].ppio)
			neta_ppio_deinit(ports[i].ppio);
	}

	/* Calculate number of buffers released from PP2 */
	for (i = 0; i < MVAPPS_NETA_MAX_NUM_CORES; i++)
		hw_buf_free_cnt += tx_shadow_q_buf_free_cnt[i];
	hw_buf_free_cnt += hw_bm_buf_free_cnt + hw_rxq_buf_free_cnt;

	if (buf_free_cnt != buf_alloc_cnt) {
		pr_err("Not all buffers were released: allocated: %lu, freed: %lu\n",
		       buf_alloc_cnt, buf_free_cnt);

		if (buf_free_cnt != hw_buf_free_cnt) {
			pr_err("pp2 freed: %lu bm free: %lu, rxq free: %lu, tx free: %lu !\n",
			       hw_buf_free_cnt, hw_bm_buf_free_cnt, hw_rxq_buf_free_cnt,
				(hw_buf_free_cnt - hw_bm_buf_free_cnt - hw_rxq_buf_free_cnt));
		}
	}

	pr_debug("allocated: %lu, app freed: %lu, pp2 freed: %lu bm free: %lu, rxq free: %lu, tx free: %lu !\n",
		 buf_alloc_cnt, buf_free_cnt, hw_buf_free_cnt, hw_bm_buf_free_cnt, hw_rxq_buf_free_cnt,
		(hw_buf_free_cnt - hw_bm_buf_free_cnt - hw_rxq_buf_free_cnt));
}

static void flush_pool(struct neta_bpool *bpool)
{
	u32 i, buf_num, cnt = 0, err = 0;

	neta_bpool_get_num_buffs(bpool, &buf_num);
	for (i = 0; i < buf_num; i++) {
		struct neta_buff_inf buff;

		err = 0;
		while (neta_bpool_get_buff(bpool, &buff)) {
			err++;
			if (err == 10000) {
				buff.cookie = 0;
				break;
			}
		}
		pr_err("pool %d: pa = 0x%lx, va = 0x%x\n", bpool->id, buff.addr, buff.cookie);
		if (err) {
			if (err == 10000) {
				pr_err("flush_pool: pool_id=%d: Got NULL buf (%d of %d)\n",
				       bpool->id, i, buf_num);
				continue;
			}
			pr_warn("flush_pool: pool_id=%d: Got buf (%d of %d) after %d retries\n",
				bpool->id, i, buf_num, err);
		}
		cnt++;
	}
	hw_bm_buf_free_cnt += cnt;
	neta_bpool_deinit(bpool);
}

static void free_pool_buffers(struct neta_buff_inf *buffs, int num)
{
	int i;

	for (i = 0; i < num; i++) {
		void *buff_virt_addr = (char *)(((uintptr_t)(buffs[i].cookie)) | neta_sys_dma_high_addr);

		mv_sys_dma_mem_free(buff_virt_addr);
		buf_free_cnt++;
	}
}

void app_free_all_pools(struct bpool_desc *pool, int num_pools)
{
	int j;

	if (pool) {
		for (j = 0; j < num_pools; j++) {
			if (pool[j].pool) {
				flush_pool(pool[j].pool);
				free_pool_buffers(pool[j].buffs_inf, pool[j].num_buffs);
				free(pool[j].buffs_inf);
			}
		}
		free(pool);
	}
}
