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

#include "mv_std.h"
#include "lib/lib_misc.h"
#include "env/mv_sys_dma.h"

#include "mvapp.h"
#include "mv_pp2.h"
#include "mv_pp2_bpool.h"
#include "mv_pp2_ppio.h"
#include "utils.h"

static u64 sys_dma_high_addr;
static u16 used_bpools = MVAPPS_PP2_BPOOLS_RSRV;
static u16 used_hifs = MVAPPS_PP2_HIFS_RSRV;


u64 app_get_sys_dma_high_addr(void)
{
	return sys_dma_high_addr;
}

static int find_free_bpool(void)
{
	int i;

	for (i = 0; i < MVAPPS_PP2_TOTAL_NUM_BPOOLS; i++) {
		if (!((uint64_t)(1 << i) & used_bpools)) {
			used_bpools |= (uint64_t)(1 << i);
			break;
		}
	}
	if (i == MVAPPS_PP2_TOTAL_NUM_BPOOLS) {
		pr_err("no free BPool found!\n");
		return -ENOSPC;
	}
	return i;
}

static int find_free_hif(void)
{
	int i;

	for (i = 0; i < MVAPPS_PP2_TOTAL_NUM_HIFS; i++) {
		if (!((uint64_t)(1 << i) & used_hifs)) {
			used_hifs |= (uint64_t)(1 << i);
			break;
		}
	}
	if (i == MVAPPS_PP2_TOTAL_NUM_HIFS) {
		pr_err("no free HIF found!\n");
		return -ENOSPC;
	}

	return i;
}

int app_hif_init(struct pp2_hif **hif)
{
	int hif_id;
	char name[15];
	struct pp2_hif_params hif_params;
	int err;

	hif_id = find_free_hif();
	if (hif_id < 0) {
		pr_err("free HIF not found!\n");
		return hif_id;
	}

	memset(name, 0, sizeof(name));
	snprintf(name, sizeof(name), "hif-%d", hif_id);
	pr_debug("found hif: %s\n", name);
	memset(&hif_params, 0, sizeof(hif_params));
	hif_params.match = name;
	hif_params.out_size = MVAPPS_Q_SIZE;
	err = pp2_hif_init(&hif_params, hif);
	if (err)
		return err;
	if (!hif) {
		pr_err("HIF init failed!\n");
		return -EIO;
	}

	return 0;
}



int app_build_all_bpools(struct pp2_bpool ****ppools, struct pp2_buff_inf ****pbuffs_inf, int num_pools,
			   struct bpool_inf infs[], struct pp2_hif *hif)
{
	struct pp2_bpool_params		bpool_params;
	int				i, j, k, err, pool_id;
	char				name[15];
	struct pp2_bpool 		***pools;
	struct pp2_buff_inf 		***buffs_inf;
	u8  pp2_num_inst = pp2_get_num_inst();

	pools = (struct pp2_bpool ***)malloc(pp2_num_inst * sizeof(struct pp2_bpool **));
	if (!pools) {
		pr_err("no mem for bpools array!\n");
		return -ENOMEM;
	}

	*ppools = pools;

	buffs_inf = (struct pp2_buff_inf ***)malloc(pp2_num_inst * sizeof(struct pp2_buff_inf **));
	if (!buffs_inf) {
		pr_err("no mem for bpools-inf array!\n");
		return -ENOMEM;
	}

	*pbuffs_inf = buffs_inf;

	for (i = 0; i < pp2_num_inst; i++) {

		pr_info("num_pools = %d, buff_size %d, num_buffs %d\n", num_pools, infs[0].buff_size, infs[0].num_buffs);

		/* TODO: temporary W/A until we have map routines of bpools to ppios */
		if (num_pools > PP2_PPIO_TC_MAX_POOLS) {
			pr_err("only %d pools allowed!\n", PP2_PPIO_TC_MAX_POOLS);
			return -EINVAL;
		}

		pools[i] = (struct pp2_bpool **)malloc(num_pools * sizeof(struct pp2_bpool *));
		if (!pools[i]) {
			pr_err("no mem for bpools array!\n");
			return -ENOMEM;
		}

		buffs_inf[i] = (struct pp2_buff_inf **)malloc(num_pools * sizeof(struct pp2_buff_inf *));
		if (!buffs_inf[i]) {
			pr_err("no mem for bpools-inf array!\n");
			return -ENOMEM;
		}

		for (j = 0; j < num_pools; j++) {
			pool_id = find_free_bpool();
			if (pool_id < 0) {
				pr_err("free bpool not found!\n");
				return pool_id;
			}
			memset(name, 0, sizeof(name));
			snprintf(name, sizeof(name), "pool-%d:%d", i, pool_id);
			pr_info("found bpool:  %s\n", name);
			memset(&bpool_params, 0, sizeof(bpool_params));
			bpool_params.match = name;
			bpool_params.buff_len = infs[j].buff_size;
			err = pp2_bpool_init(&bpool_params, &pools[i][j]);
			if (err)
				return err;

			if (!pools[i][j]) {
				pr_err("BPool init failed!\n");
				return -EIO;
			}

			buffs_inf[i][j] = (struct pp2_buff_inf *)malloc(infs[j].num_buffs * sizeof(struct pp2_buff_inf));
			if (!buffs_inf[i][j]) {
				pr_err("no mem for bpools-inf array!\n");
				return -ENOMEM;
			}

			for (k = 0; k < infs[j].num_buffs; k++) {
				void *buff_virt_addr;

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
					pr_err("buff_virt_addr(%p)  upper out of range; skipping this buff\n",
						buff_virt_addr);
					continue;
				}
				buffs_inf[i][j][k].addr = (bpool_dma_addr_t)mv_sys_dma_mem_virt2phys(buff_virt_addr);
				/* cookie contains lower_32_bits of the va */
				buffs_inf[i][j][k].cookie = lower_32_bits((u64)buff_virt_addr);
			}
			for (k = 0 ; k < infs[j].num_buffs; k++) {
				struct pp2_buff_inf	tmp_buff_inf;

				tmp_buff_inf.cookie = buffs_inf[i][j][k].cookie;
				tmp_buff_inf.addr   = buffs_inf[i][j][k].addr;
				err = pp2_bpool_put_buff(hif, pools[i][j], &tmp_buff_inf);
				if (err)
					return err;
			}
		}
	}

	return 0;
}

