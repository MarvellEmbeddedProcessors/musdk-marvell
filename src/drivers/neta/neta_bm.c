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

#include "std_internal.h"
#include "lib/lib_misc.h"
#include "env/sys_iomem.h"
#include "drivers/mv_neta_bpool.h"
#include "drivers/mv_neta.h"
#include "neta_bm.h"
#include "neta_ppio.h"

static struct mvneta_bm		musdk_bm;
static struct mvneta_bm_pool	musdk_bm_pools[NETA_BPOOL_NUM_POOLS];

struct neta_bpool neta_bpools[NETA_BPOOL_NUM_POOLS];

static void mvneta_bm_write(struct mvneta_bm *priv, u32 offset, u32 data)
{
	writel(data, (void *)priv->reg_base + offset);
}

static u32 mvneta_bm_read(struct mvneta_bm *priv, u32 offset)
{
	return readl((void *)priv->reg_base + offset);
}

static void mvneta_bm_pool_enable(struct mvneta_bm *priv, int pool_id)
{
	u32 val;

	val = mvneta_bm_read(priv, MVNETA_BM_POOL_BASE_REG(pool_id));
	val |= MVNETA_BM_POOL_ENABLE_MASK;
	mvneta_bm_write(priv, MVNETA_BM_POOL_BASE_REG(pool_id), val);

	/* Clear BM cause register */
	mvneta_bm_write(priv, MVNETA_BM_INTR_CAUSE_REG, 0);
}

static void mvneta_bm_pool_disable(struct mvneta_bm *priv, int pool_id)
{
	u32 val;

	val = mvneta_bm_read(priv, MVNETA_BM_POOL_BASE_REG(pool_id));
	val &= ~MVNETA_BM_POOL_ENABLE_MASK;
	mvneta_bm_write(priv, MVNETA_BM_POOL_BASE_REG(pool_id), val);
}

/* Create pool */
static int mvneta_bm_pool_create(int pool_id)
{
	struct mvneta_bm_pool *bm_pool = &musdk_bm_pools[pool_id];
	int size_bytes;

	size_bytes = sizeof(u32) * bm_pool->capacity;
	bm_pool->virt_addr = mv_sys_dma_mem_alloc(size_bytes, MVNETA_BM_POOL_PTR_ALIGN);
	if (!bm_pool->virt_addr) {
		pr_err("Can't allocate input DMA buffer of %d bytes\n", size_bytes);
		return -ENOMEM;
	}
	memset(bm_pool->virt_addr, 0, size_bytes);
	bm_pool->phys_addr = mv_sys_dma_mem_virt2phys(bm_pool->virt_addr);

	/* Set pool address */
	mvneta_bm_write(&musdk_bm, MVNETA_BM_POOL_BASE_REG(bm_pool->id), bm_pool->phys_addr);

	bm_pool->priv = &musdk_bm;
	mvneta_bm_pool_enable(&musdk_bm, bm_pool->id);

	neta_bpools[bm_pool->id].id = bm_pool->id;
	neta_bpools[bm_pool->id].internal_param = bm_pool;

	return 0;
}

/* Cleanup pool */
void mvneta_bm_pool_destroy(struct mvneta_bm *priv, struct mvneta_bm_pool *bm_pool)
{
	bm_pool->type = MVNETA_BM_FREE;

	if (bm_pool->virt_addr) {
		/* free pool buffer TBD */
		bm_pool->virt_addr = NULL;
	}

	mvneta_bm_pool_disable(priv, bm_pool->id);
}

static int neta_bm_init(void)
{
	int err;
	struct sys_iomem_params params;
	static int bm_init_done;

	if (bm_init_done)
		return 0;

	params.type = SYS_IOMEM_T_UIO;
	params.index = -1;
	params.devname = "bm";

	if (sys_iomem_exists(&params)) {
		err = sys_iomem_init(&params, &musdk_bm.sys_iomem);
		if (err)
			return -1;
	} else
		return -1;

	/* Map the registers physical address */
	err = sys_iomem_map(musdk_bm.sys_iomem, "bm_regs", &musdk_bm.paddr, (void **)(&musdk_bm.reg_base));
	if (err) {
		pr_info("%s: failed sys_iomem_map().\n", __func__);
		sys_iomem_deinit(musdk_bm.sys_iomem);
		return err;
	}
	pr_debug("init %s unit: physical base 0x%lx, virtual 0x%lx.\n", params.devname, musdk_bm.paddr,
		musdk_bm.reg_base);

	/* Map the internal SRAM physical address */
	err = sys_iomem_map(musdk_bm.sys_iomem, "bm_sram", &musdk_bm.bppi_phys_addr,
			    (void **)(&musdk_bm.bppi_virt_addr));
	if (err) {
		pr_info("%s: failed sys_iomem_map().\n", __func__);
		sys_iomem_deinit(musdk_bm.sys_iomem);
		return err;
	}
	pr_debug("init %s sram: physical base 0x%lx, virtual %p.\n",
		params.devname, musdk_bm.bppi_phys_addr, musdk_bm.bppi_virt_addr);

	musdk_bm.bm_pools = musdk_bm_pools;
	bm_init_done = 1;

	return 0;
}

int neta_bpool_init(struct neta_bpool_params *params, struct neta_bpool **bpool)
{
	u8 match[2];
	struct neta_bpool	*pool;
	int pool_id;

	if (neta_bm_init()) {
		pr_err("cannot map BM HW unit\n");
		return -EIO;
	}

	if (mv_sys_match(params->match, "pool", 1, match))
		return(-ENXIO);

	if (!neta_is_initialized())
		return(-EPERM);

	pool_id = match[0];

	pool = kmalloc(sizeof(struct neta_bpool), GFP_KERNEL);
	if (!pool) {
		pr_err("no mem for neta pool\n");
		return -ENOMEM;
	}
	*bpool = pool;

	pool->id = pool_id;
	pool->internal_param = &musdk_bm_pools[pool_id];

	musdk_bm_pools[pool_id].id = pool_id;
	musdk_bm_pools[pool_id].type = (params->buff_len < 300) ? MVNETA_BM_SHORT : MVNETA_BM_LONG;
	musdk_bm_pools[pool_id].buf_size = params->buff_len;
	/* always create pool with MAX number of buffers */
	musdk_bm_pools[pool_id].capacity = MVNETA_BM_POOL_CAP_MAX;

	mvneta_bm_pool_create(pool_id);

	return 0;
}

void neta_bpool_deinit(struct neta_bpool *pool)
{
	struct mvneta_bm_pool *bm_pool = (struct mvneta_bm_pool *)pool->internal_param;

	mvneta_bm_pool_destroy(&musdk_bm, bm_pool);
	kfree(pool);
}

int neta_bpool_put_buffs(struct buff_release_entry buff_entry[], int *num)
{
	struct neta_bpool *pool;
	struct mvneta_bm_pool *bm_pool;
	int i;

	for (i = 0; i < *num; i++) {
		pool = buff_entry[i].bpool;
		bm_pool = (struct mvneta_bm_pool *)pool->internal_param;

		mvneta_bm_pool_put_bp(&musdk_bm, bm_pool, buff_entry[i].buff.addr);
	}
	*num = i;

	return 0;
}

int neta_bpool_put_buff(struct neta_bpool *pool, struct neta_buff_inf *buff_entry)
{
	struct mvneta_bm_pool *bm_pool = (struct mvneta_bm_pool *)pool->internal_param;

	if ((bm_pool->buffs_num + 1) > bm_pool->capacity)
		return -1;

	mvneta_bm_pool_put_bp(&musdk_bm, bm_pool, buff_entry->addr);

	return 0;
}

int neta_bpool_get_num_buffs(struct neta_bpool *pool, u32 *num_buffs)
{
	struct mvneta_bm_pool *bm_pool = (struct mvneta_bm_pool *)pool->internal_param;

	*num_buffs = bm_pool->buffs_num;

	return 0;
}

/**
 * Set to enable complete emptying of the BPPI.
 * When set, BPPI is considered empty when it holds no BPs
 */
static inline void neta_bm_config_set_empty(void)
{
	u32 val;

	val = mvneta_bm_read(&musdk_bm, MVNETA_BM_CONFIG_REG);
	val |= MVNETA_BM_EMPTY_LIMIT_MASK;
	mvneta_bm_write(&musdk_bm, MVNETA_BM_CONFIG_REG, val);
}

/**
 * Clear BPPI empty limit bit.
 * When clear, a certain margin of BPs is applied
 */
static inline void neta_bm_config_clear_empty(void)
{
	u32 val;

	val = mvneta_bm_read(&musdk_bm, MVNETA_BM_CONFIG_REG);
	val &= ~MVNETA_BM_EMPTY_LIMIT_MASK;
	mvneta_bm_write(&musdk_bm, MVNETA_BM_CONFIG_REG, val);
}

int neta_bpool_get_buff(struct neta_bpool *pool, struct neta_buff_inf *buff)
{
	struct mvneta_bm_pool *bm_pool = (struct mvneta_bm_pool *)pool->internal_param;
	dma_addr_t buf_phys_addr;

	neta_bm_config_set_empty();

	/* Get buffer physical address (indirect access) */
	buf_phys_addr = mvneta_bm_pool_get_bp(&musdk_bm, bm_pool);
	neta_bm_config_clear_empty();

	buff->cookie = (neta_cookie_t)(uintptr_t)mv_sys_dma_mem_phys2virt(buf_phys_addr);
	buff->addr = (neta_dma_addr_t)buf_phys_addr;
	return 0;
}
