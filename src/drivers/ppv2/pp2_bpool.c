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

#include "std_internal.h"

#include "pp2_bpool.h"
#include "pp2.h"
#include "pp2_bm.h"
#include "pp2_hif.h"
#include "lib/lib_misc.h"

static struct pp2_bpool pp2_bpool[PP2_MAX_NUM_PACKPROCS][PP2_NUM_BMPOOLS];

int pp2_bpool_init(struct pp2_bpool_params *params, struct pp2_bpool **bpool)
{
	u8 match[2];
	struct bm_pool_param param;
	int pool_id, pp2_id, rc;

	if (mv_sys_match(params->match, "pool", 2, match))
		return(-ENXIO);

	if (pp2_is_init() == false)
		return(-EPERM);

	pp2_id = match[0];
	pool_id = match[1];

	if (pool_id < 0 || pool_id >= PP2_NUM_BMPOOLS) {
		pr_err("[%s] Invalid match string!\n", __func__);
		return(-ENXIO);
	}
	if (pp2_id < 0 || pp2_id >= pp2_ptr->num_pp2_inst) {
		pr_err("[%s] Invalid match string!\n", __func__);
		return(-ENXIO);
	}
	if (pp2_ptr->init.bm_pool_reserved_map & (1 << pool_id)) {
		pr_err("[%s] bm_pool is reserved.\n", __func__);
		return(-EFAULT);
	}
	if (pp2_ptr->pp2_inst[pp2_id]->bm_pools[pool_id]) {
		pr_err("[%s] bm_pool already exists.\n", __func__);
		return(-EEXIST);
	}
	pr_debug("[%s] pp2_id(%d) pool_id(%d)\n", __func__, pp2_id, pool_id);
	param.buf_num = params->max_num_buffs;
	param.buf_size = params->buff_len;
	param.id = pool_id;
	param.pp2_id = pp2_id;
	rc = pp2_bm_pool_create(pp2_ptr, &param);
	if (!rc) {
		pp2_bpool[pp2_id][pool_id].id = pool_id;
		pp2_bpool[pp2_id][pool_id].pp2_id = pp2_id;
		pp2_bpool[pp2_id][pool_id].pp2_hw_base = &pp2_ptr->pp2_inst[pp2_id]->hw.base[0];
		*bpool = &pp2_bpool[pp2_id][pool_id];
	}
	return rc;
}

void pp2_bpool_deinit(struct pp2_bpool *bpool)
{
	pr_err("[%s] routine not supported yet!\n", __func__);
}

/*TODO, move #define to correct file, maybe already exist in Linux...*/
#define MVPP22_BM_PHY_HIGH_ALLOC_MASK		0x00ff
int pp2_bpool_get_buff(struct pp2_hif *hif, struct pp2_bpool *pool, struct pp2_buff_inf *buff)
{
	uintptr_t cpu_slot;
	bpool_dma_addr_t paddr;
	int pool_id;
#ifdef CONF_PP2_BPOOL_COOKIE_SIZE
	pp2_cookie_t vaddr;
#endif

	cpu_slot = pool->pp2_hw_base[hif->regspace_slot].va;
	pool_id = pool->id;

	paddr =  pp2_reg_read(cpu_slot, MVPP2_BM_PHY_ALLOC_REG(pool_id));
	if (unlikely(!paddr)) {
		pr_err("BM: BufGet failed! (Pool ID=%d)\n", pool_id);
		return -ENOBUFS;
	}

#ifdef CONF_PP2_BPOOL_COOKIE_SIZE
	vaddr = pp2_reg_read(cpu_slot, MVPP2_BM_VIRT_ALLOC_REG);
#endif
#if ((CONF_PP2_BPOOL_COOKIE_SIZE == 64) || (defined(MVCONF_ARCH_DMA_ADDR_T_64BIT) && \
	!defined(CONF_PP2_BPOOL_DMA_ADDR_USE_32B)))
	{
		u64 high_addr_reg;

		high_addr_reg = pp2_reg_read(cpu_slot, MVPP22_BM_PHY_VIRT_HIGH_ALLOC_REG);

#if (CONF_PP2_BPOOL_COOKIE_SIZE == 64)
		vaddr |= ((high_addr_reg & MVPP22_BM_VIRT_HIGH_ALLOC_MASK) << (32 - MVPP22_BM_VIRT_HIGH_ALLOC_OFFSET));
#endif

#if (defined(MVCONF_ARCH_DMA_ADDR_T_64BIT) && !defined(CONF_PP2_BPOOL_DMA_ADDR_USE_32B))
		paddr |= ((high_addr_reg & MVPP22_BM_PHY_HIGH_ALLOC_MASK) <<  (32 - MVPP22_BM_PHY_HIGH_ALLOC_OFFSET));
#endif
	}
#endif
	buff->addr = paddr;

#ifdef CONF_PP2_BPOOL_COOKIE_SIZE
	buff->cookie = vaddr;
#endif
	return 0;
}

int pp2_bpool_put_buff(struct pp2_hif *hif, struct pp2_bpool *pool, struct pp2_buff_inf *buff)
{
	u32 phys_lo;
	uintptr_t cpu_slot;
	bpool_dma_addr_t paddr;
#ifdef CONF_PP2_BPOOL_COOKIE_SIZE
	pp2_cookie_t vaddr;
	u32 virt_lo;

	vaddr = buff->cookie;
	virt_lo = (u32)vaddr;
#endif

	cpu_slot = pool->pp2_hw_base[hif->regspace_slot].va;
	paddr = buff->addr;

#if PP2_BM_BUF_DEBUG
	/* The buffers should be 32 bytes aligned */
	if (!IS_ALIGNED(vaddr, BM_BUF_ALIGN)) {
		pr_err("BM: the buffer is not %u-byte aligned: %p", BM_BUF_ALIGN, (void *)vaddr);
		return 1;
	}
	pr_info("BM: BufPut %p\n", (void *)vaddr);
#endif

	phys_lo = (u32)paddr;

	/* First set the virt and phys hi addresses and the virt lo address.
	 * Lastly, write phys lo to BM_PHYS_RLS to trigger the BM release
	 */

#if ((CONF_PP2_BPOOL_COOKIE_SIZE == 64) || (defined(MVCONF_ARCH_DMA_ADDR_T_64BIT) && \
	!defined(CONF_PP2_BPOOL_DMA_ADDR_USE_32B)))
	{
		u32 high_addr_reg = 0;

#if (CONF_PP2_BPOOL_COOKIE_SIZE == 64)
		u32 virt_hi = vaddr >> 32;

		high_addr_reg |= (virt_hi << MVPP22_BM_VIRT_HIGH_RLS_OFFST);
#endif
#if (defined(MVCONF_ARCH_DMA_ADDR_T_64BIT) && !defined(CONF_PP2_BPOOL_DMA_ADDR_USE_32B))
		u32 phys_hi = paddr >> 32;

		high_addr_reg |= (phys_hi << MVPP22_BM_PHY_HIGH_RLS_OFFSET);
#endif

		pp2_relaxed_reg_write(cpu_slot, MVPP22_BM_PHY_VIRT_HIGH_RLS_REG, high_addr_reg);
	}
#endif
	pp2_relaxed_reg_write(cpu_slot, MVPP2_BM_VIRT_RLS_REG, virt_lo);
	pp2_relaxed_reg_write(cpu_slot, MVPP2_BM_PHY_RLS_REG(pool->id), phys_lo);

	return 0;
}

int pp2_bpool_get_num_buffs(struct pp2_hif *hif, struct pp2_bpool *pool, u32 *num_buffs)
{
	uintptr_t	cpu_slot;
	u32		num = 0;

	cpu_slot = pool->pp2_hw_base[hif->regspace_slot].va;

	num = pp2_reg_read(cpu_slot, MVPP2_BM_POOL_PTRS_NUM_REG(pool->id))
				& MVPP22_BM_POOL_PTRS_NUM_MASK;
	num += pp2_reg_read(cpu_slot, MVPP2_BM_BPPI_PTRS_NUM_REG(pool->id))
				& MVPP2_BM_BPPI_PTR_NUM_MASK;

	/* HW has one buffer ready and is not reflected in "external + internal" counters */
	if (num)
		num++;

	*num_buffs = num;

	return 0;
}

