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

#include "pp2.h"
#include "pp2_bm.h"
#include "pp2_hif.h"
#include "pp2_port.h"

#include "lib/lib_misc.h"

#define DUMMY_PKT_OFFS	64
#define DUMMY_PKT_EFEC_OFFS	(DUMMY_PKT_OFFS + MV_MH_SIZE)

#define GET_HW_BASE(pool)	((struct base_addr *)(pool)->internal_param)
#define SET_HW_BASE(pool, base)	{ (pool)->internal_param = (base); }

struct pp2_bpool pp2_bpools[PP2_MAX_NUM_PACKPROCS][PP2_BPOOL_NUM_POOLS];

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

	if (pool_id < 0 || pool_id >= PP2_BPOOL_NUM_POOLS) {
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
	param.buf_num = MVPP2_BM_POOL_SIZE_MAX;
	param.buf_size = params->buff_len;
	param.id = pool_id;
	param.pp2_id = pp2_id;
	param.likely_buffer_mem = params->likely_buffer_mem;
	rc = pp2_bm_pool_create(pp2_ptr, &param);
	if (!rc) {
		pp2_bpools[pp2_id][pool_id].id = pool_id;
		pp2_bpools[pp2_id][pool_id].pp2_id = pp2_id;
		SET_HW_BASE(&pp2_bpools[pp2_id][pool_id], &pp2_ptr->pp2_inst[pp2_id]->hw.base[0]);
		*bpool = &pp2_bpools[pp2_id][pool_id];
	}
	return rc;
}

void pp2_bpool_deinit(struct pp2_bpool *pool)
{
	uintptr_t cpu_slot;
	int pool_id;
	u32 buf_num;
	struct pp2_bm_pool *bm_pool;

	cpu_slot = GET_HW_BASE(pool)[PP2_DEFAULT_REGSPACE].va;
	pool_id = pool->id;

	/* Check buffer counters after free */
	pp2_bpool_get_num_buffs(pool, &buf_num);
	if (buf_num) {
		pr_warn("cannot free all buffers in pool %d, buf_num left %d\n",
			pool_id,
			buf_num);
	}

	bm_pool = pp2_bm_pool_get_pool_by_id(pp2_ptr->pp2_inst[pool->pp2_id], pool_id);

	if (bm_pool && !pp2_bm_pool_destroy(cpu_slot, bm_pool))
		pp2_ptr->pp2_inst[pool->pp2_id]->bm_pools[pool_id] = NULL;
	else
		pr_err("[%s] Can not destroy pool_id-%d:%d !\n", __func__,
			pool->pp2_id, pool_id);

}

/*TODO, move #define to correct file, maybe already exist in Linux...*/
#define MVPP22_BM_PHY_HIGH_ALLOC_MASK		0x00ff
int pp2_bpool_get_buff(struct pp2_hif *hif, struct pp2_bpool *pool, struct pp2_buff_inf *buff)
{
	uintptr_t cpu_slot;
	dma_addr_t paddr;
	int pool_id;
	u64 vaddr, high_addr_reg;

	cpu_slot = GET_HW_BASE(pool)[hif->regspace_slot].va;
	pool_id = pool->id;

	paddr =  pp2_reg_read(cpu_slot, MVPP2_BM_PHY_ALLOC_REG(pool_id));
	if (unlikely(!paddr)) {
		pr_err("BM: BufGet failed! (Pool ID=%d)\n", pool_id);
		return -ENOBUFS;
	}

	vaddr = pp2_reg_read(cpu_slot, MVPP2_BM_VIRT_ALLOC_REG);

	high_addr_reg = pp2_reg_read(cpu_slot, MVPP22_BM_PHY_VIRT_HIGH_ALLOC_REG);
	vaddr |= ((high_addr_reg & MVPP22_BM_VIRT_HIGH_ALLOC_MASK) << (32 - MVPP22_BM_VIRT_HIGH_ALLOC_OFFSET));

#if (MVCONF_DMA_PHYS_ADDR_T_SIZE == 64)
	paddr |= ((high_addr_reg & MVPP22_BM_PHY_HIGH_ALLOC_MASK) <<  (32 - MVPP22_BM_PHY_HIGH_ALLOC_OFFSET));
#endif
	buff->addr = paddr;

	buff->cookie = vaddr;
	return 0;
}

static inline void pp2_bpool_put_buffs_core(int pp2_id, int num_buffs, int dm_if_index,
					    struct pp2_ppio_desc pp2_descs[])
{
	u16 sent_pkts = 0;
	struct pp2_port *lb_port;

	lb_port = pp2_ptr->pp2_inst[pp2_id]->ports[PP2_LOOPBACK_PORT];
	do {
		sent_pkts += pp2_port_enqueue(lb_port, lb_port->parent->dm_ifs[dm_if_index], 0,
			     (num_buffs - sent_pkts), &pp2_descs[sent_pkts]);
	} while (sent_pkts != num_buffs);
}

/* This function does not support _not_ releasing all buffs, it continues to try until it has finished all buffers. */
int pp2_bpool_put_buffs(struct pp2_hif *hif, struct buff_release_entry buff_entry[], u16 *num)
{
	struct pp2_ppio_desc *cur_desc;
	int i, pp2_id;
	int pp_ind[PP2_NUM_PKT_PROC] = {0};

	for (i = 0; i < (*num); i++) {
		pp2_id = buff_entry[i].bpool->pp2_id;
		cur_desc = hif->rel_descs + PP2_MAX_NUM_PUT_BUFFS * pp2_id + pp_ind[pp2_id];
		pp2_ppio_outq_desc_reset(cur_desc);
		pp2_ppio_outq_desc_set_phys_addr(cur_desc, buff_entry[i].buff.addr);
		/* TODO: ASAP, check if setting to 0 creates issues. */
		pp2_ppio_outq_desc_set_pkt_offset(cur_desc, DUMMY_PKT_EFEC_OFFS);
		pp2_ppio_outq_desc_set_pkt_len(cur_desc, 0);
		pp2_ppio_outq_desc_set_cookie(cur_desc, buff_entry[i].buff.cookie);
		pp2_ppio_outq_desc_set_pool(cur_desc, buff_entry[i].bpool);
		cur_desc->cmds[3] = TXD_ERR_SUM_MASK;
		pp_ind[pp2_id]++;
#ifdef DEBUG
	do {
		int pool_id;
		struct mv_sys_dma_mem_region *likely_mem;
		dma_addr_t buf_addr = buff_entry[i].buff.addr;

		pool_id = buff_entry[i].bpool->id;
		likely_mem = pp2_ptr->pp2_inst[pp2_id]->bm_pools[pool_id]->likely_buffer_mem;

		if (likely_mem)
			if ((buf_addr < likely_mem->dma_phys_base) ||
			    (buf_addr > (likely_mem->dma_phys_base + likely_mem->size)))
				pr_debug("(%s): buf_addr" PRIdma ", not in likely_mem range mem_id(%d)\n",
					 __func__, buf_addr, likely_mem->mem_id);
	} while (0);
#endif
	}
	for (i = 0; i < PP2_NUM_PKT_PROC; i++) {
		if (pp_ind[i])
			pp2_bpool_put_buffs_core(i, pp_ind[i], hif->regspace_slot,
						 hif->rel_descs + PP2_MAX_NUM_PUT_BUFFS * i);
	}

	return 0;
}

/* Previous pp2_bpool_put_buff() function, not based on loopback_port. To be discontinued. */
#if 1
int pp2_bpool_put_buff(struct pp2_hif *hif, struct pp2_bpool *pool, struct pp2_buff_inf *buff)
{
	uintptr_t cpu_slot;
	dma_addr_t paddr;
	u64 vaddr;
	u32 virt_lo, virt_hi, phys_hi, phys_lo, high_addr_reg = 0;

	vaddr = buff->cookie;
	virt_lo = (u32)vaddr;
	virt_hi = vaddr >> 32;

	cpu_slot = GET_HW_BASE(pool)[hif->regspace_slot].va;
	paddr = buff->addr;
	phys_lo = (u32)paddr;
	phys_hi = paddr >> 32;

#if PP2_BM_BUF_DEBUG
	/* The buffers should be 32 bytes aligned */
	if (!IS_ALIGNED(vaddr, BM_BUF_ALIGN)) {
		pr_err("BM: the buffer is not %u-byte aligned: %p", BM_BUF_ALIGN, (void *)vaddr);
		return 1;
	}
	pr_info("BM: BufPut %p\n", (void *)vaddr);
#endif

	/* First set the virt and phys hi addresses and the virt lo address.
	 * Lastly, write phys lo to BM_PHYS_RLS to trigger the BM release
	 */

	high_addr_reg |= (virt_hi << MVPP22_BM_VIRT_HIGH_RLS_OFFST);
#if (MVCONF_DMA_PHYS_ADDR_T_SIZE == 64)
	high_addr_reg |= (phys_hi << MVPP22_BM_PHY_HIGH_RLS_OFFSET);
#endif

	pp2_relaxed_reg_write(cpu_slot, MVPP22_BM_PHY_VIRT_HIGH_RLS_REG, high_addr_reg);
	pp2_relaxed_reg_write(cpu_slot, MVPP2_BM_VIRT_RLS_REG, virt_lo);
	pp2_relaxed_reg_write(cpu_slot, MVPP2_BM_PHY_RLS_REG(pool->id), phys_lo);

	return 0;
}
#endif

int pp2_bpool_get_num_buffs(struct pp2_bpool *pool, u32 *num_buffs)
{
	uintptr_t	cpu_slot;
	u32		num = 0;

	cpu_slot = GET_HW_BASE(pool)[PP2_DEFAULT_REGSPACE].va;

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

