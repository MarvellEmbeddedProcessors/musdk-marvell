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
#include "lib/uio_helper.h"

#include "drivers/mv_sam.h"
#include "sam.h"

static struct sam_hw_engine_info sam_hw_engine_info[SAM_HW_ENGINE_NUM];

static const char *const sam_supported_name[] = {"uio_eip197", "uio_eip97"};
static enum sam_hw_type sam_supported_type[] = {HW_EIP197, HW_EIP97};

static struct sam_hw_engine_info *sam_uio_init(int engine)
{
	struct uio_info_t *node;
	int i;
	char name[16];

	if (sam_hw_engine_info[engine].uio_info)
		return &sam_hw_engine_info[engine];

	/* We support two HW types: eip197 or eip97 */
	for (i = 0; i < ARRAY_SIZE(sam_supported_name); i++) {
		snprintf(name, sizeof(name), "%s_%d", sam_supported_name[i], engine);
		node = uio_find_devices_byname(name);
		if (node) {
			sam_hw_engine_info[engine].uio_info = node;
			sam_hw_engine_info[engine].type = sam_supported_type[i];
			break;
		}
	}
	if (!node) {
		pr_err("Can't find %s UIO device\n", name);
		return NULL;
	}

	i = 0;
	while (node) {
		uio_get_all_info(node);
		node = node->next;
		i++;
	}
	return &sam_hw_engine_info[engine];
}

static void *sam_uio_iomap(struct uio_info_t *uio_info, dma_addr_t *pa,
			const char *name)
{
	struct uio_mem_t *mem = NULL;
	void *va = NULL;

	mem = uio_find_mem_byname(uio_info, name);
	if (!mem) {
		pr_err("%s memory not not found on uio %s\n", name, uio_info->name);
		return NULL;
	}

	if (mem->fd < 0) {
		char dev_name[16];

		snprintf(dev_name, sizeof(dev_name),
			 "/dev/uio%d", mem->info->uio_num);
		mem->fd = open(dev_name, O_RDWR);
	}

	if (mem->fd >= 0) {
		va = uio_single_mmap(mem->info, mem->map_num, mem->fd);
		if (!va)
			pr_err("%s Can't mmap memory on UIO %s\n", name, uio_info->name);

		if (pa)
			*pa = mem->info->maps[mem->map_num].addr;

		uio_free_mem_info(mem);
	}
	return va;
}

void sam_hw_reg_print(char *reg_name, void *base, u32 offset)
{
	void *addr = base + offset;

	pr_info("%-32s: %8p = 0x%08x\n", reg_name, addr, readl(addr));
}

int sam_hw_cdr_regs_reset(struct sam_hw_ring *hw_ring)
{
	u32 val32;

	/* Clear CDR count */
	val32 = BIT_31;
	sam_hw_reg_write(hw_ring->regs_vbase, HIA_CDR_COUNT_REG, val32);

	/* Re-init CDR */
	sam_hw_reg_write(hw_ring->regs_vbase, HIA_CDR_PREP_PTR_REG, 0);
	sam_hw_reg_write(hw_ring->regs_vbase, HIA_CDR_PROC_PTR_REG, 0);

	sam_hw_reg_write(hw_ring->regs_vbase, HIA_CDR_RING_BASE_ADDR_LO_REG, 0);
	sam_hw_reg_write(hw_ring->regs_vbase, HIA_CDR_RING_BASE_ADDR_HI_REG, 0);

	/* Restore default register values */
	/* EIP202_CDR_RING_SIZE_DEFAULT_WR(Device); */
	/* EIP202_CDR_DESC_SIZE_DEFAULT_WR(Device); */
	/* EIP202_CDR_CFG_DEFAULT_WR(Device); */
	/* EIP202_CDR_DMA_CFG_DEFAULT_WR(Device); */

	/* Clear and disable all CDR interrupts */
	sam_hw_reg_write(hw_ring->regs_vbase, HIA_CDR_STAT_REG, MASK_5_BITS);

	return 0;
}

int sam_hw_cdr_regs_init(struct sam_hw_ring *hw_ring)
{
	u32 val32;

	val32 = lower_32_bits(hw_ring->cdr_buf.paddr);
	sam_hw_reg_write(hw_ring->regs_vbase, HIA_CDR_RING_BASE_ADDR_LO_REG, val32);

	val32 = upper_32_bits(hw_ring->cdr_buf.paddr);
	sam_hw_reg_write(hw_ring->regs_vbase, HIA_CDR_RING_BASE_ADDR_HI_REG, val32);

	/* ring_size in words */
	val32 = ((hw_ring->ring_size * SAM_CDR_ENTRY_WORDS) & MASK_22_BITS) << 2;
	sam_hw_reg_write(hw_ring->regs_vbase, HIA_CDR_RING_SIZE_REG, val32);

	if (hw_ring->type == HW_EIP197)
		val32 = (SAM_CDR_DSCR_EXT_WORD_COUNT & MASK_8_BITS); /* size of Extended Command descriptor */
	else
		val32 = (SAM_CDR_DSCR_WORD_COUNT & MASK_8_BITS); /* size of Command descriptor */

	val32 |= (SAM_CDR_ENTRY_WORDS & MASK_8_BITS) << 16; /* distance between descriptors */
	val32 |= BIT_30; /* acdp_present (Token pointer in the descriptor) */
	val32 |= BIT_31; /* 64 bits mode */
	sam_hw_reg_write(hw_ring->regs_vbase, HIA_CDR_DESC_SIZE_REG, val32);

	val32 = (0x10 & MASK_12_BITS);		/* Number of words to fetch */
	val32 |= (0x0C & MASK_12_BITS) << 16;	/* Threshold in words to start fetch */
	sam_hw_reg_write(hw_ring->regs_vbase, HIA_CDR_CFG_REG, val32);

	val32 = (SAM_RING_DESC_SWAP_VALUE  & MASK_3_BITS);         /* cd_swap */
	val32 |= (SAM_RING_DESC_PROT_VALUE & MASK_3_BITS) << 4;    /* cd_prot */
	val32 |= (SAM_RING_DATA_SWAP_VALUE  & MASK_3_BITS) << 8;   /* data_swap*/
	val32 |= (SAM_RING_DATA_PROT_VALUE & MASK_3_BITS) << 12;   /* data_prot */
	val32 |= (SAM_RING_TOKEN_SWAP_VALUE  & MASK_3_BITS) << 16; /* acd_swap (token) */
	val32 |= (SAM_RING_TOKEN_PROT_VALUE & MASK_3_BITS) << 20;  /* acd_prot (token) */
	val32 |= BIT_24; /* wr_own_buf */
	val32 |= (SAM_RING_WRITE_CACHE_CTRL & MASK_3_BITS) << 25;  /* wr_cache */
	val32 |= (SAM_RING_READ_CACHE_CTRL & MASK_3_BITS) << 29;   /* rd_cache */
	sam_hw_reg_write(hw_ring->regs_vbase, HIA_CDR_DMA_CFG_REG, val32);

	return 0;
}

int sam_hw_rdr_regs_reset(struct sam_hw_ring *hw_ring)
{
	u32 val32;

	/* Clear RDR count */
	val32 = BIT_31;
	sam_hw_reg_write(hw_ring->regs_vbase, HIA_RDR_PREP_COUNT_REG, val32);
	sam_hw_reg_write(hw_ring->regs_vbase, HIA_RDR_PROC_COUNT_REG, val32);

	/* Re-init RDR */
	sam_hw_reg_write(hw_ring->regs_vbase, HIA_RDR_PREP_PTR_REG, 0);
	sam_hw_reg_write(hw_ring->regs_vbase, HIA_RDR_PROC_PTR_REG, 0);

	/* Restore default register values */
	sam_hw_reg_write(hw_ring->regs_vbase, HIA_RDR_RING_BASE_ADDR_LO_REG, 0);
	sam_hw_reg_write(hw_ring->regs_vbase, HIA_RDR_RING_BASE_ADDR_HI_REG, 0);

	/* EIP202_RDR_RING_SIZE_DEFAULT_WR(Device); */
	/* EIP202_RDR_DESC_SIZE_DEFAULT_WR(Device); */
	/* EIP202_RDR_CFG_DEFAULT_WR(Device); */
	/* EIP202_RDR_DMA_CFG_DEFAULT_WR(Device); */
	/* EIP202_RDR_THRESH_DEFAULT_WR(Device); - needed for interrupt */

	/* Clear and disable all RDR interrupts */
	val32 = MASK_8_BITS;
	sam_hw_reg_write(hw_ring->regs_vbase, HIA_RDR_STAT_REG, val32);

	return 0;
}

int sam_hw_rdr_regs_init(struct sam_hw_ring *hw_ring)
{
	u32 val32;

	val32 = lower_32_bits(hw_ring->rdr_buf.paddr);
	sam_hw_reg_write(hw_ring->regs_vbase, HIA_RDR_RING_BASE_ADDR_LO_REG, val32);

	val32 = upper_32_bits(hw_ring->rdr_buf.paddr);
	sam_hw_reg_write(hw_ring->regs_vbase, HIA_RDR_RING_BASE_ADDR_HI_REG, val32);

	/* ring_size in words */
	val32 = ((hw_ring->ring_size * SAM_RDR_ENTRY_WORDS) & MASK_22_BITS) << 2;
	sam_hw_reg_write(hw_ring->regs_vbase, HIA_RDR_RING_SIZE_REG, val32);

	if (hw_ring->type == HW_EIP197)
		val32 = (SAM_RDR_DSCR_EXT_WORD_COUNT & MASK_8_BITS); /* size of Extended Command descriptor */
	else
		val32 = (SAM_RDR_DSCR_WORD_COUNT & MASK_8_BITS); /* size of Command descriptor */

	val32 |= (SAM_RDR_ENTRY_WORDS & MASK_8_BITS) << 16; /* distance between descriptors */
	val32 |= BIT_31; /* 64 bits mode */
	sam_hw_reg_write(hw_ring->regs_vbase, HIA_RDR_DESC_SIZE_REG, val32);

	val32 = (0x50 & MASK_12_BITS);		/* Number of words to fetch */
	val32 |= (0x14 & MASK_9_BITS) << 16;	/* Threshold in words to start fetch */
	val32 |= BIT_25; /* oflo_irq_en */
	sam_hw_reg_write(hw_ring->regs_vbase, HIA_RDR_CFG_REG, val32);

	val32 = (SAM_RING_DESC_SWAP_VALUE  & MASK_3_BITS);         /* rd_swap */
	val32 |= (SAM_RING_DESC_PROT_VALUE & MASK_3_BITS) << 4;    /* cd_prot */
	val32 |= (SAM_RING_DATA_SWAP_VALUE  & MASK_3_BITS) << 8;   /* data_swap*/
	val32 |= (SAM_RING_DATA_PROT_VALUE & MASK_3_BITS) << 12;   /* data_prot */
	/* BIT_22 - wr_res_buf */
	/* BIT_23 - wr_ctrl_buf */
	val32 |= BIT_24; /* wr_own_buf */
	val32 |= (SAM_RING_WRITE_CACHE_CTRL & MASK_3_BITS) << 25;  /* wr_cache */
	/* BIT_28 - pad_to_offset */
	val32 |= (SAM_RING_READ_CACHE_CTRL & MASK_3_BITS) << 29;   /* rd_cache */
	sam_hw_reg_write(hw_ring->regs_vbase, HIA_RDR_DMA_CFG_REG, val32);

	return 0;
}

int sam_hw_ring_init(u32 engine, u32 ring, struct sam_cio_params *params,
		     struct sam_hw_ring *hw_ring)
{
	u32 ring_size;
	int rc;
	void *vaddr;
	dma_addr_t paddr;
	struct sam_hw_engine_info *engine_info;
	const char name[] = "regs";

	engine_info = sam_uio_init(engine);
	if (!engine_info)
		return -EINVAL;

	vaddr = sam_uio_iomap(engine_info->uio_info, &paddr, name);
	if (!vaddr) {
		pr_err("%s: Can't iomap memory area\n",	name);
		return -ENOMEM;
	}

	/* Check ring size and number of sessions with HW max values */
	if (params->size > SAM_HW_RING_SIZE) {
		/* SW value can't exceed HW restriction */
		pr_warn("Ring size %d is too large. Set to maximum = %d\n",
			params->size, SAM_HW_RING_SIZE);
		params->size = SAM_HW_RING_SIZE;
	}
	params->size += 1;

	if (params->num_sessions > SAM_HW_SA_NUM) {
		/* SW value can't exceed HW restriction */
		pr_warn("Number of sessions %d is too large. Set to maximum = %d\n",
			params->num_sessions, SAM_HW_SA_NUM);
		params->num_sessions = SAM_HW_SA_NUM;
	}
	if (engine_info->type == HW_EIP197) {
		hw_ring->regs_vbase = (((char *)vaddr) + SAM_EIP197_RING_REGS_OFFS(ring));
		hw_ring->paddr = paddr + SAM_EIP197_RING_REGS_OFFS(ring);
	} else if (engine_info->type == HW_EIP97) {
		hw_ring->regs_vbase = (((char *)vaddr) + SAM_EIP97_RING_REGS_OFFS(ring));
		hw_ring->paddr = paddr + SAM_EIP97_RING_REGS_OFFS(ring);
	} else {
		pr_err("%s: Unexpected HW type = %d\n", __func__, engine_info->type);
		return -EINVAL;
	}

	pr_info("%s: %d:%d registers: paddr: 0x%x, vaddr: 0x%p\n",
		(engine_info->type == HW_EIP197) ? "eip197" : "eip97",
		engine, ring, (unsigned)hw_ring->paddr, hw_ring->regs_vbase);

	hw_ring->engine = engine;
	hw_ring->type = engine_info->type;
	hw_ring->ring = ring;
	hw_ring->ring_size = params->size; /* number of descriptors in the ring */

	sam_hw_cdr_regs_reset(hw_ring);
	sam_hw_rdr_regs_reset(hw_ring);

	/* Allocate command descriptors ring */
	ring_size = SAM_CDR_ENTRY_WORDS * sizeof(u32) * params->size;
	rc = sam_dma_buf_alloc(ring_size, &hw_ring->cdr_buf);
	if (rc)
		return rc;

	pr_info("DMA buffer (%d bytes) for CDR #%d allocated: paddr = 0x%lx, vaddr = %p\n",
		ring_size, ring, hw_ring->cdr_buf.paddr, hw_ring->cdr_buf.vaddr);

	memset(hw_ring->cdr_buf.vaddr, 0, ring_size);
	hw_ring->cmd_desc_first = hw_ring->cdr_buf.vaddr;

	/* Init CDR registers */
	sam_hw_cdr_regs_init(hw_ring);

	/* Allocate result descriptors ring */
	ring_size = SAM_RDR_ENTRY_WORDS * sizeof(u32) * params->size;
	rc = sam_dma_buf_alloc(ring_size, &hw_ring->rdr_buf);
	if (rc)
		return rc;

	pr_info("DMA buffer (%d bytes) for RDR #%d allocated: paddr = 0x%lx, vaddr = %p\n",
		ring_size, ring, hw_ring->rdr_buf.paddr, hw_ring->rdr_buf.vaddr);

	memset(hw_ring->rdr_buf.vaddr, 0, ring_size);
	hw_ring->res_desc_first = hw_ring->rdr_buf.vaddr;

	/* Init RDR registers */
	sam_hw_rdr_regs_init(hw_ring);

	return 0;
}

int sam_hw_ring_deinit(struct sam_hw_ring *hw_ring)
{
	sam_hw_cdr_regs_reset(hw_ring);
	sam_hw_rdr_regs_reset(hw_ring);

	sam_dma_buf_free(&hw_ring->cdr_buf);
	sam_dma_buf_free(&hw_ring->rdr_buf);

	return 0;
}

int sam_hw_session_invalidate(struct sam_hw_ring *hw_ring, struct sam_buf_info *sa_buf,
				u32 next_request)
{
	sam_hw_ring_sa_inv_desc_write(hw_ring, next_request, sa_buf->paddr);

	sam_hw_ring_submit(hw_ring, 1);

	return 0;
}

int sam_hw_engine_load(void)
{
#ifdef SAM_EIP_DDK_HW_INIT
	pr_info("Load SAM HW engine\n");

	if (Driver197_Init()) {
		pr_err("Can't init eip197 driver\n");
		return -ENODEV;
	}
#endif
	return 0;
}

int sam_hw_engine_unload(void)
{
#ifdef SAM_EIP_DDK_HW_INIT
	pr_info("Unload SAM HW engine\n");
	Driver197_Exit();
#endif
	return 0;
}

void sam_hw_cdr_regs_show(struct sam_hw_ring *hw_ring)
{
	pr_info("%d:%d CDR registers\n", hw_ring->engine, hw_ring->ring);
	sam_hw_reg_print("HIA_CDR_RING_BASE_ADDR_LO_REG", hw_ring->regs_vbase, HIA_CDR_RING_BASE_ADDR_LO_REG);
	sam_hw_reg_print("HIA_CDR_RING_SIZE_REG", hw_ring->regs_vbase, HIA_CDR_RING_SIZE_REG);
	sam_hw_reg_print("HIA_CDR_DESC_SIZE_REG", hw_ring->regs_vbase, HIA_CDR_DESC_SIZE_REG);
	sam_hw_reg_print("HIA_CDR_CFG_REG", hw_ring->regs_vbase, HIA_CDR_CFG_REG);
	sam_hw_reg_print("HIA_CDR_DMA_CFG_REG", hw_ring->regs_vbase, HIA_CDR_DMA_CFG_REG);
	sam_hw_reg_print("HIA_CDR_THRESH_REG", hw_ring->regs_vbase, HIA_CDR_THRESH_REG);
	sam_hw_reg_print("HIA_CDR_STAT_REG", hw_ring->regs_vbase, HIA_CDR_STAT_REG);
}

void sam_hw_rdr_regs_show(struct sam_hw_ring *hw_ring)
{
	pr_info("%d:%d RDR registers\n", hw_ring->engine, hw_ring->ring);
	sam_hw_reg_print("HIA_RDR_RING_BASE_ADDR_LO_REG", hw_ring->regs_vbase, HIA_RDR_RING_BASE_ADDR_LO_REG);
	sam_hw_reg_print("HIA_RDR_RING_SIZE_REG", hw_ring->regs_vbase, HIA_RDR_RING_SIZE_REG);
	sam_hw_reg_print("HIA_RDR_DESC_SIZE_REG", hw_ring->regs_vbase, HIA_RDR_DESC_SIZE_REG);
	sam_hw_reg_print("HIA_RDR_CFG_REG", hw_ring->regs_vbase, HIA_RDR_CFG_REG);
	sam_hw_reg_print("HIA_RDR_DMA_CFG_REG", hw_ring->regs_vbase, HIA_RDR_DMA_CFG_REG);
	sam_hw_reg_print("HIA_RDR_THRESH_REG", hw_ring->regs_vbase, HIA_RDR_THRESH_REG);
	sam_hw_reg_print("HIA_RDR_STAT_REG", hw_ring->regs_vbase, HIA_RDR_STAT_REG);
}
