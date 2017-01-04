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
#include "drivers/mv_sam.h"
#include "sam.h"

void sam_hw_reg_print(char *reg_name, void *base, u32 offset)
{
	void *addr = base + offset;

	pr_info("  %-32s: 0x%8p = 0x%08x\n", reg_name, addr, readl(addr));
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

	val32 = (SAM_CDR_DSCR_MAX_WORD_COUNT & MASK_8_BITS); /* size of Command descriptor */
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

	val32 = (SAM_RDR_DSCR_MAX_WORD_COUNT & MASK_8_BITS); /* size of Command descriptor */
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

int sam_hw_ring_init(u32 ring, u32 size, struct sam_hw_ring *hw_ring)
{
	u32 ring_size, offset;
	int rc;
	char ring_name[64];
	Device_Handle_t device;
	u32 *base;

	sprintf(ring_name, "EIP202_CDR%d", ring);
	device = Device_Find(ring_name);
	if (device == NULL) {
		pr_err("Can't find %s device\n", ring_name);
		return -EINVAL;
	}
	base = Device_Get_Base(device);
	offset = Device_Get_Offset(device);

	hw_ring->id = ring;
	hw_ring->regs_vbase = (u32 *)(base + (offset >> 2));
	hw_ring->ring_size = size; /* number of descriptors in the ring */

	pr_info("%s: Regs: %p + 0x%08x\n", ring_name, base, offset);

	sam_hw_cdr_regs_reset(hw_ring);
	sam_hw_rdr_regs_reset(hw_ring);

	/* Allocate command descriptors ring */
	ring_size = SAM_CDR_ENTRY_WORDS * sizeof(u32) * size;
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
	ring_size = SAM_RDR_ENTRY_WORDS * sizeof(u32) * size;
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

int sam_hw_engine_load(void)
{
	pr_info("Load SAM HW engine\n");

	if (Driver197_Init()) {
		pr_err("Can't init eip197 driver\n");
		return -ENODEV;
	}
	return 0;
}

int sam_hw_engine_unload(void)
{
	pr_info("Unload SAM HW engine\n");
	Driver197_Exit();

	return 0;
}

void sam_hw_cdr_regs_show(struct sam_hw_ring *hw_ring)
{
}

void sam_hw_rdr_regs_show(struct sam_hw_ring *hw_ring)
{
}
