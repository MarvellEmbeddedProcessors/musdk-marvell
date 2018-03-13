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
#include "env/sys_iomem.h"

#include "drivers/mv_sam.h"
#include "sam.h"
#include "sam_hw.h"

static struct sam_hw_device_info sam_hw_device_info[SAM_HW_DEVICE_NUM];

static const char *const sam_supported_name[] = {"eip197", "eip97"};
static enum sam_hw_type sam_supported_type[] = {HW_EIP197, HW_EIP97};

static struct sys_iomem *sam_iomem_init(int device, enum sam_hw_type *type)
{
	struct sys_iomem *iomem_info;
	int i, err;
	struct sys_iomem_params params;

	/* We support two HW types: eip197 or eip97 */
	params.type = SYS_IOMEM_T_UIO;
	params.index = device;
	for (i = 0; i < ARRAY_SIZE(sam_supported_name); i++) {
		params.devname = sam_supported_name[i];
		if (sys_iomem_exists(&params)) {
			err = sys_iomem_init(&params, &iomem_info);
			if (!err) {
				*type = sam_supported_type[i];
				return iomem_info;
			}
		}
	}
	return NULL;
}

int sam_hw_ring_enable_irq(struct sam_hw_ring *hw_ring)
{
	/* Enable the RDR interrupt for the ring */
	sam_hw_reg_write(hw_ring->aic_regs_vbase, SAM_AIC_RING_ENABLE_REG,
			 SAM_RDR_IRQ(hw_ring->ring));

	return 0;
}

int sam_hw_ring_disable_irq(struct sam_hw_ring *hw_ring)
{
	/* Disable the RDR interrupt for the ring */
	sam_hw_reg_write(hw_ring->aic_regs_vbase, SAM_AIC_RING_ENABLE_CLR_REG,
			 SAM_RDR_IRQ(hw_ring->ring));

	return 0;
}

int sam_hw_ring_prepare_rdr_thresh(struct sam_hw_ring *hw_ring, u32 pkts_coal, u32 usec_coal)
{
	u32 timeout;

	/* multiple of 256 clock cycles of HIA clock */
	timeout = (SAM_HIA_CLOCK_FREQ * usec_coal) / 256;
	if (timeout > SAM_RING_TIMEOUT_MASK) {
		pr_warn("ISR time coalescing %d usecs is too large. Cut to maximum value %d usecs\n",
			usec_coal, (SAM_RING_TIMEOUT_MASK * 256) / SAM_HIA_CLOCK_FREQ);
		usec_coal = (SAM_RING_TIMEOUT_MASK * 256) / SAM_HIA_CLOCK_FREQ;
		timeout = SAM_RING_TIMEOUT_MASK;
	}
	if (pkts_coal > SAM_RING_PKT_THRESH_MASK) {
		pr_warn("ISR packets coalescing %d pkts is too large. Cut to maximum value %d pkts\n",
			pkts_coal, SAM_RING_PKT_THRESH_MASK);
		pkts_coal = SAM_RING_PKT_THRESH_MASK;
	}

	hw_ring->rdr_thresh_val = SAM_RING_PKT_MODE_MASK | SAM_RING_PKT_THRESH_VALUE(pkts_coal) |
				  SAM_RING_TIMEOUT_VAL(timeout);

	pr_debug("pkts = %u, usecs = %u, rdr_thresh_val = 0x%08x\n",
		 pkts_coal, usec_coal, hw_ring->rdr_thresh_val);

	return 0;
}

static bool sam_hw_ring_is_busy(struct sam_hw_ring *hw_ring)
{
	u32 cdr_lo, rdr_lo;

	cdr_lo = sam_hw_reg_read(hw_ring->regs_vbase, HIA_CDR_RING_BASE_ADDR_LO_REG);
	rdr_lo = sam_hw_reg_read(hw_ring->regs_vbase, HIA_RDR_RING_BASE_ADDR_LO_REG);

	if (cdr_lo || rdr_lo)
		return true;

	return false;
}

static int sam_hw_get_pes_num(struct sam_hw_device_info *device_info)
{
	int num_pes;
	u32 val32;

	val32 = sam_hw_reg_read(device_info->vaddr, SAM_EIP197_HIA_OPTIONS_REG);
	num_pes = SAM_N_PES_GET(val32);

	return num_pes;
}

int sam_hw_get_rings_num(u32 device, u32 *map)
{
	struct sam_hw_device_info *device_info = &sam_hw_device_info[device];

	if (device_info->rings_num == 0)
		sam_hw_device_exist(device);

	if (map)
		*map = device_info->available_rings;

	return device_info->rings_num;
}

bool sam_hw_device_exist(u32 device)
{
	int i, j, num = 0;
	u32 map = 0;
	char ring_name[16];
	struct sys_iomem_params params;
	struct sam_hw_device_info *device_info = &sam_hw_device_info[device];

	/* We support two HW types: eip197 or eip97 */
	params.type = SYS_IOMEM_T_UIO;
	params.index = -1;
	for (i = 0; i < ARRAY_SIZE(sam_supported_name); i++) {
		for (j = 0; j < SAM_HW_RING_NUM; j++) {
			snprintf(ring_name, sizeof(ring_name), "%s_%d:%d",
				 sam_supported_name[i], device, j);
			params.devname = ring_name;
			if (sys_iomem_exists(&params)) {
				map |= BIT(j);
				num++;
			}
		}
	}
	if (num)
		pr_info("%s: device #%d: rings: num = %d, map = 0x%x\n",
			__func__, device, num, map);

	device_info->rings_num = num;
	device_info->available_rings = map;

	return (num != 0);
}

void sam_hw_reg_print(char *reg_name, void *base, u32 offset)
{
	void *addr = base + offset;

	pr_info("%-32s: 0x%06x = 0x%08x\n", reg_name, offset, readl(addr));
}

int sam_hw_cdr_regs_reset(struct sam_hw_ring *hw_ring)
{
	u32 val32;

	/* Clear CDR count */
	val32 = SAM_RING_COUNT_CLEAR_MASK;
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
	sam_hw_reg_write(hw_ring->regs_vbase, HIA_CDR_STAT_REG, SAM_CDR_STAT_IRQ_MASK);

	return 0;
}

int sam_hw_cdr_regs_init(struct sam_hw_ring *hw_ring)
{
	u32 val32, desc_size;

	val32 = lower_32_bits(hw_ring->cdr_buf.paddr);
	sam_hw_reg_write(hw_ring->regs_vbase, HIA_CDR_RING_BASE_ADDR_LO_REG, val32);

	val32 = upper_32_bits(hw_ring->cdr_buf.paddr);
	sam_hw_reg_write(hw_ring->regs_vbase, HIA_CDR_RING_BASE_ADDR_HI_REG, val32);

	/* ring_size in words */
	val32 = SAM_RING_SIZE_VAL(hw_ring->ring_size * SAM_CDR_ENTRY_WORDS);
	sam_hw_reg_write(hw_ring->regs_vbase, HIA_CDR_RING_SIZE_REG, val32);

	if (hw_ring->type == HW_EIP197)
		desc_size = SAM_RING_DESC_SIZE_VAL(SAM_CDR_DSCR_EXT_WORD_COUNT); /* Extended command descriptor */
	else
		desc_size = SAM_RING_DESC_SIZE_VAL(SAM_CDR_DSCR_WORD_COUNT); /* Basic Command descriptor */

	val32 = desc_size;
	val32 |= SAM_RING_DESC_OFFSET_VAL(SAM_CDR_ENTRY_WORDS); /* distance between descriptors */
	val32 |= SAM_RING_TOKEN_PTR_MASK; /* acdp_present (Token pointer in the descriptor) */
	val32 |= SAM_RING_64B_MODE_MASK; /* 64 bits mode */
	sam_hw_reg_write(hw_ring->regs_vbase, HIA_CDR_DESC_SIZE_REG, val32);

	/* Number of words to fetch */
	val32 = SAM_RING_FETCH_SIZE_VAL(SAM_CDR_ENTRY_WORDS * SAM_CDR_FETCH_COUNT);
	/* Threshold in words to start fetch */
	val32 |= SAM_RING_FETCH_THRESH_VAL(desc_size * SAM_CDR_FETCH_COUNT);
	sam_hw_reg_write(hw_ring->regs_vbase, HIA_CDR_CFG_REG, val32);

	val32 = SAM_RING_DESC_SWAP_VAL(CONF_DESC_SWAP_VALUE);      /* cd_swap */
	val32 |= SAM_RING_DESC_PROTECT_VAL(CONF_DESC_PROT_VALUE);  /* cd_prot */
	val32 |= SAM_RING_DATA_SWAP_VAL(CONF_DATA_SWAP_VALUE);     /* data_swap*/
	val32 |= SAM_RING_DATA_PROTECT_VAL(CONF_DATA_PROT_VALUE);  /* data_prot */
	val32 |= SAM_RING_ACD_SWAP_VAL(CONF_TOKEN_SWAP_VALUE);     /* acd_swap (token) */
	val32 |= SAM_RING_ACD_PROTECT_VAL(CONF_TOKEN_PROT_VALUE);  /* acd_prot (token) */
	val32 |= SAM_RING_OWN_BUF_EN_MASK; /* wr_own_buf */
	val32 |= SAM_RING_WRITE_CACHE_VAL(CONF_WRITE_CACHE_CTRL);  /* wr_cache */
	val32 |= SAM_RING_READ_CACHE_VAL(CONF_READ_CACHE_CTRL);    /* rd_cache */
	sam_hw_reg_write(hw_ring->regs_vbase, HIA_CDR_DMA_CFG_REG, val32);

	return 0;
}

int sam_hw_rdr_regs_reset(struct sam_hw_ring *hw_ring)
{
	u32 val32;

	/* Disable the interrupt and clear the ring interrupt */
	sam_hw_reg_write(hw_ring->aic_regs_vbase, SAM_AIC_RING_ENABLE_CLR_REG,
			 SAM_AIC_RING_ALL_IRQS_MASK(hw_ring->ring));

	/* Clear RDR count */
	val32 = SAM_RING_COUNT_CLEAR_MASK;
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

	sam_hw_reg_write(hw_ring->regs_vbase, HIA_RDR_THRESH_REG, 0);

	/* Clear and disable all RDR interrupts */
	sam_hw_reg_write(hw_ring->regs_vbase, HIA_RDR_STAT_REG, SAM_RDR_STAT_IRQ_MASK);

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
	val32 = SAM_RING_SIZE_VAL(hw_ring->ring_size * SAM_RDR_ENTRY_WORDS);
	sam_hw_reg_write(hw_ring->regs_vbase, HIA_RDR_RING_SIZE_REG, val32);

	if (hw_ring->type == HW_EIP197)
		val32 = SAM_RING_DESC_SIZE_VAL(SAM_RDR_DSCR_EXT_WORD_COUNT); /* Extended Command descriptor */
	else
		val32 = SAM_RING_DESC_SIZE_VAL(SAM_RDR_DSCR_WORD_COUNT); /* Basic Command descriptor */

	val32 |= SAM_RING_DESC_OFFSET_VAL(SAM_RDR_ENTRY_WORDS); /* distance between descriptors */
	val32 |= SAM_RING_64B_MODE_MASK; /* 64 bits mode */
	sam_hw_reg_write(hw_ring->regs_vbase, HIA_RDR_DESC_SIZE_REG, val32);

	/* Number of words to fetch */
	val32 = SAM_RING_FETCH_SIZE_VAL(SAM_RDR_ENTRY_WORDS * SAM_RDR_FETCH_COUNT);
	/* Threshold in words to start fetch */
	val32 |= SAM_RING_FETCH_THRESH_VAL(SAM_RDR_PREP_WORDS * SAM_RDR_FETCH_COUNT);
	sam_hw_reg_write(hw_ring->regs_vbase, HIA_RDR_CFG_REG, val32);

	val32 = SAM_RING_DESC_SWAP_VAL(CONF_DESC_SWAP_VALUE);      /* rd_swap */
	val32 |= SAM_RING_DESC_PROTECT_VAL(CONF_DESC_PROT_VALUE);  /* cd_prot */
	val32 |= SAM_RING_DATA_SWAP_VAL(CONF_DATA_SWAP_VALUE);     /* data_swap*/
	val32 |= SAM_RING_DATA_PROTECT_VAL(CONF_DESC_PROT_VALUE);  /* data_prot */
	/* bit[22] - wr_res_buf */
	/* bit[23] - wr_ctrl_buf */
	val32 |= SAM_RING_OWN_BUF_EN_MASK; /* wr_own_buf */
	val32 |= SAM_RING_WRITE_CACHE_VAL(CONF_WRITE_CACHE_CTRL);  /* wr_cache */
	/* bit[28] - SAM_RING_PAD_TO_OFFSET_MASK; */
	val32 |= SAM_RING_READ_CACHE_VAL(CONF_READ_CACHE_CTRL);   /* rd_cache */
	sam_hw_reg_write(hw_ring->regs_vbase, HIA_RDR_DMA_CFG_REG, val32);

	return 0;
}

int sam_hw_ring_init(u32 device, u32 ring, struct sam_cio_params *params,
		     struct sam_hw_ring *hw_ring)
{
	u32 ring_size;
	int rc, nr_pes = 1;
	struct sam_hw_device_info *device_info = &sam_hw_device_info[device];

	if (!device_info->iomem_info) {
		device_info->name = "regs";
		device_info->iomem_info = sam_iomem_init(device, &device_info->type);
		if (!device_info->iomem_info) {
			pr_err("Can't init IOMEM area for device #%d\n", device);
			return -EINVAL;
		}
		rc = sys_iomem_map(device_info->iomem_info, device_info->name,
				   &device_info->paddr, &device_info->vaddr);
		if (rc) {
			pr_err("Can't map %s IOMEM area for device #%d, rc = %d\n",
				device_info->name, device, rc);
			sys_iomem_deinit(device_info->iomem_info);
			device_info->iomem_info = NULL;
			return -ENOMEM;
		}
		nr_pes = sam_hw_get_pes_num(device_info);
		if (nr_pes == 1)
			SABuilderLib_SetFwVersion(SAB_FW_VER_2_4);
		else
			SABuilderLib_SetFwVersion(SAB_FW_VER_2_7);
	}
	if (device_info->active_rings & BIT(ring)) {
		pr_err("device #%d: Ring #%d is busy. Active rings map is 0x%x\n",
			device, ring, device_info->active_rings);
		return -EBUSY;
	}
	device_info->active_rings |= BIT(ring);

	/* Add 1 to ring size for lockless ring management */
	params->size += 1;

	hw_ring->device = device;
	hw_ring->type = device_info->type;
	hw_ring->ring = ring;
	hw_ring->ring_size = params->size; /* number of descriptors in the ring */

	if (device_info->type == HW_EIP197) {
		hw_ring->aic_regs_vbase = (((char *)device_info->vaddr) + SAM_EIP197_AIC_RING_REGS_OFFS(ring));
		hw_ring->aic_regs_paddr = device_info->paddr + SAM_EIP197_AIC_RING_REGS_OFFS(ring);
		hw_ring->regs_vbase = (((char *)device_info->vaddr) + SAM_EIP197_RING_REGS_OFFS(ring));
		hw_ring->paddr = device_info->paddr + SAM_EIP197_RING_REGS_OFFS(ring);
	} else if (device_info->type == HW_EIP97) {
		hw_ring->aic_regs_vbase = (((char *)device_info->vaddr) + SAM_EIP97_AIC_RING_REGS_OFFS(ring));
		hw_ring->aic_regs_paddr = device_info->paddr + SAM_EIP97_AIC_RING_REGS_OFFS(ring);
		hw_ring->regs_vbase = (((char *)device_info->vaddr) + SAM_EIP97_RING_REGS_OFFS(ring));
		hw_ring->paddr = device_info->paddr + SAM_EIP97_RING_REGS_OFFS(ring);
	} else {
		pr_err("%s: Unexpected HW type = %d\n", __func__, device_info->type);
		rc = -EINVAL;
		goto err;
	}
/*
	if (sam_hw_ring_is_busy(hw_ring)) {
		pr_err("%s: %d:%d is busy\n",
			(device_info->type == HW_EIP197) ? "eip197" : "eip97", device, ring);
		goto err;
	}
*/
	pr_debug("%s: %d:%d registers: paddr: 0x%x, vaddr: 0x%p\n",
		(device_info->type == HW_EIP197) ? "eip197" : "eip97",
		device, ring, (unsigned)hw_ring->paddr, hw_ring->regs_vbase);

	sam_hw_cdr_regs_reset(hw_ring);
	sam_hw_rdr_regs_reset(hw_ring);

	/* Allocate command descriptors ring */
	ring_size = SAM_CDR_ENTRY_WORDS * sizeof(u32) * params->size;
	rc = sam_dma_buf_alloc(ring_size, &hw_ring->cdr_buf);
	if (rc)
		goto err;

	pr_debug("DMA buffer (%d bytes) for CDR #%d allocated: paddr = 0x%" PRIdma" , vaddr = %p\n",
		ring_size, ring, hw_ring->cdr_buf.paddr, hw_ring->cdr_buf.vaddr);

	memset(hw_ring->cdr_buf.vaddr, 0, ring_size);
	hw_ring->cmd_desc_first = hw_ring->cdr_buf.vaddr;

	/* Init CDR registers */
	sam_hw_cdr_regs_init(hw_ring);

	/* Allocate result descriptors ring */
	ring_size = SAM_RDR_ENTRY_WORDS * sizeof(u32) * params->size;
	rc = sam_dma_buf_alloc(ring_size, &hw_ring->rdr_buf);
	if (rc)
		goto err;

	pr_debug("DMA buffer (%d bytes) for RDR #%d allocated: paddr = 0x%" PRIdma", vaddr = %p\n",
		ring_size, ring, hw_ring->rdr_buf.paddr, hw_ring->rdr_buf.vaddr);

	memset(hw_ring->rdr_buf.vaddr, 0, ring_size);
	hw_ring->res_desc_first = hw_ring->rdr_buf.vaddr;

	/* Init RDR registers */
	sam_hw_rdr_regs_init(hw_ring);

	return 0;
err:
	sam_hw_ring_deinit(hw_ring);
	return rc;
}

int sam_hw_ring_deinit(struct sam_hw_ring *hw_ring)
{
	struct sam_hw_device_info *device_info = &sam_hw_device_info[hw_ring->device];

	if (hw_ring->regs_vbase) {
		sam_hw_cdr_regs_reset(hw_ring);
		sam_hw_rdr_regs_reset(hw_ring);
	}

	sam_dma_buf_free(&hw_ring->cdr_buf);
	sam_dma_buf_free(&hw_ring->rdr_buf);

	device_info->active_rings &= ~BIT(hw_ring->ring);
	if (device_info->active_rings == 0) {
		sys_iomem_unmap(device_info->iomem_info, device_info->name);
		sys_iomem_deinit(device_info->iomem_info);
		device_info->iomem_info = NULL;
	}
	return 0;
}

void sam_hw_aic_ring_regs_show(struct sam_hw_ring *hw_ring)
{
	pr_info("%d:%d AIC Ring registers - paddr = 0x%" PRIdma ", vaddr = %p\n",
		hw_ring->device, hw_ring->ring, hw_ring->aic_regs_paddr, hw_ring->aic_regs_vbase);

	sam_hw_reg_print("SAM_AIC_RING_ENABLE_REG", hw_ring->aic_regs_vbase, SAM_AIC_RING_ENABLE_REG);
	sam_hw_reg_print("SAM_AIC_RING_RAW_STAT_REG", hw_ring->aic_regs_vbase, SAM_AIC_RING_RAW_STAT_REG);
	sam_hw_reg_print("SAM_AIC_RING_STAT_ACK_REG", hw_ring->aic_regs_vbase, SAM_AIC_RING_STAT_ACK_REG);
}

void sam_hw_cdr_regs_show(struct sam_hw_ring *hw_ring)
{
	pr_info("%d:%d CDR registers - paddr = 0x%" PRIdma ", vaddr = %p\n",
		hw_ring->device, hw_ring->ring, hw_ring->paddr, hw_ring->regs_vbase);
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
	pr_info("%d:%d RDR registers - paddr = 0x%" PRIdma ", vaddr = %p\n",
		hw_ring->device, hw_ring->ring, hw_ring->paddr, hw_ring->regs_vbase);
	sam_hw_reg_print("HIA_RDR_RING_BASE_ADDR_LO_REG", hw_ring->regs_vbase, HIA_RDR_RING_BASE_ADDR_LO_REG);
	sam_hw_reg_print("HIA_RDR_RING_SIZE_REG", hw_ring->regs_vbase, HIA_RDR_RING_SIZE_REG);
	sam_hw_reg_print("HIA_RDR_DESC_SIZE_REG", hw_ring->regs_vbase, HIA_RDR_DESC_SIZE_REG);
	sam_hw_reg_print("HIA_RDR_CFG_REG", hw_ring->regs_vbase, HIA_RDR_CFG_REG);
	sam_hw_reg_print("HIA_RDR_DMA_CFG_REG", hw_ring->regs_vbase, HIA_RDR_DMA_CFG_REG);
	sam_hw_reg_print("HIA_RDR_THRESH_REG", hw_ring->regs_vbase, HIA_RDR_THRESH_REG);
	sam_hw_reg_print("HIA_RDR_STAT_REG", hw_ring->regs_vbase, HIA_RDR_STAT_REG);
}
