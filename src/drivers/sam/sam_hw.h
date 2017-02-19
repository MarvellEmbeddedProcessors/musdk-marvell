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

#ifndef _SAM_HW_H_
#define _SAM_HW_H_

#include <drivers/mv_sam.h>
#include "std_internal.h"

#include "cs_driver.h"
#include "api_pec.h"
#include "api_dmabuf.h"
#include "api_driver197_init.h"
#include "sa_builder.h"
#include "sa_builder_basic.h"
#include "token_builder.h"
#include "firmware_eip207_api_cmd.h"
#include "device_types.h"
#include "device_mgmt.h"

#define SAM_64BIT_DEVICE
#define SAM_USE_EXTENDED_DESCRIPTOR

#define SAM_HW_ENGINE_NUM	2
#define SAM_HW_RING_NUM		DRIVER_MAX_NOF_RING_TO_USE
#define SAM_HW_RING_SIZE	DRIVER_PEC_MAX_PACKETS
#define SAM_HW_SA_NUM		DRIVER_PEC_MAX_SAS

/*#define SAM_REG_READ_DEBUG*/
/*#define SAM_REG_WRITE_DEBUG*/
/*#define SAM_DMA_READ_DEBUG*/
/*#define SAM_DMA_WRITE_DEBUG*/

/* Command amd Result descriptors size and offset (in 32-bit words) */
#ifdef SAM_64BIT_DEVICE

#ifdef SAM_USE_EXTENDED_DESCRIPTOR

#define SAM_CDR_DSCR_MAX_WORD_COUNT    12
#define SAM_RDR_DSCR_MAX_WORD_COUNT    12

#define SAM_CDR_ENTRY_WORDS            16
#define SAM_RDR_ENTRY_WORDS            16

#else /* !SAM_USE_EXTENDED_DESCRIPTOR */

#define SAM_CDR_DSCR_MAX_WORD_COUNT    10
#define SAM_RDR_DSCR_MAX_WORD_COUNT    10

#define SAM_CDR_ENTRY_WORDS             8
#define SAM_RDR_ENTRY_WORDS             8

#endif /* SAM_USE_EXTENDED_DESCRIPTOR */

#else /* !SAM_64BIT_DEVICE */

#error "Only 64BIT device is supported"

#endif /* SAM_64BIT_DEVICE */

/* MRVL Specific. Bit - 47 (64-bit) set indicates that address is a DRAM address */
#ifdef MV_SOC_A39X
#define SAM_DRAM_SRC_MASK       (0x8000)
#else
#define SAM_DRAM_SRC_MASK       (0x0)
#endif /* MV_SOC_A39X */

/* CDR and RDR Registers addresses and fields */
#define HIA_RDR_REGS_OFFSET		0x800

#define HIA_CDR_RING_BASE_ADDR_LO_REG	0x0
#define HIA_CDR_RING_BASE_ADDR_HI_REG	0x4
/* Registers with offsets 0x8, 0xC, 0x10, 0x14 are not in use */
#define HIA_CDR_RING_SIZE_REG		0x18
#define HIA_CDR_DESC_SIZE_REG		0x1C
#define HIA_CDR_CFG_REG			0x20
#define HIA_CDR_DMA_CFG_REG		0x24
#define HIA_CDR_THRESH_REG		0x28
#define HIA_CDR_COUNT_REG		0x2C
#define HIA_CDR_PROC_COUNT_REG		0x30
#define HIA_CDR_PREP_PTR_REG		0x34
#define HIA_CDR_PROC_PTR_REG		0x38
#define HIA_CDR_STAT_REG		0x3C
#define HIA_CDR_OPTIONS_REG		0x7F8
#define HIA_CDR_VERSION_REG		0x7FC

#define HIA_RDR_RING_BASE_ADDR_LO_REG	(HIA_RDR_REGS_OFFSET + 0x0)
#define HIA_RDR_RING_BASE_ADDR_HI_REG	(HIA_RDR_REGS_OFFSET + 0x4)
/* Registers with offsets 0x8, 0xC, 0x10, 0x14 are not in use */
#define HIA_RDR_RING_SIZE_REG		(HIA_RDR_REGS_OFFSET + 0x18)
#define HIA_RDR_DESC_SIZE_REG		(HIA_RDR_REGS_OFFSET + 0x1C)
#define HIA_RDR_CFG_REG			(HIA_RDR_REGS_OFFSET + 0x20)
#define HIA_RDR_DMA_CFG_REG		(HIA_RDR_REGS_OFFSET + 0x24)
#define HIA_RDR_THRESH_REG		(HIA_RDR_REGS_OFFSET + 0x28)
#define HIA_RDR_PREP_COUNT_REG		(HIA_RDR_REGS_OFFSET + 0x2C)
#define HIA_RDR_PROC_COUNT_REG		(HIA_RDR_REGS_OFFSET + 0x30)
#define HIA_RDR_PREP_PTR_REG		(HIA_RDR_REGS_OFFSET + 0x34)
#define HIA_RDR_PROC_PTR_REG		(HIA_RDR_REGS_OFFSET + 0x38)
#define HIA_RDR_STAT_REG		(HIA_RDR_REGS_OFFSET + 0x3C)
#define HIA_RDR_OPTIONS_REG		(HIA_RDR_REGS_OFFSET + 0x7F8)
#define HIA_RDR_VERSION_REG		(HIA_RDR_REGS_OFFSET + 0x7FC)


#define SAM_RING_DESC_SWAP_VALUE	0
#define SAM_RING_DATA_SWAP_VALUE	0
#define SAM_RING_TOKEN_SWAP_VALUE	0
#define SAM_RING_DESC_PROT_VALUE	2
#define SAM_RING_DATA_PROT_VALUE	2
#define SAM_RING_TOKEN_PROT_VALUE	2
#define SAM_RING_WRITE_CACHE_CTRL	0x3 /* 0x7 >> 1 */
#define SAM_RING_READ_CACHE_CTRL	0x5 /* 0xB >> 1 */

struct sam_hw_cmd_desc {
	u32 words[SAM_CDR_ENTRY_WORDS];
};

struct sam_hw_res_desc {
	u32 words[SAM_RDR_ENTRY_WORDS];
};

struct sam_hw_ring_options {
};

struct sam_hw_ring {
	u32 engine;
	u32 ring;
	void *regs_vbase;			/* virtual address for ring registers */
	dma_addr_t paddr;			/* physical address for ring registers */
	u32 ring_size;                          /* CDR and RDR size in descriptors */
	struct sam_buf_info cdr_buf;            /* DMA memory buffer allocated for command descriptors */
	struct sam_buf_info rdr_buf;            /* DMA memory buffer allocated for result descriptors */
	struct sam_hw_cmd_desc *cmd_desc_first;	/* Pointer to first command descriptors in DMA memory */
	struct sam_hw_res_desc *res_desc_first;	/* Pointer to first command descriptors in DMA memory */
	PEC_CommandDescriptor_t *cmd_desc;	/* Array of command descriptors in */
	PEC_ResultDescriptor_t *result_desc;	/* Array of result descriptors */
};

static inline void sam_hw_reg_write(void *base, u32 offset, u32 data)
{
	void *addr = base + offset;

	writel(data, addr);

#ifdef SAM_REG_WRITE_DEBUG
	pr_info("sam_reg_write: %8p + 0x%04x = 0x%x\n", base, offset, data);
#endif
}

static inline u32 sam_hw_reg_read(void *base, u32 offset)
{
	void *addr = base + offset;
	u32 data;

	data = readl(addr);

#ifdef SAM_REG_READ_DEBUG
	pr_info("sam_reg_read : %8p + 0x%04x = 0x%x\n", base, offset, data);
#endif

	return data;
}

static inline struct sam_hw_cmd_desc *sam_hw_cmd_desc_get(struct sam_hw_ring *hw_ring, u32 idx)
{
	return (hw_ring->cmd_desc_first + idx);
}

static inline struct sam_hw_res_desc *sam_hw_res_desc_get(struct sam_hw_ring *hw_ring, u32 idx)
{
	return (hw_ring->res_desc_first + idx);
}

static inline void sam_hw_ring_desc_write(struct sam_hw_cmd_desc *cmd_desc, struct sam_hw_res_desc *res_desc,
				     PEC_CommandDescriptor_t *cmd)
{
	u32 ctrl_word, token_header;

	/* Write RDR descriptor first */
	ctrl_word = (2048 & MASK_20_BITS); /* ToFix: PrepSegmentByteCount */
	/* bits[24-31] - ExpectedResultWordCount = 0 */
	ctrl_word |= BIT_22 | BIT_23;  /* Last and First */

	res_desc->words[0] = ctrl_word;
	res_desc->words[1] = 0; /* skip this write */

	/* Write Destination Packet Data address */
	res_desc->words[2] = lower_32_bits((u64)cmd->DstPkt_Handle.p);
	res_desc->words[3] = upper_32_bits((u64)cmd->DstPkt_Handle.p) | SAM_DRAM_SRC_MASK;

#ifdef SAM_DMA_WRITE_DEBUG
	pr_info("RDR_Write32: 0x%8p + %2d * 4 = 0x%08x\n", res_desc->words, 0, res_desc->words[0]);
	pr_info("RDR_Write32: 0x%8p + %2d * 4 = 0x%08x\n", res_desc->words, 1, res_desc->words[1]);
	pr_info("RDR_Write32: 0x%8p + %2d * 4 = 0x%08x\n", res_desc->words, 2, res_desc->words[2]);
	pr_info("RDR_Write32: 0x%8p + %2d * 4 = 0x%08x\n", res_desc->words, 3, res_desc->words[3]);
#endif /* SAM_DMA_WRITE_DEBUG */

	/* Write CDR descriptor */
	ctrl_word = (cmd->SrcPkt_ByteCount & MASK_20_BITS);
	ctrl_word |= (cmd->Token_WordCount & MASK_8_BITS) << 24;
	ctrl_word |= BIT_22 | BIT_23;  /* Last and First */

	cmd_desc->words[0] = ctrl_word;
	cmd_desc->words[1] = 0; /* skip this write */

	/* Write Source Packet Data address */
	cmd_desc->words[2] = lower_32_bits((u64)cmd->SrcPkt_Handle.p);
	cmd_desc->words[3] = upper_32_bits((u64)cmd->SrcPkt_Handle.p) | SAM_DRAM_SRC_MASK;

	/* Write Token Data address */
	cmd_desc->words[4] = lower_32_bits((u64)cmd->Token_Handle.p);
	cmd_desc->words[5] = upper_32_bits((u64)cmd->Token_Handle.p) | SAM_DRAM_SRC_MASK;

	/* Token header word is provided as separate parameter in Control1. */
	token_header = cmd->Control1 & ~0x00300000;
	token_header |= BIT_17; /* Set token header format to EIP97. */

#ifdef SAM_USE_EXTENDED_DESCRIPTOR
	token_header |= BIT_30;
#endif

#ifdef SAM_64BIT_DEVICE
	token_header |= BIT_18; /* Set 64-bit Context (SA) pointer */
#endif
	/* Enable Context Reuse auto detect if no new SA */
	if (!cmd->User_p)
		token_header |= 0x00200000;

	cmd_desc->words[6] = token_header;

	/* EIP202_RING_ANTI_DMA_RACE_CONDITION_CDS - EIP202_DSCR_DONE_PATTERN */
	cmd_desc->words[7] = 0x0000ec00;

	cmd_desc->words[8] = lower_32_bits((u64)cmd->SA_Handle1.p) | 0x2;
	cmd_desc->words[9] = upper_32_bits((u64)cmd->SA_Handle1.p) | SAM_DRAM_SRC_MASK;

	cmd_desc->words[10] = cmd->Control2;
	cmd_desc->words[11] = cmd->Control3;

#ifdef SAM_DMA_WRITE_DEBUG
	pr_info("CDR_Write32: 0x%8p + %2d * 4 = 0x%08x\n", cmd_desc->words, 0, cmd_desc->words[0]);
	pr_info("CDR_Write32: 0x%8p + %2d * 4 = 0x%08x\n", cmd_desc->words, 1, cmd_desc->words[1]);
	pr_info("CDR_Write32: 0x%8p + %2d * 4 = 0x%08x\n", cmd_desc->words, 2, cmd_desc->words[2]);
	pr_info("CDR_Write32: 0x%8p + %2d * 4 = 0x%08x\n", cmd_desc->words, 3, cmd_desc->words[3]);
	pr_info("CDR_Write32: 0x%8p + %2d * 4 = 0x%08x\n", cmd_desc->words, 4, cmd_desc->words[4]);
	pr_info("CDR_Write32: 0x%8p + %2d * 4 = 0x%08x\n", cmd_desc->words, 5, cmd_desc->words[5]);
	pr_info("CDR_Write32: 0x%8p + %2d * 4 = 0x%08x\n", cmd_desc->words, 6, cmd_desc->words[6]);
	pr_info("CDR_Write32: 0x%8p + %2d * 4 = 0x%08x\n", cmd_desc->words, 7, cmd_desc->words[7]);
	pr_info("CDR_Write32: 0x%8p + %2d * 4 = 0x%08x\n", cmd_desc->words, 8, cmd_desc->words[8]);
	pr_info("CDR_Write32: 0x%8p + %2d * 4 = 0x%08x\n", cmd_desc->words, 9, cmd_desc->words[9]);
	pr_info("CDR_Write32: 0x%8p + %2d * 4 = 0x%08x\n", cmd_desc->words, 10, cmd_desc->words[10]);
	pr_info("CDR_Write32: 0x%8p + %2d * 4 = 0x%08x\n", cmd_desc->words, 11, cmd_desc->words[11]);
#endif /* SAM_DMA_WRITE_DEBUG */
}

static inline void sam_hw_res_desc_read(struct sam_hw_res_desc *res_desc, PEC_ResultDescriptor_t *res)
{
	u32 val32, control_word;

	/* Word 0 - Control Word */
	control_word = res_desc->words[0];

	/* Word 1 - extended length, not read. */

	/* Word 2 & 3 - Destination Packet Data Buffer Address */
	res->DstPkt_Handle.p = (void *)((u64)res_desc->words[2] | ((u64)res_desc->words[3] << 32));

	/* Words 4 ... 7 - Result Token Data - token_data[0] */
	val32 = res_desc->words[4];

	res->DstPkt_ByteCount = val32 & MASK_17_BITS;
	res->Status1 = val32;
	/* ErrorCode  = ((val32 >> 17) & MASK_15_BITS); */

	/* Copy the Buffer overflow (BIT_21) and Descriptor overflow (BIT_20) */
	/* EIP-202 error bits from the first EIP-202 result descriptor word */
	res->Status1 |= (control_word & (BIT_21 | BIT_20)) >> 6;

	/* token_data[1] */
	val32 = res_desc->words[5];
	res->Status6 = val32;
	res->Bypass_WordCount = val32 & MASK_4_BITS;
	/* HashByteCount = ((val32 >> 22) & MASK_6_BITS); */
	/* BCNL          = ((val32 >> 28) & MASK_4_BITS); */
	/* fE15          = (val32 & BIT_4) */
	/* fHash         = (val32 & BIT_21 */

	/* token_data[2] - not in use */
	/* val32 = res_desc->words[6]; */

	/* res->Status2 <- token_data[3] <- res_desc->words[7] */
	/* ((PadByteCount & MASK_8_BITS) << 8) | (NextHeader & MASK_8_BITS); */

	 /* token_data[3] <- res_desc->words[7] */
	res->Status3 = res_desc->words[7];

	res->Status4 = res_desc->words[10];
	res->Status5 = res_desc->words[11];
	/* res->Status7 and res->Status8 are not in use */

#ifdef SAM_DMA_READ_DEBUG
	pr_info("RDR_Read32: 0x%8p + %2d * 4 = 0x%08x\n", res_desc->words, 0, res_desc->words[0]);
	pr_info("RDR_Read32: 0x%8p + %2d * 4 = 0x%08x\n", res_desc->words, 1, res_desc->words[1]);
	pr_info("RDR_Read32: 0x%8p + %2d * 4 = 0x%08x\n", res_desc->words, 2, res_desc->words[2]);
	pr_info("RDR_Read32: 0x%8p + %2d * 4 = 0x%08x\n", res_desc->words, 3, res_desc->words[3]);
	pr_info("RDR_Read32: 0x%8p + %2d * 4 = 0x%08x\n", res_desc->words, 4, res_desc->words[4]);
	pr_info("RDR_Read32: 0x%8p + %2d * 4 = 0x%08x\n", res_desc->words, 5, res_desc->words[5]);
	pr_info("RDR_Read32: 0x%8p + %2d * 4 = 0x%08x\n", res_desc->words, 6, res_desc->words[6]);
	pr_info("RDR_Read32: 0x%8p + %2d * 4 = 0x%08x\n", res_desc->words, 7, res_desc->words[7]);
	pr_info("RDR_Read32: 0x%8p + %2d * 4 = 0x%08x\n", res_desc->words, 8, res_desc->words[8]);
	pr_info("RDR_Read32: 0x%8p + %2d * 4 = 0x%08x\n", res_desc->words, 9, res_desc->words[9]);
	pr_info("RDR_Read32: 0x%8p + %2d * 4 = 0x%08x\n", res_desc->words, 10, res_desc->words[10]);
	pr_info("RDR_Read32: 0x%8p + %2d * 4 = 0x%08x\n", res_desc->words, 11, res_desc->words[11]);
#endif /* SAM_DMA_READ_DEBUG */
}

static inline void sam_hw_ring_submit(struct sam_hw_ring *hw_ring, u32 todo)
{
	u32 val32;

	val32 = ((todo * SAM_RDR_ENTRY_WORDS) & MASK_14_BITS) << 2;
	sam_hw_reg_write(hw_ring->regs_vbase, HIA_RDR_PREP_COUNT_REG, val32);

	val32 = ((todo * SAM_CDR_ENTRY_WORDS) & MASK_14_BITS) << 2;
	sam_hw_reg_write(hw_ring->regs_vbase, HIA_CDR_COUNT_REG, val32);
}

static inline u32 sam_hw_ring_ready_get(struct sam_hw_ring *hw_ring)
{
	u32 val32;

	val32 = sam_hw_reg_read(hw_ring->regs_vbase, HIA_RDR_PROC_COUNT_REG);

	return (val32 >> 24) & MASK_7_BITS;
}

static inline void sam_hw_ring_update(struct sam_hw_ring *hw_ring, u32 done)
{
	u32 val32;

	val32 = (done & MASK_7_BITS) << 24;
	val32 |= ((done * SAM_RDR_ENTRY_WORDS) & MASK_14_BITS) << 2;

	sam_hw_reg_write(hw_ring->regs_vbase, HIA_RDR_PROC_COUNT_REG, val32);
}

void sam_hw_cdr_regs_show(struct sam_hw_ring *hw_ring);
void sam_hw_rdr_regs_show(struct sam_hw_ring *hw_ring);
void sam_hw_reg_print(char *reg_name, void *base, u32 offset);
int sam_hw_cdr_regs_reset(struct sam_hw_ring *hw_ring);
int sam_hw_cdr_regs_init(struct sam_hw_ring *hw_ring);
int sam_hw_rdr_regs_reset(struct sam_hw_ring *hw_ring);
int sam_hw_rdr_regs_init(struct sam_hw_ring *hw_ring);
int sam_hw_ring_init(u32 engine, u32 ring, struct sam_cio_params *params,
		     struct sam_hw_ring *hw_ring);
int sam_hw_ring_deinit(struct sam_hw_ring *hw_ring);
int sam_hw_engine_load(void);
int sam_hw_engine_unload(void);

#endif /* _SAM_HW_H_ */
