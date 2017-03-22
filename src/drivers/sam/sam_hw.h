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

#ifdef SAM_EIP_DDK_HW_INIT
#include "api_driver197_init.h"
#endif

#include "sa_builder.h"
#include "sa_builder_basic.h"
#include "token_builder.h"

#define SAM_HW_ENGINE_NUM	2
#define SAM_HW_RING_NUM		4

/*#define SAM_REG_READ_DEBUG*/
/*#define SAM_REG_WRITE_DEBUG*/
/*#define SAM_DMA_READ_DEBUG*/
/*#define SAM_DMA_WRITE_DEBUG*/

/* Command amd Result descriptors size and offset (in 32-bit words) */
#define SAM_CDR_ENTRY_WORDS            16
#define SAM_RDR_ENTRY_WORDS            16

#define SAM_CDR_DSCR_EXT_WORD_COUNT    12
#define SAM_RDR_DSCR_EXT_WORD_COUNT    12

#define SAM_CDR_DSCR_WORD_COUNT        10
#define SAM_RDR_DSCR_WORD_COUNT        10

#define SAM_EIP197_RING_REGS_OFFS(ring) (0x80000 + (ring) * 0x1000)
#define SAM_EIP97_RING_REGS_OFFS(ring)  (0x0 + (ring) * 0x1000)

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

/* Marvell specific configuration values for AXI3 */
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

/* CDR and RDR descriptors fields definition */

/* Control word #0 */
/* Segment size in bytes for Command and Result descriptors */
#define SAM_DESC_SEG_BYTES_OFFS		0
#define SAM_DESC_SEG_BYTES_BITS		17
#define SAM_DESC_SEG_BYTES_MASK		BIT_MASK(SAM_DESC_SEG_BYTES_BITS)

/* First and Last segments indication for Command and Result descriptors */
#define SAM_DESC_LAST_SEG_MASK		BIT(22)
#define SAM_DESC_FIRST_SEG_MASK		BIT(23)

/* Token size in 32bit words for first segment of Command descriptor */
#define SAM_CDR_TOKEN_BYTES_OFFS	24
#define SAM_CDR_TOKEN_BYTES_BITS	8
#define SAM_CDR_TOKEN_BYTES_MASK	BIT_MASK(SAM_CDR_TOKEN_BYTES_BITS)

/* Token header word #6 */

/* Segment size in bytes for Command and Result descriptors */
#define SAM_TOKEN_INPUT_PKT_LEN_OFFS	0
#define SAM_TOKEN_INPUT_PKT_LEN_BITS	17
#define SAM_TOKEN_INPUT_PKT_LEN_MASK	BIT_MASK(SAM_TOKEN_INPUT_PKT_LEN_BITS)

#define SAM_TOKEN_IP_EIP97_MASK		BIT(17)
#define SAM_TOKEN_CP_64B_MASK		BIT(18)

#define SAM_TOKEN_REUSE_CONTEXT_OFFS	20
#define SAM_TOKEN_REUSE_CONTEXT_MASK	(0x3 << SAM_TOKEN_REUSE_CONTEXT_OFFS)
#define SAM_TOKEN_REUSE_NO_MASK		(0x0 << SAM_TOKEN_REUSE_CONTEXT_OFFS)
#define SAM_TOKEN_REUSE_YES_MASK	(0x1 << SAM_TOKEN_REUSE_CONTEXT_OFFS)
#define SAM_TOKEN_REUSE_AUTO_MASK	(0x2 << SAM_TOKEN_REUSE_CONTEXT_OFFS)
#define SAM_TOKEN_REUSE_FORCE_MASK	(0x3 << SAM_TOKEN_REUSE_CONTEXT_OFFS)

/* Token Type */
#define SAM_TOKEN_TYPE_OFFS		30
#define SAM_TOKEN_TYPE_MASK		(0x3 << SAM_TOKEN_TYPE_OFFS)
#define SAM_TOKEN_TYPE_BASIC_MASK	(0x0 << SAM_TOKEN_TYPE_OFFS)
#define SAM_TOKEN_TYPE_EXTENDED_MASK	(0x1 << SAM_TOKEN_TYPE_OFFS)
#define SAM_TOKEN_TYPE_AUTO_MASK	(0x3 << SAM_TOKEN_TYPE_OFFS)

/* HW Services word #11 */
#define FIRMWARE_HW_SERVICES_OFFS	24
#define FIRMWARE_HW_SERVICES_BITS	6
#define FIRMWARE_HW_SERVICES_ALL_MASK	BIT_MASK_OFFS(FIRMWARE_HW_SERVICES_OFFS, FIRMWARE_HW_SERVICES_BITS)

/* Lookaside Crypto packet Flow */
#define FIRMWARE_CMD_PKT_LAC_MASK	(0x04 << FIRMWARE_HW_SERVICES_OFFS)

/* Invalidate Transform Record command */
#define FIRMWARE_CMD_INV_TR_MASK	(0x06 << FIRMWARE_HW_SERVICES_OFFS)

enum sam_hw_type {
	HW_EIP197,
	HW_EIP97,
	HW_TYPE_LAST
};

struct sam_hw_engine_info {
	struct uio_info_t *uio_info;
	enum sam_hw_type type;
};

struct sam_hw_ring {
	u32 engine;
	u32 ring;
	void *regs_vbase;			/* virtual address for ring registers */
	dma_addr_t paddr;			/* physical address for ring registers */
	enum sam_hw_type type;			/* EIP197 or EIP97 */
	u32 ring_size;                          /* CDR and RDR size in descriptors */
	struct sam_buf_info cdr_buf;            /* DMA memory buffer allocated for command descriptors */
	struct sam_buf_info rdr_buf;            /* DMA memory buffer allocated for result descriptors */
	struct sam_hw_cmd_desc *cmd_desc_first;	/* Pointer to first command descriptors in DMA memory */
	struct sam_hw_res_desc *res_desc_first;	/* Pointer to first command descriptors in DMA memory */
};

static inline void sam_hw_reg_write_relaxed(void *base, u32 offset, u32 data)
{
	void *addr = base + offset;

	writel_relaxed(data, addr);

#ifdef SAM_REG_WRITE_DEBUG
	pr_info("sam_reg_write: %8p + 0x%04x = 0x%x\n", base, offset, data);
#endif
}

static inline u32 sam_hw_reg_read_relaxed(void *base, u32 offset)
{
	void *addr = base + offset;
	u32 data;

	data = readl_relaxed(addr);

#ifdef SAM_REG_READ_DEBUG
	pr_info("sam_reg_read : %8p + 0x%04x = 0x%x\n", base, offset, data);
#endif

	return data;
}

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

static inline void sam_hw_rdr_prep_desc_write(struct sam_hw_res_desc *res_desc,
					      dma_addr_t dst_paddr, u32 prep_size)
{
	u32 ctrl_word;

	/* bits[24-31] - ExpectedResultWordCount = 0 */
	ctrl_word = (SAM_DESC_LAST_SEG_MASK | SAM_DESC_FIRST_SEG_MASK);  /* Last and First */

	ctrl_word |= (prep_size & SAM_DESC_SEG_BYTES_MASK);

	writel_relaxed(ctrl_word, &res_desc->words[0]);
	writel_relaxed(0, &res_desc->words[1]); /* skip this write */

	/* Write Destination Packet Data address */
	writel_relaxed(lower_32_bits(dst_paddr), &res_desc->words[2]);
	writel_relaxed(upper_32_bits(dst_paddr), &res_desc->words[3]);

#ifdef SAM_DMA_WRITE_DEBUG
	pr_info("RDR_Write32: 0x%8p + %2d * 4 = 0x%08x\n", res_desc, 0, readl_relaxed(&res_desc->words[0]));
	pr_info("RDR_Write32: 0x%8p + %2d * 4 = 0x%08x\n", res_desc, 1, readl_relaxed(&res_desc->words[1]));
	pr_info("RDR_Write32: 0x%8p + %2d * 4 = 0x%08x\n", res_desc, 2, readl_relaxed(&res_desc->words[2]));
	pr_info("RDR_Write32: 0x%8p + %2d * 4 = 0x%08x\n", res_desc, 3, readl_relaxed(&res_desc->words[3]));
#endif /* SAM_DMA_WRITE_DEBUG */
}

static inline void sam_hw_cdr_cmd_desc_write(struct sam_hw_cmd_desc *cmd_desc,
					     dma_addr_t src_paddr, u32 data_bytes,
					     dma_addr_t token_paddr, u32 token_words)
{
	u32 ctrl_word;

	ctrl_word = (data_bytes & SAM_DESC_SEG_BYTES_MASK);
	ctrl_word |= (token_words & SAM_CDR_TOKEN_BYTES_MASK) << SAM_CDR_TOKEN_BYTES_OFFS;
	ctrl_word |= (SAM_DESC_LAST_SEG_MASK | SAM_DESC_FIRST_SEG_MASK);  /* Last and First */

	writel_relaxed(ctrl_word, &cmd_desc->words[0]);
	writel_relaxed(0, &cmd_desc->words[1]); /* skip this write */

	/* Write Source Packet Data address */
	writel_relaxed(lower_32_bits(src_paddr), &cmd_desc->words[2]);
	writel_relaxed(upper_32_bits(src_paddr), &cmd_desc->words[3]);

	/* Write Token Data address */
	writel_relaxed(lower_32_bits(token_paddr), &cmd_desc->words[4]);
	writel_relaxed(upper_32_bits(token_paddr), &cmd_desc->words[5]);

#ifdef SAM_DMA_WRITE_DEBUG
	pr_info("CDR_Write32: 0x%8p + %2d * 4 = 0x%08x\n", cmd_desc, 0, readl_relaxed(&cmd_desc->words[0]));
	pr_info("CDR_Write32: 0x%8p + %2d * 4 = 0x%08x\n", cmd_desc, 1, readl_relaxed(&cmd_desc->words[1]));
	pr_info("CDR_Write32: 0x%8p + %2d * 4 = 0x%08x\n", cmd_desc, 2, readl_relaxed(&cmd_desc->words[2]));
	pr_info("CDR_Write32: 0x%8p + %2d * 4 = 0x%08x\n", cmd_desc, 3, readl_relaxed(&cmd_desc->words[3]));
	pr_info("CDR_Write32: 0x%8p + %2d * 4 = 0x%08x\n", cmd_desc, 4, readl_relaxed(&cmd_desc->words[4]));
	pr_info("CDR_Write32: 0x%8p + %2d * 4 = 0x%08x\n", cmd_desc, 5, readl_relaxed(&cmd_desc->words[5]));
#endif /* SAM_DMA_WRITE_DEBUG */
}

static inline void sam_hw_ring_basic_desc_write(struct sam_hw_ring *hw_ring, int next_request,
					PEC_CommandDescriptor_t *cmd)
{
	u32 token_header, val32;
	struct sam_hw_cmd_desc *cmd_desc = sam_hw_cmd_desc_get(hw_ring, next_request);
	struct sam_hw_res_desc *res_desc = sam_hw_res_desc_get(hw_ring, next_request);

	/* Write prepared RDR descriptor first */
	sam_hw_rdr_prep_desc_write(res_desc, (dma_addr_t)cmd->DstPkt_Handle.p, cmd->SrcPkt_ByteCount + 64);

	/* Write CDR descriptor */
	sam_hw_cdr_cmd_desc_write(cmd_desc, (dma_addr_t)cmd->SrcPkt_Handle.p, cmd->SrcPkt_ByteCount,
				  (dma_addr_t)cmd->Token_Handle.p, cmd->Token_WordCount);


	/* Token header word is provided as separate parameter in Control1. */

	/* Set 64-bit Context (SA) pointer and IP EIP97 */
	token_header = (cmd->Control1 | SAM_TOKEN_CP_64B_MASK | SAM_TOKEN_IP_EIP97_MASK);

	/* Enable Context Reuse auto detect if no new SA */
	token_header &= ~SAM_TOKEN_REUSE_CONTEXT_MASK;
	if (!cmd->User_p)
		token_header |= SAM_TOKEN_REUSE_AUTO_MASK;

	writel_relaxed(token_header, &cmd_desc->words[6]);

	/* EIP202_RING_ANTI_DMA_RACE_CONDITION_CDS - EIP202_DSCR_DONE_PATTERN */
	writel_relaxed(0x0000ec00, &cmd_desc->words[7]);

	val32 = lower_32_bits((u64)cmd->SA_Handle1.p);
	writel_relaxed(val32, &cmd_desc->words[8]);

	val32 = upper_32_bits((u64)cmd->SA_Handle1.p);
	writel_relaxed(val32, &cmd_desc->words[9]);

#ifdef SAM_DMA_WRITE_DEBUG
	pr_info("CDR_Write32: 0x%8p + %2d * 4 = 0x%08x\n", cmd_desc, 6, readl_relaxed(&cmd_desc->words[6]));
	pr_info("CDR_Write32: 0x%8p + %2d * 4 = 0x%08x\n", cmd_desc, 7, readl_relaxed(&cmd_desc->words[7]));
	pr_info("CDR_Write32: 0x%8p + %2d * 4 = 0x%08x\n", cmd_desc, 8, readl_relaxed(&cmd_desc->words[8]));
	pr_info("CDR_Write32: 0x%8p + %2d * 4 = 0x%08x\n", cmd_desc, 9, readl_relaxed(&cmd_desc->words[9]));
#endif /* SAM_DMA_WRITE_DEBUG */
}

static inline void sam_hw_ring_desc_write(struct sam_hw_ring *hw_ring, int next_request,
				     PEC_CommandDescriptor_t *cmd)
{
	u32 token_header, val32;
	struct sam_hw_cmd_desc *cmd_desc = sam_hw_cmd_desc_get(hw_ring, next_request);
	struct sam_hw_res_desc *res_desc = sam_hw_res_desc_get(hw_ring, next_request);

	/* Write prepared RDR descriptor first */
	sam_hw_rdr_prep_desc_write(res_desc, (dma_addr_t)cmd->DstPkt_Handle.p, cmd->SrcPkt_ByteCount + 64);

	/* Write CDR descriptor */
	sam_hw_cdr_cmd_desc_write(cmd_desc, (dma_addr_t)cmd->SrcPkt_Handle.p, cmd->SrcPkt_ByteCount,
				  (dma_addr_t)cmd->Token_Handle.p, cmd->Token_WordCount);

	/* Token header word is provided as separate parameter in Control1. */

	/* Set 64-bit Context (SA) pointer and IP EIP97 */
	token_header = (cmd->Control1 | SAM_TOKEN_CP_64B_MASK | SAM_TOKEN_IP_EIP97_MASK);

	token_header |= SAM_TOKEN_TYPE_EXTENDED_MASK;

	/* Enable Context Reuse auto detect if no new SA */
	token_header &= ~SAM_TOKEN_REUSE_CONTEXT_MASK;
	if (!cmd->User_p)
		token_header |= SAM_TOKEN_REUSE_AUTO_MASK;

	writel_relaxed(token_header, &cmd_desc->words[6]);

	/* EIP202_RING_ANTI_DMA_RACE_CONDITION_CDS - EIP202_DSCR_DONE_PATTERN */
	writel_relaxed(0x0000ec00, &cmd_desc->words[7]);

	val32 = lower_32_bits((u64)cmd->SA_Handle1.p);
	val32 |= 0x2;
	writel_relaxed(val32, &cmd_desc->words[8]);

	val32 = upper_32_bits((u64)cmd->SA_Handle1.p);
	writel_relaxed(val32, &cmd_desc->words[9]);

	writel_relaxed(FIRMWARE_CMD_PKT_LAC_MASK, &cmd_desc->words[10]);

	writel_relaxed(0, &cmd_desc->words[11]);

#ifdef SAM_DMA_WRITE_DEBUG
	pr_info("CDR_Write32: 0x%8p + %2d * 4 = 0x%08x\n", cmd_desc, 6, readl_relaxed(&cmd_desc->words[6]));
	pr_info("CDR_Write32: 0x%8p + %2d * 4 = 0x%08x\n", cmd_desc, 7, readl_relaxed(&cmd_desc->words[7]));
	pr_info("CDR_Write32: 0x%8p + %2d * 4 = 0x%08x\n", cmd_desc, 8, readl_relaxed(&cmd_desc->words[8]));
	pr_info("CDR_Write32: 0x%8p + %2d * 4 = 0x%08x\n", cmd_desc, 9, readl_relaxed(&cmd_desc->words[9]));
	pr_info("CDR_Write32: 0x%8p + %2d * 4 = 0x%08x\n", cmd_desc, 10, readl_relaxed(&cmd_desc->words[10]));
	pr_info("CDR_Write32: 0x%8p + %2d * 4 = 0x%08x\n", cmd_desc, 11, readl_relaxed(&cmd_desc->words[11]));
#endif /* SAM_DMA_WRITE_DEBUG */
}

static inline void sam_hw_ring_sa_inv_desc_write(struct sam_hw_ring *hw_ring, int next_request, dma_addr_t paddr)
{
	u32 token_header, val32, ctrl_word;
	struct sam_hw_cmd_desc *cmd_desc = sam_hw_cmd_desc_get(hw_ring, next_request);
	struct sam_hw_res_desc *res_desc = sam_hw_res_desc_get(hw_ring, next_request);

	ctrl_word = (SAM_DESC_LAST_SEG_MASK | SAM_DESC_FIRST_SEG_MASK);  /* Last and First */
	writel_relaxed(ctrl_word, &res_desc->words[0]);
	writel_relaxed(0, &res_desc->words[1]); /* skip this write */

	/* Write NULL to Destination Packet Data address */
	writel_relaxed(0, &res_desc->words[2]);
	writel_relaxed(0, &res_desc->words[3]);

	writel_relaxed(ctrl_word, &cmd_desc->words[0]);
	writel_relaxed(0, &cmd_desc->words[1]); /* skip this write */

	/* Write NULL as Source Packet Data address */
	writel_relaxed(0, &cmd_desc->words[2]);
	writel_relaxed(0, &cmd_desc->words[3]);

	/* Write NULL as Token Data address */
	writel_relaxed(0, &cmd_desc->words[4]);
	writel_relaxed(0, &cmd_desc->words[5]);

	/* Invalidate session in cache */
	token_header = SAM_TOKEN_TYPE_AUTO_MASK;
	writel_relaxed(token_header, &cmd_desc->words[6]);

	val32 = lower_32_bits((u64)paddr);
	val32 |= 0x2;
	writel_relaxed(val32, &cmd_desc->words[8]);

	val32 = upper_32_bits((u64)paddr);
	writel_relaxed(val32, &cmd_desc->words[9]);

	writel_relaxed(FIRMWARE_CMD_INV_TR_MASK, &cmd_desc->words[10]);
}

static inline void sam_hw_res_desc_read(struct sam_hw_res_desc *res_desc, PEC_ResultDescriptor_t *res)
{
	u32 val32, control_word;

	/* Word 0 - Control Word */
	control_word = readl_relaxed(&res_desc->words[0]);

	/* Word 1 - extended length, not read. */

	/* Word 2 & 3 - Destination Packet Data Buffer Address */
	res->DstPkt_Handle.p = (void *)((u64)readl_relaxed(&res_desc->words[2]) |
				       ((u64)readl_relaxed(&res_desc->words[3]) << 32));

	/* Words 4 ... 7 - Result Token Data - token_data[0] */
	val32 = readl_relaxed(&res_desc->words[4]);

	res->DstPkt_ByteCount = val32 & MASK_17_BITS;
	res->Status1 = val32;
	/* ErrorCode  = ((val32 >> 17) & MASK_15_BITS); */

	/* Copy the Buffer overflow (BIT_21) and Descriptor overflow (BIT_20) */
	/* EIP-202 error bits from the first EIP-202 result descriptor word */
	res->Status1 |= (control_word & (BIT_21 | BIT_20)) >> 6;

	/* token_data[1] */
	val32 = readl_relaxed(&res_desc->words[5]);
	res->Status6 = val32;
	res->Bypass_WordCount = val32 & MASK_4_BITS;
	/* HashByteCount = ((val32 >> 22) & MASK_6_BITS); */
	/* BCNL          = ((val32 >> 28) & MASK_4_BITS); */
	/* fE15          = (val32 & BIT_4) */
	/* fHash         = (val32 & BIT_21 */

	/* token_data[2] - not in use */
	/* val32 = readl_relaxed(&res_desc->words[6]); */

	/* res->Status2 <- token_data[3] <- res_desc->words[7] */
	/* ((PadByteCount & MASK_8_BITS) << 8) | (NextHeader & MASK_8_BITS); */

	 /* token_data[3] <- res_desc->words[7] */
	res->Status3 = readl_relaxed(&res_desc->words[7]);

	res->Status4 = readl_relaxed(&res_desc->words[10]);
	res->Status5 = readl_relaxed(&res_desc->words[11]);
	/* res->Status7 and res->Status8 are not in use */

#ifdef SAM_DMA_READ_DEBUG
	pr_info("RDR_Read32: 0x%8p + %2d * 4 = 0x%08x\n", res_desc, 0, readl_relaxed(&res_desc->words[0]));
	pr_info("RDR_Read32: 0x%8p + %2d * 4 = 0x%08x\n", res_desc, 1, readl_relaxed(&res_desc->words[1]));
	pr_info("RDR_Read32: 0x%8p + %2d * 4 = 0x%08x\n", res_desc, 2, readl_relaxed(&res_desc->words[2]));
	pr_info("RDR_Read32: 0x%8p + %2d * 4 = 0x%08x\n", res_desc, 3, readl_relaxed(&res_desc->words[3]));
	pr_info("RDR_Read32: 0x%8p + %2d * 4 = 0x%08x\n", res_desc, 4, readl_relaxed(&res_desc->words[4]));
	pr_info("RDR_Read32: 0x%8p + %2d * 4 = 0x%08x\n", res_desc, 5, readl_relaxed(&res_desc->words[5]));
	pr_info("RDR_Read32: 0x%8p + %2d * 4 = 0x%08x\n", res_desc, 6, readl_relaxed(&res_desc->words[6]));
	pr_info("RDR_Read32: 0x%8p + %2d * 4 = 0x%08x\n", res_desc, 7, readl_relaxed(&res_desc->words[7]));
	pr_info("RDR_Read32: 0x%8p + %2d * 4 = 0x%08x\n", res_desc, 8, readl_relaxed(&res_desc->words[8]));
	pr_info("RDR_Read32: 0x%8p + %2d * 4 = 0x%08x\n", res_desc, 9, readl_relaxed(&res_desc->words[9]));
	pr_info("RDR_Read32: 0x%8p + %2d * 4 = 0x%08x\n", res_desc, 10, readl_relaxed(&res_desc->words[10]));
	pr_info("RDR_Read32: 0x%8p + %2d * 4 = 0x%08x\n", res_desc, 11, readl_relaxed(&res_desc->words[11]));
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
int sam_hw_session_invalidate(struct sam_hw_ring *hw_ring, struct sam_buf_info *sa_buf,
				u32 next_request);

#endif /* _SAM_HW_H_ */
