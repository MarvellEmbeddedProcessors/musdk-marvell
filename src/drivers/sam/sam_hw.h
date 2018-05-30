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

/*#define SAM_REG_READ_DEBUG*/
/*#define SAM_REG_WRITE_DEBUG*/

/* Clcok frequency in MHz */
#define SAM_HIA_CLOCK_FREQ		500

/* HIA Options Register */
#define SAM_EIP197_HIA_OPTIONS_REG	0x9FFF8
#define SAM_EIP97_HIA_OPTIONS_REG	0x0FFF8

#define SAM_N_RINGS_OFFS		0
#define SAM_N_RINGS_BITS		4
#define SAM_N_RINGS_MASK		BIT_MASK(SAM_N_RINGS_BITS)
#define SAM_N_RINGS_GET(v32)		(((v32) >> SAM_N_RINGS_OFFS) & SAM_N_RINGS_MASK)

#define SAM_N_PES_OFFS			4
#define SAM_N_PES_BITS			3
#define SAM_N_PES_MASK			BIT_MASK(SAM_N_PES_BITS)
#define SAM_N_PES_GET(v32)		(((v32) >> SAM_N_PES_OFFS) & SAM_N_PES_MASK)

#define SAM_EIP197_AIC_RING_REGS_OFFS(ring)	(0x9E800 - (ring) * 0x1000)
#define SAM_EIP97_AIC_RING_REGS_OFFS(ring)	(0x0E000 - (ring) * 0x1000)
#define SAM_AIC_RING_POL_REG		0x0
#define SAM_AIC_RING_TYPE_REG		0x4
#define SAM_AIC_RING_ENABLE_REG		0x8
#define SAM_AIC_RING_RAW_STAT_REG	0xC	/* Read */
#define SAM_AIC_RING_ENABLE_SET_REG	0xC	/* Write*/
#define SAM_AIC_RING_STAT_ACK_REG	0x10
#define SAM_AIC_RING_ENABLE_CLR_REG	0x14
#define SAM_AIC_RING_OPTIONS_REG	0x18
#define SAM_AIC_RING_VERSION_REG	0x1C

/* EIP197_HIA_AIC_R_ENABLE_CTRL */
#define SAM_CDR_IRQ(n)			BIT((n) * 2)
#define SAM_RDR_IRQ(n)			BIT((n) * 2 + 1)

#define SAM_AIC_RING_ALL_IRQS_MASK(n)	(SAM_CDR_IRQ(n) | SAM_RDR_IRQ(n))

/* Command amd Result descriptors size and offset (in 32-bit words) */
#define SAM_CDR_ENTRY_WORDS            16
#define SAM_RDR_ENTRY_WORDS            16
#define SAM_RDR_PREP_WORDS		4

#define SAM_CDR_DSCR_EXT_WORD_COUNT    12
#define SAM_RDR_DSCR_EXT_WORD_COUNT    12

#define SAM_CDR_DSCR_WORD_COUNT        10
#define SAM_RDR_DSCR_WORD_COUNT        10

#define SAM_EIP197_RING_REGS_OFFS(ring)		(0x80000 + (ring) * 0x1000)
#define SAM_EIP97_RING_REGS_OFFS(ring)		(0x0 + (ring) * 0x1000)

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

/* Register fields definitions */

/* HIA_RDR_y_PREP_COUNT */
/* The 32-bit word count value for prep_rd_count, proc_rd_count and proc_cd_count fields */
#define SAM_RING_WORD_COUNT_OFFS		2
#define SAM_RING_WORD_COUNT_WRITE_BITS		14
#define SAM_RING_WORD_COUNT_WRITE_MASK		BIT_MASK(SAM_RING_WORD_COUNT_WRITE_BITS)
#define SAM_RING_WORD_COUNT_WRITE(words)	(((words) & SAM_RING_WORD_COUNT_WRITE_MASK) << SAM_RING_WORD_COUNT_OFFS)
#define SAM_RING_WORD_COUNT_READ_BITS		22
#define SAM_RING_WORD_COUNT_READ_MASK		BIT_MASK(SAM_RING_WORD_COUNT_READ_BITS)
#define SAM_RING_WORD_COUNT_READ(words)		(((words) & SAM_RING_WORD_COUNT_READ_MASK) << SAM_RING_WORD_COUNT_OFFS)
#define SAM_RING_WORD_COUNT_GET(val32)		(((val32) >> SAM_RING_WORD_COUNT_OFFS) & SAM_RING_WORD_COUNT_READ_MASK)

/* Descriptors count value for proc_rd_pkt_count and proc_cd_pkt_count */
#define SAM_RING_PKT_COUNT_OFFS			24
#define SAM_RING_PKT_COUNT_BITS			7
#define SAM_RING_PKT_COUNT_MASK			BIT_MASK(SAM_RING_PKT_COUNT_BITS)
#define SAM_RING_PKT_COUNT_VAL(pkts)		(((pkts) & SAM_RING_PKT_COUNT_MASK) << SAM_RING_PKT_COUNT_OFFS)
#define SAM_RING_PKT_COUNT_GET(val32)		(((val32) >> SAM_RING_PKT_COUNT_OFFS) & SAM_RING_PKT_COUNT_MASK)

/* Clear count bit for cd_proc, rd_prep and rd_proc registers */
#define SAM_RING_COUNT_CLEAR_MASK		BIT(31)

/* HIA_RDR_y_RING_SIZE and HIA_CDR_y_RING_SIZE registers */
/* Size of CDR/RDR ring in number of 32-bit words */
#define SAM_RING_SIZE_OFFS			2
#define SAM_RING_SIZE_BITS			22
#define SAM_RING_SIZE_MASK			BIT_MASK(SAM_RING_SIZE_BITS)
#define SAM_RING_SIZE_VAL(words)		(((words) & SAM_RING_SIZE_MASK) << SAM_RING_SIZE_OFFS)

/* HIA_RDR_y_DESC_SIZE and HIA_CDR_y_DESC_SIZE registers */
/* Size of the CDR/RDR descriptors in 32-bit words */
#define SAM_RING_DESC_SIZE_OFFS			0
#define SAM_RING_DESC_SIZE_BITS			8
#define SAM_RING_DESC_SIZE_MASK			BIT_MASK(SAM_RING_DESC_SIZE_BITS)
#define SAM_RING_DESC_SIZE_VAL(words)		(((words) & SAM_RING_DESC_SIZE_MASK) << SAM_RING_DESC_SIZE_OFFS)

/* Offset betwwen two CDR/RDR descriptors in 32-bit words */
#define SAM_RING_DESC_OFFSET_OFFS		16
#define SAM_RING_DESC_OFFSET_BITS		8
#define SAM_RING_DESC_OFFSET_MASK		BIT_MASK(SAM_RING_DESC_OFFSET_BITS)
#define SAM_RING_DESC_OFFSET_VAL(words)		(((words) & SAM_RING_DESC_OFFSET_MASK) << SAM_RING_DESC_OFFSET_OFFS)

/* Additional Control_Data Pointer (ACDP) present: 1b = Command Descriptors contains ACDPs */
#define SAM_RING_TOKEN_PTR_MASK			BIT(30)

/* Extended Addressing Mode: 1b = pointers in the Command Descriptor are up to 64-bit wide */
#define SAM_RING_64B_MODE_MASK			BIT(31)

/* HIA_RDR_y_CFG and HIA_CDR_y_CFG registers */
#define SAM_RING_FETCH_SIZE_OFFS		0
#define SAM_RING_FETCH_SIZE_BITS		16
#define SAM_RING_FETCH_SIZE_MASK		BIT_MASK(SAM_RING_FETCH_SIZE_BITS)
#define SAM_RING_FETCH_SIZE_VAL(words)		(((words) & SAM_RING_FETCH_SIZE_MASK) << SAM_RING_FETCH_SIZE_OFFS)

#define SAM_RING_FETCH_THRESH_OFFS		16
#define SAM_RING_FETCH_THRESH_BITS		9
#define SAM_RING_FETCH_THRESH_MASK		BIT_MASK(SAM_RING_FETCH_THRESH_BITS)
#define SAM_RING_FETCH_THRESH_VAL(words)	(((words) & SAM_RING_FETCH_THRESH_MASK) << SAM_RING_FETCH_THRESH_OFFS)

#define SAM_RING_OFLO_IRQ_EN_MASK		BIT(25)
#define SAM_RING_OWN_MODE_EN_MASK		BIT(31)

/* HIA_RDR_y_DMA_CFG and HIA_CDR_y_DMA_CFG registers */
/* ACD relevant fields are valid only for CDR_DMA_CFG register */
#define SAM_RING_DESC_SWAP_OFFS			0
#define SAM_RING_DATA_SWAP_OFFS			8
#define SAM_RING_ACD_SWAP_OFFS			16

#define SAM_RING_SWAP_BITS			3
#define SAM_RING_SWAP_MASK			BIT_MASK(SAM_RING_SWAP_BITS)

/* Bit [0] = Swap bytes within each 32 bit word */
#define SAM_RING_DESC_SWAP_VAL(val)		(((val) & SAM_RING_SWAP_MASK) << SAM_RING_DESC_SWAP_OFFS)
#define SAM_RING_DATA_SWAP_VAL(val)		(((val) & SAM_RING_SWAP_MASK) << SAM_RING_DATA_SWAP_OFFS)
#define SAM_RING_ACD_SWAP_VAL(val)		(((val) & SAM_RING_SWAP_MASK) << SAM_RING_ACD_SWAP_OFFS)

#define SAM_RING_DESC_PROTECT_OFFS		4
#define SAM_RING_DATA_PROTECT_OFFS		12
#define SAM_RING_ACD_PROTECT_OFFS		20

#define SAM_RING_PROTECT_BITS			3
#define SAM_RING_PROTECT_MASK			BIT_MASK(SAM_RING_PROTECT_BITS)

#define SAM_RING_DESC_PROTECT_VAL(val)		(((val) & SAM_RING_PROTECT_MASK) << SAM_RING_DESC_PROTECT_OFFS)
#define SAM_RING_DATA_PROTECT_VAL(val)		(((val) & SAM_RING_PROTECT_MASK) << SAM_RING_DATA_PROTECT_OFFS)
#define SAM_RING_ACD_PROTECT_VAL(val)		(((val) & SAM_RING_PROTECT_MASK) << SAM_RING_ACD_PROTECT_OFFS)

/* 2 bits below valid only for RDR_DMA_CFG register */
#define SAM_RING_RES_BUF_EN_MASK		BIT(22)
#define SAM_RING_CTR_BUF_EN_MASK		BIT(23)

/* Bufferability control for ownership word DMA writes */
#define SAM_RING_OWN_BUF_EN_MASK		BIT(24)

/* Enables padding the result descriptor to its full programmed offset with 0xAAAAAAAA */
#define SAM_RING_PAD_TO_OFFSET_MASK		BIT(28)

/* Read and Write cache control */
#define SAM_RING_WRITE_CACHE_OFFS		25
#define SAM_RING_READ_CACHE_OFFS		29

#define SAM_RING_CACHE_CTRL_BITS		3
#define SAM_RING_CACHE_CTRL_MASK		BIT_MASK(SAM_RING_CACHE_CTRL_BITS)

#define SAM_RING_WRITE_CACHE_VAL(val)		(((val) & SAM_RING_CACHE_CTRL_MASK) << SAM_RING_WRITE_CACHE_OFFS)
#define SAM_RING_READ_CACHE_VAL(val)		(((val) & SAM_RING_CACHE_CTRL_MASK) << SAM_RING_READ_CACHE_OFFS)

/* HIA_CDR_y_THRESH and HIA_RDR_y_THRESH register */
#define SAM_RING_PKT_THRESH_OFFS		0
#define SAM_RING_PKT_THRESH_BITS		16
#define SAM_RING_PKT_THRESH_MASK		BIT_MASK(SAM_RING_PKT_THRESH_BITS)
#define SAM_RING_PKT_THRESH_VALUE(val)		(((val) & SAM_RING_PKT_THRESH_MASK) << SAM_RING_PKT_THRESH_OFFS)

#define SAM_RING_PROC_MODE_MASK			BIT(22)
#define SAM_RING_PKT_MODE_MASK			BIT(23)

#define SAM_RING_TIMEOUT_OFFS			24
#define SAM_RING_TIMEOUT_BITS			8
#define SAM_RING_TIMEOUT_MASK			BIT_MASK(SAM_RING_TIMEOUT_BITS)
#define SAM_RING_TIMEOUT_VAL(val)		(((val) & SAM_RING_TIMEOUT_MASK) << SAM_RING_TIMEOUT_OFFS)

/* HIA_RDR_y_THRESH register */

/* HIA_CDR_y_STAT register */
#define SAM_CDR_STAT_IRQ_OFFS			0
#define SAM_CDR_STAT_IRQ_BITS			6
#define SAM_CDR_STAT_IRQ_MASK			BIT_MASK(SAM_CDR_STAT_IRQ_BITS)

/* HIA_RDR_y_STAT register */
#define SAM_RDR_STAT_IRQ_OFFS			0
#define SAM_RDR_STAT_IRQ_BITS			8
#define SAM_RDR_STAT_IRQ_MASK			BIT_MASK(SAM_RDR_STAT_IRQ_BITS)

/* Marvell specific configuration values for AXI3 */
#define CONF_DESC_SWAP_VALUE			0
#define CONF_DATA_SWAP_VALUE			0
#define CONF_TOKEN_SWAP_VALUE			0
#define CONF_DESC_PROT_VALUE			2
#define CONF_DATA_PROT_VALUE			2
#define CONF_TOKEN_PROT_VALUE			2
#define CONF_WRITE_CACHE_CTRL			0x3 /* 0x7 >> 1 */
#define CONF_READ_CACHE_CTRL			0x5 /* 0xB >> 1 */

/* Number of CDR/RDR descriptors used to calculate fetch size and threshold */
#define SAM_CDR_FETCH_COUNT			1
#define SAM_RDR_FETCH_COUNT			1

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

/* Buffer and Descriptor overflow bits in Result descrtiptor */
#define SAM_DESC_DESCR_OFLO_MASK	BIT(20)
#define SAM_DESC_BUF_OFLO_MASK		BIT(21)

/* First and Last segments indication for Command and Result descriptors */
#define SAM_DESC_LAST_SEG_MASK		BIT(22)
#define SAM_DESC_FIRST_SEG_MASK		BIT(23)

/* Token size in 32bit words for first segment of Command descriptor */
#define SAM_CDR_TOKEN_BYTES_OFFS	24
#define SAM_CDR_TOKEN_BYTES_BITS	8
#define SAM_CDR_TOKEN_BYTES_MASK	BIT_MASK(SAM_CDR_TOKEN_BYTES_BITS)

/* Token header - CDR word #6, Token word #0 */

/* Segment size in bytes for Command and Result descriptors */
#define SAM_TOKEN_PKT_LEN_OFFS		0
#define SAM_TOKEN_PKT_LEN_BITS		17
#define SAM_TOKEN_PKT_LEN_MASK		BIT_MASK(SAM_TOKEN_PKT_LEN_BITS)

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

/* Bits in token header work special for DTLS flow */
#define SAM_TOKEN_DTLS_CAPWAP_MASK	BIT(26)
#define SAM_TOKEN_DTLS_INBOUND_MASK	BIT(27)

/* DTLS Content Type */
#define SAM_TOKEN_DTLS_CONTENT_OFFS	28
#define SAM_TOKEN_DTLS_CONTENT_MASK	(0x3)
#define SAM_TOKEN_DTLS_CONTENT_SET(v)	(((v) & SAM_TOKEN_DTLS_CONTENT_MASK) << SAM_TOKEN_DTLS_CONTENT_OFFS)
#define SAM_TOKEN_DTLS_CONTENT_GET(v)	(((v) >> SAM_TOKEN_DTLS_CONTENT_OFFS) & SAM_TOKEN_DTLS_CONTENT_MASK)

/* Application ID - CDR word #7, Token word #1 */
#define SAM_TOKEN_APPL_ID_OFFS		9
#define SAM_TOKEN_APPL_ID_BITS		7
#define SAM_TOKEN_APPL_ID_MASK		BIT_MASK(SAM_TOKEN_APPL_ID_BITS)
#define SAM_TOKEN_APPL_ID_SET(id)	(((id) & SAM_TOKEN_APPL_ID_MASK) << SAM_TOKEN_APPL_ID_OFFS)
#define SAM_TOKEN_APPL_ID_GET(v32)	(((v32) >> SAM_TOKEN_APPL_ID_OFFS) & SAM_TOKEN_APPL_ID_MASK)

#define SAM_DEFAULT_APPL_ID		0x76

/* SA pointer - CDR words #8-9, Token words #2-3 */

/* HW Services - CDR word #10, Token word #4 */
#define FIRMWARE_HW_SERVICES_OFFS	24
#define FIRMWARE_HW_SERVICES_BITS	6
#define FIRMWARE_HW_SERVICES_ALL_MASK	BIT_MASK_OFFS(FIRMWARE_HW_SERVICES_OFFS, FIRMWARE_HW_SERVICES_BITS)

/* Lookaside IPsec packet flow */
#define FIRMWARE_CMD_PKT_LIP_MASK       (0x02 << FIRMWARE_HW_SERVICES_OFFS)

/* Lookaside Crypto packet Flow */
#define FIRMWARE_CMD_PKT_LAC_MASK	(0x04 << FIRMWARE_HW_SERVICES_OFFS)

/* Invalidate Transform Record command */
#define FIRMWARE_CMD_INV_TR_MASK	(0x06 << FIRMWARE_HW_SERVICES_OFFS)

/* Look-aside DTLS with optional CAPWAP (LDT-NPH-TB) */
#define FIRMWARE_CMD_PKT_LDT_MASK       (0x28 << FIRMWARE_HW_SERVICES_OFFS)

/* CDR word #12, Token word #5 */
#define SAM_CMD_TOKEN_OFFSET_OFFS	8
#define SAM_CMD_TOKEN_OFFSET_BITS	8
#define SAM_CMD_TOKEN_OFFSET_MASK	BIT_MASK(SAM_CMD_TOKEN_OFFSET_BITS)
#define SAM_CMD_TOKEN_OFFSET_SET(v)	(((v) & SAM_CMD_TOKEN_OFFSET_MASK) << SAM_CMD_TOKEN_OFFSET_OFFS)
#define SAM_CMD_TOKEN_OFFSET_GET(v)	(((v) >> SAM_CMD_TOKEN_OFFSET_OFFS) & SAM_CMD_TOKEN_OFFSET_MASK)

#define SAM_CMD_TOKEN_NEXT_HDR_OFFS	16
#define SAM_CMD_TOKEN_NEXT_HDR_BITS	8
#define SAM_CMD_TOKEN_NEXT_HDR_MASK	BIT_MASK(SAM_CMD_TOKEN_NEXT_HDR_BITS)
#define SAM_CMD_TOKEN_NEXT_HDR_SET(v)	(((v) & SAM_CMD_TOKEN_NEXT_HDR_MASK) << SAM_CMD_TOKEN_NEXT_HDR_OFFS)
#define SAM_CMD_TOKEN_NEXT_HDR_GET(v)	(((v) >> SAM_CMD_TOKEN_NEXT_HDR_OFFS) & SAM_CMD_TOKEN_NEXT_HDR_MASK)


/******* Result Token fields ********/

/* Errors: [E0..E14] - RDR word #4, Result token word #0 */
#define SAM_RES_TOKEN_ERRORS_OFFS	17
#define SAM_RER_TOKEN_ERRORS_BITS	15
#define SAM_RES_TOKEN_ERRORS_MASK	BIT_MASK(SAM_RES_TOKEN_ERRORS_BITS)

/* E0 - Packet length error: token instructions versus input or input DMA fetch */
#define SAM_RESULT_PKT_LEN_ERROR_MASK		BIT(0)

/* E1 - Token error, unknown token command/instruction */
#define SAM_RESULT_TOKEN_ERROR_MASK		BIT(1)

/* E2 - Token contains too much bypass data */
#define SAM_RESULT_BYPASS_ERROR_MASK		BIT(2)

/* E3 - Cryptographic block size error (ECB, CBC) */
#define SAM_RESULT_CRYPTO_SIZE_ERROR_MASK	BIT(3)

/* E4 - HASH block size error (basic hash only) */
#define SAM_RESULT_HASH_SIZE_ERROR_MASK		BIT(4)

/* E5 - Invalid command/algorithm/mode/combination or context read DMA error */
#define SAM_RESULT_INVALID_CMD_ERROR_MASK	BIT(5)

/* E6 - Prohibited algorithm or context read ECC error */
#define SAM_RESULT_BAD_ALG_ERROR_MASK		BIT(6)

/* E7 - Hash input overflow (basic hash only) */
#define SAM_RESULT_HASH_INPUT_OFLO_ERROR_MASK	BIT(7)

/* E8 - TTL / HOP-limit underflow */
#define SAM_RESULT_TTL_ERROR_MASK		BIT(8)

/* E9 - Authentication error */
#define SAM_RESULT_AUTH_ERROR_MASK		BIT(9)

/* E10 - Sequence number error */
#define SAM_RESULT_SEQ_ERROR_MASK		BIT(10)

/* E11 - SPI error */
#define SAM_RESULT_SPI_ERROR_MASK		BIT(11)

/* E12 - Checksum error */
#define SAM_RESULT_CHECKSUM_ERROR_MASK		BIT(12)

/* E13 - PAD verification error */
#define SAM_RESULT_PAD_ERROR_MASK		BIT(13)

/* E14 - Timeout error occurs */
#define SAM_RESULT_TIMEOUT_ERROR_MASK		BIT(14)

/* RDR word #5, Result token word #1 */

/* Bits[0-3] - Bypass data length */
#define SAM_RES_TOKEN_BYPASS_LEN_OFFS		0
#define SAM_RES_TOKEN_BYPASS_LEN_BITS		4
#define SAM_RES_TOKEN_BYPASS_LEN_MASK		BIT_MASK(SAM_RES_TOKEN_BYPASS_LEN_BITS)
#define SAM_RES_TOKEN_BYPASS_LEN_GET(v)		(((v) >> SAM_RES_TOKEN_BYPASS_LEN_OFFS) & SAM_RES_TOKEN_BYPASS_LEN_MASK)

/* Bit[4] - E15 extra error:  */
#define SAM_RES_TOKEN_E15_MASK			BIT(4)

/* Bits[12-5] - TOS/TC from inner IP header */
#define SAM_RES_TOKEN_TOS_OFFS			5
#define SAM_RES_TOKEN_TOS_BITS			8
#define SAM_RES_TOKEN_TOS_MASK			BIT_MASK(SAM_RES_TOKEN_TOS_BITS)
#define SAM_RES_TOKEN_TOS_GET(v)		(((v) >> SAM_RES_TOKEN_TOS_OFFS) & SAM_RES_TOKEN_TOS_MASK)

/* Bit[13] - Don't fragment flag from inner header */
#define SAM_RES_TOKEN_DF_MASK			BIT(13)

/* Bits[16..20] - CLE Errors */
#define SAM_RES_TOKEN_CLE_OFFS			16
#define SAM_RES_TOKEN_CLE_BITS			5
#define SAM_RES_TOKEN_CLE_MASK			BIT_MASK(SAM_RES_TOKEN_CLE_BITS)
#define SAM_RES_TOKEN_CLE_GET(v)		(((v) >> SAM_RES_TOKEN_CLE_OFFS) & SAM_RES_TOKEN_CLE_MASK)

/* CLE Errors values */
#define	SAM_RES_TOKEN_CLE_NOT_FOUND_ERR		1
#define	SAM_RES_TOKEN_CLE_NOT_SUPPORTED_ERR	2
#define	SAM_RES_TOKEN_CLE_PROTO_ERR		3


/* Bit[21] - HASH Bytes appended */
#define SAM_RES_TOKEN_HASH_ADDED_MASK		BIT(21)

/* Bits[22..27] - Hash Bytes */
#define SAM_RES_TOKEN_HASH_BYTES_OFFS		22
#define SAM_RES_TOKEN_HASH_BYTES_BITS		6
#define SAM_RES_TOKEN_HASH_BYTES_MASK		BIT_MASK(SAM_RES_TOKEN_HASH_BYTES_BITS)
#define SAM_RES_TOKEN_HASH_BYTES_GET(v)		(((v) >> SAM_RES_TOKEN_HASH_BYTES_OFFS) & SAM_RES_TOKEN_HASH_BYTES_MASK)

/* Bit[28] - Generic bytes appended at the end of packet */
#define SAM_RES_TOKEN_B_MASK			BIT(28)

/* Bit[29] - Checksum value appended at the end of packet */
#define SAM_RES_TOKEN_C_MASK			BIT(29)

/* Bit[30] - Next header appended at the end of packet */
#define SAM_RES_TOKEN_N_MASK			BIT(30)

/* Bit[31] - Length value appended at the end of packet */
#define SAM_RES_TOKEN_L_MASK			BIT(31)

/* RDR word #6, Result token word #2 */

/* RDR word #7, Result token word #3 */
#define SAM_RES_TOKEN_NEXT_HDR_OFFS	0
#define SAM_RES_TOKEN_NEXT_HDR_BITS	8
#define SAM_RES_TOKEN_NEXT_HDR_MASK	BIT_MASK(SAM_RES_TOKEN_NEXT_HDR_BITS)
#define SAM_RES_TOKEN_NEXT_HDR_SET(v)	(((v) & SAM_RES_TOKEN_NEXT_HDR_MASK) << SAM_RES_TOKEN_NEXT_HDR_OFFS)
#define SAM_RES_TOKEN_NEXT_HDR_GET(v)	(((v) >> SAM_RES_TOKEN_NEXT_HDR_OFFS) & SAM_RES_TOKEN_NEXT_HDR_MASK)

#define SAM_RES_TOKEN_PAD_LEN_OFFS	8
#define SAM_RES_TOKEN_PAD_LEN_BITS	8
#define SAM_RES_TOKEN_PAD_LEN_MASK	BIT_MASK(SAM_RES_TOKEN_PAD_LEN_BITS)
#define SAM_RES_TOKEN_PAD_LEN_SET(v)	(((v) & SAM_RES_TOKEN_PAD_LEN_MASK) << SAM_RES_TOKEN_PAD_LEN_OFFS)
#define SAM_RES_TOKEN_PAD_LEN_GET(v)	(((v) >> SAM_RES_TOKEN_PAD_LEN_OFFS) & SAM_RES_TOKEN_PAD_LEN_MASK)

#define SAM_RES_TOKEN_OFFSET_OFFS	16
#define SAM_RES_TOKEN_OFFSET_BITS	8
#define SAM_RES_TOKEN_OFFSET_MASK	BIT_MASK(SAM_RES_TOKEN_OFFSET_BITS)
#define SAM_RES_TOKEN_OFFSET_SET(v)	(((v) & SAM_RES_TOKEN_OFFSET_MASK) << SAM_RES_TOKEN_OFFSET_OFFS)
#define SAM_RES_TOKEN_OFFSET_GET(v)	(((v) >> SAM_RES_TOKEN_OFFSET_OFFS) & SAM_RES_TOKEN_OFFSET_MASK)

enum sam_hw_type {
	HW_EIP197,
	HW_EIP97,
	HW_TYPE_LAST
};

struct sam_hw_device_info {
	struct sys_iomem *iomem_info;
	enum sam_hw_type type;
	char *name;
	void *vaddr;		/* virtual address for device registers */
	dma_addr_t paddr;	/* physical address for device registers */
	int rings_num;
	u32 available_rings;
	u32 active_rings;
};

struct sam_hw_ring {
	u32 device;
	u32 ring;
	void *aic_regs_vbase;			/* virtual address for ring AIC registers */
	dma_addr_t aic_regs_paddr;		/* physical address for ring AIC registers */
	void *regs_vbase;			/* virtual address for ring registers */
	dma_addr_t paddr;			/* physical address for ring registers */
	enum sam_hw_type type;			/* EIP197 or EIP97 */
	u32 ring_size;                          /* CDR and RDR size in descriptors */
	u32 rdr_thresh_val;			/* Value for HIA_RDR_THRESH_REG */
	struct sam_buf_info cdr_buf;            /* DMA memory buffer allocated for command descriptors */
	struct sam_buf_info rdr_buf;            /* DMA memory buffer allocated for result descriptors */
	struct sam_hw_cmd_desc *cmd_desc_first;	/* Pointer to first command descriptors in DMA memory */
	struct sam_hw_res_desc *res_desc_first;	/* Pointer to first command descriptors in DMA memory */
	u32 next_cdr;				/* Index of first CD to write */
	u32 next_rdr;				/* Index of first RD to write */
	u32 free_cdr;				/* Index of first CD to free */
	u32 free_rdr;				/* Index of first RD to free */
};

#ifdef MVCONF_SAM_DEBUG
extern u32 sam_debug_flags;

void print_cmd_desc(struct sam_hw_cmd_desc *cmd_desc);
void print_result_desc(struct sam_hw_res_desc *res_desc, int is_prepared);
#endif

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

static inline void sam_hw_cmd_desc_put(struct sam_hw_ring *hw_ring, u32 num)
{
	if ((hw_ring->free_cdr + num) >= hw_ring->ring_size)
		hw_ring->free_cdr = num - (hw_ring->ring_size - hw_ring->free_cdr);
	else
		hw_ring->free_cdr += num;
}

static inline struct sam_hw_res_desc *sam_hw_res_desc_get(struct sam_hw_ring *hw_ring, u32 idx)
{
	return (hw_ring->res_desc_first + idx);
}

static inline u32 sam_hw_next_idx(struct sam_hw_ring *ring, u32 idx)
{
	idx++;
	if (idx == ring->ring_size)
		idx = 0;

	return idx;
}

static inline u32 sam_hw_prev_idx(struct sam_hw_ring *ring, u32 idx)
{
	if (idx)
		idx--;
	else
		idx = ring->ring_size - 1;

	return idx;
}

static inline void sam_hw_ring_roolback(struct sam_hw_ring *ring, u32 rdr_idx, u32 cdr_idx)
{
	int i;

	for (i = 0; i < rdr_idx; i++)
		ring->next_rdr = sam_hw_prev_idx(ring, ring->next_rdr);
	for (i = 0; i < cdr_idx; i++)
		ring->next_cdr = sam_hw_prev_idx(ring, ring->next_cdr);
}

static inline bool sam_hw_cmd_is_free_slot(struct sam_hw_ring *hw_ring, int num)
{
	int free_desc_num;

	/* calc number of free descriptors */
	if (hw_ring->next_cdr >= hw_ring->free_cdr)
		free_desc_num = hw_ring->ring_size -
				(hw_ring->next_cdr - hw_ring->free_cdr);
	else
		free_desc_num = hw_ring->free_cdr - hw_ring->next_cdr;

	/* compare */
	if (num < free_desc_num)
		return true;

	return false;
}

static inline void sam_hw_rdr_prep_desc_write(struct sam_hw_res_desc *res_desc,
					dma_addr_t dst_paddr, u32 prep_size, u32 seg_mask)
{
	u32 ctrl_word;

	/* bits[24-31] - ExpectedResultWordCount = 0 */
	ctrl_word = seg_mask; /* First, Last, or (Last and First) */

	ctrl_word |= (prep_size & SAM_DESC_SEG_BYTES_MASK);

	writel_relaxed(ctrl_word, &res_desc->words[0]);
	writel_relaxed(0, &res_desc->words[1]); /* skip this write */

	/* Write Destination Packet Data address */
	writel_relaxed(lower_32_bits(dst_paddr), &res_desc->words[2]);
	writel_relaxed(upper_32_bits(dst_paddr), &res_desc->words[3]);

#ifdef MVCONF_SAM_DEBUG
	if (unlikely(sam_debug_flags & SAM_CIO_DEBUG_FLAG))
		print_result_desc(res_desc, 1);
#endif /* MVCONF_SAM_DEBUG */

}

static inline void sam_hw_rdr_desc_write(struct sam_hw_ring *hw_ring,
					dma_addr_t dst_paddr, u32 prep_size, u32 seg_mask)
{
	struct sam_hw_res_desc *res_desc;

	res_desc = sam_hw_res_desc_get(hw_ring,  hw_ring->next_rdr);

	/* Write RDR descriptor */
	sam_hw_rdr_prep_desc_write(res_desc, dst_paddr, prep_size, seg_mask);

	hw_ring->next_rdr = sam_hw_next_idx(hw_ring, hw_ring->next_rdr);
}

static inline void sam_hw_cdr_cmd_desc_write(struct sam_hw_cmd_desc *cmd_desc,
					     dma_addr_t src_paddr, u32 data_bytes,
					     dma_addr_t token_paddr, u32 token_words,
					     u32 seg_mask)
{
	u32 ctrl_word;

	ctrl_word = (data_bytes & SAM_DESC_SEG_BYTES_MASK);
	ctrl_word |= (token_words & SAM_CDR_TOKEN_BYTES_MASK) << SAM_CDR_TOKEN_BYTES_OFFS;
	ctrl_word |= seg_mask;  /* First, Last or (Last and First) */

	writel_relaxed(ctrl_word, &cmd_desc->words[0]);
	writel_relaxed(0, &cmd_desc->words[1]); /* skip this write */

	/* Write Source Packet Data address */
	writel_relaxed(lower_32_bits(src_paddr), &cmd_desc->words[2]);
	writel_relaxed(upper_32_bits(src_paddr), &cmd_desc->words[3]);

	/* Write Token Data address */
	writel_relaxed(lower_32_bits(token_paddr), &cmd_desc->words[4]);
	writel_relaxed(upper_32_bits(token_paddr), &cmd_desc->words[5]);
}

static inline void sam_hw_cdr_proto_cmd_desc_write(struct sam_hw_ring *hw_ring,
						   dma_addr_t src_paddr,
						   u32 data_bytes,
						   u32 seg_mask)
{
	struct sam_hw_cmd_desc *cmd_desc;
	u32 ctrl_word;

	cmd_desc = sam_hw_cmd_desc_get(hw_ring, hw_ring->next_cdr);

	ctrl_word = (data_bytes & SAM_DESC_SEG_BYTES_MASK);
	ctrl_word |= seg_mask;  /* Last or First */

	writel_relaxed(ctrl_word, &cmd_desc->words[0]);
	writel_relaxed(0, &cmd_desc->words[1]); /* skip this write */

	/* Write Source Packet Data address */
	writel_relaxed(lower_32_bits(src_paddr), &cmd_desc->words[2]);
	writel_relaxed(upper_32_bits(src_paddr), &cmd_desc->words[3]);

	/* Token will be built inside the device */
	writel_relaxed(0, &cmd_desc->words[4]);
	writel_relaxed(0, &cmd_desc->words[5]);

#ifdef MVCONF_SAM_DEBUG
	if (unlikely(sam_debug_flags & SAM_CIO_DEBUG_FLAG))
		print_cmd_desc(cmd_desc);
#endif /* MVCONF_SAM_DEBUG */

	hw_ring->next_cdr = sam_hw_next_idx(hw_ring, hw_ring->next_cdr);
}

static inline void sam_hw_ring_basic_cdr_desc_write(struct sam_hw_ring *hw_ring,
						    struct sam_buf_info *src_buf,
						    u32 copy_len,
						    struct sam_buf_info *sa_buf,
						    struct sam_buf_info *token_buf,
						    u32 token_header_word,
						    u32 token_words, u32 seg_mask)
{
	u32 val32;
	struct sam_hw_cmd_desc *cmd_desc;

	cmd_desc = sam_hw_cmd_desc_get(hw_ring, hw_ring->next_cdr);

	/* Write CDR descriptor */
	sam_hw_cdr_cmd_desc_write(cmd_desc, src_buf->paddr, copy_len,
				  token_buf->paddr, token_words,
				  seg_mask);

	if (seg_mask & SAM_DESC_FIRST_SEG_MASK) {
		/* write token only for first descriptor */
		/* Set 64-bit Context (SA) pointer and IP EIP97 */
		token_header_word |= (SAM_TOKEN_CP_64B_MASK | SAM_TOKEN_IP_EIP97_MASK);

		writel_relaxed(token_header_word, &cmd_desc->words[6]);

		/* EIP202_RING_ANTI_DMA_RACE_CONDITION_CDS - EIP202_DSCR_DONE_PATTERN */
		writel_relaxed(SAM_TOKEN_APPL_ID_SET(SAM_DEFAULT_APPL_ID), &cmd_desc->words[7]);

		val32 = lower_32_bits((u64)sa_buf->paddr);
		writel_relaxed(val32, &cmd_desc->words[8]);

		val32 = upper_32_bits((u64)sa_buf->paddr);
		writel_relaxed(val32, &cmd_desc->words[9]);
	}

#ifdef MVCONF_SAM_DEBUG
	if (unlikely(sam_debug_flags & SAM_CIO_DEBUG_FLAG)) {
		print_cmd_desc(cmd_desc);
	}
#endif /* MVCONF_SAM_DEBUG */

	hw_ring->next_cdr = sam_hw_next_idx(hw_ring, hw_ring->next_cdr);
}

static inline void sam_hw_cdr_ext_token_write(struct sam_hw_cmd_desc *cmd_desc,
					struct sam_buf_info *sa_buf, u32 token_header_word,
					u32 hw_service, u32 proto_word)
{
	u32 val32;

	writel_relaxed(token_header_word, &cmd_desc->words[6]);

	/* EIP202_RING_ANTI_DMA_RACE_CONDITION_CDS - EIP202_DSCR_DONE_PATTERN */
	writel_relaxed(SAM_TOKEN_APPL_ID_SET(SAM_DEFAULT_APPL_ID), &cmd_desc->words[7]);

	val32 = lower_32_bits(sa_buf->paddr);
	val32 |= 0x2;
	writel_relaxed(val32, &cmd_desc->words[8]);

	val32 = upper_32_bits(sa_buf->paddr);
	writel_relaxed(val32, &cmd_desc->words[9]);

	writel_relaxed(hw_service, &cmd_desc->words[10]);

	writel_relaxed(proto_word, &cmd_desc->words[11]);
}

static inline void sam_hw_ring_ext_desc_write(struct sam_hw_ring *hw_ring,
						    struct sam_buf_info *src_buf,
						    u32 copy_len,
						    struct sam_buf_info *sa_buf,
						    struct sam_buf_info *token_buf,
						    u32 token_header_word,
						    u32 token_words,
						    u32 seg_mask)
{
	struct sam_hw_cmd_desc *cmd_desc;

	cmd_desc = sam_hw_cmd_desc_get(hw_ring, hw_ring->next_cdr);
	/* Write CDR descriptor */
	sam_hw_cdr_cmd_desc_write(cmd_desc, src_buf->paddr, copy_len,
				  token_buf->paddr, token_words, seg_mask);

	/* write token for first descriptor only */
	if (seg_mask & SAM_DESC_FIRST_SEG_MASK) {
		/* Set 64-bit Context (SA) pointer and IP EIP97 */
		token_header_word |= (SAM_TOKEN_CP_64B_MASK | SAM_TOKEN_IP_EIP97_MASK);
		token_header_word |= SAM_TOKEN_TYPE_EXTENDED_MASK;

		sam_hw_cdr_ext_token_write(cmd_desc, sa_buf, token_header_word,
					   FIRMWARE_CMD_PKT_LAC_MASK, 0);
	}

#ifdef MVCONF_SAM_DEBUG
	if (unlikely(sam_debug_flags & SAM_CIO_DEBUG_FLAG))
		print_cmd_desc(cmd_desc);
#endif /* MVCONF_SAM_DEBUG */

	hw_ring->next_cdr = sam_hw_next_idx(hw_ring, hw_ring->next_cdr);
}

static inline void sam_hw_ring_sa_inv_desc_write(struct sam_hw_ring *hw_ring,
						 dma_addr_t paddr)
{
	u32 token_header, val32, ctrl_word;
	struct sam_hw_cmd_desc *cmd_desc;
	struct sam_hw_res_desc *res_desc;

	cmd_desc = sam_hw_cmd_desc_get(hw_ring, hw_ring->next_cdr);
	res_desc = sam_hw_res_desc_get(hw_ring, hw_ring->next_rdr);

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

	hw_ring->next_cdr = sam_hw_next_idx(hw_ring, hw_ring->next_cdr);
	hw_ring->next_rdr = sam_hw_next_idx(hw_ring, hw_ring->next_rdr);
}

static inline int sam_hw_res_desc_read(struct sam_hw_res_desc *res_desc, struct sam_cio_op_result *result)
{
	u32 val32, errors, cle_err;

	/* ControlWord - Bit[20] - Descr_Oflo and Bit[21] - Buffer_Oflo */
	val32 = readl_relaxed(&res_desc->words[0]);
	errors = (val32 & (SAM_DESC_DESCR_OFLO_MASK | SAM_DESC_BUF_OFLO_MASK));

#ifdef MVCONF_SAM_DEBUG
	/* Check First and Last bits are set */
	if (unlikely((val32 & (SAM_DESC_FIRST_SEG_MASK | SAM_DESC_LAST_SEG_MASK)) !=
		     (SAM_DESC_FIRST_SEG_MASK | SAM_DESC_LAST_SEG_MASK))) {
		pr_err("%p Result descriptor is not ready, word[0] = 0x%8x\n",
			res_desc, val32);
		return -EINVAL;
	}
	/* Invalidate First and Last bits - set 0x00 to word #0 */
	writel_relaxed(0, &res_desc->words[0]);

	/* Check application ID - token_data[2] */
	val32 = readl_relaxed(&res_desc->words[6]);
	if (unlikely(SAM_TOKEN_APPL_ID_GET(val32) != SAM_DEFAULT_APPL_ID)) {
		pr_err("%p Result descriptor is not ready, word[0] = 0x%8x\n",
			res_desc, val32);
		return -EINVAL;
	}
	/* Invalidate application ID - set 0x00 to token_data[2] */
	writel_relaxed(SAM_TOKEN_APPL_ID_SET(0), &res_desc->words[6]);
#endif

	/* Result Token Data - token_data[0] */
	val32 = readl_relaxed(&res_desc->words[4]);
	result->out_len = val32 & SAM_TOKEN_PKT_LEN_MASK;

	errors |= (val32 >> SAM_RES_TOKEN_ERRORS_OFFS);

	/* E15 and CLE errors - token_data[1] */
	val32 = readl_relaxed(&res_desc->words[5]);
	if (unlikely(val32 & SAM_RES_TOKEN_E15_MASK))
		errors |= BIT(15);

	/* CLE errors bits for extended usage */
	cle_err = SAM_RES_TOKEN_CLE_GET(val32);

	if (likely((errors == 0) && (cle_err == 0)))
		result->status = SAM_CIO_OK;
	else if (errors & SAM_RESULT_AUTH_ERROR_MASK)
		result->status = SAM_CIO_ERR_ICV;
	else if (errors & SAM_RESULT_SEQ_ERROR_MASK)
		result->status = SAM_CIO_ERR_ANTIREPLAY;
	else if (cle_err == SAM_RES_TOKEN_CLE_NOT_FOUND_ERR)
		result->status = SAM_CIO_ERR_SA_LOOKUP;
	else if (cle_err == SAM_RES_TOKEN_CLE_PROTO_ERR)
		result->status = SAM_CIO_ERR_PROTO;
	else {
		result->status = SAM_CIO_ERR_HW;
#ifdef MVCONF_SAM_DEBUG
		pr_warn("HW error: errors = 0x%08x, cle_err = 0x%08x\n",
			errors, cle_err);
#endif
	}
	return 0;
}

static inline void sam_hw_rdr_ring_submit(struct sam_hw_ring *hw_ring, u32 todo)
{
	u32 val32;

	val32 = SAM_RING_WORD_COUNT_WRITE(todo * SAM_RDR_ENTRY_WORDS);
	sam_hw_reg_write(hw_ring->regs_vbase, HIA_RDR_PREP_COUNT_REG, val32);
}

static inline void sam_hw_cdr_ring_submit(struct sam_hw_ring *hw_ring, u32 todo)
{
	u32 val32;

	val32 = SAM_RING_WORD_COUNT_WRITE(todo * SAM_CDR_ENTRY_WORDS);
	sam_hw_reg_write(hw_ring->regs_vbase, HIA_CDR_COUNT_REG, val32);
}

static inline u32 sam_hw_ring_ready_get(struct sam_hw_ring *hw_ring)
{
	u32 val32;

	val32 = sam_hw_reg_read(hw_ring->regs_vbase, HIA_RDR_PROC_COUNT_REG);

	return SAM_RING_PKT_COUNT_GET(val32);
}

static inline void sam_hw_ring_update(struct sam_hw_ring *hw_ring, u32 done)
{
	u32 val32;

	val32 = SAM_RING_PKT_COUNT_VAL(done);
	val32 |= SAM_RING_WORD_COUNT_WRITE(done * SAM_RDR_ENTRY_WORDS);

	sam_hw_reg_write(hw_ring->regs_vbase, HIA_RDR_PROC_COUNT_REG, val32);
}

static inline void sam_hw_ring_ack_irq(struct sam_hw_ring *hw_ring)
{
	u32 val32;

	val32 = SAM_RDR_STAT_IRQ_MASK;

	sam_hw_reg_write(hw_ring->regs_vbase, HIA_RDR_STAT_REG, val32);
}

static inline void sam_hw_ring_trigger_irq(struct sam_hw_ring *hw_ring)
{
	sam_hw_reg_write(hw_ring->regs_vbase, HIA_RDR_THRESH_REG, hw_ring->rdr_thresh_val);
}

int sam_dma_buf_alloc(u32 buf_size, struct sam_buf_info *dma_buf);
void sam_dma_buf_free(struct sam_buf_info *dma_buf);

bool sam_hw_device_exist(u32 device);
int sam_hw_get_rings_num(u32 device, u32 *rings_map);

int sam_hw_ring_enable_irq(struct sam_hw_ring *hw_ring);
int sam_hw_ring_disable_irq(struct sam_hw_ring *hw_ring);
int sam_hw_ring_prepare_rdr_thresh(struct sam_hw_ring *hw_ring, u32 pkts_coal, u32 usec_coal);

void sam_hw_aic_ring_regs_show(struct sam_hw_ring *hw_ring);
void sam_hw_cdr_regs_show(struct sam_hw_ring *hw_ring);
void sam_hw_rdr_regs_show(struct sam_hw_ring *hw_ring);
void sam_hw_reg_print(char *reg_name, void *base, u32 offset);
int sam_hw_cdr_regs_reset(struct sam_hw_ring *hw_ring);
int sam_hw_cdr_regs_init(struct sam_hw_ring *hw_ring);
int sam_hw_rdr_regs_reset(struct sam_hw_ring *hw_ring);
int sam_hw_rdr_regs_init(struct sam_hw_ring *hw_ring);
int sam_hw_ring_init(u32 device, u32 ring, struct sam_cio_params *params,
		     struct sam_hw_ring *hw_ring);
int sam_hw_ring_deinit(struct sam_hw_ring *hw_ring);
int sam_hw_device_load(void);
int sam_hw_device_unload(void);

#endif /* _SAM_HW_H_ */
