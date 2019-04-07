/*******************************************************************************
 * Copyright (C) Marvell International Ltd. and its affiliates
 *
 * This software file (the "File") is owned and distributed by Marvell
 * International Ltd. and/or its affiliates ("Marvell") under the following
 * alternative licensing terms.  Once you have made an election to distribute the
 * File under one of the following license alternatives, please (i) delete this
 * introductory statement regarding license alternatives, (ii) delete the three
 * license alternatives that you have not elected to use and (iii) preserve the
 * Marvell copyright notice above.
 *
 ********************************************************************************
 * Marvell Commercial License Option
 *
 * If you received this File from Marvell and you have entered into a commercial
 * license agreement (a "Commercial License") with Marvell, the File is licensed
 * to you under the terms of the applicable Commercial License.
 *
 ********************************************************************************
 * Marvell GPL License Option
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File in accordance with the terms and conditions of the General
 * Public License Version 2, June 1991 (the "GPL License"), a copy of which is
 * available along with the File in the license.txt file or by writing to the Free
 * Software Foundation, Inc., or on the worldwide web at http://www.gnu.org/licenses/gpl.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY
 * DISCLAIMED.  The GPL License provides additional details about this warranty
 * disclaimer.
 *
 ********************************************************************************
 * Marvell GNU General Public License FreeRTOS Exception
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File in accordance with the terms and conditions of the Lesser
 * General Public License Version 2.1 plus the following FreeRTOS exception.
 * An independent module is a module which is not derived from or based on
 * FreeRTOS.
 * Clause 1:
 * Linking FreeRTOS statically or dynamically with other modules is making a
 * combined work based on FreeRTOS. Thus, the terms and conditions of the GNU
 * General Public License cover the whole combination.
 * As a special exception, the copyright holder of FreeRTOS gives you permission
 * to link FreeRTOS with independent modules that communicate with FreeRTOS solely
 * through the FreeRTOS API interface, regardless of the license terms of these
 * independent modules, and to copy and distribute the resulting combined work
 * under terms of your choice, provided that:
 * 1. Every copy of the combined work is accompanied by a written statement that
 * details to the recipient the version of FreeRTOS used and an offer by yourself
 * to provide the FreeRTOS source code (including any modifications you may have
 * made) should the recipient request it.
 * 2. The combined work is not itself an RTOS, scheduler, kernel or related
 * product.
 * 3. The independent modules add significant and primary functionality to
 * FreeRTOS and do not merely extend the existing functionality already present in
 * FreeRTOS.
 * Clause 2:
 * FreeRTOS may not be used for any competitive or comparative purpose, including
 * the publication of any form of run time or compile time metric, without the
 * express permission of Real Time Engineers Ltd. (this is the norm within the
 * industry and is intended to ensure information accuracy).
 *
 ********************************************************************************
 * Marvell BSD License Option
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File under the following licensing terms.
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *	* Redistributions of source code must retain the above copyright notice,
 *	  this list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the distribution.
 *
 *	* Neither the name of Marvell nor the names of its contributors may be
 *	  used to endorse or promote products derived from this software without
 *	  specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************/

#ifndef _MVPP2_HW_TYPE_H_
#define _MVPP2_HW_TYPE_H_

#include "pp2_plat.h"

/*All PPV22 Addresses are 40-bit */
#define MVPP22_ADDR_HIGH_SIZE		8
#define MVPP22_ADDR_HIGH_MASK		((1 << MVPP22_ADDR_HIGH_SIZE) - 1)

/*PPV22 ADDRESS SPACE */
#define MVPP2_ADDR_SPACE_SIZE		(64 * 1024)

/* TCLK Frequency */
#define PP2_TCLK_FREQ 333000000

/*TODO*/
/*AXI_BRIDGE*/
/*AXI_CONTEXT*/
/*Top Regfile*/

#define MVPP21_DESC_ADDR_SHIFT			0 /*Applies to RXQ, AGGR_TXQ*/
#define MVPP22_DESC_ADDR_SHIFT			(9 - 1) /*Applies to RXQ, AGGR_TXQ*/

/* RX Fifo Registers */
#define MVPP2_RX_DATA_FIFO_SIZE_REG(port)	(0x00 + 4 * (port))
#define MVPP2_RX_ATTR_FIFO_SIZE_REG(port)	(0x20 + 4 * (port))
#define MVPP2_RX_MIN_PKT_SIZE_REG		0x60
#define MVPP2_RX_FIFO_INIT_REG			0x64

/* RX DMA Top Registers */
#define MVPP2_RX_CTRL_REG(port)			(0x140 + 4 * (port))
#define MVPP2_RX_LOW_LATENCY_PKT_SIZE(s)	(((s) & 0xfff) << 16)
#define MVPP2_RX_GEM_PORT_ID_SRC_SEL(s)		(((s) & 0x7) << 8)
#define MVPP2_RX_USE_PSEUDO_FOR_CSUM_MASK	BIT(31)
#define MVPP2_POOL_BUF_SIZE_REG(pool)		(0x180 + 4 * (pool))
#define MVPP2_POOL_BUF_SIZE_OFFSET		5

/* RXQ_CONFIG_REF Generic+PPV21+PPV22 */
#define MVPP2_RXQ_CONFIG_REG(rxq)		(0x800 + 4 * (rxq))
#define MVPP2_SNOOP_PKT_SIZE_MASK		0x1ff
#define MVPP2_SNOOP_BUF_HDR_MASK		BIT(9)
#define MVPP2_RXQ_PACKET_OFFSET_OFFS		28
#define MVPP2_RXQ_PACKET_OFFSET_MASK		0x70000000
#define MVPP2_RXQ_DISABLE_MASK			BIT(31)

#define MVPP21_RXQ_POOL_SHORT_OFFS		20
#define MVPP21_RXQ_POOL_SHORT_MASK		0x700000
#define MVPP21_RXQ_POOL_LONG_OFFS		24
#define MVPP21_RXQ_POOL_LONG_MASK		0x7000000

#define MVPP22_RXQ_POOL_SHORT_OFFS		20
#define MVPP22_RXQ_POOL_SHORT_MASK		0xf00000
#define MVPP22_RXQ_POOL_LONG_OFFS		24
#define MVPP22_RXQ_POOL_LONG_MASK		0xf000000
#define MVPP22_RXQ_LLC_DEP_HDR_SIZE		0xf000
#define MVPP22_RXQ_LLC_DEP_ENABLE		BIT(16)

#define MVPP21_ETH_RX_HWQ_REG(txq)		(0xc00 + 4 * (txq))
#define MVPP21_ETH_RX_HWQ_POOL_SHORT_OFFS	0
#define MVPP21_ETH_RX_HWQ_POOL_SHORT_MASK	0x7
#define MVPP21_ETH_RX_HWQ_POOL_LONG_OFFS	4
#define MVPP21_ETH_RX_HWQ_POOL_LONG_MASK	0x70
#define MVPP21_ETH_RX_HWQ_DISABLE_MASK		BIT(31)
#define MVPP22_ETH_RX_HWQ_REG(txq)		(0xe00 + 4 * (txq))
#define MVPP22_ETH_RX_HWQ_POOL_SHORT_OFFS	0
#define MVPP22_ETH_RX_HWQ_POOL_SHORT_MASK	0xf
#define MVPP22_ETH_RX_HWQ_POOL_LONG_OFFS	4
#define MVPP22_ETH_RX_HWQ_POOL_LONG_MASK	0xf0
#define MVPP22_ETH_RX_HWQ_DISABLE_MASK		BIT(31)

#define MVPP22_RX_HWF_SNOOP_REG			(0x178)
#define MVPP22_RX_HWF_SNOOP_ENABLE		(BIT(0))

/* AXI Bridge Registers */
#define MVPP22_AXI_BM_WR_ATTR_REG		0x4100
#define MVPP22_AXI_BM_RD_ATTR_REG		0x4104
#define MVPP22_AXI_AGGRQ_DESCR_RD_ATTR_REG	0x4110
#define MVPP22_AXI_TXQ_DESCR_WR_ATTR_REG	0x4114
#define MVPP22_AXI_TXQ_DESCR_RD_ATTR_REG	0x4118
#define MVPP22_AXI_RXQ_DESCR_WR_ATTR_REG	0x411c
#define MVPP22_AXI_RX_DATA_WR_ATTR_REG		0x4120
#define MVPP22_AXI_TX_DATA_RD_ATTR_REG		0x4130

#define MVPP22_AXI_ATTR_CACHE_OFFS		0
#define MVPP22_AXI_ATTR_CACHE_SIZE		4
#define MVPP22_AXI_ATTR_CACHE_MASK		AUTO_MASK(MVPP22_AXI_ATTR_CACHE)

#define MVPP22_AXI_ATTR_QOS_OFFS		4
#define MVPP22_AXI_ATTR_QOS_SIZE		4
#define MVPP22_AXI_ATTR_QOS_MASK		AUTO_MASK(MVPP22_AXI_ATTR_QOS)

#define MVPP22_AXI_ATTR_TC_OFFS			8
#define MVPP22_AXI_ATTR_TC_SIZE			4
#define MVPP22_AXI_ATTR_TC_MASK			AUTO_MASK(MVPP22_AXI_ATTR_TC)

#define MVPP22_AXI_ATTR_DOMAIN_OFFS		12
#define MVPP22_AXI_ATTR_DOMAIN_SIZE		2
#define MVPP22_AXI_ATTR_DOMAIN_MASK		AUTO_MASK(MVPP22_AXI_ATTR_DOMAIN)

#define MVPP22_AXI_ATTR_NON_CACHE		((0x3 << MVPP22_AXI_ATTR_DOMAIN_OFFS) + \
						(0x3 << MVPP22_AXI_ATTR_CACHE_OFFS))

#define MVPP22_AXI_ATTR_SW_COH_WRITE		((0x0 << MVPP22_AXI_ATTR_DOMAIN_OFFS) + \
						(0x7 << MVPP22_AXI_ATTR_CACHE_OFFS))

#define MVPP22_AXI_ATTR_SW_COH_READ		((0x0 << MVPP22_AXI_ATTR_DOMAIN_OFFS) + \
						(0xB << MVPP22_AXI_ATTR_CACHE_OFFS))

#define MVPP22_AXI_ATTR_HW_COH_WRITE		((0x2 << MVPP22_AXI_ATTR_DOMAIN_OFFS) + \
						(0x7 << MVPP22_AXI_ATTR_CACHE_OFFS))

#define MVPP22_AXI_ATTR_HW_COH_READ		((0x2 << MVPP22_AXI_ATTR_DOMAIN_OFFS) + \
						(0xB << MVPP22_AXI_ATTR_CACHE_OFFS))

#define MVPP22_AXI_ATTR_SNOOP_CNTRL_BIT		BIT(16)

#define MVPP22_AXI_RD_NORMAL_CODE_REG		0x4150
#define MVPP22_AXI_RD_SNOOP_CODE_REG		0x4154
#define MVPP22_AXI_WR_NORMAL_CODE_REG		0x4160
#define MVPP22_AXI_WR_SNOOP_CODE_REG		0x4164
#define MVPP22_AXI_WR_DEP_CODE_REG		0x4168

#define MVPP22_AXI_CODE_CACHE_OFFS		0
#define MVPP22_AXI_CODE_CACHE_SIZE		4
#define MVPP22_AXI_CODE_CACHE_MASK		AUTO_MASK(MVPP22_AXI_CODE_CACHE)

#define MVPP22_AXI_CODE_CACHE_NON_CACHE		0x3
#define MVPP22_AXI_CODE_CACHE_RD_CACHE		0xB
#define MVPP22_AXI_CODE_CACHE_WR_CACHE		0x7

#define MVPP22_AXI_CODE_DOMAIN_OFFS		4
#define MVPP22_AXI_CODE_DOMAIN_SIZE		2
#define MVPP22_AXI_CODE_DOMAIN_MASK		AUTO_MASK(MVPP22_AXI_CODE_DOMAIN)

#define MVPP22_AXI_CODE_DOMAIN_OUTER_DOM	2
#define MVPP22_AXI_CODE_DOMAIN_SYSTEM		3
#define MVPP22_AXI_CODE_DOMAIN_NON_SHARE	0


/* Top Reg file */
#define MVPP2_MH_REG(port)			(0x5040 + 4 * (port))

#define MVPP2_MH_EN_OFFS			0
#define MVPP2_MH_EN_MASK			(1 << MVPP2_MH_EN_OFFS)

#define MVPP2_DSA_EN_OFFS			4
#define MVPP2_DSA_EN_MASK			(0x3 << MVPP2_DSA_EN_OFFS)
#define MVPP2_DSA_DISABLE			0
#define MVPP2_DSA_NON_EXTENDED			(0x1 << MVPP2_DSA_EN_OFFS)
#define MVPP2_DSA_EXTENDED			(0x2 << MVPP2_DSA_EN_OFFS)

/* Parser Registers */
#define MVPP2_PRS_INIT_LOOKUP_REG		0x1000
#define MVPP2_PRS_PORT_LU_MAX			0xf
#define MVPP2_PRS_MAX_LOOP_MIN			0x1
#define MVPP2_PRS_PORT_LU_MASK(port)		(0xff << ((port) * 4))
#define MVPP2_PRS_PORT_LU_VAL(port, val)	((val) << ((port) * 4))
#define MVPP2_PRS_INIT_OFFS_REG(port)		(0x1004 + ((port) & 4))
#define MVPP2_PRS_INIT_OFF_MASK(port)		(0x3f << (((port) % 4) * 8))
#define MVPP2_PRS_INIT_OFF_VAL(port, val)	((val) << (((port) % 4) * 8))
#define MVPP2_PRS_INIT_OFF_BITS			6
#define MVPP2_PRS_INIT_OFF_MAX			((1 << MVPP2_PRS_INIT_OFF_BITS) - 1)
#define MVPP2_PRS_MAX_LOOP_REG(port)		(0x100c + ((port) & 4))
#define MVPP2_PRS_MAX_LOOP_MASK(port)		(0xff << (((port) % 4) * 8))
#define MVPP2_PRS_MAX_LOOP_VAL(port, val)	((val) << (((port) % 4) * 8))
#define MVPP2_PRS_TCAM_IDX_REG			0x1100
#define MVPP2_PRS_TCAM_DATA_REG(idx)		(0x1104 + (idx) * 4)
#define MVPP2_PRS_TCAM_INV_MASK			BIT(31)
#define MVPP2_PRS_SRAM_IDX_REG			0x1200
#define MVPP2_PRS_SRAM_DATA_REG(idx)		(0x1204 + (idx) * 4)

#define MVPP2_PRS_EXP_REG			0x1214
#define MVPP2_PRS_EXP_MISS			0
#define MVPP2_PRS_EXP_EXEED			1
#define MVPP2_PRS_EXP_OF			2

#define MVPP2_PRS_TCAM_CTRL_REG			0x1230
#define MVPP2_PRS_TCAM_EN_MASK			BIT(0)
#define MVPP2_PRS_INTR_CAUSE_REG		(0x1020)
#define MVPP2_PRS_INTR_MASK_REG			(0x1024)

/*PPv2.1 MASS 3.20 new feature */
#define MVPP2_PRS_TCAM_HIT_IDX_REG		0x1240
/*----------------------------------------------------------------------*/
/*PPv2.1 MASS 3.20 new feature */
#define MVPP2_PRS_TCAM_HIT_CNT_REG		0x1244
#define MVPP2_PRS_TCAM_HIT_CNT_BITS		16
#define MVPP2_PRS_TCAM_HIT_CNT_OFFS		0
#define MVPP2_PRS_TCAM_HIT_CNT_MASK		\
						(((1 << MVPP2_PRS_TCAM_HIT_CNT_BITS) - 1) << \
						MVPP2_PRS_TCAM_HIT_CNT_OFFS)

/* Classifier Registers */
#define MVPP2_CLS_MODE_REG			0x1800
#define MVPP2_CLS_MODE_ACTIVE_MASK		BIT(0)
#define MVPP2_CLS_PERF_CTRL			0x1804
#define MVPP2_CLS_INSTRUCTIONS_IN_ROUND_OFFS	0
#define MVPP2_CLS_INSTRUCTIONS_IN_ROUND_MASK	0x1f
#define MVPP2_CLS_PORT_WAY_REG			0x1810
#define MVPP2_CLS_PORT_WAY_MASK(port)		(1 << (port))
#define MVPP2_CLS_LKP_INDEX_REG			0x1814
#define MVPP2_CLS_LKP_INDEX_WAY_OFFS		6
#define MVPP2_CLS_LKP_INDEX_LKP_OFFS		0
#define MVPP2_CLS_LKP_TBL_REG			0x1818
#define MVPP2_CLS_LKP_TBL_RXQ_MASK		0xff
#define MVPP2_CLS_LKP_TBL_LOOKUP_EN_MASK	BIT(25)
#define MVPP22_CLS_LKP_TBL_SEL_REG		0x181c
#define MVPP22_CLS_LKP_TBL_SEL_CDT_MASK		BIT(0)
#define MVPP22_CLS_LKP_TBL_SEL_FDT_MASK		BIT(1)
#define MVPP2_CLS_FLOW_INDEX_REG		0x1820
#define MVPP2_CLS_FLOW_TBL0_REG			0x1824
#define MVPP2_CLS_FLOW_TBL1_REG			0x1828
#define MVPP2_CLS_FLOW_TBL2_REG			0x182c

#define MVPP2_CLS_PORT_SPID_REG			0x1830

#define MVPP2_CLS_PORT_SPID_BITS		2
#define MVPP2_CLS_PORT_SPID_MAX			\
						((1 << MVPP2_CLS_PORT_SPID_BITS) - 1)
#define MVPP2_CLS_PORT_SPID_MASK(port)		((MVPP2_CLS_PORT_SPID_MAX) << \
						((port) * MVPP2_CLS_PORT_SPID_BITS))
#define MVPP2_CLS_PORT_SPID_VAL(port, val)	((val) << \
						((port) * MVPP2_CLS_PORT_SPID_BITS))

/* PORT - SPID types */
#define MVPP2_PORT_SPID_MH			0
#define MVPP2_PORT_SPID_EXT_SWITCH		1
#define MVPP2_PORT_SPID_CAS_SWITCH		2
#define MVPP2_PORT_SPID_PORT_TRUNK		3
/*----------------------------------------------------------------------*/

#define MVPP2_CLS_SPID_UNI_BASE_REG		0x1840
#define MVPP2_CLS_SPID_UNI_REG(spid)		(MVPP2_CLS_SPID_UNI_BASE_REG + \
						(((spid) >> 3) * 4))

#define MVPP2_CLS_SPID_MAX			31
#define MVPP2_CLS_SPID_UNI_REGS			4
#define MVPP2_CLS_SPID_UNI_BITS			3
#define MVPP2_CLS_SPID_UNI_FIXED_BITS		4
#define MVPP2_CLS_SPID_UNI_MAX			((1 << \
						MVPP2_CLS_SPID_UNI_BITS) - 1)
#define MVPP2_CLS_SPID_UNI_OFFS(spid)		(((spid) % 8) * \
						MVPP2_CLS_SPID_UNI_FIXED_BITS)
#define MVPP2_CLS_SPID_UNI_MASK(spid)		((MVPP2_CLS_SPID_UNI_MAX) << \
						(MVPP2_CLS_SPID_UNI_OFFS(spid)))
#define MVPP2_CLS_SPID_UNI_VAL(spid, val)	((val) << \
						(MVPP2_CLS_SPID_UNI_OFFS(spid)))

/*----------------------------------------------------------------------*/
#define MVPP2_CLS_GEM_VIRT_INDEX_REG		0x1A00
#define MVPP2_CLS_GEM_VIRT_INDEX_BITS		(7)
#define MVPP2_CLS_GEM_VIRT_INDEX_MAX		(((1 << \
						MVPP2_CLS_GEM_VIRT_INDEX_BITS) - 1) << 0)

/*----------------------------------------------------------------------*/

/* indirect rd/wr via index GEM_VIRT_INDEX */
#define MVPP2_CLS_GEM_VIRT_REGS_NUM		128
#define MVPP2_CLS_GEM_VIRT_REG			0x1A04

#define MVPP2_CLS_GEM_VIRT_BITS			12
#define MVPP2_CLS_GEM_VIRT_MAX			((1 << \
					MVPP2_CLS_GEM_VIRT_BITS) - 1)
#define MVPP2_CLS_GEM_VIRT_MASK			(((1 << \
					MVPP2_CLS_GEM_VIRT_BITS) - 1) << 0)

/*----------------------------------------------------------------------*/
#define MVPP2_CLS_UDF_BASE_REG			0x1860
#define MVPP2_CLS_UDF_REG(index)		(MVPP2_CLS_UDF_BASE_REG + \
						((index) * 4)) /*index <=63*/
#define MVPP2_CLS_UDF_REGS_NUM			64

#define MVPP2_CLS_UDF_BASE_REGS			8
#define MVPP2_CLS_UDF_OFFSET_ID_OFFS		0
#define MVPP2_CLS_UDF_OFFSET_ID_BITS		4
#define MVPP2_CLS_UDF_OFFSET_ID_MAX		((1 << \
					MVPP2_CLS_UDF_OFFSET_ID_BITS) - 1)
#define MVPP2_CLS_UDF_OFFSET_ID_MASK		\
	((MVPP2_CLS_UDF_OFFSET_ID_MAX) << MVPP2_CLS_UDF_OFFSET_ID_OFFS)

#define MVPP2_CLS_UDF_OFFSET_PACKET		0
#define MVPP2_CLS_UDF_OFFSET_L3			1
#define MVPP2_CLS_UDF_OFFSET_2			2
#define MVPP2_CLS_UDF_OFFSET_3			3
#define MVPP2_CLS_UDF_OFFSET_L4			4
#define MVPP2_CLS_UDF_OFFSET_5			5
#define MVPP2_CLS_UDF_OFFSET_6			6
#define MVPP2_CLS_UDF_OFFSET_7			7
#define MVPP2_CLS_UDF_OFFSET_OUTVLAN		8
#define MVPP2_CLS_UDF_OFFSET_INVLAN		9
#define MVPP2_CLS_UDF_OFFSET_ETHTYPE		0xA
#define MVPP2_CLS_UDF_OFFSET_DISABLE		0xF

#define MVPP2_CLS_UDF_REL_OFFSET_OFFS		4
#define MVPP2_CLS_UDF_REL_OFFSET_BITS		11
#define MVPP2_CLS_UDF_REL_OFFSET_MAX		((1 << \
					MVPP2_CLS_UDF_REL_OFFSET_BITS) - 1)
#define MVPP2_CLS_UDF_REL_OFFSET_MASK		\
	((MVPP2_CLS_UDF_REL_OFFSET_MAX) << MVPP2_CLS_UDF_REL_OFFSET_OFFS)

#define MVPP2_CLS_UDF_SIZE_OFFS			16
#define MVPP2_CLS_UDF_SIZE_BITS			8
#define MVPP2_CLS_UDF_SIZE_MIN			1
#define MVPP2_CLS_UDF_SIZE_MAX			((1 << \
					MVPP2_CLS_UDF_SIZE_BITS) - 1)
#define MVPP2_CLS_UDF_SIZE_MASK			(((1 << \
		MVPP2_CLS_UDF_SIZE_BITS) - 1) << MVPP2_CLS_UDF_SIZE_OFFS)
/*----------------------------------------------------------------------*/

#define MVPP2_CLS_MTU_BASE_REG			0x1900
/*  in PPv2.1 (feature MAS 3.7) num indicate an mtu reg index
 * in PPv2.0 num (<=31) indicate eport number , 0-15 pon txq,  16-23 ethernet
 */
#define MVPP2_CLS_MTU_REG(num)			(MVPP2_CLS_MTU_BASE_REG + \
						((num) * 4))
#define MVPP2_CLS_MTU_OFFS			0
#define MVPP2_CLS_MTU_BITS			16
#define MVPP2_CLS_MTU_MAX			((1 << \
					MVPP2_CLS_MTU_BITS) - 1)
#define MVPP2_CLS_MTU_MASK			(((1 << \
			MVPP2_CLS_MTU_BITS) - 1) << MVPP2_CLS_MTU_OFFS)
/*----------------------------------------------------------------------*/

#define MVPP2_CLS_OVERSIZE_RXQ_LOW_REG(port)	(0x1980 + ((port) * 4))
#define MVPP2_CLS_OVERSIZE_RXQ_LOW_BITS		3
#define MVPP2_CLS_OVERSIZE_RXQ_LOW_MASK		0x7
#define MVPP2_CLS_SWFWD_P2HQ_REG(port)		(0x19b0 + ((port) * 4))
#define MVPP2_CLS_SWFWD_PCTRL_REG		0x19d0
#define MVPP2_CLS_SWFWD_PCTRL_MASK(port)	(1 << (port))

/*PPv2.1 new feature MAS 3.14*/
#define MVPP2_CLS_SEQ_SIZE_REG			0x19D4
#define MVPP2_CLS_SEQ_SIZE_BITS			4
#define MVPP2_CLS_SEQ_INDEX_MAX			7
#define MVPP2_CLS_SEQ_SIZE_MAX			8
#define MVPP2_CLS_SEQ_SIZE_MASK(index)		\
				(((1 << MVPP2_CLS_SEQ_SIZE_BITS) - 1) << \
				(MVPP2_CLS_SEQ_SIZE_BITS * (index)))
#define MVPP2_CLS_SEQ_SIZE_VAL(index, val)	((val) << ((index) * \
						MVPP2_CLS_SEQ_SIZE_BITS))

/*PPv2.1 new register MAS 3.18*/
#define MVPP2_CLS_PCTRL_BASE_REG		0x1880
#define MVPP2_CLS_PCTRL_REG(port)		(MVPP2_CLS_PCTRL_BASE_REG + \
						4 * (port))
#define MVPP2_CLS_PCTRL_MH_OFFS			0
#define MVPP2_CLS_PCTRL_MH_BITS			16
#define MVPP2_CLS_PCTRL_MH_MASK			(((1 << \
		MVPP2_CLS_PCTRL_MH_BITS) - 1) << MVPP2_CLS_PCTRL_MH_OFFS)

#define MVPP2_CLS_PCTRL_VIRT_EN_OFFS		16
#define MVPP2_CLS_PCTRL_VIRT_EN_MASK		(1 << \
					MVPP2_CLS_PCTRL_VIRT_EN_OFFS)

#define MVPP2_CLS_PCTRL_UNI_EN_OFFS		17
#define MVPP2_CLS_PCTRL_UNI_EN_MASK		(1 << \
					MVPP2_CLS_PCTRL_UNI_EN_OFFS)

/*----------------------------------------------------------------------*/
/* Policer Registers */
#define MVPP2_PLCR_BASE_PERIOD_REG		0x1304
#define MVPP2_PLCR_BASE_PERIOD_OFFS		0
#define MVPP2_PLCR_BASE_PERIOD_BITS		16
#define MVPP2_PLCR_BASE_PERIOD_ALL_MASK	\
		(((1 << MVPP2_PLCR_BASE_PERIOD_BITS) - 1) << MVPP2_PLCR_BASE_PERIOD_OFFS)
#define MVPP2_PLCR_BASE_PERIOD_MASK(p)		\
		(((p) << MVPP2_PLCR_BASE_PERIOD_OFFS) & MVPP2_PLCR_BASE_PERIOD_ALL_MASK)
#define MVPP2_PLCR_ADD_TOKENS_EN_BIT	16
#define MVPP2_PLCR_ADD_TOKENS_EN_MASK	(1 << MVPP2_PLCR_ADD_TOKENS_EN_BIT)

#define MVPP2_PLCR_MODE_REG		0x1308
#define MVPP2_PLCR_MODE_BITS	3
#define MVPP2_PLCR_MODE_MASK	(((1 << MVPP2_PLCR_MODE_BITS) - 1) << 0)

#define MVPP2_PLCR_TABLE_INDEX_REG		0x130c
#define MVPP2_PLCR_COMMIT_TOKENS_REG	0x1310
#define MVPP2_PLCR_EXCESS_TOKENS_REG	0x1314

#define MVPP2_PLCR_BUCKET_SIZE_REG		0x1318
#define MVPP2_PLCR_COMMIT_SIZE_OFFS		0
#define MVPP2_PLCR_COMMIT_SIZE_BITS		16
#define MVPP2_PLCR_COMMIT_SIZE_ALL_MASK	\
		(((1 << MVPP2_PLCR_COMMIT_SIZE_BITS) - 1) << MVPP2_PLCR_COMMIT_SIZE_OFFS)
#define MVPP2_PLCR_COMMIT_SIZE_MASK(size)	\
		(((size) << MVPP2_PLCR_COMMIT_SIZE_OFFS) & MVPP2_PLCR_COMMIT_SIZE_ALL_MASK)
#define MVPP2_PLCR_EXCESS_SIZE_OFFS		16
#define MVPP2_PLCR_EXCESS_SIZE_BITS		16
#define MVPP2_PLCR_EXCESS_SIZE_ALL_MASK	\
		(((1 << MVPP2_PLCR_EXCESS_SIZE_BITS) - 1) << MVPP2_PLCR_EXCESS_SIZE_OFFS)
#define MVPP2_PLCR_EXCESS_SIZE_MASK(size)	\
		(((size) << MVPP2_PLCR_EXCESS_SIZE_OFFS) & MVPP2_PLCR_EXCESS_SIZE_ALL_MASK)

#define MVPP2_PLCR_TOKEN_CFG_REG		0x131c
#define MVPP2_PLCR_TOKEN_VALUE_OFFS		0
#define MVPP2_PLCR_TOKEN_VALUE_BITS		10
#define MVPP2_PLCR_TOKEN_VALUE_ALL_MASK	\
		(((1 << MVPP2_PLCR_TOKEN_VALUE_BITS) - 1) << MVPP2_PLCR_TOKEN_VALUE_OFFS)
#define MVPP2_PLCR_TOKEN_VALUE_MASK(val)	\
		(((val) << MVPP2_PLCR_TOKEN_VALUE_OFFS) & MVPP2_PLCR_TOKEN_VALUE_ALL_MASK)
#define MVPP2_PLCR_TOKEN_TYPE_OFFS		12
#define MVPP2_PLCR_TOKEN_TYPE_BITS		3
#define MVPP2_PLCR_TOKEN_TYPE_ALL_MASK		\
		(((1 << MVPP2_PLCR_TOKEN_TYPE_BITS) - 1) << MVPP2_PLCR_TOKEN_TYPE_OFFS)
#define MVPP2_PLCR_TOKEN_TYPE_MASK(type)	\
		(((type) << MVPP2_PLCR_TOKEN_TYPE_OFFS) & MVPP2_PLCR_TOKEN_TYPE_ALL_MASK)
#define MVPP2_PLCR_TOKEN_UNIT_BIT		31
#define MVPP2_PLCR_TOKEN_UNIT_MASK		(1 << MVPP2_PLCR_TOKEN_UNIT_BIT)
#define MVPP2_PLCR_TOKEN_UNIT_BYTES		(0 << MVPP2_PLCR_TOKEN_UNIT_BIT)
#define MVPP2_PLCR_TOKEN_UNIT_PKTS		(1 << MVPP2_PLCR_TOKEN_UNIT_BIT)
#define MVPP2_PLCR_COLOR_MODE_BIT		30
#define MVPP2_PLCR_COLOR_MODE_MASK		(1 << MVPP2_PLCR_COLOR_MODE_BIT)
#define MVPP2_PLCR_COLOR_MODE_BLIND		(0 << MVPP2_PLCR_COLOR_MODE_BIT)
#define MVPP2_PLCR_COLOR_MODE_AWARE		(1 << MVPP2_PLCR_COLOR_MODE_BIT)
#define MVPP2_PLCR_ENABLE_BIT			29
#define MVPP2_PLCR_ENABLE_MASK			(1 << MVPP2_PLCR_ENABLE_BIT)

#define MVPP2_PLCR_MIN_PKT_LEN_REG		0x1320
#define MVPP2_PLCR_MIN_PKT_LEN_OFFS		0
#define MVPP2_PLCR_MIN_PKT_LEN_BITS		8
#define MVPP2_PLCR_MIN_PKT_LEN_ALL_MASK	\
		(((1 << MVPP2_PLCR_MIN_PKT_LEN_BITS) - 1) << MVPP2_PLCR_MIN_PKT_LEN_OFFS)
#define MVPP2_PLCR_MIN_PKT_LEN_MASK(len)	\
		(((len) << MVPP2_PLCR_MIN_PKT_LEN_OFFS) & MVPP2_PLCR_MIN_PKT_LEN_ALL_MASK)

#define MVPP2_PLCR_EDROP_EN_REG		0x1330
#define MVPP2_PLCR_EDROP_EN_BIT		0
#define MVPP2_PLCR_EDROP_EN_MASK	(1 << MVPP2_PLCR_EDROP_EN_BIT)

#define MVPP2_PLCR_EDROP_THRESH_NUM		16
#define MVPP2_PLCR_EDROP_TR_OFFS		0
#define MVPP2_PLCR_EDROP_TR_BITS		14
#define MVPP2_PLCR_EDROP_TR_MASK(i)		\
		(((1 << MVPP2_PLCR_EDROP_TR_BITS) - 1) << MVPP2_PLCR_EDROP_TR_OFFS)

#define MVPP2_PLCR_EDROP_CPU_TR_REG(i)		(0x1380 + ((i) * 4))
#define MVPP2_PLCR_EDROP_HWF_TR_REG(i)		(0x13c0 + ((i) * 4))

#define MVPP2_PLCR_EDROP_RXQ_REG		0x1348
#define MVPP2_PLCR_EDROP_RXQ_TR_REG		0x134c

#define MVPP2_PLCR_EDROP_TXQ_REG		0x1358
#define MVPP2_PLCR_EDROP_TXQ_TR_REG		0x135c
/*---------------------------------------------------------------------------------------------*/

#define MVPP2_OVERRUN_DROP_REG(port)		(0x7000 + 4 * (port))
#define MVPP2_CLS_DROP_REG(port)		(0x7020 + 4 * (port))

#define MVPP2_CNT_IDX_REG			0x7040
/* LKP counters index */
#define MVPP2_CNT_IDX_LKP(lkp, way)		((way) << 6 | (lkp))
/* Flow counters index */
#define MVPP2_CNT_IDX_FLOW(index)		(index)
/* TX counters index */
#define MVPP2_CNT_IDX_TX(port, txq)		(((16 + port) << 3) | (txq))

#define MVPP2_TX_DESC_ENQ_REG			0x7100
#define MVPP2_TX_DESC_ENQ_TO_DRAM_REG		0x7104
#define MVPP2_TX_BUF_ENQ_TO_DRAM_REG		0x7108
#define MVPP2_TX_DESC_HWF_ENQ_REG		0x710c
#define MVPP2_RX_DESC_ENQ_REG			0x7120
#define MVPP2_TX_PKT_DQ_REG			0x7130

#define MVPP2_TX_PKT_FULLQ_DROP_REG		0x7200
#define MVPP2_TX_PKT_EARLY_DROP_REG		0x7204
#define MVPP2_TX_PKT_BM_DROP_REG		0x7208
#define MVPP2_TX_PKT_BM_MC_DROP_REG		0x720c
#define MVPP2_RX_PKT_FULLQ_DROP_REG		0x7220
#define MVPP2_RX_PKT_EARLY_DROP_REG		0x7224
#define MVPP2_RX_PKT_BM_DROP_REG		0x7228

#define MVPP2_BM_DROP_CNTR_REG(pool)		(0x7300 + 4 * (pool))
#define MVPP2_BM_MC_DROP_CNTR_REG(pool)		(0x7340 + 4 * (pool))

#define MVPP2_PLCR_GREEN_CNTR_REG(plcr)		(0x7400 + 4 * (plcr))
#define MVPP2_PLCR_YELLOW_CNTR_REG(plcr)	(0x7500 + 4 * (plcr))
#define MVPP2_PLCR_RED_CNTR_REG(plcr)		(0x7600 + 4 * (plcr))

#define MVPP2_CLS_LKP_TBL_HIT_REG		0x7700
#define MVPP2_CLS_FLOW_TBL_HIT_REG		0x7704
#define MVPP2_CLS4_TBL_HIT_REG			0x7708

#define MVPP2_V1_OVERFLOW_MC_DROP_REG		0x770c

/* Classifier C2 Engine Registers */
#define MVPP2_CLS2_TCAM_IDX_REG			0x1B00
#define MVPP2_CLS2_TCAM_DATA_REG(idx)		(0x1B10 + (idx) * 4)
#define MVPP2_CLS2_TCAM_INV_REG			0x1B24
#define MVPP2_CLS2_TCAM_INV_INVALID_OFF		31
#define MVPP2_CLS2_TCAM_INV_INVALID_MASK	BIT(31)
#define MVPP2_CLS2_ACT_DATA_REG			0x1B30
#define MVPP2_CLS2_ACT_DATA_TBL_ID_OFF		0
#define MVPP2_CLS2_ACT_DATA_TBL_ID_MASK		0x3F
#define MVPP2_CLS2_ACT_DATA_TBL_SEL_OFF		6
#define MVPP2_CLS2_ACT_DATA_TBL_SEL_MASK	0x40
#define MVPP2_CLS2_ACT_DATA_TBL_PRI_DSCP_OFF	7
#define MVPP2_CLS2_ACT_DATA_TBL_PRI_DSCP_MASK	0x80
#define MVPP2_CLS2_ACT_DATA_TBL_GEM_ID_OFF	8
#define MVPP2_CLS2_ACT_DATA_TBL_GEM_ID_MASK	0x100
#define MVPP2_CLS2_ACT_DATA_TBL_LOW_Q_OFF	9
#define MVPP2_CLS2_ACT_DATA_TBL_LOW_Q_MASK	0x200
#define MVPP2_CLS2_ACT_DATA_TBL_HIGH_Q_OFF	10
#define MVPP2_CLS2_ACT_DATA_TBL_HIGH_Q_MASK	0x400
#define MVPP2_CLS2_ACT_DATA_TBL_COLOR_OFF	11
#define MVPP2_CLS2_ACT_DATA_TBL_COLOR_MASK	0x800
#define MVPP2_CLS2_DSCP_PRI_INDEX_REG		0x1B40
#define MVPP2_CLS2_DSCP_PRI_INDEX_LINE_OFF	0
#define MVPP2_CLS2_DSCP_PRI_INDEX_LINE_BITS	6
#define MVPP2_CLS2_DSCP_PRI_INDEX_LINE_MASK	0x0000003f
#define MVPP2_CLS2_DSCP_PRI_INDEX_SEL_OFF	6
#define MVPP2_CLS2_DSCP_PRI_INDEX_SEL_MASK	BIT(6)
#define MVPP2_CLS2_DSCP_PRI_INDEX_TBL_ID_OFF	8
#define MVPP2_CLS2_DSCP_PRI_INDEX_TBL_ID_BITS	6
#define MVPP2_CLS2_DSCP_PRI_INDEX_TBL_ID_MASK	0x00003f00
#define MVPP2_CLS2_QOS_TBL_REG			0x1B44
#define MVPP2_CLS2_QOS_TBL_PRI_OFF		0
#define MVPP2_CLS2_QOS_TBL_PRI_BITS		3
#define MVPP2_CLS2_QOS_TBL_PRI_MASK		0x00000007
#define MVPP2_CLS2_QOS_TBL_DSCP_OFF		3
#define MVPP2_CLS2_QOS_TBL_DSCP_BITS		6
#define MVPP2_CLS2_QOS_TBL_DSCP_MASK		0x000001f8
#define MVPP2_CLS2_QOS_TBL_COLOR_OFF		9
#define MVPP2_CLS2_QOS_TBL_COLOR_BITS		3
#define MVPP2_CLS2_QOS_TBL_COLOR_MASK		0x00000e00
#define MVPP2_CLS2_QOS_TBL_GEMPORT_OFF		12
#define MVPP2_CLS2_QOS_TBL_GEMPORT_BITS		12
#define MVPP2_CLS2_QOS_TBL_GEMPORT_MASK		0x00fff000
#define MVPP2_CLS2_QOS_TBL_QUEUENUM_OFF		24
#define MVPP2_CLS2_QOS_TBL_QUEUENUM_BITS	8
#define MVPP2_CLS2_QOS_TBL_QUEUENUM_MASK	0xff000000
#define MVPP2_CLS2_HIT_CTR_REG			0x1B50
#define MVPP2_CLS2_HIT_CTR_OFF			0
#define MVPP2_CLS2_HIT_CTR_BITS			32
#define MVPP2_CLS2_HIT_CTR_MASK			0xffffffff
#define MVPP2_CLS2_HIT_CTR_CLR_REG		0x1B54
#define MVPP2_CLS2_HIT_CTR_CLR_CLR_OFF		0
#define MVPP2_CLS2_HIT_CTR_CLR_CLR_MASK		BIT(0)
#define MVPP2_CLS2_HIT_CTR_CLR_DONE_OFF		1
#define MVPP2_CLS2_HIT_CTR_CLR_DONE_MASK	BIT(1)
#define MVPP2_CLS2_ACT_REG			0x1B60
#define MVPP2_CLS2_ACT_COLOR_OFF		0
#define MVPP2_CLS2_ACT_COLOR_BITS		3
#define MVPP2_CLS2_ACT_COLOR_MASK		0x00000007
#define MVPP2_CLS2_ACT_PRI_OFF			3
#define MVPP2_CLS2_ACT_PRI_BITS			2
#define MVPP2_CLS2_ACT_PRI_MASK			0x00000018
#define MVPP2_CLS2_ACT_DSCP_OFF			5
#define MVPP2_CLS2_ACT_DSCP_BITS		2
#define MVPP2_CLS2_ACT_DSCP_MASK		0x00000060
#define MVPP2_CLS2_ACT_GEM_OFF			7
#define MVPP2_CLS2_ACT_GEM_BITS			2
#define MVPP2_CLS2_ACT_GEM_MASK			0x00000180
#define MVPP2_CLS2_ACT_QL_OFF			9
#define MVPP2_CLS2_ACT_QL_BITS			2
#define MVPP2_CLS2_ACT_QL_MASK			0x00000600
#define MVPP2_CLS2_ACT_QH_OFF			11
#define MVPP2_CLS2_ACT_QH_BITS			2
#define MVPP2_CLS2_ACT_QH_MASK			0x00001800
#define MVPP2_CLS2_ACT_FRWD_OFF			13
#define MVPP2_CLS2_ACT_FRWD_BITS		3
#define MVPP2_CLS2_ACT_FRWD_MASK		0x0000e000
#define MVPP2_CLS2_ACT_PLCR_OFF			16
#define MVPP2_CLS2_ACT_PLCR_BITS		2
#define MVPP2_CLS2_ACT_PLCR_MASK		0x00030000
#define MVPP2_CLS2_ACT_FLD_EN_OFF		18
#define MVPP2_CLS2_ACT_FLD_EN_BITS		1
#define MVPP2_CLS2_ACT_FLD_EN_MASK		0x00040000
#define MVPP2_CLS2_ACT_RSS_OFF			19
#define MVPP2_CLS2_ACT_RSS_BITS			2
#define MVPP2_CLS2_ACT_RSS_MASK			0x00180000
#define MVPP2_CLS2_ACT_QOS_ATTR_REG		0x1B64
#define MVPP2_CLS2_ACT_QOS_ATTR_PRI_OFF		0
#define MVPP2_CLS2_ACT_QOS_ATTR_PRI_BITS	3
#define MVPP2_CLS2_ACT_QOS_ATTR_PRI_MASK	0x00000007
#define MVPP2_CLS2_ACT_QOS_ATTR_PRI_MAX		((1 << MVPP2_CLS2_ACT_QOS_ATTR_PRI_BITS) - 1)
#define MVPP2_CLS2_ACT_QOS_ATTR_DSCP_OFF	3
#define MVPP2_CLS2_ACT_QOS_ATTR_DSCP_BITS	6
#define MVPP2_CLS2_ACT_QOS_ATTR_DSCP_MASK	0x000001f8
#define MVPP2_CLS2_ACT_QOS_ATTR_DSCP_MAX	((1 << MVPP2_CLS2_ACT_QOS_ATTR_DSCP_BITS) - 1)
#define MVPP2_CLS2_ACT_QOS_ATTR_GEM_OFF		9
#define MVPP2_CLS2_ACT_QOS_ATTR_GEM_BITS	12
#define MVPP2_CLS2_ACT_QOS_ATTR_GEM_MASK	0x001ffe00
#define MVPP2_CLS2_ACT_QOS_ATTR_GEM_MAX		((1 << MVPP2_CLS2_ACT_QOS_ATTR_GEM_BITS) - 1)
#define MVPP2_CLS2_ACT_QOS_ATTR_QL_OFF		21
#define MVPP2_CLS2_ACT_QOS_ATTR_QL_BITS		3
#define MVPP2_CLS2_ACT_QOS_ATTR_QL_MASK		0x00e00000
#define MVPP2_CLS2_ACT_QOS_ATTR_QH_OFF		24
#define MVPP2_CLS2_ACT_QOS_ATTR_QH_BITS		5
#define MVPP2_CLS2_ACT_QOS_ATTR_QH_MASK		0x1f000000
#define MVPP2_CLS2_ACT_HWF_ATTR_REG		0x1B68
#define MVPP2_CLS2_ACT_HWF_ATTR_DPTR_OFF	1
#define MVPP2_CLS2_ACT_HWF_ATTR_DPTR_BITS	15
#define MVPP2_CLS2_ACT_HWF_ATTR_DPTR_MASK	0x0000fffe
#define MVPP2_CLS2_ACT_HWF_ATTR_DPTR_MAX	((1 << MVPP2_CLS2_ACT_HWF_ATTR_DPTR_BITS) - 1)
#define MVPP2_CLS2_ACT_HWF_ATTR_IPTR_OFF	16
#define MVPP2_CLS2_ACT_HWF_ATTR_IPTR_BITS	8
#define MVPP2_CLS2_ACT_HWF_ATTR_IPTR_MASK	0x00ff0000
#define MVPP2_CLS2_ACT_HWF_ATTR_IPTR_MAX	((1 << MVPP2_CLS2_ACT_HWF_ATTR_IPTR_BITS) - 1)
#define MVPP2_CLS2_ACT_HWF_ATTR_L4CHK_OFF	24
#define MVPP2_CLS2_ACT_HWF_ATTR_L4CHK_BITS	1
#define MVPP2_CLS2_ACT_HWF_ATTR_L4CHK_MASK	0x01000000
#define MVPP2_CLS2_ACT_HWF_ATTR_MTUIDX_OFF	25
#define MVPP2_CLS2_ACT_HWF_ATTR_MTUIDX_BITS	4
#define MVPP2_CLS2_ACT_HWF_ATTR_MTUIDX_MASK	0x1e000000
#define MVPP2_CLS2_ACT_DUP_ATTR_REG		0x1B6C
#define MVPP2_CLS2_ACT_DUP_ATTR_DUPID_OFF	0
#define MVPP2_CLS2_ACT_DUP_ATTR_DUPID_BITS	8
#define MVPP2_CLS2_ACT_DUP_ATTR_DUPID_MASK	0x000000ff
#define MVPP2_CLS2_ACT_DUP_ATTR_DUPID_MAX	((1 << MVPP2_CLS2_ACT_DUP_ATTR_DUPID_BITS) - 1)
#define MVPP2_CLS2_ACT_DUP_ATTR_DUPCNT_OFF	8
#define MVPP2_CLS2_ACT_DUP_ATTR_DUPCNT_BITS	4
#define MVPP2_CLS2_ACT_DUP_ATTR_DUPCNT_MASK	0x00000f00
#define MVPP2_CLS2_ACT_DUP_ATTR_DUPCNT_MAX	14
#define MVPP2_CLS2_ACT_DUP_ATTR_PLCRID_OFF	24
#define MVPP2_CLS2_ACT_DUP_ATTR_PLCRID_BITS	5
#define MVPP2_CLS2_ACT_DUP_ATTR_PLCRID_MASK	0x1f000000
#define MVPP2_CLS2_ACT_DUP_ATTR_PLCRID_MAX	((1 << MVPP2_CLS2_ACT_DUP_ATTR_PLCRID_BITS) - 1)
#define MVPP2_CLS2_ACT_DUP_ATTR_PLCRBK_OFF	29
#define MVPP2_CLS2_ACT_DUP_ATTR_PLCRBK_BITS	1
#define MVPP2_CLS2_ACT_DUP_ATTR_PLCRBK_MASK	0x20000000
#define MVPP2_CLS2_ACT_DUP_ATTR_RSSEN_OFF	30
#define MVPP2_CLS2_ACT_DUP_ATTR_RSSEN_BITS	1
#define MVPP2_CLS2_ACT_DUP_ATTR_RSSEN_MASK	0x40000000
#define MVPP21_CLS2_ACT_SEQ_ATTR_REG		0x1B70
#define MVPP21_CLS2_ACT_SEQ_ATTR_ID		0
#define MVPP21_CLS2_ACT_SEQ_ATTR_ID_BITS	8
#define MVPP21_CLS2_ACT_SEQ_ATTR_ID_MASK	0x000000ff
#define MVPP21_CLS2_ACT_SEQ_ATTR_ID_MAX		((1 << MVPP21_CLS2_ACT_SEQ_ATTR_ID_BITS) - 1)
#define MVPP21_CLS2_ACT_SEQ_ATTR_MISS_OFF	8
#define MVPP21_CLS2_ACT_SEQ_ATTR_MISS_BITS	1
#define MVPP21_CLS2_ACT_SEQ_ATTR_MISS_MASK	0x00000100
#define MVPP22_CLS2_ACT_SEQ_ATTR_REG		0x1B70
#define MVPP22_CLS2_ACT_SEQ_ATTR_ID		0
#define MVPP22_CLS2_ACT_SEQ_ATTR_ID_BITS	16
#define MVPP22_CLS2_ACT_SEQ_ATTR_ID_MASK	0x0000ffff
#define MVPP22_CLS2_ACT_SEQ_ATTR_MISS_OFF	16
#define MVPP22_CLS2_ACT_SEQ_ATTR_MISS_BITS	1
#define MVPP22_CLS2_ACT_SEQ_ATTR_MISS_MASK	0x0001000
#define MVPP2_CLS2_TCAM_CFG0_REG		0x1b80
#define MVPP2_CLS2_TCAM_CFG0_EN_OFF		0
#define MVPP2_CLS2_TCAM_CFG0_EN_MASK		0x00000001
#define MVPP2_CLS2_TCAM_CFG0_SIZE_OFF		1
#define MVPP2_CLS2_TCAM_CFG0_SIZE_MASK		0x0000001e
#define MVPP2_CLS2_TCAM_CTRL_REG		0x1B90
#define MVPP2_CLS2_TCAM_CTRL_EN_OFF		0
#define MVPP2_CLS2_TCAM_CTRL_EN_MASK		0x0000001

/* Classifier C2 QOS Table (DSCP/PRI Table) */
#define MVPP2_QOS_TBL_LINE_NUM_PRI		8
#define MVPP2_QOS_TBL_NUM_PRI			64
#define MVPP2_QOS_TBL_LINE_NUM_DSCP		64
#define MVPP2_QOS_TBL_NUM_DSCP			8

/*------------------Classifier C3 Top Registers---------------------------*/
#define MVPP2_CLS3_KEY_CTRL_REG		0x1C10
#define KEY_CTRL_L4				0
#define KEY_CTRL_L4_BITS			3
#define KEY_CTRL_L4_MAX				((1 << \
					KEY_CTRL_L4_BITS) - 1)
#define KEY_CTRL_L4_MASK			(((1 << \
				KEY_CTRL_L4_BITS) - 1) << KEY_CTRL_L4)
#define KEY_CTRL_LKP_TYPE			4
#define KEY_CTRL_LKP_TYPE_BITS			6
#define KEY_CTRL_LKP_TYPE_MAX			((1 << \
					KEY_CTRL_LKP_TYPE_BITS) - 1)
#define KEY_CTRL_LKP_TYPE_MASK			(((1 << \
			KEY_CTRL_LKP_TYPE_BITS) - 1) << KEY_CTRL_LKP_TYPE)
#define KEY_CTRL_PRT_ID_TYPE			12
#define KEY_CTRL_PRT_ID_TYPE_BITS		2
#define KEY_CTRL_PRT_ID_TYPE_MAX		((1 << \
					KEY_CTRL_PRT_ID_TYPE_BITS) - 1)
#define KEY_CTRL_PRT_ID_TYPE_MASK		((KEY_CTRL_PRT_ID_TYPE_MAX) << \
					KEY_CTRL_PRT_ID_TYPE)
#define KEY_CTRL_PRT_ID				16
#define KEY_CTRL_PRT_ID_BITS			8
#define KEY_CTRL_PRT_ID_MAX			((1 << \
					KEY_CTRL_PRT_ID_BITS) - 1)
#define KEY_CTRL_PRT_ID_MASK			(((1 << \
			KEY_CTRL_PRT_ID_BITS) - 1) << KEY_CTRL_PRT_ID)
#define KEY_CTRL_HEK_SIZE			24
#define KEY_CTRL_HEK_SIZE_BITS			6
#define KEY_CTRL_HEK_SIZE_MAX			36
#define KEY_CTRL_HEK_SIZE_MASK			(((1 << \
			KEY_CTRL_HEK_SIZE_BITS) - 1) << KEY_CTRL_HEK_SIZE)

#define MVPP2_CLS3_KEY_HEK_REG(reg_num)		(0x1C34 - 4 * (reg_num))

#define MVPP2_CLS3_QRY_ACT_REG			0x1C40
#define MVPP2_CLS3_QRY_ACT			0

#define MVPP2_CLS3_QRY_RES_HASH_REG(hash)	(0x1C50 + 4 * (hash))
#define MVPP2_CLS3_HASH_BANKS_NUM		8

#define MVPP2_CLS3_INIT_HIT_CNT_REG		0x1C80
#define MVPP2_CLS3_INIT_HIT_CNT_OFFS		6
#define MVPP2_CLS3_INIT_HIT_CNT_BITS		18
#define MVPP2_CLS3_INIT_HIT_CNT_MASK		(((1 << \
	MVPP2_CLS3_INIT_HIT_CNT_BITS) - 1) << MVPP2_CLS3_INIT_HIT_CNT_OFFS)
#define MVPP2_CLS3_INIT_HIT_CNT_MAX		((1 << \
					MVPP2_CLS3_INIT_HIT_CNT_BITS) - 1)

#define MVPP2_CLS3_HASH_OP_REG			0x1C84
#define MVPP2_CLS3_HASH_OP_TBL_ADDR		0
#define MVPP2_CLS3_HASH_OP_TBL_ADDR_BITS	12
#define MVPP2_CLS3_HASH_OP_TBL_ADDR_MAX		((1 << \
					MVPP2_CLS3_HASH_OP_TBL_ADDR_BITS) - 1)
#define MVPP2_CLS3_HASH_OP_TBL_ADDR_MASK	\
	((MVPP2_CLS3_HASH_OP_TBL_ADDR_MAX) << MVPP2_CLS3_HASH_OP_TBL_ADDR)
#define MVPP2_CLS3_MISS_PTR			12
#define MVPP2_CLS3_MISS_PTR_MASK		BIT(MVPP2_CLS3_MISS_PTR)
#define MVPP2_CLS3_HASH_OP_DEL			14
#define MVPP2_CLS3_HASH_OP_ADD			15
#define MVPP2_CLS3_HASH_OP_EXT_TBL_ADDR		16
#define MVPP2_CLS3_HASH_OP_EXT_TBL_ADDR_BITS	8
#define MVPP2_CLS3_HASH_OP_EXT_TBL_ADDR_MAX	((1 << \
				MVPP2_CLS3_HASH_OP_EXT_TBL_ADDR_BITS) - 1)
#define MVPP2_CLS3_HASH_OP_EXT_TBL_ADDR_MASK	\
				((MVPP2_CLS3_HASH_OP_EXT_TBL_ADDR_MAX) << \
				MVPP2_CLS3_HASH_OP_EXT_TBL_ADDR)

#define MVPP2_CLS3_STATE_REG			0x1C8C
#define MVPP2_CLS3_STATE_CPU_DONE		0
#define MVPP2_CLS3_STATE_CPU_DONE_MASK		(1 << \
					MVPP2_CLS3_STATE_CPU_DONE)
#define MVPP2_CLS3_STATE_CLEAR_CTR_DONE		1
#define MVPP2_CLS3_STATE_CLEAR_CTR_DONE_MASK	(1 << \
					MVPP2_CLS3_STATE_CLEAR_CTR_DONE)
#define MVPP2_CLS3_STATE_SC_DONE		2
#define MVPP2_CLS3_STATE_SC_DONE_MASK		BIT(MVPP2_CLS3_STATE_SC_DONE)
#define MVPP2_CLS3_STATE_OCCIPIED		8
#define MVPP2_CLS3_STATE_OCCIPIED_BITS		8
#define MVPP2_CLS3_STATE_OCCIPIED_MASK		(((1 << \
	MVPP2_CLS3_STATE_OCCIPIED_BITS) - 1) << MVPP2_CLS3_STATE_OCCIPIED)

#define MVPP2_CLS3_STATE_SC_STATE		16
#define MVPP2_CLS3_STATE_SC_STATE_BITS		2
#define MVPP2_CLS3_STATE_SC_STATE_MASK		(((1 << \
	MVPP2_CLS3_STATE_SC_STATE_BITS) - 1) << MVPP2_CLS3_STATE_SC_STATE)

/* SCAN STATUS
 * 0 - scan compleat
 * 1 -	hit counter clear
 * 3 - scan wait
 * 4 - scan in progress
 */

#define MVPP2_CLS3_STATE_NO_OF_SC_RES		20
#define MVPP2_CLS3_STATE_NO_OF_SC_RES_BITS	9
#define MVPP2_CLS3_STATE_NO_OF_SC_RES_MASK	(((1 << \
				MVPP2_CLS3_STATE_NO_OF_SC_RES_BITS) - 1) << \
				MVPP2_CLS3_STATE_NO_OF_SC_RES)

#define MVPP2_CLS3_DB_INDEX_REG			0x1C90
#define MVPP2_CLS3_DB_MISS_OFFS			12
#define MVPP2_CLS3_DB_MISS_MASK			BIT(MVPP2_CLS3_DB_MISS_OFFS)

						/* 0-3 valid val*/
#define MVPP2_CLS3_HASH_DATA_REG(num)		(0x1CA0 + 4 * (num))
#define MVPP2_CLS3_HASH_DATA_REG_NUM		4
#define MVPP2_CLS3_HASH_EXT_DATA_REG(num)	(0x1CC0 + 4 * (num))
#define MVPP2_CLS3_HASH_EXT_DATA_REG_NUM	7

#define MVPP2_CLS3_CLEAR_COUNTERS_REG		0x1D00
#define MVPP2_CLS3_CLEAR_COUNTERS		0
#define MVPP2_CLS3_CLEAR_COUNTERS_BITS		7
#define MVPP2_CLS3_CLEAR_ALL			0x3f
#define MVPP2_CLS3_CLEAR_COUNTERS_MAX		0x3F
#define MVPP2_CLS3_CLEAR_COUNTERS_MASK		\
					((MVPP2_CLS3_CLEAR_COUNTERS_MAX) << \
					MVPP2_CLS3_CLEAR_COUNTERS)

#define MVPP2_CLS3_HIT_COUNTER_REG		0x1D08
#define MVPP2_CLS3_HIT_COUNTER			0
#define MVPP2_CLS3_HIT_COUNTER_BITS		24
#define MVPP2_CLS3_HIT_COUNTER_MAX		((1 << \
					MVPP2_CLS3_HIT_COUNTER_BITS) - 1)
#define MVPP2_CLS3_HIT_COUNTER_MASK		\
		((MVPP2_CLS3_HIT_COUNTER_MAX) << MVPP2_CLS3_HIT_COUNTER)

#define MVPP2_CLS3_SC_PROP_REG			0x1D10
#define MVPP2_CLS3_SC_PROP_TH_MODE		0
#define MVPP2_CLS3_SC_PROP_TH_MODE_MASK		(1 << \
					MVPP2_CLS3_SC_PROP_TH_MODE)
#define MVPP2_CLS3_SC_PROP_CLEAR		1
#define MVPP2_CLS3_SC_PROP_CLEAR_MASK		(1 << \
					MVPP2_CLS3_SC_PROP_CLEAR)
#define MVPP2_CLS3_SC_PROP_LKP_TYPE_EN		3
#define MVPP2_CLS3_SC_PROP_LKP_TYPE_EN_MASK	(1 << \
					MVPP2_CLS3_SC_PROP_LKP_TYPE_EN)
#define MVPP2_CLS3_SC_PROP_LKP_TYPE		4
#define MVPP2_CLS3_SC_PROP_LKP_TYPE_BITS	6
#define MVPP2_CLS3_SC_PROP_LKP_TYPE_MAX		((1 << \
					MVPP2_CLS3_SC_PROP_LKP_TYPE_BITS) - 1)
#define MVPP2_CLS3_SC_PROP_LKP_TYPE_MASK	\
	((MVPP2_CLS3_SC_PROP_LKP_TYPE_MAX) << MVPP2_CLS3_SC_PROP_LKP_TYPE)
#define MVPP2_CLS3_SC_PROP_START_ENTRY		16
#define MVPP2_CLS3_SC_PROP_START_ENTRY_MASK	\
	((MVPP2_CLS3_HASH_OP_TBL_ADDR_MAX) << MVPP2_CLS3_SC_PROP_START_ENTRY)

#define MVPP2_CLS3_SC_PROP_VAL_REG		0x1D14
#define MVPP2_CLS3_SC_PROP_VAL_DELAY		0
#define MVPP2_CLS3_SC_PROP_VAL_DELAY_BITS	16
#define MVPP2_CLS3_SC_PROP_VAL_DELAY_MAX	((1 << \
					MVPP2_CLS3_SC_PROP_VAL_DELAY_BITS) - 1)
#define MVPP2_CLS3_SC_PROP_VAL_DELAY_MASK	\
	(MVPP2_CLS3_SC_PROP_VAL_DELAY_MAX << MVPP2_CLS3_SC_PROP_VAL_DELAY)

#define MVPP2_CLS3_SC_TH_REG			0x1D18
#define MVPP2_CLS3_SC_TH			4
#define MVPP2_CLS3_SC_TH_BITS			20
#define MVPP2_CLS3_SC_TH_MAX			((1 << \
					MVPP2_CLS3_SC_TH_BITS) - 1)
#define MVPP2_CLS3_SC_TH_MASK			(((1 << \
			MVPP2_CLS3_SC_TH_BITS) - 1) << MVPP2_CLS3_SC_TH)

#define MVPP2_CLS3_SC_TIMER_REG			0x1D1c
#define MVPP2_CLS3_SC_TIMER			0
#define MVPP2_CLS3_SC_TIMER_BITS		16
#define MVPP2_CLS3_SC_TIMER_MASK		\
		(((1 << MVPP2_CLS3_SC_TIMER_BITS) - 1) << MVPP2_CLS3_SC_TIMER)

#define MVPP2_CLS3_SC_ACT_REG			0x1D20
#define MVPP2_CLS3_SC_ACT			0

#define MVPP2_CLS3_SC_INDEX_REG			0x1D28
#define MVPP2_CLS3_SC_INDEX			0

#define MVPP2_CLS3_SC_RES_REG			0x1D2C
#define MVPP2_CLS3_SC_RES_ENTRY			0
#define MVPP2_CLS3_SC_RES_ENTRY_MASK		\
		((MVPP2_CLS3_HASH_OP_TBL_ADDR_MAX) << MVPP2_CLS3_SC_RES_ENTRY)
#define MVPP2_CLS3_SC_RES_CTR			12
#define MVPP2_CLS3_SC_RES_CTR_BITS		20
#define MVPP2_CLS3_SC_RES_CTR_MASK		\
		((MVPP2_CLS3_SC_RES_CTR_BITS) << MVPP2_CLS3_SC_RES_CTR)

#define MVPP2_CLS3_ACT_REG			0x1D40
#define MVPP2_CLS3_ACT_COLOR			0
#define MVPP2_CLS3_ACT_COLOR_BITS		3
#define MVPP2_CLS3_ACT_COLOR_MASK		(((1 << MVPP2_CLS3_ACT_COLOR_BITS) \
						- 1) << MVPP2_CLS3_ACT_COLOR)
#define MVPP2_CLS3_ACT_LOW_Q			9
#define MVPP2_CLS3_ACT_LOW_Q_BITS		2
#define MVPP2_CLS3_ACT_LOW_Q_MASK		(((1 << MVPP2_CLS3_ACT_LOW_Q_BITS) \
						- 1) << MVPP2_CLS3_ACT_LOW_Q)
#define MVPP2_CLS3_ACT_HIGH_Q			11
#define MVPP2_CLS3_ACT_HIGH_Q_BITS		2
#define MVPP2_CLS3_ACT_HIGH_Q_MASK		(((1 << MVPP2_CLS3_ACT_HIGH_Q_BITS) \
						- 1) << MVPP2_CLS3_ACT_HIGH_Q)
#define MVPP2_CLS3_ACT_FWD				13
#define MVPP2_CLS3_ACT_FWD_BITS			3
#define MVPP2_CLS3_ACT_FWD_MASK			(((1 << MVPP2_CLS3_ACT_FWD_BITS) \
						- 1) << MVPP2_CLS3_ACT_FWD)
#define MVPP2_CLS3_ACT_POLICER_SELECT		16
#define MVPP2_CLS3_ACT_POLICER_SELECT_BITS	2
#define MVPP2_CLS3_ACT_POLICER_SELECT_MASK	(((1 << MVPP2_CLS3_ACT_POLICER_SELECT_BITS) \
						- 1) << MVPP2_CLS3_ACT_POLICER_SELECT)
#define MVPP2_CLS3_ACT_FLOW_ID_EN		18
#define MVPP2_CLS3_ACT_FLOW_ID_EN_MASK		BIT(MVPP2_CLS3_ACT_FLOW_ID_EN)
#define MVPP2_CLS3_ACT_RSS_EN			19
#define MVPP2_CLS3_ACT_RSS_EN_BITS		2
#define MVPP2_CLS3_ACT_RSS_EN_MASK		(((1 << MVPP2_CLS3_ACT_RSS_EN_BITS) \
						- 1) << MVPP2_CLS3_ACT_RSS_EN)

#define MVPP2_CLS3_ACT_QOS_ATTR_REG		0x1D44
#define MVPP2_CLS3_ACT_QOS_ATTR_LOW_Q		21
#define MVPP2_CLS3_ACT_QOS_ATTR_LOW_Q_BITS	3
#define MVPP2_CLS3_ACT_QOS_ATTR_LOW_Q_MAX	((1 << MVPP2_CLS3_ACT_QOS_ATTR_LOW_Q_BITS) - 1)
#define MVPP2_CLS3_ACT_QOS_ATTR_LOW_Q_MASK	(MVPP2_CLS3_ACT_QOS_ATTR_LOW_Q_MAX << \
						 MVPP2_CLS3_ACT_QOS_ATTR_LOW_Q)
#define MVPP2_CLS3_ACT_QOS_ATTR_HIGH_Q		24
#define MVPP2_CLS3_ACT_QOS_ATTR_HIGH_Q_BITS	5
#define MVPP2_CLS3_ACT_QOS_ATTR_HIGH_Q_MAX	((1 << MVPP2_CLS3_ACT_QOS_ATTR_HIGH_Q_BITS) - 1)
#define MVPP2_CLS3_ACT_QOS_ATTR_HIGH_Q_MASK	(MVPP2_CLS3_ACT_QOS_ATTR_HIGH_Q_MAX << \
						 MVPP2_CLS3_ACT_QOS_ATTR_HIGH_Q)

#define MVPP2_CLS3_ACT_QOS_ATTR_Q_MAX		((1 << (MVPP2_CLS3_ACT_QOS_ATTR_HIGH_Q_BITS + \
						 MVPP2_CLS3_ACT_QOS_ATTR_LOW_Q_BITS)) - 1)

#define MVPP2_CLS3_ACT_HWF_ATTR_REG		0x1D48
#define	MVPP2_CLS3_ACT_HWF_ATTR_DPTR		1
#define	MVPP2_CLS3_ACT_HWF_ATTR_DPTR_BITS	15
#define	MVPP2_CLS3_ACT_HWF_ATTR_DPTR_MASK	(((1 << MVPP2_CLS3_ACT_HWF_ATTR_DPTR_BITS) - 1) << \
						 MVPP2_CLS3_ACT_HWF_ATTR_DPTR)
#define	MVPP2_CLS3_ACT_HWF_ATTR_DPTR_MAX	((1 << MVPP2_CLS3_ACT_HWF_ATTR_DPTR_BITS) - 1)
#define	MVPP2_CLS3_ACT_HWF_ATTR_IPTR		16
#define	MVPP2_CLS3_ACT_HWF_ATTR_IPTR_BITS	8
#define	MVPP2_CLS3_ACT_HWF_ATTR_IPTR_MASK	(((1 << MVPP2_CLS3_ACT_HWF_ATTR_IPTR_BITS) - 1) << \
						 MVPP2_CLS3_ACT_HWF_ATTR_IPTR)
#define	MVPP2_CLS3_ACT_HWF_ATTR_IPTR_MAX	((1 << MVPP2_CLS3_ACT_HWF_ATTR_IPTR_BITS) - 1)

#define	MVPP2_CLS3_ACT_HWF_ATTR_CHKSM_EN	24
#define	MVPP2_CLS3_ACT_HWF_ATTR_CHKSM_EN_MASK	BIT(MVPP2_CLS3_ACT_HWF_ATTR_CHKSM_EN)

#define MVPP2_CLS3_ACT_HWF_ATTR_MTU_INX		25
#define MVPP2_CLS3_ACT_HWF_ATTR_MTU_INX_BITS	4
#define MVPP2_CLS3_ACT_HWF_ATTR_MTU_INX_MAX	((1 << MVPP2_CLS3_ACT_HWF_ATTR_MTU_INX_BITS) - 1)
#define	MVPP2_CLS3_ACT_HWF_ATTR_MTU_INX_MASK	((MVPP2_CLS3_ACT_HWF_ATTR_MTU_INX_MAX) << \
						 MVPP2_CLS3_ACT_HWF_ATTR_MTU_INX)

#define MVPP2_CLS3_ACT_DUP_ATTR_REG		0x1D4C
#define MVPP2_CLS3_ACT_DUP_FID			0
#define MVPP2_CLS3_ACT_DUP_FID_BITS		8
#define MVPP2_CLS3_ACT_DUP_FID_MASK		(((1 << MVPP2_CLS3_ACT_DUP_FID_BITS) - 1) << \
						MVPP2_CLS3_ACT_DUP_FID)
#define MVPP2_CLS3_ACT_DUP_FID_MAX		((1 << MVPP2_CLS3_ACT_DUP_FID_BITS) - 1)

#define MVPP2_CLS3_ACT_DUP_COUNT		8
#define MVPP2_CLS3_ACT_DUP_COUNT_BITS		4
#define MVPP2_CLS3_ACT_DUP_COUNT_MASK		(((1 << MVPP2_CLS3_ACT_DUP_COUNT_BITS) - 1) << \
						 MVPP2_CLS3_ACT_DUP_COUNT)
#define MVPP2_CLS3_ACT_DUP_COUNT_MAX		14

#define MVPP2_CLS3_ACT_DUP_POLICER_ID		24
#define MVPP2_CLS3_ACT_DUP_POLICER_ID_BITS	5
#define MVPP2_CLS3_ACT_DUP_POLICER_MASK		(((1 << MVPP2_CLS3_ACT_DUP_POLICER_ID_BITS) \
						 - 1) << MVPP2_CLS3_ACT_DUP_POLICER_ID)
#define MVPP2_CLS3_ACT_DUP_POLICER_MAX		((1 << MVPP2_CLS3_ACT_DUP_POLICER_ID_BITS) - 1)

#define MVPP2_CLS3_ACT_DUP_POLICER_BANK_BIT	29
#define MVPP2_CLS3_ACT_DUP_POLICER_BANK_MASK	BIT(MVPP2_CLS3_ACT_DUP_POLICER_BANK_BIT)

#define MVPP2_CLS3_ACT_DUP_RSS_EN_BIT		30
#define MVPP2_CLS3_ACT_DUP_RSS_EN_MASK		BIT(MVPP2_CLS3_ACT_DUP_RSS_EN_BIT)

#define MVPP2_CLS3_ACT_SEQ_L_ATTR_REG		0x1D50
#define MVPP2_CLS3_ACT_SEQ_H_ATTR_REG		0x1D54
#define MVPP2_CLS3_ACT_SEQ_SIZE			38

/* Descriptor Manager Top Registers */
#define MVPP2_RXQ_NUM_REG			0x2040

#define MVPP2_RXQ_DESC_ADDR_REG			0x2044
#define MVPP21_RXQ_DESC_ADDR_SHIFT		MVPP21_DESC_ADDR_SHIFT
#define MVPP21_RXQ_DESC_ADDR_MASK		0xfffffe00

#define MVPP22_RXQ_DESC_ADDR_SHIFT		MVPP22_DESC_ADDR_SHIFT
#define MVPP22_RXQ_DESC_ADDR_MASK		0xfffffffe

#define MVPP2_RXQ_DESC_SIZE_REG			0x2048
#define MVPP2_RXQ_DESC_SIZE_MASK		0x3ff0
#define MVPP2_RXQ_STATUS_UPDATE_REG(rxq)	(0x3000 + 4 * (rxq))
#define MVPP2_RXQ_NUM_PROCESSED_OFFSET		0
#define MVPP2_RXQ_NUM_NEW_OFFSET		16
#define MVPP2_RXQ_STATUS_REG(rxq)		(0x3400 + 4 * (rxq))
#define MVPP2_RXQ_OCCUPIED_MASK			0x3fff
#define MVPP2_RXQ_NON_OCCUPIED_OFFSET		16
#define MVPP2_RXQ_NON_OCCUPIED_MASK		0x3fff0000
#define MVPP2_RXQ_THRESH_REG			0x204c
#define MVPP2_OCCUPIED_THRESH_OFFSET		0
#define MVPP2_OCCUPIED_THRESH_MASK		0x3fff
#define MVPP2_NONOCCUPIED_THRESH_OFFSET		16
#define MVPP2_NONOCCUPIED_THRESH_MASK		(0x3fff << MVPP2_NONOCCUPIED_THRESH_OFFSET)

#define MVPP2_MAX_OCCUPIED_THRESH		(MVPP2_OCCUPIED_THRESH_MASK)
#define MVPP2_RXQ_INDEX_REG			0x2050
#define MVPP2_TXQ_NUM_REG			0x2080
#define MVPP2_TXQ_DESC_ADDR_LOW_REG		0x2084
#define MVPP2_TXQ_DESC_ADDR_LOW_SHIFT		0
#define MVPP2_TXQ_DESC_ADDR_LOW_MASK		0xfffffe00
#define MVPP22_TXQ_DESC_ADDR_HIGH_REG		0x20a8
#define MVPP22_TXQ_DESC_ADDR_HIGH_MASK		0xff
#define MVPP2_TXQ_DESC_SIZE_REG			0x2088
#define MVPP2_TXQ_DESC_HWF_SIZE_REG		0x208c
#define MVPP2_TXQ_DESC_SIZE_MASK		0x3ff0
#define MVPP2_AGGR_TXQ_UPDATE_REG		0x2090
#define MVPP2_TXQ_THRESH_REG			0x2094
#define MVPP2_TRANSMITTED_THRESH_OFFSET		16
#define MVPP2_TRANSMITTED_THRESH_MASK		0x3fff0000
#define MVPP2_TXQ_INDEX_REG			0x2098
#define MVPP2_TXQ_PREF_BUF_REG			0x209c
#define MVPP2_PREF_BUF_PTR(desc)		((desc) & 0xfff)
#define MVPP2_PREF_BUF_SIZE_4			(BIT(12) | BIT(13))
#define MVPP2_PREF_BUF_SIZE_16			(BIT(12) | BIT(14))
#define MVPP2_PREF_BUF_SIZE_32			(BIT(13) | BIT(14))
#define MVPP2_PREF_BUF_SIZE_64			(BIT(12) | BIT(13) | BIT(14))
#define MVPP2_PREF_BUF_THRESH(val)		((val) << 17)
#define MVPP2_TXQ_DRAIN_EN_MASK			BIT(31)
#define MVPP2_TXQ_PENDING_REG			0x20a0
#define MVPP2_TXQ_PENDING_MASK			0x3fff
#define MVPP2_TXQ_INT_STATUS_REG		0x20a4

#define MVPP21_TXQ_SENT_REG(txq)		(0x3c00 + 4 * (txq))
#define MVPP21_TRANSMITTED_COUNT_OFFSET		16
#define MVPP21_TRANSMITTED_COUNT_MASK		0x3fff0000
#define MVPP22_TXQ_SENT_REG(txq)		(0x3e00 + 4 * (txq - 128))
#define MVPP22_TRANSMITTED_COUNT_OFFSET		16
#define MVPP22_TRANSMITTED_COUNT_MASK		0x3fff0000

#define MVPP2_TXQ_RSVD_REQ_REG			0x20b0
#define MVPP2_TXQ_RSVD_REQ_Q_OFFSET		16
#define MVPP2_TXQ_RSVD_RSLT_REG			0x20b4
#define MVPP2_TXQ_RSVD_RSLT_MASK		0x3fff
#define MVPP2_TXQ_RSVD_CLR_REG			0x20b8
#define MVPP2_TXQ_RSVD_CLR_OFFSET		16
#define MVPP2_AGGR_TXQ_DESC_ADDR_REG(cpu)	(0x2100 + 4 * (cpu))
#define MVPP21_AGGR_TXQ_DESC_ADDR_SHIFT		MVPP21_DESC_ADDR_SHIFT
#define MVPP21_AGGR_TXQ_DESC_ADDR_MASK		0xfffffe00
#define MVPP22_AGGR_TXQ_DESC_ADDR_SHIFT		MVPP22_DESC_ADDR_SHIFT
#define MVPP22_AGGR_TXQ_DESC_ADDR_MASK		0xfffffffe

#define MVPP2_AGGR_TXQ_INIT(cpu)          (0x20C0 + 4 * (cpu))
#define MVPP2_AGGR_TXQ_DESC_SIZE_REG(cpu)	(0x2140 + 4 * (cpu))
#define MVPP2_AGGR_TXQ_DESC_SIZE_MASK		0x3ff0
#define MVPP2_AGGR_TXQ_STATUS_REG(cpu)		(0x2180 + 4 * (cpu))
#define MVPP2_AGGR_TXQ_PENDING_MASK		0x3fff
#define MVPP2_AGGR_TXQ_INDEX_REG(cpu)		(0x21c0 + 4 * (cpu))

/* MBUS bridge registers */
#define MVPP2_WIN_BASE(w)			(0x4000 + ((w) << 2))
#define MVPP2_WIN_SIZE(w)			(0x4020 + ((w) << 2))
#define MVPP2_WIN_REMAP(w)			(0x4040 + ((w) << 2))
#define MVPP2_BASE_ADDR_ENABLE			0x4060

/* Interrupt Cause and Mask registers */
#define MVPP22_ISR_TX_THRESHOLD_REG(port)	(0x5140 + 4 * (port))
#define MVPP22_ISR_TX_THRESHOLD_MASK		0xfffff0

#define MVPP2_ISR_RX_THRESHOLD_REG(rxq)		(0x5200 + 4 * (rxq))
#define MVPP2_ISR_RX_THRESHOLD_MASK		0xfffff0
#define MVPP2_MAX_ISR_RX_THRESHOLD		MVPP2_ISR_RX_THRESHOLD_MASK

#define MVPP21_ISR_RXQ_GROUP_REG(port)		(0x5400 + 4 * (port))
#define MVPP22_ISR_RXQ_GROUP_INDEX_REG		0x5400
#define MVPP22_ISR_RXQ_GROUP_INDEX_SUBGROUP_MASK 0xf
#define MVPP22_ISR_RXQ_GROUP_INDEX_GROUP_MASK   0x380
#define MVPP22_ISR_RXQ_GROUP_INDEX_GROUP_OFFSET 7

#define MVPP22_ISR_RXQ_GROUP_INDEX_SUBGROUP_MASK 0xf
#define MVPP22_ISR_RXQ_GROUP_INDEX_GROUP_MASK   0x380

#define MVPP22_ISR_RXQ_SUB_GROUP_CONFIG_REG	0x5404
#define MVPP22_ISR_RXQ_SUB_GROUP_STARTQ_MASK	0x1f
#define MVPP22_ISR_RXQ_SUB_GROUP_SIZE_MASK	0xf00
#define MVPP22_ISR_RXQ_SUB_GROUP_SIZE_OFFSET	8

#define MVPP2_ISR_ENABLE_REG(port)		(0x5420 + 4 * (port))
#define MVPP2_ISR_ENABLE_INTERRUPT(mask)	((mask) & 0xffff)
#define MVPP2_ISR_DISABLE_INTERRUPT(mask)	(((mask) << 16) & 0xffff0000)
#define MVPP2_ISR_RX_TX_CAUSE_REG(eth_port)	(0x5480 + 4 * (eth_port))
#define MVPP2_CAUSE_RXQ_OCCUP_DESC_ALL_MASK	0x00ff
#define MVPP2_CAUSE_TXQ_OCCUP_DESC_ALL_MASK	0xff0000
#define MVPP2_CAUSE_TXQ_OCCUP_DESC_ALL_OFFSET	16

#define MVPP2_CAUSE_RX_FIFO_OVERRUN_MASK	BIT(24)
#define MVPP2_CAUSE_FCS_ERR_MASK		BIT(25)
#define MVPP2_CAUSE_TX_FIFO_UNDERRUN_MASK	BIT(26)
#define MVPP2_CAUSE_TX_EXCEPTION_SUM_MASK	BIT(29)
#define MVPP2_CAUSE_RX_EXCEPTION_SUM_MASK	BIT(30)
#define MVPP2_CAUSE_MISC_SUM_MASK		BIT(31)

#define MVPP2_ISR_RX_TX_MASK_REG(port)		(0x54a0 + 4 * (port))
#define MVPP2_ISR_PON_RX_TX_MASK_REG		0x54bc
#define MVPP2_PON_CAUSE_RXQ_OCCUP_DESC_ALL_MASK	0xffff
#define MVPP2_PON_CAUSE_TXP_OCCUP_DESC_ALL_MASK	0x3fc00000
#define MVPP2_PON_CAUSE_MISC_SUM_MASK		BIT(31)

#define MVPP2_ISR_RX_ERR_CAUSE_REG(port)	(0x5500 + 4 * (port))
#define MVPP2_ISR_RX_ERR_CAUSE_NONOCC_MASK	0x00ff
#define MVPP2_ISR_RX_ERR_CAUSE_DESC_RES_MASK	0xff0000
#define MVPP2_RX_EX_INT_CAUSE_MASK_REG(port)	(0x5520 + 4 * (port))

#define MVPP2_ISR_TX_ERR_CAUSE_REG(eth_port)	(0x5540 + 4 * (eth_port))
#define MVPP2_ISR_TX_ERR_MASK_REG(eth_port)	(0x5560 + 4 * (eth_port))

#define MVPP2_ISR_MISC_CAUSE_REG		0x55b0
#define MVPP2_ISR_MISC_MASK_REG			0x55b4

#define MVPP2_ISR_NO_BUF_CAUSE_REG		0x55b8
#define MVPP2_ISR_NO_BUF_MASK_REG		0x55bc

/* Buffer Manager registers */
#define MVPP2_BM_POOL_BASE_ADDR_REG(pool)	(0x6000 + ((pool) * 4))
#define MVPP2_BM_POOL_BASE_ADDR_MASK		0xFFFFFF80
#define MVPP2_BM_POOL_SIZE_REG(pool)		(0x6040 + ((pool) * 4))
#define MVPP21_BM_POOL_SIZE_MASK		0xfff0
#define MVPP21_BM_POOL_SIZE_OFFSET		4

#define MVPP2_BM_POOL_READ_PTR_REG(pool)	(0x6080 + ((pool) * 4))
#define MVPP21_BM_POOL_READ_PTR_REG		MVPP2_BM_POOL_READ_PTR_REG

#define MVPP21_BM_POOL_GET_READ_PTR_MASK	0xfff0
#define MVPP2_BM_POOL_PTRS_NUM_REG(pool)	(0x60c0 + ((pool) * 4))
#define MVPP21_BM_POOL_PTRS_NUM_REG		MVPP2_BM_POOL_PTRS_NUM_REG

#define MVPP21_BM_POOL_PTRS_NUM_MASK		0xfff0

#define MVPP22_BM_POOL_SIZE_MASK		0xfff8
#define MVPP22_BM_POOL_SIZE_OFFSET		3

/* Use PPV21 Pool Size both for PPV21/PPV22, deliberately ignore PPV22 */
#define MVPP2_BM_POOL_SIZE_MASK			MVPP21_BM_POOL_SIZE_MASK
#define MVPP2_BM_POOL_SIZE_OFFSET		MVPP21_BM_POOL_SIZE_OFFSET
#undef  MVPP22_BM_POOL_SIZE_MASK
#undef	MVPP22_BM_POOL_SIZE_OFFSET

#define MVPP22_BM_POOL_READ_PTR_REG		MVPP2_BM_POOL_READ_PTR_REG
#define MVPP22_BM_POOL_GET_READ_PTR_MASK	0xfff8
#define MVPP22_BM_POOL_PTRS_NUM_REG		MVPP2_BM_POOL_PTRS_NUM_REG
#define MVPP22_BM_POOL_PTRS_NUM_MASK		0xfff8

#define MVPP2_BM_BPPI_READ_PTR_REG(pool)	(0x6100 + ((pool) * 4))
#define MVPP2_BM_BPPI_PTRS_NUM_REG(pool)	(0x6140 + ((pool) * 4))
#define MVPP2_BM_BPPI_PTR_NUM_MASK		0x7ff
#define MVPP2_BM_BPPI_PREFETCH_FULL_MASK	BIT(16)
#define MVPP2_BM_POOL_CTRL_REG(pool)		(0x6200 + ((pool) * 4))
#define MVPP2_BM_START_MASK			BIT(0)
#define MVPP2_BM_STOP_MASK			BIT(1)
#define MVPP2_BM_STATE_MASK			BIT(4)
#define MVPP2_BM_LOW_THRESH_OFFS		8
#define MVPP2_BM_LOW_THRESH_MASK		0x7f00
#define MVPP2_BM_LOW_THRESH_VALUE(val)		((val) << \
						MVPP2_BM_LOW_THRESH_OFFS)
#define MVPP2_BM_HIGH_THRESH_OFFS		16
#define MVPP2_BM_HIGH_THRESH_MASK		0x7f0000
#define MVPP2_BM_HIGH_THRESH_VALUE(val)		((val) << \
						MVPP2_BM_HIGH_THRESH_OFFS)

#define MVPP2_BM_BPPI_HIGH_THRESH		0x1E
#define MVPP2_BM_BPPI_LOW_THRESH		0x1C
#define MVPP23_BM_BPPI_8POOL_HIGH_THRESH	0x34
#define MVPP23_BM_BPPI_8POOL_LOW_THRESH		0x28

#define MVPP2_BM_INTR_CAUSE_REG(pool)		(0x6240 + ((pool) * 4))
#define MVPP2_BM_RELEASED_DELAY_MASK		BIT(0)
#define MVPP2_BM_ALLOC_FAILED_MASK		BIT(1)
#define MVPP2_BM_BPPE_EMPTY_MASK		BIT(2)
#define MVPP2_BM_BPPE_FULL_MASK			BIT(3)
#define MVPP2_BM_AVAILABLE_BP_LOW_MASK		BIT(4)
#define MVPP2_BM_INTR_MASK_REG(pool)		(0x6280 + ((pool) * 4))

#define MVPP21_BM_UNUSED_PTR_THRESH_REG(pool)	(0x62c0 + ((pool) * 4))
#define MVPP21_BM_UNUSED_PTR_THRESH_MASK	0xfff0
#define MVPP22_BM_UNUSED_PTR_THRESH_REG(pool)	(0x62c0 + ((pool) * 4))
#define MVPP22_BM_UNUSED_PTR_THRESH_MASK	0xfff8

#define MVPP22_BM_POOL_BASE_ADDR_HIGH_REG	0x6310
#define MVPP22_BM_POOL_BASE_ADDR_HIGH_MASK	0xff
#define MVPP23_BM_8POOL_MODE			BIT(8)


#define MVPP2_BM_PHY_ALLOC_REG(pool)		(0x6400 + ((pool) * 4))
#define MVPP2_BM_PHY_ALLOC_GRNTD_MASK		BIT(0)
#define MVPP2_BM_VIRT_ALLOC_REG			0x6440

#define MVPP22_BM_PHY_VIRT_HIGH_ALLOC_REG	0x6444
#define MVPP22_BM_PHY_HIGH_ALLOC_OFFSET		0
#define MVPP22_BM_VIRT_HIGH_ALLOC_OFFSET	8
#define MVPP22_BM_VIRT_HIGH_ALLOC_MASK		0xff00

#define MVPP2_BM_PHY_RLS_REG(pool)		(0x6480 + ((pool) * 4))
#define MVPP2_BM_PHY_RLS_MC_BUFF_MASK		BIT(0)
#define MVPP2_BM_PHY_RLS_PRIO_EN_MASK		BIT(1)
#define MVPP2_BM_PHY_RLS_GRNTD_MASK		BIT(2)

#define MVPP2_BM_VIRT_RLS_REG			0x64c0

#define MVPP21_BM_MC_RLS_REG			0x64c4 /* Not a mixup */
#define MVPP21_BM_MC_ID_MASK			0xfff
#define MVPP21_BM_FORCE_RELEASE_MASK		BIT(12)

#define MVPP22_BM_PHY_VIRT_HIGH_RLS_REG		0x64c4 /* Not a mixup */

#define MVPP22_BM_PHY_HIGH_RLS_OFFSET		0
#define MVPP22_BM_VIRT_HIGH_RLS_OFFST		8

#define MVPP22_BM_MC_RLS_REG			0x64d4 /* Not a mixup */
#define MVPP22_BM_MC_ID_MASK			0xfff
#define MVPP22_BM_FORCE_RELEASE_MASK		BIT(12)

#define MVPP2_BM_PRIO_CTRL_REG			0x6800

#define MVPP2_BM_PRIO_IDX_REG			0x6810
#define MVPP2_BM_PRIO_IDX_BITS			8
#define MVPP2_BM_PRIO_IDX_MAX			255
#define MVPP2_BM_PRIO_IDX_MASK			0xff

#define MVPP2_BM_CPU_QSET_REG			0x6814

#define MVPP2_BM_CPU_SHORT_QSET_OFFS		0
#define MVPP2_BM_CPU_SHORT_QSET_MASK		(0x7f << \
					MVPP2_BM_CPU_SHORT_QSET_OFFS)

#define MVPP2_BM_CPU_LONG_QSET_OFFS		8
#define MVPP2_BM_CPU_LONG_QSET_MASK		(0x7f << \
					MVPP2_BM_CPU_LONG_QSET_OFFS)

#define MVPP2_BM_HWF_QSET_REG			0x6818

#define MVPP2_BM_HWF_SHORT_QSET_OFFS		0
#define MVPP2_BM_HWF_SHORT_QSET_MASK		(0x7f << \
					MVPP2_BM_HWF_SHORT_QSET_OFFS)

#define MVPP2_BM_HWF_LONG_QSET_OFFS		8
#define MVPP2_BM_HWF_LONG_QSET_MASK		(0x7f << \
					MVPP2_BM_HWF_LONG_QSET_OFFS)

#define MVPP2_BM_QSET_SET_MAX_REG		0x6820

#define MVPP2_BM_QSET_MAX_SHARED_OFFS		0
#define MVPP2_BM_QSET_MAX_GRNTD_OFFS		16

#define MVPP2_BM_QSET_MAX_SHARED_MASK		(0xffff << \
					MVPP2_BM_QSET_MAX_SHARED_OFFS)
#define MVPP2_BM_QSET_MAX_GRNTD_MASK		(0xffff << \
					MVPP2_BM_QSET_MAX_GRNTD_OFFS)

#define MVPP2_BM_QSET_SET_CNTRS_REG		0x6824

/* TX Scheduler registers */
#define MVPP2_TX_PORT_NUM(port)			(0x10 | port)
#define MVPP2_TXP_SCHED_PORT_INDEX_REG		0x8000
#define MVPP2_TXP_SCHED_Q_CMD_REG		0x8004
#define MVPP2_TXP_SCHED_ENQ_MASK		0xff
#define MVPP2_TXP_SCHED_DISQ_OFFSET		8
#define MVPP2_TXP_SCHED_CMD_1_REG		0x8010
#define MVPP2_TXP_SCHED_FIXED_PRIO_REG		0x8014
#define MVPP2_TXP_SCHED_PERIOD_REG		0x8018
#define MVPP2_TXP_SCHED_MTU_REG			0x801c
#define MVPP2_TXP_MTU_MAX			0x7FFFF
#define MVPP2_TXP_SCHED_REFILL_REG		0x8020
#define MVPP2_TXP_REFILL_TOKENS_OFFS		0
#define MVPP2_TXP_REFILL_TOKENS_MAX		0x7FFFF
#define MVPP2_TXP_REFILL_TOKENS_ALL_MASK	0x7ffff
#define MVPP2_TXP_REFILL_TOKENS_MASK(val)	((val) << \
					MVPP2_TXP_REFILL_TOKENS_OFFS)
#define MVPP2_TXP_REFILL_PERIOD_MAX		0x3FF
#define MVPP2_TXP_REFILL_PERIOD_ALL_MASK	0x3ff00000
#define MVPP2_TXP_REFILL_PERIOD_MASK(v)		((v) << 20)
#define MVPP2_TXP_SCHED_TOKEN_SIZE_REG		0x8024
#define MVPP2_TXP_SCHED_TOKEN_CNTR_REG		0x8028
#define MVPP2_TXP_TOKEN_SIZE_MAX		0xffffffff
#define MVPP2_TXQ_SCHED_REFILL_REG(q)		(0x8040 + ((q) << 2))
#define MVPP2_TXQ_REFILL_TOKENS_OFFS		0
#define MVPP2_TXQ_REFILL_TOKENS_MAX		0x7FFFF
#define MVPP2_TXQ_REFILL_TOKENS_ALL_MASK	0x7ffff
#define MVPP2_TXQ_REFILL_TOKENS_MASK(val)	((val) << \
					MVPP2_TXQ_REFILL_TOKENS_OFFS)
#define MVPP2_TXQ_REFILL_PERIOD_MAX		0x3FF
#define MVPP2_TXQ_REFILL_PERIOD_ALL_MASK	0x3ff00000
#define MVPP2_TXQ_REFILL_PERIOD_MASK(v)		((v) << 20)
#define MVPP2_TXQ_SCHED_TOKEN_SIZE_REG(q)	(0x8060 + ((q) << 2))
#define MVPP2_TXQ_TOKEN_SIZE_MAX		0x7fffffff
#define MVPP2_TXQ_SCHED_TOKEN_CNTR_REG(q)	(0x8080 + ((q) << 2))
#define MVPP2_TXQ_TOKEN_CNTR_MAX		0xffffffff
/* Transmit Queue Arbiter Configuration (TQxAC) */
#define MVPP2_TXQ_SCHED_WRR_REG(q)		(0x80A0 + ((q) << 2))
#define MVPP2_TXQ_WRR_WEIGHT_OFFS		0
#define MVPP2_TXQ_WRR_WEIGHT_MAX		0xFF
#define MVPP2_TXQ_WRR_WEIGHT_ALL_MASK		(MVPP2_TXQ_WRR_WEIGHT_MAX << \
					MVPP2_TXQ_WRR_WEIGHT_OFFS)
#define MVPP2_TXQ_WRR_WEIGHT_MASK(weigth)	((weigth) << \
					MVPP2_TXQ_WRR_WEIGHT_OFFS)
#define MVPP2_TXQ_WRR_BYTE_COUNT_OFFS		8
#define MVPP2_TXQ_WRR_BYTE_COUNT_MASK		(0x3FFFF << \
					MVPP2_TXQ_WRR_BYTE_COUNT_OFFS)

/* TX general registers */
#define MVPP2_TX_SNOOP_REG			0x8800
#define MVPP2_TX_SNOOP_EN_MASK			BIT(0)
#define MVPP2_TX_SNOOP_EN_MASK			BIT(0)
#define MVPP22_TX_SNOOP_HWF_EN_MASK		BIT(1)

#define MVPP21_TX_FIFO_THRESH_REG		0x8804
#define MVPP21_TX_FIFO_THRESH_MASK		0x7ff
#define MVPP22_TX_FIFO_THRESH_REG(eth_tx_port)	(0x8840 + ((eth_tx_port) << 2))
#define MVPP22_TX_FIFO_THRESH_MASK		0x3fff

#define MVPP22_TX_FIFO_SIZE_REG(eth_tx_port)	(0x8860 + ((eth_tx_port) << 2))
#define MVPP22_TX_FIFO_SIZE_MASK		0xf

#define MVPP2_TX_PORT_FLUSH_REG			0x8810
#define MVPP2_TX_PORT_FLUSH_MASK(port)		(1 << (port))

						/* Same for PPv21/PPv22 */
#define MVPP2_TX_BAD_FCS_CNTR_REG(eth_tx_port)	(0x8940 + ((eth_tx_port) << 2))
						/* Same for PPv21/PPv22 */
#define MVPP2_TX_DROP_CNTR_REG(eth_tx_port)	(0x8980 + ((eth_tx_port) << 2))

#define MVPP2_TX_ETH_DSEC_THRESH_REG(eth_tx_port)(0x8a40 + \
					((eth_tx_port) << 2))
#define MVPP2_TX_ETH_DSEC_THRESH_MASK		0x7f0

#define MVPP22_TX_EGR_PIPE_DELAY_REG(eth_tx_port)(0x8a80 + \
					((eth_tx_port) << 2))
#define MVPP22_TX_EGR_PIPE_DELAY_MASK		0x3fff
#define MVPP22_TX_PTP_DISPATCH_ENABLE_MASK	BIT(30)

#define MVPP22_TX_PORT_SHORT_HDR_REG		0x8ac0
#define MVPP22_TX_PORT_SHORT_HDR_MASK		0x7f

/* LMS registers */
#define MVPP2_SRC_ADDR_MIDDLE			0x24
#define MVPP2_SRC_ADDR_HIGH			0x28
#define MVPP2_PHY_AN_CFG0_REG			0x34
#define MVPP2_PHY_AN_STOP_SMI0_MASK		BIT(7)
#define MVPP2_MIB_COUNTERS_BASE(port)		(0x1000 + ((port) >> 1) * \
						0x400 + (port) * 0x400)
#define MVPP2_MIB_LATE_COLLISION		0x7c
#define MVPP2_ISR_SUM_MASK_REG			0x220c
#define MVPP2_MNG_EXTENDED_GLOBAL_CTRL_REG	0x305c
#define MVPP2_EXT_GLOBAL_CTRL_DEFAULT		0x27

/* Per-port registers */
#define MVPP2_GMAC_CTRL_0_REG			0x0
#define MVPP2_GMAC_PORT_EN_MASK			BIT(0)
#define MVPP2_GMAC_MAX_RX_SIZE_OFFS		2
#define MVPP2_GMAC_MAX_RX_SIZE_MASK		0x7ffc
#define MVPP2_GMAC_MIB_CNTR_EN_MASK		BIT(15)
#define MVPP2_GMAC_CTRL_1_REG			0x4
#define MVPP2_GMAC_PERIODIC_XON_EN_MASK		BIT(1)
#define MVPP2_GMAC_GMII_LB_EN_MASK		BIT(5)
#define MVPP2_GMAC_PCS_LB_EN_BIT		6
#define MVPP2_GMAC_PCS_LB_EN_MASK		BIT(6)
#define MVPP2_GMAC_SA_LOW_OFFS			7
#define MVPP2_GMAC_CTRL_2_REG			0x8
#define MVPP2_GMAC_INBAND_AN_MASK		BIT(0)
#define MVPP2_GMAC_PCS_ENABLE_MASK		BIT(3)
#define MVPP2_GMAC_PORT_RGMII_MASK		BIT(4)
#define MVPP2_GMAC_PORT_RESET_MASK		BIT(6)
#define MVPP2_GMAC_AUTONEG_CONFIG		0xc
#define MVPP2_GMAC_FORCE_LINK_DOWN		BIT(0)
#define MVPP2_GMAC_FORCE_LINK_PASS		BIT(1)
#define MVPP2_GMAC_CONFIG_MII_SPEED		BIT(5)
#define MVPP2_GMAC_CONFIG_GMII_SPEED		BIT(6)
#define MVPP2_GMAC_AN_SPEED_EN			BIT(7)
#define MVPP2_GMAC_FC_ADV_EN			BIT(9)
#define MVPP2_GMAC_CONFIG_FULL_DUPLEX		BIT(12)
#define MVPP2_GMAC_AN_DUPLEX_EN			BIT(13)
#define MVPP2_GMAC_PORT_FIFO_CFG_1_REG		0x1c
#define MVPP2_GMAC_TX_FIFO_MIN_TH_OFFS	6
#define MVPP2_GMAC_TX_FIFO_MIN_TH_ALL_MASK	0x1fc0
#define MVPP2_GMAC_TX_FIFO_MIN_TH_MASK(v)	(((v) << 6) & \
					MVPP2_GMAC_TX_FIFO_MIN_TH_ALL_MASK)

#define MVPP2_CAUSE_TXQ_SENT_DESC_ALL_MASK	0xff

/* Lbtd 802.3 type */
#define MVPP2_IP_LBDT_TYPE		0xfffa

#define MVPP2_TX_CSUM_MAX_SIZE		9800

/* Timeout constants */
#define MVPP2_TX_DISABLE_TIMEOUT_MSEC	1000
#define MVPP2_TX_PENDING_TIMEOUT_USEC	1000

#define MVPP2_TX_MTU_MAX		0x7ffff

/* Maximum number of T-CONTs of PON port */
#define MVPP2_MAX_TCONT			16

/* Maximum number of supported ports */
#define MVPP2_MAX_PORTS			4

/* Maximum number of supported cells */
#define MVPP2_MAX_CELLS			4

/* Maximum number of TXQs used by single port */
#define MVPP2_MAX_TXQ			8

/* Maximum number of RXQs used by single port */
#define MVPP2_MAX_RXQ			8

/* Dfault number of RXQs in use */
#define MVPP2_DEFAULT_RXQ		4

/* Total number of RXQs available to all ports */
#define MVPP2_RXQ_TOTAL_NUM		(MVPP2_MAX_PORTS * MVPP2_MAX_RXQ)

#define MVPP2_TXQ_TOTAL_NUM		(128/*pon*/ + \
					MVPP2_MAX_PORTS * MVPP2_MAX_TXQ/*eth*/)

/* Max number of Rx descriptors */
#define MVPP2_MAX_RXD			1024

/* Max number of Tx descriptors */
#define MVPP2_MAX_TXD			1024

/* Amount of Tx descriptors that can be reserved at once by CPU */
#define MVPP2_CPU_DESC_CHUNK		64

/* Max number of Tx descriptors in each aggregated queue */
#define MVPP2_AGGR_TXQ_SIZE		256

/* Descriptor aligned size */
#define MVPP2_DESC_ALIGNED_SIZE		32
#define MVPP2_DESC_Q_ALIGN		512

#define MVPP2_DESCQ_MEM_SIZE(descs)	(descs * MVPP2_DESC_ALIGNED_SIZE + \
					MVPP2_DESC_Q_ALIGN)
#define MVPP2_DESCQ_MEM_ALIGN(mem)	(ALIGN(mem, MVPP2_DESC_Q_ALIGN))

/* Descriptor alignment mask */
#define MVPP2_TX_DESC_ALIGN		(MVPP2_DESC_ALIGNED_SIZE - 1)

/* RX FIFO constants */
#define MVPP2_RX_FIFO_PORT_DATA_SIZE	0x2000
#define MVPP2_RX_FIFO_PORT_ATTR_SIZE	0x80
#define MVPP2_RX_FIFO_PORT_MIN_PKT	0x80

/* RX buffer constants */
#define MVPP2_SKB_SHINFO_SIZE (0)

#define MVPP2_MTU_PKT_SIZE(mtu) \
	(ALIGN((mtu) + MV_MH_SIZE + MV_VLAN_TAG_LEN + \
	      MV_ETH_HLEN + MV_ETH_FCS_LEN, L1_CACHE_LINE_BYTES))

#define MVPP2_RX_MTU_SIZE(pkt_size) \
	(pkt_size - MV_MH_SIZE - MVPP2_VLAN_TAG_LEN - \
	 ETH_HLEN - ETH_FCS_LEN)

/* IPv6 max L3 address size */
#define MVPP2_MAX_L3_ADDR_SIZE		16

/* Port flags */
#define MVPP2_F_LOOPBACK		BIT(0)
#define MVPP2_F_IFCAP_NETMAP	BIT(1)

/* Marvell tag types */
enum mv_pp2x_tag_type {
	MVPP2_TAG_TYPE_NONE = 0,
	MVPP2_TAG_TYPE_MH   = 1,
	MVPP2_TAG_TYPE_DSA  = 2,
	MVPP2_TAG_TYPE_EDSA = 3,
	MVPP2_TAG_TYPE_VLAN = 4,
	MVPP2_TAG_TYPE_LAST = 5
};

/* Parser constants */
#define MVPP2_PRS_TCAM_SRAM_SIZE	256
#define MVPP2_PRS_TCAM_WORDS		6
#define MVPP2_PRS_SRAM_WORDS		4
#define MVPP2_PRS_FLOW_ID_SIZE		64
#define MVPP2_PRS_FLOW_ID_MASK		0x3f
#define MVPP2_PRS_TCAM_ENTRY_VALID	0
#define MVPP2_PRS_TCAM_ENTRY_INVALID	1
#define MVPP2_PRS_TCAM_DSA_TAGGED_BIT	BIT(5)
#define MVPP2_PRS_TCAM_DSA_TO_CPU_MODE		0
#define MVPP2_PRS_TCAM_DSA_FROM_CPU_MODE	BIT(6)
#define MVPP2_PRS_TCAM_DSA_TO_SNIFFER_MODE	BIT(7)
#define MVPP2_PRS_TCAM_DSA_FORWARD_MODE		(BIT(6) | BIT(7))
#define MVPP2_PRS_TCAM_DSA_MODE_MASK		0xc0
#define MVPP2_PRS_IPV4_HEAD		0x40
#define MVPP2_PRS_IPV4_HEAD_MASK	0xf0
#define MVPP2_PRS_IPV4_MC		0xe0
#define MVPP2_PRS_IPV4_MC_MASK		0xf0
#define MVPP2_PRS_IPV4_BC_MASK		0xff
#define MVPP2_PRS_IPV4_IHL		0x5
#define MVPP2_PRS_IPV4_IHL_MASK		0xf
#define MVPP2_PRS_IPV6_MC		0xff
#define MVPP2_PRS_IPV6_MC_MASK		0xff
#define MVPP2_PRS_IPV6_HOP_MASK		0xff
#define MVPP2_PRS_TCAM_PROTO_MASK	0xff
#define MVPP2_PRS_TCAM_PROTO_MASK_L	0x3f
#define MVPP2_PRS_DBL_VLANS_MAX		100

/* There is TCAM range reserved for MAC entries, range size is 113
 * 1 BC MAC entry for all ports
 * 4 M2M entries, 1 entry per port, and 4 ports in all
 * 36 UC/MC MAC filter entries per port
 * It is assumed that there are 3 ports for filter, not including loopback port
 */
#define MVPP2_PRS_MAC_UC_MC_FILT_MAX	25
#define MVPP2_PRS_MAC_RANGE_SIZE	80

#define MVPP2_PRS_VLAN_FILT_MAX		11
#define MVPP2_PRS_VLAN_FILT_RANGE_SIZE	33

#define MVPP2_PRS_VLAN_FILT_MAX_ENTRY   (MVPP2_PRS_VLAN_FILT_MAX - 2)

/* Tcam structure:
 * - lookup ID - 4 bits
 * - port ID - 1 byte
 * - additional information - 1 byte
 * - header data - 8 bytes
 * The fields are represented by MVPP2_PRS_TCAM_DATA_REG(5)->(0).
 */
#define MVPP2_PRS_AI_BITS			8
#define MVPP2_PRS_PORT_MASK			0xff
#define MVPP2_PRS_LU_MASK			0xf
#define MVPP2_PRS_TCAM_AI_BYTE			16
#define MVPP2_PRS_TCAM_PORT_BYTE		17
#define MVPP2_PRS_TCAM_LU_BYTE			20
#define MVPP2_PRS_TCAM_EN_OFFS(offs)		((offs) + 2)
#define MVPP2_PRS_TCAM_INV_WORD			5
#define MVPP2_PRS_TCAM_INV_MASK			BIT(31)
#define MVPP2_PRS_TCAM_INV_OFFS			31

/* Tcam entries ID */
#define MVPP2_PE_DROP_ALL		0
#define MVPP2_PE_FIRST_FREE_TID		1
#define MVPP2_PE_MAC_RANGE_END		(MVPP2_PE_VID_FILT_RANGE_START - 1)
#define MVPP2_PE_MAC_RANGE_START	(MVPP2_PE_MAC_RANGE_END - MVPP2_PRS_MAC_RANGE_SIZE + 1)
#define MVPP2_PE_VID_FILT_RANGE_END	(MVPP2_PRS_TCAM_SRAM_SIZE - 31)
#define MVPP2_PE_VID_FILT_RANGE_START	(MVPP2_PE_VID_FILT_RANGE_END - MVPP2_PRS_VLAN_FILT_RANGE_SIZE + 1)
#define MVPP2_PE_LAST_FREE_TID		(MVPP2_PE_MAC_RANGE_START - 1)
#define MVPP2_PE_IP6_EXT_PROTO_UN	(MVPP2_PRS_TCAM_SRAM_SIZE - 30)
#define MVPP2_PE_IP6_ADDR_UN		(MVPP2_PRS_TCAM_SRAM_SIZE - 29)
#define MVPP2_PE_IP4_ADDR_UN		(MVPP2_PRS_TCAM_SRAM_SIZE - 28)
#define MVPP2_PE_LAST_DEFAULT_FLOW	(MVPP2_PRS_TCAM_SRAM_SIZE - 27)
#define MVPP2_PE_FIRST_DEFAULT_FLOW	(MVPP2_PRS_TCAM_SRAM_SIZE - 22)
#define MVPP2_PE_EDSA_TAGGED		(MVPP2_PRS_TCAM_SRAM_SIZE - 21)
#define MVPP2_PE_EDSA_UNTAGGED		(MVPP2_PRS_TCAM_SRAM_SIZE - 20)
#define MVPP2_PE_DSA_TAGGED		(MVPP2_PRS_TCAM_SRAM_SIZE - 19)
#define MVPP2_PE_DSA_UNTAGGED		(MVPP2_PRS_TCAM_SRAM_SIZE - 18)
#define MVPP2_PE_ETYPE_EDSA_TAGGED	(MVPP2_PRS_TCAM_SRAM_SIZE - 17)
#define MVPP2_PE_ETYPE_EDSA_UNTAGGED	(MVPP2_PRS_TCAM_SRAM_SIZE - 16)
#define MVPP2_PE_ETYPE_DSA_TAGGED	(MVPP2_PRS_TCAM_SRAM_SIZE - 15)
#define MVPP2_PE_ETYPE_DSA_UNTAGGED	(MVPP2_PRS_TCAM_SRAM_SIZE - 14)
#define MVPP2_PE_MH_DEFAULT		(MVPP2_PRS_TCAM_SRAM_SIZE - 13)
#define MVPP2_PE_DSA_DEFAULT		(MVPP2_PRS_TCAM_SRAM_SIZE - 12)
#define MVPP2_PE_IP6_PROTO_UN		(MVPP2_PRS_TCAM_SRAM_SIZE - 11)
#define MVPP2_PE_IP4_PROTO_UN		(MVPP2_PRS_TCAM_SRAM_SIZE - 10)
#define MVPP2_PE_ETH_TYPE_UN		(MVPP2_PRS_TCAM_SRAM_SIZE - 9)
#define MVPP2_PE_VID_FLTR_DEFAULT	(MVPP2_PRS_TCAM_SRAM_SIZE - 8)
#define MVPP2_PE_VID_EDSA_FLTR_DEFAULT	(MVPP2_PRS_TCAM_SRAM_SIZE - 7)
#define MVPP2_PE_VLAN_DBL		(MVPP2_PRS_TCAM_SRAM_SIZE - 6)
#define MVPP2_PE_VLAN_NONE		(MVPP2_PRS_TCAM_SRAM_SIZE - 5)
#define MVPP2_PE_FC_DROP		(MVPP2_PRS_TCAM_SRAM_SIZE - 4)
#define MVPP2_PE_MAC_MC_ALL		(MVPP2_PRS_TCAM_SRAM_SIZE - 3)
#define MVPP2_PE_MAC_MC_PROMISCUOUS	(MVPP2_PRS_TCAM_SRAM_SIZE - 2)
#define MVPP2_PE_MAC_UC_PROMISCUOUS	(MVPP2_PRS_TCAM_SRAM_SIZE - 1)
#define MVPP2_PE_TID_SIZE		(MVPP2_PE_LAST_FREE_TID - MVPP2_PE_FIRST_FREE_TID)

#define MVPP2_PRS_VID_PORT_FIRST(port)	(MVPP2_PE_VID_FILT_RANGE_START + \
					 ((port) * MVPP2_PRS_VLAN_FILT_MAX))
#define MVPP2_PRS_VID_PORT_LAST(port)	(MVPP2_PRS_VID_PORT_FIRST(port) + MVPP2_PRS_VLAN_FILT_MAX_ENTRY)

/* Sram structure
 * The fields are represented by MVPP2_PRS_TCAM_DATA_REG(3)->(0).
 */
#define MVPP2_PRS_SRAM_RI_OFFS			0
#define MVPP2_PRS_SRAM_RI_WORD			0
#define MVPP2_PRS_SRAM_RI_BITS			32
#define MVPP2_PRS_SRAM_RI_CTRL_OFFS		32
#define MVPP2_PRS_SRAM_RI_CTRL_WORD		1
#define MVPP2_PRS_SRAM_RI_CTRL_BITS		32
#define MVPP2_PRS_SRAM_SHIFT_OFFS		64
#define MVPP2_PRS_SRAM_SHIFT_BITS		8
#define MVPP2_PRS_SRAM_SHIFT_SIGN_BIT		72
#define MVPP2_PRS_SRAM_UDF_OFFS			73
#define MVPP2_PRS_SRAM_UDF_BITS			8
#define MVPP2_PRS_SRAM_UDF_MASK			0xff
#define MVPP2_PRS_SRAM_UDF_SIGN_BIT		81
#define MVPP2_PRS_SRAM_UDF_TYPE_OFFS		82
#define MVPP2_PRS_SRAM_UDF_TYPE_MASK		0x7
#define MVPP2_PRS_SRAM_UDF_TYPE_L3		1
#define MVPP2_PRS_SRAM_UDF_TYPE_3		3
#define MVPP2_PRS_SRAM_UDF_TYPE_L4		4
#define MVPP2_PRS_SRAM_UDF_TYPE_5		5
#define MVPP2_PRS_SRAM_UDF_TYPE_6		6
#define MVPP2_PRS_SRAM_OP_SEL_SHIFT_OFFS	85
#define MVPP2_PRS_SRAM_OP_SEL_SHIFT_BITS	2
#define MVPP2_PRS_SRAM_OP_SEL_BITS		5
#define MVPP2_PRS_SRAM_OP_SEL_SHIFT_MASK	0x3
#define MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD		1
#define MVPP2_PRS_SRAM_OP_SEL_SHIFT_IP4_ADD	2
#define MVPP2_PRS_SRAM_OP_SEL_SHIFT_IP6_ADD	3
#define MVPP2_PRS_SRAM_OP_SEL_UDF_OFFS		87
#define MVPP2_PRS_SRAM_OP_SEL_UDF_BITS		2
#define MVPP2_PRS_SRAM_OP_SEL_UDF_MASK		0x3
#define MVPP2_PRS_SRAM_OP_SEL_UDF_ADD		0
#define MVPP2_PRS_SRAM_OP_SEL_UDF_IP4_ADD	2
#define MVPP2_PRS_SRAM_OP_SEL_UDF_IP6_ADD	3
#define MVPP2_PRS_SRAM_OP_SEL_BASE_OFFS		89
#define MVPP2_PRS_SRAM_AI_OFFS			90
#define MVPP2_PRS_SRAM_AI_CTRL_OFFS		98
#define MVPP2_PRS_SRAM_AI_CTRL_BITS		8
#define MVPP2_PRS_SRAM_AI_MASK			0xff
#define MVPP2_PRS_SRAM_NEXT_LU_OFFS		106
#define MVPP2_PRS_SRAM_NEXT_LU_MASK		0xf
#define MVPP2_PRS_SRAM_LU_DONE_BIT		110
#define MVPP2_PRS_SRAM_LU_GEN_BIT		111

/* Sram result info bits assignment */
#define MVPP2_PRS_RI_MAC_ME_MASK		0x1
#define MVPP2_PRS_RI_DSA_MASK			0x2
#define MVPP2_PRS_RI_VLAN_OFFS			2
#define MVPP2_PRS_RI_VLAN_MASK			0xc
#define MVPP2_PRS_RI_VLAN_NONE			0x0
#define MVPP2_PRS_RI_VLAN_SINGLE		BIT(2)
#define MVPP2_PRS_RI_VLAN_DOUBLE		BIT(3)
#define MVPP2_PRS_RI_VLAN_TRIPLE		(BIT(2) | BIT(3))
#define MVPP2_PRS_RI_CPU_CODE_MASK		0x70
#define MVPP2_PRS_RI_CPU_CODE_RX_SPEC		BIT(4)
#define MVPP2_PRS_RI_L2_CAST_OFFS		9
#define MVPP2_PRS_RI_L2_CAST_MASK		0x600
#define MVPP2_PRS_RI_L2_UCAST			0x0
#define MVPP2_PRS_RI_L2_MCAST			BIT(9)
#define MVPP2_PRS_RI_L2_BCAST			BIT(10)
#define MVPP2_PRS_RI_PPPOE_MASK			0x800
#define MVPP2_PRS_RI_L3_PROTO_MASK		0x7000
#define MVPP2_PRS_RI_L3_UN			0x0
#define MVPP2_PRS_RI_L3_IP4			BIT(12)
#define MVPP2_PRS_RI_L3_IP4_OPT			BIT(13)
#define MVPP2_PRS_RI_L3_IP4_OTHER		(BIT(12) | BIT(13))
#define MVPP2_PRS_RI_L3_IP6			BIT(14)
#define MVPP2_PRS_RI_L3_IP6_EXT			(BIT(12) | BIT(14))
#define MVPP2_PRS_RI_L3_ARP			(BIT(13) | BIT(14))
#define MVPP2_PRS_RI_L3_ADDR_MASK		0x18000
#define MVPP2_PRS_RI_L3_UCAST			0x0
#define MVPP2_PRS_RI_L3_MCAST			BIT(15)
#define MVPP2_PRS_RI_L3_BCAST			(BIT(15) | BIT(16))
#define MVPP2_PRS_RI_IP_FRAG_MASK		0x20000
#define MVPP2_PRS_RI_IP_FRAG_TRUE		BIT(17)
#define MVPP2_PRS_RI_IP_FRAG_FALSE		0x0
#define MVPP2_PRS_RI_UDF3_MASK			0x300000
#define MVPP2_PRS_RI_UDF3_RX_SPECIAL		BIT(21)
#define MVPP2_PRS_RI_L4_PROTO_MASK		0x1c00000
#define MVPP2_PRS_RI_L4_TCP			BIT(22)
#define MVPP2_PRS_RI_L4_UDP			BIT(23)
#define MVPP2_PRS_RI_L4_OTHER			(BIT(22) | BIT(23))
#define MVPP2_PRS_RI_UDF5_MASK			0x6000000
#define MVPP2_PRS_RI_UDF6_MASK			0x18000000
#define MVPP2_PRS_RI_UDF7_MASK			0x60000000
#define MVPP2_PRS_RI_UDF7_NIC			BIT(29)
#define MVPP2_PRS_RI_UDF7_LOG_PORT		BIT(30)
#define MVPP2_PRS_RI_UDF7_IP6_LITE		(BIT(29) | BIT(30))
#define MVPP2_PRS_RI_UDF7_CLEAR			0x0
#define MVPP2_PRS_RI_DROP_MASK			0x80000000

/* Sram additional info bits assignment */
#define MVPP2_PRS_L2_UDF_AI_BIT			BIT(0)
#define MVPP2_PRS_IPV4_DIP_AI_BIT		BIT(0)
#define MVPP2_PRS_IPV6_NO_EXT_AI_BIT		BIT(0)
#define MVPP2_PRS_IPV6_EXT_AI_BIT		BIT(1)
#define MVPP2_PRS_IPV6_EXT_AH_AI_BIT		BIT(2)
#define MVPP2_PRS_IPV6_EXT_AH_LEN_AI_BIT	BIT(3)
#define MVPP2_PRS_IPV6_EXT_AH_L4_AI_BIT		BIT(4)
#define MVPP2_PRS_SINGLE_VLAN_AI		0
#define MVPP2_PRS_DBL_VLAN_AI_BIT		BIT(7)

#define MVPP2_PRS_SRAM_SHIFT_MASK		((1 << \
					MVPP2_PRS_SRAM_SHIFT_BITS) - 1)

/* DSA/EDSA type */
#define MVPP2_PRS_TAGGED		true
#define MVPP2_PRS_UNTAGGED		false
#define MVPP2_PRS_EDSA			true
#define MVPP2_PRS_DSA			false

/* lkpid table structure	*/
#define MVPP2_FLOWID_RXQ		0
#define MVPP2_FLOWID_RXQ_BITS		8
#define MVPP2_FLOWID_RXQ_MASK		(((1 << \
			MVPP2_FLOWID_RXQ_BITS) - 1) << MVPP2_FLOWID_RXQ)

#define MVPP2_FLOWID_MODE		8
#define MVPP2_FLOWID_MODE_BITS		8
#define MVPP2_FLOWID_MODE_MASK		(((1 << \
			MVPP2_FLOWID_MODE_BITS) - 1) << MVPP2_FLOWID_MODE)
#define MVPP2_FLOWID_MODE_MAX		((1 << MVPP2_FLOWID_MODE_BITS) - 1)

#define MVPP2_FLOWID_FLOW		16
#define MVPP2_FLOWID_FLOW_BITS		9
#define MVPP2_FLOWID_FLOW_MASK		(((1 << \
			MVPP2_FLOWID_FLOW_BITS) - 1) << MVPP2_FLOWID_FLOW)

#define MVPP2_FLOWID_EN			25 /*one bit */
#define MVPP2_FLOWID_EN_MASK		BIT(MVPP2_FLOWID_EN)

/* flow table structure */
#define MVPP2_FLOW_TBL_SIZE		512
/*-------------------------  DWORD 0  --------------------------------- */
#define MVPP2_FLOW_LAST			0
#define MVPP2_FLOW_LAST_MASK		1 /*one bit*/

#define MVPP2_FLOW_ENGINE		1
#define MVPP2_FLOW_ENGINE_BITS		3
#define MVPP2_FLOW_ENGINE_MASK		(((1 << \
			MVPP2_FLOW_ENGINE_BITS) - 1) << MVPP2_FLOW_ENGINE)
#define MVPP2_FLOW_ENGINE_MAX		7 /* valid value 1 - 7 */

#define MVPP2_FLOW_PORT_ID		4
#define MVPP2_FLOW_PORT_ID_BITS		8
#define MVPP2_FLOW_PORT_ID_MASK		(((1 << \
			MVPP2_FLOW_PORT_ID_BITS) - 1) << MVPP2_FLOW_PORT_ID)
#define MVPP2_FLOW_PORT_ID_MAX		((1 << MVPP2_FLOW_PORT_ID_BITS) - 1)

#define MVPP2_FLOW_PORT_TYPE		12
#define MVPP2_FLOW_PORT_TYPE_BITS	2
#define MVPP2_FLOW_PORT_TYPE_MASK	(((1 << \
		MVPP2_FLOW_PORT_TYPE_BITS) - 1) << MVPP2_FLOW_PORT_TYPE)
#define MVPP2_FLOW_PORT_TYPE_MAX	2 /* valid value 0 - 2 */

#define MVPP2_FLOW_PPPOE		14
#define MVPP2_FLOW_PPPOE_BITS		2
#define MVPP2_FLOW_PPPOE_MASK		(((1 << \
			MVPP2_FLOW_PPPOE_BITS) - 1) << MVPP2_FLOW_PPPOE)
#define MVPP2_FLOW_PPPOE_MAX		2 /* valid value 0 - 2 */

#define MVPP2_FLOW_VLAN			16
#define MVPP2_FLOW_VLAN_BITS		3
#define MVPP2_FLOW_VLAN_MASK		(((1 << \
			MVPP2_FLOW_VLAN_BITS) - 1) << MVPP2_FLOW_VLAN)
#define MVPP2_FLOW_VLAN_MAX		((1 << MVPP2_FLOW_VLAN_BITS) - 1)

#define MVPP2_FLOW_MACME		19
#define MVPP2_FLOW_MACME_BITS		2
#define MVPP2_FLOW_MACME_MASK		(((1 << \
			MVPP2_FLOW_MACME_BITS) - 1) << MVPP2_FLOW_MACME)
#define MVPP2_FLOW_MACME_MAX		2 /* valid value 0 - 2 */

#define MVPP2_FLOW_UDF7			21
#define MVPP2_FLOW_UDF7_BITS		2
#define MVPP2_FLOW_UDF7_MASK		(((1 << \
			MVPP2_FLOW_UDF7_BITS) - 1) << MVPP2_FLOW_UDF7)
#define MVPP2_FLOW_UDF7_MAX		((1 << MVPP2_FLOW_UDF7_BITS) - 1)

#define MVPP2_FLOW_PORT_ID_SEL		23
#define MVPP2_FLOW_PORT_ID_SEL_MASK	BIT(MVPP2_FLOW_PORT_ID_SEL)

/*-----------------------  DWORD 1  ------------------------------------ */

#define MVPP2_FLOW_FIELDS_NUM		0
#define MVPP2_FLOW_FIELDS_NUM_BITS	3
#define MVPP2_FLOW_FIELDS_NUM_MASK	(((1 << \
		MVPP2_FLOW_FIELDS_NUM_BITS) - 1) << MVPP2_FLOW_FIELDS_NUM)
#define MVPP2_FLOW_FIELDS_NUM_MAX	4 /*valid vaue 0 - 4 */

#define MVPP2_FLOW_LKP_TYPE		3
#define MVPP2_FLOW_LKP_TYPE_BITS	6
#define MVPP2_FLOW_LKP_TYPE_MASK	(((1 << \
		MVPP2_FLOW_LKP_TYPE_BITS) - 1) << MVPP2_FLOW_LKP_TYPE)
#define MVPP2_FLOW_LKP_TYPE_MAX		((1 << MVPP2_FLOW_LKP_TYPE_BITS) - 1)

#define MVPP2_FLOW_FIELD_PRIO		9
#define MVPP2_FLOW_FIELD_PRIO_BITS	6
#define MVPP2_FLOW_FIELD_PRIO_MASK	(((1 << \
		MVPP2_FLOW_FIELD_PRIO_BITS) - 1) << MVPP2_FLOW_FIELD_PRIO)
#define MVPP2_FLOW_FIELD_PRIO_MAX	((1 << MVPP2_FLOW_FIELD_PRIO_BITS) - 1)

#define MVPP2_FLOW_SEQ_CTRL		15
#define MVPP2_FLOW_SEQ_CTRL_BITS	3
#define MVPP2_FLOW_SEQ_CTRL_MASK	(((1 << \
		MVPP2_FLOW_SEQ_CTRL_BITS) - 1) << MVPP2_FLOW_SEQ_CTRL)
#define MVPP2_FLOW_SEQ_CTRL_MAX		4

/*-------------------------  DWORD 2  ---------------------------------- */
#define MVPP2_FLOW_FIELD0_ID		0
#define MVPP2_FLOW_FIELD1_ID		6
#define MVPP2_FLOW_FIELD2_ID		12
#define MVPP2_FLOW_FIELD3_ID		18

#define MVPP2_FLOW_FIELD_ID_BITS	6
#define MVPP2_FLOW_FIELD_ID(num)	(MVPP2_FLOW_FIELD0_ID + \
					(MVPP2_FLOW_FIELD_ID_BITS * (num)))
#define MVPP2_FLOW_FIELD_MASK(num)	(((1 << \
	MVPP2_FLOW_FIELD_ID_BITS) - 1) << (MVPP2_FLOW_FIELD_ID_BITS * (num)))
#define MVPP2_FLOW_FIELD_MAX		((1 << MVPP2_FLOW_FIELD_ID_BITS) - 1)

/* lookup id attribute define */
#define MVPP2_PRS_FL_ATTR_VLAN_BIT	BIT(0)
#define MVPP2_PRS_FL_ATTR_IP4_BIT	BIT(1)
#define MVPP2_PRS_FL_ATTR_IP6_BIT	BIT(2)
#define MVPP2_PRS_FL_ATTR_ARP_BIT	BIT(3)
#define MVPP2_PRS_FL_ATTR_FRAG_BIT	BIT(4)
#define MVPP2_PRS_FL_ATTR_TCP_BIT	BIT(5)
#define MVPP2_PRS_FL_ATTR_UDP_BIT	BIT(6)

/* PP22 RSS Registers */
#define MVPP22_RSS_IDX_REG			0x1500
#define MVPP22_RSS_IDX_ENTRY_NUM_OFF		0
#define MVPP22_RSS_IDX_ENTRY_NUM_MASK		0x1F
#define MVPP22_RSS_IDX_TBL_NUM_OFF		8
#define MVPP22_RSS_IDX_TBL_NUM_MASK		0x700
#define MVPP22_RSS_IDX_RXQ_NUM_OFF		16
#define MVPP22_RSS_IDX_RXQ_NUM_MASK		0xFF0000
#define MVPP22_RSS_RXQ2RSS_TBL_REG		0x1504
#define MVPP22_RSS_RXQ2RSS_TBL_POINT_OFF	0
#define MVPP22_RSS_RXQ2RSS_TBL_POINT_MASK	0x7
#define MVPP22_RSS_TBL_ENTRY_REG		0x1508
#define MVPP22_RSS_TBL_ENTRY_OFF		0
#define MVPP22_RSS_TBL_ENTRY_MASK		0xFF
#define MVPP22_RSS_WIDTH_REG			0x150c
#define MVPP22_RSS_WIDTH_OFF			0
#define MVPP22_RSS_WIDTH_MASK			0xF
#define MVPP22_RSS_HASH_SEL_REG			0x1510
#define MVPP22_RSS_HASH_SEL_OFF			0
#define MVPP22_RSS_HASH_SEL_MASK		0x1
/* RSS consant */
#define MVPP22_RSS_TBL_NUM			8
#define MVPP22_RSS_TBL_LINE_NUM			32
#define MVPP22_RSS_WIDTH_MAX			8

/* Lookup ID */
enum mv_pp2x_prs_lookup {
	MVPP2_PRS_LU_MH,
	MVPP2_PRS_LU_MAC,
	MVPP2_PRS_LU_DSA,
	MVPP2_PRS_LU_VLAN,
	MVPP2_PRS_LU_VID,
	MVPP2_PRS_LU_L2,
	MVPP2_PRS_LU_PPPOE,
	MVPP2_PRS_LU_IP4,
	MVPP2_PRS_LU_IP6,
	MVPP2_PRS_LU_FLOWS,
	MVPP2_PRS_LU_LAST,
};

/* L3 cast enum */
enum mv_pp2x_prs_l3_cast {
	MVPP2_PRS_L3_UNI_CAST,
	MVPP2_PRS_L3_MULTI_CAST,
	MVPP2_PRS_L3_BROAD_CAST
};

/* Packet flow ID */
enum mv_pp2x_prs_flow {
	MVPP2_PRS_FL_START = 8,
	MVPP2_PRS_FL_IP4_TCP_NF_UNTAG = MVPP2_PRS_FL_START,
	MVPP2_PRS_FL_IP4_UDP_NF_UNTAG,
	MVPP2_PRS_FL_IP4_TCP_NF_TAG,
	MVPP2_PRS_FL_IP4_UDP_NF_TAG,
	MVPP2_PRS_FL_IP6_TCP_NF_UNTAG,
	MVPP2_PRS_FL_IP6_UDP_NF_UNTAG,
	MVPP2_PRS_FL_IP6_TCP_NF_TAG,
	MVPP2_PRS_FL_IP6_UDP_NF_TAG,
	MVPP2_PRS_FL_IP4_TCP_FRAG_UNTAG,
	MVPP2_PRS_FL_IP4_UDP_FRAG_UNTAG,
	MVPP2_PRS_FL_IP4_TCP_FRAG_TAG,
	MVPP2_PRS_FL_IP4_UDP_FRAG_TAG,
	MVPP2_PRS_FL_IP6_TCP_FRAG_UNTAG,
	MVPP2_PRS_FL_IP6_UDP_FRAG_UNTAG,
	MVPP2_PRS_FL_IP6_TCP_FRAG_TAG,
	MVPP2_PRS_FL_IP6_UDP_FRAG_TAG,
	MVPP2_PRS_FL_IP4_UNTAG, /* non-TCP, non-UDP, same for below */
	MVPP2_PRS_FL_IP4_TAG,
	MVPP2_PRS_FL_IP6_UNTAG,
	MVPP2_PRS_FL_IP6_TAG,
	MVPP2_PRS_FL_NON_IP_UNTAG,
	MVPP2_PRS_FL_NON_IP_TAG,
	MVPP2_PRS_FL_LAST,
	MVPP2_PRS_FL_TCAM_NUM = 52,	/* The parser TCAM lines needed to
					*generate flow ID
					*/
};

enum mv_pp2x_cls_engine_num {
	MVPP2_CLS_ENGINE_C2 = 1,
	MVPP2_CLS_ENGINE_C3A,
	MVPP2_CLS_ENGINE_C3B,
	MVPP2_CLS_ENGINE_C4,
	MVPP2_CLS_ENGINE_C3HA = 6,
	MVPP2_CLS_ENGINE_C3HB,
};

enum mv_pp2x_cls_lkp_type {
	MVPP2_CLS_LKP_HASH = 0,
	MVPP2_CLS_LKP_VLAN_PRI,
	MVPP2_CLS_LKP_DSCP_PRI,
	MVPP2_CLS_LKP_DEFAULT,
	MVPP2_CLS_LKP_MUSDK_LOG_HASH,
	MVPP2_CLS_LKP_MUSDK_VLAN_PRI,
	MVPP2_CLS_LKP_MUSDK_DSCP_PRI,
	MVPP2_CLS_LKP_MUSDK_LOG_PORT_DEF,
	MVPP2_CLS_LKP_MUSDK_CLS,
	MVPP2_CLS_LKP_MAX,
};

enum mv_pp2x_cls_fl_pri {
	MVPP2_CLS_FL_COS_PRI = 0,
	MVPP2_CLS_FL_RSS_PRI,
};

enum mv_pp2x_cls_filed_id {
	MVPP2_CLS_FIELD_IP4SA = 0x10,
	MVPP2_CLS_FIELD_IP4DA = 0x11,
	MVPP2_CLS_FIELD_IP6SA = 0x17,
	MVPP2_CLS_FIELD_IP6DA = 0x1A,
	MVPP2_CLS_FIELD_L4SIP = 0x1D,
	MVPP2_CLS_FIELD_L4DIP = 0x1E,
};

enum mv_pp2x_cos_type {
	MVPP2_COS_TYPE_DEF = 0,
	MVPP2_COS_TYPE_VLAN,
	MVPP2_COS_TYPE_DSCP,
};

struct mv_pp2x_prs_result_info {
	u32 ri;
	u32 ri_mask;
};

struct mv_pp2x_prs_flow_id {
	u32 flow_id;
	struct mv_pp2x_prs_result_info prs_result;
};

/* Classifier constants */
#define MVPP2_CLS_FLOWS_TBL_SIZE	512
#define MVPP2_CLS_FLOWS_TBL_DATA_WORDS	3
#define MVPP2_CLS_FLOWS_TBL_FIELDS_MAX	4

#define MVPP2_CLS_LKP_TBL_SIZE		64

/* BM cookie (32 bits) definition */
#define MVPP2_BM_COOKIE_POOL_OFFS	8
#define MVPP2_BM_COOKIE_CPU_OFFS	24

/* BM short pool packet size
 * These value assure that for SWF the total number
 * of bytes allocated for each buffer will be 512
 */

#define MVPP2_BM_JUMBO_FRAME_SIZE		10240

/* The mv_pp2x_tx_desc and mv_pp2x_rx_desc structures describe the
 * layout of the transmit and reception DMA descriptors, and their
 * layout is therefore defined by the hardware design
 */

#define MVPP2_TXD_L3_OFF_SHIFT		0
#define MVPP2_TXD_IP_HLEN_SHIFT		8
#define MVPP2_TXD_L4_CSUM_FRAG		BIT(13)
#define MVPP2_TXD_L4_CSUM_NOT		BIT(14)
#define MVPP2_TXD_IP_CSUM_DISABLE	BIT(15)
#define MVPP2_TXD_PADDING_DISABLE	BIT(23)
#define MVPP2_TXD_L4_UDP		BIT(24)
#define MVPP2_TXD_L3_IP6		BIT(26)
#define MVPP2_TXD_L_DESC		BIT(28)
#define MVPP2_TXD_F_DESC		BIT(29)

#define MVPP2_RXD_ERR_SUMMARY		BIT(15)
#define MVPP2_RXD_ERR_CODE_MASK		(BIT(13) | BIT(14))
#define MVPP2_RXD_ERR_CRC		0x0
#define MVPP2_RXD_ERR_OVERRUN		BIT(13)
#define MVPP2_RXD_ERR_RESOURCE		(BIT(13) | BIT(14))
#define MVPP2_RXD_BM_POOL_ID_OFFS	16
#define MVPP2_RXD_BM_POOL_ID_MASK	(BIT(16) | BIT(17) | BIT(18))
#define MVPP2_RXD_HWF_SYNC		BIT(21)
#define MVPP2_RXD_L4_CSUM_OK		BIT(22)
#define MVPP2_RXD_IP4_HEADER_ERR	BIT(24)
#define MVPP2_RXD_L4_TCP		BIT(25)
#define MVPP2_RXD_L4_UDP		BIT(26)
#define MVPP2_RXD_L3_IP4		BIT(28)
#define MVPP2_RXD_L3_IP6		BIT(30)
#define MVPP2_RXD_BUF_HDR		BIT(31)
/* Sub fields of "parserInfo" field */
#define MVPP2_RXD_LKP_ID_OFFS		0
#define MVPP2_RXD_LKP_ID_BITS		6
#define MVPP2_RXD_LKP_ID_MASK		(((1 << \
		MVPP2_RXD_LKP_ID_BITS) - 1) << MVPP2_RXD_LKP_ID_OFFS)
#define MVPP2_RXD_CPU_CODE_OFFS		6
#define MVPP2_RXD_CPU_CODE_BITS		3
#define MVPP2_RXD_CPU_CODE_MASK		(((1 << \
		MVPP2_RXD_CPU_CODE_BITS) - 1) << MVPP2_RXD_CPU_CODE_OFFS)
#define MVPP2_RXD_PPPOE_BIT		9
#define MVPP2_RXD_PPPOE_MASK		BIT(MVPP2_RXD_PPPOE_BIT)
#define MVPP2_RXD_L3_CAST_OFFS		10
#define MVPP2_RXD_L3_CAST_BITS		2
#define MVPP2_RXD_L3_CAST_MASK		(((1 << \
		MVPP2_RXD_L3_CAST_BITS) - 1) << MVPP2_RXD_L3_CAST_OFFS)
#define MVPP2_RXD_L2_CAST_OFFS		12
#define MVPP2_RXD_L2_CAST_BITS		2
#define MVPP2_RXD_L2_CAST_MASK		(((1 << \
		MVPP2_RXD_L2_CAST_BITS) - 1) << MVPP2_RXD_L2_CAST_OFFS)
#define MVPP2_RXD_VLAN_INFO_OFFS	14
#define MVPP2_RXD_VLAN_INFO_BITS	2
#define MVPP2_RXD_VLAN_INFO_MASK	(((1 << \
		MVPP2_RXD_VLAN_INFO_BITS) - 1) << MVPP2_RXD_VLAN_INFO_OFFS)
/* Bits of "bmQset" field */
#define MVPP2_RXD_BUFF_QSET_NUM_OFFS	0
#define MVPP2_RXD_BUFF_QSET_NUM_MASK	(0x7f << MVPP2_RXD_BUFF_QSET_NUM_OFFS)
#define MVPP2_RXD_BUFF_TYPE_OFFS	7
#define MVPP2_RXD_BUFF_TYPE_MASK	(0x1 << MVPP2_RXD_BUFF_TYPE_OFFS)
/* Bits of "status" field */
#define MVPP2_RXD_L3_OFFSET_OFFS	0
#define MVPP2_RXD_L3_OFFSET_MASK	(0x7F << MVPP2_RXD_L3_OFFSET_OFFS)
#define MVPP2_RXD_IP_HLEN_OFFS		8
#define MVPP2_RXD_IP_HLEN_MASK		(0x1F << MVPP2_RXD_IP_HLEN_OFFS)
#define MVPP2_RXD_ES_BIT		15
#define MVPP2_RXD_ES_MASK		BIT(MVPP2_RXD_ES_BIT)
#define MVPP2_RXD_HWF_SYNC_BIT		21
#define MVPP2_RXD_HWF_SYNC_MASK		BIT(MVPP2_RXD_HWF_SYNC_BIT)
#define MVPP2_RXD_L4_CHK_OK_BIT		22
#define MVPP2_RXD_L4_CHK_OK_MASK	BIT(MVPP2_RXD_L4_CHK_OK_BIT)
#define MVPP2_RXD_IP_FRAG_BIT		23
#define MVPP2_RXD_IP_FRAG_MASK		BIT(MVPP2_RXD_IP_FRAG_BIT)
#define MVPP2_RXD_IP4_HEADER_ERR_BIT	24
#define MVPP2_RXD_IP4_HEADER_ERR_MASK	BIT(MVPP2_RXD_IP4_HEADER_ERR_BIT)
#define MVPP2_RXD_L4_OFFS		25
#define MVPP2_RXD_L4_MASK		(7 << MVPP2_RXD_L4_OFFS)
/* Value 0 - N/A, 3-7 - User Defined */
#define MVPP2_RXD_L3_OFFS		28
#define MVPP2_RXD_L3_MASK		(7 << MVPP2_RXD_L3_OFFS)
/* Value 0 - N/A, 6-7 - User Defined */
#define MVPP2_RXD_L3_IP4_OPT		(2 << MVPP2_RXD_L3_OFFS)
#define MVPP2_RXD_L3_IP4_OTHER		(3 << MVPP2_RXD_L3_OFFS)
#define MVPP2_RXD_L3_IP6_EXT		(5 << MVPP2_RXD_L3_OFFS)
#define MVPP2_RXD_BUF_HDR_BIT		31
#define MVPP2_RXD_BUF_HDR_MASK		BIT(MVPP2_RXD_BUF_HDR_BIT)
/* status field MACROs */
#define MVPP2_RXD_L3_IS_IP4(status)		(((status) & \
				MVPP2_RXD_L3_MASK) == MVPP2_RXD_L3_IP4)
#define MVPP2_RXD_L3_IS_IP4_OPT(status)		(((status) & \
				MVPP2_RXD_L3_MASK) == MVPP2_RXD_L3_IP4_OPT)
#define MVPP2_RXD_L3_IS_IP4_OTHER(status)	(((status) & \
				MVPP2_RXD_L3_MASK) == MVPP2_RXD_L3_IP4_OTHER)
#define MVPP2_RXD_L3_IS_IP6(status)		(((status) & \
				MVPP2_RXD_L3_MASK) == MVPP2_RXD_L3_IP6)
#define MVPP2_RXD_L3_IS_IP6_EXT(status)		(((status) & \
				MVPP2_RXD_L3_MASK) == MVPP2_RXD_L3_IP6_EXT)
#define MVPP2_RXD_L4_IS_UDP(status)		(((status) & \
				MVPP2_RXD_L4_MASK) == MVPP2_RXD_L4_UDP)
#define MVPP2_RXD_L4_IS_TCP(status)		(((status) & \
				MVPP2_RXD_L4_MASK) == MVPP2_RXD_L4_TCP)
#define MVPP2_RXD_IP4_HDR_ERR(status)		((status) & \
				MVPP2_RXD_IP4_HEADER_ERR_MASK)
#define MVPP2_RXD_IP4_FRG(status)		((status) & \
				MVPP2_RXD_IP_FRAG_MASK)
#define MVPP2_RXD_L4_CHK_OK(status)		((status) & \
				MVPP2_RXD_L4_CHK_OK_MASK)

struct pp21_specific_tx_desc {
	u32 buf_phys_addr;	/* physical addr of transmitted buffer	*/
	u32 buf_cookie;	/* cookie for access to TX buffer in tx path */
	u32 rsrvd_hw_cmd[3];	/* hw_cmd (for future use, BM, PON, PNC) */
	u32 rsrvd1;		/* reserved (for future use)		*/
};

struct pp22_specific_tx_desc {
	u64 rsrvd_hw_cmd1;	/* hw_cmd (BM, PON, PNC) */
	u64 buf_phys_addr_hw_cmd2;
	u64 buf_cookie_bm_qset_hw_cmd3;
		/* cookie for access to RX buffer in rx path */
		/* cookie for access to RX buffer in rx path */
		/* bm_qset (for future use, BM)		*/
		/* classify_info (for future use, PnC)	*/
};

union pp2x_specific_tx_desc {
	struct pp21_specific_tx_desc pp21;
	struct pp22_specific_tx_desc pp22;
};

struct mv_pp2x_tx_desc {
	u32 command;		/* Options used by HW for packet xmitting */
	u8  packet_offset;	/* the offset from the buffer beginning	*/
	u8  phys_txq;		/* destination queue ID			*/
	u16 data_size;		/* data size of transmitted packet in bytes */
	union pp2x_specific_tx_desc u;
};

struct pp21_specific_rx_desc {
	u32 buf_phys_addr;	/* physical address of the buffer */
	u32 buf_cookie;	/* cookie for access to RX buffer in rx path */
	u16 rsrvd_gem;		/* gem_port_id (for future use, PON) */
	u16 rsrvd_l4csum;	/* csum_l4 (for future use, PnC) */
	u8  rsrvd_bm_qset;	/* bm_qset (for future use, BM) */
	u8  rsrvd1;
	u16 rsrvd_cls_info;	/* classify_info (for future use, PnC)	*/
	u32 rsrvd_flow_id;	/* flow_id (for future use, PnC) */
	u32 rsrvd_abs;
};

struct pp22_specific_rx_desc {
	u16 rsrvd_gem;		/* gem_port_id (for future use, PON)	*/
	u16 rsrvd_l4csum;	/* csum_l4 (for future use, PnC)	*/
	u32 rsrvd_timestamp;
	u64 buf_phys_addr_key_hash;
	u64 buf_cookie_bm_qset_cls_info;
	/* cookie for access to RX buffer in rx path */
	/* bm_qset (for future use, BM)		*/
	/* classify_info (for future use, PnC)	*/
};

union pp2x_specific_rx_desc {
	struct pp21_specific_rx_desc pp21;
	struct pp22_specific_rx_desc pp22;
};

struct mv_pp2x_rx_desc {
	u32 status;		/* info about received packet */
	u16 rsrvd_parser;	/* parser_info (for future use, PnC)	*/
	u16 data_size;		/* size of received packet in bytes	*/
	union pp2x_specific_rx_desc u;
};

union mv_pp2x_prs_tcam_entry {
	u32 word[MVPP2_PRS_TCAM_WORDS];
	u8  byte[MVPP2_PRS_TCAM_WORDS * 4];
};

union mv_pp2x_prs_sram_entry {
	u32 word[MVPP2_PRS_SRAM_WORDS];
	u8  byte[MVPP2_PRS_SRAM_WORDS * 4];
};

struct mv_pp2x_prs_entry {
	u32 index;
	union mv_pp2x_prs_tcam_entry tcam;
	union mv_pp2x_prs_sram_entry sram;
};

struct mv_pp2x_prs_shadow {
	u32 valid;				/* Entry is valid or not */
	int lu;					/* Lookup ID */
	u32 ri;					/* Result info */
	u32 ri_mask;				/* Result info mask*/
	union mv_pp2x_prs_tcam_entry tcam;	/* TCAM */
	u32 valid_in_kernel;			/* Used for restoring kernel parser at deinit */
	u32 prs_mac_range_start;
	u32 prs_mac_range_end;
};

struct mv_pp2x_cls_flow_entry {
	u32 index;
	u32 data[MVPP2_CLS_FLOWS_TBL_DATA_WORDS];
};

struct mv_pp2x_cls_lookup_entry {
	u32 lkpid;
	u32 way;
	u32 data;
};

struct mv_pp2x_cls_flow_info {
	u32 lkpid;
	/* The flow table entry index of CoS default rule */
	u32 flow_entry_dflt;
	/* The flow table entry index of CoS VLAN rule */
	u32 flow_entry_vlan;
	/* The flow table entry index of CoS DSCP rule */
	u32 flow_entry_dscp;
	/* The flow table entry index of RSS rule */
	u32 flow_entry_rss1;
	/* The flow table entry index of RSS rule for UDP packet to
	 * update hash mode
	 */
	u32 flow_entry_rss2;
};

/* The flow entry could become lkp pointer in lookup table */
enum mv_pp2x_cls_lkp_ptr_candidate {
	MVPP2_LKP_PTR_FLOW_DEFAULT,
	MVPP2_LKP_PTR_FLOW_VLAN,
	MVPP2_LKP_PTR_FLOW_DSCP,
	MVPP2_LKP_PTR_NUM
};

struct mv_pp2x_cls_shadow {
	struct mv_pp2x_cls_flow_info *flow_info;
	u32 flow_free_start; /* The start of free entry index in flow table */
	/* TODO: does need a spin_lock for flow_free_start? */
};

/* Classifier engine2 and QoS structure */

/* C2  constants */
#define MVPP2_CLS_C2_TCAM_SIZE			256
#define MVPP2_CLS_C2_TCAM_WORDS			5
#define MVPP2_CLS_C2_TCAM_DATA_BYTES		10
#define MVPP2_CLS_C2_SRAM_WORDS			5
#define MVPP2_CLS_C2_HEK_LKP_TYPE_OFFS		0
#define MVPP2_CLS_C2_HEK_LKP_TYPE_BITS		6
#define MVPP2_CLS_C2_HEK_LKP_TYPE_MASK		(0x3F << \
					MVPP2_CLS_C2_HEK_LKP_TYPE_OFFS)
#define MVPP2_CLS_C2_HEK_PORT_TYPE_OFFS		6
#define MVPP2_CLS_C2_HEK_PORT_TYPE_BITS		2
#define MVPP2_CLS_C2_HEK_PORT_TYPE_MASK		(0x3 << \
					MVPP2_CLS_C2_HEK_PORT_TYPE_OFFS)
#define MVPP2_CLS_C2_QOS_DSCP_TBL_SIZE		64
#define MVPP2_CLS_C2_QOS_PRIO_TBL_SIZE		8
#define MVPP2_CLS_C2_QOS_DSCP_TBL_NUM		8
#define MVPP2_CLS_C2_QOS_PRIO_TBL_NUM		64

struct mv_pp2x_cls_c2_entry {
	u32          index;
	u32         inv;
	union {
		u32	words[MVPP2_CLS_C2_TCAM_WORDS];
		u8	bytes[MVPP2_CLS_C2_TCAM_WORDS * 4];
	} tcam;
	union {
		u32	words[MVPP2_CLS_C2_SRAM_WORDS];
		struct {
			u32 action_tbl; /* 0x1B30 */
			u32 actions;    /* 0x1B60 */
			u32 qos_attr;   /* 0x1B64*/
			u32 hwf_attr;   /* 0x1B68 */
			u32 rss_attr;   /* 0x1B6C */
			u32 seq_attr;   /* 0x1B70 */
		} regs;
	} sram;
};

enum mv_pp2x_cls2_hek_offs {
	MVPP2_CLS_C2_HEK_OFF_BYTE0 = 0,
	MVPP2_CLS_C2_HEK_OFF_BYTE1,
	MVPP2_CLS_C2_HEK_OFF_BYTE2,
	MVPP2_CLS_C2_HEK_OFF_BYTE3,
	MVPP2_CLS_C2_HEK_OFF_BYTE4,
	MVPP2_CLS_C2_HEK_OFF_BYTE5,
	MVPP2_CLS_C2_HEK_OFF_BYTE6,
	MVPP2_CLS_C2_HEK_OFF_BYTE7,
	MVPP2_CLS_C2_HEK_OFF_LKP_PORT_TYPE,
	MVPP2_CLS_C2_HEK_OFF_PORT_ID,
	MVPP2_CLS_C2_HEK_OFF_MAX
};

struct mv_pp2x_cls_c2_qos_entry {
	u32 tbl_id;
	u32 tbl_sel;
	u32 tbl_line;
	u32 data;
};

enum mv_pp2x_src_port_type {
	MVPP2_SRC_PORT_TYPE_PHY,
	MVPP2_SRC_PORT_TYPE_UNI,
	MVPP2_SRC_PORT_TYPE_VIR,
	MVPP2_SRC_PORT_TYPE_MAX
};

struct mv_pp2x_src_port {
	enum mv_pp2x_src_port_type	port_type;
	u32				port_value;
	u32				port_mask;
};

enum mv_pp2x_qos_tbl_sel {
	MVPP2_QOS_TBL_SEL_PRI = 0,
	MVPP2_QOS_TBL_SEL_DSCP,
};

enum mv_pp2x_qos_src_tbl {
	MVPP2_QOS_SRC_ACTION_TBL = 0,
	MVPP2_QOS_SRC_DSCP_PBIT_TBL,
};

struct mv_pp2x_engine_qos_info {
	/* dscp pri table or none */
	enum mv_pp2x_qos_tbl_sel	qos_tbl_type;
	/* dscp or pri table index */
	u32				qos_tbl_index;
	/* policer id, 0xffff do not assign policer */
	u16				policer_id;
	/* pri/dscp comes from qos or act tbl */
	enum mv_pp2x_qos_src_tbl	pri_dscp_src;
	/* gemport comes from qos or act tbl */
	enum mv_pp2x_qos_src_tbl	gemport_src;
	enum mv_pp2x_qos_src_tbl	q_low_src;
	enum mv_pp2x_qos_src_tbl	q_high_src;
	enum mv_pp2x_qos_src_tbl	color_src;
};

enum mv_pp2x_color_action_type {
	/* Do not update color */
	MVPP2_COLOR_ACTION_TYPE_NO_UPDT = 0,
	/* Do not update color and lock */
	MVPP2_COLOR_ACTION_TYPE_NO_UPDT_LOCK,
	/* Update to green */
	MVPP2_COLOR_ACTION_TYPE_GREEN,
	/* Update to green and lock */
	MVPP2_COLOR_ACTION_TYPE_GREEN_LOCK,
	/* Update to yellow */
	MVPP2_COLOR_ACTION_TYPE_YELLOW,
	/* Update to yellow */
	MVPP2_COLOR_ACTION_TYPE_YELLOW_LOCK,
	/* Update to red */
	MVPP2_COLOR_ACTION_TYPE_RED,
	/* Update to red and lock */
	MVPP2_COLOR_ACTION_TYPE_RED_LOCK,
};

enum mv_pp2x_general_action_type {
	/* The field will be not updated */
	MVPP2_ACTION_TYPE_NO_UPDT,
	/* The field will be not updated and lock */
	MVPP2_ACTION_TYPE_NO_UPDT_LOCK,
	/* The field will be updated */
	MVPP2_ACTION_TYPE_UPDT,
	/* The field will be updated and lock */
	MVPP2_ACTION_TYPE_UPDT_LOCK,
};

enum mv_pp2x_flowid_action_type {
	/* FlowID is disable */
	MVPP2_ACTION_FLOWID_DISABLE = 0,
	/* FlowID is enable */
	MVPP2_ACTION_FLOWID_ENABLE,
};

enum mv_pp2x_frwd_action_type {
	/* The decision will be not updated */
	MVPP2_FRWD_ACTION_TYPE_NO_UPDT,
	/* The decision is not updated, and following no change to it */
	MVPP2_FRWD_ACTION_TYPE_NO_UPDT_LOCK,
	/* The packet to CPU (Software Forwarding) */
	MVPP2_FRWD_ACTION_TYPE_SWF,
	 /* The packet to CPU, and following no change to it */
	MVPP2_FRWD_ACTION_TYPE_SWF_LOCK,
	/* The packet to one transmit port (Hardware Forwarding) */
	MVPP2_FRWD_ACTION_TYPE_HWF,
	/* The packet to one tx port, and following no change to it */
	MVPP2_FRWD_ACTION_TYPE_HWF_LOCK,
	/* The pkt to one tx port, and maybe internal packets is used */
	MVPP2_FRWD_ACTION_TYPE_HWF_LOW_LATENCY,
	/* Same to above, but following no change to it*/
	MVPP2_FRWD_ACTION_TYPE_HWF_LOW_LATENCY_LOCK,
};

struct mv_pp2x_engine_pkt_action {
	enum mv_pp2x_color_action_type		color_act;
	enum mv_pp2x_general_action_type	pri_act;
	enum mv_pp2x_general_action_type	dscp_act;
	enum mv_pp2x_general_action_type	gemp_act;
	enum mv_pp2x_general_action_type	q_low_act;
	enum mv_pp2x_general_action_type	q_high_act;
	enum mv_pp2x_general_action_type	policer_act;
	enum mv_pp2x_general_action_type	rss_act;
	enum mv_pp2x_flowid_action_type		flowid_act;
	enum mv_pp2x_frwd_action_type		frwd_act;
};

struct mv_pp2x_qos_value {
	u16		pri;
	u16		dscp;
	u16		gemp;
	u16		q_low;
	u16		q_high;
};

struct mv_pp2x_engine_pkt_mod {
	u32		mod_cmd_idx;
	u32		mod_data_idx;
	u32		l4_chksum_update_flag;
};

struct mv_pp2x_duplicate_info {
	/* pkt duplication flow id */
	u32		flow_id;
	/* pkt duplication count */
	u32		flow_cnt;
};

/* The logic C2 entry, easy to understand and use */
struct mv_pp2x_c2_add_entry {
	struct mv_pp2x_src_port		port;
	u8				lkp_type;
	u8				lkp_type_mask;
	/* priority in this look_type */
	u32			priority;
	struct pp2_cls_mng_pkt_key_t	*mng_pkt_key;	/* pkt key value */
	/* all the qos input */
	struct mv_pp2x_engine_qos_info	qos_info;
	/* update&lock info */
	struct mv_pp2x_engine_pkt_action action;
	/* pri/dscp/gemport/qLow/qHigh */
	struct mv_pp2x_qos_value	qos_value;
	/* PMT cmd_idx and data_idx */
	struct mv_pp2x_engine_pkt_mod	pkt_mod;
	/* RSS enable or disable */
	int				rss_en;
	/* pkt duplication flow info */
	struct mv_pp2x_duplicate_info	flow_info;
};

struct mv_pp2x_c2_rule_idx {
	/* The TCAM rule index for VLAN pri check with QoS pbit table */
	u32 vlan_pri_idx;
	/* The TCAM rule index for DSCP check with QoS dscp table */
	u32 dscp_pri_idx;
	/* The default rule for flow untagged and non-IP */
	u32 default_rule_idx;
};

struct mv_pp2x_c2_shadow {
	int c2_tcam_free_start;
	/* Per src port */
	struct mv_pp2x_c2_rule_idx rule_idx_info[8];
};

struct mv_pp2x_buff_hdr {
	u32 next_buff_phys_addr;
	u32 next_buff_virt_addr;
	u16 byte_count;
	u16 info;
	u8  reserved1;		/* bm_qset (for future use, BM) */
};

/* Buffer header info bits */
#define MVPP2_B_HDR_INFO_MC_ID_MASK	0xfff
#define MVPP2_B_HDR_INFO_MC_ID(info)	((info) & MVPP2_B_HDR_INFO_MC_ID_MASK)
#define MVPP2_B_HDR_INFO_LAST_OFFS	12
#define MVPP2_B_HDR_INFO_LAST_MASK	BIT(12)
#define MVPP2_B_HDR_INFO_IS_LAST(info) \
	   ((info & MVPP2_B_HDR_INFO_LAST_MASK) >> MVPP2_B_HDR_INFO_LAST_OFFS)

/* Macroes */
#define MVPP2_RX_DESC_POOL(rx_desc)	((rx_desc->status & \
		MVPP2_RXD_BM_POOL_ID_MASK) >> MVPP2_RXD_BM_POOL_ID_OFFS)

/* RSS related definetions */
enum mv_pp22_rss_access_sel {
	MVPP22_RSS_ACCESS_POINTER,
	MVPP22_RSS_ACCESS_TBL,
};

enum mv_pp2_rss_hash_select {
	MVPP2_RSS_HASH_0_4,
	MVPP2_RSS_HASH_5_9,
};

/* Structure dexcribe RXQ and corresponding rss table */
struct mv_pp22_rss_tbl_ptr {
	u8 rxq_idx;
	u8 rss_tbl_ptr;
};

/* Normal RSS entry */
struct mv_pp22_rss_tbl_entry {
	u8 tbl_id;
	u8 tbl_line;
	u8 width;
	u8 rxq;
};

union mv_pp22_rss_access_entry {
	struct mv_pp22_rss_tbl_ptr pointer;
	struct mv_pp22_rss_tbl_entry entry;
};

struct mv_pp22_rss_entry {
	enum mv_pp22_rss_access_sel sel;
	union mv_pp22_rss_access_entry u;
};

/* C3 or other module definetions */
#define MVPP2_CLS_C3_HASH_TBL_SIZE			(4096)
#define MVPP2_CLS_C3_MISS_TBL_SIZE			(64)
#define MVPP2_CLS_C3_EXT_HEK_WORDS			(9)
#define MVPP2_CLS_C3_SRAM_WORDS				(5)
#define MVPP2_CLS_C3_EXT_TBL_SIZE			(256)
#define MVPP2_CLS_C3_HEK_WORDS				(3)
#define MVPP2_CLS_C3_HEK_BYTES				12 /* size in bytes */
#define MVPP2_CLS_C3_BANK_SIZE				(512)
#define MVPP2_CLS_C3_MAX_SEARCH_DEPTH			(16)

/* Classifier C3 offsets in hash table */
#define KEY_OCCUPIED				(116)
#define KEY_FORMAT				(115)
#define KEY_PTR_EXT				(107)

#define KEY_PRT_ID(ext_mode)			((ext_mode == 1) ? (99) : (107))
#define KEY_PRT_ID_MASK(ext_mode)		(((1 << KEY_CTRL_PRT_ID_BITS) - 1) << (KEY_PRT_ID(ext_mode) % 32))

#define KEY_PRT_ID_TYPE(ext_mode)		((ext_mode == 1) ? (97) : (105))
#define KEY_PRT_ID_TYPE_MASK(ext_mode)		((KEY_CTRL_PRT_ID_TYPE_MAX) << (KEY_PRT_ID_TYPE(ext_mode) % 32))

#define KEY_LKP_TYPE(ext_mode)			((ext_mode == 1) ? (91) : (99))
#define KEY_LKP_TYPE_MASK(ext_mode)		(((1 << KEY_CTRL_LKP_TYPE_BITS) - 1) << (KEY_LKP_TYPE(ext_mode) % 32))

#define KEY_L4_INFO(ext_mode)			((ext_mode == 1) ? (88) : (96))
#define KEY_L4_INFO_MASK(ext_mode)		(((1 << KEY_CTRL_L4_BITS) - 1) << (KEY_L4_INFO(ext_mode) % 32))

#define KEY_CTRL_LKP_TYPE			4
#define KEY_CTRL_LKP_TYPE_BITS			6

#define KEY_CTRL_LKP_TYPE_MAX			((1 << KEY_CTRL_LKP_TYPE_BITS) - 1)
#define KEY_CTRL_LKP_TYPE_MASK			(((1 << KEY_CTRL_LKP_TYPE_BITS) - 1) << KEY_CTRL_LKP_TYPE)

#define KEY_CTRL_PRT_ID_TYPE			12
#define KEY_CTRL_PRT_ID_TYPE_BITS		2
#define KEY_CTRL_PRT_ID_TYPE_MAX		((1 << KEY_CTRL_PRT_ID_TYPE_BITS) - 1)
#define KEY_CTRL_PRT_ID_TYPE_MASK		((KEY_CTRL_PRT_ID_TYPE_MAX) << KEY_CTRL_PRT_ID_TYPE)

#define KEY_CTRL_PRT_ID				16
#define KEY_CTRL_PRT_ID_BITS			8
#define KEY_CTRL_PRT_ID_MAX			((1 << KEY_CTRL_PRT_ID_BITS) - 1)
#define KEY_CTRL_PRT_ID_MASK			(((1 << KEY_CTRL_PRT_ID_BITS) - 1) << KEY_CTRL_PRT_ID)

#define KEY_CTRL_HEK_SIZE			24
#define KEY_CTRL_HEK_SIZE_BITS			6
#define KEY_CTRL_HEK_SIZE_MAX			36
#define KEY_CTRL_HEK_SIZE_MASK			(((1 << KEY_CTRL_HEK_SIZE_BITS) - 1) << KEY_CTRL_HEK_SIZE)

struct pp2_cls_c3_hash_pair {
	u16 pair_num;
	u16 old_idx[MVPP2_CLS_C3_MAX_SEARCH_DEPTH];
	u16 new_idx[MVPP2_CLS_C3_MAX_SEARCH_DEPTH];
};

struct pp2_cls_c3_entry {
	u32 index;
	u32 ext_index;

	struct {
		union {
			u32 words[MVPP2_CLS_C3_EXT_HEK_WORDS];
			u8 bytes[MVPP2_CLS_C3_EXT_HEK_WORDS * 4];
		} hek;
		u32 key_ctrl;/*0x1C10*/
	} key;
	union {
		u32 words[MVPP2_CLS_C3_SRAM_WORDS];
		struct {
			u32 actions;/*0x1D40*/
			u32 qos_attr;/*0x1D44*/
			u32 hwf_attr;/*0x1D48*/
			u32 dup_attr;/*0x1D4C*/
			u32 seq_l_attr;/*0x1D50*/
			u32 seq_h_attr;/*0x1D54*/
		} regs;
	} sram;
};

struct pp2_cls_c3_shadow_hash_entry {
	/* valid if size > 0 */
	/* size include the extension*/
	int ext_ptr;
	int size;
};

/* Classifier C4 Top Registers */
#define MVPP2_CLS4_PHY_TO_RL_REG(port)			(0x1E00 + ((port) * 4))
#define MVPP2_CLS4_PHY_TO_RL_GRP			0
#define MVPP2_CLS4_PHY_TO_RL_GRP_BITS			3
#define MVPP2_CLS4_PHY_TO_RL_GRP_MASK			(((1 << MVPP2_CLS4_PHY_TO_RL_GRP_BITS) - 1) << \
							 MVPP2_CLS4_PHY_TO_RL_GRP)
#define MVPP2_CLS4_PHY_TO_RL_RULE_NUM			4
#define MVPP2_CLS4_PHY_TO_RL_RULE_NUM_BITS		4
#define MVPP2_CLS4_PHY_TO_RL_RULE_NUM_MASK		(((1 << MVPP2_CLS4_PHY_TO_RL_RULE_NUM_BITS) - 1) << \
							 MVPP2_CLS4_PHY_TO_RL_RULE_NUM)

#define MVPP2_CLS4_UNI_TO_RL_REG(uni)			(0x1E20 + ((uni) * 4))
#define MVPP2_CLS4_UNI_TO_RL_GRP			0
#define MVPP2_CLS4_UNI_TO_RL_RULE_NUM			4

#define MVPP2_CLS4_RL_INDEX_REG				(0x1E40)
#define MVPP2_CLS4_RL_INDEX_RULE			0
#define MVPP2_CLS4_RL_INDEX_GRP				3

#define MVPP2_CLS4_FATTR1_REG				(0x1E50)
#define MVPP2_CLS4_FATTR2_REG				(0x1E54)
#define MVPP2_CLS4_FATTR_REG_NUM			2

#define MVPP2_CLS4_FATTR_ID(field)			(((field) * 9) % 27)
#define MVPP2_CLS4_FATTR_ID_BITS			6
#define MVPP2_CLS4_FATTR_ID_MAX				((1 << MVPP2_CLS4_FATTR_ID_BITS) - 1)
#define MVPP2_CLS4_FATTR_ID_MASK(field)			(MVPP2_CLS4_FATTR_ID_MAX << MVPP2_CLS4_FATTR_ID(field))
#define MVPP2_CLS4_FATTR_ID_VAL(field, reg_val)		((reg_val & MVPP2_CLS4_FATTR_ID_MASK(field)) >> \
							 MVPP2_CLS4_FATTR_ID(field))

#define MVPP2_CLS4_FATTR_OPCODE_BITS			3
#define MVPP2_CLS4_FATTR_OPCODE(field)			((((field) * 9) % 27) + MVPP2_CLS4_FATTR_ID_BITS)
#define MVPP2_CLS4_FATTR_OPCODE_MAX			((1 << MVPP2_CLS4_FATTR_OPCODE_BITS) - 1)
#define MVPP2_CLS4_FATTR_OPCODE_MASK(field)		(MVPP2_CLS4_FATTR_OPCODE_MAX << MVPP2_CLS4_FATTR_OPCODE(field))
#define MVPP2_CLS4_FATTR_OPCODE_VAL(field, reg_val)	((reg_val & MVPP2_CLS4_FATTR_OPCODE_MASK(field)) >> \
							 MVPP2_CLS4_FATTR_OPCODE(field))

#define MVPP2_CLS4_FDATA1_REG				(0x1E58)
#define MVPP2_CLS4_FDATA2_REG				(0x1E5C)
#define MVPP2_CLS4_FDATA3_REG				(0x1E60)
#define MVPP2_CLS4_FDATA4_REG				(0x1E64)
#define MVPP2_CLS4_FDATA5_REG				(0x1E68)
#define MVPP2_CLS4_FDATA6_REG				(0x1E6C)
#define MVPP2_CLS4_FDATA7_REG				(0x1E70)
#define MVPP2_CLS4_FDATA8_REG				(0x1E74)
#define MVPP2_CLS4_FDATA_REG(reg_num)			(0x1E58 + (4 * (reg_num)))
#define MVPP2_CLS4_FDATA_REGS_NUM			8

#define MVPP2_CLS4_FDATA7_L3INFO			16
#define MVPP2_CLS4_FDATA7_L3INFO_BITS			4
#define MVPP2_CLS4_L3INFO_MAX				((1 << MVPP2_CLS4_FDATA7_L3INFO_BITS) - 1)
#define MVPP2_CLS4_L3INFO_MASK				(MVPP2_CLS4_L3INFO_MAX << MVPP2_CLS4_FDATA7_L3INFO)
#define MVPP2_CLS4_L3INFO_VAL(reg_val)			(((reg_val) & MVPP2_CLS4_L3INFO_MASK) >> \
							 MVPP2_CLS4_FDATA7_L3INFO)

#define MVPP2_CLS4_FDATA7_L4INFO			20
#define MVPP2_CLS4_FDATA7_L4INFO_BITS			4
#define MVPP2_CLS4_L4INFO_MAX				((1 << MVPP2_CLS4_FDATA7_L4INFO_BITS) - 1)
#define MVPP2_CLS4_L4INFO_MASK				(MVPP2_CLS4_L4INFO_MAX << MVPP2_CLS4_FDATA7_L4INFO)
#define MVPP2_CLS4_L4INFO_VAL(reg_val)			(((reg_val) & MVPP2_CLS4_L4INFO_MASK) >> \
							 MVPP2_CLS4_FDATA7_L4INFO)

#define MVPP2_CLS4_FDATA7_MACME				24
#define MVPP2_CLS4_FDATA7_MACME_BITS			2
#define MVPP2_CLS4_MACME_MAX				((1 << MVPP2_CLS4_FDATA7_MACME_BITS) - 1)
#define MVPP2_CLS4_MACME_MASK				(MVPP2_CLS4_MACME_MAX << MVPP2_CLS4_FDATA7_MACME)
#define MVPP2_CLS4_MACME_VAL(reg_val)			(((reg_val) & MVPP2_CLS4_MACME_MASK) >> MVPP2_CLS4_FDATA7_MACME)

#define MVPP2_CLS4_FDATA7_PPPOE				26
#define MVPP2_CLS4_FDATA7_PPPOE_BITS			2
#define MVPP2_CLS4_PPPOE_MAX				((1 << MVPP2_CLS4_FDATA7_PPPOE_BITS) - 1)
#define MVPP2_CLS4_PPPOE_MASK				(MVPP2_CLS4_PPPOE_MAX << MVPP2_CLS4_FDATA7_PPPOE)
#define MVPP2_CLS4_PPPOE_VAL(reg_val)			(((reg_val) & MVPP2_CLS4_PPPOE_MASK) >> MVPP2_CLS4_FDATA7_PPPOE)

#define MVPP2_CLS4_FDATA7_VLAN				28
#define MVPP2_CLS4_FDATA7_VLAN_BITS			3
#define MVPP2_CLS4_VLAN_MAX				((1 << MVPP2_CLS4_FDATA7_VLAN_BITS) - 1)
#define MVPP2_CLS4_VLAN_MASK				(MVPP2_CLS4_VLAN_MAX << MVPP2_CLS4_FDATA7_VLAN)
#define MVPP2_CLS4_VLAN_VAL(reg_val)			(((reg_val) & MVPP2_CLS4_VLAN_MASK) >> MVPP2_CLS4_FDATA7_VLAN)

#define MVPP2_CLS4_ACT_REG				(0x1E80)
#define MVPP2_CLS4_ACT_QOS_ATTR_REG			(0x1E84)
#define MVPP2_CLS4_ACT_DUP_ATTR_REG			(0x1E88)
#define MVPP2_CNT_IDX_RULE(rule, set)			((rule) << 3 | (set))
#define MVPP2_CLS_C4_TBL_HIT_REG			(0x7708)

/* Classifier C4 constants */
#define MVPP2_CLS_C4_GRP_SIZE				(8)
#define MVPP2_CLS_C4_GRPS_NUM				(8)
#define MVPP2_CLS_C4_TBL_WORDS				(10)
#define MVPP2_CLS_C4_TBL_DATA_WORDS			(8)
#define MVPP2_CLS_C4_SRAM_WORDS				(3)
#define MVPP2_CLS_C4_FIELDS_NUM				(6)

/* C4 entry structure */
struct mv_pp2x_cls_c4_entry {
	u32 rule_index;
	u32 set_index;
	union {
		u32	words[MVPP2_CLS_C4_TBL_WORDS];
		struct {
			u32 attr[MVPP2_CLS4_FATTR_REG_NUM];
			u32 fdata_arr[MVPP2_CLS_C4_TBL_DATA_WORDS];
		} regs;
	} rules;
	union {
		u32 words[MVPP2_CLS_C4_SRAM_WORDS];
		struct {
			u32 actions;/* 0x1E80 */
			u32 qos_attr;/* 0x1E84*/
			u32 dup_attr;/* 0x1E88 */
		} regs;
	} sram;
};

/************** TX Packet Modification Registers *******************/
#define MVPP2_PME_TBL_IDX_REG			(0x8400)
#define MVPP2_PME_TBL_INSTR_REG			(0x8480)
/*--------------------------------------------------------------------------*/
#define MVPP2_PME_TBL_DATA1_REG			(0x8500)
#define MVPP2_PME_TBL_DATA2_REG			(0x8580)
#define MVPP2_PME_TBL_DATA_BITS			16
#define MVPP2_PME_TBL_DATA_OFFS(idx)		((idx == 0) ? MVPP2_PME_TBL_DATA_BITS : 0)
#define MVPP2_PME_TBL_DATA_MASK(idx)		(((1 << MVPP2_PME_TBL_DATA_BITS) - 1) << MVPP2_PME_TBL_DATA_OFFS(idx))
/*--------------------------------------------------------------------------*/
#define MVPP2_PME_TBL_STATUS_REG		(0x8600)
#define MVPP2_PME_TCONT_THRESH_REG		(0x8604)
#define MVPP2_PME_MTU_REG			(0x8608)

#define MVPP2_PME_MAX_VLAN_ETH_TYPES		4
#define MVPP2_PME_VLAN_ETH_TYPE_REG(i)		(0x8610 + ((i) << 2))
#define MVPP22_HIF_ALLOCATION_REG		(0x8610)
/*--------------------------------------------------------------------------*/
#define MVPP2_PME_DEF_VLAN_CFG_REG			(0x8620)
/*--------------------------------------------------------------------------*/
#define MVPP2_PME_MAX_DSA_ETH_TYPES		2
#define MVPP2_PME_DEF_DSA_CFG_REG(i)		(0x8624 + ((i) << 2))
/*--------------------------------------------------------------------------*/
#define MVPP2_PME_DEF_DSA_SRC_DEV_REG		(0x8630)
#define MVPP2_PME_DSA_SRC_DEV_OFFS		1
#define MVPP2_PME_DSA_SRC_DEV_BITS		4
#define MVPP2_PME_DSA_SRC_DEV_ALL_MASK		(((1 << MVPP2_PME_DSA_SRC_DEV_BITS) - 1) << MVPP2_PME_DSA_SRC_DEV_OFFS)
#define MVPP2_PME_DSA_SRC_DEV_MASK(dev)	((dev) << MVPP2_PME_DSA_SRC_DEV_OFFS)
/*--------------------------------------------------------------------------*/
#define MVPP2_PME_TTL_ZERO_FRWD_REG		(0x8640)
#define MVPP2_PME_TTL_ZERO_FRWD_BIT		0
#define MVPP2_PME_TTL_ZERO_FRWD_MASK		BIT(MVPP2_PME_TTL_ZERO_FRWD_BIT)
/*--------------------------------------------------------------------------*/
#define MVPP2_PME_PPPOE_ETYPE_REG		(0x8650)
#define MVPP2_PME_PPPOE_DATA_REG		(0x8654)

#define MVPP2_PME_PPPOE_CODE_OFFS		0
#define MVPP2_PME_PPPOE_CODE_BITS		8
#define MVPP2_PME_PPPOE_CODE_ALL_MASK		(((1 << MVPP2_PME_PPPOE_CODE_BITS) - 1) << MVPP2_PME_PPPOE_CODE_OFFS)
#define MVPP2_PME_PPPOE_CODE_MASK(code)		(((code) << MVPP2_PME_PPPOE_CODE_OFFS) & MVPP2_PME_PPPOE_CODE_ALL_MASK)

#define MVPP2_PME_PPPOE_TYPE_OFFS		8
#define MVPP2_PME_PPPOE_TYPE_BITS		4
#define MVPP2_PME_PPPOE_TYPE_ALL_MASK		(((1 << MVPP2_PME_PPPOE_TYPE_BITS) - 1) << MVPP2_PME_PPPOE_TYPE_OFFS)
#define MVPP2_PME_PPPOE_TYPE_MASK(type)	(((type) << MVPP2_PME_PPPOE_TYPE_OFFS) & MVPP2_PME_PPPOE_TYPE_ALL_MASK)

#define MVPP2_PME_PPPOE_VER_OFFS		12
#define MVPP2_PME_PPPOE_VER_BITS		4
#define MVPP2_PME_PPPOE_VER_ALL_MASK		(((1 << MVPP2_PME_PPPOE_VER_BITS) - 1) << MVPP2_PME_PPPOE_VER_OFFS)
#define MVPP2_PME_PPPOE_VER_MASK(ver)		(((ver) << MVPP2_PME_PPPOE_VER_OFFS) & MVPP2_PME_PPPOE_VER_ALL_MASK)

#define MVPP2_PME_PPPOE_LEN_REG			(0x8658)
#define MVPP2_PME_PPPOE_PROTO_REG		(0x865c)

#define MVPP2_PME_PPPOE_PROTO_OFFS(i)		((i == 0) ? 0 : 16)
#define MVPP2_PME_PPPOE_PROTO_BITS		(16)
#define MVPP2_PME_PPPOE_PROTO_ALL_MASK(i)	(((1 << MVPP2_PME_PPPOE_PROTO_BITS) - 1) << \
						 MVPP2_PME_PPPOE_PROTO_OFFS(i))
#define MVPP2_PME_PPPOE_PROTO_MASK(i, p)	(((p) << MVPP2_PME_PPPOE_PROTO_OFFS(i)) & \
						 MVPP2_PME_PPPOE_PROTO_ALL_MASK(i))

#define MVPP2_PME_CONFIG_REG			(0x8660)

#define MVPP2_PME_MAX_HDR_SIZE_OFFS		0
#define MVPP2_PME_MAX_HDR_SIZE_BITS		8
#define MVPP2_PME_MAX_HDR_SIZE_ALL_MASK		(((1 << MVPP2_PME_MAX_HDR_SIZE_BITS) - 1) << \
						 MVPP2_PME_MAX_HDR_SIZE_OFFS)
#define MVPP2_PME_MAX_HDR_SIZE_MASK(size)	(((size) << MVPP2_PME_MAX_HDR_SIZE_OFFS) & \
						 MVPP2_PME_MAX_HDR_SIZE_ALL_MASK)

#define MVPP2_PME_MAX_INSTR_NUM_OFFS		16
#define MVPP2_PME_MAX_INSTR_NUM_BITS		8
#define MVPP2_PME_MAX_INSTR_NUM_ALL_MASK	(((1 << MVPP2_PME_MAX_INSTR_NUM_BITS) - 1) << \
						 MVPP2_PME_MAX_INSTR_NUM_OFFS)
#define MVPP2_PME_MAX_INSTR_NUM_MASK(num)	(((num) << MVPP2_PME_MAX_INSTR_NUM_OFFS) & \
						 MVPP2_PME_MAX_INSTR_NUM_ALL_MASK)

#define MVPP2_PME_DROP_ON_ERR_BIT		24
#define MVPP2_PME_DROP_ON_ERR_MASK		BIT(MVPP2_PME_DROP_ON_ERR_BIT)
/*--------------------------------------------------------------------------*/

#define MVPP2_PME_STATUS_1_REG			(0x8664)
#define MVPP2_PME_STATUS_2_REG(txp)		(0x8700 + 4 * (txp))
#define MVPP2_PME_STATUS_3_REG(txp)		(0x8780 + 4 * (txp))

/* PME insructions table (MVPP2_PME_TBL_INSTR_REG) fields definition */
#define MVPP2_PME_DATA_OFFS			0
#define MVPP2_PME_DATA_BITS			16
#define MVPP2_PME_DATA_MASK			(((1 << MVPP2_PME_DATA_BITS) - 1) << MVPP2_PME_DATA_OFFS)

#define MVPP2_PME_CTRL_OFFS			16
#define MVPP2_PME_CTRL_BITS			16
#define MVPP2_PME_CTRL_MASK			(((1 << MVPP2_PME_CTRL_BITS) - 1) << MVPP2_PME_CTRL_OFFS)

#define MVPP2_PME_CMD_OFFS			16
#define MVPP2_PME_CMD_BITS			5
#define MVPP2_PME_CMD_ALL_MASK			(((1 << MVPP2_PME_CMD_BITS) - 1) << MVPP2_PME_CMD_OFFS)
#define MVPP2_PME_CMD_MASK(cmd)			((cmd) << MVPP2_PME_CMD_OFFS)

#define MVPP2_PME_IP4_CSUM_BIT			21
#define MVPP2_PME_IP4_CSUM_MASK			BIT(MVPP2_PME_IP4_CSUM_BIT)

#define MVPP2_PME_L4_CSUM_BIT			22
#define MVPP2_PME_L4_CSUM_MASK			BIT(MVPP2_PME_L4_CSUM_BIT)

#define MVPP2_PME_LAST_BIT			23
#define MVPP2_PME_LAST_MASK			BIT(MVPP2_PME_LAST_BIT)

#define MVPP2_PME_CMD_TYPE_OFFS			24
#define MVPP2_PME_CMD_TYPE_BITS			3
#define MVPP2_PME_CMD_TYPE_ALL_MASK		(((1 << MVPP2_PME_CMD_TYPE_BITS) - 1) << MVPP2_PME_CMD_TYPE_OFFS)
#define MVPP2_PME_CMD_TYPE_MASK(type)		((type) << MVPP2_PME_CMD_TYPE_OFFS)

#define MVPP2_TOTAL_TXP_NUM			(16 + 3 - 1)

/* PME data1 and data2 fields MVPP2_PME_TBL_DATA1_REG and MVPP2_PME_TBL_DATA2_REG */
#define MVPP2_PME_TBL_DATA_BITS		16
#define MVPP2_PME_TBL_DATA_OFFS(idx)	((idx == 0) ? MVPP2_PME_TBL_DATA_BITS : 0)
#define MVPP2_PME_TBL_DATA_MASK(idx)	(((1 << MVPP2_PME_TBL_DATA_BITS) - 1) << MVPP2_PME_TBL_DATA_OFFS(idx))

/* TX packet modification constants */
#define MVPP2_PME_INSTR_SIZE	2600
#define MVPP2_PME_DATA1_SIZE   (46 * 1024 / 2) /* 46KBytes = 23K data of 2 bytes */
#define MVPP2_PME_DATA2_SIZE   (4 * 1024 / 2) /* 4KBytes = 2K data of 2 bytes */

enum mv_pp2x_pme_instr {
	MVPP2_PME_CMD_NONE = 0,
	MVPP2_PME_CMD_ADD_2B,
	MVPP2_PME_CMD_CFG_VLAN,
	MVPP2_PME_CMD_ADD_VLAN,
	MVPP2_PME_CMD_CFG_DSA_1,
	MVPP2_PME_CMD_CFG_DSA_2,
	MVPP2_PME_CMD_ADD_DSA,
	MVPP2_PME_CMD_DEL_BYTES,
	MVPP2_PME_CMD_REPLACE_2B,
	MVPP2_PME_CMD_REPLACE_LSB,
	MVPP2_PME_CMD_REPLACE_MSB,
	MVPP2_PME_CMD_REPLACE_VLAN,
	MVPP2_PME_CMD_DEC_LSB,
	MVPP2_PME_CMD_DEC_MSB,
	MVPP2_PME_CMD_ADD_CALC_LEN,
	MVPP2_PME_CMD_REPLACE_LEN,
	MVPP2_PME_CMD_IPV4_CSUM,
	MVPP2_PME_CMD_L4_CSUM,
	MVPP2_PME_CMD_SKIP,
	MVPP2_PME_CMD_JUMP,
	MVPP2_PME_CMD_JUMP_SKIP,
	MVPP2_PME_CMD_JUMP_SUB,
	MVPP2_PME_CMD_PPPOE,
	MVPP2_PME_CMD_STORE,
	MVPP2_PME_CMD_ADD_IP4_CSUM,
	MVPP2_PME_CMD_PPPOE_2,
	MVPP2_PME_CMD_REPLACE_MID,
	MVPP2_PME_CMD_ADD_MULT,
	MVPP2_PME_CMD_REPLACE_MULT,
	MVPP2_PME_CMD_REPLACE_REM_2B,
	MVPP2_PME_CMD_ADD_IP6_HDR,
	MVPP2_PME_CMD_DROP_PKT = 0x1f,
	MVPP2_TMP_CMD_LAST
};

/* PME entry structure */
struct mv_pp2x_pme_entry {
	int     index;
	u32	word;
};

/* MC */
/*-------------------------------------------------------------------------------*/
#define MVPP2_MC_INDEX_REG			(0x160)
#define MVPP2_MC_INDEX_MAX			((1 << MVPP2_CLS2_ACT_DUP_ATTR_DUPID_BITS) - 1)
/*------------------------------------------------------------------------------*/
#define MVPP2_MC_DATA1_REG			(0x164)
#define	MVPP2_MC_DATA1_DPTR			1
#define	MVPP2_MC_DATA1_IPTR			16
/*------------------------------------------------------------------------------*/
#define MVPP2_MC_DATA2_REG			(0x168)
#define MVPP2_MC_DATA2_GEM_ID			0
#define MVPP2_MC_DATA2_PRI			12
#define MVPP2_MC_DATA2_DSCP			15
#define MVPP2_MC_DATA2_GEM_ID_EN		BIT(21)
#define MVPP2_MC_DATA2_PRI_EN			BIT(22)
#define MVPP2_MC_DATA2_DSCP_EN			BIT(23)
/*------------------------------------------------------------------------------*/
#define MVPP2_MC_DATA3_REG			(0x16C)
#define MVPP2_MC_DATA3_QUEUE			0
#define MVPP2_MC_DATA3_HWF_EN			BIT(8)
#define MVPP2_MC_DATA3_NEXT			16
#define MVPP2_MC_DATA3_NEXT_MASK		(MVPP2_MC_INDEX_MAX << MVPP2_MC_DATA3_NEXT)

#define MVPP2_MC_TBL_SIZE			256
#define MVPP2_MC_WORDS				3

/* MC entry structure */
struct mv_pp2x_mc_entry {
	u32 index;
	union {
		u32 words[MVPP2_MC_WORDS];
		struct {
			u32 data1;/* 0x164 */
			u32 data2;/* 0x168 */
			u32 data3;/* 0x16c */
		} regs;
	} sram;
};


#define MSS_CP_FC_COM_REG		0
#define FLOW_CONTROL_ENABLE_BIT		BIT(0)
#define FLOW_CONTROL_UPD_COM_BIT	BIT(31)
#define FLOW_CONTROL_QUNTA		0xFFFF
#define MSS_CP_CM3_BUF_POOL_BASE	0x40
#define MSS_CP_CM3_BUF_POOL_OFFS	4

#define MSS_CP_CM3_BUF_POOL_STOP_MASK	0xFFF
#define MSS_CP_CM3_BUF_POOL_START_MASK	(0xFFF << MSS_CP_CM3_BUF_POOL_START_OFFS)
#define MSS_CP_CM3_BUF_POOL_START_OFFS	12
#define MSS_CP_CM3_BUF_POOL_PORTS_MASK	(0xF << MSS_CP_CM3_BUF_POOL_PORTS_OFFS)
#define MSS_CP_CM3_BUF_POOL_PORTS_OFFS	24

#define MSS_CP_CM3_RXQ_ASS_BASE		0x80
#define MSS_CP_CM3_RXQ_ASS_OFFS		4
#define MSS_CP_CM3_RXQ_ASS_PER_REG	4
#define MSS_CP_CM3_RXQ_ASS_PER_OFFS	8
#define MSS_CP_CM3_RXQ_ASS_PORTID_OFFS	0
#define MSS_CP_CM3_RXQ_ASS_PORTID_MASK	0x3
#define MSS_CP_CM3_RXQ_ASS_HOSTID_OFFS	2
#define MSS_CP_CM3_RXQ_ASS_HOSTID_MASK	0x3F

#define MSS_CP_CM3_RXQ_ASS_Q_BASE(queue)	(((queue) % MSS_CP_CM3_RXQ_ASS_PER_REG) \
						* MSS_CP_CM3_RXQ_ASS_PER_OFFS)
#define MSS_CP_CM3_RXQ_ASS_PQ_BASE(queue)	(((queue) / MSS_CP_CM3_RXQ_ASS_PER_REG) \
						* MSS_CP_CM3_RXQ_ASS_OFFS)
#define MSS_CP_CM3_RXQ_ASS_REG(queue)		(MSS_CP_CM3_RXQ_ASS_BASE + MSS_CP_CM3_RXQ_ASS_PQ_BASE(queue))

#define MSS_CP_CM3_THRESHOLD_STOP	768
#define MSS_CP_CM3_THRESHOLD_START	1024


#define MSS_CP_CM3_RXQ_TR_BASE		0x200
#define MSS_CP_CM3_RXQ_TR_OFFS		4
#define MSS_CP_CM3_RXQ_TRESH_REG(queue)	(MSS_CP_CM3_RXQ_TR_BASE + ((queue) * MSS_CP_CM3_RXQ_TR_OFFS))

#define MSS_CP_CM3_RXQ_TR_START_MASK	0xFFFF
#define MSS_CP_CM3_RXQ_TR_STOP_MASK	(0xFFFF << MSS_CP_CM3_RXQ_TR_STOP_OFFS)
#define MSS_CP_CM3_RXQ_TR_STOP_OFFS	16
#define MSS_CP_CM3_RXQ_TR_START_OFFS	0


/* RX FIFO tresholld in 1KB granularity */
#define MVPP23_PORT0_FIFO_THRESHOLD	(9 * 1024)
#define MVPP23_PORT1_FIFO_THRESHOLD	(4 * 1024)
#define MVPP23_PORT2_FIFO_THRESHOLD	(2 * 1024)

#define MVPP21_DESC_ADDR_SHIFT		0 /*Applies to RXQ, AGGR_TXQ*/
#define MVPP22_DESC_ADDR_SHIFT		(9 - 1) /*Applies to RXQ, AGGR_TXQ*/

/* RX Flow Control Registers */
#define MVPP2_RX_FLOW_CONTROL_REG(port)	(0x150 + 4 * (port))
#define MVPP2_RX_FLOW_CONTROL_EN		BIT(24)
#define MVPP2_RX_FLOW_CONTROL_TRSH_OFFS	16
#define MVPP2_RX_FLOW_CONTROL_TRSH_MASK	(0xFF << MVPP2_RX_FLOW_CONTROL_TRSH_OFFS)
#define MVPP2_RX_FLOW_CONTROL_TRSH_UNIT	256

/* RX Fifo Registers */
#define MVPP2_RX_DATA_FIFO_SIZE_REG(port)	(0x00 + 4 * (port))
#define MVPP2_RX_ATTR_FIFO_SIZE_REG(port)	(0x20 + 4 * (port))
#define MVPP2_RX_MIN_PKT_SIZE_REG		0x60
#define MVPP2_RX_FIFO_INIT_REG			0x64


#define MVPP2_VER_ID_REG			0x50b0

#define MVPP2_VER_PP22				0x10
#define MVPP2_VER_PP23				0x11



#endif /*_MVPP2_HW_TYPE_H_*/
