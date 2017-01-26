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

/**
 * @file pp2_gop_dbg.h
 *
 * PPDK Group Of Ports registers debugging helpers
 *
 */

#ifndef _PP2_GOP_DBG_H_
#define _PP2_GOP_DBG_H_

#include "std_internal.h"
#include "pp2_types.h"

void pp2_gop_register_bases_dump(struct gop_hw *gop);
void pp2_gop_gmac_regs_dump(struct gop_hw *gop, int port);
void mv_pp2x_bm_pool_regs(struct pp2_hw *hw, int pool);
void pp2_gop_mac_print(struct pp2_mac_data *mac);

/* MIB Functions */
uint64_t pp2_gop_mib_read64(struct gop_hw *gop, int port, unsigned int offset);
void pp2_gop_mib_counters_show(struct gop_hw *gop, int port);

/* GMAC Functions */
static inline void pp2_gop_gmac_print(struct gop_hw *gop, const char *reg_name,
				      int mac_num, uint32_t reg)
{
	pr_info("  %-32s: 0x%x = 0x%08x\n", reg_name, reg,
		 pp2_gop_gmac_read(gop, mac_num, reg));
}

/* SMI Functions */
static inline void pp2_gop_smi_print(struct gop_hw *gop, const char *reg_name,
				     uint32_t reg)
{
	pr_info("  %-32s: 0x%x = 0x%08x\n", reg_name, reg,
		 pp2_gop_smi_read(gop, reg));
}

/* RFU1 Functions */
static inline void pp2_gop_rfu1_print(struct gop_hw *gop, const char *reg_name,
				      uint32_t reg)
{
	pr_info("  %-32s: 0x%x = 0x%08x\n", reg_name, reg,
		 pp2_gop_rfu1_read(gop, reg));
}

/* PTP Functions */
static inline void pp2_gop_ptp_print(struct gop_hw *gop, const char *reg_name,
				     int mac_num, uint32_t reg)
{
	pr_info("  %-32s: 0x%x = 0x%08x\n", reg_name, reg,
		 pp2_gop_ptp_read(gop, mac_num, reg));
}

/* Serdes Functions */
static inline void pp2_gop_serdes_print(struct gop_hw *gop, const char *reg_name,
					int lane_num, uint32_t reg)
{
	pr_info("  %-32s: 0x%x = 0x%08x\n", reg_name, reg,
		 pp2_gop_serdes_read(gop, lane_num, reg));
}

void pp2_gop_serdes_lane_regs_dump(struct gop_hw *gop, int lane);

/* MPCS Functions */
static inline void pp2_gop_mpcs_global_print(struct gop_hw *gop,
					     char *reg_name, uint32_t reg)
{
	pr_info("  %-32s: 0x%x = 0x%08x\n", reg_name, reg,
		 pp2_gop_mpcs_global_read(gop, reg));
}

/* XPCS Functions */
static inline void pp2_gop_xpcs_global_print(struct gop_hw *gop,
					     const char *reg_name, uint32_t reg)
{
	pr_info("  %-32s: 0x%x = 0x%08x\n", reg_name, reg,
		 pp2_gop_xpcs_global_read(gop, reg));
}

static inline void pp2_gop_xpcs_lane_print(struct gop_hw *gop,
					   const char *reg_name,
					  int lane_num, uint32_t reg)
{
	pr_info("  %-32s: 0x%x = 0x%08x\n", reg_name, reg,
		 pp2_gop_xpcs_lane_read(gop, lane_num, reg));
}

void pp2_gop_xpcs_gl_regs_dump(struct gop_hw *gop);

void pp2_gop_xpcs_lane_regs_dump(struct gop_hw *gop, int lane);

/* XLG MAC Functions */
static inline void pp2_gop_xlg_mac_print(struct gop_hw *gop, const char *reg_name,
					 int mac_num, uint32_t reg)
{
	pr_info("  %-32s: 0x%x = 0x%08x\n", reg_name, reg,
		 pp2_gop_xlg_mac_read(gop, mac_num, reg));
}

void pp2_gop_xlg_mac_regs_dump(struct gop_hw *gop, int port);

/* MIB MAC Functions */
static inline void pp2_gop_xmib_mac_print(struct gop_hw *gop, const char *reg_name,
					  int mac_num, uint32_t reg)
{
	pr_info("  %-32s: 0x%x = 0x%08x\n", reg_name, reg,
		 pp2_gop_xmib_mac_read(gop, mac_num, reg));
}

#endif /* _PP2_GOP_DBG_H_ */
