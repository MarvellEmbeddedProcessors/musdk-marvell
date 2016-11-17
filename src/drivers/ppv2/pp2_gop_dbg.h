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
	pp2_info("  %-32s: 0x%x = 0x%08x\n", reg_name, reg,
		pp2_gop_gmac_read(gop, mac_num, reg));
}

/* SMI Functions */
static inline void pp2_gop_smi_print(struct gop_hw *gop, const char *reg_name,
				    uint32_t reg)
{
	pp2_info("  %-32s: 0x%x = 0x%08x\n", reg_name, reg,
		pp2_gop_smi_read(gop, reg));
}

/* RFU1 Functions */
static inline void pp2_gop_rfu1_print(struct gop_hw *gop, const char *reg_name,
				     uint32_t reg)
{
	pp2_info("  %-32s: 0x%x = 0x%08x\n", reg_name, reg,
		pp2_gop_rfu1_read(gop, reg));
}

/* PTP Functions */
static inline void pp2_gop_ptp_print(struct gop_hw *gop, const char *reg_name,
				    int mac_num, uint32_t reg)
{
	pp2_info("  %-32s: 0x%x = 0x%08x\n", reg_name, reg,
		pp2_gop_ptp_read(gop, mac_num, reg));
}

/* Serdes Functions */
static inline void pp2_gop_serdes_print(struct gop_hw *gop, const char *reg_name,
				       int lane_num, uint32_t reg)
{
	pp2_info("  %-32s: 0x%x = 0x%08x\n", reg_name, reg,
		pp2_gop_serdes_read(gop, lane_num, reg));
}

void pp2_gop_serdes_lane_regs_dump(struct gop_hw *gop, int lane);

/* MPCS Functions */
static inline void pp2_gop_mpcs_global_print(struct gop_hw *gop,
					    char *reg_name, uint32_t reg)
{
	pp2_info("  %-32s: 0x%x = 0x%08x\n", reg_name, reg,
		pp2_gop_mpcs_global_read(gop, reg));
}

/* XPCS Functions */
static inline void pp2_gop_xpcs_global_print(struct gop_hw *gop,
					   const char *reg_name, uint32_t reg)
{
	pp2_info("  %-32s: 0x%x = 0x%08x\n", reg_name, reg,
		pp2_gop_xpcs_global_read(gop, reg));
}

static inline void pp2_gop_xpcs_lane_print(struct gop_hw *gop,
					  const char *reg_name,
					  int lane_num, uint32_t reg)
{
	pp2_info("  %-32s: 0x%x = 0x%08x\n", reg_name, reg,
		pp2_gop_xpcs_lane_read(gop, lane_num, reg));
}

void pp2_gop_xpcs_gl_regs_dump(struct gop_hw *gop);

void pp2_gop_xpcs_lane_regs_dump(struct gop_hw *gop, int lane);

/* XLG MAC Functions */
static inline void pp2_gop_xlg_mac_print(struct gop_hw *gop, const char *reg_name,
					int mac_num, uint32_t reg)
{
	pp2_info("  %-32s: 0x%x = 0x%08x\n", reg_name, reg,
		pp2_gop_xlg_mac_read(gop, mac_num, reg));
}

void pp2_gop_xlg_mac_regs_dump(struct gop_hw *gop, int port);

/* MIB MAC Functions */
static inline void pp2_gop_xmib_mac_print(struct gop_hw *gop, const char *reg_name,
					 int mac_num, uint32_t reg)
{
	pp2_info("  %-32s: 0x%x = 0x%08x\n", reg_name, reg,
		pp2_gop_xmib_mac_read(gop, mac_num, reg));
}

#endif /* _PP2_GOP_DBG_H_ */
