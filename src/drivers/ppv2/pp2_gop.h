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
 * @file pp2_gop.h
 *
 * Packet Processor Group Of Ports
 * Registers access routines and GOP data structures
 *
 */

#ifndef _PP2_GOP_H_
#define _PP2_GOP_H_

/* TBD: unify all debug functions
 *      or move this under a separate header file
 */

#include "pp2_types.h"

#include "pp2_print.h"

#if (PP2_DBG <= PP2_INFO_LEVEL)
#define PP2_GOP_DEBUG 1
#define GOP_DEBUG(x) x
#else
#define GOP_DEBUG(x)
#endif

#define PP2_GOP_MAC_NUM		(4)

/* Masks used for pp3_emac flags */
#define MV_EMAC_F_LINK_UP_BIT	(0)
#define MV_EMAC_F_INIT_BIT	(1)
#define MV_EMAC_F_SGMII2_5_BIT	(2)

#define MV_EMAC_F_LINK_UP	BIT(MV_EMAC_F_LINK_UP_BIT)
#define MV_EMAC_F_INIT		BIT(MV_EMAC_F_INIT_BIT)
#define MV_EMAC_F_SGMII2_5	BIT(MV_EMAC_F_SGMII2_5_BIT)

/* Interface Mode definitions */
enum phy_interface {
	PHY_INTERFACE_MODE_NA,
	PHY_INTERFACE_MODE_MII,
	PHY_INTERFACE_MODE_GMII,
	PHY_INTERFACE_MODE_SGMII,
	PHY_INTERFACE_MODE_TBI,
	PHY_INTERFACE_MODE_REVMII,
	PHY_INTERFACE_MODE_RMII,
	PHY_INTERFACE_MODE_RGMII,
	PHY_INTERFACE_MODE_RGMII_ID,
	PHY_INTERFACE_MODE_RGMII_RXID,
	PHY_INTERFACE_MODE_RGMII_TXID,
	PHY_INTERFACE_MODE_RTBI,
	PHY_INTERFACE_MODE_SMII,
	PHY_INTERFACE_MODE_XGMII,
	PHY_INTERFACE_MODE_MOCA,
	PHY_INTERFACE_MODE_QSGMII,
	PHY_INTERFACE_MODE_XAUI,
	PHY_INTERFACE_MODE_RXAUI,
	PHY_INTERFACE_MODE_KR,
	PHY_INTERFACE_MODE_MAX,
};

#define ETH_ALEN 6

struct pp2_mac_data {
	uint8_t            gop_index;
	unsigned long      flags;
	/* Whether a PHY is present, and if yes, at which address. */
	int                phy_addr;
	enum phy_interface phy_mode; /* RXAUI, SGMII, etc. */
	bool               force_link;
	unsigned int       autoneg;
	unsigned int       link;
	unsigned int       duplex;
	unsigned int       speed;
	uint8_t            mac[ETH_ALEN];
};

struct pp2_mac_unit_desc {
	struct base_addr base;
	unsigned int obj_size;
};

struct gop_port_ctrl {
	uint32_t flags;
};

struct gop_hw {
	struct pp2_mac_unit_desc gmac;
	struct pp2_mac_unit_desc xlg_mac;
	struct pp2_mac_unit_desc serdes;
	struct pp2_mac_unit_desc xmib;
	struct pp2_mac_unit_desc ptp;
	struct base_addr smi;
	struct base_addr xsmi;
	struct base_addr mspg;
	struct base_addr xpcs;
	struct base_addr rfu1;
	struct gop_port_ctrl gop_port_debug[PP2_GOP_MAC_NUM];
};

/* Sets the field located at the specified in data. */
#define U32_SET_FIELD(data, mask, val)	((data) = (((data) & ~(mask)) | (val)))

/** Port related */
enum pp2_reset { RESET, UNRESET, };

enum pp2_port_speed {
	PP2_PORT_SPEED_AN,
	PP2_PORT_SPEED_10,
	PP2_PORT_SPEED_100,
	PP2_PORT_SPEED_1000,
	PP2_PORT_SPEED_2500,
	PP2_PORT_SPEED_10000,
};

enum pp2_port_duplex {
	PP2_PORT_DUPLEX_AN,
	PP2_PORT_DUPLEX_HALF,
	PP2_PORT_DUPLEX_FULL
};

enum pp2_port_fc {
	PP2_PORT_FC_AN_NO,
	PP2_PORT_FC_AN_SYM,
	PP2_PORT_FC_AN_ASYM,
	PP2_PORT_FC_DISABLE,
	PP2_PORT_FC_ENABLE,
	PP2_PORT_FC_ACTIVE,
};

struct pp2_port_link_status {
	int linkup;		/*flag */
	enum pp2_port_speed speed;
	enum pp2_port_duplex duplex;
	enum pp2_port_fc rx_fc;
	enum pp2_port_fc tx_fc;
};

/* different loopback types can be configure on different levels:
 * MAC, PCS, SERDES
 */
enum pp2_lb_type {
	PP2_DISABLE_LB,
	PP2_RX_2_TX_LB,
	PP2_TX_2_RX_LB,		/* on SERDES level - analog loopback */
	PP2_TX_2_RX_DIGITAL_LB,	/* on SERDES level - digital loopback */
};

enum sd_media_mode { PP2_RXAUI, PP2_XAUI, };

/** Net Complex */
enum pp2_netc_topology {
	PP2_NETC_GE_MAC0_RXAUI_L23 = BIT(0),
	PP2_NETC_GE_MAC0_RXAUI_L45 = BIT(1),
	PP2_NETC_GE_MAC0_XAUI = BIT(2),
	PP2_NETC_GE_MAC2_SGMII = BIT(3),
	PP2_NETC_GE_MAC3_SGMII = BIT(4),
	PP2_NETC_GE_MAC3_RGMII = BIT(5),
};

enum pp2_netc_phase {
	PP2_NETC_FIRST_PHASE,
	PP2_NETC_SECOND_PHASE,
};

enum pp2_netc_sgmii_xmi_mode {
	PP2_NETC_GBE_SGMII,
	PP2_NETC_GBE_XMII,
};

enum pp2_netc_mii_mode {
	PP2_NETC_GBE_RGMII,
	PP2_NETC_GBE_MII,
};

enum pp2_netc_lanes {
	PP2_NETC_LANE_23,
	PP2_NETC_LANE_45,
};

enum gop_port_flags { NOT_CREATED, CREATED, UNDER_RESET, ENABLED };

#define PP2_RGMII_TX_FIFO_MIN_TH		(0x41)
#define PP2_SGMII_TX_FIFO_MIN_TH		(0x5)
#define PP2_SGMII2_5_TX_FIFO_MIN_TH	(0xB)

static inline uint32_t pp2_gop_gen_read(uintptr_t base, uint32_t offset)
{
	uintptr_t reg_ptr = base + offset;
	uint32_t val;

	val = readl((void *)reg_ptr);
	return val;
}

static inline void pp2_gop_gen_write(uintptr_t base, uint32_t offset,
				    uint32_t data)
{
	uintptr_t reg_ptr = base + offset;

	writel(data, (void *)reg_ptr);
}

/** GOP port configuration functions */
int pp2_gop_port_init(struct gop_hw *gop, struct pp2_mac_data *mac, uint32_t lb);
int pp2_gop_port_reset(struct gop_hw *gop, struct pp2_mac_data *mac);
void pp2_gop_port_enable(struct gop_hw *gop, struct pp2_mac_data *mac);
void pp2_gop_port_disable(struct gop_hw *gop, struct pp2_mac_data *mac);
void pp2_gop_port_periodic_xon_set(struct gop_hw *gop,
				  struct pp2_mac_data *mac, int enable);
bool pp2_gop_port_is_link_up(struct gop_hw *gop, struct pp2_mac_data *mac);
int pp2_gop_port_link_status(struct gop_hw *gop, struct pp2_mac_data *mac,
			    struct pp2_port_link_status *pstatus);
int pp2_gop_port_regs(struct gop_hw *gop, struct pp2_mac_data *mac);
int pp2_gop_port_events_mask(struct gop_hw *gop, struct pp2_mac_data *mac);
int pp2_gop_port_events_unmask(struct gop_hw *gop, struct pp2_mac_data *mac);
int pp2_gop_port_events_clear(struct gop_hw *gop, struct pp2_mac_data *mac);
int pp2_gop_status_show(struct gop_hw *gop, struct pp2_mac_data *mac);
int pp2_gop_speed_duplex_get(struct gop_hw *gop, struct pp2_mac_data *mac,
			    enum pp2_port_speed *speed,
			    enum pp2_port_duplex *duplex);
int pp2_gop_speed_duplex_set(struct gop_hw *gop, struct pp2_mac_data *mac,
			    enum pp2_port_speed speed,
			    enum pp2_port_duplex duplex);
int pp2_gop_autoneg_restart(struct gop_hw *gop, struct pp2_mac_data *mac);
int pp2_gop_fl_cfg(struct gop_hw *gop, struct pp2_mac_data *mac);
int pp2_gop_force_link_mode_set(struct gop_hw *gop, struct pp2_mac_data *mac,
			       bool force_link_up, bool force_link_down);
int pp2_gop_force_link_mode_get(struct gop_hw *gop, struct pp2_mac_data *mac,
			       bool *force_link_up, bool *force_link_down);
int pp2_gop_loopback_set(struct gop_hw *gop, struct pp2_mac_data *mac, bool lb);

/** Gig PCS Functions */
int pp2_gop_gpcs_mode_cfg(struct gop_hw *gop, int pcs_num, bool en);
int pp2_gop_gpcs_reset(struct gop_hw *gop, int pcs_num, enum pp2_reset act);

/** SERDES Functions */
static inline uint32_t pp2_gop_serdes_read(struct gop_hw *gop, int lane_num,
					  uint32_t offset)
{
	return (pp2_gop_gen_read(gop->serdes.base.va,
				lane_num * gop->serdes.obj_size + offset));
}

static inline void pp2_gop_serdes_write(struct gop_hw *gop, int lane_num,
				       uint32_t offset, uint32_t data)
{
	pp2_gop_gen_write(gop->serdes.base.va,
			 lane_num * gop->serdes.obj_size + offset, data);
}

void pp2_gop_serdes_init(struct gop_hw *gop, int lane, enum sd_media_mode mode);
void pp2_gop_serdes_reset(struct gop_hw *gop, int lane, bool analog_reset,
			 bool core_reset, bool digital_reset);

/** MPCS Functions */
static inline uint32_t pp2_gop_mpcs_global_read(struct gop_hw *gop,
					       uint32_t offset)
{
	return pp2_gop_gen_read(gop->mspg.va, offset);
}

static inline void pp2_gop_mpcs_global_write(struct gop_hw *gop, uint32_t offset,
					    uint32_t data)
{
	pp2_gop_gen_write(gop->mspg.va, offset, data);
}

/** XPCS Functions */
static inline uint32_t pp2_gop_xpcs_global_read(struct gop_hw *gop,
					       uint32_t offset)
{
	return pp2_gop_gen_read(gop->xpcs.va, offset);
}

static inline void pp2_gop_xpcs_global_write(struct gop_hw *gop, uint32_t offset,
					    uint32_t data)
{
	pp2_gop_gen_write(gop->xpcs.va, offset, data);
}

static inline uint32_t pp2_gop_xpcs_lane_read(struct gop_hw *gop, int lane_num,
					     uint32_t offset)
{
	(void)lane_num;

	return pp2_gop_gen_read(gop->xpcs.va, offset);
}

static inline void pp2_gop_xpcs_lane_write(struct gop_hw *gop, int lane_num,
					  uint32_t offset, uint32_t data)
{
	(void)lane_num;

	pp2_gop_gen_write(gop->xpcs.va, offset, data);
}

int pp2_gop_xpcs_reset(struct gop_hw *gop, enum pp2_reset reset);
int pp2_gop_xpcs_mode(struct gop_hw *gop, int num_of_lanes);
int pp2_gop_mpcs_mode(struct gop_hw *gop);

/** XLG MAC Functions */
static inline uint32_t pp2_gop_xlg_mac_read(struct gop_hw *gop, int mac_num,
					   uint32_t offset)
{
	return (pp2_gop_gen_read(gop->xlg_mac.base.va,
				mac_num * gop->xlg_mac.obj_size + offset));
}

static inline void pp2_gop_xlg_mac_write(struct gop_hw *gop, int mac_num,
					uint32_t offset, uint32_t data)
{
	pp2_gop_gen_write(gop->xlg_mac.base.va,
			 mac_num * gop->xlg_mac.obj_size + offset, data);
}

/** MIB MAC Functions */
static inline uint32_t pp2_gop_xmib_mac_read(struct gop_hw *gop, int mac_num,
					    uint32_t offset)
{
	return (pp2_gop_gen_read(gop->xmib.base.va,
				mac_num * gop->xmib.obj_size + offset));
}

static inline void pp2_gop_xmib_mac_write(struct gop_hw *gop, int mac_num,
					 uint32_t offset, uint32_t data)
{
	pp2_gop_gen_write(gop->xmib.base.va,
			 mac_num * gop->xmib.obj_size + offset, data);
}

int pp2_gop_xlg_mac_reset(struct gop_hw *gop, int mac_num, enum pp2_reset reset);
int pp2_gop_xlg_mac_mode_cfg(struct gop_hw *gop, int mac_num,
			    int num_of_act_lanes);
int pp2_gop_xlg_mac_loopback_cfg(struct gop_hw *gop, int mac_num,
				enum pp2_lb_type type);

bool pp2_gop_xlg_mac_link_status_get(struct gop_hw *gop, int mac_num);
void pp2_gop_xlg_mac_port_enable(struct gop_hw *gop, int mac_num);
void pp2_gop_xlg_mac_port_disable(struct gop_hw *gop, int mac_num);
void pp2_gop_xlg_mac_port_periodic_xon_set(struct gop_hw *gop,
					  int mac_num, int enable);
int pp2_gop_xlg_mac_link_status(struct gop_hw *gop, int mac_num,
			       struct pp2_port_link_status *pstatus);
int pp2_gop_xlg_mac_max_rx_size_set(struct gop_hw *gop, int mac_num,
				   int max_rx_size);
int pp2_gop_xlg_mac_force_link_mode_set(struct gop_hw *gop, int mac_num,
				       bool force_link_up,
				       bool force_link_down);
int pp2_gop_xlg_mac_speed_duplex_set(struct gop_hw *gop, int mac_num,
				    enum pp2_port_speed speed,
				    enum pp2_port_duplex duplex);
int pp2_gop_xlg_mac_speed_duplex_get(struct gop_hw *gop, int mac_num,
				    enum pp2_port_speed *speed,
				    enum pp2_port_duplex *duplex);
int pp2_gop_xlg_mac_fc_set(struct gop_hw *gop, int mac_num, enum pp2_port_fc fc);
void pp2_gop_xlg_mac_fc_get(struct gop_hw *gop, int mac_num,
			   enum pp2_port_fc *fc);
int pp2_gop_xlg_mac_port_link_speed_fc(struct gop_hw *gop, int mac_num,
				      enum pp2_port_speed speed,
				      int force_link_up);
void pp2_gop_xlg_port_link_event_mask(struct gop_hw *gop, int mac_num);
void pp2_gop_xlg_port_external_event_unmask(struct gop_hw *gop,
					   int mac_num, int bit_2_open);
void pp2_gop_xlg_port_link_event_clear(struct gop_hw *gop, int mac_num);
void pp2_gop_xlg_2_gig_mac_cfg(struct gop_hw *gop, int mac_num);

/** GMAC Functions */
static inline uint32_t pp2_gop_gmac_read(struct gop_hw *gop, int mac_num,
					uint32_t offset)
{
	return (pp2_gop_gen_read(gop->gmac.base.va,
				mac_num * gop->gmac.obj_size + offset));
}

static inline void pp2_gop_gmac_write(struct gop_hw *gop, int mac_num,
				     uint32_t offset, uint32_t data)
{
	pp2_gop_gen_write(gop->gmac.base.va,
			 mac_num * gop->gmac.obj_size + offset, data);
}

int pp2_gop_gmac_reset(struct gop_hw *gop, int mac_num, enum pp2_reset reset);
int pp2_gop_gmac_mode_cfg(struct gop_hw *gop, struct pp2_mac_data *mac);
int pp2_gop_gmac_loopback_cfg(struct gop_hw *gop, int mac_num,
			     enum pp2_lb_type type);
bool pp2_gop_gmac_link_status_get(struct gop_hw *gop, int mac_num);
void pp2_gop_gmac_port_enable(struct gop_hw *gop, int mac_num);
void pp2_gop_gmac_port_disable(struct gop_hw *gop, int mac_num);
void pp2_gop_gmac_port_periodic_xon_set(struct gop_hw *gop, int mac_num,
				       int enable);
int pp2_gop_gmac_link_status(struct gop_hw *gop, int mac_num,
			    struct pp2_port_link_status *pstatus);
int pp2_gop_gmac_max_rx_size_set(struct gop_hw *gop, int mac_num,
				int max_rx_size);
int pp2_gop_gmac_force_link_mode_set(struct gop_hw *gop, int mac_num,
				    bool force_link_up, bool force_link_down);
int pp2_gop_gmac_force_link_mode_get(struct gop_hw *gop, int mac_num,
				    bool *force_link_up, bool *force_link_down);
int pp2_gop_gmac_speed_duplex_set(struct gop_hw *gop, int mac_num,
				 enum pp2_port_speed speed,
				 enum pp2_port_duplex duplex);
int pp2_gop_gmac_speed_duplex_get(struct gop_hw *gop, int mac_num,
				 enum pp2_port_speed *speed,
				 enum pp2_port_duplex *duplex);
int pp2_gop_gmac_fc_set(struct gop_hw *gop, int mac_num, enum pp2_port_fc fc);
void pp2_gop_gmac_fc_get(struct gop_hw *gop, int mac_num, enum pp2_port_fc *fc);
int pp2_gop_gmac_port_link_speed_fc(struct gop_hw *gop, int mac_num,
				   enum pp2_port_speed speed, int force_link_up);
void pp2_gop_gmac_port_link_event_mask(struct gop_hw *gop, int mac_num);
void pp2_gop_gmac_port_link_event_unmask(struct gop_hw *gop, int mac_num);
void pp2_gop_gmac_port_link_event_clear(struct gop_hw *gop, int mac_num);
int pp2_gop_gmac_port_autoneg_restart(struct gop_hw *gop, int mac_num);

/** SMI Functions */
static inline uint32_t pp2_gop_smi_read(struct gop_hw *gop, uint32_t offset)
{
	return pp2_gop_gen_read(gop->smi.va, offset);
}

static inline void pp2_gop_smi_write(struct gop_hw *gop, uint32_t offset,
				    uint32_t data)
{
	pp2_gop_gen_write(gop->smi.va, offset, data);
}

/** RFU1 Functions */
static inline uint32_t pp2_gop_rfu1_read(struct gop_hw *gop, uint32_t offset)
{
	return pp2_gop_gen_read(gop->rfu1.va, offset);
}

static inline void pp2_gop_rfu1_write(struct gop_hw *gop, uint32_t offset,
				     uint32_t data)
{
	pp2_gop_gen_write(gop->rfu1.va, offset, data);
}

/** PTP Functions */
static inline uint32_t pp2_gop_ptp_read(struct gop_hw *gop, int mac_num,
				       uint32_t offset)
{
	return pp2_gop_gen_read(gop->ptp.base.va,
			       mac_num * gop->ptp.obj_size + offset);
}

static inline void pp2_gop_ptp_write(struct gop_hw *gop, int mac_num,
				    uint32_t offset, uint32_t data)
{
	pp2_gop_gen_write(gop->ptp.base.va,
			 mac_num * gop->ptp.obj_size + offset, data);
}

void pp2_gop_ptp_enable(struct gop_hw *gop, int port, bool state);

int pp2_gop_smi_init(struct gop_hw *gop);
int pp2_gop_smi_phy_addr_cfg(struct gop_hw *gop, int port, int addr);

/** RFU Functions */
int pp2_gop_netc_init(struct gop_hw *gop,
		     uint32_t net_comp_config, enum pp2_netc_phase phase);
void pp2_gop_netc_active_port(struct gop_hw *gop, uint32_t port, uint32_t val);
uint32_t pp2_gop_netc_cfg_create(struct pp2_mac_data *mac);

#endif /*_PP2_GOP_H_*/
