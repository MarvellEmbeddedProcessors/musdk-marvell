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

#include "std_internal.h"
#include "pp2_types.h"


#ifdef DEBUG
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
enum pp2_phy_interface {
	PP2_PHY_INTERFACE_MODE_NA,
	PP2_PHY_INTERFACE_MODE_MII,
	PP2_PHY_INTERFACE_MODE_GMII,
	PP2_PHY_INTERFACE_MODE_SGMII,
	PP2_PHY_INTERFACE_MODE_TBI,
	PP2_PHY_INTERFACE_MODE_REVMII,
	PP2_PHY_INTERFACE_MODE_RMII,
	PP2_PHY_INTERFACE_MODE_RGMII,
	PP2_PHY_INTERFACE_MODE_RGMII_ID,
	PP2_PHY_INTERFACE_MODE_RGMII_RXID,
	PP2_PHY_INTERFACE_MODE_RGMII_TXID,
	PP2_PHY_INTERFACE_MODE_RTBI,
	PP2_PHY_INTERFACE_MODE_SMII,
	PP2_PHY_INTERFACE_MODE_XGMII,
	PP2_PHY_INTERFACE_MODE_MOCA,
	PP2_PHY_INTERFACE_MODE_QSGMII,
	PP2_PHY_INTERFACE_MODE_XAUI,
	PP2_PHY_INTERFACE_MODE_RXAUI,
	PP2_PHY_INTERFACE_MODE_KR,
	PP2_PHY_INTERFACE_MODE_MAX,
};

#define ETH_ALEN 6

struct pp2_mac_data {
	u8            gop_index;
	unsigned long      flags;
	/* Whether a PHY is present, and if yes, at which address. */
	int                phy_addr;
	enum pp2_phy_interface phy_mode; /* RXAUI, SGMII, etc. */
	bool               force_link;
	unsigned int       autoneg;
	unsigned int       link;
	unsigned int       duplex;
	unsigned int       speed;
	u8            mac[ETH_ALEN];
};

struct pp2_mac_unit_desc {
	struct base_addr base;
	unsigned int obj_size;
};

struct gop_port_ctrl {
	u32 flags;
};

struct gop_hw {
	struct pp2_mac_unit_desc gmac;
	struct pp2_mac_unit_desc xlg_mac;
	struct base_addr mspg;
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
	enum pp2_phy_interface phy_mode;
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


enum gop_port_flags { NOT_CREATED, CREATED, UNDER_RESET, ENABLED };

#define PP2_RGMII_TX_FIFO_MIN_TH		(0x41)
#define PP2_SGMII_TX_FIFO_MIN_TH		(0x5)
#define PP2_SGMII2_5_TX_FIFO_MIN_TH	(0xB)

static inline uint32_t pp2_gop_gen_read(uintptr_t base, uint32_t offset)
{
	uintptr_t reg_ptr = base + offset;
	u32 val;

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




/** XLG MAC Functions */
static inline uint32_t pp2_gop_xlg_mac_read(struct gop_hw *gop, int mac_num,
					    uint32_t offset)
{
	return (pp2_gop_gen_read(gop->xlg_mac.base.va,
				 mac_num * gop->xlg_mac.obj_size + offset));
}

static inline void pp2_gop_xlg_mac_write(struct gop_hw *gop, int mac_num,
					 u32 offset, uint32_t data)
{
	pp2_gop_gen_write(gop->xlg_mac.base.va,
			  mac_num * gop->xlg_mac.obj_size + offset, data);
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
				      u32 offset, uint32_t data)
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


#endif /*_PP2_GOP_H_*/
