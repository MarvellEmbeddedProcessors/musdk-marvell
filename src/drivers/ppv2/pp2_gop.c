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

#include "pp2_types.h"

#include "pp2_hw_type.h"
#include "pp2.h"
#include "pp2_gop_def.h"

/* Set the MAC to reset or exit from reset */
int pp2_gop_gmac_reset(struct gop_hw *gop, int mac_num, enum pp2_reset reset)
{
	u32 reg_addr;
	u32 val;

	reg_addr = PP2_GMAC_PORT_CTRL2_REG;

	/* read - modify - write */
	val = pp2_gop_gmac_read(gop, mac_num, reg_addr);
	if (reset == RESET)
		val |= PP2_GMAC_PORT_CTRL2_PORTMACRESET_MASK;
	else
		val &= ~PP2_GMAC_PORT_CTRL2_PORTMACRESET_MASK;
	pp2_gop_gmac_write(gop, mac_num, reg_addr, val);

	return 0;
}

static void pp2_gop_gmac_rgmii_cfg(struct gop_hw *gop, int mac_num)
{
	u32 val, thresh, an;

	/* configure minimal level of the Tx FIFO before the lower
	 * part starts to read a packet
	 */
	thresh = PP2_RGMII_TX_FIFO_MIN_TH;
	val = pp2_gop_gmac_read(gop, mac_num, PP2_GMAC_PORT_FIFO_CFG_1_REG);
	U32_SET_FIELD(val, PP2_GMAC_PORT_FIFO_CFG_1_TX_FIFO_MIN_TH_MASK,
		      (thresh << PP2_GMAC_PORT_FIFO_CFG_1_TX_FIFO_MIN_TH_OFFS));
	pp2_gop_gmac_write(gop, mac_num, PP2_GMAC_PORT_FIFO_CFG_1_REG, val);

	/* Disable bypass of sync module */
	val = pp2_gop_gmac_read(gop, mac_num, PP2_GMAC_PORT_CTRL4_REG);
	val |= PP2_GMAC_PORT_CTRL4_SYNC_BYPASS_MASK;
	/* configure DP clock select according to mode */
	val &= ~PP2_GMAC_PORT_CTRL4_DP_CLK_SEL_MASK;
	val |= PP2_GMAC_PORT_CTRL4_QSGMII_BYPASS_ACTIVE_MASK;
	val |= PP2_GMAC_PORT_CTRL4_EXT_PIN_GMII_SEL_MASK;
	pp2_gop_gmac_write(gop, mac_num, PP2_GMAC_PORT_CTRL4_REG, val);

	val = pp2_gop_gmac_read(gop, mac_num, PP2_GMAC_PORT_CTRL2_REG);
	val |= PP2_GMAC_PORT_CTRL2_CLK_125_BYPS_EN_MASK;
	val &= ~PP2_GMAC_PORT_CTRL2_DIS_PADING_OFFS;
	pp2_gop_gmac_write(gop, mac_num, PP2_GMAC_PORT_CTRL2_REG, val);

	val = pp2_gop_gmac_read(gop, mac_num, PP2_GMAC_PORT_CTRL0_REG);
	/* configure GIG MAC to SGMII mode */
	val &= ~PP2_GMAC_PORT_CTRL0_PORTTYPE_MASK;
	pp2_gop_gmac_write(gop, mac_num, PP2_GMAC_PORT_CTRL0_REG, val);

	/* configure AN 0xb8e8 */
	an = PP2_GMAC_PORT_AUTO_NEG_CFG_AN_BYPASS_EN_MASK |
	    PP2_GMAC_PORT_AUTO_NEG_CFG_EN_AN_SPEED_MASK |
	    PP2_GMAC_PORT_AUTO_NEG_CFG_EN_FC_AN_MASK |
	    PP2_GMAC_PORT_AUTO_NEG_CFG_EN_FDX_AN_MASK |
	    PP2_GMAC_PORT_AUTO_NEG_CFG_CHOOSE_SAMPLE_TX_CONFIG_MASK;
	pp2_gop_gmac_write(gop, mac_num, PP2_GMAC_PORT_AUTO_NEG_CFG_REG, an);
}

static void pp2_gop_gmac_qsgmii_cfg(struct gop_hw *gop, int mac_num)
{
	u32 val, thresh, an;

	/* configure minimal level of the Tx FIFO before the lower
	 * part starts to read a packet
	 */
	thresh = PP2_SGMII_TX_FIFO_MIN_TH;
	val = pp2_gop_gmac_read(gop, mac_num, PP2_GMAC_PORT_FIFO_CFG_1_REG);
	U32_SET_FIELD(val, PP2_GMAC_PORT_FIFO_CFG_1_TX_FIFO_MIN_TH_MASK,
		      (thresh << PP2_GMAC_PORT_FIFO_CFG_1_TX_FIFO_MIN_TH_OFFS));
	pp2_gop_gmac_write(gop, mac_num, PP2_GMAC_PORT_FIFO_CFG_1_REG, val);

	/* Disable bypass of sync module */
	val = pp2_gop_gmac_read(gop, mac_num, PP2_GMAC_PORT_CTRL4_REG);
	val |= PP2_GMAC_PORT_CTRL4_SYNC_BYPASS_MASK;
	/* configure DP clock select according to mode */
	val &= ~PP2_GMAC_PORT_CTRL4_DP_CLK_SEL_MASK;
	val &= ~PP2_GMAC_PORT_CTRL4_EXT_PIN_GMII_SEL_MASK;
	/* configure QSGMII bypass according to mode */
	val &= ~PP2_GMAC_PORT_CTRL4_QSGMII_BYPASS_ACTIVE_MASK;
	pp2_gop_gmac_write(gop, mac_num, PP2_GMAC_PORT_CTRL4_REG, val);

	val = pp2_gop_gmac_read(gop, mac_num, PP2_GMAC_PORT_CTRL2_REG);
	val &= ~PP2_GMAC_PORT_CTRL2_DIS_PADING_OFFS;
	pp2_gop_gmac_write(gop, mac_num, PP2_GMAC_PORT_CTRL2_REG, val);

	val = pp2_gop_gmac_read(gop, mac_num, PP2_GMAC_PORT_CTRL0_REG);
	/* configure GIG MAC to SGMII mode */
	val &= ~PP2_GMAC_PORT_CTRL0_PORTTYPE_MASK;
	pp2_gop_gmac_write(gop, mac_num, PP2_GMAC_PORT_CTRL0_REG, val);

	/* configure AN 0xB8EC */
	an = PP2_GMAC_PORT_AUTO_NEG_CFG_EN_PCS_AN_MASK |
	    PP2_GMAC_PORT_AUTO_NEG_CFG_AN_BYPASS_EN_MASK |
	    PP2_GMAC_PORT_AUTO_NEG_CFG_EN_AN_SPEED_MASK |
	    PP2_GMAC_PORT_AUTO_NEG_CFG_EN_FC_AN_MASK |
	    PP2_GMAC_PORT_AUTO_NEG_CFG_EN_FDX_AN_MASK |
	    PP2_GMAC_PORT_AUTO_NEG_CFG_CHOOSE_SAMPLE_TX_CONFIG_MASK;
	pp2_gop_gmac_write(gop, mac_num, PP2_GMAC_PORT_AUTO_NEG_CFG_REG, an);
}

static void pp2_gop_gmac_sgmii_cfg(struct gop_hw *gop, int mac_num)
{
	u32 val, thresh, an;

	/* configure minimal level of the Tx FIFO before the lower
	 * part starts to read a packet
	 */
	thresh = PP2_SGMII_TX_FIFO_MIN_TH;
	val = pp2_gop_gmac_read(gop, mac_num, PP2_GMAC_PORT_FIFO_CFG_1_REG);
	U32_SET_FIELD(val, PP2_GMAC_PORT_FIFO_CFG_1_TX_FIFO_MIN_TH_MASK,
		      (thresh << PP2_GMAC_PORT_FIFO_CFG_1_TX_FIFO_MIN_TH_OFFS));
	pp2_gop_gmac_write(gop, mac_num, PP2_GMAC_PORT_FIFO_CFG_1_REG, val);

	/* Disable bypass of sync module */
	val = pp2_gop_gmac_read(gop, mac_num, PP2_GMAC_PORT_CTRL4_REG);
	val |= PP2_GMAC_PORT_CTRL4_SYNC_BYPASS_MASK;
	/* configure DP clock select according to mode */
	val &= ~PP2_GMAC_PORT_CTRL4_DP_CLK_SEL_MASK;
	/* configure QSGMII bypass according to mode */
	val |= PP2_GMAC_PORT_CTRL4_QSGMII_BYPASS_ACTIVE_MASK;
	pp2_gop_gmac_write(gop, mac_num, PP2_GMAC_PORT_CTRL4_REG, val);

	val = pp2_gop_gmac_read(gop, mac_num, PP2_GMAC_PORT_CTRL2_REG);
	val |= PP2_GMAC_PORT_CTRL2_DIS_PADING_OFFS;
	pp2_gop_gmac_write(gop, mac_num, PP2_GMAC_PORT_CTRL2_REG, val);

	val = pp2_gop_gmac_read(gop, mac_num, PP2_GMAC_PORT_CTRL0_REG);
	/* configure GIG MAC to SGMII mode */
	val &= ~PP2_GMAC_PORT_CTRL0_PORTTYPE_MASK;
	pp2_gop_gmac_write(gop, mac_num, PP2_GMAC_PORT_CTRL0_REG, val);

	/* configure AN */
	an = PP2_GMAC_PORT_AUTO_NEG_CFG_EN_PCS_AN_MASK |
	    PP2_GMAC_PORT_AUTO_NEG_CFG_AN_BYPASS_EN_MASK |
	    PP2_GMAC_PORT_AUTO_NEG_CFG_EN_AN_SPEED_MASK |
	    PP2_GMAC_PORT_AUTO_NEG_CFG_EN_FC_AN_MASK |
	    PP2_GMAC_PORT_AUTO_NEG_CFG_EN_FDX_AN_MASK |
	    PP2_GMAC_PORT_AUTO_NEG_CFG_CHOOSE_SAMPLE_TX_CONFIG_MASK;
	pp2_gop_gmac_write(gop, mac_num, PP2_GMAC_PORT_AUTO_NEG_CFG_REG, an);
}

static void pp2_gop_gmac_sgmii2_5_cfg(struct gop_hw *gop, int mac_num)
{
	u32 val, thresh, an;

	/* configure minimal level of the Tx FIFO before the lower
	 * part starts to read a packet
	 */
	thresh = PP2_SGMII2_5_TX_FIFO_MIN_TH;
	val = pp2_gop_gmac_read(gop, mac_num, PP2_GMAC_PORT_FIFO_CFG_1_REG);
	U32_SET_FIELD(val, PP2_GMAC_PORT_FIFO_CFG_1_TX_FIFO_MIN_TH_MASK,
		      (thresh << PP2_GMAC_PORT_FIFO_CFG_1_TX_FIFO_MIN_TH_OFFS));
	pp2_gop_gmac_write(gop, mac_num, PP2_GMAC_PORT_FIFO_CFG_1_REG, val);

	/* Disable bypass of sync module */
	val = pp2_gop_gmac_read(gop, mac_num, PP2_GMAC_PORT_CTRL4_REG);
	val |= PP2_GMAC_PORT_CTRL4_SYNC_BYPASS_MASK;
	/* configure DP clock select according to mode */
	val |= PP2_GMAC_PORT_CTRL4_DP_CLK_SEL_MASK;
	/* configure QSGMII bypass according to mode */
	val |= PP2_GMAC_PORT_CTRL4_QSGMII_BYPASS_ACTIVE_MASK;
	pp2_gop_gmac_write(gop, mac_num, PP2_GMAC_PORT_CTRL4_REG, val);

	val = pp2_gop_gmac_read(gop, mac_num, PP2_GMAC_PORT_CTRL2_REG);
	val |= PP2_GMAC_PORT_CTRL2_DIS_PADING_OFFS;
	pp2_gop_gmac_write(gop, mac_num, PP2_GMAC_PORT_CTRL2_REG, val);

	val = pp2_gop_gmac_read(gop, mac_num, PP2_GMAC_PORT_CTRL0_REG);
	/* configure GIG MAC to 1000Base-X mode connected to a
	 * fiber transceiver
	 */
	val |= PP2_GMAC_PORT_CTRL0_PORTTYPE_MASK;
	pp2_gop_gmac_write(gop, mac_num, PP2_GMAC_PORT_CTRL0_REG, val);

	/* configure AN 0x926C */
	an = PP2_GMAC_PORT_AUTO_NEG_CFG_EN_PCS_AN_MASK |
	    PP2_GMAC_PORT_AUTO_NEG_CFG_AN_BYPASS_EN_MASK |
	    PP2_GMAC_PORT_AUTO_NEG_CFG_SET_MII_SPEED_MASK |
	    PP2_GMAC_PORT_AUTO_NEG_CFG_SET_GMII_SPEED_MASK |
	    PP2_GMAC_PORT_AUTO_NEG_CFG_ADV_PAUSE_MASK |
	    PP2_GMAC_PORT_AUTO_NEG_CFG_SET_FULL_DX_MASK |
	    PP2_GMAC_PORT_AUTO_NEG_CFG_CHOOSE_SAMPLE_TX_CONFIG_MASK;
	pp2_gop_gmac_write(gop, mac_num, PP2_GMAC_PORT_AUTO_NEG_CFG_REG, an);
}

/* Set the internal mux's to the required MAC in the GOP */
int pp2_gop_gmac_mode_cfg(struct gop_hw *gop, struct pp2_mac_data *mac)
{
	u32 reg_addr;
	u32 val;

	int mac_num = mac->gop_index;

	/* Set TX FIFO thresholds */
	switch (mac->phy_mode) {
	case PP2_PHY_INTERFACE_MODE_SGMII:
		if (mac->speed == 2500)
			pp2_gop_gmac_sgmii2_5_cfg(gop, mac_num);
		else
			pp2_gop_gmac_sgmii_cfg(gop, mac_num);
		break;
	case PP2_PHY_INTERFACE_MODE_RGMII:
		pp2_gop_gmac_rgmii_cfg(gop, mac_num);
		break;
	case PP2_PHY_INTERFACE_MODE_QSGMII:
		pp2_gop_gmac_qsgmii_cfg(gop, mac_num);
		break;
	default:
		return -1;
	}

	/* Jumbo frame support - 0x1400*2= 0x2800 bytes */
	val = pp2_gop_gmac_read(gop, mac_num, PP2_GMAC_PORT_CTRL0_REG);
	U32_SET_FIELD(val, PP2_GMAC_PORT_CTRL0_FRAMESIZELIMIT_MASK,
		      (0x1400 << PP2_GMAC_PORT_CTRL0_FRAMESIZELIMIT_OFFS));
	pp2_gop_gmac_write(gop, mac_num, PP2_GMAC_PORT_CTRL0_REG, val);

	/* PeriodicXonEn disable */
	reg_addr = PP2_GMAC_PORT_CTRL1_REG;
	val = pp2_gop_gmac_read(gop, mac_num, reg_addr);
	val &= ~PP2_GMAC_PORT_CTRL1_EN_PERIODIC_FC_XON_MASK;
	pp2_gop_gmac_write(gop, mac_num, reg_addr, val);

	/* mask all ports interrupts */
	pp2_gop_gmac_port_link_event_mask(gop, mac_num);

	/* unmask link change interrupt */
	val = pp2_gop_gmac_read(gop, mac_num, PP2_GMAC_INTERRUPT_MASK_REG);
	val |= PP2_GMAC_INTERRUPT_CAUSE_LINK_CHANGE_MASK;
	val |= 1;		/* unmask summary bit */
	pp2_gop_gmac_write(gop, mac_num, PP2_GMAC_INTERRUPT_MASK_REG, val);

	return 0;
}

/* Configure MAC loopback */
int pp2_gop_gmac_loopback_cfg(struct gop_hw *gop, int mac_num,
			      enum pp2_lb_type type)
{
	u32 reg_addr;
	u32 val;

	reg_addr = PP2_GMAC_PORT_CTRL1_REG;
	val = pp2_gop_gmac_read(gop, mac_num, reg_addr);
	switch (type) {
	case PP2_DISABLE_LB:
		val &= ~PP2_GMAC_PORT_CTRL1_GMII_LOOPBACK_MASK;
		break;
	case PP2_TX_2_RX_LB:
		val |= PP2_GMAC_PORT_CTRL1_GMII_LOOPBACK_MASK;
		break;
	case PP2_RX_2_TX_LB:
	default:
		return -1;
	}
	pp2_gop_gmac_write(gop, mac_num, reg_addr, val);

	return 0;
}

/* Get MAC link status */
bool pp2_gop_gmac_link_status_get(struct gop_hw *gop, int mac_num)
{
	u32 reg_addr;
	u32 val;

	reg_addr = PP2_GMAC_PORT_STATUS0_REG;

	val = pp2_gop_gmac_read(gop, mac_num, reg_addr);
	return (val & 1) ? true : false;
}

/* Enable port and MIB counters */
void pp2_gop_gmac_port_enable(struct gop_hw *gop, int mac_num)
{
	u32 reg_val;

	reg_val = pp2_gop_gmac_read(gop, mac_num, PP2_GMAC_PORT_CTRL0_REG);
	reg_val |= PP2_GMAC_PORT_CTRL0_PORTEN_MASK;
	reg_val |= PP2_GMAC_PORT_CTRL0_COUNT_EN_MASK;

	pp2_gop_gmac_write(gop, mac_num, PP2_GMAC_PORT_CTRL0_REG, reg_val);
}

/* Disable port */
void pp2_gop_gmac_port_disable(struct gop_hw *gop, int mac_num)
{
	u32 reg_val;

	/* mask all ports interrupts */
	pp2_gop_gmac_port_link_event_mask(gop, mac_num);

	reg_val = pp2_gop_gmac_read(gop, mac_num, PP2_GMAC_PORT_CTRL0_REG);
	reg_val &= ~PP2_GMAC_PORT_CTRL0_PORTEN_MASK;

	pp2_gop_gmac_write(gop, mac_num, PP2_GMAC_PORT_CTRL0_REG, reg_val);
}

void pp2_gop_gmac_port_periodic_xon_set(struct gop_hw *gop,
					int mac_num, int enable)
{
	u32 reg_val;

	reg_val = pp2_gop_gmac_read(gop, mac_num, PP2_GMAC_PORT_CTRL1_REG);

	if (enable)
		reg_val |= PP2_GMAC_PORT_CTRL1_EN_PERIODIC_FC_XON_MASK;
	else
		reg_val &= ~PP2_GMAC_PORT_CTRL1_EN_PERIODIC_FC_XON_MASK;

	pp2_gop_gmac_write(gop, mac_num, PP2_GMAC_PORT_CTRL1_REG, reg_val);
}

int pp2_gop_gmac_link_status(struct gop_hw *gop, int mac_num,
			     struct pp2_port_link_status *pstatus)
{
	u32 reg_val;

	reg_val = pp2_gop_gmac_read(gop, mac_num, PP2_GMAC_PORT_STATUS0_REG);

	if (reg_val & PP2_GMAC_PORT_STATUS0_GMIISPEED_MASK)
		pstatus->speed = PP2_PORT_SPEED_1000;
	else if (reg_val & PP2_GMAC_PORT_STATUS0_MIISPEED_MASK)
		pstatus->speed = PP2_PORT_SPEED_100;
	else
		pstatus->speed = PP2_PORT_SPEED_10;

	if (reg_val & PP2_GMAC_PORT_STATUS0_LINKUP_MASK)
		pstatus->linkup = 1 /*TRUE*/;
	else
		pstatus->linkup = 0 /*FALSE*/;

	if (reg_val & PP2_GMAC_PORT_STATUS0_FULLDX_MASK)
		pstatus->duplex = PP2_PORT_DUPLEX_FULL;
	else
		pstatus->duplex = PP2_PORT_DUPLEX_HALF;

	if (reg_val & PP2_GMAC_PORT_STATUS0_PORTTXPAUSE_MASK)
		pstatus->tx_fc = PP2_PORT_FC_ACTIVE;
	else if (reg_val & PP2_GMAC_PORT_STATUS0_TXFCEN_MASK)
		pstatus->tx_fc = PP2_PORT_FC_ENABLE;
	else
		pstatus->tx_fc = PP2_PORT_FC_DISABLE;

	if (reg_val & PP2_GMAC_PORT_STATUS0_PORTRXPAUSE_MASK)
		pstatus->rx_fc = PP2_PORT_FC_ACTIVE;
	else if (reg_val & PP2_GMAC_PORT_STATUS0_RXFCEN_MASK)
		pstatus->rx_fc = PP2_PORT_FC_ENABLE;
	else
		pstatus->rx_fc = PP2_PORT_FC_DISABLE;

	return 0;
}

/* Change maximum receive size of the port */
int pp2_gop_gmac_max_rx_size_set(struct gop_hw *gop,
				 int mac_num, int max_rx_size)
{
	u32 reg_val;

	reg_val = pp2_gop_gmac_read(gop, mac_num, PP2_GMAC_PORT_CTRL0_REG);
	reg_val &= ~PP2_GMAC_PORT_CTRL0_FRAMESIZELIMIT_MASK;
	reg_val |= ((max_rx_size / 2) <<
		    PP2_GMAC_PORT_CTRL0_FRAMESIZELIMIT_OFFS);
	pp2_gop_gmac_write(gop, mac_num, PP2_GMAC_PORT_CTRL0_REG, reg_val);

	return 0;
}

/* Sets "Force Link Pass" and "Do Not Force Link Fail" bits.
 *  This function should only be called when the port is disabled.
 * INPUT:
 *	int  port		- port number
 *	bool force_link_pass	- Force Link Pass
 *	bool force_link_fail - Force Link Failure
 *		0, 0 - normal state: detect link via PHY and connector
 *		1, 1 - prohibited state.
 */
int pp2_gop_gmac_force_link_mode_set(struct gop_hw *gop, int mac_num,
				     bool force_link_up, bool force_link_down)
{
	u32 reg_val;

	/* Can't force link pass and link fail at the same time */
	if ((force_link_up) && (force_link_down))
		return -EINVAL;

	reg_val = pp2_gop_gmac_read(gop, mac_num, PP2_GMAC_PORT_AUTO_NEG_CFG_REG);

	if (force_link_up)
		reg_val |= PP2_GMAC_PORT_AUTO_NEG_CFG_FORCE_LINK_UP_MASK;
	else
		reg_val &= ~PP2_GMAC_PORT_AUTO_NEG_CFG_FORCE_LINK_UP_MASK;

	if (force_link_down)
		reg_val |= PP2_GMAC_PORT_AUTO_NEG_CFG_FORCE_LINK_DOWN_MASK;
	else
		reg_val &= ~PP2_GMAC_PORT_AUTO_NEG_CFG_FORCE_LINK_DOWN_MASK;

	pp2_gop_gmac_write(gop, mac_num, PP2_GMAC_PORT_AUTO_NEG_CFG_REG, reg_val);

	return 0;
}

/* Get "Force Link Pass" and "Do Not Force Link Fail" bits.
 * INPUT:
 *	int  port		- port number
 * OUTPUT:
 *	bool *force_link_pass	- Force Link Pass
 *	bool *force_link_fail	- Force Link Failure
 */
int pp2_gop_gmac_force_link_mode_get(struct gop_hw *gop, int mac_num,
				     bool *force_link_up, bool *force_link_down)
{
	u32 reg_val;

	/* Can't force link pass and link fail at the same time */
	if ((!force_link_up) || (!force_link_down))
		return -EINVAL;

	reg_val = pp2_gop_gmac_read(gop, mac_num, PP2_GMAC_PORT_AUTO_NEG_CFG_REG);

	if (reg_val & PP2_GMAC_PORT_AUTO_NEG_CFG_FORCE_LINK_UP_MASK)
		*force_link_up = true;
	else
		*force_link_up = false;

	if (reg_val & PP2_GMAC_PORT_AUTO_NEG_CFG_FORCE_LINK_DOWN_MASK)
		*force_link_down = true;
	else
		*force_link_down = false;

	return 0;
}

/* Sets port speed to Auto Negotiation / 1000 / 100 / 10 Mbps.
 *  Sets port duplex to Auto Negotiation / Full / Half Duplex.
 */
int pp2_gop_gmac_speed_duplex_set(struct gop_hw *gop, int mac_num,
				  enum pp2_port_speed speed,
				 enum pp2_port_duplex duplex)
{
	u32 reg_val;

	/* Check validity */
	if ((speed == PP2_PORT_SPEED_1000) && (duplex == PP2_PORT_DUPLEX_HALF))
		return -EINVAL;

	reg_val = pp2_gop_gmac_read(gop, mac_num, PP2_GMAC_PORT_AUTO_NEG_CFG_REG);

	switch (speed) {
	case PP2_PORT_SPEED_AN:
		reg_val |= PP2_GMAC_PORT_AUTO_NEG_CFG_EN_AN_SPEED_MASK;
		/* the other bits don't matter in this case */
		break;
	case PP2_PORT_SPEED_2500:
	case PP2_PORT_SPEED_1000:
		reg_val &= ~PP2_GMAC_PORT_AUTO_NEG_CFG_EN_AN_SPEED_MASK;
		reg_val |= PP2_GMAC_PORT_AUTO_NEG_CFG_SET_GMII_SPEED_MASK;
		/* the 100/10 bit doesn't matter in this case */
		break;
	case PP2_PORT_SPEED_100:
		reg_val &= ~PP2_GMAC_PORT_AUTO_NEG_CFG_EN_AN_SPEED_MASK;
		reg_val &= ~PP2_GMAC_PORT_AUTO_NEG_CFG_SET_GMII_SPEED_MASK;
		reg_val |= PP2_GMAC_PORT_AUTO_NEG_CFG_SET_MII_SPEED_MASK;
		break;
	case PP2_PORT_SPEED_10:
		reg_val &= ~PP2_GMAC_PORT_AUTO_NEG_CFG_EN_AN_SPEED_MASK;
		reg_val &= ~PP2_GMAC_PORT_AUTO_NEG_CFG_SET_GMII_SPEED_MASK;
		reg_val &= ~PP2_GMAC_PORT_AUTO_NEG_CFG_SET_MII_SPEED_MASK;
		break;
	default:
		pr_info("GMAC: Unexpected Speed value %d\n", speed);
		return -EINVAL;
	}

	switch (duplex) {
	case PP2_PORT_DUPLEX_AN:
		reg_val |= PP2_GMAC_PORT_AUTO_NEG_CFG_EN_FDX_AN_MASK;
		/* the other bits don't matter in this case */
		break;
	case PP2_PORT_DUPLEX_HALF:
		reg_val &= ~PP2_GMAC_PORT_AUTO_NEG_CFG_EN_FDX_AN_MASK;
		reg_val &= ~PP2_GMAC_PORT_AUTO_NEG_CFG_SET_FULL_DX_MASK;
		break;
	case PP2_PORT_DUPLEX_FULL:
		reg_val &= ~PP2_GMAC_PORT_AUTO_NEG_CFG_EN_FDX_AN_MASK;
		reg_val |= PP2_GMAC_PORT_AUTO_NEG_CFG_SET_FULL_DX_MASK;
		break;
	default:
		pr_err("GMAC: Unexpected Duplex value %d\n", duplex);
		return -EINVAL;
	}

	pp2_gop_gmac_write(gop, mac_num, PP2_GMAC_PORT_AUTO_NEG_CFG_REG, reg_val);
	return 0;
}

/* Gets port speed and duplex */
int pp2_gop_gmac_speed_duplex_get(struct gop_hw *gop, int mac_num,
				  enum pp2_port_speed *speed,
				 enum pp2_port_duplex *duplex)
{
	u32 reg_val;

	/* Check validity */
	if (!speed || !duplex)
		return -EINVAL;

	reg_val = pp2_gop_gmac_read(gop, mac_num, PP2_GMAC_PORT_AUTO_NEG_CFG_REG);

	if (reg_val & PP2_GMAC_PORT_AUTO_NEG_CFG_EN_AN_SPEED_MASK)
		*speed = PP2_PORT_SPEED_AN;
	else if (reg_val & PP2_GMAC_PORT_AUTO_NEG_CFG_SET_GMII_SPEED_MASK)
		*speed = PP2_PORT_SPEED_1000;
	else if (reg_val & PP2_GMAC_PORT_AUTO_NEG_CFG_SET_MII_SPEED_MASK)
		*speed = PP2_PORT_SPEED_100;
	else
		*speed = PP2_PORT_SPEED_10;

	if (reg_val & PP2_GMAC_PORT_AUTO_NEG_CFG_EN_FDX_AN_MASK)
		*duplex = PP2_PORT_DUPLEX_AN;
	else if (reg_val & PP2_GMAC_PORT_AUTO_NEG_CFG_SET_FULL_DX_MASK)
		*duplex = PP2_PORT_DUPLEX_FULL;
	else
		*duplex = PP2_PORT_DUPLEX_HALF;

	return 0;
}

/* Configure the port's Flow Control properties */
int pp2_gop_gmac_fc_set(struct gop_hw *gop, int mac_num, enum pp2_port_fc fc)
{
	u32 reg_val;
	u32 fc_en;

	reg_val = pp2_gop_gmac_read(gop, mac_num, PP2_GMAC_PORT_AUTO_NEG_CFG_REG);

	switch (fc) {
	case PP2_PORT_FC_AN_NO:
		reg_val &= ~PP2_GMAC_PORT_AUTO_NEG_CFG_EN_FC_AN_MASK;
		reg_val &= ~PP2_GMAC_PORT_AUTO_NEG_CFG_ADV_PAUSE_MASK;
		reg_val &= ~PP2_GMAC_PORT_AUTO_NEG_CFG_ADV_ASM_PAUSE_MASK;
		break;

	case PP2_PORT_FC_AN_SYM:
		reg_val |= PP2_GMAC_PORT_AUTO_NEG_CFG_EN_FC_AN_MASK;
		reg_val |= PP2_GMAC_PORT_AUTO_NEG_CFG_ADV_PAUSE_MASK;
		reg_val &= ~PP2_GMAC_PORT_AUTO_NEG_CFG_ADV_ASM_PAUSE_MASK;
		break;

	case PP2_PORT_FC_AN_ASYM:
		reg_val |= PP2_GMAC_PORT_AUTO_NEG_CFG_EN_FC_AN_MASK;
		reg_val |= PP2_GMAC_PORT_AUTO_NEG_CFG_ADV_PAUSE_MASK;
		reg_val |= PP2_GMAC_PORT_AUTO_NEG_CFG_ADV_ASM_PAUSE_MASK;
		break;

	case PP2_PORT_FC_DISABLE:
		reg_val &= ~PP2_GMAC_PORT_AUTO_NEG_CFG_EN_FC_AN_MASK;
		reg_val &= ~PP2_GMAC_PORT_AUTO_NEG_CFG_ADV_ASM_PAUSE_MASK;
		fc_en = pp2_gop_gmac_read(gop, mac_num, PP2_GMAC_PORT_CTRL4_REG);
		fc_en &= ~PP2_GMAC_PORT_CTRL4_FC_EN_RX_MASK;
		fc_en &= ~PP2_GMAC_PORT_CTRL4_FC_EN_TX_MASK;
		pp2_gop_gmac_write(gop, mac_num, PP2_GMAC_PORT_CTRL4_REG, fc_en);
		break;

	case PP2_PORT_FC_ENABLE:
		reg_val &= ~PP2_GMAC_PORT_AUTO_NEG_CFG_EN_FC_AN_MASK;
		fc_en = pp2_gop_gmac_read(gop, mac_num, PP2_GMAC_PORT_CTRL4_REG);
		fc_en |= PP2_GMAC_PORT_CTRL4_FC_EN_RX_MASK;
		fc_en |= PP2_GMAC_PORT_CTRL4_FC_EN_TX_MASK;
		pp2_gop_gmac_write(gop, mac_num, PP2_GMAC_PORT_CTRL4_REG, fc_en);
		break;

	default:
		pr_err("GMAC: Unexpected FlowControl value %d\n", fc);
		return -EINVAL;
	}

	pp2_gop_gmac_write(gop, mac_num, PP2_GMAC_PORT_AUTO_NEG_CFG_REG, reg_val);
	return 0;
}

/* Get Flow Control configuration of the port */
void pp2_gop_gmac_fc_get(struct gop_hw *gop, int mac_num, enum pp2_port_fc *fc)
{
	u32 reg_val;

	reg_val = pp2_gop_gmac_read(gop, mac_num, PP2_GMAC_PORT_AUTO_NEG_CFG_REG);

	if (reg_val & PP2_GMAC_PORT_AUTO_NEG_CFG_EN_FC_AN_MASK) {
		/* Auto negotiation is enabled */
		if (reg_val & PP2_GMAC_PORT_AUTO_NEG_CFG_ADV_PAUSE_MASK) {
			if (reg_val &
			    PP2_GMAC_PORT_AUTO_NEG_CFG_ADV_ASM_PAUSE_MASK)
				*fc = PP2_PORT_FC_AN_ASYM;
			else
				*fc = PP2_PORT_FC_AN_SYM;
		} else {
			*fc = PP2_PORT_FC_AN_NO;
		}
	} else {
		/* Auto negotiation is disabled */
		reg_val = pp2_gop_gmac_read(gop, mac_num,
					    PP2_GMAC_PORT_CTRL4_REG);
		if ((reg_val & PP2_GMAC_PORT_CTRL4_FC_EN_RX_MASK) &&
		    (reg_val & PP2_GMAC_PORT_CTRL4_FC_EN_TX_MASK))
			*fc = PP2_PORT_FC_ENABLE;
		else
			*fc = PP2_PORT_FC_DISABLE;
	}
}

int pp2_gop_gmac_port_link_speed_fc(struct gop_hw *gop, int mac_num,
				    enum pp2_port_speed speed, int force_link_up)
{
	if (force_link_up) {
		if (pp2_gop_gmac_speed_duplex_set(gop, mac_num, speed,
						  PP2_PORT_DUPLEX_FULL)) {
			pr_err("pp2_gop_gmac_speed_duplex_set failed\n");
			return -EPERM;
		}
		if (pp2_gop_gmac_fc_set(gop, mac_num, PP2_PORT_FC_ENABLE)) {
			pr_err("pp2_gop_gmac_fc_set failed\n");
			return -EPERM;
		}
		if (pp2_gop_gmac_force_link_mode_set(gop, mac_num, 1, 0)) {
			pr_err("pp2_gop_gmac_force_link_mode_set failed\n");
			return -EPERM;
		}
	} else {
		if (pp2_gop_gmac_force_link_mode_set(gop, mac_num, 0, 0)) {
			pr_err("pp2_gop_gmac_force_link_mode_set failed\n");
			return -EPERM;
		}
		if (pp2_gop_gmac_speed_duplex_set(gop, mac_num,
						  PP2_PORT_SPEED_AN,
						 PP2_PORT_DUPLEX_AN)) {
			pr_err("pp2_gop_gmac_speed_duplex_set failed\n");
			return -EPERM;
		}
		if (pp2_gop_gmac_fc_set(gop, mac_num, PP2_PORT_FC_AN_SYM)) {
			pr_err("pp2_gop_gmac_fc_set failed\n");
			return -EPERM;
		}
	}

	return 0;
}

void pp2_gop_gmac_port_link_event_mask(struct gop_hw *gop, int mac_num)
{
	u32 reg_val;

	reg_val = pp2_gop_gmac_read(gop, mac_num,
				    PP2_GMAC_INTERRUPT_SUM_MASK_REG);
	reg_val &= ~PP2_GMAC_INTERRUPT_SUM_CAUSE_LINK_CHANGE_MASK;
	pp2_gop_gmac_write(gop, mac_num,
			   PP2_GMAC_INTERRUPT_SUM_MASK_REG, reg_val);
}

void pp2_gop_gmac_port_link_event_unmask(struct gop_hw *gop, int mac_num)
{
	u32 reg_val;

	reg_val = pp2_gop_gmac_read(gop, mac_num,
				    PP2_GMAC_INTERRUPT_SUM_MASK_REG);
	reg_val |= PP2_GMAC_INTERRUPT_SUM_CAUSE_LINK_CHANGE_MASK;
	reg_val |= 1;		/* unmask summary bit */
	pp2_gop_gmac_write(gop, mac_num,
			   PP2_GMAC_INTERRUPT_SUM_MASK_REG, reg_val);
}

void pp2_gop_gmac_port_link_event_clear(struct gop_hw *gop, int mac_num)
{
	pp2_gop_gmac_read(gop, mac_num, PP2_GMAC_INTERRUPT_CAUSE_REG);
}

int pp2_gop_gmac_port_autoneg_restart(struct gop_hw *gop, int mac_num)
{
	u32 reg_val;

	reg_val = pp2_gop_gmac_read(gop, mac_num, PP2_GMAC_PORT_AUTO_NEG_CFG_REG);
	/* enable AN and restart it */
	reg_val |= PP2_GMAC_PORT_AUTO_NEG_CFG_EN_PCS_AN_MASK;
	reg_val |= PP2_GMAC_PORT_AUTO_NEG_CFG_INBAND_RESTARTAN_MASK;
	pp2_gop_gmac_write(gop, mac_num, PP2_GMAC_PORT_AUTO_NEG_CFG_REG, reg_val);
	return 0;
}

/*-------------------------------------------------------------------*/
void pp2_gop_port_enable(struct gop_hw *gop, struct pp2_mac_data *mac)
{
	int port_num = mac->gop_index;

	switch (mac->phy_mode) {
	case PP2_PHY_INTERFACE_MODE_RGMII:
	case PP2_PHY_INTERFACE_MODE_SGMII:
	case PP2_PHY_INTERFACE_MODE_QSGMII:
		pp2_gop_gmac_port_enable(gop, port_num);
		pp2_gop_force_link_mode_set(gop, mac, false, false);
		pp2_gop_gmac_reset(gop, port_num, UNRESET);
		break;
	case PP2_PHY_INTERFACE_MODE_XAUI:
	case PP2_PHY_INTERFACE_MODE_RXAUI:
	case PP2_PHY_INTERFACE_MODE_KR:
		pp2_gop_xlg_mac_port_enable(gop, port_num);
		break;
	default:
		pr_err("%s: Wrong port mode (%d)", __func__, mac->phy_mode);
		return;
	}

	GOP_DEBUG(gop->gop_port_debug[port_num].flags |= (1 << ENABLED));
}

void pp2_gop_port_disable(struct gop_hw *gop, struct pp2_mac_data *mac)
{
	int port_num = mac->gop_index;

	switch (mac->phy_mode) {
	case PP2_PHY_INTERFACE_MODE_RGMII:
	case PP2_PHY_INTERFACE_MODE_SGMII:
	case PP2_PHY_INTERFACE_MODE_QSGMII:
		pp2_gop_gmac_port_disable(gop, port_num);
		pp2_gop_force_link_mode_set(gop, mac, false, true);
		pp2_gop_gmac_reset(gop, port_num, RESET);
		break;
	case PP2_PHY_INTERFACE_MODE_XAUI:
	case PP2_PHY_INTERFACE_MODE_RXAUI:
	case PP2_PHY_INTERFACE_MODE_KR:
		pp2_gop_xlg_mac_port_disable(gop, port_num);
		break;
	default:
		pr_err("%s: Wrong port mode (%d)", __func__, mac->phy_mode);
		return;
	}

	GOP_DEBUG(gop->gop_port_debug[port_num].flags &= ~(1 << ENABLED));
}

void pp2_gop_port_periodic_xon_set(struct gop_hw *gop,
				   struct pp2_mac_data *mac, int enable)
{
	int port_num = mac->gop_index;

	switch (mac->phy_mode) {
	case PP2_PHY_INTERFACE_MODE_SGMII:
	case PP2_PHY_INTERFACE_MODE_QSGMII:
		pp2_gop_gmac_port_periodic_xon_set(gop, port_num, enable);
		break;
	case PP2_PHY_INTERFACE_MODE_XAUI:
	case PP2_PHY_INTERFACE_MODE_RXAUI:
	case PP2_PHY_INTERFACE_MODE_KR:
		pp2_gop_xlg_mac_port_periodic_xon_set(gop, port_num, enable);
		break;
	default:
		pr_err("%s: Wrong port mode (%d)", __func__, mac->phy_mode);
		return;
	}
}

bool pp2_gop_port_is_link_up(struct gop_hw *gop, struct pp2_mac_data *mac)
{
	int port_num = mac->gop_index;

	switch (mac->phy_mode) {
	case PP2_PHY_INTERFACE_MODE_RGMII:
	case PP2_PHY_INTERFACE_MODE_SGMII:
	case PP2_PHY_INTERFACE_MODE_QSGMII:
		return pp2_gop_gmac_link_status_get(gop, port_num);
	case PP2_PHY_INTERFACE_MODE_XAUI:
	case PP2_PHY_INTERFACE_MODE_RXAUI:
	case PP2_PHY_INTERFACE_MODE_KR:
		usleep_range(1000, 2000);
		return pp2_gop_xlg_mac_link_status_get(gop, port_num);
	default:
		pr_err("%s: Wrong port mode gop_port(%d), phy_mode(%d)",
			__func__, port_num, mac->phy_mode);
		return false;
	}
}

int pp2_gop_port_link_status(struct gop_hw *gop, struct pp2_mac_data *mac,
			     struct pp2_port_link_status *pstatus)
{
	int port_num = mac->gop_index;

	switch (mac->phy_mode) {
	case PP2_PHY_INTERFACE_MODE_RGMII:
	case PP2_PHY_INTERFACE_MODE_SGMII:
	case PP2_PHY_INTERFACE_MODE_QSGMII:
		pp2_gop_gmac_link_status(gop, port_num, pstatus);
		break;
	case PP2_PHY_INTERFACE_MODE_XAUI:
	case PP2_PHY_INTERFACE_MODE_RXAUI:
	case PP2_PHY_INTERFACE_MODE_KR:
		pp2_gop_xlg_mac_link_status(gop, port_num, pstatus);
		break;
	default:
		pr_err("%s: Wrong port mode (%d)", __func__, mac->phy_mode);
		return -1;
	}
	return 0;
}

int pp2_gop_port_events_mask(struct gop_hw *gop, struct pp2_mac_data *mac)
{
	int port_num = mac->gop_index;

	switch (mac->phy_mode) {
	case PP2_PHY_INTERFACE_MODE_RGMII:
	case PP2_PHY_INTERFACE_MODE_SGMII:
	case PP2_PHY_INTERFACE_MODE_QSGMII:
		pp2_gop_gmac_port_link_event_mask(gop, port_num);
		break;
	case PP2_PHY_INTERFACE_MODE_XAUI:
	case PP2_PHY_INTERFACE_MODE_RXAUI:
	case PP2_PHY_INTERFACE_MODE_KR:
		pp2_gop_xlg_port_link_event_mask(gop, port_num);
		break;
	default:
		pr_err("%s: Wrong port mode (%d)", __func__, mac->phy_mode);
		return -1;
	}
	return 0;
}

int pp2_gop_port_events_unmask(struct gop_hw *gop, struct pp2_mac_data *mac)
{
	int port_num = mac->gop_index;

	switch (mac->phy_mode) {
	case PP2_PHY_INTERFACE_MODE_RGMII:
	case PP2_PHY_INTERFACE_MODE_SGMII:
	case PP2_PHY_INTERFACE_MODE_QSGMII:
		pp2_gop_gmac_port_link_event_unmask(gop, port_num);
		/* gige interrupt cause connected to CPU via XLG
		 * external interrupt cause
		 */
		pp2_gop_xlg_port_external_event_unmask(gop, 0, 2);
		break;
	case PP2_PHY_INTERFACE_MODE_XAUI:
	case PP2_PHY_INTERFACE_MODE_RXAUI:
	case PP2_PHY_INTERFACE_MODE_KR:
		pp2_gop_xlg_port_external_event_unmask(gop, port_num, 1);
		break;
	default:
		pr_err("%s: Wrong port mode (%d)", __func__, mac->phy_mode);
		return -1;
	}
	return 0;
}

int pp2_gop_port_events_clear(struct gop_hw *gop, struct pp2_mac_data *mac)
{
	int port_num = mac->gop_index;

	switch (mac->phy_mode) {
	case PP2_PHY_INTERFACE_MODE_RGMII:
	case PP2_PHY_INTERFACE_MODE_SGMII:
	case PP2_PHY_INTERFACE_MODE_QSGMII:
		pp2_gop_gmac_port_link_event_clear(gop, port_num);
		break;
	case PP2_PHY_INTERFACE_MODE_XAUI:
	case PP2_PHY_INTERFACE_MODE_RXAUI:
	case PP2_PHY_INTERFACE_MODE_KR:
		pp2_gop_xlg_port_link_event_clear(gop, port_num);
		break;
	default:
		pr_err("%s: Wrong port mode (%d)", __func__, mac->phy_mode);
		return -1;
	}
	return 0;
}

int pp2_gop_status_show(struct gop_hw *gop, struct pp2_mac_data *mac)
{
	int port_num = mac->gop_index;
	struct pp2_port_link_status port_status;

	pp2_gop_port_link_status(gop, mac, &port_status);
	pr_info("---- GOP ID %d configuration ----\n", port_num);
	switch (mac->phy_mode) {
	case PP2_PHY_INTERFACE_MODE_RGMII:
		pr_info("Port mode               : RGMII\n");
		break;
	case PP2_PHY_INTERFACE_MODE_SGMII:
		pr_info("Port mode               : SGMII\n");
		break;
	case PP2_PHY_INTERFACE_MODE_QSGMII:
		pr_info("Port mode               : QSGMII\n");
		break;
	case PP2_PHY_INTERFACE_MODE_XAUI:
		pr_info("Port mode               : XAUI\n");
		break;
	case PP2_PHY_INTERFACE_MODE_RXAUI:
		pr_info("Port mode               : RXAUI\n");
		break;
	case PP2_PHY_INTERFACE_MODE_KR:
		pr_info("Port mode               : KR\n");
		break;
	default:
		pr_err("%s: Wrong port mode (%d)\n", __func__, mac->phy_mode);
		return -1;
	}

	pr_info("Link status             : %s",
		 (port_status.linkup) ? "link up\n" : "link down\n");

	if ((mac->phy_mode == PP2_PHY_INTERFACE_MODE_SGMII) &&
	    (mac->speed == 2500) && (port_status.speed == PP2_PORT_SPEED_1000))
		port_status.speed = PP2_PORT_SPEED_2500;

	switch (port_status.speed) {
	case PP2_PORT_SPEED_AN:
		pr_info("Port speed              : AutoNeg\n");
		break;
	case PP2_PORT_SPEED_10:
		pr_info("Port speed              : 10M\n");
		break;
	case PP2_PORT_SPEED_100:
		pr_info("Port speed              : 100M\n");
		break;
	case PP2_PORT_SPEED_1000:
		pr_info("Port speed              : 1G\n");
		break;
	case PP2_PORT_SPEED_2500:
		pr_info("Port speed              : 2.5G\n");
		break;
	case PP2_PORT_SPEED_10000:
		pr_info("Port speed              : 10G\n");
		break;
	default:
		pr_err("%s: Wrong port speed (%d)\n", __func__,
			port_status.speed);
		return -1;
	}

	switch (port_status.duplex) {
	case PP2_PORT_DUPLEX_AN:
		pr_info("Port duplex             : AutoNeg\n");
		break;
	case PP2_PORT_DUPLEX_HALF:
		pr_info("Port duplex             : half\n");
		break;
	case PP2_PORT_DUPLEX_FULL:
		pr_info("Port duplex             : full\n");
		break;
	default:
		pr_err("%s: Wrong port duplex (%d)\n", __func__,
			port_status.duplex);
		return -1;
	}

	return 0;
}

/* get port speed and duplex */
int pp2_gop_speed_duplex_get(struct gop_hw *gop, struct pp2_mac_data *mac,
			     enum pp2_port_speed *speed,
			    enum pp2_port_duplex *duplex)
{
	int port_num = mac->gop_index;

	switch (mac->phy_mode) {
	case PP2_PHY_INTERFACE_MODE_RGMII:
	case PP2_PHY_INTERFACE_MODE_SGMII:
	case PP2_PHY_INTERFACE_MODE_QSGMII:
		pp2_gop_gmac_speed_duplex_get(gop, port_num, speed, duplex);
		break;
	case PP2_PHY_INTERFACE_MODE_XAUI:
	case PP2_PHY_INTERFACE_MODE_RXAUI:
	case PP2_PHY_INTERFACE_MODE_KR:
		pp2_gop_xlg_mac_speed_duplex_get(gop, port_num, speed, duplex);
		break;
	default:
		pr_err("%s: Wrong port mode (%d)", __func__, mac->phy_mode);
		return -1;
	}
	return 0;
}

/* set port speed and duplex */
int pp2_gop_speed_duplex_set(struct gop_hw *gop, struct pp2_mac_data *mac,
			     enum pp2_port_speed speed,
			    enum pp2_port_duplex duplex)
{
	int port_num = mac->gop_index;

	switch (mac->phy_mode) {
	case PP2_PHY_INTERFACE_MODE_RGMII:
	case PP2_PHY_INTERFACE_MODE_SGMII:
	case PP2_PHY_INTERFACE_MODE_QSGMII:
		pp2_gop_gmac_speed_duplex_set(gop, port_num, speed, duplex);
		break;
	case PP2_PHY_INTERFACE_MODE_XAUI:
	case PP2_PHY_INTERFACE_MODE_RXAUI:
	case PP2_PHY_INTERFACE_MODE_KR:
		pp2_gop_xlg_mac_speed_duplex_set(gop, port_num, speed, duplex);
		break;
	default:
		pr_err("%s: Wrong port mode (%d)", __func__, mac->phy_mode);
		return -1;
	}
	return 0;
}

int pp2_gop_autoneg_restart(struct gop_hw *gop, struct pp2_mac_data *mac)
{
	int port_num = mac->gop_index;

	switch (mac->phy_mode) {
	case PP2_PHY_INTERFACE_MODE_RGMII:
		break;
	case PP2_PHY_INTERFACE_MODE_SGMII:
	case PP2_PHY_INTERFACE_MODE_QSGMII:
		pp2_gop_gmac_port_autoneg_restart(gop, port_num);
		break;
	case PP2_PHY_INTERFACE_MODE_XAUI:
	case PP2_PHY_INTERFACE_MODE_RXAUI:
	case PP2_PHY_INTERFACE_MODE_KR:
		pr_err("%s: on supported for port mode (%d)", __func__,
			mac->phy_mode);
		return -1;
	default:
		pr_err("%s: Wrong port mode (%d)", __func__, mac->phy_mode);
		return -1;
	}
	return 0;
}

int pp2_gop_fl_cfg(struct gop_hw *gop, struct pp2_mac_data *mac)
{
	int port_num = mac->gop_index;

	switch (mac->phy_mode) {
	case PP2_PHY_INTERFACE_MODE_RGMII:
	case PP2_PHY_INTERFACE_MODE_SGMII:
	case PP2_PHY_INTERFACE_MODE_QSGMII:
		/* disable AN */
		if (mac->speed == 2500)
			pp2_gop_speed_duplex_set(gop, mac,
						 PP2_PORT_SPEED_2500,
						PP2_PORT_DUPLEX_FULL);
		else
			pp2_gop_speed_duplex_set(gop, mac,
						 PP2_PORT_SPEED_1000,
						PP2_PORT_DUPLEX_FULL);
		/* force link */
		pp2_gop_gmac_force_link_mode_set(gop, port_num, true, false);
		break;
	case PP2_PHY_INTERFACE_MODE_XAUI:
	case PP2_PHY_INTERFACE_MODE_RXAUI:
	case PP2_PHY_INTERFACE_MODE_KR:
		return 0;
	default:
		pr_err("%s: Wrong port mode (%d)", __func__, mac->phy_mode);
		return -1;
	}
	return 0;
}

/* set port ForceLinkUp and ForceLinkDown*/
int pp2_gop_force_link_mode_set(struct gop_hw *gop, struct pp2_mac_data *mac,
				bool force_link_up, bool force_link_down)
{
	int port_num = mac->gop_index;

	switch (mac->phy_mode) {
	case PP2_PHY_INTERFACE_MODE_RGMII:
	case PP2_PHY_INTERFACE_MODE_SGMII:
	case PP2_PHY_INTERFACE_MODE_QSGMII:
		/* force link */
		pp2_gop_gmac_force_link_mode_set(gop, port_num,
						 force_link_up, force_link_down);
		break;
	case PP2_PHY_INTERFACE_MODE_XAUI:
	case PP2_PHY_INTERFACE_MODE_RXAUI:
	case PP2_PHY_INTERFACE_MODE_KR:
		return 0;
	default:
		pr_err("%s: Wrong port mode (%d)", __func__, mac->phy_mode);
		return -1;
	}
	return 0;
}

/* get port ForceLinkUp and ForceLinkDown*/
int pp2_gop_force_link_mode_get(struct gop_hw *gop, struct pp2_mac_data *mac,
				bool *force_link_up, bool *force_link_down)
{
	int port_num = mac->gop_index;

	switch (mac->phy_mode) {
	case PP2_PHY_INTERFACE_MODE_RGMII:
	case PP2_PHY_INTERFACE_MODE_SGMII:
	case PP2_PHY_INTERFACE_MODE_QSGMII:
		return pp2_gop_gmac_force_link_mode_get(gop, port_num,
						       force_link_up,
						       force_link_down);
		break;
	case PP2_PHY_INTERFACE_MODE_XAUI:
	case PP2_PHY_INTERFACE_MODE_RXAUI:
	case PP2_PHY_INTERFACE_MODE_KR:
		return 0;
	default:
		pr_err("%s: Wrong port mode (%d)", __func__, mac->phy_mode);
		return -1;
	}
	return 0;
}

/* set port internal loopback*/
int pp2_gop_loopback_set(struct gop_hw *gop, struct pp2_mac_data *mac, bool lb)
{
	int port_num = mac->gop_index;
	enum pp2_lb_type type;

	switch (mac->phy_mode) {
	case PP2_PHY_INTERFACE_MODE_RGMII:
	case PP2_PHY_INTERFACE_MODE_SGMII:
	case PP2_PHY_INTERFACE_MODE_QSGMII:
		/* set loopback */
		if (lb)
			type = PP2_TX_2_RX_LB;
		else
			type = PP2_DISABLE_LB;

		pp2_gop_gmac_loopback_cfg(gop, port_num, type);
		break;
	case PP2_PHY_INTERFACE_MODE_XAUI:
	case PP2_PHY_INTERFACE_MODE_RXAUI:
	case PP2_PHY_INTERFACE_MODE_KR:
		return 0;
	default:
		pr_err("%s: Wrong port mode (%d)", __func__, mac->phy_mode);
		return -1;
	}
	return 0;
}

/**************************************************************************
 * pp2_gop_gpcs_mode_cfg
 *
 * DESCRIPTION:
 *       Configure port to working with Gig PCS or don't.
 *
 * INPUTS:
 *       pcs_num   - physical PCS number
 *       en        - true to enable PCS
 *
 * OUTPUTS:
 *       None.
 *
 * RETURNS:
 *       0  - on success
 *       1  - on error
 *
 **************************************************************************/
int pp2_gop_gpcs_mode_cfg(struct gop_hw *gop, int pcs_num, bool en)
{
	u32 val;

	val = pp2_gop_gmac_read(gop, pcs_num, PP2_GMAC_PORT_CTRL2_REG);

	if (en)
		val |= PP2_GMAC_PORT_CTRL2_PCS_EN_MASK;
	else
		val &= ~PP2_GMAC_PORT_CTRL2_PCS_EN_MASK;

	/* enable / disable PCS on this port */
	pp2_gop_gmac_write(gop, pcs_num, PP2_GMAC_PORT_CTRL2_REG, val);

	return 0;
}

/**************************************************************************
 * pp2_gop_gpcs_reset
 *
 * DESCRIPTION:
 *       Set the selected PCS number to reset or exit from reset.
 *
 * INPUTS:
 *       pcs_num    - physical PCS number
 *       action    - reset / unreset
 *
 * OUTPUTS:
 *       None.
 *
 * RETURNS:
 *       0  - on success
 *       1  - on error
 *
 *************************************************************************/
int pp2_gop_gpcs_reset(struct gop_hw *gop, int pcs_num, enum pp2_reset act)
{
	u32 reg_data;

	reg_data = pp2_gop_gmac_read(gop, pcs_num, PP2_GMAC_PORT_CTRL2_REG);
	if (act == RESET)
		U32_SET_FIELD(reg_data, PP2_GMAC_PORT_CTRL2_SGMII_MODE_MASK, 0);
	else
		U32_SET_FIELD(reg_data, PP2_GMAC_PORT_CTRL2_SGMII_MODE_MASK,
			      1 << PP2_GMAC_PORT_CTRL2_SGMII_MODE_OFFS);

	pp2_gop_gmac_write(gop, pcs_num, PP2_GMAC_PORT_CTRL2_REG, reg_data);
	return 0;
}



/* Set the MAC to reset or exit from reset */
int pp2_gop_xlg_mac_reset(struct gop_hw *gop, int mac_num, enum pp2_reset reset)
{
	u32 reg_addr;
	u32 val;

	reg_addr = PP2_XLG_PORT_MAC_CTRL0_REG;

	/* read - modify - write */
	val = pp2_gop_xlg_mac_read(gop, mac_num, reg_addr);
	if (reset == RESET)
		val &= ~PP2_XLG_MAC_CTRL0_MACRESETN_MASK;
	else
		val |= PP2_XLG_MAC_CTRL0_MACRESETN_MASK;
	pp2_gop_xlg_mac_write(gop, mac_num, reg_addr, val);

	return 0;
}

/* Set the internal mux's to the required MAC in the GOP */
int pp2_gop_xlg_mac_mode_cfg(struct gop_hw *gop, int mac_num,
			     int num_of_act_lanes)
{
	u32 reg_addr;
	u32 val;

	/* configure 10G MAC mode */
	reg_addr = PP2_XLG_PORT_MAC_CTRL0_REG;
	val = pp2_gop_xlg_mac_read(gop, mac_num, reg_addr);
	U32_SET_FIELD(val, PP2_XLG_MAC_CTRL0_RXFCEN_MASK,
		      (1 << PP2_XLG_MAC_CTRL0_RXFCEN_OFFS));
	pp2_gop_xlg_mac_write(gop, mac_num, reg_addr, val);

	reg_addr = PP2_XLG_PORT_MAC_CTRL3_REG;
	val = pp2_gop_xlg_mac_read(gop, mac_num, reg_addr);
	U32_SET_FIELD(val, PP2_XLG_MAC_CTRL3_MACMODESELECT_MASK,
		      (1 << PP2_XLG_MAC_CTRL3_MACMODESELECT_OFFS));
	pp2_gop_xlg_mac_write(gop, mac_num, reg_addr, val);

	reg_addr = PP2_XLG_PORT_MAC_CTRL4_REG;

	/* read - modify - write */
	val = pp2_gop_xlg_mac_read(gop, mac_num, reg_addr);
	U32_SET_FIELD(val, PP2_XLG_MAC_CTRL4_MAC_MODE_DMA_1G_MASK, 0 <<
		      PP2_XLG_MAC_CTRL4_MAC_MODE_DMA_1G_OFFS);
	U32_SET_FIELD(val, PP2_XLG_MAC_CTRL4_FORWARD_PFC_EN_MASK, 1 <<
		      PP2_XLG_MAC_CTRL4_FORWARD_PFC_EN_OFFS);
	U32_SET_FIELD(val, PP2_XLG_MAC_CTRL4_FORWARD_802_3X_FC_EN_MASK, 1 <<
		      PP2_XLG_MAC_CTRL4_FORWARD_802_3X_FC_EN_OFFS);
	U32_SET_FIELD(val, PP2_XLG_MAC_CTRL4_EN_IDLE_CHECK_FOR_LINK_MASK, 0 <<
	      PP2_XLG_MAC_CTRL4_EN_IDLE_CHECK_FOR_LINK_OFFS);
	pp2_gop_xlg_mac_write(gop, mac_num, reg_addr, val);

	/* Jumbo frame support - 0x1400*2= 0x2800 bytes */
	val = pp2_gop_xlg_mac_read(gop, mac_num, PP2_XLG_PORT_MAC_CTRL1_REG);
	U32_SET_FIELD(val, PP2_XLG_MAC_CTRL1_FRAMESIZELIMIT_MASK, 0x1400);
	pp2_gop_xlg_mac_write(gop, mac_num, PP2_XLG_PORT_MAC_CTRL1_REG, val);

	/* mask all port interrupts */
	pp2_gop_xlg_port_link_event_mask(gop, mac_num);

	/* unmask link change interrupt */
	val = pp2_gop_xlg_mac_read(gop, mac_num, PP2_XLG_INTERRUPT_MASK_REG);
	val |= PP2_XLG_INTERRUPT_LINK_CHANGE_MASK;
	val |= 1;		/* unmask summary bit */
	pp2_gop_xlg_mac_write(gop, mac_num, PP2_XLG_INTERRUPT_MASK_REG, val);

	return 0;
}

/* Configure MAC loopback */
int pp2_gop_xlg_mac_loopback_cfg(struct gop_hw *gop, int mac_num,
				 enum pp2_lb_type type)
{
	u32 reg_addr;
	u32 val;

	reg_addr = PP2_XLG_PORT_MAC_CTRL1_REG;
	val = pp2_gop_xlg_mac_read(gop, mac_num, reg_addr);
	switch (type) {
	case PP2_DISABLE_LB:
		val &= ~PP2_XLG_MAC_CTRL1_MACLOOPBACKEN_MASK;
		val &= ~PP2_XLG_MAC_CTRL1_XGMIILOOPBACKEN_MASK;
		break;
	case PP2_RX_2_TX_LB:
		val &= ~PP2_XLG_MAC_CTRL1_MACLOOPBACKEN_MASK;
		val |= PP2_XLG_MAC_CTRL1_XGMIILOOPBACKEN_MASK;
		break;
	case PP2_TX_2_RX_LB:
		val |= PP2_XLG_MAC_CTRL1_MACLOOPBACKEN_MASK;
		val |= PP2_XLG_MAC_CTRL1_XGMIILOOPBACKEN_MASK;
		break;
	default:
		return -1;
	}
	pp2_gop_xlg_mac_write(gop, mac_num, reg_addr, val);
	return 0;
}

/* Get MAC link status */
bool pp2_gop_xlg_mac_link_status_get(struct gop_hw *gop, int mac_num)
{
	if (pp2_gop_xlg_mac_read(gop, mac_num, PP2_XLG_MAC_PORT_STATUS_REG) & 1)
		return true;

	return false;
}

/* Enable port and MIB counters update */
void pp2_gop_xlg_mac_port_enable(struct gop_hw *gop, int mac_num)
{
	u32 reg_val;

	reg_val = pp2_gop_xlg_mac_read(gop, mac_num, PP2_XLG_PORT_MAC_CTRL0_REG);
	reg_val |= PP2_XLG_MAC_CTRL0_PORTEN_MASK;
	reg_val &= ~PP2_XLG_MAC_CTRL0_MIBCNTDIS_MASK;

	pp2_gop_xlg_mac_write(gop, mac_num, PP2_XLG_PORT_MAC_CTRL0_REG, reg_val);
}

/* Disable port */
void pp2_gop_xlg_mac_port_disable(struct gop_hw *gop, int mac_num)
{
	u32 reg_val;

	/* mask all port interrupts */
	pp2_gop_xlg_port_link_event_mask(gop, mac_num);

	reg_val = pp2_gop_xlg_mac_read(gop, mac_num, PP2_XLG_PORT_MAC_CTRL0_REG);
	reg_val &= ~PP2_XLG_MAC_CTRL0_PORTEN_MASK;

	pp2_gop_xlg_mac_write(gop, mac_num, PP2_XLG_PORT_MAC_CTRL0_REG, reg_val);
}

void pp2_gop_xlg_mac_port_periodic_xon_set(struct gop_hw *gop,
					   int mac_num, int enable)
{
	u32 reg_val;

	reg_val = pp2_gop_xlg_mac_read(gop, mac_num, PP2_XLG_PORT_MAC_CTRL0_REG);

	if (enable)
		reg_val |= PP2_XLG_MAC_CTRL0_PERIODICXONEN_MASK;
	else
		reg_val &= ~PP2_XLG_MAC_CTRL0_PERIODICXONEN_MASK;

	pp2_gop_xlg_mac_write(gop, mac_num, PP2_XLG_PORT_MAC_CTRL0_REG, reg_val);
}

int pp2_gop_xlg_mac_link_status(struct gop_hw *gop,
				int mac_num, struct pp2_port_link_status *pstatus)
{
	u32 reg_val;
	u32 mac_mode;
	u32 fc_en;

	reg_val = pp2_gop_xlg_mac_read(gop, mac_num, PP2_XLG_PORT_MAC_CTRL3_REG);
	mac_mode = (reg_val & PP2_XLG_MAC_CTRL3_MACMODESELECT_MASK) >>
	    PP2_XLG_MAC_CTRL3_MACMODESELECT_OFFS;

	/* speed  and duplex */
	switch (mac_mode) {
	case 0:
		pstatus->speed = PP2_PORT_SPEED_1000;
		pstatus->duplex = PP2_PORT_DUPLEX_AN;
		break;
	case 1:
		pstatus->speed = PP2_PORT_SPEED_10000;
		pstatus->duplex = PP2_PORT_DUPLEX_FULL;
		break;
	default:
		return -1;
	}

	/* link status */
	reg_val = pp2_gop_xlg_mac_read(gop, mac_num, PP2_XLG_MAC_PORT_STATUS_REG);
	if (reg_val & PP2_XLG_MAC_PORT_STATUS_LINKSTATUS_MASK)
		pstatus->linkup = 1 /*TRUE*/;
	else
		pstatus->linkup = 0 /*FALSE*/;

	/* flow control status */
	fc_en = pp2_gop_xlg_mac_read(gop, mac_num, PP2_XLG_PORT_MAC_CTRL0_REG);
	if (reg_val & PP2_XLG_MAC_PORT_STATUS_PORTTXPAUSE_MASK)
		pstatus->tx_fc = PP2_PORT_FC_ACTIVE;
	else if (fc_en & PP2_XLG_MAC_CTRL0_TXFCEN_MASK)
		pstatus->tx_fc = PP2_PORT_FC_ENABLE;
	else
		pstatus->tx_fc = PP2_PORT_FC_DISABLE;

	if (reg_val & PP2_XLG_MAC_PORT_STATUS_PORTRXPAUSE_MASK)
		pstatus->rx_fc = PP2_PORT_FC_ACTIVE;
	else if (fc_en & PP2_XLG_MAC_CTRL0_RXFCEN_MASK)
		pstatus->rx_fc = PP2_PORT_FC_ENABLE;
	else
		pstatus->rx_fc = PP2_PORT_FC_DISABLE;

	return 0;
}

/* Change maximum receive size of the port */
int pp2_gop_xlg_mac_max_rx_size_set(struct gop_hw *gop, int mac_num,
				    int max_rx_size)
{
	u32 reg_val;

	reg_val = pp2_gop_xlg_mac_read(gop, mac_num, PP2_XLG_PORT_MAC_CTRL1_REG);
	reg_val &= ~PP2_XLG_MAC_CTRL1_FRAMESIZELIMIT_MASK;
	reg_val |= (((max_rx_size - MV_MH_SIZE) / 2) <<
		    PP2_XLG_MAC_CTRL1_FRAMESIZELIMIT_OFFS);
	pp2_gop_xlg_mac_write(gop, mac_num, PP2_XLG_PORT_MAC_CTRL1_REG, reg_val);

	return 0;
}

/* Sets "Force Link Pass" and "Do Not Force Link Fail" bits.
 *  This function should only be called when the port is disabled.
 * INPUT:
 *	int  port		- port number
 *	bool force_link_pass	- Force Link Pass
 *	bool force_link_fail - Force Link Failure
 *		0, 0 - normal state: detect link via PHY and connector
 *		1, 1 - prohibited state.
 */
int pp2_gop_xlg_mac_force_link_mode_set(struct gop_hw *gop, int mac_num,
					bool force_link_up, bool force_link_down)
{
	u32 reg_val;

	/* Can't force link pass and link fail at the same time */
	if ((force_link_up) && (force_link_down))
		return -EINVAL;

	reg_val = pp2_gop_xlg_mac_read(gop, mac_num, PP2_XLG_PORT_MAC_CTRL0_REG);

	if (force_link_up)
		reg_val |= PP2_XLG_MAC_CTRL0_FORCELINKPASS_MASK;
	else
		reg_val &= ~PP2_XLG_MAC_CTRL0_FORCELINKPASS_MASK;

	if (force_link_down)
		reg_val |= PP2_XLG_MAC_CTRL0_FORCELINKDOWN_MASK;
	else
		reg_val &= ~PP2_XLG_MAC_CTRL0_FORCELINKDOWN_MASK;

	pp2_gop_xlg_mac_write(gop, mac_num, PP2_XLG_PORT_MAC_CTRL0_REG, reg_val);

	return 0;
}

/* Sets port speed to Auto Negotiation / 1000 / 100 / 10 Mbps.
 *  Sets port duplex to Auto Negotiation / Full / Half Duplex.
 */
int pp2_gop_xlg_mac_speed_duplex_set(struct gop_hw *gop, int mac_num,
				     enum pp2_port_speed speed,
				    enum pp2_port_duplex duplex)
{
	/* not supported */
	return -1;
}

/* Gets port speed and duplex */
int pp2_gop_xlg_mac_speed_duplex_get(struct gop_hw *gop, int mac_num,
				     enum pp2_port_speed *speed,
				    enum pp2_port_duplex *duplex)
{
	/* not supported */
	return -1;
}

/* Configure the port's Flow Control properties */
int pp2_gop_xlg_mac_fc_set(struct gop_hw *gop, int mac_num, enum pp2_port_fc fc)
{
	u32 reg_val;

	reg_val = pp2_gop_xlg_mac_read(gop, mac_num, PP2_XLG_PORT_MAC_CTRL0_REG);

	switch (fc) {
	case PP2_PORT_FC_DISABLE:
		reg_val &= ~PP2_XLG_MAC_CTRL0_RXFCEN_MASK;
		reg_val &= ~PP2_XLG_MAC_CTRL0_TXFCEN_MASK;
		break;

	case PP2_PORT_FC_ENABLE:
		reg_val |= PP2_XLG_MAC_CTRL0_RXFCEN_MASK;
		reg_val |= PP2_XLG_MAC_CTRL0_TXFCEN_MASK;
		break;

	case PP2_PORT_FC_AN_NO:
	case PP2_PORT_FC_AN_SYM:
	case PP2_PORT_FC_AN_ASYM:
	default:
		pr_err("XLG MAC: Unexpected FlowControl value %d\n", fc);
		return -EINVAL;
	}

	pp2_gop_xlg_mac_write(gop, mac_num, PP2_XLG_PORT_MAC_CTRL0_REG, reg_val);
	return 0;
}

/* Get Flow Control configuration of the port */
void pp2_gop_xlg_mac_fc_get(struct gop_hw *gop, int mac_num, enum pp2_port_fc *fc)
{
	u32 reg_val;

	/* No auto negotiation for flow control */
	reg_val = pp2_gop_xlg_mac_read(gop, mac_num, PP2_XLG_PORT_MAC_CTRL0_REG);

	if ((reg_val & PP2_XLG_MAC_CTRL0_RXFCEN_MASK) &&
	    (reg_val & PP2_XLG_MAC_CTRL0_TXFCEN_MASK))
		*fc = PP2_PORT_FC_ENABLE;
	else
		*fc = PP2_PORT_FC_DISABLE;
}

int pp2_gop_xlg_mac_port_link_speed_fc(struct gop_hw *gop, int mac_num,
				       enum pp2_port_speed speed,
				      int force_link_up)
{
	if (force_link_up) {
		if (pp2_gop_xlg_mac_fc_set(gop, mac_num, PP2_PORT_FC_ENABLE)) {
			pr_err("pp2_gop_xlg_mac_fc_set failed\n");
			return -EPERM;
		}
		if (pp2_gop_xlg_mac_force_link_mode_set(gop, mac_num, 1, 0)) {
			pr_err("pp2_gop_xlg_mac_force_link_mode_set failed\n");
			return -EPERM;
		}
	} else {
		if (pp2_gop_xlg_mac_force_link_mode_set(gop, mac_num, 0, 0)) {
			pr_err("pp2_gop_xlg_mac_force_link_mode_set failed\n");
			return -EPERM;
		}
		if (pp2_gop_xlg_mac_fc_set(gop, mac_num, PP2_PORT_FC_AN_SYM)) {
			pr_err("pp2_gop_xlg_mac_fc_set failed\n");
			return -EPERM;
		}
	}

	return 0;
}

void pp2_gop_xlg_port_link_event_mask(struct gop_hw *gop, int mac_num)
{
	u32 reg_val;

	reg_val = pp2_gop_xlg_mac_read(gop, mac_num,
				       PP2_XLG_EXTERNAL_INTERRUPT_MASK_REG);
	reg_val &= ~(1 << 1);
	pp2_gop_xlg_mac_write(gop, mac_num,
			      PP2_XLG_EXTERNAL_INTERRUPT_MASK_REG, reg_val);
}

void pp2_gop_xlg_port_external_event_unmask(struct gop_hw *gop, int mac_num,
					    int bit_2_open)
{
	u32 reg_val;

	reg_val = pp2_gop_xlg_mac_read(gop, mac_num,
				       PP2_XLG_EXTERNAL_INTERRUPT_MASK_REG);
	reg_val |= (1 << bit_2_open);
	reg_val |= 1;		/* unmask summary bit */
	pp2_gop_xlg_mac_write(gop, mac_num,
			      PP2_XLG_EXTERNAL_INTERRUPT_MASK_REG, reg_val);
}

void pp2_gop_xlg_port_link_event_clear(struct gop_hw *gop, int mac_num)
{
	pp2_gop_xlg_mac_read(gop, mac_num, PP2_XLG_INTERRUPT_CAUSE_REG);
}

void pp2_gop_xlg_2_gig_mac_cfg(struct gop_hw *gop, int mac_num)
{
	u32 reg_val;

	/* relevant only for MAC0 (XLG0 and GMAC0) */
	if (mac_num > 0)
		return;

	/* configure 1Gig MAC mode */
	reg_val = pp2_gop_xlg_mac_read(gop, mac_num, PP2_XLG_PORT_MAC_CTRL3_REG);
	U32_SET_FIELD(reg_val, PP2_XLG_MAC_CTRL3_MACMODESELECT_MASK,
		      (0 << PP2_XLG_MAC_CTRL3_MACMODESELECT_OFFS));
	pp2_gop_xlg_mac_write(gop, mac_num, PP2_XLG_PORT_MAC_CTRL3_REG, reg_val);
}



