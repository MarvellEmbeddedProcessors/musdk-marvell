#include <unistd.h>

#include "pp2_types.h"

#include "pp2_hw_type.h"
#include "pp2.h"
#include "pp2_gop_def.h"

/* Set the MAC to reset or exit from reset */
int pp2_gop_gmac_reset(struct gop_hw *gop, int mac_num, enum pp2_reset reset)
{
	uint32_t reg_addr;
	uint32_t val;

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
	uint32_t val, thresh, an;

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
	uint32_t val, thresh, an;

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
	uint32_t val, thresh, an;

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
	uint32_t val, thresh, an;

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
	uint32_t reg_addr;
	uint32_t val;

	int mac_num = mac->gop_index;

	/* Set TX FIFO thresholds */
	switch (mac->phy_mode) {
	case PHY_INTERFACE_MODE_SGMII:
		if (mac->speed == 2500)
			pp2_gop_gmac_sgmii2_5_cfg(gop, mac_num);
		else
			pp2_gop_gmac_sgmii_cfg(gop, mac_num);
		break;
	case PHY_INTERFACE_MODE_RGMII:
		pp2_gop_gmac_rgmii_cfg(gop, mac_num);
		break;
	case PHY_INTERFACE_MODE_QSGMII:
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
	uint32_t reg_addr;
	uint32_t val;

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
	uint32_t reg_addr;
	uint32_t val;

	reg_addr = PP2_GMAC_PORT_STATUS0_REG;

	val = pp2_gop_gmac_read(gop, mac_num, reg_addr);
	return (val & 1) ? true : false;
}

/* Enable port and MIB counters */
void pp2_gop_gmac_port_enable(struct gop_hw *gop, int mac_num)
{
	uint32_t reg_val;

	reg_val = pp2_gop_gmac_read(gop, mac_num, PP2_GMAC_PORT_CTRL0_REG);
	reg_val |= PP2_GMAC_PORT_CTRL0_PORTEN_MASK;
	reg_val |= PP2_GMAC_PORT_CTRL0_COUNT_EN_MASK;

	pp2_gop_gmac_write(gop, mac_num, PP2_GMAC_PORT_CTRL0_REG, reg_val);
}

/* Disable port */
void pp2_gop_gmac_port_disable(struct gop_hw *gop, int mac_num)
{
	uint32_t reg_val;

	/* mask all ports interrupts */
	pp2_gop_gmac_port_link_event_mask(gop, mac_num);

	reg_val = pp2_gop_gmac_read(gop, mac_num, PP2_GMAC_PORT_CTRL0_REG);
	reg_val &= ~PP2_GMAC_PORT_CTRL0_PORTEN_MASK;

	pp2_gop_gmac_write(gop, mac_num, PP2_GMAC_PORT_CTRL0_REG, reg_val);
}

void pp2_gop_gmac_port_periodic_xon_set(struct gop_hw *gop,
				       int mac_num, int enable)
{
	uint32_t reg_val;

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
	uint32_t reg_val;

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
	uint32_t reg_val;

	reg_val = pp2_gop_gmac_read(gop, mac_num, PP2_GMAC_PORT_CTRL0_REG);
	reg_val &= ~PP2_GMAC_PORT_CTRL0_FRAMESIZELIMIT_MASK;
	reg_val |= (((max_rx_size - PP2_MH_SIZE) / 2) <<
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
	uint32_t reg_val;

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
	uint32_t reg_val;

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
	uint32_t reg_val;

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
		pp2_info("GMAC: Unexpected Speed value %d\n", speed);
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
		pp2_err("GMAC: Unexpected Duplex value %d\n", duplex);
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
	uint32_t reg_val;

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
	uint32_t reg_val;
	uint32_t fc_en;

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
		pp2_err("GMAC: Unexpected FlowControl value %d\n", fc);
		return -EINVAL;
	}

	pp2_gop_gmac_write(gop, mac_num, PP2_GMAC_PORT_AUTO_NEG_CFG_REG, reg_val);
	return 0;
}

/* Get Flow Control configuration of the port */
void pp2_gop_gmac_fc_get(struct gop_hw *gop, int mac_num, enum pp2_port_fc *fc)
{
	uint32_t reg_val;

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
			pp2_err("pp2_gop_gmac_speed_duplex_set failed\n");
			return -EPERM;
		}
		if (pp2_gop_gmac_fc_set(gop, mac_num, PP2_PORT_FC_ENABLE)) {
			pp2_err("pp2_gop_gmac_fc_set failed\n");
			return -EPERM;
		}
		if (pp2_gop_gmac_force_link_mode_set(gop, mac_num, 1, 0)) {
			pp2_err("pp2_gop_gmac_force_link_mode_set failed\n");
			return -EPERM;
		}
	} else {
		if (pp2_gop_gmac_force_link_mode_set(gop, mac_num, 0, 0)) {
			pp2_err("pp2_gop_gmac_force_link_mode_set failed\n");
			return -EPERM;
		}
		if (pp2_gop_gmac_speed_duplex_set(gop, mac_num,
						 PP2_PORT_SPEED_AN,
						 PP2_PORT_DUPLEX_AN)) {
			pp2_err("pp2_gop_gmac_speed_duplex_set failed\n");
			return -EPERM;
		}
		if (pp2_gop_gmac_fc_set(gop, mac_num, PP2_PORT_FC_AN_SYM)) {
			pp2_err("pp2_gop_gmac_fc_set failed\n");
			return -EPERM;
		}
	}

	return 0;
}

void pp2_gop_gmac_port_link_event_mask(struct gop_hw *gop, int mac_num)
{
	uint32_t reg_val;

	reg_val = pp2_gop_gmac_read(gop, mac_num,
				   PP2_GMAC_INTERRUPT_SUM_MASK_REG);
	reg_val &= ~PP2_GMAC_INTERRUPT_SUM_CAUSE_LINK_CHANGE_MASK;
	pp2_gop_gmac_write(gop, mac_num,
			  PP2_GMAC_INTERRUPT_SUM_MASK_REG, reg_val);
}

void pp2_gop_gmac_port_link_event_unmask(struct gop_hw *gop, int mac_num)
{
	uint32_t reg_val;

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
	uint32_t reg_val;

	reg_val = pp2_gop_gmac_read(gop, mac_num, PP2_GMAC_PORT_AUTO_NEG_CFG_REG);
	/* enable AN and restart it */
	reg_val |= PP2_GMAC_PORT_AUTO_NEG_CFG_EN_PCS_AN_MASK;
	reg_val |= PP2_GMAC_PORT_AUTO_NEG_CFG_INBAND_RESTARTAN_MASK;
	pp2_gop_gmac_write(gop, mac_num, PP2_GMAC_PORT_AUTO_NEG_CFG_REG, reg_val);
	return 0;
}

/*************************************************************************
* pp2_port_init
*
* DESCRIPTION:
*       Init physical port. Configures the port mode and all it's elements
*       accordingly.
*       Does not verify that the selected mode/port number is valid at the
*       core level.
*
* INPUTS:
*       port_num    - physical port number
*       port_mode   - port standard metric
*
* OUTPUTS:
*       None.
*
* RETURNS:
*       0  - on success
*       1  - on error
*
*************************************************************************/
int pp2_gop_port_init(struct gop_hw *gop, struct pp2_mac_data *mac, uint32_t lb)
{
	int num_of_act_lanes;
	int mac_num = mac->gop_index;

	if (mac_num >= PP2_GOP_MAC_NUM) {
		pp2_err("%s: illegal port number %d", __func__, mac_num);
		return -1;
	}

	switch (mac->phy_mode) {
	case PHY_INTERFACE_MODE_RGMII:
		pp2_gop_force_link_mode_set(gop, mac, false, true);
		pp2_gop_gmac_reset(gop, mac_num, RESET);

		/* configure PCS */
		pp2_gop_gpcs_mode_cfg(gop, mac_num, false);

		/* configure MAC */
		pp2_gop_gmac_mode_cfg(gop, mac);

        /* Enable or disable MAC loopback */
        pp2_gop_loopback_set(gop, mac, lb);

		/* select proper Mac mode */
		pp2_gop_xlg_2_gig_mac_cfg(gop, mac_num);

		/* pcs unreset */
		pp2_gop_gpcs_reset(gop, mac_num, UNRESET);
		/* mac unreset */
		pp2_gop_gmac_reset(gop, mac_num, UNRESET);
		pp2_gop_force_link_mode_set(gop, mac, false, false);
		break;
	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_QSGMII:
		num_of_act_lanes = 1;
		pp2_gop_force_link_mode_set(gop, mac, false, true);
		pp2_gop_gmac_reset(gop, mac_num, RESET);

		/* configure PCS */
		pp2_gop_gpcs_mode_cfg(gop, mac_num, true);

		/* configure MAC */
		pp2_gop_gmac_mode_cfg(gop, mac);

        /* Enable or disable MAC loopback */
        pp2_gop_loopback_set(gop, mac, lb);

		/* select proper Mac mode */
		pp2_gop_xlg_2_gig_mac_cfg(gop, mac_num);

		/* pcs unreset */
		pp2_gop_gpcs_reset(gop, mac_num, UNRESET);

		/* mac unreset */
		pp2_gop_gmac_reset(gop, mac_num, UNRESET);
		pp2_gop_force_link_mode_set(gop, mac, false, false);
		break;
	case PHY_INTERFACE_MODE_XAUI:
		num_of_act_lanes = 4;
		mac_num = 0;
		/* configure PCS */
		pp2_gop_xpcs_mode(gop, num_of_act_lanes);
		/* configure MAC */
		pp2_gop_xlg_mac_mode_cfg(gop, mac_num, num_of_act_lanes);

		/* pcs unreset */
		pp2_gop_xpcs_reset(gop, UNRESET);
		/* mac unreset */
		pp2_gop_xlg_mac_reset(gop, mac_num, UNRESET);
		break;
	case PHY_INTERFACE_MODE_RXAUI:
		num_of_act_lanes = 2;
		/* mapped to serdes 6 */
		pp2_gop_serdes_init(gop, 0, PP2_RXAUI);
		/* mapped to serdes 5 */
		pp2_gop_serdes_init(gop, 1, PP2_RXAUI);

		mac_num = 0;
		/* configure PCS */
		pp2_gop_xpcs_mode(gop, num_of_act_lanes);
		/* configure MAC */
		pp2_gop_xlg_mac_mode_cfg(gop, mac_num, num_of_act_lanes);

		/* pcs unreset */
		pp2_gop_xpcs_reset(gop, UNRESET);

		/* mac unreset */
		pp2_gop_xlg_mac_reset(gop, mac_num, UNRESET);

		/* run digital reset / unreset */
		pp2_gop_serdes_reset(gop, 0, false, false, true);
		pp2_gop_serdes_reset(gop, 1, false, false, true);
		pp2_gop_serdes_reset(gop, 0, false, false, false);
		pp2_gop_serdes_reset(gop, 1, false, false, false);
		break;
	case PHY_INTERFACE_MODE_KR:

		num_of_act_lanes = 2;
		mac_num = 0;
		/* configure PCS */
		pp2_gop_xpcs_mode(gop, num_of_act_lanes);
		pp2_gop_mpcs_mode(gop);
		/* configure MAC */
		pp2_gop_xlg_mac_mode_cfg(gop, mac_num, num_of_act_lanes);

        /* Enable or disable MAC loopback */
		pp2_gop_xlg_mac_loopback_cfg(gop, mac_num, lb ? PP2_TX_2_RX_LB : PP2_DISABLE_LB);

		/* pcs unreset */
		pp2_gop_xpcs_reset(gop, UNRESET);

		/* mac unreset */
		pp2_gop_xlg_mac_reset(gop, mac_num, UNRESET);
		break;
	default:
		pp2_err("%s: Requested port mode (%d) not supported",
		       __func__, mac->phy_mode);
		return -1;
	}

	GOP_DEBUG(gop->gop_port_debug[mac_num].flags = (1 << CREATED));

	return 0;
}

/**************************************************************************
* pp2_port_reset
*
* DESCRIPTION:
*       Clears the port mode and release all its resources
*       according to selected.
*       Does not verify that the selected mode/port number is valid at the core
*       level and actual terminated mode.
*
* INPUTS:
*       port_num   - physical port number
*       port_mode  - port standard metric
*       action     - Power down or reset
*
* OUTPUTS:
*       None.
*
* RETURNS:
*       0  - on success
*       1  - on error
*
**************************************************************************/
int pp2_gop_port_reset(struct gop_hw *gop, struct pp2_mac_data *mac)
{
	int mac_num = mac->gop_index;

	switch (mac->phy_mode) {
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_QSGMII:
		/* pcs unreset */
		pp2_gop_gpcs_reset(gop, mac_num, RESET);
		/* mac unreset */
		pp2_gop_gmac_reset(gop, mac_num, RESET);
		break;
	case PHY_INTERFACE_MODE_XAUI:
		/* pcs unreset */
		pp2_gop_xpcs_reset(gop, RESET);
		/* mac unreset */
		pp2_gop_xlg_mac_reset(gop, mac_num, RESET);
		break;
	case PHY_INTERFACE_MODE_RXAUI:
		/* pcs unreset */
		pp2_gop_xpcs_reset(gop, RESET);
		/* mac unreset */
		pp2_gop_xlg_mac_reset(gop, mac_num, RESET);
		break;
		/* Stefan: need to check KR case */
	case PHY_INTERFACE_MODE_KR:
		/* pcs unreset */
		pp2_gop_xpcs_reset(gop, RESET);
		/* mac unreset */
		pp2_gop_xlg_mac_reset(gop, mac_num, RESET);
		break;
	default:
		pp2_err("%s: Wrong port mode (%d)", __func__, mac->phy_mode);
		return -1;
	}

	/* TBD:serdes reset or power down if needed */

	return 0;
}

/*-------------------------------------------------------------------*/
void pp2_gop_port_enable(struct gop_hw *gop, struct pp2_mac_data *mac)
{
	int port_num = mac->gop_index;

	switch (mac->phy_mode) {
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_QSGMII:
		pp2_gop_gmac_port_enable(gop, port_num);
		pp2_gop_force_link_mode_set(gop, mac, false, false);
		pp2_gop_gmac_reset(gop, port_num, UNRESET);
		break;
	case PHY_INTERFACE_MODE_XAUI:
	case PHY_INTERFACE_MODE_RXAUI:
	case PHY_INTERFACE_MODE_KR:
		pp2_gop_xlg_mac_port_enable(gop, port_num);
		break;
	default:
		pp2_err("%s: Wrong port mode (%d)", __func__, mac->phy_mode);
		return;
	}

	GOP_DEBUG(gop->gop_port_debug[port_num].flags |= (1 << ENABLED));
}

void pp2_gop_port_disable(struct gop_hw *gop, struct pp2_mac_data *mac)
{
	int port_num = mac->gop_index;

	switch (mac->phy_mode) {
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_QSGMII:
		pp2_gop_gmac_port_disable(gop, port_num);
		pp2_gop_force_link_mode_set(gop, mac, false, true);
		pp2_gop_gmac_reset(gop, port_num, RESET);
		break;
	case PHY_INTERFACE_MODE_XAUI:
	case PHY_INTERFACE_MODE_RXAUI:
	case PHY_INTERFACE_MODE_KR:
		pp2_gop_xlg_mac_port_disable(gop, port_num);
		break;
	default:
		pp2_err("%s: Wrong port mode (%d)", __func__, mac->phy_mode);
		return;
	}

	GOP_DEBUG(gop->gop_port_debug[port_num].flags &= ~(1 << ENABLED));
}

void pp2_gop_port_periodic_xon_set(struct gop_hw *gop,
				  struct pp2_mac_data *mac, int enable)
{
	int port_num = mac->gop_index;

	switch (mac->phy_mode) {
	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_QSGMII:
		pp2_gop_gmac_port_periodic_xon_set(gop, port_num, enable);
		break;
	case PHY_INTERFACE_MODE_XAUI:
	case PHY_INTERFACE_MODE_RXAUI:
	case PHY_INTERFACE_MODE_KR:
		pp2_gop_xlg_mac_port_periodic_xon_set(gop, port_num, enable);
		break;
	default:
		pp2_err("%s: Wrong port mode (%d)", __func__, mac->phy_mode);
		return;
	}
}

bool pp2_gop_port_is_link_up(struct gop_hw *gop, struct pp2_mac_data *mac)
{
	int port_num = mac->gop_index;

	switch (mac->phy_mode) {
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_QSGMII:
		return pp2_gop_gmac_link_status_get(gop, port_num);
	case PHY_INTERFACE_MODE_XAUI:
	case PHY_INTERFACE_MODE_RXAUI:
	case PHY_INTERFACE_MODE_KR:
		usleep(1000);
		return pp2_gop_xlg_mac_link_status_get(gop, port_num);
	default:
		pp2_err("%s: Wrong port mode gop_port(%d), phy_mode(%d)",
		       __func__, port_num, mac->phy_mode);
		return false;
	}
}

int pp2_gop_port_link_status(struct gop_hw *gop, struct pp2_mac_data *mac,
			    struct pp2_port_link_status *pstatus)
{
	int port_num = mac->gop_index;

	switch (mac->phy_mode) {
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_QSGMII:
		pp2_gop_gmac_link_status(gop, port_num, pstatus);
		break;
	case PHY_INTERFACE_MODE_XAUI:
	case PHY_INTERFACE_MODE_RXAUI:
	case PHY_INTERFACE_MODE_KR:
		pp2_gop_xlg_mac_link_status(gop, port_num, pstatus);
		break;
	default:
		pp2_err("%s: Wrong port mode (%d)", __func__, mac->phy_mode);
		return -1;
	}
	return 0;
}

int pp2_gop_port_events_mask(struct gop_hw *gop, struct pp2_mac_data *mac)
{
	int port_num = mac->gop_index;

	switch (mac->phy_mode) {
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_QSGMII:
		pp2_gop_gmac_port_link_event_mask(gop, port_num);
		break;
	case PHY_INTERFACE_MODE_XAUI:
	case PHY_INTERFACE_MODE_RXAUI:
	case PHY_INTERFACE_MODE_KR:
		pp2_gop_xlg_port_link_event_mask(gop, port_num);
		break;
	default:
		pp2_err("%s: Wrong port mode (%d)", __func__, mac->phy_mode);
		return -1;
	}
	return 0;
}

int pp2_gop_port_events_unmask(struct gop_hw *gop, struct pp2_mac_data *mac)
{
	int port_num = mac->gop_index;

	switch (mac->phy_mode) {
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_QSGMII:
		pp2_gop_gmac_port_link_event_unmask(gop, port_num);
		/* gige interrupt cause connected to CPU via XLG
		 * external interrupt cause
		 */
		pp2_gop_xlg_port_external_event_unmask(gop, 0, 2);
		break;
	case PHY_INTERFACE_MODE_XAUI:
	case PHY_INTERFACE_MODE_RXAUI:
	case PHY_INTERFACE_MODE_KR:
		pp2_gop_xlg_port_external_event_unmask(gop, port_num, 1);
		break;
	default:
		pp2_err("%s: Wrong port mode (%d)", __func__, mac->phy_mode);
		return -1;
	}
	return 0;
}

int pp2_gop_port_events_clear(struct gop_hw *gop, struct pp2_mac_data *mac)
{
	int port_num = mac->gop_index;

	switch (mac->phy_mode) {
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_QSGMII:
		pp2_gop_gmac_port_link_event_clear(gop, port_num);
		break;
	case PHY_INTERFACE_MODE_XAUI:
	case PHY_INTERFACE_MODE_RXAUI:
	case PHY_INTERFACE_MODE_KR:
		pp2_gop_xlg_port_link_event_clear(gop, port_num);
		break;
	default:
		pp2_err("%s: Wrong port mode (%d)", __func__, mac->phy_mode);
		return -1;
	}
	return 0;
}

int pp2_gop_status_show(struct gop_hw *gop, struct pp2_mac_data *mac)
{
	int port_num = mac->gop_index;
	struct pp2_port_link_status port_status;
	bool port_en = false;

	pp2_gop_port_link_status(gop, mac, &port_status);
	port_en = (gop->gop_port_debug[port_num].flags &
		   (1 << ENABLED)) ? true : false;
	pp2_info("---- GOP ID %d configuration ----\n", port_num);
	switch (mac->phy_mode) {
	case PHY_INTERFACE_MODE_RGMII:
		pp2_info("Port mode               : RGMII\n");
		break;
	case PHY_INTERFACE_MODE_SGMII:
		pp2_info("Port mode               : SGMII\n");
		break;
	case PHY_INTERFACE_MODE_QSGMII:
		pp2_info("Port mode               : QSGMII\n");
		break;
	case PHY_INTERFACE_MODE_XAUI:
		pp2_info("Port mode               : XAUI\n");
		break;
	case PHY_INTERFACE_MODE_RXAUI:
		pp2_info("Port mode               : RXAUI\n");
		break;
	case PHY_INTERFACE_MODE_KR:
		pp2_info("Port mode               : KR\n");
		break;
	default:
		pp2_err("%s: Wrong port mode (%d)\n", __func__, mac->phy_mode);
		return -1;
	}

	pp2_info("MAC status              : %s",
		(port_en) ? "enabled\n" : "disabled\n");
	pp2_info("Link status             : %s",
		(port_status.linkup) ? "link up\n" : "link down\n");

	if ((mac->phy_mode == PHY_INTERFACE_MODE_SGMII) &&
	    (mac->speed == 2500) && (port_status.speed == PP2_PORT_SPEED_1000))
		port_status.speed = PP2_PORT_SPEED_2500;

	switch (port_status.speed) {
	case PP2_PORT_SPEED_AN:
		pp2_info("Port speed              : AutoNeg\n");
		break;
	case PP2_PORT_SPEED_10:
		pp2_info("Port speed              : 10M\n");
		break;
	case PP2_PORT_SPEED_100:
		pp2_info("Port speed              : 100M\n");
		break;
	case PP2_PORT_SPEED_1000:
		pp2_info("Port speed              : 1G\n");
		break;
	case PP2_PORT_SPEED_2500:
		pp2_info("Port speed              : 2.5G\n");
		break;
	case PP2_PORT_SPEED_10000:
		pp2_info("Port speed              : 10G\n");
		break;
	default:
		pp2_err("%s: Wrong port speed (%d)\n", __func__,
		       port_status.speed);
		return -1;
	}

	switch (port_status.duplex) {
	case PP2_PORT_DUPLEX_AN:
		pp2_info("Port duplex             : AutoNeg\n");
		break;
	case PP2_PORT_DUPLEX_HALF:
		pp2_info("Port duplex             : half\n");
		break;
	case PP2_PORT_DUPLEX_FULL:
		pp2_info("Port duplex             : full\n");
		break;
	default:
		pp2_err("%s: Wrong port duplex (%d)\n", __func__,
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
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_QSGMII:
		pp2_gop_gmac_speed_duplex_get(gop, port_num, speed, duplex);
		break;
	case PHY_INTERFACE_MODE_XAUI:
	case PHY_INTERFACE_MODE_RXAUI:
	case PHY_INTERFACE_MODE_KR:
		pp2_gop_xlg_mac_speed_duplex_get(gop, port_num, speed, duplex);
		break;
	default:
		pp2_err("%s: Wrong port mode (%d)", __func__, mac->phy_mode);
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
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_QSGMII:
		pp2_gop_gmac_speed_duplex_set(gop, port_num, speed, duplex);
		break;
	case PHY_INTERFACE_MODE_XAUI:
	case PHY_INTERFACE_MODE_RXAUI:
	case PHY_INTERFACE_MODE_KR:
		pp2_gop_xlg_mac_speed_duplex_set(gop, port_num, speed, duplex);
		break;
	default:
		pp2_err("%s: Wrong port mode (%d)", __func__, mac->phy_mode);
		return -1;
	}
	return 0;
}

int pp2_gop_autoneg_restart(struct gop_hw *gop, struct pp2_mac_data *mac)
{
	int port_num = mac->gop_index;

	switch (mac->phy_mode) {
	case PHY_INTERFACE_MODE_RGMII:
		break;
	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_QSGMII:
		pp2_gop_gmac_port_autoneg_restart(gop, port_num);
		break;
	case PHY_INTERFACE_MODE_XAUI:
	case PHY_INTERFACE_MODE_RXAUI:
	case PHY_INTERFACE_MODE_KR:
		pp2_err("%s: on supported for port mode (%d)", __func__,
		       mac->phy_mode);
		return -1;
	default:
		pp2_err("%s: Wrong port mode (%d)", __func__, mac->phy_mode);
		return -1;
	}
	return 0;
}

int pp2_gop_fl_cfg(struct gop_hw *gop, struct pp2_mac_data *mac)
{
	int port_num = mac->gop_index;

	switch (mac->phy_mode) {
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_QSGMII:
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
	case PHY_INTERFACE_MODE_XAUI:
	case PHY_INTERFACE_MODE_RXAUI:
	case PHY_INTERFACE_MODE_KR:
		return 0;
	default:
		pp2_err("%s: Wrong port mode (%d)", __func__, mac->phy_mode);
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
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_QSGMII:
		/* force link */
		pp2_gop_gmac_force_link_mode_set(gop, port_num,
						force_link_up, force_link_down);
		break;
	case PHY_INTERFACE_MODE_XAUI:
	case PHY_INTERFACE_MODE_RXAUI:
	case PHY_INTERFACE_MODE_KR:
		return 0;
	default:
		pp2_err("%s: Wrong port mode (%d)", __func__, mac->phy_mode);
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
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_QSGMII:
		return pp2_gop_gmac_force_link_mode_get(gop, port_num,
						       force_link_up,
						       force_link_down);
		break;
	case PHY_INTERFACE_MODE_XAUI:
	case PHY_INTERFACE_MODE_RXAUI:
	case PHY_INTERFACE_MODE_KR:
		return 0;
	default:
		pp2_err("%s: Wrong port mode (%d)", __func__, mac->phy_mode);
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
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_QSGMII:
		/* set loopback */
		if (lb)
			type = PP2_TX_2_RX_LB;
		else
			type = PP2_DISABLE_LB;

		pp2_gop_gmac_loopback_cfg(gop, port_num, type);
		break;
	case PHY_INTERFACE_MODE_XAUI:
	case PHY_INTERFACE_MODE_RXAUI:
	case PHY_INTERFACE_MODE_KR:
		return 0;
	default:
		pp2_err("%s: Wrong port mode (%d)", __func__, mac->phy_mode);
		return -1;
	}
	return 0;
}

/**************************************************************************
* pp2_gop_gpcs_mode_cfg
*
* DESCRIPTION:
	Configure port to working with Gig PCS or don't.
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
	uint32_t val;

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
	uint32_t reg_data;

	reg_data = pp2_gop_gmac_read(gop, pcs_num, PP2_GMAC_PORT_CTRL2_REG);
	if (act == RESET)
		U32_SET_FIELD(reg_data, PP2_GMAC_PORT_CTRL2_SGMII_MODE_MASK, 0);
	else
		U32_SET_FIELD(reg_data, PP2_GMAC_PORT_CTRL2_SGMII_MODE_MASK,
			      1 << PP2_GMAC_PORT_CTRL2_SGMII_MODE_OFFS);

	pp2_gop_gmac_write(gop, pcs_num, PP2_GMAC_PORT_CTRL2_REG, reg_data);
	return 0;
}

void pp2_gop_serdes_init(struct gop_hw *gop, int lane, enum sd_media_mode mode)
{
	uint32_t reg_val;

	/* Media Interface Mode */
	reg_val = pp2_gop_serdes_read(gop, lane, PP2_SERDES_CFG_0_REG);
	if (mode == PP2_RXAUI)
		reg_val |= PP2_SERDES_CFG_0_MEDIA_MODE_MASK;
	else
		reg_val &= ~PP2_SERDES_CFG_0_MEDIA_MODE_MASK;

	/* Pull-Up PLL to StandAlone mode */
	reg_val |= PP2_SERDES_CFG_0_PU_PLL_MASK;
	/* powers up the SD Rx/Tx PLL */
	reg_val |= PP2_SERDES_CFG_0_RX_PLL_MASK;
	reg_val |= PP2_SERDES_CFG_0_TX_PLL_MASK;
	pp2_gop_serdes_write(gop, lane, PP2_SERDES_CFG_0_REG, reg_val);

	pp2_gop_serdes_reset(gop, lane, false, false, false);

	reg_val = 0x17f;
	pp2_gop_serdes_write(gop, lane, PP2_SERDES_MISC_REG, reg_val);
}

void pp2_gop_serdes_reset(struct gop_hw *gop, int lane, bool analog_reset,
			 bool core_reset, bool digital_reset)
{
	uint32_t reg_val;

	reg_val = pp2_gop_serdes_read(gop, lane, PP2_SERDES_CFG_1_REG);
	if (analog_reset)
		reg_val &= ~PP2_SERDES_CFG_1_ANALOG_RESET_MASK;
	else
		reg_val |= PP2_SERDES_CFG_1_ANALOG_RESET_MASK;

	if (core_reset)
		reg_val &= ~PP2_SERDES_CFG_1_CORE_RESET_MASK;
	else
		reg_val |= PP2_SERDES_CFG_1_CORE_RESET_MASK;

	if (digital_reset)
		reg_val &= ~PP2_SERDES_CFG_1_DIGITAL_RESET_MASK;
	else
		reg_val |= PP2_SERDES_CFG_1_DIGITAL_RESET_MASK;

	pp2_gop_serdes_write(gop, lane, PP2_SERDES_CFG_1_REG, reg_val);
}

/**************************************************************************
* pp2_gop_smi_init
**************************************************************************/
int pp2_gop_smi_init(struct gop_hw *gop)
{
	uint32_t val;

	/* not invert MDC */
	val = pp2_gop_smi_read(gop, PP2_SMI_MISC_CFG_REG);
	val &= ~PP2_SMI_MISC_CFG_INVERT_MDC_MASK;
	pp2_gop_smi_write(gop, PP2_SMI_MISC_CFG_REG, val);

	return 0;
}

/**************************************************************************
* pp2_gop_phy_addr_cfg
**************************************************************************/
int pp2_gop_smi_phy_addr_cfg(struct gop_hw *gop, int port, int addr)
{
	pp2_gop_smi_write(gop, PP2_SMI_PHY_ADDRESS_REG(port), addr);

	return 0;
}

/* Set the MAC to reset or exit from reset */
int pp2_gop_xlg_mac_reset(struct gop_hw *gop, int mac_num, enum pp2_reset reset)
{
	uint32_t reg_addr;
	uint32_t val;

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
	uint32_t reg_addr;
	uint32_t val;

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
	uint32_t reg_addr;
	uint32_t val;

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
	uint32_t reg_val;

	reg_val = pp2_gop_xlg_mac_read(gop, mac_num, PP2_XLG_PORT_MAC_CTRL0_REG);
	reg_val |= PP2_XLG_MAC_CTRL0_PORTEN_MASK;
	reg_val &= ~PP2_XLG_MAC_CTRL0_MIBCNTDIS_MASK;

	pp2_gop_xlg_mac_write(gop, mac_num, PP2_XLG_PORT_MAC_CTRL0_REG, reg_val);
}

/* Disable port */
void pp2_gop_xlg_mac_port_disable(struct gop_hw *gop, int mac_num)
{
	uint32_t reg_val;

	/* mask all port interrupts */
	pp2_gop_xlg_port_link_event_mask(gop, mac_num);

	reg_val = pp2_gop_xlg_mac_read(gop, mac_num, PP2_XLG_PORT_MAC_CTRL0_REG);
	reg_val &= ~PP2_XLG_MAC_CTRL0_PORTEN_MASK;

	pp2_gop_xlg_mac_write(gop, mac_num, PP2_XLG_PORT_MAC_CTRL0_REG, reg_val);
}

void pp2_gop_xlg_mac_port_periodic_xon_set(struct gop_hw *gop,
					  int mac_num, int enable)
{
	uint32_t reg_val;

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
	uint32_t reg_val;
	uint32_t mac_mode;
	uint32_t fc_en;

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
	uint32_t reg_val;

	reg_val = pp2_gop_xlg_mac_read(gop, mac_num, PP2_XLG_PORT_MAC_CTRL1_REG);
	reg_val &= ~PP2_XLG_MAC_CTRL1_FRAMESIZELIMIT_MASK;
	reg_val |= (((max_rx_size - PP2_MH_SIZE) / 2) <<
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
	uint32_t reg_val;

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
	uint32_t reg_val;

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
		pp2_err("XLG MAC: Unexpected FlowControl value %d\n", fc);
		return -EINVAL;
	}

	pp2_gop_xlg_mac_write(gop, mac_num, PP2_XLG_PORT_MAC_CTRL0_REG, reg_val);
	return 0;
}

/* Get Flow Control configuration of the port */
void pp2_gop_xlg_mac_fc_get(struct gop_hw *gop, int mac_num, enum pp2_port_fc *fc)
{
	uint32_t reg_val;

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
			pp2_err("pp2_gop_xlg_mac_fc_set failed\n");
			return -EPERM;
		}
		if (pp2_gop_xlg_mac_force_link_mode_set(gop, mac_num, 1, 0)) {
			pp2_err("pp2_gop_xlg_mac_force_link_mode_set failed\n");
			return -EPERM;
		}
	} else {
		if (pp2_gop_xlg_mac_force_link_mode_set(gop, mac_num, 0, 0)) {
			pp2_err("pp2_gop_xlg_mac_force_link_mode_set failed\n");
			return -EPERM;
		}
		if (pp2_gop_xlg_mac_fc_set(gop, mac_num, PP2_PORT_FC_AN_SYM)) {
			pp2_err("pp2_gop_xlg_mac_fc_set failed\n");
			return -EPERM;
		}
	}

	return 0;
}

void pp2_gop_xlg_port_link_event_mask(struct gop_hw *gop, int mac_num)
{
	uint32_t reg_val;

	reg_val = pp2_gop_xlg_mac_read(gop, mac_num,
				      PP2_XLG_EXTERNAL_INTERRUPT_MASK_REG);
	reg_val &= ~(1 << 1);
	pp2_gop_xlg_mac_write(gop, mac_num,
			     PP2_XLG_EXTERNAL_INTERRUPT_MASK_REG, reg_val);
}

void pp2_gop_xlg_port_external_event_unmask(struct gop_hw *gop, int mac_num,
					   int bit_2_open)
{
	uint32_t reg_val;

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
	uint32_t reg_val;

	/* relevant only for MAC0 (XLG0 and GMAC0) */
	if (mac_num > 0)
		return;

	/* configure 1Gig MAC mode */
	reg_val = pp2_gop_xlg_mac_read(gop, mac_num, PP2_XLG_PORT_MAC_CTRL3_REG);
	U32_SET_FIELD(reg_val, PP2_XLG_MAC_CTRL3_MACMODESELECT_MASK,
		      (0 << PP2_XLG_MAC_CTRL3_MACMODESELECT_OFFS));
	pp2_gop_xlg_mac_write(gop, mac_num, PP2_XLG_PORT_MAC_CTRL3_REG, reg_val);
}

/* Set PCS to reset or exit from reset */
int pp2_gop_xpcs_reset(struct gop_hw *gop, enum pp2_reset reset)
{
	uint32_t reg_addr;
	uint32_t val;

	reg_addr = PP2_XPCS_GLOBAL_CFG_0_REG;

	/* read - modify - write */
	val = pp2_gop_xpcs_global_read(gop, reg_addr);
	if (reset == RESET)
		val &= ~PP2_XPCS_GLOBAL_CFG_0_PCSRESET_MASK;
	else
		val |= PP2_XPCS_GLOBAL_CFG_0_PCSRESET_MASK;
	pp2_gop_xpcs_global_write(gop, reg_addr, val);

	return 0;
}

/* Set the internal mux's to the required PCS in the PI */
int pp2_gop_xpcs_mode(struct gop_hw *gop, int num_of_lanes)
{
	uint32_t reg_addr;
	uint32_t val;
	int lane;

	switch (num_of_lanes) {
	case 1:
		lane = 0;
		break;
	case 2:
		lane = 1;
		break;
	case 4:
		lane = 2;
		break;
	default:
		return -1;
	}

	/* configure XG MAC mode */
	reg_addr = PP2_XPCS_GLOBAL_CFG_0_REG;
	val = pp2_gop_xpcs_global_read(gop, reg_addr);
	val &= ~PP2_XPCS_GLOBAL_CFG_0_PCSMODE_MASK;
	U32_SET_FIELD(val, PP2_XPCS_GLOBAL_CFG_0_PCSMODE_MASK, 0);
	U32_SET_FIELD(val, PP2_XPCS_GLOBAL_CFG_0_LANEACTIVE_MASK, (2 * lane) <<
		      PP2_XPCS_GLOBAL_CFG_0_LANEACTIVE_OFFS);
	pp2_gop_xpcs_global_write(gop, reg_addr, val);

	return 0;
}

int pp2_gop_mpcs_mode(struct gop_hw *gop)
{
	uint32_t reg_addr;
	uint32_t val;

	/* configure PCS40G COMMON CONTROL */
	reg_addr = PCS40G_COMMON_CONTROL;
	val = pp2_gop_mpcs_global_read(gop, reg_addr);
	U32_SET_FIELD(val, FORWARD_ERROR_CORRECTION_MASK,
		      0 << FORWARD_ERROR_CORRECTION_OFFSET);

	pp2_gop_mpcs_global_write(gop, reg_addr, val);

	/* configure PCS CLOCK RESET */
	reg_addr = PCS_CLOCK_RESET;
	val = pp2_gop_mpcs_global_read(gop, reg_addr);
	U32_SET_FIELD(val, CLK_DIVISION_RATIO_MASK,
		      1 << CLK_DIVISION_RATIO_OFFSET);

	pp2_gop_mpcs_global_write(gop, reg_addr, val);

	U32_SET_FIELD(val, CLK_DIV_PHASE_SET_MASK,
		      0 << CLK_DIV_PHASE_SET_OFFSET);
	U32_SET_FIELD(val, MAC_CLK_RESET_MASK, 1 << MAC_CLK_RESET_OFFSET);
	U32_SET_FIELD(val, RX_SD_CLK_RESET_MASK, 1 << RX_SD_CLK_RESET_OFFSET);
	U32_SET_FIELD(val, TX_SD_CLK_RESET_MASK, 1 << TX_SD_CLK_RESET_OFFSET);

	pp2_gop_mpcs_global_write(gop, reg_addr, val);

	return 0;
}

void pp2_gop_ptp_enable(struct gop_hw *gop, int port, bool state)
{
	uint32_t reg_data;

	if (state) {
		/* PTP enable */
		reg_data = pp2_gop_ptp_read(gop, port, PP2_PTP_GENERAL_CTRL_REG);
		reg_data |= PP2_PTP_GENERAL_CTRL_PTP_UNIT_ENABLE_MASK;
		/* enable PTP */
		pp2_gop_ptp_write(gop, port, PP2_PTP_GENERAL_CTRL_REG, reg_data);
		/* unreset unit */
		reg_data |= PP2_PTP_GENERAL_CTRL_PTP_RESET_MASK;
		pp2_gop_ptp_write(gop, port, PP2_PTP_GENERAL_CTRL_REG, reg_data);
	} else {
		reg_data = pp2_gop_ptp_read(gop, port, PP2_PTP_GENERAL_CTRL_REG);
		reg_data &= ~PP2_PTP_GENERAL_CTRL_PTP_UNIT_ENABLE_MASK;
		/* disable PTP */
		pp2_gop_ptp_write(gop, port, PP2_PTP_GENERAL_CTRL_REG, reg_data);
	}
}

uint32_t pp2_gop_netc_cfg_create(struct pp2_mac_data *mac)
{
	uint32_t val = 0;

	if (mac->gop_index == 0) {
		if (mac->phy_mode == PHY_INTERFACE_MODE_XAUI)
			val |= PP2_NETC_GE_MAC0_XAUI;
		else if (mac->phy_mode == PHY_INTERFACE_MODE_RXAUI)
			val |= PP2_NETC_GE_MAC0_RXAUI_L23;
	}
	if (mac->gop_index == 2) {
		if (mac->phy_mode == PHY_INTERFACE_MODE_SGMII)
			val |= PP2_NETC_GE_MAC2_SGMII;
	}
	if (mac->gop_index == 3) {
		if (mac->phy_mode == PHY_INTERFACE_MODE_SGMII)
			val |= PP2_NETC_GE_MAC3_SGMII;
		else if (mac->phy_mode == PHY_INTERFACE_MODE_RGMII)
			val |= PP2_NETC_GE_MAC3_RGMII;
	}

	return val;
}

void pp2_gop_netc_active_port(struct gop_hw *gop, uint32_t port, uint32_t val)
{
	uint32_t reg;

	reg = pp2_gop_rfu1_read(gop, PP2_NETCOMP_PORTS_CONTROL_1);
	reg &= ~(NETC_PORTS_ACTIVE_MASK(port));

	val <<= NETC_PORTS_ACTIVE_OFFSET(port);
	val &= NETC_PORTS_ACTIVE_MASK(port);

	reg |= val;

	pp2_gop_rfu1_write(gop, PP2_NETCOMP_PORTS_CONTROL_1, reg);
}

static void pp2_gop_netc_xaui_enable(struct gop_hw *gop, uint32_t port,
				    uint32_t val)
{
	uint32_t reg;

	reg = pp2_gop_rfu1_read(gop, SD1_CONTROL_1_REG);
	reg &= ~SD1_CONTROL_XAUI_EN_MASK;

	val <<= SD1_CONTROL_XAUI_EN_OFFSET;
	val &= SD1_CONTROL_XAUI_EN_MASK;

	reg |= val;

	pp2_gop_rfu1_write(gop, SD1_CONTROL_1_REG, reg);
}

static void pp2_gop_netc_rxaui0_enable(struct gop_hw *gop, uint32_t port,
				      uint32_t val)
{
	uint32_t reg;

	reg = pp2_gop_rfu1_read(gop, SD1_CONTROL_1_REG);
	reg &= ~SD1_CONTROL_RXAUI0_L23_EN_MASK;

	val <<= SD1_CONTROL_RXAUI0_L23_EN_OFFSET;
	val &= SD1_CONTROL_RXAUI0_L23_EN_MASK;

	reg |= val;

	pp2_gop_rfu1_write(gop, SD1_CONTROL_1_REG, reg);
}

static void pp2_gop_netc_rxaui1_enable(struct gop_hw *gop, uint32_t port,
				      uint32_t val)
{
	uint32_t reg;

	reg = pp2_gop_rfu1_read(gop, SD1_CONTROL_1_REG);
	reg &= ~SD1_CONTROL_RXAUI1_L45_EN_MASK;

	val <<= SD1_CONTROL_RXAUI1_L45_EN_OFFSET;
	val &= SD1_CONTROL_RXAUI1_L45_EN_MASK;

	reg |= val;

	pp2_gop_rfu1_write(gop, SD1_CONTROL_1_REG, reg);
}

static void pp2_gop_netc_mii_mode(struct gop_hw *gop, uint32_t port,
				 uint32_t val)
{
	uint32_t reg;

	reg = pp2_gop_rfu1_read(gop, PP2_NETCOMP_CONTROL_0);
	reg &= ~NETC_GBE_PORT1_MII_MODE_MASK;

	val <<= NETC_GBE_PORT1_MII_MODE_OFFSET;
	val &= NETC_GBE_PORT1_MII_MODE_MASK;

	reg |= val;

	pp2_gop_rfu1_write(gop, PP2_NETCOMP_CONTROL_0, reg);
}

static void pp2_gop_netc_reset(struct gop_hw *gop, uint32_t val)
{
	uint32_t reg;

	reg = pp2_gop_rfu1_read(gop, PP2_GOP_SOFT_RESET_1_REG);
	reg &= ~NETC_GOP_SOFT_RESET_MASK;

	val <<= NETC_GOP_SOFT_RESET_OFFSET;
	val &= NETC_GOP_SOFT_RESET_MASK;

	reg |= val;

	pp2_gop_rfu1_write(gop, PP2_GOP_SOFT_RESET_1_REG, reg);
}

static void pp2_gop_netc_clock_logic_set(struct gop_hw *gop, uint32_t val)
{
	uint32_t reg;

	reg = pp2_gop_rfu1_read(gop, PP2_NETCOMP_PORTS_CONTROL_0);
	reg &= ~NETC_CLK_DIV_PHASE_MASK;

	val <<= NETC_CLK_DIV_PHASE_OFFSET;
	val &= NETC_CLK_DIV_PHASE_MASK;

	reg |= val;

	pp2_gop_rfu1_write(gop, PP2_NETCOMP_PORTS_CONTROL_0, reg);
}

static void pp2_gop_netc_port_rf_reset(struct gop_hw *gop, uint32_t port,
				      uint32_t val)
{
	uint32_t reg;

	reg = pp2_gop_rfu1_read(gop, PP2_NETCOMP_PORTS_CONTROL_1);
	reg &= ~(NETC_PORT_GIG_RF_RESET_MASK(port));

	val <<= NETC_PORT_GIG_RF_RESET_OFFSET(port);
	val &= NETC_PORT_GIG_RF_RESET_MASK(port);

	reg |= val;

	pp2_gop_rfu1_write(gop, PP2_NETCOMP_PORTS_CONTROL_1, reg);
}

static void pp2_gop_netc_gbe_sgmii_mode_select(struct gop_hw *gop, uint32_t port,
					      uint32_t val)
{
	uint32_t reg, mask, offset;

	if (port == 2) {
		mask = NETC_GBE_PORT0_SGMII_MODE_MASK;
		offset = NETC_GBE_PORT0_SGMII_MODE_OFFSET;
	} else {
		mask = NETC_GBE_PORT1_SGMII_MODE_MASK;
		offset = NETC_GBE_PORT1_SGMII_MODE_OFFSET;
	}
	reg = pp2_gop_rfu1_read(gop, PP2_NETCOMP_CONTROL_0);
	reg &= ~mask;

	val <<= offset;
	val &= mask;

	reg |= val;

	pp2_gop_rfu1_write(gop, PP2_NETCOMP_CONTROL_0, reg);
}

static void pp2_gop_netc_bus_width_select(struct gop_hw *gop, uint32_t val)
{
	uint32_t reg;

	reg = pp2_gop_rfu1_read(gop, PP2_NETCOMP_PORTS_CONTROL_0);
	reg &= ~NETC_BUS_WIDTH_SELECT_MASK;

	val <<= NETC_BUS_WIDTH_SELECT_OFFSET;
	val &= NETC_BUS_WIDTH_SELECT_MASK;

	reg |= val;

	pp2_gop_rfu1_write(gop, PP2_NETCOMP_PORTS_CONTROL_0, reg);
}

static void pp2_gop_netc_sample_stages_timing(struct gop_hw *gop, uint32_t val)
{
	uint32_t reg;

	reg = pp2_gop_rfu1_read(gop, PP2_NETCOMP_PORTS_CONTROL_0);
	reg &= ~NETC_GIG_RX_DATA_SAMPLE_MASK;

	val <<= NETC_GIG_RX_DATA_SAMPLE_OFFSET;
	val &= NETC_GIG_RX_DATA_SAMPLE_MASK;

	reg |= val;

	pp2_gop_rfu1_write(gop, PP2_NETCOMP_PORTS_CONTROL_0, reg);
}

static void pp2_gop_netc_mac_to_xgmii(struct gop_hw *gop, uint32_t port,
				     enum pp2_netc_phase phase)
{
	switch (phase) {
	case PP2_NETC_FIRST_PHASE:
		/* Set Bus Width to HB mode = 1 */
		pp2_gop_netc_bus_width_select(gop, 1);
		/* Select RGMII mode */
		pp2_gop_netc_gbe_sgmii_mode_select(gop, port, PP2_NETC_GBE_XMII);
		break;
	case PP2_NETC_SECOND_PHASE:
		/* De-assert the relevant port HB reset */
		pp2_gop_netc_port_rf_reset(gop, port, 1);
		break;
	}
}

static void pp2_gop_netc_mac_to_sgmii(struct gop_hw *gop, uint32_t port,
				     enum pp2_netc_phase phase)
{
	switch (phase) {
	case PP2_NETC_FIRST_PHASE:
		/* Set Bus Width to HB mode = 1 */
		pp2_gop_netc_bus_width_select(gop, 1);
		/* Select SGMII mode */
		if (port >= 1)
			pp2_gop_netc_gbe_sgmii_mode_select(gop, port,
							  PP2_NETC_GBE_SGMII);

		/* Configure the sample stages */
		pp2_gop_netc_sample_stages_timing(gop, 0);
		/* Configure the ComPhy Selector */
		/* pp2_gop_netc_com_phy_selector_config(netComplex); */
		break;
	case PP2_NETC_SECOND_PHASE:
		/* De-assert the relevant port HB reset */
		pp2_gop_netc_port_rf_reset(gop, port, 1);
		break;
	}
}

static void pp2_gop_netc_mac_to_rxaui(struct gop_hw *gop, uint32_t port,
				     enum pp2_netc_phase phase,
				     enum pp2_netc_lanes lanes)
{
	/* Currently only RXAUI0 supported */
	if (port != 0)
		return;

	switch (phase) {
	case PP2_NETC_FIRST_PHASE:
		/* RXAUI Serdes/s Clock alignment */
		if (lanes == PP2_NETC_LANE_23)
			pp2_gop_netc_rxaui0_enable(gop, port, 1);
		else
			pp2_gop_netc_rxaui1_enable(gop, port, 1);
		break;
	case PP2_NETC_SECOND_PHASE:
		/* De-assert the relevant port HB reset */
		pp2_gop_netc_port_rf_reset(gop, port, 1);
		break;
	}
}

static void pp2_gop_netc_mac_to_xaui(struct gop_hw *gop, uint32_t port,
				    enum pp2_netc_phase phase)
{
	switch (phase) {
	case PP2_NETC_FIRST_PHASE:
		/* RXAUI Serdes/s Clock alignment */
		pp2_gop_netc_xaui_enable(gop, port, 1);
		break;
	case PP2_NETC_SECOND_PHASE:
		/* De-assert the relevant port HB reset */
		pp2_gop_netc_port_rf_reset(gop, port, 1);
		break;
	}
}

int pp2_gop_netc_init(struct gop_hw *gop,
		     uint32_t net_comp_config, enum pp2_netc_phase phase)
{
	uint32_t c = net_comp_config;

	if (c & PP2_NETC_GE_MAC0_RXAUI_L23)
		pp2_gop_netc_mac_to_rxaui(gop, 0, phase, PP2_NETC_LANE_23);

	if (c & PP2_NETC_GE_MAC0_RXAUI_L45)
		pp2_gop_netc_mac_to_rxaui(gop, 0, phase, PP2_NETC_LANE_45);

	if (c & PP2_NETC_GE_MAC0_XAUI)
		pp2_gop_netc_mac_to_xaui(gop, 0, phase);

	if (c & PP2_NETC_GE_MAC2_SGMII)
		pp2_gop_netc_mac_to_sgmii(gop, 2, phase);
	else
		pp2_gop_netc_mac_to_xgmii(gop, 2, phase);
	if (c & PP2_NETC_GE_MAC3_SGMII) {
		pp2_gop_netc_mac_to_sgmii(gop, 3, phase);
	} else {
		pp2_gop_netc_mac_to_xgmii(gop, 3, phase);
		if (c & PP2_NETC_GE_MAC3_RGMII)
			pp2_gop_netc_mii_mode(gop, 3, PP2_NETC_GBE_RGMII);
		else
			pp2_gop_netc_mii_mode(gop, 3, PP2_NETC_GBE_MII);
	}

	/* Activate gop ports 0, 2, 3 */
	pp2_gop_netc_active_port(gop, 0, 1);
	pp2_gop_netc_active_port(gop, 2, 1);
	pp2_gop_netc_active_port(gop, 3, 1);

	if (phase == PP2_NETC_SECOND_PHASE) {
		/* Enable the GOP internal clock logic */
		pp2_gop_netc_clock_logic_set(gop, 1);
		/* De-assert GOP unit reset */
		pp2_gop_netc_reset(gop, 1);
	}

	return 0;
}
