#include "pp2_types.h"

#include "pp2_hw_type.h"
#include "pp2_mem.h"
#include "pp2.h"
#include "pp2_gop_def.h"
#include "pp2_gop_dbg.h"
#include "pp2_print.h"

void pp2_gop_register_bases_dump(struct gop_hw *gop)
{
	pp2_info("  %-32s: 0x%lx\n", "GMAC", gop->gmac.base.pa);
	pp2_info("  %-32s: 0x%lx\n", "XLG_MAC", gop->xlg_mac.base.pa);
	pp2_info("  %-32s: 0x%lx\n", "SERDES", gop->serdes.base.pa);
	pp2_info("  %-32s: 0x%lx\n", "XMIB", gop->xmib.base.pa);
	pp2_info("  %-32s: 0x%lx\n", "SMI", gop->smi.pa);
	pp2_info("  %-32s: 0x%lx\n", "XSMI", gop->xsmi.pa);
	pp2_info("  %-32s: 0x%lx\n", "MSPG", gop->mspg.pa);
	pp2_info("  %-32s: 0x%lx\n", "XPCS", gop->xpcs.pa);
	pp2_info("  %-32s: 0x%lx\n", "PTP", gop->ptp.base.pa);
	pp2_info("  %-32s: 0x%lx\n", "RFU1", gop->rfu1.pa);
}

/* print value of unit registers */
void pp2_gop_gmac_regs_dump(struct gop_hw *gop, int port)
{
	char reg_name[32];

	pp2_gop_gmac_print(gop, "PORT_MAC_CTRL0", port, PP2_GMAC_PORT_CTRL0_REG);
	pp2_gop_gmac_print(gop, "PORT_MAC_CTRL1", port, PP2_GMAC_PORT_CTRL1_REG);
	pp2_gop_gmac_print(gop, "PORT_MAC_CTRL2", port, PP2_GMAC_PORT_CTRL2_REG);
	pp2_gop_gmac_print(gop, "PORT_AUTO_NEG_CFG", port,
			  PP2_GMAC_PORT_AUTO_NEG_CFG_REG);
	pp2_gop_gmac_print(gop, "PORT_STATUS0", port, PP2_GMAC_PORT_STATUS0_REG);
	pp2_gop_gmac_print(gop, "PORT_SERIAL_PARAM_CFG", port,
			  PP2_GMAC_PORT_SERIAL_PARAM_CFG_REG);
	pp2_gop_gmac_print(gop, "PORT_FIFO_CFG_0", port,
			  PP2_GMAC_PORT_FIFO_CFG_0_REG);
	pp2_gop_gmac_print(gop, "PORT_FIFO_CFG_1", port,
			  PP2_GMAC_PORT_FIFO_CFG_1_REG);
	pp2_gop_gmac_print(gop, "PORT_SERDES_CFG0", port,
			  PP2_GMAC_PORT_SERDES_CFG0_REG);
	pp2_gop_gmac_print(gop, "PORT_SERDES_CFG1", port,
			  PP2_GMAC_PORT_SERDES_CFG1_REG);
	pp2_gop_gmac_print(gop, "PORT_SERDES_CFG2", port,
			  PP2_GMAC_PORT_SERDES_CFG2_REG);
	pp2_gop_gmac_print(gop, "PORT_SERDES_CFG3", port,
			  PP2_GMAC_PORT_SERDES_CFG3_REG);
	pp2_gop_gmac_print(gop, "PORT_PRBS_STATUS", port,
			  PP2_GMAC_PORT_PRBS_STATUS_REG);
	pp2_gop_gmac_print(gop, "PORT_PRBS_ERR_CNTR", port,
			  PP2_GMAC_PORT_PRBS_ERR_CNTR_REG);
	pp2_gop_gmac_print(gop, "PORT_STATUS1", port, PP2_GMAC_PORT_STATUS1_REG);
	pp2_gop_gmac_print(gop, "PORT_MIB_CNTRS_CTRL", port,
			  PP2_GMAC_PORT_MIB_CNTRS_CTRL_REG);
	pp2_gop_gmac_print(gop, "PORT_MAC_CTRL3", port, PP2_GMAC_PORT_CTRL3_REG);
	pp2_gop_gmac_print(gop, "QSGMII", port, PP2_GMAC_QSGMII_REG);
	pp2_gop_gmac_print(gop, "QSGMII_STATUS", port,
			  PP2_GMAC_QSGMII_STATUS_REG);
	pp2_gop_gmac_print(gop, "QSGMII_PRBS_CNTR", port,
			  PP2_GMAC_QSGMII_PRBS_CNTR_REG);
	for (int ind = 0; ind < 8; ind++) {
		sprintf(reg_name, "CCFC_PORT_SPEED_TIMER%d", ind);
		pp2_gop_gmac_print(gop, reg_name, port,
				  PP2_GMAC_CCFC_PORT_SPEED_TIMER_REG(ind));
	}
	for (int ind = 0; ind < 4; ind++) {
		sprintf(reg_name, "FC_DSA_TAG%d", ind);
		pp2_gop_gmac_print(gop, reg_name, port,
				  PP2_GMAC_FC_DSA_TAG_REG(ind));
	}
	pp2_gop_gmac_print(gop, "LINK_LEVEL_FLOW_CTRL_WIN_REG_0", port,
			  PP2_GMAC_LINK_LEVEL_FLOW_CTRL_WINDOW_REG_0);
	pp2_gop_gmac_print(gop, "LINK_LEVEL_FLOW_CTRL_WIN_REG_1", port,
			  PP2_GMAC_LINK_LEVEL_FLOW_CTRL_WINDOW_REG_1);
	pp2_gop_gmac_print(gop, "PORT_MAC_CTRL4", port, PP2_GMAC_PORT_CTRL4_REG);
	pp2_gop_gmac_print(gop, "PORT_SERIAL_PARAM_1_CFG", port,
			  PP2_GMAC_PORT_SERIAL_PARAM_1_CFG_REG);
	pp2_gop_gmac_print(gop, "LPI_CTRL_0", port, PP2_GMAC_LPI_CTRL_0_REG);
	pp2_gop_gmac_print(gop, "LPI_CTRL_1", port, PP2_GMAC_LPI_CTRL_1_REG);
	pp2_gop_gmac_print(gop, "LPI_CTRL_2", port, PP2_GMAC_LPI_CTRL_2_REG);
	pp2_gop_gmac_print(gop, "LPI_STATUS", port, PP2_GMAC_LPI_STATUS_REG);
	pp2_gop_gmac_print(gop, "LPI_CNTR", port, PP2_GMAC_LPI_CNTR_REG);
	pp2_gop_gmac_print(gop, "PULSE_1_MS_LOW", port,
			  PP2_GMAC_PULSE_1_MS_LOW_REG);
	pp2_gop_gmac_print(gop, "PULSE_1_MS_HIGH", port,
			  PP2_GMAC_PULSE_1_MS_HIGH_REG);
	pp2_gop_gmac_print(gop, "PORT_INT_MASK", port,
			  PP2_GMAC_INTERRUPT_MASK_REG);
	pp2_gop_gmac_print(gop, "INT_SUM_MASK", port,
			  PP2_GMAC_INTERRUPT_SUM_MASK_REG);
}

/* print value of unit registers */
void pp2_gop_xlg_mac_regs_dump(struct gop_hw *gop, int port)
{
	char reg_name[16];

	pp2_gop_xlg_mac_print(gop, "PORT_MAC_CTRL0", port,
			     PP2_XLG_PORT_MAC_CTRL0_REG);
	pp2_gop_xlg_mac_print(gop, "PORT_MAC_CTRL1", port,
			     PP2_XLG_PORT_MAC_CTRL1_REG);
	pp2_gop_xlg_mac_print(gop, "PORT_MAC_CTRL2", port,
			     PP2_XLG_PORT_MAC_CTRL2_REG);
	pp2_gop_xlg_mac_print(gop, "PORT_STATUS", port,
			     PP2_XLG_MAC_PORT_STATUS_REG);
	pp2_gop_xlg_mac_print(gop, "PORT_FIFOS_THRS_CFG", port,
			     PP2_XLG_PORT_FIFOS_THRS_CFG_REG);
	pp2_gop_xlg_mac_print(gop, "PORT_MAC_CTRL3", port,
			     PP2_XLG_PORT_MAC_CTRL3_REG);
	pp2_gop_xlg_mac_print(gop, "PORT_PER_PRIO_FLOW_CTRL_STATUS", port,
			     PP2_XLG_PORT_PER_PRIO_FLOW_CTRL_STATUS_REG);
	pp2_gop_xlg_mac_print(gop, "DEBUG_BUS_STATUS", port,
			     PP2_XLG_DEBUG_BUS_STATUS_REG);
	pp2_gop_xlg_mac_print(gop, "PORT_METAL_FIX", port,
			     PP2_XLG_PORT_METAL_FIX_REG);
	pp2_gop_xlg_mac_print(gop, "XG_MIB_CNTRS_CTRL", port,
			     PP2_XLG_MIB_CNTRS_CTRL_REG);
	for (int timer = 0; timer < 8; timer++) {
		sprintf(reg_name, "CNCCFC_TIMER%d", timer);
		pp2_gop_xlg_mac_print(gop, reg_name, port,
				     PP2_XLG_CNCCFC_TIMERI_REG(timer));
	}
	pp2_gop_xlg_mac_print(gop, "PPFC_CTRL", port, PP2_XLG_MAC_PPFC_CTRL_REG);
	pp2_gop_xlg_mac_print(gop, "FC_DSA_TAG_0", port,
			     PP2_XLG_MAC_FC_DSA_TAG_0_REG);
	pp2_gop_xlg_mac_print(gop, "FC_DSA_TAG_1", port,
			     PP2_XLG_MAC_FC_DSA_TAG_1_REG);
	pp2_gop_xlg_mac_print(gop, "FC_DSA_TAG_2", port,
			     PP2_XLG_MAC_FC_DSA_TAG_2_REG);
	pp2_gop_xlg_mac_print(gop, "FC_DSA_TAG_3", port,
			     PP2_XLG_MAC_FC_DSA_TAG_3_REG);
	pp2_gop_xlg_mac_print(gop, "DIC_BUDGET_COMPENSATION", port,
			     PP2_XLG_MAC_DIC_BUDGET_COMPENSATION_REG);
	pp2_gop_xlg_mac_print(gop, "PORT_MAC_CTRL4", port,
			     PP2_XLG_PORT_MAC_CTRL4_REG);
	pp2_gop_xlg_mac_print(gop, "PORT_MAC_CTRL5", port,
			     PP2_XLG_PORT_MAC_CTRL5_REG);
	pp2_gop_xlg_mac_print(gop, "EXT_CTRL", port, PP2_XLG_MAC_EXT_CTRL_REG);
	pp2_gop_xlg_mac_print(gop, "MACRO_CTRL", port,
			     PP2_XLG_MAC_MACRO_CTRL_REG);
	pp2_gop_xlg_mac_print(gop, "MACRO_CTRL", port,
			     PP2_XLG_MAC_MACRO_CTRL_REG);
	pp2_gop_xlg_mac_print(gop, "PORT_INT_MASK", port,
			     PP2_XLG_INTERRUPT_MASK_REG);
	pp2_gop_xlg_mac_print(gop, "EXTERNAL_INT_MASK", port,
			     PP2_XLG_EXTERNAL_INTERRUPT_MASK_REG);
}

int pp2_gop_port_regs(struct gop_hw *gop, struct pp2_mac_data *mac)
{
	int port_num = mac->gop_index;

	switch (mac->phy_mode) {
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_QSGMII:
		pp2_info("\n[gop GMAC #%d registers]\n", port_num);
		pp2_gop_gmac_regs_dump(gop, port_num);
		break;
	case PHY_INTERFACE_MODE_XAUI:
	case PHY_INTERFACE_MODE_RXAUI:
	case PHY_INTERFACE_MODE_KR:
		pp2_info("\n[gop XLG MAC #%d registers]\n", port_num);
		pp2_gop_xlg_mac_regs_dump(gop, port_num);
		break;
	default:
		pp2_err("%s: Wrong port mode (%d)", __func__, mac->phy_mode);
		return -1;
	}
	return 0;
}

uint64_t pp2_gop_mib_read64(struct gop_hw *gop, int port, unsigned int offset)
{
	uint64_t val, val2;

	val = pp2_gop_xmib_mac_read(gop, port, offset);
	if (offset == PP2_MIB_GOOD_OCTETS_RECEIVED_LOW ||
	    offset == PP2_MIB_GOOD_OCTETS_SENT_LOW) {
		val2 = pp2_gop_xmib_mac_read(gop, port, offset + 4);
		val += (val2 << 32);
	}

	return val;
}

static void pp2_gop_mib_print(struct gop_hw *gop, int port, uint32_t offset,
			    const char *mib_name)
{
	uint64_t val;

	val = pp2_gop_mib_read64(gop, port, offset);
	pp2_info("  %-32s: 0x%02x = %lu\n", mib_name, offset, val);
}

void pp2_gop_mib_counters_show(struct gop_hw *gop, int port)
{
	pp2_info("\n[Rx]\n");
	pp2_gop_mib_print(gop, port, PP2_MIB_GOOD_OCTETS_RECEIVED_LOW,
			 "GOOD_OCTETS_RECEIVED");
	pp2_gop_mib_print(gop, port, PP2_MIB_BAD_OCTETS_RECEIVED,
			 "BAD_OCTETS_RECEIVED");

	pp2_gop_mib_print(gop, port, PP2_MIB_UNICAST_FRAMES_RECEIVED,
			 "UNCAST_FRAMES_RECEIVED");
	pp2_gop_mib_print(gop, port, PP2_MIB_BROADCAST_FRAMES_RECEIVED,
			 "BROADCAST_FRAMES_RECEIVED");
	pp2_gop_mib_print(gop, port, PP2_MIB_MULTICAST_FRAMES_RECEIVED,
			 "MULTICAST_FRAMES_RECEIVED");

	pp2_info("\n[RMON]\n");
	pp2_gop_mib_print(gop, port, PP2_MIB_FRAMES_64_OCTETS,
			 "FRAMES_64_OCTETS");
	pp2_gop_mib_print(gop, port, PP2_MIB_FRAMES_65_TO_127_OCTETS,
			 "FRAMES_65_TO_127_OCTETS");
	pp2_gop_mib_print(gop, port, PP2_MIB_FRAMES_128_TO_255_OCTETS,
			 "FRAMES_128_TO_255_OCTETS");
	pp2_gop_mib_print(gop, port, PP2_MIB_FRAMES_256_TO_511_OCTETS,
			 "FRAMES_256_TO_511_OCTETS");
	pp2_gop_mib_print(gop, port, PP2_MIB_FRAMES_512_TO_1023_OCTETS,
			 "FRAMES_512_TO_1023_OCTETS");
	pp2_gop_mib_print(gop, port, PP2_MIB_FRAMES_1024_TO_MAX_OCTETS,
			 "FRAMES_1024_TO_MAX_OCTETS");

	pp2_info("\n[Tx]\n");
	pp2_gop_mib_print(gop, port, PP2_MIB_GOOD_OCTETS_SENT_LOW,
			 "GOOD_OCTETS_SENT");
	pp2_gop_mib_print(gop, port, PP2_MIB_UNICAST_FRAMES_SENT,
			 "UNICAST_FRAMES_SENT");
	pp2_gop_mib_print(gop, port, PP2_MIB_MULTICAST_FRAMES_SENT,
			 "MULTICAST_FRAMES_SENT");
	pp2_gop_mib_print(gop, port, PP2_MIB_BROADCAST_FRAMES_SENT,
			 "BROADCAST_FRAMES_SENT");
	pp2_gop_mib_print(gop, port, PP2_MIB_CRC_ERRORS_SENT, "CRC_ERRORS_SENT");

	pp2_info("\n[FC control]\n");
	pp2_gop_mib_print(gop, port, PP2_MIB_FC_RECEIVED, "FC_RECEIVED");
	pp2_gop_mib_print(gop, port, PP2_MIB_FC_SENT, "FC_SENT");

	pp2_info("\n[Errors]\n");
	pp2_gop_mib_print(gop, port, PP2_MIB_RX_FIFO_OVERRUN, "RX_FIFO_OVERRUN");
	pp2_gop_mib_print(gop, port, PP2_MIB_UNDERSIZE_RECEIVED,
			 "UNDERSIZE_RECEIVED");
	pp2_gop_mib_print(gop, port, PP2_MIB_FRAGMENTS_RECEIVED,
			 "FRAGMENTS_RECEIVED");
	pp2_gop_mib_print(gop, port, PP2_MIB_OVERSIZE_RECEIVED,
			 "OVERSIZE_RECEIVED");
	pp2_gop_mib_print(gop, port, PP2_MIB_JABBER_RECEIVED, "JABBER_RECEIVED");
	pp2_gop_mib_print(gop, port, PP2_MIB_MAC_RECEIVE_ERROR,
			 "MAC_RECEIVE_ERROR");
	pp2_gop_mib_print(gop, port, PP2_MIB_BAD_CRC_EVENT, "BAD_CRC_EVENT");
	pp2_gop_mib_print(gop, port, PP2_MIB_COLLISION, "COLLISION");
	/* This counter must be read last. Read it clear all the counters */
	pp2_gop_mib_print(gop, port, PP2_MIB_LATE_COLLISION, "LATE_COLLISION");
}

/* print value of unit registers */
void pp2_gop_xpcs_gl_regs_dump(struct gop_hw *gop)
{
	pp2_info("\nXPCS Global registers]\n");
	pp2_gop_xpcs_global_print(gop, "GLOBAL_CFG_0", PP2_XPCS_GLOBAL_CFG_0_REG);
	pp2_gop_xpcs_global_print(gop, "GLOBAL_CFG_1", PP2_XPCS_GLOBAL_CFG_1_REG);
	pp2_gop_xpcs_global_print(gop, "GLOBAL_FIFO_THR_CFG",
				 PP2_XPCS_GLOBAL_FIFO_THR_CFG_REG);
	pp2_gop_xpcs_global_print(gop, "GLOBAL_MAX_IDLE_CNTR",
				 PP2_XPCS_GLOBAL_MAX_IDLE_CNTR_REG);
	pp2_gop_xpcs_global_print(gop, "GLOBAL_STATUS",
				 PP2_XPCS_GLOBAL_STATUS_REG);
	pp2_gop_xpcs_global_print(gop, "GLOBAL_DESKEW_ERR_CNTR",
				 PP2_XPCS_GLOBAL_DESKEW_ERR_CNTR_REG);
	pp2_gop_xpcs_global_print(gop, "TX_PCKTS_CNTR_LSB",
				 PP2_XPCS_TX_PCKTS_CNTR_LSB_REG);
	pp2_gop_xpcs_global_print(gop, "TX_PCKTS_CNTR_MSB",
				 PP2_XPCS_TX_PCKTS_CNTR_MSB_REG);
}

/* print value of unit registers */
void pp2_gop_xpcs_lane_regs_dump(struct gop_hw *gop, int lane)
{
	pp2_info("\nXPCS Lane #%d registers]\n", lane);
	pp2_gop_xpcs_lane_print(gop, "LANE_CFG_0", lane, PP2_XPCS_LANE_CFG_0_REG);
	pp2_gop_xpcs_lane_print(gop, "LANE_CFG_1", lane, PP2_XPCS_LANE_CFG_1_REG);
	pp2_gop_xpcs_lane_print(gop, "LANE_STATUS", lane,
			       PP2_XPCS_LANE_STATUS_REG);
	pp2_gop_xpcs_lane_print(gop, "SYMBOL_ERR_CNTR", lane,
			       PP2_XPCS_SYMBOL_ERR_CNTR_REG);
	pp2_gop_xpcs_lane_print(gop, "DISPARITY_ERR_CNTR", lane,
			       PP2_XPCS_DISPARITY_ERR_CNTR_REG);
	pp2_gop_xpcs_lane_print(gop, "PRBS_ERR_CNTR", lane,
			       PP2_XPCS_PRBS_ERR_CNTR_REG);
	pp2_gop_xpcs_lane_print(gop, "RX_PCKTS_CNTR_LSB", lane,
			       PP2_XPCS_RX_PCKTS_CNTR_LSB_REG);
	pp2_gop_xpcs_lane_print(gop, "RX_PCKTS_CNTR_MSB", lane,
			       PP2_XPCS_RX_PCKTS_CNTR_MSB_REG);
	pp2_gop_xpcs_lane_print(gop, "RX_BAD_PCKTS_CNTR_LSB", lane,
			       PP2_XPCS_RX_BAD_PCKTS_CNTR_LSB_REG);
	pp2_gop_xpcs_lane_print(gop, "RX_BAD_PCKTS_CNTR_MSB", lane,
			       PP2_XPCS_RX_BAD_PCKTS_CNTR_MSB_REG);
	pp2_gop_xpcs_lane_print(gop, "CYCLIC_DATA_0", lane,
			       PP2_XPCS_CYCLIC_DATA_0_REG);
	pp2_gop_xpcs_lane_print(gop, "CYCLIC_DATA_1", lane,
			       PP2_XPCS_CYCLIC_DATA_1_REG);
	pp2_gop_xpcs_lane_print(gop, "CYCLIC_DATA_2", lane,
			       PP2_XPCS_CYCLIC_DATA_2_REG);
	pp2_gop_xpcs_lane_print(gop, "CYCLIC_DATA_3", lane,
			       PP2_XPCS_CYCLIC_DATA_3_REG);
}

/* print value of unit registers */
void pp2_gop_serdes_lane_regs_dump(struct gop_hw *gop, int lane)
{
	pp2_info("\nSerdes Lane #%d registers]\n", lane);
	pp2_gop_serdes_print(gop, "PP2_SERDES_CFG_0_REG", lane,
			    PP2_SERDES_CFG_0_REG);
	pp2_gop_serdes_print(gop, "PP2_SERDES_CFG_1_REG", lane,
			    PP2_SERDES_CFG_1_REG);
	pp2_gop_serdes_print(gop, "PP2_SERDES_CFG_2_REG", lane,
			    PP2_SERDES_CFG_2_REG);
	pp2_gop_serdes_print(gop, "PP2_SERDES_CFG_3_REG", lane,
			    PP2_SERDES_CFG_3_REG);
	pp2_gop_serdes_print(gop, "PP2_SERDES_MISC_REG", lane,
			    PP2_SERDES_MISC_REG);
}

void pp2_gop_mac_print(struct pp2_mac_data *mac)
{
	pp2_info("\n Emac data structure\n");
	pp2_info("gop_index = %d\n",  mac->gop_index);
	pp2_info("flags = %ld\n",      mac->flags);
	pp2_info("phy_addr = %x\n",   mac->phy_addr);
	pp2_info("phy_mode = %d\n",   mac->phy_mode);
	pp2_info("force_link = %d\n", mac->force_link);
	pp2_info("autoneg = %d\n",    mac->autoneg);
	pp2_info("link = %d\n",       mac->link);
	pp2_info("duplex = %d\n",     mac->duplex);
	pp2_info("speed = %d\n",      mac->speed);
	pp2_info("MAC = {");
	for (int i = 0; i < 6; i++)
		pp2_info("%x ", mac->mac[i]);
	pp2_info("}\n");
}
