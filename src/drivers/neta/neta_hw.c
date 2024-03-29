/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "std_internal.h"
#include "env/sys_iomem.h"
#include "drivers/mv_neta.h"
#include "drivers/mv_neta_ppio.h"
#include "neta_bm.h"
#include "neta_ppio.h"
#include "neta_hw.h"


void neta_hw_reg_print(char *reg_name, void *base, u32 offset)
{
	void *addr = base + offset;

	pr_info("%-32s: %8p = 0x%08x\n", reg_name, addr, readl(addr));
}

/* Enable the port by setting the port enable bit of the MAC control register */
static void mvneta_port_enable(struct neta_port *pp)
{
	u32 val;

	/* Enable port */
	val = neta_reg_read(pp, MVNETA_GMAC_CTRL_0);
	val |= MVNETA_GMAC0_PORT_ENABLE;
	neta_reg_write(pp, MVNETA_GMAC_CTRL_0, val);
}

/* Disable the port and wait for about 200 usec before retuning */
static void neta_port_disable(struct neta_port *pp)
{
	u32 val;

	/* Reset the Enable bit in the Serial Control Register */
	val = neta_reg_read(pp, MVNETA_GMAC_CTRL_0);
	val &= ~MVNETA_GMAC0_PORT_ENABLE;
	neta_reg_write(pp, MVNETA_GMAC_CTRL_0, val);

	pp->link = 0;
	pp->duplex = -1;
	pp->speed = 0;

	udelay(200);
}

/* Power up the port */
static int neta_port_power_up(struct neta_port *pp, int phy_mode)
{
	u32 ctrl;

	/* MAC Cause register should be cleared */
	neta_reg_write(pp, MVNETA_UNIT_INTR_CAUSE, 0);

	ctrl = neta_reg_read(pp, MVNETA_GMAC_CTRL_2);

	/* Even though it might look weird, when we're configured in
	 * SGMII or QSGMII mode, the RGMII bit needs to be set.
	 */
	switch (phy_mode) {
	case PHY_INTERFACE_MODE_QSGMII:
		neta_reg_write(pp, MVNETA_SERDES_CFG, MVNETA_QSGMII_SERDES_PROTO);
		ctrl |= MVNETA_GMAC2_PCS_ENABLE | MVNETA_GMAC2_PORT_RGMII;
		break;
	case PHY_INTERFACE_MODE_SGMII:
		neta_reg_write(pp, MVNETA_SERDES_CFG, MVNETA_SGMII_SERDES_PROTO);
		ctrl |= MVNETA_GMAC2_PCS_ENABLE | MVNETA_GMAC2_PORT_RGMII;
		break;
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_RGMII_ID:
		ctrl |= MVNETA_GMAC2_PORT_RGMII;
		break;
	default:
		return -EINVAL;
	}

	/* Cancel Port Reset */
	ctrl &= ~MVNETA_GMAC2_PORT_RESET;
	neta_reg_write(pp, MVNETA_GMAC_CTRL_2, ctrl);

	while ((neta_reg_read(pp, MVNETA_GMAC_CTRL_2) &
		MVNETA_GMAC2_PORT_RESET) != 0)
		continue;

	return 0;
}

/* Clear all MIB counters */
static void neta_mib_counters_clear(struct neta_port *pp)
{
	int i;
	u32 dummy;

	/* Perform dummy reads from MIB counters */
	for (i = 0; i < MVNETA_MIB_LATE_COLLISION; i += 4)
		dummy = neta_reg_read(pp, (MVNETA_MIB_COUNTERS_BASE + i));
	dummy = neta_reg_read(pp, MVNETA_RX_DISCARD_FRAME_COUNT);
	dummy = neta_reg_read(pp, MVNETA_OVERRUN_FRAME_COUNT);
	dummy = dummy;
}

/* Start the Ethernet port RX and TX activity */
void neta_port_up(struct neta_port *pp)
{
	int queue;
	u32 q_map, val;

	val = neta_reg_read(pp, MVNETA_GMAC_AUTONEG_CONFIG);
	val &= ~MVNETA_GMAC_FORCE_LINK_DOWN;
	val |= MVNETA_GMAC_FORCE_LINK_PASS;
	neta_reg_write(pp, MVNETA_GMAC_AUTONEG_CONFIG, val);

	/* Enable all initialized TXs. */
	q_map = 0;
	for (queue = 0; queue < pp->txq_number; queue++) {
		struct neta_tx_queue *txq = &pp->txqs[queue];

		if (txq->descs != NULL)
			q_map |= (1 << queue);
	}
	neta_reg_write(pp, MVNETA_TXQ_CMD, q_map);

	q_map = 0;
	/* Enable all initialized RXQs. */
	for (queue = 0; queue < pp->rxq_number; queue++) {
		struct neta_rx_queue *rxq = &pp->rxqs[queue];

		if (rxq->descs != NULL)
			q_map |= (1 << queue);
	}
	neta_reg_write(pp, MVNETA_RXQ_CMD, q_map);

	/* start the Rx/Tx activity */
	mvneta_port_enable(pp);

}

/* Stop the Ethernet port activity */
void neta_port_down(struct neta_port *pp)
{
	u32 val;
	int count;

	val = neta_reg_read(pp, MVNETA_GMAC_AUTONEG_CONFIG);
	val &= ~MVNETA_GMAC_FORCE_LINK_PASS;
	val |= MVNETA_GMAC_FORCE_LINK_DOWN;
	neta_reg_write(pp, MVNETA_GMAC_AUTONEG_CONFIG, val);

	/* Stop Rx port activity. Check port Rx activity. */
	val = neta_reg_read(pp, MVNETA_RXQ_CMD) & MVNETA_RXQ_ENABLE_MASK;

	/* Issue stop command for active channels only */
	if (val != 0)
		neta_reg_write(pp, MVNETA_RXQ_CMD,
			    val << MVNETA_RXQ_DISABLE_SHIFT);

	/* Wait for all Rx activity to terminate. */
	count = 0;
	do {
		if (count++ >= MVNETA_RX_DISABLE_TIMEOUT_MSEC) {
			pr_warn("TIMEOUT for RX stopped ! rx_queue_cmd: 0x%08x\n", val);
			break;
		}
		udelay(1);

		val = neta_reg_read(pp, MVNETA_RXQ_CMD);
	} while (val & MVNETA_RXQ_ENABLE_MASK);

	/* Stop Tx port activity. Check port Tx activity. Issue stop
	 * command for active channels only
	 */
	val = (neta_reg_read(pp, MVNETA_TXQ_CMD)) & MVNETA_TXQ_ENABLE_MASK;

	if (val != 0)
		neta_reg_write(pp, MVNETA_TXQ_CMD,
			    (val << MVNETA_TXQ_DISABLE_SHIFT));

	/* Wait for all Tx activity to terminate. */
	count = 0;
	do {
		if (count++ >= MVNETA_TX_DISABLE_TIMEOUT_MSEC) {
			pr_warn("TIMEOUT for TX stopped status=0x%08x\n", val);
			break;
		}
		udelay(1);

		/* Check TX Command reg that all Txqs are stopped */
		val = neta_reg_read(pp, MVNETA_TXQ_CMD);

	} while (val & MVNETA_TXQ_ENABLE_MASK);

	/* Double check to verify that TX FIFO is empty */
	count = 0;
	do {
		if (count++ >= MVNETA_TX_FIFO_EMPTY_TIMEOUT) {
			pr_warn("TX FIFO empty timeout status=0x%08x\n", val);
			break;
		}
		udelay(1);

		val = neta_reg_read(pp, MVNETA_PORT_STATUS);
	} while (!(val & MVNETA_TX_FIFO_EMPTY) &&
		 (val & MVNETA_TX_IN_PRGRS));

	udelay(200);
}

/* Multicast tables methods */
/* Set all entries in Unicast MAC Table; queue==-1 means reject all */
static void mvneta_set_ucast_table(struct neta_port *pp, int queue)
{
	int offset;
	u32 val;

	if (queue == -1) {
		val = 0;
	} else {
		val = 0x1 | (queue << 1);
		val |= (val << 24) | (val << 16) | (val << 8);
	}

	for (offset = 0; offset <= 0xc; offset += 4)
		neta_reg_write(pp, MVNETA_DA_FILT_UCAST_BASE + offset, val);
}

/*MSD*/
/* Set all entries in Special Multicast MAC Table; queue==-1 means reject all */
static void mvneta_set_special_mcast_table(struct neta_port *pp, int queue)
{
	int offset;
	u32 val;

	if (queue == -1) {
		val = 0;
	} else {
		val = 0x1 | (queue << 1);
		val |= (val << 24) | (val << 16) | (val << 8);
	}

	for (offset = 0; offset <= 0xfc; offset += 4)
		neta_reg_write(pp, MVNETA_DA_FILT_SPEC_MCAST + offset, val);

}

/*MSD*/
/* Set all entries in Other Multicast MAC Table. queue==-1 means reject all */
static void mvneta_set_other_mcast_table(struct neta_port *pp, int queue)
{
	int offset;
	u32 val;

	if (queue == -1) {
		memset(pp->mcast_count, 0, sizeof(pp->mcast_count));
		val = 0;
	} else {
		memset(pp->mcast_count, 1, sizeof(pp->mcast_count));
		val = 0x1 | (queue << 1);
		val |= (val << 24) | (val << 16) | (val << 8);
	}

	for (offset = 0; offset <= 0xfc; offset += 4)
		neta_reg_write(pp, MVNETA_DA_FILT_OTH_MCAST + offset, val);
}

static void mvneta_mac_config(struct neta_port *pp)
{
	enum neta_port_type port_type = PORT_TYPE_SGMII; /* TBD: mvneta_port_type_get(pp);*/
	u32 new_ctrl2, gmac_ctrl2 = neta_reg_read(pp, MVNETA_GMAC_CTRL_2);
	u32 new_clk, gmac_clk = neta_reg_read(pp, MVNETA_GMAC_CLOCK_DIVIDER);
	u32 new_an, gmac_an = neta_reg_read(pp, MVNETA_GMAC_AUTONEG_CONFIG);

	/* Clear all fields need to config with different work mode */
	new_ctrl2 = gmac_ctrl2 & ~MVNETA_GMAC2_SGMII_INBAND_AN_MODE;
	new_clk = gmac_clk & ~MVNETA_GMAC_1MS_CLOCK_ENABLE;
	new_an = gmac_an & ~(MVNETA_GMAC_INBAND_AN_ENABLE |
			     MVNETA_GMAC_INBAND_RESTART_AN |
			     MVNETA_GMAC_CONFIG_MII_SPEED |
			     MVNETA_GMAC_CONFIG_GMII_SPEED |
			     MVNETA_GMAC_AN_SPEED_EN |
			     MVNETA_GMAC_ADVERT_SYM_FLOW_CTRL |
			     MVNETA_GMAC_CONFIG_FLOW_CTRL |
			     MVNETA_GMAC_AN_FLOW_CTRL_EN |
			     MVNETA_GMAC_CONFIG_FULL_DUPLEX |
			     MVNETA_GMAC_AN_DUPLEX_EN |
			     MVNETA_GMAC_FORCE_LINK_PASS |
			     MVNETA_GMAC_FORCE_LINK_DOWN);

	if (pp->use_inband_status) {
		switch (port_type) {
		case PORT_TYPE_SGMII:
			/* SGMII mode receives the state from the PHY */
			new_ctrl2 |= MVNETA_GMAC2_SGMII_INBAND_AN_MODE;
			new_clk |= MVNETA_GMAC_1MS_CLOCK_ENABLE;
			/* SGMII aoto-nego clock */
			new_an |= MVNETA_GMAC_INBAND_AN_ENABLE |
				   MVNETA_GMAC_INBAND_AN_BYPASS_EN |
				   MVNETA_GMAC_AN_SPEED_EN |
				   MVNETA_GMAC_AN_DUPLEX_EN;
			break;

		case PORT_TYPE_1000BASE_X:
			/* A3700 spec: In 1000BASE-X, the port must be set to work
			 * in full-duplex mode, at 1000 Mbps.
			 * Duplex and Speed Auto-Negotiation must be disabled
			 */
			new_an |= MVNETA_GMAC_INBAND_AN_ENABLE |
				  MVNETA_GMAC_INBAND_AN_BYPASS_EN |
				  MVNETA_GMAC_CONFIG_GMII_SPEED |
				  MVNETA_GMAC_ADVERT_SYM_FLOW_CTRL |
				  MVNETA_GMAC_AN_FLOW_CTRL_EN |
				  MVNETA_GMAC_CONFIG_FLOW_CTRL |
				  MVNETA_GMAC_CONFIG_FULL_DUPLEX;

			break;
		}
	}
#if 0
	else {
		/* SMI auto-nego, GMAC will get info from PHY with SMI */
		if (pp->phy_dev) {
			if (pp->phy_dev->duplex)
				new_an |= MVNETA_GMAC_CONFIG_FULL_DUPLEX;

			if (pp->phy_dev->speed == SPEED_1000)
				new_an |= MVNETA_GMAC_CONFIG_GMII_SPEED;
			else if (pp->phy_dev->speed == SPEED_100)
				new_an |= MVNETA_GMAC_CONFIG_MII_SPEED;

			if (pp->phy_dev->pause)
				new_an |= MVNETA_GMAC_CONFIG_FLOW_CTRL;

			if (pp->phy_dev->asym_pause)
				new_an |= MVNETA_GMAC_ADVERT_ASYM_FC_ADV;

			/* Fixed link, Force link up */
			if (phy_is_pseudo_fixed_link(pp->phy_dev)) {
				new_an |= MVNETA_GMAC_FORCE_LINK_PASS;
				new_an &= ~MVNETA_GMAC_FORCE_LINK_DOWN;
			}
		}
	}
#endif
	/* Armada 370 documentation says we can only change the port mode
	 * and in-band enable when the link is down, so force it down
	 * while making these changes. We also do this for GMAC_CTRL2
	 */
	if ((new_ctrl2 ^ gmac_ctrl2) & MVNETA_GMAC2_SGMII_INBAND_AN_MODE ||
	    (new_an  ^ gmac_an) & MVNETA_GMAC_INBAND_AN_ENABLE) {
		neta_reg_write(pp, MVNETA_GMAC_AUTONEG_CONFIG,
			    (gmac_an & ~MVNETA_GMAC_FORCE_LINK_PASS) |
			    MVNETA_GMAC_FORCE_LINK_DOWN);
	}

	if (new_ctrl2 != gmac_ctrl2)
		neta_reg_write(pp, MVNETA_GMAC_CTRL_2, new_ctrl2);
	if (new_clk != gmac_clk)
		neta_reg_write(pp, MVNETA_GMAC_CLOCK_DIVIDER, new_clk);
	if (new_an != gmac_an)
		neta_reg_write(pp, MVNETA_GMAC_AUTONEG_CONFIG, new_an);
}

/* This method sets defaults to the NETA port:
 *	Clears interrupt Cause and Mask registers.
 *	Clears all MAC tables.
 *	Sets defaults to all registers.
 *	Resets RX and TX descriptor rings.
 *	Resets PHY.
 * This method can be called after mvneta_port_down() to return the port
 *	settings to defaults.
 */
static void neta_defaults_set(struct neta_port *pp)
{
	int cpu;
	int queue;
	u32 val;
	int max_cpu = 2; /*num_present_cpus();*/

	/* Clear all Cause registers */
	/*on_each_cpu(mvneta_percpu_clear_intr_cause, pp, true);*/

	/* Mask all interrupts */
	/* on_each_cpu(mvneta_percpu_mask_interrupt, pp, true); */
	/* neta_reg_write(pp, MVNETA_INTR_ENABLE, 0);*/

	/* Enable MBUS Retry bit16 */
	neta_reg_write(pp, MVNETA_MBUS_RETRY, 0x20);

	/* Set CPU queue access map. CPUs are assigned to the RX and
	 * TX queues modulo their number. If there is only one TX
	 * queue then it is assigned to the CPU associated to the
	 * default RX queue. Without per-CPU processing enable all
	 * CPUs' access to all TX and RX queues.
	 */
	/*for_each_present_cpu(cpu) {*/
	for (cpu = 0; cpu < max_cpu; cpu++) {
		int rxq_map = 0, txq_map = 0;

		txq_map = MVNETA_CPU_TXQ_ACCESS_ALL_MASK;
		rxq_map = MVNETA_CPU_RXQ_ACCESS_ALL_MASK;

		neta_reg_write(pp, MVNETA_CPU_MAP(cpu), rxq_map | txq_map);
	}

	/* Reset RX and TX DMAs */
	neta_reg_write(pp, MVNETA_PORT_RX_RESET, MVNETA_PORT_RX_DMA_RESET);
	neta_reg_write(pp, MVNETA_PORT_TX_RESET, MVNETA_PORT_TX_DMA_RESET);

	/* Disable Legacy WRR, Disable EJP, Release from reset */
	neta_reg_write(pp, MVNETA_TXQ_CMD_1, 0);
	for (queue = 0; queue < pp->txq_number; queue++) {
		neta_reg_write(pp, MVETH_TXQ_TOKEN_COUNT_REG(queue), 0);
		neta_reg_write(pp, MVETH_TXQ_TOKEN_CFG_REG(queue), 0);
	}

	neta_reg_write(pp, MVNETA_PORT_TX_RESET, 0);
	neta_reg_write(pp, MVNETA_PORT_RX_RESET, 0);

	/* Update val of portCfg register accordingly with all RxQueue types */
	val = MVNETA_PORT_CONFIG_DEFL_VALUE(pp->rxq_def);
	neta_reg_write(pp, MVNETA_PORT_CONFIG, val);

	val = 0;
	neta_reg_write(pp, MVNETA_PORT_CONFIG_EXTEND, val);
	neta_reg_write(pp, MVNETA_RX_MIN_FRAME_SIZE, 64);

	/* Build PORT_SDMA_CONFIG_REG */
	val = 0;

	/* Default burst size */
	val |= MVNETA_TX_BRST_SZ_MASK(MVNETA_SDMA_BRST_SIZE_16);
	val |= MVNETA_RX_BRST_SZ_MASK(MVNETA_SDMA_BRST_SIZE_16);
	val |= MVNETA_RX_NO_DATA_SWAP | MVNETA_TX_NO_DATA_SWAP;

#if defined(__BIG_ENDIAN)
	val |= MVNETA_DESC_SWAP;
#endif

	/* Assign port SDMA configuration */
	neta_reg_write(pp, MVNETA_SDMA_CONFIG, val);

	/* Disable PHY polling in hardware, since we're using the
	 * kernel phylib to do this.
	 */
	val = neta_reg_read(pp, MVNETA_UNIT_CONTROL);
	val &= ~MVNETA_PHY_POLLING_ENABLE;
	neta_reg_write(pp, MVNETA_UNIT_CONTROL, val);
#if 0
	mvneta_mac_config(pp);
	mvneta_set_ucast_table(pp, -1);
	mvneta_set_special_mcast_table(pp, -1);
	mvneta_set_other_mcast_table(pp, -1);

	/* Set port interrupt enable register - default enable all */
	neta_reg_write(pp, MVNETA_INTR_ENABLE,
		    (MVNETA_RXQ_INTR_ENABLE_ALL_MASK
		     | MVNETA_TXQ_INTR_ENABLE_ALL_MASK));
#endif
	neta_mib_counters_clear(pp);
}

int neta_port_map(int port_id, struct neta_port *pp)
{
	int err;
	struct sys_iomem_params params;

	params.type = SYS_IOMEM_T_UIO;
	params.index = port_id;
	params.devname = "neta";

	if (sys_iomem_exists(&params)) {
		err = sys_iomem_init(&params, &pp->sys_iomem);
		if (err)
			return -1;
	} else
		return -1;

	/* Map the port physical address */
	err = sys_iomem_map(pp->sys_iomem, "neta_regs", &pp->paddr, (void **)(&pp->base));
	if (err) {
		pr_info("%s: failed sys_iomem_map().\n", __func__);
		sys_iomem_deinit(pp->sys_iomem);
		return err;
	}
	return 0;
}

/* Set max sizes for tx queues */
static void mvneta_txq_max_tx_size_set(struct neta_port *pp, int max_tx_size)

{
	u32 val, size, mtu;
	int queue;

	mtu = max_tx_size * 8;
	if (mtu > MVNETA_TX_MTU_MAX)
		mtu = MVNETA_TX_MTU_MAX;

	/* Set MTU */
	val = neta_reg_read(pp, MVNETA_TX_MTU);
	val &= ~MVNETA_TX_MTU_MAX;
	val |= mtu;
	neta_reg_write(pp, MVNETA_TX_MTU, val);

	/* TX token size and all TXQs token size must be larger that MTU */
	val = neta_reg_read(pp, MVNETA_TX_TOKEN_SIZE);

	size = val & MVNETA_TX_TOKEN_SIZE_MAX;
	if (size < mtu) {
		size = mtu;
		val &= ~MVNETA_TX_TOKEN_SIZE_MAX;
		val |= size;
		neta_reg_write(pp, MVNETA_TX_TOKEN_SIZE, val);
	}
	for (queue = 0; queue < pp->txq_number; queue++) {
		val = neta_reg_read(pp, MVNETA_TXQ_TOKEN_SIZE_REG(queue));

		size = val & MVNETA_TXQ_TOKEN_SIZE_MAX;
		if (size < mtu) {
			size = mtu;
			val &= ~MVNETA_TXQ_TOKEN_SIZE_MAX;
			val |= size;
			neta_reg_write(pp, MVNETA_TXQ_TOKEN_SIZE_REG(queue), val);
		}
	}
}

/* Rx/Tx queue initialization/cleanup methods */
/* Change maximum receive size of the port. */
static void mvneta_max_rx_size_set(struct neta_port *pp, int max_rx_size)
{
	u32 val;

	val =  neta_reg_read(pp, MVNETA_GMAC_CTRL_0);
	val &= ~MVNETA_GMAC_MAX_RX_SIZE_MASK;
	val |= ((max_rx_size - MV_MH_SIZE) / 2) <<
		MVNETA_GMAC_MAX_RX_SIZE_SHIFT;
	neta_reg_write(pp, MVNETA_GMAC_CTRL_0, val);
}

/* Set rx queue offset */
static void mvneta_rxq_offset_set(struct neta_port *pp,
				  struct neta_rx_queue *rxq,
				  int offset)
{
	u32 val;

	val = neta_reg_read(pp, MVNETA_RXQ_CONFIG_REG(rxq->id));
	val &= ~MVNETA_RXQ_PKT_OFFSET_ALL_MASK;

	/* Offset is in */
	val |= MVNETA_RXQ_PKT_OFFSET_MASK(offset >> 3);
	neta_reg_write(pp, MVNETA_RXQ_CONFIG_REG(rxq->id), val);
}

/* Set rxq buf size */
static void mvneta_rxq_buf_size_set(struct neta_port *pp,
				    struct neta_rx_queue *rxq,
				    int buf_size)
{
	u32 val;

	if (!IS_ALIGNED(buf_size, 8))
		buf_size = ALIGN(buf_size, 8);

	val = neta_reg_read(pp, MVNETA_RXQ_SIZE_REG(rxq->id));

	val &= ~MVNETA_RXQ_BUF_SIZE_MASK;
	val |= ((buf_size >> 3) << MVNETA_RXQ_BUF_SIZE_SHIFT);
	neta_reg_write(pp, MVNETA_RXQ_SIZE_REG(rxq->id), val);
}

/* Disable buffer management (BM) */
static void mvneta_rxq_bm_disable(struct neta_port *pp,
				  struct neta_rx_queue *rxq)
{
	u32 val;

	val = neta_reg_read(pp, MVNETA_RXQ_CONFIG_REG(rxq->id));
	val &= ~MVNETA_RXQ_HW_BUF_ALLOC;
	neta_reg_write(pp, MVNETA_RXQ_CONFIG_REG(rxq->id), val);
}

/* Enable buffer management (BM) */
static void mvneta_rxq_bm_enable(struct neta_port *pp,
				 struct neta_rx_queue *rxq)
{
	u32 val;

	val = neta_reg_read(pp, MVNETA_RXQ_CONFIG_REG(rxq->id));
	val |= MVNETA_RXQ_HW_BUF_ALLOC;
	neta_reg_write(pp, MVNETA_RXQ_CONFIG_REG(rxq->id), val);
}

/* Notify HW about port's assignment of pool for bigger packets */
static void mvneta_rxq_long_pool_set(struct neta_port *pp,
				     struct neta_rx_queue *rxq)
{
	u32 val;

	val = neta_reg_read(pp, MVNETA_RXQ_CONFIG_REG(rxq->id));
	val &= ~MVNETA_RXQ_LONG_POOL_ID_MASK;
	val |= (pp->pool_long->id << MVNETA_RXQ_LONG_POOL_ID_SHIFT);

	neta_reg_write(pp, MVNETA_RXQ_CONFIG_REG(rxq->id), val);
}

/* Notify HW about port's assignment of pool for smaller packets */
static void mvneta_rxq_short_pool_set(struct neta_port *pp,
				      struct neta_rx_queue *rxq)
{
	u32 val;

	val = neta_reg_read(pp, MVNETA_RXQ_CONFIG_REG(rxq->id));
	val &= ~MVNETA_RXQ_SHORT_POOL_ID_MASK;
	val |= (pp->pool_short->id << MVNETA_RXQ_SHORT_POOL_ID_SHIFT);

	neta_reg_write(pp, MVNETA_RXQ_CONFIG_REG(rxq->id), val);
}

/* Set port's receive buffer size for assigned BM pool */
void neta_bm_pool_bufsize_set(struct neta_port *pp,
				int buf_size,
				u8 pool_id)
{
	u32 val;

	if (!IS_ALIGNED(buf_size, 8)) {
		pr_err("illegal buf_size value %d, round to %d\n",
		       buf_size, ALIGN(buf_size, 8));
		buf_size = ALIGN(buf_size, 8);
	}

	val = neta_reg_read(pp, MVNETA_PORT_POOL_BUFFER_SZ_REG(pool_id));
	val &= ~MVNETA_PORT_POOL_BUFFER_SZ_MASK;
	val |= buf_size & MVNETA_PORT_POOL_BUFFER_SZ_MASK;

	neta_reg_write(pp, MVNETA_PORT_POOL_BUFFER_SZ_REG(pool_id), val);
}

/* Create a specified RX queue */
static int mvneta_rxq_init(struct neta_port *pp,
			   struct neta_rx_queue *rxq)

{
	int i;

	rxq->size = pp->rx_ring_size;

	/* Allocate memory for RX descriptors */
	rxq->descs =  mv_sys_dma_mem_alloc(rxq->size * sizeof(struct neta_ppio_desc),
					   MVNETA_DESC_ALIGNED_SIZE);
	if (rxq->descs == NULL)
		return -ENOMEM;

	for (i = 0; i < rxq->size; i++)
		/* invalidate packet descriptor */
		rxq->descs[i].cmds[1] = MVNETA_DESC_WATERMARK;

	rxq->descs_phys = mv_sys_dma_mem_virt2phys(rxq->descs);
	rxq->last_desc = rxq->size - 1;
	rxq->to_refill_cntr = rxq->size;

	/* Set Rx descriptors queue starting address */
	neta_reg_write(pp, MVNETA_RXQ_BASE_ADDR_REG(rxq->id), rxq->descs_phys);
	neta_reg_write(pp, MVNETA_RXQ_SIZE_REG(rxq->id), rxq->size);

	/* Set Offset */
	mvneta_rxq_offset_set(pp, rxq, pp->rx_offset);

	/* Configure Buffer Manager SW / HW */
	if (pp->bm_priv) {
		mvneta_rxq_bm_enable(pp, rxq);
		mvneta_rxq_long_pool_set(pp, rxq);
		mvneta_rxq_short_pool_set(pp, rxq);
		neta_port_inq_update(pp, rxq, 0, rxq->size);
	} else {
		mvneta_rxq_buf_size_set(pp, rxq, pp->buf_size);
		mvneta_rxq_bm_disable(pp, rxq);
	}

	return 0;
}

/* Cleanup Rx queue */
static void mvneta_rxq_deinit(struct neta_port *pp,
			      struct neta_rx_queue *rxq)
{
	if (rxq->descs)
		mv_sys_dma_mem_free(rxq->descs);

	rxq->descs             = NULL;
	rxq->last_desc         = 0;
	rxq->next_desc_to_proc = 0;
	rxq->descs_phys        = 0;
}

/* Create and initialize a tx queue */
static int mvneta_txq_init(struct neta_port *pp,
			   struct neta_tx_queue *txq)
{
	txq->size = pp->tx_ring_size;

	/* A queue must always have room for at least one skb.
	 * Therefore, stop the queue when the free entries reaches
	 * the maximum number of descriptors per skb.
	 */
	txq->tx_stop_threshold = txq->size; /* TBD - MVNETA_MAX_SKB_DESCS;*/
	txq->tx_wake_threshold = txq->tx_stop_threshold / 2;


	/* Allocate memory for TX descriptors */
	txq->descs =  mv_sys_dma_mem_alloc(txq->size * MVNETA_DESC_ALIGNED_SIZE,
					   MVNETA_DESC_ALIGNED_SIZE);
	if (txq->descs == NULL)
		return -ENOMEM;
	txq->descs_phys = mv_sys_dma_mem_virt2phys(txq->descs);

	txq->last_desc = txq->size - 1;

	/* Set maximum bandwidth for enabled TXQs */
	neta_reg_write(pp, MVETH_TXQ_TOKEN_CFG_REG(txq->id), 0x03ffffff);
	neta_reg_write(pp, MVETH_TXQ_TOKEN_COUNT_REG(txq->id), 0x3fffffff);

	/* Set Tx descriptors queue starting address */
	neta_reg_write(pp, MVNETA_TXQ_BASE_ADDR_REG(txq->id), txq->descs_phys);
	neta_reg_write(pp, MVNETA_TXQ_SIZE_REG(txq->id), txq->size);

	txq->tx_skb = kcalloc(txq->size, sizeof(*txq->tx_skb), GFP_KERNEL);
	if (txq->tx_skb == NULL) {
		mv_sys_dma_mem_free(txq->descs);
		return -ENOMEM;
	}
	return 0;
}

/* Free allocated resources when mvneta_txq_init() fails to allocate memory*/
static void mvneta_txq_deinit(struct neta_port *pp,
			      struct neta_tx_queue *txq)
{
	if (txq->tso_hdrs)
		mv_sys_dma_mem_free(txq->tso_hdrs);
	if (txq->descs)
		mv_sys_dma_mem_free(txq->descs);

	txq->descs             = NULL;
	txq->last_desc         = 0;
	txq->next_desc_to_proc = 0;
	txq->descs_phys        = 0;

	/* Set minimum bandwidth for disabled TXQs */
	neta_reg_write(pp, MVETH_TXQ_TOKEN_CFG_REG(txq->id), 0);
	neta_reg_write(pp, MVETH_TXQ_TOKEN_COUNT_REG(txq->id), 0);

	/* Set Tx descriptors queue starting address and size */
	neta_reg_write(pp, MVNETA_TXQ_BASE_ADDR_REG(txq->id), 0);
	neta_reg_write(pp, MVNETA_TXQ_SIZE_REG(txq->id), 0);
}

/* Cleanup all Tx queues */
static void mvneta_cleanup_txqs(struct neta_port *pp)
{
	int queue;

	for (queue = 0; queue < pp->txq_number; queue++)
		mvneta_txq_deinit(pp, &pp->txqs[queue]);
}

/* Cleanup all Rx queues */
static void mvneta_cleanup_rxqs(struct neta_port *pp)
{
	int queue;

	for (queue = 0; queue < pp->rxq_number; queue++)
		mvneta_rxq_deinit(pp, &pp->rxqs[queue]);
}

/* Init all Rx queues */
static int mvneta_setup_rxqs(struct neta_port *pp)
{
	int queue;

	for (queue = 0; queue < pp->rxq_number; queue++) {
		int err = mvneta_rxq_init(pp, &pp->rxqs[queue]);

		if (err) {
			pr_err("%s: can't create rxq=%d\n", __func__, queue);
			mvneta_cleanup_rxqs(pp);
			return err;
		}
	}

	return 0;
}

/* Init all tx queues */
static int mvneta_setup_txqs(struct neta_port *pp)
{
	int queue;

	for (queue = 0; queue < pp->txq_number; queue++) {
		int err = mvneta_txq_init(pp, &pp->txqs[queue]);

		if (err) {
			pr_err("%s: can't create txq=%d\n", __func__, queue);
			mvneta_cleanup_txqs(pp);
			return err;
		}
	}

	return 0;
}

static int mvneta_open(struct neta_port *pp)
{
	int ret, val;

	ret = mvneta_setup_rxqs(pp);
	if (ret)
		return ret;

	ret = mvneta_setup_txqs(pp);
	if (ret)
		goto err_cleanup_rxqs;

	/* Start port */
	mvneta_max_rx_size_set(pp, pp->mru);
	mvneta_txq_max_tx_size_set(pp, pp->mtu);

	/* Set Port Acceleration Mode */
	if (pp->bm_priv)
		/* HW buffer management + legacy parser */
		val = MVNETA_ACC_MODE_EXT2;
	else
		/* SW buffer management + legacy parser */
		val = MVNETA_ACC_MODE_EXT1;

	neta_reg_write(pp, MVNETA_ACC_MODE, val);

	if (pp->bm_priv)
		neta_reg_write(pp, MVNETA_BM_ADDRESS, pp->bm_priv->bppi_phys_addr);

	return 0;

err_cleanup_rxqs:
	mvneta_cleanup_rxqs(pp);
	return ret;
}

/* Free all packets pending transmit from all TXQs and reset TX port */
static void mvneta_tx_reset(struct neta_port *pp)
{

	/* free the skb's in the tx ring */
/* TBD	for (queue = 0; queue < txq_number; queue++)
 *		mvneta_txq_done_force(pp, &pp->txqs[queue]);
*/
	neta_reg_write(pp, MVNETA_PORT_TX_RESET, MVNETA_PORT_TX_DMA_RESET);
	neta_reg_write(pp, MVNETA_PORT_TX_RESET, 0);
}

static void mvneta_rx_reset(struct neta_port *pp)
{
	neta_reg_write(pp, MVNETA_PORT_RX_RESET, MVNETA_PORT_RX_DMA_RESET);
	neta_reg_write(pp, MVNETA_PORT_RX_RESET, 0);
}

/* Stop the port, free port interrupt line */
static int mvneta_stop(struct neta_port *pp)
{
	mvneta_cleanup_rxqs(pp);
	mvneta_cleanup_txqs(pp);

	mvneta_tx_reset(pp);
	mvneta_rx_reset(pp);

	return 0;
}

/* Initialize hw */
int neta_port_hw_init(struct neta_port *pp)
{
	int queue;

	/* Disable port */
	neta_port_disable(pp);
	/* reset tx/rx DMA */
	mvneta_tx_reset(pp);
	mvneta_rx_reset(pp);

	pp->txqs = kcalloc(pp->txq_number, sizeof(struct neta_tx_queue), GFP_KERNEL);
	if (!pp->txqs)
		return -ENOMEM;

	/* Initialize TX descriptor rings */
	for (queue = 0; queue < pp->txq_number; queue++) {
		struct neta_tx_queue *txq = &pp->txqs[queue];

		txq->id = queue;
		txq->size = pp->tx_ring_size;
	}

	pp->rxqs = kcalloc(pp->rxq_number, sizeof(struct neta_rx_queue), GFP_KERNEL);
	if (!pp->rxqs)
		return -ENOMEM;

	/* Create Rx descriptor rings */
	for (queue = 0; queue < pp->rxq_number; queue++) {
		struct neta_rx_queue *rxq = &pp->rxqs[queue];

		rxq->id = queue;
		rxq->size = pp->rx_ring_size;
	}

	mvneta_open(pp);
	return 0;
}

int neta_port_hw_deinit(struct neta_port *pp)
{
	mvneta_stop(pp);
	return 0;
}

static int neta_port_check_mtu_valid(struct neta_port *port, uint16_t mtu)
{
	/* Validate MTU */
	if (mtu < MVNETA_PORT_MIN_MTU) {
		pr_err("PORT: cannot change MTU to less than %u bytes\n", MVNETA_PORT_MIN_MTU);
		return -EINVAL;
	}

	/* Buffer_pool sizes are not relevant for mtu, only mru. */

	return 0;
}

/* Set and update the port MTU */
int neta_port_set_mtu(struct neta_port *port, uint16_t mtu)
{
	int err = 0;

	err = neta_port_check_mtu_valid(port, mtu);
	if (err)
		return err;

	if (port->is_running)
		neta_port_down(port);

	port->mtu = mtu;
	mvneta_txq_max_tx_size_set(port, port->mtu);

	if (port->is_running)
		neta_port_up(port);

	return err;
}

/* Get MTU */
void neta_port_get_mtu(struct neta_port *port, uint16_t *mtu)
{
	/* Straightforward. Useful for informing clients the
	 * maximum size their TX BM pool buffers should have,
	 * physical TXQs capabilities, packet fragmentation etc.
	 */
	*mtu = port->mtu;
}

static int neta_port_check_mru_valid(struct neta_port *port, uint16_t mru)
{
	if (mru < MVNETA_PORT_MIN_MRU) {
		pr_err("PORT: cannot change MRU to less than %u bytes\n", MVNETA_PORT_MIN_MRU);
		return -EINVAL;
	}

	return 0;
}

/* Set and update the port MRU. The function assumes mru valid is valid */
int neta_port_set_mru(struct neta_port *port, uint16_t mru)
{
	int err = 0;
	int q;

	err = neta_port_check_mru_valid(port, mru);
	if (err)
		return err;

	/* Stop rx / tx queues */
	if (port->is_running)
		neta_port_down(port);

	port->buf_size = mru + port->rx_offset;
	port->mru = mru;
	mvneta_max_rx_size_set(port, port->mru);
	for (q = 0; q < port->rxq_number; q++)
		mvneta_rxq_buf_size_set(port, &port->rxqs[q], port->buf_size);

	/* Start rx / tx queues */
	if (port->is_running)
		neta_port_up(port);

	return err;
}

/* Get MRU */
void neta_port_get_mru(struct neta_port *port, uint16_t *len)
{
	/* Straightforward. Useful for informing clients the
	 * maximum size their RX BM pool buffers should have
	 */
	*len = port->mru;
}

/* Set unicast address */
void neta_add_ucast_addr(struct neta_port *pp, u8 last_nibble, int queue)
{
	unsigned int unicast_reg;
	unsigned int tbl_offset;
	unsigned int reg_offset;

	/* Locate the Unicast table entry */
	last_nibble = (0xf & last_nibble);

	/* offset from unicast tbl base */
	tbl_offset = (last_nibble / 4) * 4;

	/* offset within the above reg  */
	reg_offset = last_nibble % 4;

	unicast_reg = neta_reg_read(pp, (MVNETA_DA_FILT_UCAST_BASE + tbl_offset));

	unicast_reg &= ~(0xff << (8 * reg_offset));
	unicast_reg |= ((0x01 | (queue << 1)) << (8 * reg_offset));

	neta_reg_write(pp, MVNETA_DA_FILT_UCAST_BASE + tbl_offset, unicast_reg);
}

/* Remove unicast address */
void neta_del_ucast_addr(struct neta_port *pp, u8 last_nibble)
{
	unsigned int unicast_reg;
	unsigned int tbl_offset;
	unsigned int reg_offset;

	/* Locate the Unicast table entry */
	last_nibble = (0xf & last_nibble);

	/* offset from unicast tbl base */
	tbl_offset = (last_nibble / 4) * 4;

	/* offset within the above reg  */
	reg_offset = last_nibble % 4;

	unicast_reg = neta_reg_read(pp, (MVNETA_DA_FILT_UCAST_BASE + tbl_offset));

	/* Clear accepts frame bit at specified unicast DA tbl entry */
	unicast_reg &= ~(0xff << (8 * reg_offset));

	neta_reg_write(pp, MVNETA_DA_FILT_UCAST_BASE + tbl_offset, unicast_reg);
}

/* Remove all unicast addresses */
void neta_flush_ucast_table(struct neta_port *pp)
{
	int offset;
	u32 val = 0;

	for (offset = 0; offset <= 0xc; offset += 4)
		neta_reg_write(pp, MVNETA_DA_FILT_UCAST_BASE + offset, val);
}

/* Debug dump unicast address table */
void neta_ucast_table_dump(struct neta_port *pp)
{
	int offset;
	u32 val;

	pr_info("\nNETA unicast address table dump:\n");
	for (offset = 0; offset <= 0xc; offset += 4) {
		val = neta_reg_read(pp, MVNETA_DA_FILT_UCAST_BASE + offset);
		pr_info("0x%x ", val);
	}
	pr_info("\n");
}

void neta_port_set_loopback(struct neta_port *pp, int en)
{
	u32 val;

	val = neta_reg_read(pp, MVNETA_GMAC_CTRL_1);
	if (en)
		val |= MVNETA_GMAC1_PORT_LOOPBACK_EN;
	else
		val &= ~MVNETA_GMAC1_PORT_LOOPBACK_EN;
	neta_reg_write(pp, MVNETA_GMAC_CTRL_1, val);
}

int neta_port_get_loopback(struct neta_port *pp, int *en)
{
	u32 val;

	val = neta_reg_read(pp, MVNETA_GMAC_CTRL_1);
	*en = !!(val & MVNETA_GMAC1_PORT_LOOPBACK_EN);

	return 0;
}

