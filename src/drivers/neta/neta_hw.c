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
#include "env/sys_iomem.h"

#include "drivers/mv_neta.h"
#include "drivers/mv_neta_ppio.h"
#include "neta_ppio.h"
#include "neta_hw.h"

/* Various constants */

/* Coalescing */
#define MVNETA_TXDONE_COAL_PKTS		0	/* interrupt per packet */
#define MVNETA_RX_COAL_PKTS		32
#define MVNETA_RX_COAL_USEC		100

static struct sys_iomem *neta_iomem_init(int port)
{
#if 0
	struct sys_iomem *iomem_info;
	int i, err;
	struct sys_iomem_params params;

	/* We support two HW types: eip197 or eip97 */
	params.type = SYS_IOMEM_T_UIO;
	params.index = engine;
	for (i = 0; i < ARRAY_SIZE(sam_supported_name); i++) {
		params.devname = sam_supported_name[i];
		if (sys_iomem_exists(&params)) {
			err = sys_iomem_init(&params, &iomem_info);
			if (!err) {
				*type = sam_supported_type[i];
				return iomem_info;
			}
		}
	}
#endif
	return NULL;
}

void neta_hw_reg_print(char *reg_name, void *base, u32 offset)
{
	void *addr = base + offset;

	pr_info("%-32s: %8p = 0x%08x\n", reg_name, addr, readl(addr));
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
static void neta_port_up(struct neta_port *pp)
{
	int queue;
	u32 q_map;

	/* Enable all initialized TXs. */
	q_map = 0;
	for (queue = 0; queue < pp->txq_number; queue++) {
		struct neta_tx_queue *txq = &pp->txqs[queue];

		if (txq->descs != NULL)
			q_map |= (1 << queue);
	}
	neta_reg_write(pp, MVNETA_TXQ_CMD, q_map);

	/* Enable all initialized RXQs. */
	for (queue = 0; queue < pp->rxq_number; queue++) {
		struct neta_rx_queue *rxq = &pp->rxqs[queue];

		if (rxq->descs != NULL)
			q_map |= (1 << queue);
	}
	neta_reg_write(pp, MVNETA_RXQ_CMD, q_map);
}

/* Multicast tables methods */

/*MSD*/
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
		int rxq, txq;

		if (!pp->neta_armada3700) {
			for (rxq = 0; rxq < pp->rxq_number; rxq++)
				if ((rxq % max_cpu) == cpu)
					rxq_map |= MVNETA_CPU_RXQ_ACCESS(rxq);

			for (txq = 0; txq < pp->txq_number; txq++)
				if ((txq % max_cpu) == cpu)
					txq_map |= MVNETA_CPU_TXQ_ACCESS(txq);

			/* With only one TX queue we configure a special case
			 * which will allow to get all the irq on a single
			 * CPU.
			 */
			if (pp->txq_number == 1)
				txq_map = (cpu == pp->rxq_def) ?
					MVNETA_CPU_TXQ_ACCESS(1) : 0;
		} else {
			txq_map = MVNETA_CPU_TXQ_ACCESS_ALL_MASK;
			rxq_map = MVNETA_CPU_RXQ_ACCESS_ALL_MASK;
		}

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

	/* Set Port Acceleration Mode */
	if (pp->bm_priv)
		/* HW buffer management + legacy parser */
		val = MVNETA_ACC_MODE_EXT2;
	else
		/* SW buffer management + legacy parser */
		val = MVNETA_ACC_MODE_EXT1;
	neta_reg_write(pp, MVNETA_ACC_MODE, val);
#if 0
	/* TBD */
	if (pp->bm_priv)
		neta_reg_write(pp, MVNETA_BM_ADDRESS, pp->bm_priv->bppi_phys_addr);
#endif
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

	mvneta_mac_config(pp);
	mvneta_set_ucast_table(pp, -1);
	mvneta_set_special_mcast_table(pp, -1);
	mvneta_set_other_mcast_table(pp, -1);

	/* Set port interrupt enable register - default enable all */
	neta_reg_write(pp, MVNETA_INTR_ENABLE,
		    (MVNETA_RXQ_INTR_ENABLE_ALL_MASK
		     | MVNETA_TXQ_INTR_ENABLE_ALL_MASK));

	neta_mib_counters_clear(pp);
}

/* Initialize hw */
int neta_port_hw_init(struct neta_port *pp)
{
	int queue;

	/* Disable port */
	neta_port_disable(pp);

	/* Set port default values */
	neta_defaults_set(pp);

	pp->txqs = kcalloc(pp->txq_number, sizeof(struct neta_tx_queue), GFP_KERNEL);
	if (!pp->txqs)
		return -ENOMEM;

	/* Initialize TX descriptor rings */
	for (queue = 0; queue < pp->txq_number; queue++) {
		struct neta_tx_queue *txq = &pp->txqs[queue];

		txq->id = queue;
		txq->size = pp->tx_ring_size;
		txq->done_pkts_coal = MVNETA_TXDONE_COAL_PKTS;
	}

	pp->rxqs = kcalloc(pp->rxq_number, sizeof(struct neta_rx_queue), GFP_KERNEL);
	if (!pp->rxqs)
		return -ENOMEM;

	/* Create Rx descriptor rings */
	for (queue = 0; queue < pp->rxq_number; queue++) {
		struct neta_rx_queue *rxq = &pp->rxqs[queue];

		rxq->id = queue;
		rxq->size = pp->rx_ring_size;
		rxq->pkts_coal = MVNETA_RX_COAL_PKTS;
		rxq->time_coal = MVNETA_RX_COAL_USEC;
	}

	return 0;
}

int neta_port_hw_deinit(struct neta_port *pp)
{
	return 0;
}

int neta_port_open(int port_id, struct neta_port *pp)
{
	return 0;
}
