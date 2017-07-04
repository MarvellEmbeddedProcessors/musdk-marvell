/******************************************************************************
 *	Copyright (C) 2017 Marvell International Ltd.
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
#include "neta_bm.h"
#include "neta_ppio.h"
#include "neta_hw.h"

/* Various constants */


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

/* Decrement sent descriptors counter */
void mvneta_txq_sent_desc_dec(struct neta_port *pp, int qid, int sent_desc)
{
	u32 val;

	/* Only 255 TX descriptors can be updated at once */
	while (sent_desc > 0xff) {
		val = 0xff << MVNETA_TXQ_DEC_SENT_SHIFT;
		neta_reg_write(pp, MVNETA_TXQ_UPDATE_REG(qid), val);
		sent_desc = sent_desc - 0xff;
	}

	val = sent_desc << MVNETA_TXQ_DEC_SENT_SHIFT;
	neta_reg_write(pp, MVNETA_TXQ_UPDATE_REG(qid), val);
}

/* Get number of TX descriptors already sent by HW */
int mvneta_txq_sent_desc_num_get(struct neta_port *pp, int qid)
{
	u32 val;
	int sent_desc;

	val = neta_reg_read(pp, MVNETA_TXQ_STATUS_REG(qid));
	sent_desc = (val & MVNETA_TXQ_SENT_DESC_MASK) >>
		MVNETA_TXQ_SENT_DESC_SHIFT;

	return sent_desc;
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
#if 0
	/* Set Port Acceleration Mode */
	if (pp->bm_priv)
		/* HW buffer management + legacy parser */
		val = MVNETA_ACC_MODE_EXT2;
	else
		/* SW buffer management + legacy parser */
		val = MVNETA_ACC_MODE_EXT1;
	neta_reg_write(pp, MVNETA_ACC_MODE, val);
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

int neta_port_hw_deinit(struct neta_port *pp)
{
	return 0;
}

int neta_port_open(int port_id, struct neta_port *pp)
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

	buf_size -= pp->rx_offset_correction;
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

/* Add number of descriptors ready to receive new packets */
static void mvneta_rxq_non_occup_desc_add(struct neta_port *pp,
					  struct neta_rx_queue *rxq,
					  int ndescs)
{
	/* Only MVNETA_RXQ_ADD_NON_OCCUPIED_MAX (255) descriptors can
	 * be added at once
	 */
	while (ndescs > MVNETA_RXQ_ADD_NON_OCCUPIED_MAX) {
		neta_reg_write(pp, MVNETA_RXQ_STATUS_UPDATE_REG(rxq->id),
			    (MVNETA_RXQ_ADD_NON_OCCUPIED_MAX <<
			     MVNETA_RXQ_ADD_NON_OCCUPIED_SHIFT));
		ndescs -= MVNETA_RXQ_ADD_NON_OCCUPIED_MAX;
	}

	neta_reg_write(pp, MVNETA_RXQ_STATUS_UPDATE_REG(rxq->id),
		    (ndescs << MVNETA_RXQ_ADD_NON_OCCUPIED_SHIFT));
}

/* Create a specified RX queue */
static int mvneta_rxq_init(struct neta_port *pp,
			   struct neta_rx_queue *rxq)

{
	rxq->size = pp->rx_ring_size;
	pp->rx_offset_correction = 8;

	/* Allocate memory for RX descriptors */
	rxq->descs =  mv_sys_dma_mem_alloc(rxq->size * MVNETA_DESC_ALIGNED_SIZE,
					   MVNETA_DESC_ALIGNED_SIZE);
	if (rxq->descs == NULL)
		return -ENOMEM;
	rxq->descs_phys = mv_sys_dma_mem_virt2phys(rxq->descs);
	rxq->last_desc = rxq->size - 1;

	/* Set Rx descriptors queue starting address */
	neta_reg_write(pp, MVNETA_RXQ_BASE_ADDR_REG(rxq->id), rxq->descs_phys);
	neta_reg_write(pp, MVNETA_RXQ_SIZE_REG(rxq->id), rxq->size);

	/* Set Offset */
	mvneta_rxq_offset_set(pp, rxq, pp->rx_offset_correction);

	mvneta_rxq_bm_enable(pp, rxq);
	mvneta_rxq_long_pool_set(pp, rxq);
	mvneta_rxq_short_pool_set(pp, rxq);
	mvneta_rxq_non_occup_desc_add(pp, rxq, rxq->size);

	return 0;
}

/* Cleanup Rx queue */
static void mvneta_rxq_deinit(struct neta_port *pp,
			      struct neta_rx_queue *rxq)
{
	/* TBD mvneta_rxq_drop_pkts(pp, rxq);*/

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
#ifdef CONFIG_64BIT
	void *data_tmp;

	/* In Neta HW only 32 bits data is supported, so in order to
	 * obtain whole 64 bits address from RX descriptor, we store the
	 * upper 32 bits when allocating buffer, and put it back
	 * when using buffer cookie for accessing packet in memory.
	 * Frags should be allocated from single 'memory' region, hence
	 * common upper address half should be sufficient.
	 */
	data_tmp = mvneta_frag_alloc(pp->frag_size);
	if (data_tmp) {
		pp->data_high = (u64)data_tmp & 0xffffffff00000000;
		mvneta_frag_free(pp->frag_size, data_tmp);
	}
#endif

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
	int ret;

	pp->pkt_size = MVNETA_RX_PKT_SIZE(1514);

	ret = mvneta_setup_rxqs(pp);
	if (ret)
		return ret;

	ret = mvneta_setup_txqs(pp);
	if (ret)
		goto err_cleanup_rxqs;

	/* Start port */
	mvneta_max_rx_size_set(pp, pp->pkt_size);
	mvneta_txq_max_tx_size_set(pp, pp->pkt_size);

	pp->bm_priv = pp->pool_short->priv;
	neta_reg_write(pp, MVNETA_ACC_MODE, MVNETA_ACC_MODE_EXT2);
	neta_reg_write(pp, MVNETA_BM_ADDRESS, pp->bm_priv->bppi_phys_addr);

	/* start the Rx/Tx activity */
	mvneta_port_enable(pp);

	return 0;

err_cleanup_rxqs:
	mvneta_cleanup_rxqs(pp);
	return ret;
}

/* Stop the port, free port interrupt line */
static int mvneta_stop(struct neta_port *pp)
{
	mvneta_cleanup_rxqs(pp);
	mvneta_cleanup_txqs(pp);

	return 0;
}

/* Initialize hw */
int neta_port_hw_init(struct neta_port *pp)
{
	int queue;

	/* Disable port */
	neta_port_disable(pp);

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
