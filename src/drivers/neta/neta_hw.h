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

#ifndef _MVNETA_HW_H_
#define _MVNETA_HW_H_

/* Registers */
#define MVNETA_RXQ_CONFIG_REG(q)                (0x1400 + ((q) << 2))
#define      MVNETA_RXQ_HW_BUF_ALLOC            BIT(0)
#define      MVNETA_RXQ_SHORT_POOL_ID_SHIFT	4
#define      MVNETA_RXQ_SHORT_POOL_ID_MASK	0x30
#define      MVNETA_RXQ_LONG_POOL_ID_SHIFT	6
#define      MVNETA_RXQ_LONG_POOL_ID_MASK	0xc0
#define      MVNETA_RXQ_PKT_OFFSET_ALL_MASK     (0xf    << 8)
#define      MVNETA_RXQ_PKT_OFFSET_MASK(offs)   ((offs) << 8)
#define MVNETA_RXQ_THRESHOLD_REG(q)             (0x14c0 + ((q) << 2))
#define      MVNETA_RXQ_NON_OCCUPIED(v)         ((v) << 16)
#define MVNETA_RXQ_BASE_ADDR_REG(q)             (0x1480 + ((q) << 2))
#define MVNETA_RXQ_SIZE_REG(q)                  (0x14a0 + ((q) << 2))
#define      MVNETA_RXQ_BUF_SIZE_SHIFT          19
#define      MVNETA_RXQ_BUF_SIZE_MASK           (0x1fff << 19)
#define MVNETA_RXQ_STATUS_REG(q)                (0x14e0 + ((q) << 2))
#define      MVNETA_RXQ_OCCUPIED_ALL_MASK       0x3fff
#define MVNETA_RXQ_STATUS_UPDATE_REG(q)         (0x1500 + ((q) << 2))
#define      MVNETA_RXQ_ADD_NON_OCCUPIED_SHIFT  16
#define      MVNETA_RXQ_ADD_NON_OCCUPIED_MAX    255
#define MVNETA_PORT_POOL_BUFFER_SZ_REG(pool)	(0x1700 + ((pool) << 2))
#define      MVNETA_PORT_POOL_BUFFER_SZ_SHIFT	3
#define      MVNETA_PORT_POOL_BUFFER_SZ_MASK	0xfff8
#define MVNETA_PORT_RX_RESET                    0x1cc0
#define      MVNETA_PORT_RX_DMA_RESET           BIT(0)
#define MVNETA_PHY_ADDR                         0x2000
#define      MVNETA_PHY_ADDR_MASK               0x1f
#define MVNETA_MBUS_RETRY                       0x2010
#define MVNETA_UNIT_INTR_CAUSE                  0x2080
#define MVNETA_UNIT_CONTROL                     0x20B0
#define      MVNETA_PHY_POLLING_ENABLE          BIT(1)
#define MVNETA_WIN_BASE(w)                      (0x2200 + ((w) << 3))
#define MVNETA_WIN_SIZE(w)                      (0x2204 + ((w) << 3))
#define MVNETA_WIN_REMAP(w)                     (0x2280 + ((w) << 2))
#define MVNETA_BASE_ADDR_ENABLE                 0x2290
#define MVNETA_ACCESS_PROTECT_ENABLE            0x2294
#define MVNETA_PORT_CONFIG                      0x2400
#define      MVNETA_UNI_PROMISC_MODE            BIT(0)
#define      MVNETA_DEF_RXQ(q)                  ((q) << 1)
#define      MVNETA_DEF_RXQ_ARP(q)              ((q) << 4)
#define      MVNETA_TX_UNSET_ERR_SUM            BIT(12)
#define      MVNETA_DEF_RXQ_TCP(q)              ((q) << 16)
#define      MVNETA_DEF_RXQ_UDP(q)              ((q) << 19)
#define      MVNETA_DEF_RXQ_BPDU(q)             ((q) << 22)
#define      MVNETA_RX_CSUM_WITH_PSEUDO_HDR     BIT(25)
#define      MVNETA_PORT_CONFIG_DEFL_VALUE(q)   (MVNETA_DEF_RXQ(q)       | \
						 MVNETA_DEF_RXQ_ARP(q)	 | \
						 MVNETA_DEF_RXQ_TCP(q)	 | \
						 MVNETA_DEF_RXQ_UDP(q)	 | \
						 MVNETA_DEF_RXQ_BPDU(q)	 | \
						 MVNETA_TX_UNSET_ERR_SUM | \
						 MVNETA_RX_CSUM_WITH_PSEUDO_HDR)
#define MVNETA_PORT_CONFIG_EXTEND                0x2404
#define MVNETA_MAC_ADDR_LOW                      0x2414
#define MVNETA_MAC_ADDR_HIGH                     0x2418
#define MVNETA_SDMA_CONFIG                       0x241c
#define      MVNETA_SDMA_BRST_SIZE_16            4
#define      MVNETA_RX_BRST_SZ_MASK(burst)       ((burst) << 1)
#define      MVNETA_RX_NO_DATA_SWAP              BIT(4)
#define      MVNETA_TX_NO_DATA_SWAP              BIT(5)
#define      MVNETA_DESC_SWAP                    BIT(6)
#define      MVNETA_TX_BRST_SZ_MASK(burst)       ((burst) << 22)
#define MVNETA_PORT_STATUS                       0x2444
#define      MVNETA_TX_IN_PRGRS                  BIT(1)
#define      MVNETA_TX_FIFO_EMPTY                BIT(8)
#define MVNETA_RX_MIN_FRAME_SIZE                 0x247c
#define MVNETA_SERDES_CFG			 0x24A0
#define      MVNETA_SGMII_SERDES_PROTO		 0x0cc7
#define      MVNETA_QSGMII_SERDES_PROTO		 0x0667
#define MVNETA_TYPE_PRIO                         0x24bc
#define      MVNETA_FORCE_UNI                    BIT(21)
#define MVNETA_TXQ_CMD_1                         0x24e4
#define MVNETA_TXQ_CMD                           0x2448
#define      MVNETA_TXQ_DISABLE_SHIFT            8
#define      MVNETA_TXQ_ENABLE_MASK              0x000000ff
#define MVNETA_RX_DISCARD_FRAME_COUNT		 0x2484
#define MVNETA_OVERRUN_FRAME_COUNT		 0x2488
#define MVNETA_GMAC_CLOCK_DIVIDER                0x24f4
#define      MVNETA_GMAC_1MS_CLOCK_ENABLE        BIT(31)
#define MVNETA_ACC_MODE                          0x2500
#define MVNETA_BM_ADDRESS                        0x2504
#define MVNETA_CPU_MAP(cpu)                      (0x2540 + ((cpu) << 2))
#define      MVNETA_CPU_RXQ_ACCESS_ALL_MASK      0x000000ff
#define      MVNETA_CPU_TXQ_ACCESS_ALL_MASK      0x0000ff00
#define      MVNETA_CPU_RXQ_ACCESS(rxq)		 BIT(rxq)
#define      MVNETA_CPU_TXQ_ACCESS(txq)		 BIT(txq + 8)
#define MVNETA_RXQ_TIME_COAL_REG(q)              (0x2580 + ((q) << 2))

/* Exception Interrupt Port/Queue Cause register
 *
 * Their behavior depend of the mapping done using the PCPX2Q
 * registers. For a given CPU if the bit associated to a queue is not
 * set, then for the register a read from this CPU will always return
 * 0 and a write won't do anything
 */

#define MVNETA_INTR_NEW_CAUSE                    0x25a0
#define MVNETA_INTR_NEW_MASK                     0x25a4

/* bits  0..7  = TXQ SENT, one bit per queue.
 * bits  8..15 = RXQ OCCUP, one bit per queue.
 * bits 16..23 = RXQ FREE, one bit per queue.
 * bit  29 = OLD_REG_SUM, see old reg ?
 * bit  30 = TX_ERR_SUM, one bit for 4 ports
 * bit  31 = MISC_SUM,   one bit for 4 ports
 */
#define      MVNETA_TX_INTR_MASK(nr_txqs)        (((1 << nr_txqs) - 1) << 0)
#define      MVNETA_TX_INTR_MASK_ALL             (0xff << 0)
#define      MVNETA_RX_INTR_MASK(nr_rxqs)        (((1 << nr_rxqs) - 1) << 8)
#define      MVNETA_RX_INTR_MASK_ALL             (0xff << 8)
#define      MVNETA_MISCINTR_INTR_MASK           BIT(31)

#define MVNETA_INTR_OLD_CAUSE                    0x25a8
#define MVNETA_INTR_OLD_MASK                     0x25ac

/* Data Path Port/Queue Cause Register */
#define MVNETA_INTR_MISC_CAUSE                   0x25b0
#define MVNETA_INTR_MISC_MASK                    0x25b4

#define      MVNETA_CAUSE_PHY_STATUS_CHANGE      BIT(0)
#define      MVNETA_CAUSE_LINK_CHANGE            BIT(1)
#define      MVNETA_CAUSE_PTP                    BIT(4)

#define      MVNETA_CAUSE_INTERNAL_ADDR_ERR      BIT(7)
#define      MVNETA_CAUSE_RX_OVERRUN             BIT(8)
#define      MVNETA_CAUSE_RX_CRC_ERROR           BIT(9)
#define      MVNETA_CAUSE_RX_LARGE_PKT           BIT(10)
#define      MVNETA_CAUSE_TX_UNDERRUN            BIT(11)
#define      MVNETA_CAUSE_PRBS_ERR               BIT(12)
#define      MVNETA_CAUSE_PSC_SYNC_CHANGE        BIT(13)
#define      MVNETA_CAUSE_SERDES_SYNC_ERR        BIT(14)

#define      MVNETA_CAUSE_BMU_ALLOC_ERR_SHIFT    16
#define      MVNETA_CAUSE_BMU_ALLOC_ERR_ALL_MASK   (0xF << MVNETA_CAUSE_BMU_ALLOC_ERR_SHIFT)
#define      MVNETA_CAUSE_BMU_ALLOC_ERR_MASK(pool) (1 << (MVNETA_CAUSE_BMU_ALLOC_ERR_SHIFT + (pool)))

#define      MVNETA_CAUSE_TXQ_ERROR_SHIFT        24
#define      MVNETA_CAUSE_TXQ_ERROR_ALL_MASK     (0xFF << MVNETA_CAUSE_TXQ_ERROR_SHIFT)
#define      MVNETA_CAUSE_TXQ_ERROR_MASK(q)      (1 << (MVNETA_CAUSE_TXQ_ERROR_SHIFT + (q)))

#define MVNETA_INTR_ENABLE                       0x25b8
#define      MVNETA_TXQ_INTR_ENABLE_ALL_MASK     0x0000ff00
#define      MVNETA_RXQ_INTR_ENABLE_ALL_MASK     0x000000ff

#define MVNETA_RXQ_CMD                           0x2680
#define      MVNETA_RXQ_DISABLE_SHIFT            8
#define      MVNETA_RXQ_ENABLE_MASK              0x000000ff
#define MVETH_TXQ_TOKEN_COUNT_REG(q)             (0x2700 + ((q) << 4))
#define MVETH_TXQ_TOKEN_CFG_REG(q)               (0x2704 + ((q) << 4))
#define MVNETA_GMAC_CTRL_0                       0x2c00
#define      MVNETA_GMAC_MAX_RX_SIZE_SHIFT       2
#define      MVNETA_GMAC_MAX_RX_SIZE_MASK        0x7ffc
#define      MVNETA_GMAC0_PORT_1000BASE_X        BIT(1)
#define      MVNETA_GMAC0_PORT_ENABLE            BIT(0)
#define MVNETA_GMAC_CTRL_2                       0x2c08
#define      MVNETA_GMAC2_SGMII_INBAND_AN_MODE   BIT(0)
#define      MVNETA_GMAC2_PCS_ENABLE             BIT(3)
#define      MVNETA_GMAC2_PORT_RGMII             BIT(4)
#define      MVNETA_GMAC2_PORT_RESET             BIT(6)
#define MVNETA_GMAC_STATUS                       0x2c10
#define      MVNETA_GMAC_LINK_UP                 BIT(0)
#define      MVNETA_GMAC_SPEED_1000              BIT(1)
#define      MVNETA_GMAC_SPEED_100               BIT(2)
#define      MVNETA_GMAC_FULL_DUPLEX             BIT(3)
#define      MVNETA_GMAC_RX_FLOW_CTRL_ENABLE     BIT(4)
#define      MVNETA_GMAC_TX_FLOW_CTRL_ENABLE     BIT(5)
#define      MVNETA_GMAC_RX_FLOW_CTRL_ACTIVE     BIT(6)
#define      MVNETA_GMAC_TX_FLOW_CTRL_ACTIVE     BIT(7)
#define MVNETA_GMAC_AUTONEG_CONFIG               0x2c0c
#define      MVNETA_GMAC_FORCE_LINK_DOWN         BIT(0)
#define      MVNETA_GMAC_FORCE_LINK_PASS         BIT(1)
#define      MVNETA_GMAC_INBAND_AN_ENABLE        BIT(2)
#define      MVNETA_GMAC_INBAND_AN_BYPASS_EN     BIT(3)
#define      MVNETA_GMAC_INBAND_RESTART_AN       BIT(4)
#define      MVNETA_GMAC_CONFIG_MII_SPEED        BIT(5)
#define      MVNETA_GMAC_CONFIG_GMII_SPEED       BIT(6)
#define      MVNETA_GMAC_AN_SPEED_EN             BIT(7)
#define      MVNETA_GMAC_CONFIG_FLOW_CTRL        BIT(8)
#define      MVNETA_GMAC_ADVERT_SYM_FLOW_CTRL    BIT(9)
#define      MVNETA_GMAC_ADVERT_ASYM_FC_ADV      BIT(10)
#define      MVNETA_GMAC_AN_FLOW_CTRL_EN         BIT(11)
#define      MVNETA_GMAC_CONFIG_FULL_DUPLEX      BIT(12)
#define      MVNETA_GMAC_AN_DUPLEX_EN            BIT(13)
#define MVNETA_MIB_COUNTERS_BASE                 0x3000
#define      MVNETA_MIB_LATE_COLLISION           0x7c
#define MVNETA_DA_FILT_SPEC_MCAST                0x3400
#define MVNETA_DA_FILT_OTH_MCAST                 0x3500
#define MVNETA_DA_FILT_UCAST_BASE                0x3600
#define MVNETA_TXQ_BASE_ADDR_REG(q)              (0x3c00 + ((q) << 2))
#define MVNETA_TXQ_SIZE_REG(q)                   (0x3c20 + ((q) << 2))
#define      MVNETA_TXQ_SENT_THRESH_ALL_MASK     0x3fff0000
#define      MVNETA_TXQ_SENT_THRESH_MASK(coal)   ((coal) << 16)
#define MVNETA_TXQ_UPDATE_REG(q)                 (0x3c60 + ((q) << 2))
#define      MVNETA_TXQ_DEC_SENT_SHIFT           16
#define MVNETA_TXQ_STATUS_REG(q)                 (0x3c40 + ((q) << 2))
#define      MVNETA_TXQ_SENT_DESC_SHIFT          16
#define      MVNETA_TXQ_SENT_DESC_MASK           0x3fff0000
#define MVNETA_PORT_TX_RESET                     0x3cf0
#define      MVNETA_PORT_TX_DMA_RESET            BIT(0)
#define MVNETA_TX_MTU                            0x3e0c
#define MVNETA_TX_TOKEN_SIZE                     0x3e14
#define      MVNETA_TX_TOKEN_SIZE_MAX            0xffffffff
#define MVNETA_TXQ_TOKEN_SIZE_REG(q)             (0x3e40 + ((q) << 2))
#define      MVNETA_TXQ_TOKEN_SIZE_MAX           0x7fffffff

#define MVNETA_CAUSE_TXQ_SENT_DESC_ALL_MASK	 0xff

#define MVNETA_REGS_GMAC_LEN                     0xAC9

enum neta_port_type {
	PORT_TYPE_SGMII,
	PORT_TYPE_1000BASE_X
};

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

#define MVNETA_TX_DISABLE_TIMEOUT_MSEC	1000
#define MVNETA_RX_DISABLE_TIMEOUT_MSEC	1000
#define MVNETA_TX_FIFO_EMPTY_TIMEOUT	10000

#define MVNETA_TX_CSUM_DEF_SIZE		1600
#define MVNETA_TX_CSUM_MAX_SIZE		9800
#define MVNETA_ACC_MODE_EXT1		1
#define MVNETA_ACC_MODE_EXT2		2

#define NETA_VLAN_TAG_LEN		4
#define NETA_ETH_HLEN			14
#define NETA_ETH_FCS_LEN		4

/* descriptor aligned size */
#define MVNETA_DESC_ALIGNED_SIZE	32

/* 64bytes cache line - arm64 spec */
#define L1_CACHE_LINE_BYTES      BIT(6)


static inline void neta_reg_write_relaxed(struct neta_port *pp, u32 offset, u32 data)
{
	uintptr_t addr = pp->base + offset;

	writel_relaxed(data, (void *)addr);

#ifdef NETA_REG_WRITE_DEBUG
	pr_info("%s: 0x%8lx + 0x%04x = 0x%x\n", __func__, pp->base, offset, data);
#endif
}

static inline u32 neta_reg_read_relaxed(struct neta_port *pp, u32 offset)
{
	uintptr_t addr = pp->base + offset;
	u32 data;

	data = readl_relaxed((void *)addr);

#ifdef NETA_REG_READ_DEBUG
	pr_info("%s: %8p + 0x%04x = 0x%x\n", __func__, pp->base, offset, data);
#endif

	return data;
}

static inline void neta_reg_write(struct neta_port *pp, u32 offset, u32 data)
{
	uintptr_t addr = pp->base + offset;

	writel(data, (void *)addr);

#ifdef NETA_REG_WRITE_DEBUG
	pr_info("%s: 0x%8lx + 0x%04x = 0x%x\n", __func__, pp->base, offset, data);
#endif
}

static inline u32 neta_reg_read(struct neta_port *pp, u32 offset)
{
	uintptr_t addr = pp->base + offset;
	u32 data;

	data = readl((void *)addr);

#ifdef NETA_REG_READ_DEBUG
	pr_info("%s : %8p + 0x%04x = 0x%x\n", __func__, base, offset, data);
#endif

	return data;
}

/* Get number of TX descriptors already sent by HW */
static inline int mvneta_txq_sent_desc_num_get(struct neta_port *pp, int qid)
{
	u32 val;
	int sent_desc;

	val = neta_reg_read(pp, MVNETA_TXQ_STATUS_REG(qid));
	sent_desc = (val & MVNETA_TXQ_SENT_DESC_MASK) >>
		MVNETA_TXQ_SENT_DESC_SHIFT;

	return sent_desc;
}

/* Decrement sent descriptors counter */
static inline void mvneta_txq_sent_desc_dec(struct neta_port *pp, int qid, int sent_desc)
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

/* Update HW with number of TX descriptors to be sent */
static inline void neta_txq_pend_desc_add(struct neta_port *pp,
				   struct neta_tx_queue *txq,
				   int pend_desc)
{
	u32 val;

	val = pend_desc;

	/* Only 255 TX descriptors can be updated at once */
	while (val > 0xff) {
		neta_reg_write(pp, MVNETA_TXQ_UPDATE_REG(txq->id), 0xff);
		val -= 0xff;
	}

	neta_reg_write(pp, MVNETA_TXQ_UPDATE_REG(txq->id), val);
}

/* Update num of rx desc called upon return from rx path */
static inline void neta_port_inq_update(struct neta_port *pp,
			  struct neta_rx_queue *rxq,
			  int rx_done, int rx_filled)
{
	u32 val;

	if ((rx_done <= 0xff) && (rx_filled <= 0xff)) {
		val = rx_done |
		  (rx_filled << MVNETA_RXQ_ADD_NON_OCCUPIED_SHIFT);
		neta_reg_write(pp, MVNETA_RXQ_STATUS_UPDATE_REG(rxq->id), val);
		return;
	}

	/* do one write barrier and use relaxed write in loop */
	wmb();

	/* Only 255 descriptors can be added at once */
	while ((rx_done > 0) || (rx_filled > 0)) {
		if (rx_done <= 0xff) {
			val = rx_done;
			rx_done = 0;
		} else {
			val = 0xff;
			rx_done -= 0xff;
		}
		if (rx_filled <= 0xff) {
			val |= rx_filled << MVNETA_RXQ_ADD_NON_OCCUPIED_SHIFT;
			rx_filled = 0;
		} else {
			val |= 0xff << MVNETA_RXQ_ADD_NON_OCCUPIED_SHIFT;
			rx_filled -= 0xff;
		}
		neta_reg_write(pp, MVNETA_RXQ_STATUS_UPDATE_REG(rxq->id), val);
	}
}

/* Get number of RX descriptors occupied by received packets */
static inline int neta_rxq_busy_desc_num_get(struct neta_port *pp, int qid)
{
	int val;

	val = neta_reg_read(pp, MVNETA_RXQ_STATUS_REG(qid));
	return val & MVNETA_RXQ_OCCUPIED_ALL_MASK;
}

int neta_port_map(int port_id, struct neta_port *pp);
int neta_port_hw_init(struct neta_port *pp);
int neta_port_hw_deinit(struct neta_port *pp);
void neta_hw_reg_print(char *reg_name, void *base, u32 offset);
void neta_bm_pool_bufsize_set(struct neta_port *pp, int buf_size, u8 pool_id);
void neta_port_up(struct neta_port *pp);
void neta_port_down(struct neta_port *pp);

#endif /* _MVNETA_HW_H_ */
