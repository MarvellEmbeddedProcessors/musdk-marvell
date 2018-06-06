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
 * @file pp2_port.c
 *
 * Port I/O routines
 */

#include "std_internal.h"

#include "pp2_types.h"

#include "pp2_hw_type.h"
#include "pp2.h"
#include "pp2_dm.h"
#include "pp2_port.h"
#include "pp2_bm.h"
#include "cls/pp2_hw_cls.h"
#include "cls/pp2_cls_mng.h"
#include "cls/pp2_rss.h"


struct pp2_fc_values pp2_fc[3] = {
				{
					.port_mtu = 1500,
					.pool_stop_threshold = 256,
					.pool_start_threshold = 512,
					.rxq_stop_thresh = 256,
					.rxq_start_thresh = 512
				},
				{
					.port_mtu = 4000,
					.pool_stop_threshold = 512,
					.pool_start_threshold = 768,
					.rxq_stop_thresh = 512,
					.rxq_start_thresh = 768
				},
				{
					.port_mtu = 9000,
					.pool_stop_threshold = 768,
					.pool_start_threshold = 1024,
					.rxq_stop_thresh = 768,
					.rxq_start_thresh = 1024
				}
};

/*
 * pp2_port.c
 *
 * Implements configuration and run-time Port and I/O routines
 */

static struct pp2_tc *pp2_rxq_tc_get(struct pp2_port *port, uint32_t id)
{
	u8 i;

	for (i = 0; i < port->num_tcs; i++) {
		u8 first_rxq = port->tc[i].tc_config.first_rxq;

		if (id >= first_rxq && id < (first_rxq + port->tc[i].tc_config.num_in_qs))
			return &port->tc[i];
	}
	return NULL;
}

/* Set TX FIFO size and threshold */
static inline void pp2_port_tx_fifo_config(struct pp2_port *port,
					   u32 fifo_size, uint32_t fifo_thr)
{
	/* TX FIFO size */
	pp2_reg_write(port->cpu_slot, MVPP22_TX_FIFO_SIZE_REG(port->id),
		      fifo_size & MVPP22_TX_FIFO_SIZE_MASK);

	/* TX FIFO threshold */
	pp2_reg_write(port->cpu_slot, MVPP22_TX_FIFO_THRESH_REG(port->id),
		      fifo_thr & MVPP22_TX_FIFO_THRESH_MASK);
}

/* Set TX FIFO size and threshold */
static inline uint32_t pp2_port_get_tx_fifo(struct pp2_port *port)
{
	return (MVPP22_TX_FIFO_SIZE_MASK & pp2_reg_read(port->cpu_slot, MVPP22_TX_FIFO_SIZE_REG(port->id)));
}

static void
pp2_rxq_offset_set(struct pp2_port *port,
		   int prxq, int offset)
{
	u32 val;
	uintptr_t cpu_slot = port->cpu_slot;

	/* Convert offset from bytes to units of 32 bytes */
	offset = offset >> 5;

	val = pp2_reg_read(cpu_slot, MVPP2_RXQ_CONFIG_REG(prxq));
	val &= ~MVPP2_RXQ_PACKET_OFFSET_MASK;

	/* Offset is in */
	val |= ((offset << MVPP2_RXQ_PACKET_OFFSET_OFFS) &
		MVPP2_RXQ_PACKET_OFFSET_MASK);

	pp2_reg_write(cpu_slot, MVPP2_RXQ_CONFIG_REG(prxq), val);
}

static void
pp2_port_egress_disable(struct pp2_port *port)
{
	volatile u32 tmo;
	u32 val = 0;
	u32 tx_port_num  = MVPP2_MAX_TCONT + port->id;
	uintptr_t cpu_slot = port->cpu_slot;

	/* Issue stop command for active channels only */
	pp2_reg_write(cpu_slot, MVPP2_TXP_SCHED_PORT_INDEX_REG, tx_port_num);
	val = (pp2_reg_read(cpu_slot, MVPP2_TXP_SCHED_Q_CMD_REG) & MVPP2_TXP_SCHED_ENQ_MASK);
	if (val)
		pp2_reg_write(cpu_slot, MVPP2_TXP_SCHED_Q_CMD_REG, val << MVPP2_TXP_SCHED_DISQ_OFFSET);

	/* TXQs disable. Wait for all Tx activity to terminate. */
	tmo = 0;
	do {
		if (tmo >= MVPP2_TX_DISABLE_TIMEOUT_MSEC) {
			pr_warn("Port: Egress disable timeout = 0x%08X\n", val);
			break;
		}
		/* Sleep for 1 millisecond */
		usleep_range(1000, 2000);
		tmo++;
		val = pp2_reg_read(cpu_slot, MVPP2_TXP_SCHED_Q_CMD_REG);
	} while (val & MVPP2_TXP_SCHED_ENQ_MASK);
}

static void
pp2_port_egress_enable(struct pp2_port *port)
{
	u32 qmap = 0;
	u32 queue;
	u32 tx_port_num = MVPP2_MAX_TCONT + port->id;
	uintptr_t cpu_slot = port->cpu_slot;

	/* TXQs enable */
	for (queue = 0; queue < port->num_tx_queues; queue++) {
		struct pp2_tx_queue *txq = port->txqs[queue];

		if (txq->desc_virt_arr)
			qmap |= (1 << queue);
	}
	pp2_reg_write(cpu_slot, MVPP2_TXP_SCHED_PORT_INDEX_REG, tx_port_num);
	pp2_reg_write(cpu_slot, MVPP2_TXP_SCHED_Q_CMD_REG, qmap);
	pr_debug("Port: Egress enable tx_port_num=%u qmap=0x%X\n", tx_port_num, qmap);
}

static void
pp2_port_ingress_enable(struct pp2_port *port)
{
	u32 val;
	u32 rxq, qid;
	uintptr_t cpu_slot = port->cpu_slot;

	/* RXQs enable */
	for (rxq = 0; rxq < port->num_rx_queues; rxq++) {
		qid = port->rxqs[rxq]->id;
		val = pp2_reg_read(cpu_slot, MVPP2_RXQ_CONFIG_REG(qid));
		val &= ~MVPP2_RXQ_DISABLE_MASK;
		pp2_reg_write(cpu_slot, MVPP2_RXQ_CONFIG_REG(qid), val);
	}
}

static void
pp2_port_ingress_disable(struct pp2_port *port)
{
	u32 val;
	u32 rxq, qid;
	uintptr_t cpu_slot = port->cpu_slot;

	/* RXQs disable */
	for (rxq = 0; rxq < port->num_rx_queues; rxq++) {
		qid = port->rxqs[rxq]->id;
		val = pp2_reg_read(cpu_slot, MVPP2_RXQ_CONFIG_REG(qid));
		val |= MVPP2_RXQ_DISABLE_MASK;
		pp2_reg_write(cpu_slot, MVPP2_RXQ_CONFIG_REG(qid), val);
	}
}

static void
pp2_port_interrupts_disable(struct pp2_port *port)
{
	u32 mask = 0;
	uintptr_t cpu_slot = port->cpu_slot;

	mask = pp2_hif_map_get();

	pp2_reg_write(cpu_slot, MVPP2_ISR_ENABLE_REG(port->id),
		      MVPP2_ISR_DISABLE_INTERRUPT(mask));
}


/* Configure Rx FIFO Flow control thresholds */
static void pp2_port_rx_fifo_fc_en(struct pp2_port *port, bool en)
{
	int val;
	uintptr_t cpu_slot = port->cpu_slot;

	val = pp2_reg_read(cpu_slot, MVPP2_RX_FLOW_CONTROL_REG(port->id));

	if (en)
		val |= MVPP2_RX_FLOW_CONTROL_EN;
	else
		val &= ~MVPP2_RX_FLOW_CONTROL_EN;

	pp2_reg_write(cpu_slot, MVPP2_RX_FLOW_CONTROL_REG(port->id), val);
}

static inline void pp2_port_isr_rx_group_write(struct pp2_port *port, int sub_group, int start_queue, int num_rx_queues)
{
	int val;
	uintptr_t cpu_slot = port->cpu_slot;

	val = (port->id << MVPP22_ISR_RXQ_GROUP_INDEX_GROUP_OFFSET) | sub_group;
	pp2_reg_write(cpu_slot,  MVPP22_ISR_RXQ_GROUP_INDEX_REG, val);
	val = (num_rx_queues << MVPP22_ISR_RXQ_SUB_GROUP_SIZE_OFFSET) | start_queue;
	pp2_reg_write(cpu_slot, MVPP22_ISR_RXQ_SUB_GROUP_CONFIG_REG, val);
}

static int pp2_port_check_mtu_valid(struct pp2_port *port, uint32_t mtu)
{
	u32 tx_fifo_threshold;

	/* Validate MTU */
	if (mtu < PP2_PORT_MIN_MTU) {
		pr_err("PORT: cannot change MTU to less than %u bytes\n", PP2_PORT_MIN_MTU);
		return -EINVAL;
	}

	/* Check MTU can be l4_checksummed */
	tx_fifo_threshold = PP2_PORT_TX_FIFO_KB_TO_THRESH(port->tx_fifo_size);
	if (MVPP2_MTU_PKT_SIZE(mtu) > tx_fifo_threshold) {
		port->flags &= ~PP2_PORT_FLAGS_L4_CHKSUM;
		pr_warn("PORT: mtu=%u, mtu_pkt_size=%u, tx_fifo_thresh=%u, port discontinues hw_l4_checksum support\n",
			 mtu, MVPP2_MTU_PKT_SIZE(mtu), tx_fifo_threshold);
	} else {
		port->flags |= PP2_PORT_FLAGS_L4_CHKSUM;
	}

	/* Buffer_pool sizes are not relevant for mtu, only mru. */

	return 0;
}

static void
pp2_port_mac_max_rx_size_set(struct pp2_port *port)
{
	struct gop_hw *gop = &port->parent->hw.gop;
	struct pp2_mac_data *mac = &port->mac_data;
	int mac_num = port->mac_data.gop_index;

	switch (mac->phy_mode) {
	case PP2_PHY_INTERFACE_MODE_RGMII:
	case PP2_PHY_INTERFACE_MODE_SGMII:
	case PP2_PHY_INTERFACE_MODE_QSGMII:
		pp2_gop_gmac_max_rx_size_set(gop, mac_num,
					     port->port_mru);
		 break;
	case PP2_PHY_INTERFACE_MODE_XAUI:
	case PP2_PHY_INTERFACE_MODE_RXAUI:
	case PP2_PHY_INTERFACE_MODE_KR:
		pp2_gop_xlg_mac_max_rx_size_set(gop,
						mac_num, port->port_mru);
		break;
	default:
		break;
	}
}

static void
pp2_port_mac_set_loopback(struct pp2_port *port, int en)
{
	struct gop_hw *gop = &port->parent->hw.gop;
	struct pp2_mac_data *mac = &port->mac_data;
	int mac_num = port->mac_data.gop_index;
	enum pp2_lb_type lb = (en) ? PP2_TX_2_RX_LB : PP2_DISABLE_LB;

	switch (mac->phy_mode) {
	case PP2_PHY_INTERFACE_MODE_RGMII:
	case PP2_PHY_INTERFACE_MODE_SGMII:
	case PP2_PHY_INTERFACE_MODE_QSGMII:
		pp2_gop_gmac_loopback_cfg(gop, mac_num, lb);
		break;
	case PP2_PHY_INTERFACE_MODE_XAUI:
	case PP2_PHY_INTERFACE_MODE_RXAUI:
	case PP2_PHY_INTERFACE_MODE_KR:
		pp2_gop_xlg_mac_loopback_cfg(gop, mac_num, lb);
		break;
	default:
		break;
	}
}

/* Get number of Tx descriptors waiting to be transmitted by HW */
static uint32_t
pp2_txq_pend_desc_num_get(struct pp2_port *port,
			  struct pp2_tx_queue *txq)
{
	u32 val;
	uintptr_t cpu_slot = port->cpu_slot;

	pp2_reg_write(cpu_slot, MVPP2_TXQ_NUM_REG, txq->id);
	val = pp2_reg_read(cpu_slot, MVPP2_TXQ_PENDING_REG);

	return val & MVPP2_TXQ_PENDING_MASK;
}

/* Set defaults to the MVPP2 port */
static void
pp2_port_defaults_set(struct pp2_port *port)
{
	u32 tx_port_num, val, queue, ptxq, lrxq;
	struct pp2_inst *inst = port->parent;
	struct pp2_hw *hw = &inst->hw;
	uintptr_t cpu_slot = port->cpu_slot;

	/* Disable Legacy WRR, Disable EJP, Release from reset */
	tx_port_num = MVPP2_MAX_TCONT + port->id;
	pp2_reg_write(cpu_slot, MVPP2_TXP_SCHED_PORT_INDEX_REG, tx_port_num);
	pp2_reg_write(cpu_slot, MVPP2_TXP_SCHED_CMD_1_REG, 0x0);

	/* Close bandwidth for all queues */
	for (queue = 0; queue < MVPP2_MAX_TXQ; queue++) {
		ptxq = (MVPP2_MAX_TCONT + port->id) * MVPP2_MAX_TXQ + queue;
		pp2_reg_write(cpu_slot, MVPP2_TXQ_SCHED_TOKEN_CNTR_REG(ptxq), 0x0);
	}

	/* Set refill period to 1 usec, refill tokens
	* and bucket size to maximum
	*/
	pp2_reg_write(cpu_slot, MVPP2_TXP_SCHED_PERIOD_REG, hw->tclk / 1000000); /* USEC_PER_SEC */
	val = pp2_reg_read(cpu_slot, MVPP2_TXP_SCHED_REFILL_REG);
	val &= ~MVPP2_TXP_REFILL_PERIOD_ALL_MASK;
	val |= MVPP2_TXP_REFILL_PERIOD_MASK(1);
	val |= MVPP2_TXP_REFILL_TOKENS_ALL_MASK;
	pp2_reg_write(cpu_slot, MVPP2_TXP_SCHED_REFILL_REG, val);
	val = MVPP2_TXP_TOKEN_SIZE_MAX;
	pp2_reg_write(cpu_slot, MVPP2_TXP_SCHED_TOKEN_SIZE_REG, val);

	/* Set MaximumLowLatencyPacketSize value to 256 */
	/* Set GemPortIdSrcSel from classifier */
	pp2_reg_write(cpu_slot, MVPP2_RX_CTRL_REG(port->id),
		      MVPP2_RX_USE_PSEUDO_FOR_CSUM_MASK |
		      MVPP2_RX_LOW_LATENCY_PKT_SIZE(256) |
		      MVPP2_RX_GEM_PORT_ID_SRC_SEL(2));

	/* Disable Rx cache snoop */
	for (lrxq = 0; lrxq < port->num_rx_queues; lrxq++) {
		queue = port->rxqs[lrxq]->id;
		val = pp2_reg_read(cpu_slot, MVPP2_RXQ_CONFIG_REG(queue));
		/* Coherent */
		val |= MVPP2_SNOOP_PKT_SIZE_MASK;
		val |= MVPP2_SNOOP_BUF_HDR_MASK;
		pp2_reg_write(cpu_slot, MVPP2_RXQ_CONFIG_REG(queue), val);
	}
	/* As default, mask all interrupts to all present cpus */
	pp2_port_interrupts_disable(port);
}

/* Per-TXQ hardware related initialization
 * Hardware access
 */
static void
pp2_txq_init(struct pp2_port *port, struct pp2_tx_queue *txq)
{
	uintptr_t cpu_slot;
	u32 j, val, desc_per_txq, pref_buf_size, desc;
	struct pp2_hw *hw;

	hw = &port->parent->hw;
	cpu_slot = port->cpu_slot;

	if (LPBK_PORT(port))
		desc_per_txq = PP2_LOOPBACK_PORT_TXQ_PREFETCH;
	else
		desc_per_txq = PP2_ETH_PORT_TXQ_PREFETCH;

	/* FS_A8K Table 1542: The SWF ring size + a prefetch size for HWF */
	txq->desc_total = port->txq_config[txq->log_id].size;
	txq->desc_virt_arr = mv_sys_dma_mem_region_alloc(port->tx_qs_mem, (txq->desc_total * MVPP2_DESC_ALIGNED_SIZE),
							  MVPP2_DESC_Q_ALIGN);
	if (unlikely(!txq->desc_virt_arr)) {
		pr_err("PP: cannot allocate egress descriptor array\n");
		return;
	}
	txq->desc_phys_arr = (uintptr_t)mv_sys_dma_mem_region_virt2phys(port->tx_qs_mem, txq->desc_virt_arr);
	if (!IS_ALIGNED(txq->desc_phys_arr, MVPP2_DESC_Q_ALIGN)) {
		pr_err("PP: egress descriptor array must be %u-byte aligned\n",
			MVPP2_DESC_Q_ALIGN);
		mv_sys_dma_mem_region_free(port->tx_qs_mem, txq->desc_virt_arr);
		return;
	}

	pr_debug("port[%d:%d] tx desc_phys_addr(0x%lx)\n", port->parent->id, port->id, txq->desc_phys_arr);

	/* Set Tx descriptors queue starting address - indirect access */
	pp2_reg_write(cpu_slot, MVPP2_TXQ_NUM_REG, txq->id);
	pp2_reg_write(cpu_slot, MVPP2_TXQ_DESC_ADDR_LOW_REG,
		      ((uint32_t)txq->desc_phys_arr) >> MVPP2_TXQ_DESC_ADDR_LOW_SHIFT);
	pp2_reg_write(cpu_slot, MVPP22_TXQ_DESC_ADDR_HIGH_REG,
		      0x00 & MVPP22_TXQ_DESC_ADDR_HIGH_MASK);
	pp2_reg_write(cpu_slot, MVPP2_TXQ_DESC_SIZE_REG,
		      txq->desc_total & MVPP2_TXQ_DESC_SIZE_MASK);
	pp2_reg_write(cpu_slot, MVPP2_TXQ_INDEX_REG, 0x0);
	pp2_reg_write(cpu_slot, MVPP2_TXQ_RSVD_CLR_REG,
		      txq->id << MVPP2_TXQ_RSVD_CLR_OFFSET);
	val = pp2_reg_read(cpu_slot, MVPP2_TXQ_PENDING_REG);
	val &= ~MVPP2_TXQ_PENDING_MASK;
	pp2_reg_write(cpu_slot, MVPP2_TXQ_PENDING_REG, val);

	/* Calculate base address in prefetch buffer. We reserve 16 descriptors
	* for each existing TXQ.
	* - TCONTS for PON port must be continuous from 0 to MVPP2_MAX_TCONT
	* - GBE ports assumed to be continious from 0 to MVPP2_MAX_PORTS
	*/
	if (desc_per_txq == PP2_TXQ_PREFETCH_64)
		pref_buf_size = MVPP2_PREF_BUF_SIZE_64;
	else if (desc_per_txq == PP2_TXQ_PREFETCH_32)
		pref_buf_size = MVPP2_PREF_BUF_SIZE_32;
	else if (desc_per_txq == PP2_TXQ_PREFETCH_16)
		pref_buf_size = MVPP2_PREF_BUF_SIZE_16;
	else
		pref_buf_size = MVPP2_PREF_BUF_SIZE_4;

	/* Since the loopback port is the last port, below calc. is always correct */
	desc = (port->id * MVPP2_MAX_TXQ * PP2_ETH_PORT_TXQ_PREFETCH) + (txq->log_id * desc_per_txq);

	 /* Set desc prefetch threshold to 8 units of 2 descriptors */
	 pp2_reg_write(cpu_slot, MVPP2_TXQ_PREF_BUF_REG,
		       MVPP2_PREF_BUF_PTR(desc) | pref_buf_size |
		 MVPP2_PREF_BUF_THRESH(PP2_TXQ_PREFETCH_16 / 2));

	/* Lastly, clear all ETH_TXQS for all future DM-IFs */
	for (j = 0; j < PP2_NUM_REGSPACES; j++) {
		cpu_slot = hw->base[j].va;
		pp2_reg_read(cpu_slot, MVPP22_TXQ_SENT_REG(txq->id));
	}

	memset(&txq->stats, 0, sizeof(txq->stats));
	txq->threshold_tx_pkts = 0;
}

/* Initializes and sets TXQ related registers for all TXQs
 * Hardware access
 */
static void
pp2_port_txqs_init(struct pp2_port *port)
{
	u32 qid;

	for (qid = 0; qid < port->num_tx_queues; qid++) {
		struct pp2_tx_queue *txq = port->txqs[qid];

		pp2_txq_init(port, txq);
		pp2_port_set_outq_state(port, txq, true);
	}
}

/* Allocates and sets control data for TXQs
 * No hardware access
 */
void
pp2_port_txqs_create(struct pp2_port *port)
{
	u32 qid;

	for (qid = 0; qid < port->num_tx_queues; qid++) {
		struct pp2_tx_queue *txq = kcalloc(1, sizeof(struct pp2_tx_queue), GFP_KERNEL);

		if (unlikely(!txq)) {
			pr_err("%s out of memory txq alloc\n", __func__);
			return;
		}

		txq->id = (MVPP2_MAX_TCONT + port->id) * MVPP2_MAX_TXQ + qid;
		txq->log_id = qid;
		port->txqs[qid] = txq;
	}
}

/* Deallocates all TXQs for this port
 * No hardware access
 */
static void
pp2_port_txqs_destroy(struct pp2_port *port)
{
	u32 qid;

	for (qid = 0; qid < port->num_tx_queues; qid++) {
		struct pp2_tx_queue *txq = port->txqs[qid];

		mv_sys_dma_mem_region_free(port->tx_qs_mem, txq->desc_virt_arr);
		kfree(txq);
	}
}

static inline void
pp2_rxq_update_next_desc_idx(struct pp2_rx_queue *rxq, uint32_t num_sent)
{
	u32 rx_idx;

	if (unlikely((num_sent < 1) || (num_sent > rxq->desc_total))) {
		pr_err("RxDesc number inconsistent\n");
		return;
	}

	rx_idx = rxq->desc_next_idx;

	if (likely((rx_idx + num_sent) < rxq->desc_total))
		rxq->desc_next_idx = rx_idx + num_sent;
	else
		rxq->desc_next_idx = rx_idx + num_sent - rxq->desc_total;

	pr_debug("%s\t: cur_idx=%d\tnext_idx=%d\n", __func__, rx_idx, rxq->desc_next_idx);
}

/* External:
 * Update Rx queue status with the number of occupied and available
 * Rx descriptor slots.
 */
void
pp2_port_inq_update(struct pp2_port *port, uint32_t in_qid,
		    u32 used_count, uint32_t free_count)
{
	/* Decrement the number of used descriptors and increment the
	* number of free descriptors
	*/
	u32 id = port->rxqs[in_qid]->id;
	u32 val = used_count | (free_count << MVPP2_RXQ_NUM_NEW_OFFSET);
	uintptr_t cpu_slot = port->cpu_slot;

	/* pp2_rxq_update_next_desc_idx(port->rxqs[in_qid], used_count); */

	pp2_reg_write(cpu_slot, MVPP2_RXQ_STATUS_UPDATE_REG(id), val);
}

/* Per-RXQ hardware related initialization
 * Hardware access
 */
static void
pp2_rxq_init(struct pp2_port *port, struct pp2_rx_queue *rxq)
{
	u32 val;
	uintptr_t cpu_slot;
	struct pp2_tc *tc;

	cpu_slot = port->cpu_slot;

	rxq->desc_virt_arr = mv_sys_dma_mem_region_alloc(rxq->mem, (rxq->desc_total * MVPP2_DESC_ALIGNED_SIZE),
							  MVPP2_DESC_Q_ALIGN);
	if (unlikely(!rxq->desc_virt_arr)) {
		pr_err("PP: cannot allocate ingress descriptor array\n");
		return;
	}
	rxq->desc_phys_arr = (uintptr_t)mv_sys_dma_mem_region_virt2phys(rxq->mem, rxq->desc_virt_arr);
	if (!IS_ALIGNED(rxq->desc_phys_arr, MVPP2_DESC_Q_ALIGN)) {
		pr_err("PP: ingress descriptor array must be %u-byte aligned\n",
			MVPP2_DESC_Q_ALIGN);
		mv_sys_dma_mem_region_free(rxq->mem, rxq->desc_virt_arr);
		return;
	}
	pr_debug("port[%d:%d] rxq[%d], desc_phys_addr(0x%lx)\n", port->parent->id, port->id, rxq->id,
		 rxq->desc_phys_arr);
	rxq->desc_last_idx = rxq->desc_total - 1;

	/* Zero occupied and non-occupied counters - direct access */
	pp2_reg_write(cpu_slot, MVPP2_RXQ_STATUS_REG(rxq->id), 0x0);

	/* Set Rx descriptors queue starting address - indirect access */
	pp2_reg_write(cpu_slot, MVPP2_RXQ_NUM_REG, rxq->id);

	pp2_reg_write(cpu_slot, MVPP2_RXQ_DESC_ADDR_REG,
		      (rxq->desc_phys_arr >> MVPP22_DESC_ADDR_SHIFT));
	pp2_reg_write(cpu_slot, MVPP2_RXQ_DESC_SIZE_REG, rxq->desc_total);
	pp2_reg_write(cpu_slot, MVPP2_RXQ_INDEX_REG, 0x0);

	tc = pp2_rxq_tc_get(port, rxq->id);
	if (!tc) {
		pr_err("port(%d) phy_rxq(%d), not found in tc range\n", port->id, rxq->id);
		return;
	}
	/* Set Offset */
	pp2_rxq_offset_set(port, rxq->id,  tc->tc_config.pkt_offset);

	pp2_bm_pool_assign(port, rxq->bm_pool_id[BM_TYPE_SHORT_BUF_POOL], rxq->id, BM_TYPE_SHORT_BUF_POOL);
	pp2_bm_pool_assign(port, rxq->bm_pool_id[BM_TYPE_LONG_BUF_POOL],  rxq->id, BM_TYPE_LONG_BUF_POOL);
	pr_debug("port[%d:%d] rxq[%d], short_pool(%d), long_pool(%d)\n", port->parent->id, port->id, rxq->id,
			  rxq->bm_pool_id[BM_TYPE_SHORT_BUF_POOL], rxq->bm_pool_id[BM_TYPE_LONG_BUF_POOL]);

	/* Add number of descriptors ready for receiving packets */
	val = (0 | (rxq->desc_total << MVPP2_RXQ_NUM_NEW_OFFSET));
	pp2_reg_write(cpu_slot, MVPP2_RXQ_STATUS_UPDATE_REG(rxq->id), val);

	memset(&rxq->stats, 0, sizeof(rxq->stats));
	rxq->threshold_rx_pkts = 0;
}

/* Initializes and sets RXQ related registers for all RXQs
 * Hardware access
 */
static void
pp2_port_rxqs_init(struct pp2_port *port)
{
	u32 qid;

	for (qid = 0; qid < port->num_rx_queues; qid++) {
		struct pp2_rx_queue *rxq = port->rxqs[qid];

		pp2_rxq_init(port, rxq);
	}
}

/* Allocates and sets control data for TXQs
 * No hardware access
 */
void
pp2_port_rxqs_create(struct pp2_port *port)
{
	u32 qid, tc, id = 0;

	for (tc = 0; tc < port->num_tcs; tc++) {
		struct pp2_ppio_tc_config *tc_cfg = &(port->tc[tc].tc_config);

		for (qid = 0; qid < port->tc[tc].tc_config.num_in_qs; qid++) {
			struct pp2_rx_queue *rxq = kcalloc(1, sizeof(struct pp2_rx_queue), GFP_KERNEL);
			u8 mem_index = port->tc[tc].rx_qs[qid].tc_pools_mem_id_index;
			u32 tmp_bpool_id;

			if (unlikely(!rxq)) {
				pr_err("%s out of memory rxq alloc\n", __func__);
				return;
			}
			rxq->id = tc_cfg->first_rxq + qid;
			rxq->log_id = port->tc[tc].first_log_rxq + qid;
			rxq->desc_total = port->tc[tc].rx_qs[qid].ring_size;
			rxq->mem = port->tc[tc].rx_qs[qid].mem;

			tmp_bpool_id = tc_cfg->pools[mem_index][BM_TYPE_SHORT_BUF_POOL]->bm_pool_id;
			rxq->bm_pool_id[BM_TYPE_SHORT_BUF_POOL] = tmp_bpool_id;

			tmp_bpool_id = tc_cfg->pools[mem_index][BM_TYPE_LONG_BUF_POOL]->bm_pool_id;
			rxq->bm_pool_id[BM_TYPE_LONG_BUF_POOL] = tmp_bpool_id;

			pr_debug("pp2_port_rxqs_create: port[%d:%d] tc%d rxq%d mem_index(%d)\n",
				port->parent->id, port->id, tc, rxq->id, mem_index);

			/* Double check of queue index */
			if (rxq->log_id != id) {
				pr_err("%s invalid log_id %d value (should be %d)\n",
					__func__, rxq->log_id, id);
				return;
			}
			/*TODO: are we really serializing the queue????? */
			port->rxqs[rxq->log_id] = rxq;
			id++;
		}
	}
}

/* Deallocates all TXQs for this port
 * No hardware access
 */
static void
pp2_port_rxqs_destroy(struct pp2_port *port)
{
	u32 qid;

	for (qid = 0; qid < port->num_rx_queues; qid++) {
		struct pp2_rx_queue *rxq = port->rxqs[qid];

		mv_sys_dma_mem_region_free(rxq->mem, rxq->desc_virt_arr);
		kfree(rxq);
	}
}

/* Get pointer to the next RX descriptor to be processed by SW, and update the descriptor next index */
struct pp2_desc *
pp2_rxq_get_desc(struct pp2_rx_queue *rxq,
		 u32 *num_recv,
		struct pp2_desc **extra_desc,
		uint32_t *extra_num)
{
	u32 rx_idx;

	rx_idx = rxq->desc_next_idx;
	*extra_num = 0;
	*extra_desc = NULL;

	/*
	* It looks that the continues memory allocated for rx desc
	* is treated by the HW as an circular queue.
	* When the rx desc index is very close to the end of the rx desc array
	* the next descriptors are be stored to the end of the array AND
	* from the beginning of the rx desc array. In this case the return from
	* this function will be 2 arrays of desc:
	* 1 - at the end of the array
	* 2 - starting from the beginning(extra)
	*/

	if (unlikely((rx_idx + *num_recv) > rxq->desc_total)) {
		*extra_desc = rxq->desc_virt_arr;
		/* extra_num is relative to start of desc array */
		*extra_num  = rx_idx + *num_recv - rxq->desc_total;
		/* num_recv is relative to end of desc array */
		*num_recv = rxq->desc_total - rx_idx;
		rxq->desc_next_idx = *extra_num;
	} else {
		rxq->desc_next_idx = (((rx_idx + *num_recv) == rxq->desc_total) ? 0 : (rx_idx + *num_recv));
	}

/*
 *	pr_debug("%s\tdesc array: cur_idx=%d\tlast_idx=%d\n",__func__, rx_idx, rxq->desc_last_idx);
 *	pr_debug("%s\tdesc array: num_recv=%d\textra_num=%d\n",__func__,*num_recv, *extra_num);
 */

	return (rxq->desc_virt_arr + rx_idx);
}

/* Inform about residual packets when destroying the interface */
static void
pp2_rxq_resid_pkts(struct pp2_port *port,
		   struct pp2_rx_queue *rxq)
{
	u32 rx_resid = pp2_rxq_received(port, rxq->id);

	if (!rx_resid)
		return;

	pr_warn("RXQ has %u residual packets\n", rx_resid);

	/* Cleanup for dangling RXDs can be done here by getting
	* the BM-IF associated to the BM poool associated to this
	* RXQ, but it would not be correct.
	*
	* No indirect access to BM pools assigned to this RXQ.
	* Client should handle cleanup before/after destroying the
	* interface
	*/
}

/* Per-RXQ hardware related deinitialization/cleanup
 * Hardware access
 */
static void
pp2_rxq_deinit(struct pp2_port *port,
	       struct pp2_rx_queue *rxq)
{
	uintptr_t cpu_slot = port->cpu_slot;

	pp2_rxq_resid_pkts(port, rxq);

	/* Clear Rx descriptors queue starting address and size;
	* free descriptor number
	*/
	pp2_reg_write(cpu_slot, MVPP2_RXQ_STATUS_REG(rxq->id), 0);
	pp2_reg_write(cpu_slot, MVPP2_RXQ_NUM_REG, rxq->id);
	pp2_reg_write(cpu_slot, MVPP2_RXQ_DESC_ADDR_REG, 0);
	pp2_reg_write(cpu_slot, MVPP2_RXQ_DESC_SIZE_REG, 0);
}

/* Resets RXQ related registers for all RXQs
 * Hardware access
 */
static void
pp2_port_rxqs_deinit(struct pp2_port *port)
{
	int queue;

	for (queue = 0; queue < port->num_rx_queues; queue++)
		pp2_rxq_deinit(port, port->rxqs[queue]);
}

static void pp2_port_rx_pkts_coal_set(struct pp2_port *port, struct pp2_rx_queue *rxq)
{
	uintptr_t cpu_slot = port->cpu_slot;

	if (rxq->pkts_coal > MVPP2_MAX_OCCUPIED_THRESH)
		rxq->pkts_coal = MVPP2_MAX_OCCUPIED_THRESH;

	pp2_reg_write(cpu_slot, MVPP2_RXQ_NUM_REG, rxq->id);
	pp2_reg_write(cpu_slot, MVPP2_RXQ_THRESH_REG, rxq->pkts_coal);
}

static void pp2_port_rx_non_occupied_set(struct pp2_port *port, struct pp2_rx_queue *rxq)
{
	uintptr_t cpu_slot = port->cpu_slot;
	u32 val;

	pp2_reg_write(cpu_slot, MVPP2_RXQ_NUM_REG, rxq->id);
	val = pp2_reg_read(cpu_slot, MVPP2_RXQ_THRESH_REG);
	val &= MVPP2_NONOCCUPIED_THRESH_MASK;
	val |= (rxq->fc_stop_thresh << MVPP2_NONOCCUPIED_THRESH_OFFSET);
	pp2_reg_write(cpu_slot, MVPP2_RXQ_THRESH_REG, val);

	pr_info("rxq:%d, non_occupied_thresh:%d\n", rxq->id, rxq->fc_stop_thresh);

}

/* Set the time delay in usec before Rx interrupt */
static void pp2_port_rx_time_coal_set(struct pp2_port *port, struct pp2_rx_queue *rxq)
{
	u32 val;
	uintptr_t cpu_slot = port->cpu_slot;

	val = (port->parent->hw.tclk / USEC_PER_SEC) * rxq->usec_coal;
	if (val > MVPP2_MAX_ISR_RX_THRESHOLD) {
		val = MVPP2_MAX_ISR_RX_THRESHOLD;
		rxq->usec_coal = (USEC_PER_SEC / port->parent->hw.tclk) * val;
	}
	pp2_reg_write(cpu_slot, MVPP2_ISR_RX_THRESHOLD_REG(rxq->id), val);
}

/* Routine enable flow control for RXQs conditon */
static void pp2_rxq_enable_fc(struct pp2_port *port)
{
	int val, cm3_state, host_id, queue;

	uintptr_t base = port->parent->hw.cm3_base.va;

	/* Remove Flow control enable bit to prevent race between FW and Kernel
	 * If Flow control were enabled, it would be re-enabled.
	 */
	val = cm3_read(base, MSS_CP_FC_COM_REG);
	cm3_state = (val & FLOW_CONTROL_ENABLE_BIT);
	val &= ~FLOW_CONTROL_ENABLE_BIT;
	cm3_write(base, MSS_CP_FC_COM_REG, val);

	/* Set RXQs Flow control */
	for (queue = 0; queue < PP2_PPIO_MAX_NUM_INQS; queue++) {
		struct pp2_rx_queue *rxq = port->rxqs[queue];

		if (!(BIT(queue) & port->rxq_flow_cntrl_mask))
			continue;

		/* Set stop and start Flow control RXQ thresholds */
		val = rxq->fc_start_thresh << MSS_CP_CM3_RXQ_TR_START_OFFS;
		val |= (rxq->fc_stop_thresh << MSS_CP_CM3_RXQ_TR_STOP_OFFS);
		cm3_write(base, MSS_CP_CM3_RXQ_TR_BASE + rxq->id * MSS_CP_CM3_RXQ_TR_OFFS, val);

		/* Set RXQ port ID & Host ID */
		host_id = (rxq->id % PP2_PPIO_MAX_NUM_INQS) / PP22_MAX_NUM_RXQ_PER_INTRPT;
		val = cm3_read(base, MSS_CP_CM3_RXQ_ASS_REG(rxq->id));
		val &= ~(MSS_CP_CM3_RXQ_ASS_PORTID_MASK << ((rxq->id % MSS_CP_CM3_RXQ_ASS_PER_REG)
			* MSS_CP_CM3_RXQ_ASS_PER_OFFS));
		val |= (port->id << ((rxq->id % MSS_CP_CM3_RXQ_ASS_PER_REG) * MSS_CP_CM3_RXQ_ASS_PER_OFFS));
		val &= ~(MSS_CP_CM3_RXQ_ASS_HOSTID_MASK << ((rxq->id % MSS_CP_CM3_RXQ_ASS_PER_REG)
			* MSS_CP_CM3_RXQ_ASS_PER_OFFS + MSS_CP_CM3_RXQ_ASS_HOSTID_OFFS));

		val |= (host_id << ((rxq->id % MSS_CP_CM3_RXQ_ASS_PER_REG)
			* MSS_CP_CM3_RXQ_ASS_PER_OFFS + MSS_CP_CM3_RXQ_ASS_HOSTID_OFFS));

		cm3_write(base, MSS_CP_CM3_RXQ_ASS_REG(queue), val);
	}

	/* Notify Firmware that Flow control config space ready for update */
	val = cm3_read(base, MSS_CP_FC_COM_REG);
	val |= FLOW_CONTROL_UPD_COM_BIT;
	val |= cm3_state;
	cm3_write(base, MSS_CP_FC_COM_REG, val);
}


int
pp2_port_set_outq_state(struct pp2_port *port, struct pp2_tx_queue *txq, int en)
{
	uintptr_t cpu_slot = port->cpu_slot;
	int tx_port_num = MVPP2_MAX_TCONT + port->id;
	u32 val = 0, mask;

/* TODO: add lock to protect MVPP2_TXP_SCHED_PORT_INDEX_REG */
	/* Get active channels mask */
	pp2_reg_write(cpu_slot, MVPP2_TXP_SCHED_PORT_INDEX_REG, tx_port_num);
	val = (pp2_reg_read(cpu_slot, MVPP2_TXP_SCHED_Q_CMD_REG) & MVPP2_TXP_SCHED_ENQ_MASK);
	mask = 1 << txq->log_id;

	if (en) {
		if (!(val & mask)) {
			/* Enable transmit packets to aggregation queue */
			txq->disabled = 0;

			/* Enable Tx queue */
			pp2_reg_write(cpu_slot, MVPP2_TXP_SCHED_PORT_INDEX_REG, tx_port_num);
			pp2_reg_write(cpu_slot, MVPP2_TXP_SCHED_Q_CMD_REG, mask);
		}
	} else {
		if (val & mask) {
			u32 delay = 0;
			u32 pending;

			/* Disable transmit packets to aggregation queue */
			txq->disabled = 1;

			/* Flush Tx queue */
			do {
				if (delay >= MVPP2_TX_PENDING_TIMEOUT_USEC) {
					pr_warn("Port%u: TXQ=%u clean timed out\n", port->id, txq->log_id);
					break;
				}
				/* Sleep for 1 microsecond */
				udelay(1);
				delay++;
				pending = pp2_txq_pend_desc_num_get(port, txq);
				pr_debug("pp2_txq_clean: Port%u: TXQ=%u pending: %d\n", port->id, txq->log_id, pending);
			} while (pending);

			/* Disable Tx queue */
			pp2_reg_write(cpu_slot, MVPP2_TXP_SCHED_PORT_INDEX_REG, tx_port_num);
			pp2_reg_write(cpu_slot, MVPP2_TXP_SCHED_Q_CMD_REG, mask << MVPP2_TXP_SCHED_DISQ_OFFSET);
		}
	}

	return 0;
}

/* Per-TXQ hardware related deinitialization/cleanup
 * Hardware access
 */
static void
pp2_txq_deinit(struct pp2_port *port,
	       struct pp2_tx_queue *txq)
{
	uintptr_t cpu_slot = port->cpu_slot;

	/* Set minimum bandwidth for disabled TXQs */
	pp2_reg_write(cpu_slot, MVPP2_TXQ_SCHED_TOKEN_CNTR_REG(txq->id), 0);

	/* Set Tx descriptors queue starting address and size */
	pp2_reg_write(cpu_slot, MVPP2_TXQ_NUM_REG, txq->id);
	pp2_reg_write(cpu_slot, MVPP2_TXQ_DESC_ADDR_LOW_REG, 0);
	pp2_reg_write(cpu_slot, MVPP2_TXQ_DESC_SIZE_REG, 0);
}

/* Resets TXQ related registers for all TXQs
 * Hardware access
 */
static void
pp2_port_txqs_deinit(struct pp2_port *port)
{
	u32 j;
	struct pp2_tx_queue *txq;
	u32 queue;
	u32 val;
	uintptr_t cpu_slot;

	cpu_slot = port->cpu_slot;

	val = pp2_reg_read(cpu_slot, MVPP2_TX_PORT_FLUSH_REG);

	/* Reset Tx ports and clear Tx queues */
	val |= MVPP2_TX_PORT_FLUSH_MASK(port->id);
	pp2_reg_write(cpu_slot, MVPP2_TX_PORT_FLUSH_REG, val);

	for (queue = 0; queue < port->num_tx_queues; queue++) {
		txq = port->txqs[queue];
		/* Disable and flush Tx queue */
		pp2_port_set_outq_state(port, txq, false);
		pp2_txq_deinit(port, txq);

		/* Lastly, clear all ETH_TXQS for all previous DM-IFs */
		for (j = 0; j < PP2_NUM_REGSPACES; j++) {
			struct pp2_hw *hw = &port->parent->hw;

			cpu_slot = hw->base[j].va;
			pp2_reg_read(cpu_slot, MVPP22_TXQ_SENT_REG(txq->id));
		}
	}
	/* Switch to default slot */
	cpu_slot = port->cpu_slot;

	val &= ~MVPP2_TX_PORT_FLUSH_MASK(port->id);
	pp2_reg_write(cpu_slot, MVPP2_TX_PORT_FLUSH_REG, val);
}

static void
pp2_port_start_dev(struct pp2_port *port)
{
	if ((port->t_mode & PP2_TRAFFIC_INGRESS) == PP2_TRAFFIC_INGRESS)
		pp2_port_mac_max_rx_size_set(port);

	if ((port->t_mode & PP2_TRAFFIC_EGRESS) == PP2_TRAFFIC_EGRESS)
		pp2_port_config_txsched(port);

	pr_debug("start_dev: tx_port_num %d, traffic mode %s%s\n",
		MVPP2_MAX_TCONT + port->id,
	((port->t_mode & PP2_TRAFFIC_INGRESS) == PP2_TRAFFIC_INGRESS) ? " ingress " : "",
	((port->t_mode & PP2_TRAFFIC_EGRESS) == PP2_TRAFFIC_EGRESS) ? " egress " : "");

	if ((port->t_mode & PP2_TRAFFIC_EGRESS) == PP2_TRAFFIC_EGRESS)
		pp2_port_egress_enable(port);

	if ((port->t_mode & PP2_TRAFFIC_INGRESS) == PP2_TRAFFIC_INGRESS)
		pp2_port_ingress_enable(port);
}

/* Set hw internals when stopping port */
static void
pp2_port_stop_dev(struct pp2_port *port)
{
	/* Stop new packets from arriving to RXQs */
	pp2_port_ingress_disable(port);

	/* Sleep for 10 milliseconds */
	mdelay(10);

	/* Disable interrupts on all CPUs */
	pp2_port_interrupts_disable(port);
	pp2_port_egress_disable(port);
}


void
pp2_port_config_inq(struct pp2_port *port)
{
	/* Port's classifier configuration */
	mv_pp2x_cls_oversize_rxq_set(port);
	/* Initialize hardware internals for RXQs */
	pp2_port_rxqs_init(port);
}

void
pp2_port_config_outq(struct pp2_port *port)
{
	/* TX FIFO Init to default 3KB size. Default with minimum threshold */
	/* TODO: change according to port type! */
	/* pp2_port_tx_fifo_config(port, PP2_TX_FIFO_SIZE_3KB, PP2_TX_FIFO_THRS_3KB); */
	/* Initialize hardware internals for TXQs */

	/* TODO: Temporary code to for testing, mechanism required to find this pp2's mem_id */
	u32 mem_id = pp2_get_mem_id(port->parent->id);

	port->tx_qs_mem = mv_sys_dma_mem_region_get(mem_id);
	pr_debug("(%s)Got pointer %p for mem_id(%d)\n", __func__, port->tx_qs_mem, mem_id);
	pp2_port_txqs_init(port);
}

/* External. Interface ready */
void
pp2_port_start(struct pp2_port *port, pp2_traffic_mode t_mode) /* Open from slowpath */
{

	pr_debug("pp2_port_start: %s\n", port->linux_name);
	port->t_mode = t_mode;

	/* For non-loopback port, admin_up interface in Linux. Takes care of Phy/MAC. */
	if (NOT_LPBK_PORT(port))
		pp2_port_set_enable(port, 1);
	mdelay(500);
	pp2_port_start_dev(port);

	if (NOT_LPBK_PORT(port)) {
		/* Sleep to allow link to come up before showing status */
		pp2_gop_status_show(&port->parent->hw.gop, &port->mac_data);
	}
}

/* Internal.
 * Core routine for initializing all data control
 * and hardware internals for an interface
 */
static void
pp2_port_init(struct pp2_port *port) /* port init from probe slowpath */
{
	/* Disable port transmission */
	pp2_port_egress_disable(port);

	/* Allocate TXQ slots for this port */
	port->txqs = kcalloc(1, sizeof(struct pp2_tx_queue *) * port->num_tx_queues, GFP_KERNEL);
	if (unlikely(!port->txqs)) {
		pr_err("%s out of memory txqs alloc\n", __func__);
		return;
	}

	/* Allocate RXQ slots for this port */
	port->rxqs = kcalloc(1, sizeof(struct pp2_rx_queue *) * port->num_rx_queues, GFP_KERNEL);
	if (unlikely(!port->rxqs)) {
		pr_err("%s out of memory rxqs alloc\n", __func__);
		return;
	}

	/* Allocate and associated TXQs to this port */
	pp2_port_txqs_create(port);
	/* Allocate and associated RXQs to this port */
	pp2_port_rxqs_create(port);

	/* Disable port reception */
	pp2_port_ingress_disable(port);

	/* Port default configuration */
	pp2_port_defaults_set(port);

	/* Provide an initial MTU */
	port->flags = PP2_PORT_FLAGS_L4_CHKSUM;
	port->port_mtu = PP2_PORT_DEFAULT_MTU;
	pp2_port_check_mtu_valid(port, port->port_mtu);

	/* Provide an initial MRU */
	port->port_mru = MVPP2_MTU_TO_MRU(PP2_PORT_DEFAULT_MTU);

	/* TODO: Below fn_call is incorrect.
	 * Should mask Interrupts:
	 *  - For MUSDK_NIC ports for all cpu_slots, including kernel
	 *  - For other ports, only for MUSDK cpu_slots (hif_map)
	 */
#if 0
	pp2_port_interrupts_mask(port);
#endif
	/* Get tx_fifo_size from hw_register, value was configured by Linux */
	port->tx_fifo_size = pp2_port_get_tx_fifo(port);

	port->maintain_stats = 0;
	memset(&port->stats, 0, sizeof(port->stats));

	/* Initialize RSS */
	pp2_cls_mng_rss_port_init(port, pp2_rss_map_get());

	/* Set initial cos value */
	pp2_cls_mng_config_default_cos_queue(port);
}

static int32_t
pp2_port_validate_id(const char *if_name)
{
	s32 pid = -1;

	/* Validate interface name. Signature name "<string><number>" */
	if (sscanf(if_name, "%*[^0123456789]%u", &pid) != 1) {
		/* Interface name does not contain a number.*/
		pr_err("PORT: invalid interface '%s'. Expected signature <string><number>\n", if_name);
		return -1;
	}

	if (pid > PP2_NUM_PORTS) {
		pr_err("PORT: invalid interface '%s'. Valid range [0 - %u]\n", if_name, PP2_NUM_PORTS);
		return -1;
	}
	return pid;
}

int populate_tc_pools(struct pp2_inst *pp2_inst, struct pp2_bpool *param_pools[][PP2_PPIO_TC_CLUSTER_MAX_POOLS],
			     struct pp2_bm_pool *pools[][PP2_PPIO_TC_CLUSTER_MAX_POOLS])
{
	u8 index = 0, j, k;
	bool param_pool_exist;
	struct pp2_bm_pool *temp_pool;

	/* check pool0/pool1 */

	for (j = 0; j < MV_SYS_DMA_MAX_NUM_MEM_ID; j++) {
		index = 0;
		param_pool_exist = false;
		for (k = 0; k < PP2_PPIO_TC_CLUSTER_MAX_POOLS; k++) {
			if (param_pools[j][k]) {
				param_pool_exist = true;
				if (param_pools[j][k]->pp2_id != pp2_inst->id) {
					pr_err("%s: pool_ppid[%d] does not match pp2_id[%d]\n",
						__func__, param_pools[j][k]->pp2_id, pp2_inst->id);
					return -1;
				}
				pools[j][index] = pp2_inst->bm_pools[param_pools[j][k]->id];
				if (!pools[j][index]) {
					pr_err("%s: pool_id[%d] has no matching struct\n", __func__,
					       param_pools[j][k]->id);
					return -1;
				}
				index++;
			}
		}
		/* Set pool with smallest buf_size first */
		if (index == 2) {
			if (pools[j][0]->bm_pool_buf_sz > pools[j][1]->bm_pool_buf_sz) {
				temp_pool = pools[j][0];
				pools[j][0] = pools[j][1];
				pools[j][1] = temp_pool;
			}
		} else if (index == 1) {
			pools[j][1] = pools[j][0]; /* Both small and long pool are the same one */
		} else if (param_pool_exist) {
			pr_err("%s: pool_params do not exist\n", __func__);
			return -1;
		}
	}
	return 0;
}

/* Identify the correct packet processor handle and
 * populate port control data based on input parameters
 * Initializes all hardware port internal elements,
 * including egress/ingress queues etc.
 */

int
pp2_port_open(struct pp2 *pp2, struct pp2_ppio_params *param, u8 pp2_id, u8 port_id,
	      struct pp2_port **port_hdl)
{
	u32 i, j, k, first_rxq, num_in_qs;
	u32 total_num_in_qs = 0;
	struct pp2_inst *inst;
	struct pp2_port *port;
	struct pp2_hw *hw;
	int rc;
	enum musdk_lnx_id lnx_id = lnx_id_get();

	inst = pp2->pp2_inst[pp2_id];

	/* Get the internal port handle */
	port = inst->ports[port_id];
	port->parent = inst;


	INIT_LIST(&port->added_uc_addr);

	/* Assign linux name to port */
	pp2_netdev_ifname_get(pp2_id, port_id, port->linux_name);
	pr_debug("pp2_port_open: pp2_id(%d), port_id(%d), port->linux_name(%s)\n", pp2_id, port_id, port->linux_name);

	/* Setup port based on client params
	 * TODO: Traffic Mgr and CoS stuff not implemented yet, so only
	 * the first parameter of the array is used
	 */

	first_rxq = port->id * PP2_HW_PORT_NUM_RXQS;
	if (param->type == PP2_PPIO_T_LOG)
		first_rxq += param->specific_type_params.log_port_params.first_inq;

	port->first_rxq = first_rxq;
	port->num_tcs = param->inqs_params.num_tcs;
	for (i = 0; i < port->num_tcs; i++) {
		u16 tc_pkt_offset = param->inqs_params.tcs_params[i].pkt_offset;
		u8  tc_used_mem_id_pool_mask = 0;
		num_in_qs = param->inqs_params.tcs_params[i].num_in_qs;
		for (j = 0; j < num_in_qs; j++) {
			struct pp2_rxq *rx_q = &(port->tc[i].rx_qs[j]);
			struct pp2_ppio_inq_params *inqs_params = &(param->inqs_params.tcs_params[i].inqs_params[j]);

			rx_q->ring_size = inqs_params->size;
			rx_q->mem = inqs_params->mem;
			rx_q->tc_pools_mem_id_index = inqs_params->tc_pools_mem_id_index;
			tc_used_mem_id_pool_mask |= (1 << inqs_params->tc_pools_mem_id_index);
		}
		if (tc_pkt_offset > PP2_MAX_PACKET_OFFSET) {
			pr_err("port %s: tc[%d] pkt_offset[%u] too large\n", port->linux_name, i, tc_pkt_offset);
			return -EINVAL;
		}
		if (tc_pkt_offset % PP2_BUFFER_OFFSET_GRAN) {
			pr_err("port %s: tc[%d] pkt_offset[%u] must be multiple of %d\n", port->linux_name, i,
				tc_pkt_offset, PP2_BUFFER_OFFSET_GRAN);
			return -EINVAL;
		}
		if (tc_pkt_offset)
			port->tc[i].tc_config.pkt_offset = tc_pkt_offset;
		else
			port->tc[i].tc_config.pkt_offset = PP2_PACKET_DEF_OFFSET;
		port->tc[i].first_log_rxq = total_num_in_qs;
		port->tc[i].tc_config.num_in_qs = num_in_qs;
		port->tc[i].tc_config.default_color = param->inqs_params.tcs_params[i].default_color;
		/*To support RSS, each TC must start at natural rxq boundary */
		first_rxq = roundup(first_rxq, roundup_pow_of_two(num_in_qs));
		port->tc[i].tc_config.first_rxq = first_rxq;
		rc = populate_tc_pools(inst, param->inqs_params.tcs_params[i].pools, port->tc[i].tc_config.pools);
		if (rc)
			return -EINVAL;
		for (j = 0; j < MV_SYS_DMA_MAX_NUM_MEM_ID; j++) {
			if ((1 << j) & tc_used_mem_id_pool_mask)
				if (port->tc[i].tc_config.pools[j][0] == NULL) {
					pr_err("Pool for mem_id(%d) was not configured\n", j);
					return -EINVAL;
				}
		}
		total_num_in_qs += num_in_qs;
		first_rxq += num_in_qs;
	}
	port->num_rx_queues = total_num_in_qs;
	port->num_tx_queues = param->outqs_params.num_outqs;
	for (i = 0; i < port->num_tx_queues; i++) {
		port->txq_config[i].size = param->outqs_params.outqs_params[i].size;
		port->txq_config[i].sched_mode = param->outqs_params.outqs_params[i].sched_mode;
		port->txq_config[i].weight = param->outqs_params.outqs_params[i].weight;
		port->txq_config[i].rate_limit_enable = param->outqs_params.outqs_params[i].rate_limit_enable;
		if (param->outqs_params.outqs_params[i].rate_limit_enable &&
		    param->outqs_params.outqs_params[i].rate_limit_params.cbs < PP2_PPIO_MIN_CBS) {
			pr_err("port %s: CBS for egress queue %u has to be at least %ukB.\n",
			       port->linux_name, i, PP2_PPIO_MIN_CBS);
			return -EINVAL;
		}
		if (param->outqs_params.outqs_params[i].rate_limit_enable &&
		    param->outqs_params.outqs_params[i].rate_limit_params.cir < PP2_PPIO_MIN_CIR) {
			pr_err("port %s: CIR for egress queue %u has to be at least %ukbps.\n",
			       port->linux_name, i, PP2_PPIO_MIN_CIR);
			return -EINVAL;
		}
		port->txq_config[i].rate_limit_params.cbs = param->outqs_params.outqs_params[i].rate_limit_params.cbs;
		port->txq_config[i].rate_limit_params.cir = param->outqs_params.outqs_params[i].rate_limit_params.cir;
	}

	port->enable_port_rate_limit = param->rate_limit_enable;
	if (param->rate_limit_enable && param->rate_limit_params.cbs < PP2_PPIO_MIN_CBS) {
		pr_err("port %s: CBS has to be at least %ukB.\n", port->linux_name, PP2_PPIO_MIN_CBS);
		return -EINVAL;
	}
	if (param->rate_limit_enable && param->rate_limit_params.cir < PP2_PPIO_MIN_CIR) {
		pr_err("port %s: CIR has to be at least %ukbps.\n", port->linux_name, PP2_PPIO_MIN_CIR);
		return -EINVAL;
	}
	port->rate_limit_params.cbs = param->rate_limit_params.cbs;
	port->rate_limit_params.cir = param->rate_limit_params.cir;

	port->hash_type = param->inqs_params.hash_type;
	port->default_plcr = param->inqs_params.plcr;

	if (LPBK_PORT(port))
		port->use_mac_lb = true;
	else
		port->use_mac_lb = false;

	pr_debug("PORT: ID %u (on PP%u):\n", port->id, pp2_id);
	pr_debug("PORT: %s\n", port->use_mac_lb ? "LOOPBACK" : "PHY");

	pr_debug("PORT: TXQs %u\n", port->num_tx_queues);
	pr_debug("PORT: RXQs %u\n", port->num_rx_queues);
	pr_debug("PORT: First Phy RXQ %u\n", port->first_rxq);
	pr_debug("PORT: Hash type %u\n", port->hash_type);

	for (i = 0; i < port->num_tcs; i++) {
		pr_debug("PORT: TC%u\n", i);
		pr_debug("PORT: TC RXQs %u\n", port->tc[i].tc_config.num_in_qs);
		pr_debug("PORT: TC First Log RXQ %u\n", port->tc[i].first_log_rxq);
		pr_debug("PORT: TC First Phy RXQ %u\n", port->tc[i].tc_config.first_rxq);
		pr_debug("PORT: TC PKT Offset %u\n", port->tc[i].tc_config.pkt_offset);
		for (j = 0; j < port->tc[i].tc_config.num_in_qs; j++) {
			pr_debug("PORT: TC RXQ#%u size = %u tc_pool_pair = %u\n", j,
				 port->tc[i].rx_qs[j].ring_size, port->tc[i].rx_qs[j].tc_pools_mem_id_index);
		}
		for (j = 0; j < MV_SYS_DMA_MAX_NUM_MEM_ID; j++)
			for (k = 0; k < PP2_PPIO_TC_CLUSTER_MAX_POOLS; k++)
				if (port->tc[i].tc_config.pools[j][k])
					pr_debug("PORT: TC Pool#%u = %u\n",
						 j, port->tc[i].tc_config.pools[j][k]->bm_pool_id);
	}

	/* Assing a CPU slot to avoid send cpu_slot as argument further */
	hw = &inst->hw;
	port->cpu_slot = hw->base[PP2_DEFAULT_REGSPACE].va;

	if (param->type != PP2_PPIO_T_NIC && param->type != PP2_PPIO_T_LOG)
		return -EINVAL;

	port->type = param->type;

	/* For MUSDK Ethernet ports, call uio_open to request port ownership from Linux */
	if (NOT_LPBK_PORT(port) && port->type == PP2_PPIO_T_NIC) {
		if (lnx_is_mainline(lnx_id))
			rc = pp2_port_set_priv_flags(port, MVPP22_F_IF_MUSDK_PRIV);
		else
			rc = pp2_port_open_uio(port);
		if (rc)
			return rc;
	}
	/* Assign and initialize port private data and hardware */
	pp2_port_init(port);

	port->maintain_stats = param->maintain_stats;
	inst->num_ports++;

	/* At this point, the port is default allocated and configured */
	*port_hdl = port;

	if (NOT_LPBK_PORT(port) && (param->type == PP2_PPIO_T_NIC))
		pp2_port_initialize_statistics(port);

	return 0;
}

static void
pp2_port_deinit(struct pp2_port *port)
{
	kfree(port->stats_name);

	/* Restore rate limits and arbitration to original state */
	pp2_port_deinit_txsched(port);

	pp2_port_flush_mac_addrs(port, 1, 1);

	/* Reset/disable TXQs/RXQs from hardware */
	pp2_port_rxqs_deinit(port);
	pp2_port_txqs_deinit(port);

	/* Deallocate TXQs/RXQs for this port */
	pp2_port_txqs_destroy(port);
	pp2_port_rxqs_destroy(port);

	/* Free port TXQ slots */
	kfree(port->txqs);
	/* Free port RXQ slots */
	kfree(port->rxqs);
}

/* External. Interface down */
void
pp2_port_stop(struct pp2_port *port)
{
	/* Stop new packets from arriving to RXQs */
	pr_debug("pp2_port_stop: %s\n", port->linux_name);

	pp2_port_stop_dev(port);

	/* For non-loopback port, ifconfig down the interface in Linux */
	if (NOT_LPBK_PORT(port))
		pp2_port_set_enable(port, 0);
}

/* External */
void
pp2_port_close(struct pp2_port *port)
{
	struct pp2_inst *inst;
	enum musdk_lnx_id lnx_id = lnx_id_get();

	if (!port)
		return;

	 inst = port->parent;
	 pp2_port_deinit(port);
	 inst->num_ports--;

	/* Close uio_device file, returns ownership to Linux */
	if (NOT_LPBK_PORT(port) && port->type == PP2_PPIO_T_NIC) {
		if (lnx_is_mainline(lnx_id))
			pp2_port_set_priv_flags(port, 0);
		else
			pp2_port_close_uio(port);
	}
}


/* Get RXQ based on which bit is set in the EthOccIC */
static inline struct pp2_rx_queue *
mv_pp2x_get_rx_queue(struct pp2_port *port, uint32_t cause)
{
	u32 rx_queue = fls(cause) - 1;

	if (rx_queue < 0 || rx_queue > PP2_HW_PORT_NUM_RXQS)
		return NULL;
	return port->rxqs[rx_queue];
}

/* Get TXQ based on which bit is set in the EthOccIC */
static inline struct pp2_tx_queue *
mv_pp2x_get_tx_queue(struct pp2_port *port, uint32_t cause)
{
	u32 tx_queue = fls(cause) - 1;

	return port->txqs[tx_queue];
}

/* External. Get actual number of sent descriptors
 * in order to know how many associated packets to release
 */
uint32_t
pp2_port_outq_status(struct pp2_dm_if *dm_if, uint32_t outq_physid)
{
	u32 cnt;
	/* Reading status reg resets transmitted descriptor counter */
	cnt = pp2_relaxed_reg_read(dm_if->cpu_slot, MVPP22_TXQ_SENT_REG(outq_physid));
	return (cnt & MVPP22_TRANSMITTED_COUNT_MASK) >> MVPP22_TRANSMITTED_COUNT_OFFSET;
}

/* External. Request a DM-IF object from this interface */
struct pp2_dm_if *
pp2_port_dm_if_get(struct pp2_port *port, uint32_t dm_id)
{
	 return port->parent->dm_ifs[dm_id];
}

/* External. Get physical TXQ ID */
uint32_t
pp2_port_outq_get_id(struct pp2_port *port, uint32_t out_qid)
{
	 return port->txqs[out_qid]->id;
}

/* TODO: This function is redundant, it will disappear after ppio/pp2_port unification */
static inline void pp2_port_tx_desc_swap_ncopy(struct pp2_desc *dst, struct pp2_ppio_desc *src)
{
	u32 *src_cmd = (uint32_t *)src;
	u32 *dst_cmd = (uint32_t *)dst;
	int i;

	for (i = 0; i < (sizeof(*dst) / sizeof(dst->cmd0)); i++) {
		*dst_cmd = cpu_to_le32(*src_cmd);
		dst_cmd++;
		src_cmd++;
	}
}

/* Enqueue implementation */
uint16_t pp2_port_enqueue(struct pp2_port *port, struct pp2_dm_if *dm_if, uint8_t out_qid, uint16_t num_txds,
			  struct pp2_ppio_desc desc[], struct pp2_ppio_sg_pkts *pkts)
{
	uintptr_t cpu_slot;
	struct pp2_tx_queue *txq;
	struct pp2_txq_dm_if *txq_dm_if;
	struct pp2_desc *tx_desc;
	u16 block_size, to_send = num_txds;
	int i;

	txq = port->txqs[out_qid];
	cpu_slot = dm_if->cpu_slot;

	if (unlikely(txq->disabled))
		goto error;

#ifdef DEBUG
	if ((port->flags & PP2_PORT_FLAGS_L4_CHKSUM) == 0) {
		for (i = 0; i < num_txds; i++) {
		if (DM_TXD_GET_GEN_L4_CHK((desc + i)) == TXD_L4_CHK_ENABLE) {
			pr_err("[%s] port(%d) l4_checksum flag disabled.\n", __func__, port->id);
			goto error;
		}
		}
	}
#endif
	dm_spin_lock(&dm_if->dm_lock);
	if (unlikely(dm_if->free_count < num_txds)) {
		u32 occ_desc;
		/* Update AGGR_Q status, just once */
		occ_desc = pp2_relaxed_reg_read(dm_if->cpu_slot,
						MVPP2_AGGR_TXQ_STATUS_REG(dm_if->id)) & MVPP2_AGGR_TXQ_PENDING_MASK;
		dm_if->free_count = dm_if->desc_total - occ_desc;

		if (unlikely(dm_if->free_count < num_txds)) {
			pr_debug("%s num_txds(%d), free_count(%d) occ_desc(%d)\n", __func__, num_txds,
				 dm_if->free_count, occ_desc);
			num_txds = dm_if->free_count;
		}
	}
	txq_dm_if = &txq->txq_dm_if[dm_if->id];
	if (unlikely(txq_dm_if->desc_rsrvd < num_txds)) {
		u32 req_val, result_val, res_req;

		res_req = max((uint32_t)(num_txds - txq_dm_if->desc_rsrvd), (uint32_t)MVPP2_CPU_DESC_CHUNK);

		req_val = ((txq->id << MVPP2_TXQ_RSVD_REQ_Q_OFFSET) | res_req);
		pp2_relaxed_reg_write(cpu_slot, MVPP2_TXQ_RSVD_REQ_REG, req_val);
		mb();
		result_val = pp2_relaxed_reg_read(cpu_slot, MVPP2_TXQ_RSVD_RSLT_REG) & MVPP2_TXQ_RSVD_RSLT_MASK;

		txq_dm_if->desc_rsrvd += result_val;

		if (unlikely(txq_dm_if->desc_rsrvd < num_txds)) {
			pr_debug("%s prev_desc_rsrvd(%d) desc_rsrvd(%d) res_request(%d) num_txds(%d)\n", __func__,
				 (txq_dm_if->desc_rsrvd - result_val), txq_dm_if->desc_rsrvd, res_req, num_txds);
			num_txds = txq_dm_if->desc_rsrvd;
		}
	}

	if (pkts && to_send > num_txds) {
		u16 curr_txds = 0;

		for (i = 0; i < pkts->num; i++) {
			if (curr_txds + pkts->frags[i] > num_txds)
				break;
			curr_txds += pkts->frags[i];
		}

		pr_debug("[%s:%s] Sending %u descs(%u <- %u) and %u packets (%u)\n", __func__,
			port->linux_name, curr_txds, num_txds, to_send, i, pkts->num);

		num_txds = curr_txds;
		pkts->num = i;
	}

	if (!num_txds) {
		pr_debug("[%s] num_txds is zero\n", __func__);
		dm_spin_unlock(&dm_if->dm_lock);
		goto error;
	}

	tx_desc = pp2_dm_if_next_desc_block_get(dm_if, num_txds, &block_size);

	for (i = 0; i < block_size; i++) {
	/* Destination physical queue ID */
	DM_TXD_SET_DEST_QID(&desc[i], txq->id);
#if __BYTE_ORDER == __BIG_ENDIAN
	pp2_port_tx_desc_swap_ncopy(&tx_desc[i], &desc[i]);
#else
	__builtin_memcpy(&tx_desc[i], &desc[i], sizeof(*tx_desc));
#endif
	}

	if (block_size < num_txds) {
		u16 index = block_size;
		u16 txds_remaining = num_txds - block_size;

		tx_desc = pp2_dm_if_next_desc_block_get(dm_if, txds_remaining, &block_size);
		if (unlikely((index + block_size) != num_txds)) {
			if (likely(num_txds > txq->desc_total)) {
				pr_debug("[%s] More tx_descs(%u) than txq_len(%u)\n", __func__,
					 num_txds, txq->desc_total);
			} else {
				pr_debug("[%s] failed copying tx_descs(%u),in block#1(%u),block#2(%u) txq_len(%u)\n",
					 __func__, num_txds, i, block_size, txq->desc_total);
			}
			num_txds = index + block_size;
		}

		for (i = 0; i < block_size; i++) {
			/* Destination physical queue ID */
			DM_TXD_SET_DEST_QID(&desc[index + i], txq->id);
#if __BYTE_ORDER == __BIG_ENDIAN
			pp2_port_tx_desc_swap_ncopy(&tx_desc[i], &desc[index + i]);
#else
			__builtin_memcpy(&tx_desc[i], &desc[index + i], sizeof(*tx_desc));
#endif
		}
	}

	/* Trigger TX */
	pp2_reg_write(cpu_slot, MVPP2_AGGR_TXQ_UPDATE_REG, num_txds);

	/* Sync reserve count with the AGGR_Q and the Physical TXQ */
	dm_if->free_count -= num_txds;
	txq_dm_if->desc_rsrvd -= num_txds;

	dm_spin_unlock(&dm_if->dm_lock);
	return num_txds;

error:
	if (pkts)
		pkts->num = 0;
	return 0;
}

static void
pp2_cause_error(uint32_t cause)
{
	if (cause & MVPP2_CAUSE_FCS_ERR_MASK)
		pr_err("FCS error\n");
	if (cause & MVPP2_CAUSE_RX_FIFO_OVERRUN_MASK)
		pr_err("RX FIFO overrun error\n");
	if (cause & MVPP2_CAUSE_TX_FIFO_UNDERRUN_MASK)
		pr_err("TX FIFO underrun error\n");
}

/* Dequeue routine
 * The number of packets and the RXD array shall be presented
 * to the dequeue requestor which decides what to do
 * further with these
 */
static inline uint32_t
pp2_port_dequeue(struct pp2_port *port, struct pp2_desc **rx_desc, uint32_t in_qid,
		 struct pp2_desc **extra_rx_desc, uint32_t *extra_num_recv)
{
	u32 num_recv;
	/* Get associated RX queue based on logical ingress queue ID */
	struct pp2_rx_queue *rxq = port->rxqs[in_qid];

	/* number of arrived buffs, must be >= 0!!! */
	num_recv = pp2_rxq_received(port, rxq->id);

	/* Get the start of the RXD array. Polling thread will
	 * iterate through num_recv descriptors
	 */
	*rx_desc = pp2_rxq_get_desc(rxq, &num_recv, extra_rx_desc, extra_num_recv);

	pr_debug("%s\t total num_recv from HW =%d\n", __func__, num_recv);
	pr_debug("%s\trxq_id=%d assign to port=%d is LOCKED\n", __func__, rxq->id, port->id);

	return num_recv;
}

/* Polling implementation */
uint32_t
pp2_port_poll(struct pp2_port *port, struct pp2_desc **desc, uint32_t in_qid,
	      struct pp2_desc **extra_desc, uint32_t *extra_recv)
{
	u32  cause_rx_tx, cause_misc;
	uintptr_t cpu_slot = port->cpu_slot;

	cause_rx_tx = pp2_reg_read(cpu_slot, MVPP2_ISR_RX_TX_CAUSE_REG(port->id));

	/* Check port cause register for errors */
	if (unlikely(cause_rx_tx && (cause_rx_tx & MVPP2_CAUSE_MISC_SUM_MASK))) {
		cause_misc = (cause_rx_tx & MVPP2_CAUSE_MISC_SUM_MASK);
		/* Inform of errors */
		pp2_cause_error(cause_misc);

		/* Clear the cause register */
		pp2_reg_write(cpu_slot, MVPP2_ISR_MISC_CAUSE_REG, 0);
		pp2_reg_write(cpu_slot, MVPP2_ISR_RX_TX_CAUSE_REG(port->id),
			      cause_rx_tx & ~MVPP2_CAUSE_MISC_SUM_MASK);
	}
	/* Return number of received RXDs. RXD array is updated */
	return pp2_port_dequeue(port, desc, in_qid, extra_desc, extra_recv);
}

/* Port Control routines */
static int pp2_port_check_buf_size(struct pp2_port *port, uint16_t mru)
{
	u16 pool_buf_size, req_buf_size, pkt_offset;
	int i, j;

	for (i = 0; i < port->num_tcs; i++) {
		pkt_offset = port->tc[i].tc_config.pkt_offset;
		req_buf_size = MVPP2_MRU_BUF_SIZE(mru, pkt_offset);
		for (j = 0; j < MV_SYS_DMA_MAX_NUM_MEM_ID; j++) {
			struct pp2_bm_pool *long_pool = port->tc[i].tc_config.pools[j][BM_TYPE_LONG_BUF_POOL];

			if (long_pool) {
				pool_buf_size = long_pool->bm_pool_buf_sz;
				if (pool_buf_size < req_buf_size) {
					pr_err("PORT: Oversize required buf_size=[%u]. tc[%u]:pool_id[%u]:buf_size=[%u]\n",
						req_buf_size, long_pool->bm_pool_id, i, pool_buf_size);
					return -EINVAL;
				}
			}
		}
	}
	return 0;
}

/* Set and update the port MTU */
int pp2_port_set_mtu(struct pp2_port *port, uint16_t mtu)
{
	int err = 0;

	err = pp2_port_check_mtu_valid(port, mtu);
	if (err)
		return err;

	/* Stop the port internals */
	pp2_port_stop_dev(port);

	port->port_mtu = mtu;

	/* Start and update the port internals */
	pp2_port_start_dev(port);

	return err;
}

/* Get MTU */
void pp2_port_get_mtu(struct pp2_port *port, uint16_t *mtu)
{
	/* Straightforward. Useful for informing clients the
	 * maximum size their TX BM pool buffers should have,
	 * physical TXQs capabilities, packet fragmentation etc.
	 */
	*mtu = port->port_mtu;
}

static int pp2_port_check_mru_valid(struct pp2_port *port, uint16_t mru)
{
	int err = 0;

	if (mru < PP2_PORT_MIN_MRU) {
		pr_err("PORT: cannot change MRU to less than %u bytes\n", PP2_PORT_MIN_MRU);
		return -EINVAL;
	}
	/* Check the port's related bm_pools buffer_sizes are adequate */
	if (mru > port->port_mru)
		err = pp2_port_check_buf_size(port, mru);

	return err;
}

/* Set and update the port MRU. The function assumes mru valid is valid */
int pp2_port_set_mru(struct pp2_port *port, uint16_t mru)
{
	int err = 0;

	err = pp2_port_check_mru_valid(port, mru);
	if (err)
		return err;
	port->port_mru = mru;

	if ((port->t_mode & PP2_TRAFFIC_INGRESS) == PP2_TRAFFIC_INGRESS)
		pp2_port_mac_max_rx_size_set(port);

	return err;
}

/* Get MRU */
void pp2_port_get_mru(struct pp2_port *port, uint16_t *len)
{
	/* Straightforward. Useful for informing clients the
	 * maximum size their RX BM pool buffers should have
	 */
	*len = port->port_mru;
}

/* Set Loopback */
int pp2_port_set_loopback(struct pp2_port *port, int en)
{
	pp2_port_mac_set_loopback(port, en);

	if (en)
		port->flags |= PP2_PORT_FLAGS_LOOPBACK;
	else
		port->flags &= ~PP2_PORT_FLAGS_LOOPBACK;

	return 0;
}

/* Get Loopback */
int pp2_port_get_loopback(struct pp2_port *port, int *en)
{
	*en = !!(port->flags & PP2_PORT_FLAGS_LOOPBACK);

	return 0;
}

/* Enable or disable RSS */
void pp2_port_set_rss(struct pp2_port *port, uint32_t en)
{
	pp2_rss_enable(port, en);
}

/* Get link status */
int pp2_port_link_status(struct pp2_port *port)
{
	u32 link_is_up;
	struct gop_hw *gop = &port->parent->hw.gop;

	/* Check Link status on ethernet port */
	link_is_up = pp2_gop_port_is_link_up(gop, &port->mac_data);

	if (link_is_up) {
		pr_debug("PORT: Port%u - link is up\n", port->id);
	} else {
		pr_debug("PORT: Port%u - link is down\n", port->id);
	}

	return link_is_up;
}

int ppv2_port_rx_create_event_validate(void *driver_data)
{
	struct rxq_event *rx_event = driver_data;
	int i;
	u32 cause_rx;

	/* Check that at least one of the rx_qs in the per-interrupt qs_mask is raised */
	for (i = 0; i < ARRAY_SIZE(rx_event->rx_intrpt); i++) {
		struct event_intrpt *ev_int = &rx_event->rx_intrpt[i];

		if (!ev_int->qs_mask)
			break;

		cause_rx = pp2_reg_read(ev_int->cpu_slot, MVPP2_ISR_RX_TX_CAUSE_REG(rx_event->parent->id));
		if (cause_rx & ev_int->qs_mask)
			return 1;
	}
	return 0;
}

int pp2_port_rx_create_event(struct pp2_port *port, struct pp2_ppio_rxq_event_params *params, struct mv_sys_event **ev)
{
	u32 loc_ev_qmask, param_rxq_mask = 0;
	struct rxq_event *rx_event;
	int i, cpu_slot_id, err;
	struct mv_sys_event_params ev_params = {0};

	if (port->num_rxq_events == ARRAY_SIZE(port->rx_event))  {
		pr_err("(%s) Too many events\n", __func__);
		return -EINVAL;
	}

	for (i = 0; i < port->num_tcs; i++) {
		u32 tc_rx_mask = GENMASK((port->tc[i].tc_config.first_rxq + port->tc[i].tc_config.num_in_qs - 1),
					 port->tc[i].tc_config.first_rxq);
		u32 tc_ev_par_rx_mask = params->tc_inqs_mask[i] << port->tc[i].tc_config.first_rxq;

		if (tc_ev_par_rx_mask & (~tc_rx_mask)) {
			pr_err("(%s) ppio-%d-%d, tc:%d, tc_inqs_mask:0x%x has non-existent rx_qs\n",
			__func__, port->parent->id, port->id, i, params->tc_inqs_mask[i]);
			return -EINVAL;
		}
		if (tc_ev_par_rx_mask & port->rxq_event_mask) {
			pr_err("(%s) ppio-%d-%d, tc:%d, tc_inqs_mask:0x%x contains rx_qs already in an existing event\n",
			__func__, port->parent->id, port->id, i, params->tc_inqs_mask[i]);
			return -EINVAL;
		}
		param_rxq_mask |= tc_ev_par_rx_mask;
	}
	for (; i < PP2_PPIO_MAX_NUM_TCS; i++) {
		if (params->tc_inqs_mask[i]) {
			pr_err("(%s) tc:%d, is not defined for this ppio-%d-%d\n", __func__, i,
				port->parent->id, port->id);
			return -EINVAL;
		}
	}

	loc_ev_qmask = param_rxq_mask;
	for (i = 0; i < ARRAY_SIZE(port->rx_event); i++)
		if (!port->rx_event[i].valid)
			break;
	rx_event = &port->rx_event[i];

	snprintf(ev_params.name, sizeof(ev_params.name), PORT_STRING, port->parent->id, port->id);
	ev_params.event_validate = ppv2_port_rx_create_event_validate;
	ev_params.driver_data = rx_event;

	err = mv_sys_event_create(&ev_params, ev);
	if (err) {
		pr_err("(%s) Can't open PPv2 event: %s\n", __func__, ev_params.name);
		return err;
	}
	(*ev)->events = MV_SYS_EVENT_POLLIN;

	for (i = 0; (loc_ev_qmask != 0); i++) {
		struct pp2_rx_queue *rxq = port->rxqs[i];

		if ((loc_ev_qmask & BIT(i)) == 0)
			continue;
		rxq->pkts_coal = params->pkt_coal;
		rxq->usec_coal = params->usec_coal;
		pp2_port_rx_pkts_coal_set(port, rxq);
		pp2_port_rx_time_coal_set(port, rxq);
		loc_ev_qmask &= ~(BIT(i));
	}

	/* Configure rx_event */
	memcpy(&rx_event->ev_params, params, sizeof(*params));
	rx_event->rxq_mask = param_rxq_mask;
	rx_event->parent = port;

	for (cpu_slot_id = 0, i = 0; cpu_slot_id < PP2_MAX_NUM_USED_INTERRUPTS; cpu_slot_id++) {
		u32 intrpt_qs_mask = param_rxq_mask >> (cpu_slot_id * PP22_MAX_NUM_RXQ_PER_INTRPT);
		struct event_intrpt *rx_intrpt = &rx_event->rx_intrpt[i];

		if (!intrpt_qs_mask)
			continue;

		rx_intrpt->cpu_slot = port->parent->hw.base[cpu_slot_id].va;
		rx_intrpt->cpu_slot_id = (u32)cpu_slot_id;
		rx_intrpt->qs_mask = intrpt_qs_mask;
		i++;
	}

	port->num_rxq_events++;
	port->rxq_event_mask |= param_rxq_mask;
	return 0;
}


int pp2_port_rx_set_event(struct mv_sys_event *ev, int en)
{
	int err, i;
	struct rxq_event *rx_event;
	u32 cause_mask;

	err = mv_sys_event_get_driver_data(ev, (void **)&rx_event);
	if (err) {
		pr_err("(%s) could not get event driver_data event_ptr(%p)\n", __func__, ev);
		return -EINVAL;
	}

	/* Check that at least one of the rx_qs in the per-interrupt qs_mask is raised */
	for (i = 0; i < ARRAY_SIZE(rx_event->rx_intrpt); i++) {
		struct event_intrpt *ev_int = &rx_event->rx_intrpt[i];

		/* End of participating interrupts */
		if (!ev_int->qs_mask)
			break;

		/* There is a possible double race condition:
		 *	a. Between this event and kernel interrupt code.
		 *	b. Possibly between this event and other events sharing the same interrupt_register.
		 * The code disables the entire interrupt and then continues writing the updated interrupt_mask,
		 * until Cause Mask bits are known to be written in the register, because they are read back.
		 * TODO: Add sys_event_ppio_lock, to eliminate races between US processes.
		 */

		/* Mask entire interrupt (effects events sharing this specific interrupt) */
		pp2_reg_write(ev_int->cpu_slot, MVPP2_ISR_ENABLE_REG(rx_event->parent->id),
			      MVPP2_ISR_DISABLE_INTERRUPT(BIT(ev_int->cpu_slot_id)));

		/* TODO:Add mechanism to validate kernel_interrupt was not started before above masking kicked in.
		 *	Could be one of:
		 *	- small busy_wait (not so robust)
		 *	- Designate Unused PPV22 register, as alternative to US/kernel mutual atomic_counter.
		 *	  Requires fields: [isr_cntr_bits] + [isr_active_bit]
		 */
		while (1) {
			cause_mask = pp2_reg_read(ev_int->cpu_slot, MVPP2_ISR_RX_TX_MASK_REG(rx_event->parent->id));
			if (en) {
				if ((cause_mask & ev_int->qs_mask) == ev_int->qs_mask)
					break;
				cause_mask |= ev_int->qs_mask;
			} else {
				if ((cause_mask & ev_int->qs_mask) == 0)
					break;
				cause_mask &= (~ev_int->qs_mask);
			}
			pp2_reg_write(ev_int->cpu_slot, MVPP2_ISR_RX_TX_MASK_REG(rx_event->parent->id), cause_mask);
		}
		/* Unmask interrupt */
		pp2_reg_write(ev_int->cpu_slot, MVPP2_ISR_ENABLE_REG(rx_event->parent->id),
			      MVPP2_ISR_ENABLE_INTERRUPT(BIT(ev_int->cpu_slot_id)));

	}
	return 0;
}

int pp2_port_rx_delete_event(struct mv_sys_event *ev)
{
	int err, i;
	struct rxq_event *rx_event;
	u32 loc_ev_qmask;
	struct pp2_port *port;

	err = mv_sys_event_get_driver_data(ev, (void **)&rx_event);
	if (err) {
		pr_err("(%s) could not get event driver_data event_ptr(%p)\n", __func__, ev);
		return -EINVAL;
	}

	loc_ev_qmask = rx_event->rxq_mask;
	port = rx_event->parent;

	/* Unset the event */
	pp2_port_rx_set_event(ev, 0);

	/* Reset coalescing values */
	for (i = 0; (loc_ev_qmask); i++) {
		struct pp2_rx_queue *rxq = port->rxqs[i];

		if ((loc_ev_qmask & BIT(i)) == 0)
			continue;
		rxq->pkts_coal = 0;
		rxq->usec_coal = 0;
		pp2_port_rx_pkts_coal_set(port, rxq);
		pp2_port_rx_time_coal_set(port, rxq);
		loc_ev_qmask &= ~(BIT(i));
	}

	port->rxq_event_mask &= (~rx_event->rxq_mask);
	port->num_rxq_events--;
	memset(rx_event, 0, sizeof(*rx_event));

	/* Destroy the event */
	err = mv_sys_event_destroy(ev);
	if (err) {
		pr_err("(%s) failed to delete event (%p)\n", __func__, ev);
		return -EINVAL;
	}

	return 0;
}




int pp2_port_set_tx_pause(struct pp2_port *port, struct pp2_ppio_tx_pause_params *params)
{
	u32 loc_ev_qmask, param_rxq_mask = 0, bm_pools_mask = 0;
	int i, cpu_slot_id, ena = params->en;
	struct pp2_fc_values  *fc_values = NULL;
	uintptr_t cpu_slot = port->parent->hw.base[0].va;


	if (!ena) {
		/*TODO*/
		pr_err("pp2_port_set_tx_pause: disable not yet implemented\n");
		return 0;
	}

	if (params->use_tc_pause_inqs) {
		for (i = 0; i < port->num_tcs; i++) {
			u32 tc_rx_mask = GENMASK((port->tc[i].tc_config.first_rxq +
						  port->tc[i].tc_config.num_in_qs - 1),
						 port->tc[i].tc_config.first_rxq);
			u32 tc_pause_rx_mask = params->tc_inqs_mask[i] << port->tc[i].tc_config.first_rxq;

			if (tc_pause_rx_mask & (~tc_rx_mask)) {
				pr_err("(%s) ppio-%d-%d, tc:%d, tc_inqs_mask:0x%x has non-existent rx_qs\n",
				__func__, port->parent->id, port->id, i, params->tc_inqs_mask[i]);
				return -EINVAL;
			}
			param_rxq_mask |= tc_pause_rx_mask;
		}
		for (; i < PP2_PPIO_MAX_NUM_TCS; i++) {
			if (params->tc_inqs_mask[i]) {
				pr_err("(%s) tc:%d, is not defined for this ppio-%d-%d\n", __func__, i,
					port->parent->id, port->id);
				return -EINVAL;
			}
		}
	} else {
		for (i = 0; i < port->num_tcs; i++) {
			u32 tc_rx_mask = GENMASK((port->tc[i].tc_config.first_rxq +
						  port->tc[i].tc_config.num_in_qs - 1),
						 port->tc[i].tc_config.first_rxq);
			param_rxq_mask |= tc_rx_mask;
		}

	}
	/* Find required values */
	i = 0;
	while ((port->port_mtu > pp2_fc[i].port_mtu) && (i < (ARRAY_SIZE(pp2_fc))))
		i++;
	fc_values = &pp2_fc[i];

	port->rxq_flow_cntrl_mask = param_rxq_mask;
	pr_info("port:%d, flow_control_mask:0x%x\n", port->id, port->rxq_flow_cntrl_mask);

	/* Configure rxq threshold */
	loc_ev_qmask = param_rxq_mask;
	for (i = 0; (loc_ev_qmask != 0); i++) {
		struct pp2_rx_queue *rxq = port->rxqs[i];

		if ((loc_ev_qmask & BIT(i)) == 0)
			continue;
		rxq->fc_start_thresh = fc_values->rxq_start_thresh;
		rxq->fc_stop_thresh = fc_values->rxq_stop_thresh;
		pp2_port_rx_non_occupied_set(port, rxq);
		loc_ev_qmask &= ~(BIT(i));
	}
	for (cpu_slot_id = 0; cpu_slot_id < PP2_MAX_NUM_USED_INTERRUPTS; cpu_slot_id++) {
		u32 qs_mask = param_rxq_mask >> (cpu_slot_id * PP22_MAX_NUM_RXQ_PER_INTRPT);

		cpu_slot = port->parent->hw.base[cpu_slot_id].va;

		if (!qs_mask)
			continue;

		pr_info("port:%d, cpu_slot:%d, qs_mask:0x%x\n", port->id, cpu_slot_id, qs_mask);

		/* Configure Group/Subgroup */
		pp2_port_isr_rx_group_write(port, cpu_slot_id, cpu_slot_id * PP22_MAX_NUM_RXQ_PER_INTRPT,
					    PP22_MAX_NUM_RXQ_PER_INTRPT);

		/* Configure RX Exceptions Interrupt Mask */
		pp2_reg_write(cpu_slot, MVPP2_RX_EX_INT_CAUSE_MASK_REG(port->id), qs_mask);
	}
	/* Configure in FW */
	pp2_rxq_enable_fc(port);


	/* Enable/Disable RX_FIFO */
	if (pp2_reg_read(cpu_slot, MVPP2_VER_ID_REG) == MVPP2_VER_PP23) {
		pr_info("MVPP2_VER_PP23\n");
		pr_info("musdk: not overriding kernel rx_fifo_size, only enabling\n");
		pp2_port_rx_fifo_fc_en(port, ena);
	}

	/* Configure BM_POOLS thresholds */
	loc_ev_qmask = param_rxq_mask;
	for (i = 0; (loc_ev_qmask != 0); i++) {
		struct pp2_rx_queue *rxq = port->rxqs[i];

		if ((loc_ev_qmask & BIT(i)) == 0)
			continue;

		bm_pools_mask |= BIT(rxq->bm_pool_id[0]);
		bm_pools_mask |= BIT(rxq->bm_pool_id[1]);
		pr_info("rxq:%d bpool0:%d, bpool1:%d\n", rxq->id, rxq->bm_pool_id[0], rxq->bm_pool_id[1]);
		loc_ev_qmask &= ~(BIT(i));
	}
	pr_info("bm_pools_mask:%x\n", bm_pools_mask);
	for (i = 0; (bm_pools_mask != 0); i++) {

		if ((bm_pools_mask & BIT(i)) == 0)
			continue;
		pp2_bm_port_add(port->parent, i, port->id,
				fc_values->pool_stop_threshold, fc_values->pool_start_threshold);
		bm_pools_mask &= ~(BIT(i));
	}

	return 0;
}
