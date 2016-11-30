/**
 * @file pp2_port.c
 *
 * Port I/O routines
 */

#include "std_internal.h"


#include "pp2_types.h"

#include "pp2_util.h"
#include "pp2_hw_type.h"
#include "pp2.h"
#include "pp2_bpool.h"
#include "pp2_dm.h"
#include "pp2_port.h"
#include "pp2_bm.h"
#include "pp2_hw_cls.h"
#include "pp2_gop_dbg.h"


/*
 * pp2_port.c
 *
 * Implements configuration and run-time Port and I/O routines
 */


static struct pp2_tc * pp2_rxq_tc_get(struct pp2_port *port, uint32_t id)
{
    uint8_t i;

    for(i = 0;i <port->num_tcs; i++) {
        uint8_t first_rxq = port->tc[i].tc_config.first_rxq;
        if (id >= first_rxq && id < (first_rxq + port->tc[i].tc_config.num_in_qs))
              return(&port->tc[i]);
    }
    return(NULL);
}


/* Set TX FIFO size and threshold */
static inline void pp2_port_tx_fifo_config(struct pp2_port *port,
        uint32_t fifo_size, uint32_t fifo_thr)
{
    /* TX FIFO size */
    pp2_reg_write(port->cpu_slot, MVPP22_TX_FIFO_SIZE_REG(port->id),
            fifo_size & MVPP22_TX_FIFO_SIZE_MASK);

    /* TX FIFO threshold */
    pp2_reg_write(port->cpu_slot, MVPP22_TX_FIFO_THRESH_REG(port->id),
            fifo_thr & MVPP22_TX_FIFO_THRESH_MASK);
}

/* Mask the current CPU's Rx/Tx interrupts */
static inline void
pp2_port_interrupts_mask(struct pp2_port *port)
{
    uintptr_t cpu_slot = port->cpu_slot;

    pp2_reg_write(cpu_slot, MVPP2_ISR_RX_TX_MASK_REG(port->id), 0);
}

static void
pp2_rxq_offset_set(struct pp2_port *port,
        int prxq, int offset)
{
    uint32_t val;
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
    volatile uint32_t tmo;
    uint32_t val = 0;
    uint32_t tx_port_num  = MVPP2_MAX_TCONT + port->id;
    uintptr_t cpu_slot = port->cpu_slot;

    /* Issue stop command for active channels only */
    pp2_reg_write(cpu_slot, MVPP2_TXP_SCHED_PORT_INDEX_REG, tx_port_num);
    val = (pp2_reg_read(cpu_slot, MVPP2_TXP_SCHED_Q_CMD_REG) & MVPP2_TXP_SCHED_ENQ_MASK);
    if (val)
        pp2_reg_write(cpu_slot, MVPP2_TXP_SCHED_Q_CMD_REG, val << MVPP2_TXP_SCHED_DISQ_OFFSET);

    /* TXQs disable. Wait for all Tx activity to terminate. */
    tmo = 0;
    do
    {
        if (tmo >= MVPP2_TX_DISABLE_TIMEOUT_MSEC)
        {
            pp2_warn("Port: Egress disable timeout = 0x%08X\n", val);
            break;
        }
        /* Sleep for 1 millisecond */
        usleep(1000);
        tmo++;
        val = pp2_reg_read(cpu_slot, MVPP2_TXP_SCHED_Q_CMD_REG);
    } while (val & MVPP2_TXP_SCHED_ENQ_MASK);
}

static void
pp2_port_egress_enable(struct pp2_port *port)
{
    uint32_t qmap = 0;
    uint32_t queue;
    uint32_t tx_port_num = MVPP2_MAX_TCONT + port->id;
    uintptr_t cpu_slot = port->cpu_slot;

    /* TXQs enable */
    for (queue = 0; queue < port->num_tx_queues; queue++)
    {
        struct pp2_tx_queue *txq = port->txqs[queue];
        if (NULL != txq->desc_virt_arr)
            qmap |= (1 << queue);
    }
    pp2_reg_write(cpu_slot, MVPP2_TXP_SCHED_PORT_INDEX_REG, tx_port_num);
    pp2_reg_write(cpu_slot, MVPP2_TXP_SCHED_Q_CMD_REG, qmap);
    pp2_info("Port: Egress enable tx_port_num=%u qmap=0x%X\n", tx_port_num, qmap);
}

static void
pp2_port_ingress_enable(struct pp2_port *port)
{
    uint32_t val;
    uint32_t rxq, qid;
    uintptr_t cpu_slot = port->cpu_slot;

    /* RXQs enable */
    for (rxq = 0; rxq < port->num_rx_queues; rxq++)
    {
        qid = port->rxqs[rxq]->id;
        val = pp2_reg_read(cpu_slot, MVPP2_RXQ_CONFIG_REG(qid));
        val &= ~MVPP2_RXQ_DISABLE_MASK;
        pp2_reg_write(cpu_slot, MVPP2_RXQ_CONFIG_REG(qid), val);
    }
}

static void
pp2_port_ingress_disable(struct pp2_port *port)
{
    uint32_t val;
    uint32_t rxq, qid;
    uintptr_t cpu_slot = port->cpu_slot;

    /* RXQs disable */
    for (rxq = 0; rxq < port->num_rx_queues; rxq++)
    {
        qid = port->rxqs[rxq]->id;
        val = pp2_reg_read(cpu_slot, MVPP2_RXQ_CONFIG_REG(qid));
        val |= MVPP2_RXQ_DISABLE_MASK;
        pp2_reg_write(cpu_slot, MVPP2_RXQ_CONFIG_REG(qid), val);
    }
}

static void
pp2_port_interrupts_disable(struct pp2_port *port)
{
    uint32_t j, mask = 0;
    uintptr_t cpu_slot = port->cpu_slot;

    for (j = 0; j < PP2_NUM_CPUS; j++) {
        mask |= (1 << j);
    }
    pp2_reg_write(cpu_slot, MVPP2_ISR_ENABLE_REG(port->id),
            MVPP2_ISR_DISABLE_INTERRUPT(mask));
}

/* Set max sizes for Tx queues */
static void
pp2_txp_max_tx_size_set(struct pp2_port *port)
{
    uint32_t val, size, mtu;
    uint32_t txq, tx_port_num;
    uintptr_t cpu_slot = port->cpu_slot;

    mtu = port->port_mru * 8;
    if (mtu > MVPP2_TXP_MTU_MAX)
        mtu = MVPP2_TXP_MTU_MAX;

    /* WA for wrong Token bucket update: Set MTU value = 3*real MTU value */
    mtu = 3 * mtu;

    /* Indirect access to registers */
    tx_port_num = MVPP2_MAX_TCONT + port->id;
    pp2_reg_write(cpu_slot, MVPP2_TXP_SCHED_PORT_INDEX_REG, tx_port_num);

    /* Set MTU */
    val = pp2_reg_read(cpu_slot, MVPP2_TXP_SCHED_MTU_REG);
    val &= ~MVPP2_TXP_MTU_MAX;
    val |= mtu;
    pp2_reg_write(cpu_slot, MVPP2_TXP_SCHED_MTU_REG, val);

    /* TXP token size and all TXQs token size must be larger that MTU */
    val = pp2_reg_read(cpu_slot, MVPP2_TXP_SCHED_TOKEN_SIZE_REG);
    size = val & MVPP2_TXP_TOKEN_SIZE_MAX;
    if (size < mtu) {
        size = mtu;
        val &= ~MVPP2_TXP_TOKEN_SIZE_MAX;
        val |= size;
        pp2_reg_write(cpu_slot, MVPP2_TXP_SCHED_TOKEN_SIZE_REG, val);
    }

    for (txq = 0; txq < port->num_tx_queues; txq++) {
        val = pp2_reg_read(cpu_slot, MVPP2_TXQ_SCHED_TOKEN_SIZE_REG(txq));
        size = val & MVPP2_TXQ_TOKEN_SIZE_MAX;

        if (size < mtu) {
            size = mtu;
            val &= ~MVPP2_TXQ_TOKEN_SIZE_MAX;
            val |= size;
            pp2_reg_write(cpu_slot, MVPP2_TXQ_SCHED_TOKEN_SIZE_REG(txq), val);
        }
    }
}

static void
pp2_port_mac_max_rx_size_set(struct pp2_port *port)
{
    struct gop_hw *gop = &port->parent->hw.gop;
    struct pp2_mac_data *mac = &port->mac_data;
    int mac_num = port->mac_data.gop_index;

    switch (mac->phy_mode) {
        case PHY_INTERFACE_MODE_RGMII:
        case PHY_INTERFACE_MODE_SGMII:
        case PHY_INTERFACE_MODE_QSGMII:
            pp2_gop_gmac_max_rx_size_set(gop, mac_num,
                    port->port_mru);
            break;
        case PHY_INTERFACE_MODE_XAUI:
        case PHY_INTERFACE_MODE_RXAUI:
        case PHY_INTERFACE_MODE_KR:
            pp2_gop_xlg_mac_max_rx_size_set(gop,
                    mac_num, port->port_mru);
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
    uint32_t val;
    uintptr_t cpu_slot = port->cpu_slot;

    pp2_reg_write(cpu_slot, MVPP2_TXQ_NUM_REG, txq->id);
    val = pp2_reg_read(cpu_slot, MVPP2_TXQ_PENDING_REG);

    return val & MVPP2_TXQ_PENDING_MASK;
}

/* Set defaults to the MVPP2 port */
static void
pp2_port_defaults_set(struct pp2_port *port)
{
   uint32_t tx_port_num, val, queue, ptxq, lrxq;
   struct pp2_inst *inst = port->parent;
   struct pp2_hw *hw = &inst->hw;
   uintptr_t cpu_slot = port->cpu_slot;

   /* Disable Legacy WRR, Disable EJP, Release from reset */
   tx_port_num = MVPP2_MAX_TCONT + port->id;
   pp2_reg_write(cpu_slot, MVPP2_TXP_SCHED_PORT_INDEX_REG, tx_port_num);
   pp2_reg_write(cpu_slot, MVPP2_TXP_SCHED_CMD_1_REG, 0x0);

   /* Close bandwidth for all queues */
   for (queue = 0; queue < MVPP2_MAX_TXQ; queue++)
   {
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
   pp2_reg_write(cpu_slot, MVPP2_RX_CTRL_REG(port->id),
                  MVPP2_RX_USE_PSEUDO_FOR_CSUM_MASK |
                  MVPP2_RX_LOW_LATENCY_PKT_SIZE(256));

   /* Disable Rx cache snoop */
   for (lrxq = 0; lrxq < port->num_rx_queues; lrxq++)
   {
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
    uint32_t j, val, tx_port_num, desc_per_txq, pref_buf_size, desc;
    struct pp2_hw *hw;

    hw = &port->parent->hw;
    cpu_slot = port->cpu_slot;
    desc_per_txq = PP2_TXQ_PREFETCH_16;

    /* FS_A8K Table 1542: The SWF ring size + a prefetch size for HWF */
    txq->desc_total = port->txq_config[txq->log_id].size;
    txq->cma_hdl = cma_calloc(txq->desc_total * MVPP2_DESC_ALIGNED_SIZE);
    if (unlikely(!txq->cma_hdl)) {
        pp2_err("PP: cannot allocate egress descriptor array\n");
        return;
    }
    txq->desc_phys_arr = cma_get_paddr(txq->cma_hdl);
    if (!IS_ALIGNED(txq->desc_phys_arr, MVPP2_DESC_Q_ALIGN)) {
        pp2_err("PP: egress descriptor array must be %u-byte aligned\n",
                MVPP2_DESC_Q_ALIGN);
        cma_free(txq->cma_hdl);
        return;
    }
    txq->desc_virt_arr = (struct pp2_desc *)cma_get_vaddr(txq->cma_hdl);

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
    if (PP2_TXQ_PREFETCH_32 == desc_per_txq)
        pref_buf_size = MVPP2_PREF_BUF_SIZE_32;
    else if (PP2_TXQ_PREFETCH_16 == desc_per_txq)
        pref_buf_size = MVPP2_PREF_BUF_SIZE_16;
    else
        pref_buf_size = MVPP2_PREF_BUF_SIZE_4;

    desc = (port->id * MVPP2_MAX_TXQ * desc_per_txq) + (txq->log_id * desc_per_txq);

    /* Set desc prefetch threshold to 8 units of 2 descriptors */
    pp2_reg_write(cpu_slot, MVPP2_TXQ_PREF_BUF_REG,
            MVPP2_PREF_BUF_PTR(desc) | pref_buf_size |
            MVPP2_PREF_BUF_THRESH(PP2_TXQ_PREFETCH_16 / 2));

    /* WRR / EJP configuration - indirect access */
    tx_port_num = MVPP2_MAX_TCONT + port->id;
    pp2_reg_write(cpu_slot, MVPP2_TXP_SCHED_PORT_INDEX_REG, tx_port_num);

    val = pp2_reg_read(cpu_slot, MVPP2_TXQ_SCHED_REFILL_REG(txq->log_id));
    val &= ~MVPP2_TXQ_REFILL_PERIOD_ALL_MASK;
    val |= MVPP2_TXQ_REFILL_PERIOD_MASK(1);
    val |= MVPP2_TXQ_REFILL_TOKENS_ALL_MASK;
    pp2_reg_write(cpu_slot, MVPP2_TXQ_SCHED_REFILL_REG(txq->log_id), val);

    val = MVPP2_TXQ_TOKEN_SIZE_MAX;
    pp2_reg_write(cpu_slot, MVPP2_TXQ_SCHED_TOKEN_SIZE_REG(txq->log_id), val);

    /* Lastly, clear all ETH_TXQS for all future DM-IFs */
    for (j = 0; j < PP2_NUM_REGSPACES; j++) {
        cpu_slot = hw->base[j].va;
        pp2_reg_read(cpu_slot, MVPP22_TXQ_SENT_REG(txq->id));
    }
}

/* Initializes and sets TXQ related registers for all TXQs
 * Hardware access
 */
static void
pp2_port_txqs_init(struct pp2_port *port)
{
   uint32_t qid;

   for (qid = 0; qid < port->num_tx_queues; qid++)
   {
      struct pp2_tx_queue *txq = port->txqs[qid];
      pp2_txq_init(port, txq);
   }
}

/* Allocates and sets control data for TXQs
 * No hardware access
 */
static void
pp2_port_txqs_create(struct pp2_port *port)
{
   uint32_t qid;

   for (qid = 0; qid < port->num_tx_queues; qid++)
   {
      struct pp2_tx_queue *txq = calloc(1, sizeof(struct pp2_tx_queue));
      if (unlikely(!txq)) {
          pp2_err("PPDK: %s out of memory txq alloc\n",__func__);
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
   uint32_t qid;

   for (qid = 0; qid < port->num_tx_queues; qid++)
   {
      struct pp2_tx_queue *txq = port->txqs[qid];

      cma_free(txq->cma_hdl);
      free(txq);
   }
}

static inline void
pp2_rxq_update_next_desc_idx(struct pp2_rx_queue *rxq, uint32_t num_sent)
{
    uint32_t rx_idx;

    if (unlikely((num_sent < 1) || (num_sent > rxq->desc_total))) {
        pp2_err("RxDesc number inconsistent\n");
        return;
    }

    rx_idx = rxq->desc_next_idx;

    if (likely((rx_idx + num_sent) < rxq->desc_total)) {
        rxq->desc_next_idx = rx_idx + num_sent;
    } else {
        rxq->desc_next_idx = rx_idx + num_sent - rxq->desc_total;
    }

    pp2_dbg("%s\t: cur_idx=%d\tnext_idx=%d\n",__func__, rx_idx, rxq->desc_next_idx);
}

/* External:
 * Update Rx queue status with the number of occupied and available
 * Rx descriptor slots.
 */
void
pp2_port_inq_update(struct pp2_port *port, uint32_t in_qid,
                   uint32_t used_count, uint32_t free_count)
{
   /* Decrement the number of used descriptors and increment the
    * number of free descriptors
    */
   uint32_t id = port->rxqs[in_qid]->id;
   uint32_t val = used_count | (free_count << MVPP2_RXQ_NUM_NEW_OFFSET);
   uintptr_t cpu_slot = port->cpu_slot;

   //pp2_rxq_update_next_desc_idx(port->rxqs[in_qid], used_count);

   pp2_reg_write(cpu_slot, MVPP2_RXQ_STATUS_UPDATE_REG(id), val);
}

/* Per-RXQ hardware related initialization
 * Hardware access
 */
static void
pp2_rxq_init(struct pp2_port *port, struct pp2_rx_queue *rxq)
{
   uint32_t val;
   uintptr_t cpu_slot;
   struct pp2_tc * tc;

   cpu_slot = port->cpu_slot;

   rxq->cma_hdl = cma_calloc(rxq->desc_total * MVPP2_DESC_ALIGNED_SIZE);
   if (unlikely(!rxq->cma_hdl)) {
       pp2_err("PP: cannot allocate ingress descriptor array\n");
       return;
   }
   rxq->desc_phys_arr = cma_get_paddr(rxq->cma_hdl);
   if (!IS_ALIGNED(rxq->desc_phys_arr, MVPP2_DESC_Q_ALIGN)) {
       pp2_err("PP: ingress descriptor array must be %u-byte aligned\n",
               MVPP2_DESC_Q_ALIGN);
       cma_free(rxq->cma_hdl);
       return;
   }
   rxq->desc_virt_arr = (struct pp2_desc *)cma_get_vaddr(rxq->cma_hdl);

   rxq->desc_last_idx = rxq->desc_total - 1;

   /* Zero occupied and non-occupied counters - direct access */
   pp2_reg_write(cpu_slot, MVPP2_RXQ_STATUS_REG(rxq->id), 0x0);

   /* Set Rx descriptors queue starting address - indirect access */
   pp2_reg_write(cpu_slot, MVPP2_RXQ_NUM_REG, rxq->id);

   pp2_reg_write(cpu_slot, MVPP2_RXQ_DESC_ADDR_REG,
                 (rxq->desc_phys_arr >> MVPP22_DESC_ADDR_SHIFT));
   pp2_reg_write(cpu_slot, MVPP2_RXQ_DESC_SIZE_REG, rxq->desc_total);
   pp2_reg_write(cpu_slot, MVPP2_RXQ_INDEX_REG, 0x0);

   /* Set Offset - cache line */
   pp2_rxq_offset_set(port, rxq->id, PP2_PACKET_OFFSET);
   tc = pp2_rxq_tc_get(port, rxq->id);
   if (!tc) {
       pp2_err("port(%d) phy_rxq(%d), not found in tc range \n", port->id, rxq->id);
       return;
   }
   pp2_bm_pool_assign(port, tc->tc_config.pools[BM_TYPE_SHORT_BUF_POOL], rxq->id, BM_TYPE_SHORT_BUF_POOL);
   pp2_bm_pool_assign(port, tc->tc_config.pools[BM_TYPE_LONG_BUF_POOL], rxq->id, BM_TYPE_LONG_BUF_POOL);

   /* Add number of descriptors ready for receiving packets */
   val = (0 | (rxq->desc_total << MVPP2_RXQ_NUM_NEW_OFFSET));
   pp2_reg_write(cpu_slot, MVPP2_RXQ_STATUS_UPDATE_REG(rxq->id), val);
}

/* Initializes and sets RXQ related registers for all RXQs
 * Hardware access
 */
static void
pp2_port_rxqs_init(struct pp2_port *port)
{
   uint32_t qid;
   for (qid = 0; qid < port->num_rx_queues; qid++)
   {
      struct pp2_rx_queue *rxq = port->rxqs[qid];

      pp2_rxq_init(port, rxq);
   }
}

/* Allocates and sets control data for TXQs
 * No hardware access
 */
static void
pp2_port_rxqs_create(struct pp2_port *port)
{
   uint32_t qid, tc, id=0;

   for (tc = 0; tc < port->num_tcs; tc++) {
       for (qid = 0; qid < port->tc[tc].tc_config.num_in_qs; qid++) {
           struct pp2_rx_queue *rxq = calloc(1, sizeof(struct pp2_rx_queue));
           if (unlikely(!rxq)) {
               pp2_err("PPDK: %s out of memory rxq alloc\n",__func__);
           return;
           }
           rxq->id = port->tc[tc].tc_config.first_rxq + qid;
           rxq->log_id = port->tc[tc].first_log_rxq + qid;
           rxq->desc_total = port->tc[tc].rx_ring_size;
           /*TODO: are we really serializing the queue????? */
	   port->rxqs[id++] = rxq;
       }
   }
}

/* Deallocates all TXQs for this port
 * No hardware access
 */
static void
pp2_port_rxqs_destroy(struct pp2_port *port)
{
   uint32_t qid;

   for (qid = 0; qid < port->num_rx_queues; qid++)
   {
      struct pp2_rx_queue *rxq = port->rxqs[qid];

      cma_free(rxq->cma_hdl);
      free(rxq);
   }
}

/* Get pointer to the next RX descriptor to be processed by SW, and update the descriptor next index */
struct pp2_desc *
pp2_rxq_get_desc(struct pp2_rx_queue *rxq,
                uint32_t *num_recv,
                struct pp2_desc **extra_desc,
                uint32_t *extra_num)
{
    uint32_t rx_idx;

    rx_idx = rxq->desc_next_idx;
    *extra_num = 0;
    *extra_desc = NULL;

    /*
     * It looks that the continues memory allocated for rx desc
     * is treated by the HW as an circular queue.
     * When the rx desc index is very close to the end of the rx desc array
     * the next descriptors are be stored to the end of the array AND
     * from the begining of the rx desc array. In this case the return from
     * this function will be 2 arrays of desc:
     * 1 - at the end of the array
     * 2 - starting from the begining(extra)
     */

    if (unlikely((rx_idx + *num_recv) > rxq->desc_total)) {
        *extra_desc = rxq->desc_virt_arr;
        /* extra_num is relative to start of desc array */
        *extra_num  = rx_idx + *num_recv - rxq->desc_total;
        /* num_recv is relative to end of desc array */
        *num_recv = rxq->desc_total - rx_idx;
        rxq->desc_next_idx = *extra_num;
    } else {
        rxq->desc_next_idx = (((rx_idx + *num_recv) == rxq->desc_total)? 0: (rx_idx + *num_recv));
    }

/*
    pp2_dbg("%s\tdesc array: cur_idx=%d\tlast_idx=%d\n",__func__, rx_idx, rxq->desc_last_idx);
    pp2_dbg("%s\tdesc array: num_recv=%d\textra_num=%d\n",__func__,*num_recv, *extra_num);
*/

    return (rxq->desc_virt_arr + rx_idx);
}

/* Inform about residual packets when destroying the interface */
static void
pp2_rxq_resid_pkts(struct pp2_port *port,
                 struct pp2_rx_queue *rxq)
{
   uint32_t rx_resid = pp2_rxq_received(port, rxq->id);

   if (!rx_resid)
      return;

   pp2_warn("RXQ has %u residual packets\n", rx_resid);

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

/* Per-TXQ port cleanup
 * Hardware access
 */
static void
pp2_txq_clean(struct pp2_port *port,
             struct pp2_tx_queue *txq)
{
   volatile uint32_t delay;
   uint32_t pending;
   uint32_t val;
   uint32_t egress_en = false;
   int tx_port_num = MVPP2_MAX_TCONT + port->id;
   uintptr_t cpu_slot = port->cpu_slot;

   pp2_reg_write(cpu_slot, MVPP2_TXQ_NUM_REG, txq->id);
   val = pp2_reg_read(cpu_slot, MVPP2_TXQ_PREF_BUF_REG);
   val |= MVPP2_TXQ_DRAIN_EN_MASK;
   pp2_reg_write(cpu_slot, MVPP2_TXQ_PREF_BUF_REG, val);

   /* Enable egress queue in order to allow releasing all packets*/
   pp2_reg_write(cpu_slot, MVPP2_TXP_SCHED_PORT_INDEX_REG, tx_port_num);
   val = pp2_reg_read(cpu_slot, MVPP2_TXP_SCHED_Q_CMD_REG);
   if (!(val & (1 << txq->log_id))) {
      val |= 1 << txq->log_id;
      pp2_reg_write(cpu_slot, MVPP2_TXP_SCHED_Q_CMD_REG, val);
      egress_en = true;
   }
   delay = 0;
   do
   {
      if (delay >= MVPP2_TX_PENDING_TIMEOUT_MSEC) {
         pp2_warn("Port%u: TXQ=%u clean timed out\n", port->id, txq->log_id);
         break;
      }
      /* Sleep for 1 millisecond */
      usleep(1000);
      delay++;
      pending = pp2_txq_pend_desc_num_get(port, txq);
   } while(pending);

   /* Disable egress queue */
   if (egress_en) {
      pp2_reg_write(cpu_slot, MVPP2_TXP_SCHED_PORT_INDEX_REG, tx_port_num);
      val = (pp2_reg_read(cpu_slot, MVPP2_TXP_SCHED_Q_CMD_REG)) &
                       MVPP2_TXP_SCHED_ENQ_MASK;
      val |= 1 << txq->log_id;
      pp2_reg_write(cpu_slot, MVPP2_TXP_SCHED_Q_CMD_REG,
            (val << MVPP2_TXP_SCHED_DISQ_OFFSET));
      egress_en = false;
   }


   val &= ~MVPP2_TXQ_DRAIN_EN_MASK;
   pp2_reg_write(cpu_slot, MVPP2_TXQ_PREF_BUF_REG, val);
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
    uint32_t j;
    struct pp2_tx_queue *txq;
    uint32_t queue;
    uint32_t val;
    uintptr_t cpu_slot;

    cpu_slot = port->cpu_slot;

    val = pp2_reg_read(cpu_slot, MVPP2_TX_PORT_FLUSH_REG);

    /* Reset Tx ports and clear Tx queues */
    val |= MVPP2_TX_PORT_FLUSH_MASK(port->id);
    pp2_reg_write(cpu_slot, MVPP2_TX_PORT_FLUSH_REG, val);

    for (queue = 0; queue < port->num_tx_queues; queue++) {

        txq = port->txqs[queue];
        pp2_txq_clean(port, txq);
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
	struct gop_hw *gop = &port->parent->hw.gop;
	struct pp2_mac_data *mac = &port->mac_data;


    if ((port->t_mode & PP2_TRAFFIC_INGRESS) == PP2_TRAFFIC_INGRESS)
	    pp2_port_mac_max_rx_size_set(port);

    if ((port->t_mode & PP2_TRAFFIC_EGRESS) == PP2_TRAFFIC_EGRESS)
	    pp2_txp_max_tx_size_set(port);

	pp2_dbg("start_dev: tx_port_num %d, traffic mode %s%s\n",
            MVPP2_MAX_TCONT + port->id,
        ((port->t_mode & PP2_TRAFFIC_INGRESS) == PP2_TRAFFIC_INGRESS) ? " ingress " : "",
        ((port->t_mode & PP2_TRAFFIC_EGRESS) == PP2_TRAFFIC_EGRESS) ? " egress " : "");

	/* No need for port interrupts enable */
	pp2_gop_port_events_mask(gop, mac);

#ifdef NO_MVPP2X_DRIVER
	pp2_gop_port_enable(gop, mac);
#endif
    /* Link status. Indirect access */
    pp2_port_link_status(port);

    pp2_gop_status_show(gop, mac);

    if ((port->t_mode & PP2_TRAFFIC_EGRESS) == PP2_TRAFFIC_EGRESS)
	    pp2_port_egress_enable(port);

    if ((port->t_mode & PP2_TRAFFIC_INGRESS) == PP2_TRAFFIC_INGRESS)
	    pp2_port_ingress_enable(port);

    /* TBD: Do we have interrupt issues? Check following...*/
#ifdef NO_MVPP2X_DRIVER
	pp2_gop_port_events_unmask(gop, mac);
#endif
}

/* Set hw internals when stopping port */
static void
pp2_port_stop_dev(struct pp2_port *port)
{
   struct gop_hw *gop = &port->parent->hw.gop;
   struct pp2_mac_data *mac = &port->mac_data;

   /* Stop new packets from arriving to RXQs */
   pp2_port_ingress_disable(port);

   /* Sleep for 10 milliseconds */
   usleep(10000);

   /* Disable interrupts on all CPUs */
   pp2_port_interrupts_disable(port);
   pp2_port_egress_disable(port);

   pp2_gop_port_events_mask(gop, mac);
   pp2_gop_port_disable(gop, mac);
   port->mac_data.flags &= ~MV_EMAC_F_LINK_UP;
}

static int
pp2_port_mac_hw_init(struct pp2_port *port)
{
	struct gop_hw *gop = &port->parent->hw.gop;
	struct pp2_mac_data *mac = &port->mac_data;
	int gop_port = mac->gop_index;

	if (mac->flags & MV_EMAC_F_INIT)
		return 0;

	/* configure port PHY address */
	pp2_gop_smi_phy_addr_cfg(gop, gop_port, mac->phy_addr);

	pp2_gop_port_init(gop, mac, port->use_mac_lb);

	if (mac->force_link)
		pp2_gop_fl_cfg(gop, mac);

	mac->flags |= MV_EMAC_F_INIT;

	return 0;
}

void
pp2_port_config_inq(struct pp2_port *port)
{
    /* Port's classifier configuration */
    mv_pp2x_cls_oversize_rxq_set(port);
#ifdef NO_MVPP2X_DRIVER
    mv_pp2x_cls_port_config(port);
#endif
    /* Initialize hardware internals for RXQs */
    pp2_port_rxqs_init(port);
#ifdef NO_MVPP2X_DRIVER
    /*TBD(RX) - port Classfier/Policer/Parser from driver */
    mv_pp2x_open_cls(port);
#endif
}

void
pp2_port_config_outq(struct pp2_port *port)
{
    /* TX FIFO Init to default 3KB size. Default with minimum threshold */
    /* TODO: change according to port type! */
    //pp2_port_tx_fifo_config(port, PP2_TX_FIFO_SIZE_3KB, PP2_TX_FIFO_THRS_3KB);
#ifdef NO_MVPP2X_DRIVER
    pp2_port_tx_fifo_config(port, PP2_TX_FIFO_SIZE_10KB, PP2_TX_FIFO_THRS_10KB);
#endif
    /* Initialize hardware internals for TXQs */
    pp2_port_txqs_init(port);
}

/* External. Interface ready */
void
pp2_port_start(struct pp2_port *port, pp2_traffic_mode t_mode) /* Open from slowpath */
{
    port->t_mode = t_mode;

    pp2_port_start_dev(port);
#ifdef NO_MVPP2X_DRIVER
    if ((t_mode & PP2_TRAFFIC_INGRESS) == PP2_TRAFFIC_INGRESS)
        mv_pp2x_open_cls(port);
#endif
}

/* Internal.
 * Core routine for initializing all data control
 * and hardware internals for an interface
 */
static void
pp2_port_init(struct pp2_port *port) /* port init from probe slowpath */
{
   struct gop_hw *gop = &port->parent->hw.gop;
   struct pp2_mac_data *mac = &port->mac_data;

   /* Disable port transmission */
   pp2_port_egress_disable(port);

#ifdef NO_MVPP2X_DRIVER
   pp2_gop_port_disable(gop, mac);
#endif
   /* Allocate TXQ slots for this port */
   port->txqs = calloc(1, sizeof(struct pp2_tx_queue *) * port->num_tx_queues);
   if (unlikely(!port->txqs)){
       pp2_err("PPDK: %s out of memory txqs alloc\n",__func__);
       return;
   }

   /* Allocate RXQ slots for this port */
   port->rxqs = calloc(1, sizeof(struct pp2_rx_queue *) * port->num_rx_queues);
   if (unlikely(!port->rxqs)){
       pp2_err("PPDK: %s out of memory rxqs alloc\n",__func__);
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
   port->port_mtu = PP2_PORT_DEFAULT_MTU;
   /* Provide an initial MRU */
   port->port_mru = MVPP2_RX_PKT_SIZE(PP2_PORT_DEFAULT_MTU);

   /* Here was the place to activate interrupts
    * but do not unmask CPU and RX QVec Shared interrupts yet,
    * work on polling mode
    */
   pp2_port_interrupts_mask(port);

#ifdef NO_MVPP2X_DRIVER
   pp2_port_mac_hw_init(port);
#endif

}

static int32_t
pp2_port_validate_id(const char *if_name)
{
   int32_t pid = -1;

   /* Validate interface name. Signature name "<string><number>" */
   if (1 != sscanf(if_name, "%*[^0123456789]%u", &pid))
   {
      /* Interface name does not contain a number.*/
      pp2_err("PORT: invalid interface '%s'. Expected signature <string><number>\n", if_name);
      return -1;
   }

   if (pid > PP2_NUM_PORTS)
   {
      pp2_err("PORT: invalid interface '%s'. Valid range [0 - %u]\n", if_name, PP2_NUM_PORTS);
      return -1;
   }
   return pid;
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
   uint32_t i, j, first_rxq, num_in_qs;
   uint32_t total_num_in_qs = 0;
   struct pp2_inst *inst;
   struct pp2_port *port;
   struct pp2_hw *hw;

   inst = pp2->pp2_inst[pp2_id];


   /* Get the internal port handle */
   port = inst->ports[port_id];
   port->parent = inst;

   /* Setup port based on client params
    * TODO: Traffic Mgr and CoS stuff not implemented yet, so only
    * the first parameter of the array is used
    */
   first_rxq = port->id * PP2_HW_PORT_NUM_RXQS + pp2->init.ppios[pp2_id][port_id].first_inq;
   port->first_rxq  = first_rxq;
   port->num_tcs = param->inqs_params.num_tcs;
   for (i = 0; i < port->num_tcs; i++) {
        num_in_qs = param->inqs_params.tcs_params[i].num_in_qs;
        port->tc[i].rx_ring_size = param->inqs_params.tcs_params[i].inqs_params->size;
        port->tc[i].tc_config.pkt_offset = param->inqs_params.tcs_params[i].pkt_offset;
        port->tc[i].tc_config.use_hash = param->inqs_params.tcs_params[i].use_hash;
        port->tc[i].first_log_rxq = total_num_in_qs;
        port->tc[i].tc_config.num_in_qs = num_in_qs;
        first_rxq = roundup(first_rxq, num_in_qs); /*To support RSS, each TC must start at natural rxq boundary */
        port->tc[i].tc_config.first_rxq = first_rxq;

        for (j = 0;j < PP2_PPIO_TC_MAX_POOLS; j++) {
		if (!param->inqs_params.tcs_params[i].pools[j])
			break;
		port->tc[i].tc_config.pools[j] = param->inqs_params.tcs_params[i].pools[j]->id;
        }
        total_num_in_qs += num_in_qs;
        first_rxq += num_in_qs;
   }
   port->num_rx_queues = total_num_in_qs;
   port->num_tx_queues = param->outqs_params.num_outqs;
   for (i = 0; i < port->num_tx_queues; i++) {
       port->txq_config[i].size = param->outqs_params.outqs_params[i].size;
       port->txq_config[i].weight = param->outqs_params.outqs_params[i].weight;
   }

   for (i = 0; i < PP2_PPIO_MAX_NUM_HASH; i++) {
       port->hash_type[i] = param->inqs_params.hash_type[i];
   }
   /*TODO: Delete this param */
   port->use_mac_lb = false;

   pp2_dbg("PORT: ID %u (on PP%u):\n", port->id, pp2_id);
   pp2_dbg("PORT: %s\n", port->use_mac_lb ? "LOOPBACK" : "PHY");

   pp2_dbg("PORT: TXQs %u\n", port->num_tx_queues);
   pp2_dbg("PORT: RXQs %u\n", port->num_rx_queues);
   pp2_dbg("PORT: First Phy RXQ %u\n", port->first_rxq);
   for (i = 0; i < port->num_tcs; i++) {
       pp2_dbg("PORT: TC%u\n", i);
       pp2_dbg("PORT: TC RXQs %u\n", port->tc[i].tc_config.num_in_qs);
       pp2_dbg("PORT: TC First Log RXQ %u\n", port->tc[i].first_log_rxq);
       pp2_dbg("PORT: TC First Phy RXQ %u\n", port->tc[i].tc_config.first_rxq);
       pp2_dbg("PORT: TC RXQ size %u\n", port->tc[i].rx_ring_size);
       pp2_dbg("PORT: TC PKT Offset %u\n", port->tc[i].tc_config.pkt_offset);
       pp2_dbg("PORT: TC Use Hash %u\n", port->tc[i].tc_config.use_hash);
       for (j = 0;j < PP2_PPIO_TC_MAX_POOLS; j++) {
            pp2_dbg("PORT: TC Pool#%u = %u\n", j, port->tc[i].tc_config.pools[j]);
        }
   }
   /* Assing a CPU slot to avoid send cpu_slot as argument further */
   hw = &inst->hw;
   port->cpu_slot = hw->base[PP2_DEFAULT_REGSPACE].va;

   /* Assign and initialize port private data and hardware */
   pp2_port_init(port);

   inst->num_ports++;

   /* At this point, the port is default allocated and configured */
   *port_hdl = port;
   return 0;
}

static void
pp2_port_deinit(struct pp2_port *port)
{
   /* Reset/disable TXQs/RXQs from hardware */
   pp2_port_rxqs_deinit(port);
   pp2_port_txqs_deinit(port);

   /* Deallocate TXQs/RXQs for this port */
   pp2_port_txqs_destroy(port);
   pp2_port_rxqs_destroy(port);

   /* Free port TXQ slots */
   free(port->txqs);
   /* Free port RXQ slots */
   free(port->rxqs);
}

/* External. Interface down */
void
pp2_port_stop(struct pp2_port *port)
{
   /* Stop new packets from arriving to RXQs */
   pp2_port_stop_dev(port);

   /* Redundant since IRQs already disabled at
    * port init, but keep it for simmetry
    */
   pp2_port_interrupts_mask(port);
}

/* External */
void
pp2_port_close(struct pp2_port *port)
{
    struct pp2_inst *inst;

    if (NULL == port)
        return;

    inst = port->parent;
    pp2_port_deinit(port);

    inst->num_ports--;
}

/* Get RXQ based on which bit is set in the EthOccIC */
static inline struct pp2_rx_queue *
mv_pp2x_get_rx_queue(struct pp2_port *port, uint32_t cause)
{
   uint32_t rx_queue = fls(cause) - 1;

   if (rx_queue < 0 || rx_queue > PP2_HW_PORT_NUM_RXQS)
      return NULL;
   return port->rxqs[rx_queue];
}

/* Get TXQ based on which bit is set in the EthOccIC */
static inline struct pp2_tx_queue *
mv_pp2x_get_tx_queue(struct pp2_port *port, uint32_t cause)
{
   uint32_t tx_queue = fls(cause) - 1;

   return port->txqs[tx_queue];
}

/* External. Get actual number of sent descriptors
 * in order to know how many associated packets to release
 */
uint32_t
pp2_port_outq_status(struct pp2_dm_if *dm_if, uint32_t outq_physid)
{
   uint32_t cnt;
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
static inline void pp2_port_tx_desc_swap_ncopy(struct pp2_desc *dst, struct pp2_rx_desc *src)
{
	uint32_t *src_cmd = (uint32_t *)src;
	uint32_t *dst_cmd = (uint32_t *)dst;

	for (int i = 0; i < (sizeof(*dst)/sizeof(dst->cmd0)); i++) {
		*dst_cmd = le32toh(*src_cmd);
		dst_cmd++;
		src_cmd++;
	}
}



/* Enqueue implementation */
uint16_t pp2_port_enqueue(struct pp2_port *port, struct pp2_dm_if *dm_if, uint8_t out_qid, uint16_t num_txds, struct pp2_ppio_desc desc[])
{
   uintptr_t cpu_slot;
   struct pp2_tx_queue *txq;
   struct pp2_txq_dm_if *txq_dm_if;
   struct pp2_desc * tx_desc;
   uint16_t block_size;
   int i;

   txq = port->txqs[out_qid];
   cpu_slot = dm_if->cpu_slot;

   if (unlikely(dm_if->free_count < num_txds)) {
       uint32_t occ_desc;
       /* Update AGGR_Q status, just once */
       occ_desc = pp2_relaxed_reg_read(dm_if->cpu_slot,
                  MVPP2_AGGR_TXQ_STATUS_REG(dm_if->id)) & MVPP2_AGGR_TXQ_PENDING_MASK;
       dm_if->free_count = dm_if->desc_total - occ_desc;

       if (unlikely(dm_if->free_count < num_txds)) {
	    pr_debug("%s num_txds(%d), free_count(%d) occ_desc(%d)\n", __FUNCTION__, num_txds,
	    	     dm_if->free_count, occ_desc);
            num_txds = dm_if->free_count;
       }
   }
   txq_dm_if = &(txq->txq_dm_if[dm_if->id]);
   if (unlikely(txq_dm_if->desc_rsrvd < num_txds)) {
       uint32_t req_val, result_val, res_req;

       res_req = max((num_txds - txq_dm_if->desc_rsrvd), MVPP2_CPU_DESC_CHUNK);

       req_val = ((txq->id << MVPP2_TXQ_RSVD_REQ_Q_OFFSET) | res_req);
       pp2_relaxed_reg_write(cpu_slot, MVPP2_TXQ_RSVD_REQ_REG, req_val);
       result_val = pp2_relaxed_reg_read(cpu_slot, MVPP2_TXQ_RSVD_RSLT_REG) & MVPP2_TXQ_RSVD_RSLT_MASK;

       txq_dm_if->desc_rsrvd += result_val;

       if (unlikely(txq_dm_if->desc_rsrvd < num_txds)) {
       	   pr_debug("%s prev_desc_rsrvd(%d) desc_rsrvd(%d) res_request(%d) num_txds(%d)\n",
	   	    __FUNCTION__, (txq_dm_if->desc_rsrvd - result_val), txq_dm_if->desc_rsrvd, res_req, num_txds);
           num_txds = txq_dm_if->desc_rsrvd;
       }
   }
   if (!num_txds) {
       	pr_debug("%s\ num_txds is zero \n", __FUNCTION__);
   	return 0;
   }

   tx_desc = pp2_dm_if_next_desc_block_get(dm_if, num_txds, &block_size);

   for (i = 0; i<block_size; i++) {
	/* Destination physical queue ID */
	DM_TXD_SET_DEST_QID(&desc[i], txq->id);
#if __BYTE_ORDER == __BIG_ENDIAN
	pp2_port_tx_desc_swap_ncopy(&tx_desc[i], &desc[i]);
#else
	__builtin_memcpy(&tx_desc[i], &desc[i], sizeof(*tx_desc));
#endif
   }

   if (block_size < num_txds) {
       uint16_t index = block_size;
       uint16_t txds_remaining = num_txds - block_size;
       tx_desc = pp2_dm_if_next_desc_block_get(dm_if, txds_remaining, &block_size);
       if (unlikely((index + block_size) != num_txds)) {
       	   if (likely(num_txds > dm_if->desc_total)) {
               pr_debug("[%s] More tx_descs(%u) than txq_len(%u) \n", __FUNCTION__, num_txds, txq_dm_if->desc_total);
       	   } else {
               pr_debug("[%s] failed copying tx_descs(%u),in block#1(%u),block#2(%u) txq_len(%u)\n", num_txds, i,
	       	        block_size, txq_dm_if->desc_total);
       	   }
	   num_txds = index + block_size;
       }

      for (i = 0; i < block_size; i++) {
	/* Destination physical queue ID */
	DM_TXD_SET_DEST_QID(&desc[index+i], txq->id);
#if __BYTE_ORDER == __BIG_ENDIAN
      pp2_port_tx_desc_swap_ncopy(&tx_desc[i], &desc[index+i]);
#else
	__builtin_memcpy(&tx_desc[i], &desc[index+i], sizeof(*tx_desc));
#endif

     }

   }

   /* Trigger TX */
   pp2_reg_write(cpu_slot, MVPP2_AGGR_TXQ_UPDATE_REG, num_txds);

   /* Sync reserve count with the AGGR_Q and the Physical TXQ */
   dm_if->free_count -= num_txds;
   txq_dm_if->desc_rsrvd -= num_txds;

   return num_txds;
}

static void
pp2_cause_error(uint32_t cause)
{
   if (cause & MVPP2_CAUSE_FCS_ERR_MASK)
      pp2_err("FCS error\n");
   if (cause & MVPP2_CAUSE_RX_FIFO_OVERRUN_MASK)
      pp2_err("RX FIFO overrun error\n");
   if (cause & MVPP2_CAUSE_TX_FIFO_UNDERRUN_MASK)
      pp2_err("TX FIFO underrun error\n");
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
   uint32_t num_recv;
   /* Get associated RX queue based on logical ingress queue ID */
   struct pp2_rx_queue *rxq = port->rxqs[in_qid];

   /* number of arrived buffs, must be >= 0!!! */
   num_recv = pp2_rxq_received(port, rxq->id);

   /* Get the start of the RXD array. Polling thread will
    * iterate through num_recv descriptors */
   *rx_desc = pp2_rxq_get_desc(rxq, &num_recv, extra_rx_desc, extra_num_recv);

   pp2_dbg("%s\t total num_recv from HW =%d\n",__func__, num_recv);
   pp2_dbg("%s\trxq_id=%d assign to port=%d is LOCKED\n",__func__,rxq->id, port->id);

   return num_recv;
}

/* Polling implementation */
uint32_t
pp2_port_poll(struct pp2_port *port, struct pp2_desc **desc, uint32_t in_qid,
            struct pp2_desc **extra_desc, uint32_t *extra_recv)
{
   uint32_t  cause_rx_tx, cause_misc;
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

/* Set MAC address */
int pp2_port_set_mac_addr(struct pp2_port *port, const uint8_t *addr)
{
    int err = 0;

    if (!is_valid_ether_addr(addr)) {
        pp2_err("PORT: not a valid eth address\n");
        return -EINVAL;
    }

    /* Stop the port internals */
    pp2_port_stop_dev(port);

    err = mv_pp2x_prs_update_mac_da(port, (const uint8_t *)addr);
    if (err) {
        /* Restart the port and exit */
        pp2_err("PORT: cannot update parser entries with new eth address\n");
        pp2_port_start_dev(port);
        return err;
    }

    /* Reconfigure parser to accept the original MAC address */
    err = mv_pp2x_prs_update_mac_da(port, (const uint8_t *)port->mac_data.mac);
    if (err) {
        pp2_err("PORT: cannot update parser entries with old eth address\n");
        pp2_port_start_dev(port);
        return err;
    }

    /* Start the port internals */
    pp2_port_start_dev(port);
    return err;
}

/* Get MAC address */
void pp2_port_get_mac_addr(struct pp2_port *port, uint8_t *addr)
{
    ether_addr_copy(addr, (const uint8_t *)port->mac_data.mac);
}

/* Set and update the port MTU */
int pp2_port_set_mtu(struct pp2_port *port, uint32_t mtu)
{
    int err = 0;
    uint32_t tx_fifo_thr = 0;
    uint32_t tx_fifo_size = 0;

    /* Validate MTU */
    if (mtu < PP2_PORT_MIN_MTU) {
        pp2_err("PORT: cannot change MTU to less than %u bytes\n", PP2_PORT_MIN_MTU);
        err = -EINVAL;
        return err;
    }

    /* Check maximum MTU value and round up to that plus extra control bytes */
    if (MVPP2_RX_PKT_SIZE(mtu) > MVPP2_BM_JUMBO_PKT_SIZE) {
        uint32_t max_val = MVPP2_RX_MTU_SIZE(MVPP2_BM_JUMBO_PKT_SIZE);
        pp2_info("PORT: illegal MTU value. Round down to %u bytes \n", max_val);
        mtu = max_val;
    }
    /* Stop the port internals */
    pp2_port_stop_dev(port);

    port->port_mtu = mtu;

    /* Update the TX FIFO size register in accordance with the new MTU value */
    switch (mtu / (1024)) {
        case 2:
            tx_fifo_size = PP2_TX_FIFO_SIZE_3KB;
            tx_fifo_thr = PP2_TX_FIFO_THRS_3KB;
            break;
        case 3:
            tx_fifo_size = PP2_TX_FIFO_SIZE_4KB;
            tx_fifo_thr = PP2_TX_FIFO_THRS_4KB;
            break;
        case 4:
            tx_fifo_size = PP2_TX_FIFO_SIZE_5KB;
            tx_fifo_thr = PP2_TX_FIFO_THRS_5KB;
            break;
        case 5:
            tx_fifo_size = PP2_TX_FIFO_SIZE_6KB;
            tx_fifo_thr = PP2_TX_FIFO_THRS_6KB;
            break;
        case 6:
            tx_fifo_size = PP2_TX_FIFO_SIZE_7KB;
            tx_fifo_thr = PP2_TX_FIFO_THRS_7KB;
            break;
        case 7:
            tx_fifo_size = PP2_TX_FIFO_SIZE_8KB;
            tx_fifo_thr = PP2_TX_FIFO_THRS_8KB;
            break;
        case 8:
            tx_fifo_size = PP2_TX_FIFO_SIZE_9KB;
            tx_fifo_thr = PP2_TX_FIFO_THRS_9KB;
            break;
        case 9:
            tx_fifo_size = PP2_TX_FIFO_SIZE_10KB;
            tx_fifo_thr = PP2_TX_FIFO_THRS_10KB;
            break;
        default:
            tx_fifo_size = PP2_TX_FIFO_SIZE_2KB;
            tx_fifo_thr = PP2_TX_FIFO_THRS_2KB;
            break;
    }
    /* Update FIFO in HW */
    pp2_port_tx_fifo_config(port, tx_fifo_size, tx_fifo_thr);

    /* Start and update the port internals */
    pp2_port_start_dev(port);

    return err;
}

/* Get MTU */
void pp2_port_get_mtu(struct pp2_port *port, uint32_t *mtu)
{
    /* Straightforward. Useful for informing clients the
     * maximum size their TX BM pool buffers should have,
     * physical TXQs capabilities, packet fragmentation etc.
     */
    *mtu = port->port_mtu;
}

/* Set and update the port MRU */
int pp2_port_set_mru(struct pp2_port *port, uint32_t mru)
{
    int err = 0;

    /* Validate MRU */
    if (mru < PP2_PORT_MIN_MTU) {
        pp2_err("PORT: cannot change MRU to less than %u bytes\n", PP2_PORT_MIN_MTU);
        err = -EINVAL;
        return err;
    }

    /* Check what maximum MTU value would be in relation to the input mru
     * value and round up to that plus extra meta bytes
     */
    if (MVPP2_RX_PKT_SIZE(mru) > MVPP2_BM_JUMBO_PKT_SIZE) {
        uint32_t max_mtu_val = MVPP2_RX_MTU_SIZE(MVPP2_BM_JUMBO_PKT_SIZE);
        pp2_info("PORT: illegal MRU value. Round down to %u bytes\n", max_mtu_val);
        mru = MVPP2_RX_PKT_SIZE(max_mtu_val);
    }

    /* Stop the port internals */
    pp2_port_stop_dev(port);

    /* This MRU is set for control reasons. Hardware RX FIFO is set and
     * initialized by U-Boot depending on port setup (10G, 2.5G, 1G).
     * Pools attached to RXQs of this port would have to be re-constructed
     * using buffers of corresponding sizes.
     * Since a BM interface is exported, it is better to not update
     * BM pools implicitly and leave that as client responsability
     */
    port->port_mru = mru;

    /* Start and update the port internals */
    pp2_port_start_dev(port);

    return err;
}

/* Get MRU */
void pp2_port_get_mru(struct pp2_port *port, uint32_t *len)
{
    /* Straightforward. Useful for informing clients the
     * maximum size their RX BM pool buffers should have
     */
    *len = port->port_mru;
}

/* Set Unicast promiscuous */
void pp2_port_set_uc_promisc(struct pp2_port *port, uint32_t en)
{
    struct pp2_hw *hw = &port->parent->hw;
    uint32_t id = port->id;

    /* TODO: Revise these */

    /* Enter promisc mode */
    mv_pp2x_prs_mac_promisc_set(hw, id, en);
    /* Remove all port->id's ucast enries except M2M entry */
    mv_pp2x_prs_mac_entry_del(port, MVPP2_PRS_MAC_UC, MVPP2_DEL_MAC_ALL);
}

/* Set Multicast promiscuous */
void pp2_port_set_mc_promisc(struct pp2_port *port, uint32_t en)
{
    struct pp2_hw *hw = &port->parent->hw;
    uint32_t id = port->id;

    /* TODO: Revise these */

    /* Accept all multicast */
    mv_pp2x_prs_mac_multi_set(hw, id, MVPP2_PE_MAC_MC_ALL, en);
    mv_pp2x_prs_mac_multi_set(hw, id, MVPP2_PE_MAC_MC_IP6, en);
    /* Enter promisc mode */
    mv_pp2x_prs_mac_promisc_set(hw, id, en);
    /* Remove all port->id's mcast enries */
    mv_pp2x_prs_mac_entry_del(port, MVPP2_PRS_MAC_MC, MVPP2_DEL_MAC_ALL);
}

/* Remove MAC address */
int pp2_port_remove_mac_addr(struct pp2_port *port, const uint8_t *addr)
{
    if (!is_valid_ether_addr(addr)) {
        pp2_err("PORT: not a valid eth address\n");
        return -EINVAL;
    }

    if (is_unicast_ether_addr(addr))
        mv_pp2x_prs_mac_entry_del(port, MVPP2_PRS_MAC_UC, MVPP2_DEL_MAC_NOT_IN_LIST);
    else if (is_multicast_ether_addr(addr))
        mv_pp2x_prs_mac_entry_del(port, MVPP2_PRS_MAC_MC, MVPP2_DEL_MAC_NOT_IN_LIST);
    else if (is_broadcast_ether_addr(addr))
        mv_pp2x_prs_mac_entry_del(port, MVPP2_PRS_MAC_BC, MVPP2_DEL_MAC_NOT_IN_LIST);

    return 0;
}

/* Enable or disable RSS */
void pp2_port_set_rss(struct pp2_port *port, uint32_t en)
{
    mv_pp22_rss_enable(port, en);
}

/* Flush MAC addresses */
int pp2_port_flush_mac_addrs(struct pp2_port *port, uint32_t uc, uint32_t mc)
{
    pp2_err("PORT: Not implemented\n");
    return 0;
}

/* Check if unicast promiscuous */
void pp2_port_get_uc_promisc(struct pp2_port *port, uint32_t *en)
{
    pp2_err("PORT: Not implemented\n");
}

/* Check if Multicast promiscuous */
void pp2_port_get_mc_promisc(struct pp2_port *port, uint32_t *en)
{
    pp2_err("PORT: Not implemented\n");
}

/* Add MAC address */
int pp2_port_add_mac_addr(struct pp2_port *port, const uint8_t *addr)
{
    pp2_err("PORT: Not implemented\n");
    return 0;
}

/* Get link status */
int pp2_port_link_status(struct pp2_port *port)
{
    uint32_t link_is_up;
    struct gop_hw *gop = &port->parent->hw.gop;

    /* Check Link status on ethernet port */
    link_is_up = pp2_gop_port_is_link_up(gop, &port->mac_data);

    if (link_is_up) {
        pp2_info("PORT: Port%u - link is up\n", port->id);
        port->mac_data.flags |= MV_EMAC_F_LINK_UP;
    } else {
        pp2_info("PORT: Port%u - link is down\n", port->id);
        port->mac_data.flags &= ~MV_EMAC_F_LINK_UP;
    }

    return link_is_up;
}
