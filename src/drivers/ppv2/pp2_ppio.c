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
#include "pp2_hif.h"
#include "pp2.h"
#include "pp2_port.h"
#include "lib/lib_misc.h"
#include "cls/pp2_cls_mng.h"

static struct pp2_ppio ppio_array[PP2_MAX_NUM_PACKPROCS][PP2_NUM_ETH_PPIO];

static inline struct pp2_dm_if *pp2_dm_if_get(struct pp2_ppio *ppio, struct pp2_hif *hif)
{
	return GET_PPIO_PORT(ppio)->parent->dm_ifs[hif->regspace_slot];
}


int pp2_ppio_init(struct pp2_ppio_params *params, struct pp2_ppio **ppio)
{
	u8  match[2];
	int port_id, pp2_id, rc;
	struct pp2_port **port;

	if (mv_sys_match(params->match, "ppio", 2, match)) {
		pr_err("[%s] Invalid match string!\n", __func__);
		return -ENXIO;
	}

	if (pp2_is_init() == false)
		return -EPERM;

	pp2_id = match[0];
	port_id = match[1];

	if (port_id >= PP2_NUM_ETH_PPIO) {
		pr_err("[%s] Invalid ppio.\n", __func__);
		return -ENXIO;
	}
	if (pp2_id >= pp2_ptr->num_pp2_inst) {
		pr_err("[%s] Invalid pp2 instance.\n", __func__);
		return -ENXIO;
	}

	if (!pp2_ppio_available(pp2_id, port_id)) {
		pr_err("[%s] %s is not available for musdk.\n", __func__, params->match);
		return -EINVAL;
	}

	port = GET_PPIO_PORT_PTR(ppio_array[pp2_id][port_id]);
	if (*port) {
		pr_err("[%s] ppio already exists.\n", __func__);
		return -EEXIST;
	}

	rc = pp2_port_open(pp2_ptr, params, pp2_id, port_id, port);
	if (!rc) {
		*ppio = &ppio_array[pp2_id][port_id];
	} else {
		pr_err("[%s] ppio init failed.\n", __func__);
		return(-EFAULT);
	}
	pp2_port_config_inq(*port);
	pp2_port_config_outq(*port);
	(*ppio)->pp2_id = pp2_id;
	(*ppio)->port_id = port_id;

	rc = pp2_cls_mng_eth_start_header_params_set(*ppio, params->eth_start_hdr);
	if (rc) {
		pr_err("[%s] ppio init failed while initialize ethernet start header\n", __func__);
		return -EFAULT;
	}

	if (params->type == PP2_PPIO_T_LOG) {
		rc = pp2_cls_mng_set_logical_port_params(*ppio, params);
		if (rc) {
			pr_err("[%s] ppio init failed while initialize logical port\n", __func__);
			return -EFAULT;
		}
	}

	pp2_cls_mng_set_default_policing(*ppio, false);
	if (rc) {
		pr_err("[%s] ppio init failed while initialize policing\n", __func__);
		return -EFAULT;
	}

	pp2_ppio_get_statistics(*ppio, NULL, true);
	pp2_ppio_set_loopback(*ppio, false);

	return rc;
}

void pp2_ppio_deinit(struct pp2_ppio *ppio)
{
	struct pp2_port **port_ptr = NULL;

	port_ptr = GET_PPIO_PORT_PTR(ppio_array[ppio->pp2_id][ppio->port_id]);

	if (*port_ptr) {
		if (pp2_cls_mng_set_default_policing(ppio, true))
			pr_err("[%s] ppio deinit failed while deinitialize policing\n", __func__);

		pp2_port_close(*port_ptr);
		*port_ptr = NULL;
	} else
		pr_err("[%s] ppio deinit failed close port %d:%d\n", __func__, ppio->pp2_id, ppio->port_id);
}

int pp2_ppio_enable(struct pp2_ppio *ppio)
{
	pp2_port_start(GET_PPIO_PORT(ppio), PP2_TRAFFIC_INGRESS_EGRESS);
	return 0;
}

int pp2_ppio_disable(struct pp2_ppio *ppio)
{
	pp2_port_stop(GET_PPIO_PORT(ppio));
	return 0;
}

/* Note: Function cannot be inlined, because of reference to pool->id */
void pp2_ppio_outq_desc_set_pool(struct pp2_ppio_desc *desc, struct pp2_bpool *pool)
{
	desc->cmds[0] = (desc->cmds[0] & ~(TXD_POOL_ID_MASK | TXD_BUFMODE_MASK)) |
		(pool->id << 16 & TXD_POOL_ID_MASK) | (1 << 7 & TXD_BUFMODE_MASK);
}

int pp2_ppio_inq_get_statistics(struct pp2_ppio *ppio, u8 tc, u8 qid,
				struct pp2_ppio_inq_statistics *stats, int reset)
{
	struct pp2_port *port = GET_PPIO_PORT(ppio);
	uintptr_t cpu_slot = port->cpu_slot;
	struct pp2_rx_queue *rxq;
	int log_rxq;

	if (unlikely(qid >= port->tc[tc].tc_config.num_in_qs)) {
		pr_err("[%s] invalid queue id (%d)!\n", __func__, qid);
		return -EINVAL;
	}

	log_rxq = port->tc[tc].first_log_rxq + qid;
	rxq = port->rxqs[log_rxq];

	pp2_relaxed_reg_write(cpu_slot, MVPP2_CNT_IDX_REG, rxq->id);
	PP2_READ_UPDATE_CNT64(rxq->stats.enq_desc, cpu_slot, MVPP2_RX_DESC_ENQ_REG);
	PP2_READ_UPDATE_CNT32(rxq->stats.drop_fullq, cpu_slot, MVPP2_RX_PKT_FULLQ_DROP_REG);
	PP2_READ_UPDATE_CNT16(rxq->stats.drop_early, cpu_slot, MVPP2_RX_PKT_EARLY_DROP_REG);
	PP2_READ_UPDATE_CNT16(rxq->stats.drop_bm, cpu_slot, MVPP2_RX_PKT_BM_DROP_REG);

	if (stats)
		memcpy(stats, &rxq->stats, sizeof(rxq->stats));

	if (reset)
		memset(&rxq->stats, 0, sizeof(rxq->stats));

	return 0;
}

int pp2_ppio_outq_get_statistics(struct pp2_ppio *ppio, u8 qid,
				 struct pp2_ppio_outq_statistics *stats, int reset)
{
	struct pp2_port *port = GET_PPIO_PORT(ppio);
	uintptr_t cpu_slot = port->cpu_slot;
	struct pp2_tx_queue *txq;

	if (unlikely(qid >= port->num_tx_queues)) {
		pr_err("[%s] invalid queue id (%d)!\n", __func__, qid);
		return -EINVAL;
	}
	txq = port->txqs[qid];

	pp2_relaxed_reg_write(cpu_slot, MVPP2_CNT_IDX_REG, MVPP2_CNT_IDX_TX(port->id, txq->log_id));
	PP2_READ_UPDATE_CNT64(txq->stats.enq_desc, cpu_slot, MVPP2_TX_DESC_ENQ_REG);
	PP2_READ_UPDATE_CNT64(txq->stats.enq_dec_to_ddr, cpu_slot, MVPP2_TX_DESC_ENQ_TO_DRAM_REG);
	PP2_READ_UPDATE_CNT64(txq->stats.enq_buf_to_ddr, cpu_slot, MVPP2_TX_BUF_ENQ_TO_DRAM_REG);
	PP2_READ_UPDATE_CNT64(txq->stats.deq_desc, cpu_slot, MVPP2_TX_PKT_DQ_REG);

	if (stats)
		memcpy(stats, &txq->stats, sizeof(txq->stats));

	if (reset)
		memset(&txq->stats, 0, sizeof(txq->stats));

	return 0;

}

int pp2_ppio_send(struct pp2_ppio *ppio, struct pp2_hif *hif, u8 qid, struct pp2_ppio_desc *descs, u16 *num)
{
	struct pp2_dm_if *dm_if;
	u16 desc_sent, desc_req = *num;
	struct pp2_port *port = GET_PPIO_PORT(ppio);

	dm_if = pp2_dm_if_get(ppio, hif);

	desc_sent = pp2_port_enqueue(port, dm_if, qid, desc_req, descs);
	if (unlikely(desc_sent < desc_req)) {
		pr_debug("[%s] pp2_id %u Port %u qid %u, send_request %u sent %u!\n", __func__,
			 ppio->pp2_id, ppio->port_id, qid, *num, desc_sent);
		*num = desc_sent;
	}

	if (port->maintain_stats) {
		struct pp2_tx_queue *txq;

		txq = port->txqs[qid];
		txq->threshold_tx_pkts += desc_sent;
		if (unlikely(txq->threshold_tx_pkts > PP2_STAT_UPDATE_THRESHOLD)) {
			pp2_ppio_outq_get_statistics(ppio, qid, NULL, 0);
			txq->threshold_tx_pkts = 0;
		}
	}
	return 0;
}

int pp2_ppio_send_sg(struct pp2_ppio *ppio,
		     struct pp2_hif *hif,
		     u8  qid,
		     struct pp2_ppio_sg_desc *descs,
		     u16 *num)
{
	pr_err("[%s] routine not supported yet!\n", __func__);
	return -ENOTSUP;
}

int pp2_ppio_get_num_outq_done(struct pp2_ppio *ppio,
			       struct pp2_hif *hif,
			       u8 qid,
			       u16 *num)
{
	struct pp2_dm_if *dm_if;
	u32 outq_physid;

	dm_if = pp2_dm_if_get(ppio, hif);
	outq_physid = GET_PPIO_PORT(ppio)->txqs[qid]->id;
	*num = pp2_port_outq_status(dm_if, outq_physid);

	return 0;
}

/* ALL TX Setter functions, and RX Getter functions are u32 based */
static inline void pp2_ppio_desc_swap_ncopy(struct pp2_ppio_desc *dst, struct pp2_ppio_desc *src)
{
	u32 *src_cmd = (u32 *)src;
	u32 *dst_cmd = (u32 *)dst;
	int i;

	for (i = 0; i < PP2_PPIO_DESC_NUM_WORDS; i++) {
		*dst_cmd = le32_to_cpu(*src_cmd);
		dst_cmd++;
		src_cmd++;
	}
}

int pp2_ppio_recv(struct pp2_ppio *ppio, u8 tc, u8 qid, struct pp2_ppio_desc *descs, u16 *num)
{
	struct pp2_port *port = GET_PPIO_PORT(ppio);
	struct pp2_desc *rx_desc, *extra_rx_desc;
	struct pp2_rx_queue *rxq;
	u32 recv_req = *num, extra_num = 0;
	int log_rxq;
#if __BYTE_ORDER == __BIG_ENDIAN
	int i;
#endif

	/* TODO: After validation, delete recv_req variable */
	log_rxq = port->tc[tc].first_log_rxq + qid;
	rxq = port->rxqs[log_rxq];

	if (recv_req > rxq->desc_received) {
		rxq->desc_received = pp2_rxq_received(port, rxq->id);
		if (unlikely(recv_req > rxq->desc_received)) {
			recv_req = rxq->desc_received;
			*num = recv_req;
		}
	}

	/* TODO : Make pp2_rxq_get_desc inline */
	rx_desc = pp2_rxq_get_desc(rxq, &recv_req, &extra_rx_desc, &extra_num);
#if __BYTE_ORDER == __BIG_ENDIAN
	for (i = 0; i < recv_req; i++)
		pp2_ppio_desc_swap_ncopy(&descs[i], &rx_desc[i]);
#else
	memcpy(descs, rx_desc, recv_req * sizeof(*descs));
#endif
	if (extra_num) {
		memcpy(&descs[recv_req], extra_rx_desc, extra_num * sizeof(*descs));
		recv_req += extra_num; /* Put the split numbers back together */
	}
	/*  Update HW */
	pp2_port_inq_update(port, log_rxq, recv_req, recv_req);
	rxq->desc_received -= recv_req;

	if (port->maintain_stats) {
		rxq->threshold_rx_pkts += recv_req;
		if (unlikely(rxq->threshold_rx_pkts > PP2_STAT_UPDATE_THRESHOLD)) {
			pp2_ppio_inq_get_statistics(ppio, tc, qid, NULL, 0);
			rxq->threshold_rx_pkts = 0;
		}
	}
	return 0;
}

int pp2_ppio_set_mac_addr(struct pp2_ppio *ppio, const eth_addr_t addr)
{
	int rc;

	rc = pp2_port_set_mac_addr(GET_PPIO_PORT(ppio), (const uint8_t *)addr);
	return rc;
}

int pp2_ppio_get_mac_addr(struct pp2_ppio *ppio, eth_addr_t addr)
{
	int rc;

	rc = pp2_port_get_mac_addr(GET_PPIO_PORT(ppio), (uint8_t *)addr);
	return rc;
}

int pp2_ppio_set_mtu(struct pp2_ppio *ppio, u16 mtu)
{
	int rc;

	rc = pp2_port_set_mtu(GET_PPIO_PORT(ppio), mtu);
	return rc;
}

int pp2_ppio_get_mtu(struct pp2_ppio *ppio, u16 *mtu)
{
	pp2_port_get_mtu(GET_PPIO_PORT(ppio), mtu);
	return 0;
}

int pp2_ppio_set_mru(struct pp2_ppio *ppio, u16 len)
{
	int rc;

	rc = pp2_port_set_mru(GET_PPIO_PORT(ppio), len);
	return rc;
}

int pp2_ppio_get_mru(struct pp2_ppio *ppio, u16 *len)
{
	pp2_port_get_mru(GET_PPIO_PORT(ppio), len);

	return 0;
}

int pp2_ppio_set_loopback(struct pp2_ppio *ppio, int en)
{
	int rc;

	rc = pp2_port_set_loopback(GET_PPIO_PORT(ppio), en);
	return rc;
}

int pp2_ppio_get_loopback(struct pp2_ppio *ppio, int *en)
{
	int rc;

	rc = pp2_port_get_loopback(GET_PPIO_PORT(ppio), en);
	return rc;
}

int pp2_ppio_set_promisc(struct pp2_ppio *ppio, int en)
{
	int rc;

	rc = pp2_port_set_promisc(GET_PPIO_PORT(ppio), en);
	return rc;
}

int pp2_ppio_get_promisc(struct pp2_ppio *ppio, int *en)
{
	int rc;

	rc = pp2_port_get_promisc(GET_PPIO_PORT(ppio), (u32 *)en);
	return rc;
}

int pp2_ppio_set_mc_promisc(struct pp2_ppio *ppio, int en)
{
	int rc;

	rc = pp2_port_set_mc_promisc(GET_PPIO_PORT(ppio), en);
	return rc;
}

int pp2_ppio_get_mc_promisc(struct pp2_ppio *ppio, int *en)
{
	int rc;

	rc = pp2_port_get_mc_promisc(GET_PPIO_PORT(ppio), (u32 *)en);
	return rc;
}

int pp2_ppio_add_mac_addr(struct pp2_ppio *ppio, const eth_addr_t addr)
{
	int rc;

	rc = pp2_port_add_mac_addr(GET_PPIO_PORT(ppio), (const uint8_t *)addr);
	return rc;
}

int pp2_ppio_remove_mac_addr(struct pp2_ppio *ppio, const eth_addr_t addr)
{
	int rc;

	rc = pp2_port_remove_mac_addr(GET_PPIO_PORT(ppio), (const uint8_t *)addr);
	return rc;
}

int pp2_ppio_flush_mac_addrs(struct pp2_ppio *ppio, int uc, int mc)
{
	int rc;

	rc = pp2_port_flush_mac_addrs(GET_PPIO_PORT(ppio), uc, mc);
	return rc;
}

int pp2_ppio_get_link_state(struct pp2_ppio *ppio, int *en)
{
	int rc;

	rc = pp2_port_get_loopback(GET_PPIO_PORT(ppio), en);
	if (!rc && *en)
		return rc;

	rc = pp2_port_get_link_state(GET_PPIO_PORT(ppio), en);
	return rc;
}

int pp2_ppio_add_vlan(struct pp2_ppio *ppio, u16 vlan)
{
	int rc;

	rc = pp2_port_add_vlan(GET_PPIO_PORT(ppio), vlan);
	return rc;
}

int pp2_ppio_remove_vlan(struct pp2_ppio *ppio, u16 vlan)
{
	int rc;

	rc = pp2_port_remove_vlan(GET_PPIO_PORT(ppio), vlan);
	return rc;
}

int pp2_ppio_flush_vlan(struct pp2_ppio *ppio)
{
	pr_err("[%s] routine not supported yet!\n", __func__);
	return -ENOTSUP;
}

int pp2_ppio_get_statistics(struct pp2_ppio *ppio, struct pp2_ppio_statistics *stats, int reset)
{
	struct pp2_ppio_statistics cur_stats;
	struct pp2_port *port = GET_PPIO_PORT(ppio);
	int qid, tc;

	memset(&cur_stats, 0, sizeof(struct pp2_ppio_statistics));
	pp2_port_get_statistics(port, &cur_stats);

	/* Update Rx Qs Statistics */
	for (tc = 0; tc < port->num_tcs; tc++) {
		for (qid = 0; qid < port->tc[tc].tc_config.num_in_qs; qid++) {
			struct pp2_ppio_inq_statistics rx_stats;

			pp2_ppio_inq_get_statistics(ppio, tc, qid, &rx_stats, reset);
			cur_stats.rx_packets += rx_stats.enq_desc;
			cur_stats.rx_fullq_dropped += rx_stats.drop_fullq;
			cur_stats.rx_bm_dropped += rx_stats.drop_bm;
			cur_stats.rx_early_dropped += rx_stats.drop_early;
		}
	}

	/* Update Tx Qs Statistics */
	for (qid = 0; qid < port->num_tx_queues; qid++) {
		struct pp2_ppio_outq_statistics tx_stats;

		pp2_ppio_outq_get_statistics(ppio, qid, &tx_stats, reset);
		cur_stats.tx_packets += tx_stats.enq_desc;
	}

	if (stats) {
		stats->rx_packets = cur_stats.rx_packets;
		stats->rx_fullq_dropped = cur_stats.rx_fullq_dropped;
		stats->rx_bm_dropped = cur_stats.rx_bm_dropped;
		stats->rx_early_dropped = cur_stats.rx_early_dropped;
		stats->tx_packets = cur_stats.tx_packets - port->stats.tx_packets;
		/* From KS */
		stats->rx_bytes = cur_stats.rx_bytes - port->stats.rx_bytes;
		stats->rx_unicast_packets = cur_stats.rx_unicast_packets - port->stats.rx_unicast_packets;
		stats->rx_errors = cur_stats.rx_errors - port->stats.rx_errors;
		stats->rx_fifo_dropped = cur_stats.rx_fifo_dropped - port->stats.rx_fifo_dropped;
		stats->rx_cls_dropped = cur_stats.rx_cls_dropped - port->stats.rx_cls_dropped;
		stats->tx_bytes = cur_stats.tx_bytes - port->stats.tx_bytes;
		stats->tx_unicast_packets = cur_stats.tx_unicast_packets - port->stats.tx_unicast_packets;
		stats->tx_errors = cur_stats.tx_errors - port->stats.tx_errors;
	}

	if (reset)
		memcpy(&port->stats, &cur_stats, sizeof(struct pp2_ppio_statistics));

	return 0;

}

