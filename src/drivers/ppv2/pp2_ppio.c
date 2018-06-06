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

	if (pp2_ptr->pp2_inst[pp2_id]->ppios[port_id]) {
		pr_err("[%s] ppio already exists.\n", __func__);
		return -EEXIST;
	}

	*ppio = kcalloc(1, sizeof(struct pp2_ppio), GFP_KERNEL);
	if (!*ppio) {
		pr_err("%s out of memory ppio alloc\n", __func__);
		return -ENOMEM;
	}

	port = GET_PPIO_PORT_PTR(**ppio);

	rc = pp2_port_open(pp2_ptr, params, pp2_id, port_id, port);
	if (rc) {
		pr_err("[%s] ppio init failed.\n", __func__);
		kfree(*ppio);
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

	rc = pp2_cls_mng_set_default_policing(*ppio, false);
	if (rc) {
		pr_err("[%s] ppio init failed while initialize policing\n", __func__);
		return -EFAULT;
	}

	pp2_ppio_get_statistics(*ppio, NULL, true);
	pp2_ppio_set_loopback(*ppio, false);

	pp2_ptr->pp2_inst[pp2_id]->ppios[port_id] = *ppio;

	return rc;
}

void pp2_ppio_deinit(struct pp2_ppio *ppio)
{
	struct pp2_port **port_ptr = NULL;

	port_ptr = GET_PPIO_PORT_PTR(*ppio);

	if (*port_ptr) {
		if (pp2_cls_mng_set_default_policing(ppio, true))
			pr_err("[%s] ppio deinit failed while deinitialize policing\n", __func__);

		pp2_port_close(*port_ptr);
		*port_ptr = NULL;
		pp2_ptr->pp2_inst[ppio->pp2_id]->ppios[ppio->port_id] = NULL;
		kfree(ppio);
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

int pp2_ppio_set_outq_state(struct pp2_ppio *ppio, u8 qid, int en)
{
	struct pp2_port *port = GET_PPIO_PORT(ppio);
	struct pp2_tx_queue *txq;

	if (unlikely(qid >= port->num_tx_queues)) {
		pr_err("[%s] invalid queue id (%d)!\n", __func__, qid);
		return -EINVAL;
	}
	txq = port->txqs[qid];

	if (unlikely(!txq || !txq->desc_virt_arr)) {
		pr_err("[%s] invalid queue id (%d)!\n", __func__, qid);
		return -EINVAL;
	}

	return pp2_port_set_outq_state(port, txq, en);

}

int pp2_ppio_get_outq_state(struct pp2_ppio *ppio, u8 qid, int *en)
{
	struct pp2_port *port = GET_PPIO_PORT(ppio);
	uintptr_t cpu_slot = port->cpu_slot;
	struct pp2_tx_queue *txq;
	u32 val = 0, mask;

	if (unlikely(qid >= port->num_tx_queues)) {
		pr_err("[%s] invalid queue id (%d)!\n", __func__, qid);
		return -EINVAL;
	}
	txq = port->txqs[qid];

	if (unlikely(!txq->desc_virt_arr)) {
		pr_err("[%s] invalid queue id (%d)!\n", __func__, qid);
		return -EINVAL;
	}

	/* Get active channels mask */
	pp2_reg_write(cpu_slot, MVPP2_TXP_SCHED_PORT_INDEX_REG, MVPP2_MAX_TCONT + port->id);
	val = (pp2_reg_read(cpu_slot, MVPP2_TXP_SCHED_Q_CMD_REG) & MVPP2_TXP_SCHED_ENQ_MASK);
	mask = 1 << qid;

	if (val & mask)
		*en = 1;
	else
		*en = 0;

	return 0;
}

int pp2_ppio_send(struct pp2_ppio *ppio, struct pp2_hif *hif, u8 qid, struct pp2_ppio_desc *descs, u16 *num)
{
	struct pp2_dm_if *dm_if;
	u16 desc_sent, desc_req = *num;
	struct pp2_port *port = GET_PPIO_PORT(ppio);

	dm_if = pp2_dm_if_get(ppio, hif);

	desc_sent = pp2_port_enqueue(port, dm_if, qid, desc_req, descs, NULL);
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
		     struct pp2_ppio_desc *descs,
		     u16 *desc_num,
		     struct pp2_ppio_sg_pkts *pkts
		     )
{
	struct pp2_dm_if *dm_if;
	u16 desc_sent, desc_req = *desc_num;
	struct pp2_port *port = GET_PPIO_PORT(ppio);
	int i, j, k = 0;

	dm_if = pp2_dm_if_get(ppio, hif);

	pr_debug("[%s] %u:%u: sending %d packets %d descriptors:\n", __func__,
		 ppio->pp2_id, ppio->port_id, pkts->num, desc_req);
	for (i = 0; i < pkts->num; i++) {
		/*
		 * Handle packets with only one fragment.
		 * TXD_FIRST_LAST flag was already set by pp2_ppio_outq_desc_reset().
		 */
		if (pkts->frags[i] == 1) {
			pr_debug("pkt:%d, frg:%d: first-last %.8X\n", i, k, descs[k].cmds[0]);
			k++;
			continue;
		}

		/* Handle first fragment */
		DM_TXD_SET_FIRST_LAST(&descs[k], TXD_FIRST);
		pr_debug("pkt:%d, frg:%d: first %.8X\n", i, k, descs[k].cmds[0]);
		k++;

		/* Handle middle fragments */
		for (j = 1; j < pkts->frags[i] - 1;  j++) {
			DM_TXD_SET_FIRST_LAST(&descs[k], 0);
			pr_debug("pkt:%d, frg:%d: middle %.8X\n", i, k, descs[k].cmds[0]);
			k++;
		}

		/* Handle last fragment */
		DM_TXD_SET_FIRST_LAST(&descs[k], TXD_LAST);
		pr_debug("pkt:%d, frg:%d: last %.8X\n", i, k, descs[k].cmds[0]);
		k++;
	}

	desc_sent = pp2_port_enqueue(port, dm_if, qid, desc_req, descs, pkts);
	if (unlikely(desc_sent < desc_req)) {
		pr_debug("[%s] pp2_id %u Port %u qid %u, send_request %u sent %u!\n", __func__,
			 ppio->pp2_id, ppio->port_id, qid, *desc_num, desc_sent);
		*desc_num = desc_sent;
	}

	if (port->maintain_stats) {
		struct pp2_tx_queue *txq;

		txq = port->txqs[qid];
		txq->threshold_tx_pkts += pkts->num;
		if (unlikely(txq->threshold_tx_pkts > PP2_STAT_UPDATE_THRESHOLD)) {
			pp2_ppio_outq_get_statistics(ppio, qid, NULL, 0);
			txq->threshold_tx_pkts = 0;
		}
	}
	return 0;

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
static inline void pp2_ppio_desc_swap_ncopy(struct pp2_ppio_desc *dst, struct pp2_desc *src)
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
	__builtin_memcpy(descs, rx_desc, recv_req * sizeof(*descs));
#endif
	if (extra_num) {
#if __BYTE_ORDER == __BIG_ENDIAN
		for (i = 0; i < extra_num; i++)
			pp2_ppio_desc_swap_ncopy(&descs[recv_req+i], &extra_rx_desc[i]);
#else
		__builtin_memcpy(&descs[recv_req], extra_rx_desc, extra_num * sizeof(*descs));
#endif
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

int pp2_ppio_set_rx_pause(struct pp2_ppio *ppio, int en)
{
	int rc;

	rc = pp2_port_set_rx_pause(GET_PPIO_PORT(ppio), en);
	return rc;

}

int pp2_ppio_get_rx_pause(struct pp2_ppio *ppio, int *en)
{
	int rc;

	rc = pp2_port_get_rx_pause(GET_PPIO_PORT(ppio), en);
	return rc;

}
int pp2_ppio_set_tx_pause(struct pp2_ppio *ppio, struct pp2_ppio_tx_pause_params *params)
{
	/* Check ptr */
	int rc;

	rc = pp2_port_set_tx_pause(GET_PPIO_PORT(ppio), params);

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

	/* Get rx and tx packets counters from queues and not from GOP */
	cur_stats.rx_packets = 0;
	cur_stats.tx_packets = 0;

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
		stats->tx_packets = cur_stats.tx_packets;
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

int pp2_ppio_get_capabilities(struct pp2_ppio *ppio, struct pp2_ppio_capabilities *capa)
{
	int			 i, j;
	struct pp2_port		*port;
	struct pp2_inst		*inst;

	inst = pp2_ptr->pp2_inst[ppio->pp2_id];
	port = inst->ports[ppio->port_id];

	capa->pp2_id = ppio->pp2_id;
	capa->id = ppio->port_id;
	capa->intcs_inf.num_intcs = port->num_tcs;
	for (i = 0; i < capa->intcs_inf.num_intcs; i++) {
		capa->intcs_inf.intcs_infs[i].pkt_offset = port->tc[i].tc_config.pkt_offset;
		capa->intcs_inf.intcs_infs[i].num_inqs = port->tc[i].tc_config.num_in_qs;
		for (j = 0; j < capa->intcs_inf.intcs_infs[i].num_inqs; j++)
			capa->intcs_inf.intcs_infs[i].inqs_infs[j].size = port->tc[i].rx_qs[j].ring_size;
	}
	capa->outqs_inf.num_outtcs = port->num_tx_queues;
	for (i = 0; i < capa->outqs_inf.num_outtcs; i++)
		capa->outqs_inf.outqs_infs[i].size = port->txq_config[i].size;

	return 0;
}

int pp2_ppio_serialize(struct pp2_ppio *ppio, char buff[], u32 size)
{
	struct pp2_port			*port = GET_PPIO_PORT(ppio);
	struct pp2_port			*lb_port = NULL;
	size_t				 pos = 0;
	int				 i;
	struct mv_sys_dma_mem_info	 mem_info;
	char				 dev_name[100];
	phys_addr_t			 paddr;
	char				*sec = NULL;

	/* Serialize ppio parameters*/

	/* Find if there is already a ppio-info section */
	sec = strstr(buff, "ppio-info");
	if (!sec) {
		/* no ppio-info section, create one */
		pos = strlen(buff);
		json_print_to_buffer(buff, size, 1, "\"ppio-info\": {\n");
	} else {
		/* Find the last ppio instance
		 * Note, it is assumed that all ppios are written together at the end of the buffer
		 * i.e. "insertion" of a ppio section is not supported
		 */
		/* If this is not the first object, add a "," after the last "}" to sepparate the objects*/
		pos = strlen(buff) - JSON_LEVEL_2_TAB_COMMA_ADDITION_LEN;
		json_print_to_buffer(buff, size, 0, "},\n");
	}

	mem_info.name = dev_name;
	mv_sys_dma_mem_get_info(&mem_info);
	paddr = mem_info.paddr;

	json_print_to_buffer(buff, size, 2, "\"ppio-%d:%d\": {\n", ppio->pp2_id, ppio->port_id);
	json_print_to_buffer(buff, size, 3, "\"dma_dev_name\": \"%s\",\n", mem_info.name);
	json_print_to_buffer(buff, size, 3, "\"pp2_id\": %u,\n", ppio->pp2_id);
	json_print_to_buffer(buff, size, 3, "\"port_id\": %u,\n", ppio->port_id);
	json_print_to_buffer(buff, size, 3, "\"port-info\": {\n");
	json_print_to_buffer(buff, size, 4, "\"id\": %u,\n", port->id);
	json_print_to_buffer(buff, size, 4, "\"flags\": %u,\n", port->flags);
	json_print_to_buffer(buff, size, 4, "\"port_mru\": %u,\n", port->port_mru);
	json_print_to_buffer(buff, size, 4, "\"port_mtu\": %u,\n", port->port_mtu);
	json_print_to_buffer(buff, size, 4, "\"first_rxq\": %u,\n", port->first_rxq);
	json_print_to_buffer(buff, size, 4, "\"use_mac_lb\": %u,\n", port->use_mac_lb);
	json_print_to_buffer(buff, size, 4, "\"t_mode\": %u,\n", port->t_mode);
	json_print_to_buffer(buff, size, 4, "\"hash_type\": %u,\n", port->hash_type);
	json_print_to_buffer(buff, size, 4, "\"rss_en\": %u,\n", port->rss_en);
	json_print_to_buffer(buff, size, 4, "\"tx_fifo_size\": %u,\n", port->tx_fifo_size);
	json_print_to_buffer(buff, size, 4, "\"maintain_stats\": %u,\n", port->maintain_stats);
	json_print_to_buffer(buff, size, 4, "\"type\": %u,\n", port->type);
	json_print_to_buffer(buff, size, 4, "\"linux_name\": \"%s\",\n", port->linux_name);

	json_print_to_buffer(buff, size, 4, "\"ppio-port-mac_data\": {\n");
	json_print_to_buffer(buff, size, 5, "\"gop_index\": %u,\n", port->mac_data.gop_index);
	json_print_to_buffer(buff, size, 5, "\"flags\": %lu,\n", port->mac_data.flags);
	json_print_to_buffer(buff, size, 5, "\"phy_addr\": %u,\n", port->mac_data.phy_addr);
	json_print_to_buffer(buff, size, 5, "\"phy_mode\": %u,\n", port->mac_data.phy_mode);
	json_print_to_buffer(buff, size, 5, "\"force_link\": %u,\n", port->mac_data.force_link);
	json_print_to_buffer(buff, size, 5, "\"autoneg\": %u,\n", port->mac_data.autoneg);
	json_print_to_buffer(buff, size, 5, "\"link\": %u,\n", port->mac_data.link);
	json_print_to_buffer(buff, size, 5, "\"duplex\": %u,\n", port->mac_data.duplex);
	json_print_to_buffer(buff, size, 5, "\"speed\": %u,\n", port->mac_data.speed);
	json_print_to_buffer(buff, size, 5, "\"mac_address\": \"%02x:%02x:%02x:%02x:%02x:%02x\"\n",
		port->mac_data.mac[0], port->mac_data.mac[1], port->mac_data.mac[2],
		port->mac_data.mac[3], port->mac_data.mac[4], port->mac_data.mac[5]);
	json_print_to_buffer(buff, size, 4, "},\n");
	json_print_to_buffer(buff, size, 4, "\"num_tcs\": %u,\n", port->num_tcs);
	json_print_to_buffer(buff, size, 4, "\"num_rx_queues\": %u,\n", port->num_rx_queues);
	json_print_to_buffer(buff, size, 4, "\"num_tx_queues\": %u,\n", port->num_tx_queues);

	for (i = 0; i < port->num_tcs; i++) {
		int	j, k, num_bps;

		json_print_to_buffer(buff, size, 4, "\"ppio-port-tc-%d\": {\n", i);
		json_print_to_buffer(buff, size, 5, "\"first_log_rxq\": %u,\n", port->tc[i].first_log_rxq);
		json_print_to_buffer(buff, size, 5, "\"num_in_qs\": %u,\n", port->tc[i].tc_config.num_in_qs);
		for (j = 0; j < port->tc[i].tc_config.num_in_qs; j++) {
			json_print_to_buffer(buff, size, 5, "\"ring_size-%d\": %u,\n", j,
					port->tc[i].rx_qs[j].ring_size);
			json_print_to_buffer(buff, size, 5, "\"tc_pools_mem_id_index-%d\": %u,\n", j,
					port->tc[i].rx_qs[j].tc_pools_mem_id_index);
		}
		json_print_to_buffer(buff, size, 5, "\"pkt_offset\": %u,\n", port->tc[i].tc_config.pkt_offset);
		json_print_to_buffer(buff, size, 5, "\"first_rxq\": %u,\n", port->tc[i].tc_config.first_rxq);
		num_bps = 0;
		for (j = 0; j < MV_SYS_DMA_MAX_NUM_MEM_ID; j++) {
			for (k = 0; k < PP2_PPIO_TC_CLUSTER_MAX_POOLS; k++) {
				if (port->tc[i].tc_config.pools[j][k])
					num_bps++;
			}
		}
		json_print_to_buffer(buff, size, 5, "\"num_bpools\": %u,\n", num_bps);
		for (j = 0; j < MV_SYS_DMA_MAX_NUM_MEM_ID; j++) {
			for (k = 0; k < PP2_PPIO_TC_CLUSTER_MAX_POOLS; k++) {
				if (port->tc[i].tc_config.pools[j][k]) {
					json_print_to_buffer(buff, size, 5,
							    "\"tc-%d-bpool-%d:%d\": {\n", i, j, k);
					json_print_to_buffer(buff, size, 6, "\"mem_id\": %u,\n", j);
					json_print_to_buffer(buff, size, 6, "\"cluster_id\": %u,\n", k);

					json_print_to_buffer(buff, size, 6, "\"pp2_id\": %u,\n",
							port->tc[i].tc_config.pools[j][k]->pp2_id);
					json_print_to_buffer(buff, size, 6, "\"bm_pool_id\": %u\n",
						port->tc[i].tc_config.pools[j][k]->bm_pool_id);
					if (--num_bps == 0)
						json_print_to_buffer(buff, size, 5, "}\n");
					else
						json_print_to_buffer(buff, size, 5, "},\n");
				}
			}
		}
		json_print_to_buffer(buff, size, 4, "},\n");

	}

	for (i = 0; i < port->num_rx_queues; i++) {
		json_print_to_buffer(buff, size, 4, "\"ppio-port-rxq-%d\": {\n", i);
		json_print_to_buffer(buff, size, 5, "\"id-%d\": %u\n", i, port->rxqs[i]->id);
		json_print_to_buffer(buff, size, 5, "\"log_id-%d\": %u\n", i, port->rxqs[i]->log_id);
		json_print_to_buffer(buff, size, 5, "\"desc_total-%d\": %u\n", i, port->rxqs[i]->desc_total);
		json_print_to_buffer(buff, size, 5, "\"rxq_lock-%d\": %u\n", i, port->rxqs[i]->rxq_lock);
		json_print_to_buffer(buff, size, 5, "\"threshold_rx_pkts-%d\": %u\n", i,
				     port->rxqs[i]->threshold_rx_pkts);
		json_print_to_buffer(buff, size, 5, "\"desc_phys_offset-%d\": 0x%llx\n", i,
				(unsigned long long int)port->rxqs[i]->desc_phys_arr - paddr);
		json_print_to_buffer(buff, size, 4, "},\n");
	}

	for (i = 0; i < port->num_tx_queues; i++) {
		json_print_to_buffer(buff, size, 4, "\"ppio-port-txq_config-%d\": {\n", i);
		json_print_to_buffer(buff, size, 5, "\"size\": %u\n", port->txq_config[i].size);
		json_print_to_buffer(buff, size, 5, "\"weight\": %u\n", port->txq_config[i].weight);
		json_print_to_buffer(buff, size, 4, "},\n");
	}

	for (i = 0; i < port->num_tx_queues; i++) {
		json_print_to_buffer(buff, size, 4, "\"ppio-port-txq-%d\": {\n", i);
		json_print_to_buffer(buff, size, 5, "\"id-%d\": %u,\n", i, port->txqs[i]->id);
		json_print_to_buffer(buff, size, 5, "\"log_id-%d\": %u,\n", i, port->txqs[i]->log_id);
		json_print_to_buffer(buff, size, 5, "\"desc_total-%d\": %u,\n", i, port->txqs[i]->desc_total);
		json_print_to_buffer(buff, size, 5, "\"threshold_tx_pkts-%d\": %u,\n", i,
				     port->txqs[i]->threshold_tx_pkts);
		json_print_to_buffer(buff, size, 5, "\"desc_phys_offset-%d\": 0x%llx\n", i,
				(unsigned long long int)port->txqs[i]->desc_phys_arr - paddr);
		if (i == port->num_tx_queues - 1)
			json_print_to_buffer(buff, size, 4, "}\n");
		else
			json_print_to_buffer(buff, size, 4, "},\n");
	}
	json_print_to_buffer(buff, size, 3, "},\n");

	/* Serialize the loopback port as part of the ppio */
	/* Only the tx queues are relevant */
	if (!pp2_ptr->pp2_inst[ppio->pp2_id]->ports[PP2_LOOPBACK_PORT]) {
		pr_info("no loopback port found. Serialization failed\n");
		return -EINVAL;
	}
	lb_port = pp2_ptr->pp2_inst[ppio->pp2_id]->ports[PP2_LOOPBACK_PORT];

	json_print_to_buffer(buff, size, 3, "\"port-lb-info\": {\n");
	json_print_to_buffer(buff, size, 4, "\"id\": %u,\n", lb_port->id);
	json_print_to_buffer(buff, size, 4, "\"type\": %u,\n", lb_port->type);
	json_print_to_buffer(buff, size, 4, "\"num_tx_queues\": %u,\n", lb_port->num_tx_queues);
	for (i = 0; i < lb_port->num_tx_queues; i++) {
		json_print_to_buffer(buff, size, 4, "\"lb-port-txq_config-%d\": {\n", i);
		json_print_to_buffer(buff, size, 5, "\"size\": %u\n", lb_port->txq_config[i].size);
		json_print_to_buffer(buff, size, 5, "\"weight\": %u\n", lb_port->txq_config[i].weight);
		json_print_to_buffer(buff, size, 4, "},\n");
	}
	for (i = 0; i < lb_port->num_tx_queues; i++) {
		json_print_to_buffer(buff, size, 4, "\"lb-port-txq-%d\": {\n", i);
		json_print_to_buffer(buff, size, 5, "\"id-%d\": %u,\n", i, lb_port->txqs[i]->id);
		json_print_to_buffer(buff, size, 5, "\"log_id-%d\": %u,\n", i, lb_port->txqs[i]->log_id);
		json_print_to_buffer(buff, size, 5, "\"desc_total-%d\": %u,\n", i, lb_port->txqs[i]->desc_total);
		json_print_to_buffer(buff, size, 5, "\"threshold_tx_pkts-%d\": %u,\n", i,
				lb_port->txqs[i]->threshold_tx_pkts);
		json_print_to_buffer(buff, size, 5, "\"desc_phys_offset-%d\": 0x%llx\n", i,
				(unsigned long long int)lb_port->txqs[i]->desc_phys_arr - paddr);
		if (i == lb_port->num_tx_queues - 1)
			json_print_to_buffer(buff, size, 4, "}\n");
		else
			json_print_to_buffer(buff, size, 4, "},\n");
	}
	json_print_to_buffer(buff, size, 3, "}\n");
	json_print_to_buffer(buff, size, 2, "}\n");
	json_print_to_buffer(buff, size, 1, "}\n");

	return pos;
}

static int pp2_ppio_probe_tc_params(char *buff, struct pp2_ppio *ppio)
{
	int			 i, j, k, rc;
	char			 tmp_buf[PP2_MAX_BUF_STR_LEN];
	u32			 mem_id = 0;
	u32			 cluster_id = 0;
	u32			 tmp_pp2_id = 0;
	u32			 tmp_bpool_id = 0;
	u32			 num_bps = 0;
	struct pp2_bpool	*param_pools[MV_SYS_DMA_MAX_NUM_MEM_ID][PP2_PPIO_TC_CLUSTER_MAX_POOLS];
	struct pp2_port		*port;
	struct pp2_inst		*inst;
	char			*lbuff;
	char			*sec = NULL;

	lbuff = kcalloc(1, strlen(buff), GFP_KERNEL);
	if (lbuff == NULL)
		return -ENOMEM;

	memcpy(lbuff, buff, strlen(buff));
	sec = lbuff;

	inst = pp2_ptr->pp2_inst[ppio->pp2_id];
	/* extract the loopback info */
	port = pp2_ptr->pp2_inst[ppio->pp2_id]->ports[ppio->port_id];

	for (i = 0; i < port->num_tcs; i++) {

		/* Search for the ppio-port-tc section */
		memset(tmp_buf, 0, sizeof(tmp_buf));
		snprintf(tmp_buf, sizeof(tmp_buf), "ppio-port-tc-%d", i);
		sec = strstr(sec, tmp_buf);
		if (!sec) {
			pr_err("'ppio-port-tc-%d' not found\n", i);
			kfree(lbuff);
			return -EINVAL;
		}

		/* get the TC specific parameters */
		json_buffer_to_input(sec, "first_log_rxq", port->tc[i].first_log_rxq);
		json_buffer_to_input(sec, "num_in_qs", port->tc[i].tc_config.num_in_qs);
		if (!port->tc[i].tc_config.num_in_qs || port->tc[i].tc_config.num_in_qs > PP2_PPIO_MAX_NUM_INQS) {
			pr_err("invalid num_in_qs\n");
			kfree(lbuff);
			return -EINVAL;
		}

		for (j = 0; j < port->tc[i].tc_config.num_in_qs; j++) {
			json_buffer_to_input(sec, "ring_size", port->tc[i].rx_qs[j].ring_size);
			json_buffer_to_input(sec, "tc_pools_mem_id_index", port->tc[i].rx_qs[j].tc_pools_mem_id_index);
			if (port->tc[i].rx_qs[j].tc_pools_mem_id_index < 0 ||
			    port->tc[i].rx_qs[j].tc_pools_mem_id_index >= MV_SYS_DMA_MAX_NUM_MEM_ID) {
				pr_err("invalid tc_pools_mem_id_index\n");
				kfree(lbuff);
				return -EINVAL;
			}
		}

		json_buffer_to_input(sec, "pkt_offset", port->tc[i].tc_config.pkt_offset);
		json_buffer_to_input(sec, "first_rxq", port->tc[i].tc_config.first_rxq);
		json_buffer_to_input(sec, "num_bpools", num_bps);

		memset(param_pools, 0, sizeof(param_pools));
		for (j = 0; j < num_bps; j++) {
			/* get the bpool parameters */
			json_buffer_to_input(sec, "mem_id", mem_id);
			if (mem_id < 0 || mem_id >= MV_SYS_DMA_MAX_NUM_MEM_ID) {
				pr_err("invalid mem_id\n");
				kfree(lbuff);
				return -EINVAL;
			}

			json_buffer_to_input(sec, "cluster_id", cluster_id);
			if (cluster_id < 0 || cluster_id >= PP2_PPIO_TC_CLUSTER_MAX_POOLS) {
				pr_err("invalid cluster_id\n");
				kfree(lbuff);
				return -EINVAL;
			}

			json_buffer_to_input(sec, "pp2_id", tmp_pp2_id);
			json_buffer_to_input(sec, "bm_pool_id", tmp_bpool_id);

			param_pools[mem_id][cluster_id] = kcalloc(1, sizeof(struct pp2_bpool), GFP_KERNEL);
			if (unlikely(!port->txqs)) {
				pr_err("%s out of memory txqs alloc\n", __func__);
				num_bps = j;
				rc = -ENOMEM;
				goto probe_tc_error;
			}
			memset(param_pools[mem_id][cluster_id], 0, sizeof(struct pp2_bpool));
			param_pools[mem_id][cluster_id]->pp2_id = tmp_pp2_id;
			param_pools[mem_id][cluster_id]->id = tmp_bpool_id;
		}

		if (populate_tc_pools(inst, param_pools, port->tc[i].tc_config.pools) != 0) {
			pr_err("failed to populate bpool!\n");
			rc = -EFAULT;
			goto probe_tc_error;
		}
		for (j = 0; j < MV_SYS_DMA_MAX_NUM_MEM_ID; j++) {
			for (k = 0; k < PP2_PPIO_TC_CLUSTER_MAX_POOLS; k++)
				kfree(param_pools[j][k]);
		}
	}
	kfree(lbuff);
	return 0;

probe_tc_error:
	kfree(lbuff);
	for (j = 0; j < MV_SYS_DMA_MAX_NUM_MEM_ID; j++) {
		for (k = 0; k < PP2_PPIO_TC_CLUSTER_MAX_POOLS; k++)
			kfree(param_pools[j][k]);
	}
	return rc;
}

static int pp2_ppio_probe_rxq_params(char *buff, struct pp2_port *port, phys_addr_t paddr, uintptr_t va)
{
	int		 i;
	phys_addr_t	 poffset = 0;
	char		 tmp_buf[PP2_MAX_BUF_STR_LEN];
	char		*lbuff;
	char		*sec = NULL;

	lbuff = kcalloc(1, strlen(buff), GFP_KERNEL);
	if (lbuff == NULL)
		return -ENOMEM;

	memcpy(lbuff, buff, strlen(buff));
	sec = lbuff;

	for (i = 0; i < port->num_rx_queues; i++) {
		/* Search for the ppio-port-rxq section */
		memset(tmp_buf, 0, sizeof(tmp_buf));
		snprintf(tmp_buf, sizeof(tmp_buf), "ppio-port-rxq-%d", i);
		sec = strstr(sec, tmp_buf);
		if (!sec) {
			pr_err("'ppio-port-rxq-%d' not found\n", i);
			kfree(lbuff);
			return -EINVAL;
		}

		/* get the rxq parameters */
		json_buffer_to_input(sec, "id", port->rxqs[i]->id);
		json_buffer_to_input(sec, "log_id", port->rxqs[i]->log_id);
		json_buffer_to_input(sec, "desc_total", port->rxqs[i]->desc_total);
		json_buffer_to_input(sec, "rxq_lock", port->rxqs[i]->rxq_lock);
		json_buffer_to_input(sec, "threshold_rx_pkts", port->rxqs[i]->threshold_rx_pkts);
		json_buffer_to_input(sec, "desc_phys_offset", poffset);

		port->rxqs[i]->desc_phys_arr = paddr + poffset;
		port->rxqs[i]->desc_virt_arr = (struct pp2_desc *)((phys_addr_t)va + poffset);
	}
	kfree(lbuff);
	return 0;
}

static int pp2_ppio_probe_txq_params(char *buff, struct pp2_port *port, phys_addr_t paddr, uintptr_t va)
{
	int		 i;
	phys_addr_t	 poffset = 0;
	char		 tmp_buf[PP2_MAX_BUF_STR_LEN];
	char		*lbuff;
	char		*sec = NULL;

	lbuff = kcalloc(1, strlen(buff), GFP_KERNEL);
	if (lbuff == NULL)
		return -ENOMEM;

	memcpy(lbuff, buff, strlen(buff));
	sec = lbuff;

	for (i = 0; i < port->num_tx_queues; i++) {
		/* Search for the ppio-port-txq_config section */
		memset(tmp_buf, 0, sizeof(tmp_buf));
		snprintf(tmp_buf, sizeof(tmp_buf), "ppio-port-txq_config-%d", i);
		sec = strstr(sec, tmp_buf);
		if (!sec) {
			pr_err("'ppio-port-txq_config-%d' not found\n", i);
			kfree(lbuff);
			return -EINVAL;
		}

		/* get the txq_config parameters */
		json_buffer_to_input(sec, "size", port->txq_config[i].size);
		json_buffer_to_input(sec, "weight", port->txq_config[i].weight);
	}

	for (i = 0; i < port->num_tx_queues; i++) {
		/* Search for the ppio-port-txq section */
		memset(tmp_buf, 0, sizeof(tmp_buf));
		snprintf(tmp_buf, sizeof(tmp_buf), "ppio-port-txq-%d", i);
		sec = strstr(sec, tmp_buf);
		if (!sec) {
			pr_err("'port-info' not found\n");
			kfree(lbuff);
			return -EINVAL;
		}

		/* get the txq parameters */
		json_buffer_to_input(sec, "id", port->txqs[i]->id);
		json_buffer_to_input(sec, "log_id", port->txqs[i]->log_id);
		json_buffer_to_input(sec, "desc_total", port->txqs[i]->desc_total);
		json_buffer_to_input(sec, "threshold_tx_pkts", port->txqs[i]->threshold_tx_pkts);
		json_buffer_to_input(sec, "desc_phys_offset", poffset);

		port->txqs[i]->desc_phys_arr = paddr + poffset;
		port->txqs[i]->desc_virt_arr = (struct pp2_desc *)((phys_addr_t)va + poffset);
	}
	kfree(lbuff);
	return 0;
}

static int pp2_ppio_probe_loopback_port(char *buff, struct pp2_ppio *ppio, phys_addr_t paddr, uintptr_t va)
{
	int			i;
	phys_addr_t		poffset = 0;
	struct pp2_port		*lb_port = NULL;
	struct pp2_inst		*inst;
	char			tmp_buf[PP2_MAX_BUF_STR_LEN];
	char			*lbuff;
	char			*sec = NULL;

	lbuff = kcalloc(1, strlen(buff), GFP_KERNEL);
	if (lbuff == NULL)
		return -ENOMEM;

	memcpy(lbuff, buff, strlen(buff));
	sec = lbuff;

	inst = pp2_ptr->pp2_inst[ppio->pp2_id];

	/* extract the loopback info */
	lb_port = pp2_ptr->pp2_inst[ppio->pp2_id]->ports[PP2_LOOPBACK_PORT];

	memset(lb_port, 0, sizeof(struct pp2_port));
	lb_port->parent = inst;

	sec = strstr(sec, "port-lb-info");
	if (!sec) {
		pr_err("'port-loopback-info' not found\n");
		kfree(lbuff);
		return -EINVAL;
	}

	json_buffer_to_input(sec, "id", lb_port->id);
	json_buffer_to_input(sec, "type", lb_port->type);
	if (lb_port->type < PP2_PPIO_T_LOG || lb_port->type > PP2_PPIO_T_NIC) {
		pr_err("invalid port type\n");
		kfree(lbuff);
		return -EINVAL;
	}

	json_buffer_to_input(sec, "num_tx_queues", lb_port->num_tx_queues);
	if (!lb_port->num_tx_queues || lb_port->num_tx_queues > PP2_PPIO_MAX_NUM_OUTQS) {
		pr_err("invalid num_tx_queues\n");
		kfree(lbuff);
		return -EINVAL;
	}

	/* Allocate TXQ slots for this port */
	lb_port->txqs = kcalloc(1, sizeof(struct pp2_tx_queue *) * lb_port->num_tx_queues, GFP_KERNEL);
	if (unlikely(!lb_port->txqs)) {
		pr_err("%s out of memory txqs alloc\n", __func__);
		kfree(lbuff);
		return -ENOMEM;
	}
	memset(lb_port->txqs, 0, sizeof(struct pp2_tx_queue *) * lb_port->num_tx_queues);

	/* Allocate and associated TXQs to this port */
	pp2_port_txqs_create(lb_port);

	for (i = 0; i < lb_port->num_tx_queues; i++) {
		memset(tmp_buf, 0, sizeof(tmp_buf));
		snprintf(tmp_buf, sizeof(tmp_buf), "lb-port-txq_config-%d", i);
		sec = strstr(sec, tmp_buf);
		if (!sec) {
			pr_err("'port-info' not found\n");
			kfree(lbuff);
			return -EINVAL;
		}
		json_buffer_to_input(sec, "size", lb_port->txq_config[i].size);
		json_buffer_to_input(sec, "weight", lb_port->txq_config[i].weight);
	}
	for (i = 0; i < lb_port->num_tx_queues; i++) {
		memset(tmp_buf, 0, sizeof(tmp_buf));
		snprintf(tmp_buf, sizeof(tmp_buf), "lb-port-txq-%d", i);
		sec = strstr(sec, tmp_buf);
		if (!sec) {
			pr_err("'port-info' not found\n");
			kfree(lbuff);
			return -EINVAL;
		}
		json_buffer_to_input(sec, "id", lb_port->txqs[i]->id);
		json_buffer_to_input(sec, "log_id", lb_port->txqs[i]->log_id);
		json_buffer_to_input(sec, "desc_total", lb_port->txqs[i]->desc_total);
		json_buffer_to_input(sec, "threshold_tx_pkts", lb_port->txqs[i]->threshold_tx_pkts);
		json_buffer_to_input(sec, "desc_phys_offset", poffset);

		lb_port->txqs[i]->desc_phys_arr = paddr + poffset;
		lb_port->txqs[i]->desc_virt_arr = (struct pp2_desc *)((phys_addr_t)va + poffset);
	}
	kfree(lbuff);
	return 0;
}

int pp2_ppio_probe(char *match, char *buff, struct pp2_ppio **ppio_hdl)
{
	int				 rc;
	char				*sec = NULL;
	phys_addr_t			 paddr;
	struct sys_iomem_params		 iomem_params;
	struct sys_iomem_info		sys_iomem_info;
	char				 dev_name[PP2_MAX_BUF_STR_LEN];
	uintptr_t			 va;
	u8				 id_match[2];
	int				 port_id = 0, pp2_id = 0;
	struct pp2_port			**port_hdl;
	struct pp2_port			*port;
	struct pp2_inst			*inst;
	struct pp2_ppio			*ppio = NULL;
	char				*lbuff;

	lbuff = kcalloc(1, strlen(buff), GFP_KERNEL);
	if (lbuff == NULL)
		return -ENOMEM;

	memcpy(lbuff, buff, strlen(buff));
	sec = lbuff;

	/* Search for the ppio-info section */
	sec = strstr(sec, "ppio-info");
	if (!sec) {
		pr_err("ppio-info not found\n");
		rc = -EINVAL;
		goto ppio_probe_exit;
	}

	/* Search for match (ppio-x:x) */
	sec = strstr(sec, match);
	if (!sec) {
		pr_err("match not found %s %s\n", match, sec);
		rc = -EINVAL;
		goto ppio_probe_exit;
	}

	memset(dev_name, 0, FILE_MAX_LINE_CHARS);
	json_buffer_to_input_str(sec, "dma_dev_name", dev_name);
	if (dev_name[0] == 0) {
		pr_err("'dma_dev_name' not found\n");
		rc = -EINVAL;
		goto ppio_probe_exit;
	}

	iomem_params.type = SYS_IOMEM_T_SHMEM;
	iomem_params.devname = dev_name;
	iomem_params.index = 1;

	if (sys_iomem_get_info(&iomem_params, &sys_iomem_info)) {
		pr_err("sys_iomem_get_info error\n");
		rc = -EFAULT;
		goto ppio_probe_exit;
	}

	va = (uintptr_t)sys_iomem_info.u.shmem.va;
	paddr = sys_iomem_info.u.shmem.paddr;

	/* Retireve pp2_id and pool_id */
	json_buffer_to_input(sec, "pp2_id", pp2_id);
	json_buffer_to_input(sec, "id", port_id);

	if (mv_sys_match(match, "ppio", 2, id_match)) {
		pr_err("[%s] Invalid match string!\n", __func__);
		rc = -ENXIO;
		goto ppio_probe_exit;
	}

	if (pp2_is_init() == false) {
		rc = -EPERM;
		goto ppio_probe_exit;
	}

	if (pp2_id != id_match[0]) {
		pr_err("[%s] wrong pp2_id: found %d, requested %d\n", __func__, pp2_id, id_match[0]);
		rc = -ENXIO;
		goto ppio_probe_exit;
	}

	if (port_id != id_match[1]) {
		pr_err("[%s] wrong pool_id: found %d, requested %d\n", __func__, port_id,  id_match[1]);
		rc = -ENXIO;
		goto ppio_probe_exit;
	}

	if (port_id >= PP2_NUM_ETH_PPIO) {
		pr_err("[%s] Invalid ppio.\n", __func__);
		rc = -ENXIO;
		goto ppio_probe_exit;
	}
	if (pp2_id >= pp2_ptr->num_pp2_inst) {
		pr_err("[%s] Invalid pp2 instance.\n", __func__);
		rc = -ENXIO;
		goto ppio_probe_exit;
	}

	if (!pp2_ppio_available(pp2_id, port_id)) {
		pr_err("[%s] %s is not available for musdk.\n", __func__, match);
		rc = -EINVAL;
		goto ppio_probe_exit;
	}

	if (pp2_ptr->pp2_inst[pp2_id]->ppios[port_id]) {
		pr_err("[%s] ppio already exists.\n", __func__);
		rc = -EEXIST;
		goto ppio_probe_exit;
	}

	ppio = kcalloc(1, sizeof(struct pp2_ppio), GFP_KERNEL);
	if (!ppio) {
		pr_err("%s no memory for ppio alloc\n", __func__);
		rc = -ENOMEM;
		goto ppio_probe_exit;
	}

	ppio->pp2_id = pp2_id;
	ppio->port_id = port_id;

	port_hdl = GET_PPIO_PORT_PTR(*ppio);

	inst = pp2_ptr->pp2_inst[ppio->pp2_id];
	port = inst->ports[ppio->port_id];
	memset(port, 0, sizeof(struct pp2_port));
	port->parent = inst;
	*port_hdl = port;

	/* Search for the port-info section */
	sec = strstr(sec, "port-info");
	if (!sec) {
		pr_err("'port-info' not found\n");
		rc = -EINVAL;
		goto ppio_probe_exit;
	}

	/* get the port-info parameters */
	json_buffer_to_input(sec, "id", port->id);
	json_buffer_to_input(sec, "flags", port->flags);
	json_buffer_to_input(sec, "port_mru", port->port_mru);
	json_buffer_to_input(sec, "port_mtu", port->port_mtu);
	json_buffer_to_input(sec, "first_rxq", port->first_rxq);
	json_buffer_to_input(sec, "use_mac_lb", port->use_mac_lb);
	json_buffer_to_input(sec, "t_mode", port->t_mode);
	json_buffer_to_input(sec, "hash_type", port->hash_type);
	if (port->hash_type < PP2_PPIO_HASH_T_NONE || port->hash_type >= PP2_PPIO_HASH_T_OUT_OF_RANGE) {
		pr_err("invalid hash_type\n");
		rc = -EINVAL;
		goto ppio_probe_exit;
	}

	json_buffer_to_input(sec, "rss_en", port->rss_en);
	if (port->rss_en < 0 || port->rss_en > 1) {
		pr_err("invalid rss_en\n");
		rc = -EINVAL;
		goto ppio_probe_exit;
	}

	json_buffer_to_input(sec, "tx_fifo_size", port->tx_fifo_size);
	json_buffer_to_input(sec, "maintain_stats", port->maintain_stats);
	if (port->maintain_stats < 0 || port->maintain_stats > 1) {
		pr_err("invalid maintain_stats\n");
		rc = -EINVAL;
		goto ppio_probe_exit;
	}

	json_buffer_to_input(sec, "type", port->type);
	if (port->type < PP2_PPIO_T_LOG || port->type > PP2_PPIO_T_NIC) {
		pr_err("invalid port type\n");
		rc = -EINVAL;
		goto ppio_probe_exit;
	}

	json_buffer_to_input_str(sec, "linux_name", port->linux_name);
	if (!port->linux_name) {
		pr_err("'linux_name' not found\n");
		rc = -EINVAL;
		goto ppio_probe_exit;
	}

	/* Assing a CPU slot */
	port->cpu_slot = inst->hw.base[PP2_DEFAULT_REGSPACE].va;

	/* Search for the ppio-port-mac_data section */
	sec = strstr(sec, "ppio-port-mac_data");
	if (!sec) {
		pr_err("'port-info' not found\n");
		rc = -EINVAL;
		goto ppio_probe_exit;
	}

	/* get the ppio-port-mac_data parameters */
	json_buffer_to_input(sec, "gop_index", port->mac_data.gop_index);
	json_buffer_to_input(sec, "flags", port->mac_data.flags);
	json_buffer_to_input(sec, "phy_addr", port->mac_data.phy_addr);
	json_buffer_to_input(sec, "phy_mode", port->mac_data.phy_mode);
	json_buffer_to_input(sec, "force_link", port->mac_data.force_link);
	json_buffer_to_input(sec, "autoneg", port->mac_data.autoneg);
	json_buffer_to_input(sec, "link", port->mac_data.link);
	json_buffer_to_input(sec, "duplex", port->mac_data.duplex);
	json_buffer_to_input(sec, "speed", port->mac_data.speed);
	json_buffer_to_input_mac(sec, "mac_address", port->mac_data.mac);
	if (!port->mac_data.mac) {
		pr_err("'mac_data.mac' not found\n");
		rc = -EINVAL;
		goto ppio_probe_exit;
	}

	/* get the number of TC's */
	json_buffer_to_input(sec, "num_tcs", port->num_tcs);
	if (!port->num_tcs || port->num_tcs > PP2_PPIO_MAX_NUM_TCS) {
		pr_err("invalid num_tcs\n");
		rc = -EINVAL;
		goto ppio_probe_exit;
	}

	/* get the number of rx queues */
	json_buffer_to_input(sec, "num_rx_queues", port->num_rx_queues);
	if (!port->num_rx_queues || port->num_rx_queues > PP2_PPIO_MAX_NUM_INQS) {
		pr_err("invalid num_rx_queues\n");
		rc = -EINVAL;
		goto ppio_probe_exit;
	}

	/* Allocate RXQ slots for this port */
	port->rxqs = kcalloc(1, sizeof(struct pp2_rx_queue *) * port->num_rx_queues, GFP_KERNEL);
	if (unlikely(!port->rxqs)) {
		pr_err("%s out of memory rxqs alloc\n", __func__);
		rc = -ENOMEM;
		goto ppio_probe_exit;
	}
	memset(port->rxqs, 0, sizeof(struct pp2_rx_queue *) * port->num_rx_queues);

	/* get the number of tx queues */
	json_buffer_to_input(sec, "num_tx_queues", port->num_tx_queues);
	if (!port->num_tx_queues || port->num_tx_queues > PP2_PPIO_MAX_NUM_OUTQS) {
		pr_err("invalid num_tx_queues\n");
		rc = -EINVAL;
		goto ppio_probe_exit;
	}

	/* Allocate TXQ slots for this port */
	port->txqs = kcalloc(1, sizeof(struct pp2_tx_queue *) * port->num_tx_queues, GFP_KERNEL);
	if (unlikely(!port->txqs)) {
		pr_err("%s out of memory txqs alloc\n", __func__);
		rc = -ENOMEM;
		goto ppio_probe_exit;
	}
	memset(port->txqs, 0, sizeof(struct pp2_tx_queue *) * port->num_tx_queues);

	rc = pp2_ppio_probe_tc_params(sec, ppio);
	if (rc)
		goto ppio_probe_exit;

	/* Allocate and associate RXQs to this port */
	pp2_port_rxqs_create(port);
	/* Allocate and associate TXQs to this port */
	pp2_port_txqs_create(port);

	rc = pp2_ppio_probe_rxq_params(sec, port, paddr, va);
	if (rc)
		goto ppio_probe_exit;

	rc = pp2_ppio_probe_txq_params(sec, port, paddr, va);
	if (rc)
		goto ppio_probe_exit;

	pp2_port_initialize_statistics(port);
	pp2_ppio_get_statistics(ppio, NULL, true);

	/* extract the loopback info */
	rc = pp2_ppio_probe_loopback_port(sec, ppio, paddr, va);
	if (rc)
		goto ppio_probe_exit;

	*ppio_hdl = ppio;
	pp2_ptr->pp2_inst[pp2_id]->ppios[port_id] = ppio;

	rc = 0;
ppio_probe_exit:
	kfree(lbuff);
	if (rc)
		kfree(ppio);
	return rc;

}

int pp2_ppio_remove(struct pp2_ppio *ppio)
{
	u32		 qid;
	struct pp2_port *port = NULL;
	struct pp2_port *lb_port = NULL;

	if (!ppio)
		return 0;

	port = GET_PPIO_PORT(ppio);

	if (!port) {
		pr_info("%s port is null\n", __func__);
		return -EINVAL;
	}

	for (qid = 0; qid < port->num_tx_queues; qid++) {
		struct pp2_tx_queue *txq = port->txqs[qid];

		kfree(txq);
	}

	for (qid = 0; qid < port->num_rx_queues; qid++) {
		struct pp2_rx_queue *rxq = port->rxqs[qid];

		kfree(rxq);
	}

	kfree(port->rxqs);
	kfree(port->txqs);

	pp2_ptr->pp2_inst[ppio->pp2_id]->ppios[ppio->port_id] = NULL;

	/* release tx queues for loopback port associated to this ppio*/
	lb_port = pp2_ptr->pp2_inst[ppio->pp2_id]->ports[PP2_LOOPBACK_PORT];

	for (qid = 0; qid < lb_port->num_tx_queues; qid++) {
		struct pp2_tx_queue *txq = lb_port->txqs[qid];

		kfree(txq);
	}

	kfree(lb_port->txqs);

	kfree(ppio);

	return 0;
}

int pp2_ppio_rx_create_event(struct pp2_ppio *ppio, struct pp2_ppio_rxq_event_params *params, struct mv_sys_event **ev)
{
	struct pp2_port *port = NULL;
	int err;

	if (!ppio || !params || !ev)
		return -EINVAL;

	port = GET_PPIO_PORT(ppio);
	if (!port) {
		pr_info("%s port is null\n", __func__);
		return -EINVAL;
	}

	err = pp2_port_rx_create_event(port, params, ev);

	return err;
}
int pp2_ppio_rx_set_event(struct mv_sys_event *ev, int en)
{
	int err;

	err = pp2_port_rx_set_event(ev, en);

	return err;
}

int pp2_ppio_rx_delete_event(struct mv_sys_event *ev)
{
	int err;

	err = pp2_port_rx_delete_event(ev);

	return err;
}


