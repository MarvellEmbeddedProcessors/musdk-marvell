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
#include "drivers/mv_neta.h"
#include "drivers/mv_neta_ppio.h"
#include "neta_ppio.h"
#include "neta_hw.h"
#include "neta_bm.h"
#include "lib/lib_misc.h"

static struct neta_port port_array[NETA_NUM_ETH_PPIO];

static int neta_txq_done(struct neta_port *pp, struct neta_tx_queue *txq);

/**
 * Initialize a ppio
 *
 * params	A pointer to structure that contains all relevant parameters.
 * ppio		A pointer to opaque ppio handle of type 'struct neta_ppio *'.
 *
 * retval	0 on success
 * retval	<0 on failure
 */
int neta_ppio_init(struct neta_ppio_params *params, struct neta_ppio **ppio)
{
	int port_id, rc;
	struct neta_ppio *ppio_ptr;
	struct neta_port *port;

	port_id = atoi(&params->match[3]);
	if (port_id >= NETA_NUM_ETH_PPIO) {
		pr_err("[%s] Invalid ppio number %d.\n", __func__, port_id);
		return -ENXIO;
	}

	rc = neta_port_register(params->match, port_id);
	if (rc) {
		pr_err("PP-IO init failed: interface %s doesn't exist or down!\n", params->match);
		return rc;
	}

	ppio_ptr = kcalloc(1, sizeof(struct neta_ppio), GFP_KERNEL);
	if (unlikely(!ppio_ptr)) {
		pr_err("%s: out of memory for NETA driver allocation\n", __func__);
		return -ENOMEM;
	}

	if (params->inqs_params.tcs_params[0].pkt_offset > 120) {
		pr_err("[%s] Cannot support packet offset > %d.\n",
		       __func__, params->inqs_params.tcs_params[0].pkt_offset);
		return -EINVAL;
	}
	port = &port_array[port_id];
	port->txq_number = params->outqs_params.num_outqs;
	port->rxq_number = params->inqs_params.num_tcs;
	/* all queues have the same size */
	port->rx_ring_size = params->inqs_params.tcs_params[0].size;
	port->tx_ring_size = params->outqs_params.outqs_params[0].size;
	port->rx_offset = params->inqs_params.tcs_params[0].pkt_offset;

	port->mtu = params->inqs_params.mtu;
	port->mru = MVNETA_MTU_TO_MRU(port->mtu);
	port->buf_size = port->mru + port->rx_offset;
	port->id = port_id;

	rc = neta_port_open(port_id, port);
	if (rc) {
		pr_err("[%s] ppio init failed.\n", __func__);
		return(-EFAULT);
	}
	/* build interface name */
	strcpy(port->if_name, params->match);
	printf("init %s interface\n", port->if_name);

	neta_port_initialize_statistics(port);
	neta_port_hw_init(port);
	ppio_ptr->port_id = port_id;
	ppio_ptr->internal_param = port;
	*ppio = ppio_ptr;

	return rc;
}

/**
 * Destroy a ppio
 *
 * ppio		A ppio handle.
 *
 */
void neta_ppio_deinit(struct neta_ppio *ppio)
{
	neta_port_hw_deinit(ppio->internal_param);
	neta_port_unregister(ppio->port_id);
	free(ppio);
}

/**
 * TODO - Add a Marvell DSA Tag to the packet.
 *
 * desc		A pointer to a packet descriptor structure.
 *
 */
void neta_ppio_outq_desc_set_dsa_tag(struct neta_ppio_desc *desc)
{
}

/******** RXQ  ********/

/**
 * TODO - Check if there is IPV4 fragmentation in an inq packet descriptor.
 *
 * desc		A pointer to a packet descriptor structure.
 *
 * retval	0 - not fragmented, 1 - fragmented.
 */
int neta_ppio_inq_desc_get_ip_isfrag(struct neta_ppio_desc *desc)
{
	return 0;
}

/**
 * Get number of packets sent on a queue, since last call of this API.
 *
 * ppio		A pointer to a PP-IO object.
 * qid		out-Q id on which to send the frames.
 * num		Number of frames that were sent.
 *
 * retval	0 on success
 * retval	error-code otherwise
 */
int neta_ppio_get_num_outq_done(struct neta_ppio	*ppio,
				u8			qid,
				u16			*num)
{
	struct neta_port *port = GET_PPIO_PORT(ppio);
	struct neta_tx_queue *txq = &port->txqs[qid];
	int tx_done;

	/* Get number of sent descriptors */
	tx_done = mvneta_txq_sent_desc_num_get(port, qid);

	/* Decrement sent descriptors counter */
	if (tx_done)
		mvneta_txq_sent_desc_dec(port, qid, tx_done);
	txq->count -= tx_done;

	*num = tx_done;

#ifdef NETA_STATS_SUPPORT
	txq->tx_done_pkts += tx_done;
#endif
	return 0;
}

/* Get pointer to the next RX descriptor to be processed by SW, and update the descriptor next index */
static struct neta_ppio_desc *neta_rxq_get_desc(struct neta_rx_queue *rxq)
{
	u32 rx_idx;
	struct neta_ppio_desc *rx_desc;

	rx_idx = rxq->next_desc_to_proc;
	rx_desc = rxq->descs + rx_idx;

	if (rx_desc->cmds[1] == MVNETA_DESC_WATERMARK) {
		pr_debug("Bad RX descriptor %d: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\n",
			 rx_idx, rx_desc->cmds[0], rx_desc->cmds[1],
			 rx_desc->cmds[2], rx_desc->cmds[3], rx_desc->cmds[4]);
		/* will read descriptor next time */
		rmb();
		return NULL;
	}

	if ((rx_desc->cmds[0] & NETA_RXD_FIRST_LAST_DESC_MASK) !=
	     NETA_RXD_FIRST_LAST_DESC_MASK)
		/* multi buffers on rx not supported */
		/* mark multi descriptors with error */
		rx_desc->cmds[0] |= (1 << NETA_RXD_ERROR_SUM_OFF);

	rxq->next_desc_to_proc = (((rx_idx + 1) == rxq->size) ? 0 : (rx_idx + 1));

	return (rxq->descs + rx_idx);
}

static inline void neta_ppio_desc_swap_ncopy(struct neta_ppio_desc *dst, struct neta_ppio_desc *src)
{
	u32 *src_cmd = (u32 *)src;
	u32 *dst_cmd = (u32 *)dst;

	for (int i = 0; i < NETA_PPIO_DESC_NUM_WORDS; i++) {
		*dst_cmd = le32toh(*src_cmd);
		dst_cmd++;
		src_cmd++;
	}
}

/**
 * Receive packets on a ppio.
 *
 * ppio		A pointer to a PP-IO object.
 * tc		traffic class on which to receive frames
 * qid		out-Q id on which to receive the frames.
 * descs	A pointer to an array of descriptors represents the received frames.
 * num		input: Max number of frames to receive; output: number of frames received.
 *
 * retval	0 on success
 * retval	error-code otherwise
 */
int neta_ppio_recv(struct neta_ppio		*ppio,
		   u8				qid,
		   struct neta_ppio_desc	*descs,
		   u16				*num)
{
	struct neta_port *port = GET_PPIO_PORT(ppio);
	struct neta_ppio_desc *rx_desc;
	struct neta_rx_queue *rxq;
	u32 recv_req = *num;
	int i;

	rxq = &port->rxqs[qid];

	if (recv_req > rxq->desc_received) {
		rxq->desc_received = neta_rxq_busy_desc_num_get(port, rxq->id);
		if (unlikely(recv_req > rxq->desc_received)) {
			recv_req = rxq->desc_received;
		}
	}

	if (!rxq->desc_received) {
		*num = 0;
		return 0;
	}

	pr_debug("%s: receive %d (%d) packets.\n", __func__, rxq->desc_received, recv_req);

	for (i = 0; i < recv_req; i++) {
		rx_desc = neta_rxq_get_desc(rxq);
		if (!rx_desc)
			break;

		memcpy(&descs[i], rx_desc, sizeof(struct neta_ppio_desc));
		/* invalidate packet descriptor */
		rx_desc->cmds[1] = MVNETA_DESC_WATERMARK;
	}
	rxq->to_refill_cntr += i;

	/* update HW with rx_done descriptors */
	neta_port_inq_update(port, rxq, i, 0);
	rxq->desc_received -= i;
	*num = i;

#ifdef NETA_STATS_SUPPORT
	rxq->rx_pkts += recv_req;
#endif
	return 0;
}

/* Tx descriptors helper methods */
/* Get number of free TX descriptor */
static inline int neta_txq_free_desc_num(struct neta_tx_queue *txq)
{
	return (txq->size - txq->count);
}
/* Get pointer to next TX descriptor to be processed (send) by HW */
static struct neta_ppio_desc *neta_txq_next_desc_get(struct neta_tx_queue *txq)
{
	int tx_desc = txq->next_desc_to_proc;

	txq->next_desc_to_proc = MVNETA_QUEUE_NEXT_DESC(txq, tx_desc);
	return txq->descs + tx_desc;
}

/* Release the last allocated TX descriptor. Useful to handle DMA
 * mapping failures in the TX path.
 */
static inline void neta_txq_desc_put(struct neta_tx_queue *txq)
{
	if (txq->next_desc_to_proc == 0)
		txq->next_desc_to_proc = txq->last_desc - 1;
	else
		txq->next_desc_to_proc--;
}

/* Handle end of transmission */
static int neta_txq_done(struct neta_port *pp,
			  struct neta_tx_queue *txq)
{
	int tx_done;

	/* Get number of sent descriptors */
	tx_done = mvneta_txq_sent_desc_num_get(pp, txq->id);

	/* Decrement sent descriptors counter */
	if (tx_done)
		mvneta_txq_sent_desc_dec(pp, txq->id, tx_done);

	txq->count -= tx_done;

	return (txq->size - txq->count);
}

static int neta_port_enqueue(struct neta_port *port, u8 txq_id, struct neta_ppio_desc *descs, u16 num)
{
	struct neta_tx_queue *txq = &port->txqs[txq_id];
	struct neta_ppio_desc *tx_desc;
	int i;
	int free_desc = neta_txq_free_desc_num(txq);

	if (free_desc < num)
		num = (free_desc < num) ? free_desc : num;

	for (i = 0; i < num; i++) {
		/* Get a descriptor for the packet */
		tx_desc = neta_txq_next_desc_get(txq);
#if __BYTE_ORDER == __BIG_ENDIAN
		neta_ppio_desc_swap_ncopy(&tx_desc, &descs[i]);
#else
		memcpy(tx_desc, &descs[i], sizeof(struct neta_ppio_desc));
#endif
	}
	/* be sure TX descriptors are ready to transmit */
	wmb();
	neta_txq_pend_desc_add(port, txq, i);

	txq->count += i;

	return i;
}

/*
 * Send a batch of frames (single dscriptor) on an OutQ of PP-IO.
 *
 * ppio		A pointer to a PP-IO object.
 * qid		out-Q id on which to send the frames.
 * descs	A pointer to an array of descriptors representing the frames to be sent.
 * num		input: number of frames to be sent; output: number of frames sent.
 *
 * retval	0 on success
 * retval	error-code otherwise
 */
int neta_ppio_send(struct neta_ppio	*ppio,
		  u8			 qid,
		  struct neta_ppio_desc	*descs,
		  u16			*num)
{
	u16 desc_sent, desc_req = *num;
	struct neta_port *port = GET_PPIO_PORT(ppio);

	desc_sent = neta_port_enqueue(port, qid, descs, desc_req);
	if (unlikely(desc_sent < desc_req)) {
		pr_debug("%s: Port %u qid %u, send_request %u sent %u!\n", __func__,
			 ppio->port_id, qid, *num, desc_sent);
		*num = desc_sent;
	}

#ifdef NETA_STATS_SUPPORT
	{
		struct neta_tx_queue *txq;

		txq = &port->txqs[qid];
		txq->tx_pkts += desc_sent;
	}
#endif /* NETA_STATS_SUPPORT */
	return 0;
}

int neta_ppio_enable(struct neta_ppio *ppio)
{
	struct neta_port *port = GET_PPIO_PORT(ppio);

	if (!port)
		return -1;

	neta_port_up(port);

	port->is_running = 1;

	return 0;
}

int neta_ppio_disable(struct neta_ppio *ppio)
{
	struct neta_port *port = GET_PPIO_PORT(ppio);

	if (!port)
		return -1;

	port->is_running = 0;

	neta_port_down(port);
	return 0;
}

static inline int neta_ppio_inq_put_buff(struct neta_port *pp,
				    struct neta_rx_queue *rxq,
				    struct neta_buff_inf *buf)
{
	struct neta_ppio_desc *rx_desc;

	if (unlikely(!buf->cookie || !buf->addr)) {
		pr_err("port %d: queue %d: try to refill with worng buffer: cookie(%lx), pa(%lx)!\n",
			pp->id, rxq->id, (u64)buf->cookie, (u64)buf->addr);
		return -1;
	}

	rx_desc = rxq->descs + rxq->next_desc_to_refill;

	if ((rxq->next_desc_to_refill + 1) == rxq->size)
		rxq->next_desc_to_refill = 0;
	else
		rxq->next_desc_to_refill++;

	rxq->to_refill_cntr--;

	rx_desc->cmds[4] = buf->cookie;
	rx_desc->cmds[2] = buf->addr;

	return 0;
}

/**
 * Fill RX descriptors ring with buffer pointers
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[in]		qid	in-Q id to fill.
 * @param[in]		bufs	A pointer to an array of buffers to put to descriptors.
 * @param[in,out]	num_of_buffs	input: number of buffers in array;
 *					output: number of buffers were put
 *					to descriptors.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int neta_ppio_inq_put_buffs(struct neta_ppio		*ppio,
			    u8				qid,
			    struct neta_buff_inf	*bufs,
			    u16				*num_of_buffs)
{
	struct neta_port *pp = GET_PPIO_PORT(ppio);
	struct neta_rx_queue *rxq;
	int refill_cnt = *num_of_buffs;
	int i;

	rxq = &pp->rxqs[qid];
	if (!rxq->to_refill_cntr)
		pr_warn("port %d: queue %d: queue is full, nothing to refill\n",
			pp->id, qid);

	if (refill_cnt > rxq->to_refill_cntr)
		refill_cnt = rxq->to_refill_cntr;

	for (i = 0; i < refill_cnt; i++) {
		if (neta_ppio_inq_put_buff(pp, rxq, &bufs[i]))
			break;
	}
	*num_of_buffs = i;
	/* flush updated descriptors to memory */
	wmb();

#ifdef NETA_STATS_SUPPORT
	rxq->refill_bufs += i;
#endif

	pr_debug("port %d: queue %d: refill %d buffers\n", pp->id, qid, i);
	neta_port_inq_update(pp, rxq, 0, i);
	return 0;
}

/**
 * Get all free buffers found in InQ.
 * Tis routine shal be used only to cleanup InQ.
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[in]		qid	in-Q id to get the buffer from.
 * @param[out]		bufs	A pointer to an array of buffers to free.
 * @param[in,out]	num_of_buffs	input: number of buffers in array
 *					output: number of buffers to free
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int neta_ppio_inq_get_all_buffs(struct neta_ppio	*ppio,
				u8			qid,
				struct neta_buff_inf	*bufs,
				u16			*num_of_buffs)
{
	struct neta_port *pp = GET_PPIO_PORT(ppio);
	struct neta_rx_queue *rxq;
	struct neta_ppio_desc *rx_desc;
	int i, free_cnt;

	rxq = &pp->rxqs[qid];
	free_cnt = *num_of_buffs;

	if (free_cnt > (rxq->size - rxq->to_refill_cntr))
		free_cnt = rxq->size - rxq->to_refill_cntr;

	for (i = 0; i < free_cnt; i++) {

		rx_desc = rxq->descs + rxq->next_desc_to_refill;
		rxq->to_refill_cntr++;

		if (rxq->next_desc_to_refill)
			rxq->next_desc_to_refill--;
		else
			rxq->next_desc_to_refill = rxq->last_desc;

		bufs[i].cookie = rx_desc->cmds[4];
		bufs[i].addr = rx_desc->cmds[2];
	}
	if (rxq->to_refill_cntr != rxq->size)
		pr_warn("[%s]: %d not enough to return all buffers. %d buffers doesn't free\n",
			__func__, *num_of_buffs, (rxq->size - rxq->to_refill_cntr));

	*num_of_buffs = i;
	return 0;
}

int neta_ppio_set_mac_addr(struct neta_ppio *ppio, const eth_addr_t addr)
{
	int rc;

	rc = neta_port_set_mac_addr(GET_PPIO_PORT(ppio), (const uint8_t *)addr);
	return rc;
}

int neta_ppio_get_mac_addr(struct neta_ppio *ppio, eth_addr_t addr)
{
	int rc;

	rc = neta_port_get_mac_addr(GET_PPIO_PORT(ppio), (uint8_t *)addr);
	return rc;
}

int neta_ppio_set_mtu(struct neta_ppio *ppio, u16 mtu)
{
	int rc;

	rc = neta_port_set_mtu(GET_PPIO_PORT(ppio), mtu);
	return rc;
}

int neta_ppio_get_mtu(struct neta_ppio *ppio, u16 *mtu)
{
	neta_port_get_mtu(GET_PPIO_PORT(ppio), mtu);
	return 0;
}

int neta_ppio_set_mru(struct neta_ppio *ppio, u16 len)
{
	int rc;

	rc = neta_port_set_mru(GET_PPIO_PORT(ppio), len);
	return rc;
}

int neta_ppio_get_mru(struct neta_ppio *ppio, u16 *len)
{
	neta_port_get_mru(GET_PPIO_PORT(ppio), len);

	return 0;
}

int neta_ppio_set_promisc(struct neta_ppio *ppio, int en)
{
	int rc;

	rc = neta_port_set_promisc(GET_PPIO_PORT(ppio), en);
	return rc;
}

int neta_ppio_get_promisc(struct neta_ppio *ppio, int *en)
{
	int rc;

	rc = neta_port_get_promisc(GET_PPIO_PORT(ppio), (u32 *)en);
	return rc;
}

int neta_ppio_set_mc_promisc(struct neta_ppio *ppio, int en)
{
	int rc;

	rc = neta_port_set_mc_promisc(GET_PPIO_PORT(ppio), en);
	return rc;
}

int neta_ppio_get_mc_promisc(struct neta_ppio *ppio, int *en)
{
	int rc;

	rc = neta_port_get_mc_promisc(GET_PPIO_PORT(ppio), (u32 *)en);
	return rc;
}

int neta_ppio_add_mac_addr(struct neta_ppio *ppio, const eth_addr_t addr)
{
	int rc;

	rc = neta_port_add_mac_addr(GET_PPIO_PORT(ppio), (const uint8_t *)addr);
	return rc;
}

int neta_ppio_remove_mac_addr(struct neta_ppio *ppio, const eth_addr_t addr)
{
	int rc;

	rc = neta_port_remove_mac_addr(GET_PPIO_PORT(ppio), (const uint8_t *)addr);
	return rc;
}

int neta_ppio_flush_mac_addrs(struct neta_ppio *ppio, int uc, int mc)
{
	int rc;

	if (uc)
		pr_warn("[%s]: cannot remove unicast address\n", __func__);

	rc = neta_port_flush_mac_addrs(GET_PPIO_PORT(ppio));
	return rc;
}

int neta_ppio_get_link_state(struct neta_ppio *ppio, int *en)
{
	int rc;

	rc = neta_port_get_link_state(GET_PPIO_PORT(ppio), en);
	return rc;
}

int neta_ppio_get_statistics(struct neta_ppio *ppio, struct neta_ppio_statistics *stats)
{
	struct neta_ppio_statistics cur_stats;
	struct neta_port *port = GET_PPIO_PORT(ppio);

	memset(&cur_stats, 0, sizeof(struct neta_ppio_statistics));
	neta_port_get_statistics(port, &cur_stats);

	if (stats)
		memcpy(stats, &cur_stats, sizeof(struct neta_ppio_statistics));

	return 0;

}
