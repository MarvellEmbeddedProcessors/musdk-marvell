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
#include "drivers/mv_neta_bpool.h"
#include "neta_ppio.h"
#include "neta_hw.h"
#include "lib/lib_misc.h"

static struct neta_port port_array[NETA_NUM_ETH_PPIO];

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
	u8  match[2];
	int port_id, rc;
	struct neta_ppio *ppio_ptr;
	struct neta_port *port;

	if (mv_sys_match(params->match, "neta", 1, match)) {
		pr_err("[%s] Invalid match string!\n", __func__);
		return -ENXIO;
	}

	ppio_ptr = kcalloc(1, sizeof(struct neta_ppio), GFP_KERNEL);
	if (unlikely(!ppio_ptr)) {
		pr_err("%s: out of memory for NETA driver allocation\n", __func__);
		return -ENOMEM;
	}
	/* check that application port is US port */
	port_id = match[0];

	if (port_id >= NETA_NUM_ETH_PPIO) {
		pr_err("[%s] Invalid ppio number %d.\n", __func__, port_id);
		return -ENXIO;
	}

	port = &port_array[port_id];
	port->txq_number = params->outqs_params.num_outqs;
	port->rxq_number = params->inqs_params.num_tcs;
	/* all queues have the same size */
	port->rx_ring_size = params->inqs_params.tcs_params[0].size;
	port->tx_ring_size = params->outqs_params.outqs_params[0].size;
	port->pool_short = (struct mvneta_bm_pool *)(params->inqs_params.pools[0]->internal_param);
	port->pool_long = (struct mvneta_bm_pool *)params->inqs_params.pools[1]->internal_param;

	rc = neta_port_open(port_id, port);
	if (rc) {
		pr_err("[%s] ppio init failed.\n", __func__);
		return(-EFAULT);
	}
	neta_bm_pool_bufsize_set(port, 256, 0);
	neta_bm_pool_bufsize_set(port, 1600, 1);

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
	free(ppio);
}

/**
 * Set the pool number to return the buffer to, after sending the packet.
 * Calling this API will cause PP HW to return the buffer to the PP Buffer Manager.
 *
 * desc		A pointer to a packet descriptor structure.
 * pool		A bpool handle.
 */
void neta_ppio_outq_desc_set_pool(struct neta_ppio_desc *desc, struct neta_bpool *ppool)
{
	int pid = ppool->id;

	(desc)->cmds[0] = ((desc)->cmds[0] & ~NETA_TXD_POOL_ID_MASK) | ((pid << 13) & NETA_TXD_POOL_ID_MASK);
	/* Set the HW buffer release mode in an outq packet descriptor */
	(desc)->cmds[0] = ((desc)->cmds[0] & ~NETA_TXD_BUFF_REL_MASK) | NETA_TXD_BUFF_REL_MASK;

	(desc)->cmds[0] |= NETA_TXD_FLZ_DESC_MASK;
	return;

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

struct neta_bpool *neta_ppio_inq_desc_get_bpool(struct neta_ppio_desc *desc, struct neta_ppio *ppio)
{
	struct neta_bpool *ppool;

	ppool = &neta_bpools[NETA_RXD_GET_POOL_ID(desc)];
	return ppool;
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

	*num = mvneta_txq_sent_desc_num_get(port, qid);
	return 0;
}

/* Get number of RX descriptors occupied by received packets */
static int neta_rxq_busy_desc_num_get(struct neta_port *pp, int qid)
{
	int val;

	val = neta_reg_read(pp, MVNETA_RXQ_STATUS_REG(qid));
	return val & MVNETA_RXQ_OCCUPIED_ALL_MASK;
}

/* Get pointer to the next RX descriptor to be processed by SW, and update the descriptor next index */
static struct neta_ppio_desc *neta_rxq_get_desc(struct neta_rx_queue *rxq,
						u32 *num_recv,
						struct neta_ppio_desc **extra_desc,
						u32 *extra_num)
{
	u32 rx_idx;

	rx_idx = rxq->next_desc_to_proc;
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

	if (unlikely((rx_idx + *num_recv) > rxq->size)) {
		*extra_desc = rxq->descs;
		/* extra_num is relative to start of desc array */
		*extra_num  = rx_idx + *num_recv - rxq->size;
		/* num_recv is relative to end of desc array */
		*num_recv = rxq->size - rx_idx;
		rxq->next_desc_to_proc = *extra_num;
	} else {
		rxq->next_desc_to_proc = (((rx_idx + *num_recv) == rxq->size) ? 0 : (rx_idx + *num_recv));
	}

/*
 *	pr_debug("%s\tdesc array: cur_idx=%d\tlast_idx=%d\n",__func__, rx_idx, rxq->desc_last_idx);
 *	pr_debug("%s\tdesc array: num_recv=%d\textra_num=%d\n",__func__,*num_recv, *extra_num);
 */

	return (rxq->descs + rx_idx);
}

/* Update num of rx desc called upon return from rx path */
static void neta_port_inq_update(struct neta_port *pp,
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
	struct neta_ppio_desc *rx_desc, *extra_rx_desc;
	struct neta_rx_queue *rxq;
	u32 recv_req = *num, extra_num = 0;

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
	rx_desc = neta_rxq_get_desc(rxq, &recv_req, &extra_rx_desc, &extra_num);

#if __BYTE_ORDER == __BIG_ENDIAN
	for (int i = 0; i < recv_req; i++)
		neta_ppio_desc_swap_ncopy(&descs[i], &rx_desc[i]);
#else
	memcpy(descs, rx_desc, recv_req * sizeof(*descs));
#endif
	if (extra_num) {
		memcpy(&descs[recv_req], extra_rx_desc, extra_num * sizeof(*descs));
		recv_req += extra_num; /* Put the split numbers back together */
	}
	/*  Update HW */
	neta_port_inq_update(port, rxq, recv_req, recv_req);
	rxq->desc_received -= recv_req;
	*num = recv_req;

#ifdef NETA_STATS_SUPPORT
/* TBD */
	if (port->maintain_stats) {
		rxq->threshold_rx_pkts += recv_req;
		if (unlikely(rxq->threshold_rx_pkts > PP2_STAT_UPDATE_THRESHOLD)) {
			pp2_ppio_inq_get_statistics(ppio, tc, qid, NULL, 0);
			rxq->threshold_rx_pkts = 0;
		}
	}
#endif
	return 0;
}

/* Tx descriptors helper methods */
/* Update HW with number of TX descriptors to be sent */
static void neta_txq_pend_desc_add(struct neta_port *pp,
				   struct neta_tx_queue *txq,
				   int pend_desc)
{
	u32 val;

	/* Only 255 descriptors can be added at once ; Assume caller
	 * process TX descriptors in quanta less than 256
	 */
	val = pend_desc + txq->pending;
	neta_reg_write(pp, MVNETA_TXQ_UPDATE_REG(txq->id), val);
	txq->pending = 0;
}

/* Get number of free TX descriptor */
static int neta_txq_free_desc_num(struct neta_tx_queue *txq)
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
static void neta_txq_desc_put(struct neta_tx_queue *txq)
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
		mvneta_txq_sent_desc_dec(pp, txq, tx_done);

	txq->count -= tx_done;

	return (txq->size - txq->count);
}

static int neta_port_enqueue(struct neta_port *port, u8 txq_id, struct neta_ppio_desc *descs, u16 num)
{
	struct neta_tx_queue *txq = &port->txqs[txq_id];
	struct neta_ppio_desc *tx_desc;
	int i;
	int free_desc = neta_txq_free_desc_num(txq);

	if (free_desc < num) {
		free_desc = neta_txq_done(port, txq);
		num = (free_desc < num) ? free_desc : num;
	}

	for (i = 0; i < num; i++) {
		/* Get a descriptor for the packet */
		tx_desc = neta_txq_next_desc_get(txq);
#if __BYTE_ORDER == __BIG_ENDIAN
		neta_ppio_desc_swap_ncopy(&tx_desc, &descs[i]);
#else
		memcpy(tx_desc, &descs[i], sizeof(struct neta_ppio_desc));
#endif
		/* be sure TX descriptor is ready to transmit */
		wmb();
		neta_txq_pend_desc_add(port, txq, 1);
	}
	txq->count += i;

	return i;
}

/*
 * Send a batch of frames (single dscriptor) on an OutQ of PP-IO.
 *
 * The routine assumes that the BM-Pool is either freed by HW (by appropriate desc
 * setter) or by the MUSDK client SW.
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
	if (port->maintain_stats) {
		struct neta_tx_queue *txq;

		txq = port->txqs[qid];
		txq->threshold_tx_pkts += desc_sent;
		if (unlikely(txq->threshold_tx_pkts > PP2_STAT_UPDATE_THRESHOLD)) {
			neta_ppio_outq_get_statistics(ppio, qid, NULL, 0);
			txq->threshold_tx_pkts = 0;
		}
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
	return 0;
}

int neta_ppio_disable(struct neta_ppio *ppio)
{
	return 0;
}
