/*
* ***************************************************************************
* Copyright (c) 2018 Marvell.
* ***************************************************************************
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation, either version 2 of the License, or any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
* ***************************************************************************
*/

#include <net/ip.h>

#include "giu_nic.h"

#ifdef CONFIG_MV_NSS_ENABLE
#include "mv_gnic_nss.h"
#endif

#ifdef CONFIG_STUB_MODE
int loopback_mode;
module_param(loopback_mode, int, S_IRUGO);
MODULE_PARM_DESC(loopback_mode, "Set loopback mode (disabled=0, mac echo=1, mac+ip echo=2, after skb alloc=3)");
#endif /* CONFIG_STUB_MODE */

#define AGNIC_PA_DEVICE_WATERMARK	0xcafe000000000000
#define AGNIC_COOKIE_DEVICE_WATERMARK	0xcafecafe
#define AGNIC_COOKIE_DRIVER_WATERMARK	0xdeaddead

/*
 * Swap source and destination MAC addresses
 */
static inline void agnic_stub_swap_l2(char *buf)
{
	u16 *eth_hdr;

	register u16 tmp;

	eth_hdr = (uint16_t *)buf;
	tmp = eth_hdr[0];
	eth_hdr[0] = eth_hdr[3];
	eth_hdr[3] = tmp;
	tmp = eth_hdr[1];
	eth_hdr[1] = eth_hdr[4];
	eth_hdr[4] = tmp;
	tmp = eth_hdr[2];
	eth_hdr[2] = eth_hdr[5];
	eth_hdr[5] = tmp;
}

/*
 * Swap source and destination IP addresses
 */
static inline void agnic_stub_swap_l3(char *buf)
{
	register u32 tmp32;

	buf += 14 + 12;
	tmp32 = ((uint32_t *)buf)[0];
	((uint32_t *)buf)[0] = ((uint32_t *)buf)[1];
	((uint32_t *)buf)[1] = tmp32;
}

int agnic_stub_loopback(struct agnic_q_vector *vector, struct agnic_ring *rx_ring, int budget)
{
	struct agnic_adapter *adapter = vector->adapter;
	struct agnic_rx_desc *rx_desc;
	struct agnic_tx_desc *tx_desc;
	struct agnic_bpool_desc *bp_desc;
	struct agnic_ring *bp_ring, *tx_ring;
	dma_addr_t dma_addr;
	void *data;
	int num_to_proc, num_rx_pkts, num_tx_free_pkts, num_tx_done_pkts;
	u32 tmp_flags;

	tx_ring = adapter->tx_ring[rx_ring->q_idx];
	bp_ring = &adapter->bp_ring[rx_ring->q_idx];

	/* handle RX ring. move all packets from rx ring directly to tx ring. */
	num_to_proc = AGNIC_RING_NUM_OCCUPIED(readl(rx_ring->producer_p), rx_ring->rx_cons_shadow, rx_ring->count);
	/* limit the receive to burst size */
	if (num_to_proc > budget)
		num_to_proc = budget;

	num_tx_free_pkts = AGNIC_RING_FREE(tx_ring->tx_prod_shadow, tx_ring->tx_next_to_reclaim, tx_ring->count);
	if (unlikely(num_to_proc > num_tx_free_pkts))
		num_to_proc = num_tx_free_pkts;

	num_rx_pkts = 0;
	while (num_rx_pkts < num_to_proc) {
		rx_desc = (struct agnic_rx_desc *)rx_ring->desc + rx_ring->rx_cons_shadow;

		if (unlikely(((rx_desc->buffer_addr & AGNIC_PA_DEVICE_WATERMARK) == AGNIC_PA_DEVICE_WATERMARK) ||
			(rx_desc->cookie == AGNIC_COOKIE_DEVICE_WATERMARK) ||
			(rx_desc->cookie == AGNIC_COOKIE_DRIVER_WATERMARK))) {
			agnic_dev_err("Illegal descriptor (PA:%llx, cookie:%llx @%d)!!!!\n",
				      rx_desc->buffer_addr, rx_desc->cookie, rx_ring->rx_cons_shadow);
			AGNIC_RING_PTR_INC(rx_ring->rx_cons_shadow, 1, rx_ring->count);
			adapter->stats.rx_errors++;
			num_rx_pkts++;
			continue;
		}

		if (unlikely(((struct agnic_bp_cookie *)rx_desc->cookie)->bp_ring != bp_ring)) {
			agnic_dev_err("Illegal bpool (%p@%d)!\n",
				      ((struct agnic_bp_cookie *)rx_desc->cookie)->bp_ring,
			       rx_ring->rx_cons_shadow);
			AGNIC_RING_PTR_INC(rx_ring->rx_cons_shadow, 1, rx_ring->count);
			adapter->stats.rx_errors++;
			num_rx_pkts++;
			continue;
		}

		/* Check for rx error. */
		if (unlikely(rx_desc->flags & RXD_FLAGS_RX_ERROR)) {
			agnic_dev_err("Rx descriptor error (idx %d, err: 0x%08x).\n",
				      rx_ring->rx_cons_shadow, rx_desc->flags);
desc_err:
			bp_desc = (struct agnic_bpool_desc *)bp_ring->desc + bp_ring->tx_prod_shadow;
			bp_desc->buff_addr_phys = rx_desc->buffer_addr;
			bp_desc->buff_cookie = rx_desc->cookie;

			/* Increment local / shadow consumer index. */
			AGNIC_RING_PTR_INC(bp_ring->tx_prod_shadow, 1, bp_ring->count);
			AGNIC_RING_PTR_INC(rx_ring->rx_cons_shadow, 1, rx_ring->count);
			adapter->stats.rx_errors++;
			num_rx_pkts++;
			continue;
		}

		dma_addr = rx_desc->buffer_addr;

		/* Unmap buffer after being accessed by HW. */
		dma_unmap_single(rx_ring->dev,
				dma_unmap_addr((struct agnic_bp_cookie *)rx_desc->cookie, addr),
				dma_unmap_len((struct agnic_bp_cookie *)rx_desc->cookie, len),
				DMA_FROM_DEVICE);

		data = ((struct agnic_bp_cookie *)rx_desc->cookie)->buff_virt_addr;
		data += (adapter->pkt_offset + rx_desc->pkt_offset);

		if (((*(u8 *)(data+14) & 0xf0) == 0x40) && swab16(*(u16 *)(data+12)) == 0x0800) {
			struct iphdr *ipv4hdr = data + 14;
			u16 tot_len = swab16(ipv4hdr->tot_len) + 14;
			u16 exp_len = rx_desc->byte_cnt;

#ifdef CONFIG_MV_NSS_ENABLE
			exp_len -= MV_NSS_METADATA_LEN;
#endif

			/* TODO: is this a BUG??? */
			if (tot_len < 60)
				tot_len = 60;

			if (tot_len != exp_len) {
				agnic_dev_err("core[%u] frame(%u)/rx-desc(%u) length mismatch\n",
					      smp_processor_id(), tot_len, exp_len);
				goto desc_err;
			}
		}

		agnic_stub_swap_l2(data);
		if (loopback_mode == 2)
			agnic_stub_swap_l3(data);

#ifdef CONFIG_MV_NSS_ENABLE
		data -= MV_NSS_METADATA_LEN;
#endif

		/* Map data buffer */
		dma_addr = dma_map_single(tx_ring->dev, data, rx_desc->byte_cnt, DMA_TO_DEVICE);
		if (unlikely(dma_mapping_error(tx_ring->dev, dma_addr))) {
			agnic_dev_err("Failed to map Tx buffer.\n");
			goto desc_err;
		}

		/* Set respective fields in tx cookie. */
		dma_unmap_len_set((struct agnic_bp_cookie *)rx_desc->cookie, len, rx_desc->byte_cnt);
		dma_unmap_addr_set((struct agnic_bp_cookie *)rx_desc->cookie, addr, dma_addr);

		/* handle TX ring */
		/* Get descriptor pointer. */
		tx_desc = (struct agnic_tx_desc *)tx_ring->desc + tx_ring->tx_prod_shadow;
		memset(tx_desc, 0, sizeof(*tx_desc));

		/* Initialize descriptor fields. */
		/* For now, single desc per packet. */
		tx_desc->flags = TXD_FLAGS_LAST | TXD_FLAGS_FIRST;
		tx_desc->flags |= (rx_desc->flags & RXD_FLAGS_MD_MODE) ? TXD_FLAGS_MD_MODE : 0;
		/* set the L3/L4 information */
		TXD_FLAGS_L3_OFFSET_SET(tx_desc->flags, RXD_FLAGS_L3_OFFSET_GET(rx_desc->flags));
		tmp_flags = RXD_FLAGS_L3_INFO_GET(rx_desc->flags);
		if ((tmp_flags >= RX_DESC_L3_TYPE_IPV4_NO_OPTS) &&
			(tmp_flags <= RX_DESC_L3_TYPE_IPV4_OTHER)) { /* IPv4 types */
			tmp_flags = TXD_FLAGS_L3_INFO_IPV4;
			if (unlikely(tmp_flags > RX_DESC_L3_TYPE_IPV4_NO_OPTS))
				tmp_flags |= TXD_FLAGS_GEN_L4_CSUM_NOT;
		} else if ((tmp_flags == RX_DESC_L3_TYPE_IPV6_NO_EXT) ||
			(tmp_flags == RX_DESC_L3_TYPE_IPV6_EXT)) { /* IPv6 types */
			tmp_flags = TXD_FLAGS_L3_INFO_IPV6;
			if (unlikely(tmp_flags != RX_DESC_L3_TYPE_IPV6_NO_EXT))
				tmp_flags |= TXD_FLAGS_GEN_L4_CSUM_NOT;
		} else
			tmp_flags = TXD_FLAGS_L3_INFO_OTHER;
		tx_desc->flags |= tmp_flags;
		if (likely(tmp_flags != TXD_FLAGS_L3_INFO_OTHER)) {
			TXD_FLAGS_IP_HDR_LEN_SET(tx_desc->flags, RXD_FLAGS_IP_HDR_LEN_GET(rx_desc->flags));
			tmp_flags = RXD_FLAGS_L4_TYPE_GET(rx_desc->flags);
			if (tmp_flags == RX_DESC_L4_TYPE_TCP) /* TCP */
				tmp_flags = TXD_FLAGS_L4_TCP;
			else if (tmp_flags == RX_DESC_L4_TYPE_UDP) /* UDP */
				tmp_flags = TXD_FLAGS_L4_UDP;
			else
				tmp_flags = TXD_FLAGS_L4_OTHER | TXD_FLAGS_GEN_L4_CSUM_NOT;
			tx_desc->flags |= tmp_flags;
		} else
			tx_desc->flags |= TXD_FLAGS_GEN_IPV4_CSUM_DIS;
		tx_desc->byte_cnt = rx_desc->byte_cnt;
		tx_desc->pkt_offset = rx_desc->pkt_offset;
		tx_desc->cookie = rx_desc->cookie;
		tx_desc->buffer_addr = dma_addr;

		/* TODO: remove this validation once DMA reordering issue is cleaned */
		rx_desc->cookie = AGNIC_COOKIE_DRIVER_WATERMARK;

		adapter->stats.rx_packets++;
		adapter->stats.rx_bytes += rx_desc->byte_cnt;

		/* Increment local / shadow consumer index. */
		AGNIC_RING_PTR_INC(rx_ring->rx_cons_shadow, 1, rx_ring->count);
		AGNIC_RING_PTR_INC(tx_ring->tx_prod_shadow, 1, tx_ring->count);

		num_rx_pkts++;
	}
	if (likely(num_rx_pkts)) {
		/* Update ring pointers. */
		writel(rx_ring->rx_cons_shadow, rx_ring->consumer_p);
		/* Update remote producer pointer. */
		writel(tx_ring->tx_prod_shadow, tx_ring->producer_p);
	}

	/*
	 * Check the number of sent and non-free descriptors on tx ring and free them back to bpool.
	 */
	num_to_proc = AGNIC_RING_NUM_OCCUPIED(readl(tx_ring->consumer_p), tx_ring->tx_next_to_reclaim, tx_ring->count);

	/* Loop over all transmitted descriptors, and free. */
	num_tx_done_pkts = 0;
	while (num_tx_done_pkts < num_to_proc) {
		/* Get descriptor pointer. */
		tx_desc = (struct agnic_tx_desc *)tx_ring->desc + tx_ring->tx_next_to_reclaim;

		if (((struct agnic_bp_cookie *)tx_desc->cookie)->bp_ring != bp_ring)
			agnic_dev_err("Bad BPool in TxQ (%p vs %p)\n",
				      ((struct agnic_bp_cookie *)tx_desc->cookie)->bp_ring, bp_ring);

		bp_desc = (struct agnic_bpool_desc *)bp_ring->desc + bp_ring->tx_prod_shadow;
		bp_desc->buff_addr_phys = tx_desc->buffer_addr;
		bp_desc->buff_cookie = tx_desc->cookie;
		adapter->stats.tx_bytes += tx_desc->byte_cnt;
		adapter->stats.tx_packets++;

		/* Increment local / shadow consumer index. */
		AGNIC_RING_PTR_INC(bp_ring->tx_prod_shadow, 1, bp_ring->count);
		/* Increment local / shadow consumer index. */
		AGNIC_RING_PTR_INC(tx_ring->tx_next_to_reclaim, 1, tx_ring->count);
		num_tx_done_pkts++;
	}
	if (likely(num_tx_done_pkts))
		/* Update remote producer pointer. */
		writel(bp_ring->tx_prod_shadow, bp_ring->producer_p);

	return num_rx_pkts;
}
