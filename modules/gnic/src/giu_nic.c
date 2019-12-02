/*
** Copyright (C) 1999 - 2015 Intel Corporation.
** Copyright (C) 2015-2016 Marvell International Ltd.
**
** This code was derived from Intel's ixgbe Linux driver.
**
** This program is free software: you can redistribute it and/or
** modify it under the terms of the GNU General Public License as
** published by the Free Software Foundation, either version 2 of the
** License, or any later version.
**
** This program is distributed in the hope that it will be useful, but
** WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
** General Public License for more details.
*/

#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/moduleparam.h>
#include <linux/if_vlan.h>
#include <linux/pci.h>
#include <linux/irq.h>
#include <linux/kthread.h>
#include <net/ip.h>
#include "giu_nic.h"
#include "giu_nic_hw.h"
#include "giu_nic_mgmt.h"
#include "giu_nic_ethtool.h"

#ifdef CONFIG_MV_NSS_ENABLE
#include "giu_nic_md.h"
#endif

#define AGNIC_PA_DEVICE_WATERMARK	0xcafe000000000000
#define AGNIC_COOKIE_DEVICE_WATERMARK	0xcafecafe
#define AGNIC_COOKIE_DRIVER_WATERMARK	0xdeaddead

#define NOTIFICATION_HANDLE_LIMIT	4	/* max number of async notifications which can
						 * be handled in a single timer handler
						 */

#define AGNIC_DEFAULT_SKB_PAD		NET_SKB_PAD
#define RATE_LIMIT_BURST_SIZE		64

/* Each interrupt type has # of intterupts as # if q_vectors */
#define AGNIC_VECTOR_TO_Q_VECTOR(vec, num_q_vec)	(vec % num_q_vec)

#define DEFAULT_MSG_ENABLE (NETIF_MSG_DRV|NETIF_MSG_PROBE|NETIF_MSG_LINK)
static int debug = -1;
module_param(debug, int, 0);
MODULE_PARM_DESC(debug, "Debug message level (0=Off, 1=probe, 16=All))");

#ifdef CONFIG_STUB_MODE
extern int loopback_mode;
int agnic_stub_loopback(struct agnic_q_vector *vector, struct agnic_ring *rx_ring, int budget);
#endif /* CONFIG_STUB_MODE */

static int num_tcs = 1;
module_param(num_tcs, int, S_IRUGO);
MODULE_PARM_DESC(num_tcs, "Number of TCs for both ingress and egress");

static int num_qs_per_tc = 1;
module_param(num_qs_per_tc, int, S_IRUGO);
MODULE_PARM_DESC(num_qs_per_tc, "Number of queues per TC. 2 and above means RSS is enable");

static unsigned long cpu_mask;
module_param(cpu_mask, long, S_IRUGO);
MODULE_PARM_DESC(cpu_mask, "CPU Mask (when set to 0 then online cpus mask will be used");

static int rss_mode = 0; /* 2TUPLE */;
module_param(rss_mode, int, S_IRUGO);
MODULE_PARM_DESC(rss_mode, "Set rss_mode (2TUPLE=0, 5TUPLE=1)");

static int default_queue;
module_param(default_queue, int, S_IRUGO);
MODULE_PARM_DESC(default_queue, "Set default queue for non IP frames");

static unsigned int msix_mode = 3;
module_param(msix_mode, int, S_IRUGO);
MODULE_PARM_DESC(msix_mode, "MSI-X Mode (Disabled=0, Rx=1, Tx=2, Rx & Tx=3");

static unsigned int feature_enable = AGNIC_FEATURES_KEEP_ALIVE_EN;
module_param(feature_enable, int, S_IRUGO);
MODULE_PARM_DESC(feature_enable, "Feature Enable Mask (Bit0=Keep-alive)");

#define AGNIC_MTU(adapter)	((adapter)->netdev->mtu + ETH_HLEN + ETH_FCS_LEN)

/* Forward declarations. */
static int agnic_bpool_refill_rx_buffs(struct agnic_adapter *adapter, struct agnic_ring *bp_ring,
		int buf_sz, int num);

static int agnic_net_stop(struct net_device *netdev);
/*
** agnic_net_get_stats() - Return network interface statistics.
*/
static struct net_device_stats *agnic_net_get_stats(struct net_device *dev)
{
	struct agnic_adapter *adapter = netdev_priv(dev);
	return &adapter->stats;
}

/*
** agnic_net_tx_timeout() - Tx timeout indication.
*/
static void agnic_net_tx_timeout(struct net_device *dev)
{
	pr_err("%s: Tx timeout\n", dev->name);
}

static void agnic_txdone_timer_set(struct agnic_q_vector *vector)
{
	ktime_t interval;

	/* The timer is used onlt if Tx interrupts are not enabled */
	if (vector->adapter->flags & AGNIC_FLAG_TX_MSIX_ENABLED)
		return;

	if (!vector->txdone_timer_scheduled) {
		agnic_debug(vector->adapter, "Scheduling txdone timer.\n");
		/* Set tx done state to 'scheduled' */
		vector->txdone_timer_scheduled = true;
		interval = ktime_set(0, AGNIC_TXDONE_HRTIMER_PERIOD_NS);
		hrtimer_start(&vector->tx_done_timer, interval,
				HRTIMER_MODE_REL_PINNED);
	}
}
/*
 * agnic_tx_done_handle_ring - Process tx done events for a givne ring.
 * The return value is the number of remaining unprocessed descritpros.
 */
static int agnic_tx_done_handle_ring(struct agnic_ring *ring, int limit)
{
	struct agnic_cookie *cookies_list;
	struct agnic_tx_cookie *tx_cookie;
	int rem = 0;
	u16 clean_idx, cons_idx;

	/* If interface was stopped, enable interrupts and exit. */
	if (!netif_running(ring->netdev))
		return rem;

	cookies_list = ring->cookie_list;

	clean_idx = ring->tx_next_to_reclaim;
	cons_idx = readl(ring->consumer_p);

	/* Loop over all transmitted descriptors, and free. */
	while ((clean_idx != cons_idx) && (limit)) {
		agnic_debug(ring->adapter, "Releasing tx-desc %d.\n", clean_idx);
		tx_cookie = (struct agnic_tx_cookie *)(&cookies_list[clean_idx]);
		dma_unmap_single(ring->dev,
				dma_unmap_addr(tx_cookie, dma),
				dma_unmap_len(tx_cookie, len),
				DMA_TO_DEVICE);

		dev_consume_skb_any(tx_cookie->skb);
		AGNIC_RING_PTR_INC(clean_idx, 1, ring->count);
		limit--;
	}

	if (!limit)
		agnic_debug(ring->adapter, "Reached tx-done reclaim limit.\n");

	ring->tx_next_to_reclaim = clean_idx;

	/* Re-read the consumer pointer in case new packets are ready for
	 * release.
	 */
	cons_idx = readl(ring->consumer_p);
	rem = AGNIC_RING_NUM_OCCUPIED(cons_idx, ring->tx_next_to_reclaim, ring->count);

	agnic_debug(ring->adapter, "txdone handler done, %d remaining.\n", rem);
	return rem;
}

static void agnic_txdone_tasklet_callback(unsigned long data)
{
	struct agnic_q_vector *vector = (struct agnic_q_vector *)data;
	struct net_device *netdev = vector->adapter->netdev;
	struct agnic_ring *ring;
	unsigned int rem = 0, irq;

	/* Hold the tx-lock (which is also acquired by the stack during the
	 * xmit operation, to prevent concurrent execution of tx-done and xmit
	 * callback.
	 * TODO: Remove this once we add support for tx-queue per CPU.
	 */
	netif_tx_lock(netdev);
	/* Process all the Tx ring in vector. */
	agnic_for_each_ring(ring, vector->tx)
		rem += agnic_tx_done_handle_ring(ring, INT_MAX);

	netif_tx_unlock(netdev);

	/* Set tx_done schedule state as done */
	vector->txdone_timer_scheduled = false;

	/* Enable Tx interrupts. */
	if (vector->adapter->flags & AGNIC_FLAG_TX_MSIX_ENABLED) {
		irq = vector->tx_msix_entry->vector;
		enable_irq(irq);
	}

	/* Set the timer in case not all the packets were processed */
	if (rem)
		agnic_txdone_timer_set(vector);
}

/*
 * agnic_txdone_hr_timer_callback - Handler of the tx-done timer, this function
 * simply schedules the tx-done tasklet in order not to do the txdone in
 * hrtimer context (softirq).
 */
static enum hrtimer_restart agnic_txdone_hr_timer_callback(struct hrtimer *timer)
{
	struct agnic_q_vector *vector = container_of(timer,
			 struct agnic_q_vector, tx_done_timer);

	tasklet_schedule(&vector->tx_done_tasklet);

	return HRTIMER_NORESTART;
}

/* Handle vlan info */
static inline u16 agnic_skb_tx_vlan_info(struct sk_buff *skb, struct agnic_tx_desc *desc)
{
	u16 ethertype = ntohs(skb->protocol);
	u8 vlan_num = 0;

	/* Check for VLAN */
	if (skb_vlan_tagged(skb)) {
		struct vlan_ethhdr *veth = vlan_eth_hdr(skb);

		ethertype = ntohs(veth->h_vlan_encapsulated_proto);
		vlan_num++;

		/* Check for Double VLAN */
		if (ethertype == ETH_P_8021Q) {
			/* Move to next (VLAN) header */
			struct vlan_hdr *vhdr = (void *)veth + sizeof(struct vlan_ethhdr);

			ethertype = ntohs(vhdr->h_vlan_encapsulated_proto);
			vlan_num++;
		}
	}

	/* Update number of VLANs */
	desc->vlan_info = vlan_num;

	return ethertype;
}

/* Set Tx descriptors fields relevant for CSUM calculation */
u32 agnic_tx_desc_csum(int l3_offs, int l3_proto,
			  int ip_hdr_len, int l4_proto)
{
	u32 flags = 0;

	/* fields: L3_offset, IP_hdrlen, L3_type, G_IPv4_chk,
	 * G_L4_chk, L4_type required only for checksum calculation
	 */
	TXD_FLAGS_L3_OFFSET_SET(flags, l3_offs);
	TXD_FLAGS_IP_HDR_LEN_SET(flags, ip_hdr_len);

	/* Set L3 type and csum mode
	 * Note: initial value of flags is set to L3 CSUM enabled
	 */
	switch (l3_proto) {
	case ETH_P_IPV6:
		flags |= TXD_FLAGS_L3_INFO_IPV6; /* enable IPv6 */
		break;
	case ETH_P_IP:
		/* Do nothing as for IPv4 as flags is initialized
		 * to IPv4 protocol type and L3 CSUM enabled)
		 */
		break;
	default:
		/* Set L3 type as OTHER and disable csum offload */
		flags |= (TXD_FLAGS_L3_INFO_OTHER |
				TXD_FLAGS_GEN_IPV4_CSUM_DIS);
	}

	/* Set L4 protocol type
	 * Note: initial value of flags is set to L4 CSUM enabled
	 *	 and TCP protocol type
	 */
	switch (l4_proto) {
	case IPPROTO_UDP:
		flags |= TXD_FLAGS_L4_UDP;	/* enable UDP */
		break;
	case IPPROTO_TCP:
		/* Do nothing as for TCP as flags is initialized
		 * to TCP protocol type and L4 CSUM enabled)
		 */
		break;
	default:
		flags |= (TXD_FLAGS_L4_OTHER |
				TXD_FLAGS_GEN_L4_CSUM_NOT);
	}

	return flags;
}

/* Handle tx checksum & parsing info */
static u32 agnic_skb_tx_parsing_info(struct sk_buff *skb, struct agnic_tx_desc *desc)
{
	u16 ethertype;
	struct ipv6hdr *ipv6h = NULL;
	struct iphdr *ipv4h;
	int ip_hdr_len = 0;
	u32 flags = 0;
	u8 l4_proto = 0;

	/* Check for VLAN headers and get the real Ethernet type (if
	 * VLAN exists)
	 */
	ethertype = agnic_skb_tx_vlan_info(skb, desc);

	/* Get header len and L4 protocol type
	 */
	switch (ethertype) {
	case ETH_P_IP:
		ipv4h = ip_hdr(skb);
		WARN_ON(!ipv4h);
		ip_hdr_len = ipv4h->ihl;
		l4_proto = ipv4h->protocol;
		break;
	case ETH_P_IPV6:
		ipv6h = ipv6_hdr(skb);
		WARN_ON(!ipv6h);
		if (skb_network_header_len(skb) > 0)
			ip_hdr_len = (skb_network_header_len(skb) >> 2);
		/* Read l4_protocol from one of IPv6 extra headers */
		l4_proto = ipv6h->nexthdr;
		break;
	}

	flags = agnic_tx_desc_csum(skb_network_offset(skb), ethertype,
					ip_hdr_len, l4_proto);

	/* if no need to offload csum (from any reason) override previous
	 * csum configuration and set it to disabled (for both L3/L4)
	 */
	if (skb->ip_summed != CHECKSUM_PARTIAL) {
		/* Only need to clear L4 configuration as it's a field.
		 * L3 configuration is 1 for disable so no need to clear it.
		*/
		flags &= ~TXD_FLAGS_GEN_L4_CSUM_MASK;
		flags |= (TXD_FLAGS_GEN_L4_CSUM_NOT | TXD_FLAGS_GEN_IPV4_CSUM_DIS);
	}

	return flags;
}

/*
 * agnic_net_xmit() - Net xmit function: transmit buffer through shared memory.
 * Currently, only basic transmit is supported (no offloads, no multi desc per
 * packet).
 * TODO:
 *   - Add support for multi descriptor per packet.
 *   - Add support for TSO.
 *   - Add support for CSUM offload.
*/
static int agnic_net_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct agnic_adapter *adapter = netdev_priv(dev);
	struct agnic_ring *ring = NULL;
	struct agnic_tx_desc *desc;
	struct agnic_tx_cookie *tx_buf;
	dma_addr_t dma_addr;
	unsigned int size;
	unsigned int orig_len;

#ifdef CONFIG_STUB_MODE
	if (loopback_mode == 1 || loopback_mode == 2) {
		/* In loopback mode we don't want interference from Linux stack  */
		adapter->stats.tx_dropped++;
		goto error;
	}
#endif /* CONFIG_STUB_MODE */

#ifdef AGNIC_DEBUG
	/* params validation in debug mode only. */
	if (skb->queue_mapping >= adapter->num_tx_queues) {
		agnic_dev_err("BUG: skb->queue_mapping (%d) exceeds the number of tx queues (%d).\n",
				skb->queue_mapping, adapter->num_tx_queues);
		goto error;
	}

#endif /* AGNIC_DEBUG */

	ring = adapter->tx_ring[skb->queue_mapping];

#ifdef AGNIC_DEBUG
	if (ring->tx_prod_shadow >= ring->count) {
		agnic_dev_err("BUG: tx_prod_shadow (%d) exceeds the queue size (%d).\n",
				ring->tx_prod_shadow, ring->count);
		goto error;
	}
#endif /* AGNIC_DEBUG */

	agnic_debug(adapter, "xmit - %d Bytes, Q - %d.\n", skb->len, skb->queue_mapping);

	orig_len = skb->len;
	/* Check tx Q status. */
	if (unlikely(AGNIC_RING_IS_FULL(ring->tx_prod_shadow, ring->tx_next_to_reclaim, ring->count))) {
		agnic_debug(adapter, "TxQ is full (P:%d, C:%d, N2R:%d).\n",
				ring->tx_prod_shadow, readl(ring->consumer_p), ring->tx_next_to_reclaim);
		adapter->stats.tx_dropped++;
		goto error;
	}

	/* TODO: Add support for fragmented SKBs. */
	if (unlikely(skb_shinfo(skb)->nr_frags != 0)) {
		agnic_dev_err("No support for fragmented skb.\n");
		adapter->stats.tx_dropped++;
		goto error;
	}

	if (unlikely(netif_queue_stopped(dev))) {
		adapter->stats.tx_errors++;
		adapter->stats.tx_carrier_errors++;
		agnic_dev_err("Transmitting while stopped.\n");
		goto error;
	}

	/* Get descriptor pointer. */
	desc = (struct agnic_tx_desc *)ring->desc + ring->tx_prod_shadow;
	tx_buf = (struct agnic_tx_cookie *)&ring->cookie_list[ring->tx_prod_shadow];

	memset(desc, 0, sizeof(*desc));

#ifdef CONFIG_MV_NSS_ENABLE
	/* If headroom is large enough, add metadata */
	if (likely(skb_headroom(skb) >= MV_NSS_METADATA_LEN)) {
		agnic_metadata_xmit_hdl(skb);
		desc->flags |= TXD_FLAGS_MD_MODE;
	}
#endif

	/** Initialize descriptor fields **/
	/* Update CSUM and Parsing info. */
	desc->flags |= agnic_skb_tx_parsing_info(skb, desc);
	/* For now, single desc per packet. */
	desc->flags |= TXD_FLAGS_LAST | TXD_FLAGS_FIRST;
	size = skb->len;
	desc->byte_cnt = size;
	desc->pkt_offset = 0x0;
	desc->cookie = (u64)tx_buf;

	/* Map data buffer */
	dma_addr = dma_map_single(ring->dev, skb->data, size, DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(ring->dev, dma_addr))) {
		agnic_dev_err("Failed to map Tx buffer.\n");
		adapter->stats.tx_dropped++;
		goto error_map;
	}

	/* Set respective fields in tx cookie. */
	dma_unmap_len_set(tx_buf, len, size);
	dma_unmap_addr_set(tx_buf, addr, dma_addr);
	tx_buf->skb = skb;
	tx_buf->dma = dma_addr;
	desc->buffer_addr = dma_addr;

	/* Increment local / shadow producer index. */
	AGNIC_RING_PTR_INC(ring->tx_prod_shadow, 1, ring->count);
	ring->total_packets++;

	/* Send interrupt to device side */
	adapter->xmit_notify(adapter, ring->tc);

	/* Update remote producer pointer. */
	writel(ring->tx_prod_shadow, ring->producer_p);

	adapter->stats.tx_bytes += orig_len;
	adapter->stats.tx_packets++;

	agnic_txdone_timer_set(ring->q_vector);

	return NETDEV_TX_OK;
error_map:
error:
	dev_kfree_skb_any(skb);
	if (ring)
		agnic_txdone_timer_set(ring->q_vector);

	return NETDEV_TX_OK;
}

/*
** agnic_set_rx_promisc - set/clear Rx promiscuous mode
*/
static void agnic_set_rx_promisc(struct net_device *dev, int val)
{
	struct agnic_msg_params msg_params;
	struct agnic_mgmt_cmd_params cmd_params;
	struct agnic_adapter *adapter = netdev_priv(dev);
	int ret;

	agnic_debug(adapter, "Sending promisc\n");
	cmd_params.promisc = val;
	msg_params.cmd_code = CC_PF_PROMISC;
	msg_params.client_id = 0;
	msg_params.client_type = CDT_PF;
	msg_params.msg = &cmd_params;
	msg_params.msg_len = sizeof(cmd_params);
	msg_params.resp_msg = NULL;	/* TODO: response not possible */
	msg_params.resp_msg_len = 0;	/* TODO: response not possible */

	ret = agnic_mgmt_command_send(adapter, &msg_params);
	if (ret)
		netdev_err(dev, "promisc change failed\n");
}

/*
 * agnic_set_rx_mc_promisc - set/clear Rx multicast mode
 */
static void agnic_set_rx_mc_promisc(struct net_device *dev, int val)
{
	struct agnic_msg_params msg_params;
	struct agnic_mgmt_cmd_params cmd_params;
	struct agnic_adapter *adapter = netdev_priv(dev);
	int ret;

	agnic_debug(adapter, "Sending mc promisc\n");
	cmd_params.mc_promisc = val;
	msg_params.cmd_code = CC_PF_MC_PROMISC;
	msg_params.client_id = 0;
	msg_params.client_type = CDT_PF;
	msg_params.msg = &cmd_params;
	msg_params.msg_len = sizeof(cmd_params);
	msg_params.resp_msg = NULL;	/* TODO: response not possible */
	msg_params.resp_msg_len = 0;	/* TODO: response not possible */

	ret = agnic_mgmt_command_send(adapter, &msg_params);
	if (ret)
		netdev_err(dev, "multicast change failed\n");
}

/*
 * agnic_remove_mac_addr - remove mac address
 */
static int agnic_remove_mac_addr(struct net_device *dev, char *addr)
{
	struct agnic_msg_params msg_params;
	struct agnic_mgmt_cmd_params cmd_params;
	struct agnic_adapter *adapter = netdev_priv(dev);
	int ret;

	/* mc_addr points to the removed address */
	memset(&cmd_params, 0, sizeof(cmd_params));
	ether_addr_copy(cmd_params.mac_addr, addr);
	msg_params.cmd_code = CC_PF_MC_REMOVE_ADDR;
	msg_params.client_id = 0;
	msg_params.client_type = CDT_PF;
	msg_params.msg = &cmd_params;
	msg_params.msg_len = sizeof(cmd_params);

	/* don't wait for a response while, addr_list_lock lock is acquired here */
	msg_params.resp_msg = NULL;
	msg_params.resp_msg_len = 0;
	ret = agnic_mgmt_command_send(adapter, &msg_params);

	return ret;
}

/*
 * agnic_add_mac_addr - add new mac address
 */
static int agnic_add_mac_addr(struct net_device *dev, char *addr)
{
	struct agnic_msg_params msg_params;
	struct agnic_mgmt_cmd_params cmd_params;
	struct agnic_adapter *adapter = netdev_priv(dev);
	int ret;

	memset(&cmd_params, 0, sizeof(cmd_params));
	ether_addr_copy(cmd_params.mac_addr, addr);
	msg_params.cmd_code = CC_PF_MC_ADD_ADDR;
	msg_params.client_id = 0;
	msg_params.client_type = CDT_PF;
	msg_params.msg = &cmd_params;
	msg_params.msg_len = sizeof(cmd_params);

	/* don't wait for a response while, addr_list_lock lock is acquired here */
	msg_params.resp_msg = NULL;
	msg_params.resp_msg_len = 0;
	ret = agnic_mgmt_command_send(adapter, &msg_params);

	return ret;
}

/*
 * agnic_add_rx_mc_addr - add new multicast address
 */
static void agnic_add_rx_mc_addr(struct net_device *dev)
{
	struct netdev_hw_addr *ha, *new_ha = NULL;
	struct agnic_mc_addr *mc_addr;
	int    alloc_size = sizeof(*mc_addr);
	struct agnic_adapter *adapter = netdev_priv(dev);
	int    ret;

	if (netdev_mc_empty(dev)) {
		netdev_err(dev, "Failed to add new multicast address\n");
		return;
	}

	/* the last element points to the new added multicast address.
	 * use only 'netdev_hw_addr' interface. so, list_for_each_prev
	 * and list_last_entry are out of the question.
	 */
	netdev_for_each_mc_addr(ha, dev)
		new_ha = ha;

	/* new_ha points to the last list elemet which is the new address */
	mc_addr = kmalloc(alloc_size, GFP_ATOMIC);
	if (!mc_addr)
		return;

	memset(mc_addr, 0, sizeof(*mc_addr));

	/* add to shadow mc address list */
	ether_addr_copy(mc_addr->addr, new_ha->addr);
	list_add_tail(&mc_addr->list, &adapter->mc_list.list);
	agnic_mc_count_inc(adapter);

	/* send mgmt command only if mc promisc is disabled */
	if (adapter->flags & AGNIC_FLAG_MC_PROMISC)
		return;

	ret = agnic_add_mac_addr(dev, mc_addr->addr);
	if (ret) {
		netdev_err(dev, "Multicast address add failed\n");
		return;
	}

	/* sign the mcast address as exists in remote hw */
	mc_addr->hw_exist = true;

	agnic_debug("Add multicast address %02x:%02x:%02x:%02x:%02x:%02x\n",
				mc_addr->addr[0], mc_addr->addr[1],
				mc_addr->addr[2], mc_addr->addr[3],
				mc_addr->addr[4], mc_addr->addr[5]);
}

/*
 * agnic_remove_mc_addr - set multicast new address
 */
static void agnic_remove_mc_addr(struct net_device *dev)
{
	struct agnic_adapter  *adapter = netdev_priv(dev);
	struct netdev_hw_addr *ha;
	struct agnic_mc_addr  *mc_addr;
	int ret, rep_count = 0;
	char addr[ETH_ALEN];
	bool hw_exist;

	/* run and compare both lists to get the removed address */
	list_for_each_entry(mc_addr, &adapter->mc_list.list, list) {
		rep_count = 0;
		netdev_for_each_mc_addr(ha, dev) {
			if (ether_addr_equal(ha->addr, mc_addr->addr))
				break;
			rep_count++;
		}
		if (rep_count == netdev_mc_count(dev))
			break;
	}

	/* no multicast address to remove */
	if (rep_count != netdev_mc_count(dev))
		return;

	hw_exist = mc_addr->hw_exist;
	ether_addr_copy(addr, mc_addr->addr);

	/* update the link list */
	list_del(&mc_addr->list);
	kfree(mc_addr);
	agnic_mc_count_dec(adapter);

	/* when agnic is in mc promisc mode, the ppv2 hw doesn't maintain
	 * an mc address list. thus, send mgmt command only if mc
	 * promisc is disabled
	 */
	if (!hw_exist)
		return;

	ret = agnic_remove_mac_addr(dev, addr);
	if (ret) {
		netdev_err(dev, "Multicast address remove command has failed\n");
		return;
	}

	agnic_debug("Remove multicast address %02x:%02x:%02x:%02x:%02x:%02x\n",
					addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
}

/*
 * agnic_send_mc_addr_list - send multicast list
 */
static void agnic_send_mc_addr_list(struct net_device *dev)
{
	int    ret;
	struct agnic_adapter  *adapter = netdev_priv(dev);
	struct agnic_mc_addr *mc_addr;

	list_for_each_entry(mc_addr, &adapter->mc_list.list, list) {
		ret = agnic_add_mac_addr(dev, mc_addr->addr);
		if (ret) {
			netdev_err(dev, "Failed to add multicast address %02x:%02x:%02x:%02x:%02x:%02x\n",
					   mc_addr->addr[0], mc_addr->addr[1], mc_addr->addr[2],
					   mc_addr->addr[3], mc_addr->addr[4], mc_addr->addr[5]);
		} else
			mc_addr->hw_exist = true;
	}
}

/*
 * agnic_set_rx_mode - Set Rx reception mode (Unicast, Multicast, Promiscuous).
 */
static void agnic_set_rx_mode(struct net_device *dev)
{
	struct agnic_adapter *adapter = netdev_priv(dev);

	if (dev->flags & IFF_PROMISC &&
			!(adapter->flags & AGNIC_FLAG_UC_PROMISC)) {
		agnic_set_rx_promisc(dev, AGNIC_PROMISC_ENABLE);
		adapter->flags |= AGNIC_FLAG_UC_PROMISC;
	} else if (netdev_uc_count(dev) > AGNIC_MAC_UC_FILT_MAX) {
		/* TODO: modify once uc filtering is supported */
		/* TODO: uc mac address setting is not supported by MUSDK
		 *	 set system to promisc mode instead
		 */
		if (!(adapter->flags & AGNIC_FLAG_UC_PROMISC)) {
			agnic_set_rx_promisc(dev, AGNIC_PROMISC_ENABLE);
			adapter->flags |= AGNIC_FLAG_UC_PROMISC;

			agnic_debug("enter promisc uc filtering %d, %d",
			    netdev_uc_count(dev), AGNIC_MAC_UC_FILT_MAX);
		}
	} else {
		/* TODO: remove this warning once uc filtering is supported */
		/* TODO: flush uc mac addresses */
		/* TODO: netdev_for_each_uc_addr(), add mc mac addresses */

		/* Interafce is in promiscuous and a we got a request to exit
		 * promiscuous mode
		 */
		if (!(dev->flags & IFF_PROMISC) &&
			adapter->flags & AGNIC_FLAG_UC_PROMISC) {
			agnic_set_rx_promisc(dev, AGNIC_PROMISC_DISABLE);
			adapter->flags &= ~AGNIC_FLAG_UC_PROMISC;

			agnic_debug("exit promisc uc filtering %d, %d",
			    netdev_uc_count(dev), AGNIC_MAC_UC_FILT_MAX);
		}
	}

	if (dev->flags & IFF_ALLMULTI &&
			!(adapter->flags & AGNIC_FLAG_MC_PROMISC)) {
		/* no need to send flush message. hw's driver always
		 * performs flush addresses when it enters promisc mode
		 */
		agnic_set_rx_mc_promisc(dev, AGNIC_MC_PROMISC_ENABLE);
		adapter->flags |= AGNIC_FLAG_MC_PROMISC;
	}

	/* set multiall mode incase the agnic has more
	 * than AGNIC_MAC_MC_FILT_MAX multicat addresses
	 */
	if (netdev_mc_count(dev) > AGNIC_MAC_MC_FILT_MAX &&
			!(adapter->flags & AGNIC_FLAG_MC_PROMISC)) {
		/* no need to send flush message. hw's driver always
		 * performs flush addresses when it enters promisc mode
		 */
		agnic_set_rx_mc_promisc(dev, AGNIC_MC_PROMISC_ENABLE);

		adapter->flags |= AGNIC_FLAG_MC_PROMISC;
		agnic_add_rx_mc_addr(dev);
				agnic_debug("enter promisc mc filtering %d, %d",
				    netdev_mc_count(dev), AGNIC_MAC_MC_FILT_MAX);
	} else {
		if (netdev_mc_count(dev) > agnic_mc_count(adapter))
			agnic_add_rx_mc_addr(dev);
		else if (netdev_mc_count(dev) < agnic_mc_count(adapter)) {
			agnic_remove_mc_addr(dev);
			if (netdev_mc_count(dev) == AGNIC_MAC_MC_FILT_MAX &&
				adapter->flags & AGNIC_FLAG_MC_PROMISC) {
				/* exit from promisc mode. send all mc address list */
				agnic_set_rx_mc_promisc(dev, AGNIC_MC_PROMISC_DISABLE);
				adapter->flags &= ~AGNIC_FLAG_MC_PROMISC;
				agnic_send_mc_addr_list(dev);
			}
		}
	}
}

/*
 * agnic_net_set_address() - Set new MAC address.
 * TODO: Configure MAC Address into HW.
 * TODO: Configure L2 Filter into HW.
 */
static int agnic_net_set_address(struct net_device *dev, void *p)
{
	struct agnic_msg_params msg_params;
	struct agnic_mgmt_cmd_params cmd_params;
	struct agnic_mgmt_cmd_resp cmd_resp;
	struct sockaddr *addr = p;
	struct agnic_adapter *adapter = netdev_priv(dev);
	int ret;

	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	/* Management set-address. */
	memcpy(cmd_params.mac_addr, addr->sa_data, dev->addr_len);

	agnic_debug(adapter, "Sending set-address\n");
	msg_params.cmd_code = CC_PF_MAC_ADDR;
	msg_params.client_id = 0;
	msg_params.client_type = CDT_PF;
	msg_params.msg = &cmd_params;
	msg_params.msg_len = sizeof(cmd_params);
	msg_params.resp_msg = &cmd_resp;
	msg_params.resp_msg_len = sizeof(cmd_resp);

	ret = agnic_mgmt_command_send(adapter, &msg_params);
	if (ret)
		goto error;

	memcpy(dev->dev_addr, addr->sa_data, dev->addr_len);
	return 0;
error:
	netdev_err(dev, "fail to change MAC address\n");
	return ret;
}

/*
** agnic_change_mtu - Set Maximum Transfer Unit
*/
static int agnic_change_mtu(struct net_device *dev, int new_mtu)
{
	int ret;
	struct agnic_msg_params msg_params;
	struct agnic_mgmt_cmd_params cmd_params;
	struct agnic_mgmt_cmd_resp cmd_resp;
	struct agnic_adapter *adapter = netdev_priv(dev);

	if (new_mtu > AGNIC_MAX_MTU)
		return -EINVAL;

	msg_params.cmd_code = CC_PF_MTU;
	msg_params.client_id = 0;
	msg_params.client_type = CDT_PF;
	msg_params.msg_len = sizeof(cmd_params);
	msg_params.msg = &cmd_params;

	msg_params.resp_msg = &cmd_resp;
	msg_params.resp_msg_len = sizeof(cmd_resp);

	cmd_params.pf_set_mtu.mtu = new_mtu;

	ret = agnic_mgmt_command_send(adapter, &msg_params);
	if (ret)
		goto error;

	dev->mtu = new_mtu;

	return 0;
error:
	/* TODO: Define configuration rollback at NIC level, then implement
	 * here.
	 */
	agnic_dev_err("Failed to configure network mtu for hardware.\n");
	return ret;
}

/*
** agnic_set_enable - Set enable/disable
*/
static int agnic_set_enable(struct net_device *dev, bool enable)
{
	int ret;
	struct agnic_msg_params msg_params;
	struct agnic_mgmt_cmd_resp cmd_resp;
	struct agnic_adapter *adapter = netdev_priv(dev);

	msg_params.client_id = 0;
	msg_params.client_type = CDT_PF;
	msg_params.msg = NULL;    /* No msg params */
	msg_params.msg_len = 0;

	if (enable) {
		msg_params.cmd_code = CC_PF_ENABLE;
		msg_params.resp_msg = &cmd_resp;
		msg_params.resp_msg_len = sizeof(cmd_resp);
	} else {
		msg_params.cmd_code = CC_PF_DISABLE;
		msg_params.resp_msg = 0;    /* No response in disable */
		msg_params.resp_msg_len = 0;
	}

	ret = agnic_mgmt_command_send(adapter, &msg_params);
	if (ret)
		goto error;

	return 0;
error:
	/* TODO: Define configuration rollback at NIC level, then implement
	 * here.
	 * For now, disable critical operations.
	 */

	/* Mark interface as down */
	set_bit(AGNIC_DOWN, &adapter->state);

	agnic_dev_err("Failed to enable the hardware.\n");
	return ret;
}

/*
** agnic_tx_set_port_loopback - Set port to loopback mode
*/
static int agnic_tx_set_port_loopback(struct net_device *netdev, bool loopback)
{
	int ret;
	struct agnic_msg_params msg_params;
	struct agnic_mgmt_cmd_params cmd_params;
	struct agnic_mgmt_cmd_resp cmd_resp;
	struct agnic_adapter *adapter = netdev_priv(netdev);

	cmd_params.pf_set_loopback.loopback = loopback;
	msg_params.cmd_code = CC_PF_SET_LOOPBACK;
	msg_params.client_id = 0;
	msg_params.client_type = CDT_PF;
	msg_params.msg_len = sizeof(cmd_params);
	msg_params.msg = &cmd_params;

	msg_params.resp_msg = &cmd_resp;
	msg_params.resp_msg_len = sizeof(cmd_resp);

	ret = agnic_mgmt_command_send(adapter, &msg_params);
	if (ret)
		agnic_dev_err("Failed to configure loopback for hardware\n");

	return ret;
}

/*
** agnic_ioctl - Driver IOCTL entry.
*/
static int agnic_ioctl(struct net_device *netdev, struct ifreq *ifr, int cmd)
{
	pr_crit("%s - Not Implemented.\n", __func__);
	return -EOPNOTSUPP;
}


static int agnic_set_features(struct net_device *netdev, netdev_features_t features)
{
	int ret;
	netdev_features_t changed = netdev->features ^ features;

	if (changed & NETIF_F_LOOPBACK) {
		ret = agnic_tx_set_port_loopback(netdev, features & NETIF_F_LOOPBACK);
		if (unlikely(ret))
			return ret;
	}

	return 0;
}

/* Handle RX checksum offload */
void agnic_rx_csum(u32 flags, struct sk_buff *skb)
{
	u32 ipv4_l4_status = RXD_FLAGS_L3_L4_ERR_GET(flags);

	if (((flags & RXD_FLAGS_L3_IP4) &&
	    (ipv4_l4_status != DESC_ERR_IPV4_CHECKSUM)) ||
	    (flags & RXD_FLAGS_L3_IP6))
		if (((flags & RXD_FLAGS_L4_UDP) ||
		     (flags & RXD_FLAGS_L4_TCP)) &&
		     (ipv4_l4_status != DESC_ERR_L4_CHECKSUM)) {
			skb->ip_summed = CHECKSUM_UNNECESSARY;
			return;
		}

	skb->ip_summed = CHECKSUM_NONE;
}

/*
 * agnic_populate_skb_fields - Populate skb header fields from Rx descriptor.
 *
 * Currently, we only update the rx queue on which the packet was received.
 */
static inline void agnic_populate_skb_fields(struct agnic_ring *ring,
					     struct agnic_rx_desc *rx_desc,
					     struct sk_buff *skb)
{
	struct net_device *dev = ring->netdev;

	/* Set the queue from which the packet was received. */
	skb_record_rx_queue(skb, ring->q_idx);

	skb->protocol = eth_type_trans(skb, dev);

	if (likely(dev->features & NETIF_F_RXCSUM))
		agnic_rx_csum(rx_desc->flags, skb);
}

int agnic_net_add_vlan(struct net_device *dev, __be16 proto, u16 vid)
{
	int ret;
	struct agnic_msg_params msg_params;
	struct agnic_mgmt_cmd_params cmd_params;
	struct agnic_adapter *adapter = netdev_priv(dev);

	/*  ignore vlan0 while it already added in musdk interface
	 *  by the network sub system
	 */
	if (vid == 0)
		return 0;

	if (vid >= VLAN_N_VID)
		return -EINVAL;

	cmd_params.pf_vlan.vlan = vid;
	msg_params.cmd_code = CC_PF_ADD_VLAN;
	msg_params.client_id = 0;
	msg_params.client_type = CDT_PF;
	msg_params.msg = &cmd_params;
	msg_params.msg_len = sizeof(cmd_params);

	/* don't wait for a response */
	msg_params.resp_msg = NULL;
	msg_params.resp_msg_len = 0;

	ret = agnic_mgmt_command_send(adapter, &msg_params);
	if (ret)
		netdev_err(dev, "vlan add failed\n");

	return ret;
}

int agnic_net_kill_vlan(struct net_device *dev, __be16 proto, u16 vid)
{
	int ret;
	struct agnic_msg_params msg_params;
	struct agnic_mgmt_cmd_params cmd_params;
	struct agnic_adapter *adapter = netdev_priv(dev);

	if (vid == 0)
		return 0;

	if (vid >= VLAN_N_VID)
		return -EINVAL;

	cmd_params.pf_vlan.vlan  = vid;
	msg_params.cmd_code = CC_PF_REMOVE_VLAN;
	msg_params.client_id = 0;
	msg_params.client_type = CDT_PF;
	msg_params.msg = &cmd_params;
	msg_params.msg_len = sizeof(cmd_params);

	/* don't wait for a response */
	msg_params.resp_msg = NULL;
	msg_params.resp_msg_len = 0;

	ret = agnic_mgmt_command_send(adapter, &msg_params);
	if (ret)
		netdev_err(dev, "vlan remove failed\n");

	return ret;
}

static void agnic_get_tc_by_queue(int queue_index, int *tc, int *qid)
{
	/* for now, support only one tc */
	*tc = 0;
	*qid = queue_index;
}

int agnic_set_port_rate_limit(struct net_device *dev, int enable,
							  struct rate_limit_params *params)
{
	int ret;
	struct agnic_msg_params msg_params;
	struct agnic_mgmt_cmd_params cmd_params;
	struct agnic_mgmt_cmd_resp cmd_resp;
	struct agnic_adapter *adapter = netdev_priv(dev);

	/* a Mbps max-rate set for the port, a value of zero means disabled.
	 * default is disabled.
	 */
	memset(&cmd_params, 0, sizeof(cmd_params));
	if (enable) {
		cmd_params.pf_port_rate_limit.enable = true;
		cmd_params.pf_port_rate_limit.rate_limit.cir = params->cir;
		cmd_params.pf_port_rate_limit.rate_limit.cbs = params->cbs;
	} else
		cmd_params.pf_port_rate_limit.enable = false;

	agnic_debug(adapter, "Send CC_PF_PORT_RATE_LIMIT command.\n");
	msg_params.cmd_code = CC_PF_PORT_RATE_LIMIT;
	msg_params.client_id = 0;
	msg_params.client_type = CDT_PF;
	msg_params.msg = &cmd_params;
	msg_params.msg_len = sizeof(cmd_params);
	msg_params.resp_msg = &cmd_resp;
	msg_params.resp_msg_len = sizeof(cmd_resp);
	ret = agnic_mgmt_command_send(adapter, &msg_params);

	return ret;
}

int agnic_set_queue_rate_limit(struct net_device *dev, int tc, int qid, int enable,
							   struct rate_limit_params *params)
{
	int ret;
	struct agnic_msg_params msg_params;
	struct agnic_mgmt_cmd_params cmd_params;
	struct agnic_mgmt_cmd_resp cmd_resp;
	struct agnic_adapter *adapter = netdev_priv(dev);

	/* a Mbps max-rate set for the queue, a value of zero means disabled,
	 * default is disabled.
	 */
	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.pf_queue_rate_limit.qid = qid;
	cmd_params.pf_queue_rate_limit.tc  = tc;

	if (enable) {
		cmd_params.pf_queue_rate_limit.enable = true;
		cmd_params.pf_queue_rate_limit.rate_limit.cir = params->cir;
		cmd_params.pf_queue_rate_limit.rate_limit.cbs = params->cbs;
	} else
		cmd_params.pf_queue_rate_limit.enable = false;

	agnic_debug(adapter, "Send CC_PF_QUEUE_RATE_LIMIT command.\n");
	msg_params.cmd_code = CC_PF_QUEUE_RATE_LIMIT;
	msg_params.client_id = 0;
	msg_params.client_type = CDT_PF;
	msg_params.msg = &cmd_params;
	msg_params.msg_len = sizeof(cmd_params);
	msg_params.resp_msg = &cmd_resp;
	msg_params.resp_msg_len = sizeof(cmd_resp);
	ret = agnic_mgmt_command_send(adapter, &msg_params);

	return ret;
}

int agnic_set_tx_maxrate(struct net_device *dev, int queue_index, u32 maxrate)
{
	int ret, tc, qid;
	char enable = false;
	struct rate_limit_params params;

	/* maxrate zero is indication for rate limit disable */
	if (maxrate)
		enable = true;

	agnic_get_tc_by_queue(queue_index, &tc, &qid);
	params.cir = maxrate;
	params.cbs = RATE_LIMIT_BURST_SIZE;

	ret = agnic_set_queue_rate_limit(dev, tc, qid, enable, &params);
	if (unlikely(ret)) {
		netdev_err(dev, "failed to set ratelimit for queue %d\n", queue_index);
		return -EINVAL;
	}

	return 0;
}

/*
 * agnic_rx_handle_ring - Handle rx packets for a single ring.
 * Returns the number of processed rx packets.
 */
static int agnic_rx_handle_ring(struct agnic_q_vector *vector, struct agnic_ring *ring, int budget)
{
	struct agnic_adapter *adapter = vector->adapter;
	struct agnic_rx_desc *desc_list, *desc;
	struct agnic_ring *bp_ring;
	struct sk_buff *skb;
	struct agnic_bp_cookie *cookie;
	void *data;
	int cons_idx, prod_idx;
	int rx_pkts = 0;
	int rx_bytes = 0;
	int num_occupied, ret;
	int count = 0;

	prod_idx = readl(ring->producer_p);
	if (unlikely(AGNIC_RING_IS_EMPTY(prod_idx, ring->rx_cons_shadow)))
		return 0;

	if (unlikely(test_bit(AGNIC_DOWN, &adapter->state)))
		return 0;

	bp_ring = &adapter->bp_ring[ring->q_idx];
	desc_list = (struct agnic_rx_desc *)ring->desc;

	cons_idx = ring->rx_cons_shadow;
	num_occupied = AGNIC_RING_NUM_OCCUPIED(prod_idx, cons_idx, ring->count);
	budget = min(budget, num_occupied);

	while (count < budget) {
		desc = &desc_list[cons_idx];

		/* TODO: remove this validation once DMA reordering issue is cleaned */
		if (((desc->buffer_addr & AGNIC_PA_DEVICE_WATERMARK) == AGNIC_PA_DEVICE_WATERMARK) ||
		    (desc->cookie == AGNIC_COOKIE_DEVICE_WATERMARK) ||
		    (desc->cookie == AGNIC_COOKIE_DRIVER_WATERMARK)) {
			agnic_debug(adapter, "Illegal descriptor (PA:%llx, cookie:%llx @%d)!!!!\n",
				    desc->buffer_addr, desc->cookie, cons_idx);
			goto desc_error;
		}

		cookie = (struct agnic_bp_cookie *)desc->cookie;

		/* TODO: remove this validation once DMA reordering issue is cleaned */
		if (unlikely(cookie->bp_ring != bp_ring)) {
			agnic_debug(adapter, "Illegal bpool (%p@%d)!\n", cookie->bp_ring, cons_idx);
			/* Refill buffer pool and move to next descriptor */
			goto desc_error;
		}

		data = cookie->buff_virt_addr;
		prefetch(data + adapter->pkt_offset + desc->pkt_offset);

		skb = build_skb(data, bp_ring->bp_frag_sz > PAGE_SIZE ? 0 : bp_ring->bp_frag_sz);
		if (unlikely(!skb)) {
			agnic_dev_warn("Failed to alloc rx skb (%p@%d)!\n", skb, cons_idx);
desc_error:
			adapter->stats.rx_errors++;
			count++;
			/* Refill buffer pool */
			ret = agnic_bpool_refill_rx_buffs(adapter, bp_ring, bp_ring->bp_frag_sz, 1);
			if (unlikely(ret))
				agnic_dev_warn("Can't refill BPool\n");
			AGNIC_RING_PTR_INC(cons_idx, 1, ring->count);
			continue;
		}

		/* Unmap buffer after being accessed by HW. */
		dma_unmap_single(ring->dev, dma_unmap_addr(cookie, addr),
				 dma_unmap_len(cookie, len), DMA_FROM_DEVICE);

		/* Mark this BPool entry as invalid */
		cookie->buff_virt_addr = NULL;

		/* Check for rx error. */
		if (unlikely(desc->flags & RXD_FLAGS_RX_ERROR)) {
			agnic_dev_warn("Rx descriptor error (0x%08x@%d).\n", desc->flags, cons_idx);
			/* Free the SKB and go back to refill the buffer pool */
			dev_consume_skb_any(skb);
			goto desc_error;
		}

		skb_reserve(skb, adapter->pkt_offset + desc->pkt_offset);

#ifdef CONFIG_MV_NSS_ENABLE
		/* Accumulate bytes. */
		skb_put(skb, desc->byte_cnt - MV_NSS_METADATA_LEN);
#else
		skb_put(skb, desc->byte_cnt);
#endif

		rx_bytes += skb->len;

		/* fill skb fields out of the rx-descriptor. */
		agnic_populate_skb_fields(ring, desc, skb);

		/* TODO: remove this validation once DMA reordering issue is cleaned */
		desc->cookie = AGNIC_COOKIE_DRIVER_WATERMARK;

#ifdef CONFIG_MV_NSS_ENABLE
		/* Handle metadata if the buffer contains such */
		if (likely(desc->flags & RXD_FLAGS_MD_MODE))
			agnic_metadata_receive_hdl(skb);
#endif

#ifdef CONFIG_STUB_MODE
		if (loopback_mode == 3) {
			u8 offs = RXD_FLAGS_L3_OFFSET_GET(desc->flags);

			skb_push(skb, ETH_HLEN);
#ifdef CONFIG_MV_NSS_ENABLE
			if (desc->flags & RXD_FLAGS_MD_MODE)
				offs -= MV_NSS_METADATA_LEN;
#endif
			skb_set_network_header(skb, offs);
			if (likely((desc->flags >= RX_DESC_L3_TYPE_IPV4_NO_OPTS) &&
				(desc->flags <= RX_DESC_L3_TYPE_IPV6_EXT))) /* IP types */
				skb_set_transport_header(skb, offs +
					(RXD_FLAGS_IP_HDR_LEN_GET(desc->flags) * 4));
			skb->queue_mapping = ring->q_idx;
			agnic_net_xmit(skb, vector->adapter->netdev);
		} else if (loopback_mode == 4) {
			dev_kfree_skb_any(skb);
		} else
#endif /* CONFIG_STUB_MODE */

		/* Send packet up the stack... */
		/* TODO: Check if a call to skb_mark_napi_id() is needed.
		 * TODO: Call napi_gro_receive() once we support csum offload.
		 */
		netif_receive_skb(skb);

		AGNIC_RING_PTR_INC(cons_idx, 1, ring->count);

		rx_pkts++;
		count++;
	}

	/* Update ring pointers. */
	ring->rx_cons_shadow = cons_idx;
	writel(cons_idx, ring->consumer_p);

	vector->rx.total_packets += rx_pkts;
	vector->rx.total_bytes += rx_bytes;

	ring->total_packets += rx_pkts;

	adapter->stats.rx_packets += rx_pkts;
	adapter->stats.rx_bytes += rx_bytes;

	/* Refill buffer pool */
	ret = agnic_bpool_refill_rx_buffs(adapter, bp_ring, bp_ring->bp_frag_sz, rx_pkts);
	if (unlikely(ret))
		agnic_dev_warn("Can't refill BPool\n");

	return count;
}

/*
 * agnic_napi_poll - napi poll callback
 * Returns 0 in case no more Rx descriptors are still pending.
*/
static int agnic_napi_poll(struct napi_struct *napi, int budget)
{
	struct agnic_q_vector *q_vector = container_of(napi, struct agnic_q_vector, napi);
	struct agnic_adapter *adapter = q_vector->adapter;
	struct agnic_ring *ring;
	int ring_budget, processed;
	int rx_completed = 1;
	int done = 0, irq;

	/* If interface is down (probably due to if_down request, then
	 * skip packet handling and just do napi_done actions.
	 */
	if (unlikely(test_bit(AGNIC_DOWN, &adapter->state)))
		goto napi_done;

	/* Split the budget between all rings (belonging to the same q-vector). */
	if (q_vector->rx.count > 1)
		ring_budget = budget / q_vector->rx.count;
	else
		ring_budget = budget;

	if (!ring_budget)
		ring_budget = 1;

	agnic_for_each_ring(ring, q_vector->rx) {
#ifdef CONFIG_STUB_MODE
		if (loopback_mode == 1 || loopback_mode == 2)
			processed = agnic_stub_loopback(q_vector, ring, ring_budget);
		else
#endif /* CONFIG_STUB_MODE */
		/* Perform Rx work on single ring. */
		processed = agnic_rx_handle_ring(q_vector, ring, ring_budget);

		/* Accumulate work. */
		done += processed;

		/* If number of processed packets is smaller than what we
		 * required, then this ring is now empty (i.e. work completed).
		 */
		rx_completed &= (processed < ring_budget);
	}

	/* We still have packets in (one or more) rx rings, don't stop napi
	 * scheduling.
	 */
	if (!rx_completed)
		return budget;

napi_done:
	/* No more rx packets, stop napi scheduling. */
	napi_complete_done(napi, done);

	/* Enable Rx interrupts. */
	if (adapter->flags & AGNIC_FLAG_RX_MSIX_ENABLED) {
		irq = q_vector->rx_msix_entry->vector;
		enable_irq(irq);
	}

	return 0;
}


/*
 * agnic_init_q_indices - Initialize Q pointers page.
 */
static int agnic_init_q_indeces(struct agnic_adapter *adapter)
{
	int size;

	adapter->ring_indices_arr_len = AGNIC_MAX_QUEUES;
	size = adapter->ring_indices_arr_len * sizeof(u32);
	if ((adapter->nic_cfg_base->dev_use_size + size) > AGNIC_CONFIG_BAR_SIZE) {
		agnic_dev_err("not enough memory on BAR for rings indicex pointers!\n");
		return -ENOMEM;
	}
	adapter->ring_indices_arr =
		(void *)((u64)adapter->nic_cfg_base + adapter->nic_cfg_base->dev_use_size);
	/* in case the indices are allocated on the BAR, we would like to send only the offset
	 * from the bar base address
	 */
	adapter->ring_indices_arr_phys = adapter->nic_cfg_base->dev_use_size;
	/* Set all entries to "free" state. */
	memset(adapter->ring_indices_arr, 0xFF, size);

	return 0;
}

/*
 * agnic_terminate_q_indeces - Destroy Q pointers page.
 */
static int agnic_terminate_q_indeces(struct agnic_adapter *adapter)
{
	int size;

	size = adapter->ring_indices_arr_len * sizeof(u32);
	dma_free_coherent(adapter->dev, size, adapter->ring_indices_arr,
			adapter->ring_indices_arr_phys);
	adapter->ring_indices_arr = NULL;

	return 0;
}


/*
** agnic_get_q_idx - allocate a free entry for queue control pointers.
*/
static int agnic_get_q_idx(struct agnic_adapter *adapter)
{
	int i;

	for (i = 0; i < adapter->ring_indices_arr_len; i++)
		if (adapter->ring_indices_arr[i] == 0xFFFFFFFF) {
			adapter->ring_indices_arr[i] = 0;
			return i;
		}

	/* All entries are occupied, not likely to happen. */
	agnic_dev_err("All Ring indeces are occupied (%d entries).\n",
			adapter->ring_indices_arr_len);
	return -ENODEV;
}

/*
** agnic_put_q_idx - Free a previous allocated entry for queue control pointers.
*/
static void agnic_put_q_idx(struct agnic_adapter *adapter, u32 index)
{
	adapter->ring_indices_arr[index] = 0xFFFFFFFF;
}


/*
** agnic_alloc_ring_resources - allocate Rx/Tx/cmd/Notif/... ring resources (Descriptors)
*/
static int agnic_alloc_ring_resources(struct agnic_ring *ring, int desc_size, int ring_len, bool alloc_b_info)
{
	struct agnic_adapter *adapter = ring->adapter;
	int err = -ENOMEM, size;

	ring->count = ring_len;
	/* For cmd queues, the cookie holds data regarding the command. Hence, it should contain
	 * enough entries so it can serve all commands.
	 */
	ring->cookie_count = (ring->type == agnic_cmd_ring) ?
				max(ring->count * 2, (u32)AGNIC_CMD_Q_MAX_COOKIE_LEN) : ring->count;
	ring->cookie_list = NULL;
	if (alloc_b_info) {
		size = sizeof(struct agnic_cookie) * ring->cookie_count;
		ring->cookie_list = kzalloc(size, GFP_KERNEL);
		if (ring->cookie_list == NULL) {

			agnic_dev_err("Failed to allocate %d Bytes for cookie_list.\n", size);
			goto err;
		}
	}

	/* Allocate a DMA'able queue memory. */
	size = ring->count * desc_size;
	ring->desc = dma_alloc_coherent(adapter->dev, size, &ring->dma,
			GFP_KERNEL);
	if (!ring->desc) {
		agnic_dev_err("Failed to allocate %d Bytes for descriptors ring.", size);
		goto err;
	}

	/* Get an entry in the queue indeces memory, to be pointed to by the
	 * consumer / producer pointer.
	 */
	err = agnic_get_q_idx(ring->adapter);
	if (err < 0) {
		pr_err("Unable to allocate entry for ring control pointers.\n");
		goto err;
	}
	ring->prod_idx = (u16)err;
	ring->producer_p = ring->adapter->ring_indices_arr + ring->prod_idx;
	err = agnic_get_q_idx(ring->adapter);
	if (err < 0) {
		pr_err("Unable to allocate entry for ring control pointers.\n");
		goto err;
	}
	ring->cons_idx = (u16)err;
	ring->consumer_p = ring->adapter->ring_indices_arr + ring->cons_idx;

	return 0;
err:
	ring->producer_p = NULL;
	ring->consumer_p = NULL;
	if (ring->cons_idx)
		agnic_put_q_idx(ring->adapter, ring->cons_idx);
	if (ring->prod_idx)
		agnic_put_q_idx(ring->adapter, ring->prod_idx);
	if (ring->desc)
		dma_free_coherent(adapter->dev, size, ring->desc, ring->dma);
	kfree(ring->cookie_list);
	ring->cookie_list = NULL;
	return err;
}


/*
** agnic_free_ring_resources - Free Rx/Tx/cmd/Notif/... ring resources (Descriptors)
*/
static int agnic_free_ring_resources(struct agnic_ring *ring)
{
	struct agnic_adapter *adapter = ring->adapter;
	int size;

	kfree(ring->cookie_list);
	ring->cookie_list = NULL;

	size = ring->count * ring->desc_size;
	dma_free_coherent(adapter->dev, size, ring->desc, ring->dma);
	ring->desc = NULL;

	agnic_put_q_idx(adapter, ring->prod_idx);
	agnic_put_q_idx(adapter, ring->cons_idx);

	return 0;
}

static void agnic_set_ring_defaults(struct agnic_adapter *adapter, struct agnic_ring *ring, enum agnic_ring_type type)
{
	ring->adapter = adapter;
	ring->netdev = adapter->netdev;
	ring->dev = adapter->dev;
	ring->rx_cons_shadow = 0;
	ring->type = type;
	ring->total_packets = 0;
}

/**
 * agnic_link_up - update netif_carrier status and
 *                 print link up message
 **/
static int agnic_link_up(struct agnic_adapter *adapter)
{
	struct net_device *netdev = adapter->netdev;

	/* only continue if link was previously down */
	if (netif_carrier_ok(netdev))
		return 0;

	if (unlikely(test_bit(AGNIC_DOWN, &adapter->state))) {
		pr_err("agnic is down, cannot enable carrier\n");
		return -1;
	}

#ifdef CONFIG_STUB_MODE
	/* in loopback mode, keep the carrier state to off in order to */
	/* avoid agnic xmit operation, invoked by the Linux stack      */
	if (loopback_mode) {
		agnic_dev_info("AGNIC operates in ECHO Mode (loopback=%d). Carrier off\n", loopback_mode);
		return 0;
	}
#endif

	netif_carrier_on(netdev);
	agnic_dev_info("AGNIC Link is Up\n");

	return 0;
}

/**
 * agnic_link_down - update netif_carrier status and
 *                   print link down message
 **/
static int agnic_link_down(struct agnic_adapter *adapter)
{
	struct net_device *netdev = adapter->netdev;

	/* only continue if link was up previously */
	if (!netif_carrier_ok(netdev))
		return 0;

	agnic_dev_info("AGNIC Link is Down\n");
	netif_carrier_off(netdev);

	return 0;
}

/**
 * agnic_keep_alive - handle keep alive process
 **/
static int agnic_keep_alive(struct agnic_adapter *adapter)
{
	/* Reset the watchdog */
	if (adapter->keep_alive_initialized)
		mod_timer(&adapter->keep_alive_wd, jiffies + adapter->keep_alive_timeout);

	return 0;
}

int agnic_mgmt_notif_process(struct agnic_adapter *adapter, u16 cmd_code, void *msg, u16 len)
{
	struct agnic_mgmt_notification *resp = (struct agnic_mgmt_notification *)msg;
	int ret;

	agnic_debug(adapter, "Received notification id %d\n", cmd_code);

	switch (cmd_code) {

	case NC_PF_LINK_CHANGE:
		if (resp->link_status)
			ret = agnic_link_up(adapter);
		else
			ret = agnic_link_down(adapter);

		if (ret) {
			agnic_dev_err("Failed to execute link change (%d)\n", ret);

			return ret;
		}
		break;
	case NC_PF_KEEP_ALIVE:
		ret = agnic_keep_alive(adapter);
		if (ret) {
			agnic_dev_err("Failed to execute keep alive (%d)\n", ret);

			return ret;
		}
		break;
	default:
		/* Unknown command code */
		agnic_dev_err("Unknown command code %d!! Unable to process command.\n", cmd_code);
		return -EOPNOTSUPP;
	}

	return 0;
}

static void agnic_irqpoll_timer_handler(struct timer_list *t)
{
	struct agnic_pcpu_timer_irqpoll *pcpu_timer =
		from_timer(pcpu_timer, t, list);
	struct agnic_q_vector *q_vector = pcpu_timer->data;
	struct agnic_adapter *adapter = q_vector->adapter;
	int notif_limit = NOTIFICATION_HANDLE_LIMIT;
	int ret;

	/* Check if mgmt queues should be polled AND current running CPU
	 * is the one that is assgnied to handle handle them.
	 */
	if ((adapter->flags & AGNIC_FLAG_MGMT_POLL) &&
	    (smp_processor_id() == adapter->mng_cpu)) {
		/* handle async notifications up to a certain limit */
		do {
			ret = agnic_mgmt_notif_handle(adapter);
			if (ret) {
				if (ret != -ENOMSG)
					agnic_dev_err("Failed in notification message handling (%d).\n", ret);
				break;
			}

			notif_limit--;
		} while (notif_limit);
	}

	/* If Rx interrupts are not enabled (i.e. polling mode is used),
	 * schedule napi to handle rx rings (as for interrupt mode, it
	 * is done in the interrupt handler)
	 */
	if (adapter->flags & AGNIC_FLAG_RX_POLL)
		napi_schedule(&q_vector->napi);

	/* If interrupts are not enabled for either rx or mgmt,
	 * reset the timer (of current CPU)
	 */
	if (adapter->flags & AGNIC_FLAG_IRQPOLL) {
		/* Re-trigger timer timer */
		mod_timer(&pcpu_timer->list,
			  jiffies + adapter->poll_timer_rate);
	}
}

/*
 * agnic_del_irqpoll_timer - Stop timers and free the timers list.
 */
static void agnic_del_irqpoll_timer(struct agnic_adapter *adapter)
{
	struct timer_list *timer_pcpu;
	int cpu;

	if (!adapter->irqpoll_timer_pcpu)
		return;

	/* Delete the timers (per CPU) */
	for_each_present_cpu(cpu) {
		timer_pcpu = &per_cpu_ptr(adapter->irqpoll_timer_pcpu,
					  cpu)->list;
		del_timer_sync(timer_pcpu);
	}

	/* Free timer list */
	free_percpu(adapter->irqpoll_timer_pcpu);
}

/*
 * agnic_irqpoll_start - IRQ polling timer initialization. This is used as a
 * temporary workaround until we have interrupts support.
 * TODO: Remove this function once interrupts are supported.
 */
static int agnic_irqpoll_start(struct agnic_adapter *adapter)
{
	int vector;
	int cpu;

	struct agnic_pcpu_timer_irqpoll *pcpu_timer;

	if (!(adapter->flags & AGNIC_FLAG_IRQPOLL))
		return 0; /* IRQ polling is not enabled so exit */

	if (!adapter->irqpoll_initialized) {
		/* Unset the cpu id that will poll the Mng queues */
		adapter->mng_cpu = -1;

		/* Setup polling mechanism for notification queues handling. */
		if (!adapter->irqpoll_timer_pcpu) {
			/* Alloc timer list */
			adapter->irqpoll_timer_pcpu =
				alloc_percpu(struct agnic_pcpu_timer_irqpoll);

			if (!adapter->irqpoll_timer_pcpu) {
				pr_err("failed to allocate percpu Timer resources\n");
				return -ENOMEM;
			}
		}

		/* Assumption is that q_vectors are mapped to CPUs 1 to 1.
		 * Hence, we can iterate the q_vectors and initialize the
		 * timers according to the CPU mask.
		 */
		for (vector = 0; vector < adapter->num_q_vectors; vector++) {
			struct agnic_q_vector *q_vector = adapter->q_vectors[vector];

			/* Read the cpu which this q_vector is assigned to */
			cpu = cpumask_first(&q_vector->affinity_mask);

			/* Configure timer */
			pcpu_timer = per_cpu_ptr(adapter->irqpoll_timer_pcpu,
						 cpu);
			pcpu_timer->data = q_vector;
			timer_setup(&pcpu_timer->list,
				    &agnic_irqpoll_timer_handler, 0);

			/* Trigger the timer immediately on masked cpu
			 * Note: add_timer_on is called to make sure the timer
			 * is trigger on the CPU it was assigned to.
			 * Otherwise (since it's Linux timer and not hrtimer) the
			 * timer handler may be called on any of the CPUs and we want
			 * to avoid the resource sharing (this is why we have q_vector
			 * (and its rings) per CPU)
			 */
			pcpu_timer->list.expires = jiffies;
			add_timer_on(&pcpu_timer->list, cpu);

			if (adapter->mng_cpu == -1) {
				pr_info("cpu %d will poll the Mng Qs\n", cpu);
				adapter->mng_cpu = cpu;
			}

			/* If RX interrupts are enabled, it's enough to assign
			 * a timer only to a single cpu to handle the mgmt messages
			 */
			if (adapter->flags & AGNIC_FLAG_RX_MSIX_ENABLED)
				break;
		}

		/* Mark irq polling as initialized */
		adapter->irqpoll_initialized = true;
	}

	/* If interrupts are enabled, use low refresh rate (~64ms) as the timer
	 * is used for mng only.
	 * Otherwise, since the timer is used for both data path and mng, use high
	 * refresh rate.
	 */
	if (adapter->flags & AGNIC_FLAG_RX_MSIX_ENABLED)
		adapter->poll_timer_rate = HZ / 16;
	else
		adapter->poll_timer_rate = 0;

	return 0;
}

/*
 * agnic_irqpoll_stop - Stop irq polling if not required by anyone.
 * TODO: Remove this function once interrupts are supported.
 */
static void agnic_irqpoll_stop(struct agnic_adapter *adapter)
{
	if (adapter->irqpoll_initialized) {
		/* This function can be called from different adapter setup stages,
		 * setup stages, and thus it must check if all irqpoll users have
		 * disabled polling mode.
		 */
		if (!(adapter->flags & AGNIC_FLAG_IRQPOLL)) {
			agnic_del_irqpoll_timer(adapter);

			adapter->irqpoll_initialized = false;
		}
	}
}

void agnic_wd_work_handler(struct work_struct *work)
{
	struct agnic_adapter *adapter;

	adapter = container_of(work, struct agnic_adapter, wd_work);

	/* Stop the interface (ifdown) */
	agnic_net_stop(adapter->netdev);
}

/*
 * agnic_keep_alive_wd_timeout_handler - Keep-alive watchdog handler.
 *			It is called in case of a timeout (i.e no keep-alive
 *			message was sent from the app/fw).
 *
 */
static void agnic_keep_alive_wd_timeout_handler(struct timer_list *t)
{
	struct agnic_adapter *adapter = from_timer(adapter, t, keep_alive_wd);


	pr_err("Keep alive watchdog timeout!!\n");

	/* scehdule the work queue to handle the net_stop flow */
	schedule_work(&adapter->wd_work);

#ifdef CONFIG_MV_NSS_ENABLE
	if (agnic_mgmt_notify_down(adapter))
		pr_err("Failed to notifiy keep alive timeout\n");
#endif
}

/*
 * agnic_keep_alive_wd_start - Start the keep-alive watchdog which responsible to notify
 *			if the FW/App is no longer alive.
 *
 */
static int agnic_keep_alive_wd_start(struct agnic_adapter *adapter)
{
	/* Check if keep-alive feature is enabled */
	if (!(adapter->feature_enable_mask & AGNIC_FEATURES_KEEP_ALIVE_EN))
		return 0;

	if (adapter->keep_alive_initialized)
		return 0;

	/* Configure timer */
	timer_setup(&adapter->keep_alive_wd,
		    &agnic_keep_alive_wd_timeout_handler, 0);

	/* Set refresh rate to 10 seconds */
	adapter->keep_alive_timeout = msecs_to_jiffies(AGNIC_KEEP_ALIVE_TIMEOUT * 1000);

	/* add_timer_on is called to make sure the watchdog will
	 * be triggered on the CPU which runs the management process.
	 */
	adapter->keep_alive_wd.expires = jiffies + adapter->keep_alive_timeout;
	add_timer_on(&adapter->keep_alive_wd, adapter->mng_cpu);

	/* Init work-queue which will call net_stop in case of wd event.
	 * Note that if down (agnic_net_stop) cannot be called from
	 * wd handler as the handler is softirq context and if down
	 * should be called from process context (due to sleep and etc.)
	 */
	INIT_WORK(&adapter->wd_work, agnic_wd_work_handler);

	/* Mark keep-alive watchdog as initialized */
	adapter->keep_alive_initialized = true;

	return 0;
}

/*
 * agnic_keep_alive_wd_stop - Stop the keep-alive watchdog.
 *
 */
static void agnic_keep_alive_wd_stop(struct agnic_adapter *adapter)
{
	if (!adapter->keep_alive_initialized)
		return;

	/* Free timer */
	del_timer(&adapter->keep_alive_wd);

	adapter->keep_alive_initialized = false;
}

/*
** agnic_setup_mgmt_rings - Allocate cmd and notif rings.
*/
static int agnic_setup_mgmt_rings(struct agnic_adapter *adapter)
{
	struct agnic_ring *ring;
	int ret;

	/* Command Ring. */
	ring = &adapter->cmd_ring;
	agnic_set_ring_defaults(adapter, ring, agnic_cmd_ring);
	ret = agnic_alloc_ring_resources(ring, sizeof(struct agnic_cmd_desc),
			AGNIC_CMD_Q_LEN, true);
	if (ret) {
		agnic_dev_err("Failed to allocate command Queue.\n");
		goto cmd_err;
	}

	/* Notification Queue. */
	ring = &adapter->notif_ring;
	agnic_set_ring_defaults(adapter, ring, agnic_notif_ring);
	ret = agnic_alloc_ring_resources(ring, sizeof(struct agnic_cmd_desc),
			AGNIC_NOTIF_Q_LEN, false);
	if (ret) {
		agnic_dev_err("Failed to allocate NOTIF Queue.\n");
		goto notif_err;
	}

	/* Now set the MGMT queues pointers in HW. */
	ret = agnic_mgmt_set_mgmt_queues(adapter);
	if (ret)
		goto q_setup_err;

	/* We still don't have interrupts enabled at this stage, work in
	 * polling mode.
	 */
	adapter->flags |= AGNIC_FLAG_MGMT_POLL;

	/* Wait-queue for command issuers. */
	init_waitqueue_head(&adapter->mgmt_wait_q);

	spin_lock_init(&adapter->mgmt_lock);

	return 0;
q_setup_err:
	agnic_free_ring_resources(&adapter->notif_ring);
notif_err:
	agnic_free_ring_resources(&adapter->cmd_ring);
cmd_err:
	return ret;

}

/*
** agnic_free_mgmt_rings - Allocate cmd and notif rings.
*/
static int agnic_free_mgmt_rings(struct agnic_adapter *adapter)
{
	struct agnic_ring *ring;

	/* Delete managemet timer */
	adapter->flags &= ~AGNIC_FLAG_MGMT_POLL;

	/* Management Ring. */
	ring = &adapter->cmd_ring;
	agnic_free_ring_resources(ring);

	/* Notification Queue. */
	ring = &adapter->notif_ring;
	agnic_free_ring_resources(ring);

	return 0;
}


/*
** agnic_alloc_data_rings - Initialize all Rx / Tx Queues resources.
*/
static int agnic_alloc_data_rings(struct agnic_adapter *adapter, enum agnic_ring_type type)
{
	int i, err = 0;
	int desc_size, num_rings, ring_len;
	struct agnic_ring **ring_ptr;
	bool alloc_cookies;
	char *name;

	if (type == agnic_rx_ring) {
		name = "Rx";
		desc_size = sizeof(struct agnic_rx_desc);
		ring_ptr = adapter->rx_ring;
		num_rings = adapter->num_rx_queues;
		ring_len = adapter->rx_ring_size;
		alloc_cookies = false;
	} else {
		name = "Tx";
		desc_size = sizeof(struct agnic_tx_desc);
		ring_ptr  = adapter->tx_ring;
		num_rings = adapter->num_tx_queues;
		ring_len = adapter->tx_ring_size;
		alloc_cookies = true;
	}

	for (i = 0; i < num_rings; i++) {
		agnic_set_ring_defaults(adapter, ring_ptr[i], type);
		err = agnic_alloc_ring_resources(ring_ptr[i], desc_size, ring_len, alloc_cookies);
		if (!err)
			continue;

		agnic_err(probe, "Allocation for %s Queue %u failed\n", name, i);
		goto err;
	}

	return 0;
err:
	/* Free all allocated rings */
	while (i--)
		agnic_free_ring_resources(ring_ptr[i]);
	return err;
}

/*
** agnic_free_data_rings - Free Rx/Tx rings Resources for All Queues
*/
static void agnic_free_data_rings(struct agnic_adapter *adapter, enum agnic_ring_type type)
{
	struct agnic_ring **ring_ptr;
	int num_rings;
	int i;

	if (type == agnic_rx_ring) {
		ring_ptr = adapter->rx_ring;
		num_rings = adapter->num_rx_queues;
	} else {
		ring_ptr  = adapter->tx_ring;
		num_rings = adapter->num_tx_queues;
	}

	for (i = 0; i < num_rings; i++)
		if (ring_ptr[i]->desc)
			agnic_free_ring_resources(ring_ptr[i]);
}

static void *agnic_bp_alloc_buf(int buf_sz)
{
	/* Alloc buffer's memory. */
	if (likely(buf_sz <= PAGE_SIZE))
		return netdev_alloc_frag(buf_sz);
	else
		return kmalloc(buf_sz, GFP_ATOMIC);
}

static void agnic_bp_free_buf(void *buff, int buf_sz)
{
	if (likely(buf_sz <= PAGE_SIZE))
		skb_free_frag(buff);
	else
		kfree(buff);
}

/*
 * agnic_bpool_alloc_entry - Allocate a new bpool entry.
 */
static inline int agnic_bpool_alloc_entry(struct agnic_adapter *adapter, struct agnic_bpool_desc *desc,
					  struct agnic_bp_cookie *cookie, int buf_sz)
{
	void *buf;
	u64 addr;

	/* Alloc buffer's memory. */
	buf = agnic_bp_alloc_buf(buf_sz);
	if (buf == NULL) {
		agnic_dev_err("Failed to allocate B-Pool buffer (%d Bytes).\n", buf_sz);
		return -ENOMEM;
	}

	addr = dma_map_single(adapter->dev, buf, buf_sz, DMA_FROM_DEVICE);
	if (unlikely(dma_mapping_error(adapter->dev, addr))) {
		agnic_dev_err("Failed to map B-Pool buffer (0x%p / %d).\n", buf, buf_sz);
		agnic_bp_free_buf(buf, buf_sz);
		return -ENOMEM;
	}

	/* Map into physical address. */
	desc->buff_addr_phys = addr + adapter->pkt_offset;
#ifdef CONFIG_MV_NSS_ENABLE
	desc->buff_addr_phys -= MV_NSS_METADATA_LEN;
#endif

	/* Set respective fields in bp cookie. */
	dma_unmap_len_set(cookie, len, buf_sz);
	dma_unmap_addr_set(cookie, addr, addr);

	cookie->buff_virt_addr = buf;

	/* The cookie holds buffer information (virt & phys address, bp-ring). */
	desc->buff_cookie = (u64)cookie;

	return 0;
}

/*
 * agnic_bpool_free_entry - Unmap and Free BPool buffer.
 */
static void agnic_bpool_free_entry(struct agnic_adapter *adapter, struct agnic_bpool_desc *desc, int buf_sz)
{
	struct agnic_bp_cookie *bp_cookie;

	bp_cookie = (struct agnic_bp_cookie *)desc->buff_cookie;
	if (bp_cookie->buff_virt_addr == NULL)
		return;

	dma_unmap_single(adapter->dev, desc->buff_addr_phys, buf_sz, DMA_FROM_DEVICE);
	agnic_bp_free_buf(bp_cookie->buff_virt_addr, buf_sz);
}

/*
 * agnic_alloc_bp_ring_buffers - Allocate a single BPool ring buffers.
 * For now, we support only a single buffer in each bpool descriptor.
 * TODO: Support multiple BP cookies per BP descriptor.
 */
static int agnic_alloc_bp_ring_buffers(struct agnic_ring *bp_ring, int buf_sz)
{
	struct agnic_bpool_desc *desc_list;
	struct agnic_bp_cookie *cookie_list, *bp_cookie;
	int buf, ret = 0;

	desc_list = bp_ring->desc;
	cookie_list = (struct agnic_bp_cookie *)bp_ring->cookie_list;
	for (buf = 0; buf < (AGNIC_BPOOL_SIZE - 1); buf++) {
		bp_cookie = &cookie_list[buf];
		ret = agnic_bpool_alloc_entry(bp_ring->adapter, &desc_list[buf], bp_cookie, buf_sz);
		if (ret)
			break;
		cookie_list[buf].bp_ring = bp_ring;
	}
	if (ret) {
		while (buf > 0) {
			agnic_bpool_free_entry(bp_ring->adapter, &desc_list[buf - 1], buf_sz);
			buf--;
		}
	} else {
		/* The last entry should be marked as NULL as it will be the first
		 * one to be re-filled with a new buffer ,after Rx (since the
		 * producer is always one lag behind the consumer after init)
		 */
		desc_list[buf].buff_cookie = (u64)&cookie_list[buf];
		cookie_list[buf].buff_virt_addr = NULL;
	}

	return ret;
}

/*
 * agnic_free_bp_ring_buffers - Free a single BPool ring buffers.
 */
static void agnic_free_bp_ring_buffers(struct agnic_ring *ring, int buf_sz)
{
	struct agnic_bpool_desc *desc_list;
	int buf;

	desc_list = ring->desc;

	/* If desc_list is NULL it indicates that it was released
	 * previously (probably due to rollback process after error)
	 */
	if (desc_list == NULL)
		return;

	for (buf = 0; buf < AGNIC_BPOOL_SIZE; buf++)
		agnic_bpool_free_entry(ring->adapter, &desc_list[buf], buf_sz);
}


/*
 * agnic_alloc_bpools_buffers - Allocate buffers and enqueue to B-pool.
 * The function handled all Rx B-pools in the system.
 */
static int agnic_alloc_bpools_buffers(struct agnic_adapter *adapter)
{
	struct agnic_ring *ring;
	int i, buf_sz, ret = 0;

	for (i = 0; i < adapter->num_rx_queues; i++) {
		ring = &adapter->bp_ring[i];
		buf_sz = ring->bp_frag_sz;
		agnic_debug(adapter, "Alloc buffers for %dB pool.\n", buf_sz);
		/* Allocate the buffers. */
		ret = agnic_alloc_bp_ring_buffers(ring, buf_sz);
		if (ret)
			break;

		/* Set the consumer / producer initial values. */
		writel(0x0, ring->consumer_p);
		ring->tx_prod_shadow = AGNIC_BPOOL_SIZE - 1;
		writel(ring->tx_prod_shadow, ring->producer_p);
	}

	if (ret) {
		while (i > 0) {
			ring = &adapter->bp_ring[i - 1];
			buf_sz = ring->bp_frag_sz;
			agnic_free_bp_ring_buffers(ring, buf_sz);
			i--;
		}
	}

	return ret;
}


/*
 * agnic_free_bpools_buffers - Dequeue all buffers from all B-pools and free the
 * buffer's memory.
 */
static void agnic_free_bpools_buffers(struct agnic_adapter *adapter)
{
	int i;

	for (i = 0; i < adapter->num_rx_queues; i++)
		agnic_free_bp_ring_buffers(&adapter->bp_ring[i], adapter->bp_ring[i].bp_frag_sz);
}

/*
 * agnic_bpool_refill_rx_buffs - Allocate and push rx buffers into the given bpool.
 */
static inline int agnic_bpool_refill_rx_buffs(struct agnic_adapter *adapter, struct agnic_ring *bp_ring,
					      int buf_sz, int num)
{
	struct agnic_bpool_desc *desc_list;
	struct agnic_bp_cookie *cookie_list, *bp_cookie;
	int prod_idx, ret, i, num_free;

	prod_idx = bp_ring->tx_prod_shadow;

	/* Some sanity checks. Code should never enter this condition.
	 * Added for checking corner issues while in development cycle.
	 * May be removed in future releases.
	 */
	num_free = AGNIC_RING_FREE(prod_idx, readl(bp_ring->consumer_p), bp_ring->count);
	if (unlikely(num > num_free)) {
		agnic_dev_warn("BUG: Trying to refill more buffers (%d) than allowed (%d).\n",
				num, num_free);
		num = num_free;
	}

	desc_list = (struct agnic_bpool_desc *)bp_ring->desc;
	cookie_list = (struct agnic_bp_cookie *)bp_ring->cookie_list;

	for (i = 0; i < num; i++) {
		bp_cookie = &cookie_list[prod_idx];
		ret = agnic_bpool_alloc_entry(adapter, &desc_list[prod_idx], bp_cookie, buf_sz);
		if (unlikely(ret))
			break;
		bp_cookie->bp_ring = bp_ring;
		AGNIC_RING_PTR_INC(prod_idx, 1, bp_ring->count);
	}

	/* Update ring pointers. */
	bp_ring->tx_prod_shadow = prod_idx;
	writel(prod_idx, bp_ring->producer_p);

	return ret;
}

/*
 * agnic_alloc_bp_rings - Allocate and fill NIC's Rx buffers pool(s).
 */
static int agnic_alloc_bp_rings(struct agnic_adapter *adapter)
{
	int bpool_buf_size[] = AGNIC_BPOOLS_BUFF_SZ_LIST;
	int ret, i;

#ifdef CONFIG_MV_NSS_ENABLE
	adapter->pkt_offset = MAGNIC_NSS_HEADROOM_SIZE;
#else
	adapter->pkt_offset = AGNIC_DEFAULT_SKB_PAD;
#endif

	for (i = 0; i < adapter->num_rx_queues; i++) {
		agnic_set_ring_defaults(adapter, &adapter->bp_ring[i], agnic_bpool_ring);
		ret = agnic_alloc_ring_resources(&(adapter->bp_ring[i]), sizeof(struct agnic_bpool_desc),
					AGNIC_BPOOL_SIZE, true);
		if (ret) {
			agnic_dev_err("Failed to allocate buffers pool %d.\n", i);
			break;
		}

		adapter->bp_ring[i].bp_buff_sz = bpool_buf_size[i];
		adapter->bp_ring[i].bp_frag_sz = SKB_DATA_ALIGN(adapter->bp_ring[i].bp_buff_sz +
				adapter->pkt_offset) + SKB_DATA_ALIGN(sizeof(struct skb_shared_info));
	}

	if (ret) {
		while (i) {
			agnic_free_ring_resources(&adapter->bp_ring[i - 1]);
			i--;
		}
	}

	return ret;
}


/*
 * agnic_free_bp_rings - Empty and free all Rx buffers BP rings.
 */
static void agnic_free_bp_rings(struct agnic_adapter *adapter)
{
	int i;

	/* Free rings memory. */
	for (i = 0; i < adapter->num_rx_queues; i++)
		agnic_free_ring_resources(&adapter->bp_ring[i]);
}

/*
 * Returns true if the device is already scheduled for polling.
 */
static inline int napi_is_scheduled(struct napi_struct *napi)
{
	return test_bit(NAPI_STATE_SCHED, &napi->state);
}

static irqreturn_t agnic_rx_irq_handler(int irq, void *data)
{
	struct agnic_q_vector *q_vector = data;
	struct agnic_adapter *adapter = q_vector->adapter;

	/* If interface is down, ignore this interrupt */
	if (unlikely(test_bit(AGNIC_DOWN, &adapter->state))) {
		agnic_dev_info("interface is down; nothing to do\n");
		return IRQ_HANDLED;
	}

	/* Disable Rx interrupts */
	disable_irq_nosync(irq);

	/* Schedule napi to handle rx hedule napi */
	napi_schedule(&q_vector->napi);

	return IRQ_HANDLED;
}

static irqreturn_t agnic_tx_done_handler(int irq, void *data)
{
	struct agnic_q_vector *q_vector = data;
	struct agnic_adapter *adapter = q_vector->adapter;

	/* If interface is down, ignore this interrupt */
	if (unlikely(test_bit(AGNIC_DOWN, &adapter->state))) {
		agnic_dev_info("interface is down; nothing to do\n");
		return IRQ_HANDLED;
	}

	/* Set tx_done state to 'scheduled' */
	q_vector->txdone_timer_scheduled = true;

	/* Disable the (Tx) interrupt */
	disable_irq_nosync(irq);

	tasklet_hi_schedule(&q_vector->tx_done_tasklet);

	return IRQ_HANDLED;
}

static void agnic_irq_names_update(struct agnic_adapter *adapter, int start_vector, int is_tx_interrupt)
{
	struct device *dev = adapter->dev;
	int vector, cpu;
	char str_common[32];

	/* Set common string in the format of "dev_name.eth" */
	snprintf(str_common, sizeof(str_common), "%s.%s", dev_name(dev), adapter->netdev->name);

	/* Add postfix of tx/rx.cpuX */
	for (vector = start_vector; vector < (start_vector + adapter->num_q_vectors); vector++) {
		int q_vector_id = AGNIC_VECTOR_TO_Q_VECTOR(vector, adapter->num_q_vectors);
		struct agnic_q_vector *q_vector = adapter->q_vectors[q_vector_id];
		char *irq_name;

		if (is_tx_interrupt)
			irq_name = q_vector->tx_irq_name;
		else
			irq_name = q_vector->rx_irq_name;

		/* Get q_vector affinity CPU */
		cpu = cpumask_first(&q_vector->affinity_mask);

		/* Create interrupt identification srting */
		snprintf(irq_name, IRQ_NAME_SIZE, "%s.%s.%s%d",
				str_common, (is_tx_interrupt) ? "tx" : "rx", "cpu", cpu);

	}
}

/*
 * agnic_request_msix_irqs - Register Rx / Tx / Control IRQs.
 *
 *	adapter	     - pointer to agnic adapter.
 *	start_vector - vector to start from (in case previous vector were already assigned,
 *		       this parameter isn't be 0.)
 *	is_tx_interrupt - whether the request is for Tx or Rx interrutps.
 *
 *	The function returns the next vector to be used.
 */
static int agnic_request_msix_irqs(struct agnic_adapter *adapter, int start_vector, int is_tx_interrupt)
{
	struct device *dev = adapter->dev;
	int vector, ret;

	/* First, update the IRQs names */
	agnic_irq_names_update(adapter, start_vector, is_tx_interrupt);

	/* Iterate the vectors and request the IRQs */
	for (vector = start_vector; vector < (start_vector + adapter->num_q_vectors); vector++) {
		int q_vector_id = AGNIC_VECTOR_TO_Q_VECTOR(vector, adapter->num_q_vectors);
		struct agnic_q_vector *q_vector = adapter->q_vectors[q_vector_id];
		struct msix_entry *entry = &adapter->msix_entries[vector];
		irq_handler_t handler;
		int irq = entry->vector;
		char *irq_name;

		if (is_tx_interrupt) {
			q_vector->tx_msix_entry = entry;
			handler = agnic_tx_done_handler;
			irq_name = q_vector->tx_irq_name;
		} else {
			q_vector->rx_msix_entry = entry;
			handler = agnic_rx_irq_handler;
			irq_name = q_vector->rx_irq_name;
		}

		/* Disable IRQ balancing*/
		irq_set_status_flags(irq, IRQ_NO_BALANCING);

		/* Request the IRQ */
		ret = devm_request_irq(dev, irq, handler, IRQF_ONESHOT,
				       irq_name, q_vector);
		if (ret)
			goto free_msi_irqs;

		/* Disable IRQ to avoid interrutps before NAPI is enabled.
		 *
		 * Note: this is a WA for platform mode as we cannot disable
		 *	 agnic platform-msix interrupts mechanism. So the device
		 *	 may trigger interrupts before NAPI is ready an it will
		 *	 not be handled. If the device Q will be full, he won't send
		 *	 anymore interrutps and we'll be in live-lock.
		 *	 Hence we disable each interrupt once requested.
		 *	 It will be enabled later once NAPI is enabled (during if open).
		 */
		disable_irq(irq);

		/* Assign the affinity mask for this irq */
		irq_set_affinity_hint(irq, &q_vector->affinity_mask);
	}

	return vector;

free_msi_irqs:
	while (vector) {
		vector--;
		irq_set_affinity_hint(adapter->msix_entries[vector].vector, NULL);
		devm_free_irq(dev, adapter->msix_entries[vector].vector,
				adapter->q_vectors[vector]);
	}

	return -1;

}

/*
 * agnic_request_irqs - Register Rx / Tx / Control IRQs.
 */
static int agnic_request_irqs(struct agnic_adapter *adapter)
{
	int start_vector = 0;

	if (!(adapter->flags & AGNIC_FLAG_MSIX_ENABLED))
		return 0;

	/* Request Rx IRQs */
	if (adapter->flags & AGNIC_FLAG_RX_MSIX_ENABLED) {
		start_vector = agnic_request_msix_irqs(adapter, start_vector, 0 /* Rx */);
		if (start_vector < 0)
			return start_vector;
	}

	/* Request Tx IRQs */
	if (adapter->flags & AGNIC_FLAG_TX_MSIX_ENABLED) {
		start_vector = agnic_request_msix_irqs(adapter, start_vector, 1 /* Tx */);
		if (start_vector < 0)
			return start_vector;
	}

	return 0;
}

/*
 * agnic_free_irqs - Release all Rx / Tx / Control IRQs.
 */
static void agnic_free_irqs(struct agnic_adapter *adapter)
{
	struct device *dev = adapter->dev;
	int vector;

	if (!(adapter->flags & AGNIC_FLAG_MSIX_ENABLED))
		return;

	for (vector = 0; vector < adapter->num_vectors; vector++) {
		int q_vector_id = AGNIC_VECTOR_TO_Q_VECTOR(vector, adapter->num_q_vectors);
		struct agnic_q_vector *q_vector = adapter->q_vectors[q_vector_id];
		struct msix_entry *entry = &adapter->msix_entries[vector];

		if (entry->vector == -1)
			continue;

		/* clear the affinity_mask in the IRQ descriptor */
		irq_set_affinity_hint(entry->vector, NULL);

		/* free the irq */
		devm_free_irq(dev, entry->vector, q_vector);

		entry->vector = -1;
	}
}

/*
 * agnic_config_hw_queues - Send the Rx / Tx / Control queues information into
 * the HW. This will actually send the required control messages to the giu-nic to
 * create the required "channels".
 */
static int agnic_config_hw_queues(struct agnic_adapter *adapter)
{
	struct agnic_msg_params msg_params;
	struct agnic_mgmt_cmd_params cmd_params;
	struct agnic_mgmt_cmd_resp cmd_resp;
	struct agnic_ring *ring, *bp_ring;
	enum agnic_ingress_hash_type hash_type = ING_HASH_TYPE_NONE;
	int msg_len = sizeof(struct agnic_mgmt_cmd_params);
	int resp_msg_len = sizeof(struct agnic_mgmt_cmd_resp);
	u32 msix_id;
	int ret, tc, i;

	agnic_debug(adapter, "Configure queues in GIU.\n");

	/* Management echo. */
	agnic_debug(adapter, "Sending mgmt-echo.\n");
	msg_params.cmd_code = CC_PF_MGMT_ECHO;
	msg_params.client_id = 0;
	msg_params.client_type = CDT_PF;
	msg_params.msg = NULL; /* No msg params */
	msg_params.msg_len = 0;
	msg_params.resp_msg = &cmd_resp;
	msg_params.resp_msg_len = resp_msg_len;

	ret = agnic_mgmt_command_send(adapter, &msg_params);
	if (ret)
		goto error;

	/* PF_INIT */
	agnic_debug(adapter, "Sending PF_INIT.\n");

	msg_params.cmd_code = CC_PF_INIT;
	msg_params.client_id = 0;
	msg_params.client_type = CDT_PF;

	cmd_params.pf_init.mtu_override = AGNIC_MTU(adapter);
	cmd_params.pf_init.num_host_egress_tc = adapter->num_tcs;
	cmd_params.pf_init.num_host_ingress_tc = adapter->num_tcs;
	cmd_params.pf_init.mru_override = AGNIC_MTU(adapter);
	cmd_params.pf_init.egress_sched = ES_STRICT_SCHED;
	msg_params.msg = &cmd_params;
	msg_params.msg_len = msg_len;
	msg_params.resp_msg = &cmd_resp;
	msg_params.resp_msg_len = resp_msg_len;

	ret = agnic_mgmt_command_send(adapter, &msg_params);
	if (ret)
		goto error;

	/* PF_INGRESS_TC_ADD */
	agnic_debug(adapter, "Set ingress TC configuration.\n");
	msg_params.cmd_code = CC_PF_INGRESS_TC_ADD;
	msg_params.client_id = 0;
	msg_params.client_type = CDT_PF;
	msg_params.msg = &cmd_params;
	msg_params.msg_len = msg_len;
	msg_params.resp_msg = &cmd_resp;
	msg_params.resp_msg_len = resp_msg_len;

	if (adapter->num_qs_per_tc > 1)
		hash_type = (enum agnic_ingress_hash_type)adapter->rss_mode;

	for (i = 0; i < adapter->num_tcs; i++) {
		cmd_params.pf_ingress_tc_add.tc = i;
		cmd_params.pf_ingress_tc_add.num_queues = adapter->num_qs_per_tc;
		cmd_params.pf_ingress_tc_add.pkt_offset = 0;
		cmd_params.pf_ingress_tc_add.hash_type = hash_type;

		ret = agnic_mgmt_command_send(adapter, &msg_params);
		if (ret)
			goto error;
	}

	/* PF_INGRESS_DATA_QUEUE_ADD */
	msg_params.cmd_code = CC_PF_INGRESS_DATA_Q_ADD;
	msg_params.client_id = 0;
	msg_params.client_type = CDT_PF;
	msg_params.msg = &cmd_params;
	msg_params.msg_len = msg_len;
	msg_params.resp_msg = &cmd_resp;
	msg_params.resp_msg_len = resp_msg_len;

	for (tc = 0; tc < adapter->num_tcs; tc++) {
		for (i = 0; i < adapter->num_qs_per_tc; i++) {
			agnic_debug(adapter, "TC %d: Add ingress queue #%d.\n", tc, i);
			cmd_params.pf_ingress_data_q_add.tc = tc;

			ring = adapter->rx_ring[tc * adapter->num_qs_per_tc + i];
			ring->tc = tc; /* update the TC this ring is assigned to */

			cmd_params.pf_ingress_data_q_add.q_phys_addr = ring->dma;
			cmd_params.pf_ingress_data_q_add.q_len = ring->count;
			cmd_params.pf_ingress_data_q_add.q_prod_offs = AGNIC_RING_PROD_INDX_LOCAL_PHYS(ring);
			cmd_params.pf_ingress_data_q_add.q_cons_offs = AGNIC_RING_CONS_INDX_LOCAL_PHYS(ring);

			if (adapter->flags & AGNIC_FLAG_RX_MSIX_ENABLED) {
				/* Set MSI-X id */
				msix_id = ring->q_vector->rx_msix_entry->entry;
				cmd_params.pf_ingress_data_q_add.msix_id = msix_id;
			} else {
				/* MSI-X are not enabled */
				cmd_params.pf_ingress_data_q_add.msix_id = 0;
			}

			bp_ring = &adapter->bp_ring[cmd_params.pf_ingress_data_q_add.tc * adapter->num_qs_per_tc + i];
			cmd_params.pf_ingress_data_q_add.bpool_q_phys_addr = bp_ring->dma;
			cmd_params.pf_ingress_data_q_add.bpool_q_prod_offs = AGNIC_RING_PROD_INDX_LOCAL_PHYS(bp_ring);
			cmd_params.pf_ingress_data_q_add.bpool_q_cons_offs = AGNIC_RING_CONS_INDX_LOCAL_PHYS(bp_ring);
			cmd_params.pf_ingress_data_q_add.q_buf_size = bp_ring->bp_buff_sz;

			ret = agnic_mgmt_command_send(adapter, &msg_params);
			if (ret)
				goto error;
		}
	}

	/* PF_EGRESS_TC_ADD */
	agnic_debug(adapter, "Set egress TC configuration.\n");
	msg_params.cmd_code = CC_PF_EGRESS_TC_ADD;
	msg_params.client_id = 0;
	msg_params.client_type = CDT_PF;
	msg_params.msg = &cmd_params;
	msg_params.msg_len = msg_len;
	msg_params.resp_msg = &cmd_resp;
	msg_params.resp_msg_len = resp_msg_len;

	for (i = 0; i < adapter->num_tcs; i++) {
		cmd_params.pf_egress_tc_add.tc = i;
		cmd_params.pf_egress_tc_add.num_queues = adapter->num_qs_per_tc;

		ret = agnic_mgmt_command_send(adapter, &msg_params);

		if (ret)
			goto error;
	}

	/* PF_EGRESS_DATA_QUEUE_ADD */
	msg_params.cmd_code = CC_PF_EGRESS_DATA_Q_ADD;
	msg_params.client_id = 0;
	msg_params.client_type = CDT_PF;
	msg_params.msg = &cmd_params;
	msg_params.msg_len = msg_len;
	msg_params.resp_msg = &cmd_resp;
	msg_params.resp_msg_len = resp_msg_len;

	for (tc = 0; tc < adapter->num_tcs; tc++) {
		for (i = 0; i < adapter->num_qs_per_tc; i++) {
			pr_debug("TC %d: Add egress queue #%d.\n", tc, i);
			ring = adapter->tx_ring[tc * adapter->num_qs_per_tc + i];
			ring->tc = tc; /* update the TC this ring is assigned to */

			cmd_params.pf_egress_q_add.q_phys_addr = ring->dma;
			cmd_params.pf_egress_q_add.q_len = ring->count;
			cmd_params.pf_egress_q_add.q_prod_offs = AGNIC_RING_PROD_INDX_LOCAL_PHYS(ring);
			cmd_params.pf_egress_q_add.q_cons_offs = AGNIC_RING_CONS_INDX_LOCAL_PHYS(ring);

			if (adapter->flags & AGNIC_FLAG_TX_MSIX_ENABLED) {
				/* Set MSI-X id */
				msix_id = ring->q_vector->tx_msix_entry->entry;
				cmd_params.pf_egress_q_add.msix_id = msix_id;
			} else {
				/* MSI-X are not enabled */
				cmd_params.pf_egress_q_add.msix_id = 0;
			}

			/* Meanwhile, we support only strict prio */
			cmd_params.pf_egress_q_add.q_wrr_weight = 0;
			cmd_params.pf_egress_q_add.tc = tc;

			ret = agnic_mgmt_command_send(adapter, &msg_params);

			if (ret)
				goto error;
		}
	}

	/* PF_INIT_DONE */
	agnic_debug(adapter, "Send INIT_DONE command.\n");
	msg_params.cmd_code = CC_PF_INIT_DONE;
	msg_params.client_id = 0;
	msg_params.client_type = CDT_PF;
	msg_params.msg = NULL; /* No msg params */
	msg_params.msg_len = 0;
	msg_params.resp_msg = &cmd_resp;
	msg_params.resp_msg_len = sizeof(cmd_resp);

	ret = agnic_mgmt_command_send(adapter, &msg_params);
	if (ret)
		goto error;

	return 0;
error:
	/* TODO: Define configuration rollback at NIC level, then implement
	 * here.
	 */
	agnic_dev_err("Failed to configure network queues into hardware.\n");
	return ret;
}

/*
 * agnic_destroy_hw_queues - Delete all queues configuration from the underlying
 * hardware.
 */
static void agnic_destroy_hw_queues(struct agnic_adapter *adapter)
{
	agnic_dev_info("%s - Not implemented.\n", __func__);
	/* TODO: Need to define the ifdown process at the NIC side, and
	 * implement this function.
	 */
}

/*
 * agnic_napi_enable_all - Enable NAPI on all q-vectors.
 */
static int agnic_napi_enable_all(struct agnic_adapter *adapter)
{
	struct agnic_q_vector *vector;
	int i;

	for (i = 0; i < adapter->num_q_vectors; i++) {
		vector = adapter->q_vectors[i];
		napi_enable(&vector->napi);
	}

	return 0;
}

/*
 * agnic_napi_disable_all - Disable NAPI for all q-vectors.
 */
static void agnic_napi_disable_all(struct agnic_adapter *adapter)
{
	struct agnic_q_vector *vector;
	int i;

	for (i = 0; i < adapter->num_q_vectors; i++) {
		vector = adapter->q_vectors[i];
		napi_disable(&vector->napi);
	}

	return;

}

/*
 * agnic_txdone_handler_enable - Initialize required timers and hrtimer for txdone
 * handling.
 */
static int agnic_txdone_handler_enable(struct agnic_adapter *adapter)
{
	struct agnic_q_vector *vector;
	int i;

	for (i = 0; i < adapter->num_q_vectors; i++) {
		vector = adapter->q_vectors[i];

		/* If interrupts are not enabled, use timer to schedule tx done process */
		if (!(adapter->flags & AGNIC_FLAG_TX_MSIX_ENABLED)) {
			/* Initialize tx-done hrtimer. */
			hrtimer_init(&vector->tx_done_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL_PINNED);
			vector->tx_done_timer.function = agnic_txdone_hr_timer_callback;
		}

		/* Set tx_done schedule state as done */
		vector->txdone_timer_scheduled = false;

		/* Initialize tx-done tasklet. */
		tasklet_init(&vector->tx_done_tasklet, agnic_txdone_tasklet_callback, (unsigned long)vector);
	}
	return 0;
}

/*
 * agnic_txdone_handler_disable - Stop txdone handling.
 */
static int agnic_txdone_handler_disable(struct agnic_adapter *adapter)
{
	struct agnic_q_vector *vector;
	int i;

	for (i = 0; i < adapter->num_q_vectors; i++) {
		vector = adapter->q_vectors[i];

		/* If interrupts are not enabled, stop tx done timer */
		if (!(adapter->flags & AGNIC_FLAG_TX_MSIX_ENABLED))
			hrtimer_cancel(&vector->tx_done_timer);

		/* Set tx_done schedule state as done */
		vector->txdone_timer_scheduled = false;

		tasklet_kill(&vector->tx_done_tasklet);
	}
	return 0;
}

/*
 * agnic_disable_tx - Disable Tx traffic.
 */
static int agnic_disable_tx(struct agnic_adapter *adapter)
{
	int vector;

	/* Stop packet tx from upper layers. */
	netif_tx_stop_all_queues(adapter->netdev);

	/* Wait until Tx done is completed (per q_vector) */
	for (vector = 0; vector < adapter->num_q_vectors; vector++) {
		struct agnic_q_vector *q_vector = adapter->q_vectors[vector];
		int timeout = 50000;

		/* Wait until Tx done is completed */
		do {
			if (!q_vector->txdone_timer_scheduled)
				break;

			usleep_range(10, 20);
			timeout--;
		} while (timeout);

		if (timeout == 0)
			agnic_dev_err("%s: Timeout while waiting Tx done to complete for q_vector %d.\n",
					__func__, vector);
	}

	return 0;
}

/*
 * agnic_irq_enable - Enable interrupts reception for all q-vectors.
 */
static int agnic_irq_enable(struct agnic_adapter *adapter)
{
	/* Enable NIC interrupt signalling. */
	if (adapter->bus_irq_enable)
		adapter->bus_irq_enable(adapter);

	agnic_mgmt_irq_enable(adapter);

	return 0;
}

static void agnic_irq_disable(struct agnic_adapter *adapter)
{
	/* Disable NIC interrupt signalling. */
	agnic_mgmt_irq_disable(adapter);

	if (adapter->bus_irq_disable)
		adapter->bus_irq_disable(adapter);
}

/*
 * agnic_enable_traffic - Enable traffic on NIC side.
 */
static int agnic_enable_traffic(struct agnic_adapter *adapter)
{
	return agnic_set_enable(adapter->netdev, true);
}

/*
 * agnic_disable_traffic - Disable traffic in underlying HW.
 */
static int agnic_disable_traffic(struct agnic_adapter *adapter)
{
	int vector, ret = 0;

	ret = agnic_set_enable(adapter->netdev, false);
	if (ret)
		pr_err("Failed to disable HW\n");

	/* Make sure NAPI is not scheduled on any of the q_vectors */
	for (vector = 0; vector < adapter->num_q_vectors; vector++) {
		struct agnic_q_vector *q_vector = adapter->q_vectors[vector];
		int timeout = 1000;

		/* Wait until NAPI is done */
		do {
			if (!napi_is_scheduled(&q_vector->napi))
				break;

			usleep_range(10, 20);
			timeout--;
		} while (timeout);

		if (timeout == 0)
			agnic_dev_err("%s: Timeout while waiting NAPI to complete for q_vector %d.\n",
					__func__, vector);
	}

	return ret;
}

/*
** agnic_net_open(): start network interface.
*/
static int agnic_net_open(struct net_device *netdev)
{
	struct agnic_adapter *adapter = netdev_priv(netdev);
	static bool first_time = true;
	int err;

	agnic_debug(adapter, "agnic_net_open\n");

	if (unlikely(test_bit(AGNIC_FATAL_ERROR, &adapter->state))) {
		agnic_dev_info("Exit net open due to previous fatal error\n");
		return -EFAULT;
	}

	netif_carrier_off(netdev);

	if (first_time) {
		first_time = false;

		/* Allocate transmit descriptors */
		agnic_debug(adapter, "Allocating Tx data rings.\n");
		err = agnic_alloc_data_rings(adapter, agnic_tx_ring);
		if (err)
			goto error;

		/* Allocate receive descriptors */
		agnic_debug(adapter, "Allocating Rx data rings.\n");
		err = agnic_alloc_data_rings(adapter, agnic_rx_ring);
		if (err)
			goto err_setup_rx;

		/* Allocate Rx buffer pools. */
		agnic_debug(adapter, "Allocating BP rings.\n");
		err = agnic_alloc_bp_rings(adapter);
		if (err)
			goto err_setup_bpool;

		/* Register on previously acquired IRQs */
		agnic_debug(adapter, "Interrupt registration.\n");
		err = agnic_request_irqs(adapter);
		if (err)
			goto err_req_irq;

		/* Configure Rx, Tx, BP queues information into hardware. */
		agnic_debug(adapter, "Configure HW Queues.\n");
		err = agnic_config_hw_queues(adapter);
		if (err)
			goto err_setup_hwq;

		/* Set appropriate Rx mode (uni, multi, promisc). */
		agnic_debug(adapter, "Set Rx Mode.\n");
		agnic_set_rx_mode(netdev);

		/* Set Default MTU. */
		agnic_debug(adapter, "Set Default MTU\n");
		agnic_change_mtu(netdev, netdev->mtu);

		/* Populate BP rings. */
		agnic_debug(adapter, "Allocate BP buffers.\n");
		err = agnic_alloc_bpools_buffers(adapter);
		if (err)
			goto err_pop_bpool;

		/* Enable keep-alive watchdog */
		err = agnic_keep_alive_wd_start(adapter);
		if (err)
			goto err_watchdog;
	}

	/* Enable polling if rx interrupts are not enabled */
	if (!(adapter->flags & AGNIC_FLAG_RX_MSIX_ENABLED))
		adapter->flags |= AGNIC_FLAG_RX_POLL;

	/* Enable tx done handling. */
	err = agnic_txdone_handler_enable(adapter);
	if (err)
		goto err_txdone;

	/* Enable napi on all q-vectors */
	agnic_debug(adapter, "Enable NAPI.\n");
	agnic_napi_enable_all(adapter);

	/* Interface is no more down, additional control can be done. */
	clear_bit(AGNIC_DOWN, &adapter->state);

	/* Interrupts can be received now. */
	agnic_debug(adapter, "Enable interrupts.\n");
	agnic_irq_enable(adapter);

	/* Enable NIC - Send PF_ENABLE cmd */
	agnic_debug(adapter, "Enable traffic.\n");
	err = agnic_enable_traffic(adapter);
	if (err)
		goto err_en_traffic;

	/* Start irq polling if needed, irqpoll might have been already started
	 * during _probe (for notif ring handling), the function handles this
	 * case.
	 */
	agnic_debug(adapter, "Starting irq polling.\n");
	err = agnic_irqpoll_start(adapter);
	if (err)
		goto err_irqpoll;

	/* Make sure that only queues 0 to num_tx_queues-1 are being used. */
	netif_set_real_num_tx_queues(netdev, adapter->num_tx_queues);
	netif_set_real_num_rx_queues(netdev, adapter->num_rx_queues);

	/* Allow packet Tx. */
	agnic_debug(adapter, "Start Tx queues.\n");
	netif_tx_start_all_queues(netdev);

	return 0;

err_irqpoll: /* agnic_irqpoll_start errors are handled inside it */
	agnic_disable_traffic(adapter);
err_en_traffic:
	/* Mark interface as down. */
	set_bit(AGNIC_DOWN, &adapter->state);

	agnic_irq_disable(adapter);

	agnic_napi_disable_all(adapter);

	agnic_txdone_handler_disable(adapter);
err_txdone:
	adapter->flags &= ~AGNIC_FLAG_RX_POLL;
err_watchdog:
	agnic_free_bpools_buffers(adapter);
err_pop_bpool:
	agnic_destroy_hw_queues(adapter);
err_setup_hwq:
	agnic_free_irqs(adapter);
err_req_irq:
	agnic_free_bp_rings(adapter);
err_setup_bpool:
	agnic_free_data_rings(adapter, agnic_rx_ring);
err_setup_rx:
	agnic_free_data_rings(adapter, agnic_tx_ring);
error:
	/* Mark the interface as faulted */
	set_bit(AGNIC_FATAL_ERROR, &adapter->state);

	return err;
}

/*
** agnic_net_stop() - stop network interface.
*/
static int agnic_net_stop(struct net_device *netdev)
{
	int err;
	struct agnic_adapter *adapter = netdev_priv(netdev);

	if (unlikely(test_bit(AGNIC_DOWN, &adapter->state)))
		return 0;

	/* Mark interface as down. */
	set_bit(AGNIC_DOWN, &adapter->state);

	/* Stop interrupts reception. */
	agnic_irq_disable(adapter);

	/* Stop traffic on hardware side. */
	err = agnic_disable_traffic(adapter);
	if (err)
		agnic_dev_err("Failed to disable traffic\n");

	/* Stop tx. */
	err = agnic_disable_tx(adapter);
	if (err)
		agnic_dev_err("Failed to disable Tx\n");

	/* Disable polling (if rx interrupts are not enabled) */
	if (!(adapter->flags & AGNIC_FLAG_RX_MSIX_ENABLED))
		adapter->flags &= ~AGNIC_FLAG_RX_POLL;

	agnic_napi_disable_all(adapter);

	netif_carrier_off(netdev);

	/* Stop txdone handlers. */
	agnic_txdone_handler_disable(adapter);

	/* Mark interface as down. */
	set_bit(AGNIC_DOWN, &adapter->state);

	return 0;
}

/*
** agnic_set_num_queues - Assign queues for device.
**
** This is the top level queue allocation routine.  The order here is very
** important, starting with the "most" number of features turned on at once,
** and ending with the smallest set of features.  This way large combinations
** can be allocated if they're turned on, and smaller combinations are the
** fallthrough conditions.
**
** Currently, the function performs a trivial (single) queue allocation.
** going forward, the logic will be enhanced to take sriov & RSS queues into
** account.
*/
static void agnic_set_num_queues(struct agnic_adapter *adapter)
{
	/* Start with base case */
	adapter->num_rx_queues = adapter->num_tcs * adapter->num_qs_per_tc;
	adapter->num_tx_queues = adapter->num_tcs * adapter->num_qs_per_tc;
	adapter->num_rx_pools = adapter->num_rx_queues;
}

/*
** agnic_add_ring - Helper function to add a ring to a q-vector.
*/
static void agnic_add_ring(struct agnic_ring *ring,
		struct agnic_ring_container *head)
{
	ring->next = head->ring;
	head->ring = ring;
	head->count++;
}


/**
** agnic_alloc_q_vector - Allocate memory for a single queue vector
** @adapter: board private structure to initialize
** @v_count: q_vectors allocated on adapter, used for ring interleaving
** @v_idx: index of vector in adapter struct
** @txr_count: total number of Tx rings to allocate
** @txr_idx: index of first Tx ring to allocate
** @rxr_count: total number of Rx rings to allocate
** @rxr_idx: index of first Rx ring to allocate
*/
static int agnic_alloc_q_vector(struct agnic_adapter *adapter,
				cpumask_t *available_cpus,
				int v_count, int v_idx,
				int txr_count, int txr_idx,
				int rxr_count, int rxr_idx)
{
	struct agnic_q_vector *q_vector;
	struct agnic_ring *ring;
	int ring_count, size;
	int cpu;

	ring_count = txr_count + rxr_count;
	/* The size of the q_vector includes the rings associated with it. */
	size = sizeof(struct agnic_q_vector) +
		(sizeof(struct agnic_ring) * ring_count);

	/* allocate q_vector and rings */
	q_vector = kzalloc(size, GFP_KERNEL);
	if (!q_vector)
		return -ENOMEM;

	/* initialize NAPI */
	netif_napi_add(adapter->netdev, &q_vector->napi, agnic_napi_poll, 64);
#ifdef CONFIG_LK4_4_COMPAT
	/* this function is now static in 4.14 and is called inside netif_napi_add() */
	napi_hash_add(&q_vector->napi);
#endif /* CONFIG_LK4_4_COMPAT */

	/* tie q_vector and adapter together */
	adapter->q_vectors[v_idx] = q_vector;
	q_vector->adapter = adapter;
	q_vector->v_idx = v_idx;

	/* Set the q_vector affinity mask.
	 * It would be used later to assign the q_vector to a specific CPU
	 * as we would like to make sure that there is no resource sharing
	 * so each CPU will handle a single q_vector (and its rings).
	 */

	/* Get next available CPU */
	cpu = cpumask_first(available_cpus);
	/* Clear this CPU from the list (so it won't be used to handle another q_vector) */
	cpumask_clear_cpu(cpu, available_cpus);

	/* Check that the selected CPU is online */
	if (cpu_online(cpu))
		cpumask_set_cpu(cpu, &q_vector->affinity_mask);
	else {
		pr_err("cpu %d is not online. No cpu was assigned to q_vector %d\n", cpu, v_idx);
		return -1;
	}

	/* initialize work limits */
	q_vector->tx.work_limit = adapter->tx_work_limit;

	/* initialize pointer to rings */
	ring = q_vector->ring;

	while (txr_count) {
		/* assign generic ring traits */
		ring->dev = adapter->dev;
		ring->netdev = adapter->netdev;

		/* configure backlink on ring */
		ring->q_vector = q_vector;

		/* update q_vector Tx values */
		agnic_add_ring(ring, &q_vector->tx);

		/* apply Tx specific ring traits */
		ring->count = adapter->tx_ring_size;
		ring->q_idx = txr_idx;

		/* assign ring to adapter */
		adapter->tx_ring[txr_idx] = ring;

		/* update count and index */
		txr_count--;
		txr_idx += v_count;

		/* push pointer to next ring */
		ring++;
	}

	while (rxr_count) {
		/* assign generic ring traits */
		ring->dev = adapter->dev;
		ring->netdev = adapter->netdev;

		/* configure backlink on ring */
		ring->q_vector = q_vector;

		/* update q_vector Rx values */
		agnic_add_ring(ring, &q_vector->rx);

		/* apply Rx specific ring traits */
		ring->count = adapter->rx_ring_size;
		ring->q_idx = rxr_idx;

		/* assign ring to adapter */
		adapter->rx_ring[rxr_idx] = ring;

		/* update count and index */
		rxr_count--;
		rxr_idx += v_count;

		/* push pointer to next ring */
		ring++;
	}

	return 0;
}


/**
** agnic_free_q_vector - Free memory allocated for specific queue vector
** @adapter: board private structure to initialize
** @v_idx: Index of vector to be freed
**
** This function frees the memory allocated to the q_vector.  In addition if
** NAPI is enabled it will delete any references to the NAPI struct prior
** to freeing the q_vector.
*/
static void agnic_free_q_vector(struct agnic_adapter *adapter, int v_idx)
{
	struct agnic_q_vector *q_vector = adapter->q_vectors[v_idx];
	struct agnic_ring *ring;

	agnic_for_each_ring(ring, q_vector->tx)
		adapter->tx_ring[ring->q_idx] = NULL;

	agnic_for_each_ring(ring, q_vector->rx)
		adapter->rx_ring[ring->q_idx] = NULL;

	adapter->q_vectors[v_idx] = NULL;
	napi_hash_del(&q_vector->napi);
	netif_napi_del(&q_vector->napi);
}

/**
** agnic_alloc_q_vectors - Allocate memory for interrupt vectors
*/
static int agnic_alloc_q_vectors(struct agnic_adapter *adapter)
{
	struct cpumask available_cpus;
	int q_vectors = adapter->num_q_vectors;
	int rxr_remaining = adapter->num_rx_queues;
	int txr_remaining = adapter->num_tx_queues;
	int rxr_idx = 0, txr_idx = 0, v_idx = 0;
	int err;

	/* Set temp available cpu mask (as it will be changed during vector allocation) */
	cpumask_copy(&available_cpus, &adapter->available_cpus);

	/* First case, we have enough q_vectors for all Tx & Rx rings.
	** Assign each RX-Ring to a different q_vector.
	*/
	if (q_vectors >= (rxr_remaining + txr_remaining)) {
		for (; rxr_remaining; v_idx++) {
			err = agnic_alloc_q_vector(adapter, &available_cpus, q_vectors, v_idx,
						   0, 0, 1, rxr_idx);

			if (err)
				goto err_out;

			/* update counts and index */
			rxr_remaining--;
			rxr_idx++;
		}
	}

	/* We get here in two case:
	** a. We have enought q_vectors for Rx & Tx Rings, in this case
	**    rxr_remaining == 0 (because of the previous loop).
	**    So this loop will actually assign a q_victor for each tx ring.
	** b. We don't have enough Q-Vectors for all interrupts.
	**    The below loop will divide the Rx / Tx rings into the given
	**    q_vectors. On each loop, the amount of rings to go into the
	**    q_vector is re-calculated, in order to take the remainig #
	**    of Rx / Tx queues into account.
	*/
	for (; v_idx < q_vectors; v_idx++) {
		int rqpv = DIV_ROUND_UP(rxr_remaining, q_vectors - v_idx);
		int tqpv = DIV_ROUND_UP(txr_remaining, q_vectors - v_idx);

		err = agnic_alloc_q_vector(adapter, &available_cpus, q_vectors, v_idx,
				tqpv, txr_idx,
				rqpv, rxr_idx);
		if (err)
			goto err_out;

		/* update counts and index */
		rxr_remaining -= rqpv;
		txr_remaining -= tqpv;
		rxr_idx++;
		txr_idx++;
	}

	return 0;

err_out:
	adapter->num_tx_queues = 0;
	adapter->num_rx_queues = 0;
	adapter->num_q_vectors = 0;

	while (v_idx--)
		agnic_free_q_vector(adapter, v_idx);

	return -ENOMEM;
}

/**
** agnic_free_q_vectors - Free memory allocated for interrupt vectors
** @adapter: board private structure to initialize
**
** This function frees the memory allocated to the q_vectors.  In addition, if
** NAPI is enabled it will delete any references to the NAPI struct prior
** to freeing the q_vector.
*/
static void agnic_free_q_vectors(struct agnic_adapter *adapter)
{
	int v_idx = adapter->num_q_vectors;

	adapter->num_tx_queues = 0;
	adapter->num_rx_queues = 0;
	adapter->num_q_vectors = 0;

	while (v_idx--)
		agnic_free_q_vector(adapter, v_idx);
}

/*
** agnic_set_cpu_mask - Set available CPU mask (according to module params and resources )
*/
static int agnic_set_cpu_mask(struct agnic_adapter *adapter, struct cpumask *available_cpus)
{
	int num_available_cpus;

	/* Set available cpu mask to the module param value */
	cpumask_copy(available_cpus, to_cpumask(&cpu_mask));

	/* If cpu_mask is 0, set the mask to online cpus mask */
	if (cpumask_empty(available_cpus))
		cpumask_copy(available_cpus, cpu_online_mask);

	/* Check that the mask is a subset of online cpus */
	if (!cpumask_subset(available_cpus, cpu_online_mask)) {
		pr_err("Available cpus (0x%lx) are not subset of online cpus (0x%lx)\n",
				*cpumask_bits(available_cpus), *cpumask_bits(cpu_online_mask));
		return -1;
	}

	/* Check that number of Qs '(per TC) can be assigned to the available CPUs
	 * TODO: support more than 1 per CPU
	 */
	num_available_cpus = cpumask_weight(available_cpus);
	if (adapter->num_qs_per_tc > num_available_cpus) {
		agnic_dev_warn("Qs per TC (%d) must be <= number of available CPUs. Reducing to %d\n",
				adapter->num_qs_per_tc, num_available_cpus);
		adapter->num_qs_per_tc = num_available_cpus;
	}

	pr_debug("Available cpus mask is 0x%lx\n", *cpumask_bits(available_cpus));

	return 0;
}

/*
** agnic_set_interrupt_capability - set MSI-X or MSI if supported
*/
static int agnic_set_interrupt_capability(struct agnic_adapter *adapter)
{
	int err;

	/* TODO: remove the following lines */
	if (adapter->num_tcs != 1)
		pr_debug("setting interrupts for %d TCs\n", adapter->num_tcs);

#ifdef CONFIG_STUB_MODE
		/* No support for interrupts for LB-mode 1 and 2 */
		if (loopback_mode == 1 || loopback_mode == 2)
			msix_mode = 0;
#endif /* CONFIG_STUB_MODE */

	adapter->msix_flags = msix_mode; /* msix mode can be converted to mask */

	/* Get MSI-X interrupts (if enabled). */
	if ((adapter->bus_acquire_msix_vectors) && adapter->msix_flags &&
	    (adapter->bus_acquire_msix_vectors(adapter) == 0))
		return 0;

	/* Failed to get MSIx interrupts, fallback to MSI and disable
	** features which require MSIx support (e.g. SRIOV, RSS...
	** For now, we don't support RSS & SR-IOV, nothing should be done.
	** TODO: Disable SR-IOV & RSS capabilities.
	*/

	adapter->num_q_vectors = adapter->num_qs_per_tc;

	agnic_dev_warn("Failed to allocate MSI-X interrupts, falling back to MSI\n");
	if (adapter->bus_enable_msi) {
		err = adapter->bus_enable_msi(adapter);
		if (err)
			agnic_dev_warn("Failed to enable MSI interrupt (%d).\n", err);
		else
			adapter->flags |= AGNIC_FLAG_MSI_ENABLED;
	} else {
		agnic_dev_warn("No bus_enable_msi() callback.\n");
		err = -EOPNOTSUPP;
	}

	return err;
}

static void agnic_reset_interrupt_capability(struct agnic_adapter *adapter)
{
	if (adapter->flags & AGNIC_FLAG_MSIX_ENABLED) {
		adapter->flags &= ~AGNIC_FLAG_MSIX_ENABLED;
		adapter->bus_disable_msix(adapter);
		kfree(adapter->msix_entries);
		adapter->msix_entries = NULL;
	} else if (adapter->flags & AGNIC_FLAG_MSI_ENABLED) {
		adapter->flags &= ~AGNIC_FLAG_MSI_ENABLED;
		adapter->bus_disable_msi(adapter);
	}
}

/**
** agnic_update_num_q_vectors - Update Q vector number according to available CPUs
*/
void agnic_update_num_q_vectors(struct agnic_adapter *adapter)
{
	int num_available_cpus = cpumask_weight(&adapter->available_cpus);

	/* Check that number of Qs '(per TC) can be assigned to the available CPUs
	 * TODO: support more than 1 per CPU
	 */
	if (adapter->num_q_vectors > num_available_cpus) {
		agnic_dev_warn("number of Qs per TC (%d) must be <= number of available CPUs. Reducing to %d\n",
				adapter->num_q_vectors, num_available_cpus);

		adapter->num_q_vectors = num_available_cpus;
	}
}

/*
** agnic_init_interrupt_scheme - Determine proper interrupt scheme
**
** Determine which interrupt scheme to use based on...
** - Kernel support (MSI, MSI-X)
**   - which can be user-defined (via MODULE_PARAM)
** - Hardware queue count (num_*_queues)
**   - defined by miscellaneous hardware support/features (RSS, etc.)
*/
int agnic_init_interrupt_scheme(struct agnic_adapter *adapter)
{
	int err;

	/* Set available CPUs mask */
	err = agnic_set_cpu_mask(adapter, &adapter->available_cpus);
	if (err)
		return -1;

	/* Number of supported queues */
	agnic_set_num_queues(adapter);

	/* Set interrupt mode */
	err = agnic_set_interrupt_capability(adapter);
	if (err) {
		agnic_dev_err("Unable to allocate PCIe interrupt resourced.\n");
		goto err_int_setup;
	}

	/* recalculate number of q_vectors now that we have the information about
	 * available CPUs and requested q_vectors
	*/
	agnic_update_num_q_vectors(adapter);

	/* recalculate number of queues now that many features have been
	 * changed or disabled.
	 */
	agnic_set_num_queues(adapter);

	err = agnic_alloc_q_vectors(adapter);
	if (err) {
		agnic_dev_err("Unable to allocate memory for queue vectors\n");
		goto err_alloc_q_vectors;
	}

	agnic_dev_info("Multiqueue %s: Rx Queue count = %u, Tx Queue count = %u\n",
			(adapter->num_rx_queues > 1) ? "Enabled" : "Disabled",
			adapter->num_rx_queues, adapter->num_tx_queues);

	set_bit(AGNIC_DOWN, &adapter->state);

	return 0;

err_alloc_q_vectors:
	agnic_reset_interrupt_capability(adapter);
err_int_setup:
	return err;
}

/**
** agnic_clear_interrupt_scheme - Clear the current interrupt scheme settings
*/
void agnic_clear_interrupt_scheme(struct agnic_adapter *adapter)
{
	adapter->num_tx_queues = 0;
	adapter->num_rx_queues = 0;

	agnic_free_q_vectors(adapter);
	agnic_reset_interrupt_capability(adapter);
}

static const struct net_device_ops agnic_netdev_ops = {
	.ndo_open		= agnic_net_open,
	.ndo_stop		= agnic_net_stop,
	.ndo_set_rx_mode	= agnic_set_rx_mode,
	.ndo_set_mac_address	= agnic_net_set_address,
	.ndo_change_mtu		= agnic_change_mtu,
	.ndo_start_xmit		= agnic_net_xmit,
	.ndo_tx_timeout		= agnic_net_tx_timeout,
	.ndo_get_stats		= agnic_net_get_stats,
	.ndo_do_ioctl		= agnic_ioctl,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_set_features	= agnic_set_features,
	.ndo_vlan_rx_add_vid	= agnic_net_add_vlan,
	.ndo_vlan_rx_kill_vid	= agnic_net_kill_vlan,
	.ndo_set_tx_maxrate     = agnic_set_tx_maxrate,
};

struct net_device *agnic_net_alloc_netdev(struct device *dev)
{
	struct net_device *netdev;

	netdev = alloc_etherdev_mq(sizeof(struct agnic_adapter), AGNIC_MAX_RSS_INDICES);
	if (netdev) {
		SET_NETDEV_DEV(netdev, dev);
		dev_set_drvdata(dev, netdev);
	}
	return netdev;
}

void agnic_net_free_netdev(struct net_device *netdev)
{
	free_netdev(netdev);
}

static int agnic_netdev_init(struct device *dev, struct agnic_config_mem *nic_cfg)
{
	struct net_device	*netdev = dev_get_drvdata(dev);
	struct agnic_adapter	*adapter;
	int ret;
	int timeout = 1000;

	ret = -ENOMEM;

	adapter = netdev_priv(netdev);
	adapter->netdev = netdev;
	adapter->dev = dev;
	adapter->msg_enable = netif_msg_init(debug, DEFAULT_MSG_ENABLE);
	adapter->nic_cfg_base = nic_cfg;
	adapter->irqpoll_initialized = false;
	adapter->keep_alive_initialized = false;

	netdev->netdev_ops = &agnic_netdev_ops;

	netdev->watchdog_timeo = 5 * HZ;

	strncpy(netdev->name, dev_name(dev), sizeof(netdev->name) - 1);

	netdev->features = NETIF_F_RXCSUM | NETIF_F_IP_CSUM |
				NETIF_F_IPV6_CSUM;
	netdev->hw_features = netdev->features | NETIF_F_LOOPBACK;
	netdev->vlan_features |= netdev->features;

	/* Add support for VLAN filtering */
	netdev->features |= NETIF_F_HW_VLAN_CTAG_FILTER;

	/* Wait until the device firmware sets up the BARs */
	do {
		if (nic_cfg->status & AGNIC_CFG_STATUS_DEV_READY)
			break;
		usleep_range(1000, 2000);
		timeout--;
	} while (timeout);

	if (timeout == 0) {
		agnic_dev_err("Timeout while waiting for device ready.\n");
		goto err_alloc_etherdev;
	}

	/* Get the MAC address out of the NIC configuration space. */
	memcpy(netdev->dev_addr, nic_cfg->mac_addr, netdev->addr_len);

	/* Initialize adapter fields */
	adapter->rx_buffer_len = VLAN_ETH_FRAME_LEN + ETH_FCS_LEN;
	adapter->tx_ring_size = AGNIC_DEFAULT_TXD;
	adapter->tx_work_limit = AGNIC_DEFAULT_TX_WORK;
	adapter->rx_ring_size = AGNIC_DEFAULT_RXD;
	adapter->num_tcs = num_tcs;
	adapter->num_qs_per_tc = num_qs_per_tc;
	adapter->feature_enable_mask = feature_enable;

	/* Update tx_queue_len in netdev (Max frames per queue allowed) */
	netdev->tx_queue_len = adapter->tx_ring_size;

	/* Set all queue indexes as free */
	agnic_probe_debug("Allocate local indices array.\n");
	ret = agnic_init_q_indeces(adapter);
	if (ret)
		goto err_q_index_alloc;

	spin_lock_init(&adapter->stats64_lock);

	/* Figure out interrupts connectivity and assignment. */
	agnic_probe_debug("Init interrupts scheme.\n");
	ret = agnic_init_interrupt_scheme(adapter);
	if (ret) {
		agnic_dev_err("Failed to initialize interrupts scheme (Err %d).\n", ret);
		goto err_int_init;
	}

	/* Explicitly disable IRQ since the NIC can be in any state. */
	agnic_irq_disable(adapter);

	set_bit(AGNIC_DOWN, &adapter->state);
	clear_bit(AGNIC_FATAL_ERROR, &adapter->state);

	/* Allocate and initialize Management and Notification rings. */
	agnic_probe_debug("Setup management rings.\n");
	ret = agnic_setup_mgmt_rings(adapter);
	if (ret)
		goto err_mgmt;

	/* Start irq polling if needed. This call will enable irqpoll for mgmt
	 * queues.
	 */
	agnic_probe_debug("Starting irq polling.\n");
	ret = agnic_irqpoll_start(adapter);
	if (ret)
		goto err_irqpoll;

	agnic_probe_debug("Register network device.\n");
	strcpy(netdev->name, "eth%d");

	/* Set default mtu value */
	netdev->mtu = AGNIC_MAX_MTU;
	ret = register_netdev(netdev);
	if (ret)
		goto err_register;

	/* carrier off reporting is important to ethtool even BEFORE open */
	netif_carrier_off(netdev);

	/* Initialize ethtool */
	ret = agnic_ethtool_init(netdev);
	if (ret)
		goto err_ethtool;

	/* init the shadow mc address list */
	INIT_LIST_HEAD(&adapter->mc_list.list);

	return 0;

err_ethtool:
	unregister_netdev(netdev);
err_register:
	agnic_free_mgmt_rings(adapter);
err_irqpoll: /* agnic_irqpoll_start errors are handled inside it */
err_mgmt:
	agnic_clear_interrupt_scheme(adapter);
err_int_init:
	agnic_terminate_q_indeces(adapter);
err_q_index_alloc:
err_alloc_etherdev:
	return ret;
}

int agnic_net_probe(struct device *dev, struct agnic_config_mem *nic_cfg,
		void *bus_data)
{
	struct agnic_adapter *adapter;
	int ret;

	adapter = netdev_priv(dev_get_drvdata(dev));
	adapter->bus_data = bus_data;

	/* If msix setup function is provided, then msix en / dis callbacks are
	 * mandatory.
	 */
	if (adapter->bus_acquire_msix_vectors) {
		if ((!adapter->bus_disable_msix) || (!adapter->bus_enable_msix)) {
			dev_err(dev, "Missing msix handling callbacks.\n");
			return -EINVAL;
		}
	}

	/* msi en / dis callbacks are mandatory. */
	if ((!adapter->bus_enable_msi) || (!adapter->bus_disable_msi)) {
		dev_err(dev, "MSI enable / disable callbacks are mandatory.\n");
		return -EINVAL;
	}

	/* IRQ mask / unmask is mandatory. */
	if ((!adapter->bus_irq_disable) || (!adapter->bus_irq_enable)) {
		dev_err(dev, "IRQ mask / unmask callbacks are mandatory.\n");
		return -EINVAL;
	}

	/* Convert module rss param to valid rss mode */
	adapter->rss_mode = rss_mode + ING_HASH_TYPE_2_TUPLE;
	if (!(adapter->rss_mode >= ING_HASH_TYPE_NONE && adapter->rss_mode < ING_HASH_LAST)) {
		dev_err(dev, "rss-mode not in range [%d-%d]\n", ING_HASH_TYPE_NONE, (ING_HASH_LAST-1));
		return -EINVAL;
	}

	/* Prepare & register net device. */
	agnic_probe_debug("Init netdev.\n");
	ret = agnic_netdev_init(dev, nic_cfg);
	if (ret)
		return ret;

	agnic_info(probe, "Marvell GIU-NIC Network Adapter\n");

	return 0;
}

/**
 * agnic_remove - Device Removal Routine
 * @pdev: PCI device information struct
 *
 * agnic_remove is called by the PCI subsystem to alert the driver
 * that it should release a PCI device.  The could be caused by a
 * Hot-Plug event, or because the driver is going to be removed from
 * memory.
 **/
void agnic_net_remove(struct device *dev)
{
	struct net_device *netdev = dev_get_drvdata(dev);
	struct agnic_adapter *adapter = netdev_priv(netdev);

	/* Stop keep-alive watchdog */
	agnic_keep_alive_wd_stop(adapter);

	agnic_ethtool_release();

	unregister_netdev(netdev);

	/* Delete managemet timer */
	adapter->flags &= ~AGNIC_FLAG_MGMT_POLL;

	/* Empty BP pools and release the associated buffers. */
	agnic_free_bpools_buffers(adapter);

	/* Release all hardware queues. */
	agnic_destroy_hw_queues(adapter);

	/* Unbind interrupt service routines. */
	agnic_free_irqs(adapter);

	/* Free BP rings memory. */
	agnic_free_bp_rings(adapter);

	/* Release Rx & Tx data rings. */
	agnic_free_data_rings(adapter, agnic_tx_ring);
	agnic_free_data_rings(adapter, agnic_rx_ring);

	agnic_irqpoll_stop(adapter);
	agnic_free_mgmt_rings(adapter);

	agnic_clear_interrupt_scheme(adapter);
}

MODULE_DESCRIPTION("Armada GIU-NIC Driver");
MODULE_AUTHOR("Shadi Ammouri <shadi@marvell.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION("1");
