/*
** Copyright (C) 2015-2016 Marvell International Ltd.
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

#ifndef _ARMADA_GIU_NIC_H_
#define _ARMADA_GIU_NIC_H_

/* This enables the driver to skip HW errors and emulate a real HW behavior
 * when required, in order to enable development.
 * Once we start working on a real HW, all instances of this macro must be deleted.
 */
#undef EMULATION_MODE

#include <linux/netdevice.h>
#include <linux/interrupt.h>
#include "giu_nic_hw.h"
#include "giu_nic_mgmt.h"

#ifdef CONFIG_MV_NSS_ENABLE
#include "mv_gnic_nss.h"
#endif

#undef AGNIC_DEBUG

#undef pr_fmt
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#ifdef AGNIC_DEBUG
#define agnic_debug(adapter, format, arg...)			\
	netdev_dbg(adapter->netdev, "%s - " format, __func__, ## arg)

/* agnic_init_debug should be used during the early init stages of the driver
 * where the adapter structure is stil not initialized.
 */
#define agnic_probe_debug(format, arg...)				\
	pr_info("%s - " format, __func__, ## arg)
#else
#define agnic_debug(adapter, format, arg...)
#define agnic_probe_debug(format, arg...)
#endif /* AGNIC_DEBUG */

#define agnic_err(msglvl, format, arg...) \
	netif_err(adapter, msglvl, adapter->netdev, format, ## arg)
#define agnic_info(msglvl, format, arg...) \
	netif_info(adapter, msglvl, adapter->netdev, format, ## arg)
#define agnic_warn(msglvl, format, arg...) \
	netif_warn(adapter, msglvl, adapter->netdev, format, ## arg)
#define agnic_notice(msglvl, format, arg...) \
	netif_notice(adapter, msglvl, adapter->netdev, format, ## arg)
#define agnic_dev_info(format, arg...) \
	dev_info(adapter->dev, format, ## arg)
#define agnic_dev_warn(format, arg...) \
	dev_warn(adapter->dev, "Warn: " format, ## arg)
#define agnic_dev_err(format, arg...) \
	dev_err(adapter->dev, "Error: " format, ## arg)

#ifndef ceil
#define ceil(x, y) ((((x) + ((y) - 1)) / (y)))
#endif

#define AGNIC_CONFIG_BAR_SIZE	(0x10000)
#define AGNIC_CONFIG_BAR_ID	0

/* Tx/Rx descriptor defines */
#define AGNIC_MAX_TC			8
#define AGNIC_MAX_RXQ_COUNT		128
#define AGNIC_MAX_TXQ_COUNT		AGNIC_MAX_RXQ_COUNT
#define AGNIC_BPOOLS_COUNT		AGNIC_MAX_RXQ_COUNT

/* Maximum number of queues in system, including Rx, Tx, BP, MGMT,
** Notifications...
 */
#define AGNIC_MAX_MNG_QP_COUNT		1
#define AGNIC_MAX_MNG_Q_COUNT		(2 * AGNIC_MAX_MNG_QP_COUNT)
#define AGNIC_MAX_QUEUES	\
	(AGNIC_MAX_RXQ_COUNT + AGNIC_MAX_TXQ_COUNT + AGNIC_BPOOLS_COUNT + AGNIC_MAX_MNG_Q_COUNT)
#define AGNIC_DEFAULT_TXD		256
#define AGNIC_DEFAULT_TX_WORK		128
/*
 * TODO: WA Changed AGNIC_TX_DONE_THRESHOLD from 64 to 8
 * due to a tx stops transmitting after 12 packets
 * Must be investigated and be fixed
 */
#define AGNIC_TX_DONE_THRESHOLD		64
#define AGNIC_TX_DONE_CLEAN_COUNT	128
#define AGNIC_TXDONE_HRTIMER_PERIOD_NS	200000ul

#define AGNIC_DEFAULT_RXD		512
#define AGNIC_MAX_RSS_INDICES		8
#define MAX_Q_VECTORS			64

#ifdef CONFIG_MV_NSS_ENABLE
#define AGNIC_MAX_MTU			MAGNIC_NSS_MAX_MTU
#define AGNIC_BPOOLS_BUFF_SZ_LIST	{MAGNIC_NSS_BUFFER_SIZE}
#else
#define AGNIC_MAX_MTU			1500
#define AGNIC_BPOOLS_BUFF_SZ_LIST	{1536}
#endif /* CONFIG_MV_NSS_ENABLE */
#define AGNIC_BPOOL_SIZE		AGNIC_DEFAULT_RXD

/* MGMT Desc defines */
#define AGNIC_CMD_Q_LEN			256
#define AGNIC_CMD_Q_MAX_COOKIE_LEN	1024
#define AGNIC_NOTIF_Q_LEN		(AGNIC_CMD_Q_LEN)

#define MGMT_MSI_Q_VECTORS		0  /* TODO: set it to 1 once mgmt interrupts
					    * are supported
					    */
/* Number of interrupt vectors required for non-data queues.
** This is mainly used for control & status interrupts.\
*/
#define NON_Q_VECTORS			MGMT_MSI_Q_VECTORS
#define MIN_MSIX_Q_VECTORS		1
#define MIN_MSIX_COUNT			(MIN_MSIX_Q_VECTORS + NON_Q_VECTORS)

/* MSI-X ID 0 is used to mark invald id */
#define AGNIC_MSIX_ID_INVALID		0

/* Since msix id 0 marks invalid msix id, msi_index is increament by 1 */
#define AGNIC_SET_MSI_IDX(msi_index)	(msi_index + 1)

#define AGNIC_KEEP_ALIVE_TIMEOUT	10	/* 10 sec */

#ifdef EMULATION_MODE
#define MGMT_CMD_TIMEOUT_MSECS		100
#else
#define MGMT_CMD_TIMEOUT_MSECS		5000
#endif

#define MGMT_CMD_MAX_IDX		(1 << 16)
#define MGMT_CMD_IDX_FREE		(0)
#define MGMT_CMD_IDX_OCCUPY		(1)

#define AGNIC_MAC_UC_FILT_MAX		(0)	/* TODO: change once ucast filter is supported */
#define AGNIC_MAC_MC_FILT_MAX		(10)

/* utils to handle multicast address list */
#define agnic_mc_count(adapter)     ((adapter)->mc_list.mc_count)
#define agnic_mc_count_inc(adapter) ((adapter)->mc_list.mc_count++)
#define agnic_mc_count_dec(adapter) ((adapter)->mc_list.mc_count--)

/*
** agnic_mgmt_cookie: Pointed by the command descriptor (and copied to the respective
**  notification desc), provides the condition & operation result to be read by
**  command issuer.
** agnic_tx_cookie: Holds Tx buffers specific information, to be associated with each
**  transmitted buffer.
** agnic_bp_cookie: Holds BP buffers specific info, held along with Rx B-Pool
**  elements. And received back through the Rx descriptor cookie field.
*/
struct agnic_mgmt_cookie {
	enum {
		mgmt_buff_cmd_sent = 0,
		mgmt_buff_notif_rcv = 1
	} condition;	/* The condition on which the command issuer will wait */
	int result;	/* cmd result, as received by the notification message */
	void *buf;	/* Command response buffer */
	int buf_len;	/* Command response buffer length */
};

struct agnic_tx_cookie {
	dma_addr_t dma;
	struct sk_buff *skb;
	DEFINE_DMA_UNMAP_ADDR(addr);
	DEFINE_DMA_UNMAP_LEN(len);
};

struct agnic_bp_cookie {
	void *buff_virt_addr;
	struct agnic_ring *bp_ring;
	DEFINE_DMA_UNMAP_ADDR(addr);
	DEFINE_DMA_UNMAP_LEN(len);
};

struct agnic_cookie {
	union {
		struct agnic_tx_cookie   tx;
		struct agnic_bp_cookie   bp;
		struct agnic_mgmt_cookie mgmt;
	};
};

enum agnic_ring_type {
	agnic_rx_ring	 = 0x01,
	agnic_tx_ring	 = 0x02,
	agnic_cmd_ring	 = 0x04,
	agnic_notif_ring	 = 0x08,
	agnic_bpool_ring	 = 0x10,
	agnic_all_rings	 = 0xff
};
#define AGNIC_IS_INGRESS_RING(type) ((type) & (agnic_rx_ring | agnic_notif_ring))

/* All ring sizes are a power of 2.
** This is checked by the init functions.
*/
#define AGNIC_RING_PTR_INC(idx, num, ring_sz)		\
	(idx) = (((idx) + (num)) & ((ring_sz) - 1))

#define AGNIC_RING_IS_FULL(prod, cons, ring_sz)		\
	((((prod) + 1) & ((ring_sz) - 1)) == (cons))

#define AGNIC_RING_IS_EMPTY(prod, cons)			\
	((prod) == (cons))

#define AGNIC_RING_NUM_OCCUPIED(prod, cons, ring_sz)		\
	(((prod) - (cons) + (ring_sz)) & ((ring_sz) - 1))

#define AGNIC_RING_FREE(prod, cons, ring_sz)		\
	((ring_sz) - AGNIC_RING_NUM_OCCUPIED(prod, cons, ring_sz) - 1)

/* Calc physical address of a local producer / consumer pointer. */
#define AGNIC_RING_PROD_INDX_LOCAL_PHYS(ring)		\
	((ring)->adapter->ring_indices_arr_phys + ((ring)->prod_idx * sizeof(u32)))
#define AGNIC_RING_CONS_INDX_LOCAL_PHYS(ring)		\
	((ring)->adapter->ring_indices_arr_phys + ((ring)->cons_idx * sizeof(u32)))

/*
 * agnic_ring: Rx, Tx, mgmt, B-pool ring definition.
 *  - next: Next ring in q_vectors.
 *  - adapter: back pointer to adapter.
 *  - netdev: back pointer to net-device.
 *  - q_vector: The q_vector this ring belong to, NULL in case the ring does
 *  not belong to a specific q-vector (e.g. b-pool rings).
 *  - dev: Device for DMA mappings.
 *  - type: Type of ring (see agnic_ring_type).
 *  - desc_size: Size of a single descriptor.
 *  - prod_idx: Offset of the Q control producer pointer in the q_indeces_arr array
 *  - cons_idx: Offset of the Q control consumer pointer in the q_indeces_arr array
 *  - tc: the TC this ring is assigned to.
 *  - desc: Pointer to ring memory (Array).
 *  - dma: Physical address of desc.
 *  - count: # of descriptors in ring.
 *  Queue management "pointers": queue control indexes, we use pointers
 *  because the location of these control values will be communicated &
 *  shared with the HIU (In order for it to read & update them).
 *  On the egress direction, produce_p will be located on the
 *  end-point memory, and consumer_p will be located on the host
 *  dram (The other way around for the ingress side).
 *  We always want the RO variable to be on the local memory in order to prevent
 *  read accesses over PCIe (high latency). On the other side, write transactions
 *  over PCIe are less expensive as they are posted transactions.
 *  - Tx:
 *    o producer (WO): The descriptor that will hold the next transmitted packet.
 *    o consumer (RO): Next descritor to be fetched by HW (HIU).
 *    o tx_next_to_reclaim: Next descritor to free, after being sent by HW (i.e. first
 *      descriptor to be handled by the tx-done event.) - This is a local ring mgmt index
 *      that does not get read / written by the HIU.
 *    o tx_prod_shadow: A local shadow we operate on, before writing final value
 *      to producer pointer.
 *  - Rx:
 *    o producer (RO): The next descriptor to be filled with Rx data.
 *    o consumer (WO): The descriptor from which to receive the next packet.
 *    o rx_cons_shadow: A local shadow we operate on, before writing final value
 *      to consumer pointer.
 *  - cookie_list: Array of cookies info structs. This is actually a wrapper
 *    around 3 types of cookie info structures: Tx, Rx & Mgmt.
 *  - cookie_count: # of entries in cookie list.
 *  - q_idx: For data rings (Rx / Tx) - Holds the queue index.
 *    This is the absolute index of the queue in the system.
 *    Example: 8 Tx rings might be split into 3 different q-vectors, the
 *    q_idx of each ring will hold it's sequence number in the system
 *    (regardless of the q-vector it belongs to).
 *  - bp_buff_sz: For BP rings. Holds the buffer size managed by this buffer pool.
 *  - total_packets: # of rx/tx descriptors which were copied to the ring.
 *  - prev_counter: since we don't have hw counters, this field is used to allow
 *                  "reset" of the counters, when presented to the used.
 */
struct agnic_ring {
	struct agnic_ring *next;
	struct agnic_adapter *adapter;
	struct net_device *netdev;
	struct agnic_q_vector *q_vector;
	struct device *dev;
	enum agnic_ring_type type;
	int desc_size;
	u8 prod_idx;
	u8 cons_idx;
	u32 tc;

	void *desc;
	dma_addr_t dma;
	u32 count;

	u32 *producer_p;
	u32 *consumer_p;
	u16 tx_next_to_reclaim;
	union {
		u16 tx_prod_shadow;
		u16 rx_cons_shadow;
	};

	struct agnic_cookie *cookie_list;
	u32 cookie_count;

	union {
		u16 q_idx;
		u16 bp_buff_sz;
	};

	u32 bp_frag_sz;

	u32 total_packets;
	u32 prev_counter;
};

struct agnic_ring_container {
	struct agnic_ring *ring;		/* pointer to linked list of rings */
	unsigned int total_bytes;	/* total bytes processed this int */
	unsigned int total_packets;	/* total packets processed this int */
	u16 work_limit;			/* total work allowed per interrupt */
	u8 count;			/* total number of rings in vector */
};

#define IRQ_NAME_SIZE (36)

/* iterator for handling rings in ring container */
#define agnic_for_each_ring(pos, head) \
	for (pos = (head).ring; pos != NULL; pos = pos->next)

/* MAX_Q_VECTORS of these are allocated, but we only use one per
** queue-specific vector.
** q-vectors represent a set of rx/tx rings that share the same MSIx interrupt.
** Number of q-vectors are always smaller-equal to the number of CPUS, as
** having more q-vectors, will actually mean that the same CPU will be
** interrupted more than once for two different rings, which will negatively
** affect performance.
 */
struct agnic_q_vector {
	struct agnic_adapter *adapter;

	u16 v_idx;              /* index of q_vector within array */
	struct agnic_ring_container rx, tx;

	struct napi_struct napi;
	cpumask_t affinity_mask;

	/* Interrupts */
	struct msix_entry *rx_msix_entry;
	struct msix_entry *tx_msix_entry;

	char rx_irq_name[IRQ_NAME_SIZE];
	char tx_irq_name[IRQ_NAME_SIZE];

	/* Variables for managing tx-done timer. */
	bool txdone_timer_scheduled;
	struct hrtimer tx_done_timer;
	struct tasklet_struct tx_done_tasklet;

	/* for dynamic allocation of rings associated with this q_vector */
	struct agnic_ring ring[0];
};

struct agnic_custom {
	u8 id;
	void *arg;
	void (*f_recv_custom_msg_cb)(void *arg, u8 code, u64 cookie, void *msg, u16 len);
};

struct agnic_cb_msg_params {
	u32 status;
	u32 timeout; /* timeout in msec */
	u64 cookie;
};

struct agnic_mc_addr {
	struct list_head list;
	char   addr[ETH_ALEN];
	bool   hw_exist;
};

struct agnic_mc_addr_list {
	struct list_head list;
	int    mc_count;
};
struct agnic_pcpu_timer_irqpoll {
	struct timer_list list;
	struct agnic_q_vector *data;
};

struct agnic_adapter {
	struct net_device *netdev;
	struct device *dev;

	/* bus specific data, provided by the PCI / platform drivers. Will be
	 * used by the various callbacks.
	 */
	void *bus_data;

	/* Physical & virtual addresses of the agnic PCIe / platform config space. */
	phys_addr_t nic_cfg_phys_base;
	bool nic_cfg_on_local_mem;
	struct agnic_config_mem *nic_cfg_base;

	/* Device's operational state (link up / down, ready...). */
	unsigned long state;

	/* CPUs Mask */
	struct cpumask available_cpus;

	/* MSIx interrupts info */
	u32 num_vectors;
	u32 num_q_vectors;
	struct msix_entry *msix_entries;

	struct net_device_stats	stats;

	/* Bitmask of memory bars */
	int bars_mask;

	/* Bitmask of enabled debug messages, used by netif_msg_xxx()
	** macros.
	*/
	int msg_enable;

	spinlock_t stats64_lock;

	/* Tx - one ring per active queue */
	struct agnic_ring *tx_ring[AGNIC_MAX_TXQ_COUNT];
	u16 num_tx_queues;
	u16 tx_work_limit;
	u16 tx_ring_size;

	/* Rx */
	struct agnic_ring *rx_ring[AGNIC_MAX_RXQ_COUNT];

	/* BP */
	struct agnic_ring bp_ring[AGNIC_BPOOLS_COUNT];
	u32 pkt_offset;

	/* A page to hold the consumer / producer indexes per queue.
	 * the consumer_p or producer_p pointers will actually point
	 * an entry in this array (depending on the direction of the queue).
	 */
	u32 *ring_indices_arr;
	dma_addr_t ring_indices_arr_phys;
	u32 ring_indices_arr_len;

	/* Management command & notification Rings. */
	struct agnic_ring cmd_ring;
	struct agnic_ring notif_ring;
	/* Wait queue to wait for responses. */
	wait_queue_head_t mgmt_wait_q;
	spinlock_t mgmt_lock;

	/* Interrupt polling timer. Used for mgmt-notif & Rx handling*/
	struct agnic_pcpu_timer_irqpoll __percpu *irqpoll_timer_pcpu;
	bool irqpoll_initialized;
	int poll_timer_rate;

	/* Keep-alive Watchdog timer */
	struct timer_list keep_alive_wd;
	bool keep_alive_initialized;
	int keep_alive_timeout;
	struct work_struct wd_work;

	int mng_cpu; /* CPU for polling Mng queues */

	/* custom information */
	struct agnic_custom custom_info;

	u16 num_tcs;
	u16 num_qs_per_tc;
	u16 num_rx_pools;
	u16 num_rx_queues;
	u16 rx_buffer_len;
	u16 rx_ring_size;

	struct agnic_q_vector *q_vectors[MAX_Q_VECTORS];

	/* RSS mode */
	int rss_mode;

	/* Shadow list of mc addresses. Used to allow removal of a single mc address */
	struct agnic_mc_addr_list mc_list;

#define AGNIC_FLAG_RX_MSIX_ENABLED		BIT(0)
#define AGNIC_FLAG_TX_MSIX_ENABLED		BIT(1)
#define AGNIC_FLAG_MSIX_ENABLED			(AGNIC_FLAG_RX_MSIX_ENABLED | AGNIC_FLAG_TX_MSIX_ENABLED)
#define AGNIC_FLAG_MSI_ENABLED			BIT(2)
#define AGNIC_FLAG_MGMT_POLL			BIT(3)
#define AGNIC_FLAG_RX_POLL			BIT(4)
#define AGNIC_FLAG_IRQPOLL			(AGNIC_FLAG_RX_POLL | AGNIC_FLAG_MGMT_POLL)
#define AGNIC_FLAG_UC_PROMISC			BIT(5)
#define AGNIC_FLAG_MC_PROMISC			BIT(6)
	u32 flags;
	u32 msix_flags;	/* These flags are set by the user (module param) */

	/* Bitmask of enabled features */
#define AGNIC_FEATURES_KEEP_ALIVE_EN		BIT(0)
	u32 feature_enable_mask;

	/* Pointers to bus-specific callbacks. */
	/* acquire_msix_vectors: Reserve MSIx interrupt vectors (during the probe process), to be
	 * registered during the open process.
	 */
	int (*bus_acquire_msix_vectors)(struct agnic_adapter *);
	/* Enable / Disable MSIx interrupts */
	int (*bus_enable_msix)(struct agnic_adapter *);
	int (*bus_disable_msix)(struct agnic_adapter *);
	/* Enable / Disable MSI interrupts. */
	int (*bus_enable_msi)(struct agnic_adapter *);
	int (*bus_disable_msi)(struct agnic_adapter *);
	/* Globally Mask / unmask "HW" interrupt generation. */
	void (*bus_irq_enable)(struct agnic_adapter *);
	void (*bus_irq_disable)(struct agnic_adapter *);

	/* Pointers to data-path callbacks. */
	/* Tx packet signal*/
	void (*xmit_notify)(struct agnic_adapter *, int);
};

enum __agnic_state {
	AGNIC_READY,
	AGNIC_DOWN,
	AGNIC_FATAL_ERROR,
};

/* Networking part APIs to be called by the "bus-specific" code. */
struct net_device *agnic_net_alloc_netdev(struct device *dev);
void agnic_net_free_netdev(struct net_device *netdev);
int agnic_net_probe(struct device *dev, struct agnic_config_mem *nic_cfg, void *bus_data);
void agnic_net_remove(struct device *dev);
int agnic_mgmt_notif_process(struct agnic_adapter *adapter, u16 cmd_code, void *msg, u16 len);

#endif /* _ARMADA_GIU_NIC_H_ */
