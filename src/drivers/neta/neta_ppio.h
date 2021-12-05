/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __NETA_PP_IO_H_
#define __NETA_PP_IO_H_

#include "lib/list.h"

/* internal structures */
struct neta_tx_queue {
	/* Number of this TX queue, in the range 0-7 */
	u8 id;

	/* Number of TX DMA descriptors in the descriptor ring */
	int size;

	/* Number of currently used TX DMA descriptor in the
	 * descriptor ring
	 */
	int count;
	int tx_stop_threshold;
	int tx_wake_threshold;

	/* Array of transmitted skb */
	struct sk_buff **tx_skb;

	/* Index of last TX DMA descriptor that was inserted */
	int txq_put_index;

	/* Index of the TX DMA descriptor to be cleaned up */
	int txq_get_index;

	u32 done_pkts_coal;

	/* Virtual address of the TX DMA descriptors array */
	struct neta_ppio_desc *descs;

	/* DMA address of the TX DMA descriptors array */
	dma_addr_t descs_phys;

	/* Index of the last TX DMA descriptor */
	int last_desc;

	/* Index of the next TX DMA descriptor to process */
	int next_desc_to_proc;

	/* DMA buffers for TSO headers */
	char *tso_hdrs;

	/* DMA address of TSO headers */
	dma_addr_t tso_hdrs_phys;

	/* statistics */
	u32	tx_pkts;
	u32	tx_drop;

};

struct neta_rx_queue {
	/* rx queue number, in the range 0-7 */
	u8 id;

	/* num of rx descriptors in the rx descriptor ring */
	int size;
	int desc_received;

	/* counter of times when mvneta_refill() failed */
	/*atomic_t missed;*/
	/*atomic_t refill_stop;*/
	/*struct neta_rx_desc *missed_desc;*/

	u32 pkts_coal;
	u32 time_coal;

	/* Virtual address of the RX DMA descriptors array */
	struct neta_ppio_desc *descs;

	/* DMA address of the RX DMA descriptors array */
	dma_addr_t descs_phys;

	/* Index of the last RX DMA descriptor */
	int last_desc;

	/* Index of the next RX DMA descriptor to process */
	int next_desc_to_proc;

	/* Index of the next RX DMA descriptor to refill with new buffer pointer */
	int next_desc_to_refill;
	u32 to_refill_cntr;

	/* statistics */
	u32	rx_pkts;
	u32	refill_bufs;

};

struct neta_port {
	u32	id; /* port Id */
	char	if_name[16];

	/* port state: up / down */
	bool	is_running;

	struct sys_iomem	*sys_iomem;
	uintptr_t		base;  /* virtual address for engine registers */
	phys_addr_t		paddr; /* physical address for engine registers */

	unsigned long flags;

	int buf_size;
	u16 rx_offset;

	u16 tx_ring_size;
	u16 rx_ring_size;
	u16 txq_number;
	u16 rxq_number;
	struct neta_rx_queue *rxqs;
	struct neta_tx_queue *txqs;

	int rxq_def;
	/* Protect the access to the percpu interrupt registers,
	 * ensuring that the configuration remains coherent.
	 */
	spinlock_t lock;
	bool is_stopped;

	u8 mcast_count[256];

	unsigned int link;
	unsigned int duplex;
	unsigned int speed;
	unsigned int tx_csum_limit;
	unsigned int use_inband_status:1;
	u8           mac[ETH_ALEN];

	uint16_t mtu;
	uint16_t mru;

	struct mvneta_bm *bm_priv;
	struct mvneta_bm_pool *pool_long;
	struct mvneta_bm_pool *pool_short;

	/* Port statistics */
	struct ethtool_gstrings *stats_name;
	struct neta_ppio_statistics stats;

	int is_promisc;
	struct list added_uc_addr;
	u32 num_added_uc_addr;
};

struct neta_port_uc_addr_node {
	struct list	list_node;
	u8		addr[ETH_ALEN];
};

#define GET_PPIO_PORT(ppio) ((struct neta_port *)(ppio)->internal_param)

/* Descriptor ring Macros */
#define MVNETA_QUEUE_NEXT_DESC(q, index)	\
	(((index) < (q)->last_desc) ? ((index) + 1) : 0)

/* Port TX FIFO constants */
#define MVNETA_TX_MTU_MAX		0x3ffff

/* Port minimum MTU in bytes */
#define MVNETA_PORT_MIN_MTU		(68) /* Required to support IPV4, per RFC791 */
#define MVNETA_PORT_MIN_MRU		(MV_MTU_TO_MRU(MVNETA_PORT_MIN_MTU))

/* TODO: to be reviewed when configuring QoS */
#define MVNETA_DEFAULT_RXQ		0

int neta_is_initialized(void);

/* PP-IO control routines */

/* Set MAC address */
int neta_port_set_mac_addr(struct neta_port *port, const uint8_t *addr);

/* Get MAC address */
int neta_port_get_mac_addr(struct neta_port *port, uint8_t *addr);

/* Get Link State */
int neta_port_get_link_state(struct neta_port *port, int  *en);

/* Set MTU */
int neta_port_set_mtu(struct neta_port *port, uint16_t mtu);

/* Get MTU */
void neta_port_get_mtu(struct neta_port *port, uint16_t *mtu);

/* Set MRU */
int neta_port_set_mru(struct neta_port *port, uint16_t len);

/* Get MRU */
void neta_port_get_mru(struct neta_port *port, uint16_t *len);

/* Set promiscuous */
int neta_port_set_promisc(struct neta_port *port, uint32_t en);

/* Check if promiscuous */
int neta_port_get_promisc(struct neta_port *port, uint32_t *en);

/* Set Multicast promiscuous */
int neta_port_set_mc_promisc(struct neta_port *port, uint32_t en);

/* Check if Multicast promiscuous */
int neta_port_get_mc_promisc(struct neta_port *port, uint32_t *en);

/* Add MAC address */
int neta_port_add_mac_addr(struct neta_port *port, const uint8_t *addr);

/* Remove MAC address */
int neta_port_remove_mac_addr(struct neta_port *port, const uint8_t *addr);

/* Delete all multicast MAC addresses */
int neta_port_flush_mac_addrs(struct neta_port *port);

/* Get port MAC MIB counters */
int neta_port_initialize_statistics(struct neta_port *port);
int neta_port_get_statistics(struct neta_port *port, struct neta_ppio_statistics *stats);

int neta_port_register(const char *if_name, int *id);
void neta_port_unregister(int id);

#endif /* __NETA_PP_IO_H_ */
