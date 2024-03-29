/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file pp2_port.h
 *
 * Packet Processor I/O
 *
 * Presentation API for PPDK control and data-path routines
 *
 */
#ifndef __PP2_PORT_H__
#define __PP2_PORT_H__

#include "std_internal.h"
#include "pp2_types.h"
#include "pp2_dm.h"

#define PP2_LOOPBACK_PORT_TXQ_PREFETCH	PP2_TXQ_PREFETCH_64
#define PP2_ETH_PORT_TXQ_PREFETCH	PP2_TXQ_PREFETCH_16

#define MVPP22_F_IF_MUSDK_PRIV		BIT(0)

struct pp2_port;

/**
 * pp2_deinit
 *
 * Deinitializes the Packet Processor Development Kit (PPDK)
 *
 *
 * @retval  0 on success, error otherwise
 *
 */
void pp2_deinit(void);

int pp2_is_init(void);

/**
 * pp2_port_inq_param
 *
 * Ingress queue parameters
 *
 * @field	id    Ingress queue ID
 * @field   size  Ingress queue size in ingress descriptors
 */
struct pp2_port_inq_param {
	u32 id;
	u32 size;
};

/**
 * pp2_port_intc_param
 *
 * Ingress traffic class parameters
 *
 * @field	id         Traffic class ID
 * @field   qos	   Quality of service
 * @field   use_hash   Hash
 * @field   num_in_qs  Number of associated ingress queues
 * @field   inq_param  Array of ingress queue parameters
 */
struct pp2_port_intc_param {
	u32 id;
	u32 qos;
	u32 num_in_qs;
	struct pp2_port_inq_param *inq_param;
};

/**
 * pp2_port_inqs_param
 *
 * Compound ingress parameters per port
 *
 * @field	num_in_tcs   Number of traffic classes
 * @field   intcs_param  Array of traffic class parameters
 */
struct pp2_port_inqs_param {
	u32 num_in_tcs;
	struct pp2_port_intc_param *intcs_param;
};

/**
 * pp2_port_outq_param
 *
 * Egress queue parameters
 *
 * @field	id    Egress queue ID
 * @field   size  Egress queue size in egress descriptors
 */
struct pp2_port_outq_param {
	u32 id;
	u32 size;
};

/**
 * pp2_port_outqs_param
 *
 * Compound egress parameters per port
 *
 * @field	num_out_qs   Number of associated egress queues
 * @field   outqs_param  Array of egress queue parameters
 */
struct pp2_port_outqs_param {
	u32 num_out_qs;
	struct pp2_port_outq_param *outqs_param;
};

/**
 * pp2_port_param
 *
 * Per port ingress and egress initialization parameters
 *
 * @field	if_name  Interface name. The name signature
 *                   should be composed of a string and a number,
 *                   where the number should be mapped 1:1
 *                   against a physical device tree port node
 *
 *                   Valid range of interface numbers:
 *                   [0 - PP2_NUM_PORTS]
 *                   See <pp2_plat.h>
 *
 * @field   pp2_port_inqs_param   Ingress parameters for this port
 * @field   pp2_port_outqs_param  Egress parameters for this port
 */
struct pp2_port_param {
	const char *if_name;
	struct pp2_port_inqs_param inqs_param;
	struct pp2_port_outqs_param outqs_param;
	/* TODO: Loopback. Remove after finishing debugging */
	u32 use_mac_lb;
};

/**
 * pp2_traffic_type
 * Per port traffic types can be selected
 */
typedef enum {
	PP2_TRAFFIC_NONE,
	PP2_TRAFFIC_INGRESS,
	PP2_TRAFFIC_EGRESS,
	PP2_TRAFFIC_INGRESS_EGRESS
} pp2_traffic_mode;

/**
 * pp2_port_open
 *
 * Initialize (create) and prepare a port based on input parameters
 * After this routine returns successfully, the port is in
 * a configured state until an issue of pp2_port_start()
 *
 * @param	ppdk	The PPDK handle. Based on the param input
 *                  the corresponding packet processor shall be used
 * @param   param	Egress and ingress related parameters
 *
 * @retval	Port handle on success, NULL otherwise
 */
int pp2_port_open(struct pp2 *pp2, struct pp2_ppio_params *param, u8 pp2_id, u8 port_id,
		  struct pp2_port **port_hdl);

/**
 * pp2_port_close
 *
 * Deinitialize (destroy) a port which is in a stopped state
 *
 * @param	port  Port handle of an already-opened port
 *
 * @retval	0 on success, error otherwise
 */
void pp2_port_close(struct pp2_port *port);

/**
 * pp2_port_config_inq
 *
 * Configure Ingress queues
 *
 * @param	port  Port handle of an already-opened port
 */
void pp2_port_config_inq(struct pp2_port *port);

/**
 * pp2_port_config_outq
 *
 * Configure Egress queues
 *
 * @param	port  Port handle of an already-opened port
 */
void pp2_port_config_outq(struct pp2_port *port);

/**
 * pp2_port_start
 *
 * Set an already-opened port to ready-mode, making the
 * port available for servicing various I/O control or
 * data path requests
 *
 * @param	port	Port handle of an already-opened port
 *
 * @retval	0 on success, error otherwise
 */
void pp2_port_start(struct pp2_port *port, pp2_traffic_mode t_mode);

/**
 * pp2_port_stop
 *
 * Stop an already-opened port, making it unavailable to
 * service I/O requests
 *
 * @param	port	Port handle of a running port
 *
 * @retval	0 on success, error otherwise
 */
void pp2_port_stop(struct pp2_port *port);

/**
 * pp2_port_dm_if_get
 *
 * Request a DM-IF object from this interface
 *
 * @param    port    Logical egress port handle
 *
 * @param    dm_id   Logical DM object ID.
 *                   This ID should correspond with the same ID that was
 *                   used for the DM object allocation request
 *
 * @retval   DM object handle
 */
struct pp2_dm_if *pp2_port_dm_if_get(struct pp2_port *port, uint32_t dm_id);

/**
 * pp2_port_enqueue
 *
 * Enqueue one or more descriptors through an egress port using an associated
 * TX queue ID
 *
 * The DM object should already contain TX descriptor objects, each one
 * associated with either a packet or, if the case, a buffer as part of
 * a multi-buffer fragmented packet.
 *
 * NOTE: Enqueue thread has responsibility for manipulating the TX descriptors
 * associated with the DM object before issuing this call
 *
 * @param    port     Logical egress port ID
 *
 * @param    dm_if    DM object associated with the enqueue requestor
 *
 * @param    outq_id  Logical egress queue ID associated with this egress port
 *
 * @retval   number of enqueued descriptors
 */
uint16_t pp2_port_enqueue(struct pp2_port *port, struct pp2_dm_if *dm_if, uint8_t out_qid,
			  u16 num_txds, struct pp2_ppio_desc desc[], struct pp2_ppio_sg_pkts *pkts);

/**
 * pp2_port_outq_get_id
 *
 * Get physical queue ID associated with this logical egress queue.
 *
 * Egress requestor needs to stamp the egress physical queue ID in each
 * descriptors before enqueueing the descriptors.
 *
 * @param    port     Logical egress port ID
 *
 * @param    outq_id  Logical egress queue ID that shall be used for enqueueing
 *
 * @retval   physical egress queue ID
 */
uint32_t pp2_port_outq_get_id(struct pp2_port *port, uint32_t outq_id);

/**
 * pp2_port_outq_status
 *
 * Get number of actually transmitted descriptors in order to handle
 * post-enqueue operations such as packet releases or notifications.
 *
 * NOTE: Should be called after pp2_port_enqueue()
 *
 * @param    dm_if  Aggregator queue object associated with the enqueue requestor
 *
 * @param    outq_physid  Physical egress queue ID associated with this egress port
 *                        Obtainable via pp2_port_outq_get_id().
 *
 * @retval   number of actually sent descriptors
 */
uint32_t pp2_port_outq_status(struct pp2_dm_if *dm_if, uint32_t out_physid);

/**
 * pp2_port_outq_status
 *
 * Enable/disable Tx queue
 *
 * @param    port     Logical egress port ID
 *
 * @param    txq      pointer Tx queue object
 *
 * @param    en       A flag indicates if queue is enabled or disabled
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_port_set_outq_state(struct pp2_port *port, struct pp2_tx_queue *txq, int en);

/**
 * pp2_port_poll
 *
 * Core polling based routine. Internally, this either reports miscellaneous
 * ingress/egress errors from previous transactions and/or provides RX descriptor
 * objects to the polling requestor by dequeueing from the provided ingress queue.
 *
 * Each RX descriptor object from the output RXD array contains information either
 * about a packet or about a buffer part of a fragmented packet.
 *
 * NOTE: Polling thread has responsibility for manipulating the RX descriptors
 * after this call returns
 *
 * NOTE: Polling thread has responsability for calling pp2_port_inq_update() after
 * manipulating all packets from the received RX descriptors.
 *
 * NOTE: Polling thread has resposability to check for extra descriptors that
 *       might be provided.
 *
 * @param    port  Logical ingress port ID
 *
 * @output   in_desc  RX descriptor objects provided to the dequeue requester
 *                    The routine will fill these from the ingress queue
 *                    provided as input
 *
 * @param    inq_id  Logical ingress queue ID associated with this ingress port
 *
 * @output   extra_desc  Extra RX descriptor objects provided to the dequeue
 *                       requester. This is the case when at the end of the
 *                       rx descriptor array is not enough space to accommodate
 *                       all received rx descriptors and some descriptors are
 *                       stored at the beginning. So the descriptors are split
 *                       in two for application.
 *
 * @output   extra_recv  zero or number of exatr valid RXDs ready in queue.
 *                       It is mandatory to check for extra descriptors.
 *
 *
 * @retval   zero or number of valid RXDs ready in the queue, plus the updated
 *           in_desc RXD parameter array
 */
uint32_t pp2_port_poll(struct pp2_port *port, struct pp2_desc **desc, uint32_t in_qid,
		       struct pp2_desc **extra_desc, uint32_t *extra_recv);

/**
 * pp2_port_inq_update
 *
 * Update ingress queue with the number of ingress processed packets from this queue's descriptors.
 *
 * NOTE: Should be called after the Rx packets have been processed and released to the
 *       BM pool associated with this ingress queue
 *
 * @param    port    Logical ingress port ID
 *
 * @param    inq_id  Logical ingress queue ID from which dequeue was done earlier
 *
 * @param    new_desc  Number of new descriptors. Default queue ID size
 *
 * @param    proc_desc   Number of processed descriptors
 *
 * @retval   void
 */
void pp2_port_inq_update(struct pp2_port *port, uint32_t inq_id, uint32_t new_desc, uint32_t proc_desc);

struct pp2_desc *pp2_rxq_get_desc(struct pp2_rx_queue *rxq, uint32_t *num_recv,
				  struct pp2_desc **extra_desc, uint32_t *extra_num);

/* PP-IO control routines */

/* Get link status */
int pp2_port_link_status(struct pp2_port *port);

/* Get link info */
int pp2_port_link_info(struct pp2_port *port, struct pp2_port_link_status *pstatus);

/* Set MAC address */
int pp2_port_set_mac_addr(struct pp2_port *port, const uint8_t *addr);

/* Get MAC address */
int pp2_port_get_mac_addr(struct pp2_port *port, uint8_t *addr);

/* Get Link State */
int pp2_port_get_link_state(struct pp2_port *port, int  *en);

/* Get Rx Pause FC status */
int pp2_port_get_rx_pause(struct pp2_port *port, int *en);

/* Set Rx Pause FC */
int pp2_port_set_rx_pause(struct pp2_port *port, int en);

/* Enable Tx Pause FC in Phy */
int pp2_port_set_phy_tx_pause(struct pp2_port *port, int en);

/* Set MTU */
int pp2_port_set_mtu(struct pp2_port *port, uint16_t mtu);

/* Get MTU */
void pp2_port_get_mtu(struct pp2_port *port, uint16_t *mtu);

/* Set port rate limit parameters */
int pp2_port_set_rate_limit(struct pp2_port *port, struct pp2_ppio_rate_limit *rate_limit);

/* Set queue rate limit parameters */
int pp2_port_set_queue_rate_limit(struct pp2_port *port, u8 qid, struct pp2_ppio_rate_limit *rate_limit);

/* Get port rate limit parameters */
int pp2_port_get_rate_limit(struct pp2_port *port, struct pp2_ppio_rate_limit *rate_limit);

/* Get queue rate limit parameters */
int pp2_port_get_queue_rate_limit(struct pp2_port *port, u8 qid, struct pp2_ppio_rate_limit *rate_limit);

/* Set MRU */
int pp2_port_set_mru(struct pp2_port *port, uint16_t len);

/* Get MRU */
void pp2_port_get_mru(struct pp2_port *port, uint16_t *len);

/* Set Loopback */
int pp2_port_set_loopback(struct pp2_port *port, int en);

/* Get Loopback */
int pp2_port_get_loopback(struct pp2_port *port, int *en);

/* Set Unicast promiscuous */
int pp2_port_set_promisc(struct pp2_port *port, uint32_t en);

/* Check if Unicast promiscuous */
int pp2_port_get_promisc(struct pp2_port *port, uint32_t *en);

/* Set Multicast promiscuous */
int pp2_port_set_mc_promisc(struct pp2_port *port, uint32_t en);

/* Check if Multicast promiscuous */
int pp2_port_get_mc_promisc(struct pp2_port *port, uint32_t *en);

/* Set Port enable */
int pp2_port_set_enable(struct pp2_port *port, uint32_t en);

/* Check if Port enabled */
int pp2_port_get_enable(struct pp2_port *port, uint32_t *en);

/* Add MAC address */
int pp2_port_add_mac_addr(struct pp2_port *port, const uint8_t *addr);

/* Remove MAC address */
int pp2_port_remove_mac_addr(struct pp2_port *port, const uint8_t *addr);

/* Flush MAC addresses */
int pp2_port_flush_mac_addrs(struct pp2_port *port, uint32_t uc, uint32_t mc);

/* Set vlan filtering */
int pp2_port_set_vlan_filtering(struct pp2_port *port, int en);

/* Add vlan */
int pp2_port_add_vlan(struct pp2_port *port, u16 vlan);

/* Remove vlan */
int pp2_port_remove_vlan(struct pp2_port *port, u16 vlan);

/* Clear vlan */
int pp2_port_clear_vlan(struct pp2_port *port, u16 vlan);
int pp2_port_clear_all_vlans(struct pp2_port *port);

int pp2_port_initialize_statistics(struct pp2_port *port);
int pp2_port_get_statistics(struct pp2_port *port, struct pp2_ppio_statistics *stats);

/* Initialize MTU, port and queue rate limits and txq arbitration */
int pp2_port_config_txsched(struct pp2_port *port);

/* Deinitialize port and queue rate limits and txq arbitration */
void pp2_port_deinit_txsched(struct pp2_port *port);

/* Get number of Rx descriptors occupied by received packets */
static inline uint32_t
pp2_rxq_received(struct pp2_port *port, const int rxq_id)
{
	u32 val = pp2_reg_read(port->cpu_slot, MVPP2_RXQ_STATUS_REG(rxq_id));

	return (val & MVPP2_RXQ_OCCUPIED_MASK);
}
/* Open uio_device associated with a port */
int pp2_port_open_uio(struct pp2_port *port);
int pp2_port_set_priv_flags(struct pp2_port *port, u32 val);
/* Close uio_device associated with a port */
int pp2_port_close_uio(struct pp2_port *port);

/* Functions from mvpp2x driver, for ks implementation */
int mv_pp2x_port_musdk_set(void *netdev_priv);
int mv_pp2x_port_musdk_clear(void *netdev_priv);


int populate_tc_pools(struct pp2_inst *pp2_inst, struct pp2_bpool *param_pools[][PP2_PPIO_TC_CLUSTER_MAX_POOLS],
			     struct pp2_bm_pool *pools[][PP2_PPIO_TC_CLUSTER_MAX_POOLS]);
void pp2_port_rxqs_create(struct pp2_port *port);
void pp2_port_txqs_create(struct pp2_port *port);

int pp2_port_rx_create_event(struct pp2_port *port, struct pp2_ppio_rxq_event_params *params, struct mv_sys_event **ev);
int pp2_port_rx_delete_event(struct mv_sys_event *ev);
int pp2_port_rx_set_event(struct mv_sys_event *ev, int en);
int ppv2_port_rx_create_event_validate(void *driver_data);

int pp2_port_get_tx_pause(struct pp2_port *port, int *en);
int pp2_port_set_tx_pause(struct pp2_port *port, struct pp2_ppio_tx_pause_params *params);

int pp2_port_set_inq_state(struct pp2_port *port, u8 tc, u8 qid, int en);
int pp2_port_get_inq_state(struct pp2_port *port, u8 tc, u8 qid, int *en);

int pp2_port_set_inq_early_drop(struct pp2_port *port, u8 tc, u8 qid, int en, struct pp2_cls_early_drop *edrop);

void pp2_port_rxq_cntrs_dump(struct pp2_port *port, int rxq);
void pp2_port_txq_cntrs_dump(struct pp2_port *port, int txq);


#endif /* __PP2_PORT_H__ */
