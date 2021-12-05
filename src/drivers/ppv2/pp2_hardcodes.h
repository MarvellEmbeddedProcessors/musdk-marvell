/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PP2_HC_H_
#define _PP2_HC_H_

/*
 * INTERNAL HARDCODES AND DEFINES FOR PPDK testing
 */

#define  TRUE      (1)
#define  FALSE     (0)

/**************************** START HARDCODES ***************************/
/* Hardcoded stuff for a quick demo */

/* PPK test configurations.
 * TODO: Export these as parameters when the PPDK API shall take form
 */
#define MVPP2X_NUM_TXQS              (1)
#define MVPP2X_NUM_RXQS              (1)

#define MVPP2X_NUM_AQ_TXD            (128) /* Number of descriptors per AQ */
#define MVPP2X_NUM_TX_TXD            (256) /* Number of descriptors per TXQ */
#define MVPP2X_NUM_RX_RXD            (256) /* Number of descriptors per RXQ */

#define MVPP2_RX_COAL_PKTS           (1)
#define MVPP2_RX_COAL_USEC           (0)
#define MVPP2_TXDONE_COAL_PKTS       (1)
#define MVPP2_TXDONE_COAL_USEC       (0) /* No tx_time_coalescing */

#define MVPP2X_NUM_PORT_RXQS         (32) /* Number of RXQs per port */
#define MVPP2X_NUM_PHYS              (2) /* Number of packet processor phys addr */

/* Absolute physical address from where we allocate for TXDs/RXDs etc. */
#define MVPP2X_DEVIOMEM_PKT_SIZE     (0x2000)  /* Region for packet data*/

#define MVPP2X_DEVIOMEM_PACKET        (0x5000A000) /* Packets */
#define MVPP2X_DEVIOMEM_BM_BPPE_POOL  (0x50010000) /* BM BPPE */
#define MVPP2X_DEVIOMEM_BM_BP_POOL    (0x50015000) /* BM BPs  */

/* TX FIFO constants */
#define MVPP2_TX_FIFO_DATA_SIZE_10KB      0xA
#define MVPP2_TX_FIFO_DATA_SIZE_3KB       0x3

/**************************** END HARDCODES ***************************/

/* Internal testing packet structure
 * XXX: Shall be deprecated by odp_packet
 */
#define PACKET_STEP  (64)
struct ppdk_pkt {
	uintptr_t pkt_arr_phys; /* Packet array starting phys addr */
	uintptr_t pkt_arr_virt; /* Packet array starting virt addr */
	u32  pkt_num;      /* Number of packets in this array */
	u32  pkt_len;      /* Number of bytes per packet */
};

u32 ppdk_get_packets(u32 pkt_num, struct ppdk_pkt *pkt_array, uint32_t id);
void     ppdk_put_packets(struct ppdk_pkt *pkt_array);
#endif /*_PP2_HC_H_ */
