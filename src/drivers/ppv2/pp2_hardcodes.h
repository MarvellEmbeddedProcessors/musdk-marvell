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
 * TODO: Export these as parameters when the PPDK API shall take form */
#define MVPP2X_NUM_TXQS              (1)
#define MVPP2X_NUM_RXQS              (1)

#define MVPP2X_NUM_AQ_TXD            (128) /* Number of descriptors per AQ */
#define MVPP2X_NUM_TX_TXD            (256) /* Number of descriptors per TXQ */
#define MVPP2X_NUM_RX_RXD            (256) /* Number of descriptors per RXQ */

#define MVPP2_RX_COAL_PKTS           (1)
#define MVPP2_RX_COAL_USEC           (0)
#define MVPP2_TXDONE_COAL_PKTS       (1)
#define MVPP2_TXDONE_COAL_USEC       (0) /* No tx_time_coalescing */

#define MVPP2X_NUM_PORT_RXQS        (32) /* Number of RXQs per port */
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
 * XXX: Shall be deprecated by odp_packet */
#define PACKET_STEP  (64)
struct ppdk_pkt
{
   uintptr_t pkt_arr_phys; /* Packet array starting phys addr */
   uintptr_t pkt_arr_virt; /* Packet array starting virt addr */
   uint32_t  pkt_num;      /* Number of packets in this array */
   uint32_t  pkt_len;      /* Number of bytes per packet */
};

uint32_t ppdk_get_packets(uint32_t pkt_num, struct ppdk_pkt *pkt_array, uint32_t id);
void     ppdk_put_packets(struct ppdk_pkt *pkt_array);
#endif /*_PP2_HC_H_ */
