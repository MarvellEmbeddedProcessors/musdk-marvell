/*******************************************************************************
 * Copyright (C) Marvell International Ltd. and its affiliates
 *
 * This software file (the "File") is owned and distributed by Marvell
 * International Ltd. and/or its affiliates ("Marvell") under the following
 * alternative licensing terms.  Once you have made an election to distribute the
 * File under one of the following license alternatives, please (i) delete this
 * introductory statement regarding license alternatives, (ii) delete the three
 * license alternatives that you have not elected to use and (iii) preserve the
 * Marvell copyright notice above.
 *
 ********************************************************************************
 * Marvell Commercial License Option
 *
 * If you received this File from Marvell and you have entered into a commercial
 * license agreement (a "Commercial License") with Marvell, the File is licensed
 * to you under the terms of the applicable Commercial License.
 *
 ********************************************************************************
 * Marvell GPL License Option
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File in accordance with the terms and conditions of the General
 * Public License Version 2, June 1991 (the "GPL License"), a copy of which is
 * available along with the File in the license.txt file or by writing to the Free
 * Software Foundation, Inc., or on the worldwide web at http://www.gnu.org/licenses/gpl.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY
 * DISCLAIMED.  The GPL License provides additional details about this warranty
 * disclaimer.
 *
 ********************************************************************************
 * Marvell GNU General Public License FreeRTOS Exception
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File in accordance with the terms and conditions of the Lesser
 * General Public License Version 2.1 plus the following FreeRTOS exception.
 * An independent module is a module which is not derived from or based on
 * FreeRTOS.
 * Clause 1:
 * Linking FreeRTOS statically or dynamically with other modules is making a
 * combined work based on FreeRTOS. Thus, the terms and conditions of the GNU
 * General Public License cover the whole combination.
 * As a special exception, the copyright holder of FreeRTOS gives you permission
 * to link FreeRTOS with independent modules that communicate with FreeRTOS solely
 * through the FreeRTOS API interface, regardless of the license terms of these
 * independent modules, and to copy and distribute the resulting combined work
 * under terms of your choice, provided that:
 * 1. Every copy of the combined work is accompanied by a written statement that
 * details to the recipient the version of FreeRTOS used and an offer by yourself
 * to provide the FreeRTOS source code (including any modifications you may have
 * made) should the recipient request it.
 * 2. The combined work is not itself an RTOS, scheduler, kernel or related
 * product.
 * 3. The independent modules add significant and primary functionality to
 * FreeRTOS and do not merely extend the existing functionality already present in
 * FreeRTOS.
 * Clause 2:
 * FreeRTOS may not be used for any competitive or comparative purpose, including
 * the publication of any form of run time or compile time metric, without the
 * express permission of Real Time Engineers Ltd. (this is the norm within the
 * industry and is intended to ensure information accuracy).
 *
 ********************************************************************************
 * Marvell BSD License Option
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File under the following licensing terms.
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *	* Redistributions of source code must retain the above copyright notice,
 *	  this list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the distribution.
 *
 *	* Neither the name of Marvell nor the names of its contributors may be
 *	  used to endorse or promote products derived from this software without
 *	  specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************/

#ifndef __NETA_PP_IO_H_
#define __NETA_PP_IO_H_

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
