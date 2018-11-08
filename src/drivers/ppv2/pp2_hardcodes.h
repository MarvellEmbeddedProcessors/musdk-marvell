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
