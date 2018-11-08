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

#ifndef __MV_NETA_UTILS_H__
#define __MV_NETA_UTILS_H__

#include "mv_std.h"
#include "mv_net.h"
#include "mv_neta_ppio.h"
#include "utils.h"

/* pkt offset, must be multiple of 32 bytes */
#define MVAPPS_NETA_PKT_OFFS			64
/* pkt offset including Marvell header */
#define MVAPPS_NETA_PKT_EFEC_OFFS		(MVAPPS_NETA_PKT_OFFS + MV_MH_SIZE)
/* Maximum number of CPU cores used by application */
#define MVAPPS_NETA_MAX_NUM_CORES		2
/* Maximum number of ports used by application */
#define MVAPPS_NETA_MAX_NUM_PORTS		2
/* Maximum number of queues per TC */
#define MVAPPS_NETA_MAX_NUM_QS_PER_TC	MVAPPS_NETA_MAX_NUM_CORES


/*
 * Tx shadow queue entry
 */
struct tx_shadow_q_entry {
	struct neta_buff_inf	buff_ptr;	/* pointer to the buffer object */
	u8			rx_port;
	u8			rx_queue;
};

/*
 * Tx shadow queue
 */
struct tx_shadow_q {
	u16				read_ind;	/* read index */
	u16				write_ind;	/* write index */

	struct tx_shadow_q_entry	*ents;		/* array of entries */
};

/*
 * General port parameters
 */
struct port_desc {
	char			 name[15];	/* Port name */
	int			 initialized;	/* Flag indicated is port was initialized */
	int			 ppio_id;	/* PPIO port ID */
	u32			 first_inq;	/* First RXQ - relative to the Port's first RXQ */
	u16			 num_tcs;	/* Number of TCs */
	u16			 num_inqs[MVAPPS_NETA_MAX_NUM_QS_PER_TC]; /* Number of Rx queues */
	u16			 num_outqs;	/* Number of Tx queues */
	u32			 inq_size;	/* Rx queue size */
	u32			 outq_size;	/* Tx queue size */
	struct neta_ppio	*ppio;		/* PPIO object returned by neta_ppio_init() */
	struct neta_ppio_params	 port_params;	/* PPIO configuration parameters */

};

struct neta_counters {
	u32		rx_buf_cnt;
	u32		free_buf_cnt;
	u32		tx_buf_cnt;
	u32		tx_buf_drop;
	u64		tx_bytes_cnt;
	u64		rx_bytes_cnt;
};

/*
 * Local thread port parameters
 */
struct lcl_port_desc {
	int			 lcl_id;	/* Local thread ID*/
	int			 port_id;	/* Packet Processor ID */
	struct neta_ppio	*ppio;		/* PPIO object returned by pp2_ppio_init() */
	int			 num_shadow_qs;	/* Number of Tx shadow queues */
	int			 shadow_q_size;	/* Size of Tx shadow queue */
	struct tx_shadow_q	*shadow_qs;	/* Tx shadow queue */
	struct neta_counters	 cntrs;		/* Port statistics */
};

/* Macroes to handle counters */
#define INC_RX_BYTES_COUNT(lcl_port_desc, cnt)	((lcl_port_desc)->cntrs.rx_bytes_cnt += cnt)
#define INC_TX_BYTES_COUNT(lcl_port_desc, cnt)	((lcl_port_desc)->cntrs.tx_bytes_cnt += cnt)
#define INC_RX_COUNT(lcl_port_desc, cnt)	((lcl_port_desc)->cntrs.rx_buf_cnt += cnt)
#define INC_TX_COUNT(lcl_port_desc, cnt)	((lcl_port_desc)->cntrs.tx_buf_cnt += cnt)
#define INC_TX_DROP_COUNT(lcl_port_desc, cnt)	((lcl_port_desc)->cntrs.tx_buf_drop += cnt)
#define INC_FREE_COUNT(lcl_port_desc, cnt)	((lcl_port_desc)->cntrs.free_buf_cnt += cnt)

/*
 * Parse port pp_id and ppio_id from port name
 */
int app_find_port_info(struct port_desc *port_desc);

/*
 * Init port, This function doesn't enable port.
 */
int app_port_init(struct port_desc *port, u16 mtu, u16 pkt_offset);

/*
 * Enable port.
 */
int app_port_enable(struct port_desc *port);

/*
 * Init local port object per thread according to port parameters
 */
void app_port_local_init(int id, int lcl_id, struct lcl_port_desc *lcl_port, struct port_desc *port);
/*
 * Deinit all ports
 */
void app_deinit_all_ports(struct port_desc *ports, int num_ports);
/*
 * Deinit local port object
 */
void app_port_local_deinit(struct lcl_port_desc *lcl_port);
/*
 * Disable all ports
 */
void app_disable_all_ports(struct port_desc *ports, int num_ports);

/*
 * Show queue statistics
 */
void app_show_queue_stat(struct port_desc *port_desc, u8 tc, u8 q_start, int num_qs, int reset);
/*
 * Show port statistics
 */
void app_show_port_stat(struct port_desc *port_desc, int reset);
/*
 * Register common CLI commands (currently show queue and port statistics)
 */
int app_register_cli_common_cmds(struct glb_common_args *glb_args);
int register_cli_ftr_cmds(struct glb_common_args *garg);

/* Saved sysdma virtual high address*/
extern u64 neta_sys_dma_high_addr;

#endif /*__MVUTILS_H__*/

