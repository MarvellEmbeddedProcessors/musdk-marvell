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

#ifndef _PF_TOPOLOGY_H
#define _PF_TOPOLOGY_H

#include "std_internal.h"
#include "drivers/mv_net.h"
#include "drivers/mv_mqa.h"
#include "drivers/mv_mqa_queue.h"
#include "drivers/mv_giu.h"
#include "drivers/mv_giu_bpool.h"
#include "drivers/mv_giu_gpio.h"
#include "drivers/mv_pp2_ppio.h"
#include "drivers/mv_pp2_cls.h"
#include "mng/mv_nmp.h"
#include "mng/lf/lf_mng.h"
#include "pf.h"
#include "pf_profile.h"

#define LCL		(1)
#define REM		(2)

#define MAX_PP2_CLS_TBL		10
#define PP2_CLS_KEY_MASK_STRING_FORMAT

enum func_type {
	ft_pcie_ep = 1,
	ft_plat
};

struct iomem_inf {
	void *phys_addr;
	void *virt_addr;
};

struct msix_table_entry {
	u64 msg_addr;
	u32 msg_data;
	u32 vector_ctrl;
};

/* Contains the PCI / Platform function mapping information
 *
 *  cfg_map	Mapping of the device's configuration space.
 *		For PCIe case, this points to BAR-0, for local platform case
 *		then it points to the shared memory location in local dram.
 *  host_map	In PCIe case, holds the mapping of host memory from device's
 *		POV.
 *		In platform case, holds the mapping of local memory from
 *		user-space POV (which is actually an identity mapping for the
 *		physical address, and NA for virtual address as it's not being
 *		accessed by user-space).
 */
struct pci_plat_func_map {
	struct iomem_inf	 cfg_map;
	struct iomem_inf	 bar2_map;
	struct iomem_inf	 host_map;
	enum func_type		 type;
};

/*
 * PPv2x BM pool descriptor parameters
 */
struct nmp_pp2_bpool_desc {
	struct pp2_bpool	*pool;		/* pointer to the bpool object */
	u32			 num_buffs;	/* number of buffers */
	u32			 buff_size;	/* buffer size */
};

struct nmp_pp2_outq_desc {
	int					 rate_limit_enable;
	struct pp2_ppio_rate_limit_params	 rate_limit;
};

/*
 * PPv2x port descriptor parameters
 */
struct nmp_pp2_port_desc {
	u32			 pp_id;		/* Packet Processor ID */
	u32			 ppio_id;	/* PPIO port ID */
	enum pp2_ppio_type	 ppio_type;	/* PPIO type */
	u32			 first_inq;	/* First RXQ - relative to the Port's first RXQ */
	u16			 max_num_tcs;	/* Maximum number of TCs */
	u16			 num_tcs;	/* Number of TCs */
	u16			 num_inqs[PP2_PPIO_MAX_NUM_TCS];	/* Number of Rx queues per TC*/
	u16			 num_outqs;	/* Number of Tx queues */
	u32			 inq_size;	/* Rx queue size */
	u32			 outq_size;	/* Tx queue size */
	u32			 hash_type;	/* Hash type */
	u32			 first_rss_tbl;	/* First RSS table */
	u32			 pkt_offst;	/* Packet Processor ID */
	u32			 link_state;	/* Port link state */
	struct pp2_ppio		*ppio;		/* PPIO object returned by pp2_ppio_init() */
	u32			 num_pools;
	struct nmp_pp2_bpool_desc	*pools_desc;
	struct nmp_pp2_outq_desc	 q_desc[PP2_PPIO_MAX_NUM_OUTQS];
	u8					 rate_limit_enable;
	struct pp2_ppio_rate_limit_params	 rate_limit_params;
};

/*
 * Structure containing the PPv2x related data
 */
struct pp2_data {
	u32				 num_ports;
	struct nmp_pp2_port_desc	*ports_desc;
	struct pp2_cls_tbl		*tbl[MAX_PP2_CLS_TBL];
#ifdef PP2_CLS_KEY_MASK_STRING_FORMAT
	struct pp2_cls_tbl_params	 tbl_params[MAX_PP2_CLS_TBL];
#endif /* PP2_CLS_KEY_MASK_STRING_FORMAT */
};

/* Structure containing all the NIC-PF related data
 */
struct nmnicpf {
	struct nmlf			 nmlf;		/* will be used for inheritance */
	int				 pf_id;
	u32				 guest_id;
	int				 initialized;
#define LINK_UP_MASK_REMOTE	0x1
#define LINK_UP_MASK_LOCAL	0x2
#define LINK_UP_MASK_LOCAL_PP2	0x4
#define LINK_UP_MASK		(LINK_UP_MASK_REMOTE | LINK_UP_MASK_LOCAL)
#define LINK_UP_MASK_W_PP2	(LINK_UP_MASK | LINK_UP_MASK_LOCAL_PP2)
	u8				 link_up_mask;
	int				 last_link_state;
	u8				 plat_bar_indx;
	struct msix_table_entry		*msix_table_base;
	struct sys_iomem		*sys_iomem;	/* musdk iomem handle. */
	struct sys_iomem		*msix_iomem;	/* musdk msix iomem handle. */
	struct iomem_inf		 plat_regs;	/* Relevant only for platform devices */
	struct pci_plat_func_map	 map;		/* Memory mapping - PCI / Plat */
	struct giu			*giu;
	struct pp2_data			 pp2;		/* PP2 */
	struct pp2_ppio_statistics	 stats;		/* PP2 Statistics */
	struct mqa			*mqa;		/* MQA */
	struct nmdisp			*nmdisp;	/* Dispatcher */
	struct pf_profile		 profile_data;	/* Profile */
	struct giu_gpio			*giu_gpio;	/* GIU Gpio */
	struct giu_bpool		*giu_bpools[GIU_GPIO_TC_MAX_NUM_BPOOLS];	/* GIU Bpools */
	struct giu_gpio_params		 gpio_params;		/* GIU Queue local */
	struct giu_gpio_rem_params	 gpio_rem_params;	/* GIU Queue Remy */
	struct giu_mng_ch		*giu_mng_ch;
	int				(*f_ready_cb)(void *arg, u8 lf_id);
	int				(*f_get_free_bar_cb)(void *arg, void **va, dma_addr_t *pa);
	void				(*f_put_bar_cb)(void *arg, int index);
	int				(*f_pp_find_free_bpool_cb)(void *arg, u32 pp_id);
	int				(*f_set_vf_bar_offset_base_cb)(void *arg, u8 bar, u64 phys_addr, u64 virt_addr);
	void				*arg;
};

int pf_outtc_queue_init(struct nmnicpf *nmnicpf, u32 type, u32 tc_num, u32 q_num);

/*
 *	pf_intc_queue_init
 *
 *	This function initilaize TC params object in queue topology
 *
 *	@param[in]	tc_type - traffic class type
 *	@param[in]	tc_num - number of traffic classes
 *	@param[in]	q_num - number of queues in traffic class
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int pf_intc_queue_init(struct nmnicpf *nmnicpf, u32 type, u32 tc_num, u32 q_num);

/*
 *	pf_outtc_queue_free
 *
 *	This function release TC params object in queue topology
 *
 *	@param[in]	tc_type - traffic class type
 *	@param[in]	tc_num - number of traffic classes
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int pf_outtc_queue_free(struct nmnicpf *nmnicpf, u32 type, u32 tc_num);

/*
 *	pf_intc_queue_free
 *
 *	This function release TC params object in queue topology
 *
 *	@param[in]	tc_type - traffic class type
 *	@param[in]	tc_num - number of traffic classes
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int pf_intc_queue_free(struct nmnicpf *nmnicpf, u32 type, u32 tc_num);

/*
 *	pf_intc_bm_queue_init
 *
 *	This function initilaize BM params object in queue topology
 *
 *	@param[in]	bm_type - buffer pool type
 *	@param[in]	bm_num - number of buffer pools
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int pf_intc_bm_queue_init(struct nmnicpf *nmnicpf, u32 bm_num);

/*
 *	pf_intc_bm_queue_free
 *
 *	This function release BM params object in queue topology
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int pf_intc_bm_queue_free(struct nmnicpf *nmnicpf);

#endif /* _PF_TOPOLOGY_H */
