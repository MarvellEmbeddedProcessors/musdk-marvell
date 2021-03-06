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

#ifndef _VF_TOPOLOGY_H
#define _VF_TOPOLOGY_H

#include "std_internal.h"
#include "drivers/mv_net.h"
#include "drivers/mv_mqa.h"
#include "drivers/mv_mqa_queue.h"
#include "drivers/mv_giu.h"
#include "drivers/mv_giu_bpool.h"
#include "drivers/mv_giu_gpio.h"
#include "mng/mv_nmp.h"
#include "mng/lf/lf_mng.h"
#include "vf.h"
#include "vf_profile.h"

#define LCL		(1)
#define REM		(2)


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
	struct iomem_inf	 host_map;
	struct iomem_inf	 host_msix_map;
	enum func_type		 type;
};


/* Structure containing all the NIC-VF related data
 */
struct nmnicvf {
	struct nmlf			 nmlf;		/* will be used for inheritance */
	int				 vf_id;
	u32				 guest_id;
	int				 initialized;
	u8				 plat_bar_indx;
#define LINK_UP_MASK_REMOTE	0x1
#define LINK_UP_MASK_LOCAL	0x2
#define LINK_UP_MASK	(LINK_UP_MASK_REMOTE | LINK_UP_MASK_LOCAL)
	u8				 link_up_mask; /* 0x1 for remote flag, 0x2 for local; i.e. 0x3 link is up */
	int				 last_link_state;
	struct msix_table_entry		*msix_table_base;
	struct sys_iomem		*sys_iomem;	/* musdk iomem handle. */
	struct sys_iomem		*msix_iomem;	/* musdk msix iomem handle. */
	struct iomem_inf		 plat_regs;	/* Relevant only for platform devices */
	struct pci_plat_func_map	 map;		/* Memory mapping - PCI / Plat */
	struct giu			*giu;
	struct mqa			*mqa;		/* MQA */
	struct nmdisp			*nmdisp;	/* Dispatcher */
	struct vf_profile		 profile_data;	/* Profile */
	struct giu_gpio			*giu_gpio;	/* GIU Gpio */
	struct giu_bpool		*giu_bpools[GIU_GPIO_TC_MAX_NUM_BPOOLS];	/* GIU Bpools */
	struct giu_gpio_params		 gpio_params;		/* GIU Queue local */
	struct giu_gpio_rem_params	 gpio_rem_params;	/* GIU Queue Remy */
	struct giu_mng_ch		*giu_mng_ch;
	int				(*f_ready_cb)(void *arg, u8 lf_id);
	int				(*f_get_free_bar_cb)(void *arg, void **va, dma_addr_t *pa);
	void				(*f_put_bar_cb)(void *arg, int index);
	int				(*f_get_vf_bar_cb)(void *arg, u8 vf_id, u8 bar, void **va, void **pa);
	void				*arg;
};

int vf_outtc_queue_init(struct nmnicvf *nmnicvf, u32 type, u32 tc_num, u32 q_num);

/*
 *	vf_intc_queue_init
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
int vf_intc_queue_init(struct nmnicvf *nmnicvf, u32 type, u32 tc_num, u32 q_num);

/*
 *	vf_outtc_queue_free
 *
 *	This function release TC params object in queue topology
 *
 *	@param[in]	tc_type - traffic class type
 *	@param[in]	tc_num - number of traffic classes
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int vf_outtc_queue_free(struct nmnicvf *nmnicvf, u32 type, u32 tc_num);

/*
 *	vf_intc_queue_free
 *
 *	This function release TC params object in queue topology
 *
 *	@param[in]	tc_type - traffic class type
 *	@param[in]	tc_num - number of traffic classes
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int vf_intc_queue_free(struct nmnicvf *nmnicvf, u32 type, u32 tc_num);

/*
 *	vf_intc_bm_queue_init
 *
 *	This function initilaize BM params object in queue topology
 *
 *	@param[in]	bm_type - buffer pool type
 *	@param[in]	bm_num - number of buffer pools
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int vf_intc_bm_queue_init(struct nmnicvf *nmnicvf, u32 bm_num);

/*
 *	vf_intc_bm_queue_free
 *
 *	This function release BM params object in queue topology
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int vf_intc_bm_queue_free(struct nmnicvf *nmnicvf);

#endif /* _PF_TOPOLOGY_H */
