/******************************************************************************
 *	Copyright (C) 2017 Marvell International Ltd.
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

#ifndef _DB_H
#define _DB_H

#include "drivers/giu_regfile_def.h"
#include "lf/pf/pf_queue_topology.h"
#include "lf/pf/pf_profile.h"
#include "drivers/mv_mqa_queue.h"
#include "drivers/mqa/mqa_internal.h"

#define MAX_PCI_FUNC_NAME	256
#define MAX_PCI_FUNC_BARS	3

#define LCL_EG_TC   (1)
#define LCL_ING_TC  (2)
#define HOST_EG_TC  (3)
#define HOST_ING_TC (4)

#define LCL_BM      (1)
#define HOST_BM     (2)

struct uio_mem {
	void *phys_addr;
	void *virt_addr;
};

enum func_type {
	ft_pcie_ep = 1,
	ft_plat
};

/* Contains the PCI / Platform function mapping information
 *
 *  sys_iomem	musdk iomem handle.
 *  cfg_map	Mapping of the device's configuration space.
 *		For PCIe case, this points to BAR-0, for local platform case
 *		then it points to the shared memory location in local dram.
 *  plat_regs	Relevant only for platform devices.
 *		Holds the mapping of the platform device configuration
 *		registers, associated with the platform device uio file.
 *  host_map	In PCIe case, holds the mapping of host memory from device's
 *		POV.
 *		In platform case, holds the mapping of local memory from
 *		user-space POV (which is actually an identity mapping for the
 *		physical address, and NA for virtual address as it's not being
 *		accessed by user-space).
 */
struct pci_plat_func_map {
	struct sys_iomem *sys_iomem;
	struct uio_mem cfg_map;
	struct uio_mem plat_regs;
	struct uio_mem host_map;
	enum func_type type;
};


/* Structure containing all the GIE related data
 *
 *  mng/rx/tx_gie	GIE device handles
 */
struct gie_data {
	struct gie *mng_gie;
	struct gie *rx_gie;
	struct gie *tx_gie;
};


/* Structure containing all the NIC-PF related data
 *
 *  map		Includes the mapping of the PCI function resources
 */
struct nic_pf {
	int pf_id;
	struct gie_data gie;
	struct mqa *mqa;
	struct nmdisp *nmdisp;
	struct pci_plat_func_map map;
	struct pf_profile profile_data;
	struct giu_regfile regfile_data;
	struct giu_mng_topology mng_data;
	struct giu_queue_topology topology_data;
	void *internal;
};

/* Main PF data structure
 *
 *  nic-pf	all NIC-PF related data
 *  giu		stores GIU related data
 *  mqa_global	MQA tables parameters
 */
struct nmp {
	struct nic_pf nic_pf;
	struct mqa *mqa;
	struct nmdisp *nmdisp;
};

struct giu_gpio_q {
	struct mqa_queue_params params;
	struct mqa_q *q;
};

int pf_tc_queue_init(u32 tc_type, u32 tc_num, u32 q_num);
int pf_tc_queue_free(u32 tc_type, u32 tc_num);
int pf_bm_queue_init(u32 bm_type, u32 bm_num);
int pf_bm_queue_free(u32 bm_type);
#endif /* _DB_H */
