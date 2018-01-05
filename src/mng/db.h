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
#include "drivers/mv_mqa.h"
#include "drivers/mv_mqa_queue.h"
#include "lf/pf/pf_queue_topology.h"
#include "lf/pf/pf_profile.h"
#include "drivers/mqa/mqa_internal.h"
#include "drivers/mv_giu_gpio_init.h"
#include "hw_emul/gie.h"
#include "mng/mv_nmp.h"
#include "drivers/mv_pp2_ppio.h"

#define MAX_PCI_FUNC_NAME	256
#define MAX_PCI_FUNC_BARS	3

#define LCL	(1)
#define REM	(2)

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

/*
 * PPv2x BM pool descriptor parameters
 */
struct nmp_pp2_bpool_desc {
	struct pp2_bpool	*pool;		/* pointer to the bpool object */
	u32			 num_buffs;	/* number of buffers */
	u32			 buff_size;	/* buffer size */
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
	struct pp2_ppio		 *ppio;		/* PPIO object returned by pp2_ppio_init() */
	u32			 num_pools;
	struct nmp_pp2_bpool_desc *pools_desc;
};

/*
 * Structure containing the PPv2x related data
 */
struct pp2_data {
	u32				 num_ports;
	u32				 reserved_bpools;
	struct nmp_pp2_port_desc	*ports_desc;
};

/* Structure containing all the NIC-PF related data
 */
struct nmnicpf {
	int pf_id;
	u32 guest_id;
	struct pci_plat_func_map map;               /* Memory mapping - PCI / Plat */
	struct gie_data gie;                        /* GIE */
	struct pp2_data pp2;                        /* PP2 */
	struct mqa *mqa;                            /* MQA */
	struct nmdisp *nmdisp;                      /* Dispatcher */
	struct pf_profile profile_data;             /* Profile */
	struct giu_gpio *giu_gpio;                  /* GIU Gpio */
	struct giu_bpool *giu_bpool;                /* GIU Bpool */
	struct giu_gpio_init_params topology_data;  /* GIU Queue Topology */
	struct giu_mng_topology mng_data;           /* GIU Management Topology */
	struct giu_regfile regfile_data;            /* GIU Register File */
	void (*f_ready_cb)(void *arg);
	void *internal;
};

struct nmpp2 {
	int pp2_en;				/* Flag inidicating PP2 interface is present*/
	struct nmp_pp2_params pp2_params;	/* PP2 initialization params */
};

/* Structure containing all Custom LF related data
 */
struct nmcstm {
	int id;
	int pf_id;
	struct mqa *mqa;                            /* MQA */
	struct nmdisp *nmdisp;                      /* Dispatcher */
	struct mng_ch_params mng_ctrl;
};

/* Main PF data structure
 *
 *  nic-pf	all NIC-PF related data
 *  giu		stores GIU related data
 *  mqa_global	MQA tables parameters
 */
struct nmp {
	struct nmnicpf nmnicpf;
	struct nmcstm *nmcstm;
	u32 guest_id;
	struct mqa *mqa;
	struct nmdisp *nmdisp;
	struct nmpp2 nmpp2;
};

int pf_outtc_queue_init(u32 type, u32 tc_num, u32 q_num);
int pf_outtc_queue_free(u32 type, u32 tc_num);

int pf_intc_queue_init(u32 type, u32 tc_num, u32 q_num);
int pf_intc_queue_free(u32 type, u32 tc_num);
int pf_intc_bm_queue_init(u32 bm_num);
int pf_intc_bm_queue_free(void);

#endif /* _DB_H */
