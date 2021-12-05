/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

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
