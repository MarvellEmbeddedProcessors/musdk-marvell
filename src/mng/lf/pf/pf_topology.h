/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
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
