/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#ifndef _DB_H
#define _DB_H

#include "std_internal.h"

#include "drivers/mv_mqa.h"
#include "drivers/mv_mqa_queue.h"
#include "lf/pf/pf_queue_topology.h"
#include "drivers/mv_giu.h"
#include "lf/pf/pf_profile.h"
#include "drivers/mv_giu_bpool.h"
#include "drivers/mv_giu_gpio.h"
#include "mng/mv_nmp.h"
#include "drivers/mv_pp2_ppio.h"
#include "drivers/mv_pp2_cls.h"

#define MAX_PCI_FUNC_NAME	256
#define MAX_PCI_FUNC_BARS	3

#define LCL	(1)
#define REM	(2)

#define MAX_PP2_CLS_TBL		10
#define PP2_CLS_KEY_MASK_STRING_FORMAT

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
	struct uio_mem cfg_map;
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

struct nmp_pp2_outq_desc {
	int	rate_limit_enable;
	struct pp2_ppio_rate_limit_params rate_limit;
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
	struct pp2_ppio		 *ppio;		/* PPIO object returned by pp2_ppio_init() */
	u32			 num_pools;
	struct nmp_pp2_bpool_desc *pools_desc;
	u8     rate_limit_enable;
	struct pp2_ppio_rate_limit_params rate_limit_params;
	struct nmp_pp2_outq_desc   q_desc[PP2_PPIO_MAX_NUM_OUTQS];
};

/*
 * Structure containing the PPv2x related data
 */
struct pp2_data {
	u32				 num_ports;
	u32				 reserved_bpools;
	struct nmp_pp2_port_desc	*ports_desc;
	struct pp2_cls_tbl		*tbl[MAX_PP2_CLS_TBL];
#ifdef PP2_CLS_KEY_MASK_STRING_FORMAT
	struct pp2_cls_tbl_params	tbl_params[MAX_PP2_CLS_TBL];
#endif /* PP2_CLS_KEY_MASK_STRING_FORMAT */
};

/*
 * Structure containing all the LF related data
 */
struct nmlf {
	int id;
	int (*f_maintenance_cb)(struct nmlf *nmlf);
	int (*f_serialize_cb)(struct nmlf *nmlf, char *buff, u32 size);
};

/* Structure containing all the NIC-PF related data
 */
struct nmnicpf {
	struct nmlf nmlf;			/* will be used for inheritance */
	int pf_id;
	u32 guest_id;
	int initialized;
	u8 plat_bar_indx;
	struct sys_iomem *sys_iomem;                /* musdk iomem handle. */
	struct uio_mem plat_regs;                   /* Relevant only for platform devices */
	struct pci_plat_func_map map;               /* Memory mapping - PCI / Plat */
	struct nmp *nmp;
	struct giu *giu;
	struct pp2_data pp2;                        /* PP2 */
	struct pp2_ppio_statistics stats;	    /* PP2 Statistics */
	struct mqa *mqa;                            /* MQA */
	struct nmdisp *nmdisp;                      /* Dispatcher */
	struct pf_profile profile_data;             /* Profile */
	struct giu_gpio *giu_gpio;                  /* GIU Gpio */
	struct giu_bpool *giu_bpools[GIU_GPIO_TC_MAX_NUM_BPOOLS];  /* GIU Bpools */
	struct giu_gpio_params gpio_params;  /* GIU Queue Topology */
	struct giu_mng_ch *giu_mng_ch;
	void (*f_ready_cb)(void *arg);
	void *arg;
};

struct nmpp2 {
	int pp2_en;				/* Flag inidicating PP2 interface is present*/
	struct nmp_pp2_params pp2_params;	/* PP2 initialization params */
};

/* Structure containing all Custom LF related data
 */
struct nmcstm {
	struct nmlf nmlf;			/* will be used for inheritance */
	int id;
	int pf_id;
	struct mqa *mqa;                            /* MQA */
	struct nmdisp *nmdisp;                      /* Dispatcher */
	struct mng_ch_params mng_ctrl;
};

/* Main PF data structure
 *
 *  nic-pf	all NIC-PF related data
 *
 *		Holds the mapping of the platform device configuration
 *		registers, associated with the platform device uio file.
 *  msi_regs	Mapping of the MSI-X registers.
 *		This ,apping is used for signaling the host if Ingress packets.
 *  giu		stores GIU related data
 *  mqa_global	MQA tables parameters
 */
struct nmp {
	struct nmnicpf nmnicpf;
	struct nmcstm *nmcstm;
	u32 guest_id;

	struct sys_iomem *msi_iomem;
	struct uio_mem msi_regs;
	int *emul_bars_avail_tbl;
	void *emul_bars_mem;
	struct mqa *mqa;
	struct nmdisp *nmdisp;
	struct nmpp2 nmpp2;
};

#endif /* _DB_H */
