/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#define log_fmt(fmt, ...) "vf#%d: " fmt, nmnicvf->vf_id, ##__VA_ARGS__

#include "std_internal.h"
#include "drivers/mv_mqa.h"
#include "drivers/mv_mqa_queue.h"
#include "drivers/mv_giu.h"
#include "drivers/mv_giu_gpio.h"
#include "drivers/mqa_def.h"
#include "mng/mv_nmp.h"
#include "mng/mv_nmp_guest_msg.h"
#include "mng/mv_nmp_dispatch.h"

#include "mng/include/guest_mng_cmd_desc.h"
#include "mng/pci_ep_def.h"
#include "mng/lf/mng_cmd_desc.h"

#include "vf.h"
#include "vf_topology.h"
#include "vf_pci_if_desc.h"
#include "env/trace/trc_vf.h"


#define PLAT_AGNIC_UIO_NAME	"agnic"

/* TODO: These should be removed. The local queue sizes should match the remote
 * management queue sizes, as received during the init sequence.
 */
#define LOCAL_CMD_QUEUE_SIZE	256
#define LOCAL_NOTIFY_QUEUE_SIZE	256

static int nmnicvf_init_host_ready(struct nmlf *nmlf);

static int init_nicvf_params(struct nmnicvf *nmnicvf, struct nmp_lf_nicvf_params *params)
{
	struct vf_profile		*vf_profile;
	int				 k;

	vf_profile = &(nmnicvf->profile_data);

	/* TODO - return error once all sync with this change */
	if (!params->match)
		pr_warn("GPIO match should be given\n");
	else
		strcpy(vf_profile->match, params->match);
	vf_profile->pci_en	       = params->pci_en;
	vf_profile->sg_en	       = params->sg_en;
	vf_profile->max_num_tcs        = params->max_num_tcs;
	vf_profile->lcl_egress_q_num   = params->lcl_egress_num_qs;
	vf_profile->lcl_egress_q_size  = params->lcl_egress_qs_size;
	vf_profile->lcl_ingress_q_num  = params->lcl_ingress_num_qs;
	vf_profile->lcl_ingress_q_size = params->lcl_ingress_qs_size;
	vf_profile->lcl_bp_num       = params->lcl_num_bpools;
	if (vf_profile->sg_en && vf_profile->lcl_bp_num > 1) {
		pr_err("Currently if s/g support is enabled, only one pool is allowed\n");
		return -EINVAL;
	}
	for (k = 0; k < vf_profile->lcl_bp_num; k++) {
		vf_profile->lcl_bp_params[k].lcl_bp_size      = params->lcl_bpools_params[k].max_num_buffs;
		vf_profile->lcl_bp_params[k].lcl_bp_buf_size  = params->lcl_bpools_params[k].buff_size;
	}
	vf_profile->dflt_pkt_offset = params->dflt_pkt_offset;

	return 0;
}

/**
 * NIC VF Initialization Section
 * =============================
 */

/*
 *	nmnicvf_topology_local_queue_init
 *
 *	This function initialize local queues in NIC VF queue
 *	topology database based on configuration profile
 *	Local queues include data queues and bm queues
 *
 *	@param[in]	nmnicvf - pointer to NIC VF object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
static int nmnicvf_topology_local_queue_init(struct nmnicvf *nmnicvf)
{
	int ret;
	struct vf_profile *prof = &(nmnicvf->profile_data);
	struct giu_gpio_params *gpio_p = &(nmnicvf->gpio_params);

	pr_debug("Initializing Local Queues in management Database\n");

	/* Local Egress TC */
	ret = vf_intc_queue_init(nmnicvf, LCL, gpio_p->num_intcs, prof->lcl_egress_q_num);
	if (ret) {
		pr_err("Failed to allocate Local Egress TC table\n");
		goto queue_error;
	}

	/* Local ingress TC */
	ret = vf_outtc_queue_init(nmnicvf, LCL, gpio_p->num_outtcs, prof->lcl_ingress_q_num);
	if (ret) {
		pr_err("Failed to allocate Local Ingress TC table\n");
		goto queue_error;
	}

	/* Local BM */
	ret = vf_intc_bm_queue_init(nmnicvf, prof->lcl_bp_num);
	if (ret) {
		pr_err("Failed to allocate Local BM table\n");
		goto queue_error;
	}

	return 0;

queue_error:
	vf_intc_queue_free(nmnicvf, LCL, gpio_p->num_intcs);
	vf_outtc_queue_free(nmnicvf, LCL, gpio_p->num_outtcs);
	vf_intc_bm_queue_free(nmnicvf);

	return -ENOMEM;
}

/*
 *	nmnicvf_topology_local_tc_free
 *
 *	This function frees the local TC resources in topology
 */
static void nmnicvf_topology_local_tc_free(struct nmnicvf *nmnicvf)
{
	struct giu_gpio_params *gpio_p = &(nmnicvf->gpio_params);

	pr_debug("Free Local queues DB\n");
	vf_intc_queue_free(nmnicvf, LCL, gpio_p->num_intcs);
	vf_outtc_queue_free(nmnicvf, LCL, gpio_p->num_outtcs);
	vf_intc_bm_queue_free(nmnicvf);
}

/*
 *	nmnicvf_topology_remote_queue_init
 *
 *	This function initialize NIC VF remote queue
 *	topology database based on VF_INIT management command
 *	Remote queues include data queues and bm queues
 *
 *	@param[in]	nmnicvf - pointer to NIC VF object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
static int nmnicvf_topology_remote_queue_init(struct nmnicvf *nmnicvf)
{
	int ret;
	struct giu_gpio_rem_params *gpio_p = &(nmnicvf->gpio_rem_params);

	pr_debug("Initializing Remote Queues in management Database\n");

	/* Remote Egress TC */
	/* TC queues will be update upon "tc_add" command */
	ret = vf_intc_queue_init(nmnicvf, REM, gpio_p->num_intcs, 0);
	if (ret) {
		pr_err("Failed to allocate Local Egress TC table\n");
		goto queue_error;
	}

	/* Remote ingress TC */
	/* TC queues will be update upon "tc_add" command */
	ret = vf_outtc_queue_init(nmnicvf, REM, gpio_p->num_outtcs, 0);
	if (ret) {
		pr_err("Failed to allocate Local Ingress TC table\n");
		goto queue_error;
	}

	return 0;

queue_error:

	pr_err("Remote Queues Initialization failed\n");

	vf_intc_queue_free(nmnicvf, REM, gpio_p->num_intcs);
	vf_outtc_queue_free(nmnicvf, REM, gpio_p->num_outtcs);

	return -ENOMEM;
}

/*
 *	nmnicvf_topology_remote_tc_free
 *
 *	This function frees the remote TC resources in topology
 */
static int nmnicvf_topology_remote_tc_free(struct nmnicvf *nmnicvf)
{
	struct giu_gpio_rem_params *gpio_p = &(nmnicvf->gpio_rem_params);

	pr_debug("Free Remote queues DB\n");
	vf_intc_queue_free(nmnicvf, REM, gpio_p->num_intcs);
	vf_outtc_queue_free(nmnicvf, REM, gpio_p->num_outtcs);

	return 0;
}

/*
 *	nmnicvf_topology_tc_free
 *
 *	This function frees both local & remote TC resources in DB
 */
static int nmnicvf_topology_tc_free(struct nmnicvf *nmnicvf)
{
	/* Free Local TC structures */
	nmnicvf_topology_local_tc_free(nmnicvf);

	/* Free Remote TC structures */
	nmnicvf_topology_remote_tc_free(nmnicvf);

	return 0;
}

/*
 *	nmnicvf_topology_local_queue_cfg
 *
 *	This function create NIC VF local queue based on configuration profile
 *	Local queues include data queues and bm queues
 *
 *	@param[in]	nmnicvf - pointer to NIC VF object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
static int nmnicvf_topology_local_queue_cfg(struct nmnicvf *nmnicvf)
{
	struct giu_gpio_outtc_params *outtc;
	struct giu_gpio_intc_params *intc;
	struct vf_profile *prof = &(nmnicvf->profile_data);
	struct giu_gpio_params *gpio_p = &(nmnicvf->gpio_params);
	u32 tc_idx, bm_idx, q_idx;

	/* Create Local BM queues */
	pr_debug("Configure Local Egress TC queues (#TCs %d)\n", gpio_p->num_intcs);
	for (tc_idx = 0; tc_idx < gpio_p->num_intcs; tc_idx++) {
		intc = &(gpio_p->intcs_params[tc_idx]);

		pr_debug("Configure Local BM queues (Num of queues %d) of TC %d\n",
			intc->num_inpools, tc_idx);
		for (bm_idx = 0; bm_idx < intc->num_inpools; bm_idx++)
			/* Init queue parameters */
			intc->pools[bm_idx] = nmnicvf->giu_bpools[bm_idx];

		/* Create Local Egress TC queues */
		pr_debug("Configure Local Egress TC queues (Num of queues %d) of TC %d\n",
			intc->num_inqs, tc_idx);
		for (q_idx = 0; q_idx < intc->num_inqs; q_idx++)
			intc->inqs_params[q_idx].len  = prof->lcl_egress_q_size;
	}

	/* Create Local Ingress TC queues */
	pr_debug("Configure Local Ingress TC queues (#TCs %d)\n", gpio_p->num_outtcs);
	for (tc_idx = 0; tc_idx < gpio_p->num_outtcs; tc_idx++) {
		outtc = &(gpio_p->outtcs_params[tc_idx]);

		pr_debug("Configure Local Ingress TC queues (Num of queues %d) of TC %d\n",
			outtc->num_outqs, tc_idx);
		for (q_idx = 0; q_idx < outtc->num_outqs; q_idx++)
			outtc->outqs_params[q_idx].len  = prof->lcl_ingress_q_size;
	}

	return 0;
}

static int nmnicvf_mng_chn_host_ready(struct nmnicvf *nmnicvf)
{
	volatile struct pcie_config_mem *pcie_cfg;
	struct giu_mng_ch_params mng_ch_params;
	u64 vf_cfg_virt; /* pointer to HW so it should be volatile */
	int ret = 0;
	int timeout = 2000; /* 2 sec */

	/* Get BAR0 Configuration space base address */
	vf_cfg_virt = (u64)nmnicvf->map.cfg_map.virt_addr;
	pcie_cfg    = (struct pcie_config_mem *)vf_cfg_virt;

	pr_info("Waiting for Host-Mgmt to be Ready\n");

	do {
		if (readl(&pcie_cfg->status) & PCIE_CFG_STATUS_HOST_MGMT_READY)
			break;
		udelay(1000);
	} while (--timeout);

	if (!timeout) {
		pr_err("host mgmt is not ready. timeout exceeded.\n");
		return -ETIMEDOUT;
	}

	pr_info("Host-Mgmt is Ready\n");

	memset(&mng_ch_params, 0, sizeof(mng_ch_params));

	mng_ch_params.rem_base_pa = (dma_addr_t)nmnicvf->map.host_map.phys_addr;
	mng_ch_params.rem_base_va = nmnicvf->map.host_map.virt_addr;
	mng_ch_params.desc_size = sizeof(struct cmd_desc);

	mng_ch_params.lcl_cmd_q.len = LOCAL_CMD_QUEUE_SIZE;
	mng_ch_params.lcl_resp_q.len = LOCAL_NOTIFY_QUEUE_SIZE;
	mng_ch_params.rem_cmd_q.len = pcie_cfg->cmd_q.len;
	mng_ch_params.rem_cmd_q.pa = pcie_cfg->cmd_q.q_addr;
	mng_ch_params.rem_cmd_q.prod_pa =
		(dma_addr_t)(pcie_cfg->cmd_q.q_prod_offs + nmnicvf->map.cfg_map.phys_addr);
	mng_ch_params.rem_cmd_q.cons_pa =
		(dma_addr_t)(pcie_cfg->cmd_q.q_cons_offs + nmnicvf->map.cfg_map.phys_addr);
	mng_ch_params.rem_cmd_q.prod_va =
		(void *)(pcie_cfg->cmd_q.q_prod_offs + nmnicvf->map.cfg_map.virt_addr);
	mng_ch_params.rem_cmd_q.cons_va =
		(void *)(pcie_cfg->cmd_q.q_cons_offs + nmnicvf->map.cfg_map.virt_addr);

	mng_ch_params.rem_resp_q.len = pcie_cfg->notif_q.len;
	mng_ch_params.rem_resp_q.pa = pcie_cfg->notif_q.q_addr;
	mng_ch_params.rem_resp_q.prod_pa =
		(dma_addr_t)(pcie_cfg->notif_q.q_prod_offs + nmnicvf->map.cfg_map.phys_addr);
	mng_ch_params.rem_resp_q.cons_pa =
		(dma_addr_t)(pcie_cfg->notif_q.q_cons_offs + nmnicvf->map.cfg_map.phys_addr);
	mng_ch_params.rem_resp_q.prod_va =
		(void *)(pcie_cfg->notif_q.q_prod_offs + nmnicvf->map.cfg_map.virt_addr);
	mng_ch_params.rem_resp_q.cons_va =
		(void *)(pcie_cfg->notif_q.q_cons_offs + nmnicvf->map.cfg_map.virt_addr);

	ret = giu_mng_ch_init(nmnicvf->giu, &mng_ch_params, &nmnicvf->giu_mng_ch);
	if (ret) {
		pr_err("faied to initialize GIU Management channel!\n");
		return ret;
	}

	/* Device Ready */
	/* ============ */

	/* make sure all writes are done before updating the status */
	writel(readl(&pcie_cfg->status) | PCIE_CFG_STATUS_DEV_MGMT_READY, &pcie_cfg->status);

	/* Set state to 'Device Management Ready' */
	pr_info("Set status to 'Device Management Ready'\n");

	return 0;
}

static int nmnicvf_mng_chn_init(struct nmnicvf *nmnicvf)
{
	volatile struct pcie_config_mem *pcie_cfg;
	u64 vf_cfg_virt; /* pointer to HW so it should be volatile */
	u8 mac_addr[ETH_ALEN] = {0};

	/*  Host Ready Check */
	/* ================= */

	/* Get BAR0 Configuration space base address */
	vf_cfg_virt = (u64)nmnicvf->map.cfg_map.virt_addr;
	pcie_cfg    = (struct pcie_config_mem *)vf_cfg_virt;

	pcie_cfg->vf_id = nmnicvf->vf_id;
	pcie_cfg->mac_addr[0] = mac_addr[0];
	pcie_cfg->mac_addr[1] = mac_addr[1];
	pcie_cfg->mac_addr[2] = mac_addr[2];
	pcie_cfg->mac_addr[3] = mac_addr[3];
	pcie_cfg->mac_addr[4] = mac_addr[4];
	pcie_cfg->mac_addr[5] = mac_addr[5];

	/* update the total memory needed for the device side */
	pcie_cfg->dev_use_size = PCI_BAR0_DEV_RES_BASE;

	return 0;
}

/*
 * nmnicvf_config_plat_func - Setup the platofm device registers to trigger
 * operation of the kernel based driver.
 */
static int nmnicvf_config_plat_func(struct nmnicvf *nmnicvf)
{
	u32				 cfg_mem_addr[2]; /* High and low */
	void				*cfg_mem_va;
	dma_addr_t			 phys_addr =
		(dma_addr_t)(uintptr_t)nmnicvf->map.cfg_map.phys_addr;

	pr_debug("Setting platform device registers.\n");

	/* TODO: need to read the correct address directly from DTS */
	cfg_mem_va = nmnicvf->plat_regs.virt_addr + CFG_MEM_AP8xx_OFFS;

	/* first, let's try to access the higher bits to make sure we can use 64bits */
	cfg_mem_addr[1] = CFG_MEM_64B_HI_MAGIC_VAL;
	writel(cfg_mem_addr[1], cfg_mem_va + 0x4);
	if (readl(cfg_mem_va + 0x4)) {
		if (phys_addr & ~CFG_MEM_64B_ADDR_MASK) {
			pr_err("Invalid BAR psysical address (0x%"PRIx64")!\n", phys_addr);
			return -EFAULT;
		}
		/* it seems like we can use 64bits */
		cfg_mem_addr[0]  = lower_32_bits(phys_addr) | CFG_MEM_VALID;
		cfg_mem_addr[0] |=
			((nmnicvf->plat_bar_indx & CFG_MEM_BIDX_MASK) << CFG_MEM_BIDX_SHIFT);
		cfg_mem_addr[1] |= upper_32_bits(phys_addr);
		pr_debug("passing 64bits reg: pa: %"PRIx64", bar-idx: %d (reg: %"PRIx32"%"PRIx32")\n",
			phys_addr, nmnicvf->plat_bar_indx, cfg_mem_addr[1], cfg_mem_addr[0]);

		writel(cfg_mem_addr[0], cfg_mem_va);
		writel(cfg_mem_addr[1], cfg_mem_va + 0x4);
	} else {
		u32 new_phys_addr;

		if (phys_addr & ~CFG_MEM_32B_ADDR_MASK) {
			pr_err("Invalid BAR psysical address (0x%"PRIx64")!\n", phys_addr);
			return -EFAULT;
		}

		/* make the address 32bits */
		new_phys_addr = (u32)(phys_addr >> CFG_MEM_32B_ADDR_SHIFT);
		/* it seems like we cannot use 64bits; use 43bits */
		pr_debug("cfg mem sync reg is not 64bits; trying 32bits\n");
		cfg_mem_addr[0]  = new_phys_addr | CFG_MEM_VALID;
		cfg_mem_addr[0] |=
			((nmnicvf->plat_bar_indx & CFG_MEM_BIDX_MASK) << CFG_MEM_BIDX_SHIFT);
		pr_debug("passing 32bits reg: pa: %"PRIx64", bar-idx: %d (reg: %"PRIx32")\n",
			phys_addr, nmnicvf->plat_bar_indx, cfg_mem_addr[0]);

		writel(cfg_mem_addr[0], cfg_mem_va);
	}

	return 0;
}

static int nmnicvf_map_emul_pci_bar(struct nmnicvf *nmnicvf)
{
	nmnicvf->plat_bar_indx = nmnicvf->f_get_free_bar_cb(nmnicvf->arg,
			&nmnicvf->map.cfg_map.virt_addr,
			(dma_addr_t *)&nmnicvf->map.cfg_map.phys_addr);

	if (nmnicvf->plat_bar_indx < 0) {
		pr_err("no emulated-BARs left!\n");
		return -ENAVAIL;
	}

	/* Clear the config space, to prevent false device indications. */
	memset(nmnicvf->map.cfg_map.virt_addr, 0x0, PCI_BAR0_ALLOC_SIZE);

	pr_debug("Platform config space @ %p.\n", nmnicvf->map.cfg_map.phys_addr);
	/* Setup the "host_map", which is actually an identity mapping for the
	 * physical address.
	 * The virtual map, is not needed, as not entity in the mgmt side is
	 * accessing the virtual addresses in the "host" side, all accesses are
	 * done using a DMA.
	 * Thus, we set the virtual address to some "bad address" so that we
	 * can identify faulty accesses to host's virtual space.
	 */
	nmnicvf->map.host_map.phys_addr = 0x0;
	nmnicvf->map.host_map.virt_addr = (void *)0xBAD00ADD0BAD0ADDll;

	/* Configure device registers. */
	return nmnicvf_config_plat_func(nmnicvf);
}

static int nmnicvf_map_plat_func(struct nmnicvf *nmnicvf)
{
	struct sys_iomem_params iomem_params;
	int ret;

	pr_debug("Mapping function %s\n", PLAT_AGNIC_UIO_NAME);

	iomem_params.type = SYS_IOMEM_T_UIO;
	iomem_params.devname = PLAT_AGNIC_UIO_NAME;
	iomem_params.index = 0;

	ret = sys_iomem_init(&iomem_params, &nmnicvf->sys_iomem);
	if (ret)
		return ret;

	/* Map the agnic configuration registers */
	ret = sys_iomem_map(nmnicvf->sys_iomem, "agnic_regs", (phys_addr_t *)&nmnicvf->plat_regs.phys_addr,
			&nmnicvf->plat_regs.virt_addr);
	if (ret) {
		sys_iomem_deinit(nmnicvf->sys_iomem);
		return ret;
	}

	pr_debug("agnic regs mapped at virt:%p phys:%p\n",
		nmnicvf->plat_regs.virt_addr, nmnicvf->plat_regs.phys_addr);

	return nmnicvf_map_emul_pci_bar(nmnicvf);
}

static int nmnicvf_map_pci_bar(struct nmnicvf *nmnicvf)
{
	struct sys_iomem_params iomem_params;
	u8 barid;
	int ret;

	pr_debug("Mapping function %s\n", PCI_EP_UIO_MEM_NAME);
	iomem_params.type = SYS_IOMEM_T_UIO;
	iomem_params.devname = PCI_EP_UIO_MEM_NAME;
	iomem_params.index = 0;

	ret = sys_iomem_init(&iomem_params, &nmnicvf->sys_iomem);
	if (ret)
		return ret;

	/* PCI configuration space */
	barid = 2;
	ret = nmnicvf->f_get_vf_bar_cb(nmnicvf->arg,
				       nmnicvf->vf_id,
				       barid,
				       &nmnicvf->map.cfg_map.virt_addr,
				       &nmnicvf->map.cfg_map.phys_addr);
	if (ret) {
		pr_err("failed to get BAR#%d address\n", barid);
		sys_iomem_deinit(nmnicvf->sys_iomem);
		return ret;
	}

	pr_info("BAR#%d mapped at virt:%p phys:%p\n", barid,
		nmnicvf->map.cfg_map.virt_addr, nmnicvf->map.cfg_map.phys_addr);

	/* MSI-X space */
	barid = 0;
	ret = nmnicvf->f_get_vf_bar_cb(nmnicvf->arg,
				       nmnicvf->vf_id,
				       barid,
				       &nmnicvf->map.host_msix_map.virt_addr,
				       &nmnicvf->map.host_msix_map.phys_addr);
	if (ret) {
		pr_err("failed to get BAR#%d address\n", barid);
		sys_iomem_deinit(nmnicvf->sys_iomem);
		return ret;
	}

	pr_info("BAR#%d mapped at virt:%p phys:%p\n", barid,
		nmnicvf->map.host_msix_map.virt_addr, nmnicvf->map.host_msix_map.phys_addr);

	nmnicvf->msix_table_base = (struct msix_table_entry *)nmnicvf->map.host_msix_map.virt_addr;

	/* Map the whole physical Packet Processor physical address */
	ret = sys_iomem_map(nmnicvf->sys_iomem, "host-map", (phys_addr_t *)&nmnicvf->map.host_map.phys_addr,
			&nmnicvf->map.host_map.virt_addr);
	if (ret) {
		sys_iomem_deinit(nmnicvf->sys_iomem);
		return ret;
	}

	iomem_params.devname = "MSIX";
	iomem_params.index = 0;
	iomem_params.type = SYS_IOMEM_T_MMAP;
	ret = sys_iomem_init(&iomem_params, &nmnicvf->msix_iomem);
	if (ret)
		return ret;

#ifdef PCI_EP_PLATFORM_MODE
	/* In this mode, the remote side is still running on the same platform
	 * but above the VF BAR setting.
	 * So the host_map_phys_base MUST be '0' to retrieve the correct "remote" address
	 */
	nmnicvf->map.host_map.phys_addr = 0x0;
	nmnicvf->map.host_map.virt_addr = (void *)0xBAD00ADD0BAD0ADDll;
#endif /* PCI_EP_PLATFORM_MODE */
	pr_debug("host RAM of %s remapped to phys %p virt %p\n", "host-map",
		   nmnicvf->map.host_map.phys_addr, nmnicvf->map.host_map.virt_addr);

	return 0;
}

static int nmnicvf_map_init(struct nmnicvf *nmnicvf)
{
	int ret;

	if (nmnicvf->profile_data.pci_en) {
		ret = nmnicvf_map_pci_bar(nmnicvf);
		if (ret) {
			pr_err("PCI device not found.\n");
			return ret;
		}
		nmnicvf->map.type = ft_pcie_ep;
	} else {
		ret = nmnicvf_map_plat_func(nmnicvf);
		if (ret) {
			pr_err("Platform device not found.\n");
			return ret;
		}
		nmnicvf->map.type = ft_plat;
	}

	return 0;
}

/**
 * NIC VF Termination Section
 * ==========================
 */

/*
 *	nmnicvf_local_queue_terminate
 *
 *	This function terminate NIC VF local queue based on configuration profile
 *	Local queues include data queues and bm queues
 *
 *	@param[in]	nmnicvf - pointer to NIC VF object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
static int nmnicvf_local_queue_terminate(struct nmnicvf *nmnicvf)
{
	nmnicvf = nmnicvf;

	return 0;
}

/*
 *	nmnicvf_mng_chn_terminate
 *
 *	This function terminate NIC VF management channel
 *	Execution requires handshake with Host side
 *
 *	@param[in]	nmnicvf - pointer to NIC VF object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
static int nmnicvf_mng_chn_terminate(struct nmnicvf *nmnicvf)
{
	giu_mng_ch_deinit(nmnicvf->giu_mng_ch);
	nmnicvf->giu_mng_ch = NULL;
	return 0;
}

static void nmnicvf_unmap_plat_func(struct nmnicvf *nmnicvf)
{
	mv_sys_dma_mem_free(nmnicvf->map.cfg_map.virt_addr);
}

static void nmnicvf_unmap_emul_pci_bar(struct nmnicvf *nmnicvf)
{
	sys_iomem_unmap(nmnicvf->sys_iomem, "agnic_regs");
	sys_iomem_deinit(nmnicvf->sys_iomem);

	nmnicvf->f_put_bar_cb(nmnicvf->arg, nmnicvf->plat_bar_indx);
}

static void nmnicvf_unmap_pci_bar(struct nmnicvf *nmnicvf)
{
	sys_iomem_unmap(nmnicvf->sys_iomem, "host-map");
	sys_iomem_deinit(nmnicvf->sys_iomem);
}

static void nmnicvf_unmap_bar(struct nmnicvf *nmnicvf)
{
	if (nmnicvf->map.type == ft_plat)
		nmnicvf_unmap_emul_pci_bar(nmnicvf);
	else
		nmnicvf_unmap_pci_bar(nmnicvf);
}

static int nmnicvf_map_terminate(struct nmnicvf *nmnicvf)
{
	if (nmnicvf->map.type == ft_plat)
		nmnicvf_unmap_plat_func(nmnicvf);

	nmnicvf_unmap_bar(nmnicvf);

	return 0;
}

/**
 * NIC VF Command Processing Section
 * =================================
 */

/*
 *	nmnicvf_vf_init_command
 */
static int nmnicvf_vf_init_command(struct nmnicvf *nmnicvf,
				  struct mgmt_cmd_params *params,
				  struct mgmt_cmd_resp *resp_data)
{
	int ret;

	pr_debug("VF INIT\n");
	pr_debug("Num of - Ing TC %d, Eg TC %d\n",
				params->init.num_host_ingress_tc,
				params->init.num_host_egress_tc);

	if ((params->init.num_host_ingress_tc > nmnicvf->profile_data.max_num_tcs) ||
	    (params->init.num_host_egress_tc > nmnicvf->profile_data.max_num_tcs)) {
		pr_err("num host tcs must be smaller than %d\n", nmnicvf->profile_data.max_num_tcs);
		return -1;
	}

	/* Extract message params and update database */
	nmnicvf->gpio_rem_params.num_outtcs =
		nmnicvf->gpio_params.num_outtcs =
			params->init.num_host_ingress_tc;
	nmnicvf->gpio_rem_params.num_intcs =
		nmnicvf->gpio_params.num_intcs =
			params->init.num_host_egress_tc;

	/* Initialize remote queues database */
	ret = nmnicvf_topology_remote_queue_init(nmnicvf);
	if (ret) {
		pr_err("Failed to update remote DB queue info\n");
		return ret;
	}

	pr_debug("VF INIT, Done\n\n");

	return 0;
}

/*
 *	nmnicvf_egress_tc_add_command
 */
static int nmnicvf_egress_tc_add_command(struct nmnicvf *nmnicvf,
					struct mgmt_cmd_params *params,
					struct mgmt_cmd_resp *resp_data)
{
	pr_debug("Configure Host Egress TC[%d] Queues\n", params->egress_tc_add.tc_prio);

	/* Update queue topology database */
	nmnicvf->gpio_rem_params.intcs_params[params->egress_tc_add.tc_prio].num_rem_outqs =
		params->egress_tc_add.num_queues_per_tc;
	if (!params->egress_tc_add.num_queues_per_dma)
		params->egress_tc_add.num_queues_per_dma = params->egress_tc_add.num_queues_per_tc;
	nmnicvf->gpio_rem_params.intcs_params[params->egress_tc_add.tc_prio].num_rem_outqs_per_dma =
		params->egress_tc_add.num_queues_per_dma;

	/* TODO: Add support for egress pkt-offset and RSS-type */
	nmnicvf->gpio_params.intcs_params[params->egress_tc_add.tc_prio].pkt_offset = 0;
	nmnicvf->gpio_params.intcs_params[params->egress_tc_add.tc_prio].rss_type = RSS_HASH_NONE;

	return 0;
}

/*
 *	nmnicvf_ingress_tc_add_command
 */
static int nmnicvf_ingress_tc_add_command(struct nmnicvf *nmnicvf,
					 struct mgmt_cmd_params *params,
					 struct mgmt_cmd_resp *resp_data)
{
	pr_debug("Configure Host Ingress TC[%d] Queues\n", params->ingress_tc_add.tc_prio);

	/* Update queue topology database */
	nmnicvf->gpio_rem_params.outtcs_params[params->ingress_tc_add.tc_prio].num_rem_inqs =
		params->ingress_tc_add.num_queues_per_tc;
	nmnicvf->gpio_rem_params.outtcs_params[params->ingress_tc_add.tc_prio].rem_rss_type =
		params->ingress_tc_add.hash_type;
	nmnicvf->gpio_rem_params.outtcs_params[params->ingress_tc_add.tc_prio].rem_pkt_offset =
		params->ingress_tc_add.pkt_offset;

	return 0;
}

/*
 *	tc_q_next_entry_get
 *
 *	This function return next free queue index in TC queue array
 *	in case no available index return -1
 */
static int intc_q_next_entry_get(struct giu_gpio_rem_q_params *q_id_list, u32 q_num)
{
	u32 q_idx;

	for (q_idx = 0; q_idx < q_num; q_idx++)
		if (q_id_list[q_idx].len == 0)
			return q_idx;

	return -1;
}

/*
 *	tc_q_next_entry_get
 *
 *	This function return next free queue index in TC queue array
 *	in case no available index return -1
 */
static int outtc_q_next_entry_get(struct giu_gpio_rem_inq_params *q_id_list, u32 q_num)
{
	u32 q_idx;

	for (q_idx = 0; q_idx < q_num; q_idx++)
		if (q_id_list[q_idx].q_params.len == 0)
			return q_idx;

	return -1;
}


/*
 *	nmnicvf_ingress_queue_add_command
 */
static int nmnicvf_ingress_queue_add_command(struct nmnicvf *nmnicvf,
					    struct mgmt_cmd_params *params,
					    struct mgmt_cmd_resp *resp_data)
{
	struct giu_gpio_rem_q_params giu_gpio_q;
	struct giu_gpio_rem_params *gpio_rem_p = &(nmnicvf->gpio_rem_params);
	struct giu_gpio_outtc_rem_params *outtc;
	struct msix_table_entry *msix_entry;
	struct pcie_config_mem *pcie_cfg;
	s32 active_q_id;
	u32 msg_tc;
	u8 bm_idx;
	int err;

	msg_tc = params->ingress_data_q_add.tc_prio;
	outtc = &(gpio_rem_p->outtcs_params[msg_tc]);
	pcie_cfg = (struct pcie_config_mem *)nmnicvf->map.cfg_map.virt_addr;

	pr_debug("Host Ingress TC[%d], queue Add (num of queues %d)\n", msg_tc, outtc->num_rem_inqs);

	if (nmnicvf->profile_data.sg_en) {
		if (nmnicvf->profile_data.lcl_bp_params[0].lcl_bp_buf_size != params->ingress_data_q_add.q_buf_size) {
			pr_err("For supporting S/G, Host buffer size (%u) MUST equal Local buffer size (%u)\n",
				params->ingress_data_q_add.q_buf_size,
				nmnicvf->profile_data.lcl_bp_params[0].lcl_bp_buf_size);
			return -EFAULT;
		}
	} else {
		for (bm_idx = 0; bm_idx < nmnicvf->profile_data.lcl_bp_num; bm_idx++)
			if (nmnicvf->profile_data.lcl_bp_params[bm_idx].lcl_bp_buf_size >=
				params->ingress_data_q_add.q_buf_size)
				break;
		if (bm_idx == nmnicvf->profile_data.lcl_bp_num) {
			pr_err("Host BM buffer size should be at least %d\n",
				params->ingress_data_q_add.q_buf_size);
			return -EFAULT;
		}
	}
	/* Clear queue structure */
	memset(&giu_gpio_q, 0, sizeof(struct giu_gpio_rem_q_params));

	/* Init queue parameters */
	giu_gpio_q.len          = params->ingress_data_q_add.q_len;
	giu_gpio_q.size         = giu_get_desc_size(nmnicvf->giu, GIU_DESC_OUT);
	giu_gpio_q.q_base_pa    = (phys_addr_t)params->ingress_data_q_add.q_phys_addr;
	giu_gpio_q.prod_base_pa =
		(phys_addr_t)(uintptr_t)(params->ingress_data_q_add.q_prod_offs +
					nmnicvf->map.cfg_map.phys_addr);
	giu_gpio_q.prod_base_va =
		(void *)(params->ingress_data_q_add.q_prod_offs + nmnicvf->map.cfg_map.virt_addr);
	giu_gpio_q.cons_base_pa =
		(phys_addr_t)(uintptr_t)(params->ingress_data_q_add.q_cons_offs +
					nmnicvf->map.cfg_map.phys_addr);
	giu_gpio_q.cons_base_va =
		(void *)(params->ingress_data_q_add.q_cons_offs + nmnicvf->map.cfg_map.virt_addr);
	giu_gpio_q.host_remap   = nmnicvf->map.host_map.phys_addr;

	if (params->ingress_data_q_add.msix_id != AGNIC_MGMT_MSIX_ID_INVALID) {
		msix_entry = &nmnicvf->msix_table_base[params->ingress_data_q_add.msix_id];
		giu_gpio_q.msix_inf.pa = msix_entry->msg_addr;
		giu_gpio_q.msix_inf.data = msix_entry->msg_data;
		/* msix enable hook */
		pcie_cfg->msi_x_mask[PCI_EP_VF_HOST_MSIX_GET_MASK_ARR_INDEX(params->ingress_data_q_add.msix_id)] |=
			PCI_EP_VF_HOST_MSIX_GET_MASK(params->ingress_data_q_add.msix_id);
		/* msix enable hook end */
		giu_gpio_q.msix_inf.mask_address =
		    &pcie_cfg->msi_x_mask[PCI_EP_VF_HOST_MSIX_GET_MASK_ARR_INDEX(params->ingress_data_q_add.msix_id)];
		giu_gpio_q.msix_inf.mask_value = PCI_EP_VF_HOST_MSIX_GET_MASK(params->ingress_data_q_add.msix_id);
		if (nmnicvf->map.type == ft_plat) {
			giu_get_msi_regs(nmnicvf->giu, (u64 *)giu_gpio_q.msix_inf.va, &giu_gpio_q.msix_inf.pa);
		} else {
			/* PCI mode - Need to remap the msix's PA */
			uint64_t phys_addr = (PCI_EP_VF_HOST_MSIX_BASE_ADDR(nmnicvf->vf_id) + giu_gpio_q.msix_inf.pa);

			err = sys_iomem_map(nmnicvf->msix_iomem, NULL, &phys_addr, &giu_gpio_q.msix_inf.va);
			if (err) {
				pr_err("failed to map MSIX%d!\n", params->ingress_data_q_add.msix_id);
				return err;
			}
			pr_info("Host Ingress TC[%d], MSIX%d: host_pa 0x%"PRIx64", host_data 0x%x, host_remap_pa %"PRIx64", host_remap_va %p\n",
				msg_tc, params->ingress_data_q_add.msix_id, msix_entry->msg_addr, msix_entry->msg_data,
				phys_addr, giu_gpio_q.msix_inf.va);
		}
	}

	active_q_id = outtc_q_next_entry_get(outtc->rem_inqs_params, outtc->num_rem_inqs);
	if (active_q_id < 0) {
		pr_err("Failed to configure queue in Host Ingress TC[%d] queue list\n", msg_tc);
		return active_q_id;
	}

	pr_debug("Host Ingress TC[%d], queue added at index %d\n", msg_tc, active_q_id);

	memcpy(&(outtc->rem_inqs_params[active_q_id].q_params),
		&(giu_gpio_q),
		sizeof(struct giu_gpio_rem_q_params));

	/* Set prod/cons address in response.
	 * we use the qid as the prod/cons idx in the notification space
	 * since that is the how CP-125 HW works
	 */
	resp_data->q_add_resp.q_inf = Q_INF_STATUS_OK;

	/* Clear queue structure */
	memset(&giu_gpio_q, 0, sizeof(struct giu_gpio_rem_q_params));

	/* Init queue parameters */
	giu_gpio_q.len	      = params->ingress_data_q_add.q_len;
	giu_gpio_q.size	      = giu_get_desc_size(nmnicvf->giu, GIU_DESC_BUFF);
	giu_gpio_q.q_base_pa    = (phys_addr_t)params->ingress_data_q_add.bpool_q_phys_addr;
	giu_gpio_q.prod_base_pa =
		(phys_addr_t)(uintptr_t)(params->ingress_data_q_add.bpool_q_prod_offs +
					nmnicvf->map.cfg_map.phys_addr);
	giu_gpio_q.prod_base_va =
		(void *)(params->ingress_data_q_add.bpool_q_prod_offs + nmnicvf->map.cfg_map.virt_addr);
	giu_gpio_q.cons_base_pa =
		(phys_addr_t)(uintptr_t)(params->ingress_data_q_add.bpool_q_cons_offs +
					nmnicvf->map.cfg_map.phys_addr);
	giu_gpio_q.cons_base_va =
		(void *)(params->ingress_data_q_add.bpool_q_cons_offs + nmnicvf->map.cfg_map.virt_addr);
	giu_gpio_q.host_remap   = nmnicvf->map.host_map.phys_addr;
	giu_gpio_q.buff_len = params->ingress_data_q_add.q_buf_size;

	memcpy(&(outtc->rem_inqs_params[active_q_id].poolq_params),
		&(giu_gpio_q),
		sizeof(struct giu_gpio_rem_q_params));

	/* Set queue Id in response message in case of success */
	resp_data->q_add_resp.bpool_q_inf = Q_INF_STATUS_OK;

	return 0;
}


/*
 *	nmnicvf_egress_queue_add_command
 */
static int nmnicvf_egress_queue_add_command(struct nmnicvf *nmnicvf,
					   struct mgmt_cmd_params *params,
					   struct mgmt_cmd_resp *resp_data)
{
	struct giu_gpio_rem_q_params giu_gpio_q;
	struct giu_gpio_rem_params *gpio_rem_p = &(nmnicvf->gpio_rem_params);
	struct giu_gpio_intc_rem_params *intc;
	struct msix_table_entry *msix_entry;
	struct pcie_config_mem *pcie_cfg;
	s32 active_q_id;
	u32 msg_tc;
	int err;

	pcie_cfg = (struct pcie_config_mem *)nmnicvf->map.cfg_map.virt_addr;
	msg_tc = params->egress_q_add.tc_prio;
	intc = &(gpio_rem_p->intcs_params[msg_tc]);

	pr_debug("Host Egress TC[%d], queue Add (num of queues %d)\n", msg_tc, intc->num_rem_outqs);

	/* Clear queue structure */
	memset(&giu_gpio_q, 0, sizeof(struct giu_gpio_rem_q_params));

	/* Init queue parameters */
	giu_gpio_q.len          = params->egress_q_add.q_len;
	giu_gpio_q.size         = giu_get_desc_size(nmnicvf->giu, GIU_DESC_IN);
	giu_gpio_q.q_base_pa    = (phys_addr_t)params->egress_q_add.q_phys_addr;
	giu_gpio_q.prod_base_pa =
		(phys_addr_t)(uintptr_t)(params->egress_q_add.q_prod_offs +
					nmnicvf->map.cfg_map.phys_addr);
	giu_gpio_q.prod_base_va =
		(void *)(params->egress_q_add.q_prod_offs + nmnicvf->map.cfg_map.virt_addr);
	giu_gpio_q.cons_base_pa =
		(phys_addr_t)(uintptr_t)(params->egress_q_add.q_cons_offs +
					nmnicvf->map.cfg_map.phys_addr);
	giu_gpio_q.cons_base_va =
		(void *)(params->egress_q_add.q_cons_offs + nmnicvf->map.cfg_map.virt_addr);
	giu_gpio_q.host_remap   = nmnicvf->map.host_map.phys_addr;

	if (params->egress_q_add.msix_id != AGNIC_MGMT_MSIX_ID_INVALID) {
		msix_entry = &nmnicvf->msix_table_base[params->egress_q_add.msix_id];
		giu_gpio_q.msix_inf.pa = msix_entry->msg_addr;
		giu_gpio_q.msix_inf.data = msix_entry->msg_data;
		/* msix enable hook */
		pcie_cfg->msi_x_mask[PCI_EP_VF_HOST_MSIX_GET_MASK_ARR_INDEX(params->egress_q_add.msix_id)] |=
			PCI_EP_VF_HOST_MSIX_GET_MASK(params->egress_q_add.msix_id);
		/* msix enable hook end */
		giu_gpio_q.msix_inf.mask_address =
			&pcie_cfg->msi_x_mask[PCI_EP_VF_HOST_MSIX_GET_MASK_ARR_INDEX(params->egress_q_add.msix_id)];
		giu_gpio_q.msix_inf.mask_value = PCI_EP_VF_HOST_MSIX_GET_MASK(params->egress_q_add.msix_id);
		if (nmnicvf->map.type == ft_plat) {
			giu_get_msi_regs(nmnicvf->giu, (u64 *)giu_gpio_q.msix_inf.va, &giu_gpio_q.msix_inf.pa);
		} else {
			/* PCI mode - Need to remap the msix's PA */
			uint64_t phys_addr = (PCI_EP_VF_HOST_MSIX_BASE_ADDR(nmnicvf->vf_id) + giu_gpio_q.msix_inf.pa);

			err = sys_iomem_map(nmnicvf->msix_iomem, NULL, &phys_addr, &giu_gpio_q.msix_inf.va);
			if (err) {
				pr_err("failed to map MSIX%d!\n", params->egress_q_add.msix_id);
				return err;
			}
			pr_info("Host Egress TC[%d], MSIX%d: host_pa 0x%"PRIx64", host_data 0x%x, host_remap_pa %"PRIx64", host_remap_va %p\n",
				msg_tc, params->egress_q_add.msix_id, msix_entry->msg_addr, msix_entry->msg_data,
				phys_addr, giu_gpio_q.msix_inf.va);
		}
	}

	active_q_id = intc_q_next_entry_get(intc->rem_outqs_params, intc->num_rem_outqs);
	if (active_q_id < 0) {
		pr_err("Failed to configure queue in Host Egress TC[%d] queue list\n", msg_tc);
		return active_q_id;
	}

	pr_debug("Host Egress TC[%d], queue added and index %d\n", msg_tc, active_q_id);

	memcpy(&(intc->rem_outqs_params[active_q_id]),
		&(giu_gpio_q),
		sizeof(struct giu_gpio_rem_q_params));

	/* Set queue Id in and prod/cons address in response.
	 * we use the qid as the prod/cons idx in the notification space
	 * since that is the how CP-125 HW works
	 */
	resp_data->q_add_resp.q_inf = Q_INF_STATUS_OK;

	return 0;
}

/*
 *	nmnicvf_vf_init_done_command
 */
static int nmnicvf_vf_init_done_command(struct nmnicvf *nmnicvf,
					struct mgmt_cmd_params *params,
					struct mgmt_cmd_resp *resp_data)
{
	int	ret = 0;

	ret = giu_gpio_set_remote(nmnicvf->giu_gpio, &(nmnicvf->gpio_rem_params));
	if (ret)
		pr_err("Failed to set giu gpio remote params\n");

	return ret;
}

/*
 *	nmnicvf_mgmt_echo_command
 */
static int nmnicvf_mgmt_echo_command(struct nmnicvf *nmnicvf,
				    struct mgmt_cmd_params *params,
				    struct mgmt_cmd_resp *resp_data)
{
	pr_debug("Management echo message.\n");

	nmnicvf = nmnicvf;

	return 0;
}

/*
 *	nmnicvf_link_status_command
 */
static int nmnicvf_link_status_command(struct nmnicvf *nmnicvf,
				      struct mgmt_cmd_params *params,
				      struct mgmt_cmd_resp *resp_data)
{
	pr_debug("Link status message\n");

	resp_data->link_status = nmnicvf->last_link_state;

	return 0;
}

/*
 *	nmnicvf_close_command
 */
static int nmnicvf_close_command(struct nmnicvf *nmnicvf,
				struct mgmt_cmd_params *params,
				struct mgmt_cmd_resp *resp_data)
{
	pr_debug("Close message.\n");

	if (nmnicvf->giu_gpio)
		giu_gpio_clear_remote(nmnicvf->giu_gpio);

	return 0;
}

/*
 *	nmnicvf_rx_promisc_command
 */
static int nmnicvf_rx_promisc_command(struct nmnicvf *nmnicvf,
				   struct mgmt_cmd_params *params,
				   struct mgmt_cmd_resp *resp_data)
{
	int ret = 0;

	pr_debug("Set promisc message %d\n", params->promisc);

	if (params->promisc != AGNIC_PROMISC_ENABLE)
		return -ENOTSUP;

	return ret;
}

/*
 *	nmnicvf_rx_mc_promisc_command
 */
static int nmnicvf_rx_mc_promisc_command(struct nmnicvf *nmnicvf,
				   struct mgmt_cmd_params *params,
				   struct mgmt_cmd_resp *resp_data)
{
	int ret = 0;

	pr_debug("Set mc promisc message %d\n", params->mc_promisc);

	if (params->promisc != AGNIC_MC_PROMISC_ENABLE)
		return -ENOTSUP;

	return ret;
}

static int nmnicvf_enable_command(struct nmnicvf *nmnicvf)
{
	int ret = 0;

	pr_debug("Set enable message\n");

	nmnicvf->link_up_mask |= LINK_UP_MASK_REMOTE;

	return ret;
}

static int nmnicvf_disable_command(struct nmnicvf *nmnicvf)
{
	int ret = 0;

	pr_debug("Set disable message\n");

	nmnicvf->link_up_mask &= ~LINK_UP_MASK_REMOTE;

	return ret;
}

/*
 *	nmnicvf_loopback_command
 */
static int nmnicvf_loopback_command(struct nmnicvf *nmnicvf,
				struct mgmt_cmd_params *params,
				struct mgmt_cmd_resp *resp_data)
{
	pr_debug("Set loopback message\n");

	return -ENOTSUP;
}


/*
 *	nmnicvf_gp_get_statistics
 */
static int nmnicvf_gp_get_statistics(struct nmnicvf *nmnicvf,
					struct mgmt_cmd_params *params,
					struct mgmt_cmd_resp *resp_data)
{
	struct giu_gpio_statistics stats;
	int ret;

	ret = giu_gpio_get_statistics(nmnicvf->giu_gpio, &stats, 0);
	if (ret)
		return ret;

	resp_data->gp_stats.gp_rx_packets = stats.out_packets;
	resp_data->gp_stats.gp_tx_packets = stats.in_packets;
	resp_data->gp_stats.gp_rx_fullq_dropped = 0;

	return 0;
}

/*
 *	nmnicvf_gp_queue_get_statistics
 */
static int nmnicvf_gp_queue_get_statistics(struct nmnicvf *nmnicvf,
					struct mgmt_cmd_params *params,
					struct mgmt_cmd_resp *resp_data)
{
	int ret;
	struct giu_gpio_q_statistics stats;

	ret = giu_gpio_get_q_statistics(nmnicvf->giu_gpio,
					params->q_get_statistics.out,
					1,
					params->q_get_statistics.tc,
					params->q_get_statistics.qid, &stats,
					params->q_get_statistics.reset);

	if (ret)
		return ret;

	resp_data->gp_queue_stats.packets = stats.packets;

	return 0;
}

static int nmnicvf_get_capabilities(struct nmnicvf *nmnicvf,
				    struct mgmt_cmd_params *params,
				    struct mgmt_cmd_resp *resp_data)
{
	int ret, bm_idx;

	resp_data->capabilities.max_buf_size = 0;
	for (bm_idx = 0; bm_idx < nmnicvf->profile_data.lcl_bp_num; bm_idx++)
		if (nmnicvf->profile_data.lcl_bp_params[bm_idx].lcl_bp_buf_size > resp_data->capabilities.max_buf_size)
			resp_data->capabilities.max_buf_size =
				nmnicvf->profile_data.lcl_bp_params[bm_idx].lcl_bp_buf_size;

	resp_data->capabilities.flags = 0;
	if (nmnicvf->profile_data.sg_en)
		resp_data->capabilities.flags |= CAPABILITIES_SG;

	ret = giu_get_num_dma_engines(nmnicvf->giu, GIU_ENG_IN, &resp_data->capabilities.egress_num_dma_engines);
	if (ret)
		return ret;

	return 0;
}

static void nmnicvf_guest_reset(struct nmnicvf *nmnicvf)
{
	struct giu_gpio_intc_params	*intc;
	u8				 bm_idx;

	giu_gpio_reset(nmnicvf->giu_gpio);

	/* we assume all in-TCs share the same BPools */
	intc = &(nmnicvf->gpio_params.intcs_params[0]);
	for (bm_idx = 0; bm_idx < intc->num_inpools; bm_idx++)
		giu_bpool_reset(nmnicvf->giu_bpools[bm_idx]);
}

/*
 *	nmnicvf_process_vf_command
 *
 *	This function process all VF's commands
 *
 *	@param[in]	nmnicvf - pointer to NIC VF object
 *	@param[in]	msg - pointer to nmdisp_msg object
 *	@param[out]	resp_data - pointer to mgmt_cmd_resp object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */

static int nmnicvf_process_vf_command(struct nmnicvf *nmnicvf,
				      struct nmdisp_msg *msg,
				      struct mgmt_cmd_resp *resp_data)
{
	struct mgmt_cmd_params *cmd_params = msg->msg;
	int ret = 0;

	switch (msg->code) {

	case CC_ENABLE:
		ret = nmnicvf_enable_command(nmnicvf);
		if (ret)
			pr_err("CC_ENABLE message failed\n");
		break;

	case CC_DISABLE:
		ret = nmnicvf_disable_command(nmnicvf);
		if (ret)
			pr_err("CC_DISABLE message failed\n");
		break;

	case CC_INIT:
		ret = nmnicvf_vf_init_command(nmnicvf, cmd_params, resp_data);
		if (ret)
			pr_err("VF_INIT message failed\n");
		break;

	case CC_EGRESS_TC_ADD:
		ret = nmnicvf_egress_tc_add_command(nmnicvf, cmd_params, resp_data);
		if (ret)
			pr_err("VF_EGRESS_TC_ADD message failed\n");
		break;

	case CC_EGRESS_DATA_Q_ADD:
		ret = nmnicvf_egress_queue_add_command(nmnicvf, cmd_params, resp_data);
		if (ret)
			pr_err("VF_EGRESS_DATA_Q_ADD message failed\n");
		break;

	case CC_INGRESS_TC_ADD:
		ret = nmnicvf_ingress_tc_add_command(nmnicvf, cmd_params, resp_data);
		if (ret)
			pr_err("VF_INGRESS_TC_ADD message failed\n");
		break;

	case CC_INGRESS_DATA_Q_ADD:
		ret = nmnicvf_ingress_queue_add_command(nmnicvf, cmd_params, resp_data);
		if (ret)
			pr_err("VF_INGRESS_DATA_Q_ADD message failed\n");
		break;

	case CC_INIT_DONE:
		ret = nmnicvf_vf_init_done_command(nmnicvf, cmd_params, resp_data);
		if (ret)
			pr_err("CC_INIT_DONE message failed\n");
		break;

	case CC_MGMT_ECHO:
		ret = nmnicvf_mgmt_echo_command(nmnicvf, cmd_params, resp_data);
		if (ret)
			pr_err("VF_MGMT_ECHO message failed\n");
		break;

	case CC_LINK_STATUS:
		ret = nmnicvf_link_status_command(nmnicvf, cmd_params, resp_data);
		if (ret)
			pr_err("VF_LINK_STATUS message failed\n");
		break;

	case CC_CLOSE:
		ret = nmnicvf_close_command(nmnicvf, cmd_params, resp_data);
		if (ret)
			pr_err("VF_CLOSE message failed\n");
		break;

	case CC_LOOPBACK:
		ret = nmnicvf_loopback_command(nmnicvf, cmd_params, resp_data);
		if (ret)
			pr_err("VF_LOOPBACK message failed\n");
		break;

	case CC_GET_GP_STATS:
		ret = nmnicvf_gp_get_statistics(nmnicvf, cmd_params, resp_data);
		if (ret)
			pr_err("CC_GET_GP_STATS message failed\n");
		break;

	case CC_GET_GP_QUEUE_STATS:
		ret = nmnicvf_gp_queue_get_statistics(nmnicvf, cmd_params, resp_data);
		if (ret)
			pr_err("CC_GET_GP_QUEUE_STATS message failed\n");
		break;

	case CC_GET_CAPABILITIES:
		ret = nmnicvf_get_capabilities(nmnicvf, cmd_params, resp_data);
		if (ret)
			pr_err("CC_GET_CAPABILITIES message failed\n");
		break;

	default:
		/* Unknown command code */
		pr_err("Unknown command code %d!! Unable to process command.\n", msg->code);
		ret = -1;
		break;
	}

	return ret;
}

/*
 *	nmnicvf_process_guest_command
 *
 *	This function process guest commnads
 *
 *	@param[in]	nmnicvf - pointer to NIC VF object
 *	@param[in]	msg - pointer to nmdisp_msg object
 *
 */
static void nmnicvf_process_guest_command(struct nmnicvf *nmnicvf,
					  struct nmdisp_msg *msg)
{
	struct guest_cmd_resp resp;
	int ret = 0;

	switch (msg->code) {

	case MSG_F_GUEST_GPIO_ENABLE:
		nmnicvf->link_up_mask |= LINK_UP_MASK_LOCAL;
		break;

	case MSG_F_GUEST_GPIO_DISABLE:
		nmnicvf->link_up_mask &= ~LINK_UP_MASK_LOCAL;
		break;

	case MSG_F_GUEST_GPIO_GET_LINK_STATE:
		resp.giu_resp.link_state = ((nmnicvf->link_up_mask & LINK_UP_MASK) == LINK_UP_MASK);
		break;

	case MSG_F_GUEST_GPIO_RESET:
		nmnicvf_guest_reset(nmnicvf);
		break;

	default:
		/* Unknown command code */
		pr_err("Unknown command code %d!! Unable to process command.\n", msg->code);
		ret = -1;
		break;
	}

	if (msg->resp_required) {
		resp.status = (ret) ? RESP_STATUS_FAIL : RESP_STATUS_OK;
		msg->ext = 0;
		msg->dst_client = CDT_CUSTOM;
		msg->dst_id = nmnicvf->guest_id;
		msg->src_client = CDT_VF;
		msg->src_id = nmnicvf->vf_id;
		msg->msg = &resp;
		msg->msg_len = sizeof(resp);
		ret = nmdisp_send_msg(nmnicvf->nmdisp, 0, msg);
		if (ret)
			pr_err("failed to send response message\n");
	}
}

/*
 *	nmnicvf_process_command
 *
 *	This function process all VF execution flows
 *
 *	@param[in]	nmnicvf - pointer to NIC VF object
 *	@param[in]	msg - pointer to nmdisp_msg object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
static int nmnicvf_process_command(void *arg, struct nmdisp_msg *msg)
{
	struct mgmt_cmd_resp resp_data;
	struct nmnicvf *nmnicvf = (struct nmnicvf *)arg;
	int ret, send_msg = 1;

/*
 *	Once NIC-VF get a external command from dispatcher, it shall first check the 'src-client/ID' (should be its own)
 *	and then it will authenticate the message by validating the 'code' (against the available codes). In that stage,
 *	the NIC-VF shall execute the command.
 *	Optionally, the NIC-VF will return a response to the caller by sending a response message by calling the
 *	'nmdisp_send_msg' API that is implemented as part of the dispatcher. It shall use its own 'src-client/ID' and
 *	its own 'dst-client/ID' and 'ext' set.
 *
 *	For some commands, a guest may be registered as a 'listener' (e.g. MTU change). Therefore, after executing the
 *	command, the NIC-VF shall iterate all registered 'listeners' for this specific command and initiate a call with
 *	the command message by calling the 'nmdisp_send_msg' API with 'src-client/id' of the NIC-VF and 'dst-client/id'
 *	of the Custom ('ext' should not be set).
 *
 *	Once NIC-VF get a internal command and the 'src-client' is of type Custom, it should initiate a call with
 *	the command message by calling the 'nmdisp_send_msg' API and 'ext' set.
 */

	pr_debug("NICVF got %s command code %d from client-type %d client-id %d which %s response\n",
		 (msg->ext) ? "external":"internal", msg->code, msg->src_client, msg->src_id,
		 (msg->resp_required) ? "requires" : "doesn't requires");

	if (msg->ext) {
		if ((msg->src_client == CDT_VF) && (msg->src_id == nmnicvf->vf_id)) {
			ret = nmnicvf_process_vf_command(nmnicvf, msg, &resp_data);
			send_msg = msg->resp_required;
			if (ret)
				resp_data.status = NOTIF_STATUS_FAIL;
			else
				resp_data.status = NOTIF_STATUS_OK;

			msg->msg = (void *)&resp_data;
			msg->msg_len = sizeof(resp_data);
		} else if ((msg->src_client == CDT_CUSTOM) && (msg->src_id == nmnicvf->guest_id)) {
			nmnicvf_process_guest_command(nmnicvf, msg);
			return 0;
		} else {
			pr_err("Src client %d not supported for external command\n", msg->src_client);
			return -1;
		}
	} else {
		if (msg->src_client == CDT_CUSTOM) {
			msg->ext = 1;
			pr_debug("VF-Lf got %s command code %d from client-type %d client-id %d msg: 0x%x\n",
				(msg->ext) ? "external":"internal", msg->code, msg->src_client, msg->src_id,
				*(u32 *)msg->msg);
		} else {
			pr_err("Src client %d not supported for internal command\n", msg->src_client);
			return -1;
		}
	}

	if (send_msg) {
		ret = nmdisp_send_msg(nmnicvf->nmdisp, 0, msg);
		if (ret) {
			pr_err("failed to send response message\n");
			return ret;
		}
	}

	return 0;
}

static int nmnicvf_serialize_giu(struct nmnicvf *nmnicvf, char *buff, u32 size, u8 depth)
{
	size_t	 pos = 0;
	u8	 bm_idx, num_pools;
	int	 ret;

	/* assuming all TCs share the same BPools */
	num_pools = nmnicvf->gpio_params.intcs_params[0].num_inpools;
	json_print_to_buffer(buff, size, depth, "\"giu-bpools\": {\n");
	for (bm_idx = 0; bm_idx < num_pools; bm_idx++) {
		ret = giu_bpool_serialize(nmnicvf->giu_bpools[bm_idx], &buff[pos], size - pos, depth + 1);
		if (ret < 0)
			return ret;
		pos += ret;
		if (pos != strlen(buff)) {
			pr_err("found mismatch between pos (%zu) and buff len (%zu)\n", pos, strlen(buff));
			return -EINVAL;
		}
	}
	json_print_to_buffer(buff, size, depth, "},\n");
	json_print_to_buffer(buff, size, depth, "\"giu-gpio\": {\n");
	ret = giu_gpio_serialize(nmnicvf->giu_gpio, &buff[pos], size - pos, depth + 1);
	if (ret < 0)
		return ret;
	pos += ret;
	if (pos != strlen(buff)) {
		pr_err("found mismatch between pos (%zu) and buff len (%zu)\n", pos, strlen(buff));
		return -EINVAL;
	}
	json_print_to_buffer(buff, size, depth, "},\n");

	return pos;
}

static int nmnicvf_serialize_relations_info(struct nmnicvf *nmnicvf, char *buff, u32 size, u8 depth)
{
	size_t	 pos = 0;
	u8	 bm_idx, num_pools;

	/* build relation-information first */
	json_print_to_buffer(buff, size, depth, "\"relations-info-%d\": {\n", nmnicvf->nmlf.id);
	json_print_to_buffer(buff, size, depth + 1, "\"lf_type\": %d,\n", CDT_VF);
	json_print_to_buffer(buff, size, depth + 1, "\"lf_id\": %d,\n", nmnicvf->vf_id);
	/* serialize the relations of the GIU objects */
	json_print_to_buffer(buff, size, depth + 1, "\"giu-gpio\": \"%s\",\n", nmnicvf->profile_data.match);
	/* assuming all TCs share the same BPools */
	num_pools = nmnicvf->gpio_params.intcs_params[0].num_inpools;
	json_print_to_buffer(buff, size, depth + 1, "\"num_bpools\": %d,\n", num_pools);
	for (bm_idx = 0; bm_idx < num_pools; bm_idx++) {
		if (bm_idx == num_pools - 1)
			json_print_to_buffer(buff, size, depth + 1, "\"giu-bpool-%d\": \"giu_pool-%d:%d\"\n", bm_idx,
					nmnicvf->giu_bpools[bm_idx]->giu_id, nmnicvf->giu_bpools[bm_idx]->id);
		else
			json_print_to_buffer(buff, size, depth + 1, "\"giu-bpool-%d\": \"giu_pool-%d:%d\",\n", bm_idx,
					nmnicvf->giu_bpools[bm_idx]->giu_id, nmnicvf->giu_bpools[bm_idx]->id);
	}
	json_print_to_buffer(buff, size, depth, "},\n");

	return pos;
}

/*
 *	nmnicpf_link_state_check_n_notif
 *
 *	This function checks the link state and if it was changed
 *	since the last time it was checked it notifies the host
 *
 *	@param[in]	nmnicvf - pointer to NIC VF object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
static int nmnicvf_link_state_check_n_notif(struct nmnicvf *nmnicvf)
{
	int link_state;

	link_state = ((nmnicvf->link_up_mask & LINK_UP_MASK) == LINK_UP_MASK);
	if (nmnicvf->last_link_state != link_state) {
		/* If link state was changed since last check, notify the host */
		pr_info("Link state was change to %d\n", link_state);

		if (link_state)
			giu_gpio_enable(nmnicvf->giu_gpio);
		else
			giu_gpio_disable(nmnicvf->giu_gpio);
		nmnicvf->last_link_state = link_state;
	}

	return 0;
}

static int nmnicvf_mgmt_close_check(struct nmnicvf *nmnicvf)
{
	volatile struct pcie_config_mem *pcie_cfg;
	struct giu_mng_ch_qs		mng_ch_qs;

	pcie_cfg = (struct pcie_config_mem *)nmnicvf->map.cfg_map.virt_addr;

	if (!(readl(&pcie_cfg->status) & PCIE_CFG_STATUS_HOST_MGMT_CLOSE_REQ))
		/* reset bit wan't not set */
		return 0;
	pr_debug("got 'mgmt-close' request\n");

	/* unregister mng queues from dispatcher */
	if (nmnicvf->giu_mng_ch) {
		giu_mng_ch_get_qs(nmnicvf->giu_mng_ch, &mng_ch_qs);
		nmdisp_remove_queue(nmnicvf->nmdisp, CDT_VF, nmnicvf->vf_id, mng_ch_qs.lcl_cmd_q);
		/* destroy mng queues */
		giu_mng_ch_deinit(nmnicvf->giu_mng_ch);
		nmnicvf->giu_mng_ch = NULL;
	}

	/* Request Complete */
	writel(PCIE_CFG_STATUS_HOST_MGMT_CLOSE_DONE, &pcie_cfg->status);

	return 0;
}

static int nmnicvf_reset_check(struct nmlf *nmlf)
{
	struct nmnicvf *nmnicvf = (struct nmnicvf *)nmlf;
	struct giu_mng_ch_qs		mng_ch_qs;
	volatile struct pcie_config_mem *pcie_cfg;

	pcie_cfg = (struct pcie_config_mem *)nmnicvf->map.cfg_map.virt_addr;

	if (!(readl(&pcie_cfg->status) & PCIE_CFG_STATUS_HOST_RESET))
		/* reset bit wan't not set */
		return 0;
	pr_info("got 'reset' request\n");

	/* reset bit was set, need to reconfigure VF flow */
	/* 1. unregister mng queues from dispatcher */
	if (nmnicvf->giu_mng_ch) {
		giu_mng_ch_get_qs(nmnicvf->giu_mng_ch, &mng_ch_qs);
		nmdisp_remove_queue(nmnicvf->nmdisp, CDT_VF, nmnicvf->vf_id, mng_ch_qs.lcl_cmd_q);
		/* 2. destroy mng queues */
		nmnicvf_mng_chn_terminate(nmnicvf);
	}
	/* 3. destroy 'remote' queues */
	if (nmnicvf->giu_gpio) {
		/* Emulate link down */
		nmnicvf->link_up_mask &= ~LINK_UP_MASK_REMOTE;
		nmnicvf_link_state_check_n_notif(nmnicvf);
		giu_gpio_clear_remote(nmnicvf->giu_gpio);
	}

	/* 4.  clear 'gpio_rem_params' */
	memset(&nmnicvf->gpio_rem_params, 0, sizeof(struct giu_gpio_rem_params));

	/* 5. start 'init' flow immediately */
	writel(PCIE_CFG_STATUS_DEV_READY, &pcie_cfg->status);
	nmnicvf->nmlf.f_maintenance_cb = nmnicvf_init_host_ready;

	/* return "error" to exit stop other maintenance opertions */
	return -EAGAIN;
}

static int nmnicvf_maintenance(struct nmlf *nmlf)
{
	struct nmnicvf *nmnicvf = (struct nmnicvf *)nmlf;
	int err;

	/* Check for reset signal */
	err = nmnicvf_reset_check(nmlf);
	if (err)
		return err;

	/* Check for mgmt-close signal */
	err = nmnicvf_mgmt_close_check(nmnicvf);
	if (err)
		return err;

	/* Check link state (and notify in case of a change) */
	err = nmnicvf_link_state_check_n_notif(nmnicvf);
	if (err)
		return err;

	return 0;
}

static int nmnicvf_init_gpio_local(struct nmnicvf *nmnicvf)
{
	struct giu_gpio_intc_params	*intc;
	struct giu_bpool_params		 bp_params;
	char				 name[35];
	u8				 tc_id, bm_idx;
	int				 giu_id = 0; /* Support only a single GIU */
	int				 ret;

	nmnicvf->gpio_rem_params.num_outtcs =
		nmnicvf->gpio_params.num_outtcs =
			nmnicvf->profile_data.max_num_tcs;
	nmnicvf->gpio_rem_params.num_intcs =
		nmnicvf->gpio_params.num_intcs =
			nmnicvf->profile_data.max_num_tcs;

	/* Initialize remote queues database */
	ret = nmnicvf_topology_remote_queue_init(nmnicvf);
	if (ret) {
		pr_err("Failed to update remote DB queue info\n");
		return ret;
	}

	/* Initialize local queues database */
	ret = nmnicvf_topology_local_queue_init(nmnicvf);
	if (ret) {
		pr_err("Failed to update local DB queue info\n");
		return ret;
	}
	/* Allocate and configure local queues in the database */
	ret = nmnicvf_topology_local_queue_cfg(nmnicvf);
	if (ret) {
		pr_err("Failed to configure VF topology\n");
		return ret;
	}

	bp_params.giu = nmnicvf->giu;
	bp_params.mqa = nmnicvf->mqa;

	/* we assume all in-TCs share the same BPools */
	intc = &(nmnicvf->gpio_params.intcs_params[0]);

	for (bm_idx = 0; bm_idx < intc->num_inpools; bm_idx++) {
		int bpool_id = giu_bpool_alloc();

		if (bpool_id < 0) {
			pr_err("Failed to alloc giu bpool\n");
			return ret;
		}
		/* Create bpool match string */
		memset(name, 0, sizeof(name));
		snprintf(name, sizeof(name), "giu_pool-%d:%d", giu_id, bpool_id);
		bp_params.match = name;
		bp_params.num_buffs = nmnicvf->profile_data.lcl_bp_params[bm_idx].lcl_bp_size;
		bp_params.buff_len = nmnicvf->profile_data.lcl_bp_params[bm_idx].lcl_bp_buf_size;
		ret = giu_bpool_init(&bp_params, &(nmnicvf->giu_bpools[bm_idx]));
		if (ret) {
			pr_err("Failed to init giu bpool\n");
			return ret;
		}
	}

	/* update the information needed for gpio giu-qs */
	for (tc_id = 0; tc_id < (nmnicvf->gpio_params.num_intcs); tc_id++) {
		intc = &(nmnicvf->gpio_params.intcs_params[tc_id]);

		for (bm_idx = 0; bm_idx < intc->num_inpools; bm_idx++)
			intc->pools[bm_idx] = nmnicvf->giu_bpools[bm_idx];
	}

	/* Create GPIO match string */
	nmnicvf->gpio_params.match = nmnicvf->profile_data.match;
	nmnicvf->gpio_params.sg_en = nmnicvf->profile_data.sg_en;
	ret = giu_gpio_init(&(nmnicvf->gpio_params), &(nmnicvf->giu_gpio));
	if (ret) {
		pr_err("Failed to init giu gpio\n");
		return ret;
	}

	/* Indicate nmp init_done ready */
	nmnicvf->f_ready_cb(nmnicvf->arg, nmnicvf->nmlf.id);

	nmnicvf->initialized = 1;

	return ret;
}

static int nmnicvf_init_host_ready(struct nmlf *nmlf)
{
	struct nmnicvf			*nmnicvf = (struct nmnicvf *)nmlf;
	struct nmdisp_q_pair_params	 q_params;
	struct giu_mng_ch_qs		 mng_ch_qs;
	int				 err;

	pr_debug("check if host is ready\n");

	/* Initialize management queues */
	err = nmnicvf_mng_chn_host_ready(nmnicvf);
	if (err) {
		nmnicvf->nmlf.f_maintenance_cb = nmnicvf_reset_check;
		return 0;
	}

	nmnicvf->nmlf.f_maintenance_cb = nmnicvf_maintenance;

	giu_mng_ch_get_qs(nmnicvf->giu_mng_ch, &mng_ch_qs);

	memset(&q_params, 0, sizeof(q_params));
	q_params.cmd_q	  = mng_ch_qs.lcl_cmd_q;
	q_params.notify_q = mng_ch_qs.lcl_resp_q;
	q_params.ext_desc_support = 0;
	q_params.max_msg_size = sizeof(struct mgmt_cmd_params);
	err = nmdisp_add_queue(nmnicvf->nmdisp, CDT_VF, nmnicvf->vf_id, &q_params);
	if (err)
		return err;

	return err;
}

/*
 *	nmnicvf_init
 *
 *	@param[in]	params - pointer to NIC VF parameters
 *	@param[out]	nmnicvf - pointer to the created NIC VF object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int nmnicvf_init(struct nmnicvf_params *params, struct nmnicvf **nmnicvf)
{
	struct nmnicvf			*_nmnicvf;
	struct nmdisp_client_params	 client_params;
	int				 err;

	_nmnicvf = kmalloc(sizeof(*_nmnicvf), GFP_KERNEL);
	if (!_nmnicvf)
		return -ENOMEM;
	memset(_nmnicvf, 0, sizeof(struct nmnicvf));

	_nmnicvf->nmlf.id = params->lf_id;
	_nmnicvf->vf_id = params->id;
	_nmnicvf->guest_id = params->guest_id;

	_nmnicvf->nmdisp = params->nmdisp;
	_nmnicvf->mqa = params->mqa;
	_nmnicvf->giu = params->giu;

	/* Assign the vf_init_done callback */
	_nmnicvf->f_ready_cb = params->f_ready_cb;
	_nmnicvf->f_get_free_bar_cb = params->f_get_free_bar_cb;
	_nmnicvf->f_put_bar_cb = params->f_put_bar_cb;
	_nmnicvf->f_get_vf_bar_cb = params->f_get_vf_bar_cb;
	_nmnicvf->arg = params->arg;

	*nmnicvf = _nmnicvf;

	err = init_nicvf_params(_nmnicvf, params->nmp_nicvf_params);
	if (err)
		return err;

	/* Clear queue topology batabase */
	memset(&(_nmnicvf->gpio_params), 0, sizeof(struct giu_gpio_params));

	_nmnicvf->gpio_params.mqa = _nmnicvf->mqa;
	_nmnicvf->gpio_params.giu = _nmnicvf->giu;

	/* Initialize NIC-VF map */
	err = nmnicvf_map_init(_nmnicvf);
	if (err)
		return err;

	/* Initialize management queues */
	err = nmnicvf_mng_chn_init(_nmnicvf);
	if (err)
		return err;

	_nmnicvf->nmlf.f_maintenance_cb = nmnicvf_maintenance;

	/* Register NIC VF to dispatcher */
	memset(&client_params, 0, sizeof(client_params));
	client_params.client_type	= CDT_VF;
	client_params.client_id		= _nmnicvf->vf_id;
	client_params.f_client_ctrl_cb	= nmnicvf_process_command;
	client_params.client		= _nmnicvf;
	err = nmdisp_register_client(_nmnicvf->nmdisp, &client_params);
	if (err)
		return err;

	err = nmnicvf_init_gpio_local(_nmnicvf);
	if (err)
		return err;

	return err;
}

/*
 *	nmnicvf_deinit
 *
 *	@param[in]	nmnicvf - pointer to NIC VF object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int nmnicvf_deinit(struct nmnicvf *nmnicvf)
{
	struct giu_mng_ch_qs	mng_ch_qs;
	int ret;

	ret = nmnicvf_local_queue_terminate(nmnicvf);
	if (ret)
		return ret;

	giu_mng_ch_get_qs(nmnicvf->giu_mng_ch, &mng_ch_qs);

	ret = nmdisp_remove_queue(nmnicvf->nmdisp, CDT_VF, nmnicvf->vf_id, mng_ch_qs.lcl_cmd_q);
	if (ret)
		return ret;

	ret = nmdisp_deregister_client(nmnicvf->nmdisp, CDT_VF, nmnicvf->vf_id);
	if (ret)
		return ret;

	ret = nmnicvf_mng_chn_terminate(nmnicvf);
	if (ret)
		return ret;

	/* Un-Map the NIC-VF */
	ret = nmnicvf_map_terminate(nmnicvf);
	if (ret)
		return ret;

	kfree(nmnicvf);

	pr_debug("Terminating NIC VF\n");
	return 0;
}

int nmnicvf_serialize(struct nmnicvf *nmnicvf, char *buff, u32 size, int is_relations_info)
{
	size_t	 pos = 0;
	int	 ret;

	if (is_relations_info)
		return nmnicvf_serialize_relations_info(nmnicvf, buff, size, 2);


	/* Serialize the GIU */
	ret = nmnicvf_serialize_giu(nmnicvf, &buff[pos], size - pos, 1);
	if (ret < 0)
		return ret;
	pos += ret;
	if (pos != strlen(buff)) {
		pr_err("found mismatch between pos (%zu) and buff len (%zu)\n", pos, strlen(buff));
		return -EFAULT;
	}

	return pos;
}

