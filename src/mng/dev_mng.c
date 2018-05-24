/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#define log_fmt(fmt) "dev_mng: " fmt

#include "std_internal.h"
#include "env/trace/trc_pf.h"
#include "hw_emul/gie.h"
#include "drivers/mv_mqa.h"
#include "db.h"
#include "lf/lf_mng.h"
#include "lf/pf/pf.h"
#include "lf/pf/pf_regfile.h"
#include "lf/pf/pf_topology.h"
#include "lf/custom/custom.h"
#include "lf/mng_cmd_desc.h"
#include "mng/mv_nmp_dispatch.h"
#include "lib/lib_misc.h"

#include "dev_mng.h"
#include "dev_mng_pp2.h"
#include "pci_ep_def.h"
#include "dispatch.h"

#define PLAT_AGNIC_UIO_NAME "agnic"
#define PLAT_AGNIC_CFG_SPACE_SIZE	(1 << 20)

/** =========================== **/
/** == Device Initialization == **/
/** =========================== **/


/** Hardware Functionality **/
/** ====================== **/
static int dev_mng_config_plat_func(struct pci_plat_func_map *map);

static int dev_mng_map_plat_func(struct pci_plat_func_map *map)
{
	struct sys_iomem_params iomem_params;
	int ret;

	pr_info("Mapping function %s\n", PLAT_AGNIC_UIO_NAME);

	iomem_params.type = SYS_IOMEM_T_UIO;
	iomem_params.devname = PLAT_AGNIC_UIO_NAME;
	iomem_params.index = 0;

	ret = sys_iomem_init(&iomem_params, &map->sys_iomem);
	if (ret)
		return ret;

	/* Map the agnic configuration registers */
	ret = sys_iomem_map(map->sys_iomem, "agnic_regs", (phys_addr_t *)&map->plat_regs.phys_addr,
			&map->plat_regs.virt_addr);
	if (ret)
		goto err_regs_map;

	pr_info("agnic regs mapped at virt:%p phys:%p\n", map->plat_regs.virt_addr,
		   map->plat_regs.phys_addr);

	/* Map the MSI-X registers */
	ret = sys_iomem_map(map->sys_iomem, "msi_regs", (phys_addr_t *)&map->msi_regs.phys_addr,
			&map->msi_regs.virt_addr);
	if (ret)
		goto err_msi_regs_map;

	pr_info("msi-x regs mapped at virt:%p phys:%p\n", map->msi_regs.virt_addr,
		   map->msi_regs.phys_addr);

	/* Allocate configuration space memory */
	map->cfg_map.virt_addr = mv_sys_dma_mem_alloc(PLAT_AGNIC_CFG_SPACE_SIZE, 4096);
	if (map->cfg_map.virt_addr == NULL) {
		pr_err("Failed to allocate platform configuration space.\n");
		ret = -ENOMEM;
		goto err_cfg_alloc;
	}

	/* Get the relevant physical address. */
	map->cfg_map.phys_addr = (void *)mv_sys_dma_mem_virt2phys(map->cfg_map.virt_addr);

	/* Clear the config space, to prevent false device indications. */
	memset(map->cfg_map.virt_addr, 0x0, PLAT_AGNIC_CFG_SPACE_SIZE);

	pr_info("Platform config space @ %p.\n", map->cfg_map.phys_addr);
	/* Setup the "host_map", which is actually an identity mapping for the
	 * physical address.
	 * The virtual map, is not needed, as not entity in the mgmt side is
	 * accessing the virtual addresses in the "host" side, all accesses are
	 * done using a DMA.
	 * Thus, we set the virtual address to some "bad address" so that we
	 * can identify faulty accesses to host's virtual space.
	 */
	map->host_map.phys_addr = 0x0;
	map->host_map.virt_addr = (void *)0xBAD00ADD0BAD0ADDll;

	/* Configure device registers. */
	dev_mng_config_plat_func(map);

	/* Unmap the device registers, they are no longer needed. */
	sys_iomem_unmap(map->sys_iomem, "agnic_regs");

	return 0;

err_cfg_alloc:
	sys_iomem_unmap(map->sys_iomem, "agnic_regs");
err_msi_regs_map:
err_regs_map:
	sys_iomem_deinit(map->sys_iomem);

	return ret;
}

static int dev_mng_unmap_plat_func(struct pci_plat_func_map *map)
{
	mv_sys_dma_mem_free(map->cfg_map.virt_addr);
	sys_iomem_deinit(map->sys_iomem);

	return 0;
}

/*
 * dev_mng_config_plat_func - Setup the platofm device registers to trigger
 * operation of the kernel based driver.
 */
static int dev_mng_config_plat_func(struct pci_plat_func_map *map)
{
	u32 cfg_mem_addr[2]; /* High and low */
	dma_addr_t phys_addr = (dma_addr_t)map->cfg_map.phys_addr;

	pr_info("Setting platform device registers.\n");
	cfg_mem_addr[0] = lower_32_bits(phys_addr) | 0x1;
	cfg_mem_addr[1] = upper_32_bits(phys_addr) | (0xCAFE << 16);

	writel(cfg_mem_addr[0], map->plat_regs.virt_addr + 0xA0);
	writel(cfg_mem_addr[1], map->plat_regs.virt_addr + 0xA4);

	return 0;
}


static int dev_mng_map_pci_func(struct pci_plat_func_map *map)
{
	struct sys_iomem_params iomem_params;
	int ret;

	pr_info("Mapping function %s\n", PCI_EP_UIO_MEM_NAME);

	iomem_params.type = SYS_IOMEM_T_UIO;
	iomem_params.devname = PCI_EP_UIO_MEM_NAME;
	iomem_params.index = 0;

	ret = sys_iomem_init(&iomem_params, &map->sys_iomem);
	if (ret)
		return ret;

	/* Map the whole physical Packet Processor physical address */
	ret = sys_iomem_map(map->sys_iomem,
			    PCI_EP_UIO_REGION_NAME,
			    (phys_addr_t *)&map->cfg_map.phys_addr,
			    &map->cfg_map.virt_addr);
	if (ret) {
		sys_iomem_deinit(map->sys_iomem);
		return ret;
	}

	pr_debug("BAR-0 of %s mapped at virt:%p phys:%p\n", func_name, map->cfg_map.virt_addr,
		   map->cfg_map.phys_addr);

	/* Map the whole physical Packet Processor physical address */
	ret = sys_iomem_map(map->sys_iomem, "host-map", (phys_addr_t *)&map->host_map.phys_addr,
			&map->host_map.virt_addr);
	if (ret) {
		sys_iomem_deinit(map->sys_iomem);
		return ret;
	}

	pr_debug("host RAM of %s remapped to phys %p virt %p with size 0x%lx\n", func_name,
		   map->host_map.phys_addr, map->host_map.virt_addr, map->host_map.size);

	return 0;
}

static int dev_mng_mqa_init(struct nmp *nmp)
{
	int ret;
	u64 pf_cfg_phys, pf_cfg_virt;
	struct mqa_params params;

	pr_info("Initializing MQA\n");

	/* Initializing MQA botification tables base address */
	pf_cfg_phys = (u64)nmp->nmnicpf.map.cfg_map.phys_addr;
	pf_cfg_virt = (u64)nmp->nmnicpf.map.cfg_map.virt_addr;

	params.num_qs = MQA_QUEUE_MAX;
	params.notif_tbl.qnpt_pa = (phys_addr_t)(pf_cfg_phys + PCI_BAR0_MQA_QNPT_BASE);
	params.notif_tbl.qnct_pa = (phys_addr_t)(pf_cfg_phys + PCI_BAR0_MQA_QNCT_BASE);
	params.notif_tbl.qnpt_va = (void *)(pf_cfg_virt + PCI_BAR0_MQA_QNPT_BASE);
	params.notif_tbl.qnct_va = (void *)(pf_cfg_virt + PCI_BAR0_MQA_QNCT_BASE);

	/* Initializing MQA tables */
	ret = mqa_init(&params, &(nmp->mqa));
	if (ret) {
		pr_err("Failed to initialize MQA tables\n");
		return ret;
	}

	nmp->nmnicpf.mqa = nmp->mqa;

	pr_info("Initializing MQA %p %p\n", nmp->mqa, nmp->nmnicpf.mqa);

	return 0;
}


/* Initialize the management, TX, and RX gie instances */
static int dev_mng_init_gie(struct nmp *nmp)
{
	struct gie_params gie_pars;
	char dma_name[16];
	int ret;

	/* Initialize the management GIU instance */
	pr_info("Initializing GIU devices\n");

	/* Mgmt GIE */
	gie_pars.gct_base = (u64)nmp->nmnicpf.mqa->qct_base;
	gie_pars.gpt_base = (u64)nmp->nmnicpf.mqa->qpt_base;
	gie_pars.gncs_base = (u64)nmp->nmnicpf.mqa->qnct_base;
	gie_pars.gnps_base = (u64)nmp->nmnicpf.mqa->qnpt_base;
	gie_pars.msi_regs_phys = 0; /* TODO: Add map once MSI-X is supported */
	gie_pars.msi_regs_virt = 0; /* TODO: Add map once MSI-X is supported */

	sprintf(dma_name, "dmax2-%d", 0);
	gie_pars.dmax_match = dma_name;
	gie_pars.name_match = (char *)"mng";

	ret = gie_init(&gie_pars, &(nmp->nmnicpf.gie.mng_gie));
	if (ret) {
		pr_err("Failed to initialize management GIU\n");
		return -ENODEV;
	}

	/* RX GIE */
	gie_pars.msi_regs_phys = (u64)nmp->nmnicpf.map.msi_regs.phys_addr;
	gie_pars.msi_regs_virt = (u64)nmp->nmnicpf.map.msi_regs.virt_addr;
	sprintf(dma_name, "dmax2-%d", 1);
	gie_pars.dmax_match = dma_name;
	gie_pars.name_match = (char *)"rx";

	ret = gie_init(&gie_pars, &(nmp->nmnicpf.gie.rx_gie));
	if (ret) {
		pr_err("Failed to initialize RX GIU\n");
		goto error;
	}

	/* TX GIE */
	gie_pars.msi_regs_phys = 0; /* N/A */
	gie_pars.msi_regs_virt = 0; /* N/A */
	sprintf(dma_name, "dmax2-%d", 2);
	gie_pars.dmax_match = dma_name;
	gie_pars.name_match = (char *)"tx";

	ret = gie_init(&gie_pars, &(nmp->nmnicpf.gie.tx_gie));
	if (ret) {
		pr_err("Failed to initialize TX GIU\n");
		goto error;
	}

	return 0;

error:
	gie_terminate(nmp->nmnicpf.gie.mng_gie);
	if (nmp->nmnicpf.gie.rx_gie)
		gie_terminate(nmp->nmnicpf.gie.rx_gie);

	return -ENODEV;
}


static int dev_mng_hw_init(struct nmp *nmp)
{
	int ret;

	pr_info("Initializing Device hardware\n");

	/* Map the NIC-PF */
	/* First, try to map the platform device, if failed, try the pci
	 * device.
	 */
	nmp->nmnicpf.map.type = ft_plat;
	ret = dev_mng_map_plat_func(&nmp->nmnicpf.map);
	if (ret) {
		pr_info("Platform device not found, trying the pci device.\n");
		nmp->nmnicpf.map.type = ft_pcie_ep;
		ret = dev_mng_map_pci_func(&nmp->nmnicpf.map);
	}
	if (ret)
		return ret;

	/* Initialize Register File utility */
	ret = regfile_init();
	if (ret)
		return ret;

	/* Initialize MQA - device queue management */
	ret = dev_mng_mqa_init(nmp);
	if (ret)
		return ret;

	if (nmp->nmpp2.pp2_en) {
		ret = dev_mng_pp2_init(nmp);
		if (ret)
			return ret;
	}

	ret = dev_mng_init_gie(nmp);
	if (ret)
		return ret;

	return 0;
}


/** Software Functionality **/
/** ====================== **/

/* Initialize the PP2 interface */
static void dev_mng_pf_init_done(void *arg)
{
	struct nmp *nmp = (struct nmp *)arg;
	struct nmnicpf *nmnicpf = &nmp->nmnicpf;
	struct mv_sys_dma_mem_info mem_info;
	char	 file_name[SER_MAX_FILE_NAME];
	char	 buff[SER_MAX_FILE_SIZE];
	u32	 size = SER_MAX_FILE_SIZE;
	char	 dev_name[100];
	int	 ret;
	size_t	 pos = 0;

	pr_info("nmp_pf_init_done reached\n");

	/* TODO: go over all guests */
	snprintf(file_name, sizeof(file_name), "%s%s%d", SER_FILE_VAR_DIR, SER_FILE_NAME_PREFIX, nmnicpf->guest_id);
	/* Remove the serialize files */
	remove(file_name);

	memset(buff, 0, size);

	json_print_to_buffer(buff, size, 0, "{\n");

	/* Serialize the DMA info */
	mem_info.name = dev_name;
	mv_sys_dma_mem_get_info(&mem_info);
	json_print_to_buffer(buff, size, 1, "\"dma-info\": {\n");
	json_print_to_buffer(buff, size, 2, "\"file_name\": \"%s\",\n", mem_info.name);
	json_print_to_buffer(buff, size, 2, "\"region_size\": %zu,\n", mem_info.size);
	json_print_to_buffer(buff, size, 2, "\"phys_addr\": %#zx\n", mem_info.paddr);
	json_print_to_buffer(buff, size, 1, "},\n");

	pr_info("starting serialization of guest %d\n", nmnicpf->guest_id);

	if (nmnicpf->profile_data.port_type == NMP_LF_NICPF_T_PP2_PORT) {
		ret = dev_mng_pp2_serialize(nmnicpf, &buff[pos], size - pos);
		if (ret >= 0)
			pos += ret;
		if (pos != strlen(buff))
			pr_err("found mismatch between pos (%zu) and buff len (%zu)\n", pos, strlen(buff));
	}

	if (nmp->nmcstm) {
		ret = nmcstm_serialize(nmp->nmcstm, &buff[pos], size - pos);
		if (ret >= 0)
			pos += ret;
		if (pos != strlen(buff))
			pr_err("found mismatch between pos (%zu) and buff len (%zu)\n", pos, strlen(buff));
	}

	json_print_to_buffer(buff, size, 0, "}\n");

	/* write buffer to file */
	ret = write_buf_to_file(file_name, buff, strlen(buff));
	if (ret)
		pr_err("Failed to write to guest %d file\n", nmnicpf->guest_id);

	sync();
}

static int dev_mng_sw_init(struct nmp *nmp)
{
	int ret;
	struct nmdisp_params params;

	pr_info("Initializing Device software\n");

	/* Assign the pf_init_done callback */
	nmp->nmnicpf.f_ready_cb = dev_mng_pf_init_done;
	nmp->nmnicpf.arg = nmp;

	/* Initialize Dispatcher */
	ret = nmdisp_init(&params, &(nmp->nmdisp));
	if (ret)
		return ret;

	/* Save reference to Dispatcher in PF */
	nmp->nmnicpf.nmdisp = nmp->nmdisp;

	/* Initialize topology - all PF / VF instances */
	/* topology_init API is already defined in Linux therefore use pf_ prefix */
	ret = lf_init(nmp);
	return ret;
}


/** Device Initialization **/
int dev_mng_init(struct nmp *nmp)
{
	int ret;

	pr_info("Starting Device Manager Init\n");

	ret = dev_mng_hw_init(nmp);
	if (ret)
		return ret;

	ret = dev_mng_sw_init(nmp);
	if (ret)
		return ret;

	pr_info("Completed Device Manager init\n");
	return 0;
}


/** ======================== **/
/** == Device Termination == **/
/** ======================== **/


/** Hardware Functionality **/
/** ====================== **/

static int dev_mng_terminate_giu(struct nmp *nmp)
{
	int ret;

	ret = gie_terminate(nmp->nmnicpf.gie.mng_gie);
	if (ret)
		pr_warn("Failed to close management GIU\n");

	ret = gie_terminate(nmp->nmnicpf.gie.rx_gie);
	if (ret)
		pr_warn("Failed to close RX GIU\n");

	ret = gie_terminate(nmp->nmnicpf.gie.tx_gie);
	if (ret)
		pr_warn("Failed to close TX GIU\n");

	return ret;
}


static int dev_mng_mqa_terminate(struct nmp *nmp)
{
	int ret;

	pr_info("Terminating MQA\n");

	/* Terminating MQA tables */
	ret = mqa_deinit(nmp->nmnicpf.mqa);
	if (ret) {
		pr_err("Failed to terminate MQA\n");
		return ret;
	}

	return 0;
}


static int dev_mng_unmap_pci_func(struct pci_plat_func_map *map)
{
	(void)map;

	/* TODO: need to implement uio_unmap_mem */
	return 0;
}


static int dev_mng_hw_terminate(struct nmp *nmp)
{
	int ret;

	pr_info("Terminating Device Manager Init\n");

	/* terminate PP2 */
	ret = dev_mng_pp2_terminate(nmp);
	if (ret)
		return ret;

	/* TODO - for now just close the GIU instances */
	ret = dev_mng_terminate_giu(nmp);
	if (ret)
		return ret;

	/* Terminate MQA */
	ret = dev_mng_mqa_terminate(nmp);
	if (ret)
		return ret;

	/* Un-Map the NIC-PF */
	ret = dev_mng_unmap_pci_func(&nmp->nmnicpf.map);
	if (ret)
		return ret;

	return 0;
}

/** Software Functionality **/
/** ====================== **/

static int dev_mng_sw_terminate(struct nmp *nmp)
{
	int ret;

	/* Terminate scheduling */
#if 0
	ret = nmp_schedule_terminate();
	if (ret)
		return ret;
#endif

	/* Terminate topology */
	ret = lf_deinit(nmp);
	if (ret)
		return ret;

	return 0;
}


/** Device termination **/
int dev_mng_terminate(struct nmp *nmp)
{
	int ret;

	pr_info("Terminating Device Manager Init\n");

	ret = dev_mng_sw_terminate(nmp);
	if (ret)
		return ret;

	ret = dev_mng_hw_terminate(nmp);
	if (ret)
		return ret;

	return 0;
}


