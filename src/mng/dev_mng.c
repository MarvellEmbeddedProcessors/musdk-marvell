/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#define log_fmt(fmt) "dev_mng: " fmt

#include "std_internal.h"
#include "env/trace/trc_pf.h"
#include "drivers/mv_mqa.h"
#include "drivers/mv_giu.h"
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


/** =========================== **/
/** == Device Initialization == **/
/** =========================== **/

/** Hardware Functionality **/
/** ====================== **/

/*
 * dev_mng_config_plat_func - Setup the platofm device registers to trigger
 * operation of the kernel based driver.
 */
static int dev_mng_config_plat_func(struct nmp *nmp)
{
	u32 cfg_mem_addr[2]; /* High and low */
	dma_addr_t phys_addr = (dma_addr_t)(uintptr_t)nmp->nmnicpf.map.cfg_map.phys_addr;

	pr_debug("Setting platform device registers.\n");
	cfg_mem_addr[0] = lower_32_bits(phys_addr) | 0x1;
	cfg_mem_addr[1] = upper_32_bits(phys_addr) | (0xCAFE << 16);

	writel(cfg_mem_addr[0], nmp->plat_regs.virt_addr + 0xA0);
	writel(cfg_mem_addr[1], nmp->plat_regs.virt_addr + 0xA4);

	return 0;
}

static int dev_mng_map_emul_pci_bar(struct nmp *nmp)
{
	/* Allocate configuration space memory */
	BUILD_BUG_ON(PCI_BAR0_CALC_SIZE > PCI_BAR0_ALLOC_SIZE); /* check that allocated size is enough */
	nmp->nmnicpf.map.cfg_map.virt_addr = mv_sys_dma_mem_alloc(PCI_BAR0_ALLOC_SIZE, PCI_BAR0_ALLOC_ALIGN);
	if (nmp->nmnicpf.map.cfg_map.virt_addr == NULL) {
		pr_err("Failed to allocate platform configuration space.\n");
		return -ENOMEM;
	}

	/* Get the relevant physical address. */
	nmp->nmnicpf.map.cfg_map.phys_addr =
		(void *)(uintptr_t)mv_sys_dma_mem_virt2phys(nmp->nmnicpf.map.cfg_map.virt_addr);

	/* Clear the config space, to prevent false device indications. */
	memset(nmp->nmnicpf.map.cfg_map.virt_addr, 0x0, PCI_BAR0_ALLOC_SIZE);

	pr_debug("Platform config space @ %p.\n", nmp->nmnicpf.map.cfg_map.phys_addr);
	/* Setup the "host_map", which is actually an identity mapping for the
	 * physical address.
	 * The virtual map, is not needed, as not entity in the mgmt side is
	 * accessing the virtual addresses in the "host" side, all accesses are
	 * done using a DMA.
	 * Thus, we set the virtual address to some "bad address" so that we
	 * can identify faulty accesses to host's virtual space.
	 */
	nmp->nmnicpf.map.host_map.phys_addr = 0x0;
	nmp->nmnicpf.map.host_map.virt_addr = (void *)0xBAD00ADD0BAD0ADDll;

	/* Configure device registers. */
	return dev_mng_config_plat_func(nmp);
}

static int dev_mng_map_plat_func(struct nmp *nmp)
{
	struct sys_iomem_params iomem_params;
	int ret;

	pr_debug("Mapping function %s\n", PLAT_AGNIC_UIO_NAME);

	iomem_params.type = SYS_IOMEM_T_UIO;
	iomem_params.devname = PLAT_AGNIC_UIO_NAME;
	iomem_params.index = 0;

	ret = sys_iomem_init(&iomem_params, &nmp->sys_iomem);
	if (ret)
		return ret;

	/* Map the agnic configuration registers */
	ret = sys_iomem_map(nmp->sys_iomem, "agnic_regs", (phys_addr_t *)&nmp->plat_regs.phys_addr,
			&nmp->plat_regs.virt_addr);
	if (ret)
		goto err_regs_map;

	pr_debug("agnic regs mapped at virt:%p phys:%p\n", nmp->plat_regs.virt_addr,
		   nmp->plat_regs.phys_addr);

	/* Map the MSI-X registers */
	ret = sys_iomem_map(nmp->sys_iomem, "msi_regs", (phys_addr_t *)&nmp->msi_regs.phys_addr,
			&nmp->msi_regs.virt_addr);
	if (ret)
		goto err_msi_regs_map;

	pr_debug("msi-x regs mapped at virt:%p phys:%p\n", nmp->msi_regs.virt_addr,
		   nmp->msi_regs.phys_addr);

	return 0;

err_msi_regs_map:
	sys_iomem_unmap(nmp->sys_iomem, "agnic_regs");
err_regs_map:
	sys_iomem_deinit(nmp->sys_iomem);

	return ret;
}

static int dev_mng_map_pci_bar(struct nmp *nmp)
{
	struct sys_iomem_params iomem_params;
	int ret;

	pr_debug("Mapping function %s\n", PCI_EP_UIO_MEM_NAME);

	iomem_params.type = SYS_IOMEM_T_UIO;
	iomem_params.devname = PCI_EP_UIO_MEM_NAME;
	iomem_params.index = 0;

	ret = sys_iomem_init(&iomem_params, &nmp->sys_iomem);
	if (ret)
		return ret;

	/* Map the whole physical Packet Processor physical address */
	ret = sys_iomem_map(nmp->sys_iomem,
			    PCI_EP_UIO_REGION_NAME,
			    (phys_addr_t *)&nmp->nmnicpf.map.cfg_map.phys_addr,
			    &nmp->nmnicpf.map.cfg_map.virt_addr);
	if (ret) {
		sys_iomem_deinit(nmp->sys_iomem);
		return ret;
	}

	pr_debug("BAR-0 of %s mapped at virt:%p phys:%p\n", PCI_EP_UIO_MEM_NAME,
		nmp->nmnicpf.map.cfg_map.virt_addr, nmp->nmnicpf.map.cfg_map.phys_addr);

	/* Map the whole physical Packet Processor physical address */
	ret = sys_iomem_map(nmp->sys_iomem, "host-map", (phys_addr_t *)&nmp->nmnicpf.map.host_map.phys_addr,
			&nmp->nmnicpf.map.host_map.virt_addr);
	if (ret) {
		sys_iomem_deinit(nmp->sys_iomem);
		return ret;
	}

	pr_debug("host RAM of %s remapped to phys %p virt %p\n", "host-map",
		   nmp->nmnicpf.map.host_map.phys_addr, nmp->nmnicpf.map.host_map.virt_addr);

	return 0;
}

static int dev_mng_map_bar(struct nmp *nmp)
{
	if (nmp->nmnicpf.map.type == ft_plat)
		return dev_mng_map_emul_pci_bar(nmp);
	else
		return dev_mng_map_pci_bar(nmp);
}

static int dev_mng_map_init(struct nmp *nmp)
{
	int ret;

	/* Map NMP registers if any */
	/* First, try to map the platform device, if failed, and PCI is supported
	 * try the pci device.
	 */
	nmp->nmnicpf.map.type = ft_plat;
	ret = dev_mng_map_plat_func(nmp);
	if (ret) {
		if (nmp->nmnicpf.profile_data.pci_en) {
			pr_debug("Platform device not found, trying the pci device.\n");
			nmp->nmnicpf.map.type = ft_pcie_ep;
		} else {
			pr_err("platform device not found\n");
			return ret;
		}
	}

	/* TODO: this should move into NICPF */
	ret = dev_mng_map_bar(nmp);
	if (ret) {
		pr_err("niether platform nor PCI devices were found\n");
		return ret;
	}

	return 0;
}

static int dev_mng_mqa_init(struct nmp *nmp)
{
	int ret;
	u64 pf_cfg_phys, pf_cfg_virt;
	struct mqa_params params;

	pr_debug("Initializing MQA\n");

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

	pr_debug("Initializing MQA %p %p\n", nmp->mqa, nmp->nmnicpf.mqa);

	return 0;
}

/* Initialize the GIU instance */
static int dev_mng_init_giu(struct nmp *nmp)
{
	struct giu_params giu_params;
	char dma_mng_name[16],  dma_rx_name[16],  dma_tx_name[16];
	int ret;

	/* Initialize the management GIU instance */
	pr_debug("Initializing GIU devices\n");

	memset(&giu_params, 0, sizeof(giu_params));
	giu_params.mqa = nmp->nmnicpf.mqa;
	giu_params.msi_regs_pa = (u64)nmp->msi_regs.phys_addr;
	giu_params.msi_regs_va = (u64)nmp->msi_regs.virt_addr;

	sprintf(dma_mng_name, "dmax2-%d", 0);
	giu_params.mng_gie_params.dma_eng_match = dma_mng_name;

	sprintf(dma_rx_name, "dmax2-%d", 1);
	giu_params.out_gie_params.dma_eng_match = dma_rx_name;

	sprintf(dma_tx_name, "dmax2-%d", 2);
	giu_params.in_gie_params.dma_eng_match = dma_tx_name;

	ret = giu_init(&giu_params, &nmp->nmnicpf.giu);
	if (ret) {
		pr_err("Failed to initialize GIU!\n");
		return -ENODEV;
	}

	return 0;
}

static int dev_mng_hw_init(struct nmp *nmp)
{
	int ret;

	pr_debug("Initializing Device hardware\n");

	/* Initialize NIC-PF map - (for QNPT/QNCT/etc.) */
	/* TODO: this is needed for MQa/GIU. however, this should be
	 * initialized on local memory and not on BAR; after fixing that,
	 * BAR should be initialized part of NICPF
	 */
	ret = dev_mng_map_init(nmp);
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

	ret = dev_mng_init_giu(nmp);
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

	pr_debug("nmp_pf_init_done reached\n");

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
	json_print_to_buffer(buff, size, 2, "\"phys_addr\": 0x%lx\n", (u64)mem_info.paddr);
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

	pr_debug("Initializing Device software\n");

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

/** ======================== **/
/** == Device Termination == **/
/** ======================== **/

/** Hardware Functionality **/
/** ====================== **/

static int dev_mng_terminate_giu(struct nmp *nmp)
{
	if (nmp->nmnicpf.giu)
		giu_deinit(nmp->nmnicpf.giu);

	return 0;
}

static int dev_mng_mqa_terminate(struct nmp *nmp)
{
	int ret;

	pr_debug("Terminating MQA\n");

	/* Terminating MQA tables */
	ret = mqa_deinit(nmp->nmnicpf.mqa);
	if (ret) {
		pr_err("Failed to terminate MQA\n");
		return ret;
	}

	return 0;
}

static void dev_mng_unmap_plat_func(struct nmp *nmp)
{
	mv_sys_dma_mem_free(nmp->nmnicpf.map.cfg_map.virt_addr);
}

static void dev_mng_unmap_emul_pci_bar(struct nmp *nmp)
{
	sys_iomem_unmap(nmp->sys_iomem, "msi_regs");
	sys_iomem_unmap(nmp->sys_iomem, "agnic_regs");
	sys_iomem_deinit(nmp->sys_iomem);
}

static void dev_mng_unmap_pci_bar(struct nmp *nmp)
{
	NOTUSED(nmp);
	/* TODO: complete! */
}

static void dev_mng_unmap_bar(struct nmp *nmp)
{
	if (nmp->nmnicpf.map.type == ft_plat)
		dev_mng_unmap_emul_pci_bar(nmp);
	else
		dev_mng_unmap_pci_bar(nmp);
}

static int dev_mng_map_terminate(struct nmp *nmp)
{
	if (nmp->nmnicpf.map.type == ft_plat)
		dev_mng_unmap_plat_func(nmp);

	/* TODO: this should move into NICPF */
	dev_mng_unmap_bar(nmp);

	return 0;
}

static int dev_mng_hw_terminate(struct nmp *nmp)
{
	int ret;

	pr_debug("Terminating Device Manager Init\n");

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

	regfile_destroy();

	/* Un-Map the NIC-PF */
	/* TODO: this is needed for MQa/GIU. however, this should be
	 * initialized on local memory and not on BAR; after fixing that,
	 * BAR should be initialized part of NICPF
	 */
	ret = dev_mng_map_terminate(nmp);
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
