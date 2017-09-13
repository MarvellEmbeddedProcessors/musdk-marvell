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

#define log_fmt(fmt) "dev_mng: " fmt

#include "std_internal.h"
#include "env/trace/trc_pf.h"
#include "drivers/giu_internal.h"
#include "drivers/mv_mqa.h"
#include "db.h"
#include "lf/pf/pf.h"
#include "lf/pf/pf_regfile.h"
#include "lf/pf/pf_topology.h"
#include "lf/pf/pf_mng_cmd_desc.h"
#include "mng/mv_nmp_dispatch.h"

#include "dev_mng.h"
#include "pci_ep_def.h"
#include "dispatch.h"

#define PCI_EP_UIO_MEM_NAME "pci_ep"
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
	if (ret) {
		pr_err(" No device found\n");
		return ret;
	}

	/* Map the whole physical Packet Processor physical address */
	ret = sys_iomem_map(map->sys_iomem, "agnic_regs", (phys_addr_t *)&map->plat_regs.phys_addr,
			&map->plat_regs.virt_addr);
	if (ret)
		goto err_regs_map;

	pr_info("agnic regs mapped at virt:%p phys:%p\n", map->plat_regs.virt_addr,
		   map->plat_regs.phys_addr);

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
	if (ret) {
		pr_err(" No device found\n");
		return ret;
	}

	/* Map the whole physical Packet Processor physical address */
	ret = sys_iomem_map(map->sys_iomem, "bar0", (phys_addr_t *)&map->cfg_map.phys_addr, &map->cfg_map.virt_addr);
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

static int dev_mng_mqa_init(struct nmp_dev *dev)
{
	int ret;
	u64 pf_cfg_phys, pf_cfg_virt;
	struct mqa_params params;

	pr_info("Initializing MQA\n");

	/* Initializing MQA botification tables base address */
	pf_cfg_phys = (u64)dev->nic_pf.map.cfg_map.phys_addr;
	pf_cfg_virt = (u64)dev->nic_pf.map.cfg_map.virt_addr;

	params.num_qs = MQA_QUEUE_MAX;
	params.notif_tbl.qnpt_pa = (phys_addr_t)(pf_cfg_phys + PCI_BAR0_MQA_QNPT_BASE);
	params.notif_tbl.qnct_pa = (phys_addr_t)(pf_cfg_phys + PCI_BAR0_MQA_QNCT_BASE);
	params.notif_tbl.qnpt_va = (void *)(pf_cfg_virt + PCI_BAR0_MQA_QNPT_BASE);
	params.notif_tbl.qnct_va = (void *)(pf_cfg_virt + PCI_BAR0_MQA_QNCT_BASE);

	/* Initializing MQA tables */
	ret = mqa_init(&params, &(dev->mqa));
	if (ret) {
		pr_err("Failed to initialize MQA tables\n");
		return ret;
	}

	dev->nic_pf.mqa = dev->mqa;

	pr_info("Initializing MQA %p %p\n", dev->mqa, dev->nic_pf.mqa);

	return 0;
}


/* Initialize the management, TX, and RX giu instances */
static int dev_mng_init_giu(struct nmp_dev *dev)
{
	struct giu_params giu_pars;

	/* Initialize the management GIU instance */
	pr_info("Initializing GIU devices\n");

	giu_pars.gct_base = (u64)dev->nic_pf.mqa->qct_base;
	giu_pars.gpt_base = (u64)dev->nic_pf.mqa->qpt_base;
	giu_pars.gncs_base = (u64)dev->nic_pf.mqa->qnct_base;
	giu_pars.gnps_base = (u64)dev->nic_pf.mqa->qnpt_base;

	/* TODO - setup the MSI/MSI-X tables */
	giu_pars.msi_base = 0;
	giu_pars.msix_base = 0;

	dev->nic_pf.giu.mng_giu = giu_open(&giu_pars, "mng");
	if (!dev->nic_pf.giu.mng_giu) {
		pr_err("Failed to initialize management GIU\n");
		return -ENODEV;
	}

	dev->nic_pf.giu.rx_giu = giu_open(&giu_pars, "rx");
	if (!dev->nic_pf.giu.rx_giu) {
		pr_err("Failed to initialize RX GIU\n");
		goto error;
	}

	dev->nic_pf.giu.tx_giu = giu_open(&giu_pars, "tx");
	if (!dev->nic_pf.giu.tx_giu) {
		pr_err("Failed to initialize TX GIU\n");
		goto error;
	}

	return 0;

error:
	giu_close(dev->nic_pf.giu.mng_giu);
	if (dev->nic_pf.giu.rx_giu)
		giu_close(dev->nic_pf.giu.rx_giu);

	return -ENODEV;
}


static int dev_mng_hw_init(void)
{
	int ret;

	pr_info("Initializing Device hardware\n");

	/* Map the NIC-PF */
	/* First, try to map the platform device, if failed, try the pci
	 * device.
	 */
	dev.nic_pf.map.type = ft_plat;
	ret = dev_mng_map_plat_func(&dev.nic_pf.map);
	if (ret) {
		pr_info("Platform device not found, trying the pci device.\n");
		dev.nic_pf.map.type = ft_pcie_ep;
		ret = dev_mng_map_pci_func(&dev.nic_pf.map);
	}
	if (ret)
		return ret;

	/* Initialize Register File utility */
	ret = regfile_init();
	if (ret)
		return ret;

	/* Initialize MQA - device queue management */
	ret = dev_mng_mqa_init(&dev);
	if (ret)
		return ret;

	ret = dev_mng_init_giu(&dev);
	if (ret)
		return ret;

	return 0;
}


/** Software Functionality **/
/** ====================== **/

static int dev_mng_sw_init(void)
{
	int ret;
	struct nmdisp_params params;

	pr_info("Initializing Device software\n");

	/* Initialize DB */
	ret = db_init();
	if (ret)
		return ret;

	/* Initialize Dispatcher */
	ret = nmdisp_init(&params, &(dev.nmdisp));
	if (ret)
		return ret;

	/* Save reference to Dispatcher in PF */
	dev.nic_pf.nmdisp = dev.nmdisp;

	/* Initialize topology - all PF / VF instances */
	/* topology_init API is already defined in Linux therefore use pf_ prefix */
	ret = pf_topology_init();
	return ret;
}


/** Device Initialization **/
int dev_mng_init(void)
{
	int ret;

	pr_info("Starting Device Manager Init\n");

	ret = dev_mng_hw_init();
	if (ret)
		return ret;

	ret = dev_mng_sw_init();
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

static int dev_mng_terminate_giu(struct nmp_dev *dev)
{
	int ret;

	ret = giu_close(dev->nic_pf.giu.mng_giu);
	if (ret)
		pr_warn("Failed to close management GIU\n");

	ret = giu_close(dev->nic_pf.giu.rx_giu);
	if (ret)
		pr_warn("Failed to close RX GIU\n");

	ret = giu_close(dev->nic_pf.giu.tx_giu);
	if (ret)
		pr_warn("Failed to close TX GIU\n");

	return ret;
}


static int dev_mng_mqa_terminate(void)
{
	int ret;

	pr_info("Terminating MQA\n");

	/* Terminating MQA tables */
	ret = mqa_deinit(dev.nic_pf.mqa);
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


static int dev_mng_hw_terminate(void)
{
	int ret;

	pr_info("Terminating Device Manager Init\n");

	/* TODO - for now just close the GIU instances */
	ret = dev_mng_terminate_giu(&dev);
	if (ret)
		return ret;

	/* Terminate MQA */
	ret = dev_mng_mqa_terminate();
	if (ret)
		return ret;

	/* Un-Map the NIC-PF */
	ret = dev_mng_unmap_pci_func(&dev.nic_pf.map);
	if (ret)
		return ret;

	return 0;
}

/** Software Functionality **/
/** ====================== **/

static int dev_mng_sw_terminate(void)
{
	int ret;

	/* Terminate scheduling */
#if 0
	ret = nmp_schedule_terminate();
	if (ret)
		return ret;
#endif

	/* Terminate topology */
	ret = pf_topology_terminate();
	if (ret)
		return ret;

	return 0;
}


/** Device termination **/
int dev_mng_terminate(void)
{
	int ret;

	pr_info("Terminating Device Manager Init\n");

	ret = dev_mng_sw_terminate();
	if (ret)
		return ret;

	ret = dev_mng_hw_terminate();
	if (ret)
		return ret;

	return 0;
}

