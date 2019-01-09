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
#include "lf/pf/pf_topology.h"
#include "lf/custom/custom.h"
#include "lf/mng_cmd_desc.h"
#include "mng/mv_nmp_dispatch.h"
#include "lib/lib_misc.h"

#include "dev_mng.h"
#include "dev_mng_pp2.h"
#include "pci_ep_def.h"
#include "dispatch.h"


#define NMP_MSI_MAP_REGS_INDX	"0"

			     /*   Armada 7/8K        Armada 8K+  */
static char *nmp_msi_map_names[] = {"arm,gic-v2m-frame", "arm,gic-v3-its"};
static u8 nmp_msi_map_indxs[] = {0, 0};
static u16 nmp_msi_map_regs_offs[] = {0x40, 0x40};


/** =========================== **/
/** == Device Initialization == **/
/** =========================== **/

/** Hardware Functionality **/
/** ====================== **/

static int dev_mng_alloc_emul_pci_bars(struct nmp *nmp)
{
	struct mv_sys_dma_mem_info	 mem_info;
	char				 mem_name[100];

	/* Allocate configuration space memory */
	BUILD_BUG_ON(PCI_BAR0_CALC_SIZE > PCI_BAR0_ALLOC_SIZE); /* check that allocated size is enough */

	/* Allocate configuration space memory */
	BUILD_BUG_ON(PCI_BAR0_CALC_SIZE > PCI_BAR0_ALLOC_SIZE); /* check that allocated size is enough */

	nmp->emul_bars_avail_tbl = kcalloc(NMP_PCI_MAX_NUM_BARS, sizeof(int), GFP_KERNEL);
	if (nmp->emul_bars_avail_tbl == NULL)
		return -ENOMEM;
	memset(nmp->emul_bars_avail_tbl, 0, NMP_PCI_MAX_NUM_BARS * sizeof(int));

	nmp->emul_bars_mem = mv_sys_dma_mem_alloc(NMP_PCI_MAX_NUM_BARS * PCI_BAR0_ALLOC_SIZE, PCI_BAR0_ALLOC_ALIGN);
	if (nmp->emul_bars_mem == NULL) {
		pr_err("Failed to allocate platform configuration space.\n");
		return -ENOMEM;
	}
	mem_info.name = mem_name;
	mv_sys_dma_mem_get_info(&mem_info);
	/* the emulated-BAR offsets table MUST be the first thing allocated on the dma-able memory */
	if (mv_sys_dma_mem_virt2phys(nmp->emul_bars_mem) != mem_info.paddr) {
		pr_err("failed to allocate the NMP emulated-BAR offsets table");
		pr_err(" (the table MUST be the first thing allocated on the dma-able memory)!\n");
		return -ENOMEM;
	}

	return 0;
}

static int dev_mng_mqa_init(struct nmp *nmp)
{
	int ret;
	struct mqa_params params;

	pr_debug("Initializing MQA\n");

	params.num_qs = MQA_QUEUE_MAX;

	/* Initializing MQA tables */
	ret = mqa_init(&params, &(nmp->mqa));
	if (ret) {
		pr_err("Failed to initialize MQA tables\n");
		return ret;
	}

	pr_debug("Initializing MQA %p %p\n", nmp->mqa, nmp->mqa);

	return 0;
}

/* Initialize the GIU instance */
static int dev_mng_init_giu(struct nmp *nmp)
{
	struct sys_iomem_params iomem_params;
	struct giu_params giu_params;
	char dma_name[GIU_ENG_OUT_OF_RANGE][16];
	int ret, i;

	/* Initialize the management GIU instance */
	pr_debug("Initializing GIU devices\n");

	iomem_params.type = SYS_IOMEM_T_MMAP;

	for (i = 0; i < ARRAY_SIZE(nmp_msi_map_names); i++) {
		iomem_params.devname = nmp_msi_map_names[i];
		iomem_params.index = nmp_msi_map_indxs[i];

		ret = sys_iomem_init(&iomem_params, &nmp->msi_iomem);
		if (!ret)
			break;
	}
	if (i == ARRAY_SIZE(nmp_msi_map_names)) {
		pr_err("no MSI iomap found!\n");
		return -EFAULT;
	}

	/* Map the MSI-X registers */
	ret = sys_iomem_map(nmp->msi_iomem, NMP_MSI_MAP_REGS_INDX,
			(phys_addr_t *)&nmp->msi_regs.phys_addr,
			&nmp->msi_regs.virt_addr);
	if (ret) {
		pr_err("Failed to map msi!\n");
		sys_iomem_deinit(nmp->msi_iomem);
		return ret;
	}

	pr_debug("MSI-X regs mapped at virt:%p phys:%p\n",
		nmp->msi_regs.virt_addr, nmp->msi_regs.phys_addr);

	memset(&giu_params, 0, sizeof(giu_params));
	giu_params.mqa = nmp->mqa;
	giu_params.msi_regs_pa = (u64)nmp->msi_regs.phys_addr + nmp_msi_map_regs_offs[i];
	giu_params.msi_regs_va = (u64)nmp->msi_regs.virt_addr + nmp_msi_map_regs_offs[i];

	/* TODO: get all DMA-engines information from config file */
	giu_params.num_gies = 3;

	for (i = 0; i < giu_params.num_gies; i++) {
		sprintf(dma_name[i], "dmax2-%d", i);
		giu_params.gies_params[i].dma_eng_match = dma_name[i];
	}

	ret = giu_init(&giu_params, &nmp->giu);
	if (ret) {
		pr_err("Failed to initialize GIU!\n");
		sys_iomem_unmap(nmp->msi_iomem, NMP_MSI_MAP_REGS_INDX);
		sys_iomem_deinit(nmp->msi_iomem);
		return -ENODEV;
	}

	return 0;
}

static int dev_mng_hw_init(struct nmp *nmp)
{
	int ret;

	pr_debug("Initializing Device hardware\n");

	/* first thing, allocate memory for all emulated BARs */
	ret = dev_mng_alloc_emul_pci_bars(nmp);
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

static int dev_mng_sw_init(struct nmp *nmp, struct nmp_params *params)
{
	int ret;
	struct nmdisp_params nmdisp_params;
	struct lf_mng_params lf_mng_params;

	pr_debug("Initializing Device software\n");

	/* Initialize Dispatcher */
	memset(&nmdisp_params, 0, sizeof(nmdisp_params));
	/* nothing to initialize in nmdisp_params */
	ret = nmdisp_init(&nmdisp_params, &(nmp->nmdisp));
	if (ret)
		return ret;

	memset(&lf_mng_params, 0, sizeof(lf_mng_params));
	lf_mng_params.nmp = nmp;
	lf_mng_params.nmdisp = nmp->nmdisp;
	lf_mng_params.mqa = nmp->mqa;
	lf_mng_params.giu = nmp->giu;
	lf_mng_params.num_containers = params->num_containers;
	lf_mng_params.containers_params = params->containers_params;
	ret = lf_mng_init(&lf_mng_params, &nmp->lf_mng);
	if (ret)
		return ret;

	return 0;
}

/** ======================== **/
/** == Device Termination == **/
/** ======================== **/

/** Hardware Functionality **/
/** ====================== **/

static int dev_mng_terminate_giu(struct nmp *nmp)
{
	if (nmp->giu)
		giu_deinit(nmp->giu);

	sys_iomem_unmap(nmp->msi_iomem, NMP_MSI_MAP_REGS_INDX);
	sys_iomem_deinit(nmp->msi_iomem);

	return 0;
}

static int dev_mng_mqa_terminate(struct nmp *nmp)
{
	int ret;

	pr_debug("Terminating MQA\n");

	/* Terminating MQA tables */
	ret = mqa_deinit(nmp->mqa);
	if (ret) {
		pr_err("Failed to terminate MQA\n");
		return ret;
	}

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

	mv_sys_dma_mem_free(nmp->emul_bars_mem);

	return 0;
}

/** Software Functionality **/
/** ====================== **/

static int dev_mng_sw_terminate(struct nmp *nmp)
{
	if (!nmp)
		return -EFAULT;

	nmdisp_deinit(nmp->nmdisp);

	/* Terminate topology */
	lf_mng_deinit(nmp->lf_mng);

	return 0;
}


/** Device Initialization **/
int dev_mng_init(struct nmp *nmp, struct nmp_params *params)
{
	int ret;

	pr_debug("Starting Device Manager Init\n");

	ret = dev_mng_hw_init(nmp);
	if (ret)
		return ret;

	ret = dev_mng_sw_init(nmp, params);
	if (ret)
		return ret;

	pr_debug("Completed Device Manager init\n");
	return 0;
}

/** Device termination **/
int dev_mng_terminate(struct nmp *nmp)
{
	int ret;

	pr_debug("Terminating Device Manager Init\n");

	ret = dev_mng_sw_terminate(nmp);
	if (ret)
		return ret;

	ret = dev_mng_hw_terminate(nmp);
	if (ret)
		return ret;

	return 0;
}

int dev_mng_get_free_bar(struct nmp *nmp, void **va, dma_addr_t *pa)
{
	int i;

	if (!nmp) {
		pr_err("np NMP obj!\n");
		return -EINVAL;
	}

	for (i = 0; i < NMP_PCI_MAX_NUM_BARS; i++)
		if (nmp->emul_bars_avail_tbl[i] == 0) {
			nmp->emul_bars_avail_tbl[i] = 1;
			break;
		}

	if (i == NMP_PCI_MAX_NUM_BARS) {
		/* All entries are occupied, not likely to happen. */
		pr_err("All bar-offsets table are occupied (%d entries).\n",
			NMP_PCI_MAX_NUM_BARS);

		return -ENODEV;
	}

	*va = nmp->emul_bars_mem + i * PCI_BAR0_ALLOC_SIZE;
	/* Get the relevant physical address. */
	*pa = mv_sys_dma_mem_virt2phys(*va);

	return i;
}

void dev_mng_put_bar(struct nmp *nmp, int index)
{
	if (!nmp) {
		pr_err("np NMP obj!\n");
		return;
	}

	if (index == NMP_PCI_MAX_NUM_BARS) {
		pr_err("index out of range!\n");
		return;
	}

	nmp->emul_bars_avail_tbl[index] = 0;
}
