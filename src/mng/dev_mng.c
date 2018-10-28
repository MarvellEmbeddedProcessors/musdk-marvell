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


#define NMP_AP806_MSI_MAP_NAME	"arm,gic-v2m-frame"
#define NMP_AP810_MSI_MAP_NAME	"arm,gic-v3-its"
#define NMP_MSI_MAP_INDX	0
#define NMP_MSI_MAP_REGS_INDX	"0"


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

	nmp->nmnicpf.mqa = nmp->mqa;

	pr_debug("Initializing MQA %p %p\n", nmp->mqa, nmp->nmnicpf.mqa);

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
	iomem_params.devname = NMP_AP806_MSI_MAP_NAME;
	iomem_params.index = NMP_MSI_MAP_INDX;

	ret = sys_iomem_init(&iomem_params, &nmp->msi_iomem);
	if (ret) {
		/* try AP810 */
		iomem_params.devname = NMP_AP810_MSI_MAP_NAME;

		ret = sys_iomem_init(&iomem_params, &nmp->msi_iomem);
		if (ret)
			return ret;
	}

	/* Map the MSI-X registers */
	ret = sys_iomem_map(nmp->msi_iomem, NMP_MSI_MAP_REGS_INDX, (phys_addr_t *)&nmp->msi_regs.phys_addr,
			&nmp->msi_regs.virt_addr);
	if (ret) {
		pr_err("Failed to map msi!\n");
		sys_iomem_deinit(nmp->msi_iomem);
		return ret;
	}

	pr_debug("MSI-X regs mapped at virt:%p phys:%p\n",
		nmp->msi_regs.virt_addr, nmp->msi_regs.phys_addr);

	memset(&giu_params, 0, sizeof(giu_params));
	giu_params.mqa = nmp->nmnicpf.mqa;
	giu_params.msi_regs_pa = (u64)nmp->msi_regs.phys_addr;
	giu_params.msi_regs_va = (u64)nmp->msi_regs.virt_addr;

	/* TODO: get all DMA-engines information from config file */
	giu_params.num_gies = 3;

	for (i = 0; i < giu_params.num_gies; i++) {
		sprintf(dma_name[i], "dmax2-%d", i);
		giu_params.gies_params[i].dma_eng_match = dma_name[i];
	}

	ret = giu_init(&giu_params, &nmp->nmnicpf.giu);
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

	sys_iomem_unmap(nmp->msi_iomem, NMP_MSI_MAP_REGS_INDX);
	sys_iomem_deinit(nmp->msi_iomem);

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

	mv_sys_dma_mem_free(nmp->emul_bars_mem);

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
