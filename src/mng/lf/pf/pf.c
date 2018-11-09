/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#define log_fmt(fmt) "pf: " fmt

#include "std_internal.h"
#include "drivers/mv_mqa.h"
#include "drivers/mv_mqa_queue.h"
#include "drivers/mv_giu.h"
#include "drivers/mv_giu_gpio.h"
#include "drivers/mqa_def.h"
#include "drivers/giu_regfile_def.h"
#include "mng/mv_nmp.h"
#include "mng/mv_nmp_guest_msg.h"
#include "mng/mv_nmp_dispatch.h"

#include "mng/include/guest_mng_cmd_desc.h"
#include "mng/pci_ep_def.h"
#include "mng/lf/mng_cmd_desc.h"

#include "pf_regfile.h"
#include "pf.h"
#include "pf_pp2.h"
#include "pf_topology.h"
#include "pf_pci_if_desc.h"
#include "env/trace/trc_pf.h"


#define PLAT_AGNIC_UIO_NAME	"agnic"

#define REGFILE_VAR_DIR		"/var/"
#define REGFILE_NAME_PREFIX	"nic-pf-"
#define REGFILE_MAX_FILE_NAME	64

/* TODO: These should be removed. The local queue sizes should match the remote
 * management queue sizes, as received during the init sequence.
 */
#define LOCAL_CMD_QUEUE_SIZE	256
#define LOCAL_NOTIFY_QUEUE_SIZE	256

#define REGFILE_VERSION		000002	/* Version Format: XX.XX.XX*/

#define NMP_MAX_BUF_STR_LEN		256


static int init_nicpf_params(struct nmnicpf *nmnicpf, struct nmp_lf_nicpf_params *params)
{
	struct pf_profile		*pf_profile;
	int				 k;

	pf_profile = &(nmnicpf->profile_data);

	/* TODO - return error once all sync with this change */
	if (!params->match)
		pr_warn("GPIO match should be given\n");
	else
		strcpy(pf_profile->match, params->match);
	pf_profile->pci_en = params->pci_en;
	pf_profile->max_num_tcs        = params->max_num_tcs;
	pf_profile->lcl_egress_q_num   = 1;
	pf_profile->lcl_egress_q_size  = params->lcl_egress_qs_size;
	pf_profile->lcl_ingress_q_num  = 1;
	pf_profile->lcl_ingress_q_size = params->lcl_ingress_qs_size;
	pf_profile->lcl_bm_q_num       = params->lcl_num_bpools;
	if (pf_profile->lcl_bm_q_num > 1) {
		pr_err("NMP supports only one lcl_bpool for GIU in current release\n");
		return -EINVAL;
	}
	pf_profile->lcl_bm_q_size      = params->lcl_bpools_params[0].max_num_buffs;
	pf_profile->lcl_bm_buf_size    = params->lcl_bpools_params[0].buff_size;
	pf_profile->dflt_pkt_offset = params->dflt_pkt_offset;

	pf_profile->keep_alive_thresh = params->keep_alive_thresh;

	pf_profile->port_type = params->type;
	if (pf_profile->port_type == NMP_LF_NICPF_T_PP2_PORT) {
		strcpy(pf_profile->port_match, params->port_params.pp2_port.match);
		pf_profile->pp2_port.match = pf_profile->port_match;
		pf_profile->pp2_port.lcl_num_bpools = params->port_params.pp2_port.lcl_num_bpools;
		for (k = 0; k < pf_profile->pp2_port.lcl_num_bpools; k++) {
			pf_profile->pp2_port.lcl_bpools_params[k].buff_size =
				params->port_params.pp2_port.lcl_bpools_params[k].buff_size;
			pf_profile->pp2_port.lcl_bpools_params[k].max_num_buffs =
				params->port_params.pp2_port.lcl_bpools_params[k].max_num_buffs;
		}
	}

	return 0;
}

/**
 * NIC PF Register File Section
 * ============================
 */

/*
 *	nmnicpf_regfile_size
 *
 *	This function calculates the regfile data byte size
 *
 *	@param[in]      nmnicpf       - Pointer to the NIC-PF struct which defines the topology
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise (< 0)
 */
static int nmnicpf_regfile_size(struct nmnicpf *nmnicpf)
{
	int size;
	u32 tc_id;
	struct giu_gpio_intc_params *intc;
	struct giu_gpio_outtc_params *outtc;
	struct giu_gpio_params *gpio_p = &(nmnicpf->gpio_params);

	/* Main topology structure size */
	size = sizeof(struct giu_regfile);

	/* Add Egress TCs size */
	size += (sizeof(struct giu_tc) * (gpio_p->num_intcs));

	/* Go trough Egress TCs and calc size of queues */
	for (tc_id = 0; tc_id < (gpio_p->num_intcs); tc_id++) {
		intc = &(gpio_p->intcs_params[tc_id]);
		/* Add BM Pool size */
		size += (sizeof(struct giu_queue) * intc->num_inpools);
		size += (sizeof(struct giu_queue) * (intc->num_inqs));
	}

	/* Add Ingress TCs size */
	size += (sizeof(struct giu_tc) * (gpio_p->num_outtcs));

	/* Go trough Egress TCs and calc size of queues */
	for (tc_id = 0; tc_id < (gpio_p->num_outtcs); tc_id++) {
		outtc = &(gpio_p->outtcs_params[tc_id]);
		size += (sizeof(struct giu_queue) * (outtc->num_outqs));
	}

	return size;
}

/*
 *	nmnicpf_regfile_open
 *
 *	This function opens the regfile
 *
 *	@param[in]      nmnicpf       - Pointer to the NIC-PF struct which defines the topology
 *			file_map   - Pointer to Pointer to Regfile mempry map
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise (< 0)
 */
static int nmnicpf_regfile_open(struct nmnicpf *nmnicpf, void **file_map)
{
	int size;
	char file_name[REGFILE_MAX_FILE_NAME];

	/* Concatenate file path */
	snprintf(file_name, sizeof(file_name), "%s%s%d", REGFILE_VAR_DIR, REGFILE_NAME_PREFIX, nmnicpf->pf_id);

	/* Configure queue topology in register file */
	size = nmnicpf_regfile_size(nmnicpf);

	if (size < 0) {
		pr_err("Error: failed to map file %s (size %d)\n", file_name, size);
		return -ENOENT;
	}

	*file_map = regfile_open(file_name, size);
	if (file_map == NULL) {
		pr_err("Error: failed to map file %s\n", file_name);
		return -ENOENT;
	}

	pr_debug("Regfile Parameters (Regfile Name: %s, Ver %06d)\n", file_name, REGFILE_VERSION);

	return 0;
}

/*
 *	nmnicpf_regfile_close
 *
 *	This function opens the regfile
 *
 *	@param[in]      file_map   - Pointer to Regfile mempry map for closing the file
 *
 */
static void nmnicpf_regfile_close(void *file_map)
{
	regfile_close(file_map);
}

/*
 *	nmnicpf_config_header_regfile
 *
 *	This function writes the header pf to the regfile
 *
 *	@param[in]	regfile_data - Struct of the regfile data
 *			file_map   - Pointer to Pointer to Regfile mempry map
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise (< 0)
 */
static int nmnicpf_config_header_regfile(struct giu_regfile *regfile_data, void **file_map)
{
	/* Set Regfile header configuration */
	memcpy(*file_map, regfile_data, sizeof(struct giu_regfile));
	*file_map += sizeof(struct giu_regfile);

	return 0;
}

/*
 *	nmnicpf_config_topology_and_update_regfile
 *
 *	This function configures the entite NIC-PF struct params to the register file
 *      It runs over all TCs types (Egress / Ingress / ...) and BM Queues structure and convert the
 *      Topology to the register file
 *
 *	@param[in]	nmnicpf      - Pointer to the NIC-PF struct which defines the topology
 *+
 *	@retval	0 on success
 *	@retval	error-code otherwise (< 0)
 */
static int nmnicpf_config_topology_and_update_regfile(struct nmnicpf *nmnicpf)
{
	void *file_map;
	struct giu_regfile regfile_data;
	struct giu_gpio_params *gpio_p = &(nmnicpf->gpio_params);
	struct mv_sys_dma_mem_info	 mem_info;
	char				 dev_name[100];
	u8 bm_idx;
	int ret = 0;

	mem_info.name = dev_name;
	mv_sys_dma_mem_get_info(&mem_info);

	/* Update Regfile general info */
	strcpy(regfile_data.dma_uio_mem_name, mem_info.name);

	regfile_data.version		= REGFILE_VERSION;
	regfile_data.num_bm_qs		= gpio_p->intcs_params[0].num_inpools;
	regfile_data.bm_qs		= NULL;
	regfile_data.num_egress_tcs	= gpio_p->num_intcs;
	regfile_data.egress_tcs	= NULL;
	regfile_data.num_ingress_tcs	= gpio_p->num_outtcs;
	regfile_data.ingress_tcs	= NULL;
	regfile_data.flags = 0;

	pr_debug("Start Topology configuration to register file [Regfile ver (%d), NIC-PF number (%d)]\n",
			regfile_data.version, nmnicpf->pf_id);

	ret = nmnicpf_regfile_open(nmnicpf, &file_map);
	if (ret != 0)
		return ret;

	/* Copy Header File parameters */
	pr_debug("Copy Regfile Header\n");
	ret = nmnicpf_config_header_regfile(&regfile_data, &file_map);
	if (ret != 0)
		goto config_error;

	/* Setup GIU B-Pools  */
	/* we assume all in-TCs share the same BPools */
	for (bm_idx = 0;
		bm_idx < nmnicpf->gpio_params.intcs_params[0].num_inpools;
		bm_idx++) {
		ret = giu_bpool_serialize(nmnicpf->giu_bpools[bm_idx], &file_map);
		if (ret != 0)
			goto config_error;
	}

	/* Setup GIU GPIO  */
	ret = giu_gpio_serialize(nmnicpf->giu_gpio, &file_map);
	if (ret != 0)
		goto config_error;

	return ret;

config_error:

	nmnicpf_regfile_close(file_map);

	return ret;
}

/**
 * NIC PF Initialization Section
 * =============================
 */

/*
 *	nmnicpf_topology_local_queue_init
 *
 *	This function initialize local queues in NIC PF queue
 *	topology database based on configuration profile
 *	Local queues include data queues and bm queues
 *
 *	@param[in]	nmnicpf - pointer to NIC PF object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
static int nmnicpf_topology_local_queue_init(struct nmnicpf *nmnicpf)
{
	int ret;
	struct pf_profile *prof = &(nmnicpf->profile_data);
	struct giu_gpio_params *gpio_p = &(nmnicpf->gpio_params);

	pr_debug("Initializing Local Queues in management Database\n");

	/* Local Egress TC */
	ret = pf_intc_queue_init(nmnicpf, LCL, gpio_p->num_intcs, prof->lcl_egress_q_num);
	if (ret) {
		pr_err("Failed to allocate Local Egress TC table\n");
		goto queue_error;
	}

	/* Local ingress TC */
	ret = pf_outtc_queue_init(nmnicpf, LCL, gpio_p->num_outtcs, prof->lcl_ingress_q_num);
	if (ret) {
		pr_err("Failed to allocate Local Ingress TC table\n");
		goto queue_error;
	}

	/* Local BM */
	ret = pf_intc_bm_queue_init(nmnicpf, prof->lcl_bm_q_num);
	if (ret) {
		pr_err("Failed to allocate Local BM table\n");
		goto queue_error;
	}

	return 0;

queue_error:
	pf_intc_queue_free(nmnicpf, LCL, gpio_p->num_intcs);
	pf_outtc_queue_free(nmnicpf, LCL, gpio_p->num_outtcs);
	pf_intc_bm_queue_free(nmnicpf);

	return -ENOMEM;
}

/*
 *	nmnicpf_topology_local_tc_free
 *
 *	This function frees the local TC resources in topology
 */
static void nmnicpf_topology_local_tc_free(struct nmnicpf *nmnicpf)
{
	struct giu_gpio_params *gpio_p = &(nmnicpf->gpio_params);

	pr_debug("Free Local queues DB\n");
	pf_intc_queue_free(nmnicpf, LCL, gpio_p->num_intcs);
	pf_outtc_queue_free(nmnicpf, LCL, gpio_p->num_outtcs);
	pf_intc_bm_queue_free(nmnicpf);
}

/*
 *	nmnicpf_topology_remote_queue_init
 *
 *	This function initialize NIC PF remote queue
 *	topology database based on PF_INIT management command
 *	Remote queues include data queues and bm queues
 *
 *	@param[in]	nmnicpf - pointer to NIC PF object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
static int nmnicpf_topology_remote_queue_init(struct nmnicpf *nmnicpf)
{
	int ret;
	struct giu_gpio_params *gpio_p = &(nmnicpf->gpio_params);

	pr_debug("Initializing Remote Queues in management Database\n");

	/* Remote Egress TC */
	/* TC queues will be update upon "tc_add" command */
	ret = pf_intc_queue_init(nmnicpf, REM, gpio_p->num_intcs, 0);
	if (ret) {
		pr_err("Failed to allocate Local Egress TC table\n");
		goto queue_error;
	}

	/* Remote ingress TC */
	/* TC queues will be update upon "tc_add" command */
	ret = pf_outtc_queue_init(nmnicpf, REM, gpio_p->num_outtcs, 0);
	if (ret) {
		pr_err("Failed to allocate Local Ingress TC table\n");
		goto queue_error;
	}

	return 0;

queue_error:

	pr_err("Remote Queues Initialization failed\n");

	pf_intc_queue_free(nmnicpf, REM, gpio_p->num_intcs);
	pf_outtc_queue_free(nmnicpf, REM, gpio_p->num_outtcs);

	return -ENOMEM;
}

/*
 *	nmnicpf_topology_remote_tc_free
 *
 *	This function frees the remote TC resources in topology
 */
static int nmnicpf_topology_remote_tc_free(struct nmnicpf *nmnicpf)
{
	struct giu_gpio_params *gpio_p = &(nmnicpf->gpio_params);

	pr_debug("Free Remote queues DB\n");
	pf_intc_queue_free(nmnicpf, REM, gpio_p->num_intcs);
	pf_outtc_queue_free(nmnicpf, REM, gpio_p->num_outtcs);

	return 0;
}

/*
 *	nmnicpf_topology_tc_free
 *
 *	This function frees both local & remote TC resources in DB
 */
static int nmnicpf_topology_tc_free(struct nmnicpf *nmnicpf)
{
	/* Free Local TC structures */
	nmnicpf_topology_local_tc_free(nmnicpf);

	/* Free Remote TC structures */
	nmnicpf_topology_remote_tc_free(nmnicpf);

	return 0;
}

/*
 *	nmnicpf_topology_local_queue_cfg
 *
 *	This function create NIC PF local queue based on configuration profile
 *	Local queues include data queues and bm queues
 *
 *	@param[in]	nmnicpf - pointer to NIC PF object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
static int nmnicpf_topology_local_queue_cfg(struct nmnicpf *nmnicpf)
{
	struct giu_gpio_outtc_params *outtc;
	struct giu_gpio_intc_params *intc;
	struct pf_profile *prof = &(nmnicpf->profile_data);
	struct giu_gpio_params *gpio_p = &(nmnicpf->gpio_params);
	u32 tc_idx, bm_idx, q_idx;

	/* Create Local BM queues */
	pr_debug("Configure Local Egress TC queues (#TCs %d)\n", gpio_p->num_intcs);
	for (tc_idx = 0; tc_idx < gpio_p->num_intcs; tc_idx++) {
		intc = &(gpio_p->intcs_params[tc_idx]);

		pr_debug("Configure Local BM queues (Num of queues %d) of TC %d\n",
			intc->num_inpools, tc_idx);
		for (bm_idx = 0; bm_idx < intc->num_inpools; bm_idx++)
			/* Init queue parameters */
			intc->pools[bm_idx] = nmnicpf->giu_bpools[bm_idx];

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

/*
 *	nmnicpf_mng_chn_init
 *
 *	This function create NIC PF management channel
 *	Execution requires handshake with Host side
 *
 *  The creation flow is:
 *  - Create Local Qs
 *  - Wait for the Host to indicate 'Host Management Ready'
 *  - Register Host Command Q (and set Producer index in BAR 0)
 *  - Register Host Notification Q (and set Consumer index in BAR 0)
 *  - Associate Local Command Q with Host Q: Host (src) --> Local (Dest)
 *  - Associate Local Notification Q with Host Q: Local (src) --> Host (Dest)
 *  - Set 'Device Management Ready' indication
 *
 *	@param[in]	nmnicpf - pointer to NIC PF object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
static int nmnicpf_mng_chn_init(struct nmnicpf *nmnicpf)
{
	volatile struct pcie_config_mem *pcie_cfg;
	struct giu_mng_ch_params mng_ch_params;
	u64 pf_cfg_virt; /* pointer to HW so it should be volatile */
	u8 mac_addr[ETH_ALEN];
	int ret = 0;

	/* get the initial mac-addr from the pp2 port's (if exist) kernel side as this is the correct mac-addr. */
	nmnicpf_pp2_get_mac_addr(nmnicpf, mac_addr);

	/*  Host Ready Check */
	/* ================= */

	/* Get BAR0 Configuration space base address */
	pf_cfg_virt = (u64)nmnicpf->map.cfg_map.virt_addr;
	pcie_cfg    = (struct pcie_config_mem *)pf_cfg_virt;

	/* Wait for Host to update the state to 'Host Management Ready'
	 * This means that BAR 0 configuration can be accessed as the
	 * Host updated the relevant data/fields.
	 */
	pr_info("Wait till Host change the status to 'Host Management Ready'\n");

	pcie_cfg->mac_addr[0] = mac_addr[0];
	pcie_cfg->mac_addr[1] = mac_addr[1];
	pcie_cfg->mac_addr[2] = mac_addr[2];
	pcie_cfg->mac_addr[3] = mac_addr[3];
	pcie_cfg->mac_addr[4] = mac_addr[4];
	pcie_cfg->mac_addr[5] = mac_addr[5];

	/*
	 * MSI-X table base/offset.
	 */
	pcie_cfg->msi_x_tbl_offset = PCI_BAR0_MSI_X_TBL_BASE;
	/* update the total memory needed for the device side */
	pcie_cfg->dev_use_size = PCI_BAR0_CALC_SIZE;

	/* Register MSI-X table base in GIU */
	/* TODO: register msix table only if pcie_cfg->msi_x_tbl_offset !=0
	 *       netdev side should reset it if it doesn't support msi
	 *	 hence, it should move after "Host is Ready" (below)
	 */
	nmnicpf->gpio_params.msix_table_base = (void *)(pf_cfg_virt + pcie_cfg->msi_x_tbl_offset);

	/* Make sure that above configuration are out before setting the
	 * dev-ready status for the host side.
	 */
	writel(PCIE_CFG_STATUS_DEV_READY, &pcie_cfg->status);

	while (!(readl(&pcie_cfg->status) & PCIE_CFG_STATUS_HOST_MGMT_READY))
		; /* Do Nothing. Wait till state it's updated */

	pr_debug("Host is Ready\n");

	memset(&mng_ch_params, 0, sizeof(mng_ch_params));

	mng_ch_params.rem_base_pa = (dma_addr_t)nmnicpf->map.host_map.phys_addr;
	mng_ch_params.rem_base_va = nmnicpf->map.host_map.virt_addr;
	mng_ch_params.desc_size = sizeof(struct cmd_desc);

	mng_ch_params.lcl_cmd_q.len = LOCAL_CMD_QUEUE_SIZE;
	mng_ch_params.lcl_resp_q.len = LOCAL_NOTIFY_QUEUE_SIZE;
	mng_ch_params.rem_cmd_q.len = pcie_cfg->cmd_q.len;
	mng_ch_params.rem_cmd_q.pa = pcie_cfg->cmd_q.q_addr;
	mng_ch_params.rem_cmd_q.prod_pa =
		(dma_addr_t)(pcie_cfg->cmd_q.q_prod_offs + nmnicpf->map.cfg_map.phys_addr);
	mng_ch_params.rem_cmd_q.cons_pa =
		(dma_addr_t)(pcie_cfg->cmd_q.q_cons_offs + nmnicpf->map.cfg_map.phys_addr);
	mng_ch_params.rem_cmd_q.prod_va =
		(void *)(pcie_cfg->cmd_q.q_prod_offs + nmnicpf->map.cfg_map.virt_addr);
	mng_ch_params.rem_cmd_q.cons_va =
		(void *)(pcie_cfg->cmd_q.q_cons_offs + nmnicpf->map.cfg_map.virt_addr);

	mng_ch_params.rem_resp_q.len = pcie_cfg->notif_q.len;
	mng_ch_params.rem_resp_q.pa = pcie_cfg->notif_q.q_addr;
	mng_ch_params.rem_resp_q.prod_pa =
		(dma_addr_t)(pcie_cfg->notif_q.q_prod_offs + nmnicpf->map.cfg_map.phys_addr);
	mng_ch_params.rem_resp_q.cons_pa =
		(dma_addr_t)(pcie_cfg->notif_q.q_cons_offs + nmnicpf->map.cfg_map.phys_addr);
	mng_ch_params.rem_resp_q.prod_va =
		(void *)(pcie_cfg->notif_q.q_prod_offs + nmnicpf->map.cfg_map.virt_addr);
	mng_ch_params.rem_resp_q.cons_va =
		(void *)(pcie_cfg->notif_q.q_cons_offs + nmnicpf->map.cfg_map.virt_addr);

	ret = giu_mng_ch_init(nmnicpf->giu, &mng_ch_params, &nmnicpf->giu_mng_ch);
	if (ret) {
		pr_err("faied to initialize GIU Management channel!\n");
		return ret;
	}

	/* Device Ready */
	/* ============ */

	/* make sure all writes are done before updating the status */
	writel(readl(&pcie_cfg->status) | PCIE_CFG_STATUS_DEV_MGMT_READY, &pcie_cfg->status);

	/* Set state to 'Device Management Ready' */
	pr_debug("Set status to 'Device Management Ready'\n");

	return 0;
}

/*
 * nmnicpf_config_plat_func - Setup the platofm device registers to trigger
 * operation of the kernel based driver.
 */
static int nmnicpf_config_plat_func(struct nmnicpf *nmnicpf)
{
	u32				 cfg_mem_addr[2]; /* High and low */
	void				*cfg_mem_va;
	dma_addr_t			 phys_addr =
		(dma_addr_t)(uintptr_t)nmnicpf->map.cfg_map.phys_addr;

	pr_debug("Setting platform device registers.\n");

	/* TODO: need to read the correct address directly from DTS */
	cfg_mem_va = nmnicpf->plat_regs.virt_addr + CFG_MEM_AP8xx_OFFS;

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
			((nmnicpf->plat_bar_indx & CFG_MEM_BIDX_MASK) << CFG_MEM_BIDX_SHIFT);
		cfg_mem_addr[1] |= upper_32_bits(phys_addr);
		pr_debug("passing 64bits reg: pa: %"PRIx64", bar-idx: %d (reg: %"PRIx32"%"PRIx32")\n",
			phys_addr, nmnicpf->plat_bar_indx, cfg_mem_addr[1], cfg_mem_addr[0]);

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
			((nmnicpf->plat_bar_indx & CFG_MEM_BIDX_MASK) << CFG_MEM_BIDX_SHIFT);
		pr_debug("passing 32bits reg: pa: %"PRIx64", bar-idx: %d (reg: %"PRIx32")\n",
			phys_addr, nmnicpf->plat_bar_indx, cfg_mem_addr[0]);

		writel(cfg_mem_addr[0], cfg_mem_va);
	}

	return 0;
}

static int nmnicpf_map_emul_pci_bar(struct nmnicpf *nmnicpf)
{
	nmnicpf->plat_bar_indx = nmnicpf->f_get_free_bar_cb(nmnicpf->arg,
			&nmnicpf->map.cfg_map.virt_addr,
			(dma_addr_t *)&nmnicpf->map.cfg_map.phys_addr);

	if (nmnicpf->plat_bar_indx < 0) {
		pr_err("no emulated-BARs left!\n");
		return -ENAVAIL;
	}

	/* Clear the config space, to prevent false device indications. */
	memset(nmnicpf->map.cfg_map.virt_addr, 0x0, PCI_BAR0_ALLOC_SIZE);

	pr_debug("Platform config space @ %p.\n", nmnicpf->map.cfg_map.phys_addr);
	/* Setup the "host_map", which is actually an identity mapping for the
	 * physical address.
	 * The virtual map, is not needed, as not entity in the mgmt side is
	 * accessing the virtual addresses in the "host" side, all accesses are
	 * done using a DMA.
	 * Thus, we set the virtual address to some "bad address" so that we
	 * can identify faulty accesses to host's virtual space.
	 */
	nmnicpf->map.host_map.phys_addr = 0x0;
	nmnicpf->map.host_map.virt_addr = (void *)0xBAD00ADD0BAD0ADDll;

	/* Configure device registers. */
	return nmnicpf_config_plat_func(nmnicpf);
}

static int nmnicpf_map_plat_func(struct nmnicpf *nmnicpf)
{
	struct sys_iomem_params iomem_params;
	int ret;

	pr_debug("Mapping function %s\n", PLAT_AGNIC_UIO_NAME);

	iomem_params.type = SYS_IOMEM_T_UIO;
	iomem_params.devname = PLAT_AGNIC_UIO_NAME;
	iomem_params.index = 0;

	ret = sys_iomem_init(&iomem_params, &nmnicpf->sys_iomem);
	if (ret)
		return ret;

	/* Map the agnic configuration registers */
	ret = sys_iomem_map(nmnicpf->sys_iomem, "agnic_regs", (phys_addr_t *)&nmnicpf->plat_regs.phys_addr,
			&nmnicpf->plat_regs.virt_addr);
	if (ret) {
		sys_iomem_deinit(nmnicpf->sys_iomem);
		return ret;
	}

	pr_debug("agnic regs mapped at virt:%p phys:%p\n",
		nmnicpf->plat_regs.virt_addr, nmnicpf->plat_regs.phys_addr);

	return 0;
}

static int nmnicpf_map_pci_bar(struct nmnicpf *nmnicpf)
{
	struct sys_iomem_params iomem_params;
	int ret;

	pr_debug("Mapping function %s\n", PCI_EP_UIO_MEM_NAME);

	iomem_params.type = SYS_IOMEM_T_UIO;
	iomem_params.devname = PCI_EP_UIO_MEM_NAME;
	iomem_params.index = 0;

	ret = sys_iomem_init(&iomem_params, &nmnicpf->sys_iomem);
	if (ret)
		return ret;

	/* Map the whole physical Packet Processor physical address */
	ret = sys_iomem_map(nmnicpf->sys_iomem,
			    PCI_EP_UIO_REGION_NAME,
			    (phys_addr_t *)&nmnicpf->map.cfg_map.phys_addr,
			    &nmnicpf->map.cfg_map.virt_addr);
	if (ret) {
		sys_iomem_deinit(nmnicpf->sys_iomem);
		return ret;
	}

	pr_debug("BAR-0 of %s mapped at virt:%p phys:%p\n", PCI_EP_UIO_MEM_NAME,
		nmnicpf->map.cfg_map.virt_addr, nmnicpf->map.cfg_map.phys_addr);

	/* Map the whole physical Packet Processor physical address */
	ret = sys_iomem_map(nmnicpf->sys_iomem, "host-map", (phys_addr_t *)&nmnicpf->map.host_map.phys_addr,
			&nmnicpf->map.host_map.virt_addr);
	if (ret) {
		sys_iomem_deinit(nmnicpf->sys_iomem);
		return ret;
	}

	pr_debug("host RAM of %s remapped to phys %p virt %p\n", "host-map",
		   nmnicpf->map.host_map.phys_addr, nmnicpf->map.host_map.virt_addr);

	return 0;
}

static int nmnicpf_map_bar(struct nmnicpf *nmnicpf)
{
	if (nmnicpf->map.type == ft_plat)
		return nmnicpf_map_emul_pci_bar(nmnicpf);
	else
		return nmnicpf_map_pci_bar(nmnicpf);
}

static int nmnicpf_map_init(struct nmnicpf *nmnicpf)
{
	int ret;

	/* Map NMP registers if any */
	/* First, try to map the platform device, if failed, and PCI is supported
	 * try the pci device.
	 */
	nmnicpf->map.type = ft_plat;
	ret = nmnicpf_map_plat_func(nmnicpf);
	if (ret) {
		if (nmnicpf->profile_data.pci_en) {
			pr_debug("Platform device not found, trying the pci device.\n");
			nmnicpf->map.type = ft_pcie_ep;
		} else {
			pr_err("platform device not found\n");
			return ret;
		}
	}

	ret = nmnicpf_map_bar(nmnicpf);
	if (ret) {
		pr_err("niether platform nor PCI devices were found\n");
		return ret;
	}

	return 0;
}

/**
 * NIC PF Termination Section
 * ==========================
 */

/*
 *	nmnicpf_local_queue_terminate
 *
 *	This function terminate NIC PF local queue based on configuration profile
 *	Local queues include data queues and bm queues
 *
 *	@param[in]	nmnicpf - pointer to NIC PF object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
static int nmnicpf_local_queue_terminate(struct nmnicpf *nmnicpf)
{
	nmnicpf = nmnicpf;

	return 0;
}

/*
 *	nmnicpf_mng_chn_terminate
 *
 *	This function terminate NIC PF management channel
 *	Execution requires handshake with Host side
 *
 *	@param[in]	nmnicpf - pointer to NIC PF object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
static int nmnicpf_mng_chn_terminate(struct nmnicpf *nmnicpf)
{
	giu_mng_ch_deinit(nmnicpf->giu_mng_ch);
	return 0;
}

static void nmnicpf_unmap_plat_func(struct nmnicpf *nmnicpf)
{
	mv_sys_dma_mem_free(nmnicpf->map.cfg_map.virt_addr);
}

static void nmnicpf_unmap_emul_pci_bar(struct nmnicpf *nmnicpf)
{
	sys_iomem_unmap(nmnicpf->sys_iomem, "agnic_regs");
	sys_iomem_deinit(nmnicpf->sys_iomem);

	nmnicpf->f_put_bar_cb(nmnicpf->arg, nmnicpf->plat_bar_indx);
}

static void nmnicpf_unmap_pci_bar(struct nmnicpf *nmnicpf)
{
	sys_iomem_unmap(nmnicpf->sys_iomem, "host-map");
	sys_iomem_deinit(nmnicpf->sys_iomem);
}

static void nmnicpf_unmap_bar(struct nmnicpf *nmnicpf)
{
	if (nmnicpf->map.type == ft_plat)
		nmnicpf_unmap_emul_pci_bar(nmnicpf);
	else
		nmnicpf_unmap_pci_bar(nmnicpf);
}

static int nmnicpf_map_terminate(struct nmnicpf *nmnicpf)
{
	if (nmnicpf->map.type == ft_plat)
		nmnicpf_unmap_plat_func(nmnicpf);

	nmnicpf_unmap_bar(nmnicpf);

	return 0;
}

/**
 * NIC PF Command Processing Section
 * =================================
 */

/*
 *	nmnicpf_pf_init_command
 */
static int nmnicpf_pf_init_command(struct nmnicpf *nmnicpf,
				  struct mgmt_cmd_params *params,
				  struct mgmt_cmd_resp *resp_data)
{
	int ret, i;
	struct giu_gpio_params *gpio_p = &(nmnicpf->gpio_params);

	pr_debug("PF INIT\n");
	pr_debug("Num of - Ing TC %d, Eg TC %d\n",
				params->pf_init.num_host_ingress_tc,
				params->pf_init.num_host_egress_tc);

	/* Extract message params and update database */
	gpio_p->num_outtcs = params->pf_init.num_host_ingress_tc;
	gpio_p->num_intcs = params->pf_init.num_host_egress_tc;

	/* Initialize remote queues database */
	ret = nmnicpf_topology_remote_queue_init(nmnicpf);
	if (ret) {
		pr_err("Failed to update remote DB queue info\n");
		return ret;
	}

	/**
	 * NIC PF - PP2 updates
	 */
	/* Update pp2 number of TC's */
	for (i = 0; i < nmnicpf->pp2.num_ports; i++)
		nmnicpf->pp2.ports_desc[i].num_tcs = params->pf_init.num_host_ingress_tc;

	/* Initialize local queues database */
	ret = nmnicpf_topology_local_queue_init(nmnicpf);
	if (ret) {
		pr_err("Failed to update local DB queue info\n");
		return ret;
	}
	/* Allocate and configure local queues in the database */
	ret = nmnicpf_topology_local_queue_cfg(nmnicpf);
	if (ret) {
		pr_err("Failed to configure PF regfile\n");
		return ret;
	}

	pr_debug("PF INIT, Done\n\n");

	return 0;
}

/*
 *	nmnicpf_egress_tc_add_command
 */
static int nmnicpf_egress_tc_add_command(struct nmnicpf *nmnicpf,
					struct mgmt_cmd_params *params,
					struct mgmt_cmd_resp *resp_data)
{
	pr_debug("Configure Host Egress TC[%d] Queues\n", params->pf_egress_tc_add.tc_prio);

	/* Update queue topology database */
	nmnicpf->gpio_params.intcs_params[params->pf_egress_tc_add.tc_prio].num_rem_outqs =
		params->pf_egress_tc_add.num_queues_per_tc;
	/* TODO: Add support for egress pkt-offset and RSS-type */
	nmnicpf->gpio_params.intcs_params[params->pf_egress_tc_add.tc_prio].pkt_offset = 0;
	nmnicpf->gpio_params.intcs_params[params->pf_egress_tc_add.tc_prio].rss_type = RSS_HASH_NONE;

	return 0;
}

/*
 *	nmnicpf_ingress_tc_add_command
 */
static int nmnicpf_ingress_tc_add_command(struct nmnicpf *nmnicpf,
					 struct mgmt_cmd_params *params,
					 struct mgmt_cmd_resp *resp_data)
{
	pr_debug("Configure Host Ingress TC[%d] Queues\n", params->pf_ingress_tc_add.tc_prio);

	/* Update queue topology database */
	nmnicpf->gpio_params.outtcs_params[params->pf_ingress_tc_add.tc_prio].num_rem_inqs =
		params->pf_ingress_tc_add.num_queues_per_tc;
	nmnicpf->gpio_params.outtcs_params[params->pf_ingress_tc_add.tc_prio].rem_rss_type =
		params->pf_ingress_tc_add.hash_type;
	nmnicpf->gpio_params.outtcs_params[params->pf_ingress_tc_add.tc_prio].rem_pkt_offset =
		params->pf_ingress_tc_add.pkt_offset;

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
 *	nmnicpf_ingress_queue_add_command
 */
static int nmnicpf_ingress_queue_add_command(struct nmnicpf *nmnicpf,
					    struct mgmt_cmd_params *params,
					    struct mgmt_cmd_resp *resp_data)
{
	struct giu_gpio_rem_q_params giu_gpio_q;
	struct giu_gpio_params *gpio_p = &(nmnicpf->gpio_params);
	struct giu_gpio_outtc_params *outtc;
	s32 active_q_id;
	u32 msg_tc;

	msg_tc = params->pf_ingress_data_q_add.tc_prio;
	outtc = &(gpio_p->outtcs_params[msg_tc]);

	pr_debug("Host Ingress TC[%d], queue Add (num of queues %d)\n", msg_tc, outtc->num_rem_inqs);

	if (nmnicpf->profile_data.lcl_bm_buf_size < params->pf_ingress_data_q_add.q_buf_size) {
		pr_err("Host BM buffer size should be at most %d\n", nmnicpf->profile_data.lcl_bm_buf_size);
		return -1;
	}

	/* Clear queue structure */
	memset(&giu_gpio_q, 0, sizeof(struct giu_gpio_rem_q_params));

	/* Init queue parameters */
	giu_gpio_q.len          = params->pf_ingress_data_q_add.q_len;
	giu_gpio_q.size         = giu_get_desc_size(nmnicpf->giu, GIU_DESC_OUT);
	giu_gpio_q.q_base_pa    = (phys_addr_t)params->pf_ingress_data_q_add.q_phys_addr;
	giu_gpio_q.prod_base_pa =
		(phys_addr_t)(uintptr_t)(params->pf_ingress_data_q_add.q_prod_offs +
					nmnicpf->map.cfg_map.phys_addr);
	giu_gpio_q.prod_base_va =
		(void *)(params->pf_ingress_data_q_add.q_prod_offs + nmnicpf->map.cfg_map.virt_addr);
	giu_gpio_q.cons_base_pa =
		(phys_addr_t)(uintptr_t)(params->pf_ingress_data_q_add.q_cons_offs +
					nmnicpf->map.cfg_map.phys_addr);
	giu_gpio_q.cons_base_va =
		(void *)(params->pf_ingress_data_q_add.q_cons_offs + nmnicpf->map.cfg_map.virt_addr);
	giu_gpio_q.host_remap   = nmnicpf->map.host_map.phys_addr;
	giu_gpio_q.msix_id      = params->pf_ingress_data_q_add.msix_id;

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
	giu_gpio_q.len	      = params->pf_ingress_data_q_add.q_len;
	giu_gpio_q.size	      = giu_get_desc_size(nmnicpf->giu, GIU_DESC_BUFF);
	giu_gpio_q.q_base_pa    = (phys_addr_t)params->pf_ingress_data_q_add.bpool_q_phys_addr;
	giu_gpio_q.prod_base_pa =
		(phys_addr_t)(uintptr_t)(params->pf_ingress_data_q_add.bpool_q_prod_offs +
					nmnicpf->map.cfg_map.phys_addr);
	giu_gpio_q.prod_base_va =
		(void *)(params->pf_ingress_data_q_add.bpool_q_prod_offs + nmnicpf->map.cfg_map.virt_addr);
	giu_gpio_q.cons_base_pa =
		(phys_addr_t)(uintptr_t)(params->pf_ingress_data_q_add.bpool_q_cons_offs +
					nmnicpf->map.cfg_map.phys_addr);
	giu_gpio_q.cons_base_va =
		(void *)(params->pf_ingress_data_q_add.bpool_q_cons_offs + nmnicpf->map.cfg_map.virt_addr);
	giu_gpio_q.host_remap   = nmnicpf->map.host_map.phys_addr;
	giu_gpio_q.buff_len = params->pf_ingress_data_q_add.q_buf_size;

	memcpy(&(outtc->rem_inqs_params[active_q_id].poolq_params),
		&(giu_gpio_q),
		sizeof(struct giu_gpio_rem_q_params));

	/* Set queue Id in response message in case of success */
	resp_data->q_add_resp.bpool_q_inf = Q_INF_STATUS_OK;

	return 0;
}


/*
 *	nmnicpf_egress_queue_add_command
 */
static int nmnicpf_egress_queue_add_command(struct nmnicpf *nmnicpf,
					   struct mgmt_cmd_params *params,
					   struct mgmt_cmd_resp *resp_data)
{
	struct giu_gpio_rem_q_params giu_gpio_q;
	struct giu_gpio_params *gpio_p = &(nmnicpf->gpio_params);
	struct giu_gpio_intc_params *intc;
	s32 active_q_id;
	u32 msg_tc;

	msg_tc = params->pf_egress_q_add.tc_prio;
	intc = &(gpio_p->intcs_params[msg_tc]);

	pr_debug("Host Egress TC[%d], queue Add (num of queues %d)\n", msg_tc, intc->num_rem_outqs);

	/* Clear queue structure */
	memset(&giu_gpio_q, 0, sizeof(struct giu_gpio_rem_q_params));

	/* Init queue parameters */
	giu_gpio_q.len          = params->pf_egress_q_add.q_len;
	giu_gpio_q.size         = giu_get_desc_size(nmnicpf->giu, GIU_DESC_IN);
	giu_gpio_q.q_base_pa    = (phys_addr_t)params->pf_egress_q_add.q_phys_addr;
	giu_gpio_q.prod_base_pa =
		(phys_addr_t)(uintptr_t)(params->pf_egress_q_add.q_prod_offs +
					nmnicpf->map.cfg_map.phys_addr);
	giu_gpio_q.prod_base_va =
		(void *)(params->pf_egress_q_add.q_prod_offs + nmnicpf->map.cfg_map.virt_addr);
	giu_gpio_q.cons_base_pa =
		(phys_addr_t)(uintptr_t)(params->pf_egress_q_add.q_cons_offs +
					nmnicpf->map.cfg_map.phys_addr);
	giu_gpio_q.cons_base_va =
		(void *)(params->pf_egress_q_add.q_cons_offs + nmnicpf->map.cfg_map.virt_addr);
	giu_gpio_q.host_remap   = nmnicpf->map.host_map.phys_addr;
	giu_gpio_q.msix_id      = params->pf_egress_q_add.msix_id;

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

static int nmnicpf_notif_link_change(struct nmnicpf *nmnicpf, int link_status)
{
	struct nmdisp_msg msg;
	struct mgmt_notification resp;
	int ret;

	msg.ext = 1;
	msg.code = NC_PF_LINK_CHANGE;
	msg.indx = CMD_ID_NOTIFICATION;
	msg.dst_client = CDT_PF;
	msg.dst_id = 0;
	msg.src_client = CDT_PF;
	msg.src_id = 0;
	msg.msg = &resp;
	msg.msg_len = sizeof(struct mgmt_notification);

	resp.link_status = link_status;

	ret = nmdisp_send_msg(nmnicpf->nmdisp, 0, &msg);
	if (ret) {
		pr_err("failed to send link-status notification message\n");
		return ret;
	}

	pr_debug("Link status notification was sent (cmd-code :%d).\n", NC_PF_LINK_CHANGE);

	return 0;
}

/*
 *	nmnicpf_link_state_get
 *
 *	This function checks the link state
 *
 *	@param[in]	nmnicpf - pointer to NIC PF object
 *	@param[out]	link_state - the link status
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
static int nmnicpf_link_state_get(struct nmnicpf *nmnicpf, int *link_state)
{
	struct nmp_pp2_port_desc *pdesc;
	u32 pcount = 0;
	int err;

	if (!nmnicpf->pp2.ports_desc)
		/* no pp2, just return */
		/* TODO: handle guest mode (i.e. notify guest to
		 *	 to handle the request)
		 */
		return -ENODEV;

	pdesc = (struct nmp_pp2_port_desc *)&nmnicpf->pp2.ports_desc[pcount];
	if (!pdesc->ppio)
		/* PPIO is not initialized (yet), just return */
		return -ENODEV;

	/* check PP2 link */
	err = pp2_ppio_get_link_state(pdesc->ppio, link_state);
	if (err) {
		pr_err("Link check error (pp_id: %d)\n", pdesc->pp_id);
		return -EFAULT;
	}

	return 0;
}

/*
 *	nmnicpf_link_state_check_n_notif
 *
 *	This function checks the link state and if it was changed
 *	since the last time it was checked it notifies the host
 *
 *	@param[in]	nmnicpf - pointer to NIC PF object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
static int nmnicpf_link_state_check_n_notif(struct nmnicpf *nmnicpf)
{
	struct nmp_pp2_port_desc *pdesc;
	u32 pcount = 0;
	int link_state, err;

	err = nmnicpf_link_state_get(nmnicpf, &link_state);
	if (err == -ENODEV)
		/* no pp2, just return */
		return 0;
	else if (err)
		return err;

	pdesc = (struct nmp_pp2_port_desc *)&nmnicpf->pp2.ports_desc[pcount];
	if (pdesc->link_state != link_state) {
		/* If link state was changed since last check, notify the host */
		pr_debug("Link state was change to %d\n", link_state);

		nmnicpf_notif_link_change(nmnicpf, link_state);
		pdesc->link_state = link_state;
	}

	return 0;
}

/*
 *	nmnicpf_accumulate_statistics
 *
 *	This function reads the PP2 statistics
 *
 *	@param[in]	nmnicpf - pointer to NIC PF object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
static int nmnicpf_accumulate_statistics(struct nmnicpf *nmnicpf)
{
	struct pp2_ppio_statistics stats;

	return nmnicpf_pp2_accumulate_statistics(nmnicpf, &stats, 0/*no reset*/);
}

/*
 *	nmnicpf_pf_init_done_command
 */
static int nmnicpf_pf_init_done_command(struct nmnicpf *nmnicpf,
					struct mgmt_cmd_params *params,
					struct mgmt_cmd_resp *resp_data)
{
	struct giu_gpio_intc_params	*intc;
	struct giu_bpool_params		 bp_params;
	char				 name[20];
	u8				 tc_id, bm_idx;
	int				 giu_id = 0; /* Support only a single GIU */
	int				 ret;

	bp_params.giu = nmnicpf->giu;
	bp_params.mqa = nmnicpf->mqa;

	/* we assume all in-TCs share the same BPools */
	intc = &(nmnicpf->gpio_params.intcs_params[0]);

	for (bm_idx = 0; bm_idx < intc->num_inpools; bm_idx++) {
		int		 bpool_id = giu_bpool_alloc();

		if (bpool_id < 0) {
			pr_err("Failed to alloc giu bpool\n");
			return ret;
		}
		/* Create bpool match string */
		memset(name, 0, sizeof(name));
		snprintf(name, sizeof(name), "giu_pool-%d:%d", giu_id, bpool_id);
		bp_params.match = name;
		bp_params.num_buffs = nmnicpf->profile_data.lcl_bm_q_size;
		bp_params.buff_len = nmnicpf->profile_data.lcl_bm_q_size;
		ret = giu_bpool_init(&bp_params, &(nmnicpf->giu_bpools[bm_idx]));
		if (ret) {
			pr_err("Failed to init giu bpool\n");
			return ret;
		}
	}

	/* update the information needed for gpio giu-qs */
	for (tc_id = 0; tc_id < (nmnicpf->gpio_params.num_intcs); tc_id++) {
		intc = &(nmnicpf->gpio_params.intcs_params[tc_id]);

		for (bm_idx = 0; bm_idx < intc->num_inpools; bm_idx++)
			intc->pools[bm_idx] = nmnicpf->giu_bpools[bm_idx];
	}

	/* Create GPIO match string */
	memset(name, 0, sizeof(name));
	snprintf(name, sizeof(name), "gpio-%d:%d", giu_id, nmnicpf->pf_id);
	nmnicpf->gpio_params.match = name;
	ret = giu_gpio_init(&(nmnicpf->gpio_params), &(nmnicpf->giu_gpio));
	if (ret) {
		pr_err("Failed to init giu gpio\n");
		return ret;
	}

	if (nmnicpf->profile_data.port_type == NMP_LF_NICPF_T_PP2_PORT) {
		ret = nmnicpf_pp2_init_bpools(nmnicpf);
		if (ret) {
			pr_err("nmnicpf_pp2_init_bpools failed\n");
			return ret;
		}

		ret = nmnicpf_pp2_init_ppio(nmnicpf);
		if (ret) {
			pr_err("nmnicpf_pp2_init_ppio failed\n");
			return ret;
		}
	}

	/* Indicate nmp init_done ready */
	nmnicpf->f_ready_cb(nmnicpf->arg, nmnicpf->nmlf.id);

	nmnicpf->initialized = 1;

	return ret;
}

/*
 *	nmnicpf_mgmt_echo_command
 */
static int nmnicpf_mgmt_echo_command(struct nmnicpf *nmnicpf,
				    struct mgmt_cmd_params *params,
				    struct mgmt_cmd_resp *resp_data)
{
	pr_debug("Management echo message.\n");

	nmnicpf = nmnicpf;

	return 0;
}

/*
 *	nmnicpf_link_status_command
 */
static int nmnicpf_link_status_command(struct nmnicpf *nmnicpf,
				      struct mgmt_cmd_params *params,
				      struct mgmt_cmd_resp *resp_data)
{
	int link_state, err;

	pr_debug("Link status message\n");

	err = nmnicpf_link_state_get(nmnicpf, &link_state);
	if (err) {
		pr_err("Link check error (%d)\n", err);
		return err;
	}

	resp_data->link_status = link_state;

	return 0;
}

/*
 *	nmnicpf_link_info_get
 */
static int nmnicpf_link_info_get(struct nmnicpf *nmnicpf,
				 struct mgmt_cmd_params *params,
				 struct mgmt_cmd_resp *resp_data)
{
	struct pp2_ppio_link_info link_info;
	struct nmp_pp2_port_desc *pdesc;
	u32 pcount = 0;
	int err;

	pr_debug("Link info message\n");

	if (!nmnicpf->pp2.ports_desc)
		/* no pp2, just return */
		/* TODO: handle guest mode (i.e. notify guest to
		 *	 to handle the request)
		 */
		return -ENODEV;

	pdesc = (struct nmp_pp2_port_desc *)&nmnicpf->pp2.ports_desc[pcount];
	if (!pdesc->ppio)
		/* PPIO is not initialized (yet), just return */
		return -ENODEV;


	err = pp2_ppio_get_link_info(pdesc->ppio, &link_info);
	if (err) {
		pr_err("Link info get error (%d)\n", err);
		return err;
	}

	resp_data->link_info.link_up  = link_info.up;
	resp_data->link_info.speed    = link_info.speed;
	resp_data->link_info.duplex   = link_info.duplex;
	resp_data->link_info.phy_mode = link_info.phy_mode;

	return 0;
}

/*
 *	nmnicpf_close_command
 */
static int nmnicpf_close_command(struct nmnicpf *nmnicpf,
				struct mgmt_cmd_params *params,
				struct mgmt_cmd_resp *resp_data)
{
	u8 bm_idx;
	int ret;

	pr_debug("Close message.\n");
	pr_debug("Closing PF data path resources\n");

	/* Close stages:
	 * 1) NMP should disable the PP2 and GIU disable
	 * 2) Inform the guest app about if down (it should remove GIU and PP2)
	 * 3) Wait till guest app completes the operation (serialized file is deleted)
	 * 4) De-init PP2 and GIU
	 * 5) Free resources
	 *
	 * Note: only stage 5 implemented below.
	 * TODO: implement other stages
	 */

	/* Free Data Qs and Un-register in MQA/GIU */
	pr_debug("Free Data Qs\n");
	giu_gpio_deinit(nmnicpf->giu_gpio);

	/* Free BPools and Un-register in MQA/GIU */
	pr_debug("Free BM Qs\n");
	/* we assume all in-TCs share the same BPools */
	for (bm_idx = 0;
		bm_idx < nmnicpf->gpio_params.intcs_params[0].num_inpools;
		bm_idx++)
		giu_bpool_deinit(nmnicpf->giu_bpools[bm_idx]);

	/*Free DB TCs */
	pr_debug("Free DB structures\n");
	ret = nmnicpf_topology_tc_free(nmnicpf);
	if (ret)
		pr_err("Failed to free DB resources\n");

	return 0;
}

/*
 *	nmnicpf_mac_addr_command
 */
static int nmnicpf_mac_addr_command(struct nmnicpf *nmnicpf,
				    struct mgmt_cmd_params *params,
				    struct mgmt_cmd_resp *resp_data)
{
	struct nmdisp_msg nmdisp_msg;
	int ret = 0;

	pr_debug("Set mac address message\n");

	if (!nmnicpf->pp2.ports_desc && !nmnicpf->guest_id)
		return -ENOTSUP;

	if (nmnicpf->pp2.ports_desc) {
		ret = pp2_ppio_set_mac_addr(nmnicpf->pp2.ports_desc[0].ppio, params->mac_addr);
		if (ret) {
			pr_err("Unable to set mac address\n");
			return ret;
		}
	}

	if (nmnicpf->guest_id) {
		/* Notify Guest on mac-address change */
		nmdisp_msg.ext = 0;
		nmdisp_msg.dst_client = CDT_CUSTOM;
		nmdisp_msg.dst_id = nmnicpf->guest_id;
		nmdisp_msg.src_client = CDT_PF;
		nmdisp_msg.src_id = nmnicpf->pf_id;
		nmdisp_msg.indx = CMD_ID_NOTIFICATION;
		nmdisp_msg.code = MSG_T_GUEST_MAC_ADDR_UPDATED;
		nmdisp_msg.msg = params->mac_addr;
		nmdisp_msg.msg_len = MAC_ADDR_LEN;
		ret = nmdisp_send_msg(nmnicpf->nmdisp, 0, &nmdisp_msg);
		if (ret)
			pr_err("failed to send mac-addr-updated notification message\n");
		if (nmnicpf->pp2.ports_desc)
			ret = 0; /* currently if ppio exist the 'ret' is ignored. */
	}

	return ret;
}

/*
 *	nmnicpf_mtu_command
 */
static int nmnicpf_mtu_command(struct nmnicpf *nmnicpf,
				    struct mgmt_cmd_params *params,
				    struct mgmt_cmd_resp *resp_data)
{
	struct nmdisp_msg nmdisp_msg;
	u16 new_mtu, orig_mtu;
	int ret = 0;

	pr_debug("Set mtu message\n");

	if (!nmnicpf->pp2.ports_desc && !nmnicpf->guest_id)
		return -ENOTSUP;

	if (nmnicpf->pp2.ports_desc) {
		pp2_ppio_get_mtu(nmnicpf->pp2.ports_desc[0].ppio, &orig_mtu);
		new_mtu = params->pf_set_mtu.mtu;
		if (orig_mtu == new_mtu)
			return ret;

		ret = pp2_ppio_set_mtu(nmnicpf->pp2.ports_desc[0].ppio, new_mtu);
		if (ret) {
			pr_err("Unable to set mtu\n");
			return ret;
		}

		ret = pp2_ppio_set_mru(nmnicpf->pp2.ports_desc[0].ppio, MV_MTU_TO_MRU(new_mtu));
		if (ret) {
			/* restore previous mtu value */
			pp2_ppio_set_mtu(nmnicpf->pp2.ports_desc[0].ppio, orig_mtu);
			pr_err("Unable to set mru\n");
			return ret;
		}
	}

	if (nmnicpf->guest_id) {
		/* Notify Guest on mtu change */
		nmdisp_msg.ext = 0;
		nmdisp_msg.dst_client = CDT_CUSTOM;
		nmdisp_msg.dst_id = nmnicpf->guest_id;
		nmdisp_msg.src_client = CDT_PF;
		nmdisp_msg.src_id = nmnicpf->pf_id;
		nmdisp_msg.indx = CMD_ID_NOTIFICATION;
		nmdisp_msg.code = MSG_T_GUEST_MTU_UPDATED;
		nmdisp_msg.msg  = &params->pf_set_mtu;
		nmdisp_msg.msg_len = sizeof(params->pf_set_mtu);

		ret = nmdisp_send_msg(nmnicpf->nmdisp, 0, &nmdisp_msg);
		if (ret)
			pr_err("failed to send mtu set message\n");
		if (nmnicpf->pp2.ports_desc)
			ret = 0; /* currently if ppio exist the 'ret' is ignored. */
	}

	return ret;
}

/*
 *	nmnicpf_rx_promisc_command
 */
static int nmnicpf_rx_promisc_command(struct nmnicpf *nmnicpf,
				   struct mgmt_cmd_params *params,
				   struct mgmt_cmd_resp *resp_data)
{
	int ret = 0;

	pr_debug("Set promisc message %d\n", params->promisc);

	if ((params->promisc != AGNIC_PROMISC_ENABLE) &&
	    (params->promisc != AGNIC_PROMISC_DISABLE)) {
		pr_err("Unable to set promisc\n");
		return -EINVAL;
	}

	if (!nmnicpf->pp2.ports_desc && !nmnicpf->guest_id)
		return -ENOTSUP;

	if (nmnicpf->pp2.ports_desc) {
		ret = pp2_ppio_set_promisc(nmnicpf->pp2.ports_desc[0].ppio, params->promisc);
		if (ret) {
			pr_err("Unable to set promisc\n");
			return -EFAULT;
		}
	}

	if (nmnicpf->guest_id) {
		/* TODO - Notify Guest on promisc change */
		if (nmnicpf->pp2.ports_desc)
			ret = 0; /* currently if ppio exist the 'ret' is ignored. */
	}

	return ret;
}

/*
 *	nmnicpf_rx_mc_promisc_command
 */
static int nmnicpf_rx_mc_promisc_command(struct nmnicpf *nmnicpf,
				   struct mgmt_cmd_params *params,
				   struct mgmt_cmd_resp *resp_data)
{
	int ret = 0;

	pr_debug("Set mc promisc message %d\n", params->mc_promisc);

	if ((params->promisc != AGNIC_MC_PROMISC_ENABLE) &&
	    (params->promisc != AGNIC_MC_PROMISC_DISABLE)) {
		pr_err("Unable to set mc promisc\n");
		return -EINVAL;
	}

	if (!nmnicpf->pp2.ports_desc && !nmnicpf->guest_id)
		return -ENOTSUP;

	if (nmnicpf->pp2.ports_desc) {
		ret = pp2_ppio_set_mc_promisc(nmnicpf->pp2.ports_desc[0].ppio, params->mc_promisc);
		if (ret) {
			pr_err("Unable to set mc promisc\n");
			return -EFAULT;
		}
	}

	if (nmnicpf->guest_id) {
		/* TODO - Notify Guest on mc promisc change */
		if (nmnicpf->pp2.ports_desc)
			ret = 0; /* currently if ppio exist the 'ret' is ignored. */
	}

	return ret;
}

static int nmnicpf_enable_command(struct nmnicpf *nmnicpf)
{
	int ret = 0;

	pr_debug("Set enable message\n");

	if (!nmnicpf->pp2.ports_desc && !nmnicpf->guest_id)
		return -ENOTSUP;

	if (nmnicpf->pp2.ports_desc) {
		ret = pp2_ppio_enable(nmnicpf->pp2.ports_desc[0].ppio);
		if (ret) {
			pr_err("PPIO enable failed\n");
			return ret;
		}
	}

	if (nmnicpf->guest_id) {
		if (nmnicpf->pp2.ports_desc)
			ret = 0; /* currently if ppio exist the 'ret' is ignored. */
		else
			/* TODO - Notify Guest on enable */
			nmnicpf_notif_link_change(nmnicpf, (ret == 0) ? 1 : 0);
	}

	return ret;
}

static int nmnicpf_disable_command(struct nmnicpf *nmnicpf)
{
	int ret = 0;

	pr_debug("Set disable message\n");

	if (!nmnicpf->pp2.ports_desc && !nmnicpf->guest_id)
		return -ENOTSUP;

	if (nmnicpf->pp2.ports_desc) {
		ret = pp2_ppio_disable(nmnicpf->pp2.ports_desc[0].ppio);
		if (ret) {
			pr_err("PPIO disable failed\n");
			return ret;
		}
	}

	if (nmnicpf->guest_id) {
		/* TODO - Notify Guest on disable */
		if (nmnicpf->pp2.ports_desc)
			ret = 0; /* currently if ppio exist the 'ret' is ignored. */
	}

	return ret;
}

/*
 *	nmnicpf_loopback_command
 */
static int nmnicpf_loopback_command(struct nmnicpf *nmnicpf,
				struct mgmt_cmd_params *params,
				struct mgmt_cmd_resp *resp_data)
{
	int ret = 0;

	pr_debug("Set loopback message\n");

	if (!nmnicpf->pp2.ports_desc && !nmnicpf->guest_id)
		return -ENOTSUP;

	if (nmnicpf->pp2.ports_desc) {
		ret = pp2_ppio_set_loopback(nmnicpf->pp2.ports_desc[0].ppio, params->pf_set_loopback.loopback);
		if (ret) {
			pr_err("Unable to set loopback\n");
			return ret;
		}
	}

	if (nmnicpf->guest_id) {
		/* TODO - Notify Guest on loppback */
		if (nmnicpf->pp2.ports_desc)
			ret = 0; /* currently if ppio exist the 'ret' is ignored. */
	}

	return ret;
}

/*
 *	nmnicpf_add_vlan_command
 */
static int nmnicpf_add_vlan_command(struct nmnicpf *nmnicpf,
				struct mgmt_cmd_params *params,
				struct mgmt_cmd_resp *resp_data)
{
	int ret = 0;

	pr_debug("Add vlan message\n");
	if (!nmnicpf->pp2.ports_desc && !nmnicpf->guest_id)
		return -ENOTSUP;

	if (nmnicpf->pp2.ports_desc) {
		ret = pp2_ppio_add_vlan(nmnicpf->pp2.ports_desc[0].ppio, params->pf_vlan.vlan);
		if (ret) {
			pr_err("Unable to add vlan\n");
			return ret;
		}
	}

	if (nmnicpf->guest_id) {
		if (nmnicpf->pp2.ports_desc)
			ret = 0;
	}

	return ret;
}

/*
 *	nmnicpf_remove_vlan_command
 */
static int nmnicpf_remove_vlan_command(struct nmnicpf *nmnicpf,
				struct mgmt_cmd_params *params,
				struct mgmt_cmd_resp *resp_data)
{
	int ret = 0;

	pr_debug("Remove vlan message\n");
	if (!nmnicpf->pp2.ports_desc && !nmnicpf->guest_id)
		return -ENOTSUP;

	if (nmnicpf->pp2.ports_desc) {
		ret = pp2_ppio_remove_vlan(nmnicpf->pp2.ports_desc[0].ppio, params->pf_vlan.vlan);
		if (ret) {
			pr_err("Unable to remove vlan\n");
			return ret;
		}
	}

	if (nmnicpf->guest_id) {
		if (nmnicpf->pp2.ports_desc)
			ret = 0;
	}

	return ret;
}

/*
 *	nmnicpf_gp_get_statistics
 */
static int nmnicpf_gp_get_statistics(struct nmnicpf *nmnicpf,
					struct mgmt_cmd_params *params,
					struct mgmt_cmd_resp *resp_data)
{
	struct giu_gpio_statistics stats;
	int ret;

	ret = giu_gpio_get_statistics(nmnicpf->giu_gpio, &stats, 0);
	if (ret)
		return ret;

	resp_data->gp_stats.gp_rx_packets = stats.out_packets;
	resp_data->gp_stats.gp_tx_packets = stats.in_packets;

	/* Calc gp rx full dropped packets using pp2 counter */
	if (nmnicpf->stats.rx_packets)
		resp_data->gp_stats.gp_rx_fullq_dropped = nmnicpf->stats.rx_packets -
								 stats.out_packets;
	else
		resp_data->gp_stats.gp_rx_fullq_dropped = 0;

	return 0;
}

/*
 *	nmnicpf_gp_queue_get_statistics
 */
static int nmnicpf_gp_queue_get_statistics(struct nmnicpf *nmnicpf,
					struct mgmt_cmd_params *params,
					struct mgmt_cmd_resp *resp_data)
{
	int ret;
	struct giu_gpio_q_statistics stats;

	ret = giu_gpio_get_q_statistics(nmnicpf->giu_gpio,
					params->pf_q_get_statistics.out,
					1,
					params->pf_q_get_statistics.tc,
					params->pf_q_get_statistics.qid, &stats,
					params->pf_q_get_statistics.reset);

	if (ret)
		return ret;

	resp_data->gp_queue_stats.packets = stats.packets;

	return 0;
}

/*
 *	nmnicpf_add_mc_addr_command
 */
static int nmnicpf_add_mc_addr_command(struct nmnicpf *nmnicpf,
				   struct mgmt_cmd_params *params,
				   struct mgmt_cmd_resp *resp_data)
{
	int ret = 0;

	pr_debug("Add mc address message\n");
	if (!nmnicpf->pp2.ports_desc && !nmnicpf->guest_id)
		return -ENOTSUP;

	if (nmnicpf->pp2.ports_desc) {
		ret = pp2_ppio_add_mac_addr(nmnicpf->pp2.ports_desc[0].ppio, params->mac_addr);
		if (ret) {
			pr_err("Unable to add mc address\n");
			return -EFAULT;
		}
	}

	if (nmnicpf->guest_id) {
		/* TODO - Notify Guest on mc addr add */
		if (nmnicpf->pp2.ports_desc)
			ret = 0; /* currently if ppio exist the 'ret' is ignored. */
	}

	return ret;
}

/*
 *	nmnicpf_remove_mc_addr_command
 */
static int nmnicpf_remove_mc_addr_command(struct nmnicpf *nmnicpf,
				   struct mgmt_cmd_params *params,
				   struct mgmt_cmd_resp *resp_data)
{
	int ret = 0;

	pr_debug("Remove mc address message\n");
	if (!nmnicpf->pp2.ports_desc && !nmnicpf->guest_id)
		return -ENOTSUP;

	if (nmnicpf->pp2.ports_desc) {
		ret = pp2_ppio_remove_mac_addr(nmnicpf->pp2.ports_desc[0].ppio, params->mac_addr);
		if (ret) {
			pr_err("Unable to remove mc address\n");
			return -EFAULT;
		}
	}

	if (nmnicpf->guest_id) {
		/* TODO - Notify Guest on mc addr remove */
		if (nmnicpf->pp2.ports_desc)
			ret = 0; /* currently if ppio exist the 'ret' is ignored. */
	}

	return ret;
}

/*
 *	nmnicpf_flush_mc_command
 */
static int nmnicpf_flush_mac_command(struct nmnicpf *nmnicpf,
				   struct mgmt_cmd_params *params,
				   struct mgmt_cmd_resp *resp_data)
{
	int ret = 0;

	pr_debug("Flush mac address message\n");
	if (!nmnicpf->pp2.ports_desc && !nmnicpf->guest_id)
		return -ENOTSUP;

	if (nmnicpf->pp2.ports_desc) {
		ret = pp2_ppio_flush_mac_addrs(nmnicpf->pp2.ports_desc[0].ppio,
					       params->pf_flush_addr.uc,
					       params->pf_flush_addr.mc);
		if (ret) {
			pr_err("Unable to flush mac address list\n");
			return -EFAULT;
		}
	}

	if (nmnicpf->guest_id) {
		/* TODO - Notify Guest on mac addr flush */
		if (nmnicpf->pp2.ports_desc)
			ret = 0; /* currently if ppio exist the 'ret' is ignored. */
	}

	return ret;
}

/*
 *	nmnicpf_pause_set
 */
static int nmnicpf_pause_set_command(struct nmnicpf *nmnicpf,
				     struct mgmt_cmd_params *params,
				     struct mgmt_cmd_resp *resp_data)
{
	int ret = 0;

	pr_debug("Pause set message\n");
	if (!nmnicpf->pp2.ports_desc && !nmnicpf->guest_id)
		return -ENOTSUP;

	if (nmnicpf->pp2.ports_desc) {
		struct pp2_ppio_tx_pause_params tx_params;

		tx_params.en = params->pause_params.tx;
		tx_params.use_tc_pause_inqs = 0;

		/* Set Tx Pause */
		ret = pp2_ppio_set_tx_pause(nmnicpf->pp2.ports_desc[0].ppio, &tx_params);
		if (ret) {
			pr_err("Failed to set tx pause status\n");
			return -EFAULT;
		}

		/* Set Rx Pause */
		ret = pp2_ppio_set_rx_pause(nmnicpf->pp2.ports_desc[0].ppio, params->pause_params.rx);
		if (ret) {
			pr_err("Failed to set rx pause status\n");
			return -EFAULT;
		}
	}

	if (nmnicpf->guest_id) {
		/* TODO - Notify Guest on mac addr flush */
		if (nmnicpf->pp2.ports_desc)
			ret = 0; /* currently if ppio exist the 'ret' is ignored. */
	}

	return ret;

}
/*
 *	nmnicpf_pause_get
 */
static int nmnicpf_pause_get_command(struct nmnicpf *nmnicpf,
				     struct mgmt_cmd_params *params,
				     struct mgmt_cmd_resp *resp_data)
{
	int ret = 0;
	int enable;

	pr_debug("Pause get message\n");
	if (!nmnicpf->pp2.ports_desc && !nmnicpf->guest_id)
		return -ENOTSUP;

	if (nmnicpf->pp2.ports_desc) {
		/* Get Tx Pause status */
		ret = pp2_ppio_get_tx_pause(nmnicpf->pp2.ports_desc[0].ppio, &enable);
		if (ret) {
			pr_err("Failed to get tx pause status\n");
			return -EFAULT;
		}
		resp_data->pause_params.tx = enable;

		/* Get Rx Pause status */
		ret = pp2_ppio_get_rx_pause(nmnicpf->pp2.ports_desc[0].ppio, &enable);
		if (ret) {
			pr_err("Failed to get rx pause status\n");
			return -EFAULT;
		}
		resp_data->pause_params.rx = enable;
	}

	if (nmnicpf->guest_id) {
		/* TODO - Notify Guest on mac addr flush */
		if (nmnicpf->pp2.ports_desc)
			ret = 0; /* currently if ppio exist the 'ret' is ignored. */
	}

	return ret;

}

/*
 *	nmnicpf_port_rate_limit_command
 */
static int nmnicpf_port_rate_limit_command(struct nmnicpf *nmnicpf,
				   struct mgmt_cmd_params *params,
				   struct mgmt_cmd_resp *resp_data)
{
	int i;
	struct nmp_pp2_port_desc *port_desc;

	pr_debug("port rate limit message\n");

	if (!nmnicpf->pp2.ports_desc && !nmnicpf->guest_id)
		return -ENOTSUP;

	if (nmnicpf->initialized) {
		pr_err("Can't set port rate limit while network hw is alreay initialized\n");
		return -EFAULT;
	}
	if (nmnicpf->pp2.ports_desc) {
		/* Update pp2 rate limit attributes */
		for (i = 0; i < nmnicpf->pp2.num_ports; i++) {
			port_desc = &nmnicpf->pp2.ports_desc[i];

			if (params->pf_port_rate_limit.enable) {
				port_desc->rate_limit_enable = true;
				port_desc->rate_limit_params.cbs = params->pf_port_rate_limit.rate_limit.cbs;
				port_desc->rate_limit_params.cir = params->pf_port_rate_limit.rate_limit.cir;
			} else
				port_desc->rate_limit_enable = false;
		}
	}

	return 0;
}

/*
 *	nmnicpf_queue_rate_limit_command
 */
static int nmnicpf_queue_rate_limit_command(struct nmnicpf *nmnicpf,
				   struct mgmt_cmd_params *params,
				   struct mgmt_cmd_resp *resp_data)
{
	int i, qid;
	struct nmp_pp2_outq_desc *q_desc;

	if (nmnicpf->profile_data.port_type == NMP_LF_NICPF_T_PP2_PORT &&
		params->pf_queue_rate_limit.qid) {
		pr_err("queue id %d doesn't supported\n", params->pf_queue_rate_limit.qid);
		return -EFAULT;
	}

	pr_debug("queue rate limit message\n");
	if (!nmnicpf->pp2.ports_desc && !nmnicpf->guest_id)
		return -ENOTSUP;
	if (nmnicpf->initialized) {
		pr_err("Can't set queue rate limit while network hw is alreay initialized\n");
		return -EFAULT;
	}
	if (nmnicpf->pp2.ports_desc) {
		/* Update pp2 rate limit attributes */
		for (i = 0; i < nmnicpf->pp2.num_ports; i++) {
			qid = params->pf_queue_rate_limit.tc;
			q_desc = &nmnicpf->pp2.ports_desc[i].q_desc[qid];
			q_desc->rate_limit_enable = params->pf_queue_rate_limit.enable;
			if (q_desc->rate_limit_enable)
				q_desc->rate_limit.cir = params->pf_queue_rate_limit.rate_limit.cir;
		}
	}

	return 0;
}

/*
 *	nmnicpf_process_pf_command
 *
 *	This function process all PF's commands
 *
 *	@param[in]	nmnicpf - pointer to NIC PF object
 *	@param[in]	msg - pointer to nmdisp_msg object
 *	@param[out]	resp_data - pointer to mgmt_cmd_resp object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */

static int nmnicpf_process_pf_command(struct nmnicpf *nmnicpf,
				      struct nmdisp_msg *msg,
				      struct mgmt_cmd_resp *resp_data)
{
	struct mgmt_cmd_params *cmd_params = msg->msg;
	int ret = 0;

	switch (msg->code) {

	case CC_PF_ENABLE:
		ret = nmnicpf_enable_command(nmnicpf);
		if (ret)
			pr_err("CC_PF_ENABLE message failed\n");
		break;

	case CC_PF_DISABLE:
		ret = nmnicpf_disable_command(nmnicpf);
		if (ret)
			pr_err("CC_PF_DISABLE message failed\n");
		break;

	case CC_PF_INIT:
		ret = nmnicpf_pf_init_command(nmnicpf, cmd_params, resp_data);
		if (ret)
			pr_err("PF_INIT message failed\n");
		break;

	case CC_PF_EGRESS_TC_ADD:
		ret = nmnicpf_egress_tc_add_command(nmnicpf, cmd_params, resp_data);
		if (ret)
			pr_err("PF_EGRESS_TC_ADD message failed\n");
		break;

	case CC_PF_EGRESS_DATA_Q_ADD:
		ret = nmnicpf_egress_queue_add_command(nmnicpf, cmd_params, resp_data);
		if (ret)
			pr_err("PF_EGRESS_DATA_Q_ADD message failed\n");
		break;

	case CC_PF_INGRESS_TC_ADD:
		ret = nmnicpf_ingress_tc_add_command(nmnicpf, cmd_params, resp_data);
		if (ret)
			pr_err("PF_INGRESS_TC_ADD message failed\n");
		break;

	case CC_PF_INGRESS_DATA_Q_ADD:
		ret = nmnicpf_ingress_queue_add_command(nmnicpf, cmd_params, resp_data);
		if (ret)
			pr_err("PF_INGRESS_DATA_Q_ADD message failed\n");
		break;

	case CC_PF_INIT_DONE:
		ret = nmnicpf_pf_init_done_command(nmnicpf, cmd_params, resp_data);
		if (ret)
			pr_err("CC_PF_INIT_DONE message failed\n");
		break;

	case CC_PF_MGMT_ECHO:
		ret = nmnicpf_mgmt_echo_command(nmnicpf, cmd_params, resp_data);
		if (ret)
			pr_err("PF_MGMT_ECHO message failed\n");
		break;

	case CC_PF_LINK_STATUS:
		ret = nmnicpf_link_status_command(nmnicpf, cmd_params, resp_data);
		if (ret)
			pr_err("PF_LINK_STATUS message failed\n");
		break;

	case CC_PF_GET_STATISTICS:
		ret = nmnicpf_pp2_get_statistics(nmnicpf, cmd_params, resp_data);
		if (ret)
			pr_err("PF_GET_STATISTICS message failed\n");
		break;

	case CC_PF_CLOSE:
		ret = nmnicpf_close_command(nmnicpf, cmd_params, resp_data);
		if (ret)
			pr_err("PF_IF_DOWN message failed\n");
		break;

	case CC_PF_MAC_ADDR:
		ret = nmnicpf_mac_addr_command(nmnicpf, cmd_params, resp_data);
		if (ret)
			pr_err("PF_IF_DOWN message failed\n");
		break;

	case CC_PF_PROMISC:
		ret = nmnicpf_rx_promisc_command(nmnicpf, cmd_params, resp_data);
		if (ret)
			pr_err("CC_PF_PROMISC message failed\n");
		break;

	case CC_PF_MC_PROMISC:
		ret = nmnicpf_rx_mc_promisc_command(nmnicpf, cmd_params, resp_data);
		if (ret)
			pr_err("CC_PF_MC_PROMISC message failed\n");
		break;

	case CC_PF_MTU:
		ret = nmnicpf_mtu_command(nmnicpf, cmd_params, resp_data);
		if (ret)
			pr_err("PF_IF_DOWN message failed\n");
		break;

	case CC_PF_LOOPBACK:
		ret = nmnicpf_loopback_command(nmnicpf, cmd_params, resp_data);
		if (ret)
			pr_err("PF_LOOPBACK message failed\n");
		break;

	case CC_PF_ADD_VLAN:
		ret = nmnicpf_add_vlan_command(nmnicpf, cmd_params, resp_data);
		if (ret)
			pr_err("PF_ADD_VLAN message failed\n");
		break;

	case CC_PF_REMOVE_VLAN:
		ret = nmnicpf_remove_vlan_command(nmnicpf, cmd_params, resp_data);
		if (ret)
			pr_err("PF_REMOVE_VLAN message failed\n");
		break;

	case CC_PF_GET_GP_STATS:
		ret = nmnicpf_gp_get_statistics(nmnicpf, cmd_params, resp_data);
		if (ret)
			pr_err("CC_PF_GET_GP_STATS message failed\n");
		break;

	case CC_PF_GET_GP_QUEUE_STATS:
		ret = nmnicpf_gp_queue_get_statistics(nmnicpf, cmd_params, resp_data);
		if (ret)
			pr_err("CC_PF_GET_GP_QUEUE_STATS message failed\n");
		break;

	case CC_PF_MC_ADD_ADDR:
		ret = nmnicpf_add_mc_addr_command(nmnicpf, cmd_params, resp_data);
		if (ret)
			pr_err("CC_PF_MC_ADD_ADDR message failed\n");
		break;

	case CC_PF_MC_REMOVE_ADDR:
		ret = nmnicpf_remove_mc_addr_command(nmnicpf, cmd_params, resp_data);
		if (ret)
			pr_err("CC_PF_MC_REMOVE_ADDR message failed\n");
		break;

	case CC_PF_MAC_FLUSH:
		ret = nmnicpf_flush_mac_command(nmnicpf, cmd_params, resp_data);
		if (ret)
			pr_err("CC_PF_MAC_FLUSH message failed\n");
		break;

	case CC_PF_LINK_INFO:
		ret = nmnicpf_link_info_get(nmnicpf, cmd_params, resp_data);
		if (ret)
			pr_err("CC_PF_LINK_INFO message failed\n");
		break;

	case CC_PF_PAUSE_SET:
		ret = nmnicpf_pause_set_command(nmnicpf, cmd_params, resp_data);
		if (ret)
			pr_err("CC_PF_PAUSE_SET message failed\n");
		break;

	case CC_PF_PAUSE_GET:
		ret = nmnicpf_pause_get_command(nmnicpf, cmd_params, resp_data);
		if (ret)
			pr_err("CC_PF_PAUSE_GET message failed\n");
		break;

	case CC_PF_PORT_RATE_LIMIT:
		ret = nmnicpf_port_rate_limit_command(nmnicpf, cmd_params, resp_data);
		if (ret)
			pr_err("CC_PF_PORT_RATE_LIMIT message failed\n");
		break;

	case CC_PF_QUEUE_RATE_LIMIT:
		ret = nmnicpf_queue_rate_limit_command(nmnicpf, cmd_params, resp_data);
		if (ret)
			pr_err("CC_PF_QUEUE_RATE_LIMIT message failed\n");
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
 *	nmnicpf_process_guest_command
 *
 *	This function process guest commnads
 *
 *	@param[in]	nmnicpf - pointer to NIC PF object
 *	@param[in]	msg - pointer to nmdisp_msg object
 *
 */
static void nmnicpf_process_guest_command(struct nmnicpf *nmnicpf,
					 struct nmdisp_msg *msg)
{
	struct guest_cmd_resp resp;
	int ret = 0;

	switch (msg->code) {

	case MSG_F_GUEST_TABLE_INIT:
		ret = nmnicpf_pp2_cls_table_init(nmnicpf, msg->msg, msg->msg_len, &resp.pp2_cls_resp);
		if (ret)
			pr_err("MSG_F_GUEST_TABLE_INIT message failed\n");
		break;

	case MSG_F_GUEST_TABLE_DEINIT:
		ret = nmnicpf_pp2_cls_table_deinit(nmnicpf, msg->msg, msg->msg_len);
		if (ret)
			pr_err("MSG_F_GUEST_TABLE_DEINIT message failed\n");
		break;

	case MSG_F_GUEST_ADD_RULE:
		ret = nmnicpf_pp2_cls_rule_add(nmnicpf, msg->msg, msg->msg_len);
		if (ret)
			pr_err("MSG_F_GUEST_ADD_RULE message failed\n");
		break;

	case MSG_F_GUEST_MODIFY_RULE:
		ret = nmnicpf_pp2_cls_rule_modify(nmnicpf, msg->msg, msg->msg_len);
		if (ret)
			pr_err("MSG_F_GUEST_MODIFY_RULE message failed\n");
		break;

	case MSG_F_GUEST_REMOVE_RULE:
		ret = nmnicpf_pp2_cls_rule_remove(nmnicpf, msg->msg, msg->msg_len);
		if (ret)
			pr_err("MSG_F_GUEST_REMOVE_RULE message failed\n");
		break;

	case MSG_F_GUEST_KA:
		nmnicpf->profile_data.guest_ka_recv = 1;
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
		msg->dst_id = nmnicpf->guest_id;
		msg->src_client = CDT_PF;
		msg->src_id = nmnicpf->pf_id;
		msg->msg = &resp;
		msg->msg_len = sizeof(resp);
		ret = nmdisp_send_msg(nmnicpf->nmdisp, 0, msg);
		if (ret)
			pr_err("failed to send response message\n");
	}
}

/*
 *	nmnicpf_process_command
 *
 *	This function process all PF execution flows
 *
 *	@param[in]	nmnicpf - pointer to NIC PF object
 *	@param[in]	msg - pointer to nmdisp_msg object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
static int nmnicpf_process_command(void *arg, struct nmdisp_msg *msg)
{
	struct mgmt_cmd_resp resp_data;
	struct nmnicpf *nmnicpf = (struct nmnicpf *)arg;
	int ret, send_msg = 1;

/*
 *	Once NIC-PF get a external command from dispatcher, it shall first check the 'src-client/ID' (should be its own)
 *	and then it will authenticate the message by validating the 'code' (against the available codes). In that stage,
 *	the NIC-PF shall execute the command.
 *	Optionally, the NIC-PF will return a response to the caller by sending a response message by calling the
 *	'nmdisp_send_msg' API that is implemented as part of the dispatcher. It shall use its own 'src-client/ID' and
 *	its own 'dst-client/ID' and 'ext' set.
 *
 *	For some commands, a guest may be registered as a 'listener' (e.g. MTU change). Therefore, after executing the
 *	command, the NIC-PF shall iterate all registered 'listeners' for this specific command and initiate a call with
 *	the command message by calling the 'nmdisp_send_msg' API with 'src-client/id' of the NIC-PF and 'dst-client/id'
 *	of the Custom ('ext' should not be set).
 *
 *	Once NIC-PF get a internal command and the 'src-client' is of type Custom, it should initiate a call with
 *	the command message by calling the 'nmdisp_send_msg' API and 'ext' set.
 */

	pr_debug("NICPF got %s command code %d from client-type %d client-id %d which %s response\n",
		 (msg->ext) ? "external":"internal", msg->code, msg->src_client, msg->src_id,
		 (msg->resp_required) ? "requires" : "doesn't requires");

	if (msg->ext) {
		if ((msg->src_client == CDT_PF) && (msg->src_id == nmnicpf->pf_id)) {
			ret = nmnicpf_process_pf_command(nmnicpf, msg, &resp_data);
			send_msg = msg->resp_required;
			if (ret)
				resp_data.status = NOTIF_STATUS_FAIL;
			else
				resp_data.status = NOTIF_STATUS_OK;

			msg->msg = (void *)&resp_data;
			msg->msg_len = sizeof(resp_data);
		} else if ((msg->src_client == CDT_CUSTOM) && (msg->src_id == nmnicpf->guest_id)) {
			nmnicpf_process_guest_command(nmnicpf, msg);
			return 0;
		} else {
			pr_err("Src client %d not supported for external command\n", msg->src_client);
			return -1;
		}
	} else {
		if (msg->src_client == CDT_CUSTOM) {
			msg->ext = 1;
			pr_debug("PF-Lf got %s command code %d from client-type %d client-id %d msg: 0x%x\n",
				(msg->ext) ? "external":"internal", msg->code, msg->src_client, msg->src_id,
				*(u32 *)msg->msg);
		} else {
			pr_err("Src client %d not supported for internal command\n", msg->src_client);
			return -1;
		}
	}

	if (send_msg) {
		ret = nmdisp_send_msg(nmnicpf->nmdisp, 0, msg);
		if (ret) {
			pr_err("failed to send response message\n");
			return ret;
		}
	}

	return 0;
}

static int nmnicpf_keep_alive_process(struct nmnicpf *nmnicpf)
{
	struct nmdisp_msg msg;
	struct mgmt_notification resp;
	int ret;

	if (!nmnicpf->initialized ||
	    !nmnicpf->profile_data.keep_alive_thresh ||
	    (nmnicpf->profile_data.keep_alive_counter++ != nmnicpf->profile_data.keep_alive_thresh))
		return 0;

	/* Send Keep Alive notification message */
	msg.ext = 1;
	msg.code = NC_PF_KEEP_ALIVE;
	msg.indx = CMD_ID_NOTIFICATION;
	msg.dst_client = CDT_PF;
	msg.dst_id = 0;
	msg.src_client = CDT_PF;
	msg.src_id = 0;
	msg.msg = &resp;
	msg.msg_len = sizeof(struct mgmt_notification);

	resp.keep_alive = MGMT_NOTIF_KEEP_ALIVE_FW;
	if (nmnicpf->profile_data.guest_ka_recv)
		resp.keep_alive |= MGMT_NOTIF_KEEP_ALIVE_APP;

	nmnicpf->profile_data.keep_alive_counter = 0;
	nmnicpf->profile_data.guest_ka_recv = 0;

	ret = nmdisp_send_msg(nmnicpf->nmdisp, 0, &msg);
	if (ret) {
		pr_err("failed to send keep-alive notification message\n");
		return ret;
	}

	pr_debug("Keep-alive notification was sent (cmd-code :%d).\n", NC_PF_KEEP_ALIVE);

	return 0;
}

static int nmnicpf_maintenance(struct nmlf *nmlf)
{
	struct nmnicpf *nmnicpf = (struct nmnicpf *)nmlf;
	int err;

	/* Check link state (and notify in case of a change) */
	err = nmnicpf_link_state_check_n_notif(nmnicpf);
	if (err)
		return err;

	/* Read statistics */
	err = nmnicpf_accumulate_statistics(nmnicpf);
	if (err)
		return err;

	/* Send keep-alive notification */
	err = nmnicpf_keep_alive_process(nmnicpf);
	if (err)
		return err;

	return 0;
}


/*
 *	nmnicpf_init
 *
 *	@param[in]	params - pointer to NIC PF parameters
 *	@param[out]	nmnicpf - pointer to the created NIC PF object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int nmnicpf_init(struct nmnicpf_params *params, struct nmnicpf **nmnicpf)
{
	struct nmnicpf			*_nmnicpf;
	struct nmdisp_client_params	 client_params;
	struct nmdisp_q_pair_params	 q_params;
	struct giu_mng_ch_qs		 mng_ch_qs;
	int				 err;

	_nmnicpf = kmalloc(sizeof(*_nmnicpf), GFP_KERNEL);
	if (!_nmnicpf)
		return -ENOMEM;
	memset(_nmnicpf, 0, sizeof(struct nmnicpf));

	_nmnicpf->nmlf.id = params->lf_id;
	_nmnicpf->pf_id = params->id;
	_nmnicpf->guest_id = params->guest_id;

	_nmnicpf->nmdisp = params->nmdisp;
	_nmnicpf->mqa = params->mqa;
	_nmnicpf->giu = params->giu;

	/* Assign the pf_init_done callback */
	_nmnicpf->f_ready_cb = params->f_ready_cb;
	_nmnicpf->f_get_free_bar_cb = params->f_get_free_bar_cb;
	_nmnicpf->f_put_bar_cb = params->f_put_bar_cb;
	_nmnicpf->f_pp_find_free_bpool_cb = params->f_pp_find_free_bpool_cb;
	_nmnicpf->arg = params->arg;

	err = init_nicpf_params(_nmnicpf, params->nmp_nicpf_params);
	if (err)
		return err;

	/* Initialize NIC-PF map */
	err = nmnicpf_map_init(_nmnicpf);
	if (err)
		return err;

	/* Clear queue topology batabase */
	memset(&(_nmnicpf->gpio_params), 0, sizeof(struct giu_gpio_params));

	_nmnicpf->gpio_params.mqa = _nmnicpf->mqa;
	_nmnicpf->gpio_params.giu = _nmnicpf->giu;

	_nmnicpf->nmlf.f_maintenance_cb = nmnicpf_maintenance;
	/* TODO - set this callback once defined correctly.
	 * _nmnicpf->nmlf.f_serialize_cb = nmnicpf_serialize;
	 */
	/* Initialize the nicpf PP2 port */
	err = nmnicpf_pp2_port_init(_nmnicpf);
	if (err)
		return err;

	/* Initialize management queues */
	err = nmnicpf_mng_chn_init(_nmnicpf);
	if (err)
		return err;

	/* Register NIC PF to dispatcher */
	memset(&client_params, 0, sizeof(client_params));
	client_params.client_type	= CDT_PF;
	client_params.client_id		= _nmnicpf->pf_id;
	client_params.f_client_ctrl_cb	= nmnicpf_process_command;
	client_params.client		= _nmnicpf;
	err = nmdisp_register_client(_nmnicpf->nmdisp, &client_params);
	if (err)
		return err;

	giu_mng_ch_get_qs(_nmnicpf->giu_mng_ch, &mng_ch_qs);

	memset(&q_params, 0, sizeof(q_params));
	q_params.cmd_q    = mng_ch_qs.lcl_cmd_q;
	q_params.notify_q = mng_ch_qs.lcl_resp_q;
	q_params.ext_desc_support = 0;
	q_params.max_msg_size = sizeof(struct mgmt_cmd_params);
	err = nmdisp_add_queue(_nmnicpf->nmdisp, client_params.client_type, client_params.client_id, &q_params);
	if (err)
		return err;

	*nmnicpf = _nmnicpf;
	return 0;
}

/*
 *	nmnicpf_deinit
 *
 *	@param[in]	nmnicpf - pointer to NIC PF object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int nmnicpf_deinit(struct nmnicpf *nmnicpf)
{
	int ret;

	ret = nmnicpf_local_queue_terminate(nmnicpf);
	if (ret)
		return ret;

	ret = nmnicpf_mng_chn_terminate(nmnicpf);
	if (ret)
		return ret;

	/* Un-Map the NIC-PF */
	ret = nmnicpf_map_terminate(nmnicpf);
	if (ret)
		return ret;

	kfree(nmnicpf);

	pr_debug("Terminating NIC PF\n");
	return 0;
}

int nmnicpf_serialize(struct nmnicpf *nmnicpf, char *buff, u32 size)
{
	size_t	 pos = 0;
	u8	 bm_idx, num_pools;
	int	 ret;
	int	 giu_id = 0; /* Support only a single GIU */

	/* build relation-information first */
	json_print_to_buffer(buff, size, 1, "\"relations-info\": {\n");
	/* serialize the relations of the GIU objects */
	json_print_to_buffer(buff, size, 2, "\"giu-gpio\": \"gpio-%d:%d\",\n",
			     giu_id, nmnicpf->pf_id);
	num_pools = nmnicpf->gpio_params.intcs_params[0].num_inpools;
	json_print_to_buffer(buff, size, 2, "\"num_bpools\": %d,\n", num_pools);
	for (bm_idx = 0; bm_idx < num_pools; bm_idx++) {
		if ((bm_idx == num_pools - 1) &&
			(nmnicpf->profile_data.port_type == NMP_LF_NICPF_T_NONE))
			json_print_to_buffer(buff, size, 2, "\"giu-bpool-%d\": \"giu_pool-%d:%d\"\n", bm_idx,
					nmnicpf->giu_bpools[bm_idx]->giu_id, nmnicpf->giu_bpools[bm_idx]->id);
		else
			json_print_to_buffer(buff, size, 2, "\"giu-bpool-%d\": \"giu_pool-%d:%d\",\n", bm_idx,
					nmnicpf->giu_bpools[bm_idx]->giu_id, nmnicpf->giu_bpools[bm_idx]->id);
	}
	/* serialize the relations of the PP objects */
	if (nmnicpf->profile_data.port_type == NMP_LF_NICPF_T_PP2_PORT) {
		ret = nmnicpf_pp2_serialize_relation_inf(nmnicpf, &buff[pos], size - pos);
		if (ret < 0)
			return ret;
		pos += ret;
		if (pos != strlen(buff)) {
			pr_err("found mismatch between pos (%zu) and buff len (%zu)\n", pos, strlen(buff));
			return -EFAULT;
		}
	}
	json_print_to_buffer(buff, size, 1, "},\n");

	if (nmnicpf->profile_data.port_type == NMP_LF_NICPF_T_PP2_PORT) {
		ret = nmnicpf_pp2_serialize(nmnicpf, &buff[pos], size - pos);
		if (ret < 0)
			return ret;
		pos += ret;
		if (pos != strlen(buff)) {
			pr_err("found mismatch between pos (%zu) and buff len (%zu)\n", pos, strlen(buff));
			return -EFAULT;
		}
	}

	ret = nmnicpf_config_topology_and_update_regfile(nmnicpf);
	if (ret) {
		pr_err("Failed to configure PF regfile\n");
		return ret;
	}

	return pos;
}

/* TODO: move this routine to NMP-guest (i.e. should be caled through Qs) */
int nmnicpf_create_scheduling_event(struct nmnicpf *nmnicpf,
	struct nmp_event_params *params,
	struct mv_sys_event **ev)
{
	return giu_gpio_create_event(nmnicpf->giu_gpio,
		(struct giu_gpio_event_params *)params,
		ev);
}

/* TODO: move this routine to NMP-guest (i.e. should be caled through Qs) */
int nmnicpf_delete_scheduling_event(struct mv_sys_event *ev)
{
	return giu_gpio_delete_event(ev);
}

/* TODO: move this routine to NMP-guest (i.e. should be caled through Qs) */
int nmnicpf_set_scheduling_event(struct mv_sys_event *ev, int en)
{
	return giu_gpio_set_event(ev, en);
}
