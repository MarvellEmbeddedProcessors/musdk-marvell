/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#define log_fmt(fmt) "pf: " fmt

#include "std_internal.h"
#include "hw_emul/gie.h"
#include "drivers/mv_mqa.h"
#include "drivers/mv_mqa_queue.h"
#include "drivers/mv_giu_gpio.h"
#include "drivers/mqa_def.h"
#include "mng/lf/mng_cmd_desc.h"
#include "mng/db.h"
#include "mng/mv_nmp.h"
#include "mng/mv_nmp_guest_msg.h"
#include "mng/mv_nmp_dispatch.h"
#include "mng/pci_ep_def.h"
#include "mng/include/guest_mng_cmd_desc.h"
#include "pf_regfile.h"
#include "pf.h"
#include "pf_pp2.h"
#include "pf_topology.h"
#include "pf_pci_if_desc.h"
#include "env/trace/trc_pf.h"
#include "drivers/ppv2/pp2_hw_type.h"

#define REGFILE_VAR_DIR		"/var/"
#define REGFILE_NAME_PREFIX	"nic-pf-"
#define REGFILE_MAX_FILE_NAME	64

/* TODO: These should be removed. The local queue sizes should match the remote
 * management queue sizes, as received during the init sequence.
 */
#define LOCAL_CMD_QUEUE_SIZE	256
#define LOCAL_NOTIFY_QUEUE_SIZE	256

#define REGFILE_VERSION		000002	/* Version Format: XX.XX.XX*/

static int nmnicpf_process_command(void *arg, struct nmdisp_msg *msg);
static int nmnicpf_maintenance(struct nmlf *nmlf);

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
	struct giu_gpio_params *q_top = &(nmnicpf->topology_data);

	/* Main topology structure size */
	size = sizeof(struct giu_regfile);

	/* Add BM Pool size */
	size += (sizeof(struct giu_queue) * q_top->intcs_params.intc_params->num_inpools);

	/* Add Egress TCs size */
	size += (sizeof(struct giu_tc) * (q_top->intcs_params.num_intcs));

	/* Go trough Egress TCs and calc size of queues */
	for (tc_id = 0; tc_id < (q_top->intcs_params.num_intcs); tc_id++) {
		intc = &(q_top->intcs_params.intc_params[tc_id]);
		size += (sizeof(struct giu_queue) * (intc->num_inqs));
	}

	/* Add Ingress TCs size */
	size += (sizeof(struct giu_tc) * (q_top->outtcs_params.num_outtcs));

	/* Go trough Egress TCs and calc size of queues */
	for (tc_id = 0; tc_id < (q_top->outtcs_params.num_outtcs); tc_id++) {
		outtc = &(q_top->outtcs_params.outtc_params[tc_id]);
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

	pr_info("Regfile Parameters (Regfile Name: %s, Ver %06d)\n", file_name, REGFILE_VERSION);

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
 *	get_queue_prod_phys_addr
 *
 *	This function gets the Physical addr (producer) of specific queue from the QNPT table
 *
 *      @param[in]	nmnicpf     - Pointer to the NIC-PF struct which defines the topology
 *			hw_q_id    - The real unique Queue index
 *
 *	@retval	addr on success
 */
static phys_addr_t get_queue_prod_phys_addr(struct nmnicpf *nmnicpf, int hw_q_id)
{
	void *pf_cfg_base; /* pointer to HW */

	/* Get BAR0 Configuration space base address */
	pf_cfg_base = (struct mqa_qnpt_entry *)nmnicpf->map.cfg_map.phys_addr;

	/* Calc Notification table specifi entry  */
	return (phys_addr_t)((pf_cfg_base + PCI_BAR0_MQA_QNPT_BASE) +	(sizeof(struct mqa_qnct_entry) * hw_q_id));
}


/*
 *	get_queue_cons_phys_addr
 *
 *	This function gets the Physical addr (consumer) of specific queue from the QNCT table
 *
 *      @param[in]	nmnicpf     - Pointer to the NIC-PF struct which defines the topology
 *			hw_q_id    - The real unique Queue index
 *
 *	@retval	addr on success
 */
static phys_addr_t get_queue_cons_phys_addr(struct nmnicpf *nmnicpf, int hw_q_id)
{
	void *pf_cfg_base;

	/* Get BAR0 Configuration space base address */
	pf_cfg_base = (struct mqa_qnct_entry *)nmnicpf->map.cfg_map.phys_addr;

	/* Calc Notification table specifi entry  */
	return (phys_addr_t)((pf_cfg_base + PCI_BAR0_MQA_QNCT_BASE) +	(sizeof(struct mqa_qnct_entry) * hw_q_id));
}


/*
 *	nmnicpf_regfile_register_queue
 *
 *	This function register Queue params in Regfile Queue structure
 *	It gets the info from the SNIC-DB and finally update directly the regfile
 *
 *      @param[in]	nmnicpf     - Pointer to the NIC-PF struct which defines the topology
 *			hw_q_id    - The real unique Queue index
 *			q_type     - The type of the Queue (Egress / Ingress / BM/...)
 *			file_map   - Pointer to Pointer to Regfile mempry map
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise (< 0)
 */
static int nmnicpf_regfile_register_queue(struct nmnicpf *nmnicpf,
					  union giu_gpio_q_params *giu_gpio_q_p,
					  int q_type,
					  phys_addr_t qs_phys_base,
					  phys_addr_t ptrs_phys_base,
					  void **file_map)
{
	struct giu_queue reg_giu_queue;
	struct mqa_queue_info queue_info;

	if (giu_gpio_q_p == NULL) {
		pr_err("Failed to get queue params from DB (Queue: %d)\n", (int)giu_gpio_q_p->lcl_q.q_id);
		return -ENODEV;
	}

	mqa_queue_get_info(giu_gpio_q_p->lcl_q.q, &queue_info);

	reg_giu_queue.hw_id		= queue_info.q_id;
	/** TODO - change params naming - change reg_giu_queue.size to reg_giu_queue.len*/
	reg_giu_queue.size		= queue_info.len;
	reg_giu_queue.type		= q_type;
	reg_giu_queue.phy_base_offset	= (phys_addr_t)(queue_info.phy_base_addr - qs_phys_base);
	/* Prod/Cons addr are Virtual. Needs to translate them to offset */
	reg_giu_queue.prod_offset = (phys_addr_t)(queue_info.prod_phys - ptrs_phys_base);
	reg_giu_queue.cons_offset = (phys_addr_t)(queue_info.cons_phys - ptrs_phys_base);

	/* Note: buff_size & payload_offset are union and they are set
	 *	 acoording to the Q type.
	 */
	if (q_type == QUEUE_BP)
		/** TODO - change params naming - change buff_size to buff_len */
		reg_giu_queue.buff_len = nmnicpf->profile_data.lcl_bm_buf_size;
	else
		reg_giu_queue.payload_offset = 0; /* TODO: this should not be hardcoded */

	/* Copy Queues parameters */
	pr_debug("\t\tCopy Queue %d Information to Regfile\n", reg_giu_queue.hw_id);
	memcpy(*file_map, &reg_giu_queue, sizeof(struct giu_queue));
	*file_map += sizeof(struct giu_queue);

	return 0;
}


/*
 *	nmnicpf_regfile_register_intc
 *
 *	This function configures In TC params in register file TC structure
 *
 *      @param[in]	nmnicpf - Pointer to the NIC-PF struct which defines the topology
 *			intc_params    - Pointer to In TC format of the giu gpio queue topology
 *			tc_queue_type  - Type of the Queues in this specific TC
 *			file_map       - Pointer to Pointer to Regfile mempry map
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise (< 0)
 */
static int nmnicpf_regfile_register_intc(struct nmnicpf *nmnicpf,
					 struct giu_gpio_intc_params *intc_params,
					 int tc_queue_type,
					 phys_addr_t qs_phys_base,
					 phys_addr_t ptrs_phys_base,
					 void **file_map)
{
	int queue_idx, ret = 0;
	struct giu_tc reg_giu_tc;

	reg_giu_tc.id			= intc_params->tc_id;
	reg_giu_tc.ingress_rss_type	= 0/*intc_params->rss_type*/;
	reg_giu_tc.num_queues		= intc_params->num_inqs;
	reg_giu_tc.queues		= NULL;
	reg_giu_tc.dest_num_queues	= 0;

	/* Copy TC parameters */
	pr_debug("\tCopy TC %d Information to Regfile\n", reg_giu_tc.id);
	memcpy(*file_map, &reg_giu_tc, sizeof(struct giu_tc));
	*file_map += sizeof(struct giu_tc);

	if (intc_params->inqs_params != NULL) {
		for (queue_idx = 0; queue_idx < reg_giu_tc.num_queues; queue_idx++) {
			union giu_gpio_q_params *hw_q_id = &(intc_params->inqs_params[queue_idx]);

			ret = nmnicpf_regfile_register_queue(nmnicpf,
							     hw_q_id,
							     tc_queue_type,
							     qs_phys_base,
							     ptrs_phys_base,
							     file_map);

			if (ret != 0)
				break;
		}
	} else {
		pr_info("Topology Queue list in TC is empty (NULL)\n");
	}

	return ret;

}


/*
 *	nmnicpf_regfile_register_outtc
 *
 *	This function configures Out TC params in register file TC structure
 *
 *      @param[in]	nmnicpf - Pointer to the NIC-PF struct which defines the topology
 *			outtc_params   - Pointer to Out TC format of the giu gpio queue topology
 *			tc_queue_type  - Type of the Queues in this specific TC
 *			file_map       - Pointer to Pointer to Regfile mempry map
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise (< 0)
 */
static int nmnicpf_regfile_register_outtc(struct nmnicpf *nmnicpf,
					  struct giu_gpio_outtc_params *outtc_params,
					  int tc_queue_type,
					  phys_addr_t qs_phys_base,
					  phys_addr_t ptrs_phys_base,
					  void **file_map)
{
	int queue_idx, ret = 0;
	struct giu_tc reg_giu_tc;

	reg_giu_tc.id			= outtc_params->tc_id;
	reg_giu_tc.ingress_rss_type	= outtc_params->rss_type;
	reg_giu_tc.num_queues		= outtc_params->num_outqs;
	reg_giu_tc.queues		= NULL;
	reg_giu_tc.dest_num_queues	= outtc_params->num_rem_inqs;

	/* Copy TC parameters */
	pr_debug("\tCopy TC %d Information to Regfile\n", reg_giu_tc.id);
	memcpy(*file_map, &reg_giu_tc, sizeof(struct giu_tc));
	*file_map += sizeof(struct giu_tc);

	if (outtc_params->outqs_params != NULL) {
		for (queue_idx = 0; queue_idx < reg_giu_tc.num_queues; queue_idx++) {
			union giu_gpio_q_params *hw_q_id = &(outtc_params->outqs_params[queue_idx]);

			ret = nmnicpf_regfile_register_queue(nmnicpf,
							     hw_q_id,
							     tc_queue_type,
							     qs_phys_base,
							     ptrs_phys_base,
							     file_map);

			if (ret != 0)
				break;
		}
	} else {
		pr_info("Topology Queue list in TC is empty (NULL)\n");
	}

	return ret;

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
	struct giu_regfile *regfile_data = &nmnicpf->regfile_data;
	struct giu_gpio_params *q_top = &(nmnicpf->topology_data);
	int tc_idx, queue_idx;
	int ret = 0;
	int bm_tc_id = 0;
	phys_addr_t qs_phys_base, ptrs_phys_base;
	struct mv_sys_dma_mem_info	 mem_info;
	char				 dev_name[100];

	mem_info.name = dev_name;
	mv_sys_dma_mem_get_info(&mem_info);

	/* Update Regfile general info */
	regfile_data->version		= REGFILE_VERSION;
	regfile_data->num_bm_qs		= q_top->intcs_params.intc_params->num_inpools;
	regfile_data->bm_qs		= NULL;
	regfile_data->num_egress_tcs	= q_top->intcs_params.num_intcs;
	regfile_data->egress_tcs	= NULL;
	regfile_data->num_ingress_tcs	= q_top->outtcs_params.num_outtcs;
	regfile_data->ingress_tcs	= NULL;

	qs_phys_base = mem_info.paddr;
	strcpy(regfile_data->dma_uio_mem_name, mem_info.name);
	ptrs_phys_base = qs_phys_base;

	regfile_data->flags &= ~REGFILE_PCI_MODE;
	if (nmnicpf->map.type == ft_pcie_ep) {
		regfile_data->flags |= REGFILE_PCI_MODE;
		strcpy(regfile_data->pci_uio_mem_name, PCI_EP_UIO_MEM_NAME);
		strcpy(regfile_data->pci_uio_region_name, PCI_EP_UIO_REGION_NAME);
		ptrs_phys_base = (phys_addr_t)nmnicpf->map.cfg_map.phys_addr;
	}

	pr_debug("Start Topology configuration to register file [Regfile ver (%d), NIC-PF number (%d)]\n",
			regfile_data->version, nmnicpf->pf_id);

	ret = nmnicpf_regfile_open(nmnicpf, &file_map);

	if (ret != 0)
		return ret;

	/* Copy Header File parameters */
	pr_debug("Copy Regfile Header\n");
	ret = nmnicpf_config_header_regfile(regfile_data, &file_map);

	if (ret != 0)
		goto config_error;

	/* Setup BM Queues  */
	if (q_top->intcs_params.intc_params[bm_tc_id].pools != NULL) {
		for (queue_idx = 0; queue_idx < regfile_data->num_bm_qs; queue_idx++) {

			union giu_gpio_q_params *hw_q_id =
						&(q_top->intcs_params.intc_params[bm_tc_id].pools[queue_idx]);

			ret = nmnicpf_regfile_register_queue(nmnicpf,
							     hw_q_id,
							     QUEUE_BP,
							     qs_phys_base,
							     ptrs_phys_base,
							     &file_map);

			if (ret != 0)
				goto config_error;

		}
	} else {
		pr_info("Topology BM Queue list is empty (NULL)\n");
	}

	/* Setup Egress TCs  */
	if (q_top->intcs_params.intc_params != NULL) {
		for (tc_idx = 0; tc_idx < regfile_data->num_egress_tcs; tc_idx++) {

			struct giu_gpio_intc_params *intc_params = &(q_top->intcs_params.intc_params[tc_idx]);

			ret = nmnicpf_regfile_register_intc(nmnicpf,
							    intc_params,
							    QUEUE_EGRESS,
							    qs_phys_base,
							    ptrs_phys_base,
							    &file_map);

			if (ret != 0)
				goto config_error;

		}
	} else {
		pr_info("Egress TCs Topology is empty (NULL)\n");
	}


	/* Setup Ingress TCs  */
	if (q_top->outtcs_params.outtc_params != NULL) {
		for (tc_idx = 0; tc_idx < regfile_data->num_ingress_tcs; tc_idx++) {

			struct giu_gpio_outtc_params *outtc_params = &(q_top->outtcs_params.outtc_params[tc_idx]);

			ret = nmnicpf_regfile_register_outtc(nmnicpf,
							     outtc_params,
							     QUEUE_INGRESS,
							     qs_phys_base,
							     ptrs_phys_base,
							     &file_map);

			if (ret != 0)
				goto config_error;

		}
	} else {
		pr_info("Ingress TCs Topology is empty (NULL)\n");
	}

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
	struct giu_gpio_params *q_top = &(nmnicpf->topology_data);

	pr_debug("Initializing Local Queues in management Database\n");

	/* Local Egress TC */
	ret = pf_intc_queue_init(LCL, q_top->intcs_params.num_intcs, prof->lcl_egress_q_num);
	if (ret) {
		pr_err("Failed to allocate Local Egress TC table\n");
		goto queue_error;
	}

	/* Local ingress TC */
	ret = pf_outtc_queue_init(LCL, q_top->outtcs_params.num_outtcs, prof->lcl_ingress_q_num);
	if (ret) {
		pr_err("Failed to allocate Local Ingress TC table\n");
		goto queue_error;
	}

	/* Local BM */
	ret = pf_intc_bm_queue_init(prof->lcl_bm_q_num);
	if (ret) {
		pr_err("Failed to allocate Local BM table\n");
		goto queue_error;
	}

	return 0;

queue_error:

	pf_intc_queue_free(LCL, q_top->intcs_params.num_intcs);
	pf_outtc_queue_free(LCL, q_top->outtcs_params.num_outtcs);
	pf_intc_bm_queue_free();

	return -ENOMEM;
}


/*
 *	nmnicpf_topology_local_tc_free
 *
 *	This function frees the local TC resources in topology
 */
static void nmnicpf_topology_local_tc_free(struct nmnicpf *nmnicpf)
{
	struct giu_gpio_params *q_top = &(nmnicpf->topology_data);

	pr_debug("Free Local queues DB\n");
	pf_intc_queue_free(LCL, q_top->intcs_params.num_intcs);
	pf_outtc_queue_free(LCL, q_top->outtcs_params.num_outtcs);
	pf_intc_bm_queue_free();
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
	struct giu_gpio_params *q_top = &(nmnicpf->topology_data);

	pr_debug("Initializing Remote Queues in management Database\n");

	/* Remote Egress TC */
	/* TC queues will be update upon "tc_add" command */
	ret = pf_intc_queue_init(REM, q_top->intcs_params.num_intcs, 0);
	if (ret) {
		pr_err("Failed to allocate Local Egress TC table\n");
		goto queue_error;
	}

	/* Remote ingress TC */
	/* TC queues will be update upon "tc_add" command */
	ret = pf_outtc_queue_init(REM, q_top->outtcs_params.num_outtcs, 0);
	if (ret) {
		pr_err("Failed to allocate Local Ingress TC table\n");
		goto queue_error;
	}

	return 0;

queue_error:

	pr_err("Remote Queues Initialization failed\n");

	pf_intc_queue_free(REM, q_top->intcs_params.num_intcs);
	pf_outtc_queue_free(REM, q_top->outtcs_params.num_outtcs);

	return -ENOMEM;
}


/*
 *	nmnicpf_topology_remote_tc_free
 *
 *	This function frees the remote TC resources in topology
 */
static int nmnicpf_topology_remote_tc_free(struct nmnicpf *nmnicpf)
{
	struct giu_gpio_params *q_top = &(nmnicpf->topology_data);

	pr_debug("Free Remote queues DB\n");
	pf_intc_queue_free(REM, q_top->intcs_params.num_intcs);
	pf_outtc_queue_free(REM, q_top->outtcs_params.num_outtcs);

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
	int ret;
	u32 tc_idx;
	u32 bm_idx;
	u32 q_idx;
	u32 q_id;

	struct giu_gpio_outtc_params *outtc;
	struct giu_gpio_intc_params *intc;
	union  giu_gpio_q_params giu_gpio_q;
	union  giu_gpio_q_params *giu_gpio_q_p;
	struct pf_profile *prof = &(nmnicpf->profile_data);
	struct giu_gpio_params *q_top = &(nmnicpf->topology_data);

	/* Create Local BM queues */
	pr_debug("Configure Local BM queues (Num of queues %d)\n", q_top->intcs_params.num_inpools);

	for (tc_idx = 0; tc_idx < q_top->intcs_params.num_intcs; tc_idx++) {

		intc = &(q_top->intcs_params.intc_params[tc_idx]);

		for (bm_idx = 0; bm_idx < intc->num_inpools; bm_idx++) {

			/* Clear queue structure */
			memset(&giu_gpio_q, 0, sizeof(union giu_gpio_q_params));

			/* Allocate queue from MQA */
			ret = mqa_queue_alloc(nmnicpf->mqa, &q_id);
			if (ret < 0) {
				pr_err("Failed to allocate queue from MQA\n");
				goto lcl_bm_queue_error;
			}

			/* Init queue parameters */
			giu_gpio_q.lcl_q.q_id = q_id;
			giu_gpio_q.lcl_q.len = prof->lcl_bm_q_size;

			/* Save queue info */
			memcpy(&(intc->pools[bm_idx]), &(giu_gpio_q), sizeof(union giu_gpio_q_params));

			pr_debug("Configure BM[%d] TC[%d], Id %d\n\n",
					tc_idx, bm_idx, intc->pools[bm_idx].lcl_q.q_id);
		}

		intc->pool_buf_size = prof->lcl_bm_buf_size;
	}


	/* Create Local Egress TC queues */
	pr_debug("Configure Local Egress TC queues (Num of queues %d)\n", q_top->intcs_params.num_intcs);

	for (tc_idx = 0; tc_idx < q_top->intcs_params.num_intcs; tc_idx++) {

		intc = &(q_top->intcs_params.intc_params[tc_idx]);

		for (q_idx = 0; q_idx < intc->num_inqs; q_idx++) {

			/* Clear queue structure */
			memset(&giu_gpio_q, 0, sizeof(union giu_gpio_q_params));

			/* Allocate queue from MQA */
			ret = mqa_queue_alloc(nmnicpf->mqa, &q_id);
			if (ret < 0) {
				pr_err("Failed to allocate queue from MQA\n");
				goto lcl_eg_queue_error;
			}

			giu_gpio_q.lcl_q.q_id = q_id;
			giu_gpio_q.lcl_q.len  = prof->lcl_egress_q_size;

			/* Save queue info */
			memcpy(&(intc->inqs_params[q_idx]), &(giu_gpio_q), sizeof(union giu_gpio_q_params));

			pr_debug("Configure Egress TC[%d], queue[%d] = Id %d\n\n",
					tc_idx, q_idx, intc->inqs_params[q_idx].lcl_q.q_id);
		}
	}


	/* Create Local Ingress TC queues */
	pr_debug("Configure Local Ingress TC queues (Num of queues %d)\n", q_top->outtcs_params.num_outtcs);

	for (tc_idx = 0; tc_idx < q_top->outtcs_params.num_outtcs; tc_idx++) {

		outtc = &(q_top->outtcs_params.outtc_params[tc_idx]);

		for (q_idx = 0; q_idx < outtc->num_outqs; q_idx++) {

			/* Clear queue structure */
			memset(&giu_gpio_q, 0, sizeof(union giu_gpio_q_params));

			/* Allocate queue from MQA */
			ret = mqa_queue_alloc(nmnicpf->mqa, &q_id);
			if (ret < 0) {
				pr_err("Failed to allocate queue from MQA\n");
				goto lcl_ing_queue_error;
			}

			giu_gpio_q.lcl_q.q_id = q_id;
			giu_gpio_q.lcl_q.len  = prof->lcl_ingress_q_size;

			/* Save queue info */
			memcpy(&(outtc->outqs_params[q_idx]), &(giu_gpio_q), sizeof(union giu_gpio_q_params));

			pr_debug("Configure Ingress TC[%d], queue[%d] = Id %d\n\n",
					tc_idx, q_idx, outtc->outqs_params[q_idx].lcl_q.q_id);
		}
	}

	return 0;

lcl_ing_queue_error:

	for (tc_idx = 0; tc_idx < q_top->outtcs_params.num_outtcs; tc_idx++) {
		outtc = &(q_top->outtcs_params.outtc_params[tc_idx]);

		for (q_idx = 0; q_idx < outtc->num_outqs; q_idx++) {
			giu_gpio_q_p = &(outtc->outqs_params[q_idx]);
			if (giu_gpio_q_p != NULL) {

				ret = mqa_queue_free(nmnicpf->mqa, giu_gpio_q_p->lcl_q.q_id);
				if (ret)
					pr_err("Failed to free queue Idx %x in MQA\n", giu_gpio_q_p->lcl_q.q_id);

				memset(giu_gpio_q_p, 0, sizeof(union giu_gpio_q_params));
			}
		}
	}

lcl_eg_queue_error:

	for (tc_idx = 0; tc_idx < q_top->intcs_params.num_intcs; tc_idx++) {
		intc = &(q_top->intcs_params.intc_params[tc_idx]);

		for (q_idx = 0; q_idx < intc->num_inqs; q_idx++) {
			giu_gpio_q_p = &(intc->inqs_params[q_idx]);
			if (giu_gpio_q_p != NULL) {

				ret = mqa_queue_free(nmnicpf->mqa, (u32)giu_gpio_q_p->lcl_q.q_id);
				if (ret)
					pr_err("Failed to free queue Idx %x in MQA\n", giu_gpio_q_p->lcl_q.q_id);

				memset(giu_gpio_q_p, 0, sizeof(union giu_gpio_q_params));
			}
		}
	}

lcl_bm_queue_error:

	for (bm_idx = 0; bm_idx < q_top->intcs_params.intc_params->num_inpools; bm_idx++) {
		giu_gpio_q_p = &(q_top->intcs_params.intc_params->pools[bm_idx]);
		if (giu_gpio_q_p != NULL) {

			ret = mqa_queue_free(nmnicpf->mqa, (u32)giu_gpio_q_p->lcl_q.q_id);
			if (ret)
				pr_err("Failed to free queue Idx %x in MQA\n", giu_gpio_q_p->lcl_q.q_id);

			memset(giu_gpio_q_p, 0, sizeof(union giu_gpio_q_params));
		}
	}

	return -1;
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
	u64 pf_cfg_phys, pf_cfg_virt; /* pointer to HW so it should be volatile */
	void *qnpt_phys, *qnct_phys;
	u32 local_cmd_queue, local_notify_queue;
	u32 remote_cmd_queue, remote_notify_queue;
	u8 mac_addr[ETH_ALEN];
	int ret = 0;

	struct mqa_queue_params params;
	struct mqa_queue_info queue_info;
	struct mqa_q *lcl_cmd_queue_p    = NULL;
	struct mqa_q *lcl_notify_queue_p = NULL;
	struct mqa_q *rem_cmd_queue_p    = NULL;
	struct mqa_q *rem_notify_queue_p = NULL;

	/*  Create Local Queues */
	/* ==================== */

	/* Allocate and Register Local Command queue in MQA */
	pr_info("Register Local Command Q\n");

	/* Allocate queue from MQA */
	ret = mqa_queue_alloc(nmnicpf->mqa, &local_cmd_queue);
	if (ret < 0) {
		pr_err("Failed to allocate queue from MQA\n");
		goto exit_error;
	}

	memset(&params, 0, sizeof(struct mqa_queue_params));

	params.idx  = local_cmd_queue;
	params.len  = LOCAL_CMD_QUEUE_SIZE;
	params.size = sizeof(struct cmd_desc);
	params.attr = MQA_QUEUE_LOCAL | MQA_QUEUE_EGRESS;
	params.prio = 0;

	ret = mqa_queue_create(nmnicpf->mqa, &(params), &(lcl_cmd_queue_p));
	if (ret < 0) {
		pr_info("Failed to register Host Management Q\n");
		goto exit_error;
	}

	nmnicpf->mng_data.lcl_mng_ctrl.cmd_queue = (struct mqa_q *)lcl_cmd_queue_p;

	/* Allocate and Register Local Notification queue in MQA */
	pr_info("Register Local Notification Q\n");

	/* Allocate queue from MQA */
	ret = mqa_queue_alloc(nmnicpf->mqa, &local_notify_queue);
	if (ret < 0) {
		pr_err("Failed to allocate queue from MQA\n");
		goto exit_error;
	}
	memset(&params, 0, sizeof(struct mqa_queue_params));

	params.idx   = local_notify_queue;
	params.len   = LOCAL_NOTIFY_QUEUE_SIZE;
	params.size  = sizeof(struct cmd_desc);
	params.attr  = MQA_QUEUE_LOCAL | MQA_QUEUE_INGRESS;
	params.prio  = 0;

	ret = mqa_queue_create(nmnicpf->mqa, &(params), &(lcl_notify_queue_p));
	if (ret < 0) {
		pr_info("Failed to register Host Management Q\n");
		goto exit_error;
	}

	nmnicpf->mng_data.lcl_mng_ctrl.notify_queue = (struct mqa_q *)lcl_notify_queue_p;

	/* get the initial mac-addr from the pp2 port's (if exist) kernel side as this is the correct mac-addr. */
	nmnicpf_pp2_get_mac_addr(nmnicpf, mac_addr);

	/*  Host Ready Check */
	/* ================= */

	/* Get BAR0 Configuration space base address */
	pf_cfg_phys = (u64)nmnicpf->map.cfg_map.phys_addr;
	pf_cfg_virt = (u64)nmnicpf->map.cfg_map.virt_addr;
	pcie_cfg    = (void *)(pf_cfg_virt + PCI_BAR0_MNG_CH_BASE);

	/* Calc Notification tables base */
	qnct_phys = (void *)(pf_cfg_phys + PCI_BAR0_MQA_QNCT_BASE);
	qnpt_phys = (void *)(pf_cfg_phys + PCI_BAR0_MQA_QNPT_BASE);

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
	 * Notification Tables Info:
	 * TODO: Move the Mac address and notification tables configuration to
	 * a separate function, as they are not really related to nic-pf
	 * mng-channel creation.
	 */
	pcie_cfg->prod_notif_tbl_offset = PCI_BAR0_MQA_QNPT_BASE;
	pcie_cfg->prod_notif_tbl_size   = PCI_BAR0_MQA_QNPT_SIZE;
	pcie_cfg->cons_notif_tbl_offset = PCI_BAR0_MQA_QNCT_BASE;
	pcie_cfg->cons_notif_tbl_size   = PCI_BAR0_MQA_QNCT_SIZE;

	/*
	 * MSI-X table base/offset.
	 */
	pcie_cfg->msi_x_tbl_offset = PCI_BAR0_MSI_X_TBL_BASE;

	/* Register MSI-X table base in GIE */
	/* TODO: register msix table only if pcie_cfg->msi_x_tbl_offset !=0
	 *       netdev side should reset it if it doesn't support msi
	 *	 hence, it should move after "Host is Ready" (below)
	 */
	gie_register_msix_table(nmnicpf->gie.mng_gie, pf_cfg_virt + pcie_cfg->msi_x_tbl_offset);
	gie_register_msix_table(nmnicpf->gie.rx_gie, pf_cfg_virt + pcie_cfg->msi_x_tbl_offset);
	gie_register_msix_table(nmnicpf->gie.tx_gie, pf_cfg_virt + pcie_cfg->msi_x_tbl_offset);

	/* Make sure that above configuration are out before setting the
	 * dev-ready status for the host side.
	 */
	wmb();

	pcie_cfg->status = PCIE_CFG_STATUS_DEV_READY;

	while (!(pcie_cfg->status & PCIE_CFG_STATUS_HOST_MGMT_READY))
		; /* Do Nothing. Wait till state it's updated */

	/* Set remote index mode to MQA */
	mqa_set_remote_index_mode(nmnicpf->mqa, pcie_cfg->remote_index_location);

	/* Set remote index mode to GIE */
	gie_set_remote_index_mode(pcie_cfg->remote_index_location);

	pr_info("Host is Ready\n");

	/*  Register Remote Queues */
	/* ======================= */

	/* Register Host Command management queue */
	pr_info("Register host command queue\n");

	/* Allocate queue from MQA */
	ret = mqa_queue_alloc(nmnicpf->mqa, &remote_cmd_queue);
	if (ret < 0) {
		pr_err("Failed to allocate queue from MQA\n");
		goto exit_error;
	}

	memset(&params, 0, sizeof(struct mqa_queue_params));

	params.idx             = remote_cmd_queue;
	params.len             = pcie_cfg->cmd_q.len;
	params.size            = sizeof(struct cmd_desc);
	params.attr            = MQA_QUEUE_REMOTE | MQA_QUEUE_EGRESS;
	params.prio            = 0;
	params.remote_phy_addr = (void *)pcie_cfg->cmd_q.q_addr;
	params.cons_phys       = (void *)(pcie_cfg->cmd_q.consumer_idx_addr + nmnicpf->map.host_map.phys_addr);
	params.cons_virt       = (void *)(pcie_cfg->cmd_q.consumer_idx_addr + nmnicpf->map.host_map.virt_addr);
	params.host_remap      = nmnicpf->map.host_map.phys_addr;
	params.peer_id         = local_cmd_queue;

	/* Allocate queue from MQA */
	ret = mqa_queue_create(nmnicpf->mqa, &(params), &(rem_cmd_queue_p));
	if (ret < 0) {
		pr_err("Failed to register Host Management Q\n");
		goto exit_error;
	}

	/* Update PCI BAR0 with producer address (Entry index in notification table) */
	mqa_queue_get_info(rem_cmd_queue_p, &queue_info);
	pcie_cfg->cmd_q.producer_idx_addr = (u64)(queue_info.prod_phys - qnpt_phys) / sizeof(u32);
	if (!pcie_cfg->remote_index_location)
		pcie_cfg->cmd_q.consumer_idx_addr = (u64)(queue_info.cons_phys - qnct_phys) / sizeof(u32);

	nmnicpf->mng_data.host_mng_ctrl.cmd_queue = (struct mqa_q *)rem_cmd_queue_p;


	/* Register Host Notification queue */
	pr_info("Register host notification queue\n");

	/* Allocate queue from MQA */
	ret = mqa_queue_alloc(nmnicpf->mqa, &remote_notify_queue);
	if (ret < 0) {
		pr_err("Failed to allocate queue from MQA\n");
		goto exit_error;
	}

	memset(&params, 0, sizeof(struct mqa_queue_params));

	params.idx             = remote_notify_queue;
	params.len             = pcie_cfg->notif_q.len;
	params.size            = sizeof(struct cmd_desc);
	params.attr            = MQA_QUEUE_REMOTE | MQA_QUEUE_INGRESS;
	params.prio            = 0;
	params.remote_phy_addr = (void *)pcie_cfg->notif_q.q_addr;
	params.prod_phys       = (void *)(pcie_cfg->notif_q.producer_idx_addr + nmnicpf->map.host_map.phys_addr);
	params.prod_virt       = (void *)(pcie_cfg->notif_q.producer_idx_addr + nmnicpf->map.host_map.virt_addr);
	params.host_remap      = nmnicpf->map.host_map.phys_addr;

	ret = mqa_queue_create(nmnicpf->mqa, &(params), &(rem_notify_queue_p));
	if (ret < 0) {
		pr_err("Failed to register Host Management Q\n");
		goto exit_error;
	}

	ret = mqa_queue_associate_pair(nmnicpf->mqa, local_notify_queue, remote_notify_queue);
	if (ret < 0) {
		pr_err("Failed to associate Notification queues (Src %d Dest %d)\n",
				local_notify_queue, remote_notify_queue);
		goto exit_error;
	}

	/* Update PCI BAR0 with consumer address (Entry index in notification table) */
	mqa_queue_get_info(rem_notify_queue_p, &queue_info);
	pcie_cfg->notif_q.consumer_idx_addr = (u64)(queue_info.cons_phys - qnct_phys) / sizeof(u32);
	if (!pcie_cfg->remote_index_location)
		pcie_cfg->notif_q.producer_idx_addr = (u64)(queue_info.prod_phys - qnpt_phys) / sizeof(u32);

	nmnicpf->mng_data.host_mng_ctrl.notify_queue = (struct mqa_q *)rem_notify_queue_p;

	/* Register Qs in GIU */
	/* ================== */

	/* Register Command channel */
	gie_add_queue(nmnicpf->gie.mng_gie, remote_cmd_queue, 1);

	/* Register Notification channel */
	gie_add_queue(nmnicpf->gie.mng_gie, local_notify_queue, 0);


	/* Device Ready */
	/* ============ */

	/* make sure all writes are done before updating the status */
	wmb();

	/* Set state to 'Device Management Ready' */
	pr_info("Set status to 'Device Management Ready'\n");
	pcie_cfg->status |= PCIE_CFG_STATUS_DEV_MGMT_READY;

	return 0;

exit_error:

	if (local_cmd_queue >= 0) {
		if (lcl_cmd_queue_p) {
			ret = mqa_queue_destroy(nmnicpf->mqa, lcl_cmd_queue_p);
			if (ret < 0)
				pr_err("Failed to free Local Cmd Q %d in DB\n", local_cmd_queue);
			kfree(lcl_cmd_queue_p);
		}
		ret = mqa_queue_free(nmnicpf->mqa, local_cmd_queue);
		if (ret < 0)
			pr_err("Failed to free Local Cmd Q %d in MQA\n", local_cmd_queue);
	}

	if (local_notify_queue >= 0) {
		if (lcl_notify_queue_p) {
			ret = mqa_queue_destroy(nmnicpf->mqa, lcl_notify_queue_p);
			if (ret < 0)
				pr_err("Failed to free Local Notify Q %d in DB\n", local_notify_queue);
			kfree(lcl_notify_queue_p);
		}
		ret = mqa_queue_free(nmnicpf->mqa, local_notify_queue);
		if (ret < 0)
			pr_err("Failed to free Local Notify Q %d in MQA\n", local_notify_queue);
	}

	if (remote_cmd_queue >= 0) {
		if (rem_cmd_queue_p) {
			ret = mqa_queue_destroy(nmnicpf->mqa, rem_cmd_queue_p);
			if (ret < 0)
				pr_err("Failed to free remote Cmd Q %d in DB\n", remote_cmd_queue);
			kfree(rem_cmd_queue_p);
		}
		ret = mqa_queue_free(nmnicpf->mqa, remote_cmd_queue);
		if (ret < 0)
			pr_err("Failed to free remote Cmd Q %d in MQA\n", remote_cmd_queue);
	}

	if (remote_notify_queue >= 0) {
		if (rem_notify_queue_p) {
			ret = mqa_queue_destroy(nmnicpf->mqa, rem_notify_queue_p);
			if (ret < 0)
				pr_err("Failed to free Remote Notify Q %d in DB\n", remote_notify_queue);
			kfree(rem_notify_queue_p);
		}
		ret = mqa_queue_free(nmnicpf->mqa, remote_notify_queue);
		if (ret < 0)
			pr_err("Failed to free Remote Notify Q %d in MQA\n", remote_notify_queue);
	}

	return ret;
}

/*
 *	nmnicpf_init
 *
 *	@param[in]	nmnicpf - pointer to NIC PF object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int nmnicpf_init(struct nmnicpf *nmnicpf)
{
	int ret;
	struct nmdisp_client_params params;
	struct nmdisp_q_pair_params q_params;

	pf_topology_init(nmnicpf);

	/* Clear queue topology batabase */
	memset(&(nmnicpf->topology_data), 0, sizeof(struct giu_gpio_params));

	nmnicpf->topology_data.mqa = nmnicpf->mqa;
	nmnicpf->topology_data.gie = &(nmnicpf->gie);

	nmnicpf->pf_id = 0;
	nmnicpf->nmlf.id = 0;
	nmnicpf->nmlf.f_maintenance_cb = nmnicpf_maintenance;
	/* TODO - set this callback once defined correctly.
	 * nmnicpf->nmlf.f_serialize_cb = nmnicpf_serialize;
	 */
	/* Initialize the nicpf PP2 port */
	nmnicpf_pp2_port_init(nmnicpf);

	/* Initialize management queues */
	ret = nmnicpf_mng_chn_init(nmnicpf);
	if (ret)
		return ret;

	/* Register NIC PF to dispatcher */
	params.client_type	= CDT_PF;
	params.client_id	= nmnicpf->pf_id;
	params.f_client_ctrl_cb = nmnicpf_process_command;
	params.client		= nmnicpf;
	ret = nmdisp_register_client(nmnicpf->nmdisp, &params);
	if (ret)
		return ret;

	q_params.cmd_q    = nmnicpf->mng_data.lcl_mng_ctrl.cmd_queue;
	q_params.notify_q = nmnicpf->mng_data.lcl_mng_ctrl.notify_queue;
	q_params.ext_desc_support = 0;
	q_params.max_msg_size = sizeof(struct mgmt_cmd_params);
	ret = nmdisp_add_queue(nmnicpf->nmdisp, params.client_type, params.client_id, &q_params);
	if (ret)
		return ret;

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
	u32 local_cmd_queue, local_notify_queue;
	u32 remote_cmd_queue, remote_notify_queue;
	int ret = 0;

	struct mqa_q *lcl_cmd_queue_p    = nmnicpf->mng_data.lcl_mng_ctrl.cmd_queue;
	struct mqa_q *lcl_notify_queue_p = nmnicpf->mng_data.lcl_mng_ctrl.notify_queue;
	struct mqa_q *rem_cmd_queue_p    = nmnicpf->mng_data.host_mng_ctrl.cmd_queue;
	struct mqa_q *rem_notify_queue_p = nmnicpf->mng_data.host_mng_ctrl.notify_queue;

	if (lcl_cmd_queue_p) {
		mqa_queue_get_id(lcl_cmd_queue_p, &local_cmd_queue);
		ret = mqa_queue_destroy(nmnicpf->mqa, lcl_cmd_queue_p);
		if (ret < 0)
			pr_err("Failed to free Local Cmd Q %d in DB\n", local_cmd_queue);
		ret = mqa_queue_free(nmnicpf->mqa, local_cmd_queue);
		if (ret < 0)
			pr_err("Failed to free Local Cmd Q %d in MQA\n", local_cmd_queue);

		kfree(lcl_cmd_queue_p);
	}

	if (lcl_notify_queue_p) {
		mqa_queue_get_id(lcl_notify_queue_p, &local_notify_queue);
		ret = mqa_queue_destroy(nmnicpf->mqa, lcl_notify_queue_p);
		if (ret < 0)
			pr_err("Failed to free Local Notify Q %d in DB\n", local_notify_queue);
		ret = mqa_queue_free(nmnicpf->mqa, local_notify_queue);
		if (ret < 0)
			pr_err("Failed to free Local Notify Q %d in MQA\n", local_notify_queue);

		kfree(lcl_notify_queue_p);
	}

	if (rem_cmd_queue_p) {
		mqa_queue_get_id(rem_cmd_queue_p, &remote_cmd_queue);
		ret = mqa_queue_destroy(nmnicpf->mqa, rem_cmd_queue_p);
		if (ret < 0)
			pr_err("Failed to free remote Cmd Q %d in DB\n", remote_cmd_queue);
		ret = mqa_queue_free(nmnicpf->mqa, remote_cmd_queue);
		if (ret < 0)
			pr_err("Failed to free remote Cmd Q %d in MQA\n", remote_cmd_queue);

		kfree(rem_cmd_queue_p);
	}

	if (rem_notify_queue_p) {
		mqa_queue_get_id(rem_notify_queue_p, &remote_notify_queue);
		ret = mqa_queue_destroy(nmnicpf->mqa, rem_notify_queue_p);
		if (ret < 0)
			pr_err("Failed to free Remote Notify Q %d in DB\n", remote_notify_queue);
		ret = mqa_queue_free(nmnicpf->mqa, remote_notify_queue);
		if (ret < 0)
			pr_err("Failed to free Remote Notify Q %d in MQA\n", remote_notify_queue);

		kfree(rem_notify_queue_p);
	}

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

	pr_info("Terminating NIC PF\n");
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
	struct giu_gpio_params *q_top = &(nmnicpf->topology_data);

	pr_debug("PF INIT\n");
	pr_debug("Num of - Ing TC %d, Eg TC %d\n",
				params->pf_init.num_host_ingress_tc,
				params->pf_init.num_host_egress_tc);

	/* Extract message params and update database */
	q_top->outtcs_params.num_outtcs = params->pf_init.num_host_ingress_tc;
	q_top->intcs_params.num_intcs = params->pf_init.num_host_egress_tc;

	/* Initialize remote queues database */
	ret = nmnicpf_topology_remote_queue_init(nmnicpf);
	if (ret)
		pr_err("Failed to update remote DB queue info\n");

	/**
	 * NIC PF - PP2 updates
	 */
	/* Update pp2 number of TC's */
	for (i = 0; i < nmnicpf->pp2.num_ports; i++)
		nmnicpf->pp2.ports_desc[i].num_tcs = params->pf_init.num_host_ingress_tc;

#ifndef GIE_NO_MULTI_Q_SUPPORT_FOR_RSS
	/* Initialize local queues database */
	ret = nmnicpf_topology_local_queue_init(nmnicpf);
	if (ret) {
		pr_err("Failed to update local DB queue info\n");
		goto pf_init_exit;
	}

	/* Allocate and configure local queues in the database */
	ret = nmnicpf_topology_local_queue_cfg(nmnicpf);
	if (ret)
		pr_err("Failed to configure PF regfile\n");
pf_init_exit:
#endif

	pr_debug("PF INIT, Done\n\n");

	return ret;
}


/*
 *	nic_pf_egress_tc_add_command
 */
static int nmnicpf_egress_tc_add_command(struct nmnicpf *nmnicpf,
					struct mgmt_cmd_params *params,
					struct mgmt_cmd_resp *resp_data)
{
	int ret = 0;
	union giu_gpio_q_params *tc_queues;

	struct giu_gpio_params *q_top = &(nmnicpf->topology_data);
	struct giu_gpio_intc_params *intc = &(q_top->intcs_params.intc_params[params->pf_egress_tc_add.tc_prio]);

	pr_debug("Configure Host Egress TC[%d] Queues\n", params->pf_egress_tc_add.tc_prio);

	tc_queues = kcalloc(params->pf_egress_tc_add.num_queues_per_tc, sizeof(union giu_gpio_q_params), GFP_KERNEL);
	if (tc_queues == NULL) {
		ret = -ENOMEM;
		goto tc_exit;
	}

	/* Update queue topology database */
	intc->rem_outqs_params = tc_queues;
	intc->num_rem_outqs = params->pf_egress_tc_add.num_queues_per_tc;

tc_exit:

	if (ret) {
		if (tc_queues != NULL)
			kfree(tc_queues);

		pr_err("Host Egress TC[%d] Add failed\n", params->pf_egress_tc_add.tc_prio);
	}

	return ret;
}


/*
 *	nmnicpf_egress_tc_add_command
 */
static int nmnicpf_ingress_tc_add_command(struct nmnicpf *nmnicpf,
					 struct mgmt_cmd_params *params,
					 struct mgmt_cmd_resp *resp_data)
{
	int ret = 0;

	struct giu_gpio_params *q_top = &(nmnicpf->topology_data);
	struct giu_gpio_outtc_params *outtc = &(q_top->outtcs_params.outtc_params[params->pf_ingress_tc_add.tc_prio]);

	pr_debug("Configure Host Ingress TC[%d] Queues\n", params->pf_ingress_tc_add.tc_prio);

	outtc->rem_inqs_params = kcalloc(params->pf_ingress_tc_add.num_queues_per_tc,
					 sizeof(union giu_gpio_q_params),
					 GFP_KERNEL);
	if (outtc->rem_inqs_params == NULL) {
		ret = -ENOMEM;
		goto tc_exit;
	}

	outtc->rem_poolqs_params = kcalloc(params->pf_ingress_tc_add.num_queues_per_tc,
					   sizeof(union giu_gpio_q_params),
					   GFP_KERNEL);
	if (outtc->rem_poolqs_params == NULL) {
		ret = -ENOMEM;
		goto tc_exit;
	}

	/* Update queue topology database */
	outtc->num_rem_inqs = params->pf_ingress_tc_add.num_queues_per_tc;
	outtc->rss_type = params->pf_ingress_tc_add.hash_type;
	/** TODO - Add support for params->pf_ingress_tc_add.pkt_offset */

tc_exit:

	if (ret) {
		if (outtc->rem_inqs_params != NULL)
			kfree(outtc->rem_inqs_params);

		if (outtc->rem_poolqs_params != NULL)
			kfree(outtc->rem_poolqs_params);

		pr_err("Host ingress TC[%d] Add failed\n", params->pf_ingress_tc_add.tc_prio);
	}

	return ret;
}


/*
 *	tc_q_next_entry_get
 *
 *	This function return next free queue index in TC queue array
 *	in case no available index return -1
 */
static int tc_q_next_entry_get(union giu_gpio_q_params *q_id_list, u32 q_num)
{
	u32 q_idx;

	for (q_idx = 0; q_idx < q_num; q_idx++) {
		if (q_id_list[q_idx].rem_q.q_id == 0)
			return q_idx;
	}

	return -1;
}


/*
 *	nmnicpf_ingress_queue_add_command
 */
static int nmnicpf_ingress_queue_add_command(struct nmnicpf *nmnicpf,
					    struct mgmt_cmd_params *params,
					    struct mgmt_cmd_resp *resp_data)
{
	int ret = 0;
	s32 active_q_id;
	u32 msg_tc;
	u32 q_id, bpool_q_id;

	union giu_gpio_q_params giu_gpio_q;
	struct giu_gpio_params *q_top = &(nmnicpf->topology_data);
	struct giu_gpio_outtc_params *outtc;

	msg_tc = params->pf_ingress_data_q_add.tc_prio;
	outtc = &(q_top->outtcs_params.outtc_params[msg_tc]);

	pr_debug("Host Ingress TC[%d], queue Add (num of queues %d)\n", msg_tc, outtc->num_rem_inqs);

	if (nmnicpf->profile_data.lcl_bm_buf_size < params->pf_ingress_data_q_add.q_buf_size) {
		pr_err("Host BM buffer size should be at most %d\n", nmnicpf->profile_data.lcl_bm_buf_size);
		return -1;
	}

	/* Clear queue structure */
	memset(&giu_gpio_q, 0, sizeof(union giu_gpio_q_params));

	/* Allocate queue from MQA */
	ret = mqa_queue_alloc(nmnicpf->mqa, &q_id);
	if (ret < 0) {
		pr_err("Failed to allocate queue from MQA\n");
		return ret;
	}

	/* Init queue parameters */
	giu_gpio_q.rem_q.q_id         = q_id;
	giu_gpio_q.rem_q.len          = params->pf_ingress_data_q_add.q_len;
	giu_gpio_q.rem_q.size         = gie_get_desc_size(RX_DESC);
	giu_gpio_q.rem_q.q_base_pa    = (phys_addr_t)params->pf_ingress_data_q_add.q_phys_addr;
	giu_gpio_q.rem_q.prod_base_pa = (phys_addr_t)(params->pf_ingress_data_q_add.q_prod_phys_addr +
										nmnicpf->map.host_map.phys_addr);
	giu_gpio_q.rem_q.prod_base_va = (void *)(params->pf_ingress_data_q_add.q_prod_phys_addr +
										nmnicpf->map.host_map.virt_addr);
	giu_gpio_q.rem_q.host_remap   = nmnicpf->map.host_map.phys_addr;
	giu_gpio_q.rem_q.msix_id      = params->pf_ingress_data_q_add.msix_id;

	active_q_id = tc_q_next_entry_get(outtc->rem_inqs_params, outtc->num_rem_inqs);
	if (active_q_id < 0) {
		pr_err("Failed to configure queue in Host Ingress TC[%d] queue list\n", msg_tc);
		ret = active_q_id;
		goto ingress_queue_exit;
	}

	pr_debug("Host Ingress TC[%d], queue %d added at index %d\n", msg_tc, q_id, active_q_id);

	memcpy(&(outtc->rem_inqs_params[active_q_id]), &(giu_gpio_q), sizeof(union giu_gpio_q_params));

	/* Set prod/cons address in response.
	 * we use the qid as the prod/cons idx in the notification space
	 * since that is the how CP-125 HW works
	 */
	resp_data->q_add_resp.q_prod_cons_phys_addr = q_id;

	/* Clear queue structure */
	memset(&giu_gpio_q, 0, sizeof(union giu_gpio_q_params));

	/* Allocate queue from MQA */
	ret = mqa_queue_alloc(nmnicpf->mqa, &bpool_q_id);
	if (ret < 0) {
		pr_err("Failed to allocate queue from MQA\n");
		goto ingress_queue_exit;
	}

	/* Init queue parameters */
	giu_gpio_q.rem_q.q_id	      = bpool_q_id;
	giu_gpio_q.rem_q.len	      = params->pf_ingress_data_q_add.q_len;
	giu_gpio_q.rem_q.size	      = params->pf_ingress_data_q_add.q_buf_size;
	giu_gpio_q.rem_q.q_base_pa    = (phys_addr_t)params->pf_ingress_data_q_add.bpool_q_phys_addr;
	giu_gpio_q.rem_q.cons_base_pa = (phys_addr_t)(params->pf_ingress_data_q_add.bpool_q_cons_phys_addr +
									nmnicpf->map.host_map.phys_addr);
	giu_gpio_q.rem_q.cons_base_va = (void *)(params->pf_ingress_data_q_add.bpool_q_cons_phys_addr +
									nmnicpf->map.host_map.virt_addr);
	giu_gpio_q.rem_q.host_remap   = nmnicpf->map.host_map.phys_addr;

	memcpy(&(outtc->rem_poolqs_params[active_q_id]), &(giu_gpio_q), sizeof(union giu_gpio_q_params));

	/* Set queue Id in response message in case of success */
	resp_data->q_add_resp.bpool_q_prod_cons_phys_addr = bpool_q_id;

ingress_queue_exit:

	if (ret < 0) {
		if (q_id > 0) {
			ret = mqa_queue_free(nmnicpf->mqa, q_id);
			if (ret)
				pr_err("Failed to free queue Idx %x in MQA\n", q_id);
		}
		if (bpool_q_id > 0) {
			ret = mqa_queue_free(nmnicpf->mqa, bpool_q_id);
			if (ret)
				pr_err("Failed to free queue Idx %x in MQA\n", bpool_q_id);
		}
		pr_err("Host ingress TC[%d] Add queue failed\n", msg_tc);
	}

	return ret;
}


/*
 *	nmnicpf_egress_queue_add_command
 */
static int nmnicpf_egress_queue_add_command(struct nmnicpf *nmnicpf,
					   struct mgmt_cmd_params *params,
					   struct mgmt_cmd_resp *resp_data)
{
	int ret = 0;
	s32 active_q_id;
	u32 msg_tc;
	u32 q_id;

	union giu_gpio_q_params giu_gpio_q;
	struct giu_gpio_params *q_top = &(nmnicpf->topology_data);
	struct giu_gpio_intc_params *intc;

	msg_tc = params->pf_egress_q_add.tc_prio;
	intc = &(q_top->intcs_params.intc_params[msg_tc]);

	pr_debug("Host Egress TC[%d], queue Add (num of queues %d)\n", msg_tc, intc->num_rem_outqs);

	/* Clear queue structure */
	memset(&giu_gpio_q, 0, sizeof(union giu_gpio_q_params));

	/* Allocate queue from MQA */
	ret = mqa_queue_alloc(nmnicpf->mqa, &q_id);
	if (ret < 0) {
		pr_err("Failed to allocate queue from MQA\n");
		return ret;
	}

	giu_gpio_q.rem_q.q_id         = q_id;
	giu_gpio_q.rem_q.len          = params->pf_egress_q_add.q_len;
	giu_gpio_q.rem_q.size         = gie_get_desc_size(TX_DESC);
	giu_gpio_q.rem_q.q_base_pa    = (phys_addr_t)params->pf_egress_q_add.q_phys_addr;
	giu_gpio_q.rem_q.cons_base_pa = (phys_addr_t)(params->pf_egress_q_add.q_cons_phys_addr +
										nmnicpf->map.host_map.phys_addr);
	giu_gpio_q.rem_q.cons_base_va = (void *)(params->pf_egress_q_add.q_cons_phys_addr +
										nmnicpf->map.host_map.virt_addr);
	giu_gpio_q.rem_q.host_remap   = nmnicpf->map.host_map.phys_addr;
	giu_gpio_q.rem_q.msix_id      = params->pf_egress_q_add.msix_id;

	active_q_id = tc_q_next_entry_get(intc->rem_outqs_params, intc->num_rem_outqs);
	if (active_q_id < 0) {
		pr_err("Failed to configure queue in Host Egress TC[%d] queue list\n", msg_tc);
		ret = active_q_id;
		goto egress_queue_exit;
	}

	pr_debug("Host Egress TC[%d], queue %d added and index %d\n", msg_tc, q_id, active_q_id);

	memcpy(&(intc->rem_outqs_params[active_q_id]), &(giu_gpio_q), sizeof(union giu_gpio_q_params));

	/* Set queue Id in and prod/cons address in response.
	 * we use the qid as the prod/cons idx in the notification space
	 * since that is the how CP-125 HW works
	 */
	resp_data->q_add_resp.q_prod_cons_phys_addr = q_id;

egress_queue_exit:

	if (ret < 0) {
		if (q_id > 0) {
			mqa_queue_free(nmnicpf->mqa, q_id);
			if (ret)
				pr_err("Failed to free queue Idx %x in MQA\n", q_id);
		}
		pr_err("Host Egress TC[%d] Add queue failed\n", msg_tc);
	}

	return ret;
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
	int ret;

#ifdef GIE_NO_MULTI_Q_SUPPORT_FOR_RSS
	struct giu_gpio_params *q_top = &(nmnicpf->topology_data);

	/* Override Local Ingress number of queues */
	nmnicpf->profile_data.lcl_ingress_q_num =
		q_top->outtcs_params.outtc_params[0].num_rem_inqs;

	/* Override Local Egress number of queues */
	nmnicpf->profile_data.lcl_egress_q_num =
		q_top->intcs_params.intc_params[0].num_rem_outqs;

	/* Initialize local queues database */
	ret = nmnicpf_topology_local_queue_init(nmnicpf);
	if (ret)
		pr_err("Failed to update local DB queue info\n");

	/* Allocate and configure local queues in the database */
	ret = nmnicpf_topology_local_queue_cfg(nmnicpf);
	if (ret)
		pr_err("Failed to configure PF regfile\n");
#endif

	ret = giu_bpool_init(&(nmnicpf->topology_data), &(nmnicpf->giu_bpool));
	if (ret)
		pr_err("Failed to init giu bpool\n");

	ret = giu_gpio_init(&(nmnicpf->topology_data), &(nmnicpf->giu_gpio));
	if (ret)
		pr_err("Failed to init giu gpio\n");

	if (nmnicpf->profile_data.port_type == NMP_LF_NICPF_T_PP2_PORT) {
		ret = nmnicpf_pp2_init_bpools(nmnicpf);
		if (ret)
			pr_err("nmnicpf_pp2_init_bpools failed\n");

		ret = nmnicpf_pp2_init_ppio(nmnicpf);
		if (ret)
			pr_err("nmnicpf_pp2_init_ppio failed\n");
	}

	/* Indicate nmp init_done ready */
	nmnicpf->f_ready_cb(nmnicpf->arg);

	ret = nmnicpf_config_topology_and_update_regfile(nmnicpf);
	if (ret)
		pr_err("Failed to configure PF regfile\n");

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
	int ret;

	pr_debug("Close message.\n");
	pr_info("Closing PF data path resources\n");

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

	/* Free Data Qs and Un-register in MQA/GIE */
	pr_debug("Free Data Qs\n");
	giu_gpio_deinit(nmnicpf->giu_gpio);

	/* Free BPools and Un-register in MQA/GIE */
	pr_debug("Free BM Qs\n");
	giu_bpool_deinit(nmnicpf->giu_bpool);

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

		ret = pp2_ppio_set_mru(nmnicpf->pp2.ports_desc[0].ppio, MVPP2_MTU_TO_MRU(new_mtu));
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

