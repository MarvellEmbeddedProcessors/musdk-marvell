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

#define log_fmt(fmt) "pf: " fmt

#include "std_internal.h"
#include "hw_emul/gie.h"
#include "drivers/mv_mqa_queue.h"
#include "pf_mng_cmd_desc.h"
#include "mng/db.h"
#include "mng/mv_nmp.h"
#include "mng/pci_ep_def.h"
#include "mng/dispatch.h"
#include "pf_regfile.h"
#include "pf.h"
#include "pf_pci_if_desc.h"

#include "env/trace/trc_pf.h"

#define REGFILE_VAR_DIR		"/var/"
#define REGFILE_NAME_PREFIX	"nic-pf-"
#define REGFILE_MAX_FILE_NAME	64

/* TODO: These should be removed. The local queue sizes should match the remote
 * management queue sizes, as received during the init sequence.
 */
#define LOCAL_CMD_QUEUE_SIZE	256
#define LOCAL_NOTIFY_QUEUE_SIZE	256

#define GIU_LCL_Q_IND (0)
#define GIU_REM_Q_IND (1)

#define REGFILE_VERSION		000002	/* Version Format: XX.XX.XX*/

/*
 *	bpool_q_list_set
 *
 *	This function initialize queue configuration parameters
 *	with bpool array received from control message
 */
static void bpool_q_list_set(struct mqa_queue_params *params,
						u32 *bm_pool_q_id_list, u32 max_bm_pool)
{
	u32 bm_pool_num;

	for (bm_pool_num = 0; bm_pool_num < max_bm_pool; bm_pool_num++) {
		if (bm_pool_q_id_list[bm_pool_num] != 0xFFFF) {
			params->bpool_qids[bm_pool_num] = bm_pool_q_id_list[bm_pool_num];
			pr_debug("BP ID: %d.\n", params->bpool_qids[bm_pool_num]);
		} else
			break;
	}

	params->bpool_num = bm_pool_num;
}

/**
 * NIC PF Register File Section
 * ============================
 */

/*
 *	nic_pf_regfile_size
 *
 *	This function calculates the regfile data byte size
 *
 *	@param[in]      nic_pf       - Pointer to the NIC-PF struct which defines the topology
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise (< 0)
 */
static int nic_pf_regfile_size(struct nic_pf *nic_pf)
{
	int size;
	u32 tc_id;

	/* Main topology structure size */
	size = sizeof(struct giu_regfile);

	/* Add BM Pool size */
	size += (sizeof(struct giu_queue) * nic_pf->topology_data.lcl_bm_qs_num);

	/* Add Egress TCs size */
	size += (sizeof(struct giu_tc) * (nic_pf->topology_data.lcl_eg_tcs_num));

	/* Go trough Egress TCs and calc size of queues */
	for (tc_id = 0; tc_id < (nic_pf->topology_data.lcl_eg_tcs_num); tc_id++)
		size += (sizeof(struct giu_queue) * (nic_pf->topology_data.lcl_eg_tcs[tc_id].num_of_queues));

	/* Add Ingress TCs size */
	size += (sizeof(struct giu_tc) * (nic_pf->topology_data.lcl_ing_tcs_num));

	/* Go trough Egress TCs and calc size of queues */
	for (tc_id = 0; tc_id < (nic_pf->topology_data.lcl_ing_tcs_num); tc_id++)
		size += (sizeof(struct giu_queue) * (nic_pf->topology_data.lcl_ing_tcs[tc_id].num_of_queues));

	return size;
}


/*
 *	nic_pf_regfile_open
 *
 *	This function opens the regfile
 *
 *	@param[in]      nic_pf       - Pointer to the NIC-PF struct which defines the topology
 *			file_map   - Pointer to Pointer to Regfile mempry map
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise (< 0)
 */
static int nic_pf_regfile_open(struct nic_pf *nic_pf, void **file_map)
{
	int size;
	char file_name[REGFILE_MAX_FILE_NAME];

	/* Concatenate file path */
	snprintf(file_name, sizeof(file_name), "%s%s%d", REGFILE_VAR_DIR, REGFILE_NAME_PREFIX, nic_pf->pf_id);

	/* Configure queue topology in register file */
	size = nic_pf_regfile_size(nic_pf);

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
 *	nic_pf_regfile_close
 *
 *	This function opens the regfile
 *
 *	@param[in]      file_map   - Pointer to Regfile mempry map for closing the file
 *
 */
static void nic_pf_regfile_close(void *file_map)
{
	regfile_close(file_map);
}


/*
 *	nic_pf_config_header_regfile
 *
 *	This function writes the header pf to the regfile
 *
 *	@param[in]	regfile_data - Struct of the regfile data
 *			file_map   - Pointer to Pointer to Regfile mempry map
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise (< 0)
 */
static int nic_pf_config_header_regfile(struct giu_regfile *regfile_data, void **file_map)
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
 *      @param[in]	nic_pf     - Pointer to the NIC-PF struct which defines the topology
 *			hw_q_id    - The real unique Queue index
 *
 *	@retval	addr on success
 */
static void *get_queue_prod_phys_addr(struct nic_pf *nic_pf, int hw_q_id)
{
	void *pf_cfg_base; /* pointer to HW */

	/* Get BAR0 Configuration space base address */
	pf_cfg_base = (struct mqa_qnpt_entry *)nic_pf->map.cfg_map.phys_addr;

	/* Calc Notification table specifi entry  */
	return (pf_cfg_base + PCI_BAR0_MQA_QNPT_BASE) +	(sizeof(struct mqa_qnct_entry) * hw_q_id);
}


/*
 *	get_queue_cons_phys_addr
 *
 *	This function gets the Physical addr (consumer) of specific queue from the QNCT table
 *
 *      @param[in]	nic_pf     - Pointer to the NIC-PF struct which defines the topology
 *			hw_q_id    - The real unique Queue index
 *
 *	@retval	addr on success
 */
static void *get_queue_cons_phys_addr(struct nic_pf *nic_pf, int hw_q_id)
{
	void *pf_cfg_base;

	/* Get BAR0 Configuration space base address */
	pf_cfg_base = (struct mqa_qnct_entry *)nic_pf->map.cfg_map.phys_addr;

	/* Calc Notification table specifi entry  */
	return (pf_cfg_base + PCI_BAR0_MQA_QNCT_BASE) +	(sizeof(struct mqa_qnct_entry) * hw_q_id);
}


/*
 *	nic_pf_regfile_register_queue
 *
 *	This function register Queue params in Regfile Queue structure
 *	It gets the info from the SNIC-DB and finally update directly the regfile
 *
 *      @param[in]	nic_pf     - Pointer to the NIC-PF struct which defines the topology
 *			hw_q_id    - The real unique Queue index
 *			q_type     - The type of the Queue (Egress / Ingress / BM/...)
 *			file_map   - Pointer to Pointer to Regfile mempry map
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise (< 0)
 */
static int nic_pf_regfile_register_queue(struct nic_pf *nic_pf, u32 hw_q_id,
					 int q_type, void **file_map)
{
	struct giu_queue reg_giu_queue;
	struct db_q *db_q_p = db_queue_get(hw_q_id);

	if (db_q_p == NULL) {
		pr_err("Failed to get queue params from DB (Queue: %d)\n", (int) hw_q_id);
		return -ENODEV;
	}

	reg_giu_queue.hw_id		= db_q_p->params.idx;
	/** TODO - change params naming - change reg_giu_queue.size to reg_giu_queue.len*/
	reg_giu_queue.size		= db_q_p->params.len;
	reg_giu_queue.type		= q_type;
	reg_giu_queue.phy_base_addr	= db_q_p->params.phy_base_addr;
	/* Prod/Cons addr are Virtual. Needs to translate them to Phys addr */
	reg_giu_queue.prod_addr	= get_queue_prod_phys_addr(nic_pf, hw_q_id);
	reg_giu_queue.cons_addr	= get_queue_cons_phys_addr(nic_pf, hw_q_id);

	/* Note: buff_size & payload_offset are union and they are set
	 *	 acoording to the Q type.
	 */
	if (q_type == QUEUE_BP)
		/** TODO - change params naming - change buff_size to buff_len */
		reg_giu_queue.buff_len = nic_pf->profile_data.lcl_bm_buf_size;
	else
		reg_giu_queue.payload_offset = 0; /* TODO: this should not be hardcoded */

	/* Copy Queues parameters */
	pr_debug("\t\tCopy Queue %d Information to Regfile\n", reg_giu_queue.hw_id);
	memcpy(*file_map, &reg_giu_queue, sizeof(struct giu_queue));
	*file_map += sizeof(struct giu_queue);

	return 0;
}


/*
 *	nic_pf_config_topology_tcs_and_update_regfile
 *
 *	This function configures TC params in register file TC structure
 *	It gets the info from general NIC-PF TC params struct, and finally updates the regfile
 *
 *      @param[in]	nic_pf           - Pointer to the NIC-PF struct which defines the topology
 *			tc_params        - Pointer to TC format of the NIC-PF (part of array of such struct)
 *			tc_queue_type    - Type of the Queues in this specific TC
 *			file_map         - Pointer to Pointer to Regfile mempry map
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise (< 0)
 */
static int nic_pf_regfile_register_tc(struct nic_pf *nic_pf, struct tc_params *tc_params,
				      int tc_queue_type, void **file_map)
{
	int queue_idx, ret = 0;
	struct giu_tc reg_giu_tc;

	reg_giu_tc.id			= tc_params->tc_id;
	reg_giu_tc.ingress_rss_type	= tc_params->rss_type;
	reg_giu_tc.num_queues		= tc_params->num_of_queues;
	reg_giu_tc.queues		= NULL;

	/* Copy TC parameters */
	pr_debug("\tCopy TC %d Information to Regfile\n", reg_giu_tc.id);
	memcpy(*file_map, &reg_giu_tc, sizeof(struct giu_tc));
	*file_map += sizeof(struct giu_tc);

	if (tc_params->tc_queues_idx != NULL) {
		for (queue_idx = 0; queue_idx < reg_giu_tc.num_queues; queue_idx++) {
			u32 hw_q_id = tc_params->tc_queues_idx[queue_idx];

			ret = nic_pf_regfile_register_queue(nic_pf, hw_q_id, tc_queue_type, file_map);

			if (ret != 0)
				break;
		}
	} else {
		pr_info("Topology Queue list in TC (%d) is empty (NULL)\n", tc_params->tc_id);
	}

	return ret;

}


/*
 *	nic_pf_config_topology_and_update_regfile
 *
 *	This function configures the entite NIC-PF struct params to the register file
 *      It runs over all TCs types (Egress / Ingress / ...) and BM Queues structure and convert the
 *      Topology to the register file
 *
 *	@param[in]	nic_pf      - Pointer to the NIC-PF struct which defines the topology
 *+
 *	@retval	0 on success
 *	@retval	error-code otherwise (< 0)
 */

static int nic_pf_config_topology_and_update_regfile(struct nic_pf *nic_pf)
{
	void *file_map;
	struct giu_regfile *regfile_data = &nic_pf->regfile_data;
	int tc_idx, queue_idx;
	void *pf_cfg_base;
	int ret = 0;

	/* Update Regfile general info */
	regfile_data->version		= REGFILE_VERSION;
	regfile_data->num_bm_qs		= nic_pf->topology_data.lcl_bm_qs_num;
	regfile_data->bm_qs		= NULL;
	regfile_data->num_egress_tcs	= nic_pf->topology_data.lcl_eg_tcs_num;
	regfile_data->egress_tcs	= NULL;
	regfile_data->num_ingress_tcs	= nic_pf->topology_data.lcl_ing_tcs_num;
	regfile_data->ingress_tcs	= NULL;

	pf_cfg_base = nic_pf->map.cfg_map.virt_addr;
	regfile_data->prod_tbl_base_phys = get_queue_prod_phys_addr(nic_pf, 0);
	regfile_data->prod_tbl_base_virt = (void *)(pf_cfg_base + PCI_BAR0_MQA_QNPT_BASE);
	regfile_data->cons_tbl_base_phys = get_queue_cons_phys_addr(nic_pf, 0);
	regfile_data->cons_tbl_base_virt = (void *)(pf_cfg_base + PCI_BAR0_MQA_QNCT_BASE);

	pr_info("PCI tables addr: cp: %p, cv: %p, pp: %p, pv: %p.\n", regfile_data->prod_tbl_base_phys,
			regfile_data->prod_tbl_base_virt, regfile_data->cons_tbl_base_phys,
			regfile_data->cons_tbl_base_virt);

	pr_info("Start Topology configuration to register file [Regfile ver (%d), NIC-PF number (%d)]\n",
		  regfile_data->version, nic_pf->pf_id);

	ret = nic_pf_regfile_open(nic_pf, &file_map);

	if (ret != 0)
		return ret;

	/* Copy Header File parameters */
	pr_debug("Copy Regfile Header\n");
	ret = nic_pf_config_header_regfile(regfile_data, &file_map);

	if (ret != 0)
		goto config_error;

	/* Setup BM Queues  */
	pr_info("Configure BM Queues Topology (num of BM Queues %d)\n", regfile_data->num_bm_qs);

	if (nic_pf->topology_data.lcl_bm_qs_idx != NULL) {
		for (queue_idx = 0; queue_idx < regfile_data->num_bm_qs; queue_idx++) {

			u32 hw_q_id = nic_pf->topology_data.lcl_bm_qs_idx[queue_idx];

			ret = nic_pf_regfile_register_queue(nic_pf, hw_q_id, QUEUE_BP, &file_map);

			if (ret != 0)
				goto config_error;

		}
	} else {
		pr_info("Topology BM Queue list is empty (NULL)\n");
	}

	/* Setup Egress TCs  */
	pr_info("Configure Egress TCs Topology (num of TCs %d)\n", regfile_data->num_egress_tcs);

	if (nic_pf->topology_data.lcl_eg_tcs != NULL) {
		for (tc_idx = 0; tc_idx < regfile_data->num_egress_tcs; tc_idx++) {

			struct tc_params *tc_params = &(nic_pf->topology_data.lcl_eg_tcs[tc_idx]);

			ret = nic_pf_regfile_register_tc(nic_pf, tc_params, QUEUE_EGRESS, &file_map);

			if (ret != 0)
				goto config_error;

		}
	} else {
		pr_info("Egress TCs Topology is empty (NULL)\n");
	}


	/* Setup Ingress TCs  */
	pr_info("Configure Ingress TCs Topology (num of TCs %d)\n", regfile_data->num_ingress_tcs);

	if (nic_pf->topology_data.lcl_ing_tcs != NULL) {
		for (tc_idx = 0; tc_idx < regfile_data->num_ingress_tcs; tc_idx++) {

			struct tc_params *tc_params = &(nic_pf->topology_data.lcl_ing_tcs[tc_idx]);

			ret = nic_pf_regfile_register_tc(nic_pf, tc_params, QUEUE_INGRESS, &file_map);

			if (ret != 0)
				goto config_error;

		}
	} else {
		pr_info("Ingress TCs Topology is empty (NULL)\n");
	}

	return ret;

config_error:

	nic_pf_regfile_close(file_map);

	return ret;
}


/**
 * NIC PF Initialization Section
 * =============================
 */

/*
 *	nic_pf_db_local_queue_init
 *
 *	This function initialize local queues in NIC PF queue
 *	topology database based on configuration profile
 *	Local queues include data queues and bm queues
 *
 *	@param[in]	nic_pf - pointer to NIC PF object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
static int nic_pf_db_local_queue_init(struct nic_pf *nic_pf)
{
	int ret;
	struct pf_profile *prof = &(nic_pf->profile_data);
	struct pf_queue_topology *q_top = &(nic_pf->topology_data);

	pr_info("Initializing Local Queues in management Database\n");

	/* Local Egress TC */
	ret = db_tc_init(DB_LCL_EG_TC, q_top->lcl_eg_tcs_num, prof->lcl_egress_q_num);
	if (ret) {
		pr_err("Failed to allocate Local Egress TC table\n");
		goto db_error;
	}

	/* Local ingress TC */
	ret = db_tc_init(DB_LCL_ING_TC, q_top->lcl_ing_tcs_num, prof->lcl_ingress_q_num);
	if (ret) {
		pr_err("Failed to allocate Local Ingress TC table\n");
		goto db_error;
	}

	/* Local BM */
	ret = db_bm_init(DB_LCL_BM, prof->lcl_bm_q_num);
	if (ret) {
		pr_err("Failed to allocate Local BM table\n");
		goto db_error;
	}

	return 0;

db_error:

	db_tc_free(DB_LCL_EG_TC, q_top->lcl_eg_tcs_num);
	db_tc_free(DB_LCL_ING_TC, q_top->lcl_ing_tcs_num);
	db_bm_free(DB_LCL_BM);

	return -ENOMEM;
}


/*
 *	nic_pf_db_local_tc_free
 *
 *	This function frees the local TC resources in DB
 */
static void nic_pf_db_local_tc_free(struct nic_pf *nic_pf)
{
	struct pf_queue_topology *q_top = &(nic_pf->topology_data);

	pr_debug("Free Local queues DB\n");
	db_tc_free(DB_LCL_EG_TC, q_top->lcl_eg_tcs_num);
	db_tc_free(DB_LCL_ING_TC, q_top->lcl_ing_tcs_num);
	db_bm_free(DB_LCL_BM);
}


/*
 *	nic_pf_db_remote_queue_init
 *
 *	This function initialize NIC PF remote queue
 *	topology database based on PF_INIT management command
 *	Remote queues include data queues and bm queues
 *
 *	@param[in]	nic_pf - pointer to NIC PF object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
static int nic_pf_db_remote_queue_init(struct nic_pf *nic_pf)
{
	int ret;
	struct pf_queue_topology *q_top = &(nic_pf->topology_data);

	pr_info("Initializing Remote Queues in management Database\n");

	/* Remote Egress TC */
	/* TC queues will be update upon "tc_add" command */
	ret = db_tc_init(DB_HOST_EG_TC, q_top->host_eg_tcs_num, 0);
	if (ret) {
		pr_err("Failed to allocate Local Egress TC table\n");
		goto db_error;
	}

	/* Remote ingress TC */
	/* TC queues will be update upon "tc_add" command */
	ret = db_tc_init(DB_HOST_ING_TC, q_top->host_ing_tcs_num, 0);
	if (ret) {
		pr_err("Failed to allocate Local Ingress TC table\n");
		goto db_error;
	}

	/* Remote BM */
	ret = db_bm_init(DB_HOST_BM, q_top->host_bm_qs_num);
	if (ret) {
		pr_err("Failed to allocate Local BM table\n");
		goto db_error;
	}

	return 0;

db_error:

	pr_err("Remote Queues Initialization failed\n");

	db_tc_free(DB_HOST_EG_TC, q_top->host_eg_tcs_num);
	db_tc_free(DB_HOST_ING_TC, q_top->host_ing_tcs_num);
	db_bm_free(DB_HOST_BM);

	return -ENOMEM;
}


/*
 *	nic_pf_db_remote_tc_free
 *
 *	This function frees the remote TC resources in DB
 */
static int nic_pf_db_remote_tc_free(struct nic_pf *nic_pf)
{
	struct pf_queue_topology *q_top = &(nic_pf->topology_data);

	pr_debug("Free Remote queues DB\n");
	db_tc_free(DB_HOST_EG_TC, q_top->host_eg_tcs_num);
	db_tc_free(DB_HOST_ING_TC, q_top->host_ing_tcs_num);
	db_bm_free(DB_HOST_BM);

	return 0;
}


/*
 *	nic_pf_db_tc_free
 *
 *	This function frees both local & remote TC resources in DB
 */
static int nic_pf_db_tc_free(struct nic_pf *nic_pf)
{
	/* Free Local TC DB structures */
	nic_pf_db_local_tc_free(nic_pf);

	/* Free Remote TC DB structures */
	nic_pf_db_remote_tc_free(nic_pf);

	return 0;
}


/*
 *	nic_pf_db_local_queue_cfg
 *
 *	This function create NIC PF local queue based on configuration profile
 *	Local queues include data queues and bm queues
 *
 *	@param[in]	nic_pf - pointer to NIC PF object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
static int nic_pf_db_local_queue_cfg(struct nic_pf *nic_pf)
{
	int ret;
	u32 tc_idx;
	u32 q_idx;
	u32 bm_idx;
	u32 q_id;

	struct tc_params *tc;
	struct db_q db_q;
	struct db_q *db_q_p;
	struct pf_profile *prof = &(nic_pf->profile_data);
	struct pf_queue_topology *q_top = &(nic_pf->topology_data);

	pr_info("Configure Local BM queues\n");

	/* Create Local BM queues */
	pr_debug("Num of BM queues - %d\n", q_top->lcl_bm_qs_num);

	for (bm_idx = 0; bm_idx < q_top->lcl_bm_qs_num; bm_idx++) {
		/* Clear queue structure */
		memset(&db_q, 0, sizeof(struct db_q));

		/* Allocate queue from MQA */
		ret = mqa_queue_alloc(nic_pf->mqa, &q_id);
		if (ret < 0) {
			pr_err("Failed to allocate queue from MQA\n");
			goto lcl_bm_queue_error;
		}

		/* Init queue parameters */
		db_q.params.idx  = q_id;
		db_q.params.len  = prof->lcl_bm_q_size;
		db_q.params.size = gie_get_desc_size(BUFF_DESC);
		db_q.params.attr = LOCAL_QUEUE | EGRESS_QUEUE;

		/* Save queue info */
		ret = db_queue_set(q_id, &db_q);
		if (ret < 0) {
			pr_err("Failed to save queue info\n");
			goto lcl_bm_queue_error;
		}

		/* Update queue topology database */
		q_top->lcl_bm_qs_idx[bm_idx] = (u32)q_id;
		pr_debug("Configure BM[%d], queue Id %d\n", bm_idx, q_top->lcl_bm_qs_idx[bm_idx]);
	}

	pr_info("Configure Local Egress TC queues\n");

	/* Create Local Egress TC queues */
	pr_debug("Num of Egress TC - %d\n", q_top->lcl_eg_tcs_num);

	for (tc_idx = 0; tc_idx < q_top->lcl_eg_tcs_num; tc_idx++) {

		pr_debug("Egress TC[%d] addr - %p\n", tc_idx, &(q_top->lcl_eg_tcs[tc_idx]));
		tc = &(q_top->lcl_eg_tcs[tc_idx]);

		for (q_idx = 0; q_idx < tc->num_of_queues; q_idx++) {

			/* Clear queue structure */
			memset(&db_q, 0, sizeof(struct db_q));

			/* Allocate queue from MQA */
			ret = mqa_queue_alloc(nic_pf->mqa, &q_id);
			if (ret < 0) {
				pr_err("Failed to allocate queue from MQA\n");
				goto lcl_eg_queue_error;
			}

			db_q.params.idx  = q_id;
			db_q.params.len  = prof->lcl_egress_q_size;
			db_q.params.size = gie_get_desc_size(TX_DESC);
			db_q.params.attr = LOCAL_QUEUE | EGRESS_QUEUE;
			bpool_q_list_set(&db_q.params, q_top->lcl_bm_qs_idx, q_top->lcl_bm_qs_num);

			/* Save queue info */
			ret = db_queue_set(q_id, &db_q);
			if (ret < 0) {
				pr_err("Failed to save queue info\n");
				goto lcl_eg_queue_error;
			}

			/* Update queue topology database */
			tc->tc_queues_idx[q_idx] = (u32)q_id;
			pr_debug("Configure Egress TC[%d], queue[%d] = Id %d\n\n",
					tc_idx, q_idx, tc->tc_queues_idx[q_idx]);
		}
	}

	pr_info("Configure Local Ingress TC queues\n");

	/* Create Local Ingress TC queues */
	pr_debug("Num of Ingress TC - %d\n", q_top->lcl_ing_tcs_num);

	for (tc_idx = 0; tc_idx < q_top->lcl_ing_tcs_num; tc_idx++) {

		pr_debug("Ingress TC[%d] addr - %p\n", tc_idx, &(q_top->lcl_ing_tcs[tc_idx]));
		tc = &(q_top->lcl_ing_tcs[tc_idx]);

		for (q_idx = 0; q_idx < tc->num_of_queues; q_idx++) {

			/* Clear queue structure */
			memset(&db_q, 0, sizeof(struct db_q));

			/* Allocate queue from MQA */
			ret = mqa_queue_alloc(nic_pf->mqa, &q_id);
			if (ret < 0) {
				pr_err("Failed to allocate queue from MQA\n");
				goto lcl_ing_queue_error;
			}

			db_q.params.idx          = (u32)q_id;
			db_q.params.len          = prof->lcl_ingress_q_size;
			db_q.params.size         = gie_get_desc_size(RX_DESC);
			db_q.params.attr         = LOCAL_QUEUE | INGRESS_QUEUE;
			db_q.params.copy_payload = 1;

			/* Save queue info */
			ret = db_queue_set(q_id, &db_q);
			if (ret < 0) {
				pr_err("Failed to save queue info\n");
				goto lcl_ing_queue_error;
			}

			/* Update queue topology database */
			tc->tc_queues_idx[q_idx] = (u32)q_id;
			pr_debug("Configure Ingress TC[%d], queue[%d] = Id %d\n",
					tc_idx, q_idx, tc->tc_queues_idx[q_idx]);
		}
	}

	return 0;

lcl_ing_queue_error:

	for (tc_idx = 0; tc_idx < q_top->lcl_eg_tcs_num; tc_idx++) {
		tc = &(q_top->lcl_eg_tcs[tc_idx]);

		for (q_idx = 0; q_idx < tc->num_of_queues; q_idx++) {

			db_q_p = db_queue_get(tc->tc_queues_idx[q_idx]);
			if (db_q_p != NULL) {
				ret = db_queue_reset(db_q_p->params.idx);
				if (ret)
					pr_err("Failed to reset queue Idx %x in DB\n", db_q_p->params.idx);
				ret = mqa_queue_free(nic_pf->mqa, (u32)db_q_p->params.idx);
				if (ret)
					pr_err("Failed to free queue Idx %x in MQA\n", db_q_p->params.idx);
			}
		}
	}

lcl_eg_queue_error:

	for (tc_idx = 0; tc_idx < q_top->lcl_ing_tcs_num; tc_idx++) {
		tc = &(q_top->lcl_ing_tcs[tc_idx]);

		for (q_idx = 0; q_idx < tc->num_of_queues; q_idx++) {
			db_q_p = db_queue_get(tc->tc_queues_idx[q_idx]);
			if (db_q_p != NULL) {
				ret = db_queue_reset(db_q_p->params.idx);
				if (ret)
					pr_err("Failed to free queue Idx %x in DB\n", db_q_p->params.idx);
				ret = mqa_queue_free(nic_pf->mqa, (u32)db_q_p->params.idx);
				if (ret)
					pr_err("Failed to free queue Idx %x in MQA\n", db_q_p->params.idx);
			}
		}
	}

lcl_bm_queue_error:

	for (bm_idx = 0; bm_idx < q_top->lcl_bm_qs_num; bm_idx++) {
		db_q_p = db_queue_get(q_top->lcl_bm_qs_idx[bm_idx]);
		if (db_q_p != NULL) {
			ret  = db_queue_reset(db_q_p->params.idx);
			if (ret)
				pr_err("Failed to free queue Idx %x in DB\n", db_q_p->params.idx);
			ret = mqa_queue_free(nic_pf->mqa, (u32)db_q_p->params.idx);
			if (ret)
				pr_err("Failed to free queue Idx %x in MQA\n", db_q_p->params.idx);
		}
	}

	return -1;
}


/*
 *	nic_pf_mng_chn_init
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
 *	@param[in]	nic_pf - pointer to NIC PF object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
static int nic_pf_mng_chn_init(struct nic_pf *nic_pf)
{
	volatile struct pcie_config_mem *pcie_cfg;
	struct db_q db_q;
	struct db_q *db_q_p;
	u64 pf_cfg_phys, pf_cfg_virt; /* pointer to HW so it should be volatile */
	void *qnpt_phys, *qnct_phys;
	u32 local_cmd_queue, local_notify_queue;
	u32 remote_cmd_queue, remote_notify_queue;
	int ret = 0;


	/*  Create Local Queues */
	/* ==================== */

	/* Allocate and Register Local Command queue in MQA */
	pr_info("Register Local Command Q\n");

	/* Clear queue structure */
	memset(&db_q, 0, sizeof(struct db_q));

	/* Allocate queue from MQA */
	ret = mqa_queue_alloc(nic_pf->mqa, &local_cmd_queue);
	if (ret < 0) {
		pr_err("Failed to allocate queue from MQA\n");
		goto exit_error;
	}

	db_q.params.idx  = local_cmd_queue;
	db_q.params.len  = LOCAL_CMD_QUEUE_SIZE;
	db_q.params.size = sizeof(struct cmd_desc);
	db_q.params.attr = LOCAL_QUEUE | EGRESS_QUEUE;
	db_q.params.prio = 0;

	/* Save queue info */
	ret = db_queue_set(local_cmd_queue, &db_q);
	if (ret < 0) {
		pr_err("Failed to save queue info\n");
		goto exit_error;
	}

	/* Create the Q and update Q params */
	db_q_p = db_queue_get(local_cmd_queue);
	ret = mqa_queue_create(nic_pf->mqa, &db_q_p->params, &(db_q_p->q));
	if (ret < 0) {
		pr_info("Failed to register Host Management Q\n");
		goto exit_error;
	}


	/* Allocate and Register Local Notification queue in MQA */
	pr_info("Register Local Notification Q\n");

	/* Clear queue structure */
	memset(&db_q, 0, sizeof(struct db_q));

	/* Allocate queue from MQA */
	ret = mqa_queue_alloc(nic_pf->mqa, &local_notify_queue);
	if (ret < 0) {
		pr_err("Failed to allocate queue from MQA\n");
		goto exit_error;
	}

	db_q.params.idx   = local_notify_queue;
	db_q.params.len   = LOCAL_NOTIFY_QUEUE_SIZE;
	db_q.params.size  = sizeof(struct notif_desc);
	db_q.params.attr  = LOCAL_QUEUE | INGRESS_QUEUE;
	db_q.params.prio  = 0;

	/* Save queue info */
	ret = db_queue_set(local_notify_queue, &db_q);
	if (ret < 0) {
		pr_err("Failed to save queue info\n");
		goto exit_error;
	}

	/* Create the Q and update Q params */
	db_q_p = db_queue_get(local_notify_queue);
	ret = mqa_queue_create(nic_pf->mqa, &db_q_p->params, &(db_q_p->q));
	if (ret < 0) {
		pr_info("Failed to register Host Management Q\n");
		goto exit_error;
	}


	/*  Host Ready Check */
	/* ================= */

	/* Get BAR0 Configuration space base address */
	pf_cfg_phys = (u64)nic_pf->map.cfg_map.phys_addr;
	pf_cfg_virt = (u64)nic_pf->map.cfg_map.virt_addr;
	pcie_cfg    = (void *)(pf_cfg_virt + PCI_BAR0_MNG_CH_BASE);

	/* Calc Notification tables base */
	qnct_phys = (void *)(pf_cfg_phys + PCI_BAR0_MQA_QNCT_BASE);
	qnpt_phys = (void *)(pf_cfg_phys + PCI_BAR0_MQA_QNPT_BASE);

	/* Wait for Host to update the state to 'Host Management Ready'
	 * This means that BAR 0 configuration can be accessed as the
	 * Host updated the relevant data/fields.
	 */
	pr_info("Wait till Host change the status to 'Host Management Ready'\n");

	/* TODO - get the mac address from somewhere that makes sense */
	pcie_cfg->mac_addr[0] = 0x0;
	pcie_cfg->mac_addr[1] = 0x1;
	pcie_cfg->mac_addr[2] = 0x2;
	pcie_cfg->mac_addr[3] = 0x3;
	pcie_cfg->mac_addr[4] = 0x4;
	pcie_cfg->mac_addr[5] = 0x5;

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

	/* Make sure that above configuration are out before setting the
	 * dev-ready status for the host side.
	 */
	wmb();

	pcie_cfg->status = PCIE_CFG_STATUS_DEV_READY;

	while (!(pcie_cfg->status & PCIE_CFG_STATUS_HOST_MGMT_READY))
		; /* Do Nothing. Wait till state it's updated */

	pr_info("Host is Ready\n");

	/*  Register Remote Queues */
	/* ======================= */

	/* Register Host Command management queue */
	pr_info("Register host command queue\n");

	/* Clear queue structure */
	memset(&db_q, 0, sizeof(struct db_q));

	/* Allocate queue from MQA */
	ret = mqa_queue_alloc(nic_pf->mqa, &remote_cmd_queue);
	if (ret < 0) {
		pr_err("Failed to allocate queue from MQA\n");
		goto exit_error;
	}

	db_q.params.idx             = remote_cmd_queue;
	db_q.params.len             = pcie_cfg->cmd_q.len;
	db_q.params.size            = sizeof(struct cmd_desc);
	db_q.params.attr            = REMOTE_QUEUE | EGRESS_QUEUE;
	db_q.params.prio            = 0;
	db_q.params.remote_phy_addr = (void *)pcie_cfg->cmd_q.q_addr;
	db_q.params.cons_phys       = (void *)(pcie_cfg->cmd_q.consumer_idx_addr + nic_pf->map.host_map.phys_addr);
	db_q.params.cons_virt       = (void *)(pcie_cfg->cmd_q.consumer_idx_addr + nic_pf->map.host_map.virt_addr);
	db_q.params.host_remap      = nic_pf->map.host_map.phys_addr;
	db_q.params.peer_id         = local_cmd_queue;

	/* Save queue info */
	ret = db_queue_set(remote_cmd_queue, &db_q);
	if (ret < 0) {
		pr_err("Failed to save queue info\n");
		goto exit_error;
	}

	/* Create the Q and update Q params */
	db_q_p = db_queue_get(remote_cmd_queue);
	ret = mqa_queue_create(nic_pf->mqa, &db_q_p->params, &(db_q_p->q));
	if (ret < 0) {
		pr_err("Failed to register Host Management Q\n");
		goto exit_error;
	}

	/* Update PCI BAR0 with producer address (Entry index in notification table) */
	pcie_cfg->cmd_q.producer_idx_addr = (u64)(db_q_p->params.prod_phys - qnpt_phys) / sizeof(u32);


	/* Register Host Notification queue */
	pr_info("Register host notification queue\n");

	/* Clear queue structure */
	memset(&db_q, 0, sizeof(struct db_q));

	/* Allocate queue from MQA */
	ret = mqa_queue_alloc(nic_pf->mqa, &remote_notify_queue);
	if (ret < 0) {
		pr_err("Failed to allocate queue from MQA\n");
		goto exit_error;
	}

	db_q.params.idx             = remote_notify_queue;
	db_q.params.len             = pcie_cfg->notif_q.len;
	db_q.params.size            = sizeof(struct notif_desc);
	db_q.params.attr            = REMOTE_QUEUE | INGRESS_QUEUE;
	db_q.params.prio            = 0;
	db_q.params.remote_phy_addr = (void *)pcie_cfg->notif_q.q_addr;
	db_q.params.prod_phys       = (void *)(pcie_cfg->notif_q.producer_idx_addr + nic_pf->map.host_map.phys_addr);
	db_q.params.prod_virt       = (void *)(pcie_cfg->notif_q.producer_idx_addr + nic_pf->map.host_map.virt_addr);
	db_q.params.host_remap      = nic_pf->map.host_map.phys_addr;

	/* Save queue info */
	ret = db_queue_set(remote_notify_queue, &db_q);
	if (ret < 0) {
		pr_err("Failed to save queue info\n");
		goto exit_error;
	}

	/* Create the Q and update Q params */
	db_q_p = db_queue_get(remote_notify_queue);
	ret = mqa_queue_create(nic_pf->mqa, &db_q_p->params, &(db_q_p->q));
	if (ret < 0) {
		pr_err("Failed to register Host Management Q\n");
		goto exit_error;
	}

	ret = mqa_queue_associate_pair(nic_pf->mqa, local_notify_queue, remote_notify_queue);
	if (ret < 0) {
		pr_err("Failed to associate Notification queues (Src %d Dest %d)\n",
				local_notify_queue, remote_notify_queue);
		goto exit_error;
	}

	/* Update PCI BAR0 with consumer address (Entry index in notification table) */
	pcie_cfg->notif_q.consumer_idx_addr = (u64)(db_q_p->params.cons_phys - qnct_phys) / sizeof(u32);

	/* Update DB */
	/* ========= */
	nic_pf->topology_data.lcl_mng_ctrl.cmd_queue_id     = local_cmd_queue;
	nic_pf->topology_data.lcl_mng_ctrl.notify_queue_id  = local_notify_queue;
	nic_pf->topology_data.host_mng_ctrl.cmd_queue_id    = remote_cmd_queue;
	nic_pf->topology_data.host_mng_ctrl.notify_queue_id = remote_notify_queue;

	/* Register Qs in GIU */
	/* ================== */

	/* Register Command channel */
	gie_add_queue(nic_pf->gie.mng_gie, remote_cmd_queue, 1);

	/* Register Notification channel */
	gie_add_queue(nic_pf->gie.mng_gie, local_notify_queue, 0);


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
		db_q_p = db_queue_get(local_cmd_queue);
		ret = mqa_queue_destroy(nic_pf->mqa, db_q_p->q);
		if (ret < 0)
			pr_err("Failed to free Local Cmd Q %d in DB\n", local_cmd_queue);
		ret = mqa_queue_free(nic_pf->mqa, local_cmd_queue);
		if (ret < 0)
			pr_err("Failed to free Local Cmd Q %d in MQA\n", local_cmd_queue);

		/* Reset the Queue entry in the SNIC-DB */
		db_queue_reset(local_cmd_queue);
	}

	if (local_notify_queue >= 0) {
		db_q_p = db_queue_get(local_notify_queue);
		ret = mqa_queue_destroy(nic_pf->mqa, db_q_p->q);
		if (ret < 0)
			pr_err("Failed to free Local Notify Q %d in DB\n", local_notify_queue);
		ret = mqa_queue_free(nic_pf->mqa, local_notify_queue);
		if (ret < 0)
			pr_err("Failed to free Local Notify Q %d in MQA\n", local_notify_queue);

		/* Reset the Queue entry in the SNIC-DB */
		db_queue_reset(local_notify_queue);
	}

	if (remote_cmd_queue >= 0) {
		db_q_p = db_queue_get(remote_cmd_queue);
		ret = mqa_queue_destroy(nic_pf->mqa, db_q_p->q);
		if (ret < 0)
			pr_err("Failed to free remote Cmd Q %d in DB\n", remote_cmd_queue);
		ret = mqa_queue_free(nic_pf->mqa, remote_cmd_queue);
		if (ret < 0)
			pr_err("Failed to free remote Cmd Q %d in MQA\n", remote_cmd_queue);

		/* Reset the Queue entry in the SNIC-DB */
		db_queue_reset(remote_cmd_queue);
	}

	if (remote_notify_queue >= 0) {
		db_q_p = db_queue_get(remote_notify_queue);
		ret = mqa_queue_destroy(nic_pf->mqa, db_q_p->q);
		if (ret < 0)
			pr_err("Failed to free Remote Notify Q %d in DB\n", remote_notify_queue);
		ret = mqa_queue_free(nic_pf->mqa, remote_notify_queue);
		if (ret < 0)
			pr_err("Failed to free Remote Notify Q %d in MQA\n", remote_notify_queue);

		/* Reset the Queue entry in the SNIC-DB */
		db_queue_reset(remote_notify_queue);
	}

	return ret;
}


/*
 *	nic_pf_init
 *
 *	@param[in]	nic_pf - pointer to NIC PF object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int nic_pf_init(struct nic_pf *nic_pf)
{
	int ret;
	struct nmdisp_client_params params;
	struct nmdisp_q_pair_params q_params;

	nic_pf->pf_id = 0;

	/* Initialize management queues */
	ret = nic_pf_mng_chn_init(nic_pf);
	if (ret)
		return ret;

	/* Register NIC PF to dispatcher */
	params.client_type  = CDT_PF;
	params.client_id    = nic_pf->pf_id;
	params.client_sr_cb = nic_pf_process_command;
	params.client       = nic_pf;

	ret = nmdisp_register_client(nic_pf->nmdisp, &params);
	if (ret)
		return ret;

	/* Add NIC PF command / notification queues to dispatcher */
	q_params.cmd_q.q_id    = nic_pf->topology_data.lcl_mng_ctrl.cmd_queue_id;
	q_params.notify_q.q_id = nic_pf->topology_data.lcl_mng_ctrl.notify_queue_id;

	ret = nmdisp_add_queue(nic_pf->nmdisp, params.client_type, params.client_id, &q_params);
	if (ret)
		return ret;

	return 0;
}


/**
 * NIC PF Termination Section
 * ==========================
 */

/*
 *	nic_pf_local_queue_terminate
 *
 *	This function terminate NIC PF local queue based on configuration profile
 *	Local queues include data queues and bm queues
 *
 *	@param[in]	nic_pf - pointer to NIC PF object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
static int nic_pf_local_queue_terminate(struct nic_pf *nic_pf)
{
	nic_pf = nic_pf;

	return 0;
}


/*
 *	nic_pf_mng_chn_terminate
 *
 *	This function terminate NIC PF management channel
 *	Execution requires handshake with Host side
 *
 *	@param[in]	nic_pf - pointer to NIC PF object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
static int nic_pf_mng_chn_terminate(struct nic_pf *nic_pf)
{
	int local_cmd_queue, local_notify_queue;
	int remote_cmd_queue, remote_notify_queue;
	struct db_q *db_q_p;
	int ret = 0;

	local_cmd_queue = nic_pf->topology_data.lcl_mng_ctrl.cmd_queue_id;
	if (local_cmd_queue >= 0) {
		db_q_p = db_queue_get(local_cmd_queue);
		ret = mqa_queue_destroy(nic_pf->mqa, db_q_p->q);
		if (ret < 0)
			pr_err("Failed to free Local Cmd Q %d in DB\n", local_cmd_queue);
		ret = mqa_queue_free(nic_pf->mqa, local_cmd_queue);
		if (ret < 0)
			pr_err("Failed to free Local Cmd Q %d in MQA\n", local_cmd_queue);

		/* Reset the Queue entry in the SNIC-DB */
		db_queue_reset(local_cmd_queue);
	}

	local_notify_queue = nic_pf->topology_data.lcl_mng_ctrl.notify_queue_id;
	if (local_notify_queue >= 0) {
		db_q_p = db_queue_get(local_notify_queue);
		ret = mqa_queue_destroy(nic_pf->mqa, db_q_p->q);
		if (ret < 0)
			pr_err("Failed to free Local Notify Q %d in DB\n", local_notify_queue);
		ret = mqa_queue_free(nic_pf->mqa, local_notify_queue);
		if (ret < 0)
			pr_err("Failed to free Local Notify Q %d in MQA\n", local_notify_queue);

		/* Reset the Queue entry in the SNIC-DB */
		db_queue_reset(local_notify_queue);
	}

	remote_cmd_queue = nic_pf->topology_data.host_mng_ctrl.cmd_queue_id;
	if (remote_cmd_queue >= 0) {
		db_q_p = db_queue_get(remote_cmd_queue);
		ret = mqa_queue_destroy(nic_pf->mqa, db_q_p->q);
		if (ret < 0)
			pr_err("Failed to free remote Cmd Q %d in DB\n", remote_cmd_queue);
		ret = mqa_queue_free(nic_pf->mqa, remote_cmd_queue);
		if (ret < 0)
			pr_err("Failed to free remote Cmd Q %d in MQA\n", remote_cmd_queue);

		/* Reset the Queue entry in the SNIC-DB */
		db_queue_reset(remote_cmd_queue);
	}

	remote_notify_queue = nic_pf->topology_data.host_mng_ctrl.notify_queue_id;
	if (remote_notify_queue >= 0) {
		db_q_p = db_queue_get(remote_notify_queue);
		ret = mqa_queue_destroy(nic_pf->mqa, db_q_p->q);
		if (ret < 0)
			pr_err("Failed to free Remote Notify Q %d in DB\n", remote_notify_queue);
		ret = mqa_queue_free(nic_pf->mqa, remote_notify_queue);
		if (ret < 0)
			pr_err("Failed to free Remote Notify Q %d in MQA\n", remote_notify_queue);

		/* Reset the Queue entry in the SNIC-DB */
		db_queue_reset(remote_notify_queue);
	}

	return 0;
}


/*
 *	nic_pf_terminate
 *
 *	@param[in]	nic_pf - pointer to NIC PF object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int nic_pf_terminate(struct nic_pf *nic_pf)
{
	int ret;

	ret = nic_pf_local_queue_terminate(nic_pf);
	if (ret)
		return ret;

	ret = nic_pf_mng_chn_terminate(nic_pf);
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
 *	nic_pf_gen_resp_msg
 *
 *	This function initialize response message generic parameters
 *	=== Important: Response data is updated at the scope of
 *	nic_pf_process_init_command / nic_pf_process_exec_command APIs
 *
 *	@param[in]	status - command execution result
 *	@param[in]	cmd - pointer to cmd_desc object
 *	@param[out]	resp - pointer to notif_desc object
 *
 *	@retval	none
 */
static void nic_pf_gen_resp_msg(u32 status, struct cmd_desc *cmd,
					struct notif_desc *resp)
{
	resp->cmd_idx  = cmd->cmd_idx;
	resp->app_code = AC_HOST_SNIC_NETDEV;
	resp->status   = (u8)status;
	resp->flags    = 0;

	/* TODO - Add desc / resp parameters size */
	resp->resp_param_size = 0;
	resp->desc_param_size = 0;
}


/*
 *	nic_pf_pf_init_command
 */
static int nic_pf_pf_init_command(struct nic_pf *nic_pf,
					struct cmd_desc *cmd, struct notif_desc *resp)
{
	int ret;

	struct mgmt_cmd_params *params = &(cmd->params);
	struct pf_queue_topology *q_top = &(nic_pf->topology_data);

	pr_debug("PF INIT\n");
	pr_debug("Num of - Ing TC %d, Eg TC %d, Bpool %d\n",
				params->pf_init.num_host_ingress_tc,
				params->pf_init.num_host_egress_tc,
				params->pf_init.num_host_bm_pools);

	/* Extract message params and update database */
	q_top->host_ing_tcs_num = q_top->lcl_ing_tcs_num = params->pf_init.num_host_ingress_tc;
	q_top->host_eg_tcs_num  = q_top->lcl_eg_tcs_num  = params->pf_init.num_host_egress_tc;
	q_top->host_bm_qs_num   = params->pf_init.num_host_bm_pools;

	/* Initialize remote queues database */
	ret = nic_pf_db_remote_queue_init(nic_pf);
	if (ret)
		pr_err("Failed to update remote DB queue info\n");

	/* Initialize local queues database */
	ret = nic_pf_db_local_queue_init(nic_pf);
	if (ret) {
		pr_err("Failed to update local DB queue info\n");
		goto pf_init_exit;
	}

	/* Allocate and configure local queues in the database */
	ret = nic_pf_db_local_queue_cfg(nic_pf);
	if (ret)
		pr_err("Failed to configure PF regfile\n");

pf_init_exit:

	/* Generate response message */
	nic_pf_gen_resp_msg(ret, cmd, resp);

	pr_debug("PF INIT, Done\n\n");

	return ret;
}


/*
 *	bpool_next_q_get
 *
 *	This function return next available queue in bpool array
 */
static int bpool_next_q_get(u32 *bpool_q_id_list, u32 bpool_q_num)
{
	u32 bpool_q_idx;

	for (bpool_q_idx = 0; bpool_q_idx < bpool_q_num; bpool_q_idx++) {
		if (bpool_q_id_list[bpool_q_idx] == 0)
			return bpool_q_idx;
	}

	return -1;
}


/*
 *	nic_pf_bpool_add_command
 */
static int nic_pf_bpool_add_command(struct nic_pf *nic_pf,
					struct cmd_desc *cmd, struct notif_desc *resp)
{
	int ret = 0;
	s32 next_q_id;
	u32 q_id;

	struct db_q db_q;
	struct mgmt_cmd_params *params = &(cmd->params);
	struct pf_queue_topology *q_top = &(nic_pf->topology_data);

	pr_info("Configure Host BM queues\n");

	next_q_id = bpool_next_q_get(q_top->host_bm_qs_idx, q_top->host_bm_qs_num);
	if (next_q_id < 0) {
		pr_err("Host BM queue add failed, No free entry\n");
		return next_q_id;
	}

	/* Clear queue structure */
	memset(&db_q, 0, sizeof(struct db_q));

	/* Allocate queue from MQA */
	ret = mqa_queue_alloc(nic_pf->mqa, &q_id);
	if (ret < 0) {
		pr_err("Failed to allocate queue from MQA\n");
		goto bpool_exit;
	}

	/* Init queue parameters */
	db_q.params.idx             = q_id;
	db_q.params.len             = params->bm_pool_add.q_len;
	db_q.params.size            = params->bm_pool_add.q_buf_size;
	db_q.params.attr            = REMOTE_QUEUE | EGRESS_QUEUE;
	db_q.params.remote_phy_addr = (void *)params->bm_pool_add.q_phys_addr;
	db_q.params.cons_phys       = (void *)(params->bm_pool_add.q_cons_phys_addr + nic_pf->map.host_map.phys_addr);
	db_q.params.cons_virt       = (void *)(params->bm_pool_add.q_cons_phys_addr + nic_pf->map.host_map.virt_addr);
	db_q.params.host_remap      = nic_pf->map.host_map.phys_addr;

	/* Save queue info */
	ret = db_queue_set(q_id, &db_q);
	if (ret < 0) {
		pr_err("Failed to save queue info\n");
		goto bpool_exit;
	}

	/* Update queue topology database */
	q_top->host_bm_qs_idx[next_q_id] = (u32)q_id;
	pr_debug("BM[%d], queue Id %d\n", next_q_id, q_top->host_bm_qs_idx[next_q_id]);

	/* Set queue Id in response message in case of success */
	resp->resp_data.q_add_resp.q_id = q_id;
	resp->resp_data.q_add_resp.q_prod_cons_phys_addr = q_id;

	pr_debug("BM POOL ADD, Done\n\n");

bpool_exit:

	nic_pf_gen_resp_msg(ret, cmd, resp);

	if (ret < 0) {
		if (q_id > 0) {
			ret = db_queue_reset(q_id);
			if (ret)
				pr_err("Failed to free queue Idx %x in DB\n", q_id);
			ret = mqa_queue_free(nic_pf->mqa, q_id);
			if (ret)
				pr_err("Failed to free queue Idx %x in MQA\n", q_id);
		}
	}

	return ret;
}


/*
 *	nic_pf_egress_tc_add_command
 */
static int nic_pf_egress_tc_add_command(struct nic_pf *nic_pf,
					struct cmd_desc *cmd, struct notif_desc *resp)
{
	int ret = 0;
	u32 *tc_queues;

	struct mgmt_cmd_params *params = &(cmd->params);
	struct pf_queue_topology *q_top = &(nic_pf->topology_data);

	pr_info("Configure Host Egress TC[%d] Queues\n", params->pf_egress_tc_add.tc_prio);

	tc_queues = kcalloc(params->pf_egress_tc_add.num_queues_per_tc, sizeof(u32), GFP_KERNEL);
	if (tc_queues == NULL) {
		ret = -ENOMEM;
		goto tc_exit;
	}

	/* Update queue topology database */
	q_top->host_eg_tcs[params->pf_egress_tc_add.tc_prio].tc_queues_idx = tc_queues;
	q_top->host_eg_tcs[params->pf_egress_tc_add.tc_prio].num_of_queues =
						params->pf_egress_tc_add.num_queues_per_tc;


	pr_debug("EGRESS TC ADD, Done\n\n");

tc_exit:

	/* Generate response message */
	nic_pf_gen_resp_msg(ret, cmd, resp);

	if (ret) {
		if (tc_queues != NULL)
			kfree(tc_queues);

		pr_err("Host Egress TC[%d] Add failed\n", params->pf_egress_tc_add.tc_prio);
	}

	return ret;
}


/*
 *	nic_pf_egress_tc_add_command
 */
static int nic_pf_ingress_tc_add_command(struct nic_pf *nic_pf,
					struct cmd_desc *cmd, struct notif_desc *resp)
{
	int ret = 0;
	u32 *tc_queues;

	struct mgmt_cmd_params *params = &(cmd->params);
	struct pf_queue_topology *q_top = &(nic_pf->topology_data);

	pr_info("Configure Host Ingress TC[%d] Queues\n", params->pf_ingress_tc_add.tc_prio);

	tc_queues = kcalloc(params->pf_ingress_tc_add.num_queues_per_tc, sizeof(u32), GFP_KERNEL);
	if (tc_queues == NULL) {
		ret = -ENOMEM;
		goto tc_exit;
	}

	/* Update queue topology database */
	q_top->host_ing_tcs[params->pf_ingress_tc_add.tc_prio].tc_queues_idx = tc_queues;
	q_top->host_ing_tcs[params->pf_ingress_tc_add.tc_prio].num_of_queues =
						params->pf_ingress_tc_add.num_queues_per_tc;
	q_top->host_ing_tcs[params->pf_ingress_tc_add.tc_prio].rss_type =
						params->pf_ingress_tc_add.hash_type;
	/** TODO - Add support for params->pf_ingress_tc_add.pkt_offset */

	pr_debug("INGRESS TC ADD, Done\n\n");

tc_exit:

	/* Generate response message */
	nic_pf_gen_resp_msg(ret, cmd, resp);

	if (ret) {
		if (tc_queues != NULL)
			kfree(tc_queues);

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
static int tc_q_next_entry_get(u32 *q_id_list, u32 q_num)
{
	u32 q_idx;

	for (q_idx = 0; q_idx < q_num; q_idx++) {
		if (q_id_list[q_idx] == 0)
			return q_idx;
	}

	return -1;
}


/*
 *	nic_pf_ingress_queue_add_command
 */
static int nic_pf_ingress_queue_add_command(struct nic_pf *nic_pf,
					struct cmd_desc *cmd, struct notif_desc *resp)
{
	int ret = 0;
	s32 active_q_id;
	u32 msg_tc;
	u32 q_id;

	struct db_q db_q;
	struct mgmt_cmd_params *params = &(cmd->params);
	struct pf_queue_topology *q_top = &(nic_pf->topology_data);
	struct tc_params *tc;

	msg_tc = params->pf_ingress_data_q_add.tc_prio;
	tc = &(q_top->host_ing_tcs[msg_tc]);

	pr_info("Configure Host Ingress TC queues\n");

	pr_debug("INGRESS_DATA_Q_ADD\n");
	pr_debug("Host Ingress TC[%d], queue Add (num of queues %d)\n", msg_tc, q_top->host_ing_tcs_num);

	/* Clear queue structure */
	memset(&db_q, 0, sizeof(struct db_q));

	/* Allocate queue from MQA */
	ret = mqa_queue_alloc(nic_pf->mqa, &q_id);
	if (ret < 0) {
		pr_err("Failed to allocate queue from MQA\n");
		return ret;
	}

	/* Init queue parameters */
	db_q.params.idx             = q_id;
	db_q.params.len             = params->pf_ingress_data_q_add.q_len;
	db_q.params.size            = gie_get_desc_size(RX_DESC);
	db_q.params.attr            = REMOTE_QUEUE | INGRESS_QUEUE;
	db_q.params.prio            = msg_tc;
	db_q.params.remote_phy_addr = (void *)params->pf_ingress_data_q_add.q_phys_addr;
	db_q.params.prod_phys       = (void *)(params->pf_ingress_data_q_add.q_prod_phys_addr +
											nic_pf->map.host_map.phys_addr);
	db_q.params.prod_virt       = (void *)(params->pf_ingress_data_q_add.q_prod_phys_addr +
											nic_pf->map.host_map.virt_addr);
	db_q.params.host_remap      = nic_pf->map.host_map.phys_addr;
	bpool_q_list_set(&db_q.params, params->pf_ingress_data_q_add.bm_pool_q_id_list, q_top->host_bm_qs_num);

	/* Save queue info */
	ret = db_queue_set(q_id, &db_q);
	if (ret < 0) {
		pr_err("Failed to save queue info\n");
		goto ingress_queue_exit;
	}

	active_q_id = tc_q_next_entry_get(tc->tc_queues_idx, q_top->host_ing_tcs_num);
	if (active_q_id < 0) {
		pr_err("Failed to configure queue in Host Ingress TC[%d] queue list\n", msg_tc);
		ret = active_q_id;
		goto ingress_queue_exit;
	}

	pr_debug("Host Ingress TC[%d], queue %d added and index %d\n", msg_tc, q_id, active_q_id);

	/* Update queue topology database */
	tc->tc_queues_idx[active_q_id] = (u32)q_id;

	/* Set queue Id in and prod/cons address in response.
	 * we use the qid as the prod/cons idx in the notification space
	 * since that is the how CP-125 HW works
	 */
	resp->resp_data.q_add_resp.q_id = q_id;
	resp->resp_data.q_add_resp.q_prod_cons_phys_addr = q_id;

	pr_debug("INGRESS_DATA_Q_ADD, Done\n");

ingress_queue_exit:

	nic_pf_gen_resp_msg(ret, cmd, resp);

	if (ret < 0) {
		if (q_id > 0) {
			ret = db_queue_reset(q_id);
			if (ret)
				pr_err("Failed to free queue Idx %x in DB\n", q_id);
			ret = mqa_queue_free(nic_pf->mqa, q_id);
			if (ret)
				pr_err("Failed to free queue Idx %x in MQA\n", q_id);
		}
		pr_err("Host ingress TC[%d] Add queue failed\n", msg_tc);
	}

	return ret;
}


/*
 *	nic_pf_egress_queue_add_command
 */
static int nic_pf_egress_queue_add_command(struct nic_pf *nic_pf,
					struct cmd_desc *cmd, struct notif_desc *resp)
{
	int ret = 0;
	s32 active_q_id;
	u32 msg_tc;
	u32 q_id;

	struct db_q db_q;
	struct mgmt_cmd_params *params = &(cmd->params);
	struct pf_queue_topology *q_top = &(nic_pf->topology_data);
	struct tc_params *tc;

	pr_info("Configure Host Egress TC queues\n");

	msg_tc = params->pf_egress_q_add.tc_prio;
	tc = &(q_top->host_eg_tcs[msg_tc]);

	pr_debug("EGRESS_DATA_Q_ADD\n");
	pr_debug("Host Engress TC[%d], queue Add\n", msg_tc);

	/* Clear queue structure */
	memset(&db_q, 0, sizeof(struct db_q));

	/* Allocate queue from MQA */
	ret = mqa_queue_alloc(nic_pf->mqa, &q_id);
	if (ret < 0) {
		pr_err("Failed to allocate queue from MQA\n");
		return ret;
	}

	db_q.params.idx             = q_id;
	db_q.params.len             = params->pf_egress_q_add.q_len;
	db_q.params.size            = gie_get_desc_size(TX_DESC);
	db_q.params.attr            = REMOTE_QUEUE | EGRESS_QUEUE;
	db_q.params.prio            = msg_tc;
	db_q.params.remote_phy_addr = (void *)params->pf_egress_q_add.q_phys_addr;
	db_q.params.host_remap      = nic_pf->map.host_map.phys_addr;
	db_q.params.cons_phys       = (void *)(params->pf_egress_q_add.q_cons_phys_addr +
											nic_pf->map.host_map.phys_addr);
	db_q.params.cons_virt       = (void *)(params->pf_egress_q_add.q_cons_phys_addr +
											nic_pf->map.host_map.virt_addr);
	db_q.params.copy_payload    = 1;

	/* Save queue info */
	ret = db_queue_set(q_id, &db_q);
	if (ret < 0) {
		pr_err("Failed to save queue info\n");
		goto egress_queue_exit;
	}

	active_q_id = tc_q_next_entry_get(tc->tc_queues_idx, q_top->host_eg_tcs_num);
	if (active_q_id < 0) {
		pr_err("Failed to configure queue in Host Egress TC[%d] queue list\n", msg_tc);
		ret = active_q_id;
		goto egress_queue_exit;
	}

	pr_debug("Host Egress TC[%d], queue %d added and index %d\n", msg_tc, q_id, active_q_id);

	/* Update queue topology database */
	tc->tc_queues_idx[active_q_id] = (u32)q_id;

	/* Set queue Id in and prod/cons address in response.
	 * we use the qid as the prod/cons idx in the notification space
	 * since that is the how CP-125 HW works
	 */
	resp->resp_data.q_add_resp.q_id = q_id;
	resp->resp_data.q_add_resp.q_prod_cons_phys_addr = q_id;

	pr_debug("EGRESS_DATA_Q_ADD, Done\n");

egress_queue_exit:

	nic_pf_gen_resp_msg(ret, cmd, resp);

	if (ret < 0) {
		if (q_id > 0) {
			ret = db_queue_reset(q_id);
			if (ret)
				pr_err("Failed to free queue Idx %x in DB\n", q_id);
			mqa_queue_free(nic_pf->mqa, q_id);
			if (ret)
				pr_err("Failed to free queue Idx %x in MQA\n", q_id);
		}
		pr_err("Host Egress TC[%d] Add queue failed\n", msg_tc);
	}

	return ret;
}

/*
 *	nic_pf_queue_remove
 */
static int nic_pf_queue_remove(struct nic_pf *nic_pf, struct db_q *db_q, enum queue_type queue_type, void *giu_handle)
{
	u32 q_id = db_q->params.idx;
	int ret = 0;

	pr_debug("Remove queue %d (type %d)\n", q_id, queue_type);

	if (giu_handle) {
		/* Un-register Q from GIU */
		if (queue_type == LOCAL_BM_QUEUE ||
		    queue_type == HOST_BM_QUEUE)
			ret = gie_remove_bm_queue(giu_handle, q_id);
		else
			ret = gie_remove_queue(giu_handle, q_id);
		if (ret)
			pr_err("Failed to remove queue Idx %x from GIU\n", q_id);
	}

	/* For local queue: destroy the queue (as it was allocated by the NIC */
	if (queue_type == LOCAL_INGRESS_DATA_QUEUE ||
	    queue_type == LOCAL_EGRESS_DATA_QUEUE ||
	    queue_type == LOCAL_BM_QUEUE) {
		ret = mqa_queue_destroy(nic_pf->mqa, db_q->q);
		if (ret)
			pr_err("Failed to free queue Idx %x in DB\n", q_id);
	}

	/* Free the MQA resource */
	ret = mqa_queue_free(nic_pf->mqa, q_id);
	if (ret)
		pr_err("Failed to free queue Idx %x in MQA\n", q_id);

	/* Reset the Queue DB entry */
	db_queue_reset(q_id);

	return ret;
}

/*
 *	nic_pf_giu_bpool_init
 *
 *
 *	@param[in]	nic_pf - pointer to NIC PF object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
static int nic_pf_giu_bpool_init(struct nic_pf *nic_pf)
{
	int ret;
	u32 bm_idx;
	struct db_q *db_q_p;
	struct pf_profile *prof = &(nic_pf->profile_data);
	struct pf_queue_topology *q_top = &(nic_pf->topology_data);

	pr_info("Initializing Local BM queues\n");

	/* Create Local BM queues */
	for (bm_idx = 0; bm_idx < q_top->lcl_bm_qs_num; bm_idx++) {

		db_q_p = db_queue_get(q_top->lcl_bm_qs_idx[bm_idx]);

		ret = mqa_queue_create(nic_pf->mqa, &db_q_p->params, &(db_q_p->q));
		if (ret < 0) {
			pr_err("Failed to allocate Local BM queues\n");
			goto lcl_queue_error;
		}

		/* Register Local BM Queue to GIU */
		ret = gie_add_bm_queue(nic_pf->gie.tx_gie, db_q_p->params.idx, prof->lcl_bm_buf_size, GIU_LCL_Q_IND);
		if (ret) {
			pr_err("Failed to register BM Queue %d to GIU\n", db_q_p->params.idx);
			goto lcl_queue_error;
		}
		pr_debug("Local BM[%d], queue Id %d, Registered to GIU TX\n\n", bm_idx, db_q_p->params.idx);
	}


	pr_info("Initializing Remote BM queues\n");

	/* Create Remote BM queues */
	for (bm_idx = 0; bm_idx < q_top->host_bm_qs_num; bm_idx++) {

		db_q_p = db_queue_get(q_top->host_bm_qs_idx[bm_idx]);

		ret = mqa_queue_create(nic_pf->mqa, &db_q_p->params, &(db_q_p->q));
		if (ret < 0) {
			pr_err("Failed to allocate queue for Host BM\n");
			goto host_queue_error;
		}

		/* Register Host BM Queue to GIU */
		ret = gie_add_bm_queue(nic_pf->gie.rx_gie, db_q_p->params.idx, db_q_p->params.size, GIU_REM_Q_IND);
		if (ret) {
			pr_err("Failed to register BM Queue %d to GIU\n", db_q_p->params.idx);
			goto host_queue_error;
		}
		pr_debug("Host BM[%d], queue Id %d, Registered to GIU RX\n\n", bm_idx, db_q_p->params.idx);
	}

	return 0;

host_queue_error:

	for (bm_idx = 0; bm_idx < q_top->host_bm_qs_num; bm_idx++) {
		db_q_p = db_queue_get(q_top->host_bm_qs_idx[bm_idx]);
		if (db_q_p != NULL) {
			ret = gie_remove_bm_queue(nic_pf->gie.rx_gie, db_q_p->params.idx);
			if (ret)
				pr_err("Failed to remove queue Idx %x from GIU TX\n", db_q_p->params.idx);
			ret = mqa_queue_destroy(nic_pf->mqa, db_q_p->q);
			if (ret)
				pr_err("Failed to free queue Idx %x in DB\n", db_q_p->params.idx);
			ret = mqa_queue_free(nic_pf->mqa, (u32)db_q_p->params.idx);
			if (ret)
				pr_err("Failed to free queue Idx %x in MQA\n", db_q_p->params.idx);

			/* Reset the Queue DB entry */
			db_queue_reset(db_q_p->params.idx);
		}
	}

lcl_queue_error:

	for (bm_idx = 0; bm_idx < q_top->lcl_bm_qs_num; bm_idx++) {
		db_q_p = db_queue_get(q_top->lcl_bm_qs_idx[bm_idx]);
		if (db_q_p != NULL) {
			ret = gie_remove_bm_queue(nic_pf->gie.tx_gie, db_q_p->params.idx);
			if (ret)
				pr_err("Failed to remove queue Idx %x from GIU TX\n", db_q_p->params.idx);
			ret = mqa_queue_destroy(nic_pf->mqa, db_q_p->q);
			if (ret)
				pr_err("Failed to free queue Idx %x in DB\n", db_q_p->params.idx);
			ret = mqa_queue_free(nic_pf->mqa, (u32)db_q_p->params.idx);
			if (ret)
				pr_err("Failed to free queue Idx %x in MQA\n", db_q_p->params.idx);

			/* Reset the Queue DB entry */
			db_queue_reset(db_q_p->params.idx);
		}
	}

	return -1;
}


/*
 *	nic_pf_giu_bpool_deinit
 *
 *
 *	@param[in]	nic_pf - pointer to NIC PF object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
static int nic_pf_giu_bpool_deinit(struct nic_pf *nic_pf)
{
	int ret;
	u32 bm_idx;
	struct db_q *db_q_p;
	struct pf_queue_topology *q_top = &(nic_pf->topology_data);

	pr_debug("De-initializing Remote BM queues\n");
	for (bm_idx = 0; bm_idx < q_top->host_bm_qs_num; bm_idx++) {
		db_q_p = db_queue_get(q_top->host_bm_qs_idx[bm_idx]);
		if (db_q_p != NULL) {
			ret = nic_pf_queue_remove(nic_pf, db_q_p, HOST_BM_QUEUE, nic_pf->gie.rx_gie);
			if (ret)
				pr_err("Failed to remove queue Idx %x\n", db_q_p->params.idx);
		}
	}

	pr_debug("De-initializing Local BM queues\n");
	for (bm_idx = 0; bm_idx < q_top->lcl_bm_qs_num; bm_idx++) {
		db_q_p = db_queue_get(q_top->lcl_bm_qs_idx[bm_idx]);
		if (db_q_p != NULL) {
			ret = nic_pf_queue_remove(nic_pf, db_q_p, LOCAL_BM_QUEUE, nic_pf->gie.tx_gie);
			if (ret)
				pr_err("Failed to remove queue Idx %x\n", db_q_p->params.idx);
		}
	}

	return 0;
}


static int nic_pf_free_tc_queues(struct nic_pf *nic_pf, struct tc_params *tc, void *gie)
{
	struct db_q *db_q_p;
	u32 q_idx;
	int ret;

	for (q_idx = 0; q_idx < tc->num_of_queues; q_idx++) {
		db_q_p = db_queue_get(tc->tc_queues_idx[q_idx]);
		if (db_q_p != NULL) {
			ret  = db_queue_reset(db_q_p->params.idx);
			if (ret)
				pr_err("Failed to free queue Idx %x\n", db_q_p->params.idx);
			mqa_queue_free(nic_pf->mqa, (u32)db_q_p->params.idx);

			/* If needed, unregister the queue from GIU polling */
			if (gie) {
				ret = gie_remove_queue(gie, db_q_p->params.idx);
				if (ret)
					pr_err("Failed to remove queue Idx %x from GIU TX\n", db_q_p->params.idx);
			}
		}
	}

	return 0;
}


/*
 *	nic_pf_giu_gpio_init
 *
 *
 *	@param[in]	nic_pf - pointer to NIC PF object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
static int nic_pf_giu_gpio_init(struct nic_pf *nic_pf)
{
	int ret;
	u32 tc_idx;
	u32 q_idx;
	s32 pair_qid;
	struct tc_params *tc;
	struct db_q *db_q_p;
	struct pf_queue_topology *q_top = &(nic_pf->topology_data);

	pr_info("Initializing Local Egress TC queues\n");

	/* Create Local Egress TC queues */
	pr_debug("Num of Egress TC - %d\n", q_top->lcl_eg_tcs_num);
	pr_debug("Egress TCs Array addr - %p\n", q_top->lcl_eg_tcs);

	for (tc_idx = 0; tc_idx < q_top->lcl_eg_tcs_num; tc_idx++) {

		pr_debug("Egress TC[%d] addr - %p\n", tc_idx, &(q_top->lcl_eg_tcs[tc_idx]));
		tc = &(q_top->lcl_eg_tcs[tc_idx]);

		for (q_idx = 0; q_idx < tc->num_of_queues; q_idx++) {

			db_q_p = db_queue_get(tc->tc_queues_idx[q_idx]);
			if (db_q_p) {
				ret = mqa_queue_create(nic_pf->mqa, &db_q_p->params, &(db_q_p->q));
				if (ret < 0) {
					pr_err("Failed to allocate queue for Host BM\n");
					goto lcl_eg_queue_error;
				}
			}
		}
	}


	pr_info("Initializing Local Ingress TC queues\n");

	/* Create Local Ingress TC queues */
	pr_debug("Num of Ingress TC - %d\n", q_top->lcl_ing_tcs_num);
	pr_debug("Ingress TCs Array addr - %p\n", q_top->lcl_ing_tcs);

	for (tc_idx = 0; tc_idx < q_top->lcl_ing_tcs_num; tc_idx++) {

		pr_debug("Ingress TC[%d] addr - %p\n", tc_idx, &(q_top->lcl_ing_tcs[tc_idx]));
		tc = &(q_top->lcl_ing_tcs[tc_idx]);

		for (q_idx = 0; q_idx < tc->num_of_queues; q_idx++) {

			db_q_p = db_queue_get(tc->tc_queues_idx[q_idx]);
			if (db_q_p) {
				ret = mqa_queue_create(nic_pf->mqa, &db_q_p->params, &(db_q_p->q));
				if (ret < 0) {
					pr_err("Failed to allocate queue for Host BM\n");
					goto lcl_ing_queue_error;
				}
			}
		}
	}

	pr_info("Initializing Host Ingress TC queues\n");

	/* Create Host Ingress TC queues */
	for (tc_idx = 0; tc_idx < q_top->host_ing_tcs_num; tc_idx++) {

		pr_info("Host ingress TC[%d] addr - %p\n", tc_idx, &(q_top->host_ing_tcs[tc_idx]));
		tc = &(q_top->host_ing_tcs[tc_idx]);

		for (q_idx = 0; q_idx < tc->num_of_queues; q_idx++) {

			db_q_p = db_queue_get(tc->tc_queues_idx[q_idx]);
			if (db_q_p) {

				pair_qid = q_top->lcl_ing_tcs[db_q_p->params.prio].tc_queues_idx[0];

				ret = mqa_queue_create(nic_pf->mqa, &db_q_p->params, &(db_q_p->q));
				if (ret < 0) {
					pr_err("Failed to allocate queue for Host BM\n");
					goto host_ing_queue_error;
				}

				ret = mqa_queue_associate_pair(nic_pf->mqa, pair_qid, db_q_p->params.idx);
				if (ret) {
					pr_err("Failed to associate remote egress Queue %d\n", db_q_p->params.idx);
					goto host_ing_queue_error;
				}

				ret = gie_add_queue(nic_pf->gie.rx_gie, pair_qid, GIU_LCL_Q_IND);
				if (ret) {
					pr_err("Failed to register Host Egress Queue %d to GIU\n", db_q_p->params.idx);
					goto host_ing_queue_error;
				}
			}
		}
	}


	pr_info("Initializing Host Egress TC queues\n");

	/* Create Host Egress TC queues */
	for (tc_idx = 0; tc_idx < q_top->host_eg_tcs_num; tc_idx++) {

		pr_info("Host Egress TC[%d] addr - %p\n", tc_idx, &(q_top->host_eg_tcs[tc_idx]));
		tc = &(q_top->host_eg_tcs[tc_idx]);

		for (q_idx = 0; q_idx < tc->num_of_queues; q_idx++) {

			db_q_p = db_queue_get(tc->tc_queues_idx[q_idx]);
			if (db_q_p) {

				pair_qid = q_top->lcl_eg_tcs[db_q_p->params.prio].tc_queues_idx[0];

				ret = mqa_queue_create(nic_pf->mqa, &db_q_p->params, &(db_q_p->q));
				if (ret < 0) {
					pr_err("Failed to allocate queue for Host BM\n");
					goto host_eg_queue_error;
				}

				ret = mqa_queue_associate_pair(nic_pf->mqa, db_q_p->params.idx, pair_qid);
				if (ret) {
					pr_err("Failed to associate remote egress Queue %d\n", db_q_p->params.idx);
					goto host_eg_queue_error;
				}

				/* Register Host Egress Queue to GIU */
				ret = gie_add_queue(nic_pf->gie.tx_gie, db_q_p->params.idx, GIU_REM_Q_IND);
				if (ret) {
					pr_err("Failed to register Host Egress Queue %d to GIU\n", db_q_p->params.idx);
					goto host_eg_queue_error;
				}
			}
		}
	}

	return 0;

host_eg_queue_error:

	for (tc_idx = 0; tc_idx < q_top->host_eg_tcs_num; tc_idx++) {
		tc = &(q_top->host_eg_tcs[tc_idx]);

		/* Free queue resources and registrations.
		 * for Egress, also un-register from Tx GIU.
		 */
		ret = nic_pf_free_tc_queues(nic_pf, tc, nic_pf->gie.tx_gie);
		if (ret)
			pr_err("Failed to free TC %d queues\n", tc_idx);
	}

host_ing_queue_error:

	for (tc_idx = 0; tc_idx < q_top->host_ing_tcs_num; tc_idx++) {
		tc = &(q_top->host_ing_tcs[tc_idx]);

		/* Free queue resources and registrations.
		 * No need to unregister the Q from GIU as it's done on local side.
		 */
		ret = nic_pf_free_tc_queues(nic_pf, tc, 0);
		if (ret)
			pr_err("Failed to free TC %d queues\n", tc_idx);
	}

lcl_ing_queue_error:

	for (tc_idx = 0; tc_idx < q_top->lcl_ing_tcs_num; tc_idx++) {
		tc = &(q_top->lcl_ing_tcs[tc_idx]);

		/* Free queue resources and registrations.
		 * for Ingress, also un-register from Rx GIU.
		 */
		ret = nic_pf_free_tc_queues(nic_pf, tc, nic_pf->gie.rx_gie);
		if (ret)
			pr_err("Failed to free TC %d queues\n", tc_idx);
	}

lcl_eg_queue_error:

	for (tc_idx = 0; tc_idx < q_top->lcl_eg_tcs_num; tc_idx++) {
		tc = &(q_top->lcl_eg_tcs[tc_idx]);

		/* Free queue resources and registrations.
		 * No need to unregister the Q from GIU as it was done on remote side.
		 */
		ret = nic_pf_free_tc_queues(nic_pf, tc, 0);
		if (ret)
			pr_err("Failed to free TC %d queues\n", tc_idx);
	}

	return -1;
}


/*
 *	nic_pf_giu_gpio_deinit
 *
 *
 *	@param[in]	nic_pf - pointer to NIC PF object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
static int nic_pf_giu_gpio_deinit(struct nic_pf *nic_pf)
{
	int ret;
	u32 tc_idx;
	u32 q_idx;
	struct tc_params *tc;
	struct db_q *db_q_p;
	struct pf_queue_topology *q_top = &(nic_pf->topology_data);

	pr_debug("De-initializing Host Egress TC queues\n");

	for (tc_idx = 0; tc_idx < q_top->host_eg_tcs_num; tc_idx++) {
		tc = &(q_top->host_eg_tcs[tc_idx]);

		for (q_idx = 0; q_idx < tc->num_of_queues; q_idx++) {
			db_q_p = db_queue_get(tc->tc_queues_idx[q_idx]);
			if (db_q_p != NULL) {
				ret = nic_pf_queue_remove(nic_pf, db_q_p, HOST_EGRESS_DATA_QUEUE, nic_pf->gie.tx_gie);
				if (ret)
					pr_err("Failed to remove queue Idx %x\n", db_q_p->params.idx);
			}
		}
	}

	pr_debug("De-initializing Host Ingress TC queues\n");

	for (tc_idx = 0; tc_idx < q_top->host_ing_tcs_num; tc_idx++) {
		tc = &(q_top->host_ing_tcs[tc_idx]);

		for (q_idx = 0; q_idx < tc->num_of_queues; q_idx++) {
			db_q_p = db_queue_get(tc->tc_queues_idx[q_idx]);
			if (db_q_p != NULL) {
				ret = nic_pf_queue_remove(nic_pf, db_q_p, HOST_INGRESS_DATA_QUEUE, 0);
				if (ret)
					pr_err("Failed to remove queue Idx %x\n", db_q_p->params.idx);
			}
		}
	}

	pr_debug("De-initializing Local Ingress TC queues\n");

	for (tc_idx = 0; tc_idx < q_top->lcl_ing_tcs_num; tc_idx++) {
		tc = &(q_top->lcl_ing_tcs[tc_idx]);

		for (q_idx = 0; q_idx < tc->num_of_queues; q_idx++) {
			db_q_p = db_queue_get(tc->tc_queues_idx[q_idx]);
			if (db_q_p != NULL) {
				ret = nic_pf_queue_remove(nic_pf, db_q_p, LOCAL_INGRESS_DATA_QUEUE, nic_pf->gie.rx_gie);
				if (ret)
					pr_err("Failed to remove queue Idx %x\n", db_q_p->params.idx);
			}
		}
	}

	pr_debug("De-initializing Local Egress TC queues\n");

	for (tc_idx = 0; tc_idx < q_top->lcl_eg_tcs_num; tc_idx++) {
		tc = &(q_top->lcl_eg_tcs[tc_idx]);

		for (q_idx = 0; q_idx < tc->num_of_queues; q_idx++) {
			db_q_p = db_queue_get(tc->tc_queues_idx[q_idx]);
			if (db_q_p != NULL) {
				ret = nic_pf_queue_remove(nic_pf, db_q_p, LOCAL_EGRESS_DATA_QUEUE, 0);
				if (ret)
					pr_err("Failed to remove queue Idx %x\n", db_q_p->params.idx);
			}
		}
	}

	return 0;
}


/*
 *	nic_pf_pf_init_done_command
 */
static void nic_pf_pf_init_done_command(struct nic_pf *nic_pf,
					struct cmd_desc *cmd, struct notif_desc *resp)
{
	int ret;


	ret = nic_pf_giu_bpool_init(nic_pf);
	if (ret)
		pr_err("Failed to init giu bpool\n");

	ret = nic_pf_giu_gpio_init(nic_pf);
	if (ret)
		pr_err("Failed to init giu gpio\n");

	ret = nic_pf_config_topology_and_update_regfile(nic_pf);
	if (ret)
		pr_err("Failed to configure PF regfile\n");

	/* Generate response message */
	nic_pf_gen_resp_msg(ret, cmd, resp);
}


/*
 *	nic_pf_mgmt_echo_command
 */
static int nic_pf_mgmt_echo_command(struct nic_pf *nic_pf,
					struct cmd_desc *cmd, struct notif_desc *resp)
{
	pr_debug("Management echo message idx:%d.\n", cmd->cmd_idx);

	nic_pf = nic_pf;

	/* Generate response message */
	nic_pf_gen_resp_msg(0, cmd, resp);

	return 0;
}


/*
 *	nic_pf_link_status_command
 */
static int nic_pf_link_status_command(struct nic_pf *nic_pf,
					struct cmd_desc *cmd, struct notif_desc *resp)
{
	pr_debug("Link status message idx:%d.\n", cmd->cmd_idx);

	nic_pf = nic_pf;

	/* Generate response message */
	nic_pf_gen_resp_msg(0, cmd, resp);

	/* TODO: check PP2 link and report it back to the host */
	pr_warn("GIU Link status is set to 'up'\n");
	resp->resp_data.link_status = 1;

	return 0;
}


/*
 *	nic_pf_process_command
 *
 *	This function process all PF initialization commands
 *
 *	@param[in]	nic_pf - pointer to NIC PF object
 *	@param[in]	cmd - pointer to cmd_desc object
 *	@param[out]	resp - pointer to notif_desc object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int nic_pf_process_command(void *nic_pf, u8 cmd_code, void *cmd)
{
	int ret;
	struct notif_desc resp;

	switch (cmd_code) {

	case CC_PF_ENABLE:
		break;

	case CC_PF_DISABLE:
		break;

	case CC_PF_INIT:
		ret = nic_pf_pf_init_command((struct nic_pf *)nic_pf, (struct cmd_desc *)cmd, &resp);
		if (ret)
			pr_err("PF_INIT message failed\n");
		break;

	case CC_PF_BM_POOL_ADD:
		ret = nic_pf_bpool_add_command((struct nic_pf *)nic_pf, (struct cmd_desc *)cmd, &resp);
		if (ret)
			pr_err("PF_BM_POOL_ADD message failed\n");
		break;

	case CC_PF_EGRESS_TC_ADD:
		ret = nic_pf_egress_tc_add_command((struct nic_pf *)nic_pf, (struct cmd_desc *)cmd, &resp);
		if (ret)
			pr_err("PF_EGRESS_TC_ADD message failed\n");
		break;

	case CC_PF_EGRESS_DATA_Q_ADD:
		ret = nic_pf_egress_queue_add_command((struct nic_pf *)nic_pf, (struct cmd_desc *)cmd, &resp);
		if (ret)
			pr_err("PF_EGRESS_DATA_Q_ADD message failed\n");
		break;

	case CC_PF_INGRESS_TC_ADD:
		ret = nic_pf_ingress_tc_add_command((struct nic_pf *)nic_pf, (struct cmd_desc *)cmd, &resp);
		if (ret)
			pr_err("PF_INGRESS_TC_ADD message failed\n");
		break;

	case CC_PF_INGRESS_DATA_Q_ADD:
		ret = nic_pf_ingress_queue_add_command((struct nic_pf *)nic_pf, (struct cmd_desc *)cmd, &resp);
		if (ret)
			pr_err("PF_INGRESS_DATA_Q_ADD message failed\n");
		break;

	case CC_PF_INIT_DONE:
		nic_pf_pf_init_done_command((struct nic_pf *)nic_pf, (struct cmd_desc *)cmd, &resp);
		break;

	case CC_PF_MGMT_ECHO:
		ret = nic_pf_mgmt_echo_command((struct nic_pf *)nic_pf, (struct cmd_desc *)cmd, &resp);
		if (ret)
			pr_err("PF_MGMT_ECHO message failed\n");
		break;

	case CC_PF_LINK_STATUS:
		ret = nic_pf_link_status_command((struct nic_pf *)nic_pf, (struct cmd_desc *)cmd, &resp);
		if (ret)
			pr_err("PF_LINK_STATUS message failed\n");
		break;

	default:
		/* Unknown command code */
		pr_err("Unknown command code %d!! Unable to process command.\n", cmd_code);
		resp.status = NOTIF_STATUS_FAIL;

		break;
	}

	ret = nmdisp_send(((struct nic_pf *)nic_pf)->nmdisp, CDT_PF,
					  ((struct nic_pf *)nic_pf)->pf_id, 0, (void *)&resp);
	if (ret) {
		pr_err("failed to send response message\n");
		return ret;
	}

	return 0;
}

