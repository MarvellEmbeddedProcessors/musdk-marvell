/******************************************************************************
 *	Copyright (C) 2016 Marvell International Ltd.
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

/**
 * @file pp2_cls_db.c
 *
 * access routines to pp2_cls_db
 */

/***********************/
/* c file declarations */
/***********************/
#include "std_internal.h"

#include "../pp2_types.h"
#include "../pp2.h"
#include "../pp2_hw_type.h"
#include "../pp2_hw_cls.h"

/*******************************************************************************
 * pp2_cls_db_mem_alloc_init
 *
 * DESCRIPTION: The routine will allcate mem for pp2_cls db and init it.
 *
 * INPUTS:
 *	inst   - packet processor instance
 *
 * OUTPUTS:
 *	None
 *
 * RETURN:
 *	0 on success, error-code otherwise
 ******************************************************************************/
static int pp2_cls_db_mem_alloc_init(struct pp2_inst *inst)
{
	/* Allocation for per-instance database */
	inst->cls_db = kmalloc(sizeof(*inst->cls_db), GFP_KERNEL);
	if (!inst->cls_db)
		goto fail1;

	/* Erase DB */
	MVPP2_MEMSET_ZERO(*inst->cls_db);

	return 0;

fail1:
	pr_err("PP2_CLS DB memory allocation failed\n");
	return -ENOMEM;
}

/*******************************************************************************
 * pp2_cls_db_mem_free
 *
 * DESCRIPTION: The routine will free memory for allocated for pp2_cls db.
 *
 * INPUTS:
 *	inst   - packet processor instance
 *
 * OUTPUTS:
 *	None
 *
 * RETURN:
 *	0 on success, error-code otherwise
 ******************************************************************************/
static int pp2_cls_db_mem_free(struct pp2_inst *inst)
{
	kfree(inst->cls_db);

	return 0;
}

/*******************************************************************************
 * pp2_cls_db_c3_free_logic_idx_get()
 *
 * DESCRIPTION: Get a free logic index from list.
 *
 * INPUTS:
 *	inst   - packet processor instance
 *
 * OUTPUTS:
 *	logic_idx - logical index
 *
 * RETURN:
 *	0 on success, error-code otherwise
 ******************************************************************************/
int pp2_cls_db_c3_free_logic_idx_get(struct pp2_inst *inst, u32 *logic_idx)
{
	int idx;

	if (mv_pp2x_ptr_validate(logic_idx))
		return -EINVAL;

	/* search for valid C3 logical index */
	for (idx = 0; idx < MVPP2_CLS_C3_HASH_TBL_SIZE; idx++) {
		if (inst->cls_db->c3_db.hash_idx_tbl[idx].valid == MVPP2_C3_ENTRY_INVALID)
			break;
	}

	if (idx >= MVPP2_CLS_C3_HASH_TBL_SIZE) {
		*logic_idx = 0;
		return -EIO;
	}

	*logic_idx = idx;

	return 0;
}

/*******************************************************************************
 * pp2_cls_db_c3_entry_add()
 *
 * DESCRIPTION: Add C3 entry to DB.
 *
 * INPUTS:
 *	inst   - packet processor instance
 *	logic_idx      - logical index
 *	hash_idx       - multihash index
 *
 * OUTPUTS:
 *	None
 *
 * RETURN:
 *	0 on success, error-code otherwise
 ******************************************************************************/
int pp2_cls_db_c3_entry_add(struct pp2_inst *inst, u32 logic_idx, u32 hash_idx)
{
	if (mv_pp2x_range_validate(logic_idx, 0, MVPP2_CLS_C3_HASH_TBL_SIZE - 1))
		return -EINVAL;

	if (mv_pp2x_range_validate(hash_idx, 0, MVPP2_CLS_C3_HASH_TBL_SIZE - 1))
		return -EINVAL;

	/* add or update hash index table */
	inst->cls_db->c3_db.hash_idx_tbl[logic_idx].valid = MVPP2_C3_ENTRY_VALID;
	inst->cls_db->c3_db.hash_idx_tbl[logic_idx].hash_idx = hash_idx;

	/* add or update logical index table */
	inst->cls_db->c3_db.logic_idx_tbl[hash_idx].valid = MVPP2_C3_ENTRY_VALID;
	inst->cls_db->c3_db.logic_idx_tbl[hash_idx].logic_idx = logic_idx;

	return 0;
}

/*******************************************************************************
 * pp2_cls_db_c3_entry_del()
 *
 * DESCRIPTION: Delete C3 entry to DB.
 *
 * INPUTS:
 *	inst   - packet processor instance
 *	logic_idx - logical index
 *
 * OUTPUTS:
 *	None
 *
 * RETURN:
 *	0 on success, error-code otherwise
 ******************************************************************************/
int pp2_cls_db_c3_entry_del(struct pp2_inst *inst, int logic_idx)
{
	u32 hash_idx;
	struct pp2_cls_c3_hash_index_entry_t *p_hash_entry = NULL;
	struct pp2_cls_c3_logic_index_entry_t *p_logic_entry = NULL;

	if (mv_pp2x_range_validate(logic_idx, 0, MVPP2_CLS_C3_HASH_TBL_SIZE - 1))
		return -EINVAL;

	/* get hash index entry */
	p_hash_entry = &inst->cls_db->c3_db.hash_idx_tbl[logic_idx];
	if (p_hash_entry->valid == MVPP2_C3_ENTRY_VALID) {
		/* clear hash entry */
		hash_idx = p_hash_entry->hash_idx;
		p_hash_entry->valid = MVPP2_C3_ENTRY_INVALID;
		p_hash_entry->hash_idx = MVPP2_C3_INVALID_ENTRY_NUM;

		/* get logical index entry by hash index and clear it */
		if (mv_pp2x_range_validate(hash_idx, 0, MVPP2_CLS_C3_HASH_TBL_SIZE - 1))
			return -EINVAL;
		p_logic_entry = &inst->cls_db->c3_db.logic_idx_tbl[hash_idx];
		p_logic_entry->valid = MVPP2_C3_ENTRY_INVALID;
		p_logic_entry->logic_idx = MVPP2_C3_INVALID_ENTRY_NUM;
	} else {
		pr_err("hash entry is invalid, do not need to delete it\n");
	}

	return 0;
}

/*******************************************************************************
 * pp2_cls_db_c3_hash_idx_get()
 *
 * DESCRIPTION: Get C3 multihash index by logical index.
 *
 * INPUTS:
 *	inst   - packet processor instance
 *	logic_idx - logical index
 *
 * OUTPUTS:
 *	hash_idx  - multihash index
 *
 * RETURN:
 *	0 on success, error-code otherwise
 ******************************************************************************/
int pp2_cls_db_c3_hash_idx_get(struct pp2_inst *inst, u32 logic_idx, u32 *hash_idx)
{
	struct pp2_cls_c3_hash_index_entry_t *p_hash_entry = NULL;

	if (mv_pp2x_range_validate(logic_idx, 0, MVPP2_CLS_C3_HASH_TBL_SIZE - 1))
		return -EINVAL;

	if (mv_pp2x_ptr_validate(hash_idx))
		return -EINVAL;

	/* get hash index entry */
	p_hash_entry = &inst->cls_db->c3_db.hash_idx_tbl[logic_idx];
	if (p_hash_entry->valid == MVPP2_C3_ENTRY_VALID) {
		*hash_idx = p_hash_entry->hash_idx;
		return 0;
	} else {
		return -EIO;
	}
	return 0;
}

/*******************************************************************************
 * pp2_cls_db_c3_logic_idx_get()
 *
 * DESCRIPTION: Get the multihash index.
 *
 * INPUTS:
 *	inst      - packet processor instance
 *	hash_idx  - multihash index
 *
 * OUTPUTS:
 *	logic_idx - logical index
 *
 * RETURN:
 *	0 on success, error-code otherwise
 ******************************************************************************/
int pp2_cls_db_c3_logic_idx_get(struct pp2_inst *inst, int hash_idx, int *logic_idx)
{
	struct pp2_cls_c3_logic_index_entry_t *p_logic_entry = NULL;

	if (mv_pp2x_range_validate(hash_idx, 0, MVPP2_CLS_C3_HASH_TBL_SIZE - 1))
		return -EINVAL;

	if (mv_pp2x_ptr_validate(logic_idx))
		return -EINVAL;

	/* get hash index entry */
	p_logic_entry = &inst->cls_db->c3_db.logic_idx_tbl[hash_idx];
	if (p_logic_entry->valid == MVPP2_C3_ENTRY_VALID) {
		*logic_idx = p_logic_entry->logic_idx;
		return 0;
	} else {
		return -EIO;
	}

	return 0;
}

/*******************************************************************************
 * pp2_cls_db_c3_hash_idx_update()
 *
 * DESCRIPTION: Update the multihash index.
 *
 * INPUTS:
 *	inst      - packet processor instance
 *	hash_pair_arr  - multihash modification array
 *
 * OUTPUTS:
 *	None
 *
 * RETURN:
 *	0 on success, error-code otherwise
 ******************************************************************************/
int pp2_cls_db_c3_hash_idx_update(struct pp2_inst *inst, struct pp2_cls_c3_hash_pair *hash_pair_arr)
{
	int idx;
	u32 old_idx;
	u32 new_idx;
	u32 logic_idx;
	struct pp2_cls_c3_hash_index_entry_t *p_hash_entry = NULL;
	struct pp2_cls_c3_logic_index_entry_t *p_logic_entry = NULL;

	if (mv_pp2x_ptr_validate(hash_pair_arr))
		return -EINVAL;

	/* update the multihash mapping in loop */
	for (idx = 0; idx < hash_pair_arr->pair_num; idx++) {
		old_idx = hash_pair_arr->old_idx[idx];
		new_idx = hash_pair_arr->new_idx[idx];

		if (mv_pp2x_range_validate(old_idx, 0, MVPP2_CLS_C3_HASH_TBL_SIZE - 1))
			return -EINVAL;

		if (mv_pp2x_range_validate(new_idx, 0, MVPP2_CLS_C3_HASH_TBL_SIZE - 1))
			return -EINVAL;

		p_logic_entry = &inst->cls_db->c3_db.logic_idx_tbl[old_idx];
		if (p_logic_entry->valid == MVPP2_C3_ENTRY_INVALID) {
			pr_err("hash entry is invalid w/ index(%d)\n", old_idx);
			return MV_ERROR;
		}

		logic_idx = p_logic_entry->logic_idx;

		if (mv_pp2x_range_validate(logic_idx, 0, MVPP2_CLS_C3_HASH_TBL_SIZE - 1))
			return -EINVAL;

		/* update logical index table */
		p_logic_entry->valid = MVPP2_C3_ENTRY_INVALID;
		p_logic_entry->logic_idx = MVPP2_C3_INVALID_ENTRY_NUM;
		p_logic_entry = &inst->cls_db->c3_db.logic_idx_tbl[new_idx];
		p_logic_entry->valid = MVPP2_C3_ENTRY_VALID;
		p_logic_entry->logic_idx = logic_idx;

		/* update hash index table */
		p_hash_entry = &inst->cls_db->c3_db.hash_idx_tbl[logic_idx];
		p_hash_entry->valid = MVPP2_C3_ENTRY_VALID;
		p_hash_entry->hash_idx = new_idx;
	}

	return 0;
}

/*******************************************************************************
 * pp2_cls_db_c3_scan_param_set()
 *
 * DESCRIPTION: set scan parameters.
 *
 * INPUTS:
 *	inst      - packet processor instance
 *	scan_config  - scan configuration parameters
 *
 * OUTPUTS:
 *	None
 *
 * RETURN:
 *	0 on success, error-code otherwise
 ******************************************************************************/
int pp2_cls_db_c3_scan_param_set(struct pp2_inst *inst, struct pp2_cls_c3_scan_config_t *scan_config)
{
	struct pp2_cls_c3_scan_config_t *p_scan_config = NULL;

	if (mv_pp2x_ptr_validate(scan_config))
		return -EINVAL;

	p_scan_config = &inst->cls_db->c3_db.scan_config;
	memcpy(p_scan_config, scan_config, sizeof(struct pp2_cls_c3_scan_config_t));

	return 0;
}

/*******************************************************************************
 * pp2_cls_db_c3_scan_param_get()
 *
 * DESCRIPTION: get scan parameters.
 *
 * INPUTS:
 *	inst      - packet processor instance
 *
 * OUTPUTS:
 *	can_config  - scan configuration parameters
 *
 * RETURN:
 *	0 on success, error-code otherwise
 ******************************************************************************/
int pp2_cls_db_c3_scan_param_get(struct pp2_inst *inst, struct pp2_cls_c3_scan_config_t *scan_config)
{
	struct pp2_cls_c3_scan_config_t *p_scan_config = NULL;

	if (mv_pp2x_ptr_validate(scan_config))
		return -EINVAL;

	p_scan_config = &inst->cls_db->c3_db.scan_config;
	memcpy(scan_config, p_scan_config, sizeof(struct pp2_cls_c3_scan_config_t));

	return 0;
}

/*******************************************************************************
 * pp2_cls_db_c3_search_depth_set()
 *
 * DESCRIPTION: set cuckoo search depth.
 *
 * INPUTS:
 *	inst      - packet processor instance
 *	search_depth  - cuckoo search depth
 *
 * OUTPUTS:
 *	None
 *
 * RETURN:
 *	0 on success, error-code otherwise
 ******************************************************************************/
int pp2_cls_db_c3_search_depth_set(struct pp2_inst *inst, u32 search_depth)
{
	inst->cls_db->c3_db.max_search_depth = search_depth;
	return 0;
}

/*******************************************************************************
 * pp2_cls_db_c3_search_depth_get()
 *
 * DESCRIPTION: get cuckoo search depth.
 *
 * INPUTS:
 *	inst      - packet processor instance
 *
 * OUTPUTS:
 *	search_depth  - cuckoo search depth
 *
 * RETURN:
 *	0 on success, error-code otherwise
 ******************************************************************************/
int pp2_cls_db_c3_search_depth_get(struct pp2_inst *inst, u32 *search_depth)
{
	if (mv_pp2x_ptr_validate(search_depth))
		return -EINVAL;

	*search_depth = inst->cls_db->c3_db.max_search_depth;

	return 0;
}

/*******************************************************************************
 * pp2_cls_db_c3_init()
 *
 * DESCRIPTION: Perform DB Initialization for C3 engine.
 *
 * INPUTS:
 *	inst      - packet processor instance
 *
 * OUTPUTS:
 *	None.
 *
 * RETURN:
 *	0 on success, error-code otherwise
 ******************************************************************************/
int pp2_cls_db_c3_init(struct pp2_inst *inst)
{
	int idx;

	if (!inst->cls_db)
		return -EINVAL;

	/* Clear C3 db */
	memset(&inst->cls_db->c3_db, 0, sizeof(struct pp2_cls_db_c3_t));

	/* Init C3 multihash index table and logical index table */
	for (idx = 0; idx < MVPP2_CLS_C3_HASH_TBL_SIZE; idx++) {
		inst->cls_db->c3_db.hash_idx_tbl[idx].valid = MVPP2_C3_ENTRY_INVALID;
		inst->cls_db->c3_db.logic_idx_tbl[idx].valid = MVPP2_C3_ENTRY_INVALID;
	}

	return 0;
}

/*******************************************************************************
 * pp2_cls_db_init()
 *
 * DESCRIPTION: Perform DB Initialization.
 *
 * INPUTS:
 *	inst      - packet processor instance
 *
 * OUTPUTS:
 *	None.
 *
 * RETURN:
 *	0 on success, error-code otherwise
 ******************************************************************************/
int pp2_cls_db_init(struct pp2_inst *inst)
{
	int ret_code;

	/* Each database can be initialized only once */
	if (inst->cls_db) {
		pr_err("Classifier database alraedy initialized.");
		return -EINVAL;
	}

	/* Allocation for pp2_cls db */
	ret_code = pp2_cls_db_mem_alloc_init(inst);

	if (ret_code != 0) {
		pr_err("Failed to allocate memory for PP2_CLS DB\n");
		return -ENOMEM;
	}

	return 0;
}

/*******************************************************************************
 * pp2_cls_db_exit()
 *
 * DESCRIPTION: Perform DB memory free when exit.
 *
 * INPUTS:
 *	inst      - packet processor instance
 *
 * OUTPUTS:
 *	None.
 *
 * RETURN:
 *	0 on success, error-code otherwise
 ******************************************************************************/
int pp2_cls_db_exit(struct pp2_inst *inst)
{
	int ret_code;

	ret_code = pp2_cls_db_mem_free(inst);
	if (ret_code != 0) {
		pr_err("Failed to free memory allocated for PP2_CLS DB\n");
		return -ENOMEM;
	}

	return 0;
}

/*******************************************************************************
 * pp2_db_cls_fl_ctrl_set
 *
 * DESCRIPTION: The routine sets the CLS control structure
 *
 * INPUTS:
 *	inst      - packet processor instance
 *	fl_ctrl - flow control structure
 *
 * OUTPUTS:
 *	None
 *
 * RETURN:
 *	0 on success, error-code otherwise
 ******************************************************************************/
int pp2_db_cls_fl_ctrl_set(struct pp2_inst *inst, struct pp2_db_cls_fl_ctrl_t *fl_ctrl)
{
	if (!fl_ctrl) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}

	 memcpy(&inst->cls_db->cls_db.fl_ctrl, fl_ctrl, sizeof(struct pp2_db_cls_fl_ctrl_t));

	return 0;
}

/*******************************************************************************
 * pp2_db_cls_fl_ctrl_get
 *
 * DESCRIPTION: The routine gets the CLS control structure
 *
 * INPUTS:
 *	inst      - packet processor instance
 *
 * OUTPUTS:
 *	fl_ctrl - flow control structure
 *
 * RETURN:
 *	0 on success, error-code otherwise
 ******************************************************************************/
int pp2_db_cls_fl_ctrl_get(struct pp2_inst *inst, struct pp2_db_cls_fl_ctrl_t *fl_ctrl)
{
	if (!fl_ctrl) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}

	 memcpy(fl_ctrl, &inst->cls_db->cls_db.fl_ctrl, sizeof(struct pp2_db_cls_fl_ctrl_t));

	 return 0;
}

/*******************************************************************************
 * pp2_db_cls_fl_rule_set
 *
 * DESCRIPTION: The routine sets a single rule in a rule flow
 *
 * INPUTS:
 *	inst      - packet processor instance
 *	off - the rule offset
 *	fl_rule - the flow the rule is in
 *
 * OUTPUTS:
 *	None
 *
 * RETURN:
 *	0 on success, error-code otherwise
 ******************************************************************************/
int pp2_db_cls_fl_rule_set(struct pp2_inst *inst, u32 off, struct pp2_db_cls_fl_rule_t *fl_rule)
{
	if (!fl_rule) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}

	if (off >= MVPP2_FLOW_TBL_SIZE) {
		pr_err("Invalid parameter\n");
		return -EINVAL;
	}
	memcpy(&inst->cls_db->cls_db.fl_rule[off], fl_rule, sizeof(struct pp2_db_cls_fl_rule_t));

	return 0;
}

/*******************************************************************************
 * pp2_db_cls_fl_rule_get
 *
 * DESCRIPTION: The routine gets a single rule in a rule flow
 *
 * INPUTS:
 *	inst      - packet processor instance
 *	off - the rule offset
 *
 * OUTPUTS:
 *	fl_rule - the flow the rule is in
 *
 * RETURN:
 *	0 on success, error-code otherwise
 ******************************************************************************/
int pp2_db_cls_fl_rule_get(struct pp2_inst *inst, u32 off, struct pp2_db_cls_fl_rule_t *fl_rule)
{
	if (!fl_rule) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}

	if (off >= MVPP2_FLOW_TBL_SIZE) {
		pr_err("Invalid parameter\n");
		return -EINVAL;
	}
	memcpy(fl_rule, &inst->cls_db->cls_db.fl_rule[off], sizeof(struct pp2_db_cls_fl_rule_t));

	return 0;
}

/*******************************************************************************
 * pp2_db_cls_fl_rule_list_get
 *
 * DESCRIPTION: The routine gets a whole rule flow
 *
 * INPUTS:
 *	inst      - packet processor instance
 *	off - the first rule offset
 *	len - the flow length
 *
 * OUTPUTS:
 *	fl_rl_list - the flow the rules are in
 *
 * RETURN:
 *	0 on success, error-code otherwise
 ******************************************************************************/
int pp2_db_cls_fl_rule_list_get(struct pp2_inst *inst, u32 off, u32 len,
				struct pp2_db_cls_fl_rule_t *fl_rl_list)
{
	if (!fl_rl_list) {
		pr_err("%s: null pointer.\n", __func__);
		return -EFAULT;
	}

	if (off >= MVPP2_FLOW_TBL_SIZE ||
	    off + len >= MVPP2_FLOW_TBL_SIZE) {
		pr_err("requested rule list too big [offset=%d length=%d]\n", off, len);
		return -EINVAL;
	}

	memcpy(fl_rl_list, &inst->cls_db->cls_db.fl_rule[off],
	       sizeof(struct pp2_db_cls_fl_rule_t) * len);

	return 0;
}

/*******************************************************************************
 * pp2_db_cls_lkp_dcod_set
 *
 * DESCRIPTION: The routine sets a lookup decode entry
 *
 * INPUTS:
 *	inst      - packet processor instance
 *	fl_log_id - index of the entry (logical flow ID)
 *	lkp_dcod - the lookup decode entry structure
 *
 * OUTPUTS:
 *	None
 *
 * RETURN:
 *	0 on success, error-code otherwise
 ******************************************************************************/
int pp2_db_cls_lkp_dcod_set(struct pp2_inst *inst, u32 fl_log_id, struct pp2_db_cls_lkp_dcod_t *lkp_dcod)
{
	if (!lkp_dcod) {
		pr_err("%s: null pointer.\n", __func__);
		return -EFAULT;
	}

	if (fl_log_id >= MVPP2_MNG_FLOW_ID_MAX) {
		pr_err("Invalid parameter\n");
		return -EINVAL;
	}

	memcpy(&inst->cls_db->cls_db.lkp_dcod[fl_log_id], lkp_dcod, sizeof(struct pp2_db_cls_lkp_dcod_t));

	return 0;
}

/*******************************************************************************
 * pp2_db_cls_lkp_dcod_get
 *
 * DESCRIPTION: The routine gets a lookup decode entry
 *
 * INPUTS:
 *	inst      - packet processor instance
 *	fl_log_id - index of the entry (logical flow ID)
 *
 * OUTPUTS:
 *	lkp_dcod - the lookup decode entry structure
 *
 * RETURN:
 *	0 on success, error-code otherwise
 ******************************************************************************/
int pp2_db_cls_lkp_dcod_get(struct pp2_inst *inst, u32 fl_log_id, struct pp2_db_cls_lkp_dcod_t *lkp_dcod)
{
	if (!lkp_dcod) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}

	if (fl_log_id >= MVPP2_MNG_FLOW_ID_MAX) {
		pr_err("Invalid parameter\n");
		return -EINVAL;
	}
	memcpy(lkp_dcod, &inst->cls_db->cls_db.lkp_dcod[fl_log_id], sizeof(struct pp2_db_cls_lkp_dcod_t));

	return 0;
}

/*******************************************************************************
 * pp2_db_cls_rl_off_lkp_dcod_get
 *
 * DESCRIPTION: The routine returns the lookup decode entry for a flow rule
 *
 * INPUTS:
 *	inst      - packet processor instance
 *	rl_off - flow rule offset to search
 *
 * OUTPUTS:
 *	lkp_dcod - the lookup decode entry structure
 *
 * RETURN:
 *	0 on success, error-code otherwise
 ******************************************************************************/
int pp2_db_cls_rl_off_lkp_dcod_get(struct pp2_inst *inst, u16 rl_off, struct pp2_db_cls_lkp_dcod_t *lkp_dcod)
{
	u32 i;
	struct pp2_db_cls_lkp_dcod_t *p_lkp_dcod;

	if (!lkp_dcod) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}

	if (rl_off >= MVPP2_FLOW_TBL_SIZE) {
		pr_err("Invalid parameter\n");
		return -EINVAL;
	}

	for (i = 0; i < MVPP2_MNG_FLOW_ID_MAX; i++) {
		p_lkp_dcod = &inst->cls_db->cls_db.lkp_dcod[i];
		if (p_lkp_dcod->flow_off <= rl_off && p_lkp_dcod->flow_off + p_lkp_dcod->flow_len  >= rl_off)
			break;
	}

	if (i == MVPP2_MNG_FLOW_ID_MAX) {
		pr_err("rule offset [%d] not found\n", rl_off);
		return -EINVAL;
	}

	memcpy(lkp_dcod, &inst->cls_db->cls_db.lkp_dcod[i], sizeof(struct pp2_db_cls_lkp_dcod_t));

	return 0;
}

/*******************************************************************************
 * pp2_db_cls_rl_off_free_nr
 *
 * DESCRIPTION: The routine returns the number of free logical rule entries
 *
 * INPUTS:
 *	inst      - packet processor instance
 *
 * OUTPUTS:
 *	free_nr - number of free logical rule entries
 *
 * RETURN:
 *	0 on success, error-code otherwise
 ******************************************************************************/
int pp2_db_cls_rl_off_free_nr(struct pp2_inst *inst, u32 *free_nr)
{
	if (!free_nr) {
		pr_err("%s: null pointer.\n", __func__);
		return -EFAULT;
	}

	*free_nr = ((MVPP2_CLS_LOG2OFF_TBL_SIZE) - inst->cls_db->cls_db.log2off[MVPP2_CLS_FREE_LOG2OFF]);

	return 0;
}

/*******************************************************************************
 * pp2_db_cls_rl_off_free_set
 *
 * DESCRIPTION: The routine allocates a new logical rule number and assignes it with offset
 *
 * INPUTS:
 *	inst      - packet processor instance
 *	off - the offset the rule is at
 *
 * OUTPUTS:
 *	log - the new logical rule number
 *
 * RETURN:
 *	0 on success, error-code otherwise
 ******************************************************************************/
int pp2_db_cls_rl_off_free_set(struct pp2_inst *inst, u16 off, u16 *log)
{
	if (!log) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}

	if (off >= MVPP2_FLOW_TBL_SIZE) {
		pr_err("Invalid parameter\n");
		return -EINVAL;
	}

	if ((MVPP2_CLS_LOG2OFF_TBL_SIZE - inst->cls_db->cls_db.log2off[MVPP2_CLS_FREE_LOG2OFF]) == 0)
		return -EINVAL;

	*log = inst->cls_db->cls_db.log2off[MVPP2_CLS_FREE_LOG2OFF];

	inst->cls_db->cls_db.log2off[*log] = off;

	inst->cls_db->cls_db.log2off[MVPP2_CLS_FREE_LOG2OFF]++;

	return 0;
}

/*******************************************************************************
 * pp2_db_cls_rl_off_get
 *
 * DESCRIPTION: The routine returns the offset of a rule according to logical rule number
 *
 * INPUTS:
 *	inst      - packet processor instance
 *	log - the new logical rule number
 *
 * OUTPUTS:
 *	off - the offset the rule is at
 *
 * RETURN:
 *	0 on success, error-code otherwise
 ******************************************************************************/
int pp2_db_cls_rl_off_get(struct pp2_inst *inst, u16 *off, u16 log)
{
	if (!off) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}

	if (log > MVPP2_CLS_LOG2OFF_TBL_SIZE) {
		pr_err("Invalid parameter\n");
		return -EFAULT;
	}

	if (inst->cls_db->cls_db.log2off[log] == MVPP2_CLS_FREE_FL_LOG)
		return -EINVAL;

	*off = inst->cls_db->cls_db.log2off[log];

	return 0;
}

/*******************************************************************************
 * pp2_db_cls_rl_off_set
 *
 * DESCRIPTION: The routine updates a logical rule with a new offset
 *
 * INPUTS:
 *	inst      - packet processor instance
 *	off - the new offset the rule is at
 *	log - the logical rule number
 *
 * OUTPUTS:
 *	None
 *
 * RETURN:
 *	0 on success, error-code otherwise
 ******************************************************************************/
int pp2_db_cls_rl_off_set(struct pp2_inst *inst, u16 off, u16 log)
{
	if (log > MVPP2_CLS_LOG2OFF_TBL_SIZE) {
		pr_err("Invalid parameter\n");
		return -EINVAL;
	}
	inst->cls_db->cls_db.log2off[log] = off;

	return 0;
}

/*******************************************************************************
 * pp2_db_cls_init
 *
 * DESCRIPTION: The routine initializes the CLS DBs
 *
 ******************************************************************************/
void pp2_db_cls_init(struct pp2_inst *inst)
{
	int i;

	/* set the CLS control to default values */
	memset(&inst->cls_db->cls_db, 0, sizeof(inst->cls_db->cls_db));

	/* f_start = 1 for kernel alignment */
	inst->cls_db->cls_db.fl_ctrl.f_start = 1;
	inst->cls_db->cls_db.fl_ctrl.f_end = MVPP2_FLOW_TBL_SIZE - 1;

	for (i = MVPP2_CLS_LOG2OFF_START; i < MVPP2_CLS_LOG2OFF_TBL_SIZE; i++)
		inst->cls_db->cls_db.log2off[i] = MVPP2_CLS_FREE_FL_LOG;

	inst->cls_db->cls_db.log2off[MVPP2_CLS_FREE_LOG2OFF] = MVPP2_CLS_LOG2OFF_START;
}

/*******************************************************************************
 * pp2_cls_db_c2_lkp_type_list_head_get()
 *
 * DESCRIPTION: Get the head of the lookup type list.
 *
 * INPUTS:
 *	inst      - packet processor instance
 *          lkp_type  - C2 lookup type
 *
 * OUTPUTS: None.
 *
 * RETURNS:
 *          The head pointer of list.
 *
 ******************************************************************************/
struct list *pp2_cls_db_c2_lkp_type_list_head_get(struct pp2_inst *inst,
						  u8 lkp_type)
{
	return &inst->cls_db->c2_db.c2_lu_type_head_db[lkp_type];
}

/*******************************************************************************
 * pp2_cls_db_c2_free_list_head_get()
 *
 * DESCRIPTION: Get the head of free list.
 *
 * INPUTS:
 *	inst      - packet processor instance
 *
 * OUTPUTS: None.
 *
 * RETURNS:
 *          The head pointer of list.
 *
 ******************************************************************************/
struct list *pp2_cls_db_c2_free_list_head_get(struct pp2_inst *inst)
{
	return &inst->cls_db->c2_db.c2_free_head_db;
}

/*******************************************************************************
 * pp2_cls_db_c2_index_node_get()
 *
 * DESCRIPTION: Get the index node in the list according to their db index.
 *
 * INPUTS:
 *	inst      - packet processor instance
 *          c2_node_idx  - the db index of C2 index node.
 *
 * OUTPUTS: None.
 *
 * RETURNS:
 *          The node pointer.
 *
 ******************************************************************************/
struct pp2_cls_c2_index_t *pp2_cls_db_c2_index_node_get(struct pp2_inst *inst,
							u32 c2_node_idx)
{
	/* Para check */
	if (c2_node_idx >= MVPP2_C2_ENTRY_MAX) {
		pr_err("Invalid parameter\n");
		return NULL;
	}
	return &inst->cls_db->c2_db.c2_index_db[c2_node_idx];
}

/*******************************************************************************
 * pp2_cls_db_c2_index_node_set()
 *
 * DESCRIPTION: Set the index node in the list according to their db index.
 *
 * INPUTS:
 *	inst      - packet processor instance
 *          c2_node_idx   - the db index of C2 index node.
 *          c2_index_node - the new value of the node to set.
 *
 * OUTPUTS: None.
 *
 * RETURN:
 *	0 on success, error-code otherwise
 ******************************************************************************/
int pp2_cls_db_c2_index_node_set(struct pp2_inst *inst, u32 c2_node_idx,
				 struct pp2_cls_c2_index_t *c2_index_node)
{
	/* Param check */
	if (c2_node_idx >= MVPP2_C2_ENTRY_MAX) {
		pr_err("Invalid parameter\n");
		return -EINVAL;
	}

	if (!c2_index_node) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}

	memcpy(&inst->cls_db->c2_db.c2_index_db[c2_node_idx], c2_index_node, sizeof(struct pp2_cls_c2_index_t));

	return 0;
}

/*******************************************************************************
 * pp2_cls_db_c2_data_get()
 *
 * DESCRIPTION: Get the C2 entry data from DB according to their db index.
 *
 * INPUTS:
 *	inst      - packet processor instance
 *          c2_node_idx   - the db index of C2 entry data.
 *
 * OUTPUTS:
 *          c2_data       - C2 entry data.
 *
 * RETURN:
 *	0 on success, error-code otherwise
 ******************************************************************************/
int pp2_cls_db_c2_data_get(struct pp2_inst *inst, u32 c2_db_idx,
			   struct pp2_cls_c2_data_t *c2_data)
{
	/* Param check */
	if (c2_db_idx > MVPP2_C2_LAST_ENTRY) {
		pr_err("Invalid parameter\n");
		return -EINVAL;
	}

	if (!c2_data) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}

	memcpy(c2_data, &inst->cls_db->c2_db.c2_data_db[c2_db_idx], sizeof(struct pp2_cls_c2_data_t));

	return 0;
}

/*******************************************************************************
 * pp2_cls_db_c2_data_set()
 *
 * DESCRIPTION: Set the C2 entry data to DB according to their allocated db index.
 *
 * INPUTS:
 *	inst      - packet processor instance
 *          c2_node_idx   - the db index of C2 entry data.
 *          c2_data       - C2 entry data.
 *
 * OUTPUTS:
 *          None.
 *
 * RETURN:
 *	0 on success, error-code otherwise
 ******************************************************************************/
int pp2_cls_db_c2_data_set(struct pp2_inst *inst, u32 c2_db_idx,
			   struct pp2_cls_c2_data_t *c2_data)
{
	/* Param check */
	if (c2_db_idx > MVPP2_C2_LAST_ENTRY) {
		pr_err("Invalid parameter\n");
		return -EINVAL;
	}

	if (!c2_data) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}

	memcpy(&inst->cls_db->c2_db.c2_data_db[c2_db_idx], c2_data, sizeof(struct pp2_cls_c2_data_t));

	return 0;
}

/*******************************************************************************
 * pp2_cls_db_c2_init()
 *
 * DESCRIPTION: Perform DB Initialization for C2 section.
 *
 * INPUTS:
 *	inst      - packet processor instance
 *
 * OUTPUTS: None.
 *
 * RETURN:
 *	0 on success, error-code otherwise
 ******************************************************************************/
int pp2_cls_db_c2_init(struct pp2_inst *inst)
{
	int i;

	/* Clear C2 db */
	memset(&inst->cls_db->c2_db, 0, sizeof(struct pp2_cls_db_c2_t));

	/* Init c2_hw_idx to C2 corresponding c2 hw index in c2 data db and index db */
	for (i = 0; i < MVPP2_C2_ENTRY_MAX - MVPP2_C2_FIRST_ENTRY; i++) {
		inst->cls_db->c2_db.c2_data_db[i].valid = MVPP2_C2_ENTRY_INVALID;
		inst->cls_db->c2_db.c2_index_db[i].valid = MVPP2_C2_ENTRY_INVALID;
		inst->cls_db->c2_db.c2_index_db[i].c2_hw_idx = i + MVPP2_C2_FIRST_ENTRY;
	}

	/* Init C2 list head */
	INIT_LIST(&inst->cls_db->c2_db.c2_free_head_db);
	for (i = 0; i < MVPP2_C2_LKP_TYPE_MAX; i++)
		INIT_LIST(&inst->cls_db->c2_db.c2_lu_type_head_db[i]);

	/* Init free list, last entry is used for default entry, always not available */
	for (i = MVPP2_C2_LAST_ENTRY - MVPP2_C2_FIRST_ENTRY - 1; i >= 0; i--) {
		list_add(&inst->cls_db->c2_db.c2_index_db[i].list_node, &inst->cls_db->c2_db.c2_free_head_db);
		/* Change index node valid status after adding to free list */
		inst->cls_db->c2_db.c2_index_db[i].valid = MVPP2_C2_ENTRY_VALID;
	}
	/* Reserve the last one for default miss entry */
	inst->cls_db->c2_db.c2_index_db[MVPP2_C2_LAST_ENTRY - MVPP2_C2_FIRST_ENTRY].valid = MVPP2_C2_ENTRY_VALID;
	inst->cls_db->c2_db.c2_data_db[MVPP2_C2_LAST_ENTRY - MVPP2_C2_FIRST_ENTRY].valid = MVPP2_C2_ENTRY_VALID;

	return 0;
}
