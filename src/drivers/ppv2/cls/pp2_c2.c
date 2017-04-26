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
 * @file pp2_c2.c
 *
 * C3 High level routines
 */

/***********************/
/* c file declarations */
/***********************/
#include "std_internal.h"

#include "../pp2_types.h"
#include "../pp2.h"
#include "../pp2_hw_type.h"
#include "../pp2_hw_cls.h"
#include "../pp2_hw_cls_dbg.h"
#include "pp2_c2.h"

/*******************************************************************************
 * pp2_cls_c2_new_logic_idx_allocate()
 *
 * DESCRIPTION: This routine will allocate logical index for one c2 rule entry
 *
 * INPUTS:
 *          reset   - indeicate it is reset or not
 *
 * RETURNS:
 *          logical index allocated or after reset
 ******************************************************************************/
static u32 pp2_cls_c2_new_logic_idx_allocate(bool reset)
{
	/* Base logical index for C2 entry, every time a new index is allocated, perform pp2_cls_c2_logic_base_idx++ */
	static u32 pp2_cls_c2_logic_base_idx = MVPP2_C2_LOGIC_IDX_BASE;

	pp2_cls_c2_logic_base_idx++;
	/* if c2_reset, restore pp2_cls_c2_logic_base_idx to original value */
	if (reset)
		pp2_cls_c2_logic_base_idx = MVPP2_C2_LOGIC_IDX_BASE;

	return pp2_cls_c2_logic_base_idx;
}

/*******************************************************************************
 * pp2_cls_c2_free_list_add()
 *
 * DESCRIPTION: This routine will add one C2 TCAM to free list
 *
 * INPUTS:
 *          inst        - packet processor instance
 *          c2_hw_idx   - the free C2 TCAM entry
 *
 * RETURNS:
 *	0 on success, error-code otherwise
 ******************************************************************************/
static int pp2_cls_c2_free_list_add(struct pp2_inst *inst, u32 c2_hw_idx)
{
	struct list *free_list_head;
	struct pp2_cls_c2_index_t *c2_index_node, *temp_node;
	u32 index;
	bool bigger_found = false;

	/* Get list head */
	free_list_head = pp2_cls_db_c2_free_list_head_get(inst);

	/* Check hw index already exist or not */
	LIST_FOR_EACH_OBJECT(temp_node, struct pp2_cls_c2_index_t, free_list_head, list_node) {
		if (temp_node->c2_hw_idx == c2_hw_idx)
			return 0;
	}

	/* Get the invalid index node */
	for (index = 0; index < MVPP2_C2_ENTRY_MAX; index++) {
		c2_index_node = pp2_cls_db_c2_index_node_get(inst, index);
		if (!c2_index_node)
			return -ENXIO;
		if (c2_index_node->valid == MVPP2_C2_ENTRY_INVALID)
			break;
	}
	if (index == MVPP2_C2_ENTRY_MAX)
		return -ENXIO;

	c2_index_node->c2_hw_idx = c2_hw_idx;

	/* Check free list empty */
	if (list_is_empty(free_list_head)) {
		list_add(&c2_index_node->list_node, free_list_head);
	} else {
		/* Add to free list */
		LIST_FOR_EACH_OBJECT(temp_node, struct pp2_cls_c2_index_t, free_list_head, list_node) {
			if (temp_node->c2_hw_idx > c2_hw_idx) {
				bigger_found = true;
				list_add_to_tail(&c2_index_node->list_node, &temp_node->list_node);
				break;
			}
		}
		if (!bigger_found)
			list_add_to_tail(&c2_index_node->list_node, free_list_head);
	}

	/* Change Valid status to valid */
	c2_index_node->valid = MVPP2_C2_ENTRY_VALID;

	return 0;
}

/*******************************************************************************
 * pp2_cls_c2_data_entry_db_add()
 *
 * DESCRIPTION: Add HW TCAM entry data in data DB.
 *
 * INPUTS:
 *          inst       - packet processor instance
 *          c2_entry   - c2 TCAM entry data to record
 *
 * OUTPUTS:
 *          c2_db_idx  - the data db entry index
 *
 * RETURNS:
 *	0 on success, error-code otherwise
 ******************************************************************************/
static int pp2_cls_c2_data_entry_db_add(struct pp2_inst *inst,
					struct mv_pp2x_c2_add_entry *c2_entry,
					u32 *c2_db_idx)
{
	int ret_code;
	struct pp2_cls_c2_data_t *c2_entry_db;	/*use heap to reduce stack size*/
	u32 index;

	if (!c2_entry) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}
	if (!c2_db_idx) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}

	c2_entry_db = kmalloc(sizeof(*c2_entry_db), GFP_KERNEL);
	if (!c2_entry_db)
		return -ENOMEM;

	memset(c2_entry_db, 0, sizeof(struct pp2_cls_c2_data_t));

	/* Get available db entry for c2 entry */
	for (index = 0; index < MVPP2_C2_ENTRY_MAX; index++) {
		ret_code = pp2_cls_db_c2_data_get(inst, index, c2_entry_db);
		if (ret_code) {
			pr_err("recvd ret_code(%d)\n", ret_code);
			free(c2_entry_db);
			return ret_code;
		}
		if (c2_entry_db->valid == MVPP2_C2_ENTRY_INVALID)
			break;
	}
	if (index == MVPP2_C2_ENTRY_MAX) {
		pr_err("No free space in DB for C2 entry\n");
		free(c2_entry_db);
		return -EINVAL;
	}

	/* record c2 entry to DB */
	memcpy(&c2_entry_db->port, &c2_entry->port, sizeof(struct mv_pp2x_src_port));
	c2_entry_db->lkp_type = c2_entry->lkp_type;
	c2_entry_db->lkp_type_mask = c2_entry->lkp_type_mask;
	c2_entry_db->priority = c2_entry->priority;
	c2_entry_db->field_bm = c2_entry->mng_pkt_key->pkt_key->field_bm;
	/* pkt Key */
	c2_entry_db->mng_pkt_key.ttl = c2_entry->mng_pkt_key->ttl;
	c2_entry_db->mng_pkt_key.tcp_flag = c2_entry->mng_pkt_key->tcp_flag;
	c2_entry_db->mng_pkt_key.tcp_flag_mask = c2_entry->mng_pkt_key->tcp_flag_mask;
	memcpy(&c2_entry_db->mng_pkt_key.pkt_key, c2_entry->mng_pkt_key->pkt_key, sizeof(struct pp2_cls_pkt_key_t));
	/* Qos */
	memcpy(&c2_entry_db->qos_info, &c2_entry->qos_info, sizeof(struct mv_pp2x_engine_qos_info));
	memcpy(&c2_entry_db->action, &c2_entry->action, sizeof(struct mv_pp2x_engine_pkt_action));
	memcpy(&c2_entry_db->qos_value, &c2_entry->qos_value, sizeof(struct mv_pp2x_qos_value));
	memcpy(&c2_entry_db->pkt_mod, &c2_entry->pkt_mod, sizeof(struct mv_pp2x_engine_pkt_mod));
	memcpy(&c2_entry_db->flow_info, &c2_entry->flow_info, sizeof(struct mv_pp2x_duplicate_info));
	c2_entry_db->valid = MVPP2_C2_ENTRY_VALID;

	/* Write to db */
	ret_code = pp2_cls_db_c2_data_set(inst, index, c2_entry_db);
	if (ret_code) {
		pr_err("recvd ret_code(%d)\n", ret_code);
		free(c2_entry_db);
		return ret_code;
	}

	/* Return db index */
	*c2_db_idx = index;

	free(c2_entry_db);
	return 0;
}

/*******************************************************************************
 * pp2_cls_c2_data_entry_db_del()
 *
 * DESCRIPTION: Delete HW TCAM entry data record in data DB.
 *
 * INPUTS:
 *          inst       - packet processor instance
 *          c2_db_idx  - the data db entry index
 *
 * OUTPUTS:
 *          None
 *
 * RETURNS:
 *	0 on success, error-code otherwise
 * COMMENTS:
 *          None
 ******************************************************************************/
static int pp2_cls_c2_data_entry_db_del(struct pp2_inst *inst, u32 c2_db_idx)
{
	struct pp2_cls_c2_data_t *c2_db_data;		/*use heap to reduce stack size*/

	c2_db_data = kmalloc(sizeof(*c2_db_data), GFP_KERNEL);
	if (!c2_db_data)
		return -ENOMEM;

	memset(c2_db_data, 0, sizeof(struct pp2_cls_c2_data_t));

	if (pp2_cls_db_c2_data_set(inst, c2_db_idx, c2_db_data)) {
		free(c2_db_data);
		return -EINVAL;
	}

	free(c2_db_data);
	return 0;
}

/*******************************************************************************
 * pp2_cls_c2_lkp_type_list_pri_get()
 *
 * DESCRIPTION: Get the highest priority and lowest priority of the lookup type list.
 *
 * INPUTS:
 *	   inst	      - packet processor instance
 *          lkp_type   - lookup type of list
 *
 * OUTPUTS:
 *          hignest_pri  - highest priority
 *          lowest_pri   - lowest priority
 *
 * RETURNS:
 *	0 on success, error-code otherwise
 * COMMENTS:
 *          The list can not be empty.
 ******************************************************************************/
static int pp2_cls_c2_lkp_type_list_pri_get(struct pp2_inst *inst,
					    u8 lkp_type,
					    u32 *hignest_pri,
					    u32 *lowest_pri)
{
	struct list *lkp_type_list_head;
	struct pp2_cls_c2_index_t *c2_index_node;
	struct pp2_cls_c2_data_t *c2_entry_data;		/*use heap to reduce stack size*/

	/* param check */
	if (!hignest_pri) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}
	if (!lowest_pri) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}
	/* Get list head */
	lkp_type_list_head = pp2_cls_db_c2_lkp_type_list_head_get(inst, lkp_type);

	if (list_is_empty(lkp_type_list_head)) {
		pr_err("C2 engine lookup type list (%d) is empty\n", lkp_type);
		return -EFAULT;
	}

	/* Get first priority */
	c2_index_node = LIST_FIRST_OBJECT(lkp_type_list_head,
					  struct pp2_cls_c2_index_t,
					  list_node);

	c2_entry_data = kmalloc(sizeof(*c2_entry_data), GFP_KERNEL);
	if (!c2_entry_data)
		return -ENOMEM;

	memset(c2_entry_data, 0, sizeof(struct pp2_cls_c2_data_t));

	/* get C2 db entry data */
	if (pp2_cls_db_c2_data_get(inst, c2_index_node->c2_data_db_idx, c2_entry_data)) {
		free(c2_entry_data);
		return -EINVAL;
	}
	*hignest_pri = c2_entry_data->priority;
	*lowest_pri = c2_entry_data->priority;

	/* Search the list */
	LIST_FOR_EACH_OBJECT(c2_index_node, struct pp2_cls_c2_index_t, lkp_type_list_head, list_node) {
		if (pp2_cls_db_c2_data_get(inst, c2_index_node->c2_data_db_idx, c2_entry_data)) {
			free(c2_entry_data);
			return -EINVAL;
		}
		if ((*hignest_pri) > c2_entry_data->priority)
			*hignest_pri = c2_entry_data->priority;
		if ((*lowest_pri) < c2_entry_data->priority)
			*lowest_pri = c2_entry_data->priority;
	}

	free(c2_entry_data);
	return 0;
}

/*******************************************************************************
 * pp2_cls_c2_lkp_type_list_neighbour_pri_get()
 *
 * DESCRIPTION: The routine will get the neighbour priority of given priority.
 *
 * INPUTS:
 *	   inst	      - packet processor instance
 *          lkp_type   - lookup type of list
 *          priority   - interal priority.
 *          highest_pri- current highest priority of list
 *          lowest_pri - current highest priority of list
 *
 * OUTPUTS:
 *          pri_prev   - privious neighbour priority
 *          pri_next   - following(next) neighbour priority
 *
 * RETURNS:
 *	0 on success, error-code otherwise
 * COMMENTS:
 *          The list can not be empty, meet: priority > highest, priority < lowest
 ******************************************************************************/
static int pp2_cls_c2_lkp_type_list_neighbour_pri_get(struct pp2_inst *inst,
						      u32 lkp_type,
						      u32 priority,
						      u32 highest_pri,
						      u32 lowest_pri,
						      u32 *pri_prev,
						      u32 *pri_next)
{
	struct pp2_cls_c2_index_t *c2_index_node;
	struct pp2_cls_c2_data_t *c2_entry_data;		/*use heap to reduce stack size*/
	u32 pri_temp_h = 0, pri_temp_l = 0;

	/* para check */
	if (!pri_prev) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}
	if (!pri_next) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}
	if (priority < highest_pri || priority > lowest_pri) {
		pr_err("Invalid C2 internal priority %d\n", priority);
		return -EFAULT;
	}

	if (highest_pri == lowest_pri) {
		*pri_prev = MVPP2_C2_LKP_TYPE_INVALID_PRI;
		*pri_next = MVPP2_C2_LKP_TYPE_INVALID_PRI;
		return 0;
	}

	c2_entry_data = kmalloc(sizeof(*c2_entry_data), GFP_KERNEL);
	if (!c2_entry_data)
		return -ENOMEM;

	memset(c2_entry_data, 0, sizeof(struct pp2_cls_c2_data_t));

	/* Traverse lookup type list */
	LIST_FOR_EACH_OBJECT(c2_index_node, struct pp2_cls_c2_index_t,
			     pp2_cls_db_c2_lkp_type_list_head_get(inst, lkp_type),
			     list_node) {
		/* get C2 db entry data */
		if (pp2_cls_db_c2_data_get(inst, c2_index_node->c2_data_db_idx, c2_entry_data)) {
			free(c2_entry_data);
			return -EINVAL;
		}
		if (priority > c2_entry_data->priority)
			pri_temp_h = c2_entry_data->priority;
		if (priority < c2_entry_data->priority && pri_temp_l == 0)
			pri_temp_l = c2_entry_data->priority;
	}

	*pri_prev = pri_temp_h;
	*pri_next = pri_temp_l;

	free(c2_entry_data);
	return 0;
}

/*******************************************************************************
 * pp2_cls_c2_lkp_type_pri_node_info_get()
 *
 * DESCRIPTION: Get the node information in the list with the same internal priority,
 *              including, first/last entry node in C2 HW, count of node with same priority
 *
 * INPUTS:
 *          inst       - packet processor instance
 *          lkp_type   - lookup type of list
 *          priority   - internal priority
 *
 * OUTPUTS:
 *          c2_hw_first_node  - first c2 hw node in C2 HW table
 *          c2_hw_last_node   - last c2 hw node in C2 HW table
 *          node_count        - count of node with the same priority
 *
 * RETURNS:
 *	0 on success, error-code otherwise
 * COMMENTS:
 *          None
 ******************************************************************************/
static int pp2_cls_c2_lkp_type_pri_node_info_get(struct pp2_inst *inst,
						 u8 lkp_type,
						 u32 priority,
						 struct pp2_cls_c2_index_t **c2_hw_first_node,
						 struct pp2_cls_c2_index_t **c2_hw_last_node,
						 u32 *node_count)
{
	struct pp2_cls_c2_index_t *c2_index_node;
	struct pp2_cls_c2_data_t *c2_entry_data;		/*use heap to reduce stack size*/
	int i;

	/* param check */
	if (!c2_hw_first_node) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}
	if (!c2_hw_last_node) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}
	if (!node_count) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}
	c2_entry_data = kmalloc(sizeof(*c2_entry_data), GFP_KERNEL);
	if (!c2_entry_data)
		return -ENOMEM;

	memset(c2_entry_data, 0, sizeof(struct pp2_cls_c2_data_t));

	i = 0;
	/* Traverse lookup type list */
	LIST_FOR_EACH_OBJECT(c2_index_node, struct pp2_cls_c2_index_t,
			     pp2_cls_db_c2_lkp_type_list_head_get(inst, lkp_type),
			     list_node) {
		/* get C2 db entry data */
		if (pp2_cls_db_c2_data_get(inst, c2_index_node->c2_data_db_idx, c2_entry_data)) {
			free(c2_entry_data);
			return -EINVAL;
		}
		if (c2_entry_data->priority == priority) {
			if (i == 0) {
				*c2_hw_first_node = c2_index_node;
				*c2_hw_last_node = c2_index_node;
			} else {
				/* Find the first C2 HW entry in HW table */
				if ((*c2_hw_first_node)->c2_hw_idx > c2_index_node->c2_hw_idx)
					*c2_hw_first_node = c2_index_node;
				/* Find the last C2 HW entry in HW table */
				if ((*c2_hw_last_node)->c2_hw_idx < c2_index_node->c2_hw_idx)
					*c2_hw_last_node = c2_index_node;
			}
			i++;
		}
	}

	/* if no node with pri, return invalid index */
	if (i == 0) {
		*c2_hw_first_node = NULL;
		*c2_hw_last_node = NULL;
	}
	*node_count = i;

	free(c2_entry_data);
	return 0;
}

/*******************************************************************************
 * pp2_cls_c2_lkp_search_up_block_get()
 *
 * DESCRIPTION: Find the search block between priority pri_start and its privious
 *              neighbour priority.
 *
 * INPUTS:
 *	   inst	      - packet processor instance
 *          lkp_type   - lookup type of list
 *          pri_start  - start priority
 *
 * OUTPUTS:
 *          c2_search_start  - start C2 HW index to search
 *          c2_search_end    - end C2 HW index to search
 *
 * RETURNS:
 *	0 on success, error-code otherwise
 * COMMENTS:
 *          The list can not be empty.
 ******************************************************************************/
static int pp2_cls_c2_lkp_search_up_block_get(struct pp2_inst *inst,
					      u8 lkp_type,
					      u32 pri_start,
					      u32 *c2_search_start,
					      u32 *c2_search_end)
{
	struct pp2_cls_c2_index_t *c2_index_node;
	struct pp2_cls_c2_data_t *c2_entry_data;		/*use heap to reduce stack size*/
	u32 first_pri_find, prev_pri_find;
	u32 next_pri;

	/* Para check */
	if (!c2_search_start) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}
	if (!c2_search_end) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}
	c2_entry_data = kmalloc(sizeof(*c2_entry_data), GFP_KERNEL);
	if (!c2_entry_data)
		return -ENOMEM;

	memset(c2_entry_data, 0, sizeof(struct pp2_cls_c2_data_t));

	first_pri_find = 0;
	prev_pri_find = 0;
	next_pri = 0;
	/* Traverse lookup type list */
	LIST_FOR_EACH_OBJECT_REVERSE(c2_index_node, struct pp2_cls_c2_index_t,
				     pp2_cls_db_c2_lkp_type_list_head_get(inst, lkp_type),
				     list_node) {
		/* get C2 db entry data */
		if (pp2_cls_db_c2_data_get(inst, c2_index_node->c2_data_db_idx, c2_entry_data)) {
			free(c2_entry_data);
			return -EINVAL;
		}
		if (c2_entry_data->priority == pri_start) {
			if (!first_pri_find) {
				*c2_search_end = c2_index_node->c2_hw_idx;
			} else {
				/* Find the last C2 HW entry in HW table */
				if (*c2_search_end < c2_index_node->c2_hw_idx)
					*c2_search_end = c2_index_node->c2_hw_idx;
			}
			first_pri_find++;
		}
		/* Find the next priority */
		if (first_pri_find != 0 && c2_entry_data->priority != pri_start) {
			if (prev_pri_find == 0) {
				next_pri = c2_entry_data->priority;
				*c2_search_start = c2_index_node->c2_hw_idx;
			} else {
				/* Find the last C2 HW entry in HW table */
				if (*c2_search_start < c2_index_node->c2_hw_idx)
					*c2_search_start = c2_index_node->c2_hw_idx;
			}
			prev_pri_find++;
		}
		/* Stop search */
		if (prev_pri_find != 0 && c2_entry_data->priority != next_pri)
			break;
	}
	/* if no node with pri, return invalid index */
	if (first_pri_find == 0)
		*c2_search_end = MVPP2_C2_ENTRY_INVALID_IDX;
	if (prev_pri_find == 0) {
		if (first_pri_find == 0)
			*c2_search_start = MVPP2_C2_ENTRY_INVALID_IDX;
		else
			*c2_search_start = MVPP2_C2_FIRST_ENTRY;
	}

	free(c2_entry_data);
	return 0;
}

/*******************************************************************************
 * pp2_cls_c2_lkp_search_down_block_get()
 *
 * DESCRIPTION: Find the search block between priority pri_start and its next
 *              neighbour priority.
 *
 * INPUTS:
 *	   inst	      - packet processor instance
 *          lkp_type   - lookup type of list
 *          pri_start  - start priority
 *
 * OUTPUTS:
 *          c2_search_start  - start C2 HW index to search
 *          c2_search_end    - end C2 HW index to search
 *
 * RETURNS:
 *	0 on success, error-code otherwise
 * COMMENTS:
 *          The list can not be empty.
 ******************************************************************************/
static int pp2_cls_c2_lkp_search_down_block_get(struct pp2_inst *inst,
						u8 lkp_type,
						u32 pri_start,
						u32 *c2_search_start,
						u32 *c2_search_end)
{
	struct pp2_cls_c2_index_t *c2_index_node;
	struct pp2_cls_c2_data_t *c2_entry_data;		/*use heap to reduce stack size*/
	u32 first_pri_find, next_pri_find;
	u32 next_pri;

	/* Para check */
	if (!c2_search_start) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}
	if (!c2_search_end) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}
	c2_entry_data = kmalloc(sizeof(*c2_entry_data), GFP_KERNEL);
	if (!c2_entry_data)
		return -ENOMEM;

	memset(c2_entry_data, 0, sizeof(struct pp2_cls_c2_data_t));

	first_pri_find = 0;
	next_pri_find = 0;
	next_pri = 0;
	/* Traverse lookup type list */
	LIST_FOR_EACH_OBJECT(c2_index_node, struct pp2_cls_c2_index_t,
			     pp2_cls_db_c2_lkp_type_list_head_get(inst, lkp_type),
			     list_node) {
		/* get C2 db entry data */
		if (pp2_cls_db_c2_data_get(inst, c2_index_node->c2_data_db_idx, c2_entry_data)) {
			free(c2_entry_data);
			return -EINVAL;
		}
		if (c2_entry_data->priority == pri_start) {
			if (first_pri_find == 0) {
				*c2_search_start = c2_index_node->c2_hw_idx;
			} else {
				/* Find the first C2 HW entry in HW table */
				if (*c2_search_start > c2_index_node->c2_hw_idx)
					*c2_search_start = c2_index_node->c2_hw_idx;
			}
			first_pri_find++;
		}
		/* Find the next priority */
		if (first_pri_find != 0 && c2_entry_data->priority > pri_start) {
			if (next_pri_find == 0) {
				next_pri = c2_entry_data->priority;
				*c2_search_end = c2_index_node->c2_hw_idx;
			} else {
				/* Find the first C2 HW entry in HW table */
				if (*c2_search_end > c2_index_node->c2_hw_idx)
					*c2_search_end = c2_index_node->c2_hw_idx;
			}
			next_pri_find++;
		}
		/* Stop search */
		if (next_pri_find != 0 && c2_entry_data->priority > next_pri)
			break;
	}
	/* if no node with pri, return invalid index */
	if (first_pri_find == 0)
		*c2_search_start = MVPP2_C2_ENTRY_INVALID_IDX;
	if (next_pri_find == 0) {
		if (first_pri_find == 0)
			*c2_search_end = MVPP2_C2_ENTRY_INVALID_IDX;
		else
			*c2_search_end = MVPP2_C2_LAST_ENTRY;
	}

	free(c2_entry_data);
	return 0;
}

/*******************************************************************************
 * pp2_cls_c2_lkp_type_list_add()
 *
 * DESCRIPTION: Add the new index node to lookup type list
 *
 * INPUTS:
 *	   inst	      - packet processor instance
 *          lkp_type     - lookup type
 *          priority     - new entry internal priority
 *          c2_hw_idx    - new entry HW index
 *          c2_db_idx    - DB index store new entry data
 *          c2_logic_idx - logic index allocated when add the C2 rule
 *
 * RETURNS:
 *	0 on success, error-code otherwise
 ******************************************************************************/
static int pp2_cls_c2_lkp_type_list_add(struct pp2_inst *inst,
					u8 lkp_type,
					u32 priority,
					u32 c2_hw_idx,
					u32 c2_db_idx,
					u32 c2_logic_idx)
{
	int ret_code;
	struct list *lkp_type_list_head;
	struct pp2_cls_c2_index_t *c2_index_node, *temp_node;
	struct pp2_cls_c2_data_t *c2_entry_data;			/*use heap to reduce stack size*/
	u32 highest_pri, lowest_pri;
	u32 index;

	/* Get list head */
	lkp_type_list_head = pp2_cls_db_c2_lkp_type_list_head_get(inst, lkp_type);

	/* Get the invalid index node */
	for (index = 0; index < MVPP2_C2_ENTRY_MAX; index++) {
		c2_index_node = pp2_cls_db_c2_index_node_get(inst, index);
		if (!c2_index_node)
			return -ENXIO;
		if (c2_index_node->valid == MVPP2_C2_ENTRY_INVALID)
			break;
	}
	if (index == MVPP2_C2_ENTRY_MAX)
		return -ENXIO;

	c2_index_node->c2_hw_idx = c2_hw_idx;
	c2_index_node->c2_data_db_idx = c2_db_idx;
	c2_index_node->c2_logic_idx = c2_logic_idx;

	/* Check lkp list is empty or not */
	if (list_is_empty(lkp_type_list_head)) {
		/* Just add the new node */
		list_add(&c2_index_node->list_node, lkp_type_list_head);
		/* Change Valid status to valid */
		c2_index_node->valid = MVPP2_C2_ENTRY_VALID;
		return 0;
	}

	ret_code = pp2_cls_c2_lkp_type_list_pri_get(inst, lkp_type, &highest_pri, &lowest_pri);
	if (ret_code) {
		pr_err("recvd ret_code(%d)\n", ret_code);
		return ret_code;
	}

	/* New node with highest priority */
	if (highest_pri >= priority) {
		/* Just add the new node to first */
		list_add(&c2_index_node->list_node, lkp_type_list_head);
		/* Change Valid status to valid */
		c2_index_node->valid = MVPP2_C2_ENTRY_VALID;
		return 0;
	}

	/* New node with lowest priority */
	if (highest_pri <= priority) {
		/* Just add the new node to end */
		list_add_to_tail(&c2_index_node->list_node, lkp_type_list_head);
		/* Change Valid status to valid */
		c2_index_node->valid = MVPP2_C2_ENTRY_VALID;
		return 0;
	}

	c2_entry_data = kmalloc(sizeof(*c2_entry_data), GFP_KERNEL);
	if (!c2_entry_data)
		return -ENOMEM;

	memset(c2_entry_data, 0, sizeof(struct pp2_cls_c2_data_t));

	/* New node not the highest and lowest, add it after first node with priority lower than new priority */
	/* Traverse lookup type list */
	LIST_FOR_EACH_OBJECT(temp_node, struct pp2_cls_c2_index_t, lkp_type_list_head, list_node) {
		/* get C2 db entry data */
		if (pp2_cls_db_c2_data_get(inst, c2_index_node->c2_data_db_idx, c2_entry_data)) {
			free(c2_entry_data);
			return -EINVAL;
		}
		if (c2_entry_data->priority > priority) {
			list_add_to_tail(&c2_index_node->list_node, &temp_node->list_node);
			/* Change Valid status to valid */
			c2_index_node->valid = MVPP2_C2_ENTRY_VALID;
		}
	}

	free(c2_entry_data);
	return 0;
}

/*******************************************************************************
 * pp2_cls_c2_entry_is_free
 *
 * DESCRIPTION: The API will to check whether the c2 entry is free or not.
 * INPUTS:
 *	    inst	  - packet processor instance
 *           c2_hw_idx     - first C2 entry index occupied
 *
 * OUTPUTS:
 *           c2_index_node - C2 index node if free, or NULL
 *
 * RETURNS:
 *         0 - free, 1 - occupied
 *
 * COMMENTS:
 *           None.
 ******************************************************************************/
static int pp2_cls_c2_entry_is_free(struct pp2_inst *inst, u32 c2_hw_idx,
				    struct pp2_cls_c2_index_t **c2_index_node)
{
	struct list *list;

	if (!c2_index_node) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}
	/* Traverse free list */
	LIST_FOR_EACH(list, pp2_cls_db_c2_free_list_head_get(inst)) {
		/* get list node */
		*c2_index_node = LIST_OBJECT(list, struct pp2_cls_c2_index_t, list_node);
		if ((*c2_index_node)->c2_hw_idx == c2_hw_idx)
			return MVPP2_C2_ENTRY_FREE_TRUE;
	}
	*c2_index_node = NULL;
	return MVPP2_C2_ENTRY_FREE_FALSE;
}

/*******************************************************************************
 * pp2_cls_c2_free_slot_find
 *
 * DESCRIPTION: The API will find the first free slot between index1 and index2.
 *              If found free slot, delete it from free list.
 * INPUTS:
 *           inst    - packet processor instance
 *           index1  - first C2 entry index occupied
 *           index2  - second C2 entry index occupied
 *
 * OUTPUTS:
 *           c2_hw_idx - Free C2 HW entry index
 *
 * RETURNS:
 *	0 on success, error-code otherwise
 * COMMENTS:
 *           index2 must greater than index1.
 ******************************************************************************/
static int pp2_cls_c2_free_slot_find(struct pp2_inst *inst, u32 index1,
				     u32 index2,
				     u32 *free_idx)
{
	struct pp2_cls_c2_index_t *c2_index_node;
	int found = 0;

	if (!free_idx) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}
	if (index1 >= index2) {
		*free_idx = MVPP2_C2_ENTRY_INVALID_IDX;
		return 0;
	}
	/* Traverse free list */
	LIST_FOR_EACH_OBJECT(c2_index_node, struct pp2_cls_c2_index_t,
			     pp2_cls_db_c2_free_list_head_get(inst), list_node) {
		if ((c2_index_node->c2_hw_idx > index1) &&
		    (c2_index_node->c2_hw_idx < index2)) {
			found++;
			/* delete it from free list */
			list_del(&c2_index_node->list_node);
			/* Change to node valid status to invalid */
			c2_index_node->valid = MVPP2_C2_ENTRY_INVALID;
			*free_idx = c2_index_node->c2_hw_idx;
			break;
		}
	}
	if (found == 0)
		*free_idx = MVPP2_C2_ENTRY_INVALID_IDX;

	return 0;
}

/*******************************************************************************
 * pp2_cls_c2_make_slot_high()
 *
 * DESCRIPTION: This routine is sub routine of "pp2_cls_c2_make_slot"
 *
 * INPUTS:
 *	   inst	      - packet processor instance
 *          lkp_type   - lookup type
 *          priority   - priority for new slot
 *          highest_pri- highest priority of current lookup type list
 *          lowest_pri - lowest priority of current lookup type list
 * OUTPUTS:
 *          c2_hw_idx  - available C2 TCAM index for new rule
 * RETURNS:
 *	0 on success, error-code otherwise
 * COMMENTS:
 *           The lkp_type list can not be empty.
 *           priority <= highest_pri
 ******************************************************************************/
static int pp2_cls_c2_make_slot_high(struct pp2_inst *inst,
				     u32 lkp_type,
				     u32 priority,
				     u32 highest_pri,
				     u32 lowest_pri,
				     u32 *c2_hw_idx)
{
	int ret_code;
	struct pp2_cls_c2_index_t *c2_index_node = NULL, *c2_first_node = NULL, *c2_last_node = NULL;
	u32 free_idx, pp2_cls_idx, node_count;
	u32 c2_search_start, c2_search_end;
	int pri_tmp, i;
	struct mv_pp2x_cls_c2_entry c2_entry;
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);

	/* No need to agjust C2 way */
	if (pp2_cls_c2_entry_is_free(inst, MVPP2_C2_FIRST_ENTRY, &c2_index_node) == MVPP2_C2_ENTRY_FREE_TRUE) {
		/* delete it from free list */
		list_del(&c2_index_node->list_node);
		/* Change to node valid status to invalid */
		c2_index_node->valid = MVPP2_C2_ENTRY_INVALID;
		/* return c2 hw index */
		*c2_hw_idx = MVPP2_C2_FIRST_ENTRY;
		return 0;
	}

	ret_code = pp2_cls_c2_lkp_type_pri_node_info_get(inst,
							 lkp_type,
							 highest_pri,
							 &c2_first_node,
							 &c2_last_node,
							 &node_count);
	if (ret_code) {
		pr_err("C2 lookup type list(%d) priority (%d) node info get failed\n",
		       lkp_type, highest_pri);
		return -EINVAL;
	}
	if (c2_first_node &&
	    c2_first_node->c2_hw_idx > MVPP2_C2_FIRST_ENTRY) {
		/* Find the available slot */
		ret_code = pp2_cls_c2_free_slot_find(inst, MVPP2_C2_FIRST_ENTRY, c2_first_node->c2_hw_idx, &free_idx);
		if (ret_code) {
			pr_err("No found free slot between entry(%d) and entry(%d)\n",
			       MVPP2_C2_FIRST_ENTRY, c2_first_node->c2_hw_idx);
			return -EINVAL;
		}
		if (free_idx != MVPP2_C2_ENTRY_INVALID_IDX) {
			/* return c2 hw index */
			*c2_hw_idx = free_idx;
			return 0;
		}
	}

	/* Search down and adjust C2 original entries */
	for (pri_tmp = highest_pri; pri_tmp <= lowest_pri; pri_tmp++) {
		/* Get search block */
		ret_code = pp2_cls_c2_lkp_search_down_block_get(inst,
								lkp_type,
								pri_tmp,
								&c2_search_start,
								&c2_search_end);
		if (ret_code) {
			pr_err("Search blcok get failed\n");
			return -EINVAL;
		}
		if (c2_search_start != MVPP2_C2_ENTRY_INVALID_IDX) {
			ret_code = pp2_cls_c2_free_slot_find(inst, c2_search_start, c2_search_end, &free_idx);
			if (ret_code) {
				pr_err("No found free slot between (%d) and (%d)\n",
				       c2_search_start, c2_search_end);
				return -EINVAL;
			}
			if (free_idx != MVPP2_C2_ENTRY_INVALID_IDX) {
				/* Find free entry, adjust C2 table */
				for (i = pri_tmp; i >= (int)highest_pri; i--) {
					pp2_cls_idx = free_idx;
					/* Find the first entry with priority found free slot */
					ret_code = pp2_cls_c2_lkp_type_pri_node_info_get(inst,
											 lkp_type,
											 i,
											 &c2_first_node,
											 &c2_last_node,
											 &node_count);
					if (ret_code)
						return -EINVAL;
					if (c2_first_node) {
						mv_pp2x_c2_sw_clear(&c2_entry);
						mv_pp2x_cls_c2_hw_read(cpu_slot, c2_first_node->c2_hw_idx, &c2_entry);
						mv_pp2x_cls_c2_hw_write(cpu_slot, free_idx, &c2_entry);
						/* Continue next move */
						free_idx = c2_first_node->c2_hw_idx;
						/* Update C2 index node of the lookup type */
						c2_first_node->c2_hw_idx = pp2_cls_idx;
					}
				}
				*c2_hw_idx = free_idx;
				return 0;
			}
		}
	}

	return -EINVAL;
}

/*******************************************************************************
 * pp2_cls_c2_make_slot_middle()
 *
 * DESCRIPTION: This routine is sub routine of "pp2_cls_c2_make_slot"
 *
 * INPUTS:
 *	   inst	      - packet processor instance
 *          lkp_type   - lookup type
 *          priority   - priority for new slot
 *          highest_pri- highest priority of current lookup type list
 *          lowest_pri - lowest priority of current lookup type list
 * OUTPUTS:
 *          c2_hw_idx  - available C2 TCAM index for new rule
 * RETURNS:
 *	0 on success, error-code otherwise
 * COMMENTS:
 *           The lkp_type list can not be empty.
 *            priority > highest_pri; priority < lowest_pri
 ******************************************************************************/
static int pp2_cls_c2_make_slot_middle(struct pp2_inst *inst,
				       u32 lkp_type,
				       u32 priority,
				       u32 highest_pri,
				       u32 lowest_pri,
				       u32 *c2_hw_idx)
{
	int ret_code;
	struct pp2_cls_c2_index_t *c2_index_node = NULL, *c2_first_node = NULL, *c2_last_node = NULL;
	u32 free_idx, pp2_cls_idx, node_count;
	u32 c2_search_start, c2_search_end, pri_prev, pri_next;
	int pri_tmp, i;
	struct mv_pp2x_cls_c2_entry c2_entry;
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);

	/* No need adjust other C2 entry way */
	ret_code = pp2_cls_c2_lkp_type_list_neighbour_pri_get(inst,
							      lkp_type,
							      priority,
							      highest_pri,
							      lowest_pri,
							      &pri_prev,
							      &pri_next);
	if (ret_code)
		return -EINVAL;
	/* Get node info with priority */
	if (pri_prev != MVPP2_C2_LKP_TYPE_INVALID_PRI) {
		ret_code = pp2_cls_c2_lkp_type_pri_node_info_get(inst,
								 lkp_type,
								 pri_prev,
								 &c2_first_node,
								 &c2_last_node,
								 &node_count);
		if (ret_code)
			return -EINVAL;
		if (c2_last_node)
			c2_search_start = c2_last_node->c2_hw_idx;
	}
	if (pri_next != MVPP2_C2_LKP_TYPE_INVALID_PRI) {
		ret_code = pp2_cls_c2_lkp_type_pri_node_info_get(inst,
								 lkp_type,
								 pri_next,
								 &c2_first_node,
								 &c2_last_node,
								 &node_count);
		if (ret_code)
			return -EINVAL;
		if (c2_first_node)
			c2_search_end = c2_first_node->c2_hw_idx;
	}
	/* Find the available before first node */
	ret_code = pp2_cls_c2_free_slot_find(inst, c2_search_start, c2_search_end, &free_idx);
	if (ret_code) {
		pr_err("No found free slot between (%d) and (%d) on C2 engine\n",
		       c2_search_start, c2_search_end);
		return -EINVAL;
	}
	if (free_idx != MVPP2_C2_ENTRY_INVALID_IDX) {
		/* return c2 hw index */
		*c2_hw_idx = free_idx;

		return 0;
	}

	/* Search up and adjust C2 original entries */
	for (pri_tmp = pri_prev; pri_tmp >= (int)highest_pri; pri_tmp--) {
		/* Get search block */
		ret_code = pp2_cls_c2_lkp_search_up_block_get(inst,
							      lkp_type,
							      pri_tmp,
							      &c2_search_start,
							      &c2_search_end);
		if (c2_search_end != MVPP2_C2_ENTRY_INVALID_IDX) {
			/* Special handle entry 0 */
			if (c2_search_start == MVPP2_C2_FIRST_ENTRY &&
			    pp2_cls_c2_entry_is_free(inst, c2_search_start, &c2_index_node) ==
							MVPP2_C2_ENTRY_FREE_TRUE) {
				/* delete it from free list */
				list_del(&c2_index_node->list_node);
				/* Change to node valid status to invalid */
				c2_index_node->valid = MVPP2_C2_ENTRY_INVALID;
				free_idx = c2_search_start;
			} else {
				ret_code = pp2_cls_c2_free_slot_find(inst, c2_search_start, c2_search_end, &free_idx);
				if (ret_code) {
					pr_err("No found free slot between (%d) and (%d) failed\n",
					       c2_search_start, c2_search_end);
					return -EINVAL;
				}
			}
			if (free_idx != MVPP2_C2_ENTRY_INVALID_IDX) {
				/* Find free entry, adjust C2 table */
				for (i = pri_tmp; i <= pri_prev; i++) {
					pp2_cls_idx = free_idx;
					/* Find the first entry with priority found free slot */
					ret_code = pp2_cls_c2_lkp_type_pri_node_info_get(inst,
											 lkp_type,
											 i,
											 &c2_first_node,
											 &c2_last_node,
											 &node_count);
					if (ret_code)
						return -EINVAL;
					if (c2_last_node) {
						mv_pp2x_c2_sw_clear(&c2_entry);
						mv_pp2x_cls_c2_hw_read(cpu_slot, c2_last_node->c2_hw_idx, &c2_entry);
						mv_pp2x_cls_c2_hw_write(cpu_slot, free_idx, &c2_entry);
						/* Continue next move */
						free_idx = c2_last_node->c2_hw_idx;
						/* Update C2 index node of the lookup type */
						c2_last_node->c2_hw_idx = pp2_cls_idx;
					}
				}
				*c2_hw_idx = free_idx;
				return 0;
			}
		}
	}

	/* Search down and adjust C2 original entries */
	for (pri_tmp = pri_next; pri_tmp <= (int)lowest_pri; pri_tmp++) {
		/* Get search block */
		ret_code = pp2_cls_c2_lkp_search_down_block_get(inst,
								lkp_type,
								pri_tmp,
								&c2_search_start,
								&c2_search_end);
		if (c2_search_start != MVPP2_C2_ENTRY_INVALID_IDX) {
			ret_code = pp2_cls_c2_free_slot_find(inst, c2_search_start, c2_search_end, &free_idx);
			if (ret_code) {
				pr_err("No found free slot between (%d) and (%d) failed\n",
				       c2_search_start, c2_search_end);
				return -EINVAL;
			}
			if (free_idx != MVPP2_C2_ENTRY_INVALID_IDX) {
				/* Find free entry, adjust C2 table */
				for (i = pri_tmp; i >= pri_next; i--) {
					pp2_cls_idx = free_idx;
					/* Find the first entry with priority found free slot */
					ret_code = pp2_cls_c2_lkp_type_pri_node_info_get(inst,
											 lkp_type,
											 i,
											 &c2_first_node,
											 &c2_last_node,
											 &node_count);
					if (ret_code)
						return -EINVAL;
					if (c2_last_node) {
						mv_pp2x_c2_sw_clear(&c2_entry);
						mv_pp2x_cls_c2_hw_read(cpu_slot, c2_first_node->c2_hw_idx, &c2_entry);
						mv_pp2x_cls_c2_hw_write(cpu_slot, free_idx, &c2_entry);
						/* Continue next move */
						free_idx = c2_first_node->c2_hw_idx;
						/* Update C2 index node of the lookup type */
						c2_first_node->c2_hw_idx = pp2_cls_idx;
					}
				}
				*c2_hw_idx = free_idx;
				return 0;
			}
		}
	}

	return -EINVAL;
}

/*******************************************************************************
 * pp2_cls_c2_make_slot_low()
 *
 * DESCRIPTION: This routine is sub routine of "pp2_cls_c2_make_slot"
 *
 * INPUTS:
 *	   inst	      - packet processor instance
 *          lkp_type   - lookup type
 *          priority   - priority for new slot
 *          highest_pri- highest priority of current lookup type list
 *          lowest_pri - lowest priority of current lookup type list
 * OUTPUTS:
 *          c2_hw_idx  - available C2 TCAM index for new rule
 * RETURNS:
 *	0 on success, error-code otherwise
 * COMMENTS:
 *           The lkp_type list can not be empty.
 *           priority >= lowest_pri
 ******************************************************************************/
static int pp2_cls_c2_make_slot_low(struct pp2_inst *inst,
				    u32 lkp_type,
				    u32 priority,
				    u32 highest_pri,
				    u32 lowest_pri,
				    u32 *c2_hw_idx)
{
	int ret_code;
	struct pp2_cls_c2_index_t *c2_index_node = NULL, *c2_first_node = NULL, *c2_last_node = NULL;
	u32 free_idx, pp2_cls_idx, node_count;
	u32 c2_search_start, c2_search_end;
	int pri_tmp, i;
	struct mv_pp2x_cls_c2_entry c2_entry;
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);

	/* No need adjust other C2 entry way */
	/* get last node with lowest priority */
	ret_code = pp2_cls_c2_lkp_type_pri_node_info_get(inst,
							 lkp_type,
							 lowest_pri,
							 &c2_first_node,
							 &c2_last_node,
							 &node_count);
	if (ret_code) {
		pr_err("Lookup type list(%d) priority (%d) node info get failed\n",
		       lkp_type, lowest_pri);
		return -EINVAL;
	}
	/* try to find the slot after last node with lowest_pri */
	if (c2_last_node) {
		/* Find the available slot */
		ret_code = pp2_cls_c2_free_slot_find(inst, c2_last_node->c2_hw_idx, MVPP2_C2_LAST_ENTRY, &free_idx);
		if (ret_code) {
			pr_err("Free slot between (%d) and (%d) failed\n",
			       c2_last_node->c2_hw_idx, MVPP2_C2_LAST_ENTRY);
			return -EINVAL;
		}
		if (free_idx != MVPP2_C2_ENTRY_INVALID_IDX) {
			/* return c2 hw index */
			*c2_hw_idx = free_idx;

			return 0;
		}
	}
	/* find available slot above node with current lowest priority */
	/* Search up and adjust C2 original entries */
	for (pri_tmp = lowest_pri; pri_tmp >= (int)highest_pri; pri_tmp--) {
		/* Get search block */
		ret_code = pp2_cls_c2_lkp_search_up_block_get(inst,
							      lkp_type,
							      pri_tmp,
							      &c2_search_start,
							      &c2_search_end);
		if (c2_search_end != MVPP2_C2_ENTRY_INVALID_IDX) {
			/* Special handle entry 0 */
			if (c2_search_start == MVPP2_C2_FIRST_ENTRY &&
			    pp2_cls_c2_entry_is_free(inst, c2_search_start, &c2_index_node) ==
											MVPP2_C2_ENTRY_FREE_TRUE) {
				/* delete it from free list */
				list_del(&c2_index_node->list_node);
				/* Change to node valid status to invalid */
				c2_index_node->valid = MVPP2_C2_ENTRY_INVALID;
				free_idx = c2_search_start;
			} else {
				ret_code = pp2_cls_c2_free_slot_find(inst, c2_search_start, c2_search_end, &free_idx);
				if (ret_code) {
					pr_err("Free slot between (%d) and (%d) failed\n",
					       c2_search_start, c2_search_end);
					return -EINVAL;
				}
			}
			if (free_idx != MVPP2_C2_ENTRY_INVALID_IDX) {
				/* Find free entry, adjust C2 table */
				for (i = pri_tmp; i <= (int)lowest_pri; i++) {
					pp2_cls_idx = free_idx;
					/* Find the first entry with priority found free slot */
					ret_code = pp2_cls_c2_lkp_type_pri_node_info_get(inst,
											 lkp_type,
											 i,
											 &c2_first_node,
											 &c2_last_node,
											 &node_count);
					if (ret_code)
						return -EINVAL;
					if (c2_last_node) {
						mv_pp2x_c2_sw_clear(&c2_entry);
						mv_pp2x_cls_c2_hw_read(cpu_slot, c2_last_node->c2_hw_idx, &c2_entry);
						mv_pp2x_cls_c2_hw_write(cpu_slot, free_idx, &c2_entry);
						/* Continue next move */
						free_idx = c2_last_node->c2_hw_idx;
						/* Update C2 index node of the lookup type */
						c2_last_node->c2_hw_idx = pp2_cls_idx;
					}
				}
				*c2_hw_idx = free_idx;
				return 0;
			}
		}
	}

	return -EINVAL;
}

/*******************************************************************************
 * pp2_cls_c2_make_slot
 *
 * DESCRIPTION: The API is called when creating a new C2 entry in HW,
 *              and need to make room for it and adjust original C2 entries down or up.
 * INPUTS:
 *	    inst      - packet processor instance
 *           lkp_type  - C2 entry lookup type
 *           priority  - C2 entry internal priority with same lookup type
 *
 * OUTPUTS:
 *           c2_hw_idx - Free C2 HW entry index
 *
 * RETURNS:
 *	0 on success, error-code otherwise
 ******************************************************************************/
static int pp2_cls_c2_make_slot(struct pp2_inst *inst,
				u8 lkp_type,
				u32 priority,
				u32 *c2_hw_idx)
{
	int ret_code = 0;
	struct pp2_cls_c2_index_t *c2_index_node = NULL;
	u32 highest_pri = 0, lowest_pri = 0;

	/* Patameter check */
	if (!c2_hw_idx) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}
	/* check free list empty or not */
	if (list_is_empty(pp2_cls_db_c2_free_list_head_get(inst))) {
		pr_err("No free C2 entry for lookup typs %d, priority %d\n", lkp_type, priority);
		return -EINVAL;
	}

	/* lkp type list empty, find any available slot */
	if (list_is_empty(pp2_cls_db_c2_lkp_type_list_head_get(inst, lkp_type))) {
		c2_index_node = LIST_FIRST_OBJECT(pp2_cls_db_c2_free_list_head_get(inst),
						  struct pp2_cls_c2_index_t,
						  list_node);
		/* delete it from free list */
		list_del(&c2_index_node->list_node);
		/* Change to node valid status to invalid */
		c2_index_node->valid = MVPP2_C2_ENTRY_INVALID;
		/* return c2 hw index */
		*c2_hw_idx = c2_index_node->c2_hw_idx;
		return 0;
	}

	/* Get the highest priority and lowest priority */
	ret_code = pp2_cls_c2_lkp_type_list_pri_get(inst, lkp_type, &highest_pri, &lowest_pri);
	if (ret_code != 0) {
		pr_err("Lookup type list(%d) priority get failed\n", lkp_type);
		return -EINVAL;
	}

	/* According to priority and current priority in list, search available slot */
	if (priority <= highest_pri)
		ret_code = pp2_cls_c2_make_slot_high(inst, lkp_type, priority, highest_pri,
						     lowest_pri, c2_hw_idx);
	else if (priority >= lowest_pri)
		ret_code = pp2_cls_c2_make_slot_low(inst, lkp_type, priority, highest_pri,
						    lowest_pri, c2_hw_idx);
	else
		ret_code = pp2_cls_c2_make_slot_middle(inst, lkp_type, priority, highest_pri,
						       lowest_pri, c2_hw_idx);

	return ret_code;
}

/*******************************************************************************
 * pp2_cls_c2_rule_add_check
 *
 * DESCRIPTION: The routine will check the input parameters when adding c2 rule.
 * INPUTS:
 *           c2_entry  - parameters needed when adding C2 entry
 *
 * OUTPUTS:
 *           None.
 * RETURNS:
 *	0 on success, error-code otherwise
 ******************************************************************************/
static int pp2_cls_c2_rule_add_check(struct mv_pp2x_c2_add_entry *c2_entry)
{
	struct pp2_cls_field_match_info field_info[MVPP2_FLOW_FIELD_COUNT_MAX + 1];
	int i;
	u32 bits_cnt = 0;

	/* check c2_entry NULL */
	if (!c2_entry) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}
	/* Port check */
	if (c2_entry->port.port_type > MVPP2_SRC_PORT_TYPE_VIR ||
	    c2_entry->port.port_type < MVPP2_SRC_PORT_TYPE_PHY) {
		pr_err("Invalid port type(%d)\n", c2_entry->port.port_type);
		return -EFAULT;
	}
	if (c2_entry->port.port_type == MVPP2_SRC_PORT_TYPE_VIR) {
		if (c2_entry->port.port_value > MVPP2_VIRT_PORT_ID_MAX) {
			pr_err("Invalid Virt port ID(%d)\n", c2_entry->port.port_value);
			return -EFAULT;
		}
	}

	/* Lookup type check */
	if (c2_entry->lkp_type >= MVPP2_C2_LKP_TYPE_MAX) {
		pr_err("Invalid lookup type(%d)\n", c2_entry->lkp_type);
		return -EFAULT;
	}

	/* Packet key check  */
	if (!c2_entry->mng_pkt_key) {
		pr_err("Input NULL pointer\n");
		return -EFAULT;
	}
	/* Get field info */
	memset(field_info, 0, sizeof(struct pp2_cls_field_match_info) * (MVPP2_FLOW_FIELD_COUNT_MAX + 1));
	if (pp2_cls_field_bm_to_field_info(c2_entry->mng_pkt_key->pkt_key->field_bm,
					   c2_entry->mng_pkt_key,
					   MVPP2_FLOW_FIELD_COUNT_MAX + 1,
					   true,
					   field_info)) {
		pr_err("Field info get failed\n");
		return -EFAULT;
	}
	/* Check field number, if greater than 4, invalid */
	if (field_info[MVPP2_FLOW_FIELD_COUNT_MAX].valid == MVPP2_FIELD_VALID) {
		pr_err("At most 4 fileds are supported\n");
		return -EFAULT;
	}
	/*
	 * Raw check field length(detail check will be done in pp2_cls_c2_tcam_hek_get),
	 * total can not more than 8 bytes
	 */
	for (i = 0; i < MVPP2_FLOW_FIELD_COUNT_MAX; i++) {
		if (field_info[i].valid == MVPP2_FIELD_VALID)
			bits_cnt += pp2_cls_field_size_get(field_info[i].field_id);
	}
	if (bits_cnt > MVPP2_C2_TCAM_KEY_LEN_MAX * BYTE_BITS) {
		pr_err("Packet key length(%d bits) beyond C2 capability\n", bits_cnt);
		return -EFAULT;
	}

	/* QOS check, TBD */

	/* Action check, TBD */

	/* Mod info check */
	if (c2_entry->pkt_mod.mod_cmd_idx > MVPP2_HWF_MOD_IPTR_MAX) {
		pr_err("Invalid modification cmd index(%d)\n", c2_entry->pkt_mod.mod_cmd_idx);
		return -EFAULT;
	}
	if (c2_entry->pkt_mod.mod_data_idx > MVPP2_HW_MOD_DPTR_MAX) {
		pr_err("Invalid data index(%d)\n", c2_entry->pkt_mod.mod_data_idx);
		return -EFAULT;
	}

	/* Duplication flow info check */

	return 0;
}

/*******************************************************************************
 * pp2_cls_c2_tcam_common_field_hek_get
 *
 * DESCRIPTION: The routine will transfer pkacket key with common field whose
 *              field size do not greater than 32 bits to C2 HEK
 *
 * INPUTS:
 *           pkt_value         - field value
 *           pkt_value_mask    - field mask
 *           field_bytes       - bytes the filed occupied
 *           field_size        - field size
 *           bytes_used        - pointer to HEK bytes has been used
 *
 * OUTPUTS:
 *           c2_hek            - HEK for C2
 *           c2_hek_mask       - HEK mask for C2
 *           bytes_used        - pointer to record HEK bytes has been used
 *
 * RETURNS:
 *	0 on success, error-code otherwise
 ******************************************************************************/
static int pp2_cls_c2_tcam_common_field_hek_get(u32 pkt_value,
						u32 pkt_value_mask,
						u32 field_bytes,
						u32 field_size,
						u8 filed_unmask,
						u8 c2_hek[],
						u8 c2_hek_mask[],
						u32 *bytes_used)
{
	int i;
	u32 c2_hek_bytes_used;

	/* Para check */
	if (!c2_hek) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}
	if (!c2_hek_mask) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}
	if (!bytes_used) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}

	if (field_size == 0 ||
	    field_bytes == 0)
		return -EFAULT;

	/* Parse packet key */
	c2_hek_bytes_used = *bytes_used;
	for (i = 0; i < field_bytes; i++) {
		if (field_size % BYTE_BITS) {
			if (i < (field_bytes - 1)) {
				/* HEK Value */
				c2_hek[c2_hek_bytes_used] =
				(((pkt_value & common_mask_gen(field_size)) >>
				  (BYTE_BITS * (field_bytes - 2 - i) + field_size % BYTE_BITS)) &
				 BYTE_MASK);
				/* HEK Mask */
				c2_hek_mask[c2_hek_bytes_used] =
				(((pkt_value_mask & common_mask_gen(field_size) &
				   (filed_unmask ? (~(common_mask_gen(field_size))) : (common_mask_gen(field_size)))) >>
				  (BYTE_BITS * (field_bytes - 2 - i) + field_size % BYTE_BITS)) &
				 BYTE_MASK);
			} else {
				/* HEK Value */
				c2_hek[c2_hek_bytes_used] =
				((pkt_value << (BYTE_BITS - field_size % BYTE_BITS)) & BYTE_MASK);
				/* HEK Mask */
				if (!filed_unmask)
					c2_hek_mask[c2_hek_bytes_used] =
					((pkt_value_mask << (BYTE_BITS - field_size % BYTE_BITS)) & BYTE_MASK);
			}
		} else {
			/* HEK Value */
			c2_hek[c2_hek_bytes_used] =
			(((pkt_value & common_mask_gen(field_size)) >>
			  (BYTE_BITS * (field_bytes - 1 - i))) &
			 BYTE_MASK);
			/* HEK Mask */
			c2_hek_mask[c2_hek_bytes_used] =
			(((pkt_value_mask & common_mask_gen(field_size) &
			   (filed_unmask ? (~(common_mask_gen(field_size))) : (common_mask_gen(field_size)))) >>
			  (BYTE_BITS * (field_bytes - 1 - i))) &
			 BYTE_MASK);
		}
		/* Increase HEK byte count */
		c2_hek_bytes_used++;
	}
	/* Update bytes_used */
	*bytes_used = c2_hek_bytes_used;

	return 0;
}

/*******************************************************************************
 * pp2_cls_c2_tcam_shared_field_hek_get
 *
 * DESCRIPTION: The routine will transfer pkacket key with common field who
 *              shares one byte with other field to C2 HEK
 *
 * INPUTS:
 *           pkt_value         - field value
 *           pkt_value_mask    - field mask
 *           field_bytes       - bytes the filed occupied
 *           field_size        - field size
 *           comb_flag         - indicate combination is needed or not
 *           comb_offset       - combination bit offset
 *           bytes_used        - pointer to HEK bytes has been used
 *
 * OUTPUTS:
 *           c2_hek            - HEK for C2
 *           c2_hek_mask       - HEK mask for C2
 *           bytes_used        - pointer to record HEK bytes has been used
 *
 * RETURNS:
 *	0 on success, error-code otherwise
 ******************************************************************************/
static int pp2_cls_c2_tcam_shared_field_hek_get(u32 pkt_value,
						u32 pkt_value_mask,
						u32 field_bytes,
						u32 field_size,
						u8 filed_unmask,
						bool comb_flag,
						u8 comb_offset,
						u8 c2_hek[],
						u8 c2_hek_mask[],
						u32 *bytes_used)
{
	int i;
	u32 left_bits, c2_hek_bytes_used;
	bool comb_flag1, comb_flag2;

	/* Para check */
	if (!c2_hek) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}
	if (!c2_hek_mask) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}
	if (!bytes_used) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}

	if (field_size == 0 ||
	    field_bytes == 0)
		return -EFAULT;

	left_bits = field_size;
	c2_hek_bytes_used = *bytes_used;
	comb_flag1 = comb_flag;
	comb_flag2 = comb_flag;
	for (i = 0; i < field_bytes; i++) {
		if (comb_flag2 && comb_flag1) {
			c2_hek_bytes_used--;
			/* HEK Value */
			c2_hek[c2_hek_bytes_used] |=
			(((pkt_value >> (field_size - comb_offset)) &
			  common_mask_gen(comb_offset)) &
			 BYTE_MASK);
			/* HEK Mask */
			c2_hek_mask[c2_hek_bytes_used] |=
			(((pkt_value_mask >> (field_size - comb_offset)) &
			  common_mask_gen(comb_offset) &
			  (filed_unmask ? (~(common_mask_gen(comb_offset))) : common_mask_gen(comb_offset))) &
			 BYTE_MASK);
			if (((field_size % BYTE_BITS) + comb_offset) > BYTE_BITS ||
			    (field_size > BYTE_BITS)) {
				pkt_value &= common_mask_gen(field_size - comb_offset);
				pkt_value_mask &= common_mask_gen(field_size - comb_offset);
			}
			c2_hek_bytes_used++;
			left_bits = field_size - comb_offset;
			comb_flag1 = false;
		} else if (comb_flag2) {
			if (left_bits % BYTE_BITS) {
				if (i < (field_bytes - 1)) {
					/* HEK Value */
					c2_hek[c2_hek_bytes_used] =
					((pkt_value & common_mask_gen(field_size)) >>
					 (BYTE_BITS * (field_bytes - 2 - i) +
					  left_bits % BYTE_BITS)) &
					BYTE_MASK;
					/* HEK Mask */
					c2_hek_mask[c2_hek_bytes_used] =
					((pkt_value_mask & common_mask_gen(field_size) &
					  (filed_unmask ?
					   (~(common_mask_gen(field_size))) : common_mask_gen(field_size))) >>
					 (BYTE_BITS * (field_bytes - 2 - i) +
					  left_bits % BYTE_BITS)) &
					BYTE_MASK;
				} else {
					/* HEK Value */
					c2_hek[c2_hek_bytes_used] =
					(pkt_value << (BYTE_BITS - left_bits % BYTE_BITS)) &
					BYTE_MASK;
					/* HEK Mask */
					if (!filed_unmask)
						c2_hek_mask[c2_hek_bytes_used] =
						(pkt_value_mask << (BYTE_BITS - left_bits % BYTE_BITS)) &
						BYTE_MASK;
				}
			} else {
				/* HEK Value */
				c2_hek[c2_hek_bytes_used] =
				((pkt_value & common_mask_gen(field_size)) >>
				 (BYTE_BITS * (field_bytes - 1 - i))) &
				BYTE_MASK;
				/* HEK Mask */
				if (!filed_unmask)
					c2_hek_mask[c2_hek_bytes_used] =
					((pkt_value_mask & common_mask_gen(field_size)) >>
					 (BYTE_BITS * (field_bytes - 1 - i))) &
					BYTE_MASK;
			}
			c2_hek_bytes_used++;
			comb_flag2 = false;
		} else {
			/* HEK Value */
			c2_hek[c2_hek_bytes_used] =
			((pkt_value & common_mask_gen(field_size)) >>
			 (BYTE_BITS * (field_bytes - 1 - i))) &
			BYTE_MASK;
			/* HEK Mask */
			c2_hek_mask[c2_hek_bytes_used] =
			((pkt_value_mask & common_mask_gen(field_size) &
			  (filed_unmask ? (~(common_mask_gen(field_size))) : common_mask_gen(field_size))) >>
			 (BYTE_BITS * (field_bytes - 1 - i))) &
			BYTE_MASK;
			c2_hek_bytes_used++;
		}
	}
	*bytes_used = c2_hek_bytes_used;

	return 0;
}

/*******************************************************************************
 * pp2_cls_c2_field_unmask_check
 *
 * DESCRIPTION: The routine check the field is unmask or not
 *
 * INPUTS:
 *           field_id        - match fiels bit map
 *           field_unmask    - filed array need unmask
 *
 * OUTPUTS:
 *           None
 *
 * RETURNS:
 * unmask, 0 - means need exact match the filed; 1 - means do not care the filed value
 *
 * COMMENTS:
 *           None.
 ******************************************************************************/
static u8 pp2_cls_c2_field_unmask_check(u32 field_id, struct pp2_cls_field_match_info *field_unmask)
{
	u8 unmask = 0;
	u32 idx;

	if (!field_unmask)
		return unmask;

	for (idx = 0; idx < MVPP2_FLOW_FIELD_COUNT_MAX; idx++) {
		if (field_unmask[idx].valid == MVPP2_FIELD_VALID &&
		    field_unmask[idx].field_id == field_id) {
			unmask = 1;
			break;
		}
	}

	return unmask;
}

/*******************************************************************************
 * pp2_cls_c2_tcam_hek_get
 *
 * DESCRIPTION: The routine will transfer pkacket key to C2 HEK
 *
 * INPUTS:
 *           field_bm    - match fiels bit map
*            c2_entry    - C2 entry para to get packet key info
 *
 * OUTPUTS:
*            hek         - HEK for C2
 *           hek_mask    - HEK mask for C2
 *
 * RETURNS:
 *	0 on success, error-code otherwise
 ******************************************************************************/
int pp2_cls_c2_tcam_hek_get(u32 field_bm,
			    struct mv_pp2x_c2_add_entry *c2_entry,
			    u8 hek[],
			    u8 hek_mask[])
{
	int ret_code = 0;
	struct pp2_cls_field_match_info *field_info, *field_unmask;	/*use heap to reduce stack size*/
	u8 c2_hek[MVPP2_C2_HEK_OFF_MAX];
	u8 c2_hek_mask[MVPP2_C2_HEK_OFF_MAX];
	u32 field_bytes, field_id, field_size, pkt_value, pkt_value_mask;
	u32 c2_hek_bytes_used = 0;/* used to recoed current bytes filled in HEK */
	int field_num, i;
	u32 pre_field_id = 0;
	bool comb_flag = false;
	u8 comb_offset = 0;

	if (!c2_entry) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}
	if (!hek) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}
	if (!hek_mask) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}
	field_info = kcalloc(MVPP2_FLOW_FIELD_COUNT_MAX, sizeof(struct pp2_cls_field_match_info), GFP_KERNEL);
	if (!field_info)
		return -ENOMEM;

	memset(field_info, 0, MVPP2_FLOW_FIELD_COUNT_MAX * sizeof(struct pp2_cls_field_match_info));

	field_unmask = kcalloc(MVPP2_FLOW_FIELD_COUNT_MAX, sizeof(struct pp2_cls_field_match_info), GFP_KERNEL);
	if (!field_unmask) {
		free(field_info);
		return -ENOMEM;
	}
	memset(field_unmask, 0, MVPP2_FLOW_FIELD_COUNT_MAX * sizeof(struct pp2_cls_field_match_info));

	/* clear related structure */
	memset(&field_info[0], 0, sizeof(struct pp2_cls_field_match_info) * MVPP2_FLOW_FIELD_COUNT_MAX);
	memset(c2_hek, 0, MVPP2_C2_HEK_OFF_MAX);
	memset(c2_hek_mask, 0, MVPP2_C2_HEK_OFF_MAX);
	/* get field info */
	ret_code = pp2_cls_field_bm_to_field_info(field_bm,
						  c2_entry->mng_pkt_key,
						  MVPP2_FLOW_FIELD_COUNT_MAX,
						  true,
						  field_info);
	if (ret_code) {
		pr_err("recvd ret_code(%d)\n", ret_code);
		free(field_info);
		free(field_unmask);
		return ret_code;
	}

	/* Get filed need to unmask */
	ret_code = pp2_cls_field_bm_to_field_info(field_bm &
						  ((~(c2_entry->mng_pkt_key->pkt_key->field_bm_mask)) |
						  MVPP2_MATCH_IPV6_PKT | MVPP2_MATCH_IPV4_PKT),
						  c2_entry->mng_pkt_key,
						  MVPP2_FLOW_FIELD_COUNT_MAX,
						  true,
						  field_unmask);
	if (ret_code) {
		pr_err("recvd ret_code(%d)\n", ret_code);
		free(field_info);
		free(field_unmask);
		return ret_code;
	}
	/* Set C2 TCAM HEK */
	field_num = 0;
	while (field_num < MVPP2_FLOW_FIELD_COUNT_MAX && field_info[field_num].valid == MVPP2_FIELD_VALID) {
		field_id = field_info[field_num].field_id;
		field_size = pp2_cls_field_size_get(field_id);
		if (field_size % BYTE_BITS)
			field_bytes = (field_size / BYTE_BITS) + 1;
		else
			field_bytes = field_size / BYTE_BITS;

		/* Organize pkt key according to field size and order */
		switch (field_id) {
		case MH_FIELD_ID:
		case MH_UNTAGGED_PRI_FIELD_ID:
		case OUT_VLAN_PRI_FIELD_ID:
		case ETH_TYPE_FIELD_ID:
		case PPPOE_FIELD_ID:
		case IP_VER_FIELD_ID:
		case IPV4_DSCP_FIELD_ID:
		case IPV4_LEN_FIELD_ID:
		case IPV4_TTL_FIELD_ID:
		case IPV4_PROTO_FIELD_ID:
		case IPV6_PAYLOAD_LEN_FIELD_ID:
		case IPV6_NH_FIELD_ID:
		case L4_SRC_FIELD_ID:
		case L4_DST_FIELD_ID:
		case TCP_FLAGS_FIELD_ID:
		case IN_VLAN_PRI_FIELD_ID:
		case PPPOE_PROTO_ID:
		case OUT_TPID_FIELD_ID:
		case IN_TPID_FIELD_ID:
			/* Get HEK data */
			pkt_value = field_info[field_num].filed_value.int_data.parsed_int_val;
			pkt_value_mask = field_info[field_num].filed_value.int_data.parsed_int_val_mask;
			/* Store HEK in c2_hek, each filed byte boutary */
			ret_code = pp2_cls_c2_tcam_common_field_hek_get(pkt_value,
									pkt_value_mask,
									field_bytes,
									field_size,
									pp2_cls_c2_field_unmask_check(field_id,
												      field_unmask),
									c2_hek,
									c2_hek_mask,
									&c2_hek_bytes_used);
			if (ret_code) {
				pr_err("recvd ret_code(%d)\n", ret_code);
				free(field_info);
				free(field_unmask);
				return ret_code;
			}
			/* Check HEK bytes number */
			if (c2_hek_bytes_used > MVPP2_C2_HEK_OFF_LKP_PORT_TYPE) {
				pr_info("HEK bytes (%d) beyond C2 capcity\n", c2_hek_bytes_used);
				free(field_info);
				free(field_unmask);
				return -EFAULT;
			}
			break;
		case OUT_VLAN_CFI_FIELD_ID:
		case IN_VLAN_CFI_FIELD_ID:
			/* Get HEK data */
			pkt_value = field_info[field_num].filed_value.int_data.parsed_int_val;
			pkt_value_mask = field_info[field_num].filed_value.int_data.parsed_int_val_mask;
			c2_hek_bytes_used++;
			if (c2_hek_bytes_used > MVPP2_C2_HEK_OFF_LKP_PORT_TYPE) {
				pr_info("HEK bytes (%d) is beyond C2 capcity\n", c2_hek_bytes_used);
				free(field_info);
				free(field_unmask);
				return -EFAULT;
			}
			c2_hek[c2_hek_bytes_used - 1] =
				(pkt_value << (BYTE_BITS - 1 - (MVPP2_CFI_OFFSET_BITS % BYTE_BITS)));
			c2_hek_mask[c2_hek_bytes_used - 1] =
				(pkt_value_mask << (BYTE_BITS - 1 - (MVPP2_CFI_OFFSET_BITS % BYTE_BITS)));
			break;
		/* Share bits combination */
		case GEM_PORT_ID_FIELD_ID:
		case IN_VLAN_ID_FIELD_ID:
		case OUT_VLAN_ID_FIELD_ID:
		case IPV4_ECN_FIELD_ID:
		case IPV6_DSCP_FIELD_ID:
		case IPV6_ECN_FIELD_ID:
		case IPV6_FLOW_LBL_FIELD_ID:
			if (pre_field_id == OUT_VLAN_PRI_FIELD_ID && field_id == OUT_VLAN_ID_FIELD_ID) {
				comb_flag = true;
				comb_offset = 4;
			}
			if (pre_field_id == IN_VLAN_PRI_FIELD_ID && field_id == IN_VLAN_ID_FIELD_ID) {
				comb_flag = true;
				comb_offset = 4;
			}
			if (pre_field_id == IPV4_DSCP_FIELD_ID && field_id == IPV4_ECN_FIELD_ID) {
				comb_flag = true;
				comb_offset = 2;
			}
			if (pre_field_id == IP_VER_FIELD_ID || field_id == IPV6_DSCP_FIELD_ID) {
				comb_flag = true;
				comb_offset = 4;
				if (pre_field_id != IP_VER_FIELD_ID)
					c2_hek_bytes_used++;
			}
			if (pre_field_id == IPV6_DSCP_FIELD_ID && field_id == IPV6_ECN_FIELD_ID) {
				comb_flag = true;
				comb_offset = 2;
			}
			if (field_id == IPV6_FLOW_LBL_FIELD_ID &&
			    (pre_field_id == IPV6_DSCP_FIELD_ID || pre_field_id == IPV6_ECN_FIELD_ID)) {
				comb_flag = true;
				comb_offset = 4;
			}

			/* Get HEK data */
			pkt_value = field_info[field_num].filed_value.int_data.parsed_int_val;
			pkt_value_mask = field_info[field_num].filed_value.int_data.parsed_int_val_mask;
			/* Check Combination */
			if (comb_flag &&
			    (field_size < BYTE_BITS) &&
			    ((field_size + comb_offset) > BYTE_BITS))
				field_bytes++;

			ret_code = pp2_cls_c2_tcam_shared_field_hek_get(pkt_value,
									pkt_value_mask,
									field_bytes,
									field_size,
									pp2_cls_c2_field_unmask_check(field_id,
												      field_unmask),
									comb_flag,
									comb_offset,
									c2_hek,
									c2_hek_mask,
									&c2_hek_bytes_used);
			if (ret_code) {
				pr_err("recvd ret_code(%d)\n", ret_code);
				free(field_info);
				free(field_unmask);
				return ret_code;
			}
			/* Check HEK bytes number */
			if (c2_hek_bytes_used > MVPP2_C2_HEK_OFF_LKP_PORT_TYPE) {
				pr_info("HEK bytes (%d) beyond C2 capcity\n", c2_hek_bytes_used);
				free(field_info);
				free(field_unmask);
				return -EFAULT;
			}
			break;

		case MAC_DA_FIELD_ID:
		case MAC_SA_FIELD_ID:
		case IPV4_SA_FIELD_ID:
		case IPV4_DA_FIELD_ID:
		case ARP_IPV4_DA_FIELD_ID:
			for (i = 0; i < field_bytes; i++) {
				if (field_id == MAC_DA_FIELD_ID || field_id == MAC_SA_FIELD_ID) {
					/* HEK Value */
					c2_hek[c2_hek_bytes_used] =
						field_info[field_num].filed_value.mac_addr.parsed_mac_addr[i];
					/* HEK Mask */
					if (!pp2_cls_c2_field_unmask_check(field_id, field_unmask))
						c2_hek_mask[c2_hek_bytes_used] =
						field_info[field_num].filed_value.mac_addr.parsed_mac_addr_mask[i];
				} else {
					/* HEK Value */
					c2_hek[c2_hek_bytes_used] =
						field_info[field_num].filed_value.ipv4_addr.parsed_ipv4_addr[i];
					/* HEK Mask */
					if (!pp2_cls_c2_field_unmask_check(field_id, field_unmask))
						c2_hek_mask[c2_hek_bytes_used] =
						field_info[field_num].filed_value.ipv4_addr.parsed_ipv4_addr_mask[i];
				}
				c2_hek_bytes_used++;
			}
			/* Check HEK bytes number */
			if (c2_hek_bytes_used > MVPP2_C2_HEK_OFF_LKP_PORT_TYPE) {
				pr_info("HEK bytes (%d) beyond C2 capcity\n", c2_hek_bytes_used);
				free(field_info);
				free(field_unmask);
				return -EFAULT;
			}
			break;

		case IPV6_SA_PREF_FIELD_ID:
		case IPV6_DA_PREF_FIELD_ID:
			for (i = 0; i < field_bytes; i++) {
				/* HEK Value */
				c2_hek[c2_hek_bytes_used] =
					field_info[field_num].filed_value.ipv6_addr.parsed_ipv6_addr[i];
				/* HEK Mask */
				if (!pp2_cls_c2_field_unmask_check(field_id, field_unmask))
					c2_hek_mask[c2_hek_bytes_used] =
						field_info[field_num].filed_value.ipv6_addr.parsed_ipv6_addr_mask[i];
				c2_hek_bytes_used++;
			}
			/* Check HEK bytes number */
			if (c2_hek_bytes_used > MVPP2_C2_HEK_OFF_LKP_PORT_TYPE) {
				pr_info("HEK bytes (%d) beyond C2 capcity\n", c2_hek_bytes_used);
				free(field_info);
				free(field_unmask);
				return -EFAULT;
			}
			break;

		case IPV6_SA_SUFF_FIELD_ID:
		case IPV6_DA_SUFF_FIELD_ID:
			/* IPv6 suffix needs to be moved to MSB bytes for SRAM */
			for (i = field_bytes; i < IPV6_ADDR_SIZE; i++) {
				/* HEK Value */
				c2_hek[c2_hek_bytes_used] =
					field_info[field_num].filed_value.ipv6_addr.parsed_ipv6_addr[i];
				/* HEK Mask */
				if (!pp2_cls_c2_field_unmask_check(field_id, field_unmask))
					c2_hek_mask[c2_hek_bytes_used] =
						field_info[field_num].filed_value.ipv6_addr.parsed_ipv6_addr_mask[i];
				c2_hek_bytes_used++;
			}
			/* Check HEK bytes number */
			if (c2_hek_bytes_used > MVPP2_C2_HEK_OFF_LKP_PORT_TYPE) {
				pr_info("HEK bytes (%d) beyond C2 capcity\n", c2_hek_bytes_used);
				free(field_info);
				free(field_unmask);
				return -EFAULT;
			}
			break;

		case IPV6_SA_FIELD_ID:
		case IPV6_DA_FIELD_ID:
			/* IPv6 SIP and DIP cant not fit into C2 SRAM */
			free(field_info);
			free(field_unmask);
			return -EFAULT;

		default:
			pr_err("Invalid field ID (%d) on C2 engine\n", field_id);
			free(field_info);
			free(field_unmask);
			return -EFAULT;
		}
		/* record previous id */
		pre_field_id = field_id;

		/* Increase filed number */
		field_num++;

		/* Clear combine flag */
		comb_flag = false;
	}
	/* Return hek and hek_mask */
	for (i = 0; i < c2_hek_bytes_used; i++) {
		hek[MVPP2_C2_HEK_OFF_BYTE7 - i] = c2_hek[i];
		hek_mask[MVPP2_C2_HEK_OFF_BYTE7 - i] = c2_hek_mask[i];
	}
	/* HEK offs 8, lookup type, port type */
	hek[MVPP2_C2_HEK_OFF_LKP_PORT_TYPE] = (c2_entry->port.port_type << MVPP2_C2_HEK_PORT_TYPE_OFFS) |
					    (c2_entry->lkp_type << MVPP2_C2_HEK_LKP_TYPE_OFFS);
	hek_mask[MVPP2_C2_HEK_OFF_LKP_PORT_TYPE] = MVPP2_C2_HEK_PORT_TYPE_MASK |
						 ((c2_entry->lkp_type_mask << MVPP2_C2_HEK_LKP_TYPE_OFFS) &
						 MVPP2_C2_HEK_LKP_TYPE_MASK);
	/* HEK offs 9, port ID */
	hek[MVPP2_C2_HEK_OFF_PORT_ID] = c2_entry->port.port_value;
	hek_mask[MVPP2_C2_HEK_OFF_PORT_ID] = c2_entry->port.port_mask;

	free(field_info);
	free(field_unmask);
	return 0;
}

/*******************************************************************************
 * pp2_cls_c2_tcam_set
 *
 * DESCRIPTION: The routine will get C2 entry data and write it to HW
 *
 * INPUTS:
 *           c2_entry    - C2 entry data
 *           c2_hw_idx   - C2 TCAM HW index
 *
 * OUTPUTS:
 *           c2_index  - The logical index of C2 entry, used to delete the entry
 *
 * RETURNS:
 *	0 on success, error-code otherwise
 ******************************************************************************/
static int pp2_cls_c2_tcam_set(uintptr_t cpu_slot, struct mv_pp2x_c2_add_entry *c2_entry,
			       u32 c2_hw_idx)
{
	int ret_code;
	struct mv_pp2x_cls_c2_entry pp2_cls_c2_entry;
	int hek_offs;
	u8 hek_byte[MVPP2_C2_HEK_OFF_MAX], hek_byte_mask[MVPP2_C2_HEK_OFF_MAX];

	if (!c2_entry) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}
	/* Clear C2 sw data */
	memset(&pp2_cls_c2_entry, 0, sizeof(struct mv_pp2x_cls_c2_entry));

	/* Set QOS table, selection and ID */
	ret_code = mv_pp2x_cls_c2_qos_tbl_set(&pp2_cls_c2_entry,
					      c2_entry->qos_info.qos_tbl_index,
					      c2_entry->qos_info.qos_tbl_type);
	if (ret_code) {
		pr_err("recvd ret_code(%d)\n", ret_code);
		return ret_code;
	}

	/* Set color, cmd and source */
	ret_code = mv_pp2x_cls_c2_color_set(&pp2_cls_c2_entry,
					    c2_entry->action.color_act,
					    c2_entry->qos_info.color_src);
	if (ret_code) {
		pr_err("recvd ret_code(%d)\n", ret_code);
		return ret_code;
	}

	/* Set priority(pbit), cmd, value(not from qos table) and source */
	ret_code = mv_pp2x_cls_c2_prio_set(&pp2_cls_c2_entry,
					   c2_entry->action.pri_act,
					   c2_entry->qos_value.pri,
					   c2_entry->qos_info.pri_dscp_src);
	if (ret_code) {
		pr_err("recvd ret_code(%d)\n", ret_code);
		return ret_code;
	}

	/* Set DSCP, cmd, value(not from qos table) and source */
	ret_code = mv_pp2x_cls_c2_dscp_set(&pp2_cls_c2_entry,
					   c2_entry->action.dscp_act,
					   c2_entry->qos_value.dscp,
					   c2_entry->qos_info.pri_dscp_src);
	if (ret_code) {
		pr_err("recvd ret_code(%d)\n", ret_code);
		return ret_code;
	}

	/* Set gemport ID, cmd, value, and source */
	ret_code = mv_pp2x_cls_c2_gpid_set(&pp2_cls_c2_entry,
					   c2_entry->action.gemp_act,
					   c2_entry->qos_value.gemp,
					   c2_entry->qos_info.gemport_src);
	if (ret_code) {
		pr_err("recvd ret_code(%d)\n", ret_code);
		return ret_code;
	}

	/* Set queue low, cmd, value, and source */
	ret_code = mv_pp2x_cls_c2_queue_low_set(&pp2_cls_c2_entry,
						c2_entry->action.q_low_act,
						c2_entry->qos_value.q_low,
						c2_entry->qos_info.q_low_src);
	if (ret_code) {
		pr_err("recvd ret_code(%d)\n", ret_code);
		return ret_code;
	}

	/* Set queue high, cmd, value and source */
	ret_code = mv_pp2x_cls_c2_queue_high_set(&pp2_cls_c2_entry,
						 c2_entry->action.q_high_act,
						 c2_entry->qos_value.q_high,
						 c2_entry->qos_info.q_high_src);
	if (ret_code) {
		pr_err("recvd ret_code(%d)\n", ret_code);
		return ret_code;
	}

	/* Set forward */
	ret_code = mv_pp2x_cls_c2_forward_set(&pp2_cls_c2_entry,
					      c2_entry->action.frwd_act);
	if (ret_code) {
		pr_err("recvd ret_code(%d)\n", ret_code);
		return ret_code;
	}

	/* Set RSS */
	ret_code = mv_pp2x_cls_c2_rss_set(&pp2_cls_c2_entry,
					  c2_entry->action.rss_act,
					  c2_entry->rss_en);
	if (ret_code) {
		pr_err("recvd ret_code(%d)\n", ret_code);
		return ret_code;
	}

	/* Set policer */
	/* TODO - to be added later */
	/*
	 * ret_code = mv_pp2x_cls_c2_policer_set(&pp2_cls_c2_entry,
	 *				c2_entry->action.policer_act,
	 *				c2_entry->qos_info.policer_id & MVPP2_CLS2_ACT_DUP_ATTR_PLCRID_MAX,
	 *				MVPP2_POLICER_2_BANK(c2_entry->qos_info.policer_id));
	 * if (ret_code) {
	 *	pr_err("recvd ret_code(%d)\n", ret_code);
	 *	return ret_code;
	 * }
	 */

	/* Set flowID(not for multicast) */
	ret_code = mv_pp2x_cls_c2_flow_id_en(&pp2_cls_c2_entry,
					     c2_entry->action.flowid_act);
	if (ret_code) {
		pr_err("recvd ret_code(%d)\n", ret_code);
		return ret_code;
	}

	/* Set modification info */
	ret_code = mv_pp2x_cls_c2_mod_set(&pp2_cls_c2_entry,
					  c2_entry->pkt_mod.mod_data_idx,
					  c2_entry->pkt_mod.mod_cmd_idx,
					  c2_entry->pkt_mod.l4_chksum_update_flag);
	if (ret_code) {
		pr_err("recvd ret_code(%d)\n", ret_code);
		return ret_code;
	}

	/* Set duplication */
	ret_code = mv_pp2x_cls_c2_dup_set(&pp2_cls_c2_entry, c2_entry->flow_info.flow_id, c2_entry->flow_info.flow_cnt);
	if (ret_code) {
		pr_err("failed to call mv_pp2x_cls_c2_dup_set\n");
		return ret_code;
	}

	/* Set C2 HEK */
	memset(hek_byte, 0, MVPP2_C2_HEK_OFF_MAX);
	memset(hek_byte_mask, 0, MVPP2_C2_HEK_OFF_MAX);
	ret_code = pp2_cls_c2_tcam_hek_get(c2_entry->mng_pkt_key->pkt_key->field_bm,
					   c2_entry,
					   hek_byte,
					   hek_byte_mask);
	if (ret_code) {
		pr_err("recvd ret_code(%d)\n", ret_code);
		return ret_code;
	}

	for (hek_offs = MVPP2_C2_HEK_OFF_PORT_ID; hek_offs >= MVPP2_C2_HEK_OFF_BYTE0; hek_offs--) {
		ret_code = mv_pp2x_cls_c2_tcam_byte_set(&pp2_cls_c2_entry,
							hek_offs,
							hek_byte[hek_offs],
							hek_byte_mask[hek_offs]);
		if (ret_code) {
			pr_err("recvd ret_code(%d)\n", ret_code);
			return ret_code;
		}
	}

	/* Write C2 entry data to HW */
	ret_code = mv_pp2x_cls_c2_hw_write(cpu_slot, c2_hw_idx, &pp2_cls_c2_entry);
	if (ret_code) {
		pr_err("recvd ret_code(%d)\n", ret_code);
		return ret_code;
	}

	return 0;
}

/*******************************************************************************
 * pp2_cls_c2_hit_cntr_clear_all
 *
 * DESCRIPTION: The routine will clear all HW hit counters
 *
 * INPUTS:
 *	None
 *
 * OUTPUTS:
 *	None
 *
 * RETURNS:
 *	0 on success, error-code otherwise
 ******************************************************************************/
int pp2_cls_c2_hit_cntr_clear_all(uintptr_t cpu_slot)
{
	int		rc;
	int		i;
	u32	cnt;

	for (i = 0; i < MVPP2_CLS_C2_TCAM_SIZE; i++) {
		rc = mv_pp2x_c2_hit_cntr_read(cpu_slot, i, &cnt);
		if (rc) {
			pr_err("recvd ret_code(%d)\n", rc);
			return rc;
		}
	}
	return 0;
}

/*******************************************************************************
 * pp2_cls_c2_hit_cntr_all_get
 *
 * DESCRIPTION: The routine returns all hit counters above threshold
 *
 * INPUTS:
 *	inst		- packet processor instance
 *	hit_low_thresh	- low threshold, hit counters above this will be returned
 *	num_of_cntrs	- size of cntr_info
 *
 * OUTPUTS:
 *	cntr_info	- returned counter array with logical and physical index
 *	num_of_cntrs	- number of updated counters in cntr_info
 *
 * RETURNS:
 *	0 on success, error-code otherwise
 ******************************************************************************/
int pp2_cls_c2_hit_cntr_all_get(struct pp2_inst *inst, int hit_low_thresh,
				struct pp2_cls_hit_cnt_t cntr_info[],
				u32 *num_of_cntrs)
{
	int i;
	u32 cnt;
	u32 rc;
	u32 cntr_idx;
	u32 phys_idx;
	struct pp2_cls_c2_index_t *c2_index_node;
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);

	if (!cntr_info) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}
	if (!num_of_cntrs) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}
	cntr_idx = 0;

	for (i = 0; i < MVPP2_CLS_C2_TCAM_SIZE - MVPP2_C2_FIRST_ENTRY; i++) {
		phys_idx = i + MVPP2_C2_FIRST_ENTRY;
		rc = mv_pp2x_c2_hit_cntr_read(cpu_slot, phys_idx, &cnt);
		if (rc) {
			pr_err("recvd ret_code(%d)\n", rc);
			return rc;
		}

		if (cnt >= hit_low_thresh) {
			if (*num_of_cntrs < cntr_idx) {
				pr_err("counter array too small, size = %d\n", *num_of_cntrs);
				return -EINVAL;
			}

			c2_index_node = pp2_cls_db_c2_index_node_get(inst, i);
			if (!c2_index_node) {
				pr_err("C2 index %d error\n", i);
				return -EINVAL;
			}

			if (c2_index_node->valid != MVPP2_C2_ENTRY_VALID) {
				pr_err("C2 index %d is not valid\n", i);
				return -EINVAL;
			}
			cntr_info[cntr_idx].log_idx = c2_index_node->c2_logic_idx;
			cntr_info[cntr_idx].phys_idx = phys_idx;
			cntr_info[cntr_idx].cntr_val = cnt;
			cntr_idx++;
		}
	}

	/* update actual counters updated */
	*num_of_cntrs = cntr_idx;

	return 0;
}

/*******************************************************************************
 * pp2_cls_c2_hit_cntr_get
 *
 * DESCRIPTION: The routine returns hit counters of C2 entry
 *
 * INPUTS:
 *	inst  - packet processor instance
 *	c2_id - entry logic index
 *
 * OUTPUTS:
 *	cntr - returned counter
 *
 * RETURNS:
 *	0 on success, error-code otherwise
 ******************************************************************************/
int pp2_cls_c2_hit_cntr_get(struct pp2_inst *inst, int	c2_id,
			    u32 *cntr)
{
	u32		hw_id;
	u32		db_id;
	u32		rc;
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);

	if (!cntr) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}
	rc = pp2_cls_c2_get_hw_idx_from_logic_idx(inst, c2_id, &hw_id, &db_id);
	if (!rc) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}
	rc = mv_pp2x_c2_hit_cntr_read(cpu_slot, hw_id, cntr);
	if (!rc) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}
	return 0;
}

/*******************************************************************************
 * pp2_cls_c2_get_hw_idx_from_logic_idx
 *
 * DESCRIPTION: The API will translate C2 entry logic index to HW index.
 * INPUTS:
 *	    inst      - packet processor instance
 *           logic_idx - C2 entry logical index
 *
 * OUTPUTS:
 *           c2_hw_idx - C2 HW entry index
 *
 * RETURNS:
 *	0 on success, error-code otherwise
 ******************************************************************************/
int pp2_cls_c2_get_hw_idx_from_logic_idx(struct pp2_inst *inst,
					 u32	logic_idx,
					 u32	*c2_hw_idx,
					 u32	*c2_db_idx)
{
	u32 index;
	struct pp2_cls_c2_index_t *c2_index_node;

	if (!c2_hw_idx) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}
	if (!c2_db_idx) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}
	/* Get the valid index node */
	for (index = 0; index < MVPP2_C2_ENTRY_MAX; index++) {
		c2_index_node = pp2_cls_db_c2_index_node_get(inst, index);
		if (!c2_index_node)
			return -ENXIO;
		if (c2_index_node->valid == MVPP2_C2_ENTRY_VALID &&
		    c2_index_node->c2_logic_idx == logic_idx)
			break;
	}
	if (index == MVPP2_C2_ENTRY_MAX) {
		*c2_hw_idx = MVPP2_C2_ENTRY_INVALID_IDX;
		return -ENXIO;
	}

	*c2_hw_idx = c2_index_node->c2_hw_idx;
	*c2_db_idx = index;

	return 0;
}

/*******************************************************************************
 * pp2_cls_c2_free_entry_number_get
 *
 * DESCRIPTION: The routine will get the free entry in C2.
 * INPUTS:
 *	    inst	      - packet processor instance
 *
 * OUTPUTS:
 *           free_entry_number - free entry entry number
 *
 * RETURNS:
 *	0 on success, error-code otherwise
 ******************************************************************************/
int pp2_cls_c2_free_entry_number_get(struct pp2_inst *inst, u32 *free_entry_number)
{
	u32 i = 0;
	struct list *list;

	if (!free_entry_number) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}
	/* Traverse free list */
	LIST_FOR_EACH(list, pp2_cls_db_c2_free_list_head_get(inst))
		i++;

	*free_entry_number = i;

	return 0;
}

/*******************************************************************************
 * pp2_cls_c2_rule_add
 *
 * DESCRIPTION: The API will add one C2 entry
 *
 * INPUTS:
 *	    inst	    - packet processor instance
 *           c2_entry        - contains all parameters needed for C2 rule adding
 *
 * OUTPUTS:
 *           c2_logic_index  - The logical index of C2 entry, used to delete the entry
 *
 * RETURNS:
 *	0 on success, error-code otherwise
 ******************************************************************************/
int pp2_cls_c2_rule_add(struct pp2_inst *inst, struct mv_pp2x_c2_add_entry *c2_entry,
			u32 *c2_logic_index)
{
	int ret_code;
	u32 c2_db_idx, c2_logic_idx, c2_hw_idx = 0;
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);

	/* Parameter check */
	ret_code = pp2_cls_c2_rule_add_check(c2_entry);
	if (ret_code) {
		pr_err("recvd ret_code(%d)\n", ret_code);
		return ret_code;
	}
	if (!c2_logic_index) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}
	/* Make C2 slot */
	ret_code = pp2_cls_c2_make_slot(inst, c2_entry->lkp_type,
					c2_entry->priority,
					&c2_hw_idx);
	if (ret_code) {
		pr_err("recvd ret_code(%d)\n", ret_code);
		return ret_code;
	}

	/* Write TCAM */
	ret_code = pp2_cls_c2_tcam_set(cpu_slot, c2_entry, c2_hw_idx);
	if (ret_code != 0) {
		/* Return slot to free list */
		pp2_cls_c2_free_list_add(inst, c2_hw_idx);
		pr_err("C2 TCAM(%d) set failed\n", c2_hw_idx);
		return ret_code;
	}

	/* Update MVPP2 DB */
	ret_code = pp2_cls_c2_data_entry_db_add(inst, c2_entry, &c2_db_idx);
	if (ret_code) {
		pr_err("recvd ret_code(%d)\n", ret_code);
		return ret_code;
	}

	/* Allocate logical index for delete */
	c2_logic_idx = pp2_cls_c2_new_logic_idx_allocate(false);

	/* Update corresponding lookup type list */
	ret_code = pp2_cls_c2_lkp_type_list_add(inst,
						c2_entry->lkp_type,
						c2_entry->priority,
						c2_hw_idx,
						c2_db_idx,
						c2_logic_idx);
	if (ret_code) {
		pr_err("recvd ret_code(%d)\n", ret_code);
		return ret_code;
	}

	/* Return logic index */
	*c2_logic_index = c2_logic_idx;

	return 0;
}

/*******************************************************************************
 * pp2_cls_c2_rule_del
 *
 * DESCRIPTION: The API will delete one C2 entry
 *
 * INPUTS:
 *	    inst	    - packet processor instance
 *           c2_logic_index  - The logical index of C2 entry to delete
 *
 * OUTPUTS:
 *           None
 *
 * RETURNS:
 *	0 on success, error-code otherwise
 ******************************************************************************/
int pp2_cls_c2_rule_del(struct pp2_inst *inst, u32 c2_logic_index)
{
	int ret_code;
	struct pp2_cls_c2_index_t *c2_index_node;
	u32 c2_hw_idx, c2_db_idx, node_idx;
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);

	/* Get HW index and DB index */
	for (node_idx = 0; node_idx < MVPP2_C2_ENTRY_MAX; node_idx++) {
		c2_index_node = pp2_cls_db_c2_index_node_get(inst, node_idx);
		if (!c2_index_node) {
			pr_err("Invalid rule index(%d)\n", c2_logic_index);
			return -ENXIO;
		}
		if (c2_index_node->valid == MVPP2_C2_ENTRY_VALID &&
		    c2_index_node->c2_logic_idx == c2_logic_index)
			break;
	}
	if (node_idx == MVPP2_C2_ENTRY_MAX) {
		pr_err("Invalid rule index(%d)\n", c2_logic_index);
		return -ENXIO;
	}

	c2_hw_idx = c2_index_node->c2_hw_idx;
	c2_db_idx = c2_index_node->c2_data_db_idx;

	/* Delete HW entry */
	if (mv_pp2x_cls_c2_hw_inv(cpu_slot, c2_hw_idx)) {
		pr_err("Failed to invalid C2 TCAM entry(%d)\n", c2_hw_idx);
		return -EIO;
	}

	/* Invalid data db entry */
	ret_code = pp2_cls_c2_data_entry_db_del(inst, c2_db_idx);
	if (ret_code) {
		pr_err("recvd ret_code(%d)\n", ret_code);
		return ret_code;
	}

	/* delete index node from lkp type list, add to free list */
	list_del(&c2_index_node->list_node);
	memset(c2_index_node, 0, sizeof(struct pp2_cls_c2_index_t));
	ret_code = pp2_cls_c2_free_list_add(inst, c2_hw_idx);
	if (ret_code) {
		pr_err("recvd ret_code(%d)\n", ret_code);
		return ret_code;
	}

	return 0;
}

/*******************************************************************************
 * pp2_cls_c2_rule_sram_get
 *
 * DESCRIPTION: The routine will get a SRAM of a existed rule.
 *
 * INPUTS:
 *	inst	  - packet processor instance
 *	logic_idx - C2 logical index
 *
 * OUTPUTS:
 *	sram      - C2 new sram
 *
 * RETURNS:
 *	0 on success, error-code otherwise
 ******************************************************************************/
int pp2_cls_c2_rule_sram_get(struct pp2_inst *inst, u32 logic_index,
			     struct pp2_cls_engine_sram_t *sram)
{
	int ret_code;
	struct pp2_cls_c2_data_t *c2_data;		/*use heap to reduce stack size*/
	u32 hw_idx, db_idx;

	if (!sram) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}
	/* Get C2 db idx */
	ret_code = pp2_cls_c2_get_hw_idx_from_logic_idx(inst,
							logic_index,
							&hw_idx,
							&db_idx);
	if (ret_code) {
		pr_err("C2 DB entry index get fail\n");
		return ret_code;
	}
	c2_data = kmalloc(sizeof(*c2_data), GFP_KERNEL);
	if (!c2_data)
		return -ENOMEM;

	memset(c2_data, 0, sizeof(struct pp2_cls_c2_data_t));

	/* Get DB data */
	ret_code = pp2_cls_db_c2_data_get(inst, db_idx, c2_data);
	if (ret_code) {
		pr_err("C2 DB entry get fail\n");
		free(c2_data);
		return ret_code;
	}
	/* Get SRAM */
	memcpy(&sram->action, &c2_data->action, sizeof(struct mv_pp2x_engine_pkt_action));
	memcpy(&sram->qos_info, &c2_data->qos_info, sizeof(struct mv_pp2x_engine_qos_info));
	memcpy(&sram->qos_value, &c2_data->qos_value, sizeof(struct mv_pp2x_qos_value));
	memcpy(&sram->pkt_mod, &c2_data->pkt_mod, sizeof(struct mv_pp2x_engine_pkt_mod));
	memcpy(&sram->dup_info, &c2_data->flow_info, sizeof(struct mv_pp2x_duplicate_info));

	free(c2_data);
	return 0;
}

/*******************************************************************************
 * pp2_cls_c2_rule_sram_update
 *
 * DESCRIPTION: The routine will update C2 HW entry with new SRAM.
 *
 * INPUTS:
 *	inst	  - packet processor instance
 *	logic_idx - C2 logical index
 *
 * OUTPUTS:
 *	sram      - C2 new sram
 *
 * RETURNS:
 *	0 on success, error-code otherwise
 ******************************************************************************/
int pp2_cls_c2_rule_sram_update(struct pp2_inst *inst, u32 logic_index,
				struct pp2_cls_engine_sram_t *sram)
{
	int ret_code;
	struct mv_pp2x_cls_c2_entry hw_entry;
	struct pp2_cls_c2_data_t *db_entry;		/*use heap to reduce stack size*/
	u32 hw_idx, db_idx;
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);

	if (!sram) {
		pr_err("%s: null pointer\n", __func__);
		return -EFAULT;
	}

	MVPP2_MEMSET_ZERO(hw_entry);
	MVPP2_MEMSET_ZERO(db_entry);

	/* Get the HW and DB index */
	ret_code = pp2_cls_c2_get_hw_idx_from_logic_idx(inst, logic_index, &hw_idx, &db_idx);
	if (ret_code) {
		pr_err("fail to access get HW and DB index\n");
		return ret_code;
	}
	/* Read HW entry */
	ret_code = mv_pp2x_cls_c2_hw_read(cpu_slot, hw_idx, &hw_entry);
	if (ret_code) {
		pr_err("mv_pp2x_cls_c2_hw_read fail\n");
		return ret_code;
	}
	/* Update SRAM */
	/* Set QOS table, selection and ID */
	ret_code = mv_pp2x_cls_c2_qos_tbl_set(&hw_entry,
					      sram->qos_info.qos_tbl_index,
					      sram->qos_info.qos_tbl_type);
	if (ret_code) {
		pr_err("mvPp2ClsC2QosTblSet fail\n");
		return ret_code;
	}
	/* Set color, cmd and source */
	ret_code = mv_pp2x_cls_c2_color_set(&hw_entry,
					    sram->action.color_act,
					    sram->qos_info.color_src);
	if (ret_code) {
		pr_err("mvPp2ClsC2ColorSet fail\n");
		return ret_code;
	}
	/* Set priority(pbit), cmd, value(not from qos table) and source */
	ret_code = mv_pp2x_cls_c2_prio_set(&hw_entry,
					   sram->action.pri_act,
					   sram->qos_value.pri,
					   sram->qos_info.pri_dscp_src);
	if (ret_code) {
		pr_err("mvPp2ClsC2PrioSet fail\n");
		return ret_code;
	}
	/* Set DSCP, cmd, value(not from qos table) and source */
	ret_code = mv_pp2x_cls_c2_dscp_set(&hw_entry,
					   sram->action.dscp_act,
					   sram->qos_value.dscp,
					   sram->qos_info.pri_dscp_src);
	if (ret_code) {
		pr_err("mvPp2ClsC2DscpSet fail\n");
		return ret_code;
	}
	/* Set gemport ID, cmd, value, and source */
	ret_code = mv_pp2x_cls_c2_gpid_set(&hw_entry,
					   sram->action.gemp_act,
					   sram->qos_value.gemp,
					   sram->qos_info.gemport_src);
	if (ret_code) {
		pr_err("mv_pp2x_cls_c2_gpid_set fail\n");
		return ret_code;
	}
	/* Set queue low, cmd, value, and source */
	ret_code = mv_pp2x_cls_c2_queue_low_set(&hw_entry,
						sram->action.q_low_act,
						sram->qos_value.q_low,
						sram->qos_info.q_low_src);
	if (ret_code) {
		pr_err("mvPp2ClsC2QueueLowSet fail\n");
		return ret_code;
	}
	/* Set queue high, cmd, value and source */
	ret_code = mv_pp2x_cls_c2_queue_high_set(&hw_entry,
						 sram->action.q_high_act,
						 sram->qos_value.q_high,
						 sram->qos_info.q_high_src);
	if (ret_code) {
		pr_err("mvPp2ClsC2QueueHighSet fail\n");
		return ret_code;
	}
	/* Set forward */
	ret_code = mv_pp2x_cls_c2_forward_set(&hw_entry,
					      sram->action.frwd_act);
	if (ret_code) {
		pr_err("mvPp2ClsC2ForwardSet fail\n");
		return ret_code;
	}
	/* Set policer */
	/* TODO - to be added later */
	/*
	 * ret_code = mv_pp2x_cls_c2_policer_set(&hw_entry,
	 *				sram->action.policer_act,
	 *				sram->qos_info.policer_id & MVPP2_CLS2_ACT_DUP_ATTR_PLCRID_MAX,
	 *				MVPP2_POLICER_2_BANK(sram->qos_info.policer_id));
	 * if (ret_code) {
	 *	pr_err("mv_pp2x_cls_c2_policer_set fail\n");
	 *	return ret_code;
	 * }
	 */
	/* Set flowID */
	ret_code = mv_pp2x_cls_c2_flow_id_en(&hw_entry,
					     sram->action.flowid_act);
	if (ret_code) {
		pr_err("mvPp2ClsC2FlowIDSet fail\n");
		return ret_code;
	}
	/* Set modification info */
	ret_code = mv_pp2x_cls_c2_mod_set(&hw_entry,
					  sram->pkt_mod.mod_data_idx,
					  sram->pkt_mod.mod_cmd_idx,
					  sram->pkt_mod.l4_chksum_update_flag);
	if (ret_code) {
		pr_err("mv_pp2x_cls_c2_mod_set fail\n");
		return ret_code;
	}
	/* set duplication */
	ret_code = mv_pp2x_cls_c2_dup_set(&hw_entry, sram->dup_info.flow_id, sram->dup_info.flow_cnt);
	if (ret_code) {
		pr_err("failed to call mv_pp2x_cls_c2_dup_set\n");
		return ret_code;
	}
	/* set seqence instruction info */
	ret_code = mv_pp2x_cls_c2_seq_set(&hw_entry, 0, sram->instr_info.instr_value.instr_low & 0xff);
	if (ret_code) {
		pr_err("failed to call mv_pp2x_cls_c2_seq_set\n");
		return ret_code;
	}
	/* Write to HW again */
	ret_code = mv_pp2x_cls_c2_hw_write(cpu_slot, hw_idx, &hw_entry);
	if (ret_code) {
		pr_err("failed to call mv_pp2x_cls_c2_hw_write\n");
		return ret_code;
	}
	db_entry = kmalloc(sizeof(*db_entry), GFP_KERNEL);
	if (!db_entry)
		return -ENOMEM;

	memset(db_entry, 0, sizeof(struct pp2_cls_c2_data_t));

	/* Update DB */
	ret_code = pp2_cls_db_c2_data_get(inst, db_idx, db_entry);
	if (ret_code) {
		pr_err("failed to read DB\n");
		free(db_entry);
		return ret_code;
	}

	memcpy(&db_entry->action, &sram->action, sizeof(struct mv_pp2x_engine_pkt_action));
	memcpy(&db_entry->qos_info, &sram->qos_info, sizeof(struct mv_pp2x_engine_qos_info));
	memcpy(&db_entry->qos_value, &sram->qos_value, sizeof(struct mv_pp2x_qos_value));
	memcpy(&db_entry->pkt_mod, &sram->pkt_mod, sizeof(struct mv_pp2x_engine_pkt_mod));
	memcpy(&db_entry->flow_info, &sram->dup_info, sizeof(struct mv_pp2x_duplicate_info));

	ret_code = pp2_cls_db_c2_data_set(inst, db_idx, db_entry);
	if (ret_code) {
		pr_err("failed to set DB\n");
		free(db_entry);
		return ret_code;
	}

	free(db_entry);
	return 0;
}

/*******************************************************************************
 * pp2_cls_c2_reset
 *
 * DESCRIPTION: The API will clean all the entries in C2 engine,
 *              and clear C2 sub-module DB at the same time
 *
 * INPUTS:
 *	    inst   - packet processor instance
 *
 * OUTPUTS:
 *           None
 * RETURNS:
 *	0 on success, error-code otherwise
 *
 * COMMENTS:
 *           None.
 ******************************************************************************/
int pp2_cls_c2_reset(struct pp2_inst *inst)
{
	int ret_code;
	int index;
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);

	/* Clear all TCAM entry, except last one added by LSP */
	for (index = MVPP2_C2_FIRST_ENTRY; index < MVPP2_CLS_C2_TCAM_SIZE; index++)
		mv_pp2x_cls_c2_hw_inv(cpu_slot, index);

	/* Clear MVPP2 C2 DB */
	ret_code = pp2_cls_db_c2_init(inst);
	if (ret_code) {
		pr_err("recvd ret_code(%d)\n", ret_code);
		return ret_code;
	}

	/* Reset logic index */
	pp2_cls_c2_new_logic_idx_allocate(true);

	return 0;
}

/*******************************************************************************
 * pp2_cls_c2_start
 *
 * DESCRIPTION: The API will do following operations:
 *              1)Mark every C2 related DB entry invalid, C2_ENTRY_IDX_TBL and C2_ENTRY_DATA_TBL
 *              2)Add all the C2 DB entry in free list, except last one, 255
 * INPUTS:
 *	    inst   - packet processor instance
 *
 * OUTPUTS:
 *           None
 *
 * RETURNS:
 *	0 on success, error-code otherwise
 * COMMENTS:
 *           It is called by pp2_cls_start.
 ******************************************************************************/
int pp2_cls_c2_start(struct pp2_inst *inst)
{
	if (pp2_cls_c2_reset(inst)) {
		pr_err("MVPP2 C2 start failed\n");
		return -EINVAL;
	}

	return 0;
}
