/******************************************************************************
	Copyright (C) 2016 Marvell International Ltd.

  If you received this File from Marvell, you may opt to use, redistribute
  and/or modify this File under the following licensing terms.
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

	* Redistributions of source code must retain the above copyright
	  notice, this list of conditions and the following disclaimer.

	* Redistributions in binary form must reproduce the above copyright
	  notice, this list of conditions and the following disclaimer in the
	  documentation and/or other materials provided with the distribution.

	* Neither the name of Marvell nor the names of its contributors may be
	  used to endorse or promote products derived from this software
	  without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#ifndef __MV_PP2_CLS_H__
#define __MV_PP2_CLS_H__

#include "mv_std.h"
#include "mv_net.h"

struct pp2_cls_tbl;

#define PP2_CLS_TBL_MAX_NUM_FIELDS	5

enum pp2_cls_tbl_type {
	PP2_CLS_TBL_EXACT_MATCH = 0,
	PP2_CLS_TBL_MASKABLE
};

/**
 * TODO: Future
 * enum pp2_cls_tbl_statistics_mode {
 *	PP2_CLS_TBL_STATS_M_NONE = 0,
 *	PP2_CLS_TBL_STATS_M_FRM,
 *	PP2_CLS_TBL_STATS_M_BYTES
 * };
 */

enum pp2_cls_tbl_action_type {
	PP2_CLS_TBL_ACT_DROP = 0,
	PP2_CLS_TBL_ACT_DONE,
	/* TODO: PP2_CLS_TBL_ACT_LU */
};

struct pp2_cls_cos_desc {
	struct pp2_ppio	*ppio;
	u8		 tc;
};

struct pp2_cls_tbl_action {
	enum pp2_cls_tbl_action_type		type;
	/* TODO: struct pp2_cls_tbl			*next_tbl; */
	/**< valid only in case of next-action is LU */
	/* TODO: enum pp2_cls_tbl_mark_type	 mark_type; */
	/*
	* TODO :
	* union {
	*	u16			 flow_id;
	*	u16			 qos;
	* } u;
	*/
	/** 'NULL' value means no-cos change; i.e. keep original cos */
	struct pp2_cls_cos_desc		*cos;
};

struct pp2_cls_tbl_key {
	u8				key_size;
	u8				num_fields;
	struct {
		enum mv_net_proto		proto;
		union mv_net_proto_fields	field;
	} proto_field[PP2_CLS_TBL_MAX_NUM_FIELDS];
};

struct pp2_cls_tbl_params {
	enum pp2_cls_tbl_type			 type;
	u16					 max_num_rules;
	struct pp2_cls_tbl_key			 key;
	/* TODO: enum pp2_cls_tbl_statistics_mode	 stats_mode; */
	/* TODO: enum pp2_cls_tbl_aging_mode	 aging_mode; */
	/* TODO: enum pp2_cls_tbl_priority_mode	 prio_mode; */
	struct pp2_cls_tbl_action		 default_act;
};

/**
 * Add a classifier rule
 *
 * @param[in]	*params		A pointer to the classifier table parameters
 *
 * @param[out]	**tbl		A pointer to an allocated classifier table
 *
 * @retval		0 on success
 * @retval		error-code otherwise
 */
int pp2_cls_tbl_init(struct pp2_cls_tbl_params *params, struct pp2_cls_tbl **tbl);

/**
 * Deinit a classifier table object
 *
 * @param[in]	*tbl		A pointer to a classifier table object
 *
 */
void pp2_cls_tbl_deinit(struct pp2_cls_tbl *tbl);

struct pp2_cls_rule_key_field {
	u8	size;
	u8	*key;
	u8	*mask;
};

/*TODO : Add union to support future API's */

struct pp2_cls_tbl_rule {
	u8	num_fields;
	struct pp2_cls_rule_key_field fields[PP2_CLS_TBL_MAX_NUM_FIELDS];
};

/**
 * Add a classifier rule
 *
 * @param[in]	tbl		A pointer to a classifier table object
 * @param[in]	rule		A pointer to a classifier rule
 * @param[in]	action		A pointer to a classifier action
 *
 * @retval	0 on success
 * @retval	positive value - reserved
 * @retval	error-code otherwise (negative value)
 */
int pp2_cls_tbl_add_rule(struct pp2_cls_tbl		*tbl,
			 struct pp2_cls_tbl_rule	*rule,
			 struct pp2_cls_tbl_action	*action);

/**
 * Modify the action of an existing classifier rule
 *
 * The rule must be already in place
 *
 * @param[in]	tbl		A pointer to a classifier table object
 * @param[in]	rule		A pointer to a classifier rule
 * @param[in]	action		A pointer to a classifier action
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_cls_tbl_modify_rule(struct pp2_cls_tbl		*tbl,
			    struct pp2_cls_tbl_rule	*rule,
			    struct pp2_cls_tbl_action	*action);

/**
 * Remove a classifier rule
 *
 * @param[in]	tbl		A pointer to a classifier table object
 * @param[in]	rule		A pointer to a classifier rule
 *
 * @retval		0 on success
 * @retval		error-code otherwise
 */
int pp2_cls_tbl_remove_rule(struct pp2_cls_tbl		*tbl,
			    struct pp2_cls_tbl_rule	*rule);

#endif /* __MV_PP2_CLS_H__ */
