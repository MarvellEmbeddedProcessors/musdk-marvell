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


#define PP2_CLS_TBL_MAX_NUM_FIELDS	12


enum pp2_cls_tbl_type {
	PP2_CLS_TBL_T_EM = 0,
	PP2_CLS_TBL_T_EM_w_MASK
};

enum pp2_cls_tbl_statistics_mode {
	PP2_CLS_TBL_STATS_M_NONE = 0,
	PP2_CLS_TBL_STATS_M_FRM,
	PP2_CLS_TBL_STATS_M_BYTES
};

enum pp2_cls_tbl_mark_type {
	PP2_CLS_TBL_MARK_T_NONE = 0,
	PP2_CLS_TBL_MARK_T_FLOW_ID,
	PP2_CLS_TBL_MARK_T_QOS
};

enum pp2_cls_tbl_act {
	PP2_CLS_TBL_ACT_T_DROP = 0,
	PP2_CLS_TBL_ACT_T_LU,
	PP2_CLS_TBL_ACT_T_DONE,
};

struct cls_cos_desc {
	struct pp2_ppio	*ppio;
	u8		 tc;
};

struct pp2_cls_tbl_entry_action {
	enum pp2_cls_tbl_act		 next_act;
	struct pp2_cls_tbl		*next_tbl;
	/**< valid only in case of next-action is LU */

#if 0 /* future ... */
	enum pp2_cls_tbl_mark_type	 mark_type;
	union {
		u16			 flow_id;
		u16			 qos;
	} u;
#endif /* 0 */

	/* 'NULL' value means no-cos change; i.e. keep original cos */
	struct cls_cos_desc		*cos;
};

struct pp2_cls_tbl_key {
	u8				 key_size;
	u8				 num_fields;
	struct {
		enum mv_net_prtcl	 prtcl;
		mv_net_prtcl_field_t	 field;
	} prtcl_fld[PP2_CLS_TBL_MAX_NUM_FIELDS];
};

struct pp2_cls_tbl_params {
	enum pp2_cls_tbl_type			 type;
	u16					 max_num_rules;
	struct pp2_cls_tbl_key			 key;

	enum pp2_cls_tbl_statistics_mode	 stats_mode;

	/* TODO: enum pp2_cls_tbl_aging_mode	 aging_mode; */
	/* TODO: enum pp2_cls_tbl_priority_mode	 prio_mode; */

	struct pp2_cls_tbl_entry_action		 dflt_act;
};

int pp2_cls_tbl_init(struct pp2_cls_tbl_params *params, struct pp2_cls_tbl **tbl);
void pp2_cls_tbl_deinit(struct pp2_cls_tbl *tbl);

struct pp2_cls_tbl_rule {
	union {
		struct {
			u8	size;
			void	*key;
		} em;
		struct {
			u8	size;
			void	*key;
			void	*mask;
		} em_w_mask;
	} key;
};

int pp2_cls_tbl_add_rule(struct pp2_cls_tbl			*tbl,
			 struct pp2_cls_tbl_rule		*rule,
			 struct pp2_cls_tbl_entry_action	*action);
int pp2_cls_tbl_modify_rule(struct pp2_cls_tbl			*tbl,
			    struct pp2_cls_tbl_rule		*rule,
			    struct pp2_cls_tbl_entry_action	*action);
int pp2_cls_tbl_remove_rule(struct pp2_cls_tbl			*tbl,
			    struct pp2_cls_tbl_rule		*rule);

#endif /* __MV_PP2_CLS_H__ */
