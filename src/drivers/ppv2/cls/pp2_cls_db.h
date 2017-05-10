
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
 * @file pp2_cls_db.h
 *
 *  pp2_cls_db definitions
 */

/***********************/
/* h file declarations */
/***********************/
#ifndef _PP2_CLS_DB_H_
#define _PP2_CLS_DB_H_

#include "drivers/mv_pp2_cls.h"

#define MVPP2_MNG_FLOW_ID_MAX 50
/********************************************************************************/
/*			MACROS							*/
/********************************************************************************/
/*PP2 CLS DB init module definition */
#define MVPP2_CLS_DB_INIT_INVALID_VALUE	(0)	/* Default PP2 CLS DB invalid value	*/

/********************************************************************************/
/*			ENUMERATIONS						*/
/********************************************************************************/

enum pp2_cls_params_tbl_type {
	PP2_CLS_FLOW_TBL,
	PP2_CLS_QOS_TBL
};

/********************************************************************************/
/*			STRUCTURES						*/
/********************************************************************************/

/* C2 module db structure */
struct pp2_cls_db_c2_t {
	/* info of each entry in C2 engine */
	struct pp2_cls_c2_data_t c2_data_db[MVPP2_C2_ENTRY_MAX - MVPP2_C2_FIRST_ENTRY];
	/* logic index and hw index of C2 entry */
	struct pp2_cls_c2_index_t c2_index_db[MVPP2_C2_ENTRY_MAX - MVPP2_C2_FIRST_ENTRY];
	/* header of list of the valid lookup_types */
	struct list c2_lu_type_head_db[MVPP2_C2_LKP_TYPE_MAX];
	/* header of free C2 entry list */
	struct list c2_free_head_db;
};

/* C3 module db structure */
struct pp2_cls_db_c3_t {
	struct pp2_cls_c3_scan_config_t		scan_config;					/* scan config       */
	u32					max_search_depth;				/* max search depth  */
	struct pp2_cls_c3_hash_index_entry_t	hash_idx_tbl[MVPP2_CLS_C3_HASH_TBL_SIZE];	/* tbl for hash idx  */
	struct pp2_cls_c3_logic_index_entry_t	logic_idx_tbl[MVPP2_CLS_C3_HASH_TBL_SIZE];	/* tbl for logic idx */
};

/* CLS module db structure */
struct pp2_db_cls_fl_ctrl_t {
	u16			fl_max_len;			/* the max flow length		*/
	u16			lkp_dcod_en;			/* swap section index		*/
	u16			f_start;			/* free start index		*/
	u16			f_end;				/* free end index		*/
};

struct pp2_db_cls_lkp_dcod_t {
	bool			enabled;			/* enabled flag			*/
	u8			cpu_q;				/* CPU queue			*/
	u8			way;				/* entry way			*/
	u8			flow_alloc_len;			/* flow allocation length	*/
	u8			flow_len;			/* flow current length		*/
	u16			flow_off;			/* flow offset			*/
	u16			luid_num;			/* Lookup ID number		*/
	struct pp2_cls_luid_conf_t	luid_list[MVPP2_CLS_LOG_FLOW_LUID_MAX];/* Lookup ID list		*/
};

struct pp2_db_cls_fl_rule_t {
	u16			port_type;			/* port type			*/
	u16			port_bm;			/* port bitmap			*/
	u16			lu_type;	/* lookup type			*/
	bool				enabled;		/* enable flag			*/
	u8			prio;	/* HW priority			*/
	u8			engine;	/* rule engine			*/
	u8			udf7;
	u8			field_id_cnt;			/* field ID count		*/
	u8			field_id[MVPP2_FLOW_FIELD_COUNT_MAX];/* field IDs			*/
	u16			ref_cnt[MVPP2_MAX_NUM_GMACS];	/* reference count		*/
	u16			rl_log_id;			/* rule logical id              */
};

struct pp2_db_cls_fl_rule_list_t {
	struct pp2_db_cls_fl_rule_t	flow[MVPP2_CLS_FLOW_RULE_MAX];	/* flow rules			*/
	u16			flow_len;			/* flow length			*/
};

struct pp2_cls_db_cls_t {
	struct pp2_db_cls_fl_ctrl_t	fl_ctrl;			/* flow control DB		*/
	struct pp2_db_cls_fl_rule_t	fl_rule[MVPP2_FLOW_TBL_SIZE];/*CLS rule DB		*/
	u16			log2off[MVPP2_CLS_LOG2OFF_TBL_SIZE];/* logical rule ID to offset	*/
	struct pp2_db_cls_lkp_dcod_t	lkp_dcod[MVPP2_MNG_FLOW_ID_MAX];	/* lookup decode DB	*/
};

struct pp2_cls_db_prs_t {
	struct mv_pp2x_prs_shadow *prs_shadow;
};

struct rss_tbl_map_t {
	u16 hw_tbl;
	u16 num_in_q;
};

struct pp2_cls_db_rss_t {
	u32 num_musdk_tbls;					/* number of RSS tables required by MUSDK */
	u32 num_kernel_rsrvd_tbls;				/* number of RSS tables reserved by kernel */
	struct rss_tbl_map_t rss_tbl_map[MVPP22_RSS_TBL_NUM];	/* RSS table mapping for MUSDK RSS tables */
};

struct pp2_cls_db_t {
	struct pp2_cls_db_c2_t	c2_db;			/* PP2_CLS module C2 db		*/
	struct pp2_cls_db_c3_t	c3_db;			/* PP2_CLS module C3 db		*/
	struct pp2_cls_db_cls_t	cls_db;			/* PP2_CLS module CLS db		*/
	struct pp2_cls_db_prs_t	prs_db;			/* PP2_CLS module PARSER db	*/
	struct pp2_cls_db_rss_t	rss_db;			/* PP2_CLS module RSS db		*/
};

/* table db is not instance dependent, so it is defined separately in db */

struct pp2_cls_rule_node {
	struct pp2_cls_tbl_rule		rule;
	u32				logic_index;	/* Logical index in C2 or C3 database */
	struct pp2_cls_tbl_action	action;
	struct list			list_node;
};

struct pp2_cls_tbl {
	enum pp2_cls_params_tbl_type	type;
	struct pp2_cls_tbl_params	params;
	struct pp2_cls_qos_tbl_params	qos_params;
};

struct pp2_cls_tbl_node {
	struct pp2_cls_tbl		tbl;
	struct list			list_node;
	struct list			pp2_cls_tbl_rule_head;
	struct pp2_cls_rule_node	rule_node;
};

struct pp2_cls_db_mng_t {
	struct list			pp2_cls_tbl_head;
};

/********************************************************************************/
/*			PROTOTYPE						*/
/********************************************************************************/

/* C2 section */
struct list *pp2_cls_db_c2_lkp_type_list_head_get(struct pp2_inst *inst, u8 lkp_type);
struct list *pp2_cls_db_c2_free_list_head_get(struct pp2_inst *inst);
struct pp2_cls_c2_index_t *pp2_cls_db_c2_index_node_get(struct pp2_inst *inst, u32 c2_node_idx);
int pp2_cls_db_c2_index_node_set(struct pp2_inst *inst, u32 c2_node_idx, struct pp2_cls_c2_index_t *c2_index_node);
int pp2_cls_db_c2_data_get(struct pp2_inst *inst, u32 c2_db_idx, struct pp2_cls_c2_data_t *c2_data);
int pp2_cls_db_c2_data_set(struct pp2_inst *inst, u32 c2_db_idx, struct pp2_cls_c2_data_t *c2_data);
int pp2_cls_db_c2_init(struct pp2_inst *inst);

/* C3 section */
int pp2_cls_db_c3_free_logic_idx_get(struct pp2_inst *inst, u32 *logic_idx);
int pp2_cls_db_c3_entry_add(struct pp2_inst *inst, u32 logic_idx, u32 hash_idx);
int pp2_cls_db_c3_entry_del(struct pp2_inst *inst, int logic_idx);
int pp2_cls_db_c3_hash_idx_get(struct pp2_inst *inst, u32 logic_idx, u32 *hash_idx);
int pp2_cls_db_c3_logic_idx_get(struct pp2_inst *inst, int hash_idx, int *logic_idx);
int pp2_cls_db_c3_hash_idx_update(struct pp2_inst *inst, struct pp2_cls_c3_hash_pair *hash_pair_arr);
int pp2_cls_db_c3_scan_param_set(struct pp2_inst *inst, struct pp2_cls_c3_scan_config_t *scan_config);
int pp2_cls_db_c3_scan_param_get(struct pp2_inst *inst, struct pp2_cls_c3_scan_config_t *scan_config);
int pp2_cls_db_c3_search_depth_set(struct pp2_inst *inst, u32 search_depth);
int pp2_cls_db_c3_search_depth_get(struct pp2_inst *inst, u32 *search_depth);
int pp2_cls_db_c3_init(struct pp2_inst *inst);

/* CLS section */
void pp2_db_cls_init(struct pp2_inst *inst);
int  pp2_db_cls_fl_ctrl_set(struct pp2_inst *inst, struct pp2_db_cls_fl_ctrl_t *fl_ctrl);
int  pp2_db_cls_fl_ctrl_get(struct pp2_inst *inst, struct pp2_db_cls_fl_ctrl_t *fl_ctrl);
int  pp2_db_cls_fl_rule_set(struct pp2_inst *inst, u32 idx, struct pp2_db_cls_fl_rule_t *fl_rule);
int  pp2_db_cls_fl_rule_get(struct pp2_inst *inst, u32 idx, struct pp2_db_cls_fl_rule_t *fl_rule);
int  pp2_db_cls_fl_rule_list_get(struct pp2_inst *inst, u32 idx, u32 len, struct pp2_db_cls_fl_rule_t *fl_rl_list);
int  pp2_db_cls_lkp_dcod_set(struct pp2_inst *inst, u32 idx, struct pp2_db_cls_lkp_dcod_t *lkp_dcod);
int  pp2_db_cls_lkp_dcod_get(struct pp2_inst *inst, u32 idx, struct pp2_db_cls_lkp_dcod_t *lkp_dcod);
int  pp2_db_cls_rl_off_lkp_dcod_get(struct pp2_inst *inst, u16 rl_off, struct pp2_db_cls_lkp_dcod_t	*lkp_dcod);
void pp2_db_cls_rl_off_init(void);
int  pp2_db_cls_rl_off_free_nr(struct pp2_inst *inst, u32 *free_nr);
int  pp2_db_cls_rl_off_free_set(struct pp2_inst *inst, u16 off, u16 *log);
int  pp2_db_cls_rl_off_get(struct pp2_inst *inst, u16 *off, u16 log);
int  pp2_db_cls_rl_off_set(struct pp2_inst *inst, u16 off, u16 log);

/* CLS Manager section */
int pp2_cls_db_mng_init(void);
int pp2_cls_db_mng_tbl_add(struct pp2_cls_tbl **tbl);
int pp2_cls_db_mng_tbl_remove(struct pp2_cls_tbl *tbl);
int pp2_cls_db_mng_tbl_check(struct pp2_cls_tbl *tbl);
int pp2_cls_db_mng_tbl_num_get(void);
int pp2_cls_db_mng_tbl_rule_add(struct pp2_cls_tbl *tbl, struct pp2_cls_tbl_rule **rule, u32 logic_index,
				struct pp2_cls_tbl_action **action);
int pp2_cls_db_mng_rule_check(struct pp2_cls_tbl *tbl, struct pp2_cls_tbl_rule *rule);
int pp2_cls_db_mng_tbl_rule_remove(struct pp2_cls_tbl *tbl, struct pp2_cls_tbl_rule *rule, u32 *logic_index);
int pp2_cls_db_mng_tbl_rule_next_get(struct pp2_cls_tbl *tbl, struct pp2_cls_tbl_rule **rule);
int pp2_cls_db_mng_rule_list_dump(struct pp2_cls_tbl *tbl);
int pp2_cls_db_mng_tbl_list_dump(void);
int pp2_cls_db_mng_qos_tbl_add(struct pp2_cls_tbl **tbl);

/* RSS section */
int pp2_cls_db_rss_init(struct pp2_inst *inst);
int pp2_cls_db_rss_check_tbl_entry(u8 num_in_q);
void pp2_cls_db_rss_kernel_rsvd_tbl_set(struct pp2_inst *inst, u16 kernel_rss_tbl);
u16 pp2_cls_db_rss_kernel_rsvd_tbl_get(struct pp2_inst *inst);
void pp2_cls_db_rss_num_musdk_tbl_set(struct pp2_inst *inst, u16 num_musdk_tbl);
u16 pp2_cls_db_rss_num_musdk_tbl_get(struct pp2_inst *inst);
int pp2_cls_db_rss_tbl_map_set(struct pp2_inst *inst, u16 idx, u16 hw_tbl, u16 num_in_q);
int pp2_cls_db_rss_tbl_map_get(struct pp2_inst *inst, u16 idx, u16 *hw_tbl, u16 *num_in_q);
int pp2_cls_db_rss_get_hw_tbl_from_in_q(struct pp2_inst *inst, u8 num_in_q);
int pp2_cls_db_rss_tbl_map_get_next_free_idx(struct pp2_inst *inst);

/* DB general section */
int pp2_cls_db_init(struct pp2_inst *inst);
int pp2_cls_db_exit(struct pp2_inst *inst);

#endif /* _PP2_CLS_DB_H_ */
