
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

/********************************************************************************/
/*			MACROS							*/
/********************************************************************************/
/*PP2 CLS DB init module definition */
#define PP2_CLS_DB_INIT_INVALID_VALUE	(0)	/* Default PP2 CLS DB invalid value	*/

/********************************************************************************/
/*			ENUMERATIONS						*/
/********************************************************************************/

/********************************************************************************/
/*			STRUCTURES						*/
/********************************************************************************/

/* C3 module db structure */
struct pp2_cls_db_c3_t {
	struct pp2_cls_c3_scan_config_t		scan_config;					/* scan config       */
	u32					max_search_depth;				/* max search depth  */
	struct pp2_cls_c3_hash_index_entry_t	hash_idx_tbl[MVPP2_CLS_C3_HASH_TBL_SIZE];	/* tbl for hash idx  */
	struct pp2_cls_c3_logic_index_entry_t	logic_idx_tbl[MVPP2_CLS_C3_HASH_TBL_SIZE];	/* tbl for logic idx */
};

struct pp2_cls_db_t {
	enum pp2_cls_module_state_t	pp2_cls_module_init_state;	/* PP2_CLS module init state	*/
	struct pp2_cls_db_c3_t	c3_db;			/* PP2_CLS module C3 db		*/
};

/********************************************************************************/
/*			PROTOTYPE						*/
/********************************************************************************/
/* PP2_CLS init section */
int pp2_cls_db_module_state_set(enum pp2_cls_module_state_t state);
u32 pp2_cls_db_module_state_get(void);

/* C3 section */
int pp2_cls_db_c3_free_logic_idx_get(u32 *logic_idx);
int pp2_cls_db_c3_entry_add(u32 logic_idx, u32 hash_idx);
int pp2_cls_db_c3_entry_del(int logic_idx);
int pp2_cls_db_c3_hash_idx_get(u32 logic_idx, u32 *hash_idx);
int pp2_cls_db_c3_logic_idx_get(int hash_idx, int *logic_idx);
int pp2_cls_db_c3_hash_idx_update(struct pp2_cls_c3_hash_pair *hash_pair_arr);
int pp2_cls_db_c3_scan_param_set(struct pp2_cls_c3_scan_config_t *scan_config);
int pp2_cls_db_c3_scan_param_get(struct pp2_cls_c3_scan_config_t *scan_config);
int pp2_cls_db_c3_search_depth_set(u32 search_depth);
int pp2_cls_db_c3_search_depth_get(u32 *search_depth);
int pp2_cls_db_c3_init(void);

/* DB general section */
int pp2_cls_db_init(void);
int pp2_cls_db_exit(void);

#endif /* _PP2_CLS_DB_H_ */
