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
 * @file pp2_c3.h
 *
 * internal and external definitions for C3 High level routines 
 */

/***********************/
/* h file declarations */
/***********************/
#ifndef _PP2_CLS_C3_H_
#define _PP2_CLS_C3_H_

/********************************************************************************/
/*			MACROS							*/
/********************************************************************************/
#define MVPP2_C3_INVALID_ENTRY_NUM	(0x1FFF)	/* invalid C3 entry number	*/
#define MVPP2_C3_MAX_SEARCH_DEPTH	(16)		/* max cuckoo search depth	*/
#define MVPP2_C3_DEFAULT_SEARCH_DEPTH	(3)		/* default cuckoo search depth	*/
#define MVPP2_C3_MAX_HASH_KEY_SIZE	(MVPP2_CLS_C3_EXT_HEK_WORDS * WORD_BYTES)	/* max key size */

/********************************************************************************/
/*			ENUMERATIONS						*/
/********************************************************************************/
enum pp2_cls_scan_mode_t {
	MVPP2_SCAN_BELOW_THRESHOLD = 0,	/* scan the entries whose hit counter are below theshold	*/
	MVPP2_SCAN_ABOVE_THRESHOLD	/* scan the entries whose hit counter are above theshold	*/
};

enum pp2_cls_c3_db_entry_valid_t {
	MVPP2_C3_ENTRY_INVALID = 0,	/* invalid C3 entry	*/
	MVPP2_C3_ENTRY_VALID		/* valid C3 entry	*/
};

/* below enums are used by C3 debug */
enum pp2_cls_c3_port_dump_idx_t {
	MVPP2_C3_PORT_DUMP_TYPE = 0,
	MVPP2_C3_PORT_DUMP_VALUE
};

enum pp2_cls_c3_hek_dump_idx_t {
	MVPP2_C3_HEK_DUMP_LEN = 0,
	MVPP2_C3_HEK_DUMP_KEY1,
	MVPP2_C3_HEK_DUMP_KEY2,
	MVPP2_C3_HEK_DUMP_KEY3
};

enum pp2_cls_c3_act_dump_idx_t {
	MVPP2_C3_ACT_DUMP_COLOR = 0,
	MVPP2_C3_ACT_DUMP_QUEUE_LOW,
	MVPP2_C3_ACT_DUMP_QUEUE_HIGH,
	MVPP2_C3_ACT_DUMP_FRWD,
	MVPP2_C3_ACT_DUMP_POLICER,
	MVPP2_C3_ACT_DUMP_FLOWID
};

enum pp2_cls_c3_qos_dump_idx_t {
	MVPP2_C3_QOS_DUMP_QUEUE_LOW = 0,
	MVPP2_C3_QOS_DUMP_QUEUE_HIGH
};

enum pp2_cls_c3_mod_dump_idx_t {
	MVPP2_C3_MOD_DUMP_DPTR = 0,
	MVPP2_C3_MOD_DUMP_IPTR,
	MVPP2_C3_MOD_DUMP_L4_CHECKSUM
};

enum pp2_cls_c3_flow_dump_idx_t {
	MVPP2_C3_FLOW_DUMP_ID = 0,
	MVPP2_C3_FLOW_DUMP_CNT
};

enum pp2_cls_c3_index_dump_idx_t {
	MVPP2_C3_INDEX_DUMP_HASH = 0,
	MVPP2_C3_INDEX_DUMP_LOGICAL
};

enum pp2_cls_c3_table_dump_mode_t {
	MVPP2_C3_TABLE_DUMP_VALID = 0,	/* only dump valid entries	*/
	MVPP2_C3_TABLE_DUMP_ALL		/* dump all entries		*/
};

enum pp2_cls_c3_entry_dump_mode_t {
	MVPP2_C3_ENTRY_DUMP_LOGIC_IDX = 0,
	MVPP2_C3_ENTRY_DUMP_HASH_IDX,
	MVPP2_C3_ENTRY_DUMP_LU_TYPE,
	MVPP2_C3_ENTRY_DUMP_ALL
};

#define MVPP2_MATCH_IPV4_5T     (MVPP2_MATCH_IP_SRC | MVPP2_MATCH_IP_DST | MVPP2_MATCH_L4_DST \
			      | MVPP2_MATCH_L4_SRC | MVPP2_MATCH_IP_PROTO | MVPP2_MATCH_IPV4_PKT)

#define MVPP2_MATCH_IPV6_5T     (MVPP2_MATCH_IP_SRC | MVPP2_MATCH_IP_DST | MVPP2_MATCH_L4_DST \
			      | MVPP2_MATCH_L4_SRC | MVPP2_MATCH_IP_PROTO | MVPP2_MATCH_IPV6_PKT)

/********************************************************************************/
/*			STRUCTURES						*/
/********************************************************************************/
struct pp2_cls_c3_add_entry_t {
	struct mv_pp2x_src_port			port;		/* port information	*/
	u8					lkp_type;	/* lookup type*/
	struct pp2_cls_mng_pkt_key_t		*mng_pkt_key;	/* pkt key value*/
	struct mv_pp2x_engine_qos_info		qos_info;	/* all the qos input	*/
	struct mv_pp2x_engine_pkt_action	action;		/* update&lock info*/
	struct mv_pp2x_qos_value		qos_value;	/* pri/dscp/gemport/qLow/qHigh*/
	struct pp2_cls_engine_pkt_mod_t		pkt_mod;	/* PMT cmd_idx and data_idx*/
	struct pp2_cls_duplicate_t		flow_info;	/* pkt duplication flow info*/
	u8					rss_en;		/* lookup type*/
};

struct pp2_cls_c3_hash_index_entry_t {
	u16	valid;		/* indicate whether this logical index is valid*/
	u16	hash_idx;	/* multihash index*/
};

struct pp2_cls_c3_logic_index_entry_t {
	u16	valid;		/* indicate whether this hash index is valid*/
	u16	logic_idx;	/* logical index*/
};

struct pp2_cls_c3_scan_config_t {
	u8	clear_before_scan;	/* clear counter before scan	*/
	u8	lkp_type_scan;		/* scan by lookup type*/
	u8	lkp_type;		/* lookup type*/
	u8	scan_mode;		/* scan mode*/
	u32	start_entry;		/* scan startted entry*/
	u32	scan_delay;		/* scan delay time*/
	u32	scan_threshold;		/* scan threshold*/
};

/* below data structure are used by C3 debug */
struct pp2_cls_c3_data_t {
	struct mv_pp2x_src_port			port;		/* port information	*/
	u8					lkp_type;	/* lookup type*/
	enum pp2_cls_l4_type_t			l4_type;	/* L4 type, 0:tcp, 1:udp*/
	u8					hek_len;	/* the length of HEK*/
	u8					hek[MVPP2_C3_MAX_HASH_KEY_SIZE];/* HEK value*/
	struct mv_pp2x_engine_pkt_action	action;		/* update&lock info*/
	struct mv_pp2x_qos_value		qos_value;	/* qLow/qHigh*/
	u16					policer_id;	/* policer id, 0xffff*/
	struct pp2_cls_engine_pkt_mod_t		pkt_mod;	/* PMT cmd idx and data idx	*/
	struct pp2_cls_duplicate_t		dup_info;	/* pkt duplication flow info*/
};

struct pp2_cls_c3_scan_entry_t {
	u32	hash_idx;	/* multihash index*/
	u32	logic_idx;	/* logical index*/
	u32	hit_cnt;	/* hit counter*/
};

/******************************************************************************/
/*                                PROTOTYPE                                   */
/******************************************************************************/

/* Internal  */
int pp2_cls_c3_rule_convert(struct pp2_cls_c3_add_entry_t *mng_entry, struct pp2_cls_c3_entry *hw_entry);
int pp2_cls_c3_default_rule_convert(struct pp2_cls_c3_add_entry_t *mng_entry, struct pp2_cls_c3_entry *hw_entry);
int pp2_cls_c3_rule_check(struct pp2_cls_c3_add_entry_t *c3_entry);
int pp2_cls_c3_default_rule_check(struct pp2_cls_c3_add_entry_t *c3_entry);
/* Internal debug */
void print_horizontal_line(u32 char_count, const char *char_val);
void pp2_cls_c3_entry_header_dump(void);
void pp2_cls_c3_entry_convert(struct pp2_cls_c3_entry *pp2_entry, struct pp2_cls_c3_data_t *mng_entry);
void pp2_cls_c3_entry_line_dump(u32 dump_idx, u32 hash_idx, u32 logic_idx, u32 hit_count,
				struct pp2_cls_c3_data_t *c3_entry);
int pp2_cls_cli_c3_type_entry_dump(void *arg, int argc, char *argv[]);
int pp2_cls_cli_c3_index_entry_dump(void *arg, int argc, char *argv[]);
int pp2_cls_cli_c3_scan_param_set(void *arg, int argc, char *argv[]);
int pp2_cls_cli_c3_scan_result_get(void *arg, int argc, char *argv[]);
int pp2_cls_cli_c3_hit_count_get(void *arg, int argc, char *argv[]);
int pp2_cls_cli_c3_search_depth_set(void *arg, int argc, char *argv[]);
int pp2_cls_cli_c3_rule_delete(void *arg, int argc, char *argv[]);
int pp2_cls_cli_c3_rule_add(void *arg, int argc, char *argv[]);

/* External  */
int pp2_cls_c3_rule_add(uintptr_t cpu_slot, struct pp2_cls_c3_add_entry_t *c3_entry, u32 *logic_idx);
int pp2_cls_c3_default_rule_add(uintptr_t cpu_slot, struct pp2_cls_c3_add_entry_t *c3_entry, u32 *logic_idx);
int pp2_cls_c3_rule_del(uintptr_t cpu_slot, u32 logic_idx);
int pp2_cls_c3_rule_get(uintptr_t cpu_slot, struct pp2_cls_c3_add_entry_t *c3_entry, u32 *entry_num,
			u32 *logic_idx_arr[]);
int pp2_cls_c3_hit_count_get(uintptr_t cpu_slot, int logic_idx, u32 *hit_count);
int pp2_cls_c3_hit_cntr_all_get(uintptr_t cpu_slot, int hit_low_thresh, struct pp2_cls_hit_cnt_t cntr_info[],
				u32 *num_of_cntrs);
int pp2_cls_c3_scan_param_set(uintptr_t cpu_slot, struct pp2_cls_c3_scan_config_t *scan_config);
int pp2_cls_c3_scan_result_get(uintptr_t cpu_slot, u32 max_entry_num, u32 *entry_num,
			       struct pp2_cls_c3_scan_entry_t result_entry[]);
int pp2_cls_c3_entry_get(uintptr_t cpu_slot, u32 logic_idx, struct pp2_cls_c3_data_t *c3_entry);
int pp2_cls_c3_reset(uintptr_t cpu_slot);
int pp2_cls_c3_start(uintptr_t cpu_slot);
int pp2_cls_c3_test(uintptr_t cpu_slot, int num);
#endif /*_PP2_CLS_C3_H_*/
