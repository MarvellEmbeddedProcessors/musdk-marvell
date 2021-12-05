/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file pp2_cls_utils.h
 *
 *  api protypes and macro of pp2_cls_utils.c
 */

/***********************/
/* h file declarations */
/***********************/
#ifndef _PP2_CLS_UTILS_H_
#define _PP2_CLS_UTILS_H_

/********************************************************************************/
/*			MACROS							*/
/********************************************************************************/
#define MVPP2_MEMBER_NUM(array)	ARRAY_SIZE(array)

/********************************************************************************/
/*			STRUCTURES						*/
/********************************************************************************/

struct pp2_cls_enum_str_t {
	int enum_value;	/* the value of enum */
	const char *enum_str;	/* the string name of enum	*/
};

struct pp2_cls_enum_array_t {
	int enum_num;				/* total enum number	*/
	struct pp2_cls_enum_str_t *enum_array;	/* enum array		*/
};

/********************************************************************************/
/*			PROTOTYPE						*/
/********************************************************************************/
u32 common_mask_gen(int bit_num);
const char *pp2_utils_eng_no_str_get(int value);
const char *pp2_utils_field_id_str_get(int value);
const char *pp2_utils_valid_str_get(int value);
const char *pp2_cls_utils_field_match_str_get(int value);
const char *pp2_cls_utils_field_id_str_get(int value);
const char *pp2_cls_utils_port_type_str_get(int value);
const char *pp2_cls_utils_l4_type_str_get(int value);
const char *pp2_cls_utils_qos_action_str_get(int value);
const char *pp2_cls_utils_common_action_str_get(int value);
const char *pp2_cls_utils_flow_id_action_str_get(int value);
const char *pp2_cls_utils_frwd_action_str_get(int value);
const char *pp2_cls_utils_scan_mode_str_get(int value);
const char *pp2_cls_utils_tbl_action_type_str_get(int value);
const char *pp2_g_enum_prs_proto_num_str_get(int value);
const char *pp2_g_enum_prs_net_proto_str_get(int value);
const char *pp2_g_enum_prs_net_proto_field_str_get(int value);
const char *pp2_g_enum_prs_lookup_str_get(int value);
const char *pp2_g_enum_prs_log_port_str_get(int value);
const char *pp2_cls_utils_plcr_state_str_get(int value);
const char *pp2_cls_utils_plcr_token_type_str_get(int value);
const char *pp2_cls_utils_plcr_color_mode_str_get(int value);
void print_horizontal_line(u32 char_count, const char *char_val);
int mv_pp2x_ptr_validate(const void *ptr);
int mv_pp2x_range_validate(int value, int min, int max);
int mv_pp2x_parse_mac_address(char *buf, u8 *macaddr_parts);

#endif /* _PP2_CLS_UTILS_H_ */
