/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file pp2_cls_common.h
 *
 * Internal definitions for pp2_cls_common.c
 */

/***********************/
/* h file declarations */
/***********************/
#ifndef _PP2_CLS_COMMON_H_
#define _PP2_CLS_COMMON_H_

/********************************************************************************/
/*			MACROS							*/
/********************************************************************************/
#define MVPP2_MEMSET_ZERO(STRUCT) memset(&(STRUCT), 0, sizeof((STRUCT)))
#define MVPP2_MEMSET_FF(STRUCT)	  memset(&(STRUCT), 0xff, sizeof((STRUCT)))

/********************************************************************************/
/*			PROTOTYPE						*/
/********************************************************************************/
int pp2_cls_field_bm_to_field_info(u32 field_bm, struct pp2_cls_mng_pkt_key_t *pp2_cls_pkt_key,
				   u32	field_max, u8 l4_info, struct pp2_cls_field_match_info field_info[]);
u32 pp2_cls_field_size_get(u32 field_id);

#endif /* _PP2_CLS_COMMON_H_ */
