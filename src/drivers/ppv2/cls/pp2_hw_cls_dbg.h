/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file pp2_hw_cls_debug.h
 *
 * PPDK Quality of Service(QoS) assured by
 *      Parser, Classifier and Policer
 *
 */

#ifndef _PP2_HW_CLS_DEBUG_H_
#define _PP2_HW_CLS_DEBUG_H_

#include "../pp2_types.h"
#include "../pp2_hw_type.h"

#define HEK_EXT_FMT				"%8.8x %8.8x %8.8x | %8.8x %8.8x %8.8x %8.8x %8.8x %8.8x"
#define HEK_EXT_VAL(p)				p[8], p[7], p[6], p[5], p[4], p[3], p[2], p[1], p[0]

#define HEK_FMT					"%8.8x %8.8x %8.8x"
#define HEK_VAL(p)				p[8], p[7], p[6]

/*-------------------------------------------------------------------------------*/
/*	DUMP APIs for Classification C3 engine					  */
/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_hw_dump(uintptr_t cpu_slot);
int pp2_cls_c3_hw_miss_dump(uintptr_t cpu_slot);
int pp2_cls_c3_hw_ext_dump(uintptr_t cpu_slot);
int pp2_cls_c3_sw_dump(struct pp2_cls_c3_entry *c3);
int pp2_cls_c3_scan_regs_dump(uintptr_t cpu_slot);
int pp2_cls_c3_scan_res_dump(uintptr_t cpu_slot);

#endif /* _PP2_HW_CLS_DEBUG_H_ */
