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
 * @file pp2_hw_cls_debug.h
 *
 * PPDK Quality of Service(QoS) assured by
 *      Parser, Classifier and Policer
 *
 */

#ifndef _PP2_HW_CLS_DEBUG_H_
#define _PP2_HW_CLS_DEBUG_H_

#include "pp2_types.h"
#include "pp2_hw_type.h"

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
