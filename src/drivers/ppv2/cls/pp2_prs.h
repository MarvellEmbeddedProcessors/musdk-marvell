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
 * @file pp2_cls_prs.h
 *
 * internal and external definitions for parser High level routines
 */

#ifndef _PP2_CLS_PRS_H_
#define _PP2_CLS_PRS_H_

#include "pp2_cls_types.h"
#include "pp2_cls_internal_types.h"
#include "pp2_cls_common.h"
#include "pp2_cls_utils.h"
#include "pp2_flow_rules.h"
#include "pp2_c3.h"
#include "pp2_c2.h"
#include "pp2_cls_db.h"

/********************************************************************************/
/*			MACROS							*/
/********************************************************************************/
/* HW_BYTE_OFFS
 * return HW byte offset in 4 bytes register
 * _offs_: native offset (LE)
 * LE example: HW_BYTE_OFFS(1) = 1
 * BE example: HW_BYTE_OFFS(1) = 2
 */

#if defined(__LITTLE_ENDIAN)
#define HW_BYTE_OFFS(_offs_) (_offs_)
#else
#define HW_BYTE_OFFS(_offs_) ((3 - ((_offs_) % 4)) + (((_offs_) / 4) * 4))
#endif

#define SRAM_BIT_TO_BYTE(_bit_) HW_BYTE_OFFS((_bit_) / 8)

#define TCAM_DATA_BYTE_OFFS_LE(_offs_)		(((_offs_) - \
	((_offs_) % 2)) * 2 + ((_offs_) % 2))
#define TCAM_DATA_MASK_OFFS_LE(_offs_) (((_offs_) * 2) - ((_offs_) % 2)  + 2)

/* TCAM_DATA_BYTE/MASK
 * tcam data divide into 4 bytes registers
 * each register include 2 bytes of data and 2 bytes of mask
 * the next macros calc data/mask offset in 4 bytes register
 * _offs_: native offset (LE) in data bytes array
 * relevant only for TCAM data bytes
 * used by PRS and CLS2
 */
#define TCAM_DATA_BYTE(_offs_) (HW_BYTE_OFFS(TCAM_DATA_BYTE_OFFS_LE(_offs_)))
#define TCAM_DATA_MASK(_offs_) (HW_BYTE_OFFS(TCAM_DATA_MASK_OFFS_LE(_offs_)))

/********************************************************************************/
/*			ENUMERATIONS						*/
/********************************************************************************/

/********************************************************************************/
/*			STRUCTURES						*/
/********************************************************************************/

/********************************************************************************/
/*			PROTOTYPE						*/
/********************************************************************************/
int pp2_cls_prs_init(struct pp2_inst *inst);
int mv_pp2x_prs_flow_id_attr_get(int flow_id);

#endif /*_PP2_CLS_PRS_H_*/

