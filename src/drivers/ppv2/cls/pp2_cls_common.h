
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
