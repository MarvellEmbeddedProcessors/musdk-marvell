/******************************************************************************
 *	Copyright (C) 2018 Marvell International Ltd.
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

/***********************/
/* h file declarations */
/***********************/
#ifndef _PP2_EDROP_H_
#define _PP2_EDROP_H_

#include "std_internal.h"
#include "drivers/mv_pp2_cls.h"

struct pp2_cls_early_drop {
	int	pp2_id;		/* PP2 Instance */
	int	id;		/* early-drop id */
};


/******************************************************************************/
/*                                 MACROS                                     */
/******************************************************************************/
#define MVPP2_EDROP_MAX			MVPP2_PLCR_EDROP_THRESH_NUM
#define MVPP2_EDROP_BYPASS_THRESH_ID	(0)	/* "bypass" early-drop configuration */
/* minimium early-drop ID, start with 1, since 0 is used for "bypass" early-drop configuration */
#define MVPP2_EDROP_MIN_ENTRY_ID	(1)
#define MVPP2_EDROP_MAX_THESH		((1 << MVPP2_PLCR_EDROP_TR_BITS) - 1)	/* max theshold value */

/******************************************************************************/
/*                              ENUMERATIONS                                  */
/******************************************************************************/
enum pp2_cls_edrop_entry_state_t {
	MVPP2_EDROP_ENTRY_INVALID_STATE = 0,	/* invalid early-drop entry	*/
	MVPP2_EDROP_ENTRY_VALID_STATE		/* valid early-drop entry	*/
};

enum pp2_cls_edrop_ref_cnt_action_t {
	MVPP2_EDROP_REF_CNT_INC = 0,	/* increase reference counter by 1	*/
	MVPP2_EDROP_REF_CNT_DEC,	/* decrease reference counter by 1	*/
	MVPP2_EDROP_REF_CNT_CLEAR	/* clear reference counter to be 0	*/
};


/******************************************************************************/
/*                                PROTOTYPE                                   */
/******************************************************************************/
int pp2_cls_edrop_entry_add(struct pp2_inst				*inst,
			    const struct pp2_cls_early_drop_params	*edrop_params,
			    u8						edrop_id);
int pp2_cls_edrop_entry_del(struct pp2_inst *inst, u8 edrop_id);
int pp2_cls_edrop_entry_state_get(struct pp2_inst *inst, u8 edrop_id, enum pp2_cls_edrop_entry_state_t *state);
int pp2_cls_edrop_assign_qid(struct pp2_inst *inst, u8 edrop_id, u8 qid, int assign);
void pp2_cls_edrop_bypass_assign_qid(struct pp2_inst *inst, u8 qid);
int pp2_cls_edrop_start(struct pp2_inst *inst);
void pp2_cls_edrop_finish(struct pp2_inst *inst);
void pp2_cls_edrop_dump(struct pp2_inst *inst);

#endif /* _PP2_EDROP_H_ */
