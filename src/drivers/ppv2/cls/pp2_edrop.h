/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

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
