/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#ifndef __GIU_INTERNAL_H__
#define __GIU_INTERNAL_H__

#include "std_internal.h"
#include "drivers/mv_mqa.h"
#include "drivers/mv_giu.h"
#include "drivers/mv_giu_gpio.h"
#include "hw_emul/gie.h"

#define GIU_LCL_Q_IND (0)
#define GIU_REM_Q_IND (1)

/**
 * queue type
 *
 */
enum queue_type {
	MNG_CMD_QUEUE,
	MNG_NOTIFY_QUEUE,
	LOCAL_INGRESS_DATA_QUEUE,
	LOCAL_EGRESS_DATA_QUEUE,
	LOCAL_BM_QUEUE,
	HOST_INGRESS_DATA_QUEUE,
	HOST_EGRESS_DATA_QUEUE,
	HOST_BM_QUEUE

};

struct giu {
	enum giu_indices_copy_mode	 indices_mode;

	struct mqa			*mqa;
	struct gie			*gies[GIU_ENG_OUT_OF_RANGE];
};

struct giu_mng_ch {
	struct giu	*giu;

	u32		 lcl_cmd_q_idx;
	struct mqa_q	*lcl_cmd_q;
	u32		 lcl_resp_q_idx;
	struct mqa_q	*lcl_resp_q;
	u32		 rem_cmd_q_idx;
	struct mqa_q	*rem_cmd_q;
	u32		 rem_resp_q_idx;
	struct mqa_q	*rem_resp_q;
};

int giu_destroy_q(struct giu *giu, enum giu_eng eng, struct mqa *mqa,
	struct mqa_q *q, enum queue_type queue_type);

struct gie *giu_get_gie_handle(struct giu *giu, enum giu_eng eng);

#endif /* __GIU_INTERNAL_H__ */
