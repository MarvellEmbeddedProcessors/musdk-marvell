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
#include "include/gie.h"

#define GIU_LCL_Q (0)
#define GIU_REM_Q (1)

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

	u64				 msi_regs_pa;	/**< MSI phys-address registers base */
	u64				 msi_regs_va;	/**< MSI virt-address registers base */
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

struct gie *giu_get_gie_handle(struct giu *giu, enum giu_eng eng);

int giu_get_msi_regs(struct giu *giu, u64 *va, u64 *pa);

int giu_bpool_get_mqa_q_id(struct giu_bpool *bpool);

#endif /* __GIU_INTERNAL_H__ */
