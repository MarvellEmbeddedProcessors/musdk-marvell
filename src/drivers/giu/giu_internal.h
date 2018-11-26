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

#define GIU_MAX_NUM_GPIO		3 /**< Maximum number of gpio instances */

/* Queue handling macros. assumes q size is a power of 2 */
#define QUEUE_INDEX_INC(index_val, inc_val, q_size)	\
	(((index_val) + (inc_val)) & ((q_size) - 1))

#define QUEUE_OCCUPANCY(prod, cons, q_size)	\
	(((prod) - (cons) + (q_size)) & ((q_size) - 1))

#define QUEUE_SPACE(prod, cons, q_size)	\
	((q_size) - QUEUE_OCCUPANCY((prod), (cons), (q_size)) - 1)

#define QUEUE_FULL(prod, cons, q_size)	\
	((((prod) + 1) & ((q_size) - 1)) == (cons))

/****************************************************************************
 *	gpio queue structures
 ****************************************************************************/
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

struct msix_table_entry {
	u64 msg_addr;
	u32 msg_data;
	u32 vector_ctrl;
};

/**
 * gpio queue parameters
 *
 */
struct giu_gpio_queue {
	u32			 desc_total; /**< number of descriptors in the ring */
	struct giu_gpio_desc	*desc_ring_base; /**< descriptor ring virtual address */
	u32			 last_cons_val; /**< last consumer index value */

	u32			*prod_addr; /**< producer index virtual address */
	u32			*cons_addr; /**< consumer index virtual address */

	union {
		u32		 buff_len; /**< Buffer length (relevant for BPool only) */
		u32		 payload_offset; /**< Offset of the PL in the buffer (relevant for Data Qs only) */
	};
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
