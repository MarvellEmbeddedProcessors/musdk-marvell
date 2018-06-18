/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#ifndef __GIU_INTERNAL_H__
#define __GIU_INTERNAL_H__


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

#define GIU_LCL_Q_IND (0)
#define GIU_REM_Q_IND (1)

/**
 *
 * @retval	0 on success
 * @retval	<0 on failure
 */
int giu_free_tc_queues(struct mqa *mqa, union giu_gpio_q_params *giu_gpio_q,
						u32 queue_num, u32 queue_type, void *gie);

/**
 *
 * @retval	0 on success
 * @retval	<0 on failure
 */
int giu_queue_remove(struct mqa *mqa, struct mqa_q *q,
					 enum queue_type queue_type, void *giu_handle);

#endif /* __GIU_INTERNAL_H__ */
