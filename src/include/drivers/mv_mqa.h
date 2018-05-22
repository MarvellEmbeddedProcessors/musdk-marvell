/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#ifndef _MV_MQA_H
#define _MV_MQA_H

#include "mv_std.h"

/** @addtogroup grp_mqa_init Marvell Queue Arch (MQA) Init
 *
 *  Marvell Queue Arch (MQA) Initialization API documentation
 *
 *  @{
 */


#define MQA_QUEUE_MAX		(1024)	/**< Max number of queues in MQA */
#define MQA_BM_QUEUE_ARRAY	(3)	/**< Number of BM pools associated with data queue */

struct mqa;

/**
 * struct notif_tbl_params - Notification Table Params
 */

struct notif_tbl_params {
	/** Virtual Addr of Global Queue Producer Idx Notif Table.*/
	void		*qnpt_va;

	/** Physical Addr of Global Queue Producer Idx Notif Table.*/
	phys_addr_t	 qnpt_pa;

	/** Virtual Addr of Global Queue Consumer Idx Notif Table.*/
	void		*qnct_va;

	/** Physical Addr of Global Queue Consumer Idx Notif Table.*/
	phys_addr_t	 qnct_pa;

};

/**
 * struct mqa_params - MQA Params
 */

struct mqa_params {
	/** Match string (reserved/unused) */
	char *match;

	/** Number of Queues */
	u16  num_qs;

	/** Notification Table struct  */
	struct notif_tbl_params notif_tbl;

};

/**
 *	Initialize MQA infrastructure.
 *	MQA tables - GPT, GCT, GNPT, and GNCT are allocated and initialized.
 *	MQA region map is statically initialized and divide MQA tables into dedicated regions.
 *
 *	@param[in]	params	A pointer to MQA parameters
 *	@param[out]	mqa	A pointer to MQA object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int mqa_init(struct mqa_params *params, struct mqa **mqa);

/**
 *	Release MQA tables - GPT, GCT, GNPT, and GNCT.
 *	Clear MQA Region tables.
 *
 *	@param[in]	mqa	A pointer to MQA object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int mqa_deinit(struct mqa *mqa);

/** @} */ /* end of grp_mqa_init */

#endif /* _MV_MQA_H */



