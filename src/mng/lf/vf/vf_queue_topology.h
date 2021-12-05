/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PF_QUEUE_TOPOLOGY_H
#define _PF_QUEUE_TOPOLOGY_H

#include "std_internal.h"
#include "drivers/mv_mqa_queue.h"

#define QUEUE_FREE_STATUS (-1)

/*	Management Channels information
 *
 *	cmd_queue_id	 - command queue Id
 *	notify_queue_id - notification queue Id
 */
struct mng_ch_params {
	struct mqa_q *cmd_queue;
	struct mqa_q *notify_queue;

};

/* Mng Queue topology */
struct giu_mng_topology {
	struct mng_ch_params lcl_mng_ctrl;
	struct mng_ch_params host_mng_ctrl;

};

#endif /* _PF_QUEUE_TOPOLOGY_H */

