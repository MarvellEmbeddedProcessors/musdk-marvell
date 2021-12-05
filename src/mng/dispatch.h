/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _DISPATCH_H
#define _DISPATCH_H

#include "mng/mv_nmp_dispatch.h"

#define NMDISP_MAX_CLIENTS      (10)
#define NMDISP_MAX_CLIENTS_TYPE (5)
#define NMDISP_MAX_CLIENTS_ID   (10)

/* dispatcher client parameters */
struct nmdisp_client {
	u8 client_type;
	u8 client_id;
	void *client;
	int (*client_ctrl_cb)(void *client, struct nmdisp_msg *msg);
	struct nmdisp_q_pair_params client_q[MV_NMP_Q_PAIR_MAX];
};

/* dispatcher handler */
struct nmdisp {
	u8  *cmd_msg;	/**< command message buffer to hold more than one desc */
	u32 max_msg_size; /**< maximum message size, reflect the size of 'cmd_msg' */
	struct nmdisp_client clients[NMDISP_MAX_CLIENTS];

};

void nmdisp_dispatch_dump(struct nmdisp *nmdisp_p);

#endif /* _DISPATCH_H */
