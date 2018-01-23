/******************************************************************************
 *	Copyright (C) 2017 Marvell International Ltd.
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
