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

#ifndef _MV_NMP_DISPATCH_H
#define _MV_NMP_DISPATCH_H

#define MV_NMP_Q_PAIR_MAX (1)

struct nmdisp;

struct nmdisp_params {
	/* TODO: */
};

/* dispatcher client queue parameters */
struct nmdisp_q_pair_params {
	struct mqa_q cmd_q;
	struct mqa_q notify_q;

};

/* dispatcher client parameters */
struct nmdisp_client_params {
	u8 client_type;
	u8 client_id;
	int (*client_sr_cb)(void *client, u8 code, void *msg);
	void *client;
};

/* dispatcher initialization API's */
int nmdisp_init(struct nmdisp_params *params, struct nmdisp **nmdisp);
void nmdisp_deinit(struct nmdisp *nmdisp);

/* dispatcher configuration API's */
int nmdisp_register_client(struct nmdisp *nmdisp, struct nmdisp_client_params *params);
int nmdisp_deregister_client(struct nmdisp *nmdisp, u8 client, u8 id);
int nmdisp_add_queue(struct nmdisp *nmdisp, u8 client, u8 id, struct nmdisp_q_pair_params *q_params);

/* dispatcher execution API's */
int nmdisp_dispatch(struct nmdisp *nmdisp);
int nmdisp_send(struct nmdisp *nmdisp, u8 client, u8 id, u8 qid, void *msg);

#endif /* _MV_NMP_DISPATCH_H */
