/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#ifndef _MV_NMP_DISPATCH_H
#define _MV_NMP_DISPATCH_H

/** @addtogroup grp_nmp_dispatch NMP Dispatch
 *
 *  Networking Management Proxy (NMP) Dispatch API documentation
 *
 *  @{
 */

#define MV_NMP_Q_PAIR_MAX (1)	/**< Max Queue Pair */

struct nmdisp;

struct nmdisp_params {
	/* TODO: */
};

/**
 *  struct nmdisp_q_pair_params - dispatcher client queue
 *  parameters
 */
struct nmdisp_q_pair_params {
	int ext_desc_support;	/**< if TRUE, dispatch can use the 'num_ext_desc' field in case of large message */
	u32 max_msg_size;	/**< maximum message size, will be used to allocate buffer to hold it */
	struct mqa_q *cmd_q;	/**< Command Queue */
	struct mqa_q *notify_q;	/**< Notification Queue */
};

struct nmdisp_msg {
	int	ext;
	int	resp_required;
	u8	src_client;
	u8	src_id;
	u8	dst_client; /**< relevant only for 'send_msg' routine */
	u8	dst_id; /**< relevant only for 'send_msg' routine */
	u8	code;
	u16	indx;
	u16	msg_len;
	void	*msg;
};

/**
 *  struct nmdisp_client_params - dispatcher client
 *  parameters
 */
struct nmdisp_client_params {
	u8 client_type;
	u8 client_id;
	void *client;
	int (*f_client_ctrl_cb)(void *client, struct nmdisp_msg *msg);
};

/**
 *  Dispatcher initialization API
 */
int nmdisp_init(struct nmdisp_params *params, struct nmdisp **nmdisp);

/**
 *  Dispatcher release (deinit) API
 */
void nmdisp_deinit(struct nmdisp *nmdisp);

/**
 *  Dispatcher configuration API - Register a new Client
 */
int nmdisp_register_client(struct nmdisp *nmdisp, struct nmdisp_client_params *params);

/**
 *  Dispatcher configuration API - De-Register a Client
 */
int nmdisp_deregister_client(struct nmdisp *nmdisp, u8 client, u8 id);

/**
 *  Dispatcher configuration API - Add Queue to a Client
 */
int nmdisp_add_queue(struct nmdisp *nmdisp, u8 client, u8 id, struct nmdisp_q_pair_params *q_params);

/**
 *  Dispatcher execution API - Run Dispatcher
 */
int nmdisp_dispatch(struct nmdisp *nmdisp);

/**
 *  Dispatcher execution API - Send Message
 */
int nmdisp_send_msg(struct nmdisp *nmdisp, u8 qid, struct nmdisp_msg *msg);



/** @} */ /* end of grp_nmp_dispatch */

#endif /* _MV_NMP_DISPATCH_H */
