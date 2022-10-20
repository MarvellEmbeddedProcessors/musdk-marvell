/* Copyright (c) 2018 Marvell.
 * SPDX-License-Identifier: GPL-2.0-only
 */

#ifndef _MV_DPAPI_TYPES_H_
#define _MV_DPAPI_TYPES_H_


#include <linux/list.h>
#include "mv_dp_fw_if.h"
#include "mv_dp_defs.h"
#include "mv_nss_dp.h"
#include "mv_dp_includes.h"

#ifdef __cplusplus
extern "C" {
#endif


/* DP Return Status Codes */
enum mv_dp_rc {
	MV_DP_RC_OK = 0, /*same as the DPAPI return codes*/
	MV_DP_RC_ERR_FAILED,
	MV_DP_RC_TOO_MANY_REQUESTS,
	MV_DP_RC_END_OF_LIST,
	MV_DP_RC_ERR_INVALID_PARAM,
	MV_DP_RC_ERR_OUT_OF_RESOURCES,
	MV_DP_RC_ITEM_NOT_FOUND,
	MV_DP_RC_EXEC_TIMEOUT,
	MV_DP_RC_SHUTDOWN,
	MV_DP_RC_DP_NOT_SUPPORTED,
	MV_DP_RC_ERR_ALLOC,
	MV_DP_RC_ERR_MSG_TX,
	MV_DP_RC_ERR_MSG_NOT_FOUND,
	MV_DP_RC_ERR_MSG_SN_NOT_EQ,
	MV_DP_RC_ERR_OFFLINE,
	MV_DP_RC_ERR_ALREADY_INITILIZED,
	MV_DP_RC_ERR_EVENT_PARSE_FAILED,
	MV_DP_RC_ERR_MSG_PARSE_FAILED,
	MV_DP_RC_ERR_MSG_SIZE,
	MV_DP_RC_ERR_CH_CREATE,
	MV_DP_RC_ERR_COUNT,
	MV_DP_RC_ERR_FW_STATUS,
	MV_DP_RC_ERR_TIMER_DELETE,
	MV_DP_RC_ERR_AGED,
	MV_DP_RC_ERR_NO_DP_KO,
	MV_DP_RC_ERR_NULL_PTR,
	MV_DP_RC_ERR_MSGID_ILLEGAL,
	MV_DP_RC_ERR_WRONG_RX_CHAN,
	MV_DP_RC_ERR_WRONG_CHAN,
	MV_DP_RC_ERR_EXT_BUF_POPULATE,
	MV_DP_RC_ERR_TIMER_SET,
	MV_DP_RC_ERR_MSG_RX,
	MV_DP_RC_ERR_ALREADY_ONLINE,
	MV_DP_RC_ERR_ALREADY_OFFLINE,
	MV_DP_RC_ERR_FW_VERSION,
	MV_DP_RC_ERR_EVENT_HNDL_FAILED,
	MV_DP_RC_ERR_MSG_HNDL_FAILED,
	MV_DP_RC_ERR_WRONG_STATE,
	MV_DP_RC_ERR_BUILD_TX,
	MV_DP_RC_LAST
};


/*statistics per API - MSG opcode, stored per opcode*/
struct mv_dp_msg_stat {
	u32 msg_tx;             /*counts for messages transmitted */
	u32 msg_rx;             /*counts for messages received*/
	u32 err_msg_rx;         /*errors while rx*/
	u32 err_msg_tx;         /*errors while tx*/
};

/*a msg struct to be kept in pending rx buffer*/
struct mv_dp_msg_info_tx {

	u16			sn;	/*out parameter*/
	u16			flags;
	u16			count;
	u16			opcode;
	int			cpu_id;	/*out parameter*/
	int			msg_size;
	void			*msg_data;
	void			*out_buf; /*used to store external ptr*/
	const mv_nss_dp_result_spec_t	*res; /*result struct*/
};

struct mv_dp_msg_info_rx {
	u16			sn;
	u16			flags;
	u16			num_ok;
	u16			opcode;
	int			msg_size;
	int			ch;
	int			rc;
	void			*msg_data;
};



/*a msg struct to be kept in pending rx buffer*/
struct mv_dp_msg_buf {

	u8			age; /*used for aging*/
	u16			sn; /*for debug only -- sn is index*/
	u16			opcode;
	int			cpu_id; /*for debug only*/
	void			*out_buf; /*external ptr to copy the result to*/
	u32			*ext_buf;
	/*to be replaced with ext_buffer struct instead of single buffer*/
	mv_nss_dp_result_spec_t res; /*result struct*/
#ifdef MV_DP_USE_LIST_BUFFER
	struct list_head	list_elem;
#endif
};

struct mv_dp_msg_bulk {
	u32			index;
	u32			options;
};



/*DP RX/TX message serializers deserializers*/
typedef enum mv_dp_rc (*mv_dp_msg_rx_func)(struct mv_dp_msg_info_rx *msg, void *param);
typedef enum mv_dp_rc (*mv_dp_msg_rx_func_new)(void *dest, void *src);
typedef enum mv_dp_rc (*mv_dp_msg_tx_func)(void *buf, const void  *data, int index);


/*holds opcode - message type and rx parser prt */
struct mv_dp_msg_info {
	u8			trace; /*RX,TX,BUFF*/
	u8			type; /*internal, Event etc*/
	u8			return_evt;		/*return type to be specified in cb*/
	u8			tx_flags;
	s16			tx_size; /*expected rx msg size in bytes;*/
	/*expected rx msg size in bytes in bulks -- the size of a single entity, -1 for don't check*/
	s16			rx_size;
	u16			rx_cb_size; /*size of the return struct if any*/
	mv_dp_msg_rx_func_new	rx_handle; /*rx handler: new version deserializer*/
	mv_dp_msg_tx_func	tx_handle; /*tx handler: serializer*/
	const char		*name;
};

struct mv_dp_event_info {
	mv_nss_dp_evt_handler evt; /*event callback*/
};

struct mv_dp_error_info {
	u32		sevirety;
	const char	*name;
};


/*pending rx buffer is a slining window indexed by sn + head_sn*/
struct mv_dp_cbuf {
	u16 head_sn;
	int head;
	int tail;
	int size;
	struct mv_dp_msg_buf buff[MV_DP_RX_BUFF_SIZE];
};

struct mv_dp_timer {
	struct timer_list		timer;
	unsigned			counter;
	unsigned			interval;
};


/* DPAPI per CPU struct
*
*
*/
struct mv_dp_ch_data {
	/*channelid to cpu ?*/
	u16			sn; /*SN is per channel*/
	int			ch_id; /*is array index*/
	int			cpu_id; /*debug - redundant*/
	spinlock_t		lock;
	struct mv_dp_timer	ch_timer;

#ifdef MV_DP_USE_LIST_BUFFER
	struct mv_dp_msg_buf	rx_buf;
	u16			rx_size;
	u16			max_rx_size;
#else
	struct mv_dp_cbuf	rx_buf; /*sliding window circular buffer should be used*/
#endif
	struct mv_dp_msg_stat	api_stats[MV_DP_MSGID_LAST]; /*msg stats per message type*/
};




struct mv_dp_main {

	u8				ch_mode;
	u8				allowed_mask; /*mask of message types allowed to rx/tx*/
	u8				status; /*initialized, initializing, online*/
	u8				dbg_lvl; /*debug level*/
	int				cpu_to_ch_id[CONFIG_NR_CPUS]; /*convert cpu_id to channels*/
	struct mv_dp_ch_data		*ch_data[MV_DP_MAX_CHAN_NUM];
	struct mv_dp_msg_info		msg_info[MV_DP_MSGID_LAST]; /*message info and rx handler*/
	struct mv_dp_event_info		event_info[MV_DP_EVENTID_LAST]; /*message info and rx handler*/
	struct mv_dp_error_info		error_info[MV_DP_RC_LAST];
	u32				errors[MV_DP_RC_LAST];          /*error counters*/
	struct net_device *netdev; /* agnic net device*/
};



#ifdef __cplusplus
}
#endif


#endif
