/*
** Copyright (c) 2015 Marvell.
**
** This program is free software: you can redistribute it and/or
** modify it under the terms of the GNU General Public License as
** published by the Free Software Foundation, either version 2 of the
** License, or any later version.
**
** This program is distributed in the hope that it will be useful, but
** WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
** General Public License for more details.
*/

#ifndef _MV_AGNIC_CUSTOM_MGMT_H_
#define _MV_AGNIC_CUSTOM_MGMT_H_

#include <linux/types.h>


#define AGNIC_CUSTOM_CODE_NOTIFY_DOWN		(0xFF)

struct agnic_inband_mng_msg_params {
	u8   msg_code;
	void *msg;
	u16  msg_len;
	u32  timeout; /* timeout in msec */
	u64  cookie; /*< user cookie. Use '0' if no response is needed.
		* Value '-1' should not be used as it represents 'notification' message.
		*/
	void *resp_msg;
	u16  resp_msg_len;
};

int agnic_register_custom(struct net_device *netdev, u8 id, void *arg,
		void (*recv_custom_msg_cb)(void *arg, u8 code, u64 cookie, void *msg, u16 len));

int agnic_send_custom_msg(struct net_device *netdev,
		struct agnic_inband_mng_msg_params *msg);

#endif /* _MV_AGNIC_CUSTOM_MGMT_H_ */
