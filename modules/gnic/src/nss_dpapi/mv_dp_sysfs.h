/* Copyright (c) 2018 Marvell.
 * SPDX-License-Identifier: GPL-2.0-only
 */

#ifndef _MV_DP_SYSFS_H_
#define _MV_DP_SYSFS_H_

#ifdef __cplusplus
extern "C" {
#endif
#include <linux/completion.h>
#include "mv_nss_dp.h"


#define MV_DP_CLI_TMSG(fmt, ...)		(pr_info("DP SENT>" fmt, ##__VA_ARGS__))
#define MV_DP_CLI_OK(fmt, ...)			(pr_info("DP OK>" fmt, ##__VA_ARGS__))
#define MV_DP_CLI_OK_CB(fmt, evt, ...)		\
			(pr_info("DP OK>S:%d:X:0x%X:K:%p:T:%d:C:%d>" fmt, ((evt)->status), ((evt)->xid), \
			((evt)->cookie), ((evt)->type), ((evt)->count), ##__VA_ARGS__))
#define MV_DP_CLI_FAIL(fmt, s, ...)		(pr_info("DP FAIL>S:%d>" fmt, (s), ##__VA_ARGS__))
#define MV_DP_CLI_FAIL_CB(fmt, evt, ...)		\
			(pr_info("DP FAIL>S:%d:X:0x%X:K:%p:T:%d:C:%d>" fmt, ((evt)->status), ((evt)->xid), \
			((evt)->cookie), ((evt)->type), ((evt)->count), ##__VA_ARGS__))
#define MV_DP_CLI_CONT(fmt, ...)		(pr_cont(fmt, ##__VA_ARGS__))



enum mv_dp_sys_dir_type {
	QOS_DIR_TYPE_INGRESS,
	QOS_DIR_TYPE_EGRESS,
	QOS_DIR_TYPE_LAST
};
#define MV_DP_SYS_QOS_DIR_TYPE_IS_OK(v) ((v) >= QOS_DIR_TYPE_INGRESS && (v) <= QOS_DIR_TYPE_LAST)


int mv_dp_sysfs_init(void);
int mv_dp_sysfs_init_entities(void);
void mv_dp_sysfs_exit(void);
void mv_dp_sysfs_exit_entities(void);
int mv_dp_sysfs_delay_get(void);
void mv_dp_sysfs_delay_set(int to);

void mv_dp_sysfs_ip_to_str(char *dst_str, mv_nss_dp_ip_addr_t  *ip_ptr);

#ifdef __cplusplus
}
#endif


#endif
