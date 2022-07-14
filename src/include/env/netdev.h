/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __NETDEV_H__
#define __NETDEV_H__

#include <sys/socket.h>
#include <linux/if.h>

#define PP2_PORT_IF_NAME_MAX_ITER	256

int mv_netdev_feature_set(const char *netdev, const char *featstr, int val);

int mv_netdev_ioctl(u32 ctl, struct ifreq *s);

#endif /* __NETDEV_H__ */
