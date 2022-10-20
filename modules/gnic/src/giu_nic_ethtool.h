/* Copyright (c) 2018 Marvell.
 * SPDX-License-Identifier: GPL-2.0-only
 */

#ifndef _ARMADA_GIU_NIC_ETHTOOL_H_
#define _ARMADA_GIU_NIC_ETHTOOL_H_

#include <linux/netdevice.h>


/* Function Prototypes */
int agnic_ethtool_init(struct net_device *netdev);
void agnic_ethtool_release(void);

#endif /* _ARMADA_GIU_NIC_ETHTOOL_H_ */
