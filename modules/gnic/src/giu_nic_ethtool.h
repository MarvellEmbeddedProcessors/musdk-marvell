/*
** Copyright (c) 2018 Marvell.
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

#ifndef _ARMADA_GIU_NIC_ETHTOOL_H_
#define _ARMADA_GIU_NIC_ETHTOOL_H_

#include <linux/netdevice.h>


/* Function Prototypes */
int agnic_ethtool_init(struct net_device *netdev);
void agnic_ethtool_release(void);

#endif /* _ARMADA_GIU_NIC_ETHTOOL_H_ */
