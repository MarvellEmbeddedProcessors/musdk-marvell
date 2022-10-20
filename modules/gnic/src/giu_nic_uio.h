/* Copyright (c) 2018 Marvell.
 * SPDX-License-Identifier: GPL-2.0-only
 */

#ifndef _ARMADA_GIU_NIC_UIO_H_
#define _ARMADA_GIU_NIC_UIO_H_

int agnic_uio_probe(struct device *dev);
int agnic_uio_remove(struct device *dev);

void agnic_uio_notify(struct agnic_adapter *adapter, int tc);

#endif /* _ARMADA_GIU_NIC_UIO_H_ */
