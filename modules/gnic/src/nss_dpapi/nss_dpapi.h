/* Copyright (c) 2018 Marvell.
 * SPDX-License-Identifier: GPL-2.0-only
 */

#ifndef INCLUDE_NSS_DPAPI_H_
#define INCLUDE_NSS_DPAPI_H_


/*
 * mv_dpapi_init
 *
 * Description:
 *       nss_dpapi initialization routine as part of module init.
 *
 * Parameters:
 *       netdev - pointer to net device
 *
 * Returns:
 *        0 - On success or an error code, otherwise.
 */
int mv_dpapi_init(struct net_device *netdev);

/*
 * mv_dpapi_exit
 *
 * Description:
 *       nss_dpapi exit routine as part or module exit.
 *
 * Parameters:
 *       None
 *
 * Returns:
 *       VOID
 */
void mv_dpapi_exit(void);

#endif /* INCLUDE_NSS_DPAPI_H_ */
