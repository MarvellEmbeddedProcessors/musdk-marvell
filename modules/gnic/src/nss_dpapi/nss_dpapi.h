/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is free software: you can redistribute it and/or
*  modify it under the terms of the GNU General Public License as
*  published by the Free Software Foundation, either version 2 of the
*  License, or any later version.
*
*  This program is distributed in the hope that it will be useful, but
*  WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
*  General Public License for more details.
*
*******************************************************************************/

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
