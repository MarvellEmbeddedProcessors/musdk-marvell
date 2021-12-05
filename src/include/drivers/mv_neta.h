/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __MV_NETA_H__
#define __MV_NETA_H__

#include "mv_std.h"


#define NETA_NUM_ETH_PPIO	4 /**< Maximum number of io instances in each packet processor */

/** @addtogroup grp_neta_init Packet Processor: Initialization
 *
 *  Packet Processor Initialization API documentation
 *
 *  @{
 */


/**
 * Initialize the global NETA
 *
 * @param[in]	params	A pointer to structure that contains all relevant parameters.
 *
 * @retval	A pointer to a DMA memory on success
 * @retval	<0 on failure
 */
int neta_init(void);

/**
 * Destroy the global NETA
 *
 */
void neta_deinit(void);

int neta_netdev_get_port_info(char *ifname, u8 *port_id);

/** @} */ /* end of grp_neta_init */

#endif /* __MV_NETA_H__ */
