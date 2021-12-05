/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _MV_NMP_GUEST_GIU_H
#define _MV_NMP_GUEST_GIU_H

/** @addtogroup grp_nmp_guest_giu NMP Guest: GIU
 *
 *  Networking Management Proxy (NMP) Guest GIU API documentation
 *
 *  @{
 */


int nmp_guest_giu_gpio_enable(char *gpio_match);

int nmp_guest_giu_gpio_disable(char *gpio_match);

int nmp_guest_giu_gpio_get_link_state(char *gpio_match, int *en);

int nmp_guest_giu_gpio_reset(char *gpio_match);

/** @} */ /* end of grp_nmp_guest_giu */

#endif /* _MV_NMP_GUEST_GIU_H */

