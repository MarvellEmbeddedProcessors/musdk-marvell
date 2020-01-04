/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

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

/** @} */ /* end of grp_nmp_guest_giu */

#endif /* _MV_NMP_GUEST_GIU_H */

