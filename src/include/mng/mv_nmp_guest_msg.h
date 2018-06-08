/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#ifndef _MV_NMP_GUEST_MSG_H
#define _MV_NMP_GUEST_MSG_H

/** @addtogroup grp_nmp_init Networking Mgmt Proxy Init
 *
 *  Networking Management Proxy (NMP) Guest messages API
 *  documentation
 *
 *  @{
 */

#define CMD_IDX_NOTIFICATION	0xFFFF

 /*
 * msg_to_guest_codes - Define the list of messages codes that can be send to guest.
 */
enum msg_to_guest_codes {
	MSG_T_GUEST_NONE = 0,
	MSG_T_GUEST_LINK_CHANGED,
	MSG_T_GUEST_MAC_ADDR_UPDATED,
	MSG_T_GUEST_MTU_UPDATED,
	MSG_T_GUEST_LAST,
};

#endif /* _MV_NMP_GUEST_MSG_H */

