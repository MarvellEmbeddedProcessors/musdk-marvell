/******************************************************************************
 *	Copyright (C) 2016 Marvell International Ltd.
 *
 *  If you received this File from Marvell, you may opt to use, redistribute
 *  and/or modify this File under the following licensing terms.
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *	* Redistributions of source code must retain the above copyright
 *	  notice, this list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the distribution.
 *
 *	* Neither the name of Marvell nor the names of its contributors may be
 *	  used to endorse or promote products derived from this software
 *	  without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

/**
 * @file pp2_port_ks.c
 *
 * Port I/O routines - kernel space specific
 */

#include "std_internal.h"

#include "pp2_types.h"
#include "pp2.h"
#include "pp2_dm.h"
#include "pp2_port.h"

/* Port Control routines */

/* Set MAC address */
int pp2_port_set_mac_addr(struct pp2_port *port, const uint8_t *addr)
{
	pr_err("[%s] not implemented yet\n", __func__);
	return -EINVAL;
}

/* Get MAC address */
int pp2_port_get_mac_addr(struct pp2_port *port, uint8_t *addr)
{
	pr_err("[%s] not implemented yet\n", __func__);
	return -EINVAL;
}

/* Set Unicast promiscuous */
int pp2_port_set_uc_promisc(struct pp2_port *port, uint32_t en)
{
	pr_err("[%s] not implemented yet\n", __func__);
	return -EINVAL;
}

/* Check if unicast promiscuous */
int pp2_port_get_uc_promisc(struct pp2_port *port, uint32_t *en)
{
	pr_err("[%s] not implemented yet\n", __func__);
	return -EINVAL;
}

/* Set Multicast promiscuous */
int pp2_port_set_mc_promisc(struct pp2_port *port, uint32_t en)
{
	pr_err("[%s] not implemented yet\n", __func__);
	return -EINVAL;
}

/* Check if Multicast promiscuous */
int pp2_port_get_mc_promisc(struct pp2_port *port, uint32_t *en)
{
	pr_err("[%s] not implemented yet\n", __func__);
	return -EINVAL;
}

/* Add MAC address */
int pp2_port_add_mac_addr(struct pp2_port *port, const uint8_t *addr)
{
	pr_err("[%s] not implemented yet\n", __func__);
	return -EINVAL;
}

/* Remove MAC address */
int pp2_port_remove_mac_addr(struct pp2_port *port, const uint8_t *addr)
{
	pr_err("[%s] not implemented yet\n", __func__);
	return -EINVAL;
}

int pp2_port_flush_mac_addrs(struct pp2_port *port, uint32_t uc, uint32_t mc)
{
	pr_err("[%s] not implemented yet\n", __func__);
	return -EINVAL;
}

/* Add vlan */
int pp2_port_add_vlan(struct pp2_port *port, u16 vlan)
{
	pr_err("[%s] not implemented yet\n", __func__);
	return -EINVAL;
}

/* Remove vlan */
int pp2_port_remove_vlan(struct pp2_port *port, u16 vlan)
{
	pr_err("[%s] not implemented yet\n", __func__);
	return -EINVAL;
}

/* Get Link State */
int pp2_port_get_link_state(struct pp2_port *port, int  *en)
{
	pr_err("[%s] not implemented yet\n", __func__);
	return -EINVAL;
}

int pp2_port_initialize_statistics(struct pp2_port *port)
{
	pr_err("[%s] not implemented yet\n", __func__);
	return -EINVAL;
}

int pp2_port_get_statistics(struct pp2_port *port)
{
	pr_err("[%s] not implemented yet\n", __func__);
	return -EINVAL;
}

