/*******************************************************************************
 * Copyright (C) Marvell International Ltd. and its affiliates
 *
 * This software file (the "File") is owned and distributed by Marvell
 * International Ltd. and/or its affiliates ("Marvell") under the following
 * alternative licensing terms.  Once you have made an election to distribute the
 * File under one of the following license alternatives, please (i) delete this
 * introductory statement regarding license alternatives, (ii) delete the three
 * license alternatives that you have not elected to use and (iii) preserve the
 * Marvell copyright notice above.
 *
 ********************************************************************************
 * Marvell Commercial License Option
 *
 * If you received this File from Marvell and you have entered into a commercial
 * license agreement (a "Commercial License") with Marvell, the File is licensed
 * to you under the terms of the applicable Commercial License.
 *
 ********************************************************************************
 * Marvell GPL License Option
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File in accordance with the terms and conditions of the General
 * Public License Version 2, June 1991 (the "GPL License"), a copy of which is
 * available along with the File in the license.txt file or by writing to the Free
 * Software Foundation, Inc., or on the worldwide web at http://www.gnu.org/licenses/gpl.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY
 * DISCLAIMED.  The GPL License provides additional details about this warranty
 * disclaimer.
 *
 ********************************************************************************
 * Marvell GNU General Public License FreeRTOS Exception
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File in accordance with the terms and conditions of the Lesser
 * General Public License Version 2.1 plus the following FreeRTOS exception.
 * An independent module is a module which is not derived from or based on
 * FreeRTOS.
 * Clause 1:
 * Linking FreeRTOS statically or dynamically with other modules is making a
 * combined work based on FreeRTOS. Thus, the terms and conditions of the GNU
 * General Public License cover the whole combination.
 * As a special exception, the copyright holder of FreeRTOS gives you permission
 * to link FreeRTOS with independent modules that communicate with FreeRTOS solely
 * through the FreeRTOS API interface, regardless of the license terms of these
 * independent modules, and to copy and distribute the resulting combined work
 * under terms of your choice, provided that:
 * 1. Every copy of the combined work is accompanied by a written statement that
 * details to the recipient the version of FreeRTOS used and an offer by yourself
 * to provide the FreeRTOS source code (including any modifications you may have
 * made) should the recipient request it.
 * 2. The combined work is not itself an RTOS, scheduler, kernel or related
 * product.
 * 3. The independent modules add significant and primary functionality to
 * FreeRTOS and do not merely extend the existing functionality already present in
 * FreeRTOS.
 * Clause 2:
 * FreeRTOS may not be used for any competitive or comparative purpose, including
 * the publication of any form of run time or compile time metric, without the
 * express permission of Real Time Engineers Ltd. (this is the norm within the
 * industry and is intended to ensure information accuracy).
 *
 ********************************************************************************
 * Marvell BSD License Option
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File under the following licensing terms.
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *	* Redistributions of source code must retain the above copyright notice,
 *	  this list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the distribution.
 *
 *	* Neither the name of Marvell nor the names of its contributors may be
 *	  used to endorse or promote products derived from this software without
 *	  specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************/

#ifndef __MV_NET_H__
#define __MV_NET_H__

/* The two bytes Marvell header ("MH"). Either contains a special value used
 * by Marvell switches when a specific hardware mode is enabled (not
 * supported by this driver), or is filled automatically by zeroes on
 * the RX side. These two bytes are at the front of the Ethernet
 * header, allow to have the IP header aligned on a 4 bytes
 * boundary.
 */
#define MV_MH_SIZE		2

/* Number of octets (8-bit bytes) in an ethernet address */
#define MV_ETH_ALEN		6
#define MV_ETH_HLEN		14
#define MV_ETH_FCS_LEN		4
#define MV_ETH_ETYPE_LEN	2

/* Similar to if_vlan.h */
#define MV_VLAN_TAG_LEN		4
#define MV_VLAN_PRIO_MASK	0xe000 /* Priority Code Point */
#define MV_VLAN_PRIO_SHIFT	13
#define MV_VLAN_CFI_MASK	0x1000 /* Canonical Format Indicator */
#define MV_VLAN_TAG_PRESENT	MV_VLAN_CFI_MASK
#define MV_VLAN_VID_MASK	0x0fff /* VLAN Identifier */
#define MV_VLAN_N_VID		4096

/* Local addition */
#define MV_VLAN_PRIO_NUM	(1 + (MV_VLAN_PRIO_MASK >> MV_VLAN_PRIO_SHIFT))

/* Similar to xt_dscp.h */
#define MV_XT_DSCP_MASK		0xfc	/* 11111100 */
#define MV_XT_DSCP_SHIFT	2
#define MV_XT_DSCP_MAX		0x3f	/* 00111111 */

/* Local addition */
#define MV_DSCP_NUM		(1 + MV_XT_DSCP_MAX)

#define MV_DEFAULT_MTU		(1500)

/* Max-Transmit-unit (L3) to Max-Receive-Unit */
#define MV_MTU_TO_MRU(mtu) \
	((mtu) + MV_MH_SIZE + MV_VLAN_TAG_LEN + \
	MV_ETH_HLEN + MV_ETH_FCS_LEN)

/* Max-Receive-Unit to Max-Transmit-unit (L3) */
#define MV_MRU_TO_MTU(mru) \
	((mru) - MV_MH_SIZE - MV_VLAN_TAG_LEN - \
	MV_ETH_HLEN - MV_ETH_FCS_LEN)

/* Max-Transmit-unit (L3) to Max-Frame-Length (L2 inc) */
#define MV_MTU_TO_MFL(mtu) \
	((mtu) + MV_VLAN_TAG_LEN + MV_ETH_HLEN)

/* Max-Frame-Length (L2 inc) to Max-Transmit-unit (L3) */
#define MV_MFL_TO_MTU(mfl) \
	((mfl) - MV_VLAN_TAG_LEN - MV_ETH_HLEN)

enum mv_net_eth_dsa_tag_mode_values {
	MV_NET_TO_CPU_DSA_TAG_MODE =		0,
	MV_NET_FROM_CPU_DSA_TAG_MODE =		1,
	MV_NET_TO_SNIFFER_DSA_TAG_MODE =	2,
	MV_NET_FORWARD_DSA_TAG_MODE =		3,
};

enum mv_net_eth_fields {
	MV_NET_ETH_F_SA = 0,
	MV_NET_ETH_F_DA,
	MV_NET_ETH_F_TYPE
};

enum mv_net_eth_dsa_fields {
	MV_NET_ETH_F_DSA_TAG_MODE = 0,
};

enum mv_net_vlan_fields {
	MV_NET_VLAN_F_PRI = 0,
	MV_NET_VLAN_F_ID,
	MV_NET_VLAN_F_TCI	/* not supported */
};

enum mv_net_ipv4_fields {
	MV_NET_IP4_F_DSCP = 0,
	MV_NET_IP4_F_SA,
	MV_NET_IP4_F_DA,
	MV_NET_IP4_F_PROTO,
};

enum mv_net_ipv6_fields {
	MV_NET_IP6_F_TC = 0,	/* not supported */
	MV_NET_IP6_F_SA,
	MV_NET_IP6_F_DA,
	MV_NET_IP6_F_FLOW,
	MV_NET_IP6_F_NEXT_HDR
};

enum mv_net_l4_fields {
	MV_NET_L4_F_SP = 0,
	MV_NET_L4_F_DP,
};

enum mv_net_udp_fields {
	MV_NET_UDP_F_SP = 0,
	MV_NET_UDP_F_DP,
};

enum mv_net_tcp_fields {
	MV_NET_TCP_F_SP = 0,
	MV_NET_TCP_F_DP,
};

struct mv_net_udf {
	u8	id;	/**< udf-id [0 - PP2_MAX_UDFS_SUPPORTED] from pp2_parse_udf_params udf struct */
	u8	size;	/**< udf field size 1-4 byte */
};

enum mv_net_proto {
	MV_NET_PROTO_NONE	= 0,
	MV_NET_PROTO_ETH,
	MV_NET_PROTO_ETH_DSA,
	MV_NET_PROTO_VLAN,
	MV_NET_PROTO_PPPOE,
	MV_NET_PROTO_IP,
	MV_NET_PROTO_IP4,
	MV_NET_PROTO_IP6,
	MV_NET_PROTO_L4,
	MV_NET_PROTO_TCP,
	MV_NET_PROTO_UDP,
	MV_NET_PROTO_ICMP,
	MV_NET_PROTO_ARP,
	MV_NET_UDF,
	MV_NET_PROTO_LAST
};

union mv_net_proto_fields {
	enum mv_net_eth_fields		eth;
	enum mv_net_eth_dsa_fields	eth_dsa;
	enum mv_net_vlan_fields		vlan;
	enum mv_net_ipv4_fields		ipv4;
	enum mv_net_ipv6_fields		ipv6;
	enum mv_net_l4_fields		l4;
	enum mv_net_udp_fields		udp;
	enum mv_net_tcp_fields		tcp;
	struct mv_net_udf		udf;
};

typedef u8 eth_addr_t[MV_ETH_ALEN];

enum mv_net_link_speed {
	MV_NET_LINK_SPEED_AN = 0,
	MV_NET_LINK_SPEED_10,
	MV_NET_LINK_SPEED_100,
	MV_NET_LINK_SPEED_1000,
	MV_NET_LINK_SPEED_2500,
	MV_NET_LINK_SPEED_10000,
};

enum mv_net_link_duplex {
	MV_NET_LINK_DUPLEX_AN = 0,
	MV_NET_LINK_DUPLEX_HALF,
	MV_NET_LINK_DUPLEX_FULL
};

enum mv_net_phy_mode {
	MV_NET_PHY_MODE_NONE = 0,
	MV_NET_PHY_MODE_MII,
	MV_NET_PHY_MODE_GMII,
	MV_NET_PHY_MODE_SGMII,
	MV_NET_PHY_MODE_TBI,
	MV_NET_PHY_MODE_REVMII,
	MV_NET_PHY_MODE_RMII,
	MV_NET_PHY_MODE_RGMII,
	MV_NET_PHY_MODE_RGMII_ID,
	MV_NET_PHY_MODE_RGMII_RXID,
	MV_NET_PHY_MODE_RGMII_TXID,
	MV_NET_PHY_MODE_RTBI,
	MV_NET_PHY_MODE_SMII,
	MV_NET_PHY_MODE_XGMII,
	MV_NET_PHY_MODE_MOCA,
	MV_NET_PHY_MODE_QSGMII,
	MV_NET_PHY_MODE_XAUI,
	MV_NET_PHY_MODE_RXAUI,
	MV_NET_PHY_MODE_KR,
	MV_NET_PHY_MODE_OUT_OF_RANGE,
};

#endif /* __MV_NET_H__ */
