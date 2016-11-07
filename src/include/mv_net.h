/******************************************************************************
	Copyright (C) 2016 Marvell International Ltd.

  If you received this File from Marvell, you may opt to use, redistribute
  and/or modify this File under the following licensing terms.
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  	* Redistributions of source code must retain the above copyright
	  notice, this list of conditions and the following disclaimer.

  	* Redistributions in binary form must reproduce the above copyright
  	  notice, this list of conditions and the following disclaimer in the
  	  documentation and/or other materials provided with the distribution.

  	* Neither the name of Marvell nor the names of its contributors may be
  	  used to endorse or promote products derived from this software
	  without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#ifndef __MV_NET_H__
#define __MV_NET_H__

#define MV_NET_MAX_NUM_FIELDS	32

#define MV_NET_PROT_ETH_SHIFT	1
#define MV_NET_PROT_VLAN_SHIFT	2
#define MV_NET_PROT_PPPoE_SHIFT	3
#define MV_NET_PROT_IPv4_SHIFT	4
#define MV_NET_PROT_IPv6_SHIFT	5
#define MV_NET_PROT_IP_SHIFT	6
#define MV_NET_PROT_L4_SHIFT	7
#define MV_NET_PROT_UDP_SHIFT	8
#define MV_NET_PROT_TCP_SHIFT	9
#define MV_NET_PROT_ICMP_SHIFT	10
#define MV_NET_PROT_ARP_SHIFT	11
#define MV_NET_PROT_LAST_SHIFT	20


enum mv_net_eth_fields {
	MV_NET_ETH_F_SA = 0,
	MV_NET_ETH_F_DA,
	MV_NET_ETH_F_TYPE
};

enum mv_net_vlan_fields {
	MV_NET_VLAN_F_PRI = 0,
	MV_NET_VLAN_F_ID,
	MV_NET_VLAN_F_TCI
};

enum mv_net_ipv4_fields {
	MV_NET_IPv4_F_TOS = 0,
	MV_NET_IPv4_F_SA,
	MV_NET_IPv4_F_DA,
	MV_NET_IPv4_F_PROTO
};

enum mv_net_ipv6_fields {
	MV_NET_IPv6_F_TC = 0,
	MV_NET_IPv6_F_SA,
	MV_NET_IPv6_F_DA,
	MV_NET_IPv6_F_FLOW,
	MV_NET_IPv6_F_NEXT_HDR
};

enum mv_net_l4_fields {
	MV_NET_L4_F_SP = 0,
	MV_NET_L4_F_DP,
	MV_NET_L4_F_CSUM
};

enum mv_net_udp_fields {
	MV_NET_UDP_F_SP = 0,
	MV_NET_UDP_F_DP,
	MV_NET_UDP_F_CSUM
};

enum mv_net_tcp_fields {
	MV_NET_TCP_F_SP = 0,
	MV_NET_TCP_F_DP,
	MV_NET_TCP_F_CSUM
};

/*
enum mv_net_prtcl_fields {
	union {
		enum mv_net_eth_fields	eth;
		enum mv_net_vlan_fields	vlan;
		enum mv_net_ipv4_fields	ipv4;
		enum mv_net_ipv6_fields	ipv6;
		enum mv_net_l4_fields	l4;
		enum mv_net_udp_fields	udp;
		enum mv_net_tcp_fields	tcp;
	} u;
};
*/

enum mv_net_prtcl {
	MV_NET_PRTCL_NONE	= 0,

	MV_NET_PRTCL_ETH	= (MV_NET_PROT_ETH_SHIFT*MV_NET_MAX_NUM_FIELDS),
	MV_NET_PRTCL_VLAN	= (MV_NET_PROT_VLAN_SHIFT*MV_NET_MAX_NUM_FIELDS),
	MV_NET_PRTCL_PPPoE	= (MV_NET_PROT_PPPoE_SHIFT*MV_NET_MAX_NUM_FIELDS),
	MV_NET_PRTCL_IP		= (MV_NET_PROT_IP_SHIFT*MV_NET_MAX_NUM_FIELDS),
	MV_NET_PRTCL_IPv4	= (MV_NET_PROT_IPv4_SHIFT*MV_NET_MAX_NUM_FIELDS),
	MV_NET_PRTCL_IPv6	= (MV_NET_PROT_IPv6_SHIFT*MV_NET_MAX_NUM_FIELDS),
	MV_NET_PRTCL_L4		= (MV_NET_PROT_L4_SHIFT*MV_NET_MAX_NUM_FIELDS),
	MV_NET_PRTCL_TCP	= (MV_NET_PROT_TCP_SHIFT*MV_NET_MAX_NUM_FIELDS),
	MV_NET_PRTCL_UDP	= (MV_NET_PROT_UDP_SHIFT*MV_NET_MAX_NUM_FIELDS),
	MV_NET_PRTCL_ICMP	= (MV_NET_PROT_ICMP_SHIFT*MV_NET_MAX_NUM_FIELDS),
	MV_NET_PRTCL_ARP	= (MV_NET_PROT_ARP_SHIFT*MV_NET_MAX_NUM_FIELDS),

	MV_NET_PRTCL_LAST	= (MV_NET_PROT_LAST_SHIFT*MV_NET_MAX_NUM_FIELDS)
};

typedef u16 mv_net_prtcl_field_t;

/*
#define MV_NET_PRTCL_FIELD(_hdr, _fld)	\
	(mv_net_prtcl_field_t)((_hdr*))
*/

#endif /* __MV_NET_H__ */
