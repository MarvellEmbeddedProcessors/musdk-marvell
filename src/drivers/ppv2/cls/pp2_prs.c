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

#include "std_internal.h"

#include "../pp2_types.h"
#include "../pp2.h"
#include "../pp2_hw_type.h"
#include "pp2_prs.h"

/* Flow ID definetion array */
static struct mv_pp2x_prs_flow_id
	mv_pp2x_prs_flow_id_array[MVPP2_PRS_FL_TCAM_NUM] = {
	/***********#Flow ID#**************#Result Info#************/
	{MVPP2_PRS_FL_IP4_TCP_NF_UNTAG, {MVPP2_PRS_RI_VLAN_NONE |
					 MVPP2_PRS_RI_L3_IP4 |
					 MVPP2_PRS_RI_IP_FRAG_FALSE |
					 MVPP2_PRS_RI_L4_TCP,
					 MVPP2_PRS_RI_VLAN_MASK |
					 MVPP2_PRS_RI_L3_PROTO_MASK |
					 MVPP2_PRS_RI_IP_FRAG_MASK |
					 MVPP2_PRS_RI_L4_PROTO_MASK} },
	{MVPP2_PRS_FL_IP4_TCP_NF_UNTAG, {MVPP2_PRS_RI_VLAN_NONE |
					 MVPP2_PRS_RI_L3_IP4_OPT |
					 MVPP2_PRS_RI_IP_FRAG_FALSE |
					 MVPP2_PRS_RI_L4_TCP,
					 MVPP2_PRS_RI_VLAN_MASK |
					 MVPP2_PRS_RI_L3_PROTO_MASK |
					 MVPP2_PRS_RI_IP_FRAG_MASK |
					 MVPP2_PRS_RI_L4_PROTO_MASK} },
	{MVPP2_PRS_FL_IP4_TCP_NF_UNTAG, {MVPP2_PRS_RI_VLAN_NONE |
					 MVPP2_PRS_RI_L3_IP4_OTHER |
					 MVPP2_PRS_RI_IP_FRAG_FALSE |
					 MVPP2_PRS_RI_L4_TCP,
					 MVPP2_PRS_RI_VLAN_MASK |
					 MVPP2_PRS_RI_L3_PROTO_MASK |
					 MVPP2_PRS_RI_IP_FRAG_MASK |
					 MVPP2_PRS_RI_L4_PROTO_MASK} },

	{MVPP2_PRS_FL_IP4_UDP_NF_UNTAG, {MVPP2_PRS_RI_VLAN_NONE |
					 MVPP2_PRS_RI_L3_IP4 |
					 MVPP2_PRS_RI_IP_FRAG_FALSE |
					 MVPP2_PRS_RI_L4_UDP,
					 MVPP2_PRS_RI_VLAN_MASK |
					 MVPP2_PRS_RI_L3_PROTO_MASK |
					 MVPP2_PRS_RI_IP_FRAG_MASK |
					 MVPP2_PRS_RI_L4_PROTO_MASK} },
	{MVPP2_PRS_FL_IP4_UDP_NF_UNTAG, {MVPP2_PRS_RI_VLAN_NONE |
					 MVPP2_PRS_RI_L3_IP4_OPT |
					 MVPP2_PRS_RI_IP_FRAG_FALSE |
					 MVPP2_PRS_RI_L4_UDP,
					 MVPP2_PRS_RI_VLAN_MASK |
					 MVPP2_PRS_RI_L3_PROTO_MASK |
					 MVPP2_PRS_RI_IP_FRAG_MASK |
					 MVPP2_PRS_RI_L4_PROTO_MASK} },
	{MVPP2_PRS_FL_IP4_UDP_NF_UNTAG, {MVPP2_PRS_RI_VLAN_NONE |
					 MVPP2_PRS_RI_L3_IP4_OTHER |
					 MVPP2_PRS_RI_IP_FRAG_FALSE |
					 MVPP2_PRS_RI_L4_UDP,
					 MVPP2_PRS_RI_VLAN_MASK |
					 MVPP2_PRS_RI_L3_PROTO_MASK |
					 MVPP2_PRS_RI_IP_FRAG_MASK |
					 MVPP2_PRS_RI_L4_PROTO_MASK} },

	{MVPP2_PRS_FL_IP4_TCP_NF_TAG,	{MVPP2_PRS_RI_L3_IP4 |
					 MVPP2_PRS_RI_IP_FRAG_FALSE |
					 MVPP2_PRS_RI_L4_TCP,
					 MVPP2_PRS_RI_L3_PROTO_MASK |
					 MVPP2_PRS_RI_IP_FRAG_MASK |
					 MVPP2_PRS_RI_L4_PROTO_MASK} },
	{MVPP2_PRS_FL_IP4_TCP_NF_TAG,	{MVPP2_PRS_RI_L3_IP4_OPT |
					 MVPP2_PRS_RI_IP_FRAG_FALSE |
					 MVPP2_PRS_RI_L4_TCP,
					 MVPP2_PRS_RI_L3_PROTO_MASK |
					 MVPP2_PRS_RI_IP_FRAG_MASK |
					 MVPP2_PRS_RI_L4_PROTO_MASK} },
	{MVPP2_PRS_FL_IP4_TCP_NF_TAG,	{MVPP2_PRS_RI_L3_IP4_OTHER |
					 MVPP2_PRS_RI_IP_FRAG_FALSE |
					 MVPP2_PRS_RI_L4_TCP,
					 MVPP2_PRS_RI_L3_PROTO_MASK |
					 MVPP2_PRS_RI_IP_FRAG_MASK |
					 MVPP2_PRS_RI_L4_PROTO_MASK} },

	{MVPP2_PRS_FL_IP4_UDP_NF_TAG,	{MVPP2_PRS_RI_L3_IP4 |
					 MVPP2_PRS_RI_IP_FRAG_FALSE |
					 MVPP2_PRS_RI_L4_UDP,
					 MVPP2_PRS_RI_L3_PROTO_MASK |
					 MVPP2_PRS_RI_IP_FRAG_MASK |
					 MVPP2_PRS_RI_L4_PROTO_MASK} },
	{MVPP2_PRS_FL_IP4_UDP_NF_TAG,	{MVPP2_PRS_RI_L3_IP4_OPT |
					 MVPP2_PRS_RI_IP_FRAG_FALSE |
					 MVPP2_PRS_RI_L4_UDP,
					 MVPP2_PRS_RI_L3_PROTO_MASK |
					 MVPP2_PRS_RI_IP_FRAG_MASK |
					 MVPP2_PRS_RI_L4_PROTO_MASK} },
	{MVPP2_PRS_FL_IP4_UDP_NF_TAG,	{MVPP2_PRS_RI_L3_IP4_OTHER |
					 MVPP2_PRS_RI_IP_FRAG_FALSE |
					 MVPP2_PRS_RI_L4_UDP,
					 MVPP2_PRS_RI_L3_PROTO_MASK |
					 MVPP2_PRS_RI_IP_FRAG_MASK |
					 MVPP2_PRS_RI_L4_PROTO_MASK} },

	{MVPP2_PRS_FL_IP6_TCP_NF_UNTAG, {MVPP2_PRS_RI_VLAN_NONE |
					 MVPP2_PRS_RI_L3_IP6 |
					 MVPP2_PRS_RI_IP_FRAG_FALSE |
					 MVPP2_PRS_RI_L4_TCP,
					 MVPP2_PRS_RI_VLAN_MASK |
					 MVPP2_PRS_RI_L3_PROTO_MASK |
					 MVPP2_PRS_RI_IP_FRAG_MASK |
					 MVPP2_PRS_RI_L4_PROTO_MASK} },
	{MVPP2_PRS_FL_IP6_TCP_NF_UNTAG, {MVPP2_PRS_RI_VLAN_NONE |
					 MVPP2_PRS_RI_L3_IP6_EXT |
					 MVPP2_PRS_RI_IP_FRAG_FALSE |
					 MVPP2_PRS_RI_L4_TCP,
					 MVPP2_PRS_RI_VLAN_MASK |
					 MVPP2_PRS_RI_L3_PROTO_MASK |
					 MVPP2_PRS_RI_IP_FRAG_MASK |
					 MVPP2_PRS_RI_L4_PROTO_MASK} },

	{MVPP2_PRS_FL_IP6_UDP_NF_UNTAG, {MVPP2_PRS_RI_VLAN_NONE |
					 MVPP2_PRS_RI_L3_IP6 |
					 MVPP2_PRS_RI_IP_FRAG_FALSE |
					 MVPP2_PRS_RI_L4_UDP,
					 MVPP2_PRS_RI_VLAN_MASK |
					 MVPP2_PRS_RI_L3_PROTO_MASK |
					 MVPP2_PRS_RI_IP_FRAG_MASK |
					 MVPP2_PRS_RI_L4_PROTO_MASK} },
	{MVPP2_PRS_FL_IP6_UDP_NF_UNTAG, {MVPP2_PRS_RI_VLAN_NONE |
					 MVPP2_PRS_RI_L3_IP6_EXT |
					 MVPP2_PRS_RI_IP_FRAG_FALSE |
					 MVPP2_PRS_RI_L4_UDP,
					 MVPP2_PRS_RI_VLAN_MASK |
					 MVPP2_PRS_RI_L3_PROTO_MASK |
					 MVPP2_PRS_RI_IP_FRAG_MASK |
					 MVPP2_PRS_RI_L4_PROTO_MASK} },

	{MVPP2_PRS_FL_IP6_TCP_NF_TAG,	{MVPP2_PRS_RI_L3_IP6 |
					 MVPP2_PRS_RI_IP_FRAG_FALSE |
					 MVPP2_PRS_RI_L4_TCP,
					 MVPP2_PRS_RI_L3_PROTO_MASK |
					 MVPP2_PRS_RI_IP_FRAG_MASK |
					 MVPP2_PRS_RI_L4_PROTO_MASK} },
	{MVPP2_PRS_FL_IP6_TCP_NF_TAG,	{MVPP2_PRS_RI_L3_IP6_EXT |
					 MVPP2_PRS_RI_IP_FRAG_FALSE |
					 MVPP2_PRS_RI_L4_TCP,
					 MVPP2_PRS_RI_L3_PROTO_MASK |
					 MVPP2_PRS_RI_IP_FRAG_MASK |
					 MVPP2_PRS_RI_L4_PROTO_MASK} },

	{MVPP2_PRS_FL_IP6_UDP_NF_TAG,	{MVPP2_PRS_RI_L3_IP6 |
					 MVPP2_PRS_RI_IP_FRAG_FALSE |
					 MVPP2_PRS_RI_L4_UDP,
					 MVPP2_PRS_RI_L3_PROTO_MASK |
					 MVPP2_PRS_RI_IP_FRAG_MASK |
					 MVPP2_PRS_RI_L4_PROTO_MASK} },
	{MVPP2_PRS_FL_IP6_UDP_NF_TAG,	{MVPP2_PRS_RI_L3_IP6_EXT |
					 MVPP2_PRS_RI_IP_FRAG_FALSE |
					 MVPP2_PRS_RI_L4_UDP,
					 MVPP2_PRS_RI_L3_PROTO_MASK |
					 MVPP2_PRS_RI_IP_FRAG_MASK |
					 MVPP2_PRS_RI_L4_PROTO_MASK} },

	{MVPP2_PRS_FL_IP4_TCP_FRAG_UNTAG, {MVPP2_PRS_RI_VLAN_NONE |
					   MVPP2_PRS_RI_L3_IP4 |
					   MVPP2_PRS_RI_IP_FRAG_TRUE |
					   MVPP2_PRS_RI_L4_TCP,
					   MVPP2_PRS_RI_VLAN_MASK |
					   MVPP2_PRS_RI_L3_PROTO_MASK |
					   MVPP2_PRS_RI_IP_FRAG_MASK |
					   MVPP2_PRS_RI_L4_PROTO_MASK} },
	{MVPP2_PRS_FL_IP4_TCP_FRAG_UNTAG, {MVPP2_PRS_RI_VLAN_NONE |
					   MVPP2_PRS_RI_L3_IP4_OPT |
					   MVPP2_PRS_RI_IP_FRAG_TRUE |
					   MVPP2_PRS_RI_L4_TCP,
					   MVPP2_PRS_RI_VLAN_MASK |
					   MVPP2_PRS_RI_L3_PROTO_MASK |
					   MVPP2_PRS_RI_IP_FRAG_MASK |
					   MVPP2_PRS_RI_L4_PROTO_MASK} },
	{MVPP2_PRS_FL_IP4_TCP_FRAG_UNTAG, {MVPP2_PRS_RI_VLAN_NONE |
					   MVPP2_PRS_RI_L3_IP4_OTHER |
					   MVPP2_PRS_RI_IP_FRAG_TRUE |
					   MVPP2_PRS_RI_L4_TCP,
					   MVPP2_PRS_RI_VLAN_MASK |
					   MVPP2_PRS_RI_L3_PROTO_MASK |
					   MVPP2_PRS_RI_IP_FRAG_MASK |
					   MVPP2_PRS_RI_L4_PROTO_MASK} },

	{MVPP2_PRS_FL_IP4_UDP_FRAG_UNTAG, {MVPP2_PRS_RI_VLAN_NONE |
					   MVPP2_PRS_RI_L3_IP4 |
					   MVPP2_PRS_RI_IP_FRAG_TRUE |
					   MVPP2_PRS_RI_L4_UDP,
					   MVPP2_PRS_RI_VLAN_MASK |
					   MVPP2_PRS_RI_L3_PROTO_MASK |
					   MVPP2_PRS_RI_IP_FRAG_MASK |
					   MVPP2_PRS_RI_L4_PROTO_MASK} },
	{MVPP2_PRS_FL_IP4_UDP_FRAG_UNTAG, {MVPP2_PRS_RI_VLAN_NONE |
					   MVPP2_PRS_RI_L3_IP4_OPT |
					   MVPP2_PRS_RI_IP_FRAG_TRUE |
					   MVPP2_PRS_RI_L4_UDP,
					   MVPP2_PRS_RI_VLAN_MASK |
					   MVPP2_PRS_RI_L3_PROTO_MASK |
					   MVPP2_PRS_RI_IP_FRAG_MASK |
					   MVPP2_PRS_RI_L4_PROTO_MASK} },
	{MVPP2_PRS_FL_IP4_UDP_FRAG_UNTAG, {MVPP2_PRS_RI_VLAN_NONE |
					   MVPP2_PRS_RI_L3_IP4_OTHER |
					   MVPP2_PRS_RI_IP_FRAG_TRUE |
					   MVPP2_PRS_RI_L4_UDP,
					   MVPP2_PRS_RI_VLAN_MASK |
					   MVPP2_PRS_RI_L3_PROTO_MASK |
					   MVPP2_PRS_RI_IP_FRAG_MASK |
					   MVPP2_PRS_RI_L4_PROTO_MASK} },

	{MVPP2_PRS_FL_IP4_TCP_FRAG_TAG, {MVPP2_PRS_RI_L3_IP4 |
					 MVPP2_PRS_RI_IP_FRAG_TRUE |
					 MVPP2_PRS_RI_L4_TCP,
					 MVPP2_PRS_RI_L3_PROTO_MASK |
					 MVPP2_PRS_RI_IP_FRAG_MASK |
					 MVPP2_PRS_RI_L4_PROTO_MASK} },
	{MVPP2_PRS_FL_IP4_TCP_FRAG_TAG, {MVPP2_PRS_RI_L3_IP4_OPT |
					 MVPP2_PRS_RI_IP_FRAG_TRUE |
					 MVPP2_PRS_RI_L4_TCP,
					 MVPP2_PRS_RI_L3_PROTO_MASK |
					 MVPP2_PRS_RI_IP_FRAG_MASK |
					 MVPP2_PRS_RI_L4_PROTO_MASK} },
	{MVPP2_PRS_FL_IP4_TCP_FRAG_TAG, {MVPP2_PRS_RI_L3_IP4_OTHER |
					 MVPP2_PRS_RI_IP_FRAG_TRUE |
					 MVPP2_PRS_RI_L4_TCP,
					 MVPP2_PRS_RI_L3_PROTO_MASK |
					 MVPP2_PRS_RI_IP_FRAG_MASK |
					 MVPP2_PRS_RI_L4_PROTO_MASK} },

	{MVPP2_PRS_FL_IP4_UDP_FRAG_TAG, {MVPP2_PRS_RI_L3_IP4 |
					 MVPP2_PRS_RI_IP_FRAG_TRUE |
					 MVPP2_PRS_RI_L4_UDP,
					 MVPP2_PRS_RI_L3_PROTO_MASK |
					 MVPP2_PRS_RI_IP_FRAG_MASK |
					 MVPP2_PRS_RI_L4_PROTO_MASK} },
	{MVPP2_PRS_FL_IP4_UDP_FRAG_TAG,	{MVPP2_PRS_RI_L3_IP4_OPT |
					 MVPP2_PRS_RI_IP_FRAG_TRUE |
					 MVPP2_PRS_RI_L4_UDP,
					 MVPP2_PRS_RI_L3_PROTO_MASK |
					 MVPP2_PRS_RI_IP_FRAG_MASK |
					 MVPP2_PRS_RI_L4_PROTO_MASK} },
	{MVPP2_PRS_FL_IP4_UDP_FRAG_TAG,	{MVPP2_PRS_RI_L3_IP4_OTHER |
					 MVPP2_PRS_RI_IP_FRAG_TRUE |
					 MVPP2_PRS_RI_L4_UDP,
					 MVPP2_PRS_RI_L3_PROTO_MASK |
					 MVPP2_PRS_RI_IP_FRAG_MASK |
					 MVPP2_PRS_RI_L4_PROTO_MASK} },

	{MVPP2_PRS_FL_IP6_TCP_FRAG_UNTAG, {MVPP2_PRS_RI_VLAN_NONE |
					   MVPP2_PRS_RI_L3_IP6 |
					   MVPP2_PRS_RI_IP_FRAG_TRUE |
					   MVPP2_PRS_RI_L4_TCP,
					   MVPP2_PRS_RI_VLAN_MASK |
					   MVPP2_PRS_RI_L3_PROTO_MASK |
					   MVPP2_PRS_RI_IP_FRAG_MASK |
					   MVPP2_PRS_RI_L4_PROTO_MASK} },
	{MVPP2_PRS_FL_IP6_TCP_FRAG_UNTAG, {MVPP2_PRS_RI_VLAN_NONE |
					   MVPP2_PRS_RI_L3_IP6_EXT |
					   MVPP2_PRS_RI_IP_FRAG_TRUE |
					   MVPP2_PRS_RI_L4_TCP,
					   MVPP2_PRS_RI_VLAN_MASK |
					   MVPP2_PRS_RI_L3_PROTO_MASK |
					   MVPP2_PRS_RI_IP_FRAG_MASK |
					   MVPP2_PRS_RI_L4_PROTO_MASK} },

	{MVPP2_PRS_FL_IP6_UDP_FRAG_UNTAG, {MVPP2_PRS_RI_VLAN_NONE |
					   MVPP2_PRS_RI_L3_IP6 |
					   MVPP2_PRS_RI_IP_FRAG_TRUE |
					   MVPP2_PRS_RI_L4_UDP,
					   MVPP2_PRS_RI_VLAN_MASK |
					   MVPP2_PRS_RI_L3_PROTO_MASK |
					   MVPP2_PRS_RI_IP_FRAG_MASK |
					   MVPP2_PRS_RI_L4_PROTO_MASK} },
	{MVPP2_PRS_FL_IP6_UDP_FRAG_UNTAG, {MVPP2_PRS_RI_VLAN_NONE |
					   MVPP2_PRS_RI_L3_IP6_EXT |
					   MVPP2_PRS_RI_IP_FRAG_TRUE |
					   MVPP2_PRS_RI_L4_UDP,
					   MVPP2_PRS_RI_VLAN_MASK |
					   MVPP2_PRS_RI_L3_PROTO_MASK |
					   MVPP2_PRS_RI_IP_FRAG_MASK |
					   MVPP2_PRS_RI_L4_PROTO_MASK} },

	{MVPP2_PRS_FL_IP6_TCP_FRAG_TAG,	{MVPP2_PRS_RI_L3_IP6 |
					 MVPP2_PRS_RI_IP_FRAG_TRUE |
					 MVPP2_PRS_RI_L4_TCP,
					 MVPP2_PRS_RI_L3_PROTO_MASK |
					 MVPP2_PRS_RI_IP_FRAG_MASK |
					 MVPP2_PRS_RI_L4_PROTO_MASK} },
	{MVPP2_PRS_FL_IP6_TCP_FRAG_TAG,	{MVPP2_PRS_RI_L3_IP6_EXT |
					 MVPP2_PRS_RI_IP_FRAG_TRUE |
					 MVPP2_PRS_RI_L4_TCP,
					 MVPP2_PRS_RI_L3_PROTO_MASK |
					 MVPP2_PRS_RI_IP_FRAG_MASK |
					 MVPP2_PRS_RI_L4_PROTO_MASK} },

	{MVPP2_PRS_FL_IP6_UDP_FRAG_TAG,	{MVPP2_PRS_RI_L3_IP6 |
					 MVPP2_PRS_RI_IP_FRAG_TRUE |
					 MVPP2_PRS_RI_L4_UDP,
					 MVPP2_PRS_RI_L3_PROTO_MASK |
					 MVPP2_PRS_RI_IP_FRAG_MASK |
					 MVPP2_PRS_RI_L4_PROTO_MASK} },
	{MVPP2_PRS_FL_IP6_UDP_FRAG_TAG, {MVPP2_PRS_RI_L3_IP6_EXT |
					 MVPP2_PRS_RI_IP_FRAG_TRUE |
					 MVPP2_PRS_RI_L4_UDP,
					 MVPP2_PRS_RI_L3_PROTO_MASK |
					 MVPP2_PRS_RI_IP_FRAG_MASK |
					 MVPP2_PRS_RI_L4_PROTO_MASK} },

	{MVPP2_PRS_FL_IP4_UNTAG, {MVPP2_PRS_RI_VLAN_NONE |
				  MVPP2_PRS_RI_L3_IP4,
				  MVPP2_PRS_RI_VLAN_MASK |
				  MVPP2_PRS_RI_L3_PROTO_MASK} },
	{MVPP2_PRS_FL_IP4_UNTAG, {MVPP2_PRS_RI_VLAN_NONE |
				  MVPP2_PRS_RI_L3_IP4_OPT,
				  MVPP2_PRS_RI_VLAN_MASK |
				  MVPP2_PRS_RI_L3_PROTO_MASK} },
	{MVPP2_PRS_FL_IP4_UNTAG, {MVPP2_PRS_RI_VLAN_NONE |
				  MVPP2_PRS_RI_L3_IP4_OTHER,
				  MVPP2_PRS_RI_VLAN_MASK |
				  MVPP2_PRS_RI_L3_PROTO_MASK} },

	{MVPP2_PRS_FL_IP4_TAG, {MVPP2_PRS_RI_L3_IP4,
				MVPP2_PRS_RI_L3_PROTO_MASK} },
	{MVPP2_PRS_FL_IP4_TAG, {MVPP2_PRS_RI_L3_IP4_OPT,
				MVPP2_PRS_RI_L3_PROTO_MASK} },
	{MVPP2_PRS_FL_IP4_TAG, {MVPP2_PRS_RI_L3_IP4_OTHER,
				MVPP2_PRS_RI_L3_PROTO_MASK} },

	{MVPP2_PRS_FL_IP6_UNTAG, {MVPP2_PRS_RI_VLAN_NONE |
				  MVPP2_PRS_RI_L3_IP6,
				  MVPP2_PRS_RI_VLAN_MASK |
				  MVPP2_PRS_RI_L3_PROTO_MASK} },
	{MVPP2_PRS_FL_IP6_UNTAG, {MVPP2_PRS_RI_VLAN_NONE |
				  MVPP2_PRS_RI_L3_IP6_EXT,
				  MVPP2_PRS_RI_VLAN_MASK |
				  MVPP2_PRS_RI_L3_PROTO_MASK} },

	{MVPP2_PRS_FL_IP6_TAG, {MVPP2_PRS_RI_L3_IP6,
				MVPP2_PRS_RI_L3_PROTO_MASK} },
	{MVPP2_PRS_FL_IP6_TAG, {MVPP2_PRS_RI_L3_IP6_EXT,
				MVPP2_PRS_RI_L3_PROTO_MASK} },

	{MVPP2_PRS_FL_NON_IP_UNTAG, {MVPP2_PRS_RI_VLAN_NONE,
				     MVPP2_PRS_RI_VLAN_MASK} },

	{MVPP2_PRS_FL_NON_IP_TAG, {0, 0} },
};

/* Array of bitmask to indicate flow id attribute */
static int mv_pp2x_prs_flow_id_attr_tbl[MVPP2_PRS_FL_LAST];

static void mv_pp2x_prs_flow_id_attr_set(int flow_id, int ri, int ri_mask)
{
	int flow_attr = 0;

	flow_attr |= MVPP2_PRS_FL_ATTR_VLAN_BIT;
	if (ri_mask & MVPP2_PRS_RI_VLAN_MASK &&
	    (ri & MVPP2_PRS_RI_VLAN_MASK) == MVPP2_PRS_RI_VLAN_NONE)
		flow_attr &= ~MVPP2_PRS_FL_ATTR_VLAN_BIT;

	if ((ri & MVPP2_PRS_RI_L3_PROTO_MASK) == MVPP2_PRS_RI_L3_IP4 ||
	    (ri & MVPP2_PRS_RI_L3_PROTO_MASK) == MVPP2_PRS_RI_L3_IP4_OPT ||
	    (ri & MVPP2_PRS_RI_L3_PROTO_MASK) == MVPP2_PRS_RI_L3_IP4_OTHER)
		flow_attr |= MVPP2_PRS_FL_ATTR_IP4_BIT;

	if ((ri & MVPP2_PRS_RI_L3_PROTO_MASK) == MVPP2_PRS_RI_L3_IP6 ||
	    (ri & MVPP2_PRS_RI_L3_PROTO_MASK) == MVPP2_PRS_RI_L3_IP6_EXT)
		flow_attr |= MVPP2_PRS_FL_ATTR_IP6_BIT;

	if ((ri & MVPP2_PRS_RI_L3_PROTO_MASK) == MVPP2_PRS_RI_L3_ARP)
		flow_attr |= MVPP2_PRS_FL_ATTR_ARP_BIT;

	if (ri & MVPP2_PRS_RI_IP_FRAG_MASK)
		flow_attr |= MVPP2_PRS_FL_ATTR_FRAG_BIT;

	if ((ri & MVPP2_PRS_RI_L4_PROTO_MASK) == MVPP2_PRS_RI_L4_TCP)
		flow_attr |= MVPP2_PRS_FL_ATTR_TCP_BIT;

	if ((ri & MVPP2_PRS_RI_L4_PROTO_MASK) == MVPP2_PRS_RI_L4_UDP)
		flow_attr |= MVPP2_PRS_FL_ATTR_UDP_BIT;

	mv_pp2x_prs_flow_id_attr_tbl[flow_id] = flow_attr;
}

/* Init lookup id attribute array */
static void mv_pp2x_prs_flow_id_attr_init(void)
{
	int index;
	u32 ri, ri_mask, flow_id;

	for (index = 0; index < MVPP2_PRS_FL_TCAM_NUM; index++) {
		ri = mv_pp2x_prs_flow_id_array[index].prs_result.ri;
		ri_mask = mv_pp2x_prs_flow_id_array[index].prs_result.ri_mask;
		flow_id = mv_pp2x_prs_flow_id_array[index].flow_id;

		mv_pp2x_prs_flow_id_attr_set(flow_id, ri, ri_mask);
	}
}

int mv_pp2x_prs_flow_id_attr_get(int flow_id)
{
	return mv_pp2x_prs_flow_id_attr_tbl[flow_id];
}

/* Update parser tcam and sram hw entries */
static int mv_pp2x_prs_hw_write(uintptr_t cpu_slot, struct mv_pp2x_prs_entry *pe)
{
	int i;

	if (pe->index > MVPP2_PRS_TCAM_SRAM_SIZE - 1)
		return -EINVAL;

	/* Clear entry invalidation bit */
	pe->tcam.word[MVPP2_PRS_TCAM_INV_WORD] &= ~MVPP2_PRS_TCAM_INV_MASK;

	/* Write tcam index - indirect access */
	pp2_reg_write(cpu_slot, MVPP2_PRS_TCAM_IDX_REG, pe->index);
	for (i = 0; i < MVPP2_PRS_TCAM_WORDS; i++)
		pp2_reg_write(cpu_slot, MVPP2_PRS_TCAM_DATA_REG(i), pe->tcam.word[i]);

	/* Write sram index - indirect access */
	pp2_reg_write(cpu_slot, MVPP2_PRS_SRAM_IDX_REG, pe->index);
	for (i = 0; i < MVPP2_PRS_SRAM_WORDS; i++)
		pp2_reg_write(cpu_slot, MVPP2_PRS_SRAM_DATA_REG(i), pe->sram.word[i]);

	return 0;
}

/* Read tcam entry from hw */
static int mv_pp2x_prs_hw_read(uintptr_t cpu_slot, struct mv_pp2x_prs_entry *pe)
{
	int i;

	if (pe->index > MVPP2_PRS_TCAM_SRAM_SIZE - 1)
		return -EINVAL;

	/* Write tcam index - indirect access */
	pp2_reg_write(cpu_slot, MVPP2_PRS_TCAM_IDX_REG, pe->index);

	pe->tcam.word[MVPP2_PRS_TCAM_INV_WORD] =
	    pp2_reg_read(cpu_slot, MVPP2_PRS_TCAM_DATA_REG(MVPP2_PRS_TCAM_INV_WORD));
	if (pe->tcam.word[MVPP2_PRS_TCAM_INV_WORD] & MVPP2_PRS_TCAM_INV_MASK)
		return MVPP2_PRS_TCAM_ENTRY_INVALID;

	for (i = 0; i < MVPP2_PRS_TCAM_WORDS; i++)
		pe->tcam.word[i] = pp2_reg_read(cpu_slot, MVPP2_PRS_TCAM_DATA_REG(i));

	/* Write sram index - indirect access */
	pp2_reg_write(cpu_slot, MVPP2_PRS_SRAM_IDX_REG, pe->index);
	for (i = 0; i < MVPP2_PRS_SRAM_WORDS; i++)
		pe->sram.word[i] = pp2_reg_read(cpu_slot, MVPP2_PRS_SRAM_DATA_REG(i));

	return 0;
}

/* Invalidate tcam hw entry */
static void mv_pp2x_prs_hw_inv(uintptr_t cpu_slot, int index)
{
	/* Write index - indirect access */
	pp2_reg_write(cpu_slot, MVPP2_PRS_TCAM_IDX_REG, index);
	pp2_reg_write(cpu_slot, MVPP2_PRS_TCAM_DATA_REG(MVPP2_PRS_TCAM_INV_WORD), MVPP2_PRS_TCAM_INV_MASK);
}

/* Set bits in sram sw entry */
static void mv_pp2x_prs_sram_bits_set(struct mv_pp2x_prs_entry *pe, int bit_num, int val)
{
	pe->sram.byte[SRAM_BIT_TO_BYTE(bit_num)] |= (val << (bit_num % 8));
}

/* Clear bits in sram sw entry */
static void mv_pp2x_prs_sram_bits_clear(struct mv_pp2x_prs_entry *pe, int bit_num, int val)
{
	pe->sram.byte[SRAM_BIT_TO_BYTE(bit_num)] &= ~(val << (bit_num % 8));
}

/* Update ri bits in sram sw entry */
static void mv_pp2x_prs_sram_ri_update(struct mv_pp2x_prs_entry *pe,
				unsigned int bits, unsigned int mask)
{
	unsigned int i;

	for (i = 0; i < MVPP2_PRS_SRAM_RI_CTRL_BITS; i++) {
		int ri_off = MVPP2_PRS_SRAM_RI_OFFS;

		if (!(mask & BIT(i)))
			continue;

		if (bits & BIT(i))
			mv_pp2x_prs_sram_bits_set(pe, ri_off + i, 1);
		else
			mv_pp2x_prs_sram_bits_clear(pe, ri_off + i, 1);

		mv_pp2x_prs_sram_bits_set(pe,
					  MVPP2_PRS_SRAM_RI_CTRL_OFFS + i, 1);
	}
}

/* Obtain ri bits from sram sw entry */
static int mv_pp2x_prs_sram_ri_get(struct mv_pp2x_prs_entry *pe)
{
	return pe->sram.word[MVPP2_PRS_SRAM_RI_WORD];
}

static int mv_pp2x_prs_sram_ri_mask_get(struct mv_pp2x_prs_entry *pe)
{
	return pe->sram.word[MVPP2_PRS_SRAM_RI_CTRL_WORD];
}

/* Update ai bits in sram sw entry */
static void mv_pp2x_prs_sram_ai_update(struct mv_pp2x_prs_entry *pe,
				unsigned int bits, unsigned int mask)
{
	unsigned int i;
	int ai_off = MVPP2_PRS_SRAM_AI_OFFS;

	for (i = 0; i < MVPP2_PRS_SRAM_AI_CTRL_BITS; i++) {
		if (!(mask & BIT(i)))
			continue;

		if (bits & BIT(i))
			mv_pp2x_prs_sram_bits_set(pe, ai_off + i, 1);
		else
			mv_pp2x_prs_sram_bits_clear(pe, ai_off + i, 1);

		mv_pp2x_prs_sram_bits_set(pe,
					  MVPP2_PRS_SRAM_AI_CTRL_OFFS + i, 1);
	}
}

/* Read ai bits from sram sw entry */
static int mv_pp2x_prs_sram_ai_get(struct mv_pp2x_prs_entry *pe)
{
	u8 bits;
	int ai_off = SRAM_BIT_TO_BYTE(MVPP2_PRS_SRAM_AI_OFFS);
	int ai_en_off = ai_off + 1;
	int ai_shift = MVPP2_PRS_SRAM_AI_OFFS % 8;

	bits = (pe->sram.byte[ai_off] >> ai_shift) |
	    (pe->sram.byte[ai_en_off] << (8 - ai_shift));

	return bits;
}

/* In sram sw entry set lookup ID field of the tcam key to be used in the next
 * lookup interation
 */
static void mv_pp2x_prs_sram_next_lu_set(struct mv_pp2x_prs_entry *pe, unsigned int lu)
{
	int sram_next_off = MVPP2_PRS_SRAM_NEXT_LU_OFFS;

	mv_pp2x_prs_sram_bits_clear(pe, sram_next_off,
				    MVPP2_PRS_SRAM_NEXT_LU_MASK);
	mv_pp2x_prs_sram_bits_set(pe, sram_next_off, lu);
}

/* In the sram sw entry set sign and value of the next lookup offset
 * and the offset value generated to the classifier
 */
static void mv_pp2x_prs_sram_shift_set(struct mv_pp2x_prs_entry *pe, int shift,
				       unsigned int op)
{
	/* Set sign */
	if (shift < 0) {
		mv_pp2x_prs_sram_bits_set(pe, MVPP2_PRS_SRAM_SHIFT_SIGN_BIT, 1);
		shift = 0 - shift;
	} else {
		mv_pp2x_prs_sram_bits_clear(pe,
					    MVPP2_PRS_SRAM_SHIFT_SIGN_BIT, 1);
	}

	/* Set value */
	pe->sram.byte[SRAM_BIT_TO_BYTE(MVPP2_PRS_SRAM_SHIFT_OFFS)] =
	    (unsigned char)shift;

	/* Reset and set operation */
	mv_pp2x_prs_sram_bits_clear(pe, MVPP2_PRS_SRAM_OP_SEL_SHIFT_OFFS,
				    MVPP2_PRS_SRAM_OP_SEL_SHIFT_MASK);
	mv_pp2x_prs_sram_bits_set(pe, MVPP2_PRS_SRAM_OP_SEL_SHIFT_OFFS, op);

	/* Set base offset as current */
	mv_pp2x_prs_sram_bits_clear(pe, MVPP2_PRS_SRAM_OP_SEL_BASE_OFFS, 1);
}

/* In the sram sw entry set sign and value of the user defined offset
 * generated to the classifier
 */
static void mv_pp2x_prs_sram_offset_set(struct mv_pp2x_prs_entry *pe,
					unsigned int type, int offset,
					unsigned int op)
{
	/* Set sign */
	if (offset < 0) {
		mv_pp2x_prs_sram_bits_set(pe, MVPP2_PRS_SRAM_UDF_SIGN_BIT, 1);
		offset = 0 - offset;
	} else {
		mv_pp2x_prs_sram_bits_clear(pe, MVPP2_PRS_SRAM_UDF_SIGN_BIT, 1);
	}

	/* Set value */
	mv_pp2x_prs_sram_bits_clear(pe, MVPP2_PRS_SRAM_UDF_OFFS,
				    MVPP2_PRS_SRAM_UDF_MASK);
	mv_pp2x_prs_sram_bits_set(pe, MVPP2_PRS_SRAM_UDF_OFFS, offset);
	pe->sram.byte[SRAM_BIT_TO_BYTE(MVPP2_PRS_SRAM_UDF_OFFS +
				       MVPP2_PRS_SRAM_UDF_BITS)] &=
	    ~(MVPP2_PRS_SRAM_UDF_MASK >> (8 - (MVPP2_PRS_SRAM_UDF_OFFS % 8)));
	pe->sram.byte[SRAM_BIT_TO_BYTE(MVPP2_PRS_SRAM_UDF_OFFS +
				       MVPP2_PRS_SRAM_UDF_BITS)] |=
	    (offset >> (8 - (MVPP2_PRS_SRAM_UDF_OFFS % 8)));

	/* Set offset type */
	mv_pp2x_prs_sram_bits_clear(pe, MVPP2_PRS_SRAM_UDF_TYPE_OFFS,
				    MVPP2_PRS_SRAM_UDF_TYPE_MASK);
	mv_pp2x_prs_sram_bits_set(pe, MVPP2_PRS_SRAM_UDF_TYPE_OFFS, type);

	/* Set offset operation */
	mv_pp2x_prs_sram_bits_clear(pe, MVPP2_PRS_SRAM_OP_SEL_UDF_OFFS,
				    MVPP2_PRS_SRAM_OP_SEL_UDF_MASK);
	mv_pp2x_prs_sram_bits_set(pe, MVPP2_PRS_SRAM_OP_SEL_UDF_OFFS, op);

	pe->sram.byte[SRAM_BIT_TO_BYTE(MVPP2_PRS_SRAM_OP_SEL_UDF_OFFS +
				       MVPP2_PRS_SRAM_OP_SEL_UDF_BITS)] &=
	    ~(MVPP2_PRS_SRAM_OP_SEL_UDF_MASK >>
	      (8 - (MVPP2_PRS_SRAM_OP_SEL_UDF_OFFS % 8)));

	pe->sram.byte[SRAM_BIT_TO_BYTE(MVPP2_PRS_SRAM_OP_SEL_UDF_OFFS +
				       MVPP2_PRS_SRAM_OP_SEL_UDF_BITS)] |=
	    (op >> (8 - (MVPP2_PRS_SRAM_OP_SEL_UDF_OFFS % 8)));

	/* Set base offset as current */
	mv_pp2x_prs_sram_bits_clear(pe, MVPP2_PRS_SRAM_OP_SEL_BASE_OFFS, 1);
}

/* Get byte of data and its enable bits from tcam sw entry */
static void mv_pp2x_prs_tcam_data_byte_get(struct mv_pp2x_prs_entry *pe,
					   unsigned int offs,
					   unsigned char *byte,
					   unsigned char *enable)
{
	*byte = pe->tcam.byte[TCAM_DATA_BYTE(offs)];
	*enable = pe->tcam.byte[TCAM_DATA_MASK(offs)];
}

/* Set byte of data and its enable bits in tcam sw entry */
static void mv_pp2x_prs_tcam_data_byte_set(struct mv_pp2x_prs_entry *pe,
				    unsigned int offs,
				    unsigned char byte, unsigned char enable)
{
	pe->tcam.byte[TCAM_DATA_BYTE(offs)] = byte;
	pe->tcam.byte[TCAM_DATA_MASK(offs)] = enable;
}

/* Set dword of data and its enable bits in tcam sw entry */
static void mv_pp2x_prs_tcam_data_dword_set(struct mv_pp2x_prs_entry *pe,
					    unsigned int offs,
					    unsigned int word,
					    unsigned int enable)
{
	int index, offset;
	unsigned char byte, byte_mask;

	for (index = 0; index < 4; index++) {
		offset = (offs * 4) + index;
		byte = ((unsigned char *)&word)[HW_BYTE_OFFS(index)];
		byte_mask = ((unsigned char *)&enable)[HW_BYTE_OFFS(index)];
		mv_pp2x_prs_tcam_data_byte_set(pe, offset, byte, byte_mask);
	}
}

/* Update ai bits in tcam sw entry */
static void mv_pp2x_prs_tcam_ai_update(struct mv_pp2x_prs_entry *pe,
				unsigned int bits, unsigned int enable)
{
	int i, ai_idx = MVPP2_PRS_TCAM_AI_BYTE;

	for (i = 0; i < MVPP2_PRS_AI_BITS; i++) {
		if (!(enable & BIT(i)))
			continue;

		if (bits & BIT(i))
			pe->tcam.byte[HW_BYTE_OFFS(ai_idx)] |= 1 << i;
		else
			pe->tcam.byte[HW_BYTE_OFFS(ai_idx)] &= ~(1 << i);
	}

	pe->tcam.byte[HW_BYTE_OFFS(MVPP2_PRS_TCAM_EN_OFFS(ai_idx))] |= enable;
}

/* Get ai bits from tcam sw entry */
static int mv_pp2x_prs_tcam_ai_get(struct mv_pp2x_prs_entry *pe)
{
	return pe->tcam.byte[HW_BYTE_OFFS(MVPP2_PRS_TCAM_AI_BYTE)];
}

/* Set ethertype in tcam sw entry */
static void mv_pp2x_prs_match_etype(struct mv_pp2x_prs_entry *pe, int offset,
				    u16 ethertype)
{
	mv_pp2x_prs_tcam_data_byte_set(pe, offset + 0, ethertype >> 8, 0xff);
	mv_pp2x_prs_tcam_data_byte_set(pe, offset + 1, ethertype & 0xff, 0xff);
}

/* Compare MAC DA with tcam entry data */
static bool mv_pp2x_prs_mac_range_equals(struct mv_pp2x_prs_entry *pe,
					 const u8 *da, unsigned char *mask)
{
	unsigned char tcam_byte, tcam_mask;
	int index;

	for (index = 0; index < ETH_ALEN; index++) {
		mv_pp2x_prs_tcam_data_byte_get(pe, index, &tcam_byte,
					       &tcam_mask);
		if (tcam_mask != mask[index])
			return false;

		if ((tcam_mask & tcam_byte) != (da[index] & mask[index]))
			return false;
	}

	return true;
}

/* Obtain port map from tcam sw entry */
static unsigned int mv_pp2x_prs_tcam_port_map_get(struct mv_pp2x_prs_entry *pe)
{
	int enable_off =
	    HW_BYTE_OFFS(MVPP2_PRS_TCAM_EN_OFFS(MVPP2_PRS_TCAM_PORT_BYTE));

	return ~(pe->tcam.byte[enable_off]) & MVPP2_PRS_PORT_MASK;
}

/* Update port map in tcam sw entry */
static void mv_pp2x_prs_tcam_port_map_set(struct mv_pp2x_prs_entry *pe,
				   unsigned int ports)
{
	unsigned char port_mask = MVPP2_PRS_PORT_MASK;
	int enable_off =
	    HW_BYTE_OFFS(MVPP2_PRS_TCAM_EN_OFFS(MVPP2_PRS_TCAM_PORT_BYTE));

	pe->tcam.byte[HW_BYTE_OFFS(MVPP2_PRS_TCAM_PORT_BYTE)] = 0;
	pe->tcam.byte[enable_off] &= ~port_mask;
	pe->tcam.byte[enable_off] |= ~ports & MVPP2_PRS_PORT_MASK;
}

/* Update mask for single port in tcam sw entry */
static void mv_pp2x_prs_tcam_port_set(struct mv_pp2x_prs_entry *pe,
			       unsigned int port, bool add)
{
	int enable_off =
	    HW_BYTE_OFFS(MVPP2_PRS_TCAM_EN_OFFS(MVPP2_PRS_TCAM_PORT_BYTE));

	if (add)
		pe->tcam.byte[enable_off] &= ~(1 << port);
	else
		pe->tcam.byte[enable_off] |= 1 << port;
}

static void mv_pp2x_prs_tcam_port_get(struct mv_pp2x_prs_entry *pe, u8 *port)
{
	int enable_off =
	    HW_BYTE_OFFS(MVPP2_PRS_TCAM_EN_OFFS(MVPP2_PRS_TCAM_PORT_BYTE));

	*port = ~pe->tcam.byte[enable_off];
}

/* Enable shadow table entry and set its lookup ID */
static void mv_pp2x_prs_shadow_set(struct pp2_inst *inst, int index, int lu)
{
	inst->cls_db->prs_db.prs_shadow[index].valid = true;
	inst->cls_db->prs_db.prs_shadow[index].lu = lu;
}

/* Update ri fields in shadow table entry */
static void mv_pp2x_prs_shadow_ri_set(struct pp2_inst *inst, int index,
				      unsigned int ri, unsigned int ri_mask)
{
	inst->cls_db->prs_db.prs_shadow[index].ri_mask = ri_mask;
	inst->cls_db->prs_db.prs_shadow[index].ri = ri;
}

/* Update lookup field in tcam sw entry */
static void mv_pp2x_prs_tcam_lu_set(struct mv_pp2x_prs_entry *pe, unsigned int lu)
{
	unsigned int offset = MVPP2_PRS_TCAM_LU_BYTE;
	unsigned int enable_off =
	    MVPP2_PRS_TCAM_EN_OFFS(MVPP2_PRS_TCAM_LU_BYTE);

	pe->tcam.byte[HW_BYTE_OFFS(offset)] = lu;
	pe->tcam.byte[HW_BYTE_OFFS(enable_off)] = MVPP2_PRS_LU_MASK;
}

static int mv_pp2x_prs_tcam_lu_get(struct mv_pp2x_prs_entry *pe)
{
	return pe->tcam.byte[HW_BYTE_OFFS(MVPP2_PRS_TCAM_LU_BYTE)];
}

static int mv_pp2x_prs_tcam_valid_get(struct mv_pp2x_prs_entry *pe)
{
	return ((pe->tcam.word[MVPP2_PRS_TCAM_INV_WORD] & MVPP2_PRS_TCAM_INV_MASK) >> MVPP2_PRS_TCAM_INV_OFFS);
}

/* Return first free tcam index, seeking from start to end */
static int pp2_prs_tcam_first_free(struct pp2_inst *inst, unsigned char start, unsigned char end)
{
	int tid;

	if (start > end)
		swap(start, end);

	if (end >= MVPP2_PRS_TCAM_SRAM_SIZE)
		end = MVPP2_PRS_TCAM_SRAM_SIZE - 1;

	for (tid = start; tid <= end; tid++) {
		if (!inst->cls_db->prs_db.prs_shadow[tid].valid)
			return tid;
	}
	pr_err("Out of TCAM Entries !!\n");
	return -EINVAL;
}


/* Get dword of data and its enable bits from tcam sw entry */
static void mv_pp2x_prs_tcam_data_dword_get(struct mv_pp2x_prs_entry *pe,
					    unsigned int offs,
					    unsigned int *word,
					    unsigned int *enable)
{
	int index, offset;
	unsigned char byte, mask;

	for (index = 0; index < 4; index++) {
		offset = (offs * 4) + index;
		mv_pp2x_prs_tcam_data_byte_get(pe, offset, &byte, &mask);
		((unsigned char *)word)[HW_BYTE_OFFS(index)] = byte;
		((unsigned char *)enable)[HW_BYTE_OFFS(index)] = mask;
	}
}
/*******************************************************************************
 * mv_pp2x_prs_shadow_update
 *
 * DESCRIPTION:	Update MUSDK parser shadow db
 *
 * INPUTS:	inst - packet processor instance
 *
 * OUTPUTS:	None
 *
 * RETURNS:	0 on success, error-code otherwise
 *
 * COMMENTS:
 ******************************************************************************/
static int mv_pp2x_prs_shadow_update(struct pp2_inst *inst)
{
	int i, invalid;
	struct mv_pp2x_prs_entry pe;
	struct mv_pp2x_prs_shadow *prs_shadow;
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);

	if (!inst->cls_db->prs_db.prs_shadow) {
		inst->cls_db->prs_db.prs_shadow = kcalloc(MVPP2_PRS_TCAM_SRAM_SIZE,
							  sizeof(struct mv_pp2x_prs_shadow), GFP_KERNEL);
		if (!inst->cls_db->prs_db.prs_shadow)
			return -ENOMEM;
	}

	prs_shadow = inst->cls_db->prs_db.prs_shadow;

	for (i = 0; i < MVPP2_PRS_TCAM_SRAM_SIZE; i++) {
		pe.index = i;
		mv_pp2x_prs_hw_read(cpu_slot, &pe);
		prs_shadow[i].ri = mv_pp2x_prs_sram_ri_get(&pe);
		prs_shadow[i].ri_mask = mv_pp2x_prs_sram_ri_mask_get(&pe);
		prs_shadow[i].lu = mv_pp2x_prs_tcam_lu_get(&pe);
		invalid = mv_pp2x_prs_tcam_valid_get(&pe);
		prs_shadow[i].valid = invalid ? 0 : 1;
	}
	return 0;
}
/*******************************************************************************
 * pp2_cls_prs_init
 *
 * DESCRIPTION:	Initialize parser for MUSDK.
 *		It assumes parser is initialized by kernel, and MUSDK only performs
 *		additional initializations
 *
 * INPUTS:	inst - packet processor instance
 *
 * OUTPUTS:	None
 *
 * RETURNS:	0 on success, error-code otherwise
 *
 * COMMENTS:	Called by pp2_cls_mng_init.
 ******************************************************************************/
int pp2_cls_prs_init(struct pp2_inst *inst)
{
	int rc;
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);
	u32 val;

	/* Check if tcam table enabled*/
	val = pp2_reg_read(cpu_slot, MVPP2_PRS_TCAM_CTRL_REG);
	if (val != MVPP2_PRS_TCAM_EN_MASK) {
		pr_err("Can't initialize logical port: parser not initialized yet\n");
		return -EFAULT;
	}

	/* Update MUSDK parser shadow table from kernel configuration */
	rc = mv_pp2x_prs_shadow_update(inst);
	if (rc)
		return -EFAULT;

	mv_pp2x_prs_flow_id_attr_init();

	return 0;
}
