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

/* pp2_prs_tcam_lk_idx_list_get
 *
 * DESCRIPTION:	return tcam indexes that matches the specified lookup
 *
 * INPUTS:	inst	- packet processor instance
 *		lookup	- parser lookup id
 *		proto	- protocol to update in parser
 *		ri	- results info field to configure
 *
 * OUTPUTS:	tcam_list	- list of TCAM indexes which match the lookup id
 *
 * RETURNS:	0 on success, error-code otherwise
 */
static int pp2_prs_tcam_lk_idx_list_get(struct pp2_inst *inst, u32 lookup,
					u16 proto, struct prs_lkp_tcam_list *tcam_list, u32 ri)
{
	int tid, i = 0;
	u8 byte;
	struct mv_pp2x_prs_shadow *prs_shadow = inst->cls_db->prs_db.prs_shadow;

	for (tid = MVPP2_PE_FIRST_FREE_TID;
	     tid <= MVPP2_PE_LAST_FREE_TID; tid++) {
		if ((prs_shadow[tid].valid) &&
		    (prs_shadow[tid].lu == lookup)) {
			if (proto) {
				byte = prs_shadow[tid].tcam.byte[TCAM_DATA_BYTE(5)];
				if (byte == proto) {
					*(tcam_list->idx + i) = tid;
					if (prs_shadow[tid].ri & ri)
						*(tcam_list->log_port + i) = 1;
					pr_debug("%d, idx %d, log_port %d\n", i, *(tcam_list->idx + i),
					       *(tcam_list->log_port + i));
					i++;
				}
			} else {
				*(tcam_list->idx + i) = tid;
				if (prs_shadow[tid].ri & ri)
					*(tcam_list->log_port + i) = 1;
				pr_debug("%d, idx %d, log_port %d\n", i, *(tcam_list->idx + i),
				       *(tcam_list->log_port + i));
				i++;
			}
		}
	}
	tcam_list->size = i;
	return 0;
}

/* pp2_prs_log_port_tcam_check
 *
 * DESCRIPTION:	check if logical port entries for this port already exist
 *
 * INPUTS:	tcam_list	- list of TCAM indexes to update
 *
 * OUTPUTS:	None
 *
 * RETURNS:	0 not found, 1 found
 */
static int pp2_prs_log_port_tcam_check(struct prs_lkp_tcam_list *tcam_list)
{
	int ret = 0;
	int i;

	for (i = 0; i < tcam_list->size; i++) {
		if (tcam_list->log_port[i])
			ret = 1;
	}
	return ret;
}

/* pp2_prs_port_update
 *
 * DESCRIPTION:	Update port in specified entry
 *
 * INPUTS:	port	- logical port to be configured
 *		add	- add or remove port
 *		tid	- prs entry index
 *		ri	- results info field to configure
 *		ri_mask	- results info mask field to configure
 *
 * OUTPUTS:	None
 *
 * RETURNS:	0 on success, error-code otherwise
 */
static int pp2_prs_port_update(struct pp2_port *port, u32 add, u32 tid, u32 ri, u32 ri_mask)
{
	struct mv_pp2x_prs_entry pe;
	struct pp2_inst *inst = port->parent;
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);

	if (!inst->cls_db->prs_db.prs_shadow[tid].valid) {
		pr_err("parser logical port special field DSA mode: entry not found\n");
		return -EFAULT;
	}

	pe.index = tid;
	mv_pp2x_prs_hw_read(cpu_slot, &pe);

	/* update UDF7 */
	mv_pp2x_prs_sram_ri_update(&pe, ri, ri_mask);

	/* Update port mask */
	mv_pp2x_prs_tcam_port_set(&pe, port->id, add);

	mv_pp2x_prs_hw_write(cpu_slot, &pe);

	return 0;
}

/* pp2_prs_non_log_port_update
 *
 * DESCRIPTION:	Check and update port mask in non-logical port  entries
 *
 * INPUTS:	port	- logical port to be configured
 *		tcam_list	- list of TCAM indexes to update
 *		ri	- results info field to configure
 *		ri_mask	- results info mask field to configure
 *
 * OUTPUTS:	None
 *
 * RETURNS:	0 on success, error-code otherwise
 */
static int pp2_prs_non_log_port_update(struct pp2_port *port, struct prs_lkp_tcam_list *tcam_list, u32 ri, u32 ri_mask)
{
	int i;
	struct pp2_inst *inst = port->parent;
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);
	struct mv_pp2x_prs_entry pe;

	for (i = 0; i < tcam_list->size; i++) {
		if (!tcam_list->log_port[i]) {
			pe.index = tcam_list->idx[i];
			mv_pp2x_prs_hw_read(cpu_slot, &pe);
			/* update UDF7 to slow path */
			mv_pp2x_prs_sram_ri_update(&pe, MVPP2_PRS_RI_UDF7_CLEAR, MVPP2_PRS_RI_UDF7_MASK);
			mv_pp2x_prs_sram_ri_update(&pe, ri, ri_mask);
			/* Update port mask */
			mv_pp2x_prs_tcam_port_set(&pe, port->id, false);
			mv_pp2x_prs_hw_write(cpu_slot, &pe);
		}
	}
	return 0;
}

/* pp2_prs_log_port_pppoe
 *
 * DESCRIPTION:	Configure parser pppoe for logical port
 *
 * INPUTS:	port	- logical port to be configured
 *		target	- target classification (logical port or nic)
 *
 * OUTPUTS:	None
 *
 * RETURNS:	0 on success, error-code otherwise
 */
static int pp2_prs_log_port_pppoe(struct pp2_port *port, enum pp2_ppio_cls_target target)
{
	struct mv_pp2x_prs_entry pe;
	int tid;
	struct prs_lkp_tcam_list tcam_list;
	struct pp2_inst *inst = port->parent;
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);
	u32 ri = 0, nri = 0;
	u32 ri_mask = 0;

	memset(&tcam_list, 0, sizeof(struct prs_lkp_tcam_list));

	if (target == PP2_CLS_TARGET_LOCAL_PPIO) {
		ri |= MVPP2_PRS_RI_UDF7_LOG_PORT;
		nri |= MVPP2_PRS_RI_UDF7_NIC;
	} else {
		ri |= MVPP2_PRS_RI_UDF7_NIC;
		nri |= MVPP2_PRS_RI_UDF7_LOG_PORT;
	}

	ri_mask |= MVPP2_PRS_RI_UDF7_MASK;

	/* get the current indexes with lookup id MVPP2_PRS_LU_IP4 and specified proto */
	pp2_prs_tcam_lk_idx_list_get(inst, MVPP2_PRS_LU_PPPOE, 0, &tcam_list, ri);

	pr_debug("%s target %d, ri %x, nri %x, mask %x\n", __func__, target, ri, nri, ri_mask);

	if (!pp2_prs_log_port_tcam_check(&tcam_list)) {

		/*Step 1: IPv4 over PPPoE with options */
		tid = pp2_prs_tcam_first_free(inst, MVPP2_PE_FIRST_FREE_TID, MVPP2_PE_LAST_FREE_TID);
		if (tid < 0)
			return tid;

		memset(&pe, 0, sizeof(struct mv_pp2x_prs_entry));
		mv_pp2x_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_PPPOE);
		pe.index = tid;

		mv_pp2x_prs_match_etype(&pe, 0, PPP_PROTO_IPV4);

		mv_pp2x_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_IP4);
		mv_pp2x_prs_sram_ri_update(&pe, ri | MVPP2_PRS_RI_L3_IP4_OPT, ri_mask | MVPP2_PRS_RI_L3_PROTO_MASK);

		/* Skip eth_type + 4 bytes of IP header */
		mv_pp2x_prs_sram_shift_set(&pe, MVPP2_ETH_TYPE_LEN + 4, MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);
		/* Set L3 offset */
		mv_pp2x_prs_sram_offset_set(&pe, MVPP2_PRS_SRAM_UDF_TYPE_L3,
					    MVPP2_ETH_TYPE_LEN,
					    MVPP2_PRS_SRAM_OP_SEL_UDF_ADD);

		/* Update shadow table and hw entry */
		mv_pp2x_prs_shadow_set(inst, pe.index, MVPP2_PRS_LU_PPPOE);
		mv_pp2x_prs_hw_write(cpu_slot, &pe);

		/*Step 2: IPv4 over PPPoE without options */
		tid = pp2_prs_tcam_first_free(inst, MVPP2_PE_FIRST_FREE_TID, MVPP2_PE_LAST_FREE_TID);
		if (tid < 0)
			return tid;

		pe.index = tid;

		mv_pp2x_prs_tcam_data_byte_set(&pe, MVPP2_ETH_TYPE_LEN,
					       MVPP2_PRS_IPV4_HEAD | MVPP2_PRS_IPV4_IHL,
					       MVPP2_PRS_IPV4_HEAD_MASK |
					       MVPP2_PRS_IPV4_IHL_MASK);

		/* Clear ri before updating */
		pe.sram.word[MVPP2_PRS_SRAM_RI_WORD] = 0x0;
		pe.sram.word[MVPP2_PRS_SRAM_RI_CTRL_WORD] = 0x0;
		mv_pp2x_prs_sram_ri_update(&pe, ri | MVPP2_PRS_RI_L3_IP4, ri_mask | MVPP2_PRS_RI_L3_PROTO_MASK);
		/* Update shadow table and hw entry */
		mv_pp2x_prs_shadow_set(inst, pe.index, MVPP2_PRS_LU_PPPOE);
		mv_pp2x_prs_hw_write(cpu_slot, &pe);

		/*Step 3: IPv6 over PPPoE */
		tid = pp2_prs_tcam_first_free(inst, MVPP2_PE_FIRST_FREE_TID, MVPP2_PE_LAST_FREE_TID);
		if (tid < 0)
			return tid;

		memset(&pe, 0, sizeof(struct mv_pp2x_prs_entry));
		mv_pp2x_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_PPPOE);
		pe.index = tid;

		mv_pp2x_prs_match_etype(&pe, 0, PPP_PROTO_IPV6);

		mv_pp2x_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_IP6);
		mv_pp2x_prs_sram_ri_update(&pe, ri | MVPP2_PRS_RI_L3_IP6, ri_mask | MVPP2_PRS_RI_L3_PROTO_MASK);

		/* Skip eth_type + 4 bytes of IPv6 header */
		mv_pp2x_prs_sram_shift_set(&pe, MVPP2_ETH_TYPE_LEN + 4, MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);
		/* Set L3 offset */
		mv_pp2x_prs_sram_offset_set(&pe, MVPP2_PRS_SRAM_UDF_TYPE_L3,
					    MVPP2_ETH_TYPE_LEN,
					    MVPP2_PRS_SRAM_OP_SEL_UDF_ADD);

		/* Update shadow table and hw entry */
		mv_pp2x_prs_shadow_set(inst, pe.index, MVPP2_PRS_LU_PPPOE);
		mv_pp2x_prs_hw_write(cpu_slot, &pe);

		/*Step 4: Non-IP over PPPoE */
		tid = pp2_prs_tcam_first_free(inst, MVPP2_PE_FIRST_FREE_TID, MVPP2_PE_LAST_FREE_TID);
		if (tid < 0)
			return tid;

		memset(&pe, 0, sizeof(struct mv_pp2x_prs_entry));
		mv_pp2x_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_PPPOE);
		pe.index = tid;
		mv_pp2x_prs_sram_ri_update(&pe, ri | MVPP2_PRS_RI_L3_UN, ri_mask | MVPP2_PRS_RI_L3_PROTO_MASK);

		/* Finished: go to flowid generation */
		mv_pp2x_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_FLOWS);
		mv_pp2x_prs_sram_bits_set(&pe, MVPP2_PRS_SRAM_LU_GEN_BIT, 1);
		/* Set L3 offset even if it's unknown L3 */
		mv_pp2x_prs_sram_offset_set(&pe, MVPP2_PRS_SRAM_UDF_TYPE_L3,
					    MVPP2_ETH_TYPE_LEN,
					    MVPP2_PRS_SRAM_OP_SEL_UDF_ADD);

		/* Update shadow table and hw entry */
		mv_pp2x_prs_shadow_set(inst, pe.index, MVPP2_PRS_LU_PPPOE);
		mv_pp2x_prs_hw_write(cpu_slot, &pe);
	}

	pp2_prs_non_log_port_update(port, &tcam_list, nri, ri_mask);

	return 0;
}


/* pp2_prs_log_port_ip4_proto
 *
 * DESCRIPTION:	Configure parser IPv4 specific protocol for logical port
 *
 * INPUTS:	port	- logical port to be configured
 *		proto	- protocol to update in parser
 *		ri	- results info field to configure
 *		ri_mask	- results info mask field to configure
 *		target	- target classification (logical port or nic)
 *
 * OUTPUTS:	None
 *
 * RETURNS:	0 on success, error-code otherwise
 */
static int pp2_prs_log_port_ip4_proto(struct pp2_port *port, u16 proto, u32 ri, u32 ri_mask,
				      enum pp2_ppio_cls_target target)
{
	struct mv_pp2x_prs_entry pe;
	int tid;
	struct pp2_inst *inst = port->parent;
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);
	struct prs_lkp_tcam_list tcam_list;
	u32 nri = 0;

	if ((proto != IPPROTO_TCP) && (proto != IPPROTO_UDP) &&
	    (proto != IPPROTO_IGMP))
		return -EINVAL;

	memset(&tcam_list, 0, sizeof(struct prs_lkp_tcam_list));

	if (target == PP2_CLS_TARGET_LOCAL_PPIO) {
		ri |= MVPP2_PRS_RI_UDF7_LOG_PORT;
		nri |= MVPP2_PRS_RI_UDF7_NIC;
		/* get the current indexes with lookup id MVPP2_PRS_LU_IP4 and specified proto */
		pp2_prs_tcam_lk_idx_list_get(inst, MVPP2_PRS_LU_IP4, proto, &tcam_list, MVPP2_PRS_RI_UDF7_LOG_PORT);
	} else {
		ri |= MVPP2_PRS_RI_UDF7_NIC;
		nri |= MVPP2_PRS_RI_UDF7_LOG_PORT;
		/* get the current indexes with lookup id MVPP2_PRS_LU_IP4 and specified proto */
		pp2_prs_tcam_lk_idx_list_get(inst, MVPP2_PRS_LU_IP4, proto, &tcam_list, MVPP2_PRS_RI_UDF7_NIC);
	}

	ri_mask |= MVPP2_PRS_RI_UDF7_MASK;

	pr_debug("%s target %d, ri %x, nri %x, mask %x\n", __func__, target, ri, nri, ri_mask);

	if (!pp2_prs_log_port_tcam_check(&tcam_list)) {
		/* Create new parser entries for the specified logical port */
		/* step 1: Not fragmented packet */
		tid = pp2_prs_tcam_first_free(inst, MVPP2_PE_FIRST_FREE_TID,
						  MVPP2_PE_LAST_FREE_TID);
		if (tid < 0)
			return tid;

		memset(&pe, 0, sizeof(struct mv_pp2x_prs_entry));
		mv_pp2x_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_IP4);
		pe.index = tid;

		/* Set next lu to IPv4 */
		mv_pp2x_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_IP4);
		mv_pp2x_prs_sram_shift_set(&pe, 12, MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);
		/* Set L4 offset */
		mv_pp2x_prs_sram_offset_set(&pe, MVPP2_PRS_SRAM_UDF_TYPE_L4,
					    sizeof(struct mv_pp2x_iphdr) - 4,
					    MVPP2_PRS_SRAM_OP_SEL_UDF_ADD);
		mv_pp2x_prs_sram_ai_update(&pe, MVPP2_PRS_IPV4_DIP_AI_BIT,
					   MVPP2_PRS_IPV4_DIP_AI_BIT);
		mv_pp2x_prs_sram_ri_update(&pe, ri | MVPP2_PRS_RI_IP_FRAG_FALSE,
					   ri_mask | MVPP2_PRS_RI_IP_FRAG_MASK);

		mv_pp2x_prs_tcam_data_byte_set(&pe, 2, 0x00,
					       MVPP2_PRS_TCAM_PROTO_MASK_L);
		mv_pp2x_prs_tcam_data_byte_set(&pe, 3, 0x00,
					       MVPP2_PRS_TCAM_PROTO_MASK);
		mv_pp2x_prs_tcam_data_byte_set(&pe, 5, proto,
					       MVPP2_PRS_TCAM_PROTO_MASK);
		mv_pp2x_prs_tcam_ai_update(&pe, 0, MVPP2_PRS_IPV4_DIP_AI_BIT);

		/* Mask all ports */
		mv_pp2x_prs_tcam_port_map_set(&pe, 0);

		/* Update port mask */
		mv_pp2x_prs_tcam_port_set(&pe, port->id, true);

		/* Update shadow table and hw entry */
		mv_pp2x_prs_shadow_set(inst, pe.index, MVPP2_PRS_LU_IP4);
		mv_pp2x_prs_hw_write(cpu_slot, &pe);

		/* step 2: Fragmented packet */
		tid = pp2_prs_tcam_first_free(inst, MVPP2_PE_FIRST_FREE_TID,
						  MVPP2_PE_LAST_FREE_TID);
		if (tid < 0)
			return tid;

		pe.index = tid;
		/* Clear ri before updating */
		pe.sram.word[MVPP2_PRS_SRAM_RI_WORD] = 0x0;
		pe.sram.word[MVPP2_PRS_SRAM_RI_CTRL_WORD] = 0x0;
		mv_pp2x_prs_sram_ri_update(&pe, ri, ri_mask);
		mv_pp2x_prs_sram_ri_update(&pe, ri | MVPP2_PRS_RI_IP_FRAG_TRUE,
					   ri_mask | MVPP2_PRS_RI_IP_FRAG_MASK);

		mv_pp2x_prs_tcam_data_byte_set(&pe, 2, 0x00, 0x0);
		mv_pp2x_prs_tcam_data_byte_set(&pe, 3, 0x00, 0x0);

		/* Update shadow table and hw entry */
		mv_pp2x_prs_shadow_set(inst, pe.index, MVPP2_PRS_LU_IP4);
		mv_pp2x_prs_hw_write(cpu_slot, &pe);
	}

	pp2_prs_non_log_port_update(port, &tcam_list, nri, ri_mask);

	return 0;
}


/* pp2_prs_dsa_tag_mode_set
 *
 * DESCRIPTION:	Configure parser DSA entries
 *
 * INPUTS:	port	- logical port to be configured
 *		val	- validate invalidate entry in parser
 *		tagged	- DSA tagged or not
 *		extend	- DSA extended or not
 *		ri	- results info field to configure
 *		ri_mask	- results info mask field to configure
 *
 * OUTPUTS:	None
 *
 * RETURNS:	0 on success, error-code otherwise
 */
static int pp2_prs_dsa_tag_mode_set(struct pp2_port *port, u32 val, int tagged, int extend, u32 ri, u32 ri_mask)
{
	struct mv_pp2x_prs_entry pe;
	int tid, shift;
	struct pp2_inst *inst = port->parent;
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);
	struct prs_lkp_tcam_list tcam_list;

	memset(&tcam_list, 0, sizeof(struct prs_lkp_tcam_list));

	if (extend) {
		tid = tagged ? MVPP2_PE_ETYPE_EDSA_TAGGED :
		      MVPP2_PE_ETYPE_EDSA_UNTAGGED;
		shift = 8;
	} else {
		tid = tagged ? MVPP2_PE_ETYPE_DSA_TAGGED :
		      MVPP2_PE_ETYPE_DSA_UNTAGGED;
		shift = 4;
	}

	/* get the current indexes with lookup id MVPP2_PRS_LU_DSA and specified proto */
	pp2_prs_tcam_lk_idx_list_get(inst, MVPP2_PRS_LU_DSA, 0, &tcam_list, ri);

	if (!pp2_prs_log_port_tcam_check(&tcam_list)) {
		/* Create new parser entries for the specified logical port */
		/* step 1: Not fragmented packet */
		tid = pp2_prs_tcam_first_free(inst, MVPP2_PE_FIRST_FREE_TID,
						  MVPP2_PE_LAST_FREE_TID);
		if (tid < 0)
			return tid;

		memset(&pe, 0, sizeof(struct mv_pp2x_prs_entry));
		mv_pp2x_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_DSA);
		pe.index = tid;

		/* Shift 4 bytes if DSA tag or 8 bytes in case of EDSA tag*/
		mv_pp2x_prs_sram_shift_set(&pe, shift,
					   MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);

		/* Update shadow table */
		mv_pp2x_prs_shadow_set(inst, pe.index, MVPP2_PRS_LU_DSA);

		if (tagged) {
			/* Set tagged bit in DSA tag */
			mv_pp2x_prs_tcam_data_byte_set(&pe, 0,
						       MVPP2_PRS_TCAM_DSA_TAGGED_BIT,
					MVPP2_PRS_TCAM_DSA_TAGGED_BIT);
			/* Clear all ai bits for next iteration */
			mv_pp2x_prs_sram_ai_update(&pe, 0,
						   MVPP2_PRS_SRAM_AI_MASK);
			/* If packet is tagged continue check vlans */
			mv_pp2x_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_VLAN);
		} else {
			/* Set result info bits to 'no vlans' */
			mv_pp2x_prs_sram_ri_update(&pe, MVPP2_PRS_RI_VLAN_NONE,
						   MVPP2_PRS_RI_VLAN_MASK);
			mv_pp2x_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_L2);
		}

		/* update TCAM */
		switch (val) {
		case MV_NET_TO_CPU_DSA_TAG_MODE:
			mv_pp2x_prs_tcam_data_byte_set(&pe, 0, MVPP2_PRS_TCAM_DSA_TO_CPU_MODE,
						       MVPP2_PRS_TCAM_DSA_MODE_MASK);
			break;
		case MV_NET_FROM_CPU_DSA_TAG_MODE:
			mv_pp2x_prs_tcam_data_byte_set(&pe, 0, MVPP2_PRS_TCAM_DSA_FROM_CPU_MODE,
						       MVPP2_PRS_TCAM_DSA_MODE_MASK);
			break;
		case MV_NET_TO_SNIFFER_DSA_TAG_MODE:
			mv_pp2x_prs_tcam_data_byte_set(&pe, 0, MVPP2_PRS_TCAM_DSA_TO_SNIFFER_MODE,
						       MVPP2_PRS_TCAM_DSA_MODE_MASK);
			break;
		case MV_NET_FORWARD_DSA_TAG_MODE:
			mv_pp2x_prs_tcam_data_byte_set(&pe, 0, MVPP2_PRS_TCAM_DSA_FORWARD_MODE,
						       MVPP2_PRS_TCAM_DSA_MODE_MASK);
			break;
		default:
		pr_err("parser logical port special field DSA mode: invalid tcam value\n");
			return -EFAULT;
		}

		/* update UDF7 */
		mv_pp2x_prs_sram_ri_update(&pe, ri, ri_mask);

		/* Mask all ports */
		mv_pp2x_prs_tcam_port_map_set(&pe, 0);

		/* Update port mask */
		mv_pp2x_prs_tcam_port_set(&pe, port->id, true);

		/* Update shadow table and hw entry */
		mv_pp2x_prs_shadow_set(inst, pe.index, MVPP2_PRS_LU_DSA);
		mv_pp2x_prs_hw_write(cpu_slot, &pe);
	}

	return 0;
}

/* pp2_prs_tag_mode_set
 *
 * DESCRIPTION:	Configure parser DSA entries for specific port
 *
 * INPUTS:	port	- logical port to be configured
 *		type	- MH tag mode
 *		target	- target classification (logical port or nic)
 *
 * OUTPUTS:	None
 *
 * RETURNS:	0 on success, error-code otherwise
*/
static int pp2_prs_tag_mode_set(struct pp2_port *port, int type, int val, enum pp2_ppio_cls_target target)
{
	u32 ri = 0, nri = 0;
	u32 ri_mask = 0;

	if (target == PP2_CLS_TARGET_LOCAL_PPIO) {
		ri |= MVPP2_PRS_RI_UDF7_LOG_PORT;
		nri |= MVPP2_PRS_RI_UDF7_NIC;
	} else {
		ri |= MVPP2_PRS_RI_UDF7_NIC;
		nri |= MVPP2_PRS_RI_UDF7_LOG_PORT;
	}

	ri_mask |= MVPP2_PRS_RI_UDF7_MASK;

	pr_debug("%s target %d, ri %x, nri %x, mask %x\n", __func__, target, ri, nri, ri_mask);

	switch (type) {
	case MVPP2_TAG_TYPE_EDSA:
		/* Add port to EDSA entries */
		pp2_prs_port_update(port, true, MVPP2_PE_EDSA_TAGGED, nri, ri_mask);
		pp2_prs_port_update(port, true, MVPP2_PE_EDSA_UNTAGGED, nri, ri_mask);

		/* Remove port from DSA entries */
		pp2_prs_port_update(port, false, MVPP2_PE_DSA_TAGGED, nri, ri_mask);
		pp2_prs_port_update(port, false, MVPP2_PE_DSA_UNTAGGED, nri, ri_mask);

		/* create new entries for DSA mode*/
		pp2_prs_dsa_tag_mode_set(port, val, MVPP2_PRS_TAGGED, MVPP2_PRS_EDSA, ri, ri_mask);
		pp2_prs_dsa_tag_mode_set(port, val, MVPP2_PRS_UNTAGGED, MVPP2_PRS_EDSA, ri, ri_mask);
		break;
	case MVPP2_TAG_TYPE_DSA:
		/* Add port to DSA entries */
		pp2_prs_port_update(port, true, MVPP2_PE_DSA_TAGGED, nri, ri_mask);
		pp2_prs_port_update(port, true, MVPP2_PE_DSA_UNTAGGED, nri, ri_mask);

		/* Remove port from EDSA entries */
		pp2_prs_port_update(port, false, MVPP2_PE_EDSA_TAGGED, nri, ri_mask);
		pp2_prs_port_update(port, false, MVPP2_PE_EDSA_UNTAGGED, nri, ri_mask);

		/* create new entries for DSA mode*/
		pp2_prs_dsa_tag_mode_set(port, val, MVPP2_PRS_TAGGED, MVPP2_PRS_DSA, ri, ri_mask);
		pp2_prs_dsa_tag_mode_set(port, val, MVPP2_PRS_UNTAGGED, MVPP2_PRS_DSA, ri, ri_mask);
		break;
	case MVPP2_TAG_TYPE_MH:
	case MVPP2_TAG_TYPE_NONE:
		/* Remove port form EDSA and DSA entries */
		pp2_prs_port_update(port, false, MVPP2_PE_DSA_TAGGED, nri, ri_mask);
		pp2_prs_port_update(port, false, MVPP2_PE_DSA_UNTAGGED, nri, ri_mask);
		pp2_prs_port_update(port, false, MVPP2_PE_EDSA_TAGGED, nri, ri_mask);
		pp2_prs_port_update(port, false, MVPP2_PE_EDSA_UNTAGGED, nri, ri_mask);
		break;
	default:
		if ((type < 0) || (type > MVPP2_TAG_TYPE_EDSA))
			return -EINVAL;
	}

	return 0;
}

/* mv_pp2x_prs_log_port_init
 *
 * DESCRIPTION:	Initialize parser MUSDK for logical port support
 *		Sets UDF7 bit in MH lookup to specified value
 *
 * INPUTS:	inst	- packet processor instance
 *		target	- target classification (logical port or nic)
 *
 * OUTPUTS:	None
 *
 * RETURNS:	0 on success, error-code otherwise
 */
static int mv_pp2x_prs_log_port_init(struct pp2_inst *inst, enum pp2_ppio_cls_target target)
{
	u32 i;
	struct mv_pp2x_prs_entry pe;
	struct mv_pp2x_prs_shadow *prs_shadow = inst->cls_db->prs_db.prs_shadow;
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);

	if (!prs_shadow) {
		pr_err("prs_shadow is null\n");
		return -EFAULT;
	}

	for (i = 0; i < MVPP2_PRS_TCAM_SRAM_SIZE; i++) {
		/* Clean all UDF7 bits from shadow and from HW (maybe were set in previous runs) */
		if (prs_shadow[i].valid &&
		    ((prs_shadow[i].ri & MVPP2_PRS_RI_UDF7_NIC) ||
		    (prs_shadow[i].ri & MVPP2_PRS_RI_UDF7_LOG_PORT))) {
			prs_shadow[i].ri &= ~MVPP2_PRS_RI_UDF7_NIC;
			prs_shadow[i].ri &= ~MVPP2_PRS_RI_UDF7_LOG_PORT;
			prs_shadow[i].ri_mask &= ~MVPP2_PRS_RI_UDF7_MASK;
			pe.index = i;
			mv_pp2x_prs_hw_read(cpu_slot, &pe);
			mv_pp2x_prs_sram_ri_update(&pe, MVPP2_PRS_RI_UDF7_CLEAR, MVPP2_PRS_RI_UDF7_MASK);
			mv_pp2x_prs_hw_write(cpu_slot, &pe);
		}

		/* Set default UDF7 to all MH entries according to specified target */
		if (prs_shadow[i].valid && prs_shadow[i].lu == MVPP2_PRS_LU_MH) {
			pe.index = i;
			mv_pp2x_prs_hw_read(cpu_slot, &pe);
			mv_pp2x_prs_sram_ri_update(&pe, MVPP2_PRS_RI_UDF7_CLEAR, MVPP2_PRS_RI_UDF7_MASK);
			if (target == PP2_CLS_TARGET_LOCAL_PPIO)
				mv_pp2x_prs_sram_ri_update(&pe, MVPP2_PRS_RI_UDF7_NIC, MVPP2_PRS_RI_UDF7_MASK);
			else if (target == PP2_CLS_TARGET_OTHER)
				mv_pp2x_prs_sram_ri_update(&pe, MVPP2_PRS_RI_UDF7_LOG_PORT, MVPP2_PRS_RI_UDF7_MASK);
			mv_pp2x_prs_hw_write(cpu_slot, &pe);
		}
	}
	return 0;
}

/* pp2_prs_log_port_proto_set
 *
 * DESCRIPTION:	Update parser with logical port protocol
 *
 * INPUTS:	proto -protocol to update in parser
 *		val = 0 - all traffic except defined protocol will be forwarded to logical port
 *		val = 1 - only traffic matching defined protocol will be forwarded to logical port
 *		target	- target classification (logical port or nic)
 *
 * OUTPUTS:	None
 *
 * RETURNS:	0 on success, error-code otherwise
 */
static int pp2_prs_log_port_proto_set(struct pp2_port *port, enum mv_net_proto proto, int val,
				      enum pp2_ppio_cls_target target)
{
	int err = 0;

	switch (proto) {
	case MV_NET_PROTO_VLAN:
	case MV_NET_PROTO_IP:
	case MV_NET_PROTO_IP6:
		pr_err("log port proto %d not supported yet\n", proto);
		break;
	case MV_NET_PROTO_PPPOE:
		err = pp2_prs_log_port_pppoe(port, target);
		break;
	case MV_NET_PROTO_IP4:
		err = pp2_prs_log_port_ip4_proto(port, IPPROTO_UDP,
						 MVPP2_PRS_RI_L4_UDP,
						 MVPP2_PRS_RI_L4_PROTO_MASK, target);
		if (err)
			return -EFAULT;

		err = pp2_prs_log_port_ip4_proto(port, IPPROTO_TCP,
						 MVPP2_PRS_RI_L4_TCP,
						 MVPP2_PRS_RI_L4_PROTO_MASK, target);
		if (err)
			return -EFAULT;
		break;
	case MV_NET_PROTO_UDP:
		err = pp2_prs_log_port_ip4_proto(port, IPPROTO_UDP,
						 MVPP2_PRS_RI_L4_UDP,
						 MVPP2_PRS_RI_L4_PROTO_MASK, target);
		if (err)
			return -EFAULT;
		break;
	case MV_NET_PROTO_TCP:
		err = pp2_prs_log_port_ip4_proto(port, IPPROTO_TCP,
						 MVPP2_PRS_RI_L4_TCP,
						 MVPP2_PRS_RI_L4_PROTO_MASK, target);
		if (err)
			return -EFAULT;
		break;
	default:
		pr_err("log port proto %d not supported\n", proto);
		break;
	}

	return 0;
}

/* pp2_prs_eth_start_hdr_get
 *
 * DESCRIPTION:	Get Marvell's port ethernet start header mode
 *
 * INPUTS:	port -port to be configured
 *
 * OUTPUTS:	mode - Marvell header mode
 *
 * RETURNS:	Marvell's header mode
 */
static u32 pp2_prs_eth_start_hdr_get(struct pp2_port *port)
{
	u32 reg_val;
	struct pp2_inst *inst = port->parent;
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);
	u32 ret = 0;

	reg_val = pp2_reg_read(cpu_slot, MVPP2_MH_REG(port->id));
	if (reg_val & MVPP2_DSA_NON_EXTENDED)
		ret = MVPP2_TAG_TYPE_DSA;
	else if (reg_val & MVPP2_DSA_EXTENDED)
		ret = MVPP2_TAG_TYPE_EDSA;
	else
		ret = MVPP2_TAG_TYPE_NONE;

	return ret;
}

/* pp2_prs_log_port_field_set
 *
 * DESCRIPTION:	Update parser with logical port protocol
 *
 * INPUTS:	field -special protocol field to be added to parser
 *		val - value of the special field
 *		target	- target classification (logical port or nic)
 *
 * OUTPUTS:	None
 *
 * RETURNS:	0 on success, error-code otherwise
 */
static int pp2_prs_log_port_field_set(struct pp2_port *port, struct pp2_proto_field proto_field, int val,
				      enum pp2_ppio_cls_target target)
{
	u32 type;
	int rc;

	switch (proto_field.proto) {
	case MV_NET_PROTO_ETH_DSA:
		if (proto_field.field.eth_dsa != MV_NET_ETH_F_DSA_TAG_MODE) {
			pr_err("parser logical port special protocol field not supported %d\n",
			       proto_field.field.eth_dsa);
				return -EFAULT;
		}

		if (val < MV_NET_TO_CPU_DSA_TAG_MODE || val > MV_NET_FORWARD_DSA_TAG_MODE) {
			pr_err("parser logical port special protocol field values not supported\n");
				return -EFAULT;
		}

		/*Get MH register configured mode */
		type = pp2_prs_eth_start_hdr_get(port);

		/* Configure parser DSA entries */
		rc = pp2_prs_tag_mode_set(port, type, val, target);
		if (rc)
			return -EFAULT;
		break;
	default:
		pr_err("parser logical port special protocol not supported %d\n", proto_field.proto);
		return -EFAULT;
	}
	return 0;
}

/* pp2_prs_space_check
 *
 * DESCRIPTION:	Check there is enough space in parser to configure logical port requirements
 *
 * INPUTS:	port	-port to be configured
 *		params	-pointer to parameters to be updated in parser
 *
 * OUTPUTS:	None
 *
 * RETURNS:	0 on success, error-code otherwise
 */
static int pp2_prs_space_check(struct pp2_port *port, struct pp2_ppio_log_port_params *params)
{
	/* TODO */
	return 0;
}

/* pp2_prs_set_log_port
 *
 * DESCRIPTION:	Update parser according to logical port parameters
 *
 * INPUTS:	port	-port to be configured
 *		params	-pointer to parameters to be updated in parser
 *
 * OUTPUTS:	None
 *
 * RETURNS:	0 on success, error-code otherwise
 */
int pp2_prs_set_log_port(struct pp2_port *port, struct pp2_ppio_log_port_params *params)
{
	int rc, i, j;
	struct pp2_inst *inst = port->parent;

	/* Check parameters validity*/
	if (mv_pp2x_ptr_validate(port))
		return -EINVAL;

	if (mv_pp2x_ptr_validate(params))
		return -EINVAL;

	/* Calculate required space in parser */
	rc = pp2_prs_space_check(port, params);
	if (rc) {
		pr_err("Unable to configure parser logical port: not enough space\n");
		return -EFAULT;
	}

	/* TODO - remove limitation */
	if (params->proto_based_target.num_proto_rule_sets > 1) {
		pr_err("only one rule set is supported\n");
		return -EFAULT;
	}

	/* Initialize logical port according to target:
	 * PP2_CLS_TARGET_LOCAL_PPIO	-> default traffic going to kernel and logical port to MUSDK
	 * PP2_CLS_TARGET_OTHER		-> default traffic going to MUSDK and logical port to kernel
	 */
	rc = mv_pp2x_prs_log_port_init(inst, params->proto_based_target.target);
	if (rc)
		return -EFAULT;

	/* Configure protocols and protocol fields*/
	for (i = 0; i < params->proto_based_target.num_proto_rule_sets; i++) {
		for (j = 0; j < params->proto_based_target.rule_sets[i].num_rules; j++) {
			struct pp2_ppio_log_port_rule_params *rule_params =
				&params->proto_based_target.rule_sets[i].rules[j];

			/* TODO - remove limitation */
			if (rule_params->u.proto_params.val != 0) {
				pr_err("only val 0 is supported\n");
				return -EFAULT;
			}

			pr_debug("%d:%d %d\n", i, j, rule_params->rule_type);
			if (rule_params->rule_type == PP2_RULE_TYPE_PROTO) {
				rc = pp2_prs_log_port_proto_set(port, rule_params->u.proto_params.proto,
								rule_params->u.proto_params.val,
								params->proto_based_target.target);
				if (rc)
					return -EFAULT;
			} else if (rule_params->rule_type == PP2_RULE_TYPE_PROTO_FIELD) {
				rc = pp2_prs_log_port_field_set(port, rule_params->u.proto_field_params.proto_field,
								rule_params->u.proto_field_params.val,
								params->proto_based_target.target);
				if (rc)
					return -EFAULT;
			} else {
				pr_err("Invalid rule_proto_field %d\n", rule_params->rule_type);
				return -EFAULT;
			}
		}
	}

	return 0;
}

/* pp2_prs_eth_start_hdr_set
 *
 * DESCRIPTION:	Set Marvell's port ethernet start header mode
 *
 * INPUTS:	port -port to be configured
 *		eth_start_hdr - Marvell header mode
 *
 * OUTPUTS:	None
 *
 * RETURNS:	0 on success, error-code otherwise
 */
int pp2_prs_eth_start_hdr_set(struct pp2_port *port, enum pp2_ppio_eth_start_hdr eth_start_hdr)
{
	u32 reg_val;
	struct pp2_inst *inst = port->parent;
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);

	reg_val = pp2_reg_read(cpu_slot, MVPP2_MH_REG(port->id));
	reg_val &= ~(MVPP2_DSA_EN_MASK | MVPP2_MH_EN_MASK);

	switch (eth_start_hdr) {
	case PP2_PPIO_HDR_ETH:
		break;
	case PP2_PPIO_HDR_ETH_DSA:
		reg_val |= MVPP2_DSA_NON_EXTENDED;
		break;
	case PP2_PPIO_HDR_ETH_EXT_DSA:
		reg_val |= MVPP2_DSA_EXTENDED;
		break;
	default:
		pr_err("invalid eth_start_hdr, eth_start_hdr = %d\n", eth_start_hdr);
		return -EINVAL;
	}

	/* Write to register */
	pp2_reg_write(cpu_slot, MVPP2_MH_REG(port->id), reg_val);

	return 0;
}

/* mv_pp2x_prs_shadow_update
 *
 * DESCRIPTION:	Update MUSDK parser shadow db
 *
 * INPUTS:	inst - packet processor instance
 *
 * OUTPUTS:	None
 *
 * RETURNS:	0 on success, error-code otherwise
 */
static int mv_pp2x_prs_shadow_update(struct pp2_inst *inst)
{
	int i, j, invalid;
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
		for (j = 0; j < MVPP2_PRS_TCAM_WORDS; j++)
			prs_shadow[i].tcam.word[j] = pe.tcam.word[j];
		invalid = mv_pp2x_prs_tcam_valid_get(&pe);
		prs_shadow[i].valid = invalid ? 0 : 1;
		prs_shadow[i].valid_in_kernel = invalid ? 0 : 1;
	}
	return 0;
}

/* pp2_prs_deinit
 *
 * DESCRIPTION:	De-initialize parser for MUSDK.
 *		Returns parser to original configuration
 *
 * INPUTS:	inst - packet processor instance
 *
 * OUTPUTS:	None
 *
 * RETURNS:	0 on success, error-code otherwise
 */
void pp2_cls_prs_deinit(struct pp2_inst *inst)
{
	u32 i;
	struct mv_pp2x_prs_entry pe;
	struct mv_pp2x_prs_shadow *prs_shadow = inst->cls_db->prs_db.prs_shadow;
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);

	/* Invalidate all MUSDK added entries */
	for (i = 0; i < MVPP2_PRS_TCAM_SRAM_SIZE; i++) {
		if (prs_shadow[i].valid_in_kernel) {
			/* Return parser to initial kernel configuration, except MH since
			 * all flows are changed to look at UDF7 bit
			 */
			pe.index = i;
			mv_pp2x_prs_hw_read(cpu_slot, &pe);
			pe.sram.word[MVPP2_PRS_SRAM_RI_WORD] = prs_shadow[i].ri;
			pe.sram.word[MVPP2_PRS_SRAM_RI_CTRL_WORD] = prs_shadow[i].ri_mask;
			pe.tcam.byte[HW_BYTE_OFFS(MVPP2_PRS_TCAM_EN_OFFS(MVPP2_PRS_TCAM_PORT_BYTE))] =
				prs_shadow[i].tcam.byte[HW_BYTE_OFFS(MVPP2_PRS_TCAM_EN_OFFS(MVPP2_PRS_TCAM_PORT_BYTE))];

			pr_debug("%d ri %x, mask %x port %x\n", i, pe.sram.word[MVPP2_PRS_SRAM_RI_WORD],
				pe.sram.word[MVPP2_PRS_SRAM_RI_CTRL_WORD],
				pe.tcam.byte[HW_BYTE_OFFS(MVPP2_PRS_TCAM_EN_OFFS(MVPP2_PRS_TCAM_PORT_BYTE))]);

			if (prs_shadow[i].lu == MVPP2_PRS_LU_MH) {
				mv_pp2x_prs_sram_ri_update(&pe, MVPP2_PRS_RI_UDF7_CLEAR, MVPP2_PRS_RI_UDF7_MASK);
				mv_pp2x_prs_sram_ri_update(&pe, MVPP2_PRS_RI_UDF7_NIC, MVPP2_PRS_RI_UDF7_MASK);
			}
			mv_pp2x_prs_hw_write(cpu_slot, &pe);
			continue;
		}

		if (prs_shadow[i].valid) {
			pr_debug("parser: removing idx %d\n", i);
			mv_pp2x_prs_hw_inv(cpu_slot, i);
		}
	}
}

/* pp2_cls_prs_init
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
 */
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

	/* Initialize parser for logical port support */
	rc = mv_pp2x_prs_log_port_init(inst, PP2_CLS_TARGET_LOCAL_PPIO);
	if (rc)
		return -EINVAL;

	/* MUSDK local parser flow id attribute tbl init (used in classifier)
	 * TODO  this table needs to be synchronized with kernel
	 * TODO  this table will be dynamic once struct pp2_parse_params is implemented
	 */
	mv_pp2x_prs_flow_id_attr_init();

	return 0;
}

