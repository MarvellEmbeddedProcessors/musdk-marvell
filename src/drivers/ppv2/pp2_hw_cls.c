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

#include "pp2_types.h"

#include "pp2.h"
#include "pp2_hw_cls.h"
#include "pp2_hw_type.h"

/* TODO: Keep these until classifier phase */
#pragma GCC diagnostic ignored "-Wmissing-prototypes"

/*	C3 declarations	*/
struct pp2_cls_c3_shadow_hash_entry pp2_cls_c3_shadow_tbl[MVPP2_CLS_C3_HASH_TBL_SIZE];
int pp2_cls_c3_shadow_ext_tbl[MVPP2_CLS_C3_EXT_TBL_SIZE];
static int sw_init_cnt_set;

/* Parser configuration routines */

/* Flow ID definition array */
static struct mv_pp2x_prs_flow_id
mv_pp2x_prs_flow_id_array[MVPP2_PRS_FL_TCAM_NUM] = {
	/***********#Flow ID#**************#Result Info#************/
	{ MVPP2_PRS_FL_IP4_TCP_NF_UNTAG, { MVPP2_PRS_RI_VLAN_NONE |
					  MVPP2_PRS_RI_L3_IP4 |
					  MVPP2_PRS_RI_IP_FRAG_FALSE |
					  MVPP2_PRS_RI_L4_TCP,
					  MVPP2_PRS_RI_VLAN_MASK |
					  MVPP2_PRS_RI_L3_PROTO_MASK |
					  MVPP2_PRS_RI_IP_FRAG_MASK |
					  MVPP2_PRS_RI_L4_PROTO_MASK} },

	{ MVPP2_PRS_FL_IP4_TCP_NF_UNTAG, { MVPP2_PRS_RI_VLAN_NONE |
					  MVPP2_PRS_RI_L3_IP4_OPT |
					  MVPP2_PRS_RI_IP_FRAG_FALSE |
					  MVPP2_PRS_RI_L4_TCP,
					  MVPP2_PRS_RI_VLAN_MASK |
					  MVPP2_PRS_RI_L3_PROTO_MASK |
					  MVPP2_PRS_RI_IP_FRAG_MASK |
					  MVPP2_PRS_RI_L4_PROTO_MASK} },

	{ MVPP2_PRS_FL_IP4_TCP_NF_UNTAG, { MVPP2_PRS_RI_VLAN_NONE |
					  MVPP2_PRS_RI_L3_IP4_OTHER |
					  MVPP2_PRS_RI_IP_FRAG_FALSE |
					  MVPP2_PRS_RI_L4_TCP,
					  MVPP2_PRS_RI_VLAN_MASK |
					  MVPP2_PRS_RI_L3_PROTO_MASK |
					  MVPP2_PRS_RI_IP_FRAG_MASK |
					  MVPP2_PRS_RI_L4_PROTO_MASK} },

	{ MVPP2_PRS_FL_IP4_UDP_NF_UNTAG, { MVPP2_PRS_RI_VLAN_NONE |
					  MVPP2_PRS_RI_L3_IP4 |
					  MVPP2_PRS_RI_IP_FRAG_FALSE |
					  MVPP2_PRS_RI_L4_UDP,
					  MVPP2_PRS_RI_VLAN_MASK |
					  MVPP2_PRS_RI_L3_PROTO_MASK |
					  MVPP2_PRS_RI_IP_FRAG_MASK |
					  MVPP2_PRS_RI_L4_PROTO_MASK} },

	{ MVPP2_PRS_FL_IP4_UDP_NF_UNTAG, { MVPP2_PRS_RI_VLAN_NONE |
					  MVPP2_PRS_RI_L3_IP4_OPT |
					  MVPP2_PRS_RI_IP_FRAG_FALSE |
					  MVPP2_PRS_RI_L4_UDP,
					  MVPP2_PRS_RI_VLAN_MASK |
					  MVPP2_PRS_RI_L3_PROTO_MASK |
					  MVPP2_PRS_RI_IP_FRAG_MASK |
					  MVPP2_PRS_RI_L4_PROTO_MASK} },

	{ MVPP2_PRS_FL_IP4_UDP_NF_UNTAG, { MVPP2_PRS_RI_VLAN_NONE |
					  MVPP2_PRS_RI_L3_IP4_OTHER |
					  MVPP2_PRS_RI_IP_FRAG_FALSE |
					  MVPP2_PRS_RI_L4_UDP,
					  MVPP2_PRS_RI_VLAN_MASK |
					  MVPP2_PRS_RI_L3_PROTO_MASK |
					  MVPP2_PRS_RI_IP_FRAG_MASK |
					  MVPP2_PRS_RI_L4_PROTO_MASK} },

	{ MVPP2_PRS_FL_IP4_TCP_NF_TAG, { MVPP2_PRS_RI_L3_IP4 |
					MVPP2_PRS_RI_IP_FRAG_FALSE |
					MVPP2_PRS_RI_L4_TCP,
					MVPP2_PRS_RI_L3_PROTO_MASK |
					MVPP2_PRS_RI_IP_FRAG_MASK |
					MVPP2_PRS_RI_L4_PROTO_MASK} },

	{ MVPP2_PRS_FL_IP4_TCP_NF_TAG, { MVPP2_PRS_RI_L3_IP4_OPT |
					MVPP2_PRS_RI_IP_FRAG_FALSE |
					MVPP2_PRS_RI_L4_TCP,
					MVPP2_PRS_RI_L3_PROTO_MASK |
					MVPP2_PRS_RI_IP_FRAG_MASK |
					MVPP2_PRS_RI_L4_PROTO_MASK} },

	{ MVPP2_PRS_FL_IP4_TCP_NF_TAG, { MVPP2_PRS_RI_L3_IP4_OTHER |
					MVPP2_PRS_RI_IP_FRAG_FALSE |
					MVPP2_PRS_RI_L4_TCP,
					MVPP2_PRS_RI_L3_PROTO_MASK |
					MVPP2_PRS_RI_IP_FRAG_MASK |
					MVPP2_PRS_RI_L4_PROTO_MASK} },

	{ MVPP2_PRS_FL_IP4_UDP_NF_TAG, { MVPP2_PRS_RI_L3_IP4 |
					MVPP2_PRS_RI_IP_FRAG_FALSE |
					MVPP2_PRS_RI_L4_UDP,
					MVPP2_PRS_RI_L3_PROTO_MASK |
					MVPP2_PRS_RI_IP_FRAG_MASK |
					MVPP2_PRS_RI_L4_PROTO_MASK} },

	{ MVPP2_PRS_FL_IP4_UDP_NF_TAG, { MVPP2_PRS_RI_L3_IP4_OPT |
					MVPP2_PRS_RI_IP_FRAG_FALSE |
					MVPP2_PRS_RI_L4_UDP,
					MVPP2_PRS_RI_L3_PROTO_MASK |
					MVPP2_PRS_RI_IP_FRAG_MASK |
					MVPP2_PRS_RI_L4_PROTO_MASK} },

	{ MVPP2_PRS_FL_IP4_UDP_NF_TAG, { MVPP2_PRS_RI_L3_IP4_OTHER |
					MVPP2_PRS_RI_IP_FRAG_FALSE |
					MVPP2_PRS_RI_L4_UDP,
					MVPP2_PRS_RI_L3_PROTO_MASK |
					MVPP2_PRS_RI_IP_FRAG_MASK |
					MVPP2_PRS_RI_L4_PROTO_MASK} },

	{ MVPP2_PRS_FL_IP6_TCP_NF_UNTAG, { MVPP2_PRS_RI_VLAN_NONE |
					  MVPP2_PRS_RI_L3_IP6 |
					  MVPP2_PRS_RI_IP_FRAG_FALSE |
					  MVPP2_PRS_RI_L4_TCP,
					  MVPP2_PRS_RI_VLAN_MASK |
					  MVPP2_PRS_RI_L3_PROTO_MASK |
					  MVPP2_PRS_RI_IP_FRAG_MASK |
					  MVPP2_PRS_RI_L4_PROTO_MASK} },

	{ MVPP2_PRS_FL_IP6_TCP_NF_UNTAG, { MVPP2_PRS_RI_VLAN_NONE |
					  MVPP2_PRS_RI_L3_IP6_EXT |
					  MVPP2_PRS_RI_IP_FRAG_FALSE |
					  MVPP2_PRS_RI_L4_TCP,
					  MVPP2_PRS_RI_VLAN_MASK |
					  MVPP2_PRS_RI_L3_PROTO_MASK |
					  MVPP2_PRS_RI_IP_FRAG_MASK |
					  MVPP2_PRS_RI_L4_PROTO_MASK} },

	{ MVPP2_PRS_FL_IP6_UDP_NF_UNTAG, { MVPP2_PRS_RI_VLAN_NONE |
					  MVPP2_PRS_RI_L3_IP6 |
					  MVPP2_PRS_RI_IP_FRAG_FALSE |
					  MVPP2_PRS_RI_L4_UDP,
					  MVPP2_PRS_RI_VLAN_MASK |
					  MVPP2_PRS_RI_L3_PROTO_MASK |
					  MVPP2_PRS_RI_IP_FRAG_MASK |
					  MVPP2_PRS_RI_L4_PROTO_MASK} },

	{ MVPP2_PRS_FL_IP6_UDP_NF_UNTAG, { MVPP2_PRS_RI_VLAN_NONE |
					  MVPP2_PRS_RI_L3_IP6_EXT |
					  MVPP2_PRS_RI_IP_FRAG_FALSE |
					  MVPP2_PRS_RI_L4_UDP,
					  MVPP2_PRS_RI_VLAN_MASK |
					  MVPP2_PRS_RI_L3_PROTO_MASK |
					  MVPP2_PRS_RI_IP_FRAG_MASK |
					  MVPP2_PRS_RI_L4_PROTO_MASK} },

	{ MVPP2_PRS_FL_IP6_TCP_NF_TAG, { MVPP2_PRS_RI_L3_IP6 |
					MVPP2_PRS_RI_IP_FRAG_FALSE |
					MVPP2_PRS_RI_L4_TCP,
					MVPP2_PRS_RI_L3_PROTO_MASK |
					MVPP2_PRS_RI_IP_FRAG_MASK |
					MVPP2_PRS_RI_L4_PROTO_MASK} },

	{ MVPP2_PRS_FL_IP6_TCP_NF_TAG, { MVPP2_PRS_RI_L3_IP6_EXT |
					MVPP2_PRS_RI_IP_FRAG_FALSE |
					MVPP2_PRS_RI_L4_TCP,
					MVPP2_PRS_RI_L3_PROTO_MASK |
					MVPP2_PRS_RI_IP_FRAG_MASK |
					MVPP2_PRS_RI_L4_PROTO_MASK} },

	{ MVPP2_PRS_FL_IP6_UDP_NF_TAG, { MVPP2_PRS_RI_L3_IP6 |
					MVPP2_PRS_RI_IP_FRAG_FALSE |
					MVPP2_PRS_RI_L4_UDP,
					MVPP2_PRS_RI_L3_PROTO_MASK |
					MVPP2_PRS_RI_IP_FRAG_MASK |
					MVPP2_PRS_RI_L4_PROTO_MASK} },

	{ MVPP2_PRS_FL_IP6_UDP_NF_TAG, { MVPP2_PRS_RI_L3_IP6_EXT |
					MVPP2_PRS_RI_IP_FRAG_FALSE |
					MVPP2_PRS_RI_L4_UDP,
					MVPP2_PRS_RI_L3_PROTO_MASK |
					MVPP2_PRS_RI_IP_FRAG_MASK |
					MVPP2_PRS_RI_L4_PROTO_MASK} },

	{ MVPP2_PRS_FL_IP4_TCP_FRAG_UNTAG, { MVPP2_PRS_RI_VLAN_NONE |
					    MVPP2_PRS_RI_L3_IP4 |
					    MVPP2_PRS_RI_IP_FRAG_TRUE |
					    MVPP2_PRS_RI_L4_TCP,
					    MVPP2_PRS_RI_VLAN_MASK |
					    MVPP2_PRS_RI_L3_PROTO_MASK |
					    MVPP2_PRS_RI_IP_FRAG_MASK |
					    MVPP2_PRS_RI_L4_PROTO_MASK} },

	{ MVPP2_PRS_FL_IP4_TCP_FRAG_UNTAG, { MVPP2_PRS_RI_VLAN_NONE |
					    MVPP2_PRS_RI_L3_IP4_OPT |
					    MVPP2_PRS_RI_IP_FRAG_TRUE |
					    MVPP2_PRS_RI_L4_TCP,
					    MVPP2_PRS_RI_VLAN_MASK |
					    MVPP2_PRS_RI_L3_PROTO_MASK |
					    MVPP2_PRS_RI_IP_FRAG_MASK |
					    MVPP2_PRS_RI_L4_PROTO_MASK} },

	{ MVPP2_PRS_FL_IP4_TCP_FRAG_UNTAG, { MVPP2_PRS_RI_VLAN_NONE |
					    MVPP2_PRS_RI_L3_IP4_OTHER |
					    MVPP2_PRS_RI_IP_FRAG_TRUE |
					    MVPP2_PRS_RI_L4_TCP,
					    MVPP2_PRS_RI_VLAN_MASK |
					    MVPP2_PRS_RI_L3_PROTO_MASK |
					    MVPP2_PRS_RI_IP_FRAG_MASK |
					    MVPP2_PRS_RI_L4_PROTO_MASK} },

	{ MVPP2_PRS_FL_IP4_UDP_FRAG_UNTAG, { MVPP2_PRS_RI_VLAN_NONE |
					    MVPP2_PRS_RI_L3_IP4 |
					    MVPP2_PRS_RI_IP_FRAG_TRUE |
					    MVPP2_PRS_RI_L4_UDP,
					    MVPP2_PRS_RI_VLAN_MASK |
					    MVPP2_PRS_RI_L3_PROTO_MASK |
					    MVPP2_PRS_RI_IP_FRAG_MASK |
					    MVPP2_PRS_RI_L4_PROTO_MASK} },

	{ MVPP2_PRS_FL_IP4_UDP_FRAG_UNTAG, { MVPP2_PRS_RI_VLAN_NONE |
					    MVPP2_PRS_RI_L3_IP4_OPT |
					    MVPP2_PRS_RI_IP_FRAG_TRUE |
					    MVPP2_PRS_RI_L4_UDP,
					    MVPP2_PRS_RI_VLAN_MASK |
					    MVPP2_PRS_RI_L3_PROTO_MASK |
					    MVPP2_PRS_RI_IP_FRAG_MASK |
					    MVPP2_PRS_RI_L4_PROTO_MASK} },

	{ MVPP2_PRS_FL_IP4_UDP_FRAG_UNTAG, { MVPP2_PRS_RI_VLAN_NONE |
					    MVPP2_PRS_RI_L3_IP4_OTHER |
					    MVPP2_PRS_RI_IP_FRAG_TRUE |
					    MVPP2_PRS_RI_L4_UDP,
					    MVPP2_PRS_RI_VLAN_MASK |
					    MVPP2_PRS_RI_L3_PROTO_MASK |
					    MVPP2_PRS_RI_IP_FRAG_MASK |
					    MVPP2_PRS_RI_L4_PROTO_MASK} },

	{ MVPP2_PRS_FL_IP4_TCP_FRAG_TAG, { MVPP2_PRS_RI_L3_IP4 |
					  MVPP2_PRS_RI_IP_FRAG_TRUE |
					  MVPP2_PRS_RI_L4_TCP,
					  MVPP2_PRS_RI_L3_PROTO_MASK |
					  MVPP2_PRS_RI_IP_FRAG_MASK |
					  MVPP2_PRS_RI_L4_PROTO_MASK} },

	{ MVPP2_PRS_FL_IP4_TCP_FRAG_TAG, { MVPP2_PRS_RI_L3_IP4_OPT |
					  MVPP2_PRS_RI_IP_FRAG_TRUE |
					  MVPP2_PRS_RI_L4_TCP,
					  MVPP2_PRS_RI_L3_PROTO_MASK |
					  MVPP2_PRS_RI_IP_FRAG_MASK |
					  MVPP2_PRS_RI_L4_PROTO_MASK} },

	{ MVPP2_PRS_FL_IP4_TCP_FRAG_TAG, { MVPP2_PRS_RI_L3_IP4_OTHER |
					  MVPP2_PRS_RI_IP_FRAG_TRUE |
					  MVPP2_PRS_RI_L4_TCP,
					  MVPP2_PRS_RI_L3_PROTO_MASK |
					  MVPP2_PRS_RI_IP_FRAG_MASK |
					  MVPP2_PRS_RI_L4_PROTO_MASK} },

	{ MVPP2_PRS_FL_IP4_UDP_FRAG_TAG, { MVPP2_PRS_RI_L3_IP4 |
					  MVPP2_PRS_RI_IP_FRAG_TRUE |
					  MVPP2_PRS_RI_L4_UDP,
					  MVPP2_PRS_RI_L3_PROTO_MASK |
					  MVPP2_PRS_RI_IP_FRAG_MASK |
					  MVPP2_PRS_RI_L4_PROTO_MASK} },

	{ MVPP2_PRS_FL_IP4_UDP_FRAG_TAG, { MVPP2_PRS_RI_L3_IP4_OPT |
					  MVPP2_PRS_RI_IP_FRAG_TRUE |
					  MVPP2_PRS_RI_L4_UDP,
					  MVPP2_PRS_RI_L3_PROTO_MASK |
					  MVPP2_PRS_RI_IP_FRAG_MASK |
					  MVPP2_PRS_RI_L4_PROTO_MASK} },

	{ MVPP2_PRS_FL_IP4_UDP_FRAG_TAG, { MVPP2_PRS_RI_L3_IP4_OTHER |
					  MVPP2_PRS_RI_IP_FRAG_TRUE |
					  MVPP2_PRS_RI_L4_UDP,
					  MVPP2_PRS_RI_L3_PROTO_MASK |
					  MVPP2_PRS_RI_IP_FRAG_MASK |
					  MVPP2_PRS_RI_L4_PROTO_MASK} },

	{ MVPP2_PRS_FL_IP6_TCP_FRAG_UNTAG, { MVPP2_PRS_RI_VLAN_NONE |
					    MVPP2_PRS_RI_L3_IP6 |
					    MVPP2_PRS_RI_IP_FRAG_TRUE |
					    MVPP2_PRS_RI_L4_TCP,
					    MVPP2_PRS_RI_VLAN_MASK |
					    MVPP2_PRS_RI_L3_PROTO_MASK |
					    MVPP2_PRS_RI_IP_FRAG_MASK |
					    MVPP2_PRS_RI_L4_PROTO_MASK} },

	{ MVPP2_PRS_FL_IP6_TCP_FRAG_UNTAG, { MVPP2_PRS_RI_VLAN_NONE |
					    MVPP2_PRS_RI_L3_IP6_EXT |
					    MVPP2_PRS_RI_IP_FRAG_TRUE |
					    MVPP2_PRS_RI_L4_TCP,
					    MVPP2_PRS_RI_VLAN_MASK |
					    MVPP2_PRS_RI_L3_PROTO_MASK |
					    MVPP2_PRS_RI_IP_FRAG_MASK |
					    MVPP2_PRS_RI_L4_PROTO_MASK} },

	{ MVPP2_PRS_FL_IP6_UDP_FRAG_UNTAG, { MVPP2_PRS_RI_VLAN_NONE |
					    MVPP2_PRS_RI_L3_IP6 |
					    MVPP2_PRS_RI_IP_FRAG_TRUE |
					    MVPP2_PRS_RI_L4_UDP,
					    MVPP2_PRS_RI_VLAN_MASK |
					    MVPP2_PRS_RI_L3_PROTO_MASK |
					    MVPP2_PRS_RI_IP_FRAG_MASK |
					    MVPP2_PRS_RI_L4_PROTO_MASK} },

	{ MVPP2_PRS_FL_IP6_UDP_FRAG_UNTAG, { MVPP2_PRS_RI_VLAN_NONE |
					    MVPP2_PRS_RI_L3_IP6_EXT |
					    MVPP2_PRS_RI_IP_FRAG_TRUE |
					    MVPP2_PRS_RI_L4_UDP,
					    MVPP2_PRS_RI_VLAN_MASK |
					    MVPP2_PRS_RI_L3_PROTO_MASK |
					    MVPP2_PRS_RI_IP_FRAG_MASK |
					    MVPP2_PRS_RI_L4_PROTO_MASK} },

	{ MVPP2_PRS_FL_IP6_TCP_FRAG_TAG, { MVPP2_PRS_RI_L3_IP6 |
					  MVPP2_PRS_RI_IP_FRAG_TRUE |
					  MVPP2_PRS_RI_L4_TCP,
					  MVPP2_PRS_RI_L3_PROTO_MASK |
					  MVPP2_PRS_RI_IP_FRAG_MASK |
					  MVPP2_PRS_RI_L4_PROTO_MASK} },

	{ MVPP2_PRS_FL_IP6_TCP_FRAG_TAG, { MVPP2_PRS_RI_L3_IP6_EXT |
					  MVPP2_PRS_RI_IP_FRAG_TRUE |
					  MVPP2_PRS_RI_L4_TCP,
					  MVPP2_PRS_RI_L3_PROTO_MASK |
					  MVPP2_PRS_RI_IP_FRAG_MASK |
					  MVPP2_PRS_RI_L4_PROTO_MASK} },

	{ MVPP2_PRS_FL_IP6_UDP_FRAG_TAG, { MVPP2_PRS_RI_L3_IP6 |
					  MVPP2_PRS_RI_IP_FRAG_TRUE |
					  MVPP2_PRS_RI_L4_UDP,
					  MVPP2_PRS_RI_L3_PROTO_MASK |
					  MVPP2_PRS_RI_IP_FRAG_MASK |
					  MVPP2_PRS_RI_L4_PROTO_MASK} },

	{ MVPP2_PRS_FL_IP6_UDP_FRAG_TAG, { MVPP2_PRS_RI_L3_IP6_EXT |
					  MVPP2_PRS_RI_IP_FRAG_TRUE |
					  MVPP2_PRS_RI_L4_UDP,
					  MVPP2_PRS_RI_L3_PROTO_MASK |
					  MVPP2_PRS_RI_IP_FRAG_MASK |
					  MVPP2_PRS_RI_L4_PROTO_MASK} },

	{ MVPP2_PRS_FL_IP4_UNTAG, { MVPP2_PRS_RI_VLAN_NONE |
				   MVPP2_PRS_RI_L3_IP4,
				   MVPP2_PRS_RI_VLAN_MASK |
				   MVPP2_PRS_RI_L3_PROTO_MASK} },

	{ MVPP2_PRS_FL_IP4_UNTAG, { MVPP2_PRS_RI_VLAN_NONE |
				   MVPP2_PRS_RI_L3_IP4_OPT,
				   MVPP2_PRS_RI_VLAN_MASK |
				   MVPP2_PRS_RI_L3_PROTO_MASK} },

	{ MVPP2_PRS_FL_IP4_UNTAG, { MVPP2_PRS_RI_VLAN_NONE |
				   MVPP2_PRS_RI_L3_IP4_OTHER,
				   MVPP2_PRS_RI_VLAN_MASK |
				   MVPP2_PRS_RI_L3_PROTO_MASK} },

	{ MVPP2_PRS_FL_IP4_TAG, { MVPP2_PRS_RI_L3_IP4,
				 MVPP2_PRS_RI_L3_PROTO_MASK} },

	{ MVPP2_PRS_FL_IP4_TAG, { MVPP2_PRS_RI_L3_IP4_OPT,
				 MVPP2_PRS_RI_L3_PROTO_MASK} },

	{ MVPP2_PRS_FL_IP4_TAG, { MVPP2_PRS_RI_L3_IP4_OTHER,
				 MVPP2_PRS_RI_L3_PROTO_MASK} },

	{ MVPP2_PRS_FL_IP6_UNTAG, { MVPP2_PRS_RI_VLAN_NONE |
				   MVPP2_PRS_RI_L3_IP6,
				   MVPP2_PRS_RI_VLAN_MASK |
				   MVPP2_PRS_RI_L3_PROTO_MASK} },

	{ MVPP2_PRS_FL_IP6_UNTAG, { MVPP2_PRS_RI_VLAN_NONE |
				   MVPP2_PRS_RI_L3_IP6_EXT,
				   MVPP2_PRS_RI_VLAN_MASK |
				   MVPP2_PRS_RI_L3_PROTO_MASK} },

	{ MVPP2_PRS_FL_IP6_TAG, { MVPP2_PRS_RI_L3_IP6,
				 MVPP2_PRS_RI_L3_PROTO_MASK} },

	{ MVPP2_PRS_FL_IP6_TAG, { MVPP2_PRS_RI_L3_IP6_EXT,
				 MVPP2_PRS_RI_L3_PROTO_MASK} },

	{ MVPP2_PRS_FL_NON_IP_UNTAG, { MVPP2_PRS_RI_VLAN_NONE,
				      MVPP2_PRS_RI_VLAN_MASK} },

	{ MVPP2_PRS_FL_NON_IP_TAG, { 0, 0} },
};

/* Array of bitmask to indicate flow id attribute */
static int mv_pp2x_prs_flow_id_attr_tbl[MVPP2_PRS_FL_LAST];

int mv_pp2x_ptr_validate(const void *ptr)
{
	if (!ptr) {
		pr_err("%s: null pointer.\n", __func__);
		return MV_ERROR;
	}
	return 0;
}

int mv_pp2x_range_validate(int value, int min, int max)
{
	if (((value) > (max)) || ((value) < (min))) {
		pr_err("%s: value 0x%X (%d) is out of range [0x%X , 0x%X].\n",
			__func__, (value), (value), (min), (max));
		return MV_ERROR;
	}
	return 0;
}

/********************************************************************/
/***************** Classifier Top Public lkpid table APIs ********************/
/********************************************************************/

/*------------------------------------------------------------------*/

int mv_pp2x_cls_hw_lkp_read(uintptr_t cpu_slot, int lkpid, int way,
			    struct mv_pp2x_cls_lookup_entry *fe)
{
	unsigned int reg_val = 0;

	if (mv_pp2x_ptr_validate(fe) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_range_validate(way, 0, WAY_MAX) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_range_validate(lkpid, 0,
				   MVPP2_CLS_FLOWS_TBL_SIZE) == MV_ERROR)
		return MV_ERROR;

	/* write index reg */
	reg_val = (way << MVPP2_CLS_LKP_INDEX_WAY_OFFS) |
	    (lkpid << MVPP2_CLS_LKP_INDEX_LKP_OFFS);
	pp2_reg_write(cpu_slot, MVPP2_CLS_LKP_INDEX_REG, reg_val);

	fe->way = way;
	fe->lkpid = lkpid;

	fe->data = pp2_reg_read(cpu_slot, MVPP2_CLS_LKP_TBL_REG);

	return 0;
}

int mv_pp2x_cls_hw_lkp_write(uintptr_t cpu_slot, struct mv_pp2x_cls_lookup_entry *fe)
{
	u32 reg_val = 0;

	if (mv_pp2x_ptr_validate(fe) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_range_validate(fe->way, 0, 1) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_range_validate(fe->lkpid, 0,
				   MVPP2_CLS_FLOWS_TBL_SIZE) == MV_ERROR)
		return MV_ERROR;

	/* write index reg */
	reg_val = (fe->way << MVPP2_CLS_LKP_INDEX_WAY_OFFS) |
	    (fe->lkpid << MVPP2_CLS_LKP_INDEX_LKP_OFFS);
	pp2_reg_write(cpu_slot, MVPP2_CLS_LKP_INDEX_REG, reg_val);

	/* write flow_id reg */
	pp2_reg_write(cpu_slot, MVPP2_CLS_LKP_TBL_REG, fe->data);

	return 0;
}

void mv_pp2x_cls_sw_lkp_clear(struct mv_pp2x_cls_lookup_entry *fe)
{
	memset(fe, 0, sizeof(struct mv_pp2x_cls_lookup_entry));
}

int mv_pp2x_cls_hw_lkp_clear(uintptr_t cpu_slot, int lkpid, int way)
{
	struct mv_pp2x_cls_lookup_entry fe;

	if (mv_pp2x_range_validate(lkpid, 0,
				   MVPP2_CLS_FLOWS_TBL_SIZE) == MV_ERROR)
		return -EINVAL;
	if (mv_pp2x_range_validate(way, 0, 1) == MV_ERROR)
		return -EINVAL;

	/* clear entry */
	mv_pp2x_cls_sw_lkp_clear(&fe);
	fe.lkpid = lkpid;
	fe.way = way;
	mv_pp2x_cls_hw_lkp_write(cpu_slot, &fe);

	return 0;
}

int mv_pp2x_cls_hw_lkp_clear_all(uintptr_t cpu_slot)
{
	int lkpid;

	for (lkpid = 0; lkpid < MVPP2_CLS_LKP_TBL_SIZE; lkpid++) {
		if (mv_pp2x_cls_hw_lkp_clear(cpu_slot, lkpid, 0))
			return -EINVAL;
		if (mv_pp2x_cls_hw_lkp_clear(cpu_slot, lkpid, 1))
			return -EINVAL;
	}
	return 0;
}

int mv_pp2x_cls_hw_cls_enable(uintptr_t cpu_slot, uint32_t en)
{
	if (mv_pp2x_range_validate(en, 0, 1) == MV_ERROR)
		return -EINVAL;

	/* Enable classifier */
	pp2_reg_write(cpu_slot, MVPP2_CLS_MODE_REG, en);

	return 0;
}

/*----------------------------------------------------------------------*/

int mv_pp2x_cls_sw_lkp_rxq_get(struct mv_pp2x_cls_lookup_entry *lkp, int *rxq)
{
	if (mv_pp2x_ptr_validate(lkp) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_ptr_validate(rxq) == MV_ERROR)
		return MV_ERROR;

	*rxq = (lkp->data & MVPP2_FLOWID_RXQ_MASK) >> MVPP2_FLOWID_RXQ;
	return 0;
}

int mv_pp2x_cls_sw_lkp_rxq_set(struct mv_pp2x_cls_lookup_entry *lkp, int rxq)
{
	if (mv_pp2x_ptr_validate(lkp) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_range_validate(rxq, 0,
				   (1 << MVPP2_FLOWID_RXQ_BITS) - 1) ==
	    MV_ERROR)
		return MV_ERROR;

	lkp->data &= ~MVPP2_FLOWID_RXQ_MASK;
	lkp->data |= (rxq << MVPP2_FLOWID_RXQ);

	return 0;
}

int mv_pp2x_cls_sw_lkp_mod_get(struct mv_pp2x_cls_lookup_entry *le, int *mod_base)
{
	if (mv_pp2x_ptr_validate(le) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_ptr_validate(mod_base) == MV_ERROR)
		return MV_ERROR;

	*mod_base = (le->data & MVPP2_FLOWID_MODE_MASK) >> MVPP2_FLOWID_MODE;

	return 0;
}

int mv_pp2x_cls_sw_lkp_flow_get(struct mv_pp2x_cls_lookup_entry *le, int *flow_idx)
{
	if (mv_pp2x_ptr_validate(le) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_ptr_validate(flow_idx) == MV_ERROR)
		return MV_ERROR;

	*flow_idx = (le->data & MVPP2_FLOWID_FLOW_MASK) >> MVPP2_FLOWID_FLOW;

	return 0;
}

int mv_pp2x_cls_sw_lkp_flow_set(struct mv_pp2x_cls_lookup_entry *lkp,
				int flow_idx)
{
	if (mv_pp2x_ptr_validate(lkp) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_range_validate(flow_idx, 0,
				   MVPP2_CLS_FLOWS_TBL_SIZE) == MV_ERROR)
		return MV_ERROR;

	lkp->data &= ~MVPP2_FLOWID_FLOW_MASK;
	lkp->data |= (flow_idx << MVPP2_FLOWID_FLOW);

	return 0;
}

int mv_pp2x_cls_sw_lkp_en_get(struct mv_pp2x_cls_lookup_entry *le, int *en)
{
	if (mv_pp2x_ptr_validate(le) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_ptr_validate(en) == MV_ERROR)
		return MV_ERROR;

	*en = (le->data & MVPP2_FLOWID_EN_MASK) >> MVPP2_FLOWID_EN;

	return 0;
}

int mv_pp2x_cls_sw_lkp_en_set(struct mv_pp2x_cls_lookup_entry *lkp, int en)
{
	if (mv_pp2x_ptr_validate(lkp) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_range_validate(en, 0, 1) == MV_ERROR)
		return MV_ERROR;

	lkp->data &= ~MVPP2_FLOWID_EN_MASK;
	lkp->data |= (en << MVPP2_FLOWID_EN);

	return 0;
}

/* Update classification lookup table register */
static void mv_pp2x_cls_lookup_write(struct pp2_hw *hw,
				     struct mv_pp2x_cls_lookup_entry *le)
{
	u32 val;

	val = (le->way << MVPP2_CLS_LKP_INDEX_WAY_OFFS) | le->lkpid;
	pp2_reg_write(hw->base[0].va, MVPP2_CLS_LKP_INDEX_REG, val);
	pp2_reg_write(hw->base[0].va, MVPP2_CLS_LKP_TBL_REG, le->data);
}

/* Init lookup decoding table with lookup id */
void mv_pp2x_cls_lookup_tbl_config(struct pp2_hw *hw)
{
	int index, flow_idx;
	int data[3];
	struct mv_pp2x_cls_lookup_entry le;
	struct mv_pp2x_cls_flow_info *flow_info;

	memset(&le, 0, sizeof(struct mv_pp2x_cls_lookup_entry));
	/* Enable classifier engine */
	mv_pp2x_cls_sw_lkp_en_set(&le, 1);

	for (index = 0; index < (MVPP2_PRS_FL_LAST - MVPP2_PRS_FL_START);
	     index++) {
		flow_info = &hw->cls_shadow->flow_info[index];
		data[0] = MVPP2_FLOW_TBL_SIZE;
		data[1] = MVPP2_FLOW_TBL_SIZE;
		data[2] = MVPP2_FLOW_TBL_SIZE;
		le.lkpid = hw->cls_shadow->flow_info[index].lkpid;
		/* Find the min non-zero one in flow_entry_dflt,
		 * flow_entry_vlan, and flow_entry_dscp
		 */
		if (flow_info->flow_entry_dflt)
			data[0] = flow_info->flow_entry_dflt;
		if (flow_info->flow_entry_vlan)
			data[1] = flow_info->flow_entry_vlan;
		if (flow_info->flow_entry_dscp)
			data[2] = flow_info->flow_entry_dscp;
		flow_idx = min(data[0], min(data[1], data[2]));

		/* Set flow pointer index */
		mv_pp2x_cls_sw_lkp_flow_set(&le, flow_idx);

		/* Set initial rx queue */
		mv_pp2x_cls_sw_lkp_rxq_set(&le, 0x0);

		le.way = 0;

		/* Update lookup ID table entry */
		mv_pp2x_cls_lookup_write(hw, &le);

		le.way = 1;

		/* Update lookup ID table entry */
		mv_pp2x_cls_lookup_write(hw, &le);
	}
}

/* Update the flow index for flow of lkpid */
void mv_pp2x_cls_lkp_flow_set(struct pp2_hw *hw, int lkpid, int way,
			      int flow_idx)
{
	struct mv_pp2x_cls_lookup_entry le;

	mv_pp2x_cls_lookup_read(hw, lkpid, way, &le);
	mv_pp2x_cls_sw_lkp_flow_set(&le, flow_idx);
	mv_pp2x_cls_lookup_write(hw, &le);
}

/* Classifier default initialization */
int mv_pp2x_cls_init(struct pp2_hw *hw)
{
	struct mv_pp2x_cls_lookup_entry le;
	struct mv_pp2x_cls_flow_entry fe;
	int index;

	/* Enable classifier */
	pp2_reg_write(hw->base[0].va, MVPP2_CLS_MODE_REG,
		      MVPP2_CLS_MODE_ACTIVE_MASK);

	/* Clear classifier flow table */
	memset(&fe.data, 0, MVPP2_CLS_FLOWS_TBL_DATA_WORDS);
	for (index = 0; index < MVPP2_CLS_FLOWS_TBL_SIZE; index++) {
		fe.index = index;
		mv_pp2x_cls_flow_write(hw, &fe);
	}

	/* Clear classifier lookup table */
	le.data = 0;
	for (index = 0; index < MVPP2_CLS_LKP_TBL_SIZE; index++) {
		le.lkpid = index;
		le.way = 0;
		mv_pp2x_cls_lookup_write(hw, &le);

		le.way = 1;
		mv_pp2x_cls_lookup_write(hw, &le);
	}

	hw->cls_shadow = kcalloc(1, sizeof(struct mv_pp2x_cls_shadow), GFP_KERNEL);
	if (!hw->cls_shadow)
		return -ENOMEM;

	hw->cls_shadow->flow_info =
	    kcalloc((MVPP2_PRS_FL_LAST - MVPP2_PRS_FL_START),
		    sizeof(struct mv_pp2x_cls_flow_info), GFP_KERNEL);
	if (!hw->cls_shadow->flow_info)
		return -ENOMEM;

	/* Start from entry 1 to allocate flow table */
	hw->cls_shadow->flow_free_start = 1;
	for (index = 0; index < (MVPP2_PRS_FL_LAST - MVPP2_PRS_FL_START);
	     index++)
		hw->cls_shadow->flow_info[index].lkpid = index +
		    MVPP2_PRS_FL_START;

	/* Init flow table */
	mv_pp2x_cls_flow_tbl_config(hw);

	/* Init lookup table */
	mv_pp2x_cls_lookup_tbl_config(hw);

	return 0;
}

int mv_pp2x_cls_c2_hw_inv(struct pp2_hw *hw, int index)
{
	if (!hw || index >= MVPP2_CLS_C2_TCAM_SIZE)
		return -EINVAL;

	/* write index reg */
	pp2_reg_write(hw->base[0].va, MVPP2_CLS2_TCAM_IDX_REG, index);

	/* set invalid bit */
	pp2_reg_write(hw->base[0].va, MVPP2_CLS2_TCAM_INV_REG,
		      (1 << MVPP2_CLS2_TCAM_INV_INVALID_OFF));

	/* trigger */
	pp2_reg_write(hw->base[0].va, MVPP2_CLS2_TCAM_DATA_REG(4), 0);

	return 0;
}

void mv_pp2x_cls_c2_hw_inv_all(struct pp2_hw *hw)
{
	int index;

	for (index = 0; index < MVPP2_CLS_C2_TCAM_SIZE; index++)
		mv_pp2x_cls_c2_hw_inv(hw, index);
}

static void mv_pp2x_cls_c2_qos_hw_clear_all(struct pp2_hw *hw)
{
	struct mv_pp2x_cls_c2_qos_entry qos;

	memset(&qos, 0, sizeof(struct mv_pp2x_cls_c2_qos_entry));

	/* clear DSCP tables */
	qos.tbl_sel = MVPP2_QOS_TBL_SEL_DSCP;
	for (qos.tbl_id = 0; qos.tbl_id < MVPP2_QOS_TBL_NUM_DSCP;
	     qos.tbl_id++) {
		for (qos.tbl_line = 0; qos.tbl_line <
		     MVPP2_QOS_TBL_LINE_NUM_DSCP; qos.tbl_line++) {
			mv_pp2x_cls_c2_qos_hw_write(hw, &qos);
		}
	}

	/* clear PRIO tables */
	qos.tbl_sel = MVPP2_QOS_TBL_SEL_PRI;
	for (qos.tbl_id = 0; qos.tbl_id < MVPP2_QOS_TBL_NUM_PRI; qos.tbl_id++)
		for (qos.tbl_line = 0; qos.tbl_line <
		     MVPP2_QOS_TBL_LINE_NUM_PRI; qos.tbl_line++) {
			mv_pp2x_cls_c2_qos_hw_write(hw, &qos);
		}
}

/* C2 TCAM init */
int mv_pp2x_c2_init(struct pp2_hw *hw)
{
	int i;

	/* Invalid all C2 and QoS entries */
	mv_pp2x_cls_c2_hw_inv_all(hw);

	mv_pp2x_cls_c2_qos_hw_clear_all(hw);

	/* Set CLSC2_TCAM_CTRL to enable C2, or C2 does not work */
	pp2_reg_write(hw->base[0].va, MVPP2_CLS2_TCAM_CTRL_REG,
		      MVPP2_CLS2_TCAM_CTRL_EN_MASK);

	/* Allocate mem for c2 shadow */
	hw->c2_shadow = kcalloc(1, sizeof(struct mv_pp2x_c2_shadow), GFP_KERNEL);
	if (!hw->c2_shadow)
		return -ENOMEM;

	/* Init the rule idx to invalid value */
	for (i = 0; i < 8; i++) {
		hw->c2_shadow->rule_idx_info[i].vlan_pri_idx =
		    MVPP2_CLS_C2_TCAM_SIZE;
		hw->c2_shadow->rule_idx_info[i].dscp_pri_idx =
		    MVPP2_CLS_C2_TCAM_SIZE;
		hw->c2_shadow->rule_idx_info[i].default_rule_idx =
		    MVPP2_CLS_C2_TCAM_SIZE;
	}
	hw->c2_shadow->c2_tcam_free_start = 0;

	return 0;
}

void mv_pp2x_cls_oversize_rxq_set(struct pp2_port *port)
{
	uintptr_t cpu_slot = port->cpu_slot;

	pp2_reg_write(cpu_slot, MVPP2_CLS_OVERSIZE_RXQ_LOW_REG(port->id),
		      port->first_rxq);
}

void mv_pp2x_cls_port_config(struct pp2_port *port)
{
	struct mv_pp2x_cls_lookup_entry le;
	struct pp2_hw *hw = &port->parent->hw;
	uintptr_t cpu_slot = port->cpu_slot;
	u32 val;

	/* Set way for the port */
	val = pp2_reg_read(cpu_slot, MVPP2_CLS_PORT_WAY_REG);
	val &= ~MVPP2_CLS_PORT_WAY_MASK(port->id);
	pp2_reg_write(cpu_slot, MVPP2_CLS_PORT_WAY_REG, val);

	/* Pick the entry to be accessed in lookup ID decoding table
	 * according to the way and lkpid.
	 */
	le.lkpid = port->id;
	le.way = 0;
	le.data = 0;

	/* Set initial CPU queue for receiving packets */
	le.data &= ~MVPP2_CLS_LKP_TBL_RXQ_MASK;
	le.data |= port->first_rxq;

	/* Disable classification engines */
	le.data &= ~MVPP2_CLS_LKP_TBL_LOOKUP_EN_MASK;

	/* Update lookup ID table entry */
	mv_pp2x_cls_lookup_write(hw, &le);
}

/* The function get the number of cpu online */
static inline int mv_pp2x_num_online_cpu_get(struct pp2_inst *pp)
{
	u8 num_online_cpus = 0;
	u16 x = pp->cpu_map;

	while (x) {
		x &= (x - 1);
		num_online_cpus++;
	}

	return num_online_cpus;
}

/* The function calculate the width, such as cpu width, cos queue width */
static inline void mv_pp2x_width_calc(struct pp2_inst *pp2, uint32_t *cpu_width,
				      u32 *cos_width,
				      uint32_t *port_rxq_width)
{
	if (pp2) {
		/* Calculate CPU width */
		if (cpu_width)
			*cpu_width =
			    ilog2(roundup_pow_of_two
				  (mv_pp2x_num_online_cpu_get(pp2)));

		/* Calculate cos queue width */
		if (cos_width)
			*cos_width =
			    ilog2(roundup_pow_of_two
				  (pp2->pp2_cfg.cos_cfg.num_cos_queues));
		/* Calculate rx queue width on the port */
		if (port_rxq_width)
			*port_rxq_width =
			    ilog2(roundup_pow_of_two
				  (pp2->pp2xdata.pp2x_max_port_rxqs));
	}
}

/* Set bits in sram sw entry */
static void mv_pp2x_prs_sram_bits_set(struct mv_pp2x_prs_entry *pe, int bit_num,
				      int val)
{
	pe->sram.byte[SRAM_BIT_TO_BYTE(bit_num)] |= (val << (bit_num % 8));
}

/* Clear bits in sram sw entry */
static void mv_pp2x_prs_sram_bits_clear(struct mv_pp2x_prs_entry *pe,
					int bit_num, int val)
{
	pe->sram.byte[SRAM_BIT_TO_BYTE(bit_num)] &= ~(val << (bit_num % 8));
}

/* Update ri bits in sram sw entry */
void mv_pp2x_prs_sram_ri_update(struct mv_pp2x_prs_entry *pe,
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

/* Update ai bits in sram sw entry */
void mv_pp2x_prs_sram_ai_update(struct mv_pp2x_prs_entry *pe,
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
void mv_pp2x_prs_sram_next_lu_set(struct mv_pp2x_prs_entry *pe, unsigned int lu)
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

/* Update parser tcam and sram hw entries */
int mv_pp2x_prs_hw_write(struct pp2_hw *hw, struct mv_pp2x_prs_entry *pe)
{
	int i;

	if (pe->index > MVPP2_PRS_TCAM_SRAM_SIZE - 1)
		return -EINVAL;

	/* Clear entry invalidation bit */
	pe->tcam.word[MVPP2_PRS_TCAM_INV_WORD] &= ~MVPP2_PRS_TCAM_INV_MASK;

	/* Write tcam index - indirect access */
	pp2_reg_write(hw->base[0].va, MVPP2_PRS_TCAM_IDX_REG, pe->index);
	for (i = 0; i < MVPP2_PRS_TCAM_WORDS; i++)
		pp2_reg_write(hw->base[0].va, MVPP2_PRS_TCAM_DATA_REG(i),
			      pe->tcam.word[i]);

	/* Write sram index - indirect access */
	pp2_reg_write(hw->base[0].va, MVPP2_PRS_SRAM_IDX_REG, pe->index);
	for (i = 0; i < MVPP2_PRS_SRAM_WORDS; i++)
		pp2_reg_write(hw->base[0].va, MVPP2_PRS_SRAM_DATA_REG(i),
			      pe->sram.word[i]);

	return 0;
}

/* Read tcam entry from hw */
int mv_pp2x_prs_hw_read(struct pp2_hw *hw, struct mv_pp2x_prs_entry *pe)
{
	int i;

	if (pe->index > MVPP2_PRS_TCAM_SRAM_SIZE - 1)
		return -EINVAL;

	/* Write tcam index - indirect access */
	pp2_reg_write(hw->base[0].va, MVPP2_PRS_TCAM_IDX_REG, pe->index);

	pe->tcam.word[MVPP2_PRS_TCAM_INV_WORD] =
	    pp2_reg_read(hw->base[0].va,
			 MVPP2_PRS_TCAM_DATA_REG(MVPP2_PRS_TCAM_INV_WORD));
	if (pe->tcam.word[MVPP2_PRS_TCAM_INV_WORD] & MVPP2_PRS_TCAM_INV_MASK)
		return MVPP2_PRS_TCAM_ENTRY_INVALID;

	for (i = 0; i < MVPP2_PRS_TCAM_WORDS; i++)
		pe->tcam.word[i] =
		    pp2_reg_read(hw->base[0].va, MVPP2_PRS_TCAM_DATA_REG(i));

	/* Write sram index - indirect access */
	pp2_reg_write(hw->base[0].va, MVPP2_PRS_SRAM_IDX_REG, pe->index);
	for (i = 0; i < MVPP2_PRS_SRAM_WORDS; i++)
		pe->sram.word[i] =
		    pp2_reg_read(hw->base[0].va, MVPP2_PRS_SRAM_DATA_REG(i));

	return 0;
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
void mv_pp2x_prs_tcam_data_byte_set(struct mv_pp2x_prs_entry *pe,
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
void mv_pp2x_prs_tcam_ai_update(struct mv_pp2x_prs_entry *pe,
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
				    unsigned short ethertype)
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
void mv_pp2x_prs_tcam_port_map_set(struct mv_pp2x_prs_entry *pe,
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
void mv_pp2x_prs_tcam_port_set(struct mv_pp2x_prs_entry *pe,
			       unsigned int port, bool add)
{
	int enable_off =
	    HW_BYTE_OFFS(MVPP2_PRS_TCAM_EN_OFFS(MVPP2_PRS_TCAM_PORT_BYTE));

	if (add)
		pe->tcam.byte[enable_off] &= ~(1 << port);
	else
		pe->tcam.byte[enable_off] |= 1 << port;
}

/* Enable shadow table entry and set its lookup ID */
static void mv_pp2x_prs_shadow_set(struct pp2_hw *hw, int index, int lu)
{
	hw->prs_shadow[index].valid = true;
	hw->prs_shadow[index].lu = lu;
}

/* Update ri fields in shadow table entry */
static void mv_pp2x_prs_shadow_ri_set(struct pp2_hw *hw, int index,
				      unsigned int ri, unsigned int ri_mask)
{
	hw->prs_shadow[index].ri_mask = ri_mask;
	hw->prs_shadow[index].ri = ri;
}

/* Update lookup field in tcam sw entry */
void mv_pp2x_prs_tcam_lu_set(struct mv_pp2x_prs_entry *pe, unsigned int lu)
{
	unsigned int offset = MVPP2_PRS_TCAM_LU_BYTE;
	unsigned int enable_off =
	    MVPP2_PRS_TCAM_EN_OFFS(MVPP2_PRS_TCAM_LU_BYTE);

	pe->tcam.byte[HW_BYTE_OFFS(offset)] = lu;
	pe->tcam.byte[HW_BYTE_OFFS(enable_off)] = MVPP2_PRS_LU_MASK;
}

/* Return first free tcam index, seeking from start to end */
static int mv_pp2x_prs_tcam_first_free(struct pp2_hw *hw,
				       unsigned char start, unsigned char end)
{
	int tid;

	if (start > end)
		swap(start, end);

	if (end >= MVPP2_PRS_TCAM_SRAM_SIZE)
		end = MVPP2_PRS_TCAM_SRAM_SIZE - 1;

	for (tid = start; tid <= end; tid++) {
		if (!hw->prs_shadow[tid].valid)
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

/* Compare tcam data bytes with a pattern */
static bool mv_pp2x_prs_tcam_data_cmp(struct mv_pp2x_prs_entry *pe, int offs,
				      uint16_t data)
{
	u16 tcam_data;

	tcam_data = (pe->tcam.byte[TCAM_DATA_BYTE(offs + 1)] << 8) |
	    pe->tcam.byte[TCAM_DATA_BYTE(offs)];
	if (tcam_data != data)
		return false;
	return true;
}

/* Find tcam entry with matched pair <MAC DA, port> */
static struct mv_pp2x_prs_entry *
mv_pp2x_prs_mac_da_range_find(struct pp2_hw *hw, int pmap,
			      const u8 *da, unsigned char *mask,
			      int udf_type)
{
	struct mv_pp2x_prs_entry *pe;
	int tid;

	pe = kcalloc(1, sizeof(*pe), GFP_KERNEL);
	if (!pe)
		return NULL;
	mv_pp2x_prs_tcam_lu_set(pe, MVPP2_PRS_LU_MAC);

	/* Go through the all entires with MVPP2_PRS_LU_MAC */
	for (tid = MVPP2_PE_FIRST_FREE_TID;
	     tid <= MVPP2_PE_LAST_FREE_TID; tid++) {
		unsigned int entry_pmap;

		if (!hw->prs_shadow[tid].valid ||
		    (hw->prs_shadow[tid].lu != MVPP2_PRS_LU_MAC) ||
		    (hw->prs_shadow[tid].udf != udf_type))
			continue;

		pe->index = tid;
		mv_pp2x_prs_hw_read(hw, pe);
		entry_pmap = mv_pp2x_prs_tcam_port_map_get(pe);
	(void)entry_pmap;

		if (mv_pp2x_prs_mac_range_equals(pe, da, mask))
			return pe;
	}
	kfree(pe);

	return NULL;
}

/* Set entry for dsa packets */
static void mv_pp2x_prs_dsa_tag_set(struct pp2_hw *hw, int port, bool add,
				    bool tagged, bool extend)
{
	struct mv_pp2x_prs_entry pe;
	int tid, shift;

	if (extend) {
		tid = tagged ? MVPP2_PE_EDSA_TAGGED : MVPP2_PE_EDSA_UNTAGGED;
		shift = 8;
	} else {
		tid = tagged ? MVPP2_PE_DSA_TAGGED : MVPP2_PE_DSA_UNTAGGED;
		shift = 4;
	}

	if (hw->prs_shadow[tid].valid) {
		/* Entry exist - update port only */
		pe.index = tid;
		mv_pp2x_prs_hw_read(hw, &pe);
	} else {
		/* Entry doesn't exist - create new */
		memset(&pe, 0, sizeof(struct mv_pp2x_prs_entry));
		mv_pp2x_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_DSA);
		pe.index = tid;

		/* Shift 4 bytes if DSA tag or 8 bytes in case of EDSA tag */
		mv_pp2x_prs_sram_shift_set(&pe, shift,
					   MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);

		/* Update shadow table */
		mv_pp2x_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_DSA);

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

		/* Mask all ports */
		mv_pp2x_prs_tcam_port_map_set(&pe, 0);
	}

	/* Update port mask */
	mv_pp2x_prs_tcam_port_set(&pe, port, add);

	mv_pp2x_prs_hw_write(hw, &pe);
}

/* Update parser's mac da entry */
int mv_pp2x_prs_mac_da_accept(struct pp2_hw *hw, int port,
			      const u8 *da, bool add)
{
	struct mv_pp2x_prs_entry *pe;
	unsigned int pmap, len, ri;
	unsigned char mask[ETH_ALEN] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
	int tid;

	/* Scan TCAM and see if entry with this <MAC DA, port> already exist */
	pe = mv_pp2x_prs_mac_da_range_find(hw, (1 << port), da, mask,
					   MVPP2_PRS_UDF_MAC_DEF);

	/* No such entry */
	if (!pe) {
		if (!add)
			return 0;

		/* Create new TCAM entry */
		/* Find first range mac entry */
		for (tid = MVPP2_PE_FIRST_FREE_TID;
		     tid <= MVPP2_PE_LAST_FREE_TID; tid++)
			if (hw->prs_shadow[tid].valid &&
			    (hw->prs_shadow[tid].lu == MVPP2_PRS_LU_MAC) &&
			    (hw->prs_shadow[tid].udf ==
			     MVPP2_PRS_UDF_MAC_RANGE))
				break;

		/* Go through the all entries from first to last */
		tid = mv_pp2x_prs_tcam_first_free(hw, MVPP2_PE_FIRST_FREE_TID,
						  tid - 1);
		if (tid < 0)
			return tid;

		pe = kcalloc(1, sizeof(*pe), GFP_KERNEL);
		if (!pe)
			return -1;
		mv_pp2x_prs_tcam_lu_set(pe, MVPP2_PRS_LU_MAC);
		pe->index = tid;

		/* Mask all ports */
		mv_pp2x_prs_tcam_port_map_set(pe, 0);
	}

	/* Update port mask */
	mv_pp2x_prs_tcam_port_set(pe, port, add);

	/* Invalidate the entry if no ports are left enabled */
	pmap = mv_pp2x_prs_tcam_port_map_get(pe);
	if (pmap == 0) {
		if (add) {
			kfree(pe);
			return -1;
		}
		mv_pp2x_prs_hw_inv(hw, pe->index);
		hw->prs_shadow[pe->index].valid = false;
		kfree(pe);
		return 0;
	}

	/* Continue - set next lookup */
	mv_pp2x_prs_sram_next_lu_set(pe, MVPP2_PRS_LU_DSA);

	/* Set match on DA */
	len = ETH_ALEN;
	while (len--)
		mv_pp2x_prs_tcam_data_byte_set(pe, len, da[len], 0xff);

	/* Set result info bits */
	if (mv_check_eaddr_bc(da))
		ri = MVPP2_PRS_RI_L2_BCAST;
	else if (mv_check_eaddr_mc(da))
		ri = MVPP2_PRS_RI_L2_MCAST;
	else
		ri = MVPP2_PRS_RI_L2_UCAST | MVPP2_PRS_RI_MAC_ME_MASK;

	mv_pp2x_prs_sram_ri_update(pe, ri, MVPP2_PRS_RI_L2_CAST_MASK |
				   MVPP2_PRS_RI_MAC_ME_MASK);
	mv_pp2x_prs_shadow_ri_set(hw, pe->index, ri, MVPP2_PRS_RI_L2_CAST_MASK |
				  MVPP2_PRS_RI_MAC_ME_MASK);

	/* Shift to ethertype */
	mv_pp2x_prs_sram_shift_set(pe, 2 * ETH_ALEN,
				   MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);

	/* Update shadow table and hw entry */
	hw->prs_shadow[pe->index].udf = MVPP2_PRS_UDF_MAC_DEF;
	mv_pp2x_prs_shadow_set(hw, pe->index, MVPP2_PRS_LU_MAC);
	mv_pp2x_prs_hw_write(hw, pe);

	kfree(pe);

	return 0;
}

int mv_pp2x_prs_tag_mode_set(struct pp2_hw *hw, int port, int type)
{
	switch (type) {
	case MVPP2_TAG_TYPE_EDSA:
		/* Add port to EDSA entries */
		mv_pp2x_prs_dsa_tag_set(hw, port, true,
					MVPP2_PRS_TAGGED, MVPP2_PRS_EDSA);
		mv_pp2x_prs_dsa_tag_set(hw, port, true,
					MVPP2_PRS_UNTAGGED, MVPP2_PRS_EDSA);
		/* Remove port from DSA entries */
		mv_pp2x_prs_dsa_tag_set(hw, port, false,
					MVPP2_PRS_TAGGED, MVPP2_PRS_DSA);
		mv_pp2x_prs_dsa_tag_set(hw, port, false,
					MVPP2_PRS_UNTAGGED, MVPP2_PRS_DSA);
		break;

	case MVPP2_TAG_TYPE_DSA:
		/* Add port to DSA entries */
		mv_pp2x_prs_dsa_tag_set(hw, port, true,
					MVPP2_PRS_TAGGED, MVPP2_PRS_DSA);
		mv_pp2x_prs_dsa_tag_set(hw, port, true,
					MVPP2_PRS_UNTAGGED, MVPP2_PRS_DSA);
		/* Remove port from EDSA entries */
		mv_pp2x_prs_dsa_tag_set(hw, port, false,
					MVPP2_PRS_TAGGED, MVPP2_PRS_EDSA);
		mv_pp2x_prs_dsa_tag_set(hw, port, false,
					MVPP2_PRS_UNTAGGED, MVPP2_PRS_EDSA);
		break;

	case MVPP2_TAG_TYPE_MH:
	case MVPP2_TAG_TYPE_NONE:
		/* Remove port form EDSA and DSA entries */
		mv_pp2x_prs_dsa_tag_set(hw, port, false,
					MVPP2_PRS_TAGGED, MVPP2_PRS_DSA);
		mv_pp2x_prs_dsa_tag_set(hw, port, false,
					MVPP2_PRS_UNTAGGED, MVPP2_PRS_DSA);
		mv_pp2x_prs_dsa_tag_set(hw, port, false,
					MVPP2_PRS_TAGGED, MVPP2_PRS_EDSA);
		mv_pp2x_prs_dsa_tag_set(hw, port, false,
					MVPP2_PRS_UNTAGGED, MVPP2_PRS_EDSA);
		break;

	default:
		if ((type < 0) || (type > MVPP2_TAG_TYPE_EDSA))
			return -EINVAL;
	}

	return 0;
}

/* Find parser flow entry */
static struct mv_pp2x_prs_entry *mv_pp2x_prs_flow_find(struct pp2_hw *hw,
						       int flow,
						       unsigned int ri,
						       unsigned int ri_mask)
{
	struct mv_pp2x_prs_entry *pe;
	int tid;
	unsigned int dword, enable;

	pe = kcalloc(1, sizeof(*pe), GFP_KERNEL);
	if (!pe)
		return NULL;
	mv_pp2x_prs_tcam_lu_set(pe, MVPP2_PRS_LU_FLOWS);

	/* Go through the all entires with MVPP2_PRS_LU_FLOWS */
	for (tid = MVPP2_PRS_TCAM_SRAM_SIZE - 1; tid >= 0; tid--) {
		u8 bits;

		if (!hw->prs_shadow[tid].valid ||
		    hw->prs_shadow[tid].lu != MVPP2_PRS_LU_FLOWS)
			continue;

		pe->index = tid;
		mv_pp2x_prs_hw_read(hw, pe);

		/* Check result info, because there maybe several
		 * TCAM lines to generate the same flow
		 */
		mv_pp2x_prs_tcam_data_dword_get(pe, 0, &dword, &enable);
		if ((dword != ri) || (enable != ri_mask))
			continue;

		bits = mv_pp2x_prs_sram_ai_get(pe);

		/* Sram store classification lookup ID in AI bits [5:0] */
		if ((bits & MVPP2_PRS_FLOW_ID_MASK) == flow)
			return pe;
	}
	kfree(pe);

	return NULL;
}

/* Set prs dedicated flow for the port */
int mv_pp2x_prs_flow_id_gen(struct pp2_port *port, uint32_t flow_id,
			    u32 res, uint32_t res_mask)
{
	struct mv_pp2x_prs_entry *pe;
	struct pp2_hw *hw = &port->parent->hw;
	int tid;
	unsigned int pmap = 0;

	pe = mv_pp2x_prs_flow_find(hw, flow_id, res, res_mask);

	/* Such entry not exist */
	if (!pe) {
		pe = kcalloc(1, sizeof(*pe), GFP_KERNEL);
		if (!pe)
			return -ENOMEM;

		/* Go through the all entires from last to first */
		tid = mv_pp2x_prs_tcam_first_free(hw,
						  MVPP2_PE_LAST_FREE_TID,
						  MVPP2_PE_FIRST_FREE_TID);
		if (tid < 0) {
			kfree(pe);
			return tid;
		}

		mv_pp2x_prs_tcam_lu_set(pe, MVPP2_PRS_LU_FLOWS);
		pe->index = tid;

		mv_pp2x_prs_sram_ai_update(pe, flow_id, MVPP2_PRS_FLOW_ID_MASK);
		mv_pp2x_prs_sram_bits_set(pe, MVPP2_PRS_SRAM_LU_DONE_BIT, 1);

		/* Update shadow table */
		mv_pp2x_prs_shadow_set(hw, pe->index, MVPP2_PRS_LU_FLOWS);

		/*update result data and mask */
		mv_pp2x_prs_tcam_data_dword_set(pe, 0, res, res_mask);
	} else {
		pmap = mv_pp2x_prs_tcam_port_map_get(pe);
	}

	mv_pp2x_prs_tcam_port_map_set(pe, (1 << port->id) | pmap);
	mv_pp2x_prs_hw_write(hw, pe);
	kfree(pe);

	return 0;
}

int mv_pp2x_prs_flow_set(struct pp2_port *port)
{
	int index, ret;

	for (index = 0; index < MVPP2_PRS_FL_TCAM_NUM; index++) {
		ret = mv_pp2x_prs_flow_id_gen(port,
					      mv_pp2x_prs_flow_id_array
					      [index].flow_id,
					      mv_pp2x_prs_flow_id_array
					      [index].prs_result.ri,
					      mv_pp2x_prs_flow_id_array
					      [index].prs_result.ri_mask);
		if (ret)
			return ret;
	}
	return 0;
}

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
void mv_pp2x_prs_flow_id_attr_init(void)
{
	int index;
	u32 ri, ri_mask, flow_id;

	for (index = 0; index < MVPP2_PRS_FL_TCAM_NUM; index++) {
		ri      = mv_pp2x_prs_flow_id_array[index].prs_result.ri;
		ri_mask = mv_pp2x_prs_flow_id_array[index].prs_result.ri_mask;
		flow_id = mv_pp2x_prs_flow_id_array[index].flow_id;

		mv_pp2x_prs_flow_id_attr_set(flow_id, ri, ri_mask);
	}
}

int mv_pp2x_prs_flow_id_attr_get(int flow_id)
{
	return mv_pp2x_prs_flow_id_attr_tbl[flow_id];
}

/*********************************************************************/
/***************** Classifier Top Public flows table APIs  ********************/
/********************************************************************/

int mv_pp2x_cls_hw_flow_write(uintptr_t cpu_slot,
			      struct mv_pp2x_cls_flow_entry *fe)
{
	if (mv_pp2x_range_validate(fe->index, 0,
				   MVPP2_CLS_FLOWS_TBL_SIZE) == MV_ERROR)
		return -EINVAL;

	/* write index */
	pp2_reg_write(cpu_slot, MVPP2_CLS_FLOW_INDEX_REG, fe->index);

	pp2_reg_write(cpu_slot, MVPP2_CLS_FLOW_TBL0_REG, fe->data[0]);
	pp2_reg_write(cpu_slot, MVPP2_CLS_FLOW_TBL1_REG, fe->data[1]);
	pp2_reg_write(cpu_slot, MVPP2_CLS_FLOW_TBL2_REG, fe->data[2]);

	return 0;
}

int mv_pp2x_cls_hw_flow_read(uintptr_t cpu_slot, int index,
			     struct mv_pp2x_cls_flow_entry *fe)
{
	if (mv_pp2x_ptr_validate(fe) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_range_validate(index, 0,
				   MVPP2_CLS_FLOWS_TBL_SIZE) == MV_ERROR)
		return MV_ERROR;

	fe->index = index;

	/*write index */
	pp2_reg_write(cpu_slot, MVPP2_CLS_FLOW_INDEX_REG, index);

	fe->data[0] = pp2_reg_read(cpu_slot, MVPP2_CLS_FLOW_TBL0_REG);
	fe->data[1] = pp2_reg_read(cpu_slot, MVPP2_CLS_FLOW_TBL1_REG);
	fe->data[2] = pp2_reg_read(cpu_slot, MVPP2_CLS_FLOW_TBL2_REG);

	return 0;
}

int mv_pp2x_cls_sw_flow_hek_get(struct mv_pp2x_cls_flow_entry *fe,
				int *num_of_fields, int field_ids[])
{
	int index;

	if (mv_pp2x_ptr_validate(fe) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_ptr_validate(num_of_fields) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_ptr_validate(field_ids) == MV_ERROR)
		return MV_ERROR;

	*num_of_fields = (fe->data[1] &
			  MVPP2_FLOW_FIELDS_NUM_MASK) >> MVPP2_FLOW_FIELDS_NUM;

	for (index = 0; index < (*num_of_fields); index++)
		field_ids[index] = ((fe->data[2] &
				     MVPP2_FLOW_FIELD_MASK(index)) >>
				    MVPP2_FLOW_FIELD_ID(index));

	return 0;
}

int mv_pp2x_cls_sw_flow_port_get(struct mv_pp2x_cls_flow_entry *fe,
				 int *type, int *portid)
{
	if (mv_pp2x_ptr_validate(fe) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_ptr_validate(type) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_ptr_validate(portid) == MV_ERROR)
		return MV_ERROR;

	*type = (fe->data[0] & MVPP2_FLOW_PORT_TYPE_MASK) >>
	    MVPP2_FLOW_PORT_TYPE;
	*portid = (fe->data[0] & MVPP2_FLOW_PORT_ID_MASK) >> MVPP2_FLOW_PORT_ID;

	return 0;
}

int mv_pp2x_cls_sw_flow_port_set(struct mv_pp2x_cls_flow_entry *fe,
				 int type, int portid)
{
	if (mv_pp2x_ptr_validate(fe) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_range_validate(type, 0,
				   ((1 << MVPP2_FLOW_PORT_TYPE_BITS) - 1)) ==
	    MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_range_validate(portid, 0,
				   ((1 << MVPP2_FLOW_PORT_ID_BITS) - 1)) ==
	    MV_ERROR)
		return MV_ERROR;

	fe->data[0] &= ~MVPP2_FLOW_PORT_ID_MASK;
	fe->data[0] &= ~MVPP2_FLOW_PORT_TYPE_MASK;

	fe->data[0] |= (portid << MVPP2_FLOW_PORT_ID);
	fe->data[0] |= (type << MVPP2_FLOW_PORT_TYPE);

	return 0;
}

int mv_pp2x_cls_sw_flow_portid_select(struct mv_pp2x_cls_flow_entry *fe,
				      int from)
{
	if (mv_pp2x_ptr_validate(fe) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_range_validate(from, 0, 1) == MV_ERROR)
		return MV_ERROR;

	if (from)
		fe->data[0] |= MVPP2_FLOW_PORT_ID_SEL_MASK;
	else
		fe->data[0] &= ~MVPP2_FLOW_PORT_ID_SEL_MASK;

	return 0;
}

int mv_pp2x_cls_sw_flow_pppoe_set(struct mv_pp2x_cls_flow_entry *fe, int mode)
{
	if (mv_pp2x_ptr_validate(fe) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_range_validate(mode, 0, MVPP2_FLOW_PPPOE_MAX) == MV_ERROR)
		return MV_ERROR;

	fe->data[0] &= ~MVPP2_FLOW_PPPOE_MASK;
	fe->data[0] |= (mode << MVPP2_FLOW_PPPOE);
	return 0;
}

int mv_pp2x_cls_sw_flow_vlan_set(struct mv_pp2x_cls_flow_entry *fe, int mode)
{
	if (mv_pp2x_ptr_validate(fe) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_range_validate(mode, 0, MVPP2_FLOW_VLAN_MAX) == MV_ERROR)
		return MV_ERROR;

	fe->data[0] &= ~MVPP2_FLOW_VLAN_MASK;
	fe->data[0] |= (mode << MVPP2_FLOW_VLAN);
	return 0;
}

int mv_pp2x_cls_sw_flow_macme_set(struct mv_pp2x_cls_flow_entry *fe, int mode)
{
	if (mv_pp2x_ptr_validate(fe) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_range_validate(mode, 0, MVPP2_FLOW_MACME_MAX) == MV_ERROR)
		return MV_ERROR;

	fe->data[0] &= ~MVPP2_FLOW_MACME_MASK;
	fe->data[0] |= (mode << MVPP2_FLOW_MACME);
	return 0;
}

int mv_pp2x_cls_sw_flow_udf7_set(struct mv_pp2x_cls_flow_entry *fe, int mode)
{
	if (mv_pp2x_ptr_validate(fe) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_range_validate(mode, 0, MVPP2_FLOW_UDF7_MAX) == MV_ERROR)
		return MV_ERROR;

	fe->data[0] &= ~MVPP2_FLOW_UDF7_MASK;
	fe->data[0] |= (mode << MVPP2_FLOW_UDF7);
	return 0;
}

int mv_pp2x_cls_sw_flow_seq_ctrl_set(struct mv_pp2x_cls_flow_entry *fe,
				     int mode)
{
	if (mv_pp2x_ptr_validate(fe) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_range_validate(mode, 0, MVPP2_FLOW_ENGINE_MAX) == MV_ERROR)
		return MV_ERROR;

	fe->data[1] &= ~MVPP2_FLOW_SEQ_CTRL_MASK;
	fe->data[1] |= (mode << MVPP2_FLOW_SEQ_CTRL);

	return 0;
}

int mv_pp2x_cls_sw_flow_seq_ctrl_get(struct mv_pp2x_cls_flow_entry *fe,
				     int *mode)
{
	if (mv_pp2x_ptr_validate(fe) == MV_ERROR)
		return MV_ERROR;
	if (mv_pp2x_ptr_validate(mode) == MV_ERROR)
		return MV_ERROR;

	*mode = (fe->data[1] & MVPP2_FLOW_SEQ_CTRL_MASK) >> MVPP2_FLOW_SEQ_CTRL;

	return 0;
}

int mv_pp2x_cls_sw_flow_engine_get(struct mv_pp2x_cls_flow_entry *fe,
				   int *engine, int *is_last)
{
	if (mv_pp2x_ptr_validate(fe) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_ptr_validate(engine) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_ptr_validate(is_last) == MV_ERROR)
		return MV_ERROR;

	*engine = (fe->data[0] & MVPP2_FLOW_ENGINE_MASK) >> MVPP2_FLOW_ENGINE;
	*is_last = fe->data[0] & MVPP2_FLOW_LAST_MASK;

	return 0;
}

int mv_pp2x_cls_sw_flow_engine_set(struct mv_pp2x_cls_flow_entry *fe,
				   int engine, int is_last)
{
	if (mv_pp2x_ptr_validate(fe) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_range_validate(is_last, 0, 1) == MV_ERROR)
		return MV_ERROR;

	fe->data[0] &= ~MVPP2_FLOW_LAST_MASK;
	fe->data[0] &= ~MVPP2_FLOW_ENGINE_MASK;

	fe->data[0] |= is_last;
	fe->data[0] |= (engine << MVPP2_FLOW_ENGINE);

	return 0;
}

int mv_pp2x_cls_sw_flow_extra_get(struct mv_pp2x_cls_flow_entry *fe,
				  int *type, int *prio)
{
	if (mv_pp2x_ptr_validate(fe) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_ptr_validate(type) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_ptr_validate(prio) == MV_ERROR)
		return MV_ERROR;

	*type = (fe->data[1] & MVPP2_FLOW_LKP_TYPE_MASK) >> MVPP2_FLOW_LKP_TYPE;
	*prio = (fe->data[1] & MVPP2_FLOW_FIELD_PRIO_MASK) >>
	    MVPP2_FLOW_FIELD_PRIO;

	return 0;
}

int mv_pp2x_cls_sw_flow_extra_set(struct mv_pp2x_cls_flow_entry *fe,
				  int type, int prio)
{
	if (mv_pp2x_ptr_validate(fe) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_range_validate(type, 0, MVPP2_FLOW_PORT_ID_MAX) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_range_validate(prio, 0,
				   ((1 << MVPP2_FLOW_FIELD_ID_BITS) - 1)) == MV_ERROR)
		return MV_ERROR;

	fe->data[1] &= ~MVPP2_FLOW_LKP_TYPE_MASK;
	fe->data[1] |= (type << MVPP2_FLOW_LKP_TYPE);

	fe->data[1] &= ~MVPP2_FLOW_FIELD_PRIO_MASK;
	fe->data[1] |= (prio << MVPP2_FLOW_FIELD_PRIO);

	return 0;
}

/* Classifier configuration routines */

/* Update classification flow table registers */
void mv_pp2x_cls_flow_write(struct pp2_hw *hw,
			    struct mv_pp2x_cls_flow_entry *fe)
{
	pp2_reg_write(hw->base[0].va, MVPP2_CLS_FLOW_INDEX_REG, fe->index);
	pp2_reg_write(hw->base[0].va, MVPP2_CLS_FLOW_TBL0_REG, fe->data[0]);
	pp2_reg_write(hw->base[0].va, MVPP2_CLS_FLOW_TBL1_REG, fe->data[1]);
	pp2_reg_write(hw->base[0].va, MVPP2_CLS_FLOW_TBL2_REG, fe->data[2]);
}

static void mv_pp2x_cls_flow_read(struct pp2_hw *hw, int index,
				  struct mv_pp2x_cls_flow_entry *fe)
{
	fe->index = index;
	/*write index */
	pp2_reg_write(hw->base[0].va, MVPP2_CLS_FLOW_INDEX_REG, index);

	fe->data[0] = pp2_reg_read(hw->base[0].va, MVPP2_CLS_FLOW_TBL0_REG);
	fe->data[1] = pp2_reg_read(hw->base[0].va, MVPP2_CLS_FLOW_TBL1_REG);
	fe->data[2] = pp2_reg_read(hw->base[0].va, MVPP2_CLS_FLOW_TBL2_REG);
}

int mv_pp2x_cls_sw_flow_dump(struct mv_pp2x_cls_flow_entry *fe)
{
	int int32bit_1, int32bit_2, i;
	int fields_arr[MVPP2_CLS_FLOWS_TBL_FIELDS_MAX];
	int status = 0;

	if (mv_pp2x_ptr_validate(fe) == MV_ERROR)
		return MV_ERROR;

	printf("INDEX: F[0] F[1] F[2] F[3] PRT[T  ID] ENG LAST LKP_TYP  PRIO\n");

	/* index */
	printf("%5d  ", fe->index);

	/* filed[0] filed[1] filed[2] filed[3] */
	status |= mv_pp2x_cls_sw_flow_hek_get(fe, &int32bit_1, fields_arr);

	for (i = 0 ; i < MVPP2_CLS_FLOWS_TBL_FIELDS_MAX; i++)
		if (i < int32bit_1)
			printf("0x%2.2x ", fields_arr[i]);
		else
			printf(" NA  ");

	/* port_type port_id */
	status |= mv_pp2x_cls_sw_flow_port_get(fe, &int32bit_1, &int32bit_2);
	printf("[%1d  0x%3.3x]  ", int32bit_1, int32bit_2);

	/* engine_num last_bit */
	status |= mv_pp2x_cls_sw_flow_engine_get(fe, &int32bit_1, &int32bit_2);
	printf("%1d   %1d    ", int32bit_1, int32bit_2);

	/* lookup_type priority */
	status |= mv_pp2x_cls_sw_flow_extra_get(fe, &int32bit_1, &int32bit_2);
	printf("0x%2.2x    0x%2.2x", int32bit_1, int32bit_2);

	printf("\n       PPPEO   VLAN   MACME   UDF7   SELECT SEQ_CTRL\n");
	printf("         %1d      %1d      %1d       %1d      %1d      %1d\n",
	       (u32)((fe->data[0] & MVPP2_FLOW_PPPOE_MASK) >> MVPP2_FLOW_PPPOE),
	       (u32)((fe->data[0] & MVPP2_FLOW_VLAN_MASK) >> MVPP2_FLOW_VLAN),
	       (u32)((fe->data[0] & MVPP2_FLOW_MACME_MASK) >> MVPP2_FLOW_MACME),
	       (u32)((fe->data[0] & MVPP2_FLOW_UDF7_MASK) >> MVPP2_FLOW_UDF7),
	       (u32)((fe->data[0] & MVPP2_FLOW_PORT_ID_SEL_MASK) >> MVPP2_FLOW_PORT_ID_SEL),
	       (u32)((fe->data[1] & MVPP2_FLOW_SEQ_CTRL_MASK) >> MVPP2_FLOW_SEQ_CTRL));
	printf("\n");

	return 0;
}

void mv_pp2x_cls_sw_flow_clear(struct mv_pp2x_cls_flow_entry *fe)
{
	memset(fe, 0, sizeof(struct mv_pp2x_cls_flow_entry));
}

int mv_pp2x_cls_hw_flow_clear_all(uintptr_t cpu_slot)
{
	int index;

	struct mv_pp2x_cls_flow_entry fe;

	mv_pp2x_cls_sw_flow_clear(&fe);

	for (index = 0; index < MVPP2_CLS_FLOWS_TBL_SIZE ; index++) {
		fe.index = index;
		if (mv_pp2x_cls_hw_flow_write(cpu_slot, &fe))
			return -EINVAL;
	}
	return 0;
}

static int mv_pp2x_cls_hw_flow_hit_get(uintptr_t cpu_slot, int index, unsigned int *cnt)
{
	if (mv_pp2x_range_validate(index, 0, MVPP2_CLS_FLOWS_TBL_SIZE) == MV_ERROR)
		return -EINVAL;

	/*set index */
	pp2_reg_write(cpu_slot, MVPP2_CNT_IDX_REG, MVPP2_CNT_IDX_FLOW(index));

	if (cnt)
		*cnt = pp2_reg_read(cpu_slot, MVPP2_CLS_FLOW_TBL_HIT_REG);
	else
		printf("HITS = %d\n", pp2_reg_read(cpu_slot, MVPP2_CLS_FLOW_TBL_HIT_REG));

	return 0;
}

int mv_pp2x_cls_hw_lkp_hit_get(uintptr_t cpu_slot, int lkpid, u32 *cnt)
{
	int way = 0;

	if (mv_pp2x_range_validate(lkpid, 0, MVPP2_CLS_LKP_TBL_SIZE) == MV_ERROR)
		return -EINVAL;

	/*set index */
	pp2_reg_write(cpu_slot, MVPP2_CNT_IDX_REG, MVPP2_CNT_IDX_LKP(lkpid, way));

	if (cnt)
		*cnt = pp2_reg_read(cpu_slot, MVPP2_CLS_LKP_TBL_HIT_REG);
	else
		printf("HITS: %d\n", pp2_reg_read(cpu_slot, MVPP2_CLS_LKP_TBL_HIT_REG));

	return 0;
}

int mv_pp2x_cls_hw_flow_dump(uintptr_t cpu_slot)
{
	int index;

	struct mv_pp2x_cls_flow_entry fe;

	for (index = 0; index < MVPP2_CLS_FLOWS_TBL_SIZE; index++) {
		mv_pp2x_cls_hw_flow_read(cpu_slot, index, &fe);
		mv_pp2x_cls_sw_flow_dump(&fe);
		mv_pp2x_cls_hw_flow_hit_get(cpu_slot, index, NULL);
		printf("\n");
	}

	return 0;
}

int mv_pp2x_cls_hw_flow_hits_dump(uintptr_t cpu_slot)
{
	struct mv_pp2x_cls_flow_entry fe;
	int index;
	u32 cnt = 0;

	for (index = 0; index < MVPP2_CLS_FLOWS_TBL_SIZE; index++) {
		mv_pp2x_cls_hw_flow_hit_get(cpu_slot, index, &cnt);
		if (cnt != 0) {
			mv_pp2x_cls_hw_flow_read(cpu_slot, index, &fe);
			mv_pp2x_cls_sw_flow_dump(&fe);
			printf("HITS = %d\n", cnt);
		}
	}

	return 0;
}

int mv_pp2x_cls_hw_lkp_hits_dump(uintptr_t cpu_slot)
{
	int index, way = 0;
	u32 cnt;

	printf("< ID  WAY >:	HITS\n");
	for (index = 0; index < MVPP2_CLS_LKP_TBL_SIZE; index++) {
		mv_pp2x_cls_hw_lkp_hit_get(cpu_slot, index, &cnt);
		if (cnt != 0)
			printf(" 0x%2.2x  %1.1d\t0x%8.8x\n", index, way, cnt);
	}
	return 0;
}

int mv_pp2x_cls_hw_lkp_dump(uintptr_t cpu_slot)
{
	int index, way = 0, rxq, en, flow, mod;
	u32 uint32bit;
	struct mv_pp2x_cls_lookup_entry le;

	printf("\n ID :	RXQ	EN	FLOW	MODE_BASE  HITS\n");
	for (index = 0; index < MVPP2_CLS_LKP_TBL_SIZE; index++) {
		mv_pp2x_cls_hw_lkp_read(cpu_slot, index, way, &le);
		printf(" 0x%2.2x \t", le.lkpid);
		mv_pp2x_cls_sw_lkp_rxq_get(&le, &rxq);
		printf("0x%2.2x\t", rxq);
		mv_pp2x_cls_sw_lkp_en_get(&le, &en);
		printf("%1.1d\t", en);
		mv_pp2x_cls_sw_lkp_flow_get(&le, &flow);
		printf("0x%3.3x\t", flow);
		mv_pp2x_cls_sw_lkp_mod_get(&le, &mod);
		printf(" 0x%2.2x\t", mod);
		mv_pp2x_cls_hw_lkp_hit_get(cpu_slot, index, &uint32bit);
		printf(" 0x%8.8x\n", 0);
		printf("\n");
	}
	return 0;
}

/* Operations on flow entry */
int mv_pp2x_cls_sw_flow_hek_num_set(struct mv_pp2x_cls_flow_entry *fe,
				    int num_of_fields)
{
	if (mv_pp2x_ptr_validate(fe) == MV_ERROR)
		return MV_ERROR;

	if (mv_pp2x_range_validate(num_of_fields, 0,
				   MVPP2_CLS_FLOWS_TBL_FIELDS_MAX) == MV_ERROR)
		return MV_ERROR;

	fe->data[1] &= ~MVPP2_FLOW_FIELDS_NUM_MASK;
	fe->data[1] |= (num_of_fields << MVPP2_FLOW_FIELDS_NUM);

	return 0;
}

int mv_pp2x_cls_sw_flow_hek_set(struct mv_pp2x_cls_flow_entry *fe,
				int field_index, int field_id)
{
	int num_of_fields;

	/* get current num_of_fields */
	num_of_fields = ((fe->data[1] &
			  MVPP2_FLOW_FIELDS_NUM_MASK) >> MVPP2_FLOW_FIELDS_NUM);

	if (num_of_fields < (field_index + 1)) {
		pr_debug("%s:num of heks=%d ,idx(%d) out of range\n",
			__func__, num_of_fields, field_index);
		return -1;
	}

	fe->data[2] &= ~MVPP2_FLOW_FIELD_MASK(field_index);
	fe->data[2] |= (field_id << MVPP2_FLOW_FIELD_ID(field_index));

	return 0;
}

static void mv_pp2x_cls_sw_flow_eng_set(struct mv_pp2x_cls_flow_entry *fe,
					int engine, int is_last)
{
	fe->data[0] &= ~MVPP2_FLOW_LAST_MASK;
	fe->data[0] &= ~MVPP2_FLOW_ENGINE_MASK;

	fe->data[0] |= is_last;
	fe->data[0] |= (engine << MVPP2_FLOW_ENGINE);
	fe->data[0] |= MVPP2_FLOW_PORT_ID_SEL_MASK;
}

void mv_pp2x_cls_lookup_read(struct pp2_hw *hw, int lkpid, int way,
			     struct mv_pp2x_cls_lookup_entry *le)
{
	unsigned int val = 0;

	/* write index reg */
	val = (way << MVPP2_CLS_LKP_INDEX_WAY_OFFS) | lkpid;
	pp2_reg_write(hw->base[0].va, MVPP2_CLS_LKP_INDEX_REG, val);
	le->way = way;
	le->lkpid = lkpid;
	le->data = pp2_reg_read(hw->base[0].va, MVPP2_CLS_LKP_TBL_REG);
}

/* Set prs flow for the port */
int mv_pp2x_prs_def_flow(struct pp2_port *port)
{
	struct mv_pp2x_prs_entry *pe;
	struct pp2_hw *hw = &port->parent->hw;
	int tid;

	pe = mv_pp2x_prs_flow_find(hw, port->id, 0, 0);

	/* Such entry not exist */
	if (!pe) {
		/* Go through the all entires from last to first */
		tid = mv_pp2x_prs_tcam_first_free(hw,
						  MVPP2_PE_LAST_FREE_TID,
						  MVPP2_PE_FIRST_FREE_TID);
		if (tid < 0)
			return tid;

		pe = kcalloc(1, sizeof(*pe), GFP_KERNEL);
		if (!pe)
			return -ENOMEM;

		mv_pp2x_prs_tcam_lu_set(pe, MVPP2_PRS_LU_FLOWS);
		pe->index = tid;

		/* Set flow ID */
		mv_pp2x_prs_sram_ai_update(pe, port->id,
					   MVPP2_PRS_FLOW_ID_MASK);
		mv_pp2x_prs_sram_bits_set(pe, MVPP2_PRS_SRAM_LU_DONE_BIT, 1);

		/* Update shadow table */
		mv_pp2x_prs_shadow_set(hw, pe->index, MVPP2_PRS_LU_FLOWS);

		/*pr_crit("mv_pp2x_prs_def_flow: index(%d) port->id\n",
		 * pe->index, port->id);
		 */
	}

	mv_pp2x_prs_tcam_port_map_set(pe, (1 << port->id));
	mv_pp2x_prs_hw_write(hw, pe);
	kfree(pe);

	return 0;
}

void mv_pp2x_cls_flow_port_add(struct pp2_hw *hw, int index, int port_id)
{
	u32 data;

	/* Write flow index */
	pp2_reg_write(hw->base[0].va, MVPP2_CLS_FLOW_INDEX_REG, index);
	/* Read first data with port info */
	data = pp2_reg_read(hw->base[0].va, MVPP2_CLS_FLOW_TBL0_REG);
	/* Add the port */
	data |= ((1 << port_id) << MVPP2_FLOW_PORT_ID);
	/* Update the register */
	pp2_reg_write(hw->base[0].va, MVPP2_CLS_FLOW_TBL0_REG, data);
}

void mv_pp2x_cls_flow_port_del(struct pp2_hw *hw, int index, int port_id)
{
	u32 data;

	/* Write flow index */
	pp2_reg_write(hw->base[0].va, MVPP2_CLS_FLOW_INDEX_REG, index);
	/* Read first data with port info */
	data = pp2_reg_read(hw->base[0].va, MVPP2_CLS_FLOW_TBL0_REG);
	/* Delete the port */
	data &= ~(((1 << port_id) << MVPP2_FLOW_PORT_ID));
	/* Update the register */
	pp2_reg_write(hw->base[0].va, MVPP2_CLS_FLOW_TBL0_REG, data);
}

/* The function prepare a temporary flow table for lkpid flow,
 * in order to change the original one
 */
void mv_pp2x_cls_flow_tbl_temp_copy(struct pp2_hw *hw, int lkpid,
				    int *temp_flow_idx)
{
	struct mv_pp2x_cls_flow_entry fe;
	int index = lkpid - MVPP2_PRS_FL_START;
	int flow_start = hw->cls_shadow->flow_free_start;
	struct mv_pp2x_cls_flow_info *flow_info;

	flow_info = &hw->cls_shadow->flow_info[index];

	if (flow_info->flow_entry_dflt) {
		mv_pp2x_cls_flow_read(hw, flow_info->flow_entry_dflt, &fe);
		fe.index = flow_start++;
		mv_pp2x_cls_flow_write(hw, &fe);
	}
	if (flow_info->flow_entry_vlan) {
		mv_pp2x_cls_flow_read(hw, flow_info->flow_entry_vlan, &fe);
		fe.index = flow_start++;
		mv_pp2x_cls_flow_write(hw, &fe);
	}
	if (flow_info->flow_entry_dscp) {
		mv_pp2x_cls_flow_read(hw, flow_info->flow_entry_dscp, &fe);
		fe.index = flow_start++;
		mv_pp2x_cls_flow_write(hw, &fe);
	}
	if (flow_info->flow_entry_rss1) {
		mv_pp2x_cls_flow_read(hw, flow_info->flow_entry_rss1, &fe);
		fe.index = flow_start++;
		mv_pp2x_cls_flow_write(hw, &fe);
	}
	if (flow_info->flow_entry_rss2) {
		mv_pp2x_cls_flow_read(hw, flow_info->flow_entry_rss2, &fe);
		fe.index = flow_start++;
		mv_pp2x_cls_flow_write(hw, &fe);
	}

	*temp_flow_idx = hw->cls_shadow->flow_free_start;
}

/* To init flow table waccording to different flow */
static inline void mv_pp2x_cls_flow_cos(struct pp2_hw *hw,
					struct mv_pp2x_cls_flow_entry *fe,
					int lkpid, int cos_type)
{
	int hek_num, field_id, lkp_type, is_last;
	int entry_idx = hw->cls_shadow->flow_free_start;

	switch (cos_type) {
	case MVPP2_COS_TYPE_VLAN:
		lkp_type = MVPP2_CLS_LKP_VLAN_PRI;
		break;
	case MVPP2_COS_TYPE_DSCP:
		lkp_type = MVPP2_CLS_LKP_DSCP_PRI;
		break;
	default:
		lkp_type = MVPP2_CLS_LKP_DEFAULT;
		break;
	}
	hek_num = 0;
	if ((lkpid == MVPP2_PRS_FL_NON_IP_UNTAG &&
	     cos_type == MVPP2_COS_TYPE_DEF) ||
	    (lkpid == MVPP2_PRS_FL_NON_IP_TAG &&
	     cos_type == MVPP2_COS_TYPE_VLAN))
		is_last = 1;
	else
		is_last = 0;

	/* Set SW */
	memset(fe, 0, sizeof(struct mv_pp2x_cls_flow_entry));
	mv_pp2x_cls_sw_flow_hek_num_set(fe, hek_num);
	if (hek_num)
		mv_pp2x_cls_sw_flow_hek_set(fe, 0, field_id);
	mv_pp2x_cls_sw_flow_eng_set(fe, MVPP2_CLS_ENGINE_C2, is_last);
	mv_pp2x_cls_sw_flow_extra_set(fe, lkp_type, MVPP2_CLS_FL_COS_PRI);
	fe->index = entry_idx;

	/* Write HW */
	mv_pp2x_cls_flow_write(hw, fe);

	/* Update Shadow */
	if (cos_type == MVPP2_COS_TYPE_DEF)
		hw->cls_shadow->flow_info[lkpid -
					  MVPP2_PRS_FL_START].flow_entry_dflt =
		    entry_idx;
	else if (cos_type == MVPP2_COS_TYPE_VLAN)
		hw->cls_shadow->flow_info[lkpid -
					  MVPP2_PRS_FL_START].flow_entry_vlan =
		    entry_idx;
	else
		hw->cls_shadow->flow_info[lkpid -
					  MVPP2_PRS_FL_START].flow_entry_dscp =
		    entry_idx;

	/* Update first available flow entry */
	hw->cls_shadow->flow_free_start++;
}

/* Init flow entry for RSS hash in PP22 */
static inline void mv_pp2x_cls_flow_rss_hash(struct pp2_hw *hw,
					     struct mv_pp2x_cls_flow_entry *fe,
					     int lkpid, int rss_mode)
{
	int field_id[4] = { 0 };
	int entry_idx = hw->cls_shadow->flow_free_start;
	int lkpid_attr = mv_pp2x_prs_flow_id_attr_get(lkpid);

	/* IP4 packet */
	if (lkpid_attr & MVPP2_PRS_FL_ATTR_IP4_BIT) {
		field_id[0] = MVPP2_CLS_FIELD_IP4SA;
		field_id[1] = MVPP2_CLS_FIELD_IP4DA;
	} else if (lkpid_attr & MVPP2_PRS_FL_ATTR_IP6_BIT) {
		field_id[0] = MVPP2_CLS_FIELD_IP6SA;
		field_id[1] = MVPP2_CLS_FIELD_IP6DA;
	}
	/* L4 port */
	field_id[2] = MVPP2_CLS_FIELD_L4SIP;
	field_id[3] = MVPP2_CLS_FIELD_L4DIP;

	/* Set SW */
	memset(fe, 0, sizeof(struct mv_pp2x_cls_flow_entry));
	if (rss_mode == MVPP2_RSS_HASH_2T) {
		mv_pp2x_cls_sw_flow_hek_num_set(fe, 2);
		mv_pp2x_cls_sw_flow_eng_set(fe, MVPP2_CLS_ENGINE_C3HA, 1);
		mv_pp2x_cls_sw_flow_hek_set(fe, 0, field_id[0]);
		mv_pp2x_cls_sw_flow_hek_set(fe, 1, field_id[1]);
	} else {
		mv_pp2x_cls_sw_flow_hek_num_set(fe, 4);
		mv_pp2x_cls_sw_flow_hek_set(fe, 0, field_id[0]);
		mv_pp2x_cls_sw_flow_hek_set(fe, 1, field_id[1]);
		mv_pp2x_cls_sw_flow_hek_set(fe, 2, field_id[2]);
		mv_pp2x_cls_sw_flow_hek_set(fe, 3, field_id[3]);
		mv_pp2x_cls_sw_flow_eng_set(fe, MVPP2_CLS_ENGINE_C3HB, 1);
	}
	mv_pp2x_cls_sw_flow_extra_set(fe,
				      MVPP2_CLS_LKP_HASH, MVPP2_CLS_FL_RSS_PRI);
	fe->index = entry_idx;

	/* Update last for UDP NF flow */
	if ((lkpid_attr & MVPP2_PRS_FL_ATTR_UDP_BIT) &&
	    !(lkpid_attr & MVPP2_PRS_FL_ATTR_FRAG_BIT)) {
		if (!hw->cls_shadow->flow_info[lkpid -
					       MVPP2_PRS_FL_START].
		    flow_entry_rss1) {
			if (rss_mode == MVPP2_RSS_HASH_2T)
				mv_pp2x_cls_sw_flow_eng_set(fe,
							    MVPP2_CLS_ENGINE_C3HA, 0);
			else
				mv_pp2x_cls_sw_flow_eng_set(fe,
							    MVPP2_CLS_ENGINE_C3HB, 0);
		}
	}

	/* Write HW */
	mv_pp2x_cls_flow_write(hw, fe);

	/* Update Shadow */
	if (hw->cls_shadow->flow_info[lkpid -
				      MVPP2_PRS_FL_START].flow_entry_rss1 == 0)
		hw->cls_shadow->flow_info[lkpid -
					  MVPP2_PRS_FL_START].flow_entry_rss1 =
		    entry_idx;
	else
		hw->cls_shadow->flow_info[lkpid -
					  MVPP2_PRS_FL_START].flow_entry_rss2 =
		    entry_idx;

	/* Update first available flow entry */
	hw->cls_shadow->flow_free_start++;
}

/* Init cls flow table according to different flow id */
void mv_pp2x_cls_flow_tbl_config(struct pp2_hw *hw)
{
	int lkpid, rss_mode, lkpid_attr;
	struct mv_pp2x_cls_flow_entry fe;

	for (lkpid = MVPP2_PRS_FL_START; lkpid < MVPP2_PRS_FL_LAST; lkpid++) {
		/* Get lookup id attribute */
		lkpid_attr = mv_pp2x_prs_flow_id_attr_get(lkpid);
		/* Default rss hash is based on 5T */
		rss_mode = MVPP2_RSS_HASH_5T;
		/* For frag packets or non-TCP&UDP, rss must be based on 2T */
		if ((lkpid_attr & MVPP2_PRS_FL_ATTR_FRAG_BIT) ||
		    !(lkpid_attr & (MVPP2_PRS_FL_ATTR_TCP_BIT |
				    MVPP2_PRS_FL_ATTR_UDP_BIT)))
			rss_mode = MVPP2_RSS_HASH_2T;

		/* For untagged IP packets, only need default
		 * rule and dscp rule
		 */
		if ((lkpid_attr & (MVPP2_PRS_FL_ATTR_IP4_BIT |
				   MVPP2_PRS_FL_ATTR_IP6_BIT)) &&
		    (!(lkpid_attr & MVPP2_PRS_FL_ATTR_VLAN_BIT))) {
			/* Default rule */
			mv_pp2x_cls_flow_cos(hw, &fe, lkpid,
					     MVPP2_COS_TYPE_DEF);
			/* DSCP rule */
			mv_pp2x_cls_flow_cos(hw, &fe, lkpid,
					     MVPP2_COS_TYPE_DSCP);
			/* RSS hash rule */
			if ((!(lkpid_attr & MVPP2_PRS_FL_ATTR_FRAG_BIT)) &&
			    (lkpid_attr & MVPP2_PRS_FL_ATTR_UDP_BIT)) {
				/* RSS hash rules for UDP rss mode update */
				mv_pp2x_cls_flow_rss_hash(hw, &fe, lkpid,
							  MVPP2_RSS_HASH_2T);
				mv_pp2x_cls_flow_rss_hash(hw, &fe, lkpid,
							  MVPP2_RSS_HASH_5T);
			} else {
				mv_pp2x_cls_flow_rss_hash(hw, &fe, lkpid,
							  rss_mode);
			}
		}

		/* For tagged IP packets, only need vlan rule and dscp rule */
		if ((lkpid_attr & (MVPP2_PRS_FL_ATTR_IP4_BIT |
				   MVPP2_PRS_FL_ATTR_IP6_BIT)) &&
		    (lkpid_attr & MVPP2_PRS_FL_ATTR_VLAN_BIT)) {
			/* VLAN rule */
			mv_pp2x_cls_flow_cos(hw, &fe, lkpid,
					     MVPP2_COS_TYPE_VLAN);
			/* DSCP rule */
			mv_pp2x_cls_flow_cos(hw, &fe, lkpid,
					     MVPP2_COS_TYPE_DSCP);
			/* RSS hash rule */
			if ((!(lkpid_attr & MVPP2_PRS_FL_ATTR_FRAG_BIT)) &&
			    (lkpid_attr & MVPP2_PRS_FL_ATTR_UDP_BIT)) {
				/* RSS hash rules for UDP rss mode update */
				mv_pp2x_cls_flow_rss_hash(hw, &fe, lkpid,
							  MVPP2_RSS_HASH_2T);
				mv_pp2x_cls_flow_rss_hash(hw, &fe, lkpid,
							  MVPP2_RSS_HASH_5T);
			} else {
				mv_pp2x_cls_flow_rss_hash(hw, &fe, lkpid,
							  rss_mode);
			}
		}

		/* For non-IP packets, only need default rule if untagged,
		 * vlan rule also needed if tagged
		 */
		if (!(lkpid_attr & (MVPP2_PRS_FL_ATTR_IP4_BIT |
				    MVPP2_PRS_FL_ATTR_IP6_BIT))) {
			/* Default rule */
			mv_pp2x_cls_flow_cos(hw, &fe, lkpid,
					     MVPP2_COS_TYPE_DEF);
			/* VLAN rule if tagged */
			if (lkpid_attr & MVPP2_PRS_FL_ATTR_VLAN_BIT)
				mv_pp2x_cls_flow_cos(hw, &fe, lkpid,
						     MVPP2_COS_TYPE_VLAN);
		}
	}
}

static inline uint8_t mv_pp2x_bound_cpu_first_rxq_calc(struct pp2_port
						       *port)
{
	u8 cos_width, bind_cpu;

	cos_width =
	    ilog2(roundup_pow_of_two
		  (port->parent->pp2_cfg.cos_cfg.num_cos_queues));
	bind_cpu = (port->parent->pp2_cfg.rx_cpu_map >> (4 * port->id)) & 0xF;

	return (port->first_rxq + (bind_cpu << cos_width));
}

u8 mv_pp2x_cosval_queue_map(struct pp2_port *port, uint8_t cos_value)
{
	int cos_width, cos_mask;

	cos_width = ilog2(roundup_pow_of_two(port->num_tcs));
	cos_mask = (1 << cos_width) - 1;
	return ((port->parent->pp2_cfg.cos_cfg.pri_map >> (cos_value * 4)) & cos_mask);
}

int mv_pp2x_cls_c2_qos_hw_write(struct pp2_hw *hw,
				struct mv_pp2x_cls_c2_qos_entry *qos)
{
	unsigned int reg_val = 0;

	if (!qos || qos->tbl_sel > MVPP2_QOS_TBL_SEL_DSCP)
		return -EINVAL;

	if (qos->tbl_sel == MVPP2_QOS_TBL_SEL_DSCP) {
		/*dscp */
		if (qos->tbl_id >= MVPP2_QOS_TBL_NUM_DSCP ||
		    qos->tbl_line >= MVPP2_QOS_TBL_LINE_NUM_DSCP)
			return -EINVAL;
	} else {
		/*pri */
		if (qos->tbl_id >= MVPP2_QOS_TBL_NUM_PRI ||
		    qos->tbl_line >= MVPP2_QOS_TBL_LINE_NUM_PRI)
			return -EINVAL;
	}
	/* write index reg */
	reg_val |= (qos->tbl_line << MVPP2_CLS2_DSCP_PRI_INDEX_LINE_OFF);
	reg_val |= (qos->tbl_sel << MVPP2_CLS2_DSCP_PRI_INDEX_SEL_OFF);
	reg_val |= (qos->tbl_id << MVPP2_CLS2_DSCP_PRI_INDEX_TBL_ID_OFF);
	pp2_reg_write(hw->base[0].va, MVPP2_CLS2_DSCP_PRI_INDEX_REG, reg_val);

	/* write data reg */
	pp2_reg_write(hw->base[0].va, MVPP2_CLS2_QOS_TBL_REG, qos->data);

	return 0;
}

int mv_pp2x_cls_c2_qos_queue_set(struct mv_pp2x_cls_c2_qos_entry *qos,
				 uint8_t queue)
{
	if (!qos || queue >= (1 << MVPP2_CLS2_QOS_TBL_QUEUENUM_BITS))
		return -EINVAL;

	qos->data &= ~MVPP2_CLS2_QOS_TBL_QUEUENUM_MASK;
	qos->data |= (((uint32_t)queue) << MVPP2_CLS2_QOS_TBL_QUEUENUM_OFF);
	return 0;
}

/* CoS API */

/* mv_pp2x_cos_classifier_set
*  -- The API supplies interface to config cos classifier:
*     0: cos based on vlan pri;
*     1: cos based on dscp;
*     2: cos based on vlan for tagged packets,
*		and based on dscp for untagged IP packets;
*     3: cos based on dscp for IP packets, and based on vlan for non-IP packets;
*/
/* Fill the qos table with queue */
static void mv_pp2x_cls_c2_qos_tbl_fill(struct pp2_port *port,
					u8 tbl_sel, uint8_t start_queue)
{
	struct mv_pp2x_cls_c2_qos_entry qos_entry;
	u32 pri, line_num;
	u8 cos_value, cos_queue, queue;

	if (tbl_sel == MVPP2_QOS_TBL_SEL_PRI)
		line_num = MVPP2_QOS_TBL_LINE_NUM_PRI;
	else
		line_num = MVPP2_QOS_TBL_LINE_NUM_DSCP;

	memset(&qos_entry, 0, sizeof(struct mv_pp2x_cls_c2_qos_entry));
	qos_entry.tbl_id = port->id;
	qos_entry.tbl_sel = tbl_sel;

	/* Fill the QoS dscp/pbit table */
	for (pri = 0; pri < line_num; pri++) {
		/* cos_value equal to dscp/8 or pbit value */
		cos_value = ((tbl_sel == MVPP2_QOS_TBL_SEL_PRI) ?
			     pri : (pri / 8));
		/* each nibble of pri_map stands for a cos-value,
		 * nibble value is the queue
		 */
		cos_queue = mv_pp2x_cosval_queue_map(port, cos_value);
		qos_entry.tbl_line = pri;
		/* map cos queue to physical queue */
		/* Physical queue contains 2 parts: port ID and CPU ID,
		 * CPU ID will be used in RSS
		 */
		queue = start_queue + cos_queue;
		mv_pp2x_cls_c2_qos_queue_set(&qos_entry, queue);
		mv_pp2x_cls_c2_qos_hw_write(&port->parent->hw, &qos_entry);
	}
}

int mv_pp2x_cls_c2_qos_tbl_set(struct mv_pp2x_cls_c2_entry *c2,
			       int tbl_id, int tbl_sel)
{
	if (!c2 || tbl_sel > 1)
		return -EINVAL;

	if (tbl_sel == 1) {
		/*dscp */
		if (tbl_id >= MVPP2_QOS_TBL_NUM_DSCP)
			return -EINVAL;
	} else {
		/*pri */
		if (tbl_id >= MVPP2_QOS_TBL_NUM_PRI)
			return -EINVAL;
	}
	c2->sram.regs.action_tbl = (tbl_id <<
				    MVPP2_CLS2_ACT_DATA_TBL_ID_OFF) |
	    (tbl_sel << MVPP2_CLS2_ACT_DATA_TBL_SEL_OFF);

	return 0;
}

int mv_pp2x_cls_c2_color_set(struct mv_pp2x_cls_c2_entry *c2, int cmd, int from)
{
	if (!c2 || cmd > MVPP2_COLOR_ACTION_TYPE_RED_LOCK)
		return -EINVAL;

	c2->sram.regs.actions &= ~MVPP2_CLS2_ACT_COLOR_MASK;
	c2->sram.regs.actions |= (cmd << MVPP2_CLS2_ACT_COLOR_OFF);

	if (from == 1)
		c2->sram.regs.action_tbl |=
		    (1 << MVPP2_CLS2_ACT_DATA_TBL_COLOR_OFF);
	else
		c2->sram.regs.action_tbl &=
		    ~(1 << MVPP2_CLS2_ACT_DATA_TBL_COLOR_OFF);

	return 0;
}

int mv_pp2x_cls_c2_prio_set(struct mv_pp2x_cls_c2_entry *c2, int cmd,
			    int prio, int from)
{
	if (!c2 || cmd > MVPP2_ACTION_TYPE_UPDT_LOCK ||
	    prio >= MVPP2_QOS_TBL_LINE_NUM_PRI)
		return -EINVAL;

	/*set command */
	c2->sram.regs.actions &= ~MVPP2_CLS2_ACT_PRI_MASK;
	c2->sram.regs.actions |= (cmd << MVPP2_CLS2_ACT_PRI_OFF);

	/*set modify priority value */
	c2->sram.regs.qos_attr &= ~MVPP2_CLS2_ACT_QOS_ATTR_PRI_MASK;
	c2->sram.regs.qos_attr |= ((prio << MVPP2_CLS2_ACT_QOS_ATTR_PRI_OFF) &
				   MVPP2_CLS2_ACT_QOS_ATTR_PRI_MASK);

	if (from == 1)
		c2->sram.regs.action_tbl |=
		    (1 << MVPP2_CLS2_ACT_DATA_TBL_PRI_DSCP_OFF);
	else
		c2->sram.regs.action_tbl &=
		    ~(1 << MVPP2_CLS2_ACT_DATA_TBL_PRI_DSCP_OFF);

	return 0;
}

int mv_pp2x_cls_c2_dscp_set(struct mv_pp2x_cls_c2_entry *c2,
			    int cmd, int dscp, int from)
{
	if (!c2 || cmd > MVPP2_ACTION_TYPE_UPDT_LOCK ||
	    dscp >= MVPP2_QOS_TBL_LINE_NUM_DSCP)
		return -EINVAL;

	/*set command */
	c2->sram.regs.actions &= ~MVPP2_CLS2_ACT_DSCP_MASK;
	c2->sram.regs.actions |= (cmd << MVPP2_CLS2_ACT_DSCP_OFF);

	/*set modify DSCP value */
	c2->sram.regs.qos_attr &= ~MVPP2_CLS2_ACT_QOS_ATTR_DSCP_MASK;
	c2->sram.regs.qos_attr |= ((dscp <<
				    MVPP2_CLS2_ACT_QOS_ATTR_DSCP_OFF) &
				   MVPP2_CLS2_ACT_QOS_ATTR_DSCP_MASK);

	if (from == 1)
		c2->sram.regs.action_tbl |=
		    (1 << MVPP2_CLS2_ACT_DATA_TBL_PRI_DSCP_OFF);
	else
		c2->sram.regs.action_tbl &=
		    ~(1 << MVPP2_CLS2_ACT_DATA_TBL_PRI_DSCP_OFF);

	return 0;
}

int mv_pp2x_cls_c2_queue_low_set(struct mv_pp2x_cls_c2_entry *c2,
				 int cmd, int queue, int from)
{
	if (!c2 || cmd > MVPP2_ACTION_TYPE_UPDT_LOCK ||
	    queue >= (1 << MVPP2_CLS2_ACT_QOS_ATTR_QL_BITS))
		return -EINVAL;

	/*set command */
	c2->sram.regs.actions &= ~MVPP2_CLS2_ACT_QL_MASK;
	c2->sram.regs.actions |= (cmd << MVPP2_CLS2_ACT_QL_OFF);

	/*set modify Low queue value */
	c2->sram.regs.qos_attr &= ~MVPP2_CLS2_ACT_QOS_ATTR_QL_MASK;
	c2->sram.regs.qos_attr |= ((queue <<
				    MVPP2_CLS2_ACT_QOS_ATTR_QL_OFF) &
				   MVPP2_CLS2_ACT_QOS_ATTR_QL_MASK);

	if (from == 1)
		c2->sram.regs.action_tbl |=
		    (1 << MVPP2_CLS2_ACT_DATA_TBL_LOW_Q_OFF);
	else
		c2->sram.regs.action_tbl &=
		    ~(1 << MVPP2_CLS2_ACT_DATA_TBL_LOW_Q_OFF);

	return 0;
}

int mv_pp2x_cls_c2_queue_high_set(struct mv_pp2x_cls_c2_entry *c2,
				  int cmd, int queue, int from)
{
	if (!c2 || cmd > MVPP2_ACTION_TYPE_UPDT_LOCK ||
	    queue >= (1 << MVPP2_CLS2_ACT_QOS_ATTR_QH_BITS))
		return -EINVAL;

	/*set command */
	c2->sram.regs.actions &= ~MVPP2_CLS2_ACT_QH_MASK;
	c2->sram.regs.actions |= (cmd << MVPP2_CLS2_ACT_QH_OFF);

	/*set modify High queue value */
	c2->sram.regs.qos_attr &= ~MVPP2_CLS2_ACT_QOS_ATTR_QH_MASK;
	c2->sram.regs.qos_attr |= ((queue <<
				    MVPP2_CLS2_ACT_QOS_ATTR_QH_OFF) &
				   MVPP2_CLS2_ACT_QOS_ATTR_QH_MASK);

	if (from == 1)
		c2->sram.regs.action_tbl |=
		    (1 << MVPP2_CLS2_ACT_DATA_TBL_HIGH_Q_OFF);
	else
		c2->sram.regs.action_tbl &=
		    ~(1 << MVPP2_CLS2_ACT_DATA_TBL_HIGH_Q_OFF);

	return 0;
}

int mv_pp2x_cls_c2_forward_set(struct mv_pp2x_cls_c2_entry *c2, int cmd)
{
	if (!c2 || cmd > MVPP2_ACTION_TYPE_UPDT_LOCK)
		return -EINVAL;

	c2->sram.regs.actions &= ~MVPP2_CLS2_ACT_FRWD_MASK;
	c2->sram.regs.actions |= (cmd << MVPP2_CLS2_ACT_FRWD_OFF);

	return 0;
}

int mv_pp2x_cls_c2_rss_set(struct mv_pp2x_cls_c2_entry *c2, int cmd, int rss_en)
{
	if (!c2 || cmd > MVPP2_ACTION_TYPE_UPDT_LOCK || rss_en >=
	    (1 << MVPP2_CLS2_ACT_DUP_ATTR_RSSEN_BITS))
		return -EINVAL;

	c2->sram.regs.actions &= ~MVPP2_CLS2_ACT_RSS_MASK;
	c2->sram.regs.actions |= (cmd << MVPP2_CLS2_ACT_RSS_OFF);

	c2->sram.regs.rss_attr &= ~MVPP2_CLS2_ACT_DUP_ATTR_RSSEN_MASK;
	c2->sram.regs.rss_attr |= (rss_en << MVPP2_CLS2_ACT_DUP_ATTR_RSSEN_OFF);

	return 0;
}

int mv_pp2x_cls_c2_FLOWID_en(struct mv_pp2x_cls_c2_entry *c2, int flow_id_en)
{
	if (!c2)
		return -EINVAL;

	/*set Flow ID enable or disable */
	if (flow_id_en)
		c2->sram.regs.actions |= (1 << MVPP2_CLS2_ACT_FLD_EN_OFF);
	else
		c2->sram.regs.actions &= ~(1 << MVPP2_CLS2_ACT_FLD_EN_OFF);

	return 0;
}

int mv_pp2x_cls_c2_tcam_byte_set(struct mv_pp2x_cls_c2_entry *c2,
				 unsigned int offs, unsigned char byte,
				 unsigned char enable)
{
	if (!c2 || offs >= MVPP2_CLS_C2_TCAM_DATA_BYTES)
		return -EINVAL;

	c2->tcam.bytes[TCAM_DATA_BYTE(offs)] = byte;
	c2->tcam.bytes[TCAM_DATA_MASK(offs)] = enable;

	return 0;
}

/* C2 rule and Qos table */
int mv_pp2x_cls_c2_hw_write(struct pp2_hw *hw, int index,
			    struct mv_pp2x_cls_c2_entry *c2)
{
	int tcm_idx;

	if (!c2 || index >= MVPP2_CLS_C2_TCAM_SIZE)
		return -EINVAL;

	c2->index = index;

	/* write index reg */
	pp2_reg_write(hw->base[0].va, MVPP2_CLS2_TCAM_IDX_REG, index);

	/* write valid bit */
	c2->inv = 0;
	pp2_reg_write(hw->base[0].va, MVPP2_CLS2_TCAM_INV_REG,
		      ((c2->inv) << MVPP2_CLS2_TCAM_INV_INVALID_OFF));

	for (tcm_idx = 0; tcm_idx < MVPP2_CLS_C2_TCAM_WORDS; tcm_idx++)
		pp2_reg_write(hw->base[0].va, MVPP2_CLS2_TCAM_DATA_REG(tcm_idx),
			      c2->tcam.words[tcm_idx]);

	/* write action_tbl CLSC2_ACT_DATA */
	pp2_reg_write(hw->base[0].va, MVPP2_CLS2_ACT_DATA_REG,
		      c2->sram.regs.action_tbl);

	/* write actions CLSC2_ACT */
	pp2_reg_write(hw->base[0].va, MVPP2_CLS2_ACT_REG, c2->sram.regs.actions);

	/* write qos_attr CLSC2_ATTR0 */
	pp2_reg_write(hw->base[0].va, MVPP2_CLS2_ACT_QOS_ATTR_REG,
		      c2->sram.regs.qos_attr);

	/* write hwf_attr CLSC2_ATTR1 */
	pp2_reg_write(hw->base[0].va, MVPP2_CLS2_ACT_HWF_ATTR_REG,
		      c2->sram.regs.hwf_attr);

	/* write rss_attr CLSC2_ATTR2 */
	pp2_reg_write(hw->base[0].va, MVPP2_CLS2_ACT_DUP_ATTR_REG,
		      c2->sram.regs.rss_attr);

	return 0;
}

static int mv_pp2x_c2_tcam_set(struct pp2_hw *hw,
			       struct mv_pp2x_c2_add_entry *c2_add_entry,
			       unsigned int c2_hw_idx)
{
	int ret_code;
	struct mv_pp2x_cls_c2_entry c2_entry;
	int hek_offs;
	unsigned char hek_byte[MVPP2_CLS_C2_HEK_OFF_MAX],
	    hek_byte_mask[MVPP2_CLS_C2_HEK_OFF_MAX];

	if (!c2_add_entry || !hw || c2_hw_idx >= MVPP2_CLS_C2_TCAM_SIZE)
		return -EINVAL;

	/* Clear C2 sw data */
	memset(&c2_entry, 0, sizeof(struct mv_pp2x_cls_c2_entry));

	/* Set QOS table, selection and ID */
	ret_code = mv_pp2x_cls_c2_qos_tbl_set(&c2_entry,
					      c2_add_entry->qos_info.
					      qos_tbl_index,
					      c2_add_entry->qos_info.
					      qos_tbl_type);
	if (ret_code)
		return ret_code;

	/* Set color, cmd and source */
	ret_code = mv_pp2x_cls_c2_color_set(&c2_entry,
					    c2_add_entry->action.color_act,
					    c2_add_entry->qos_info.color_src);
	if (ret_code)
		return ret_code;

	/* Set priority(pbit), cmd, value(not from qos table) and source */
	ret_code = mv_pp2x_cls_c2_prio_set(&c2_entry,
					   c2_add_entry->action.pri_act,
					   c2_add_entry->qos_value.pri,
					   c2_add_entry->qos_info.pri_dscp_src);
	if (ret_code)
		return ret_code;

	/* Set DSCP, cmd, value(not from qos table) and source */
	ret_code = mv_pp2x_cls_c2_dscp_set(&c2_entry,
					   c2_add_entry->action.dscp_act,
					   c2_add_entry->qos_value.dscp,
					   c2_add_entry->qos_info.pri_dscp_src);
	if (ret_code)
		return ret_code;

	/* Set queue low, cmd, value, and source */
	ret_code = mv_pp2x_cls_c2_queue_low_set(&c2_entry,
						c2_add_entry->action.q_low_act,
						c2_add_entry->qos_value.q_low,
						c2_add_entry->qos_info.
						q_low_src);
	if (ret_code)
		return ret_code;

	/* Set queue high, cmd, value and source */
	ret_code = mv_pp2x_cls_c2_queue_high_set(&c2_entry,
						 c2_add_entry->action.
						 q_high_act,
						 c2_add_entry->qos_value.q_high,
						 c2_add_entry->qos_info.
						 q_high_src);
	if (ret_code)
		return ret_code;

	/* Set forward */
	ret_code = mv_pp2x_cls_c2_forward_set(&c2_entry,
					      c2_add_entry->action.frwd_act);
	if (ret_code)
		return ret_code;

	/* Set RSS */
	ret_code = mv_pp2x_cls_c2_rss_set(&c2_entry,
					  c2_add_entry->action.rss_act,
					  c2_add_entry->rss_en);
	if (ret_code)
		return ret_code;

	/* Set flow_id(not for multicast) */
	ret_code = mv_pp2x_cls_c2_FLOWID_en(&c2_entry,
					    c2_add_entry->action.flowid_act);
	if (ret_code)
		return ret_code;

	/* Set C2 HEK */
	memset(hek_byte, 0, MVPP2_CLS_C2_HEK_OFF_MAX);
	memset(hek_byte_mask, 0, MVPP2_CLS_C2_HEK_OFF_MAX);

	/* HEK offs 8, lookup type, port type */
	hek_byte[MVPP2_CLS_C2_HEK_OFF_LKP_PORT_TYPE] =
	    (c2_add_entry->port.port_type <<
	     MVPP2_CLS_C2_HEK_PORT_TYPE_OFFS) |
	    (c2_add_entry->lkp_type << MVPP2_CLS_C2_HEK_LKP_TYPE_OFFS);
	hek_byte_mask[MVPP2_CLS_C2_HEK_OFF_LKP_PORT_TYPE] =
	    MVPP2_CLS_C2_HEK_PORT_TYPE_MASK |
	    ((c2_add_entry->lkp_type_mask <<
	      MVPP2_CLS_C2_HEK_LKP_TYPE_OFFS) & MVPP2_CLS_C2_HEK_LKP_TYPE_MASK);
	/* HEK offs 9, port ID */
	hek_byte[MVPP2_CLS_C2_HEK_OFF_PORT_ID] = c2_add_entry->port.port_value;
	hek_byte_mask[MVPP2_CLS_C2_HEK_OFF_PORT_ID] =
	    c2_add_entry->port.port_mask;

	for (hek_offs = MVPP2_CLS_C2_HEK_OFF_PORT_ID; hek_offs >=
	     MVPP2_CLS_C2_HEK_OFF_BYTE0; hek_offs--) {
		ret_code = mv_pp2x_cls_c2_tcam_byte_set(&c2_entry,
							hek_offs,
							hek_byte[hek_offs],
							hek_byte_mask
							[hek_offs]);
		if (ret_code)
			return ret_code;
	}

	/* Write C2 entry data to HW */
	ret_code = mv_pp2x_cls_c2_hw_write(hw, c2_hw_idx, &c2_entry);
	if (ret_code)
		return ret_code;

	return 0;
}

static int mv_pp2x_c2_rule_add(struct pp2_port *port,
			       struct mv_pp2x_c2_add_entry *c2_add_entry)
{
	int ret, lkp_type, c2_index = 0;
	bool first_free_update = false;
	struct mv_pp2x_c2_rule_idx *rule_idx;

	rule_idx = &port->parent->hw.c2_shadow->rule_idx_info[port->id];

	if (!port || !c2_add_entry)
		return -EINVAL;

	lkp_type = c2_add_entry->lkp_type;
	/* Write rule in C2 TCAM */
	if (lkp_type == MVPP2_CLS_LKP_VLAN_PRI) {
		if (rule_idx->vlan_pri_idx == MVPP2_CLS_C2_TCAM_SIZE) {
			/* If the C2 rule is new, apply a free c2 rule index */
			c2_index = port->parent->hw.c2_shadow->c2_tcam_free_start;
			first_free_update = true;
		} else {
			/* If the C2 rule is exist one,
			 * take the C2 index from shadow
			 */
			c2_index = rule_idx->vlan_pri_idx;
			first_free_update = false;
		}
	} else if (lkp_type == MVPP2_CLS_LKP_DSCP_PRI) {
		if (rule_idx->dscp_pri_idx == MVPP2_CLS_C2_TCAM_SIZE) {
			c2_index = port->parent->hw.c2_shadow->c2_tcam_free_start;
			first_free_update = true;
		} else {
			c2_index = rule_idx->dscp_pri_idx;
			first_free_update = false;
		}
	} else if (lkp_type == MVPP2_CLS_LKP_DEFAULT) {
		if (rule_idx->default_rule_idx == MVPP2_CLS_C2_TCAM_SIZE) {
			c2_index = port->parent->hw.c2_shadow->c2_tcam_free_start;
			first_free_update = true;
		} else {
			c2_index = rule_idx->default_rule_idx;
			first_free_update = false;
		}
	} else {
		return -EINVAL;
	}

	/* Write C2 TCAM HW */
	ret = mv_pp2x_c2_tcam_set(&port->parent->hw, c2_add_entry, c2_index);
	if (ret)
		return ret;

	/* Update first free rule */
	if (first_free_update)
		port->parent->hw.c2_shadow->c2_tcam_free_start++;

	/* Update shadow */
	if (lkp_type == MVPP2_CLS_LKP_VLAN_PRI)
		rule_idx->vlan_pri_idx = c2_index;
	else if (lkp_type == MVPP2_CLS_LKP_DSCP_PRI)
		rule_idx->dscp_pri_idx = c2_index;
	else if (lkp_type == MVPP2_CLS_LKP_DEFAULT)
		rule_idx->default_rule_idx = c2_index;

	return 0;
}

/* C2 rule set */
int mv_pp2x_cls_c2_rule_set(struct pp2_port *port, uint8_t start_queue)
{
	struct mv_pp2x_c2_add_entry c2_init_entry;
	int ret;
	u8 cos_value, cos_queue, queue, lkp_type;

	/* QoS of pbit rule */
	for (lkp_type = MVPP2_CLS_LKP_VLAN_PRI; lkp_type <=
	     MVPP2_CLS_LKP_DEFAULT; lkp_type++) {
		memset(&c2_init_entry, 0, sizeof(struct mv_pp2x_c2_add_entry));

		/* Port info */
		c2_init_entry.port.port_type = MVPP2_SRC_PORT_TYPE_PHY;
		c2_init_entry.port.port_value = (1 << port->id);
		c2_init_entry.port.port_mask = 0xff;
		/* Lookup type */
		c2_init_entry.lkp_type = lkp_type;
		c2_init_entry.lkp_type_mask = 0x3F;
		/* Action info */
		c2_init_entry.action.color_act = MVPP2_COLOR_ACTION_TYPE_NO_UPDT_LOCK;
		c2_init_entry.action.pri_act = MVPP2_ACTION_TYPE_NO_UPDT_LOCK;
		c2_init_entry.action.dscp_act = MVPP2_ACTION_TYPE_NO_UPDT_LOCK;
		c2_init_entry.action.q_low_act = MVPP2_ACTION_TYPE_UPDT_LOCK;
		c2_init_entry.action.q_high_act = MVPP2_ACTION_TYPE_UPDT_LOCK;
		c2_init_entry.action.rss_act = MVPP2_ACTION_TYPE_UPDT_LOCK;
		/* To CPU */
		c2_init_entry.action.frwd_act = MVPP2_FRWD_ACTION_TYPE_SWF_LOCK;

		/* QoS info */
		if (lkp_type != MVPP2_CLS_LKP_DEFAULT) {
			/* QoS info from C2 QoS table */
			/* Set the QoS table index equal to port ID */
			c2_init_entry.qos_info.qos_tbl_index = port->id;
			c2_init_entry.qos_info.q_low_src =
			    MVPP2_QOS_SRC_DSCP_PBIT_TBL;
			c2_init_entry.qos_info.q_high_src =
			    MVPP2_QOS_SRC_DSCP_PBIT_TBL;
			if (lkp_type == MVPP2_CLS_LKP_VLAN_PRI) {
				c2_init_entry.qos_info.qos_tbl_type =
				    MVPP2_QOS_TBL_SEL_PRI;
				mv_pp2x_cls_c2_qos_tbl_fill(port,
							    MVPP2_QOS_TBL_SEL_PRI, start_queue);
			} else if (lkp_type == MVPP2_CLS_LKP_DSCP_PRI) {
				c2_init_entry.qos_info.qos_tbl_type =
				    MVPP2_QOS_TBL_SEL_DSCP;
				mv_pp2x_cls_c2_qos_tbl_fill(port,
							    MVPP2_QOS_TBL_SEL_DSCP, start_queue);
			}
		} else {
			/* QoS info from C2 action table */
			c2_init_entry.qos_info.q_low_src = MVPP2_QOS_SRC_ACTION_TBL;
			c2_init_entry.qos_info.q_high_src = MVPP2_QOS_SRC_ACTION_TBL;
			cos_value = port->parent->pp2_cfg.cos_cfg.default_cos;
			cos_queue = mv_pp2x_cosval_queue_map(port, cos_value);
			/* map to physical queue */
			/* Physical queue contains 2 parts: port ID and CPU ID,
			 * CPU ID will be used in RSS
			 */
			queue = start_queue + cos_queue;
			c2_init_entry.qos_value.q_low = ((uint16_t)queue) &
			    ((1 << MVPP2_CLS2_ACT_QOS_ATTR_QL_BITS) - 1);
			c2_init_entry.qos_value.q_high = ((uint16_t)queue) >>
			    MVPP2_CLS2_ACT_QOS_ATTR_QL_BITS;
		}
		/* RSS En in PP22 */
		c2_init_entry.rss_en = port->parent->pp2_cfg.rss_cfg.rss_en;

		/* Add rule to C2 TCAM */
		ret = mv_pp2x_c2_rule_add(port, &c2_init_entry);
		if (ret)
			return ret;
	}

	return 0;
}

int mv_pp2x_cos_classifier_set(struct pp2_port *port,
			       enum mv_pp2x_cos_classifier cos_mode)
{
	int index, flow_idx, lkpid;
	int data[3];
	struct pp2_hw *hw = &port->parent->hw;
	struct mv_pp2x_cls_flow_info *flow_info;

	for (index = 0; index < (MVPP2_PRS_FL_LAST - MVPP2_PRS_FL_START);
	     index++) {
		flow_info = &hw->cls_shadow->flow_info[index];
		data[0] = MVPP2_FLOW_TBL_SIZE;
		data[1] = MVPP2_FLOW_TBL_SIZE;
		data[2] = MVPP2_FLOW_TBL_SIZE;
		lkpid = index + MVPP2_PRS_FL_START;
		/* Prepare a temp table for the lkpid */
		mv_pp2x_cls_flow_tbl_temp_copy(hw, lkpid, &flow_idx);
		/* Update lookup table to temp flow table */
		mv_pp2x_cls_lkp_flow_set(hw, lkpid, 0, flow_idx);
		mv_pp2x_cls_lkp_flow_set(hw, lkpid, 1, flow_idx);
		/* Update original flow table */
		/* First, remove the port from original table */
		if (flow_info->flow_entry_dflt) {
			mv_pp2x_cls_flow_port_del(hw,
						  flow_info->flow_entry_dflt,
						  port->id);
			data[0] = flow_info->flow_entry_dflt;
		}
		if (flow_info->flow_entry_vlan) {
			mv_pp2x_cls_flow_port_del(hw,
						  flow_info->flow_entry_vlan,
						  port->id);
			data[1] = flow_info->flow_entry_vlan;
		}
		if (flow_info->flow_entry_dscp) {
			mv_pp2x_cls_flow_port_del(hw,
						  flow_info->flow_entry_dscp,
						  port->id);
			data[2] = flow_info->flow_entry_dscp;
		}

		/* Second, update the port in original table */
		if (mv_pp2x_prs_flow_id_attr_get(lkpid) &
		    MVPP2_PRS_FL_ATTR_VLAN_BIT) {
			if (cos_mode == MVPP2_COS_CLS_VLAN ||
			    cos_mode == MVPP2_COS_CLS_VLAN_DSCP ||
			    (cos_mode == MVPP2_COS_CLS_DSCP_VLAN &&
			     lkpid == MVPP2_PRS_FL_NON_IP_TAG))
				mv_pp2x_cls_flow_port_add(hw,
							  flow_info->
							  flow_entry_vlan,
							  port->id);
			/* Hanlde NON-IP tagged packet */
			else if (cos_mode == MVPP2_COS_CLS_DSCP &&
				 lkpid == MVPP2_PRS_FL_NON_IP_TAG)
				mv_pp2x_cls_flow_port_add(hw,
							  flow_info->
							  flow_entry_dflt,
							  port->id);
			else if (cos_mode == MVPP2_COS_CLS_DSCP ||
				 cos_mode == MVPP2_COS_CLS_DSCP_VLAN)
				mv_pp2x_cls_flow_port_add(hw,
							  flow_info->
							  flow_entry_dscp,
							  port->id);
		} else {
			if (lkpid == MVPP2_PRS_FL_NON_IP_UNTAG ||
			    cos_mode == MVPP2_COS_CLS_VLAN)
				mv_pp2x_cls_flow_port_add(hw,
							  flow_info->
							  flow_entry_dflt,
							  port->id);
			else if (cos_mode == MVPP2_COS_CLS_DSCP ||
				 cos_mode == MVPP2_COS_CLS_VLAN_DSCP ||
				 cos_mode == MVPP2_COS_CLS_DSCP_VLAN)
				mv_pp2x_cls_flow_port_add(hw,
							  flow_info->
							  flow_entry_dscp,
							  port->id);
		}
		/* Restore lookup table */
		flow_idx = min(data[0], min(data[1], data[2]));
		mv_pp2x_cls_lkp_flow_set(hw, lkpid, 0, flow_idx);
		mv_pp2x_cls_lkp_flow_set(hw, lkpid, 1, flow_idx);
	}

	/* Update it in priv */
	port->parent->pp2_cfg.cos_cfg.cos_classifier = cos_mode;

	return 0;
}

/* mv_pp2x_cos_classifier_get
*  -- Get the cos classifier on the port.
*/
int mv_pp2x_cos_classifier_get(struct pp2_port *port)
{
	return port->parent->pp2_cfg.cos_cfg.cos_classifier;
}

/* mv_pp2x_cos_pri_map_set
*  -- Set priority_map per port, nibble for each cos value(0~7).
*/
int mv_pp2x_cos_pri_map_set(struct pp2_port *port, int cos_pri_map)
{
	int ret, prev_pri_map;
	u8 bound_cpu_first_rxq;

	if (port->parent->pp2_cfg.cos_cfg.pri_map == cos_pri_map)
		return 0;

	prev_pri_map = port->parent->pp2_cfg.cos_cfg.pri_map;
	port->parent->pp2_cfg.cos_cfg.pri_map = cos_pri_map;

	/* Update C2 rules with nre pri_map */
	bound_cpu_first_rxq = mv_pp2x_bound_cpu_first_rxq_calc(port);
	ret = mv_pp2x_cls_c2_rule_set(port, bound_cpu_first_rxq);
	if (ret) {
		port->parent->pp2_cfg.cos_cfg.pri_map = prev_pri_map;
		return ret;
	}

	return 0;
}

/* mv_pp2x_cos_pri_map_get
*  -- Get priority_map on the port.
*/
int mv_pp2x_cos_pri_map_get(struct pp2_port *port)
{
	return port->parent->pp2_cfg.cos_cfg.pri_map;
}

/* mv_pp2x_cos_default_value_set
*  -- Set default cos value for untagged or non-IP packets per port.
*/
int mv_pp2x_cos_default_value_set(struct pp2_port *port, int cos_value)
{
	int ret, prev_cos_value;
	u8 bound_cpu_first_rxq;

	if (port->parent->pp2_cfg.cos_cfg.default_cos == cos_value)
		return 0;

	prev_cos_value = port->parent->pp2_cfg.cos_cfg.default_cos;
	port->parent->pp2_cfg.cos_cfg.default_cos = cos_value;

	/* Update C2 rules with the pri_map */
	bound_cpu_first_rxq = mv_pp2x_bound_cpu_first_rxq_calc(port);
	ret = mv_pp2x_cls_c2_rule_set(port, bound_cpu_first_rxq);
	if (ret) {
		port->parent->pp2_cfg.cos_cfg.default_cos = prev_cos_value;
		return ret;
	}

	return 0;
}

/* mv_pp2x_cos_default_value_get
*  -- Get default cos value for untagged or non-IP packets on the port.
*/
int mv_pp2x_cos_default_value_get(struct pp2_port *port)
{
	return port->parent->pp2_cfg.cos_cfg.default_cos;
}

/* The function get the queue in the C2 rule with input index */
uint8_t mv_pp2x_cls_c2_rule_queue_get(struct pp2_hw *hw, uint32_t rule_idx)
{
	u32 reg_val;
	u8 queue;

	/* Write index reg */
	pp2_reg_write(hw->base[0].va, MVPP2_CLS2_TCAM_IDX_REG, rule_idx);

	/* Read Reg CLSC2_ATTR0 */
	reg_val = pp2_reg_read(hw->base[0].va, MVPP2_CLS2_ACT_QOS_ATTR_REG);
	queue = (reg_val & (MVPP2_CLS2_ACT_QOS_ATTR_QL_MASK |
			   MVPP2_CLS2_ACT_QOS_ATTR_QH_MASK)) >>
	    MVPP2_CLS2_ACT_QOS_ATTR_QL_OFF;
	return queue;
}

/* The function set the qos queue in one C2 rule */
void mv_pp2x_cls_c2_rule_queue_set(struct pp2_hw *hw, uint32_t rule_idx,
				   uint8_t queue)
{
	u32 reg_val;

	/* Write index reg */
	pp2_reg_write(hw->base[0].va, MVPP2_CLS2_TCAM_IDX_REG, rule_idx);

	/* Read Reg CLSC2_ATTR0 */
	reg_val = pp2_reg_read(hw->base[0].va, MVPP2_CLS2_ACT_QOS_ATTR_REG);
	/* Update Value */
	reg_val &= (~(MVPP2_CLS2_ACT_QOS_ATTR_QL_MASK |
		     MVPP2_CLS2_ACT_QOS_ATTR_QH_MASK));
	reg_val |= (((uint32_t)queue) << MVPP2_CLS2_ACT_QOS_ATTR_QL_OFF);

	/* Write Reg CLSC2_ATTR0 */
	pp2_reg_write(hw->base[0].va, MVPP2_CLS2_ACT_QOS_ATTR_REG, reg_val);
}

/* The function get the queue in the pbit table entry */
uint8_t mv_pp2x_cls_c2_pbit_tbl_queue_get(struct pp2_hw *hw, uint8_t tbl_id,
					  uint8_t tbl_line)
{
	u8 queue;
	u32 reg_val = 0;

	/* write index reg */
	reg_val |= (tbl_line << MVPP2_CLS2_DSCP_PRI_INDEX_LINE_OFF);
	reg_val |= (MVPP2_QOS_TBL_SEL_PRI << MVPP2_CLS2_DSCP_PRI_INDEX_SEL_OFF);
	reg_val |= (tbl_id << MVPP2_CLS2_DSCP_PRI_INDEX_TBL_ID_OFF);
	pp2_reg_write(hw->base[0].va, MVPP2_CLS2_DSCP_PRI_INDEX_REG, reg_val);
	/* Read Reg CLSC2_DSCP_PRI */
	reg_val = pp2_reg_read(hw->base[0].va, MVPP2_CLS2_QOS_TBL_REG);
	queue = (reg_val & MVPP2_CLS2_QOS_TBL_QUEUENUM_MASK) >>
	    MVPP2_CLS2_QOS_TBL_QUEUENUM_OFF;

	return queue;
}

/* The function set the queue in the pbit table entry */
void mv_pp2x_cls_c2_pbit_tbl_queue_set(struct pp2_hw *hw,
				       u8 tbl_id, uint8_t tbl_line,
				       uint8_t queue)
{
	u32 reg_val = 0;

	/* write index reg */
	reg_val |= (tbl_line << MVPP2_CLS2_DSCP_PRI_INDEX_LINE_OFF);
	reg_val |= (MVPP2_QOS_TBL_SEL_PRI << MVPP2_CLS2_DSCP_PRI_INDEX_SEL_OFF);
	reg_val |= (tbl_id << MVPP2_CLS2_DSCP_PRI_INDEX_TBL_ID_OFF);
	pp2_reg_write(hw->base[0].va, MVPP2_CLS2_DSCP_PRI_INDEX_REG, reg_val);

	/* Read Reg CLSC2_DSCP_PRI */
	reg_val = pp2_reg_read(hw->base[0].va, MVPP2_CLS2_QOS_TBL_REG);
	reg_val &= (~MVPP2_CLS2_QOS_TBL_QUEUENUM_MASK);
	reg_val |= (((uint32_t)queue) << MVPP2_CLS2_QOS_TBL_QUEUENUM_OFF);

	/* Write Reg CLSC2_DSCP_PRI */
	pp2_reg_write(hw->base[0].va, MVPP2_CLS2_QOS_TBL_REG, reg_val);
}

/* RSS */
/* The function will set rss table entry */
int mv_pp22_rss_tbl_entry_set(struct pp2_hw *hw,
			      struct mv_pp22_rss_entry *rss)
{
	unsigned int reg_val = 0;

	if (!rss || rss->sel > MVPP22_RSS_ACCESS_TBL)
		return -EINVAL;

	if (rss->sel == MVPP22_RSS_ACCESS_POINTER) {
		if (rss->u.pointer.rss_tbl_ptr >= MVPP22_RSS_TBL_NUM)
			return -EINVAL;
		/* Write index */
		reg_val |= rss->u.pointer.rxq_idx << MVPP22_RSS_IDX_RXQ_NUM_OFF;
		pp2_reg_write(hw->base[0].va, MVPP22_RSS_IDX_REG, reg_val);
		/* Write entry */
		reg_val &= (~MVPP22_RSS_RXQ2RSS_TBL_POINT_MASK);
		reg_val |= rss->u.pointer.rss_tbl_ptr <<
		    MVPP22_RSS_RXQ2RSS_TBL_POINT_OFF;
		pp2_reg_write(hw->base[0].va, MVPP22_RSS_RXQ2RSS_TBL_REG,
			      reg_val);
	} else if (rss->sel == MVPP22_RSS_ACCESS_TBL) {
		if (rss->u.entry.tbl_id >= MVPP22_RSS_TBL_NUM ||
		    rss->u.entry.tbl_line >= MVPP22_RSS_TBL_LINE_NUM ||
		    rss->u.entry.width >= MVPP22_RSS_WIDTH_MAX)
			return -EINVAL;
		/* Write index */
		reg_val |= (rss->u.entry.tbl_line <<
			   MVPP22_RSS_IDX_ENTRY_NUM_OFF |
			   rss->u.entry.tbl_id << MVPP22_RSS_IDX_TBL_NUM_OFF);
		pp2_reg_write(hw->base[0].va, MVPP22_RSS_IDX_REG, reg_val);
		/* Write entry */
		reg_val &= (~MVPP22_RSS_TBL_ENTRY_MASK);
		reg_val |= (rss->u.entry.rxq << MVPP22_RSS_TBL_ENTRY_OFF);
		pp2_reg_write(hw->base[0].va, MVPP22_RSS_TBL_ENTRY_REG, reg_val);
		reg_val &= (~MVPP22_RSS_WIDTH_MASK);
		reg_val |= (rss->u.entry.width << MVPP22_RSS_WIDTH_OFF);
		pp2_reg_write(hw->base[0].va, MVPP22_RSS_WIDTH_REG, reg_val);
	}
	return 0;
}

/* Translate CPU sequence number to real CPU ID */
static inline int mv_pp22_cpu_id_from_indir_tbl_get(struct pp2_inst *pp2,
						    int cpu_seq,
						    uint32_t *cpu_id)
{
	int i;
	int seq = 0;

	if (!pp2 || !cpu_id || cpu_seq >= 16)
		return -EINVAL;

	for (i = 0; i < 16; i++) {
		if (pp2->cpu_map & (1 << i)) {
			if (seq == cpu_seq) {
				*cpu_id = i;
				return 0;
			}
			seq++;
		}
	}

	return -1;
}

/* mv_pp22_rss_default_cpu_set
*  -- The API to update the default CPU to handle the non-IP packets.
*/
int mv_pp22_rss_default_cpu_set(struct pp2_port *port, int default_cpu)
{
	u8 index, queue, q_cpu_mask;
	u32 cpu_width = 0, cos_width = 0;
	struct pp2_hw *hw = &port->parent->hw;

	if (port->parent->pp2_cfg.queue_mode == MVPP2_QDIST_SINGLE_MODE)
		return -1;

	/* Calculate width */
	mv_pp2x_width_calc(port->parent, &cpu_width, &cos_width, NULL);
	q_cpu_mask = (1 << cpu_width) - 1;

	/* Update LSB[cpu_width + cos_width - 1 : cos_width]
	 * of queue (queue high and low) on c2 rule.
	 */
	index = hw->c2_shadow->rule_idx_info[port->id].default_rule_idx;
	queue = mv_pp2x_cls_c2_rule_queue_get(hw, index);
	queue &= ~(q_cpu_mask << cos_width);
	queue |= (default_cpu << cos_width);
	mv_pp2x_cls_c2_rule_queue_set(hw, index, queue);

	/* Update LSB[cpu_width + cos_width - 1 : cos_width]
	 * of queue on pbit table, table id equals to port id
	 */
	for (index = 0; index < MVPP2_QOS_TBL_LINE_NUM_PRI; index++) {
		queue = mv_pp2x_cls_c2_pbit_tbl_queue_get(hw, port->id, index);
		queue &= ~(q_cpu_mask << cos_width);
		queue |= (default_cpu << cos_width);
		mv_pp2x_cls_c2_pbit_tbl_queue_set(hw, port->id, index, queue);
	}

	/* Update default cpu in cfg */
	port->parent->pp2_cfg.rss_cfg.dflt_cpu = default_cpu;

	return 0;
}

/* mv_pp22_rss_rxfh_indir_set
*  -- The API set the RSS table according to CPU weight from ethtool
*/
int mv_pp22_rss_rxfh_indir_set(struct pp2_port *port)
{
	struct mv_pp22_rss_entry rss_entry;
	int rss_tbl, entry_idx;
	u32 cos_width = 0, cpu_width = 0, cpu_id = 0;
	int rss_tbl_needed = port->parent->pp2_cfg.cos_cfg.num_cos_queues;

	if (port->parent->pp2_cfg.queue_mode == MVPP2_QDIST_SINGLE_MODE)
		return -1;

	memset(&rss_entry, 0, sizeof(struct mv_pp22_rss_entry));

	if (!port->parent->cpu_map)
		return -1;

	/* Calculate cpu and cos width */
	mv_pp2x_width_calc(port->parent, &cpu_width, &cos_width, NULL);

	rss_entry.u.entry.width = cos_width + cpu_width;

	rss_entry.sel = MVPP22_RSS_ACCESS_TBL;

	for (rss_tbl = 0; rss_tbl < rss_tbl_needed; rss_tbl++) {
		for (entry_idx = 0; entry_idx < MVPP22_RSS_TBL_LINE_NUM;
		     entry_idx++) {
			rss_entry.u.entry.tbl_id = rss_tbl;
			rss_entry.u.entry.tbl_line = entry_idx;
			if (mv_pp22_cpu_id_from_indir_tbl_get(port->parent,
							      port->parent->
							      rx_table
							      [entry_idx],
							      &cpu_id))
				return -1;
			/* Value of rss_tbl equals to cos queue */
			rss_entry.u.entry.rxq = (cpu_id << cos_width) | rss_tbl;
			if (mv_pp22_rss_tbl_entry_set
			    (&port->parent->hw, &rss_entry))
				return -1;
		}
	}

	return 0;
}

/* The function allocate a rss table for each phisical rxq,
 * they have same cos priority
 */
int mv_pp22_rss_rxq_set(struct pp2_port *port, uint32_t cos_width)
{
	int rxq;
	struct mv_pp22_rss_entry rss_entry;
	int cos_mask = ((1 << cos_width) - 1);

	memset(&rss_entry, 0, sizeof(struct mv_pp22_rss_entry));

	rss_entry.sel = MVPP22_RSS_ACCESS_POINTER;

	for (rxq = 0; rxq < port->num_rx_queues; rxq++) {
		rss_entry.u.pointer.rxq_idx = port->rxqs[rxq]->id;
		rss_entry.u.pointer.rss_tbl_ptr =
		    port->rxqs[rxq]->id & cos_mask;
		if (mv_pp22_rss_tbl_entry_set(&port->parent->hw, &rss_entry))
			return -1;
	}

	return 0;
}

void mv_pp22_rss_c2_enable(struct pp2_port *port, bool en)
{
	int lkp_type, reg_val;
	int c2_index[MVPP2_CLS_LKP_MAX];
	struct mv_pp2x_c2_rule_idx *rule_idx;
	uintptr_t cpu_slot = port->cpu_slot;

	rule_idx = &port->parent->hw.c2_shadow->rule_idx_info[port->id];

	/* Get the C2 index from shadow */
	c2_index[MVPP2_CLS_LKP_VLAN_PRI] = rule_idx->vlan_pri_idx;
	c2_index[MVPP2_CLS_LKP_DSCP_PRI] = rule_idx->dscp_pri_idx;
	c2_index[MVPP2_CLS_LKP_DEFAULT] = rule_idx->default_rule_idx;

	for (lkp_type = 0; lkp_type < MVPP2_CLS_LKP_MAX; lkp_type++) {
		/* For lookup type of MVPP2_CLS_LKP_HASH,
		 * there is no corresponding C2 rule, so skip it
		 */
		if (lkp_type == MVPP2_CLS_LKP_HASH)
			continue;
		/* write index reg */
		pp2_reg_write(cpu_slot, MVPP2_CLS2_TCAM_IDX_REG,
			      c2_index[lkp_type]);
		/* Update rss_attr in reg CLSC2_ATTR2 */
		reg_val = pp2_reg_read(cpu_slot, MVPP2_CLS2_ACT_DUP_ATTR_REG);
		if (en)
			reg_val |= MVPP2_CLS2_ACT_DUP_ATTR_RSSEN_MASK;
		else
			reg_val &= (~MVPP2_CLS2_ACT_DUP_ATTR_RSSEN_MASK);

		pp2_reg_write(cpu_slot, MVPP2_CLS2_ACT_DUP_ATTR_REG, reg_val);
	}
}

/* mv_pp22_rss_enable_set
*  -- The API enable or disable RSS on the port
*/
void mv_pp22_rss_enable(struct pp2_port *port, uint32_t en)
{
	u8 bound_cpu_first_rxq;

	if (port->parent->pp2_cfg.rss_cfg.rss_en == en)
		return;

	bound_cpu_first_rxq = mv_pp2x_bound_cpu_first_rxq_calc(port);

	if (port->parent->pp2_cfg.queue_mode == MVPP2_QDIST_MULTI_MODE) {
		mv_pp22_rss_c2_enable(port, en);
		if (en) {
			if (mv_pp22_rss_default_cpu_set(port,
							port->parent->pp2_cfg.
							rss_cfg.dflt_cpu))
				pr_err("cannot set rss cpu on port(%d)\n",
					port->id);
			else
				port->parent->pp2_cfg.rss_cfg.rss_en = 1;
		} else {
			if (mv_pp2x_cls_c2_rule_set(port, bound_cpu_first_rxq))
				pr_err("cannot set c2, qos table on port(%d)\n",
					port->id);
			else
				port->parent->pp2_cfg.rss_cfg.rss_en = 0;
		}
	}
}

/* mv_pp2x_rss_mode_set
*  -- The API to update RSS hash mode for non-fragemnt UDP packet per port.
*/
int mv_pp22_rss_mode_set(struct pp2_port *port, int rss_mode)
{
	int index, flow_idx, flow_idx_rss, lkpid, lkpid_attr;
	int data[3];
	struct pp2_hw *hw = &port->parent->hw;
	struct mv_pp2x_cls_flow_info *flow_info;

	if (port->parent->pp2_cfg.queue_mode == MVPP2_QDIST_SINGLE_MODE)
		return -1;

	for (index = 0; index < (MVPP2_PRS_FL_LAST - MVPP2_PRS_FL_START);
	     index++) {
		flow_info = &hw->cls_shadow->flow_info[index];
		data[0] = MVPP2_FLOW_TBL_SIZE;
		data[1] = MVPP2_FLOW_TBL_SIZE;
		data[2] = MVPP2_FLOW_TBL_SIZE;
		lkpid = index + MVPP2_PRS_FL_START;
		/* Get lookup ID attribute */
		lkpid_attr = mv_pp2x_prs_flow_id_attr_get(lkpid);
		/* Only non-frag UDP can set rss mode */
		if ((lkpid_attr & MVPP2_PRS_FL_ATTR_UDP_BIT) &&
		    !(lkpid_attr & MVPP2_PRS_FL_ATTR_FRAG_BIT)) {
			/* Prepare a temp table for the lkpid */
			mv_pp2x_cls_flow_tbl_temp_copy(hw, lkpid, &flow_idx);
			/* Update lookup table to temp flow table */
			mv_pp2x_cls_lkp_flow_set(hw, lkpid, 0, flow_idx);
			mv_pp2x_cls_lkp_flow_set(hw, lkpid, 1, flow_idx);
			/* Update original flow table */
			/* First, remove the port from original table */
			mv_pp2x_cls_flow_port_del(hw,
						  flow_info->flow_entry_rss1,
						  port->id);
			mv_pp2x_cls_flow_port_del(hw,
						  flow_info->flow_entry_rss2,
						  port->id);
			if (flow_info->flow_entry_dflt)
				data[0] = flow_info->flow_entry_dflt;
			if (flow_info->flow_entry_vlan)
				data[1] = flow_info->flow_entry_vlan;
			if (flow_info->flow_entry_dscp)
				data[2] = flow_info->flow_entry_dscp;
			/* Second, update port in original table -> rss_mode */
			if (rss_mode == MVPP2_RSS_NF_UDP_2T)
				flow_idx_rss = flow_info->flow_entry_rss1;
			else
				flow_idx_rss = flow_info->flow_entry_rss2;
			mv_pp2x_cls_flow_port_add(hw, flow_idx_rss, port->id);

			/*Find the ptr of flow table, the min flow index */
			flow_idx_rss = min(flow_info->flow_entry_rss1,
					   flow_info->flow_entry_rss2);
			flow_idx = min(min(data[0], data[1]),
				       min(data[2], flow_idx_rss));
			/*Third, restore lookup table */
			mv_pp2x_cls_lkp_flow_set(hw, lkpid, 0, flow_idx);
			mv_pp2x_cls_lkp_flow_set(hw, lkpid, 1, flow_idx);
		} else if (flow_info->flow_entry_rss1) {
			flow_idx_rss = flow_info->flow_entry_rss1;
			mv_pp2x_cls_flow_port_add(hw, flow_idx_rss, port->id);
		}
	}
	/* Record it in priv */
	port->parent->pp2_cfg.rss_cfg.rss_mode = rss_mode;

	return 0;
}

/* Invalidate tcam hw entry */
void mv_pp2x_prs_hw_inv(struct pp2_hw *hw, int index)
{
	/* Write index - indirect access */
	pp2_reg_write(hw->base[0].va, MVPP2_PRS_TCAM_IDX_REG, index);
	pp2_reg_write(hw->base[0].va,
		      MVPP2_PRS_TCAM_DATA_REG(MVPP2_PRS_TCAM_INV_WORD),
		     MVPP2_PRS_TCAM_INV_MASK);
}

/* Parser per-port initialization */
void mv_pp2x_prs_hw_port_init(struct pp2_hw *hw, int port, int lu_first,
			      int lu_max, int offset)
{
	u32 val;

	/* Set lookup ID */
	val = pp2_reg_read(hw->base[0].va, MVPP2_PRS_INIT_LOOKUP_REG);
	val &= ~MVPP2_PRS_PORT_LU_MASK(port);
	val |= MVPP2_PRS_PORT_LU_VAL(port, lu_first);
	pp2_reg_write(hw->base[0].va, MVPP2_PRS_INIT_LOOKUP_REG, val);

	/* Set maximum number of loops for packet received from port */
	val = pp2_reg_read(hw->base[0].va, MVPP2_PRS_MAX_LOOP_REG(port));
	val &= ~MVPP2_PRS_MAX_LOOP_MASK(port);
	val |= MVPP2_PRS_MAX_LOOP_VAL(port, lu_max);
	pp2_reg_write(hw->base[0].va, MVPP2_PRS_MAX_LOOP_REG(port), val);

	/* Set initial offset for packet header extraction for the first
	 * searching loop
	 */
	val = pp2_reg_read(hw->base[0].va, MVPP2_PRS_INIT_OFFS_REG(port));
	val &= ~MVPP2_PRS_INIT_OFF_MASK(port);
	val |= MVPP2_PRS_INIT_OFF_VAL(port, offset);
	pp2_reg_write(hw->base[0].va, MVPP2_PRS_INIT_OFFS_REG(port), val);
}

/* Enable/disable dropping all mac da's */
static void mv_pp2x_prs_mac_drop_all_set(struct pp2_hw *hw,
					 int port, bool add)
{
	struct mv_pp2x_prs_entry pe;

	if (hw->prs_shadow[MVPP2_PE_DROP_ALL].valid) {
		/* Entry exist - update port only */
		pe.index = MVPP2_PE_DROP_ALL;
		mv_pp2x_prs_hw_read(hw, &pe);
	} else {
		/* Entry doesn't exist - create new */
		memset(&pe, 0, sizeof(struct mv_pp2x_prs_entry));
		mv_pp2x_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_MAC);
		pe.index = MVPP2_PE_DROP_ALL;

		/* Non-promiscuous mode for all ports - DROP unknown packets */
		mv_pp2x_prs_sram_ri_update(&pe, MVPP2_PRS_RI_DROP_MASK,
					   MVPP2_PRS_RI_DROP_MASK);

		mv_pp2x_prs_sram_bits_set(&pe, MVPP2_PRS_SRAM_LU_GEN_BIT, 1);
		mv_pp2x_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_FLOWS);

		/* Update shadow table */
		mv_pp2x_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_MAC);

		/* Mask all ports */
		mv_pp2x_prs_tcam_port_map_set(&pe, 0);
	}

	/* Update port mask */
	mv_pp2x_prs_tcam_port_set(&pe, port, add);

	mv_pp2x_prs_hw_write(hw, &pe);
}

/* Set port to promiscuous mode */
void mv_pp2x_prs_mac_promisc_set(struct pp2_hw *hw, int port, bool add)
{
	struct mv_pp2x_prs_entry pe;

	/* Promiscuous mode - Accept unknown packets */

	if (hw->prs_shadow[MVPP2_PE_MAC_PROMISCUOUS].valid) {
		/* Entry exist - update port only */
		pe.index = MVPP2_PE_MAC_PROMISCUOUS;
		mv_pp2x_prs_hw_read(hw, &pe);
	} else {
		/* Entry doesn't exist - create new */
		memset(&pe, 0, sizeof(struct mv_pp2x_prs_entry));
		mv_pp2x_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_MAC);
		pe.index = MVPP2_PE_MAC_PROMISCUOUS;

		/* Continue - set next lookup */
		mv_pp2x_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_DSA);

		/* Set result info bits */
		mv_pp2x_prs_sram_ri_update(&pe, MVPP2_PRS_RI_L2_UCAST,
					   MVPP2_PRS_RI_L2_CAST_MASK);

		/* Shift to ethertype */
		mv_pp2x_prs_sram_shift_set(&pe, 2 * ETH_ALEN,
					   MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);

		/* Mask all ports */
		mv_pp2x_prs_tcam_port_map_set(&pe, 0);

		/* Update shadow table */
		mv_pp2x_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_MAC);
	}

	/* Update port mask */
	mv_pp2x_prs_tcam_port_set(&pe, port, add);

	mv_pp2x_prs_hw_write(hw, &pe);
}

/* Accept multicast */
void mv_pp2x_prs_mac_multi_set(struct pp2_hw *hw, int port, int index,
			       bool add)
{
	struct mv_pp2x_prs_entry pe;
	unsigned char da_mc;

	/* Ethernet multicast address first byte is
	 * 0x01 for IPv4 and 0x33 for IPv6
	 */
	da_mc = (index == MVPP2_PE_MAC_MC_ALL) ? 0x01 : 0x33;

	if (hw->prs_shadow[index].valid) {
		/* Entry exist - update port only */
		pe.index = index;
		mv_pp2x_prs_hw_read(hw, &pe);
	} else {
		/* Entry doesn't exist - create new */
		memset(&pe, 0, sizeof(struct mv_pp2x_prs_entry));
		mv_pp2x_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_MAC);
		pe.index = index;

		/* Continue - set next lookup */
		mv_pp2x_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_DSA);

		/* Set result info bits */
		mv_pp2x_prs_sram_ri_update(&pe, MVPP2_PRS_RI_L2_MCAST,
					   MVPP2_PRS_RI_L2_CAST_MASK);

		/* Update tcam entry data first byte */
		mv_pp2x_prs_tcam_data_byte_set(&pe, 0, da_mc, 0xff);

		/* Shift to ethertype */
		mv_pp2x_prs_sram_shift_set(&pe, 2 * ETH_ALEN,
					   MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);

		/* Mask all ports */
		mv_pp2x_prs_tcam_port_map_set(&pe, 0);

		/* Update shadow table */
		mv_pp2x_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_MAC);
	}

	/* Update port mask */
	mv_pp2x_prs_tcam_port_set(&pe, port, add);

	mv_pp2x_prs_hw_write(hw, &pe);
}

/* Set entry for dsa ethertype */
static void mv_pp2x_prs_dsa_tag_ethertype_set(struct pp2_hw *hw, int port,
					      bool add, bool tagged,
					      bool extend)
{
	struct mv_pp2x_prs_entry pe;
	int tid, shift, port_mask;

	if (extend) {
		tid = tagged ? MVPP2_PE_ETYPE_EDSA_TAGGED :
		    MVPP2_PE_ETYPE_EDSA_UNTAGGED;
		port_mask = 0;
		shift = 8;
	} else {
		tid = tagged ? MVPP2_PE_ETYPE_DSA_TAGGED :
		    MVPP2_PE_ETYPE_DSA_UNTAGGED;
		port_mask = MVPP2_PRS_PORT_MASK;
		shift = 4;
	}

	if (hw->prs_shadow[tid].valid) {
		/* Entry exist - update port only */
		pe.index = tid;
		mv_pp2x_prs_hw_read(hw, &pe);
	} else {
		/* Entry doesn't exist - create new */
		memset(&pe, 0, sizeof(struct mv_pp2x_prs_entry));
		mv_pp2x_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_DSA);
		pe.index = tid;

		/* Set ethertype */
		mv_pp2x_prs_match_etype(&pe, 0, ETH_P_EDSA);
		mv_pp2x_prs_match_etype(&pe, 2, 0);

		mv_pp2x_prs_sram_ri_update(&pe, MVPP2_PRS_RI_DSA_MASK,
					   MVPP2_PRS_RI_DSA_MASK);
		/* Shift ethertype + 2 byte reserved + tag */
		mv_pp2x_prs_sram_shift_set(&pe, 2 + MVPP2_ETH_TYPE_LEN + shift,
					   MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);

		/* Update shadow table */
		mv_pp2x_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_DSA);

		if (tagged) {
			/* Set tagged bit in DSA tag */
			mv_pp2x_prs_tcam_data_byte_set(&pe,
						       MVPP2_ETH_TYPE_LEN + 2 + 3,
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
		/* Mask/unmask all ports, depending on dsa type */
		mv_pp2x_prs_tcam_port_map_set(&pe, port_mask);
	}

	/* Update port mask */
	mv_pp2x_prs_tcam_port_set(&pe, port, add);

	mv_pp2x_prs_hw_write(hw, &pe);
}

/* Default flow entries initialization for all ports */
static void mv_pp2x_prs_def_flow_init(struct pp2_hw *hw)
{
	struct mv_pp2x_prs_entry pe;
	int port;

	for (port = 0; port < MVPP2_MAX_PORTS; port++) {
		memset(&pe, 0, sizeof(struct mv_pp2x_prs_entry));
		mv_pp2x_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_FLOWS);
		pe.index = MVPP2_PE_FIRST_DEFAULT_FLOW - port;

		/* Mask all ports */
		mv_pp2x_prs_tcam_port_map_set(&pe, 0);

		/* Set flow ID */
		mv_pp2x_prs_sram_ai_update(&pe, port, MVPP2_PRS_FLOW_ID_MASK);
		mv_pp2x_prs_sram_bits_set(&pe, MVPP2_PRS_SRAM_LU_DONE_BIT, 1);

		/* Update shadow table and hw entry */
		mv_pp2x_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_FLOWS);

		/*pr_crit("mv_pp2x_prs_def_flow_init: port(%d), index(%d)\n",
		 * port, pe.index);
		 */
		mv_pp2x_prs_hw_write(hw, &pe);
	}
}

/* Set default entry for Marvell Header field */
static void mv_pp2x_prs_mh_init(struct pp2_hw *hw)
{
	struct mv_pp2x_prs_entry pe;

	memset(&pe, 0, sizeof(struct mv_pp2x_prs_entry));

	pe.index = MVPP2_PE_MH_DEFAULT;
	mv_pp2x_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_MH);
	mv_pp2x_prs_sram_shift_set(&pe, PP2_MH_SIZE,
				   MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);
	mv_pp2x_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_MAC);

	/* Unmask all ports */
	mv_pp2x_prs_tcam_port_map_set(&pe, MVPP2_PRS_PORT_MASK);

	/* Update shadow table and hw entry */
	mv_pp2x_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_MH);
	mv_pp2x_prs_hw_write(hw, &pe);
}

/* Set default entires (place holder) for promiscuous, non-promiscuous and
 * multicast MAC addresses
 */
static void mv_pp2x_prs_mac_init(struct pp2_hw *hw)
{
	struct mv_pp2x_prs_entry pe;

	memset(&pe, 0, sizeof(struct mv_pp2x_prs_entry));

	/* Non-promiscuous mode for all ports - DROP unknown packets */
	pe.index = MVPP2_PE_MAC_NON_PROMISCUOUS;
	mv_pp2x_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_MAC);

	mv_pp2x_prs_sram_ri_update(&pe, MVPP2_PRS_RI_DROP_MASK,
				   MVPP2_PRS_RI_DROP_MASK);
	mv_pp2x_prs_sram_bits_set(&pe, MVPP2_PRS_SRAM_LU_GEN_BIT, 1);
	mv_pp2x_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_FLOWS);

	/* Unmask all ports */
	mv_pp2x_prs_tcam_port_map_set(&pe, MVPP2_PRS_PORT_MASK);

	/* Update shadow table and hw entry */
	mv_pp2x_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_MAC);
	mv_pp2x_prs_hw_write(hw, &pe);

	/* place holders only - no ports */
	mv_pp2x_prs_mac_drop_all_set(hw, 0, false);
	mv_pp2x_prs_mac_promisc_set(hw, 0, false);

	mv_pp2x_prs_mac_multi_set(hw, MVPP2_PE_MAC_MC_ALL, 0, false);
	mv_pp2x_prs_mac_multi_set(hw, MVPP2_PE_MAC_MC_IP6, 0, false);
}

/* Set default entries for various types of dsa packets */
static void mv_pp2x_prs_dsa_init(struct pp2_hw *hw)
{
	struct mv_pp2x_prs_entry pe;

	/* None tagged EDSA entry - place holder */
	mv_pp2x_prs_dsa_tag_set(hw, 0, false, MVPP2_PRS_UNTAGGED,
				MVPP2_PRS_EDSA);

	/* Tagged EDSA entry - place holder */
	mv_pp2x_prs_dsa_tag_set(hw, 0, false, MVPP2_PRS_TAGGED, MVPP2_PRS_EDSA);

	/* None tagged DSA entry - place holder */
	mv_pp2x_prs_dsa_tag_set(hw, 0, false, MVPP2_PRS_UNTAGGED,
				MVPP2_PRS_DSA);

	/* Tagged DSA entry - place holder */
	mv_pp2x_prs_dsa_tag_set(hw, 0, false, MVPP2_PRS_TAGGED, MVPP2_PRS_DSA);

	/* None tagged EDSA ethertype entry - place holder */
	mv_pp2x_prs_dsa_tag_ethertype_set(hw, 0, false,
					  MVPP2_PRS_UNTAGGED, MVPP2_PRS_EDSA);

	/* Tagged EDSA ethertype entry - place holder */
	mv_pp2x_prs_dsa_tag_ethertype_set(hw, 0, false,
					  MVPP2_PRS_TAGGED, MVPP2_PRS_EDSA);

	/* None tagged DSA ethertype entry */
	mv_pp2x_prs_dsa_tag_ethertype_set(hw, 0, true,
					  MVPP2_PRS_UNTAGGED, MVPP2_PRS_DSA);

	/* Tagged DSA ethertype entry */
	mv_pp2x_prs_dsa_tag_ethertype_set(hw, 0, true,
					  MVPP2_PRS_TAGGED, MVPP2_PRS_DSA);

	/* Set default entry, in case DSA or EDSA tag not found */
	memset(&pe, 0, sizeof(struct mv_pp2x_prs_entry));
	mv_pp2x_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_DSA);
	pe.index = MVPP2_PE_DSA_DEFAULT;
	mv_pp2x_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_VLAN);

	/* Shift 0 bytes */
	mv_pp2x_prs_sram_shift_set(&pe, 0, MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);
	mv_pp2x_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_MAC);

	/* Clear all sram ai bits for next iteration */
	mv_pp2x_prs_sram_ai_update(&pe, 0, MVPP2_PRS_SRAM_AI_MASK);

	/* Unmask all ports */
	mv_pp2x_prs_tcam_port_map_set(&pe, MVPP2_PRS_PORT_MASK);

	mv_pp2x_prs_hw_write(hw, &pe);
}

/* Match basic ethertypes */
static int mv_pp2x_prs_etype_init(struct pp2_hw *hw)
{
	struct mv_pp2x_prs_entry pe;
	int tid;

	/* Ethertype: PPPoE */
	tid = mv_pp2x_prs_tcam_first_free(hw, MVPP2_PE_FIRST_FREE_TID,
					  MVPP2_PE_LAST_FREE_TID);
	if (tid < 0)
		return tid;

	memset(&pe, 0, sizeof(struct mv_pp2x_prs_entry));
	mv_pp2x_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_L2);
	pe.index = tid;

	mv_pp2x_prs_match_etype(&pe, 0, ETH_P_PPP2_SES);

	mv_pp2x_prs_sram_shift_set(&pe, MVPP2_PPPOE_HDR_SIZE,
				   MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);
	mv_pp2x_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_PPPOE);
	mv_pp2x_prs_sram_ri_update(&pe, MVPP2_PRS_RI_PPPOE_MASK,
				   MVPP2_PRS_RI_PPPOE_MASK);

	/* Update shadow table and hw entry */
	mv_pp2x_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_L2);
	hw->prs_shadow[pe.index].udf = MVPP2_PRS_UDF_L2_DEF;
	hw->prs_shadow[pe.index].finish = false;
	mv_pp2x_prs_shadow_ri_set(hw, pe.index, MVPP2_PRS_RI_PPPOE_MASK,
				  MVPP2_PRS_RI_PPPOE_MASK);
	mv_pp2x_prs_hw_write(hw, &pe);

	/* Ethertype: ARP */
	tid = mv_pp2x_prs_tcam_first_free(hw, MVPP2_PE_FIRST_FREE_TID,
					  MVPP2_PE_LAST_FREE_TID);
	if (tid < 0)
		return tid;

	memset(&pe, 0, sizeof(struct mv_pp2x_prs_entry));
	mv_pp2x_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_L2);
	pe.index = tid;

	mv_pp2x_prs_match_etype(&pe, 0, ETH_P_ARP);

	/* Generate flow in the next iteration */
	mv_pp2x_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_FLOWS);
	mv_pp2x_prs_sram_bits_set(&pe, MVPP2_PRS_SRAM_LU_GEN_BIT, 1);
	mv_pp2x_prs_sram_ri_update(&pe, MVPP2_PRS_RI_L3_ARP,
				   MVPP2_PRS_RI_L3_PROTO_MASK);
	/* Set L3 offset */
	mv_pp2x_prs_sram_offset_set(&pe, MVPP2_PRS_SRAM_UDF_TYPE_L3,
				    MVPP2_ETH_TYPE_LEN,
				    MVPP2_PRS_SRAM_OP_SEL_UDF_ADD);

	/* Update shadow table and hw entry */
	mv_pp2x_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_L2);
	hw->prs_shadow[pe.index].udf = MVPP2_PRS_UDF_L2_DEF;
	hw->prs_shadow[pe.index].finish = true;
	mv_pp2x_prs_shadow_ri_set(hw, pe.index, MVPP2_PRS_RI_L3_ARP,
				  MVPP2_PRS_RI_L3_PROTO_MASK);
	mv_pp2x_prs_hw_write(hw, &pe);

	/* Ethertype: LBTD */
	tid = mv_pp2x_prs_tcam_first_free(hw, MVPP2_PE_FIRST_FREE_TID,
					  MVPP2_PE_LAST_FREE_TID);
	if (tid < 0)
		return tid;

	memset(&pe, 0, sizeof(struct mv_pp2x_prs_entry));
	mv_pp2x_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_L2);
	pe.index = tid;

	mv_pp2x_prs_match_etype(&pe, 0, MVPP2_IP_LBDT_TYPE);

	/* Generate flow in the next iteration */
	mv_pp2x_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_FLOWS);
	mv_pp2x_prs_sram_bits_set(&pe, MVPP2_PRS_SRAM_LU_GEN_BIT, 1);
	mv_pp2x_prs_sram_ri_update(&pe, MVPP2_PRS_RI_CPU_CODE_RX_SPEC |
				   MVPP2_PRS_RI_UDF3_RX_SPECIAL,
				   MVPP2_PRS_RI_CPU_CODE_MASK |
				   MVPP2_PRS_RI_UDF3_MASK);
	/* Set L3 offset */
	mv_pp2x_prs_sram_offset_set(&pe, MVPP2_PRS_SRAM_UDF_TYPE_L3,
				    MVPP2_ETH_TYPE_LEN,
				    MVPP2_PRS_SRAM_OP_SEL_UDF_ADD);

	/* Update shadow table and hw entry */
	mv_pp2x_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_L2);
	hw->prs_shadow[pe.index].udf = MVPP2_PRS_UDF_L2_DEF;
	hw->prs_shadow[pe.index].finish = true;
	mv_pp2x_prs_shadow_ri_set(hw, pe.index, MVPP2_PRS_RI_CPU_CODE_RX_SPEC |
				  MVPP2_PRS_RI_UDF3_RX_SPECIAL,
				  MVPP2_PRS_RI_CPU_CODE_MASK |
				  MVPP2_PRS_RI_UDF3_MASK);
	mv_pp2x_prs_hw_write(hw, &pe);

	/* Ethertype: IPv4 without options */
	tid = mv_pp2x_prs_tcam_first_free(hw, MVPP2_PE_FIRST_FREE_TID,
					  MVPP2_PE_LAST_FREE_TID);
	if (tid < 0)
		return tid;

	memset(&pe, 0, sizeof(struct mv_pp2x_prs_entry));
	mv_pp2x_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_L2);
	pe.index = tid;

	mv_pp2x_prs_match_etype(&pe, 0, ETH_P_IP);
	mv_pp2x_prs_tcam_data_byte_set(&pe, MVPP2_ETH_TYPE_LEN,
				       MVPP2_PRS_IPV4_HEAD |
				       MVPP2_PRS_IPV4_IHL,
				       MVPP2_PRS_IPV4_HEAD_MASK |
				       MVPP2_PRS_IPV4_IHL_MASK);

	mv_pp2x_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_IP4);
	mv_pp2x_prs_sram_ri_update(&pe, MVPP2_PRS_RI_L3_IP4,
				   MVPP2_PRS_RI_L3_PROTO_MASK);
	/* Skip eth_type + 4 bytes of IP header */
	mv_pp2x_prs_sram_shift_set(&pe, MVPP2_ETH_TYPE_LEN + 4,
				   MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);
	/* Set L3 offset */
	mv_pp2x_prs_sram_offset_set(&pe, MVPP2_PRS_SRAM_UDF_TYPE_L3,
				    MVPP2_ETH_TYPE_LEN,
				    MVPP2_PRS_SRAM_OP_SEL_UDF_ADD);

	/* Update shadow table and hw entry */
	mv_pp2x_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_L2);
	hw->prs_shadow[pe.index].udf = MVPP2_PRS_UDF_L2_DEF;
	hw->prs_shadow[pe.index].finish = false;
	mv_pp2x_prs_shadow_ri_set(hw, pe.index, MVPP2_PRS_RI_L3_IP4,
				  MVPP2_PRS_RI_L3_PROTO_MASK);
	mv_pp2x_prs_hw_write(hw, &pe);

	/* Ethertype: IPv4 with options */
	tid = mv_pp2x_prs_tcam_first_free(hw, MVPP2_PE_FIRST_FREE_TID,
					  MVPP2_PE_LAST_FREE_TID);
	if (tid < 0)
		return tid;

	pe.index = tid;

	/* Clear tcam data before updating */
	pe.tcam.byte[TCAM_DATA_BYTE(MVPP2_ETH_TYPE_LEN)] = 0x0;
	pe.tcam.byte[TCAM_DATA_MASK(MVPP2_ETH_TYPE_LEN)] = 0x0;

	mv_pp2x_prs_tcam_data_byte_set(&pe, MVPP2_ETH_TYPE_LEN,
				       MVPP2_PRS_IPV4_HEAD,
				       MVPP2_PRS_IPV4_HEAD_MASK);

	/* Clear ri before updating */
	pe.sram.word[MVPP2_PRS_SRAM_RI_WORD] = 0x0;
	pe.sram.word[MVPP2_PRS_SRAM_RI_CTRL_WORD] = 0x0;
	mv_pp2x_prs_sram_ri_update(&pe, MVPP2_PRS_RI_L3_IP4_OPT,
				   MVPP2_PRS_RI_L3_PROTO_MASK);

	/* Update shadow table and hw entry */
	mv_pp2x_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_L2);
	hw->prs_shadow[pe.index].udf = MVPP2_PRS_UDF_L2_DEF;
	hw->prs_shadow[pe.index].finish = false;
	mv_pp2x_prs_shadow_ri_set(hw, pe.index, MVPP2_PRS_RI_L3_IP4_OPT,
				  MVPP2_PRS_RI_L3_PROTO_MASK);
	mv_pp2x_prs_hw_write(hw, &pe);

	/* Ethertype: IPv6 without options */
	tid = mv_pp2x_prs_tcam_first_free(hw, MVPP2_PE_FIRST_FREE_TID,
					  MVPP2_PE_LAST_FREE_TID);
	if (tid < 0)
		return tid;

	memset(&pe, 0, sizeof(struct mv_pp2x_prs_entry));
	mv_pp2x_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_L2);
	pe.index = tid;

	mv_pp2x_prs_match_etype(&pe, 0, ETH_P_IPV6);

	/* Skip DIP of IPV6 header */
	mv_pp2x_prs_sram_shift_set(&pe, MVPP2_ETH_TYPE_LEN + 8 +
				   MVPP2_MAX_L3_ADDR_SIZE,
				   MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);
	mv_pp2x_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_IP6);
	mv_pp2x_prs_sram_ri_update(&pe, MVPP2_PRS_RI_L3_IP6,
				   MVPP2_PRS_RI_L3_PROTO_MASK);
	/* Set L3 offset */
	mv_pp2x_prs_sram_offset_set(&pe, MVPP2_PRS_SRAM_UDF_TYPE_L3,
				    MVPP2_ETH_TYPE_LEN,
				    MVPP2_PRS_SRAM_OP_SEL_UDF_ADD);

	mv_pp2x_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_L2);
	hw->prs_shadow[pe.index].udf = MVPP2_PRS_UDF_L2_DEF;
	hw->prs_shadow[pe.index].finish = false;
	mv_pp2x_prs_shadow_ri_set(hw, pe.index, MVPP2_PRS_RI_L3_IP6,
				  MVPP2_PRS_RI_L3_PROTO_MASK);
	mv_pp2x_prs_hw_write(hw, &pe);

	/* Default entry for MVPP2_PRS_LU_L2 - Unknown ethtype */
	memset(&pe, 0, sizeof(struct mv_pp2x_prs_entry));
	mv_pp2x_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_L2);
	pe.index = MVPP2_PE_ETH_TYPE_UN;

	/* Unmask all ports */
	mv_pp2x_prs_tcam_port_map_set(&pe, MVPP2_PRS_PORT_MASK);

	/* Generate flow in the next iteration */
	mv_pp2x_prs_sram_bits_set(&pe, MVPP2_PRS_SRAM_LU_GEN_BIT, 1);
	mv_pp2x_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_FLOWS);
	mv_pp2x_prs_sram_ri_update(&pe, MVPP2_PRS_RI_L3_UN,
				   MVPP2_PRS_RI_L3_PROTO_MASK);
	/* Set L3 offset even it's unknown L3 */
	mv_pp2x_prs_sram_offset_set(&pe, MVPP2_PRS_SRAM_UDF_TYPE_L3,
				    MVPP2_ETH_TYPE_LEN,
				    MVPP2_PRS_SRAM_OP_SEL_UDF_ADD);

	/* Update shadow table and hw entry */
	mv_pp2x_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_L2);
	hw->prs_shadow[pe.index].udf = MVPP2_PRS_UDF_L2_DEF;
	hw->prs_shadow[pe.index].finish = true;
	mv_pp2x_prs_shadow_ri_set(hw, pe.index, MVPP2_PRS_RI_L3_UN,
				  MVPP2_PRS_RI_L3_PROTO_MASK);
	mv_pp2x_prs_hw_write(hw, &pe);
	return 0;
}

/* Search for existing single/triple vlan entry */
static struct mv_pp2x_prs_entry *mv_pp2x_prs_vlan_find(struct pp2_hw *hw,
						       unsigned short tpid,
						       int ai)
{
	struct mv_pp2x_prs_entry *pe;
	int tid;

	pe = kcalloc(1, sizeof(*pe), GFP_KERNEL);
	if (!pe)
		return NULL;
	mv_pp2x_prs_tcam_lu_set(pe, MVPP2_PRS_LU_VLAN);

	/* Go through the all entries with MVPP2_PRS_LU_VLAN */
	for (tid = MVPP2_PE_FIRST_FREE_TID;
	     tid <= MVPP2_PE_LAST_FREE_TID; tid++) {
		unsigned int ri_bits, ai_bits;
		bool match;

		if (!hw->prs_shadow[tid].valid ||
		    hw->prs_shadow[tid].lu != MVPP2_PRS_LU_VLAN)
			continue;

		pe->index = tid;

		mv_pp2x_prs_hw_read(hw, pe);
		match = mv_pp2x_prs_tcam_data_cmp(pe, 0, swab16(tpid));
		if (!match)
			continue;

		/* Get vlan type */
		ri_bits = mv_pp2x_prs_sram_ri_get(pe);
		ri_bits &= MVPP2_PRS_RI_VLAN_MASK;

		/* Get current ai value from tcam */
		ai_bits = mv_pp2x_prs_tcam_ai_get(pe);
		/* Clear double vlan bit */
		ai_bits &= ~MVPP2_PRS_DBL_VLAN_AI_BIT;

		if (ai != ai_bits)
			continue;

		if (ri_bits == MVPP2_PRS_RI_VLAN_SINGLE ||
		    ri_bits == MVPP2_PRS_RI_VLAN_TRIPLE)
			return pe;
	}
	kfree(pe);

	return NULL;
}

/* Add/update single/triple vlan entry */
static int mv_pp2x_prs_vlan_add(struct pp2_hw *hw, unsigned short tpid,
				int ai, unsigned int port_map)
{
	struct mv_pp2x_prs_entry *pe;
	int tid_aux, tid;
	int ret = 0;

	pe = mv_pp2x_prs_vlan_find(hw, tpid, ai);

	if (!pe) {
		/* Create new tcam entry */
		tid = mv_pp2x_prs_tcam_first_free(hw, MVPP2_PE_LAST_FREE_TID,
						  MVPP2_PE_FIRST_FREE_TID);
		if (tid < 0)
			return tid;

		pe = kcalloc(1, sizeof(*pe), GFP_KERNEL);
		if (!pe)
			return -ENOMEM;

		/* Get last double vlan tid */
		for (tid_aux = MVPP2_PE_LAST_FREE_TID;
		     tid_aux >= MVPP2_PE_FIRST_FREE_TID; tid_aux--) {
			unsigned int ri_bits;

			if (!hw->prs_shadow[tid_aux].valid ||
			    hw->prs_shadow[tid_aux].lu != MVPP2_PRS_LU_VLAN)
				continue;

			pe->index = tid_aux;
			mv_pp2x_prs_hw_read(hw, pe);
			ri_bits = mv_pp2x_prs_sram_ri_get(pe);
			if ((ri_bits & MVPP2_PRS_RI_VLAN_MASK) ==
			    MVPP2_PRS_RI_VLAN_DOUBLE)
				break;
		}

		if (tid <= tid_aux) {
			ret = -EINVAL;
			goto error;
		}

		memset(pe, 0, sizeof(struct mv_pp2x_prs_entry));
		mv_pp2x_prs_tcam_lu_set(pe, MVPP2_PRS_LU_VLAN);
		pe->index = tid;

		mv_pp2x_prs_match_etype(pe, 0, tpid);

		mv_pp2x_prs_sram_next_lu_set(pe, MVPP2_PRS_LU_L2);
		/* Shift 4 bytes - skip 1 vlan tag */
		mv_pp2x_prs_sram_shift_set(pe, MVPP2_VLAN_TAG_LEN,
					   MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);
		/* Clear all ai bits for next iteration */
		mv_pp2x_prs_sram_ai_update(pe, 0, MVPP2_PRS_SRAM_AI_MASK);

		if (ai == MVPP2_PRS_SINGLE_VLAN_AI) {
			mv_pp2x_prs_sram_ri_update(pe, MVPP2_PRS_RI_VLAN_SINGLE,
						   MVPP2_PRS_RI_VLAN_MASK);
		} else {
			ai |= MVPP2_PRS_DBL_VLAN_AI_BIT;
			mv_pp2x_prs_sram_ri_update(pe, MVPP2_PRS_RI_VLAN_TRIPLE,
						   MVPP2_PRS_RI_VLAN_MASK);
		}
		mv_pp2x_prs_tcam_ai_update(pe, ai, MVPP2_PRS_SRAM_AI_MASK);

		mv_pp2x_prs_shadow_set(hw, pe->index, MVPP2_PRS_LU_VLAN);
	}
	/* Update ports' mask */
	mv_pp2x_prs_tcam_port_map_set(pe, port_map);

	mv_pp2x_prs_hw_write(hw, pe);

error:
	kfree(pe);

	return ret;
}

/* Get first free double vlan ai number */
static int mv_pp2x_prs_double_vlan_ai_free_get(struct pp2_hw *hw)
{
	int i;

	for (i = 1; i < MVPP2_PRS_DBL_VLANS_MAX; i++) {
		if (!hw->prs_double_vlans[i])
			return i;
	}

	return -EINVAL;
}

/* Search for existing double vlan entry */
static struct mv_pp2x_prs_entry *mv_pp2x_prs_double_vlan_find(struct pp2_hw
							      *hw,
							      unsigned short
							      tpid1,
							      unsigned short
							      tpid2)
{
	struct mv_pp2x_prs_entry *pe;
	int tid;

	pe = kcalloc(1, sizeof(*pe), GFP_KERNEL);
	if (!pe)
		return NULL;
	mv_pp2x_prs_tcam_lu_set(pe, MVPP2_PRS_LU_VLAN);

	/* Go through the all entries with MVPP2_PRS_LU_VLAN */
	for (tid = MVPP2_PE_FIRST_FREE_TID;
	     tid <= MVPP2_PE_LAST_FREE_TID; tid++) {
		unsigned int ri_mask;
		bool match;

		if (!hw->prs_shadow[tid].valid ||
		    hw->prs_shadow[tid].lu != MVPP2_PRS_LU_VLAN)
			continue;

		pe->index = tid;
		mv_pp2x_prs_hw_read(hw, pe);

		match = mv_pp2x_prs_tcam_data_cmp(pe, 0, swab16(tpid1)) &&
		    mv_pp2x_prs_tcam_data_cmp(pe, 4, swab16(tpid2));

		if (!match)
			continue;

		ri_mask = mv_pp2x_prs_sram_ri_get(pe) & MVPP2_PRS_RI_VLAN_MASK;
		if (ri_mask == MVPP2_PRS_RI_VLAN_DOUBLE)
			return pe;
	}
	kfree(pe);

	return NULL;
}

/* Add or update double vlan entry */
static int mv_pp2x_prs_double_vlan_add(struct pp2_hw *hw,
				       unsigned short tpid1,
				       unsigned short tpid2,
				       unsigned int port_map)
{
	struct mv_pp2x_prs_entry *pe;
	int tid_aux, tid, ai, ret = 0;

	pe = mv_pp2x_prs_double_vlan_find(hw, tpid1, tpid2);

	if (!pe) {
		/* Create new tcam entry */
		tid = mv_pp2x_prs_tcam_first_free(hw, MVPP2_PE_FIRST_FREE_TID,
						  MVPP2_PE_LAST_FREE_TID);
		if (tid < 0)
			return tid;

		pe = kcalloc(1, sizeof(*pe), GFP_KERNEL);
		if (!pe)
			return -ENOMEM;

		/* Set ai value for new double vlan entry */
		ai = mv_pp2x_prs_double_vlan_ai_free_get(hw);
		if (ai < 0) {
			ret = ai;
			goto error;
		}

		/* Get first single/triple vlan tid */
		for (tid_aux = MVPP2_PE_FIRST_FREE_TID;
		     tid_aux <= MVPP2_PE_LAST_FREE_TID; tid_aux++) {
			unsigned int ri_bits;

			if (!hw->prs_shadow[tid_aux].valid ||
			    hw->prs_shadow[tid_aux].lu != MVPP2_PRS_LU_VLAN)
				continue;

			pe->index = tid_aux;
			mv_pp2x_prs_hw_read(hw, pe);
			ri_bits = mv_pp2x_prs_sram_ri_get(pe);
			ri_bits &= MVPP2_PRS_RI_VLAN_MASK;
			if (ri_bits == MVPP2_PRS_RI_VLAN_SINGLE ||
			    ri_bits == MVPP2_PRS_RI_VLAN_TRIPLE)
				break;
		}

		if (tid >= tid_aux) {
			ret = -EINVAL;
			goto error;
		}

		memset(pe, 0, sizeof(struct mv_pp2x_prs_entry));
		mv_pp2x_prs_tcam_lu_set(pe, MVPP2_PRS_LU_VLAN);
		pe->index = tid;

		hw->prs_double_vlans[ai] = true;

		mv_pp2x_prs_match_etype(pe, 0, tpid1);
		mv_pp2x_prs_match_etype(pe, 4, tpid2);

		mv_pp2x_prs_sram_next_lu_set(pe, MVPP2_PRS_LU_VLAN);
		/* Shift 8 bytes - skip 2 vlan tags */
		mv_pp2x_prs_sram_shift_set(pe, 2 * MVPP2_VLAN_TAG_LEN,
					   MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);
		mv_pp2x_prs_sram_ri_update(pe, MVPP2_PRS_RI_VLAN_DOUBLE,
					   MVPP2_PRS_RI_VLAN_MASK);
		mv_pp2x_prs_sram_ai_update(pe, ai | MVPP2_PRS_DBL_VLAN_AI_BIT,
					   MVPP2_PRS_SRAM_AI_MASK);

		mv_pp2x_prs_shadow_set(hw, pe->index, MVPP2_PRS_LU_VLAN);
	}

	/* Update ports' mask */
	mv_pp2x_prs_tcam_port_map_set(pe, port_map);
	mv_pp2x_prs_hw_write(hw, pe);

error:
	kfree(pe);
	return ret;
}

/* IPv4 header parsing for fragmentation and L4 offset */
static int mv_pp2x_prs_ip4_proto(struct pp2_hw *hw, unsigned short proto,
				 unsigned int ri, unsigned int ri_mask)
{
	struct mv_pp2x_prs_entry pe;
	int tid;

	if ((proto != IPPROTO_TCP) && (proto != IPPROTO_UDP) &&
	    (proto != IPPROTO_IGMP))
		return -EINVAL;

	/* Not fragmented packet */
	tid = mv_pp2x_prs_tcam_first_free(hw, MVPP2_PE_FIRST_FREE_TID,
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
				    sizeof(struct iphdr) - 4,
				    MVPP2_PRS_SRAM_OP_SEL_UDF_ADD);
	mv_pp2x_prs_sram_ai_update(&pe, MVPP2_PRS_IPV4_DIP_AI_BIT,
				   MVPP2_PRS_IPV4_DIP_AI_BIT);
	mv_pp2x_prs_sram_ri_update(&pe, ri | MVPP2_PRS_RI_IP_FRAG_FALSE,
				   ri_mask | MVPP2_PRS_RI_IP_FRAG_MASK);

	mv_pp2x_prs_tcam_data_byte_set(&pe, 2, 0x00,
				       MVPP2_PRS_TCAM_PROTO_MASK_L);
	mv_pp2x_prs_tcam_data_byte_set(&pe, 3, 0x00, MVPP2_PRS_TCAM_PROTO_MASK);
	mv_pp2x_prs_tcam_data_byte_set(&pe, 5, proto,
				       MVPP2_PRS_TCAM_PROTO_MASK);
	mv_pp2x_prs_tcam_ai_update(&pe, 0, MVPP2_PRS_IPV4_DIP_AI_BIT);
	/* Unmask all ports */
	mv_pp2x_prs_tcam_port_map_set(&pe, MVPP2_PRS_PORT_MASK);

	/* Update shadow table and hw entry */
	mv_pp2x_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_IP4);
	mv_pp2x_prs_hw_write(hw, &pe);

	/* Fragmented packet */
	tid = mv_pp2x_prs_tcam_first_free(hw, MVPP2_PE_FIRST_FREE_TID,
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
	mv_pp2x_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_IP4);
	mv_pp2x_prs_hw_write(hw, &pe);

	return 0;
}

/* IPv4 L3 multicast or broadcast */
static int mv_pp2x_prs_ip4_cast(struct pp2_hw *hw, unsigned short l3_cast)
{
	struct mv_pp2x_prs_entry pe;
	int mask, tid;

	tid = mv_pp2x_prs_tcam_first_free(hw, MVPP2_PE_FIRST_FREE_TID,
					  MVPP2_PE_LAST_FREE_TID);
	if (tid < 0)
		return tid;

	memset(&pe, 0, sizeof(struct mv_pp2x_prs_entry));
	mv_pp2x_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_IP4);
	pe.index = tid;

	switch (l3_cast) {
	case MVPP2_PRS_L3_MULTI_CAST:
		mv_pp2x_prs_tcam_data_byte_set(&pe, 0, MVPP2_PRS_IPV4_MC,
					       MVPP2_PRS_IPV4_MC_MASK);
		mv_pp2x_prs_sram_ri_update(&pe, MVPP2_PRS_RI_L3_MCAST,
					   MVPP2_PRS_RI_L3_ADDR_MASK);
		break;
	case MVPP2_PRS_L3_BROAD_CAST:
		mask = MVPP2_PRS_IPV4_BC_MASK;
		mv_pp2x_prs_tcam_data_byte_set(&pe, 0, mask, mask);
		mv_pp2x_prs_tcam_data_byte_set(&pe, 1, mask, mask);
		mv_pp2x_prs_tcam_data_byte_set(&pe, 2, mask, mask);
		mv_pp2x_prs_tcam_data_byte_set(&pe, 3, mask, mask);
		mv_pp2x_prs_sram_ri_update(&pe, MVPP2_PRS_RI_L3_BCAST,
					   MVPP2_PRS_RI_L3_ADDR_MASK);
		break;
	default:
		return -EINVAL;
	}

	/* Finished: go to flow_id generation */
	mv_pp2x_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_FLOWS);
	mv_pp2x_prs_sram_bits_set(&pe, MVPP2_PRS_SRAM_LU_GEN_BIT, 1);

	mv_pp2x_prs_tcam_ai_update(&pe, MVPP2_PRS_IPV4_DIP_AI_BIT,
				   MVPP2_PRS_IPV4_DIP_AI_BIT);
	/* Unmask all ports */
	mv_pp2x_prs_tcam_port_map_set(&pe, MVPP2_PRS_PORT_MASK);

	/* Update shadow table and hw entry */
	mv_pp2x_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_IP4);
	mv_pp2x_prs_hw_write(hw, &pe);

	return 0;
}

/* Set entries for protocols over IPv6  */
static int mv_pp2x_prs_ip6_proto(struct pp2_hw *hw, unsigned short proto,
				 unsigned int ri, unsigned int ri_mask)
{
	struct mv_pp2x_prs_entry pe;
	int tid;

	if ((proto != IPPROTO_TCP) && (proto != IPPROTO_UDP) &&
	    (proto != IPPROTO_ICMPV6) && (proto != IPPROTO_IPIP))
		return -EINVAL;

	tid = mv_pp2x_prs_tcam_first_free(hw, MVPP2_PE_FIRST_FREE_TID,
					  MVPP2_PE_LAST_FREE_TID);
	if (tid < 0)
		return tid;

	memset(&pe, 0, sizeof(struct mv_pp2x_prs_entry));
	mv_pp2x_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_IP6);
	pe.index = tid;

	/* Finished: go to flow_id generation */
	mv_pp2x_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_FLOWS);
	mv_pp2x_prs_sram_bits_set(&pe, MVPP2_PRS_SRAM_LU_GEN_BIT, 1);
	mv_pp2x_prs_sram_ri_update(&pe, ri, ri_mask);
	mv_pp2x_prs_sram_offset_set(&pe, MVPP2_PRS_SRAM_UDF_TYPE_L4,
				    sizeof(struct ipv6hdr) - 6,
				    MVPP2_PRS_SRAM_OP_SEL_UDF_ADD);

	mv_pp2x_prs_tcam_data_byte_set(&pe, 0, proto,
				       MVPP2_PRS_TCAM_PROTO_MASK);
	mv_pp2x_prs_tcam_ai_update(&pe, MVPP2_PRS_IPV6_NO_EXT_AI_BIT,
				   MVPP2_PRS_IPV6_NO_EXT_AI_BIT);
	/* Unmask all ports */
	mv_pp2x_prs_tcam_port_map_set(&pe, MVPP2_PRS_PORT_MASK);

	/* Write HW */
	mv_pp2x_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_IP6);
	mv_pp2x_prs_hw_write(hw, &pe);

	return 0;
}

/* IPv6 L3 multicast entry */
static int mv_pp2x_prs_ip6_cast(struct pp2_hw *hw, unsigned short l3_cast)
{
	struct mv_pp2x_prs_entry pe;
	int tid;

	if (l3_cast != MVPP2_PRS_L3_MULTI_CAST)
		return -EINVAL;

	tid = mv_pp2x_prs_tcam_first_free(hw, MVPP2_PE_FIRST_FREE_TID,
					  MVPP2_PE_LAST_FREE_TID);
	if (tid < 0)
		return tid;

	memset(&pe, 0, sizeof(struct mv_pp2x_prs_entry));
	mv_pp2x_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_IP6);
	pe.index = tid;

	/* Finished: go to flow_id generation */
	mv_pp2x_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_IP6);
	mv_pp2x_prs_sram_ri_update(&pe, MVPP2_PRS_RI_L3_MCAST,
				   MVPP2_PRS_RI_L3_ADDR_MASK);
	mv_pp2x_prs_sram_ai_update(&pe, MVPP2_PRS_IPV6_NO_EXT_AI_BIT,
				   MVPP2_PRS_IPV6_NO_EXT_AI_BIT);
	/* Shift back to IPv6 NH */
	mv_pp2x_prs_sram_shift_set(&pe, -18, MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);

	mv_pp2x_prs_tcam_data_byte_set(&pe, 0, MVPP2_PRS_IPV6_MC,
				       MVPP2_PRS_IPV6_MC_MASK);
	mv_pp2x_prs_tcam_ai_update(&pe, 0, MVPP2_PRS_IPV6_NO_EXT_AI_BIT);
	/* Unmask all ports */
	mv_pp2x_prs_tcam_port_map_set(&pe, MVPP2_PRS_PORT_MASK);

	/* Update shadow table and hw entry */
	mv_pp2x_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_IP6);
	mv_pp2x_prs_hw_write(hw, &pe);

	return 0;
}

/* Configure vlan entries and detect up to 2 successive VLAN tags.
 * Possible options:
 * 0x8100, 0x88A8
 * 0x8100, 0x8100
 * 0x8100
 * 0x88A8
 */
static int mv_pp2x_prs_vlan_init(struct pp2_hw *hw)
{
	struct mv_pp2x_prs_entry pe;
	int err;

	hw->prs_double_vlans = kcalloc(MVPP2_PRS_DBL_VLANS_MAX, sizeof(bool), GFP_KERNEL);
	if (!hw->prs_double_vlans)
		return -ENOMEM;
	/* Double VLAN: 0x8100, 0x88A8 */
	err = mv_pp2x_prs_double_vlan_add(hw, ETH_P_8021Q, ETH_P_8021AD,
					  MVPP2_PRS_PORT_MASK);
	if (err)
		return err;

	/* Double VLAN: 0x8100, 0x8100 */
	err = mv_pp2x_prs_double_vlan_add(hw, ETH_P_8021Q, ETH_P_8021Q,
					  MVPP2_PRS_PORT_MASK);
	if (err)
		return err;

	/* Single VLAN: 0x88a8 */
	err = mv_pp2x_prs_vlan_add(hw, ETH_P_8021AD, MVPP2_PRS_SINGLE_VLAN_AI,
				   MVPP2_PRS_PORT_MASK);
	if (err)
		return err;

	/* Single VLAN: 0x8100 */
	err = mv_pp2x_prs_vlan_add(hw, ETH_P_8021Q, MVPP2_PRS_SINGLE_VLAN_AI,
				   MVPP2_PRS_PORT_MASK);
	if (err)
		return err;

	/* Set default double vlan entry */
	memset(&pe, 0, sizeof(struct mv_pp2x_prs_entry));
	mv_pp2x_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_VLAN);
	pe.index = MVPP2_PE_VLAN_DBL;

	mv_pp2x_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_L2);
	/* Clear ai for next iterations */
	mv_pp2x_prs_sram_ai_update(&pe, 0, MVPP2_PRS_SRAM_AI_MASK);
	mv_pp2x_prs_sram_ri_update(&pe, MVPP2_PRS_RI_VLAN_DOUBLE,
				   MVPP2_PRS_RI_VLAN_MASK);

	mv_pp2x_prs_tcam_ai_update(&pe, MVPP2_PRS_DBL_VLAN_AI_BIT,
				   MVPP2_PRS_DBL_VLAN_AI_BIT);
	/* Unmask all ports */
	mv_pp2x_prs_tcam_port_map_set(&pe, MVPP2_PRS_PORT_MASK);

	/* Update shadow table and hw entry */
	mv_pp2x_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_VLAN);
	mv_pp2x_prs_hw_write(hw, &pe);

	/* Set default vlan none entry */
	memset(&pe, 0, sizeof(struct mv_pp2x_prs_entry));
	mv_pp2x_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_VLAN);
	pe.index = MVPP2_PE_VLAN_NONE;

	mv_pp2x_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_L2);
	mv_pp2x_prs_sram_ri_update(&pe, MVPP2_PRS_RI_VLAN_NONE,
				   MVPP2_PRS_RI_VLAN_MASK);

	/* Unmask all ports */
	mv_pp2x_prs_tcam_port_map_set(&pe, MVPP2_PRS_PORT_MASK);

	/* Update shadow table and hw entry */
	mv_pp2x_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_VLAN);
	mv_pp2x_prs_hw_write(hw, &pe);

	return 0;
}

/* Set entries for PPPoE ethertype */
static int mv_pp2x_prs_pppoe_init(struct pp2_hw *hw)
{
	struct mv_pp2x_prs_entry pe;
	int tid;

	/* IPv4 over PPPoE with options */
	tid = mv_pp2x_prs_tcam_first_free(hw, MVPP2_PE_FIRST_FREE_TID,
					  MVPP2_PE_LAST_FREE_TID);
	if (tid < 0)
		return tid;

	memset(&pe, 0, sizeof(struct mv_pp2x_prs_entry));
	mv_pp2x_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_PPPOE);
	pe.index = tid;

	mv_pp2x_prs_match_etype(&pe, 0, PPP2_IP);

	mv_pp2x_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_IP4);
	mv_pp2x_prs_sram_ri_update(&pe, MVPP2_PRS_RI_L3_IP4_OPT,
				   MVPP2_PRS_RI_L3_PROTO_MASK);
	/* Skip eth_type + 4 bytes of IP header */
	mv_pp2x_prs_sram_shift_set(&pe, MVPP2_ETH_TYPE_LEN + 4,
				   MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);
	/* Set L3 offset */
	mv_pp2x_prs_sram_offset_set(&pe, MVPP2_PRS_SRAM_UDF_TYPE_L3,
				    MVPP2_ETH_TYPE_LEN,
				    MVPP2_PRS_SRAM_OP_SEL_UDF_ADD);

	/* Update shadow table and hw entry */
	mv_pp2x_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_PPPOE);
	mv_pp2x_prs_hw_write(hw, &pe);

	/* IPv4 over PPPoE without options */
	tid = mv_pp2x_prs_tcam_first_free(hw, MVPP2_PE_FIRST_FREE_TID,
					  MVPP2_PE_LAST_FREE_TID);
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
	mv_pp2x_prs_sram_ri_update(&pe, MVPP2_PRS_RI_L3_IP4,
				   MVPP2_PRS_RI_L3_PROTO_MASK);

	/* Update shadow table and hw entry */
	mv_pp2x_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_PPPOE);
	mv_pp2x_prs_hw_write(hw, &pe);

	/* IPv6 over PPPoE */
	tid = mv_pp2x_prs_tcam_first_free(hw, MVPP2_PE_FIRST_FREE_TID,
					  MVPP2_PE_LAST_FREE_TID);
	if (tid < 0)
		return tid;

	memset(&pe, 0, sizeof(struct mv_pp2x_prs_entry));
	mv_pp2x_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_PPPOE);
	pe.index = tid;

	mv_pp2x_prs_match_etype(&pe, 0, PPP2_IPV6);

	mv_pp2x_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_IP6);
	mv_pp2x_prs_sram_ri_update(&pe, MVPP2_PRS_RI_L3_IP6,
				   MVPP2_PRS_RI_L3_PROTO_MASK);
	/* Skip eth_type + 4 bytes of IPv6 header */
	mv_pp2x_prs_sram_shift_set(&pe, MVPP2_ETH_TYPE_LEN + 4,
				   MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);
	/* Set L3 offset */
	mv_pp2x_prs_sram_offset_set(&pe, MVPP2_PRS_SRAM_UDF_TYPE_L3,
				    MVPP2_ETH_TYPE_LEN,
				    MVPP2_PRS_SRAM_OP_SEL_UDF_ADD);

	/* Update shadow table and hw entry */
	mv_pp2x_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_PPPOE);
	mv_pp2x_prs_hw_write(hw, &pe);

	/* Non-IP over PPPoE */
	tid = mv_pp2x_prs_tcam_first_free(hw, MVPP2_PE_FIRST_FREE_TID,
					  MVPP2_PE_LAST_FREE_TID);
	if (tid < 0)
		return tid;

	memset(&pe, 0, sizeof(struct mv_pp2x_prs_entry));
	mv_pp2x_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_PPPOE);
	pe.index = tid;

	mv_pp2x_prs_sram_ri_update(&pe, MVPP2_PRS_RI_L3_UN,
				   MVPP2_PRS_RI_L3_PROTO_MASK);

	/* Finished: go to flow_id generation */
	mv_pp2x_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_FLOWS);
	mv_pp2x_prs_sram_bits_set(&pe, MVPP2_PRS_SRAM_LU_GEN_BIT, 1);
	/* Set L3 offset even if it's unknown L3 */
	mv_pp2x_prs_sram_offset_set(&pe, MVPP2_PRS_SRAM_UDF_TYPE_L3,
				    MVPP2_ETH_TYPE_LEN,
				    MVPP2_PRS_SRAM_OP_SEL_UDF_ADD);

	/* Update shadow table and hw entry */
	mv_pp2x_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_PPPOE);
	mv_pp2x_prs_hw_write(hw, &pe);

	return 0;
}

/* Initialize entries for IPv4 */
static int mv_pp2x_prs_ip4_init(struct pp2_hw *hw)
{
	struct mv_pp2x_prs_entry pe;
	int err;

	/* Set entries for TCP, UDP and IGMP over IPv4 */
	err = mv_pp2x_prs_ip4_proto(hw, IPPROTO_TCP, MVPP2_PRS_RI_L4_TCP,
				    MVPP2_PRS_RI_L4_PROTO_MASK);

	if (err)
		return err;

	err = mv_pp2x_prs_ip4_proto(hw, IPPROTO_UDP, MVPP2_PRS_RI_L4_UDP,
				    MVPP2_PRS_RI_L4_PROTO_MASK);

	if (err)
		return err;

	err = mv_pp2x_prs_ip4_proto(hw, IPPROTO_IGMP,
				    MVPP2_PRS_RI_CPU_CODE_RX_SPEC |
				    MVPP2_PRS_RI_UDF3_RX_SPECIAL,
				    MVPP2_PRS_RI_CPU_CODE_MASK |
				    MVPP2_PRS_RI_UDF3_MASK);

	if (err)
		return err;

	/* IPv4 Broadcast */
	err = mv_pp2x_prs_ip4_cast(hw, MVPP2_PRS_L3_BROAD_CAST);

	if (err)
		return err;

	/* IPv4 Multicast */
	err = mv_pp2x_prs_ip4_cast(hw, MVPP2_PRS_L3_MULTI_CAST);

	if (err)
		return err;

	/* Default IPv4 entry for unknown protocols */
	memset(&pe, 0, sizeof(struct mv_pp2x_prs_entry));
	mv_pp2x_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_IP4);
	pe.index = MVPP2_PE_IP4_PROTO_UN;

	/* Set next lu to IPv4 */
	mv_pp2x_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_IP4);
	mv_pp2x_prs_sram_shift_set(&pe, 12, MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);
	/* Set L4 offset */
	mv_pp2x_prs_sram_offset_set(&pe, MVPP2_PRS_SRAM_UDF_TYPE_L4,
				    sizeof(struct iphdr) - 4,
				    MVPP2_PRS_SRAM_OP_SEL_UDF_ADD);
	mv_pp2x_prs_sram_ai_update(&pe, MVPP2_PRS_IPV4_DIP_AI_BIT,
				   MVPP2_PRS_IPV4_DIP_AI_BIT);
	mv_pp2x_prs_sram_ri_update(&pe, MVPP2_PRS_RI_L4_OTHER,
				   MVPP2_PRS_RI_L4_PROTO_MASK);

	mv_pp2x_prs_tcam_ai_update(&pe, 0, MVPP2_PRS_IPV4_DIP_AI_BIT);
	/* Unmask all ports */
	mv_pp2x_prs_tcam_port_map_set(&pe, MVPP2_PRS_PORT_MASK);

	/* Update shadow table and hw entry */
	mv_pp2x_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_IP4);
	mv_pp2x_prs_hw_write(hw, &pe);

	/* Default IPv4 entry for unicast address */
	memset(&pe, 0, sizeof(struct mv_pp2x_prs_entry));
	mv_pp2x_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_IP4);
	pe.index = MVPP2_PE_IP4_ADDR_UN;

	/* Finished: go to flow_id generation */
	mv_pp2x_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_FLOWS);
	mv_pp2x_prs_sram_bits_set(&pe, MVPP2_PRS_SRAM_LU_GEN_BIT, 1);
	mv_pp2x_prs_sram_ri_update(&pe, MVPP2_PRS_RI_L3_UCAST,
				   MVPP2_PRS_RI_L3_ADDR_MASK);

	mv_pp2x_prs_tcam_ai_update(&pe, MVPP2_PRS_IPV4_DIP_AI_BIT,
				   MVPP2_PRS_IPV4_DIP_AI_BIT);
	/* Unmask all ports */
	mv_pp2x_prs_tcam_port_map_set(&pe, MVPP2_PRS_PORT_MASK);

	/* Update shadow table and hw entry */
	mv_pp2x_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_IP4);
	mv_pp2x_prs_hw_write(hw, &pe);
	return 0;
}

/* Initialize entries for IPv6 */
static int mv_pp2x_prs_ip6_init(struct pp2_hw *hw)
{
	struct mv_pp2x_prs_entry pe;
	int tid, err;

	/* Set entries for TCP, UDP and ICMP over IPv6 */
	err = mv_pp2x_prs_ip6_proto(hw, IPPROTO_TCP,
				    MVPP2_PRS_RI_L4_TCP,
				    MVPP2_PRS_RI_L4_PROTO_MASK);
	if (err)
		return err;

	err = mv_pp2x_prs_ip6_proto(hw, IPPROTO_UDP,
				    MVPP2_PRS_RI_L4_UDP,
				    MVPP2_PRS_RI_L4_PROTO_MASK);
	if (err)
		return err;

	err = mv_pp2x_prs_ip6_proto(hw, IPPROTO_ICMPV6,
				    MVPP2_PRS_RI_CPU_CODE_RX_SPEC |
				    MVPP2_PRS_RI_UDF3_RX_SPECIAL,
				    MVPP2_PRS_RI_CPU_CODE_MASK |
				    MVPP2_PRS_RI_UDF3_MASK);
	if (err)
		return err;

	/* IPv4 is the last header. This is similar case as 6-TCP or 17-UDP */
	/* Result Info: UDF7=1, DS lite */
	err = mv_pp2x_prs_ip6_proto(hw, IPPROTO_IPIP,
				    MVPP2_PRS_RI_UDF7_IP6_LITE,
				    MVPP2_PRS_RI_UDF7_MASK);
	if (err)
		return err;

	/* IPv6 multicast */
	err = mv_pp2x_prs_ip6_cast(hw, MVPP2_PRS_L3_MULTI_CAST);
	if (err)
		return err;

	/* Entry for checking hop limit */
	tid = mv_pp2x_prs_tcam_first_free(hw, MVPP2_PE_FIRST_FREE_TID,
					  MVPP2_PE_LAST_FREE_TID);
	if (tid < 0)
		return tid;

	memset(&pe, 0, sizeof(struct mv_pp2x_prs_entry));
	mv_pp2x_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_IP6);
	pe.index = tid;

	/* Finished: go to flow_id generation */
	mv_pp2x_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_FLOWS);
	mv_pp2x_prs_sram_bits_set(&pe, MVPP2_PRS_SRAM_LU_GEN_BIT, 1);
	mv_pp2x_prs_sram_ri_update(&pe, MVPP2_PRS_RI_L3_UN |
				   MVPP2_PRS_RI_DROP_MASK,
				   MVPP2_PRS_RI_L3_PROTO_MASK |
				   MVPP2_PRS_RI_DROP_MASK);

	mv_pp2x_prs_tcam_data_byte_set(&pe, 1, 0x00, MVPP2_PRS_IPV6_HOP_MASK);
	mv_pp2x_prs_tcam_ai_update(&pe, MVPP2_PRS_IPV6_NO_EXT_AI_BIT,
				   MVPP2_PRS_IPV6_NO_EXT_AI_BIT);

	/* Update shadow table and hw entry */
	mv_pp2x_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_IP4);
	mv_pp2x_prs_hw_write(hw, &pe);

	/* Default IPv6 entry for unknown protocols */
	memset(&pe, 0, sizeof(struct mv_pp2x_prs_entry));
	mv_pp2x_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_IP6);
	pe.index = MVPP2_PE_IP6_PROTO_UN;

	/* Finished: go to flow_id generation */
	mv_pp2x_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_FLOWS);
	mv_pp2x_prs_sram_bits_set(&pe, MVPP2_PRS_SRAM_LU_GEN_BIT, 1);
	mv_pp2x_prs_sram_ri_update(&pe, MVPP2_PRS_RI_L4_OTHER,
				   MVPP2_PRS_RI_L4_PROTO_MASK);
	/* Set L4 offset relatively to our current place */
	mv_pp2x_prs_sram_offset_set(&pe, MVPP2_PRS_SRAM_UDF_TYPE_L4,
				    sizeof(struct ipv6hdr) - 4,
				    MVPP2_PRS_SRAM_OP_SEL_UDF_ADD);

	mv_pp2x_prs_tcam_ai_update(&pe, MVPP2_PRS_IPV6_NO_EXT_AI_BIT,
				   MVPP2_PRS_IPV6_NO_EXT_AI_BIT);
	/* Unmask all ports */
	mv_pp2x_prs_tcam_port_map_set(&pe, MVPP2_PRS_PORT_MASK);

	/* Update shadow table and hw entry */
	mv_pp2x_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_IP4);
	mv_pp2x_prs_hw_write(hw, &pe);

	/* Default IPv6 entry for unknown ext protocols */
	memset(&pe, 0, sizeof(struct mv_pp2x_prs_entry));
	mv_pp2x_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_IP6);
	pe.index = MVPP2_PE_IP6_EXT_PROTO_UN;

	/* Finished: go to flow_id generation */
	mv_pp2x_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_FLOWS);
	mv_pp2x_prs_sram_bits_set(&pe, MVPP2_PRS_SRAM_LU_GEN_BIT, 1);
	mv_pp2x_prs_sram_ri_update(&pe, MVPP2_PRS_RI_L4_OTHER,
				   MVPP2_PRS_RI_L4_PROTO_MASK);

	mv_pp2x_prs_tcam_ai_update(&pe, MVPP2_PRS_IPV6_EXT_AI_BIT,
				   MVPP2_PRS_IPV6_EXT_AI_BIT);
	/* Unmask all ports */
	mv_pp2x_prs_tcam_port_map_set(&pe, MVPP2_PRS_PORT_MASK);

	/* Update shadow table and hw entry */
	mv_pp2x_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_IP4);
	mv_pp2x_prs_hw_write(hw, &pe);

	/* Default IPv6 entry for unicast address */
	memset(&pe, 0, sizeof(struct mv_pp2x_prs_entry));
	mv_pp2x_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_IP6);
	pe.index = MVPP2_PE_IP6_ADDR_UN;

	/* Finished: go to IPv6 again */
	mv_pp2x_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_IP6);
	mv_pp2x_prs_sram_ri_update(&pe, MVPP2_PRS_RI_L3_UCAST,
				   MVPP2_PRS_RI_L3_ADDR_MASK);
	mv_pp2x_prs_sram_ai_update(&pe, MVPP2_PRS_IPV6_NO_EXT_AI_BIT,
				   MVPP2_PRS_IPV6_NO_EXT_AI_BIT);
	/* Shift back to IPV6 NH */
	mv_pp2x_prs_sram_shift_set(&pe, -18, MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);

	mv_pp2x_prs_tcam_ai_update(&pe, 0, MVPP2_PRS_IPV6_NO_EXT_AI_BIT);
	/* Unmask all ports */
	mv_pp2x_prs_tcam_port_map_set(&pe, MVPP2_PRS_PORT_MASK);

	/* Update shadow table and hw entry */
	mv_pp2x_prs_shadow_set(hw, pe.index, MVPP2_PRS_LU_IP6);
	mv_pp2x_prs_hw_write(hw, &pe);

	return 0;
}

/* Parser default initialization */
int mv_pp2x_prs_default_init(struct pp2_hw *hw)
{
	int err, index, i;

	/* Enable tcam table */
	pp2_reg_write(hw->base[0].va,
		      MVPP2_PRS_TCAM_CTRL_REG, MVPP2_PRS_TCAM_EN_MASK);

	/* Clear all tcam and sram entries */
	for (index = 0; index < MVPP2_PRS_TCAM_SRAM_SIZE; index++) {
		pp2_reg_write(hw->base[0].va, MVPP2_PRS_TCAM_IDX_REG, index);
		for (i = 0; i < MVPP2_PRS_TCAM_WORDS; i++)
			pp2_reg_write(hw->base[0].va,
				      MVPP2_PRS_TCAM_DATA_REG(i), 0);

		pp2_reg_write(hw->base[0].va, MVPP2_PRS_SRAM_IDX_REG, index);
		for (i = 0; i < MVPP2_PRS_SRAM_WORDS; i++)
			pp2_reg_write(hw->base[0].va,
				      MVPP2_PRS_SRAM_DATA_REG(i), 0);
	}

	/* Invalidate all tcam entries */
	for (index = 0; index < MVPP2_PRS_TCAM_SRAM_SIZE; index++)
		mv_pp2x_prs_hw_inv(hw, index);

	hw->prs_shadow = kcalloc(MVPP2_PRS_TCAM_SRAM_SIZE,
				sizeof(struct mv_pp2x_prs_shadow), GFP_KERNEL);

	if (!hw->prs_shadow)
		return -ENOMEM;

	/* Always start from lookup = 0 */
	for (index = 0; index < MVPP2_MAX_PORTS; index++)
		mv_pp2x_prs_hw_port_init(hw, index, MVPP2_PRS_LU_MH,
					 MVPP2_PRS_PORT_LU_MAX, 0);

	mv_pp2x_prs_def_flow_init(hw);

	mv_pp2x_prs_mh_init(hw);

	mv_pp2x_prs_mac_init(hw);

	mv_pp2x_prs_dsa_init(hw);

	err = mv_pp2x_prs_etype_init(hw);
	if (err)
		return err;

	err = mv_pp2x_prs_vlan_init(hw);
	if (err)
		return err;
	err = mv_pp2x_prs_pppoe_init(hw);
	if (err)
		return err;

	err = mv_pp2x_prs_ip6_init(hw);
	if (err)
		return err;

	err = mv_pp2x_prs_ip4_init(hw);
	if (err)
		return err;
	return 0;
}

/* [TODO] (classifier) semi static configuration
 * Need to review all values and if necessary to init values per port
 */
void ppdk_cls_default_config_set(struct pp2_inst *inst)
{
	if (unlikely(!inst))
		return;

	inst->pp2_cfg.cos_cfg.num_cos_queues = 1;
	inst->pp2_cfg.cos_cfg.cos_classifier = 0;
	inst->pp2_cfg.cos_cfg.default_cos = 0;
	inst->pp2_cfg.cos_cfg.reserved = 0;
	inst->pp2_cfg.cos_cfg.pri_map = 0x3210;

	inst->pp2_cfg.rss_cfg.rss_mode = 0;
	inst->pp2_cfg.rss_cfg.dflt_cpu = 0; /* non IP pkts */
	inst->pp2_cfg.rss_cfg.rss_en   = 0;

	inst->pp2_cfg.first_bm_pool = 0;
	inst->pp2_cfg.first_sw_thread = 0;
	inst->pp2_cfg.first_log_rxq = 0;
	inst->pp2_cfg.queue_mode = 0;
	inst->pp2_cfg.rx_cpu_map = 0x00;

	inst->cpu_map = 0x01;

	inst->pp2xdata.pp2x_max_port_rxqs = 32;
}

int mv_pp2x_open_cls(struct pp2_port *port)
{
	unsigned char mac_bcast[ETH_ALEN] = {
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff
	};
	struct pp2_hw *hw = &port->parent->hw;
	int err;
	u32 cpu_width = 0, cos_width = 0, port_rxq_width = 0;
	u8 bound_cpu_first_rxq;
	u8 cos_classifer = port->parent->pp2_cfg.cos_cfg.cos_classifier;

	/* Calculate width */
	mv_pp2x_width_calc(port->parent, &cpu_width, &cos_width, &port_rxq_width);
	if (cpu_width + cos_width > port_rxq_width) {
		err = -1;
		pr_err("cpu or cos queue width invalid\n");
		return err;
	}
	pr_debug("PPDK:\t%s\t cpu_width:%d, cos_width:%d, port_rxq_width:%d\n",
		__func__, cpu_width, cos_width, port_rxq_width);

	err = mv_pp2x_prs_mac_da_accept(hw, port->id, mac_bcast, true);
	if (err) {
		pr_err("mv_pp2x_prs_mac_da_accept BC failed\n");
		return err;
	}

	err = mv_pp2x_prs_mac_da_accept(hw, port->id, (const uint8_t *)port->mac_data.mac, true);
	if (err) {
		pr_err("mv_pp2x_prs_mac_da_accept M2M failed\n");
		return err;
	}

	err = mv_pp2x_prs_tag_mode_set(hw, port->id, MVPP2_TAG_TYPE_MH);

	if (err) {
		pr_err("mv_pp2x_prs_tag_mode_set failed\n");
		return err;
	}

	err = mv_pp2x_prs_def_flow(port);
	if (err) {
		pr_err("mv_pp2x_prs_def_flow failed\n");
		return err;
	}

	err = mv_pp2x_prs_flow_set(port);
	if (err) {
		pr_err("mv_pp2x_prs_flow_set failed\n");
		return err;
	}

	/* Set CoS classifier */
	err = mv_pp2x_cos_classifier_set(port, cos_classifer);
	if (err) {
		pr_err("cannot set cos classifier\n");
		return err;
	}

	/* Init C2 rules */
	bound_cpu_first_rxq = mv_pp2x_bound_cpu_first_rxq_calc(port);
	err = mv_pp2x_cls_c2_rule_set(port, bound_cpu_first_rxq);
	if (err) {
		pr_err("cannot init C2 rules\n");
		return err;
	}

	/* Assign rss table for rxq belong to this port */
	err = mv_pp22_rss_rxq_set(port, cos_width);
	if (err) {
		pr_err("cannot allocate rss table for rxq\n");
		return err;
	}
	/* RSS related config */
	if (port->parent->pp2_cfg.queue_mode == MVPP2_QDIST_MULTI_MODE) {
		/* Set RSS mode */
		err = mv_pp22_rss_mode_set(port,
					   port->parent->pp2_cfg.rss_cfg.
					   rss_mode);
		if (err) {
			pr_err("cannot set rss mode\n");
			return err;
		}

		/* Init RSS table */
		err = mv_pp22_rss_rxfh_indir_set(port);
		if (err) {
			pr_err("cannot init rss rxfh indir\n");
			return err;
		}

		/* Set rss default CPU only when rss enabled */
		if (port->parent->pp2_cfg.rss_cfg.rss_en) {
			err = mv_pp22_rss_default_cpu_set(port,
							  port->parent->pp2_cfg.
							  rss_cfg.dflt_cpu);
			if (err) {
				pr_err("cannot set rss default cpu\n");
				return err;
			}
		}
	}
	return 0;
}

int mv_pp2x_prs_update_mac_da(struct pp2_port *port, const uint8_t *da)
{
	int err;
	u8 old_da[ETH_ALEN];
	struct pp2_mac_data *mac = &port->mac_data;
	struct pp2_hw *hw = &port->parent->hw;

	if (mv_eaddr_identical(da, mac->mac))
		return 0;

	/* Store current MAC */
	mv_cp_eaddr(old_da, mac->mac);

	/* Remove old parser entry */
	err = mv_pp2x_prs_mac_da_accept(hw, port->id, mac->mac, false);
	if (err)
		return err;

	/* Set addr in the device */
	mv_cp_eaddr(mac->mac, da);

	/* Add new parser entry */
	err = mv_pp2x_prs_mac_da_accept(hw, port->id, da, true);
	if (err) {
		/* Restore addr in the device */
		mv_cp_eaddr(mac->mac, old_da);
		return err;
	}

	return 0;
}

static bool mv_pp2x_mac_in_uc_list(struct pp2_port *port, const uint8_t *da)
{
	if (mv_eaddr_identical(da, (const uint8_t *)&port->mac_data.mac))
		return true;
	return false;
}

static bool mv_pp2x_mac_in_mc_list(struct pp2_port *port, const uint8_t *da)
{
	if (mv_eaddr_identical(da, (const uint8_t *)&port->mac_data.mac))
		return true;
	return false;
}

/* Delete port's uc/mc/bc simple (not range) entries with options */
void mv_pp2x_prs_mac_entry_del(struct pp2_port *port,
			       enum mv_pp2x_l2_cast l2_cast,
			       enum mv_pp2x_mac_del_option op)
{
	struct mv_pp2x_prs_entry pe;
	struct pp2_hw *hw = &port->parent->hw;
	int index, tid;

	for (tid = MVPP2_PE_MAC_RANGE_START;
	     tid <= MVPP2_PE_MAC_RANGE_END; tid++) {
		unsigned char da[ETH_ALEN], da_mask[ETH_ALEN];

		if (!hw->prs_shadow[tid].valid ||
		    (hw->prs_shadow[tid].lu != MVPP2_PRS_LU_MAC) ||
		   (hw->prs_shadow[tid].udf != MVPP2_PRS_UDF_MAC_DEF))
			continue;

		/* Only simple mac entries */
		pe.index = tid;
		mv_pp2x_prs_hw_read(hw, &pe);

		/* Read mac addr from entry */
		for (index = 0; index < ETH_ALEN; index++)
			mv_pp2x_prs_tcam_data_byte_get(&pe, index, &da[index],
						       &da_mask[index]);
		switch (l2_cast) {
		case MVPP2_PRS_MAC_UC:
			/* Do not delete M2M entry */
			if (mv_check_eaddr_uc(da) &&
			    !mv_eaddr_identical(da, (const uint8_t *)&port->mac_data.mac)) {
				if (op == MVPP2_DEL_MAC_NOT_IN_LIST &&
				    mv_pp2x_mac_in_uc_list(port, da))
					continue;
				/* Delete this entry */
				mv_pp2x_prs_mac_da_accept(hw, port->id, da, false);
			}
			break;
		case MVPP2_PRS_MAC_MC:
			if (mv_check_eaddr_mc(da) &&
			    !mv_check_eaddr_bc(da)) {
				if (op == MVPP2_DEL_MAC_NOT_IN_LIST &&
				    mv_pp2x_mac_in_mc_list(port, da))
					continue;
				/* Delete this entry */
				mv_pp2x_prs_mac_da_accept(hw, port->id, da, false);
			}
			break;
		case MVPP2_PRS_MAC_BC:
			if (mv_check_eaddr_bc(da))
				/* Delete this entry */
				mv_pp2x_prs_mac_da_accept(hw, port->id, da, false);
			break;
		}
	}
}

/* C3 engine */

/********************************************************************************/
/*		C3 Common utilities						*/
/********************************************************************************/
static void pp2_cls_c3_shadow_set(int hek_size, int index, int ext_index)
{
	pp2_cls_c3_shadow_tbl[index].size = hek_size;

	if (hek_size > MVPP2_CLS_C3_HEK_BYTES) {
		pp2_cls_c3_shadow_tbl[index].ext_ptr = ext_index;
		pp2_cls_c3_shadow_ext_tbl[ext_index] = IN_USE;
	} else {
		pp2_cls_c3_shadow_tbl[index].ext_ptr = NOT_IN_USE;
	}
}

/*-----------------------------------------------------------------------------*/
void pp2_cls_c3_shadow_get(int index, int *hek_size, int *ext_index)
{
	*hek_size = pp2_cls_c3_shadow_tbl[index].size;

	if (pp2_cls_c3_shadow_tbl[index].size > MVPP2_CLS_C3_HEK_BYTES)
		*ext_index = pp2_cls_c3_shadow_tbl[index].ext_ptr;
	else
		*ext_index = 0;
}

/*-----------------------------------------------------------------------------*/
void pp2_cls_c3_shadow_init(void)
{
	/* clear hash shadow and extension shadow */
	int index;

	for (index = 0; index < MVPP2_CLS_C3_HASH_TBL_SIZE; index++) {
		pp2_cls_c3_shadow_tbl[index].size = 0;
		pp2_cls_c3_shadow_tbl[index].ext_ptr = NOT_IN_USE;
	}

	for (index = 0; index < MVPP2_CLS_C3_EXT_TBL_SIZE; index++)
		pp2_cls_c3_shadow_ext_tbl[index] = NOT_IN_USE;
}

/*-----------------------------------------------------------------------------*/
int pp2_cls_c3_shadow_free_get(void)
{
	int index;

	/* Go through the all entires from first to last */
	for (index = 0; index < MVPP2_CLS_C3_HASH_TBL_SIZE; index++) {
		if (!pp2_cls_c3_shadow_tbl[index].size)
			break;
	}
	return index;
}

/*-----------------------------------------------------------------------------*/
int pp2_cls_c3_shadow_ext_free_get(void)
{
	int index;

	/* Go through the all entires from first to last */
	for (index = 0; index < MVPP2_CLS_C3_EXT_TBL_SIZE; index++) {
		if (pp2_cls_c3_shadow_ext_tbl[index] == NOT_IN_USE)
			break;
	}
	return index;
}

/*-----------------------------------------------------------------------------*/
int pp2_cls_c3_shadow_ext_status_get(int index)
{
	return pp2_cls_c3_shadow_ext_tbl[index];
}

/*-----------------------------------------------------------------------------*/
void pp2_cls_c3_shadow_clear(int index)
{
	int ext_ptr;

	pp2_cls_c3_shadow_tbl[index].size = 0;
	ext_ptr = pp2_cls_c3_shadow_tbl[index].ext_ptr;

	if (ext_ptr != NOT_IN_USE)
		pp2_cls_c3_shadow_ext_tbl[ext_ptr] = NOT_IN_USE;

	pp2_cls_c3_shadow_tbl[index].ext_ptr = NOT_IN_USE;
}

/*-------------------------------------------------------------------------------*/
/* retun 1 scan procedure completed							  */
/*-------------------------------------------------------------------------------*/
static int pp2_cls_c3_scan_complete(uintptr_t cpu_slot)
{
	u32 reg_val;

	reg_val = pp2_reg_read(cpu_slot, MVPP2_CLS3_STATE_REG);
	reg_val &= MVPP2_CLS3_STATE_SC_DONE_MASK;
	reg_val >>= MVPP2_CLS3_STATE_SC_DONE;

	return reg_val;
}

/*-------------------------------------------------------------------------------*/
/* return 1 if that the last CPU access (Query,Add or Delete) was completed			  */
/*-------------------------------------------------------------------------------*/
static int pp2_cls_c3_cpu_done(uintptr_t cpu_slot)
{
	u32 reg_val;

	reg_val = pp2_reg_read(cpu_slot, MVPP2_CLS3_STATE_REG);
	reg_val &= MVPP2_CLS3_STATE_CPU_DONE_MASK;
	reg_val >>= MVPP2_CLS3_STATE_CPU_DONE;
	return reg_val;
}

/*-------------------------------------------------------------------------------*/
/* 0x0  "ScanCompleted"  scan completed and the scan results are ready in hardware		  */
/* 0x1  "HitCountersClear"  The engine is clearing the Hit Counters				  */
/* 0x2  "ScanWait"  The engine waits for the scan delay timer				  */
/* 0x3  "ScanInProgress"  The scan process is in progress					  */
/*-------------------------------------------------------------------------------*/
static int pp2_cls_c3_scan_state_get(uintptr_t cpu_slot, u32 *state)
{
	u32 reg_val;

	if (mv_pp2x_ptr_validate(state))
		return -EINVAL;

	reg_val = pp2_reg_read(cpu_slot, MVPP2_CLS3_STATE_REG);
	reg_val &= MVPP2_CLS3_STATE_SC_STATE_MASK;
	reg_val >>= MVPP2_CLS3_STATE_SC_STATE;
	*state = reg_val;

	return 0;
}

/*-------------------------------------------------------------------------------*/
/* return 1 if counters clearing is completed						  */
/*-------------------------------------------------------------------------------*/

static int pp2_cls_c3_hit_cntr_clear_done(uintptr_t cpu_slot)
{
	u32 reg_val;

	reg_val = pp2_reg_read(cpu_slot, MVPP2_CLS3_STATE_REG);
	reg_val &= MVPP2_CLS3_STATE_CLEAR_CTR_DONE_MASK;
	reg_val >>= MVPP2_CLS3_STATE_CLEAR_CTR_DONE;
	return reg_val;
}

/*-------------------------------------------------------------------------------*/
void pp2_cls_c3_sw_clear(struct pp2_cls_c3_entry *c3)
{
	memset(c3, 0, sizeof(struct pp2_cls_c3_entry));
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_init(uintptr_t cpu_slot)
{
	int rc;

	pp2_cls_c3_shadow_init();
	rc = pp2_cls_c3_hit_cntrs_clear_all(cpu_slot);
	return rc;
}

/*-------------------------------------------------------------------------------*/
/* Add entry to hash table								  */
/* ext_index used only if hek size < 12						  */
/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_hw_add(uintptr_t cpu_slot, struct pp2_cls_c3_entry *c3, int index, int ext_index)
{
	int reg_start_ind, hek_size, iter = 0;
	u32 reg_val = 0;

	if (mv_pp2x_ptr_validate(c3))
		return -EINVAL;

	if (mv_pp2x_range_validate(index, 0, MVPP2_CLS3_HASH_OP_TBL_ADDR_MAX))
		return -EINVAL;

	c3->index = index;

	/* write key control */
	pp2_reg_write(cpu_slot, MVPP2_CLS3_KEY_CTRL_REG, c3->key.key_ctrl);

	hek_size = ((c3->key.key_ctrl & KEY_CTRL_HEK_SIZE_MASK) >> KEY_CTRL_HEK_SIZE);

	if (hek_size > MVPP2_CLS_C3_HEK_BYTES) {
		/* Extension */

		if (mv_pp2x_range_validate(ext_index, 0, MVPP2_CLS3_HASH_OP_EXT_TBL_ADDR_MAX))
			return -EINVAL;

		c3->ext_index = ext_index;
		reg_val |= (ext_index << MVPP2_CLS3_HASH_OP_EXT_TBL_ADDR);

		/* write 9 hek registers */
		reg_start_ind = 0;
	} else
		/* write 3 hek registers */
		reg_start_ind = 6;

	for (; reg_start_ind < MVPP2_CLS_C3_EXT_HEK_WORDS; reg_start_ind++)
		pp2_reg_write(cpu_slot, MVPP2_CLS3_KEY_HEK_REG(reg_start_ind),
			      c3->key.hek.words[reg_start_ind]);

	reg_val |= (index << MVPP2_CLS3_HASH_OP_TBL_ADDR);
	reg_val &= ~MVPP2_CLS3_MISS_PTR_MASK; /*set miss bit to 0*/
	reg_val |= (1 << MVPP2_CLS3_HASH_OP_ADD);

	/* set hit counter init value */
	pp2_reg_write(cpu_slot, MVPP2_CLS3_INIT_HIT_CNT_REG, sw_init_cnt_set << MVPP2_CLS3_INIT_HIT_CNT_OFFS),
	/*trigger ADD operation*/
	pp2_reg_write(cpu_slot, MVPP2_CLS3_HASH_OP_REG, reg_val);

	/* wait to cpu access done bit */
	while (!pp2_cls_c3_cpu_done(cpu_slot))
		if (++iter >= RETRIES_EXCEEDED) {
			pr_err("%s:Error - retries exceeded.\n", __func__);
			return -EBUSY;
		}

	/* write action table registers */
	pp2_reg_write(cpu_slot, MVPP2_CLS3_ACT_REG, c3->sram.regs.actions);
	pp2_reg_write(cpu_slot, MVPP2_CLS3_ACT_QOS_ATTR_REG, c3->sram.regs.qos_attr);
	pp2_reg_write(cpu_slot, MVPP2_CLS3_ACT_HWF_ATTR_REG, c3->sram.regs.hwf_attr);
	pp2_reg_write(cpu_slot, MVPP2_CLS3_ACT_DUP_ATTR_REG, c3->sram.regs.dup_attr);
	pp2_reg_write(cpu_slot, MVPP2_CLS3_ACT_SEQ_L_ATTR_REG, c3->sram.regs.seq_l_attr);
	pp2_reg_write(cpu_slot, MVPP2_CLS3_ACT_SEQ_H_ATTR_REG, c3->sram.regs.seq_h_attr);
	/* set entry as valid, extesion pointer in use only if size > 12*/
	pp2_cls_c3_shadow_set(hek_size, index, ext_index);

	return 0;
}

/*-------------------------------------------------------------------------------*/
/*	Add entry to miss hash table							  */
/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_hw_miss_add(uintptr_t cpu_slot, struct pp2_cls_c3_entry *c3, int lkp_type)
{
	u32 reg_val = 0;

	if (mv_pp2x_ptr_validate(c3))
		return -EINVAL;

	if (mv_pp2x_range_validate(lkp_type, 0, MVPP2_CLS_C3_MISS_TBL_SIZE - 1))
		return -EINVAL;

	c3->index = lkp_type;

	reg_val |= (lkp_type << MVPP2_CLS3_HASH_OP_TBL_ADDR);
	reg_val |= (1 << MVPP2_CLS3_HASH_OP_ADD);
	reg_val |= MVPP2_CLS3_MISS_PTR_MASK;/*set miss bit to 1*/

	/*index to miss table */
	pp2_reg_write(cpu_slot, MVPP2_CLS3_HASH_OP_REG, reg_val);

	/* write action table registers */
	pp2_reg_write(cpu_slot, MVPP2_CLS3_ACT_REG, c3->sram.regs.actions);
	pp2_reg_write(cpu_slot, MVPP2_CLS3_ACT_QOS_ATTR_REG, c3->sram.regs.qos_attr);
	pp2_reg_write(cpu_slot, MVPP2_CLS3_ACT_HWF_ATTR_REG, c3->sram.regs.hwf_attr);
	pp2_reg_write(cpu_slot, MVPP2_CLS3_ACT_DUP_ATTR_REG, c3->sram.regs.dup_attr);
	pp2_reg_write(cpu_slot, MVPP2_CLS3_ACT_SEQ_L_ATTR_REG, c3->sram.regs.seq_l_attr);
	pp2_reg_write(cpu_slot, MVPP2_CLS3_ACT_SEQ_H_ATTR_REG, c3->sram.regs.seq_h_attr);
	/*clear hit counter, clear on read */
	pp2_cls_c3_hit_cntrs_miss_read(cpu_slot, lkp_type, &reg_val);

	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_hw_del(uintptr_t cpu_slot, int index)
{
	u32 reg_val = 0;
	int iter = 0;

	if (mv_pp2x_range_validate(index, 0, MVPP2_CLS3_HASH_OP_TBL_ADDR_MAX))
		return -EINVAL;

	reg_val |= (index << MVPP2_CLS3_HASH_OP_TBL_ADDR);
	reg_val |= (1 << MVPP2_CLS3_HASH_OP_DEL);
	reg_val &= ~MVPP2_CLS3_MISS_PTR_MASK;/*set miss bit to 1*/

	/*trigger del operation*/
	pp2_reg_write(cpu_slot, MVPP2_CLS3_HASH_OP_REG, reg_val);

	/* wait to cpu access done bit */
	while (!pp2_cls_c3_cpu_done(cpu_slot))
		if (++iter >= RETRIES_EXCEEDED) {
			pr_err("%s:Error - retries exceeded.\n", __func__);
			return -EBUSY;
		}

	/* delete form shadow and extension shadow if exist */
	pp2_cls_c3_shadow_clear(index);

	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_hw_del_all(uintptr_t cpu_slot)
{
	int index, status;

	for (index = 0; index < MVPP2_CLS_C3_HASH_TBL_SIZE; index++) {
		status = pp2_cls_c3_hw_del(cpu_slot, index);
		if (status != 0)
			return status;
	}
	return 0;
}

/*-------------------------------------------------------------------------------*/
void pp2_cls_c3_hw_init_ctr_set(int cnt_val)
{
	sw_init_cnt_set = cnt_val;
}

/*-------------------------------------------------------------------------------*/
static int pp2_cls_c3_hw_query_add_relocate(uintptr_t cpu_slot, int new_idx, int max_depth, int cur_depth,
					    struct pp2_cls_c3_hash_pair *hash_pair_arr)
{
	int ret_val = 0, index_free, idx = 0;
	u8 occupied_bmp;
	struct pp2_cls_c3_entry local_c3;
	int used_index[MVPP2_CLS3_HASH_BANKS_NUM] = {0};

	if (cur_depth >= max_depth)
		return -EINVAL;

	pp2_cls_c3_sw_clear(&local_c3);

	ret_val = pp2_cls_c3_hw_read(cpu_slot, &local_c3, new_idx);
	if (ret_val) {
		pr_err("%s could not get key for index [0x%x]\n", __func__, new_idx);
		return ret_val;
	}

	ret_val = pp2_cls_c3_hw_query(cpu_slot, &local_c3, &occupied_bmp, used_index);
	if (ret_val) {
		pr_err("%s: pp2_cls_c3_hw_query failed, depth = %d\n", __func__, cur_depth);
		return ret_val;
	}

	/* fill in indices for this key */
	for (idx = 0; idx < MVPP2_CLS3_HASH_BANKS_NUM; idx++) {
		/* if new index is in the bank index, skip it */
		if (new_idx == used_index[idx]) {
			used_index[idx] = 0;
			continue;
		}

		/* found a vacant index */
		if (!(occupied_bmp & (1 << idx))) {
			index_free = used_index[idx];
			break;
		}
	}

	/* no free index, recurse and relocate another key */
	if (idx == MVPP2_CLS3_HASH_BANKS_NUM) {
#ifdef MV_DEBUG
		pr_debug("new[0x%.3x]:%.1d ", new_idx, cur_depth);
		for (idx = 0; idx < MVPP2_CLS3_HASH_BANKS_NUM; idx++)
			pr_debug("0x%.3x ", used_index[idx]);
		pr_debug("\n");
#endif

		/* recurse over all valid indices */
		for (idx = 0; idx < MVPP2_CLS3_HASH_BANKS_NUM; idx++) {
			if (used_index[idx] == 0)
				continue;

			if (pp2_cls_c3_hw_query_add_relocate(cpu_slot, used_index[idx], max_depth, cur_depth + 1,
							     hash_pair_arr) == 0)
				break;
		}

		/* tried relocate, no valid entries found */
		if (idx == MVPP2_CLS3_HASH_BANKS_NUM)
			return -EIO;
	}

	/* if we reached here, we found a valid free index */
	index_free = used_index[idx];

	/* new_idx del is not necessary */

	/*We do not chage extension tabe*/
	ret_val = pp2_cls_c3_hw_add(cpu_slot, &local_c3, index_free, local_c3.ext_index);

	/* update the hash pair */
	if (!hash_pair_arr) {
		hash_pair_arr->old_idx[hash_pair_arr->pair_num] = new_idx;
		hash_pair_arr->new_idx[hash_pair_arr->pair_num] = index_free;
		hash_pair_arr->pair_num++;
	}

	if (ret_val != 0) {
		pr_err("%s:Error - pp2_cls_c3_hw_add failed, depth = %d\\n", __func__, cur_depth);
		return ret_val;
	}

	pr_info("key relocated  0x%.3x->0x%.3x\n", new_idx, index_free);

	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_hw_query_add(uintptr_t cpu_slot, struct pp2_cls_c3_entry *c3, int max_search_depth,
			    struct pp2_cls_c3_hash_pair *hash_pair_arr)
{
	int used_index[MVPP2_CLS3_HASH_BANKS_NUM] = {0};
	u8 occupied_bmp;
	int idx, index_free, hek_size, ret_val, ext_index = 0;

	ret_val = pp2_cls_c3_hw_query(cpu_slot, c3, &occupied_bmp, used_index);
	if (ret_val != 0) {
		pr_err("%s:Error - pp2_cls_c3_hw_query failed\n", __func__);
		return ret_val;
	}

	/* Select available entry index */
	for (idx = 0; idx < MVPP2_CLS3_HASH_BANKS_NUM; idx++) {
		if (!(occupied_bmp & (1 << idx)))
			break;
	}

	/* Available index did not found, try to relocate another key */
	if (idx == MVPP2_CLS3_HASH_BANKS_NUM) {
		for (idx = 0; idx < MVPP2_CLS3_HASH_BANKS_NUM; idx++) {
			if (pp2_cls_c3_hw_query_add_relocate(cpu_slot, used_index[idx], max_search_depth,
							     0 /*curren depth*/, hash_pair_arr) == 0)
				break;
		}

		if (idx == MVPP2_CLS3_HASH_BANKS_NUM) {
			/* Available index did not found*/
			pr_err("%s:Error - HASH table is full.\n", __func__);
			return -EIO;
		}
	}

	index_free = used_index[idx];

	hek_size = ((c3->key.key_ctrl & KEY_CTRL_HEK_SIZE_MASK) >> KEY_CTRL_HEK_SIZE);

	if (hek_size > MVPP2_CLS_C3_HEK_BYTES) {
		/* Get Free Extension Index */
		ext_index = pp2_cls_c3_shadow_ext_free_get();

		if (ext_index == MVPP2_CLS_C3_EXT_TBL_SIZE) {
			pr_err("%s:Error - Extension table is full.\n", __func__);
			return -EIO;
		}
	}

	ret_val = pp2_cls_c3_hw_add(cpu_slot, c3, index_free, ext_index);
	if (ret_val != 0) {
		pr_err("%s:Error - pp2_cls_c3_hw_add failed\n", __func__);
		return ret_val;
	}

	if (hek_size > MVPP2_CLS_C3_HEK_BYTES)
		pr_info("Added C3 entry @ index=0x%.3x ext=0x%.3x\n", index_free, ext_index);
	else
		pr_info("Added C3 entry @ index=0x%.3x\n", index_free);

	return 0;
}

/*-------------------------------------------------------------------------------*/
/*	if index or occupied_bmp is NULL dump the data					  */
/*	index[] size must be 8							  */
/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_hw_query(uintptr_t cpu_slot, struct pp2_cls_c3_entry *c3, u8 *occupied_bmp, int index[])
{
	int idx = 0;
	u32 reg_val = 0;

	if (mv_pp2x_ptr_validate(c3))
		return -EINVAL;

	/* write key control */
	pp2_reg_write(cpu_slot, MVPP2_CLS3_KEY_CTRL_REG, c3->key.key_ctrl);

	/* write hek */
	for (idx = 0; idx < MVPP2_CLS_C3_EXT_HEK_WORDS; idx++)
		pp2_reg_write(cpu_slot, MVPP2_CLS3_KEY_HEK_REG(idx), c3->key.hek.words[idx]);

	/*trigger query operation*/
	pp2_reg_write(cpu_slot, MVPP2_CLS3_QRY_ACT_REG, (1 << MVPP2_CLS3_QRY_ACT));

	idx = 0;
	while (!pp2_cls_c3_cpu_done(cpu_slot))
		if (++idx >= RETRIES_EXCEEDED) {
			pr_err("%s:Error - retries exceeded.\n", __func__);
			return -EBUSY;
		}

	reg_val = pp2_reg_read(cpu_slot, MVPP2_CLS3_STATE_REG) & MVPP2_CLS3_STATE_OCCIPIED_MASK;
	reg_val = reg_val >> MVPP2_CLS3_STATE_OCCIPIED;

	if ((!occupied_bmp) || (!index)) {
		/* print to screen - call from sysfs*/
		for (idx = 0; idx < MVPP2_CLS3_HASH_BANKS_NUM; idx++)
			pr_info("0x%8.8x	%s\n", pp2_reg_read(cpu_slot, MVPP2_CLS3_QRY_RES_HASH_REG(idx)),
				 (reg_val & (1 << idx)) ? "OCCUPIED" : "FREE");
		return 0;
	}

	*occupied_bmp = reg_val;
	for (idx = 0; idx < MVPP2_CLS3_HASH_BANKS_NUM; idx++)
		index[idx] = pp2_reg_read(cpu_slot, MVPP2_CLS3_QRY_RES_HASH_REG(idx));

	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_hw_read(uintptr_t cpu_slot, struct pp2_cls_c3_entry *c3, int index)
{
	int i, is_ext;
	int reg_val = 0;
	u32 hash_data[MVPP2_CLS3_HASH_DATA_REG_NUM];
	u32 hash_ext_data[MVPP2_CLS3_HASH_EXT_DATA_REG_NUM];

	if (mv_pp2x_ptr_validate(c3))
		return -EINVAL;

	if (mv_pp2x_range_validate(index, 0, MVPP2_CLS3_HASH_OP_TBL_ADDR_MAX))
		return -EINVAL;

	pp2_cls_c3_sw_clear(c3);

	c3->index = index;
	c3->ext_index = NOT_IN_USE;

	/* write index */
	pp2_reg_write(cpu_slot, MVPP2_CLS3_DB_INDEX_REG, index);

	reg_val |= (index << MVPP2_CLS3_HASH_OP_TBL_ADDR);
	pp2_reg_write(cpu_slot, MVPP2_CLS3_HASH_OP_REG, reg_val);

	/* read action table */
	c3->sram.regs.actions = pp2_reg_read(cpu_slot, MVPP2_CLS3_ACT_REG);
	c3->sram.regs.qos_attr = pp2_reg_read(cpu_slot, MVPP2_CLS3_ACT_QOS_ATTR_REG);
	c3->sram.regs.hwf_attr = pp2_reg_read(cpu_slot, MVPP2_CLS3_ACT_HWF_ATTR_REG);
	c3->sram.regs.dup_attr = pp2_reg_read(cpu_slot, MVPP2_CLS3_ACT_DUP_ATTR_REG);

	c3->sram.regs.seq_l_attr = pp2_reg_read(cpu_slot, MVPP2_CLS3_ACT_SEQ_L_ATTR_REG);
	c3->sram.regs.seq_h_attr = pp2_reg_read(cpu_slot, MVPP2_CLS3_ACT_SEQ_H_ATTR_REG);

	/* read hash data*/
	for (i = 0; i < MVPP2_CLS3_HASH_DATA_REG_NUM; i++)
		hash_data[i] = pp2_reg_read(cpu_slot, MVPP2_CLS3_HASH_DATA_REG(i));

	if (pp2_cls_c3_shadow_tbl[index].size == 0)
		/* entry not in use */
		return 0;

	c3->key.key_ctrl = 0;

	if (pp2_cls_c3_shadow_tbl[index].ext_ptr == NOT_IN_USE) {
		is_ext = 0;
		/* TODO REMOVE NEXT LINES- ONLY FOR INTERNAL VALIDATION */
		if ((pp2_cls_c3_shadow_tbl[index].size == 0) || (pp2_cls_c3_shadow_tbl[index].ext_ptr != NOT_IN_USE)) {
			pr_err("%s: SW internal error.\n", __func__);
			return -EIO;
		}

		/*read Multihash entry data*/
		c3->key.hek.words[6] = hash_data[0]; /* hek 0*/
		c3->key.hek.words[7] = hash_data[1]; /* hek 1*/
		c3->key.hek.words[8] = hash_data[2]; /* hek 2*/

		/* write key control data to SW */
		c3->key.key_ctrl |= (((hash_data[3] & KEY_PRT_ID_MASK(is_ext)) >>
					(KEY_PRT_ID(is_ext) % DWORD_BITS_LEN)) << KEY_CTRL_PRT_ID);

		c3->key.key_ctrl |= (((hash_data[3] & KEY_PRT_ID_TYPE_MASK(is_ext)) >>
					(KEY_PRT_ID_TYPE(is_ext) % DWORD_BITS_LEN)) << KEY_CTRL_PRT_ID_TYPE);

		c3->key.key_ctrl |= (((hash_data[3] & KEY_LKP_TYPE_MASK(is_ext)) >>
					(KEY_LKP_TYPE(is_ext) % DWORD_BITS_LEN)) << KEY_CTRL_LKP_TYPE);

		c3->key.key_ctrl |= (((hash_data[3] & KEY_L4_INFO_MASK(is_ext)) >>
					(KEY_L4_INFO(is_ext) % DWORD_BITS_LEN)) << KEY_CTRL_L4);

	} else {
		is_ext = 1;
		/* TODO REMOVE NEXT LINES- ONLY FOR INTERNAL VALIDATION */
		if ((pp2_cls_c3_shadow_tbl[index].size == 0) || (pp2_cls_c3_shadow_tbl[index].ext_ptr == NOT_IN_USE)) {
			pr_err("%s: SW internal error.\n", __func__);
			return -EIO;
		}
		c3->ext_index = pp2_cls_c3_shadow_tbl[index].ext_ptr;

		/* write extension index */
		pp2_reg_write(cpu_slot, MVPP2_CLS3_DB_INDEX_REG, pp2_cls_c3_shadow_tbl[index].ext_ptr);

		/* read hash extesion data*/
		for (i = 0; i < MVPP2_CLS3_HASH_EXT_DATA_REG_NUM; i++)
			hash_ext_data[i] = pp2_reg_read(cpu_slot, MVPP2_CLS3_HASH_EXT_DATA_REG(i));

		/* heks bytes 35 - 32 */
		c3->key.hek.words[8] = ((hash_data[2] & 0x00FFFFFF) << 8) | ((hash_data[1] & 0xFF000000) >> 24);

		/* heks bytes 31 - 28 */
		c3->key.hek.words[7] = ((hash_data[1] & 0x00FFFFFF) << 8) | ((hash_data[0] & 0xFF000000) >> 24);

		/* heks bytes 27 - 24 */
		c3->key.hek.words[6] = ((hash_data[0] & 0x00FFFFFF) << 8) | (hash_ext_data[6] & 0x000000FF);

		c3->key.hek.words[5] = hash_ext_data[5]; /* heks bytes 23 - 20 */
		c3->key.hek.words[4] = hash_ext_data[4]; /* heks bytes 19 - 16 */
		c3->key.hek.words[3] = hash_ext_data[3]; /* heks bytes 15 - 12 */
		c3->key.hek.words[2] = hash_ext_data[2]; /* heks bytes 11 - 8  */
		c3->key.hek.words[1] = hash_ext_data[1]; /* heks bytes 7 - 4   */
		c3->key.hek.words[0] = hash_ext_data[0]; /* heks bytes 3 - 0   */

		/* write key control data to SW*/

		c3->key.key_ctrl |= (((hash_data[3] & KEY_PRT_ID_MASK(is_ext)) >>
					(KEY_PRT_ID(is_ext) % DWORD_BITS_LEN)) << KEY_CTRL_PRT_ID);

		/* PPv2.1 (feature MAS 3.16) LKP_TYPE size and offset changed */

		c3->key.key_ctrl |= (((hash_data[3] & KEY_PRT_ID_TYPE_MASK(is_ext)) >>
					(KEY_PRT_ID_TYPE(is_ext) % DWORD_BITS_LEN)) << KEY_CTRL_PRT_ID_TYPE);

		c3->key.key_ctrl |= ((((hash_data[2] & 0xf8000000) >> 27) |
					((hash_data[3] & 0x1) << 5)) << KEY_CTRL_LKP_TYPE);

		c3->key.key_ctrl |= (((hash_data[2] & KEY_L4_INFO_MASK(is_ext)) >>
					(KEY_L4_INFO(is_ext) % DWORD_BITS_LEN)) << KEY_CTRL_L4);
	}

	/* update hek size */
	c3->key.key_ctrl |= ((pp2_cls_c3_shadow_tbl[index].size << KEY_CTRL_HEK_SIZE) & KEY_CTRL_HEK_SIZE_MASK);

	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_hw_miss_read(uintptr_t cpu_slot, struct pp2_cls_c3_entry *c3, int lkp_type)
{
	u32 reg_val = 0;

	if (mv_pp2x_ptr_validate(c3))
		return -EINVAL;

	if (mv_pp2x_range_validate(lkp_type, 0, MVPP2_CLS_C3_MISS_TBL_SIZE - 1))
		return -EINVAL;

	pp2_cls_c3_sw_clear(c3);

	c3->index = lkp_type;
	c3->ext_index = NOT_IN_USE;

	reg_val = (lkp_type << MVPP2_CLS3_HASH_OP_TBL_ADDR) | MVPP2_CLS3_MISS_PTR_MASK;
	pp2_reg_write(cpu_slot, MVPP2_CLS3_HASH_OP_REG, reg_val);

	/* read action table */
	c3->sram.regs.actions = pp2_reg_read(cpu_slot, MVPP2_CLS3_ACT_REG);
	c3->sram.regs.qos_attr = pp2_reg_read(cpu_slot, MVPP2_CLS3_ACT_QOS_ATTR_REG);
	c3->sram.regs.hwf_attr = pp2_reg_read(cpu_slot, MVPP2_CLS3_ACT_HWF_ATTR_REG);
	c3->sram.regs.dup_attr = pp2_reg_read(cpu_slot, MVPP2_CLS3_ACT_DUP_ATTR_REG);
	c3->sram.regs.seq_l_attr = pp2_reg_read(cpu_slot, MVPP2_CLS3_ACT_SEQ_L_ATTR_REG);
	c3->sram.regs.seq_h_attr = pp2_reg_read(cpu_slot, MVPP2_CLS3_ACT_SEQ_H_ATTR_REG);

	return 0;
}

/*-------------------------------------------------------------------------------*/
/*	APIs for Classification C3 key fields						  */
/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_sw_l4_info_set(struct pp2_cls_c3_entry *c3, int l4info)
{
	if (mv_pp2x_ptr_validate(c3))
		return -EINVAL;

	if (mv_pp2x_range_validate(l4info, 0, KEY_CTRL_L4_MAX))
		return -EINVAL;

	c3->key.key_ctrl &= ~KEY_CTRL_L4_MASK;
	c3->key.key_ctrl |= (l4info << KEY_CTRL_L4);
	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_sw_lkp_type_set(struct pp2_cls_c3_entry *c3, int lkp_type)
{
	if (mv_pp2x_ptr_validate(c3))
		return -EINVAL;

	if (mv_pp2x_range_validate(lkp_type, 0, KEY_CTRL_LKP_TYPE_MAX))
		return -EINVAL;

	c3->key.key_ctrl &= ~KEY_CTRL_LKP_TYPE_MASK;
	c3->key.key_ctrl |= (lkp_type << KEY_CTRL_LKP_TYPE);
	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_sw_port_id_set(struct pp2_cls_c3_entry *c3, int type, int portid)
{
	if (mv_pp2x_ptr_validate(c3))
		return -EINVAL;

	if (mv_pp2x_range_validate(portid, 0, KEY_CTRL_PRT_ID_MAX))
		return -EINVAL;

	if (mv_pp2x_range_validate(type, 0, KEY_CTRL_PRT_ID_TYPE_MAX))
		return -EINVAL;

	c3->key.key_ctrl &= ~(KEY_CTRL_PRT_ID_MASK | KEY_CTRL_PRT_ID_TYPE_MASK);
	c3->key.key_ctrl |= ((portid << KEY_CTRL_PRT_ID) | (type << KEY_CTRL_PRT_ID_TYPE));

	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_sw_hek_size_set(struct pp2_cls_c3_entry *c3, int hek_size)
{
	if (mv_pp2x_ptr_validate(c3))
		return -EINVAL;

	if (mv_pp2x_range_validate(hek_size, 0, KEY_CTRL_HEK_SIZE_MAX))
		return -EINVAL;

	c3->key.key_ctrl &= ~KEY_CTRL_HEK_SIZE_MASK;
	c3->key.key_ctrl |= (hek_size << KEY_CTRL_HEK_SIZE);
	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_sw_hek_byte_set(struct pp2_cls_c3_entry *c3, u32 offs, u8 byte)
{
	if (mv_pp2x_ptr_validate(c3))
		return -EINVAL;

	if (mv_pp2x_range_validate(offs, 0, ((MVPP2_CLS_C3_EXT_HEK_WORDS * 4) - 1)))
		return -EINVAL;

	c3->key.hek.bytes[HW_BYTE_OFFS(offs)] = byte;

	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_sw_hek_word_set(struct pp2_cls_c3_entry *c3, u32 offs, u32 word)
{
	if (mv_pp2x_ptr_validate(c3))
		return -EINVAL;

	if (mv_pp2x_range_validate(offs, 0, ((MVPP2_CLS_C3_EXT_HEK_WORDS) - 1)))
		return -EINVAL;

	c3->key.hek.words[offs] = word;

	return 0;
}

/*-------------------------------------------------------------------------------*/
/*	APIs for Classification C3 action table fields					  */
/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_color_set(struct pp2_cls_c3_entry *c3, int cmd)
{
	if (mv_pp2x_ptr_validate(c3))
		return -EINVAL;

	if (mv_pp2x_range_validate(cmd, 0, MVPP2_COLOR_ACTION_TYPE_RED_LOCK))
		return -EINVAL;

	c3->sram.regs.actions &= ~MVPP2_CLS3_ACT_COLOR_MASK;
	c3->sram.regs.actions |= (cmd << MVPP2_CLS3_ACT_COLOR);

	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_queue_high_set(struct pp2_cls_c3_entry *c3, int cmd, int queue)
{
	if (mv_pp2x_ptr_validate(c3))
		return -EINVAL;

	if (mv_pp2x_range_validate(cmd, 0, MVPP2_ACTION_TYPE_UPDT_LOCK))
		return -EINVAL;

	if (mv_pp2x_range_validate(queue, 0, MVPP2_CLS3_ACT_QOS_ATTR_HIGH_Q_MAX))
		return -EINVAL;

	/*set command*/
	c3->sram.regs.actions &= ~MVPP2_CLS3_ACT_HIGH_Q_MASK;
	c3->sram.regs.actions |= (cmd << MVPP2_CLS3_ACT_HIGH_Q);

	/*set modify High queue value*/
	c3->sram.regs.qos_attr &= ~MVPP2_CLS3_ACT_QOS_ATTR_HIGH_Q_MASK;
	c3->sram.regs.qos_attr |= (queue << MVPP2_CLS3_ACT_QOS_ATTR_HIGH_Q);

	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_queue_low_set(struct pp2_cls_c3_entry *c3, int cmd, int queue)
{
	if (mv_pp2x_ptr_validate(c3))
		return -EINVAL;

	if (mv_pp2x_range_validate(cmd, 0, MVPP2_ACTION_TYPE_UPDT_LOCK))
		return -EINVAL;

	if (mv_pp2x_range_validate(queue, 0, MVPP2_CLS3_ACT_QOS_ATTR_LOW_Q_MAX))
		return -EINVAL;

	/*set command*/
	c3->sram.regs.actions &= ~MVPP2_CLS3_ACT_LOW_Q_MASK;
	c3->sram.regs.actions |= (cmd << MVPP2_CLS3_ACT_LOW_Q);

	/*set modify High queue value*/
	c3->sram.regs.qos_attr &= ~MVPP2_CLS3_ACT_QOS_ATTR_LOW_Q_MASK;
	c3->sram.regs.qos_attr |= (queue << MVPP2_CLS3_ACT_QOS_ATTR_LOW_Q);

	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_queue_set(struct pp2_cls_c3_entry *c3, int cmd, int queue)
{
	int status = 0;
	int q_high, q_low;

	if (mv_pp2x_ptr_validate(c3))
		return -EINVAL;

	if (mv_pp2x_range_validate(queue, 0, MVPP2_CLS3_ACT_QOS_ATTR_Q_MAX))
		return -EINVAL;

	/* cmd validation in set functions */

	q_high = (queue & MVPP2_CLS3_ACT_QOS_ATTR_HIGH_Q_MASK) >> MVPP2_CLS3_ACT_QOS_ATTR_HIGH_Q;
	q_low = (queue & MVPP2_CLS3_ACT_QOS_ATTR_LOW_Q_MASK) >> MVPP2_CLS3_ACT_QOS_ATTR_LOW_Q;

	status |= pp2_cls_c3_queue_low_set(c3, cmd, q_high);
	status |= pp2_cls_c3_queue_high_set(c3, cmd, q_low);

	return status;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_forward_set(struct pp2_cls_c3_entry *c3, int cmd)
{
	if (mv_pp2x_ptr_validate(c3))
		return -EINVAL;

	if (mv_pp2x_range_validate(cmd, 0, MVPP2_FRWD_ACTION_TYPE_HWF_LOW_LATENCY_LOCK))
		return -EINVAL;

	c3->sram.regs.actions &= ~MVPP2_CLS3_ACT_FWD_MASK;
	c3->sram.regs.actions |= (cmd << MVPP2_CLS3_ACT_FWD);
	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_policer_set(struct pp2_cls_c3_entry *c3, int cmd, int policer_id, int bank)
{
	if (mv_pp2x_ptr_validate(c3))
		return -EINVAL;

	if (mv_pp2x_range_validate(cmd, 0, MVPP2_ACTION_TYPE_UPDT_LOCK))
		return -EINVAL;

	if (mv_pp2x_range_validate(policer_id, 0, MVPP2_CLS3_ACT_DUP_POLICER_MAX))
		return -EINVAL;

	if (mv_pp2x_range_validate(bank, 0, 1))
		return -EINVAL;

	c3->sram.regs.actions &= ~MVPP2_CLS3_ACT_POLICER_SELECT_MASK;
	c3->sram.regs.actions |= (cmd << MVPP2_CLS3_ACT_POLICER_SELECT);

	c3->sram.regs.dup_attr &= ~MVPP2_CLS3_ACT_DUP_POLICER_MASK;
	c3->sram.regs.dup_attr |= (policer_id << MVPP2_CLS3_ACT_DUP_POLICER_ID);

	if (bank)
		c3->sram.regs.dup_attr |= MVPP2_CLS3_ACT_DUP_POLICER_BANK_MASK;
	else
		c3->sram.regs.dup_attr &= ~MVPP2_CLS3_ACT_DUP_POLICER_BANK_MASK;

	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_flow_id_en(struct pp2_cls_c3_entry *c3, int flowid_en)
{
	 if (mv_pp2x_ptr_validate(c3))
		return -EINVAL;

	/*set Flow ID enable or disable*/
	if (flowid_en)
		c3->sram.regs.actions |= (1 << MVPP2_CLS3_ACT_FLOW_ID_EN);
	else
		c3->sram.regs.actions &= ~(1 << MVPP2_CLS3_ACT_FLOW_ID_EN);

	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_rss_set(struct pp2_cls_c3_entry *c3, int cmd, int rss_en)
{
	 if (mv_pp2x_ptr_validate(c3))
		return -EINVAL;

	 if (mv_pp2x_range_validate(cmd, 0, MVPP2_ACTION_TYPE_UPDT_LOCK))
		return -EINVAL;

	 if (mv_pp2x_range_validate(rss_en, 0, 1))
		return -EINVAL;

	 c3->sram.regs.actions &= ~MVPP2_CLS3_ACT_RSS_EN_MASK;
	 c3->sram.regs.actions |= (cmd << MVPP2_CLS3_ACT_RSS_EN);

	 c3->sram.regs.dup_attr &= ~MVPP2_CLS3_ACT_DUP_RSS_EN_MASK;
	 c3->sram.regs.dup_attr |= (rss_en << MVPP2_CLS3_ACT_DUP_RSS_EN_BIT);

	 return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_mod_set(struct pp2_cls_c3_entry *c3, int data_ptr, int instr_offs, int l4_csum)
{
	if (mv_pp2x_ptr_validate(c3))
		return -EINVAL;

	if (mv_pp2x_range_validate(data_ptr, 0, MVPP2_CLS3_ACT_HWF_ATTR_DPTR_MAX))
		return -EINVAL;

	if (mv_pp2x_range_validate(instr_offs, 0, MVPP2_CLS3_ACT_HWF_ATTR_IPTR_MAX))
		return -EINVAL;

	if (mv_pp2x_range_validate(l4_csum, 0, 1))
		return -EINVAL;

	c3->sram.regs.hwf_attr &= ~MVPP2_CLS3_ACT_HWF_ATTR_DPTR_MASK;
	c3->sram.regs.hwf_attr &= ~MVPP2_CLS3_ACT_HWF_ATTR_IPTR_MASK;
	c3->sram.regs.hwf_attr &= ~MVPP2_CLS3_ACT_HWF_ATTR_CHKSM_EN_MASK;

	c3->sram.regs.hwf_attr |= (data_ptr << MVPP2_CLS3_ACT_HWF_ATTR_DPTR);
	c3->sram.regs.hwf_attr |= (instr_offs << MVPP2_CLS3_ACT_HWF_ATTR_IPTR);
	c3->sram.regs.hwf_attr |= (l4_csum << MVPP2_CLS3_ACT_HWF_ATTR_CHKSM_EN);

	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_mtu_set(struct pp2_cls_c3_entry *c3, int mtu_inx)
{
	if (mv_pp2x_ptr_validate(c3))
		return -EINVAL;

	if (mv_pp2x_range_validate(mtu_inx, 0, MVPP2_CLS3_ACT_HWF_ATTR_MTU_INX_MAX))
		return -EINVAL;

	c3->sram.regs.hwf_attr &= ~MVPP2_CLS3_ACT_HWF_ATTR_MTU_INX_MASK;
	c3->sram.regs.hwf_attr |= (mtu_inx << MVPP2_CLS3_ACT_HWF_ATTR_MTU_INX);
	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_dup_set(struct pp2_cls_c3_entry *c3, int dupid, int count)
{
	if (mv_pp2x_ptr_validate(c3))
		return -EINVAL;

	if (mv_pp2x_range_validate(count, 0, MVPP2_CLS3_ACT_DUP_COUNT_MAX))
		return -EINVAL;

	if (mv_pp2x_range_validate(dupid, 0, MVPP2_CLS3_ACT_DUP_FID_MAX))
		return -EINVAL;

	/*set flowid and count*/
	c3->sram.regs.dup_attr &= ~(MVPP2_CLS3_ACT_DUP_FID_MASK | MVPP2_CLS3_ACT_DUP_COUNT_MASK);
	c3->sram.regs.dup_attr |= (dupid << MVPP2_CLS3_ACT_DUP_FID);
	c3->sram.regs.dup_attr |= (count << MVPP2_CLS3_ACT_DUP_COUNT);

	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_seq_set(struct pp2_cls_c3_entry *c3, int id,  int bits_offs,  int bits)
{
	u32 low_bits, high_bits = 0;

	if (mv_pp2x_ptr_validate(c3))
		return -EINVAL;

	if (mv_pp2x_range_validate(bits, 0, MVPP2_CLS_SEQ_SIZE_MAX))
		return -EINVAL;

	if (mv_pp2x_range_validate(id, 0, (1 << bits) - 1))
		return -EINVAL;

	if (mv_pp2x_range_validate(bits_offs + bits, 0, MVPP2_CLS3_ACT_SEQ_SIZE))
		return -EINVAL;

	if (bits_offs >= DWORD_BITS_LEN)
		high_bits = bits;

	else if (bits_offs + bits > DWORD_BITS_LEN)
		high_bits = (bits_offs + bits) % DWORD_BITS_LEN;

	low_bits = bits - high_bits;

	/*
	* high_bits hold the num of bits that we need to write in seq_h_attr
	* low_bits hold the num of bits that we need to write in seq_l_attr
	*/

	if (low_bits) {
		/* mask and set new value in seq_l_attr*/
		c3->sram.regs.seq_l_attr &= ~(((1 << low_bits) - 1)  << bits_offs);
		c3->sram.regs.seq_l_attr |= (id  << bits_offs);
	}

	if (high_bits) {
		int high_id = id >> low_bits;
		int high_offs = (low_bits == 0) ? (bits_offs % DWORD_BITS_LEN) : 0;

		/* mask and set new value in seq_h_attr*/
		c3->sram.regs.seq_h_attr &= ~(((1 << high_bits) - 1)  << high_offs);
		c3->sram.regs.seq_h_attr |= (high_id << high_offs);
	}

	return 0;
}

/*-------------------------------------------------------------------------------*/
/*	APIs for Classification C3 Hit counters management				  */
/*-------------------------------------------------------------------------------*/

int pp2_cls_c3_hit_cntrs_clear(uintptr_t cpu_slot, int lkp_type)
{
	/* clear all counters that entry lookup type corresponding to lkp_type */
	int iter = 0;

	if (mv_pp2x_range_validate(lkp_type, 0, KEY_CTRL_LKP_TYPE_MAX))
		return -EINVAL;

	pp2_reg_write(cpu_slot, MVPP2_CLS3_CLEAR_COUNTERS_REG, lkp_type);

	/* wait to clear het counters done bit */
	while (!pp2_cls_c3_hit_cntr_clear_done(cpu_slot))
		if (++iter >= RETRIES_EXCEEDED) {
			pr_err("%s:Error - retries exceeded.\n", __func__);
			return -EBUSY;
		}

	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_hit_cntrs_clear_all(uintptr_t cpu_slot)
{
	int iter = 0;

	pp2_reg_write(cpu_slot, MVPP2_CLS3_CLEAR_COUNTERS_REG, MVPP2_CLS3_CLEAR_ALL);
	/* wait to clear het counters done bit */
	while (!pp2_cls_c3_hit_cntr_clear_done(cpu_slot))
		if (++iter >= RETRIES_EXCEEDED) {
			pr_err("%s:Error - retries exceeded.\n", __func__);
			return -EBUSY;
		}

	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_hit_cntrs_read(uintptr_t cpu_slot, int index, u32 *cntr)
{
	u32 counter;

	if (mv_pp2x_range_validate(index, 0, MVPP2_CLS3_HASH_OP_TBL_ADDR_MAX))
		return -EINVAL;

	/*write entry index*/
	pp2_reg_write(cpu_slot, MVPP2_CLS3_DB_INDEX_REG, index);

	/*counter read*/
	counter = pp2_reg_read(cpu_slot, MVPP2_CLS3_HIT_COUNTER_REG) & MVPP2_CLS3_HIT_COUNTER_MASK;

	if (!cntr)
		pr_info("ADDR:0x%3.3x	COUNTER VAL:0x%6.6x\n", index, counter);
	else
		*cntr = counter;
	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_hit_cntrs_miss_read(uintptr_t cpu_slot, int lkp_type, u32 *cntr)
{
	u32 counter;
	int index;

	if (mv_pp2x_range_validate(lkp_type, 0, MVPP2_CLS_C3_MISS_TBL_SIZE - 1))
		return -EINVAL;

	/*set miss bit to 1, ppv2.1 mas 3.16*/
	index = (lkp_type | MVPP2_CLS3_DB_MISS_MASK);

	/*write entry index*/
	pp2_reg_write(cpu_slot, MVPP2_CLS3_DB_INDEX_REG, index);

	/*counter read*/
	counter = pp2_reg_read(cpu_slot, MVPP2_CLS3_HIT_COUNTER_REG) & MVPP2_CLS3_HIT_COUNTER_MASK;

	if (!cntr)
		pr_info("LKPT:0x%3.3x	COUNTER VAL:0x%6.6x\n", lkp_type, counter);
	else
		*cntr = counter;
	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_hit_cntrs_read_all(uintptr_t cpu_slot)
{
	u32 counter, index;

	for (index = 0; index < MVPP2_CLS_C3_HASH_TBL_SIZE; index++) {
		pp2_cls_c3_hit_cntrs_read(cpu_slot, index, &counter);

		/* skip initial counter value */
		if (counter == 0)
			continue;

		pr_info("ADDR:0x%3.3x	COUNTER VAL:0x%6.6x\n", index, counter);
	}

	for (index = 0; index < MVPP2_CLS_C3_MISS_TBL_SIZE; index++) {
		pp2_cls_c3_hit_cntrs_miss_read(cpu_slot, index, &counter);

		/* skip initial counter value */
		if (counter == 0)
			continue;

		pr_info("LKPT:0x%3.3x	COUNTER VAL:0x%6.6x\n", index, counter);
	}
	return 0;
}

/*-------------------------------------------------------------------------------*/
/*	 APIs for Classification C3 hit counters scan fields operation			  */
/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_scan_start(uintptr_t cpu_slot)
{
	int complete, iter = 0;

	/* trigger scan operation */
	pp2_reg_write(cpu_slot, MVPP2_CLS3_SC_ACT_REG, (1 << MVPP2_CLS3_SC_ACT));

	do {
		complete = pp2_cls_c3_scan_complete(cpu_slot);

	} while ((!complete) && ((iter++) < RETRIES_EXCEEDED));/*scan compleated*/

	if (iter >= RETRIES_EXCEEDED)
		return -EBUSY;

	return 0;
}

/*-------------------------------------------------------------------------------*/
/*mod = 0 below th . mode = 1 above threshold*/
int pp2_cls_c3_scan_thresh_set(uintptr_t cpu_slot, int mode, int thresh)
{
	u32 reg_val;

	if (mv_pp2x_range_validate(mode, 0, 1))
		return -EINVAL;

	if (mv_pp2x_range_validate(thresh, 0, MVPP2_CLS3_SC_TH_MAX))
		return -EINVAL;

	reg_val = pp2_reg_read(cpu_slot, MVPP2_CLS3_SC_PROP_REG);
	reg_val &= ~MVPP2_CLS3_SC_PROP_TH_MODE_MASK;
	reg_val |= (mode << MVPP2_CLS3_SC_PROP_TH_MODE);
	pp2_reg_write(cpu_slot, MVPP2_CLS3_SC_PROP_REG, reg_val);

	reg_val = pp2_reg_read(cpu_slot, MVPP2_CLS3_SC_TH_REG);
	reg_val &= ~MVPP2_CLS3_SC_TH_MASK;
	reg_val |= (thresh << MVPP2_CLS3_SC_TH);
	pp2_reg_write(cpu_slot, MVPP2_CLS3_SC_TH_REG, reg_val);

	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_scan_lkp_type_set(uintptr_t cpu_slot, int type)
{
	u32 prop;

	if (mv_pp2x_range_validate(type, -1, MVPP2_CLS3_SC_PROP_LKP_TYPE_MAX))
		return -EINVAL;

	prop = pp2_reg_read(cpu_slot, MVPP2_CLS3_SC_PROP_REG);

	if (type == -1)
		/* scan all entries */
		prop &= ~(1 << MVPP2_CLS3_SC_PROP_LKP_TYPE_EN);
	else {
		/* scan according to lookup type */
		prop |= (1 << MVPP2_CLS3_SC_PROP_LKP_TYPE_EN);
		prop &= ~MVPP2_CLS3_SC_PROP_LKP_TYPE_MASK;
		prop |= (type << MVPP2_CLS3_SC_PROP_LKP_TYPE);
	}

	pp2_reg_write(cpu_slot, MVPP2_CLS3_SC_PROP_REG, prop);

	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_scan_clear_before_en_set(uintptr_t cpu_slot, int en)
{
	u32 prop;

	if (mv_pp2x_range_validate(en, 0, 1))
		return -EINVAL;

	prop = pp2_reg_read(cpu_slot, MVPP2_CLS3_SC_PROP_REG);

	prop &= ~MVPP2_CLS3_SC_PROP_CLEAR_MASK;
	prop |= (en << MVPP2_CLS3_SC_PROP_CLEAR);

	pp2_reg_write(cpu_slot, MVPP2_CLS3_SC_PROP_REG, prop);

	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_scan_start_index_set(uintptr_t cpu_slot, int idx)
{
	u32 prop;

	if (mv_pp2x_range_validate(idx, 0, MVPP2_CLS3_HASH_OP_TBL_ADDR_MAX))
		return -EINVAL;

	prop = pp2_reg_read(cpu_slot, MVPP2_CLS3_SC_PROP_REG);

	prop &= ~MVPP2_CLS3_SC_PROP_START_ENTRY_MASK;
	prop |= (idx << MVPP2_CLS3_SC_PROP_START_ENTRY);

	pp2_reg_write(cpu_slot, MVPP2_CLS3_SC_PROP_REG, prop);

	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_scan_delay_set(uintptr_t cpu_slot, u32 time)
{
	u32 prop_val;

	if (mv_pp2x_range_validate(time, 0, MVPP2_CLS3_SC_PROP_VAL_DELAY_MAX))
		return -EINVAL;

	prop_val = pp2_reg_read(cpu_slot, MVPP2_CLS3_SC_PROP_VAL_REG);
	prop_val &= ~MVPP2_CLS3_SC_PROP_VAL_DELAY_MASK;
	prop_val |= (time << MVPP2_CLS3_SC_PROP_VAL_DELAY);
	pp2_reg_write(cpu_slot, MVPP2_CLS3_SC_PROP_VAL_REG, prop_val);

	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_scan_res_read(uintptr_t cpu_slot, int index, int *addr, int *cnt)
{
	u32 reg_val, sc_state, address, counter;
	int iter = 0;

	if (mv_pp2x_range_validate(index, 0, MVPP2_CLS_C3_SC_RES_TBL_SIZE - 1))
		return -EINVAL;

	do {
		pp2_cls_c3_scan_state_get(cpu_slot, &sc_state);
	} while (sc_state != 0 && ((iter++) < RETRIES_EXCEEDED));/*scan compleated*/

	if (iter >= RETRIES_EXCEEDED) {
		pr_err("%s:Error - retries exceeded.\n", __func__);
		return -EBUSY;
	}

	/*write index*/
	pp2_reg_write(cpu_slot, MVPP2_CLS3_SC_INDEX_REG, index);

	/*read date*/
	reg_val = pp2_reg_read(cpu_slot, MVPP2_CLS3_SC_RES_REG);
	address = (reg_val & MVPP2_CLS3_SC_RES_ENTRY_MASK) >> MVPP2_CLS3_SC_RES_ENTRY;
	counter = (reg_val & MVPP2_CLS3_SC_RES_CTR_MASK) >> MVPP2_CLS3_SC_RES_CTR;
	/* if one of parameters is null - func call from sysfs*/
	if ((!addr) | (!cnt)) {
		pr_info("INDEX:0x%2.2x	ADDR:0x%3.3x	COUNTER VAL:0x%6.6x\n", index, address, counter);
	} else {
		*addr = address;
		*cnt = counter;
	}

	return 0;
}

/*-------------------------------------------------------------------------------*/
int pp2_cls_c3_scan_num_of_res_get(uintptr_t cpu_slot, int *res_num)
{
	u32 reg_val, sc_state;
	int iter = 0;

	do {
		pp2_cls_c3_scan_state_get(cpu_slot, &sc_state);
	} while (sc_state != 0 && ((iter++) < RETRIES_EXCEEDED));/*scan compleated*/

	if (iter >= RETRIES_EXCEEDED) {
		pr_err("%s:Error - retries exceeded.\n", __func__);
		return -EBUSY;
	}

	reg_val = pp2_reg_read(cpu_slot, MVPP2_CLS3_STATE_REG);
	reg_val &= MVPP2_CLS3_STATE_NO_OF_SC_RES_MASK;
	reg_val >>= MVPP2_CLS3_STATE_NO_OF_SC_RES;
	*res_num = reg_val;
	return 0;
}

/*-------------------------------------------------------------------------------*/

/* *INDENT-ON* */
