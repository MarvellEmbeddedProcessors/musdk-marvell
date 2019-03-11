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

#include "std_internal.h"

#include "../pp2_types.h"
#include "../pp2.h"
#include "../pp2_hw_type.h"
#include "pp2_prs.h"

/* Flow ID definition array */
static struct mv_pp2x_prs_flow_id
	mv_pp2x_prs_flow_id_array[MVPP2_PRS_FL_TCAM_NUM] = {
	/***********#Flow ID#**************#Result Info#************/
	/* TCP over IPv4 flows, Not fragmented, no vlan tag */
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

	/* TCP over IPv4 flows, Not fragmented, with vlan tag */
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

	/* TCP over IPv4 flows, fragmented, no vlan tag */
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

	/* TCP over IPv4 flows, fragmented, with vlan tag */
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

	/* UDP over IPv4 flows, Not fragmented, no vlan tag */
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

	/* UDP over IPv4 flows, Not fragmented, with vlan tag */
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

	/* UDP over IPv4 flows, fragmented, no vlan tag */
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

	/* UDP over IPv4 flows, fragmented, with vlan tag */
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

	/* TCP over IPv6 flows, not fragmented, no vlan tag */
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

	/* TCP over IPv6 flows, not fragmented, with vlan tag */
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

	/* TCP over IPv6 flows, fragmented, no vlan tag */
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

	/* TCP over IPv6 flows, fragmented, with vlan tag */
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

	/* UDP over IPv6 flows, not fragmented, no vlan tag */
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

	/* UDP over IPv6 flows, not fragmented, with vlan tag */
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

	/* UDP over IPv6 flows, fragmented, no vlan tag */
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

	/* UDP over IPv6 flows, fragmented, with vlan tag */
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

	/* IPv4 flows, no vlan tag */
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

	/* IPv4 flows, with vlan tag */
	{MVPP2_PRS_FL_IP4_TAG, {MVPP2_PRS_RI_L3_IP4,
				MVPP2_PRS_RI_L3_PROTO_MASK} },
	{MVPP2_PRS_FL_IP4_TAG, {MVPP2_PRS_RI_L3_IP4_OPT,
				MVPP2_PRS_RI_L3_PROTO_MASK} },
	{MVPP2_PRS_FL_IP4_TAG, {MVPP2_PRS_RI_L3_IP4_OTHER,
				MVPP2_PRS_RI_L3_PROTO_MASK} },

	/* IPv6 flows, no vlan tag */
	{MVPP2_PRS_FL_IP6_UNTAG, {MVPP2_PRS_RI_VLAN_NONE |
				  MVPP2_PRS_RI_L3_IP6,
				  MVPP2_PRS_RI_VLAN_MASK |
				  MVPP2_PRS_RI_L3_PROTO_MASK} },
	{MVPP2_PRS_FL_IP6_UNTAG, {MVPP2_PRS_RI_VLAN_NONE |
				  MVPP2_PRS_RI_L3_IP6_EXT,
				  MVPP2_PRS_RI_VLAN_MASK |
				  MVPP2_PRS_RI_L3_PROTO_MASK} },

	/* IPv6 flows, with vlan tag */
	{MVPP2_PRS_FL_IP6_TAG, {MVPP2_PRS_RI_L3_IP6,
				MVPP2_PRS_RI_L3_PROTO_MASK} },
	{MVPP2_PRS_FL_IP6_TAG, {MVPP2_PRS_RI_L3_IP6_EXT,
				MVPP2_PRS_RI_L3_PROTO_MASK} },

	/* Non IP flow, no vlan tag */
	{MVPP2_PRS_FL_NON_IP_UNTAG, {MVPP2_PRS_RI_VLAN_NONE,
				     MVPP2_PRS_RI_VLAN_MASK} },

	/* Non IP flow, with vlan tag */
	{MVPP2_PRS_FL_NON_IP_TAG, {0, 0} },
};

/* For backwards compatibility to LK 4.4, the following array is added.
 * This array may be removed once no more backwards compatibility is
 * needed.
 */
static struct mv_pp2x_prs_flow_id
	mv_pp2x_prs_flow_id_array_4_4[MVPP2_PRS_FL_TCAM_NUM] = {
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
	enum musdk_lnx_id lnx_id = lnx_id_get();
	struct mv_pp2x_prs_flow_id *prs_flow_id_array;

	/* For backwards compatibility to LK 4.4 */
	if (lnx_is_mainline(lnx_id))
		prs_flow_id_array = mv_pp2x_prs_flow_id_array;
	else
		prs_flow_id_array = mv_pp2x_prs_flow_id_array_4_4;

	for (index = 0; index < MVPP2_PRS_FL_TCAM_NUM; index++) {
		ri = prs_flow_id_array[index].prs_result.ri;
		ri_mask = prs_flow_id_array[index].prs_result.ri_mask;
		flow_id = prs_flow_id_array[index].flow_id;

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
					 const u8 *da, const u8 *mask)
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


/* pp2_prs_create_entry
 *
 * DESCRIPTION:	Create a new entry in PRS
 *
 * INPUTS:	port	- logical port to be configured
 *		index	- index to read from
 *		target	- target classification (logical port or nic)
 *
 * OUTPUTS:
 *
 * RETURNS:	-1 on error
 */
static int pp2_prs_create_entry(struct pp2_port *port, u32 index, enum pp2_ppio_cls_target target)
{
	struct pp2_inst *inst = port->parent;
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);
	u32 ri = 0;
	struct mv_pp2x_prs_entry pe_orig, pe_log_port;
	int tid;

	/* create a new MH entry for the specified port and set UDF7 to log_port */
	memset(&pe_orig, 0, sizeof(struct mv_pp2x_prs_entry));
	memset(&pe_log_port, 0, sizeof(struct mv_pp2x_prs_entry));

	/* Read pe from HW */
	pe_orig.index = index;
	mv_pp2x_prs_hw_read(cpu_slot, &pe_orig);

	memcpy(&pe_log_port, &pe_orig, sizeof(struct mv_pp2x_prs_entry));

	/* Find first empty slot in TCAM */
	tid = pp2_prs_tcam_first_free(inst, MVPP2_PE_FIRST_FREE_TID, MVPP2_PE_LAST_FREE_TID);
	if (tid < 0)
		return tid;

	/* Update ri and ri_mask */
	pe_log_port.index = tid;

	if (target == PP2_CLS_TARGET_LOCAL_PPIO)
		ri = MVPP2_PRS_RI_UDF7_LOG_PORT;
	else
		ri = MVPP2_PRS_RI_UDF7_NIC;

	mv_pp2x_prs_sram_ri_update(&pe_log_port, ri, MVPP2_PRS_RI_UDF7_MASK);

	/* Mask all ports */
	mv_pp2x_prs_tcam_port_map_set(&pe_log_port, 0);

	/* Update port mask */
	mv_pp2x_prs_tcam_port_set(&pe_log_port, port->id, true);

	pr_debug("target %d, ri %x\n", target, ri);

	/* Update shadow table and hw entry for new entry*/
	mv_pp2x_prs_shadow_set(inst, pe_log_port.index, mv_pp2x_prs_tcam_lu_get(&pe_log_port));
	mv_pp2x_prs_shadow_ri_set(inst, pe_log_port.index, ri, MVPP2_PRS_RI_UDF7_MASK);
	mv_pp2x_prs_hw_write(cpu_slot, &pe_log_port);

	/* update port mask of existing non-logical entry */
	mv_pp2x_prs_tcam_port_set(&pe_orig, port->id, false);

	/* write entry to HW */
	mv_pp2x_prs_hw_write(cpu_slot, &pe_orig);

	return 0;
}

/* pp2_prs_proto_lookup
 *
 * DESCRIPTION:	convert from proto defined in mv_net.h file to prs lookup and protocol to match in TCAM
 *
 * INPUTS:	proto		- protocol to convert
 *
 * OUTPUTS:	lookup		- parser lookup id
 *		proto_num	- protocol number to match in TCAM
 *
 * RETURNS:	-1 on error
 */
static int pp2_prs_proto_lookup(u16 proto, u16 lookup[], u16 proto_num[])
{
	switch (proto) {
	case MV_NET_PROTO_VLAN:
		proto_num[0] = ETH_P_8021Q;
		proto_num[1] = ETH_P_8021AD;
		lookup[0] = MVPP2_PRS_LU_VLAN;
		break;
	case MV_NET_PROTO_ARP:
		proto_num[0] = ARP_PROTO;
		lookup[0] = MVPP2_PRS_LU_L2;
		break;
	case MV_NET_PROTO_PPPOE:
		proto_num[0] = PPPOE_PROTO;
		lookup[0] = MVPP2_PRS_LU_L2;
		break;
	case MV_NET_PROTO_IP:
		proto_num[0] = ETH_P_IP;
		proto_num[1] = ETH_P_IPV6;
		lookup[0] = MVPP2_PRS_LU_L2;
		break;
	case MV_NET_PROTO_IP4:
		proto_num[0] = ETH_P_IP;
		lookup[0] = MVPP2_PRS_LU_L2;
		break;
	case MV_NET_PROTO_IP6:
		proto_num[0] = ETH_P_IPV6;
		lookup[0] = MVPP2_PRS_LU_L2;
		break;
		break;
	case MV_NET_PROTO_TCP:
		proto_num[0] = IPPROTO_TCP;
		lookup[0] = MVPP2_PRS_LU_IP4;
		lookup[1] = MVPP2_PRS_LU_IP6;
		break;
	case MV_NET_PROTO_UDP:
		proto_num[0] = IPPROTO_UDP;
		lookup[0] = MVPP2_PRS_LU_IP4;
		lookup[1] = MVPP2_PRS_LU_IP6;
		break;
	case MV_NET_PROTO_ICMP:
		proto_num[0] = IPPROTO_ICMP;
		lookup[0] = MVPP2_PRS_LU_IP4;
		lookup[1] = MVPP2_PRS_LU_IP6;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/* pp2_prs_tcam_idx_list_build
 *
 * DESCRIPTION:	return tcam indexes that matches the specified lookup
 *
 * INPUTS:	inst	- packet processor instance
 *		proto	- protocol to update in parser
 *		negate	- match protocol or negated protocol
 *		ri	- results info field to configure
 *
 * OUTPUTS:	tcam_list	- list of TCAM indexes which match the lookup id
 *
 * RETURNS:	0 on success, error-code otherwise
 */
static int pp2_prs_tcam_idx_list_build(struct pp2_inst *inst, u32 lookup, u16 proto, int negate, u32 ri)
{
	int tid, i = 0;
	u8 byte;
	u16 word;
	u8 tcam_ai;
	int update = false;
	int found = 0;
	struct mv_pp2x_prs_shadow *prs_shadow = inst->cls_db->prs_db.prs_shadow;
	struct prs_log_port_tcam_negated_proto_node *neg_proto_node;
	int rc;

	for (tid = MVPP2_PE_FIRST_FREE_TID;
	     tid <= MVPP2_PRS_TCAM_SRAM_SIZE; tid++) {

		update = false;

		if (i >= MVPP2_PE_TID_SIZE)
			return -EFAULT;

		if (tid == MVPP2_PE_LAST_FREE_TID)
			/* Skip parser filtering area and increment to start of default area */
			tid = MVPP2_PE_LAST_FREE_TID + MVPP2_PRS_MAC_RANGE_SIZE + MVPP2_PRS_VLAN_FILT_RANGE_SIZE;

		if (!prs_shadow[tid].valid)
			continue;

		if (prs_shadow[tid].lu != lookup)
			continue;

		if (negate) {
			/* Add protocol to negated list */
			rc = pp2_prs_tcam_neg_proto_check(inst, proto);
			if (!rc) {
				neg_proto_node = kmalloc(sizeof(*neg_proto_node), GFP_KERNEL);
				if (!neg_proto_node)
					return -ENOMEM;

				neg_proto_node->proto = proto;
				list_add_to_tail(&neg_proto_node->list_node, &inst->cls_db->prs_db.tcam_neg_proto_list);
			}
		}

		switch (proto) {
		case ARP_PROTO:
		case PPPOE_PROTO:
		case ETH_P_IP:
		case ETH_P_IPV6:
		case ETH_P_8021Q:
		case ETH_P_8021AD:
			/* For L2, need to match type in TCAM words 0 and 1 */
			word = (prs_shadow[tid].tcam.byte[TCAM_DATA_BYTE(0)] << 8) +
				prs_shadow[tid].tcam.byte[TCAM_DATA_BYTE(1)];
			if ((word == proto && negate == 0) ||
			    (word != proto && negate == 1)) {
				/* Check if protocol was already negated before. In this case, skip adding it */
				found = pp2_prs_tcam_neg_proto_check(inst, word);
				if (found)
					continue;

				update = true;
			} else if (word == proto && negate == 1) {
				/* If negated, need to check if the protocol was already added to
				 * the match list (maybe in another rule). In this case, remove from the list
				 */
				found = pp2_cls_db_prs_match_list_check(inst, tid);
				if (found)
					pp2_cls_db_prs_match_list_remove_idx(inst, tid);
			}
			break;
		case IPPROTO_TCP:
		case IPPROTO_UDP:
		case IPPROTO_ICMP:
			if (lookup == MVPP2_PRS_LU_IP4) {
				/* For L3, need to match the protocol in TCAM byte 5 */
				byte = prs_shadow[tid].tcam.byte[TCAM_DATA_BYTE(5)];
				if ((byte == proto && negate == 0) ||
				    (byte != proto && negate == 1)) {
					/*
					* In IPv4 parser entries, there are 2 rounds performed:
					* the first round is to match the protocol, while the second round
					* is to set the cast flag. The entries needed are those that match
					* the protocol and not the ones that set the cast flag
					*/
					tcam_ai = prs_shadow[tid].tcam.byte[HW_BYTE_OFFS(MVPP2_PRS_TCAM_AI_BYTE)];
					if (tcam_ai != 0x0)
						continue;

					/* Check if protocol was already negated before. In this case, skip adding it */
					found = pp2_prs_tcam_neg_proto_check(inst, byte);
					if (found)
						continue;

					update = true;
				} else if (byte == proto && negate == 1) {
					/* If negated, need to check if the protocol was already added to
					 * the match list (maybe in another rule). In this case, remove from the list
					 */
					found = pp2_cls_db_prs_match_list_check(inst, tid);
					if (found)
						pp2_cls_db_prs_match_list_remove_idx(inst, tid);
				}
			} else if (lookup == MVPP2_PRS_LU_IP6) {
				/* For IPv6, need to match the protocol in TCAM byte 0 */
				byte = prs_shadow[tid].tcam.byte[TCAM_DATA_BYTE(0)];
				if ((byte == proto && negate == 0) ||
				    (byte != proto && negate == 1)) {
					/*
					* In IPv6 parser entries, there are 2 rounds performed:
					* the first round is to match the cast flag, while the second round
					* is to set the protocol. The entries needed are those that match
					* the protocol and not the ones that set the cast flag
					*/
					tcam_ai = prs_shadow[tid].tcam.byte[HW_BYTE_OFFS(MVPP2_PRS_TCAM_AI_BYTE)];
					if (tcam_ai != 0x1)
						continue;

					/* Check if protocol was already negated before. In this case, skip adding it */
					found = pp2_prs_tcam_neg_proto_check(inst, byte);
					if (found)
						continue;

					update = true;
				} else if (byte == proto && negate == 1) {
					/* If negated, need to check if the protocol was already added to
					 * the match list (maybe in another rule). In this case, remove from the list
					 */
					found = pp2_cls_db_prs_match_list_check(inst, tid);
					if (found)
						pp2_cls_db_prs_match_list_remove_idx(inst, tid);
				}
		}
			break;
		case 0:
			/* DSA will enter here */
			update = true;
		default:
			pr_err("No matching protocol found for proto: %x , lookup %d\n", proto, lookup);
		}

		if (update) {
			/* add match to db */
			found = pp2_cls_db_prs_match_list_check(inst, tid);
			if (!found) {
				if (prs_shadow[tid].ri & ri)
					pp2_cls_db_prs_match_list_add(inst, tid, 1);
				else
					pp2_cls_db_prs_match_list_add(inst, tid, 0);
				i++;
			}
		}
	}

	return 0;
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

	if (extend) {
		tid = tagged ? MVPP2_PE_ETYPE_EDSA_TAGGED :
		      MVPP2_PE_ETYPE_EDSA_UNTAGGED;
		shift = 8;
	} else {
		tid = tagged ? MVPP2_PE_ETYPE_DSA_TAGGED :
		      MVPP2_PE_ETYPE_DSA_UNTAGGED;
		shift = 4;
	}

	/* Build a list with indexes matching the specified lookup id and proto */
	pp2_prs_tcam_idx_list_build(inst, MVPP2_PRS_LU_DSA, 0, 0, ri);

	if (!pp2_cls_db_prs_match_list_log_port_check(inst)) {
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
		ri = MVPP2_PRS_RI_UDF7_LOG_PORT;
		nri = MVPP2_PRS_RI_UDF7_NIC;
	} else {
		ri = MVPP2_PRS_RI_UDF7_NIC;
		nri = MVPP2_PRS_RI_UDF7_LOG_PORT;
	}

	ri_mask = MVPP2_PRS_RI_UDF7_MASK;

	pr_debug("%s target %d, ri %x, nri %x, mask %x\n", __func__, target, ri, nri, ri_mask);

	switch (type) {
	case MVPP2_TAG_TYPE_EDSA:
		/* create new entries for DSA mode*/
		pp2_prs_dsa_tag_mode_set(port, val, MVPP2_PRS_TAGGED, MVPP2_PRS_EDSA, ri, ri_mask);
		pp2_prs_dsa_tag_mode_set(port, val, MVPP2_PRS_UNTAGGED, MVPP2_PRS_EDSA, ri, ri_mask);
		break;
	case MVPP2_TAG_TYPE_DSA:
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
static int mv_pp2x_prs_log_port_init(struct pp2_inst *inst)
{
	u32 i;
	struct mv_pp2x_prs_entry pe;
	struct mv_pp2x_prs_shadow *prs_shadow = inst->cls_db->prs_db.prs_shadow;
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);

	if (!prs_shadow) {
		pr_err("prs_shadow is null\n");
		return -EFAULT;
	}

	/* Init parser logical port lists*/
	pp2_cls_db_prs_init_list(inst);

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

		/* Set default UDF7 to all MH entries to send traffic to kernel */
		if (prs_shadow[i].valid && prs_shadow[i].lu == MVPP2_PRS_LU_MH) {
			pe.index = i;
			mv_pp2x_prs_hw_read(cpu_slot, &pe);
			mv_pp2x_prs_sram_ri_update(&pe, MVPP2_PRS_RI_UDF7_CLEAR, MVPP2_PRS_RI_UDF7_MASK);
			mv_pp2x_prs_sram_ri_update(&pe, MVPP2_PRS_RI_UDF7_NIC, MVPP2_PRS_RI_UDF7_MASK);
			mv_pp2x_prs_hw_write(cpu_slot, &pe);
		}
	}
	return 0;
}

/* pp2_prs_log_port_proto_update
 *
 * DESCRIPTION: Add to parser new logical port entries
 *
 * INPUTS:	port	- port to write
 *		target	- target classification (logical port or nic)
 *
 * OUTPUTS:	None
 *
 * RETURNS:	0 on success, error-code otherwise
 */
static int pp2_prs_log_port_proto_update(struct pp2_port *port, enum pp2_ppio_cls_target target)
{
	int i;
	struct pp2_inst *inst = port->parent;
	struct prs_log_port_tcam_node tcam_match_node;

	for (i = 0; i < pp2_cls_db_prs_match_list_num_get(inst); i++) {
		pp2_cls_db_prs_match_list_idx_get(inst, i, &tcam_match_node);

		if (tcam_match_node.log_port == 0)
			pp2_prs_create_entry(port, tcam_match_node.idx, target);
	}

	return 0;
}

/* pp2_prs_log_port_proto_set
 *
 * DESCRIPTION:	Build a list of parser entries according to logical port requirements
 *
 * INPUTS:	proto -protocol to update in parser
 *		negate	- match protocol or negated protocol
 *		target	- target classification (logical port or nic)
 *
 * OUTPUTS:	None
 *
 * RETURNS:	0 on success, error-code otherwise
 */
static int pp2_prs_log_port_proto_set(struct pp2_port *port, enum mv_net_proto proto, int negate,
				      enum pp2_ppio_cls_target target)
{
	struct pp2_inst *inst = port->parent;
	u32 ri = 0;
	u16 lookup[MAX_LOOKUP] = {0, 0, 0};
	u16 proto_num[MAX_PROTO_NUM] = {0, 0, 0};
	int i, j, rc;

	rc = pp2_prs_proto_lookup(proto, lookup, proto_num);

	if (rc)
		return -EFAULT;

	if (target == PP2_CLS_TARGET_LOCAL_PPIO)
		ri = MVPP2_PRS_RI_UDF7_LOG_PORT;
	else
		ri = MVPP2_PRS_RI_UDF7_NIC;

	for (i = 0; i < MAX_LOOKUP; i++) {

		if (lookup[i] == 0)
			continue;

		for (j = 0; j < MAX_PROTO_NUM; j++) {

			if (proto_num[j] == 0)
				continue;

			pr_info("Logical port: Building list for lookup %s, protocol %s\n",
				pp2_g_enum_prs_lookup_str_get(lookup[i]),
				pp2_g_enum_prs_proto_num_str_get(proto_num[j]));
			/* Build a list with indexes matching the specified lookup id and proto */
			rc = pp2_prs_tcam_idx_list_build(inst, lookup[i], proto_num[j], negate, ri);
			if (rc) {
				pr_err("Logical port: no space in TCAM for adding specified rules. Operation failed\n");
				return -EFAULT;
			}
		}
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
	 * By default parser is initialized with PP2_CLS_TARGET_LOCAL_PPIO, so only need to change
	 * if target is PP2_CLS_TARGET_OTHER
	 */
	if (params->proto_based_target.target == PP2_CLS_TARGET_OTHER)
		pp2_prs_create_entry(port, MVPP2_PE_MH_DEFAULT, PP2_CLS_TARGET_LOCAL_PPIO);

	/* Go over all requested protocols and protocol fields*/
	for (i = 0; i < params->proto_based_target.num_proto_rule_sets; i++) {
		for (j = 0; j < params->proto_based_target.rule_sets[i].num_rules; j++) {
			struct pp2_ppio_log_port_rule_params *rule_params =
				&params->proto_based_target.rule_sets[i].rules[j];

			pr_debug("%d:%d %d\n", i, j, rule_params->rule_type);
			if (rule_params->rule_type == PP2_RULE_TYPE_PROTO) {
				/* Create a list of protocols according to imputs. This list will then be
				 * written to parser once completed
				 */
				rc = pp2_prs_log_port_proto_set(port, rule_params->u.proto_params.proto,
								rule_params->u.proto_params.val,
								params->proto_based_target.target);
				if (rc)
					return -EFAULT;
			} else if (rule_params->rule_type == PP2_RULE_TYPE_PROTO_FIELD) {
				/* Create a list of protocol fields according to imputs. This list will then be
				 * written to parser once completed
				 */
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

	/* List of protocols and protool fields was build, now need to create the new entries in parser */
	pp2_prs_log_port_proto_update(port,  params->proto_based_target.target);

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
	static int i, j, invalid, mac_range_start = -1, mac_range_end = -1;
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

		/* Dynamically find the mac_range from hw_parser configuration */
		if (!invalid && mac_range_start == -1 && prs_shadow[i].lu == MVPP2_PRS_LU_MAC
		    && i >= MVPP2_PE_FIRST_FREE_TID)
			mac_range_start = i;
		if (!invalid && mac_range_start != -1 && mac_range_end == -1 && prs_shadow[i].lu != MVPP2_PRS_LU_MAC)
			mac_range_end = i - 1;
	}
	prs_shadow->prs_mac_range_start = (u32) mac_range_start;
	prs_shadow->prs_mac_range_end   = (u32) mac_range_end;
	pr_debug("%s: mac_start:%u, mac_end:%u\n", __func__, prs_shadow->prs_mac_range_start,
		 prs_shadow->prs_mac_range_end);
	return 0;
}


/* pp2_prs_eth_start_header_set
 *
 * DESCRIPTION:	Configure a port to be DSA aware
 *
 * INPUTS:	port
 *		dsa-val
 *
 * OUTPUTS:	None
 *
 * RETURNS:	0 on success, error-code otherwise
 */
int pp2_prs_eth_start_header_set(struct pp2_port *port, enum pp2_ppio_eth_start_hdr mode)
{
	u32 type;
	int rc;
	u32 nri = 0, ri_mask = 0;

	rc = pp2_prs_eth_start_hdr_set(port, mode);
	if (rc)
		return -EFAULT;

	/*Get MH register configured mode */
	type = pp2_prs_eth_start_hdr_get(port);

	/* Configure parser DSA entries */
	switch (type) {
	case MVPP2_TAG_TYPE_EDSA:
		/* Add port to EDSA entries */
		pp2_prs_port_update(port, true, MVPP2_PE_EDSA_TAGGED, nri, ri_mask);
		pp2_prs_port_update(port, true, MVPP2_PE_EDSA_UNTAGGED, nri, ri_mask);

		/* Remove port from DSA entries */
		pp2_prs_port_update(port, false, MVPP2_PE_DSA_TAGGED, nri, ri_mask);
		pp2_prs_port_update(port, false, MVPP2_PE_DSA_UNTAGGED, nri, ri_mask);

		break;
	case MVPP2_TAG_TYPE_DSA:
		/* Add port to DSA entries */
		pp2_prs_port_update(port, true, MVPP2_PE_DSA_TAGGED, nri, ri_mask);
		pp2_prs_port_update(port, true, MVPP2_PE_DSA_UNTAGGED, nri, ri_mask);

		/* Remove port from EDSA entries */
		pp2_prs_port_update(port, false, MVPP2_PE_EDSA_TAGGED, nri, ri_mask);
		pp2_prs_port_update(port, false, MVPP2_PE_EDSA_UNTAGGED, nri, ri_mask);

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

	/* remove all tables */
	pp2_cls_db_prs_match_list_remove(inst);

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
	rc = mv_pp2x_prs_log_port_init(inst);
	if (rc)
		return -EINVAL;

	/* MUSDK local parser flow id attribute tbl init (used in classifier)
	 * TODO  this table needs to be synchronized with kernel
	 * TODO  this table will be dynamic once struct pp2_parse_params is implemented
	 */
	mv_pp2x_prs_flow_id_attr_init();

	return 0;
}

/* Find tcam entry with matched pair <MAC DA, port> */
static int mvpp2x_prs_mac_da_range_find(struct pp2_inst *inst, uintptr_t cpu_slot, int pmap, const u8 *da,
					const u8 *mask, int udf_type)
{
	struct mv_pp2x_prs_entry pe;
	int tid;
	struct mv_pp2x_prs_shadow *prs_shadow = inst->cls_db->prs_db.prs_shadow;

	/* Go through all entries with MVPP2_PRS_LU_MAC */
	for (tid = prs_shadow->prs_mac_range_start;
	     tid <= prs_shadow->prs_mac_range_end; tid++) {
		unsigned int entry_pmap;

		if (!prs_shadow[tid].valid || prs_shadow[tid].lu != MVPP2_PRS_LU_MAC)
			continue;
		pe.index = tid;
		mv_pp2x_prs_hw_read(cpu_slot, &pe);
		entry_pmap = mv_pp2x_prs_tcam_port_map_get(&pe);

		if (mv_pp2x_prs_mac_range_equals(&pe, da, mask)) {
			pr_debug("maps: %d:%d\n", entry_pmap, pmap);
			if (entry_pmap == pmap)
				return tid;
		}
	}

	return -ENOENT;
}


/* mv_pp2x_prs_clear_active_vlans
 *
 * DESCRIPTION:	Read active vlans into vlans array and clear them from shadow
 *
 * INPUTS:	port
 *
 * OUTPUTS:	vlans - vlan filtering entries configured in PRS for input port
 *
 * RETURNS:	none
 */
void mv_pp2x_prs_clear_active_vlans(struct pp2_port *port, uint32_t *vlans)
{
	struct pp2_inst *inst = port->parent;
	struct mv_pp2x_prs_shadow *prs_shadow = inst->cls_db->prs_db.prs_shadow;
	int index = 0;
	int tid;

	for (tid = MVPP2_PRS_VID_PORT_FIRST(port->id);
	     tid <= MVPP2_PRS_VID_PORT_LAST(port->id); tid++) {
		if (prs_shadow[tid].valid && prs_shadow[tid].lu == MVPP2_PRS_LU_VID) {
			vlans[index++] = ((prs_shadow[tid].tcam.byte[TCAM_DATA_BYTE(2)] & 0xF) << 8) +
					 prs_shadow[tid].tcam.byte[TCAM_DATA_BYTE(3)];
			prs_shadow[tid].valid = false;
		}
	}
}


/* Update parser's mac da entry */
int mv_pp2x_prs_mac_da_accept(struct pp2_port *port, const u8 *da, bool add)
{
	unsigned char mask[ETH_ALEN] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
	unsigned int pmap, len, ri;
	struct mv_pp2x_prs_shadow *prs_shadow = port->parent->cls_db->prs_db.prs_shadow;
	struct mv_pp2x_prs_entry pe;
	int tid;

	memset(&pe, 0, sizeof(pe));

	/* Scan TCAM and see if entry with this <MAC DA, port> already exist */
	tid = mvpp2x_prs_mac_da_range_find(port->parent, port->cpu_slot, BIT(port->id), da, mask, 0);

	/* No such entry */
	if (tid < 0) {
		if (!add)
			return 0;

		/* Create new TCAM entry */
		/* Go through the all entries from first to last */
		tid = pp2_prs_tcam_first_free(port->parent, prs_shadow->prs_mac_range_start,
					      prs_shadow->prs_mac_range_end);
		if (tid < 0)
			return tid;

		pe.index = tid;

		/* Mask all ports */
		mv_pp2x_prs_tcam_port_map_set(&pe, 0);
	} else {
		pe.index = tid;
		mv_pp2x_prs_hw_read(port->cpu_slot, &pe);
	}

	mv_pp2x_prs_tcam_lu_set(&pe, MVPP2_PRS_LU_MAC);

	/* Update port mask */
	mv_pp2x_prs_tcam_port_set(&pe, port->id, add);

	/* Invalidate the entry if no ports are left enabled */
	pmap = mv_pp2x_prs_tcam_port_map_get(&pe);
	if (pmap == 0) {
		if (add)
			return -EINVAL;

		mv_pp2x_prs_hw_inv(port->cpu_slot, pe.index);
		prs_shadow[pe.index].valid = false;
		return 0;
	}

	/* Continue - set next lookup */
	mv_pp2x_prs_sram_next_lu_set(&pe, MVPP2_PRS_LU_DSA);

	/* Set match on DA */
	len = ETH_ALEN;
	while (len--)
		mv_pp2x_prs_tcam_data_byte_set(&pe, len, da[len], 0xff);

	/* Set result info bits */
	if (mv_check_eaddr_bc(da)) {
		ri = MVPP2_PRS_RI_L2_BCAST;
	} else if (mv_check_eaddr_mc(da)) {
		ri = MVPP2_PRS_RI_L2_MCAST;
	} else {
		ri = MVPP2_PRS_RI_L2_UCAST;

		/* These mac_addresses are not the MAC-TO-ME address */
		/* ri |= MVPP2_PRS_RI_MAC_ME_MASK; */
	}

	mv_pp2x_prs_sram_ri_update(&pe, ri, MVPP2_PRS_RI_L2_CAST_MASK |
				   MVPP2_PRS_RI_MAC_ME_MASK);

	mv_pp2x_prs_shadow_ri_set(port->parent, pe.index, ri, MVPP2_PRS_RI_L2_CAST_MASK |
				  MVPP2_PRS_RI_MAC_ME_MASK);

	/* Shift to ethertype */
	mv_pp2x_prs_sram_shift_set(&pe, 2 * ETH_ALEN,
				   MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);

	/* Update shadow table and hw entry */
	mv_pp2x_prs_shadow_set(port->parent, pe.index, MVPP2_PRS_LU_MAC);
	mv_pp2x_prs_hw_write(port->cpu_slot, &pe);

	return 0;
}

static int mv_pp2x_prs_sw_sram_shift_get(struct mv_pp2x_prs_entry *pe, int *shift)
{
	int sign;

	if (mv_pp2x_ptr_validate(pe))
		return -1;
	if (mv_pp2x_ptr_validate(shift))
		return -1;

	sign = pe->sram.byte[SRAM_BIT_TO_BYTE(
		MVPP2_PRS_SRAM_SHIFT_SIGN_BIT)] &
		(1 << (MVPP2_PRS_SRAM_SHIFT_SIGN_BIT % 8));
	*shift = ((int)(pe->sram.byte[SRAM_BIT_TO_BYTE(
		MVPP2_PRS_SRAM_SHIFT_OFFS)])) &
		MVPP2_PRS_SRAM_SHIFT_MASK;

	if (sign == 1)
		*shift *= -1;
	return 0;
}

static int mv_pp2x_prs_sw_sram_offset_get(struct mv_pp2x_prs_entry *pe,
					  unsigned int *type, int *offset,
					  unsigned int *op)
{
	int sign;

	if (mv_pp2x_ptr_validate(pe))
		return -1;

	if (mv_pp2x_ptr_validate(offset))
		return -1;

	if (mv_pp2x_ptr_validate(type))
		return -1;

	*type = pe->sram.byte[SRAM_BIT_TO_BYTE(
		MVPP2_PRS_SRAM_UDF_TYPE_OFFS)] >>
		(MVPP2_PRS_SRAM_UDF_TYPE_OFFS % 8);
	*type &= MVPP2_PRS_SRAM_UDF_TYPE_MASK;

	*offset = (pe->sram.byte[SRAM_BIT_TO_BYTE(
		MVPP2_PRS_SRAM_UDF_OFFS)] >>
		(MVPP2_PRS_SRAM_UDF_OFFS % 8)) & 0x7f;
	*offset |= (pe->sram.byte[
		    SRAM_BIT_TO_BYTE(MVPP2_PRS_SRAM_UDF_OFFS +
		    MVPP2_PRS_SRAM_UDF_BITS)] <<
		    (8 - (MVPP2_PRS_SRAM_UDF_OFFS % 8))) & 0x80;

	*op = (pe->sram.byte[SRAM_BIT_TO_BYTE(
		MVPP2_PRS_SRAM_OP_SEL_SHIFT_OFFS)] >>
		(MVPP2_PRS_SRAM_OP_SEL_SHIFT_OFFS % 8)) & 0x7;
	*op |= (pe->sram.byte[SRAM_BIT_TO_BYTE(
		MVPP2_PRS_SRAM_OP_SEL_SHIFT_OFFS +
		MVPP2_PRS_SRAM_OP_SEL_SHIFT_BITS)] <<
		(8 - (MVPP2_PRS_SRAM_OP_SEL_SHIFT_OFFS % 8))) & 0x18;

	/* if signed bit is tes */
	sign = pe->sram.byte[SRAM_BIT_TO_BYTE(
		MVPP2_PRS_SRAM_UDF_SIGN_BIT)] &
		(1 << (MVPP2_PRS_SRAM_UDF_SIGN_BIT % 8));
	if (sign != 0)
		*offset = 1 - (*offset);

	return 0;
}

static int mv_pp2x_prs_sram_bit_get(struct mv_pp2x_prs_entry *pe, int bit_num,
				    unsigned int *bit)
{
	if (mv_pp2x_ptr_validate(pe))
		return -1;

	*bit = pe->sram.byte[SRAM_BIT_TO_BYTE(bit_num)]  &
		(1 << (bit_num % 8));
	*bit = (*bit) >> (bit_num % 8);
	return 0;
}

static int mv_pp2x_prs_sw_sram_ri_get(struct mv_pp2x_prs_entry *pe,
				      unsigned int *bits, unsigned int *enable)
{
	if (mv_pp2x_ptr_validate(pe))
		return -1;

	if (mv_pp2x_ptr_validate(bits))
		return -1;

	if (mv_pp2x_ptr_validate(enable))
		return -1;

	*bits = pe->sram.word[MVPP2_PRS_SRAM_RI_OFFS / 32];
	*enable = pe->sram.word[MVPP2_PRS_SRAM_RI_CTRL_OFFS / 32];
	return 0;
}

static int mv_pp2x_prs_sw_sram_ai_get(struct mv_pp2x_prs_entry *pe,
				      unsigned int *bits, unsigned int *enable)
{
	if (mv_pp2x_ptr_validate(pe))
		return -1;

	if (mv_pp2x_ptr_validate(bits))
		return -1;

	if (mv_pp2x_ptr_validate(enable))
		return -1;

	*bits = (pe->sram.byte[SRAM_BIT_TO_BYTE(
		MVPP2_PRS_SRAM_AI_OFFS)] >> (MVPP2_PRS_SRAM_AI_OFFS % 8)) |
		(pe->sram.byte[SRAM_BIT_TO_BYTE(
			MVPP2_PRS_SRAM_AI_OFFS +
			MVPP2_PRS_SRAM_AI_CTRL_BITS)] <<
			(8 - (MVPP2_PRS_SRAM_AI_OFFS % 8)));

	*enable = (pe->sram.byte[SRAM_BIT_TO_BYTE(
		MVPP2_PRS_SRAM_AI_CTRL_OFFS)] >>
		(MVPP2_PRS_SRAM_AI_CTRL_OFFS % 8)) |
		(pe->sram.byte[SRAM_BIT_TO_BYTE(
			MVPP2_PRS_SRAM_AI_CTRL_OFFS +
			MVPP2_PRS_SRAM_AI_CTRL_BITS)] <<
			(8 - (MVPP2_PRS_SRAM_AI_CTRL_OFFS % 8)));

	*bits &= MVPP2_PRS_SRAM_AI_MASK;
	*enable &= MVPP2_PRS_SRAM_AI_MASK;

	return 0;
}


static int mv_pp2x_prs_sw_sram_lu_done_get(struct mv_pp2x_prs_entry *pe,
					   unsigned int *bit)
{
	return mv_pp2x_prs_sram_bit_get(pe, MVPP2_PRS_SRAM_LU_DONE_BIT, bit);
}

static int mv_pp2x_prs_sw_sram_next_lu_get(struct mv_pp2x_prs_entry *pe,
					   unsigned int *lu)
{
	if (mv_pp2x_ptr_validate(pe))
		return -1;

	if (mv_pp2x_ptr_validate(lu))
		return -1;

	*lu = pe->sram.byte[SRAM_BIT_TO_BYTE(
		MVPP2_PRS_SRAM_NEXT_LU_OFFS)];
	*lu = ((*lu) >> MVPP2_PRS_SRAM_NEXT_LU_OFFS % 8);
	*lu &= MVPP2_PRS_SRAM_NEXT_LU_MASK;
	return 0;
}

static int mv_pp2x_prs_sw_sram_flowid_gen_get(struct mv_pp2x_prs_entry *pe,
				       unsigned int *bit)
{
	return mv_pp2x_prs_sram_bit_get(pe, MVPP2_PRS_SRAM_LU_GEN_BIT, bit);
}

static int mv_pp2x_prs_sw_sram_ri_dump(struct mv_pp2x_prs_entry *pe)
{
	unsigned int data, mask;
	int i, bitsOffs = 0;
	char bits[100];

	if (mv_pp2x_ptr_validate(pe))
		return -1;

	mv_pp2x_prs_sw_sram_ri_get(pe, &data, &mask);
	if (!mask)
		return 0;

	pr_info("\n       ");

	pr_info("S_RI=");
	for (i = (MVPP2_PRS_SRAM_RI_CTRL_BITS-1); i > -1 ; i--)
		if (mask & (1 << i)) {
			pr_info("%d", ((data & (1 << i)) != 0));
			bitsOffs += sprintf(bits + bitsOffs, "%d:", i);
		} else
			pr_info("x");

	bits[bitsOffs] = '\0';
	pr_info(" %s", bits);

	return 0;
}

static int mv_pp2x_prs_sw_sram_ai_dump(struct mv_pp2x_prs_entry *pe)
{
	int i, bitsOffs = 0;
	unsigned int data, mask;
	char bits[30];

	if (mv_pp2x_ptr_validate(pe))
		return -1;

	mv_pp2x_prs_sw_sram_ai_get(pe, &data, &mask);

	if (mask == 0)
		return 0;

	pr_info("\n       ");

	pr_info("S_AI=");
	for (i = (MVPP2_PRS_SRAM_AI_CTRL_BITS-1); i > -1 ; i--)
		if (mask & (1 << i)) {
			pr_info("%d", ((data & (1 << i)) != 0));
			bitsOffs += sprintf(bits + bitsOffs, "%d:", i);
		} else
			pr_info("x");
	bits[bitsOffs] = '\0';
	pr_info(" %s", bits);
	return 0;
}


static int mv_pp2x_prs_sw_dump(struct mv_pp2x_prs_entry *pe)
{
	u32 op = 0, type, lu, done = 0, flowid = 0;
	int	shift, offset, i;

	if (mv_pp2x_ptr_validate(pe))
		return -1;

	/* hw entry id */
	pr_info("[%4d] ", pe->index);

	i = MVPP2_PRS_TCAM_WORDS - 1;
	pr_info("%1.1x ", pe->tcam.word[i--] & 0xF);

	while (i >= 0)
		pr_info("%4.4x ", (pe->tcam.word[i--]) & 0xFFFF);

	pr_info("| ");

	/*DBG_MSG(PRS_SRAM_FMT, PRS_SRAM_VAL(pe->sram.word)); */
	pr_info("%4.4x %8.8x %8.8x %8.8x", pe->sram.word[3] & 0xFFFF,
		 pe->sram.word[2],  pe->sram.word[1],  pe->sram.word[0]);

	pr_info("\n       ");

	i = MVPP2_PRS_TCAM_WORDS - 1;
	pr_info("%1.1x ", (pe->tcam.word[i--] >> 16) & 0xF);

	while (i >= 0)
		pr_info("%4.4x ", ((pe->tcam.word[i--]) >> 16)  & 0xFFFF);

	pr_info("| ");

	mv_pp2x_prs_sw_sram_shift_get(pe, &shift);
	pr_info("SH=%d ", shift);

	mv_pp2x_prs_sw_sram_offset_get(pe, &type, &offset, &op);
	if (offset != 0 || ((op >> MVPP2_PRS_SRAM_OP_SEL_SHIFT_BITS) != 0))
		pr_info("UDFT=%u UDFO=%d ", type, offset);

	pr_info("op=%u ", op);

	mv_pp2x_prs_sw_sram_next_lu_get(pe, &lu);
	pr_info("LU=%u ", lu);

	mv_pp2x_prs_sw_sram_lu_done_get(pe, &done);
	pr_info("%s ", done ? "DONE" : "N_DONE");

	/*flow id generation bit*/
	mv_pp2x_prs_sw_sram_flowid_gen_get(pe, &flowid);
	pr_info("%s ", flowid ? "FIDG" : "N_FIDG");

	if ((pe->tcam.word[MVPP2_PRS_TCAM_INV_WORD] & MVPP2_PRS_TCAM_INV_MASK))
		pr_info(" [inv]");

	if (mv_pp2x_prs_sw_sram_ri_dump(pe))
		return -1;

	if (mv_pp2x_prs_sw_sram_ai_dump(pe))
		return -1;

	pr_info("\n");

	return 0;

}

static int mv_pp2x_prs_hw_tcam_cnt_dump(struct pp2_port *port,
					int tid, unsigned int *cnt)
{
	unsigned int regVal;

	if (mv_pp2x_range_validate(tid, 0,
	    MVPP2_PRS_TCAM_SRAM_SIZE - 1))
		return -1;

	/* write index */
	pp2_reg_write(port->cpu_slot, MVPP2_PRS_TCAM_HIT_IDX_REG, tid);

	regVal = pp2_reg_read(port->cpu_slot, MVPP2_PRS_TCAM_HIT_CNT_REG);
	regVal &= MVPP2_PRS_TCAM_HIT_CNT_MASK;

	if (cnt)
		*cnt = regVal;
	else
		pr_info("HIT COUNTER: %d\n", regVal);

	return 0;
}


int mv_pp2x_prs_hw_dump(struct pp2_port *port)
{
	int index;
	struct mv_pp2x_prs_entry pe;


	pr_info("%s\n", __func__);

	for (index = 0; index < MVPP2_PRS_TCAM_SRAM_SIZE; index++) {
		pe.index = index;
		mv_pp2x_prs_hw_read(port->cpu_slot, &pe);

		if ((pe.tcam.word[MVPP2_PRS_TCAM_INV_WORD] &
			MVPP2_PRS_TCAM_INV_MASK) ==
			MVPP2_PRS_TCAM_ENTRY_VALID) {
			mv_pp2x_prs_sw_dump(&pe);
			mv_pp2x_prs_hw_tcam_cnt_dump(port, index, NULL);
			pr_info("-----------------------------------------\n");
		}
	}

	return 0;
}

int mv_pp2x_prs_hw_hits_dump(struct pp2_port *port)
{
	int index;
	unsigned int cnt;
	struct mv_pp2x_prs_entry pe;

	for (index = 0; index < MVPP2_PRS_TCAM_SRAM_SIZE; index++) {
		pe.index = index;
		mv_pp2x_prs_hw_read(port->cpu_slot, &pe);
		if ((pe.tcam.word[MVPP2_PRS_TCAM_INV_WORD] &
			MVPP2_PRS_TCAM_INV_MASK) ==
			MVPP2_PRS_TCAM_ENTRY_VALID) {
			mv_pp2x_prs_hw_tcam_cnt_dump(port, index, &cnt);
			if (cnt == 0)
				continue;
			mv_pp2x_prs_sw_dump(&pe);
			pr_info("INDEX: %d       HITS: %d\n", index, cnt);
			pr_info("-----------------------------------------\n");
		}
	}
	return 0;
}

