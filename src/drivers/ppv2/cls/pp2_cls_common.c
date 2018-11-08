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

/**
 * @file pp2_cls_common.c
 *
 * routines shared by pp2 cls sub-modules
 */

/***********************/
/* c file declarations */
/***********************/
#include "std_internal.h"

#include "../pp2_types.h"
#include "../pp2.h"
#include "../pp2_hw_type.h"
#include "pp2_hw_cls.h"

#define PP2_CLS_DEBUG_MASK 1

/* Field size array */
static u32 pp2_cls_field_size_array[CLS_FIELD_MAX] = {
	MH_FIELD_SIZE,
	GEM_PORT_ID_FIELD_SIZE,
	MH_UNTAGGED_PRI_FIELD_SIZE,
	MAC_DA_FIELD_SIZE,
	MAC_SA_FIELD_SIZE,
	OUT_VLAN_PRI_FIELD_SIZE,
	OUT_VLAN_ID_FIELD_SIZE,
	IN_VLAN_ID_FIELD_SIZE,
	ETH_TYPE_FIELD_SIZE,
	PPPOE_FIELD_SIZE,
	IP_VER_FIELD_SIZE,
	IPV4_DSCP_FIELD_SIZE,
	IPV4_ECN_FIELD_SIZE,
	IPV4_LEN_FIELD_SIZE,
	IPV4_TTL_FIELD_SIZE,/*IPV6_HL_FIELD_SIZE*/
	IPV4_PROTO_FIELD_SIZE,/*IPV6_PROTO_FIELD_SIZE*/
	IPV4_SA_FIELD_SIZE,
	IPV4_DA_FIELD_SIZE,
	IPV6_DSCP_FIELD_SIZE,
	IPV6_ECN_FIELD_SIZE,
	IPV6_FLOW_LBL_FIELD_SIZE,
	IPV6_PAYLOAD_LEN_FIELD_SIZE,
	IPV6_NH_FIELD_SIZE,
	IPV6_SA_FIELD_SIZE,
	IPV6_SA_PREF_FIELD_SIZE,
	IPV6_SA_SUFF_FIELD_SIZE,
	IPV6_DA_FIELD_SIZE,
	IPV6_DA_PREF_FIELD_SIZE,
	IPV6_DA_SUFF_FIELD_SIZE,
	L4_SRC_FIELD_SIZE,
	L4_DST_FIELD_SIZE,
	TCP_FLAGS_FIELD_SIZE,
	OUT_TPID_FIELD_SIZE,
	OUT_VLAN_CFI_FIELD_SIZE,
	IN_TPID_FIELD_SIZE,
	IN_VLAN_CFI_FIELD_SIZE,
	ARP_IPV4_DA_FIELD_SIZE
};

/*******************************************************************************
 * pp2_cls_field_bm_to_field_id
 *
 * DESCRIPTION: The routine will transfer field bitmap and pkt key to field info
 *
 * INPUTS:
 *	field_bm    - match fiels bit map
 *	pp2_cls_pkt_key - matched field value
 *	field_max   - the max count of field checked
 *	l4_info     - whether count on l4_info
 *
 * OUTPUTS:
 *	field_info  - corresponding field info array
 *
 * RETURNS:
 *	0 on success, error-code otherwise
 * COMMENTS:
 *	NOTE: When update this routine, please obey the field sequence in packet.
 ******************************************************************************/
int pp2_cls_field_bm_to_field_info(u32 field_bm, struct pp2_cls_mng_pkt_key_t *pp2_cls_pkt_key,
				   u32 field_max, u8 l4_info, struct pp2_cls_field_match_info field_info[])
{
	int i = 0;
	int field_size = 0;
	u8 is_ipv4 = true;

	/* Para check */
	if (mv_pp2x_ptr_validate(pp2_cls_pkt_key) == MV_ERROR)
		return MV_ERROR;
	if (mv_pp2x_ptr_validate(field_info) == MV_ERROR)
		return MV_ERROR;

	if (field_bm & (MVPP2_MATCH_IPV6_PKT | MVPP2_MATCH_IPV6_PREF | MVPP2_MATCH_IPV6_SUFF))
		is_ipv4 = false;

	pr_debug("field_bm 0x%x is_ipv4 %d\n", field_bm, is_ipv4);

	if (field_bm & MVPP2_MATCH_MH && i < field_max) {
		field_info[i].valid = MVPP2_FIELD_VALID;
		field_info[i].field_id = MH_FIELD_ID;
		field_info[i].filed_value.int_data.parsed_int_val = pp2_cls_pkt_key->mh;
		field_size = pp2_cls_field_size_array[field_info[i].field_id];
		field_info[i].filed_value.int_data.parsed_int_val_mask = pp2_cls_pkt_key->mh_mask;
		pr_debug("field_info[%d] %s val 0x%x mask 0x%x\n", i,
			 pp2_cls_utils_field_id_str_get(field_info[i].field_id),
			 field_info[i].filed_value.int_data.parsed_int_val,
			 field_info[i].filed_value.int_data.parsed_int_val_mask);
		i++;
	}

	if (field_bm & MVPP2_MATCH_ETH_DST && i < field_max) {
		field_info[i].valid = MVPP2_FIELD_VALID;
		field_info[i].field_id = MAC_DA_FIELD_ID;
		memcpy(&field_info[i].filed_value.mac_addr.parsed_mac_addr[0],
		       &pp2_cls_pkt_key->pkt_key->eth_dst.eth_add[0],
		       MAC_ADDR_SIZE);
		memcpy(&field_info[i].filed_value.mac_addr.parsed_mac_addr_mask[0],
		       &pp2_cls_pkt_key->pkt_key->eth_dst.eth_add_mask[0],
		       MAC_ADDR_SIZE);
		i++;
	}
	if (field_bm & MVPP2_MATCH_ETH_SRC && i < field_max) {
		field_info[i].valid = MVPP2_FIELD_VALID;
		field_info[i].field_id = MAC_SA_FIELD_ID;
		memcpy(&field_info[i].filed_value.mac_addr.parsed_mac_addr[0],
		       &pp2_cls_pkt_key->pkt_key->eth_src.eth_add[0],
		       MAC_ADDR_SIZE);
		memcpy(&field_info[i].filed_value.mac_addr.parsed_mac_addr_mask[0],
		       &pp2_cls_pkt_key->pkt_key->eth_src.eth_add_mask[0],
		       MAC_ADDR_SIZE);
		i++;
	}
	if (field_bm & MVPP2_MATCH_PBITS_OUTER && i < field_max) {
		field_info[i].valid = MVPP2_FIELD_VALID;
		field_info[i].field_id = OUT_VLAN_PRI_FIELD_ID;
		field_info[i].filed_value.int_data.parsed_int_val = pp2_cls_pkt_key->pkt_key->out_pbit;
		field_size = pp2_cls_field_size_array[field_info[i].field_id];
		field_info[i].filed_value.int_data.parsed_int_val_mask = common_mask_gen(field_size);
		pr_debug("field_info[%d] %s val 0x%x mask 0x%x\n", i,
			 pp2_cls_utils_field_id_str_get(field_info[i].field_id),
			 field_info[i].filed_value.int_data.parsed_int_val,
			 field_info[i].filed_value.int_data.parsed_int_val_mask);
		i++;
	}
	if (field_bm & MVPP2_MATCH_VID_OUTER && i < field_max) {
		field_info[i].valid = MVPP2_FIELD_VALID;
		field_info[i].field_id = OUT_VLAN_ID_FIELD_ID;
		field_info[i].filed_value.int_data.parsed_int_val = pp2_cls_pkt_key->pkt_key->out_vid;
		field_size = pp2_cls_field_size_array[field_info[i].field_id];
		field_info[i].filed_value.int_data.parsed_int_val_mask = common_mask_gen(field_size);
		pr_debug("field_info[%d] %s val 0x%x mask 0x%x\n", i,
			 pp2_cls_utils_field_id_str_get(field_info[i].field_id),
			 field_info[i].filed_value.int_data.parsed_int_val,
			 field_info[i].filed_value.int_data.parsed_int_val_mask);
		i++;
	}
	if (field_bm & MVPP2_MATCH_PBITS_INNER && i < field_max) {
		field_info[i].valid = MVPP2_FIELD_VALID;
		field_info[i].field_id = IN_VLAN_PRI_FIELD_ID;
		field_info[i].filed_value.int_data.parsed_int_val = pp2_cls_pkt_key->pkt_key->inn_pbit;
		field_size = pp2_cls_field_size_get(field_info[i].field_id);
		field_info[i].filed_value.int_data.parsed_int_val_mask = common_mask_gen(field_size);
		pr_debug("field_info[%d] %s val 0x%x mask 0x%x\n", i,
			 pp2_cls_utils_field_id_str_get(field_info[i].field_id),
			 field_info[i].filed_value.int_data.parsed_int_val,
			 field_info[i].filed_value.int_data.parsed_int_val_mask);
		i++;
	}
	if (field_bm & MVPP2_MATCH_VID_INNER && i < field_max) {
		field_info[i].valid = MVPP2_FIELD_VALID;
		field_info[i].field_id = IN_VLAN_ID_FIELD_ID;
		field_info[i].filed_value.int_data.parsed_int_val = pp2_cls_pkt_key->pkt_key->inn_vid;
		field_size = pp2_cls_field_size_array[field_info[i].field_id];
		field_info[i].filed_value.int_data.parsed_int_val_mask = common_mask_gen(field_size);
		pr_debug("field_info[%d] %s val 0x%x mask 0x%x\n", i,
			 pp2_cls_utils_field_id_str_get(field_info[i].field_id),
			 field_info[i].filed_value.int_data.parsed_int_val,
			 field_info[i].filed_value.int_data.parsed_int_val_mask);
		i++;
	}

	if (field_bm & MVPP2_MATCH_ETH_TYPE && i < field_max) {
		field_info[i].valid = MVPP2_FIELD_VALID;
		field_info[i].field_id = ETH_TYPE_FIELD_ID;
		field_info[i].filed_value.int_data.parsed_int_val = pp2_cls_pkt_key->pkt_key->ether_type;
		field_size = pp2_cls_field_size_array[field_info[i].field_id];
		field_info[i].filed_value.int_data.parsed_int_val_mask = common_mask_gen(field_size);
		pr_debug("field_info[%d] %s val 0x%x mask 0x%x\n", i,
			 pp2_cls_utils_field_id_str_get(field_info[i].field_id),
			 field_info[i].filed_value.int_data.parsed_int_val,
			 field_info[i].filed_value.int_data.parsed_int_val_mask);
		i++;
	}

	if (field_bm & MVPP2_MATCH_PPPOE_SES && i < field_max) {
		field_info[i].valid = MVPP2_FIELD_VALID;
		field_info[i].field_id = PPPOE_FIELD_ID;
		field_info[i].filed_value.int_data.parsed_int_val = pp2_cls_pkt_key->pkt_key->ppp_info.ppp_session;
		field_size = pp2_cls_field_size_array[field_info[i].field_id];
		field_info[i].filed_value.int_data.parsed_int_val_mask = common_mask_gen(field_size);
		pr_debug("field_info[%d] %s val 0x%x mask 0x%x\n", i,
			 pp2_cls_utils_field_id_str_get(field_info[i].field_id),
			 field_info[i].filed_value.int_data.parsed_int_val,
			 field_info[i].filed_value.int_data.parsed_int_val_mask);
		i++;
	}

	if (field_bm & MVPP2_MATCH_PPPOE_PROTO && i < field_max) {
		field_info[i].valid = MVPP2_FIELD_VALID;
		field_info[i].field_id = PPPOE_PROTO_ID;
		field_info[i].filed_value.int_data.parsed_int_val = pp2_cls_pkt_key->pkt_key->ppp_info.ppp_proto;
		field_size = pp2_cls_field_size_get(field_info[i].field_id);
		field_info[i].filed_value.int_data.parsed_int_val_mask = common_mask_gen(field_size);
		pr_debug("field_info[%d] %s val 0x%x mask 0x%x\n", i,
			 pp2_cls_utils_field_id_str_get(field_info[i].field_id),
			 field_info[i].filed_value.int_data.parsed_int_val,
			 field_info[i].filed_value.int_data.parsed_int_val_mask);
		i++;
	}
	if ((field_bm & (MVPP2_MATCH_IP_VERSION)) && i < field_max) {
		field_info[i].valid = MVPP2_FIELD_VALID;
		field_info[i].field_id = IP_VER_FIELD_ID;
		field_info[i].filed_value.int_data.parsed_int_val = pp2_cls_pkt_key->pkt_key->ipvx_add.ip_ver;
		field_size = pp2_cls_field_size_array[field_info[i].field_id];
		field_info[i].filed_value.int_data.parsed_int_val_mask = common_mask_gen(field_size);
		pr_debug("field_info[%d] %s val 0x%x mask 0x%x\n", i,
			 pp2_cls_utils_field_id_str_get(field_info[i].field_id),
			 field_info[i].filed_value.int_data.parsed_int_val,
			 field_info[i].filed_value.int_data.parsed_int_val_mask);
		i++;
	}
	if (field_bm & MVPP2_MATCH_IP_DSCP && i < field_max) {
		if (is_ipv4) {
			field_info[i].valid = MVPP2_FIELD_VALID;
			field_info[i].field_id = IPV4_DSCP_FIELD_ID;
			field_info[i].filed_value.int_data.parsed_int_val = pp2_cls_pkt_key->pkt_key->ipvx_add.dscp;
			field_info[i].filed_value.int_data.parsed_int_val_mask =
				pp2_cls_pkt_key->pkt_key->ipvx_add.dscp_mask;
		} else {
			field_info[i].valid = MVPP2_FIELD_VALID;
			field_info[i].field_id = IPV6_DSCP_FIELD_ID;
			field_info[i].filed_value.int_data.parsed_int_val = pp2_cls_pkt_key->pkt_key->ipvx_add.dscp;
			field_info[i].filed_value.int_data.parsed_int_val_mask =
				pp2_cls_pkt_key->pkt_key->ipvx_add.dscp_mask;
		}
		pr_debug("field_info[%d] %s val 0x%x mask 0x%x\n", i,
			 pp2_cls_utils_field_id_str_get(field_info[i].field_id),
			 field_info[i].filed_value.int_data.parsed_int_val,
			 field_info[i].filed_value.int_data.parsed_int_val_mask);
		i++;
	}
	if (field_bm & MVPP2_MATCH_IPV6_FLBL && i < field_max) {
		field_info[i].valid = MVPP2_FIELD_VALID;
		field_info[i].field_id = IPV6_FLOW_LBL_FIELD_ID;
		field_info[i].filed_value.int_data.parsed_int_val = pp2_cls_pkt_key->pkt_key->ipvx_add.flow_label;
		field_info[i].filed_value.int_data.parsed_int_val_mask =
			pp2_cls_pkt_key->pkt_key->ipvx_add.flow_label_mask;
		pr_debug("field_info[%d] %s val 0x%x mask 0x%x\n", i,
			 pp2_cls_utils_field_id_str_get(field_info[i].field_id),
			 field_info[i].filed_value.int_data.parsed_int_val,
			 field_info[i].filed_value.int_data.parsed_int_val_mask);
		i++;
	}
	if (field_bm & MVPP2_MATCH_TTL && i < field_max) {
		field_info[i].valid = MVPP2_FIELD_VALID;
		field_info[i].field_id = IPV4_TTL_FIELD_ID;
		field_info[i].filed_value.int_data.parsed_int_val = pp2_cls_pkt_key->ttl;
		field_size = pp2_cls_field_size_array[field_info[i].field_id];
		field_info[i].filed_value.int_data.parsed_int_val_mask = common_mask_gen(field_size);
		pr_debug("field_info[%d] %s val 0x%x mask 0x%x\n", i,
			 pp2_cls_utils_field_id_str_get(field_info[i].field_id),
			 field_info[i].filed_value.int_data.parsed_int_val,
			 field_info[i].filed_value.int_data.parsed_int_val_mask);
		i++;
	}
	if (l4_info && field_bm & MVPP2_MATCH_IP_PROTO && i < field_max) {
		if (is_ipv4) {
			field_info[i].valid = MVPP2_FIELD_VALID;
			field_info[i].field_id = IPV4_PROTO_FIELD_ID;
			field_info[i].filed_value.int_data.parsed_int_val = pp2_cls_pkt_key->pkt_key->ipvx_add.ip_proto;
			field_size = pp2_cls_field_size_array[field_info[i].field_id];
			field_info[i].filed_value.int_data.parsed_int_val_mask = common_mask_gen(field_size);
		} else {
			field_info[i].valid = MVPP2_FIELD_VALID;
			field_info[i].field_id = IPV6_PROTO_FIELD_ID;
			field_info[i].filed_value.int_data.parsed_int_val = pp2_cls_pkt_key->pkt_key->ipvx_add.ip_proto;
			field_size = pp2_cls_field_size_array[field_info[i].field_id];
			field_info[i].filed_value.int_data.parsed_int_val_mask = common_mask_gen(field_size);
		}
		pr_debug("field_info[%d] %s val 0x%x mask 0x%x\n", i,
			 pp2_cls_utils_field_id_str_get(field_info[i].field_id),
			 field_info[i].filed_value.int_data.parsed_int_val,
			 field_info[i].filed_value.int_data.parsed_int_val_mask);
		i++;
	}
	if (field_bm & MVPP2_MATCH_IP_SRC && i < field_max) {
		if (is_ipv4) {
			field_info[i].valid = MVPP2_FIELD_VALID;
			field_info[i].field_id = IPV4_SA_FIELD_ID;
			memcpy(&field_info[i].filed_value.ipv4_addr.parsed_ipv4_addr[0],
			       &pp2_cls_pkt_key->pkt_key->ipvx_add.ip_src.ip_add.ipv4[0],
			       IPV4_ADDR_SIZE);
			memcpy(&field_info[i].filed_value.ipv4_addr.parsed_ipv4_addr_mask[0],
			       &pp2_cls_pkt_key->pkt_key->ipvx_add.ip_src.ip_add_mask.ipv4[0],
			       IPV4_ADDR_SIZE);
			pr_debug("field_info[%d] %s val %d.%d.%d.%d mask %d.%d.%d.%d\n", i,
				 pp2_cls_utils_field_id_str_get(field_info[i].field_id),
				 field_info[i].filed_value.ipv4_addr.parsed_ipv4_addr[0],
				 field_info[i].filed_value.ipv4_addr.parsed_ipv4_addr[1],
				 field_info[i].filed_value.ipv4_addr.parsed_ipv4_addr[2],
				 field_info[i].filed_value.ipv4_addr.parsed_ipv4_addr[3],
				 field_info[i].filed_value.ipv4_addr.parsed_ipv4_addr_mask[0],
				 field_info[i].filed_value.ipv4_addr.parsed_ipv4_addr_mask[1],
				 field_info[i].filed_value.ipv4_addr.parsed_ipv4_addr_mask[2],
				 field_info[i].filed_value.ipv4_addr.parsed_ipv4_addr_mask[3]);
			i++;
		} else {
			if (field_bm & MVPP2_MATCH_IPV6_PREF) {
				field_info[i].field_id = IPV6_SA_PREF_FIELD_ID;
			} else if (field_bm & MVPP2_MATCH_IPV6_SUFF) {
				field_info[i].field_id = IPV6_SA_SUFF_FIELD_ID;
			} else if (field_bm & MVPP2_MATCH_IPV6_PKT) {
				field_info[i].field_id = IPV6_SA_FIELD_ID;
			} else {
				pr_debug("field_bm %s must include %s or %s\n",
					 pp2_cls_utils_field_match_str_get(MVPP2_MATCH_IPV6_PREF),
					 pp2_cls_utils_field_match_str_get(MVPP2_MATCH_IP_SRC),
					 pp2_cls_utils_field_match_str_get(MVPP2_MATCH_IP_DST));
			}
			field_info[i].valid = MVPP2_FIELD_VALID;
			memcpy(&field_info[i].filed_value.ipv6_addr.parsed_ipv6_addr[0],
			       &pp2_cls_pkt_key->pkt_key->ipvx_add.ip_src.ip_add.ipv6[0],
			       IPV6_ADDR_SIZE);
			memcpy(&field_info[i].filed_value.ipv6_addr.parsed_ipv6_addr_mask[0],
			       &pp2_cls_pkt_key->pkt_key->ipvx_add.ip_src.ip_add_mask.ipv6[0],
			       IPV6_ADDR_SIZE);
			i++;
		}
	}
	if (field_bm & MVPP2_MATCH_IP_DST && i < field_max) {
		if (is_ipv4) {
			field_info[i].valid = MVPP2_FIELD_VALID;
			field_info[i].field_id = IPV4_DA_FIELD_ID;
			memcpy(&field_info[i].filed_value.ipv4_addr.parsed_ipv4_addr[0],
			       &pp2_cls_pkt_key->pkt_key->ipvx_add.ip_dst.ip_add.ipv4[0],
			       IPV4_ADDR_SIZE);
			memcpy(&field_info[i].filed_value.ipv4_addr.parsed_ipv4_addr_mask[0],
			       &pp2_cls_pkt_key->pkt_key->ipvx_add.ip_dst.ip_add_mask.ipv4[0],
			       IPV4_ADDR_SIZE);
			pr_debug("field_info[%d] %s val %d.%d.%d.%d mask %d.%d.%d.%d\n", i,
				 pp2_cls_utils_field_id_str_get(field_info[i].field_id),
				 field_info[i].filed_value.ipv4_addr.parsed_ipv4_addr[0],
				 field_info[i].filed_value.ipv4_addr.parsed_ipv4_addr[1],
				 field_info[i].filed_value.ipv4_addr.parsed_ipv4_addr[2],
				 field_info[i].filed_value.ipv4_addr.parsed_ipv4_addr[3],
				 field_info[i].filed_value.ipv4_addr.parsed_ipv4_addr_mask[0],
				 field_info[i].filed_value.ipv4_addr.parsed_ipv4_addr_mask[1],
				 field_info[i].filed_value.ipv4_addr.parsed_ipv4_addr_mask[2],
				 field_info[i].filed_value.ipv4_addr.parsed_ipv4_addr_mask[3]);
			i++;
		} else {
			if (field_bm & MVPP2_MATCH_IPV6_PREF) {
				field_info[i].field_id = IPV6_DA_PREF_FIELD_ID;
			} else if (field_bm & MVPP2_MATCH_IPV6_SUFF) {
				field_info[i].field_id = IPV6_DA_SUFF_FIELD_ID;
			} else if (field_bm & MVPP2_MATCH_IPV6_PKT) {
				field_info[i].field_id = IPV6_DA_FIELD_ID;
			} else {
				pr_debug("field_bm %s must include %s or %s\n",
					 pp2_cls_utils_field_match_str_get(MVPP2_MATCH_IPV6_PREF),
					 pp2_cls_utils_field_match_str_get(MVPP2_MATCH_IP_SRC),
					 pp2_cls_utils_field_match_str_get(MVPP2_MATCH_IP_DST));
			}
			field_info[i].valid = MVPP2_FIELD_VALID;
			memcpy(&field_info[i].filed_value.ipv6_addr.parsed_ipv6_addr[0],
			       &pp2_cls_pkt_key->pkt_key->ipvx_add.ip_dst.ip_add.ipv6[0],
			       IPV6_ADDR_SIZE);
			memcpy(&field_info[i].filed_value.ipv6_addr.parsed_ipv6_addr_mask[0],
			       &pp2_cls_pkt_key->pkt_key->ipvx_add.ip_dst.ip_add_mask.ipv6[0],
			       IPV6_ADDR_SIZE);
			i++;
		}
	}
	if (field_bm & MVPP2_MATCH_L4_SRC && i < field_max) {
		field_info[i].valid = MVPP2_FIELD_VALID;
		field_info[i].field_id = L4_SRC_FIELD_ID;
		field_info[i].filed_value.int_data.parsed_int_val = pp2_cls_pkt_key->pkt_key->l4_src;
		field_size = pp2_cls_field_size_array[field_info[i].field_id];
		field_info[i].filed_value.int_data.parsed_int_val_mask = common_mask_gen(field_size);
		pr_debug("field_info[%d] %s val %d mask 0x%x\n", i,
			 pp2_cls_utils_field_id_str_get(field_info[i].field_id),
			 field_info[i].filed_value.int_data.parsed_int_val,
			 field_info[i].filed_value.int_data.parsed_int_val_mask);
		i++;
	}
	if (field_bm & MVPP2_MATCH_L4_DST && i < field_max) {
		field_info[i].valid = MVPP2_FIELD_VALID;
		field_info[i].field_id = L4_DST_FIELD_ID;
		field_info[i].filed_value.int_data.parsed_int_val = pp2_cls_pkt_key->pkt_key->l4_dst;
		field_size = pp2_cls_field_size_array[field_info[i].field_id];
		field_info[i].filed_value.int_data.parsed_int_val_mask = common_mask_gen(field_size);
		pr_debug("field_info[%d] %s val %d mask 0x%x\n", i,
			 pp2_cls_utils_field_id_str_get(field_info[i].field_id),
			 field_info[i].filed_value.int_data.parsed_int_val,
			 field_info[i].filed_value.int_data.parsed_int_val_mask);
		i++;
	}
	if ((field_bm & MVPP2_MATCH_TCP_FLAG_RF || field_bm & MVPP2_MATCH_TCP_FLAG_S) && i < field_max) {
		field_info[i].valid = MVPP2_FIELD_VALID;
		field_info[i].field_id = TCP_FLAGS_FIELD_ID;
		field_info[i].filed_value.int_data.parsed_int_val = pp2_cls_pkt_key->tcp_flag;
		field_info[i].filed_value.int_data.parsed_int_val_mask = pp2_cls_pkt_key->tcp_flag_mask;
		pr_debug("field_info[%d] %s val 0x%x mask 0x%x\n", i,
			 pp2_cls_utils_field_id_str_get(field_info[i].field_id),
			 field_info[i].filed_value.int_data.parsed_int_val,
			 field_info[i].filed_value.int_data.parsed_int_val_mask);
		i++;
	}

	if (field_bm & MVPP2_MATCH_ARP_TRGT_IP_ADDR && i < field_max) {
		field_info[i].valid = MVPP2_FIELD_VALID;
		field_info[i].field_id = ARP_IPV4_DA_FIELD_ID;
		memcpy(&field_info[i].filed_value.ipv4_addr.parsed_ipv4_addr[0],
		       &pp2_cls_pkt_key->pkt_key->arp_ip_dst.ip_add.ipv4[0],
		       IPV4_ADDR_SIZE);
		memcpy(&field_info[i].filed_value.ipv4_addr.parsed_ipv4_addr_mask[0],
		       &pp2_cls_pkt_key->pkt_key->arp_ip_dst.ip_add_mask.ipv4[0],
		       IPV4_ADDR_SIZE);
		i++;
	}

	/*add udf set fields - tpid and cfi*/
	if (field_bm & MVPP2_MATCH_TPID_OUTER && i < field_max) {
		field_info[i].valid = MVPP2_FIELD_VALID;
		field_info[i].field_id = OUT_TPID_FIELD_ID;
		field_info[i].filed_value.int_data.parsed_int_val = pp2_cls_pkt_key->pkt_key->out_tpid;
		field_size = pp2_cls_field_size_array[field_info[i].field_id];
		field_info[i].filed_value.int_data.parsed_int_val_mask = common_mask_gen(field_size);
		pr_debug("field_info[%d] %s val 0x%x mask 0x%x\n", i,
			 pp2_cls_utils_field_id_str_get(field_info[i].field_id),
			 field_info[i].filed_value.int_data.parsed_int_val,
			 field_info[i].filed_value.int_data.parsed_int_val_mask);
		i++;
	}
	if (field_bm & MVPP2_MATCH_CFI_OUTER && i < field_max) {
		field_info[i].valid = MVPP2_FIELD_VALID;
		field_info[i].field_id = OUT_VLAN_CFI_FIELD_ID;
		field_info[i].filed_value.int_data.parsed_int_val = pp2_cls_pkt_key->pkt_key->out_cfi;
		field_size = pp2_cls_field_size_array[field_info[i].field_id];
		field_info[i].filed_value.int_data.parsed_int_val_mask = common_mask_gen(field_size);
		pr_debug("field_info[%d] %s val 0x%x mask 0x%x\n", i,
			 pp2_cls_utils_field_id_str_get(field_info[i].field_id),
			 field_info[i].filed_value.int_data.parsed_int_val,
			 field_info[i].filed_value.int_data.parsed_int_val_mask);
		i++;
	}
	if (field_bm & MVPP2_MATCH_TPID_INNER && i < field_max) {
		field_info[i].valid = MVPP2_FIELD_VALID;
		field_info[i].field_id = IN_TPID_FIELD_ID;
		field_info[i].filed_value.int_data.parsed_int_val = pp2_cls_pkt_key->pkt_key->inn_tpid;
		field_size = pp2_cls_field_size_array[field_info[i].field_id];
		field_info[i].filed_value.int_data.parsed_int_val_mask = common_mask_gen(field_size);
		pr_debug("field_info[%d] %s val 0x%x mask 0x%x\n", i,
			 pp2_cls_utils_field_id_str_get(field_info[i].field_id),
			 field_info[i].filed_value.int_data.parsed_int_val,
			 field_info[i].filed_value.int_data.parsed_int_val_mask);
		i++;
	}
	if (field_bm & MVPP2_MATCH_CFI_INNER && i < field_max) {
		field_info[i].valid = MVPP2_FIELD_VALID;
		field_info[i].field_id = IN_VLAN_CFI_FIELD_ID;
		field_info[i].filed_value.int_data.parsed_int_val = pp2_cls_pkt_key->pkt_key->inn_cfi;
		field_size = pp2_cls_field_size_array[field_info[i].field_id];
		field_info[i].filed_value.int_data.parsed_int_val_mask = common_mask_gen(field_size);
		pr_debug("field_info[%d] %s val 0x%x mask 0x%x\n", i,
			 pp2_cls_utils_field_id_str_get(field_info[i].field_id),
			 field_info[i].filed_value.int_data.parsed_int_val,
			 field_info[i].filed_value.int_data.parsed_int_val_mask);
		i++;
	}

	return 0;
}

/*******************************************************************************
 * pp2_cls_field_size_get
 *
 * DESCRIPTION: The routine will get the size(unit: bit) of expected field
 *
 * INPUTS:
 *           field_id    - field ID
 *
 * OUTPUTS:
 *           None.
 *
 * RETURNS:
 *           The size of the packet field.
 *
 * COMMENTS:
 *           None.
 ******************************************************************************/
u32 pp2_cls_field_size_get(u32 field_id)
{
	if (field_id == ARP_IPV4_DA_FIELD_ID)
		return ARP_IPV4_DA_FIELD_SIZE;

	if (field_id == IN_VLAN_PRI_FIELD_ID)
		return IN_VLAN_PRI_FIELD_SIZE;

	if (field_id == PPPOE_PROTO_ID)
		return PPPOE_PROTO_SIZE;

	if (field_id >= CLS_FIELD_MAX)
		return 0;

	return pp2_cls_field_size_array[field_id];
}
