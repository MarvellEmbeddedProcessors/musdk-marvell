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
 * @file pp2_c3.c
 *
 * C3 High level routines
 */

/***********************/
/* c file declarations */
/***********************/
#include "std_internal.h"

#include "../pp2_types.h"
#include "../pp2.h"
#include "../pp2_hw_type.h"
#include "pp2_hw_cls.h"
#include "pp2_hw_cls_dbg.h"

/* #define PP2_CLS_C3_DEBUG */

/**
 * pp2_cls_c3_common_field_hek_get
 *
 * The routine will transfer packet key with common field whose field size should
 *  not be greater than 32 bits to C3 HEK
 *
 * @param[in]	pkt_value	field value
 * @param[in]	field_bytes	bytes the field occupied
 * @param[in]	field_size		field size in unit of bits
 *
 * @param[out]	c3_hek		HEK for C3
 * @param[out]	bytes_used	pointer to record HEK bytes has been used
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
static int pp2_cls_c3_common_field_hek_get(u32 pkt_value, u32 field_bytes, u32 field_size, u8 c3_hek[],
					   u32 *bytes_used)
{
	int idx;
	u32 c3_hek_bytes_used;

	pr_debug("reached\n");

	/* NULL validation */
	if (mv_pp2x_ptr_validate(c3_hek))
		return -EINVAL;

	if (mv_pp2x_ptr_validate(bytes_used))
		return -EINVAL;

	if (field_size == 0 || field_bytes == 0)
		return -EINVAL;

	/* parse packet key */
	c3_hek_bytes_used = *bytes_used;
	for (idx = 0; idx < field_bytes; idx++) {
		if (field_size % BYTE_BITS) {
			if (idx < (field_bytes - 1)) {
				/* HEK value */
				c3_hek[c3_hek_bytes_used] =
				((pkt_value  >> (BYTE_BITS * (field_bytes - 2 - idx) + field_size % BYTE_BITS)) &
				 BYTE_MASK);
			} else {
				/* HEK value */
				c3_hek[c3_hek_bytes_used] =
				((pkt_value << (BYTE_BITS - field_size % BYTE_BITS)) & BYTE_MASK);
			}
		} else {
			/* HEK value */
			c3_hek[c3_hek_bytes_used] =
			((pkt_value >> (BYTE_BITS * (field_bytes - 1 - idx))) & BYTE_MASK);
		}
		/* increase HEK byte count */
		c3_hek_bytes_used++;
	}
	/* update bytes_used */
	*bytes_used = c3_hek_bytes_used;

	return 0;
}

/**
 * pp2_cls_c3_shared_field_hek_get
 *
 * The routine will transfer packet key with common field who shares one byte with other field to C3 HEK
 *
 * @param[in]	pkt_value	field value
 * @param[in]	field_bytes	bytes the field occupied
 * @param[in]	field_size		field size in unit of bits
 * @param[in]	comb_flag	indicate combination is needed or not
 * @param[in]	comb_offset	combination bit offset
 * @param[in]	bytes_used	pointer to HEK bytes has been used
 *
 * @param[out]	c3_hek		HEK for C3
 * @param[out]	bytes_used	pointer to record HEK bytes has been used
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
static int pp2_cls_c3_shared_field_hek_get(u32 pkt_value, u32 field_bytes, u32 field_size, u8 comb_flag,
					   u8 comb_offset, u8 c3_hek[], u32 *bytes_used)
{
	int idx;
	u32 left_bits;
	u32 c3_hek_bytes_used;
	u8 comb_flag1;
	u8 comb_flag2;

	pr_debug("reached\n");

	/* Para check */
	if (mv_pp2x_ptr_validate(c3_hek))
		return -EINVAL;

	if (mv_pp2x_ptr_validate(bytes_used))
		return -EINVAL;

	if (field_size == 0 || field_bytes == 0)
		return MV_ERROR;

	left_bits = field_size;
	c3_hek_bytes_used = *bytes_used;
	comb_flag1 = comb_flag;
	comb_flag2 = comb_flag;
	for (idx = 0; idx < field_bytes; idx++) {
		if (comb_flag2) {
			if (comb_flag1) {
				c3_hek_bytes_used--;
				/* HEK value */
				c3_hek[c3_hek_bytes_used] |=
				((pkt_value >> (field_size - comb_offset)) & BYTE_MASK);
				if (((field_size % BYTE_BITS) + comb_offset) > BYTE_BITS ||
				    (field_size > BYTE_BITS)) {
					pkt_value &= common_mask_gen(field_size - comb_offset);
				}
				c3_hek_bytes_used++;
				left_bits = field_size - comb_offset;
				comb_flag1 = false;
			} else {
				if (left_bits % BYTE_BITS) {
					if (idx < (field_bytes - 1)) {
						/* HEK value */
						c3_hek[c3_hek_bytes_used] =
						(pkt_value  >>
						 (BYTE_BITS * (field_bytes - 2 - idx) + left_bits % BYTE_BITS)) &
						BYTE_MASK;
					} else {
						/* HEK value */
						c3_hek[c3_hek_bytes_used] =
						(pkt_value << (BYTE_BITS - left_bits % BYTE_BITS)) & BYTE_MASK;
					}
				} else {
					/* HEK value */
					c3_hek[c3_hek_bytes_used] =
					(pkt_value >> (BYTE_BITS * (field_bytes - 1 - idx))) & BYTE_MASK;
				}
				c3_hek_bytes_used++;
				comb_flag2 = false;
			}
		} else {
			/* HEK Value */
			c3_hek[c3_hek_bytes_used] =
			(pkt_value >> (BYTE_BITS * (field_bytes - 1 - idx))) & BYTE_MASK;
			c3_hek_bytes_used++;
		}
	}
	*bytes_used = c3_hek_bytes_used;

	return 0;
}

/**
 * pp2_cls_c3_hek_generate
 *
 * The routine will generate C3 HEK by management key
 *
 * @param[in]	c3_entry		C3 entry para to get packet key info
 *
 * @param[out]	c3_hek		HEK for C3
 * @param[out]	size		size of HEK in unit of bytes
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
static int pp2_cls_c3_hek_generate(struct pp2_cls_c3_add_entry_t *c3_entry, u32 *size, u8 hek[])
{
	struct pp2_cls_field_match_info field_info[MVPP2_FLOW_FIELD_COUNT_MAX];
	u8 c3_hek[MVPP2_C3_MAX_HASH_KEY_SIZE];
	u32 field_bytes;
	u32 field_id;
	u32 field_size;
	u32 pkt_value = 0;
	u32 c3_hek_bytes_used = 0;/* used to recoed current bytes filled in HEK */
	int field_num;
	int idx;
	u32 pre_field_id = 0;
	u8 comb_flag = false;
	u8 comb_offset = 0;
	int rc = MV_OK;
	u8 l4_info;

	pr_debug("reached\n");

	if (mv_pp2x_ptr_validate(c3_entry))
		return -EINVAL;

	if (mv_pp2x_ptr_validate(size))
		return -EINVAL;

	if (mv_pp2x_ptr_validate(hek))
		return -EINVAL;

	/* clear related structure */
	memset(&field_info[0], 0, sizeof(struct pp2_cls_field_match_info) * MVPP2_FLOW_FIELD_COUNT_MAX);
	memset(c3_hek, 0, MVPP2_C3_MAX_HASH_KEY_SIZE);

	if (c3_entry->mng_pkt_key->pkt_key->field_bm == MVPP2_MATCH_IPV4_5T ||
	    c3_entry->mng_pkt_key->pkt_key->field_bm == MVPP2_MATCH_IPV6_5T)
		l4_info = false;
	else
		l4_info = true;

	/* get field info */
	rc = pp2_cls_field_bm_to_field_info(c3_entry->mng_pkt_key->pkt_key->field_bm, c3_entry->mng_pkt_key,
					    MVPP2_FLOW_FIELD_COUNT_MAX,	l4_info, field_info);

	if (rc) {
		pr_err("failed to get field information\n");
		return rc;
	}

	/* Set C3 TCAM HEK */
	field_num = 0;
	while (field_num < MVPP2_FLOW_FIELD_COUNT_MAX && field_info[field_num].valid == MVPP2_FIELD_VALID) {
		field_id = field_info[field_num].field_id;
		field_size = pp2_cls_field_size_get(field_id);
		if (field_size % BYTE_BITS)
			field_bytes = (field_size / BYTE_BITS) + 1;
		else
			field_bytes = field_size / BYTE_BITS;
		/* Check HEK bytes number */
		if (c3_hek_bytes_used >= MVPP2_C3_MAX_HASH_KEY_SIZE ||
		    (field_bytes > (MVPP2_C3_MAX_HASH_KEY_SIZE - c3_hek_bytes_used))) {
			pr_err("HEK bytes (%d) beyond C3 capcity\n", (c3_hek_bytes_used + field_bytes));
			return -EINVAL;
		}
		/* Organize pkt key according to field size and order */
		switch (field_id) {
		case MH_FIELD_ID:
		case MH_UNTAGGED_PRI_FIELD_ID:
		case OUT_VLAN_PRI_FIELD_ID:
		case ETH_TYPE_FIELD_ID:
		case PPPOE_FIELD_ID:
		case IP_VER_FIELD_ID:
		case IPV4_DSCP_FIELD_ID:
		case IPV4_LEN_FIELD_ID:
		case IPV4_TTL_FIELD_ID:
		case IPV4_PROTO_FIELD_ID:
		case IPV6_PAYLOAD_LEN_FIELD_ID:
		case IPV6_NH_FIELD_ID:
		case L4_SRC_FIELD_ID:
		case L4_DST_FIELD_ID:
		case TCP_FLAGS_FIELD_ID:
		case IN_VLAN_PRI_FIELD_ID:
		case PPPOE_PROTO_ID:
		case OUT_TPID_FIELD_ID:
		case IN_TPID_FIELD_ID:
			/* Get HEK data */
			pkt_value = field_info[field_num].filed_value.int_data.parsed_int_val;
			/* Store HEK in c3_hek, each filed byte boutary */
			rc = pp2_cls_c3_common_field_hek_get(pkt_value, field_bytes, field_size, c3_hek,
							     &c3_hek_bytes_used);
			if (rc) {
				pr_err("failed to get HEK\n");
				return rc;
			}

			break;
		case OUT_VLAN_CFI_FIELD_ID:
		case IN_VLAN_CFI_FIELD_ID:
			c3_hek_bytes_used++;
			if (c3_hek_bytes_used > MVPP2_C3_MAX_HASH_KEY_SIZE) {
				pr_err("HEK bytes (%d) out C3 capcity\n", c3_hek_bytes_used);
				/*mvOsFree(field_info);*/ /* [AW] TBD */
				return -EINVAL;
			}
			c3_hek[c3_hek_bytes_used - 1] =
				(pkt_value << (BYTE_BITS - 1 - (MVPP2_CFI_OFFSET_BITS % BYTE_BITS)));
			break;
		/* Share bits combination */
		case GEM_PORT_ID_FIELD_ID:
		case IN_VLAN_ID_FIELD_ID:
		case OUT_VLAN_ID_FIELD_ID:
			if (pre_field_id == OUT_VLAN_PRI_FIELD_ID && field_id == OUT_VLAN_ID_FIELD_ID) {
				comb_flag = true;
				comb_offset = 4;
			}
			if (pre_field_id == IN_VLAN_PRI_FIELD_ID && field_id == IN_VLAN_ID_FIELD_ID) {
				comb_flag = true;
				comb_offset = 4;
			}
		/* fallthru */
		case IPV4_ECN_FIELD_ID:
			if (pre_field_id == IPV4_DSCP_FIELD_ID && field_id == IPV4_ECN_FIELD_ID) {
				comb_flag = true;
				comb_offset = 2;
			}
		/* fallthru */
		case IPV6_DSCP_FIELD_ID:
			if (pre_field_id == IP_VER_FIELD_ID || field_id == IPV6_DSCP_FIELD_ID) {
				comb_flag = true;
				comb_offset = 4;
				if (pre_field_id != IP_VER_FIELD_ID)
					c3_hek_bytes_used++;
			}
		/* fallthru */
		case IPV6_ECN_FIELD_ID:
			if (pre_field_id == IPV6_DSCP_FIELD_ID && field_id == IPV6_ECN_FIELD_ID) {
				comb_flag = true;
				comb_offset = 2;
			}
		/* fallthru */
		case IPV6_FLOW_LBL_FIELD_ID:
			if (field_id == IPV6_FLOW_LBL_FIELD_ID &&
			    (pre_field_id == IPV6_DSCP_FIELD_ID || pre_field_id == IPV6_ECN_FIELD_ID)) {
				comb_flag = true;
				comb_offset = 4;
			}

			/* Get HEK data */
			pkt_value = field_info[field_num].filed_value.int_data.parsed_int_val;
			/* Check Combination */
			if (comb_flag && (field_size < BYTE_BITS) && ((field_size + comb_offset) > BYTE_BITS))
				field_bytes++;

			if (c3_hek_bytes_used >= MVPP2_C3_MAX_HASH_KEY_SIZE ||
			    (field_bytes > (MVPP2_C3_MAX_HASH_KEY_SIZE - c3_hek_bytes_used))) {
				pr_err("HEK bytes (%d) beyond C3 capcity\n", (c3_hek_bytes_used + field_bytes));
				return -EINVAL;
			}

			rc = pp2_cls_c3_shared_field_hek_get(pkt_value, field_bytes, field_size, comb_flag,
							     comb_offset, c3_hek, &c3_hek_bytes_used);
			if (rc) {
				pr_err("failed to get HEK\n");
				return rc;
			}
			break;

		case MAC_DA_FIELD_ID:
		case MAC_SA_FIELD_ID:
		case IPV4_SA_FIELD_ID:
		case IPV4_DA_FIELD_ID:
		case ARP_IPV4_DA_FIELD_ID:
			for (idx = 0; idx < field_bytes; idx++) {
				if (field_id == MAC_DA_FIELD_ID || field_id == MAC_SA_FIELD_ID) {
					/* HEK value */
					c3_hek[c3_hek_bytes_used] =
						field_info[field_num].filed_value.mac_addr.parsed_mac_addr[idx];
				} else {
					/* HEK value */
					c3_hek[c3_hek_bytes_used] =
						field_info[field_num].filed_value.ipv4_addr.parsed_ipv4_addr[idx];
				}
				c3_hek_bytes_used++;
			}
			break;

		case IPV6_SA_FIELD_ID:
		case IPV6_DA_FIELD_ID:
		case IPV6_SA_PREF_FIELD_ID:
		case IPV6_DA_PREF_FIELD_ID:
			for (idx = 0; idx < field_bytes; idx++) {
				/* HEK value */
				c3_hek[c3_hek_bytes_used] =
					field_info[field_num].filed_value.ipv6_addr.parsed_ipv6_addr[idx];
				c3_hek_bytes_used++;
			}
			break;

		case IPV6_SA_SUFF_FIELD_ID:
		case IPV6_DA_SUFF_FIELD_ID:
			/* IPv6 suffix needs to be moved to MSB bytes for SRAM */
			for (idx = field_bytes; idx < IPV6_ADDR_SIZE; idx++) {
				/* HEK value */
				c3_hek[c3_hek_bytes_used] =
					field_info[field_num].filed_value.ipv6_addr.parsed_ipv6_addr[idx];
				c3_hek_bytes_used++;
			}
			break;
		default:
			pr_err("Invalid field ID (%d) on C3 engine\n", field_id);
			return MV_ERROR;
		}
		/* record previous id */
		pre_field_id = field_id;

		/* increase field number */
		field_num++;

		/* Clear combine flag */
		comb_flag = false;
	}

	/* save HEK */
	*size = c3_hek_bytes_used;
	for (idx = 0; idx < c3_hek_bytes_used; idx++)
		hek[MVPP2_C3_MAX_HASH_KEY_SIZE - 1 - idx] = c3_hek[idx];

	return 0;
}

/**
 * pp2_cls_c3_rule_convert
 *
 * The routine converts a management C3 entry rule to PP2 one
 *
 * @param[in]	mng_entry	CLS management C3 engine entry
 *
 * @param[out]	hw_entry		CLS PP2 C3 engine entry
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_cls_c3_rule_convert(struct pp2_cls_c3_add_entry_t *mng_entry, struct pp2_cls_c3_entry *hw_entry)
{
	enum pp2_cls_l4_type_t l4_type;
	u32 hek_bytes;
	int hek_offs;
	u8 hek[MVPP2_C3_MAX_HASH_KEY_SIZE];
	int rc = 0;

	pr_debug("reached\n");

	/* NULL validation */
	if (mv_pp2x_ptr_validate(mng_entry))
		return -EINVAL;

	if (mv_pp2x_ptr_validate(hw_entry))
		return -EINVAL;

	/* init c3 entry */
	pp2_cls_c3_sw_clear(hw_entry);

	/* set L4 info  for NAPT 5-tupple */
	if (mng_entry->mng_pkt_key->pkt_key->field_bm == MVPP2_MATCH_IPV4_5T ||
	    mng_entry->mng_pkt_key->pkt_key->field_bm == MVPP2_MATCH_IPV6_5T) {
		if (mng_entry->mng_pkt_key->pkt_key->ipvx_add.ip_proto == IPPROTO_TCP)
			l4_type = MVPP2_L4_TYPE_TCP;
		else if (mng_entry->mng_pkt_key->pkt_key->ipvx_add.ip_proto == IPPROTO_UDP)
			l4_type = MVPP2_L4_TYPE_UDP;
		else
			l4_type = MVPP2_L4_TYPE_TCP; /* default one */

		rc = pp2_cls_c3_sw_l4_info_set(hw_entry, l4_type);
		if (rc) {
			pr_err("failed to call pp2_cls_c3_sw_l4_info_set\n");
			return rc;
		}
	}

	/* set lookup type */
	rc = pp2_cls_c3_sw_lkp_type_set(hw_entry, mng_entry->lkp_type);
	if (rc) {
		pr_err("failed to call pp2_cls_c3_sw_lkp_type_set\n");
		return rc;
	}

	/* set port ID */
	rc = pp2_cls_c3_sw_port_id_set(hw_entry, mng_entry->port.port_type, mng_entry->port.port_value);
	if (rc) {
		pr_err("failed to call pp2_cls_c3_sw_port_id_set\n");
		return rc;
	}

	/* set HEK */
	rc = pp2_cls_c3_hek_generate(mng_entry, &hek_bytes, hek);
	if (rc) {
		pr_err("failed to call pp2_cls_c3_hek_generate\n");
		return rc;
	}

	for (hek_offs = MVPP2_C3_MAX_HASH_KEY_SIZE - 1; hek_offs >= 0; hek_offs--) {
		if (pp2_cls_c3_sw_hek_byte_set(hw_entry, hek_offs, hek[hek_offs]) != MV_OK)
			return MV_ERROR;
	}

	rc = pp2_cls_c3_sw_hek_size_set(hw_entry, hek_bytes);
	if (rc) {
		pr_err("failed to call pp2_cls_c3_sw_hek_size_set\n");
		return rc;
	}

	/* set color */
	rc = pp2_cls_c3_color_set(hw_entry, mng_entry->action.color_act);
	if (rc) {
		pr_err("failed to call pp2_cls_c3_color_set\n");
		return rc;
	}

	/* set queue high */
	rc = pp2_cls_c3_queue_high_set(hw_entry, mng_entry->action.q_high_act, mng_entry->qos_value.q_high);
	if (rc) {
		pr_err("failed to call pp2_cls_c3_queue_high_set\n");
		return rc;
	}

	/* set queue low */
	rc = pp2_cls_c3_queue_low_set(hw_entry, mng_entry->action.q_low_act, mng_entry->qos_value.q_low);
	if (rc) {
		pr_err("failed to call pp2_cls_c3_queue_low_set\n");
		return rc;
	}

	/* set forward */
	rc = pp2_cls_c3_forward_set(hw_entry, mng_entry->action.frwd_act);
	if (rc) {
		pr_err("failed to call pp2_cls_c3_forward_set\n");
		return rc;
	}

	/* set rss */
	rc = pp2_cls_c3_rss_set(hw_entry, mng_entry->action.rss_act, mng_entry->rss_en);
	if (rc) {
		pr_err("failed to call pp2_cls_c3_rss_set\n");
		return rc;
	}

	/* Set policer */
	rc = pp2_cls_c3_policer_set(hw_entry,
				    mng_entry->action.policer_act,
				    mng_entry->qos_info.policer_id & MVPP2_CLS3_ACT_DUP_POLICER_MAX,
				    MVPP2_POLICER_2_BANK(mng_entry->qos_info.policer_id));
	if (rc) {
		pr_err("failed to call pp2_cls_c3_policer_set\n");
		return rc;
	}

	/* set flow ID */
	rc = pp2_cls_c3_flow_id_en(hw_entry, mng_entry->action.flowid_act);
	if (rc) {
		pr_err("failed to call pp2_cls_c3_flow_id_en\n");
		return rc;
	}

	/* set mod */
	rc = pp2_cls_c3_mod_set(hw_entry, mng_entry->pkt_mod.mod_data_idx, mng_entry->pkt_mod.mod_cmd_idx,
				mng_entry->pkt_mod.l4_chksum_update_flag);
	if (rc) {
		pr_err("failed to call pp2_cls_c3_mod_set\n");
		return rc;
	}

	/* set duplication */
	rc = pp2_cls_c3_dup_set(hw_entry, mng_entry->flow_info.flow_id, mng_entry->flow_info.flow_cnt);
	if (rc) {
		pr_err("failed to call pp2_cls_c3_dup_set\n");
		return rc;
	}

	return rc;
}

/**
 * pp2_cls_c3_default_rule_convert
 *
 * The routine converts a management C3 default entry rule to PP2 one
 *
 * @param[in]	mng_entry	CLS management C3 engine entry
 *
 * @param[out]	hw_entry		CLS PP2 C3 engine entry
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_cls_c3_default_rule_convert(struct pp2_cls_c3_add_entry_t *mng_entry, struct pp2_cls_c3_entry *hw_entry)
{
	int rc = 0;

	pr_debug("reached\n");

	/* NULL validation */
	if (mv_pp2x_ptr_validate(mng_entry))
		return -EINVAL;

	if (mv_pp2x_ptr_validate(hw_entry))
		return -EINVAL;

	/* init c3 entry */
	pp2_cls_c3_sw_clear(hw_entry);

	/* set lookup type */
	rc = pp2_cls_c3_sw_lkp_type_set(hw_entry, mng_entry->lkp_type);
	if (rc) {
		pr_err("failed to call pp2_cls_c3_sw_lkp_type_set\n");
		return rc;
	}

	/* set color */
	rc = pp2_cls_c3_color_set(hw_entry, mng_entry->action.color_act);
	if (rc) {
		pr_err("failed to call pp2_cls_c3_color_set\n");
		return rc;
	}

	/* set queue high */
	rc = pp2_cls_c3_queue_high_set(hw_entry, mng_entry->action.q_high_act, mng_entry->qos_value.q_high);
	if (rc) {
		pr_err("failed to call pp2_cls_c3_queue_high_set\n");
		return rc;
	}

	/* set queue low */
	rc = pp2_cls_c3_queue_low_set(hw_entry, mng_entry->action.q_low_act, mng_entry->qos_value.q_low);
	if (rc) {
		pr_err("failed to call pp2_cls_c3_queue_low_set\n");
		return rc;
	}

	/* set forward */
	rc = pp2_cls_c3_forward_set(hw_entry, mng_entry->action.frwd_act);
	if (rc) {
		pr_err("failed to call pp2_cls_c3_forward_set\n");
		return rc;
	}

	/* set rss */
	rc = pp2_cls_c3_rss_set(hw_entry, mng_entry->action.rss_act, mng_entry->rss_en);
	if (rc) {
		pr_err("failed to call pp2_cls_c3_rss_set\n");
		return rc;
	}

	/* Set policer */
	rc = pp2_cls_c3_policer_set(hw_entry,
				    mng_entry->action.policer_act,
				    mng_entry->qos_info.policer_id & MVPP2_CLS3_ACT_DUP_POLICER_MAX,
				    MVPP2_POLICER_2_BANK(mng_entry->qos_info.policer_id));
	if (rc) {
		pr_err("failed to call pp2_cls_c3_policer_set\n");
		return rc;
	}

	/* set flow ID */
	rc = pp2_cls_c3_flow_id_en(hw_entry, mng_entry->action.flowid_act);
	if (rc) {
		pr_err("failed to call pp2_cls_c3_flow_id_en\n");
		return rc;
	}

	/* set mod */
	rc = pp2_cls_c3_mod_set(hw_entry, mng_entry->pkt_mod.mod_data_idx, mng_entry->pkt_mod.mod_cmd_idx,
				mng_entry->pkt_mod.l4_chksum_update_flag);
	if (rc) {
		pr_err("failed to call pp2_cls_c3_mod_set\n");
		return rc;
	}

	/* set duplication */
	rc = pp2_cls_c3_dup_set(hw_entry, mng_entry->flow_info.flow_id, mng_entry->flow_info.flow_cnt);
	if (rc) {
		pr_err("failed to call pp2_cls_c3_dup_set\n");
		return rc;
	}

	return rc;
}

/**
 * pp2_cls_c3_rule_check
 *
 * The routine validate C3 entry
 *
 * @param[in]	c3_entry		CLS C3 engine entry
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_cls_c3_rule_check(struct pp2_cls_c3_add_entry_t *c3_entry)
{
	struct pp2_cls_field_match_info field_info[MVPP2_FLOW_FIELD_COUNT_MAX + 1];
	int idx;
	struct pp2_cls_c3_entry c3;
	u32 bits_cnt = 0;
	int rc = 0;

	pr_debug("reached\n");

	/* NULL validation */
	if (mv_pp2x_ptr_validate(c3_entry))
		return -EINVAL;

	/* port check */
	if (mv_pp2x_range_validate(c3_entry->port.port_type, 0, MVPP2_SRC_PORT_TYPE_VIR))
		return -EINVAL;

	if (c3_entry->port.port_type == MVPP2_SRC_PORT_TYPE_VIR) {
		if (c3_entry->port.port_value > MVPP2_VIRT_PORT_ID_MAX) {
			pr_err("Invalid Virt port ID(%d)\n", c3_entry->port.port_value);
			return -EIO;
		}
	}

	/* lookup type check */
	if (mv_pp2x_range_validate(c3_entry->lkp_type, 0, KEY_CTRL_LKP_TYPE_MAX))
		return -EINVAL;

	/* packet key check  */
	if (mv_pp2x_ptr_validate(c3_entry->mng_pkt_key) == MV_ERROR)
		return -EINVAL;

	/* get field info */
	memset(field_info, 0, sizeof(struct pp2_cls_field_match_info) * (MVPP2_FLOW_FIELD_COUNT_MAX + 1));
	if (pp2_cls_field_bm_to_field_info(c3_entry->mng_pkt_key->pkt_key->field_bm, c3_entry->mng_pkt_key,
					   MVPP2_FLOW_FIELD_COUNT_MAX + 1, false, field_info)) {
		pr_err("Field info get failed\n");
		return -EIO;
	}

	/* if not 5T, check field number -> if greater than 4 is invalid */
	if ((c3_entry->mng_pkt_key->pkt_key->field_bm != MVPP2_MATCH_IPV4_5T) &&
	    (c3_entry->mng_pkt_key->pkt_key->field_bm != MVPP2_MATCH_IPV6_5T) &&
	    (field_info[MVPP2_FLOW_FIELD_COUNT_MAX].valid == MVPP2_FIELD_VALID)) {
		pr_err("At most 4 fileds are supported\n");
		return -EIO;
	}
	/* raw check field length, total can not more than 36 bytes */
	for (idx = 0; idx < MVPP2_FLOW_FIELD_COUNT_MAX; idx++) {
		if (field_info[idx].valid == MVPP2_FIELD_VALID)
			bits_cnt += pp2_cls_field_size_get(field_info[idx].field_id);
	}
	if (bits_cnt > MVPP2_C3_MAX_HASH_KEY_SIZE * BYTE_BITS) {
		pr_err("Packet key length(%d bits) beyond C3 capability\n", bits_cnt);
		return -EIO;
	}

	/* QOS check, TBD */

	/* Action check, TBD */

	/* Mod info check */
	if (c3_entry->pkt_mod.mod_cmd_idx > MVPP2_HWF_MOD_IPTR_MAX) {
		pr_err("Invalid modification cmd index(%d)\n", c3_entry->pkt_mod.mod_cmd_idx);
		return -EIO;
	}
	if (c3_entry->pkt_mod.mod_data_idx > MVPP2_HW_MOD_DPTR_MAX) {
		pr_err("Invalid data index(%d)\n", c3_entry->pkt_mod.mod_data_idx);
		return -EIO;
	}

	/* Duplication flow info check */
	rc = pp2_cls_c3_rule_convert(c3_entry, &c3);
	if (rc) {
		pr_err("failed to convert C3 key\n");
		return rc;
	}

	/* To be implemented later: for rule update, maybe 2 rules have the same key may co-exist for a short time,
	*  so the repeat check will blcok rule update, remove it temporary, and in future commit
	*  maybe a new method found to implement it.
	*/

	return 0;
}

/**
 * pp2_cls_c3_default_rule_check
 *
 * The routine validate C3 entry
 *
 * @param[in]	c3_entry		CLS C3 engine default entry
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_cls_c3_default_rule_check(struct pp2_cls_c3_add_entry_t *c3_entry)
{
	pr_debug("reached\n");

	/* NULL validation */
	if (mv_pp2x_ptr_validate(c3_entry))
		return -EINVAL;

	/* lookup type check */
	if (mv_pp2x_range_validate(c3_entry->lkp_type, 0, KEY_CTRL_LKP_TYPE_MAX))
		return -EINVAL;

	/* QOS check, TBD */

	/* action check, TBD */

	/* mod info check */
	if (c3_entry->pkt_mod.mod_cmd_idx > MVPP2_HWF_MOD_IPTR_MAX) {
		pr_err("Invalid modification cmd index(%d)\n", c3_entry->pkt_mod.mod_cmd_idx);
		return -EIO;
	}
	if (c3_entry->pkt_mod.mod_data_idx > MVPP2_HW_MOD_DPTR_MAX) {
		pr_err("Invalid data index(%d)\n", c3_entry->pkt_mod.mod_data_idx);
		return -EIO;
	}

	return 0;
}

/**
 * pp2_cls_c3_rule_add
 *
 *The routine adds C3 entry
 *
 * @param[in]   inst            packet processor instance
 * @param[in]	c3_entry		CLS C3 engine entry
 *
 * @param[out]	logic_idx		logical index
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_cls_c3_rule_add(struct pp2_inst *inst,
			struct pp2_cls_c3_add_entry_t *c3_entry,
			u32 *logic_idx)
{
	u32 l_logic_idx;
	u32 hash_idx;
	struct pp2_cls_c3_entry c3;
	u32 max_search_depth;
	struct pp2_cls_c3_hash_pair hash_pair_arr;
	int rc = 0;
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);
#ifdef PP2_CLS_C3_DEBUG
	int idx;
#endif

	pr_debug("reached\n");

	/* validation */
	if (mv_pp2x_ptr_validate(c3_entry))
		return -EINVAL;

	if (mv_pp2x_ptr_validate(logic_idx))
		return -EINVAL;

	/* check C3 rule */
	rc = pp2_cls_c3_rule_check(c3_entry);
	if (rc) {
		pr_err("failed to check C3 entry\n");
		return rc;
	}

	/* convert the C3 mng entry to LSP entry */
	pp2_cls_c3_sw_clear(&c3);
	rc = pp2_cls_c3_rule_convert(c3_entry, &c3);
	if (rc) {
		pr_err("failed to call pp2_cls_c3_rule_convert\n");
		return rc;
	}

#ifdef PP2_CLS_C3_DEBUG
	pp2_cls_c3_sw_dump(&c3);
#endif

	/* get free logical index, also check whether there is an available entry */
	rc = pp2_cls_db_c3_free_logic_idx_get(inst, &l_logic_idx);
	if (rc) {
		pr_err("failed to get free logical index\n");
		return rc;
	}

	*logic_idx = l_logic_idx;

	/* add C3 entry */
	rc = pp2_cls_db_c3_search_depth_get(inst, &max_search_depth);
	if (rc) {
		pr_err("fail to get PP2_CLS C3 max search depth\n");
		return rc;
	}
	MVPP2_MEMSET_ZERO(hash_pair_arr);

#ifdef PP2_CLS_C3_DEBUG
	pr_debug("C3 HEK %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
		    c3.key.hek.bytes[35], c3.key.hek.bytes[34], c3.key.hek.bytes[33], c3.key.hek.bytes[32],
		    c3.key.hek.bytes[31], c3.key.hek.bytes[30], c3.key.hek.bytes[29], c3.key.hek.bytes[28]);
#endif
	rc = pp2_cls_c3_hw_query_add(cpu_slot, &c3, max_search_depth, &hash_pair_arr);
	/* do not need to release logic index since it is still not occuppied */
	if (rc) {
		pr_err("failed to add C3 entry to HW\n");
		return rc;
	}
	hash_idx = c3.index;

	/* update C3 DB multihash index */
#ifdef PP2_CLS_C3_DEBUG
	if (hash_pair_arr.pair_num) {
		pr_debug("hash pair number=%d\n", hash_pair_arr.pair_num);
		for (idx = 0; idx < hash_pair_arr.pair_num; idx++)
			pr_debug("hash pair(%d) %x-->%x\n",
				    idx, hash_pair_arr.old_idx[idx], hash_pair_arr.new_idx[idx]);
	}
#endif
	rc = pp2_cls_db_c3_hash_idx_update(inst, &hash_pair_arr);
	if (rc) {
		pr_err("failed to update C3 multihash index\n");
		return rc;
	}

	/* save to DB */
	rc = pp2_cls_db_c3_entry_add(inst, l_logic_idx, hash_idx);
	if (rc) {
		pr_err("failed to add C3 entry to DB\n");
		return rc;
	}

	return 0;
}

/**
 * pp2_cls_c3_default_rule_add
 *
 * The routine adds default C3 entry to handle the mismathched packets for specific lookup type
 *
 * @param[in]   inst            packet processor instance
 * @param[in]	c3_entry		CLS C3 engine entry
 *
 * @param[out]	logic_idx		logical index
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_cls_c3_default_rule_add(struct pp2_inst *inst,
				struct pp2_cls_c3_add_entry_t *c3_entry,
				u32 *logic_idx)
{
	u32 l_logic_idx;
	struct pp2_cls_c3_entry c3;
	int rc = 0;
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);

	pr_debug("reached\n");

	/* validation */
	if (mv_pp2x_ptr_validate(c3_entry))
		return -EINVAL;

	if (mv_pp2x_ptr_validate(logic_idx))
		return -EINVAL;

	/* check C3 rule */
	rc = pp2_cls_c3_default_rule_check(c3_entry);
	if (rc) {
		pr_err("failed to check C3 default entry\n");
		return rc;
	}

	/* convert the C3 mng entry to LSP entry */
	MVPP2_MEMSET_ZERO(c3);
	rc = pp2_cls_c3_default_rule_convert(c3_entry, &c3);
	if (rc) {
		pr_err("failed to call pp2_cls_c3_default_rule_convert()\n");
		return rc;
	}

	/* get free logical index, aslo check whether there is free entry */
	rc = pp2_cls_db_c3_free_logic_idx_get(inst, &l_logic_idx);
	if (rc) {
		pr_err("failed to get free logical index\n");
		return rc;
	}
	*logic_idx = l_logic_idx;

	rc = pp2_cls_c3_hw_miss_add(cpu_slot, &c3, c3_entry->lkp_type);
	if (rc) {
		pr_err("failed to add C3 miss entry to HW\n");
		return rc;
	}

	/* save to DB */
	rc = pp2_cls_db_c3_entry_add(inst, l_logic_idx, c3_entry->lkp_type);
	if (rc) {
		pr_err("failed to add C3 entry to DB\n");
		return rc;
	}

	return 0;
}

/**
 * pp2_cls_c3_rule_del
 *
 * The routine deletes C3 entry
 *
 * @param[in]   inst            packet processor instance
 * @param[in]	logic_idx		logical index
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_cls_c3_rule_del(struct pp2_inst *inst, u32 logic_idx)
{
	u32 hash_idx;
	int rc = 0;
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);

	pr_debug("reached\n");

	/* check C3 rule, return OK if the there is no this logical index */
	if (mv_pp2x_range_validate(logic_idx, 0, MVPP2_CLS_C3_HASH_TBL_SIZE - 1))
		return -EINVAL;

	rc = pp2_cls_db_c3_hash_idx_get(inst, logic_idx, &hash_idx);
	if (rc) {
		pr_err("The logical index(%d) does not exist", logic_idx);
		return rc;
	}

	/* delete C3 entry */
	rc = pp2_cls_c3_hw_del(cpu_slot, hash_idx);
	if (rc) {
		pr_err("failed to delete C3 entry from HW\n");
		return rc;
	}

	/* remove from DB */
	rc = pp2_cls_db_c3_entry_del(inst, logic_idx);
	if (rc) {
		pr_err("failed to delete C3 entry from DB\n");
		return rc;
	}

	return 0;
}

/**
 * pp2_cls_c3_rule_get
 *
 * The routine gets the C3 entries which occupy same multihash entries
 *
 * @param[in]	c3_entry		CLS C3 engine entry
 *
 * @param[out]	entry_num	number of matched multihash entries
 * @param[out]	logic_idx_arr	logical index array
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_cls_c3_rule_get(uintptr_t cpu_slot, struct pp2_cls_c3_add_entry_t *c3_entry, u32 *entry_num,
			u32 *logic_idx_arr[])
{
	int rc = 0;

	pr_debug("reached\n");

	/* leave this routine to be implemented in future when needed */
	return rc;
}

/**
 * pp2_cls_c3_hit_count_get
 *
 * The routine gets hit counter by logical index
 *
 * @param[in]   inst            packet processor instance
 * @param[in]	logic_idx		C3 logical index
 *
 * @param[out]	hit_count		multihash entry hit counter
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_cls_c3_hit_count_get(struct pp2_inst *inst, int logic_idx, u32 *hit_count)
{
	u32 hash_idx;
	int rc = 0;
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);

	/* validation */
	if (mv_pp2x_ptr_validate(hit_count))
		return -EINVAL;

	/* check C3 rule, return OK if the there is no this logical index */
	if (mv_pp2x_range_validate(logic_idx, 0, MVPP2_CLS_C3_HASH_TBL_SIZE - 1))
		return -EINVAL;

	rc = pp2_cls_db_c3_hash_idx_get(inst, logic_idx, &hash_idx);
	if (rc) {
		pr_err("The logical index(%d) does not exist\n", logic_idx);
		return rc;
	}

	/* get hit counter */
	rc = pp2_cls_c3_hit_cntrs_read(cpu_slot, hash_idx, hit_count);
	if (rc) {
		pr_err("fail to read hit counter for logical index(%d)\n", logic_idx);
		return rc;
	}

	return 0;
}

/**
 * pp2_cls_c3_hit_cntr_all_get
 *
 * The routine returns all hit counters above threshold
 *
 * @param[in]   inst            packet processor instance
 * @param[in]	hit_low_thresh	low threshold, hit counters above this will be returned
 * @param[in]	num_of_cntrs	size of cntr_info
 *
 * @param[out]	cntr_info		returned counter array with logical and physical index
 * @param[out]	num_of_cntrs	number of updated counters in cntr_info
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_cls_c3_hit_cntr_all_get(struct pp2_inst *inst,
				int hit_low_thresh,
				struct pp2_cls_hit_cnt_t cntr_info[],
				u32 *num_of_cntrs)
{
	int		phys_i, log_i;
	u32		cnt;
	u32		rc = 0;
	u32		cntr_idx;
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);

	pr_debug("reached\n");

	if (mv_pp2x_ptr_validate(cntr_info))
		return -EINVAL;

	if (mv_pp2x_ptr_validate(num_of_cntrs))
		return -EINVAL;

	cntr_idx = 0;

	for (phys_i = 0; phys_i < MVPP2_CLS_C3_HASH_TBL_SIZE; phys_i++) {
		rc = pp2_cls_c3_hit_cntrs_read(cpu_slot, phys_i, &cnt);
		if (rc)
			return rc;

		if (cnt >= hit_low_thresh) {
			if (*num_of_cntrs < cntr_idx) {
				pr_err("counter array too small, size = %d\n", *num_of_cntrs);
				return rc;
			}

			rc = pp2_cls_db_c3_logic_idx_get(inst, phys_i, &log_i);
			if (rc)
				continue;

			cntr_info[cntr_idx].log_idx = log_i;
			cntr_info[cntr_idx].phys_idx = phys_i;
			cntr_info[cntr_idx].cntr_val = cnt;
			cntr_idx++;
		}
	}

	/* update actual counters updated */
	*num_of_cntrs = cntr_idx;

	return 0;
}

/**
 * pp2_cls_c3_scan_param_set
 *
 * The routine sets scan configuration parameters
 *
 * @param[in]   inst            packet processor instance
 * @param[in]	scan_config	scan configuration
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_cls_c3_scan_param_set(struct pp2_inst *inst, struct pp2_cls_c3_scan_config_t *scan_config)
{
	int type;
	int rc = 0;
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);

	pr_debug("reached\n");

	/* validation */
	if (mv_pp2x_ptr_validate(scan_config))
		return -EINVAL;

	if (mv_pp2x_range_validate(scan_config->clear_before_scan, 0, 1))
		return -EINVAL;

	if (mv_pp2x_range_validate(scan_config->lkp_type_scan, 0, 1))
		return -EINVAL;

	if (mv_pp2x_range_validate(scan_config->lkp_type, 0, MVPP2_CLS3_SC_PROP_LKP_TYPE_MAX))
		return -EINVAL;

	if (mv_pp2x_range_validate(scan_config->scan_mode, 0, MVPP2_SCAN_ABOVE_THRESHOLD))
		return -EINVAL;

	if (mv_pp2x_range_validate(scan_config->start_entry, 0, MVPP2_CLS_C3_HASH_TBL_SIZE - 1))
		return -EINVAL;

	if (mv_pp2x_range_validate(scan_config->scan_delay, 0, MVPP2_CLS3_SC_PROP_VAL_DELAY_MAX))
		return -EINVAL;

	if (mv_pp2x_range_validate(scan_config->scan_threshold, 0, MVPP2_CLS3_SC_TH_MAX))
		return -EINVAL;

	/* set the configuration to HW */
	rc = pp2_cls_c3_scan_thresh_set(cpu_slot, scan_config->scan_mode, scan_config->scan_threshold);
	if (rc) {
		pr_err("fail to set scan mode and theshold\n");
		return rc;
	}

	rc = pp2_cls_c3_scan_clear_before_en_set(cpu_slot, scan_config->clear_before_scan);
	if (rc) {
		pr_err("fail to set clear before scan\n");
		return rc;
	}

	if (scan_config->lkp_type_scan)
		type = scan_config->lkp_type;
	else
		type = -1; /* MVPP2 defined to indicate that do not care about lkp_type */
	rc = pp2_cls_c3_scan_lkp_type_set(cpu_slot, type);
	if (rc) {
		pr_err("fail to set scan lookup type\n");
		return rc;
	}

	rc = pp2_cls_c3_scan_start_index_set(cpu_slot, scan_config->start_entry);
	if (rc) {
		pr_err("fail to set scan start index\n");
		return rc;
	}

	rc = pp2_cls_c3_scan_delay_set(cpu_slot, scan_config->scan_delay);
	if (rc) {
		pr_err("fail to set scan delay time\n");
		return rc;
	}

	/* save scan config to DB */
	rc = pp2_cls_db_c3_scan_param_set(inst, scan_config);
	if (rc) {
		pr_err("fail to set scan parameters to C3 DB\n");
		return rc;
	}

	return 0;
}

/**
 * pp2_cls_c3_scan_result_get
 *
 * The routine will trigger scan, wait and get scan results
 *
 * @param[in]   inst            packet processor instance
 * @param[in]	max_entry_num	Maximum entry number allowed
 *
 * @param[out]	entry_num	entry number
 * @param[out]	result_entry	hold result entry, including hash_idx, logic_idx, hit_count
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_cls_c3_scan_result_get(struct pp2_inst *inst, u32 max_entry_num, u32 *entry_num,
			       struct pp2_cls_c3_scan_entry_t result_entry[])
{
	int num;
	int idx;
	int hash_idx;
	int logic_idx;
	int hit_cnt;
	int rc = 0;
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);

	pr_debug("reached\n");

	/* validation */
	if (mv_pp2x_ptr_validate(entry_num))
		return -EINVAL;

	if (mv_pp2x_ptr_validate(result_entry))
		return -EINVAL;

	/* trigger scan */
	rc = pp2_cls_c3_scan_start(cpu_slot);
	if (rc) {
		pr_err("fail to start scan\n");
		return MV_ERROR;
	}

	/* Get scan entry number and compare w/ input one */
	rc = pp2_cls_c3_scan_num_of_res_get(cpu_slot, &num);
	if (rc) {
		pr_err("fail to get scan number\n");
		return rc;
	}

	if (max_entry_num < num)
		num = max_entry_num;

	*entry_num = 0;
	/* Get multihash index and hit counter array */
	for (idx = 0; idx < num; idx++) {
		rc = pp2_cls_c3_scan_res_read(cpu_slot, idx, &hash_idx, &hit_cnt);
		if (rc) {
			pr_err("fail to start scan\n");
			return rc;
		}

		/* get logical index by hash_idx */
		rc = pp2_cls_db_c3_logic_idx_get(inst, hash_idx, &logic_idx);
		if (rc) {
			pr_err("fail to get logical index\n");
			return rc;
		}

		/* fill array */
		result_entry[idx].hash_idx  = hash_idx;
		result_entry[idx].logic_idx = logic_idx;
		result_entry[idx].hit_cnt   = hit_cnt;
		(*entry_num)++;
	}

	return 0;
}

/**
 * pp2_cls_c3_entry_get
 *
 * The routine will get C3 entry from HW with C3 logical index
 *
 * @param[in]   inst            packet processor instance
 * @param[in]	logic_idx		C3 logical index
 *
 * @param[out]	c3_entry		C3 entry data
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_cls_c3_entry_get(struct pp2_inst *inst, u32 logic_idx, struct pp2_cls_c3_data_t *c3_entry)
{
	int rc = 0;
	u32 hash_idx;
	struct pp2_cls_c3_entry c3_data;
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);

	pr_debug("reached\n");

	/* Parameter check */
	if (mv_pp2x_range_validate(logic_idx, 0, MVPP2_CLS_C3_HASH_TBL_SIZE - 1))
		return -EINVAL;

	if (mv_pp2x_ptr_validate(c3_entry))
		return -EINVAL;

	/* Get HASH index */
	rc = pp2_cls_db_c3_hash_idx_get(inst, logic_idx, &hash_idx);
	if (rc) {
		pr_err("fail to access DB\n");
		return rc;
	}

	/* Read HW data with LSP API */
	rc = pp2_cls_c3_hw_read(cpu_slot, &c3_data, hash_idx);
	if (rc) {
		pr_err("pp2_cls_c3_hw_read fail\n");
		return rc;
	}

	/* Convert c3_data to pp2_cls c3_entry */
	c3_entry->port.port_type = (c3_data.key.key_ctrl & KEY_CTRL_PRT_ID_MASK) >> KEY_CTRL_PRT_ID;
	c3_entry->port.port_value = (c3_data.key.key_ctrl & KEY_CTRL_PRT_ID_TYPE_MASK) >> KEY_CTRL_PRT_ID_TYPE;
	c3_entry->lkp_type = (c3_data.key.key_ctrl & KEY_CTRL_LKP_TYPE_MASK) >> KEY_CTRL_LKP_TYPE;
	c3_entry->l4_type = (c3_data.key.key_ctrl & KEY_CTRL_L4_MASK) >> KEY_CTRL_L4;
	c3_entry->hek_len = (c3_data.key.key_ctrl & KEY_CTRL_HEK_SIZE_MASK) >> KEY_CTRL_HEK_SIZE;

	memcpy(c3_entry->hek, c3_data.key.hek.bytes, MVPP2_C3_MAX_HASH_KEY_SIZE);

	c3_entry->action.color_act = (c3_data.sram.regs.actions & MVPP2_CLS3_ACT_COLOR_MASK) >> MVPP2_CLS3_ACT_COLOR;
	c3_entry->action.q_low_act = (c3_data.sram.regs.actions & MVPP2_CLS3_ACT_LOW_Q_MASK) >> MVPP2_CLS3_ACT_LOW_Q;
	c3_entry->action.q_high_act = (c3_data.sram.regs.actions & MVPP2_CLS3_ACT_HIGH_Q_MASK) >> MVPP2_CLS3_ACT_HIGH_Q;
	c3_entry->action.policer_act = (c3_data.sram.regs.actions & MVPP2_CLS3_ACT_POLICER_SELECT_MASK) >>
					MVPP2_CLS3_ACT_POLICER_SELECT;
	c3_entry->action.rss_act = (c3_data.sram.regs.actions & MVPP2_CLS3_ACT_RSS_EN_MASK) >> MVPP2_CLS3_ACT_RSS_EN;
	c3_entry->action.flowid_act = (c3_data.sram.regs.actions & MVPP2_CLS3_ACT_FLOW_ID_EN_MASK) >>
				       MVPP2_CLS3_ACT_FLOW_ID_EN;
	c3_entry->action.frwd_act = (c3_data.sram.regs.actions & MVPP2_CLS3_ACT_FWD_MASK) >> MVPP2_CLS3_ACT_FWD;
	c3_entry->qos_value.q_high = (c3_data.sram.regs.qos_attr & MVPP2_CLS3_ACT_QOS_ATTR_HIGH_Q_MASK) >>
				     MVPP2_CLS3_ACT_QOS_ATTR_HIGH_Q;
	c3_entry->qos_value.q_low = (c3_data.sram.regs.qos_attr & MVPP2_CLS3_ACT_QOS_ATTR_LOW_Q_MASK) >>
				    MVPP2_CLS3_ACT_QOS_ATTR_LOW_Q;
	c3_entry->pkt_mod.mod_cmd_idx = (c3_data.sram.regs.hwf_attr & MVPP2_CLS3_ACT_HWF_ATTR_IPTR_MASK) >>
					 MVPP2_CLS3_ACT_HWF_ATTR_IPTR;
	c3_entry->pkt_mod.mod_data_idx = (c3_data.sram.regs.hwf_attr & MVPP2_CLS3_ACT_HWF_ATTR_DPTR_MASK) >>
					  MVPP2_CLS3_ACT_HWF_ATTR_DPTR;
	c3_entry->pkt_mod.l4_chksum_update_flag = (c3_data.sram.regs.hwf_attr &
						   MVPP2_CLS3_ACT_HWF_ATTR_CHKSM_EN_MASK) >>
						   MVPP2_CLS3_ACT_HWF_ATTR_CHKSM_EN;
	c3_entry->policer_id = (c3_data.sram.regs.dup_attr & MVPP2_CLS3_ACT_DUP_POLICER_MASK) >>
				MVPP2_CLS3_ACT_DUP_POLICER_ID;
	c3_entry->dup_info.flow_id = (c3_data.sram.regs.dup_attr & MVPP2_CLS3_ACT_DUP_FID_MASK) >>
				      MVPP2_CLS3_ACT_DUP_FID;
	c3_entry->dup_info.flow_cnt = (c3_data.sram.regs.dup_attr & MVPP2_CLS3_ACT_DUP_COUNT_MASK) >>
				       MVPP2_CLS3_ACT_DUP_COUNT;

	return 0;
}

/**
 * pp2_cls_c3_reset
 *
 * The routine reset and re-satrt PP2_CLS C3 sub-module
 *
 * @param[in]   inst            packet processor instance
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_cls_c3_reset(struct pp2_inst *inst)
{
	int rc = 0;
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);

	pr_debug("reached\n");

	/* clear all C3 HW entries */
	rc = pp2_cls_c3_hw_del_all(cpu_slot);
	if (rc) {
		pr_err("fail to delete C3 HW entries\n");
		return rc;
	}
	pr_debug("PP2_CLS C3 HW entries deleted\n");

	/* clear all C3 HW counters */
	rc = pp2_cls_c3_hit_cntrs_clear_all(cpu_slot);
	if (rc) {
		pr_err("fail to clear C3 HW counters\n");
		return rc;
	}
	pr_debug("PP2_CLS C3 HW counters cleared\n");

	/* init PP2_CLS C3 DB */
	rc = pp2_cls_db_c3_init(inst);
	if (rc) {
		pr_err("fail to init PP2_CLS C3 DB\n");
		return rc;
	}
	pr_debug("PP2_CLS C3 DB initialized\n");

	/* init PP2_CLS C3 HAL */
	rc = pp2_cls_c3_init(cpu_slot);
	if (rc) {
		pr_err("fail to init PP2_CLS C3 DB\n");
		return rc;
	}
	pr_debug("PP2_CLS C3 DB initialized\n");

	/* set PP2_CLS C3 maximum search depth */
	rc = pp2_cls_db_c3_search_depth_set(inst, MVPP2_C3_DEFAULT_SEARCH_DEPTH);
	if (rc) {
		pr_err("fail to set PP2_CLS C3 max search depth\n");
		return rc;
	}
	pr_debug("PP2_CLS C3 max depth set to %d\n", MVPP2_C3_DEFAULT_SEARCH_DEPTH);

	return 0;
}

/**
 * pp2_cls_c3_start
 *
 * The routine starts PP2_CLS C3 sub-module
 *
 * @param[in]   inst            packet processor instance
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_cls_c3_start(struct pp2_inst *inst)
{
	if (pp2_cls_c3_reset(inst)) {
		pr_err("PP2_CLS C3 start failed\n");
		return -EIO;
	}
	pr_debug("PP2_CLS C3 started\n");

	return 0;
}
