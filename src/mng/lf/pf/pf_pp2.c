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

#define log_fmt(fmt, ...) "pf_pp2#%d: " fmt, nmnicpf->pf_id, ##__VA_ARGS__

#include "std_internal.h"

#include "mng/lf/lf_mng.h"
#include "mng/lf/mng_cmd_desc.h"
#include "mng/include/guest_mng_cmd_desc.h"

#include "pf_pp2.h"


/* Maximum size of port name */
#define NMP_PPIO_NAME_MAX			20


static int nmnicpf_pp2_find_free_cls_table(struct nmnicpf *nmnicpf)
{
	int i;

	for (i = 0; i < MAX_PP2_CLS_TBL; i++) {
		if (!nmnicpf->pp2.tbl[i])
			break;
	}
	if (i == MAX_PP2_CLS_TBL) {
		pr_err("no free cls table found!\n");
		return -ENOSPC;
	}
	return i;
}

#ifdef PP2_CLS_KEY_MASK_STRING_FORMAT
static int build_rule(struct nmnicpf *nmnicpf,
		      struct pp2_cls_tbl_rule *dst_rule,
		      struct guest_pp2_cls_tbl_rule *src_rule,
		      struct pp2_cls_tbl_params *tbl_params)
{
	struct pp2_proto_field *proto_field;
	u8 buf[GUEST_PP2_CLS_KEY_SIZE_MAX];
	int i;

	dst_rule->num_fields = src_rule->num_fields;
	for (i = 0; i < dst_rule->num_fields; i++) {
		dst_rule->fields[i].size = src_rule->fields[i].size;
		if (src_rule->fields[i].key_valid)
			dst_rule->fields[i].key = src_rule->fields[i].key;
		if (src_rule->fields[i].mask_valid)
			dst_rule->fields[i].mask = src_rule->fields[i].mask;
		proto_field = &tbl_params->key.proto_field[i];
		if ((proto_field->proto == MV_NET_PROTO_ETH) &&
		    ((proto_field->field.eth == MV_NET_ETH_F_SA) || (proto_field->field.eth == MV_NET_ETH_F_DA))) {
			memcpy(buf, src_rule->fields[i].key, GUEST_PP2_CLS_KEY_SIZE_MAX);
			snprintf((char *)src_rule->fields[i].key,
				 GUEST_PP2_CLS_KEY_SIZE_MAX,
				 "%02x:%02x:%02x:%02x:%02x:%02x",
				 buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
			memcpy(buf, src_rule->fields[i].mask, GUEST_PP2_CLS_KEY_SIZE_MAX);
			snprintf((char *)src_rule->fields[i].mask,
				 GUEST_PP2_CLS_KEY_SIZE_MAX,
				 "%02x:%02x:%02x:%02x:%02x:%02x",
				 buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
		} else if (proto_field->proto == MV_NET_PROTO_VLAN) {
			if (proto_field->field.vlan == MV_NET_VLAN_F_PRI) {
				memcpy(buf, src_rule->fields[i].key, GUEST_PP2_CLS_KEY_SIZE_MAX);
				snprintf((char *)src_rule->fields[i].key,
					 GUEST_PP2_CLS_KEY_SIZE_MAX,
					 "%02x", buf[0]);
				memcpy(buf, src_rule->fields[i].mask, GUEST_PP2_CLS_KEY_SIZE_MAX);
				snprintf((char *)src_rule->fields[i].mask,
					GUEST_PP2_CLS_KEY_SIZE_MAX,
					"%02x", buf[0]);
			} else if (proto_field->field.vlan == MV_NET_VLAN_F_ID) {
				memcpy(buf, src_rule->fields[i].key, GUEST_PP2_CLS_KEY_SIZE_MAX);
				snprintf((char *)src_rule->fields[i].key,
					 GUEST_PP2_CLS_KEY_SIZE_MAX,
					 "%02x%02x", buf[0], buf[1]);
				memcpy(buf, src_rule->fields[i].mask, GUEST_PP2_CLS_KEY_SIZE_MAX);
				snprintf((char *)src_rule->fields[i].mask,
					GUEST_PP2_CLS_KEY_SIZE_MAX,
					"%02x%02x", buf[0], buf[1]);
			} else {
				pr_err("field[%d] is not supported\n", i);
				return -ENOTSUP;
			}
		} else {
			pr_err("field[%d] is not supported\n", i);
			return -ENOTSUP;
		}
	}

	return 0;
}
#else
static void build_rule(struct nmnicpf *nmnicpf, struct pp2_cls_tbl_rule *dst_rule,
		       struct guest_pp2_cls_tbl_rule *src_rule)
{
	int i;

	dst_rule->num_fields = src_rule->num_fields;
	for (i = 0; i < dst_rule->num_fields; i++) {
		dst_rule->fields[i].size = src_rule->fields[i].size;
		if (src_rule->fields[i].key_valid)
			dst_rule->fields[i].key = src_rule->fields[i].key;
		if (src_rule->fields[i].mask_valid)
			dst_rule->fields[i].mask = src_rule->fields[i].mask;
		pr_debug("src-key %s, dst-key %s\n", src_rule->fields[i].key, dst_rule->fields[i].key);
	}
}
#endif /* PP2_CLS_KEY_MASK_STRING_FORMAT */


/*
 *	nmnicpf_pp2_port_pp2_init
 *
 *	Initialize the pp2_port of type PP2
 *
 *	@param[in]	nmnicpf - pointer to NIC PF object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int nmnicpf_pp2_port_pp2_init(struct nmnicpf *nmnicpf)
{
	int					err;
	int					k;
	struct nmp_pp2_bpool_desc		*pools = NULL;
	u32					 pp_id, port_id, scanned;
	struct nmp_lf_nicpf_pp2_port_params	*pf_pp2_profile;
	struct nmp_pp2_port_desc		*pdesc;

	pf_pp2_profile = &nmnicpf->profile_data.pp2_port;
	nmnicpf->pp2.num_ports = 1;

	/* Allocate memory for the pp2 descriptors */
	pdesc = kzalloc(sizeof(struct nmp_pp2_port_desc), GFP_KERNEL);
	if (!pdesc)
		return -ENOMEM;

	nmnicpf->pp2.ports_desc = pdesc;
	pdesc->num_pools = pf_pp2_profile->lcl_num_bpools;
	pr_debug("Number of pools %d\n", pdesc->num_pools);
	pools = kmalloc((sizeof(struct nmp_pp2_bpool_desc) * pdesc->num_pools), GFP_KERNEL);
	if (!pools) {
		pr_err("no mem for bpool_desc array!\n");
		err =  -ENOMEM;
		goto init_desc_exit1;
	}
	pdesc->pools_desc = pools;

	/* Parse match string to ring number */
	scanned = sscanf(pf_pp2_profile->match, "ppio-%d:%d\n", &pp_id, &port_id);
	if (scanned != 2) {
		pr_err("Invalid match string %s. Expected: ppio-0:X\n",
			pf_pp2_profile->match);
		err =  -EINVAL;
		goto init_desc_exit2;
	}
	pdesc->pp_id = pp_id;
	pdesc->ppio_id = port_id;

	pr_debug("pp_id %d, port_id %d\n", pdesc->pp_id, pdesc->ppio_id);
	for (k = 0; k < pdesc->num_pools; k++) {
		pools[k].num_buffs = pf_pp2_profile->lcl_bpools_params[k].max_num_buffs;
		pools[k].buff_size = pf_pp2_profile->lcl_bpools_params[k].buff_size;
	}
	pdesc->first_inq = 0; /* Fixed value */
	pdesc->first_rss_tbl = 0; /* Fixed value */
	pdesc->hash_type = 0; /* Fixed value */
	pdesc->pkt_offst = nmnicpf->profile_data.dflt_pkt_offset;
	pr_debug("pdesc pkt_offset: %d\n", pdesc->pkt_offst);
	pdesc->inq_size = nmnicpf->profile_data.lcl_ingress_q_size;
	pdesc->max_num_tcs = nmnicpf->profile_data.max_num_tcs;
	pdesc->num_tcs = 1; /* Value is updated after init_done command */
	for (k = 0; k < pdesc->num_tcs; k++)
	pdesc->num_inqs[k] = 1; /* Value is updated after init_done command */
	pdesc->num_outqs = 1; /* Value is updated after init_done command */
	pdesc->outq_size = nmnicpf->profile_data.lcl_egress_q_size;

	return 0;

init_desc_exit2:
	kfree(pools);
init_desc_exit1:
	kfree(pdesc);
	return err;
}

/*
 *	nmnicpf_pp2_port_init
 *
 *	Initialize the pp2_port according to port type (PP2 or LAG)
 *
 *	@param[in]	nmnicpf - pointer to NIC PF object
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise
 */
int nmnicpf_pp2_port_init(struct nmnicpf *nmnicpf)
{
	int					err = 0;

	/* Get the number of ports requested */
	/* TODO: currently only one container and one LF supported. */

	switch (nmnicpf->profile_data.port_type) {
	case NMP_LF_NICPF_T_NONE:
		/* no pp2 port, just return */
		return 0;
	case NMP_LF_NICPF_T_PP2_PORT:
		err = nmnicpf_pp2_port_pp2_init(nmnicpf);
		break;
	case NMP_LF_NICPF_T_PP2_LAG:
		pr_err("nicpf of type PP2_LAG is not supported yet\n");
		err = -EINVAL;
		break;
	default:
		pr_err("invalid nicpf type\n");
		err = -EINVAL;
	}

	return err;
}

/** =========================== **/
/** == Serialization helpers == **/
/** =========================== **/

/* Serialize PP2 relation information */
int nmnicpf_pp2_serialize_relation_inf(struct nmnicpf *nmnicpf, char *buff, u32 size, u8 depth)
{
	size_t	 pos = 0;
	u32	 port_index, bp_index;

	if (!nmnicpf->pp2.ports_desc)
		/* no pp2, just return */
		return 0;

	/* Serialize relations info */
	json_print_to_buffer(buff, size, depth, "\"num_pp2_ports\": %d,\n", nmnicpf->pp2.num_ports);
	for (port_index = 0; port_index < nmnicpf->pp2.num_ports; port_index++) {
		struct nmp_pp2_port_desc *port = (struct nmp_pp2_port_desc *)&nmnicpf->pp2.ports_desc[port_index];

		json_print_to_buffer(buff, size, depth, "\"ppio-%d\": \"ppio-%d:%d\",\n",
				     port_index, port->pp_id, port->ppio_id);
		json_print_to_buffer(buff, size, depth, "\"num_pp2_bpools\": %d,\n", port->num_pools);
		for (bp_index = 0; bp_index < port->num_pools; bp_index++) {
			if (bp_index == port->num_pools - 1)
				json_print_to_buffer(buff, size, depth, "\"bpool-%d\": \"pool-%d:%d\"\n", bp_index,
						port->pools_desc[bp_index].pool->pp2_id,
						port->pools_desc[bp_index].pool->id);
			else
				json_print_to_buffer(buff, size, depth, "\"bpool-%d\": \"pool-%d:%d\",\n", bp_index,
						port->pools_desc[bp_index].pool->pp2_id,
						port->pools_desc[bp_index].pool->id);
		}
	}

	return pos;
}

/* Serialize PP2 */
int nmnicpf_pp2_serialize(struct nmnicpf *nmnicpf, char *buff, u32 size)
{
	int	 ret;
	size_t	 pos = 0;
	u32	 port_index, bp_index;

	if (!nmnicpf->pp2.ports_desc)
		/* no pp2, just return */
		return 0;

	/* Serialize bpools */
	for (port_index = 0; port_index < nmnicpf->pp2.num_ports; port_index++) {
		struct nmp_pp2_port_desc *port = (struct nmp_pp2_port_desc *)&nmnicpf->pp2.ports_desc[port_index];

		for (bp_index = 0; bp_index < port->num_pools; bp_index++) {
			ret = pp2_bpool_serialize(port->pools_desc[bp_index].pool, &buff[pos], size - pos);
			if (ret < 0)
				return ret;
			pos += ret;
			if (pos != strlen(buff)) {
				pr_err("found mismatch between pos (%zu) and buff len (%zu)\n", pos, strlen(buff));
				return -EINVAL;
			}
		}
	}

	/* serialize ppio info */
	for (port_index = 0; port_index < nmnicpf->pp2.num_ports; port_index++) {
		struct nmp_pp2_port_desc *port = (struct nmp_pp2_port_desc *)&nmnicpf->pp2.ports_desc[port_index];

		/* Serialize ppio */
		ret = pp2_ppio_serialize(port->ppio, &buff[pos], size - pos);
		if (ret < 0)
			return ret;
		pos += ret;
		if (pos != strlen(buff)) {
			pr_err("found mismatch between pos (%zu) and buff len (%zu)\n", pos, strlen(buff));
			return -EINVAL;
		}
	}

	return pos;
}

int nmnicpf_pp2_init_bpools(struct nmnicpf *nmnicpf)
{
	int				 i;
	struct pp2_bpool_params		 bpool_params;
	int				 err, pool_id;
	char				 name[15];
	u32				 pcount = 0;
	struct nmp_pp2_port_desc	*pdesc;
	u32				 num_pools = 0;

	pdesc = (struct nmp_pp2_port_desc *)&nmnicpf->pp2.ports_desc[pcount];

	for (i = 0; i < pdesc->num_pools; i++) {
		pool_id = nmnicpf->f_pp_find_free_bpool_cb(nmnicpf->arg, pdesc->pp_id);
		if (pool_id < 0) {
			pr_err("free bpool not found!\n");
			return pool_id;
		}
		memset(name, 0, sizeof(name));
		snprintf(name, sizeof(name), "pool-%d:%d", pdesc->pp_id, pool_id);
		memset(&bpool_params, 0, sizeof(bpool_params));
		bpool_params.match = name;
		bpool_params.buff_len = pdesc->pools_desc[i].buff_size;
		pr_debug("%s: buff_size %d, num_buffs %d\n", name,
			bpool_params.buff_len, pdesc->pools_desc[i].num_buffs);
		err = pp2_bpool_init(&bpool_params, &pdesc->pools_desc[i].pool);
		if (err)
			goto init_bpools_err;

		if (!pdesc->pools_desc[i].pool) {
			err = -EINVAL;
			pr_err("BPool id%d init failed!\n", pool_id);
			goto init_bpools_err;
		}
		num_pools++;
	}
	return 0;

init_bpools_err:
	for (i = 0; i < num_pools; i++)
		pp2_bpool_deinit(pdesc->pools_desc[i].pool);
	return err;
}

int nmnicpf_pp2_init_ppio(struct nmnicpf *nmnicpf)
{
	struct pp2_ppio_params		 port_params;
	struct pp2_ppio_inq_params	 inq_params[PP2_PPIO_MAX_NUM_INQS];
	char				 name[NMP_PPIO_NAME_MAX];
	int				 i, j, err = 0;
	u32				 pcount = 0;
	struct nmp_pp2_port_desc	*pdesc;
	int				 num_pools;
	struct nmp_pp2_bpool_desc	*pools;
	struct pp2_ppio_outq_params	*outq_params;

	if (!nmnicpf->pp2.ports_desc)
		/* no pp2, just return */
		return 0;

	pdesc = (struct nmp_pp2_port_desc *)&nmnicpf->pp2.ports_desc[pcount];
	num_pools = pdesc->num_pools;
	pools = pdesc->pools_desc;

	if (!pp2_ppio_available(pdesc->pp_id, pdesc->ppio_id))
		return -EINVAL;

	memset(&port_params, 0, sizeof(struct pp2_ppio_params));

	memset(name, 0, sizeof(name));
	snprintf(name, sizeof(name), "ppio-%d:%d", pdesc->pp_id, pdesc->ppio_id);
	pr_debug("found port: %s\n", name);
	port_params.match = name;
	port_params.type = PP2_PPIO_T_NIC;
	port_params.eth_start_hdr = PP2_PPIO_HDR_ETH;
	if (pdesc->num_tcs > pdesc->max_num_tcs) {
		pr_err("Number of TC's configured (%d) exceeds PP2 available TC's (%d)\n",
		       pdesc->num_tcs, pdesc->max_num_tcs);
		return -EINVAL;
	}
	port_params.inqs_params.num_tcs = pdesc->num_tcs;
	port_params.inqs_params.hash_type = pdesc->hash_type;
	port_params.specific_type_params.log_port_params.first_inq = pdesc->first_inq;

	for (i = 0; i < pdesc->num_tcs; i++) {
		port_params.inqs_params.tcs_params[i].pkt_offset = pdesc->pkt_offst;
		port_params.inqs_params.tcs_params[i].num_in_qs = pdesc->num_inqs[i];
		for (j = 0; j < pdesc->num_inqs[i]; j++) {
			inq_params[j].size = pdesc->inq_size;
			inq_params[j].mem = NULL;
			inq_params[j].tc_pools_mem_id_index = 0;
		}
		port_params.inqs_params.tcs_params[i].inqs_params = inq_params;
		for (j = 0; j < num_pools; j++)
			port_params.inqs_params.tcs_params[i].pools[0][j] = pools[j].pool;

	}
	port_params.outqs_params.num_outqs = pdesc->num_outqs;
	for (i = 0; i < pdesc->num_outqs; i++) {
		port_params.outqs_params.outqs_params[i].size = pdesc->outq_size;
		if (pdesc->q_desc[i].rate_limit_enable) {
			outq_params = &port_params.outqs_params.outqs_params[i];
			outq_params->rate_limit_enable = 1;
			outq_params->rate_limit_params.cbs = pdesc->q_desc[i].rate_limit.cbs;
			outq_params->rate_limit_params.cir = pdesc->q_desc[i].rate_limit.cir;
		}
	}

	err = pp2_ppio_init(&port_params, &pdesc->ppio);
	if (err) {
		pr_err("PP-IO init failed (error: %d)!\n", err);
		return err;
	}

	if (!pdesc->ppio) {
		pr_err("PP-IO init failed!\n");
		return -EIO;
	}

	return 0;
}

int nmnicpf_pp2_accumulate_statistics(struct nmnicpf *nmnicpf,
				      struct pp2_ppio_statistics *stats,
				      int    reset)
{
	struct nmp_pp2_port_desc *pdesc;
	u32 pcount = 0;
	int err;

	if (!nmnicpf->pp2.ports_desc)
		/* no pp2, just return */
		/* TODO: handle guest mode (i.e. notify guest to
		 *	 to handle the request)
		 */
		return 0;

	pdesc = (struct nmp_pp2_port_desc *)&nmnicpf->pp2.ports_desc[pcount];
	if (!pdesc->ppio)
		/* PPIO is not initialized (yet), just return */
		return 0;

	memset(stats, 0, sizeof(struct pp2_ppio_statistics));

	/* Read PP2 statistics */
	err = pp2_ppio_get_statistics(pdesc->ppio, stats, reset);
	if (err) {
		pr_err("Failed to read pp2 statistics (%d)\n", err);
		return -EFAULT;
	}

	return 0;
}

int nmnicpf_pp2_get_statistics(struct nmnicpf *nmnicpf,
			       struct mgmt_cmd_params *params,
			       struct mgmt_cmd_resp *resp_data)
{
	int ret;

	ret = nmnicpf_pp2_accumulate_statistics(nmnicpf, &nmnicpf->stats, params->get_statistics.reset);
	if (ret)
		return ret;

	if (!resp_data)
		return -EFAULT;

	resp_data->agnic_stats.rx_bytes = nmnicpf->stats.rx_bytes;
	resp_data->agnic_stats.rx_packets = nmnicpf->stats.rx_packets;
	resp_data->agnic_stats.rx_unicast_packets = nmnicpf->stats.rx_unicast_packets;
	resp_data->agnic_stats.rx_errors = nmnicpf->stats.rx_errors;
	resp_data->agnic_stats.rx_fullq_dropped = nmnicpf->stats.rx_fullq_dropped;
	resp_data->agnic_stats.rx_bm_dropped = nmnicpf->stats.rx_bm_dropped;
	resp_data->agnic_stats.rx_early_dropped = nmnicpf->stats.rx_early_dropped;
	resp_data->agnic_stats.rx_fifo_dropped = nmnicpf->stats.rx_fifo_dropped;
	resp_data->agnic_stats.rx_cls_dropped = nmnicpf->stats.rx_cls_dropped;
	resp_data->agnic_stats.tx_bytes = nmnicpf->stats.tx_bytes;
	resp_data->agnic_stats.tx_packets = nmnicpf->stats.tx_packets;
	resp_data->agnic_stats.tx_unicast_packets = nmnicpf->stats.tx_unicast_packets;
	resp_data->agnic_stats.tx_errors = nmnicpf->stats.tx_errors;

	return 0;
}

void nmnicpf_pp2_get_mac_addr(struct nmnicpf *nmnicpf, u8 *mac_addr)
{
	struct ifreq s;
	char linux_name[16];
	int ret, i;
	u8 default_mac_addr[] = INITIAL_MAC_ADDR;

	if (nmnicpf->pp2.num_ports >= 1) {
		ret = pp2_netdev_get_ifname(nmnicpf->pp2.ports_desc[0].pp_id,
					nmnicpf->pp2.ports_desc[0].ppio_id,
					linux_name);
		if (ret)
			goto set_default;

		strcpy(s.ifr_name, linux_name);
		ret = mv_netdev_ioctl(SIOCGIFHWADDR, &s);
		if (ret)
			goto set_default;

		for (i = 0; i < ETH_ALEN; i++)
			mac_addr[i] = s.ifr_hwaddr.sa_data[i];
		return;
	}
set_default:
	memcpy(mac_addr, default_mac_addr, sizeof(default_mac_addr));
}

int nmnicpf_pp2_cls_table_init(struct nmnicpf *nmnicpf,
			       void *msg,
			       u16 msg_len,
			       struct guest_pp2_cls_cmd_resp *resp)
{
	struct guest_pp2_cls_tbl_params *tbl_params;
	struct pp2_cls_cos_desc	cos;
	int tbl_idx, ret;

	if (!nmnicpf->pp2.ports_desc || !nmnicpf->pp2.ports_desc[0].ppio)
		return -ENOTSUP;

	if (msg_len < sizeof(struct guest_pp2_cls_tbl_params)) {
		pr_err("invalid msg length. expected %lu, got %u\n",
			sizeof(struct guest_pp2_cls_tbl_params), msg_len);
		return -EINVAL;
	}

	tbl_params = (struct guest_pp2_cls_tbl_params *)msg;
	if (tbl_params->params.default_act.cos) {
		pr_err("Setting CoS is not supported\n");
		return -EINVAL;
	}

	tbl_idx = nmnicpf_pp2_find_free_cls_table(nmnicpf);
	if (tbl_idx < 0)
		return tbl_idx;

	cos.ppio = nmnicpf->pp2.ports_desc[0].ppio;
	cos.tc = 0;
	tbl_params->params.default_act.cos = &cos;
	ret = pp2_cls_tbl_init(&tbl_params->params, &nmnicpf->pp2.tbl[tbl_idx]);
	if (ret)
		return ret;

	resp->tbl_init.tbl_id = tbl_idx;
#ifdef PP2_CLS_KEY_MASK_STRING_FORMAT
	memcpy(&nmnicpf->pp2.tbl_params[tbl_idx], &tbl_params->params, sizeof(tbl_params->params));
#endif /* PP2_CLS_KEY_MASK_STRING_FORMAT */

	return 0;
}

int nmnicpf_pp2_cls_table_deinit(struct nmnicpf *nmnicpf,
				 void *msg,
				 u16 msg_len)
{
	u32 *tbl_idx;

	if (!nmnicpf->pp2.ports_desc || !nmnicpf->pp2.ports_desc[0].ppio)
		return -ENOTSUP;

	if (msg_len < sizeof(u32)) {
		pr_err("invalid msg length. expected %lu, got %u\n",
			sizeof(u32), msg_len);
		return -EINVAL;
	}

	tbl_idx = msg;

	if (*tbl_idx >= MAX_PP2_CLS_TBL)
		pr_err("invalid table index.\n");
	else
		pp2_cls_tbl_deinit(nmnicpf->pp2.tbl[*tbl_idx]);

	return 0;
}

int nmnicpf_pp2_cls_rule_add(struct nmnicpf *nmnicpf,
			     void *msg,
			     u16 msg_len)
{
	struct guest_pp2_cls_rule_add *rule_add;
	struct pp2_cls_tbl_rule rule;
	int ret;

	if (!nmnicpf->pp2.ports_desc || !nmnicpf->pp2.ports_desc[0].ppio)
		return -ENOTSUP;

	if (msg_len < sizeof(struct guest_pp2_cls_rule_add)) {
		pr_err("invalid msg length. expected %lu, got %u\n",
			sizeof(struct guest_pp2_cls_rule_add), msg_len);
		return -EINVAL;
	}

	rule_add = (struct guest_pp2_cls_rule_add *)msg;

	if (rule_add->tbl_id >= MAX_PP2_CLS_TBL) {
		pr_err("invalid table index.\n");
		return -EINVAL;
	}

	if (rule_add->action.action.cos) {
		pr_err("Setting CoS is not supported\n");
		return -EINVAL;
	}

#ifdef PP2_CLS_KEY_MASK_STRING_FORMAT
	ret = build_rule(nmnicpf, &rule,
			 &rule_add->rule,
			 &nmnicpf->pp2.tbl_params[rule_add->tbl_id]);
	if (ret)
		return ret;
#else
	build_rule(nmnicpf, &rule, &rule_add->rule);
#endif /* PP2_CLS_KEY_MASK_STRING_FORMAT */
	ret = pp2_cls_tbl_add_rule(nmnicpf->pp2.tbl[rule_add->tbl_id],
				   &rule,
				   &rule_add->action.action);
	return ret;
}

int nmnicpf_pp2_cls_rule_modify(struct nmnicpf *nmnicpf,
				void *msg,
				u16 msg_len)
{
	struct guest_pp2_cls_rule_add *rule_add;
	struct pp2_cls_tbl_rule rule;
	int ret;

	if (!nmnicpf->pp2.ports_desc || !nmnicpf->pp2.ports_desc[0].ppio)
		return -ENOTSUP;

	if (msg_len < sizeof(struct guest_pp2_cls_rule_add)) {
		pr_err("invalid msg length. expected %lu, got %u\n",
			sizeof(struct guest_pp2_cls_rule_add), msg_len);
		return -EINVAL;
	}

	rule_add = (struct guest_pp2_cls_rule_add *)msg;

	if (rule_add->tbl_id >= MAX_PP2_CLS_TBL) {
		pr_err("invalid table index.\n");
		return -EINVAL;
	}

	if (rule_add->action.action.cos) {
		pr_err("Setting CoS is not supported\n");
		return -EINVAL;
	}

#ifdef PP2_CLS_KEY_MASK_STRING_FORMAT
	ret = build_rule(nmnicpf, &rule,
			 &rule_add->rule,
			 &nmnicpf->pp2.tbl_params[rule_add->tbl_id]);
	if (ret)
		return ret;
#else
	build_rule(nmnicpf, &rule, &rule_add->rule);
#endif /* PP2_CLS_KEY_MASK_STRING_FORMAT */
	ret = pp2_cls_tbl_modify_rule(nmnicpf->pp2.tbl[rule_add->tbl_id],
				      &rule,
				      &rule_add->action.action);
	return ret;
}

int nmnicpf_pp2_cls_rule_remove(struct nmnicpf *nmnicpf,
				void *msg,
				u16 msg_len)
{
	struct guest_pp2_cls_rule_remove *rule_rem;
	struct pp2_cls_tbl_rule rule;
	int ret;

	if (!nmnicpf->pp2.ports_desc || !nmnicpf->pp2.ports_desc[0].ppio)
		return -ENOTSUP;

	if (msg_len < sizeof(struct guest_pp2_cls_rule_remove)) {
		pr_err("invalid msg length. expected %lu, got %u\n",
			sizeof(struct guest_pp2_cls_rule_remove), msg_len);
		return -EINVAL;
	}

	rule_rem = (struct guest_pp2_cls_rule_remove *)msg;

	if (rule_rem->tbl_id >= MAX_PP2_CLS_TBL) {
		pr_err("invalid table index.\n");
		return -EINVAL;
	}

#ifdef PP2_CLS_KEY_MASK_STRING_FORMAT
	ret = build_rule(nmnicpf, &rule,
			 &rule_rem->rule,
			 &nmnicpf->pp2.tbl_params[rule_rem->tbl_id]);
	if (ret)
		return ret;
#else
	build_rule(nmnicpf, &rule, &rule_rem->rule);
#endif /* PP2_CLS_KEY_MASK_STRING_FORMAT */
	ret = pp2_cls_tbl_remove_rule(nmnicpf->pp2.tbl[rule_rem->tbl_id],
				      &rule);
	return ret;
}
