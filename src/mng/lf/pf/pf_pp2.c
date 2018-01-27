/******************************************************************************
 *	Copyright (C) 2018 Marvell International Ltd.
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

#define log_fmt(fmt) "pf-pp2: " fmt

#include "std_internal.h"
#include "mng/db.h"
#include "mng/lf/mng_cmd_desc.h"
#include "pf_pp2.h"
#include "src/drivers/ppv2/pp2.h"

/* Maximum size of port name */
#define NMP_PPIO_NAME_MAX			20

/* Maximum number of packet processors used by NMP */
#define NMP_PP2_MAX_PKT_PROC			2

/* sysfs path for reading relevant parameters from kernel driver */
#define NMP_PP2_SYSFS_MUSDK_PATH		"/sys/devices/platform/pp2/musdk"
#define NMP_PP2_SYSFS_DEBUG_PORT_SET_FILE	"sysfs_current_port"
#define NMP_PP2_SYSFS_RX_FIRST_RXQ_FILE		"first_rxq"
#define NMP_PP2_SYSFS_RX_NUM_RXQ_FILE		"num_rx_queues"
#define NMP_PP2_SYSFS_TX_NUM_TXQ_FILE		"num_tx_queues"

static u16 used_bpools[NMP_PP2_MAX_PKT_PROC];

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
	nmnicpf->pp2.reserved_bpools = nmnicpf->profile_data.pp2_bm_pool_reserved_map;

	/* Allocate memory for the pp2 descriptors */
	pdesc = kmalloc(sizeof(struct nmp_pp2_port_desc), GFP_KERNEL);
	if (!pdesc)
		return -ENOMEM;

	nmnicpf->pp2.ports_desc = pdesc;
	pdesc->num_pools = pf_pp2_profile->lcl_num_bpools;
	pr_info("Number of pools %d\n", pdesc->num_pools);
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
	pr_info("pdesc pkt_offset: %d\n", pdesc->pkt_offst);
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


static void nmnicpf_pp2_set_reserved_bpools(u32 reserved_bpool_map)
{
	int i;

	for (i = 0; i < NMP_PP2_MAX_PKT_PROC; i++)
		used_bpools[i] = reserved_bpool_map;
}

static int nmnicpf_pp2_find_free_bpool(u32 pp_id)
{
	int i;

	for (i = 0; i < PP2_BPOOL_NUM_POOLS; i++) {
		if (!((1 << i) & used_bpools[pp_id])) {
			used_bpools[pp_id] |= (1 << i);
			break;
		}
	}
	if (i == PP2_BPOOL_NUM_POOLS) {
		pr_err("no free BPool found!\n");
		return -ENOSPC;
	}
	return i;
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

	if (!nmnicpf->pp2.reserved_bpools)
		/* No pp2 initialized, just return */
		return 0;

	nmnicpf_pp2_set_reserved_bpools(nmnicpf->pp2.reserved_bpools);

	pdesc = (struct nmp_pp2_port_desc *)&nmnicpf->pp2.ports_desc[pcount];

	for (i = 0; i < pdesc->num_pools; i++) {
		pool_id = nmnicpf_pp2_find_free_bpool(pdesc->pp_id);
		if (pool_id < 0) {
			pr_err("free bpool not found!\n");
			return pool_id;
		}
		memset(name, 0, sizeof(name));
		snprintf(name, sizeof(name), "pool-%d:%d", pdesc->pp_id, pool_id);
		memset(&bpool_params, 0, sizeof(bpool_params));
		bpool_params.match = name;
		bpool_params.buff_len = pdesc->pools_desc[i].buff_size;
		pr_info("%s: buff_size %d, num_buffs %d\n", name,
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
	struct pp2_ppio_inq_params	 inq_params[PP2_HW_PORT_NUM_RXQS];
	char				 name[NMP_PPIO_NAME_MAX];
	int				 i, j, err = 0;
	u32				 pcount = 0;
	struct nmp_pp2_port_desc	*pdesc;
	int				 num_pools;
	struct nmp_pp2_bpool_desc	*pools;
	eth_addr_t			addr = INITIAL_MAC_ADDR;

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
	for (i = 0; i < pdesc->num_outqs; i++)
		port_params.outqs_params.outqs_params[i].size = pdesc->outq_size;

	err = pp2_ppio_init(&port_params, &pdesc->ppio);
	if (err) {
		pr_err("PP-IO init failed (error: %d)!\n", err);
		return err;
	}

	if (!pdesc->ppio) {
		pr_err("PP-IO init failed!\n");
		return -EIO;
	}

	err = pp2_ppio_set_mac_addr(pdesc->ppio, addr);
	if (err) {
		pr_err("PPIO set mac address failed\n");
		return err;
	}

	err = pp2_ppio_enable(pdesc->ppio);
	if (err) {
		pr_err("PPIO enable failed\n");
		return err;
	}

	return 0;
}

int nmnicpf_pp2_get_statistics(struct nmnicpf *nmnicpf,
			       struct mgmt_cmd_params *params,
			       struct mgmt_cmd_resp *resp_data)
{
	struct				 pp2_ppio_statistics stats;
	u32				 pcount = 0;
	struct nmp_pp2_port_desc	*port_desc = NULL;
	int				 ret;

	port_desc = (struct nmp_pp2_port_desc *)&nmnicpf->pp2.ports_desc[pcount];

	ret = pp2_ppio_get_statistics(port_desc->ppio, &stats, params->pf_get_statistics.reset);
	if (ret) {
		resp_data->status = NOTIF_STATUS_FAIL;
		return ret;
	}

	resp_data->status = NOTIF_STATUS_OK;
	resp_data->agnic_stats.rx_bytes = stats.rx_bytes;
	resp_data->agnic_stats.rx_packets = stats.rx_packets;
	resp_data->agnic_stats.rx_unicast_packets = stats.rx_unicast_packets;
	resp_data->agnic_stats.rx_errors = stats.rx_errors;
	resp_data->agnic_stats.rx_fullq_dropped = stats.rx_fullq_dropped;
	resp_data->agnic_stats.rx_bm_dropped = stats.rx_bm_dropped;
	resp_data->agnic_stats.rx_early_dropped = stats.rx_early_dropped;
	resp_data->agnic_stats.rx_fifo_dropped = stats.rx_fifo_dropped;
	resp_data->agnic_stats.rx_cls_dropped = stats.rx_cls_dropped;
	resp_data->agnic_stats.tx_bytes = stats.tx_bytes;
	resp_data->agnic_stats.tx_packets = stats.tx_packets;
	resp_data->agnic_stats.tx_unicast_packets = stats.tx_unicast_packets;
	resp_data->agnic_stats.tx_errors = stats.tx_errors;

	return 0;
}

