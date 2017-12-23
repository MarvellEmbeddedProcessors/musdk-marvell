/******************************************************************************
 *	Copyright (C) 2017 Marvell International Ltd.
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
#include "mng/mv_nmp.h"
#include "db.h"
#include "dev_mng.h"
#include "mng/dispatch.h"
#include "config.h"

#define NMP_MAX_BUF_STR_LEN		256

int nmp_init(struct nmp_params *params, struct nmp **nmp)
{
	int				 k, ret;
	char				 pp2_name[NMP_MAX_BUF_STR_LEN];
	struct pf_profile		*pf_profile;
	struct nmp_lf_params_new	*lf_params;

	pr_info("Starting %s %s\n", "giu_main", VERSION);

	*nmp = kcalloc(1, sizeof(struct nmp), GFP_KERNEL);
	if (*nmp == NULL) {
		pr_err("Failed to allocate NMP handler\n");
		return -ENOMEM;
	}

	pf_profile = &(*nmp)->nic_pf.profile_data;

	/* TODO: remove this API from nmp_params. Left for backwards compatibility */
	/* GIU params should be retrieved from params->containers_params[0].lfs_params[0] */
	pf_profile->lcl_egress_q_num   = params->lfs_params->pf.lcl_egress_q_num;
	pf_profile->lcl_egress_q_size  = params->lfs_params->pf.lcl_egress_q_size;
	pf_profile->lcl_ingress_q_num  = params->lfs_params->pf.lcl_ingress_q_num;
	pf_profile->lcl_ingress_q_size = params->lfs_params->pf.lcl_ingress_q_size;
	pf_profile->lcl_bm_q_num       = params->lfs_params->pf.lcl_bm_q_num;
	pf_profile->lcl_bm_q_size      = params->lfs_params->pf.lcl_bm_q_size;
	pf_profile->lcl_bm_buf_size    = params->lfs_params->pf.lcl_bm_buf_size;

	/* pp2 init params */
	(*nmp)->nmpp2.pp2_en = params->pp2_en;
	(*nmp)->nmpp2.pp2_params.bm_pool_reserved_map = params->pp2_params.bm_pool_reserved_map;

	/*TODO: currently only one container is supported*/
	if (params->num_containers > 1) {
		pr_err("NMP supports only one container in current release\n");
		ret = -EINVAL;
		kfree(nmp);
		return ret;
	}

	/*TODO: currently only one LF is supported*/
	if (params->containers_params[0].num_lfs > 1) {
		pr_err("NMP supports only one container in current release\n");
		ret = -EINVAL;
		kfree(nmp);
		return ret;
	}

	lf_params = &params->containers_params[0].lfs_params[0];

	pf_profile->pp2_bm_pool_reserved_map = params->pp2_params.bm_pool_reserved_map;
	pf_profile->dflt_pkt_offset = lf_params->u.nicpf.dflt_pkt_offset;
	pf_profile->max_num_tcs = lf_params->u.nicpf.max_num_tcs;
	pf_profile->port_type = lf_params->u.nicpf.type;
	strcpy(pp2_name, lf_params->u.nicpf.port_params.pp2_port.match);
	pf_profile->pp2_port.match = pp2_name;
	pf_profile->pp2_port.lcl_num_bpools = lf_params->u.nicpf.port_params.pp2_port.lcl_num_bpools;
	for (k = 0; k < pf_profile->pp2_port.lcl_num_bpools; k++) {
		pf_profile->pp2_port.lcl_bpools_params[k].buff_size =
			lf_params->u.nicpf.port_params.pp2_port.lcl_bpools_params[k].buff_size;
		pf_profile->pp2_port.lcl_bpools_params[k].max_num_buffs =
			lf_params->u.nicpf.port_params.pp2_port.lcl_bpools_params[k].max_num_buffs;
	}

	(*nmp)->nic_pf.guest_id = params->containers_params[0].guest_id;

	ret = dev_mng_init(*nmp);
	if (ret) {
		pr_err("Management init failed with error %d\n", ret);
		kfree(nmp);
		exit(ret);
	}

	pr_info("Completed management init\n");

	return 0;
}

int nmp_schedule(struct nmp *nmp, enum nmp_sched_type type)
{
	switch (type) {

	case NMP_SCHED_MNG:
		gie_schedule(nmp->nic_pf.gie.mng_gie, 0, 1);
		nmdisp_dispatch(nmp->nmdisp);
		break;

	case NMP_SCHED_RX:
		gie_schedule(nmp->nic_pf.gie.rx_gie, 0, 1);
		break;

	case NMP_SCHED_TX:
		gie_schedule(nmp->nic_pf.gie.tx_gie, 0, 1);
		break;
	}
	return 0;
}

