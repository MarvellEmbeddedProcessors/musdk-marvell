/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#include "std_internal.h"
#include "mng/mv_nmp.h"
#include "db.h"
#include "dev_mng.h"
#include "mng/dispatch.h"
#include "mng/lf/pf/pf.h"
#include "config.h"

#define SCHED_MAX_MNG_ELEMENTS		10
#define SCHED_MAX_DATA_ELEMENTS		1000

int nmp_init(struct nmp_params *params, struct nmp **nmp)
{
	int				 k, ret;
	char				 pp2_name[NMP_MAX_BUF_STR_LEN];
	struct pf_profile		*pf_profile;
	struct nmp_lf_params		*lf_params;

	pr_info("Starting %s %s\n", "giu_main", VERSION);

	*nmp = kcalloc(1, sizeof(struct nmp), GFP_KERNEL);
	if (*nmp == NULL) {
		pr_err("Failed to allocate NMP handler\n");
		return -ENOMEM;
	}

	pf_profile = &(*nmp)->nmnicpf.profile_data;

	/*TODO: currently only one container is supported*/
	if (params->num_containers > 1) {
		pr_err("NMP supports only one container in current release\n");
		ret = -EINVAL;
		kfree(*nmp);
		return ret;
	}

	/*TODO: currently only one LF is supported*/
	if (params->containers_params[0].num_lfs > 1) {
		pr_err("NMP supports only one container in current release\n");
		ret = -EINVAL;
		kfree(*nmp);
		return ret;
	}
	lf_params = &params->containers_params[0].lfs_params[0];

	/* TODO - return error once all sync with this change */
	if (!lf_params->u.nicpf.match)
		pr_warn("GPIO match should be given\n");
	else
		strcpy(pf_profile->match, lf_params->u.nicpf.match);
	pf_profile->pci_en = lf_params->u.nicpf.pci_en;
	pf_profile->lcl_egress_q_num   = 1;
	pf_profile->lcl_egress_q_size  = lf_params->u.nicpf.lcl_egress_qs_size;
	pf_profile->lcl_ingress_q_num  = 1;
	pf_profile->lcl_ingress_q_size = lf_params->u.nicpf.lcl_ingress_qs_size;
	pf_profile->lcl_bm_q_num       = lf_params->u.nicpf.lcl_num_bpools;
	if (pf_profile->lcl_bm_q_num > 1) {
		pr_err("NMP supports only one lcl_bpool for GIU in current release\n");
		ret = -EINVAL;
		kfree(*nmp);
		return ret;
	}
	pf_profile->lcl_bm_q_size      = lf_params->u.nicpf.lcl_bpools_params[0].max_num_buffs;
	pf_profile->lcl_bm_buf_size    = lf_params->u.nicpf.lcl_bpools_params[0].buff_size;

	/* pp2 init params */
	(*nmp)->nmpp2.pp2_en = params->pp2_en;

	if (params->pp2_en) {
		(*nmp)->nmpp2.pp2_params.bm_pool_reserved_map = params->pp2_params.bm_pool_reserved_map;
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
	}

	if (params->num_containers) {
		(*nmp)->nmnicpf.guest_id = params->containers_params[0].guest_id;
		(*nmp)->guest_id = params->containers_params[0].guest_id;
	}

	ret = dev_mng_init(*nmp);
	if (ret) {
		pr_err("Management init failed with error %d\n", ret);
		kfree(*nmp);
		exit(ret);
	}

	pr_info("Completed management init\n");

	return 0;
}

int nmp_schedule(struct nmp *nmp, enum nmp_sched_type type)
{
	int ans = 0;

	switch (type) {

	case NMP_SCHED_MNG:
		gie_schedule(nmp->nmnicpf.gie.mng_gie, 0, SCHED_MAX_MNG_ELEMENTS);
		nmdisp_dispatch(nmp->nmdisp);
		/* check for link change */
		nmnicpf_check_link_change(&nmp->nmnicpf);
		gie_schedule(nmp->nmnicpf.gie.mng_gie, 0, SCHED_MAX_MNG_ELEMENTS);
		break;

	case NMP_SCHED_RX:
		ans = gie_schedule(nmp->nmnicpf.gie.rx_gie, 0, SCHED_MAX_DATA_ELEMENTS);
		break;

	case NMP_SCHED_TX:
		ans = gie_schedule(nmp->nmnicpf.gie.tx_gie, 0, SCHED_MAX_DATA_ELEMENTS);
		break;
	}
	return ans;
}

int nmp_create_scheduling_event(struct nmp *nmp, struct nmp_event_params *params, struct mv_sys_event **ev)
{
	return gie_create_event(nmp->nmnicpf.gie.tx_gie, (struct gie_event_params *)params, ev);
}

int nmp_delete_scheduling_event(struct mv_sys_event *ev)
{
	return gie_delete_event(ev);
}

int nmp_set_scheduling_event(struct mv_sys_event *ev, int en)
{
	return gie_set_event(ev, en);
}
