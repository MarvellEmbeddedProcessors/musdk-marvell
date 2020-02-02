/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#include "std_internal.h"
#include "drivers/mv_pp2_bpool.h"
#include "mng/mv_nmp.h"
#include "db.h"
#include "dev_mng.h"
#include "dispatch.h"
#include "lf/lf_mng.h"
#include "lf/pf/pf.h"
#include "config.h"


#define SCHED_MAX_MNG_ELEMENTS		10
#define SCHED_MAX_DATA_ELEMENTS		1000

#define NMP_MAX_BUF_STR_LEN		256


static int nmp_range_validate(int value, int min, int max)
{
	if (((value) > (max)) || ((value) < (min))) {
		pr_err("%s: value 0x%X (%d) is out of range [0x%X , 0x%X].\n",
			__func__, (value), (value), (min), (max));
		return -EINVAL;
	}
	return 0;
}


int nmp_init(struct nmp_params *params, struct nmp **nmp)
{
	int ret;

	pr_debug("Starting %s %s\n", "giu_main", VERSION);

	*nmp = kcalloc(1, sizeof(struct nmp), GFP_KERNEL);
	if (*nmp == NULL) {
		pr_err("Failed to allocate NMP handler\n");
		return -ENOMEM;
	}

	/* pp2 init params */
	(*nmp)->nmpp2.pp2_en = params->pp2_en;

	if (params->pp2_en)
		(*nmp)->nmpp2.bm_pool_reserved_map = params->pp2_params.bm_pool_reserved_map;

	ret = dev_mng_init(*nmp, params);
	if (ret) {
		pr_err("Management init failed with error %d\n", ret);
		kfree(*nmp);
		return ret;
	}

#ifdef DEBUG
	nmdisp_dispatch_dump((*nmp)->nmdisp);
#endif /* DEBUG */

	pr_debug("Completed management init\n");

	return 0;
}

int nmp_schedule(struct nmp *nmp, enum nmp_sched_type type, u16 *pending)
{
	int ans = 0;

	switch (type) {

	case NMP_SCHED_MNG:
		giu_schedule(nmp->giu, GIU_ENG_MNG, 0, SCHED_MAX_MNG_ELEMENTS, pending);
		nmdisp_dispatch(nmp->nmdisp);
		lf_mng_run_maintenance(nmp->lf_mng);
		giu_schedule(nmp->giu, GIU_ENG_MNG, 0, SCHED_MAX_MNG_ELEMENTS, pending);
		break;

	case NMP_SCHED_RX:
		ans = giu_schedule(nmp->giu, GIU_ENG_OUT, 0, SCHED_MAX_DATA_ELEMENTS, pending);
		break;

	case NMP_SCHED_TX:
		ans = giu_schedule(nmp->giu, GIU_ENG_IN, 0, SCHED_MAX_DATA_ELEMENTS, pending);
		break;
	}
	return ans;
}

/* TODO: move this routine to NMP-guest (i.e. should be caled through Qs) */
int nmp_create_scheduling_event(struct nmp *nmp, struct nmp_event_params *params, struct mv_sys_event **ev)
{
	return lf_mng_create_scheduling_event(nmp->lf_mng, params, ev);
}

/* TODO: move this routine to NMP-guest (i.e. should be caled through Qs) */
int nmp_delete_scheduling_event(struct mv_sys_event *ev)
{
	return lf_mng_delete_scheduling_event(ev);
}

/* TODO: move this routine to NMP-guest (i.e. should be caled through Qs) */
int nmp_set_scheduling_event(struct mv_sys_event *ev, int en)
{
	return lf_mng_set_scheduling_event(ev, en);
}

int nmp_read_cfg_file(char *cfg_file, struct nmp_params *params)
{
	u32				 i, j, k, rc;
	char				 buff[SER_MAX_FILE_SIZE];
	char				*sec = NULL;
	char				 tmp_buf[NMP_MAX_BUF_STR_LEN];
	char				 pp2_name[NMP_MAX_BUF_STR_LEN], giu_name[NMP_MAX_NUM_LFS][NMP_MAX_BUF_STR_LEN];
	char				*tmp_str;
	struct nmp_lf_nicpf_params	*pf;
	struct nmp_lf_nicvf_params	*vf;

	/* cfg-file must be provided, read the nmp-config from this location. */
	rc = read_file_to_buf(cfg_file, buff, SER_MAX_FILE_SIZE);
	if (rc) {
		pr_err("nmp config-file (%s) not found!\n", cfg_file);
		return rc;
	}

	/* Check if there are nmp-params */
	sec = strstr(buff, "nmp_params");
	if (!sec) {
		pr_err("nmp_params section not found!\n");
		return -EINVAL;
	}

	memset(params, 0, sizeof(struct nmp_params));

	/* Check if pp2 is enabled */
	json_buffer_to_input(sec, "pp2_en", params->pp2_en);
	if (nmp_range_validate(params->pp2_en, 0, 1) != 0) {
		pr_err("pp2_en not in tange!\n");
		return -EINVAL;
	}

	/* if pp2 enabled, set the pp2_params*/
	if (params->pp2_en) {
		if (strstr(sec, "pp2_params")) {
			sec = strstr(sec, "pp2_params");
			json_buffer_to_input(sec, "bm_pool_reserved_map",
				params->pp2_params.bm_pool_reserved_map);
			if (nmp_range_validate(params->pp2_params.bm_pool_reserved_map,
				0,
				((1 << PP2_BPOOL_NUM_POOLS) - 1))) {
				pr_err("bm_pool_reserved_map not in range!\n");
				rc = -EINVAL;
				goto read_cfg_exit1;
			}
		}
	}

	/* Read number of containers */
	json_buffer_to_input(sec, "num_containers", params->num_containers);
	if (nmp_range_validate(params->num_containers, 1, NMP_MAX_NUM_CONTAINERS)) {
		pr_err("num_containers not in range!\n");
		rc = -EINVAL;
		goto read_cfg_exit1;
	}

	params->containers_params = kcalloc(1, sizeof(struct nmp_container_params) *
					    params->num_containers, GFP_KERNEL);
	if (params->containers_params == NULL) {
		rc = -ENOMEM;
		goto read_cfg_exit1;
	}

	for (i = 0; i < params->num_containers; i++) {
		memset(tmp_buf, 0, sizeof(tmp_buf));
		snprintf(tmp_buf, sizeof(tmp_buf), "containers_params-%d", i);
		sec = strstr(sec, tmp_buf);
		if (!sec) {
			pr_err("'containers_params' not found\n");
			rc = -EINVAL;
			goto read_cfg_exit1;
		}
		/* Read number of lfs */
		json_buffer_to_input(sec, "num_lfs", params->containers_params[i].num_lfs);
		if (nmp_range_validate(params->containers_params[i].num_lfs, 1, NMP_MAX_NUM_LFS) != 0) {
			pr_err("num_lfs not in range!\n");
			rc = -EINVAL;
			goto read_cfg_exit1;
		}

		params->containers_params[i].lfs_params = kcalloc(1, sizeof(struct nmp_lf_params) *
								params->containers_params[i].num_lfs, GFP_KERNEL);
		if (params->containers_params[i].lfs_params == NULL) {
			rc = -ENOMEM;
			goto read_cfg_exit2;
		}

		for (j = 0; j < params->containers_params[i].num_lfs; j++) {
			memset(tmp_buf, 0, sizeof(tmp_buf));
			snprintf(tmp_buf, sizeof(tmp_buf), "lf_params-%d", j);
			sec = strstr(sec, tmp_buf);
			if (!sec) {
				pr_err("'lf_params-%d' not found\n", j);
				rc = -EINVAL;
				goto read_cfg_exit2;
			}

			/* Read lf type*/
			json_buffer_to_input(sec, "lf_type", params->containers_params[i].lfs_params[j].type);
			if (nmp_range_validate(params->containers_params[i].lfs_params[j].type,
					       NMP_LF_T_NIC_NONE, NMP_LF_T_NIC_LAST - 1) != 0) {
				pr_err("lf_type not in range!\n");
				rc = -EINVAL;
				goto read_cfg_exit2;
			}

			if (params->containers_params[i].lfs_params[j].type == NMP_LF_T_NIC_PF) {
				/* Read nicpf*/
				pf = (struct nmp_lf_nicpf_params *)
				     &params->containers_params[i].lfs_params[j].u.nicpf;

				pf->match = &giu_name[j][0];
				json_buffer_to_input_str(sec, "match", pf->match);

				json_buffer_to_input(sec, "keep_alive_thresh", pf->keep_alive_thresh);

				json_buffer_to_input(sec, "pci_en", pf->pci_en);
				if (nmp_range_validate(pf->pci_en, 0, 1) != 0) {
					pr_err("pci_en not in range!\n");
					rc = -EINVAL;
					goto read_cfg_exit2;
				}

				json_buffer_to_input(sec, "sg_en", pf->sg_en);

				json_buffer_to_input(sec, "lcl_egress_qs_size", pf->lcl_egress_qs_size);
				if (!pf->lcl_egress_qs_size) {
					pr_err("missing lcl_egress_qs_size!\n");
					rc = -EINVAL;
					goto read_cfg_exit2;
				}

				json_buffer_to_input(sec, "lcl_ingress_qs_size", pf->lcl_ingress_qs_size);
				if (!pf->lcl_ingress_qs_size) {
					pr_err("missing lcl_ingress_qs_size!\n");
					rc = -EINVAL;
					goto read_cfg_exit2;
				}

				json_buffer_to_input(sec, "dflt_pkt_offset", pf->dflt_pkt_offset);
				if (nmp_range_validate(pf->dflt_pkt_offset, 0, 1024) != 0) {
					pr_err("missing dflt_pkt_offset!\n");
					rc = -EINVAL;
					goto read_cfg_exit2;
				}

				json_buffer_to_input(sec, "max_num_tcs", pf->max_num_tcs);
				if (nmp_range_validate(pf->max_num_tcs, 0, NMP_LF_MAX_NUM_TCS) != 0) {
					pr_err("missing max_num_tcs!\n");
					rc = -EINVAL;
					goto read_cfg_exit2;
				}

				json_buffer_to_input(sec, "lcl_num_bpools", pf->lcl_num_bpools);
				if (nmp_range_validate(pf->lcl_num_bpools, 0, NMP_LF_MAX_NUM_LCL_BPOOLS) != 0) {
					pr_err("missing lcl_num_bpools!\n");
					rc = -EINVAL;
					goto read_cfg_exit2;
				}

				for (k = 0; k < pf->lcl_num_bpools; k++) {
					memset(tmp_buf, 0, sizeof(tmp_buf));
					snprintf(tmp_buf, sizeof(tmp_buf), "lcl_bpools_params-%d", k);
					sec = strstr(sec, tmp_buf);
					if (!sec) {
						pr_err("'lcl_bpools_params' not found\n");
						rc = -EINVAL;
						goto read_cfg_exit2;
					}

					json_buffer_to_input(sec, "max_num_buffs",
							     pf->lcl_bpools_params[k].max_num_buffs);
					if (!pf->lcl_bpools_params[k].max_num_buffs) {
						pr_err("missing max_num_buffs!\n");
						rc = -EINVAL;
						goto read_cfg_exit2;
					}

					json_buffer_to_input(sec, "buff_size", pf->lcl_bpools_params[k].buff_size);
					if (!pf->lcl_bpools_params[k].buff_size) {
						pr_err("missing buff_size!\n");
						rc = -EINVAL;
						goto read_cfg_exit2;
					}
				}

				json_buffer_to_input(sec, "nicpf_type", pf->type);
				if (nmp_range_validate(pf->type, NMP_LF_NICPF_T_NONE,
						       NMP_LF_NICPF_T_LAST - 1) != 0) {
					pr_err("nicpf_type not in range!\n");
					rc = -EINVAL;
					goto read_cfg_exit2;
				}

				if (pf->type != NMP_LF_NICPF_T_PP2_PORT)
					continue;

				sec = strstr(sec, "port-params-pp2-port");
				if (!sec) {
					pr_err("'port-params-pp2-port' not found\n");
					return -EINVAL;
				}

				pf->port_params.pp2_port.match = pp2_name;
				json_buffer_to_input_str(sec, "match", pf->port_params.pp2_port.match);
				if (!pf->port_params.pp2_port.match) {
					pr_err("'pp2 match' not found\n");
					return -EINVAL;
				}

				json_buffer_to_input(sec, "lcl_num_bpools", pf->port_params.pp2_port.lcl_num_bpools);
				if (nmp_range_validate(pf->port_params.pp2_port.lcl_num_bpools,
						       1, NMP_LF_MAX_NUM_LCL_BPOOLS) != 0)
					return -EINVAL;

				for (k = 0; k < pf->port_params.pp2_port.lcl_num_bpools; k++) {
					struct nmp_lf_bpool_params *lcl_bpools_params =
						&pf->port_params.pp2_port.lcl_bpools_params[k];

					memset(tmp_buf, 0, sizeof(tmp_buf));
					snprintf(tmp_buf, sizeof(tmp_buf), "lcl_bpools_params-%d", k);
					sec = strstr(sec, tmp_buf);
					if (!sec) {
						pr_err("'lcl_bpools_params' not found\n");
						return -EINVAL;
					}

					json_buffer_to_input(sec, "max_num_buffs",
							lcl_bpools_params->max_num_buffs);
					if (nmp_range_validate(pf->port_params.pp2_port.lcl_num_bpools,
							       0, 4096) != 0) {
						return -EINVAL;
					}

					json_buffer_to_input(sec, "buff_size", lcl_bpools_params->buff_size);
					if (nmp_range_validate(lcl_bpools_params->buff_size, 0, 4096) != 0)
						return -EINVAL;
				}
			} else if (params->containers_params[i].lfs_params[j].type == NMP_LF_T_NIC_VF) {
				/* Read nicvf*/
				vf = (struct nmp_lf_nicvf_params *)
				     &params->containers_params[i].lfs_params[j].u.nicvf;

				vf->match = &giu_name[j][0];
				json_buffer_to_input_str(sec, "match", vf->match);

				json_buffer_to_input(sec, "keep_alive_thresh", vf->keep_alive_thresh);

				json_buffer_to_input(sec, "pci_en", vf->pci_en);
				if (nmp_range_validate(vf->pci_en, 0, 1) != 0) {
					pr_err("pci_en not in range!\n");
					rc = -EINVAL;
					goto read_cfg_exit2;
				}

				json_buffer_to_input(sec, "sg_en", vf->sg_en);

				json_buffer_to_input(sec, "lcl_egress_qs_size", vf->lcl_egress_qs_size);
				if (!vf->lcl_egress_qs_size) {
					pr_err("missing lcl_egress_qs_size!\n");
					rc = -EINVAL;
					goto read_cfg_exit2;
				}

				json_buffer_to_input(sec, "lcl_ingress_qs_size", vf->lcl_ingress_qs_size);
				if (!vf->lcl_ingress_qs_size) {
					pr_err("missing lcl_ingress_qs_size!\n");
					rc = -EINVAL;
					goto read_cfg_exit2;
				}

				json_buffer_to_input(sec, "dflt_pkt_offset", vf->dflt_pkt_offset);
				if (nmp_range_validate(vf->dflt_pkt_offset, 0, 1024) != 0) {
					pr_err("missing dflt_pkt_offset!\n");
					rc = -EINVAL;
					goto read_cfg_exit2;
				}

				json_buffer_to_input(sec, "max_num_tcs", vf->max_num_tcs);
				if (nmp_range_validate(vf->max_num_tcs, 0, NMP_LF_MAX_NUM_TCS) != 0) {
					pr_err("missing max_num_tcs!\n");
					rc = -EINVAL;
					goto read_cfg_exit2;
				}

				json_buffer_to_input(sec, "lcl_num_bpools", vf->lcl_num_bpools);
				if (nmp_range_validate(vf->lcl_num_bpools, 0, NMP_LF_MAX_NUM_LCL_BPOOLS) != 0) {
					pr_err("missing lcl_num_bpools!\n");
					rc = -EINVAL;
					goto read_cfg_exit2;
				}

				for (k = 0; k < vf->lcl_num_bpools; k++) {
					memset(tmp_buf, 0, sizeof(tmp_buf));
					snprintf(tmp_buf, sizeof(tmp_buf), "lcl_bpools_params-%d", k);
					sec = strstr(sec, tmp_buf);
					if (!sec) {
						pr_err("'lcl_bpools_params' not found\n");
						rc = -EINVAL;
						goto read_cfg_exit2;
					}

					json_buffer_to_input(sec, "max_num_buffs",
							     vf->lcl_bpools_params[k].max_num_buffs);
					if (!vf->lcl_bpools_params[k].max_num_buffs) {
						pr_err("missing max_num_buffs!\n");
						rc = -EINVAL;
						goto read_cfg_exit2;
					}

					json_buffer_to_input(sec, "buff_size", vf->lcl_bpools_params[k].buff_size);
					if (!vf->lcl_bpools_params[k].buff_size) {
						pr_err("missing buff_size!\n");
						rc = -EINVAL;
						goto read_cfg_exit2;
					}
				}
			}
		}

		json_buffer_to_input(sec, "guest_id", params->containers_params[i].guest_id);
		if (nmp_range_validate(params->containers_params[i].guest_id, 0, 10) != 0) {
			rc = -EINVAL;
			goto read_cfg_exit2;
		}
	}

	/* Check if the optional section 'dma_engine' exists */
	sec = strstr(sec, "dma_engines");
	if (!sec) {
		pr_err("'dma_engines' section must be set\n");
		rc = -EINVAL;
		goto read_cfg_exit2;
	}
	tmp_str = tmp_buf;
	memset(tmp_buf, 0, sizeof(tmp_buf));

	json_buffer_to_input_str(sec, "mng", tmp_str);
	if (tmp_buf[0] == 0) {
		pr_err("'dma_engines' section: 'mng' engine not found\n");
		rc = -EINVAL;
		goto read_cfg_exit2;
	}
	if (!strstr(tmp_str, "dmax2-")) {
		pr_err("dma engine name should start with 'dmax2-'\n");
		rc = -EINVAL;
		goto read_cfg_exit2;
	}
	strcpy(params->dma_eng_params.engine_name[GIU_ENG_MNG], tmp_str);

	memset(tmp_buf, 0, sizeof(tmp_buf));
	json_buffer_to_input_str(sec, "in", tmp_str);
	if (tmp_buf[0] == 0) {
		pr_err("'dma_engines' section: 'in' engine not found\n");
		rc = -EINVAL;
		goto read_cfg_exit2;
	}
	if (!strstr(tmp_str, "dmax2-")) {
		pr_err("dma engine name should start with 'dmax2-'\n");
		rc = -EINVAL;
		goto read_cfg_exit2;
	}
	strcpy(params->dma_eng_params.engine_name[GIU_ENG_IN], tmp_str);

	memset(tmp_buf, 0, sizeof(tmp_buf));
	json_buffer_to_input_str(sec, "out", tmp_str);
	if (tmp_buf[0] == 0) {
		pr_err("'dma_engines' section: 'out' engine not found\n");
		rc = -EINVAL;
		goto read_cfg_exit2;
	}
	if (!strstr(tmp_str, "dmax2-")) {
		pr_err("dma engine name should start with 'dmax2-'\n");
		rc = -EINVAL;
		goto read_cfg_exit2;
	}
	strcpy(params->dma_eng_params.engine_name[GIU_ENG_OUT], tmp_str);
	params->dma_eng_params.num_engines = 3;

	return 0;
read_cfg_exit2:
	for (i = 0; i < params->num_containers; i++)
		kfree(params->containers_params[i].lfs_params);
read_cfg_exit1:
	kfree(params->containers_params);
	return rc;
}

