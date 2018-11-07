/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#define log_fmt(fmt) "lf_mng " fmt

#include "std_internal.h"
#include "env/trace/trc_pf.h"
#include "mng/dispatch.h"
#include "mng/dev_mng.h"
#include "lf_mng.h"
#include "pf/pf.h"
#include "custom/custom.h"


#define LF_MNG_MAX_NUM_CONTAINERS	2
#define LF_MNG_MAX_NUM_CONTAINER_LFS	8

#define SER_FILE_VAR_DIR		"/var/"
#define SER_FILE_NAME_PREFIX		"musdk-serial-cfg"
#define SER_MAX_FILE_NAME		64
#define SER_MAX_FILE_SIZE		(30 * 1024)


struct lf_mng_lf {
	u8			 id;
	enum nmp_lf_type	 type;
	union {
		struct nmnicpf	nmnicpf;
	} u;
};

struct lf_mng_container {
	u8			 guest_id;

	u8			 num_lfs;

	struct lf_mng_lf	 lfs[LF_MNG_MAX_NUM_CONTAINER_LFS];

	struct nmcstm		*nmcstm;
};

struct lf_mng {
	struct nmp		*nmp;
	struct nmdisp		*nmdisp;
	struct mqa		*mqa;
	struct giu		*giu;

	u8			 num_containers;
	struct lf_mng_container	 containers[LF_MNG_MAX_NUM_CONTAINERS];

	u8			 total_num_lfs;
};


static int init_nicpf_params(struct nmnicpf *nmnicpf, struct nmp_lf_nicpf_params *params)
{
	struct pf_profile		*pf_profile;
	int				 k;

	pf_profile = &(nmnicpf->profile_data);

	/* TODO - return error once all sync with this change */
	if (!params->match)
		pr_warn("GPIO match should be given\n");
	else
		strcpy(pf_profile->match, params->match);
	pf_profile->pci_en = params->pci_en;
	pf_profile->max_num_tcs        = params->max_num_tcs;
	pf_profile->lcl_egress_q_num   = 1;
	pf_profile->lcl_egress_q_size  = params->lcl_egress_qs_size;
	pf_profile->lcl_ingress_q_num  = 1;
	pf_profile->lcl_ingress_q_size = params->lcl_ingress_qs_size;
	pf_profile->lcl_bm_q_num       = params->lcl_num_bpools;
	if (pf_profile->lcl_bm_q_num > 1) {
		pr_err("NMP supports only one lcl_bpool for GIU in current release\n");
		return -EINVAL;
	}
	pf_profile->lcl_bm_q_size      = params->lcl_bpools_params[0].max_num_buffs;
	pf_profile->lcl_bm_buf_size    = params->lcl_bpools_params[0].buff_size;

	pf_profile->keep_alive_thresh = params->keep_alive_thresh;

	pf_profile->port_type = params->type;
	if (pf_profile->port_type == NMP_LF_NICPF_T_PP2_PORT) {
		pf_profile->dflt_pkt_offset = params->dflt_pkt_offset;
		strcpy(pf_profile->port_match, params->port_params.pp2_port.match);
		pf_profile->pp2_port.match = pf_profile->port_match;
		pf_profile->pp2_port.lcl_num_bpools = params->port_params.pp2_port.lcl_num_bpools;
		for (k = 0; k < pf_profile->pp2_port.lcl_num_bpools; k++) {
			pf_profile->pp2_port.lcl_bpools_params[k].buff_size =
				params->port_params.pp2_port.lcl_bpools_params[k].buff_size;
			pf_profile->pp2_port.lcl_bpools_params[k].max_num_buffs =
				params->port_params.pp2_port.lcl_bpools_params[k].max_num_buffs;
		}
	}

	return 0;
}

static int lookup_lf(struct lf_mng *lf_mng, u8 lf_id, u8 *cntr, u8 *lf)
{
	int			 _cntr, _lf;

	for (_cntr = 0; _cntr < lf_mng->num_containers; _cntr++)
		for (_lf = 0; _lf < lf_mng->containers[_cntr].num_lfs; _lf++)
			if (lf_mng->containers[_cntr].lfs[_lf].id == lf_id) {
				*cntr = _cntr;
				*lf = _lf;
				return 0;
			}

	return -ENOENT;
}

static int lf_init_done(void *arg, u8 lf_id)
{
	struct lf_mng			*lf_mng = (struct lf_mng *)arg;
	struct mv_sys_dma_mem_info	 mem_info;
	char	 file_name[SER_MAX_FILE_NAME];
	char	 buff[SER_MAX_FILE_SIZE];
	u32	 size = SER_MAX_FILE_SIZE;
	char	 dev_name[100];
	size_t	 pos = 0;
	u8	 cntr, lf, guest_id;
	int	 ret;

	if (!lf_mng) {
		pr_err("no LF-MNG obj!\n");
		return -EFAULT;
	}

	pr_debug("nmp_pf_init_done reached\n");

	ret = lookup_lf(lf_mng, lf_id, &cntr, &lf);
	if (ret) {
		pr_err("LF %d not found!\n", lf_id);
		return -EIO;
	}

	if ((lf_mng->containers[cntr].lfs[lf].type != NMP_LF_T_NIC_PF) ||
		(lf_mng->containers[cntr].lfs[lf].id != 0)) {
		pr_err("temporary only single NICPF is supported!\n");
		return -EIO;
	}

	guest_id = lf_mng->containers[cntr].lfs[lf].u.nmnicpf.guest_id;

	/* TODO: go over all guests */
	snprintf(file_name, sizeof(file_name), "%s%s%d", SER_FILE_VAR_DIR, SER_FILE_NAME_PREFIX, guest_id);
	/* Remove the serialize files */
	remove(file_name);

	memset(buff, 0, size);

	json_print_to_buffer(buff, size, 0, "{\n");

	/* Serialize the DMA info */
	mem_info.name = dev_name;
	mv_sys_dma_mem_get_info(&mem_info);
	json_print_to_buffer(buff, size, 1, "\"dma-info\": {\n");
	json_print_to_buffer(buff, size, 2, "\"file_name\": \"%s\",\n", mem_info.name);
	json_print_to_buffer(buff, size, 2, "\"region_size\": %zu,\n", mem_info.size);
	json_print_to_buffer(buff, size, 2, "\"phys_addr\": 0x%lx\n", (u64)mem_info.paddr);
	json_print_to_buffer(buff, size, 1, "},\n");

	pr_debug("starting serialization of guest %d\n", guest_id);

	ret = nmnicpf_serialize(&lf_mng->containers[cntr].lfs[lf].u.nmnicpf, &buff[pos], size - pos);
	if (ret >= 0)
		pos += ret;
	if (pos != strlen(buff)) {
		pr_err("found mismatch between pos (%zu) and buff len (%zu)\n", pos, strlen(buff));
		return -EFAULT;
	}

	if (lf_mng->containers[cntr].nmcstm) {
		ret = nmcstm_serialize(lf_mng->containers[cntr].nmcstm, &buff[pos], size - pos);
		if (ret >= 0)
			pos += ret;
		if (pos != strlen(buff)) {
			pr_err("found mismatch between pos (%zu) and buff len (%zu)\n", pos, strlen(buff));
			return -EFAULT;
		}
	}

	json_print_to_buffer(buff, size, 0, "}\n");

	/* write buffer to file */
	ret = write_buf_to_file(file_name, buff, strlen(buff));
	if (ret) {
		pr_err("Failed to write to guest %d file\n", guest_id);
		return ret;
	}

	sync();
	return 0;
}


int lf_mng_init(struct lf_mng_params *params, struct lf_mng **lf_mng)
{
	struct lf_mng		*_lf_mng;
	struct nmcstm_params	 nmcstm_params;
	int			 err, cntr, lf;

	/*TODO: currently only one container is supported*/
	if (params->num_containers > 1) {
		pr_err("NMP supports only one container in current release\n");
		return -EINVAL;
	}
	/*TODO: currently only one LF is supported*/
	if (params->containers_params[0].num_lfs > 1) {
		pr_err("NMP supports only one container in current release\n");
		return -EINVAL;
	}

	_lf_mng = kmalloc(sizeof(*_lf_mng), GFP_KERNEL);
	if (!_lf_mng)
		return -ENOMEM;

	_lf_mng->nmp = params->nmp;
	_lf_mng->nmdisp = params->nmdisp;
	_lf_mng->mqa = params->mqa;
	_lf_mng->giu = params->giu;

	_lf_mng->num_containers = params->num_containers;
	for (cntr = 0; cntr < _lf_mng->num_containers; cntr++) {
		_lf_mng->containers[cntr].guest_id = params->containers_params[cntr].guest_id;
		_lf_mng->containers[cntr].num_lfs = params->containers_params[cntr].num_lfs;
		for (lf = 0; lf < _lf_mng->containers[cntr].num_lfs; lf++) {
			_lf_mng->containers[cntr].lfs[lf].type =
				params->containers_params[cntr].lfs_params[lf].type;
			if (_lf_mng->containers[cntr].lfs[lf].type == NMP_LF_T_NIC_PF) {
				struct nmnicpf *nmnicpf = &(_lf_mng->containers[cntr].lfs[lf].u.nmnicpf);

				nmnicpf->nmlf.id = _lf_mng->total_num_lfs++;
				nmnicpf->pf_id = 0;

				nmnicpf->nmp = _lf_mng->nmp;
				nmnicpf->mqa = _lf_mng->mqa;
				nmnicpf->giu = _lf_mng->giu;

				nmnicpf->guest_id = _lf_mng->containers[cntr].guest_id;

				/* Save reference to Dispatcher in PF */
				nmnicpf->nmdisp = _lf_mng->nmdisp;

				/* Assign the pf_init_done callback */
				nmnicpf->f_ready_cb = lf_init_done;
				nmnicpf->arg = _lf_mng;

				err = init_nicpf_params(nmnicpf,
					&(params->containers_params[cntr].lfs_params[lf].u.nicpf));
				if (err)
					return err;

				/* TODO: init nmp-nicpf only in case it is supported in the nmp_params! */
				err = nmnicpf_init(nmnicpf);
				if (err)
					return err;
			} else {
				pr_err("LF type %d not supported!\n", _lf_mng->containers[cntr].lfs[lf].type);
				return -EINVAL;
			}
		}

		/* TODO: init nmp-customer only in case it is supported in the nmp_params! */
		memset(&nmcstm_params, 0, sizeof(nmcstm_params));
		nmcstm_params.id = _lf_mng->containers[cntr].guest_id;
		nmcstm_params.mqa = _lf_mng->mqa;
		nmcstm_params.nmdisp = _lf_mng->nmdisp;
		nmcstm_params.pf_id = _lf_mng->containers[cntr].lfs[0].u.nmnicpf.pf_id;
		err = nmcstm_init(&nmcstm_params, &(_lf_mng->containers[cntr].nmcstm));
		if (err) {
			pr_err("Custom init failed\n");
			return err;
		}
	}

	*lf_mng = _lf_mng;
	return 0;
}

void lf_mng_deinit(struct lf_mng *lf_mng)
{
	int	 cntr, lf;

	if (!lf_mng)
		return;


	for (cntr = 0; cntr < lf_mng->num_containers; cntr++) {
		for (lf = 0; lf < lf_mng->containers[cntr].num_lfs; lf++)
			if (lf_mng->containers[cntr].lfs[lf].type == NMP_LF_T_NIC_PF)
				nmnicpf_deinit(&(lf_mng->containers[cntr].lfs[lf].u.nmnicpf));
			else
				pr_warn("LF type %d not supported!\n", lf_mng->containers[cntr].lfs[lf].type);
		nmcstm_deinit(lf_mng->containers[cntr].nmcstm);
	}

	kfree(lf_mng);
}

int lf_mng_run_maintenance(struct lf_mng *lf_mng)
{
	u8 cntr = 0; /* TODO: need to iterate all NICPFs! */
	u8 lf = 0; /* TODO: need to iterate all NICPFs! */

	return lf_mng->containers[cntr].lfs[lf].u.nmnicpf.nmlf.f_maintenance_cb(
		&lf_mng->containers[cntr].lfs[lf].u.nmnicpf.nmlf);
}

int lf_mng_create_scheduling_event(struct lf_mng *lf_mng,
	struct nmp_event_params *params,
	struct mv_sys_event **ev)
{
	u8 cntr = 0; /* TODO: need to iterate all NICPFs! */
	u8 lf = 0; /* TODO: need to iterate all NICPFs! */

	return giu_gpio_create_event(lf_mng->containers[cntr].lfs[lf].u.nmnicpf.giu_gpio,
		(struct giu_gpio_event_params *)params,
		ev);
}

int lf_mng_delete_scheduling_event(struct mv_sys_event *ev)
{
	return giu_gpio_delete_event(ev);
}

/* TODO: move this routine to NMP-guest (i.e. should be caled through Qs) */
int lf_mng_set_scheduling_event(struct mv_sys_event *ev, int en)
{
	return giu_gpio_set_event(ev, en);
}
