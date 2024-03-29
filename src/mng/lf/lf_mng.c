/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#define log_fmt(fmt, ...) "lf_mng: " fmt, ##__VA_ARGS__

#include "std_internal.h"
#include "env/trace/trc_pf.h"
#include "mng/dispatch.h"
#include "mng/dev_mng.h"
#include "mng/dev_mng_pp2.h"
#include "pf/pf.h"
#include "vf/vf.h"
#include "custom/custom.h"
#include "lf_mng.h"

#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#define LF_MNG_MAX_NUM_CONTAINERS	NMP_MAX_NUM_CONTAINERS
#define LF_MNG_MAX_NUM_CONTAINER_LFS	NMP_MAX_NUM_LFS

#define SER_FILE_VAR_DIR		"/tmp/"
#define SER_FILE_NAME_PREFIX		"musdk-serial-cfg"
#define SER_MAX_FILE_NAME		64
#define SER_MAX_FILE_SIZE		(30 * 1024)

struct lf_mng_lf {
	u8			 id;
	enum nmp_lf_type	 type;
	union {
		struct nmnicpf	*nmnicpf;
		struct nmnicvf	*nmnicvf;
	} u;
};

struct iomem_inf {
	u64	phys_addr;
	u64	virt_addr;
};

struct lf_mng_container {
	struct lf_mng		*lf_mng;
	u8			 guest_id;
	u8			 master_id;

	u8			 num_lfs;
	u8			 num_lfs_done; /* only when all LFs finished their initialization,
						* serialize can happaned.
						*/

	struct lf_mng_lf	 lfs[LF_MNG_MAX_NUM_CONTAINER_LFS];

	struct iomem_inf	 vf_bar0_map;
	struct iomem_inf	 vf_bar2_map;
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
	u8			 total_num_nicpfs;
	u8			 total_num_nicvfs;
};

static int lookup_lf(struct lf_mng_container *lf_mng_cont, u8 lf_id, struct lf_mng_lf **lf)
{
	int	_lf;

	for (_lf = 0; _lf < lf_mng_cont->num_lfs; _lf++)
		if (lf_mng_cont->lfs[_lf].id == lf_id) {
			*lf = &lf_mng_cont->lfs[_lf];
			return 0;
		}

	return -ENOENT;
}

static int lf_get_free_bar(void *arg, void **va, dma_addr_t *pa)
{
	struct lf_mng	*lf_mng = ((struct lf_mng_container *)arg)->lf_mng;

	if (!lf_mng) {
		pr_err("no LF-MNG obj!\n");
		return -EFAULT;
	}

	return dev_mng_get_free_bar(lf_mng->nmp, va, pa);
}

static void lf_put_bar(void *arg, int index)
{
	struct lf_mng	*lf_mng = ((struct lf_mng_container *)arg)->lf_mng;

	if (!lf_mng) {
		pr_err("no LF-MNG obj!\n");
		return;
	}

	dev_mng_put_bar(lf_mng->nmp, index);
}

static int lf_pp_find_free_bpool(void *arg, u32 pp_id)
{
	struct lf_mng	*lf_mng = ((struct lf_mng_container *)arg)->lf_mng;

	if (!lf_mng) {
		pr_err("no LF-MNG obj!\n");
		return -EFAULT;
	}

	return dev_mng_pp2_find_free_bpool(lf_mng->nmp, pp_id);
}

static int lf_set_vf_bar_offset_base(void *arg, u8 bar, u64 phys_addr, u64 virt_addr)
{
	struct lf_mng_container	*lf_cont = (struct lf_mng_container *)arg;

	if (!lf_cont) {
		pr_err("no LF-CONTAINER obj!\n");
		return -EFAULT;
	}

	if (bar == 0) {
		lf_cont->vf_bar0_map.phys_addr = phys_addr;
		lf_cont->vf_bar0_map.virt_addr = virt_addr;
	} else if (bar == 2) {
		lf_cont->vf_bar2_map.phys_addr = phys_addr;
		lf_cont->vf_bar2_map.virt_addr = virt_addr;
	} else {
		pr_err("bar %d in invalid!\n", bar);
		return -EINVAL;
	}

	pr_info("PF set BAR#%d:\n"
		"\tVF phys offset base to 0x%" PRIx64 "\n"
		"\tVF virt offset base to 0x%" PRIx64 "\n",
		bar, phys_addr, virt_addr);
	return 0;
}

static int lf_get_vf_bar(void *arg, u8 vf_id, u8 bar, void **va, void **pa)
{
	struct lf_mng_container	*lf_cont = (struct lf_mng_container *)arg;

	if (!lf_cont) {
		pr_err("no LF-CONTAINER obj!\n");
		return -EFAULT;
	}

#define PCI_EP_VF_BAR0_SIZE 0x1000
#define PCI_EP_VF_BAR2_SIZE 0x10000
#define PCI_EP_VF_BAR0_OFFSET(id)	(id * PCI_EP_VF_BAR0_SIZE)
#define PCI_EP_VF_BAR2_OFFSET(id)	(id * PCI_EP_VF_BAR2_SIZE)

	if (bar == 0) {
		*pa = (void *)(lf_cont->vf_bar0_map.phys_addr + PCI_EP_VF_BAR0_OFFSET(vf_id));
		*va = (void *)(lf_cont->vf_bar0_map.virt_addr + PCI_EP_VF_BAR0_OFFSET(vf_id));
	} else if (bar == 2) {
		*pa = (void *)(lf_cont->vf_bar2_map.phys_addr + PCI_EP_VF_BAR2_OFFSET(vf_id));
		*va = (void *)(lf_cont->vf_bar2_map.virt_addr + PCI_EP_VF_BAR2_OFFSET(vf_id));
	} else {
		pr_err("bar %d in invalid!\n", bar);
		return -EINVAL;
	}

	return 0;
}

static int lf_init_done(void *arg, u8 lf_id)
{
	struct lf_mng_container		*lf_mng_cont = (struct lf_mng_container *)arg;
	struct mv_sys_dma_mem_info	 mem_info;
	char	 file_name[SER_MAX_FILE_NAME];
	char	 buff[SER_MAX_FILE_SIZE];
	u32	 size = SER_MAX_FILE_SIZE;
	char	 dev_name[100], strl[12];
	size_t	 len, len1, pos = 0;
	struct lf_mng_lf *lf;
	int	 ret;

	if (!lf_mng_cont) {
		pr_err("no LF-MNG-container obj!\n");
		return -EFAULT;
	}

	pr_debug("nmp_pf_init_done reached\n");

	ret = lookup_lf(lf_mng_cont, lf_id, &lf);
	if (ret) {
		pr_err("LF %d not found!\n", lf_id);
		return -EIO;
	}

	lf_mng_cont->num_lfs_done++;
	if (lf_mng_cont->num_lfs_done < lf_mng_cont->num_lfs)
		return 0;

	/* TODO: go over all guests */
	snprintf(file_name,
		sizeof(file_name),
		"%s%s%d",
		SER_FILE_VAR_DIR,
		SER_FILE_NAME_PREFIX,
		lf_mng_cont->guest_id);
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

	pr_debug("starting serialization of guest %d\n", lf_mng_cont->guest_id);

	json_print_to_buffer(buff, size, 1, "\"relations-info\": {\n");
	json_print_to_buffer(buff, size, 2, "\"num-relations-info\": %u,\n", lf_mng_cont->num_lfs);

	for (lf_id = 0; lf_id < lf_mng_cont->num_lfs; lf_id++) {
		if (lf_mng_cont->lfs[lf_id].type == NMP_LF_T_NIC_PF)
			ret = nmnicpf_serialize(lf_mng_cont->lfs[lf_id].u.nmnicpf, &buff[pos], size - pos, 1);
		else if (lf_mng_cont->lfs[lf_id].type == NMP_LF_T_NIC_VF)
			ret = nmnicvf_serialize(lf_mng_cont->lfs[lf_id].u.nmnicvf, &buff[pos], size - pos, 1);
		else {
			pr_warn("LF type %d not supported!\n", lf_mng_cont->lfs[lf_id].type);
			continue;
		}
		if (ret >= 0)
			pos += ret;
		if (pos != strlen(buff)) {
			pr_err("found mismatch between pos (%zu) and buff len (%zu)\n", pos, strlen(buff));
			return -EFAULT;
		}
	}

	len = strlen("\"sizeof-relations-info\": ,");
	sprintf(strl, "%zu", len + pos);
	len1 = strlen(strl);
	json_print_to_buffer(buff, size, 2, "\"sizeof-relations-info\": %zu,\n", pos + len + len1);
	json_print_to_buffer(buff, size, 1, "},\n");

	/* Now serialize only the objects of each LF */
	for (lf_id = 0; lf_id < lf_mng_cont->num_lfs; lf_id++) {
		if (lf_mng_cont->lfs[lf_id].type == NMP_LF_T_NIC_PF)
			ret = nmnicpf_serialize(lf_mng_cont->lfs[lf_id].u.nmnicpf, &buff[pos], size - pos, 0);
		else if (lf_mng_cont->lfs[lf_id].type == NMP_LF_T_NIC_VF)
			ret = nmnicvf_serialize(lf_mng_cont->lfs[lf_id].u.nmnicvf, &buff[pos], size - pos, 0);
		else {
			pr_warn("LF type %d not supported!\n", lf_mng_cont->lfs[lf_id].type);
			continue;
		}
		if (ret >= 0)
			pos += ret;
		if (pos != strlen(buff)) {
			pr_err("found mismatch between pos (%zu) and buff len (%zu)\n", pos, strlen(buff));
			return -EFAULT;
		}
	}

	if (lf_mng_cont->nmcstm) {
		ret = nmcstm_serialize(lf_mng_cont->nmcstm, &buff[pos], size - pos);
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
		pr_err("Failed to write to guest %d file\n", lf_mng_cont->guest_id);
		return ret;
	}

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

	_lf_mng = kmalloc(sizeof(*_lf_mng), GFP_KERNEL);
	if (!_lf_mng)
		return -ENOMEM;
	memset(_lf_mng, 0, sizeof(struct lf_mng));

	_lf_mng->nmp = params->nmp;
	_lf_mng->nmdisp = params->nmdisp;
	_lf_mng->mqa = params->mqa;
	_lf_mng->giu = params->giu;

	_lf_mng->num_containers = params->num_containers;
	for (cntr = 0; cntr < _lf_mng->num_containers; cntr++) {
		_lf_mng->containers[cntr].lf_mng = _lf_mng;
		_lf_mng->containers[cntr].guest_id = params->containers_params[cntr].guest_id;
		/* We assume the first LF in the container is the "master" */
		_lf_mng->containers[cntr].master_id = _lf_mng->total_num_nicpfs;
		_lf_mng->containers[cntr].num_lfs = params->containers_params[cntr].num_lfs;

		/* TODO: init nmp-customer only in case it is supported in the nmp_params! */
		/* We assume the 'custom' LF is always the last LF in the container */
		memset(&nmcstm_params, 0, sizeof(nmcstm_params));
		nmcstm_params.id = _lf_mng->containers[cntr].guest_id;
		nmcstm_params.pf_id = _lf_mng->containers[cntr].master_id;
		nmcstm_params.mqa = _lf_mng->mqa;
		nmcstm_params.nmdisp = _lf_mng->nmdisp;
		err = nmcstm_init(&nmcstm_params, &(_lf_mng->containers[cntr].nmcstm));
		if (err) {
			pr_err("Custom init failed\n");
			return err;
		}

		for (lf = 0; lf < _lf_mng->containers[cntr].num_lfs; lf++) {
			_lf_mng->containers[cntr].lfs[lf].type =
				params->containers_params[cntr].lfs_params[lf].type;
			if (_lf_mng->containers[cntr].lfs[lf].type == NMP_LF_T_NIC_PF) {
				struct nmnicpf_params nmnicpf_params;

				memset(&nmnicpf_params, 0, sizeof(nmnicpf_params));
				_lf_mng->containers[cntr].lfs[lf].id = _lf_mng->total_num_lfs++;
				nmnicpf_params.lf_id = _lf_mng->containers[cntr].lfs[lf].id;
				nmnicpf_params.id = _lf_mng->total_num_nicpfs++;
				nmnicpf_params.guest_id = _lf_mng->containers[cntr].guest_id;

				/* Save reference to Dispatcher in PF */
				nmnicpf_params.nmdisp = _lf_mng->nmdisp;
				nmnicpf_params.mqa = _lf_mng->mqa;
				nmnicpf_params.giu = _lf_mng->giu;

				nmnicpf_params.nmp_nicpf_params =
					&params->containers_params[cntr].lfs_params[lf].u.nicpf;

				/* Assign the pf_init_done callback */
				nmnicpf_params.f_ready_cb = lf_init_done;
				nmnicpf_params.f_get_free_bar_cb = lf_get_free_bar;
				nmnicpf_params.f_put_bar_cb = lf_put_bar;
				nmnicpf_params.f_pp_find_free_bpool_cb = lf_pp_find_free_bpool;
				nmnicpf_params.f_set_vf_bar_offset_base_cb = lf_set_vf_bar_offset_base;
				nmnicpf_params.arg = &_lf_mng->containers[cntr];

				err = nmnicpf_init(&nmnicpf_params,
					&_lf_mng->containers[cntr].lfs[lf].u.nmnicpf);
				if (err)
					return err;
			} else if (_lf_mng->containers[cntr].lfs[lf].type == NMP_LF_T_NIC_VF) {
				struct nmnicvf_params nmnicvf_params;

				memset(&nmnicvf_params, 0, sizeof(nmnicvf_params));
				_lf_mng->containers[cntr].lfs[lf].id = _lf_mng->total_num_lfs++;
				nmnicvf_params.lf_id = _lf_mng->containers[cntr].lfs[lf].id;
				nmnicvf_params.id = _lf_mng->total_num_nicvfs++;
				nmnicvf_params.guest_id = _lf_mng->containers[cntr].guest_id;

				/* Save reference to Dispatcher in PF */
				nmnicvf_params.nmdisp = _lf_mng->nmdisp;
				nmnicvf_params.mqa = _lf_mng->mqa;
				nmnicvf_params.giu = _lf_mng->giu;

				nmnicvf_params.nmp_nicvf_params =
					&params->containers_params[cntr].lfs_params[lf].u.nicvf;

				/* Assign the pf_init_done callback */
				nmnicvf_params.f_ready_cb = lf_init_done;
				nmnicvf_params.f_get_free_bar_cb = lf_get_free_bar;
				nmnicvf_params.f_put_bar_cb = lf_put_bar;
				nmnicvf_params.f_get_vf_bar_cb = lf_get_vf_bar;
				nmnicvf_params.arg = &_lf_mng->containers[cntr];

				err = nmnicvf_init(&nmnicvf_params,
					&_lf_mng->containers[cntr].lfs[lf].u.nmnicvf);
				if (err)
					return err;
			} else {
				pr_err("LF type %d not supported!\n",
					_lf_mng->containers[cntr].lfs[lf].type);
				return -EINVAL;
			}
		}
	}

	*lf_mng = _lf_mng;
	return 0;
}

void lf_mng_deinit(struct lf_mng *lf_mng)
{
	u8	 cntr, lf;

	if (!lf_mng)
		return;

	for (cntr = 0; cntr < lf_mng->num_containers; cntr++) {
		for (lf = 0; lf < lf_mng->containers[cntr].num_lfs; lf++) {
			if (lf_mng->containers[cntr].lfs[lf].type == NMP_LF_T_NIC_PF)
				nmnicpf_deinit(lf_mng->containers[cntr].lfs[lf].u.nmnicpf);
			else if (lf_mng->containers[cntr].lfs[lf].type == NMP_LF_T_NIC_VF)
				nmnicvf_deinit(lf_mng->containers[cntr].lfs[lf].u.nmnicvf);
			else
				pr_warn("LF type %d not supported!\n", lf_mng->containers[cntr].lfs[lf].type);
		}
		nmcstm_deinit(lf_mng->containers[cntr].nmcstm);
	}

	kfree(lf_mng);
}

int lf_mng_run_maintenance(struct lf_mng *lf_mng)
{
	struct nmlf	*nmlf;
	u8		 cntr, lf;
	int		 ret;

	if (!lf_mng)
		return -1;

	for (cntr = 0; cntr < lf_mng->num_containers; cntr++)
		for (lf = 0; lf < lf_mng->containers[cntr].num_lfs; lf++) {
			/* As the lf is represented in a union so we can cast one of them and it works on all */
			nmlf = (struct nmlf *)lf_mng->containers[cntr].lfs[lf].u.nmnicpf;
			if (nmlf->f_maintenance_cb) {
				ret = nmlf->f_maintenance_cb(nmlf);
				if (ret)
					return ret;
			}
		}

	return 0;
}

/* TODO: move this routine to NMP-guest (i.e. should be caled through Qs) */
int lf_mng_create_scheduling_event(struct lf_mng *lf_mng,
	struct nmp_event_params *params,
	struct mv_sys_event **ev)
{
	u8 cntr = 0; /* TODO: need to iterate all NICPFs! */
	u8 lf = 0; /* TODO: need to iterate all NICPFs! */

	return nmnicpf_create_scheduling_event(lf_mng->containers[cntr].lfs[lf].u.nmnicpf,
		params,
		ev);
}

/* TODO: move this routine to NMP-guest (i.e. should be caled through Qs) */
int lf_mng_delete_scheduling_event(struct mv_sys_event *ev)
{
	return nmnicpf_delete_scheduling_event(ev);
}

/* TODO: move this routine to NMP-guest (i.e. should be caled through Qs) */
int lf_mng_set_scheduling_event(struct mv_sys_event *ev, int en)
{
	return nmnicpf_set_scheduling_event(ev, en);
}
