/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#define _GNU_SOURCE         /* See feature_test_macros(7) */
#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <sched.h>
#include <fcntl.h>
#include <sys/time.h>

#include "mv_std.h"
#include "utils.h"
#include "env/mv_sys_dma.h"
#include "mvapp.h"
#include "mng/mv_nmp.h"
#include "lib/file_utils.h"
#include "lib/lib_misc.h"
#include "mv_pp2_bpool.h"

#define APP_DMA_MEM_SIZE		(8 * 1024 * 1024)

struct glob_arg {
	struct glb_common_args		 cmn_args; /* Keep first */
	struct nmp			*nmp;
};

static struct glob_arg garg = {};

#define NMP_MAX_BUF_STR_LEN		256
#define NMP_MAX_NUM_CONTAINERS		4
#define NMP_MAX_NUM_LFS			8
#define NMP_GIE_MAX_TCS			8
#define NMP_GIE_MAX_Q_PER_TC		128
#define NMP_GIE_MAX_BPOOLS		16
#define NMP_GIE_MAX_BM_PER_Q		1


static int nmp_range_validate(int value, int min, int max)
{
	if (((value) > (max)) || ((value) < (min))) {
		pr_err("%s: value 0x%X (%d) is out of range [0x%X , 0x%X].\n",
			__func__, (value), (value), (min), (max));
		return -EINVAL;
	}
	return 0;
}

static int nmp_read_cfg_file(char *cfg_file, struct nmp_params *params)
{
	u32				 i, j, k, rc;
	char				 buff[SER_MAX_FILE_SIZE];
	char				*sec = NULL;
	char				 tmp_buf[NMP_MAX_BUF_STR_LEN];
	char				 pp2_name[NMP_MAX_BUF_STR_LEN], giu_name[NMP_MAX_BUF_STR_LEN];
	struct nmp_lf_nicpf_params	*pf;
	u32				 num_lfs = 0;

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

	/* Check if pp2 is enabled */
	json_buffer_to_input(sec, "pp2_en", params->pp2_en);
	if (nmp_range_validate(params->pp2_en, 0, 1) != 0) {
		pr_err("pp2_en not in tange!\n");
		return -EINVAL;
	}

	/* if pp2 enabled, set the pp2_params*/
	if (params->pp2_en) {
		sec = strstr(sec, "pp2_params");
		if (!sec) {
			pr_err("'pp2_params' not found\n");
			return -EINVAL;
		}

		json_buffer_to_input(sec, "bm_pool_reserved_map", params->pp2_params.bm_pool_reserved_map);
		if (nmp_range_validate(params->pp2_params.bm_pool_reserved_map, 0, PP2_BPOOL_NUM_POOLS)) {
			pr_err("bm_pool_reserved_map not in range!\n");
			rc = -EINVAL;
			goto read_cfg_exit1;
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
		num_lfs++;

		for (j = 0; j < params->containers_params[i].num_lfs; j++) {
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

				pf->match = giu_name;
				json_buffer_to_input_str(sec, "match", pf->match);

				pf->keep_alive_thresh = (u32)-1;
				json_buffer_to_input(sec, "keep_alive_thresh", pf->keep_alive_thresh);
				if (pf->keep_alive_thresh == (u32)-1) {
					pr_err("missing keep_alive_thresh!\n");
					rc = -EINVAL;
					goto read_cfg_exit2;
				}

				json_buffer_to_input(sec, "pci_en", pf->pci_en);
				if (nmp_range_validate(pf->pci_en, 0, 1) != 0) {
					pr_err("pci_en not in range!\n");
					rc = -EINVAL;
					goto read_cfg_exit2;
				}

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
				if (nmp_range_validate(pf->max_num_tcs, 0, NMP_GIE_MAX_TCS) != 0) {
					pr_err("missing max_num_tcs!\n");
					rc = -EINVAL;
					goto read_cfg_exit2;
				}

				json_buffer_to_input(sec, "lcl_num_bpools", pf->lcl_num_bpools);
				if (nmp_range_validate(pf->lcl_num_bpools, 0, NMP_GIE_MAX_BPOOLS) != 0) {
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
			}
		}

		json_buffer_to_input(sec, "guest_id", params->containers_params[i].guest_id);
		if (nmp_range_validate(params->containers_params[i].guest_id, 0, 10) != 0) {
			rc = -EINVAL;
			goto read_cfg_exit2;
		}
	}

	return 0;
read_cfg_exit2:
	for (i = 0; i < num_lfs; i++)
		kfree(params->containers_params[i].lfs_params);
read_cfg_exit1:
	kfree(params->containers_params);
	return rc;
}

static int main_loop_cb(void *arg, int *running)
{
	while (*running) {
		nmp_schedule(garg.nmp, NMP_SCHED_RX);
		nmp_schedule(garg.nmp, NMP_SCHED_TX);
	}

	return 0;
}

static int ctrl_cb(void *arg)
{
	struct glob_arg *garg = (struct glob_arg *)arg;

	if (!garg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	nmp_schedule(garg->nmp, NMP_SCHED_MNG);

	return 0;
}

static int init_all_modules(void)
{
	int			err;
	struct			nmp_params nmp_params;

	pr_info("Global initializations ...\n");
	err = mv_sys_dma_mem_init(APP_DMA_MEM_SIZE);
	if (err)
		return err;

	/* NMP initializations */
	memset(&nmp_params, 0, sizeof(nmp_params));
	err = nmp_read_cfg_file(garg.cmn_args.nmp_cfg_location, &nmp_params);
	if (err) {
		pr_err("NMP preinit failed with error %d\n", err);
		return -EIO;
	}

	nmp_init(&nmp_params, &(garg.nmp));

	return 0;
}

static int init_global(void *arg)
{
	struct glob_arg *garg = (struct glob_arg *)arg;
	int		 err;

	if (!garg) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	err = init_all_modules();
	if (err)
		return err;

	return 0;
}

static void deinit_global(void *arg)
{
	mv_sys_dma_mem_destroy();
}

static void usage(char *progname)
{
	printf("\n"
	       "MUSDK NMP standalone application.\n"
	       "\n"
	       "Usage: %s OPTIONS\n"
	       "  E.g. %s -a 1\n"
	       "\n"
	       "Optional OPTIONS:\n"
	       "\t-a, --affinity <number>  Use setaffinity (default is no affinity)\n"
	       "\t- f, --file              Location and name of the nmp-config file to load\n"
	       "\t?, -h, --help            Display help and exit.\n\n"
	       "\n", MVAPPS_NO_PATH(progname), MVAPPS_NO_PATH(progname));
}

static int parse_args(struct glob_arg *garg, int argc, char *argv[])
{
	int	i = 1;
	int	file_set = 0;

	garg->cmn_args.cli = 0;
	garg->cmn_args.cpus = 1;
	garg->cmn_args.affinity = MVAPPS_INVALID_AFFINITY;

	while (i < argc) {
		if ((strcmp(argv[i], "?") == 0) ||
		    (strcmp(argv[i], "-h") == 0) ||
		    (strcmp(argv[i], "--help") == 0)) {
			usage(argv[0]);
			exit(0);
		} else if (strcmp(argv[i], "-f") == 0) {
			if (argc < (i + 2)) {
				pr_err("Invalid number of arguments!\n");
				return -EINVAL;
			}
			if (argv[i + 1][0] == '-') {
				pr_err("Invalid arguments format!\n");
				return -EINVAL;
			}

			strcpy(garg->cmn_args.nmp_cfg_location, argv[i + 1]);
			i += 2;
			file_set = 1;
		} else if (strcmp(argv[i], "-a") == 0) {
			garg->cmn_args.affinity = atoi(argv[i + 1]);
			i += 2;
		} else {
			pr_err("argument (%s) not supported!\n", argv[i]);
			return -EINVAL;
		}
	}

	if ((garg->cmn_args.affinity != -1) &&
	    ((garg->cmn_args.cpus + garg->cmn_args.affinity) > MVAPPS_MAX_NUM_CORES)) {
		pr_err("illegal num cores or affinity requested (%d,%d vs %d)!\n",
		       garg->cmn_args.cpus, garg->cmn_args.affinity, MVAPPS_MAX_NUM_CORES);
		return -EINVAL;
	}

	if (!file_set) {
		pr_err("nmp config file must be given (use -f)\n");
		return -EINVAL;
	}

	return 0;
}

static void init_app_params(struct mvapp_params *mvapp_params, u64 cores_mask)
{
	memset(mvapp_params, 0, sizeof(struct mvapp_params));
	mvapp_params->use_cli		= garg.cmn_args.cli;
	mvapp_params->num_cores		= garg.cmn_args.cpus;
	mvapp_params->cores_mask	= cores_mask;
	mvapp_params->global_arg	= (void *)&garg;
	mvapp_params->init_global_cb	= init_global;
	mvapp_params->deinit_global_cb	= deinit_global;
	mvapp_params->init_local_cb	= NULL;
	mvapp_params->deinit_local_cb	= NULL;
	mvapp_params->main_loop_cb	= main_loop_cb;
	mvapp_params->ctrl_cb		= ctrl_cb;
}

int main(int argc, char *argv[])
{
	struct mvapp_params	mvapp_params;
	u64			cores_mask;
	int			err;

	setbuf(stdout, NULL);

	pr_info("nmp-standalone is started\n");
	pr_debug("pr_debug is enabled\n");

	err = parse_args(&garg, argc, argv);
	if (err)
		return err;

	cores_mask = apps_cores_mask_create(garg.cmn_args.cpus, garg.cmn_args.affinity);

	init_app_params(&mvapp_params, cores_mask);

	return mvapp_go(&mvapp_params);
}

