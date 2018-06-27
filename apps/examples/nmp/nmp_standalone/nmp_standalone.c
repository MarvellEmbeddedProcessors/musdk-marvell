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

