/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#define log_fmt(fmt, ...) "dev_mng: " fmt, ##__VA_ARGS__

#include "std_internal.h"
#include "env/trace/trc_pf.h"
#include "drivers/mv_pp2.h"
#include "drivers/mv_pp2_bpool.h"
#include "drivers/mv_pp2_ppio.h"
#include "lib/lib_misc.h"

#include "db.h"
#include "dev_mng.h"
#include "dev_mng_pp2.h"


#define NMP_PP2_FIRST_MUSDK_IN_QUEUE		0

/* Maximum number of ports used by NMP */
#define NMP_PP2_MAX_NUM_PORTS			3

#define PP2_SYSFS_MUSDK_PATH		"/sys/devices/platform/pp2/musdk"
#define PP2_SYSFS_RSS_PATH		"/sys/devices/platform/pp2/rss"

#define PP2_SYSFS_RSS_NUM_TABLES_FILE	"num_rss_tables"
#define PP2_SYSFS_DEBUG_PORT_SET_FILE	"sysfs_current_port"

#define PP2_DEF_KERNEL_NUM_RSS_TBL	4

#define PP2_MAX_BUF_STR_LEN		256


/** =========================== **/
/** == Device Initialization == **/
/** =========================== **/

/** ====================== **/
/** Hardware Functionality **/
/** ====================== **/

static int pp2_sysfs_param_get(char *if_name, char *file)
{
	FILE *fp;
	char r_buf[PP2_MAX_BUF_STR_LEN];
	char w_buf[PP2_MAX_BUF_STR_LEN];
	u32 param = 0, scanned;

	fp = fopen(file, "r");
	if (!fp) {
		pr_err("%s Failed to open file: %s\n", __func__, file);
		return -EEXIST;
	}

	sprintf(w_buf, "echo %s > %s/%s", if_name, PP2_SYSFS_MUSDK_PATH, PP2_SYSFS_DEBUG_PORT_SET_FILE);
	system(w_buf);

	fgets(r_buf, sizeof(r_buf), fp);
	scanned = sscanf(r_buf, "%d\n", &param);
	if (scanned != 1) {
		pr_err("Invalid number of parameters read %s\n", r_buf);
		fclose(fp);
		return -EINVAL;
	}

	fclose(fp);
	return param;
}

static void pp2_set_reserved_bpools(struct nmp *nmp)
{
	int i;

	for (i = 0; i < PP2_NUM_PKT_PROC; i++)
		nmp->nmpp2.used_bpools[i] = nmp->nmpp2.bm_pool_reserved_map;
}

static int pp2_get_rss_num_tbl(char *if_name __attribute__((__unused__)))
{
	return PP2_DEF_KERNEL_NUM_RSS_TBL;
}

/* Initialize the PP2 */
static int pp2_inst_init(struct nmp *nmp)
{
	struct pp2_init_params	 pp2_params;
	char			 if_name[16];
	int			 num_rss_tables;
	int			 err;

	if (!nmp->nmpp2.pp2_en)
		return 0;

	num_rss_tables = pp2_get_rss_num_tbl(if_name);
	if (num_rss_tables < 0) {
		pr_err("Failed to read kernel RSS tables. Please check mvpp2x_sysfs.ko is loaded\n");
		return -EFAULT;
	}

	memset(&pp2_params, 0, sizeof(struct pp2_init_params));
	pp2_params.rss_tbl_reserved_map = (1 << num_rss_tables) - 1;
	pp2_params.res_maps_auto_detect_map = PP2_RSRVD_MAP_HIF_AUTO;
	if (nmp->nmpp2.bm_pool_reserved_map)
		pp2_params.bm_pool_reserved_map = nmp->nmpp2.bm_pool_reserved_map;
	else
		pp2_params.res_maps_auto_detect_map |= PP2_RSRVD_MAP_BM_POOL_AUTO;

	err = pp2_init(&pp2_params);
	if (err)
		return -EINVAL;

	nmp->nmpp2.bm_pool_reserved_map = pp2_params.bm_pool_reserved_map;

	pp2_set_reserved_bpools(nmp);

	return 0;
}


/* Initialize the PP2 interface */
int dev_mng_pp2_init(struct nmp *nmp)
{
	int err;

	pr_debug("Initializing PP2...\n");

	/* Initialize the ppio instances */
	err = pp2_inst_init(nmp);
	if (err) {
		pr_err("dev_mng_pp2_inst_init failed\n");
		return -EINVAL;
	}

	pr_debug("pp2 enabled: %d\n", nmp->nmpp2.pp2_en);

	return 0;
}

/** ======================== **/
/** == Device Termination == **/
/** ======================== **/

/** ======================== **/
/** Hardware Functionality **/
/** ====================== **/

int dev_mng_pp2_terminate(struct nmp *nmp)
{
	pp2_deinit();
	return 0;
}

int dev_mng_pp2_find_free_bpool(struct nmp *nmp, u32 pp_id)
{
	int i;

	for (i = 0; i < PP2_BPOOL_NUM_POOLS; i++) {
		if (!((1 << i) & nmp->nmpp2.used_bpools[pp_id])) {
			nmp->nmpp2.used_bpools[pp_id] |= (1 << i);
			break;
		}
	}
	if (i == PP2_BPOOL_NUM_POOLS) {
		pr_err("no free BPool found!\n");
		return -ENOSPC;
	}
	return i;
}

