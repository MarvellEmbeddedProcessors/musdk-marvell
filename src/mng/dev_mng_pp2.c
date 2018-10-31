/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#define log_fmt(fmt) "dev_mng: " fmt

#include "std_internal.h"
#include "env/trace/trc_pf.h"

#include "db.h"
#include "drivers/mv_pp2.h"
#include "drivers/mv_pp2_bpool.h"
#include "drivers/mv_pp2_ppio.h"
#include "src/drivers/ppv2/pp2.h"
#include "dev_mng.h"
#include "dev_mng_pp2.h"
#include "lib/lib_misc.h"

#define NMP_PP2_FIRST_MUSDK_IN_QUEUE		0

/* Maximum number of ports used by NMP */
#define NMP_PP2_MAX_NUM_PORTS			3

/** =========================== **/
/** == Device Initialization == **/
/** =========================== **/

/** ====================== **/
/** Hardware Functionality **/
/** ====================== **/

/* Initialize the PP2 */
static int dev_mng_pp2_inst_init(struct nmp *nmp)
{
	struct pp2_init_params	 pp2_params;
	int			 err;

	if (!nmp->nmpp2.pp2_en)
		return 0;

	memset(&pp2_params, 0, sizeof(struct pp2_init_params));
	pp2_params.bm_pool_reserved_map = nmp->nmpp2.pp2_params.bm_pool_reserved_map;
	pp2_params.hif_reserved_map = 0;
	pp2_params.rss_tbl_reserved_map = 0;

	err = pp2_init(&pp2_params);
	if (err)
		return -EINVAL;

	return 0;
}

/* Initialize the PP2 interface */
int dev_mng_pp2_init(struct nmp *nmp)
{
	int err;

	pr_info("Initializing PP2...\n");

	/* Initialize the ppio instances */
	err = dev_mng_pp2_inst_init(nmp);
	if (err) {
		pr_err("dev_mng_pp2_inst_init failed\n");
		return -EINVAL;
	}

	pr_info("pp2 enabled: %d\n", nmp->nmpp2.pp2_en);

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

