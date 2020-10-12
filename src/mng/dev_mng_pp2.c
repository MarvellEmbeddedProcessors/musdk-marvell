/*******************************************************************************
 * Copyright (C) Marvell International Ltd. and its affiliates
 *
 * This software file (the "File") is owned and distributed by Marvell
 * International Ltd. and/or its affiliates ("Marvell") under the following
 * alternative licensing terms.  Once you have made an election to distribute the
 * File under one of the following license alternatives, please (i) delete this
 * introductory statement regarding license alternatives, (ii) delete the three
 * license alternatives that you have not elected to use and (iii) preserve the
 * Marvell copyright notice above.
 *
 ********************************************************************************
 * Marvell Commercial License Option
 *
 * If you received this File from Marvell and you have entered into a commercial
 * license agreement (a "Commercial License") with Marvell, the File is licensed
 * to you under the terms of the applicable Commercial License.
 *
 ********************************************************************************
 * Marvell GPL License Option
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File in accordance with the terms and conditions of the General
 * Public License Version 2, June 1991 (the "GPL License"), a copy of which is
 * available along with the File in the license.txt file or by writing to the Free
 * Software Foundation, Inc., or on the worldwide web at http://www.gnu.org/licenses/gpl.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY
 * DISCLAIMED.  The GPL License provides additional details about this warranty
 * disclaimer.
 *
 ********************************************************************************
 * Marvell GNU General Public License FreeRTOS Exception
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File in accordance with the terms and conditions of the Lesser
 * General Public License Version 2.1 plus the following FreeRTOS exception.
 * An independent module is a module which is not derived from or based on
 * FreeRTOS.
 * Clause 1:
 * Linking FreeRTOS statically or dynamically with other modules is making a
 * combined work based on FreeRTOS. Thus, the terms and conditions of the GNU
 * General Public License cover the whole combination.
 * As a special exception, the copyright holder of FreeRTOS gives you permission
 * to link FreeRTOS with independent modules that communicate with FreeRTOS solely
 * through the FreeRTOS API interface, regardless of the license terms of these
 * independent modules, and to copy and distribute the resulting combined work
 * under terms of your choice, provided that:
 * 1. Every copy of the combined work is accompanied by a written statement that
 * details to the recipient the version of FreeRTOS used and an offer by yourself
 * to provide the FreeRTOS source code (including any modifications you may have
 * made) should the recipient request it.
 * 2. The combined work is not itself an RTOS, scheduler, kernel or related
 * product.
 * 3. The independent modules add significant and primary functionality to
 * FreeRTOS and do not merely extend the existing functionality already present in
 * FreeRTOS.
 * Clause 2:
 * FreeRTOS may not be used for any competitive or comparative purpose, including
 * the publication of any form of run time or compile time metric, without the
 * express permission of Real Time Engineers Ltd. (this is the norm within the
 * industry and is intended to ensure information accuracy).
 *
 ********************************************************************************
 * Marvell BSD License Option
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File under the following licensing terms.
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *	* Redistributions of source code must retain the above copyright notice,
 *	  this list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the distribution.
 *
 *	* Neither the name of Marvell nor the names of its contributors may be
 *	  used to endorse or promote products derived from this software without
 *	  specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
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

