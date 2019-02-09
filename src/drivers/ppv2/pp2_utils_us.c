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

/**
 * @file pp2_utils_us.c
 *
 * pp2 util functions specific for user space
 */

#include "std_internal.h"

#include "pp2.h"


#define PP2_NETDEV_PATH		"/sys/class/net/"




/* Get device tree data of the PPV2 ethernet ports.
 * Does not include the loopback port.
 */

static int pp2_get_devtree_port_data(struct netdev_if_params *netdev_params)
{
	FILE *fp;
	char cp110path[PP2_MAX_BUF_STR_LEN];
	char temppath[PP2_MAX_BUF_STR_LEN];
	char subpath[PP2_MAX_BUF_STR_LEN];
	char fullpath[PP2_MAX_BUF_STR_LEN];
	char buf[PP2_MAX_BUF_STR_LEN];
	int i, j, idx = 0, cp110_num, err;
	u8 num_inst;
	enum musdk_lnx_id lnx_id = lnx_id_get();


	if (!netdev_params)
		return -EFAULT;

	num_inst = pp2_get_num_inst();

	for (i = 0, cp110_num = 0; i < num_inst; i++, cp110_num++) {
		err = -1;
		while (cp110_num < PP2_MAX_NUM_PACKPROCS) {
			if (lnx_id == LNX_4_4_x)
				sprintf(cp110path, pp2_frm[lnx_id].devtree_path, cp110_num);
			else
				sprintf(cp110path, pp2_frm[lnx_id].devtree_path, cp110_num);
			err = access(cp110path, F_OK);
			if (!err)
				break;
			cp110_num++;
		}
		if (err) {
			pr_err("error accessing file %s\n", cp110path);
			return -EEXIST;
		}
		for (j = 0; j < PP2_NUM_ETH_PPIO; j++) {

			idx = i * PP2_NUM_ETH_PPIO + j;
			strcpy(temppath, cp110path);
			if (lnx_id == LNX_4_4_x)
				sprintf(subpath, pp2_frm[lnx_id].eth_format, j, j + 1);
			else
				sprintf(subpath, pp2_frm[lnx_id].eth_format, j);
			strcat(temppath, subpath);
			strcpy(fullpath, temppath);
			strcat(fullpath, "/status");
			fp = fopen(fullpath, "r");
			if (!fp) {
				pr_err("error opening file %s\n", fullpath);
				return -EEXIST;
			}

			netdev_params[idx].ppio_id = j;
			netdev_params[idx].pp_id = i;

			fgets(buf, sizeof(buf), fp);
			fclose(fp);

			if (strcmp("disabled", buf) == 0) {
				pr_debug("port %d:%d is disabled\n", i, j);
				netdev_params[idx].admin_status = PP2_PORT_DISABLED;
			} else {
				netdev_params[idx].admin_status = PP2_PORT_KERNEL;
				pr_debug("port %d:%d is ok\n", i, j);
			}
		}
	}
	return 0;
}

/* pp2_netdev_if_info_get()
 * Retireve information from netdev for all instances and ports:
 * - if name
 * - port status (disabled/Kernel)
 */
int pp2_netdev_if_info_get(struct netdev_if_params *netdev_params)
{
	FILE *fp;
	char path[PP2_MAX_BUF_STR_LEN];
	char subpath[PP2_MAX_BUF_STR_LEN];
	char buf[PP2_MAX_BUF_STR_LEN];
	u32 i, idx = 0;
	int if_dup = false;
	struct ifaddrs *ifap, *ifa;
static int first_time = true;

	if (!first_time)
		return 0;

	if (!netdev_params)
		return -EFAULT;

	/* Step 1: check in dtb the status of the port */
	pp2_get_devtree_port_data(netdev_params);

	/* Step 2: parse through netdev if devices and get the if name  */
	/*check in uevent file OF_NODE=ppv22 exists */
	if (getifaddrs(&ifap) != 0) {
		pr_err("unable to get netdev if info");
		return -EFAULT;
	}

	for (ifa = ifap; ifa; ifa = ifa->ifa_next) {

		if (strncmp("eth", ifa->ifa_name, 3) != 0)
			continue;

		/* Filter already parsed interfaces, since getifaddrs linked list contains entries
		 * for the same interface and different family types
		 */
		for (i = 0; i < PP2_MAX_NUM_PACKPROCS * PP2_NUM_ETH_PPIO; i++) {
			if (strcmp(netdev_params[i].if_name, ifa->ifa_name) == 0) {
				if_dup = true;
				break;
			}
		}

		if (if_dup) {
			if_dup = false;
			continue;
		}

		sprintf(path, PP2_NETDEV_PATH);
		sprintf(subpath, "%s/device/uevent", ifa->ifa_name);

		strcat(path, subpath);
		fp = fopen(path, "r");
		if (!fp) {
			pr_err("error opening %s\n", path);
			freeifaddrs(ifap);
			return -EEXIST;
		}
		while (fgets(buf, PP2_MAX_BUF_STR_LEN, fp)) {
			if (strncmp("DRIVER=mvpp2", buf, 12) == 0) {
				while (netdev_params[idx].admin_status == PP2_PORT_DISABLED)
					idx++;
				strcpy(netdev_params[idx].if_name, ifa->ifa_name);
				idx++;
			}
		}
		fclose(fp);
	}
	freeifaddrs(ifap);

	first_time = false;

	return 0;
}
