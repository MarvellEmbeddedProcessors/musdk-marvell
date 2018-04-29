/******************************************************************************
 *	Copyright (C) 2016 Marvell International Ltd.
 *
 *  If you received this File from Marvell, you may opt to use, redistribute
 *  and/or modify this File under the following licensing terms.
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *	* Redistributions of source code must retain the above copyright
 *	  notice, this list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the distribution.
 *
 *	* Neither the name of Marvell nor the names of its contributors may be
 *	  used to endorse or promote products derived from this software
 *	  without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

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
	u8 num_inst;

	num_inst = pp2_get_num_inst();

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
		for (i = 0; i < num_inst * PP2_NUM_ETH_PPIO; i++) {
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
	return 0;
}
