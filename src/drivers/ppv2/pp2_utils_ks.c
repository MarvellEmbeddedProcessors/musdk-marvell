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

/*
 * @file pp2_utils_ks.c
 *
 * pp2 util functions specific for kernel space
 */

#include "std_internal.h"
#include <linux/of_device.h>
#include <linux/of_platform.h>

#include "pp2.h"

#define PP2_NETDEV_PATH_TEMPLATE_K	"/cp%u/config-space/ppv22@000000/"

/*
 * Get device tree data of the PPV2 ethernet ports.
 * Does not include the loopback port.
 */
int pp2_netdev_if_info_get(struct netdev_if_params *netdev_params)
{
	struct device_node *port_node;
	const char *musdk_status, *status;
	char subpath[PP2_MAX_BUF_STR_LEN];
	char fullpath[PP2_MAX_BUF_STR_LEN];
	u32 if_id = 0;
	int i, j, idx;
	u8 num_inst;

	num_inst = pp2_get_num_inst();

	for (i = 0; i < num_inst; i++) {

		for (j = 0; j < PP2_NUM_ETH_PPIO; j++) {

			idx = i * PP2_NUM_ETH_PPIO + j;

			/* TODO -We assume that the temppath is static.
			 * Need to substitute this with a function that searches for the following string:
			 * <.compatible = "marvell,mv-pp22"> in /proc/device-tree directories
			 * and returns the temppath
			 */
			sprintf(fullpath, PP2_NETDEV_PATH_TEMPLATE_K, i);

			netdev_params[idx].ppio_id = j;
			netdev_params[idx].pp_id = i;
			sprintf(subpath, "eth%d@0%d0000", j, j + 1);
			strcat(fullpath, subpath);
			pr_debug("%s: of_find_node_by_path(%s)\n", __func__, fullpath);

			port_node = of_find_node_by_path(fullpath);

			if (!port_node) {
				pr_err("Failed to find device-tree node: %s\n", fullpath);
				return -ENODEV;
			}
			status = of_get_property(port_node, "status", NULL);
			if (!strcmp("disabled", status)) {
				pr_debug("%s: port %d:%d is disabled\n", __func__, i, j);
				netdev_params[idx].admin_status = PP2_PORT_DISABLED;
			} else {
				sprintf(netdev_params[idx].if_name, "eth%d", if_id++);
				musdk_status = of_get_property(port_node, "musdk-status", NULL);
				/* Set musdk_flag, only if status is "private", not if status is "shared" */
				if (musdk_status && !strcmp(musdk_status, "private")) {
					pr_debug("private\n");
					netdev_params[idx].admin_status = PP2_PORT_MUSDK;
				} else if (musdk_status && !strcmp(musdk_status, "shared")) {
					pr_debug("shared\n");
					netdev_params[idx].admin_status = PP2_PORT_SHARED;
				} else {
					pr_debug("kernel\n");
					netdev_params[idx].admin_status = PP2_PORT_KERNEL;
				}
			}
			pr_debug("%s: Port '%s' (ppio%d:%d) is %s.\n", __func__, netdev_params[idx].if_name, i, j,
				 netdev_params[idx].admin_status == PP2_PORT_DISABLED ? "disabled" :
				 netdev_params[idx].admin_status == PP2_PORT_MUSDK ? "owned by musdk" :
				 netdev_params[idx].admin_status == PP2_PORT_SHARED ? "shared" :
				 netdev_params[idx].admin_status == PP2_PORT_KERNEL ? "owned by kernel" :
				 "in an unknown state");
		}
	}

	return 0;
}
