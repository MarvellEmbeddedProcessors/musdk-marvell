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

static const struct of_device_id mv_pp2_match_tbl[] = {
		{
			.compatible = "marvell,mv-pp22",
		},
		{
			.compatible = "marvell,armada-7k-pp22",
		},

	{ }
};


int pp2_netdev_if_info_get(struct netdev_if_params *netdev_params)
{
	struct device_node *pp2_next_node, *pp2_cur_node = NULL;
	struct device_node *port_node;
	struct net_device *netdev;
	const char *status;
	u32 pp2_inst = 0, port_id, idx;


	while ((pp2_next_node = of_find_matching_node(pp2_cur_node, mv_pp2_match_tbl)) != NULL) {
		pp2_cur_node = pp2_next_node;
		for_each_child_of_node(pp2_cur_node, port_node) {

			if (of_property_read_u32(port_node, "port-id", &port_id)) {
				pr_err("missing port-id value\n");
				return -EINVAL;
			}
			idx = pp2_inst * PP2_NUM_ETH_PPIO + port_id;
			netdev_params[idx].ppio_id = port_id;
			netdev_params[idx].pp_id = pp2_inst;

			status = of_get_property(port_node, "status", NULL);
			if (!strcmp("disabled", status)) {
				pr_debug("%s: port %d:%d is disabled\n", __func__, pp2_inst, port_id);
				netdev_params[idx].admin_status = PP2_PORT_DISABLED;
			} else {
				netdev_params[idx].admin_status = PP2_PORT_KERNEL;
				netdev = of_find_net_device_by_node(port_node);
				if (netdev)
					strcpy(netdev_params[idx].if_name, netdev->name);
			}
			pr_debug("%s: Port '%s' (ppio%d:%d) is %s.\n", __func__, netdev_params[idx].if_name,
				 pp2_inst, port_id,
				 netdev_params[idx].admin_status == PP2_PORT_DISABLED ? "disabled" :
				 netdev_params[idx].admin_status == PP2_PORT_KERNEL ? "owned by kernel" :
				 "in an unknown state");
			}
		 pp2_inst++;
	}

	return 0;
}
