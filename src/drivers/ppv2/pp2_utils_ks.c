/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

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
