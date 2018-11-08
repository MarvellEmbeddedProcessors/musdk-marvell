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
