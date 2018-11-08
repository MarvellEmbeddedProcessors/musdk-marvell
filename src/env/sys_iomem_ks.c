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

#include "std_internal.h"
#include "env/sys_iomem.h"

#include <linux/io.h>
#include <linux/of.h>

/* MAX_RESOURCES is set to the actual number in current dts.
 * Should probably switch to searching in strings instead of globbing them
 */
#define MAX_RESOURCES		14
#define MAX_RESOURCE_NAME	32
#define MAX_COMPAT_LEN		32

#define COMPAT_TEMPLATE		"marvell,mv-%s"

struct sys_iomem_resource {
	char		name[MAX_RESOURCE_NAME];
	phys_addr_t	pa;
	u32		length;
	void	        *va;
};

struct sys_iomem {
	u8				no_of_resources;
	struct sys_iomem_resource	res[MAX_RESOURCES];
	struct device_node	       *node;
	phys_addr_t			base_addr;
};

int sys_iomem_params_to_node(struct sys_iomem_params *params, struct device_node **pnode)
{
	char compat[MAX_COMPAT_LEN];
	struct device_node *node = NULL;
	u8 index = params->index;

	*pnode = NULL;

	if (params->type != SYS_IOMEM_T_UIO) {
		pr_err("%s: Unsupported iomem type %d (only SYS_IOMEM_T_UIO supported).\n",
		       __func__, params->type);
		return -EINVAL;
	}

	sprintf(compat, COMPAT_TEMPLATE, params->devname);

	do {
		node = of_find_compatible_node(node, NULL, compat);

		if (!node) {
			pr_debug("%s: Node '%s' (compat='%s') no. %d not found.\n",
				 __func__, params->devname, compat, index);
			return -ENOENT;
		}

	} while (index--);

	*pnode = node;

	return 0;
}

int sys_iomem_exists(struct sys_iomem_params *params)
{
	struct device_node *node;

	return !sys_iomem_params_to_node(params, &node);
}

int sys_iomem_get_base_addr(struct device_node *node, phys_addr_t *base_addr)
{
	struct device_node *config_space = of_get_parent(node);
	int rc;
	u32 ranges[4];

	if (!config_space) {
		pr_err("%s: Orphan node.\n", __func__);
		return -EINVAL;
	}

	rc = of_property_read_u32_array(config_space, "ranges", ranges, 4);
	if (rc) {
		pr_err("%s: Failed reading address ranges: %d.\n", __func__, rc);
		return rc;
	}

	*base_addr = ranges[2];

	if (!*base_addr) {
		pr_err("%s: NULL base address.\n", __func__);
		return -EINVAL;
	}


	return 0;
}

int sys_iomem_init(struct sys_iomem_params *params, struct sys_iomem **iomem)
{
	int rc, i;
	char *names[MAX_RESOURCES];
	u32 uints[MAX_RESOURCES * 2];

	memset(names, 0, sizeof(char *) * MAX_RESOURCES);
	memset(uints, 0, sizeof(u32) * 2 * MAX_RESOURCES);

	*iomem = kzalloc(sizeof(struct sys_iomem), GFP_KERNEL);
	if (!*iomem)
		return -ENOMEM;

	rc = sys_iomem_params_to_node(params, &((*iomem)->node));
	if (rc) {
		pr_err("%s: Can't find device tree node for %s(%d).\n",
		       __func__, params->devname, params->index);
	}

	rc = sys_iomem_get_base_addr((*iomem)->node, &(*iomem)->base_addr);

	if (rc) {
		pr_err("%s: Failed to get base address: %d.\n", __func__, rc);
		return rc;
	}

	rc = of_property_read_u32_array((*iomem)->node, "reg", uints, MAX_RESOURCES*2);

	if (rc < 0) {
		pr_err("%s: Error reading offset/length array: %d.\n", __func__, rc);
		return rc ? rc : -EINVAL;
	}

	rc = of_property_read_string_array((*iomem)->node, "reg-names", (const char **) names, MAX_RESOURCES);

	if (rc < 0) {
		pr_err("%s: Error reading reg-names array: %d.\n", __func__, rc);
		return rc ? rc : -EINVAL;
	}

	for (i = 0; i < MAX_RESOURCES; i++) {

		if (!names[i] || !strlen(names[i]) || !uints[i*2+1])
			break;

		strncpy((*iomem)->res[i].name, names[i], MAX_RESOURCE_NAME);
		(*iomem)->res[i].pa = (*iomem)->base_addr + uints[i*2];
		(*iomem)->res[i].length = uints[i*2+1];

	}
	/* TODO: add check if we exceeded MAX_RESOURCES!! */

	(*iomem)->no_of_resources = i;

	pr_debug("%s: Done. %d resources initialized at 0x%llx.\n", __func__, i, (*iomem)->base_addr);

	return 0;
}

void sys_iomem_deinit(struct sys_iomem *iomem)
{
	int i;

	for (i = 0; i < iomem->no_of_resources; i++) {

		if (iomem->res[i].va)
			sys_iomem_unmap(iomem, iomem->res[i].name);
	}

	kfree(iomem);
}


int sys_iomem_map(struct sys_iomem *iomem, const char *name, phys_addr_t *pa, void **va)
{
	int i, rc;

	for (i = 0; i < iomem->no_of_resources; i++) {

		if (!strcmp(name, iomem->res[i].name)) {

			if (iomem->res[i].va) {

				pr_warn("%s: Resource '%s' already ioremapped.\n", __func__, name);
			} else {

				/* Do not request memory region, because it's alreay occupied by the other driver */

				iomem->res[i].va = ioremap(iomem->res[i].pa, iomem->res[i].length);

				if (IS_ERR_VALUE((long) iomem->res[i].va)) {

					pr_err("%s: Failed to ioremap resource '%s': %ld.\n",
					       __func__, name, (long) iomem->res[i].va);

					rc = (int) (long) iomem->res[i].va;
					iomem->res[i].va = NULL;
					return rc;
				}
			}

			*pa = iomem->res[i].pa;
			*va = iomem->res[i].va;

			return 0;
		}
	}

	pr_err("%s: Can't find resource with name '%s'.\n", __func__, name);
	return -EINVAL;
}

int sys_iomem_unmap(struct sys_iomem *iomem, const char *name)
{
	int i;

	for (i = 0; i < iomem->no_of_resources; i++) {

		if (!strcmp(name, iomem->res[i].name)) {

			if (iomem->res[i].va) {

				pr_err("%s: Unmapping an already unmapped resource '%s'.\n", __func__, name);
				return -EINVAL;
			}

			iounmap(iomem->res[i].va);
			iomem->res[i].va = NULL;

			return 0;
		}
	}

	pr_err("%s: Resource not found: '%s'.\n", __func__, name);
	return -EINVAL;
}

int sys_iomem_get_info(struct sys_iomem_params *params, struct sys_iomem_info *info)
{
	pr_err("[%s] routine not supported yet!\n", __func__);
	return -ENOTSUP;
}
