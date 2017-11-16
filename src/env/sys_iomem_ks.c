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

#include "std_internal.h"
#include "env/sys_iomem.h"

#include <linux/io.h>
#include <linux/of.h>

/* MAX_RESOURCES is set to the actual number in current dts.
 * Should probably switch to searching in strings instead of globbing them
 */
#define MAX_RESOURCES		6
#define MAX_RESOURCE_NAME	32
#define MAX_COMPAT_LEN		32

#define COMPAT_TEMPLATE		"marvell,mv-%s-uio"

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
