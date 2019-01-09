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

#include "of_sh.h"


#define OF_DEFAULT_NA 1
#define OF_DEFAULT_NS 1

#define OF_SH_FILENAME	"./of.sh"

#define OF_MAX_NODES	64


static u8			current;
static struct device_node	current_node[OF_MAX_NODES];
static struct device_node	root = {.full_name	= "/",
					.name		= root.full_name
					};


static int create_of_sh(void)
{
	FILE	*of_sh;
	char	*script_str = OF_SCRIPT_STR;
	char	 command[PATH_MAX];

	of_sh = fopen(OF_SH_FILENAME, "r");
	if (of_sh) {
		fclose(of_sh);
		return 0;
	}

	of_sh = fopen(OF_SH_FILENAME, "w");
	if (unlikely(!of_sh)) {
		pr_err("Couldn't create OF file!\n");
		return -EFAULT;
	}
	fputs(script_str, of_sh);
	fclose(of_sh);
	snprintf(command, sizeof(command), "chmod 755 %s", OF_SH_FILENAME);
	system(command);
	return 0;
}

static char *of_basename(const char *full_name)
{
	char	*name;

	name = strrchr(full_name, '/');
	if (unlikely(name == NULL))
		return NULL;
	if (name != full_name)
		name++;
	return name;
}

static void of_bus_default_count_cells(const struct device_node	*dev_node,
					u32		*addr,
					u32		*size)
{
	if (dev_node == NULL)
		dev_node = &root;

	if (addr != NULL)
		*addr = of_n_addr_cells(dev_node);
	if (size != NULL)
		*size = of_n_size_cells(dev_node);
}

static const u32 *of_get_address_prop(
	struct device_node	*dev_node,
	int			 index,
	u64			*size,
	u32			*flags,
	const char		*rprop)
{
	const u32	*uint32_prop;
#if __BYTE_ORDER != __BIG_ENDIAN
	u32		*uint32_prop_tmp;
#endif /* __BYTE_ORDER != __BIG_ENDIAN */
	size_t		 lenp;
	u32		 na, ns;

	assert(dev_node != NULL);

	of_bus_default_count_cells(dev_node, &na, &ns);

	uint32_prop = of_get_property(dev_node, rprop, &lenp);
	if (unlikely(uint32_prop == NULL))
		return NULL;

	assert((lenp % ((na + ns) * sizeof(u32))) == 0);

	uint32_prop += (na + ns) * index;
	if (size != NULL)
		for (*size = 0; ns > 0; ns--, na++)
			*size = (*size << 32) + uint32_prop[na];

#if __BYTE_ORDER == __BIG_ENDIAN
	/* do nothing */
#else
	uint32_prop_tmp = (u32 *)uint32_prop;
	if (size != NULL)
		*size = swab64(*size);
	*uint32_prop_tmp = swab32(*uint32_prop_tmp);
#endif /* __BYTE_ORDER == __BIG_ENDIAN */

	return uint32_prop;
}

static struct device_node *find_compatible_node_by_indx(
	const struct device_node	*from,
	const int			 indx,
	const char			*type,
	const char			*compatible)
{
	int			 _err, __err;
	char			 command[PATH_MAX * 2], *full_name;
	FILE			*of_sh;
	struct device_node	*dev_node;

	assert(compatible != NULL);

	if (from == NULL)
		from = &root;

	_err = create_of_sh();
	if (_err)
		return NULL;

	snprintf(command,
		 sizeof(command),
		 "%s of_find_compatible_node \"%s\" \"%s\" %hhu",
		 OF_SH_FILENAME, from->full_name, compatible, indx+1);

	of_sh = popen(command, "r");
	if (unlikely(!of_sh))
		return NULL;

	dev_node = &current_node[current];
	full_name = fgets(dev_node->full_name, sizeof(dev_node->full_name), of_sh);
	if (full_name == NULL) {
		_err = ferror(of_sh);
		if (_err == 0) {
			_err = feof(of_sh);
			if (_err == 0)
				assert(0);
		}
	} else
		_err = 0;
	__err = pclose(of_sh);

	if (unlikely(_err != 0 || __err != 0))
		return NULL;

	dev_node->name = of_basename(dev_node->full_name);
	if (dev_node->name == NULL)
		return NULL;

	current = (current + 1) % ARRAY_SIZE(current_node);

	return dev_node;
}

static int sort_nodes_by_addrs(struct device_node **dev_nodes, u64 *devs_pa, u8 num_nodes)
{
	u8 i, j;

	if (num_nodes < 2)
		return 0;

	for (i = 0; i < num_nodes - 1; i++)
		for (j = 0; j < num_nodes - i - 1; j++)
			if (devs_pa[j] > devs_pa[j + 1]) {
				struct device_node	*tmp_node;
				u64			 tmp_pa;

				tmp_pa = devs_pa[j];
				tmp_node = dev_nodes[j];
				devs_pa[j] = devs_pa[j + 1];
				dev_nodes[j] = dev_nodes[j + 1];
				devs_pa[j + 1] = tmp_pa;
				dev_nodes[j + 1] = tmp_node;
			}

	return 0;
}

struct device_node *of_get_parent(const struct device_node *dev_node)
{
	struct device_node	*_current_node;

	if (dev_node == NULL)
		dev_node = &root;

	*(_current_node = &current_node[current]) = *dev_node;
	_current_node->name = strrchr(_current_node->full_name, '/');
	if (unlikely(_current_node->name == NULL))
		return NULL;
	if (_current_node->name == _current_node->full_name)
		return &root;

	*_current_node->name = 0;
	_current_node->name = of_basename(_current_node->full_name);
	if (unlikely(_current_node->name == NULL))
		return NULL;

	current = (current + 1) % ARRAY_SIZE(current_node);
	return _current_node;
}

void *of_get_property(struct device_node *dev_node, const char *name, size_t *lenp)
{
	int	 _err, __err;
	size_t	 len;
	char	 command[PATH_MAX * 2];
	FILE	*of_sh;

	assert(name != NULL);

	if (dev_node == NULL)
		dev_node = &root;

	_err = create_of_sh();
	if (_err)
		return NULL;

	snprintf(command, sizeof(command), "%s %s \"%s\" \"%s\"",
		 OF_SH_FILENAME, __func__, dev_node->full_name, name);

	of_sh = popen(command, "r");
	if (unlikely(!of_sh))
		return NULL;

	len = fread(dev_node->_property,
		sizeof(*dev_node->_property),
		sizeof(dev_node->_property),
		of_sh);
	_err = len == 0 ? ferror(of_sh) : 0;
	__err = pclose(of_sh);

	if (unlikely(_err != 0 || __err != 0))
		return NULL;

	if (lenp != NULL)
		*lenp = len * sizeof(*dev_node->_property);

	return dev_node->_property;
}

u32 of_n_addr_cells(const struct device_node *dev_node)
{
	struct device_node	*parent_node;
	size_t			 lenp;
	const u32		*na;

	if (dev_node == NULL)
		dev_node = &root;

	do {
		parent_node = of_get_parent(dev_node);

		na = of_get_property(parent_node, "#address-cells", &lenp);
		if (na != NULL) {
			u32	ans;

			assert(lenp == sizeof(u32));
			ans = *na;
#if __BYTE_ORDER == __BIG_ENDIAN
			/* Do nothing */
#else
			ans = swab32(ans);
#endif /* __BYTE_ORDER == __BIG_ENDIAN */
			return ans;
		}
	} while (parent_node != &root);

	return OF_DEFAULT_NA;
}

u32 of_n_size_cells(const struct device_node *dev_node)
{
	struct device_node	*parent_node;
	size_t			 lenp;
	const u32		*ns;

	if (dev_node == NULL)
		dev_node = &root;

	do {
		parent_node = of_get_parent(dev_node);

		ns = of_get_property(parent_node, "#size-cells", &lenp);
		if (ns != NULL) {
			u32	ans;

			assert(lenp == sizeof(u32));
			ans = *ns;
#if __BYTE_ORDER == __BIG_ENDIAN
			/* Do nothing */
#else
			ans = swab32(ans);
#endif /* __BYTE_ORDER == __BIG_ENDIAN */
			return ans;
		}
	} while (parent_node != &root);

	return OF_DEFAULT_NS;
}

const void *of_get_mac_address(struct device_node *dev_node)
{
	void   *addr;

	addr = of_get_property(dev_node, "mac-address", NULL);
	if (addr)
		return addr;

	addr = of_get_property(dev_node, "local-mac-address", NULL);
	if (addr)
		return addr;

	addr = of_get_property(dev_node, "address", NULL);
	if (addr)
		return addr;

	return NULL;
}

const u32 *of_get_address(
	struct device_node	*dev_node,
	int			 index,
	u64			*size,
	u32			*flags)
{
	return of_get_address_prop(dev_node, index, size, flags, "reg");
}

u64 of_translate_address(
	struct device_node	*dev_node,
	const u32		*addr)
{
	u32		 na;
	u64		 phys_addr, prev_addr_offs, curr_addr_offs;
	const u32	*regs_addr;

	assert(dev_node != NULL);

	phys_addr = *addr;
	prev_addr_offs = phys_addr;
	do {
		dev_node = of_get_parent(dev_node);
		if (unlikely(dev_node == NULL)) {
			/* we got to the root; let's break and return */
			phys_addr = 0;
			break;
		}

		/* look for field in the name 'ranges' */
		regs_addr = of_get_address_prop(dev_node, 0, NULL, NULL, "ranges");
		if (regs_addr == NULL) {
			/* if 'ranges' not found, look for field 'regs' */
			regs_addr = of_get_address_prop(dev_node, 0, NULL, NULL, "regs");
			if (regs_addr == NULL)
				/* in that case, there's probably no registers information in this node */
				continue;
		}

		na = of_n_addr_cells(dev_node);
		for (curr_addr_offs = 0; na > 0; na--, regs_addr += 2)
#if __BYTE_ORDER == __BIG_ENDIAN
			curr_addr_offs = (curr_addr_offs << 32) + *regs_addr;
#else
			curr_addr_offs = (curr_addr_offs << 32) + swab64(*regs_addr);
#endif /* __BYTE_ORDER == __BIG_ENDIAN */
		/* TODO: for some reason, we may get the same offset twice;
		 * Do not allow it (skip this cycle)!
		 * We assume that the same offset may not be apeared twice
		 * but as we go upper in the tree, the offsets must get bigger already.
		 */
		if (curr_addr_offs <= prev_addr_offs)
			continue;
		phys_addr += curr_addr_offs;
		prev_addr_offs = curr_addr_offs;
	} while (dev_node != &root);

	return phys_addr;
}

struct device_node *of_find_compatible_node_by_indx(
	const struct device_node	*from,
	const int			 indx,
	const char			*type,
	const char			*compatible)
{
	struct device_node	*dev_nodes[OF_MAX_NODES];
	u64			 devs_pa[OF_MAX_NODES];
	u8			 i, j, num_nodes, num_nodes_skipped;

	/* First, let's collect all nodes available from this compatible */
	num_nodes = 0;
	do {
		dev_nodes[num_nodes] = find_compatible_node_by_indx(from, num_nodes, type, compatible);
		if (!dev_nodes[num_nodes])
			break;
	} while (num_nodes++ < OF_MAX_NODES);

	if (!num_nodes)
		return NULL;

	/* There's a chance that we got some nodes that are not fully match; let's remove them from list */
	num_nodes_skipped = 0;
	for (i = 0; i < num_nodes; i++)
		if (!of_device_is_compatible(dev_nodes[i], compatible)) {
			for (j = i; j < num_nodes - 1; j++)
				dev_nodes[j] = dev_nodes[j + 1];
			num_nodes_skipped++;
		}
	num_nodes -= num_nodes_skipped;

	if (indx >= num_nodes)
		return NULL;

	/* in case we have only 1 device found, no need to sort it */
	if (num_nodes == 1)
		return dev_nodes[0];

	/* Iterate all nodes we found and retrieve their base-address */
	for (i = 0; i < num_nodes; i++) {
		const uint32_t	*uint32_prop;
		u64		 tmp_size;

		uint32_prop = of_get_address(dev_nodes[i], 0, &tmp_size, NULL);
		if (!uint32_prop) {
			/* In case we don't find registers-region already in first entry,
			 * we assume all entires has no regs. in that case, return whatever we found.
			 */
			if (i == 0)
				return dev_nodes[indx];
			/* if this is not the first entry, something is wrong here (since we assume all
			 * entries should look the same)
			 */
			pr_err("registers region (%s @ %d) not found!\n", compatible, i);
			return NULL;
		}
		devs_pa[i] = of_translate_address(dev_nodes[i], uint32_prop);
	}

	/* There's a chance that we got some nodes that are not fully match; let's remove them from list */
	num_nodes_skipped = 0;
	for (i = 0; i < num_nodes; i++)
		if (!devs_pa[i]) {
			for (j = i; j < num_nodes - 1; j++) {
				dev_nodes[j] = dev_nodes[j + 1];
				devs_pa[j] = devs_pa[j + 1];
			}
			num_nodes_skipped++;
		}
	num_nodes -= num_nodes_skipped;

	if (indx >= num_nodes)
		return NULL;

	/* now, we sort the nodes by their addresses (since it is possible that we got the nodes not-sorted) */
	sort_nodes_by_addrs(dev_nodes, devs_pa, num_nodes);

	return dev_nodes[indx];
}

struct device_node *of_find_compatible_node(
	const struct device_node	*from,
	const char			*type,
	const char			*compatible)
{
	return of_find_compatible_node_by_indx(from, 0, type, compatible);
}

struct device_node *of_find_node_by_phandle(phandle ph)
{
	int			 _err, __err;
	char			 command[PATH_MAX], *full_name;
	FILE			*of_sh;
	struct device_node	*dev_node;

	_err = create_of_sh();
	if (_err)
		return NULL;

	snprintf(command,
		 sizeof(command), "%s %s %c",
		 OF_SH_FILENAME,
		 __func__,
		 *((char *)&ph + sizeof(ph) - sizeof(char)));

	of_sh = popen(command, "r");
	if (unlikely(!of_sh))
		return NULL;

	dev_node = &current_node[current];
	full_name = fgets(dev_node->full_name, sizeof(dev_node->full_name), of_sh);
	_err = full_name == NULL ? ferror(of_sh) : 0;
	__err = pclose(of_sh);

	if (unlikely(_err != 0 || __err != 0)) {
		pr_err("command resulted with err (%d, %d)\n", _err, __err);
		return NULL;
	}

	dev_node->name = of_basename(dev_node->full_name);
	if (dev_node->name == NULL)
		return NULL;

	current = (current + 1) % ARRAY_SIZE(current_node);

	return dev_node;
}

int of_device_is_available(struct device_node *dev_node)
{
	size_t		 lenp;
	const char	*status;

	status = of_get_property(dev_node, "status", &lenp);
	if (status == NULL)
		return 1;

	return lenp > 0 &&
		(strcmp(status, "okay") == 0 || strcmp(status, "ok") == 0);
}

int of_device_is_compatible(
	struct device_node	*dev_node,
	const char		*compatible)
{
	size_t		 lenp, len;
	const char	*_compatible;

	_compatible = of_get_property(dev_node, "compatible", &lenp);
	if (unlikely(_compatible == NULL))
		return 0;

	while (lenp > 0) {
		if (strncasecmp(compatible, _compatible, strlen(compatible)+1) == 0)
			return 1;

		len = strlen(_compatible) + 1;
		_compatible += len;
		lenp -= len;
	}

	return 0;
}
