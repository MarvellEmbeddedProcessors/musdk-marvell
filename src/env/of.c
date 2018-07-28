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

#include "of_sh.h"


#define OF_DEFAULT_NA 1
#define OF_DEFAULT_NS 1

#define OF_SH_FILENAME	"./of.sh"


static u8			current;
static struct device_node	current_node[16];
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
	u64		 phys_addr, tmp_addr;
	const u32	*regs_addr;

	assert(dev_node != NULL);

	phys_addr = *addr;
	do {
		dev_node = of_get_parent(dev_node);
		if (unlikely(dev_node == NULL)) {
			phys_addr = 0;
			break;
		}

		regs_addr = of_get_address_prop(dev_node, 0, NULL, NULL, "ranges");
		if (regs_addr == NULL)
			continue;

		na = of_n_addr_cells(dev_node);
		for (tmp_addr = 0; na > 0; na--, regs_addr += 2)
#if __BYTE_ORDER == __BIG_ENDIAN
			tmp_addr = (tmp_addr << 32) + *regs_addr;
#else
			tmp_addr = (tmp_addr << 32) + swab64(*regs_addr);
#endif /* __BYTE_ORDER == __BIG_ENDIAN */
		phys_addr += tmp_addr;
		/* TODO: for some reason, the range is initialized in two levels;
		 * therefore, we break after read the first
		 */
		break;
	} while (dev_node != &root);

	return phys_addr;
}

struct device_node *of_find_compatible_node_by_indx(
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
