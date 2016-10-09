/**************************************************************************//**
	Copyright (C) 2016 Marvell International Ltd.
*//***************************************************************************/

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <assert.h>

#include "mv_std.h"

#include "lib/of.h"


#define OF_DEFAULT_NA 1
#define OF_DEFAULT_NS 1


static uint8_t				current = 0;
static struct device_node	current_node[16];
static struct device_node	root =
	{
		.full_name	= "/",
		.name		= root.full_name
	};


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

static void of_bus_default_count_cells(const struct device_node *dev_node,
					uint32_t		 *addr,
					uint32_t		 *size)
{
	if (dev_node == NULL)
		dev_node = &root;

	if (addr != NULL)
		*addr = of_n_addr_cells(dev_node);
	if (size != NULL)
		*size = of_n_size_cells(dev_node);
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
	int		 _err, __err;
	size_t	 len;
	char	 command[PATH_MAX];
	FILE	*of_sh;

	assert(name != NULL);

	if (dev_node == NULL)
		dev_node = &root;

	snprintf(command, sizeof(command), "./of.sh %s \"%s\" \"%s\"",
			__func__, dev_node->full_name, name);

	of_sh = popen(command, "r");
	if (unlikely(of_sh == NULL))
		return NULL;

	len = fread(dev_node->_property,
				sizeof(*dev_node->_property),
				sizeof(dev_node->_property), of_sh);
	_err = len == 0 ? ferror(of_sh) : 0;
	__err = pclose(of_sh);

	if (unlikely(_err != 0 || __err != 0))
		return NULL;

	if (lenp != NULL)
		*lenp = len * sizeof(*dev_node->_property);

	return dev_node->_property;
}

uint32_t of_n_addr_cells(const struct device_node *dev_node)
{
	struct device_node  *parent_node;
	size_t		   lenp;
	const uint32_t	  *na;

	if (dev_node == NULL)
		dev_node = &root;

	do {
		parent_node = of_get_parent(dev_node);

		na = of_get_property(parent_node, "#address-cells", &lenp);
		if (na != NULL) {
			assert(lenp == sizeof(uint32_t));

			return *na;
		}
	} while (parent_node != &root);

	return OF_DEFAULT_NA;
}

uint32_t of_n_size_cells(const struct device_node *dev_node)
{
	struct device_node	*parent_node;
	size_t				 lenp;
	const uint32_t		*ns;

	if (dev_node == NULL)
		dev_node = &root;

	do {
		parent_node = of_get_parent(dev_node);

		ns = of_get_property(parent_node, "#size-cells", &lenp);
		if (ns != NULL) {
			assert(lenp == sizeof(uint32_t));

			return *ns;
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

const uint32_t *of_get_address(
	struct device_node	*dev_node,
	size_t				 index,
	uint64_t			*size,
	uint32_t			*flags)
{
	const uint32_t	*uint32_prop;
	size_t			 lenp;
	uint32_t		 na, ns;

	assert(dev_node != NULL);

	of_bus_default_count_cells(dev_node, &na, &ns);

	uint32_prop = of_get_property(dev_node, "reg", &lenp);
	if (unlikely(uint32_prop == NULL))
		return NULL;
	assert((lenp % ((na + ns) * sizeof(uint32_t))) == 0);

	uint32_prop += (na + ns) * index;
	if (size != NULL)
		for (*size = 0; ns > 0; ns--, na++)
			*size = (*size << 32) + uint32_prop[na];
	return uint32_prop;
}

uint64_t of_translate_address(
	struct device_node	*dev_node,
	const uint32_t		*addr)
{
	uint32_t		 na;
	uint64_t		 phys_addr, tmp_addr;
	const uint32_t	*regs_addr;

	assert(dev_node != NULL);

	phys_addr = *addr;
	do {
		dev_node = of_get_parent(dev_node);
		if (unlikely(dev_node == NULL)) {
			phys_addr = 0;
			break;
		}

		regs_addr = of_get_address(dev_node, 0, NULL, NULL);
		if (regs_addr == NULL)
			break;

		na = of_n_addr_cells(dev_node);
		for (tmp_addr = 0; na > 0; na--, regs_addr++)
			tmp_addr = (tmp_addr << 32) + *regs_addr;
		phys_addr += tmp_addr;

	} while (dev_node != &root);

	return phys_addr;
}

struct device_node *of_find_compatible_node_by_indx(
	const struct device_node	*from,
	const int					 indx,
	const char					*type,
	const char					*compatible)
{
	int					 _err, __err;
	char				 command[PATH_MAX], *full_name;
	FILE				*of_sh;
	struct device_node	*dev_node;

	assert(compatible != NULL);

	if (from == NULL)
		from = &root;

	snprintf(command,
			 sizeof(command),
			 "./of.sh of_find_compatible_node \"%s\" \"%s\" %hhu",
			 from->full_name, compatible, indx);

	of_sh = popen(command, "r");
	if (unlikely(of_sh == NULL))
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
	const char					*type,
	const char					*compatible)
{
	return of_find_compatible_node_by_indx(from, 1, type, compatible);
}

struct device_node *of_find_node_by_phandle(phandle ph)
{
	int					 _err, __err;
	char				 command[PATH_MAX], *full_name;
	FILE				*of_sh;
	struct device_node	*dev_node;

	snprintf(command,
			sizeof(command), "./of.sh %s %c",
			__func__,
			*((char *)&ph + sizeof(ph) - sizeof(char)));

	of_sh = popen(command, "r");
	if (unlikely(of_sh == NULL))
		return NULL;

	dev_node = &current_node[current];
	full_name = fgets(dev_node->full_name, sizeof(dev_node->full_name), of_sh);
	_err = full_name == NULL ? ferror(of_sh) : 0;
	__err = pclose(of_sh);

	if (unlikely(_err != 0 || __err != 0))
		return NULL;

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
	struct device_node *dev_node,
	const char *compatible)
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
