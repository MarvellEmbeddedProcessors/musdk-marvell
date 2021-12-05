/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __MV_OF_H__
#define __MV_OF_H__

#include <stdint.h>
#include <limits.h> /* PATH_MAX */
#include "env/mv_types.h"

typedef u32	phandle;

struct device_node {
	char	*name;
	char	 full_name[PATH_MAX];

	u8	 _property[64];
};

struct device_node *mv_of_get_parent(const struct device_node *dev_node);

void *mv_of_get_property(struct device_node *from, const char *name, size_t *lenp)
	__attribute__((nonnull(2)));

u32 mv_of_n_addr_cells(const struct device_node *dev_node);
u32 mv_of_n_size_cells(const struct device_node *dev_node);
const void *mv_of_get_mac_address(struct device_node *dev_node);

const u32 *mv_of_get_address(
	struct device_node	*dev_node,
	int			 index,
	u64			*size,
	u32			*flags);

u64 mv_of_translate_address(
	struct device_node	*dev_node,
	const u32		*addr)
	__attribute__((nonnull));

struct device_node *mv_of_find_compatible_node(
	const struct device_node	*from,
	const char			*type,
	const char			*compatible)
	__attribute__((nonnull(3)));

struct device_node *mv_of_find_compatible_node_by_indx(
	const struct device_node	*from,
	const int			 indx,
	const char			*type,
	const char			*compatible)
	__attribute__((nonnull(4)));

#define for_each_compatible_node(_dev_node, _type, _compatible)				\
	int local_index_node;								\
	for (local_index_node = 1,							\
		 _dev_node = of_find_compatible_node_by_indx(NULL,			\
							     local_index_node,		\
							     _type, _compatible);	\
		 _dev_node;							\
		 _dev_node = of_find_compatible_node_by_indx(NULL,			\
							     ++local_index_node,	\
							     _type, _compatible))

struct device_node *mv_of_find_node_by_phandle(phandle ph);

int mv_of_device_is_available(struct device_node *dev_node);
int mv_of_device_is_compatible(
	struct device_node *dev_node,
	const char *compatible);

#endif  /*  __OF_H__ */
