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

#ifndef __OF_H__
#define __OF_H__

#include <stdint.h>
#include <limits.h> /* PATH_MAX */
#include "env/mv_types.h"

typedef u32	phandle;

struct device_node {
	char	*name;
	char	 full_name[PATH_MAX];

	u8	 _property[64];
};

struct device_node *of_get_parent(const struct device_node *dev_node);

void *of_get_property(struct device_node *from, const char *name, size_t *lenp)
	__attribute__((nonnull(2)));

u32 of_n_addr_cells(const struct device_node *dev_node);
u32 of_n_size_cells(const struct device_node *dev_node);
const void *of_get_mac_address(struct device_node *dev_node);

const u32 *of_get_address(
	struct device_node	*dev_node,
	size_t			 index,
	u64			*size,
	u32			*flags);

u64 of_translate_address(
	struct device_node	*dev_node,
	const u32		*addr)
	__attribute__((nonnull));

struct device_node *of_find_compatible_node(
	const struct device_node	*from,
	const char			*type,
	const char			*compatible)
	__attribute__((nonnull(3)));

struct device_node *of_find_compatible_node_by_indx(
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

struct device_node *of_find_node_by_phandle(phandle ph);

int of_device_is_available(struct device_node *dev_node);
int of_device_is_compatible(
	struct device_node *dev_node,
	const char *compatible);

#endif  /*  __OF_H__ */
