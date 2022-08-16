/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* a simple stack-based ID allocator */
#ifndef __ID_ALLOC_H__
#define __ID_ALLOC_H__

struct id_alloc_data {
	u32 num_ids;
	u32 stack_idx;
	u32 stack[0];
};

static inline int
id_alloc_get(struct id_alloc_data *iad, u32 *id)
{
	if (iad->stack_idx == 0)
		return -ENOENT;

	iad->stack_idx -= 1;
	*id = iad->stack[iad->stack_idx];

	return 0;
}

static inline int
id_alloc_put(struct id_alloc_data *iad, u32 id)
{
	if (unlikely(id >= iad->num_ids))
		return -EINVAL;

	if (iad->stack_idx == iad->num_ids)
		return -EEXIST;

	iad->stack[iad->stack_idx] = id;
	iad->stack_idx += 1;

	return 0;
}

static inline struct id_alloc_data *
id_alloc_init(uint32_t num_ids)
{
	struct id_alloc_data *iad;
	u32 i;

	if (num_ids == 0) {
		pr_err("%s invalid num_ids %d\n", __func__, num_ids);
		return NULL;
	}

	iad = kmalloc(sizeof(*iad) + num_ids * sizeof(iad->stack[0]), GFP_KERNEL);
	if (iad == NULL) {
		pr_err("%s out of memory\n", __func__);
		return NULL;
	}

	for (i = 0; i < num_ids; i++)
		iad->stack[i] = i;

	iad->stack_idx = num_ids;
	iad->num_ids = num_ids;

	return iad;
}

static inline void
id_alloc_deinit(struct id_alloc_data *iad)
{
	kfree(iad);
}

#endif /* __ID_ALLOC_H__ */
