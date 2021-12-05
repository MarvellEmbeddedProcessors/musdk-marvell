/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "std_internal.h"

spinlock_t * spin_lock_create(void)
{
	spinlock_t *lock = kmalloc(sizeof(spinlock_t), GFP_KERNEL);

	spin_lock_init(lock);
	return (spinlock_t *)lock;
}

void spin_lock_destroy(spinlock_t *lock)
{
	kfree(lock);
}

