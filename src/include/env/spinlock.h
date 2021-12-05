/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __SPINLOCK_H__
#define __SPINLOCK_H__

#ifndef __KERNEL__
#include "env/mv_debug.h"

#ifndef spinlock_t

typedef struct  spinlock {
	char lock;
} spinlock_t;

static inline void spin_lock_init(spinlock_t *spinlock)
{
	__atomic_clear((&spinlock->lock), __ATOMIC_RELAXED);
}

static inline void spin_lock(spinlock_t *spinlock)
{
	while (__atomic_test_and_set((&spinlock->lock), __ATOMIC_ACQUIRE))
		while (__atomic_load_n((&spinlock->lock), __ATOMIC_RELAXED))
			;
}

static inline int spin_trylock(spinlock_t *spinlock)
{
	return (__atomic_test_and_set((&spinlock->lock), __ATOMIC_ACQUIRE) == 0);
}

static inline void spin_unlock(spinlock_t *spinlock)
{
	__atomic_clear((&spinlock->lock), __ATOMIC_RELEASE);
}

#define spin_lock_irqsave(_lock, _flags)\
	do {				\
		local_irq_save(_flags);	\
		spin_lock(_lock);	\
	} while (0)

#define spin_unlock_irqrestore(_lock, _flags)	\
	do {					\
		local_irq_restore(_flags);	\
		spin_unlock(_lock);		\
	} while (0)

#endif /* !spinlock_t */
#endif /* __KERNEL__ */

spinlock_t *spin_lock_create(void);
void spin_lock_destroy(spinlock_t *lock);

#endif /* __SPINLOCK_H__ */
