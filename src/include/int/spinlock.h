/**************************************************************************//**
	Copyright (C) 2016 Marvell International Ltd.
*//***************************************************************************/

#ifndef __SPINLOCK_H__
#define __SPINLOCK_H__

#include <pthread.h>

#include "int/io.h"


#define spinlock_t		pthread_mutex_t

spinlock_t * spin_lock_create(void);
void spin_lock_destroy(spinlock_t *lock);

#define spin_lock_init(_lock)										\
	do {															\
		int err = pthread_mutex_init(_lock, NULL);					\
		if (err)													\
			pr_warn("Failed to initialize spinlock (%d)!", err);	\
	} while (0)

#define spin_lock(_lock)			\
	do {							\
		pthread_mutex_lock(_lock);	\
	} while (0)

#define spin_unlock(_lock)			\
	do {							\
		pthread_mutex_unlock(_lock);\
	} while (0)

#define spin_lock_irqsave(_lock, _flags)\
	do {								\
		local_irq_save(_flags);			\
		spin_lock(_lock);				\
	} while (0)

#define spin_unlock_irqrestore(_lock, _flags)	\
	do {										\
		local_irq_restore(_flags);				\
		spin_unlock(_lock);						\
	} while (0)

#endif /* __SPINLOCK_H__ */
