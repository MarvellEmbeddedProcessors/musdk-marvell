/**************************************************************************//**
	Copyright (C) 2016 Marvell International Ltd.
*//***************************************************************************/

#include "std_internal.h"



spinlock_t * spin_lock_create(void)
{
    pthread_mutex_t *mutex = (pthread_mutex_t *)malloc(sizeof(pthread_mutex_t));
    int err = pthread_mutex_init(mutex, NULL);
    if (err) return NULL;

    return (spinlock_t *)mutex;
}

void spin_lock_destroy(spinlock_t *lock)
{
	pthread_mutex_t *mutex = (pthread_mutex_t *)lock;
	if (mutex)
		free(mutex);
}
