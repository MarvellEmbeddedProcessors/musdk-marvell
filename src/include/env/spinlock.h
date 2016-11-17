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

#ifndef __SPINLOCK_H__
#define __SPINLOCK_H__

#include <pthread.h>
#include "env/mv_debug.h"

#ifndef spinlock_t

#define spinlock_t		pthread_mutex_t

spinlock_t * spin_lock_create(void);
void spin_lock_destroy(spinlock_t *lock);

#define spin_lock_init(_lock)							\
	do {									\
		int err = pthread_mutex_init(_lock, NULL);			\
		if (err)							\
			pr_warn("Failed to initialize spinlock (%d)!", err);	\
	} while (0)

#define spin_lock(_lock)			\
	do {					\
		pthread_mutex_lock(_lock);	\
	} while (0)

#define spin_unlock(_lock)			\
	do {					\
		pthread_mutex_unlock(_lock);	\
	} while (0)

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

#endif /* __SPINLOCK_H__ */
