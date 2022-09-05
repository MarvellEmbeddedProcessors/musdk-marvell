/************************************************************************
*  Copyright (c) 2018 Marvell.
*
*  This program is free software: you can redistribute it and/or
*  modify it under the terms of the GNU General Public License as
*  published by the Free Software Foundation, either version 2 of the
*  License, or any later version.
*
*  This program is distributed in the hope that it will be useful, but
*  WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
*  General Public License for more details.
*
*******************************************************************************/

#ifndef _MV_DP_INCLUDES_H_
#define _MV_DP_INCLUDES_H_


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/hardirq.h>
#include <linux/types.h>
#include <linux/io.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>

#ifdef __cplusplus
extern "C" {
#endif



#ifdef MV_DP_DEBUG
	#define MV_DP_PHYS_TO_VIRT(addr)	(_mv_dp_phys_to_virt((addr)))
	#define MV_DP_VIRT_TO_PHYS(addr)	(_mv_dp_virt_to_phys((addr)))
#else
	#define MV_DP_PHYS_TO_VIRT(addr)	(phys_to_virt((u64)(addr)))
	#define MV_DP_VIRT_TO_PHYS(addr)	((void *)virt_to_phys((addr)))
#endif


#define mv_dma_single_cpu_to_dev(ptr, size, dir)

/* extern void ___dma_single_cpu_to_dev(const void *, size_t, enum dma_data_direction); */



#define MV_DP_LOCK_SMP(lock, flags)					\
do {								\
	if (in_interrupt())					\
		spin_lock((lock));				\
	else							\
		spin_lock_irqsave((lock), (flags));		\
} while (0)

#define MV_DP_UNLOCK_SMP(lock, flags)				\
do {								\
	if (in_interrupt())					\
		spin_unlock((lock));				\
	else							\
		spin_unlock_irqrestore((lock), (flags));	\
} while (0)

#define MV_DP_LIGHT_LOCK(flags)					\
do {								\
	if (!in_interrupt())					\
		local_irq_save(flags);				\
} while (0)

#define MV_DP_LIGHT_UNLOCK(flags)				\
do {								\
	if (!in_interrupt())					\
		local_irq_restore(flags);			\
} while (0)


/*type=1 -> shared ->spinlock*/
#define MV_DP_LOCK(lock, flags) \
do { \
	if ((mv_dp_instance.ch_mode))\
		MV_DP_LOCK_SMP((lock), (flags));\
	else\
		MV_DP_LIGHT_LOCK((flags));\
} while (0)

#define MV_DP_UNLOCK(lock, flags) \
do { \
	if ((mv_dp_instance.ch_mode))\
		MV_DP_UNLOCK_SMP((lock), (flags));\
	else\
		MV_DP_LIGHT_UNLOCK((flags));\
} while (0)


#define MV_DP_F_CFH_MSG_ACK	(0x1)
#define MV_DP_F_CFH_EXT_HDR	(0x2)
#define MV_DP_MAX_CHAN_NUM	16


#define mv_dp_chan_create(client_id, size, rcv_cb)

#define mv_dp_chan_delete(chan)

#define mv_dp_channel_reg(chan, cpu_num)


#define MV_DP_CH_ID_IS_OK(c)	(((c) >= 0) && (c) < MV_DP_MAX_CHAN_NUM)






#ifdef __cplusplus
}
#endif


#endif
