/* Copyright (c) 2016 Marvell.
 * SPDX-License-Identifier: GPL-2.0-only
 */

#ifndef __MUSDK_UIO_IOCTLS_H___
#define __MUSDK_UIO_IOCTLS_H___

#include <asm/ioctl.h>

/**
 * @Collection	MUSDK IOCTL device ('/dev') definitions
 */

/** defines the IOCTL type for all the MUSDK Linux module commands */
#define MUSDK_IOC_TYPE_BASE		0x23
#define DEV_MUSDK_MINOR_BASE	0
#define MUSDK_IOC_NUM(n)		(n)
/* @} */

/**
 * Allocate DMA-memory.
 *
 * @param[in]	size(u64)	Size of the memory to be allocated.
 *
 * @return	0 on success.
 */
#define MUSDK_IOC_CMA_ALLOC		_IOW(MUSDK_IOC_TYPE_BASE, MUSDK_IOC_NUM(1), u64)

/**
 * Free an allocated DMA-memory.
 *
 * @param[in]	cma_admin	TODO.
 *
 * @return	0 on success.
 */
#define MUSDK_IOC_CMA_FREE		_IOW(MUSDK_IOC_TYPE_BASE, MUSDK_IOC_NUM(2), u64)

#endif /* __MUSDK_UIO_IOCTLS_H___ */
