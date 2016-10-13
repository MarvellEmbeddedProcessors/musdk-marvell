/*
 * User I/O driver for Armada SDK.
 *
 * Copyright (C) 2016, ENEA AB
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 2 of the License, or any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef __MUSDK_UIO_IOCTLS_H___
#define __MUSDK_UIO_IOCTLS_H___

#include <asm/ioctl.h>

/**
 * @Collection	MUSDK IOCTL device ('/dev') definitions
 */

/** defines the IOCTL type for all the MUSDK Linux module commands */
#define MUSDK_IOC_TYPE_BASE		0x23
/** Name of the MUSDK-UIO chardev */
#define DEV_MUSDK_NAME			"musdk_uio"
#define DEV_MUSDK_MINOR_BASE	0
#define MUSDK_IOC_NUM(n)		(n)
/* @} */

#define CMA_MAGIC_NUMBER	0xAA55AA55
#define CMA_PAGE_SIZE		4096

/*
 * Administration area used to keep information about each contiguous
 * buffer allocated
 */
struct cma_admin {
	union {
		struct {
			u32					magic;
			/* Physical memory address of buffer allocated */
			u64					paddr;
			/* Kernel virtual memory address */
			u64					kvaddr;
			/* User virtual memory address */
			u64					uvaddr;
			/* size of buffer with admin area */
			size_t				size;
#ifdef __KERNEL__
			struct list_head	list;
#endif /* __KERNEL__ */
		};
		u8						rsvd[CMA_PAGE_SIZE];
	};
};

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
#define MUSDK_IOC_CMA_FREE		_IOW(MUSDK_IOC_TYPE_BASE, MUSDK_IOC_NUM(2), struct cma_admin)

#endif /* __MUSDK_UIO_IOCTLS_H___ */
