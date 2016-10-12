/*
 * User I/O driver for Armada 7K/8K Packet Processor.
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

#ifndef _MV_PP_UIO_H_
#define _MV_PP_UIO_H_

#define UIO_PP_0 "uio_pp_0"
#define UIO_PP_1 "uio_pp_1"

#define PP_UIO_IOC_KEY	0x00002200
#define PP_UIO_IOC_MASK	0x000000ff

/* Supported cmds for the driver. */
enum pp_iocmd {
	PP_IOC_CMA_ALLOC = 1,
	PP_IOC_CMA_FREE,
};

#define CMA_MAGIC_NUMBER	0xAA55AA55
#define CMA_PAGE_SIZE		4096

/*
 * Administration area used to keep information about each contiguous
 * buffer allocated
 */
struct cma_admin {
	union {
		struct {
			uint32_t magic;
			/* Physical memory address of buffer allocated */
			uint64_t paddr;
			/* Kernel virtual memory address */
			uint64_t kvaddr;
			/* User virtual memory address */
			uint64_t uvaddr;
			/* size of buffer with admin area */
			size_t   size;
#ifdef __KERNEL__
			struct list_head list;
#endif
		};
		uint8_t  rsvd[CMA_PAGE_SIZE];
	};
};

#endif
