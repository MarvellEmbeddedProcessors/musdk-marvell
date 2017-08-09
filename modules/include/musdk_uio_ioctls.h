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
