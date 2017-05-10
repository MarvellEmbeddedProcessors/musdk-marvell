/******************************************************************************
 *      Copyright (C) 2016 Marvell International Ltd.
 *
 *  If you received this File from Marvell, you may opt to use, redistribute
 *  and/or modify this File under the following licensing terms.
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *
 *      * Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *
 *      * Neither the name of Marvell nor the names of its contributors may be
 *        used to endorse or promote products derived from this software
 *        without specific prior written permission.
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

#include "std_internal.h"
#include "drivers/mv_pp2.h"

#define DRIVER_NAME     "musdk"
#define DRIVER_VERSION  "0.1"
#define DRIVER_AUTHOR   "Marvell"
#define DRIVER_DESC     "Marvell User Space Development Kit"

#define MISC_DEV_NAME   "musdk"

#define MUSDK_MODULE_DMA_MEM_SIZE	(48 * 1024 * 1024)

static int __init musdk_init_module(void)
{
	int	err;

	err = mv_sys_dma_mem_init(MUSDK_MODULE_DMA_MEM_SIZE);
	if (err)
		return err;

	/* TODO: complete here …. */

	return 0;
}

static void __exit musdk_cleanup_module(void)
{
	/* TODO: complete here …. */

	mv_sys_dma_mem_destroy();
}

module_init(musdk_init_module);
module_exit(musdk_cleanup_module);

