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

#include "std_internal.h"
#include "lib/mem_mng.h"

#include <linux/dma-mapping.h>

struct mem_mng	*mm;
dma_addr_t	 __dma_phys_base;
void		*__dma_virt_base;
struct device	*dev;
size_t		 dma_size;

void *mv_sys_dma_mem_alloc(size_t size, size_t align)
{
	u64 ret;

	if (!mm) {
		pr_err("%s: Memory Manager uninitialized.\n", __func__);
		return NULL;
	}

	ret = mem_mng_get(mm, size, align, "temp");

	if (ret == MEM_MNG_ILLEGAL_BASE) {

		pr_err("%s: Memory Manager: Illegal Base.", __func__);
		return NULL;
	}

	return (void *)ret;
}

void mv_sys_dma_mem_free(void *ptr)
{
	if (!mm)
		pr_err("%s: Memory Manager uninitialized.\n", __func__);
	else
		mem_mng_put(mm, (u64)ptr);
}

int mv_sys_dma_mem_init(struct device *adev, u64 size)
{
	int err;

	dev = adev;
	dma_size = size;

	if (mm) {
		dev_err(dev, "Memory Manager already initialized.\n");
		return -EEXIST;
	}

	if (!dev->archdata.dma_coherent)
		dev_warn(dev, "dma_coherent not set. Using coherent mode anyway.\n");

	/* Setting up DMA subsystem */
	dev->dma_mask = devm_kzalloc(dev, sizeof(*dev->dma_mask), GFP_KERNEL);
	err = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (err) {
		dev_err(dev, "Cannot set dma_mask.\n");
		return err;
	}

	/* Allocating DMA memory */
	__dma_virt_base = dma_zalloc_coherent(dev, size, &__dma_phys_base, GFP_KERNEL | GFP_DMA);

	if (!__dma_virt_base) {
		dev_err(dev, "Not enough CMA memory to allocate %llu bytes.\n", size);
		return -ENOMEM;
	}

	/* Giving the memory to manager */
	err = mem_mng_init((u64)__dma_virt_base, size, &mm);

	if (err) {
		dma_free_coherent(dev, size, __dma_virt_base, __dma_phys_base);

		dev_err(dev, "Failed to initialize memory manager.\n");
		return err;
	}

	return 0;
}

void mv_sys_dma_mem_destroy(void)
{
	if (!mm)
		return;

	mem_mng_free(mm);
	dma_free_coherent(dev, dma_size, __dma_virt_base, __dma_phys_base);
}
