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


struct sys_dma {
	struct mem_mng	*mm;
	void		*dma_virt_base;
	phys_addr_t	dma_phys_base;
#ifdef MVCONF_SYS_DMA_UIO
	int		en;
	uintptr_t       cma_ptr;
#endif /* MVCONF_SYS_DMA_UIO */
};


phys_addr_t __dma_phys_base = 0;
void *__dma_virt_base = NULL;
struct sys_dma	*sys_dma = NULL;


#ifdef MVCONF_SYS_DMA_UIO
/* TODO: Update pp_cma_calloc to accept u64 */
static int init_mem(struct sys_dma *sdma, u64 size)
{
	BUG_ON(!sdma);
	uintptr_t cma_ptr;

	if (!sdma->en) {
		int err;
		if ((err = cma_init()) != 0) {
			pr_err("Failed to init DMA memory (%d)!\n", err);
			return err;
		}
		sdma->en = 1;
	}

	cma_ptr = cma_calloc((size_t)size);
	if (!cma_ptr) {
		pr_err("Failed to allocate DMA memory!\n");
		return -ENOMEM;
	}

	sdma->dma_virt_base = (void *)cma_get_vaddr(cma_ptr);
	sdma->dma_phys_base = (phys_addr_t)cma_get_paddr(cma_ptr);
	sdma->cma_ptr = cma_ptr;
	return 0;
}

static void free_mem(struct sys_dma *sdma)
{
	BUG_ON(!sdma);
	if (!sdma->dma_virt_base)
		return;
	cma_free(sdma->cma_ptr);
}

#else
static int init_mem(struct sys_dma *sdma, u64 size)
{
        BUG_ON(!sdma);
        sdma->dma_virt_base = malloc(size);
        if (!sdma->dma_virt_base) {
                pr_err("Failed to allocate DMA memory!\n");
                return -ENOMEM;
        }
        sdma->dma_phys_base = (u64)sdma->dma_virt_base;
        return 0;
}

static void free_mem(struct sys_dma *sdma)
{
        BUG_ON(!sdma);
        if (!sdma->dma_virt_base)
                return;
        free(sdma->dma_virt_base);
}
#endif /* MVCONF_SYS_DMA_UIO */

int mv_sys_dma_mem_init(u64 size)
{
	struct sys_dma	*i_sys_dma;
	int err;

	if (sys_dma)
		i_sys_dma = sys_dma;
	else {
		i_sys_dma = (struct sys_dma *)malloc(sizeof(struct sys_dma));
		if (!i_sys_dma) {
			pr_err("no mem for sys-dma obj!\n");
			return -ENOMEM;
		}
		memset(i_sys_dma, 0, sizeof(struct sys_dma));
	}

	if ((err = init_mem(i_sys_dma, size)) != 0)
		return err;

	if ((err = mem_mng_init((u64)i_sys_dma->dma_virt_base,
							size,
							&i_sys_dma->mm)) != 0)
		return err;

	if (!sys_dma) {
		sys_dma = i_sys_dma;
		__dma_phys_base = sys_dma->dma_phys_base;
		__dma_virt_base = sys_dma->dma_virt_base;
	}
	pr_debug("[%s] __dma_phys_base(0x%lx) __dma_virt_base(%p)\n", __FUNCTION__,
		  __dma_phys_base, __dma_virt_base);
#ifdef DEBUG
	for (int i = 0; i < size; i++) {
		*((char *)__dma_virt_base + i) = 0xA;
	}
#endif
	return 0;
}

void mv_sys_dma_mem_destroy(void)
{
	if (!sys_dma)
		return;

	mem_mng_free(sys_dma->mm);
	free_mem(sys_dma);
	free(sys_dma);

	sys_dma = NULL;
	__dma_phys_base = 0;
	__dma_virt_base = NULL;
}

void *mv_sys_dma_mem_alloc(size_t size, size_t align)
{
	u64	ans;

	if (!sys_dma) {
		pr_err("no dma obj (not initialized?)!\n");
		return NULL;
	}

	ans = mem_mng_get(sys_dma->mm, size, align, "temp");
	if (ans == MEM_MNG_ILLEGAL_BASE) {
		pr_err("failed to alloc mem!\n");
		return NULL;
	}

	return (void *)ans;
}

void mv_sys_dma_mem_free(void *ptr)
{
	if (!sys_dma) {
		pr_err("no dma obj (not initialized?)!\n");
		return;
	}

	mem_mng_put(sys_dma->mm, (u64)ptr);
}
