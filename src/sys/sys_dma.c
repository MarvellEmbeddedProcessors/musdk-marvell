/**************************************************************************//**
	Copyright (C) 2016 Marvell International Ltd.
*//***************************************************************************/

#include <string.h>

#include "mem_mng.h"
#include "sys_dma.h"


struct sys_dma {
	struct mem_mng		*mm;
	void				*dma_virt_base;
	phys_addr_t			 dma_phys_base;
};


phys_addr_t __dma_phys_base = 0;
void *__dma_virt_base = NULL;
struct sys_dma	*sys_dma = NULL;


/* TODO: Temporary place holder until we'll have UIO/VFIO mechanism
 * to allocate big contigous memory
 */
static int init_mem_simple_us(struct sys_dma *sdma, u64 size)
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

static void free_mem_simple_us(struct sys_dma *sdma)
{
	BUG_ON(!sdma);
	if (!sdma->dma_virt_base)
		return;
	free(sdma->dma_virt_base);
}


int mv_sys_dma_mem_init(u64 size)
{
	struct sys_dma	*i_sys_dma;
	int				 err;

	i_sys_dma = (struct sys_dma *)malloc(sizeof(struct sys_dma));
	if (!i_sys_dma) {
		pr_err("no mem for sys-dma obj!\n");
		return -ENOMEM;
	}
	memset(i_sys_dma, 0, sizeof(struct sys_dma));

	if ((err = init_mem_simple_us(i_sys_dma, size)) != 0)
		return err;

	if ((err = mem_mng_init((u64)i_sys_dma->dma_virt_base,
							size,
							&i_sys_dma->mm)) != 0)
		return err;

	sys_dma = i_sys_dma;
	__dma_phys_base = sys_dma->dma_phys_base;
	__dma_virt_base = sys_dma->dma_virt_base;
	return 0;
}

void mv_sys_dma_mem_destroy(void)
{
	if (!sys_dma)
		return;

	mem_mng_free(sys_dma->mm);
	free_mem_simple_us(sys_dma);
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
