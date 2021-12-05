/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <linux/dma-mapping.h>

#include "std_internal.h"
#include "env/mv_sys_dma.h"
#include "lib/mem_mng.h"


#define MUSDK_DEV_FILE "musdk-cma"

#ifndef NOTUSED
#define NOTUSED(_a) ((_a) = (_a))
#endif /* !NOTUSED */


struct mem_mng	*mm;
dma_addr_t	 __dma_phys_base;
void		*__dma_virt_base;
struct device	*dev;
size_t		 __dma_size;


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

int mv_sys_dma_mem_init(struct device *adev, size_t size)
{
	int err;

	dev = adev;
	__dma_size = size;

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
		dev_err(dev, "Not enough CMA memory to allocate %u bytes.\n", (unsigned int)size);
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
	dma_free_coherent(dev, __dma_size, __dma_virt_base, __dma_phys_base);
}

int mv_sys_dma_mem_get_info(struct mv_sys_dma_mem_info *mem_info)
{
	if (mem_info->name)
		strcpy(mem_info->name, MUSDK_DEV_FILE);

	mem_info->size = __dma_size;
	mem_info->paddr = __dma_phys_base;
	return 0;
}

void *mv_sys_dma_mem_phys2virt(phys_addr_t pa)
{
	return (void *)(uintptr_t)((pa - __dma_phys_base) + (phys_addr_t)(uintptr_t)__dma_virt_base);
}

phys_addr_t mv_sys_dma_mem_virt2phys(void *va)
{
	return ((phys_addr_t)(uintptr_t)va - (phys_addr_t)(uintptr_t)__dma_virt_base) + __dma_phys_base;
}

int mv_sys_dma_virt_is_valid(void *va)
{
	if ((va >= __dma_virt_base) && (va < (void *)((uintptr_t)__dma_virt_base + __dma_size)))
		return 1;

	return 0;
}

int mv_sys_dma_phys_is_valid(phys_addr_t pa)
{
	if ((pa >= __dma_phys_base) && (pa < __dma_phys_base + __dma_size))
		return 1;

	return 0;
}

int mv_sys_dma_mem_region_init(struct mv_sys_dma_mem_region_params *params, struct mv_sys_dma_mem_region **mem)
{
	NOTUSED(params); NOTUSED(mem);
	pr_err("%s not supported!\n", __func__);
	return -ENOTSUPP;
}

void mv_sys_dma_mem_region_destroy(struct mv_sys_dma_mem_region *mem)
{
	NOTUSED(mem);
	pr_err("%s not supported!\n", __func__);
}

void *mv_sys_dma_mem_region_alloc(struct mv_sys_dma_mem_region *mem, size_t size, size_t align)
{
	static bool warn_once;

	if (!mem) {
		if (!warn_once) {
			pr_warn("(%s) redirected to mv_sys_dma_mem_alloc()\n", __func__);
			warn_once = true;
		}
		return mv_sys_dma_mem_alloc(size, align);
	}

	pr_err("%s not supported!\n", __func__);
	return NULL;
}

void mv_sys_dma_mem_region_free(struct mv_sys_dma_mem_region *mem, void *ptr)
{
	if (!mem) {
		mv_sys_dma_mem_free(ptr);
		return;
	}

	NOTUSED(ptr);
	pr_err("%s not supported!\n", __func__);
}

struct mv_sys_dma_mem_region *mv_sys_dma_mem_region_get(u32 mem_id)
{
	NOTUSED(mem_id);
	pr_warn("%s not supported!\n", __func__);
	return NULL;
}

int mv_sys_dma_mem_region_exist(u32 mem_id)
{
	NOTUSED(mem_id);
	pr_warn("%s not supported!\n", __func__);
	return 0;
}
