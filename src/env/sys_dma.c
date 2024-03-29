/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "std_internal.h"
#include "lib/mem_mng.h"

#ifdef MVCONF_SYS_DMA_HUGE_PAGE
#include "hugepage_mem.h"
#elif defined MVCONF_SYS_DMA_UIO
#include "cma.h"
#endif /* MVCONF_SYS_DMA_UIO */


#define MEM_DMA_MAX_REGIONS	16

struct sys_dma {
	struct mem_mng	*mm;
	void		*mem;
	void		*dma_virt_base;
	phys_addr_t	dma_phys_base;
	size_t		dma_size;
	int		en;
};

struct sys_mem_dma_region_priv {
	struct mem_mng	*mm;
	void		*mem;
};


struct mv_sys_dma_mem_region	*sys_dma_regions[MEM_DMA_MAX_REGIONS] = {NULL};

phys_addr_t __dma_phys_base = 0;
void *__dma_virt_base = NULL;
size_t __dma_size;
struct sys_dma	*sys_dma = NULL;


/* UIO supports 2 memory allocations types:
 * 1. CMA
 * 2. Huge pages
 */
#ifdef MVCONF_SYS_DMA_HUGE_PAGE
static int init_mem(struct sys_dma *sdma, size_t size)
{
	if (!sdma) {
		pr_err("no SDMA obj!\n");
		return -EINVAL;
	}

	if (sdma->en) {
		pr_err("%s: Memory allocation already initialized!\n" , __func__);
		return -ENOMEM;
	}

	sdma->mem = hugepage_init_mem(size, &sdma->dma_virt_base);
	if (!sdma->mem) {
		pr_err("Failed to allocate DMA memory!\n");
		return -ENOMEM;
	}
	/* Indicate that memory allocation is initialized */
	sdma->en = 1;

	return 0;
}

static void free_mem(struct sys_dma *sdma)
{
	BUG_ON(!sdma);
	if (!sdma->mem)
		return;

	hugepage_free_mem(sdma->mem);
	sdma->mem = NULL;

	/* clear indication of active memory initialization */
	sdma->en = 0;
}

#elif defined MVCONF_SYS_DMA_UIO /* MVCONF_SYS_DMA_HUGE_PAGE */
static int init_mem(struct sys_dma *sdma, size_t size)
{
	BUG_ON(!sdma);

	if (!sdma->en) {
		int err;

		err = cma_init();
		if (err) {
			pr_err("Failed to init DMA memory (%d)!\n", err);
			return err;
		}
		sdma->en = 1;
	}

	sdma->mem = cma_calloc((size_t)size);
	if (!sdma->mem) {
		pr_err("Failed to allocate DMA memory!\n");
		return -ENOMEM;
	}

	sdma->dma_virt_base = (void *)cma_get_vaddr(sdma->mem);
	sdma->dma_phys_base = (phys_addr_t)cma_get_paddr(sdma->mem);
	pr_debug("init_mem dma_phys_base(0x%" PRIdma ")\n", sdma->dma_phys_base);

	sdma->dma_size = (size_t)cma_get_size(sdma->mem);
	return 0;
}

static void free_mem(struct sys_dma *sdma)
{
	BUG_ON(!sdma);
	if (!sdma->dma_virt_base)
		return;
	cma_free(sdma->mem);
}

int mv_sys_dma_mem_get_info(struct mv_sys_dma_mem_info *mem_info)
{
	int err;

	if (mem_info->name) {
		err = cma_get_dev_name(mem_info->name);
		if (err) {
			pr_err("Unable to retrieve sys dma device name\n");
			return -EINVAL;
		}
	}

	mem_info->size = __dma_size;
	mem_info->paddr = __dma_phys_base;
	return 0;
}

#else /* MVCONF_SYS_DMA_UIO */
static int init_mem(struct sys_dma *sdma, size_t size)
{
	WARN_ON(!sdma);
	if (!sdma)
		return -EINVAL;

	sdma->dma_virt_base = kmalloc(size, GFP_KERNEL);
	if (!sdma->dma_virt_base)
		return -ENOMEM;

	sdma->dma_phys_base = (phys_addr_t)sdma->dma_virt_base;
	return 0;
}

static void free_mem(struct sys_dma *sdma)
{
	WARN_ON(!sdma);
	if (!sdma)
		return;
	if (!sdma->dma_virt_base)
		return;
	kfree(sdma->dma_virt_base);
}
#endif /* MVCONF_SYS_DMA_UIO */

int mv_sys_dma_mem_init(size_t size)
{
	struct sys_dma	*i_sys_dma;
	int err;

#ifdef MVCONF_SYSLOG
	/* Enable the logging facility
	 * Temporarily set always print to stderr
	 */
	log_init(1);
#endif

	if (sys_dma) {
		pr_err("Dma object already exits.\n");
		return -EEXIST;
	}
	else {
		i_sys_dma = (struct sys_dma *)kmalloc(sizeof(struct sys_dma), GFP_KERNEL);
		if (!i_sys_dma) {
			pr_err("No mem for sys-dma object\n");
			return -ENOMEM;
		}
		memset(i_sys_dma, 0, sizeof(struct sys_dma));
	}

	if ((err = init_mem(i_sys_dma, size)) != 0)
		return err;

	err = mem_mng_init((u64)(uintptr_t)i_sys_dma->dma_virt_base,
				size, &i_sys_dma->mm);
	if (err != 0)
		return err;

	if (!sys_dma) {
		sys_dma = i_sys_dma;
		__dma_phys_base = sys_dma->dma_phys_base;
		__dma_virt_base = sys_dma->dma_virt_base;
		__dma_size = sys_dma->dma_size;
	}
	pr_debug("[%s] __dma_phys_base(0x%lx) __dma_virt_base(%p) __dma_size (%zu)\n", __func__,
		  __dma_phys_base, __dma_virt_base, __dma_size);
#ifdef DEBUG
	memset(__dma_virt_base, 0xA, size);
#endif
	return 0;
}

void mv_sys_dma_mem_destroy(void)
{
	if (!sys_dma)
		return;

	mem_mng_free(sys_dma->mm);
	free_mem(sys_dma);
	kfree(sys_dma);

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

	return (void *)(uintptr_t)ans;
}

void mv_sys_dma_mem_free(void *ptr)
{
	if (!sys_dma) {
		pr_err("no dma obj (not initialized?)!\n");
		return;
	}

	mem_mng_put(sys_dma->mm, (u64)(uintptr_t)ptr);
}

static bool mem_region_exist(struct mv_sys_dma_mem_region *mem)
{
	int i;

	for (i = 0; i < MEM_DMA_MAX_REGIONS; i++)
		if (sys_dma_regions[i] == mem)
			return true;
	return false;
}

#ifdef MVCONF_SYS_DMA_HUGE_PAGE
static int init_mem_region(struct mv_sys_dma_mem_region *mem)
{
	return (-1);
}

static void free_mem_region(struct mv_sys_dma_mem_region *mem)
{
	return (-1);
}

#elif defined MVCONF_SYS_DMA_UIO /* MVCONF_SYS_DMA_HUGE_PAGE */
static int init_mem_region(struct mv_sys_dma_mem_region *mem)
{
	int err;
	struct sys_mem_dma_region_priv *priv;

	err = cma_init_region(mem->mem_id);
	if (err != 0) {
		pr_err("Failed to init DMA memory region(%d)!\n", mem->mem_id);
		return err;
	}

	priv = mem->priv;
	priv->mem = cma_calloc_region(mem->mem_id, mem->size);
	if (!priv->mem) {
		pr_err("Failed to allocate DMA memory region(%d)!\n", mem->mem_id);
		return -ENOMEM;
	}

	mem->dma_virt_base = (void *)cma_get_vaddr(priv->mem);
	mem->dma_phys_base = (phys_addr_t)cma_get_paddr(priv->mem);

	return 0;
}

static void free_mem_region(struct mv_sys_dma_mem_region *mem)
{
	struct sys_mem_dma_region_priv *priv;

	if (!mem->dma_virt_base)
		return;

	priv = mem->priv;
	cma_free_region(mem->mem_id, priv->mem);
}

#else /* MVCONF_SYS_DMA_UIO */
static int init_mem_region(struct mv_sys_dma_mem_region *mem)
{
	WARN_ON(!mem);
	if (!mem)
		return -EINVAL;

	mem->dma_virt_base = kmalloc(mem->size, GFP_KERNEL);
	if (!mem->dma_virt_base)
		return -ENOMEM;

	mem->dma_phys_base = (phys_addr_t)mem->dma_virt_base;
	return 0;
}

static void free_mem_region(struct mv_sys_dma_mem_region *mem)
{
	if (!mem->dma_virt_base)
		return;
	kfree(mem->dma_virt_base);
}
#endif /* MVCONF_SYS_DMA_UIO */

int mv_sys_dma_mem_region_init(struct mv_sys_dma_mem_region_params *params, struct mv_sys_dma_mem_region **mem)
{
	int i, err;
	struct mv_sys_dma_mem_region *region;
	struct sys_mem_dma_region_priv *priv;

	pr_debug("Start mv_sys_dma_mem_region_init: mem_ID(%d)\n", params->mem_id);

	/* Get next Region */
	for (i = 0; i < MEM_DMA_MAX_REGIONS; i++)
		if (!sys_dma_regions[i])
			break;

	if (i == MEM_DMA_MAX_REGIONS) {
		pr_err("Exceeded maximum number of regions (%d)\n", i);
		return -ENOSPC;
	}
	if (params->mem_id > MV_SYS_DMA_MAX_MEM_ID) {
		pr_err("Exceeded maximum mem_id (%d)\n", params->mem_id);
		return -EINVAL;
	}
	region = kzalloc(sizeof(struct mv_sys_dma_mem_region), GFP_KERNEL);
	if (!region)
		return -ENOMEM;

	priv = kzalloc(sizeof(struct sys_mem_dma_region_priv), GFP_KERNEL);
	if (!priv) {
		kfree(region);
		return -ENOMEM;
	}
	region->priv = priv;
	region->size = params->size;
	region->manage = params->manage;
	region->mem_id = params->mem_id;

	err = init_mem_region(region);
	if (err != 0)
		goto err;

	if (region->manage)
		err = mem_mng_init((u64)(uintptr_t)region->dma_virt_base, region->size, &priv->mm);
	if (err != 0)
		goto err;

	sys_dma_regions[i] = region;

	/* Set Output */
	(*mem) = sys_dma_regions[i];

	return 0;

err:
	pr_err("%s failed\n", __func__);
	free_mem_region(region);
	kfree(priv);
	kfree(region);
	return err;
}

void mv_sys_dma_mem_region_destroy(struct mv_sys_dma_mem_region *mem)
{
	struct sys_mem_dma_region_priv *priv;

	if (!mem || !mem_region_exist(mem)) {
		pr_err("memory region not created\n");
		return;
	}

	priv = mem->priv;
	if (mem->manage)
		mem_mng_free(priv->mm);
	free_mem_region(mem);
	kfree(priv);
	kfree(mem);

}

void *mv_sys_dma_mem_region_alloc(struct mv_sys_dma_mem_region *mem, size_t size, size_t align)
{
	u64	ans;
	struct sys_mem_dma_region_priv *priv;
	static bool warn_once;

	if (!mem) {
		if (!warn_once) {
			pr_warn("(%s) redirected to mv_sys_dma_mem_alloc()\n", __func__);
			warn_once = true;
		}
		return mv_sys_dma_mem_alloc(size, align);
	}

	if (!mem_region_exist(mem)) {
		pr_err("memory region not created\n");
		return NULL;
	}

	priv = mem->priv;
	ans = mem_mng_get(priv->mm, size, align, "temp");
	if (ans == MEM_MNG_ILLEGAL_BASE) {
		pr_err("failed to alloc mem!\n");
		return NULL;
	}

	return (void *)(uintptr_t)ans;
}

void mv_sys_dma_mem_region_free(struct mv_sys_dma_mem_region *mem, void *ptr)
{
	struct sys_mem_dma_region_priv *priv;

	if (!mem) {
		mv_sys_dma_mem_free(ptr);
		return;
	}

	if (!mem_region_exist(mem)) {
		pr_err("memory region not created\n");
		return;
	}

	priv = mem->priv;
	mem_mng_put(priv->mm, (u64)(uintptr_t)ptr);
}

struct mv_sys_dma_mem_region *mv_sys_dma_mem_region_get(u32 mem_id)
{
	int i;

	for (i = 0; i < MEM_DMA_MAX_REGIONS; i++) {
		if (sys_dma_regions[i] && sys_dma_regions[i]->mem_id == mem_id)
			return sys_dma_regions[i];
	}
	return NULL;
}

int mv_sys_dma_mem_region_exist(u32 mem_id)
{
	return (int)cma_region_exist(mem_id);
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

