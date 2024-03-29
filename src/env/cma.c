/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "std_internal.h"
#include "../../modules/include/musdk_cma_ioctls.h"

#include "env/cma.h"
#include "lib/lib_misc.h"

#define MUSDK_DEV_FILE "/dev/musdk-cma"
#define MUSDK_REGION_DEV_FILE "/dev/musdk-cma-reg%u"
#define MUSDK_MAX_MEM_REGIONS	2

static int fd = -1;

static int region_fd[MUSDK_MAX_MEM_REGIONS] = {
	[0 ... (MUSDK_MAX_MEM_REGIONS - 1)] = -1
};

struct cma_buf_info {
	void *uvaddr;
	phys_addr_t paddr;
	size_t size;
};

int cma_init(void)
{
	if (fd >= 0)
		return 0;

	fd = open(MUSDK_DEV_FILE, O_RDWR);

	if (fd < 0) {
		pr_err("CMA: open() failed\n");
		return -1;
	}
	return 0;
}

bool cma_region_exist(int mem_id)
{
	char buf[32];

	if (mem_id >= MUSDK_MAX_MEM_REGIONS)
		return 0;

	if (region_fd[mem_id] >= 0)
		return true;

	sprintf(buf, MUSDK_REGION_DEV_FILE, mem_id);

	region_fd[mem_id] = open(buf, O_RDWR);
	if (region_fd[mem_id] < 0)
		return false;

	cma_deinit_region(mem_id);
	return true;
}

int cma_init_region(int mem_id)
{
	char buf[32];

	if (mem_id >= MUSDK_MAX_MEM_REGIONS) {
		pr_err("CMA: Init failed, invalid mem_id(%d)\n", mem_id);
		return -1;
	}
	if (region_fd[mem_id] >= 0)
		return 0;

	sprintf(buf, MUSDK_REGION_DEV_FILE, mem_id);

	region_fd[mem_id] = open(buf, O_RDWR);
	if (region_fd[mem_id] < 0) {
		pr_err("CMA region %d: open() failed\n", mem_id);
		return -1;
	}
	return 0;
}


static void *cma_calloc_internal(int fd, size_t size)
{
	struct cma_buf_info *ptr;
	u64 param;
	off_t pgoff;
	void *ret;
	int err;

	if (fd < 0)
		return NULL;

	ptr = kcalloc(1, sizeof(struct cma_buf_info), GFP_KERNEL);
	if (!ptr)
		return NULL;

	param = size;
	if ((err = ioctl(fd, MUSDK_IOC_CMA_ALLOC, &param)) != 0) {
		pr_err("CMA: ioctl(MUSDK_IOC_CMA_ALLOC) failed. size=%zu, error=%d\n",
			size, err);
		kfree(ptr);
		return 0;
	}
	/* Alloc IOCTL return physical address of allocated CMA memory */
	pgoff = param;

	/* Map the CMA buffer payload with R/W rights */
	ret = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, pgoff);
	if (ret == MAP_FAILED) {
		pr_err("CMA: mmap() payload failed (%d)\n", (int)(uintptr_t)ret);
		kfree(ptr);
		return 0;
	}
	ptr->uvaddr = ret;
	ptr->paddr = (phys_addr_t)param;
	ptr->size = size;

	pr_debug("%s: cma_buf_info = %p, uvaddr=%p, paddr=0x%lx, size=%ld\n",
		__func__, ptr, ptr->uvaddr, ptr->paddr, ptr->size);

	return (void *)ptr;
}

void *cma_calloc(size_t size)
{

	return cma_calloc_internal(fd, size);
}

void *cma_calloc_region(int mem_id, size_t size)
{
	if (mem_id >= MUSDK_MAX_MEM_REGIONS) {
		pr_err("CMA: Alloc failed, invalid mem_id(%d)\n", mem_id);
		return NULL;
	}
	return cma_calloc_internal(region_fd[mem_id], size);
}



static void cma_free_internal(int fd, void *handle)
{
	struct cma_buf_info *ptr = (struct cma_buf_info *)handle;
	u64 paddr;
	int err;

	if (!ptr || fd < 0)
		return;

	pr_debug("free %p of %zu bytes\n", ptr, ptr->size);

	paddr = (u64)ptr->paddr;

	munmap(ptr->uvaddr, ptr->size);

	err = ioctl(fd, MUSDK_IOC_CMA_FREE, &paddr);
	if (err)
		pr_err("CMA: ioctl() MUSDK_IOC_CMA_FREE failed (%d)\n", err);

	kfree(ptr);
}

void cma_free(void *handle)
{
	cma_free_internal(fd, handle);
}

void cma_free_region(int mem_id, void *handle)
{
	if (mem_id >= MUSDK_MAX_MEM_REGIONS)
		pr_err("CMA: Free failed, invalid mem_id(%d)\n", mem_id);

	cma_free_internal(region_fd[mem_id], handle);
}


void cma_deinit(void)
{
	if (fd >= 0) {
		close(fd);
		fd = -1;
	}
}

void cma_deinit_region(int mem_id)
{
	if (region_fd[mem_id] >= 0) {
		close(region_fd[mem_id]);
		region_fd[mem_id] = -1;
	}
}


void *cma_get_vaddr(void *handle)
{
	struct cma_buf_info *ptr = (struct cma_buf_info *)handle;

	if (!ptr)
		return NULL;

	pr_debug("cma_buf_info = %p, vaddr = %p\n", ptr, ptr->uvaddr);

	return ptr->uvaddr;
}

phys_addr_t cma_get_paddr(void *handle)
{
	struct cma_buf_info *ptr = (struct cma_buf_info *)handle;

	if (!ptr)
		return 0;

	pr_debug("cma_admin = %p, paddr = 0x%lx\n", ptr, ptr->paddr);

	return ptr->paddr;
}

int cma_get_dev_name(char *dev_name)
{
	strcpy(dev_name, MUSDK_DEV_FILE);
	return 0;
}

int cma_get_dev_name_region(int mem_id, char *dev_name)
{
	char buf[32];

	sprintf(buf, MUSDK_REGION_DEV_FILE, mem_id);
	strcpy(dev_name, buf);
	return 0;
}


size_t cma_get_size(void *handle)
{
	struct cma_buf_info *ptr = (struct cma_buf_info *)handle;

	if (!ptr)
		return 0;

	return ptr->size;
}

