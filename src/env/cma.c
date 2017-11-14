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
#include "../../modules/include/musdk_uio_ioctls.h"

#include "env/cma.h"
#include "lib/lib_misc.h"

#define MUSDK_DEV_FILE "/dev/uio-cma"


static volatile int fd = -1;

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
	} else
		return 0;
}

void *cma_calloc(size_t size)
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

void cma_free(void *handle)
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

void cma_deinit(void)
{
	if (fd >= 0) {
		close(fd);
		fd = -1;
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
	memcpy(dev_name, MUSDK_DEV_FILE, sizeof(MUSDK_DEV_FILE));
	return 0;
}

size_t cma_get_size(void *handle)
{
	struct cma_buf_info *ptr = (struct cma_buf_info *)handle;

	if (!ptr)
		return 0;

	return ptr->size;
}

