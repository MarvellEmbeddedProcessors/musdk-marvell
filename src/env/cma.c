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


#define MUSDK_DEV_FILE "/dev/uio-cma"


static volatile int fd = -1;


int cma_init(void)
{
	fd = open(MUSDK_DEV_FILE, O_RDWR);
	if (fd < 0) {
		pr_err("CMA: open() failed\n");
		return -1;
	}
	else
		return 0;
}

uintptr_t cma_calloc(size_t size)
{
	struct cma_admin *ptr;
	uint64_t param;
	void *ret;
	int err;

	if (fd < 0)
		return 0;

	/* Add admin area size */
	size += sizeof(struct cma_admin);

	param = size;

	if ((err = ioctl(fd, MUSDK_IOC_CMA_ALLOC, &param)) != 0) {
		pr_err("CMA: ioctl(MUSDK_IOC_CMA_ALLOC) for size=%lu failed with "
			"error %d \n", size, err);
		return 0;
	}

	/* Map admin area with read only righst */
	ret = mmap(NULL, CMA_PAGE_SIZE, PROT_READ , MAP_SHARED, fd, param);

	if (ret == MAP_FAILED) {
		pr_err("CMA: mmap() failed (%d)\n", (int)(uintptr_t)ret);
		return 0;
	}
	ptr = (struct cma_admin *)ret;

	/* Map the buffer payload with R/W rights */
	ret = mmap(NULL, size - CMA_PAGE_SIZE,
			PROT_READ | PROT_WRITE,
			MAP_SHARED,
			fd, param);

	if (ret == MAP_FAILED) {
		pr_err("CMA: mmap() payload failed (%d)\n", (int)(uintptr_t)ret);
		return 0;
	}

	pr_debug("%p mapped to virt address = %lX\n",
		(void *)ptr->paddr, ptr->uvaddr);

	return (uintptr_t) ptr;
}

void cma_free(uintptr_t buf)
{
	struct cma_admin *ptr = (struct cma_admin *) buf;
	uint64_t kvaddr;
	int err;

	if (!buf || fd < 0)
		return;

	pr_debug("free %p of %lu bytes\n", ptr, ptr->size);

	/* Save kernel logical address before unmap buffer admin area */
	kvaddr = ptr->kvaddr;

	munmap((void *)ptr->uvaddr, ptr->size - CMA_PAGE_SIZE);
	munmap((void *)ptr, sizeof(*ptr));

	if ((err = ioctl(fd, MUSDK_IOC_CMA_FREE, &kvaddr)) != 0)
		pr_err("CMA: ioctl() MUSDK_IOC_CMA_FREE failed (%d)\n", err);
}

void cma_deinit(void)
{
	if (fd >= 0) {
		close(fd);
		fd = -1;
	}
}

uintptr_t cma_get_vaddr(uintptr_t buf)
{
	struct cma_admin *ptr = (struct cma_admin *) buf;

	/* Take in consideration the size of admin area */
	uintptr_t ret = (!ptr) ? (uintptr_t)0 : (uintptr_t) ptr->uvaddr;

	pr_debug("%p va %p\n", ptr, (void *)ret);

	return ret;
}

uintptr_t cma_get_paddr(uintptr_t buf)
{
	struct cma_admin *ptr = (struct cma_admin *) buf;

	/* Take in consideration the size of admin area */
	uintptr_t ret = (!ptr) ? (uintptr_t)0 :
		(uintptr_t)((uint8_t *)ptr->paddr + sizeof(*ptr));

	pr_debug("%p pa %p\n", ptr, (void *)ret);

	return ret;
}
