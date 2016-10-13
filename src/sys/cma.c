
#include <sys/mman.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <fcntl.h>
#include <stropts.h>
#include <stdio.h>
#include <string.h>

#include "mv_std.h"

#include "../../modules/include/musdk_uio_ioctls.h"

#include "lib/uio_helper.h"

#include "cma.h"


#define UIO_CMA_DEV "/dev/uio-cma"


static volatile int fd = -1;


int cma_init(void)
{
	fd = open(UIO_CMA_DEV, O_RDWR);
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

	pr_info("%p mapped to virt address = %lX\n", (void *)ptr->paddr,
			ptr->uvaddr);

	return (uintptr_t) ptr;
}

void cma_free(uintptr_t buf)
{
	struct cma_admin *ptr = (struct cma_admin *) buf;
	uint64_t kvaddr;
	int err;

	if (!buf || fd < 0)
		return;

	pr_info("free %p of %lu bytes\n", ptr, ptr->size);

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

	pr_info("%p va %p\n", ptr, (void *)ret);

	return ret;
}

uintptr_t cma_get_paddr(uintptr_t buf)
{
	struct cma_admin *ptr = (struct cma_admin *) buf;

	/* Take in consideration the size of admin area */
	uintptr_t ret = (!ptr) ? (uintptr_t)0 :
		(uintptr_t)((uint8_t *)ptr->paddr + sizeof(*ptr));

	pr_info("%p pa %p\n", ptr, (void *)ret);

	return ret;
}
