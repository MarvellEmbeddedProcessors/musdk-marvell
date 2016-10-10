#include <pthread.h>
#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <fcntl.h>
#include <stropts.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <sys/mman.h>

#include <mv_pp_uio.h>

#include "pp2_cma.h"
#include "pp2_print.h"

#define UIO_CMA_DEV "/dev/uio-cma"

static volatile int fd = -1;

int pp2_cma_init(void)
{
	fd = open(UIO_CMA_DEV, O_RDWR);
	if (fd < 0) {
		pp2_err("CMA: open() failed\n");
		return -1;
	}
	else
		return 0;
}

uintptr_t pp2_cma_calloc(size_t size)
{
	struct cma_admin *ptr;
	uint64_t param;
	void *ret;

	if (fd < 0)
		return 0;

	/* Add admin area size */
	size += sizeof(struct cma_admin);

	param = size;

	if (ioctl(fd, PP_UIO_IOC_KEY | PP_IOC_CMA_ALLOC, &param) == -1) {
		pp2_err("CMA: ioctl(PP_IOC_CMA_ALLOC) for size=%lu failed with "
                "error %s \n", size, strerror(errno));
		return 0;
	}

	/* Map admin area with read only righst */
	ret = mmap(NULL, CMA_PAGE_SIZE, PROT_READ , MAP_SHARED, fd, param);

	if (ret == MAP_FAILED) {
		pp2_err("CMA: mmap() failed (%s)\n", strerror(errno));
		return 0;
	}
	ptr = (struct cma_admin *)ret;

	/* Map the buffer payload with R/W rights */
	ret = mmap(NULL, size - CMA_PAGE_SIZE,
			PROT_READ | PROT_WRITE,
			MAP_SHARED,
			fd, param);

	if (ret == MAP_FAILED) {
		pp2_err("CMA: mmap() payload failed (%s)\n", strerror(errno));
		return 0;
	}

	pp2_info("%p mapped to virt address = %lX\n", (void *)ptr->paddr,
			ptr->uvaddr);

	return (uintptr_t) ptr;
}

void pp2_cma_free(uintptr_t buf)
{
	struct cma_admin *ptr = (struct cma_admin *) buf;
	uint64_t kvaddr;

	if (!buf || fd < 0)
		return;

	pp2_info("free %p of %lu bytes\n", ptr, ptr->size);

	/* Save kernel logical address before unmap buffer admin area */
	kvaddr = ptr->kvaddr;

	munmap((void *)ptr->uvaddr, ptr->size - CMA_PAGE_SIZE);
	munmap((void *)ptr, sizeof(*ptr));

	errno = 0;
	if (ioctl(fd, PP_UIO_IOC_KEY | PP_IOC_CMA_FREE, &kvaddr)) {
		if (errno) {
			pp2_err("CMA: ioctl() PP_IOC_CMA_FREE failed\n");
		}
	}
}

void pp2_cma_deinit(void)
{
	if (fd >= 0) {
		close(fd);
		fd = -1;
	}
}

uintptr_t pp2_cma_vaddr(uintptr_t buf)
{
	struct cma_admin *ptr = (struct cma_admin *) buf;

	/* Take in consideration the size of admin area */
	uintptr_t ret = (!ptr) ? (uintptr_t)0 : (uintptr_t) ptr->uvaddr;

	pp2_info("%p va %p\n", ptr, (void *)ret);

	return ret;
}

uintptr_t pp2_cma_paddr(uintptr_t buf)
{
	struct cma_admin *ptr = (struct cma_admin *) buf;

	/* Take in consideration the size of admin area */
	uintptr_t ret = (!ptr) ? (uintptr_t)0 :
		(uintptr_t)((uint8_t *)ptr->paddr + sizeof(*ptr));

	pp2_info("%p pa %p\n", ptr, (void *)ret);

	return ret;
}
