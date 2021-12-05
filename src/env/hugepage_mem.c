/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "std_internal.h"
#ifdef MVCONF_SYS_DMA_HUGE_PAGE
#include <sys/shm.h>
#include <sys/param.h>
#include "hugepage_mem.h"

/* sys_hugepage - per each allocated memory section
 * size:		Total allocated memory in section
 * shm_id:		Allocated memory range ID
 * huge_page_size:	Kernel Huge Page size
 * shm_va[]:		Virtual Address array -  per each huge page page
 * shm_pa[]:		Physical Address array - per each huge page page
 */
struct sys_hugepage {
	u64	size;
	u64	shm_id;
	u64 huge_page_size;
	void *shm_va[HUGE_PAGE_MAX_PAGE_COUNT];
	phys_addr_t shm_pa[HUGE_PAGE_MAX_PAGE_COUNT];
};

#define ADDR (void *)(0x0UL)
struct sys_hugepage *hugepage_struct;

/* Translation of Physical to Virtual address for an allocated DMA memory address:
 * Scan predefined PA array (shm_pa), and find page number of request PA of page base.
 * Then with the help of predefined VA array (shm_va), return VA of that page base,
 * with addition of preserved page offset.
 */
void *mv_sys_dma_mem_phys2virt(phys_addr_t pa)
{
	phys_addr_t page_offset = pa & (hugepage_struct->huge_page_size - 1);

	/* Scan for page base only */
	pa = pa & ~page_offset;

	/* Go over Physical Address array and find request PA base, return PA + page offset */
	for (int i = 0 ; i < HUGE_PAGE_MAX_PAGE_COUNT; i++)
		if (pa == (phys_addr_t)(hugepage_struct->shm_pa[i]))
			return hugepage_struct->shm_va[i] + page_offset;

	return 0;
}

/* Translation of Virtual to Physical address for an allocated DMA memory address */
phys_addr_t mv_sys_dma_mem_virt2phys(void *va)
{
	/* VA range is contiguous, so decrease VA base from requested VA to get VA offset,
	 * divide offset by huge page size, to get huge page number
	 * with page number, get Physical address of page, and add address offset
	 */
	u64 huge_page_num = (va - __dma_virt_base) / hugepage_struct->huge_page_size;

	return (phys_addr_t)(hugepage_struct->shm_pa[huge_page_num] +
			((unsigned long int)va & (hugepage_struct->huge_page_size - 1)));
}
/* Get VA, scan process page map for VA, and get it's PA */
intptr_t hugepage_virt2phys_scan_proc_pagemap(void *vaddr)
{
	FILE *pagemap;
	phys_addr_t paddr = 0;
	u64 page_entry, offset = ((u64)vaddr / sysconf(_SC_PAGESIZE)) * sizeof(unsigned long int);

	pagemap = fopen("/proc/self/pagemap", "r");
	if (!pagemap) {
		pr_err("Hugepage: Cannot open /proc/self/pagemap: %s.\n", strerror(errno));
		return -EINVAL;
	}

	/* Search requested page entry in proccess pagemap */
	if (!(lseek(fileno(pagemap), offset, SEEK_SET) == offset))
		goto error_close_pagemap;

	/* Once found, read page entry */
	if (!(fread(&page_entry, sizeof(u64), 1, pagemap)))
		goto error_close_pagemap;

	/* Is page present ? */
	if (!(page_entry & (1ULL << 63)))
		goto error_close_pagemap;

	/* pfn mask */
	paddr = page_entry & ((1ULL << 54) - 1);
	paddr = paddr * sysconf(_SC_PAGESIZE);

	/* Add offset within page */
	paddr = paddr | ((uintptr_t)vaddr & (sysconf(_SC_PAGESIZE) - 1));

	fclose(pagemap);
	return (intptr_t)paddr;

error_close_pagemap:
	fclose(pagemap);
	return -EINVAL;
}

/* 1. Retrieve and returns Huge Page size (parse 'Hugepagesize' entry from /proc/meminfo)
 * 2. Verify there is enough free huge pages allocated by kernel (parse 'HugePages_Free')
 */
u64 hugepage_verify_hugepage_pagesize(u64 size)
{
	FILE   *in;
	u64 huge_page_size = 0, huge_page_free = 0;
	int huge_page_count;

	in = fopen("/proc/meminfo", "rb");
	if (!in) {
		pr_err("Hugepage: Cannot open /proc/meminfo: %s.\n", strerror(errno));
		return 0;
	}

	while (1) {
		char *line, buffer[1024];

		line = fgets(buffer, sizeof(buffer), in);
		if (!line)
			break;

		/* Search "HugePages_Free" entry */
		if (strstr(line, "HugePages_Free"))
			huge_page_free =  strtoul(line + 15, &line, 10);

		/* Search "Hugepagesize" entry */
		if (strstr(line, "Hugepagesize")) {
			/* Entry size is at kB, so convert it to actual size */
			huge_page_size = 1024 * strtoul(line + 13, &line, 10);
			break;
		}
	}

	fclose(in);

	if (huge_page_size == 0)
		return 0;

	/* Calculate required page count: round up(size / huge_page_size) */
	huge_page_count = roundup(size, huge_page_size) / huge_page_size;
	pr_debug("%s: proc/meminfo parsing: Free huge pages = %ld, Required = %d).\n",
			__func__, huge_page_free, huge_page_count);

	/* verify there are enough free huge pages allocated in kernel for required space */
	if (huge_page_free < huge_page_count) {
		pr_err("Hugepage: not enough free huge pages (Free = %ld, Required = %d)\n"
			"Please allocate more hugepages in /proc/sys/vm/nr_hugepages.\n"
			, huge_page_free, huge_page_count);
		return 0;
	}

	return huge_page_size;
}

/* Unlock, Detach, and Free allocated memory */
void hugepage_free_mem(struct sys_hugepage *hugepage)
{
	/* Unlock the allocated buffer from RAM */
	if (munlock((const void *)hugepage->shm_va[0], hugepage->size) != 0)
		pr_err("Hugepage: Unlocking of allocated memory failed");

	/* Detach buffer */
	if (shmdt((const void *)hugepage->shm_va[0]) < 0) {
		pr_err("Hugepage: Detach of allocated memory failed");
		return;
	}

	/* Free buffer */
	if (shmctl(hugepage->shm_id, IPC_RMID, NULL) < 0)
		pr_err("Hugepage: Free of allocated memory failed");
}

/* hugepage_init_mem:
 * - Allocate memory pool with huge pages
 * - Lock Pages in memory to avoid swap/migration
 * - Initialize Physical <-> Virtual conversion array
 */
void *hugepage_init_mem(u64 size, void **dma_virt_base)
{
	u64 huge_pages_count, i;
	intptr_t paddr;
	char *shmaddr;
	uintptr_t va;

	/* Hugepage struct - per each allocated memory section */
	hugepage_struct = (struct sys_hugepage *)calloc(1, sizeof(struct sys_hugepage));
	if (!hugepage_struct) {
		pr_err("no memory for sys_hugepage obj!\n");
		return NULL;
	}

	/* Fetch Kernel huge page size, and verify there are enough huge pages */
	hugepage_struct->huge_page_size = hugepage_verify_hugepage_pagesize(size);
	if (!hugepage_struct->huge_page_size) {
		pr_err("Hugepage: failed verifying kernel huge page size\n");
		goto free_memory;
	}

	/* Allocate memory */
	hugepage_struct->shm_id = shmget(IPC_PRIVATE, size, SHM_HUGETLB | IPC_CREAT | SHM_R | SHM_W);
	hugepage_struct->size = size;
	if (hugepage_struct->shm_id < 0) {
		pr_err("Hugepage: Failed allocating memory (shmget)");
		goto free_memory;
	}
	pr_debug("%s: Allocated memory - shmid: 0x%lx , size = 0x%lx\n", __func__, hugepage_struct->shm_id, size);

	/* Attach memory */
	shmaddr = shmat(hugepage_struct->shm_id, ADDR, 0);
	if (shmaddr == (char *)-1) {
		pr_err("Hugepage: Shared memory attach failure");
		if (shmctl(hugepage_struct->shm_id, IPC_RMID, NULL) < 0)
			pr_err("Hugepage: Free of allocated memory failed");
		goto free_memory;
	}
	pr_debug("%s: Attached memory - shmaddr: %p\n", __func__, shmaddr);

	/* lock allocated buffer into RAM, to avoid migration and swap on the buffer */
	if (mlock(shmaddr, size)) {
		pr_err("Hugepage: failed to lock allocated buffer in RAM\n");
		goto free_hugepage_memory;
	}

	/* Calculate required page count: round up(size / huge_page_size) */
	huge_pages_count = roundup(size, hugepage_struct->huge_page_size) / hugepage_struct->huge_page_size;

	/* Initialize Virtual <-> Physical address conversion information */
	for (i = 0; i < huge_pages_count; i++) {
		/* Virtual addresses are contiguous, so it's straight forward */
		hugepage_struct->shm_va[i] = (void *)(shmaddr + i * hugepage_struct->huge_page_size);
		paddr = hugepage_virt2phys_scan_proc_pagemap((void *)(hugepage_struct->shm_va[i]));
		if (paddr == -EINVAL) {
			pr_err("Hugepage: failed to scan physical address from pagemap\n");
			goto free_hugepage_memory;
		}
		hugepage_struct->shm_pa[i] = (phys_addr_t)paddr;
		pr_debug("page-%ld: VA = 0x%lx, PA = 0x%lx\n"
			, (u64)i, (uintptr_t)(hugepage_struct->shm_va[i]), (uintptr_t)(hugepage_struct->shm_pa[i]));
	}

	/* Validate PA to VA conversion */
	for (i = 0; i < huge_pages_count; i++) {
		/* Convert PA to VA */
		va = (uintptr_t)mv_sys_dma_mem_phys2virt((phys_addr_t)hugepage_struct->shm_pa[i]);

		/* Compare converted VA to expected VA */
		if (va != (uintptr_t)(hugepage_struct->shm_va[i]))
			pr_err("Bad VA<->PA conversion: converted VA = 0x%lx (Expected = 0x%lx)\n"
			       , va, (uintptr_t)(hugepage_struct->shm_va[i]));
	}

	/* Initialize required information for mem_mng APIs*/
	*dma_virt_base = shmaddr;	/* allocated buffer address is the VA base */
	return (void *) hugepage_struct;

free_hugepage_memory:
	hugepage_free_mem(hugepage_struct);

free_memory:
	free(hugepage_struct);
	return NULL;
}
#endif /* MVCONF_SYS_DMA_HUGE_PAGE */
