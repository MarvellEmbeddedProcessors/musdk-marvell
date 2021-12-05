/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "std_internal.h"


#define DMA_MEM_SIZE 	(11*1024*1024)


struct mem {
	phys_addr_t	pa;
	void		*va;
};


static int single_test(int num_allocs, int alloc_size, int align)
{
	struct mem	*mems;
	int		i;
	unsigned long j;
	char *addr;

	mems = (struct mem *)malloc(num_allocs*sizeof(struct mem));
	if (!mems) {
		pr_err("No mem for dma-test mem obj!\n");
		return -ENOMEM;
	}

	for (i=0; i<num_allocs; i++) {
		mems[i].va = mv_sys_dma_mem_alloc(alloc_size, align);
		if (!mems[i].va) {
			pr_err("failed to allocate mem (%d,%d)!\n", i, alloc_size);
			break;
		}
		mems[i].pa = mv_sys_dma_mem_virt2phys(mems[i].va);
		pr_debug("va=%p,pa=%llx\n",mems[i].va,(long long unsigned int)mems[i].pa);
	}
	if (i!= num_allocs)
		return -EFAULT;

	/* Run some actual write & read tests to allocated memory */
	for (i=0; i<num_allocs; i++) {
		/* Write to all allocated range */
		for (j = 0; j < alloc_size; j++) {
			addr = mems[i].va;
			addr[j] = (char)(j);
		}
		if ((i % 500) == 0)
			printf(".");

		/* Verify all written data is as expected */
		for (j = 0; j < alloc_size; j++) {
			addr = mems[0].va;
			if (addr[j] != (char)j) {
				printf("\nError: Index %lu mismatched (Data written is not as expected)\n", j);
				exit(3);
			}
		}
	}

	for (i=0; i<num_allocs; i++) {
		if (!mems[i].va) continue;
		mv_sys_dma_mem_free(mems[i].va);
	}

	return 0;
}


int main (int argc, char *argv[])
{
	int		err;

	printf("Marvell Armada US (Build: %s %s)\n", __DATE__, __TIME__);

	if ((err = mv_sys_dma_mem_init(DMA_MEM_SIZE)) != 0)
		return err;

	printf("DMA memory test:\n");
	printf(".");
	if (!err)
		err = single_test(10000, 100, 4);
	printf(".");
	if (!err)
		err = single_test(100, 1000, 64);
	printf(".");
	if (!err)
		err = single_test(6, 1500000, 256);
	printf(".");
	if (!err)
		err = single_test(100, 1000, 64);
	printf(".");
	if (!err)
		err = single_test(10000, 100, 4);
	printf(".");
	if (!err)
		err = single_test(10, 0x100000, 4);
	printf(".");
	if (!err)
		err = single_test(3, 0x300000, 4);
	printf(".");

	mv_sys_dma_mem_destroy();

	if (err)
		printf("\nFAILED!\n");
	else
		printf("\npassed\n");

	return 0;
}
