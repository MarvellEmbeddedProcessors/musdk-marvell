/**************************************************************************//**
	Copyright (C) 2016 Marvell International Ltd.
*//***************************************************************************/

#include "std.h"
#include "sys_dma.h"


struct mem {
	phys_addr_t	pa;
	void		*va;
};


int main (int argc, char *argv[])
{
    struct mem	mems[100];
	int			i, err;

	printf("Marvell Armada US (Build: %s %s)\n", __DATE__, __TIME__);

	if ((err = mv_sys_dma_mem_init(16*1024*1024)) != 0)
		return err;

	for (i=0; i<10; i++) {
		mems[i].va = mv_sys_dma_mem_alloc(100, 4);
		if (!mems[i].va) {
			pr_err("failed to allocate mem (%d,%d)!\n", i, 100);
			break;
		}
		mems[i].pa = mv_sys_dma_mem_virt2phys(mems[i].va);
		printf("va=%p,pa=%llx\n",mems[i].va,(long long unsigned int)mems[i].pa);
	}

	for (i=0; i<100; i++) {
		if (!mems[i].va) continue;
		mv_sys_dma_mem_free(mems[i].va);
	}

	mv_sys_dma_mem_destroy();
	printf("bye ...\n");
	return 0;
}
