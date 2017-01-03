/*******************************************************************************
	Copyright (C) 2016 Marvell International Ltd.

	If you received this File from Marvell, you may opt to use, redistribute
	and/or modify this File under the following licensing terms.
	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are met:

		* Redistributions of source code must retain the above copyright notice,
		  this list of conditions and the following disclaimer.

		* Redistributions in binary form must reproduce the above copyright
		  notice, this list of conditions and the following disclaimer in the
		  documentation and/or other materials provided with the distribution.

		* Neither the name of Marvell nor the names of its contributors may be
		  used to endorse or promote products derived from this software without
		  specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
	AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
	IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
	ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
	LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
	CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
	SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
	INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
	CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
	ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

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

	mv_sys_dma_mem_destroy();

	if (err)
		printf("\nFAILED!\n");
	else
		printf("\npassed\n");

	return 0;
}
