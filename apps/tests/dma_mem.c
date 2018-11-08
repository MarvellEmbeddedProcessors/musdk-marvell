/*******************************************************************************
 * Copyright (C) Marvell International Ltd. and its affiliates
 *
 * This software file (the "File") is owned and distributed by Marvell
 * International Ltd. and/or its affiliates ("Marvell") under the following
 * alternative licensing terms.  Once you have made an election to distribute the
 * File under one of the following license alternatives, please (i) delete this
 * introductory statement regarding license alternatives, (ii) delete the three
 * license alternatives that you have not elected to use and (iii) preserve the
 * Marvell copyright notice above.
 *
 ********************************************************************************
 * Marvell Commercial License Option
 *
 * If you received this File from Marvell and you have entered into a commercial
 * license agreement (a "Commercial License") with Marvell, the File is licensed
 * to you under the terms of the applicable Commercial License.
 *
 ********************************************************************************
 * Marvell GPL License Option
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File in accordance with the terms and conditions of the General
 * Public License Version 2, June 1991 (the "GPL License"), a copy of which is
 * available along with the File in the license.txt file or by writing to the Free
 * Software Foundation, Inc., or on the worldwide web at http://www.gnu.org/licenses/gpl.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY
 * DISCLAIMED.  The GPL License provides additional details about this warranty
 * disclaimer.
 *
 ********************************************************************************
 * Marvell GNU General Public License FreeRTOS Exception
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File in accordance with the terms and conditions of the Lesser
 * General Public License Version 2.1 plus the following FreeRTOS exception.
 * An independent module is a module which is not derived from or based on
 * FreeRTOS.
 * Clause 1:
 * Linking FreeRTOS statically or dynamically with other modules is making a
 * combined work based on FreeRTOS. Thus, the terms and conditions of the GNU
 * General Public License cover the whole combination.
 * As a special exception, the copyright holder of FreeRTOS gives you permission
 * to link FreeRTOS with independent modules that communicate with FreeRTOS solely
 * through the FreeRTOS API interface, regardless of the license terms of these
 * independent modules, and to copy and distribute the resulting combined work
 * under terms of your choice, provided that:
 * 1. Every copy of the combined work is accompanied by a written statement that
 * details to the recipient the version of FreeRTOS used and an offer by yourself
 * to provide the FreeRTOS source code (including any modifications you may have
 * made) should the recipient request it.
 * 2. The combined work is not itself an RTOS, scheduler, kernel or related
 * product.
 * 3. The independent modules add significant and primary functionality to
 * FreeRTOS and do not merely extend the existing functionality already present in
 * FreeRTOS.
 * Clause 2:
 * FreeRTOS may not be used for any competitive or comparative purpose, including
 * the publication of any form of run time or compile time metric, without the
 * express permission of Real Time Engineers Ltd. (this is the norm within the
 * industry and is intended to ensure information accuracy).
 *
 ********************************************************************************
 * Marvell BSD License Option
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File under the following licensing terms.
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *	* Redistributions of source code must retain the above copyright notice,
 *	  this list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the distribution.
 *
 *	* Neither the name of Marvell nor the names of its contributors may be
 *	  used to endorse or promote products derived from this software without
 *	  specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
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
