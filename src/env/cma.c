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

