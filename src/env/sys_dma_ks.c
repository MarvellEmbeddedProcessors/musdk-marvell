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

#include <linux/dma-mapping.h>

#include "std_internal.h"
#include "env/mv_sys_dma.h"
#include "lib/mem_mng.h"


#define MUSDK_DEV_FILE "musdk-cma"

#ifndef NOTUSED
#define NOTUSED(_a) ((_a) = (_a))
#endif /* !NOTUSED */


struct mem_mng	*mm;
dma_addr_t	 __dma_phys_base;
void		*__dma_virt_base;
struct device	*dev;
size_t		 __dma_size;


void *mv_sys_dma_mem_alloc(size_t size, size_t align)
{
	u64 ret;

	if (!mm) {
		pr_err("%s: Memory Manager uninitialized.\n", __func__);
		return NULL;
	}

	ret = mem_mng_get(mm, size, align, "temp");

	if (ret == MEM_MNG_ILLEGAL_BASE) {

		pr_err("%s: Memory Manager: Illegal Base.", __func__);
		return NULL;
	}

	return (void *)ret;
}

void mv_sys_dma_mem_free(void *ptr)
{
	if (!mm)
		pr_err("%s: Memory Manager uninitialized.\n", __func__);
	else
		mem_mng_put(mm, (u64)ptr);
}

int mv_sys_dma_mem_init(struct device *adev, size_t size)
{
	int err;

	dev = adev;
	__dma_size = size;

	if (mm) {
		dev_err(dev, "Memory Manager already initialized.\n");
		return -EEXIST;
	}

	if (!dev->archdata.dma_coherent)
		dev_warn(dev, "dma_coherent not set. Using coherent mode anyway.\n");

	/* Setting up DMA subsystem */
	dev->dma_mask = devm_kzalloc(dev, sizeof(*dev->dma_mask), GFP_KERNEL);
	err = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (err) {
		dev_err(dev, "Cannot set dma_mask.\n");
		return err;
	}

	/* Allocating DMA memory */
	__dma_virt_base = dma_zalloc_coherent(dev, size, &__dma_phys_base, GFP_KERNEL | GFP_DMA);

	if (!__dma_virt_base) {
		dev_err(dev, "Not enough CMA memory to allocate %u bytes.\n", (unsigned int)size);
		return -ENOMEM;
	}

	/* Giving the memory to manager */
	err = mem_mng_init((u64)__dma_virt_base, size, &mm);

	if (err) {
		dma_free_coherent(dev, size, __dma_virt_base, __dma_phys_base);

		dev_err(dev, "Failed to initialize memory manager.\n");
		return err;
	}

	return 0;
}

void mv_sys_dma_mem_destroy(void)
{
	if (!mm)
		return;

	mem_mng_free(mm);
	dma_free_coherent(dev, __dma_size, __dma_virt_base, __dma_phys_base);
}

int mv_sys_dma_mem_get_info(struct mv_sys_dma_mem_info *mem_info)
{
	if (mem_info->name)
		strcpy(mem_info->name, MUSDK_DEV_FILE);

	mem_info->size = __dma_size;
	mem_info->paddr = __dma_phys_base;
	return 0;
}

void *mv_sys_dma_mem_phys2virt(phys_addr_t pa)
{
	return (void *)(uintptr_t)((pa - __dma_phys_base) + (phys_addr_t)(uintptr_t)__dma_virt_base);
}

phys_addr_t mv_sys_dma_mem_virt2phys(void *va)
{
	return ((phys_addr_t)(uintptr_t)va - (phys_addr_t)(uintptr_t)__dma_virt_base) + __dma_phys_base;
}

int mv_sys_dma_virt_is_valid(void *va)
{
	if ((va >= __dma_virt_base) && (va < (void *)((uintptr_t)__dma_virt_base + __dma_size)))
		return 1;

	return 0;
}

int mv_sys_dma_phys_is_valid(phys_addr_t pa)
{
	if ((pa >= __dma_phys_base) && (pa < __dma_phys_base + __dma_size))
		return 1;

	return 0;
}

int mv_sys_dma_mem_region_init(struct mv_sys_dma_mem_region_params *params, struct mv_sys_dma_mem_region **mem)
{
	NOTUSED(params); NOTUSED(mem);
	pr_err("%s not supported!\n", __func__);
	return -ENOTSUPP;
}

void mv_sys_dma_mem_region_destroy(struct mv_sys_dma_mem_region *mem)
{
	NOTUSED(mem);
	pr_err("%s not supported!\n", __func__);
}

void *mv_sys_dma_mem_region_alloc(struct mv_sys_dma_mem_region *mem, size_t size, size_t align)
{
	static bool warn_once;

	if (!mem) {
		if (!warn_once) {
			pr_warn("(%s) redirected to mv_sys_dma_mem_alloc()\n", __func__);
			warn_once = true;
		}
		return mv_sys_dma_mem_alloc(size, align);
	}

	pr_err("%s not supported!\n", __func__);
	return NULL;
}

void mv_sys_dma_mem_region_free(struct mv_sys_dma_mem_region *mem, void *ptr)
{
	if (!mem) {
		mv_sys_dma_mem_free(ptr);
		return;
	}

	NOTUSED(ptr);
	pr_err("%s not supported!\n", __func__);
}

struct mv_sys_dma_mem_region *mv_sys_dma_mem_region_get(u32 mem_id)
{
	NOTUSED(mem_id);
	pr_warn("%s not supported!\n", __func__);
	return NULL;
}

int mv_sys_dma_mem_region_exist(u32 mem_id)
{
	NOTUSED(mem_id);
	pr_warn("%s not supported!\n", __func__);
	return 0;
}
