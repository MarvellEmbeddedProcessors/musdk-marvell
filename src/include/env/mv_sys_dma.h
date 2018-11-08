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

#ifndef __MV_SYS_DMA_H__
#define __MV_SYS_DMA_H__

#include "mv_types.h"

/** @addtogroup grp_sys_dma DMA Memory Manager
 *
 *  DMA Memory Manager API documentation
 *
 *  @{
 */

#define MV_SYS_DMA_MAX_NUM_MEM_ID		4
#define MV_SYS_DMA_MAX_MEM_ID			(MV_SYS_DMA_MAX_NUM_MEM_ID - 1)


struct mv_sys_dma_mem_region {
	void		*dma_virt_base;
	phys_addr_t	dma_phys_base;
	u64		size;
	int		manage;		/* Is this region managed by alloc and free functions */
	void		*priv;
	u32		mem_id;		/* Which DRAM will the region be allocated on. */
};

struct mv_sys_dma_mem_region_params {
	u64		size;
	int		manage;		/* Is this region managed by alloc and free functions */
	u32		mem_id;		/* Which DRAM will the region be allocated on. */
					/* TODO: change after understanding DRAM */
};

struct mv_sys_dma_mem_info {
	char		*name;
	size_t		 size;
	phys_addr_t	 paddr;
};

/**
 * Initialize the system DMA memory manager
 *
 * MUSDK system provides a mechanism for DMA-able memory allocation.
 * Assume this memory is a single, contiguously allocated region.
 * In some implementations (e.g. when using hugepages), it is only contiguous
 * in the virtual address space, while in other implementations (e.g. CMA),
 * it is allocated as a contiguous memory both in virtual as well as in
 * physical address space. It is created in order to support an efficient
 * conversion between user-space virtual address map(s) and bus addresses
 * required by hardware for DMA.
 *
 * @param[in]	size	size of the request DMA memory.
 *
 * @retval	0 on success.
 * @retval	error code on failure.
 */
#ifdef __KERNEL__
int mv_sys_dma_mem_init(struct device *dev, size_t size);
#else
int mv_sys_dma_mem_init(size_t size);
#endif

/**
 * Destroy the system DMA memory manager.
 */
void mv_sys_dma_mem_destroy(void);

/**
 * Allocate DMA memory slice from the system DMA memory.
 *
 * @param[in]	size	size of the request DMA memory.
 * @param[in]	align	Alignment of the request DMA memory.
 *
 * @retval	A pointer to a DMA memory on success
 * @retval	<0 on failure
 */
void *mv_sys_dma_mem_alloc(size_t size, size_t align);

/**
 * Free an allocated DMA memory slice of the system DMA memory.
 *
 * @param[in]	ptr		A pointer to a DMA memory.
 */
void mv_sys_dma_mem_free(void *ptr);

/**
 * Retrieve DMA associated info
 *
 * @param[out]	mem_info	DMA associated info
 */
int mv_sys_dma_mem_get_info(struct mv_sys_dma_mem_info *mem_info);

/**
 * Physical to Virtual address translation of an allocated DMA memory.
 *
 * @param[in]	pa	Physical Address to be translated.
 *
 * @retval	A pointer to a virtual DMA memory on success
 * @retval	<0 on failure
 */
void *mv_sys_dma_mem_phys2virt(phys_addr_t pa);


/**
 * Virtual to Physical address translation of an allocated DMA memory.
 *
 * @param[in]	va	A pointer to a virtual DMA memory to be translated.
 *
 * @retval	Physical-address on success
 * @retval	<0 on failure
 */
phys_addr_t mv_sys_dma_mem_virt2phys(void *va);

/**
 * Check validity of virtual address allocated from DMA memory.
 *
 * @param[in]	va	Virtual address to be checked.
 *
 * @retval	1 - virtual address is valid
 * @retval	0 - virtual address is invalid
 */
int mv_sys_dma_virt_is_valid(void *va);

/**
 * Check validity of physical address allocated from DMA memory.
 *
 * @param[in]	pa	Physical Address to be checked.
 *
 * @retval	1 - physical address is valid
 * @retval	0 - physical address is invalid
 */
int mv_sys_dma_phys_is_valid(phys_addr_t pa);


int mv_sys_dma_mem_region_init(struct mv_sys_dma_mem_region_params *params, struct mv_sys_dma_mem_region **mem);
void mv_sys_dma_mem_region_destroy(struct mv_sys_dma_mem_region *mem);
void *mv_sys_dma_mem_region_alloc(struct mv_sys_dma_mem_region *mem, size_t size, size_t align);
void mv_sys_dma_mem_region_free(struct mv_sys_dma_mem_region *mem, void *ptr);

static inline void *mv_sys_dma_mem_region_get_vaddr_base(struct mv_sys_dma_mem_region *mem)
{
	return mem->dma_virt_base;
}

static inline phys_addr_t mv_sys_dma_mem_region_get_paddr_base(struct mv_sys_dma_mem_region *mem)
{
	return mem->dma_phys_base;
}

#ifdef MVCONF_SYS_DMA_HUGE_PAGE
void *mv_sys_dma_mem_region_phys2virt(struct mv_sys_dma_mem_region *mem, phys_addr_t pa);
#else
static inline void *mv_sys_dma_mem_region_phys2virt(struct mv_sys_dma_mem_region *mem, phys_addr_t pa)
{
	if (!mem)
		return mv_sys_dma_mem_phys2virt(pa);
	return (void *)(uintptr_t)((pa - mem->dma_phys_base) + (phys_addr_t)(uintptr_t)mem->dma_virt_base);
}
#endif /* MVCONF_SYS_DMA_HUGE_PAGE */

#ifdef MVCONF_SYS_DMA_HUGE_PAGE
phys_addr_t mv_sys_dma_mem_region_virt2phys(struct mv_sys_dma_mem_region *mem, void *va);
#else
static inline phys_addr_t mv_sys_dma_mem_region_virt2phys(struct mv_sys_dma_mem_region *mem, void *va)
{
	if (!mem)
		return mv_sys_dma_mem_virt2phys(va);
	return ((phys_addr_t)(uintptr_t)va - (phys_addr_t)(uintptr_t)mem->dma_virt_base) + mem->dma_phys_base;
}
#endif /* MVCONF_SYS_DMA_HUGE_PAGE */

struct mv_sys_dma_mem_region *mv_sys_dma_mem_region_get(u32 mem_id);

int mv_sys_dma_mem_region_exist(u32 mem_id);
/** @} */ /* end of grp_sys_dma */

#endif /* __MV_SYS_DMA_H__ */
