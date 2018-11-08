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

#ifndef __SYS_IOMEM_H__
#define __SYS_IOMEM_H__

#include "mv_types.h"

/**
 * struct sys_iomem
 * Handle for IO-memory maps context structure. The handle should be
 * initialized by callingsys_ioinit() and will be passed to all the
 * other IO-maps routines.
 */
struct sys_iomem;

/**
 * Enumeration that represent all the supported types of iomem.
 */
enum sys_iomem_type {
	SYS_IOMEM_T_MMAP = 0, /**< type mmap */
	SYS_IOMEM_T_UIO,      /**< type UIO */
	SYS_IOMEM_T_VFIO,     /**< type VFIO; TODO: not supported yet! */
	SYS_IOMEM_T_SHMEM     /**< type Shared Memory */
};

struct sys_iomem_params {
	enum sys_iomem_type	 type; /**< select to a method for the mapping */
	/** A device name as it is exposed by the device driver (in case of
	 * UIO/VFIO) or Device-Tree (in case of mmap).
	 * For 'shmem', devname is the shmem file name of the device as created by the
	 * master process.
	 * UIO examples: for PPv2, use 'pp'; for SAM, use 'eip197'.
	 * mmap examples: for PPv2, use 'marvell,mv-pp22', for DMA-XOR, use 'marvell,mv-xor-v2'.
	 */
	const char		*devname;
	int			 index; /**< the device index, or -1 for single device (like BM) */
	/** The size of memory to be allocated. Relevant only for shmem mapping type.
	 * If absent, default size value is set to 1 page size (_SC_PAGE_SIZE)
	 */
	size_t			 size;
};

struct sys_iomem_info {
	union {
		struct {
			void		*va;
			size_t		 size;
			phys_addr_t	 paddr;
		} shmem;
	} u;
};

/**
 * Check if an IO-memory device exists
 *
 * @param	params	A pointer to a io-mem parameter structure.
 *
 * @retval 1 exists
 * @retval 0 no found
 */
int sys_iomem_exists(struct sys_iomem_params *params);

/**
 * Check if an IO-memory device exists and return its handle
 *
 * @param	params		A pointer to a io-mem parameter structure.
 * @param[out]	iomem_info	IO-mem associated info
 *
 * @retval 0 Success
 * @retval < 0 Failure
 */
int sys_iomem_get_info(struct sys_iomem_params *params, struct sys_iomem_info *iomem_info);

/**
 * Create and Initialize IO-memory device and maps structures
 *
 * This routine will allocate and initialize the io-mem structure.
 * The routine will filter the devices based on driver name and index.
 *
 * @param	params	A pointer to a io-mem parameter structure.
 * @param[out]	iomem	A pointer to an initialized io-mem structure
 *
 * @retval 0 Success
 * @retval < 0 Failure
 */
int sys_iomem_init(struct sys_iomem_params *params, struct sys_iomem **iomem);

/**
 * Destroy IO-memory structure
 *
 * Release the memory and all other resources for the io-mem structure.
 *
 * @param	iomem	A pointer to an initialized io-mem structure
 */
void sys_iomem_deinit(struct sys_iomem *iomem);

/**
 * I/O mapping based on map name exported by I/O driver
 *
 * The routine will search through available maps and filter by map name.
 * This method will take care of opening the device if not
 * already opened by another call and will mmap the region
 * requested. This function exports physical the address of the
 * mapped region if pa != NULL. This should never be accessed via r/w calls.
 * The possible use case of the physical address is for debug prints
 * or passing it by value to registers (i.e. contiguous dma mapping).
 * Each successful sys_iomem_map call will add the map to a list.
 *
 * @param	iomem	A pointer to an initialized io-mem structure
 * @param	name	A name of the memory map. In case of mmap type, the name
 *			should be actually the iomap decimal index (given as string).
 * @param[out]  pa      physical address of the mapped region. In case of mmap type,
 *			it is possible to pass here phys-address and name as NULL
 *			directly to get iomap. In that case, the routine maps one page (i.e. 4KB).
 * @param[out]	va      Virtual address of the mapped region
 *
 * @retval 0 Success
 * @retval < 0 Failure
 */
int sys_iomem_map(struct sys_iomem *iomem, const char *name, phys_addr_t *pa, void **va);

/**
 * I/O unmapping based on map name exported by I/O driver
 *
 * The routine will search through the map list and filter by map name.
 * This method will take care of closing the device if no map is registered
 * as mapped. Each successful sys_iomem_unmap call will remove the map from
 * the list.
 *
 * @param       iomem   struct sys_iomem handle
 * @param       name    name of the memory map
 *
 * @retval 0 Success
 * @retval < 0 Failure. Memory map not found.
 */
int sys_iomem_unmap(struct sys_iomem *iomem, const char *name);

/* TODO:
 * void * sys_iomem_phys2virt (phys_addr_t addr);
 * phys_addr_t sys_iomem_virt2phys (void *addr);
*/

#endif /* __SYS_IOMEM_H__ */
