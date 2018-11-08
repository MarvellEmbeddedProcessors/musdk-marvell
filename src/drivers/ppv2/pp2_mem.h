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

/**
 * @file pp2_mem.h
 *
 * Packet Processor I/O Memory mapping and Contiguous Allocator
 *
 */

#ifndef _PP2_MEM_H_
#define _PP2_MEM_H_

#include "std_internal.h"
#include "pp2_types.h"
#include "pp2_hw_type.h"

/**
 * User I/O map API
 *
 */

 /**
 * pp2_maps_handle_t
 * Handle for pp maps context structure
 * The handle should be initialized by calling
 * pp2_sys_ioinit(&pp2_maps_hdl, pp2_name) and will be passed
 * to all the other pp maps methods.
 *
 */
typedef uintptr_t pp2_maps_handle_t;

int pp2_sys_io_exists(const char *name);

/**
 * Initialize user I/O devices and maps structures
 *
 * In order to initialize Marvell packet processor structures
 * Marvell UIO driver should be inserted and probed.
 *
 * pp2_sys_ioinit will allocate the memory for the pp structure
 * pp2_sys_ioinit will filter the devices based on driver name
 *
 * @param    pp2_maps_hdl    pointer to pp2_maps_handle_t
 * @param    name           name of the packet processor
 *
 * @retval 0 Success
 * @retval < 0 Failure. Info could not be retrieved.
 */
int pp2_sys_ioinit(pp2_maps_handle_t *pp2_maps_hdl, const char *name);

/**
 * I/O mapping based on map name exported by I/O driver
 *
 * pp2_sys_iomap will search through available maps and filter by map name.
 * This method will take care of opening the device if not
 * already opened by another call and will mmap the region
 * requested. This function exports physical the address of the
 * mapped region if pa != NULL. This should never be accessed via r/w calls.
 * The possible use case of the physical address is for debug prints
 * or passing it by value to registers (i.e. contiguous dma mapping).
 * Each successful pp2_sys_iomap call will add the map to a list.
 *
 * @param    pp2_maps_handle_t    pp2_maps_handle_t handle
 * @param    pa                  physical address of the mapped region
 * @param    name                name of the memory map
 *
 * @retval Virtual address of the mapped region
 *
 */
uintptr_t pp2_sys_iomap(pp2_maps_handle_t pp2_maps_hdl,
			u32 *pa, const char *name);

/**
 * I/O unmapping based on map name exported by I/O driver
 *
 * pp2_sys_iounmap will search through the map list and filter by map name.
 *
 * This method will take care of closing the device if no map
 * is registered as mapped.
 * Each successful pp2_sys_iounmap call will remove the map from the list.
 *
 * @param    pp2_maps_handle_t    pp2_maps_handle_t handle
 * @param    name                name of the memory map
 *
 * @retval 0 Success
 * @retval < 0 Failure. Memory map not found.
 *
 */
int pp2_sys_iounmap(pp2_maps_handle_t pp2_maps_hdl, const char *name);

/**
 * Destroy user I/O pp structure
 *
 * @param    pp2_maps_handle_t    pp2_maps_handle_t handle
 *
 * pp2_sys_iodestroy will release the memory for the pp structure
 *
 */
void pp2_sys_iodestroy(pp2_maps_handle_t pp2_maps_hdl);

/** Slot specific r/w access routines */

/**
 * Packet Processor register write function
 * Offers lock-less access to shares resources based on PP CPU memory slots
 *
 * @param cpu_slot PP CPU slot
 * @offset register offset
 * @data data to feed register
 *
 */
static inline void pp2_reg_write(uintptr_t cpu_slot, uint32_t offset,
				 uint32_t data)
{
	uintptr_t addr = cpu_slot + offset;

	writel(data, (void *)addr);
}

/**
 * Packet Processor register relaxed write function
 * Offers lock-less access to shares resources based on PP CPU memory slots, without memory barriers.
 *
 * @param cpu_slot PP CPU slot
 * @offset register offset
 * @data data to feed register
 *
 */
static inline void pp2_relaxed_reg_write(uintptr_t cpu_slot, uint32_t offset,
					 uint32_t data)
{
	uintptr_t addr = cpu_slot + offset;

	writel_relaxed(data, (void *)addr);
}

/**
 * Packet Processor register read function
 * Offers lock-less access to shares resources based on PP CPU memory slots
 *
 * @param cpu_slot PP CPU slot
 * @offset register offset
 *
 * @retval content of register
 *
 */
static inline uint32_t pp2_reg_read(uintptr_t cpu_slot, uint32_t offset)
{
	uintptr_t addr = cpu_slot + offset;

	return readl((void *)addr);
}

/**
 * Packet Processor register relaxed read function
 * Offers lock-less access to shares resources based on PP CPU memory slots, without memory mem_barriers.
 *
 * @param cpu_slot PP CPU slot
 * @offset register offset
 *
 * @retval content of register
 *
 */
static inline uint32_t pp2_relaxed_reg_read(uintptr_t cpu_slot, uint32_t offset)
{
	uintptr_t addr = cpu_slot + offset;

	return readl_relaxed((void *)addr);
}

#define pp2_relaxed_read pp2_reg_read

static inline void cm3_write(uintptr_t base, u32 offset, u32 data)
{
	uintptr_t addr = base + offset;

	writel(data, (void *)addr);
}

static inline u32 cm3_read(uintptr_t base, u32 offset)
{

	uintptr_t addr = base + offset;

	return readl((void *)addr);
}


#endif /* _PP2_MEM_H_ */
