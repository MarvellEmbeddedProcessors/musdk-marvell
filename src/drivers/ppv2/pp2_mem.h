/******************************************************************************
 *	Copyright (C) 2016 Marvell International Ltd.
 *
 *  If you received this File from Marvell, you may opt to use, redistribute
 *  and/or modify this File under the following licensing terms.
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *	* Redistributions of source code must retain the above copyright
 *	  notice, this list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the distribution.
 *
 *	* Neither the name of Marvell nor the names of its contributors may be
 *	  used to endorse or promote products derived from this software
 *	  without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

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

#endif /* _PP2_MEM_H_ */
