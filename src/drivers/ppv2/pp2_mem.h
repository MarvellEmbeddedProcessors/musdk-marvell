/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

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
