/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

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
