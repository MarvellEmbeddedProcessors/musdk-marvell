/**************************************************************************//**
	Copyright (C) 2016 Marvell International Ltd.
*//***************************************************************************/

#ifndef __SYS_IO_H__
#define __SYS_IO_H__

#include "std.h"

int mv_sys_register_iomap(phys_addr_t pa, u64 size, int mmap, u64 off, void **va);
int  mv_sys_unregister_iomap (void *va);

void * mv_sys_phys2virt (phys_addr_t addr);
phys_addr_t mv_sys_virt2phys (void *addr);

#endif /* __SYS_IO_H__ */
