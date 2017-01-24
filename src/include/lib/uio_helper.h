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

#ifndef __UIO_HELPER_H__
#define __UIO_HELPER_H__

#include <sys/mman.h>
#include "mv_std.h"

/* convensions */

#define UIO_MAX_NAME_SIZE       64
#define UIO_MAX_NUM             255

#define UIO_INVALID_SIZE        -1
#define UIO_INVALID_ADDR        (~0)

#define UIO_MMAP_NOT_DONE       0
#define UIO_MMAP_OK             1
#define UIO_MMAP_FAILED         2

#define UIO_MEM_NOT_INSTALLED   0x5
#define UIO_INVALID_FD          -1

/* This should be identical to the define in include/linux/uio_driver.h !!! */
#define MAX_UIO_MAPS    5

struct uio_map_t {
	unsigned long addr;
	int size;
	char name[ UIO_MAX_NAME_SIZE ];
	int mmap_result;
	void* internal_addr;
};

struct uio_dev_attr_t {
	char name[ UIO_MAX_NAME_SIZE ];
	char value[ UIO_MAX_NAME_SIZE ];
	struct uio_dev_attr_t* next;
};

struct uio_info_t {
	int uio_num;
	struct uio_map_t maps[ MAX_UIO_MAPS ];
	unsigned long event_count;
	char name[ UIO_MAX_NAME_SIZE ];
	char version[ UIO_MAX_NAME_SIZE ];
	struct uio_dev_attr_t* dev_attrs;
	struct uio_info_t* next;  /* for linked list */
};

struct uio_mem_t {
	int map_num;
	int fd;
	struct uio_info_t* info;
	struct uio_mem_t* next;  /* for linked list */
};

/* function prototypes */
/*
 * Disabled uio_lib_* function prototypes because crash ODP build with:
 * error: 'uio_lib_*' declared 'static' but never defined [-Werror=unused-function]
 *
 * TODO: Remove the prototypes or implement them
 */
#if 0
static inline char* uio_lib_name(void);
static inline char* uio_lib_version(void);
static inline int uio_lib_ifcurrent(void);
static inline int uio_lib_ifrevision(void);
static inline int uio_lib_ifage(void);
#endif

int uio_get_mem_size(struct uio_info_t* info, int map_num);
int uio_get_mem_addr(struct uio_info_t* info, int map_num);
int uio_get_mem_name(struct uio_info_t* info, int map_num);
int uio_get_event_count(struct uio_info_t* info);
int uio_get_name(struct uio_info_t* info);
int uio_get_version(struct uio_info_t* info);
int uio_get_all_info(struct uio_info_t* info);
int uio_get_device_attributes(struct uio_info_t* info);

void* uio_single_mmap(struct uio_info_t* info, int map_num, int fd);

static inline void uio_mmap(struct uio_info_t* info, int fd);
static inline void uio_single_munmap(struct uio_info_t* info, int map_num);
static inline void uio_munmap(struct uio_info_t* info);

static inline void uio_mmap(struct uio_info_t* info, int fd)
{
	int map_num;
	if (!fd)
		return;
	for (map_num= 0; map_num < MAX_UIO_MAPS; map_num++)
		uio_single_mmap(info, map_num, fd);
}

static inline void uio_single_munmap(struct uio_info_t* info, int map_num)
{
	munmap(info->maps[map_num].internal_addr, info->maps[map_num].size);
	info->maps[map_num].mmap_result = UIO_MMAP_NOT_DONE;
}

static inline void uio_munmap(struct uio_info_t* info)
{
	int i;
	for (i = 0; i < MAX_UIO_MAPS; i++)
		uio_single_munmap(info, i);
}

void uio_free_dev_attrs(struct uio_info_t* info);
void uio_free_info(struct uio_info_t* info);
void uio_free_mem_info(struct uio_mem_t* info);

struct uio_info_t* uio_find_devices(int filter_num);
struct uio_info_t* uio_find_devices_byname(const char *filter_name);
struct uio_mem_t* uio_find_mem_byname(struct uio_info_t* info,
		const char *filter);

#endif /* __UIO_HELPER_H__ */
