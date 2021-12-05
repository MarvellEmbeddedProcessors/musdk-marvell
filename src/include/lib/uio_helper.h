/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __UIO_HELPER_H__
#define __UIO_HELPER_H__

#include <sys/mman.h>
#include "mv_std.h"

/* convensions */

#define UIO_MAX_NAME_SIZE       64
#define UIO_MAX_NUM             255

#define UIO_INVALID_SIZE        0
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
	unsigned long size;
	char name[UIO_MAX_NAME_SIZE];
	int mmap_result;
	void *internal_addr;
};

struct uio_dev_attr_t {
	char name[UIO_MAX_NAME_SIZE];
	char value[UIO_MAX_NAME_SIZE];
	struct uio_dev_attr_t *next;
};

struct uio_info_t {
	int uio_num;
	struct uio_map_t maps[MAX_UIO_MAPS];
	unsigned long event_count;
	char name[UIO_MAX_NAME_SIZE];
	char version[UIO_MAX_NAME_SIZE];
	struct uio_dev_attr_t *dev_attrs;
	struct uio_info_t *next;  /* for linked list */
};

struct uio_mem_t {
	int map_num;
	int fd;
	struct uio_info_t *info;
	struct uio_mem_t *next;  /* for linked list */
};

/* function prototypes */
/*
 * Disabled uio_lib_* function prototypes because crash ODP build with:
 * error: 'uio_lib_*' declared 'static' but never defined [-Werror=unused-function]
 *
 * TODO: Remove the prototypes or implement them
 */
#if 0
static inline char *uio_lib_name(void);
static inline char *uio_lib_version(void);
static inline int uio_lib_ifcurrent(void);
static inline int uio_lib_ifrevision(void);
static inline int uio_lib_ifage(void);
#endif

int uio_get_mem_size(struct uio_info_t *info, int map_num);
int uio_get_mem_addr(struct uio_info_t *info, int map_num);
int uio_get_mem_name(struct uio_info_t *info, int map_num);
int uio_get_event_count(struct uio_info_t *info);
int uio_get_name(struct uio_info_t *info);
int uio_get_version(struct uio_info_t *info);
int uio_get_all_info(struct uio_info_t *info);

void *uio_single_mmap(struct uio_info_t *info, int map_num, int fd);

static inline void uio_mmap(struct uio_info_t *info, int fd);
static inline void uio_single_munmap(struct uio_info_t *info, int map_num);
static inline void uio_munmap(struct uio_info_t *info);

static inline void uio_mmap(struct uio_info_t *info, int fd)
{
	int map_num;

	if (!fd)
		return;
	for (map_num = 0; map_num < MAX_UIO_MAPS; map_num++)
		uio_single_mmap(info, map_num, fd);
}

static inline void uio_single_munmap(struct uio_info_t *info, int map_num)
{
	munmap(info->maps[map_num].internal_addr, info->maps[map_num].size);
	info->maps[map_num].mmap_result = UIO_MMAP_NOT_DONE;
}

static inline void uio_munmap(struct uio_info_t *info)
{
	int i;

	for (i = 0; i < MAX_UIO_MAPS; i++)
		uio_single_munmap(info, i);
}

void uio_free_dev_attrs(struct uio_info_t *info);
void uio_free_info(struct uio_info_t *info);
void uio_free_mem_info(struct uio_mem_t *info);

struct uio_info_t *uio_find_devices(int filter_num);
struct uio_info_t *uio_find_devices_byname(const char *filter_name);
struct uio_mem_t *uio_find_mem_byname(struct uio_info_t *info,
				      const char *filter);

#endif /* __UIO_HELPER_H__ */
