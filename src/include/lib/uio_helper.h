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
