/*
 * uio_find_mem_byname.c
 * UIO helper function: uio_find_mem_byname
 *
 * Copyright (C) 2016, Lucian Zala <lucian.zala@enea.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>

#include <sys/types.h>
#include <sys/mman.h>
#include <sys/stat.h>

#include "lib/uio_helper.h"

/**
 * uio_find_mem_byname
 *
 * Search for memory map name exported by UIO drv.
 * Same map name in different maps is not accepted.
 *
 * @param info      Linked list uio_info_t
 * @param filter    Memory map filter name
 *
 * @return uio_info_fd_t structure having map_num, uio_num, invalid fd
 */
struct uio_mem_t *uio_find_mem_byname(struct uio_info_t *info,
		const char *filter)
{
	struct uio_info_t *infp = info;
	struct uio_mem_t *uiofdp = NULL;

	if (!infp || !filter)
		return NULL;

	while(infp) {
		int i;
		for (i = 0; i < MAX_UIO_MAPS; i++) {
			if(strncmp(infp->maps[i].name, filter,
				UIO_MAX_NAME_SIZE)) {
				continue;
			} else {
				uiofdp = calloc(1, sizeof(struct uio_info_t));
				uiofdp->map_num = i;
				uiofdp->fd = UIO_INVALID_FD;
				uiofdp->info = infp;
				return uiofdp;
			}
		}
		infp = infp->next;
	}

	return uiofdp;
}
