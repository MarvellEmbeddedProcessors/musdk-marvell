/*
 * uio_get_mem_name.c
 * UIO helper function: uio_get_mem_name
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

#include "lib/uio_helper.h"

extern int __uio_line_from_file(char *filename, char *linebuf);

/**
 * uio_get_mem_name
 *
 * Search for memory map name exported by UIO drv.
 * Same map name in different maps is not accepted.
 *
 * @param info      Linked list uio_info_t
 * @param map_num    Memory map number
 *
 * @return Negative value if map name file not found or corrupted
 */
int uio_get_mem_name(struct uio_info_t* info, int map_num)
{
	char filename[64];
	if (map_num >= MAX_UIO_MAPS)
		return -1;
	snprintf(filename, sizeof(filename),
		"/sys/class/uio/uio%d/maps/map%d/name",
		info->uio_num, map_num);

	return __uio_line_from_file(filename, info->maps[map_num].name);
}
