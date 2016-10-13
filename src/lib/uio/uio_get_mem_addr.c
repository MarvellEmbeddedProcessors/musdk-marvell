/*
   uio_get_mem_addr.c
   UIO helper function: ... ... ...

   Copyright (C) 2009, Hans J. Koch <hjk@linutronix.de>
   Copyright (C) 2009, Stephan Linz <linz@li-pro.net>

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License version 2 as
   published by the Free Software Foundation.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software Foundation,
   Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
*/
#include <stdio.h>
#include <stdlib.h>

#include "lib/uio_helper.h"

int uio_get_mem_addr(struct uio_info_t* info, int map_num)
{
	int ret;
	char filename[64];
	if (map_num >= MAX_UIO_MAPS) return -1;
	info->maps[map_num].addr = UIO_INVALID_ADDR;
	snprintf(filename, sizeof(filename),
		 "/sys/class/uio/uio%d/maps/map%d/addr",
		info->uio_num, map_num);
	FILE* file = fopen(filename,"r");
	if (!file) return -1;
	ret = fscanf(file,"0x%lx",&info->maps[map_num].addr);
	fclose(file);
	if (ret<0) return -2;
	return 0;
}
