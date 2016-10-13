/*
   uio_get_all_info.c
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

int uio_get_all_info(struct uio_info_t* info)
{
	int i;
	if (!info)
		return -1;
	if ((info->uio_num < 0)||(info->uio_num > UIO_MAX_NUM))
		return -1;
	for (i = 0; i < MAX_UIO_MAPS; i++) {
		uio_get_mem_size(info, i);
		uio_get_mem_addr(info, i);
		uio_get_mem_name(info, i);
	}
	uio_get_event_count(info);
	uio_get_name(info);
	uio_get_version(info);
	return 0;
}
