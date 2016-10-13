/*
   uio_get_event_count.c
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

int uio_get_event_count(struct uio_info_t* info)
{
	int ret;
	char filename[64];
	info->event_count = 0;
	snprintf(filename, sizeof(filename),
		 "/sys/class/uio/uio%d/event", info->uio_num);
	FILE* file = fopen(filename,"r");
	if (!file) return -1;
	ret = fscanf(file,"%d",(int *)(&info->event_count));
	fclose(file);
	if (ret<0) return -2;
	return 0;
}
