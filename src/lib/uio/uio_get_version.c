/*
   uio_get_version.c
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

extern int __uio_line_from_file(char *filename, char *linebuf);

int uio_get_version(struct uio_info_t* info)
{
	char filename[64];
	snprintf(filename, sizeof(filename),
		 "/sys/class/uio/uio%d/version", info->uio_num);

	return __uio_line_from_file(filename, info->version);
}
