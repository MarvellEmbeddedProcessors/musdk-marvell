/*
   uio_find_devices_byname.c
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
#include <unistd.h>
#include <fcntl.h>
#include <dirent.h>
#include <string.h>

#include <sys/types.h>
#include <sys/mman.h>
#include <sys/stat.h>

#include "lib/uio_helper.h"

extern int __uio_line_from_file(char *filename, char *linebuf);
extern int __uio_num_from_filename(char* name);

static struct uio_info_t *__uio_info_byname(char* name, const char *filter_name)
{
	struct uio_info_t *info;
	char linebuf[UIO_MAX_NAME_SIZE];
	char filename[64];

	snprintf(filename, sizeof(filename), "/sys/class/uio/%s/name", name);
	if (__uio_line_from_file(filename, linebuf))
		return NULL;

	if (strncmp(linebuf, filter_name, strlen(filter_name)))
		return NULL;

	info = kmalloc(sizeof(struct uio_info_t), GFP_KERNEL);
	if (!info)
		return NULL;
	memset(info, 0, sizeof(struct uio_info_t));
	info->uio_num = __uio_num_from_filename(name);

	return info;
}

struct uio_info_t *uio_find_devices_byname(const char *filter_name)
{
	struct dirent **namelist;
	struct uio_info_t *infolist = NULL, *infp, *last;
	int n;

	n = scandir("/sys/class/uio", &namelist, 0, alphasort);
	if (n <= 0) {
		pr_err("scandir for /sys/class/uio failed. errno = %d (%s)\n",
			errno, strerror(errno));
		return NULL;
	}
	while(n--) {
		infp = __uio_info_byname(namelist[n]->d_name, filter_name);
		kfree(namelist[n]);
		if (!infp)
			continue;

		if (!infolist)
			infolist = infp;
		else
			last->next = infp;
		last = infp;
	}
	kfree(namelist);

	return infolist;
}
