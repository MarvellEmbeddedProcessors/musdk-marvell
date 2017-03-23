/*
   uio_get_device_attributes.c
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

static int __uio_dev_attr_filter(char *filename)
{
	struct stat filestat;

	if (lstat(filename, &filestat))
		return 0;
	if (S_ISREG(filestat.st_mode))
		return 1;
	return 0;
}

int uio_get_device_attributes(struct uio_info_t* info)
{
	struct dirent **namelist;
	struct uio_dev_attr_t *attr, *last = NULL;
	char fullname[96];
	int n;

	info->dev_attrs = NULL;
	snprintf(fullname, sizeof(fullname),
		 "/sys/class/uio/uio%d/device", info->uio_num);
	n = scandir(fullname, &namelist, 0, alphasort);
	if (n < 0)
		return -1;

	while(n--) {
		snprintf(fullname, sizeof(fullname),
			 "/sys/class/uio/uio%d/device/%s",
			info->uio_num, namelist[n]->d_name);
		if (!__uio_dev_attr_filter(fullname))
			continue;
		attr = kmalloc(sizeof(struct uio_dev_attr_t), GFP_KERNEL);
		if (!attr)
			return -1;
		strncpy(attr->name, namelist[n]->d_name, UIO_MAX_NAME_SIZE);
		free(namelist[n]);
		if (__uio_line_from_file(fullname, attr->value)) {
			kfree(attr);
			continue;
		}

		if (!info->dev_attrs) {
			info->dev_attrs = attr;
		} else {
			if (last) {
				last->next = attr;
			} else {
				pr_err("[%s]Empty attribute list.\n", __func__);
				return -1;
			}
		}
		attr->next = NULL;
		last = attr;
	}
	free(namelist);

	return 0;
}
