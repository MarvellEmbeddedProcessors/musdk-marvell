/*
   uio_free.c
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

void uio_free_dev_attrs(struct uio_info_t* info)
{
	struct uio_dev_attr_t *p1, *p2;
	p1 = info->dev_attrs;
	while (p1) {
		p2 = p1->next;
		free(p1);
		p1 = p2;
	}
	info->dev_attrs = NULL;
}

void uio_free_info(struct uio_info_t* info)
{
	struct uio_info_t *p1,*p2;
	p1 = info;
	while (p1) {
		uio_free_dev_attrs(p1);
		p2 = p1->next;
		free(p1);
		p1 = p2;
	}
}

/**
 * uio_free_mem_info
 *
 * Deallocate uio_mem_info_t list
 *
 * @param info      Item  uio_mem_info_t
 */
void uio_free_mem_info(struct uio_mem_t *info)
{
	if (info) {
		free(info);
		info = NULL;
	}
}
