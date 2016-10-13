/*
   uio_num_from_filename.c
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

int __uio_num_from_filename(char* name);

int __uio_num_from_filename(char* name)
{
	enum scan_states { ss_u, ss_i, ss_o, ss_num, ss_err };
	enum scan_states state = ss_u;
	int i=0, num = -1;
	char ch = name[0];
	while (ch && (state != ss_err)) {
		switch (ch) {
			case 'u':
				if (state == ss_u) state = ss_i;
				else state = ss_err;
				break;
			case 'i':
				if (state == ss_i) state = ss_o;
				else state = ss_err;
				break;
			case 'o':
				if (state == ss_o) state = ss_num;
				else state = ss_err;
				break;
			default:
				if ( (ch>='0') && (ch<='9')
						&& (state == ss_num) ) {
					if (num < 0) num = (ch - '0');
					else num = (num * 10) + (ch - '0');
				}
				else state = ss_err;
		}
		i++;
		ch = name[i];
	}
	if (state == ss_err) num = -1;
	return num;
}
