#
# File Name: Makefile
#
# Description: Makefile to build kernel mv_pp_uio module.
#
# Copyright (C) 2016 ENEA AB
#
# This program is free software; you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the
# Free Software Foundation version 2.
#
# This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
# whether express or implied; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Public License for more details.
#

KDIR ?=

CFLAGS_mv_pp_uio.o := -DDEBUG
ccflags-y := -std=gnu99 -Wno-declaration-after-statement

obj-m += mv_pp_uio.o

default all:
	make -C $(KDIR) M=$$PWD modules

clean:
	make -C $(KDIR) M=$$PWD clean

install:
	@mkdir -p $(DESTDIR)/usr/local/lib
	@cp ../../tmp/lib/libmusdk.la $(DESTDIR)/usr/local/lib
	@cp ../../tmp/lib/libmusdk.so $(DESTDIR)/usr/local/lib
