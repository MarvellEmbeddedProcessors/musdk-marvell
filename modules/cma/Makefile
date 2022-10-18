#*****************************************************************************
# Copyright (c) 2016 Marvell.
# SPDX-License-Identifier: GPL-2.0-only
#****************************************************************************/

KDIR ?=

CFLAGS_musdk_cma.o := -DDEBUG
ccflags-y := -std=gnu99 -Wno-declaration-after-statement

obj-m += musdk_cma.o

default all:
	make -C $(KDIR) M=$$PWD modules

clean:
	make -C $(KDIR) M=$$PWD clean

install:
