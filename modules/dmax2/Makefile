#*****************************************************************************
# Copyright (c) 2016 Marvell.
# SPDX-License-Identifier: GPL-2.0-only
#****************************************************************************/

KDIR ?=

CFLAGS_mv_dmax2_uio.o := -DDEBUG
ccflags-y := -std=gnu99 -Wno-declaration-after-statement

obj-m += mv_dmax2_uio.o

default all:
	make -C $(KDIR) M=$$PWD modules

clean:
	make -C $(KDIR) M=$$PWD clean

install:
