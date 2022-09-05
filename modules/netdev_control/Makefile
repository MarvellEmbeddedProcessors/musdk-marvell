#  SPDX-License-Identifier: GPL-2.0-only
#
#  Copyright (c) 2021 Marvell.

KDIR ?=/usr/src/linux-headers-`uname -r`/

CFLAGS_mv_netdev_control.o :=
ccflags-y := -std=gnu99 -Wno-declaration-after-statement

obj-m += netdev_control.o

default all:
	make -C $(KDIR) M=$$PWD modules

clean:
	make -C $(KDIR) M=$$PWD clean
