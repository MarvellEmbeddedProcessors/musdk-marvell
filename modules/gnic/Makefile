#*****************************************************************************
# Copyright (c) 2016 Marvell.
# SPDX-License-Identifier: GPL-2.0-only
#****************************************************************************/

ifneq ($(KERNELRELEASE),)
# kbuild part of makefile
include Kbuild

else
# normal makefile
HDRS := $(shell uname -r)
KDIR ?=/usr/src/linux-headers-$(HDRS)/

default:
	ln -sf giu_nic.c src/giu_nic.plat.c; \
	ln -sf giu_nic_ethtool.c src/giu_nic_ethtool.plat.c; \
	$(MAKE) -C $(KDIR) M=$$PWD modules

clean:
	rm -f src/*.plat.c; \
	$(MAKE) -C $(KDIR) M=$$PWD clean
endif
