/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __STD_INTERNAL_H__
#define __STD_INTERNAL_H__

#ifdef __KERNEL__

#include <linux/types.h>
#include <linux/stddef.h>
#include <linux/inet.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/if_arp.h>
#include <linux/export.h>
#include <linux/ctype.h>
#include <linux/spinlock.h>
#include <linux/etherdevice.h>
#include <linux/if_vlan.h>
#include <linux/of_net.h>
#include <linux/utsname.h>

#include <asm-generic/io.h>

#include "env/spinlock.h"

#define ENOTSUP          252
#define mvlog2(n)	ilog2(n)


#ifndef __BIG_ENDIAN
	#define __BIG_ENDIAN __ORDER_BIG_ENDIAN__
#endif
#ifndef __BYTE_ORDER
	#define __BYTE_ORDER __BYTE_ORDER__
#endif

#else /* __KERNEL__ */

#include "mv_std.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include <ctype.h>
#include <dirent.h>
#include <fcntl.h>
#include <limits.h>
#include <pthread.h>
#include <sys/mman.h>
#include <unistd.h>
#include <assert.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <netinet/ip6.h>
#include <netinet/udp.h>
#include <arpa/inet.h>
#include <net/if_arp.h>
#include <sys/ioctl.h>
#include <sys/utsname.h>
#include <linux/ethtool.h>
#include <linux/sockios.h>
#include <linux/kdev_t.h>

#include <getopt.h>
#include <ifaddrs.h>

#include "env/mv_of.h"
#include "env/spinlock.h"
#include "env/io.h"
#include "env/netdev.h"

/* TODO: move to configure script */
#define MVCONF_IOMEM_USE_UIO

#endif /* __KERNEL__ */

#include "env/sys_iomem.h"
#include "lib/file_utils.h"
#include "lib/lib_misc.h"

#endif /* __STD_INTERNAL_H__ */
