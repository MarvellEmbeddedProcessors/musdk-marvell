/******************************************************************************
 *	Copyright (C) 2016 Marvell International Ltd.
 *
 *  If you received this File from Marvell, you may opt to use, redistribute
 *  and/or modify this File under the following licensing terms.
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *	* Redistributions of source code must retain the above copyright
 *	  notice, this list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the distribution.
 *
 *	* Neither the name of Marvell nor the names of its contributors may be
 *	  used to endorse or promote products derived from this software
 *	  without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

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

#include "env/of.h"
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
