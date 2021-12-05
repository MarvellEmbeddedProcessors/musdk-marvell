/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "std_internal.h"
#include "env/netdev.h"

/* Send IOCTL to linux*/
int mv_netdev_ioctl(u32 ctl, struct ifreq *s)
{
	int rc;
	int fd;

	fd = socket(AF_INET, SOCK_STREAM, 0);
	if (fd == -1) {
		pr_err("can't open socket: errno %d", errno);
		return -EFAULT;
	}

	rc = ioctl(fd, ctl, (char *)s);
	if (rc == -1) {
		pr_err("ioctl request failed: errno %d\n", errno);
		close(fd);
		return -EFAULT;
	}
	close(fd);
	return 0;
}

