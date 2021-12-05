/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "std_internal.h"
#include "env/mv_sys_event.h"


#ifndef NOTUSED
#define NOTUSED(_a) ((_a) = (_a))
#endif /* !NOTUSED */


int mv_sys_event_create(struct mv_sys_event_params *params, struct mv_sys_event **ev)
{
	NOTUSED(params); NOTUSED(ev);
	pr_err("%s not supported!\n", __func__);
	return -ENOTSUPP;
}

int mv_sys_event_destroy(struct mv_sys_event *ev)
{
	NOTUSED(ev);
	pr_err("%s not supported!\n", __func__);
	return -ENOTSUPP;
}

int mv_sys_event_get_fd(struct mv_sys_event *ev, int *fd)
{
	NOTUSED(ev); NOTUSED(fd);
	pr_err("%s not supported!\n", __func__);
	return -ENOTSUPP;
}

int mv_sys_event_get_driver_data(struct mv_sys_event *ev, void **driver_data)
{
	NOTUSED(ev); NOTUSED(driver_data);
	pr_err("%s not supported!\n", __func__);
	return -ENOTSUPP;
}

int mv_sys_event_poll(struct mv_sys_event **ev, int num, int timeout)
{
	NOTUSED(ev); NOTUSED(num); NOTUSED(timeout);
	pr_err("%s not supported!\n", __func__);
	return -ENOTSUPP;
}
