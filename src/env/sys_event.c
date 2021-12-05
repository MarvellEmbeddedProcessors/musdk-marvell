/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <fcntl.h>
#include <unistd.h>
#include <sys/poll.h>

#include "std_internal.h"
#include "lib/uio_helper.h"

#include "env/mv_sys_event.h"

#define UIO_EV_FILE_NAME	"/dev/uio"

struct sys_event_priv {
	int fd;
	int (*event_validate)(void *); /* fn validates that returned event matches blocking criteria */
	void *driver_data; /* driver specific data */
};

int mv_sys_event_create(struct mv_sys_event_params *params, struct mv_sys_event **ev)
{
	struct mv_sys_event *ev_local;
	struct sys_event_priv *ev_priv;
	struct uio_info_t *uio_info;
	char tmp_name[32];
	char ev_name[16];

	ev_local = kcalloc(1, sizeof(struct mv_sys_event), GFP_KERNEL);
	if (!ev_local)
		return -ENOMEM;

	ev_priv = kcalloc(1, sizeof(struct sys_event_priv), GFP_KERNEL);
	if (!ev_priv) {
		kfree(ev_priv);
		return -ENOMEM;
	}
	ev_local->priv = ev_priv;

	snprintf(tmp_name, sizeof(tmp_name), "uio_%s", params->name);
	uio_info = uio_find_devices_byname(tmp_name);
	if (!uio_info) {
		pr_err("uio device (%s) is not found!\n", tmp_name);
		goto error;
	}

	snprintf(ev_name, sizeof(ev_name), UIO_EV_FILE_NAME"%d", uio_info->uio_num);
	ev_priv->fd = open(ev_name, O_RDONLY);
	if (ev_priv->fd < 0) {
		pr_err("Can't open file (%s) - %s\n", ev_name, strerror(errno));
		goto error;
	}
	ev_priv->event_validate = params->event_validate;
	ev_priv->driver_data = params->driver_data;

	*ev = ev_local;
	pr_debug("%s: Event opened: fd = %d\n", __func__, ev_priv->fd);

	return 0;
error:
	kfree(ev_priv);
	kfree(ev_local);

	return -EINVAL;
}

int mv_sys_event_destroy(struct mv_sys_event *ev)
{
	struct sys_event_priv *ev_priv = (struct sys_event_priv *)ev->priv;

	close(ev_priv->fd);
	kfree(ev_priv);
	kfree(ev);

	return 0;
}

int mv_sys_event_get_fd(struct mv_sys_event *ev, int *fd)
{
	struct sys_event_priv *ev_priv = (struct sys_event_priv *)ev->priv;

	if (!ev || !fd)
		return -1;

	*fd = ev_priv->fd;

	return 0;
}

int mv_sys_event_get_driver_data(struct mv_sys_event *ev, void **driver_data)
{
	struct sys_event_priv *ev_priv = (struct sys_event_priv *)ev->priv;

	if (!ev || !driver_data)
		return -1;

	*driver_data = ev_priv->driver_data;
	return 0;
}


#define USE_POLL
#if defined(USE_POLL)
#define MAX_POLL_EVENTS		32
int mv_sys_event_poll(struct mv_sys_event **ev, int num, int timeout)
{
	int i, ret, done = 0;
	struct sys_event_priv *ev_priv;
	struct pollfd fds[MAX_POLL_EVENTS] = { {0} };

	if (num > MAX_POLL_EVENTS) {
		pr_warn("%s: Too many events %d. Max number of events is %d\n",
			__func__, num, MAX_POLL_EVENTS);
		num = MAX_POLL_EVENTS;
	}
	for (i = 0; i < num; i++) {
		struct sys_event_priv *ev_priv = (struct sys_event_priv *)ev[i]->priv;

		fds[i].fd = ev_priv->fd;
		fds[i].events = ev[i]->events;
		fds[i].revents = 0;
		ev[i]->revents = 0;
	}
	ret = poll(fds, num, timeout);
	if (unlikely(ret <= 0))
		return ret;

	for (i = 0; i < num; i++) {
		if (fds[i].revents) {
			u32 info, nbytes;

			ev_priv = (struct sys_event_priv *)ev[i]->priv;
			if (ev_priv->event_validate) {
				fds[i].revents &= ev_priv->event_validate(ev_priv->driver_data);
				if (!fds[i].revents)
					continue;
			}
			/* Consume event */
			nbytes = read(fds[i].fd, &info, sizeof(info));
			if (nbytes == (ssize_t)sizeof(info)) {
				ev[i]->revents = fds[i].revents;
				done++;
				if (done == ret)
					break;
			}
		}
	}
	return done;
}
#elif defined(USE_READ)
int mv_sys_event_poll(struct mv_sys_event **ev, int num, int timeout)
{
	struct sys_event_priv *ev_priv = (struct sys_event_priv *)ev[0]->priv;
	u32 isr, count;

	count  = read(ev_priv->fd, &isr, sizeof(isr));
	ev[0]->revents = ev[0]->events;

	pr_debug("%s: %d bytes returned. num of ISRs = %d\n", __func__, count, isr);

	return 1;
}
#else
#error "USE_POLL or USE_READ must be defined"
#endif

