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
	int i, ret;
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
	}
	ret = poll(fds, num, timeout);

	for (i = 0; i < num; i++) {
		struct sys_event_priv *ev_priv = (struct sys_event_priv *)ev[i]->priv;

		if (ev_priv->event_validate)
			ev[i]->revents = fds[i].revents & ev_priv->event_validate(ev_priv->driver_data);
		else
			ev[i]->revents = fds[i].revents;
	}

	pr_debug("%s: poll() returned. ret = %d revents[0]:%d\n", __func__, ret, ev[0]->revents);

	return ret;
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

