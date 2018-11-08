/*******************************************************************************
 * Copyright (C) Marvell International Ltd. and its affiliates
 *
 * This software file (the "File") is owned and distributed by Marvell
 * International Ltd. and/or its affiliates ("Marvell") under the following
 * alternative licensing terms.  Once you have made an election to distribute the
 * File under one of the following license alternatives, please (i) delete this
 * introductory statement regarding license alternatives, (ii) delete the three
 * license alternatives that you have not elected to use and (iii) preserve the
 * Marvell copyright notice above.
 *
 ********************************************************************************
 * Marvell Commercial License Option
 *
 * If you received this File from Marvell and you have entered into a commercial
 * license agreement (a "Commercial License") with Marvell, the File is licensed
 * to you under the terms of the applicable Commercial License.
 *
 ********************************************************************************
 * Marvell GPL License Option
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File in accordance with the terms and conditions of the General
 * Public License Version 2, June 1991 (the "GPL License"), a copy of which is
 * available along with the File in the license.txt file or by writing to the Free
 * Software Foundation, Inc., or on the worldwide web at http://www.gnu.org/licenses/gpl.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY
 * DISCLAIMED.  The GPL License provides additional details about this warranty
 * disclaimer.
 *
 ********************************************************************************
 * Marvell GNU General Public License FreeRTOS Exception
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File in accordance with the terms and conditions of the Lesser
 * General Public License Version 2.1 plus the following FreeRTOS exception.
 * An independent module is a module which is not derived from or based on
 * FreeRTOS.
 * Clause 1:
 * Linking FreeRTOS statically or dynamically with other modules is making a
 * combined work based on FreeRTOS. Thus, the terms and conditions of the GNU
 * General Public License cover the whole combination.
 * As a special exception, the copyright holder of FreeRTOS gives you permission
 * to link FreeRTOS with independent modules that communicate with FreeRTOS solely
 * through the FreeRTOS API interface, regardless of the license terms of these
 * independent modules, and to copy and distribute the resulting combined work
 * under terms of your choice, provided that:
 * 1. Every copy of the combined work is accompanied by a written statement that
 * details to the recipient the version of FreeRTOS used and an offer by yourself
 * to provide the FreeRTOS source code (including any modifications you may have
 * made) should the recipient request it.
 * 2. The combined work is not itself an RTOS, scheduler, kernel or related
 * product.
 * 3. The independent modules add significant and primary functionality to
 * FreeRTOS and do not merely extend the existing functionality already present in
 * FreeRTOS.
 * Clause 2:
 * FreeRTOS may not be used for any competitive or comparative purpose, including
 * the publication of any form of run time or compile time metric, without the
 * express permission of Real Time Engineers Ltd. (this is the norm within the
 * industry and is intended to ensure information accuracy).
 *
 ********************************************************************************
 * Marvell BSD License Option
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File under the following licensing terms.
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *	* Redistributions of source code must retain the above copyright notice,
 *	  this list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the distribution.
 *
 *	* Neither the name of Marvell nor the names of its contributors may be
 *	  used to endorse or promote products derived from this software without
 *	  specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************/

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

