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

#ifndef __MV_SYS_EVENT_H__
#define __MV_SYS_EVENT_H__

#include "mv_types.h"

/**
 *	List of bits can be used in "events" and "revents" bitmap.
 *	Only bit0 - POLLIN is supported.
 */
#define MV_SYS_EVENT_POLLIN	0x1

/**
 *	System event structure
 */
struct mv_sys_event {
	u16 events;	/**< requested events */
	u16 revents;	/**< returned events */
	void *priv;	/**< MUSDK proprietary event structure */
};

/* System event initialization parameters */
struct mv_sys_event_params {
	char name[16];	/**< Event name as defined in the file /sys/class/uio/uioX/name */
};

/**
 * Create and initialize system event instance
 *
 * @param[in]	params    - pointer to structure with system event initialization parameters.
 * @param[out]	ev        - address of place to save handler of new created system event instance.
 *
 * @retval	0         - success
 * @retval	Negative  - failure
 */
int mv_sys_event_create(struct mv_sys_event_params *params, struct mv_sys_event **ev);

/**
 * Delete system event instance
 *
 * @param[in]	ev	  - system event instance handler.
 *
 * @retval	0         - success
 * @retval	Negative  - failure
 */
int mv_sys_event_destroy(struct mv_sys_event *ev);

/**
 * Wait on system event instance (optionally with timeout)
 * The calling thread will be blocked
 *
 * @param[in]	ev	  - array of system event handlers.
 * @param[in]	num	  - number of system event handlers in "ev" array
 * @param[in]	timeout	  - time in msec to wakeup if no events occurred.
 *			    value -1 means no timeout.
 *
 * @retval	Positive  - number of system events which have nonzero revents fields
 * @retval	0	  - timeout, no ready events
 * @retval	Negative  - failure
 */
int mv_sys_event_poll(struct mv_sys_event *ev, int num, int timeout);

/**
 * Return event file descriptor
 *
 * @param[in]	ev	  - system event instance handler.
 * @param[out]	fd	  - file descriptor.
 *
 * @retval	0         - success
 * @retval	Negative  - failure
 */
int mv_sys_event_get_fd(struct mv_sys_event *ev, int *fd);

#endif /* __MV_SYS_EVENT_H__ */
