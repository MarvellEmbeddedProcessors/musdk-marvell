/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

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
	int (*event_validate)(void *); /* fn validates that returned event matches blocking criteria */
	void *driver_data; /* driver specific data */
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
int mv_sys_event_poll(struct mv_sys_event **ev, int num, int timeout);

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


int mv_sys_event_get_driver_data(struct mv_sys_event *ev, void **driver_data);


#endif /* __MV_SYS_EVENT_H__ */
