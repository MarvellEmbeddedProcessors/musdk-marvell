/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "std_internal.h"

void log_init(int log_to_stderr)
{
	int option;

	option = LOG_PID;
	if (log_to_stderr)
		option |= LOG_PERROR;

	/* Open a channel to the syslog daemon. We always use the application name
	 * as the log message prefix together with the PID. We use the default
	 * facility LOG_USER which best fits our app
	 */
	openlog(NULL, option, 0);
}

/* Close the log facility.
 * Currently we just close the syslog channel.
 * In the future we might dump or send the log somewhere.
 */
void log_close(void)
{
	closelog();
}
