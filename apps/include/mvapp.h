/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __MVAPP_H__
#define __MVAPP_H__

#include "mvapp_std.h"
#include "cli.h"

enum mvapp_cli_mode {
	MVAPP_CLI_MODE_NONE = 0,
	MVAPP_CLI_MODE_ENABLED,
	MVAPP_CLI_MODE_ENABLED_BY_FILE,
	MVAPP_CLI_MODE_OUT_OF_RANGE
};

struct mvapp_params {
	enum mvapp_cli_mode	 use_cli;
	int			 num_cores;
	u64			 cores_mask;

	void			*global_arg;
	int			 (*init_global_cb)(void *);
	void			 (*deinit_global_cb)(void *);

	int			 (*init_local_cb)(void *, int id, void **);
	void			 (*deinit_local_cb)(void *);
	/** Main application loop thread callback; application may run in endless loop within this
	 *  callback as long as the 'running' flag is set.
	 */
	int			 (*main_loop_cb)(void *, int *);
	/** Application control thread callback; application may use this callback in order to run
	 *  some control operations. Note that app must not run "forever" loop within this callback.
	 */
	int			 (*ctrl_cb)(void *);
	/** Threshold that will be used between the calls for 'ctrl_cb' in m-secs.
	 *  '0' value means to use the default; By default, the threshold is 100mSecs.
	 */
	int			 ctrl_cb_threshold;
};

int mvapp_go(struct mvapp_params *mvapp_params);

void mvapp_barrier(void);

int mvapp_register_cli_cmd(struct cli_cmd_params *cmd_params);
int mvapp_unregister_cli_cmd(char *name);

int mvapp_print(const char *fmt, ...);

#endif /* __MVAPP_H__ */
