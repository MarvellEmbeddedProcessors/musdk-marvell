/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __CLI_H__
#define __CLI_H__

#include "mvapp_std.h"

struct cli;

struct cli_cmd_params {
	const char	*name;
	const char	*desc;
	const char	*format;
	void		*cmd_arg;
	int		(*do_cmd_cb)(void *, int, char *[]);
};

struct cli_params {
	char		*prompt;
	int		 no_block;
	int		 echo;

	int		 (*print_cb)(const char *fmt, ...);
	char		 (*get_char_cb)(void);
};

struct cli *cli_init(struct cli_params *cli_paramss);
int  cli_free(struct cli *cli);

int  cli_run(struct cli *cli);
int  cli_stop(struct cli *cli);

int  cli_register_cmd(struct cli *cli, struct cli_cmd_params *cmd_params);
int  cli_unregister_cmd(struct cli *cli, char *name);

#endif /* __CLI_H__ */
