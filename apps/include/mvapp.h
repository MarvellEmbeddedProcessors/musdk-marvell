/******************************************************************************
	Copyright (C) 2016 Marvell International Ltd.

  If you received this File from Marvell, you may opt to use, redistribute
  and/or modify this File under the following licensing terms.
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  	* Redistributions of source code must retain the above copyright
	  notice, this list of conditions and the following disclaimer.

  	* Redistributions in binary form must reproduce the above copyright
  	  notice, this list of conditions and the following disclaimer in the
  	  documentation and/or other materials provided with the distribution.

  	* Neither the name of Marvell nor the names of its contributors may be
  	  used to endorse or promote products derived from this software
	  without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#ifndef __MVAPP_H__
#define __MVAPP_H__

#include "mvapp_std.h"
#include "cli.h"

struct mvapp_params {
	int			 use_cli;
	int			 num_cores;
	u64			 cores_mask;

	void		*global_arg;
	int			 (*init_global_cb)(void *);
	void		 (*deinit_global_cb)(void *);

	int			 (*init_local_cb)(void *, void **);
	int			 (*main_loop_cb)(void *, volatile int *);
	void		 (*deinit_local_cb)(void *);
};

int mvapp_go(struct mvapp_params *mvapp_params);

void mvapp_barrier(void);

int mvapp_register_cli_cmd(struct cli_cmd_params *cmd_params);
int mvapp_unregister_cli_cmd(char *name);

#endif /* __MVAPP_H__ */
