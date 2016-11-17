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


#include <string.h>
#include <pthread.h>

#include "mv_std.h"
#include "cli.h"
#include "mvapp.h"

#define MAX_NUM_CORES	32


struct mvapp {
	int			 num_cores;
	u32			 cores_mask;

	volatile int		 running;

	struct cli		*cli;
	pthread_t		 cli_trd;

	void			*global_arg;
	int			 (*init_local_cb)(void *, void **);
	int			 (*main_loop_cb)(void *, volatile int *);
	void			 (*deinit_local_cb)(void *);

	struct {
		pthread_t	 trd;
	} lcls[MAX_NUM_CORES];
};


struct mvapp *_mvapp = NULL;


static char getchar_cb(void)
{
	return (char)getchar();
}

static int run_local(struct mvapp *mvapp)
{
	void	*local_arg = NULL;
	int	 err = 0;

	if (mvapp->init_local_cb &&
	    ((err = mvapp->init_local_cb(mvapp->global_arg, &local_arg)) != 0))
		return err;

	/* TODO: add barrier */

	if (mvapp->main_loop_cb)
		while (mvapp->running && !err)
			err = mvapp->main_loop_cb(local_arg, &mvapp->running);

	if (mvapp->deinit_local_cb)
		mvapp->deinit_local_cb(local_arg);

	return err;
}

static void * cli_thr_cb(void *arg)
{
	struct mvapp	*mvapp = (struct mvapp *)arg;
	int		 err = 0;

	if (!mvapp) {
		pr_err("no mvapp obj given!\n");
		err = -EINVAL;
		pthread_exit(&err);
		return NULL;
	}
	if (!mvapp->cli) {
		pr_err("no CLI obj given!\n");
		err = -EINVAL;
		pthread_exit(&err);
		return NULL;
	}

	cli_run(mvapp->cli);

	mvapp->running = 0;

	pthread_exit(&err);
	return NULL;
}

static void * local_thr_cb(void *arg)
{
	struct mvapp	*mvapp = (struct mvapp *)arg;
	int		 err = 0;

	if (!mvapp) {
		pr_err("no mvapp obj given!\n");
		err = -EINVAL;
		pthread_exit(&err);
		return NULL;
	}

	/* TODO: set affinity */

	err = run_local(mvapp);

	pthread_exit(&err);
	return NULL;
}

int mvapp_go(struct mvapp_params *mvapp_params)
{
	struct mvapp	*mvapp;
	int		 i, err = 0;
	u32		 mask;

	printf("Marvell Armada US (Build: %s %s)\n", __DATE__, __TIME__);

	mvapp = (struct mvapp *)malloc(sizeof(struct mvapp));
	if (!mvapp) {
		pr_err("no mem for MVAPP obj!\n");
		return -ENOMEM;
	}
	memset(mvapp, 0, sizeof(struct mvapp));
	mvapp->num_cores = mvapp_params->num_cores;
	mvapp->cores_mask = mvapp_params->cores_mask;
	if (!mvapp->cores_mask) {
		mask = 1;
		for (i=0; i<mvapp->num_cores; i++, mask<<=1)
			mvapp->cores_mask |= mask;
	}

	if (mvapp_params->use_cli) {
		struct cli_params	cli_params;
		memset(&cli_params, 0, sizeof(cli_params));
		cli_params.print_cb = printf;
		cli_params.get_char_cb = getchar_cb;
		mvapp->cli = cli_init(&cli_params);
		if (!mvapp->cli) {
			pr_err("CLI init failed!\n");
			return -EIO;
		}
	}

	if (mvapp_params->init_global_cb &&
	    ((err = mvapp_params->init_global_cb(mvapp_params->global_arg)) != 0)) {
		if (mvapp->cli)
			cli_free(mvapp->cli);
		free(mvapp);
		return err;
	}

	mvapp->running = 1;

	if (mvapp->cli &&
	    ((err = pthread_create (&mvapp->cli_trd,
				    NULL,
				    cli_thr_cb,
				    mvapp)) != 0)) {
		pr_err("Failed to start CLI thread!\n");
		return -EFAULT;
	}

	mvapp->global_arg	= mvapp_params->global_arg;
	mvapp->init_local_cb	= mvapp_params->init_local_cb;
	mvapp->main_loop_cb	= mvapp_params->main_loop_cb;
	mvapp->deinit_local_cb	= mvapp_params->deinit_local_cb;

	if ((mvapp->num_cores == 1) && (mvapp->cores_mask == 1))
		err = run_local(mvapp);
	else {
		mask = 1;
		for (i=0; i<mvapp->num_cores; i++) {
			for (; !(mask & mvapp->cores_mask); mask<<=1) ;
			if ((err = pthread_create (&mvapp->lcls[i].trd,
						    NULL,
						    local_thr_cb,
						    mvapp)) != 0) {
				pr_err("Failed to start CLI thread!\n");
				return -EFAULT;
			}
		}

		for (i=0; i<mvapp->num_cores; i++)
			err |= pthread_join (mvapp->lcls[i].trd, NULL);
	}

	mvapp->running = 0;
	if (mvapp->cli)
		cli_stop(mvapp->cli);

	if (mvapp->cli)
		err |= pthread_join (mvapp->cli_trd, NULL);

	if (mvapp_params->deinit_global_cb)
		mvapp_params->deinit_global_cb(mvapp_params->global_arg);

	if (mvapp->cli)
		cli_free(mvapp->cli);
	free(mvapp);

	printf("bye ...\n");

	return err;
}
