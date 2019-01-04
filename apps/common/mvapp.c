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

#define _GNU_SOURCE         /* See feature_test_macros(7) */
#include <string.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <termios.h>
#include <fcntl.h>
#include <stdarg.h>

#include "mvapp_std.h"
#include "utils.h"
#include "cli.h"
#include "mvapp.h"

#define MAX_NUM_CORES		32
#define CTRL_TRD_DEFAULT_THRESH	100

#define CLI_FILE_VAR_DIR	"/var/"
#define CLI_FILE_NAME_PREFIX	"musdk-cli"
#define CLI_FILE_MAX_FILE_NAME	32
#define CLI_BUFSIZE		0x1000

#define cpuset_t	cpu_set_t

struct trd_desc {
	int		 id;
	int		 cpu;
	pthread_t	 trd;
	struct mvapp	*mvapp;
};

struct mvapp {
	int			 num_cores;
	u64			 cores_mask;
	int			 master_core;

	int			 running;

	struct cli		*cli;
	int			 (*print_cb)(const char *fmt, ...);
	char			 cli_in_filename[CLI_FILE_MAX_FILE_NAME];
	char			 cli_out_filename[CLI_FILE_MAX_FILE_NAME];
	int			 fd;
	pthread_t		 cli_trd;

	void			*global_arg;
	int			 (*init_local_cb)(void *, int id, void **);
	int			 (*main_loop_cb)(void *, int *);
	int			 (*ctrl_cb)(void *);
	void			 (*deinit_local_cb)(void *);

	pthread_mutex_t		 trd_lock;
	volatile u64		 bar_mask;

	int			 ctrl_thresh; /* in u-secs */
	struct trd_desc		 ctrl;
	struct trd_desc		 lcls[MAX_NUM_CORES];
};

struct mvapp *_mvapp;


/* set the thread affinity. */
static int setaffinity(pthread_t me, int i)
{
	cpuset_t cpumask;

	if (i == -1)
		return 0;

	/* Set thread affinity.*/
	CPU_ZERO(&cpumask);
	CPU_SET(i, &cpumask);

	if (pthread_setaffinity_np(me, sizeof(cpuset_t), &cpumask) != 0) {
		pr_err("Unable to set affinity: %s\n", strerror(errno));
		return 1;
	}
	return 0;
}

static char getchar_by_file_cb(void)
{
	struct mvapp	*mvapp = _mvapp;
	int              b_read;
	char             ch;

	if (mvapp->fd <= 0) {
		mvapp->fd = open(mvapp->cli_in_filename, O_RDONLY);
		/* in case no input, sleep for while before the next attempt */
		if (mvapp->fd <= 0) {
			usleep(mvapp->ctrl_thresh);
			return 0;
		}
	}
	b_read = read(mvapp->fd, &ch, 1);
	if (!b_read || (ch == '\n')) {
		close(mvapp->fd);
		remove(mvapp->cli_in_filename);
		mvapp->fd = 0;
	}

	return ch;
}

static char getchar_cb(void)
{
	struct mvapp	*mvapp = _mvapp;
	struct termios	 oldattr, newattr;
	int		 ch;

	tcgetattr(STDIN_FILENO, &oldattr);
	newattr = oldattr;
	newattr.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newattr);
	ch = getchar();
	tcsetattr(STDIN_FILENO, TCSANOW, &oldattr);

	/* in case no input, sleep for while before the next attempt */
	if (ch == '\0')
		usleep(mvapp->ctrl_thresh);

	return ch;
}

static int print_to_file_cb(const char *fmt, ...)
{
	struct mvapp	*mvapp = _mvapp;
	va_list ap;
	char buf[CLI_BUFSIZE];
	int  n, fd, size = CLI_BUFSIZE;

	if (!mvapp) {
		pr_err("No mvapp or mvapp-cli obj!\n");
		return -EINVAL;
	}

	va_start(ap, fmt);
	n = vsnprintf(buf, size, fmt, ap);
	if (!((n > 0) && (n < size)))
		printf("%s: buffer overflow (%d chars)\n", __func__, n);
	va_end(ap);

	fd = open(mvapp->cli_out_filename, O_CREAT | O_WRONLY | O_APPEND);
	if (fd <= 0) {
		pr_err("can't open CLI file (%s)\n", mvapp->cli_out_filename);
		return -EIO;
	}
	write(fd, buf, n);
	close(fd);
	return 0;
}

static int print_cb(const char *fmt, ...)
{
	va_list ap;
	char buf[CLI_BUFSIZE];
	int  n, size = CLI_BUFSIZE;

	va_start(ap, fmt);
	n = vsnprintf(buf, size, fmt, ap);
	if (!((n > 0) && (n < size)))
		printf("%s: buffer overflow (%d chars)\n", __func__, n);
	va_end(ap);

	return printf("%s", buf);
}

static void sigint_h(int sig)
{
	(void)sig;	/* UNUSED */
	printf("\nPress enter to exit...\n");
	_mvapp->running = 0;
	signal(SIGINT, SIG_DFL);
}

static int run_local(struct mvapp *mvapp, int id)
{
	void	*local_arg = NULL;
	int	 err = 0;

	if (mvapp->init_local_cb) {
		err = mvapp->init_local_cb(mvapp->global_arg, id, &local_arg);
		if (err) {
			mvapp->running = 0;
			return err;
		}
	}
	/* wait until all threads will complete initialization stage */
	mvapp_barrier();

	if (mvapp->main_loop_cb)
		while (mvapp->running && !err)
			err = mvapp->main_loop_cb(local_arg, &mvapp->running);

	/* mark all other threads to exit in case there was an error */
	mvapp->running = 0;

	/* wait until all threads will stop running */
	mvapp_barrier();

	if (mvapp->deinit_local_cb)
		mvapp->deinit_local_cb(local_arg);

	return err;
}

static void *cli_thr_cb(void *arg)
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

static void *ctrl_thr_cb(void *arg)
{
	struct trd_desc	*desc = (struct trd_desc *)arg;
	struct mvapp	*mvapp;
	int		 err = 0;

	if (!desc) {
		pr_err("no thread descriptor obj given!\n");
		err = -EINVAL;
		pthread_exit(&err);
		return NULL;
	}
	if (!desc->mvapp) {
		pr_err("no mvapp obj given to thread!\n");
		err = -EINVAL;
		pthread_exit(&err);
		return NULL;
	}
	mvapp = desc->mvapp;

	if (!mvapp->ctrl_cb) {
		pr_err("no CTRL CB given!\n");
		err = -EINVAL;
		pthread_exit(&err);
		return NULL;
	}

	if (desc->cpu > 0) {
		/* Set thread affinity */
		if (setaffinity(desc->trd, desc->cpu)) {
			pr_err("Failed to set affinity for CTRL thread!\n");
			err = -EFAULT;
			pthread_exit(&err);
			return NULL;
		}
	}
	pr_debug("CTRL thread is running on CPU %d\n", sched_getcpu());

	while (mvapp->running && !err) {
		usleep(mvapp->ctrl_thresh);
		err = mvapp->ctrl_cb(mvapp->global_arg);
	}

	pthread_exit(&err);
	return NULL;
}

static void *local_thr_cb(void *arg)
{
	struct trd_desc	*desc = (struct trd_desc *)arg;
	int		 err = 0;

	if (!desc) {
		pr_err("no thread descriptor obj given!\n");
		err = -EINVAL;
		pthread_exit(&err);
		return NULL;
	}
	if (!desc->mvapp) {
		pr_err("no mvapp obj given to thread!\n");
		err = -EINVAL;
		pthread_exit(&err);
		return NULL;
	}

	/* Set thread affinity */
	if (setaffinity(desc->trd, desc->cpu)) {
		pr_err("Failed to set affinity for app thread!\n");
		err = -EFAULT;
		pthread_exit(&err);
		return NULL;
	}
	pr_debug("Thread %d is running on CPU %d\n", desc->id, sched_getcpu());

	err = run_local(desc->mvapp, desc->id);

	pthread_exit(&err);
	return NULL;
}

int mvapp_go(struct mvapp_params *mvapp_params)
{
	struct mvapp	*mvapp;
	int		 i, j, err = 0;
	u64		 mask;

	printf("Marvell Armada US (Build: %s %s)\n", __DATE__, __TIME__);

	mvapp = (struct mvapp *)malloc(sizeof(struct mvapp));
	if (!mvapp) {
		pr_err("no mem for MVAPP obj!\n");
		return -ENOMEM;
	}
	memset(mvapp, 0, sizeof(struct mvapp));

	mvapp->num_cores = mvapp_params->num_cores;
	if (mvapp->num_cores > system_ncpus()) {
		pr_err("Invalid num cores (%d, vs %d)!\n",
		       mvapp->num_cores, system_ncpus());
		return -EINVAL;
	}
	mvapp->cores_mask = mvapp_params->cores_mask;
	if (!mvapp->cores_mask) {
		mask = 1;
		for (i = 0; i < mvapp->num_cores; i++, mask <<= 1)
			mvapp->cores_mask |= mask;
	} else {
		mask = 1;
		for (i = 0; i < mvapp->num_cores; i++, mask <<= 1)
			if (mask && mvapp->cores_mask) {
				mvapp->master_core = i;
				break;
			}
	}
	mvapp->bar_mask = mvapp->cores_mask;

	if (pthread_mutex_init(&mvapp->trd_lock, NULL) != 0) {
		pr_err("init lock failed!\n");
		free(mvapp);
		return -EIO;
	}

	if (mvapp_params->ctrl_cb_threshold)
		mvapp->ctrl_thresh = mvapp_params->ctrl_cb_threshold;
	else
		mvapp->ctrl_thresh = CTRL_TRD_DEFAULT_THRESH;

	if (mvapp_params->use_cli) {
		struct cli_params	cli_params;

		memset(&cli_params, 0, sizeof(cli_params));

		if (mvapp_params->use_cli == MVAPP_CLI_MODE_ENABLED_BY_FILE) {
			mvapp->print_cb = print_to_file_cb;
			cli_params.get_char_cb = getchar_by_file_cb;

			snprintf(mvapp->cli_in_filename,
				sizeof(mvapp->cli_in_filename),
				"%s%s-in-%d",
				CLI_FILE_VAR_DIR,
				CLI_FILE_NAME_PREFIX,
				0);
			/* remove file from previous runs */
			err = remove(mvapp->cli_in_filename);
			/* check if there was an error and if so check that it's not "No such file or directory" */
			if (err && errno != 2) {
				pr_err("can't delete regfile! (%s)\n", strerror(errno));
				cli_free(mvapp->cli);
				free(mvapp);
				return err;
			}
			snprintf(mvapp->cli_out_filename,
				sizeof(mvapp->cli_out_filename),
				"%s%s-out-%d",
				CLI_FILE_VAR_DIR,
				CLI_FILE_NAME_PREFIX,
				0);
			/* remove file from previous runs */
			err = remove(mvapp->cli_out_filename);
			/* check if there was an error and if so check that it's not "No such file or directory" */
			if (err && errno != 2) {
				pr_err("can't delete regfile! (%s)\n", strerror(errno));
				cli_free(mvapp->cli);
				free(mvapp);
				return err;
			}
		} else {
			mvapp->print_cb = print_cb;
			cli_params.get_char_cb = getchar_cb;
		}
		cli_params.print_cb = mvapp->print_cb;
		cli_params.echo = 1;
		mvapp->cli = cli_init(&cli_params);
		if (!mvapp->cli) {
			pr_err("CLI init failed!\n");
			free(mvapp);
			return -EIO;
		}
	}

	/* convert the threshold from m-secs to u-secs */
	mvapp->ctrl_thresh *= 1000;

	_mvapp = mvapp;

	if (mvapp_params->init_global_cb) {
		err = mvapp_params->init_global_cb(mvapp_params->global_arg);
		if (err) {
			if (mvapp->cli)
				cli_free(mvapp->cli);
			free(mvapp);
			return err;
		}
	}

	mvapp->running = 1;
	signal(SIGINT, sigint_h);

	if (mvapp->cli) {
		err = pthread_create(&mvapp->cli_trd,
				     NULL,
				     cli_thr_cb,
				     mvapp);
		if (err) {
			pr_err("Failed to start CLI thread!\n");
			return -EFAULT;
		}
	}

	mvapp->global_arg	= mvapp_params->global_arg;
	mvapp->init_local_cb	= mvapp_params->init_local_cb;
	mvapp->main_loop_cb	= mvapp_params->main_loop_cb;
	mvapp->ctrl_cb		= mvapp_params->ctrl_cb;
	mvapp->deinit_local_cb	= mvapp_params->deinit_local_cb;

	j = 0;
	for (i = 0; i < mvapp->num_cores; i++) {
		/* Calculate affinity for this thread */
		for (; !((1 << j) & mvapp->cores_mask); j++)
			;
		mvapp->lcls[i].id = i;
		mvapp->lcls[i].cpu = j++;
		mvapp->lcls[i].mvapp = mvapp;

		err = pthread_create(&mvapp->lcls[i].trd, NULL, local_thr_cb, &mvapp->lcls[i]);
		if (err) {
			pr_err("Failed to start app thread %d (on CPU %d)!\n", i, j);
			return -EFAULT;
		}
	}

	if (mvapp->ctrl_cb) {
		mvapp->ctrl.id = 0;
		mvapp->ctrl.cpu = -1; /* let the CTRL thread run on any CPU */
		mvapp->ctrl.mvapp = mvapp;

		err = pthread_create(&mvapp->ctrl.trd, NULL, ctrl_thr_cb, &mvapp->ctrl);
		if (err) {
			pr_err("Failed to start CTRL thread (on CPU %d)!\n", j);
			return -EFAULT;
		}
		err |= pthread_join(mvapp->ctrl.trd, NULL);
	}

	for (i = 0; i < mvapp->num_cores; i++)
		err |= pthread_join(mvapp->lcls[i].trd, NULL);

	mvapp->running = 0;
	if (mvapp->cli)
		cli_stop(mvapp->cli);

	if (mvapp->cli)
		err |= pthread_join(mvapp->cli_trd, NULL);

	if (mvapp_params->deinit_global_cb)
		mvapp_params->deinit_global_cb(mvapp_params->global_arg);

	if (mvapp->cli)
		cli_free(mvapp->cli);
	free(mvapp);

	printf("bye ...\n");

	return err;
}

void mvapp_barrier(void)
{
	struct mvapp	*mvapp = _mvapp;
	u64		 cores_mask = (u64)(1 << sched_getcpu());

	if (!mvapp) {
		pr_err("No mvapp or mvapp-cli obj!\n");
		return;
	}

	pthread_mutex_lock(&mvapp->trd_lock);
	/* Mark this core's presence */
	mvapp->bar_mask &= ~(cores_mask);

	if (mvapp->bar_mask) {
		pthread_mutex_unlock(&mvapp->trd_lock);
		/* Wait until barrier is reset */
		while (!(mvapp->bar_mask & cores_mask))
			;
	} else {
		/* Last core to arrive - reset the barrier */
		mvapp->bar_mask = mvapp->cores_mask;
		pthread_mutex_unlock(&mvapp->trd_lock);
	}
}

int mvapp_register_cli_cmd(struct cli_cmd_params *cmd_params)
{
	struct mvapp *mvapp = _mvapp;

	if (!mvapp || !mvapp->cli) {
		pr_err("No mvapp or mvapp-cli obj!\n");
		return -EINVAL;
	}

	return cli_register_cmd(mvapp->cli, cmd_params);
}

int mvapp_unregister_cli_cmd(char *name)
{
	struct mvapp *mvapp = _mvapp;

	if (!mvapp || !mvapp->cli) {
		pr_err("No mvapp or mvapp-cli obj!\n");
		return -EINVAL;
	}

	return cli_unregister_cmd(mvapp->cli, name);
}

int mvapp_print(const char *fmt, ...)
{
	struct mvapp	*mvapp = _mvapp;
	va_list ap;
	char buf[CLI_BUFSIZE];
	int  n, size = CLI_BUFSIZE;
	int	 (*print_cb)(const char *fmt, ...);

	if (!mvapp || !mvapp->print_cb) {
		pr_warn("No mvapp or mvapp-cli obj!\n");
		print_cb = printf;
	} else
		print_cb = mvapp->print_cb;

	va_start(ap, fmt);
	n = vsnprintf(buf, size, fmt, ap);
	if (!((n > 0) && (n < size)))
		printf("%s: buffer overflow (%d chars)\n", __func__, n);
	va_end(ap);

	return print_cb("%s", buf);
}
