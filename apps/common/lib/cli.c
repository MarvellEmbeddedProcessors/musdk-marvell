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


#include <signal.h>
#include <time.h>
#include <pthread.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>

#include "mvapp_std.h"
#include "lib/list.h"

#include "cli.h"


#define CLI_MAX_LINE_LENGTH	256
#define CLI_TIMER_INTERVAL	128 /* 128 m-secs */
#define CLI_MAX_HISTORY		10


typedef struct mvtimer {
	uint64_t	 expires;
	int		 periodic;
	void		 (*tmr_expired_cb) (void *);
	void		*arg;

	timer_t		 timerid;
	struct sigevent	 sev;
} mvtimer_t;

static void timer_complete_cb(int sig, siginfo_t *si, void *uc)
{
	mvtimer_t	*tmr;

	tmr = LIST_OBJECT(si->si_value.sival_ptr, mvtimer_t, timerid);

	if (tmr->periodic) {
		struct itimerspec   its;

		/* Start the timer */
		its.it_value.tv_sec = tmr->expires / 1000;
		its.it_value.tv_nsec = (tmr->expires % 1000) * 1000000;
		its.it_interval.tv_sec = its.it_value.tv_sec;
		its.it_interval.tv_nsec = its.it_value.tv_nsec;

		if (timer_settime(tmr->timerid, 0, &its, NULL) == -1)
			pr_err("timer_settime failed!\n");
	}

	if (tmr->tmr_expired_cb)
		tmr->tmr_expired_cb(tmr->arg);

	//signal(sig, SIG_IGN);
}

static struct mvtimer * create_timer(void)
{
#define SIG				 SIGUSR1
	mvtimer_t		*tmr;
	static struct sigaction	 sa = {};

	tmr = (mvtimer_t *)malloc(sizeof(mvtimer_t));
	if (!tmr) {
		pr_err("No mem for timer obj!\n");
		return NULL;
	}
	memset(tmr, 0, sizeof(mvtimer_t));

	if (!sa.sa_flags) {
		//static sigset_t		 mask;

		/* Establish handler for timer signal */
		sa.sa_flags = SA_RESTART | SA_SIGINFO;
		sa.sa_sigaction = timer_complete_cb;
		sigemptyset(&sa.sa_mask);
		if (sigaction(SIG, &sa, NULL) == -1) {
			pr_err("sigaction failed!\n");
			return NULL;
		}

		/* Block timer signal temporarily */
/*
		sigemptyset(&mask);
		sigaddset(&mask, SIG);
		if (sigprocmask(SIG_SETMASK, &mask, NULL) == -1)
		{
			pr_err("sigprocmask failed!\n");
			return NULL;
		}
*/
	}

	tmr->sev.sigev_notify = SIGEV_SIGNAL;
	tmr->sev.sigev_signo = SIG;
	tmr->sev.sigev_value.sival_ptr = &tmr->timerid;
	if (timer_create(CLOCK_REALTIME, &tmr->sev, &tmr->timerid) == -1) {
		pr_err("timer_create failed!\n");
		return NULL;
	}

	return tmr;
}

static void free_timer(struct mvtimer *timer)
{
	mvtimer_t *tmr = (mvtimer_t *)timer;

	if (tmr) {
		if (tmr->expires)
			timer_delete(tmr->timerid);
		free(tmr);
	}
}

static void start_timer(struct mvtimer	*timer,
	uint32_t	 msecs,
	int		 periodic,
	void		 (*tmr_expired_cb)(void *arg),
	void		*arg)
{
	mvtimer_t	*tmr = (mvtimer_t *)timer;
	struct itimerspec   its;

	tmr->expires		= msecs;
	tmr->periodic	   = periodic;
	tmr->tmr_expired_cb = tmr_expired_cb;
	tmr->arg		  = arg;

	/* Start the timer */
	its.it_value.tv_sec = tmr->expires / 1000;
	its.it_value.tv_nsec = (tmr->expires % 1000) * 1000000;
	its.it_interval.tv_sec = its.it_value.tv_sec;
	its.it_interval.tv_nsec = its.it_value.tv_nsec;

	if (timer_settime(tmr->timerid, 0, &its, NULL) == -1)
		pr_err("timer_settime failed!\n");
}

static void stop_timer(struct mvtimer *timer)
{
	pr_warn("no supported yet!\n");
}

static void mod_timer(struct mvtimer *timer, uint32_t msecs)
{
	mvtimer_t		*tmr = (mvtimer_t *)timer;
	struct itimerspec	 its;

	tmr->expires		= msecs;

	/* Start the timer */
	its.it_value.tv_sec = tmr->expires / 1000;
	its.it_value.tv_nsec = (tmr->expires % 1000) * 1000000;
	its.it_interval.tv_sec = its.it_value.tv_sec;
	its.it_interval.tv_nsec = its.it_value.tv_nsec;

	if (timer_settime(tmr->timerid, 0, &its, NULL) == -1)
		pr_err("timer_settime failed!\n");
}


typedef struct cli_cmd {
	char		*name;
	char		*desc;
	char		*format;
	void		*cmd_arg;
	int		 (*do_cmd_cb)(void *, int, char *[]);
	struct list	 node;
} cli_cmd_t;
#define CLI_COMMAND_OBJ(ptr)  LIST_OBJECT(ptr, cli_cmd_t, node)

typedef struct cli_cmd_hist {
	char		line[CLI_MAX_LINE_LENGTH+5];
	int		length;
	struct list	node;
} cli_cmd_hist_t;
#define CLI_HISTORY_LINE_OBJ(ptr)  LIST_OBJECT(ptr, cli_cmd_hist_t, node)

typedef struct cli {
	char		*prompt;
	volatile int	 running;
	int		 no_block;
	int		 echo;
	int		 (*print_cb)   (const char *str, ...);
	char		 (*get_char_cb) (void);
	struct mvtimer	*timer;
	pthread_mutex_t	 lock;
	struct list	 cmds_lst;
	char		 line[CLI_MAX_LINE_LENGTH+5];
	int		 curr_pos;
	struct list	 hist_lst;
	int		 hist_req;
	struct list	*hist_lst_pos;
} cli_t;


static inline void clear_line(cli_t *cli_p)
{
	char empty_line[CLI_MAX_LINE_LENGTH];
	u32 size = strlen(cli_p->line) + strlen(cli_p->prompt) + 2;

	if (size > CLI_MAX_LINE_LENGTH)
		size = CLI_MAX_LINE_LENGTH;
	memset(empty_line, ' ', size);
	cli_p->print_cb("\r%s", empty_line);
	cli_p->print_cb("\r");
}


static void dump_usage(cli_t *cli_p)
{
	cli_p->print_cb("Usage: > <command> [options] [arg1 ... argN]\n");
	cli_p->print_cb("Type ? for help.\n");
}

static void dump_help(cli_t *cli_p)
{
	cli_cmd_t	*cmd;
	struct list	*lst_pos;

	cli_p->print_cb("Usage: <command> [options] [arg1 ... argN]\n"
			 "\n"
			 "Available commands:\n");

	LIST_FOR_EACH(lst_pos, &cli_p->cmds_lst) {
		cmd = CLI_COMMAND_OBJ(lst_pos);
		cli_p->print_cb("\t%s\t\tUsage: %s", cmd->name, cmd->name);
		if (cmd->format)
			cli_p->print_cb(" %s", cmd->format);
		cli_p->print_cb("\n\t\t%s\n", cmd->desc);
	}
}

static int help_cmd_cb(struct cli *cli, int argc, char *argv[])
{
	cli_t	   *cli_p = (cli_t *)cli;

	if (!cli_p) {
		pr_err("Invalid CLI obj provided!\n");
		return -EINVAL;
	}

	NOTUSED(argc);NOTUSED(argv);

	dump_help(cli_p);

	return 0;
}

static int quit_cmd_cb(struct cli *cli, int argc, char *argv[])
{
	cli_t	   *cli_p = (cli_t *)cli;

	if (!cli_p) {
		pr_err("Invalid CLI obj provided!\n");
		return -EINVAL;
	}

	NOTUSED(argc);NOTUSED(argv);

	cli_p->running = 0;

	return 0;
}


static void print_logo(cli_t *cli_p)
{
	cli_p->print_cb("              ......        ......	Marvell Inc. Copyright (c) 2016		\n");
	cli_p->print_cb("            ##......      ##......	All Rights Reserved			\n");
	cli_p->print_cb("          ####......    ####......						\n");
	cli_p->print_cb("        ######......  ######......						\n");
	cli_p->print_cb("      ######  ......######  ......						\n");
	cli_p->print_cb("    ######    ....######    ######	MUSDK Device Drivers			\n");
	cli_p->print_cb("  ######      ..######      ......	Version 0.1				\n");
	cli_p->print_cb("######        ######        ######	built on %s				\n\n\n",__DATE__);
}

static void get_hist_lst(cli_t *cli_p)
{
	cli_cmd_hist_t *hist_line;

	if (cli_p->line[cli_p->curr_pos] == '[')
		return;
	if (cli_p->line[cli_p->curr_pos] == 'A') { /* up */
		if ((cli_p->hist_lst_pos->next != &cli_p->hist_lst) && /* not end of hist_lst */
			(CLI_HISTORY_LINE_OBJ(cli_p->hist_lst_pos->next)->length != 0)) {
			cli_p->hist_lst_pos = cli_p->hist_lst_pos->next;
			hist_line = CLI_HISTORY_LINE_OBJ(cli_p->hist_lst_pos);
			clear_line(cli_p);
			cli_p->print_cb("%s> %s", cli_p->prompt, hist_line->line);
			cli_p->curr_pos = hist_line->length;
			memcpy(cli_p->line, hist_line->line, (uint32_t)cli_p->curr_pos);
		}
		cli_p->hist_req = 0;
	} else if (cli_p->line[cli_p->curr_pos] == 'B') { /* down */
		if ((cli_p->hist_lst_pos != &cli_p->hist_lst) &&
			(CLI_HISTORY_LINE_OBJ(cli_p->hist_lst_pos)->length != 0)) {
			hist_line = CLI_HISTORY_LINE_OBJ(cli_p->hist_lst_pos);
			clear_line(cli_p);
			cli_p->print_cb("%s> %s", cli_p->prompt, hist_line->line);
			cli_p->curr_pos = hist_line->length;
			memcpy(cli_p->line, hist_line->line, (uint32_t)cli_p->curr_pos);
			cli_p->hist_lst_pos = cli_p->hist_lst_pos->prev;
		}
		cli_p->hist_req = 0;
	}
}

static int get_line(cli_t *cli_p, int *p_NumOfCharsRead)
{
	cli_cmd_hist_t *hist_line;

	do {
		if ((cli_p->line[cli_p->curr_pos] = cli_p->get_char_cb()) == '\0')
			return -ENOMSG;

		if (cli_p->line[cli_p->curr_pos] == '\x1B') {
			return -ENOMSG;
		}

		if (cli_p->line[cli_p->curr_pos] == '[') {
			cli_p->hist_req = 1;
			continue;
		}

		if (cli_p->hist_req) {
			get_hist_lst(cli_p);
			continue;
		}

		if (cli_p->line[cli_p->curr_pos] == '\b') {
			if (cli_p->curr_pos) {
				cli_p->curr_pos--;
				if (cli_p->echo)
					cli_p->print_cb("\b \b");
			}
			continue;
		}

		if (cli_p->echo)
			cli_p->print_cb("%c", cli_p->line[cli_p->curr_pos]);
		if (cli_p->line[cli_p->curr_pos] != '\n') {
			if (cli_p->line[cli_p->curr_pos] == '\r') {
				if (cli_p->echo)
					cli_p->print_cb("\n");
				break;
			}
			cli_p->curr_pos++;
		}
		else
			break;
	} while (cli_p->curr_pos < CLI_MAX_LINE_LENGTH);
	cli_p->line[cli_p->curr_pos] = '\0';

	*p_NumOfCharsRead = cli_p->curr_pos;
	cli_p->curr_pos = 0;
	cli_p->hist_req = 0;
	cli_p->hist_lst_pos = &cli_p->hist_lst;

	/* Update the hist_lst lines */
	if (*p_NumOfCharsRead != 0) {
		hist_line = CLI_HISTORY_LINE_OBJ(cli_p->hist_lst.prev);
		list_del_init(&hist_line->node);
		memcpy(hist_line->line, cli_p->line, (uint32_t)(*p_NumOfCharsRead));
		hist_line->length = *p_NumOfCharsRead;
		list_add(&hist_line->node, &cli_p->hist_lst);
	}

	return 0;
}

static cli_cmd_t * find_cmd(cli_t *cli_p, char *name)
{
	cli_cmd_t	*cmd;
	struct list	*lst_pos;

	LIST_FOR_EACH(lst_pos, &cli_p->cmds_lst) {
		cmd = CLI_COMMAND_OBJ(lst_pos);
		if (strcmp(cmd->name, name) == 0)
			return cmd;
	}

	return NULL;
}

static int cmd_is_valid(cli_t *cli_p, const char *cmd)
{
	if (strstr (cmd, "|")) {
		cli_p->print_cb("character | (cmds_lst-pipeline) is not supported!\n");
		return 0;
	}
	if (strstr (cmd, ">") || strstr (cmd, "<")) {
		cli_p->print_cb("characters < or > (redirections) are not supported!\n");
		return 0;
	}
	if (strstr (cmd, "\t")) {
		cli_p->print_cb("character \\t (tab) is not supported! use space instead.\n");
		return 0;
	}
	return 1;
}

static void main_loop(struct cli *cli)
{
	cli_t	*cli_p = (cli_t *)cli;
	int	 rc;
	int	 num_chars_read;

	if ((rc = get_line(cli_p, &num_chars_read)) == 0) {
		char		*token;
		char		 tmp_line[CLI_MAX_LINE_LENGTH+5];
		cli_cmd_t	*cmd;

		memcpy(tmp_line, cli_p->line, CLI_MAX_LINE_LENGTH+5);

		if (cmd_is_valid(cli_p, tmp_line)){
			token = strtok(tmp_line, " ");

			if (token){
				cmd = find_cmd(cli_p, token);
				if (!cmd) {
					dump_usage(cli_p);
				} else {
					char	*tokens[CLI_MAX_LINE_LENGTH];
					int	 tokens_cnt = 0;

					while ((tokens_cnt<CLI_MAX_LINE_LENGTH) && token)
					{
						tokens[tokens_cnt++] = token;
						token = strtok(NULL, " ");
					}
					if (cmd->do_cmd_cb(cmd->cmd_arg, tokens_cnt, tokens) != 0)
						cli_p->print_cb("Illegal command usage!\n");
				}
			}
		}
		memset(cli_p->line,0,(uint32_t)(CLI_MAX_LINE_LENGTH+5));
		cli_p->print_cb("%s> ", cli_p->prompt);
	}

	if (cli_p->running && cli_p->timer)
		mod_timer(cli_p->timer, CLI_TIMER_INTERVAL);
}

static void init_builtin_cmds(cli_t *cli_p)
{
	struct cli_cmd_params	 cmd_params;

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "help";
	cmd_params.desc		= "Help; print command description/usage";
	cmd_params.format	= NULL;
	cmd_params.cmd_arg	= cli_p;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))help_cmd_cb;
	cli_register_cmd(cli_p, &cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "?";
	cmd_params.desc		= "Alias for help";
	cmd_params.format	= NULL;
	cmd_params.cmd_arg	= cli_p;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))help_cmd_cb;
	cli_register_cmd(cli_p, &cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "q";
	cmd_params.desc		= "Quit; TODO";
	cmd_params.format	= NULL;
	cmd_params.cmd_arg	= cli_p;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))quit_cmd_cb;
	cli_register_cmd(cli_p, &cmd_params);
}


struct cli * cli_init(struct cli_params *cli_params)
{
	cli_t	*cli_p;
	char	 dflt_promt[] = "MUSDK";
	int	 i;

	cli_p = malloc(sizeof(cli_t));
	if (!cli_p) {
		pr_err("No mem for CLI obj!\n");
		return NULL;
	}
	memset(cli_p, 0, sizeof(cli_t));

	INIT_LIST(&cli_p->cmds_lst);
	INIT_LIST(&cli_p->hist_lst);

	if (cli_params->prompt == NULL)
		cli_params->prompt = dflt_promt;

	cli_p->prompt = (char *)malloc(strlen(cli_params->prompt)+1);
	if (!cli_p->prompt) {
		cli_free(cli_p);
		pr_err("No mem for CLI-prompt obj\n");
		return NULL;
	}
	memset(cli_p->prompt, 0, strlen(cli_params->prompt)+1);
	cli_p->prompt[strlen(cli_params->prompt)] = '\0';

	memcpy(cli_p->prompt, cli_params->prompt, strlen(cli_params->prompt));
	cli_p->no_block = cli_params->no_block;
	cli_p->echo = cli_params->echo;
	cli_p->print_cb	= cli_params->print_cb;
	cli_p->get_char_cb  = cli_params->get_char_cb;

	if (pthread_mutex_init(&cli_p->lock, NULL) != 0) {
		cli_free(cli_p);
		pr_err("init lock failed!\n");
		return NULL;
	}

	init_builtin_cmds(cli_p);

	cli_p->hist_lst_pos = &cli_p->hist_lst;
	for (i=0; i<CLI_MAX_HISTORY; i++) {
		cli_cmd_hist_t *hist_line = (cli_cmd_hist_t *)malloc(sizeof(cli_cmd_hist_t));
		if (!hist_line) {
			cli_free(cli_p);
			pr_err("No mem for CLI-lines-hist_lst obj\n");
			return NULL;
		}
		memset(hist_line, 0, sizeof(cli_cmd_hist_t));
		INIT_LIST(&hist_line->node);
		list_add_to_tail(&hist_line->node, &cli_p->hist_lst);
	}

	return cli_p;
}

int cli_free(struct cli *cli)
{
	cli_t		*cli_p = (cli_t *)cli;
	cli_cmd_t	*cmd;
	cli_cmd_hist_t	*hist_line;

	if (!cli_p) {
		pr_err("Invalid CLI obj provided!\n");
		return -EINVAL;
	}

	cli_stop(cli_p);

	pthread_mutex_lock(&cli_p->lock);
	while (!list_is_empty(&cli_p->cmds_lst)) {
		cmd = CLI_COMMAND_OBJ(cli_p->cmds_lst.next);
		list_del_init(&cmd->node);
		free(cmd);
	}
	pthread_mutex_unlock(&cli_p->lock);

	while (!list_is_empty(&cli_p->hist_lst)) {
		hist_line = CLI_HISTORY_LINE_OBJ(cli_p->hist_lst.next);
		list_del_init(&hist_line->node);
		free(hist_line);
	}

	if (cli_p->prompt)
		free(cli_p->prompt);

	free(cli_p);

	return 0;
}

int cli_run(struct cli *cli)
{
	cli_t	*cli_p = (cli_t *)cli;

	if (!cli_p) {
		pr_err("Invalid CLI obj provided!\n");
		return -EINVAL;
	}

	if (cli_p->no_block) {
		cli_p->timer = create_timer();
		if (!cli_p->timer) {
			pr_err("Failed to init CLI timer!\n");
			return -EFAULT;
		}
	}

	print_logo(cli_p);

	dump_usage(cli_p);

	cli_p->print_cb("%s> ", cli_p->prompt);

	cli_p->running = 1;
	if (cli_p->timer)
		start_timer(cli_p->timer, CLI_TIMER_INTERVAL, 0, (void (*)(void *))main_loop, cli_p);
	else
		while (cli_p->running)
			main_loop(cli_p);

	return 0;
}

int cli_stop(struct cli *cli)
{
	cli_t	*cli_p = (cli_t *)cli;

	if (!cli_p) {
		pr_err("Invalid CLI obj provided!\n");
		return -EINVAL;
	}

	cli_p->running = 0;
	if (cli_p->timer) {
		stop_timer(cli_p->timer);
		free_timer(cli_p->timer);
		cli_p->timer = NULL;
	}

	return 0;
}

int cli_register_cmd(struct cli *cli, struct cli_cmd_params *cmd_params)
{
	cli_t		*cli_p = (cli_t *)cli;
	cli_cmd_t	*cmd;

	if (!cli_p) {
		pr_err("Invalid CLI obj provided!\n");
		return -EINVAL;
	}

	cmd = malloc(sizeof(cli_cmd_t));
	if (!cmd) {
		pr_err("No mem for CLI-command obj\n");
		return -ENOMEM;
	}
	memset(cmd, 0, sizeof(cli_cmd_t));

	cmd->name = malloc(strlen(cmd_params->name)+1);
	if (!cmd->name) {
		pr_err("No mem for CLI-command-name obj\n");
		free(cmd);
		return -ENOMEM;
	}
	memcpy(cmd->name, cmd_params->name, strlen(cmd_params->name));
	cmd->name[strlen(cmd_params->name)] = '\0';

	cmd->desc = malloc(strlen(cmd_params->desc)+1);
	if (!cmd->desc) {
		pr_err("No mem for CLI-command-desc obj\n");
		free(cmd->name);
		free(cmd);
		return -ENOMEM;
	}
	memcpy(cmd->desc, cmd_params->desc, strlen(cmd_params->desc));
	cmd->desc[strlen(cmd_params->desc)] = '\0';

	if (cmd_params->format) {
		cmd->format = malloc(strlen(cmd_params->format)+1);
		if (!cmd->format) {
			pr_err("No mem for CLI-command-format obj\n");
			free(cmd->desc);
			free(cmd->name);
			free(cmd);
			return -ENOMEM;
		}
		memcpy(cmd->format, cmd_params->format, strlen(cmd_params->format));
		cmd->format[strlen(cmd_params->format)] = '\0';
	}

	INIT_LIST(&cmd->node);
	cmd->cmd_arg	= cmd_params->cmd_arg;
	cmd->do_cmd_cb	= cmd_params->do_cmd_cb;

	pthread_mutex_lock(&cli_p->lock);
	list_add_to_tail(&cmd->node, &cli_p->cmds_lst);
	pthread_mutex_unlock(&cli_p->lock);

	return 0;
}

int cli_unregister_cmd(struct cli *cli, char *name)
{
	cli_t		*cli_p = (cli_t *)cli;
	cli_cmd_t	*cmd;

	if (!cli_p) {
		pr_err("Invalid CLI obj provided!\n");
		return -EINVAL;
	}

	cmd = find_cmd(cli_p, name);

	if (cmd) {
		pthread_mutex_lock(&cli_p->lock);
		list_del(&(cmd->node));
		pthread_mutex_unlock(&cli_p->lock);

		if (cmd->format)
			free(cmd->format);
		if (cmd->desc)
			free(cmd->desc);
		if (cmd->name)
			free(cmd->name);
		free(cmd);
	}

	return 0;
}
