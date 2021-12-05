/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __MV_DEBUG_H__
#define __MV_DEBUG_H__

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#ifdef MVCONF_SYSLOG
#include <syslog.h>
#endif

#ifndef log_fmt
#define log_fmt(fmt, ...) fmt, ##__VA_ARGS__
#endif

#define printk printf

#define MV_DBG_L_EMERG	0
#define MV_DBG_L_ALERT	1
#define MV_DBG_L_CRIT	2
#define MV_DBG_L_ERR	3
#define MV_DBG_L_WARN	4
#define MV_DBG_L_NOTICE	5
#define MV_DBG_L_INFO	6
#define MV_DBG_L_DBG	7

#ifndef MVCONF_SYSLOG
#define mv_print(_level, fmt, ...)					\
do {									\
	if ((_level) <= (MVCONF_DBG_LEVEL))	{			\
		struct timespec spec;					\
		clock_gettime(CLOCK_BOOTTIME, &spec);			\
		printf("[%5lu.%06lu] ", spec.tv_sec, spec.tv_nsec/1000);\
		printf(log_fmt(fmt, ##__VA_ARGS__));			\
	}								\
} while (0)
#else /* MVCONF_SYSLOG */

#define mv_print(_level, fmt, ...)		\
	syslog(_level, log_fmt(fmt, ##__VA_ARGS__))

void log_init(int log_to_stderr);
void log_close(void);

#endif /* MVCONF_SYSLOG */

#ifndef pr_emerg
#define pr_emerg(fmt, ...) \
	mv_print(MV_DBG_L_EMERG, fmt, ##__VA_ARGS__)
#endif /* !pr_emerg */
#ifndef pr_alert
#define pr_alert(fmt, ...) \
	mv_print(MV_DBG_L_ALERT, fmt, ##__VA_ARGS__)
#endif /* !pr_alert */
#ifndef pr_crit
#define pr_crit(fmt, ...) \
	mv_print(MV_DBG_L_CRIT, "[CRITICAL] " fmt, ##__VA_ARGS__)
#endif /* !pr_crit */
#ifndef pr_err
#define pr_err(fmt, ...) \
	mv_print(MV_DBG_L_ERR, "[ERROR] " fmt, ##__VA_ARGS__)
#endif /* !pr_err */
#ifndef pr_warn
#define pr_warn(fmt, ...) \
	mv_print(MV_DBG_L_WARN, "[WARN] " fmt, ##__VA_ARGS__)
#endif /* !pr_warn */
#ifndef pr_notice
#define pr_notice(fmt, ...) \
	mv_print(MV_DBG_L_NOTICE, fmt, ##__VA_ARGS__)
#endif /* !pr_notice */
#ifndef pr_info
#define pr_info(fmt, ...) \
	mv_print(MV_DBG_L_INFO, fmt, ##__VA_ARGS__)
#endif /* !pr_info */
#ifndef pr_debug
#ifdef DEBUG
#define pr_debug(fmt, ...) \
	mv_print(MV_DBG_L_DBG, "[DBG] " fmt, ##__VA_ARGS__)
#else
#define pr_debug(...)
#endif /* DEBUG */
#endif /* !pr_debug */

#ifndef pr_line
#define pr_line	\
	printf("%s: %d\n", __func__, __LINE__)
#endif /* !pr_line */

#ifndef BUG
#define BUG	abort
#endif /* !BUG */

#ifndef BUG_ON
#define BUG_ON(_cond)					\
	do {						\
		if (_cond) {				\
			pr_crit("[%s:%d] found BUG!\n",	\
				__FILE__, __LINE__);	\
			BUG();				\
	}						\
} while (0)
#endif /* !BUG_ON */


#ifndef pr_debug_fmt
#define pr_debug_fmt(fmt, ...) \
	pr_debug("%24s : %3d : " fmt "\n", \
		 __func__, __LINE__, ##__VA_ARGS__)
#endif


#endif /* __MV_DEBUG_H__ */
