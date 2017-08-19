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

#ifndef __MV_DEBUG_H__
#define __MV_DEBUG_H__

#include <stdio.h>
#include <stdlib.h>
#ifdef MVCONF_SYSLOG
#include <syslog.h>
#endif

#ifndef log_fmt
#define log_fmt(fmt) fmt
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
#define mv_print(_level, fmt, ...)			\
do {							\
	if ((_level) <= (MVCONF_DBG_LEVEL))		\
		printf(log_fmt(fmt), ##__VA_ARGS__);	\
} while (0)
#else /* MVCONF_SYSLOG */

#define mv_print(_level, fmt, ...)		\
	syslog(_level, log_fmt(fmt), ##__VA_ARGS__)

void log_init(int log_to_stderr);
void log_close(void);

#endif /* MVCONF_SYSLOG */

#ifndef pr_emerg
#define pr_emerg(...) \
	mv_print(MV_DBG_L_EMERG, __VA_ARGS__)
#endif /* !pr_emerg */
#ifndef pr_alert
#define pr_alert(...) \
	mv_print(MV_DBG_L_ALERT, __VA_ARGS__)
#endif /* !pr_alert */
#ifndef pr_crit
#define pr_crit(...) \
	mv_print(MV_DBG_L_CRIT, "[CRITICAL] " __VA_ARGS__)
#endif /* !pr_crit */
#ifndef pr_err
#define pr_err(...) \
	mv_print(MV_DBG_L_ERR, "[ERROR] " __VA_ARGS__)
#endif /* !pr_err */
#ifndef pr_warn
#define pr_warn(...) \
	mv_print(MV_DBG_L_WARN, "[WARN] " __VA_ARGS__)
#endif /* !pr_warn */
#ifndef pr_notice
#define pr_notice(...) \
	mv_print(MV_DBG_L_NOTICE, __VA_ARGS__)
#endif /* !pr_notice */
#ifndef pr_info
#define pr_info(...) \
	mv_print(MV_DBG_L_INFO, __VA_ARGS__)
#endif /* !pr_info */
#ifndef pr_debug
#ifdef DEBUG
#define pr_debug(...) \
	mv_print(MV_DBG_L_DBG, "[DBG] " __VA_ARGS__)
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
