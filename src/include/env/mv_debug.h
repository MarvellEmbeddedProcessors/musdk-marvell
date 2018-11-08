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
