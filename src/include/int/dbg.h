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

#ifndef __DBG_H__
#define __DBG_H__

#include <stdio.h>
#include <stdlib.h>

#define MV_DBG_L_CRIT	1
#define MV_DBG_L_ERR	2
#define MV_DBG_L_WARN	3
#define MV_DBG_L_INFO	4
#define MV_DBG_L_DBG	5

#define mv_print(_level, ...) 			\
do {						\
	if ((_level) <= (MVCONF_DBG_LEVEL))	\
		printf(__VA_ARGS__);		\
} while (0)

#ifndef pr_crit
#define pr_crit(...) \
	mv_print(MV_DBG_L_CRIT, "[CRITICAL] " __VA_ARGS__)
#endif /* !pr_crit */
#ifndef pr_err
#define pr_err(...) \
	mv_print(MV_DBG_L_CRIT, "[ERROR] " __VA_ARGS__)
#endif /* !pr_err */
#ifndef pr_warning
#define pr_warning(...) \
	mv_print(MV_DBG_L_CRIT, "[WARN] " __VA_ARGS__)
#endif /* !pr_warning */
#ifndef pr_info
#define pr_info(...) \
	mv_print(MV_DBG_L_CRIT, __VA_ARGS__)
#endif /* !pr_info */
#ifndef pr_debug
#ifdef DEBUG
#define pr_debug(...) \
	mv_print(MV_DBG_L_CRIT, "[DBG] " __VA_ARGS__)
#else
#define pr_debug(...)
#endif /* !pr_debug */
#endif /* DEBUG */

#ifndef pr_line
#define pr_line	\
	printf("%s: %d\n",__FUNCTION__,__LINE__);
#endif /* !pr_line */

#ifndef BUG
#define BUG	abort
#endif /* !BUG */

#ifndef BUG_ON
#define BUG_ON(_cond)					\
	do {						\
		if (_cond) {				\
			pr_crit("[%s:%d] found BUG!\n",	\
				__FILE__, __LINE__); 	\
			BUG(); 				\
	} 						\
} while(0)
#endif /* !BUG_ON */

#endif /* __DBG_H__ */
