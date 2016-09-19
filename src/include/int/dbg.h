/**************************************************************************//**
	Copyright (C) 2016 Marvell International Ltd.
*//***************************************************************************/

#ifndef __DBG_H__
#define __DBG_H__

#include <stdio.h>
#include <stdlib.h>

#ifndef pr_fmt
#define pr_fmt(fmt) fmt
#endif

#define pr_crit(fmt, ...) \
	printf("[CRITICAL] " pr_fmt(fmt), ##__VA_ARGS__)
#define pr_err(fmt, ...) \
	printf("[ERROR] " pr_fmt(fmt), ##__VA_ARGS__)
#define pr_warning(fmt, ...) \
	printf("[WARN]  " pr_fmt(fmt), ##__VA_ARGS__)
#define pr_info(fmt, ...) \
	printf(pr_fmt(fmt), ##__VA_ARGS__)
#ifdef DEBUG
#define pr_debug(fmt, ...) \
	printf("[DBG]   " pr_fmt(fmt), ##__VA_ARGS__)
#else
#define pr_debug(fmt, ...)
#endif /* DEBUG */

#define pr_line	\
	printf("%s: %d\n",__FUNCTION__,__LINE__);

#define BUG	abort
#define BUG_ON(_cond)						\
	do {									\
		if (_cond) {						\
			pr_crit("[%s:%d] found BUG!\n",	\
					__FILE__, __LINE__); 	\
			BUG(); 							\
	} 										\
} while(0)

#endif /* __DBG_H__ */
