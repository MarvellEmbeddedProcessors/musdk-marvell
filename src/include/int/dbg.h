/**************************************************************************//**
	Copyright (C) 2016 Marvell International Ltd.
*//***************************************************************************/

#ifndef __DBG_H__
#define __DBG_H__

extern int printf(const char *format, ...);

#ifndef pr_fmt
#define pr_fmt(fmt) fmt
#endif

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

#endif /* __DBG_H__ */
