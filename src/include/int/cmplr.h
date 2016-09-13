/**************************************************************************//**
	Copyright (C) 2016 Marvell International Ltd.
*//***************************************************************************/

#ifndef __CMPLR_H__
#define __CMPLR_H__

#define __maybe_unused	__attribute__((unused))
#define __always_unused	__attribute__((unused))

#define likely(x)	__builtin_expect(!!(x), 1)
#define unlikely(x)	__builtin_expect(!!(x), 0)

#define __packed	__attribute__((__packed__))
#define __user

#define ____cacheline_aligned __attribute__((aligned(L1_CACHE_BYTES)))

#define container_of(p, t, f) (t *)((void *)p - offsetof(t, f))

#define __stringify_1(x) #x
#define __stringify(x)	__stringify_1(x)

#endif /* __CMPLR_H__ */
