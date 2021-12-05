/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __MV_COMPILER_H__
#define __MV_COMPILER_H__

#define __maybe_unused	__attribute__((unused))
#define __always_unused	__attribute__((unused))

#ifndef likely
#define likely(x)	__builtin_expect(!!(x), 1)
#endif
#ifndef unlikely
#define unlikely(x)	__builtin_expect(!!(x), 0)
#endif

#define __packed	__attribute__((__packed__))
#define __user

#define ____cacheline_aligned __attribute__((aligned(L1_CACHE_BYTES)))

#ifndef container_of
#define container_of(p, t, f) (t *)((void *)p - offsetof(t, f))
#endif

#define __stringify_1(x) #x
#define __stringify(x)	__stringify_1(x)

#endif /* __MV_COMPILER_H__ */
