/* Copyright (c) 2016, Linaro Limited
 * All rights reserved.
 *
 * SPDX-License-Identifier:     BSD-3-Clause
 */

/**
 * @file
 *
 * pp2 byteorder
 */

#ifndef _BYTEORDER_INLINES_H_
#define _BYTEORDER_INLINES_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "mv_std.h"
inline u16 pp2_le_to_cpu_16(u16 le16)
{
#if __BYTE_ORDER == __LITTLE_ENDIAN
	return (u16)le16;
#else
	return __builtin_bswap16((u16)le16);
#endif
}

inline u32 pp2_le_to_cpu_32(u32 le32)
{
#if __BYTE_ORDER == __LITTLE_ENDIAN
	return (u32)le32;
#else
	return __builtin_bswap32((u32)le32);
#endif
}

inline u16 pp2_cpu_to_be_16(u16 cpu16)
{
#if __BYTE_ORDER == __LITTLE_ENDIAN
	return (u16)__builtin_bswap16(cpu16);
#else
	return (u16)cpu16;
#endif
}

inline u16 pp2_be_to_cpu_16(u16 be16)
{
#if __BYTE_ORDER == __LITTLE_ENDIAN
	return __builtin_bswap16((u16)be16);
#else
	return (u16)be16;
#endif
}

inline u32 pp2_cpu_to_be_32(u32 cpu32)
{
#if __BYTE_ORDER == __LITTLE_ENDIAN
	return (u32)__builtin_bswap32(cpu32);
#else
	return (u32)cpu32;
#endif
}

inline u64 pp2_cpu_to_be_64(u64 cpu64)
{
#if __BYTE_ORDER == __LITTLE_ENDIAN
	return (u64)__builtin_bswap64(cpu64);
#else
	return (u64)cpu64;
#endif
}

inline u32 pp2_be_to_cpu_32(u32 be32)
{
#if __BYTE_ORDER == __LITTLE_ENDIAN
	return __builtin_bswap32((u32)be32);
#else
	return (u32)be32;
#endif
}

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif
