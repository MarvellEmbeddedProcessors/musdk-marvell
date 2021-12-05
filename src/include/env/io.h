/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __IO_H__
#define __IO_H__

#include "mv_types.h"

#define local_irq_disable()	do {; } while (0)
#define local_irq_enable()	do {; } while (0)
#define local_irq_save(_flags)	do {_flags = 0; } while (0)
#define local_irq_restore(_flags)	do {_flags = _flags; } while (0)

#define __iomem

#if __WORDSIZE == 64
#define dsb(opt)	({ asm volatile("dsb " #opt : : : "memory"); })
#define mb()		dsb(sy)
#define rmb()		dsb(ld)
#define wmb()		dsb(st)
#define __iormb()	rmb()
#define __iowmb()	wmb()
#define smp_mb()	dmb(ish)
#define smp_rmb()	dmb(ishld)
#define smp_wmb()	dmb(ishst)
#define cpu_relax()    ({ asm volatile("yield" : : : "memory"); })
#else
#define dmb(opt)	({ asm volatile("dmb " #opt : : : "memory"); })
#define rmb()		dmb(sy)
#define wmb()		dmb(sy)
#define __iormb()	rmb()
#define __iowmb()	wmb()

#define barrier()	({ asm volatile("" : : : "memory"); })
#define cpu_relax()	barrier()

#endif

#define dccivac(_p)	({ __asm__ __volatile__("dc civac, %0\n\t" : : "r" (_p) : "memory"); })

/*
 * Generic IO read/write.  These perform native-endian accesses.
*/

#if __WORDSIZE == 64
static inline u8 __raw_mv_readb(const volatile void __iomem *addr)
{
	u8 val;

	asm volatile("ldrb %w0, [%1]" : "=r" (val) : "r" (addr));
	return val;
}

static inline u16 __raw_mv_readw(const volatile void __iomem *addr)
{
	u16 val;

	asm volatile("ldrh %w0, [%1]" : "=r" (val) : "r" (addr));
	return val;
}

static inline u32 __raw_mv_readl(const volatile void __iomem *addr)
{
	u32 val;

	asm volatile("ldr %w0, [%1]" : "=r" (val) : "r" (addr));
	return val;
}

static inline u64 __raw_mv_readq(const volatile void __iomem *addr)
{
	u64 val;

	asm volatile("ldr %0, [%1]" : "=r" (val) : "r" (addr));
	return val;
}

static inline void __raw_mv_writeb(u8 val, volatile void __iomem *addr)
{
	asm volatile("strb %w0, [%1]" : : "r" (val), "r" (addr));
}

static inline void __raw_mv_writew(u16 val, volatile void __iomem *addr)
{
	asm volatile("strh %w0, [%1]" : : "r" (val), "r" (addr));
}

static inline void __raw_mv_writel(u32 val, volatile void __iomem *addr)
{
	asm volatile("str %w0, [%1]" : : "r" (val), "r" (addr));
}

static inline void __raw_mv_writeq(u64 val, volatile void __iomem *addr)
{
	asm volatile("str %0, [%1]" : : "r" (val), "r" (addr));
}

#else
static inline u8 __raw_mv_readb(const volatile void __iomem *addr)
{
	u8 val;

	asm volatile("ldrb %0, %1"
		     : "=r" (val)
		     : "Qo" (*(volatile u8 *)addr));
	return val;
}

static inline u16 __raw_mv_readw(const volatile void __iomem *addr)
{
	u16 val;

	asm volatile("ldrh %0, %1"
		     : "=r" (val)
		     : "Q" (*(u16 *)addr));
	return val;
}

static inline u32 __raw_mv_readl(const volatile void __iomem *addr)
{
	u32 val;

	asm volatile("ldr %0, %1"
		     : "=r" (val)
		     : "Qo" (*(u32 *)addr));
	return val;
}

static inline void __raw_mv_writeb(u8 val, volatile void __iomem *addr)
{
	asm volatile("strb %1, %0"
		: : "Qo" (*(u8 *)addr), "r" (val));
}

static inline void __raw_mv_writew(u16 val, volatile void __iomem *addr)
{
	asm volatile("strh %1, %0"
		     : : "Q" (*(u16 *)addr), "r" (val));
}

static inline void __raw_mv_writel(u32 val, volatile void __iomem *addr)
{
	asm volatile("str %1, %0"
		     : : "Qo" (*(u32 *)addr), "r" (val));
}
#endif

/*
 * Relaxed I/O memory access primitives. These follow the Device memory
 * ordering rules but do not guarantee any ordering relative to Normal memory
 * accesses.
 */
#define mv_readb_relaxed(c)	({ u8  __r = __raw_mv_readb(c); __r; })
#define mv_readw_relaxed(c)	({ u16 __r = le16toh(__raw_mv_readw(c)); __r; })
#define mv_readl_relaxed(c)	({ u32 __r = le32toh(__raw_mv_readl(c)); __r; })
#define mv_readq_relaxed(c)	({ u64 __r = le64toh(__raw_mv_readq(c)); __r; })

#define mv_writeb_relaxed(v, c)	((void)__raw_mv_writeb((v), (c)))
#define mv_writew_relaxed(v, c)	((void)__raw_mv_writew(htole16(v), (c)))
#define mv_writel_relaxed(v, c)	((void)__raw_mv_writel(htole32(v), (c)))
#define mv_writeq_relaxed(v, c)	((void)__raw_mv_writeq(htole64(v), (c)))

#define readb_relaxed	mv_readb_relaxed
#define readw_relaxed	mv_readw_relaxed
#define readl_relaxed	mv_readl_relaxed
#define readq_relaxed	mv_readq_relaxed

#define writeb_relaxed	mv_writeb_relaxed
#define writew_relaxed	mv_writew_relaxed
#define writel_relaxed	mv_writel_relaxed
#define writeq_relaxed	mv_writeq_relaxed

/*
 * I/O memory access primitives. Reads are ordered relative to any
 * following Normal memory access. Writes are ordered relative to any prior
 * Normal memory access.
 */
#define readb(c)		({ u8  __v = mv_readb_relaxed(c); __iormb(); __v; })
#define readw(c)		({ u16 __v = mv_readw_relaxed(c); __iormb(); __v; })
#define readl(c)		({ u32 __v = mv_readl_relaxed(c); __iormb(); __v; })
#define readq(c)		({ u64 __v = mv_readq_relaxed(c); __iormb(); __v; })

#define writeb(v, c)		({ __iowmb(); mv_writeb_relaxed((v), (c)); })
#define writew(v, c)		({ __iowmb(); mv_writew_relaxed((v), (c)); })
#define writel(v, c)		({ __iowmb(); mv_writel_relaxed((v), (c)); })
#define writeq(v, c)		({ __iowmb(); mv_writeq_relaxed((v), (c)); })

#define flush_cache_line(_p)	dccivac((unsigned long long)_p)

#endif /* __IO_H__ */
