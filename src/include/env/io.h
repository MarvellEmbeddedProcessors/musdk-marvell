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
