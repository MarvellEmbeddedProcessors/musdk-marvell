/**************************************************************************//**
	Copyright (C) 2016 Marvell International Ltd.
*//***************************************************************************/

#ifndef __IO_H__
#define __IO_H__

#define __iomem

/* TODO: add wmb, rmb, mb */
/* TODO: add writel, readl, etc. */

#define local_irq_disable()	do{;} while(0)
#define local_irq_enable()	do{;} while(0)
#define local_irq_save(_flags)	do{_flags=0;} while(0)
#define local_irq_restore(_flags)	do{_flags=_flags;} while(0)

#endif /* __IO_H__ */
