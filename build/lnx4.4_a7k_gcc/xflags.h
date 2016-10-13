#ifndef __XFLAGS_H__
#define __XFLAGS_H__

/* TODOL remove the following from here */
#define PP2_SOC_NUM_PACKPROCS		1

#define MV_DBG_LEVEL				5
#define CONF_PP2_BPOOL_COOKIE_SIZE	64
/** Note: in 64bits systems and user would like to use only 32bits,
 * use CONF_PP2_BPOOL_DMA_ADDR_USE_32B */
#define MVCONF_ARCH_DMA_ADDR_T_64BIT
#define MVCONF_SYS_DMA_UIO

#endif /* __XFLAGS_H__ */
