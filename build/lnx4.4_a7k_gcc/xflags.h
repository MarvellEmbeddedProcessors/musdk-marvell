#ifndef __XFLAGS_H__
#define __XFLAGS_H__

/* TODO: move the following to a per-soc conf file */
#define PP2_SOC_NUM_PACKPROCS		1

#define MVCONF_DBG_LEVEL			5

#define CONF_PP2_BPOOL_COOKIE_SIZE	32
/** Note: in 64bits systems and user would like to use only 32bits,
 * use CONF_PP2_BPOOL_DMA_ADDR_USE_32B */
#define MVCONF_ARCH_DMA_ADDR_T_64BIT
#define CONF_PP2_BPOOL_DMA_ADDR_USE_32B

#define MVCONF_SYS_DMA_UIO

#endif /* __XFLAGS_H__ */
