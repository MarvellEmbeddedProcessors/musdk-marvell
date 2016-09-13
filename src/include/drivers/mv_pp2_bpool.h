/**************************************************************************//**
	Copyright (C) 2016 Marvell International Ltd.
*//***************************************************************************/

#ifndef __MV_PP2_BPOOL_H__
#define __MV_PP2_BPOOL_H__

#include "std.h"

#include "mv_pp2_hif.h"


struct pp2_bpool;


enum pp2_bpool_cookie_type {
	PP2_BPOOL_COOKIE_T_32B = 0,
	PP2_BPOOL_COOKIE_T_64B
};


struct pp2_bpool_params {
	char						 *match;

	u32							 max_num_buffs;
	u32							 buff_len;
	enum pp2_bpool_cookie_type	 cookie_type;

	/* TODO: will not be supported at first stage. Need to look how to handle HW IRQ */
	int			(*empty_cb) (void *arg, u32 status);
	void		*emty_cb_arg;
	u16			threashold_hi;
	u16			threashold_lo;
};

int pp2_bpool_init(struct pp2_bpool_params *params, struct pp2_bpool **bpool);


struct pp2_buff_inf {
	dma_addr_t		addr;
	u64				cookie;
};

int pp2_bpool_get_buff(struct pp2_hif *hif, struct pp2_bpool *pool, struct pp2_buff_inf *buff);
int pp2_bpool_put_buff(struct pp2_hif *hif, struct pp2_bpool *pool, struct pp2_buff_inf *buff);

int pp2_bpool_get_num_buffs(struct pp2_bpool *pool, u32 *num_buffs);

#endif /* __MV_PP2_BPOOL_H__ */
