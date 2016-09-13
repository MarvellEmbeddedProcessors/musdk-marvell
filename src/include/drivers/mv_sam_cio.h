/**************************************************************************//**
	Copyright (C) 2016 Marvell International Ltd.
*//***************************************************************************/

#ifndef __MV_SAM_CIO_H__
#define __MV_SAM_CIO_H__

#include "std.h"

struct sam_cio;

struct sam_cioparams {
	int			id;
/* TODO: complete */
};

struct sam_cio * sam_cio_init(struct sam_cioparams *params);


#define SAM_CIO_MAX_FRAGS 20

struct sam_buf_info {
	void			*buf;
	unsigned int	 len;
};

struct sam_cio_enq_desc {
	int						 num_bufs;
	struct sam_buf_info	 bufs[SAM_CIO_MAX_FRAGS];
	int						 total_len;
	int						 crp_len;
	int						 crp_offset;
	int						 iv_fromuser; /*flag*/
	int						 iv_offset;
	int						 mac_len;
	int						 mac_offset;  /* in encrypt direction: staring calculation. usually   */
		/* will be 20 (right after the IP header of the tunnel) */
	char					*iv;
	int						 iv_len;
};

struct sam_cio_deq_desc {
	int						 num_bufs;
	struct sam_buf_info	 bufs[SAM_CIO_MAX_FRAGS];
	int						 total_len;
	u32						 status;
	/* TODO: complete. */
};

int sam_cio_enq(struct sam_cio *cio, struct sam_cio_enq_desc *desc);
int sam_cio_deq(struct sam_cio *cio, struct sam_cio_deq_desc *desc);

int sam_cio_enable(struct sam_cio *cio);
int sam_cio_disable(struct sam_cio *cio);

#endif /* __MV_SAM_CIO_H__ */
