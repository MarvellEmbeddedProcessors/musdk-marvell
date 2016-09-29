/******************************************************************************
	Copyright (C) 2016 Marvell International Ltd.

  If you received this File from Marvell, you may opt to use, redistribute
  and/or modify this File under the following licensing terms.
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
  
  	* Redistributions of source code must retain the above copyright
	  notice, this list of conditions and the following disclaimer.
  
  	* Redistributions in binary form must reproduce the above copyright
  	  notice, this list of conditions and the following disclaimer in the
  	  documentation and/or other materials provided with the distribution.
  
  	* Neither the name of Marvell nor the names of its contributors may be
  	  used to endorse or promote products derived from this software
	  without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#ifndef __MV_SAM_CIO_H__
#define __MV_SAM_CIO_H__

#include "std.h"


struct sam_cio;

struct sam_cioparams {
	int		 id;
/* TODO: complete */
};


struct sam_cio * sam_cio_init(struct sam_cioparams *params);


#define SAM_CIO_MAX_FRAGS 20


struct sam_buf_info {
	void		*buf;
	unsigned int	 len;
};

struct sam_cio_enq_desc {
	int			 num_bufs;
	struct sam_buf_info	 bufs[SAM_CIO_MAX_FRAGS];
	int			 total_len;
	int			 crp_len;
	int			 crp_offset;
	int			 iv_fromuser; /*flag*/
	int			 iv_offset;
	int			 mac_len;
	int			 mac_offset;  /* in encrypt direction: staring calculation. usually   */
		/* will be 20 (right after the IP header of the tunnel) */
	char			*iv;
	int			 iv_len;
};

struct sam_cio_deq_desc {
	int			 num_bufs;
	struct sam_buf_info	 bufs[SAM_CIO_MAX_FRAGS];
	int			 total_len;
	u32			 status;
	/* TODO: complete. */
};


int sam_cio_enq(struct sam_cio *cio, struct sam_cio_enq_desc *desc);
int sam_cio_deq(struct sam_cio *cio, struct sam_cio_deq_desc *desc);

int sam_cio_enable(struct sam_cio *cio);
int sam_cio_disable(struct sam_cio *cio);

#endif /* __MV_SAM_CIO_H__ */
