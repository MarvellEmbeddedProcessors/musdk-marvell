/*******************************************************************************
	Copyright (C) 2016 Marvell International Ltd.

	If you received this File from Marvell, you may opt to use, redistribute
	and/or modify this File under the following licensing terms.
	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are met:

		* Redistributions of source code must retain the above copyright notice,
		  this list of conditions and the following disclaimer.

		* Redistributions in binary form must reproduce the above copyright
		  notice, this list of conditions and the following disclaimer in the
		  documentation and/or other materials provided with the distribution.

		* Neither the name of Marvell nor the names of its contributors may be
		  used to endorse or promote products derived from this software without
		  specific prior written permission.

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
*******************************************************************************/

#ifndef __MV_PP2_H__
#define __MV_PP2_H__

#include "std.h"


#define PP2_NUM_PKT_PROC 2
#define PP2_NUM_ETH_PPIO 3


struct ppio_init_params {
	int		is_enabled; /**/
	u32		first_inq;
};

struct pp2_init_params {
	u16 hif_reserved_map; /* Bitmap of reserved HIF objects (0-8), that may not be used by MUSDK. bit0=hif0, etc. */
	u16 bm_pool_reserved_map; /* Bitmap of reserved bm_pools (0-15). The pools are reserved in all packet_processors. */
	u8  rss_tbl_reserved_map; /* Bitmap of RSS Tables (0-7). The tables are reserved in all packet_processors. */
	struct ppio_init_params ppio[PP2_NUM_PKT_PROC][PP2_NUM_ETH_PPIO];
};


int pp2_init(struct pp2_init_params *params);

#endif /* __MV_PP2_H__ */
