/******************************************************************************
 *	Copyright (C) 2016 Marvell International Ltd.
 *
 *  If you received this File from Marvell, you may opt to use, redistribute
 *  and/or modify this File under the following licensing terms.
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *	* Redistributions of source code must retain the above copyright
 *	  notice, this list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the distribution.
 *
 *	* Neither the name of Marvell nor the names of its contributors may be
 *	  used to endorse or promote products derived from this software
 *	  without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#ifndef __MV_NMP_GUEST_UTILS_H__
#define __MV_NMP_GUEST_UTILS_H__

#include "mng/mv_nmp.h"
#include "utils.h"
#include "pp2_utils.h"

struct pp2_ppio_bpool_info {
	char	bpool_name[20];
};

struct pp2_ppio_info {
	char				 ppio_name[20];
	u32				 num_bpools;
	struct pp2_ppio_bpool_info	*bpool_info;
};


struct pp2_info {
	u32			 num_ports;
	struct pp2_ppio_info	 *port_info;
};

/*
 * nmp_guest mode related functions.
 */
int nmp_read_cfg_file(struct nmp_params *params);
int guest_util_get_relations_info(char *buff, struct pp2_info *pp2_info);
int app_guest_utils_build_all_bpools(char *buff, struct pp2_info *pp2_info,
				     struct bpool_desc ***ppools,
				     struct pp2_glb_common_args *pp2_args,
				     struct bpool_inf infs[]);
int app_nmp_guest_port_init(char *buff, struct pp2_info *pp2_info, struct port_desc *port);


#endif /*__MV_NMP_GUEST_UTILS_H__*/

