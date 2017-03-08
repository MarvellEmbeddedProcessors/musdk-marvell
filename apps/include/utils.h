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

#ifndef __MVUTILS_H__
#define __MVUTILS_H__

#include "mv_std.h"
#include "mv_pp2_hif.h"

#define MVAPPS_PKT_OFFS		64
#define MVAPPS_MH_SIZE		(2) /* TODO: take this from ppio definitions.*/
#define MVAPPS_PKT_EFEC_OFFS	(MVAPPS_PKT_OFFS + MVAPPS_MH_SIZE)

#define MVAPPS_Q_SIZE		1024
#define MVAPPS_MAX_BURST_SIZE	(MVAPPS_Q_SIZE >> 1)

#define MVAPPS_PP2_BPOOLS_RSRV	0x7
#define MVAPPS_PP2_HIFS_RSRV	0xF
#define MVAPPS_DFLT_BURST_SIZE	256
#define MVAPPS_MAX_NUM_PORTS 2

#define MVAPPS_MAX_NUM_CORES	4
#define MVAPPS_MAX_NUM_QS_PER_TC		MVAPPS_MAX_NUM_CORES
#define MVAPPS_MAX_NUM_QS_PER_CORE		MVAPPS_MAX_NUM_QS_PER_TC

/* TODO: find more generic way to get the following parameters */
#define MVAPPS_PP2_TOTAL_NUM_BPOOLS	16
#define MVAPPS_PP2_TOTAL_NUM_HIFS	9

#define MVAPPS_BPOOLS_INF	{{2048, 1024} }

struct bpool_inf {
	int	buff_size;
	int	num_buffs;
};

static inline void swap_l2(char *buf)
{
	uint16_t *eth_hdr;

	register uint16_t tmp;

	eth_hdr = (uint16_t *)buf;
	tmp = eth_hdr[0];
	eth_hdr[0] = eth_hdr[3];
	eth_hdr[3] = tmp;
	tmp = eth_hdr[1];
	eth_hdr[1] = eth_hdr[4];
	eth_hdr[4] = tmp;
	tmp = eth_hdr[2];
	eth_hdr[2] = eth_hdr[5];
	eth_hdr[5] = tmp;
}

static inline void swap_l3(char *buf)
{
	register uint32_t tmp32;

	buf += 14 + 12;
	tmp32 = ((uint32_t *)buf)[0];
	((uint32_t *)buf)[0] = ((uint32_t *)buf)[1];
	((uint32_t *)buf)[1] = tmp32;
}

u64 app_get_sys_dma_high_addr(void);
int app_hif_init(struct pp2_hif **hif);
int app_build_all_bpools(struct pp2_bpool ****ppools, struct pp2_buff_inf ****pbuffs_inf, int num_pools,
			   struct bpool_inf infs[], struct pp2_hif *hif);

#endif /*__MVUTILS_H__*/

