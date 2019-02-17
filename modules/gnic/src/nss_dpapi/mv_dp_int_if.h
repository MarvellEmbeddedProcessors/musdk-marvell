/************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is free software: you can redistribute it and/or
*  modify it under the terms of the GNU General Public License as
*  published by the Free Software Foundation, either version 2 of the
*  License, or any later version.
*
*  This program is distributed in the hope that it will be useful, but
*  WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
*  General Public License for more details.
*
*******************************************************************************/

#ifndef _MV_DP_INT_IF_H_
#define _MV_DP_INT_IF_H_

#include "mv_dp_defs.h"
#include "mv_dp_types.h"
#include "mv_nss_dp.h"

#ifdef __cplusplus
extern "C" {
#endif


struct mv_dp_dbg_nss_mem {
	u8	type;
	u8	size;
	u16	filler;
	u32	offset;
	u8	arr[MV_DP_MSG_NSS_MEM_DUMP_SIZE_MAX_B];
};

struct mv_dp_ctx {
	int			cnt;
	enum mv_dp_msg_id 	opc;
	void			*data;
	int			size;
};


enum mv_dp_rc mv_dp_msg_nop_tx(u32 cookie, mv_nss_dp_result_spec_t *res);
enum mv_dp_rc mv_dp_msg_ctx_tx(struct mv_dp_ctx *ctx, const mv_nss_dp_result_spec_t *res);

enum mv_dp_rc mv_dp_msg_ver_tx(mv_nss_dp_result_spec_t *res);

enum mv_dp_rc mv_dp_msg_event_sys_msg_parse(void *msg_in, void *buf_out);
enum mv_dp_rc mv_dp_msg_event_sys_parse(void *msg_in, void *buf_out);

mv_nss_dp_status_t mv_nss_dp_client_get_mac(const mv_nss_dp_l2_addr_t *l2_addr,
					    const mv_nss_dp_result_spec_t *res);
mv_nss_dp_status_t mv_nss_dp_hash_profile_get(mv_nss_dp_hash_profile_id_t profile_id,
					      const mv_nss_dp_result_spec_t *res);

enum mv_dp_rc mv_dp_msg_nss_mem_write_tx(struct mv_dp_dbg_nss_mem *mem, mv_nss_dp_result_spec_t *res);
enum mv_dp_rc mv_dp_msg_nss_mem_read_tx(struct mv_dp_dbg_nss_mem *mem, mv_nss_dp_result_spec_t *res);

/*no specific out parsing*/
enum mv_dp_rc mv_dp_msg_def_msg_rx(struct mv_dp_msg_info_rx *msg_rx, void *p);
void mv_dp_default_rx_cb(mv_nss_dp_event_t *evt);


/*utilities*/
enum mv_dp_rc mv_dp_check_internal_prio(const u8 *prio, int size, int max_prio);
enum mv_dp_rc mv_dp_check_pkt_prio(const u8 *prio, int size, u8 mask);
void mv_dp_nss_mem_show(const struct mv_dp_dbg_nss_mem *const mem);


#define MV_DP_MSG_DIM_TYPE_IS_OK(v)	((v) >= MV_NSS_DP_VIRT_PORT && (v) <= MV_NSS_DP_L4_FLOW)



#ifdef __cplusplus
}
#endif


#endif
