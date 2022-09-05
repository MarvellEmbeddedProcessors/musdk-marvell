/************************************************************************
*  Copyright (c) 2018 Marvell.
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

#ifndef _MV_DPAPI_MAIN_H_
#define _MV_DPAPI_MAIN_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "mv_dp_types.h"
#include "mv_nss_dp.h"

extern struct mv_dp_main mv_dp_instance;

enum mv_dp_rc mv_dp_init_main(unsigned short rx_buff_size, mv_nss_dp_evt_handler evt, struct net_device *netdev);
void mv_dp_exit_main(void);

enum mv_dp_rc mv_dp_msg_tx(struct mv_dp_msg_info_tx *tx_msg);
void mv_dp_msg_rx(void *arg, u8 rx_opcode, u64 cookie, void *msg, u16 len);
enum mv_dp_rc mv_dp_event_info_init(struct mv_dp_event_info *events, mv_nss_dp_evt_handler evt);
enum mv_dp_rc mv_dp_rx_buffer_remove(int cpu_id, u16 sn);

enum mv_dp_rc mv_dp_online_set(const mv_nss_dp_init_cfg_t *cfg);
enum mv_dp_rc mv_dp_offline_set(void);

void mv_dp_show_msg_rx(struct mv_dp_msg_info_rx const *msg);
void mv_dp_show_msg_tx(struct mv_dp_msg_info_tx const *msg);
void mv_dp_show_msg_cb(mv_nss_dp_event_t const *evt, const unsigned p_size);
void mv_dp_show_ch_map(void);
void mv_dp_show_msg_buf(struct mv_dp_msg_buf const *msg, int header);
void mv_dp_show_msg_info(void);
void mv_dp_show_ch_rx_buffer(struct mv_dp_ch_data const *ch);
void mv_dp_show_ch_data(struct mv_dp_ch_data const *ch);
void mv_dp_show_error_counters(void);
void mv_dp_show_cfg(void);
void mv_dp_show_ch_counters(struct mv_dp_ch_data const *ch);
void mv_dp_clear_all_counters(void);
void mv_dp_show_event_info(void);

const struct mv_dp_ch_data *const mv_dp_get_ch_data(int i);
const struct mv_dp_msg_buf *const mv_dp_get_rx_buff(int i);

void mv_dp_dbg_set(u8 lvl);
enum mv_dp_rc  mv_dp_trace_set(int msg_id, u8 flag);
void mv_dp_trace_set_all(u8 type, u8 mask);


void mv_dp_timer_setup(struct mv_dp_ch_data *ch);
void mv_dp_timer_set(struct mv_dp_ch_data *ch);
void mv_dp_timer_delete(struct mv_dp_ch_data *ch);
void mv_dp_timer_set_interval(struct mv_dp_ch_data *ch, int interval);


enum mv_dp_rc mv_dp_msg_build(struct mv_dp_msg_info_tx *tx_msg, enum mv_dp_msg_id opcode, const void *ptr_in);
enum mv_dp_rc mv_dp_msg_build_out(struct mv_dp_msg_info_tx *tx_msg, enum mv_dp_msg_id opcode, void *ptr_in);
enum mv_dp_rc mv_dp_msg_build_bulk(struct mv_dp_msg_info_tx *tx_msg, enum mv_dp_msg_id opcode, void *bulk);

void mv_dp_inc_error(int num);
u8 mv_dp_dbg_lvl(void);
u8 mv_dp_msg_evt_type(int num);
u8 mv_dp_msg_trace_type(int num);
const char *mv_dp_err_name_get(int num);

u32 _mv_dp_virt_to_phys(void *addr);
void *_mv_dp_phys_to_virt(phys_addr_t addr);

#ifdef __cplusplus
}
#endif


#endif
