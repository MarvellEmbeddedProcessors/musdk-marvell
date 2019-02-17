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

#ifndef _MV_DP_MSG_H_
#define _MV_DP_MSG_H_

#include "mv_dp_defs.h"
#include "mv_dp_types.h"
#include "mv_nss_dp.h"

#ifdef __cplusplus
extern "C" {
#endif



enum mv_dp_rc mv_dp_nop_parse_msg(void *dest, void  *src);
enum mv_dp_rc mv_dp_ctx_parse_msg(void *nop_ptr, void *buf);
enum mv_dp_rc mv_dp_ctx_populate_msg(void *buf, const void *data, int index);
enum mv_dp_rc mv_dp_nop_populate_msg(void *buf, const void  *data, int index);

enum mv_dp_rc mv_dp_ver_parse_msg(void *dest, void  *src);

enum mv_dp_rc mv_dp_msg_def_event_rx(void *msg, void *p);
enum mv_dp_rc mv_dp_msg_event_msg_rx(void *msg, void *p);

enum mv_dp_rc mv_dp_client_parse_struct(void *client_ptr, void *buf);
enum mv_dp_rc mv_dp_client_populate_msg(void *buf, const void  *data, int index);
enum mv_dp_rc mv_dp_client_ind_populate_msg(void *buf, const void  *data, int index);
enum mv_dp_rc mv_dp_client_mac_populate_msg(void *buf, const void  *data, int index);

enum mv_dp_rc mv_dp_port_parse_struct(void *port_ptr, void *buf);
enum mv_dp_rc mv_dp_eport_parse_struct(void *port_ptr, void *buf);
enum mv_dp_rc mv_dp_lagport_parse_struct(void *port_ptr, void *buf);
enum mv_dp_rc mv_dp_cwport_parse_struct(void *port_ptr, void *buf);
enum mv_dp_rc mv_dp_cpuport_parse_struct(void *port_ptr, void *buf);
enum mv_dp_rc mv_dp_port_stats_parse_struct(void *port_stats_ptr, void *buf);
enum mv_dp_rc mv_dp_eport_populate_msg(void *buf, const void  *data, int index);
enum mv_dp_rc mv_dp_lagport_populate_msg(void *buf, const void  *data, int index);
enum mv_dp_rc mv_dp_cpuport_populate_msg(void *buf, const void  *data, int index);
enum mv_dp_rc mv_dp_cwport_populate_msg(void *buf, const void  *data, int index);
enum mv_dp_rc mv_dp_port_id_populate_msg(void *buf, const void  *data, int index);
enum mv_dp_rc mv_dp_port_link_parse_struct(void *port_ptr, void *buf);

enum mv_dp_rc mv_dp_vlan_parse_struct(void *vlan_ptr, void *buf);
enum mv_dp_rc mv_dp_vlan_populate_msg(void *buf, const void  *data, int index);
enum mv_dp_rc mv_dp_vlan_ind_populate_msg(void *buf, const void  *data, int index);
enum mv_dp_rc mv_dp_vlan_id_populate_msg(void *buf, const void  *data, int index);

enum mv_dp_rc mv_dp_dtls_parse_struct(void *dtls_ptr, void *buf);
enum mv_dp_rc mv_dp_dtls_populate_msg(void *buf, const void  *data, int index);
enum mv_dp_rc mv_dp_dtls_id_populate_msg(void *buf, const void  *data, int index);


enum mv_dp_rc mv_dp_flow_populate_param_msg(void *buf, const mv_nss_dp_params_t *const param);
enum mv_dp_rc mv_dp_flow_populate_msg(void *buf, const void  *data, int index);
enum mv_dp_rc mv_dp_flow_param_parse_struct(mv_nss_dp_params_t *param, void *buf);
enum mv_dp_rc mv_dp_flow_parse_struct(void *flow_ptr, void *buf);
enum mv_dp_rc mv_dp_flow_id_populate_msg(void *buf, const void  *data, int index);
enum mv_dp_rc mv_dp_flow_count_parse_struct(void *flow_ptr, void *buf);
enum mv_dp_rc mv_dp_flow_status_parse_struct(void *flow_status_ptr, void *buf);
enum mv_dp_rc mv_dp_flow_status_populate_msg(void *buf, const void  *data, int index);
enum mv_dp_rc mv_dp_flow_stats_parse_struct(void *flow_stats_ptr, void *buf);


enum mv_dp_rc mv_dp_ingress_prio_cfg_populate_msg(void *buf, const void  *data, int index);
enum mv_dp_rc mv_dp_ingress_prio_cfg_parse_struct(void *prio_ptr, void *buf);
enum mv_dp_rc mv_dp_egress_prio_cfg_populate_msg(void *buf, const void  *data, int index);
enum mv_dp_rc mv_dp_egress_prio_cfg_parse_struct(void *prio_ptr, void *buf);


enum mv_dp_rc mv_dp_policy_id_populate_msg(void *buf, const void  *data, int index);
enum mv_dp_rc mv_dp_ingress_qos_policy_parse_struct(void *prio_ptr, void *buf);
enum mv_dp_rc mv_dp_egress_qos_policy_parse_struct(void *prio_ptr, void *buf);
enum mv_dp_rc mv_dp_ingress_qos_policy_populate_msg(void *buf, const void  *data, int index);
enum mv_dp_rc mv_dp_egress_qos_policy_populate_msg(void *buf, const void  *data, int index);



enum mv_dp_rc mv_dp_egress_queue_cfg_parse_struct(void *queue_ptr, void *buf);
enum mv_dp_rc mv_dp_ingress_queue_cfg_parse_struct(void *queue_ptr, void *buf);
enum mv_dp_rc mv_dp_queue_stats_parse_struct(void *queue_stats_ptr, void *buf);

enum mv_dp_rc mv_dp_ingress_queue_cfg_populate_msg(void *buf, const void  *data, int index);
enum mv_dp_rc mv_dp_egress_queue_cfg_populate_msg(void *buf, const void  *data, int index);
enum mv_dp_rc mv_dp_queue_id_cfg_populate_msg(void *buf, const void  *data, int index);

enum mv_dp_rc mv_dp_init_populate_msg(void *buf, const void  *data, int index);

enum mv_dp_rc mv_dp_hash_prof_populate_msg(void *buf, const void *data, int index);
enum mv_dp_rc mv_dp_hash_prof_id_populate_msg(void *buf, const void *data, int index);
enum mv_dp_rc mv_dp_hash_prof_parse_struct(void *except_stats_ptr, void *buf);

enum mv_dp_rc mv_dp_bulk_index_populate_msg(void *buf, const void *data, int offset);
enum mv_dp_rc mv_dp_bulk_options_populate_msg(void *buf, const void *data, int offset);

enum mv_dp_rc mv_dp_mc_bridged_populate_msg(void *buf, const void *data, int index);
enum mv_dp_rc mv_dp_mc_ind_populate_msg(void *buf, const void *data, int index);
enum mv_dp_rc mv_dp_mc_bridged_parse_struct(void *mc_ptr, void *buf);
enum mv_dp_rc mv_dp_mc_tunneled_populate_msg(void *buf, const void *data, int index);
enum mv_dp_rc mv_dp_mc_tunneled_parse_struct(void *mc_ptr, void *buf);
enum mv_dp_rc mv_dp_mc_mgid_populate_msg(void *buf, const void *data, int index);

enum mv_dp_rc mv_dp_dbg_nss_mem_parse_struct(void *out_ptr, void *buf);
enum mv_dp_rc mv_dp_dbg_nss_mem_write_populate_msg(void *buf, const void  *data, int index);
enum mv_dp_rc mv_dp_dbg_nss_mem_read_populate_msg(void *buf, const void  *data, int index);
enum mv_dp_rc mv_dp_hwq_queue_id_cfg_populate_msg(void *buf, const void *data, int index);

/*no specific out parsing*/
enum mv_dp_rc mv_dp_msg_def_msg_rx(struct mv_dp_msg_info_rx *msg_rx, void *p);


#ifdef __cplusplus
}
#endif


#endif
