/*******************************************************************************
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

#include <linux/module.h>
#include <linux/types.h>

#include "mv_nss_dp.h"
#include "mv_dp_types.h"
#include "mv_dp_main.h"
#include "mv_dp_int_if.h"
#include "mv_dp_msg.h"
#include "mv_dp_fw_if.h"

#include "mv_dp_includes.h"

#ifndef MV_DP_USE_TM_LOOPBACK
/*#include "net_dev/mv_dev_vq.h"*/
#endif



/*************************DTLS BEGIN*****************************************/

enum mv_dp_rc mv_dp_dtls_populate_msg(void *buf, const void *data, int index)
{
	const mv_nss_dp_dtls_t *dtls = (const mv_nss_dp_dtls_t *)data + index;
	u32 *msg_buf = (u32 *)buf;

	MV_DP_LOG_DBG3("ENTER: %s buf:%p, data:%p, index:%d\n", __func__, buf, data, index);

	if (!data || !buf)
		return MV_NSS_DP_FAILED;


	msg_buf[MV_DP_MSG_DTLS_DTLS_ID_WORD0] = cpu_to_be32(MV_DP_MSG_DTLS_DTLS_ID_SET(dtls->dtls_id));

	msg_buf[MV_DP_MSG_DTLS_READ_MAC_SEC_LEN_WORD1] =
					(MV_DP_MSG_DTLS_READ_MAC_SEC_LEN_SET(dtls->read_mac_secret_len));
	msg_buf[MV_DP_MSG_DTLS_WRITE_MAC_SEC_LEN_WORD1] |=
					(MV_DP_MSG_DTLS_WRITE_MAC_SEC_LEN_SET(dtls->write_mac_secret_len));
	cpu_to_be32s(&msg_buf[MV_DP_MSG_DTLS_READ_MAC_SEC_LEN_WORD1]);

	memcpy(((u8 *)(&msg_buf[MV_DP_MSG_DTLS_SEQ_ID_WORD2]) + MV_DP_MSG_DTLS_SEQ_ID_OFFS),
		(void *)(&(dtls->seq_id)), MV_DP_MSG_DTLS_SEQ_ID_SIZE);

	msg_buf[MV_DP_MSG_DTLS_EPOCH_WORD4] = (MV_DP_MSG_DTLS_EPOCH_SET(dtls->epoch));
	msg_buf[MV_DP_MSG_DTLS_VER_WORD4] |= (MV_DP_MSG_DTLS_VER_SET(dtls->version));
	cpu_to_be32s(&msg_buf[MV_DP_MSG_DTLS_EPOCH_WORD4]);

	msg_buf[MV_DP_MSG_DTLS_MODE_WORD5] = cpu_to_be32(MV_DP_MSG_DTLS_MODE_SET(dtls->mode));

	memcpy(((u8 *)(&msg_buf[MV_DP_MSG_DTLS_READ_MAC_SEC_WORD6]) + MV_DP_MSG_DTLS_READ_MAC_SEC_OFFS),
		(void *)(&(dtls->read_mac_secret)), dtls->read_mac_secret_len);

	memcpy(((u8 *)(&msg_buf[MV_DP_MSG_DTLS_WRITE_MAC_SEC_WORD22]) + MV_DP_MSG_DTLS_WRITE_MAC_SEC_OFFS),
		(void *)(&(dtls->write_mac_secret)), dtls->write_mac_secret_len);

	memcpy(((u8 *)(&msg_buf[MV_DP_MSG_DTLS_READ_KEY_WORD38]) + MV_DP_MSG_DTLS_READ_KEY_OFFS),
		(void *)(&(dtls->read_key)), MV_DP_MSG_DTLS_READ_KEY_SIZE);

	memcpy(((u8 *)(&msg_buf[MV_DP_MSG_DTLS_WRITE_KEY_WORD46]) + MV_DP_MSG_DTLS_WRITE_KEY_OFFS),
		(void *)(&(dtls->write_key)), MV_DP_MSG_DTLS_WRITE_KEY_SIZE);


	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_DP_RC_OK;
}

enum mv_dp_rc mv_dp_dtls_id_populate_msg(void *buf, const void  *data, int index)
{
	u16 dtls_id = *((u16 *)data  + index);
	u32 *msg_buf = (u32 *)buf;

	MV_DP_LOG_DBG3("ENTER: %s buf:%p, data:%p, index:%d\n", __func__, buf, data, index);

	if (!data || !buf)
		return MV_NSS_DP_FAILED;

	msg_buf[MV_DP_MSG_DTLS_DTLS_ID_WORD0] = cpu_to_be32(MV_DP_MSG_DTLS_DTLS_ID_SET(dtls_id));


	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}

enum mv_dp_rc mv_dp_dtls_parse_struct(void *dtls_ptr, void *buf)
{
	mv_nss_dp_dtls_t *dtls = (mv_nss_dp_dtls_t *)dtls_ptr;
	u32 *msg_buf = (u32 *)buf;
	u32 tmp;

	MV_DP_LOG_DBG3("ENTER: %s\n", __func__);

	if (!dtls || !buf) {
		MV_DP_LOG_ERR("Null Pointer DTLS:%p buf:%p\n", MV_DP_RC_ERR_NULL_PTR, dtls, buf);
		return MV_DP_RC_ERR_NULL_PTR;
	}

	dtls->dtls_id = MV_DP_MSG_DTLS_DTLS_ID_GET(be32_to_cpu(msg_buf[MV_DP_MSG_DTLS_DTLS_ID_WORD0]));

	tmp = be32_to_cpu(msg_buf[MV_DP_MSG_DTLS_READ_MAC_SEC_LEN_WORD1]);
	dtls->read_mac_secret_len = MV_DP_MSG_DTLS_READ_MAC_SEC_LEN_GET(tmp);
	dtls->write_mac_secret_len = MV_DP_MSG_DTLS_WRITE_MAC_SEC_LEN_GET(tmp);

	memcpy((void *)(&(dtls->seq_id)), ((u8 *)(&msg_buf[MV_DP_MSG_DTLS_SEQ_ID_WORD2]) + MV_DP_MSG_DTLS_SEQ_ID_OFFS),
		MV_DP_MSG_DTLS_SEQ_ID_SIZE);

	tmp = be32_to_cpu(msg_buf[MV_DP_MSG_DTLS_EPOCH_WORD4]);
	dtls->epoch = MV_DP_MSG_DTLS_EPOCH_GET(tmp);
	dtls->version = MV_DP_MSG_DTLS_VER_GET(tmp);

	dtls->mode = MV_DP_MSG_DTLS_MODE_GET(be32_to_cpu(msg_buf[MV_DP_MSG_DTLS_MODE_WORD5]));

	memcpy((void *)(&(dtls->read_mac_secret)),
		((u8 *)(&msg_buf[MV_DP_MSG_DTLS_READ_MAC_SEC_WORD6]) + MV_DP_MSG_DTLS_READ_MAC_SEC_OFFS),
		dtls->read_mac_secret_len);

	memcpy((void *)(&(dtls->write_mac_secret)),
		((u8 *)(&msg_buf[MV_DP_MSG_DTLS_WRITE_MAC_SEC_WORD22]) + MV_DP_MSG_DTLS_WRITE_MAC_SEC_OFFS),
		dtls->write_mac_secret_len);

	memcpy((void *)(&(dtls->read_key)),
		((u8 *)(&msg_buf[MV_DP_MSG_DTLS_READ_KEY_WORD38]) + MV_DP_MSG_DTLS_READ_KEY_OFFS),
		MV_DP_MSG_DTLS_READ_KEY_SIZE);

	memcpy((void *)(&(dtls->write_key)),
		((u8 *)(&msg_buf[MV_DP_MSG_DTLS_WRITE_KEY_WORD46]) + MV_DP_MSG_DTLS_WRITE_KEY_OFFS),
		MV_DP_MSG_DTLS_WRITE_KEY_SIZE);


	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}

/*************************DTLS END*******************************************/


/*************************PORT START*******************************************/

enum mv_dp_rc mv_dp_cpuport_populate_msg(void *buf, const void  *data, int index)
{
	const mv_nss_dp_port_t *port = (const mv_nss_dp_port_t *)data  + index;
	u32 *msg_buf = (u32 *)buf;

	MV_DP_LOG_DBG3("ENTER: %s buf:%p, data:%p, index:%d\n", __func__, buf, data, index);

	if (!data || !buf)
		return MV_NSS_DP_FAILED;

	msg_buf[MV_DP_MSG_PORT_ID_WORD0] = (MV_DP_MSG_PORT_ID_SET(port->port_id));
	msg_buf[MV_DP_MSG_PORT_TYPE_WORD0] |= (MV_DP_MSG_PORT_TYPE_SET(port->type));
	msg_buf[MV_DP_MSG_PORT_STATE_WORD0] |= (MV_DP_MSG_PORT_STATE_SET(port->state));
	cpu_to_be32s(&msg_buf[MV_DP_MSG_PORT_ID_WORD0]);

	msg_buf[MV_DP_MSG_CPUPORT_BITMASK_WORD1] =
		cpu_to_be32(MV_DP_MSG_CPUPORT_BITMASK_SET(port->params.cpu.cores));

	msg_buf[MV_DP_MSG_CPUPORT_HASH_PROF_WORD2] =
		cpu_to_be32(MV_DP_MSG_CPUPORT_HASH_PROF_SET(port->params.cpu.hash_prof_id));


	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_DP_RC_OK;
}

enum mv_dp_rc mv_dp_lagport_populate_msg(void *buf, const void  *data, int index)
{
	const mv_nss_dp_port_t *port = (const mv_nss_dp_port_t *)data  + index;
	u32 *msg_buf = (u32 *)buf;

	MV_DP_LOG_DBG3("ENTER: %s buf:%p, data:%p, index:%d\n", __func__, buf, data, index);

	if (!data || !buf)
		return MV_NSS_DP_FAILED;

	msg_buf[MV_DP_MSG_PORT_ID_WORD0] = (MV_DP_MSG_PORT_ID_SET(port->port_id));
	msg_buf[MV_DP_MSG_PORT_TYPE_WORD0] |= (MV_DP_MSG_PORT_TYPE_SET(port->type));
	msg_buf[MV_DP_MSG_PORT_STATE_WORD0] |= (MV_DP_MSG_PORT_STATE_SET(port->state));
	cpu_to_be32s(&msg_buf[MV_DP_MSG_PORT_ID_WORD0]);

	msg_buf[MV_DP_MSG_LAGPORT_LINKS_WORD1] =
		cpu_to_be32(MV_DP_MSG_LAGPORT_LINKS_SET(port->params.eth_lag.links));

	msg_buf[MV_DP_MSG_LAGPORT_HASH_PROF_WORD2] =
		cpu_to_be32(MV_DP_MSG_LAGPORT_HASH_PROF_SET(port->params.eth_lag.hash_prof_id));

	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_DP_RC_OK;
}

enum mv_dp_rc mv_dp_eport_populate_msg(void *buf, const void  *data, int index)
{
	const mv_nss_dp_port_t *port = (const mv_nss_dp_port_t *)data  + index;
	u32 *msg_buf = (u32 *)buf;

	MV_DP_LOG_DBG3("ENTER: %s buf:%p, data:%p, index:%d\n", __func__, buf, data, index);

	if (!data || !buf)
		return MV_NSS_DP_FAILED;

	msg_buf[MV_DP_MSG_PORT_ID_WORD0] = (MV_DP_MSG_PORT_ID_SET(port->port_id));
	msg_buf[MV_DP_MSG_PORT_TYPE_WORD0] |= (MV_DP_MSG_PORT_TYPE_SET(port->type));
	msg_buf[MV_DP_MSG_PORT_STATE_WORD0] |= (MV_DP_MSG_PORT_STATE_SET(port->state));
	cpu_to_be32s(&msg_buf[MV_DP_MSG_PORT_ID_WORD0]);

	msg_buf[MV_DP_MSG_EPORT_OPTIONS_WORD1] = cpu_to_be32(MV_DP_MSG_EPORT_OPTIONS_SET(port->params.eth.options));

	msg_buf[MV_DP_MSG_EPORT_NVLAN_WORD2] = MV_DP_MSG_EPORT_NVLAN_SET(port->params.eth.native_vlan);
	msg_buf[MV_DP_MSG_EPORT_VLAN_POL_WORD2] |= MV_DP_MSG_EPORT_VLAN_POL_SET(port->params.eth.policy);
	cpu_to_be32s(&msg_buf[MV_DP_MSG_EPORT_NVLAN_WORD2]);

	msg_buf[MV_DP_MSG_EPORT_TPID_WORD3] = MV_DP_MSG_EPORT_TPID_SET(port->params.eth.tpid);
	msg_buf[MV_DP_MSG_EPORT_MTU_WORD3] |= MV_DP_MSG_EPORT_MTU_SET(port->params.eth.mtu);
	cpu_to_be32s(&msg_buf[MV_DP_MSG_EPORT_TPID_WORD3]);

	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_DP_RC_OK;
}



enum mv_dp_rc mv_dp_cwport_populate_msg(void *buf, const void  *data, int index)
{
	const mv_nss_dp_port_t *port = (const mv_nss_dp_port_t *)data  + index;
	u32 *msg_buf = (u32 *)buf;

	MV_DP_LOG_DBG3("ENTER: %s buf:%p, data:%p, index:%d\n", __func__, buf, data, index);

	if (!data || !buf)
		return MV_NSS_DP_FAILED;

	msg_buf[MV_DP_MSG_PORT_ID_WORD0] = (MV_DP_MSG_PORT_ID_SET(port->port_id));
	msg_buf[MV_DP_MSG_PORT_TYPE_WORD0] |= (MV_DP_MSG_PORT_TYPE_SET(port->type));
	msg_buf[MV_DP_MSG_PORT_STATE_WORD0] |= (MV_DP_MSG_PORT_STATE_SET(port->state));
	cpu_to_be32s(&msg_buf[MV_DP_MSG_PORT_ID_WORD0]);



	memcpy(((u8 *)(&msg_buf[MV_DP_MSG_CWPORT_RMAC_WORD1]) + MV_DP_MSG_CWPORT_RMAC_OFFS_B),
	       (void *)(&(port->params.capwap.remote_mac.addr)), MV_DP_MSG_CWPORT_RMAC_SIZE);

	memcpy(((u8 *)(&msg_buf[MV_DP_MSG_CWPORT_LMAC_WORD2]) + MV_DP_MSG_CWPORT_LMAC_OFFS_B),
	       (void *)(&(port->params.capwap.local_mac.addr)), MV_DP_MSG_CWPORT_LMAC_SIZE);

	memcpy(((u8 *)(&msg_buf[MV_DP_MSG_CWPORT_BSSID_WORD4]) + MV_DP_MSG_CWPORT_BSSID_OFFS_B),
	       (void *)(&(port->params.capwap.bssid)), MV_DP_MSG_CWPORT_BSSID_SIZE);

	memcpy(((u8 *)(&msg_buf[MV_DP_MSG_CWPORT_RIP_WORD6]) + MV_DP_MSG_CWPORT_RIP_OFFS_B),
	       (void *)(&(port->params.capwap.remote_ip.ip)), MV_DP_MSG_CWPORT_IP_SIZE);

	memcpy(((u8 *)(&msg_buf[MV_DP_MSG_CWPORT_LIP_WORD10]) + MV_DP_MSG_CWPORT_LIP_OFFS_B),
	       (void *)(&(port->params.capwap.local_ip.ip)), MV_DP_MSG_CWPORT_IP_SIZE);

	msg_buf[MV_DP_MSG_CWPORT_TTL_WORD14] = cpu_to_be32(MV_DP_MSG_CWPORT_TTL_SET(port->params.capwap.ttl));

	msg_buf[MV_DP_MSG_CWPORT_FLOW_LBL_WORD15] =
					cpu_to_be32(MV_DP_MSG_CWPORT_FLOW_LBL_SET(port->params.capwap.flow_lbl));

	msg_buf[MV_DP_MSG_CWPORT_DTLS_IND_WORD16] = MV_DP_MSG_CWPORT_DTLS_IND_SET(port->params.capwap.dtls_index);
	msg_buf[MV_DP_MSG_CWPORT_PMTU_WORD16] |= MV_DP_MSG_CWPORT_PMTU_SET(port->params.capwap.pmtu);
	cpu_to_be32s(&msg_buf[MV_DP_MSG_CWPORT_DTLS_IND_WORD16]);

	msg_buf[MV_DP_MSG_CWPORT_RPORT_WORD17] = MV_DP_MSG_CWPORT_RPORT_SET(port->params.capwap.remote_port);
	msg_buf[MV_DP_MSG_CWPORT_LPORT_WORD17] |= MV_DP_MSG_CWPORT_LPORT_SET(port->params.capwap.local_port);
	cpu_to_be32s(&msg_buf[MV_DP_MSG_CWPORT_RPORT_WORD17]);

	msg_buf[MV_DP_MSG_CWPORT_UC_QOS_WORD18] = MV_DP_MSG_CWPORT_UC_QOS_SET(port->params.capwap.uc_qos_policy);
	msg_buf[MV_DP_MSG_CWPORT_MC_QOS_WORD18] |= MV_DP_MSG_CWPORT_MC_QOS_SET(port->params.capwap.mc_qos_policy);
	msg_buf[MV_DP_MSG_CWPORT_CSUM_WORD18] |= MV_DP_MSG_CWPORT_CSUM_SET(port->params.capwap.calc_cs);
	cpu_to_be32s(&msg_buf[MV_DP_MSG_CWPORT_UC_QOS_WORD18]);

	msg_buf[MV_DP_MSG_CWPORT_PORT_ID_WORD19] = MV_DP_MSG_CWPORT_PORT_ID_SET(port->params.capwap.port_id);
	msg_buf[MV_DP_MSG_CWPORT_OPTIONS_WORD19] |= MV_DP_MSG_CWPORT_OPTIONS_SET(port->params.capwap.options);
	msg_buf[MV_DP_MSG_CWPORT_L4_PROTO_WORD19] |= MV_DP_MSG_CWPORT_L4_PROTO_SET(port->params.capwap.proto);
	msg_buf[MV_DP_MSG_CWPORT_IPVER_WORD19] |= MV_DP_MSG_CWPORT_IPVER_SET(port->params.capwap.remote_ip.ver);
	cpu_to_be32s(&msg_buf[MV_DP_MSG_CWPORT_PORT_ID_WORD19]);


	msg_buf[MV_DP_MSG_CWPORT_VLAN_ID_WORD21] = MV_DP_MSG_CWPORT_VLAN_ID_SET(port->params.capwap.vlan_id);

	cpu_to_be32s(&msg_buf[MV_DP_MSG_CWPORT_VLAN_ID_WORD21]);

	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_DP_RC_OK;
}


enum mv_dp_rc mv_dp_port_id_populate_msg(void *buf, const void  *data, int index)
{
	mv_nss_dp_port_id_t port_id = *((mv_nss_dp_port_id_t *)data  + index);
	u32 *msg_buf = (u32 *)buf;

	MV_DP_LOG_DBG3("ENTER: %s buf:%p, data:%p, index:%d port_id:%d\n", __func__, buf, data, index, port_id);

	if (!data || !buf)
		return MV_NSS_DP_FAILED;

	msg_buf[MV_DP_MSG_PORT_ID_WORD0] = cpu_to_be32(MV_DP_MSG_PORT_ID_SET(port_id));

	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_DP_RC_OK;
}

/*populates buffer from msg to port structure*/
enum mv_dp_rc mv_dp_port_parse_struct(void *msg_ptr, void *buf)
{
	struct mv_dp_msg_info_rx *rx_msg = (struct mv_dp_msg_info_rx *)msg_ptr;
	mv_nss_dp_port_type_t	prt_type;


	MV_DP_LOG_DBG3("ENTER: %s msg_ptr:%p\n", __func__, msg_ptr);

	if (!msg_ptr) {
		MV_DP_LOG_ERR("Null Pointer msg_ptr:%p buf:%p\n", MV_DP_RC_ERR_NULL_PTR, msg_ptr, buf);
		return MV_DP_RC_ERR_NULL_PTR;
	}
	/*correct the opcode based on type*/

	prt_type = MV_DP_MSG_PORT_TYPE_GET(be32_to_cpu(*((u32 *)rx_msg->msg_data + MV_DP_MSG_PORT_TYPE_WORD0)));
	MV_DP_LOG_DBG2("%s, Got msg port type:%d word:0x%08X\n", __func__, prt_type,
							*((u32 *)rx_msg->msg_data + MV_DP_MSG_PORT_TYPE_WORD0));
#ifdef REV2
	switch (prt_type) {
	case (MV_FW_DP_PORT_ETH):
		rx_msg->opcode = MV_DP_MSGID_EPORT_GET;
		break;
	case (MV_FW_DP_PORT_ETH_LAG):
		rx_msg->opcode = MV_DP_MSGID_LAGPORT_GET;
		break;
	case (MV_FW_DP_PORT_CPU):
		rx_msg->opcode = MV_DP_MSGID_CPUPORT_GET;
		break;
	case (MV_FW_DP_PORT_CAPWAP):
		rx_msg->opcode = MV_DP_MSGID_CWPORT_GET;
		break;
	default:
		MV_DP_LOG_ERR("INVALID Port Type in Get Port type:%d\n", MV_DP_RC_ERR_NULL_PTR, prt_type);
		return MV_DP_RC_ERR_MSG_PARSE_FAILED;
	}
#endif
	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_DP_RC_OK;
}

enum mv_dp_rc mv_dp_cwport_parse_struct(void *port_ptr, void *buf)
{
	mv_nss_dp_port_t *port = (mv_nss_dp_port_t *)port_ptr;
	u32 *msg_buf = (u32 *)buf;
	u32 tmp;

	MV_DP_LOG_DBG3("ENTER: %s\n", __func__);

	if (!port || !buf) {
		MV_DP_LOG_ERR("Null Pointer port:%p buf:%p\n", MV_DP_RC_ERR_NULL_PTR, port, buf);
		return MV_DP_RC_ERR_NULL_PTR;
	}

	tmp = be32_to_cpu(msg_buf[MV_DP_MSG_PORT_ID_WORD0]);
	port->port_id = MV_DP_MSG_PORT_ID_GET(tmp);
	port->type = MV_DP_MSG_PORT_TYPE_GET(tmp);
	port->state = MV_DP_MSG_PORT_STATE_GET(tmp);

#ifdef MV_DP_DEBUG
if (port->type != MV_NSS_DP_PORT_CAPWAP) {
	MV_DP_LOG_ERR("Wrong port type port_id:%d type:%d expected:%d\n",
		      MV_DP_RC_ERR_MSG_PARSE_FAILED, port->port_id, port->type, MV_NSS_DP_PORT_CAPWAP);
	return MV_DP_RC_ERR_MSG_PARSE_FAILED;
}
#endif

	memcpy((void *)(&(port->params.capwap.remote_mac.addr)),
	       ((u8 *)(&msg_buf[MV_DP_MSG_CWPORT_RMAC_WORD1]) + MV_DP_MSG_CWPORT_RMAC_OFFS_B),
	       MV_DP_MSG_CWPORT_RMAC_SIZE);

	memcpy((void *)(&(port->params.capwap.local_mac.addr)),
	       ((u8 *)(&msg_buf[MV_DP_MSG_CWPORT_LMAC_WORD2]) + MV_DP_MSG_CWPORT_LMAC_OFFS_B),
	       MV_DP_MSG_CWPORT_LMAC_SIZE);

	memcpy((void *)(&(port->params.capwap.bssid)),
	       ((u8 *)(&msg_buf[MV_DP_MSG_CWPORT_BSSID_WORD4]) + MV_DP_MSG_CWPORT_BSSID_OFFS_B),
	       MV_DP_MSG_CWPORT_BSSID_SIZE);

	memcpy((void *)(&(port->params.capwap.remote_ip.ip)),
	       ((u8 *)(&msg_buf[MV_DP_MSG_CWPORT_RIP_WORD6]) + MV_DP_MSG_CWPORT_RIP_OFFS_B),
	       MV_DP_MSG_CWPORT_IP_SIZE);

	memcpy((void *)(&(port->params.capwap.local_ip.ip)),
	       ((u8 *)(&msg_buf[MV_DP_MSG_CWPORT_LIP_WORD10]) + MV_DP_MSG_CWPORT_LIP_OFFS_B),
	       MV_DP_MSG_CWPORT_IP_SIZE);

	port->params.capwap.ttl = MV_DP_MSG_CWPORT_TTL_GET(be32_to_cpu(msg_buf[MV_DP_MSG_CWPORT_TTL_WORD14]));

	port->params.capwap.flow_lbl =
		MV_DP_MSG_CWPORT_FLOW_LBL_GET(be32_to_cpu(msg_buf[MV_DP_MSG_CWPORT_FLOW_LBL_WORD15]));

	tmp = be32_to_cpu(msg_buf[MV_DP_MSG_CWPORT_DTLS_IND_WORD16]);
	port->params.capwap.dtls_index = MV_DP_MSG_CWPORT_DTLS_IND_GET(tmp);
	port->params.capwap.pmtu = MV_DP_MSG_CWPORT_PMTU_GET(tmp);

	tmp = be32_to_cpu(msg_buf[MV_DP_MSG_CWPORT_RPORT_WORD17]);
	port->params.capwap.remote_port = MV_DP_MSG_CWPORT_RPORT_GET(tmp);
	port->params.capwap.local_port = MV_DP_MSG_CWPORT_LPORT_GET(tmp);

	tmp = be32_to_cpu(msg_buf[MV_DP_MSG_CWPORT_UC_QOS_WORD18]);
	port->params.capwap.uc_qos_policy = MV_DP_MSG_CWPORT_UC_QOS_GET(tmp);
	port->params.capwap.mc_qos_policy = MV_DP_MSG_CWPORT_MC_QOS_GET(tmp);
	port->params.capwap.calc_cs = MV_DP_MSG_CWPORT_CSUM_GET(tmp);

	tmp = be32_to_cpu(msg_buf[MV_DP_MSG_CWPORT_PORT_ID_WORD19]);
	port->params.capwap.port_id = MV_DP_MSG_CWPORT_PORT_ID_GET(tmp);
	port->params.capwap.proto = MV_DP_MSG_CWPORT_L4_PROTO_GET(tmp);
	port->params.capwap.options = MV_DP_MSG_CWPORT_OPTIONS_GET(tmp);

	port->params.capwap.local_ip.ver =
	port->params.capwap.remote_ip.ver =
	MV_DP_MSG_CWPORT_IPVER_GET(tmp);


	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}


enum mv_dp_rc mv_dp_eport_parse_struct(void *port_ptr, void *buf)
{
	mv_nss_dp_port_t *port = (mv_nss_dp_port_t *)port_ptr;
	u32 *msg_buf = (u32 *)buf;
	u32 tmp;

	MV_DP_LOG_DBG3("ENTER: %s\n", __func__);

	if (!port || !buf) {
		MV_DP_LOG_ERR("Null Pointer port:%p buf:%p\n", MV_DP_RC_ERR_NULL_PTR, port, buf);
		return MV_DP_RC_ERR_NULL_PTR;
	}

	tmp = be32_to_cpu(msg_buf[MV_DP_MSG_PORT_ID_WORD0]);
	port->port_id = MV_DP_MSG_PORT_ID_GET(tmp);
	port->type = MV_DP_MSG_PORT_TYPE_GET(tmp);
	port->state = MV_DP_MSG_PORT_STATE_GET(tmp);

#ifdef MV_DP_DEBUG
if (port->type != MV_NSS_DP_PORT_ETH) {
	MV_DP_LOG_ERR("Wrong port type port_id:%d type:%d expected:%d\n",
		      MV_DP_RC_ERR_MSG_PARSE_FAILED, port->port_id, port->type, MV_NSS_DP_PORT_ETH);
	return MV_DP_RC_ERR_MSG_PARSE_FAILED;
}
#endif

	port->params.eth.options = MV_DP_MSG_EPORT_OPTIONS_GET(be32_to_cpu(msg_buf[MV_DP_MSG_EPORT_OPTIONS_WORD1]));

	tmp = be32_to_cpu(msg_buf[MV_DP_MSG_EPORT_NVLAN_WORD2]);
	port->params.eth.native_vlan = MV_DP_MSG_EPORT_NVLAN_GET(tmp);
	port->params.eth.policy = MV_DP_MSG_EPORT_VLAN_POL_GET(tmp);

	tmp = be32_to_cpu(msg_buf[MV_DP_MSG_EPORT_TPID_WORD3]);
	port->params.eth.tpid = MV_DP_MSG_EPORT_TPID_GET(tmp);
	port->params.eth.mtu = MV_DP_MSG_EPORT_MTU_GET(tmp);

	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}


enum mv_dp_rc mv_dp_lagport_parse_struct(void *port_ptr, void *buf)
{
	mv_nss_dp_port_t *port = (mv_nss_dp_port_t *)port_ptr;
	u32 *msg_buf = (u32 *)buf;
	u32 tmp;

	MV_DP_LOG_DBG3("ENTER: %s\n", __func__);

	if (!port || !buf) {
		MV_DP_LOG_ERR("Null Pointer port:%p buf:%p\n", MV_DP_RC_ERR_NULL_PTR, port, buf);
		return MV_DP_RC_ERR_NULL_PTR;
	}

	tmp = be32_to_cpu(msg_buf[MV_DP_MSG_PORT_ID_WORD0]);
	port->port_id = MV_DP_MSG_PORT_ID_GET(tmp);
	port->type = MV_DP_MSG_PORT_TYPE_GET(tmp);
	port->state = MV_DP_MSG_PORT_STATE_GET(tmp);

#ifdef MV_DP_DEBUG
if (port->type != MV_NSS_DP_PORT_ETH_LAG) {
	MV_DP_LOG_ERR("Wrong port type port_id:%d type:%d expected:%d\n",
		      MV_DP_RC_ERR_MSG_PARSE_FAILED, port->port_id, port->type, MV_NSS_DP_PORT_ETH_LAG);
	return MV_DP_RC_ERR_MSG_PARSE_FAILED;
}
#endif

	port->params.eth_lag.links =
		MV_DP_MSG_LAGPORT_LINKS_GET(be32_to_cpu(msg_buf[MV_DP_MSG_LAGPORT_LINKS_WORD1]));
	port->params.eth_lag.hash_prof_id =
		MV_DP_MSG_LAGPORT_HASH_PROF_GET(be32_to_cpu(msg_buf[MV_DP_MSG_LAGPORT_HASH_PROF_WORD2]));

	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}


enum mv_dp_rc mv_dp_cpuport_parse_struct(void *port_ptr, void *buf)
{
	mv_nss_dp_port_t *port = (mv_nss_dp_port_t *)port_ptr;
	u32 *msg_buf = (u32 *)buf;
	u32 tmp;

	MV_DP_LOG_DBG3("ENTER: %s\n", __func__);

	if (!port || !buf) {
		MV_DP_LOG_ERR("Null Pointer port:%p buf:%p\n", MV_DP_RC_ERR_NULL_PTR, port, buf);
		return MV_DP_RC_ERR_NULL_PTR;
	}

	tmp = be32_to_cpu(msg_buf[MV_DP_MSG_PORT_ID_WORD0]);
	port->port_id = MV_DP_MSG_PORT_ID_GET(tmp);
	port->type = MV_DP_MSG_PORT_TYPE_GET(tmp);
	port->state = MV_DP_MSG_PORT_STATE_GET(tmp);


#ifdef MV_DP_DEBUG
if (port->type != MV_NSS_DP_PORT_CPU) {
	MV_DP_LOG_ERR("Wrong port type port_id:%d type:%d expected:%d\n",
		      MV_DP_RC_ERR_MSG_PARSE_FAILED, port->port_id, port->type, MV_NSS_DP_PORT_CPU);
	return MV_DP_RC_ERR_MSG_PARSE_FAILED;
}
#endif

	port->params.cpu.hash_prof_id =
		MV_DP_MSG_CPUPORT_HASH_PROF_GET(be32_to_cpu(msg_buf[MV_DP_MSG_CPUPORT_HASH_PROF_WORD2]));
	port->params.cpu.cores =
		MV_DP_MSG_CPUPORT_BITMASK_GET(be32_to_cpu(msg_buf[MV_DP_MSG_CPUPORT_BITMASK_WORD1]));

	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}


/*populates buffer from msg to port stats structure*/
enum mv_dp_rc mv_dp_port_stats_parse_struct(void *port_stats_ptr, void *buf)
{
	mv_nss_dp_port_stats_t *port_stats = (mv_nss_dp_port_stats_t *)port_stats_ptr;
	u32 *msg_buf = (u32 *)buf;


	MV_DP_LOG_DBG3("ENTER: %s\n", __func__);

	if (!port_stats || !buf) {
		MV_DP_LOG_ERR("Null Pointer port stats:%p buf:%p\n", MV_DP_RC_ERR_NULL_PTR, port_stats, buf);
		return MV_DP_RC_ERR_NULL_PTR;
	}

	/*word 0*/
	be32_to_cpus(&msg_buf[MV_DP_MSG_PORT_ID_WORD0]);

	port_stats->port_id = MV_DP_MSG_PORT_ID_GET(msg_buf[MV_DP_MSG_PORT_ID_WORD0]);
	port_stats->type = MV_DP_MSG_PORT_TYPE_GET(msg_buf[MV_DP_MSG_PORT_TYPE_WORD0]);

	port_stats->rx_pkts = be64_to_cpu(*(uint64_t *)(&msg_buf[MV_DP_MSG_PORT_STAT_RX_PKTS_WORD1]));
	port_stats->rx_errors = be64_to_cpu(*(uint64_t *)(&msg_buf[MV_DP_MSG_PORT_STAT_RX_ERR_WORD3]));
	port_stats->tx_pkts = be64_to_cpu(*(uint64_t *)(&msg_buf[MV_DP_MSG_PORT_STAT_TX_PKTS_WORD5]));
	port_stats->tx_errors = be64_to_cpu(*(uint64_t *)(&msg_buf[MV_DP_MSG_PORT_STAT_TX_ERR_WORD7]));
	port_stats->rx_octets = be64_to_cpu(*(uint64_t *)(&msg_buf[MV_DP_MSG_PORT_STAT_RX_OCTS_WORD9]));
	port_stats->tx_octets = be64_to_cpu(*(uint64_t *)(&msg_buf[MV_DP_MSG_PORT_STAT_TX_OCTS_WORD11]));

	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}

/*************************PORT END*******************************************/

/*************************VLAN START*******************************************/

enum mv_dp_rc mv_dp_vlan_populate_msg(void *buf, const void *data, int index)
{
	const mv_nss_dp_vlan_cfg_t *vlan = (const mv_nss_dp_vlan_cfg_t *)data  + index;
	u32 *msg_buf = (u32 *)buf;

	MV_DP_LOG_DBG3("ENTER: %s buf:%p, data:%p, index:%d\n", __func__, buf, data, index);

	if (!data || !buf)
		return MV_NSS_DP_FAILED;

	msg_buf[MV_DP_MSG_VLAN_CFG_VLAN_ID_WORD0] = cpu_to_be32(MV_DP_MSG_VLAN_CFG_VLAN_ID_SET(vlan->vlan_id));

	msg_buf[MV_DP_MSG_VLAN_CFG_UC_QOS_POL_WORD1] = MV_DP_MSG_VLAN_CFG_UC_QOS_POL_SET(vlan->uc_qos_policy);
	msg_buf[MV_DP_MSG_VLAN_CFG_MC_QOS_POL_WORD1] |= MV_DP_MSG_VLAN_CFG_MC_QOS_POL_SET(vlan->mc_qos_policy);
	msg_buf[MV_DP_MSG_VLAN_CFG_PORTS_MASK_WORD1] |= MV_DP_MSG_VLAN_CFG_PORTS_MASK_SET(vlan->ports_mask);
	cpu_to_be32s(&msg_buf[MV_DP_MSG_VLAN_CFG_UC_QOS_POL_WORD1]);

	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}

enum mv_dp_rc mv_dp_vlan_ind_populate_msg(void *buf, const void *data, int index)
{
	u16 vlan_index = *((u16 *)data  + index);
	u32 *msg_buf = (u32 *)buf;

	MV_DP_LOG_DBG3("ENTER: %s buf:%p, data:%p, index:%d\n", __func__, buf, data, index);

	if (!data || !buf)
		return MV_NSS_DP_FAILED;

	msg_buf[MV_DP_MSG_VLAN_CFG_INDEX_WORD0] = cpu_to_be32(MV_DP_MSG_VLAN_CFG_INDEX_SET(vlan_index));

	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}

enum mv_dp_rc mv_dp_vlan_id_populate_msg(void *buf, const void *data, int index)
{
	u16 vlan_id = *((u16 *)data  + index);
	u32 *msg_buf = (u32 *)buf;

	MV_DP_LOG_DBG3("ENTER: %s buf:%p, data:%p, index:%d\n", __func__, buf, data, index);

	if (!data || !buf)
		return MV_NSS_DP_FAILED;

	msg_buf[MV_DP_MSG_VLAN_CFG_VLAN_ID_WORD0] = cpu_to_be32(MV_DP_MSG_VLAN_CFG_VLAN_ID_SET(vlan_id));

	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}

enum mv_dp_rc mv_dp_vlan_parse_struct(void *vlan_ptr, void *buf)
{
	mv_nss_dp_vlan_cfg_t *vlan = (mv_nss_dp_vlan_cfg_t *)vlan_ptr;
	u32 *msg_buf = (u32 *)buf;
	u32 tmp;

	MV_DP_LOG_DBG3("ENTER: %s\n", __func__);

	if (!vlan || !buf) {
		MV_DP_LOG_ERR("Null Pointer vlan:%p buf:%p\n", MV_DP_RC_ERR_NULL_PTR, vlan, buf);
		return MV_DP_RC_ERR_NULL_PTR;
	}

	vlan->vlan_id = MV_DP_MSG_VLAN_CFG_VLAN_ID_GET(be32_to_cpu(msg_buf[MV_DP_MSG_VLAN_CFG_VLAN_ID_WORD0]));

	tmp = be32_to_cpu(msg_buf[MV_DP_MSG_VLAN_CFG_UC_QOS_POL_WORD1]);
	vlan->uc_qos_policy = MV_DP_MSG_VLAN_CFG_UC_QOS_POL_GET(tmp);
	vlan->mc_qos_policy = MV_DP_MSG_VLAN_CFG_MC_QOS_POL_GET(tmp);
	vlan->ports_mask = MV_DP_MSG_VLAN_CFG_PORTS_MASK_GET(tmp);

	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}


/*************************VLAN END*******************************************/


/*************************MC START*******************************************/

enum mv_dp_rc mv_dp_mc_bridged_populate_msg(void *buf, const void *data, int index)
{
	const mv_nss_dp_mc_bridged_cfg_t *mc = (const mv_nss_dp_mc_bridged_cfg_t *)data  + index;
	u32 *msg_buf = (u32 *)buf;

	MV_DP_LOG_DBG3("ENTER: %s buf:%p, data:%p, index:%d\n", __func__, buf, data, index);

	if (!data || !buf)
		return MV_NSS_DP_FAILED;

	memcpy(((u8 *)(&msg_buf[MV_DP_MSG_MC_BRIDGED_CFG_MAC_WORD0]) + MV_DP_MSG_MC_BRIDGED_CFG_MAC_OFFS_B),
	       (void *)(&(mc->l2_addr.addr)), MV_DP_MSG_MC_BRIDGED_CFG_MAC_SIZE);

	msg_buf[MV_DP_MSG_MC_BRIDGED_CFG_VLAN_WORD2] = cpu_to_be32(MV_DP_MSG_MC_BRIDGED_CFG_VLAN_SET(mc->vlan_id));
	msg_buf[MV_DP_MSG_MC_BRIDGED_CFG_OD_WORD3] = cpu_to_be32(MV_DP_MSG_MC_BRIDGED_CFG_OD_SET(mc->opaque));

	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}

enum mv_dp_rc mv_dp_mc_ind_populate_msg(void *buf, const void *data, int index)
{
	u16 ind = *((u16 *)data  + index);
	u32 *msg_buf = (u32 *)buf;

	MV_DP_LOG_DBG3("ENTER: %s buf:%p, data:%p, index:%d\n", __func__, buf, data, index);

	if (!data || !buf)
		return MV_NSS_DP_FAILED;

	msg_buf[MV_DP_MSG_MC_CFG_INDEX_WORD0] = cpu_to_be32(MV_DP_MSG_MC_CFG_INDEX_SET(ind));

	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}

enum mv_dp_rc mv_dp_mc_mgid_populate_msg(void *buf, const void *data, int index)
{
	u16 mgid = *((u16 *)data  + index);
	u32 *msg_buf = (u32 *)buf;

	MV_DP_LOG_DBG3("ENTER: %s buf:%p, data:%p, index:%d\n", __func__, buf, data, index);

	if (!data || !buf)
		return MV_NSS_DP_FAILED;

	msg_buf[MV_DP_MSG_MC_TUNNELED_CFG_MGID_WORD0] = cpu_to_be32(MV_DP_MSG_MC_TUNNELED_CFG_MGID_SET(mgid));

	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}


enum mv_dp_rc mv_dp_mc_bridged_parse_struct(void *mc_ptr, void *buf)
{
	mv_nss_dp_mc_bridged_cfg_t *mc = (mv_nss_dp_mc_bridged_cfg_t *)mc_ptr;
	u32 *msg_buf = (u32 *)buf;

	MV_DP_LOG_DBG3("ENTER: %s\n", __func__);

	if (!mc || !buf) {
		MV_DP_LOG_ERR("Null Pointer mc:%p buf:%p\n", MV_DP_RC_ERR_NULL_PTR, mc, buf);
		return MV_DP_RC_ERR_NULL_PTR;
	}

	memcpy((void *)(&(mc->l2_addr)),
	       ((u8 *)(&msg_buf[MV_DP_MSG_MC_BRIDGED_CFG_MAC_WORD0]) + MV_DP_MSG_MC_BRIDGED_CFG_MAC_OFFS_B),
	       MV_DP_MSG_MC_BRIDGED_CFG_MAC_SIZE);

	mc->vlan_id = MV_DP_MSG_MC_BRIDGED_CFG_VLAN_GET(be32_to_cpu(msg_buf[MV_DP_MSG_MC_BRIDGED_CFG_VLAN_WORD2]));
	mc->opaque = MV_DP_MSG_MC_BRIDGED_CFG_OD_GET(be32_to_cpu(msg_buf[MV_DP_MSG_MC_BRIDGED_CFG_OD_WORD3]));

	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}

enum mv_dp_rc mv_dp_mc_tunneled_populate_msg(void *buf, const void *data, int index)
{
	const mv_nss_dp_mc_tunneled_cfg_t *mc = (const mv_nss_dp_mc_tunneled_cfg_t *)data  + index;
	u32 *msg_buf = (u32 *)buf;

	MV_DP_LOG_DBG3("ENTER: %s buf:%p, data:%p, index:%d\n", __func__, buf, data, index);

	if (!data || !buf)
		return MV_NSS_DP_FAILED;

	msg_buf[MV_DP_MSG_MC_TUNNELED_CFG_MGID_WORD0] = cpu_to_be32(MV_DP_MSG_MC_TUNNELED_CFG_MGID_SET(mc->mgid));
	msg_buf[MV_DP_MSG_MC_TUNNELED_CFG_POL_ID_WORD1] = cpu_to_be32(MV_DP_MSG_MC_TUNNELED_CFG_POL_ID_SET(mc->policy_id));

	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}


enum mv_dp_rc mv_dp_mc_tunneled_parse_struct(void *mc_ptr, void *buf)
{
	mv_nss_dp_mc_tunneled_cfg_t *mc = (mv_nss_dp_mc_tunneled_cfg_t *)mc_ptr;
	u32 *msg_buf = (u32 *)buf;


	MV_DP_LOG_DBG3("ENTER: %s\n", __func__);

	if (!mc || !buf) {
		MV_DP_LOG_ERR("Null Pointer mc:%p buf:%p\n", MV_DP_RC_ERR_NULL_PTR, mc, buf);
		return MV_DP_RC_ERR_NULL_PTR;
	}


	mc->mgid = MV_DP_MSG_MC_TUNNELED_CFG_MGID_GET(be32_to_cpu(msg_buf[MV_DP_MSG_MC_TUNNELED_CFG_MGID_WORD0]));
	mc->policy_id = MV_DP_MSG_MC_TUNNELED_CFG_POL_ID_GET(be32_to_cpu(msg_buf[MV_DP_MSG_MC_TUNNELED_CFG_POL_ID_WORD1]));

	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}


/*************************MC END*******************************************/


/*************************QOS START*******************************************/

enum mv_dp_rc mv_dp_policy_id_populate_msg(void *buf, const void *data, int index)
{
	const u8 *policy_id = (const u8 *)data  + index;
	u32 *msg_buf = (u32 *)buf;

	MV_DP_LOG_DBG3("ENTER: %s buf:%p, data:%p, index:%d\n", __func__, buf, data, index);

#ifdef MV_DP_DEBUG
	if (!policy_id || !buf) {
		MV_DP_LOG_ERR("populate policy ID null ptr\n", MV_DP_RC_ERR_NULL_PTR);
		return MV_DP_RC_ERR_NULL_PTR;
	}
#endif

	msg_buf[MV_DP_MSG_QOS_POLICY_ID_WORD0] = cpu_to_be32(MV_DP_MSG_QOS_POLICY_ID_SET(*policy_id));


	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}

enum mv_dp_rc mv_dp_egress_qos_policy_populate_msg(void *buf, const void *data, int index)
{
	const mv_nss_dp_egress_qos_policy_t *policy = (const mv_nss_dp_egress_qos_policy_t *)data  + index;
	u32 *msg_buf = (u32 *)buf;

	MV_DP_LOG_DBG3("ENTER: %s buf:%p, data:%p, index:%d\n", __func__, buf, data, index);

#ifdef MV_DP_DEBUG
	if (!policy || !buf) {
		MV_DP_LOG_ERR("populate egress policy qos null ptr\n", MV_DP_RC_ERR_NULL_PTR);
		return MV_DP_RC_ERR_NULL_PTR;
	}
#endif

	msg_buf[MV_DP_MSG_QOS_POLICY_ID_WORD0] = cpu_to_be32(MV_DP_MSG_QOS_POLICY_ID_SET(policy->policy_id));


	memcpy((((u8 *)(&msg_buf[MV_DP_MSG_EGRESS_QOS_POLICY_PRIO2UP_WORD1])) +
		MV_DP_MSG_EGRESS_QOS_POLICY_PRIO2UP_OFFS_B),
		(u8 *)(&(policy->prio_to_up)), MV_DP_MSG_EGRESS_QOS_POLICY_PRIO2UP_SIZE);

	memcpy((((u8 *)(&msg_buf[MV_DP_MSG_EGRESS_QOS_POLICY_PRIO2DSCP_WORD5])) +
		MV_DP_MSG_EGRESS_QOS_POLICY_PRIO2DSCP_OFFS_B),
		(u8 *)(&(policy->prio_to_dscp)), MV_DP_MSG_EGRESS_QOS_POLICY_PRIO2DSCP_SIZE);

	memcpy((((u8 *)(&msg_buf[MV_DP_MSG_EGRESS_QOS_POLICY_PRIO2PCP_WORD9])) +
		MV_DP_MSG_EGRESS_QOS_POLICY_PRIO2PCP_OFFS_B),
		(u8 *)(&(policy->prio_to_pcp)), MV_DP_MSG_EGRESS_QOS_POLICY_PRIO2PCP_SIZE);


	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}


enum mv_dp_rc mv_dp_ingress_qos_policy_populate_msg(void *buf, const void  *data, int index)
{
	const mv_nss_dp_ingress_qos_policy_t *policy = (const mv_nss_dp_ingress_qos_policy_t *)data  + index;
	u32 *msg_buf = (u32 *)buf;

	MV_DP_LOG_DBG3("ENTER: %s buf:%p, data:%p, index:%d\n", __func__, buf, data, index);

#ifdef MV_DP_DEBUG
	if (!policy || !buf) {
		MV_DP_LOG_ERR("populate ingress policy qos null ptr\n", MV_DP_RC_ERR_NULL_PTR);
		return MV_DP_RC_ERR_NULL_PTR;
	}
#endif


	msg_buf[MV_DP_MSG_QOS_POLICY_ID_WORD0] = cpu_to_be32(MV_DP_MSG_QOS_POLICY_ID_SET(policy->policy_id));

	msg_buf[MV_DP_MSG_INGRESS_QOS_POLICY_TYPE_WORD1] = MV_DP_MSG_INGRESS_QOS_POLICY_TYPE_SET(policy->type);
	msg_buf[MV_DP_MSG_INGRESS_QOS_POLICY_PRIO_DEF_WORD1] |=
		MV_DP_MSG_INGRESS_QOS_POLICY_PRIO_DEF_SET(policy->prio_def);
	cpu_to_be32s(&msg_buf[MV_DP_MSG_INGRESS_QOS_POLICY_TYPE_WORD1]);

	memcpy((((u8 *)(&msg_buf[MV_DP_MSG_INGRESS_QOS_POLICY_L2TOPRIO_WORD2])) +
		MV_DP_MSG_INGRESS_QOS_POLICY_L2TOPRIO_OFFS_B),
		(u8 *)(&(policy->l2_to_prio)), MV_DP_MSG_INGRESS_QOS_POLICY_L2TOPRIO_SIZE);

	memcpy((((u8 *)(&msg_buf[MV_DP_MSG_INGRESS_QOS_POLICY_DSCPTOPRIO_WORD4])) +
		MV_DP_MSG_INGRESS_QOS_POLICY_DSCPTOPRIO_OFFS_B),
		(u8 *)(&(policy->dscp_to_prio)), MV_DP_MSG_INGRESS_QOS_POLICY_DSCPTOPRIO_SIZE);

	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}

enum mv_dp_rc mv_dp_ingress_qos_policy_parse_struct(void *prio_ptr, void *buf)
{
	mv_nss_dp_ingress_qos_policy_t *policy = (mv_nss_dp_ingress_qos_policy_t *)prio_ptr;
	u32 *msg_buf = (u32 *)buf;
	u32 tmp = 0;


	MV_DP_LOG_DBG3("ENTER: %s\n", __func__);
#ifdef MV_DP_DEBUG
	if (!policy || !buf) {
		MV_DP_LOG_ERR("Null ingress qos policy:%p buf:%p\n", MV_DP_RC_ERR_NULL_PTR, policy, buf);
		return MV_DP_RC_ERR_NULL_PTR;
	}
#endif
	policy->policy_id =
		MV_DP_MSG_QOS_POLICY_ID_GET(be32_to_cpu(msg_buf[MV_DP_MSG_QOS_POLICY_ID_WORD0]));

	tmp = be32_to_cpu(msg_buf[MV_DP_MSG_INGRESS_QOS_POLICY_TYPE_WORD1]);
	policy->type = MV_DP_MSG_INGRESS_QOS_POLICY_TYPE_GET(tmp);
	policy->prio_def = MV_DP_MSG_INGRESS_QOS_POLICY_PRIO_DEF_GET(tmp);


	memcpy((u8 *)(&(policy->l2_to_prio)),
			(((u8 *)(&msg_buf[MV_DP_MSG_INGRESS_QOS_POLICY_L2TOPRIO_WORD2])) +
			MV_DP_MSG_INGRESS_QOS_POLICY_L2TOPRIO_OFFS_B), MV_DP_MSG_INGRESS_QOS_POLICY_L2TOPRIO_SIZE);

	memcpy((u8 *)(&(policy->dscp_to_prio)),
			(((u8 *)(&msg_buf[MV_DP_MSG_INGRESS_QOS_POLICY_DSCPTOPRIO_WORD4])) +
			MV_DP_MSG_INGRESS_QOS_POLICY_DSCPTOPRIO_OFFS_B), MV_DP_MSG_INGRESS_QOS_POLICY_DSCPTOPRIO_SIZE);


	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}



enum mv_dp_rc mv_dp_egress_qos_policy_parse_struct(void *prio_ptr, void *buf)
{
	mv_nss_dp_egress_qos_policy_t *policy = (mv_nss_dp_egress_qos_policy_t *)prio_ptr;
	u32 *msg_buf = (u32 *)buf;


	MV_DP_LOG_DBG3("ENTER: %s\n", __func__);
#ifdef MV_DP_DEBUG
	if (!policy || !buf) {
		MV_DP_LOG_ERR("Null egress qos policy:%p buf:%p\n", MV_DP_RC_ERR_NULL_PTR, policy, buf);
		return MV_DP_RC_ERR_NULL_PTR;
	}
#endif
	policy->policy_id =
		MV_DP_MSG_QOS_POLICY_ID_GET(be32_to_cpu(msg_buf[MV_DP_MSG_QOS_POLICY_ID_WORD0]));


	memcpy((u8 *)(&(policy->prio_to_up)),
			(((u8 *)(&msg_buf[MV_DP_MSG_EGRESS_QOS_POLICY_PRIO2UP_WORD1])) +
			MV_DP_MSG_EGRESS_QOS_POLICY_PRIO2UP_OFFS_B), MV_DP_MSG_EGRESS_QOS_POLICY_PRIO2UP_SIZE);

	memcpy((u8 *)(&(policy->prio_to_dscp)),
			(((u8 *)(&msg_buf[MV_DP_MSG_EGRESS_QOS_POLICY_PRIO2DSCP_WORD5])) +
			MV_DP_MSG_EGRESS_QOS_POLICY_PRIO2DSCP_OFFS_B), MV_DP_MSG_EGRESS_QOS_POLICY_PRIO2DSCP_SIZE);

	memcpy((u8 *)(&(policy->prio_to_pcp)),
			(((u8 *)(&msg_buf[MV_DP_MSG_EGRESS_QOS_POLICY_PRIO2PCP_WORD9])) +
			MV_DP_MSG_EGRESS_QOS_POLICY_PRIO2PCP_OFFS_B), MV_DP_MSG_EGRESS_QOS_POLICY_PRIO2PCP_SIZE);


	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}



enum mv_dp_rc mv_dp_ingress_prio_cfg_parse_struct(void *prio_ptr, void *buf)
{
	mv_nss_dp_ingress_prio_cfg_t *prio_cfg = (mv_nss_dp_ingress_prio_cfg_t *)prio_ptr;
	u32 *msg_buf = (u32 *)buf;


	MV_DP_LOG_DBG3("ENTER: %s\n", __func__);
#ifdef MV_DP_DEBUG
	if (!prio_cfg || !buf) {
		MV_DP_LOG_ERR("Null Pointer prio_cfg:%p buf:%p\n", MV_DP_RC_ERR_NULL_PTR, prio_cfg, buf);
		return MV_DP_RC_ERR_NULL_PTR;
	}
#endif
	prio_cfg->prio_mngt =
		MV_DP_MSG_INGRESS_PRIO_CFG_MNGMT_GET(be32_to_cpu(msg_buf[MV_DP_MSG_INGRESS_PRIO_CFG_MNGMT_WORD0]));

	memcpy((u8 *)(&(prio_cfg->queue)),
			(((u8 *)(&msg_buf[MV_DP_MSG_INGRESS_PRIO_CFG_MAP_WORD1])) +
			MV_DP_MSG_INGRESS_PRIO_CFG_MAP_OFFS_B), MV_DP_MSG_INGRESS_PRIO_CFG_MAP_SIZE);

	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}


enum mv_dp_rc mv_dp_ingress_prio_cfg_populate_msg(void *buf, const void *data, int index)
{
	const mv_nss_dp_ingress_prio_cfg_t *prio_cfg = (const mv_nss_dp_ingress_prio_cfg_t *)data  + index;
	u32 *msg_buf = (u32 *)buf;

	MV_DP_LOG_DBG3("ENTER: %s buf:%p, data:%p, index:%d\n", __func__, buf, data, index);

#ifdef MV_DP_DEBUG
	if (!prio_cfg || !buf) {
		MV_DP_LOG_ERR("populate ingress prio null ptr\n", MV_DP_RC_ERR_NULL_PTR);
		return MV_DP_RC_ERR_NULL_PTR;
	}
#endif

	msg_buf[MV_DP_MSG_INGRESS_PRIO_CFG_MNGMT_WORD0] =
		cpu_to_be32(MV_DP_MSG_INGRESS_PRIO_CFG_MNGMT_SET(prio_cfg->prio_mngt));


	memcpy((((u8 *)(&msg_buf[MV_DP_MSG_INGRESS_PRIO_CFG_MAP_WORD1])) + MV_DP_MSG_INGRESS_PRIO_CFG_MAP_OFFS_B),
	       (u8 *)(&(prio_cfg->queue)), MV_DP_MSG_INGRESS_PRIO_CFG_MAP_SIZE);


	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}

enum mv_dp_rc mv_dp_egress_prio_cfg_parse_struct(void *prio_ptr, void *buf)
{
	mv_nss_dp_egress_prio_cfg_t *prio_cfg = (mv_nss_dp_egress_prio_cfg_t *)prio_ptr;
	u32 *msg_buf = (u32 *)buf;


	MV_DP_LOG_DBG3("ENTER: %s\n", __func__);
#ifdef MV_DP_DEBUG
	if (!prio_cfg || !buf) {
		MV_DP_LOG_ERR("Null Pointer prio_cfg:%p buf:%p\n", MV_DP_RC_ERR_NULL_PTR, prio_cfg, buf);
		return MV_DP_RC_ERR_NULL_PTR;
	}
#endif

	memcpy((u8 *)(&(prio_cfg->queue)),
			(((u8 *)(&msg_buf[MV_DP_MSG_EGRESS_PRIO_CFG_MAP_WORD0])) +
			MV_DP_MSG_EGRESS_PRIO_CFG_MAP_OFFS_B), MV_DP_MSG_EGRESS_PRIO_CFG_MAP_SIZE);

	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}



enum mv_dp_rc mv_dp_egress_prio_cfg_populate_msg(void *buf, const void *data, int index)
{
	const mv_nss_dp_egress_prio_cfg_t *prio_cfg = (const mv_nss_dp_egress_prio_cfg_t *)data  + index;
	u32 *msg_buf = (u32 *)buf;

	MV_DP_LOG_DBG3("ENTER: %s buf:%p, data:%p, index:%d\n", __func__, buf, data, index);

#ifdef MV_DP_DEBUG
	if (!prio_cfg || !buf) {
		MV_DP_LOG_ERR("populate egress prio null ptr\n", MV_DP_RC_ERR_NULL_PTR);
		return MV_DP_RC_ERR_NULL_PTR;
	}
#endif

	memcpy((((u8 *)(&msg_buf[MV_DP_MSG_EGRESS_PRIO_CFG_MAP_WORD0])) + MV_DP_MSG_EGRESS_PRIO_CFG_MAP_OFFS_B),
	       (u8 *)(&(prio_cfg->queue)), MV_DP_MSG_EGRESS_PRIO_CFG_MAP_SIZE);


	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}


enum mv_dp_rc mv_dp_ingress_queue_cfg_parse_struct(void *queue_ptr, void *buf)
{
	mv_nss_dp_ingress_queue_cfg_t *cfg = (mv_nss_dp_ingress_queue_cfg_t *)queue_ptr;
	u32 *msg_buf = (u32 *)buf;
	u32 tmp;

	MV_DP_LOG_DBG3("ENTER: %s\n", __func__);
#ifdef MV_DP_DEBUG
	if (!cfg || !buf) {
		MV_DP_LOG_ERR("Null Pointer queue cfg:%p buf:%p\n", MV_DP_RC_ERR_NULL_PTR, cfg, buf);
		return MV_DP_RC_ERR_NULL_PTR;
	}
#endif

	cfg->queue =
	MV_DP_MSG_QUEUE_CFG_QUEUE_ID_GET(be32_to_cpu(msg_buf[MV_DP_MSG_QUEUE_CFG_QUEUE_ID_WORD0]));
	cfg->policer.cir =
	MV_DP_MSG_QUEUE_CFG_POLICER_CIR_GET(be32_to_cpu(msg_buf[MV_DP_MSG_QUEUE_CFG_POLICER_CIR_WORD1]));
	cfg->policer.eir =
	MV_DP_MSG_QUEUE_CFG_POLICER_EIR_GET(be32_to_cpu(msg_buf[MV_DP_MSG_QUEUE_CFG_POLICER_EIR_WORD2]));
	cfg->policer.cbs =
	MV_DP_MSG_QUEUE_CFG_POLICER_CBS_GET(be32_to_cpu(msg_buf[MV_DP_MSG_QUEUE_CFG_POLICER_CBS_WORD3]));
	cfg->policer.ebs =
	MV_DP_MSG_QUEUE_CFG_POLICER_EBS_GET(be32_to_cpu(msg_buf[MV_DP_MSG_QUEUE_CFG_POLICER_EBS_WORD4]));

	tmp = be32_to_cpu(msg_buf[MV_DP_MSG_QUEUE_CFG_QUEUE_PRIO_WORD5]);
	cfg->sched_priority =  MV_DP_MSG_QUEUE_CFG_QUEUE_PRIO_GET(tmp);
	cfg->sched_weight =  MV_DP_MSG_QUEUE_CFG_QUEUE_WEIGHT_GET(tmp);

/*	if (mv_nss_ingress_vq_drop_get(cfg->queue, &cfg->tail_drop_thresh, &cfg->red_thresh)) {
		MV_DP_LOG_ERR("Failed ingr vqueue drop get\n", MV_NSS_DP_FAILED);
		goto err;
	}*/

	/* OLD to be removed or check with FW ?
	if (mv_nss_ingress_vq_sched_get(cfg->queue, &cfg->sched_priority, &cfg->sched_weight)) {
		MV_DP_LOG_ERR("Failed ingr vqueue prio get\n", MV_NSS_DP_FAILED);
		goto err;
	}
	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);
	*/

	return MV_NSS_DP_OK;
}



enum mv_dp_rc mv_dp_egress_queue_cfg_parse_struct(void *queue_ptr, void *buf)
{
	mv_nss_dp_egress_queue_cfg_t *cfg = (mv_nss_dp_egress_queue_cfg_t *)queue_ptr;
	u32 *msg_buf = (u32 *)buf;


	MV_DP_LOG_DBG3("ENTER: %s\n", __func__);
#ifdef MV_DP_DEBUG
	if (!cfg || !buf) {
		MV_DP_LOG_ERR("Null Pointer queue cfg:%p buf:%p\n", MV_DP_RC_ERR_NULL_PTR, cfg, buf);
		return MV_DP_RC_ERR_NULL_PTR;
	}
#endif

	cfg->queue =
	MV_DP_MSG_QUEUE_CFG_QUEUE_ID_GET(be32_to_cpu(msg_buf[MV_DP_MSG_QUEUE_CFG_QUEUE_ID_WORD0]));
	cfg->policer.cir =
	MV_DP_MSG_QUEUE_CFG_POLICER_CIR_GET(be32_to_cpu(msg_buf[MV_DP_MSG_QUEUE_CFG_POLICER_CIR_WORD1]));
	cfg->policer.eir =
	MV_DP_MSG_QUEUE_CFG_POLICER_EIR_GET(be32_to_cpu(msg_buf[MV_DP_MSG_QUEUE_CFG_POLICER_EIR_WORD2]));
	cfg->policer.cbs =
	MV_DP_MSG_QUEUE_CFG_POLICER_CBS_GET(be32_to_cpu(msg_buf[MV_DP_MSG_QUEUE_CFG_POLICER_CBS_WORD3]));
	cfg->policer.ebs =
	MV_DP_MSG_QUEUE_CFG_POLICER_EBS_GET(be32_to_cpu(msg_buf[MV_DP_MSG_QUEUE_CFG_POLICER_EBS_WORD4]));


/*	if (mv_nss_egress_vq_size_get(cfg->queue, &cfg->tail_drop_thresh)) {
		MV_DP_LOG_ERR("Failed egr vqueue size get\n", MV_NSS_DP_FAILED);
		goto err;
	}

	if (mv_nss_egress_vq_sched_get(cfg->queue, &cfg->sched_priority, &cfg->sched_weight)) {
		MV_DP_LOG_ERR("Failed egr vqueue prio get\n", MV_NSS_DP_FAILED);
		goto err;
	}

	if (mv_nss_egress_vq_shaper_get(cfg->queue,
					&(cfg->shaper.cir),
					&(cfg->shaper.eir),
					&(cfg->shaper.cbs),
					&(cfg->shaper.ebs))) {
		MV_DP_LOG_ERR("Failed egr vqueue shaper get\n", MV_NSS_DP_FAILED);
		goto err;
	}*/

	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}

enum mv_dp_rc mv_dp_queue_stats_parse_struct(void *queue_stats_ptr, void *buf)
{
	mv_nss_dp_queue_stats_t *queue_stats = (mv_nss_dp_queue_stats_t *)queue_stats_ptr;
	u32 *msg_buf = (u32 *)buf;
	uint64_t cnt_ptr;


	MV_DP_LOG_DBG3("ENTER: %s\n", __func__);
#ifdef MV_DP_DEBUG
	if (!queue_stats || !buf) {
		MV_DP_LOG_ERR("Null queue stats ptr:%p buf:%p\n", MV_DP_RC_ERR_NULL_PTR, queue_stats, buf);
		return MV_DP_RC_ERR_NULL_PTR;
	}
#endif
	queue_stats->queue =
		MV_DP_MSG_Q_STATS_QUEUE_ID_GET(be32_to_cpu(msg_buf[MV_DP_MSG_Q_STATS_QUEUE_ID_WORD0]));

	cnt_ptr = *(uint64_t *)(((u8 *)&msg_buf[MV_DP_MSG_Q_STATS_PKTS_WORD1]) +
				MV_DP_MSG_Q_STATS_PKTS_OFFS);
	queue_stats->pkts = be64_to_cpu(cnt_ptr);

	cnt_ptr = *(uint64_t *)(((u8 *)&msg_buf[MV_DP_MSG_Q_STATS_ERR_WORD3]) +
				MV_DP_MSG_Q_STATS_ERR_OFFS);
	queue_stats->errors = be64_to_cpu(cnt_ptr);

	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}

enum mv_dp_rc mv_dp_queue_id_cfg_populate_msg(void *buf, const void *data, int index)
{
	const uint8_t q_id = *((const uint8_t *)data  + index);
	u32 *msg_buf = (u32 *)buf;

	MV_DP_LOG_DBG3("ENTER: %s buf:%p, data:%p, index:%d\n", __func__, buf, data, index);

	if (!data || !buf)
		return MV_NSS_DP_FAILED;

	msg_buf[MV_DP_MSG_QUEUE_CFG_QUEUE_ID_WORD0] = cpu_to_be32(MV_DP_MSG_QUEUE_CFG_QUEUE_ID_SET(q_id));

	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}


enum mv_dp_rc mv_dp_egress_queue_cfg_populate_msg(void *buf, const void *data, int index)
{
	const mv_nss_dp_egress_queue_cfg_t *cfg = ((const mv_nss_dp_egress_queue_cfg_t *)data  + index);
	u32 *msg_buf = (u32 *)buf;

	MV_DP_LOG_DBG3("ENTER: %s buf:%p, data:%p, index:%d\n", __func__, buf, data, index);

	if (!data || !buf)
		return MV_NSS_DP_FAILED;

	msg_buf[MV_DP_MSG_QUEUE_CFG_QUEUE_ID_WORD0] =
		cpu_to_be32(MV_DP_MSG_QUEUE_CFG_QUEUE_ID_SET(cfg->queue));
	msg_buf[MV_DP_MSG_QUEUE_CFG_POLICER_CIR_WORD1] =
		cpu_to_be32(MV_DP_MSG_QUEUE_CFG_POLICER_CIR_SET(cfg->policer.cir));
	msg_buf[MV_DP_MSG_QUEUE_CFG_POLICER_EIR_WORD2] =
		cpu_to_be32(MV_DP_MSG_QUEUE_CFG_POLICER_EIR_SET(cfg->policer.eir));
	msg_buf[MV_DP_MSG_QUEUE_CFG_POLICER_CBS_WORD3] =
		cpu_to_be32(MV_DP_MSG_QUEUE_CFG_POLICER_CBS_SET(cfg->policer.cbs));
	msg_buf[MV_DP_MSG_QUEUE_CFG_POLICER_EBS_WORD4] =
		cpu_to_be32(MV_DP_MSG_QUEUE_CFG_POLICER_EBS_SET(cfg->policer.ebs));

	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}

enum mv_dp_rc mv_dp_ingress_queue_cfg_populate_msg(void *buf, const void *data, int index)
{
	const mv_nss_dp_ingress_queue_cfg_t *cfg = ((const mv_nss_dp_ingress_queue_cfg_t *)data  + index);
	u32 *msg_buf = (u32 *)buf;

	MV_DP_LOG_DBG3("ENTER: %s buf:%p, data:%p, index:%d\n", __func__, buf, data, index);

	if (!data || !buf)
		return MV_NSS_DP_FAILED;

	msg_buf[MV_DP_MSG_QUEUE_CFG_QUEUE_ID_WORD0] =
		cpu_to_be32(MV_DP_MSG_QUEUE_CFG_QUEUE_ID_SET(cfg->queue));
	msg_buf[MV_DP_MSG_QUEUE_CFG_POLICER_CIR_WORD1] =
		cpu_to_be32(MV_DP_MSG_QUEUE_CFG_POLICER_CIR_SET(cfg->policer.cir));
	msg_buf[MV_DP_MSG_QUEUE_CFG_POLICER_EIR_WORD2] =
		cpu_to_be32(MV_DP_MSG_QUEUE_CFG_POLICER_EIR_SET(cfg->policer.eir));
	msg_buf[MV_DP_MSG_QUEUE_CFG_POLICER_CBS_WORD3] =
		cpu_to_be32(MV_DP_MSG_QUEUE_CFG_POLICER_CBS_SET(cfg->policer.cbs));
	msg_buf[MV_DP_MSG_QUEUE_CFG_POLICER_EBS_WORD4] =
		cpu_to_be32(MV_DP_MSG_QUEUE_CFG_POLICER_EBS_SET(cfg->policer.ebs));

	msg_buf[MV_DP_MSG_QUEUE_CFG_QUEUE_PRIO_WORD5] = MV_DP_MSG_QUEUE_CFG_QUEUE_PRIO_SET(cfg->sched_priority);
	msg_buf[MV_DP_MSG_QUEUE_CFG_QUEUE_WEIGHT_WORD5] |= MV_DP_MSG_QUEUE_CFG_QUEUE_WEIGHT_SET(cfg->sched_weight);
	cpu_to_be32s(&msg_buf[MV_DP_MSG_QUEUE_CFG_QUEUE_PRIO_WORD5]);


	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}

/*************************QOS END*******************************************/




/*************************FLOW START****************************************/

enum mv_dp_rc mv_dp_flow_parse_struct(void *flow_ptr, void *buf)
{
	mv_nss_dp_flow_t *flow = (mv_nss_dp_flow_t *)flow_ptr;
	u32 *msg_buf = (u32 *)buf;


	MV_DP_LOG_DBG3("ENTER: %s\n", __func__);
#ifdef MV_DP_DEBUG
	if (!flow || !buf) {
		MV_DP_LOG_ERR("Null Pointer flow:%p buf:%p\n", MV_DP_RC_ERR_NULL_PTR, flow, buf);
		return MV_DP_RC_ERR_NULL_PTR;
	}
#endif
	flow->flow_id = MV_DP_MSG_FLOW_ID_GET(be32_to_cpu(msg_buf[MV_DP_MSG_FLOW_ID_WORD0]));
	flow->status = MV_DP_MSG_FLOW_STATUS_GET(be32_to_cpu(msg_buf[MV_DP_MSG_FLOW_STATUS_WORD1]));
	flow->idle_timeout = MV_DP_MSG_FLOW_IDLE_GET(be32_to_cpu(msg_buf[MV_DP_MSG_FLOW_IDLE_WORD2]));

	mv_dp_flow_param_parse_struct(&flow->cls, &msg_buf[MV_DP_MSG_FLOW_CLS_BASE_WORD3]);
	mv_dp_flow_param_parse_struct(&flow->act, &msg_buf[MV_DP_MSG_FLOW_ACT_BASE_WORD23]);

	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}



enum mv_dp_rc mv_dp_flow_param_parse_struct(mv_nss_dp_params_t *param, void *buf)
{
	u32 *msg_buf = (u32 *)buf;
	u32 tmp = 0;
	int ip_ver = 0;

	MV_DP_LOG_DBG3("ENTER: %s\n", __func__);
#ifdef MV_DP_DEBUG
	if (!param || !buf) {
		MV_DP_LOG_ERR("Null Pointer param:%p buf:%p\n", MV_DP_RC_ERR_NULL_PTR, param, buf);
		return MV_DP_RC_ERR_NULL_PTR;
	}
#endif
	/*+W0*/
	param->present = MV_DP_MSG_FLOW_PRESENT_GET(be32_to_cpu(msg_buf[MV_DP_MSG_FLOW_PRESENT_WORD0]));

	/*+W1*/
	param->flbl = MV_DP_MSG_FLOW_FLBL_GET(be32_to_cpu(msg_buf[MV_DP_MSG_FLOW_FLBL_WORD1]));

	/*+W2*/
	param->opaque = MV_DP_MSG_FLOW_OPAQUE_GET(be32_to_cpu(msg_buf[MV_DP_MSG_FLOW_OPAQUE_WORD2]));

	/*W3*/
	tmp = be32_to_cpu(msg_buf[MV_DP_MSG_FLOW_ETH_TYPE_WORD3]);
	param->eth_type = MV_DP_MSG_FLOW_ETH_TYPE_GET(tmp);
	param->port_dst = MV_DP_MSG_FLOW_PORT_DST_GET(tmp);
	param->port_src = MV_DP_MSG_FLOW_PORT_SRC_GET(tmp);

	/*W4*/
	tmp = be32_to_cpu(msg_buf[MV_DP_MSG_FLOW_LLC_SSAP_WORD4]);
	param->llc_ssap = MV_DP_MSG_FLOW_LLC_SSAP_GET(tmp);
	param->llc_dsap = MV_DP_MSG_FLOW_LLC_DSAP_GET(tmp);

	/*W5*/
	tmp = be32_to_cpu(msg_buf[MV_DP_MSG_FLOW_L4_DST_WORD5]);
	param->l4_port_dst = MV_DP_MSG_FLOW_L4_DST_GET(tmp);
	param->l4_port_src = MV_DP_MSG_FLOW_L4_SRC_GET(tmp);

	/*W6*/
	tmp = be32_to_cpu(msg_buf[MV_DP_MSG_FLOW_PRIO_VAL_WORD6]);
	param->prio_val = MV_DP_MSG_FLOW_PRIO_VAL_GET(tmp);
	param->prio_action = MV_DP_MSG_FLOW_PRIO_ACT_GET(tmp);
	param->l4_proto = MV_DP_MSG_FLOW_L4_PROTO_GET(tmp);
	ip_ver =  MV_DP_MSG_FLOW_IPVER_GET(tmp);

#ifdef MV_DP_DEBUG
	if (!(ip_ver == 4 || ip_ver == 6) &&
		(param->present & MV_NSS_DP_IP_DST) && (param->present & MV_NSS_DP_IP_SRC)) {
			MV_DP_LOG_ERR("ILLEGAL IP VER:%d\n", MV_DP_RC_ERR_INVALID_PARAM, ip_ver);
			ip_ver = 16;
	}
#endif
	param->ip_addr_src.ver = param->ip_addr_dst.ver = ip_ver;
	if (6 == ip_ver)
		ip_ver = 16;

	memcpy((void *)(&(param->ip_addr_dst.ip)),
			(((u8 *)&(msg_buf[MV_DP_MSG_FLOW_IP_DST_WORD9])) +
			MV_DP_MSG_FLOW_IP_DST_OFFS_B), ip_ver);
	memcpy((void *)(&(param->ip_addr_src.ip)),
			(((u8 *)&(msg_buf[MV_DP_MSG_FLOW_IP_SRC_WORD13])) +
			MV_DP_MSG_FLOW_IP_SRC_OFFS_B), ip_ver);


	memcpy((u8 *)(&(param->l2_addr_dst.addr)),
			(((u8 *)(&msg_buf[MV_DP_MSG_FLOW_L2_DST_WORD17])) + MV_DP_MSG_FLOW_L2_DST_OFFS_B),
			MV_DP_MSG_FLOW_L2_SIZE);
	memcpy((u8 *)(&(param->l2_addr_src.addr)),
			(((u8 *)(&msg_buf[MV_DP_MSG_FLOW_L2_SRC_WORD18])) + MV_DP_MSG_FLOW_L2_SRC_OFFS_B),
			MV_DP_MSG_FLOW_L2_SIZE);

	memcpy((u8 *)(&(param->snap)),
			(((u8 *)(&msg_buf[MV_DP_MSG_FLOW_SNAP_WORD7])) + MV_DP_MSG_FLOW_SNAP_OFFS_B),
		       MV_DP_MSG_FLOW_SNAP_SIZE);


	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}


enum mv_dp_rc mv_dp_flow_populate_msg(void *buf, const void *data, int index)
{
	const mv_nss_dp_flow_t *flow = (const mv_nss_dp_flow_t *)data  + index;
	u32 *msg_buf = (u32 *)buf;

	MV_DP_LOG_DBG3("ENTER: %s buf:%p, data:%p, index:%d\n", __func__, buf, data, index);

#ifdef MV_DP_DEBUG
	if (!flow || !buf) {
		MV_DP_LOG_ERR("populate flow null ptr\n", MV_DP_RC_ERR_NULL_PTR);
		return MV_DP_RC_ERR_NULL_PTR;
	}
#endif

	msg_buf[MV_DP_MSG_FLOW_ID_WORD0] = cpu_to_be32(MV_DP_MSG_FLOW_ID_SET(flow->flow_id));
	msg_buf[MV_DP_MSG_FLOW_STATUS_WORD1] = cpu_to_be32(MV_DP_MSG_FLOW_STATUS_SET((flow->status ? 1 : 0)));
	msg_buf[MV_DP_MSG_FLOW_IDLE_WORD2] = cpu_to_be32(MV_DP_MSG_FLOW_IDLE_SET(flow->idle_timeout));

	mv_dp_flow_populate_param_msg(&msg_buf[MV_DP_MSG_FLOW_CLS_BASE_WORD3], &flow->cls);
	mv_dp_flow_populate_param_msg(&msg_buf[MV_DP_MSG_FLOW_ACT_BASE_WORD23], &flow->act);

	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}


enum mv_dp_rc mv_dp_flow_status_populate_msg(void *buf, const void *data, int index)
{
	const mv_nss_dp_flow_status_t *flow_status = (const mv_nss_dp_flow_status_t *)data  + index;
	u32 *msg_buf = (u32 *)buf;

	MV_DP_LOG_DBG3("ENTER: %s buf:%p, data:%p, index:%d\n", __func__, buf, data, index);

#ifdef MV_DP_DEBUG
	if (!flow_status || !buf) {
		MV_DP_LOG_ERR("populate flow_status null ptr\n", MV_DP_RC_ERR_NULL_PTR);
		return MV_DP_RC_ERR_NULL_PTR;
	}
#endif

	msg_buf[MV_DP_MSG_FLOW_ID_WORD0] = cpu_to_be32(MV_DP_MSG_FLOW_ID_SET(flow_status->flow_id));
	msg_buf[MV_DP_MSG_FLOW_STATUS_WORD1] = cpu_to_be32(MV_DP_MSG_FLOW_STATUS_SET((flow_status->status ? 1 : 0)));

	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}

enum mv_dp_rc mv_dp_flow_id_populate_msg(void *buf, const void *data, int index)
{
	const mv_nss_dp_flow_id_t *flow_id = (const mv_nss_dp_flow_id_t *)data  + index;
	u32 *msg_buf = (u32 *)buf;

	MV_DP_LOG_DBG3("ENTER: %s buf:%p, data:%p, index:%d\n", __func__, buf, data, index);
#ifdef MV_DP_DEBUG
	if (!flow_id || !buf) {
		MV_DP_LOG_ERR("populate flow_id null ptr\n", MV_DP_RC_ERR_NULL_PTR);
		return MV_DP_RC_ERR_NULL_PTR;
	}
#endif

	msg_buf[MV_DP_MSG_FLOW_ID_WORD0] = cpu_to_be32(MV_DP_MSG_FLOW_ID_SET(*flow_id));

	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}


enum mv_dp_rc mv_dp_flow_populate_param_msg(void *buf, const mv_nss_dp_params_t *const param)
{
	u32 *msg_buf = (u32 *)buf;

	MV_DP_LOG_DBG3("ENTER: %s buf:%p, data:%p\n", __func__, buf, param);


#ifdef MV_DP_DEBUG
	if (!param || !buf) {
		MV_DP_LOG_ERR("populate param null ptr\n", MV_DP_RC_ERR_NULL_PTR);
		return MV_DP_RC_ERR_NULL_PTR;
	}
#endif

/*W0*/
	msg_buf[MV_DP_MSG_FLOW_PRESENT_WORD0] = cpu_to_be32(MV_DP_MSG_FLOW_PRESENT_SET(param->present));
/*W1*/
	msg_buf[MV_DP_MSG_FLOW_FLBL_WORD1] = cpu_to_be32(MV_DP_MSG_FLOW_FLBL_SET(param->flbl));
/*W2*/
	msg_buf[MV_DP_MSG_FLOW_OPAQUE_WORD2] = cpu_to_be32(MV_DP_MSG_FLOW_OPAQUE_SET(param->opaque));
/*W3*/
	msg_buf[MV_DP_MSG_FLOW_ETH_TYPE_WORD3] = MV_DP_MSG_FLOW_ETH_TYPE_SET(param->eth_type);
	msg_buf[MV_DP_MSG_FLOW_PORT_DST_WORD3] |= MV_DP_MSG_FLOW_PORT_DST_SET(param->port_dst);
	msg_buf[MV_DP_MSG_FLOW_PORT_SRC_WORD3] |= MV_DP_MSG_FLOW_PORT_SRC_SET(param->port_src);
	cpu_to_be32s(&msg_buf[MV_DP_MSG_FLOW_ETH_TYPE_WORD3]);
/*W4*/
	msg_buf[MV_DP_MSG_FLOW_LLC_SSAP_WORD4] = MV_DP_MSG_FLOW_LLC_SSAP_SET(param->llc_ssap);
	msg_buf[MV_DP_MSG_FLOW_LLC_DSAP_WORD4] |= MV_DP_MSG_FLOW_LLC_DSAP_SET(param->llc_dsap);
	cpu_to_be32s(&msg_buf[MV_DP_MSG_FLOW_LLC_SSAP_WORD4]);
/*W5*/
	msg_buf[MV_DP_MSG_FLOW_L4_DST_WORD5] = MV_DP_MSG_FLOW_L4_DST_SET(param->l4_port_dst);
	msg_buf[MV_DP_MSG_FLOW_L4_SRC_WORD5] |= MV_DP_MSG_FLOW_L4_SRC_SET(param->l4_port_src);
	cpu_to_be32s(&msg_buf[MV_DP_MSG_FLOW_L4_DST_WORD5]);
/*W6*/
	msg_buf[MV_DP_MSG_FLOW_PRIO_VAL_WORD6] = MV_DP_MSG_FLOW_PRIO_VAL_SET(param->prio_val);
	msg_buf[MV_DP_MSG_FLOW_PRIO_ACT_WORD6] |= MV_DP_MSG_FLOW_PRIO_ACT_SET(param->prio_action);
	msg_buf[MV_DP_MSG_FLOW_L4_PROTO_WORD6] |= MV_DP_MSG_FLOW_L4_PROTO_SET(param->l4_proto);
	msg_buf[MV_DP_MSG_FLOW_IPVER_WORD6] |= MV_DP_MSG_FLOW_IPVER_SET(param->ip_addr_dst.ver);
	cpu_to_be32s(&msg_buf[MV_DP_MSG_FLOW_PRIO_VAL_WORD6]);
	if ((param->ip_addr_dst.ver != param->ip_addr_src.ver) &&
	    (param->present & MV_NSS_DP_IP_DST) && (param->present & MV_NSS_DP_IP_SRC)) {
		MV_DP_LOG_INF("Warning SRC and DST IP ver is different!: %d %d assuming:%d\n",
			       param->ip_addr_dst.ver,
			       param->ip_addr_src.ver,
			       param->ip_addr_dst.ver);
	}

	msg_buf[MV_DP_MSG_FLOW_RFU1_WORD8] = 0;

	memcpy((((u8 *)(&msg_buf[MV_DP_MSG_FLOW_SNAP_WORD7])) + MV_DP_MSG_FLOW_SNAP_OFFS_B),
	       (u8 *)(&(param->snap)), MV_DP_MSG_FLOW_SNAP_SIZE);

	memcpy((((u8 *)(&msg_buf[MV_DP_MSG_FLOW_IP_DST_WORD9])) + MV_DP_MSG_FLOW_IP_DST_OFFS_B),
	       (u8 *)(&(param->ip_addr_dst)), MV_DP_MSG_FLOW_IP_DST_SIZE);

	memcpy((((u8 *)(&msg_buf[MV_DP_MSG_FLOW_IP_SRC_WORD13])) + MV_DP_MSG_FLOW_IP_SRC_OFFS_B),
	       (u8 *)(&(param->ip_addr_src)), MV_DP_MSG_FLOW_IP_SRC_SIZE);

	memcpy((((u8 *)(&msg_buf[MV_DP_MSG_FLOW_L2_DST_WORD17])) + MV_DP_MSG_FLOW_L2_DST_OFFS_B),
	       (u8 *)(&(param->l2_addr_dst.addr)), MV_DP_MSG_FLOW_L2_SIZE);

	memcpy((((u8 *)(&msg_buf[MV_DP_MSG_FLOW_L2_SRC_WORD18])) + MV_DP_MSG_FLOW_L2_SRC_OFFS_B),
	       (u8 *)(&(param->l2_addr_src.addr)), MV_DP_MSG_FLOW_L2_SIZE);

	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}


enum mv_dp_rc mv_dp_flow_count_parse_struct(void *count_ptr, void *buf)
{
	uint32_t *count = (uint32_t *)count_ptr;
	u32 *msg_buf = (u32 *)buf;


	MV_DP_LOG_DBG3("ENTER: %s\n", __func__);
#ifdef MV_DP_DEBUG
	if (!count || !buf) {
		MV_DP_LOG_ERR("Null Pointer flow:%p buf:%p\n", MV_DP_RC_ERR_NULL_PTR, count, buf);
		return MV_DP_RC_ERR_NULL_PTR;
	}
#endif

	*count = MV_DP_MSG_FLOW_COUNT_GET(be32_to_cpu(msg_buf[MV_DP_MSG_FLOW_COUNT_WORD0]));

	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}

enum mv_dp_rc mv_dp_flow_status_parse_struct(void *flow_status_ptr, void *buf)
{
	mv_nss_dp_flow_status_t *flow_status = (mv_nss_dp_flow_status_t *)flow_status_ptr;
	u32 *msg_buf = (u32 *)buf;

	MV_DP_LOG_DBG3("ENTER: %s\n", __func__);
#ifdef MV_DP_DEBUG
	if (!flow_status || !buf) {
		MV_DP_LOG_ERR("Null flow status ptr:%p buf:%p\n", MV_DP_RC_ERR_NULL_PTR, flow_status, buf);
		return MV_DP_RC_ERR_NULL_PTR;
	}
#endif
	flow_status->flow_id = MV_DP_MSG_FLOW_ID_GET(be32_to_cpu(msg_buf[MV_DP_MSG_FLOW_ID_WORD0]));
	flow_status->status = MV_DP_MSG_FLOW_STATUS_GET(be32_to_cpu(msg_buf[MV_DP_MSG_FLOW_STATUS_WORD1]));

	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}


enum mv_dp_rc mv_dp_flow_stats_parse_struct(void *flow_stats_ptr, void *buf)
{
	mv_nss_dp_flow_stats_t *flow_stats = (mv_nss_dp_flow_stats_t *)flow_stats_ptr;
	u32 *msg_buf = (u32 *)buf;
	uint64_t cnt_ptr;


	MV_DP_LOG_DBG3("ENTER: %s\n", __func__);
#ifdef MV_DP_DEBUG
	if (!flow_stats || !buf) {
		MV_DP_LOG_ERR("Null flow stats ptr:%p buf:%p\n", MV_DP_RC_ERR_NULL_PTR, flow_stats, buf);
		return MV_DP_RC_ERR_NULL_PTR;
	}
#endif
	flow_stats->flow_id =
		MV_DP_MSG_FLOW_ID_GET(be32_to_cpu(msg_buf[MV_DP_MSG_FLOW_ID_WORD0]));
	flow_stats->rx_last_ts =
		MV_DP_MSG_FLOW_STATS_LAST_TS_GET(be32_to_cpu(msg_buf[MV_DP_MSG_FLOW_STATS_LAST_TS_WORD1]));

	cnt_ptr = *(uint64_t *)(((u8 *)&msg_buf[MV_DP_MSG_FLOW_STATS_RX_PKTS_WORD2]) +
				MV_DP_MSG_FLOW_STATS_RX_PKTS_OFFS_B);
	flow_stats->rx_pkts = be64_to_cpu(cnt_ptr);

	cnt_ptr = *(uint64_t *)(((u8 *)&msg_buf[MV_DP_MSG_FLOW_STATS_RX_OCTS_WORD4]) +
				MV_DP_MSG_FLOW_STATS_RX_OCTS_OFFS_B);
	flow_stats->rx_octets = be64_to_cpu(cnt_ptr);

	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}
/*************************FLOW END******************************************/

/*************************CLIENT START**************************************/

/*populates buffer from msg to client structure*/
enum mv_dp_rc mv_dp_client_parse_struct(void *client_ptr, void *buf)
{
	mv_nss_dp_client_t *client = (mv_nss_dp_client_t *)client_ptr;
	u32 *msg_buf = (u32 *)buf;


	MV_DP_LOG_DBG3("ENTER: %s\n", __func__);
#ifdef MV_DP_DEBUG
	if (!client || !buf) {
		MV_DP_LOG_ERR("Null Pointer client:%p buf:%p\n", MV_DP_RC_ERR_NULL_PTR, client, buf);
		return MV_DP_RC_ERR_NULL_PTR;
	}
#endif
	memcpy((void *)(&(client->l2_addr.addr)),
	       (void *)(&(msg_buf[MV_DP_MSG_CLIENT_MAC_WORD0])),
	       MV_DP_MSG_CLIENT_MAC_SIZE);


	/*word 3*/
	be32_to_cpus(&msg_buf[MV_DP_MSG_CLIENT_BSS_WORD3]);

	client->vlan = MV_DP_MSG_CLIENT_VLAN_GET(msg_buf[MV_DP_MSG_CLIENT_VLAN_WORD3]);
	client->prio_policy = MV_DP_MSG_CLIENT_PRIO_GET(msg_buf[MV_DP_MSG_CLIENT_PRIO_WORD3]);
	client->qcf_needed = MV_DP_MSG_CLIENT_QCF_GET(msg_buf[MV_DP_MSG_CLIENT_QCF_WORD3]);
	client->radio_id = MV_DP_MSG_CLIENT_RADIO_GET(msg_buf[MV_DP_MSG_CLIENT_RADIO_WORD3]);
	client->is_my_mac = MV_DP_MSG_CLIENT_MYMAC_GET(msg_buf[MV_DP_MSG_CLIENT_MYMAC_WORD3]);
	client->bssid = MV_DP_MSG_CLIENT_BSS_GET(msg_buf[MV_DP_MSG_CLIENT_BSS_WORD3]);


	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}


enum mv_dp_rc mv_dp_client_populate_msg(void *buf, const void  *data, int index)
{
	const mv_nss_dp_client_t *client = (const mv_nss_dp_client_t *)data  + index;
	u32 *msg_buf = (u32 *)buf;

	MV_DP_LOG_DBG3("ENTER: %s buf:%p, data:%p, index:%d\n", __func__, buf, data, index);

	if (!client || !buf)
		return MV_NSS_DP_FAILED;

	msg_buf[MV_DP_MSG_CLIENT_RFU1_WORD1] = 0;

	memcpy((void *)(&(msg_buf[MV_DP_MSG_CLIENT_MAC_WORD0])), (void *)(&(client->l2_addr.addr)),
	       MV_DP_MSG_CLIENT_MAC_SIZE);


	msg_buf[MV_DP_MSG_CLIENT_VLAN_WORD3] = MV_DP_MSG_CLIENT_VLAN_SET(client->vlan);
	msg_buf[MV_DP_MSG_CLIENT_PRIO_WORD3] |= MV_DP_MSG_CLIENT_PRIO_SET(client->prio_policy);
	msg_buf[MV_DP_MSG_CLIENT_QCF_WORD3] |= MV_DP_MSG_CLIENT_QCF_SET(client->qcf_needed);
	msg_buf[MV_DP_MSG_CLIENT_RADIO_WORD3] |= MV_DP_MSG_CLIENT_RADIO_SET(client->radio_id);
	msg_buf[MV_DP_MSG_CLIENT_MYMAC_WORD3] |= MV_DP_MSG_CLIENT_MYMAC_SET(client->is_my_mac);
	msg_buf[MV_DP_MSG_CLIENT_BSS_WORD3] |= MV_DP_MSG_CLIENT_BSS_SET(client->bssid);

	cpu_to_be32s(&msg_buf[MV_DP_MSG_CLIENT_BSS_WORD3]);

	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}


enum mv_dp_rc mv_dp_client_ind_populate_msg(void *buf, const void *data, int index)
{
	u16 client_index = *((u16 *)data  + index);
	u32 *msg_buf = (u32 *)buf;

	MV_DP_LOG_DBG3("ENTER: %s client_index:%d, data:%p, index:%d\n", __func__, client_index, data, index);

	if (!buf || !data)
		return MV_NSS_DP_FAILED;

	msg_buf[MV_DP_MSG_CLIENT_IND_WORD0] = cpu_to_be32(MV_DP_MSG_CLIENT_IND_SET(client_index));

	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}

enum mv_dp_rc mv_dp_client_mac_populate_msg(void *buf, const void  *data, int index)
{
	const mv_nss_dp_l2_addr_t *l2 = (const mv_nss_dp_l2_addr_t *)data  + index;

	MV_DP_LOG_DBG3("ENTER: %s buf:%p, data:%p, index:%d\n", __func__, buf, data, index);

	if (!buf || !data)
		return MV_NSS_DP_FAILED;

	memcpy(buf, &(l2->addr), MV_DP_MSG_CLIENT_MAC_SIZE);


	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}

/********************CLIENT END**********************************************/


/**********************INIT START*****************************************************************/
enum mv_dp_rc mv_dp_init_populate_msg(void *buf, const void *const data, int index)
{
	const mv_nss_dp_init_cfg_t *cfg = ((const mv_nss_dp_init_cfg_t *)data  + index);
	u32 *msg_buf = (u32 *)buf;

	MV_DP_LOG_DBG3("ENTER: %s buf:%p, data:%p, index:%d\n", __func__, buf, data, index);


	if (!data || !buf)
		return MV_NSS_DP_FAILED;

	msg_buf[MV_DP_MSG_INIT_DST_PORT_ID_WORD0] = cpu_to_be32(MV_DP_MSG_INIT_DST_PORT_ID_SET(cfg->port_dst));

	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_DP_RC_OK;
}
/**********************INIT END*****************************************************************/

enum mv_dp_rc mv_dp_ctx_populate_msg(void *buf, const void *data, int index)
{
	const struct mv_dp_ctx *ctx = (const struct mv_dp_ctx *)data  + index;
	u32		*msg_buf = (u32 *)buf;


	MV_DP_LOG_DBG3("ENTER: %s buf:%p, data:%p, index:%d\n", __func__, buf, data, index);

	if (!data || !buf)
		return MV_NSS_DP_FAILED;

	MV_DP_LOG_DBG3("%s opcode:%u, cnt:%d, index:%d\n", __func__, ctx->opc, ctx->cnt, index);
	msg_buf[MV_DP_MSG_CTX_OPC_WORD0] = MV_DP_MSG_CTX_OPC_SET(ctx->opc);
	msg_buf[MV_DP_MSG_CTX_CNT_WORD0] |= MV_DP_MSG_CTX_CNT_SET(ctx->cnt);
	cpu_to_be32s(&msg_buf[MV_DP_MSG_CTX_OPC_WORD0]);

	MV_DP_LOG_DBG3("EXIT: %s %08X\n", __func__, msg_buf[MV_DP_MSG_CTX_OPC_WORD0]);

	return MV_NSS_DP_OK;
}

enum mv_dp_rc mv_dp_ctx_parse_msg(void *msg_ptr, void *buf)
{
	struct mv_dp_msg_info_rx *rx_msg = (struct mv_dp_msg_info_rx *)msg_ptr;
	u16 opcode;
	u16 cnt;
	u32			tmp;


	MV_DP_LOG_DBG3("ENTER: %s msg_ptr:%p\n", __func__, msg_ptr);

	if (!msg_ptr) {
		MV_DP_LOG_ERR("Null Pointer msg_ptr:%p buf:%p\n", MV_DP_RC_ERR_NULL_PTR, msg_ptr, buf);
		return MV_DP_RC_ERR_NULL_PTR;
	}
	/*correct the opcode based on type*/

	tmp = be32_to_cpu(*((u32 *)rx_msg->msg_data + MV_DP_MSG_CTX_OPC_WORD0));
	opcode = MV_DP_MSG_CTX_OPC_GET(tmp);
	cnt = MV_DP_MSG_CTX_CNT_GET(tmp);
	MV_DP_LOG_DBG2("%s, Got opcode:%d cnt:%d\n", __func__, opcode, cnt);


	switch (opcode) {
	case (MV_DP_MSGID_PORT_LINK_GET):
		rx_msg->opcode = MV_DP_MSGID_PORT_LINK_GET;
		break;
	case (MV_DP_MSGID_NOP):
		rx_msg->opcode = MV_DP_MSGID_NOP;
		break;

	default:
		MV_DP_LOG_ERR("NOT SUPPORTED OPCODE:%d\n", MV_DP_RC_ERR_MSG_PARSE_FAILED, opcode);
		return MV_DP_RC_ERR_MSG_PARSE_FAILED;
	}

	rx_msg->num_ok = cnt;
	MV_DP_LOG_DBG3("%s, opcode:%d num_ok:%d\n", __func__, rx_msg->opcode, rx_msg->num_ok);
	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_DP_RC_OK;
}


enum mv_dp_rc mv_dp_nop_parse_msg(void *dest, void  *src)
{
	u32 *msg_out = (u32 *)dest;
	u32 *msg_buf = (u32 *)src;

	MV_DP_LOG_DBG3("ENTER: %s dest:%p, src:%p\n", __func__, dest, src);

	if (!msg_out || !msg_buf) {
		MV_DP_LOG_ERR("Null msg_out:%p or msg_buf:%p\n", MV_DP_RC_ERR_NULL_PTR, msg_out, msg_buf);
		return MV_NSS_DP_FAILED;
	}

	msg_out[MV_DP_MSG_NOP_COOKIE_WORD0] =
				MV_DP_MSG_NOP_COOKIE_GET(be32_to_cpu(msg_buf[MV_DP_MSG_NOP_COOKIE_WORD0]));

	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}

enum mv_dp_rc mv_dp_nop_populate_msg(void *buf, const void  *data, int index)
{
	u32 cookie = *(u32 *)data  + index;
	u32 *msg_buf = (u32 *)buf;

	MV_DP_LOG_DBG3("ENTER: %s buf:%p, data:%p, index:%d\n", __func__, buf, data, index);

	if (!buf)
		return MV_NSS_DP_FAILED;

	msg_buf[MV_DP_MSG_NOP_COOKIE_WORD0] = cpu_to_be32(MV_DP_MSG_NOP_COOKIE_SET(cookie));

	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}

enum mv_dp_rc mv_dp_ver_parse_msg(void *dest, void  *src)
{
	u32 *msg_out = (u32 *)dest;
	u32 *msg_buf = (u32 *)src;

	MV_DP_LOG_DBG3("ENTER: %s dest:%p, src:%p\n", __func__, dest, src);

	if (!msg_out || !msg_buf) {
		MV_DP_LOG_ERR("Null msg_out:%p or msg_buf:%p\n", MV_DP_RC_ERR_NULL_PTR, msg_out, msg_buf);
		return MV_NSS_DP_FAILED;
	}

	msg_out[0] = be32_to_cpu(msg_buf[MV_DP_MSG_VER_MAJ_WORD0]);

	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}

/***************************************HASH START************************************************/
enum mv_dp_rc mv_dp_hash_prof_populate_msg(void *buf, const void *data, int index)
{
	const mv_nss_dp_hash_profile_t *prof_type = (const mv_nss_dp_hash_profile_t *)data  + index;

	u32		*msg_buf = (u32 *)buf;
	int		i;

	MV_DP_LOG_DBG3("ENTER: %s buf:%p, data:%p, index:%d\n", __func__, buf, data, index);

	if (!data || !buf)
		return MV_NSS_DP_FAILED;

	msg_buf[MV_DP_MSG_HASH_PROF_PROF_ID_WORD0] =
		cpu_to_be32(MV_DP_MSG_HASH_PROF_PROF_ID_SET(prof_type->profile_id));
	msg_buf[MV_DP_MSG_HASH_PROF_TCP_UDP_BM_WORD1] =
		cpu_to_be32(MV_DP_MSG_HASH_PROF_TCP_UDP_BM_SET(prof_type->tcp_udp));
	msg_buf[MV_DP_MSG_HASH_PROF_IP_BM_WORD2] =
		cpu_to_be32(MV_DP_MSG_HASH_PROF_IP_BM_SET(prof_type->ip));
	msg_buf[MV_DP_MSG_HASH_PROF_NON_IP_BM_WORD3] =
		cpu_to_be32(MV_DP_MSG_HASH_PROF_NON_IP_BM_SET(prof_type->non_ip));

	for (i = 0; i < MV_NSS_DP_HASH_KNEES; i++)
		msg_buf[MV_DP_MSG_HASH_PROF_RESULT_WORD_GET(i)] =
			cpu_to_be32(MV_DP_MSG_HASH_PROF_RESULT_SET(prof_type->result[i]));


	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}


enum mv_dp_rc mv_dp_hash_prof_id_populate_msg(void *buf, const void *data, int index)
{
	mv_nss_dp_hash_profile_id_t prof_id = *((mv_nss_dp_hash_profile_id_t *)data  + index);
	u32 *msg_buf = (u32 *)buf;

	MV_DP_LOG_DBG3("ENTER: %s buf:%p, data:%p, index:%d prof_id:%d\n", __func__, buf, data, index, prof_id);

	if (!data || !buf)
		return MV_NSS_DP_FAILED;

	msg_buf[MV_DP_MSG_HASH_PROF_PROF_ID_WORD0] = cpu_to_be32(MV_DP_MSG_HASH_PROF_PROF_ID_SET(prof_id));

	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_DP_RC_OK;
}


enum mv_dp_rc mv_dp_hash_prof_parse_struct(void *except_stats_ptr, void *buf)
{
	mv_nss_dp_hash_profile_t *hash_prof = (mv_nss_dp_hash_profile_t *)except_stats_ptr;
	u32 *msg_buf = (u32 *)buf;

	int i;

	MV_DP_LOG_DBG3("ENTER: %s\n", __func__);

	if (!hash_prof || !buf) {
		MV_DP_LOG_ERR("Null Pointer prof: %p buf: %p\n", MV_DP_RC_ERR_NULL_PTR, hash_prof, buf);
		return MV_DP_RC_ERR_NULL_PTR;
	}

	/*word 0*/
	hash_prof->profile_id =
		MV_DP_MSG_HASH_PROF_PROF_ID_GET(be32_to_cpu(msg_buf[MV_DP_MSG_HASH_PROF_PROF_ID_WORD0]));
	hash_prof->tcp_udp =
		MV_DP_MSG_HASH_PROF_TCP_UDP_BM_GET(be32_to_cpu(msg_buf[MV_DP_MSG_HASH_PROF_TCP_UDP_BM_WORD1]));
	hash_prof->ip =
		MV_DP_MSG_HASH_PROF_IP_BM_GET(be32_to_cpu(msg_buf[MV_DP_MSG_HASH_PROF_IP_BM_WORD2]));
	hash_prof->non_ip =
		MV_DP_MSG_HASH_PROF_NON_IP_BM_GET(be32_to_cpu(msg_buf[MV_DP_MSG_HASH_PROF_NON_IP_BM_WORD3]));


	for (i = 0; i < MV_NSS_DP_HASH_KNEES; i++)
		hash_prof->result[i] =
		MV_DP_MSG_HASH_PROF_RESULT_GET(be32_to_cpu(msg_buf[MV_DP_MSG_HASH_PROF_RESULT_WORD_GET(i)]));

	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}

/****************************************HASH END****************************************************/



/****************************************BULK****************************************************/
enum mv_dp_rc mv_dp_bulk_index_populate_msg(void *buf, const void *data, int offset)
{
	const u32 index = *((const u32 *)data  + offset);
	u32 *msg_buf = (u32 *)buf;

	MV_DP_LOG_DBG3("ENTER: %s buf:%p, data:%p, index:%d offset:%d\n", __func__, buf, data, index, offset);

	if (!data || !buf)
		return MV_NSS_DP_FAILED;

	msg_buf[MV_DP_MSG_BULK_INDEX_WORD0] = cpu_to_be32(MV_DP_MSG_BULK_INDEX_SET(index));

	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}

enum mv_dp_rc mv_dp_bulk_options_populate_msg(void *buf, const void *data, int offset)
{
	const struct mv_dp_msg_bulk *bulk = (const struct mv_dp_msg_bulk *)data  + offset;
	u32 *msg_buf = (u32 *)buf;

	if (!data || !buf)
		return MV_NSS_DP_FAILED;

	MV_DP_LOG_DBG3("ENTER: %s buf:%p, data:%p, index:%d options:%08X offset:%d\n", __func__,
		       buf, data, bulk->index, bulk->options, offset);

	msg_buf[MV_DP_MSG_BULK_INDEX_WORD0] = cpu_to_be32(MV_DP_MSG_BULK_INDEX_SET(bulk->index));
	msg_buf[MV_DP_MSG_FLOW_OPTIONS_WORD1] = cpu_to_be32(MV_DP_MSG_FLOW_OPTIONS_SET(bulk->options));

	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}

/****************************************BULK END****************************************************/

/****************************************NSS MEM DBG****************************************************/
static enum mv_dp_rc mv_dp_dbg_nss_mem_populate_hdr(u32 *msg_buf, const struct mv_dp_dbg_nss_mem *mem)
{
	msg_buf[MV_DP_MSG_NSS_MEM_OFFSET_WORD0] = cpu_to_be32(MV_DP_MSG_NSS_MEM_OFFSET_SET(mem->offset));

	msg_buf[MV_DP_MSG_NSS_MEM_TYPE_WORD1] = MV_DP_MSG_NSS_MEM_TYPE_SET(mem->type);
	msg_buf[MV_DP_MSG_NSS_MEM_SIZE_WORD1] |= MV_DP_MSG_NSS_MEM_SIZE_SET(mem->size);
	cpu_to_be32s(&msg_buf[MV_DP_MSG_NSS_MEM_TYPE_WORD1]);

	return MV_DP_RC_OK;
}

enum mv_dp_rc mv_dp_dbg_nss_mem_write_populate_msg(void *buf, const void  *data, int index)
{
	const struct mv_dp_dbg_nss_mem *mem = (const struct mv_dp_dbg_nss_mem *)data  + index;
	u32 *msg_buf = (u32 *)buf;

	MV_DP_LOG_DBG3("ENTER: %s buf:%p, data:%p, index:%d\n", __func__, buf, data, index);

	if (!data || !buf)
		return MV_NSS_DP_FAILED;

	if (mem->size > MV_DP_MSG_NSS_MEM_DUMP_SIZE_WRITE_B || mem->size < 0)
		return MV_NSS_DP_INVALID_PARAM;

	mv_dp_dbg_nss_mem_populate_hdr(msg_buf, mem);

	memcpy(((u8 *)(&msg_buf[MV_DP_MSG_NSS_MEM_DUMP_WORD0]) + MV_DP_MSG_NSS_MEM_DUMP_WORD0_OFF_B),
	       (void *)(&mem->arr), mem->size);


	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_DP_RC_OK;
}

enum mv_dp_rc mv_dp_dbg_nss_mem_read_populate_msg(void *buf, const void  *data, int index)
{
	const struct mv_dp_dbg_nss_mem *mem = (const struct mv_dp_dbg_nss_mem *)data  + index;
	u32 *msg_buf = (u32 *)buf;

	MV_DP_LOG_DBG3("ENTER: %s buf:%p, data:%p, index:%d\n", __func__, buf, data, index);

	if (!data || !buf)
		return MV_NSS_DP_FAILED;

	if (mem->size > MV_DP_MSG_NSS_MEM_DUMP_SIZE_READ_B || mem->size < 0)
		return MV_NSS_DP_INVALID_PARAM;

	mv_dp_dbg_nss_mem_populate_hdr(msg_buf, mem);


	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_DP_RC_OK;
}


enum mv_dp_rc mv_dp_dbg_nss_mem_parse_struct(void *out_ptr, void *buf)
{
	struct mv_dp_dbg_nss_mem *mem = (struct mv_dp_dbg_nss_mem *)out_ptr;
	u32 *msg_buf = (u32 *)buf;
	u32 tmp;


	MV_DP_LOG_DBG3("ENTER: %s\n", __func__);

	if (!mem || !buf) {
		MV_DP_LOG_ERR("Null Pointer memory ptr:%p buf:%p\n", MV_DP_RC_ERR_NULL_PTR, mem, buf);
		return MV_DP_RC_ERR_NULL_PTR;
	}

	msg_buf[MV_DP_MSG_NSS_MEM_OFFSET_WORD0] = cpu_to_be32(MV_DP_MSG_NSS_MEM_OFFSET_GET(mem->offset));

	tmp = be32_to_cpu(msg_buf[MV_DP_MSG_NSS_MEM_TYPE_WORD1]);
	mem->type = MV_DP_MSG_NSS_MEM_TYPE_GET(tmp);
	mem->size = MV_DP_MSG_NSS_MEM_SIZE_GET(tmp);

	if (mem->size > MV_DP_MSG_NSS_MEM_DUMP_SIZE_READ_B)
		mem->size = MV_DP_MSG_NSS_MEM_DUMP_SIZE_READ_B;

	memcpy((void *)(&(mem->arr)),
	       ((u8 *)(&msg_buf[MV_DP_MSG_NSS_MEM_DUMP_WORD0]) + MV_DP_MSG_NSS_MEM_DUMP_WORD0_OFF_B),
	       mem->size);


	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}

/****************************************NSS MEM DBG END**************************************************/
