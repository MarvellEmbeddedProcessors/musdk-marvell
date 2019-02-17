/*******************************************************************************
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

#include <linux/module.h>
#include <linux/types.h>

#include "mv_nss_dp.h"
#include "mv_dp_types.h"
#include "mv_dp_main.h"
#include "mv_dp_int_if.h"
#include "mv_dp_ext_hdr.h"
#include "mv_dp_fw_if.h"
#include "mv_dp_msg.h"

#ifdef MV_DP_QOS_SHADOW
#include "mv_dp_qos.h"
#endif
#include "mv_dp_includes.h"

#ifndef MV_DP_USE_TM_LOOPBACK
/*#include "net_dev/mv_dev_vq.h"*/
#endif

/*#include "common/mv_sw_if.h"*/

/*#include "fw/mv_fw.h"*/

#define MV_NSS_DP_SCHED_PRIO_IS_OK(p)	((p) >= 0 && (p) < MV_NSS_DP_SCHED_PRIO_NUM)
#ifdef REV3
static mv_nss_dp_status_t mv_nss_dp_ingress_policer_set(const mv_nss_dp_ingress_queue_cfg_t *queue,
						    uint32_t count,
						    const mv_nss_dp_result_spec_t *res);
static mv_nss_dp_status_t mv_nss_dp_egress_policer_set(const mv_nss_dp_egress_queue_cfg_t *queue,
						    uint32_t count,
						    const mv_nss_dp_result_spec_t *res);
/*
* static enum mv_sched_type mv_dp_sched_to_hw(mv_nss_dp_sched_plcy_t sw_pol);
* static mv_nss_dp_sched_plcy_t mv_dp_sched_to_dpapi(enum mv_sched_type hw_pol);
*/

/*Validations*/
static mv_nss_dp_status_t mv_nss_dp_ingress_policer_get(uint8_t queue, const mv_nss_dp_result_spec_t *res);
static mv_nss_dp_status_t mv_nss_dp_egress_policer_get(uint8_t queue, const mv_nss_dp_result_spec_t *res);
#endif


/*
 * mv_nss_dp_init
 *
 * Description:
 *       Initialize the Data Path API.
 *
 * Parameters:
 *       cfg - API initialization parameters.
 *       res - Result delivery specification or NULL.
 *
 * Returns:
 *        MV_NSS_DP_OK - On success or an error code, otherwise.
 */
mv_nss_dp_status_t mv_nss_dp_init(const mv_nss_dp_init_cfg_t *cfg, const mv_nss_dp_result_spec_t *res)
{
	struct	mv_dp_msg_info_tx tx_msg;
	u32	msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum	mv_dp_rc rc;


	MV_DP_LOG_DBG3("ENTER: %s CFG: %p\n", __func__, cfg);


	if (!cfg) {
		MV_DP_LOG_ERR("NULL CFG PTR\n", MV_DP_RC_ERR_NULL_PTR);
		rc = MV_NSS_DP_INVALID_PARAM;
		goto e_ret;
	}

	if (!MV_DP_RX_BUFF_IS_OK(cfg->requests_num)) {
		MV_DP_LOG_ERR("ILLEGAL Buffer Size: %d\n", MV_DP_RC_ERR_INVALID_PARAM, cfg->requests_num);
		rc = MV_NSS_DP_INVALID_PARAM;
		goto e_ret;
	}

	rc = mv_dp_online_set(cfg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("DPAPI INITIALIZATION FAILED buffer:%d rc:%d\n",
			      MV_DP_RC_ERR_FAILED, cfg->requests_num, rc);
		rc = MV_NSS_DP_FAILED;
		goto e_ret;
	}

		/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_INIT;
	tx_msg.res = res;
	tx_msg.count = 1;

	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_INIT, cfg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("INIT TX MSG Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("INIT TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("INIT TX OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	return MV_NSS_DP_OK;

err:


e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;

}
EXPORT_SYMBOL(mv_nss_dp_init);



/*
 * mv_nss_dp_shutdown
 *
 * Description:
 *       Terminate the Data Path API.
 *
 * Parameters:
 *       res - Result delivery specification or NULL.
 *
 * Returns:
 *        MV_NSS_DP_OK - On success or an error code, otherwise.
 */
mv_nss_dp_status_t mv_nss_dp_shutdown(const mv_nss_dp_result_spec_t *res)
{

	struct	mv_dp_msg_info_tx tx_msg;
	u32	msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum	mv_dp_rc rc;


	MV_DP_LOG_DBG3("ENTER: %s\n", __func__);
	/*send message if needed; call exit_main from callback*/


	rc = mv_dp_offline_set();
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("DPAPI SHUTDOWN FAILED\n", rc);
		goto e_ret;
	}
		/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_SHUTDOWN;
	tx_msg.res = res;
	tx_msg.count = 1;

	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_SHUTDOWN, NULL);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("INIT TX MSG Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("SHUTDOWN TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("SHUTDOWN TX OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return MV_NSS_DP_OK;

err:


e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;

}
EXPORT_SYMBOL(mv_nss_dp_shutdown);


/*
 * mv_nss_dp_hash_profile_set
 *
 * Description:
 *       Create or update a hash profile.
 *
 * Parameters:
 *       profile - Hash profile specification.
 *       res     - Result event specification or NULL.
 *
 * Returns:
 *        MV_NSS_DP_OK - On success or an error code, otherwise.
 */
mv_nss_dp_status_t mv_nss_dp_hash_profile_set(const mv_nss_dp_hash_profile_t *profile,
		const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s Hash Prof: %p\n", __func__, profile);
	rc = MV_NSS_DP_FAILED;


	if (!profile) {
		MV_DP_LOG_ERR("NULL DTLS PTR\n", MV_DP_RC_ERR_NULL_PTR);
		rc = MV_NSS_DP_INVALID_PARAM;
		goto e_ret;
	}

	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_HASH_PROFILE_SET;
	tx_msg.res = res;
	tx_msg.count = 1;

	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_HASH_PROFILE_SET, profile);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("Hash Prof Set TX MSG Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("Hash Prof set TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("Hash set TX OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:



e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
EXPORT_SYMBOL(mv_nss_dp_hash_profile_set);




/*
 * mv_nss_dp_hash_profile_delete
 *
 * Description:
 *       Delete a hash profile.
 *
 * Parameters:
 *       profile_id - Hash profile ID.
 *       res        - Result event specification or NULL.
 *
 * Returns:
 *        MV_NSS_DP_OK - On success or an error code, otherwise.
 */
mv_nss_dp_status_t mv_nss_dp_hash_profile_delete(mv_nss_dp_hash_profile_id_t profile_id,
		const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s Profile_id: %u\n", __func__, profile_id);
	rc = MV_NSS_DP_FAILED;


	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_HASH_PROFILE_DELETE;
	tx_msg.res = res;
	tx_msg.count = 1;

	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_HASH_PROFILE_DELETE, &profile_id);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("Hash Prof Del TX MSG Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("Hash Prof Del TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("Hash Del TX OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:



e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
EXPORT_SYMBOL(mv_nss_dp_hash_profile_delete);

/*
 * mv_nss_dp_hash_profile_get
 *
 * Description:
 *       Retrieve hash profile.
 *
 * Parameters:
 *       id - ID of an existing hash profile.
 *       res - Result event specification.
 *
 * Returns:
 *    MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *           .type - MV_NSS_DP_EVT_HASH_PROFILE_GET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation
 *           .params.hash_prof - Hash profile.
 *
 *    Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_hash_profile_get(mv_nss_dp_hash_profile_id_t profile_id,
		const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s Profile_id: %u\n", __func__, profile_id);
	rc = MV_NSS_DP_FAILED;


	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_HASH_PROFILE_GET;
	tx_msg.res = res;
	tx_msg.count = 1;

	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_HASH_PROFILE_GET, &profile_id);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("Hash Prof Get TX MSG Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("Hash Prof Get TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("Hash Get TX OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:



e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
EXPORT_SYMBOL(mv_nss_dp_hash_profile_get);


/*
 * mv_nss_dp_dtls_set
 *
 * Description:
 *       Create or update a DTLS record.
 *
 * Parameters:
 *       dtls - DTLS record to create or update.
 *       res  - Result delivery specification or NULL.
 *
 * Returns:
 *        MV_NSS_DP_OK - On success or an error code, otherwise.
 */
mv_nss_dp_status_t mv_nss_dp_dtls_set(const mv_nss_dp_dtls_t *dtls, const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s DTLS:%p\n", __func__, dtls);
	rc = MV_NSS_DP_FAILED;


	if (!dtls) {
		MV_DP_LOG_ERR("NULL DTLS PTR\n", MV_DP_RC_ERR_NULL_PTR);
		rc = MV_NSS_DP_INVALID_PARAM;
		goto e_ret;
	}

	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_DTLS_SET;
	tx_msg.res = res;
	tx_msg.count = 1;

	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_DTLS_SET, dtls);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("DTLS set TX MSG Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("DTLS set TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("DTLS set TX OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:



e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
EXPORT_SYMBOL(mv_nss_dp_dtls_set);


/*
 * mv_nss_dp_dtls_delete
 *
 * Description:
 *       Delete a DTLS record.
 *
 * Parameters:
 *       dtls_id - DTLS record ID to delete.
 *       res     - Result delivery specification or NULL.
 *
 * Returns:
 *        MV_NSS_DP_OK - On success or an error code, otherwise.
 */
mv_nss_dp_status_t mv_nss_dp_dtls_delete(uint16_t dtls_id, const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s DTLS ID: %d\n", __func__, dtls_id);
	rc = MV_NSS_DP_FAILED;

	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_DTLS_DELETE;
	tx_msg.res = res;
	tx_msg.count = 1;


	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_DTLS_DELETE, &dtls_id);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("DTLS del TX MSG Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("DTLS del TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("DTLS del TX OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:



e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
EXPORT_SYMBOL(mv_nss_dp_dtls_delete);


/*
 * mv_nss_dp_dtls_get
 *
 * Description:
 *       Return parameters of a DTLS record. The result is returned via event MV_NSS_DP_EVT_DTLS.
 *       In case, the specified record ID is out of range or the record has not been created
 *       MV_NSS_DP_INVALID_PARAM status is returned in the result event.
 *
 * Parameters:
 *       id - ID of an existing DTSL record.
 *       res  - Result delivery specification.
 *
 * Returns:
 *        MV_NSS_DP_OK - On success or an error code, otherwise.
 */
mv_nss_dp_status_t mv_nss_dp_dtls_get(uint16_t dtls_id, const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s DTLS ID: %d\n", __func__, dtls_id);
	rc = MV_NSS_DP_FAILED;

	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_DTLS_GET;
	tx_msg.res = res;
	tx_msg.count = 1;


	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_DTLS_GET, &dtls_id);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("DTLS get TX MSG Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("DTLS get TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("DTLS get TX OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:



e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
EXPORT_SYMBOL(mv_nss_dp_dtls_get);

/*************************DTLS END*******************************************/


/*
 * mv_nss_dp_port_set
 *
 * Description:
 *       Create or update an existing virtual port.
 *
 * Parameters:
 *       port - Virtual port specification.
 *       res   - Result delivery specification or NULL.
 *
 * Returns:
 *        MV_NSS_DP_OK - On success or an error code, otherwise.
 */
mv_nss_dp_status_t mv_nss_dp_port_set(const mv_nss_dp_port_t *in_port, const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;
	enum mv_dp_msg_id	port_opcode;
	mv_nss_dp_port_t	*port = (mv_nss_dp_port_t *)in_port;

	MV_DP_LOG_DBG3("ENTER: %s port:%p\n", __func__, port);
	rc = MV_NSS_DP_FAILED;

	if (!port) {
		MV_DP_LOG_ERR("NULL Port PTR\n", MV_DP_RC_ERR_NULL_PTR);
		rc = MV_NSS_DP_INVALID_PARAM;
		goto e_ret;
	}

	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_PORT_SET;
	tx_msg.res = res;
	tx_msg.count = 1;

	switch (port->type) {
	case (MV_NSS_DP_PORT_ETH):
		if (!MV_DP_EPORT_ID_IS_OK(port->port_id)) {
			MV_DP_LOG_ERR("INVALID ETH PORT ID:%d\n", MV_DP_RC_ERR_INVALID_PARAM, port->port_id);
			return MV_NSS_DP_INVALID_PARAM;
		}
		port_opcode = MV_DP_MSGID_EPORT_SET;
		/*port->type = MV_FW_DP_PORT_ETH;*/
		break;

	case (MV_NSS_DP_PORT_ETH_LAG):
#ifdef REV2
		if (!MV_DP_PORT_ID_IS_OK(port->port_id)) {
			MV_DP_LOG_ERR("INVALID ETH LAG PORT ID:%d\n", MV_DP_RC_ERR_INVALID_PARAM, port->port_id);
			return MV_NSS_DP_INVALID_PARAM;
		}
#endif
		port_opcode = MV_DP_MSGID_LAGPORT_SET;
		/*port->type = MV_FW_DP_PORT_ETH_LAG;*/
		break;

	case (MV_NSS_DP_PORT_CPU):
#ifdef rev2
		if (!MV_DP_CPUPORT_ID_IS_OK(port->port_id)) {
			MV_DP_LOG_ERR("INVALID CPU PORT ID:%d\n", MV_DP_RC_ERR_INVALID_PARAM, port->port_id);
			return MV_NSS_DP_INVALID_PARAM;
		}
#endif
		port_opcode = MV_DP_MSGID_CPUPORT_SET;
		/*port->type = MV_FW_DP_PORT_CPU;*/
		break;

	case (MV_NSS_DP_PORT_CAPWAP):
#ifdef REV2
		if (!MV_DP_PORT_ID_IS_OK(port->port_id)) {
			MV_DP_LOG_ERR("INVALID CAPWAP PORT ID:%d\n", MV_DP_RC_ERR_INVALID_PARAM, port->port_id);
			return MV_NSS_DP_INVALID_PARAM;
		}
#endif
		if (port->params.capwap.local_ip.ver != port->params.capwap.remote_ip.ver) {
			MV_DP_LOG_ERR("CAPWAP PORT: Local and remote IP address type mismatch\n",
										MV_DP_RC_ERR_INVALID_PARAM);
			return MV_NSS_DP_INVALID_PARAM;
		}
		port_opcode = MV_DP_MSGID_CWPORT_SET;
		/*port->type = MV_NSS_DP_PORT_CAPWAP;*/
		break;

	case (MV_NSS_DP_PORT_VXLAN):
#ifdef REV2
		if (!MV_DP_PORT_ID_IS_OK(port->port_id)) {
			MV_DP_LOG_ERR("INVALID VXLAN PORT ID:%d\n", MV_DP_RC_ERR_INVALID_PARAM, port->port_id);
			return MV_NSS_DP_INVALID_PARAM;
		}
#endif
		if (port->params.vxlan.local_ip.ver != port->params.vxlan.remote_ip.ver) {
			MV_DP_LOG_ERR("VXLAN PORT: Local and remote IP address type mismatch\n",
										MV_DP_RC_ERR_INVALID_PARAM);
			return MV_NSS_DP_INVALID_PARAM;
		}
		port_opcode = MV_DP_MSGID_VXPORT_SET;
		/*port->type = MV_FW_DP_PORT_VXLAN;*/
		break;
	default:
		MV_DP_LOG_ERR("INVALID PORT TYPE:%d PORT ID:%d\n", MV_DP_RC_ERR_INVALID_PARAM,
			      port->type, port->port_id);
		return MV_NSS_DP_INVALID_PARAM;
	}

	rc = mv_dp_msg_build(&tx_msg, port_opcode, port);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("port_set TX MSG Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("port_set TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("Port Set TX OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:



/*consider returning the current rc*/
e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
EXPORT_SYMBOL(mv_nss_dp_port_set);


/*
 * mv_nss_dp_port_delete
 *
 * Description:
 *       Destroy a virtual port. The request would fail if there are currently
 *       entities dependent on that port (eg flows.)
 *
 * Parameters:
 *       port_id - ID of an existing virtual port.
 *       res     - Result delivery specification or NULL.
 *
 * Returns:
 *        MV_NSS_DP_OK - On success or an error code, otherwise.
 */
mv_nss_dp_status_t mv_nss_dp_port_delete(mv_nss_dp_port_id_t port_id, const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s portid:%d\n", __func__, port_id);
	rc = MV_NSS_DP_FAILED;

	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_PORT_DEL;
	tx_msg.res = res;
	tx_msg.count = 1;

	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_PORT_DEL, &port_id);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("Port del TX MSG Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("Port del TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("Port del TX OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:



e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
EXPORT_SYMBOL(mv_nss_dp_port_delete);

/*
 * mv_nss_dp_port_get
 *
 * Description:
 *       Return parameters of a virtual port. The result is returned via event MV_NSS_DP_EVT_PORT.
 *
 * Parameters:
 *       port_id - ID of an existing virtual port.
 *       res     - Result delivery specification.
 *
 * Returns:
 *        MV_NSS_DP_OK - On success or an error code, otherwise.
 */
mv_nss_dp_status_t mv_nss_dp_port_get(mv_nss_dp_port_id_t port_id, const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s portid:%d\n", __func__, port_id);
	rc = MV_NSS_DP_FAILED;

	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_PORT_GET;
	tx_msg.res = res;
	tx_msg.count = 1;


	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_PORT_GET, &port_id);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("Port_get TX MSG Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("Port_get TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("Port GET TX OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:



e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
EXPORT_SYMBOL(mv_nss_dp_port_get);



/*
 * mv_nss_dp_port_stats_get
 *
 * Description:
 *       Retrieve statistics counters of a virtual port. The result is returned via event MV_NSS_DP_EVT_PORT_STATS.
 *
 * Parameters:
 *       port_id - ID of an existing virtual port.
 *       res     - Result delivery specification.
 *
 * Returns:
 *        MV_NSS_DP_OK - On success or an error code, otherwise.
 */
mv_nss_dp_status_t mv_nss_dp_port_stats_get(mv_nss_dp_port_id_t port_id, const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx	tx_msg;
	u32				msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc			rc;

	MV_DP_LOG_DBG3("ENTER: %s index:%d\n", __func__, port_id);

	rc = MV_NSS_DP_FAILED;

	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_PORT_STATS_GET;
	tx_msg.res = res;
	tx_msg.count = 1;


	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_PORT_STATS_GET, &port_id);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("port_stats_get TX MSG Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("port_stats_get TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("Port STATS GET TX OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:



e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
EXPORT_SYMBOL(mv_nss_dp_port_stats_get);


/*
 * mv_nss_dp_port_bulk_stats_get
 *
 * Description:
 *       Retrieve statistics counters of a range of virtual ports.
 *
 * Parameters:
 *       index - Index of the first record in range, starting from 0.
 *       count - Total number of records to get.
 *       stats  - Application allocated buffer to receive the data whose size must
 *                 be greater or equal to sizeof(mv_nss_dp_port_stats_t) * count.
 *                  Else, the maximum number of fitting records is returned.
 *       res     - Result delivery specification.
 *
 * Returns:
 *        MV_NSS_DP_OK - On success or an error code, otherwise.
 */
mv_nss_dp_status_t mv_nss_dp_port_bulk_stats_get(uint32_t index,
		uint32_t count,
		mv_nss_dp_port_stats_t *stats,
		const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx	tx_msg;
	u32				msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc			rc;

	return MV_NSS_DP_NOT_SUPPORTED;
	MV_DP_LOG_DBG3("ENTER: %s index:%d count:%d\n", __func__, index, count);

	rc = MV_NSS_DP_FAILED;

	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = stats;
	tx_msg.opcode = MV_DP_MSGID_PORT_STATS_GET_BULK;
	tx_msg.res = res;
	tx_msg.count = count;


	rc = mv_dp_msg_build_bulk(&tx_msg, MV_DP_MSGID_PORT_STATS_GET_BULK, &index);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("Port bulk stats get TX MSG Build, msg_size:%d, index:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, index, tx_msg.count, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("Port bulk stats get TX Failed, msg_size:%d, index:%dcount:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, index, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("Port bulk stats get TX OK msg_size:%d, index:%d count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, index, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:



e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
EXPORT_SYMBOL(mv_nss_dp_port_bulk_stats_get);




/************************************************************************************
 * mv_nss_dp_port_stats_reset
 *
 * Description:
 *       Reset counters of a virtual port or all ports.
 *
 * Parameters:
 *       port_id - ID of an existing virtual port or MV_NSS_PORT_ALL for all ports.
 *       res    - Result delivery specification.
 *
 * Returns:
 *        MV_NSS_DP_OK - On success or an error code, otherwise.
 ************************************************************************************/
mv_nss_dp_status_t mv_nss_dp_port_stats_reset(mv_nss_dp_port_id_t port_id, const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx	tx_msg;
	u32				msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc			rc;

	MV_DP_LOG_DBG3("ENTER: %s index:%d\n", __func__, port_id);

	rc = MV_NSS_DP_FAILED;

	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_PORT_STATS_RST;
	tx_msg.res = res;
	tx_msg.count = 1;


	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_PORT_STATS_RST, &port_id);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("port_stats_reset TX MSG Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("port_stats_reset TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("Port STATS RESET TX OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:



e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
EXPORT_SYMBOL(mv_nss_dp_port_stats_reset);




/*
 * mv_nss_dp_port_dst_set
 *
 * Description:
 *       Set destination virtual port for ingress packets not matching any flows.
 *
 * Parameters:
 *       port_id - ID of an existing virtual port.
 *       res - Result event specification or NULL for no event.
 *
 * Returns:
 *    MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *           .type - MV_NSS_DP_EVT_PORT_DST_SET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation
 *
 *    Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_port_dst_set(mv_nss_dp_port_id_t port_id,
		const mv_nss_dp_result_spec_t *res)
{

	struct mv_dp_msg_info_tx	tx_msg;
	u32				msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc			rc;

	MV_DP_LOG_DBG3("ENTER: %s index:%d\n", __func__, port_id);

	rc = MV_NSS_DP_FAILED;
#ifdef REV2
	if (!MV_DP_CPUPORT_ID_IS_OK(port_id)) {
		MV_DP_LOG_ERR("INVALID CPU PORT ID:%d\n", MV_DP_RC_ERR_INVALID_PARAM, port_id);
		return MV_NSS_DP_INVALID_PARAM;
	}
#endif

	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_DP_PORT_DST_SET;
	tx_msg.res = res;
	tx_msg.count = 1;


	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_DP_PORT_DST_SET, &port_id);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("port dflt dst set MSG Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("port dflt dst set TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("port dflt dst set TX OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:



e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
EXPORT_SYMBOL(mv_nss_dp_port_dst_set);

/*
 * mv_nss_dp_vlan_cfg_set
 *
 * Description:
 *       Set VLAN configuration.
 *
 * Parameters:
 *        cfg - VLAN configuration(s).
 *        count - Number of records pointed by 'cfg', greater than or equal to 1.
 *        res - Result delivery specification or NULL.
 *
 * Returns:
 *        MV_NSS_DP_OK - On success or an error code, otherwise.
 */
mv_nss_dp_status_t mv_nss_dp_vlan_cfg_set(const mv_nss_dp_vlan_cfg_t *cfg,
		uint32_t count,
		const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s VLAN cfg:%p count:%d\n", __func__, cfg, count);
	rc = MV_NSS_DP_FAILED;


	if (!cfg) {
		MV_DP_LOG_ERR("NULL VLAN cfg PTR\n", MV_DP_RC_ERR_NULL_PTR);
		rc = MV_NSS_DP_INVALID_PARAM;
		goto e_ret;
	}

	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_VLAN_CFG_SET;
	tx_msg.res = res;
	tx_msg.count = count;

	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_VLAN_CFG_SET, cfg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("VLAN set TX MSG Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, count, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("VLAN set TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("VLAN Set TX OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:



e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
EXPORT_SYMBOL(mv_nss_dp_vlan_cfg_set);

/**************************************************************************
 * mv_nss_dp_vlan_cfg_delete
 *
 * Description:
 *       Delete VLAN configuration.
 *
 * Parameters:
 *        vlan_id - VLAN ID for configuration.
 *        res - Result delivery specification or NULL.
 *
 * Returns:
 *        MV_NSS_DP_OK - On success or an error code, otherwise.
 ****************************************************************************/
mv_nss_dp_status_t mv_nss_dp_vlan_cfg_delete(const uint16_t *vlan_id,
		uint32_t count,
		const mv_nss_dp_result_spec_t *res)

{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	if (!vlan_id) {
		MV_DP_LOG_ERR("NULL VLAN_ID PTR\n", MV_DP_RC_ERR_NULL_PTR);
		rc = MV_NSS_DP_INVALID_PARAM;
		goto e_ret;
	}

	MV_DP_LOG_DBG3("ENTER: %s count:%d\n", __func__, count);
	rc = MV_NSS_DP_FAILED;

	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_VLAN_CFG_DEL;
	tx_msg.res = res;
	tx_msg.count = count;

	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_VLAN_CFG_DEL, vlan_id);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("VLAN del TX MSG Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("VLAN del TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("VLAN del TX OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:



e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
EXPORT_SYMBOL(mv_nss_dp_vlan_cfg_delete);



/*
 * mv_nss_dp_vlan_cfg_get_by_id
 *
 * Description:
 *       Get VLAN configuration matching a VLAN ID.
 *
 * Parameters:
 *       vlan_id - VLAN ID of the record to retrieve.
 *       res     - Result event specification.
 *
 * Returns:
 *		 MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *			    .type - MV_NSS_DP_EVT_VLAN_CFG_GET
 *              .cookie - Equal to res->cookie
 *			    .xid - Equal to res->xid
 *			    .status - Result of operation. In case no matching record is
 *                        found, MV_NSS_DP_ITEM_NOT_FOUND status is returned.
 *              .params.vlan_cfg - VLAN configuration record.
 *
 *		 Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_vlan_cfg_get_by_id(uint16_t vlan_id,
		const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	if (!vlan_id) {
		MV_DP_LOG_ERR("NULL VLAN_ID PTR\n", MV_DP_RC_ERR_NULL_PTR);
		rc = MV_NSS_DP_INVALID_PARAM;
		goto e_ret;
	}

	MV_DP_LOG_DBG3("ENTER: %s vlan_id:%d\n", __func__, vlan_id);
	rc = MV_NSS_DP_FAILED;

	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_VLAN_CFG_GET_BY_ID;
	tx_msg.res = res;
	tx_msg.count = 1;


	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_VLAN_CFG_GET_BY_ID, &vlan_id);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("VLAN Get BY ID TX MSG Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("VLAN Get BY ID TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("VLAN Get BY ID TX OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:



e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
EXPORT_SYMBOL(mv_nss_dp_vlan_cfg_get_by_id);



/*
 * mv_nss_dp_vlan_cfg_get
 *
 * Description:
 *       Get VLAN configuration. The result is returned in event of type MV_NSS_DP_EVT_VLAN_CFG.
 *       In case, the provided index the number of existing records, MV_NSS_DP_END_OF_LIST status is returned.
 *
 * Parameters:
 *        index    - Configuration record index starting from 0.
 *        res       - Result delivery specification.
 *
 * Returns:
 *        MV_NSS_DP_OK - On success or an error code, otherwise.
 */
mv_nss_dp_status_t mv_nss_dp_vlan_cfg_get(uint16_t index, const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s VLAN idx:%d\n", __func__, index);
	rc = MV_NSS_DP_FAILED;

	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_VLAN_CFG_GET;
	tx_msg.res = res;
	tx_msg.count = 1;


	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_VLAN_CFG_GET, &index);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("VLAN get TX MSG Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("VLAN get TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("VLAN GET TX OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:



e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
EXPORT_SYMBOL(mv_nss_dp_vlan_cfg_get);




/*
 * mv_nss_dp_mc_bridged_cfg_set
 *
 * Description:
 *       Set bridged multicast configuration.
 *
 * Parameters:
 *        cfg - Bridged multicast configuration(s).
 *        count - Number of records pointed by 'cfg', greater than or equal to 1.
 *        res - Result delivery specification or NULL.
 *
 * Returns:
 *        MV_NSS_DP_OK - On success or an error code, otherwise.
 */
mv_nss_dp_status_t mv_nss_dp_mc_bridged_cfg_set(const mv_nss_dp_mc_bridged_cfg_t *cfg,
		uint32_t count,
		const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s mc:%p\n", __func__, cfg);
	rc = MV_NSS_DP_FAILED;

	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_MC_BRIDGED_CFG_SET;
	tx_msg.res = res;
	tx_msg.count = count;


	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_MC_BRIDGED_CFG_SET, cfg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("MC Bridged set TX MSG Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("MC Bridged set TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("MC Bridged set TX OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:



e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
EXPORT_SYMBOL(mv_nss_dp_mc_bridged_cfg_set);





/*
 * mv_nss_dp_mc_bridged_cfg_delete
 *
 * Description:
 *       Delete bridged multicast configuration(s) matching a multicast
 *       Layer 2 address.
 *
 * Parameters:
 *       cfg - Pointer to array of bridged multicast configurations to delete.
 *             The .opaque field is ignored when performing the configuration match.
 *       count - Number of records pointed by 'l2_addr', greater than or equal to 1.
 *       res - Result event specification or NULL.
 *
 * Returns:
 *    MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *           .type - MV_NSS_DP_EVT_MC_BRIDGED_CFG_DELETE
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .count - The number of records from the beginning of
 *                    the 'cfg' array successfully processed
 *           .status - Result of operation
 *
 *    Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_mc_bridged_cfg_delete(const mv_nss_dp_mc_bridged_cfg_t *cfg,
		uint32_t count,
		const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s mc:%p\n", __func__, cfg);
	rc = MV_NSS_DP_FAILED;

	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_MC_BRIDGED_CFG_DELETE;
	tx_msg.res = res;
	tx_msg.count = count;


	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_MC_BRIDGED_CFG_DELETE, cfg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("MC Bridged delete TX MSG Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("MC Bridged delete TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("MC Bridged delete TX OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:



e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
EXPORT_SYMBOL(mv_nss_dp_mc_bridged_cfg_delete);





/*
 * mv_nss_dp_mc_bridged_cfg_get
 *
 * Description:
 *       Get bridged multicast configuration. The result is returned in event of type MV_NSS_DP_EVT_MC_BRIDGED_CFG.
 *       In case, the provided index the number of existing records, MV_NSS_DP_END_OF_LIST status is returned.
 *
 * Parameters:
 *        index - Configuration record index starting from 0.
 *        res - Result delivery specification.
 *
 * Returns:
 *        MV_NSS_DP_OK - On success or an error code, otherwise.
 */
mv_nss_dp_status_t mv_nss_dp_mc_bridged_cfg_get(uint16_t index, const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s index:%d\n", __func__, index);
	rc = MV_NSS_DP_FAILED;

	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_MC_BRIDGED_CFG_GET;
	tx_msg.res = res;
	tx_msg.count = 1;


	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_MC_BRIDGED_CFG_GET, &index);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("MC Bridged get TX MSG Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("MC Bridged get TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("MC Bridged get TX OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:



e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
EXPORT_SYMBOL(mv_nss_dp_mc_bridged_cfg_get);


/*
 * mv_nss_dp_mc_tunneled_cfg_set
 *
 * Description:
 *       Set tunneled multicast configuration.
 *
 * Parameters:
 *        cfg - Tunneled multicast configuration(s).
 *        count - Number of records pointed by 'cfg', greater than or equal to 1.
 *        res - Result delivery specification or NULL.
 *
 * Returns:
 *        MV_NSS_DP_OK - On success or an error code, otherwise.
 */
mv_nss_dp_status_t mv_nss_dp_mc_tunneled_cfg_set(const mv_nss_dp_mc_tunneled_cfg_t *cfg,
		uint32_t count,
		const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s mc:%p\n", __func__, cfg);
	rc = MV_NSS_DP_FAILED;

	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_MC_TUNNELED_CFG_SET;
	tx_msg.res = res;
	tx_msg.count = count;


	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_MC_TUNNELED_CFG_SET, cfg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("MC Tunneled set TX MSG Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("MC Tunneled set TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("MC Tunneled set TX OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:



e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
EXPORT_SYMBOL(mv_nss_dp_mc_tunneled_cfg_set);





/*
 * mv_nss_dp_mc_tunneled_cfg_delete
 *
 * Description:
 *       Delete tunneled multicast configuration.
 *       The result is returned in event of type MV_NSS_DP_EVT_ACK.
 *
 * Parameters:
 *        mgid - Multicast group ID.
 *        res - Result delivery specification or NULL.
 *
 * Returns:
 *        MV_NSS_DP_OK - On success or an error code, otherwise.
 */
mv_nss_dp_status_t mv_nss_dp_mc_tunneled_cfg_delete(const uint16_t *mgid,
		uint32_t count,
		const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s mgid:%p\n", __func__, mgid);
	rc = MV_NSS_DP_FAILED;

	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_MC_TUNNELED_CFG_DELETE;
	tx_msg.res = res;
	tx_msg.count = count;


	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_MC_TUNNELED_CFG_DELETE, mgid);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("MC Tunneled delete TX MSG Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("MC Tunneled delete TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("MC Tunneled delete TX OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:



e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
EXPORT_SYMBOL(mv_nss_dp_mc_tunneled_cfg_delete);





/*
 * mv_nss_dp_mc_tunneled_cfg_get
 *
 * Description:
 *       Get tunneled multicast configuration. The result is returned in event of type MV_NSS_DP_EVT_MC_TUNNELED_CFG.
 *       In case, the provided index the number of existing records, MV_NSS_DP_END_OF_LIST status is returned.
 *
 * Parameters:
 *        index - Configuration record index starting from 0.
 *        res - Result delivery specification.
 *
 * Returns:
 *        MV_NSS_DP_OK - On success or an error code, otherwise.
 */
mv_nss_dp_status_t mv_nss_dp_mc_tunneled_cfg_get(uint16_t index, const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s index:%d\n", __func__, index);
	rc = MV_NSS_DP_FAILED;

	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_MC_TUNNELED_CFG_GET;
	tx_msg.res = res;
	tx_msg.count = 1;


	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_MC_TUNNELED_CFG_GET, &index);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("MC Tunneled get TX MSG Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("MC Tunneled get TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("MC Tunneled get TX OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:



e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
EXPORT_SYMBOL(mv_nss_dp_mc_tunneled_cfg_get);





/*
 * mv_nss_dp_flow_set
 *
 * Description:
 *       Add or update flows.
 *
 * Parameters:
 *       flow - Pointer to array of flows to be added or updated.  For new flows the .flow_id
 *               field must be set to MV_NSS_DP_FLOW_ID_NONE.
 *               The actual flow IDs are auto-generated and returned via the result event.
 *               IMPORTANT: Application must not deallocate the 'flow' array until receiving the resut event.
 *       count - Number of records pointed by flows, greater than or equal to 1.
 *       res  - Result delivery specification.
 *
 * Returns:
 *        MV_NSS_DP_OK - Request has been accepted. The result is returned via event
 *                           with the following values of mv_nss_dp_event_t fields:
 *           .type  - MV_NSS_DP_EVT_FLOW_SET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation.
 *           .count - Index of a flow ID in the 'flow_id' array corresponding
 *                     to the last successfully created or updated flow.
 *           .params.flow - Equal to 'flow'.
 *
 *       Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_flow_set(mv_nss_dp_flow_t *flow,
		uint32_t count,
		const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s flow:%p count:%d\n", __func__, flow, count);
	rc = MV_NSS_DP_FAILED;

#ifdef MV_DP_DEBUG
	if (!flow) {
		MV_DP_LOG_ERR("NULL Flow PTR\n", MV_DP_RC_ERR_NULL_PTR);
		rc = MV_NSS_DP_INVALID_PARAM;
		goto e_ret;
	}
#endif
	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = flow;
	tx_msg.opcode = MV_DP_MSGID_FLOW_SET;
	tx_msg.res = res;
	tx_msg.count = count;

	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_FLOW_SET, flow);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("FLOW_SET TX MSG Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, count, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("FLOW_SET TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("FLOW_SET TX OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:



/*consider returning the current rc*/
e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
EXPORT_SYMBOL(mv_nss_dp_flow_set);




/*
 * mv_nss_dp_flow_delete
 *
 * Description:
 *       Remove an existing from packet processor.
 *
 * Parameters:
 *       flow_id - Pointer to array of flow IDs to remove.
 *       count - Number of records pointed by 'flow_id', greater than or equal to 1.
 *       res     - Result delivery specification or NULL.
 *
 * Returns:
 *        MV_NSS_DP_OK - Request has been accepted.
 *                       The result is returned via event with the following values of mv_nss_dp_event_t fields:
 *           .type  - MV_NSS_DP_EVT_FLOW_DELETE
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation.
 *           .count - Index of a flow ID in the 'flow_id' array corresponding to the last successfully deleted flow.
 *
 *       Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_flow_delete(const mv_nss_dp_flow_id_t *flow_id,
		uint32_t count,
		const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s flow_id:%p count:%d\n", __func__, flow_id, count);
	rc = MV_NSS_DP_FAILED;

#ifdef MV_DP_DEBUG
	if (!flow_id) {
		MV_DP_LOG_ERR("NULL Flow PTR\n", MV_DP_RC_ERR_NULL_PTR);
		rc = MV_NSS_DP_INVALID_PARAM;
		goto e_ret;
	}
#endif
	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_FLOW_DELETE;
	tx_msg.res = res;
	tx_msg.count = count;

	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_FLOW_DELETE, flow_id);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("FLOW_DELETE TX MSG Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, count, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("FLOW_DELETE TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("FLOW_DELETE TX OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:



/*consider returning the current rc*/
e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
EXPORT_SYMBOL(mv_nss_dp_flow_delete);




/*
 * mv_nss_dp_flow_delete_all
 *
 * Description:
 *       Remove all flows from packet processor.
 *
 * Parameters:
 *       res - Result delivery specification or NULL.
 *
 * Returns:
 *        MV_NSS_DP_OK - On success or an error code, otherwise.
 */
mv_nss_dp_status_t mv_nss_dp_flow_delete_all(const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s\n", __func__);
	rc = MV_NSS_DP_FAILED;

	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_FLOW_DELETE_ALL;
	tx_msg.res = res;
	tx_msg.count = 1;


	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_FLOW_DELETE_ALL, NULL);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("FLOW DELETE ALL Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("FLOW_DELETE_ALL TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("FLOW_DELETE_ALL OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:


/*consider returning the current rc*/
e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
EXPORT_SYMBOL(mv_nss_dp_flow_delete_all);

/*
 * mv_nss_dp_flow_get
 *
 * Description:
 *       Retrieve flow record.
 *
 * Parameters:
 *       flow_id - ID of an existing flow to retrieve.
 *       res     - Result event specification.
 *
 * Returns:
 *		 MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *			    .type - MV_NSS_DP_EVT_FLOW_GET
 *              .cookie - Equal to res->cookie
 *			    .xid - Equal to res->xid
 *			    .status - Result of operation
 *              .params.flow - Client record retrieved
 *
 *		 Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_flow_get(mv_nss_dp_flow_id_t flow_id,
		const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s flow_id:%d\n", __func__, flow_id);
	rc = MV_NSS_DP_FAILED;

	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_FLOW_GET;
	tx_msg.res = res;
	tx_msg.count = 1;

	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_FLOW_GET, &flow_id);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("FLOW_GET TX MSG Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, 1, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("FLOW_GET TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("FLOW_GET TX OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:



/*consider returning the current rc*/
e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
EXPORT_SYMBOL(mv_nss_dp_flow_get);


/*
 * mv_nss_dp_flow_status_set
 *
 * Description:
 *       Set flow activity status. The result is returned in event of type  MV_NSS_DP_EVT_FLOW_STATUS_SET.
 *
 * Parameters:
 *       flow_id - Pointer to array of IDs of flows whose statui are to be updated.
 *       res     - Result delivery specification.
 *
 * Returns:
 *        MV_NSS_DP_OK - On success or an error code, otherwise.
 */
mv_nss_dp_status_t mv_nss_dp_flow_status_set(const mv_nss_dp_flow_status_t *flow_status,
		uint32_t count,
		const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s flow_status:%p count:%d\n", __func__, flow_status, count);
	rc = MV_NSS_DP_FAILED;

	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_FLOW_STATUS_SET;
	tx_msg.res = res;
	tx_msg.count = count;

	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_FLOW_STATUS_SET, flow_status);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("FLOW_STATUS_SET TX MSG Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, count, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("FLOW_STATUS_SET TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("FLOW_STATUS_SET TX OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:



/*consider returning the current rc*/
e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
EXPORT_SYMBOL(mv_nss_dp_flow_status_set);




/*
 * mv_nss_dp_flow_status_get
 *
 * Description:
 *       Retrieve flow activity status. The result is returned in event of type  MV_NSS_DP_EVT_FLOW_STATUS.
 *
 * Parameters:
 *       flow_id - ID of an existing flow.
 *       res     - Result delivery specification.
 *
 * Returns:
 *        MV_NSS_DP_OK - On success or an error code, otherwise.
 */
mv_nss_dp_status_t mv_nss_dp_flow_status_get(mv_nss_dp_flow_id_t flow_id, const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s flow_id:%d\n", __func__, flow_id);
	rc = MV_NSS_DP_FAILED;

	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_FLOW_STATUS_GET;
	tx_msg.res = res;
	tx_msg.count = 1;

	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_FLOW_STATUS_GET, &flow_id);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("FLOW_STATUS_GET TX MSG Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, 1, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("FLOW_STATUS_GET TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("FLOW_STATUS_SET TX OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:



/*consider returning the current rc*/
e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
EXPORT_SYMBOL(mv_nss_dp_flow_status_get);




/*
 * mv_nss_dp_flow_stats_get
 *
 * Description:
 *       Retrieve a flow statistics. The result is returned in event of type  MV_NSS_DP_EVT_FLOW_STATS.
 *
 * Parameters:
 *       flow_id - ID of an existing flow.
 *       res     - Result delivery specification.
 *
 * Returns:
 *        MV_NSS_DP_OK - On success or an error code, otherwise.
 */
mv_nss_dp_status_t mv_nss_dp_flow_stats_get(mv_nss_dp_flow_id_t flow_id, const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s flow_id:%d\n", __func__, flow_id);
	rc = MV_NSS_DP_FAILED;

	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_FLOW_STATS_GET;
	tx_msg.res = res;
	tx_msg.count = 1;

	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_FLOW_STATS_GET, &flow_id);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("FLOW_STATS_GET TX MSG Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, 1, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("FLOW_STATS_GET TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("FLOW_STATS_GET TX OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:



/*consider returning the current rc*/
e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
EXPORT_SYMBOL(mv_nss_dp_flow_stats_get);





/*
 * mv_nss_dp_flow_stats_reset
 *
 * Description:
 *       Reset a flow statistics.
 *
 * Parameters:
 *       flow - ID of existing flow.
 *       res     - Result delivery specification or NULL.
 *
 * Returns:
 *        MV_NSS_DP_OK - On success or an error code, otherwise.
 */
mv_nss_dp_status_t mv_nss_dp_flow_stats_reset(mv_nss_dp_flow_id_t flow_id, const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s flow_id:%d\n", __func__, flow_id);
	rc = MV_NSS_DP_FAILED;

	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_FLOW_STATS_RESET;
	tx_msg.res = res;
	tx_msg.count = 1;

	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_FLOW_STATS_RESET, &flow_id);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("FLOW_STATS_RST TX MSG Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, 1, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("FLOW_STATS_RST TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("FLOW_STATS_RST TX OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:



/*consider returning the current rc*/
e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
EXPORT_SYMBOL(mv_nss_dp_flow_stats_reset);




/*
 * mv_nss_dp_flow_bulk_stats_get
 *
 * Description:
 *       Retrieve snapshot of flow statistics.
 *
 * Parameters:
 *       index - Index of the first record in range, starting from 0.
 *       count - Total number of records to retrieve.
 *       options - Bit-mask specifying the subset of flows to query. If zero, all flows are implied.
 *               bit [0] - Flow status:
 *                  1 - Flows with inactive status only
 *                  0 - Flows with any activity status
 *       stats - Application allocated buffer to receive the data whose size
 *               must be greater or equal to  sizeof(mv_nss_dp_flow_stats_t) * count.
 *               Else, the maximum number of fitting records is returned.
 *               IMPORTANT: Application must not deallocate the 'stats' buffer
 *               until receiving the result event.
 *
 *       res     - Result event specification.
 *
 * Returns:
 *       MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *
 *           .type  - MV_NSS_DP_EVT_FLOW_STATS_GET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .count - Number of records in params.flow_stats.
 *                    Can be smaller than the requested number 'count' in case
 *                    there are fewer records.
 *           .status - Result of operation. In case, 'index' exceeds
 *                     the number of existing records, MV_NSS_DP_END_OF_LIST
 *                     status is returned.
 *           .params.flow_stats - Equal to 'stats'.
 *
 *       Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_flow_bulk_stats_get(uint32_t index,
		uint32_t count,
		uint32_t options,
		mv_nss_dp_flow_stats_t *stats,
		const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx	tx_msg;
	struct mv_dp_msg_bulk		tx_bulk;
	u32				msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc			rc;

	MV_DP_LOG_DBG3("ENTER: %s index:%d count:%d\n", __func__, index, count);
	return MV_NSS_DP_NOT_SUPPORTED;
	rc = MV_NSS_DP_FAILED;

	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = stats;
	tx_msg.opcode = MV_DP_MSGID_FLOW_STATS_GET_BULK;
	tx_msg.res = res;
	tx_msg.count = count;
	tx_bulk.index = index;
	tx_bulk.options = options;


	rc = mv_dp_msg_build_bulk(&tx_msg, MV_DP_MSGID_FLOW_STATS_GET_BULK, &tx_bulk);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("Flow bulk stats get TX MSG Build, msg_size:%d, index:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, index, tx_msg.count, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("Flow bulk stats get TX Failed, msg_size:%d, index:%dcount:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, index, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("Flow bulk stats get TX OK msg_size:%d, index:%d count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, index, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:



e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
EXPORT_SYMBOL(mv_nss_dp_flow_bulk_stats_get);




/*
 * mv_nss_dp_flow_get_count
 *
 * Description:
 *       Total number of flows in packet processor. The result is returned in event of type  MV_NSS_DP_EVT_FLOW_COUNT.
 *
 * Parameters:
 *       res     - Result delivery specification.
 *
 * Returns:
 *        MV_NSS_DP_OK - On success or an error code, otherwise.
 */
mv_nss_dp_status_t mv_nss_dp_flow_count_get(const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s\n", __func__);
	rc = MV_NSS_DP_FAILED;

	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_FLOW_COUNT_GET;
	tx_msg.res = res;
	tx_msg.count = 1;

	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_FLOW_COUNT_GET, NULL);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("FLOW GET COUNT Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto e_ret;
	}


	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("FLOW_COUNT_GET TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("FLOW_COUNT_GET OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:


/*consider returning the current rc*/
e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
EXPORT_SYMBOL(mv_nss_dp_flow_count_get);




/*
 * mv_nss_dp_ingress_queue_cfg_set
 *
 * Description:
 *       Update virtual ingress queue configuration.
 *
 * Parameters:
 *       cfg - Virtual ingress queue configuration.
 *       res - Result delivery specification or NULL.
 *
 * Returns:
 *        MV_NSS_DP_OK - On success or an error code, otherwise.
 */
mv_nss_dp_status_t mv_nss_dp_ingress_queue_cfg_set(const mv_nss_dp_ingress_queue_cfg_t *cfg,
		uint32_t count,
		const mv_nss_dp_result_spec_t *res)
{
#ifdef REV3
	int i;

	mv_nss_dp_ingress_queue_cfg_t *tmp = (mv_nss_dp_ingress_queue_cfg_t *)cfg;

	MV_DP_LOG_DBG3("ENTER: %s\n", __func__);

	if (!cfg) {
		MV_DP_LOG_ERR("INGRESS QUEUE CFG SET\n", MV_DP_RC_ERR_INVALID_PARAM);
		return MV_DP_RC_ERR_INVALID_PARAM;
	}


	for (i = 0; i < count; i++) {
		if (!MV_NSS_DP_SCHED_PRIO_IS_OK(tmp->sched_priority)) {
			MV_DP_LOG_ERR("INGRESS QUEUE:%d ILLEGAL PRIO:%d\n", MV_DP_RC_ERR_INVALID_PARAM,
				      tmp->queue, tmp->sched_priority);
			goto err;
		}
		/*ingress queue check each set for error -- return invalid param error*/
		if (mv_nss_ingress_vq_drop_set(tmp->queue, tmp->tail_drop_thresh, tmp->red_thresh)) {
			MV_DP_LOG_ERR("INGRESS QUEUE:%d DROP PARAMS tail_drop_threshold:%d, red:%d\n",
				      MV_DP_RC_ERR_INVALID_PARAM,
				      tmp->queue, tmp->tail_drop_thresh, tmp->red_thresh);
			goto err;
		}

		if (mv_nss_ingress_vq_sched_set(tmp->queue, tmp->sched_priority, tmp->sched_weight)) {
			MV_DP_LOG_ERR("INGRESS QUEUE:%d ILLEGAL SCHED PARAMS prio:%d, weight:%d\n",
				      MV_DP_RC_ERR_INVALID_PARAM,
				      tmp->queue, tmp->sched_priority, tmp->sched_weight);
			goto err;
		}
		tmp++;
	}

	/*send message to FW: policer cir/pir/cbs/ebs*/
	return mv_nss_dp_ingress_policer_set(cfg, count, res);

err:

	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);
#endif
	return MV_NSS_DP_NOT_SUPPORTED;
}
/*EXPORT_SYMBOL(mv_nss_dp_ingress_queue_cfg_set);*/




/*
 * mv_nss_dp_ingress_queue_cfg_get
 *
 * Description:
 *       Retrieve virtual ingress queue configuration.
 *       The result is returned in event of type  MV_NSS_DP_EVT_INGRESS_Q_CFG.
 *
 * Parameters:
 *       queue - virtual ingress queue which configuration to retrieve.
 *       res - Result delivery specification.
 *
 * Returns:
 *        MV_NSS_DP_OK - On success or an error code, otherwise.
 */
mv_nss_dp_status_t mv_nss_dp_ingress_queue_cfg_get(uint8_t queue, const mv_nss_dp_result_spec_t *res)
{
#ifdef REV3
	return mv_nss_dp_ingress_policer_get(queue, res);
#endif
	return MV_NSS_DP_NOT_SUPPORTED;
}
/*EXPORT_SYMBOL(mv_nss_dp_ingress_queue_cfg_get);*/




/*
 * mv_nss_dp_ingress_prio_map_set
 *
 * Description:
 *       Update mapping of ingress priority values to ingress queues.
 *
 * Parameters:
 *       res - Result delivery specification or NULL.
 *
 * Returns:
 *        MV_NSS_DP_OK - On success or an error code, otherwise.
 */
mv_nss_dp_status_t mv_nss_dp_ingress_prio_cfg_set(const mv_nss_dp_ingress_prio_cfg_t *prio,
		const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s\n", __func__);
	rc = MV_NSS_DP_FAILED;

	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_INGRESS_PRIO_CFG_SET;
	tx_msg.res = res;
	tx_msg.count = 1;

	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_INGRESS_PRIO_CFG_SET, prio);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("INGRESS PRIO CFG SET Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("INGRESS PRIO CFG SET TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("INGRESS PRIO CFG SET OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:


/*consider returning the current rc*/
e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
EXPORT_SYMBOL(mv_nss_dp_ingress_prio_cfg_set);



/*
 * mv_nss_dp_ingress_prio_map_get
 *
 * Description:
 *       Retrieve current mapping of ingress priority values to ingress queues.
 *       The result is returned in event of type  MV_NSS_DP_EVT_INGRESS_PRIO.
 *
 * Parameters:
 *       res - Result delivery specification.
 *
 * Returns:
 *        MV_NSS_DP_OK - On success or an error code, otherwise.
 */
mv_nss_dp_status_t mv_nss_dp_ingress_prio_cfg_get(const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s\n", __func__);
	rc = MV_NSS_DP_FAILED;

	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_INGRESS_PRIO_CFG_GET;
	tx_msg.res = res;
	tx_msg.count = 1;

	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_INGRESS_PRIO_CFG_GET, NULL);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("INGRESS PRIO CFG GET Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("INGRESS PRIO CFG GET TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("INGRESS PRIO CFG GET TX OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:



/*consider returning the current rc*/
e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
EXPORT_SYMBOL(mv_nss_dp_ingress_prio_cfg_get);






/*
 * mv_nss_dp_ingress_qos_policy_set
 *
 * Description:
 *       Update ingress QoS policy record.
 *
 * Parameters:
 *       policy - New priority policy record.
 *       count - Number of records pointed by 'policy', greater than or equal to 1.
 *       res - Result delivery specification or NULL.
 *
 * Returns:
 *        MV_NSS_DP_OK - On success or an error code, otherwise.
 */
mv_nss_dp_status_t mv_nss_dp_ingress_qos_policy_set(const mv_nss_dp_ingress_qos_policy_t *policy,
		uint32_t count,
		const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;
	int			i;

	MV_DP_LOG_DBG3("ENTER: %s\n", __func__);
	rc = MV_NSS_DP_FAILED;

	for (i = 0; i < count; i++) {
		MV_DP_LOG_DBG2("setting policy count:%d\n", i);
		if (mv_dp_check_internal_prio(policy[i].dscp_to_prio,
							     sizeof(policy[i].dscp_to_prio),
							     MV_NSS_DP_PRIO_NUM) != MV_DP_RC_OK) {
			MV_DP_LOG_ERR("INGRESS QOS DSCP POLICY INVALID VALUE\n", MV_DP_RC_ERR_INVALID_PARAM);
			rc = MV_DP_RC_ERR_INVALID_PARAM;
			goto e_ret;
		}

		if (mv_dp_check_internal_prio(policy[i].l2_to_prio,
							     sizeof(policy[i].l2_to_prio),
							     MV_NSS_DP_PRIO_NUM) != MV_DP_RC_OK) {
			MV_DP_LOG_ERR("INGRESS QOS L2 POLICY INVALID VALUE\n", MV_DP_RC_ERR_INVALID_PARAM);
			rc = MV_DP_RC_ERR_INVALID_PARAM;
			goto e_ret;
		}
	}


	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_INGRESS_QOS_POLICY_SET;
	tx_msg.res = res;
	tx_msg.count = count;

	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_INGRESS_QOS_POLICY_SET, policy);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("INGRESS QOS POLICY SET Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("INGRESS QOS POLICY SET TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("INGRESS QOS POLICY SET OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:



/*consider returning the current rc*/
e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
EXPORT_SYMBOL(mv_nss_dp_ingress_qos_policy_set);



/*
 * mv_nss_dp_egress_queue_cfg_set
 *
 * Description:
 *       Update virtual egress queue configuration.
 *
 * Parameters:
 *       cfg - Pointer to array of virtual egress queue configurations.
 *       count - Number of records pointed by 'cfg', greater than or equal to 1.
 *       res - Result event specification or NULL for no event.
 *
 * Returns:
 *       MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *
 *           .type  - MV_NSS_DP_EVT_EGRESS_Q_CFG_SET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *			 .status - Result of operation.
 *           .count - The number of records from the beginning of
 *                    the 'cfg' array successfully processed
 *
 *       Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_egress_queue_cfg_set(const mv_nss_dp_egress_queue_cfg_t *cfg,
		uint32_t count,
		const mv_nss_dp_result_spec_t *res)
{
#ifdef REV3
	int i;

	mv_nss_dp_egress_queue_cfg_t *tmp = (mv_nss_dp_egress_queue_cfg_t *)cfg;

	MV_DP_LOG_DBG3("ENTER: %s\n", __func__);

	if (!cfg) {
		MV_DP_LOG_ERR("EGRESS QUEUE CFG SET\n", MV_DP_RC_ERR_INVALID_PARAM);
		return MV_DP_RC_ERR_INVALID_PARAM;
	}

	for (i = 0; i < count; i++) {
		if (!MV_NSS_DP_SCHED_PRIO_IS_OK(tmp->sched_priority)) {
			MV_DP_LOG_ERR("EGRESS QUEUE:%d ILLEGAL PRIO:%d\n", MV_DP_RC_ERR_INVALID_PARAM,
				      tmp->queue, tmp->sched_priority);
			goto err;
		}

		/*ingress queue check each set for error -- return invalid param error*/
		if (mv_nss_egress_vq_size_set(tmp->queue, tmp->tail_drop_thresh)) {
			MV_DP_LOG_ERR("EGRESS QUEUE:%d size set ILLEGAL PARAM LENGTH:%d\n",
				      MV_DP_RC_ERR_INVALID_PARAM, tmp->queue, tmp->tail_drop_thresh);
			goto err;
		}

		if (mv_nss_egress_vq_sched_set(tmp->queue, tmp->sched_priority, tmp->sched_weight)) {
			MV_DP_LOG_ERR("EGRESS QUEUE:%d sched set ILLEGAL PARAM, PRIO:%d WEIGHT:%d\n",
				      MV_DP_RC_ERR_INVALID_PARAM, tmp->queue, tmp->sched_priority, tmp->sched_weight);
			goto err;
		}

		if (mv_nss_egress_vq_shaper_set(tmp->queue,
						tmp->shaper.cir,
						tmp->shaper.eir,
						tmp->shaper.cbs,
						tmp->shaper.ebs)) {
			MV_DP_LOG_ERR("EGRESS QUEUE:%d shaper set\n", MV_DP_RC_ERR_INVALID_PARAM, tmp->queue);
			goto err;
		}

		tmp++;
	}

	/*send message to FW: policer cir/pir/cbs/ebs*/
	return mv_nss_dp_egress_policer_set(cfg, count, res);

err:
	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);
#endif
	return MV_NSS_DP_NOT_SUPPORTED;
}
/*EXPORT_SYMBOL(mv_nss_dp_egress_queue_cfg_set);*/



/*
 * mv_nss_dp_egress_queue_cfg_get
 *
 * Description:
 *       Retrieve virtual egress queue configuration.
 *
 * Parameters:
 *       queue - Virtual egress queue which configuration to retrieve.
 *       res - Result event specification.
 *
 * Returns:
 *       MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *
 *           .type  - MV_NSS_DP_EVT_EGRESS_Q_CFG_GET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *			 .status - Result of operation.
 *			 .params.egress_queue_cfg - Ingress queue configuration.
 *
 *       Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_egress_queue_cfg_get(uint8_t queue,
		const mv_nss_dp_result_spec_t *res)
{
#ifdef REV3
	return  mv_nss_dp_egress_policer_get(queue, res);
#endif
	return MV_NSS_DP_NOT_SUPPORTED;
}
/*EXPORT_SYMBOL(mv_nss_dp_egress_queue_cfg_get);*/


/*
 * mv_nss_dp_ingress_qos_policy_get
 *
 * Description:
 *       Retrieve an ingress QoS policy.
 *       The result is returned in event of type  MV_NSS_DP_EVT_PRIO_POLICY.
 *
 * Parameters:
 *       policy_id - ID of the priority policy to retrieve.
 *       res - Result delivery specification.
 *
 * Returns:
 *        MV_NSS_DP_OK - On success or an error code, otherwise.
 */
mv_nss_dp_status_t mv_nss_dp_ingress_qos_policy_get(uint8_t policy_id, const mv_nss_dp_result_spec_t *res)
{

	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s\n", __func__);
	rc = MV_NSS_DP_FAILED;

	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_INGRESS_QOS_POLICY_GET;
	tx_msg.res = res;
	tx_msg.count = 1;

	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_INGRESS_QOS_POLICY_GET, &policy_id);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("INGRESS QOS POLICY GET Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("INGRESS QOS POLICY GET TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("INGRESS QOS POLICY GET OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:



/*consider returning the current rc*/
e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
EXPORT_SYMBOL(mv_nss_dp_ingress_qos_policy_get);






/*
 * mv_nss_dp_egress_qos_policy_set
 *
 * Description:
 *       Update egress QoS policy record.
 *
 * Parameters:
 *       policy - New priority policy record.
 *       count - Number of records pointed by 'policy'. Must be equal to 1.
 *       res - Result delivery specification or NULL.
 *
 * Returns:
 *        MV_NSS_DP_OK - On success or an error code, otherwise.
 */
mv_nss_dp_status_t mv_nss_dp_egress_qos_policy_set(const mv_nss_dp_egress_qos_policy_t *policy,
		uint32_t count,
		const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;
	int			i;

	MV_DP_LOG_DBG3("ENTER: %s\n", __func__);
	rc = MV_NSS_DP_FAILED;

	for (i = 0; i < count; i++) {
		MV_DP_LOG_DBG2("setting policy count:%d\n", i);
		if (mv_dp_check_pkt_prio(policy[i].prio_to_up, sizeof(policy[i].prio_to_up), 0x7) !=  MV_DP_RC_OK) {
			MV_DP_LOG_ERR("EGRESS QOS UP POLICY INVALID VALUE\n", MV_DP_RC_ERR_INVALID_PARAM);
			rc = MV_DP_RC_ERR_INVALID_PARAM;
			goto e_ret;
		}

		if (mv_dp_check_pkt_prio(policy[i].prio_to_pcp, sizeof(policy[i].prio_to_pcp), 0x7) != MV_DP_RC_OK) {
			MV_DP_LOG_ERR("EGRESS QOS PCP POLICY INVALID VALUE\n", MV_DP_RC_ERR_INVALID_PARAM);
			rc = MV_DP_RC_ERR_INVALID_PARAM;
			goto e_ret;
		}

		if (mv_dp_check_pkt_prio(policy[i].prio_to_dscp,
				sizeof(policy[i].prio_to_dscp), 0x3F) !=  MV_DP_RC_OK) {
			MV_DP_LOG_ERR("EGRESS QOS DSCP POLICY INVALID VALUE\n", MV_DP_RC_ERR_INVALID_PARAM);
			rc = MV_DP_RC_ERR_INVALID_PARAM;
			goto e_ret;
		}
	}


	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_EGRESS_QOS_POLICY_SET;
	tx_msg.res = res;
	tx_msg.count = count;

	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_EGRESS_QOS_POLICY_SET, policy);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("EGRESS QOS POLICY SET Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("EGRESS QOS POLICY SET TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("EGRESS QOS POLICY SET OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:


/*consider returning the current rc*/
e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
EXPORT_SYMBOL(mv_nss_dp_egress_qos_policy_set);






/*
 * mv_nss_dp_egress_qos_policy_get
 *
 * Description:
 *       Retrieve an egress QoS policy.
 *       The result is returned in event of type  MV_NSS_DP_EVT_PRIO_POLICY.
 *
 * Parameters:
 *       policy_id - ID of the priority policy to retrieve. Must be 0.
 *       res - Result delivery specification.
 *
 * Returns:
 *        MV_NSS_DP_OK - On success or an error code, otherwise.
 */
mv_nss_dp_status_t mv_nss_dp_egress_qos_policy_get(uint8_t policy_id, const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s\n", __func__);
	rc = MV_NSS_DP_FAILED;

	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_EGRESS_QOS_POLICY_GET;
	tx_msg.res = res;
	tx_msg.count = 1;

	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_EGRESS_QOS_POLICY_GET, &policy_id);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("EGRESS QOS POLICY GET Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("EGRESS QOS POLICY GET TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("EGRESS QOS POLICY GET OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:



/*consider returning the current rc*/
e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
EXPORT_SYMBOL(mv_nss_dp_egress_qos_policy_get);




/*
 * mv_nss_dp_egress_prio_cfg_set
 *
 * Description:
 *       Update egress priority configuration.
 *
 * Parameters:
 *       cfg - Pointer to egress priority configuration.
 *       res - Result event specification or NULL for no event.
 *
 * Returns:
 *       MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *
 *           .type  - MV_NSS_DP_EVT_EGRESS_PRIO_CFG_SET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *			 .status - Result of operation.
 *
 *       Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_egress_prio_cfg_set(const mv_nss_dp_egress_prio_cfg_t *cfg,
		const mv_nss_dp_result_spec_t *res)

{
#ifdef REV3
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;
	int			i;

	MV_DP_LOG_DBG3("ENTER: %s\n", __func__);
	rc = MV_NSS_DP_FAILED;

	if (!cfg) {
		rc = MV_NSS_DP_INVALID_PARAM;
		MV_DP_LOG_ERR("NULL Egress Q CFG\n", MV_DP_RC_ERR_INVALID_PARAM);
		goto err;
	}

	for (i = 0; i < MV_NSS_DP_EGR_ACTIVE_PRIO_NUM; i++) {
		if (mv_nss_egress_prio_to_vq_set(i, cfg->queue[i])) {
			rc = MV_NSS_DP_INVALID_PARAM;
			MV_DP_LOG_ERR("Fail setting egress prio for queue:%d\n", MV_DP_RC_ERR_INVALID_PARAM, i);
			goto e_ret;
		}
	}

	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_EGRESS_PRIO_CFG_SET;
	tx_msg.res = res;
	tx_msg.count = 1;

	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_EGRESS_PRIO_CFG_SET, cfg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("EGRESS PRIO CFG SET Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("EGRESS PRIO CFG SET TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("EGRESS PRIO CFG SET OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:



/*consider returning the current rc*/
e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
#endif
	return MV_NSS_DP_NOT_SUPPORTED;
}
/*EXPORT_SYMBOL(mv_nss_dp_egress_prio_cfg_set);*/




/*
 * mv_nss_dp_egress_prio_cfg_get
 *
 * Description:
 *       Retrieve egress priority configuration.
 *
 * Parameters:
 *       res - Result event specification.
 *
 * Returns:
 *       MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *
 *           .type  - MV_NSS_DP_EVT_EGRESS_PRIO_CFG_SET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *			 .status - Result of operation.
 *			 .params.ingress_prio_cfg - Egress priority configuration.
 *
 *
 *       Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_egress_prio_cfg_get(const mv_nss_dp_result_spec_t *res)
{
#ifdef REV3
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s\n", __func__);
	rc = MV_NSS_DP_FAILED;

	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_EGRESS_PRIO_CFG_GET;
	tx_msg.res = res;
	tx_msg.count = 1;

	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_EGRESS_PRIO_CFG_GET, NULL);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("EGRESS PRIO CFG GET Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("EGRESS PRIO CFG GET TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("EGRESS PRIO CFG GET TX OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:



/*consider returning the current rc*/
e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
#endif
	return MV_NSS_DP_NOT_SUPPORTED;
}
/*EXPORT_SYMBOL(mv_nss_dp_egress_prio_cfg_get);*/



/*
 * mv_nss_dp_client_set
 *
 * Description:
 *       Add or update a client record.
 *
 * Parameters:
 *       client - Client record to add or update.
 *       count - Number of records pointed by 'client', greater than or equal to 1.
 *       res    - Result delivery specification or NULL.
 *
 * Returns:
 *        MV_NSS_DP_OK - On success or an error code, otherwise.
 */

mv_nss_dp_status_t mv_nss_dp_client_set(const mv_nss_dp_client_t *client,
		uint32_t count,
		const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s client:%p count:%d\n", __func__, client, count);
	rc = MV_NSS_DP_FAILED;

#ifdef MV_DP_DEBUG
	if (!client) {
		MV_DP_LOG_ERR("NULL Client PTR\n", MV_DP_RC_ERR_NULL_PTR);
		rc = MV_NSS_DP_INVALID_PARAM;
		goto e_ret;
	}
#endif
	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_CLIENT_SET;
	tx_msg.res = res;
	tx_msg.count = count;

	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_CLIENT_SET, client);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("Client_set TX MSG Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, count, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("Client_set TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("Client Set TX OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:



/*consider returning the current rc*/
e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
EXPORT_SYMBOL(mv_nss_dp_client_set);


/*
 * mv_nss_dp_client_get
 *
 * Description:
 *       Retrieve client record.
 *
 * Parameters:
 *       client_id - ID of an existing Client to retrieve.
 *       res   - Result event specification.
 *
 * Returns:
 *       MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *
 *           .type - MV_NSS_DP_EVT_MC_BRIDGED_CFG_GET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation.
 *           .params.client - Client record retrieved.
 *
 *       Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_client_get(uint16_t client_id, const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s client id:%d\n", __func__, client_id);
	rc = MV_NSS_DP_FAILED;

	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_CLIENT_GET;
	tx_msg.res = res;
	tx_msg.count = 1;


	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_CLIENT_GET, &client_id);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("Client_get TX MSG Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("Client_get TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("Client GET TX OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
EXPORT_SYMBOL(mv_nss_dp_client_get);


mv_nss_dp_status_t mv_nss_dp_client_get_mac(const mv_nss_dp_l2_addr_t *l2_addr, const mv_nss_dp_result_spec_t *res)
{

	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s\n", __func__);
	rc = MV_NSS_DP_FAILED;

	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_CLIENT_GET_MAC;
	tx_msg.res = res;
	tx_msg.count = 1;

	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_CLIENT_GET_MAC, l2_addr);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("Client_get TX MSG Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("Client_get TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("Client GET TX OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}



/*
 * mv_nss_dp_client_delete
 *
 * Description:
 *       Remove an existing client record.
 *
 * Parameters:
 *       client_id - Client ID to remove.
 *       res - Result event specification or NULL for no event.
 *
 * Returns:
 *       MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *
 *           .type  - MV_NSS_DP_EVT_CLIENT_DELETE
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation.
 *           .count - Index of the last successfully processed
 *                    record in the 'l2_addr' array
 *
 *       Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_client_delete(uint16_t client_id,
		const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s client id:%d\n", __func__, client_id);
	rc = MV_NSS_DP_FAILED;

	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_CLIENT_DEL;
	tx_msg.res = res;
	tx_msg.count = 1;

	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_CLIENT_DEL, &client_id);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("Client_get TX MSG Build, msg_size:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, tx_msg.flags);
		goto err;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("Client_get TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("Client GET TX OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
EXPORT_SYMBOL(mv_nss_dp_client_delete);

/********************CLIENT END**********************************************/

/*
 * mv_nss_dp_system_dim_get
 *
 * Description:
 *       Return system dimensions.
 *
 * Parameters:
 *       type - Type of system entity.
 *       res  - Result event specification.
 *
 * Returns:
 *		 MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *			    .type - MV_NSS_DP_EVT_SYSTEM_DIM_GET
 *              .cookie - Equal to res->cookie
 *			    .xid - Equal to res->xid
 *			    .status - Result of operation
 *              .params.dim - Supported number of instances.
 *
 *		 Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_system_dim_get(mv_nss_dp_entity_type_t type,
		const mv_nss_dp_result_spec_t *res)
{

	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s vlan_id:%d\n", __func__, type);
	rc = MV_NSS_DP_FAILED;

	if (!MV_DP_MSG_DIM_TYPE_IS_OK(type)) {
		MV_DP_LOG_ERR("SYS DIMENSION ILLEGAL type:%d\n", MV_NSS_DP_INVALID_PARAM, type);
		goto e_ret;
	}

	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_SYS_DIMENSION_GET;
	tx_msg.res = res;
	tx_msg.count = 1;


	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_SYS_DIMENSION_GET, &type);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("SYS DIMENSION Get TX MSG Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("SYS DIMENSION Get TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("SYS DIMENSION Get TX OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:



e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
EXPORT_SYMBOL(mv_nss_dp_system_dim_get);


#ifdef REV3
/*sets policer only*/
static mv_nss_dp_status_t mv_nss_dp_ingress_policer_set(const mv_nss_dp_ingress_queue_cfg_t *queue,
							uint32_t count,
							const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s\n", __func__);
	rc = MV_NSS_DP_FAILED;

	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_INGRESS_QUEUE_CFG_SET;
	tx_msg.res = res;
	tx_msg.count = count;

	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_INGRESS_QUEUE_CFG_SET, queue);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("INGRESS QUEUE SET Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("INGRESS QUEUE SET TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("INGRESS QUEUE SET OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:



/*consider returning the current rc*/
e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}



static mv_nss_dp_status_t mv_nss_dp_ingress_policer_get(uint8_t queue, const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s\n", __func__);
	rc = MV_NSS_DP_FAILED;

	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_INGRESS_QUEUE_CFG_GET;
	tx_msg.res = res;
	tx_msg.count = 1;

	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_INGRESS_QUEUE_CFG_GET, &queue);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("INGRESS QUEUE GET Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("INGRESS QUEUE GET TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("INGRESS QUEUE GET OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:



/*consider returning the current rc*/
e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}


/*sets policer only*/
static mv_nss_dp_status_t mv_nss_dp_egress_policer_set(const mv_nss_dp_egress_queue_cfg_t *queue,
						       uint32_t count,
						       const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s\n", __func__);
	rc = MV_NSS_DP_FAILED;

	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_EGRESS_QUEUE_CFG_SET;
	tx_msg.res = res;
	tx_msg.count = count;

	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_EGRESS_QUEUE_CFG_SET, queue);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("EGRESS QUEUE SET Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("EGRESS QUEUE SET TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("EGRESS QUEUE SET OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:



/*consider returning the current rc*/
e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}


static mv_nss_dp_status_t mv_nss_dp_egress_policer_get(uint8_t queue, const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s\n", __func__);
	rc = MV_NSS_DP_FAILED;

	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_EGRESS_QUEUE_CFG_GET;
	tx_msg.res = res;
	tx_msg.count = 1;

	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_EGRESS_QUEUE_CFG_GET, &queue);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("EGRESS QUEUE GET Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("EGRESS QUEUE GET TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("EGRESS QUEUE GET OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:



/*consider returning the current rc*/
e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
#endif


/*
 * mv_nss_eth_link_state_get
 *
 * Description:
 *       Retrieve Ethernet physical link state.
 *
 * Parameters:
 *       port_id - ID of an existing Ethernet virtual port.
 *       res     - Result event specification.
 *
 * Returns:
 *    MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *           .type - MV_NSS_DP_EVT_ETH_LINK_STATE_GET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation
 *           .params.eth_link_state - Ethernet physical link state.
 *
 *    Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_eth_link_state_get(mv_nss_dp_port_id_t port_id,
		const mv_nss_dp_result_spec_t *res)
{
	enum mv_dp_rc		rc = MV_DP_RC_OK;
	mv_nss_dp_eth_link_state_t *link;
	struct mv_dp_ctx ctx;
	u32			speed = 0;

	link = kzalloc(sizeof(mv_nss_dp_eth_link_state_t), GFP_KERNEL);
	if (!link) {
		MV_DP_LOG_ERR("link state response alloc failed\n", MV_DP_RC_ERR_OUT_OF_RESOURCES);
		rc = MV_DP_RC_ERR_OUT_OF_RESOURCES;
		goto e_ret;
	}

	/*alloc data_out*/

	link->port_id = port_id;
#ifdef REV3
	if (mv_nss_emac_link_status_get(port_id, &link->is_up, &link->duplex, &speed)) {
		MV_DP_LOG_ERR("link state driver err for port:%u\n", MV_DP_RC_ERR_INVALID_PARAM, port_id);
		rc = MV_DP_RC_ERR_INVALID_PARAM;
		goto e_free;
	}
#endif
	link->speed = speed;


	ctx.data = link;
	ctx.size = sizeof(mv_nss_dp_eth_link_state_t);
	ctx.cnt = 1;
	ctx.opc = MV_DP_MSGID_PORT_LINK_GET;

	rc = mv_dp_msg_ctx_tx(&ctx, res);
/*e_free:*/
	if (rc != MV_DP_RC_OK)
		kfree(link);
e_ret:
	return rc;
}
EXPORT_SYMBOL(mv_nss_eth_link_state_get);

/*
 * mv_nss_dp_ingress_queue_stats_get
 *
 * Description:
 *       Get counters of a virtual Ingress queue.
 *
 * Parameters:
 *       queue - Virtual Ingress queue number starting from 0.
 *       res - Result event specification or NULL.
 *
 * Returns:
 *    MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *           .type - MV_NSS_DP_EVT_INGRESS_Q_STATS_GET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .count - Equal to 1
 *           .status - Result of operation
 *           .params.queue_stats - Pointer to queue counters record
 *
 *    Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_ingress_queue_stats_get(uint8_t queue,
		const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s queue:%d\n", __func__, queue);
	rc = MV_NSS_DP_FAILED;

	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_INGRESS_Q_STATS_GET;
	tx_msg.res = res;
	tx_msg.count = 1;

	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_INGRESS_Q_STATS_GET, &queue);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("INGRESS_Q_STATS_GET TX MSG Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, 1, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("INGRESS_Q_STATS_GET TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("INGRESS_Q_STATS_GET TX OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:



/*consider returning the current rc*/
e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
EXPORT_SYMBOL(mv_nss_dp_ingress_queue_stats_get);

/*
 * mv_nss_dp_ingress_queue_stats_reset
 *
 * Description:
 *       Reset counters of a virtual Ingress queue.
 *
 * Parameters:
 *       queue - Virtual Ingress queue number starting from 0.
 *       res - Result event specification or NULL.
 *
 * Returns:
 *    MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *           .type - MV_NSS_DP_EVT_INGRESS_Q_STATS_RESET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .count - Equal to 1
 *           .status - Result of operation
 *
 *    Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_ingress_queue_stats_reset(uint8_t queue,
		const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s queue:%d\n", __func__, queue);
	rc = MV_NSS_DP_FAILED;

	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_INGRESS_Q_STATS_RESET;
	tx_msg.res = res;
	tx_msg.count = 1;

	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_INGRESS_Q_STATS_RESET, &queue);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("INGRESS_Q_STATS_RESET TX MSG Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, 1, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("INGRESS_Q_STATS_RESET TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("INGRESS_Q_STATS_RESET TX OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:



/*consider returning the current rc*/
e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
EXPORT_SYMBOL(mv_nss_dp_ingress_queue_stats_reset);

/*
 * mv_nss_dp_egress_queue_stats_get
 *
 * Description:
 *       Get counters of a virtual Egress queue.
 *
 * Parameters:
 *       queue - Virtual Egress queue number starting from 0.
 *       res - Result event specification or NULL.
 *
 * Returns:
 *    MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *           .type - MV_NSS_DP_EVT_EGRESS_Q_STATS_GET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .count - Equal to 1
 *           .status - Result of operation
 *           .params.egress_queue_stats - Pointer to queue counters record
 *
 *    Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_egress_queue_stats_get(uint8_t queue,
		const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s queue:%d\n", __func__, queue);
	rc = MV_NSS_DP_FAILED;

	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_EGRESS_Q_STATS_GET;
	tx_msg.res = res;
	tx_msg.count = 1;

	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_EGRESS_Q_STATS_GET, &queue);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("EGRESS_Q_STATS_GET TX MSG Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, 1, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("EGRESS_Q_STATS_GET TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("EGRESS_Q_STATS_GET TX OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:


/*consider returning the current rc*/
e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
EXPORT_SYMBOL(mv_nss_dp_egress_queue_stats_get);

/*
 * mv_nss_dp_ingress_queue_stats_reset
 *
 * Description:
 *       Reset counters of a virtual Egress queue.
 *
 * Parameters:
 *       queue - Virtual Egress queue number starting from 0.
 *       res - Result event specification or NULL.
 *
 * Returns:
 *    MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via event with the following values
 *           of mv_nss_dp_event_t structure fields:
 *           .type - MV_NSS_DP_EVT_EGRESS_Q_STATS_RESET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .count - Equal to 1
 *           .status - Result of operation
 *
 *    Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_egress_queue_stats_reset(uint8_t queue,
		const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s queue:%d\n", __func__, queue);
	rc = MV_NSS_DP_FAILED;

	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_EGRESS_Q_STATS_RESET;
	tx_msg.res = res;
	tx_msg.count = 1;

	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_EGRESS_Q_STATS_RESET, &queue);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("EGRESS_Q_STATS_RESET TX MSG Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, 1, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("EGRESS_Q_STATS_RESET TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("EGRESS_Q_STATS_RESET TX OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:



/*consider returning the current rc*/
e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
EXPORT_SYMBOL(mv_nss_dp_egress_queue_stats_reset);


/*
 * mv_nss_dp_state_get
 *
 * Description:
 *       Retrieve current network sub-system state.
 *
 *       The API is working in synchronous mode only and res->cb must be equal to zero.
 *
 * Parameters:
 *       state - pointer for result.
 *       res - Result event specification or NULL.
 *
 * Returns:
 *    MV_NSS_DP_OK - Request has been accepted.
 *           The result is returned via state pointer with the values
 *           of mv_nss_dp_state_t enum.
 *
 *    Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_state_get(mv_nss_dp_state_t *state, const mv_nss_dp_result_spec_t *res)
{
	mv_nss_dp_status_t		rc;
	bool	pp3_status = false;

	rc = MV_NSS_DP_OK;
	MV_DP_LOG_DBG3("ENTER: %s\n", __func__);

	if (res->cb) {
		MV_DP_LOG_ERR("API only works in SYNC mode\n", MV_DP_RC_ERR_INVALID_PARAM);
		rc = MV_NSS_DP_INVALID_PARAM;
		goto e_ret;
	}

	if (!state) {
		MV_DP_LOG_ERR("State pointer is NULL\n", MV_DP_RC_ERR_INVALID_PARAM);
		rc = MV_NSS_DP_INVALID_PARAM;
		goto e_ret;
	}

	if (!MV_DP_IS_ONLINE(mv_dp_instance.status))
		*state = MV_NSS_DP_STATE_NONINIT;
	else{
#ifdef REV3
		if (mv_pp3_fw_state_get(&pp3_status) != 0) {
			rc = MV_NSS_DP_FAILED;
			goto e_ret;
		}
#endif
		if (pp3_status)
			*state = MV_NSS_DP_STATE_LIVE;
		else
			*state = MV_NSS_DP_STATE_INACTIVE;
	}

e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
/*EXPORT_SYMBOL(mv_nss_dp_state_get);*/



/*
 * mv_nss_dp_bypass_state_set
 *
 * Description:
 *       Set bypass enabled or disabled
 *
 * Parameters:
 *    state  - bypass state.
 *
 * Returns:
 *       MV_NSS_DP_OK - Request has been accepted.
 *         The result is returned via event with the following values
 *         of mv_nss_dp_event_t structure fields:
 *
 *           .type - MV_NSS_DP_EVT_MIRROR_CFG_SET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation
 *
 *       Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_bypass_state_set(const uint8_t enable,
		const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s enable:%d\n", __func__, enable);
	rc = MV_NSS_DP_FAILED;

	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_BYPASS_STATE_SET;
	tx_msg.res = res;
	tx_msg.count = 1;

	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_BYPASS_STATE_SET, &enable);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("BYPASS_STATE_SET TX MSG Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, 1, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("BYPASS_STATE_SET TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("BYPASS_STATE_SET TX OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:


/*consider returning the current rc*/
e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;

}

/*
 * mv_nss_dp_bypass_state_get
 *
 * Description:
 *       Get bypass state
 *
 * Parameters:
 *       res - Result event specification.
 *
 * Returns:
 *       MV_NSS_DP_OK - Request has been accepted.
 *         The result is returned via event with the following values
 *         of mv_nss_dp_event_t structure fields:
 *
 *           .type - MV_NSS_DP_EVT_MIRROR_CFG_SET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation
 *           .params.bypass_state - Bypass state enabled or disabled.
 *
 *       Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_bypass_state_get(const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s\n", __func__);
	rc = MV_NSS_DP_FAILED;


	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_BYPASS_STATE_GET;
	tx_msg.res = res;
	tx_msg.count = 1;

	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_BYPASS_STATE_GET, NULL);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("BYPASS_STATE_GET TX MSG Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("BYPASS_STATE_GET TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("BYPASS_STATE_GET TX OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:



e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}

/*
 * mv_nss_dp_vxlan_vni_cfg_set
 *
 * Description:
 *       Set VXLAN VNI configuration
 *
 * Parameters:
 *    cfg  - Pointer to VXLAN VNI configuration.
 *
 * Returns:
 *       MV_NSS_DP_OK - Request has been accepted.
 *         The result is returned via event with the following values
 *         of mv_nss_dp_event_t structure fields:
 *
 *           .type - MV_NSS_DP_EVT_VXLAN_VNI_CFG_SET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation
 *
 *       Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_vxlan_vni_cfg_set(const mv_nss_dp_vxlan_vni_cfg_t *cfg,
		const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s cfg:%p\n", __func__, cfg);
	rc = MV_NSS_DP_FAILED;

	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_VXLAN_VNI_CFG_SET;
	tx_msg.res = res;
	tx_msg.count = 1;

	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_VXLAN_VNI_CFG_SET, cfg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("VXLAN_VNI_CFG_SET TX MSG Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, 1, tx_msg.flags);
		goto err;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("VXLAN_VNI_CFG_SET TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("VXLAN_VNI_CFG_SET TX OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

err:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
EXPORT_SYMBOL(mv_nss_dp_vxlan_vni_cfg_set);

/*
 * mv_nss_dp_vxlan_vni_cfg_get
 *
 * Description:
 *       Get VXLAN VNI configuration
 *
 * Parameters:
 *       cfg  - Pointer to VXLAN VNI configuration to retrieve.
 *       res  - Result event specification.
 *
 * Returns:
 *       MV_NSS_DP_OK - Request has been accepted.
 *         The result is returned via event with the following values
 *         of mv_nss_dp_event_t structure fields:
 *
 *           .type - MV_NSS_DP_EVT_VXLAN_VNI_CFG_GET
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation
 *           .params.vxlan_vni_cfg - VXLAN VNI configuration.
 *
 *       Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_vxlan_vni_cfg_get(const mv_nss_dp_vxlan_vni_cfg_t *cfg,
		const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s cfg:%p\n", __func__, cfg);
	rc = MV_NSS_DP_FAILED;


	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_VXLAN_VNI_CFG_GET;
	tx_msg.res = res;
	tx_msg.count = 1;

	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_VXLAN_VNI_CFG_GET, cfg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("VXLAN_VNI_CFG_GET TX MSG Build, msg_size:%d, count:%d, flags:0x%04X\n",
				MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("VXLAN_VNI_CFG_GET TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
				MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("VXLAN_VNI_CFG_GET TX OK msg_size:%d, count:%d, flags:0x%04X\n",
			tx_msg.msg_size, tx_msg.count, tx_msg.flags);

err:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
EXPORT_SYMBOL(mv_nss_dp_vxlan_vni_cfg_get);

/*
 * mv_nss_dp_vxlan_vni_cfg_delete
 *
 * Description:
 *       Delete VXLAN VNI configuration
 *
 * Parameters:
 *       cfg  - Pointer to VXLAN VNI configuration to delete.
 *       res - Result event specification.
 *
 * Returns:
 *       MV_NSS_DP_OK - Request has been accepted.
 *         The result is returned via event with the following values
 *         of mv_nss_dp_event_t structure fields:
 *
 *           .type - MV_NSS_DP_EVT_VXLAN_VNI_CFG_DELETE
 *           .cookie - Equal to res->cookie
 *           .xid - Equal to res->xid
 *           .status - Result of operation
 *
 *       Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_vxlan_vni_cfg_delete(const mv_nss_dp_vxlan_vni_cfg_t *cfg,
		const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s cfg:%p\n", __func__, cfg);
	rc = MV_NSS_DP_FAILED;


	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_VXLAN_VNI_CFG_DEL;
	tx_msg.res = res;
	tx_msg.count = 1;

	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_VXLAN_VNI_CFG_DEL, cfg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("VXLAN_VNI_CFG_DEL TX MSG Build, msg_size:%d, count:%d, flags:0x%04X\n",
				MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("VXLAN_VNI_CFG_DEL TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
				MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("VXLAN_VNI_CFG_DEL TX OK msg_size:%d, count:%d, flags:0x%04X\n",
			tx_msg.msg_size, tx_msg.count, tx_msg.flags);

err:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
EXPORT_SYMBOL(mv_nss_dp_vxlan_vni_cfg_delete);

/*
 * mv_nss_dp_register_notify_cb
 *
 * Description:
 *       Register application callback routine.
 *
 * Parameters:
 *    cb       - Application callback routine to be invoked upon async notification.
 *
 * Returns:
 *       MV_NSS_DP_OK - Request has been accepted.
 *       Error code - Request has been rejected.
 */
mv_nss_dp_status_t mv_nss_dp_register_notify_cb(mv_nss_dp_evt_handler cb)
{
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s cb:%p\n", __func__, cb);
	rc = MV_NSS_DP_FAILED;
	if (!cb) {
		MV_DP_LOG_ERR("NULL CB PTR\n", MV_DP_RC_ERR_NULL_PTR);
		rc = MV_NSS_DP_INVALID_PARAM;
		goto err;
	}
	/*init event handler*/
	rc = mv_dp_event_info_init(mv_dp_instance.event_info, cb);
	if (rc != MV_DP_RC_OK) {
		MV_DP_LOG_ERR("DPAPI CB notification registration FAILED rc:%d\n",
				MV_DP_RC_ERR_FAILED, rc);
		rc = MV_NSS_DP_FAILED;
		goto err;
	}

err:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}
EXPORT_SYMBOL(mv_nss_dp_register_notify_cb);
