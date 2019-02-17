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

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>

#include "mv_dp_types.h"
#include "mv_dp_int_if.h"
#include "mv_dp_sysfs.h"
#include "mv_dp_main.h"
#include "mv_dp_msg.h"
#include "mv_dp_ext_hdr.h"
#ifdef MV_DP_QOS_SHADOW
#include "mv_dp_qos.h"
#endif

#include "mv_gnic_custom_mgmt.h"

/*input parameters*/
/****************************************************************
* channel mode -- ch_mode
* ushort  -- 1 - shared channel is used (default),
*            0 - channel per cpu is used
*****************************************************************/
static unsigned short ch_mode = MV_DP_CH_MODE_SHARED;
/****************************************************************
* debug level - dbg_lvl
* unsigned int mode -- 0xF is print all
*****************************************************************/
static unsigned short dbg_lvl = MV_DP_DBG_LVL;

/****************************************************************
* number of outstanding pending API calls - rx_buff
*****************************************************************/
static unsigned short rx_buff = MV_DP_RX_BUFF_SIZE;


/**********************************************
*Data Structures
**********************************************/

struct mv_dp_main mv_dp_instance;

/**********************************************
*Forward Declarations
**********************************************/

static enum mv_dp_rc mv_dp_ch_data_init(struct mv_dp_ch_data *cpu[], u16 size);
static enum mv_dp_rc mv_dp_ch_data_release(struct mv_dp_ch_data *channel);
static enum mv_dp_rc mv_dp_msg_check_rx(struct mv_dp_msg_info_rx *msg_rx, struct mv_dp_msg_buf *msg);
static enum mv_dp_rc mv_dp_rx_buffer_init(struct mv_dp_msg_buf *buffer);
static enum mv_dp_rc mv_dp_rx_buffer_release(struct mv_dp_ch_data *ch);
static enum mv_dp_rc mv_dp_rx_buffer_age(struct mv_dp_ch_data *ch);
static enum mv_dp_rc mv_dp_rx_buf_delete(struct mv_dp_ch_data *tmp, struct mv_dp_msg_buf *rx_msg_buf, bool invoke);
static void mv_dp_msg_info_init(struct mv_dp_msg_info *msgs);
static void mv_dp_err_info_init(struct mv_dp_error_info *err);
static enum mv_dp_rc mv_dp_msg_parse(struct mv_dp_msg_info_rx *rx_msg, void *ptr_buf);
static enum mv_dp_rc mv_dp_event_parse(struct mv_dp_msg_info_rx *rx_msg);

#if 0
static void mv_dp_init_2(mv_nss_dp_event_t *event);
#endif
static enum mv_dp_rc mv_dp_event_handler_init(enum mv_dp_msg_id ev_id, mv_nss_dp_evt_handler evt);
static void mv_dp_clear_ch_counters(struct mv_dp_ch_data *ch);
static void mv_dp_timer_cb(unsigned long ch);
static struct mv_dp_msg_buf *mv_dp_rx_buffer_find_sn(struct mv_dp_msg_buf *buffer, u16 sn);

static enum mv_dp_rc mv_dp_ch_data_size_set(struct mv_dp_ch_data *channels[], u16 size);
static enum mv_dp_rc mv_dp_init_parse_msg(void *dest, void  *src);

static enum mv_dp_rc mv_dp_shutdown_parse_msg(void *msg, void  *ptr_buf);
static enum mv_dp_rc mv_dp_ch_offline_set(struct mv_dp_ch_data *ch);

static inline mv_nss_dp_evt_handler mv_dp_get_event_hndl(int event_id);

/*
 * mv_dpapi_init
 *
 * Description:
 *       nss_dpapi initialization routine as part of module init.
 *
 * Parameters:
 *       netdev - pointer to net device
 *
 * Returns:
 *        0 - On success or an error code, otherwise.
 */
int mv_dpapi_init(struct net_device *netdev)
{
	int rc;
	MV_DP_LOG_INF("DPAPI ver: %d.%d.%d INSTANTIATED IN %s mode\n",
		      MV_DP_API_MAJ_VER, MV_DP_API_MIN_VER, MV_DP_API_LOC_VER,
		      (ch_mode == MV_DP_CH_MODE_SHARED) ? "shared" : "per cpu");

	if (MV_DP_IS_INITIALISED(mv_dp_instance.status)) {
		MV_DP_LOG_ERR("mv_dpapi_init already initialized\n", MV_DP_RC_ERR_ALREADY_INITILIZED);
		return -EEXIST;
	}

	rc = mv_dp_sysfs_init();
	if (rc) {
		MV_DP_LOG_ERR("DPAPI sysfs init failed\n", MV_DP_RC_ERR_FAILED);
		return rc;
	}

	if (MV_DP_RC_OK != mv_dp_init_main(rx_buff, NULL, netdev)) {
		MV_DP_LOG_INF("mv_dpapi INITIALIZATION FAILED\n");
		return -EAGAIN;
	}

	return 0;
}

/*
 * mv_dpapi_exit
 *
 * Description:
 *       nss_dpapi exit routine as part or module exit.
 *
 * Parameters:
 *       None
 *
 * Returns:
 *       VOID
 */
void mv_dpapi_exit(void)
{

	if (MV_DP_IS_INITIALISED(mv_dp_instance.status))
		mv_dp_exit_main();

	mv_dp_sysfs_exit();

	MV_DP_LOG_INF("DPAPI EXIT\n");
}


/*
 * mv_dp_init
 *
 * Description:
 *       nss_dpapi initialization routine.
 *
 * Parameters:
 *       None
 *
 * Returns:
 *        0 - On success or an error code, otherwise.
 */


enum mv_dp_rc mv_dp_init_main(u16 rx_buff, mv_nss_dp_evt_handler evt_hdlr, struct net_device *netdev)
{

	int rc;

	if (MV_DP_IS_INITIALISED(mv_dp_instance.status)) {
		MV_DP_LOG_ERR("ALREADY INITIALIZED", MV_DP_RC_ERR_ALREADY_ONLINE);
		return -EPERM;
	}

	memset(&mv_dp_instance, 0, sizeof(struct mv_dp_main));


	MV_DP_MSG_RXTX_DISABLE(mv_dp_instance);

	MV_DP_LOG_INF("DPAPI INITIALIZING\n");
	/*hopefully these will be recieved as well from init struct*/
	mv_dp_instance.dbg_lvl = dbg_lvl;
	mv_dp_instance.ch_mode = ch_mode;
	mv_dp_instance.netdev = netdev;

	mv_dp_sysfs_init_entities();
	mv_dp_err_info_init(mv_dp_instance.error_info);
	mv_dp_event_info_init(mv_dp_instance.event_info, evt_hdlr);
	mv_dp_msg_info_init(mv_dp_instance.msg_info);
	rc = mv_dp_ch_data_init(mv_dp_instance.ch_data, rx_buff);
	if (MV_DP_RC_OK != rc)
		goto err;


	MV_DP_MSG_ALLOW_INTERNAL(mv_dp_instance);
	MV_DP_LOG_INF("DPAPI INITIALIZED\n");
	MV_DP_INITIALISED_SET(mv_dp_instance.status);

	return MV_DP_RC_OK;
err:
	MV_DP_LOG_ERR("DPAPI Init Failed\n", rc);
	mv_dp_exit_main();
	return MV_DP_RC_ERR_FAILED;

}

#if 0
/*callback for get version*/
static void mv_dp_init_2(mv_nss_dp_event_t *event)
{

	u32 ver;

	if (!event) {
		MV_DP_LOG_ERR("INIT FAILED: NULL VERSION EVENT RX\n", MV_DP_RC_ERR_NULL_PTR);
		goto err;
	}
	ver = *((u32 *)event->params.msg);

	if (event->status != MV_NSS_DP_OK) {
		MV_DP_LOG_ERR("INIT FAILED: NO FW VERSION ANSWER, Status:%d\n", MV_DP_RC_ERR_FW_VERSION, event->status);
		goto err;
	}

	if ((ver & MV_DP_FW_VERSION_MASK) != (MV_DP_FW_VERSION & MV_DP_FW_VERSION_MASK)) {
		MV_DP_LOG_ERR("INIT FAILED: WRONG FW VERSION:0x%08X, Expected:0x%08X, Status:%d\n",
			      MV_DP_RC_ERR_FW_VERSION, ver, MV_DP_FW_VERSION, event->status);
		goto err;
	}

	MV_DP_LOG_INF("FW DPAPI Version: ................%d.%d.%d\n",
		      MV_DP_MSG_VER_MAJ_GET(ver), MV_DP_MSG_VER_MID_GET(ver), MV_DP_MSG_VER_MIN_GET(ver));
	MV_DP_LOG_INF("DPAPI    Version: ................%d.%d.%d\n",
		      MV_DP_API_MAJ_VER, MV_DP_API_MIN_VER, MV_DP_API_LOC_VER);

/*
	MV_DP_ONLINE_SET(mv_dp_instance.status);
	MV_DP_MSG_ALLOW_MSG(mv_dp_instance);
*/
	MV_DP_LOG_INF("DPAPI INITIALIZED\n");
	return;
err:
	mv_dp_exit_main();
}
#endif

static enum mv_dp_rc mv_dp_ch_data_init(struct mv_dp_ch_data *channels[], u16 size)
{
	/*zero the structure -- already done*/
	int ret, ch = 0;
	int core;
	struct mv_dp_ch_data *tmp;

	/*if shared*/
	if (mv_dp_instance.ch_mode) {
		/*init single channel for all cores*/

		if (0 > ch) {
			MV_DP_LOG_ERR("Channel creation failed\n", MV_DP_RC_ERR_CH_CREATE);
			return MV_DP_RC_ERR_CH_CREATE;
		}
		tmp = kzalloc(sizeof(struct mv_dp_ch_data), GFP_KERNEL);
		if (!tmp) {
			mv_dp_chan_delete(ch);
			channels[ch] = 0;
			MV_DP_LOG_ERR("Error Allocating struct for ch:%d\n", MV_DP_RC_ERR_ALLOC, ch);
			return MV_DP_RC_ERR_ALLOC;
		}
		/* TODO define client number */
		ret = agnic_register_custom(mv_dp_instance.netdev, 2, (void *)tmp, mv_dp_msg_rx);
		if (ret) {
			MV_DP_LOG_ERR("agnic_register_custom failed\n", MV_DP_RC_ERR_FAILED);
			return MV_DP_RC_ERR_FAILED;
		}
		MV_DP_LOG_DBG3("Register mv_dp_msg_rx Done.\n");
		channels[ch] = tmp;
		spin_lock_init(&tmp->lock);
		tmp->ch_id = ch;
		tmp->cpu_id = smp_processor_id();
		tmp->max_rx_size = size;	/*counter should be a part of rx_buffer*/
		tmp->rx_size = 0;
		mv_dp_rx_buffer_init(&tmp->rx_buf);

		for_each_possible_cpu(core)
			mv_dp_instance.cpu_to_ch_id[core] = ch;
		mv_dp_channel_reg(ch,  tmp->cpu_id);
		/*init_timer: make it to run on the current cpu*/

		mv_dp_timer_setup(tmp);
		mv_dp_timer_set(tmp);
	} else {
		for_each_possible_cpu(core) {

			if (0 > ch) {
				MV_DP_LOG_ERR("Channel creation failed, cpu:%d\n", MV_DP_RC_ERR_CH_CREATE, core);
				return MV_DP_RC_ERR_CH_CREATE;
			}

			tmp = kzalloc(sizeof(struct mv_dp_ch_data), GFP_KERNEL);
			if (!tmp) {
				mv_dp_chan_delete(ch);
				channels[ch] = 0;
				MV_DP_LOG_ERR("Error Allocating channel struct: %d\n", MV_DP_RC_ERR_ALLOC, ch);
				return MV_DP_RC_ERR_ALLOC;
			}
			ret = agnic_register_custom(mv_dp_instance.netdev, 0, (void *)tmp, mv_dp_msg_rx);
			if (ret) {
				MV_DP_LOG_ERR("agnic_register_custom failed\n", MV_DP_RC_ERR_FAILED);
				return MV_DP_RC_ERR_FAILED;
			}
			channels[ch] = tmp;
			tmp->max_rx_size = size;	/*counter should be a part of rx_buffer*/
			tmp->rx_size = 0;
			tmp->ch_id = ch;
			tmp->cpu_id = smp_processor_id();
			mv_dp_rx_buffer_init(&tmp->rx_buf);
			spin_lock_init(&tmp->lock);
			mv_dp_instance.cpu_to_ch_id[core] = ch;
			mv_dp_channel_reg(ch,  tmp->cpu_id);
			/*init_timer: make it to run on the cpu_id core*/
			mv_dp_timer_setup(tmp);
			mv_dp_timer_set(tmp);
		}
	}

	return MV_DP_RC_OK;
}


static void mv_dp_err_info_init(struct mv_dp_error_info *errors)
{
	errors[MV_DP_RC_OK].name			= "NSS_OK";
	errors[MV_DP_RC_ERR_FAILED].name		= "NSS_FAILED";
	errors[MV_DP_RC_TOO_MANY_REQUESTS].name		= "NSS_TOOMANYREQTS";
	errors[MV_DP_RC_END_OF_LIST].name		= "NSS_END_OF_LIST";
	errors[MV_DP_RC_ERR_INVALID_PARAM].name		= "NSS_INVALID_PARAM";
	errors[MV_DP_RC_ERR_OUT_OF_RESOURCES].name	= "NSS_OUT_OF_RES";
	errors[MV_DP_RC_ITEM_NOT_FOUND].name		= "NSS_NOT_FOUND";
	errors[MV_DP_RC_EXEC_TIMEOUT].name		= "NSS_EXEC_TOUT";
	errors[MV_DP_RC_SHUTDOWN].name			= "NSS_SHUTDOWN";
	errors[MV_DP_RC_DP_NOT_SUPPORTED].name		= "NSS_SUPPORTED";
	errors[MV_DP_RC_ERR_ALLOC].name			= "ALLOCATION";
	errors[MV_DP_RC_ERR_MSG_TX].name		= "MSG_TX";
	errors[MV_DP_RC_ERR_MSG_NOT_FOUND].name		= "MSG_NOT_FOUND";
	errors[MV_DP_RC_ERR_MSG_SN_NOT_EQ].name		= "WRONG_SN";
	errors[MV_DP_RC_ERR_OFFLINE].name		= "DP_OFFLINE";
	errors[MV_DP_RC_ERR_ALREADY_INITILIZED].name	= "ALREADY_INIT";
	errors[MV_DP_RC_ERR_EVENT_PARSE_FAILED].name	= "EVTPARSEFAIL";
	errors[MV_DP_RC_ERR_MSG_PARSE_FAILED].name	= "MSGPARSEFAIL";
	errors[MV_DP_RC_ERR_MSG_SIZE].name		= "WRONGMSGSIZE";
	errors[MV_DP_RC_ERR_CH_CREATE].name		= "CHCREATE";
	errors[MV_DP_RC_ERR_COUNT].name			= "COUNT";
	errors[MV_DP_RC_ERR_FW_STATUS].name		= "RX_FW_STATUS";
	errors[MV_DP_RC_ERR_TIMER_DELETE].name		= "TMR_DELETE";
	errors[MV_DP_RC_ERR_AGED].name			= "MSG_AGED";
	errors[MV_DP_RC_ERR_NO_DP_KO].name		= "KO_NOTFOUND";
	errors[MV_DP_RC_ERR_NULL_PTR].name		= "NULL_PTR";
	errors[MV_DP_RC_ERR_MSGID_ILLEGAL].name		= "ILLEGAL_OPCODE";
	errors[MV_DP_RC_ERR_WRONG_RX_CHAN].name		= "WRONG_RX_CHAN";
	errors[MV_DP_RC_ERR_WRONG_CHAN].name		= "WRONG_CHAN";
	errors[MV_DP_RC_ERR_EXT_BUF_POPULATE].name	= "EXT_BUF_FILL";
	errors[MV_DP_RC_ERR_TIMER_SET].name		= "ERROR_TIMER_SET";
	errors[MV_DP_RC_ERR_MSG_RX].name		= "ERROR_MSG_RX";
	errors[MV_DP_RC_ERR_ALREADY_ONLINE].name	= "ALREADY_ONLINE";
	errors[MV_DP_RC_ERR_FW_VERSION].name		= "WRONG_FW_VER";
	errors[MV_DP_RC_ERR_EVENT_HNDL_FAILED].name	= "EVENT_HNDL_FAILED";
	errors[MV_DP_RC_ERR_MSG_HNDL_FAILED].name	= "MSG_HNDL_FAILED";
	errors[MV_DP_RC_ERR_WRONG_STATE].name		= "WRONG_STATE";
	errors[MV_DP_RC_ERR_BUILD_TX].name		= "ERROR_BUILD_TX";
}


void mv_dp_show_msg_info(void)
{
	int i;

	MV_DP_CLI_OK("|MSG INFO|%d|<\n", MV_DP_MSGID_LAST);
	MV_DP_LOG_CONT("|OPCODE|     MNEMONIC   |RX HANDLE|TX HANDLE|TXSIZE|RXSIZE|CBSIZE|TFLG|EVTN|TYPE| TRACE|<\n");

	for (i = 0; i < MV_DP_MSGID_LAST; i++) {
		MV_DP_LOG_CONT("|%6d|%16s|%9p|%9p|%6d|%6d|%6d|%04X|%4d| %c%c%c|%c%c%c%c%c|<\n", i,
			       mv_dp_instance.msg_info[i].name,
			       mv_dp_instance.msg_info[i].rx_handle,
			       mv_dp_instance.msg_info[i].tx_handle,
			       mv_dp_instance.msg_info[i].tx_size,
			       mv_dp_instance.msg_info[i].rx_size,
			       mv_dp_instance.msg_info[i].rx_cb_size,
			       mv_dp_instance.msg_info[i].tx_flags,
			       mv_dp_instance.msg_info[i].return_evt,
			       (MV_DP_MSG_IS_EVENT(mv_dp_instance.msg_info[i].type)) ? ('E') : ('_'),
			       (MV_DP_MSG_IS_INTERNAL(mv_dp_instance.msg_info[i].type)) ? ('I') : ('_'),
			       (MV_DP_MSG_IS_MSG(mv_dp_instance.msg_info[i].type)) ? ('M') : ('_'),

			       (MV_DP_LOG_TRC_ON(i, MV_DP_LOG_TRC_RX) ? ('R') : ('_')),
			       (MV_DP_LOG_TRC_ON(i, MV_DP_LOG_TRC_TX) ? ('T') : ('_')),
			       (MV_DP_LOG_TRC_ON(i, MV_DP_LOG_TRC_CB) ? ('C') : ('_')),
			       (MV_DP_LOG_TRC_ON(i, MV_DP_LOG_TRC_BUF) ? ('B') : ('_')),
			       (MV_DP_LOG_TRC_ON(i, MV_DP_LOG_TRC_EXT) ? ('E') : ('_')));
	}

	MV_DP_LOG_CONT("|MSG INFO END|<\n");
}


static void mv_dp_msg_info_init(struct mv_dp_msg_info *msgs)
{

	msgs[MV_DP_MSGID_NOP].type		= MV_DP_MSG_TYPE_INTERNAL | MV_DP_MSG_TYPE_RLS_BUF;
	msgs[MV_DP_MSGID_NOP].rx_size		= MV_DP_MSG_NOP_RX_SIZE;
	msgs[MV_DP_MSGID_NOP].tx_size		= MV_DP_MSG_NOP_TX_SIZE;
	msgs[MV_DP_MSGID_NOP].rx_handle		= mv_dp_nop_parse_msg;
	msgs[MV_DP_MSGID_NOP].tx_handle		= mv_dp_nop_populate_msg;
	msgs[MV_DP_MSGID_NOP].rx_cb_size	= MV_DP_MSG_NOP_RX_SIZE;
	msgs[MV_DP_MSGID_NOP].tx_flags		= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_NOP].return_evt	= MV_NSS_DP_EVT_NOTIFY_MSG;
	msgs[MV_DP_MSGID_NOP].name		= "NOP";
	msgs[MV_DP_MSGID_NOP].trace		= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_CTX].type		= MV_DP_MSG_TYPE_INTERNAL | MV_DP_MSG_TYPE_RLS_BUF;
	msgs[MV_DP_MSGID_CTX].rx_size		= MV_DP_MSG_CTX_RX_SIZE;
	msgs[MV_DP_MSGID_CTX].tx_size		= MV_DP_MSG_CTX_TX_SIZE;
	msgs[MV_DP_MSGID_CTX].rx_handle		= mv_dp_ctx_parse_msg;
	msgs[MV_DP_MSGID_CTX].tx_handle		= mv_dp_ctx_populate_msg;
	msgs[MV_DP_MSGID_CTX].rx_cb_size	= 0;
	msgs[MV_DP_MSGID_CTX].tx_flags		= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_CTX].return_evt	= MV_NSS_DP_EVT_NOTIFY_MSG;
	msgs[MV_DP_MSGID_CTX].name		= "CTX";
	msgs[MV_DP_MSGID_CTX].trace		= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_PORT_LINK_GET].type		= MV_DP_MSG_TYPE_MSG | MV_DP_MSG_TYPE_RLS_BUF;
	msgs[MV_DP_MSGID_PORT_LINK_GET].rx_size		= MV_DP_MSG_PORT_LINK_RX_SIZE;
	msgs[MV_DP_MSGID_PORT_LINK_GET].tx_size		= 0;
	msgs[MV_DP_MSGID_PORT_LINK_GET].rx_handle	= 0;
	msgs[MV_DP_MSGID_PORT_LINK_GET].tx_handle	= 0;
	msgs[MV_DP_MSGID_PORT_LINK_GET].rx_cb_size	= sizeof(mv_nss_dp_eth_link_state_t);
	msgs[MV_DP_MSGID_PORT_LINK_GET].tx_flags	= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_PORT_LINK_GET].return_evt	= MV_NSS_DP_EVT_ETH_LINK_STATE_GET;
	msgs[MV_DP_MSGID_PORT_LINK_GET].name		= "ETH_LINK_GET";
	msgs[MV_DP_MSGID_PORT_LINK_GET].trace		= MV_DP_LOG_TRC_NONE;


	msgs[MV_DP_MSGID_VER].type		= MV_DP_MSG_TYPE_INTERNAL;
	msgs[MV_DP_MSGID_VER].rx_size		= MV_DP_MSG_VER_RX_SIZE;
	msgs[MV_DP_MSGID_VER].tx_size		= MV_DP_MSG_VER_TX_SIZE;
	msgs[MV_DP_MSGID_VER].rx_handle		= mv_dp_ver_parse_msg;
	msgs[MV_DP_MSGID_VER].tx_handle		= 0;
	msgs[MV_DP_MSGID_VER].rx_cb_size	= MV_DP_MSG_VER_RX_SIZE;
	msgs[MV_DP_MSGID_VER].tx_flags		= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_VER].return_evt	= MV_NSS_DP_EVT_NOTIFY_MSG;
	msgs[MV_DP_MSGID_VER].name		= "GET VER";
	msgs[MV_DP_MSGID_VER].trace		= MV_DP_LOG_TRC_NONE;


	msgs[MV_DP_MSGID_INIT].type		= MV_DP_MSG_TYPE_INTERNAL;
	msgs[MV_DP_MSGID_INIT].rx_size		= MV_DP_MSG_INIT_RX_SIZE;
	msgs[MV_DP_MSGID_INIT].tx_size		= MV_DP_MSG_INIT_TX_SIZE;
	msgs[MV_DP_MSGID_INIT].rx_handle	= mv_dp_init_parse_msg;
	msgs[MV_DP_MSGID_INIT].tx_handle	= mv_dp_init_populate_msg;
	msgs[MV_DP_MSGID_INIT].rx_cb_size	= MV_DP_MSG_INIT_RX_SIZE;
	msgs[MV_DP_MSGID_INIT].tx_flags		= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_INIT].return_evt	= MV_NSS_DP_EVT_INIT;
	msgs[MV_DP_MSGID_INIT].name		= "INIT";
	msgs[MV_DP_MSGID_INIT].trace		= MV_DP_LOG_TRC_NONE;


	msgs[MV_DP_MSGID_SHUTDOWN].type		= MV_DP_MSG_TYPE_INTERNAL;
	msgs[MV_DP_MSGID_SHUTDOWN].rx_size	= MV_DP_MSG_SHUTDOWN_RX_SIZE;
	msgs[MV_DP_MSGID_SHUTDOWN].tx_size	= MV_DP_MSG_SHUTDOWN_TX_SIZE;
	msgs[MV_DP_MSGID_SHUTDOWN].rx_handle	= mv_dp_shutdown_parse_msg;
	msgs[MV_DP_MSGID_SHUTDOWN].tx_handle	= 0;
	msgs[MV_DP_MSGID_SHUTDOWN].rx_cb_size	= 0;
	msgs[MV_DP_MSGID_SHUTDOWN].tx_flags	= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_SHUTDOWN].return_evt	= MV_NSS_DP_EVT_SHUTDOWN;
	msgs[MV_DP_MSGID_SHUTDOWN].name		= "SHUTDOWN";
	msgs[MV_DP_MSGID_SHUTDOWN].trace	= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_CLIENT_SET].type	= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_CLIENT_SET].rx_size	= sizeof(uint16_t);
	msgs[MV_DP_MSGID_CLIENT_SET].tx_size	= sizeof(mv_nss_dp_client_t);
	msgs[MV_DP_MSGID_CLIENT_SET].rx_handle	= 0;
	msgs[MV_DP_MSGID_CLIENT_SET].rx_cb_size	= sizeof(uint16_t);
	msgs[MV_DP_MSGID_CLIENT_SET].tx_handle	= 0;
	msgs[MV_DP_MSGID_CLIENT_SET].tx_flags	= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_CLIENT_SET].return_evt	= MV_NSS_DP_EVT_CLIENT_SET;
	msgs[MV_DP_MSGID_CLIENT_SET].name	= "CLIENT SET";
	msgs[MV_DP_MSGID_CLIENT_SET].trace	= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_CLIENT_GET].type	= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_CLIENT_GET].rx_size = sizeof(mv_nss_dp_client_t);
	msgs[MV_DP_MSGID_CLIENT_GET].tx_size = sizeof(uint16_t);
	msgs[MV_DP_MSGID_CLIENT_GET].rx_handle = 0;
	msgs[MV_DP_MSGID_CLIENT_GET].tx_handle = 0;
	msgs[MV_DP_MSGID_CLIENT_GET].rx_cb_size = sizeof(mv_nss_dp_client_t);
	msgs[MV_DP_MSGID_CLIENT_GET].tx_flags = MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_CLIENT_GET].return_evt = MV_NSS_DP_EVT_CLIENT_GET;
	msgs[MV_DP_MSGID_CLIENT_GET].name	= "CLIENT GET";
	msgs[MV_DP_MSGID_CLIENT_GET].trace	= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_CLIENT_GET_MAC].type	= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_CLIENT_GET_MAC].rx_size = sizeof(mv_nss_dp_client_t);
	msgs[MV_DP_MSGID_CLIENT_GET_MAC].tx_size = sizeof(mv_nss_dp_l2_addr_t);
	msgs[MV_DP_MSGID_CLIENT_GET_MAC].rx_handle = 0;
	msgs[MV_DP_MSGID_CLIENT_GET_MAC].tx_handle = 0;
	msgs[MV_DP_MSGID_CLIENT_GET_MAC].rx_cb_size = sizeof(mv_nss_dp_client_t);
	msgs[MV_DP_MSGID_CLIENT_GET_MAC].tx_flags = MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_CLIENT_GET_MAC].return_evt = MV_NSS_DP_EVT_CLIENT_GET;
	msgs[MV_DP_MSGID_CLIENT_GET_MAC].name	= "CLIENT GET_MAC";
	msgs[MV_DP_MSGID_CLIENT_GET_MAC].trace	= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_CLIENT_DEL].type	= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_CLIENT_DEL].rx_size	= MV_DP_MSG_CLIENT_DEL_RX_SIZE;
	msgs[MV_DP_MSGID_CLIENT_DEL].tx_size	= sizeof(uint16_t);
	msgs[MV_DP_MSGID_CLIENT_DEL].rx_handle  = 0;
	msgs[MV_DP_MSGID_CLIENT_DEL].tx_handle	= 0;
	msgs[MV_DP_MSGID_CLIENT_DEL].rx_cb_size	= 0;
	msgs[MV_DP_MSGID_CLIENT_DEL].tx_flags	= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_CLIENT_DEL].return_evt	= MV_NSS_DP_EVT_CLIENT_DELETE;
	msgs[MV_DP_MSGID_CLIENT_DEL].name	= "CLIENT DEL";
	msgs[MV_DP_MSGID_CLIENT_DEL].trace	= MV_DP_LOG_TRC_NONE;


	msgs[MV_DP_MSGID_PORT_GET].type		= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_PORT_GET].rx_size	= sizeof(mv_nss_dp_port_t);
	msgs[MV_DP_MSGID_PORT_GET].tx_size	= sizeof(uint8_t);
	msgs[MV_DP_MSGID_PORT_GET].tx_handle	= 0;
	msgs[MV_DP_MSGID_PORT_GET].rx_handle	= 0;
	msgs[MV_DP_MSGID_PORT_GET].rx_cb_size	= sizeof(mv_nss_dp_port_t);
	msgs[MV_DP_MSGID_PORT_GET].tx_flags	= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_PORT_GET].return_evt	= MV_NSS_DP_EVT_PORT_GET;
	msgs[MV_DP_MSGID_PORT_GET].name		= "PORT GET";
	msgs[MV_DP_MSGID_PORT_GET].trace	= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_EPORT_GET].type	= 0;
	msgs[MV_DP_MSGID_EPORT_GET].rx_size	= MV_DP_MSG_EPORT_GET_RX_SIZE;
	msgs[MV_DP_MSGID_EPORT_GET].tx_size	= MV_DP_MSG_EPORT_GET_TX_SIZE;
	msgs[MV_DP_MSGID_EPORT_GET].rx_handle	= mv_dp_eport_parse_struct;
	msgs[MV_DP_MSGID_EPORT_GET].tx_handle	= 0;
	msgs[MV_DP_MSGID_EPORT_GET].rx_cb_size	= sizeof(mv_nss_dp_port_t);
	msgs[MV_DP_MSGID_EPORT_GET].tx_flags	= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_EPORT_GET].return_evt	= MV_NSS_DP_EVT_PORT_GET;
	msgs[MV_DP_MSGID_EPORT_GET].name	= "EPORT GET";
	msgs[MV_DP_MSGID_EPORT_GET].trace	= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_LAGPORT_GET].type	= 0;
	msgs[MV_DP_MSGID_LAGPORT_GET].rx_size	= MV_DP_MSG_LAGPORT_GET_RX_SIZE;
	msgs[MV_DP_MSGID_LAGPORT_GET].tx_size	= MV_DP_MSG_LAGPORT_GET_TX_SIZE;
	msgs[MV_DP_MSGID_LAGPORT_GET].rx_handle	= mv_dp_lagport_parse_struct;
	msgs[MV_DP_MSGID_LAGPORT_GET].tx_handle	= 0;
	msgs[MV_DP_MSGID_LAGPORT_GET].rx_cb_size = sizeof(mv_nss_dp_port_t);
	msgs[MV_DP_MSGID_LAGPORT_GET].tx_flags	= MV_DP_F_CFH_MSG_ACK; /*example only*/
	msgs[MV_DP_MSGID_LAGPORT_GET].return_evt = MV_NSS_DP_EVT_PORT_GET;
	msgs[MV_DP_MSGID_LAGPORT_GET].name	= "LAGPORT GET";
	msgs[MV_DP_MSGID_LAGPORT_GET].trace	= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_CWPORT_GET].type	= 0;
	msgs[MV_DP_MSGID_CWPORT_GET].rx_size	= sizeof(mv_nss_dp_port_t);
	msgs[MV_DP_MSGID_CWPORT_GET].tx_size	= MV_DP_MSG_CWPORT_GET_TX_SIZE;
	msgs[MV_DP_MSGID_CWPORT_GET].rx_handle	= mv_dp_cwport_parse_struct;
	msgs[MV_DP_MSGID_CWPORT_GET].tx_handle	= 0;
	msgs[MV_DP_MSGID_CWPORT_GET].rx_cb_size	= sizeof(mv_nss_dp_port_t);
	msgs[MV_DP_MSGID_CWPORT_GET].tx_flags	= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_CWPORT_GET].return_evt	= MV_NSS_DP_EVT_PORT_GET;
	msgs[MV_DP_MSGID_CWPORT_GET].name	= "CWPORT GET";
	msgs[MV_DP_MSGID_CWPORT_GET].trace	= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_CPUPORT_GET].type	= 0;
	msgs[MV_DP_MSGID_CPUPORT_GET].rx_size	= MV_DP_MSG_CPUPORT_GET_RX_SIZE;
	msgs[MV_DP_MSGID_CPUPORT_GET].tx_size	= MV_DP_MSG_CPUPORT_GET_TX_SIZE;
	msgs[MV_DP_MSGID_CPUPORT_GET].rx_handle	= mv_dp_cpuport_parse_struct;
	msgs[MV_DP_MSGID_CPUPORT_GET].tx_handle	= 0;
	msgs[MV_DP_MSGID_CPUPORT_GET].rx_cb_size = sizeof(mv_nss_dp_port_t);
	msgs[MV_DP_MSGID_CPUPORT_GET].tx_flags	= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_CPUPORT_GET].return_evt = MV_NSS_DP_EVT_PORT_GET;
	msgs[MV_DP_MSGID_CPUPORT_GET].name	= "CPU PORT GET";
	msgs[MV_DP_MSGID_CPUPORT_GET].trace	= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_VXPORT_GET].type	= 0;
	msgs[MV_DP_MSGID_VXPORT_GET].rx_size	= sizeof(mv_nss_dp_port_t);
	msgs[MV_DP_MSGID_VXPORT_GET].tx_size	= sizeof(uint8_t);
	msgs[MV_DP_MSGID_VXPORT_GET].rx_handle	= 0;
	msgs[MV_DP_MSGID_VXPORT_GET].tx_handle	= 0;
	msgs[MV_DP_MSGID_VXPORT_GET].rx_cb_size	= sizeof(mv_nss_dp_port_t);
	msgs[MV_DP_MSGID_VXPORT_GET].tx_flags	= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_VXPORT_GET].return_evt	= MV_NSS_DP_EVT_PORT_GET;
	msgs[MV_DP_MSGID_VXPORT_GET].name	= "VXLAN PORT GET";
	msgs[MV_DP_MSGID_VXPORT_GET].trace	= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_PORT_SET].type		= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_PORT_SET].rx_size	= sizeof(uint8_t);
	msgs[MV_DP_MSGID_PORT_SET].tx_size	= MV_DP_MSG_PORT_SET_TX_SIZE;
	msgs[MV_DP_MSGID_PORT_SET].rx_handle	= 0;
	msgs[MV_DP_MSGID_PORT_SET].tx_handle	= 0;
	msgs[MV_DP_MSGID_PORT_SET].rx_cb_size	= sizeof(uint8_t);
	msgs[MV_DP_MSGID_PORT_SET].tx_flags	= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_PORT_SET].return_evt	= MV_NSS_DP_EVT_PORT_SET;
	msgs[MV_DP_MSGID_PORT_SET].name		= "PORT SET";
	msgs[MV_DP_MSGID_PORT_SET].trace	= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_EPORT_SET].type	= 0;
	msgs[MV_DP_MSGID_EPORT_SET].rx_size	= MV_DP_MSG_EPORT_SET_RX_SIZE;
	msgs[MV_DP_MSGID_EPORT_SET].tx_size	= MV_DP_MSG_EPORT_SET_TX_SIZE;
	msgs[MV_DP_MSGID_EPORT_SET].rx_handle	= 0;
	msgs[MV_DP_MSGID_EPORT_SET].tx_handle	= mv_dp_eport_populate_msg;
	msgs[MV_DP_MSGID_EPORT_SET].rx_cb_size	= 0;
	msgs[MV_DP_MSGID_EPORT_SET].tx_flags	= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_EPORT_SET].return_evt	= MV_NSS_DP_EVT_PORT_SET;
	msgs[MV_DP_MSGID_EPORT_SET].name	= "EPORT SET";
	msgs[MV_DP_MSGID_EPORT_SET].trace	= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_LAGPORT_SET].type	= 0;
	msgs[MV_DP_MSGID_LAGPORT_SET].rx_size	= MV_DP_MSG_LAGPORT_SET_RX_SIZE;
	msgs[MV_DP_MSGID_LAGPORT_SET].tx_size	= MV_DP_MSG_LAGPORT_SET_TX_SIZE;
	msgs[MV_DP_MSGID_LAGPORT_SET].rx_handle	= 0;
	msgs[MV_DP_MSGID_LAGPORT_SET].tx_handle	= mv_dp_lagport_populate_msg;
	msgs[MV_DP_MSGID_LAGPORT_SET].rx_cb_size = 0;
	msgs[MV_DP_MSGID_LAGPORT_SET].tx_flags	= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_LAGPORT_SET].return_evt = MV_NSS_DP_EVT_PORT_SET;
	msgs[MV_DP_MSGID_LAGPORT_SET].name	= "LAGPORT SET";
	msgs[MV_DP_MSGID_LAGPORT_SET].trace	= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_CPUPORT_SET].type	= 0;
	msgs[MV_DP_MSGID_CPUPORT_SET].rx_size	= MV_DP_MSG_CPUPORT_SET_RX_SIZE;
	msgs[MV_DP_MSGID_CPUPORT_SET].tx_size	= MV_DP_MSG_CPUPORT_SET_TX_SIZE;
	msgs[MV_DP_MSGID_CPUPORT_SET].rx_handle	= 0;
	msgs[MV_DP_MSGID_CPUPORT_SET].tx_handle	= mv_dp_cpuport_populate_msg;
	msgs[MV_DP_MSGID_CPUPORT_SET].rx_cb_size = 0;
	msgs[MV_DP_MSGID_CPUPORT_SET].tx_flags	= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_CPUPORT_SET].return_evt = MV_NSS_DP_EVT_PORT_SET;
	msgs[MV_DP_MSGID_CPUPORT_SET].name	= "CPU PORT SET";
	msgs[MV_DP_MSGID_CPUPORT_SET].trace	= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_CWPORT_SET].type	= 0;
	msgs[MV_DP_MSGID_CWPORT_SET].rx_size	= MV_DP_MSG_CWPORT_SET_RX_SIZE;
	msgs[MV_DP_MSGID_CWPORT_SET].tx_size	= sizeof(mv_nss_dp_port_t);
	msgs[MV_DP_MSGID_CWPORT_SET].rx_handle	= 0;
	msgs[MV_DP_MSGID_CWPORT_SET].tx_handle	= 0;
	msgs[MV_DP_MSGID_CWPORT_SET].rx_cb_size	= 0;
	msgs[MV_DP_MSGID_CWPORT_SET].tx_flags	= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_CWPORT_SET].return_evt	= MV_NSS_DP_EVT_PORT_SET;
	msgs[MV_DP_MSGID_CWPORT_SET].name	= "CWPORT SET";
	msgs[MV_DP_MSGID_CWPORT_SET].trace	= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_VXPORT_SET].type	= 0;
	msgs[MV_DP_MSGID_VXPORT_SET].rx_size	= sizeof(uint8_t);
	msgs[MV_DP_MSGID_VXPORT_SET].tx_size	= sizeof(mv_nss_dp_port_t);
	msgs[MV_DP_MSGID_VXPORT_SET].rx_handle	= 0;
	msgs[MV_DP_MSGID_VXPORT_SET].tx_handle	= 0;
	msgs[MV_DP_MSGID_VXPORT_SET].rx_cb_size	= 0;
	msgs[MV_DP_MSGID_VXPORT_SET].tx_flags	= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_VXPORT_SET].return_evt	= MV_NSS_DP_EVT_PORT_SET;
	msgs[MV_DP_MSGID_VXPORT_SET].name	= "VXLAN PORT SET";
	msgs[MV_DP_MSGID_VXPORT_SET].trace	= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_PORT_DEL].type		= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_PORT_DEL].rx_size	= MV_DP_MSG_PORT_DEL_RX_SIZE;
	msgs[MV_DP_MSGID_PORT_DEL].tx_size	= sizeof(uint8_t);
	msgs[MV_DP_MSGID_PORT_DEL].tx_handle	= 0;
	msgs[MV_DP_MSGID_PORT_DEL].rx_handle	= 0;
	msgs[MV_DP_MSGID_PORT_DEL].rx_cb_size	= 0;
	msgs[MV_DP_MSGID_PORT_DEL].tx_flags	= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_PORT_DEL].return_evt	= MV_NSS_DP_EVT_PORT_DELETE;
	msgs[MV_DP_MSGID_PORT_DEL].name		= "PORT DEL";
	msgs[MV_DP_MSGID_PORT_DEL].trace	= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_DP_PORT_DST_SET].type		= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_DP_PORT_DST_SET].rx_size	= MV_DP_MSG_PORT_DST_SET_CFG_RX_SIZE;
	msgs[MV_DP_MSGID_DP_PORT_DST_SET].tx_size	= MV_DP_MSG_PORT_DST_SET_CFG_TX_SIZE;
	msgs[MV_DP_MSGID_DP_PORT_DST_SET].tx_handle	= mv_dp_port_id_populate_msg;
	msgs[MV_DP_MSGID_DP_PORT_DST_SET].rx_handle	= 0;
	msgs[MV_DP_MSGID_DP_PORT_DST_SET].rx_cb_size	= 0;
	msgs[MV_DP_MSGID_DP_PORT_DST_SET].tx_flags	= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_DP_PORT_DST_SET].return_evt	= MV_NSS_DP_EVT_PORT_DST_SET;
	msgs[MV_DP_MSGID_DP_PORT_DST_SET].name		= "PORT DFLT_DST SET";
	msgs[MV_DP_MSGID_DP_PORT_DST_SET].trace		= MV_DP_LOG_TRC_NONE;


	msgs[MV_DP_MSGID_PORT_STATS_GET_BULK].type	= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_PORT_STATS_GET_BULK].rx_size	= MV_DP_MSG_PORT_STATS_GET_BULK_RX_SIZE;
	msgs[MV_DP_MSGID_PORT_STATS_GET_BULK].tx_size	= MV_DP_MSG_PORT_STATS_GET_BULK_TX_SIZE;
	msgs[MV_DP_MSGID_PORT_STATS_GET_BULK].rx_handle	= mv_dp_port_stats_parse_struct;
	msgs[MV_DP_MSGID_PORT_STATS_GET_BULK].tx_handle	= mv_dp_bulk_index_populate_msg;
	msgs[MV_DP_MSGID_PORT_STATS_GET_BULK].rx_cb_size = sizeof(mv_nss_dp_port_stats_t);
	msgs[MV_DP_MSGID_PORT_STATS_GET_BULK].tx_flags	= MV_DP_F_CFH_MSG_ACK; /*example only*/
	msgs[MV_DP_MSGID_PORT_STATS_GET_BULK].return_evt = MV_NSS_DP_EVT_PORT_STATS_GET;
	msgs[MV_DP_MSGID_PORT_STATS_GET_BULK].name	= "PORT STATS GET BULK";
	msgs[MV_DP_MSGID_PORT_STATS_GET_BULK].trace	= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_PORT_STATS_GET].type		= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_PORT_STATS_GET].rx_size	= sizeof(mv_nss_dp_port_stats_t);
	msgs[MV_DP_MSGID_PORT_STATS_GET].tx_size	= sizeof(uint8_t);
	msgs[MV_DP_MSGID_PORT_STATS_GET].rx_handle	= 0;
	msgs[MV_DP_MSGID_PORT_STATS_GET].tx_handle	= 0;
	msgs[MV_DP_MSGID_PORT_STATS_GET].rx_cb_size	= sizeof(mv_nss_dp_port_stats_t);
	msgs[MV_DP_MSGID_PORT_STATS_GET].tx_flags	= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_PORT_STATS_GET].return_evt	= MV_NSS_DP_EVT_PORT_STATS_GET;
	msgs[MV_DP_MSGID_PORT_STATS_GET].name		= "PORT STATS GET";
	msgs[MV_DP_MSGID_PORT_STATS_GET].trace		= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_PORT_STATS_RST].type		= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_PORT_STATS_RST].rx_size	= MV_DP_MSG_PORT_STATS_RST_RX_SIZE;
	msgs[MV_DP_MSGID_PORT_STATS_RST].tx_size	= sizeof(uint8_t);
	msgs[MV_DP_MSGID_PORT_STATS_RST].rx_handle	= 0;
	msgs[MV_DP_MSGID_PORT_STATS_RST].tx_handle	= 0;
	msgs[MV_DP_MSGID_PORT_STATS_RST].rx_cb_size	= 0;
	msgs[MV_DP_MSGID_PORT_STATS_RST].tx_flags	= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_PORT_STATS_RST].return_evt	= MV_NSS_DP_EVT_PORT_STATS_RESET;
	msgs[MV_DP_MSGID_PORT_STATS_RST].name		= "PORT STATS RST";
	msgs[MV_DP_MSGID_PORT_STATS_RST].trace		= MV_DP_LOG_TRC_NONE;


	msgs[MV_DP_MSGID_VLAN_CFG_SET].type		= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_VLAN_CFG_SET].rx_size		= MV_DP_MSG_VLAN_CFG_SET_RX_SIZE;
	msgs[MV_DP_MSGID_VLAN_CFG_SET].tx_size		= MV_DP_MSG_VLAN_CFG_SET_TX_SIZE;
	msgs[MV_DP_MSGID_VLAN_CFG_SET].rx_handle	= 0;
	msgs[MV_DP_MSGID_VLAN_CFG_SET].rx_cb_size	= 0;
	msgs[MV_DP_MSGID_VLAN_CFG_SET].tx_handle	= mv_dp_vlan_populate_msg;
	msgs[MV_DP_MSGID_VLAN_CFG_SET].tx_flags		= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_VLAN_CFG_SET].return_evt	= MV_NSS_DP_EVT_VLAN_CFG_SET;
	msgs[MV_DP_MSGID_VLAN_CFG_SET].name		= "VLAN SET";
	msgs[MV_DP_MSGID_VLAN_CFG_SET].trace		= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_VLAN_CFG_GET].type		= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_VLAN_CFG_GET].rx_size		= MV_DP_MSG_VLAN_CFG_GET_RX_SIZE;
	msgs[MV_DP_MSGID_VLAN_CFG_GET].tx_size		= MV_DP_MSG_VLAN_CFG_GET_TX_SIZE;
	msgs[MV_DP_MSGID_VLAN_CFG_GET].rx_handle	= mv_dp_vlan_parse_struct;
	msgs[MV_DP_MSGID_VLAN_CFG_GET].tx_handle	= mv_dp_vlan_ind_populate_msg;
	msgs[MV_DP_MSGID_VLAN_CFG_GET].rx_cb_size	= sizeof(mv_nss_dp_vlan_cfg_t);
	msgs[MV_DP_MSGID_VLAN_CFG_GET].tx_flags		= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_VLAN_CFG_GET].return_evt	= MV_NSS_DP_EVT_VLAN_CFG_GET;
	msgs[MV_DP_MSGID_VLAN_CFG_GET].name		= "VLAN GET";
	msgs[MV_DP_MSGID_VLAN_CFG_GET].trace		= MV_DP_LOG_TRC_NONE;


	msgs[MV_DP_MSGID_VLAN_CFG_GET_BY_ID].type	= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_VLAN_CFG_GET_BY_ID].rx_size	= MV_DP_MSG_VLAN_CFG_GET_BY_ID_RX_SIZE;
	msgs[MV_DP_MSGID_VLAN_CFG_GET_BY_ID].tx_size	= MV_DP_MSG_VLAN_CFG_GET_BY_ID_TX_SIZE;
	msgs[MV_DP_MSGID_VLAN_CFG_GET_BY_ID].rx_handle	= mv_dp_vlan_parse_struct;
	msgs[MV_DP_MSGID_VLAN_CFG_GET_BY_ID].tx_handle	= mv_dp_vlan_id_populate_msg;
	msgs[MV_DP_MSGID_VLAN_CFG_GET_BY_ID].rx_cb_size	= sizeof(mv_nss_dp_vlan_cfg_t);
	msgs[MV_DP_MSGID_VLAN_CFG_GET_BY_ID].tx_flags	= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_VLAN_CFG_GET_BY_ID].return_evt	= MV_NSS_DP_EVT_VLAN_CFG_GET;
	msgs[MV_DP_MSGID_VLAN_CFG_GET_BY_ID].name	= "VLAN GET BY ID";
	msgs[MV_DP_MSGID_VLAN_CFG_GET_BY_ID].trace	= MV_DP_LOG_TRC_NONE;


	msgs[MV_DP_MSGID_VLAN_CFG_DEL].type		= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_VLAN_CFG_DEL].rx_size		= MV_DP_MSG_VLAN_CFG_DEL_RX_SIZE;
	msgs[MV_DP_MSGID_VLAN_CFG_DEL].tx_size		= MV_DP_MSG_VLAN_CFG_DEL_TX_SIZE;
	msgs[MV_DP_MSGID_VLAN_CFG_DEL].rx_handle	= 0;
	msgs[MV_DP_MSGID_VLAN_CFG_DEL].tx_handle	= mv_dp_vlan_id_populate_msg;
	msgs[MV_DP_MSGID_VLAN_CFG_DEL].rx_cb_size	= 0;
	msgs[MV_DP_MSGID_VLAN_CFG_DEL].tx_flags		= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_VLAN_CFG_DEL].return_evt	= MV_NSS_DP_EVT_VLAN_CFG_DELETE;
	msgs[MV_DP_MSGID_VLAN_CFG_DEL].name		= "VLAN DEL";
	msgs[MV_DP_MSGID_VLAN_CFG_DEL].trace		= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_FLOW_GET].type			= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_FLOW_GET].rx_size		= MV_DP_MSG_FLOW_GET_RX_SIZE;
	msgs[MV_DP_MSGID_FLOW_GET].tx_size		= MV_DP_MSG_FLOW_GET_TX_SIZE;
	msgs[MV_DP_MSGID_FLOW_GET].rx_handle		= mv_dp_flow_parse_struct;
	msgs[MV_DP_MSGID_FLOW_GET].tx_handle		= mv_dp_flow_id_populate_msg;
	msgs[MV_DP_MSGID_FLOW_GET].rx_cb_size		= sizeof(mv_nss_dp_flow_t);
	msgs[MV_DP_MSGID_FLOW_GET].tx_flags		= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_FLOW_GET].return_evt		= MV_NSS_DP_EVT_FLOW_GET;
	msgs[MV_DP_MSGID_FLOW_GET].name			= "FLOW GET";
	msgs[MV_DP_MSGID_FLOW_GET].trace		= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_FLOW_SET].type			= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_FLOW_SET].rx_size		= MV_DP_MSG_FLOW_SET_RX_SIZE;
	msgs[MV_DP_MSGID_FLOW_SET].tx_size		= MV_DP_MSG_FLOW_SET_TX_SIZE;
	msgs[MV_DP_MSGID_FLOW_SET].rx_handle		= mv_dp_flow_parse_struct;
	msgs[MV_DP_MSGID_FLOW_SET].tx_handle		= mv_dp_flow_populate_msg;
	msgs[MV_DP_MSGID_FLOW_SET].rx_cb_size		= sizeof(mv_nss_dp_flow_t);
	msgs[MV_DP_MSGID_FLOW_SET].tx_flags		= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_FLOW_SET].return_evt		= MV_NSS_DP_EVT_FLOW_SET;
	msgs[MV_DP_MSGID_FLOW_SET].name			= "FLOW SET";
	msgs[MV_DP_MSGID_FLOW_SET].trace		= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_FLOW_DELETE].type		= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_FLOW_DELETE].rx_size		= MV_DP_MSG_FLOW_DELETE_RX_SIZE;
	msgs[MV_DP_MSGID_FLOW_DELETE].tx_size		= MV_DP_MSG_FLOW_DELETE_TX_SIZE;
	msgs[MV_DP_MSGID_FLOW_DELETE].rx_handle		= 0;
	msgs[MV_DP_MSGID_FLOW_DELETE].tx_handle		= mv_dp_flow_id_populate_msg;
	msgs[MV_DP_MSGID_FLOW_DELETE].rx_cb_size	= 0;
	msgs[MV_DP_MSGID_FLOW_DELETE].tx_flags		= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_FLOW_DELETE].return_evt	= MV_NSS_DP_EVT_FLOW_DELETE;
	msgs[MV_DP_MSGID_FLOW_DELETE].name		= "FLOW DELETE";
	msgs[MV_DP_MSGID_FLOW_DELETE].trace		= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_FLOW_COUNT_GET].type		= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_FLOW_COUNT_GET].rx_size	= MV_DP_MSG_FLOW_COUNT_GET_RX_SIZE;
	msgs[MV_DP_MSGID_FLOW_COUNT_GET].tx_size	= MV_DP_MSG_FLOW_COUNT_GET_TX_SIZE;
	msgs[MV_DP_MSGID_FLOW_COUNT_GET].rx_handle	= mv_dp_flow_count_parse_struct;
	msgs[MV_DP_MSGID_FLOW_COUNT_GET].tx_handle	= 0;
	msgs[MV_DP_MSGID_FLOW_COUNT_GET].rx_cb_size	= sizeof(uint32_t);
	msgs[MV_DP_MSGID_FLOW_COUNT_GET].tx_flags	= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_FLOW_COUNT_GET].return_evt	= MV_NSS_DP_EVT_FLOW_COUNT_GET;
	msgs[MV_DP_MSGID_FLOW_COUNT_GET].name		= "FLOW GET COUNT";
	msgs[MV_DP_MSGID_FLOW_COUNT_GET].trace		= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_FLOW_DELETE_ALL].type		= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_FLOW_DELETE_ALL].rx_size	= MV_DP_MSG_FLOW_DELETE_ALL_RX_SIZE;
	msgs[MV_DP_MSGID_FLOW_DELETE_ALL].tx_size	= MV_DP_MSG_FLOW_DELETE_ALL_TX_SIZE;
	msgs[MV_DP_MSGID_FLOW_DELETE_ALL].rx_handle	= 0;
	msgs[MV_DP_MSGID_FLOW_DELETE_ALL].tx_handle	= 0;
	msgs[MV_DP_MSGID_FLOW_DELETE_ALL].rx_cb_size	= 0;
	msgs[MV_DP_MSGID_FLOW_DELETE_ALL].tx_flags	= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_FLOW_DELETE_ALL].return_evt	= MV_NSS_DP_EVT_FLOW_DELETE;
	msgs[MV_DP_MSGID_FLOW_DELETE_ALL].name		= "FLOW DELETE ALL";
	msgs[MV_DP_MSGID_FLOW_DELETE_ALL].trace		= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_FLOW_STATUS_GET].type		= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_FLOW_STATUS_GET].rx_size	= MV_DP_MSG_FLOW_STATUS_GET_RX_SIZE;
	msgs[MV_DP_MSGID_FLOW_STATUS_GET].tx_size	= MV_DP_MSG_FLOW_STATUS_GET_TX_SIZE;
	msgs[MV_DP_MSGID_FLOW_STATUS_GET].rx_handle	= mv_dp_flow_status_parse_struct;
	msgs[MV_DP_MSGID_FLOW_STATUS_GET].tx_handle	= mv_dp_flow_id_populate_msg;
	msgs[MV_DP_MSGID_FLOW_STATUS_GET].rx_cb_size	= sizeof(mv_nss_dp_flow_status_t);
	msgs[MV_DP_MSGID_FLOW_STATUS_GET].tx_flags	= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_FLOW_STATUS_GET].return_evt	= MV_NSS_DP_EVT_FLOW_STATUS_GET;
	msgs[MV_DP_MSGID_FLOW_STATUS_GET].name		= "FLOW STATUS GET";
	msgs[MV_DP_MSGID_FLOW_STATUS_GET].trace		= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_FLOW_STATUS_SET].type		= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_FLOW_STATUS_SET].rx_size	= MV_DP_MSG_FLOW_STATUS_SET_RX_SIZE;
	msgs[MV_DP_MSGID_FLOW_STATUS_SET].tx_size	= MV_DP_MSG_FLOW_STATUS_SET_TX_SIZE;
	msgs[MV_DP_MSGID_FLOW_STATUS_SET].rx_handle	= mv_dp_flow_status_parse_struct;
	msgs[MV_DP_MSGID_FLOW_STATUS_SET].tx_handle	= mv_dp_flow_status_populate_msg;
	msgs[MV_DP_MSGID_FLOW_STATUS_SET].rx_cb_size	= sizeof(mv_nss_dp_flow_status_t);
	msgs[MV_DP_MSGID_FLOW_STATUS_SET].tx_flags	= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_FLOW_STATUS_SET].return_evt	= MV_NSS_DP_EVT_FLOW_STATUS_SET;
	msgs[MV_DP_MSGID_FLOW_STATUS_SET].name		= "FLOW STATUS SET";
	msgs[MV_DP_MSGID_FLOW_STATUS_SET].trace		= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_FLOW_STATS_GET].type		= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_FLOW_STATS_GET].rx_size	= MV_DP_MSG_FLOW_STATS_GET_RX_SIZE;
	msgs[MV_DP_MSGID_FLOW_STATS_GET].tx_size	= MV_DP_MSG_FLOW_STATS_GET_TX_SIZE;
	msgs[MV_DP_MSGID_FLOW_STATS_GET].rx_handle	= mv_dp_flow_stats_parse_struct;
	msgs[MV_DP_MSGID_FLOW_STATS_GET].tx_handle	= mv_dp_flow_id_populate_msg;
	msgs[MV_DP_MSGID_FLOW_STATS_GET].rx_cb_size	= sizeof(mv_nss_dp_flow_stats_t);
	msgs[MV_DP_MSGID_FLOW_STATS_GET].tx_flags	= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_FLOW_STATS_GET].return_evt	= MV_NSS_DP_EVT_FLOW_STATS_GET;
	msgs[MV_DP_MSGID_FLOW_STATS_GET].name		= "FLOW STATS GET";
	msgs[MV_DP_MSGID_FLOW_STATS_GET].trace		= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_FLOW_STATS_GET_BULK].type	= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_FLOW_STATS_GET_BULK].rx_size	= MV_DP_MSG_FLOW_STATS_GET_BULK_RX_SIZE;
	msgs[MV_DP_MSGID_FLOW_STATS_GET_BULK].tx_size	= MV_DP_MSG_FLOW_STATS_GET_BULK_TX_SIZE;
	msgs[MV_DP_MSGID_FLOW_STATS_GET_BULK].rx_handle	= mv_dp_flow_stats_parse_struct;
	msgs[MV_DP_MSGID_FLOW_STATS_GET_BULK].tx_handle	= mv_dp_bulk_options_populate_msg;
	msgs[MV_DP_MSGID_FLOW_STATS_GET_BULK].rx_cb_size = sizeof(mv_nss_dp_flow_stats_t);
	msgs[MV_DP_MSGID_FLOW_STATS_GET_BULK].tx_flags	= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_FLOW_STATS_GET_BULK].return_evt = MV_NSS_DP_EVT_FLOW_STATS_GET;
	msgs[MV_DP_MSGID_FLOW_STATS_GET_BULK].name	= "FLOW STATS GET BULK";
	msgs[MV_DP_MSGID_FLOW_STATS_GET_BULK].trace	= MV_DP_LOG_TRC_NONE;


	msgs[MV_DP_MSGID_FLOW_STATS_RESET].type		= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_FLOW_STATS_RESET].rx_size	= MV_DP_MSG_FLOW_STATS_RESET_RX_SIZE;
	msgs[MV_DP_MSGID_FLOW_STATS_RESET].tx_size	= MV_DP_MSG_FLOW_STATS_RESET_TX_SIZE;
	msgs[MV_DP_MSGID_FLOW_STATS_RESET].rx_handle	= 0;
	msgs[MV_DP_MSGID_FLOW_STATS_RESET].tx_handle	= mv_dp_flow_id_populate_msg;
	msgs[MV_DP_MSGID_FLOW_STATS_RESET].rx_cb_size	= 0;
	msgs[MV_DP_MSGID_FLOW_STATS_RESET].tx_flags	= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_FLOW_STATS_RESET].return_evt	= MV_NSS_DP_EVT_FLOW_STATS_RESET;
	msgs[MV_DP_MSGID_FLOW_STATS_RESET].name		= "FLOW STATS RESET";
	msgs[MV_DP_MSGID_FLOW_STATS_RESET].trace	= MV_DP_LOG_TRC_NONE;


	msgs[MV_DP_MSGID_EVT_SYS].type			= MV_DP_MSG_TYPE_EVENT;
	msgs[MV_DP_MSGID_EVT_SYS].rx_size		= MV_DP_MSG_EVT_RX_SIZE;
	msgs[MV_DP_MSGID_EVT_SYS].rx_handle		= mv_dp_msg_event_sys_parse;
	msgs[MV_DP_MSGID_EVT_SYS].tx_handle		= 0;
	msgs[MV_DP_MSGID_EVT_SYS].rx_cb_size		= sizeof(uint32_t);
	msgs[MV_DP_MSGID_EVT_SYS].return_evt		= MV_NSS_DP_EVT_NOTIFY_CODE;
	msgs[MV_DP_MSGID_EVT_SYS].name			= "EVT_SYS";
	msgs[MV_DP_MSGID_EVT_SYS].trace			= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_EVT_SYS_MSG].type		= MV_DP_MSG_TYPE_EVENT;
	msgs[MV_DP_MSGID_EVT_SYS_MSG].rx_size		= MV_DP_MSG_EVT_TEXT_RX_SIZE;
	msgs[MV_DP_MSGID_EVT_SYS_MSG].tx_handle		= 0;
	msgs[MV_DP_MSGID_EVT_SYS_MSG].rx_handle		= mv_dp_msg_event_sys_msg_parse;
	msgs[MV_DP_MSGID_EVT_SYS_MSG].rx_cb_size	= MV_DP_MSG_EVT_TEXT_MAX_SIZE;
	msgs[MV_DP_MSGID_EVT_SYS_MSG].return_evt	= MV_NSS_DP_EVT_NOTIFY_MSG;
	msgs[MV_DP_MSGID_EVT_SYS_MSG].name		= "EVT_SYS_MSG";
	msgs[MV_DP_MSGID_EVT_SYS_MSG].trace		= MV_DP_LOG_TRC_NONE;



	msgs[MV_DP_MSGID_DTLS_SET].type		= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_DTLS_SET].rx_size	= MV_DP_MSG_DTLS_SET_RX_SIZE;
	msgs[MV_DP_MSGID_DTLS_SET].tx_size	= sizeof(mv_nss_dp_dtls_t);
	msgs[MV_DP_MSGID_DTLS_SET].rx_handle	= 0;
	msgs[MV_DP_MSGID_DTLS_SET].rx_cb_size	= 0;
	msgs[MV_DP_MSGID_DTLS_SET].tx_handle	= 0;
	msgs[MV_DP_MSGID_DTLS_SET].tx_flags	= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_DTLS_SET].return_evt	= MV_NSS_DP_EVT_DTLS_SET;
	msgs[MV_DP_MSGID_DTLS_SET].name		= "DTLS SET";
	msgs[MV_DP_MSGID_DTLS_SET].trace	= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_DTLS_GET].type		= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_DTLS_GET].rx_size	= sizeof(mv_nss_dp_dtls_t);
	msgs[MV_DP_MSGID_DTLS_GET].tx_size	= sizeof(uint16_t);
	msgs[MV_DP_MSGID_DTLS_GET].rx_handle	= 0;
	msgs[MV_DP_MSGID_DTLS_GET].tx_handle	= 0;
	msgs[MV_DP_MSGID_DTLS_GET].rx_cb_size	= sizeof(mv_nss_dp_dtls_t);
	msgs[MV_DP_MSGID_DTLS_GET].tx_flags	= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_DTLS_GET].return_evt	= MV_NSS_DP_EVT_DTLS_GET;
	msgs[MV_DP_MSGID_DTLS_GET].name		= "DTLS GET";
	msgs[MV_DP_MSGID_DTLS_GET].trace	= MV_DP_LOG_TRC_NONE;


	msgs[MV_DP_MSGID_DTLS_DELETE].type		= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_DTLS_DELETE].rx_size		= MV_DP_MSG_DTLS_DELETE_RX_SIZE;
	msgs[MV_DP_MSGID_DTLS_DELETE].tx_size		= sizeof(uint16_t);
	msgs[MV_DP_MSGID_DTLS_DELETE].rx_handle		= 0;
	msgs[MV_DP_MSGID_DTLS_DELETE].tx_handle		= 0;
	msgs[MV_DP_MSGID_DTLS_DELETE].rx_cb_size	= 0;
	msgs[MV_DP_MSGID_DTLS_DELETE].tx_flags		= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_DTLS_DELETE].return_evt	= MV_NSS_DP_EVT_DTLS_DELETE;
	msgs[MV_DP_MSGID_DTLS_DELETE].name		= "DTLS DELETE";
	msgs[MV_DP_MSGID_DTLS_DELETE].trace		= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_INGRESS_PRIO_CFG_SET].type		= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_INGRESS_PRIO_CFG_SET].rx_size		= MV_DP_MSG_INGRESS_PRIO_CFG_SET_RX_SIZE;
	msgs[MV_DP_MSGID_INGRESS_PRIO_CFG_SET].tx_size		= MV_DP_MSG_INGRESS_PRIO_CFG_SET_TX_SIZE;
	msgs[MV_DP_MSGID_INGRESS_PRIO_CFG_SET].rx_handle	= 0;
	msgs[MV_DP_MSGID_INGRESS_PRIO_CFG_SET].tx_handle	= mv_dp_ingress_prio_cfg_populate_msg;
	msgs[MV_DP_MSGID_INGRESS_PRIO_CFG_SET].rx_cb_size	= 0;
	msgs[MV_DP_MSGID_INGRESS_PRIO_CFG_SET].tx_flags		= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_INGRESS_PRIO_CFG_SET].return_evt	= MV_NSS_DP_EVT_INGRESS_PRIO_CFG_SET;
	msgs[MV_DP_MSGID_INGRESS_PRIO_CFG_SET].name		= "INGRESS PRIO CFG SET";
	msgs[MV_DP_MSGID_INGRESS_PRIO_CFG_SET].trace		= MV_DP_LOG_TRC_NONE;


	msgs[MV_DP_MSGID_INGRESS_PRIO_CFG_GET].type		= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_INGRESS_PRIO_CFG_GET].rx_size		= MV_DP_MSG_INGRESS_PRIO_CFG_GET_RX_SIZE;
	msgs[MV_DP_MSGID_INGRESS_PRIO_CFG_GET].tx_size		= MV_DP_MSG_INGRESS_PRIO_CFG_GET_TX_SIZE;
	msgs[MV_DP_MSGID_INGRESS_PRIO_CFG_GET].rx_handle	= mv_dp_ingress_prio_cfg_parse_struct;
	msgs[MV_DP_MSGID_INGRESS_PRIO_CFG_GET].tx_handle	= 0;
	msgs[MV_DP_MSGID_INGRESS_PRIO_CFG_GET].rx_cb_size	= sizeof(mv_nss_dp_ingress_prio_cfg_t);
	msgs[MV_DP_MSGID_INGRESS_PRIO_CFG_GET].tx_flags		= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_INGRESS_PRIO_CFG_GET].return_evt	= MV_NSS_DP_EVT_INGRESS_PRIO_CFG_GET;
	msgs[MV_DP_MSGID_INGRESS_PRIO_CFG_GET].name		= "INGRESS PRIO CFG GET";
	msgs[MV_DP_MSGID_INGRESS_PRIO_CFG_GET].trace		= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_EGRESS_PRIO_CFG_SET].type		= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_EGRESS_PRIO_CFG_SET].rx_size		= MV_DP_MSG_EGRESS_PRIO_CFG_SET_RX_SIZE;
	msgs[MV_DP_MSGID_EGRESS_PRIO_CFG_SET].tx_size		= MV_DP_MSG_EGRESS_PRIO_CFG_SET_TX_SIZE;
	msgs[MV_DP_MSGID_EGRESS_PRIO_CFG_SET].rx_handle		= 0;
	msgs[MV_DP_MSGID_EGRESS_PRIO_CFG_SET].tx_handle		= mv_dp_egress_prio_cfg_populate_msg;
	msgs[MV_DP_MSGID_EGRESS_PRIO_CFG_SET].rx_cb_size	= 0;
	msgs[MV_DP_MSGID_EGRESS_PRIO_CFG_SET].tx_flags		= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_EGRESS_PRIO_CFG_SET].return_evt	= MV_NSS_DP_EVT_EGRESS_PRIO_CFG_SET;
	msgs[MV_DP_MSGID_EGRESS_PRIO_CFG_SET].name		= "EGRESS PRIO CFG SET";
	msgs[MV_DP_MSGID_EGRESS_PRIO_CFG_SET].trace		= MV_DP_LOG_TRC_NONE;


	msgs[MV_DP_MSGID_EGRESS_PRIO_CFG_GET].type		= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_EGRESS_PRIO_CFG_GET].rx_size		= MV_DP_MSG_EGRESS_PRIO_CFG_GET_RX_SIZE;
	msgs[MV_DP_MSGID_EGRESS_PRIO_CFG_GET].tx_size		= MV_DP_MSG_EGRESS_PRIO_CFG_GET_TX_SIZE;
	msgs[MV_DP_MSGID_EGRESS_PRIO_CFG_GET].rx_handle		= mv_dp_egress_prio_cfg_parse_struct;
	msgs[MV_DP_MSGID_EGRESS_PRIO_CFG_GET].tx_handle		= 0;
	msgs[MV_DP_MSGID_EGRESS_PRIO_CFG_GET].rx_cb_size	= sizeof(mv_nss_dp_egress_prio_cfg_t);
	msgs[MV_DP_MSGID_EGRESS_PRIO_CFG_GET].tx_flags		= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_EGRESS_PRIO_CFG_GET].return_evt	= MV_NSS_DP_EVT_EGRESS_PRIO_CFG_GET;
	msgs[MV_DP_MSGID_EGRESS_PRIO_CFG_GET].name		= "EGRESS PRIO CFG GET";
	msgs[MV_DP_MSGID_EGRESS_PRIO_CFG_GET].trace		= MV_DP_LOG_TRC_NONE;


	msgs[MV_DP_MSGID_INGRESS_QOS_POLICY_SET].type		= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_INGRESS_QOS_POLICY_SET].rx_size	= MV_DP_MSG_INGRESS_QOS_POLICY_SET_RX_SIZE;
	msgs[MV_DP_MSGID_INGRESS_QOS_POLICY_SET].tx_size	= sizeof(mv_nss_dp_ingress_qos_policy_t);
	msgs[MV_DP_MSGID_INGRESS_QOS_POLICY_SET].rx_handle	= 0;
	msgs[MV_DP_MSGID_INGRESS_QOS_POLICY_SET].tx_handle	= 0;
	msgs[MV_DP_MSGID_INGRESS_QOS_POLICY_SET].rx_cb_size	= 0;
	msgs[MV_DP_MSGID_INGRESS_QOS_POLICY_SET].tx_flags	= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_INGRESS_QOS_POLICY_SET].return_evt	= MV_NSS_DP_EVT_INGRESS_QOS_POLICY_SET;
	msgs[MV_DP_MSGID_INGRESS_QOS_POLICY_SET].name		= "INGRESS QOS POLICY SET";
	msgs[MV_DP_MSGID_INGRESS_QOS_POLICY_SET].trace		= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_INGRESS_QOS_POLICY_GET].type		= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_INGRESS_QOS_POLICY_GET].rx_size	= sizeof(mv_nss_dp_ingress_qos_policy_t);
	msgs[MV_DP_MSGID_INGRESS_QOS_POLICY_GET].tx_size	= sizeof(uint8_t);
	msgs[MV_DP_MSGID_INGRESS_QOS_POLICY_GET].rx_handle	= 0;
	msgs[MV_DP_MSGID_INGRESS_QOS_POLICY_GET].tx_handle	= 0;
	msgs[MV_DP_MSGID_INGRESS_QOS_POLICY_GET].rx_cb_size	= sizeof(mv_nss_dp_ingress_qos_policy_t);
	msgs[MV_DP_MSGID_INGRESS_QOS_POLICY_GET].tx_flags	= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_INGRESS_QOS_POLICY_GET].return_evt	= MV_NSS_DP_EVT_INGRESS_QOS_POLICY_GET;
	msgs[MV_DP_MSGID_INGRESS_QOS_POLICY_GET].name		= "INGRESS QOS POLICY GET";
	msgs[MV_DP_MSGID_INGRESS_QOS_POLICY_GET].trace		= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_EGRESS_QOS_POLICY_SET].type		= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_EGRESS_QOS_POLICY_SET].rx_size		= MV_DP_MSG_EGRESS_QOS_POLICY_SET_RX_SIZE;
	msgs[MV_DP_MSGID_EGRESS_QOS_POLICY_SET].tx_size		= sizeof(mv_nss_dp_egress_qos_policy_t);
	msgs[MV_DP_MSGID_EGRESS_QOS_POLICY_SET].rx_handle	= 0;
	msgs[MV_DP_MSGID_EGRESS_QOS_POLICY_SET].tx_handle	= 0;
	msgs[MV_DP_MSGID_EGRESS_QOS_POLICY_SET].rx_cb_size	= 0;
	msgs[MV_DP_MSGID_EGRESS_QOS_POLICY_SET].tx_flags	= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_EGRESS_QOS_POLICY_SET].return_evt	= MV_NSS_DP_EVT_EGRESS_QOS_POLICY_SET;
	msgs[MV_DP_MSGID_EGRESS_QOS_POLICY_SET].name		= "EGRESS QOS POLICY SET";
	msgs[MV_DP_MSGID_EGRESS_QOS_POLICY_SET].trace		= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_EGRESS_QOS_POLICY_GET].type		= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_EGRESS_QOS_POLICY_GET].rx_size		= sizeof(mv_nss_dp_egress_qos_policy_t);
	msgs[MV_DP_MSGID_EGRESS_QOS_POLICY_GET].tx_size		= sizeof(uint8_t);
	msgs[MV_DP_MSGID_EGRESS_QOS_POLICY_GET].rx_handle	= 0;
	msgs[MV_DP_MSGID_EGRESS_QOS_POLICY_GET].tx_handle	= 0;
	msgs[MV_DP_MSGID_EGRESS_QOS_POLICY_GET].rx_cb_size	= sizeof(mv_nss_dp_egress_qos_policy_t);
	msgs[MV_DP_MSGID_EGRESS_QOS_POLICY_GET].tx_flags	= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_EGRESS_QOS_POLICY_GET].return_evt	= MV_NSS_DP_EVT_EGRESS_QOS_POLICY_GET;
	msgs[MV_DP_MSGID_EGRESS_QOS_POLICY_GET].name		= "EGRESS QOS POLICY GET";
	msgs[MV_DP_MSGID_EGRESS_QOS_POLICY_GET].trace		= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_EGRESS_QUEUE_CFG_SET].type		= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_EGRESS_QUEUE_CFG_SET].rx_size		= MV_DP_MSG_QUEUE_CFG_SET_RX_SIZE;
	msgs[MV_DP_MSGID_EGRESS_QUEUE_CFG_SET].tx_size		= MV_DP_MSG_QUEUE_CFG_SET_TX_SIZE;
	msgs[MV_DP_MSGID_EGRESS_QUEUE_CFG_SET].rx_handle	= 0;
	msgs[MV_DP_MSGID_EGRESS_QUEUE_CFG_SET].tx_handle	= mv_dp_egress_queue_cfg_populate_msg;
	msgs[MV_DP_MSGID_EGRESS_QUEUE_CFG_SET].rx_cb_size	= 0;
	msgs[MV_DP_MSGID_EGRESS_QUEUE_CFG_SET].tx_flags		= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_EGRESS_QUEUE_CFG_SET].return_evt	= MV_NSS_DP_EVT_EGRESS_Q_CFG_SET;
	msgs[MV_DP_MSGID_EGRESS_QUEUE_CFG_SET].name		= "EGRESS Q CFG SET";
	msgs[MV_DP_MSGID_EGRESS_QUEUE_CFG_SET].trace		= MV_DP_LOG_TRC_NONE;


	msgs[MV_DP_MSGID_EGRESS_QUEUE_CFG_GET].type		= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_EGRESS_QUEUE_CFG_GET].rx_size		= MV_DP_MSG_QUEUE_CFG_GET_RX_SIZE;
	msgs[MV_DP_MSGID_EGRESS_QUEUE_CFG_GET].tx_size		= MV_DP_MSG_QUEUE_CFG_GET_TX_SIZE;
	msgs[MV_DP_MSGID_EGRESS_QUEUE_CFG_GET].rx_handle	= mv_dp_egress_queue_cfg_parse_struct;
	msgs[MV_DP_MSGID_EGRESS_QUEUE_CFG_GET].tx_handle	= mv_dp_queue_id_cfg_populate_msg;
	msgs[MV_DP_MSGID_EGRESS_QUEUE_CFG_GET].rx_cb_size	= sizeof(mv_nss_dp_egress_queue_cfg_t);
	msgs[MV_DP_MSGID_EGRESS_QUEUE_CFG_GET].tx_flags		= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_EGRESS_QUEUE_CFG_GET].return_evt	= MV_NSS_DP_EVT_EGRESS_Q_CFG_GET;
	msgs[MV_DP_MSGID_EGRESS_QUEUE_CFG_GET].name		= "EGRESS Q CFG GET";
	msgs[MV_DP_MSGID_EGRESS_QUEUE_CFG_GET].trace		= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_INGRESS_QUEUE_CFG_SET].type		= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_INGRESS_QUEUE_CFG_SET].rx_size		= MV_DP_MSG_QUEUE_CFG_SET_RX_SIZE;
	msgs[MV_DP_MSGID_INGRESS_QUEUE_CFG_SET].tx_size		= MV_DP_MSG_QUEUE_CFG_SET_TX_SIZE;
	msgs[MV_DP_MSGID_INGRESS_QUEUE_CFG_SET].rx_handle	= 0;
	msgs[MV_DP_MSGID_INGRESS_QUEUE_CFG_SET].tx_handle	= mv_dp_ingress_queue_cfg_populate_msg;
	msgs[MV_DP_MSGID_INGRESS_QUEUE_CFG_SET].rx_cb_size	= 0;
	msgs[MV_DP_MSGID_INGRESS_QUEUE_CFG_SET].tx_flags	= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_INGRESS_QUEUE_CFG_SET].return_evt	= MV_NSS_DP_EVT_INGRESS_Q_CFG_SET;
	msgs[MV_DP_MSGID_INGRESS_QUEUE_CFG_SET].name		= "INGRESS Q CFG SET";
	msgs[MV_DP_MSGID_INGRESS_QUEUE_CFG_SET].trace		= MV_DP_LOG_TRC_NONE;


	msgs[MV_DP_MSGID_INGRESS_QUEUE_CFG_GET].type		= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_INGRESS_QUEUE_CFG_GET].rx_size		= MV_DP_MSG_QUEUE_CFG_GET_RX_SIZE;
	msgs[MV_DP_MSGID_INGRESS_QUEUE_CFG_GET].tx_size		= MV_DP_MSG_QUEUE_CFG_GET_TX_SIZE;
	msgs[MV_DP_MSGID_INGRESS_QUEUE_CFG_GET].rx_handle	= mv_dp_ingress_queue_cfg_parse_struct;
	msgs[MV_DP_MSGID_INGRESS_QUEUE_CFG_GET].tx_handle	= mv_dp_queue_id_cfg_populate_msg;
	msgs[MV_DP_MSGID_INGRESS_QUEUE_CFG_GET].rx_cb_size	= sizeof(mv_nss_dp_ingress_queue_cfg_t);
	msgs[MV_DP_MSGID_INGRESS_QUEUE_CFG_GET].tx_flags	= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_INGRESS_QUEUE_CFG_GET].return_evt	= MV_NSS_DP_EVT_INGRESS_Q_CFG_GET;
	msgs[MV_DP_MSGID_INGRESS_QUEUE_CFG_GET].name		= "INGRESS Q CFG GET";
	msgs[MV_DP_MSGID_INGRESS_QUEUE_CFG_GET].trace		= MV_DP_LOG_TRC_NONE;


	msgs[MV_DP_MSGID_HASH_PROFILE_SET].type			= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_HASH_PROFILE_SET].rx_size		= MV_DP_MSG_HASH_PROF_SET_RX_SIZE;
	msgs[MV_DP_MSGID_HASH_PROFILE_SET].tx_size		= MV_DP_MSG_HASH_PROF_SET_TX_SIZE;
	msgs[MV_DP_MSGID_HASH_PROFILE_SET].rx_handle		= 0;
	msgs[MV_DP_MSGID_HASH_PROFILE_SET].tx_handle		= mv_dp_hash_prof_populate_msg;
	msgs[MV_DP_MSGID_HASH_PROFILE_SET].rx_cb_size		= 0;
	msgs[MV_DP_MSGID_HASH_PROFILE_SET].tx_flags		= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_HASH_PROFILE_SET].return_evt		= MV_NSS_DP_EVT_HASH_PROFILE_SET;
	msgs[MV_DP_MSGID_HASH_PROFILE_SET].name			= "HASH PROF SET";
	msgs[MV_DP_MSGID_HASH_PROFILE_SET].trace		= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_HASH_PROFILE_DELETE].type		= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_HASH_PROFILE_DELETE].rx_size		= MV_DP_MSG_HASH_PROF_DEL_RX_SIZE;
	msgs[MV_DP_MSGID_HASH_PROFILE_DELETE].tx_size		= MV_DP_MSG_HASH_PROF_DEL_TX_SIZE;
	msgs[MV_DP_MSGID_HASH_PROFILE_DELETE].rx_handle		= 0;
	msgs[MV_DP_MSGID_HASH_PROFILE_DELETE].tx_handle		= mv_dp_hash_prof_id_populate_msg;
	msgs[MV_DP_MSGID_HASH_PROFILE_DELETE].rx_cb_size	= 0;
	msgs[MV_DP_MSGID_HASH_PROFILE_DELETE].tx_flags		= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_HASH_PROFILE_DELETE].return_evt	= MV_NSS_DP_EVT_HASH_PROFILE_DELETE;
	msgs[MV_DP_MSGID_HASH_PROFILE_DELETE].name		= "HASH PROF DEL";
	msgs[MV_DP_MSGID_HASH_PROFILE_DELETE].trace		= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_HASH_PROFILE_GET].type			= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_HASH_PROFILE_GET].rx_size		= MV_DP_MSG_HASH_PROF_GET_RX_SIZE;
	msgs[MV_DP_MSGID_HASH_PROFILE_GET].tx_size		= MV_DP_MSG_HASH_PROF_GET_TX_SIZE;
	msgs[MV_DP_MSGID_HASH_PROFILE_GET].rx_handle		= mv_dp_hash_prof_parse_struct;
	msgs[MV_DP_MSGID_HASH_PROFILE_GET].tx_handle		= mv_dp_hash_prof_id_populate_msg;
	msgs[MV_DP_MSGID_HASH_PROFILE_GET].rx_cb_size		= sizeof(mv_nss_dp_hash_profile_t);
	msgs[MV_DP_MSGID_HASH_PROFILE_GET].tx_flags		= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_HASH_PROFILE_GET].return_evt		= MV_NSS_DP_EVT_HASH_PROFILE_GET;
	msgs[MV_DP_MSGID_HASH_PROFILE_GET].name			= "HASH PROF GET";
	msgs[MV_DP_MSGID_HASH_PROFILE_GET].trace		= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_MC_BRIDGED_CFG_SET].type		= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_MC_BRIDGED_CFG_SET].rx_size		= MV_DP_MSG_MC_BRIDGED_SET_RX_SIZE;
	msgs[MV_DP_MSGID_MC_BRIDGED_CFG_SET].tx_size		= MV_DP_MSG_MC_BRIDGED_SET_TX_SIZE;
	msgs[MV_DP_MSGID_MC_BRIDGED_CFG_SET].rx_handle		= 0;
	msgs[MV_DP_MSGID_MC_BRIDGED_CFG_SET].tx_handle		= mv_dp_mc_bridged_populate_msg;
	msgs[MV_DP_MSGID_MC_BRIDGED_CFG_SET].rx_cb_size		= 0;
	msgs[MV_DP_MSGID_MC_BRIDGED_CFG_SET].tx_flags		= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_MC_BRIDGED_CFG_SET].return_evt		= MV_NSS_DP_EVT_MC_BRIDGED_CFG_SET;
	msgs[MV_DP_MSGID_MC_BRIDGED_CFG_SET].name		= "MC BR SET";
	msgs[MV_DP_MSGID_MC_BRIDGED_CFG_SET].trace		= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_MC_BRIDGED_CFG_DELETE].type		= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_MC_BRIDGED_CFG_DELETE].rx_size		= MV_DP_MSG_MC_BRIDGED_DEL_RX_SIZE;
	msgs[MV_DP_MSGID_MC_BRIDGED_CFG_DELETE].tx_size		= MV_DP_MSG_MC_BRIDGED_DEL_TX_SIZE;
	msgs[MV_DP_MSGID_MC_BRIDGED_CFG_DELETE].rx_handle	= 0;
	msgs[MV_DP_MSGID_MC_BRIDGED_CFG_DELETE].tx_handle	= mv_dp_mc_bridged_populate_msg;
	msgs[MV_DP_MSGID_MC_BRIDGED_CFG_DELETE].rx_cb_size	= 0;
	msgs[MV_DP_MSGID_MC_BRIDGED_CFG_DELETE].tx_flags	= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_MC_BRIDGED_CFG_DELETE].return_evt	= MV_NSS_DP_EVT_MC_BRIDGED_CFG_DELETE;
	msgs[MV_DP_MSGID_MC_BRIDGED_CFG_DELETE].name		= "MC BR DEL";
	msgs[MV_DP_MSGID_MC_BRIDGED_CFG_DELETE].trace		= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_MC_BRIDGED_CFG_GET].type		= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_MC_BRIDGED_CFG_GET].rx_size		= MV_DP_MSG_MC_BRIDGED_GET_RX_SIZE;
	msgs[MV_DP_MSGID_MC_BRIDGED_CFG_GET].tx_size		= MV_DP_MSG_MC_BRIDGED_GET_TX_SIZE;
	msgs[MV_DP_MSGID_MC_BRIDGED_CFG_GET].rx_handle		= mv_dp_mc_bridged_parse_struct;
	msgs[MV_DP_MSGID_MC_BRIDGED_CFG_GET].tx_handle		= mv_dp_mc_ind_populate_msg;
	msgs[MV_DP_MSGID_MC_BRIDGED_CFG_GET].rx_cb_size		= sizeof(mv_nss_dp_mc_bridged_cfg_t);
	msgs[MV_DP_MSGID_MC_BRIDGED_CFG_GET].tx_flags		= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_MC_BRIDGED_CFG_GET].return_evt		= MV_NSS_DP_EVT_MC_BRIDGED_CFG_GET;
	msgs[MV_DP_MSGID_MC_BRIDGED_CFG_GET].name		= "MC BR GET";
	msgs[MV_DP_MSGID_MC_BRIDGED_CFG_GET].trace		= MV_DP_LOG_TRC_NONE;


	msgs[MV_DP_MSGID_MC_TUNNELED_CFG_SET].type		= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_MC_TUNNELED_CFG_SET].rx_size		= MV_DP_MSG_MC_TUNNELED_SET_RX_SIZE;
	msgs[MV_DP_MSGID_MC_TUNNELED_CFG_SET].tx_size		= MV_DP_MSG_MC_TUNNELED_SET_TX_SIZE;
	msgs[MV_DP_MSGID_MC_TUNNELED_CFG_SET].rx_handle		= 0;
	msgs[MV_DP_MSGID_MC_TUNNELED_CFG_SET].tx_handle		= mv_dp_mc_tunneled_populate_msg;
	msgs[MV_DP_MSGID_MC_TUNNELED_CFG_SET].rx_cb_size	= 0;
	msgs[MV_DP_MSGID_MC_TUNNELED_CFG_SET].tx_flags		= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_MC_TUNNELED_CFG_SET].return_evt	= MV_NSS_DP_EVT_MC_TUNNELED_CFG_SET;
	msgs[MV_DP_MSGID_MC_TUNNELED_CFG_SET].name		= "MC TN SET";
	msgs[MV_DP_MSGID_MC_TUNNELED_CFG_SET].trace		= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_MC_TUNNELED_CFG_DELETE].type		= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_MC_TUNNELED_CFG_DELETE].rx_size	= MV_DP_MSG_MC_TUNNELED_DEL_RX_SIZE;
	msgs[MV_DP_MSGID_MC_TUNNELED_CFG_DELETE].tx_size	= MV_DP_MSG_MC_TUNNELED_DEL_TX_SIZE;
	msgs[MV_DP_MSGID_MC_TUNNELED_CFG_DELETE].rx_handle	= 0;
	msgs[MV_DP_MSGID_MC_TUNNELED_CFG_DELETE].tx_handle	= mv_dp_mc_mgid_populate_msg;
	msgs[MV_DP_MSGID_MC_TUNNELED_CFG_DELETE].rx_cb_size	= 0;
	msgs[MV_DP_MSGID_MC_TUNNELED_CFG_DELETE].tx_flags	= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_MC_TUNNELED_CFG_DELETE].return_evt	= MV_NSS_DP_EVT_MC_TUNNELED_CFG_DELETE;
	msgs[MV_DP_MSGID_MC_TUNNELED_CFG_DELETE].name		= "MC TN DEL";
	msgs[MV_DP_MSGID_MC_TUNNELED_CFG_DELETE].trace		= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_MC_TUNNELED_CFG_GET].type		= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_MC_TUNNELED_CFG_GET].rx_size		= MV_DP_MSG_MC_TUNNELED_GET_RX_SIZE;
	msgs[MV_DP_MSGID_MC_TUNNELED_CFG_GET].tx_size		= MV_DP_MSG_MC_TUNNELED_GET_TX_SIZE;
	msgs[MV_DP_MSGID_MC_TUNNELED_CFG_GET].rx_handle		= mv_dp_mc_tunneled_parse_struct;
	msgs[MV_DP_MSGID_MC_TUNNELED_CFG_GET].tx_handle		= mv_dp_mc_ind_populate_msg;
	msgs[MV_DP_MSGID_MC_TUNNELED_CFG_GET].rx_cb_size	= sizeof(mv_nss_dp_mc_tunneled_cfg_t);
	msgs[MV_DP_MSGID_MC_TUNNELED_CFG_GET].tx_flags		= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_MC_TUNNELED_CFG_GET].return_evt	= MV_NSS_DP_EVT_MC_TUNNELED_CFG_GET;
	msgs[MV_DP_MSGID_MC_TUNNELED_CFG_GET].name		= "MC TN GET";
	msgs[MV_DP_MSGID_MC_TUNNELED_CFG_GET].trace		= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_DBG_NSS_MEM_WRITE].type		= MV_DP_MSG_TYPE_INTERNAL;
	msgs[MV_DP_MSGID_DBG_NSS_MEM_WRITE].rx_size		= 0;
	msgs[MV_DP_MSGID_DBG_NSS_MEM_WRITE].tx_size		= MV_DP_MSG_NSS_MEM_WRITE_TX_SIZE;
	msgs[MV_DP_MSGID_DBG_NSS_MEM_WRITE].rx_handle		= 0;
	msgs[MV_DP_MSGID_DBG_NSS_MEM_WRITE].tx_handle		= mv_dp_dbg_nss_mem_write_populate_msg;
	msgs[MV_DP_MSGID_DBG_NSS_MEM_WRITE].rx_cb_size		= 0;
	msgs[MV_DP_MSGID_DBG_NSS_MEM_WRITE].tx_flags		= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_DBG_NSS_MEM_WRITE].return_evt		= MV_NSS_DP_EVT_NOTIFY_MSG;
	msgs[MV_DP_MSGID_DBG_NSS_MEM_WRITE].name		= "NSS MEM WRITE";
	msgs[MV_DP_MSGID_DBG_NSS_MEM_WRITE].trace		= MV_DP_LOG_TRC_NONE;


	msgs[MV_DP_MSGID_DBG_NSS_MEM_READ].type			= MV_DP_MSG_TYPE_INTERNAL;
	msgs[MV_DP_MSGID_DBG_NSS_MEM_READ].rx_size		= MV_DP_MSG_NSS_MEM_READ_RX_SIZE;
	msgs[MV_DP_MSGID_DBG_NSS_MEM_READ].tx_size		= MV_DP_MSG_NSS_MEM_READ_TX_SIZE;
	msgs[MV_DP_MSGID_DBG_NSS_MEM_READ].rx_handle		= mv_dp_dbg_nss_mem_parse_struct;
	msgs[MV_DP_MSGID_DBG_NSS_MEM_READ].tx_handle		= mv_dp_dbg_nss_mem_read_populate_msg;
	msgs[MV_DP_MSGID_DBG_NSS_MEM_READ].rx_cb_size		= sizeof(struct mv_dp_dbg_nss_mem);
	msgs[MV_DP_MSGID_DBG_NSS_MEM_READ].tx_flags		= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_DBG_NSS_MEM_READ].return_evt		= MV_NSS_DP_EVT_NOTIFY_MSG;
	msgs[MV_DP_MSGID_DBG_NSS_MEM_READ].name			= "NSS MEM READ";
	msgs[MV_DP_MSGID_DBG_NSS_MEM_READ].trace		= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_INGRESS_Q_STATS_GET].type		= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_INGRESS_Q_STATS_GET].rx_size	= MV_DP_MSG_Q_STATS_GET_RX_SIZE;
	msgs[MV_DP_MSGID_INGRESS_Q_STATS_GET].tx_size	= MV_DP_MSG_Q_STATS_GET_TX_SIZE;
	msgs[MV_DP_MSGID_INGRESS_Q_STATS_GET].rx_handle	= mv_dp_queue_stats_parse_struct;
	msgs[MV_DP_MSGID_INGRESS_Q_STATS_GET].tx_handle	= mv_dp_queue_id_cfg_populate_msg;
	msgs[MV_DP_MSGID_INGRESS_Q_STATS_GET].rx_cb_size = sizeof(mv_nss_dp_queue_stats_t);
	msgs[MV_DP_MSGID_INGRESS_Q_STATS_GET].tx_flags	= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_INGRESS_Q_STATS_GET].return_evt = MV_NSS_DP_EVT_INGRESS_Q_STATS_GET;
	msgs[MV_DP_MSGID_INGRESS_Q_STATS_GET].name		= "INGRESS Q STATS GET";
	msgs[MV_DP_MSGID_INGRESS_Q_STATS_GET].trace		= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_INGRESS_Q_STATS_RESET].type		= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_INGRESS_Q_STATS_RESET].rx_size	    = MV_DP_MSG_Q_STATS_RESET_RX_SIZE;
	msgs[MV_DP_MSGID_INGRESS_Q_STATS_RESET].tx_size	    = MV_DP_MSG_Q_STATS_RESET_TX_SIZE;
	msgs[MV_DP_MSGID_INGRESS_Q_STATS_RESET].rx_handle	= 0;
	msgs[MV_DP_MSGID_INGRESS_Q_STATS_RESET].tx_handle	= mv_dp_queue_id_cfg_populate_msg;
	msgs[MV_DP_MSGID_INGRESS_Q_STATS_RESET].rx_cb_size  = 0;
	msgs[MV_DP_MSGID_INGRESS_Q_STATS_RESET].tx_flags	= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_INGRESS_Q_STATS_RESET].return_evt  = MV_NSS_DP_EVT_INGRESS_Q_STATS_RESET;
	msgs[MV_DP_MSGID_INGRESS_Q_STATS_RESET].name		= "INGRESS Q STATS RESET";
	msgs[MV_DP_MSGID_INGRESS_Q_STATS_RESET].trace		= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_EGRESS_Q_STATS_GET].type		= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_EGRESS_Q_STATS_GET].rx_size	= MV_DP_MSG_Q_STATS_GET_RX_SIZE;
	msgs[MV_DP_MSGID_EGRESS_Q_STATS_GET].tx_size	= MV_DP_MSG_Q_STATS_GET_TX_SIZE;
	msgs[MV_DP_MSGID_EGRESS_Q_STATS_GET].rx_handle	= mv_dp_queue_stats_parse_struct;
	msgs[MV_DP_MSGID_EGRESS_Q_STATS_GET].tx_handle	= mv_dp_queue_id_cfg_populate_msg;
	msgs[MV_DP_MSGID_EGRESS_Q_STATS_GET].rx_cb_size = sizeof(mv_nss_dp_queue_stats_t);
	msgs[MV_DP_MSGID_EGRESS_Q_STATS_GET].tx_flags	= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_EGRESS_Q_STATS_GET].return_evt = MV_NSS_DP_EVT_EGRESS_Q_STATS_GET;
	msgs[MV_DP_MSGID_EGRESS_Q_STATS_GET].name		= "EGRESS Q STATS GET";
	msgs[MV_DP_MSGID_EGRESS_Q_STATS_GET].trace		= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_EGRESS_Q_STATS_RESET].type			= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_EGRESS_Q_STATS_RESET].rx_size	    = MV_DP_MSG_Q_STATS_RESET_RX_SIZE;
	msgs[MV_DP_MSGID_EGRESS_Q_STATS_RESET].tx_size	    = MV_DP_MSG_Q_STATS_RESET_TX_SIZE;
	msgs[MV_DP_MSGID_EGRESS_Q_STATS_RESET].rx_handle	= 0;
	msgs[MV_DP_MSGID_EGRESS_Q_STATS_RESET].tx_handle	= mv_dp_queue_id_cfg_populate_msg;
	msgs[MV_DP_MSGID_EGRESS_Q_STATS_RESET].rx_cb_size	= 0;
	msgs[MV_DP_MSGID_EGRESS_Q_STATS_RESET].tx_flags		= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_EGRESS_Q_STATS_RESET].return_evt  	= MV_NSS_DP_EVT_EGRESS_Q_STATS_RESET;
	msgs[MV_DP_MSGID_EGRESS_Q_STATS_RESET].name			= "EGRESS Q STATS RESET";
	msgs[MV_DP_MSGID_EGRESS_Q_STATS_RESET].trace		= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_BYPASS_STATE_SET].type		= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_BYPASS_STATE_SET].rx_size		= (0);
	msgs[MV_DP_MSGID_BYPASS_STATE_SET].tx_size		= sizeof(uint8_t);
	msgs[MV_DP_MSGID_BYPASS_STATE_SET].rx_handle		= 0;
	msgs[MV_DP_MSGID_BYPASS_STATE_SET].tx_handle		= 0;
	msgs[MV_DP_MSGID_BYPASS_STATE_SET].rx_cb_size	= 0;
	msgs[MV_DP_MSGID_BYPASS_STATE_SET].tx_flags		= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_BYPASS_STATE_SET].return_evt	= MV_NSS_DP_EVT_BYPASS_STATE_SET;
	msgs[MV_DP_MSGID_BYPASS_STATE_SET].name		= "BYPASS STATE SET";
	msgs[MV_DP_MSGID_BYPASS_STATE_SET].trace		= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_BYPASS_STATE_GET].type		= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_BYPASS_STATE_GET].rx_size		= sizeof(uint8_t);
	msgs[MV_DP_MSGID_BYPASS_STATE_GET].tx_size		= 0;
	msgs[MV_DP_MSGID_BYPASS_STATE_GET].rx_handle		= 0;
	msgs[MV_DP_MSGID_BYPASS_STATE_GET].tx_handle		= 0;
	msgs[MV_DP_MSGID_BYPASS_STATE_GET].rx_cb_size	= sizeof(uint8_t);
	msgs[MV_DP_MSGID_BYPASS_STATE_GET].tx_flags		= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_BYPASS_STATE_GET].return_evt	= MV_NSS_DP_EVT_BYPASS_STATE_GET;
	msgs[MV_DP_MSGID_BYPASS_STATE_GET].name		= "BYPASS STATE GET";
	msgs[MV_DP_MSGID_BYPASS_STATE_GET].trace		= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_VXLAN_VNI_CFG_SET].type		= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_VXLAN_VNI_CFG_SET].rx_size		= (0);
	msgs[MV_DP_MSGID_VXLAN_VNI_CFG_SET].tx_size		= sizeof(mv_nss_dp_vxlan_vni_cfg_t);
	msgs[MV_DP_MSGID_VXLAN_VNI_CFG_SET].rx_handle		= 0;
	msgs[MV_DP_MSGID_VXLAN_VNI_CFG_SET].tx_handle		= 0;
	msgs[MV_DP_MSGID_VXLAN_VNI_CFG_SET].rx_cb_size	= 0;
	msgs[MV_DP_MSGID_VXLAN_VNI_CFG_SET].tx_flags		= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_VXLAN_VNI_CFG_SET].return_evt	= MV_NSS_DP_EVT_VXLAN_VNI_CFG_SET;
	msgs[MV_DP_MSGID_VXLAN_VNI_CFG_SET].name		= "VXLAN VNI CFG SET";
	msgs[MV_DP_MSGID_VXLAN_VNI_CFG_SET].trace		= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_VXLAN_VNI_CFG_GET].type		= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_VXLAN_VNI_CFG_GET].rx_size		= sizeof(mv_nss_dp_vxlan_vni_cfg_t);
	msgs[MV_DP_MSGID_VXLAN_VNI_CFG_GET].tx_size		= sizeof(mv_nss_dp_vxlan_vni_cfg_t);
	msgs[MV_DP_MSGID_VXLAN_VNI_CFG_GET].rx_handle		= 0;
	msgs[MV_DP_MSGID_VXLAN_VNI_CFG_GET].tx_handle		= 0;
	msgs[MV_DP_MSGID_VXLAN_VNI_CFG_GET].rx_cb_size	= sizeof(mv_nss_dp_vxlan_vni_cfg_t);
	msgs[MV_DP_MSGID_VXLAN_VNI_CFG_GET].tx_flags		= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_VXLAN_VNI_CFG_GET].return_evt	= MV_NSS_DP_EVT_VXLAN_VNI_CFG_GET;
	msgs[MV_DP_MSGID_VXLAN_VNI_CFG_GET].name		= "VXLAN VNI CFG GET";
	msgs[MV_DP_MSGID_VXLAN_VNI_CFG_GET].trace		= MV_DP_LOG_TRC_NONE;

	msgs[MV_DP_MSGID_VXLAN_VNI_CFG_DEL].type		= MV_DP_MSG_TYPE_MSG;
	msgs[MV_DP_MSGID_VXLAN_VNI_CFG_DEL].rx_size		= (0);
	msgs[MV_DP_MSGID_VXLAN_VNI_CFG_DEL].tx_size		= sizeof(mv_nss_dp_vxlan_vni_cfg_t);
	msgs[MV_DP_MSGID_VXLAN_VNI_CFG_DEL].rx_handle		= 0;
	msgs[MV_DP_MSGID_VXLAN_VNI_CFG_DEL].tx_handle		= 0;
	msgs[MV_DP_MSGID_VXLAN_VNI_CFG_DEL].rx_cb_size	= 0;
	msgs[MV_DP_MSGID_VXLAN_VNI_CFG_DEL].tx_flags		= MV_DP_F_CFH_MSG_ACK;
	msgs[MV_DP_MSGID_VXLAN_VNI_CFG_DEL].return_evt	= MV_NSS_DP_EVT_VXLAN_VNI_CFG_DELETE;
	msgs[MV_DP_MSGID_VXLAN_VNI_CFG_DEL].name		= "VXLAN VNI CFG DELETE";
	msgs[MV_DP_MSGID_VXLAN_VNI_CFG_DEL].trace		= MV_DP_LOG_TRC_NONE;
}


/*event ID is 0 based!!!*/
static enum mv_dp_rc mv_dp_event_handler_init(enum mv_dp_msg_id ev_id, mv_nss_dp_evt_handler evt)
{
	if (!evt) {
		MV_DP_LOG_DBG1("Default Event Callback is initialized");
		mv_dp_instance.event_info[ev_id].evt = mv_dp_default_rx_cb;

	} else {
		mv_dp_instance.event_info[ev_id].evt = evt;
	}

	return MV_DP_RC_OK;
}

/*for now -- use default event handler*/
enum mv_dp_rc mv_dp_event_info_init(struct mv_dp_event_info *events, mv_nss_dp_evt_handler evt)
{
	int i;

	if (!evt) {
		MV_DP_LOG_DBG1("Default Event Callback is initialized");
		evt = mv_dp_default_rx_cb;
	}

	for (i = 0; i < MV_DP_EVENTID_LAST; i++)
		mv_dp_event_handler_init(i, evt);

	return MV_DP_RC_OK;
}



void mv_dp_show_event_info(void)
{
	int i;

	MV_DP_CLI_OK("|EVENT INFO %d...%d|<\n", 0, MV_DP_EVENTID_LAST);
	MV_DP_CLI_CONT("| MSG ID |EVENT ID|CALLBACK|<\n");
	for (i = 0; i < MV_DP_EVENTID_LAST; i++)
		MV_DP_CLI_CONT("|%8d|%8d|%8p|<\n", MV_DP_EVENTID_TO_MSGID(i), i, mv_dp_instance.event_info[i].evt);
	MV_DP_CLI_CONT("|EVENT INFO END|<\n");

}



static enum mv_dp_rc mv_dp_ch_data_release(struct mv_dp_ch_data *channel)
{
/*
	unsigned long iflags = 0;
*/
	if (!channel)
		return MV_DP_RC_OK;

	mv_dp_timer_delete(channel);
	mv_dp_chan_delete(channel->ch_id);

	mdelay(MV_DP_MSG_COMPLETE_TIME);
/*in per cpu mode this should run on its own core ?*/
/*	MV_DP_LOCK(&channel->lock, iflags);*/
	mv_dp_rx_buffer_release(channel);
	/*release completion lock if any.*/
/*	MV_DP_UNLOCK(&channel->lock, iflags);*/
	kfree(channel);
	MV_DP_LOG_DBG2("Channel:%d Released\n", channel->ch_id);

	return MV_DP_RC_OK;
}


void mv_dp_exit_main(void)
{
	int core = 0; /* TODO: how to get the correct core? */
	int ch;

	MV_DP_OFFLINE_SET(mv_dp_instance.status);
	MV_DP_MSG_RXTX_DISABLE(mv_dp_instance);
	/*wait until all finished*/


	if (MV_DP_IS_SHARED()) {
		ch = mv_dp_instance.cpu_to_ch_id[smp_processor_id()];
		mv_dp_ch_data_release(mv_dp_instance.ch_data[ch]);

		mv_dp_instance.ch_data[ch] = 0;
		mv_dp_instance.cpu_to_ch_id[core] = 0;
	} else {
		for_each_possible_cpu(core) {
			ch = mv_dp_instance.cpu_to_ch_id[core];
			if (mv_dp_instance.ch_data[ch]) {
				/*consider affinity to run it on the core processor*/
				mv_dp_ch_data_release(mv_dp_instance.ch_data[ch]);
				mv_dp_instance.ch_data[ch] = 0;
			}
			mv_dp_instance.cpu_to_ch_id[core] = 0;
		}

	}
#ifdef MV_DP_QOS_SHADOW
	mv_dp_qos_release();
#endif
	/*release all entities sysFS except for a main*/
	mv_dp_sysfs_exit_entities();

	MV_DP_LOG_INF("DPAPI NOT INITIALIZED\n");
}

/*return null on not found*/
struct mv_dp_msg_buf *mv_dp_rx_buffer_find(int cpu_id, u16 sn)
{
	int chan = MV_DP_CPU_TO_CH(cpu_id);
	struct mv_dp_ch_data *tmp = mv_dp_instance.ch_data[chan];
	struct mv_dp_msg_buf *msg_buf;
	struct list_head *curr, *q;

	list_for_each_safe(curr, q, &(tmp->rx_buf.list_elem)) {
		msg_buf = list_entry(curr, struct mv_dp_msg_buf, list_elem);

		if (msg_buf->sn == sn)
			return msg_buf;
	}
	return NULL;
}

enum mv_dp_rc mv_dp_rx_buffer_remove(int cpu_id, u16 sn)
{
	struct mv_dp_ch_data *chan;
	struct mv_dp_msg_buf *msg_buf;
	struct list_head *curr, *q;
	unsigned long    iflags = 0;

	chan = mv_dp_instance.ch_data[MV_DP_CPU_TO_CH(cpu_id)];

	MV_DP_LOCK(&chan->lock, iflags);

	list_for_each_safe(curr, q, &(chan->rx_buf.list_elem)) {
		msg_buf = list_entry(curr, struct mv_dp_msg_buf, list_elem);
		if (msg_buf->sn == sn) {
			list_del(curr);
			MV_DP_DEC_RX_SIZE(chan);
			MV_DP_UNLOCK(&chan->lock, iflags);
#ifdef MV_DP_DEBUG
			if (MV_DP_CPU_TO_CH(cpu_id) != MV_DP_CPU_TO_CH(msg_buf->cpu_id)) {
				MV_DP_LOG_ERR("REMOVE WRONG CH caller ch:%d  expected(msg) ch:%d\n",
					      MV_DP_RC_ERR_WRONG_CHAN,
					      MV_DP_CPU_TO_CH(cpu_id), MV_DP_CPU_TO_CH(msg_buf->cpu_id));
				chan->api_stats[msg_buf->opcode].err_msg_tx++;
			}
#endif
			if (msg_buf->ext_buf) {
				mv_dp_ext_hdr_be_release(msg_buf->ext_buf);
				kfree(msg_buf->ext_buf);
			}
			kfree(msg_buf);
			return MV_DP_RC_OK;
		}
	}

	MV_DP_UNLOCK(&chan->lock, iflags);
	return MV_DP_RC_ERR_MSG_NOT_FOUND;
}

static enum mv_dp_rc mv_dp_msg_send(struct mv_dp_msg_info_tx *tx_msg, u16 msg_seq_num)
{
	struct agnic_inband_mng_msg_params mng_msg_params;
	int rc;
	u16 cb_size = mv_dp_instance.msg_info[tx_msg->opcode].rx_cb_size + MV_DP_MSG_HDR_SIZE;
	struct mv_dp_msg_header *msg_hdr = tx_msg->msg_data;

	MV_DP_LOG_DBG3("ENTER %s\n", __func__);

	msg_hdr->inst_num = tx_msg->count;
	msg_hdr->rc = 0;
	msg_hdr->flags = tx_msg->flags;

	mng_msg_params.msg = tx_msg->msg_data;
	mng_msg_params.msg_code = tx_msg->opcode;
	mng_msg_params.msg_len = tx_msg->msg_size + MV_DP_MSG_HDR_SIZE;
	/* TODO: add buffer pool */
	mng_msg_params.resp_msg = kmalloc(cb_size, GFP_ATOMIC);
	if (!mng_msg_params.resp_msg) {
		MV_DP_LOG_ERR("PTR Buffer Allocation failed size:%d\n",
			      MV_DP_RC_ERR_ALLOC, cb_size);
		return MV_DP_RC_ERR_OUT_OF_RESOURCES;
	}
	mng_msg_params.cookie = msg_seq_num;
	mng_msg_params.resp_msg_len = cb_size;
	mng_msg_params.timeout = 1000;
	MV_DP_LOG_DBG2("Sending agnic custom msg code:%d msg_len: %d cookie: %lld resp_msg_len: %d timeout: %d\n",
			mng_msg_params.msg_code,
			mng_msg_params.msg_len,
			mng_msg_params.cookie,
			mng_msg_params.resp_msg_len,
			mng_msg_params.timeout);

	rc =  agnic_send_custom_msg(mv_dp_instance.netdev, &mng_msg_params);
	if (rc) {
		MV_DP_LOG_ERR("Fail agnic_send_custom_msg\n", MV_DP_RC_ERR_FAILED);
	}
	MV_DP_LOG_DBG3("EXIT %s\n", __func__);
	return rc;
}

enum mv_dp_rc mv_dp_msg_tx(struct mv_dp_msg_info_tx *tx_msg)
{
	struct mv_dp_ch_data *tmp;
	struct mv_dp_msg_buf *rx_msg_buf = NULL;
	int ch;
	unsigned long iflags = 0;
	int     rc;
	u16     sn;


	MV_DP_LOG_DBG3("ENTER %s\n", __func__);
#ifdef MV_DP_DEBUG
	if (!tx_msg) {
		MV_DP_LOG_ERR("Fail msg TX. NULL tx_msg\n", MV_DP_RC_ERR_NULL_PTR);
		return MV_DP_RC_ERR_INVALID_PARAM;
	}
#endif
	if (!MV_DP_MSGID_IS_OK(tx_msg->opcode)) {
		MV_DP_LOG_ERR("Fail msg TX. ILLEGAL MSGID:%d\n",
			      MV_DP_RC_ERR_MSGID_ILLEGAL, tx_msg->opcode);
		return MV_DP_RC_ERR_INVALID_PARAM;
	}

	ch = mv_dp_instance.cpu_to_ch_id[(tx_msg->cpu_id = smp_processor_id())];
	tmp = mv_dp_instance.ch_data[ch];

	if (!MV_DP_MSG_IS_ALLOWED(mv_dp_instance, tx_msg->opcode)) {
		tmp->api_stats[tx_msg->opcode].err_msg_tx++;
		MV_DP_LOG_ERR("Fail msg TX. DP is OFFLINE opcode:%d\n",
			      MV_DP_RC_SHUTDOWN, tx_msg->opcode);
		return MV_DP_RC_SHUTDOWN;
	}

	MV_DP_LOG_DBG2("Sending msg opcode:%d ch:%d\n", tx_msg->opcode, ch);

	MV_DP_LOCK(&tmp->lock, iflags);
	if (!MV_DP_CHECK_RX_SIZE(tmp)) {
		tmp->api_stats[tx_msg->opcode].err_msg_tx++;
		MV_DP_UNLOCK(&tmp->lock, iflags);
		MV_DP_LOG_ERR("Fail msg TX Too Many requests opcode:%d, size:%d\n",
			      MV_DP_RC_TOO_MANY_REQUESTS, tx_msg->opcode, tmp->rx_size);
		return MV_DP_RC_TOO_MANY_REQUESTS;
	}
	/*TODO: WRAP AROUND 11 bit*/
	sn = MV_DP_MSG_SN_INC(tmp->sn);

	MV_DP_INC_RX_SIZE(tmp);
	MV_DP_UNLOCK(&tmp->lock, iflags);

#ifdef MV_DP_DEBUG
	tx_msg->sn = sn;
	if (MV_DP_LOG_TRC_ON(tx_msg->opcode, MV_DP_LOG_TRC_TX))
		mv_dp_show_msg_tx(tx_msg);
#endif

	if (tx_msg->flags & MV_DP_F_CFH_MSG_ACK) {
#ifdef MV_DP_USE_LIST_BUFFER
		/*allocate rx entry*/
		rx_msg_buf = kzalloc(sizeof(struct mv_dp_msg_buf), GFP_KERNEL);
		if (!rx_msg_buf) {
			MV_DP_LOG_ERR("Fail Allocating buff msg for opcode: %d\n",
				      MV_DP_RC_ERR_ALLOC, tx_msg->opcode);
			goto err_alloc;
		}
		rx_msg_buf->ext_buf = 0;
		if (tx_msg->flags & MV_DP_F_CFH_EXT_HDR) {
			MV_DP_LOG_DBG3("MV_DP_F_CFH_EXT_HDR: flags=%d", tx_msg->flags);
			rx_msg_buf->ext_buf = kzalloc(tx_msg->msg_size, GFP_KERNEL);
			if (!rx_msg_buf->ext_buf) {
				kfree(rx_msg_buf);
				MV_DP_LOG_ERR("Alloc buff msg exthdr for opcode: %d hdr_size:%d\n",
					      MV_DP_RC_ERR_ALLOC, tx_msg->opcode, tx_msg->msg_size);
				goto err_alloc;
			}
			memcpy(rx_msg_buf->ext_buf, tx_msg->msg_data, tx_msg->msg_size);
		}

		rx_msg_buf->age = MV_DP_RX_AGE_COUNT;
		rx_msg_buf->opcode = tx_msg->opcode;
		rx_msg_buf->out_buf = tx_msg->out_buf;
		memcpy(&rx_msg_buf->res, tx_msg->res, sizeof(mv_nss_dp_result_spec_t));
		rx_msg_buf->sn = sn;

		MV_DP_LOCK(&tmp->lock, iflags);
		list_add_tail(&rx_msg_buf->list_elem, &(tmp->rx_buf.list_elem));
		MV_DP_UNLOCK(&tmp->lock, iflags);
#endif

#ifdef MV_DP_DEBUG
		if (MV_DP_LOG_TRC_ON(rx_msg_buf->opcode, MV_DP_LOG_TRC_BUF) &&
		    MV_DP_LOG_TRC_ON(rx_msg_buf->opcode, MV_DP_LOG_TRC_TX)) {
			MV_DP_LOG_MSG("|ENQUEUE RX BUFF|<\n");
			mv_dp_show_msg_buf(rx_msg_buf, MV_DP_CLI_HDR | MV_DP_CLI_DATA | MV_DP_CLI_TRAILER);
		}
#endif
	}

	/*send*/
	rc = mv_dp_msg_send(tx_msg, sn);
	if (rc) {
		MV_DP_LOG_ERR("%03d:%04d>Error Sending msg opcode: %d count:%d send_rc:%d\n",
			      MV_DP_RC_ERR_MSG_TX, ch, sn,
			      tx_msg->opcode, tx_msg->count, rc);
		/*remove from rx_buff*/
		tmp->api_stats[tx_msg->opcode].err_msg_tx++;
		MV_DP_LOCK(&tmp->lock, iflags);
		mv_dp_rx_buf_delete(tmp, rx_msg_buf, false);
		MV_DP_UNLOCK(&tmp->lock, iflags);
		/*if extension header -- have to be released by caller*/
		return MV_DP_RC_ERR_FAILED;
	}

	tmp->api_stats[tx_msg->opcode].msg_tx++;

	MV_DP_LOG_DBG2("%03d:%04d>Sent OK msg opcode:%d count:%d flags:%04X\n", ch, sn,
		       tx_msg->opcode, tx_msg->count, tx_msg->flags);
	MV_DP_LOG_DBG3("EXIT %s\n", __func__);
	return MV_DP_RC_OK;

err_alloc:
	MV_DP_DEC_RX_SIZE(tmp);
	tmp->api_stats[tx_msg->opcode].err_msg_tx++;
	MV_DP_LOG_DBG3("EXIT %s\n", __func__);

	return MV_DP_RC_ERR_OUT_OF_RESOURCES;
}


/*ch to delete from; entry to delete, invoke -- invoke the callback and free ext_buf  or not*/
static enum mv_dp_rc mv_dp_rx_buf_delete(struct mv_dp_ch_data *ch, struct mv_dp_msg_buf *rx_msg_buf, bool invoke)
{

	mv_nss_dp_event_t param;

	MV_DP_LOG_DBG3("ENTER:%s\n", __func__);

#ifdef MV_DP_DEBUG
	if (!ch) {
		MV_DP_LOG_ERR("NULL channel ptr\n", MV_DP_RC_ERR_NULL_PTR);
		return MV_DP_RC_ERR_FAILED;
	}
#endif
	if (!rx_msg_buf) {
		MV_DP_LOG_ERR("NULL msg_rx_buf\n", MV_DP_RC_ERR_NULL_PTR);
		return MV_DP_RC_ERR_NULL_PTR;
	}

	MV_DP_LOG_DBG2("RX Buf Releasing SN:%d, OPCODE:%d", rx_msg_buf->sn, rx_msg_buf->opcode);

	MV_DP_DEC_RX_SIZE(ch);
	list_del(&rx_msg_buf->list_elem);
	if (invoke) {
		if (rx_msg_buf->res.cb) {
			param.cookie = rx_msg_buf->res.cookie;
			param.xid = rx_msg_buf->res.xid;
			param.status = MV_NSS_DP_API_EXEC_TIMEOUT;
			param.type = mv_dp_instance.msg_info[rx_msg_buf->opcode].return_evt;
			param.count = 0;
			rx_msg_buf->res.cb(&param);
		}

		if (rx_msg_buf->ext_buf) {
			MV_DP_LOG_DBG2("Releasing EXT buff\n");
			mv_dp_ext_hdr_be_release(rx_msg_buf->ext_buf);
			kfree(rx_msg_buf->ext_buf);
		}

		if (rx_msg_buf->out_buf && MV_DP_MSG_RLS_BUF(mv_dp_instance.msg_info[rx_msg_buf->opcode].type)) {
			MV_DP_LOG_DBG2("Releasing out_buff opcode%d ptr:%p\n", rx_msg_buf->opcode, rx_msg_buf->out_buf);
			kfree(rx_msg_buf->out_buf);
		}
	} else {
		if (rx_msg_buf->ext_buf) {
			MV_DP_LOG_DBG2("Releasing EXT buff\n");
			kfree(rx_msg_buf->ext_buf);
		}
	}

	kfree(rx_msg_buf);


	MV_DP_LOG_DBG3("EXIT %s\n", __func__);

	return MV_DP_RC_OK;
}

/*check the sizes and callback on error*/
static enum mv_dp_rc mv_dp_msg_check_rx(struct mv_dp_msg_info_rx *msg_rx, struct mv_dp_msg_buf *msg)
{
	mv_nss_dp_event_t param;

	MV_DP_LOG_DBG3("ENTER %s\n", __func__);

#ifdef MV_DP_DEBUG
	if (!msg_rx || !msg) {
		MV_DP_LOG_ERR("NULL msg_rx or msg\n", MV_DP_RC_ERR_NULL_PTR);
		return MV_DP_RC_ERR_NULL_PTR;
	}
#endif

	if ((msg_rx->rc == MV_NSS_DP_OK) || (msg_rx->rc == MV_NSS_DP_END_OF_LIST))
		return MV_DP_RC_OK;
#if 0
/*Workaround for Queue CFG message*/
	if (msg_rx->opcode == MV_DP_MSGID_INGRESS_QUEUE_CFG_SET || msg_rx->opcode == MV_DP_MSGID_EGRESS_QUEUE_CFG_SET) {
		msg_rx->rc = MV_NSS_DP_OK;
		msg_rx->msg_size = MV_DP_MSG_QUEUE_CFG_SET_RX_SIZE;
		return MV_DP_RC_OK;
	}
	if (msg_rx->opcode == MV_DP_MSGID_INGRESS_QUEUE_CFG_GET || msg_rx->opcode == MV_DP_MSGID_EGRESS_QUEUE_CFG_GET) {
		msg_rx->rc = MV_NSS_DP_OK;
		msg_rx->msg_size = MV_DP_MSG_QUEUE_CFG_GET_RX_SIZE;
		return MV_DP_RC_OK;
	}
#endif

	if (msg_rx->opcode == MV_DP_MSGID_CTX) {
		msg_rx->rc = MV_NSS_DP_OK;
		msg_rx->msg_size = MV_DP_MSG_CTX_TX_SIZE;
		return MV_DP_RC_OK;
	}

	if (msg_rx->opcode == MV_DP_MSGID_INIT) {
		msg_rx->rc = MV_NSS_DP_OK;
		msg_rx->msg_size = MV_DP_MSG_INIT_RX_SIZE;
		return MV_DP_RC_OK;
	}

	if (msg_rx->opcode == MV_DP_MSGID_SHUTDOWN) {
		msg_rx->rc = MV_NSS_DP_OK;
		msg_rx->msg_size = MV_DP_MSG_SHUTDOWN_RX_SIZE;
		return MV_DP_RC_OK;
	}

/*workaround end*/

	MV_DP_LOG_ERR("FW Failure Status for opcode:%d, sn:%d status:%d flags: %d\n", MV_DP_RC_ERR_FW_STATUS,
				      msg_rx->opcode, msg_rx->sn, msg_rx->rc, msg_rx->flags);

	if (msg->res.cb) {
		param.cookie = msg->res.cookie;
		param.xid = msg->res.xid;
		param.status = msg_rx->rc; /* changed from  MV_NSS_DP_FAILED;*/
		param.type = mv_dp_instance.msg_info[msg_rx->opcode].return_evt;
		param.count = 0;
		msg->res.cb(&param);
	}

	MV_DP_LOG_DBG3("EXIT (FAIL) %s\n", __func__);

	return MV_DP_RC_ERR_FW_STATUS;
}

void mv_dp_event_send(void *arg)
{
	u8 msg[MV_DP_MSG_HDR_SIZE+MV_DP_MSG_EVT_RX_SIZE];
	struct mv_dp_msg_header *msg_hdr = (struct mv_dp_msg_header *)msg;

	MV_DP_LOG_DBG3("ENTER %s\n", __func__);
	msg_hdr->flags = 0;
	msg_hdr->inst_num = 1;
	msg_hdr->rc = MV_NSS_DP_OK;
	*(uint32_t *)(msg+MV_DP_MSG_HDR_SIZE) = MV_NSS_DP_NOTIFY_DOWN;
	mv_dp_msg_rx(arg, MV_DP_MSGID_EVT_SYS, (-1), msg, MV_DP_MSG_HDR_SIZE+MV_DP_MSG_EVT_RX_SIZE);
	MV_DP_LOG_DBG3("Exit %s\n", __func__);
}

/*msg rx handler
Input: message parameters
Output: void
Desc: message rx handler, registered with and called by messenger

*/
void mv_dp_msg_rx(void *arg, u8 rx_opcode, u64 cookie, void *msg, u16 len)
{

	struct mv_dp_ch_data		*tmp;
	struct mv_dp_msg_info_rx	rx_msg;
	struct mv_dp_msg_buf		*buf_msg;
	void 						*msg_data = (u8 *)msg + MV_DP_MSG_HDR_SIZE;
	unsigned long				iflags = 0;
	int 						size = len - MV_DP_MSG_HDR_SIZE;
	int seq_num = 0;
	int flags;
	int ret_code;
	int num_ok;
	int				rc = MV_DP_RC_OK;
	struct mv_dp_msg_header *msg_hdr = msg;
	MV_DP_LOG_DBG3("ENTER %s\n", __func__);

	MV_DP_LOG_DBG2("mv_dp_msg_rx params: arg:%p, rx_opcode:%d, cookie:%lld msg:%p len:%d\n",
				arg, rx_opcode, cookie, msg, len);

	if ((cookie == -1) && (rx_opcode == AGNIC_CUSTOM_CODE_NOTIFY_DOWN)) {
		mv_dp_event_send(arg);
		MV_DP_LOG_DBG3("Exit %s\n", __func__);
		return;
	}
	seq_num = cookie;
	ret_code = msg_hdr->rc;
	flags = msg_hdr->flags;
	num_ok = msg_hdr->inst_num;

	tmp = (struct mv_dp_ch_data	*)arg;/*mv_dp_instance.ch_data[chan];*/

#ifdef MV_DP_DEBUG
	if (!tmp) {
		MV_DP_LOG_ERR("CH not defined:%d\n", MV_DP_RC_ERR_WRONG_CHAN, tmp->ch_id);
		return;
	}
#endif
	MV_DP_LOG_DBG2("Recieved Message: ch:%d, opcode:%d, sn:%d size:%d flags:0x%04X rc:%d num:%d\n",
			tmp->ch_id, rx_opcode, seq_num, size, flags, ret_code, num_ok);

	if (!MV_DP_MSGID_IS_OK(rx_opcode)) {
		MV_DP_LOG_ERR("Fail msg RX. ILLEGAL OPCODE:%d\n", MV_DP_RC_ERR_MSGID_ILLEGAL, rx_opcode);
		return;
	}

	if (!MV_DP_MSG_IS_ALLOWED(mv_dp_instance, rx_opcode)) {
		tmp->api_stats[rx_opcode].err_msg_rx++;
		return;
	}

	/*ask to change interface to a struct*/
	rx_msg.ch = tmp->ch_id;
	rx_msg.flags = flags;
	rx_msg.msg_data = msg_data;
	rx_msg.msg_size = size;
	rx_msg.num_ok = num_ok;
	rx_msg.opcode = rx_opcode;
	rx_msg.sn = seq_num;
	rx_msg.rc = ret_code;

#ifdef MV_DP_DEBUG
	if (MV_DP_LOG_TRC_ON(rx_opcode, MV_DP_LOG_TRC_RX))
		mv_dp_show_msg_rx(&rx_msg);
#endif

	/*if event -- call handler*/
	if (MV_DP_MSG_IS_EVENT(mv_dp_instance.msg_info[rx_opcode].type)) {
		rc = mv_dp_event_parse(&rx_msg);
		if (MV_DP_RC_OK != rc) {
			tmp->api_stats[rx_opcode].err_msg_rx++;
			MV_DP_LOG_ERR("Event rx Failed opcode:%d,size:%d hdl_rc:%d\n",
				      MV_DP_RC_ERR_EVENT_HNDL_FAILED, rx_opcode, size, rc);
			return;
		}
		tmp->api_stats[rx_opcode].msg_rx++;
		MV_DP_LOG_DBG2("Event Processed ok opcode:%d,size:%d\n", rx_opcode, size);
		MV_DP_LOG_DBG3("Exit %s\n", __func__);
		return;
	} /*EVENT END*/

	/*as soon as msg is found it is removed from the rx buffer*/
	MV_DP_LOCK(&tmp->lock, iflags);
	/*replace with find*/
	buf_msg = mv_dp_rx_buffer_find_sn(&tmp->rx_buf, rx_msg.sn);
	if (NULL == buf_msg) {
		MV_DP_UNLOCK(&tmp->lock, iflags);
		tmp->api_stats[rx_opcode].err_msg_rx++;
		MV_DP_LOG_ERR("RX Message SN not found: opcode:%d, sn:%d",
			      MV_DP_RC_ERR_MSG_NOT_FOUND, rx_msg.opcode, rx_msg.sn);
		return;
	}

	list_del(&buf_msg->list_elem);
	MV_DP_DEC_RX_SIZE(tmp);
	MV_DP_UNLOCK(&tmp->lock, iflags);

#ifdef MV_DP_DEBUG
	if (MV_DP_LOG_TRC_ON(rx_opcode, MV_DP_LOG_TRC_BUF) &&
	    MV_DP_LOG_TRC_ON(rx_opcode, MV_DP_LOG_TRC_RX)) {
		MV_DP_LOG_MSG("|DEQUEUE RX BUFF|<\n");
		mv_dp_show_msg_buf(buf_msg, MV_DP_CLI_HDR | MV_DP_CLI_DATA);
	}

	if (MV_DP_CPU_TO_CH(buf_msg->cpu_id) != tmp->ch_id)
		MV_DP_LOG_ERR("WRONG CH RX:%d expected:%d\n", MV_DP_RC_ERR_WRONG_RX_CHAN,
				tmp->ch_id, MV_DP_CPU_TO_CH(buf_msg->cpu_id));
#endif

	rc = mv_dp_msg_check_rx(&rx_msg, buf_msg);
	/*check message for status and sizes and call callback*/
	if (MV_DP_RC_OK != rc) {
		/*log error*/
		MV_DP_LOG_ERR("MSG RX Check Failed rc:%d opcode:%d,sn:%d,num_ok:%d,size:%d\n",
			      rx_msg.rc, rc, rx_msg.opcode, rx_msg.sn, rx_msg.num_ok, size);
		tmp->api_stats[rx_opcode].err_msg_rx++;
	} else {
		rc = mv_dp_msg_parse(&rx_msg, buf_msg);
		if (MV_DP_RC_OK != rc) {
			MV_DP_LOG_DBG1("MSG RX MSG Processing Failed rc:%d opcode:%d,sn:%d,rc:%d, num_ok:%d,size:%d\n",
				       MV_DP_RC_ERR_MSG_HNDL_FAILED, rx_msg.opcode, rx_msg.sn,
				       rx_msg.rc, rx_msg.num_ok, rx_msg.msg_size);
			tmp->api_stats[rx_opcode].err_msg_rx++;
		}
	}

	MV_DP_LOG_DBG2("Response Processed rc:%d opcode:%d,sn:%d,num_ok:%d,size:%d\n",
		       rc, rx_msg.opcode, rx_msg.sn, rx_msg.num_ok, size);
	/* free ext header here on in msg_handler ? --- in handler as anyway processed*/

	/*check for rx_msg.flags & MV_DP_F_CFH_EXT_HDR and !buf_msg->ext_buf)*/
#ifdef MV_SP_DEBUG
	if (!(buf_msg->ext_buf) && (rx_msg.flags & MV_DP_F_CFH_EXT_HDR))
		MV_DP_LOG_DBG1("EXTHDR flag set but no rx_msg_buf->ext_buf opcode:%d, sn:%d,"\
			       " num_ok:%d, size:%d flags:0x%04X\n",
		       rx_msg.opcode, rx_msg.sn, rx_msg.num_ok, rx_msg.msg_size, rx_msg.flags);

#endif

	/*TBD: if */
	if (MV_DP_MSG_RLS_BUF(mv_dp_instance.msg_info[rx_msg.opcode].type) && buf_msg->out_buf)
		kfree(buf_msg->out_buf);
	if (buf_msg->ext_buf) {
		mv_dp_ext_hdr_be_release(buf_msg->ext_buf);
		kfree(buf_msg->ext_buf);
	}
	kfree(buf_msg);
	kfree(msg);
	MV_DP_LOG_DBG3("EXIT %s\n", __func__);
}

static void mv_dp_timer_cb(unsigned long ptr)
{
	struct mv_dp_ch_data *ch_ptr = (struct mv_dp_ch_data *) ptr;
/*
	consider speciale LOG level for timer
	MV_DP_LOG_DBG3("DP timer event %ld cpu:%d ch_id:%d\n", jiffies, smp_processor_id(), ch_ptr->ch_id);
*/
	if ((ch_ptr->ch_timer.interval) && MV_DP_IS_INITIALISED(mv_dp_instance.status)) {
		/* go thru the ch number rx buffer and age it*/
		ch_ptr->ch_timer.counter++;
		mv_dp_rx_buffer_age(ch_ptr);
		/*reset the timer cb*/
		mv_dp_timer_set(ch_ptr);
	}
}

void mv_dp_timer_setup(struct mv_dp_ch_data *ch)
{
	ch->ch_timer.interval = MV_DP_RX_AGE_TIMEOUT;
	ch->ch_timer.timer.function = mv_dp_timer_cb;
	ch->ch_timer.counter = 0;
	ch->ch_timer.timer.data = (unsigned long)ch;
	init_timer(&(ch->ch_timer.timer));
	MV_DP_LOG_DBG2("Timer Initialized ch:%d\n", ch->ch_id);
}

/*timer is primed if interval is positive*/
void mv_dp_timer_set_interval(struct mv_dp_ch_data *ch, int interval)
{
	ch->ch_timer.interval = interval;
}

void mv_dp_timer_set(struct mv_dp_ch_data *ch)
{
	if (ch->ch_timer.interval > 0) {
		ch->ch_timer.timer.expires = jiffies + msecs_to_jiffies(ch->ch_timer.interval);
		add_timer_on(&(ch->ch_timer.timer), ch->cpu_id);
	}
}

void mv_dp_timer_delete(struct mv_dp_ch_data *ch)
{
	int rc = 0;
	int tmp;
	struct mv_dp_ch_data *called_ch;

	tmp = ch->ch_timer.interval;
	ch->ch_timer.interval = 0;


	called_ch = (struct mv_dp_ch_data *)ch->ch_timer.timer.data;
	if (tmp) {
		mdelay(tmp * 2);
		if (timer_pending(&(ch->ch_timer.timer))) {
			MV_DP_LOG_DBG1("Timer IS pending this_cpu:%d tmr_init_cpu:%d\n",
				      smp_processor_id(), called_ch->cpu_id);
		}
		rc = del_timer_sync(&(ch->ch_timer.timer));

		MV_DP_LOG_DBG1("Timer Delete RC:%d this_cpu:%d tmr_init_cpu:%d\n",
			      rc, smp_processor_id(), called_ch->cpu_id);
	}


}

void mv_dp_dbg_set(u8 lvl)
{
	if (!MV_DP_DBG_LVL_IS_OK(lvl))
		MV_DP_LOG_ERR("DBG level is out of Range:%d\n", MV_DP_RC_ERR_INVALID_PARAM, lvl);

	mv_dp_instance.dbg_lvl = lvl;
}


enum mv_dp_rc mv_dp_trace_set(int msg_id, u8 flag)
{
	if (!MV_DP_MSGID_IS_OK(msg_id)) {
		MV_DP_LOG_ERR("MSG ID out of range:%d\n", MV_DP_RC_ERR_INVALID_PARAM, msg_id);
		return MV_DP_RC_ERR_INVALID_PARAM;
	}

	mv_dp_instance.msg_info[msg_id].trace = flag;
	return MV_DP_RC_OK;
}

/*set trace for all msg of the given type*/
void mv_dp_trace_set_all(u8 type, u8 mask)
{
	int i;
	for (i = 0; i < MV_DP_MSGID_LAST; i++)
		if (mv_dp_instance.msg_info[i].type & type)
			mv_dp_instance.msg_info[i].trace = mask;
}

void mv_dp_show_msg_rx(struct mv_dp_msg_info_rx const *msg)
{
	int i;
	if (!msg) {
		MV_DP_LOG_ERR("EMPTY RX BUFF MSG (NULL)\n", MV_DP_RC_ERR_NULL_PTR);
		return;
	}

	MV_DP_LOG_MSG("|MSG RX|<\n");
	MV_DP_LOG_CONT("|OPCODE| CH |  SN  | FL | NUM |STATUS| SIZE |<\n");
	MV_DP_LOG_CONT("|%6d|%4d|%6d|%04X|%5d|%6d|%6d|<\n", msg->opcode,
		       msg->ch, msg->sn, msg->flags, msg->num_ok, msg->rc, msg->msg_size);

	MV_DP_LOG_CONT("|MSG DATA|<");
	for (i = 0; i < msg->msg_size; i++) {
		if (!(i % MV_DP_LOG_BYTE_DUMP_LINE))
			MV_DP_LOG_CONT("<\n");
		MV_DP_LOG_CONT("%02X ", ((u8 *)(msg->msg_data))[i]);
	}

	if (MV_DP_LOG_TRC_ON(msg->opcode, MV_DP_LOG_TRC_EXT) && (msg->flags & MV_DP_F_CFH_EXT_HDR))
		mv_dp_ext_hdr_be_show(msg->msg_data, MV_DP_LOG_TRC_EXT_BYTES, false);

	MV_DP_LOG_CONT("\n|MSG RX END|<\n");


}


void mv_dp_show_msg_cb(mv_nss_dp_event_t const *evt, const unsigned p_size)
{

	int i;
	u8 *ptr;

	if (!evt) {
		MV_DP_LOG_ERR("NULL *EVT\n", MV_DP_RC_ERR_NULL_PTR);
		return;
	}

	MV_DP_LOG_MSG("|EVENT CB|<\n");
	MV_DP_LOG_CONT("|TYPE|STATUS|   XID  | COOKIE |COUNT|DATA SIZE|<\n");
	MV_DP_LOG_CONT("|%4d|%6d|%08X|%8p|%5d|%9d|<\n", evt->type,
		       evt->status, evt->xid, evt->cookie, evt->count, p_size);
	if (p_size) {
		/*callback st size * count_ok*/
		MV_DP_LOG_CONT("|PARAM DATA|<\n");
		ptr = evt->params.notify_msg;
		for (i = 0; i < p_size; i++) {
			if (!(i % MV_DP_LOG_BYTE_DUMP_LINE))
				MV_DP_LOG_CONT("<\n");
			MV_DP_LOG_CONT("%02X ", ptr[i]);
		}
	}
	MV_DP_LOG_CONT("\n|EVENT CB END|<\n");
}


/*input channel index*/
const struct mv_dp_ch_data *const mv_dp_get_ch_data(int i)
{
#ifdef MV_DP_DEBUG
	if (!MV_DP_CH_ID_IS_OK(i)) {
		MV_DP_LOG_ERR("CH number out of range:%d\n", MV_DP_RC_ERR_INVALID_PARAM, i);
		return NULL;
	}
	if (!mv_dp_instance.ch_data[i])
		MV_DP_LOG_ERR("CH number not defined:%d\n", MV_DP_RC_ERR_INVALID_PARAM, i);
#endif
	return mv_dp_instance.ch_data[i];
}

/*input channel index*/
const struct mv_dp_msg_buf *const mv_dp_get_rx_buff(int i)
{
	if (!MV_DP_CH_ID_IS_OK(i)) {
		MV_DP_LOG_ERR("CH number out of range:%d\n", MV_DP_RC_ERR_INVALID_PARAM, i);
		return NULL;
	}

	return &(mv_dp_instance.ch_data[i]->rx_buf);
}


void mv_dp_show_msg_tx(struct mv_dp_msg_info_tx const *msg)
{
	int i;

	if (!msg) {
		MV_DP_LOG_ERR("NULL *MSG\n", MV_DP_RC_ERR_NULL_PTR);
		return;
	}

	MV_DP_LOG_MSG("|MSG TX|<\n");
	MV_DP_LOG_CONT("|OPCODE|CPU| CH |  SN  |  FL  | COUNT|M SIZE|   CB   |   XID  | COOKIE | OUT PTR|<\n");
	if (msg->res)
		MV_DP_LOG_CONT("|%6d|%3d|%4d|%6d|%06X|%6d|%6d|%8p|%08X|%8p|%8p|<\n", msg->opcode,
			       msg->cpu_id, MV_DP_CPU_TO_CH(msg->cpu_id), msg->sn, msg->flags,
			       msg->count, msg->msg_size, msg->res->cb, msg->res->xid, msg->res->cookie,
			       msg->out_buf);
	else
		MV_DP_LOG_CONT("|%6d|%3d|%4d|%6d|%06X|%6d|%6d|  NULL  |  NULL  |  NULL  |%8p|<\n", msg->opcode,
			       msg->cpu_id, MV_DP_CPU_TO_CH(msg->cpu_id), msg->sn, msg->flags,
			       msg->count, msg->msg_size, msg->out_buf);

	MV_DP_LOG_CONT("|MSG DATA|<");
	for (i = 0; i < msg->msg_size; i++) {
		if (!(i % MV_DP_LOG_BYTE_DUMP_LINE))
			MV_DP_LOG_CONT("<\n");
		MV_DP_LOG_CONT("%02X ", ((u8 *)(msg->msg_data))[i]);
	}

	if (MV_DP_LOG_TRC_ON(msg->opcode, MV_DP_LOG_TRC_EXT) && (msg->flags & MV_DP_F_CFH_EXT_HDR))
		mv_dp_ext_hdr_be_show(msg->msg_data, MV_DP_LOG_TRC_EXT_BYTES, true);

	MV_DP_LOG_CONT("\n|MSG TX END|<\n");
}


void mv_dp_show_msg_buf(struct mv_dp_msg_buf const *msg, int header)
{

	if (header & MV_DP_CLI_HDR)
		MV_DP_CLI_CONT("|OPCODE|CPU| CH |  SN  |AGE|   CB   |   XID  | COOKIE | OUT PTR|EXT  BUF|<\n");

	if (!msg && (header & MV_DP_CLI_DATA)) {
		MV_DP_LOG_ERR("RX BUFF NULL PTR\n", MV_DP_RC_ERR_NULL_PTR);
		return;
	}
	if (header & MV_DP_CLI_DATA) {
		MV_DP_CLI_CONT("|%6d|%3d|%4d|%6d|%3d|%8p|%08X|%8p|%8p|%8p|<\n", msg->opcode,
			       msg->cpu_id, MV_DP_CPU_TO_CH(msg->cpu_id), msg->sn,
			       msg->age, msg->res.cb, msg->res.xid, msg->res.cookie, msg->out_buf,
			       msg->ext_buf);

		if (MV_DP_LOG_TRC_ON(msg->opcode, MV_DP_LOG_TRC_EXT) && (msg->ext_buf))
			mv_dp_ext_hdr_be_show(msg->ext_buf, 0, false); /*false as no buffer context is accessed*/
	}

	if (header & MV_DP_CLI_TRAILER)
		MV_DP_CLI_CONT("|RX BUFF END|<n");
}



void mv_dp_show_cfg(void)
{

	MV_DP_CLI_OK("Config Parameters:\n");
	MV_DP_CLI_CONT("Channel Mode:.......%s\n",
		       (MV_DP_IS_SHARED()) ? ("Shared") : ("Per CPU"));
	MV_DP_CLI_CONT("Debug Level:........0x%02X\n", mv_dp_instance.dbg_lvl);
#if 0
	MV_DP_CLI_CONT("sysFS Sync:.........'%c'(%ld)\n",
		       ((mv_dp_instance.sysfs_delay == 0) ? ('N') : ('Y')),
		       mv_dp_instance.sysfs_delay);
#endif

	MV_DP_CLI_CONT("Status mask:........0x%02X [%c%c]\n", mv_dp_instance.status,
		       (MV_DP_IS_INITIALISED(mv_dp_instance.status)) ? ('I') : ('_'),
		       (MV_DP_IS_ONLINE(mv_dp_instance.status)) ? ('O') : ('_'));
	MV_DP_CLI_CONT("ALLOWED TX/RX mask:.0x%02X [%c%c%c]\n", mv_dp_instance.allowed_mask,
		       (MV_DP_MSG_IS_EVENT(mv_dp_instance.allowed_mask)) ? ('E') : ('_'),
		       (MV_DP_MSG_IS_INTERNAL(mv_dp_instance.allowed_mask)) ? ('I') : ('_'),
		       (MV_DP_MSG_IS_MSG(mv_dp_instance.allowed_mask)) ? ('M') : ('_'));
}

void mv_dp_show_ch_rx_buffer(struct mv_dp_ch_data const *ch)
{
	struct mv_dp_msg_buf    *tmp;
	struct list_head        *curr, *p;

	if (!ch) {
		MV_DP_LOG_ERR("NULL *CH\n", MV_DP_RC_ERR_NULL_PTR);
		return;
	}

	MV_DP_CLI_OK("|RX BUFFER|%6d|<\n", ch->rx_size);
	mv_dp_show_msg_buf(0, MV_DP_CLI_HDR);

	list_for_each_safe(curr, p, &(ch->rx_buf).list_elem) {
		tmp = list_entry(curr, struct mv_dp_msg_buf, list_elem);
		if (tmp)
			mv_dp_show_msg_buf(tmp, MV_DP_CLI_DATA);
	}
	MV_DP_LOG_CONT("\n|RX BUFFER END|<\n");

}


void mv_dp_show_ch_data(struct mv_dp_ch_data const *ch)
{

	if (!ch) {
		MV_DP_LOG_ERR("NULL *CH\n", MV_DP_RC_ERR_NULL_PTR);
		return;
	}

	MV_DP_CLI_OK("Channel Parameters CHID:.%d\n", ch->ch_id);
	MV_DP_CLI_CONT("Channel CPUID:...........%d\n", ch->cpu_id);
	MV_DP_CLI_CONT("Channel SN:..............%d\n", ch->sn);
	MV_DP_CLI_CONT("Buffer Size:.............%d out of:%d\n", ch->rx_size, ch->max_rx_size);
	MV_DP_CLI_CONT("Timer Interval...........%d\n", ch->ch_timer.interval);
	MV_DP_CLI_CONT("Timer Counter............%d\n", ch->ch_timer.counter);
	MV_DP_CLI_CONT("Timer CB.................%p\n", ch->ch_timer.timer.function);
}



void mv_dp_show_error_counters(void)
{
	int i, errors = 0;

	MV_DP_CLI_OK("|ERROR COUNTERS|<\n");
	MV_DP_CLI_CONT("|ERROR |    MNEMONIC    | COUNT  |<\n");
	for (i = 0; i < MV_DP_RC_LAST; i++) {
		MV_DP_CLI_CONT("|%6d|%16.16s|%8d|<\n", i,
			       mv_dp_instance.error_info[i].name,
			       mv_dp_instance.errors[i]);
		errors += mv_dp_instance.errors[i];
	}

	MV_DP_CLI_CONT("|TOTAL:-----------------|%8d|<\n", errors);

	MV_DP_CLI_CONT("|ERROR COUNTERS END|<\n");
}

void mv_dp_show_ch_counters(struct mv_dp_ch_data const *ch)
{
	int i;
	struct mv_dp_msg_stat total;
	int errors;


	if (!ch) {
		MV_DP_CLI_FAIL("WRONG Channel\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	errors = 0;
	memset(&total, 0, sizeof(total));

	MV_DP_CLI_OK("Channel Statistics: CH:%d<\n", ch->ch_id);
	MV_DP_CLI_CONT("|OPCODE|  MNEMONIC  |  TX_OK | TX_ERR |  RX_OK | RX_ERR |<\n");
	for (i = 0; i < MV_DP_MSGID_LAST; i++) {
		MV_DP_CLI_CONT("|%6d|%12.12s|%8d|%8d|%8d|%8d|<\n", i,
			       mv_dp_instance.msg_info[i].name,
			       ch->api_stats[i].msg_tx,
			       ch->api_stats[i].err_msg_tx,
			       ch->api_stats[i].msg_rx,
			       ch->api_stats[i].err_msg_rx);
		total.err_msg_rx += ch->api_stats[i].err_msg_rx;
		total.err_msg_tx += ch->api_stats[i].err_msg_tx;
		total.msg_rx += ch->api_stats[i].msg_rx;
		total.msg_tx += ch->api_stats[i].msg_tx;
	}
	MV_DP_CLI_CONT("|-------------------|  TX_OK | TX_ERR |  RX_OK | RX_ERR |<\n");
	MV_DP_CLI_CONT("|TOTAL:-------------|%8d|%8d|%8d|%8d|<\n",
		       total.msg_tx,
		       total.err_msg_tx,
		       total.msg_rx,
		       total.err_msg_rx);
	MV_DP_CLI_CONT("Channel Statistics: CH:%d END\n", ch->ch_id);
}


void mv_dp_show_ch_map(void)
{
	int i;
	MV_DP_CLI_OK("|CPU to CHANNEL ID|%d|<\n", CONFIG_NR_CPUS);

	MV_DP_CLI_CONT("|CPU ->|CHANNEL ID|<\n");
	for (i = 0; i < CONFIG_NR_CPUS; i++)
		MV_DP_CLI_CONT("|%6d|%10d|<\n", i, mv_dp_instance.cpu_to_ch_id[i]);
}

static void mv_dp_clear_ch_counters(struct mv_dp_ch_data *ch)
{

	memset(&ch->api_stats, 0, sizeof(ch->api_stats));
}


void mv_dp_clear_all_counters(void)
{
	int i;
	unsigned long iflags = 0;

	struct mv_dp_ch_data *ch;

	memset(&mv_dp_instance.errors, 0, sizeof(mv_dp_instance.errors));
	for (i = 0; i < MV_DP_MAX_CHAN_NUM; i++) {
		ch = mv_dp_instance.ch_data[i];
		if (ch) {
			MV_DP_LOCK(&ch->lock, iflags);
			mv_dp_clear_ch_counters(ch);
			MV_DP_UNLOCK(&ch->lock, iflags);
		}
	}
}



/*
 * mv_dp_msg_build
 *
 * Description:
 *       Builds a message from input data using parser from msg_info on per opcode type.
 *
 * Parameters:
 *       tx_msg - Client record to populate, including generic opcode, count and msg_bufffer
 *       opcode - the fully specified opcode of the message to be polulated
 *       ptr_in - generic input to be populated into a message.
 *
 *Note:   tx_msg->opcode must not be changed
 * Returns:
 *        MV_DP_RC_OK - On success or an error code, otherwise.
 */
enum mv_dp_rc mv_dp_msg_build(struct mv_dp_msg_info_tx *tx_msg, enum mv_dp_msg_id opcode, const void *ptr_in)
{

	u32			*msg_buf;
	u8			*msg_ext = NULL;
	u8			*tmp = NULL;
	int			w;
	enum mv_dp_rc		rc;

	int			chunk_index_in, chunk_index_out;
	int			ext_ptr;
	int			tx_size, rx_size, total_rx_size, total_tx_size;
	int			tx_chunks, rx_chunks;
	struct mv_dp_msg_info	*msg_info;
#ifdef MV_DP_USE_DRAM_PTR
	u32			*ptr_buf;
#endif

	MV_DP_LOG_DBG3("ENTER: %s tx_msg:%p, opcode:%d ptr_in%p, count:%d\n", __func__,
		       tx_msg, opcode, ptr_in, tx_msg->count);
	rc = MV_NSS_DP_FAILED;

#ifdef MV_DP_DEBUG
	if (!tx_msg) {
		MV_DP_LOG_ERR("NULL tx_msg:%p ptr_in:%p opcode:%d, count:%d\n",
			      MV_DP_RC_ERR_NULL_PTR, tx_msg, ptr_in, opcode, tx_msg->count);
		rc = MV_NSS_DP_INVALID_PARAM;
		goto e_ret;
	}

	if (!MV_DP_MSGID_IS_OK(opcode)) {
		MV_DP_LOG_ERR("Illegal opocode:%d\n", MV_DP_RC_ERR_MSGID_ILLEGAL, opcode);
		rc = MV_NSS_DP_INVALID_PARAM;
		goto e_ret; /*should be MV_DP_WRONG_SIZE */
	}

	if (!MV_DP_MSG_COUNT_VALID(tx_msg->count)) {
		MV_DP_LOG_ERR("Illegal count:%d opcode:%d\n", MV_DP_RC_ERR_COUNT, tx_msg->count, opcode);
		rc = MV_NSS_DP_INVALID_PARAM;
		goto e_ret; /*should be MV_DP_WRONG_SIZE */
	}
#endif

	msg_info = &mv_dp_instance.msg_info[opcode];
	msg_buf = tx_msg->msg_data;
	tx_size  = msg_info->tx_size;
	rx_size  = msg_info->rx_size;
	total_tx_size = tx_size * tx_msg->count;
	total_rx_size = rx_size * tx_msg->count;

	chunk_index_in = chunk_index_out = 0;
	tx_msg->msg_size = 0;
	tx_msg->flags = msg_info->tx_flags;

	if (!tx_size && (total_rx_size <= MV_DP_CFH_MSG_BUF_SIZE_B)) {
		/*nothing to populate: no input and out fits into CFH*/
		MV_DP_LOG_DBG2("Nothing to populate opcode:%d msg_size:%d count:%d rx_size:%d tx_size:%d\n",
			       opcode, tx_msg->msg_size, tx_msg->count, rx_size, tx_size);
		return MV_NSS_DP_OK;
	}
#if 0
#ifdef MV_DP_DEBUG
if (tx_size && tx_msg->count && !msg_info->tx_handle) {
	MV_DP_LOG_ERR("Null tx_handle for opcode:%d count:%d tx_size:%d\n",
		      MV_DP_RC_ERR_NULL_PTR, opcode, tx_msg->count, tx_size);
	rc = MV_DP_RC_ERR_FAILED;
	goto e_ret;
}
#endif
#endif
	/*check if to use local or allocated buffer: if in or out is out of CFH-- put IN in EXT even if fits*/
	if ((total_tx_size <= MV_DP_CFH_MSG_BUF_SIZE_B) &&
					(total_rx_size <= MV_DP_CFH_MSG_BUF_SIZE_B) && ptr_in) {
		for (w = 0; w < tx_msg->count; w++) {
			if (msg_info->tx_handle)
				rc = msg_info->tx_handle(((u8 *)msg_buf + (w * tx_size)), ptr_in, w);
			else {
				MV_DP_LOG_DBG1("copy to %p from %p size: %d \n",
						((u8 *)msg_buf + (w * tx_size)) + MV_DP_MSG_HDR_SIZE,
						ptr_in, tx_size);
				memcpy(((u8 *)msg_buf + (w * tx_size)) + MV_DP_MSG_HDR_SIZE, ptr_in, tx_size);
				rc = MV_DP_RC_OK;
			}
			if (MV_DP_RC_OK != rc) {
				MV_DP_LOG_ERR("Populate CFH opcode:%d, count: %d\n",
					      MV_DP_RC_ERR_EXT_BUF_POPULATE, opcode, tx_msg->count);
				rc = MV_DP_RC_ERR_FAILED;
				goto e_ret;
			}
			tx_msg->msg_size += tx_size;
		}
	} else {
		tx_chunks = (tx_size) ? (MV_DP_EXT_BUF_GET_CHUNK_COUNT(tx_msg->count, tx_size)) : (0);
		rx_chunks = (rx_size) ? (MV_DP_EXT_BUF_GET_CHUNK_COUNT(tx_msg->count, rx_size)) : (0);
		if (total_tx_size <= MV_DP_CFH_MSG_BUF_SIZE_B - MV_DP_MSG_EXT_HDR_SIZE_CALC_B(tx_chunks + rx_chunks))
			tx_chunks = 0;

		if (total_rx_size <= MV_DP_CFH_MSG_BUF_SIZE_B - MV_DP_MSG_EXT_HDR_SIZE_CALC_B(tx_chunks + rx_chunks))
			rx_chunks = 0;

		MV_DP_LOG_DBG2("EXT HDR opcode: %d, tx_size:%d, rx_size:%d, count:%d,"\
								"ptrs_needed:%d(I=%d|O=%d), each:%d(B)\n",
				opcode, tx_size, rx_size, tx_msg->count, tx_chunks + rx_chunks, tx_chunks, rx_chunks,
				MV_DP_EXT_BUF_CHUNK_SIZE);

		rx_chunks = rx_chunks - tx_chunks;
		if (rx_chunks < 0)
			rx_chunks = 0;

	/*EXT: check if fits ext*/
		if (MV_DP_MSG_EXT_HDR_MAX_PTR < (tx_chunks + rx_chunks)) {
			MV_DP_LOG_ERR("Too many Ext Buf ptrs requested opcode:%d,cnt:%d(%d:%d),"\
				      "ptrs_needed:%d,each:%d(B),max:%d\n",
				      MV_DP_RC_ERR_INVALID_PARAM, opcode, tx_msg->count,
				      (tx_chunks + rx_chunks), tx_chunks, rx_chunks,
				      MV_DP_EXT_BUF_CHUNK_SIZE, MV_DP_MSG_EXT_HDR_MAX_PTR);
			rc = MV_NSS_DP_INVALID_PARAM;
			goto e_ret;
		}
		MV_DP_LOG_DBG2("EXT HDR opcode:%d,tx_size:%d,rx_size:%d,cnt:%d,ptrs_needed:%d(I=%d|O=%d),each:%d(B)\n",
				opcode, tx_size, rx_size, tx_msg->count, tx_chunks + rx_chunks, tx_chunks, rx_chunks,
				MV_DP_EXT_BUF_CHUNK_SIZE);
		/*allocated DRAM PTR*/
#ifdef MV_DP_USE_DRAM_PTR
		ptr_buf = kmalloc(MV_DP_EXT_BUF_CHUNK_SIZE, GFP_KERNEL);
		if (!ptr_buf) {
			MV_DP_LOG_ERR("PTR Buffer Allocation failed size:%d\n",
				      MV_DP_RC_ERR_ALLOC, MV_DP_EXT_BUF_CHUNK_SIZE);
			rc = MV_DP_RC_ERR_OUT_OF_RESOURCES;
			goto e_ret;
		}
		/* store buffer ptr*/
		msg_buf[MV_DP_MSG_EXT_HDR_PTR_WORD] =
				cpu_to_be32(MV_DP_MSG_EXT_HDR_PTR_SET(MV_DP_VIRT_TO_PHYS(ptr_buf)));
		MV_DP_LOG_DBG2("Allocated DRAM PTR_BUF: %p\n", ptr_buf);
#endif
		/*mark EXT is present*/
		tx_msg->flags |= MV_DP_F_CFH_EXT_HDR;
		/*fill IN chunk size*/
		msg_buf[MV_DP_MSG_EXT_HDR_IN_TOTAL_SIZE_WORD0] = (MV_DP_MSG_EXT_HDR_IN_TOTAL_SIZE_SET(0) |
						MV_DP_MSG_EXT_HDR_IN_FLAGS_SET(MV_DP_MSG_EXT_HDR_IN_FLAG_REUSE));

		msg_buf[MV_DP_MSG_EXT_HDR_IN_CHUNK_SIZE_WORD1] =
						MV_DP_MSG_EXT_HDR_IN_CHUNK_SIZE_SET(MV_DP_EXT_BUF_CHUNK_SIZE);
		msg_buf[MV_DP_MSG_EXT_HDR_OUT_CHUNK_SIZE_WORD3] =
						MV_DP_MSG_EXT_HDR_OUT_CHUNK_SIZE_SET(MV_DP_EXT_BUF_CHUNK_SIZE);

		msg_buf[MV_DP_MSG_EXT_HDR_IN_NUM_WORD1] |= MV_DP_MSG_EXT_HDR_IN_NUM_SET(0);
		msg_buf[MV_DP_MSG_EXT_HDR_OUT_NUM_WORD3] |= MV_DP_MSG_EXT_HDR_OUT_NUM_SET(0);

		/*check if IN buffers are needed or it could be put after out ext buffers*/

		for (w = 0; w < tx_msg->count && tx_chunks; w++) {
			/*allocate new chunk and put it into buffer*/
			ext_ptr = w % MV_DP_EXT_BUF_GET_ENTRY_COUNT(tx_size);
			if (!ext_ptr) {
				if (msg_ext) /*need to flush the previous filled chunk from cache except for the first*/
					mv_dma_single_cpu_to_dev(msg_ext, MV_DP_EXT_BUF_CHUNK_SIZE, DMA_TO_DEVICE);
				msg_ext = kmalloc(MV_DP_EXT_BUF_CHUNK_SIZE, GFP_KERNEL);
				if (!msg_ext) {
					MV_DP_LOG_ERR("Ext IN Buffer Allocation failed count:%d size:%d\n",
						      MV_DP_RC_ERR_ALLOC, w, MV_DP_EXT_BUF_CHUNK_SIZE);
					rc = MV_DP_RC_ERR_OUT_OF_RESOURCES;
					goto err_be;
				}
				MV_DP_LOG_DBG2("Allocated IN chunk, chunk_ind:%d, entry_ind:%d ptr: 0x%08X\n",
					       chunk_index_in, w, ext_ptr);
#ifdef MV_DP_USE_DRAM_PTR
				ptr_buf[chunk_index_in] =
				cpu_to_be32(MV_DP_MSG_EXT_HDR_PTR_SET(MV_DP_VIRT_TO_PHYS(msg_ext)));
#else
				msg_buf[MV_DP_MSG_EXT_HDR_PTR_WORD + chunk_index_in] =
				cpu_to_be32(MV_DP_MSG_EXT_HDR_PTR_SET(MV_DP_VIRT_TO_PHYS(msg_ext)));
#endif
				chunk_index_in++;
			}

			rc = msg_info->tx_handle(msg_ext + (ext_ptr * tx_size), ptr_in, w);
			if (MV_DP_RC_OK != rc) {
				MV_DP_LOG_ERR("Populate IN EXT entry:%d ext_ptr:%d\n", MV_DP_RC_ERR_EXT_BUF_POPULATE,
					      tx_msg->count, ext_ptr);
				rc = MV_DP_RC_ERR_FAILED;
				goto err_be;
			}
			MV_DP_LOG_DBG2("Entry %d Populated next chunk_ind_in:%d, last ext_ptr:%d\n",
				       w, chunk_index_in, ext_ptr);
		} /*count loop*/
		/*if IN ext buffer used -> flush the last chunk and update the populated size*/

		if (msg_ext) {
			mv_dma_single_cpu_to_dev(msg_ext, MV_DP_EXT_BUF_CHUNK_SIZE, DMA_TO_DEVICE);
		/*put total in bytes w*sizeof single*/
			msg_buf[MV_DP_MSG_EXT_HDR_IN_TOTAL_SIZE_WORD0] |=
				MV_DP_MSG_EXT_HDR_IN_TOTAL_SIZE_SET((tx_msg->count * tx_size));
		/*BE conversion of ptr -- already done*/
		}

		cpu_to_be32s(&msg_buf[MV_DP_MSG_EXT_HDR_IN_TOTAL_SIZE_WORD0]);

		/*allocate the required number of OUT BUFFERs count*size_rx */
		chunk_index_out = chunk_index_in;

		while (chunk_index_out < (rx_chunks + chunk_index_in)) {
			/*allocate new chunk and put it into buffer*/
			msg_ext = kmalloc(MV_DP_EXT_BUF_CHUNK_SIZE, GFP_KERNEL);
			if (!msg_ext) {
				MV_DP_LOG_ERR("Ext OUT Buffer Allocation failed chunk:%d size:%d\n",
					      MV_DP_RC_ERR_ALLOC, chunk_index_out, MV_DP_EXT_BUF_CHUNK_SIZE);
				rc = MV_DP_RC_ERR_OUT_OF_RESOURCES;
				goto err_be;
			}
			MV_DP_LOG_DBG2("Allocated OUT chunk, chunk_ind:%d\n", chunk_index_out);
#ifdef MV_DP_USE_DRAM_PTR
			ptr_buf[chunk_index_out] =
				cpu_to_be32(MV_DP_MSG_EXT_HDR_PTR_SET(MV_DP_VIRT_TO_PHYS(msg_ext)));
#else
			msg_buf[MV_DP_MSG_EXT_HDR_PTR_WORD + chunk_index_out] =
				cpu_to_be32(MV_DP_MSG_EXT_HDR_PTR_SET(MV_DP_VIRT_TO_PHYS(msg_ext)));
#endif
			mv_dma_single_cpu_to_dev(msg_ext, MV_DP_EXT_BUF_CHUNK_SIZE, DMA_FROM_DEVICE);
			chunk_index_out++;
		}

		msg_buf[MV_DP_MSG_EXT_HDR_IN_NUM_WORD1] |= MV_DP_MSG_EXT_HDR_IN_NUM_SET(chunk_index_in);
		msg_buf[MV_DP_MSG_EXT_HDR_OUT_NUM_WORD3] |=
			MV_DP_MSG_EXT_HDR_OUT_NUM_SET(chunk_index_out-chunk_index_in);
		msg_buf[MV_DP_MSG_EXT_HDR_OUT_TOTAL_SIZE_WORD2] = cpu_to_be32 (MV_DP_MSG_EXT_HDR_OUT_TOTAL_SIZE_SET(0));

		cpu_to_be32s(&msg_buf[MV_DP_MSG_EXT_HDR_OUT_NUM_WORD3]);
		cpu_to_be32s(&msg_buf[MV_DP_MSG_EXT_HDR_IN_NUM_WORD1]);
		/*set total size*/
		tx_msg->msg_size = MV_DP_MSG_EXT_HDR_SIZE_CALC_B(chunk_index_out);

#ifdef MV_DP_USE_DRAM_PTR
		mv_dma_single_cpu_to_dev(ptr_buf, MV_DP_EXT_BUF_CHUNK_SIZE, DMA_TO_DEVICE);
#endif
		/*put tx message after ext buffer*/
		if (!tx_chunks && total_tx_size) {
			tmp = (u8 *)&msg_buf[MV_DP_MSG_EXT_HDR_SIZE_CALC_W(chunk_index_out)];
			for (w = 0; w < tx_msg->count ; w++) {
				MV_DP_LOG_DBG2("Message Populating, chunk_out_index:%d, w: %d\n", chunk_index_out, w);
				rc = msg_info->tx_handle((tmp + (w * tx_size)), ptr_in, w);
				if (MV_DP_RC_OK != rc) {
					MV_DP_LOG_ERR("Populate CFH opcode:%d, count: %d\n",
						      MV_DP_RC_ERR_EXT_BUF_POPULATE, opcode, tx_msg->count);
					rc = MV_DP_RC_ERR_FAILED;
					goto e_ret;
				}
				tx_msg->msg_size += tx_size;
			}
		}

	} /*use ext hdrs*/


	MV_DP_LOG_DBG2("Message Populated OK, EXT PTRS:%d(IN:%d), count:%d msg_size:%d\n",
		       chunk_index_out, chunk_index_in, tx_msg->count, tx_msg->msg_size);
	MV_DP_LOG_DBG3("EXIT %s\n", __func__);

	return MV_DP_RC_OK;

err_be:
	/*convert to ext header to host: call ext_hdr_to_host*/
#ifdef MV_DP_DEBUG
if (!(tx_msg->flags & MV_DP_F_CFH_EXT_HDR))
	MV_DP_LOG_ERR("No EXT HDR Present, count:%d\n", MV_DP_RC_ERR_WRONG_STATE, tx_msg->count);
#endif
	msg_buf[MV_DP_MSG_EXT_HDR_IN_NUM_WORD1] |= MV_DP_MSG_EXT_HDR_IN_NUM_SET(chunk_index_in);
	msg_buf[MV_DP_MSG_EXT_HDR_OUT_NUM_WORD3] |= MV_DP_MSG_EXT_HDR_OUT_NUM_SET(chunk_index_out-chunk_index_in);
	cpu_to_be32s(&msg_buf[MV_DP_MSG_EXT_HDR_IN_NUM_WORD1]);
	cpu_to_be32s(&msg_buf[MV_DP_MSG_EXT_HDR_OUT_NUM_WORD3]);

	mv_dp_ext_hdr_be_release(msg_buf);

/*consider returning the current rc*/
e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}



/*
 * mv_dp_msg_build_out
 *
 * Description:
 *       Builds a message from input data using parser from msg_info on per opcode type.
 *       this builder is used to build requests where there is an out_buffer and
 *       small input after out ext hdr pointers IMPORTANT: tx_size is not x Count but 1
 *
 * Parameters:
 *       tx_msg - record to populate, including generic opcode, count and msg_bufffer
 *       opcode - the fully specified opcode of the message to be polulated
 *       ptr_in - generic input to be populated into a message.
 *
 *Note:   tx_msg->opcode must not be changed
 * Returns:
 *        MV_DP_RC_OK - On success or an error code, otherwise.
 */
enum mv_dp_rc mv_dp_msg_build_out(struct mv_dp_msg_info_tx *tx_msg, enum mv_dp_msg_id opcode, void *ptr_in)
{

	u32			*msg_buf;
	u8			*msg_ext = NULL;
	enum mv_dp_rc		rc;

	int			chunk_index_out;
	int			tx_size, rx_size;
	int			rx_chunks;
	struct mv_dp_msg_info	*msg_info;
#ifdef MV_DP_USE_DRAM_PTR
	u32			*ptr_buf;
#endif


	MV_DP_LOG_DBG3("ENTER: %s tx_msg:%p, opcode:%d ptr_in%p, count:%d\n", __func__,
		       tx_msg, opcode, ptr_in, tx_msg->count);

	rc = MV_NSS_DP_FAILED;

	if (!tx_msg) {
		MV_DP_LOG_ERR("NULL tx_msg:%p ptr_in:%p opcode:%d, count:%d\n",
			      MV_DP_RC_ERR_NULL_PTR, tx_msg, ptr_in, opcode, tx_msg->count);
		rc = MV_NSS_DP_INVALID_PARAM;
		goto e_ret;
	}

	if (!MV_DP_MSGID_IS_OK(opcode)) {
		MV_DP_LOG_ERR("Illegal opocode:%d\n", MV_DP_RC_ERR_MSGID_ILLEGAL, opcode);
		rc = MV_NSS_DP_INVALID_PARAM;
		goto e_ret; /*should be MV_DP_WRONG_SIZE */
	}

#ifdef MV_DP_DEBUG
	if (!MV_DP_MSG_COUNT_VALID(tx_msg->count)) {
		MV_DP_LOG_ERR("Illegal count:%d opcode:%d\n", MV_DP_RC_ERR_COUNT, tx_msg->count, opcode);
		rc = MV_NSS_DP_INVALID_PARAM;
		goto e_ret; /*should be MV_DP_WRONG_SIZE */
	}
#endif

	msg_info = &mv_dp_instance.msg_info[opcode];
	msg_buf = tx_msg->msg_data;
	tx_size  = msg_info->tx_size;
	rx_size  = msg_info->rx_size;

	chunk_index_out = 0;
	tx_msg->msg_size = 0;
	tx_msg->flags = msg_info->tx_flags;

	if (!tx_size &&  (tx_msg->count * rx_size <= MV_DP_CFH_MSG_BUF_SIZE_B)) {
		/*no ext buffer same as regular build*/
		MV_DP_LOG_DBG2("Nothing to populate opcode:%d msg_size:%d count:%d rx_size:%d tx_size:%d\n",
			       opcode, tx_msg->msg_size, tx_msg->count, rx_size, tx_size);
		return MV_NSS_DP_OK;
	}
	/*get the number of rx chunks: tx_chunks must be 0*/

	rx_chunks = (rx_size) ? (MV_DP_EXT_BUF_GET_CHUNK_COUNT(tx_msg->count, rx_size)) : (0);

	/*check if there is enough space left for tx input after ext hdr NB: tx_size is single message not xcount*/
	if ((tx_size + MV_DP_MSG_EXT_HDR_SIZE_CALC_B(rx_chunks)) > MV_DP_CFH_MSG_BUF_SIZE_B) {
		MV_DP_LOG_ERR("Illegal msg size total opcode:%d msg_size:%d maximum:%d rx_chunks:%d count:%d\n",
			      MV_DP_RC_ERR_COUNT, opcode,
			      (tx_size + (rx_chunks * MV_DP_MSG_EXT_HDR_PTR_SIZE) + MV_DP_MSG_EXT_HDR_SIZE_B),
			      MV_DP_CFH_MSG_BUF_SIZE_B, rx_chunks, tx_msg->count);
			rc = MV_NSS_DP_INVALID_PARAM;
			goto e_ret; /*should be MV_DP_WRONG_SIZE */
	}

#ifdef MV_DP_USE_DRAM_PTR
		ptr_buf = kmalloc(MV_DP_EXT_BUF_CHUNK_SIZE, GFP_KERNEL);
		if (!ptr_buf) {
			MV_DP_LOG_ERR("PTR Buffer Allocation failed size:%d\n",
				      MV_DP_RC_ERR_ALLOC, MV_DP_EXT_BUF_CHUNK_SIZE);
			rc = MV_DP_RC_ERR_OUT_OF_RESOURCES;
			goto e_ret;
		}
		/* store buffer ptr*/
		msg_buf[MV_DP_MSG_EXT_HDR_PTR_WORD] =
				cpu_to_be32(MV_DP_MSG_EXT_HDR_PTR_SET(MV_DP_VIRT_TO_PHYS(ptr_buf)));
#endif


	/*populate out chunks*/
	msg_buf[MV_DP_MSG_EXT_HDR_IN_TOTAL_SIZE_WORD0] = cpu_to_be32(
						MV_DP_MSG_EXT_HDR_IN_TOTAL_SIZE_SET(0) |
						MV_DP_MSG_EXT_HDR_IN_FLAGS_SET(MV_DP_MSG_EXT_HDR_IN_FLAG_REUSE));
	msg_buf[MV_DP_MSG_EXT_HDR_OUT_TOTAL_SIZE_WORD2] = cpu_to_be32(
						MV_DP_MSG_EXT_HDR_OUT_TOTAL_SIZE_SET(0) |
						MV_DP_MSG_EXT_HDR_OUT_FLAGS_SET(0));

	msg_buf[MV_DP_MSG_EXT_HDR_IN_CHUNK_SIZE_WORD1] = MV_DP_MSG_EXT_HDR_IN_CHUNK_SIZE_SET(0);
	msg_buf[MV_DP_MSG_EXT_HDR_OUT_CHUNK_SIZE_WORD3] =
		MV_DP_MSG_EXT_HDR_OUT_CHUNK_SIZE_SET(MV_DP_EXT_BUF_CHUNK_SIZE);
	msg_buf[MV_DP_MSG_EXT_HDR_IN_NUM_WORD1] |= MV_DP_MSG_EXT_HDR_IN_NUM_SET(0);
	msg_buf[MV_DP_MSG_EXT_HDR_OUT_NUM_WORD3] |= MV_DP_MSG_EXT_HDR_OUT_NUM_SET(0);
	cpu_to_be32s(&msg_buf[MV_DP_MSG_EXT_HDR_IN_NUM_WORD1]);


	MV_DP_LOG_DBG2("EXT HDR opcode: %d, tx_size:%d, rx_size:%d, count:%d, ptrs_needed:%d(O=%d), each:%d(B)\n",
		       opcode, tx_size, rx_size, tx_msg->count, rx_chunks, rx_chunks,
		       MV_DP_EXT_BUF_CHUNK_SIZE);
	/*mark EXT is present*/
	tx_msg->flags |= MV_DP_F_CFH_EXT_HDR;

	chunk_index_out = 0;
	while (chunk_index_out < rx_chunks) {
		/*allocate new chunk and put it into buffer*/
		msg_ext = kmalloc(MV_DP_EXT_BUF_CHUNK_SIZE, GFP_KERNEL);
		if (!msg_ext) {
			MV_DP_LOG_ERR("Ext OUT Buffer Allocation failed chunk:%d size:%d\n",
				      MV_DP_RC_ERR_ALLOC, chunk_index_out, MV_DP_EXT_BUF_CHUNK_SIZE);
			rc = MV_DP_RC_ERR_OUT_OF_RESOURCES;
			goto err_be;
		}
		MV_DP_LOG_DBG2("Allocated OUT chunk, chunk_ind:%d\n", chunk_index_out);
#ifdef MV_DP_USE_DRAM_PTR
		ptr_buf[chunk_index_out] =
			cpu_to_be32(MV_DP_MSG_EXT_HDR_PTR_SET(MV_DP_VIRT_TO_PHYS(msg_ext)));
#else
		msg_buf[MV_DP_MSG_EXT_HDR_PTR_WORD + chunk_index_out] =
			cpu_to_be32(MV_DP_MSG_EXT_HDR_PTR_SET(MV_DP_VIRT_TO_PHYS(msg_ext)));
#endif
		mv_dma_single_cpu_to_dev(msg_ext, MV_DP_EXT_BUF_CHUNK_SIZE, DMA_FROM_DEVICE);
		chunk_index_out++;
	}

	msg_buf[MV_DP_MSG_EXT_HDR_OUT_NUM_WORD3] |= MV_DP_MSG_EXT_HDR_OUT_NUM_SET(chunk_index_out);
	cpu_to_be32s(&msg_buf[MV_DP_MSG_EXT_HDR_OUT_NUM_WORD3]);
	tx_msg->msg_size =  MV_DP_MSG_EXT_HDR_SIZE_CALC_B(chunk_index_out);

#ifdef MV_DP_USE_DRAM_PTR
		mv_dma_single_cpu_to_dev(ptr_buf, MV_DP_EXT_BUF_CHUNK_SIZE, DMA_TO_DEVICE);
#endif
	/*put input data after ext hdr: one entry only !*/

	rc = msg_info->tx_handle(((u8 *)msg_buf + MV_DP_MSG_EXT_HDR_SIZE_CALC_B(chunk_index_out)), ptr_in, 0);
	if (MV_DP_RC_OK != rc) {
		MV_DP_LOG_ERR("Populate CFH opcode:%d, count: %d\n",
			      MV_DP_RC_ERR_EXT_BUF_POPULATE, opcode, tx_msg->count);
		rc = MV_DP_RC_ERR_FAILED;
		goto err_be;
	}
	tx_msg->msg_size += tx_size;

	MV_DP_LOG_DBG2("Message Populated OK, EXT PTRS OUT:%d, count:%d msg_size:%d, FL:0x%04X\n",
		       chunk_index_out, tx_msg->count, tx_msg->msg_size, tx_msg->flags);
	MV_DP_LOG_DBG3("EXIT %s\n", __func__);

	return MV_DP_RC_OK;

err_be:
	/*convert to ext header to host: call ext_hdr_to_host*/
#ifdef MV_DP_DEBUG
if (!(tx_msg->flags & MV_DP_F_CFH_EXT_HDR))
	MV_DP_LOG_ERR("No EXT HDR Present, count:%d\n", MV_DP_RC_ERR_WRONG_STATE, tx_msg->count);
#endif
	msg_buf[MV_DP_MSG_EXT_HDR_OUT_NUM_WORD3] |= MV_DP_MSG_EXT_HDR_OUT_NUM_SET(chunk_index_out);
	cpu_to_be32s(&msg_buf[MV_DP_MSG_EXT_HDR_OUT_NUM_WORD3]);

	mv_dp_ext_hdr_be_release(msg_buf);

/*consider returning the current rc*/
e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}

/*
 * mv_dp_msg_build_bulk
 *
 * Description:
 *       Builds a bulk get message for a given opcode.
 *       Builds ext header (if needed) and index and count fields after ext header
 *
 * Parameters:
 *       tx_msg - record to populate, including generic opcode, count and msg_bufffer
 *       opcode - the fully specified opcode of the message to be polulated
 *       index  - first index to return into a message.
 *       count  - number of entries to return
 *
 *Note:   tx_msg->opcode must not be changed
 * Returns:
 *        MV_DP_RC_OK - On success or an error code, otherwise.
 */
enum mv_dp_rc mv_dp_msg_build_bulk(struct mv_dp_msg_info_tx *tx_msg, enum mv_dp_msg_id opcode, void *bulk)
{

	u32			*msg_buf;
	u8			*msg_ext = NULL;
	enum mv_dp_rc		rc;

	int			chunk_index_out, ind_off = 0;
	int			tx_size, rx_size;
	int			rx_chunks;
	struct mv_dp_msg_info	*msg_info;
#ifdef MV_DP_USE_DRAM_PTR
	u32			*ptr_buf;
#endif


	MV_DP_LOG_DBG3("ENTER: %s tx_msg:%p, opcode:%d bulk:%p\n", __func__,
		       tx_msg, opcode, bulk);

	rc = MV_NSS_DP_FAILED;

	if (!MV_DP_MSGID_IS_OK(opcode)) {
		MV_DP_LOG_ERR("Illegal opocode:%d\n", MV_DP_RC_ERR_MSGID_ILLEGAL, opcode);
		rc = MV_NSS_DP_INVALID_PARAM;
		goto e_ret;
	}

	if (!bulk) {
		MV_DP_LOG_ERR("NULL bulk\n", MV_DP_RC_ERR_NULL_PTR);
		rc = MV_NSS_DP_INVALID_PARAM;
		goto e_ret;
	}
#ifdef MV_DP_DEBUG
	if (!tx_msg) {
		MV_DP_LOG_ERR("NULL tx_msg:%p index: %d opcode:%d\n",
			      MV_DP_RC_ERR_NULL_PTR, tx_msg, ((struct mv_dp_msg_bulk *)bulk)->index, opcode);
		rc = MV_NSS_DP_INVALID_PARAM;
		goto e_ret;
	}

	if (!MV_DP_MSG_COUNT_VALID(tx_msg->count)) {
		MV_DP_LOG_ERR("Illegal count:%d opcode:%d\n", MV_DP_RC_ERR_COUNT, tx_msg->count, opcode);
		rc = MV_NSS_DP_INVALID_PARAM;
		goto e_ret; /*should be MV_DP_WRONG_SIZE */
	}
#endif

	msg_info = &mv_dp_instance.msg_info[opcode];
	msg_buf = tx_msg->msg_data;
	tx_size  = msg_info->tx_size;
	rx_size  = msg_info->rx_size;

	chunk_index_out = 0;
	tx_msg->msg_size = 0;
	/*tx_msg->count = count;*/
	tx_msg->flags = msg_info->tx_flags;


	if ((tx_msg->count * rx_size) + tx_size > MV_DP_CFH_MSG_BUF_SIZE_B) {
		/*use ext buffer*/
		/*get the number of rx chunks: tx_chunks must be 0*/
		rx_chunks = (rx_size) ? (MV_DP_EXT_BUF_GET_CHUNK_COUNT(tx_msg->count, rx_size)) : (0);
		if ((tx_size + MV_DP_MSG_EXT_HDR_SIZE_CALC_B(rx_chunks)) > MV_DP_CFH_MSG_BUF_SIZE_B) {
			MV_DP_LOG_ERR("Illegal msg size total opcode:%d msg_size:%d maximum:%d rx_chunks:%d count:%d\n",
				      MV_DP_RC_ERR_COUNT, opcode,
				      (tx_size + (rx_chunks * MV_DP_MSG_EXT_HDR_PTR_SIZE) + MV_DP_MSG_EXT_HDR_SIZE_B),
				      MV_DP_CFH_MSG_BUF_SIZE_B, rx_chunks, tx_msg->count);
			rc = MV_NSS_DP_INVALID_PARAM;
			goto e_ret; /*should be MV_DP_WRONG_SIZE */
		}

		/*check if total pointers number is ok*/
		if (MV_DP_MSG_EXT_HDR_MAX_PTR < rx_chunks) {
			MV_DP_LOG_ERR("Too many Buf ptrs needed opcode:%d,count:%d,ptrs_needed:%d,each:%d(B),max:%d\n",
			      MV_DP_RC_ERR_INVALID_PARAM, opcode, tx_msg->count, rx_chunks,
			      MV_DP_EXT_BUF_CHUNK_SIZE, MV_DP_MSG_EXT_HDR_MAX_PTR);
			rc = MV_NSS_DP_INVALID_PARAM;
			goto e_ret;
		}

#ifdef MV_DP_USE_DRAM_PTR
		ptr_buf = kmalloc(MV_DP_EXT_BUF_CHUNK_SIZE, GFP_KERNEL);
		if (!ptr_buf) {
			MV_DP_LOG_ERR("PTR Buffer Allocation failed size:%d\n",
				      MV_DP_RC_ERR_ALLOC, MV_DP_EXT_BUF_CHUNK_SIZE);
			rc = MV_DP_RC_ERR_OUT_OF_RESOURCES;
			goto e_ret;
		}
		/* store buffer ptr*/
		msg_buf[MV_DP_MSG_EXT_HDR_PTR_WORD] =
				cpu_to_be32(MV_DP_MSG_EXT_HDR_PTR_SET(MV_DP_VIRT_TO_PHYS(ptr_buf)));
#endif


	/*populate out chunks*/
	msg_buf[MV_DP_MSG_EXT_HDR_IN_TOTAL_SIZE_WORD0] =
		cpu_to_be32(MV_DP_MSG_EXT_HDR_IN_TOTAL_SIZE_SET(0)
			     | MV_DP_MSG_EXT_HDR_IN_FLAGS_SET(MV_DP_MSG_EXT_HDR_IN_FLAG_REUSE));
	msg_buf[MV_DP_MSG_EXT_HDR_OUT_TOTAL_SIZE_WORD2] = cpu_to_be32(MV_DP_MSG_EXT_HDR_OUT_TOTAL_SIZE_SET(0));
	msg_buf[MV_DP_MSG_EXT_HDR_IN_CHUNK_SIZE_WORD1] = MV_DP_MSG_EXT_HDR_IN_CHUNK_SIZE_SET(0);
	msg_buf[MV_DP_MSG_EXT_HDR_OUT_CHUNK_SIZE_WORD3] =
		MV_DP_MSG_EXT_HDR_OUT_CHUNK_SIZE_SET(MV_DP_EXT_BUF_CHUNK_SIZE);
	msg_buf[MV_DP_MSG_EXT_HDR_IN_NUM_WORD1] |= MV_DP_MSG_EXT_HDR_IN_NUM_SET(0);
	msg_buf[MV_DP_MSG_EXT_HDR_OUT_NUM_WORD3] |= MV_DP_MSG_EXT_HDR_OUT_NUM_SET(0);
	cpu_to_be32s(&msg_buf[MV_DP_MSG_EXT_HDR_IN_NUM_WORD1]);


	MV_DP_LOG_DBG2("EXT HDR opcode: %d, tx_size:%d, rx_size:%d, count:%d, ptrs_needed:%d(O=%d), each:%d(B)\n",
		       opcode, tx_size, rx_size, tx_msg->count, rx_chunks, rx_chunks,
		       MV_DP_EXT_BUF_CHUNK_SIZE);
	/*mark EXT is present*/
	tx_msg->flags |= MV_DP_F_CFH_EXT_HDR;

	chunk_index_out = 0;
	while (chunk_index_out < rx_chunks) {
		/*allocate new chunk and put it into buffer*/
		msg_ext = kmalloc(MV_DP_EXT_BUF_CHUNK_SIZE, GFP_KERNEL);
		if (!msg_ext) {
			MV_DP_LOG_ERR("Ext OUT Buffer Allocation failed chunk:%d size:%d\n",
				      MV_DP_RC_ERR_ALLOC, chunk_index_out, MV_DP_EXT_BUF_CHUNK_SIZE);
			rc = MV_DP_RC_ERR_OUT_OF_RESOURCES;
			goto err_be;
		}
		MV_DP_LOG_DBG2("Allocated OUT chunk, chunk_ind:%d\n", chunk_index_out);
#ifdef MV_DP_USE_DRAM_PTR
		ptr_buf[chunk_index_out] =
			cpu_to_be32(MV_DP_MSG_EXT_HDR_PTR_SET(MV_DP_VIRT_TO_PHYS(msg_ext)));
#else
		msg_buf[MV_DP_MSG_EXT_HDR_PTR_WORD + chunk_index_out] =
			cpu_to_be32(MV_DP_MSG_EXT_HDR_PTR_SET(MV_DP_VIRT_TO_PHYS(msg_ext)));
#endif
		mv_dma_single_cpu_to_dev(msg_ext, MV_DP_EXT_BUF_CHUNK_SIZE, DMA_FROM_DEVICE);
		chunk_index_out++;
	}

		msg_buf[MV_DP_MSG_EXT_HDR_OUT_NUM_WORD3] |= MV_DP_MSG_EXT_HDR_OUT_NUM_SET(chunk_index_out);
		cpu_to_be32s(&msg_buf[MV_DP_MSG_EXT_HDR_OUT_NUM_WORD3]);
		tx_msg->msg_size =  MV_DP_MSG_EXT_HDR_SIZE_CALC_B(chunk_index_out);

#ifdef MV_DP_USE_DRAM_PTR
		mv_dma_single_cpu_to_dev(ptr_buf, MV_DP_EXT_BUF_CHUNK_SIZE, DMA_TO_DEVICE);
#endif
		ind_off = MV_DP_MSG_EXT_HDR_SIZE_CALC_W(chunk_index_out);
	}

	/*put input data after ext hdr (if any): one entry only !*/
	if (tx_size) {
		/*no ext buffer same all fits inside*/
		MV_DP_LOG_DBG2("No Ext HDR populated opcode:%d msg_size:%d count:%d rx_size:%d tx_size:%d\n",
		       opcode, tx_msg->msg_size, tx_msg->count, rx_size, tx_size);
		rc = msg_info->tx_handle(&msg_buf[ind_off], bulk, 0);
		if (MV_DP_RC_OK != rc) {
			MV_DP_LOG_ERR("Populate CFH opcode:%d, count: %d\n",
				      MV_DP_RC_ERR_EXT_BUF_POPULATE, opcode, tx_msg->count);
			rc = MV_DP_RC_ERR_FAILED;
			goto err_le;
		}

	}

	tx_msg->msg_size += tx_size;

	MV_DP_LOG_DBG2("Message Populated OK, EXT PTRS OUT:%d, count:%d msg_size:%d, FL:0x%04X\n",
		       chunk_index_out, tx_msg->count, tx_msg->msg_size, tx_msg->flags);
	MV_DP_LOG_DBG3("EXIT %s\n", __func__);

	return MV_DP_RC_OK;

err_be:
	/*convert to ext header to host: call ext_hdr_to_host*/
#ifdef MV_DP_DEBUG
if (tx_msg->flags & MV_DP_F_CFH_EXT_HDR)
	MV_DP_LOG_ERR("No EXT HDR Present, count:%d\n", MV_DP_RC_ERR_WRONG_STATE, tx_msg->count);
#endif
	msg_buf[MV_DP_MSG_EXT_HDR_OUT_NUM_WORD3] |= MV_DP_MSG_EXT_HDR_OUT_NUM_SET(chunk_index_out);
	cpu_to_be32s(&msg_buf[MV_DP_MSG_EXT_HDR_OUT_NUM_WORD3]);

err_le:	mv_dp_ext_hdr_be_release(msg_buf);

/*consider returning the current rc*/
e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}



/*
 * mv_dp_msg_parse
 *
 * Description:
 *      calls specific message parser on CFH or EXT OUT buffers, than calls callback.
 *      checks size before parser is called
 *
 * Parameters:
 *       rx_msg - RX message
 *       ptr_buf- pending message as kept in the buffer
 *
 *Note:   should reach here only if meaninful buffer is present, i.e. only FW_OK and END_OF_LIST
 * Returns:
 *        MV_DP_RC_OK - On success or an error code, otherwise.
 */
static enum mv_dp_rc mv_dp_msg_parse(struct mv_dp_msg_info_rx *rx_msg, void *ptr_buf)
{
	mv_nss_dp_event_t	param;
	struct mv_dp_msg_buf	*msg_rx_buf = (struct mv_dp_msg_buf *) ptr_buf;
	u32			*msg_buf, *ptr_chunks;
	int			entity;
	mv_dp_msg_rx_func_new	rx_parser;
	int			rx_size, cb_size;
	u8			*out_struct = NULL;
	int			chunk_ind, ext_ind;
	int			expected_size, hdr_size;
	u8			*ext_ptr;
	int			in_chunks, out_chunks;

	MV_DP_LOG_DBG3("ENTER: %s rx_msg:%p ptr_buf:%p\n", __func__, rx_msg, ptr_buf);

#ifdef MV_DP_DEBUG
	if (!rx_msg || !ptr_buf) {
		MV_DP_LOG_ERR("NULL rx_msg:%p or ptr_in:%p\n", MV_DP_RC_ERR_NULL_PTR, rx_msg, ptr_buf);
		return MV_DP_RC_ERR_NULL_PTR;
	}

	if (!MV_DP_MSGID_IS_OK(rx_msg->opcode)) {
		MV_DP_LOG_ERR("Illegal opocode:%d sn:%d\n", MV_DP_RC_ERR_MSGID_ILLEGAL, rx_msg->opcode, rx_msg->sn);
		return MV_DP_RC_ERR_MSGID_ILLEGAL;
	}

	if (!(rx_msg->rc == MV_NSS_DP_END_OF_LIST  ||  rx_msg->rc == MV_NSS_DP_OK)) {
		MV_DP_LOG_ERR("Illegal State Status:%d opocode:%d sn:%d\n",
			      MV_DP_RC_ERR_MSG_PARSE_FAILED, rx_msg->rc, rx_msg->opcode, rx_msg->sn);
		goto err_cb;
	}
#endif


	rx_size = mv_dp_instance.msg_info[rx_msg->opcode].rx_size;

	rx_parser = mv_dp_instance.msg_info[rx_msg->opcode].rx_handle;
	/*check sizes*/
	/*need special status to call a callback*/
	if (rx_size == 0 && rx_parser) {
		/*perform rx internal callback if needed*/
		MV_DP_LOG_DBG1("%s: Internal parser call for opcode:%d  sn:%d, count:%d msg_size:%d\n", __func__,
			       rx_msg->opcode, rx_msg->sn, rx_msg->num_ok, rx_msg->msg_size);
		if (MV_DP_RC_OK != rx_parser(rx_msg, ptr_buf)) {
			MV_DP_LOG_ERR("internal Parser failed for opocode:%d rx_sise:%d msg_size:%d\n",
				      MV_DP_RC_ERR_MSG_PARSE_FAILED, rx_msg->opcode, rx_size, rx_msg->msg_size);
			goto err_cb;
		}
	}

	/*enough to check res, as if no callback the whole is 0*/
	if (!msg_rx_buf->res.cb) {
		/*no callback defined*/
		MV_DP_LOG_DBG1("%s: NO CB for opcode:%d  sn:%d, ptr_buf:%p, count:%d msg_size:%d\n", __func__,
			       rx_msg->opcode, rx_msg->sn, ptr_buf, rx_msg->num_ok, rx_msg->msg_size);
		return MV_DP_RC_OK;
	}
	/*if rx_size ==-1 call parser once to get and recurse on the fully specified opcode*/
	/*if rx_size > 0 but cb_size =0 call the call back but don't allocated, else --> parse*/
	/*if no callback -- exit*/

	param.cookie = msg_rx_buf->res.cookie;
	param.xid = msg_rx_buf->res.xid;
	param.type = mv_dp_instance.msg_info[rx_msg->opcode].return_evt;
	param.count = rx_msg->num_ok;
	param.status = rx_msg->rc;
	param.params.notify_msg = msg_rx_buf->out_buf;


	if (rx_size == 0) {
		/*nothing to parse -> callback if any and return OK*/
		MV_DP_LOG_DBG1("%s: nothing to parse -- return ok opcode:%d  sn:%d, count:%d msg_size:%d\n", __func__,
			       rx_msg->opcode, rx_msg->sn, rx_msg->num_ok, rx_msg->msg_size);
		goto ret_ok;
	}
#if 0
	if (!rx_parser) {
		/*error as the message is present or expected*/
		MV_DP_LOG_ERR("Null Parser for opocode:%d rx_size:%d msg_size:%d\n",
			      MV_DP_RC_ERR_NULL_PTR, rx_msg->opcode,
			      rx_size, rx_msg->msg_size);
		goto err_cb;
	}
#endif
	/*check if additional type specification needed*/
	if (MV_DP_MSG_SIZE_INVALID == rx_size) {
		/*e.g. for PortGet, the parser will change the opcode inside the rx_msg to the fully specified*/
		if (MV_DP_RC_OK != rx_parser(rx_msg, ptr_buf)) { /*this is supposed to update the opcode in rx_msg*/
			MV_DP_LOG_ERR("Fix opcode parser failed for opocode:%d rx_sise:%d msg_size:%d\n",
				      MV_DP_RC_ERR_MSG_PARSE_FAILED, rx_msg->opcode,
				      rx_size, rx_msg->msg_size);
			goto err_cb;
		}
		rx_size = mv_dp_instance.msg_info[rx_msg->opcode].rx_size;
		param.type = mv_dp_instance.msg_info[rx_msg->opcode].return_evt;

		MV_DP_LOG_DBG2("%s: opcode changed to:%d  new rx_size:%d\n", __func__, rx_msg->opcode, rx_size);
		/*update the parser to a new one*/
		rx_parser = mv_dp_instance.msg_info[rx_msg->opcode].rx_handle;
		if (!rx_parser) {
			MV_DP_LOG_DBG1("%s: null parser -- return ok opcode:%d  sn:%d, count:%d msg_size:%d\n", __func__,
				       rx_msg->opcode, rx_msg->sn, rx_msg->num_ok, rx_msg->msg_size);
			goto ret_ok;
		}
	}
	cb_size = mv_dp_instance.msg_info[rx_msg->opcode].rx_cb_size;

	MV_DP_LOG_DBG2("%s: opcode:%d  sn:%d, ptr_buf: %p, count:%d msg_size:%d, rx_size:%d, cb_size:%d\n", __func__,
		       rx_msg->opcode, rx_msg->sn, ptr_buf, rx_msg->num_ok, rx_msg->msg_size, rx_size, cb_size);

	if (msg_rx_buf->out_buf) {
		out_struct = msg_rx_buf->out_buf;
	} else {
		out_struct = kmalloc((cb_size * rx_msg->num_ok), GFP_ATOMIC);
		if (!out_struct) {
			MV_DP_LOG_ERR("Failed Alloc return struct opocode:%d, sn:%d, size:%d\n",
				      MV_DP_RC_ERR_NULL_PTR, rx_msg->opcode, rx_msg->sn, cb_size * rx_msg->num_ok);
			goto err_cb;
		}
	}

	param.params.notify_msg = out_struct;
	msg_buf = rx_msg->msg_data;
	ext_ptr = 0;
	out_chunks = in_chunks = hdr_size = expected_size = 0;
	if (rx_msg->flags & MV_DP_F_CFH_EXT_HDR) {
		out_chunks = MV_DP_MSG_EXT_HDR_OUT_NUM_GET(be32_to_cpu(msg_buf[MV_DP_MSG_EXT_HDR_OUT_NUM_WORD3]));
		in_chunks = MV_DP_MSG_EXT_HDR_IN_NUM_GET(be32_to_cpu(msg_buf[MV_DP_MSG_EXT_HDR_IN_NUM_WORD1]));
		hdr_size = MV_DP_MSG_EXT_HDR_SIZE_CALC_B(out_chunks + in_chunks);
	}

	MV_DP_LOG_DBG3("out_chunks:%d, in_chunks:%d, msg_size:%d hdr_size: %d\n",
		       out_chunks, in_chunks, rx_msg->msg_size,
		       MV_DP_MSG_EXT_HDR_SIZE_CALC_B(out_chunks + in_chunks));
	if (out_chunks) {
		if (rx_msg->msg_size != hdr_size) {
			MV_DP_LOG_ERR("Wrong ext_hdr size:%d expected:%d num_ok:%d, opocode:%d sn:%d\n",
				      MV_DP_RC_ERR_MSG_SIZE, rx_msg->msg_size, hdr_size,
				      rx_msg->num_ok, rx_msg->opcode, rx_msg->sn);
				goto err_cb;
		}
		/*EXT OUT HEADER size check*/
		/*get out size*/
		expected_size = MV_DP_MSG_EXT_HDR_OUT_TOTAL_SIZE_GET(
				be32_to_cpu(msg_buf[MV_DP_MSG_EXT_HDR_OUT_TOTAL_SIZE_WORD2]));
		if (rx_size * rx_msg->num_ok != expected_size) {
			MV_DP_LOG_ERR("Wrong ext hdr data size:%d expected:%d num_ok:%d, opocode:%d sn:%d\n",
				      MV_DP_RC_ERR_MSG_SIZE, expected_size, rx_size * rx_msg->num_ok,
				      rx_msg->num_ok, rx_msg->opcode, rx_msg->sn);
				goto err_cb;
		}

#ifdef MV_DP_USE_DRAM_PTR
	ptr_chunks = MV_DP_PHYS_TO_VIRT(MV_DP_MSG_EXT_HDR_PTR_GET(be32_to_cpu(msg_buf[MV_DP_MSG_EXT_HDR_PTR_WORD])));
#else
	ptr_chunks = &msg_buf[MV_DP_MSG_EXT_HDR_PTR_WORD];
#endif

		/*need to fix for in pointers before out ptrs*/
		chunk_ind = in_chunks;
		for (entity = 0; entity < rx_msg->num_ok; entity++) {
			ext_ind = entity % MV_DP_EXT_BUF_GET_ENTRY_COUNT(rx_size);
			if (!ext_ind) {
				MV_DP_LOG_DBG2("%s: Processing chunk_ind:%d entity:%d\n", __func__, chunk_ind, entity);
				ext_ptr = (u8 *)(MV_DP_PHYS_TO_VIRT(MV_DP_MSG_EXT_HDR_PTR_GET(
						be32_to_cpu(ptr_chunks[chunk_ind]))));
				/*invalidate cache*/
				chunk_ind++;
			}

			MV_DP_LOG_DBG3("%s: parser call: chunk_ind:%d entity:%d ext_ind:%d, p1:%p, p2:%p\n", __func__,
				       chunk_ind, entity, ext_ind, ext_ptr + ext_ind * rx_size,
				       ((u8 *)out_struct) + entity * cb_size);
			if (MV_DP_RC_OK != (rx_parser(((u8 *)out_struct) + entity * cb_size,
						      ext_ptr + ext_ind * rx_size))) {
					MV_DP_LOG_ERR("Parsing failed opocode:%d, sn:%d, entity:%d\n",
						      MV_DP_RC_ERR_MSG_PARSE_FAILED,
						      rx_msg->opcode, rx_msg->sn, entity);
					goto err_cb;
				}
		}
		MV_DP_LOG_DBG3("%s: Processed: chunks:%d entities:%d\n", __func__, chunk_ind, entity);

	} else {
	/*CFH MSG -- could be after ext buffer as well*/
		if (rx_size * rx_msg->num_ok != rx_msg->msg_size - hdr_size) {
			MV_DP_LOG_ERR("Wrong msg size:%d(-hdr:%d) expected:%d num_ok:%d, opocode:%d sn:%d\n",
				      MV_DP_RC_ERR_MSG_SIZE, rx_msg->msg_size, hdr_size, (rx_size * rx_msg->num_ok),
				      rx_msg->num_ok, rx_msg->opcode, rx_msg->sn);
				goto err_cb;
		}
		ext_ptr = (u8 *)&msg_buf[hdr_size];

		for (entity = 0; entity < rx_msg->num_ok; entity++) {
			MV_DP_LOG_DBG2("Populating entity:%d, msg_ptr:%p struct_ptr:%p\n",
				       entity, ext_ptr + entity * rx_size, out_struct + entity * cb_size);
			if (rx_parser) {
				if (MV_DP_RC_OK != (rx_parser(((u8 *)out_struct) + entity * cb_size,
						(ext_ptr + entity * rx_size)))) {
					MV_DP_LOG_ERR("Parsing failed opocode:%d, sn:%d, entity:%d\n",
							MV_DP_RC_ERR_MSG_PARSE_FAILED, rx_msg->opcode, rx_msg->sn, entity);
					goto err_cb;
				}
			} else {
				MV_DP_LOG_DBG2("No rx parse, memcpy: msg_ptr:%p struct_ptr:%p\n",
					       ext_ptr, out_struct);
				/* TODO: verify size */
				memcpy(out_struct, ext_ptr, cb_size);
			}

		}
	}

ret_ok:
#ifdef MV_DP_DEBUG
	if (MV_DP_LOG_TRC_ON(rx_msg->opcode, MV_DP_LOG_TRC_CB))
		mv_dp_show_msg_cb(&param, 0);
#endif
	MV_DP_LOG_DBG2("Parsed OK opcode:%d  sn:%d, count:%d msg_size:%d out_data:%p\n",
		       rx_msg->opcode, rx_msg->sn, rx_msg->num_ok, rx_msg->msg_size, param.params.notify_msg);
	/*callback if defined*/
	msg_rx_buf->res.cb(&param);
	if (0 == msg_rx_buf->out_buf && out_struct) {
		MV_DP_LOG_DBG3("Released out_struct\n");
		kfree(out_struct);
	}

	return MV_DP_RC_OK;

err_cb:
	param.count = 0;
	param.status = MV_NSS_DP_FAILED;
	param.params.notify_msg = 0;
	MV_DP_LOG_DBG2("Parse Failed opcode:%d  sn:%d, count:%d msg_size:%d out_data:%p\n",
		       rx_msg->opcode, rx_msg->sn, rx_msg->num_ok, rx_msg->msg_size, param.params.notify_msg);

	msg_rx_buf->res.cb(&param);
	if (0 == msg_rx_buf->out_buf && out_struct)
		kfree(out_struct);
	return MV_NSS_DP_FAILED;
}


static enum mv_dp_rc mv_dp_event_parse(struct mv_dp_msg_info_rx *rx_msg)
{
	mv_nss_dp_event_t	param;
	mv_dp_msg_rx_func_new	rx_parser;
	u8			*out_struct = NULL;
	mv_nss_dp_evt_handler   event_hndl;

	MV_DP_LOG_DBG3("ENTER: %s rx_msg:%p\n", __func__, rx_msg);

#ifdef MV_DP_DEBUG
	if (!rx_msg) {
		MV_DP_LOG_ERR("NULL rx_msg:%p\n", MV_DP_RC_ERR_NULL_PTR, rx_msg);
		return MV_DP_RC_ERR_NULL_PTR;
	}

	if (!MV_DP_EVENTID_IS_OK(rx_msg->opcode)) {
		MV_DP_LOG_ERR("Illegal opocode:%d sn:%d\n", MV_DP_RC_ERR_MSGID_ILLEGAL, rx_msg->opcode, rx_msg->sn);
		return MV_DP_RC_ERR_MSGID_ILLEGAL;
	}
#endif

	/*get the event handler*/
	event_hndl = mv_dp_get_event_hndl(MV_DP_MSGID_TO_EVENTID(rx_msg->opcode));
	if (!event_hndl) {
		MV_DP_LOG_DBG1("NO Event Handler for Event: %d\n", MV_DP_MSGID_TO_EVENTID(rx_msg->opcode));
		return MV_DP_RC_OK;
	}

	rx_parser = mv_dp_instance.msg_info[rx_msg->opcode].rx_handle;
	if (!rx_parser) {
		MV_DP_LOG_DBG1("NO Parser for Event MSG_ID: %d\n", rx_msg->opcode);
		return MV_DP_RC_OK;
	}

	param.cookie = 0;
	param.xid = 0;
	param.type = mv_dp_instance.msg_info[rx_msg->opcode].return_evt;
	param.count = rx_msg->num_ok;
	param.status = rx_msg->rc;


	out_struct = kmalloc(((mv_dp_instance.msg_info[rx_msg->opcode].rx_cb_size) * rx_msg->num_ok), GFP_ATOMIC);
	if (!out_struct) {
		MV_DP_LOG_ERR("Failed Alloc return struct opocode:%d, sn:%d, size:%d\n",
			      MV_DP_RC_ERR_NULL_PTR, rx_msg->opcode, rx_msg->sn,
			      (mv_dp_instance.msg_info[rx_msg->opcode].rx_cb_size) * rx_msg->num_ok);
		goto err_cb;
	}

	param.params.notify_msg = out_struct;

	/*call the parser then the handler*/
	if (MV_DP_RC_OK != rx_parser(out_struct, rx_msg)) {
		MV_DP_LOG_ERR("Event Parse Failed opcode:%d msg_size:%d\n",
			      MV_DP_RC_ERR_EVENT_PARSE_FAILED, rx_msg->opcode,
			      rx_msg->msg_size);
		goto err_cb;
	}

#ifdef MV_DP_DEBUG
	if (MV_DP_LOG_TRC_ON(rx_msg->opcode, MV_DP_LOG_TRC_CB))
		mv_dp_show_msg_cb(&param, 0);
#endif
	MV_DP_LOG_DBG2("Parsed OK opcode:%d  sn:%d, count:%d msg_size:%d out_data:%p sys_msg:%p\n",
		       rx_msg->opcode, rx_msg->sn, rx_msg->num_ok, rx_msg->msg_size, out_struct,
			   param.params.notify_msg);
	/*callback if defined*/
	event_hndl(&param);
	kfree(out_struct);
	return MV_DP_RC_OK;

err_cb:
	param.count = 0;
	param.status = MV_NSS_DP_FAILED;
	param.params.notify_msg = NULL;
	MV_DP_LOG_DBG2("Parse Failed opcode:%d  sn:%d, count:%d msg_size:%d out_data:%p\n",
		       rx_msg->opcode, rx_msg->sn, rx_msg->num_ok, rx_msg->msg_size, param.params.notify_msg);

	event_hndl(&param);
	kfree(out_struct);
	return MV_NSS_DP_FAILED;
}

static inline mv_nss_dp_evt_handler mv_dp_get_event_hndl(int id)
{
	return mv_dp_instance.event_info[id].evt;
}

inline void mv_dp_inc_error(int num)
{
	mv_dp_instance.errors[num]++;
}

inline u8 mv_dp_dbg_lvl(void)
{
	return mv_dp_instance.dbg_lvl;
}


inline u8 mv_dp_msg_evt_type(int num)
{
	return mv_dp_instance.msg_info[num].return_evt;
}

inline u8 mv_dp_msg_trace_type(int num)
{
	return mv_dp_instance.msg_info[num].trace;
}

const char *mv_dp_err_name_get(int num)
{
	if (MV_DP_ERR_NUM_IS_OK(num))
		return mv_dp_instance.error_info[num].name;
	else
		return "ERROR";
}



#ifdef MV_DP_USE_LIST_BUFFER
static enum mv_dp_rc mv_dp_rx_buffer_init(struct mv_dp_msg_buf *buffer)
{
	INIT_LIST_HEAD(&buffer->list_elem);
	return MV_DP_RC_OK;
}


static enum mv_dp_rc mv_dp_rx_buffer_release(struct mv_dp_ch_data *ch)
{

	struct mv_dp_msg_buf *tmp;

	/*for each member --> callback with error*/
	struct list_head *curr, *q;

	MV_DP_LOG_DBG3("ENTER: %s\n", __func__);

	list_for_each_safe(curr, q, &(ch->rx_buf.list_elem)) {
		tmp = list_entry(curr, struct mv_dp_msg_buf, list_elem);
		ch->api_stats[tmp->opcode].err_msg_rx++;
		mv_dp_rx_buf_delete(ch, tmp, true);
	}

	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_DP_RC_OK;
}


static enum mv_dp_rc mv_dp_rx_buffer_age(struct mv_dp_ch_data *ch)
{

	struct mv_dp_msg_buf *tmp;
	unsigned long   iflags = 0;
	struct list_head *curr;
	struct list_head *q;


	MV_DP_LOCK(&ch->lock, iflags);
	list_for_each_safe(curr, q, &(ch->rx_buf.list_elem)) {
		tmp = list_entry(curr, struct mv_dp_msg_buf, list_elem);
		/*decriment age --> if 0 --> remove*/
		if ((--(tmp->age)) == 0) {
			MV_DP_LOG_ERR("Aged opcode:%d, sn:%d\n", MV_DP_RC_ERR_AGED, tmp->opcode, tmp->sn);
			ch->api_stats[tmp->opcode].err_msg_rx++;
			mv_dp_rx_buf_delete(ch, tmp, true);
		}
	}
	MV_DP_UNLOCK(&ch->lock, iflags);

	return MV_DP_RC_OK;
}


static struct mv_dp_msg_buf *mv_dp_rx_buffer_find_sn(struct mv_dp_msg_buf *buffer, u16 sn)
{
	struct list_head *curr;
	struct list_head *q;
	struct mv_dp_msg_buf *tmp;

#ifdef MV_DP_DEBUG
	if (!buffer) {
		MV_DP_LOG_ERR("Find by sn: NULL RX BUF, sn:%d\n", MV_DP_RC_ERR_NULL_PTR, sn);
		return NULL;
	}
#endif

	list_for_each_safe(curr, q, &buffer->list_elem) {
		tmp = list_entry(curr, struct mv_dp_msg_buf, list_elem);
		if (tmp->sn == sn)
			return tmp;
	}

	MV_DP_LOG_DBG2("Not Found sn:%d\n", sn);

	return NULL;
}


#else

#endif


u32 _mv_dp_virt_to_phys(void *v_addr)
{
	u32 p_addr = virt_to_phys(v_addr);
	if (!p_addr)
		MV_DP_LOG_ERR("NULL P ADDR for V ADDR:%p\n", MV_DP_RC_ERR_NULL_PTR, v_addr);

	MV_DP_LOG_DBG2("%s: v_addr:%p -> p_addr:0x%08X\n", __func__, v_addr, p_addr);

	return p_addr;
}

void *_mv_dp_phys_to_virt(phys_addr_t p_addr)
{

	void *v_addr = phys_to_virt(p_addr);

	if (!v_addr)
		MV_DP_LOG_ERR("NULL V ADDR for P ADDR:0x%08llx\n", MV_DP_RC_ERR_NULL_PTR, p_addr);
	MV_DP_LOG_DBG2("%s: p_addr:0x%08llX -> v_addr:%p\n", __func__, p_addr, v_addr);

	return v_addr;
}






/*
 * mv_dp_init
 *
 * Description:
 *       mv_dpapi.ko initialization routine.
 *
 * Parameters:
 *       None
 *
 * Returns:
 *        0 - On success or an error code, otherwise.
 */

enum mv_dp_rc mv_dp_online_set(const mv_nss_dp_init_cfg_t *cfg)
{

	if (MV_DP_IS_ONLINE(mv_dp_instance.status)) {
		MV_DP_LOG_ERR("DPAPI  Already INITIALIZED", MV_DP_RC_ERR_ALREADY_ONLINE);
		return MV_DP_RC_ERR_ALREADY_ONLINE;
	}

	MV_DP_LOG_DBG1("DPAPI GOING ONLINE...\n");

	/*init event handler*/
	mv_dp_event_info_init(mv_dp_instance.event_info, cfg->notify_cb);

	/*check and set channel RX buffer size*/
	mv_dp_ch_data_size_set(mv_dp_instance.ch_data, cfg->requests_num);

	return MV_DP_RC_OK;
}


enum mv_dp_rc mv_dp_offline_set(void)
{


	if (!MV_DP_IS_ONLINE(mv_dp_instance.status)) {
		MV_DP_LOG_ERR("Already SHUTDOWN", MV_DP_RC_SHUTDOWN);
		return MV_DP_RC_SHUTDOWN;
	}

	/*allow only internal*/
	MV_DP_MSG_RXTX_SET(mv_dp_instance, MV_DP_MSG_TYPE_INTERNAL);
	/*set OFFLINE in CB*/
	mdelay(MV_DP_MSG_COMPLETE_TIME);

	return MV_DP_RC_OK;
}


static enum mv_dp_rc mv_dp_ch_data_size_set(struct mv_dp_ch_data *channels[], u16 size)
{

	int i;

	for (i = 0; i < MV_DP_MAX_CHAN_NUM; i++) {
		if (channels[i]) {
			channels[i]->max_rx_size = size;
			MV_DP_LOG_DBG1("CH: %d RX Size set to: %d", i, size);
		}
	}

	return MV_DP_RC_OK;
}


static enum mv_dp_rc mv_dp_init_parse_msg(void *out_ptr, void *buf)
{
/*no output data*/

	u32 *msg_buf = (u32 *)buf;
	u32 ver;

	MV_DP_LOG_DBG3("ENTER: %s msg:%p, out_ptr:%p\n", __func__, buf, out_ptr);

	if (!msg_buf) {
		MV_DP_LOG_ERR("Null msg_buf:%p\n", MV_DP_RC_ERR_NULL_PTR, msg_buf);
		return MV_NSS_DP_FAILED;
	}

	ver = be32_to_cpu(msg_buf[MV_DP_MSG_INIT_VER_MAJ_WORD0]);
	/*MV_DP_LOG_INF("FW DPAPI Version: ................%d.%d.%d\n",
		      MV_DP_MSG_VER_MAJ_GET(ver), MV_DP_MSG_VER_MID_GET(ver), MV_DP_MSG_VER_MIN_GET(ver));*/
	MV_DP_LOG_INF("DPAPI    Version: ................%d.%d.%d\n",
		      MV_DP_API_MAJ_VER, MV_DP_API_MIN_VER, MV_DP_API_LOC_VER);


	MV_DP_MSG_ALLOW_MSG(mv_dp_instance);
	MV_DP_MSG_ALLOW_EVENT(mv_dp_instance);
	MV_DP_ONLINE_SET(mv_dp_instance.status);

	MV_DP_LOG_DBG1("DPAPI ONLINE\n");

	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}




static enum mv_dp_rc mv_dp_shutdown_parse_msg(void *msg, void  *rx_buf)
{
#if 0
	/*pend_msg -- rx_buff pointer, ptr_buf -- recieved message*/
	struct mv_dp_msg_info_rx *rx_msg = (struct mv_dp_msg_info_rx *)msg;
	struct mv_dp_msg_buf	*msg_rx_buf = (struct mv_dp_msg_buf *) rx_buf;

#endif
	int i;
	struct mv_dp_ch_data *ch;

	MV_DP_LOG_DBG3("ENTER: %s dest:%p, src:%p\n", __func__, msg, rx_buf);

	MV_DP_OFFLINE_SET(mv_dp_instance.status);

	for (i = 0; i < MV_DP_MAX_CHAN_NUM; i++) {
		ch = mv_dp_instance.ch_data[i];
		if (ch)
			mv_dp_ch_offline_set(ch);
	}

	MV_DP_LOG_DBG1("DPAPI OFFLINE\n");

	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}

static enum mv_dp_rc mv_dp_ch_offline_set(struct mv_dp_ch_data *ch)
{
	unsigned long iflags = 0;

	MV_DP_LOCK(&ch->lock, iflags);
	/*release all buffer*/
	mv_dp_rx_buffer_release(ch);
	ch->sn = 0;
	mv_dp_clear_ch_counters(ch);
	/*debug verify rx_buff is 0*/
	MV_DP_UNLOCK(&ch->lock, iflags);

	return MV_DP_RC_OK;
}

