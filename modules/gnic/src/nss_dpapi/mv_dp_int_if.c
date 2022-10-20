/* Copyright (c) 2018 Marvell.
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>

#include "mv_dp_int_if.h"
#include "mv_dp_main.h"
#include "mv_dp_ext_hdr.h"
#include "mv_nss_dp.h"



/****************INTERNAL MESSAGES RX/TX**************************/
/*
 * mv_dp_nop_tx
 *
 * Description:
 *       build message and call msg_tx
 *
 * Parameters:
 *
 * Returns:
 *        MV_NSS_DP_OK - On success or an error code, otherwise.
 */



enum mv_dp_rc mv_dp_msg_nop_tx(u32 cookie, mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s NOP cookie:%d\n", __func__, cookie);
	rc = MV_NSS_DP_FAILED;


	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_NOP;
	tx_msg.res = res;
	tx_msg.count = 1;

	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_NOP, &cookie);
	if (MV_DP_RC_OK != rc) {
		MV_DP_LOG_ERR("NOP TX MSG Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (MV_DP_RC_OK != rc) {
		MV_DP_LOG_ERR("NOPt TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("NOP TX OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

/*redundant - example as it is impossible*/
err:
	if (tx_msg.flags & MV_DP_F_CFH_EXT_HDR)
		mv_dp_ext_hdr_be_release(msg_buf);

e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;

}


enum mv_dp_rc mv_dp_msg_ctx_tx(struct mv_dp_ctx *ctx, const mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	if (!ctx) {
		MV_DP_LOG_ERR("CTX TX null ctx\n", MV_DP_RC_ERR_INVALID_PARAM);
		return MV_DP_RC_ERR_INVALID_PARAM;
	}

	MV_DP_LOG_DBG3("ENTER: %s CTX opcode:%d cnt:%d data:%p\n", __func__, ctx->opc, ctx->cnt, ctx->data);
	rc = MV_NSS_DP_FAILED;


	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = ctx->data;
	tx_msg.opcode = MV_DP_MSGID_CTX;
	tx_msg.res = res;
	tx_msg.count = 1;

	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_CTX, ctx);
	if (MV_DP_RC_OK != rc) {
		MV_DP_LOG_ERR("CTX TX MSG Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (MV_DP_RC_OK != rc) {
		MV_DP_LOG_ERR("CTX TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("CTX TX OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s opc:%d cnt:%d\n", MV_NSS_DP_OK, __func__, ctx->opc, ctx->cnt);
	return MV_NSS_DP_OK;

/*redundant - example as it is impossible*/
err:
	if (tx_msg.flags & MV_DP_F_CFH_EXT_HDR)
		mv_dp_ext_hdr_be_release(msg_buf);

e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;

}



enum mv_dp_rc mv_dp_msg_ver_tx(mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s\n", __func__);
	rc = MV_NSS_DP_FAILED;

	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_VER;
	tx_msg.res = res;
	tx_msg.count = 1;

	/* redundant used for debug will do nothing as no input*/
	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_VER, NULL);
	if (MV_DP_RC_OK != rc) {
		MV_DP_LOG_ERR("VER TX MSG Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (MV_DP_RC_OK != rc) {
		MV_DP_LOG_ERR("VER TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("VER TX OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:
	if (tx_msg.flags & MV_DP_F_CFH_EXT_HDR)
		mv_dp_ext_hdr_be_release(msg_buf);

e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}


/*****************************NSS MEM DEBUG*******************************************/
enum mv_dp_rc mv_dp_msg_nss_mem_write_tx(struct mv_dp_dbg_nss_mem *mem, mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s\n", __func__);
	rc = MV_NSS_DP_FAILED;

	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_DBG_NSS_MEM_WRITE;
	tx_msg.res = res;
	tx_msg.count = 1;

	/* redundant used for debug will do nothing as no input*/
	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_DBG_NSS_MEM_WRITE, mem);
	if (MV_DP_RC_OK != rc) {
		MV_DP_LOG_ERR("nss mem write Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto e_ret;
	}

	tx_msg.msg_size = mem->size + (MV_DP_MSG_NSS_MEM_DUMP_WORD0 - MV_DP_MSG_NSS_MEM_OFFSET_WORD0) * 4;

	rc = mv_dp_msg_tx(&tx_msg);
	if (MV_DP_RC_OK != rc) {
		MV_DP_LOG_ERR("nss mem write TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("nss mem write  TX OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:
	if (tx_msg.flags & MV_DP_F_CFH_EXT_HDR)
		mv_dp_ext_hdr_be_release(msg_buf);

e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}

enum mv_dp_rc mv_dp_msg_nss_mem_read_tx(struct mv_dp_dbg_nss_mem *mem, mv_nss_dp_result_spec_t *res)
{
	struct mv_dp_msg_info_tx tx_msg;
	u32			msg_buf[MV_DP_CFH_MSG_BUF_SIZE_W];
	enum mv_dp_rc		rc;

	MV_DP_LOG_DBG3("ENTER: %s\n", __func__);
	rc = MV_NSS_DP_FAILED;

	/*count is checked inside builder and is set per system*/
	tx_msg.msg_data = &msg_buf;
	tx_msg.out_buf = 0;
	tx_msg.opcode = MV_DP_MSGID_DBG_NSS_MEM_READ;
	tx_msg.res = res;
	tx_msg.count = 1;

	/* redundant used for debug will do nothing as no input*/
	rc = mv_dp_msg_build(&tx_msg, MV_DP_MSGID_DBG_NSS_MEM_READ, mem);
	if (MV_DP_RC_OK != rc) {
		MV_DP_LOG_ERR("nss mem read Build, msg_size:%d, count:%d, flags:0x%04X\n",
			       MV_DP_RC_ERR_BUILD_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto e_ret;
	}

	rc = mv_dp_msg_tx(&tx_msg);
	if (MV_DP_RC_OK != rc) {
		MV_DP_LOG_ERR("nss mem read TX Failed, msg_size:%d, count:%d, flags:0x%04X\n",
			      MV_DP_RC_ERR_MSG_TX, tx_msg.msg_size, tx_msg.count, tx_msg.flags);
		goto err;
	}

	MV_DP_LOG_DBG2("nss mem write  TX OK msg_size:%d, count:%d, flags:0x%04X\n",
		       tx_msg.msg_size, tx_msg.count, tx_msg.flags);

	MV_DP_LOG_DBG3("EXIT(%d): %s\n", MV_NSS_DP_OK, __func__);
	return MV_NSS_DP_OK;

err:
	if (tx_msg.flags & MV_DP_F_CFH_EXT_HDR)
		mv_dp_ext_hdr_be_release(msg_buf);

e_ret:
	MV_DP_LOG_DBG3("EXIT(%d): %s\n", rc, __func__);
	return rc;
}




void mv_dp_nss_mem_show(const struct mv_dp_dbg_nss_mem *const mem)
{
	int i;

	MV_DP_LOG_CONT("NSS MEM Type:%d Off:0x%08X Size:%d", mem->type, mem->offset, mem->size);
	for (i = 0; i < mem->size; i++) {
		if ((i % 4) == 0)
			MV_DP_LOG_CONT(" ");
		if ((i % 16) == 0)
			MV_DP_LOG_CONT("\n");
		MV_DP_LOG_CONT("%02X", mem->arr[i]);
	}
	MV_DP_LOG_CONT("\n");

}
/***********NSS MEM DBG END*******************************************************/


/***********EVENT RX Handlers******************************/
enum mv_dp_rc mv_dp_msg_event_sys_parse(void *buf_out, void *msg_in)
{
	struct mv_dp_msg_info_rx *msg_rx = (struct mv_dp_msg_info_rx *)msg_in;
	uint32_t *code = (u32 *)buf_out;
	u32 *msg_buf = (u32 *)msg_rx->msg_data;


#ifdef MV_DP_DEBUG
	if (!msg_buf || !code) {
		MV_DP_LOG_ERR("Null Pointer event msg_buf:%p code:%p\n", MV_DP_RC_ERR_NULL_PTR, msg_buf, code);
		return MV_DP_RC_ERR_NULL_PTR;
	}
#endif

	*code = MV_DP_MSG_EVT_CODE_GET(msg_buf[MV_DP_MSG_EVT_CODE_WORD0]);


	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}


enum mv_dp_rc mv_dp_msg_event_sys_msg_parse(void *buf_out, void *msg_in)
{
	struct mv_dp_msg_info_rx *msg_rx = (struct mv_dp_msg_info_rx *)msg_in;
	u32 *out_msg = (u32 *)buf_out;
	u32 *msg_buf = (u32 *)msg_rx->msg_data;

	int msg_size = (msg_rx->msg_size > MV_DP_SYSTEM_MSG_SIZE) ? (MV_DP_SYSTEM_MSG_SIZE) : (msg_rx->msg_size);

#ifdef MV_DP_DEBUG
	if (!msg_buf || !out_msg) {
		MV_DP_LOG_ERR("Null Pointer event msg_buf:%p out_msg:%p\n", MV_DP_RC_ERR_NULL_PTR, msg_buf, out_msg);
		return MV_DP_RC_ERR_NULL_PTR;
	}
#endif

	out_msg[0] = MV_DP_MSG_EVT_CODE_GET(msg_buf[MV_DP_MSG_EVT_CODE_WORD0]);

	memcpy(&out_msg[1], &msg_buf[MV_DP_MSG_EVT_TEXT_WORD1], msg_size);

	MV_DP_LOG_DBG3("EXIT: %s\n", __func__);

	return MV_NSS_DP_OK;
}




/****************Default RX handlers**********************************/

void mv_dp_default_rx_cb(mv_nss_dp_event_t *evt)
{
	char *c = evt->params.notify_msg;
	int i = 4;

	/*on event type*/
	if (!evt) {
		MV_DP_LOG_INF("NULL Event CB Recieved\n");
		return;
	}

	switch (evt->type) {
	case MV_NSS_DP_EVT_NOTIFY_CODE:
		MV_DP_LOG_EVT("SYS EVENT CODE:0x%08X\n", evt, *(evt->params.notify_code));
		break;
	case MV_NSS_DP_EVT_NOTIFY_MSG:
		MV_DP_LOG_EVT("SYS EVENT CODE:0x%08X\n", evt, *(evt->params.notify_code));
		/*print count chars*/
		while (c[i] != '\0' && i < MV_DP_SYSTEM_MSG_SIZE) {
			MV_DP_LOG_CONT("%c", c[i]);
			i++;
		}
		MV_DP_LOG_CONT("\nevent msg total: %d bytes\n", i);
		break;

	default:
		MV_DP_LOG_INF("Unknown event type in CB:%d\n", evt->type);
		return;
	}

}

/**********UTILITIES***************************************************/

/*max_prio is the number of internal priorities*/
enum mv_dp_rc mv_dp_check_internal_prio(const u8 *prio, int size, int max_prio)
{
	int i = 0;

	for (i = 0; i < size; i++) {
		if (prio[i] >= max_prio) {
			MV_DP_LOG_ERR("INVLAID PRIO VALUE: %d max_prio:%d\n", MV_DP_RC_ERR_INVALID_PARAM,
				      prio[i], max_prio);
			return MV_DP_RC_ERR_INVALID_PARAM;
		}
	}

	return MV_DP_RC_OK;
}

enum mv_dp_rc mv_dp_check_pkt_prio(const u8 *prio, int size, u8 mask)
{
	int i = 0;

	for (i = 0; i < size; i++) {
		if (prio[i] & ~(mask)) {
			MV_DP_LOG_ERR("INVLAID PRIO VALUE: %d max_prio:0x%02X\n", MV_DP_RC_ERR_INVALID_PARAM,
				      prio[i], mask);
			return MV_DP_RC_ERR_INVALID_PARAM;
		}
	}

	return MV_DP_RC_OK;
}

