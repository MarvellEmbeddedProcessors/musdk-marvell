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

#ifndef _MV_DP_DEFS_H_
#define _MV_DP_DEFS_H_

#ifdef __cplusplus
extern "C" {
#endif


#define MV_DP_USE_LIST_BUFFER	/*linux list is used, if not circular buffer*/

#define MV_DP_API_STR_HELPER(x)	#x
#define MV_DP_API_STR(x)	MV_DP_API_STR_HELPER(x)


#define MV_DP_API_MAJ_VER	18
#define MV_DP_API_MIN_VER	7
#define MV_DP_API_LOC_VER	0
#define MV_DP_API_VERSION	((MV_DP_API_MAJ_VER<<16) | (MV_DP_API_MIN_VER<<8) | MV_DP_API_LOC_VER)
#define MV_DP_API_VERSION_STR	MV_DP_API_STR(MV_DP_API_MAJ_VER)"."\
				MV_DP_API_STR(MV_DP_API_MIN_VER)"."\
				MV_DP_API_STR(MV_DP_API_LOC_VER)




#define MV_DP_DBG_FATAL		(0x1)
#define MV_DP_DBG_CRITICAL	(0x2)
#define MV_DP_DBG_INFO		(0x3)
#define MV_DP_DBG_ERROR		(0x4)
#define MV_DP_DBG_DBG1		(0x5)
#define MV_DP_DBG_DBG2		(0x6)
#define MV_DP_DBG_DBG3		(0x7)
#define MV_DP_DBG_LAST		(0x8)

#define MV_DP_DBG_LVL_IS_OK(i)	((i) >= 0 && (i) < MV_DP_DBG_LAST)


/*channel mode*/
#define MV_DP_CH_MODE_SHARED	(0x01)
#define MV_DP_IS_SHARED()	((mv_dp_instance.ch_mode) & MV_DP_CH_MODE_SHARED)


#define MV_DP_DBG_LVL		(0x04)

#define MV_DP_RX_BUFF_SIZE	(100)
#define MV_DP_RX_BUFF_MAX	(1024)
#define MV_DP_RX_BUFF_IS_OK(l)	((l) > 0 && (l) < MV_DP_RX_BUFF_MAX)

#define MV_DP_CH_QUEUE_SIZE	(512)
#define MV_DP_MSG_SN_MASK	(0x7FF)

#define MV_DP_RX_AGE_TIMEOUT	(1000) /*timeout for pending buffer aging; 0 - disabled*/
#define MV_DP_RX_AGE_COUNT	(2)   /*number of timeout intervals before aging*/

#define MV_DP_MSG_COMPLETE_TIME	(100) /*time in ms to release all already taken references*/

/*status flags*/
#define MV_DP_ST_INITIALISED		(0x01) /*module is instanciated*/
#define MV_DP_IS_INITIALISED(l)		((l) & MV_DP_ST_INITIALISED)
#define MV_DP_INITIALISED_SET(l)	((l) |= MV_DP_ST_INITIALISED)
#define MV_DP_INITIALISED_RESET(l)	((l) &= ~MV_DP_ST_INITIALISED)

#define MV_DP_ST_ONLINE			(0x04) /*module is online -- channels are created and FW version is checked*/
#define MV_DP_IS_ONLINE(l)		((l) & MV_DP_ST_ONLINE)
#define MV_DP_ONLINE_SET(l)		((l) |= MV_DP_ST_ONLINE)
#define MV_DP_OFFLINE_SET(l)		((l) &= ~MV_DP_ST_ONLINE)


/*Allowed messages flags*/
#define MV_DP_MSG_TYPE_FIRST		(0x01)
#define MV_DP_MSG_TYPE_EVENT		(0x01)
#define MV_DP_MSG_TYPE_INTERNAL		(0x02)
#define MV_DP_MSG_TYPE_MSG		(0x04)
#define MV_DP_MSG_TYPE_RLS_BUF		(0x08)
#define MV_DP_MSG_TYPE_LAST		((MV_DP_MSG_TYPE_EVENT|MV_DP_MSG_TYPE_INTERNAL|MV_DP_MSG_TYPE_MSG))

#define MV_DP_MSG_TYPE_IS_OK(t)		((t) >= MV_DP_MSG_TYPE_FIRST && (t) <= MV_DP_MSG_TYPE_LAST)

#define MV_DP_MSG_SIZE_INVALID		(-1)
#define MV_DP_MSG_RX_SIZE_CHECK(v, op)	((v).msg_info[(m)].rx_size != MV_DP_MSG_SIZE_INVALID)


#define MV_DP_MSG_IS_ALLOWED(v, m)	((v).allowed_mask & (v).msg_info[(m)].type)
#define MV_DP_MSG_RXTX_DISABLE(v)	((v).allowed_mask = 0)
#define MV_DP_MSG_RXTX_SET(v, f)	((v).allowed_mask = (f))
#define MV_DP_MSG_ALLOW_EVENT(v)	((v).allowed_mask |= (MV_DP_MSG_TYPE_EVENT))
#define MV_DP_MSG_ALLOW_INTERNAL(v)	((v).allowed_mask |= (MV_DP_MSG_TYPE_INTERNAL))
#define MV_DP_MSG_ALLOW_MSG(v)		((v).allowed_mask |= (MV_DP_MSG_TYPE_MSG))

#define MV_DP_MSG_IS_EVENT(v)		((v) & MV_DP_MSG_TYPE_EVENT)
#define MV_DP_MSG_IS_MSG(v)		((v) & MV_DP_MSG_TYPE_MSG)
#define MV_DP_MSG_IS_INTERNAL(v)	((v) & MV_DP_MSG_TYPE_INTERNAL)

#define MV_DP_MSG_RLS_BUF(v)		((v) & MV_DP_MSG_TYPE_RLS_BUF)

#define MV_DP_MSG_GET_HANDLE(v, m)	((v).msg_info[(m)].rx_handle)
#define MV_DP_EVENT_GET_PARAM(v, m)	((v).event_info[(m)].res)

#define MV_DP_FW_ERR_IS_OK(v)		((v) >= 0 && (v) <= MV_DP_RC_ERR_DPAPI3)
#define MV_DP_ERR_NUM_IS_OK(n)		((n) >= MV_DP_RC_OK && (n) < MV_DP_RC_LAST)

/*trace message TX/RX flags*/
#define MV_DP_LOG_TRC_NONE		(0x00)
#define MV_DP_LOG_TRC_RX		(0x01)
#define MV_DP_LOG_TRC_TX		(0x02)
#define MV_DP_LOG_TRC_CB		(0x04)
#define MV_DP_LOG_TRC_BUF		(0x08)
#define MV_DP_LOG_TRC_EXT		(0x10)
#define MV_DP_LOG_TRC_LAST		(0x20)

#define MV_DP_LOG_TRC_EXT_BYTES		(176)

#define MV_DP_LOG_TRC_IS_OK(m)		((m) >= MV_DP_LOG_TRC_NONE && (m) < MV_DP_LOG_TRC_LAST)
#define MV_DP_LOG_TRC_ON(o, m)		(mv_dp_msg_trace_type(o) & (m))

#define MV_DP_CPU_TO_CH(c)		(mv_dp_instance.cpu_to_ch_id[c])


#define MV_DP_LOG_CRIT(fmt, ...)	(pr_crit("DP ERR>" fmt, ##__VA_ARGS__))
#define MV_DP_LOG_INF(fmt, ...)		(pr_info("DP INFO>" fmt, ##__VA_ARGS__))
/*
#define MV_DP_LOG_DBG(lvl,fmt,...)	pr_debug("DP DBG%d>" fmt, (lvl), ##__VA_ARGS__)
*/
/*(mv_dp_instance.dbg_lvl))\*/
#define MV_DP_LOG_DBG_L(lvl, fmt, ...)	do { \
					if ((lvl) <= mv_dp_dbg_lvl())\
						pr_debug("DP DBG%d>" fmt, \
						(lvl + 1 - MV_DP_DBG_DBG1), ##__VA_ARGS__);\
					} while (0)

#ifdef MV_DP_DEBUG

/*mv_dp_instance.errors[(lvl)]++;\*/
#define MV_DP_LOG_ERR(fmt, lvl, ...)     do {\
						mv_dp_inc_error(lvl);\
						pr_err("DP ERR>%d>%s:%d>" fmt,\
						(lvl), __func__, __LINE__, ##__VA_ARGS__);\
					} while (0)
/*
#define MV_DP_LOG_ERR(fmt,lvl,...)	(pr_err("DP ERR>%d>%s:%d>" fmt, (lvl), __BASE_FILE__ , __LINE__, ##__VA_ARGS__))
*/

#define MV_DP_LOG_DBG3(fmt, ...)	MV_DP_LOG_DBG_L(MV_DP_DBG_DBG3, fmt, ##__VA_ARGS__)
#define MV_DP_LOG_DBG2(fmt, ...)	MV_DP_LOG_DBG_L(MV_DP_DBG_DBG2, fmt, ##__VA_ARGS__)
#define MV_DP_LOG_DBG1(fmt, ...)	MV_DP_LOG_DBG_L(MV_DP_DBG_DBG1, fmt, ##__VA_ARGS__)


#else
#define MV_DP_LOG_ERR(fmt, lvl, ...)	do {\
						mv_dp_inc_error(lvl);\
						pr_err("DP ERR>%d>" fmt, (lvl), ##__VA_ARGS__);\
					} while (0)
/*
#define MV_DP_LOG_ERR(fmt,lvl,...)	(pr_err("DP ERR>%d>" fmt, (lvl), ##__VA_ARGS__))
*/
#define MV_DP_LOG_DBG3(fmt, ...)
#define MV_DP_LOG_DBG2(fmt, ...)
#define MV_DP_LOG_DBG1(fmt, ...)

#endif


#define MV_DP_LOG_MSG(fmt, ...)		(pr_info("DP TRC>" fmt, ##__VA_ARGS__))
#define MV_DP_LOG_CONT(fmt, ...)	(pr_cont(fmt, ##__VA_ARGS__))
#define MV_DP_LOG_BYTE_DUMP_LINE	(16)

#define MV_DP_LOG_EVT(fmt, evt, ...)		\
			(pr_info("DP EVENT>S:%d:X:0x%X:K:%p:T:%d:C:%d>" fmt, ((evt)->status), ((evt)->xid), \
			((evt)->cookie), ((evt)->type), ((evt)->count), ##__VA_ARGS__))



#define MV_DP_CHECK_RX_SIZE(t)		(((t)->rx_size < (t)->max_rx_size))
#define MV_DP_INC_RX_SIZE(t)		(((t)->rx_size++))
#define MV_DP_DEC_RX_SIZE(t)		(((t)->rx_size--))


#define MV_DP_INC_WRAP(v, l)		((v) = ((v) + 1) & (l))
#define MV_DP_MSG_SN_INC(v)		(MV_DP_INC_WRAP((v), MV_DP_MSG_SN_MASK))

/*general print format flags*/
#define MV_DP_CLI_DATA			(0x1)
#define MV_DP_CLI_HDR			(0x2)
#define MV_DP_CLI_TRAILER		(0x4)

#define MV_DP_MSG_COUNT			(4*1024) /*maximum number of entities in a signle API call - count param*/
#define MV_DP_MSG_COUNT_VALID(v)	((v) > 0 && (v) <= MV_DP_MSG_COUNT)

#define MV_DP_MSG_HDR_SIZE		(4)

/*moved here from mv_nss_dp.h*/
#define MV_NSS_DP_ETH_PORT_MIN          (0)

/* Maximal Ethernet virtual port ID */
#define MV_NSS_DP_ETH_PORT_MAX          (7)

/* Maximal Ethernet virtual port ID */
#define MV_NSS_DP_CPU_PORT_MAX          (MV_NSS_DP_CPU_PORT_MIN + nr_cpu_ids  - 1)

#define MV_NSS_DP_EXCEPT_LAST		(MV_NSS_DP_EXCEPT_OUTER_DTLS_PADDING + 1)

#define MV_NSS_DP_EGR_ACTIVE_PRIO_NUM   (CONFIG_MV_PP3_NSS_TXQ_NUM)


#ifdef __cplusplus
}
#endif


#endif

