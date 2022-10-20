/* Copyright (c) 2018 Marvell.
 * SPDX-License-Identifier: GPL-2.0-only
 */

#ifndef _MV_DP_QOS_H_
#define _MV_DP_QOS_H_

#include "mv_dp_defs.h"
#include "mv_dp_types.h"
#include "mv_nss_dp.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MV_DP_Q_STATUS_POPULATED_FLAG		(0x01)
#define MV_DP_Q_STATUS_IN_HW_FLAG		(0x03)

#define MV_DP_Q_STATUS_IS_EMPTY(v)		(!((v) & MV_DP_Q_STATUS_POPULATED_FLAG))
#define MV_DP_Q_STATUS_POPULATED_SET(v)		((v) |= MV_DP_Q_STATUS_POPULATED_FLAG)
#define MV_DP_Q_STATUS_POPULATED_RESET(v)	((v) &= ~MV_DP_Q_STATUS_POPULATED_FLAG)

#define MV_DP_Q_STATUS_IS_IN_HW(v)		((v) & MV_DP_Q_STATUS_IN_HW_FLAG)
#define MV_DP_Q_STATUS_IN_HW_SET(v)		((v) |= MV_DP_Q_STATUS_IN_HW_FLAG)
#define MV_DP_Q_STATUS_IN_HW_RESET(v)		((v) &= ~MV_DP_Q_STATUS_IN_HW_FLAG)

#define MV_DP_Q_ID_INGR_IS_OK(v)		((v) >= 0 && (v) <= MV_NSS_DP_PRIO_NUM)
#define MV_DP_Q_ID_EGR_IS_OK(v)			((v) >= 0 && (v) <= MV_NSS_DP_PRIO_NUM)



struct mv_dp_ingr_q_entry {
	int				qid;
	uint				status;
	mv_nss_dp_ingress_queue_cfg_t	dp_queue;
	/*could be extended to hold swq*/
};

struct mv_dp_egr_q_entry {
	int				qid;
	uint				status;
	mv_nss_dp_egress_queue_cfg_t	dp_queue;
	/*could be extended to hold swq*/
};


struct mv_dp_ingr_q_db {
	struct mv_dp_ingr_q_entry	db[MV_NSS_DP_PRIO_NUM];
};


struct mv_dp_egr_q_db {
	struct mv_dp_egr_q_entry	db[MV_NSS_DP_PRIO_NUM];
};

inline struct mv_dp_ingr_q_entry *mv_dp_qos_ingr_get_q_entry(int id);
inline struct mv_dp_egr_q_entry *mv_dp_qos_egr_get_q_entry(int id);

enum mv_dp_rc	mv_dp_qos_init(void);
enum mv_dp_rc	mv_dp_qos_release(void);


void		mv_dp_qos_show_ingr_entry(int ind, uint flags);
enum mv_dp_rc	mv_dp_qos_set_ingr_entry(const mv_nss_dp_ingress_queue_cfg_t *entry);
enum mv_dp_rc	mv_dp_qos_get_ingr_entry(int id, mv_nss_dp_ingress_queue_cfg_t *entry);

void		mv_dp_qos_show_egr_entry(int ind, uint flags);
enum mv_dp_rc	mv_dp_qos_set_egr_entry(const mv_nss_dp_egress_queue_cfg_t *entry);
enum mv_dp_rc	mv_dp_qos_get_egr_entry(int id, mv_nss_dp_egress_queue_cfg_t *entry);

enum mv_dp_rc	mv_dp_qos_ingr_db_clear(void);
enum mv_dp_rc	mv_dp_qos_egr_db_clear(void);

#ifdef __cplusplus
}
#endif


#endif
