
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

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>

#include "mv_dp_int_if.h"
#include "mv_dp_main.h"
#include "mv_dp_qos.h"


static struct mv_dp_ingr_q_db *ingr_q_db;
static struct mv_dp_egr_q_db *egr_q_db;


static enum mv_dp_rc	mv_dp_qos_clear_ingr_entry(struct mv_dp_ingr_q_entry *entry);
static enum mv_dp_rc	mv_dp_qos_clear_egr_entry(struct mv_dp_egr_q_entry *entry);

static void mv_dp_qos_show_single_egr_entry(struct mv_dp_egr_q_entry *entry);
static void mv_dp_qos_show_single_ingr_entry(struct mv_dp_ingr_q_entry *entry);

static enum mv_dp_rc	mv_dp_qos_ingr_db_alloc(void);
static enum mv_dp_rc	mv_dp_qos_ingr_db_release(void);
static enum mv_dp_rc	mv_dp_qos_egr_db_alloc(void);
static enum mv_dp_rc	mv_dp_qos_egr_db_release(void);


inline struct mv_dp_ingr_q_entry *mv_dp_qos_ingr_get_q_entry(int q_id)
{

	return ingr_q_db ? &(ingr_q_db->db[q_id]) : 0;

}

inline struct mv_dp_egr_q_entry *mv_dp_qos_egr_get_q_entry(int q_id)
{
	return egr_q_db ? &(egr_q_db->db[q_id]) : 0;
}


enum mv_dp_rc	mv_dp_qos_init(void)
{

	enum mv_dp_rc rc;

	if (ingr_q_db || egr_q_db) {
		MV_DP_LOG_ERR("Ingress(%p) or Egress(%p) Q DB already set\n",
			      MV_DP_RC_ERR_ALREADY_INITILIZED, ingr_q_db, egr_q_db);
		return MV_DP_RC_ERR_ALREADY_INITILIZED;
	}

	rc = mv_dp_qos_ingr_db_alloc();
	if (MV_DP_RC_OK != rc)
		goto err;

	rc = mv_dp_qos_egr_db_alloc();
	if (MV_DP_RC_OK != rc)
		goto err;

	return MV_DP_RC_OK;

err:
	MV_DP_LOG_ERR("Q DB Init Failed\n", rc);


	kfree(ingr_q_db);
	kfree(egr_q_db);
	return rc;
}


enum mv_dp_rc	mv_dp_qos_release(void)
{

	mv_dp_qos_ingr_db_release();
	mv_dp_qos_egr_db_release();

	ingr_q_db = NULL;
	egr_q_db = NULL;

	return MV_DP_RC_OK;
}


static enum mv_dp_rc	mv_dp_qos_ingr_db_alloc(void)
{

	ingr_q_db = kzalloc(sizeof(struct mv_dp_ingr_q_db), GFP_KERNEL);
	if (!ingr_q_db) {
		MV_DP_LOG_ERR("Ingress Q DB Alloc failed\n", MV_DP_RC_ERR_ALLOC);
		return MV_DP_RC_ERR_ALLOC;
	}

	return MV_DP_RC_OK;
}

static enum mv_dp_rc	mv_dp_qos_clear_ingr_entry(struct mv_dp_ingr_q_entry *entry)
{
#ifdef MV_DP_DEBUG
	if (!entry) {
		MV_DP_LOG_ERR("Ingress Q Entry NULL ptr\n", MV_DP_RC_ERR_NULL_PTR);
		return MV_DP_RC_ERR_NULL_PTR;
	}
#endif

	memset(entry, 0, sizeof(struct mv_dp_ingr_q_entry));
	MV_DP_Q_STATUS_POPULATED_RESET(entry->status);

	return MV_DP_RC_OK;
}


static enum mv_dp_rc	mv_dp_qos_clear_egr_entry(struct mv_dp_egr_q_entry *entry)
{
#ifdef MV_DP_DEBUG
	if (!entry) {
		MV_DP_LOG_ERR("Ingress Q Entry NULL ptr\n", MV_DP_RC_ERR_NULL_PTR);
		return MV_DP_RC_ERR_NULL_PTR;
	}
#endif

	memset(entry, 0, sizeof(struct mv_dp_egr_q_entry));
	MV_DP_Q_STATUS_POPULATED_RESET(entry->status);

	return MV_DP_RC_OK;

}


enum mv_dp_rc	mv_dp_qos_ingr_db_clear(void)
{
	int i;
#ifdef MV_DP_DEBUG
	if (!ingr_q_db) {
		MV_DP_LOG_ERR("Ingress Q DB not allocated\n", MV_DP_RC_ERR_NULL_PTR);
		return MV_DP_RC_ERR_NULL_PTR;
	}
#endif

	for (i = 0; i < MV_NSS_DP_PRIO_NUM; i++)
		mv_dp_qos_clear_ingr_entry(mv_dp_qos_ingr_get_q_entry(i));

	return MV_DP_RC_OK;

}


static enum mv_dp_rc	mv_dp_qos_ingr_db_release(void)
{


	kfree(ingr_q_db);

	ingr_q_db = NULL;


	return MV_DP_RC_OK;

}


enum mv_dp_rc	mv_dp_qos_set_ingr_entry(const mv_nss_dp_ingress_queue_cfg_t *entry)
{
#ifdef MV_DP_DEBUG
	if (!ingr_q_db) {
		MV_DP_LOG_ERR("Ingress Q DB not allocated\n", MV_DP_RC_ERR_NULL_PTR);
		return MV_DP_RC_ERR_NULL_PTR;
	}

	if (!entry) {
		MV_DP_LOG_ERR("Null CFG Ingress Queue ptr\n", MV_DP_RC_ERR_NULL_PTR);
		return MV_DP_RC_ERR_NULL_PTR;
	}
#endif

	if (!MV_DP_Q_ID_INGR_IS_OK(entry->queue)) {
		MV_DP_LOG_ERR("Ingress Q DB ID:%d out of range\n", MV_DP_RC_ERR_INVALID_PARAM, entry->queue);
		return MV_DP_RC_ERR_INVALID_PARAM;
	}


	memcpy(&(ingr_q_db->db[entry->queue].dp_queue), entry, sizeof(mv_nss_dp_ingress_queue_cfg_t));
	ingr_q_db->db[entry->queue].qid = entry->queue;
	MV_DP_Q_STATUS_POPULATED_SET(ingr_q_db->db[entry->queue].status);
	MV_DP_Q_STATUS_IN_HW_RESET(ingr_q_db->db[entry->queue].status);

	return MV_DP_RC_OK;
}



static enum mv_dp_rc	mv_dp_qos_egr_db_alloc(void)
{

	egr_q_db = kzalloc(sizeof(struct mv_dp_egr_q_db), GFP_KERNEL);
	if (!egr_q_db) {
		MV_DP_LOG_ERR("Egress Q DB Alloc failed\n", MV_DP_RC_ERR_ALLOC);
		return MV_DP_RC_ERR_ALLOC;
	}

	return MV_DP_RC_OK;
}


enum mv_dp_rc	mv_dp_qos_egr_db_clear(void)
{
	int i;

#ifdef MV_DP_DEBUG
	if (!egr_q_db) {
		MV_DP_LOG_ERR("egress Q DB not allocated\n", MV_DP_RC_ERR_NULL_PTR);
		return MV_DP_RC_ERR_NULL_PTR;
	}
#endif
	for (i = 0; i < MV_NSS_DP_PRIO_NUM; i++)
		mv_dp_qos_clear_egr_entry(mv_dp_qos_egr_get_q_entry(i));

	return MV_DP_RC_OK;

}

static enum mv_dp_rc	mv_dp_qos_egr_db_release(void)
{
	kfree(egr_q_db);

	egr_q_db = NULL;

	return MV_DP_RC_OK;

}

enum mv_dp_rc	mv_dp_qos_set_egr_entry(const mv_nss_dp_egress_queue_cfg_t *entry)
{
#ifdef MV_DP_DEBUG
	if (!egr_q_db) {
		MV_DP_LOG_ERR("Egress Q DB not allocated\n", MV_DP_RC_ERR_NULL_PTR);
		return MV_DP_RC_ERR_NULL_PTR;
	}

	if (!entry) {
		MV_DP_LOG_ERR("Null CFG Egress Queue ptr\n", MV_DP_RC_ERR_NULL_PTR);
		return MV_DP_RC_ERR_NULL_PTR;
	}
#endif

	if (!MV_DP_Q_ID_EGR_IS_OK(entry->queue)) {
		MV_DP_LOG_ERR("Egress Q DB ID:%d out of range\n", MV_DP_RC_ERR_INVALID_PARAM, entry->queue);
		return MV_DP_RC_ERR_INVALID_PARAM;
	}


	memcpy(&(egr_q_db->db[entry->queue].dp_queue), entry, sizeof(mv_nss_dp_egress_queue_cfg_t));
	egr_q_db->db[entry->queue].qid = entry->queue;
	MV_DP_Q_STATUS_POPULATED_SET(egr_q_db->db[entry->queue].status);
	MV_DP_Q_STATUS_IN_HW_RESET(egr_q_db->db[entry->queue].status);

	return MV_DP_RC_OK;
}

enum mv_dp_rc	mv_dp_qos_get_egr_entry(int id, mv_nss_dp_egress_queue_cfg_t *entry)
{
#ifdef MV_DP_DEBUG
	if (!egr_q_db) {
		MV_DP_LOG_ERR("Egress Q DB not allocated\n", MV_DP_RC_ERR_NULL_PTR);
		return MV_DP_RC_ERR_NULL_PTR;
	}

	if (!entry) {
		MV_DP_LOG_ERR("Null CFG Egress Queue ptr\n", MV_DP_RC_ERR_NULL_PTR);
		return MV_DP_RC_ERR_NULL_PTR;
	}
#endif

	if (!MV_DP_Q_ID_EGR_IS_OK(id)) {
		MV_DP_LOG_ERR("Egress Q DB ID:%d out of range\n", MV_DP_RC_ERR_INVALID_PARAM, id);
		return MV_DP_RC_ERR_INVALID_PARAM;
	}

	if (MV_DP_Q_STATUS_IS_EMPTY(egr_q_db->db[id].status)) {
		MV_DP_LOG_ERR("Egress Q ID:%d not set\n", MV_DP_RC_ERR_INVALID_PARAM, id);
		return MV_DP_RC_ERR_INVALID_PARAM;
	}

	memcpy(entry, &(egr_q_db->db[id].dp_queue), sizeof(mv_nss_dp_egress_queue_cfg_t));

	return MV_DP_RC_OK;
}


enum mv_dp_rc	mv_dp_qos_get_ingr_entry(int id, mv_nss_dp_ingress_queue_cfg_t *entry)
{
#ifdef MV_DP_DEBUG
	if (!ingr_q_db) {
		MV_DP_LOG_ERR("Ingress Q DB not allocated\n", MV_DP_RC_ERR_NULL_PTR);
		return MV_DP_RC_ERR_NULL_PTR;
	}

	if (!entry) {
		MV_DP_LOG_ERR("Null CFG Ingress Queue ptr\n", MV_DP_RC_ERR_NULL_PTR);
		return MV_DP_RC_ERR_NULL_PTR;
	}
#endif

	if (!MV_DP_Q_ID_INGR_IS_OK(id)) {
		MV_DP_LOG_ERR("Ingress Q DB ID:%d out of range\n", MV_DP_RC_ERR_INVALID_PARAM, id);
		return MV_DP_RC_ERR_INVALID_PARAM;
	}

	if (MV_DP_Q_STATUS_IS_EMPTY(ingr_q_db->db[id].status)) {
		MV_DP_LOG_ERR("Ingress Q ID:%d not set\n", MV_DP_RC_ERR_INVALID_PARAM, id);
		return MV_DP_RC_ERR_INVALID_PARAM;
	}

	memcpy(entry, &(ingr_q_db->db[id].dp_queue), sizeof(mv_nss_dp_egress_queue_cfg_t));

	return MV_DP_RC_OK;
}


static void mv_dp_qos_show_single_egr_entry(struct mv_dp_egr_q_entry *entry)
{

	if (!entry) {
		MV_DP_LOG_ERR("Null Egress Q\n", MV_DP_RC_ERR_NULL_PTR);
		return;
	}


	MV_DP_LOG_CONT("|%4u|%3s%3s|%6u|%6u-%6u|%6u-%6u|<\n",
		       entry->qid,
		       (MV_DP_Q_STATUS_IS_EMPTY(entry->status)) ? (" E ") : ("   "),
		       (MV_DP_Q_STATUS_IS_IN_HW(entry->status)) ? (" H ") : ("   "),
		       entry->dp_queue.queue,
		       entry->dp_queue.policer.cir,
		       entry->dp_queue.policer.eir,
		       entry->dp_queue.shaper.cir,
		       entry->dp_queue.shaper.eir);

}



static void mv_dp_qos_show_single_ingr_entry(struct mv_dp_ingr_q_entry *entry)
{

	if (!entry) {
		MV_DP_LOG_ERR("Null Ingress Q\n", MV_DP_RC_ERR_NULL_PTR);
		return;
	}



	MV_DP_LOG_CONT("|%4u|%3s%3s|%6u|%8u|%6u-%6u|<\n",
		       entry->qid,
		       (MV_DP_Q_STATUS_IS_EMPTY(entry->status)) ? (" E ") : ("   "),
		       (MV_DP_Q_STATUS_IS_IN_HW(entry->status)) ? (" H ") : ("   "),
		       entry->dp_queue.queue,
		       entry->dp_queue.red_thresh,
		       entry->dp_queue.policer.cir,
		       entry->dp_queue.policer.eir);
}


void mv_dp_qos_show_ingr_entry(int ind, uint flags)
{

	if (flags & MV_DP_CLI_HDR)
		MV_DP_LOG_CONT("\n| ID |STATUS|QUEUE|SCHED| VALUE|LENGTH|   POLICER  |    SHAPER  |<\n");

	if (!MV_DP_Q_ID_INGR_IS_OK(ind)) {
		MV_DP_LOG_ERR("Ingress Q DB ID:%d out of range\n", MV_DP_RC_ERR_INVALID_PARAM, ind);
		return;
	}

	if (flags & MV_DP_CLI_DATA)
		mv_dp_qos_show_single_ingr_entry(mv_dp_qos_ingr_get_q_entry(ind));
}


void mv_dp_qos_show_egr_entry(int ind, uint flags)
{
	if (flags & MV_DP_CLI_HDR)
		MV_DP_LOG_CONT("\n| ID |STATUS|QUEUE|SCHED| VALUE|LENGTH|RED TRSH|   POLICER  |<\n");

	if (!MV_DP_Q_ID_EGR_IS_OK(ind)) {
		MV_DP_LOG_ERR("Ingress Q DB ID:%d out of range\n", MV_DP_RC_ERR_INVALID_PARAM, ind);
		return;
	}

	if (flags & MV_DP_CLI_DATA)
		mv_dp_qos_show_single_egr_entry(mv_dp_qos_egr_get_q_entry(ind));
}
