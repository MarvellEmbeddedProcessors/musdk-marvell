/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _LF_MNG_H
#define _LF_MNG_H

#include "mv_std.h"
#include "env/mv_sys_event.h"
#include "drivers/mv_giu.h"
#include "mng/mv_nmp.h"
#include "mng/mv_nmp_dispatch.h"

struct lf_mng;

struct lf_mng_params {
	struct nmp	*nmp;
	struct nmdisp	*nmdisp;
	struct mqa	*mqa;
	struct giu	*giu;

	u8				 num_containers;
	struct nmp_container_params	*containers_params;
};

int lf_mng_init(struct lf_mng_params *params, struct lf_mng **lf_mng);
void lf_mng_deinit(struct lf_mng *lf_mng);

int lf_mng_run_maintenance(struct lf_mng *lf_mng);

/*
 * Structure containing all the LF related data
 */
struct nmlf {
	int id;
	int (*f_maintenance_cb)(struct nmlf *nmlf);
	int (*f_serialize_cb)(struct nmlf *nmlf, char *buff, u32 size);
};

/* TODO: temporary routine until this will be done through guest message passing! */
int lf_mng_create_scheduling_event(struct lf_mng *lf_mng,
	struct nmp_event_params *params,
	struct mv_sys_event **ev);
int lf_mng_delete_scheduling_event(struct mv_sys_event *ev);
int lf_mng_set_scheduling_event(struct mv_sys_event *ev, int en);

#endif /* _LF_MNG_H */
