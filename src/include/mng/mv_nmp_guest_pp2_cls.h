/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _MV_NMP_GUEST_PP2_CLS_H
#define _MV_NMP_GUEST_PP2_CLS_H

#include "drivers/mv_pp2_cls.h"

/** @addtogroup grp_nmp_guest_pp2_cls NMP Guest: PP2-CLS
 *
 *  Networking Management Proxy (NMP) Guest PP2-CLS API documentation
 *
 *  @{
 */

/* nmp_guest handler declaration */
struct nmp_guest;

/**
 * For explanation of all pp2_cls_xxx definitions, please refer to mv_pp2_cls.h.
 *
 * NOTE: Guest in AGNIC architecture MUST not use 'cos' and/or 'plcr' parameters
 *	 which are part of 'pp2_cls_tbl_action'
 */

/**
 * Create a classifier table object.
 *
 * @param[in]	params	A pointer to the classifier table parameters
 * @param[out]	tbl	A pointer to an allocated classifier table
 *
 * @retval		0 on success
 * @retval		error-code otherwise
 */
int nmp_guest_pp2_cls_tbl_init(struct nmp_guest *g,
			       struct pp2_cls_tbl_params *params,
			       struct pp2_cls_tbl **tbl);

/**
 * Deinit a classifier table object
 *
 * @param[in]	tbl		A pointer to a classifier table object
 */
void nmp_guest_pp2_cls_tbl_deinit(struct nmp_guest *g, struct pp2_cls_tbl *tbl);

/**
 * Add a classifier rule
 *
 * @param[in]	tbl		A pointer to a classifier table object
 * @param[in]	rule		A pointer to a classifier rule
 * @param[in]	action		A pointer to a classifier action
 *
 * @retval	0 on success
 * @retval	positive value - reserved
 * @retval	error-code otherwise (negative value)
 */
int nmp_guest_pp2_cls_tbl_add_rule(struct nmp_guest *g,
				   struct pp2_cls_tbl *tbl,
				   struct pp2_cls_tbl_rule *rule,
				   struct pp2_cls_tbl_action *action);

/**
 * Modify the action of an existing classifier rule
 *
 * The rule must be already in place
 *
 * @param[in]	tbl		A pointer to a classifier table object
 * @param[in]	rule		A pointer to a classifier rule
 * @param[in]	action		A pointer to a classifier action
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int nmp_guest_pp2_cls_tbl_modify_rule(struct nmp_guest *g,
				      struct pp2_cls_tbl *tbl,
				      struct pp2_cls_tbl_rule *rule,
				      struct pp2_cls_tbl_action *action);

/**
 * Remove a classifier rule
 *
 * @param[in]	tbl		A pointer to a classifier table object
 * @param[in]	rule		A pointer to a classifier rule
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int nmp_guest_pp2_cls_tbl_remove_rule(struct nmp_guest *g,
				      struct pp2_cls_tbl *tbl,
				      struct pp2_cls_tbl_rule *rule);

/** @} */ /* end of grp_nmp_guest_pp2_cls */

#endif /* _MV_NMP_GUEST_PP2_CLS_H */

