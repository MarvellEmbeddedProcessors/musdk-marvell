/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _GIE_H
#define _GIE_H

#include "mv_std.h"
#include "env/mv_sys_event.h"

struct gie;

struct gie_params {
	u64 gct_base;
	u64 gpt_base;

	/* MSI phys/virt register base */
	u64 msi_regs_phys;
	u64 msi_regs_virt;

	char *name_match;
	char *dmax_match;
};

enum gie_desc_type {
	TX_DESC = 0,
	RX_DESC,
	BUFF_DESC
};

enum gie_copy_mode_type {
	GIE_MODE_VIRT = 0,
	GIE_MODE_DMA,

	GIE_MODE_MAX
};

struct gie_event_params {
	u32 pkt_coal;
	u32 usec_coal;
	u32 tc_mask;
};


/**
 * Initialize the emulator.
 *
 * @param[in]	gie_reg		A pointer to GIE registers base.
 * @param[in]	dma_id		DMA ID.
 * @param[in]	name		A pointer to the emulator name.
 *
 * @retval	handler to the emulator.
 *
 */
int gie_init(struct gie_params *gie_pars, struct gie **gie);

/**
 * Terminate the emulator.
 *
 * @param[in]	gie		A GIE handler.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 *
 */
int gie_terminate(struct gie *gie);

/**
 * Return the GIE registers.
 *
 * @param[in]	gie		A GIE handler.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 *
 */
void *gie_regs(void *gie);

/**
 * Add Queue to GIE scheduling.
 *
 * @param[in]	gie		A GIE handler.
 * @param[in]	qid		queue ID.
 * @param[in]	is_remote	is this queue remote or local.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 *
 */
int gie_add_queue(void *gie, u16 qid, int is_remote);

/**
 * Add Buffer Management Queue to GIE scheduling.
 *
 * @param[in]	gie		A GIE handler.
 * @param[in]	qid		queue ID.
 * @param[in]	buff_size	buffer size.
 * @param[in]	is_remote	is this queue remote or local.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 *
 */
int gie_add_bm_queue(void *gie, u16 qid, int buf_size, int is_remote, void **bm_queue_ref);

/**
 * Remove Queue from GIE scheduling.
 *
 * @param[in]	gie		A GIE handler.
 * @param[in]	qid		queue ID.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 *
 */
int gie_remove_queue(void *gie, u16 qid);

/**
 * Remove Buffer Management Queue from GIE scheduling.
 *
 * @param[in]	gie		A GIE handler.
 * @param[in]	qid		queue ID.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 *
 */
int gie_remove_bm_queue(void *gie, u16 qid);

/**
 * Suspend Queue from GIE scheduling.
 *
 * @param[in]	gie		A GIE handler.
 * @param[in]	qid		queue ID.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 *
 */
int gie_suspend_queue(void *gie, u16 qid);

/**
 * Resume Queue to GIE scheduling.
 *
 * @param[in]	gie		A GIE handler.
 * @param[in]	qid		queue ID.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 *
 */
int gie_resume_queue(void *gie, u16 qid);

/**
 * Start GIE scheduling.
 *
 * @param[in]	gie		A GIE handler.
 * @param[in]	time_limit	schedule time lime (0 == infinite).
 * @param[in]	qe_limit	queue elements limit for processing.
 * @param[out]	pending		pending jobs
 *
 * @retval	0 on success
 * @retval	<0 on failure
 *
 */
int gie_schedule(void *gie, u64 time_limit, u64 qe_limit, u16 *pending);

/**
 * Return the GIE descriptor size.
 *
 * @param[in]	type		type of GIE queue/descriptor.
 *
 * @retval	The size fo the descriptor
 *
 */
int gie_get_desc_size(enum gie_desc_type type);

/**
 * Create a GIE event
 *
 * The event API is called to create a sys_event for a GIE, that
 * can later be polled through the mv_sys_event_poll() API.
 * This is only releavnt to 'NMP_SCHED_TX'
 *
 * @param[in]	gie		A pointer to a GIE object.
 * @param[in]	params		Parameters for the event.
 * @param[out]	ev		A pointer to event handle of type 'struct mv_sys_event *'.
 *
 * @retval      0 on success
 * @retval      <0 on failure
 */
int gie_create_event(struct gie *gie, struct gie_event_params *params, struct mv_sys_event **ev);

/**
 * Delete a GIE event
 *
 * @param[in]	ev		A sys_event handle.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 */
int gie_delete_event(struct mv_sys_event *ev);

/**
 * Set a GIE event
 *
 * The set_event API is called to enable the creation of events for the related GIE.
 *
 * @param[in]	ev		A sys_event handle.
 * @param[in]	en		enable/disable
 *
 * @retval      0 on success
 * @retval      <0 on failure
 */
int gie_set_event(struct mv_sys_event *ev, int en);

/**
 * Get statistics
 *
 * The gie_get_stats API is called to get the statistics packets
 * count of a specific gie queue
 *
 * @param[in]	gie		A pointer to a GIE object.
 * @param[in]	qid		queue ID.
 * @param[out]	pkt_cnt	stats result.
 * @param[in]	reset	stats reset flag.
 *
 * @retval      0 on success
 * @retval      <0 on failure
 */
int gie_get_queue_stats(void *gie, u16 qid, u64 *pkt_cnt, int reset);
int gie_dump_queue_pair(void *giu, u16 qid);

int gie_add_bm_queue_reference(void *giu, void *bm_queue_ref);
int gie_remove_bm_queue_refernce(void *giu, void *bm_queue_ref);

#endif /* _GIE_H */
