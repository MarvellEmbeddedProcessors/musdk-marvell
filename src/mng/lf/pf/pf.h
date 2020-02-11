/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#ifndef _PF_H
#define _PF_H

#include "mv_std.h"
#include "drivers/mv_mqa.h"
#include "drivers/mv_giu.h"
#include "mng/mv_nmp_dispatch.h"

struct nmnicpf;

struct nmnicpf_params {
	u8				 lf_id;
	u8				 id;
	u8				 guest_id;

	struct nmdisp			*nmdisp;
	struct mqa			*mqa;
	struct giu			*giu;

	struct nmp_lf_nicpf_params	*nmp_nicpf_params;

	int				(*f_ready_cb)(void *arg, u8 lf_id);
	int				(*f_get_free_bar_cb)(void *arg, void **va, dma_addr_t *pa);
	void				(*f_put_bar_cb)(void *arg, int index);
	int				(*f_pp_find_free_bpool_cb)(void *arg, u32 pp_id);
	int				(*f_set_vf_bar_offset_base_cb)(void *arg, u64 vf_bar_offset_base);
	void				*arg;
};

int nmnicpf_init(struct nmnicpf_params *params, struct nmnicpf **nmnicpf);
int nmnicpf_deinit(struct nmnicpf *nmnicpf);

/**
 * Serialize the NIC-PF parameters
 *
 * @param[in]	nmnicpf		A NIC-PF handle.
 * @param[in]	buff		Pointer to a buffer where the resulting string is stored.
 *				The buffer should have a size of at least 'size' characters.
 * @param[in]	size		size of buffer.
 * @param[in]	is_relations_info serialize relations info
 *
 * @retval	The number of characters that would have been written if 'size' had been sufficiently large
 * @retval	<0 on failure
 */
int nmnicpf_serialize(struct nmnicpf *nmnicpf, char *buff, u32 size, int is_relations_info);

/* TODO: temporary routine until this will be done through guest message passing! */
int nmnicpf_create_scheduling_event(struct nmnicpf *nmnicpf,
	struct nmp_event_params *params,
	struct mv_sys_event **ev);
int nmnicpf_delete_scheduling_event(struct mv_sys_event *ev);
int nmnicpf_set_scheduling_event(struct mv_sys_event *ev, int en);

#endif /* _PF_H */
