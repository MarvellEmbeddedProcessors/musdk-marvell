/******************************************************************************
*  Copyright (C) 2019 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#ifndef _VF_H
#define _VF_H

#include "mv_std.h"
#include "drivers/mv_mqa.h"
#include "drivers/mv_giu.h"
#include "mng/mv_nmp_dispatch.h"

struct nmnicvf;

struct nmnicvf_params {
	u8				 lf_id;
	u8				 id;
	u8				 guest_id;

	struct nmdisp			*nmdisp;
	struct mqa			*mqa;
	struct giu			*giu;

	struct nmp_lf_nicvf_params	*nmp_nicvf_params;

	int				(*f_ready_cb)(void *arg, u8 lf_id);
	int				(*f_get_free_bar_cb)(void *arg, void **va, dma_addr_t *pa);
	void				(*f_put_bar_cb)(void *arg, int index);
	int				(*f_get_vf_bar_cb)(void *arg, u8 vf_id, u8 bar, void **va, void **pa);

	void				*arg;
};

int nmnicvf_init(struct nmnicvf_params *params, struct nmnicvf **nmnicpf);
int nmnicvf_deinit(struct nmnicvf *nmnicvf);

/**
 * Serialize the NIC-VF parameters
 *
 * @param[in]	nmnicvf		A NIC-VF handle.
 * @param[in]	buff		Pointer to a buffer where the resulting string is stored.
 *				The buffer should have a size of at least 'size' characters.
 * @param[in]	size		size of buffer.
 * @param[in]	is_relations_info serialize relations info
 *
 * @retval	The number of characters that would have been written if 'size' had been sufficiently large
 * @retval	<0 on failure
 */
int nmnicvf_serialize(struct nmnicvf *nmnicvf, char *buff, u32 size, int is_relations_info);


/* TODO: temporary routine until this will be done through guest message passing! */
/* TODO: Not used right now
* int nmnicpf_create_scheduling_event(struct nmnicpf *nmnicpf,
*	struct nmp_event_params *params,
*	struct mv_sys_event **ev);
* int nmnicpf_delete_scheduling_event(struct mv_sys_event *ev);
* int nmnicpf_set_scheduling_event(struct mv_sys_event *ev, int en);
*/
#endif /* _VF_H */
