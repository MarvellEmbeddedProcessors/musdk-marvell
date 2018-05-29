/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#ifndef _PF_H
#define _PF_H

#include "mng/mv_nmp_dispatch.h"

int nmnicpf_init(struct nmnicpf *nmnicpf);
int nmnicpf_deinit(struct nmnicpf *nmnicpf);
int nmnicpf_process_command(void *nmnicpf, struct nmdisp_msg *msg);
int nmnicpf_check_link_change(struct nmnicpf *nmnicpf);

#endif /* _PF_H */
