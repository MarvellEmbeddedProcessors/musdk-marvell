/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __MV_SAM_UTILS_H__
#define __MV_SAM_UTILS_H__

#include "mv_std.h"
#include "mv_sam.h"

/*
 * Show cio statistics
 */
int app_sam_show_cio_stats(struct sam_cio *cio_hndl, const char *name, int reset);

/*
 * Show global statistics
 */
int app_sam_show_stats(int reset);


#endif /* __MV_SAM_UTILS_H__ */

