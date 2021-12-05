/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <string.h>
#include <stdio.h>
#include <inttypes.h>

#include "mv_std.h"

#include "sam_utils.h"

/*
 * Show cio statistics
 */
int app_sam_show_cio_stats(struct sam_cio *cio_hndl, const char *name, int reset)
{
	struct sam_cio_stats cio_stats;

	if (!sam_cio_get_stats(cio_hndl, &cio_stats, reset)) {
		printf("-------- SAM %s CIO statistics ------\n", name);
		printf("Enqueue packets             : %" PRIu64 " packets\n", cio_stats.enq_pkts);
		printf("Enqueue bytes               : %" PRIu64 " bytes\n", cio_stats.enq_bytes);
		printf("Enqueue full                : %" PRIu64 " times\n", cio_stats.enq_full);
		printf("Dequeue packets             : %" PRIu64 " packets\n", cio_stats.deq_pkts);
		printf("Dequeue bytes               : %" PRIu64 " bytes\n", cio_stats.deq_bytes);
		printf("Dequeue empty               : %" PRIu64 " times\n", cio_stats.deq_empty);
		printf("\n");
		return 0;
	}
	return -ENOENT;
}

/*
 * Show global statistics
 */
int app_sam_show_stats(int reset)
{
	struct sam_session_stats sa_stats;

	if (!sam_session_get_stats(&sa_stats, reset)) {
		printf("-------- SAM SAs statistics ------\n");
		printf("Created sessions            : %" PRIu64 "\n", sa_stats.sa_add);
		printf("Deleted sessions:	    : %" PRIu64 "\n", sa_stats.sa_del);
		printf("Invalidated sessions:	    : %" PRIu64 "\n", sa_stats.sa_inv);
		printf("\n");
		return 0;
	}
	return -ENOENT;
}


