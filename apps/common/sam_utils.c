/******************************************************************************
 *	Copyright (C) 2016 Marvell International Ltd.
 *
 *  If you received this File from Marvell, you may opt to use, redistribute
 *  and/or modify this File under the following licensing terms.
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *	* Redistributions of source code must retain the above copyright
 *	  notice, this list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the distribution.
 *
 *	* Neither the name of Marvell nor the names of its contributors may be
 *	  used to endorse or promote products derived from this software
 *	  without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

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


