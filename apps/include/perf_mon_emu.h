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

#ifndef __PERF_MON_EMU_H__
#define __PERF_MON_EMU_H__

#include <string.h>
#include <sys/time.h>

#include "mvapp_std.h"


#define PME_MAX_NAME_SIZE	20
#define PME_MAX_EVENT_CNTS	8

struct event_counters {
	char		name[PME_MAX_NAME_SIZE];
	int		in_use;
	int		ext_print;
	u32		max_cnt;
	u64		usecs;
	/** number of times the event-counter was triggered */
	u64		trig_cnt;
	/** number of times the event-counter was triggered with '0' value */
	u64		zero_cnt;
	/** sum of all values that were passed to the event-counter upooen trigger */
	u64		evs_cnt;
	u64		max_evs;
	struct timeval	t_start;
	struct timeval	t_last;
};


extern struct event_counters	counters[PME_MAX_EVENT_CNTS];


int pme_ev_cnt_create(char *name, u32 max_cnt, int ext_print);
void pme_ev_cnt_destroy(int cnt);

void pme_ev_cnt_dump(int cnt, int reset);


static inline void pme_ev_cnt_start(int cnt)
{
	gettimeofday(&counters[cnt].t_start, NULL);
}

static inline void pme_ev_cnt_stop(int cnt, u32 num)
{
	struct event_counters	*ev_cnt = &counters[cnt];
	struct timeval		 t_curr;

	gettimeofday(&t_curr, NULL);

	/* compute and print the elapsed time in millisec */
	if (num) {
		ev_cnt->usecs += (t_curr.tv_sec - ev_cnt->t_start.tv_sec) * 1000000;
		ev_cnt->usecs += (t_curr.tv_usec - ev_cnt->t_start.tv_usec);
		ev_cnt->evs_cnt += num;
		ev_cnt->trig_cnt++;
		if (num > ev_cnt->max_evs)
			ev_cnt->max_evs = num;
	} else
		ev_cnt->zero_cnt++;
}

static inline void pme_ev_cnt_stop_n_report(int cnt, u32 num)
{
	struct event_counters	*ev_cnt = &counters[cnt];

	pme_ev_cnt_stop(cnt, num);
	if (ev_cnt->evs_cnt >= ev_cnt->max_cnt)
		pme_ev_cnt_dump(cnt, 1);
}

#endif /* __PERF_MON_EMU_H__ */
