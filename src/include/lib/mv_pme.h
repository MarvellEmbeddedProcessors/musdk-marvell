/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __MV_PME_H__
#define __MV_PME_H__

#include <string.h>
#include <sys/time.h>

#include "mv_std.h"

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

extern struct event_counters counters[PME_MAX_EVENT_CNTS];

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
	} else {
		ev_cnt->zero_cnt++;
	}
}

static inline void pme_ev_cnt_stop_n_report(int cnt, u32 num)
{
	struct event_counters	*ev_cnt = &counters[cnt];

	pme_ev_cnt_stop(cnt, num);
	if (ev_cnt->evs_cnt >= ev_cnt->max_cnt)
		pme_ev_cnt_dump(cnt, 1);
}

#endif /* __MV_PME_H__ */
