/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "std_internal.h"

#include "lib/mv_pme.h"

struct event_counters counters[PME_MAX_EVENT_CNTS] = {0};

static int clk_mhz;

static uint32_t read_clock_mhz(void)
{
	char buffer[256], *endptr = NULL;
	FILE *file;
	uint32_t ret = 0;

	file = fopen("/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq", "r");
	if (file == NULL)
		return ret;

	if (fgets(buffer, sizeof(buffer), file) != NULL)
		ret = strtoull(buffer, &endptr, 0) / 1000;

	fclose(file);

	pr_info("CPU MHZ: %u\n", ret);
	return ret;
}

int pme_ev_cnt_create(char *name, u32 max_cnt, int ext_print)
{
	int i;

	if (!clk_mhz)
		clk_mhz = read_clock_mhz();

	if (strlen(name) > (PME_MAX_NAME_SIZE - 1)) {
		pr_err("Event counter name too long!\n");
		return -EINVAL;
	}
	for (i = 0; i < PME_MAX_EVENT_CNTS; i++)
		if (!counters[i].in_use) {
			memset(&counters[i], 0, sizeof(counters[i]));
			snprintf(counters[i].name, sizeof(counters[i].name), "%s", name);
			counters[i].max_cnt = max_cnt;
			counters[i].ext_print = ext_print;
			counters[i].in_use = 1;
			break;
		}
	if (i == PME_MAX_EVENT_CNTS) {
		pr_err("Maximum number of event counters exceeded!\n");
		return -EIO;
	}

	pr_debug("Allocated event %d for %s\n", i, counters[i].name);
	return i;
}

void pme_ev_cnt_destroy(int cnt)
{
	counters[cnt].in_use = 0;
}

void pme_ev_cnt_dump(int cnt, int reset)
{
	struct event_counters	*ev_cnt = &counters[cnt];
	struct timeval		t_curr;
	u64			tmp;

	gettimeofday(&t_curr, NULL);

	tmp  = (t_curr.tv_sec - ev_cnt->t_last.tv_sec) * 1000000;
	tmp += (t_curr.tv_usec - ev_cnt->t_last.tv_usec);

	printf("Event: %s: Avg cycles: %d, burst: %.2f\n",
	       ev_cnt->name,
	       (int)(ev_cnt->usecs * clk_mhz / ev_cnt->evs_cnt),
	       (float)ev_cnt->evs_cnt / ev_cnt->trig_cnt);
	if (ev_cnt->ext_print)
		printf("\t%d calls for 0 pkts, max was: %d, est. perf: %dKpps\n",
		       (int)ev_cnt->zero_cnt,
		       (int)ev_cnt->max_evs,
		       (int)((ev_cnt->evs_cnt * 1000) / tmp));
	ev_cnt->usecs = ev_cnt->evs_cnt = ev_cnt->trig_cnt = ev_cnt->zero_cnt = 0;
	gettimeofday(&ev_cnt->t_last, NULL);
}
