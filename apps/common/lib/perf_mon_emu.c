#include <stdio.h>
#include <stdlib.h>

#include "mvapp_std.h"

#include "perf_mon_emu.h"

struct event_counters	counters[PME_MAX_EVENT_CNTS] = {0};

static int clk_mhz;

static int read_clock_mhz(void)
{
	char		 buffer[20];
	int		 ans;
	FILE		*fp;

	memset(buffer, 0, sizeof(buffer));
	snprintf(buffer, sizeof(buffer), "mhz");
	/* Open the command for reading. */
	fp = popen(buffer, "r");
	if (!fp) {
		pr_err("Failed to run command\n");
		return 0;
	}
	/* Read the output a line at a time - output it. */
	if (fgets(buffer, sizeof(buffer), fp) == NULL) {
		pr_err("Failed to run command\n");
		pclose(fp);
		return 0;
	}
	buffer[strlen(buffer) - 1] = '\0';
	/* close */
	pclose(fp);
	ans = strtol(buffer, NULL, 0);
	return ans;
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
	struct timeval		 t_curr;
	u64			 tmp;

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
