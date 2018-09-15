/******************************************************************************
 *	Copyright (C) 2018 Marvell International Ltd.
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

#include "std_internal.h"

#include "lib/mv_pme.h"

struct event_counters counters[PME_MAX_EVENT_CNTS] = {0};

static int clk_mhz;

static int read_clock_mhz(void)
{
	char	buffer[20];
	int	ans;
	FILE	*fp;

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
