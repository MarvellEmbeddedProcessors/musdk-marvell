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
#include <getopt.h>
#include <sys/time.h>

#include "mv_std.h"
#include "lib/lib_misc.h"
#include "env/mv_sys_dma.h"
#include "utils.h"



int apps_cores_mask_create(int cpus, int affinity)
{
	u64 cores_mask_orig = 0, max_cores_mask, cores_mask;

	max_cores_mask = (1 << system_ncpus());
	cores_mask_orig = (1 << cpus) - 1;

	cores_mask = cores_mask_orig;
	cores_mask <<= (affinity != MVAPPS_INVALID_AFFINITY) ? affinity : MVAPPS_DEFAULT_AFFINITY;

	/* cores_mask must stay in CPU boundaries */
	if (cores_mask > max_cores_mask)
		cores_mask = cores_mask_orig;

	pr_debug("%s: cpus:%d, affinity:%d, cores_mask:0x%lx\n", __func__, cpus, affinity, cores_mask);
	return cores_mask;
}

int apps_thread_to_cpu(struct glb_common_args *cmn_args, int thread)
{
	int j = 0, i = 0;
	u64 core_mask;

	if (cmn_args->cores_mask)
		core_mask = cmn_args->cores_mask;
	else
		core_mask = apps_cores_mask_create(cmn_args->cpus, cmn_args->affinity);

	core_mask = apps_cores_mask_create(cmn_args->cpus, cmn_args->affinity);

	while (core_mask) {
		for (; !((1 << j) & core_mask); j++)
			;

		if (i++ == thread)
			return j;

		core_mask &= ~(1 << j);
	}
	printf("CPU not found for thread #%d\n", thread);
	return -1;
}

int apps_cpu_to_thread(struct glb_common_args *cmn_args, int cpu)
{
	int j = 0, i = 0;
	u64 core_mask;

	if (cmn_args->cores_mask)
		core_mask = cmn_args->cores_mask;
	else
		core_mask = apps_cores_mask_create(cmn_args->cpus, cmn_args->affinity);

	while (core_mask) {
		for (; !((1 << j) & core_mask); j++)
			;

		if (j == cpu)
			return i;

		i++;
		core_mask &= ~(1 << j);
	}
	printf("Thread not found for CPU #%d\n", cpu);
	return -1;

}

int apps_perf_dump(struct glb_common_args *cmn_args)
{
	struct timeval	curr_time;
	u64		tmp_time_inter;
	u64		tmp_rx_cnt = 0, tmp_tx_cnt = 0, drop_cnt = 0;
	u64		tmp_enc_cnt = 0, tmp_dec_cnt = 0;
	int		i;
	struct perf_cmn_cntrs *perf_cntrs;

	gettimeofday(&curr_time, NULL);
	tmp_time_inter = (curr_time.tv_sec - cmn_args->ctrl_trd_last_time.tv_sec) * 1000;
	tmp_time_inter += (curr_time.tv_usec - cmn_args->ctrl_trd_last_time.tv_usec) / 1000;

	for (i = 0; i < MVAPPS_MAX_NUM_CORES; i++) {
		if (cmn_args->largs[i]) {
			perf_cntrs = &((struct local_common_args *)cmn_args->largs[i])->perf_cntrs;
			drop_cnt   += perf_cntrs->drop_cnt;
			tmp_rx_cnt += perf_cntrs->rx_cnt;
			tmp_tx_cnt += perf_cntrs->tx_cnt;
			tmp_enc_cnt += perf_cntrs->enc_cnt;
			tmp_dec_cnt += perf_cntrs->dec_cnt;
		}
	}
	printf("Perf: RX: %d Kpps",
	       (int)((tmp_rx_cnt - cmn_args->last_rx_cnt) / tmp_time_inter));

	if (tmp_enc_cnt - cmn_args->last_enc_cnt)
		printf(" -> Enc: %d Kpps",
		       (int)((tmp_enc_cnt - cmn_args->last_enc_cnt) / tmp_time_inter));

	if (tmp_dec_cnt - cmn_args->last_dec_cnt)
		printf(" -> Dec: %d Kpps",
		       (int)((tmp_dec_cnt - cmn_args->last_dec_cnt) / tmp_time_inter));

	printf(" -> TX: %d Kpps",
	       (int)((tmp_tx_cnt - cmn_args->last_tx_cnt) / tmp_time_inter));

	cmn_args->last_rx_cnt = tmp_rx_cnt;
	cmn_args->last_tx_cnt = tmp_tx_cnt;
	cmn_args->last_enc_cnt = tmp_enc_cnt;
	cmn_args->last_dec_cnt = tmp_dec_cnt;
	if (drop_cnt)
		printf(", drop: %lu", drop_cnt);
	printf("\n");
	gettimeofday(&cmn_args->ctrl_trd_last_time, NULL);

	return 0;
}

int app_ctrl_cb(void *arg)
{
	struct glb_common_args	*cmn_args = (struct glb_common_args *)arg;
	struct timeval	 curr_time;
	u64		 tmp_time_inter;

	if (!cmn_args) {
		pr_err("no obj!\n");
		return -EINVAL;
	}

	gettimeofday(&curr_time, NULL);
	tmp_time_inter = (curr_time.tv_sec - cmn_args->ctrl_trd_last_time.tv_sec) * 1000;
	tmp_time_inter += (curr_time.tv_usec - cmn_args->ctrl_trd_last_time.tv_usec) / 1000;
	if (tmp_time_inter >= cmn_args->ctrl_thresh)
		return apps_perf_dump(cmn_args);
	return 0;
}

int apps_prefetch_cmd_cb(void *arg, int argc, char *argv[])
{
	struct glb_common_args *g_cmn_args = (struct glb_common_args *)arg;

	if (!g_cmn_args) {
		pr_err("no garg obj passed!\n");
		return -EINVAL;
	}
	if ((argc != 1) && (argc != 2)) {
		pr_err("Invalid number of arguments for prefetch cmd!\n");
		return -EINVAL;
	}

	if (argc == 1) {
		printf("%d\n", g_cmn_args->prefetch_shift);
		return 0;
	}

	g_cmn_args->prefetch_shift = atoi(argv[1]);

	return 0;
}

void app_print_horizontal_line(u32 char_count, const char *char_val)
{
	u32 cnt;

	for (cnt = 0; cnt < char_count; cnt++)
		printf("%s", char_val);
	printf("\n");
}

int app_range_validate(int value, int min, int max)
{
	if (((value) > (max)) || ((value) < (min))) {
		pr_err("%s: value 0x%X (%d) is out of range [0x%X , 0x%X].\n",
			__func__, (value), (value), (min), (max));
		return -EFAULT;
	}
	return 0;
}

int app_parse_mac_address(char *buf, u8 *macaddr_parts)
{
	if (sscanf(buf, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
		   &macaddr_parts[0], &macaddr_parts[1],
		   &macaddr_parts[2], &macaddr_parts[3],
		   &macaddr_parts[4], &macaddr_parts[5]) == ETH_ALEN)
		return 0;
	else
		return -EFAULT;
}
