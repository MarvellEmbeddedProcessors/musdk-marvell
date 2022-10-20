/* Copyright (c) 2018 Marvell.
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/errno.h>
#include <linux/capability.h>
#include <linux/delay.h>

#include "mv_dp_defs.h"
#include "mv_dp_sysfs.h"
#include "mv_dp_main.h"
#include "mv_dp_types.h"
#include "mv_dp_int_if.h"
#include "mv_nss_dp.h"

/*#include "common/mv_sw_if.h"*/

static int mv_dp_dbg_show_ch_rx_buff(int i);
static int mv_dp_dbg_show_ch_data(int i);
static int mv_dp_dbg_show_ch_counters(int i);
static int mv_dp_dbg_level_set(int i);
static int mv_dp_dbg_trace_set(int i, int flag);
static int mv_dp_dbg_trace_set_all(int t, int flag);
static ssize_t mv_dp_dbg_sysfs_help(char *b);
static int mv_dp_dbg_test_log(int level, int p1);
static int mv_dp_dbg_timer_ena(int ch_id, unsigned ms);
static int mv_dp_dbg_set_sysfs_sync(int i);

static void mv_dp_dbg_get_ver_cb(mv_nss_dp_event_t *event);
static void mv_dp_dbg_get_fw_ver(void);
static void mv_dp_dbg_set_cb(void);

static void mv_dp_dbg_send_nop_cb(mv_nss_dp_event_t *event);
static void mv_dp_dbg_send_nop(unsigned c);

static void mv_dp_dbg_nss_mem_write(int t, int off, int size, u8 *data);
static void mv_dp_dbg_nss_mem_read(int t, int off, int size);

static void mv_dp_dbg_nss_mem_write_cb(mv_nss_dp_event_t *event);
static void mv_dp_dbg_nss_mem_read_cb(mv_nss_dp_event_t *event);

#if 0
static void mv_dp_dbg_send_shutdown_cb(mv_nss_dp_event_t *event);
static void mv_dp_dbg_send_shutdown(void);

static void mv_dp_dbg_send_init_cb(mv_nss_dp_event_t *event);
static void mv_dp_dbg_send_init(int i);
#endif

static int mv_dp_dbg_send_exception(int ch, int opcode, int flags, char *msg, size_t size);

static ssize_t mv_dp_dbg_sysfs_help(char *b)
{
	int o = 0;

	o += scnprintf(b + o, PAGE_SIZE - o,
		       "cat                  show_status     - Show DPAPI status info\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "cat                  show_errors     - Show DPAPI error counters\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "cat                  show_ch_map     - Show CPU to CHannel ID map\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "cat                  show_msg_info   - Show MSG configuration parameters\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "cat                  show_event_info - Show Event configuration parameters\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "cat                  counters_clear_all-Clear all DPAPI counters\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "cat                  get_fw_ver      - Get FW version and print it\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "cat                  set_cb          - Set notification callback\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [x]           > send_nop        - Send NOP message with x (32 bit hex)\n"
		       "                                       to FW and get Result\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "cat                  send_shutdown   - Send Shutdown message to FW\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo  [c]          > show_ch_info    - Show channel [c] info\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo  [c]          > show_ch_stats   - Show channel [c] stats\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo  [d]          > dbg_lvl_set     - Set DPAPI debug printout level to\n"
		       "                                       [f]:0(NONE) - %d(ALL)\n", MV_DP_DBG_LAST-1);
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo  [c] [ms]     > timer_set       - Enable//Disable RX Aging Timer for\n"
		       "                                       Channel[c];[ms]-period; 0-Dis;0>Ena\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo  [m] [f]      > msg_trace_set   - Set trace for messageID [m], Trace [f]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo  [t] [f]      > msg_trace_set_all- Set trace for messages per Type [tt]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo  [c]          > show_ch_rx_buf  - Show Pending Recieve message buffer\n"
		       "                                       for channel [n]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo  [s]          > send_init       - Send init message to FW with [s] size\n"
		       "                                       (dec 16 bit)\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo  [ms]         > sysfs_sync_set  - Set sysFS execution mode;\n"
		       "                                       timeout [ms]:0-async; 0<-sync\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo  [u][o][s][d] > nss_mem_write   - Write [s] byte of data[d] to NSS mem type[u] at offset[o]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo  [u][o][s]    > nss_mem_read    - Read [s] byte of memory type[u] at offset[o] from NSS\n");

	o += scnprintf(b + o, PAGE_SIZE - o,
		       "\n      [t]=BITWISE TYPE = EVENT:0x%2X|INTERNAL:0x%2X|MESSAGE:0x%2X\n",
		       MV_DP_MSG_TYPE_EVENT, MV_DP_MSG_TYPE_INTERNAL, MV_DP_MSG_TYPE_MSG);
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "        [f]=BITWISE TRACE = OFF:0x0|RX:0x%2X|TX:0x%2X|CB:0x%2X|\n"
		       "        additional to (RX|TX): RX_BUF:0x%02X|EXT_HDR:0x%2X|\n"
		       "        ALL:0x%2X\n",
		       MV_DP_LOG_TRC_RX, MV_DP_LOG_TRC_TX, MV_DP_LOG_TRC_CB, MV_DP_LOG_TRC_BUF, MV_DP_LOG_TRC_EXT,
		       MV_DP_LOG_TRC_RX|MV_DP_LOG_TRC_TX|MV_DP_LOG_TRC_CB|MV_DP_LOG_TRC_EXT|MV_DP_LOG_TRC_EXT);
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "         TUNNEL_CW:%d, DTLS:%d; L2:%d, L4:%d\n",
		       MV_NSS_DP_TUNNEL_CAPWAP,
		       MV_NSS_DP_DTLS,
		       MV_NSS_DP_L2_FLOW,
		       MV_NSS_DP_L4_FLOW
		       );
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "      [o]= memory offset to read/write (hex 0x...)\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "      [s]= memory size to read/write in bytes (dec) max %d|%d(write|read)\n",
		       MV_DP_MSG_NSS_MEM_DUMP_SIZE_WRITE_B,
		       MV_DP_MSG_NSS_MEM_DUMP_SIZE_READ_B);
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "      [d]= memory byte stream to write (01DEADBEEF02...)\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "      [u]= memory type to read/write 0-SSRAM, 1-CSRAM, 2-SE...\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "\nSysFS Execution mode: %s (%d)\n", (mv_dp_sysfs_delay_get()) ? ("Sync") : ("Async"),
										mv_dp_sysfs_delay_get());



	return o;
}

static ssize_t mv_dp_dbg_sysfs_show(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	int off = 0;
	const char  *name = attr->attr.name;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	if (!strcmp(name, "help")) {
		return mv_dp_dbg_sysfs_help(buf);
	} else if (!strcmp(name, "show_status")) {
		mv_dp_show_cfg();
	} else if (!strcmp(name, "counters_clear_all")) {
		mv_dp_clear_all_counters();
	} else if (!strcmp(name, "show_ch_map")) {
		mv_dp_show_ch_map();
	} else if (!strcmp(name, "show_msg_info")) {
		mv_dp_show_msg_info();
	} else if (!strcmp(name, "show_event_info")) {
		mv_dp_show_event_info();
	} else if (!strcmp(name, "show_errors")) {
		mv_dp_show_error_counters();
	} else if (!strcmp(name, "get_fw_ver")) {
		mv_dp_dbg_get_fw_ver();
	} else if (!strcmp(name, "set_cb")) {
		mv_dp_dbg_set_cb();
#if 0
	} else if (!strcmp(name, "send_shutdown")) {
		mv_dp_dbg_send_shutdown();
#endif
	} else {

		MV_DP_CLI_FAIL("Parse Error CMD: <%s>\n", MV_NSS_DP_INVALID_PARAM, name);
		off = mv_dp_dbg_sysfs_help(buf);
	}

	return off;
}

static ssize_t mv_dp_dbg_sysfs_store(struct device *dev,
			   struct device_attribute *attr, const char *buf,
			   size_t len)
{
	const char *name = attr->attr.name;
	unsigned int    a, b, c, d;
	char str[128];
	u8 mem_arr[MV_DP_MSG_NSS_MEM_DUMP_SIZE_WRITE_B];

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	a = b = c = 0;

	if (!strcmp(name, "show_ch_rx_buf")) {
		if (1 != sscanf(buf, "%d", &a))
			goto err;
		mv_dp_dbg_show_ch_rx_buff(a);
	} else if (!strcmp(name, "show_ch_info")) {
		if (1 != sscanf(buf, "%d", &a))
			goto err;
		mv_dp_dbg_show_ch_data(a);
	} else if (!strcmp(name, "show_ch_stats")) {
		if (1 != sscanf(buf, "%d", &a))
			goto err;
		mv_dp_dbg_show_ch_counters(a);
	} else if (!strcmp(name, "dbg_lvl_set")) {
		if (1 != sscanf(buf, "%d", &a))
			goto err;
		mv_dp_dbg_level_set(a);
	} else if (!strcmp(name, "msg_trace_set")) {
		if (2 != sscanf(buf, "%d %x", &a, &b))
			goto err;
		mv_dp_dbg_trace_set(a, b);
	} else if (!strcmp(name, "timer_set")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_dbg_timer_ena(a, b);
	} else if (!strcmp(name, "msg_trace_set_all")) {
		if (2 != sscanf(buf, "0x%x 0x%x", &a, &b))
			goto err;
		mv_dp_dbg_trace_set_all(a, b);
#if 0
	} else if (!strcmp(name, "send_init")) {
		if (1 != sscanf(buf, "%d", &a))
			goto err;
		mv_dp_dbg_send_init(a);
#endif
	} else if (!strcmp(name, "send_nop")) {
		if (1 != sscanf(buf, "0x%08X", &a))
			goto err;
		mv_dp_dbg_send_nop(a);
	} else if (!strcmp(name, "test_log")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_dbg_test_log(a, b);
	} else if (!strcmp(name, "sysfs_sync_set")) {
		if (1 != sscanf(buf, "%d", &a))
			goto err;
		mv_dp_dbg_set_sysfs_sync(a);
	} else if (!strcmp(name, "send_exception")) {
		if (5 != sscanf(buf, "%d %d %d %d %s", &a, &b, &c, &d, str))
			goto err;
		mv_dp_dbg_send_exception(a, b, c, str, d);
	} else if (!strcmp(name, "nss_mem_write")) {
		if (4 != sscanf(buf, "%d %i %d %s", &a, &b, &c, str))
			goto err;
		if (c > MV_DP_MSG_NSS_MEM_DUMP_SIZE_WRITE_B || c < 1) {
			MV_DP_CLI_FAIL("Wrong <size>\n", MV_NSS_DP_INVALID_PARAM);
			goto err;
		}
/*		str_to_hex(str, strlen(str), mem_arr, c);*/
		mv_dp_dbg_nss_mem_write(a, b, c, mem_arr);
	} else if (!strcmp(name, "nss_mem_read")) {
		if (3 != sscanf(buf, "%d %i %d", &a, &b, &c))
			goto err;
		if (c > MV_DP_MSG_NSS_MEM_DUMP_SIZE_READ_B || c < 1)
			goto err;
		mv_dp_dbg_nss_mem_read(a, b, c);
	} else {
		goto err;
	}

	return len;
err:

	MV_DP_CLI_FAIL("Parse Error CMD: <%s>\n", MV_NSS_DP_INVALID_PARAM, name);
	return -EINVAL;
}

static DEVICE_ATTR(help,		S_IRUSR, mv_dp_dbg_sysfs_show, NULL);
static DEVICE_ATTR(show_status,		S_IRUSR, mv_dp_dbg_sysfs_show, NULL);
static DEVICE_ATTR(show_errors,		S_IRUSR, mv_dp_dbg_sysfs_show, NULL);
static DEVICE_ATTR(show_ch_map,		S_IRUSR, mv_dp_dbg_sysfs_show, NULL);
static DEVICE_ATTR(show_msg_info,	S_IRUSR, mv_dp_dbg_sysfs_show, NULL);
static DEVICE_ATTR(show_event_info,	S_IRUSR, mv_dp_dbg_sysfs_show, NULL);
static DEVICE_ATTR(get_fw_ver,		S_IRUSR, mv_dp_dbg_sysfs_show, NULL);
static DEVICE_ATTR(set_cb,		S_IRUSR, mv_dp_dbg_sysfs_show, NULL);
static DEVICE_ATTR(send_shutdown,	S_IRUSR, mv_dp_dbg_sysfs_show, NULL);
static DEVICE_ATTR(counters_clear_all,	S_IRUSR, mv_dp_dbg_sysfs_show, NULL);
static DEVICE_ATTR(show_ch_rx_buf,	S_IWUSR, NULL, mv_dp_dbg_sysfs_store);
static DEVICE_ATTR(show_ch_info,	S_IWUSR, NULL, mv_dp_dbg_sysfs_store);
static DEVICE_ATTR(show_ch_stats,	S_IWUSR, NULL, mv_dp_dbg_sysfs_store);
static DEVICE_ATTR(dbg_lvl_set,		S_IWUSR, NULL, mv_dp_dbg_sysfs_store);
static DEVICE_ATTR(timer_set,		S_IWUSR, NULL, mv_dp_dbg_sysfs_store);
static DEVICE_ATTR(msg_trace_set,	S_IWUSR, NULL, mv_dp_dbg_sysfs_store);
static DEVICE_ATTR(msg_trace_set_all,	S_IWUSR, NULL, mv_dp_dbg_sysfs_store);
static DEVICE_ATTR(test_log,		S_IWUSR, NULL, mv_dp_dbg_sysfs_store);
static DEVICE_ATTR(send_init,		S_IWUSR, NULL, mv_dp_dbg_sysfs_store);
static DEVICE_ATTR(send_nop,		S_IWUSR, NULL, mv_dp_dbg_sysfs_store);
static DEVICE_ATTR(sysfs_sync_set,	S_IWUSR, NULL, mv_dp_dbg_sysfs_store);
static DEVICE_ATTR(send_exception,	S_IWUSR, NULL, mv_dp_dbg_sysfs_store);
static DEVICE_ATTR(nss_mem_write,	S_IWUSR, NULL, mv_dp_dbg_sysfs_store);
static DEVICE_ATTR(nss_mem_read,	S_IWUSR, NULL, mv_dp_dbg_sysfs_store);


static struct attribute *mv_dp_dbg_sysfs_attrs[] = {
	&dev_attr_help.attr,
	&dev_attr_show_status.attr,
	&dev_attr_show_errors.attr,
	&dev_attr_show_ch_map.attr,
	&dev_attr_show_msg_info.attr,
	&dev_attr_show_event_info.attr,
	&dev_attr_show_ch_rx_buf.attr,
	&dev_attr_show_ch_info.attr,
	&dev_attr_show_ch_stats.attr,
	&dev_attr_dbg_lvl_set.attr,
	&dev_attr_timer_set.attr,
	&dev_attr_msg_trace_set.attr,
	&dev_attr_msg_trace_set_all.attr,
	&dev_attr_test_log.attr,
	&dev_attr_get_fw_ver.attr,
	&dev_attr_set_cb.attr,
	&dev_attr_send_nop.attr,
	&dev_attr_send_init.attr,
	&dev_attr_send_shutdown.attr,
	&dev_attr_sysfs_sync_set.attr,
	&dev_attr_send_exception.attr,
	&dev_attr_counters_clear_all.attr,
	&dev_attr_nss_mem_write.attr,
	&dev_attr_nss_mem_read.attr,

	NULL
};


static struct attribute_group mv_dp_dbg_sysfs_group = {
	.name = "dbg",
	.attrs = mv_dp_dbg_sysfs_attrs,
};



int mv_dp_dbg_sysfs_init(struct kobject *ko)
{
	int err;
	err = sysfs_create_group(ko, &mv_dp_dbg_sysfs_group);

	if (err) {
		MV_DP_LOG_INF("DBG sysFS group init failed %d\n", err);
		return err;
	}

	MV_DP_LOG_DBG1("DBG sysFS INITALIZED\n");
	return err;
}

int mv_dp_dbg_sysfs_exit(struct kobject *ko)
{
	sysfs_remove_group(ko, &mv_dp_dbg_sysfs_group);
	return 0;
}


static int mv_dp_dbg_show_ch_rx_buff(int i)
{
	struct mv_dp_ch_data const *ch;

	if (!MV_DP_CH_ID_IS_OK(i)) {
		MV_DP_CLI_FAIL("Channel ID out of Range: 0..%d\n", MV_NSS_DP_INVALID_PARAM, MV_DP_MAX_CHAN_NUM);
		return -EINVAL;
	}

	ch = mv_dp_get_ch_data(i);
	if (!ch) {
		MV_DP_CLI_FAIL("CH not Initialized: %d\n", MV_NSS_DP_FAILED, i);
		return -EINVAL;
	}

	mv_dp_show_ch_rx_buffer(ch);

	return 0;
}

static int mv_dp_dbg_show_ch_data(int i)
{
	struct mv_dp_ch_data const *ch;

	if (!MV_DP_CH_ID_IS_OK(i)) {
		MV_DP_CLI_FAIL("Channel ID out of Range: 0..%d\n", MV_NSS_DP_INVALID_PARAM, MV_DP_MAX_CHAN_NUM);
		return -EINVAL;
	}

	ch = mv_dp_get_ch_data(i);
	if (!ch) {
		MV_DP_CLI_FAIL("CH not Initialized: %d\n", MV_NSS_DP_FAILED, i);
		return -EINVAL;
	}

	mv_dp_show_ch_data(ch);

	return 0;
}


static int mv_dp_dbg_show_ch_counters(int i)
{
	struct mv_dp_ch_data const *ch;

	if (!MV_DP_CH_ID_IS_OK(i)) {
		MV_DP_CLI_FAIL("Channel ID out of Range: 0..%d\n", MV_NSS_DP_INVALID_PARAM, MV_DP_MAX_CHAN_NUM);
		return -EINVAL;
	}

	ch = mv_dp_get_ch_data(i);
	if (!ch) {
		MV_DP_CLI_FAIL("CH not Initialized: %d\n", MV_NSS_DP_FAILED, i);
		return -EINVAL;
	}

	mv_dp_show_ch_counters(ch);

	return 0;
}



static int mv_dp_dbg_set_sysfs_sync(int i)
{

	mv_dp_sysfs_delay_set(i);
	MV_DP_CLI_OK("DPAPI Sync mode:%c (%d)\n", (i) ? ('Y') : ('N'), mv_dp_sysfs_delay_get());

	return 0;
}



static int mv_dp_dbg_level_set(int i)
{

	if (!MV_DP_DBG_LVL_IS_OK(i)) {
		MV_DP_CLI_FAIL("DBG LEVEL out of Range: 0(none)..%d(all)\n", MV_NSS_DP_INVALID_PARAM, MV_DP_DBG_LAST-1);
		return -EINVAL;
	}

	mv_dp_dbg_set(i);

	MV_DP_CLI_OK("Debug Level set to %d\n", i);
	return 0;
}

static int mv_dp_dbg_timer_ena(int ch_id, unsigned ms)
{

	struct mv_dp_ch_data *ch;

	ch = (struct mv_dp_ch_data *)mv_dp_get_ch_data(ch_id);
	if (!ch) {
		MV_DP_CLI_FAIL("CH not Initialized: %d\n", MV_NSS_DP_FAILED, ch_id);
		return -EINVAL;
	}

	mv_dp_timer_set_interval(ch, ms);
	mdelay(ms * 2);
	mv_dp_timer_set(ch);
	MV_DP_CLI_OK("Timer Set ch:%d to:%d\n", ch_id, ms);

	return 0;
}


static int mv_dp_dbg_trace_set(int i, int f)
{
	if (!MV_DP_MSGID_IS_OK(i)) {
		MV_DP_CLI_FAIL("MSG ID is out of Range: 0..%d\n", MV_NSS_DP_INVALID_PARAM, MV_DP_MSGID_LAST);
		return -EINVAL;
	}

	if (!MV_DP_LOG_TRC_IS_OK(f)) {
		MV_DP_CLI_FAIL("MSG TRACE is out of Range:(BITWISE) 0x%02X - none,\n"
		       "RX-0x%02X, TX-0x%02X, CB-0x%02X\n"
		       "Coupled with (TX|RX): RX_BUFFER-0x%2X EXT-0x%2X\n",
		       MV_NSS_DP_INVALID_PARAM,
		       MV_DP_LOG_TRC_NONE,
		       MV_DP_LOG_TRC_RX,
		       MV_DP_LOG_TRC_TX,
		       MV_DP_LOG_TRC_CB,
		       MV_DP_LOG_TRC_BUF,
		       MV_DP_LOG_TRC_EXT);
		return -EINVAL;
	}

	if (MV_DP_RC_OK != mv_dp_trace_set(i, f))
		MV_DP_CLI_FAIL("MSG ID %d trace NOT set to %d\n", MV_NSS_DP_FAILED, i, f);
	else
		MV_DP_CLI_OK("MSG ID %d trace set to %d\n", i, f);
	return 0;
}

static int mv_dp_dbg_trace_set_all(int i, int f)
{
	if (!MV_DP_MSG_TYPE_IS_OK(i)) {
		MV_DP_CLI_FAIL("MSG TYPE is out of Range: 0x%x..0x%x\n",
			       MV_NSS_DP_INVALID_PARAM,
			       MV_DP_MSG_TYPE_FIRST,
			       MV_DP_MSG_TYPE_LAST);
		return -EINVAL;
	}

	if (!MV_DP_LOG_TRC_IS_OK(f)) {
		MV_DP_CLI_FAIL("MSG TRACE is out of Range:(BITWISE) 0x%02X - none,\n"
			       "RX-0x%02X, TX-0x%02X, CB-0x%02X\n"
			       "Coupled with (TX|RX): RX_BUFFER-0x%2X EXT-0x%2X\n",
			       MV_NSS_DP_INVALID_PARAM,
			       MV_DP_LOG_TRC_NONE,
			       MV_DP_LOG_TRC_RX,
			       MV_DP_LOG_TRC_TX,
			       MV_DP_LOG_TRC_CB,
			       MV_DP_LOG_TRC_BUF,
			       MV_DP_LOG_TRC_EXT);
		return -EINVAL;
	}

	mv_dp_trace_set_all((u8)i, (u8)f);

	MV_DP_CLI_OK("MSG Type %d trace set to %d\n", i, f);
	return 0;
}


static int mv_dp_dbg_test_log(int level, int p1)
{

	MV_DP_LOG_DBG3("DBG3 l:%d param: %d\n", level, p1);
	MV_DP_LOG_DBG2("DBG2 l:%d param: %d\n", level, p1);
	MV_DP_LOG_DBG1("DBG1 l:%d param: %d\n", level, p1);

	return 0;
}


static int mv_dp_dbg_send_exception(int ch, int opcode, int flags,  char *txt, size_t size)
{
	u32	msg[MV_DP_CFH_MSG_BUF_SIZE_B];


	MV_DP_CLI_OK("Generate Exception: ch:%d, opcode:%d, flags:%X, size:%ld, txt:%s\n",
		     ch, opcode, flags, size, txt);

	msg[0] = cpu_to_be32(opcode);

	if (size) {
		memcpy(&msg[1], txt, size);
		msg[size/4 + 4] = '\0';
		/*mv_dp_msg_rx(ch, msg, sizeof(u32) * 2 + size, 0, flags, opcode, 0, 1);*/
	} else {
		/*mv_dp_msg_rx(ch, msg, sizeof(u32) * 2, 0, flags, opcode, 0, 1);*/
	}

	return 0;
}


static void mv_dp_dbg_get_fw_ver(void)
{
	mv_nss_dp_result_spec_t cb;
	enum mv_dp_rc rc;
	struct completion *compl_ptr;

	memset(&cb, 0, sizeof(cb));

	cb.cb = mv_dp_dbg_get_ver_cb;

	if (mv_dp_sysfs_delay_get()) {
		compl_ptr = kmalloc(sizeof(struct completion), GFP_KERNEL);
		if (!compl_ptr) {
			MV_DP_CLI_FAIL("Completion allocation Failed\n", MV_NSS_DP_OUT_OF_RESOURCES);
			return;
		}
		init_completion(compl_ptr);
		cb.cookie = compl_ptr;
	} else {
		cb.cookie = 0;
	}

	rc = mv_dp_msg_ver_tx(&cb);
	if (MV_DP_RC_OK != rc) {
		MV_DP_CLI_FAIL("Get Version rc:%d\n", MV_NSS_DP_FAILED, rc);
		if (cb.cookie)
			kfree(cb.cookie);
		return;
	}

	MV_DP_CLI_TMSG("Get Version Send to FW\n");
	if (cb.cookie) {
		if (wait_for_completion_timeout(cb.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Get Version Timeout\n", MV_NSS_DP_API_EXEC_TIMEOUT);
		kfree(cb.cookie);
	}
}

void mv_dp_dbg_rx_cb(mv_nss_dp_event_t *evt)
{
	char *c = evt->params.notify_msg;
	int i = 4;

	/*on event type*/
	if (!evt) {
		MV_DP_LOG_INF("DBG NULL Event CB Received\n");
		return;
	}

	switch (evt->type) {
	case MV_NSS_DP_EVT_NOTIFY_CODE:
		MV_DP_LOG_EVT("DBG SYS EVENT CODE:0x%08X\n", evt, *(evt->params.notify_code));
		break;
	case MV_NSS_DP_EVT_NOTIFY_MSG:
		MV_DP_LOG_EVT("DBG SYS EVENT CODE:0x%08X\n", evt, *(evt->params.notify_code));
		/*print count chars*/
		while (c[i] != '\0' && i < MV_DP_SYSTEM_MSG_SIZE) {
			MV_DP_LOG_CONT("%c", c[i]);
			i++;
		}
		MV_DP_LOG_CONT("\nevent msg total: %d bytes\n", i);
		break;

	default:
		MV_DP_LOG_INF("DBG Unknown event type in CB:%d\n", evt->type);
		return;
	}

}

static void mv_dp_dbg_set_cb(void)
{
	enum mv_dp_rc rc;

	rc = mv_nss_dp_register_notify_cb(mv_dp_dbg_rx_cb);
	if (rc != MV_DP_RC_OK) {
		MV_DP_CLI_FAIL("Set CB rc:%d\n", MV_NSS_DP_FAILED, rc);
		return;
	}

	MV_DP_CLI_OK("Set CB\n");
}

static void mv_dp_dbg_nss_mem_write(int type, int off, int size, u8 *mem)
{
	mv_nss_dp_result_spec_t res;
	struct mv_dp_dbg_nss_mem mem_tx;
	enum mv_dp_rc rc;
	struct completion *compl_ptr;

	memset(&res, 0, sizeof(res));

	res.cb = mv_dp_dbg_nss_mem_write_cb;
	res.xid = off;

	if (mv_dp_sysfs_delay_get()) {
		compl_ptr = kmalloc(sizeof(struct completion), GFP_KERNEL);
		if (!compl_ptr) {
			MV_DP_CLI_FAIL("Completion allocation Failed\n", MV_NSS_DP_OUT_OF_RESOURCES);
			return;
		}
		init_completion(compl_ptr);
		res.cookie = compl_ptr;
	} else {
		res.cookie = 0;
	}

	mem_tx.type = type;
	mem_tx.offset = off;
	mem_tx.size = size;
	memcpy(mem_tx.arr, mem, size);

	rc = mv_dp_msg_nss_mem_write_tx(&mem_tx, &res);
	if (MV_DP_RC_OK != rc) {
		MV_DP_CLI_FAIL("nss mem write failed\n", MV_NSS_DP_FAILED);
		if (res.cookie)
			kfree(res.cookie);
		return;
	}

	MV_DP_CLI_TMSG("nss mem write Sent:\n");
	mv_dp_nss_mem_show(&mem_tx);
	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("nss mem write Timeout\n", MV_NSS_DP_API_EXEC_TIMEOUT);
		kfree(res.cookie);
	}
}


static void mv_dp_dbg_nss_mem_read(int type, int off, int size)
{
	mv_nss_dp_result_spec_t res;
	struct mv_dp_dbg_nss_mem mem_tx;
	enum mv_dp_rc rc;
	struct completion *compl_ptr;

	memset(&res, 0, sizeof(res));

	res.cb = mv_dp_dbg_nss_mem_read_cb;
	res.xid = off;

	if (mv_dp_sysfs_delay_get()) {
		compl_ptr = kmalloc(sizeof(struct completion), GFP_KERNEL);
		if (!compl_ptr) {
			MV_DP_CLI_FAIL("Completion allocation Failed\n", MV_NSS_DP_OUT_OF_RESOURCES);
			return;
		}
		init_completion(compl_ptr);
		res.cookie = compl_ptr;
	} else {
		res.cookie = 0;
	}

	mem_tx.type = type;
	mem_tx.offset = off;
	mem_tx.size = size;

	rc = mv_dp_msg_nss_mem_read_tx(&mem_tx, &res);
	if (MV_DP_RC_OK != rc) {
		MV_DP_CLI_FAIL("nss mem write failed\n", MV_NSS_DP_FAILED);
		if (res.cookie)
			kfree(res.cookie);
		return;
	}

	MV_DP_CLI_TMSG("nss mem read Sent\n");
	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("nss mem write Timeout\n", MV_NSS_DP_API_EXEC_TIMEOUT);
		kfree(res.cookie);
	}
}

static void mv_dp_dbg_get_ver_cb(mv_nss_dp_event_t *event)
{
	u32	ver;

	if (!event)
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);

	if (MV_NSS_DP_OK != event->status) {
		MV_DP_CLI_FAIL_CB("Get FW Version Status: %s\n", event, mv_dp_err_name_get(event->status));
		goto err;
	}

	ver = *((u32 *)event->params.notify_msg);

	MV_DP_CLI_OK_CB("FW version:%02X-%02X-%02X (0x%08X)\n",
			event,
			MV_DP_MSG_VER_MAJ_GET(ver),
			MV_DP_MSG_VER_MID_GET(ver),
			MV_DP_MSG_VER_MIN_GET(ver),
			ver);

err:	if (event->cookie && event->status != MV_NSS_DP_API_EXEC_TIMEOUT)
		complete(event->cookie);
}

static void mv_dp_dbg_nss_mem_write_cb(mv_nss_dp_event_t *event)
{

	if (!event)
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);

	if (MV_NSS_DP_OK != event->status) {
		MV_DP_CLI_FAIL_CB("NSS mem write Status: %s\n", event, mv_dp_err_name_get(event->status));
		goto err;
	}

	MV_DP_CLI_OK_CB("NSS mem written to 0x%08X\n", event, event->xid);


err:	if (event->cookie && event->status != MV_NSS_DP_API_EXEC_TIMEOUT)
		complete(event->cookie);
}


static void mv_dp_dbg_nss_mem_read_cb(mv_nss_dp_event_t *event)
{
	struct mv_dp_dbg_nss_mem *mem;

	if (!event)
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);

	if (MV_NSS_DP_OK != event->status) {
		MV_DP_CLI_FAIL_CB("NSS mem read Status: %s\n", event, mv_dp_err_name_get(event->status));
		goto err;
	}

	mem = (struct mv_dp_dbg_nss_mem *)event->params.notify_msg;
	MV_DP_CLI_OK_CB("NSS mem Read\n", event);
	mv_dp_nss_mem_show(mem);

err:	if (event->cookie && event->status != MV_NSS_DP_API_EXEC_TIMEOUT)
		complete(event->cookie);
}

static void mv_dp_dbg_send_nop(unsigned cookie)
{
	mv_nss_dp_result_spec_t		cb;
	enum mv_dp_rc			rc;
	struct completion		*compl_ptr;

	cb.cb = mv_dp_dbg_send_nop_cb;
	cb.xid = cookie;

	if (mv_dp_sysfs_delay_get()) {
		compl_ptr = kmalloc(sizeof(struct completion), GFP_KERNEL);
		if (!compl_ptr) {
			MV_DP_CLI_FAIL("Completion allocation Failed\n", MV_NSS_DP_OUT_OF_RESOURCES);
			return;
		}
		init_completion(compl_ptr);
		cb.cookie = compl_ptr;
	} else {
		cb.cookie = 0;
	}

	rc = mv_dp_msg_nop_tx(cookie, &cb);
	if (MV_DP_RC_OK != rc) {
		MV_DP_CLI_FAIL("Send Request rc:%d\n", MV_NSS_DP_FAILED, rc);
		if (cb.cookie)
			kfree(cb.cookie);
		return;
	}

	MV_DP_CLI_TMSG("Nop:0x%08X Send to FW\n", cookie);
	if (cb.cookie) {
		if (wait_for_completion_timeout(cb.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Send Request Timeout\n", MV_NSS_DP_API_EXEC_TIMEOUT);
		kfree(cb.cookie);
	}

}

static void mv_dp_dbg_send_nop_cb(mv_nss_dp_event_t *event)
{
	u32	nop;

	if (!event)
		MV_DP_CLI_FAIL("Null event ptr\n", MV_NSS_DP_INVALID_PARAM);

	if (MV_NSS_DP_OK != event->status) {
		MV_DP_CLI_FAIL_CB("Get NOP Status: %s\n", event, mv_dp_err_name_get(event->status));
		goto err;
	}

	nop = *((u32 *)event->params.notify_msg);

	MV_DP_LOG_INF("sizeof param :%ld Pointers: msg:%p port:%p p_stats:%p client:%p vlan:%p\n",
		      sizeof(event->params), event->params.notify_msg, event->params.port,
		      event->params.port_stats, event->params.client, event->params.vlan_cfg);
	MV_DP_CLI_OK_CB("Got NOP\n", event);

err:	if (event->cookie && event->status != MV_NSS_DP_API_EXEC_TIMEOUT)
		complete(event->cookie);
}


#if 0
static void mv_dp_dbg_send_shutdown(void)
{

	mv_nss_dp_result_spec_t res;

	res.cb = mv_dp_dbg_send_shutdown_cb;
	res.xid = 0;
	res.cookie = 0;

	if (MV_NSS_DP_OK != mv_dp_shutdown_tx(&res)) {
		MV_DP_CLI_FAIL("Send Shutdown\n", MV_NSS_DP_FAILED);
		return;
	}

	MV_DP_CLI_TMSG("Send Shutdown\n");
}


static void mv_dp_dbg_send_shutdown_cb(mv_nss_dp_event_t *event)
{
	if (!event)
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);

	if (MV_NSS_DP_OK != event->status) {
		MV_DP_CLI_FAIL_CB("Send Shutdown Status: %s\n", event, mv_dp_err_name_get(event->status));
		return;
	}

	MV_DP_CLI_OK_CB("Shutdown ACK\n", event);
}


static void mv_dp_dbg_send_init(int i)
{
	mv_nss_dp_result_spec_t res;
	mv_nss_dp_init_cfg_t cfg;

	/*src index validation is to be performed in dpapi*/

	/*the index is stored in xid*/
	res.cb = mv_dp_dbg_send_init_cb;
	res.xid = i;
	res.cookie = 0;

	cfg.requests_num = i;
	cfg.sys_evt_handler = 0;

	if (MV_NSS_DP_OK != mv_dp_init_tx(&cfg, &res)) {
		MV_DP_CLI_FAIL("Send Shutdown\n", MV_NSS_DP_FAILED);
		return;
	}

	MV_DP_CLI_TMSG("Send Init:%d\n", i);
}


static void mv_dp_dbg_send_init_cb(mv_nss_dp_event_t *event)
{
	if (!event)
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);

	if (MV_NSS_DP_OK != event->status) {
		MV_DP_CLI_FAIL_CB("Send Init Status: %s\n", event, mv_dp_err_name_get(event->status));
		return;
	}

	MV_DP_CLI_OK_CB("Got INIT ACK\n", event);
}
#endif



