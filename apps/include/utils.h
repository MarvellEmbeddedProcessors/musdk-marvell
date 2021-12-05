/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __MVUTILS_H__
#define __MVUTILS_H__

#include <stdbool.h>
#include <sys/sysinfo.h>

#include "mv_std.h"
#include "mv_net.h"


/* CA-72 prefetch command */
#if __WORDSIZE == 64
static inline void prefetch(const void *ptr)
{
	asm volatile("prfm pldl1keep, %a0\n" : : "p" (ptr));
}
#else
static inline void prefetch(const void *ptr)
{
	__asm__ __volatile__("pld\t%a0"	: : "p" (ptr));
}
#endif

#define MVAPPS_INVALID_COOKIE_HIGH_BITS		(~0)

/** Get rid of path in filename - only for unix-type paths using '/' */
#define MVAPPS_NO_PATH(file_name)	(strrchr((file_name), '/') ? \
					 strrchr((file_name), '/') + 1 : (file_name))

/* Maximum size of port name */
#define MVAPPS_PPIO_NAME_MAX		20
#define MV_MAX_BUF_STR_LEN		256
#define MVAPPS_MAX_BURST_SIZE		256

/* Maximum number of CPU cores used by application */
#define MVAPPS_NUM_CORES_PER_AP		8
#define MVAPPS_MAX_NUM_AP		2
#define MVAPPS_MAX_NUM_CORES		(MVAPPS_MAX_NUM_AP*MVAPPS_NUM_CORES_PER_AP)

#define MVAPPS_MAX_MEM_REGIONS		2

#define MVAPPS_DEFAULT_AFFINITY		1
#define MVAPPS_INVALID_AFFINITY		-1

#define MVAPPS_DEFAULT_CORE_LOAD	100

#define MVAPPS_INVALID_MEMREGIONS	-1

#define MVAPPS_PKT_SIZE_INC	(-1)
#define MVAPPS_PKT_SIZE_RAND	(-2)
#define MVAPPS_PKT_SIZE_IMIX	(-3)

#define MVAPPS_PLD_MIN_SIZE	8
#define MVAPPS_PLD_WATERMARK	0xfefafefa

/* JSON Serializatiion definitions */
#define SER_FILE_VAR_DIR	"/tmp/"
#define SER_FILE_NAME_PREFIX	"musdk-serial-cfg"
#define SER_MAX_FILE_NAME	64
#define SER_MAX_FILE_SIZE	(30 * 1024)


/* Default MTU */
#define DEFAULT_MTU			1500
/* VLAN header length */
#define VLAN_HLEN			4
/* Ethernet header length */
#define ETH_HLEN			14
/* Ethernet address length */
#define ETH_ALEN			6
/* Ethernet FCS length */
#define ETH_FCS_LEN			4
/* Ethernet IPG length */
#define ETH_IPG_LEN			20


/* IP version 4 */
#define IP_VERSION_4		4
/* IPv6 version */
#define IP_VERSION_6		6
/* Returns IPv4 version */
#define IPV4_HDR_VER(ver_ihl)	(((ver_ihl) & 0xf0) >> 4)
/* IPv6 address length in bytes */
#define IPV6_ADDR_LEN		16

#define IPV4_HDR_LEN		20

#define UDP_HDR_LEN		8

/* Packet Header defines */
#define IP_PROTOCOL_TCP		0x06
#define IP_PROTOCOL_UDP		0x11

/* Macro to convert MTU to MRU */
#define MVAPPS_MTU_TO_MRU(mtu) \
	((mtu) + MV_MH_SIZE + VLAN_HLEN + \
	ETH_HLEN + ETH_FCS_LEN)

/* Macro to convert MRU to MTU */
#define MVAPPS_MRU_TO_MTU(mru) \
	((mru) - MV_MH_SIZE - VLAN_HLEN - \
	ETH_HLEN - ETH_FCS_LEN)

/* GNU flavor of num_cpus */
#define system_ncpus()		get_nprocs()


extern uintptr_t cookie_high_bits;


enum pp2_op_mode_type {
	PP2_OP_MODE_SINGLE_PROCESS = 0,		/* App operational mode is single process, only one process allowed*/
	PP2_OP_MODE_MASTER,			/* App operational mode is master */
	PP2_OP_MODE_GUEST,			/* App operational mode is guest */
	PP2_OP_MODE_NMP_MASTER,			/* App operational mode is master and NMP is started*/
	PP2_OP_MODE_NMP_GUEST			/* App operational mode is NMP guest */
};

struct glb_common_args {
	u64			cores_mask;
	u64			qs_map;
	int			prefetch_shift;
	int			num_ports;
	int			echo;
	u16			burst;
	int			cpus;
	int			affinity;
	int			num_cpu_hash_qs;
	int			verbose;
	int			cli;
	int			(*cli_unregister_cb)(void *);
	int			qs_map_shift;
	u16			pkt_offset; /* Zero maintains default pkt_offset */
	u16			mtu;
	int			ctrl_thresh;
	struct timeval		ctrl_trd_last_time;
	u64			last_rx_cnt;
	u64			last_tx_cnt;
	u64			last_enc_cnt;
	u64			last_dec_cnt;
	u32			busy_wait;
	pthread_mutex_t		thread_lock; /* General Purpose Lock, not intended for data-path */
	bool			shared_hifs; /* Indicates system has shared hifs. */
	struct local_arg	*largs[MVAPPS_MAX_NUM_CORES];
	void			*plat;
	u32			op_mode;
	u32			guest_id;
	char			nmp_cfg_location[SER_MAX_FILE_NAME];
	struct mv_sys_dma_mem_region *mem_region[MVAPPS_MAX_NUM_AP];
	int			num_mem_regions;
	int			num_clusters;
	int			port_forwarding;
	int			min_sg_frag;
	int			max_sg_frag;
	int			core_load;
};

struct buffer_desc {
	void		*virt_addr;
	dma_addr_t	 phy_addr;
	u16		 size;
	u16		 res;
};

struct ip_range {
	u32 start, end, curr; /* same as struct in_addr */
	u16 port0, port1, port_curr;
};

struct mac_range {
	eth_addr_t start;
	eth_addr_t end;
};

struct perf_cmn_cntrs {
	u64			rx_cnt;
	u64			tx_cnt;
	u64			enc_cnt;
	u64			dec_cnt;
	u64			drop_cnt;
};
struct local_common_args {
	u64			qs_map;
	int			prefetch_shift;
	int			num_ports;
	int			id;
	int			echo;
	u16			burst;
	int			verbose;
	u32			busy_wait;
	struct perf_cmn_cntrs	perf_cntrs;
	struct glob_arg		*garg;
	void			*plat;
	int			min_sg_frag;
	int			max_sg_frag;
};


/*
 * Swap source and destination MAC addresses
 */
static inline void swap_l2(char *buf)
{
	u16 *eth_hdr;

	register u16 tmp;

	eth_hdr = (uint16_t *)buf;
	tmp = eth_hdr[0];
	eth_hdr[0] = eth_hdr[3];
	eth_hdr[3] = tmp;
	tmp = eth_hdr[1];
	eth_hdr[1] = eth_hdr[4];
	eth_hdr[4] = tmp;
	tmp = eth_hdr[2];
	eth_hdr[2] = eth_hdr[5];
	eth_hdr[5] = tmp;
}

/*
 * Swap source and destination IP addresses
 */
static inline void swap_l3(char *buf)
{
	register u32 tmp32;

	buf += 14 + 12;
	tmp32 = ((uint32_t *)buf)[0];
	((uint32_t *)buf)[0] = ((uint32_t *)buf)[1];
	((uint32_t *)buf)[1] = tmp32;
}

/*
 * app_get_line()
 * get input from stdin into buffer and according to it
 * create argc and argv, which need while calling for getopt
 */
static inline int app_get_line(char *prmpt, char *buff, size_t sz, int *argc, char *argv[])
{
	int ch, extra;
	char *p2;

	/* because getopt starting parsing from argument = 1 we are skipping argument zero */
	*argc = 1;

	/* Get line with buffer overrun protection */
	if (prmpt) {
		printf("%s", prmpt);
		fflush(stdout);
	}
	if (!fgets(buff, sz, stdin))
		return -EINVAL;

	/*
	 * if it was too long, there'll be no newline. In that case, we flush
	 * to end of line so that excess doesn't affect the next call.
	 */
	if (buff[strlen(buff) - 1] != '\n') {
		extra = 0;
		while (((ch = getchar()) != '\n') && (ch != EOF))
		extra = 1;
		return (extra == 1) ? -EFAULT : 0;
	}

	/* otherwise remove newline and give string back to caller */
	buff[strlen(buff) - 1] = '\0';
	pr_info("input: %s\n", buff);
	sleep(1);

	p2 = strtok(buff, " ");
	while (p2 && *argc < sz - 1) {
		argv[(*argc)++] = p2;
		p2 = strtok(NULL, " ");
	}
	argv[*argc] = NULL;

	return 0;
}

static inline uintptr_t app_get_high_addr(void)
{
	return cookie_high_bits;
}

static inline void app_set_high_addr(uintptr_t high_addr)
{
	cookie_high_bits = high_addr;
}


int apps_perf_dump(struct glb_common_args *cmn_args);
int app_ctrl_cb(void *arg);
int apps_prefetch_cmd_cb(void *arg, int argc, char *argv[]);
void app_print_horizontal_line(u32 char_count, const char *char_val);
int apps_cores_mask_create(int cpus, int affinity);
int apps_thread_to_cpu(struct glb_common_args *cmn_args, int thread);
int apps_cpu_to_thread(struct glb_common_args *cmn_args, int cpu);
int app_parse_mac_address(char *buf, u8 *macaddr_parts);
int app_range_validate(int value, int min, int max);

int app_build_pkt_pool(void			**mem,
		       struct buffer_desc	*buffs,
		       u16			 num_buffs,
		       u16			 min_pkt_size,
		       u16			 max_pkt_size,
		       int			 pkt_size,
		       struct ip_range		*src_ipr,
		       struct ip_range		*dst_ipr,
		       eth_addr_t		 src_mac,
		       eth_addr_t		 dst_mac
		       );

#endif /*__MVUTILS_H__*/

