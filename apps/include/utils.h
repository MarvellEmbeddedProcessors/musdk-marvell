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

#ifndef __MVUTILS_H__
#define __MVUTILS_H__

#include "mv_std.h"
#include "mv_pp2_hif.h"
#include "mv_pp2_ppio.h"

/* pkt offset, must be multiple of 32 bytes */
#define MVAPPS_PKT_OFFS			64
/* pkt offset including Marvell header */
#define MVAPPS_PKT_EFEC_OFFS		(MVAPPS_PKT_OFFS + PP2_MH_SIZE)
/* Maximum number of packet processors used by application */
#define MVAPPS_MAX_PKT_PROC		2
/* Maximum number of CPU cores used by application */
#define MVAPPS_MAX_NUM_CORES		4
/* Maximum number of ports used by application */
#define MVAPPS_MAX_NUM_PORTS		2

/* Maximum number of queues per TC */
#define MVAPPS_MAX_NUM_QS_PER_TC	MVAPPS_MAX_NUM_CORES
/* Total number of BM pools supported */
#define MVAPPS_PP2_TOTAL_NUM_BPOOLS	(PP2_PPIO_TC_MAX_POOLS * PP2_PPIO_MAX_NUM_TCS)
/* Number of BM pools reserved by kernel */
#define MVAPPS_PP2_NUM_BPOOLS_RSRV	3
/* Reserved BM pools mask */
#define MVAPPS_PP2_BPOOLS_RSRV		((1 << MVAPPS_PP2_NUM_BPOOLS_RSRV) - 1)
/* Maximum number of pools per packet processor */
#define MVAPPS_PP2_MAX_NUM_BPOOLS	min(PP2_PPIO_TC_MAX_POOLS * PP2_PPIO_MAX_NUM_TCS, \
					MVAPPS_PP2_TOTAL_NUM_BPOOLS - MVAPPS_PP2_NUM_BPOOLS_RSRV)
/* Total number of HIFs supported */
#define MVAPPS_PP2_TOTAL_NUM_HIFS	9 /* PP2_NUM_REGSPACES - move to API h file */
/* Number of HIFs reserved by kernel */
#define MVAPPS_PP2_NUM_HIFS_RSRV	4
/* Reserved HIFs mask */
#define MVAPPS_PP2_HIFS_RSRV		((1 << MVAPPS_PP2_NUM_HIFS_RSRV) - 1)

/** Get rid of path in filename - only for unix-type paths using '/' */
#define MVAPPS_NO_PATH(file_name)	(strrchr((file_name), '/') ? \
					 strrchr((file_name), '/') + 1 : (file_name))

/* Maximum size of port name */
#define MVAPPS_PPIO_NAME_MAX		20

/* Default MTU */
#define DEFAULT_MTU			1500
/* VLAN header length */
#define VLAN_HLEN			4
/* Ethernet header length */
#define ETH_HLEN			14
/* Ethernet FCS length */
#define ETH_FCS_LEN			4

/* Macro to convert MTU to MRU */
#define MVAPPS_MTU_TO_MRU(mtu) \
	((mtu) + PP2_MH_SIZE + VLAN_HLEN + \
	ETH_HLEN + ETH_FCS_LEN)

/* Macro to convert MRU to MTU */
#define MVAPPS_MRU_TO_MTU(mru) \
	((mru) - PP2_MH_SIZE - VLAN_HLEN - \
	ETH_HLEN - ETH_FCS_LEN)

/* sysfs path for reading relevant parameters from kernel driver */
#define PP2_SYSFS_MUSDK_PATH		"/sys/devices/platform/pp2/musdk"
#define PP2_SYSFS_DEBUG_PORT_SET_FILE	"sysfs_current_port"
#define PP2_SYSFS_RX_FIRST_RXQ_FILE	"first_rxq"
#define PP2_SYSFS_RX_NUM_RXQ_FILE	"num_rx_queues"
#define PP2_SYSFS_TX_NUM_TXQ_FILE	"num_tx_queues"

#define PP2_SYSFS_RSS_PATH		"/sys/devices/platform/pp2/rss"
#define PP2_SYSFS_RSS_NUM_TABLES_FILE	"num_rss_tables"

#define PP2_MAX_BUF_STR_LEN	256

/*
 * Tx shadow queue entry
 */
struct tx_shadow_q_entry {
	struct pp2_buff_inf	 buff_ptr;	/* pointer to the buffer object */
	struct pp2_bpool	*bpool;		/* pointer to the bpool object */
};

/*
 * Tx shadow queue
 */
struct tx_shadow_q {
	u16				read_ind;	/* read index */
	u16				write_ind;	/* write index */

	struct tx_shadow_q_entry	*ents;		/* array of entries */
};

/*
 * General port parameters
 */
struct port_desc {
	char			 name[15];	/* Port name */
	int			 initialized;	/* Flag indicated is port was initialized */
	int			 pp_id;		/* Packet Processor ID */
	int			 ppio_id;	/* PPIO port ID */
	enum pp2_ppio_type	 ppio_type;	/* PPIO type */
	u32			 first_inq;	/* First RXQ - relative to the Port's first RXQ */
	u16			 num_tcs;	/* Number of TCs */
	u16			 num_inqs[MVAPPS_MAX_NUM_QS_PER_TC];	/* Number of Rx queues */
	u16			 num_outqs;	/* Number of Tx queues */
	u32			 inq_size;	/* Rx queue size */
	u32			 outq_size;	/* Tx queue size */
	u32			 hash_type;	/* Hash type */
	u32			 first_rss_tbl;	/* First RSS table */
	struct pp2_ppio		 *ppio;		/* PPIO object returned by pp2_ppio_init() */
	struct pp2_ppio_params	 port_params;	/* PPIO configuration parameters */

};

/*
 * Local thread port parameters
 */
struct lcl_port_desc {
	int			 id;		/* Local port ID*/
	int			 lcl_id;	/* Local thread ID*/
	int			 pp_id;		/* Packet Processor ID */
	int			 ppio_id;	/* PPIO port ID */
	struct pp2_ppio		*ppio;		/* PPIO object returned by pp2_ppio_init() */
	int			 num_shadow_qs;	/* Number of Tx shadow queues */
	int			 shadow_q_size;	/* Size of Tx shadow queue */
	struct tx_shadow_q	*shadow_qs;	/* Tx shadow queue */
};

/*
 * BM pool size parameters
 */
struct bpool_inf {
	int	buff_size;	/* buffer size */
	int	num_buffs;	/* number of buffers */
};

/*
 * BM pool parameters
 */
struct bpool_desc {
	struct pp2_bpool	*pool;		/* pointer to the bpool object */
	struct pp2_buff_inf	*buffs_inf;	/* array of buffer objects */
	int			 num_buffs;	/* number of buffers */
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
 * Init HIF object
 */
int app_hif_init(struct pp2_hif **hif, u32 queue_size);
/*
 * Build all pools
 */
int app_build_all_bpools(struct bpool_desc ***ppools, int num_pools, struct bpool_inf infs[], struct pp2_hif *hif);
/*
 * Free all pools
 */
void app_free_all_pools(struct bpool_desc **pools, int num_pools, struct pp2_hif *hif);

/*
 * Parse port pp_id and ppio_id from port name
 */
int app_find_port_info(struct port_desc *port_desc);
/*
 * Init port
 */
int app_port_init(struct port_desc *port, int num_pools, struct bpool_desc *pools, u16 mtu);
/*
 * Init local port object per thread according to port parameters
 */
void app_port_local_init(int id, int lcl_id, struct lcl_port_desc *lcl_port, struct port_desc *port);
/*
 * Deinit all ports
 */
void app_deinit_all_ports(struct port_desc *ports, int num_ports);
/*
 * Deinit local port object
 */
void app_port_local_deinit(struct lcl_port_desc *lcl_port);
/*
 * Disable all ports
 */
void app_disable_all_ports(struct port_desc *ports, int num_ports, u16 num_tcs, u16 num_inqs);

/* TODO: may be need to move to other file */
/*
 * Get line
 */
int app_get_line(char *prmpt, char *buff, size_t sz, int *argc, char *argv[]);

/*
 * Show queue statistics
 */
void app_show_queue_stat(struct port_desc *port_desc, u8 tc, u8 q_start, int num_qs, int reset);
/*
 * Show port statistics
 */
void app_show_port_stat(struct port_desc *port_desc, int reset);
/*
 * Register common CLI commands (currently show queue and port statistics)
 */
int app_register_cli_common_cmds(struct port_desc *port_desc);

/*
 * Get sysfs parameter from kernel driver
 */
u32 appp_pp2_sysfs_param_get(char *if_name, char *file);

/* Saved sysdma virtual high address*/
extern u64 sys_dma_high_addr;

#endif /*__MVUTILS_H__*/

