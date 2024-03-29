/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __MV_NETA_UTILS_H__
#define __MV_NETA_UTILS_H__

#include "mv_std.h"
#include "mv_net.h"
#include "mv_neta_ppio.h"
#include "utils.h"

/* pkt offset, must be multiple of 32 bytes */
#define MVAPPS_NETA_PKT_OFFS			64
/* pkt offset including Marvell header */
#define MVAPPS_NETA_PKT_EFEC_OFFS		(MVAPPS_NETA_PKT_OFFS + MV_MH_SIZE)
/* Maximum number of CPU cores used by application */
#define MVAPPS_NETA_MAX_NUM_CORES		2
/* Maximum number of ports used by application */
#define MVAPPS_NETA_MAX_NUM_PORTS		2
/* Maximum number of queues per TC */
#define MVAPPS_NETA_MAX_NUM_QS_PER_TC	MVAPPS_NETA_MAX_NUM_CORES


/*
 * Tx shadow queue entry
 */
struct tx_shadow_q_entry {
	struct neta_buff_inf	buff_ptr;	/* pointer to the buffer object */
	u8			rx_port;
	u8			rx_queue;
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
	int			 ppio_id;	/* PPIO port ID */
	u32			 first_inq;	/* First RXQ - relative to the Port's first RXQ */
	u16			 num_tcs;	/* Number of TCs */
	u16			 num_inqs[MVAPPS_NETA_MAX_NUM_QS_PER_TC]; /* Number of Rx queues */
	u16			 num_outqs;	/* Number of Tx queues */
	u32			 inq_size;	/* Rx queue size */
	u32			 outq_size;	/* Tx queue size */
	struct neta_ppio	*ppio;		/* PPIO object returned by neta_ppio_init() */
	struct neta_ppio_params	 port_params;	/* PPIO configuration parameters */

};

struct neta_counters {
	u32		rx_buf_cnt;
	u32		free_buf_cnt;
	u32		tx_buf_cnt;
	u32		tx_buf_drop;
	u64		tx_bytes_cnt;
	u64		rx_bytes_cnt;
};

/*
 * Local thread port parameters
 */
struct lcl_port_desc {
	int			 lcl_id;	/* Local thread ID*/
	int			 port_id;	/* Packet Processor ID */
	struct neta_ppio	*ppio;		/* PPIO object returned by pp2_ppio_init() */
	int			 num_shadow_qs;	/* Number of Tx shadow queues */
	int			 shadow_q_size;	/* Size of Tx shadow queue */
	struct tx_shadow_q	*shadow_qs;	/* Tx shadow queue */
	struct neta_counters	 cntrs;		/* Port statistics */
};

/* Macroes to handle counters */
#define INC_RX_BYTES_COUNT(lcl_port_desc, cnt)	((lcl_port_desc)->cntrs.rx_bytes_cnt += cnt)
#define INC_TX_BYTES_COUNT(lcl_port_desc, cnt)	((lcl_port_desc)->cntrs.tx_bytes_cnt += cnt)
#define INC_RX_COUNT(lcl_port_desc, cnt)	((lcl_port_desc)->cntrs.rx_buf_cnt += cnt)
#define INC_TX_COUNT(lcl_port_desc, cnt)	((lcl_port_desc)->cntrs.tx_buf_cnt += cnt)
#define INC_TX_DROP_COUNT(lcl_port_desc, cnt)	((lcl_port_desc)->cntrs.tx_buf_drop += cnt)
#define INC_FREE_COUNT(lcl_port_desc, cnt)	((lcl_port_desc)->cntrs.free_buf_cnt += cnt)

/*
 * Parse port pp_id and ppio_id from port name
 */
int app_find_port_info(struct port_desc *port_desc);

/*
 * Init port, This function doesn't enable port.
 */
int app_port_init(struct port_desc *port, u16 mtu, u16 pkt_offset);

/*
 * Enable port.
 */
int app_port_enable(struct port_desc *port);

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
void app_disable_all_ports(struct port_desc *ports, int num_ports);

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
int app_register_cli_common_cmds(struct glb_common_args *glb_args);
int register_cli_ftr_cmds(struct glb_common_args *garg);

/* Saved sysdma virtual high address*/
extern u64 neta_sys_dma_high_addr;

#endif /*__MVUTILS_H__*/

