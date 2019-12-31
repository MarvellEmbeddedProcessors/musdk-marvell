/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#ifndef _MV_NMP_INIT_H
#define _MV_NMP_INIT_H

#include "env/mv_sys_event.h"
#include "drivers/mv_giu.h"

/** @addtogroup grp_nmp_init Networking Mgmt Proxy Init
 *
 *  Networking Management Proxy (NMP) Initialization API
 *  documentation
 *
 *  @{
 */

#define NMP_MAX_NUM_CONTAINERS		4
#define NMP_MAX_NUM_LFS			8
#define NMP_LF_MAX_NUM_LCL_BPOOLS	3
#define NMP_LF_MAX_NUM_TCS		8
#define NMP_MAX_CMD_MSG_SIZE		128

/* nmp handler declaration */
struct nmp;

/* nmp parameters definition */

/**
 * nmp logical function types
 *
 */
enum nmp_lf_type {
	NMP_LF_T_NIC_NONE = 0,
	NMP_LF_T_NIC_PF,		/**< Logical function of type physcal function */
	NMP_LF_T_NIC_VF,		/**< Logical function of type virtual function */
	NMP_LF_T_NIC_LAST
};

/**
 * nmp physical function nic types
 *
 */
enum nmp_lf_nicpf_type {
	NMP_LF_NICPF_T_NONE = 0,
	NMP_LF_NICPF_T_PP2_PORT,	/**< nic_pf of type PP2_PORT */
	NMP_LF_NICPF_T_PP2_LAG,		/**< nic_pf of type PP2_LAG */
	NMP_LF_NICPF_T_LAST
};

/**
 * NMP pp2 parameters structure
 *
 */
struct nmp_pp2_params {
	u16 bm_pool_reserved_map;	/**< pp2 bpool reserved map */
};

/**
 * NMP bpool parameters structure
 *
 */
struct nmp_lf_bpool_params {
	u16 max_num_buffs;		/**< maximum number of buffers in the bpool */
	u16 buff_size;			/**< bpool buffer size */
};

/**
 * NMP pp2 port parameters structure
 *
 */
struct nmp_lf_nicpf_pp2_port_params {
	char *match;			/**< matching ppio name */
	u8 lcl_num_bpools;		/**< number of pools in pp2 ppio */
	/** local bpools parameters for each bpool in the ppio */
	struct nmp_lf_bpool_params lcl_bpools_params[NMP_LF_MAX_NUM_LCL_BPOOLS];
};

/**
 * NMP Physical function structure
 *
 */
struct nmp_lf_nicpf_params {
	char *match;				/**< matching gpio name */
	u32 keep_alive_thresh;			/**< for disabling this feature use '0'; otherwise, this value reflect
						 * the number of times nmp_schedule should be called before sending the
						 * keep-alive msg
						 */
	int pci_en;				/**< Flag inidicating PCI interface is present*/
	u16 lcl_egress_qs_size;			/**< local egress queue size */
	u16 lcl_ingress_qs_size;		/**< local ingress queue size */
	u16 dflt_pkt_offset;			/**< default packet offset */
	u8 max_num_tcs;				/**< maximum number of TC's */
	u8 lcl_num_bpools;			/**< local number of pools for GIU*/
	/** local bpools parameters for each bpool in GIU */
	struct nmp_lf_bpool_params lcl_bpools_params[NMP_LF_MAX_NUM_LCL_BPOOLS];
	enum nmp_lf_nicpf_type type;		/**< Type of nic pf (pp2_port, pp2_lag, etc) */
	union {
		/** nic physical function pp2_port parameters */
		struct nmp_lf_nicpf_pp2_port_params pp2_port;
	} port_params;
};

/**
 * NMP Virtual function structure
 *
 */
struct nmp_lf_nicvf_params {
	char *match;				/**< matching gpio name */
	u32 keep_alive_thresh;			/**< for disabling this feature use '0'; otherwise, this value reflect
						 * the number of times nmp_schedule should be called before sending the
						 * keep-alive msg
						 */
	int pci_en;				/**< Flag inidicating PCI interface is present*/
	u16 lcl_egress_qs_size;			/**< local egress queue size */
	u16 lcl_ingress_qs_size;		/**< local ingress queue size */
	u16 dflt_pkt_offset;			/**< default packet offset */
	u8 max_num_tcs;				/**< maximum number of TC's */
	u8 lcl_num_bpools;			/**< local number of pools for GIU*/
	/** local bpools parameters for each bpool in GIU */
	struct nmp_lf_bpool_params lcl_bpools_params[NMP_LF_MAX_NUM_LCL_BPOOLS];
};

/**
 * NMP logical function structure
 *
 */
struct nmp_lf_params {
	enum nmp_lf_type type;		/**< Type of logical function (Virtual function, physcal function, etc)*/
	union {
		/** Logical function layer parameters for Physical Function type */
		struct nmp_lf_nicpf_params nicpf;
		struct nmp_lf_nicvf_params nicvf;
	} u;
};

/**
 * NMP container structure
 *
 */
struct nmp_container_params {
	u8 num_lfs;		/**< Number of NMP logical function layers*/
	/** Parameters relevant to the initialization of the NMP logical function layer
	 * including MUSDK networking drivers like MVPP2, MVSAM, MVGIU)
	 */
	struct nmp_lf_params *lfs_params;
	u8 guest_id;		/**< entity with visibility to all the container's data and cntrl traffic */
				/**< if no guest are using this container, use "0"; other wise, use >1 value */
};

/**
 * NMP DMA-engines information
 *
 */
struct nmp_dma_engines_params {
	u8 num_engines;		/**< Number of DMA engines*/
	char engine_name[GIU_ENG_OUT_OF_RANGE][16];
};

/**
 * nmp scheduling types
 *
 */
enum nmp_sched_type {
	NMP_SCHED_RX = 0,
	NMP_SCHED_TX,
	NMP_SCHED_MNG
};

struct nmp_params {
	int pp2_en;		/**< Flag inidicating PP2 interface is present*/
	/** pp2_params is used for initializing pp2 interface (pp2_init)
	 * relevant only when pp2_en is set
	 */
	struct nmp_pp2_params pp2_params;
	/** NMP may have several containers, each one representing a user process/VM/container */
	u8 num_containers;
	struct nmp_container_params *containers_params;
	struct nmp_dma_engines_params dma_eng_params;
};

struct nmp_event_params {
	u32 pkt_coal;
	u32 usec_coal;
	u32 tc_mask;
};

/**
 * Initialize the NMP.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 */
int nmp_init(struct nmp_params *params, struct nmp **nmp);

/**
 * Trigger an NMP scheduling loop
 *
 * @param[in]	nmp		A pointer to a NMP object.
 * @param[in]	nmp_sched_type
 * @param[out]	pending		Get how many DMA jobs are still waiting for processing

 * @retval	0 on success
 * @retval	<0 on failure
 */
int nmp_schedule(struct nmp *nmp, enum nmp_sched_type, u16 *pending);

/**
 * Create a NMP (GIE) scheduling event
 *
 * The event API is called to create a sys_event for a GIE, that
 * can later be polled through the mv_sys_event_poll() API.
 * This is only releavnt to 'NMP_SCHED_TX'
 *
 * @param[in]	nmp		A pointer to a NMP object.
 * @param[in]	params		Parameters for the event.
 * @param[out]	ev		A pointer to event handle of type 'struct mv_sys_event *'.
 *
 * @retval      0 on success
 * @retval      <0 on failure
 */
int nmp_create_scheduling_event(struct nmp *nmp, struct nmp_event_params *params, struct mv_sys_event **ev);

/**
 * Delete a NMP(GIE) scheduling event
 *
 * @param[in]	ev		A sys_event handle.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 */
int nmp_delete_scheduling_event(struct mv_sys_event *ev);

/**
 * Set a NMP (GIE) scheduling event
 *
 * The set_event API is called to enable the creation of events for the related NMP.
 *
 * @param[in]	ev		A sys_event handle.
 * @param[in]	en		enable/disable
 *
 * @retval      0 on success
 * @retval      <0 on failure
 */
int nmp_set_scheduling_event(struct mv_sys_event *ev, int en);

/**
 * Read nmp-params from a config file
 *
 * This helper API can be used to build nmp-params from an input cfg file (which MUST be given).
 *
 * @param[in]	cfg_file	file location.
 * @param[out]	params		builded nmp-parameters
 *
 * @retval      0 on success
 * @retval      <0 on failure
 */
int nmp_read_cfg_file(char *cfg_file, struct nmp_params *params);

/** @} */ /* end of grp_nmp_init */

#endif /* _MV_NMP_INIT_H */

