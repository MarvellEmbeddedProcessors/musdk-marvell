/******************************************************************************
 *	Copyright (C) 2017 Marvell International Ltd.
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

#ifndef _MV_NMP_INIT_H
#define _MV_NMP_INIT_H

/** @addtogroup grp_nmp_init Networking Mgmt Proxy Init
 *
 *  Networking Management Proxy (NMP) Initialization API
 *  documentation
 *
 *  @{
 */

#define NMP_LF_MAX_NUM_LCL_BPOOLS	3
#define NMP_LF_MAX_NUM_CONTAINERS	4
#define NMP_LF_MAX_NUM_LFS		8
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
 * NMP logical function structure
 *
 * TODO: rename to .nmp_lf_nicpf_params. Left for backwards compatibility until API is updated in all apps
 */
struct nmp_lf_nicpf_params {
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
 * NMP logical function structure
 *
 * TODO: rename to .nmp_lf_params. Left for backwards compatibility until API is updated in all apps
 */
struct nmp_lf_params {
	enum nmp_lf_type type;		/**< Type of logical function (Virtual function, physcal function, etc)*/
	union {
		/** Logical function layer parameters for Physical Function type */
		struct nmp_lf_nicpf_params nicpf;
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
				/**< if no guest are using this container, use ‘0’; other wise, use >1 value */
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
};

/**
 *	Initialize the NMP.
 *
 *	@retval	0 on success
 *	@retval	<0 on failure
 */
int nmp_init(struct nmp_params *params, struct nmp **nmp);

/**
 *	Trigger an NMP scheduling loop
 *
 *	@retval	0 on success
 *	@retval	<0 on failure
 */
int nmp_schedule(struct nmp *nmp, enum nmp_sched_type);

/** @} */ /* end of grp_nmp_init */

#endif /* _MV_NMP_INIT_H */

