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

/* nmp handler declaration */
struct nmp;

/* nmp parameters definition */

/* nmp logical functional type */
enum nmp_lf_type {
	NMP_LF_T_NONE = 0,
	NMP_LF_T_PF,
	NMP_LF_T_VF,
	NMP_LF_T_LAST,
};

/* nmp scheduling options */
enum nmp_sched_type {
	NMP_SCHED_RX = 0,
	NMP_SCHED_TX,
	NMP_SCHED_MNG
};

/* nmp PF parameters */
struct nmp_lf_pf_params {
	enum nmp_lf_type type;
	int  pci_en;
	int  lcl_egress_q_num;
	int  lcl_egress_q_size;
	int  lcl_ingress_q_num;
	int  lcl_ingress_q_size;
	int  lcl_bm_q_num;
	int  lcl_bm_q_size;
	int  lcl_bm_buf_size;
};

/* nmp union for logical functions types */
union nmp_lf_params {
	struct nmp_lf_pf_params pf;
};

struct nmp_params {
	u8     num_lfs;
	union nmp_lf_params *lfs_params;
};

/**
 *	Initialize the NMP.
 *
 *	@retval	0 on success
 *	@retval	<0 on failure
 */
int nmp_init(struct nmp_params *params, struct nmp **nmp);

/**
 *	Start NMP scheduling.
 *
 *	@retval	0 on success
 *	@retval	<0 on failure
 */
int nmp_schedule(struct nmp *nmp, enum nmp_sched_type);

/** @} */ /* end of grp_nmp_init */

#endif /* _MV_NMP_INIT_H */

