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

#ifndef _PF_PROFILE_H
#define _PF_PROFILE_H

#define PROFILE_NAME_LEN (32)
#define PROFILE_DESC_LEN (128)


/* SNIC PF / VF Instance profile
 *
 * name               - profile name
 * desc               - profile name
 * lcl_egress_q_num   - number of local egress data queues
 * lcl_egress_q_size  - size of local egress data queue
 * lcl_ingress_q_num  - number of local ingress data queues
 * lcl_ingress_q_size - size of local ingress data queues
 * lcl_bm_q_num       - number of local bm queues
 * lcl_bm_q_size      - size of bm queue
 */
struct pf_profile {
	char name[PROFILE_NAME_LEN];
	char desc[PROFILE_DESC_LEN];

	u32 lcl_egress_q_num;
	u32 lcl_egress_q_size;
	u32 lcl_ingress_q_num;
	u32 lcl_ingress_q_size;
	u32 lcl_bm_q_num;
	u32 lcl_bm_q_size;
	u32 lcl_bm_buf_size;

};

#endif /* _PF_PROFILE_H */

