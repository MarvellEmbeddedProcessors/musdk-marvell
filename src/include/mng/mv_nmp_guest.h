/*******************************************************************************
 * Copyright (C) Marvell International Ltd. and its affiliates
 *
 * This software file (the "File") is owned and distributed by Marvell
 * International Ltd. and/or its affiliates ("Marvell") under the following
 * alternative licensing terms.  Once you have made an election to distribute the
 * File under one of the following license alternatives, please (i) delete this
 * introductory statement regarding license alternatives, (ii) delete the three
 * license alternatives that you have not elected to use and (iii) preserve the
 * Marvell copyright notice above.
 *
 ********************************************************************************
 * Marvell Commercial License Option
 *
 * If you received this File from Marvell and you have entered into a commercial
 * license agreement (a "Commercial License") with Marvell, the File is licensed
 * to you under the terms of the applicable Commercial License.
 *
 ********************************************************************************
 * Marvell GPL License Option
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File in accordance with the terms and conditions of the General
 * Public License Version 2, June 1991 (the "GPL License"), a copy of which is
 * available along with the File in the license.txt file or by writing to the Free
 * Software Foundation, Inc., or on the worldwide web at http://www.gnu.org/licenses/gpl.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY
 * DISCLAIMED.  The GPL License provides additional details about this warranty
 * disclaimer.
 *
 ********************************************************************************
 * Marvell GNU General Public License FreeRTOS Exception
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File in accordance with the terms and conditions of the Lesser
 * General Public License Version 2.1 plus the following FreeRTOS exception.
 * An independent module is a module which is not derived from or based on
 * FreeRTOS.
 * Clause 1:
 * Linking FreeRTOS statically or dynamically with other modules is making a
 * combined work based on FreeRTOS. Thus, the terms and conditions of the GNU
 * General Public License cover the whole combination.
 * As a special exception, the copyright holder of FreeRTOS gives you permission
 * to link FreeRTOS with independent modules that communicate with FreeRTOS solely
 * through the FreeRTOS API interface, regardless of the license terms of these
 * independent modules, and to copy and distribute the resulting combined work
 * under terms of your choice, provided that:
 * 1. Every copy of the combined work is accompanied by a written statement that
 * details to the recipient the version of FreeRTOS used and an offer by yourself
 * to provide the FreeRTOS source code (including any modifications you may have
 * made) should the recipient request it.
 * 2. The combined work is not itself an RTOS, scheduler, kernel or related
 * product.
 * 3. The independent modules add significant and primary functionality to
 * FreeRTOS and do not merely extend the existing functionality already present in
 * FreeRTOS.
 * Clause 2:
 * FreeRTOS may not be used for any competitive or comparative purpose, including
 * the publication of any form of run time or compile time metric, without the
 * express permission of Real Time Engineers Ltd. (this is the norm within the
 * industry and is intended to ensure information accuracy).
 *
 ********************************************************************************
 * Marvell BSD License Option
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File under the following licensing terms.
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *	* Redistributions of source code must retain the above copyright notice,
 *	  this list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the distribution.
 *
 *	* Neither the name of Marvell nor the names of its contributors may be
 *	  used to endorse or promote products derived from this software without
 *	  specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************/

#ifndef _MV_NMP_GUEST_H
#define _MV_NMP_GUEST_H

#include "mv_std.h"
#include "mv_nmp_guest_msg.h"

/** @addtogroup grp_nmp_guest_init NMP Guest
 *
 *  Networking Management Proxy (NMP) Guest API documentation
 *
 *  @{
 */

/* nmp_guest handler declaration */
struct nmp_guest;

struct nmp_guest_bpool_info {
	char	bpool_name[20];
};

struct nmp_guest_port_info {
	char		port_name[20];
	u32		num_bpools;
	struct nmp_guest_bpool_info	*bpool_info;
};

struct nmp_guest_module_info {
	u32		num_ports;
	struct nmp_guest_port_info *port_info;
};

struct nmp_guest_info {
	struct nmp_guest_module_info ports_info;
	u8	num_giu_ports;
	struct nmp_guest_port_info *giu_info;
};

/**
 * NMP Guest initialization params
 *
 */
struct nmp_guest_params {
	/** the guest ID; this should be aligned with the guest-ID registered in the
	* NMP JSON parameters file
	*/
	u8 id;
	u32 timeout;		/**< timeout in mili-secs */
	u32 keep_alive_thresh;	/**< if '0' nmp_guest will not send keep-alive msg by itself, but nmp_guest_send_ka_msg
				 * can be called explicitly for generating such a msg; otherwise, this value reflects
				 * the number of times nmp_guest_schedule should be called before sending the
				 * keep-alive msg
				 */
	void *nmp;	/**< the NMP structure should be passed in case the NMP and the NMP-guest
				 * are runningin the same process.
				 */
};

/**
 * Initialize the NMP Guest.
 *
 * @param[in]	params		pointer to structure containing NMP guest config parameters.
 * @param[out]	g		pointer to opaque guest handle of type 'struct nmp_guest *'
 *
 *	@retval	0 on success
 *	@retval	<0 on failure
 */
int nmp_guest_init(struct nmp_guest_params *params, struct nmp_guest **g);

/**
 * De-initialize the NMP Guest.
 *
 * @param[in]	g		pointer to a guest handle.
 *
 *	@retval	0 on success
 *	@retval	<0 on failure
 */
void nmp_guest_deinit(struct nmp_guest *g);

/**
 * Get the NMP guest probe string.
 *
 * this API should be called by the user application in order to retrieve the probing string that was extracted
 * from the guest JSON file. the application shall use this string in order to extract and execute init calls
 * to all registered objects (e.g. pp2-bpool, giu-gpio, etc.).
 *
 * @param[in]	g		pointer to a guest handle.
 * @param[out]	prb_str		pointer to the guest string containing the serialized data.
 *
 *	@retval	0 on success
 *	@retval	<0 on failure
 */
int nmp_guest_get_probe_str(struct nmp_guest *g, char **prb_str);

/**
 * Get the NMP guest relations information.
 *
 * @param[in]	g		pointer to a guest handle.
 * @param[out]	guest_info	pointer to the guest relations information.
 *
 *	@retval	0 on success
 *	@retval	<0 on failure
 */
int nmp_guest_get_relations_info(struct nmp_guest *g, struct nmp_guest_info *guest_info);

/****************************************************************************
 *	Run-time API
 ****************************************************************************/

#define NMP_GUEST_EV_CUSTOM		0x1LL
#define NMP_GUEST_EV_NICPF_MTU		0x2LL
#define NMP_GUEST_EV_NICPF_MAC_ADDR	0x3LL

/**
 * NMP Guest LF Type
 *
 */
enum nmp_guest_lf_type {
	/** TODO */
	NMP_GUEST_LF_T_NONE = 0,
	/** TODO */
	NMP_GUEST_LF_T_NICPF,
	/** TODO */
	NMP_GUEST_LF_T_NICVF,
	/** TODO */
	NMP_GUEST_LF_T_CUSTOM,
	/** TODO */
	NMP_GUEST_LF_T_LAST
};

/**
 * Register the NMP guest event handler.
 *
 * this API should be called by the user application (may be called several times)
 * in order to register events that this guest would like to listen.
 *
 * @param[in]	g		pointer to a guest handle.
 * @param[in]	lf_type		LF type for notifications from non-CUSTOM LF
 * @param[in]	lf_id		LF id
 * @param[in]	ev_mask		event mask.
 * @param[in]	client		dispatcher client.
 * @param[in]	guest_ev_cb	event callback function.
 *
 *	@retval	0 on success
 *	@retval	<0 on failure
 */
int nmp_guest_register_event_handler(struct nmp_guest *g,
				     enum nmp_guest_lf_type lf_type,
				     u8 lf_id,
				     u64 ev_mask,
				     void *arg,
				     int (*guest_ev_cb)(void *arg, enum nmp_guest_lf_type client, u8 id, u8 code,
							u16 indx, void *msg, u16 len));

/**
 * NMP Guest schedule.
 *
 * this API should be called by the guest application in order to check the guest message
 * queue for incoming messages (inband-management commands)
 *
 * @param[in]	g		pointer to a guest handle.
 *
 *	@retval	0 on success
 *	@retval	<0 on failure
 */
int nmp_guest_schedule(struct nmp_guest *g);

/**
 * NMP Guest message send.
 *
 * @param[in]	g		pointer to a guest handle.
 * @param[in]	code		message code.
 * @param[in]	indx		message index. '0' is invalid, 0xFFFF is for notifications
 * @param[in]	msg		pointer to the message.
 * @param[in]	len		message length.
 *
 *	@retval	0 on success
 *	@retval	<0 on failure
 */
int nmp_guest_send_msg(struct nmp_guest *g, u8 code, u16 indx, void *msg, u16 len);

/**
 * NMP Guest keep-alive message send.
 *
 * @param[in]	g		pointer to a guest handle.
 *
 *	@retval	0 on success
 *	@retval	<0 on failure
 */
int nmp_guest_send_ka_msg(struct nmp_guest *g);

#ifdef MVCONF_NMP_BUILT
/**
 * Set NMP handle. Should be used if both NMP and NMP-Guest are running on the same process
 *
 * @param[in]	g		pointer to a guest handle.
 * @param[in]	nmp		pointer to a nmp handle.
 *
 */
void nmp_guest_set_nmp(struct nmp_guest *g, void *nmp);
#endif /* MVCONF_NMP_BUILT */

/** @} */ /* end of grp_nmp_guest_init */

#endif /* _MV_NMP_GUEST_H */

