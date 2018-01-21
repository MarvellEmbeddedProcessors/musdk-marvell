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

#ifndef _MV_NMP_GUEST_H
#define _MV_NMP_GUEST_H

/** @addtogroup grp_nmp_init Networking Mgmt Proxy Init
 *
 *  Networking Management Proxy (NMP) Guest Initialization API
 *  documentation
 *
 *  @{
 */

/* nmp_guest handler declaration */
struct nmp_guest;

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
	NMP_GUEST_LF_T_NONE = 0,
	NMP_GUEST_LF_T_NICPF,
	NMP_GUEST_LF_T_CUSTOM,
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

#endif /* _MV_NMP_GUEST_H */

