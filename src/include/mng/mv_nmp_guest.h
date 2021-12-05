/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

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

