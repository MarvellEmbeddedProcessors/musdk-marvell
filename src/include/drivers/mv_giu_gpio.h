/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#ifndef __MV_GIU_GPIO_H__
#define __MV_GIU_GPIO_H__

#include "mv_std.h"
#include "mv_mqa.h"
#include "mv_giu.h"
#include "mv_giu_bpool.h"
#include "env/mv_sys_event.h"

/** @addtogroup grp_giu_io GIU Port: I/O (GP-IO)
 *
 *  GIU Port I/O (GP-IO) API documentation
 *
 *  @{
 */

#define GIU_GPIO_MAX_NUM_TCS		8
#define GIU_GPIO_TC_MAX_NUM_QS		4
#define GIU_GPIO_TC_MAX_NUM_BPOOLS	3

#define GIU_GPIO_DESC_NUM_WORDS		8

#define GIU_GPIO_DESC_PA_WATERMARK	0xcafe0000
#define GIU_GPIO_DESC_COOKIE_WATERMARK	0xcafecafe

/* GPIO Handler */
struct giu_gpio;

/** RSS Hash Type enumaration
 */
enum rss_hash_type {
	RSS_HASH_NONE = 0,
	RSS_HASH_2_TUPLE,	/**< 2-tuple: IP-src, IP-dst */
	RSS_HASH_5_TUPLE,	/**< 2-tuple: IP-src, IP-dst, IP-Prot, L4-src, L4-dst */
	RSS_HASH_OUT_OF_RANGE
};

struct giu_gpio_lcl_q_params {
	u32		 len;
};

struct giu_gpio_rem_q_params {
	u32		 len;
	u32		 size;
	u32		 buff_len;
	void		*host_remap;
	phys_addr_t	 q_base_pa;
	void		*prod_base_va;
	phys_addr_t	 prod_base_pa;
	void		*cons_base_va;
	phys_addr_t	 cons_base_pa;
	u32		 msix_id;
};

/** In TC - Queue topology
 */
struct giu_gpio_intc_params {
	u32				 pkt_offset;
	enum rss_hash_type		 rss_type;
	u32				 num_inqs;
	struct giu_gpio_lcl_q_params	 inqs_params[GIU_GPIO_TC_MAX_NUM_QS];

	u32				 num_inpools;
	struct giu_bpool		*pools[GIU_GPIO_TC_MAX_NUM_BPOOLS];

	u32				 num_rem_outqs;
	struct giu_gpio_rem_q_params	 rem_outqs_params[GIU_GPIO_TC_MAX_NUM_QS];
};

struct giu_gpio_rem_inq_params {
	struct giu_gpio_rem_q_params	 q_params;
	struct giu_gpio_rem_q_params	 poolq_params;
};

/** Out TC - Queue topology
 */
struct giu_gpio_outtc_params {
	u32				 num_outqs;
	struct giu_gpio_lcl_q_params	 outqs_params[GIU_GPIO_TC_MAX_NUM_QS];

	u32				 rem_pkt_offset;
	enum rss_hash_type		 rem_rss_type;
	u32				 num_rem_inqs;
	struct giu_gpio_rem_inq_params	 rem_inqs_params[GIU_GPIO_TC_MAX_NUM_QS];
};

struct giu_gpio_params {
	/** Used for DTS acc to find appropriate "physical" GP-IO obj;
	 * E.g. "gpio-0:0" means GIU[0],port[0]
	 */
	const char			*match;

	struct mqa			*mqa;
	struct giu			*giu;
	void				*msix_table_base;

	u32				 num_intcs;
	struct giu_gpio_intc_params	 intcs_params[GIU_GPIO_MAX_NUM_TCS];
	u32				 num_outtcs;
	struct giu_gpio_outtc_params	 outtcs_params[GIU_GPIO_MAX_NUM_TCS];
};

/**
 * Initialize a gpio
 *
 * @param[in]	params		gpio initialization parameters
 * @param[out]	gpio		A pointer to opaque gpio handle of type 'struct giu_gpio *'.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 */
int giu_gpio_init(struct giu_gpio_params *params, struct giu_gpio **gpio);

/**
 * De-initialize a gpio
 *
 * @param[in]	gpio	A gpio handle.
 *
 */
void giu_gpio_deinit(struct giu_gpio *gpio);

/* TODO: this is temporary API. wil be replaced later with correct serialization */
int giu_gpio_serialize_old(struct giu_gpio *gpio, void **file_map);
int giu_gpio_probe(char *match, char *regfile_name, struct giu_gpio **gpio);

/**
 * Serialize the GPIO parameters
 *
 * The serialization API is called by the 'master' user application to serialize a buffer-pool object.
 * The output string is created in a JSON format.
 * Below is how a bpool config-string looks like:
 *	gpio-<giu_id>:<port-id>: {
 *	iomap_filename: <str>,	(TBD)
 *	giu_id: <int>,
 *	id: <int>,
 *	num-in-tcs: <int>,		(used for in QoS according to #priorities)
 *	intc: {
 *		num-inqs : <int>,	(used for in RSS (according to remote side #cores))
 *		inqs : {[<int>>],...,[<int>]}, ([q-size])
 *		bpool : <int>
 *	},
 *	...
 *	num-out-tcs: <int>,	(used for out QoS according to #priorities)
 *	outtc: {
 *		num-outqs : <int>,	(used for in TSS (according to remote side #cores))
 *		outqs : {[<int>>],...,[<int>]}, ([q-size])
 *	},
 *
 * The guest application can then access the created buffer pool object, and retrieve the bpool config string
 *
 * @param[in]	gpio		A gpio handle.
 * @param[in]	buf		buffer
 * @param[in]	size		size of the buffer
 * @param[in]	depth		size of the buffer
 *
 * @retval	>0 (length of data written to buffer) on success
 * @retval	<0 on failure
 */
int giu_gpio_serialize(struct giu_gpio *gpio, char *buff, u32 size, u8 depth);

/**
 * Probe a gpio
 *
 * The probe API should be called by the user application to create the buffer-gpio object for a guest application.
 *
 * @param[in]	match		The matching string to search for in the Buffer pool object.
 * @param[in]	buff		Buffer gpio object.
 * @param[out]	gpio		gpio structure containing the results of the match
 *
 * @retval      0 on success
 * @retval      <0 on failure
 */
int giu_ppio_probe_new(char *match, char *buff, struct giu_gpio **gpio);

/**
 * Remove a gpio
 *
 * @param[in]	gpio	A gpio handle.
 *
 */
void giu_gpio_remove(struct giu_gpio *gpio);

/****************************************************************************
 *	Run-time API
 ****************************************************************************/

/**
 * Rx/Tx packet's descriptor
 *
 * This is the representation of the Rx/Tx descriptor
 * (which is defined in Management/Host levels)
 */
struct giu_gpio_desc {
	u32                      cmds[GIU_GPIO_DESC_NUM_WORDS];
};

enum giu_vlan_tag {
	GIU_VLAN_TAG_NONE = 0,	/* No VLANs */
	GIU_VLAN_TAG_SINGLE,	/* Single VLAN */
	GIU_VLAN_TAG_DOUBLE,	/* Double VLANs */
	GIU_VLAN_TAG_RESERVED,	/* Reserved */
};

enum giu_inq_l3_type {
	GIU_INQ_L3_TYPE_IPV4 = 0,
	GIU_INQ_L3_TYPE_IPV6,
	GIU_INQ_L3_TYPE_OTHER
};

enum giu_inq_l4_type {
	GIU_INQ_L4_TYPE_TCP = 0,
	GIU_INQ_L4_TYPE_UDP,
	GIU_INQ_L4_TYPE_OTHER
};

enum giu_outq_l3_type {
	GIU_OUTQ_L3_TYPE_NA = 0,
	GIU_OUTQ_L3_TYPE_IPV4_NO_OPTS,	/* IPv4 with IHL=5, TTL>0 */
	GIU_OUTQ_L3_TYPE_IPV4_OK,	/* IPv4 with IHL>5, TTL>0 */
	GIU_OUTQ_L3_TYPE_IPV4_TTL_ZERO,	/* Other IPV4 packets */
	GIU_OUTQ_L3_TYPE_IPV6_NO_EXT,	/* IPV6 without extensions */
	GIU_OUTQ_L3_TYPE_IPV6_EXT,	/* IPV6 with extensions */
	GIU_OUTQ_L3_TYPE_ARP,		/* ARP */
	GIU_OUTQ_L3_TYPE_USER_DEFINED	/* User defined */
};

enum giu_outq_l4_type {
	GIU_OUTQ_L4_TYPE_NA = 0,
	GIU_OUTQ_L4_TYPE_TCP = 1,
	GIU_OUTQ_L4_TYPE_UDP = 2,
	GIU_OUTQ_L4_TYPE_OTHER = 3
};

enum giu_outq_l2_cast_type {
	GIU_OUTQ_L2_UNICAST = 0,	/* L2 Unicast */
	GIU_OUTQ_L2_MULTICAST,		/* L2 Multicast */
	GIU_OUTQ_L2_BROADCAST,		/* L2 Broadcast */
};

enum giu_outq_l3_cast_type {
	GIU_OUTQ_L3_UNICAST = 0,	/* L3 Unicast */
	GIU_OUTQ_L3_MULTICAST,		/* L3 Multicast */
	GIU_OUTQ_L3_ANYCAST,		/* L3 Anycast */
	GIU_OUTQ_L3_BROADCAST,		/* L3 Broadcast */
};

enum giu_outq_desc_status {
	GIU_DESC_ERR_OK = 0,
	GIU_DESC_ERR_MAC_CRC,		/* L2 MAC error (for example CRC Error) */
	GIU_DESC_ERR_MAC_OVERRUN,	/* L2 Overrun Error*/
	GIU_DESC_ERR_MAC_RESERVED,	/* L2 Reserved */
	GIU_DESC_ERR_MAC_RESOURCE,	/* L2 Resource Error (No buffers for multi-buffer frame) */
	GIU_DESC_ERR_IPV4_HDR,		/* L3 IPv4 Header error */
	GIU_DESC_ERR_L4_CHECKSUM,	/* L4 checksum error */
	GIU_DESC_ERR_IPV4_CHECKSUM	/* IPv4 checksum error */
};

enum giu_outq_desc_ipv4_l4_status {
	GIU_DESC_IPV4_L4_ERR_OK = 0,
	GIU_DESC_IPV4_L4_ERR_IPV4_HDR,	    /* L3 IPv4 Header error */
	GIU_DESC_IPV4_L4_ERR_L4_CHECKSUM,   /* L4 checksum error */
	GIU_DESC_IPV4_L4_ERR_IPV4_CHECKSUM, /* IPv4 checksum error */
};

/******** RXQ-Desc ********/
/* cmd 0 */
#define GIU_RXD_FIRST			(0x2)
#define GIU_RXD_LAST			(0x1)
#define GIU_RXD_FIRST_LAST		(0x3)
#define GIU_RXD_L_MASK			(0x10000000)
#define GIU_RXD_F_MASK			(0x20000000)
#define GIU_RXD_FL_MASK			(TXD_F_MASK | TXD_L_MASK)
#define GIU_RXD_L3_TYPE_MASK		(0x0C000000)
#define GIU_RXD_L4_TYPE_MASK		(0x03000000)
#define GIU_RXD_PAD_DIS			(0x00800000)
#define GIU_RXD_MD_MODE			(0x00400000)
#define GIU_RXD_POOL_ID_MASK		(0x001F0000)
#define GIU_RXD_GEN_IP_CHK_MASK		(0x00008000)
#define GIU_RXD_GEN_L4_CHK_MASK		(0x00006000)
#define GIU_RXD_IP_HEAD_LEN_MASK	(0x00001F00)
#define GIU_RXD_BUF_MODE		(0x00000080)
#define GIU_RXD_L3_OFFSET_MASK		(0x0000007F)
/* cmd 1 */
#define GIU_RXD_PKT_OFF_MASK		(0x000000FF)
#define GIU_RXD_VLAN_INFO_MASK		(0x0000C000)
#define GIU_RXD_BYTE_COUNT_MASK		(0xFFFF0000)
/* cmd 2 */
#define GIU_RXD_L4_INITIAL_CHK		(0xFFFF0000)
/* cmd 4 */
#define GIU_RXD_BUF_PHYS_LO_MASK	(0xFFFFFFFF)
/* cmd 5 */
#define GIU_RXD_BUF_PHYS_HI_MASK	(0x000000FF)
/* cmd 6 */
#define GIU_RXD_BUF_COOKIE_LO_MASK	(0xFFFFFFFF)
/* cmd 7 */
#define GIU_RXD_BUF_COOKIE_HI_MASK	(0x000000FF)


#define GIU_RXD_GET_L3_OFF(desc)         (((desc)->cmds[0] & GIU_RXD_L3_OFFSET_MASK) >> 0)
#define GIU_RXD_GET_IPHDR_LEN(desc)      (((desc)->cmds[0] & GIU_RXD_IP_HEAD_LEN_MASK) >> 8)
#define GIU_RXD_GET_GEN_L4_CHK(desc)     (((desc)->cmds[0] & GIU_RXD_GEN_L4_CHK_MASK) >> 13)
#define GIU_RXD_GET_GEN_IP_CHK(desc)     (((desc)->cmds[0] & GIU_RXD_GEN_IP_CHK_MASK) >> 15)
#define GIU_RXD_GET_POOL_ID(desc)        (((desc)->cmds[0] & GIU_RXD_POOL_ID_MASK) >> 16)
#define GIU_RXD_GET_L4_PRS_INFO(desc)    (((desc)->cmds[0] & GIU_RXD_L4_TYPE_MASK) >> 24)
#define GIU_RXD_GET_L3_PRS_INFO(desc)    (((desc)->cmds[0] & GIU_RXD_L3_TYPE_MASK) >> 26)
#define GIU_RXD_GET_MD_MODE(desc)        (((desc)->cmds[0] & GIU_RXD_MD_MODE) >> 22)

#define GIU_RXD_GET_VLAN_INFO(desc)	 (((desc)->cmds[1] & GIU_RXD_VLAN_INFO_MASK) >> 14)

/******** TXQ-Desc ********/
/* cmd 0 */
#define GIU_TXD_L3_OFF_MASK		(0x0000007F)
#define GIU_TXD_DP_MASK			(0x00000080)
#define GIU_TXD_IPHDR_LEN_MASK		(0x00001F00)
#define GIU_TXD_EC_MASK			(0x00006000)
#define GIU_TXD_ES_MASK			(0x00008000)
#define GIU_TXD_MD_MODE			(0x00400000)
#define GIU_TXD_IPv4_L4_ERR_MASK	(0x01800000)
#define GIU_TXD_L4_PRS_INFO_MASK	(0x0E000000)
#define GIU_TXD_L3_PRS_INFO_MASK	(0x70000000)
/* cmd 1 */
#define GIU_TXD_PKT_OFF_MASK		(0x000000FF)
#define GIU_TXD_BYTE_COUNT_MASK		(0xFFFF0000)
#define GIU_TXD_L3_CAST_INFO_MASK	(0x00000C00)
#define GIU_TXD_L2_CAST_INFO_MASK	(0x00003000)
#define GIU_TXD_VLAN_INFO_MASK		(0x0000C000)
/* cmd 2 */
#define GIU_TXD_L4_INITIAL_CHK		(0xFFFF0000)
#define GIU_TXD_PORT_NUM_MASK		(0x0000E000)
#define GIU_TXD_DEST_QID		(0x00001FFF)
/* cmd 4 */
#define GIU_TXD_BUF_PHYS_LO_MASK	(0xFFFFFFFF)
/* cmd 5 */
#define GIU_TXD_BUF_PHYS_HI_MASK	(0x000000FF)
/* cmd 6 */
#define GIU_TXD_BUF_VIRT_LO_MASK	(0xFFFFFFFF)
/* cmd 7 */
#define GIU_TXD_BUF_VIRT_HI_MASK	(0xFFFFFFFF)

#define GIU_TXD_SET_L3_OFF(desc, data)	\
	((desc)->cmds[0] = ((desc)->cmds[0] & ~GIU_TXD_L3_OFF_MASK) | (data << 0 & GIU_TXD_L3_OFF_MASK))
#define GIU_TXD_SET_IP_HEAD_LEN(desc, data)	\
	((desc)->cmds[0] = ((desc)->cmds[0] & ~GIU_TXD_IPHDR_LEN_MASK) | (data << 8 & GIU_TXD_IPHDR_LEN_MASK))
#define GIU_TXD_SET_EC(desc, data)	\
	((desc)->cmds[0] = ((desc)->cmds[0] & ~GIU_TXD_EC_MASK) | ((data) << 13 & GIU_TXD_EC_MASK))
#define GIU_TXD_SET_ES(desc, data)	\
	((desc)->cmds[0] = ((desc)->cmds[0] & ~GIU_TXD_EC_MASK) | (data << 15 & ~GIU_TXD_EC_MASK))
#define GIU_TXD_SET_MD_MODE(desc, data)	\
	((desc)->cmds[0] = ((desc)->cmds[0] & ~GIU_TXD_MD_MODE) | (data << 22 & GIU_TXD_MD_MODE))
#define GIU_TXD_SET_IPV4_L4_ERR(desc, data) \
	((desc)->cmds[0] = ((desc)->cmds[0] & ~GIU_TXD_IPv4_L4_ERR_MASK) | (data << 23 & GIU_TXD_IPv4_L4_ERR_MASK))
#define GIU_TXD_SET_L4_TYPE(desc, data)	\
	((desc)->cmds[0] = ((desc)->cmds[0] & ~GIU_TXD_L4_PRS_INFO_MASK) | (data << 25 & GIU_TXD_L4_PRS_INFO_MASK))
#define GIU_TXD_SET_L3_TYPE(desc, data)	\
	((desc)->cmds[0] = ((desc)->cmds[0] & ~GIU_TXD_L3_PRS_INFO_MASK) | (data << 28 & GIU_TXD_L3_PRS_INFO_MASK))

#define GIU_TXD_SET_L3_CAST_INFO(desc, data)	\
	((desc)->cmds[1] = ((desc)->cmds[1] & ~GIU_TXD_L3_CAST_INFO_MASK) | (data << 10 & GIU_TXD_L3_CAST_INFO_MASK))
#define GIU_TXD_SET_L2_CAST_INFO(desc, data)	\
	((desc)->cmds[1] = ((desc)->cmds[1] & ~GIU_TXD_L2_CAST_INFO_MASK) | (data << 12 & GIU_TXD_L2_CAST_INFO_MASK))
#define GIU_TXD_SET_VLAN_INFO(desc, data)	\
	((desc)->cmds[1] = ((desc)->cmds[1] & ~GIU_TXD_VLAN_INFO_MASK) | (data << 14 & GIU_TXD_VLAN_INFO_MASK))

#define GIU_TXD_SET_DEST_QID(desc, data)	\
	((desc)->cmds[2] = ((desc)->cmds[2] & ~GIU_TXD_DEST_QID) | (data << 0 & GIU_TXD_DEST_QID))

#define GIU_TXD_GET_L3_OFF(desc)	(((desc)->cmds[0] & GIU_TXD_L3_OFF_MASK) >> 0)
#define GIU_TXD_GET_IPHDR_LEN(desc)	(((desc)->cmds[0] & GIU_TXD_IPHDR_LEN_MASK) >> 8)
#define GIU_TXD_GET_PKT_OFF(desc)	(((desc)->cmds[1] & GIU_TXD_PKT_OFF_MASK) >> 0)
#define GIU_TXD_GET_L4_PRS_INFO(desc)	(((desc)->cmds[0] & GIU_TXD_L4_PRS_INFO_MASK) >> 25)
#define GIU_TXD_GET_L3_PRS_INFO(desc)	(((desc)->cmds[0] & GIU_TXD_L3_PRS_INFO_MASK) >> 28)
#define GIU_TXD_GET_DEST_QID(desc)	(((desc)->cmds[2] & GIU_TXD_DEST_QID) >> 0)

/**
 * Send a batch of frames (single descriptor) on an OutQ of PP-IO.
 *
 * The routine assumes that the BM-Pool is either freed by HW (by appropriate desc
 * setter) or by the MUSDK client SW.
 *
 * @param[in]		gpio	A pointer to a PP-IO object.
 * @param[in]		tc	traffic class on which to send the frames.
 * @param[in]		qid	out-Q id (in a TC) on which to send the frames.
 * @param[in]		descs	A pointer to an array of descriptors representing the
 *				frames to be sent.
 * @param[in,out]	num	input: number of frames to be sent; output: number of frames sent.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int giu_gpio_send(struct giu_gpio	*gpio,
		  u8			 tc,
		  u8			 qid,
		  struct giu_gpio_desc	*desc,
		  u16			*num);

/**
 * Get number of packets sent on a queue, since last call of this API.
 *
 * @param[in]		gpio	A pointer to a PP-IO object.
 * @param[in]		tc	traffic class on which to check for done tx frames.
 * @param[in]		qid	out-Q id (in a TC) on which to check for done tx frames.
 * @param[out]		num	Number of frames that were sent.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int giu_gpio_get_num_outq_done(struct giu_gpio	*gpio,
			       u8		 tc,
			       u8		 qid,
			       u16		*num);

/**
 * Receive packets on a gpio.
 *
 * @param[in]		gpio	A pointer to a PP-IO object.
 * @param[in]		tc	traffic class on which to receive frames
 * @param[in]		qid	in-Q id (in a TC) on which to receive the frames.
 * @param[in]		descs	A pointer to an array of descriptors represents the
 *				received frames.
 * @param[in,out]	num	input: Max number of frames to receive;
 *				output: number of frames received.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int giu_gpio_recv(struct giu_gpio	*gpio,
		  u8			 tc,
		  u8			 qid,
		  struct giu_gpio_desc	*desc,
		  u16			*num);


/****************************************************************************
 *	Run-time Control API
 ****************************************************************************/

/* Capabilities structures*/
struct giu_gpio_q_info {
	u16	size;	 /* Q size */
	u16	offset;	 /* Offset of the data within the buffer */
};

struct giu_gpio_intc_info {
	u8	num_inqs;
	struct	giu_gpio_q_info inqs_inf[GIU_GPIO_TC_MAX_NUM_QS];
	struct	giu_bpool *pools[GIU_GPIO_TC_MAX_NUM_BPOOLS];
};

struct giu_gpio_intcs_info {
	u8				num_intcs;
	struct giu_gpio_intc_info	intcs_inf[GIU_GPIO_MAX_NUM_TCS];
};

struct giu_gpio_outtc_info {
	u8				num_outqs;
	struct giu_gpio_q_info		outqs_inf[GIU_GPIO_TC_MAX_NUM_QS];
	struct giu_gpio_q_info		doneqs_inf[GIU_GPIO_TC_MAX_NUM_QS];
};

struct giu_gpio_outtcs_info {
	u8				num_outtcs;
	struct giu_gpio_outtc_info	outtcs_inf[GIU_GPIO_MAX_NUM_TCS];
};

struct giu_gpio_capabilities {
	u8				id;
	struct giu_gpio_intcs_info	intcs_inf;
	struct giu_gpio_outtcs_info	outtcs_inf;
};

/**
 * GIU event parameters
 */
struct giu_gpio_event_params {
	u32 pkt_coal;
	u32 usec_coal;
	u32 tc_mask;
};

struct giu_gpio_statistics {
	u64 in_packets;
	u64 out_packets;
};

struct giu_gpio_q_statistics {
	u64 packets;
};

/**
 * Get GPIO capabilities.
 *
 * @param[in]		gpio	A pointer to a GPIO object.
 * @param[out]		capa	capability structure
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int giu_gpio_get_capabilities(struct giu_gpio *gpio, struct giu_gpio_capabilities *capa);

/**
 * Enable a gpio for sending/receiving packets to/from GIU
 *
 * @param[in]		gpio	A pointer to a PP-IO object.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int giu_gpio_enable(struct giu_gpio *gpio);

/**
 * Disable a gpio from sending/receiving packets to/from GIU
 *
 * @param[in]		gpio	A pointer to a PP-IO object.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int giu_gpio_disable(struct giu_gpio *gpio);

/**
 * Create a GPIO event
 *
 * The event API is called to create a sys_event for a GIU-GPIO, that
 * can later be polled through the mv_sys_event_poll() API.
 * This is only releavnt to 'NMP_SCHED_TX'
 *
 * @param[in]	gpio		A pointer to a GP-IO object.
 * @param[in]	params		Parameters for the event.
 * @param[out]	ev		A pointer to event handle of type 'struct mv_sys_event *'.
 *
 * @retval      0 on success
 * @retval      <0 on failure
 */
int giu_gpio_create_event(struct giu_gpio *gpio, struct giu_gpio_event_params *params, struct mv_sys_event **ev);

/**
 * Delete a GPIO event
 *
 * @param[in]	ev		A sys_event handle.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 */
int giu_gpio_delete_event(struct mv_sys_event *ev);

/**
 * Set a GPIO event
 *
 * The set_event API is called to enable the creation of events for the related GIU.
 *
 * @param[in]	ev		A sys_event handle.
 * @param[in]	en		enable/disable
 *
 * @retval      0 on success
 * @retval      <0 on failure
 */
int giu_gpio_set_event(struct mv_sys_event *ev, int en);

/**
 * Get total statistics of all tc and queues
 *
 * @param[in]	gpio	A pointer to a GP-IO object.
 * @param[out]	stats   A pointer to statistics structure.
 * @param[in]	reset   '1' for reset staticistics

 * @retval	0 on success
 * @retval	error-code otherwise
 */
int giu_gpio_get_statistics(struct giu_gpio *gpio, struct giu_gpio_statistics *stats, int reset);

/**
 * Get the statistics of a specific queue
 *
 * @param[in]	gpio	A pointer to a GP-IO object.
 * @param[in]	out    '1' for out direction
 * @param[in]	rem    '1' for remote direcition
 * @param[in]	tc id
 * @param[in]	qid     queue id
 * @param[out]	stats   A pointer to statistics structure.
 * @param[in]	reset  '1' for reset staticistics
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int giu_gpio_get_q_statistics(struct giu_gpio *gpio, int out, int rem, u8 tc, u8 qid,
							  struct giu_gpio_q_statistics *stats, int reset);


/****************************************************************************
 *	Desc inspections APIs
 ****************************************************************************/

/******** TXQ  ********/

/**
 * Reset an outq packet descriptor to default value.
 *
 * @param[in]	desc	A pointer to a packet descriptor structure to be set.
 */
static inline void giu_gpio_outq_desc_reset(struct giu_gpio_desc *desc)
{
	/* Note: no need to 'zeroed' cmds[4] as they will be overridden
	 *	 when preparing the descriptor
	 */
	desc->cmds[0] = desc->cmds[1] = desc->cmds[2] = desc->cmds[3] =
	desc->cmds[5] = desc->cmds[6] = 0;
}

/**
 * Set the physical address in an outq packet descriptor.
 *
 * @param[in]	desc	A pointer to a packet descriptor structure to be set.
 * @param[in]	addr	Physical DMA address containing the packet to be sent.
 */
static inline void giu_gpio_outq_desc_set_phys_addr(struct giu_gpio_desc *desc, dma_addr_t addr)
{
	/* cmd[4] and cmd[5] holds the buffer physical address (Low and High parts) */
	desc->cmds[4] = (u32)addr;
	desc->cmds[5] = (desc->cmds[5] & ~GIU_TXD_BUF_PHYS_HI_MASK) | ((u64)addr >> 32 & GIU_TXD_BUF_PHYS_HI_MASK);
	/* mark cmds[5/7] so we can validate it in the remote side */
	desc->cmds[5] |= GIU_GPIO_DESC_PA_WATERMARK;
	desc->cmds[7] = GIU_GPIO_DESC_COOKIE_WATERMARK;
}

/**
 * Set the virtual address in an outq packet descriptor.
 *
 * This routine should be used by upper layer in cases where the buffer
 * is being allocated outside of MUSDK-dmamem allocator. The virtual-address
 * is used localy by the GIU-GPIO driver in order to calculate RSS.
 *
 * @param[in]	desc	A pointer to a packet descriptor structure to be set.
 * @param[in]	addr	Virtual address containing the packet to be sent.
 */
static inline void giu_gpio_outq_desc_set_virt_addr(struct giu_gpio_desc *desc, void *addr)
{
	/* cmd[6] and cmd[7] holds the buffer virtual address (Low and High parts) */
	desc->cmds[6] = (u32)(uintptr_t)addr;
	desc->cmds[7] = (desc->cmds[7] & ~GIU_TXD_BUF_VIRT_HI_MASK) |
			((u64)(uintptr_t)addr >> 32 & GIU_TXD_BUF_VIRT_HI_MASK);
}

/**
 * Set the packet offset in an outq packet descriptor..
 *
 * @param[in]	desc	A pointer to a packet descriptor structure to be set.
 * @param[in]	offset	The packet offset.
 */
static inline void giu_gpio_outq_desc_set_pkt_offset(struct giu_gpio_desc *desc, u8  offset)
{
	desc->cmds[1] = (u32)offset;
}

/**
 * Set the packet length in an outq packet descriptor..
 *
 * @param[in]	desc	A pointer to a packet descriptor structure to be set.
 * @param[in]	len	The packet length, not including CRC.
 */
static inline void giu_gpio_outq_desc_set_pkt_len(struct giu_gpio_desc *desc, u16 len)
{
	desc->cmds[1] = (desc->cmds[1] & ~GIU_TXD_BYTE_COUNT_MASK) | (len << 16 & GIU_TXD_BYTE_COUNT_MASK);
}

/**
 * Set the protocol info in an outq packet descriptor.
 * This API must always be called.
 *
 * @param[in]	desc		A pointer to a packet descriptor structure to be set.
 * @param[in]	status		error status of the lowest layer (L2/L3/L4).
 * @param[in]	l2_cast		The l2 cast info.
 * @param[in]	vlan_type	The vlan tag.
 * @param[in]	l3_cast		The l3 cast info.
 * @param[in]	l3_type		The l3 type of the packet.
 * @param[in]	l3_offset	The l3 offset of the packet.
 * @param[in]	l4_type		The l4 type of the packet.
 * @param[in]	l4_offset	The l4 offset of the packet.
 */
static inline void giu_gpio_outq_desc_set_proto_info(struct giu_gpio_desc *desc,
						     enum giu_outq_desc_status status,
						     enum giu_outq_l2_cast_type l2_cast,
						     enum giu_vlan_tag vlan_type,
						     enum giu_outq_l3_cast_type l3_cast,
						     enum giu_outq_l3_type l3_type,
						     u8 l3_offset,
						     enum giu_outq_l4_type l4_type,
						     u8 l4_offset
)
{
	if (status == GIU_DESC_ERR_IPV4_HDR)
		GIU_TXD_SET_IPV4_L4_ERR(desc, GIU_DESC_IPV4_L4_ERR_IPV4_HDR);
	else if (status == GIU_DESC_ERR_IPV4_CHECKSUM)
		GIU_TXD_SET_IPV4_L4_ERR(desc, GIU_DESC_IPV4_L4_ERR_IPV4_CHECKSUM);
	else if (status == GIU_DESC_ERR_L4_CHECKSUM)
		GIU_TXD_SET_IPV4_L4_ERR(desc, GIU_DESC_IPV4_L4_ERR_L4_CHECKSUM);
	else if (status) {
		GIU_TXD_SET_ES(desc, 1);
		GIU_TXD_SET_EC(desc, status - 1);
	}

	/* L2 info */
	GIU_TXD_SET_L2_CAST_INFO(desc, l2_cast);
	GIU_TXD_SET_VLAN_INFO(desc, vlan_type);

	/* L3 info */
	GIU_TXD_SET_L3_CAST_INFO(desc, l3_cast);
	GIU_TXD_SET_L3_TYPE(desc, l3_type);
	GIU_TXD_SET_L3_OFF(desc, l3_offset);

	/* L4 info */
	GIU_TXD_SET_L4_TYPE(desc, l4_type);
	GIU_TXD_SET_IP_HEAD_LEN(desc, (l4_offset - l3_offset)/sizeof(u32));
}

/**
 * Set the metadata mode in an outq packet descriptor.
 *
 * @param[in]	desc		A pointer to a packet descriptor structure to be set.
 * @param[in]	md_mode		'1' for setting metadata mode
 */
static inline void giu_gpio_outq_desc_set_md_mode(struct giu_gpio_desc *desc, int md_mode)
{
	GIU_TXD_SET_MD_MODE(desc, md_mode);
}

/******** RXQ  ********/

/**
 * Get the physical DMA address from an inq packet descriptor.
 *
 * @param[in]	desc	A pointer to a packet descriptor structure.
 *
 * @retval	physical dma address
 */
static inline u64 giu_gpio_inq_desc_get_phys_addr(struct giu_gpio_desc *desc)
{
	/* cmd[4] and cmd[5] holds the buffer physical address (Low and High parts) */
	return ((u64)((desc->cmds[5] & GIU_RXD_BUF_PHYS_HI_MASK) >> 0) << 32) |	(u64)desc->cmds[4];
}

/**
 * Get the user defined cookie from an inq packet descriptor.
 *
 * @param[in]	desc	A pointer to a packet descriptor structure.
 *
 * @retval	cookie
 */
static inline u64 giu_gpio_inq_desc_get_cookie(struct giu_gpio_desc *desc)
{
	/* cmd[6] and cmd[7] holds the cookie (Low and High parts) */
	return ((u64)((desc->cmds[7] & GIU_RXD_BUF_COOKIE_HI_MASK) >> 0) << 32) | (u64)desc->cmds[6];
}

/**
 * Get the packet length from an inq packet descriptor.
 *
 * @param[in]	desc	A pointer to a packet descriptor structure.
 *
 * @retval	packet length
 */
static inline u16 giu_gpio_inq_desc_get_pkt_len(struct giu_gpio_desc *desc)
{
	u16 len = (desc->cmds[1] & GIU_RXD_BYTE_COUNT_MASK) >> 16;
	return len;
}

/**
 * Get the bpool of an inq packet descriptor.
 *
 * @param[in]	desc	A pointer to a packet descriptor structure.
 * @param[in]	gpio	A pointer to a PP-IO object.
 *
 * @retval	pointer to bpool
 */
static inline struct giu_bpool *giu_gpio_inq_desc_get_bpool(struct giu_gpio_desc *desc,
							    struct giu_gpio *gpio __attribute__((__unused__)))
{
	return &giu_bpools[GIU_RXD_GET_POOL_ID(desc)];
}

/**
 * get the metadata mode of an inq packet descriptor.
 *
 * @param[in]	desc	A pointer to a packet descriptor structure.
 *
 * @retval	metadata mode
 */
static inline int giu_gpio_inq_desc_get_md_mode(struct giu_gpio_desc *desc)
{
	return GIU_RXD_GET_MD_MODE(desc);
}

/**
 * Get L2 info from an inq packet descriptor.
 *
 * @param[in]	desc		A pointer to a packet descriptor structure.
 * @param[out]	vlan		The vlan info.
 *
 */
static inline void giu_gpio_inq_desc_get_l2_info(struct giu_gpio_desc *desc,
						 enum giu_vlan_tag *vlan)
{
	*vlan = GIU_RXD_GET_VLAN_INFO(desc);
}

/**
 * Get L3 info from an inq packet descriptor.
 *
 * @param[in]	desc		A pointer to a packet descriptor structure.
 * @param[out]	l3_type		The l3 type of the packet.
 * @param[out]	l3_offset	The l3 offset of the packet.
 * @param[out]	gen_l3_chk	Whether to generate IPV4 checksum.
 *
 */
static inline void giu_gpio_inq_desc_get_l3_info(struct giu_gpio_desc *desc,
						 enum giu_inq_l3_type *l3_type,
						 u8 *l3_offset,
						 int *gen_l3_chk)
{
	*l3_type = GIU_RXD_GET_L3_PRS_INFO(desc);
	*l3_offset = GIU_RXD_GET_L3_OFF(desc);
	/* when this bit's value is 0 it means that L3 csum should be generated */
	*gen_l3_chk = !GIU_RXD_GET_GEN_IP_CHK(desc);
}

/**
 * Get L4 info from an inq packet descriptor.
 *
 * @param[in]	desc		A pointer to a packet descriptor structure.
 * @param[out]	l4_type		The l4 type of the packet.
 * @param[out]	l4_offset	The l4 offset of the packet.
 * @param[out]	gen_l4_chk	Whether to generate TCP/UDP checksum.
 *
 */
static inline void giu_gpio_inq_desc_get_l4_info(struct giu_gpio_desc *desc,
						 enum giu_inq_l4_type *l4_type,
						 u8 *l4_offset,
						 int *gen_l4_chk)
{
	u8 l3_offset = GIU_RXD_GET_L3_OFF(desc);

	*l4_type = GIU_RXD_GET_L4_PRS_INFO(desc);
	*l4_offset = l3_offset + sizeof(u32)*GIU_RXD_GET_IPHDR_LEN(desc);
	/* when this bit's value is 0 it means that L4 csum should be generated */
	*gen_l4_chk = !GIU_RXD_GET_GEN_L4_CHK(desc);
}

/** @} */ /* end of grp_giu_io */

#endif /* __MV_GIU_GPIO_H__ */
