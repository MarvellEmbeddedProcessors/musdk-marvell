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

#ifndef __MV_GIU_GPIO_H__
#define __MV_GIU_GPIO_H__

#include "mv_std.h"
#include "mv_mqa.h"
#include "mv_giu_gpio_init.h"
#include "mv_giu_bpool.h"
#include "mv_pp2_ppio.h" /* for descriptor inspection functionality */

/** @addtogroup grp_giu_io GIU Port: I/O (GP-IO)
 *
 *  GIU Port I/O (GP-IO) API documentation
 *
 *  @{
 */

#define GIU_GPIO_DESC_NUM_WORDS		8

#define GIU_GPIO_DESC_PA_WATERMARK	0xcafe0000
#define GIU_GPIO_DESC_COOKIE_WATERMARK	0xcafecafe


/* GPIO Handler */
struct giu_gpio;


/**
 * Rx/Tx packet's descriptor
 *
 * This is the representation of the Rx/Tx descriptor
 * (which is defined in Management/Host levels)
 */
struct giu_gpio_desc {
	u32                      cmds[GIU_GPIO_DESC_NUM_WORDS];
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
#define GIU_RXD_BYTE_COUNT_MASK		(0xFFFF0000)
/* cmd 2 */
#define GIU_RXD_L4_INITIAL_CHK		(0xFFFF0000)
/* cmd 4 */
#define GIU_RXD_BUF_PHYS_LO_MASK	(0xFFFFFFFF)
/* cmd 5 */
#define GIU_RXD_BUF_PHYS_HI_MASK	(0x000000FF)
/* cmd 6 */
#define GIU_RXD_BUF_VIRT_LO_MASK	(0xFFFFFFFF)
/* cmd 7 */
#define GIU_RXD_BUF_VIRT_HI_MASK	(0x000000FF)


#define GIU_RXD_GET_L3_OFF(desc)         (((desc)->cmds[0] & GIU_RXD_L3_OFFSET_MASK) >> 0)
#define GIU_RXD_GET_IPHDR_LEN(desc)      (((desc)->cmds[0] & GIU_RXD_IP_HEAD_LEN_MASK) >> 8)
#define GIU_RXD_GET_POOL_ID(desc)        (((desc)->cmds[0] & GIU_RXD_POOL_ID_MASK) >> 16)
#define GIU_RXD_GET_L4_PRS_INFO(desc)    (((desc)->cmds[0] & GIU_RXD_L4_TYPE_MASK) >> 24)
#define GIU_RXD_GET_L3_PRS_INFO(desc)    (((desc)->cmds[0] & GIU_RXD_L3_TYPE_MASK) >> 26)
#define GIU_RXD_GET_MD_MODE(desc)        (((desc)->cmds[0] & GIU_TXD_MD_MODE) >> 22)



/******** TXQ-Desc ********/
/* cmd 0 */
#define GIU_TXD_L3_OFF_MASK		(0x0000007F)
#define GIU_TXD_IPHDR_LEN_MASK		(0x00001F00)
#define GIU_TXD_EC_MASK			(0x00006000)
#define GIU_TXD_ES_MASK			(0x00008000)
#define GIU_TXD_MD_MODE			(0x00400000)
#define GIU_TXD_L4_CHK_OK_MASK		(0x00800000)
#define GIU_TXD_L3_IP4_HDR_ERR_MASK	(0x01000000)
#define GIU_TXD_L4_PRS_INFO_MASK	(0x0E000000)
#define GIU_TXD_L3_PRS_INFO_MASK	(0x70000000)
/* cmd 1 */
#define GIU_TXD_PKT_OFF_MASK		(0x000000FF)
#define GIU_TXD_DP_MASK			(0x00001800)
#define GIU_TXD_BYTE_COUNT_MASK		(0xFFFF0000)
#define GIU_TXD_PORT_NUM_MASK		(0x0000E000)
/* cmd 2 */
#define GIU_TXD_L4_INITIAL_CHK		(0xFFFF0000)
#define GIU_TXD_DEST_QID		(0x00001FFF)
/* cmd 4 */
#define GIU_TXD_BUF_PHYS_LO_MASK	(0xFFFFFFFF)
/* cmd 5 */
#define GIU_TXD_BUF_PHYS_HI_MASK	(0x000000FF)
/* cmd 6 */
#define GIU_TXD_BUF_VIRT_LO_MASK	(0xFFFFFFFF)
/* cmd 7 */
#define GIU_TXD_BUF_VIRT_HI_MASK	(0x000000FF)


#define GIU_TXD_SET_L3_OFF(desc, data)	\
	((desc)->cmds[0] = ((desc)->cmds[0] & ~GIU_TXD_L3_OFF_MASK) | (data << 0 & GIU_TXD_L3_OFF_MASK))
#define GIU_TXD_SET_IP_HEAD_LEN(desc, data)	\
	((desc)->cmds[0] = ((desc)->cmds[0] & ~GIU_TXD_IPHDR_LEN_MASK) | (data << 8 & GIU_TXD_IPHDR_LEN_MASK))
#define GIU_TXD_SET_L3_TYPE(desc, data)	\
	((desc)->cmds[0] = ((desc)->cmds[0] & ~GIU_TXD_L3_PRS_INFO_MASK) | (data << 28 & GIU_TXD_L3_PRS_INFO_MASK))
#define GIU_TXD_SET_L4_TYPE(desc, data)	\
	((desc)->cmds[0] = ((desc)->cmds[0] & ~GIU_TXD_L4_PRS_INFO_MASK) | (data << 25 & GIU_TXD_L4_PRS_INFO_MASK))
#define GIU_TXD_SET_MD_MODE(desc, data)	\
	((desc)->cmds[0] = ((desc)->cmds[0] & ~GIU_TXD_MD_MODE) | (data << 22 & GIU_TXD_MD_MODE))
#define GIU_TXD_SET_DEST_QID(desc, data)	\
	((desc)->cmds[2] = ((desc)->cmds[2] & ~GIU_TXD_DEST_QID) | (data << 0 & GIU_TXD_DEST_QID))

#define GIU_TXD_GET_L3_OFF(desc)	(((desc)->cmds[0] & GIU_TXD_L3_OFF_MASK) >> 0)
#define GIU_TXD_GET_IPHDR_LEN(desc)	(((desc)->cmds[0] & GIU_TXD_IPHDR_LEN_MASK) >> 8)
#define GIU_TXD_GET_PKT_OFF(desc)	(((desc)->cmds[1] & GIU_TXD_PKT_OFF_MASK) >> 0)
#define GIU_TXD_GET_L4_PRS_INFO(desc)	(((desc)->cmds[0] & GIU_TXD_L4_PRS_INFO_MASK) >> 25)
#define GIU_TXD_GET_L3_PRS_INFO(desc)	(((desc)->cmds[0] & GIU_TXD_L3_PRS_INFO_MASK) >> 28)
#define GIU_TXD_GET_DEST_QID(desc)	(((desc)->cmds[2] & GIU_TXD_DEST_QID) >> 0)

/**
 * Initialize a gpio
 *
 * @param[in]	init_params	gpio initialization parameters
 * @param[out]	gpio		A pointer to opaque gpio handle of type 'struct giu_gpio *'.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 */
int giu_gpio_init(struct giu_gpio_init_params *init_params, struct giu_gpio **gpio);

/**
 * De-initialize a gpio
 *
 * @param[in]	gpio	A gpio handle.
 *
 */
void giu_gpio_deinit(struct giu_gpio *gpio);

/**
 * Probe a gpio
 *
 * @param[in]	regfile_name	register file location.
 * @param[in]	regfile_name	register file location.
 * @param[out]	gpio		A pointer to opaque gpio handle of type 'struct giu_gpio *'.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 */
int giu_gpio_probe(char *match, char *regfile_name, struct giu_gpio **gpio);

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

#define GIU_GPIO_MAX_NUM_TCS		8
#define GIU_GPIO_TC_MAX_NUM_QS		4
#define GIU_GPIO_TC_MAX_NUM_BPOOLS	3


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


/****************************************************************************
 *	Desc inspections APIs
 ****************************************************************************/

/******** TXQ  ********/

/**
 * Reset an outq packet descriptor to default value.
 *
 * @param[out]	desc	A pointer to a packet descriptor structure.
 *
 */
static inline void giu_gpio_outq_desc_reset(struct giu_gpio_desc *desc)
{
	/* Note: no need to 'zeroed' cmds[4] as they will be overridden
	 *	 when preparing the descriptor
	 */
	desc->cmds[0] = desc->cmds[1] = desc->cmds[2] = desc->cmds[3] =
	desc->cmds[5] = 0;
}

/**
 * Set the physical address in an outq packet descriptor.
 *
 * @param[out]	desc	A pointer to a packet descriptor structure.
 * @param[in]	addr	Physical DMA address containing the packet to be sent.
 *
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

static inline void giu_gpio_outq_desc_set_pkt_offset(struct giu_gpio_desc *desc, u8  offset)
{
	return pp2_ppio_outq_desc_set_pkt_offset((struct pp2_ppio_desc *)desc, offset);
}

/**
 * Set the packet length in an outq packet descriptor..
 *
 * @param[out]	desc	A pointer to a packet descriptor structure.
 * @param[in]	len	The packet length, not including CRC.
 *
 */
static inline void giu_gpio_outq_desc_set_pkt_len(struct giu_gpio_desc *desc, u16 len)
{
	return pp2_ppio_outq_desc_set_pkt_len((struct pp2_ppio_desc *)desc, len);
}

/**
 * Set the protocol info in an outq packet descriptor.
 * This API must always be called.
 *
 * @param[out]	desc		A pointer to a packet descriptor structure.
 * @param[in]	l3_type		The l3 type of the packet.
 * @param[in]	l4_type		The l4 type of the packet.
 * @param[in]	l3_offset	The l3 offset of the packet.
 * @param[in]	l4_offset	The l4 offset of the packet.
 *
 */
static inline void giu_gpio_outq_desc_set_proto_info(struct giu_gpio_desc *desc, enum giu_outq_l3_type l3_type,
						     enum giu_outq_l4_type l4_type, u8  l3_offset, u8 l4_offset)
{
	GIU_TXD_SET_L3_TYPE(desc, l3_type);
	GIU_TXD_SET_L4_TYPE(desc, l4_type);
	GIU_TXD_SET_L3_OFF(desc, l3_offset);
	GIU_TXD_SET_IP_HEAD_LEN(desc, (l4_offset - l3_offset)/sizeof(u32));
}

/**
 * Set the metadata mode in an outq packet descriptor.
 *
 * @param[out]	desc		A pointer to a packet descriptor structure.
 * @param[in]	md_mode		'1' for setting metadata mode
 *
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
	return ((u64)((desc->cmds[7] & GIU_RXD_BUF_VIRT_HI_MASK) >> 0) << 32) | (u64)desc->cmds[6];
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
	u16 len = (desc->cmds[1] & RXD_BYTE_COUNT_MASK) >> 16;
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

/** @} */ /* end of grp_giu_io */

#endif /* __MV_GIU_GPIO_H__ */
