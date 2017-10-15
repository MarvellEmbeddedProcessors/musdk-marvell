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
#include "mv_giu_bpool.h"
#include "mv_pp2_ppio.h" /* for descriptor inspection functionality */

/** @addtogroup grp_giu_io GIU Port: I/O (GP-IO)
 *
 *  GIU Port I/O (GP-IO) API documentation
 *
 *  @{
 */

#define GIU_GPIO_DESC_NUM_WORDS		8


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
	/* Reset control part of the descriptor (4 first dwords)
	 * Note: no need to the pointers as they will be overridden
	 *	 when preparing the descriptor
	 */
	memset(desc->cmds, 0, sizeof(u32) * 4);
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
	desc->cmds[5] = ((u64)addr >> 32);
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
	return ((u64)desc->cmds[5] << 32) | ((u64)(desc->cmds[4]));
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
	return ((u64)desc->cmds[7] << 32) | ((u64)(desc->cmds[6]));
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
	return &giu_bpools[DM_RXD_GET_POOL_ID((struct pp2_ppio_desc *)desc)];
}

/** @} */ /* end of grp_giu_io */

#endif /* __MV_GIU_GPIO_H__ */
