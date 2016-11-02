
/**
 * @file sam.h
 *
 * Security accelerator internal structures
 *
 */

#ifndef _SAM_H_
#define _SAM_H_

#include <mv_std.h>

#include <drivers/mv_sam.h>

#include "api_pec.h"
#include "api_dmabuf.h"
#include "api_driver197_init.h"
#include "sa_builder.h"
#include "sa_builder_basic.h"
#include "token_builder.h"
#include "cs_driver.h"

#define SAM_DMABUF_ALIGN	4 /* cache line */

#define SAM_DMA_BANK_PKT	0 /* dynamic bank */

#define SAM_DMA_BANK_SA		1 /* static bank */


#define SAM_HW_RING_NUM		DRIVER_MAX_NOF_RING_TO_USE
#define SAM_HW_RING_SIZE	DRIVER_PEC_MAX_PACKETS
#define SAM_HW_SA_NUM		DRIVER_PEC_MAX_SAS

#define SAM_HW_RING_RETRY_COUNT	(1000)
#define SAM_HW_RING_RETRY_US	(10)

#define SAM_AAD_IN_TOKEN_MAX_SIZE	(64)

/* max token size in bytes */
#define SAM_TOKEN_DMABUF_SIZE		(16 * 4)

/* max SA buffer size in bytes */
#define SAM_SA_DMABUF_SIZE		(64 * 4)

/* max TCR data size in bytes */
#define SAM_TCR_DATA_SIZE		(9 * 4)

struct sam_dmabuf {
	u32                     size;
	DMABuf_Handle_t		hndl;
	DMABuf_HostAddress_t	host_addr;
};

struct sam_cio_op {
	bool is_valid;
	struct sam_sa *sa;
	void *cookie;
	struct sam_dmabuf token_dmabuf; /* DMA buffer for token  */
	struct sam_dmabuf data_dmabuf;  /* DMA buffer for data */
};


struct sam_cio {
	struct sam_cio_params params;
	struct sam_cio_op *operations;    /* array of operations */
	u32 next_request;
	u32 next_result;
};

struct sam_sa {
	bool is_valid;
	struct sam_session_params params;
	/* Fields needed for EIP197 HW */
	SABuilder_Params_Basic_t	basic_params;
	SABuilder_Params_t		sa_params;
	struct sam_dmabuf		sa_dmabuf;
	u32				sa_word_count;
	u8				tcr_data[SAM_TCR_DATA_SIZE];
	u32				tcr_word_count;
};

#endif /* _SAM_H_ */
