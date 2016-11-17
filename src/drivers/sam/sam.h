
/**
 * @file sam.h
 *
 * Security accelerator internal structures
 *
 */

#ifndef _SAM_H_
#define _SAM_H_

#include <drivers/mv_sam.h>
#include "std_internal.h"

#include "cs_driver.h"
#include "api_pec.h"
#include "api_dmabuf.h"
#include "api_driver197_init.h"
#include "sa_builder.h"
#include "sa_builder_basic.h"
#include "token_builder.h"
#include "firmware_eip207_api_cmd.h"

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
	u32  num_bufs;        /* number of output buffers */
	struct sam_buf_info out_frags[SAM_CIO_MAX_FRAGS]; /* array of output buffers */
	u32  auth_icv_offset; /* offset of ICV in the buffer (in bytes) */
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
	struct sam_session_params	params;
	struct sam_cio			*cio;
	/* Fields needed for EIP197 HW */
	SABuilder_Params_Basic_t	basic_params;
	SABuilder_Params_t		sa_params;
	struct sam_dmabuf		sa_dmabuf;
	u32				sa_words;
	u8				tcr_data[SAM_TCR_DATA_SIZE];
	u32				tcr_words;
	u32				token_words;
};

static inline u32 sam_cio_next_idx(struct sam_cio *cio, u32 idx)
{
	idx++;
	if (idx == cio->params.size)
		idx = 0;

	return idx;
}

static inline u32 sam_cio_prev_idx(struct sam_cio *cio, u32 idx)
{
	if (idx == 0)
		idx = cio->params.size - 1;
	else
		idx--;

	return idx;
}

/* next_request + 1 == next_result -> Full */
static inline bool sam_cio_is_full(struct sam_cio *cio)
{
	return (sam_cio_next_idx(cio, cio->next_request) == cio->next_result);
}

/* next_request == next_result -> Empty */
static inline bool sam_cio_is_empty(struct sam_cio *cio)
{
	return (cio->next_request == cio->next_result);
}

static inline int sam_max_check(int value, int limit, const char *name)
{
	if ((value < 0) || (value >= limit)) {
		pr_err("%s %d is out of range [0..%d]\n",
			name ? name : "value", value, (limit - 1));
		return 1;
	}
	return 0;
}
#endif /* _SAM_H_ */
