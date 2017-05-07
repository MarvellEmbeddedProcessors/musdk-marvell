/******************************************************************************
 *	Copyright (C) 2016 Marvell International Ltd.
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

#ifndef __MV_SAM_H__
#define __MV_SAM_H__

#include "mv_std.h"

struct sam_cio;
struct sam_sa;

#include "mv_sam_session.h"
#include "mv_sam_cio.h"

/** Maximum number of supported crypto engines */
#define SAM_HW_ENGINE_NUM	2

/* Maximum number of supported rings (cios) per engine */
#define SAM_HW_RING_NUM		4

/* Maximum number of supported CIOs for all engines */
#define SAM_MAX_CIO_NUM		(SAM_HW_RING_NUM * SAM_HW_ENGINE_NUM)

#define SAM_SA_DEBUG_FLAG	0x1
#define SAM_CIO_DEBUG_FLAG	0x2

struct sam_capability {
	u32 cipher_algos; /** Bit mask of supported cipher algorithms as defined in "enum sam_cipher_alg" */
	u32 cipher_modes; /** Bit mask of supported cipher modes as defined in "enum sam_cipher_mode" */
	u32 auth_algos;	  /** Bit mask of supported authentication algorithms as defined in "enum sam_auth_alg" */
};

struct sam_init_params {
	u32 max_num_sessions; /** Maximum number of concurrent sessions can be created */
};

/**
 * Return supported algorithms and modes for encryption and authentication.
 *
 * @param[out]     capa		- pointer to capability staructure to be filled
 *
 * @retval	0		- success
 * @retval	Negative	- failure
 */
int sam_get_capability(struct sam_capability *capa);

/**
 * Return number of crypto engines.
 *
 * @retval      number of crypto engines
 */
u8 sam_get_num_inst(void);

/**
 * Set debug flags.
 *
 * To enable debug information of the SAM driver,
 *	use "--enable-sam-debug" flag during ./configure
 *
 * @param[in]     flags    - debug flags.
 *                         0x1 - SA, 0x2 - CIO. (default: 0x0)
 *
 * @retval      0          - debug flags are set
 * @retval	-ENOTSUP   - debug flags are not supported
 */
int sam_set_debug_flags(u32 flags);

/**
 * Init SAM driver and allocate global resources.
 *
 * @param[in]     params   - pointer to structure with SAM driver
 *                           initialization parameters.
 *
 * @retval      0          - success
 * @retval	-EINVAL    - failure
 */
int sam_init(struct sam_init_params *params);

/**
 * Deinit SAM driver and free global resources.
 */
void sam_deinit(void);

#endif /* __MV_SAM_H__ */
