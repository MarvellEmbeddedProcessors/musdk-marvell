/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __MV_SAM_H__
#define __MV_SAM_H__

#include "mv_std.h"

struct sam_cio;
struct sam_sa;

#include "mv_sam_session.h"
#include "mv_sam_cio.h"

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
 * Return number of crypto devices.
 *
 * @retval      number of crypto devices
 */
u8 sam_get_num_inst(void);

/**
 * Return number of available crypto IOs.
 *
 * @retval      number of available crypto IOs
 */
u32 sam_get_num_cios(u32 inst);

/**
 * Return bit map of available crypto IOs.
 *
 * @param[in]     inst - crypto device number.
 * @param[out]    map  - bit map of available crypto IOs.
 *
 * @retval	number of available crypto IOs
 */
u32 sam_get_available_cios(u32 inst, u32 *map);

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
