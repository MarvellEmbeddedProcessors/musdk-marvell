/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#ifndef __MV_GIU_H__
#define __MV_GIU_H__

#include "mv_std.h"

/** @addtogroup grp_giu_init GIU: Initialization
 *
 *  Generic Interface Unit (GIU) Initialization API documentation
 *
 *  @{
 */

#define GIU_MAX_NUM_GPIO	8 /**< Maximum number of gpio instances */
#define GIU_MAX_ENG_PER_TYPE	4 /**< Maximum number of dma-engines per giu eng */

struct giu;
struct giu_mng_ch;


/**
 * GIU engine type enumaration
 */
enum giu_eng {
	GIU_ENG_MNG = 0,
	GIU_ENG_IN,
	GIU_ENG_OUT,

	GIU_ENG_OUT_OF_RANGE,
};

/**
 * GIU descriptor type enumaration
 */
enum giu_desc_type {
	GIU_DESC_IN = 0,
	GIU_DESC_OUT,
	GIU_DESC_BUFF
};

/**
 * GIU indices mode enumaration
 */
enum giu_indices_copy_mode {
	GIU_INDX_CPY_MODE_VIRT = 0,
	GIU_INDX_CPY_MODE_DMA,

	GIU_INDX_CPY_MODE_MAX
};

/**
 * GIU multi queues mode enumaration
 */
enum giu_multi_qs_mode {
	GIU_MULTI_QS_MODE_REAL = 0,
	GIU_MULTI_QS_MODE_VIRT
};

/**
 * GIU emulation parameters
 */
struct giu_emul_params {
	u8 num_dma_engines;	/**< Number of DMA engines for this type*/
	char *engine_name[GIU_MAX_ENG_PER_TYPE];
};

/**
 * GIU parameters
 */
struct giu_params {
	struct mqa		*mqa; /** a pointer to MQA handle */

	u64			 msi_regs_pa;	/**< MSI phys-address registers base */
	u64			 msi_regs_va;	/**< MSI virt-address registers base */

	u8			 num_gie_types;
	struct giu_emul_params	 gie_type_params[GIU_ENG_OUT_OF_RANGE];
};

/**
 * GIU management-channel local queue parameters
 */
struct giu_mng_ch_lcl_q_params {
	u32		len; /**< queue length */
};

/**
 * GIU management-channel remote queue parameters
 */
struct giu_mng_ch_rem_q_params {
	dma_addr_t	 pa; /**< Remote queue phys base address */
	dma_addr_t	 prod_pa; /**< queue producer physical-address */
	void		*prod_va; /**< queue producer virtual-address */
	dma_addr_t	 cons_pa; /**< queue consumer physical-address */
	void		*cons_va; /**< queue consumer virtual-address */
	u32		 len; /**< queue length */
};

/**
 * GIU management-channel parameters
 */
struct giu_mng_ch_params {
	dma_addr_t			 rem_base_pa; /**< remote queues base address phys address*/
	void				*rem_base_va; /**< remote queues base address virt address*/
	u16				 desc_size; /**< the management channel descriptors size */

	struct giu_mng_ch_lcl_q_params	 lcl_cmd_q;  /**< Local command-queue parameters */
	struct giu_mng_ch_lcl_q_params	 lcl_resp_q; /**< Local response-queue parameters */
	struct giu_mng_ch_rem_q_params	 rem_cmd_q;  /**< Remote command-queue parameters */
	struct giu_mng_ch_rem_q_params	 rem_resp_q; /**< Remote response-queue parameters */
};

/**
 * GIU management-channel MQA queues
 */
struct giu_mng_ch_qs {
	struct mqa_q			*lcl_cmd_q;  /**< Local command MQA queue handle */
	struct mqa_q			*lcl_resp_q; /**< Local response MQA queue handle */
	struct mqa_q			*rem_cmd_q;  /**< Remote command MQA queue handle */
	struct mqa_q			*rem_resp_q; /**< Remote response MQA queue handle */
};

/**
 * Initialize the global GIU
 *
 * @param[in]	params	A pointer to structure that contains all relevant parameters.
 * @param[out]	giu	A pointer to returned GIU handle.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 */
int giu_init(struct giu_params *params, struct giu **giu);

/**
 * Destroy the global GIU
 *
 * @param[in]	giu		A pointer to GIU handler.
 */
void giu_deinit(struct giu *giu);

/**
 * Create and initialize GIU Management channel
 *
 * @param[in]	giu		A pointer to GIU handler.
 * @param[in]	params		GIU emulator engine.
 * @param[out]	mng_ch		A pointer to the returned GIU MNG CH handle.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 */
int giu_mng_ch_init(struct giu *giu, struct giu_mng_ch_params *params, struct giu_mng_ch **mng_ch);

/**
 * Destroy GIU Management channel
 *
 * @param[out]	mng_ch		A pointer to GIU MNG CH handle.
 */
void giu_mng_ch_deinit(struct giu_mng_ch *mng_ch);

/**
 * get GIU Management channel MQA queues objects
 *
 * @param[out]	mng_ch		A pointer to GIU MNG CH handle.
 * @param[out]	qs		A pointer to the returned GIU MNG CH queues struct to be filled.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 */
int giu_mng_ch_get_qs(struct giu_mng_ch *mng_ch, struct giu_mng_ch_qs *qs);

/**
 * Start GIU emulation scheduling.
 *
 * @param[in]	giu		A pointer to GIU handler.
 * @param[in]	eng		GIU emulation engine.
 * @param[in]	time_limit	schedule time lime (0 == infinite).
 * @param[in]	qe_limit	queue elements limit for processing.
 * @param[out]	pending		pending jobs
 *
 * @retval	0 on success
 * @retval	<0 on failure
 */
int giu_schedule(struct giu *giu, enum giu_eng eng, u64 time_limit, u64 qe_limit, u16 *pending);

/**
 * Return the GIU emualtion descriptor size.
 *
 * @param[in]	giu		A pointer to GIU handler.
 * @param[in]	type		type of GIU queue/descriptor.
 *
 * @retval	The size fo the descriptor
 */
int giu_get_desc_size(struct giu *giu, enum giu_desc_type type);

int giu_get_msi_regs(struct giu *giu, u64 *va, u64 *pa);

int giu_get_num_dma_engines(struct giu *giu, enum giu_eng eng, u8 *num_dma_engines);

/** @} */ /* end of grp_giu_init */

#endif /* __MV_GIU_H__ */
