/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#define log_fmt(fmt) "dev_mng: " fmt

#include "std_internal.h"
#include "env/trace/trc_pf.h"

#include "db.h"
#include "drivers/mv_pp2.h"
#include "drivers/mv_pp2_bpool.h"
#include "drivers/mv_pp2_ppio.h"
#include "src/drivers/ppv2/pp2.h"
#include "dev_mng.h"
#include "dev_mng_pp2.h"
#include "lib/lib_misc.h"

#define NMP_PP2_FIRST_MUSDK_IN_QUEUE		0

/* Maximum number of ports used by NMP */
#define NMP_PP2_MAX_NUM_PORTS			3

/** =========================== **/
/** == Device Initialization == **/
/** =========================== **/

/** =========================== **/
/** == Serialization helpers == **/
/** =========================== **/

/* Serialize PP2 */
int dev_mng_pp2_serialize(struct nmnicpf *nmnicpf, char *buff, u32 size)
{
	int	 ret;
	size_t	 pos = 0;
	u32	 port_index;
	u32	 j;

	if (!nmnicpf->pp2.ports_desc)
		/* no pp2, just return */
		return 0;

	/* Serialize relations info */
	json_print_to_buffer(buff, size, 1, "\"relations-info\": {\n");
	json_print_to_buffer(buff, size, 2, "\"num_pp2_ports\": %d,\n", nmnicpf->pp2.num_ports);
	for (port_index = 0; port_index < nmnicpf->pp2.num_ports; port_index++) {
		struct nmp_pp2_port_desc *port = (struct nmp_pp2_port_desc *)&nmnicpf->pp2.ports_desc[port_index];

		json_print_to_buffer(buff, size, 2, "\"ppio-%d\": \"ppio-%d:%d\",\n",
				     port_index, port->pp_id, port->ppio_id);
		json_print_to_buffer(buff, size, 2, "\"num_bpools\": %d,\n", port->num_pools);
		for (j = 0; j < port->num_pools; j++) {
			if (j == port->num_pools - 1)
				json_print_to_buffer(buff, size, 2, "\"bpool-%d\": \"pool-%d:%d\"\n", j,
						port->pools_desc[j].pool->pp2_id, port->pools_desc[j].pool->id);
			else
				json_print_to_buffer(buff, size, 2, "\"bpool-%d\": \"pool-%d:%d\",\n", j,
						port->pools_desc[j].pool->pp2_id, port->pools_desc[j].pool->id);
		}
	}
	json_print_to_buffer(buff, size, 1, "},\n");

	/* Serialize bpools */
	for (port_index = 0; port_index < nmnicpf->pp2.num_ports; port_index++) {
		struct nmp_pp2_port_desc *port = (struct nmp_pp2_port_desc *)&nmnicpf->pp2.ports_desc[port_index];

		for (j = 0; j < port->num_pools; j++) {
			ret = pp2_bpool_serialize(port->pools_desc[j].pool, &buff[pos], size - pos);
			if (ret < 0)
				return ret;
			pos += ret;
			if (pos != strlen(buff)) {
				pr_err("found mismatch between pos (%zu) and buff len (%zu)\n", pos, strlen(buff));
				return -EINVAL;
			}
		}
	}

	/* serialize ppio info */
	for (port_index = 0; port_index < nmnicpf->pp2.num_ports; port_index++) {
		struct nmp_pp2_port_desc *port = (struct nmp_pp2_port_desc *)&nmnicpf->pp2.ports_desc[port_index];

		/* Serialize ppio */
		ret = pp2_ppio_serialize(port->ppio, &buff[pos], size - pos);
		if (ret < 0)
			return ret;
		pos += ret;
		if (pos != strlen(buff)) {
			pr_err("found mismatch between pos (%zu) and buff len (%zu)\n", pos, strlen(buff));
			return -EINVAL;
		}
	}

	return pos;
}

/** ====================== **/
/** Hardware Functionality **/
/** ====================== **/

/* Initialize the PP2 */
static int dev_mng_pp2_inst_init(struct nmp *nmp)
{
	struct pp2_init_params	 pp2_params;
	int			 err;

	if (!nmp->nmpp2.pp2_en)
		return 0;

	memset(&pp2_params, 0, sizeof(struct pp2_init_params));
	pp2_params.bm_pool_reserved_map = nmp->nmpp2.pp2_params.bm_pool_reserved_map;
	pp2_params.hif_reserved_map = 0;
	pp2_params.rss_tbl_reserved_map = 0;

	err = pp2_init(&pp2_params);
	if (err)
		return -EINVAL;

	return 0;
}

/* Initialize the PP2 interface */
int dev_mng_pp2_init(struct nmp *nmp)
{
	int err;

	pr_info("Initializing PP2...\n");

	/* Initialize the ppio instances */
	err = dev_mng_pp2_inst_init(nmp);
	if (err) {
		pr_err("dev_mng_pp2_inst_init failed\n");
		return -EINVAL;
	}

	pr_info("pp2 enabled: %d\n", nmp->nmpp2.pp2_en);

	return 0;
}

/** ======================== **/
/** == Device Termination == **/
/** ======================== **/

/** ======================== **/
/** Hardware Functionality **/
/** ====================== **/

int dev_mng_pp2_terminate(struct nmp *nmp)
{
	pp2_deinit();
	return 0;
}

