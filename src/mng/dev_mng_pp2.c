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
int dev_mng_pp2_serialize(struct nmnicpf *nmnicpf)
{
	int	 err;
	char	 file_name[SER_MAX_FILE_NAME];
	char	 buff[SER_MAX_FILE_SIZE];
	size_t	 pos = 0;
	char	 dev_name[100];
	u32	 size;
	u32	 port_index;
	u32	 j;
	struct mv_sys_dma_mem_info	 mem_info;

	/* Remove the serialize files */

	/* TODO: move this function to a higher level and remove all guests */
	snprintf(file_name, sizeof(file_name), "%s%s%d", SER_FILE_VAR_DIR, SER_FILE_NAME_PREFIX, nmnicpf->guest_id);
	remove(file_name);

	if (!nmnicpf->pp2.ports_desc)
		/* no pp2, just return */
		return 0;

	pr_info("starting serialization of guest %d\n", nmnicpf->guest_id);

	size = SER_MAX_FILE_SIZE;
	memset(buff, 0, SER_MAX_FILE_SIZE);

	/* Serialize the DMA info */
	mem_info.name = dev_name;
	mv_sys_dma_mem_get_info(&mem_info);
	json_print_to_buffer(buff, size, 0, "{\n");
	json_print_to_buffer(buff, size, 1, "\"dma-info\": {\n");
	json_print_to_buffer(buff, size, 2, "\"file_name\": \"%s\",\n", mem_info.name);
	json_print_to_buffer(buff, size, 2, "\"region_size\": %zu,\n", mem_info.size);
	json_print_to_buffer(buff, size, 2, "\"phys_addr\": %#zx\n", mem_info.paddr);
	json_print_to_buffer(buff, size, 1, "},\n");

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
			err = pp2_bpool_serialize(port->pools_desc[j].pool, buff, SER_MAX_FILE_SIZE);
			if (err)
				return err;
		}
	}

	/* serialize ppio info */
	for (port_index = 0; port_index < nmnicpf->pp2.num_ports; port_index++) {
		struct nmp_pp2_port_desc *port = (struct nmp_pp2_port_desc *)&nmnicpf->pp2.ports_desc[port_index];

		/* Serialize ppio */
		err = pp2_ppio_serialize(port->ppio, buff, SER_MAX_FILE_SIZE);
		if (err)
			return err;
	}

	pos = strlen(buff);
	json_print_to_buffer(buff, size, 0, "}\n");

	/* write buffer to file */
	err = write_buf_to_file(file_name, buff, strlen(buff));
	if (err)
		return -EIO;

	return 0;
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

