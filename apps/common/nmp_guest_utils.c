/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#include <string.h>
#include <stdio.h>
#include "mv_std.h"
#include "lib/lib_misc.h"
#include "lib/file_utils.h"
#include "nmp_guest_utils.h"
#include "mv_pp2_bpool.h"

#define NMP_MAX_BUF_STR_LEN		256
#define NMP_CFG_FILE_LOCAL_DIR		"./"
#define NMP_CFG_FILE_VAR_DIR		"/var/"
#define NMP_CFG_FILE_NAME_PREFIX	"musdk-nmp-config.txt"

/* Number of BM pools reserved by kernel */
#define MVAPPS_PP2_NUM_BPOOLS_RSRV	3
/* Reserved BM pools mask */
#define MVAPPS_PP2_BPOOLS_RSRV		((1 << MVAPPS_PP2_NUM_BPOOLS_RSRV) - 1)
/* Maximum number of pools per packet processor */
#define MVAPPS_PP2_MAX_NUM_BPOOLS	(PP2_BPOOL_NUM_POOLS - MVAPPS_PP2_NUM_BPOOLS_RSRV)


/*---------------------------*/
/*  Guest mode related functions  */
/*--------------------------*/

int app_read_nmp_cfg_file(char *cfg_file, struct nmp_params *params)
{
	int	rc;
	char	file_name[SER_MAX_FILE_NAME];

	/* If cfg-file is provided, read the nmp-config from this location. Otherwise try to read either from
	 * local dir or from /var dir
	 */
	strcpy(file_name, cfg_file);
	rc = nmp_read_cfg_file(file_name, params);
	if (rc) {
		memset(file_name, 0, SER_MAX_FILE_NAME);
		snprintf(file_name, sizeof(file_name), "%s%s", NMP_CFG_FILE_LOCAL_DIR, NMP_CFG_FILE_NAME_PREFIX);
		rc = nmp_read_cfg_file(file_name, params);
		if (rc) {
			memset(file_name, 0, SER_MAX_FILE_NAME);
			snprintf(file_name, sizeof(file_name), "%s%s", NMP_CFG_FILE_VAR_DIR, NMP_CFG_FILE_NAME_PREFIX);
			rc = nmp_read_cfg_file(file_name, params);
			if (rc) {
				pr_info("nmp_config_file not found\n");
				return rc;
			}
		}
	}

	pr_info("nmp_cfg_location: %s\n", file_name);

	return 0;
}

int app_guest_utils_build_all_giu_bpools(char *buff, struct nmp_guest_info *guest_info,
					 struct giu_bpools_desc *pools_desc,
					 u32 num_buffs)
{
	struct nmp_guest_port_info	*giu_info = &guest_info->giu_info;
	int				 i, err;

	if (giu_info->num_bpools > GIU_GPIO_TC_MAX_NUM_BPOOLS) {
		pr_err("only %d pools allowed! %d\n",
			GIU_GPIO_TC_MAX_NUM_BPOOLS, giu_info->num_bpools);
		return -EINVAL;
	}

	pools_desc->num_bpools = giu_info->num_bpools;

	for (i = 0; i < pools_desc->num_bpools; i++) {
		err = giu_bpool_probe(giu_info->bpool_info[i].bpool_name,
				buff, &pools_desc->bpools[i]);
		if (err) {
			pr_err("probe giu-bpool buffs failed!\n");
			return err;
		}

		if (num_buffs) {
			err = app_giu_build_bpool(i, num_buffs);
			if (err) {
				pr_err("allocate giu-bpool buffs failed!\n");
				return err;
			}
		}
	}

	return 0;
}

int app_nmp_guest_giu_port_init(char *buff, struct nmp_guest_info *guest_info, struct giu_port_desc *port)
{
	struct giu_gpio_capabilities	 capa;
	struct nmp_guest_port_info	*giu_info = &guest_info->giu_info;
	int				 err, i;

	err = giu_ppio_probe(giu_info->port_name, buff, &port->gpio);
	if (err) {
		pr_err("pp2_ppio_probe failed for %s\n", giu_info->port_name);
		return err;
	}

	if (!port->gpio) {
		pr_err("PP-IO init failed!\n");
		return -EIO;
	}

	err = giu_gpio_get_capabilities(port->gpio, &capa);
	if (err) {
		pr_err("giu_gpio_get_capabilities failed for %s\n", giu_info->port_name);
		return err;
	}

	if (capa.intcs_inf.num_intcs != capa.outtcs_inf.num_outtcs) {
		pr_err("Number of IN and OUT TCS must be equal!\n");
		return -EINVAL;
	}

	port->num_tcs = capa.intcs_inf.num_intcs;
	for (i = 0; i < port[i].num_tcs; i++) {
		port->num_inqs[i] = capa.intcs_inf.intcs_inf[i].num_inqs;
		port->num_outqs[i] = capa.outtcs_inf.outtcs_inf[i].num_outqs;
	}
	port->inq_size = capa.intcs_inf.intcs_inf[0].inqs_inf[0].size;
	port->outq_size = capa.outtcs_inf.outtcs_inf[0].outqs_inf[0].size;

	port->initialized = 1;

	return err;
}


int app_guest_utils_build_all_pp2_bpools(char *buff, struct nmp_guest_info *guest_info,
				     struct bpool_desc ***ppools,
				     struct pp2_glb_common_args *pp2_args,
				     struct bpool_inf infs[])
{
	u32				 num_pools = 0;
	u32				 alloc_num_pools = 0;
	u32				 idx;
	struct pp2_bpool_params		 bpool_params;
	int				 i, j, k, err;
	struct bpool_desc		**pools = NULL;
	struct pp2_bpool_capabilities	 capa;
	struct pp2_buff_inf		*buffs_inf = NULL;
	struct nmp_guest_module_info	*pp2_info = &guest_info->ports_info;

	for (i = 0; i < pp2_info->num_ports; i++)
		num_pools += pp2_info->port_info[i].num_bpools;

	if (num_pools > MVAPPS_PP2_MAX_NUM_BPOOLS) {
		pr_err("only %d pools allowed! %d\n", MVAPPS_PP2_MAX_NUM_BPOOLS, num_pools);
		return -EINVAL;
	}

	if (num_pools != pp2_args->num_pools) {
		pr_err("Number of bpools in guest file differs from expected in inf struct %d, %d\n",
			num_pools, pp2_args->num_pools);
		return -EINVAL;
	}

	pools = (struct bpool_desc **)malloc(pp2_args->pp2_num_inst * sizeof(struct bpool_desc *));
	if (!pools) {
		pr_err("no mem for bpool_desc array!\n");
		return -ENOMEM;
	}

	*ppools = pools;
	memset(pools, 0, pp2_args->pp2_num_inst * sizeof(struct bpool_desc *));

	pools[0] = (struct bpool_desc *)malloc(num_pools * sizeof(struct bpool_desc));
	if (!pools[0]) {
		pr_err("no mem for bpool_desc array!\n");
		err = -ENOMEM;
		goto bpool_alloc_err1;
	}
	memset(pools[0], 0, num_pools * sizeof(struct bpool_desc));

	for (j = 0; j < pp2_info->num_ports; j++) {
		for (k = 0; k < pp2_info->port_info[j].num_bpools; k++) {
			memset(&bpool_params, 0, sizeof(bpool_params));
			bpool_params.match = pp2_info->port_info[j].bpool_info[k].bpool_name;

			idx = j * pp2_info->port_info[j].num_bpools + k;
			err = pp2_bpool_probe((char *)bpool_params.match, buff, &pools[0][idx].pool);
			if (err) {
				pr_err("pp2_bpool_probe failed for %s\n", bpool_params.match);
				goto bpool_alloc_err2;
			}
			err = pp2_bpool_get_capabilities(pools[0][idx].pool, &capa);
			if (err) {
				pr_err("pp2_bpool_get_capabilities failed for %s\n", bpool_params.match);
				goto bpool_alloc_err2;
			}

			if (infs[k].num_buffs > capa.max_num_buffs) {
				pr_err("Bpool requested buffers (%d) is too big. Available: %d\n",
				       infs[k].num_buffs, capa.max_num_buffs);
				err = -EIO;
				goto bpool_alloc_err2;
			}

			if (!pools[0][idx].pool) {
				pr_err("bpool-0:%d init failed!\n", idx);
				err = -EIO;
				goto bpool_alloc_err2;
			}

			pools[0][idx].buffs_inf =
				(struct pp2_buff_inf *)malloc(infs[k].num_buffs * sizeof(struct pp2_buff_inf));
			if (!pools[0][idx].buffs_inf) {
				pr_err("no mem for bpools-inf array!\n");
				err = -ENOMEM;
				goto bpool_alloc_err3;
			}
			alloc_num_pools++;
			memset(pools[0][idx].buffs_inf, 0, infs[k].num_buffs * sizeof(struct pp2_buff_inf));
			buffs_inf = pools[0][idx].buffs_inf;
			pools[0][idx].num_buffs = infs[k].num_buffs;

			pr_debug("pools[0][%d].num_buffs %d, max_num_buff: %d, buf_size %d\n", idx,
				pools[0][idx].num_buffs, capa.max_num_buffs, capa.buff_len);

			err = app_allocate_bpool_buffs(&infs[k], buffs_inf, pools[0][idx].pool, pp2_args->hif);
			if (err) {
				pr_err("allocate bpool buffs failed!\n");
				goto bpool_alloc_err3;
			}
		}
	}
	return 0;

bpool_alloc_err3:
	for (j = 0; j < alloc_num_pools; j++)
		kfree(pools[0][j].buffs_inf);

bpool_alloc_err2:
	kfree(pools[0]);

bpool_alloc_err1:
	kfree(pools);
	return err;
}

int app_nmp_guest_pp2_port_init(char *buff, struct nmp_guest_info *guest_info, struct port_desc *port)
{
	int				 err;
	struct pp2_ppio_capabilities	 capa;
	int				 i, j;
	struct nmp_guest_module_info	*pp2_info = &guest_info->ports_info;

	pr_debug("pp2_info: num_ports %d\n", pp2_info->num_ports);
	for (i = 0; i < pp2_info->num_ports; i++) {
		pr_debug("	port: %d, name %s\n", i, pp2_info->port_info[i].port_name);

		err = pp2_ppio_probe(pp2_info->port_info[i].port_name, buff, &port[i].ppio);
		if (err) {
			pr_err("pp2_ppio_probe failed for %s\n", pp2_info->port_info[i].port_name);
			return err;
		}

		if (!port[i].ppio) {
			pr_err("PP-IO init failed!\n");
			return -EIO;
		}

		err = pp2_ppio_get_capabilities(port[i].ppio, &capa);
		if (err) {
			pr_err("pp2_ppio_get_capabilities failed for %s\n", pp2_info->port_info[i].port_name);
			return err;
		}

		port[i].num_tcs = capa.intcs_inf.num_intcs;
		for (j = 0; j < port[j].num_tcs; j++) {
			port[i].port_params.inqs_params.tcs_params[j].pkt_offset =
				capa.intcs_inf.intcs_infs[i].pkt_offset;
			port[i].num_inqs[j] = capa.intcs_inf.intcs_infs[i].num_inqs;
		}
		port[i].num_outqs = capa.outqs_inf.num_outtcs;
		port[i].inq_size = capa.intcs_inf.intcs_infs[0].inqs_infs[0].size;
		port[i].outq_size = capa.outqs_inf.outqs_infs[0].size;
		port[i].initialized = 1;
	}

	return err;
}

