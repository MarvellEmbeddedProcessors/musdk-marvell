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

#define NMP_MAX_NUM_CONTAINERS		4
#define NMP_MAX_NUM_LFS			8
#define NMP_GIE_MAX_TCS			8
#define NMP_GIE_MAX_Q_PER_TC		128
#define NMP_GIE_MAX_BPOOLS		16
#define NMP_GIE_MAX_BM_PER_Q		1

/*---------------------------*/
/*  Guest mode related functions  */
/*--------------------------*/

static int nmp_range_validate(int value, int min, int max)
{
	if (((value) > (max)) || ((value) < (min))) {
		pr_err("%s: value 0x%X (%d) is out of range [0x%X , 0x%X].\n",
			__func__, (value), (value), (min), (max));
		return -EINVAL;
	}
	return 0;
}

int nmp_read_cfg_file(char *cfg_file, struct nmp_params *params)
{
	u32				 i, j, k, rc;
	char				 file_name[SER_MAX_FILE_NAME];
	char				 buff[SER_MAX_FILE_SIZE];
	char				*sec = NULL;
	char				 tmp_buf[NMP_MAX_BUF_STR_LEN];
	char				 pp2_name[NMP_MAX_BUF_STR_LEN];
	struct nmp_lf_nicpf_params	*pf;
	u32				 num_lfs = 0;

	/* If cfg-file is provided, read the nmp-config from this location. Otherwise try to read either from
	 * local dir or from /var dir
	 */
	strcpy(file_name, cfg_file);
	rc = read_file_to_buf(file_name, buff, SER_MAX_FILE_SIZE);
	if (rc) {
		memset(file_name, 0, SER_MAX_FILE_NAME);
		snprintf(file_name, sizeof(file_name), "%s%s", NMP_CFG_FILE_LOCAL_DIR, NMP_CFG_FILE_NAME_PREFIX);
		rc = read_file_to_buf(file_name, buff, SER_MAX_FILE_SIZE);
		if (rc) {
			memset(file_name, 0, SER_MAX_FILE_NAME);
			snprintf(file_name, sizeof(file_name), "%s%s", NMP_CFG_FILE_VAR_DIR, NMP_CFG_FILE_NAME_PREFIX);
			rc = read_file_to_buf(file_name, buff, SER_MAX_FILE_SIZE);
			if (rc) {
				pr_info("nmp_config_file not found\n");
				return rc;
			}
		}
	}
	pr_info("nmp_cfg_location: %s\n", file_name);

	/* Check if there are nmp-params */
	sec = strstr(buff, "nmp_params");
	if (!sec) {
		pr_err("nmp_params section not found!\n");
		return -EINVAL;
	}

	/* Check if pp2 is enabled */
	json_buffer_to_input(sec, "pp2_en", params->pp2_en);
	if (nmp_range_validate(params->pp2_en, 0, 1) != 0) {
		pr_err("pp2_en not in tange!\n");
		return -EINVAL;
	}

	/* if pp2 enabled, set the pp2_params*/
	if (params->pp2_en) {
		sec = strstr(sec, "pp2_params");
		if (!sec) {
			pr_err("'pp2_params' not found\n");
			return -EINVAL;
		}

		json_buffer_to_input(sec, "bm_pool_reserved_map", params->pp2_params.bm_pool_reserved_map);
		if (nmp_range_validate(params->pp2_params.bm_pool_reserved_map, 0, PP2_BPOOL_NUM_POOLS)) {
			pr_info("bm_pool_reserved_map not in tange!\n");
			rc = -EINVAL;
			goto read_cfg_exit1;
		}
	}

	/* Read number of containers */
	json_buffer_to_input(sec, "num_containers", params->num_containers);
	if (nmp_range_validate(params->num_containers, 1, NMP_MAX_NUM_CONTAINERS)) {
		pr_info("num_containers not in tange!\n");
		rc = -EINVAL;
		goto read_cfg_exit1;
	}

	params->containers_params = kcalloc(1, sizeof(struct nmp_container_params) *
					    params->num_containers, GFP_KERNEL);
	if (params->containers_params == NULL) {
		rc = -ENOMEM;
		goto read_cfg_exit1;
	}

	for (i = 0; i < params->num_containers; i++) {
		memset(tmp_buf, 0, sizeof(tmp_buf));
		snprintf(tmp_buf, sizeof(tmp_buf), "containers_params-%d", i);
		sec = strstr(sec, tmp_buf);
		if (!sec) {
			pr_err("'containers_params' not found\n");
			rc = -EINVAL;
			goto read_cfg_exit1;
		}
		/* Read number of lfs */
		json_buffer_to_input(sec, "num_lfs", params->containers_params[i].num_lfs);
		if (nmp_range_validate(params->containers_params[i].num_lfs, 1, NMP_MAX_NUM_LFS) != 0) {
			rc = -EINVAL;
			goto read_cfg_exit1;
		}

		params->containers_params[i].lfs_params = kcalloc(1, sizeof(struct nmp_lf_params) *
								params->containers_params[i].num_lfs, GFP_KERNEL);
		if (params->containers_params[i].lfs_params == NULL) {
			rc = -ENOMEM;
			goto read_cfg_exit2;
		}
		num_lfs++;

		for (j = 0; j < params->containers_params[i].num_lfs; j++) {
			/* Read lf type*/
			json_buffer_to_input(sec, "lf_type", params->containers_params[i].lfs_params[j].type);
			if (nmp_range_validate(params->containers_params[i].lfs_params[j].type,
					       NMP_LF_T_NIC_NONE, NMP_LF_T_NIC_LAST - 1) != 0) {
				rc = -EINVAL;
				goto read_cfg_exit2;
			}

			if (params->containers_params[i].lfs_params[j].type == NMP_LF_T_NIC_PF) {
				/* Read nicpf*/
				pf = (struct nmp_lf_nicpf_params *)
				     &params->containers_params[i].lfs_params[j].u.nicpf;

				json_buffer_to_input(sec, "pci_en", pf->pci_en);
				if (nmp_range_validate(pf->pci_en, 0, 1) != 0) {
					rc = -EINVAL;
					goto read_cfg_exit2;
				}

				json_buffer_to_input(sec, "lcl_egress_qs_size", pf->lcl_egress_qs_size);
				if (!pf->lcl_egress_qs_size) {
					rc = -EINVAL;
					goto read_cfg_exit2;
				}

				json_buffer_to_input(sec, "lcl_ingress_qs_size", pf->lcl_ingress_qs_size);
				if (!pf->lcl_ingress_qs_size) {
					rc = -EINVAL;
					goto read_cfg_exit2;
				}

				json_buffer_to_input(sec, "dflt_pkt_offset", pf->dflt_pkt_offset);
				if (nmp_range_validate(pf->dflt_pkt_offset, 0, 1024) != 0) {
					rc = -EINVAL;
					goto read_cfg_exit2;
				}

				json_buffer_to_input(sec, "max_num_tcs", pf->max_num_tcs);
				if (nmp_range_validate(pf->max_num_tcs, 0, NMP_GIE_MAX_TCS) != 0) {
					rc = -EINVAL;
					goto read_cfg_exit2;
				}

				json_buffer_to_input(sec, "lcl_num_bpools", pf->lcl_num_bpools);
				if (nmp_range_validate(pf->lcl_num_bpools, 0, NMP_GIE_MAX_BPOOLS) != 0) {
					rc = -EINVAL;
					goto read_cfg_exit2;
				}

				for (k = 0; k < pf->lcl_num_bpools; k++) {
					memset(tmp_buf, 0, sizeof(tmp_buf));
					snprintf(tmp_buf, sizeof(tmp_buf), "lcl_bpools_params-%d", k);
					sec = strstr(sec, tmp_buf);
					if (!sec) {
						pr_err("'lcl_bpools_params' not found\n");
						rc = -EINVAL;
						goto read_cfg_exit2;
					}

					json_buffer_to_input(sec, "max_num_buffs",
							     pf->lcl_bpools_params[k].max_num_buffs);
					if (!pf->lcl_bpools_params[k].max_num_buffs) {
						rc = -EINVAL;
						goto read_cfg_exit2;
					}

					json_buffer_to_input(sec, "buff_size", pf->lcl_bpools_params[k].buff_size);
					if (!pf->lcl_bpools_params[k].buff_size) {
						rc = -EINVAL;
						goto read_cfg_exit2;
					}
				}

				json_buffer_to_input(sec, "nicpf_type", pf->type);
				if (nmp_range_validate(pf->type, NMP_LF_NICPF_T_NONE,
						       NMP_LF_NICPF_T_LAST - 1) != 0) {
					rc = -EINVAL;
					goto read_cfg_exit2;
				}

				if (pf->type != NMP_LF_NICPF_T_PP2_PORT) {
					rc = -EINVAL;
					goto read_cfg_exit2;
				}

				sec = strstr(sec, "port-params-pp2-port");
				if (!sec) {
					pr_err("'port-params-pp2-port' not found\n");
					return -EINVAL;
				}

				pf->port_params.pp2_port.match = pp2_name;
				json_buffer_to_input_str(sec, "match", pf->port_params.pp2_port.match);
				if (!pf->port_params.pp2_port.match) {
					pr_err("'pp2 match' not found\n");
					return -EINVAL;
				}

				json_buffer_to_input(sec, "lcl_num_bpools", pf->port_params.pp2_port.lcl_num_bpools);
				if (nmp_range_validate(pf->port_params.pp2_port.lcl_num_bpools,
						       1, NMP_LF_MAX_NUM_LCL_BPOOLS) != 0)
					return -EINVAL;

				for (k = 0; k < pf->port_params.pp2_port.lcl_num_bpools; k++) {
					struct nmp_lf_bpool_params *lcl_bpools_params =
						&pf->port_params.pp2_port.lcl_bpools_params[k];

					memset(tmp_buf, 0, sizeof(tmp_buf));
					snprintf(tmp_buf, sizeof(tmp_buf), "lcl_bpools_params-%d", k);
					sec = strstr(sec, tmp_buf);
					if (!sec) {
						pr_err("'lcl_bpools_params' not found\n");
						return -EINVAL;
					}

					json_buffer_to_input(sec, "max_num_buffs",
							lcl_bpools_params->max_num_buffs);
					if (nmp_range_validate(pf->port_params.pp2_port.lcl_num_bpools,
							       0, 4096) != 0) {
						return -EINVAL;
					}

					json_buffer_to_input(sec, "buff_size", lcl_bpools_params->buff_size);
					if (nmp_range_validate(lcl_bpools_params->buff_size, 0, 4096) != 0)
						return -EINVAL;
				}
			}
		}

		json_buffer_to_input(sec, "guest_id", params->containers_params[i].guest_id);
		if (nmp_range_validate(params->containers_params[i].guest_id, 0, 10) != 0) {
			rc = -EINVAL;
			goto read_cfg_exit2;
		}
	}

	return 0;
read_cfg_exit2:
	for (i = 0; i < num_lfs; i++)
		kfree(params->containers_params[i].lfs_params);
read_cfg_exit1:
	kfree(params->containers_params);
	return rc;
}

int guest_util_get_relations_info(char *buff, struct pp2_info *pp2_info)
{
	int	 i, j;
	char	*sec = NULL;
	int	 rc;
	char	*lbuff;
	char	 tmp_buf[MV_MAX_BUF_STR_LEN];

	lbuff = kcalloc(1, SER_MAX_FILE_SIZE, GFP_KERNEL);
	if (lbuff == NULL)
		return -ENOMEM;

	memcpy(lbuff, buff, SER_MAX_FILE_SIZE);

	sec = strstr(lbuff, "relations-info");
	if (!sec) {
		pr_err("'relations-info' not found\n");
		rc = -EINVAL;
		goto rel_info_exit1;
	}

	json_buffer_to_input(sec, "num_pp2_ports", pp2_info->num_ports);
	pr_debug("num_ports: %d\n", pp2_info->num_ports);

	pp2_info->port_info = kcalloc(1, sizeof(struct pp2_ppio_info) * pp2_info->num_ports, GFP_KERNEL);
	if (pp2_info->port_info == NULL) {
		rc = -ENOMEM;
		goto rel_info_exit1;
	}

	for (i = 0; i < pp2_info->num_ports; i++) {
		memset(tmp_buf, 0, sizeof(tmp_buf));
		snprintf(tmp_buf, sizeof(tmp_buf), "ppio-%d", i);
		json_buffer_to_input_str(sec, tmp_buf, pp2_info->port_info[i].ppio_name);
		pr_debug("port: %d, ppio_name %s\n", i, pp2_info->port_info[i].ppio_name);

		json_buffer_to_input(sec, "num_bpools", pp2_info->port_info[i].num_bpools);
		pr_debug("port: %d, num_pools %d\n", i, pp2_info->port_info[i].num_bpools);

		pp2_info->port_info[i].bpool_info = kcalloc(1, sizeof(struct pp2_ppio_bpool_info) *
							    pp2_info->port_info[i].num_bpools, GFP_KERNEL);
		if (pp2_info->port_info[i].bpool_info == NULL) {
			rc = -ENOMEM;
			goto rel_info_exit2;
		}
		for (j = 0; j < pp2_info->port_info[i].num_bpools; j++) {
			memset(tmp_buf, 0, sizeof(tmp_buf));
			snprintf(tmp_buf, sizeof(tmp_buf), "bpool-%d", j);
			json_buffer_to_input_str(sec, tmp_buf, pp2_info->port_info[i].bpool_info[j].bpool_name);
			pr_debug("port: %d, pool name %s\n", i, pp2_info->port_info[i].bpool_info[j].bpool_name);
		}
	}
	kfree(lbuff);
	return 0;

rel_info_exit2:
	kfree(pp2_info->port_info);
rel_info_exit1:
	kfree(lbuff);
	return rc;

}


int app_guest_utils_build_all_bpools(char *buff, struct pp2_info *pp2_info,
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

int app_nmp_guest_port_init(char *buff, struct pp2_info *pp2_info, struct port_desc *port)
{
	int				 err;
	struct pp2_ppio_capabilities	 capa;
	int				 i, j;

	pr_debug("pp2_info: num_ports %d\n", pp2_info->num_ports);
	for (i = 0; i < pp2_info->num_ports; i++) {
		pr_debug("	port: %d, name %s\n", i, pp2_info->port_info[i].ppio_name);

		err = pp2_ppio_probe(pp2_info->port_info[i].ppio_name, buff, &port[i].ppio);
		if (err) {
			pr_err("pp2_ppio_probe failed for %s\n", pp2_info->port_info[i].ppio_name);
			return err;
		}

		if (!port[i].ppio) {
			pr_err("PP-IO init failed!\n");
			return -EIO;
		}

		err = pp2_ppio_get_capabilities(port[i].ppio, &capa);
		if (err) {
			pr_err("pp2_ppio_get_capabilities failed for %s\n", pp2_info->port_info[i].ppio_name);
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

