/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#define log_fmt(fmt) "lf_mng " fmt

#include "std_internal.h"
#include "env/trace/trc_pf.h"
#include "mng/db.h"
#include "mng/dispatch.h"
#include "pf/pf.h"
#include "custom/custom.h"
#include "lf_mng.h"


#define SER_FILE_VAR_DIR	"/var/"
#define SER_FILE_NAME_PREFIX	"musdk-serial-cfg"
#define SER_MAX_FILE_NAME	64
#define SER_MAX_FILE_SIZE	(30 * 1024)


int lf_init(struct nmp *nmp)
{
	int ret;
	struct nmcstm_params params;

	nmp->nmnicpf.nmp = nmp;

	/* TODO: init nmp-nicpf only in case it is supported in the nmp_params! */
	ret = nmnicpf_init(&nmp->nmnicpf);
	if (ret)
		return ret;

	/* TODO: init nmp-customer only in case it is supported in the nmp_params! */
	params.id = nmp->guest_id;
	params.mqa = nmp->mqa;
	params.nmdisp = nmp->nmdisp;
	params.pf_id = nmp->nmnicpf.pf_id;
	ret = nmcstm_init(&params, &(nmp->nmcstm));
	if (ret) {
		pr_err("Custom init failed\n");
		return ret;
	}

	nmdisp_dispatch_dump(nmp->nmdisp);

	return 0;
}

int lf_deinit(struct nmp *nmp)
{
	int ret;

	ret = nmcstm_deinit(nmp->nmcstm);
	if (ret) {
		pr_err("Custom init failed\n");
		return ret;
	}

	ret = nmnicpf_deinit(&nmp->nmnicpf);
	if (ret)
		return ret;

	return 0;
}

void lf_init_done(struct nmp *nmp)
{
	struct mv_sys_dma_mem_info mem_info;
	char	 file_name[SER_MAX_FILE_NAME];
	char	 buff[SER_MAX_FILE_SIZE];
	u32	 size = SER_MAX_FILE_SIZE;
	char	 dev_name[100];
	int	 ret;
	size_t	 pos = 0;
	u8 guest_id = nmp->nmnicpf.guest_id;

	pr_debug("nmp_pf_init_done reached\n");

	/* TODO: go over all guests */
	snprintf(file_name, sizeof(file_name), "%s%s%d", SER_FILE_VAR_DIR, SER_FILE_NAME_PREFIX, guest_id);
	/* Remove the serialize files */
	remove(file_name);

	memset(buff, 0, size);

	json_print_to_buffer(buff, size, 0, "{\n");

	/* Serialize the DMA info */
	mem_info.name = dev_name;
	mv_sys_dma_mem_get_info(&mem_info);
	json_print_to_buffer(buff, size, 1, "\"dma-info\": {\n");
	json_print_to_buffer(buff, size, 2, "\"file_name\": \"%s\",\n", mem_info.name);
	json_print_to_buffer(buff, size, 2, "\"region_size\": %zu,\n", mem_info.size);
	json_print_to_buffer(buff, size, 2, "\"phys_addr\": 0x%lx\n", (u64)mem_info.paddr);
	json_print_to_buffer(buff, size, 1, "},\n");

	pr_debug("starting serialization of guest %d\n", guest_id);

	ret = nmnicpf_serialize(&nmp->nmnicpf, &buff[pos], size - pos);
	if (ret >= 0)
		pos += ret;
	if (pos != strlen(buff))
		pr_err("found mismatch between pos (%zu) and buff len (%zu)\n", pos, strlen(buff));

	if (nmp->nmcstm) {
		ret = nmcstm_serialize(nmp->nmcstm, &buff[pos], size - pos);
		if (ret >= 0)
			pos += ret;
		if (pos != strlen(buff))
			pr_err("found mismatch between pos (%zu) and buff len (%zu)\n", pos, strlen(buff));
	}

	json_print_to_buffer(buff, size, 0, "}\n");

	/* write buffer to file */
	ret = write_buf_to_file(file_name, buff, strlen(buff));
	if (ret)
		pr_err("Failed to write to guest %d file\n", guest_id);

	sync();
}
